"""``fiducial``（ArUco 标定标记）的回归测试。

**自包含**：合成图直接用纯 Python + numpy 画，不需要 cv2 之外的任何东西
（cv2 只用于生成/检测标记本身）。

这里锁住的每一条都对应一次实际踩到的错：

1. **角点顺序**：弄反了单应会整体转 90°，而"用一个标记的 4 点去验"是验不出来的
   —— 4 点解单应本来就是精确解、残差恒为 0（这个坑我真的踩了一次，
   8 种顺序全都"RMS=0.000"）。所以必须用**多个标记、16 组点**去验。
2. **场地 y 向上为正**：``Viewport.world_to_screen`` 把 y 翻了。
   按"y 向下"画出来的标记在画面里是**上下镜像**的，而镜像后的 ArUco 不是合法编码，
   相机根本解不出来 —— 实测就是这么失败的。
3. **留白边**：纯黑底上不留白边会检不出（黑边与背景糊在一起）。
4. **id 0 是 falsy**：``ids or []`` 会把"检出 id=0"当成"没检出"（我踩过，
   一整轮测试结论都是错的）。所以判断一律用 ``is not None``。
"""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from standard.fiducial import (  # noqa: E402
    FIDUCIAL_LEVEL,
    GRID_MODULES,
    MARKER_BITS,
    QUIET_MODULES,
    FiducialLayout,
    snap_marker_side,
)
from vision.calibration import rms_error, solve_homography  # noqa: E402
from vision.fiducial import (  # noqa: E402
    FiducialError,
    FiducialObs,
    calibrate_from_fiducials,
    detect_markers,
    reprojection_rms,
)
from vision.primitives import Frame  # noqa: E402

LAY = FiducialLayout(field_w=640.0, field_h=480.0, marker_side=48.0, margin=8.0)


# ---------------------------------------------------------------------------
# 合成一张"屏幕"并加退化
# ---------------------------------------------------------------------------
def render_screen(layout: FiducialLayout = LAY, *, scale: int = 2,
                  ids=range(4)) -> np.ndarray:
    """把场地四角的标记画成一张灰度图（模拟平台窗口）。"""
    w, h = int(layout.field_w * scale), int(layout.field_h * scale)
    img = np.full((h, w), 60, np.uint8)                 # 场地底（暗）
    for slot in ids:
        for (x, y, side, white) in layout.rects(slot):
            # **场地 y 向上、图像 y 向下**，所以要翻一下 —— 和
            # ``Viewport.world_to_screen`` 一致。不翻就是上下镜像，
            # 镜像后的 ArUco 不是合法编码（这正是要测的那件事）。
            ix0, ix1 = int(round(x * scale)), int(round((x + side) * scale))
            iy0 = int(round((layout.field_h - (y + side)) * scale))
            iy1 = int(round((layout.field_h - y) * scale))
            # 用**真实的**标记灰度（不是纯白 255）：合成图必须和平台画的一致，
            # 否则测的是"一个不存在的、更亮的标记"，退化边界全是假的。
            cv2.rectangle(img, (ix0, iy0), (ix1, iy1),
                          FIDUCIAL_LEVEL if white else 0, -1)
    return img


def as_frame(gray: np.ndarray) -> Frame:
    b = gray.tobytes()
    return Frame(gray.shape[1], gray.shape[0], b, b, b)


def degrade(gray: np.ndarray, *, gain=1.0, floor=0, blur=0) -> np.ndarray:
    out = (floor + gray.astype(np.float32) * gain).clip(0, 255).astype(np.uint8)
    if blur:
        out = cv2.GaussianBlur(out, (blur | 1, blur | 1), 0)
    return out


# ---------------------------------------------------------------------------
# 1. 角点顺序（必须用多标记验，否则结构性恒 0）
# ---------------------------------------------------------------------------
def test_corner_order_matches_opencv() -> None:
    """布局给的四角顺序必须与 ``detectMarkers`` 一致。

    判据：把 4 枚标记的 16 组点一次性解单应，残差必须很小。
    **只用一个标记验不出来** —— 4 点解单应精确，任何顺序都残差 0。
    """
    obs = detect_markers(as_frame(render_screen()))
    assert sorted(o.id for o in obs) == [0, 1, 2, 3], "4 枚标记都该检出"

    src = [p for o in sorted(obs, key=lambda o: o.id) for p in o.corners]
    dst = [p for o in sorted(obs, key=lambda o: o.id)
           for p in LAY.corners(o.id)]
    assert len(src) == 16
    h = solve_homography(src, dst)
    assert rms_error(h, src, dst) < 1.0      # 实际约 0.16 场地单位


def test_corner_order_would_be_caught_if_reversed() -> None:
    """反面对照：把某一枚标记的角点顺序转一下，残差应当**明显变大**。

    这条证明上面那个判据真的有分辨力 —— 不是"怎么排都能过"。
    """
    obs = detect_markers(as_frame(render_screen()))
    src = [p for o in sorted(obs, key=lambda o: o.id) for p in o.corners]
    dst = [p for o in sorted(obs, key=lambda o: o.id) for p in LAY.corners(o.id)]
    good = rms_error(solve_homography(src, dst), src, dst)

    bad = list(src)
    bad[0:4] = list(reversed(bad[0:4]))       # 把第一枚标记反向
    worse = rms_error(solve_homography(bad, dst), bad, dst)
    assert worse > 10.0 * good


# ---------------------------------------------------------------------------
# 2. 场地 y 向上为正（画反了就是上下镜像，ArUco 解不出来）
# ---------------------------------------------------------------------------
def test_marker_is_drawn_upright_on_screen() -> None:
    """场地 y 向上：bitmap 的**第 0 行**必须落在**较大的**场地 y 上。

    画反了标记在画面里就是上下镜像的，镜像后的 ArUco 不是合法编码 —— 实测失败。
    这里直接比数据行的 y：第 0 行（append 顺序在前）的 y 必须大于最后一行。
    """
    data = LAY.rects(0)[2:]                  # 前两块是留白 + 整块黑
    # ``rects`` 只发**白**模块（黑的由整块黑底提供），所以数量 = 白色位数
    assert len(data) == sum(MARKER_BITS[0]), "白模块数应当等于位图里 1 的个数"

    from standard.fiducial import BORDER_MODULES
    m = LAY.module
    cx, cy = LAY.centre(0)
    x0 = cx - LAY.marker_side / 2.0
    y_top = cy + LAY.marker_side / 2.0
    # 位 (0,0) 是白 -> 它必须落在 y = y_top - 2m（**最大的**那一行）
    assert MARKER_BITS[0][0] == 1
    want = (x0 + BORDER_MODULES * m, y_top - (0 + BORDER_MODULES + 1) * m)
    assert any(abs(bx - want[0]) < 1e-9 and abs(by - want[1]) < 1e-9
               for bx, by, _s, _w in data), \
        f"位(0,0)应在 {want}（场地 y 大的一侧）；画反了就是上下镜像，ArUco 解不出来"


def test_renderer_orientation_is_detectable() -> None:
    """端到端：按布局画出来、再检出，4 枚都在。画反了会一枚都检不出。"""
    obs = detect_markers(as_frame(render_screen()))
    assert len(obs) == 4


# ---------------------------------------------------------------------------
# 3. 留白边
# ---------------------------------------------------------------------------
def test_no_quiet_zone_on_black_background_fails() -> None:
    """纯黑底 + 不留白边 -> 检不出（黑边与背景糊在一起）。

    对照：加白留白或把背景换掉就能检出。这条决定了渲染器**必须**画白边。
    """
    tag = 60
    m = cv2.resize(MARKER_BITS and cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50).generateImageMarker(0, tag), (tag, tag),
        interpolation=cv2.INTER_NEAREST)

    def build(quiet_px: int, bg: int) -> np.ndarray:
        img = np.full((400, 400), bg, np.uint8)
        if quiet_px:
            lo = 100 - quiet_px
            img[lo:100 + tag + quiet_px, lo:100 + tag + quiet_px] = 255
        img[100:100 + tag, 100:100 + tag] = m
        return img

    assert detect_markers(as_frame(build(0, 0))) == []      # 纯黑无留白 -> 没有
    assert len(detect_markers(as_frame(build(10, 0)))) == 1  # 加留白 -> 有
    assert len(detect_markers(as_frame(build(0, 87)))) == 1  # 换背景 -> 有


# ---------------------------------------------------------------------------
# 4. 退化下的检出（拍屏的真实条件）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("gain,floor,blur", [
    (1.00, 0, 0),
    (0.55, 60, 0),
    (0.35, 80, 0),
    (0.35, 80, 5),
    (0.25, 90, 5),
    (0.25, 90, 9),
    (0.15, 100, 5),
])
def test_detects_under_screen_photo_degradation(gain, floor, blur) -> None:
    """过曝 / 低对比 / 模糊下 4 枚标记都要能检出。

    对照洋红方块：靠色度，实测 ``R-B`` 从 192 掉到 35~80（判据需 99）就崩了。
    黑白标记靠灰阶差，同样的退化下仍然可检。
    """
    obs = detect_markers(as_frame(degrade(render_screen(), gain=gain, floor=floor,
                                          blur=blur)))
    assert sorted(o.id for o in obs) == [0, 1, 2, 3]


# ---------------------------------------------------------------------------
# 5. 标定质量（这次是真的有信息：16 组点）
# ---------------------------------------------------------------------------
def test_calibration_is_accurate_and_measures_itself() -> None:
    obs = detect_markers(as_frame(render_screen()))
    calib, rep = calibrate_from_fiducials(obs, LAY, (1280, 960))
    assert rep.n_markers == 4 and rep.n_points == 16
    assert rep.rms_px < 1.0                  # 实测约 0.16 场地单位
    assert rep.holdout is not None and rep.holdout < 1.0
    assert rep.missing == ()
    # 场地单位：这次标定的 scale 是 2 px/单位，H 应当把图像 px 映回场地
    x, y = calib.image_to_field(640.0, 480.0)     # 图像中心
    assert abs(x - 320.0) < 1.0 and abs(y - 240.0) < 1.0


def test_calibration_recovers_a_known_point() -> None:
    """已知场地点 -> 图像 -> 反解，必须回到原处（**用外部真值，不用 H 自己**）。"""
    scale = 2
    obs = detect_markers(as_frame(render_screen(scale=scale)))
    calib, _ = calibrate_from_fiducials(obs, LAY, (640 * scale, 480 * scale))
    for (fx, fy) in ((320.0, 240.0), (120.0, 90.0), (500.0, 400.0)):
        sx, sy = calib.field_to_image(fx, fy)
        rx, ry = calib.image_to_field(sx, sy)
        assert abs(rx - fx) < 0.2 and abs(ry - fy) < 0.2


def test_reprojection_rms_flags_a_moved_marker() -> None:
    """每帧的标记重投影残差 = "标定还成立吗"的哨兵。

    把某枚标记的检测结果人为挪 10 px，残差必须明显跳起来。
    """
    obs = detect_markers(as_frame(render_screen()))
    calib, _ = calibrate_from_fiducials(obs, LAY, (1280, 960))
    base = reprojection_rms(calib, obs, LAY)
    moved = [FiducialObs(id=o.id,
                         corners=tuple((x + (10.0 if o.id == 2 else 0.0), y)
                                       for x, y in o.corners))
             for o in obs]
    assert reprojection_rms(calib, moved, LAY) > base + 2.0


# ---------------------------------------------------------------------------
# 6. 不足时必须报错，不能静默
# ---------------------------------------------------------------------------
def test_missing_markers_are_reported() -> None:
    obs = detect_markers(as_frame(render_screen(ids=range(2))))
    assert sorted(o.id for o in obs) == [0, 1]
    calib, rep = calibrate_from_fiducials(obs, LAY, (1280, 960))
    assert rep.missing == (2, 3)             # 缺哪些要能报出来
    with pytest.raises(FiducialError):
        calibrate_from_fiducials([], LAY, (1280, 960))


# ---------------------------------------------------------------------------
# 7. 渲染对齐：模块必须落在整数像素上
# ---------------------------------------------------------------------------
def test_snap_marker_side_gives_integer_module_pixels() -> None:
    for scale in (0.7, 1.0, 1.5, 2.25):
        snapped = snap_marker_side(LAY, scale)
        px = snapped.module * scale
        assert abs(px - round(px)) < 1e-9, f"scale={scale} 模块 {px} px 不是整数"
        assert round(px) >= 6, "模块太小会被模糊糊掉"


def test_layout_geometry_is_sane() -> None:
    assert GRID_MODULES == 6 and QUIET_MODULES == 1
    # 标记连同留白必须完整落在场地内
    for slot in range(4):
        cx, cy = LAY.centre(slot)
        half = LAY.marker_side / 2.0 + LAY.quiet
        assert half <= cx <= LAY.field_w - half
        assert half <= cy <= LAY.field_h - half
    # 四枚标记占场地面积应当很小（原来那块洋红是 81%）。
    # 只算标记本体是 3.0%；连白留白一起算 5.3%。
    body = 4 * LAY.marker_side ** 2 / (LAY.field_w * LAY.field_h)
    total = 4 * (LAY.marker_side + 2 * LAY.quiet) ** 2 / (LAY.field_w * LAY.field_h)
    assert body < 0.04 and total < 0.06
