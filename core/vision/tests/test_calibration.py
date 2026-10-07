"""任务一回归测试：参考物 → 坐标系（单应 + 尺度）。

不需要摄像头：全部用**合成对应点**验证。数学对不对可以离线判定，
这也是"相机不可用也能把代码写完"的前提。
"""

from __future__ import annotations

import math

import pytest

from vision.calibration import (
    FieldCalibration,
    Marker,
    apply_homography,
    calibrate_from_marker,
    detect_marker_quad,
    invert_homography,
    rms_error,
    solve_homography,
)

#: 造一个"斜视相机"：世界(米) → 图像(px)。有透视、有旋转。
H_TRUE = (620.0, 40.0, 320.0,
          -12.0, 520.0, 300.0,
          0.0012, 0.0007, 1.0)


def to_px(X: float, Y: float) -> tuple[float, float]:
    return apply_homography(H_TRUE, X, Y)


MARKER = Marker("test-marker", 0.30, 0.20)


# --------------------------------------------------------------------------
# 单应本身
# --------------------------------------------------------------------------
def test_four_points_recover_the_homography_exactly():
    src = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    dst = list(MARKER.world_corners())
    H = solve_homography(src, dst)
    assert rms_error(H, src, dst) < 1e-9
    # 任意点往返
    for X, Y in ((0.15, 0.10), (0.07, 0.03), (0.25, 0.18), (0.0, 0.0)):
        u, v = apply_homography(H_TRUE, X, Y)
        X2, Y2 = apply_homography(H, u, v)
        assert math.hypot(X2 - X, Y2 - Y) < 1e-9


def test_more_points_with_noise_gives_sub_millimetre_rms():
    """多余点做最小二乘：±0.5 px 的角点噪声应压到毫米以下。"""
    import random
    random.seed(7)
    world = [(0, 0), (0.3, 0), (0.3, 0.2), (0, 0.2),
             (0.15, 0), (0.3, 0.1), (0.15, 0.2), (0, 0.1)]
    src, dst = [], []
    for X, Y in world:
        u, v = to_px(X, Y)
        src.append((u + random.uniform(-0.5, 0.5), v + random.uniform(-0.5, 0.5)))
        dst.append((X, Y))
    H = solve_homography(src, dst)
    assert rms_error(H, src, dst) < 0.002          # < 2 mm


def test_fewer_than_four_points_is_rejected():
    with pytest.raises(ValueError, match="至少需要 4 组"):
        solve_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)])


def test_collinear_points_are_rejected():
    src = [(0, 0), (1, 1), (2, 2), (3, 3)]
    dst = [(0, 0), (1, 0), (2, 0), (3, 0)]
    with pytest.raises(ValueError, match="奇异|退化"):
        solve_homography(src, dst)


def test_inverse_round_trips():
    H = solve_homography([to_px(X, Y) for X, Y in MARKER.world_corners()],
                         list(MARKER.world_corners()))
    Hi = invert_homography(H)
    for X, Y in ((0.1, 0.05), (0.28, 0.19)):
        u, v = apply_homography(H, X, Y)
        X2, Y2 = apply_homography(Hi, u, v)
        assert math.hypot(X2 - X, Y2 - Y) < 1e-9


# --------------------------------------------------------------------------
# 标定记录
# --------------------------------------------------------------------------
def test_calibrate_from_marker_gives_metric_scale():
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER, image_size=(640, 480))
    cal.validate()
    assert cal.rms_px < 1e-9
    # 度量衡：用图像点量参考物自己的宽高，必须等于真实尺寸
    assert cal.distance_metres(quad[0], quad[1]) == pytest.approx(0.30, abs=1e-6)
    assert cal.distance_metres(quad[0], quad[3]) == pytest.approx(0.20, abs=1e-6)
    # 世界单位就是米
    assert cal.metres_per_unit == 1.0
    assert cal.unit_to_metres(2.5) == 2.5


def test_calibration_json_round_trip(tmp_path):
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER, image_size=(640, 480))
    path = cal.save(tmp_path / "cal.json")
    back = FieldCalibration.load(path)
    assert back.H == pytest.approx(cal.H)
    assert back.metres_per_unit == cal.metres_per_unit
    assert back.rms_px == cal.rms_px
    assert back.image_to_field(*quad[0]) == pytest.approx(cal.image_to_field(*quad[0]))


def test_validate_rejects_bad_scale():
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    cal = calibrate_from_marker(quad, MARKER)
    cal.metres_per_unit = 0.0
    with pytest.raises(ValueError, match="metres_per_unit"):
        cal.validate()


# --------------------------------------------------------------------------
# 参考物检测（合成图，不用摄像头）
# --------------------------------------------------------------------------
def _synthetic_marker_frame(size=(480, 360)):
    """深色背景 + 一块亮矩形（参考物）。"""
    w, h = size
    buf = bytearray(bytes((30, 34, 44)) * (w * h))
    left, top, right, bottom = 60, 40, 240, 160
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            i = (y * w + x) * 3
            buf[i:i + 3] = bytes((240, 240, 240))
    return _frame(w, h, buf)


def _frame(w, h, rgb):
    from vision.primitives import Frame
    return Frame.from_rgb(w, h, rgb)


def test_detect_marker_quad_finds_a_bright_rectangle():
    frame = _synthetic_marker_frame()
    quad = detect_marker_quad(frame, min_area_frac=0.01)
    assert quad is not None and len(quad) == 4
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    assert min(xs) == pytest.approx(60, abs=3) and max(xs) == pytest.approx(240, abs=3)
    assert min(ys) == pytest.approx(40, abs=3) and max(ys) == pytest.approx(160, abs=3)
    # 四角必须互不相同（对角极值取角的正确性）
    assert len(set(quad)) == 4


def test_detect_marker_quad_returns_none_on_empty_frame():
    frame = _frame(64, 48, bytes((20, 20, 20)) * (64 * 48))
    assert detect_marker_quad(frame, min_area_frac=0.01) is None
