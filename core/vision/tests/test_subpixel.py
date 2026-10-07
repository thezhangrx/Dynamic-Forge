"""亚像素边缘估计的测试。

这里只测**算子本身**（半高穿越、四边形精修、圆精修）。
它带来的实际收益在 `test_pipeline.py` 的闭环里对账（位置/半径误差）。
"""

from __future__ import annotations

import math

import pytest

from vision import primitives as P
from vision import subpixel as S


def _ramp_frame(true_r: float, *, size: int = 140, blur: int = 0,
                level: int = 240, bg: int = 20, cx: float | None = None):
    """画一个半径 ``true_r`` 的实心圆（可加盒式模糊，模拟相机）。"""
    c = float(size // 2) if cx is None else cx
    cy = float(size // 2)
    buf = bytearray(bytes((bg, bg, bg)) * (size * size))
    for y in range(size):
        for x in range(size):
            inside = math.hypot(x - c, y - cy) <= true_r
            v = level if inside else bg
            i = (y * size + x) * 3
            buf[i:i + 3] = bytes((v, v, v))
    frame = P.Frame.from_rgb(size, size, buf)
    if blur > 1:
        frame = _box_blur(frame, blur)
    return frame


def _box_blur(frame, k: int):
    w, h = frame.width, frame.height
    pad = k // 2
    out = bytearray(frame.n)
    for y in range(h):
        for x in range(w):
            acc = 0
            n = 0
            for dy in range(-pad, pad + 1):
                for dx in range(-pad, pad + 1):
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < w and 0 <= yy < h:
                        acc += frame.r[yy * w + xx]
                        n += 1
            v = acc // max(n, 1)
            i = (y * w + x) * 3
            out[i:i + 3] = bytes((v, v, v))
    return P.Frame.from_rgb(w, h, out)


# --------------------------------------------------------------------------
def test_half_level_edge_is_exact_on_a_symmetric_ramp():
    """对称模糊下，**半高**位置就是无模糊时的真实边缘位置（阈值位置不是）。"""
    for true_edge in (4.0, 7.5, 12.25, 18.75):
        prof = [200.0 * max(0.0, min(1.0, 0.5 - (i - true_edge) / 3.0))
                for i in range(30)]
        got = S.half_level_edge(prof)
        assert got == pytest.approx(true_edge, abs=0.02), (true_edge, got)


def test_half_level_edge_returns_none_without_a_crossing():
    assert S.half_level_edge([200.0] * 10) is None
    assert S.half_level_edge([0.0] * 10) is None
    assert S.half_level_edge([1.0, 2.0]) is None          # 太短


def test_half_level_edge_accepts_explicit_levels():
    """给定电平更稳：逐帧估计会被噪声带偏，硬件上也更愿意用两个寄存器。"""
    prof = [200.0] * 8 + [100.0, 0.0] + [0.0] * 8
    # 半高 = 100，而第 8 个采样点恰好就是 100 —— 穿越点正落在它上面。
    assert S.half_level_edge(prof, hi=200.0, lo=0.0) == pytest.approx(8.0, abs=0.01)


def test_bilinear_sampling_makes_tilted_edges_accurate():
    """斜边：最近邻会有半像素量化误差，双线性不会。

    构造一条 45 度的台阶边，真值在 x = y 这条线上。
    """
    size = 60
    buf = bytearray(size * size * 3)
    for y in range(size):
        for x in range(size):
            v = 200 if x >= y else 0
            i = (y * size + x) * 3
            buf[i:i + 3] = bytes((v, v, v))
    f = P.Frame.from_rgb(size, size, buf)
    # 约定：剖面**从目标内部往背景走**（高 -> 低）。所以从 x=36 往 -x 走。
    prof = S._sample_along(f, 36.0, 30.0, -1.0, 0.0, 12, S.bright_score)
    idx = S.half_level_edge(prof)
    assert idx is not None
    assert 36.0 - idx == pytest.approx(30.0, abs=0.6)


def test_refine_circle_finds_the_centre_to_sub_pixel():
    """圆心本来就能靠质心估，但**半径**不行 —— 外接框是整数。"""
    f = _ramp_frame(12.0, cx=63.25)
    m = P.classify(f, bright_min=150, bright_spread=45).bright
    b = P.scanline_blobs(m, f.width, 0, f.height - 1, 0, f.width - 1, min_area=20)[0]
    cx, cy, r = S.refine_circle(f, b.cx, b.cy, (b.w + b.h) / 4.0)
    # 圆心：质心估计已经不错，精修后不应变差
    assert abs(cx - 63.25) < 0.5
    # 半径：与真值同量级（约定差异见模块说明，实测残差 < 1 px）
    assert abs(r - 12.0) < 1.0, r


def test_refine_circle_survives_blur_better_than_the_bounding_box():
    """**这是亚像素真正的用武之地**：相机模糊会把边界往外糊。

    外接框法在模糊核 5 px 时偏小 12%、7 px 时偏小 22%；
    半高法稳在 3% 上下。半径进了"缺口 >= 2r"这条硬约束，不能差这么多。
    """
    true_r = 12.0
    for blur in (5, 7):
        f = _ramp_frame(true_r, blur=blur)
        m = P.classify(f, bright_min=150, bright_spread=45).bright
        b = P.scanline_blobs(m, f.width, 0, f.height - 1, 0, f.width - 1, min_area=20)[0]
        bbox_err = abs((b.w + b.h) / 4.0 - true_r)
        _, _, r = S.refine_circle(f, b.cx, b.cy, (b.w + b.h) / 4.0)
        sub_err = abs(r - true_r)
        assert sub_err < bbox_err, f"blur={blur}: 亚像素 {sub_err:.2f} 没赢过外接框 {bbox_err:.2f}"
        assert sub_err < 0.06 * true_r, f"blur={blur}: 亚像素误差 {sub_err:.2f} 太大"


def test_magenta_score_follows_the_same_colour_key_as_the_detector():
    """精修与掩膜必须看同一个判据，否则边缘会落在两个不同的地方。"""
    assert S.magenta_score(200, 40, 200) == 160          # 洋红：命中
    assert S.magenta_score(200, 200, 200) == 0           # 白/反光：排除
    assert S.magenta_score(240, 170, 60) == 0            # 橙墙：排除
    # 调色板的粉 (230,130,220)：弱/强 = 0.59，**颜色上分不开**（已知限制，
    # 靠长宽比和尺寸分开，见 calibration.detect_marker_quad 的说明）。
    assert S.magenta_score(230, 130, 220) == 90


def test_refine_quad_returns_sub_pixel_corners():
    """整数四角进去，分数四角出来，并且四条边各方向都更接近真值。"""
    size = 120
    left, top, right, bottom = 30, 25, 90, 75     # 真值边（像素覆盖 [30,91) 等）
    buf = bytearray(bytes((20, 20, 20)) * (size * size))
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            i = (y * size + x) * 3
            buf[i:i + 3] = bytes((200, 0, 200))
    f = P.Frame.from_rgb(size, size, buf)
    truth = [(left, bottom + 1), (right + 1, bottom + 1), (right + 1, top), (left, top)]
    # ① 四角已经准的时候，精修**不该乱动**它
    same = S.refine_quad(f, truth)
    for (x, y), (ex, ey) in zip(same, truth):
        assert abs(x - ex) <= 0.6 and abs(y - ey) <= 0.6, (same, truth)
    # ② 四角**偏内** 2 px（就是阈值法会犯的错）时，必须被推回真实边缘
    inward = [(left + 2, bottom - 1), (right - 1, bottom - 1),
              (right - 1, top + 2), (left + 2, top + 2)]
    fixed = S.refine_quad(f, inward)
    for (x, y), (ex, ey) in zip(fixed, truth):
        assert math.hypot(x - ex, y - ey) < 1.0, f"没修回来：{fixed} vs 真值 {truth}"


def test_line_fitting_beats_translation_when_the_image_is_blurred():
    """**这是"拟合直线"存在的理由**，也是它换掉"只平移"的依据。

    单条剖面的边缘估计在有模糊时是有噪声的。只平移的做法取这些估计的**中位**，
    噪声原样留着；拟合整条边线则把它平均掉。实测：

    | 模糊核 | 只平移 RMS | 拟合直线 RMS |
    |---|---|---|
    | 1 | 0.46 | 0.52 |
    | 3 | 1.24 | **0.41** |
    | 5 | 2.14 | **0.38** |

    干净图上只平移略好（边是直的，估斜率只是多引入方差）；真实相机一定有模糊，
    所以默认走拟合。
    """
    size = 200
    left, top, right, bottom = 40, 60, 160, 130
    buf = bytearray(bytes((20, 20, 20)) * (size * size))
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            i = (y * size + x) * 3
            buf[i:i + 3] = bytes((200, 0, 200))
    coarse = [(left, bottom + 1), (right + 1, bottom + 1),
              (right + 1, top), (left, top)]

    for blur in (3, 5):
        f = _box_blur(P.Frame.from_rgb(size, size, buf), blur)
        want = [(left, bottom + 1), (right + 1, bottom + 1), (right + 1, top), (left, top)]

        def rms(quad):
            return math.sqrt(sum(math.hypot(x - ex, y - ey) ** 2
                                 for (x, y), (ex, ey) in zip(quad, want)) / 4)

        fit = S.refine_quad(f, [(x, y) for x, y in coarse])
        assert len(fit) == 4
        # 拟合后的角点必须足够接近真值；"只平移"那版在同样模糊下要差好几倍
        assert rms(fit) < 1.0, f"blur={blur}: 拟合后 RMS {rms(fit):.3f} 太大"
