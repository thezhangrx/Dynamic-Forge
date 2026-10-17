"""缺口检出（匹配滤波 + 滞回）的性质：**几何量不随相机参数变**。

这是一条可以反复跑的断言，不是"某张图上看着对"：

* 同一个合成缺口，压上 9 组相机退化（曝光 / 白平衡 / 失焦 / 噪声 / MJPG），
  `cx` 与 `width` 必须几乎不动；
* **只有单侧是墙**（另一侧是场地背景）不算缺口 —— `min(左肩, 右肩)` 就是为了这个；
* 缺口宽度的量法是"两段墙之间的背景连续段"（滞回），所以墙边被抹开也不影响；
* 最小缺口宽度是**几何先验**（默认 = 墙的厚度），不是标定出来的像素数。
"""

from __future__ import annotations

from vision import camera as C
from vision import primitives as P
from vision.relative import best_gap, best_wall_band

WARM = (240, 160, 48)
BG = (34, 40, 55)


def field(*, w: int = 320, h: int = 180, band=(30, 36), walls=((0, 139), (185, 319)),
          bg=BG, warm=WARM) -> P.Frame:
    """合成场地：若干段横墙（同一行带内），中间留缺口，其余是背景。"""
    y0, y1 = band
    rgb = bytearray()
    for y in range(h):
        for x in range(w):
            c = bg
            if y0 <= y <= y1:
                for a, b in walls:
                    if a <= x <= b:
                        c = warm
                        break
            rgb += bytes(c)
    n = w * h
    return P.Frame(width=w, height=h, r=rgb[0::3], g=rgb[1::3], b=rgb[2::3]) if n else None


def test_band_and_gap_are_located_within_a_pixel_or_two():
    f = field()
    band = best_wall_band(f)
    assert band["found"]
    assert abs(band["y0"] - 30) <= 1 and abs(band["y1"] - 36) <= 1
    g = best_gap(f, band["y0"], band["y1"])
    assert g["found"]
    assert abs(g["cx"] - 0.5 * (140 + 184)) <= 2.0
    assert abs(g["width"] - 45.0) <= 2.0


PRESETS = [
    C.CameraParams(gain=0.55, offset=-8),
    C.CameraParams(gain=1.6, offset=45),
    C.CameraParams(wb=(1.25, 0.95, 0.80)),
    C.CameraParams(wb=(0.80, 1.00, 1.30)),
    C.CameraParams(blur=1),
    C.CameraParams(blur=2),
    C.CameraParams(noise=5.0, seed=7),
    C.CameraParams(jpeg=50),
    C.CameraParams(wb=(1.2, 0.95, 0.85), gain=0.7, offset=10,
                   blur=1, noise=4.0, jpeg=60, seed=3),
]


def test_gap_is_invariant_across_camera_degradations():
    clean = field()
    band = best_wall_band(clean)
    base = best_gap(clean, band["y0"], band["y1"])
    assert base["found"]
    for p in PRESETS:
        f = C.apply(clean, p)
        b = best_wall_band(f)
        assert b["found"], f"退化 {p} 下墙带都找不到"
        g = best_gap(f, b["y0"], b["y1"])
        assert g["found"], f"退化 {p} 下缺口没找到"
        assert abs(g["cx"] - base["cx"]) <= 3.0, f"{p}: 中心漂了 {g['cx'] - base['cx']:+.1f} px"
        assert abs(g["width"] - base["width"]) <= 3.0, f"{p}: 宽度漂了 {g['width'] - base['width']:+.1f} px"


def test_a_wall_that_ends_in_the_middle_is_not_a_gap():
    """只有一侧是墙：`min(左肩, 右肩)` 会把这种"缺口"压到 0 分以下。"""
    f = field(walls=((0, 200),))
    band = best_wall_band(f)
    g = best_gap(f, band["y0"], band["y1"])
    assert not g["found"], f"把墙的尽头当成了缺口（{g}）"


def test_no_wall_means_no_gap():
    f = field(walls=())
    band = best_wall_band(f)
    assert not band["found"]
    g = best_gap(f, 30, 36)
    assert not g["found"]


def test_blurred_gap_edges_are_measured_at_the_half_height():
    """滞回：宽度量到"两段墙之间的背景连续段"，所以墙边被抹开也不缩水。"""
    clean = field()
    band = best_wall_band(clean)
    base = best_gap(clean, band["y0"], band["y1"])
    soft = C.apply(clean, C.CameraParams(blur=3))
    b = best_wall_band(soft)
    g = best_gap(soft, b["y0"], b["y1"])
    assert abs(g["width"] - base["width"]) <= 3.0
    assert abs(g["cx"] - base["cx"]) <= 2.0


def test_min_gap_width_is_a_geometric_prior_not_a_tuned_number():
    """缺口不能比墙还窄：默认 `min_gap_px` = 墙厚（7 行），所以 4 px 的缝不算门。"""
    f = field(walls=((0, 158), (163, 319)))          # 缝只有 4 px
    band = best_wall_band(f)
    assert not best_gap(f, band["y0"], band["y1"])["found"]
    # 显式允许更窄时能找到 —— 说明"没找到"确实来自这个先验，而不是别的东西
    g = best_gap(f, band["y0"], band["y1"], min_gap_px=3)
    assert g["found"] and abs(g["width"] - 4.0) <= 2.0


def test_gap_position_does_not_depend_on_gain_at_all():
    """乘法增益下**逐位相同**：判据里没有任何绝对亮度/色度阈值。"""
    clean = field()
    band = best_wall_band(clean)
    base = best_gap(clean, band["y0"], band["y1"])
    dark = C.apply(clean, C.CameraParams(gain=0.35))
    g = best_gap(dark, band["y0"], band["y1"])
    assert g["x0"] == base["x0"] and g["x1"] == base["x1"]


def test_snr_is_reported_and_drops_with_noise_but_stays_above_the_gate():
    clean = field()
    band = best_wall_band(clean)
    quiet = best_gap(clean, band["y0"], band["y1"])
    noisy = best_gap(C.apply(clean, C.CameraParams(noise=6.0, seed=2)),
                     band["y0"], band["y1"])
    assert noisy["found"]
    assert noisy["snr"] < quiet["snr"]
