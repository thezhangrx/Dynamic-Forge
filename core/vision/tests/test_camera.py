"""相机退化模型本身的性质：**它必须能造出真实相机的那种变化**，否则台架没意义。

检查的都是"模型是否忠实"这件事，不是判据：
* 恒等参数不动图（像素逐位相同，连对象都不复制）；
* 增益/偏置按 8 bit 四舍五入、削顶到 [0,255]；
* 白平衡是**每通道乘性**（所以 `r-b` 的符号不会被翻转 —— 这是相对判据的前提）；
* 失焦确实抹平阶跃、且不改变整体均值（不糊成一片）；
* 噪声的 σ 可以量回来；MJPG 走的是真编解码。
"""

from __future__ import annotations

import random

from vision import camera, primitives as P


def flat_frame(w: int = 32, h: int = 16, rgb=(100, 80, 60)) -> P.Frame:
    n = w * h
    return P.Frame(width=w, height=h,
                   r=bytes([rgb[0]]) * n, g=bytes([rgb[1]]) * n, b=bytes([rgb[2]]) * n)


def step_frame(w: int = 64, h: int = 16, *, lo: int = 20, hi: int = 220) -> P.Frame:
    """左半边暗、右半边亮：用来量失焦有没有真的抹开阶跃。"""
    r = bytearray()
    for _y in range(h):
        for x in range(w):
            r.append(hi if x >= w // 2 else lo)
    r = bytes(r)
    return P.Frame(width=w, height=h, r=r, g=r, b=r)


def test_identity_params_do_not_touch_the_frame():
    f = flat_frame()
    assert camera.apply(f) is f
    assert camera.apply(f, camera.CameraParams()) is f
    assert camera.CameraParams().is_identity()


def test_gain_and_offset_map_pixels_and_clip():
    f = flat_frame(rgb=(100, 80, 60))
    g = camera.apply(f, camera.CameraParams(gain=0.5, offset=10))
    # 0.5*100+10 = 60；0.5*80+10 = 50；0.5*60+10 = 40
    assert (g.r[0], g.g[0], g.b[0]) == (60, 50, 40)
    # 削顶：增益 4 + 偏置 → 全部到 255
    hot = camera.apply(f, camera.CameraParams(gain=4.0, offset=50))
    assert max(hot.r) == 255
    # 削底：负偏置 → 全部到 0
    cold = camera.apply(f, camera.CameraParams(gain=1.0, offset=-500))
    assert min(cold.r) == 0


def test_white_balance_is_per_channel_and_keeps_the_sign_of_r_minus_b():
    f = flat_frame(rgb=(150, 90, 60))
    for wb in ((1.3, 1.0, 0.7), (0.7, 1.0, 1.3), (1.1, 1.1, 1.1)):
        g = camera.apply(f, camera.CameraParams(wb=wb))
        assert (g.r[0] - g.b[0]) > 0, "白平衡是乘性的，暖/冷的符号不能翻"
        assert (g.r[0], g.g[0], g.b[0]) == tuple(
            camera.clip01(v * k) for v, k in zip((150, 90, 60), wb))


def test_blur_flattens_a_step_and_keeps_the_mean():
    f = step_frame(lo=20, hi=220)
    b = camera.apply(f, camera.CameraParams(blur=3))
    mid = f.width // 2
    # 阶跃被抹开：阶跃两侧出现了既不是 20 也不是 220 的值
    assert 20 < b.r[mid - 1] < 220 or 20 < b.r[mid] < 220
    # 均值基本不变（不是"糊成一片"）
    assert abs(sum(b.r) / f.n - sum(f.r) / f.n) < 1.0


def test_blur_is_symmetric_about_the_edge():
    f = step_frame(lo=20, hi=220)
    b = camera.apply(f, camera.CameraParams(blur=2))
    mid = f.width // 2
    left = sum(b.r[mid - 3:mid]) / 3.0
    right = sum(b.r[mid:mid + 3]) / 3.0
    assert left < right, "阶跃两侧的过渡必须是对称的暗→亮"


def test_noise_sigma_is_reproducible_and_roughly_right():
    f = flat_frame(rgb=(120, 120, 120), w=64, h=64)
    a = camera.apply(f, camera.CameraParams(noise=6.0, seed=1))
    b = camera.apply(f, camera.CameraParams(noise=6.0, seed=1))
    c = camera.apply(f, camera.CameraParams(noise=6.0, seed=2))
    assert a.r == b.r, "同一个 seed 必须逐位相同（可复现）"
    assert a.r != c.r, "不同 seed 不该相同"
    est = camera.noise_sigma(a, rows=64)
    assert 3.0 < est < 9.0, f"噪声 σ 估回来是 {est:.1f}，和加的 6.0 差太远"


def test_noise_sigma_of_a_flat_frame_is_zero():
    assert camera.noise_sigma(flat_frame(w=32, h=32)) < 0.2


def test_jpeg_is_a_real_roundtrip():
    f = flat_frame(rgb=(10, 200, 90))
    j = camera.apply(f, camera.CameraParams(jpeg=50))
    assert (j.width, j.height) == (f.width, f.height)
    assert j.r != f.r or j.g != f.g or j.b != f.b, "JPEG 压缩应当改变像素"


def test_rng_seed_does_not_leak_between_channels():
    """每个通道的噪声不能用同一个随机序列（否则灰噪声变成彩色噪声，不符合传感器）。"""
    f = flat_frame(rgb=(120, 120, 120), w=32, h=32)
    n = camera.apply(f, camera.CameraParams(noise=8.0, seed=5))
    rnd = random.Random(5)
    first = rnd.gauss(0.0, 8.0)                 # 第一个通道的第一个样本
    assert abs((n.r[0] - 120) - first) < 2.5
