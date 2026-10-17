"""相机退化模型 —— 用**物理上会发生的那几种失真**去压判据，而不是调阈值。

为什么要有这个东西
------------------
真板子上的那次教训很硬：同一个缺口，四张不同曝光的图（相机亮度 40/70/100/130）
被逐列阈值切出 110 / 16 / 6 / 46 px。要判"判据稳不稳"，必须能**独立地**
制造曝光、白平衡、失焦、噪声、压缩这几种变化，然后看同一个几何量输出多少。
只在一台相机、一个亮度下"看起来对"，是没有说服力的。

模型（尽量少，但每一项都对应相机里真实的一环）
----------------------------------------------
```text
out_c = clip( gain * wb_c * in_c + offset , 0, 255 )
```

* `gain` / `offset` —— **自动曝光**：乘法增益 + 黑电平偏置。实测自动曝光会把
  背景均值抬到 ~180，屏幕上的橙色 `r-b` 从 192 掉到 20~57，就是这个模型。
* `wb_c` —— **自动白平衡**：每通道一个乘法增益（乘性，所以 `r-b` 的**符号**
  不变 —— 这正是相对判据敢用 `d>0` 的理由）。
* `blur` —— **失焦**：方框核（可分离，前缀和实现）。真实镜头是高斯，但在
  "阶跃被抹开"这件事上方框核已经足够，而且没有可调的标准差。
* `noise` —— **传感器噪声**：高斯，单位是 8 bit 计数。
* `jpeg` —— **UVC 输出的是 MJPG**：块效应会制造假边缘，这一项不能省。

用法
----
```python
from vision import camera
weak = camera.apply(frame, camera.CameraParams(gain=0.6, offset=-20))
```
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import primitives as P

__all__ = ["CameraParams", "apply", "clip01"]


@dataclass(frozen=True)
class CameraParams:
    """一组退化参数。默认值是**恒等变换**（什么都不做）。"""

    wb: tuple = (1.0, 1.0, 1.0)   # 白平衡：每通道乘性增益
    gain: float = 1.0             # 曝光增益
    offset: float = 0.0           # 黑电平偏置（8 bit 计数）
    blur: int = 0                 # 方框核半径（px），0 = 不做
    noise: float = 0.0            # 高斯噪声 σ（8 bit 计数）
    jpeg: int | None = None       # JPEG 质量（UVC 是 MJPG），None = 不做
    seed: int = 0

    def is_identity(self) -> bool:
        return (self.wb == (1.0, 1.0, 1.0) and self.gain == 1.0
                and self.offset == 0.0 and self.blur == 0
                and self.noise == 0.0 and self.jpeg is None)


def clip01(x: float) -> int:
    """浮点 → 8 bit，四舍五入后夹到 [0, 255]。"""
    v = int(x + 0.5)
    return 0 if v < 0 else (255 if v > 255 else v)


def _box_blur(channel: bytes, w: int, h: int, r: int) -> bytes:
    """可分离方框核（半径 r）。前缀和实现：**每行/每列只走两遍**。"""
    if r <= 0 or w <= 0 or h <= 0:
        return bytes(channel)
    n = w * h
    tmp = [0] * n
    # 水平方向
    for y in range(h):
        base = y * w
        run = 0
        for x in range(min(r + 1, w)):
            run += channel[base + x]
        cnt = min(r + 1, w)
        for x in range(w):
            tmp[base + x] = run // cnt
            out_x = x - r
            in_x = x + r + 1
            if out_x >= 0:
                run -= channel[base + out_x]
                cnt -= 1
            if in_x < w:
                run += channel[base + in_x]
                cnt += 1
    # 垂直方向
    out = bytearray(n)
    for x in range(w):
        run = 0
        cnt = 0
        for y in range(min(r + 1, h)):
            run += tmp[y * w + x]
            cnt += 1
        for y in range(h):
            out[y * w + x] = run // cnt
            out_y = y - r
            in_y = y + r + 1
            if out_y >= 0:
                run -= tmp[out_y * w + x]
                cnt -= 1
            if in_y < h:
                run += tmp[in_y * w + x]
                cnt += 1
    return bytes(out)


def _jpeg_roundtrip(frame: "P.Frame", quality: int) -> "P.Frame":
    """走一遍真正的 JPEG 编解码（UVC 送出来的就是 MJPG，块效应是真的）。"""
    try:
        from io import BytesIO

        from PIL import Image
    except Exception:                      # pragma: no cover - 没装 PIL 就跳过这一环
        return frame
    img = Image.frombytes("RGB", (frame.width, frame.height), bytes(frame.to_rgb()))
    buf = BytesIO()
    img.save(buf, format="JPEG", quality=int(quality))
    buf.seek(0)
    back = Image.open(buf).convert("RGB")
    data = back.tobytes()
    return P.Frame(width=frame.width, height=frame.height,
                   r=data[0::3], g=data[1::3], b=data[2::3])


def apply(frame: "P.Frame", p: CameraParams | None = None) -> "P.Frame":
    """把 `p` 这组退化作用到 `frame` 上，返回新帧（不改输入）。"""
    p = p or CameraParams()
    if p.is_identity():
        return frame
    w, h = frame.width, frame.height
    out = frame
    if p.blur:
        out = P.Frame(width=w, height=h,
                      r=_box_blur(out.r, w, h, int(p.blur)),
                      g=_box_blur(out.g, w, h, int(p.blur)),
                      b=_box_blur(out.b, w, h, int(p.blur)))
    if (p.gain != 1.0 or p.offset != 0.0 or p.wb != (1.0, 1.0, 1.0)
            or p.noise > 0.0):
        rng = random.Random(p.seed)
        chans = []
        for c, src in enumerate((out.r, out.g, out.b)):
            k = p.gain * p.wb[c]
            lut = [clip01(k * v + p.offset) for v in range(256)]
            if p.noise > 0.0:
                chans.append(bytes(clip01(lut[v] + rng.gauss(0.0, p.noise)) for v in src))
            else:
                chans.append(bytes(lut[v] for v in src))
        out = P.Frame(width=w, height=h, r=chans[0], g=chans[1], b=chans[2])
    if p.jpeg is not None:
        out = _jpeg_roundtrip(out, int(p.jpeg))
    return out


def noise_sigma(frame: "P.Frame", *, rows: int = 8) -> float:
    """估一张图的传感器噪声（相邻像素差分的稳健标准差）。

    用途：给仿真加"和某个真实素材一样脏"的噪声，而不是拍一个数。
    """
    w, h = frame.width, frame.height
    if w < 2 or h < 2:
        return 0.0
    diffs = []
    step = max(1, h // max(1, rows))
    for y in range(0, h, step):
        base = y * w
        for x in range(w - 1):
            diffs.append(frame.r[base + x + 1] - frame.r[base + x])
    if not diffs:
        return 0.0
    diffs.sort()
    med = diffs[len(diffs) // 2]
    dev = sorted(abs(d - med) for d in diffs)
    mad = dev[len(dev) // 2]
    return 1.4826 * mad / math.sqrt(2.0)
