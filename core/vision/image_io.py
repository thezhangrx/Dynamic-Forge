"""图片读写 —— **整个 vision 模块里唯一依赖第三方库的地方**。

为什么要单独一个文件
--------------------
算法（``primitives.py`` / ``wall_with_gap.py``）要往 Verilog 翻译，那里不能出现
numpy / OpenCV 的任何东西。但"把 JPEG 解码成像素"和"把像素存成 PNG"与算法无关，
纯粹是文件格式问题，必须借助库。所以把它们全部收在这一个文件里，边界清清楚楚：

    JPEG/PNG  ──load()──▶  Frame(三个通道平面)  ──▶  算法（纯 Python）
    JPEG/PNG  ◀──save()──  Frame / RGB 缓冲      ◀──  标注（纯 Python）

上板之后这一层会被替换掉：像素不再来自文件，而是来自摄像头的像素流
（DVP/MIPI → 行有效 / 帧有效），``Frame`` 的三个平面就是三条并行数据线。

依赖：``numpy`` + ``opencv-python``（仅此一处）。
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from vision.primitives import Frame


def load(path: str | Path) -> Frame:
    """读一张图 → :class:`Frame`（BGR 解码后拆成 R/G/B 平面）。"""
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"读不到图片: {path}")
    h, w = bgr.shape[:2]
    # cv2 给的是 BGR（numpy 数组），这里只做一次拷贝到 bytes 平面
    return Frame(width=w, height=h,
                 r=bgr[:, :, 2].tobytes(),
                 g=bgr[:, :, 1].tobytes(),
                 b=bgr[:, :, 0].tobytes())


def save(path: str | Path, frame: Frame | bytearray | bytes,
         width: int | None = None, height: int | None = None) -> None:
    """把 :class:`Frame` 或交错 RGB 缓冲写成图片（后缀决定格式）。"""
    if isinstance(frame, Frame):
        w, h = frame.width, frame.height
        rgb = np.frombuffer(bytes(frame.to_rgb()), dtype=np.uint8).reshape(h, w, 3)
    else:
        if width is None or height is None:
            raise ValueError("传交错 RGB 缓冲时必须给 width/height")
        w, h = int(width), int(height)
        rgb = np.frombuffer(bytes(frame), dtype=np.uint8).reshape(h, w, 3)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise OSError(f"写图片失败: {path}")


__all__ = ["load", "save"]
