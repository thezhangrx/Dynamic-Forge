"""采集守卫的测试。

`capture_one_frame.frame_is_usable` 挡的是**整帧废掉**的两种真实故障 ——
不是内容判断，那种属于具体算法。

设备相关的部分（真开摄像头）不在这里测：CI 上没有相机，而且相机不可用时
这些算子依然要能被单独判定，这正是把它们写成纯函数的原因。
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("cv2")          # capture_one_frame 顶层 import cv2

from vision.capture_one_frame import frame_is_usable  # noqa: E402


def _frame(bgr_value, size=(48, 32)):
    h, w = size
    return np.full((h, w, 3), bgr_value, dtype=np.uint8)


def test_frame_is_usable_accepts_a_normal_scene():
    a = _frame(20)
    a[8:24, 10:30] = 200                 # 一块亮区，均值远没到饱和
    assert frame_is_usable(a)


def test_frame_is_usable_rejects_a_blown_out_frame():
    """整帧纯白 —— 实测出现过：自动曝光被拉满后连续 12 帧逐字节相同、均值 254。"""
    assert not frame_is_usable(_frame(255))
    assert not frame_is_usable(_frame(254))


def test_frame_is_usable_rejects_a_black_frame():
    """整帧全黑 —— 笔记本内置摄像头的隐私快门关闭时就是这样（实测 privacy=1）。"""
    assert not frame_is_usable(_frame(0))
    assert not frame_is_usable(_frame(3))


def test_frame_is_usable_rejects_an_empty_array():
    assert not frame_is_usable(np.zeros((0, 0, 3), dtype=np.uint8))
