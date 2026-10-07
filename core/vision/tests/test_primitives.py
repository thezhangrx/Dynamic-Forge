"""`vision.primitives` 里那些"要翻成 Verilog"的算子的直接测试。

这些函数是整个视觉模块的地基（比较器、累加器、行缓冲状态机），
所以单独测它们的**语义边界**，而不是只在标定流程里间接跑到。
"""

from __future__ import annotations

from vision.primitives import (
    Blob,
    drop_short,
    join_close,
    runs,
    scanline_blobs,
)


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def _canvas(w: int, h: int) -> bytearray:
    return bytearray(w * h)


def _fill(mask: bytearray, w: int, x0: int, y0: int, x1: int, y1: int) -> None:
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            mask[y * w + x] = 1


# --------------------------------------------------------------------------
# 游程
# --------------------------------------------------------------------------
def test_runs_are_closed_intervals():
    assert runs([0, 1, 1, 0, 1]) == [(1, 2), (4, 4)]
    assert runs([1, 1]) == [(0, 1)]
    assert runs([0, 0]) == []


def test_join_close_bridges_only_small_gaps():
    assert join_close([(0, 4), (6, 9)], 1) == [(0, 9)]     # 间隔 1 → 合并
    assert join_close([(0, 4), (8, 9)], 1) == [(0, 4), (8, 9)]


def test_drop_short_keeps_the_threshold_itself():
    assert drop_short([(0, 2), (5, 9)], 3) == [(0, 2), (5, 9)]
    assert drop_short([(0, 1)], 3) == []


def test_blob_derived_geometry():
    b = Blob(0, 0, 9, 3, 40, 0, 0)
    assert (b.w, b.h) == (10, 4)
    assert b.fill == 1.0
    assert b.aspect == 2.5
