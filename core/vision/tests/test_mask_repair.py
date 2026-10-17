"""掩膜修复 + 墙段合并：只做该做的，绝不把真缺口封死。

这两条规则的价值全在"保守"上：修掉小洞、接上碎段，但**真实通道必须原样保留** ——
对避障来说"把缺口封死"比"墙断裂"危险得多。所以这里的反面用例和正面用例一样重要。
"""

from __future__ import annotations

from vision import primitives as P
from vision.mask_repair import merge_segments, repair_mask, repair_stats
from vision.wall_with_gap import WallSegment


def _mask(rows: list[str]) -> tuple[bytearray, int, int]:
    h = len(rows)
    w = len(rows[0])
    m = bytearray(w * h)
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            m[y * w + x] = 1 if ch == "#" else 0
    return m, w, h


def _rows(mask: bytearray, w: int, h: int) -> list[str]:
    return ["".join("#" if mask[y * w + x] else "." for x in range(w)) for y in range(h)]


# --------------------------------------------------------------------------
# 正面：小洞接上、孤立点丢掉
# --------------------------------------------------------------------------
def test_gap_within_radius_is_closed():
    m, w, h = _mask([
        "####.####",
    ])
    out = repair_mask(m, w, h, gap_x=1, short_x=1)
    assert _rows(out, w, h) == ["#########"]


def test_gap_larger_than_radius_is_kept():
    """这是最重要的一条：**不许把真缺口封死**。"""
    m, w, h = _mask([
        "###....###",
    ])
    out = repair_mask(m, w, h, gap_x=2, short_x=1)
    assert _rows(out, w, h) == ["###....###"], "半径 2 不该接上 4 px 的缺口"


def test_isolated_speck_is_dropped():
    """短段丢掉：`##.#.#####` 里那 1 px 的点没了，长段原样留着。"""
    m, w, h = _mask([
        "##.#.#####",
    ])
    out = repair_mask(m, w, h, gap_x=0, short_x=2)
    assert _rows(out, w, h) == ["##...#####"], "1 px 的孤立点应当丢掉、长段保住"


def test_vertical_repair_joins_rows():
    m, w, h = _mask([
        "#",
        ".",
        "#",
    ])
    out = repair_mask(m, w, h, gap_y=1, short_y=1)
    assert _rows(out, w, h) == ["#", "#", "#"]


def test_no_op_returns_copy_and_does_not_touch_input():
    m, w, h = _mask(["#.#"])
    out = repair_mask(m, w, h)
    assert bytes(out) == bytes(m)
    out[0] = 9
    assert m[0] == 1, "不能就地改输入"


def test_repair_stats_counts_runs():
    m, w, h = _mask([
        "#.#.#",
        "#####",
    ])
    st = repair_stats(m, w, h)
    assert st["runs"] == 4.0 and st["rows"] == 2.0
    assert st["runs_per_row"] == 2.0


# --------------------------------------------------------------------------
# 墙段合并
# --------------------------------------------------------------------------
def _seg(x0: int, x1: int, cy: float = 10.0, thickness: float = 8.0) -> WallSegment:
    return WallSegment(x0=x0, x1=x1, cy=cy, thickness=thickness, angle_deg=0.0)


def test_collinear_fragments_merge():
    segs = [_seg(0, 40), _seg(45, 90), _seg(95, 140)]
    out = merge_segments(segs, max_gap_px=8, max_dy_px=3.0)
    assert len(out) == 1 and (out[0].x0, out[0].x1) == (0, 140)


def test_real_opening_is_not_merged():
    """缺口（这里 40 px）远大于 max_gap 时必须保持两段。"""
    segs = [_seg(0, 100), _seg(140, 240)]
    out = merge_segments(segs, max_gap_px=8, max_dy_px=3.0)
    assert len(out) == 2, "真缺口不许被封死"


def test_different_rows_do_not_merge():
    """两堵上下分开的墙（Δcy 大）不许并成一段。"""
    segs = [_seg(0, 100, cy=20.0), _seg(105, 200, cy=200.0)]
    out = merge_segments(segs, max_gap_px=50, max_dy_px=3.0)
    assert len(out) == 2


def test_merge_keeps_the_longer_segment_geometry():
    segs = [_seg(0, 20, thickness=4.0), _seg(24, 200, thickness=9.0)]
    out = merge_segments(segs, max_gap_px=8, max_dy_px=3.0)
    assert len(out) == 1 and out[0].thickness == 9.0, "厚度取较长那一段"
    assert (out[0].x0, out[0].x1) == (0, 200)
