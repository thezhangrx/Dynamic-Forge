"""掩膜修复：**颜色分割 → 掩膜修复 → 形状/连通性 → 时序跟踪** 的第 2 步。

参考的两个开源工程（`docs/reference/division/`）：

* **fpga-edge-detection-accelerator** —— 流式"行缓存 + 窗口"架构：不做整帧缓存就能
  在像素流上做邻域运算（它是 3×3 Sobel，这里是形态学开/闭，窗口结构一样），
  而且它的验收方式是**和 PC/OpenCV 参考实现逐像素对账**。本模块就是那份参考实现，
  将来进 RTL 时按它的 `line_buffer.v` / `window_gen.v` 结构翻。
* **image_processing_fpga** —— 阈值**运行时可配**（AXI-Lite + `cv_inrange`）：我们的
  7 个分类阈值现在也已经是运行时寄存器；所以本模块的窗口半径同样做成参数。

## 为什么需要这一步（实测，不是推演）

把墙的色度阈值放宽之后（`--wall-rb 20 --wall-min-r 90`），墙的检出从 28/100 帧升到
96/100 帧 —— 但**一堵墙的掩膜会碎成 4~6 段**（背景的暖色斑块被纳进来 + 斑点把墙
切开）。下游按"行带 → 段 → 缺口"解释掩膜，于是会挑到**碎段之间的小缝**当缺口，
角色就往错的方向对。

单纯继续放宽阈值治不了这个：它只会让背景进来得更多。

## 两条规则，都必须**保守**

1. :func:`repair_mask`：行内"接小缝 + 丢短段"（一趟游程状态机，和 RTL 里
   `primitives.runs/join_close/drop_short` 一一对应），可选再按列做同样的事。
   半径**必须明显小于真实缺口宽度的一半** —— 否则会把通道封死，而对避障来说
   "把缺口封死"比"墙断裂"危险得多。
2. :func:`merge_segments`：把**共线且间隔很小**的细长墙段并成一段（在
   `wall_with_gap.WallSegment` 这一层做，属于"形状/连通性"约束）。

接口刻意只用整数、只做邻域与/或、可分离 —— 直接能翻成 RTL。
"""

from __future__ import annotations

from typing import Iterable, Sequence

from . import primitives as P
from .wall_with_gap import WallSegment

__all__ = ["repair_mask", "repair_stats", "merge_segments"]


# --------------------------------------------------------------------------
# 掩膜修复（行内 + 可选列内）
# --------------------------------------------------------------------------
def _repair_line(flags: Sequence[int], *, gap: int, short: int) -> list[int]:
    """一维（一行或一列）：先接 gap 以内的小缝，再丢掉长度 < short 的碎段。

    与 `primitives.join_close` / `drop_short` 同一套语义（RTL 也是这么写的），
    这里直接复用它们，保证"参考实现和硬件是同一串判断"。
    """
    rs = P.runs(flags)
    if gap > 0:
        rs = P.join_close(rs, gap)
    if short > 1:
        rs = P.drop_short(rs, short)
    out = [0] * len(flags)
    for a, b in rs:
        for i in range(a, b + 1):
            out[i] = 1
    return out


def repair_mask(
    mask: bytearray,
    width: int,
    height: int,
    *,
    gap_x: int = 0,
    short_x: int = 1,
    gap_y: int = 0,
    short_y: int = 1,
) -> bytearray:
    """修复一张 1bit 掩膜（就地不改，返回新的）。

    参数都是**像素**：
    * ``gap_x`` / ``gap_y``：横向 / 纵向允许接上的最大空隙（内含）；
    * ``short_x`` / ``short_y``：横向 / 纵向短于这个长度的碎段丢掉（1 = 不丢）。

    典型用法（保守优先）：``repair_mask(m, w, h, gap_x=3, short_x=2)``
    —— 只接 3 px 以内的小洞、只丢 1 px 的孤立点。
    """
    if width <= 0 or height <= 0:
        return bytearray(mask)
    if gap_x <= 0 and gap_y <= 0 and short_x <= 1 and short_y <= 1:
        return bytearray(mask)

    cur = bytearray(mask)
    if gap_x > 0 or short_x > 1:
        nxt = bytearray(len(cur))
        for y in range(height):
            base = y * width
            row = cur[base:base + width]
            fixed = _repair_line(row, gap=gap_x, short=short_x)
            nxt[base:base + width] = bytes(fixed)
        cur = nxt
    if gap_y > 0 or short_y > 1:
        nxt = bytearray(len(cur))
        for x in range(width):
            col = cur[x::width]
            fixed = _repair_line(col, gap=gap_y, short=short_y)
            nxt[x::width] = bytes(fixed)
        cur = nxt
    return cur


def repair_stats(mask: bytearray, width: int, height: int) -> dict[str, float]:
    """掩膜的"碎裂度"：游程个数 / 每行的平均游程数（越小越连贯）。

    用来量化修复前后的**连续性**（用户提的指标之一）。
    """
    total = 0
    rows_with = 0
    for y in range(height):
        rs = P.runs(mask[y * width:(y + 1) * width])
        if rs:
            rows_with += 1
            total += len(rs)
    return {
        "runs": float(total),
        "rows": float(rows_with),
        "runs_per_row": float(total / rows_with) if rows_with else 0.0,
    }


# --------------------------------------------------------------------------
# 墙段合并（共线 + 小间隔）
# --------------------------------------------------------------------------
def merge_segments(
    segments: Iterable[WallSegment],
    *,
    max_gap_px: int,
    max_dy_px: float,
) -> tuple[WallSegment, ...]:
    """把共线且间隔很小的细长墙段并成一段。

    只做一件事：**同一行带里**（``|Δcy| <= max_dy_px``）且横向间隔
    ``<= max_gap_px`` 的两段合成一段，新的 ``x0/x1`` 取两端。段的厚度与角度取
    较长那一段的（合并本来就是为了"补断"，不该让碎段主导几何）。

    ⚠ ``max_gap_px`` 必须**明显小于真实通道宽度**：这是"不许把缺口封死"的唯一防线。
    真实缺口宽度可以从平台契约反推（缺口 ≥ 1.6 × 角色直径），所以调用方应当传
    它的一个保守分数（例如 1/4）。
    """
    segs = sorted(segments, key=lambda s: s.x0)
    out: list[WallSegment] = []
    for s in segs:
        if out:
            prev = out[-1]
            close_in_y = abs(prev.cy - s.cy) <= max_dy_px
            gap = s.x0 - prev.x1 - 1
            if close_in_y and gap <= max_gap_px:
                longer, other = (prev, s) if prev.length >= s.length else (s, prev)
                out[-1] = WallSegment(
                    x0=min(prev.x0, s.x0),
                    x1=max(prev.x1, s.x1),
                    cy=(prev.cy + s.cy) / 2.0,
                    thickness=longer.thickness,
                    angle_deg=longer.angle_deg,
                )
                _ = other
                continue
        out.append(s)
    return tuple(out)
