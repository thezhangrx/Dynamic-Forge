"""角色码带（tag strip）的**契约**：平台画它，板端读它。

为什么要它
----------
板端要把画面里的墙行映射成世界 y，就必须知道"**这张图是哪一刻的位姿**"。
靠主机给位姿（`@GPS`）不行：GPS 走 TCP（毫秒），画面走 屏幕→相机（0.16~2.7 s），
两条路延迟不同，量到的永远不是画面那一段（见 `docs/闭环标准配置.md` §5f）。
把**位姿直接编进画面**就绕开了整件事：编码与位姿天然同期，相机延迟再大也不影响。

为什么挂在角色身上
------------------
* 角色永远在视口中心，且画在**最后一层** ⇒ 码带不会被墙盖住；
* 它就是"需要定姿的那个物体" ⇒ 码带与位姿天然同期；
* 不需要改页面布局/标定（比"在场地外立尺子"省掉 6% 分辨率：场地右缘在相机里
  已经贴到幅面边缘，见 §5g 的实测）。

量纲（都是**场地单位**，1 单位 ≈ 0.87 相机像素）
------------------------------------------------
    GUARD 6 │ cell 9 × BITS 9 │ GUARD 6        总宽 93，高 10
    位置：**画面上**角色圆心下方 18（世界 y 更小；角色半径 10 ⇒ 留 8 的空隙）

9 位 × 10 单位 ⇒ 覆盖 5120 > 场地 3420（无歧义），量化误差 ±5。

颜色
----
青 `(R,G,B)=(60,235,235)`：

* 墙是暖白 `(255,240,200)` ⇒ 用 **B−R** 通道就能把码带从墙里分出来
  （灰度上分不开：实测墙 p95 只有 110，码带与墙会被连成一片）；
* `min(R,G,B)=60 < 195` ⇒ **不会**触发视觉链现有的"亮块"判据
  （`vision.primitives` 的 `bright_min`），不会把角色检测挤掉 —— 这是
  `standard/fiducial.py` 里用中灰 128 解决的同一个坑。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

#: 码元宽/高（场地单位）
CELL_W = 9.0
CELL_H = 10.0
#: 两端护带宽度（场地单位）：解码时用**两端护带**锚定整条码带的跨度
GUARD = 6.0
#: 码带矩形**下边缘**相对角色圆心的世界 y 偏移（场地单位，负 = 世界 y 更小 = 画面更下方）。
#: 想清楚这一条：`rects_for` 的矩形 `y` 是**下边缘**（世界 y 向上），高度 10，
#: 所以矩形覆盖世界 y ∈ [player−28, player−18] ⇒ 屏幕上覆盖 [player+18, player+28]
#: ⇒ 与角色圆（半径 10）之间留 8 的缝隙。第一版写成 −18，结果码带贴到圆上（屏幕 +8..+18）。
OFFSET_Y = -28.0
#: 位数与每位代表的世界 y 步长 ⇒ 覆盖 2^BITS * STEP
BITS = 9
STEP = 10.0
#: 码带颜色（RGB）。见文件头"颜色"一节的理由。
COLOR_RGB = (60, 235, 235)
#: 判定"这是码带"用的通道差阈值（B−R，0~255 量纲）
CHANNEL_MIN = 60

#: 一位的两种状态在通道差上的可分性由渲染保证：亮码元 = COLOR_RGB，暗码元 = 背景。
assert 0 <= BITS <= 16
assert STEP * (1 << BITS) > 3420.0, "编码范围必须覆盖整个场地高度"


def code_of(y: float) -> int:
    """世界 y → 码字（取整到 STEP，再对 2^BITS 取模）。"""
    return int(round(float(y) / STEP)) & ((1 << BITS) - 1)


def y_of(code: int) -> float:
    """码字 → 世界 y（量化中点，误差 ±STEP/2）。"""
    return float(int(code) & ((1 << BITS) - 1)) * STEP


def bits_of(y: float) -> list[int]:
    """世界 y → 位序列（**高位在前**，即 bits[0] 是最靠近护带的那一位）。"""
    c = code_of(y)
    return [(c >> (BITS - 1 - i)) & 1 for i in range(BITS)]


def code_of_bits(bits: Sequence[int]) -> int:
    """位序列（高位在前）→ 码字。"""
    c = 0
    for b in bits:
        c = (c << 1) | (1 if b else 0)
    return c


@dataclass(frozen=True)
class StripeRect:
    """一个要画的矩形（世界坐标，**左下角** + 宽高，单位 = 场地单位）。"""

    x: float
    y: float
    w: float
    h: float


def total_width() -> float:
    return 2.0 * GUARD + BITS * CELL_W


def rects_for(player_x: float, player_y: float) -> list[StripeRect]:
    """给定角色世界坐标，返回这条码带要画的矩形（护带 + 亮码元）。

    与 `standard/fiducial.py` 的 `layout.rects()` 同一套语义：只返回**要画的**
    （亮）矩形；暗码元不画（留背景）。
    """
    w = total_width()
    x0 = float(player_x) - w * 0.5
    y0 = float(player_y) + OFFSET_Y      #: −18 ⇒ 画面上在角色下方（见 OFFSET_Y 注释）
    out: list[StripeRect] = [StripeRect(x0, y0, GUARD, CELL_H)]
    bits = bits_of(player_y)
    for i, b in enumerate(bits):
        if b:
            out.append(StripeRect(x0 + GUARD + i * CELL_W, y0, CELL_W, CELL_H))
    out.append(StripeRect(x0 + GUARD + BITS * CELL_W, y0, GUARD, CELL_H))
    return out


def decode_profile(samples: Iterable[float], threshold: float) -> tuple[int, int]:
    """一维剖面 → (码字, 亮元个数)。

    `samples` 是**沿码带宽度方向**等间隔采到的 `B−R` 值（已沿高度平均过），
    调用方负责用两端护带锚定跨度、把采样点落在 `GUARD..GUARD+BITS*CELL_W` 之间。
    这里只做"每位取中位数 → 位 → 码字"。
    """
    vals = list(samples)
    if not vals:
        return 0, 0
    bits: list[int] = []
    n = len(vals)
    for i in range(BITS):
        a = int(n * i / BITS)
        b = max(a + 1, int(n * (i + 1) / BITS))
        seg = vals[a:b]
        seg.sort()
        mid = seg[len(seg) // 2]
        bits.append(1 if mid > threshold else 0)
    return code_of_bits(bits), sum(bits)
