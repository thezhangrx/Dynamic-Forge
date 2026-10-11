"""网格势场 + 波前测地距离 —— 路点选择（纯 Python，零依赖）。

--------------------------------------------------------------------------
这个模块解决什么问题
--------------------------------------------------------------------------
势场法（APF）有一个结构性缺陷：**势是"点"的性质，可达性是"路径"的性质。**

于是"紧贴墙背后"那块地方——它是**自由空间**（排斥势 = 0）、而且
**直线距离很近**——在纯势场看来是一个"又安全又近"的好去处。实测
（玩家在实体墙段正下方、目标在墙后正中）：

    候选点             排斥势   直线距离   测地距离
    A 墙后 (200,276)    0.0      170.4      325.7
    B 缺口 (350.8,260)  0.0      215.8      217.3

两个点的**排斥势都是 0**（都在自由空间里），A 的**直线距离还更短**。
任何"势 + 直线距离"的形式都会选 A，角色就一头撞进实体墙。

修法只有一条：**把"直线距离"换成"可通行距离"（测地距离）**。
测地距离的传播只在自由格里走，穿过墙的那条路代价会自然变大。

--------------------------------------------------------------------------
算法（三步）
--------------------------------------------------------------------------
1. **栅格化**：把障碍按 ``inflate``（= 玩家半径 + 安全余量）膨胀后
   打进一张布尔网格。膨胀后的格子就是"配置空间障碍"。
2. **势场**：两遍 chamfer 距离变换得到每个格到最近障碍的距离 ``clear``，
   再取 ``risk = f(clear)``；被膨胀障碍覆盖的格子记 ``BLOCKED``。
3. **波前**：从玩家所在格做 8 邻接 Dijkstra，得到**测地距离** ``travel``。

然后逐个候选格算决策函数，取最小值：

    J(p) = risk(p) + λ · travel(p)        ← 本模块的默认形式

可选的第二种形式（"过路点总长"，更完整，但要跑两次波前）：

    J(p) = travel_from_player(p) + travel_to_goal(p) + λ · risk(p)

--------------------------------------------------------------------------
为什么不是 `J = 势 / 距离`
--------------------------------------------------------------------------
``∂(U/d)/∂d = -U/d² < 0``：**距离越大值越小**。取最小值等于**奖励"远"**。
实测在 640x480 网格上它选出 (4,4)——场地对角。
要"就近"必须用**加法**（``U + λd``，最小化）或**收益密度**
（``(U_当前 − U(p))/d``，最大化）。

--------------------------------------------------------------------------
交付标准（与 gap_avoid.py 对齐）
--------------------------------------------------------------------------
* 依赖：**只有 stdlib**（math / heapq / dataclasses / typing），无 numpy；
* 平台耦合：**零** —— 输入是一个鸭子类型的 view（见 :class:`FieldView`）；
* 确定性：并列时**按下标取值**，不依赖 dict 迭代顺序（C++ 的
  ``unordered_map`` 会给出不同答案，这一条和 ``gap_avoid.group_walls`` 同源）；
* C++ 陷阱：``heapq`` 的元组比较是字典序，C++ 需自定义 strict weak ordering；
  浮点累加顺序会影响 chamfer 的最后一两位，比对时要允许容差。

--------------------------------------------------------------------------
三种障碍类型都支持 —— 网格是"形状无关"的
--------------------------------------------------------------------------
本模块**不关心障碍是什么**，只关心它的形状与位置：

======================  ==================  ==================================
障碍类型                 光栅化走哪条路        说明
======================  ==================  ==================================
``wall_with_gap``（墙）   ``_raster_rect``    靠波前自然绕到缺口
``moving_block``（方块）  ``_raster_rect``    和墙同一条路；整块挡死时靠降级
``small_obstacles``（弹幕）``_raster_circle`` 圆按 ``半径 + 膨胀量`` 精确膨胀
======================  ==================  ==================================

膨胀对两种形状都是**精确的 Minkowski 和**：矩形用真实 SDF 比阈值
（角上自然变成圆角），圆直接加半径。所以不需要为每种障碍单独写势函数。

实测（每种 3 个种子、30 秒、目标=场中央 ``(320,240)``）：

===============  ==========  ==========  ==========
障碍类型          碰撞事件     重叠帧       原地不动
===============  ==========  ==========  ==========
wall_with_gap     **9**        **189**     48 / 1677
moving_block      **0**        **0**       7 / 535
small_obstacles   **0**        **0**       32 / 793
===============  ==========  ==========  ==========

**弹幕最省事**（小而分散，波前很容易绕）；**方块最干净**（大而少，绕开就行）；
**带缺口的墙最难**（必须精确地钻过去，而且缺口还可能在动）。

**但注意**：这些数字是在**修掉"目标不可达就冻结"这个 bug 之后**才成立的。
修之前是 51 / 8 / 26，看不出方案有用 —— 见下一条。

--------------------------------------------------------------------------
★ 最大的一个坑：目标不可达时会**原地不动当靶子**
--------------------------------------------------------------------------
障碍把场地切开以后，玩家所在连通分量里到不了目标，于是决策场里玩家周围
全是 ``inf``，下降找不到更低的邻居 → 零速度 → **不动**。

实测（修之前）"冻结帧"与"目标不可达帧"**完全相等**：
wall_with_gap 2658/2658、moving_block 793/793、small_obstacles 1197/1197。

判据必须是**玩家所在格**能不能到目标，而不是"全场有几个格能到目标" ——
后者在场地被切开时会给出"能到"的错误答案。目标确实到不了时降级成
"只用风险场"（往最安全处去），并在 ``diag["fallback"]`` 里写明。

修之后冻结帧降到 90 / 0 / 0，碰撞从 51 / 8 / 26 降到 9 / 0 / 0。

--------------------------------------------------------------------------
什么时候**不该**用本模块
--------------------------------------------------------------------------
本模块比 ``gap_avoid.plan()`` 贵得多（前者 O(网格)，后者 O(墙数)）。
它换来的是"能看见绕行代价"这件事。所以：

* 场景里**只有圆弹幕**、没有长条障碍 → 用 ``gap_avoid`` 或平台自带的
  风险栅格决策层就够了，不必上波前；
* 场景里有**带缺口的墙**（必须"绕到缺口再过去"）→ 本模块的价值才体现出来。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

from cpu.gap_avoid import Rect, WorldView, _rect_sdf

#: 被膨胀障碍覆盖的格子，势取这个值（有限大，避免 inf 参与比较时出 NaN）。
BLOCKED = 1.0e9

#: 8 邻接的偏移与单位步长（(di, dj, 权重)）。
_NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
    (0, -1, 1.0),                          (0, 1, 1.0),
    (1, -1, math.sqrt(2.0)),  (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
)


# ==========================================================================
# 输入 / 输出
# ==========================================================================
@dataclass(frozen=True)
class Circle:
    """圆形障碍（世界坐标）。弹幕、小圆球都是它。"""

    ent_id: int
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0


@dataclass(frozen=True)
class FieldView:
    """本模块的输入（鸭子类型，平台适配层负责填）。"""

    #: ``[x, y, vx, vy, radius, speed, alive]``，与 ``gap_avoid.WorldView`` 同构。
    player: Sequence[float]
    rects: Sequence[Rect] = ()
    circles: Sequence[Circle] = ()
    field_w: float = 640.0
    field_h: float = 480.0
    #: 目标点；``None`` 表示只按"安全 + 省路"选路点（不朝任何地方去）。
    goal: Optional[tuple[float, float]] = None
    step_index: int = 0
    dt: float = 1.0 / 120.0


@dataclass
class FieldConfig:
    """网格与决策函数的参数。**每一项都有几何含义，不要盲调。**"""

    #: 网格边长（世界单位）。越小越精细、越慢。
    #: 必须让**可用缺口**至少占 3~4 格：``cell <= usable_gap / 4``。
    cell: float = 8.0
    #: 膨胀量 = 玩家半径 + 安全余量。膨胀后的格子算"撞上"。
    safety_margin: float = 6.0
    #: 排斥势的作用范围（世界单位）。应当 >= 刹车距离。
    repel_range: float = 120.0
    #: 排斥势增益。
    repel_gain: float = 1.0
    #: 行程代价权重 λ。**本模块最重要的一个参数**：
    #: λ 太小 → 只看安全，会挑到"看着近实际远"的陷阱点（见模块头部实测）；
    #: λ 太大 → 只挑最近的，可能贴着障碍走。
    travel_weight: float = 1.0
    #: "该点→目标"那一项的权重。0 = 关掉（那就退化成"逃"，只当负面对照用）。
    via_goal_weight: float = 1.0
    #: 每次重算时沿决策场下降多少格才取方向。
    #: 1 = 只看一个邻居，方向会抖；3~5 明显更平滑（实测）。
    descend_steps: int = 4
    #: 每 N 步重算一次网格（1 = 每步）。>1 时复用上次的路点。
    recompute: int = 1
    #: 网格格数上限；超过就自动加粗 ``cell``，避免病态场景把时间吃光。
    max_cells: int = 6000

    # ---- 时间维（把"静态波前"换成"时间扩展的倒推 DP"）----
    #: 向前预测多少个切片。**0 = 静态**（当前/旧行为），>0 启用时间维。
    #: 代价与它成正比：每多一层多一次栅格化 + 一次全网格松弛。
    time_steps: int = 0
    #: 切片间隔（秒）。``time_steps * time_dt`` 就是决策视野。
    #:
    #: **会被自动放大到至少"走一格的时间"（``cell / speed``）** —— 一层之内
    #: 迈不出一格的话，倒推 DP 全程只能原地不动，时间维会**静默失效**
    #: （实测 cell=16 / speed=200 / dt=1/30 就是这种情况，碰撞 0 → 48）。
    #: 放大后实际值写在 ``diag["time_dt_clamped_to"]`` 里。
    time_dt: float = 1.0 / 30.0

    # ---- 目标来源 ----
    #: ``"auto"``      —— 有 ``plan()`` 的缺口就用，没有就用 ``target``，再不行降级
    #: ``"plan"``      —— 只用 ``plan()`` 的缺口（``moving_block``/``small_obstacles``
    #:                    上它 100% 给不出，会一路降级）
    #: ``"target"``    —— 用平台 ``target`` 实体的位置（``view.goal`` 已是它）
    #: ``"safest"``    —— 不给目标，只用风险场（往最安全处去）
    #: ``"none"``      —— 同 ``"safest"``
    #:
    #: **默认 ``"view"``**：用 ``view.goal``。``plan_field`` 只认这个 ——
    #: 它不会偷偷换掉调用方给的目标（一开始写成 ``"auto"`` 就这样踩了：
    #: 显式传 ``goal=(200,450)`` 被 ``plan()`` 的缺口覆盖，导航测试全挂）。
    goal_policy: str = "view"

    def cell_for(self, field_w: float, field_h: float) -> float:
        """按格数上限反推实际格边长（保证不超过 ``max_cells``）。"""
        cell = max(1.0, float(self.cell))
        while (field_w / cell) * (field_h / cell) > self.max_cells:
            cell *= 1.25
        return cell


@dataclass
class FieldDecision:
    """一帧的输出。``wx, wy`` 是归一化速度指令（模长 <= 1）。"""

    wx: float
    wy: float
    #: 选中的路点（世界坐标）；``None`` 表示没有可去的格子。
    target: Optional[tuple[float, float]] = None
    #: 选中点的决策函数值。
    cost: float = float("inf")
    #: 选中的格下标 ``(ix, iy)``。
    target_cell: Optional[tuple[int, int]] = None
    #: 诊断量。
    diag: dict = field(default_factory=dict)


@dataclass
class FieldMemory:
    """跨帧状态。**每局必须新建**，否则会带着上一局的网格。"""

    grid: Any = None
    last_target: Optional[tuple[float, float]] = None
    last_cost: float = float("inf")
    last_diag: dict = field(default_factory=dict)
    #: **内部**调用计数，用来判缓存是否还有效。
    #:
    #: 这里刻意**不用 ``view.step_index``**：那是调用方填的，填错或忘填
    #: （默认 0）会让缓存永远命中，路点就永远不变 —— 角色朝一个固定的点走过去
    #: 然后原地抖，而**外面看不出任何异常**。写这个模块时正是这么踩了一次：
    #: 导航测试里忘了传 step_index，400 步只净移动了 34 个单位。
    tick: int = 0
    last_tick: int = -1
    recomputes: int = 0
    cells: int = 0
    #: 上一次重算得到的决策场与下降路径。**只给可视化读**，算法本身不用。
    #: 放在内存里而不是让调用方重算一遍：重算一次要 25~55 ms，且可能与
    #: 决策时用的那次不是同一帧（障碍在动），画出来会"对不上"。
    last_field: Optional[list[float]] = None
    last_path: list = field(default_factory=list)
    #: ``goal_policy="auto"`` 时要跑 ``plan()``，它有跨帧状态（墙的身份记忆）。
    gap_mem: Any = None


# ==========================================================================
# 网格
# ==========================================================================
@dataclass
class Grid:
    """一张栅格化的场。``blocked`` 是膨胀后的配置空间障碍。"""

    nx: int
    ny: int
    cell: float
    #: 格中心的 x/y 坐标，长度 nx / ny。
    xs: list[float]
    ys: list[float]
    #: 长度 nx*ny，行主序 ``i*ny + j``（i 是 x 下标）。
    blocked: list[bool]
    clear: list[float]
    risk: list[float]

    def index(self, ix: int, iy: int) -> int:
        return ix * self.ny + iy

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        ix = int(x / self.cell)
        iy = int(y / self.cell)
        return (min(max(ix, 0), self.nx - 1), min(max(iy, 0), self.ny - 1))

    def centre(self, ix: int, iy: int) -> tuple[float, float]:
        return (self.xs[ix], self.ys[iy])


def build_grid(view: FieldView, cfg: FieldConfig) -> Grid:
    """栅格化障碍 + 算距离场 + 算排斥势。O(格数) + O(障碍包围盒)。"""
    cell = cfg.cell_for(view.field_w, view.field_h)
    nx = max(2, int(math.ceil(view.field_w / cell)))
    ny = max(2, int(math.ceil(view.field_h / cell)))
    xs = [(i + 0.5) * cell for i in range(nx)]
    ys = [(j + 0.5) * cell for j in range(ny)]

    rp = float(view.player[4]) if len(view.player) > 4 else 3.0
    inflate = rp + float(cfg.safety_margin)
    blocked = [False] * (nx * ny)

    # ---- ① 栅格化：只扫障碍的膨胀包围盒 ----
    for r in view.rects:
        _raster_rect(r, xs, ys, cell, inflate, nx, ny, blocked)
    for c in view.circles:
        _raster_circle(c, xs, ys, cell, inflate, nx, ny, blocked)

    # ---- ② 两遍 chamfer 距离变换：每个格到最近【障碍】的距离 ----
    clear = _chamfer(blocked, nx, ny, cell)

    # ---- ③ 排斥势 ----
    risk = [0.0] * (nx * ny)
    rng = max(1e-6, float(cfg.repel_range))
    for k in range(nx * ny):
        if blocked[k]:
            risk[k] = BLOCKED
            continue
        d = clear[k]
        if d < rng:
            risk[k] = 0.5 * cfg.repel_gain * (1.0 / max(d, 1e-6) - 1.0 / rng) ** 2
    return Grid(nx, ny, cell, xs, ys, blocked, clear, risk)


def _raster_rect(r: Rect, xs, ys, cell, inflate, nx, ny, blocked) -> None:
    """把一个旋转矩形（按 inflate 膨胀）打进网格。"""
    reach = math.hypot(r.half_w, r.half_h) + inflate
    i0 = max(0, int((r.x - reach) / cell))
    i1 = min(nx - 1, int((r.x + reach) / cell))
    j0 = max(0, int((r.y - reach) / cell))
    j1 = min(ny - 1, int((r.y + reach) / cell))
    for i in range(i0, i1 + 1):
        px = xs[i]
        for j in range(j0, j1 + 1):
            k = i * ny + j
            if blocked[k]:
                continue
            sdf, _ = _rect_sdf(r, px, ys[j])
            if sdf <= inflate:
                blocked[k] = True


def _raster_circle(c: Circle, xs, ys, cell, inflate, nx, ny, blocked) -> None:
    """把一个圆（按 inflate 膨胀）打进网格。"""
    reach = c.radius + inflate
    i0 = max(0, int((c.x - reach) / cell))
    i1 = min(nx - 1, int((c.x + reach) / cell))
    j0 = max(0, int((c.y - reach) / cell))
    j1 = min(ny - 1, int((c.y + reach) / cell))
    r2 = reach * reach
    for i in range(i0, i1 + 1):
        dx = xs[i] - c.x
        for j in range(j0, j1 + 1):
            k = i * ny + j
            if blocked[k]:
                continue
            dy = ys[j] - c.y
            if dx * dx + dy * dy <= r2:
                blocked[k] = True


def _chamfer(blocked: list[bool], nx: int, ny: int, cell: float) -> list[float]:
    """两遍 chamfer 距离变换：到最近 ``blocked`` 格的近似欧氏距离。

    近似误差 < 4%（3-4 chamfer 的固有误差），对"选路点"这个用途远够。
    障碍格自身给 0（下游用 ``risk = BLOCKED`` 覆盖）。
    """
    INF = float("inf")
    d = [0.0 if blocked[i * ny + j] else INF for i in range(nx) for j in range(ny)]
    a, b = cell, cell * math.sqrt(2.0)

    def relax(i, j, di, dj, w):
        ii, jj = i + di, j + dj
        if 0 <= ii < nx and 0 <= jj < ny:
            cand = d[ii * ny + jj] + w
            k = i * ny + j
            if cand < d[k]:
                d[k] = cand

    for i in range(nx):                       # 前向：从左上
        for j in range(ny):
            relax(i, j, -1, 0, a); relax(i, j, 0, -1, a)
            relax(i, j, -1, -1, b); relax(i, j, -1, 1, b)
    for i in range(nx - 1, -1, -1):           # 后向：从右下
        for j in range(ny - 1, -1, -1):
            relax(i, j, 1, 0, a); relax(i, j, 0, 1, a)
            relax(i, j, 1, 1, b); relax(i, j, 1, -1, b)
    return d


# ==========================================================================
# 波前：测地距离
# ==========================================================================
def wavefront_mask(blocked: Sequence[bool], nx: int, ny: int, cell: float,
                   start: tuple[int, int],
                   *, limit: float = float("inf")) -> list[float]:
    """在**任意 blocked 掩膜**上做 8 邻接 Dijkstra，返回测地距离场。

    只走没被占的格 —— 这就是"不会把目标跨墙取"的全部机制。
    """
    n = nx * ny
    dist = [float("inf")] * n
    si = start[0] * ny + start[1]
    if not (0 <= start[0] < nx and 0 <= start[1] < ny) or blocked[si]:
        return dist
    dist[si] = 0.0
    heap: list[tuple[float, int]] = [(0.0, si)]
    while heap:
        d, k = heapq.heappop(heap)
        if d > dist[k]:
            continue
        if d > limit:
            break
        i, j = divmod(k, ny)
        for di, dj, w in _NEIGHBOURS:
            ii, jj = i + di, j + dj
            if not (0 <= ii < nx and 0 <= jj < ny):
                continue
            kk = ii * ny + jj
            if blocked[kk]:
                continue
            nd = d + w * cell
            if nd < dist[kk]:
                dist[kk] = nd
                heapq.heappush(heap, (nd, kk))
    return dist


def wavefront(grid: Grid, start: tuple[int, int],
              *, limit: float = float("inf")) -> list[float]:
    """``Grid`` 上的测地距离场（:func:`wavefront_mask` 的便捷包装）。"""
    return wavefront_mask(grid.blocked, grid.nx, grid.ny, grid.cell, start,
                          limit=limit)



# ==========================================================================
# 时间维：未来 K 个时刻的膨胀障碍 + 时间扩展的倒推 DP
# ==========================================================================
def _motion_offsets(radius: float, cell: float, *, tol: float = 0.5
                    ) -> list[tuple[int, int, float]]:
    """一个 dt 内能到达的格偏移 ``(di, dj, 距离)``，**含原地**。

    等价于平台的 ``safety/path_search.movement_kernel``，纯 Python 重写
    （本模块要零平台耦合）。``tol`` 是半格容差：没有它，一步走不满一格时
    洪泛会**完全卡住** —— 那是离散化假象，不是物理限制。
    """
    r = max(0.0, float(radius))
    slack = float(tol) * cell
    n = int(math.ceil(r / max(cell, 1e-9))) + 1
    out: list[tuple[int, int, float]] = []
    for di in range(-n, n + 1):
        for dj in range(-n, n + 1):
            d = math.hypot(di * cell, dj * cell)
            if (di or dj) and d <= r + slack:
                out.append((di, dj, d))
    out.append((0, 0, 0.0))                      # 原地等一拍
    return out


def blocked_at(view: FieldView, cfg: FieldConfig, cell: float, nx: int, ny: int,
               t: float, inflate: float) -> list[bool]:
    """**未来 t 秒时刻**的膨胀障碍掩膜（``True`` = 被占）。

    外推模型与平台运动积分一致：``p + v·t``、``rot + ω·t``
    （``physics/motion.py``：直线弹幕匀速、自转矩形匀角速度）。
    """
    xs = [(i + 0.5) * cell for i in range(nx)]
    ys = [(j + 0.5) * cell for j in range(ny)]
    blocked = [False] * (nx * ny)
    for r in view.rects:
        rr = Rect(r.ent_id, r.x + r.vx * t, r.y + r.vy * t, r.vx, r.vy,
                  r.half_w, r.half_h, r.rotation + r.angular_velocity * t,
                  r.angular_velocity)
        _raster_rect(rr, xs, ys, cell, inflate, nx, ny, blocked)
    for c in view.circles:
        _raster_circle(Circle(c.ent_id, c.x + c.vx * t, c.y + c.vy * t,
                              c.radius, c.vx, c.vy),
                       xs, ys, cell, inflate, nx, ny, blocked)
    return blocked


def build_layers(view: FieldView, cfg: FieldConfig, *, steps: int, pred_dt: float
                 ) -> tuple[list[list[bool]], float, int, int]:
    """未来 ``steps+1`` 个时刻的膨胀掩膜。

    第 0 层直接复用 ``build_grid`` 的 blocked（**同一份光栅化**，不重算），
    这样"t=0 的层"和静态路径看到的东西完全一样。

    **返回的是 `blocked` 掩膜（``True`` = 被膨胀障碍占住），不是自由掩膜。**
    这条一定要写清楚 —— 一开始没写，``backward_arrival`` 按"自由掩膜"去用，
    条件整个反过来，倒推 DP 只能从"被占的格"往外传，``V[0]`` 几乎全 ``inf``，
    于是**静默降级成"无目标"**（``wall_with_gap`` 从 0 碰撞变 48）。

    返回 ``(layers, cell, nx, ny)``，``layers[k]`` 是 ``t = k·pred_dt`` 的 blocked。
    """
    cell = cfg.cell_for(view.field_w, view.field_h)
    nx = max(2, int(math.ceil(view.field_w / cell)))
    ny = max(2, int(math.ceil(view.field_h / cell)))
    rp = float(view.player[4]) if len(view.player) > 4 else 3.0
    inflate = rp + float(cfg.safety_margin)
    g0 = build_grid(view, cfg)
    layers = [list(g0.blocked)]
    for k in range(1, int(steps) + 1):
        layers.append(blocked_at(view, cfg, cell, nx, ny,
                                 k * float(pred_dt), inflate))
    return layers, cell, nx, ny


def backward_arrival(blocked_layers: Sequence[Sequence[bool]],
                     goal_cell: tuple[int, int], nx: int, ny: int, cell: float,
                     *, speed: float, dt: float,
                     tol: float = 0.5) -> list[float]:
    """**时间扩展的倒推 DP**：``V[0][p]`` = 从 p（此刻）出发到终点所需的最短时间。

    ``blocked_layers[k]`` 是 ``t = k·dt`` 时刻的 **blocked** 掩膜
    （``True`` = 被膨胀障碍占住；与 :func:`build_layers` 一致）。

    ```
    V[K]  = 在最后一层掩膜上的静态测地距离 / 速度      ← 终端代价
    V[k][p] = min over q∈一步可达(p) of ( |p−q|/v + V[k+1][q] )
              要求 p、q 都不被占
    ```

    **终端代价不能写成"只有终点=0、其余 inf"**：那样 K 步倒推只能覆盖
    "K 步内够得着终点"的锥形区域（实测 K=6 时 971 格里只剩 129 格有限），
    玩家一旦在锥外就被判成"目标不可达"而**静默降级**。
    用静态测地距离当初始终端，等价于"视野内考虑障碍运动、视野外假设静止"，
    这是标准的有限视野 DP 写法。

    比"静态波前"多出来的能力：

    * 一颗**正在飞过来**的弹幕 —— 波前知道"等它飞过去再走那一格"；
    * 一个**正在关上的缺口** —— 波前知道赶不上，不会一头扎进去；
    * 一次**正在打开的缺口** —— 波前知道在附近等。

    **必须逐层把被占的格作废**（平台 ``reachable_set`` 的注释点过这个坑：
    不作废的话，某个格会带着"早先某层算出来的有限到达时间"继续当种子，
    等于**直接从移动的墙里穿过去**）。

    返回长度 ``nx*ny`` 的列表（秒）；不可达为 ``inf``。
    """
    if not blocked_layers:
        return [float("inf")] * (nx * ny)
    k_max = len(blocked_layers) - 1
    gi, gj = int(goal_cell[0]), int(goal_cell[1])
    if not (0 <= gi < nx and 0 <= gj < ny):
        return [float("inf")] * (nx * ny)
    # 终点必须在**最后一层**没被占；被占了就挪到最近的空格。
    # **判断别写反**：`blocked_layers` 里 True = 被占。
    if blocked_layers[k_max][gi * ny + gj]:
        alt = _nearest_free(blocked_layers[k_max], nx, ny, gi, gj)
        if alt is None:
            return [float("inf")] * (nx * ny)
        gi, gj = alt

    v = max(1e-9, float(speed))
    offs = _motion_offsets(max(1e-9, v * float(dt)), cell, tol=tol)
    term = wavefront_mask(blocked_layers[k_max], nx, ny, cell, (gi, gj))
    nxt = [(x / v if math.isfinite(x) else float("inf")) for x in term]
    for k in range(k_max - 1, -1, -1):
        blocked_k = blocked_layers[k]
        cur = [float("inf")] * (nx * ny)
        for i in range(nx):
            for j in range(ny):
                if blocked_k[i * ny + j]:
                    continue                       # 此刻被占：不能站
                best = float("inf")
                for di, dj, dist in offs:
                    ii, jj = i + di, j + dj
                    if not (0 <= ii < nx and 0 <= jj < ny):
                        continue
                    nv = nxt[ii * ny + jj]
                    if nv < float("inf"):
                        cand = dist / v + nv
                        if cand < best:
                            best = cand
                cur[i * ny + j] = best
        nxt = cur
    return nxt


def _nearest_free(blocked: Sequence[bool], nx: int, ny: int, i: int, j: int,
                  *, max_ring: int = 12) -> Optional[tuple[int, int]]:
    """离 ``(i,j)`` 最近的**没被占**的格；找不到返回 ``None``。

    ``blocked`` 是 blocked 掩膜（``True`` = 被占）。
    """
    if not blocked[i * ny + j]:
        return (i, j)
    for r in range(1, int(max_ring) + 1):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue
                ii, jj = i + di, j + dj
                if 0 <= ii < nx and 0 <= jj < ny and not blocked[ii * ny + jj]:
                    return (ii, jj)
    return None

# ==========================================================================
# 决策
# ==========================================================================
def build_field(grid: Grid, view: FieldView, cfg: FieldConfig,
                start: Optional[tuple[int, int]] = None
                ) -> tuple[list[float], dict]:
    """合成决策场 ``F(p) = w_目标·travel_到目标(p) + λ·risk(p)``。

    ``travel_到目标`` 用**波前**算（只走自由格），所以它天然含"必须绕到缺口"
    这件事 —— 这就是"不会把目标跨墙取"的全部机制，不需要额外做什么。

    **``start``（玩家所在格）必须传**：判"目标能不能到"要看**玩家所在连通分量**，
    不是"全场有多少格能到目标"。这两件事在障碍把场地切开时完全不同 ——
    只看后者会出现"别的格子能到目标、玩家到不了"，于是玩家的所有邻居 ``F`` 都是
    ``inf``，下降找不到更低的邻居，**角色原地不动当靶子**。
    实测（三种障碍各 3 个种子）：冻结帧与"目标不可达"帧**完全相等**
    （wall_with_gap 2658/2658、moving_block 793/793、small_obstacles 1197/1197）。

    目标确实到不了时**自动降级**成"只用风险场"（往最安全的地方去），
    并在 ``diag["fallback"]`` 里如实说明。

    返回 ``(F, 诊断)``。
    """
    n = grid.nx * grid.ny
    F = [float("inf")] * n
    diag: dict = {"cell": grid.cell}

    base = [grid.risk[k] * cfg.travel_weight for k in range(n)]
    use_goal = view.goal is not None and cfg.via_goal_weight > 0.0
    if use_goal and cfg.time_steps > 0:
        # ---- 时间维：时间扩展的倒推 DP ----
        speed = float(view.player[5]) if len(view.player) > 5 else 200.0
        # ★ **切片间隔必须 ≥ 走一格的时间**，否则一层之内一步都迈不出去，
        #   倒推 DP 全程只能"原地不动" ⇒ V[0] 全 inf ⇒ 静默降级成"无目标"。
        #   实测（cell=16、speed=200、dt=1/30）：一层只能走 6.67，而最近邻格在
        #   16 处 ⇒ 整个时间维失效，`wall_with_gap` 从 0 碰撞变成 48 碰撞，
        #   而 diag 里只看到 `fallback=goal_unreachable`，看不出是间隔太细。
        step_time = grid.cell / max(1e-9, speed)
        eff_dt = max(float(cfg.time_dt), step_time)
        if eff_dt > float(cfg.time_dt) + 1e-12:
            diag["time_dt_clamped_to"] = round(eff_dt, 4)
        layers, cell, lnx, lny = build_layers(
            view, cfg, steps=cfg.time_steps, pred_dt=eff_dt)
        if (lnx, lny) != (grid.nx, grid.ny):
            # 层用和 grid 同一套 cell_for，理论上必然一致；不一致就说明有 bug，
            # 宁可回退到静态也不要画出"对不上的场"。
            diag["time_fallback"] = "layer_size_mismatch"
        else:
            gcell = grid.cell_of(*view.goal)
            arrival = backward_arrival(layers, gcell, grid.nx, grid.ny,
                                       grid.cell, speed=speed, dt=eff_dt)
            n_goal_reach = 0
            for k in range(n):
                if grid.blocked[k] or not math.isfinite(arrival[k]):
                    continue
                # arrival 是**秒**；乘速度换成"等效距离"，让 λ 的量纲不变
                F[k] = cfg.via_goal_weight * arrival[k] * speed + base[k]
                n_goal_reach += 1
            diag["n_goal_reach"] = n_goal_reach
            diag["goal_cell"] = gcell
            diag["time_steps"] = int(cfg.time_steps)
            diag["time_vision_s"] = round(cfg.time_steps * eff_dt, 3)
            start_i = grid.index(*start) if start is not None else None
            player_can_reach = (start_i is not None
                                and math.isfinite(arrival[start_i]))
            diag["player_can_reach_goal"] = bool(player_can_reach)
            if start is not None and not player_can_reach:
                use_goal = False
                diag["fallback"] = ("goal_unreachable_in_time"
                                    if n_goal_reach else "goal_unreachable")
    if use_goal and cfg.time_steps <= 0:
        gcell = grid.cell_of(*view.goal)
        to_goal = wavefront(grid, gcell)
        diag["goal_cell"] = gcell
        if grid.blocked[grid.index(*gcell)]:
            diag["goal_blocked"] = True
        n_goal_reach = sum(1 for k in range(n)
                           if not grid.blocked[k] and math.isfinite(to_goal[k]))
        diag["n_goal_reach"] = n_goal_reach
        # ★ 判据是【玩家所在格】能不能到目标，不是"有几个格能到"
        player_can_reach = (start is not None
                            and math.isfinite(to_goal[grid.index(*start)]))
        diag["player_can_reach_goal"] = bool(player_can_reach)
        if start is not None and not player_can_reach:
            use_goal = False
            diag["fallback"] = ("goal_unreachable_from_player"
                                if n_goal_reach else "goal_unreachable")
        else:
            for k in range(n):
                if not grid.blocked[k] and math.isfinite(to_goal[k]):
                    F[k] = cfg.via_goal_weight * to_goal[k] + base[k]

    if not use_goal:
        # 目标到不了（或本来就没给目标）：只用风险场 —— 往最安全的地方去。
        # 这**会退化成"逃"**（不做任务），但比"原地不动当靶子"好得多。
        for k in range(n):
            if not grid.blocked[k]:
                F[k] = base[k]
        diag.setdefault("fallback", "no_goal")

    finite = [v for v in F if math.isfinite(v)]
    diag["n_finite"] = len(finite)
    diag["f_min"] = min(finite) if finite else float("inf")
    diag["f_max"] = max(finite) if finite else float("inf")
    return F, diag


def line_clear(grid: Grid, a: tuple[int, int], b: tuple[int, int],
               *, samples: int = 0) -> bool:
    """格 ``a`` 到格 ``b`` 的**直线**是否全程落在自由格里。

    为什么要这一步：:func:`descend_field` 给出的是"沿最优路径往前几步"的落点，
    但**从当前位置到那个落点的直线**可能切进膨胀障碍（缺口喉部最容易发生）。
    没有这个检查，输出的方向就可能指向墙里 —— 那和"跨墙取目标"是同一个错误，
    只是换到了输出端。实测 ``cell=4`` 时正是这样卡在缺口口上的。
    """
    ax, ay = grid.centre(*a)
    bx, by = grid.centre(*b)
    n = int(samples) or max(2, int(math.hypot(bx - ax, by - ay) / (0.5 * grid.cell)) + 1)
    for s in range(n + 1):
        t = s / n
        ix, iy = grid.cell_of(ax + (bx - ax) * t, ay + (by - ay) * t)
        if grid.blocked[ix * grid.ny + iy]:
            return False
    return True


def descend_field(grid: Grid, F: list[float], start: tuple[int, int],
                  *, steps: int = 1) -> tuple[tuple[int, int], dict]:
    """从 ``start`` 沿 ``F`` 下降 ``steps`` 步（8 邻接，贪心取最小）。

    这是 **brushfire / navigation function 的下降**：``travel_到目标`` 是波前
    出来的，除了目标点没有局部极小，所以贪心下降能一路走到目标；``risk``
    叠加进去以后可能人为造出局部极小，这里检测并如实报告（不假装没发生）。

    **最后再做一次可视性裁剪**（:func:`line_clear`）：沿下降路径往回收，
    直到"当前位置 → 该落点"的直线全在自由格里。这样输出的方向**不会指向墙**。

    返回 ``(到达的格, 诊断)``。
    """
    i, j = start
    # 起点落在膨胀障碍里（离散推进可能切进来）：先朝最近的自由格挪出去
    if grid.blocked[grid.index(i, j)]:
        escaped = None
        for r in range(1, 4):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    ii, jj = i + di, j + dj
                    if (0 <= ii < grid.nx and 0 <= jj < grid.ny
                            and not grid.blocked[ii * grid.ny + jj]
                            and math.isfinite(F[ii * grid.ny + jj])):
                        escaped = (ii, jj)
                        break
                if escaped:
                    break
            if escaped:
                break
        if escaped is None:
            return (i, j), {"stalled": True, "escaped": False, "descend_steps": 0,
                            "moved": False}
        return escaped, {"stalled": False, "escaped": True, "descend_steps": 1,
                         "path_cells": 2, "clipped": True, "moved": True,
                         "path": [(i, j), escaped]}

    path: list[tuple[int, int]] = [(i, j)]
    for _ in range(max(1, int(steps))):
        best = None
        best_v = F[grid.index(i, j)]
        for di, dj, _w in _NEIGHBOURS:
            ii, jj = i + di, j + dj
            if not (0 <= ii < grid.nx and 0 <= jj < grid.ny):
                continue
            v = F[ii * grid.ny + jj]
            # 严格小于 → 并列取偏移表里靠前的那个（确定性）
            if v < best_v:
                best_v = v
                best = (ii, jj)
        if best is None:
            break
        i, j = best
        path.append((i, j))

    # ---- 可视性裁剪：从最远往回退，取第一个"直线可达"的落点 ----
    chosen = path[-1]
    for cand in reversed(path):
        if cand == start:
            break
        if line_clear(grid, start, cand):
            chosen = cand
            break
    else:
        chosen = path[1] if len(path) > 1 else start
    return chosen, {"stalled": chosen == start, "escaped": False,
                    "descend_steps": len(path) - 1, "path_cells": len(path),
                    "clipped": chosen != path[-1],
                    #: 下降经过的格（可视化用；算法本身不读它）
                    "path": path,
                    # **有没有真的往下走过**：走不通就说明当前格已是（局部）最优，
                    # 这时该【停】，而不是朝本格中心挪 —— 格中心最多离玩家
                    # 0.7·cell，朝它走会让角色在最优点周围以满速抖（实测过）。
                    "moved": len(path) > 1}


def resolve_goal(view: FieldView, cfg: FieldConfig, mem: FieldMemory
                 ) -> tuple[Optional[tuple[float, float]], str, Optional[tuple]]:
    """按 ``cfg.goal_policy`` 决定这一帧去哪。返回 ``(目标, 来源, plan()的瞄准点)``。

    **为什么需要这个**：实测 ``plan()`` 在两种障碍上 **100% 给不出缺口** ——
    ``moving_block`` 是单块（没有"缺口"这个概念）、``small_obstacles`` 全是圆
    （``plan()`` 只读矩形，看不见）。这时如果直接降级成"没有目标"，角色会
    因为风险场平局而**立刻停在原地**（实测）；用平台的 ``target`` 实体就有方向了。

    策略：

    ==========  ==========================================================
    ``auto``    有 ``plan()`` 的缺口就用它；没有就用 ``view.goal``（平台 target）；
                再不行返回 ``None``（降级成只走风险场）
    ``plan``    只用 ``plan()`` 的缺口
    ``target``  只用 ``view.goal``
    ``safest``  不用目标（往最安全处去）
    ``none``    同上
    ==========  ==========================================================
    """
    # **调用方显式给的 ``view.goal`` 绝对优先** —— 决策函数不该偷偷换掉目标。
    # 策略只在"调用方没给"时起作用。``plan_field`` 本身**不调这个函数**：
    # 它保持"目标进 → 方向出"的纯语义；本函数是给调用方（demo / 评测台）用的。
    if view.goal is not None:
        return (float(view.goal[0]), float(view.goal[1])), "view", None

    pol = str(cfg.goal_policy)
    if pol in ("safest", "none", "view"):
        return None, "none", None

    aim: Optional[tuple[float, float]] = None
    if pol in ("auto", "plan"):
        from cpu.gap_avoid import AlgoConfig, GapMemory, plan
        if mem.gap_mem is None:
            mem.gap_mem = GapMemory()
        wv = WorldView(player=list(view.player), rects=list(view.rects),
                       dt=float(view.dt), step_index=int(view.step_index),
                       field_w=float(view.field_w), field_h=float(view.field_h))
        dec = plan(wv, mem.gap_mem, AlgoConfig(), horizon_remaining=1e9)
        ax, ay = float(dec.aim_world[0]), float(dec.aim_world[1])
        if math.isfinite(ax) and math.isfinite(ay):
            aim = (ax, ay)
        if pol == "plan":
            return aim, ("plan" if aim is not None else "none"), aim

    if aim is not None:
        return aim, "plan", aim
    return None, "none", None


def replace_view_goal(view: FieldView, goal: Optional[tuple[float, float]]
                      ) -> FieldView:
    """换掉 ``view.goal``，其余照抄（``FieldView`` 是 frozen dataclass）。"""
    return FieldView(player=view.player, rects=view.rects, circles=view.circles,
                     field_w=view.field_w, field_h=view.field_h, goal=goal,
                     step_index=view.step_index, dt=view.dt)


def _recompute(mem: FieldMemory, view: FieldView, cfg: FieldConfig,
               px: float, py: float, tick: int) -> Optional[FieldDecision]:
    """建网格 + 合成场 + 下降，结果写进 ``mem``。失败时返回一个零速决策。"""
    mem.grid = build_grid(view, cfg)
    mem.cells = mem.grid.nx * mem.grid.ny
    origin = mem.grid.cell_of(px, py)
    F, diag = build_field(mem.grid, view, cfg, origin)
    reach, ddiag = descend_field(mem.grid, F, origin, steps=cfg.descend_steps)
    diag.update(ddiag)
    diag["start_cell"] = origin
    diag.setdefault("goal_policy", cfg.goal_policy)
    mem.last_tick = tick
    mem.last_diag = diag
    mem.recomputes += 1
    if not math.isfinite(F[mem.grid.index(*reach)]):
        mem.last_target = None
        return FieldDecision(0.0, 0.0, None, float("inf"), None,
                             {**diag, "reason": "no finite field near player"})
    mem.last_target = mem.grid.centre(*reach)
    mem.last_cost = F[mem.grid.index(*reach)]
    mem.last_field = F
    mem.last_path = list(ddiag.get("path", ()))
    return None


def plan_field(view: FieldView, mem: FieldMemory, cfg: FieldConfig | None = None
               ) -> FieldDecision:
    """一帧完整流程：建网格 → 合成决策场 → 下降取方向。

    ``recompute > 1`` 时按**调用次数**复用上次的路点（不看 ``view.step_index``：
    那个字段是调用方填的，填错会让缓存永远命中而外面看不出来）。

    **路点落到脚下时会强制重算一次**（``diag["forced_recompute"]``）。
    不这样做会出现一个**退化平衡**：玩家正好停在某个格心上、而那个格心就是
    缓存的路点 ⇒ ``|路点 − 位置| = 0`` ⇒ 输出零速度 ⇒ 位置不变 ⇒ 下一帧还是零，
    一直停到缓存过期（``recompute`` 步）。实测这正是"零速度帧"的**全部**来源：
    28 帧/种子，其中 20 帧是"真的到了目标"、8 帧是这个退化平衡。
    """
    cfg = cfg or FieldConfig()
    p = view.player
    px, py = float(p[0]), float(p[1])
    speed = float(p[5]) if len(p) > 5 else 1.0

    mem.tick += 1
    # (tick-1)//period：第 1..period 次调用都落在同一档，
    # 否则 recompute=N 实际只有 N-1 的周期（差一，写的时候踩过）。
    tick = (mem.tick - 1) // max(1, int(cfg.recompute))

    force = False
    tx = ty = 0.0
    for _attempt in range(2):
        fresh = force or (mem.grid is None or tick != mem.last_tick
                          or mem.last_target is None)
        if fresh:
            bad = _recompute(mem, view, cfg, px, py, tick)
            if bad is not None:
                return bad
            if not mem.last_diag.get("moved", True):
                # 下降找不到任何更低的邻居 ⇒ 已经在（局部）最优。
                # **停**是正确行为；朝本格中心挪会在最优点周围满速抖。
                return FieldDecision(0.0, 0.0, None, mem.last_cost, None,
                                     {**dict(mem.last_diag),
                                      "reason": "at_optimum", "at_optimum": True})
        tx, ty = mem.last_target
        n = math.hypot(tx - px, ty - py)
        if n >= 1e-9:
            ux, uy = (tx - px) / n, (ty - py) / n
            diag = dict(mem.last_diag)
            if force:
                diag["forced_recompute"] = True
            return FieldDecision(ux, uy, (tx, ty), mem.last_cost, None,
                                 {**diag, "dist_to_target": n, "speed": speed,
                                  "player_cell": mem.grid.cell_of(px, py)})
        force = True        # 路点在脚下：不信缓存，逼一次真重算

    # 重算之后路点**仍然**在脚下 ⇒ 玩家已经在最优点（目标本身，或降级后
    # 风险场的最低点）。这时停下是对的。
    return FieldDecision(0.0, 0.0, (tx, ty), mem.last_cost, None,
                         {**dict(mem.last_diag), "reason": "waypoint_underfoot",
                          "at_optimum": True})


def field_from_world(world: Any, *, goal: Optional[tuple[float, float]] = None,
                     step_index: int = 0, dt: float = 1.0 / 120.0) -> FieldView:
    """平台适配：从 bullet_sim 的 ``World`` 抽出 :class:`FieldView`。

    障碍在平台里就是"子弹池"里的实体，靠 ``shape`` 字段区分圆/矩形
    （``0`` = 圆，``1`` = 矩形，见 ``bullet_sim.collision.obstacles``）。
    """
    s = world.state
    pool = s.bullets
    d = pool.data
    idx = pool.active_indices()
    pl = world.player_state()
    spec = getattr(world, "spec", None)
    fw = float(getattr(spec, "field_w", 640.0) or 640.0)
    fh = float(getattr(spec, "field_h", 480.0) or 480.0)

    rects: list[Rect] = []
    circles: list[Circle] = []
    for i in idx:
        eid = int(d["id"][i])
        if int(d["shape"][i]) == 1:
            rects.append(Rect(eid, float(d["x"][i]), float(d["y"][i]),
                              float(d["vx"][i]), float(d["vy"][i]),
                              float(d["half_w"][i]), float(d["half_h"][i]),
                              float(d["rotation"][i]),
                              float(d["angular_velocity"][i])))
        else:
            circles.append(Circle(eid, float(d["x"][i]), float(d["y"][i]),
                                  float(d["radius"][i]),
                                  float(d["vx"][i]), float(d["vy"][i])))
    player = [float(pl.x), float(pl.y), float(pl.vx), float(pl.vy),
              float(pl.radius), float(pl.speed), 1.0 if pl.alive else 0.0]
    return FieldView(player=player, rects=tuple(rects), circles=tuple(circles),
                     field_w=fw, field_h=fh, goal=goal,
                     step_index=step_index, dt=dt)


__all__ = [
    "BLOCKED", "Circle", "FieldView", "FieldConfig", "FieldDecision",
    "FieldMemory", "Grid",
    "build_grid", "wavefront", "build_field", "line_clear", "descend_field",
    "build_layers", "backward_arrival", "resolve_goal", "replace_view_goal",
    "plan_field",
    "field_from_world",
]
