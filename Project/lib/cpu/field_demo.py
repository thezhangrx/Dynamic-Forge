"""网格势场方案的**可视化** —— 把决策场、下降路径、缺口瞄准点画在同一帧上。

--------------------------------------------------------------------------
为什么必须有这个
--------------------------------------------------------------------------
``plan()`` 一直有可视化（``gap_wall_demo.AlgoRenderer`` 画缺口区间 / 瞄准点 /
速度箭头 / 诊断文本），而本方案在加上这个文件之前**完全看不见**。

后果不是"不方便"，是**参数没法调**：``λ``（排斥权重）、``cell``（网格边长）、
``rr``（排斥范围）、``descend_steps``（下降步数）四个参数互相耦合，
只看碰撞数字是**盲调** —— 数字不降的时候你不知道是场没引流对、
还是路点选错、还是膨胀量把缺口堵死了。

看不到的东西不能调。

--------------------------------------------------------------------------
画什么（按"调参时最想知道什么"排序）
--------------------------------------------------------------------------
======================  ====================================================
决策场 F                等值带 + 等值线，绿→黄→红 = 小→大。**这是核心**：
                        一眼看出"场在往哪引流"、有没有异常的墙/沟
膨胀障碍（blocked）     深灰覆盖。看膨胀量合不合适、**缺口有没有被堵死**
下降路径                白线。看贪心下降走的是不是你以为的那条路
路点（waypoint）        青圈。这是最终输出方向的目标
``plan()`` 的瞄准点      黄圈。和路点对比 —— 两者差很多说明有东西不对
速度箭头                橙箭头（玩家的）
HUD                     诊断：cell / λ / f_min / fallback / at_optimum …
======================  ====================================================

--------------------------------------------------------------------------
三种障碍类型：**叠加层是"类型感知"的**
--------------------------------------------------------------------------
``plan()`` 只认"带缺口的墙"。实测它在另外两种类型上**100% 给不出缺口**：

========================  ======  ======  ==========================
障碍类型                   矩形    圆      plan() 给不出缺口的比例
========================  ======  ======  ==========================
``wall_with_gap``           8       0     **0%**
``moving_block``            4       0     **100%**（单块，没有"缺口"）
``small_obstacles``         0      48     **100%**（``plan()`` 看不见圆）
========================  ======  ======  ==========================

所以两种类型上画出来的黄叉是**不可信的** —— 面板会显式写
``plan() : 给不出缺口（黄叉不可信）``，不让它看起来像真目标。

另外这两种障碍的关键信息**全在运动上**（方块横扫、弹幕高速），
只看当前位置说明不了问题。于是多了两个图层：

======================  ====================================================
**预测幽灵**            每个障碍在未来 0.15 / 0.30 / 0.45 秒的位置。
                        方块画旋转矩形轮廓（青），弹幕画圆（粉），越近越实。
**障碍速度箭头**        每个障碍一根，长度 = 未来 0.35 秒的位移。
======================  ====================================================

外推模型是**匀速直行 + 匀角速度**（和平台的运动积分一致），5 行算完，
不复用平台预测器 —— 本模块要零平台耦合。

**幽灵顺手把"静态波前"这个局限画出来了**：你会看到决策场在绕开一个
"此刻在那、0.30 秒后就不在了"的弹幕。那正是"波前加时间维"要修的东西
（平台已有 ``safety/path_search.reachable_set``，按时间分层）。

--------------------------------------------------------------------------
用法
--------------------------------------------------------------------------
    python cpu_field_demo.py                  # 交互窗口（人看）
    python cpu_field_demo.py --save out.png   # 离屏渲一帧存 PNG（验证/汇报）
    python cpu_field_demo.py --goal center    # 目标=场中央（平台 target 默认）
    python cpu_field_demo.py --goal gap       # 目标=plan() 每帧给的缺口

``--save`` 那条是刻意留的：**离屏渲图让"可视化对不对"本身可以被验证**
（而不是"窗口开了没报错"就算对）。
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
from dataclasses import dataclass
from typing import Any, Optional, Sequence

# --- 让 `python core/cpu/field_demo.py` 也能跑 -------------------------------
# 本模块的导入名是 **``cpu.field_demo``**（仓库约定：``core/<模块>/`` 放源码，
# 包名就是 ``<模块>``），所以 ``from cpu.xxx import ...`` 需要 ``core/`` 在
# ``sys.path`` 上。装过项目（``pip install -e .``）或走 pytest 时天然满足
# （后者由 ``core/cpu/conftest.py`` 补），但**直接跑这个文件时没有** ——
# 那时 ``sys.path[0]`` 是 ``core/cpu/``，``import cpu`` 找不到。
#
# 相对导入（``from .gap_avoid import``）在包内可以，但直接跑文件时 Python 把
# 它当成 ``__main__``、没有父包，会报 "attempted relative import with no known
# parent package"。所以这里既用绝对导入，又自己把 ``core/`` 补上。
# 同一个做法见 ``core/cpu/gap_wall_demo.py``。
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # core/
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
# ---------------------------------------------------------------------------

from cpu.gap_avoid import AlgoConfig, GapMemory, plan          # noqa: E402
from cpu.potential_field import (BLOCKED, FieldConfig, FieldMemory,  # noqa: E402
                                 FieldView, field_from_world, plan_field,
                                 resolve_goal)

#: 场值色标（小 → 大）。绿=最优、黄=中、红=差。
_RAMP: tuple[tuple[int, int, int], ...] = (
    (40, 170, 90), (120, 200, 70), (210, 210, 70), (235, 150, 60), (215, 70, 60),
)
_BLOCKED_COLOUR = (58, 60, 72)
_PATH_COLOUR = (250, 250, 250)
_WAYPOINT_COLOUR = (80, 230, 235)
_AIM_COLOUR = (255, 220, 60)
_ARROW_COLOUR = (255, 140, 40)
#: 预测幽灵：矩形（方块/墙）偏青，圆（弹幕）偏粉，和场色区分开。
_GHOST_RECT = (120, 205, 255)
_GHOST_CIRCLE = (255, 130, 200)


@dataclass
class FieldOverlay:
    """一帧要画的东西。算法侧填、渲染侧读，中间不耦合。"""

    #: ``(nx, ny, cell, field)``；``field`` 是被 ``BLOCKED`` 标记过的格值。
    field: Optional[tuple[int, int, float, Sequence[float]]] = None
    #: 下降路径（格下标 ``(i, j)`` 列表）。
    path: Sequence[tuple[int, int]] = ()
    #: 最终路点（世界坐标）。
    waypoint: Optional[tuple[float, float]] = None
    #: ``plan()`` 给的缺口瞄准点（世界坐标）；用来和路点对比。
    aim: Optional[tuple[float, float]] = None
    #: 归一化速度指令。
    command: tuple[float, float] = (0.0, 0.0)
    #: **预测幽灵**：每个障碍在未来几个时刻的位置。
    #: 每项是 ``(t, kind, cx, cy, half_w, half_h, rot, radius)``，
    #: ``kind`` 取 ``"rect"`` / ``"circle"``。
    #:
    #: 这是给**移动方块**和**弹幕**准备的核心图层：
    #:
    #: * 方块会横扫过来，只看当前位置不知道它要去哪；
    #: * 弹幕飞得快，当前位置几乎不说明问题。
    #:
    #: 更重要的是它**把"静态波前"这个局限画出来了** —— 你会看到波前在绕开
    #: 一个"此刻在那、0.3 秒后就不在了"的障碍。这是"波前加时间维"要解决的问题。
    ghosts: Sequence[tuple] = ()
    #: 障碍构成 ``(矩形数, 圆数)``。
    mix: tuple[int, int] = (0, 0)
    #: ``plan()`` 是不是在这一帧给不出可用目标（``moving_block`` / ``small_obstacles``
    #: 上是常态）。为真时画出来的黄叉是**不可信**的，必须显式说明。
    aim_blind: bool = False
    #: 诊断文本行。
    rows: Sequence[str] = ()


#: 画几个未来时刻的幽灵。
GHOST_STEPS: tuple[float, ...] = (0.15, 0.30, 0.45)


def predict_ghosts(view: FieldView, *, horizons: Sequence[float] = GHOST_STEPS
                   ) -> list[tuple]:
    """按**匀速直行 + 匀角速度**外推每个障碍的未来位置。

    这个外推模型和平台的运动积分是一致的（``physics/motion.py``：直线弹幕
    ``p + v·t``；自转矩形 ``rot + ω·t``）。**不复用平台的预测器**是为了让可视化
    不依赖平台 —— 本模块的目标之一就是零平台耦合，而 5 行就能算对。

    注意：这里**不预测玩家**。玩家往哪走由决策决定，画出来会变成自证。
    """
    out: list[tuple] = []
    for r in view.rects:
        for h in horizons:
            out.append((h, "rect", r.x + r.vx * h, r.y + r.vy * h,
                        r.half_w, r.half_h, r.rotation + r.angular_velocity * h, 0.0))
    for c in view.circles:
        for h in horizons:
            out.append((h, "circle", c.x + c.vx * h, c.y + c.vy * h,
                        0.0, 0.0, 0.0, c.radius))
    return out


def field_overlay(view: FieldView, mem: FieldMemory, cfg: FieldConfig,
                  *, aim: Optional[tuple[float, float]] = None,
                  label: str = "", ghosts: Optional[Sequence[float]] = None
                  ) -> FieldOverlay:
    """跑一次决策并打包成可画的东西。**只读** ``mem``，不改状态。"""
    dec = plan_field(view, mem, cfg)
    grid = mem.grid
    field = mem.last_field
    packed = None
    if grid is not None and field is not None:
        packed = (grid.nx, grid.ny, grid.cell, field)

    diag = dict(dec.diag)
    p = view.player
    n_rect, n_circ = len(view.rects), len(view.circles)
    ghost_list = predict_ghosts(view, horizons=ghosts) if ghosts else []
    rows = [
        label,
        f"障碍      : 矩形 {n_rect}  圆 {n_circ}",
        f"cell      : {cfg.cell:g}    λ: {cfg.travel_weight:g}",
        (f"时间维    : K={diag['time_steps']} 视野 {diag['time_vision_s']}s"
         + (f" (dt→{diag['time_dt_clamped_to']})" if 'time_dt_clamped_to' in diag
            else "")) if cfg.time_steps else "时间维    : 静态波前",
        f"rr        : {cfg.repel_range:g}  下降: {cfg.descend_steps}",
        f"decision  : {'STOP(at_optimum)' if dec.diag.get('at_optimum') else 'MOVE'}",
        f"cost F    : {dec.cost:.1f}",
        f"fallback  : {diag.get('fallback', '-')}   目标: {cfg.goal_policy}",
        f"reach     : {diag.get('n_goal_reach', '-')}"
        f"  can_reach: {diag.get('player_can_reach_goal', '-')}",
        f"f_min/max : {diag.get('f_min', float('nan')):.0f}"
        f" / {diag.get('f_max', float('nan')):.0f}",
        f"recompute : {mem.recomputes}   格数: {mem.cells}",
        f"clipped   : {diag.get('clipped', '-')}   moved: {diag.get('moved', '-')}",
        f"player    : ({float(p[0]):.0f},{float(p[1]):.0f})",
    ]
    # plan() 在纯圆场景/单块场景上**看不见或找不到缺口**（实测 100%），
    # 这时它的瞄准点是不可信的 —— 必须写在面板上，不能让它看起来像真的。
    blind = bool(n_circ and not n_rect) or (aim is None)
    if blind:
        rows.insert(1, "plan()    : 给不出缺口（黄叉不可信）")
    return FieldOverlay(field=packed, path=tuple(mem.last_path),
                        waypoint=dec.target, aim=aim,
                        command=(dec.wx, dec.wy), rows=rows,
                        ghosts=tuple(ghost_list), mix=(n_rect, n_circ),
                        aim_blind=blind)


# ==========================================================================
# 渲染
# ==========================================================================
def _ramp(t: float) -> tuple[int, int, int]:
    """``[0,1]`` → 色标。"""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    k = t * (len(_RAMP) - 1)
    i = min(int(k), len(_RAMP) - 2)
    f = k - i
    a, b = _RAMP[i], _RAMP[i + 1]
    return (int(a[0] + (b[0] - a[0]) * f),
            int(a[1] + (b[1] - a[1]) * f),
            int(a[2] + (b[2] - a[2]) * f))


def draw_field(screen: Any, vp: Any, ov: FieldOverlay, *, alpha: int = 205) -> None:
    """只画**决策场本身**（等值带 + 膨胀障碍）。

    ``alpha`` 默认 205 是实测调出来的：95 的时候整个场被世界的暗背景
    （``(12,14,22)``）糊成一片分不清的深绿/深红，**看不出漏斗形状** ——
    而看漏斗正是这个可视化的全部意义。

    **障碍和玩家要在这一步之后重新画一遍**（见 :func:`make_renderer`），
    否则会被场盖住。只画场、不画别的，就是为了让调用方能控制这个顺序。
    """
    import pygame

    w, h = screen.get_size()
    layer = pygame.Surface((w, h), pygame.SRCALPHA)

    # ---- 决策场：等值带 + 等值线 ----
    # 连续色阶太吵，看不出"场在往哪引流"。量化成 N_BANDS 档、并在档边界画细线，
    # 漏斗形状一眼就能看出来 —— 这是调参时最需要的那一眼。
    if ov.field is not None:
        nx, ny, cell, field = ov.field
        finite = [v for v in field if math.isfinite(v) and v < BLOCKED]
        fmax = max(finite) if finite else 1.0
        fmin = min(finite) if finite else 0.0
        span = max(1e-9, fmax - fmin)
        n_bands = 6
        band = [[0] * ny for _ in range(nx)]
        for i in range(nx):
            for j in range(ny):
                v = field[i * ny + j]
                if not math.isfinite(v) or v >= BLOCKED:
                    band[i][j] = -1
                else:
                    band[i][j] = min(n_bands - 1,
                                     int((v - fmin) / span * n_bands))
        for i in range(nx):
            for j in range(ny):
                b = band[i][j]
                if b < 0:
                    continue
                p0 = vp.world_to_screen(i * cell, j * cell)
                p1 = vp.world_to_screen((i + 1) * cell, (j + 1) * cell)
                rect = pygame.Rect(int(min(p0[0], p1[0])), int(min(p0[1], p1[1])),
                                   max(1, int(abs(p1[0] - p0[0]))),
                                   max(1, int(abs(p1[1] - p0[1]))))
                col = _ramp((b + 0.5) / n_bands)
                layer.fill((*col, alpha), rect)
        for i in range(nx):
            for j in range(ny):
                b = band[i][j]
                if b <= 0:
                    continue
                # 和左/下邻居不同档 → 画一条等值线
                if i > 0 and band[i - 1][j] >= 0 and band[i - 1][j] != b:
                    q0 = vp.world_to_screen(i * cell, j * cell)
                    q1 = vp.world_to_screen(i * cell, (j + 1) * cell)
                    pygame.draw.line(layer, (20, 22, 30, 200), q0, q1, 1)
                if j > 0 and band[i][j - 1] >= 0 and band[i][j - 1] != b:
                    q0 = vp.world_to_screen(i * cell, j * cell)
                    q1 = vp.world_to_screen((i + 1) * cell, j * cell)
                    pygame.draw.line(layer, (20, 22, 30, 200), q0, q1, 1)
        # 膨胀障碍：单独一层实色，看膨胀量和"缺口有没有被堵死"
        for i in range(nx):
            for j in range(ny):
                if band[i][j] != -1:
                    continue
                p0 = vp.world_to_screen(i * cell, j * cell)
                p1 = vp.world_to_screen((i + 1) * cell, (j + 1) * cell)
                rect = pygame.Rect(int(min(p0[0], p1[0])), int(min(p0[1], p1[1])),
                                   max(1, int(abs(p1[0] - p0[0]))),
                                   max(1, int(abs(p1[1] - p0[1]))))
                layer.fill((*_BLOCKED_COLOUR, 165), rect)
    screen.blit(layer, (0, 0))


def draw_marks(screen: Any, vp: Any, ov: FieldOverlay) -> None:
    """画**结论**：下降路径、路点、``plan()`` 的瞄准点。

    在障碍/玩家重新画过之后再调用，这样线和圈压在最上面，看得清。
    """
    import pygame

    # ---- 下降路径 ----
    if len(ov.path) >= 2 and ov.field is not None:
        nx, ny, cell, _ = ov.field
        pts = [vp.world_to_screen((i + 0.5) * cell, (j + 0.5) * cell)
               for (i, j) in ov.path
               if 0 <= i < nx and 0 <= j < ny]
        for k in range(1, len(pts)):
            pygame.draw.line(screen, _PATH_COLOUR, pts[k - 1], pts[k], 2)

    # ---- plan() 给的目标（和路点对比用）----
    if ov.aim is not None and all(math.isfinite(v) for v in ov.aim):
        a = vp.world_to_screen(*ov.aim)
        pygame.draw.circle(screen, _AIM_COLOUR, (int(a[0]), int(a[1])), 8, 2)
        pygame.draw.line(screen, _AIM_COLOUR, (a[0] - 12, a[1]), (a[0] + 12, a[1]), 2)
        pygame.draw.line(screen, _AIM_COLOUR, (a[0], a[1] - 12), (a[0], a[1] + 12), 2)

    # ---- 路点 ----
    if ov.waypoint is not None:
        wp = vp.world_to_screen(*ov.waypoint)
        pygame.draw.circle(screen, _WAYPOINT_COLOUR, (int(wp[0]), int(wp[1])), 6, 2)


def _rect_corners(cx: float, cy: float, hw: float, hh: float,
                  rot: float) -> list[tuple[float, float]]:
    """旋转矩形的四个角（世界坐标，逆时针）。"""
    c, s = math.cos(rot), math.sin(rot)
    return [(cx + dx * c - dy * s, cy + dx * s + dy * c)
            for dx, dy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))]


def draw_ghosts(screen: Any, vp: Any, ov: FieldOverlay, *, horizon: float = 0.45
                ) -> None:
    """画**预测幽灵**：每个障碍在未来几个时刻的位置。

    越近的时刻越不透明 —— 所以看上去是一条"运动的拖尾"，一眼能看出
    障碍往哪去、有多快。

    为什么这个图层对**移动方块**和**弹幕**是必需的：这两种障碍的关键信息
    全在运动上（方块横扫、弹幕高速）。只看当前位置几乎说明不了问题。

    另外它顺手把**"静态波前"的局限画出来了**：你会看到决策场在绕开一个
    "此刻在那、0.30 秒后就不在了"的障碍 —— 那正是"波前加时间维"要修的。
    """
    import pygame

    if not ov.ghosts:
        return
    layer = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    for (t, kind, cx, cy, hw, hh, rot, radius) in ov.ghosts:
        a = int(200 * max(0.12, 1.0 - t / max(horizon, 1e-6)))
        col = _GHOST_RECT if kind == "rect" else _GHOST_CIRCLE
        if kind == "rect":
            pts = [vp.world_to_screen(*p) for p in _rect_corners(cx, cy, hw, hh, rot)]
            pygame.draw.polygon(layer, (*col, a), pts, 1)
        else:
            c = vp.world_to_screen(cx, cy)
            r = max(2, int(vp.len_to_screen(radius)))
            pygame.draw.circle(layer, (*col, a), (int(c[0]), int(c[1])), r, 1)
        # 从当前位置到幽灵的连线：把"速度"也画出来
        if kind == "rect":
            o = vp.world_to_screen(cx - 0.0, cy - 0.0)
            del o
    screen.blit(layer, (0, 0))


def draw_obstacle_velocity(screen: Any, vp: Any, view: FieldView,
                           *, length: float = 0.35) -> None:
    """每个障碍画一根速度箭头（长度 = 未来 0.35 秒的位移）。

    和幽灵互补：幽灵给"会到哪"，箭头给"往哪去"。两者一起才看得懂运动。
    """
    import pygame

    layer = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    for c in view.circles:
        if math.hypot(c.vx, c.vy) < 1e-6:
            continue
        p0 = vp.world_to_screen(c.x, c.y)
        p1 = vp.world_to_screen(c.x + c.vx * length, c.y + c.vy * length)
        pygame.draw.line(layer, (*_GHOST_CIRCLE, 170), p0, p1, 2)
    for r in view.rects:
        if math.hypot(r.vx, r.vy) < 1e-6:
            continue
        p0 = vp.world_to_screen(r.x, r.y)
        p1 = vp.world_to_screen(r.x + r.vx * length, r.y + r.vy * length)
        pygame.draw.line(layer, (*_GHOST_RECT, 170), p0, p1, 2)
    screen.blit(layer, (0, 0))


def draw_command_arrow(screen: Any, vp: Any, ov: FieldOverlay,
                       origin: tuple[float, float], *, length: float = 48.0) -> None:
    """速度箭头。``origin`` 是世界坐标（通常就是玩家）。"""
    import pygame

    wx, wy = ov.command
    n = math.hypot(wx, wy)
    if n < 1e-9:
        return
    ox, oy = vp.world_to_screen(*origin)
    ex = ox + wx / n * length
    ey = oy - wy / n * length
    pygame.draw.line(screen, _ARROW_COLOUR, (ox, oy), (ex, ey), 3)
    pygame.draw.circle(screen, _ARROW_COLOUR, (int(ex), int(ey)), 4)


#: 顺序有讲究：``DroidSansFallbackFull`` 只有 CJK 字形、**缺拉丁字母**，
#: 用它渲染 "plan()" 会出六个方块（实测）。Noto Sans CJK 拉丁 + CJK 都有，
#: 所以放前面。
_CJK_CANDIDATES = (
    "/usr/share/fonts/NotoSansCJK-Regular.otf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
)


def _cjk_font(size: int) -> Any:
    """找一个能显示中文的字体；找不到就退回默认（图例会变方块，但不崩）。"""
    import os
    import pygame

    for path in _CJK_CANDIDATES:
        if os.path.exists(path):
            try:
                return pygame.font.Font(path, size)
            except Exception:                            # pragma: no cover
                continue
    return pygame.font.SysFont("monospace", size)


def draw_hud(screen: Any, ov: FieldOverlay, *, x: int = 8, y: int = 8,
             size: int = 13, right: bool = True) -> None:
    """诊断文本。

    ``right=True`` 时**靠右排**：平台自带的 HUD 占着左侧（x=4 起，约 10 行），
    排在左边会重叠成一片糊的（第一次渲出来就是这样）。
    """
    import pygame

    font = _cjk_font(size)
    w = screen.get_size()[0]
    for k, row in enumerate(ov.rows):
        if not row:
            continue
        surf = font.render(row, True, (235, 240, 250))
        bx = (w - surf.get_width() - 12) if right else x
        by = y + k * (size + 3)
        bg = pygame.Surface((surf.get_width() + 6, surf.get_height() + 2),
                            pygame.SRCALPHA)
        bg.fill((10, 12, 20, 185))
        screen.blit(bg, (bx - 3, by - 1))
        screen.blit(surf, (bx, by))


#: 系统里带中日韩字形的字体。pygame 默认字体**没有中文**，直接渲染会出一堆方块。
def draw_legend(screen: Any, ov: FieldOverlay, *, size: int = 13) -> None:
    """底部图例：把"圈是什么颜色"写出来，免得看图的人猜。"""
    import pygame

    font = _cjk_font(size)
    w, h = screen.get_size()
    items = [(_AIM_COLOUR, "plan()瞄准点"), (_WAYPOINT_COLOUR, "势场路点"),
             (_PATH_COLOUR, "下降路径"), (_ARROW_COLOUR, "速度指令"),
             (_GHOST_RECT, "方块预测"), (_GHOST_CIRCLE, "弹幕预测"),
             (_BLOCKED_COLOUR, "膨胀障碍")]
    y = h - size - 8
    x = 10
    for col, text in items:
        pygame.draw.rect(screen, col, (x, y + 2, 10, 10))
        surf = font.render(text, True, (230, 235, 245))
        bg = pygame.Surface((surf.get_width() + 4, surf.get_height()), pygame.SRCALPHA)
        bg.fill((10, 12, 20, 150))
        screen.blit(bg, (x + 13, y))
        screen.blit(surf, (x + 14, y))
        x += 13 + surf.get_width() + 16


# ==========================================================================
# 渲染器：把叠加层挂进 _draw_hud（同一次 flip，不闪）
# ==========================================================================
def make_renderer(**kw: Any) -> Any:
    """建一个带势场叠加层的渲染器。

    **必须走 ``_draw_hud`` 这个钩子**：``PygameRenderer.render()`` 的顺序是
    ``... -> _draw_hud -> （rgb_array 模式）取像素返回``。把叠加层画在
    ``_draw_hud`` 末尾，才能保证它**也在返回的数组里**；画在 ``render()``
    之后就没机会了（那时像素已经取完）。这也是 ``gap_wall_demo.AlgoRenderer``
    用的同一个钩子。

    另外 ``show_text`` 默认是 ``False``，而 ``render()`` 只在
    ``show_hud and show_text`` 时才调 ``_draw_hud`` —— 所以这里强制打开。
    """
    import pygame

    from bullet_sim.render.pygame_view import PygameRenderer

    class FieldRenderer(PygameRenderer):
        def __init__(self, **inner: Any) -> None:
            inner.setdefault("show_text", True)
            inner.setdefault("show_hud", True)
            # 大块 CONTROLS 面板正好压在场中央，挡视野；本可视化不需要它
            inner.setdefault("help_seconds", 0.0)
            super().__init__(**inner)
            self.overlay: Optional[FieldOverlay] = None
            #: 场的透明度。205 是实测值：太低（95）会被暗背景糊成一片。
            self.field_alpha: int = 205
            #: 画不画预测幽灵（移动方块/弹幕必需；带缺口的墙可以不画）
            self.show_ghosts: bool = True
            self.ghost_horizon: float = 0.45
            #: 画幽灵用的视图；由 ``render_frame`` 填。
            self.view_for_ghosts: Optional[FieldView] = None

        def _draw_hud(self, screen: Any, world: Any, fps: float) -> None:
            super()._draw_hud(screen, world, fps)
            if self.overlay is None or self.viewport is None:
                return
            vp = self.viewport
            # ① 决策场（半透明）
            draw_field(screen, vp, self.overlay, alpha=self.field_alpha)
            # ② ★ 把障碍和玩家**重新画一遍**，压在场上。
            #    否则半透明的场会把它们糊掉 —— 而"色块对应哪堵墙"正是要看的东西。
            self._draw_obstacles(screen, world, vp)
            self._draw_player(screen, world, vp)
            # ③ 运动信息：障碍速度箭头 + 预测幽灵（移动方块/弹幕的关键图层）
            if self.view_for_ghosts is not None and self.show_ghosts:
                draw_obstacle_velocity(screen, vp, self.view_for_ghosts)
                draw_ghosts(screen, vp, self.overlay, horizon=self.ghost_horizon)
            # ④ 结论：路径 / 路点 / 瞄准点 / 速度箭头 / 文本
            draw_marks(screen, vp, self.overlay)
            draw_command_arrow(screen, vp, self.overlay,
                               (float(world.player[0]), float(world.player[1])))
            draw_hud(screen, self.overlay)
            draw_legend(screen, self.overlay)

    del pygame
    return FieldRenderer(**kw)


# ==========================================================================
# 一帧：跑决策 + 画 + 返回 RGB 数组
# ==========================================================================
def render_frame(world: Any, mem: FieldMemory, cfg: FieldConfig, renderer: Any,
                 gap_mem: Optional[GapMemory] = None, *, goal_mode: str = "center",
                 step: int = 0, label: str = "",
                 ghost_horizon: float = 0.45) -> tuple[Any, FieldOverlay, tuple]:
    """渲一帧并返回 ``(rgb_array, overlay, 速度指令)``。

    ``goal_mode``：

    * ``"center"`` —— 目标固定在场地中央（平台 ``target`` 的默认位置）
    * ``"gap"``    —— 目标 = ``plan()`` 当帧给出的缺口瞄准点（``Decision.aim_world``）
    * ``"none"``   —— 不给目标（只用风险场，负面对照用）
    """
    # 目标由**调用方**决定（``plan_field`` 只认 ``view.goal``，不会自己换目标）。
    # 四种模式，语义显式：
    #   auto   —— 有 ``plan()`` 的缺口就用，没有就用平台 ``target``（=场中央）
    #   gap    —— 只用 ``plan()`` 的缺口；给不出就是"没有目标"
    #   center —— 固定用平台 ``target``（场中央）
    #   none   —— 不给目标（只用风险场）
    aim = None
    target_goal = (float(world.state.env.field_w) / 2.0,
                   float(world.state.env.field_h) / 2.0)
    if goal_mode in ("gap", "auto"):
        from cpu.adapters.sim_adapter import view_from_world
        dec = plan(view_from_world(world),
                   gap_mem if gap_mem is not None else GapMemory(),
                   AlgoConfig(), horizon_remaining=1e9)
        ax, ay = float(dec.aim_world[0]), float(dec.aim_world[1])
        if math.isfinite(ax) and math.isfinite(ay):
            aim = (ax, ay)
    if goal_mode == "none":
        goal = None
    elif goal_mode == "center":
        goal = target_goal
    elif goal_mode == "gap":
        goal = aim                       # 给不出就是 None
    else:                                # auto
        goal = aim if aim is not None else target_goal

    view = field_from_world(world, goal=goal, step_index=step)
    steps = tuple(h for h in GHOST_STEPS if h <= ghost_horizon + 1e-9)
    ov = field_overlay(view, mem, cfg, aim=aim, label=label, ghosts=steps)
    renderer.overlay = ov
    renderer.view_for_ghosts = view
    arr = renderer.render(world)
    return arr, ov, ov.command


def save_png(path: Any, arr: Any) -> None:
    """把 ``render_frame`` 的数组存成 PNG（不依赖 PIL：用 pygame 写）。"""
    import pygame

    h, w = arr.shape[0], arr.shape[1]
    surf = pygame.Surface((w, h))
    pygame.surfarray.blit_array(surf, arr.transpose(1, 0, 2))
    pygame.image.save(surf, str(path))


# ==========================================================================
# 完整 demo：直接跑本文件 或 走根脚本 cpu_field_demo.py，都调这个
# ==========================================================================
def run_demo(*, obstacle: str = "wall_with_gap", seed: int = 1,
             goal: str = "gap", cell: float = 16.0, descend: int = 6,
             recompute: int = 1, lam: float = 1.0, margin: float = 6.0,
             repel_range: float = 120.0, width: int = 960, height: int = 720,
             frames: int = 0, save: Any = None, hitboxes: bool = False,
             axes: bool = False, quiet: bool = False,
             ghosts: bool = True, ghost_horizon: float = 0.45,
             time_steps: int = 0, time_dt: float = 1.0 / 30.0,
             goal_policy: str = "auto") -> int:
    """跑一整局并把决策场画出来。返回进程退出码。

    逻辑放在模块里（而不是根脚本里），是为了让
    ``python core/cpu/field_demo.py`` 和 ``python cpu_field_demo.py``
    走**同一份**代码 —— 否则两条入口迟早会漂移。
    """
    from bullet_sim.obstacles.scenario import ObstacleScenario
    from bullet_sim.simulator.world import World

    if save is not None:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    mode = "rgb_array" if save is not None else "human"

    scenario = ObstacleScenario(name=obstacle, seed=seed,
                                obstacles=[{"type": obstacle}], require_valid=False)
    built = scenario.build(validate=False)
    world = World(built.spec, codec="velocity")
    cfg = FieldConfig(cell=cell, descend_steps=descend, recompute=recompute,
                      travel_weight=lam, safety_margin=margin,
                      repel_range=repel_range, time_steps=time_steps,
                      time_dt=time_dt, goal_policy=goal_policy)
    renderer = make_renderer(mode=mode, window_size=(width, height),
                             show_hitboxes=hitboxes, show_world_axes=axes,
                             show_fiducials=False)
    renderer.show_ghosts = bool(ghosts)
    renderer.ghost_horizon = float(ghost_horizon)
    mem, gap_mem = FieldMemory(), GapMemory()
    dt = float(built.spec.dt)
    total = int(frames) if frames else int(built.spec.duration / dt)
    last: tuple[Any, FieldOverlay, tuple] | None = None

    for t in range(total):
        last = render_frame(world, mem, cfg, renderer, gap_mem, goal_mode=goal,
                            step=t, label=f"t={t}  目标={goal}",
                            ghost_horizon=float(ghost_horizon) if ghosts else 0.0)
        if mode == "human":
            import pygame
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (
                        ev.type == pygame.KEYDOWN
                        and ev.key in (pygame.K_ESCAPE, pygame.K_q)):
                    renderer.close()
                    return 0
        wx, wy = last[2]
        speed = float(world.player[5]) if world.player.size > 5 else 200.0
        world.step((wx * speed, wy * speed))
        if not world.player_state().alive:
            if not quiet:
                print(f"第 {t} 帧碰撞，结束")
            break

    renderer.close()
    if last is None:
        return 0
    arr, ov, (wx, wy) = last
    if save is not None:
        save_png(save, arr)
        if not quiet:
            print(f"已保存 {save}  （{arr.shape[1]}x{arr.shape[0]}）")
            print(f"路点 {ov.waypoint}   plan()瞄准点 {ov.aim}   "
                  f"指令 ({wx:.3f},{wy:.3f})")
            print(f"下降路径 {len(ov.path)} 格   重算 {mem.recomputes} 次")
    return 0


# ==========================================================================
# 命令行入口（**argparse 放在模块里**，根脚本只转发 —— 同 gap_wall_demo.py）
# ==========================================================================
def build_parser() -> Any:
    import argparse

    ap = argparse.ArgumentParser(
        prog="cpu_field_demo.py",
        description="网格势场方案的决策可视化（决策场 / 下降路径 / 缺口瞄准点）",
        epilog="看图看什么见文件头。直接跑本模块文件与本脚本等效。")
    ap.add_argument("--obstacle", default="wall_with_gap",
                    choices=["wall_with_gap", "moving_block", "small_obstacles",
                             "corridor", "cross_traffic"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--goal", default="auto",
                    choices=["auto", "gap", "center", "none"],
                    help="auto=plan缺口否则平台target；gap=只要plan缺口；"
                         "center=固定场中央；none=不用目标")
    ap.add_argument("--cell", type=float, default=16.0, help="网格边长（世界单位）")
    ap.add_argument("--descend", type=int, default=6, help="沿场下降几步取方向")
    ap.add_argument("--recompute", type=int, default=1, help="每 N 次调用重算一次")
    ap.add_argument("--lam", type=float, default=1.0, help="排斥项权重 λ")
    ap.add_argument("--margin", type=float, default=6.0, help="安全余量")
    ap.add_argument("--repel-range", type=float, default=120.0, dest="repel_range")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--frames", type=int, default=0, help="跑多少帧（0=整局）")
    ap.add_argument("--save", type=pathlib.Path, default=None,
                    help="离屏渲一帧存 PNG（设了就自动用无窗口驱动）")
    ap.add_argument("--hitboxes", action="store_true", help="画碰撞体")
    ap.add_argument("--axes", action="store_true", help="画世界坐标轴")
    ap.add_argument("--time-steps", type=int, default=0, dest="time_steps",
                    help="时间维：向前预测几层（0=静态；6 大约够用，CPU 贵 5 倍）")
    ap.add_argument("--time-dt", type=float, default=1.0 / 30.0, dest="time_dt",
                    help="切片间隔（秒）；会被自动放大到至少走一格的时间")
    ap.add_argument("--goal-policy", default="auto", dest="goal_policy",
                    choices=["auto", "plan", "view", "safest"],
                    help="目标来源：auto=plan缺口→平台target→降级")
    ap.add_argument("--no-ghosts", dest="ghosts", action="store_false",
                    help="不画预测幽灵（移动方块/弹幕默认画，它们是运动的关键）")
    ap.add_argument("--ghost-horizon", type=float, default=0.45,
                    dest="ghost_horizon", help="幽灵最远预测到几秒（默认 0.45）")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    """命令行入口。``argv=None`` 时读 ``sys.argv``。"""
    args = build_parser().parse_args(argv)
    return run_demo(obstacle=args.obstacle, seed=args.seed, goal=args.goal,
                    cell=args.cell, descend=args.descend,
                    recompute=args.recompute, lam=args.lam, margin=args.margin,
                    repel_range=args.repel_range, width=args.width,
                    height=args.height, frames=args.frames, save=args.save,
                    hitboxes=args.hitboxes, axes=args.axes,
                    ghosts=args.ghosts, ghost_horizon=args.ghost_horizon,
                    time_steps=args.time_steps, time_dt=args.time_dt,
                    goal_policy=args.goal_policy)


if __name__ == "__main__":
    # 直接 `python core/cpu/field_demo.py` 也能跑（带完整参数）
    raise SystemExit(main())


__all__ = ["FieldOverlay", "field_overlay", "predict_ghosts", "draw_field",
           "draw_marks", "draw_ghosts", "draw_obstacle_velocity",
           "draw_command_arrow", "draw_hud", "draw_legend", "make_renderer",
           "render_frame", "save_png", "run_demo", "build_parser", "main"]
