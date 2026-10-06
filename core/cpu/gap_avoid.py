"""wall_with_gap 移动缺口墙 —— 避障算法（纯 Python，零依赖）。

从 Delete_old/gap_wall_demo.py 里抽出（决策部分与原版逐位一致，
4 个场景 × 900 帧比对 0 差异），只去掉了渲染 / 赛道注入 / CLI / 主循环。

--------------------------------------------------------------------------
接口：喂三样东西进来，拿一个速度指令出去
--------------------------------------------------------------------------
输入 WorldView：
    player     : float64[7]  = [x, y, vx, vy, radius, speed, alive]
    rects      : list[Rect]  = 当前帧所有**矩形障碍**（墙的两段就在里面）
    dt         : 仿真步长（秒）
    step_index : 步序号（sim_time = step_index * dt）
    field_w    : 场地宽
    field_h    : 场地高

输出 Decision：
    wx, wy : 归一化速度指令，模长 <= 1；乘上 player[P_SPEED] 就是世界速度
    tier   : 这一帧走了哪个分支（HOLD/TRIM/ALIGN/SLOW/STOP/PANIC/NO_GAP/CRUISE）
    其余字段：诊断量（t_arr / clearance / 缺口区间 / 瞄准点 / 估出的缺口速度）

用法：
    mem = GapMemory()                      # 跨帧状态，每局必须新建
    dec = plan(view, mem, AlgoConfig(), horizon_remaining=2.0)
    wx, wy = to_command(dec)

约定（很重要，代码里到处在用）：
    û = 墙的**跨度**方向（缺口沿它排布）
    n̂ = 墙的**厚度**方向，且规定**从墙指向车**
    于是 (v_wall − v_p)·n̂ > 0 表示"墙在逼近"；
    前进（朝墙）= −n̂，后退（远离墙）= +n̂。

单位：全部世界单位/秒。
--------------------------------------------------------------------------
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

# 玩家状态向量下标（与 bullet_sim/entities/player.py 一致；这里本地定义，整份可抄走）
P_X, P_Y, P_VX, P_VY, P_RADIUS, P_SPEED, P_ALIVE = 0, 1, 2, 3, 4, 5, 6

SHAPE_RECT = 1                # 矩形形状码（圆 = 0）
EPS = 1e-9
#: 几何比较的容差。实测缺口宽度会算成 200.00000000000003 而自由空间是 200.0，
#: 直接用 >= 会因为这个 3e-14 的误差判成"放不下"，算法就整帧摆烂。
TOL = 1e-6


@dataclass
class WorldView:
    """plan() 需要的全部输入。接自己的数据源时只要构造这个对象。"""
    player: Sequence[float]
    rects: list
    dt: float
    step_index: int
    field_w: float
    field_h: float = 0.0


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


@dataclass(frozen=True)
class AlgoConfig:
    #: README 判据 ①：膨胀量 = 玩家外接圆 + safety_margin。
    #: README 没有给出这个数值，这里取 safety/validate.py 里 SafetyConfig 的默认值 6.0。
    safety_margin: float = 6.0
    #: README 判据 ③：验证时窗 = min(回合剩余, max_window)。同样取 SafetyConfig 默认 8.0。
    max_window: float = 8.0
    #: 「已经在缺口里」的判定：横向差 <= align_frac * 可用半孔径 就算到位。
    align_frac: float = 0.6
    #: 横向差小于这个比例 * A 时连微调都不做（防抖）。
    dead_frac: float = 0.05
    #: 是否允许"穿缺口到墙另一侧"。默认关：见文件头说明。
    allow_cross: bool = False
    #: 瞄准用的预测时长（秒）。
    #:
    #: 诚实记录：这一项**在 A/B 里没有测出明显收益**。
    #: 原理上缺口会动时不该用 t_arr（可达 4 秒）去外推 —— 线性拟合外推正弦是垃圾；
    #: 但实测把它从 0.25s 放开到 99s（等于退回"瞄准 t_arr"）后，
    #: 3 局 × 14s 的碰撞数是 0/0/0/0/3/12/6（分别对应下面的扫动档位），
    #: 与 0.25s 的 0/0/0/0/3/6/12 相比，差异都在噪声量级且方向不一致。
    #: 原因大概是：算法每个仿真步都重算，就算瞄准点算错了，"往哪边动"的符号
    #: 仍然是对的，于是它实际上退化成了追踪，最后 t_arr→0 时预测自动收敛到真值。
    #: 保留这个旋钮的理由只是：它把外推误差限制在短时距内，更不容易出怪事；
    #: 嫌多一个参数的话，把它设大即可回到旧行为。
    aim_lookahead: float = 0.25

    #: 允许控制**前后**速度（含减到 0 停住）。
    #:
    #: 方向定义：n̂ 从墙指向车，所以 −n̂ = 朝墙 = 前进，+n̂ = 远离墙 = 后退。
    #: （上一版把这两个方向写反了：代码是 "− n̂" 却注释成"后退"，实测那是在朝墙冲，
    #:   所以那次 A/B 根本没测到后退。这次先把方向钉死。）
    #:
    #: 为什么在"墙朝车压过来"的场景里前后移动没用、在"车朝墙开过去"的场景里却是关键：
    #: 两者相对运动相同，但可用的自由度不同 —— 墙有法向速度时，车停住也拦不住墙；
    #: 墙在行驶方向静止时，**车停住 = 对齐时间变成无限**，这就是"先过前两堵、
    #: 停在第三堵前面慢慢对缺口"能成立的物理原因。
    front_back: bool = False
    #: 关掉前后控制时，恒定的前进速度比例（车不能停）。0 = 完全不前进。
    fwd_speed: float = 0.0
    #: 对齐时与墙保持的净距离（世界单位）。大于 0 才有"停下来对准"的余地。
    hold_clearance: float = 10.0
    #: 固定行驶轴（如 (0,1) = 一直朝 +y 开）。给了它，墙的法向就不再靠
    #: "车在墙哪一侧"来判断 —— 静态墙那个判断是退化的，会让前进方向抖动。
    forward: Optional[tuple[float, float]] = None
    #: 指令方向的**最大转向角速度**（度/秒）。0 = 不限（默认，保持旧行为）。
    #:
    #: 为什么需要：目标墙交接的那一帧，两道缺口可能差 180px，指令方向会瞬间
    #: 转 100° 以上 —— 对真车来说是无穷大横向加速度，画面上就是"突然旋转"。
    #: 限住它才是真车能执行的指令。注意这是**指令层**的限制，不是动力学模型：
    #: 真正的车还要有最小转弯半径（README 里目前没有这个约束）。
    max_turn_rate: float = 0.0


@dataclass(frozen=True)
class Rect:
    """一个矩形实体（README :116-132 的字段子集）。"""

    ent_id: int
    x: float
    y: float
    vx: float
    vy: float
    half_w: float
    half_h: float
    rotation: float
    angular_velocity: float


@dataclass(frozen=True)
class Wall:
    """一堵缺口墙：1~2 段共线矩形 + 它的墙坐标系。

    û = 墙的跨度方向（缺口沿它排布，实测就是 half_w 轴）
    n̂ = 墙厚方向，**规定从墙指向玩家**，(v_wall - v_p)·n̂ > 0 表示"墙在逼近"。
    """

    rects: tuple[Rect, ...]
    u_hat: tuple[float, float]
    n_hat: tuple[float, float]

    @property
    def key(self) -> int:
        """最小实体 id（仅作诊断；真正的跨帧身份见 GapMemory.match_keys）。"""
        return min(r.ent_id for r in self.rects)


def _half_extent_along(r: Rect, d: Sequence[float]) -> float:
    """旋转矩形在单位方向 d 上的半投影长度（精确支撑函数，不是小角度近似）。

        h(d) = half_w * |d · x̂_local| + half_h * |d · ŷ_local|
    """
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    aw = (ct, st)                 # 局部 x 轴 = half_w 方向
    ah = (-st, ct)                # 局部 y 轴 = half_h 方向
    return r.half_w * abs(_dot(d, aw)) + r.half_h * abs(_dot(d, ah))


def _rect_sdf(r: Rect, px: float, py: float) -> tuple[float, tuple[float, float]]:
    """旋转矩形 SDF + 它的单位梯度（README :496：引擎自带圆+旋转矩形 SDF）。"""
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    dx, dy = px - r.x, py - r.y
    lx = dx * ct + dy * st                 # 世界 -> 局部
    ly = -dx * st + dy * ct
    qx = abs(lx) - r.half_w
    qy = abs(ly) - r.half_h
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    sdf = outside + inside

    sx = 1.0 if lx >= 0 else -1.0
    sy = 1.0 if ly >= 0 else -1.0
    if qx > 0.0 and qy > 0.0:              # 角点区
        gx_l, gy_l = sx * qx, sy * qy
        n = math.hypot(gx_l, gy_l) or 1.0
        gx_l, gy_l = gx_l / n, gy_l / n
    elif qx > 0.0:
        gx_l, gy_l = sx, 0.0
    elif qy > 0.0:
        gx_l, gy_l = 0.0, sy
    else:                                  # 在矩形内部：沿最浅的轴推出
        gx_l, gy_l = (sx, 0.0) if qx > qy else (0.0, sy)
    return sdf, (gx_l * ct - gy_l * st, gx_l * st + gy_l * ct)


def group_walls(rects: Sequence[Rect], px: float, py: float,
               forward: Optional[tuple[float, float]] = None) -> list[Wall]:
    """把矩形聚成"一堵墙"。

    **不能用 group_id**：实测本场景里所有墙段的 group_id 都是 0。
    改用几何分组：同一堵墙的各段在"墙厚方向"上坐标几乎相同，
    把该坐标量化到 2 个世界单位一档作为分组键（相邻墙之间 >= 100 单位，不会误并）。
    """
    buckets: dict[tuple[int, int], list[Rect]] = {}
    for r in rects:
        nx = -math.sin(r.rotation)
        ny = math.cos(r.rotation)
        band = r.x * nx + r.y * ny            # 墙厚方向的坐标
        key = (int(round(r.rotation * 1000.0)), int(round(band / 2.0)))
        buckets.setdefault(key, []).append(r)

    walls: list[Wall] = []
    for members in buckets.values():
        members = sorted(members, key=lambda r: r.x)
        ct, st = math.cos(members[0].rotation), math.sin(members[0].rotation)
        u_hat = (ct, st)                      # half_w 轴 = 墙的跨度方向
        # 规范化 û 让它大致指向 +x，这样 e_lo/e_hi、"左侧/右侧"才有稳定含义
        if u_hat[0] < -EPS or (abs(u_hat[0]) <= EPS and u_hat[1] < 0.0):
            u_hat = (-u_hat[0], -u_hat[1])
        n_hat = (-u_hat[1], u_hat[0])

        mx = sum(r.x for r in members) / len(members)
        my = sum(r.y for r in members) / len(members)
        to_player = (px - mx, py - my)
        if forward is not None:
            # 固定行驶轴的场景（小车朝 +forward 开）：n̂ 从墙指向车，恒等于 -forward。
            # **必须这样钉死**：静态墙的法向速度是 0，而"车在墙的哪一侧"在车正好
            # 走到墙这条线上时是退化的（点积 ≈ 0），于是每帧重新判定的 n̂ 会来回翻，
            # "前进"跟着隔帧反号，车就原地抖、永远过不去（实测卡在墙 y 上不动）。
            n_hat = (-forward[0], -forward[1])
        elif abs(_dot(n_hat, to_player)) > TOL:
            if _dot(n_hat, to_player) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
        else:
            # 退化：玩家正好落在墙所在的那条线上，用墙自己的速度定向
            vx = sum(r.vx for r in members) / len(members)
            vy = sum(r.vy for r in members) / len(members)
            if _dot((vx, vy), n_hat) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
        walls.append(Wall(rects=tuple(members[:2]), u_hat=u_hat, n_hat=n_hat))
    return walls


@dataclass
class WallMemory:
    ring: deque = field(default_factory=lambda: deque(maxlen=24))
    gap_width_seen: Optional[float] = None      # 曾经亲眼看到过的缺口宽度
    gap_center_seen: Optional[float] = None     # 曾经亲眼看到过的缺口中心
    n_single_frames: int = 0                    # 只有一段可见的帧数（诊断用）


@dataclass
class GapMemory:
    """跨帧记忆。墙身份按"上一帧最近的墙"关联（不能用 min(ent_id)，见文件头第 4 条）。"""

    walls: dict[int, WallMemory] = field(default_factory=dict)
    global_gap_width: Optional[float] = None
    step_index: int = -1
    _next_key: int = 1
    _prev: list = field(default_factory=list)   # [(band, key)]
    last_dir: Optional[tuple[float, float]] = None   # 上一帧下发过的方向（转向限速用）

    def get(self, key: int) -> WallMemory:
        return self.walls.setdefault(key, WallMemory())

    def match_keys(self, bands: Sequence[float], tol: float = 24.0) -> list[int]:
        prev = list(self._prev)
        keys: list[int] = []
        for band in bands:
            pick = None
            for i, (pb, pk) in enumerate(prev):
                dd = abs(pb - band)
                if pick is None or dd < pick[0]:
                    pick = (dd, i, pk)
            if pick is not None and pick[0] <= tol:
                keys.append(pick[2])
                prev.pop(pick[1])              # 一堵旧墙只能被认领一次
            else:
                keys.append(self._next_key)
                self._next_key += 1
        self._prev = list(zip(bands, keys))
        return keys

    @property
    def single_segment_frames(self) -> int:
        return sum(w.n_single_frames for w in self.walls.values())


@dataclass(frozen=True)
class Aperture:
    e_lo: float          # 可用区间的左端（沿 û，已按场地裁剪）
    e_hi: float          # 右端
    gap_raw: float       # 未裁剪的缺口宽度
    center: float        # 可用区间中心
    usable_half: float   # A = 可用宽度 / 2 - r_p - margin
    complete: bool       # True = 两段都看见了


def reconstruct_aperture(
    wall: Wall, mem: WallMemory, r_p: float, margin: float, field_w: float,
    global_gap: Optional[float] = None,
) -> Aperture:
    """从可见的矩形反算缺口。

    两段都在：e_lo / e_hi = 两条内缘取 min/max，gap = e_hi - e_lo。
    只有一段可见（实测会发生）：用"别处见过"的缺口宽度补出来 ——
      1. 优先用这堵墙自己历史上量到过的宽度；
      2. 否则用任意一堵墙量到过的宽度（同一场景 gap_width 是同一个参数）；
      3. 缺口在可见段的哪一侧：有本墙记忆就按记忆方位，没记忆就看哪一侧放得下。
    """
    rects = sorted(wall.rects, key=lambda r: _dot((r.x, r.y), wall.u_hat))
    u_hat = wall.u_hat

    if len(rects) >= 2:
        rl, rr = rects[0], rects[1]
        inner_a = _dot((rl.x, rl.y), u_hat) + _half_extent_along(rl, u_hat)
        inner_b = _dot((rr.x, rr.y), u_hat) - _half_extent_along(rr, u_hat)
        e_lo, e_hi = min(inner_a, inner_b), max(inner_a, inner_b)
        gap = e_hi - e_lo
        if gap > 0.0:
            mem.gap_width_seen = gap
            mem.gap_center_seen = 0.5 * (e_lo + e_hi)
        complete = True
    else:
        mem.n_single_frames += 1
        r = rects[0]
        proj = _dot((r.x, r.y), u_hat)
        half = _half_extent_along(r, u_hat)
        g_mem = mem.gap_width_seen if mem.gap_width_seen is not None else global_gap
        if g_mem is None or g_mem <= 0.0:
            return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)

        if mem.gap_center_seen is not None:
            on_left = proj < mem.gap_center_seen
        else:
            free_left = proj - half
            free_right = field_w - (proj + half)
            if free_left + TOL >= g_mem and free_left >= free_right:
                on_left = False                      # 缺口在左边 => 可见的是右段
            elif free_right + TOL >= g_mem:
                on_left = True                       # 缺口在右边 => 可见的是左段
            else:
                return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)

        if on_left:
            e_lo = proj + half
            e_hi = e_lo + g_mem
        else:
            e_hi = proj - half
            e_lo = e_hi - g_mem
        gap = e_hi - e_lo
        complete = False

    lo = max(e_lo, 0.0)
    hi = min(e_hi, field_w)
    usable_w = hi - lo
    return Aperture(lo, hi, gap, 0.5 * (lo + hi),
                    0.5 * usable_w - r_p - margin, complete)


@dataclass
class Decision:
    wx: float = 0.0
    wy: float = 0.0
    tier: str = "IDLE"
    reason: str = ""
    t_arr: float = math.inf
    clearance: float = math.inf
    gap: float = math.nan
    usable_half: float = math.nan
    e_lo: float = math.nan
    e_hi: float = math.nan
    aim_lat: float = math.nan
    s_hat: float = 0.0
    n_rects: int = 0
    gap_line: tuple[tuple[float, float], tuple[float, float]] = (
        (math.nan, math.nan), (math.nan, math.nan))
    aim_world: tuple[float, float] = (math.nan, math.nan)
    wall_band: float = math.nan


def _band_point(wall: Wall, lateral: float, band: float) -> tuple[float, float]:
    """（沿 û 的横向坐标, 沿 n̂ 的坐标）-> 世界坐标：P = lateral*û + band*n̂。"""
    ux, uy = wall.u_hat
    nx, ny = wall.n_hat
    return (lateral * ux + band * nx, lateral * uy + band * ny)


def plan(world: Any, mem: GapMemory, cfg: AlgoConfig, horizon_remaining: float) -> Decision:
    """每帧调用一次：世界状态 -> Decision。只读，不修改世界。"""
    p = world.player
    px, py = float(p[P_X]), float(p[P_Y])
    pvx, pvy = float(p[P_VX]), float(p[P_VY])
    r_p = float(p[P_RADIUS])
    v_max = float(p[P_SPEED])
    dt = float(world.dt)
    step = int(world.step_index)
    field_w = float(world.field_w)
    mem.step_index = step

    walls = group_walls(world.rects, px, py, cfg.forward)
    if not walls:
        mem.match_keys(())
        return Decision(tier="IDLE", reason="no_wall")

    bands = [sum(_dot((r.x, r.y), w.n_hat) for r in w.rects) / len(w.rects)
             for w in walls]
    keys = mem.match_keys(bands)

    # --- 选最近的那堵正在逼近的墙；每一堵都要重建缺口（学习全局缺口宽度）---
    cands: list = []
    for w, key in zip(walls, keys):
        front = max(_dot((r.x, r.y), w.n_hat) + _half_extent_along(r, w.n_hat)
                    for r in w.rects)
        thick = 2.0 * max(_half_extent_along(r, w.n_hat) for r in w.rects)
        back = front - thick
        pn = _dot((px, py), w.n_hat)
        vn_wall = sum(_dot((r.vx, r.vy), w.n_hat) for r in w.rects) / len(w.rects)
        v_n = vn_wall - _dot((pvx, pvy), w.n_hat)

        # 必须按玩家半径膨胀墙带、分三段判断（见文件头第 5 条）
        near = front + r_p
        far = back - r_p
        if pn > near:
            t_arr = (pn - near) / v_n if v_n > EPS else math.inf
            prio, key_d = 1, pn - near           # 还在前面：越近越优先
        elif pn >= far:
            t_arr = 0.0                          # 玩家的圆正压在墙带上
            prio, key_d = 0, 0.0                 # 正压着它 => 必须继续盯住这一堵
        else:
            t_arr = math.inf                     # 整面墙已经在身后
            prio, key_d = 2, 0.0                 # 身后 => 不再是约束

        # 选目标墙**必须按位置**（prio/key_d），不能按 t_arr：
        # t_arr = d_n / v_n 里含**车自己的速度**。静态墙场景下，车只要往后动一点点
        # （v_n <= 0），所有墙的 t_arr 同时变成 inf，min() 就会随便挑一堵 ——
        # 目标在两道缺口之间来回跳（实测 ±175px），输出的速度方向隔帧反号，
        # 车原地抖 1.5 秒过不去，画面上就是"过完第一堵墙突然旋转"。
        # 位置判据是单调的，不含速度，永远不会这样退化。

        sdf = min(_rect_sdf(r, px, py)[0] for r in w.rects)
        clearance = sdf - r_p - cfg.safety_margin

        wmem = mem.get(key)
        ap_w = reconstruct_aperture(w, wmem, r_p, cfg.safety_margin, field_w,
                                    mem.global_gap_width)
        if ap_w.complete and ap_w.gap_raw > 0.0:
            mem.global_gap_width = ap_w.gap_raw
        cands.append((w, key, t_arr, clearance, v_n, front, thick, ap_w,
                      prio, key_d))

    (wall, key, t_arr, clearance, v_n, front, thickness, ap, _p, _d) = min(
        cands, key=lambda c: (c[8], c[9]))
    wmem = mem.get(key)
    lat = _dot((px, py), wall.u_hat)

    if ap.usable_half <= 0.0:
        # 推不出缺口 / 膨胀后装不下人：朝远离最近那段墙的方向横移兜底，
        # 别把自己顶到场地边界上。
        r_near = min(wall.rects, key=lambda r: _rect_sdf(r, px, py)[0])
        proj_near = _dot((r_near.x, r_near.y), wall.u_hat)
        sgn = 1.0 if lat < proj_near else -1.0
        if not (0.0 <= lat + sgn * 40.0 <= field_w):
            sgn = -sgn
        return Decision(wx=sgn * wall.u_hat[0], wy=sgn * wall.u_hat[1],
                        tier="NO_GAP", reason="aperture_unknown_flee",
                        t_arr=t_arr, clearance=clearance, gap=ap.gap_raw,
                        usable_half=ap.usable_half, e_lo=ap.e_lo, e_hi=ap.e_hi,
                        n_rects=len(wall.rects),
                        gap_line=(_band_point(wall, ap.e_lo, front),
                                  _band_point(wall, ap.e_hi, front)))

    # --- 缺口轨迹：按墙分别估计（线性拟合 + 残差包络）---
    wmem.ring.append((step, ap.center))
    s_hat, resid = 0.0, 0.0
    if len(wmem.ring) >= 2:
        t0, _s0 = wmem.ring[0]
        n = len(wmem.ring)
        st = stt = ss = sts = 0.0
        for (ti, si) in wmem.ring:
            tt = (ti - t0) * dt
            st += tt
            stt += tt * tt
            ss += si
            sts += tt * si
        den = n * stt - st * st
        s_hat = 0.0 if abs(den) < EPS else (n * sts - st * ss) / den
        b = (ss - s_hat * st) / n
        for (ti, si) in wmem.ring:
            resid = max(resid, abs(si - (b + s_hat * (ti - t0) * dt)))

    def s_lo(t: float) -> float:
        return ap.center + min(0.0, s_hat) * t - resid * t

    def s_hi(t: float) -> float:
        return ap.center + max(0.0, s_hat) * t + resid * t

    H = min(float(horizon_remaining), cfg.max_window)

    def worst_offset(t: float) -> float:
        return max(abs(s_lo(t) - lat), abs(s_hi(t) - lat)) + 0.5 * (s_hi(t) - s_lo(t))

    def reach(t: float) -> float:
        return v_max * max(0.0, t)          # README 判据 ②（本脚本不开加速度）

    def feasible(t: float) -> bool:
        off = worst_offset(t)
        return off <= ap.usable_half or (off - ap.usable_half) <= reach(t)

    # --- 瞄准与判定 ------------------------------------------------------
    # 关键判据（README :300）：墙到达我这条线时，我必须已经在缺口里。
    #
    # 瞄准时刻不取 t_arr 而取 min(t_arr, aim_lookahead)：缺口会动时，用线性拟合
    # 外推 t_arr（开局可达 4 秒）得到的位置毫无意义；短视距 + 每步重算 = 追踪。
    # 瞄准点用**拦截时间**求，而不是固定前瞻：
    #     T 满足 |s(T) - lat| = v_max * T     （车横移 T 秒到达，缺口也走了 T 秒）
    # 迭代几次就收敛（只要缺口速度 < 车速）。为什么必须这样：缺口在滑动时，
    # 固定前瞻 + 比例控制会有稳态滞后 —— 车永远差一截追不上（实测 drive 场景里
    # 车停在墙前左右追着缺口跑、始终不进去）。缺口静止时 s(T) 恒等于 s0，
    # 结果与原来完全一致，所以 incoming 场景的基线不受影响。
    t_aim = abs(0.5 * (s_lo(0.0) + s_hi(0.0)) - lat) / max(v_max, EPS)
    for _ in range(4):
        cand = abs(0.5 * (s_lo(t_aim) + s_hi(t_aim)) - lat) / max(v_max, EPS)
        t_aim = cand if not math.isfinite(t_arr) else min(t_arr, cand)
    aim_lat = 0.5 * (s_lo(t_aim) + s_hi(t_aim))
    d_lat = aim_lat - lat
    need = max(0.0, abs(d_lat) - ap.usable_half)
    sgn_lat = 1.0 if d_lat >= 0 else -1.0

    band = front
    gl = (_band_point(wall, ap.e_lo, band), _band_point(wall, ap.e_hi, band))

    # 横向分量（分级）。这里不再直接 return，因为下面还要叠加前后分量。
    if abs(d_lat) <= cfg.dead_frac * ap.usable_half:
        tier, reason, u_mag = "HOLD", "gap_will_pass_me", 0.0
    elif abs(d_lat) <= cfg.align_frac * ap.usable_half:
        u_mag = min(1.0, abs(d_lat) / max(ap.usable_half, EPS))
        tier, reason = "TRIM", f"recenter dlat={d_lat:+.1f}"
    else:
        u_mag, tier, reason = 1.0, "ALIGN", f"move_to_gap dlat={d_lat:+.1f}"

    # 纯横向来不来得及。注意：前后可控时"减速/停住"能把这条救回来，
    # 所以那种情况下不判 PANIC —— 那正是前后移动的价值所在。
    if math.isfinite(t_arr) and need > reach(t_arr) and not cfg.front_back:
        u_mag, tier, reason = 1.0, "PANIC", "unreachable_before_arrival"

    # --- 前后分量 -------------------------------------------------------
    # n̂ 从墙指向车 => 前进（朝墙）= −n̂，后退（远离墙）= +n̂。
    fwd = (-wall.n_hat[0], -wall.n_hat[1])
    d_n = max(0.0, _dot((px, py), wall.n_hat) - (front + r_p))   # 车到墙的净距离

    u_speed = u_mag * v_max
    w_cap = math.sqrt(max(0.0, v_max * v_max - u_speed * u_speed))  # 剩余速度预算
    if cfg.front_back:
        t_align = need / max(u_speed, EPS) if need > 1e-6 else 0.0
        room = max(0.0, d_n - cfg.hold_clearance)
        t_reach = room / max(w_cap, EPS)
        if t_reach >= t_align:
            w = w_cap                                   # 时间够：全速通过
            if tier == "PANIC":
                tier, reason = "ALIGN", reason
        else:
            # 时间不够：减速到"恰好对齐完成时抵达"，必要时直接停住等
            w = min(w_cap, room / max(t_align, EPS))
            tier = "SLOW" if w > 1e-6 else "STOP"
            reason = f"hold_to_align need={need:.1f} t_align={t_align:.2f}"
    else:
        # 不允许前后控制：**油门是固定的**（车不能停、不能减速），横向只能用剩下的预算。
        # 这才是"只会左右移动的车"的诚实模型：|v| <= v_max 是同一份预算。
        w = min(cfg.fwd_speed * v_max, v_max)
        u_cap = math.sqrt(max(0.0, v_max * v_max - w * w))
        u_mag = min(u_mag, u_cap / max(v_max, EPS))
        if cfg.fwd_speed > 0.0:
            tier, reason = "CRUISE", reason

    w_frac = w / v_max
    wx = u_mag * sgn_lat * wall.u_hat[0] + w_frac * fwd[0]
    wy = u_mag * sgn_lat * wall.u_hat[1] + w_frac * fwd[1]

    # 转向限速：把这一帧的方向相对上一帧的夹角限住。这是"真车做不出来"的那一步。
    if cfg.max_turn_rate > 0.0 and mem.last_dir is not None:
        mag = math.hypot(wx, wy)
        if mag > EPS:
            a_new = math.atan2(wy, wx)
            a_old = math.atan2(mem.last_dir[1], mem.last_dir[0])
            d = (a_new - a_old + math.pi) % (2.0 * math.pi) - math.pi   # 归一化到 (-pi, pi]
            lim = math.radians(cfg.max_turn_rate) * dt
            if abs(d) > lim:
                a_new = a_old + math.copysign(lim, d)
                wx, wy = mag * math.cos(a_new), mag * math.sin(a_new)
    if math.hypot(wx, wy) > EPS:
        mem.last_dir = (wx, wy)

    return Decision(wx=wx, wy=wy, tier=tier, reason=reason,
                    t_arr=t_arr, clearance=clearance, gap=ap.gap_raw,
                    usable_half=ap.usable_half, e_lo=ap.e_lo, e_hi=ap.e_hi,
                    aim_lat=aim_lat, s_hat=s_hat, n_rects=len(wall.rects),
                    gap_line=gl, aim_world=_band_point(wall, aim_lat, band),
                    wall_band=band)

# ===========================================================================
# 6. 适配层（接 bullet_sim 用；接别的数据源可以整段删掉）
#    plan() 本身只用 world.rects，不调用这里的任何函数。
# ===========================================================================
def rect_from_pool(pool: Any, i: int) -> Rect:
    """从 SoA 池子里读一个矩形实体（protocol v3 的几何字段）。"""
    d = pool.data
    return Rect(ent_id=int(d["id"][i]),
                x=float(d["x"][i]), y=float(d["y"][i]),
                vx=float(d["vx"][i]), vy=float(d["vy"][i]),
                half_w=float(d["half_w"][i]), half_h=float(d["half_h"][i]),
                rotation=float(d["rotation"][i]),
                angular_velocity=float(d["angular_velocity"][i]))


def observe_rects(pool: Any) -> list:
    """把池子里所有矩形读成 Rect 列表（非矩形跳过）。"""
    return [rect_from_pool(pool, i) for i in pool.active_indices()
            if int(pool.data["shape"][i]) == SHAPE_RECT]


def view_from_world(world: Any) -> WorldView:
    """把 bullet_sim 的 world 包成 WorldView。"""
    return WorldView(player=[float(v) for v in world.player],
                     rects=observe_rects(world.state.bullets),
                     dt=float(world.state.env.dt),
                     step_index=int(world.state.env.step_index),
                     field_w=float(world.state.env.field_w),
                     field_h=float(world.state.env.field_h))


def to_command(dec: Decision) -> tuple:
    """Decision -> 归一化速度指令 (wx, wy)，模长 <= 1。零向量 = 不动。

    bullet_sim 里再包一层：Action.from_vector((wx, wy))
    """
    if math.hypot(dec.wx, dec.wy) < EPS:
        return (0.0, 0.0)
    return (dec.wx, dec.wy)


if __name__ == "__main__":
    # 自检：不放任何障碍时应当输出 IDLE / 零指令
    _v = WorldView(player=[0.0, 0.0, 0.0, 0.0, 10.0, 200.0, 1.0],
                   rects=[], dt=1.0 / 120.0, step_index=0, field_w=640.0, field_h=480.0)
    _d = plan(_v, GapMemory(), AlgoConfig(), horizon_remaining=2.0)
    assert _d.tier == "IDLE" and to_command(_d) == (0.0, 0.0)
    print("gap_avoid.py 自检通过：空场景 ->", _d.tier, to_command(_d))
