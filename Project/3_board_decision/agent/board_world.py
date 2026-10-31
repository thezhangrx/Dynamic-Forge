#!/usr/bin/env python3
"""板端"我认为的世界" —— **`core/board/virt_truth.hpp` 的逐条 Python 移植**。

为什么是移植而不是新写：`虚拟真值约定.md` + `virt_truth.hpp` 是已经用大量真机数据
验收过的那套（角色 8 帧锁初值、零/慢校正、墙"4 帧稳定→冻结"、相对量契约）。
本轮只加**一处**衔接：位姿推进除了老的 `speed_board × 板秒`，还可以吃主机回的
`@STEP <已执行步数>` —— 步数是主机 `t_sim` 的精确计数，比"板子时钟 × 156.6
单位/板秒"更硬（后者那 9% 的补偿就是为了挡板子时钟快）。

对照表（改这里时**必须**同步看 `virt_truth.hpp`，两边不许各写各的）：

    virt_truth.hpp::PlayerState  <-> 本文件 PlayerState
      n_init_need=8 / median 锁初值 / 样本极差 >20 判"检到别的东西"
      gate=60 / alpha=0.25 / bad_run>=5 重锚 / still_eps=10 / step_gate=45 /
      step_ratio=1.8 / 行为一致性门 / 残差门 / correct() 慢校正
    virt_truth.hpp::WallQueue    <-> 本文件 WallQueue
      cap=4 / stable_n=4 / similar=40 / 极差<=20 冻结 / 静 world 不积分 /
      update_relative（rel = 视觉 − 视觉角色，再 + 虚拟角色）/ purge 出场地
"""
from __future__ import annotations

import json
import math
import statistics
from pathlib import Path

#: 动作码 → 单位方向（与 `bullet_sim/core/actions.py`、`virt_truth.hpp` 一致）
_S = 1.0 / math.sqrt(2.0)
DIRS: dict[int, tuple[float, float]] = {
    0: (0.0, 0.0), 1: (-1.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 1.0), 4: (0.0, -1.0),
    5: (-_S, _S), 6: (_S, _S), 7: (-_S, -_S), 8: (_S, -_S),
}


def load_task(path: str | Path) -> dict:
    """板上那份任务文件（**只有自己的信息 + 任务 + 定位基准，没有墙**）。"""
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        "spawn": [float(d["spawn"][0]), float(d["spawn"][1])],
        "goal": [float(d["goal"][0]), float(d["goal"][1])],
        "goal_r": float(d.get("goal_r", 24.0)),
        "field": [float(d["field"][0]), float(d["field"][1])],
        "radius": float(d.get("radius", 10.0)),
        "speed": float(d.get("speed", 200.0)),
        "dt": float(d.get("dt", 1.0 / 120.0)),
        "page": [float(d.get("page", [640.0, 480.0])[0]),
                 float(d.get("page", [640.0, 480.0])[1])],
        #: 老口径的"板秒速度"（boot_cal 实测 156.6）。只有拿不到 `@STEP` 时才用。
        "speed_board": float(d.get("speed_board", 156.6)),
    }


class PlayerState:
    """`virt_truth.hpp::PlayerState`（角色）。单位：场地单位 / 秒。"""

    def __init__(self, half_page: float = 240.0) -> None:
        self.locked = False
        self.x = 0.0
        self.y = 0.0
        self.init: list[tuple[float, float]] = []
        self.n_init_need = 8            # 攒够这么多帧才锁
        self.n_reinit = 0               # 重锚次数（正常应为 0）
        self.bad_run = 0
        self.gate = 60.0
        self.alpha = 0.25
        self.still_eps = 10.0
        self.step_gate = 45.0
        self.step_ratio = 1.8
        self.have_prev_vis = False
        self.pvx = self.pvy = self.pvt = 0.0
        self.n_reject_missing = 0
        self.n_reject_drift = 0
        self.half = float(half_page)
        #: 自己维护的"上一条实际发出的动作"，给积分用
        self.act = 0

    # ---- 锁定 ---------------------------------------------------------
    def observe_init(self, px: float, py: float) -> None:
        if not self.locked:
            self.init.append((float(px), float(py)))

    def try_lock(self) -> bool:
        """攒够样本就锁；初值抖动大 = 检到别的东西，清空重来。"""
        if self.locked or len(self.init) < self.n_init_need:
            return False
        xs = [p[0] for p in self.init]
        ys = [p[1] for p in self.init]
        if (max(xs) - min(xs) > 20.0) or (max(ys) - min(ys) > 20.0):
            self.init.clear()
            return False
        self.x = statistics.median(xs)
        self.y = statistics.median(ys)
        self.locked = True
        self.init.clear()
        return True

    # ---- 推进 ---------------------------------------------------------
    def integrate(self, dt: float, speed_board: float,
                  field_w: float, field_h: float, radius: float) -> None:
        """老口径：上一条实际发出的方向 × 板秒速度 × dt。"""
        if not self.locked:
            return
        ux, uy = DIRS.get(int(self.act), (0.0, 0.0))
        if 0.0 < dt < 0.5:
            self.x += ux * speed_board * dt
            self.y += uy * speed_board * dt
        self._clamp(field_w, field_h, radius)

    def integrate_steps(self, n_steps: int, step_len: float,
                        field_w: float, field_h: float, radius: float) -> float:
        """**新增的衔接**：按主机回的已执行步数推进（t_sim 精确计数）。"""
        if not self.locked or n_steps <= 0:
            return 0.0
        ux, uy = DIRS.get(int(self.act), (0.0, 0.0))
        d = step_len * n_steps
        self.x += ux * d
        self.y += uy * d
        self._clamp(field_w, field_h, radius)
        return abs(d) * math.hypot(ux, uy)

    def _clamp(self, field_w: float, field_h: float, radius: float) -> None:
        m = max(1.0, float(radius) + 1.0)
        self.x = min(max(self.x, m), float(field_w) - m)
        self.y = min(max(self.y, m), float(field_h) - m)

    # ---- 视觉可信度（三道门，任一不过就只积分不更新）------------------
    def vision_ok(self, vx: float, vy: float, t: float, dt: float,
                  speed_board: float) -> bool:
        if not self.locked:
            return False
        ux, uy = DIRS.get(int(self.act), (0.0, 0.0))
        if self.have_prev_vis:
            sx, sy = float(vx) - self.pvx, float(vy) - self.pvy
            moved = math.hypot(sx, sy)
            cmd_stay = (abs(ux) + abs(uy)) < 1e-6
            if cmd_stay and moved > self.still_eps:
                self.n_reject_drift += 1
                self.pvx, self.pvy, self.pvt = float(vx), float(vy), float(t)
                if self._bad():
                    self.x, self.y = float(vx), float(vy)
                return False
            if not cmd_stay:
                expect = speed_board * max(dt, 1e-3)
                along = sx * ux + sy * uy
                if along < -self.still_eps or moved > self.step_ratio * expect + self.step_gate:
                    self.n_reject_drift += 1
                    self.pvx, self.pvy, self.pvt = float(vx), float(vy), float(t)
                    if self._bad():
                        self.x, self.y = float(vx), float(vy)
                    return False
            self.pvx, self.pvy, self.pvt = float(vx), float(vy), float(t)
        else:
            self.pvx, self.pvy, self.pvt = float(vx), float(vy), float(t)
            self.have_prev_vis = True
        if math.hypot(float(vx) - self.x, float(vy) - self.y) <= self.gate:
            self.bad_run = 0
            return True
        self._bad()
        return False

    def _bad(self) -> bool:
        """连续超限 → 判定积分分叉、重锚（True 表示刚重锚）。"""
        self.bad_run += 1
        if self.bad_run >= 5:
            self.bad_run = 0
            self.n_reinit += 1
            return True
        return False

    def correct(self, vx: float, vy: float) -> None:
        """慢校正（alpha=0.25）。只在"视觉被采信"时调用。"""
        self.x += self.alpha * (float(vx) - self.x)
        self.y += self.alpha * (float(vy) - self.y)

    def as_dict(self) -> dict:
        return {"x": round(self.x, 1), "y": round(self.y, 1),
                "locked": bool(self.locked), "n_reinit": int(self.n_reinit),
                "rej_missing": int(self.n_reject_missing),
                "rej_drift": int(self.n_reject_drift)}


class WallSlot:
    __slots__ = ("y", "gap_l", "gap_r", "vy", "t_last", "frozen", "age", "buf")

    def __init__(self) -> None:
        self.y = 0.0
        self.gap_l = 0.0
        self.gap_r = 0.0
        self.vy = 0.0
        self.t_last = 0.0
        self.frozen = False
        self.age = 0
        self.buf: list[tuple[float, float, float, float]] = []

    def as_dict(self) -> dict:
        return {"y": round(self.y, 1), "gl": round(self.gap_l, 1),
                "gr": round(self.gap_r, 1), "frozen": bool(self.frozen),
                "n": len(self.buf)}


class WallQueue:
    """`virt_truth.hpp::WallQueue`（相对观测 + 稳定冻结 + 容量）。"""

    def __init__(self) -> None:
        self.slots: list[WallSlot] = []
        #: 老平台（全幅可见 4 道）cap=4 是对的；**本场景是长廊**（24 行、跟随视口
        #: 一次只看得见 3~4 行），墙要**沿路累积**，所以容量放大。留下 `purge`
        #: 与 `similar=40` 去重，不会无限涨。
        self.cap = 64
        self.stable_n = 4
        self.similar = 40.0
        self.n_admitted = 0
        self.n_released = 0
        #: 平台 v2：墙在世界系静止；冻结后不积分（`vy` 只作诊断）
        self.static_world = True
        self.require_motion = False
        self.vy_min, self.vy_max = 60.0, 260.0

    # ---- 观测 ---------------------------------------------------------
    def update(self, t: float, have: bool, y: float, gap_l: float, gap_r: float) -> str:
        """喂一次观测。冻结的墙**不会被刷新**。返回 freeze/buf/new/frozen/full/none。"""
        if not have:
            return "none"
        y, gap_l, gap_r = float(y), float(gap_l), float(gap_r)
        for s in self.slots:
            ref = s.y if s.frozen else (s.buf[-1][1] if s.buf else s.y)
            if abs(ref - y) <= self.similar:
                if not s.frozen:
                    s.buf.append((float(t), y, gap_l, gap_r))
                    if len(s.buf) >= self.stable_n:
                        ymax, ymin0 = s.buf[-1][1], s.buf[0][1]
                        dtb = s.buf[-1][0] - s.buf[0][0]
                        vy_est = (ymax - ymin0) / dtb if dtb > 0.05 else 0.0
                        stable = abs(ymax - ymin0) <= 20.0
                        plausible = (not self.require_motion
                                     or (self.vy_min <= abs(vy_est) <= self.vy_max))
                        if stable and plausible:
                            self.freeze(s)
                            return "freeze"
                        if len(s.buf) > 16:            # 兜底：攒太多也冻
                            self.freeze(s)
                            return "freeze"
                    return "buf"
                return "frozen"
        if len(self.slots) >= self.cap:
            return "full"
        s = WallSlot()
        s.y, s.gap_l, s.gap_r, s.t_last = y, gap_l, gap_r, float(t)
        s.buf.append((float(t), y, gap_l, gap_r))
        self.slots.append(s)
        self.n_admitted += 1
        return "new"

    def update_relative(self, t: float, have: bool, px: float, py: float,
                        rx: float, ry: float, y: float, gl: float, gr: float) -> str:
        """**相对量契约**：rel = 视觉 − 视觉角色，再 + 虚拟角色 -> 场地坐标。

        `px,py` = 本帧视觉角色（页面坐标）；`rx,ry` = 我们的虚拟角色位姿；
        `y,gl,gr` = 视觉墙几何（与 px/py 同帧、同一套坐标）。
        跟随视口的整体平移、以及任何"公共 y 偏移"都被减掉。
        """
        return self.update(float(t), have, float(ry) + (float(y) - float(py)),
                           float(rx) + (float(gl) - float(px)),
                           float(rx) + (float(gr) - float(px)))

    @staticmethod
    def freeze(s: WallSlot) -> None:
        ys = [b[1] for b in s.buf]
        gl = [b[2] for b in s.buf]
        gr = [b[3] for b in s.buf]
        s.y = statistics.median(ys)
        s.gap_l = statistics.median(gl)
        s.gap_r = statistics.median(gr)
        dt = s.buf[-1][0] - s.buf[0][0]
        s.vy = ((s.buf[-1][1] - s.buf[0][1]) / dt) if dt > 0.05 else 0.0
        s.frozen = True

    def advance(self, t: float) -> None:
        for s in self.slots:
            if s.frozen and not s.static_world:
                dt = max(0.0, min(0.5, float(t) - s.t_last))
                s.y += s.vy * dt
            s.t_last = float(t)
            s.age += 1

    def purge(self, field_h: float) -> None:
        keep = []
        for s in self.slots:
            if s.y < -8.0 or s.y > float(field_h) + 8.0:
                self.n_released += 1
                continue
            keep.append(s)
        self.slots = keep

    # ---- 给上层（agent/M1）的口径 -------------------------------------
    def frozen(self) -> list[WallSlot]:
        return [s for s in self.slots if s.frozen]

    def known_y(self) -> list[float]:
        return sorted(round(s.y, 1) for s in self.frozen())

    def view(self) -> list[dict]:
        return [{"y": round(s.y, 1), "gl": round(s.gap_l, 1), "gr": round(s.gap_r, 1)}
                for s in self.frozen()]

    def pending(self) -> list[WallSlot]:
        return [s for s in self.slots if not s.frozen]

    def as_dict(self) -> dict:
        return {"slots": len(self.slots), "frozen": len(self.frozen()),
                "admitted": int(self.n_admitted), "released": int(self.n_released),
                "cap": int(self.cap)}


class World:
    """角色 + 墙队列 + 场地参数（老 `main.cpp` 里 vp/wq 那对的容器）。"""

    def __init__(self, task: dict) -> None:
        self.task = task
        self.field_w = float(task["field"][0])
        self.field_h = float(task["field"][1])
        self.r = float(task["radius"])
        self.speed = float(task["speed"])
        self.dt = float(task["dt"])
        self.speed_board = float(task.get("speed_board", 156.6))
        self.step_len = self.speed * self.dt
        self.player = PlayerState(half_page=float(task["page"][1]) * 0.5)
        self.walls = WallQueue()
        #: 出生点已知（任务文件给的），先喂进去；锁定用 lock_now()
        self.player.observe_init(float(task["spawn"][0]), float(task["spawn"][1]))
        self.steps: int | None = None
        self.acks = 0
        self.odom_moved = 0.0
        #: 拍摄延迟估计（秒）+ 上一次的残差/样本数（诊断）
        self.delta_s = 0.0
        self.delta_res = None
        self.delta_n = 0
        self.delta_src = ""
        self._dbg_curve: list[tuple] = []
        self._dbg: dict = {}
        self._delay_buf_mov: list[tuple[float, list[float]]] = []
        self._delay_buf: list[tuple[float, list[float]]] = []

    # ---- 位姿 ---------------------------------------------------------
    @property
    def px(self) -> float:
        return self.player.x

    @property
    def py(self) -> float:
        return self.player.y

    def lock_now(self) -> None:
        """出生点已知时直接锁（等价"攒 8 帧锁中位"，这里样本就是出生点）。"""
        if not self.player.locked:
            while len(self.player.init) < self.player.n_init_need:
                self.player.init.append(self.player.init[0])
            self.player.try_lock()

    def on_step(self, steps: int, act: int | None = None) -> float:
        """吃一条 `@STEP <已执行步数> <动作>`：按步数推进（精确）。"""
        steps = int(steps)
        self.acks += 1
        if self.steps is None:
            self.steps = steps
            if act is not None:
                self.player.act = int(act)
            return 0.0
        n = steps - self.steps
        if n < 0:                     # 主机重连/重启：以主机为准重同步
            self.steps = steps
            return 0.0
        if n == 0:
            if act is not None:
                self.player.act = int(act)
            return 0.0
        d = self.player.integrate_steps(n, self.step_len,
                                        self.field_w, self.field_h, self.r)
        self.odom_moved += d
        self.steps = steps
        if act is not None:
            self.player.act = int(act)
        return d

    def on_clock(self, dt: float) -> None:
        """没有 `@STEP` 时的老口径（板秒 × 156.6）。"""
        self.player.integrate(dt, self.speed_board, self.field_w, self.field_h, self.r)

    def reset(self) -> None:
        """新的一段（@START）：位姿回出生点、墙清空。"""
        half = self.player.half
        self.player = PlayerState(half_page=half)
        self.player.observe_init(float(self.task["spawn"][0]),
                                 float(self.task["spawn"][1]))
        self.walls = WallQueue()
        self.steps = None
        self.odom_moved = 0.0

    # ---- 观测 -> 相对量 -----------------------------------------------
    def observe_frame(self, t: float, world_y: float, gl: float, gr: float) -> str:
        """喂一次**已换算到场地系**的墙观测（y 由 `rel_to_world_y` 给出）。

        相对量契约已经在 `rel_to_world_y` 里生效（它用**我们的**角色位姿、而不是
        拍摄时刻的位姿），所以这里直接把场地坐标交给墙队列即可。
        """
        return self.walls.update(float(t), True, float(world_y),
                                 float(gl), float(gr))

    # ---- 拍摄延迟 Δ 的在线估计 ------------------------------------------
    def note_frame(self, t_frame: float, vys: list[float]) -> None:
        """记一帧"板端读到它的时刻 + 它量到的墙行"，给 Δ 估计用。

        **在线估的好处：帧时间和 `py_at()` 用的是**同一个板端单调钟**，
        不需要像离线那样去凑时钟基准（那一步正是离线扫描翻车的地方）。
        """
        if not vys:
            return
        self._delay_buf.append((float(t_frame), [float(v) for v in vys]))
        if len(self._delay_buf) > 240:          # ≈10 s @ 12 fps 的窗
            del self._delay_buf[:-240]

    def bootstrap_delta(self, pose_at, y_scale: float, y_bias: float, *,
                        lo: float = 0.0, hi: float = 1.0, step: float = 0.02,
                        min_span: float = 30.0, need_frames: int = 5,
                        tol_score: float = 6.0, alpha: float = 0.4) -> float:
        """**不依赖任何冻结墙**自举拍摄延迟 Δ（这是方案 A 的正确因果顺序）。

        原理：墙在世界系静止。同一道墙在相邻帧里（vy 变化 ≤25 px，跨不到下一行）
        连成一条链，则

            y(Δ) = (pose_at(t−Δ) − half) + scale·vy(t) + bias

        对整条链必须**恒定**；只有 Δ 等于真实延迟时它才恒定（因为运动单调）。
        所以取"链内 y 极差的中位最小"的那个 Δ。

        缺点/前提：链必须跨过**真的在走**的一段（位姿变化 ≥`min_span`），
        静止段的链对 Δ 没有信息，要丢掉。
        """
        self._dbg = {"buf": len(self._delay_buf)}
        if len(self._delay_buf) < need_frames * 3:
            self._dbg["why"] = "buf_small"
            return self.delta_s
        # 0) **先筛出"在走"的帧**：静止帧对 Δ 零信息，混进来会把估计拉平。
        mov = []
        for (t, vys) in self._delay_buf:
            p0, p1 = pose_at(t), pose_at(t - 0.25)
            if p0 is not None and p1 is not None and abs(p0 - p1) >= 8.0:
                mov.append((t, vys))
        self._delay_buf_mov = mov
        self._dbg["mov"] = len(mov)
        if len(mov) < 8:
            self._dbg["why"] = "mov_small"
            return self.delta_s
        # 1) 相邻帧按最近 vy 串链（只在这批"在走"的帧上）
        #: **每帧的每道墙只许接到一条链上**：以前写成"对所有已在容差内的链
        #: 都 append"，于是一帧的墙被同时接进多条链，链里混进不同墙/不同行，
        #: 估计必然偏（表现为 Δ̂ 乱跳、L=0.2 反而给出 0.42）。
        chains: list[list[tuple[float, float]]] = []
        for (t, vys) in self._delay_buf_mov:
            for v in vys:
                best = None
                for tr in chains:
                    lt, lv = tr[-1]
                    if t - lt > 0.20:
                        continue
                    dv = abs(v - lv)
                    if dv <= 25.0 and (best is None or dv < best[0]):
                        best = (dv, tr)
                if best is not None:
                    best[1].append((t, v))
                else:
                    chains.append([(t, v)])
        self._dbg["chains"] = len(chains)
        chains = [tr for tr in chains if len(tr) >= need_frames]
        self._dbg["chains_long"] = len(chains)
        # 2) 只留"跨过运动"的链
        good = []
        for tr in chains:
            p0, p1 = pose_at(tr[0][0]), pose_at(tr[-1][0])
            if p0 is None or p1 is None or abs(p1 - p0) < min_span:
                continue
            #: **整条链都必须在动**：跨"动→停"的链里，静止段的 y 对任何 Δ 都一致，
            #: 会把估计往小 Δ 拉（实测系统性 −0.18 s）。
            good.append(tr)
        self._dbg["good"] = len(good)
        if not good:
            self._dbg["why"] = "no_moving_chain"
            return self.delta_s
        # 3) 扫 Δ（连续）。曾经改成"按整帧扫"，是为了绕开冒烟注入的粗台阶；
        #: 根因修在冒烟侧（注入连续位姿）之后，连续扫才是对的。
        ts = sorted(t for (t, _v) in self._delay_buf)
        diffs = [b - a for a, b in zip(ts, ts[1:]) if 0.01 < b - a < 0.5]
        self._dbg["dt_frame"] = round(statistics.median(diffs), 4) if diffs else 0.08
        best = None
        d = lo
        while d <= hi + 1e-9:
            spans = []
            for tr in good:
                ys = []
                bad = False
                for (t, vy) in tr:
                    #: **链内也要剔静止帧**：停下那几帧的位姿对任何 Δ 都一样，
                    #: 混进来只会把链内极差按"动帧占比"稀释掉 —— 实测斜率
                    #: ≈0.65 正好等于日程里动帧占比 9/15，看起来像"只估出 65%"。
                    p0, p1 = pose_at(t), pose_at(t - 0.10)
                    if p0 is not None and p1 is not None and abs(p0 - p1) < 3.0:
                        continue
                    p = pose_at(t - d)
                    if p is None:
                        bad = True
                        break
                    ys.append((p - self.player.half) + y_scale * vy + y_bias)
                if not bad and len(ys) >= need_frames:
                    spans.append(max(ys) - min(ys))
            if len(spans) >= 2:
                sc = statistics.median(spans)
                self._dbg_curve.append((round(d, 2), round(sc, 1), len(spans)))
                if best is None or sc < best[0] - 1e-6:
                    best = (sc, d)
            d += step
        self._dbg["curve_len"] = len(self._dbg_curve)
        if self._dbg_curve:
            #: 曲线最低的 3 个点（诊断用：看最小值到底落在哪、平台有多宽）
            self._dbg["top"] = sorted(self._dbg_curve, key=lambda z: z[1])[:3]
        if best is None:
            self._dbg["why"] = "no_curve"
            return self.delta_s
        #: **平台取中**：一段连续 Δ 上分数完全相同（并列）说明它们同等一致，
        #: 取这段的中位而不是左沿（左沿会系统性偏小约半帧~一帧）。
        sc0 = best[0]
        plate = [d for (d, s_) in [(c[0], c[1]) for c in self._dbg_curve]
                 if s_ <= sc0 + 0.2]
        if plate:
            d_best = statistics.median(plate)
            self._dbg["plateau"] = [round(min(plate), 2), round(max(plate), 2)]
        sc, d_best = best[0], d_best
        if sc <= tol_score:
            self.delta_s = (1.0 - alpha) * self.delta_s + alpha * d_best
            self.delta_res = float(sc)
            self.delta_n = len(good)
            self.delta_src = "bootstrap"
        return self.delta_s

    def refresh_delta(self, pose_at, y_scale: float, y_bias: float,
                      *, hi: float = 1.0, step: float = 0.05,
                      tol_px: float = 25.0, alpha: float = 0.3) -> float:
        """扫 Δ∈[0,hi]：让**冻结墙**在当前图上的残差最小（`pose_at(t)` 传板端自己的历史）。

        判据：`predicted_vy = (row.y − (pose_at(t−Δ) − half) − bias)/scale`，
        实测 `vy` 与它差 ≤ `tol_px` 才配对；残差取所有配对的中位。只有明显更好
        （残差 <8 单位）才更新，并做 α 平滑，避免抖动。
        """
        rows = self.walls.frozen()
        if len(rows) < 1 or len(self._delay_buf) < 30:
            return self.delta_s
        #: **可辨识性闸门**：机器人几乎不动时，任何 Δ 都能把残差压到 0（退化），
        #: 于是 Δ 会随机漂（冒烟里就漂到 0.22）。要求这段缓冲里位姿真的走过
        #: ≥60 单位，才允许估；否则保持原值。
        _p0 = pose_at(self._delay_buf[0][0])
        _p1 = pose_at(self._delay_buf[-1][0])
        if _p0 is None or _p1 is None or abs(_p1 - _p0) < 60.0:
            return self.delta_s
        #: **只用"在动"的帧**：静止时任何 Δ 的残差都是 0（没有信息量），
        #: 混进来会把中位数/分位数拉平，估出来的 Δ 随机漂（实测 L=0.2 → 0.01）。
        mov = []
        for (t, vys) in self._delay_buf:
            p0 = pose_at(t)
            p1 = pose_at(t - 0.3)
            if p0 is not None and p1 is not None and abs(p0 - p1) >= 8.0:
                mov.append((t, vys))
        self._dbg["mov"] = len(mov)
        if len(mov) < 8:
            self._dbg["why"] = "mov_small"
            return self.delta_s
        self._delay_buf_mov = mov
        self._dbg["mov"] = len(mov)
        best = None
        n = 0
        d = 0.0
        while d <= hi + 1e-9:
            res = []
            for (t, vys) in mov:
                p = pose_at(t - d)
                if p is None:
                    continue
                for vy in vys:
                    for r in rows:
                        pv = (r.y - (p - self.player.half) - y_bias) / y_scale
                        if abs(pv - vy) <= tol_px:
                            res.append(abs(((p - self.player.half) + y_scale * vy
                                            + y_bias) - r.y))
                            break
            if len(res) >= 4:
                med = sorted(res)[int(0.9 * (len(res) - 1))]
                n = len(res)
                if best is None or med < best[0]:
                    best = (med, d)
            d += step
        if best is None or n < 8:
            return self.delta_s
        #: 平局取**最小** Δ（奥卡姆）：残差一样好时不要凭空造延迟
        med_best, _ = best
        d_best = 0.0
        for d in [i * step for i in range(int(hi / step) + 1)]:
            res = []
            for (t, vys) in getattr(self, "_delay_buf_mov", self._delay_buf):
                p = pose_at(t - d)
                if p is None:
                    continue
                for vy in vys:
                    for r in rows:
                        pv = (r.y - (p - self.player.half) - y_bias) / y_scale
                        if abs(pv - vy) <= tol_px:
                            res.append(abs(((p - self.player.half) + y_scale * vy
                                            + y_bias) - r.y))
                            break
            if res and statistics.median(res) <= med_best + 0.5:
                d_best = d
                break
        med = med_best
        if med <= 8.0:
            self.delta_s = (1.0 - alpha) * self.delta_s + alpha * d_best
            self.delta_res = med
            self.delta_n = n
        return self.delta_s

    def rel_to_world_y(self, vy: float, y_scale: float, y_bias: float,
                       player_page_y: float | None = None,
                       py_world: float | None = None) -> float:
        """页面 vy -> 场地 y（**相对量**版：不依赖拍摄时刻的位姿）。"""
        half = self.player.half
        py_page = half if player_page_y is None else float(player_page_y)
        rel = float(vy) - py_page
        py_use = self.player.y if py_world is None else float(py_world)
        return ((py_use - half) + y_scale * (rel + half) + y_bias)

    def as_dict(self) -> dict:
        return {"player": self.player.as_dict(), "walls": self.walls.as_dict(),
                "steps": int(self.steps or 0), "acks": int(self.acks),
                "odom_moved": round(self.odom_moved, 1),
                "delta_s": round(float(self.delta_s), 3),
                "delta_res": self.delta_res, "delta_n": int(self.delta_n)}


# ----------------------------------------------------------------------
if __name__ == "__main__":                 # 板上/本机自检：python3 board_world.py <task.json>
    import sys
    task = load_task(sys.argv[1] if len(sys.argv) > 1 else "/root/vision_state/task.json")
    w = World(task)
    w.lock_now()
    print("lock:", w.px, w.py)
    w.on_step(0, 3)
    w.on_step(120)                          # 120 步 x 1.667 = 200 单位
    print("after 120 steps +y -> py=%.3f（应 %.3f）" % (w.py, task["spawn"][1] + 200.0))
    for i in range(4):                      # 同一道墙连喂 4 帧 -> 冻结
        print("  obs", i, w.observe_frame(i * 0.07, 300.0, 220.0, 320.0))
    print("frozen:", [s.as_dict() for s in w.walls.frozen()])
    print("world y of vy=300 ->", round(w.rel_to_world_y(300.0, 0.9587, 10.37), 1))
