"""网格势场方案的评测台 —— 在平台真实场景上测碰撞与延迟。

**这是评测代码，不是算法**（算法在 :mod:`cpu.potential_field`）。
单独放一个文件是为了让算法模块保持零平台耦合、可以脱离平台单独移植。

跑什么：
* ``run_policy``  —— 用某个策略在给定的 ``(难度, 种子)`` 上跑一整局，返回碰撞与延迟；
* ``compare``     —— 横向比"本方案 vs 原地不动（负面对照）"，并可扫参数。

**必须有负面对照。** 这个项目已经吃过一次亏：碰撞计数器读错字段时
``collision_events`` 恒为 0，于是"0 碰撞"这个结论**一直是假的**而且不报错
（见 ``core/harness/loop.py`` 的注释）。所以任何"无碰撞率"结论都要和
"站着不动"一起看 —— 如果连站着不动都报 0，那是计数器坏了，不是算法好。
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

from cpu.potential_field import (FieldConfig, FieldMemory,
                                 field_from_world, plan_field)

#: 场中央 —— 平台 ``target`` 实体的默认位置，也是"始终以中心为目标"那个想法。
CENTER = (320.0, 240.0)


@dataclass
class Episode:
    """一局的统计。"""

    events: int = 0                 # 碰撞**事件**数（进入接触的次数）
    overlap_frames: int = 0         # 重叠帧数（持续接触会一直累加）
    steps: int = 0
    ended_by: str = "steps"         # steps / dead
    lat_p50_ms: float = 0.0
    lat_p95_ms: float = 0.0
    cpu_seconds: float = 0.0
    recomputes: int = 0
    final: tuple[float, float] = (0.0, 0.0)
    diag: dict = field(default_factory=dict)


def run_policy(*, difficulty: float = 0.4, seed: int = 1,
               policy: str = "field", goal: Optional[tuple[float, float]] = CENTER,
               cfg: Optional[FieldConfig] = None,
               max_seconds: Optional[float] = None) -> Episode:
    """跑一局。``policy`` 取 ``"field"``（本方案）或 ``"hold"``（负面对照）。"""
    from bullet_sim.scenarios.builder import build_scenario      # noqa: PLC0415
    from bullet_sim.simulator.world import World                 # noqa: PLC0415

    built = build_scenario(difficulty, seed=seed)
    world = World(built.spec, codec="velocity")
    dt = float(built.spec.dt)
    horizon = float(max_seconds) if max_seconds else float(built.spec.duration)
    total = int(horizon / dt)

    mem = FieldMemory()
    conf = cfg or FieldConfig()
    lat: list[float] = []
    ep = Episode()

    for _ in range(total):
        if policy == "field":
            view = field_from_world(world, goal=goal, step_index=ep.steps, dt=dt)
            t0 = time.perf_counter()
            dec = plan_field(view, mem, conf)
            lat.append(time.perf_counter() - t0)
            speed = float(view.player[5]) if len(view.player) > 5 else 200.0
            world.step((dec.wx * speed, dec.wy * speed))
        else:
            world.step((0.0, 0.0))
        ep.steps += 1
        if not world.player_state().alive:
            ep.ended_by = "dead"
            break

    c = world.contacts
    ep.events = int(c.entered_total)
    ep.overlap_frames = int(c.overlap_frames)
    ep.recomputes = mem.recomputes
    p = world.player_state()
    ep.final = (float(p.x), float(p.y))
    if lat:
        lat.sort()
        ep.lat_p50_ms = statistics.median(lat) * 1e3
        ep.lat_p95_ms = lat[int(0.95 * (len(lat) - 1))] * 1e3
        ep.cpu_seconds = sum(lat)
    ep.diag = dict(mem.last_diag)
    return ep


@dataclass
class Tally:
    """多局汇总。"""

    label: str = ""
    events: int = 0
    overlap_frames: int = 0
    steps: int = 0
    lat_p50_ms: float = 0.0
    lat_p95_ms: float = 0.0
    cpu_seconds: float = 0.0

    def row(self) -> str:
        return (f"{self.label:>30} {self.events:>9} {self.overlap_frames:>10} "
                f"{self.steps:>7} {self.lat_p50_ms:>8.2f}ms {self.lat_p95_ms:>8.2f}ms "
                f"{self.cpu_seconds:>7.1f}s")


def tally(label: str, episodes: Sequence[Episode]) -> Tally:
    t = Tally(label=label)
    for e in episodes:
        t.events += e.events
        t.overlap_frames += e.overlap_frames
        t.steps += e.steps
        t.lat_p50_ms = max(t.lat_p50_ms, e.lat_p50_ms)
        t.lat_p95_ms = max(t.lat_p95_ms, e.lat_p95_ms)
        t.cpu_seconds += e.cpu_seconds
    return t


def compare(*, seeds: Sequence[int] = (1, 2, 3, 4), difficulty: float = 0.4,
            goals: Optional[dict[str, Optional[tuple[float, float]]]] = None,
            configs: Optional[dict[str, FieldConfig]] = None,
            weights: Sequence[float] = (1.0,),
            verbose: bool = True) -> list[Tally]:
    """横向比较。返回每一行的汇总（同时按 ``verbose`` 打印）。"""
    goals = goals if goals is not None else {"场中央": CENTER, "不用目标": None}
    configs = configs if configs is not None else {"默认": FieldConfig()}
    out: list[Tally] = []

    if verbose:
        print(f"难度 {difficulty}，{len(seeds)} 个种子，每局 {len(seeds)} 条轨迹")
        print(f"{'策略 / 目标 / 参数':>30} {'碰撞事件':>9} {'重叠帧':>10} "
              f"{'步数':>7} {'p50':>9} {'p95':>9} {'总CPU':>7}")
        print("-" * 90)

    for cname, cfg in configs.items():
        for gname, goal in goals.items():
            for lam in weights:
                c = FieldConfig(**{**cfg.__dict__, "travel_weight": lam})
                eps = [run_policy(difficulty=difficulty, seed=s, policy="field",
                                  goal=goal, cfg=c) for s in seeds]
                label = f"{gname} λ={lam:g} [{cname}]"
                t = tally(label, eps)
                out.append(t)
                if verbose:
                    print(t.row(), flush=True)

    eps = [run_policy(difficulty=difficulty, seed=s, policy="hold") for s in seeds]
    t = tally("原地不动（负面对照）", eps)
    out.append(t)
    if verbose:
        print(t.row(), flush=True)
    return out


__all__ = ["CENTER", "Episode", "Tally", "run_policy", "tally", "compare"]
