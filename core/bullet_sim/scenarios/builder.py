"""Scenario builder: ``ScenarioSpec`` -> deterministic initial state + timeline.

Also home of the analytic bullet-count estimator used to *calibrate* a scene to
a requested number of simultaneously-live bullets ("one click, N bullets")
without ever running the simulation.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from functools import lru_cache
from pathlib import Path
from typing import Any, Sequence

import numpy as np

import numpy as np

from bullet_sim.core.clock import FixedClock
from bullet_sim.core.rng import SeedManager
from bullet_sim.generators.burst import SpawnTimeline
from bullet_sim.generators.composite import build_timeline
from bullet_sim.generators.spec import PatternSpec
from bullet_sim.scenarios.complexity import (
    LEVEL_OBJECT_CAP,
    ComplexityProfile,
    level_object_count,
    profile_for,
    resolve_complexity,
    safety_count_cap,
)
from bullet_sim.scenarios.spec import ScenarioSpec, scenario_from

# --------------------------------------------------------------------------
# built scenario
# --------------------------------------------------------------------------


@dataclass
class BuiltScenario:
    """A scenario spec plus everything derived from it deterministically."""

    spec: ScenarioSpec
    timeline: SpawnTimeline
    profile: ComplexityProfile | None = None
    peak_bullets: int = 0
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def seed(self) -> int:
        return int(self.spec.seed)

    @property
    def total_steps(self) -> int:
        return self.spec.total_steps

    def summary(self) -> dict[str, Any]:
        s = self.timeline.summary()
        s.update(
            {
                "name": self.spec.name,
                "seed": self.spec.seed,
                "dt": self.spec.dt,
                "duration": self.spec.duration,
                "steps": self.total_steps,
                "field": [self.spec.field_w, self.spec.field_h],
                "patterns": self.spec.pattern_summary(),
                "estimated_peak_bullets": self.peak_bullets,
                "complexity": None if self.profile is None else self.profile.value,
                "label": None if self.profile is None else self.profile.label,
            }
        )
        return s

    def save(self, path: str | Path) -> Path:
        import json

        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "spec": self.spec.to_dict(),
            "summary": self.summary(),
            "timeline": self.timeline.to_list(),
        }
        p.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return p


# --------------------------------------------------------------------------
# analytic peak estimator
# --------------------------------------------------------------------------


def estimate_peak_bullets(timeline: SpawnTimeline, dt: float) -> int:
    """Peak number of simultaneously-live bullets implied by a timeline.

    A sweep over emission windows: each event contributes ``n`` bullets over
    ``[step, step + ttl/dt]``.  Boundary culling can only *lower* the real
    count, so this is a safe upper bound and an accurate calibration target.
    """
    deltas: list[tuple[int, int]] = []
    for ev in timeline.iter_events():
        if ev.n == 0:
            continue
        start = int(ev.step)
        finite = np.isfinite(ev.ttl)
        if finite.all():
            life = int(math.ceil(float(np.max(ev.ttl)) / dt))
        else:
            life = None
        deltas.append((start, ev.n))
        if life is not None:
            deltas.append((start + life, -ev.n))
        else:
            deltas.append((10 ** 12, -ev.n))
    if not deltas:
        return 0
    deltas.sort(key=lambda p: (p[0], p[1] >= 0))
    cur = 0
    peak = 0
    for _, change in deltas:
        cur += change
        peak = max(peak, cur)
    return int(peak)


# --------------------------------------------------------------------------
# staged obstacle introduction
# --------------------------------------------------------------------------

#: Order in which obstacle *kinds* appear when a scene is staged.
#:
#: The complexity ladder decides **which** kinds a scene may contain; staging
#: decides **when** each kind may first appear.  The wall with a gap comes first
#: on purpose: it is the only kind the real camera -> PL geometry -> decision
#: loop reads unambiguously, so a staged scene opens with exactly one obstacle
#: and the loop can be verified before a second kind is added.
STAGE_ORDER: tuple[str, ...] = (
    "wall_with_gap",
    "moving_block",
    "small_obstacles",
    "cross_traffic",
    "corridor",
)

#: ``PatternSpec.params['layout']`` -> obstacle kind name.
_LAYOUT_KINDS: dict[str, str] = {
    "wall_gap": "wall_with_gap",
    "block": "moving_block",
    "small": "small_obstacles",
    "cross": "cross_traffic",
    "corridor": "corridor",
}


def pattern_kind(pattern: PatternSpec) -> str | None:
    """Obstacle kind behind a generator spec, or ``None`` if it is not an obstacle."""
    if str(getattr(pattern, "kind", "")) != "obstacle":
        return None
    layout = str(pattern.params.get("layout", "")).lower()
    return _LAYOUT_KINDS.get(layout)


def kind_order(
    patterns: Sequence[PatternSpec], preferred: Sequence[str] | None = None
) -> list[str]:
    """障碍类型在时间轴上的先后顺序。

    ``preferred`` 是**调用方明确给的**顺序（例如用户 `--obstacle-type` 的书写顺序）：
    用户既然一个个列出来了，那个顺序就是他要的展示顺序，不能被我们按"可读性"重排。
    没给就按 :data:`STAGE_ORDER`（墙最先），剩下的未知类型排在最后。
    """
    present: list[str] = []
    for p in patterns:
        kind = pattern_kind(p)
        if kind is not None and kind not in present:
            present.append(kind)
    head = [k for k in (preferred or ()) if k in present]
    if not head:
        head = [k for k in STAGE_ORDER if k in present]
    tail = [k for k in STAGE_ORDER if k in present and k not in head]
    tail += [k for k in present if k not in head and k not in tail]
    return head + tail


def stage_offsets(
    patterns: Sequence[PatternSpec],
    stage_seconds: float,
    *,
    order: Sequence[str] | None = None,
) -> dict[str, float]:
    """First-appearance delay per obstacle kind, in the staged order."""
    ordered = kind_order(patterns, order)
    return {kind: i * float(stage_seconds) for i, kind in enumerate(ordered)}


def stage_patterns(
    patterns: Sequence[PatternSpec],
    stage_seconds: float,
    *,
    order: Sequence[str] | None = None,
) -> list[PatternSpec]:
    """Push every obstacle kind after the first to its own first-appearance time.

    ``stage_seconds <= 0`` returns the patterns untouched, so the historic
    "everything starts at t = 0" scene stays available and unchanged.
    """
    seconds = float(stage_seconds)
    if seconds <= 0.0 or not patterns:
        return list(patterns)
    offsets = stage_offsets(patterns, seconds, order=order)
    out: list[PatternSpec] = []
    for p in patterns:
        kind = pattern_kind(p)
        offset = 0.0 if kind is None else float(offsets.get(kind, 0.0))
        out.append(p if offset == 0.0 else p.replaced(start_time=float(p.start_time) + offset))
    return out


def stage_spec(
    spec: ScenarioSpec, stage_seconds: float, *, order: Sequence[str] | None = None
) -> ScenarioSpec:
    """Apply :func:`stage_patterns` to a whole scenario and record the schedule.

    The schedule lands in ``spec.meta['obstacle_stages']`` (kind + real first
    spawn time), so a report can state *when* each kind enters instead of only
    *which* kinds exist.

    注意这是"**叠加**"：第 2 种进场时第 1 种还在场上。要"同一时刻只有一种"
    用 :func:`cycle_spec`（障碍**分开展示**）。
    """
    seconds = float(stage_seconds)
    if seconds <= 0.0:
        return spec
    if len(kind_order(spec.patterns, order)) < 2:
        return spec  # 只有一种类型，分批没有意义（"第 2 种"不存在）
    patterns = stage_patterns(spec.patterns, seconds, order=order)
    first: dict[str, float] = {}
    for p in patterns:
        kind = pattern_kind(p)
        if kind is None:
            continue
        first[kind] = min(first.get(kind, float("inf")), float(p.start_time))
    meta = dict(spec.meta)
    meta["stage_seconds"] = seconds
    meta["obstacle_stages"] = [
        {"type": kind, "first_spawn_s": round(t, 6)}
        for kind, t in sorted(first.items(), key=lambda kv: (kv[1], kv[0]))
    ]
    return spec.replaced(patterns=patterns, meta=meta)


def cycle_period(patterns: Sequence[PatternSpec], slot_seconds: float,
                 *, order: Sequence[str] | None = None) -> float:
    """一个完整轮次有多长：类型数 × 每类的档期。"""
    return float(slot_seconds) * max(1, len(kind_order(patterns, order)))


def cycle_patterns(
    patterns: Sequence[PatternSpec],
    slot_seconds: float,
    *,
    order: Sequence[str] | None = None,
) -> list[PatternSpec]:
    """**分开展示**：每个障碍类型独占一段档期，整轮循环。

    与 :func:`stage_patterns` 的区别就在于"叠加"还是"轮播"：

    * 分批进场（stage）：10 秒后场上同时有墙和移动块；
    * 轮播（cycle）：每个类型只在自己的档期里出场，**同一时刻场上只有一种**
      （上一档的障碍正在离场属于物理自然现象），整轮 ``n × S`` 秒后重复。

    做法只用现有语义，不新增字段：``start_time = 档期起点``、
    ``interval = 整轮时长``、``repetitions = 0``（一直发到回合结束）。
    """
    seconds = float(slot_seconds)
    if seconds <= 0.0 or not patterns:
        return list(patterns)
    ordered = kind_order(patterns, order)
    if len(ordered) < 2:
        return list(patterns)  # 只有一种类型，轮播没有意义
    period = cycle_period(patterns, seconds, order=order)
    starts = {kind: i * seconds for i, kind in enumerate(ordered)}
    out: list[PatternSpec] = []
    for p in patterns:
        kind = pattern_kind(p)
        if kind is None:
            out.append(p)
            continue
        out.append(p.replaced(start_time=starts[kind], interval=period, repetitions=0))
    return out


def cycle_spec(
    spec: ScenarioSpec, slot_seconds: float, *, order: Sequence[str] | None = None
) -> ScenarioSpec:
    """Apply :func:`cycle_patterns` to a scenario and record the rotation."""
    seconds = float(slot_seconds)
    if seconds <= 0.0:
        return spec
    patterns = cycle_patterns(spec.patterns, seconds, order=order)
    ordered = kind_order(spec.patterns, order)
    if len(ordered) < 2:
        return spec
    period = cycle_period(spec.patterns, seconds, order=order)
    meta = dict(spec.meta)
    meta["cycle_seconds"] = seconds
    meta["cycle_period_s"] = period
    meta["obstacle_cycle"] = [
        {"type": kind, "slot_s": round(i * seconds, 6), "until_s": round((i + 1) * seconds, 6)}
        for i, kind in enumerate(ordered)
    ]
    return spec.replaced(patterns=patterns, meta=meta)


# --------------------------------------------------------------------------
# complexity -> scenario
# --------------------------------------------------------------------------


def template_patterns(
    profile: ComplexityProfile,
    field_w: float,
    field_h: float,
    *,
    player_radius: float = 10.0,
) -> list[PatternSpec]:
    """Concrete *obstacle* set for a complexity profile.

    ``complexity`` is the single intuitive master control: as it rises, every
    level gets **more obstacle types, more obstacles and faster obstacles**,
    all multiplicatively (``profile.count_scale`` / ``profile.speed_scale``).
    Counts are relative; :func:`calibrate_bullet_count` rescales them so the
    scene hits the requested live-object budget.

    Only realistic dynamic obstacles are emitted - the environment no longer
    fires decorative bullets from the field centre.
    """
    from bullet_sim.obstacles.catalog import get_obstacle_type
    from bullet_sim.obstacles.spec import ObstacleSpawn, resolve_spec

    v = float(profile.value)
    kinds = set(profile.obstacle_kinds)
    player_d = 2.0 * float(player_radius)
    ss = float(profile.speed_scale)
    cs = float(profile.count_scale)
    has_wall = "wall_with_gap" in kinds

    spawns: list[ObstacleSpawn] = []
    if "small_obstacles" in kinds:
        spawns.append(
            ObstacleSpawn(
                type_key="small_obstacles",
                count=max(4, int(round(18.0 * cs))),
                size=0.35,
                speed=(55.0 + 95.0 * v) * ss,
                interval=max(0.25, 1.1 - 0.7 * v),
                extra={"angle_jitter": 30.0 + 15.0 * v},
            )
        )

    # The wall is the slowest large obstacle; the block is deliberately much
    # faster and phase-shifted, and enters from a *perpendicular* edge, so the
    # block can never sit in the wall's opening at the moment the wall arrives.
    wall_speed = (44.0 + 52.0 * v) * ss
    wall_interval = max(1.5, 5.0 - 1.6 * v)
    if has_wall:
        spawns.append(
            ObstacleSpawn(
                type_key="wall_with_gap",
                count=1,
                speed=wall_speed,
                interval=wall_interval,
                gap_width=max(player_d * 1.6, player_d * (4.5 - 1.8 * v)),
                gap_motion="sweep",
                extra={"gap_jitter": 0.08, "gap_sweep": 0.38},
            )
        )

    if "moving_block" in kinds:
        spawns.append(
            ObstacleSpawn(
                type_key="moving_block",
                count=1,
                size=1.6 + 1.4 * v,
                speed=wall_speed * 1.55 if has_wall else (65.0 + 70.0 * v) * ss,
                interval=max(1.0, 3.6 - 1.4 * v),
                entry_span=0.35 + 0.4 * v,
                region="left" if has_wall else "top",
                start_time=(0.5 * wall_interval) if has_wall else 0.0,
            )
        )

    if "cross_traffic" in kinds:
        spawns.append(
            ObstacleSpawn(
                type_key="cross_traffic",
                count=max(3, int(round(8.0 * cs))),
                size=0.8,
                speed=(70.0 + 90.0 * v) * ss,
                interval=max(0.4, 1.3 - 0.5 * v),
            )
        )

    if "corridor" in kinds:
        # The walls span the whole field, so a *same-direction* tilt can be
        # large (the width never changes), while an *opposite* scissor is
        # limited by geometry: the gap at the wall ends is w - 2a*tan(theta).
        # Extreme therefore opens the channel wider so the scissor still reads.
        if v < 0.5:
            motion, change, width_ratio = "open", 1.5, 4.5 - 1.0 * v
        elif v < 0.85:
            motion, change, width_ratio = "rotate_same", 16.0, 4.5 - 1.0 * v
        else:
            motion, change, width_ratio = "rotate_opposite", 20.0, 8.0
        spawns.append(
            ObstacleSpawn(
                type_key="corridor",
                count=1,
                corridor_width=max(player_d * 1.6, player_d * width_ratio),
                walls=2,
                motion=motion,
                change=change,
                # wall_length omitted on purpose: the corridor extends across
                # the whole field, so it reads as a road, not as two blocks
                tilt_deg=0.0,
            )
        )

    patterns: list[PatternSpec] = []
    for i, spawn in enumerate(spawns):
        patterns.append(
            resolve_spec(
                get_obstacle_type(spawn.type_key),
                spawn,
                player_radius=player_radius,
                field_w=field_w,
                field_h=field_h,
                group_id=i,
            )
        )
    return patterns


def make_scenario(
    complexity: float | str = 0.4,
    *,
    seed: int = 0,
    duration: float | None = None,
    bullet_count: int | None = None,
    field_w: float = 640.0,
    field_h: float = 480.0,
    dt: float = 1.0 / 120.0,
    name: str | None = None,
    calibrate: bool | None = None,
    ensure_valid: bool = True,
    stage_seconds: float = 0.0,
    cycle_seconds: float = 0.0,
    kinds: Sequence[str] | None = None,
    **spec_overrides: Any,
) -> ScenarioSpec:
    """Build a fully-specified scenario for a difficulty level or bullet count.

    ``complexity`` accepts ``'easy'``/``'medium'``/``'hard'``/``'extreme'``,
    any float in ``[0, 1]``, or an int bullet count.

    ``stage_seconds > 0`` stages the obstacle kinds in time (see
    :func:`stage_spec`): the scene opens with one kind only and unlocks one more
    kind every ``stage_seconds``.  Staging happens **before** calibration and the
    safety check, so the measured peak and the safety report describe the scene
    that actually runs.

    ``cycle_seconds > 0`` **分开展示**（see :func:`cycle_spec`）: every kind gets a
    slot of its own and the rotation repeats, so only one kind is on the field at
    a time.  It replaces staging when both are given, and for a *level* scene it
    also turns ``calibrate`` off by default (a per-type demo wants each type's own
    counts, not the level's object budget) — an explicit bullet count
    (``--bullets N``) or ``calibrate=True`` still calibrates.

    ``kinds=(...)`` restricts **which** obstacle kinds the scene may contain,
    keeping the level's speed/count/interval scaling.  That is how the board
    path says "只放墙+缺口" (see ``cli.BOARD_KINDS``): the gap_avoid decision only
    has a meaning for a wall with an opening, and a second kind on the field only
    makes the picture harder to read.  Restricted scenes are demos too, so they
    skip the object-budget calibration unless a bullet count is asked for.

    Any further keyword is forwarded verbatim to :class:`ScenarioSpec`
    (``player_speed``, ``player_x``, ``wrap``, ``cull_margin``, ...).
    """
    explicit_count = bullet_count is not None or isinstance(complexity, int)
    if bullet_count is None and isinstance(complexity, int):
        bullet_count = int(complexity)
    value = resolve_complexity(complexity) if not isinstance(complexity, int) else 0.5
    #: The player is a circle; level scenes default to a readable size so the
    #: "obstacle size = N x my own diameter" statement stays meaningful.
    player_radius = float(spec_overrides.pop("player_radius", 10.0))
    count_cap: int | None = None
    if not explicit_count:
        # A *level* must always keep a safe region, so it uses the measured
        # safety-capped ladder instead of the raw 100..10000 bullet ladder.
        # An explicit `stress(N)` bypasses the cap on purpose: that is the
        # "does it still run" stress test.
        count_cap = LEVEL_OBJECT_CAP
        requested = level_object_count(value)
    else:
        requested = bullet_count
    profile = profile_for(
        value,
        duration=duration,
        bullet_count=requested,
    )
    if kinds is not None:
        # 只放指定的类型（例如"板子在环里时只放墙+缺口"）。难度曲线照旧决定速度 /
        # 数量 / 间隔，改的只是**有哪些类型**。
        wanted = tuple(str(k) for k in kinds)
        unknown = [k for k in wanted if k not in profile.obstacle_kinds]
        if unknown:
            raise ConfigError(
                f"kinds {unknown} are not in this level's vocabulary "
                f"{list(profile.obstacle_kinds)}")
        profile = replace(profile, obstacle_kinds=wanted)
    patterns = template_patterns(profile, field_w, field_h, player_radius=player_radius)
    base: dict[str, Any] = dict(
        name=name or f"{profile.label}_{profile.bullet_count}",
        seed=int(seed),
        duration=profile.duration,
        dt=dt,
        field_w=field_w,
        field_h=field_h,
        player_radius=player_radius,
        patterns=patterns,
        difficulty=profile.label,
        complexity=profile.value,
        meta={"complexity": profile.value, "label": profile.label,
              "target_bullet_count": profile.bullet_count,
              "count_scale": profile.count_scale,
              "speed_scale": profile.speed_scale,
              "safety_count_cap": count_cap,
              "obstacle_kinds": list(profile.obstacle_kinds),
              **({"kinds_override": list(profile.obstacle_kinds)} if kinds is not None else {})},
    )
    base.update(spec_overrides)
    spec = ScenarioSpec(**base)
    cycling = bool(cycle_seconds) and float(cycle_seconds) > 0.0
    if cycling:
        spec = cycle_spec(spec, float(cycle_seconds))
    elif stage_seconds and float(stage_seconds) > 0.0:
        spec = stage_spec(spec, float(stage_seconds))
    # 分开展示（轮播）/ 只放一种（kinds）都是**演示 / 逐项验收**模式：每种类型就用
    # 它自己的数量定义。按难度预算去标定会把某一档放大成一大团（例如 208 个小圆点
    # 同时涌进来），那样反而看不清这一种类型长什么样 —— 而这正是开这个开关的目的。
    # 但**显式点名要多少**（`--bullets N` / `stress(N)`）仍然优先：那是明确要求。
    if calibrate is None:
        demo = cycling or kinds is not None
        calibrate = not (demo and not explicit_count)
    if calibrate and profile.bullet_count > 0:
        spec, peak = calibrate_bullet_count(spec, profile.bullet_count, return_peak=True)
        spec.meta["target_bullet_count"] = profile.bullet_count
        spec.meta["measured_peak_bullets"] = float(peak)
    else:
        # 不标定也要量一次：这个数既写进 meta（不假装达到了难度预算），
        # 也决定障碍池容量（容量给小了会静默丢障碍）。
        peak = int(measure_live_bullets(spec)["peak"])
        spec.meta["measured_peak_bullets"] = float(peak)

    # A *level* must always keep a safe region ("extreme 也要有安全区"), so the
    # built scene is measured and thinned until it passes.  Rescaling the already
    # calibrated patterns is cheap - no re-measurement, just one validation.
    if ensure_valid and not explicit_count:
        spec, accepted, report, attempts = _thin_until_safe(spec)
        # The *requested* ladder stays untouched (the level is the master
        # control); the safety-thinned value is recorded separately.
        spec.meta["achieved_bullet_target"] = accepted
        spec.meta["level_reduced_by"] = attempts
        spec.meta["safety"] = report.to_dict() if report is not None else None

    spec.bullet_capacity = max(64, int(peak * 1.25) + 64)
    return spec


def _level_safety_config(duration: float):
    from bullet_sim.safety.validate import SafetyConfig

    return SafetyConfig(
        horizon=min(2.0, max(0.5, float(duration))),
        sample_dt=0.12,
        resolution=28.0,
        max_window=2.0,
        # a safe region must be *left over*, not merely "a path exists"
        min_free_fraction=0.15,
        min_reachable_cells=12,
    )


def _thin_until_safe(spec: ScenarioSpec, *, max_attempts: int = 5, factor: float = 0.6):
    """Measure a level scene and thin it until a safe region remains.

    Returns ``(spec, accepted_target, report, reductions)``.  The last report is
    kept even when it never becomes feasible, so the caller can document the
    honest outcome instead of pretending the level is safe.
    """
    from bullet_sim.safety.validate import validate_scenario
    from bullet_sim.simulator.world import World

    config = _level_safety_config(spec.duration)
    target = int(spec.meta.get("target_bullet_count", 0))
    report = None
    reductions = 0
    for _ in range(max(1, int(max_attempts))):
        world = World(spec, reward="zero", terminate_on_collision=False)
        report = validate_scenario(world, config=config)
        if report.feasible:
            return spec, target, report, reductions
        new_target = max(20, int(round(target * factor)))
        if new_target >= target:
            break
        ratio = new_target / max(target, 1)
        spec = spec.replaced(patterns=_scale_patterns(spec.patterns, ratio))
        target = new_target
        reductions += 1
    return spec, target, report, reductions


def calibrate_bullet_count(
    spec: ScenarioSpec,
    target: int,
    *,
    max_iterations: int = 4,
    tolerance: float = 0.08,
    return_peak: bool = False,
):
    """Rescale pattern counts so the measured live-bullet peak reaches ``target``.

    The analytic estimate is only an upper bound (it ignores boundary culling),
    so calibration *measures* the scene with the real spawn/motion/lifecycle
    code path but no player and no collision detection.

    **收敛不了就停，不许指数级放大。** 缩放的是每种模式的 ``count``，可有些
    布局对 ``count`` 不敏感（``wall_gap`` 一次就是一道墙），再加上分批进场会把
    "数量敏感"的类型推到回合之外（例如 `--duration 0.5` + `--stage-seconds 8`：
    0.5 秒里只有墙）—— 这时"实测峰值"永远不动，而 ``ratio = target / peak``
    每一轮都把数量乘一遍，4 轮就能堆到 10^8 个障碍，光建时间线就卡死（实测过：
    180 秒超时）。所以每轮之后比较峰值：**不涨就带着当前最好的结果收工**，
    真实达到的峰值照实记在 ``measured_peak_bullets`` 里。
    """
    if target <= 0:
        return (spec, 0) if return_peak else spec
    # Cheap analytic pre-scale (upper bound) so measurement starts near target.
    pre = build_timeline(
        spec.patterns,
        field_w=spec.field_w,
        field_h=spec.field_h,
        dt=spec.dt,
        duration=spec.duration,
        seeds=SeedManager(spec.seed),
        player_xy=np.array(spec.player_spawn),
        clock=FixedClock.make(dt=spec.dt),
    )
    est = estimate_peak_bullets(pre, spec.dt)
    best = spec
    if est > 0:
        best = best.replaced(patterns=_scale_patterns(best.patterns, target / est))
    best_peak = measure_live_bullets(best)["peak"]
    if best_peak <= 0:
        return (best, 0) if return_peak else best
    for _ in range(max_iterations):
        ratio = target / best_peak
        if abs(ratio - 1.0) <= tolerance:
            break
        candidate = best.replaced(patterns=_scale_patterns(best.patterns, ratio))
        peak = measure_live_bullets(candidate)["peak"]
        if abs(float(peak) - target) >= abs(float(best_peak) - target):
            # 不再更接近目标（最坏情况：峰值**完全不动**，因为场上只剩对
            # `count` 不敏感的墙，或者数量敏感的类型被分批推到了回合之外）——
            # 此时再乘下去只是每轮翻倍地堆数字，4 轮就能到 10^8 个障碍，光建
            # 时间线就卡死（实测过：180 秒超时）。收工，照实返回达到的峰值。
            break
        best, best_peak = candidate, peak
    return (best, int(best_peak)) if return_peak else best


def _scale_patterns(patterns: Sequence[PatternSpec], ratio: float) -> list[PatternSpec]:
    out: list[PatternSpec] = []
    for p in patterns:
        n = int(round(p.resolved_count * ratio))
        out.append(p.replaced(count=max(1, n)))
    return out


@lru_cache(maxsize=64)
def _measure_cached(spec_json: str, sample_every: int, action_code: int) -> tuple:
    """Pure cache around the measurement (keyed by the scenario JSON)."""
    spec = ScenarioSpec.from_json(spec_json)
    result = _measure_live_bullets_uncached(
        spec, sample_every=sample_every, action=action_code
    )
    return tuple(sorted(result.items()))


def measure_live_bullets(
    spec: ScenarioSpec, *, sample_every: int = 1, action: Any = 0
) -> dict[str, float]:
    """Cached, deterministic measurement (see :func:`_measure_live_bullets_uncached`).

    Measuring a 10 000-bullet scene costs ~20 s, and the same scenario is
    measured repeatedly while calibrating and again by the benchmark suites, so
    results are memoised on the scenario JSON.  Only integer (discrete) actions
    are cached; anything else is measured directly.
    """
    if isinstance(action, (int, np.integer)) and not isinstance(action, bool):
        return dict(_measure_cached(spec.to_json(), int(sample_every), int(action)))
    return _measure_live_bullets_uncached(spec, sample_every=sample_every, action=action)


def _measure_live_bullets_uncached(
    spec: ScenarioSpec, *, sample_every: int = 1, action: Any = 0
) -> dict[str, float]:
    """Measure the *actual* live-bullet profile of a scenario.

    This runs the real :class:`~bullet_sim.simulator.world.World` with collision
    and rewards disabled, because it must reproduce what a simulation run will
    see.  In particular ``aimed`` patterns resolve against the **live** player
    position; a cheaper spawn-only reconstruction would resolve them against a
    frozen spawn point and therefore mis-measure every aiming scene by up to
    ~20% (which is exactly the kind of silent calibration error this function
    exists to avoid).

    Returns ``peak``, ``mean``, ``final`` and ``emitted`` counts.
    """
    from bullet_sim.simulator.world import World  # local: avoids an import cycle

    # Budget must not truncate the scene while we are measuring it, so allow a
    # generous capacity derived from the analytic upper bound.
    working = spec
    cap = int(spec.bullet_capacity or 0)
    need = max(256, int(estimate_peak_bullets(
        build_timeline(
            spec.patterns,
            field_w=spec.field_w,
            field_h=spec.field_h,
            dt=spec.dt,
            duration=spec.duration,
            seeds=SeedManager(spec.seed),
            player_xy=np.array(spec.player_spawn),
            clock=FixedClock.make(dt=spec.dt),
        ),
        spec.dt,
    ) * 2) + 256)
    if cap < need:
        working = spec.replaced(bullet_capacity=need)

    world = World(
        working,
        collision="null",
        reward="zero",
        terminate_on_collision=False,
        clamp_player=True,
    )
    peak = 0
    total = 0
    samples = 0
    emitted = 0

    def sample(step: int) -> None:
        nonlocal peak, total, samples
        if step % sample_every:
            return
        c = world.active_bullet_count()
        peak = max(peak, c)
        total += c
        samples += 1

    sample(0)
    for step in range(working.total_steps):
        world.step(action)
        sample(step + 1)
    emitted = world.timeline.total_bullets
    return {
        "peak": float(peak),
        "mean": float(total / max(samples, 1)),
        "final": float(world.active_bullet_count()),
        "emitted": float(emitted),
    }


# --------------------------------------------------------------------------
# entry points
# --------------------------------------------------------------------------


def build_scenario(source: Any, *, seed: int | None = None, **overrides: Any) -> BuiltScenario:
    """Build a :class:`BuiltScenario` from a spec/dict/path or a difficulty."""
    profile: ComplexityProfile | None = None
    if isinstance(source, (int, float, str)) and not Path(str(source)).exists():
        spec = make_scenario(source, seed=seed or 0, **overrides)
        profile = profile_for(spec.meta.get("complexity", 0.5))
    elif isinstance(source, str) and Path(source).exists():
        spec = ScenarioSpec.load(source)
        profile = _profile_from_meta(spec)
    else:
        spec = scenario_from(source)
        profile = _profile_from_meta(spec)
    if seed is not None:
        spec = spec.replaced(seed=int(seed))
    return build_from_spec(spec, profile=profile)


def _profile_from_meta(spec: ScenarioSpec) -> ComplexityProfile | None:
    if "complexity" not in spec.meta:
        return None
    return profile_for(
        float(spec.meta["complexity"]),
        duration=spec.duration,
        bullet_count=spec.meta.get("target_bullet_count"),
    )


def build_from_spec(
    spec: ScenarioSpec, profile: ComplexityProfile | None = None
) -> BuiltScenario:
    """Materialise a spec into a timeline (the deterministic scene recipe)."""
    seeds = SeedManager(spec.seed)
    timeline = build_timeline(
        spec.patterns,
        field_w=spec.field_w,
        field_h=spec.field_h,
        dt=spec.dt,
        duration=spec.duration,
        seeds=seeds,
        player_xy=np.array(spec.player_spawn),
        clock=FixedClock.make(dt=spec.dt),
    )
    peak = estimate_peak_bullets(timeline, spec.dt)
    return BuiltScenario(spec=spec, timeline=timeline, profile=profile, peak_bullets=peak)


def build_many(
    count: int,
    complexity: float | str = 0.4,
    *,
    base_seed: int = 0,
    seed_stride: int = 1000003,
    **kwargs: Any,
) -> list[BuiltScenario]:
    """Build ``count`` distinct-but-reproducible scenarios.

    Seed ``base_seed + i * seed_stride`` is deterministic, so a batch dataset is
    reproducible from ``(base_seed, count, complexity)`` alone.
    """
    out: list[BuiltScenario] = []
    for i in range(int(count)):
        seed = int(base_seed) + i * int(seed_stride)
        spec = make_scenario(complexity, seed=seed, **kwargs)
        spec = spec.replaced(name=f"{spec.name}_{i:04d}")
        out.append(build_from_spec(spec, profile=_profile_from_meta(spec)))
    return out
