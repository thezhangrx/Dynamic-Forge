"""Scenario builder: ``ScenarioSpec`` -> deterministic initial state + timeline.

Also home of the analytic bullet-count estimator used to *calibrate* a scene to
a requested number of simultaneously-live bullets ("one click, N bullets")
without ever running the simulation.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
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
    calibrate: bool = True,
    ensure_valid: bool = True,
    **spec_overrides: Any,
) -> ScenarioSpec:
    """Build a fully-specified scenario for a difficulty level or bullet count.

    ``complexity`` accepts ``'easy'``/``'medium'``/``'hard'``/``'extreme'``,
    any float in ``[0, 1]``, or an int bullet count.

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
              "obstacle_kinds": list(profile.obstacle_kinds)},
    )
    base.update(spec_overrides)
    spec = ScenarioSpec(**base)
    if calibrate and profile.bullet_count > 0:
        spec, peak = calibrate_bullet_count(spec, profile.bullet_count, return_peak=True)
        spec.meta["target_bullet_count"] = profile.bullet_count
        spec.meta["measured_peak_bullets"] = float(peak)
    else:
        peak = int(measure_live_bullets(spec)["peak"]) if calibrate else 0

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
    current = spec
    if est > 0:
        current = current.replaced(patterns=_scale_patterns(current.patterns, target / est))
    stats = measure_live_bullets(current)
    for _ in range(max_iterations):
        peak = stats["peak"]
        if peak == 0:
            return (current, 0) if return_peak else current
        ratio = target / peak
        if abs(ratio - 1.0) <= tolerance:
            break
        current = current.replaced(patterns=_scale_patterns(current.patterns, ratio))
        stats = measure_live_bullets(current)
    return (current, int(stats["peak"])) if return_peak else current


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
