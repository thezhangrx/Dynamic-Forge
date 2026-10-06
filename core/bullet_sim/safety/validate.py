"""Scenario safety validation: **有挑战 ≠ 必死**.

Public API (requirement 18)::

    is_scenario_valid(...)        -> bool
    find_safe_path(...)           -> SafePath | None
    compute_safe_region(...)      -> SafeRegion
    predict_collision(...)        -> dict
    validate_scenario(...)        -> SafetyReport
    generate_valid_scenario(...)  -> (ObstacleScenario, SafetyReport, attempts)

The generator -> free space -> path search -> accept/resample loop lives here and
is kept **decoupled from the scenario generator**: it consumes a built scenario
(or a plain spec) and returns a verdict plus the numbers behind it.

What "valid" means
------------------
There exists a sequence of positions such that, sampled over the horizon:

* every position is outside the obstacle field inflated by
  ``player_circumradius + safety_margin``;
* every consecutive displacement is achievable within ``max_speed`` (and
  ``max_accel`` when configured);
* the sequence starts at the player's actual spawn position.

That is *geometry + dynamics + time*, not an overlap test on the current frame.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from bullet_sim.collision.obstacles import player_circumradius
from bullet_sim.collision.shapes import make_hitbox
from bullet_sim.core.errors import ConfigError
from bullet_sim.obstacles.catalog import CATALOG
from bullet_sim.obstacles.scenario import ObstacleScenario
from bullet_sim.safety.free_space import (
    FreeSpaceGrid,
    FreeSpaceSequence,
    rasterize_free_space,
)
from bullet_sim.safety.path_search import ReachabilityResult, SafePath, backtrack_path, reachable_set

#: Per-type safety strategies (requirement 11).  Each type names the check that
#: actually matters for it; the generic path search is always available.
STRATEGY_NOTES: dict[str, str] = {
    "bypass": "检查移动大物体两侧是否留有足够的绕行空间",
    "gap": "检查缺口宽度 > 玩家碰撞宽度，且玩家能在墙到达前抵达缺口",
    "local_free": "检查局部自由空间是否形成足够大的连通区域",
    "corridor": "检查通道宽度与未来是否会被堵塞",
    "generic": "通用：时间窗自由空间 + 可行路径搜索",
}


@dataclass
class SafetyConfig:
    """Knobs of the feasibility check."""

    #: Extra clearance beyond the player hitbox (world units).
    margin: float = 6.0
    #: How far ahead feasibility must hold (seconds).
    horizon: float = 2.0
    #: Grid resolution; ``None`` picks ``max(8, player_radius/1.5)``.
    resolution: float | None = None
    #: Free-space sample interval (seconds).
    sample_dt: float = 0.05
    #: Override the player's speed limit (defaults to the scenario's).
    max_speed: float | None = None
    max_accel: float | None = None
    #: Minimum fraction of the field that must stay free (guards "no room at all").
    min_free_fraction: float = 0.03
    #: Minimum reachable cells at the horizon.
    min_reachable_cells: int = 4
    #: Seconds of the episode that must be survivable.  ``None`` = the whole
    #: episode (capped by ``max_window``).
    validation_window: float | None = None
    max_window: float = 8.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "margin": self.margin,
            "horizon": self.horizon,
            "resolution": self.resolution,
            "sample_dt": self.sample_dt,
            "max_speed": self.max_speed,
            "max_accel": self.max_accel,
            "min_free_fraction": self.min_free_fraction,
            "min_reachable_cells": self.min_reachable_cells,
            "validation_window": self.validation_window,
            "max_window": self.max_window,
        }


@dataclass
class SafeRegion:
    """Where the player can be, now and within the window."""

    grid: FreeSpaceGrid | None = None
    sequence: FreeSpaceSequence | None = None
    reach: ReachabilityResult | None = None

    @property
    def free_now(self) -> np.ndarray:
        return self.grid.free if self.grid is not None else np.zeros((0, 0), dtype=bool)

    @property
    def reachable(self) -> np.ndarray:
        if self.reach is None or self.reach.reachable_anytime is None:
            return np.zeros((0, 0), dtype=bool)
        return self.reach.reachable_anytime

    @property
    def always_free(self) -> np.ndarray:
        return self.sequence.intersection() if self.sequence is not None else self.free_now

    @property
    def free_fraction(self) -> float:
        return self.grid.free_fraction if self.grid is not None else 0.0

    @property
    def reachable_fraction(self) -> float:
        return self.reach.reachable_fraction if self.reach is not None else 0.0

    def safe_points(self, *, limit: int = 64) -> list[tuple[float, float]]:
        """World coordinates of reachable safe cells (for planners / datasets)."""
        if self.grid is None or self.reach is None:
            return []
        mask = self.reachable & self.always_free
        rr, cc = np.nonzero(mask)
        out = [self.grid.cell_to_world(int(r), int(c)) for r, c in zip(rr[:limit], cc[:limit])]
        return out

    def to_dict(self) -> dict[str, Any]:
        return {
            "grid": self.grid.describe() if self.grid else None,
            "sequence": self.sequence.describe() if self.sequence else None,
            "reach": self.reach.to_dict() if self.reach else None,
            "free_fraction": round(self.free_fraction, 4),
            "reachable_fraction": round(self.reachable_fraction, 4),
        }


@dataclass
class SafetyReport:
    """Verdict + the numbers behind it."""

    feasible: bool = False
    reason: str = ""
    strategy: str = "generic"
    strategy_note: str = ""
    config: SafetyConfig = field(default_factory=SafetyConfig)
    free_fraction: float = 0.0
    min_free_fraction: float = 0.0
    reachable_cells: int = 0
    reachable_fraction: float = 0.0
    min_clearance: float = 0.0
    path_length: int = 0
    path_displacement: float = 0.0
    horizon: float = 0.0
    samples: int = 0
    time_to_first_collision: float | None = None
    inflated_by: float = 0.0
    obstacle_count: int = 0
    suggestions: list[str] = field(default_factory=list)
    attempts: int = 1
    #: Measured difficulty of this concrete scene (0..1) and its components.
    #: Computed from what the scene *is* (free space, reachable set, clearance,
    #: relative speed), never from a user-picked label.
    complexity: float = 0.0
    complexity_label: str = ""
    complexity_terms: dict[str, float] = field(default_factory=dict)
    complexity_advice: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "feasible": self.feasible,
            "reason": self.reason,
            "strategy": self.strategy,
            "strategy_note": self.strategy_note,
            "config": self.config.to_dict(),
            "free_fraction": round(self.free_fraction, 4),
            "min_free_fraction": self.min_free_fraction,
            "reachable_cells": self.reachable_cells,
            "reachable_fraction": round(self.reachable_fraction, 4),
            "min_clearance": round(self.min_clearance, 3),
            "path_length": self.path_length,
            "path_displacement": round(self.path_displacement, 3),
            "horizon": round(self.horizon, 3),
            "samples": self.samples,
            "time_to_first_collision": self.time_to_first_collision,
            "inflated_by": round(self.inflated_by, 3),
            "obstacle_count": self.obstacle_count,
            "suggestions": list(self.suggestions),
            "attempts": self.attempts,
            "complexity": round(self.complexity, 4),
            "complexity_label": self.complexity_label,
            "complexity_terms": {k: round(v, 4) for k, v in self.complexity_terms.items()},
            "complexity_advice": list(self.complexity_advice),
        }

    def format(self) -> str:
        head = "可行 (feasible)" if self.feasible else "不可行 (INFEASIBLE)"
        lines = [
            f"场景安全校验: {head}",
            f"  策略      : {self.strategy} - {self.strategy_note}",
            f"  原因      : {self.reason}",
            f"  自由空间  : {self.free_fraction:.1%} (下限 {self.min_free_fraction:.1%})",
            f"  可达空间  : {self.reachable_cells} 格 / {self.reachable_fraction:.1%}",
            f"  最小余量  : {self.min_clearance:.1f} 世界单位（膨胀 {self.inflated_by:.1f}）",
            f"  预测时窗  : {self.horizon:.2f}s / {self.samples} 采样",
            f"  障碍数量  : {self.obstacle_count}",
        ]
        if self.complexity_label or self.complexity:
            terms = " ".join(f"{k}={v:.2f}" for k, v in self.complexity_terms.items())
            lines.append(
                f"  实测难度  : {self.complexity:.3f} ({self.complexity_label})  [{terms}]"
            )
        if self.time_to_first_collision is not None:
            lines.append(f"  首次碰撞  : {self.time_to_first_collision:.3f}s（静止直行）")
        if self.suggestions:
            lines.append("  建议调整  :")
            lines.extend(f"    - {s}" for s in self.suggestions)
        return "\n".join(lines)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _inflate_for(spec: Any, config: SafetyConfig) -> float:
    hitbox = make_hitbox(None, radius=float(spec.player_radius))
    return player_circumradius(hitbox) + float(config.margin)


def _resolution_for(spec: Any, config: SafetyConfig) -> float:
    if config.resolution is not None:
        return max(4.0, float(config.resolution))
    return max(8.0, float(spec.player_radius) / 1.5)


def free_space_sequence(
    world: Any,
    config: SafetyConfig | None = None,
    *,
    spec: Any = None,
) -> FreeSpaceSequence:
    """Level-2 analysis: free space over the horizon, driven by real physics.

    The obstacle field is advanced with an actual ``World`` clone (action
    ``stay``), so dynamic layouts are evaluated against the same motion the
    simulator will run - no duplicated dynamics.
    """
    config = config or SafetyConfig()
    spec = spec if spec is not None else world.spec
    inflate = _inflate_for(spec, config)
    resolution = _resolution_for(spec, config)
    n_samples = max(1, int(np.ceil(float(config.horizon) / max(config.sample_dt, 1e-6))))
    steps_per_sample = max(1, int(round(config.sample_dt / spec.dt)))

    roll = world.clone()
    grids: list[FreeSpaceGrid] = []
    times: list[float] = []
    for k in range(n_samples + 1):
        grids.append(
            rasterize_free_space(
                roll.state.bullets,
                field_w=spec.field_w, field_h=spec.field_h,
                inflate=inflate, resolution=resolution,
            )
        )
        times.append(k * config.sample_dt)
        for _ in range(steps_per_sample):
            roll.step(0)
    return FreeSpaceSequence(grids=grids, times=times, dt=config.sample_dt)


def compute_safe_region(
    world: Any,
    config: SafetyConfig | None = None,
    *,
    spec: Any = None,
    with_path: bool = True,
) -> SafeRegion:
    """Free space now, over the horizon, and what the player can actually reach."""
    config = config or SafetyConfig()
    world = _as_world(world, spec=spec)
    spec = spec if spec is not None else world.spec
    seq = free_space_sequence(world, config, spec=spec)
    grid = seq.grids[0]
    speed = float(config.max_speed if config.max_speed is not None else spec.player_speed)
    start = grid.world_to_cell(
        float(world.player[0]) if not hasattr(world, "player_state") else world.player_state().x,
        float(world.player[1]) if not hasattr(world, "player_state") else world.player_state().y,
    )
    reach = reachable_set(
        [g.free for g in seq.grids[1:]] or [grid.free],
        start_cell=start,
        dt=config.sample_dt,
        max_speed=speed,
        cell_w=grid.cell_w,
        cell_h=grid.cell_h,
        max_accel=config.max_accel,
    )
    reach.strategy = "generic"
    return SafeRegion(grid=grid, sequence=seq, reach=reach)


def find_safe_path(
    world: Any,
    config: SafetyConfig | None = None,
    *,
    spec: Any = None,
    target_cell: tuple[int, int] | None = None,
) -> SafePath | None:
    """A concrete collision-free trajectory over the horizon, or ``None``."""
    world = _as_world(world, spec=spec)
    region = compute_safe_region(world, config, spec=spec)
    if region.reach is None or not region.reach.feasible:
        return None
    layers = [g.free for g in region.sequence.grids[1:]] if region.sequence else []
    return backtrack_path(
        region.reach, layers, dt=config.sample_dt if config else 0.05,
        target_cell=target_cell, grid=region.grid,
    )


def predict_collision(
    world: Any,
    action: Any = 0,
    horizon: float = 2.0,
    *,
    stride: int = 1,
) -> dict[str, Any]:
    """When would a collision happen if ``action`` is held from here?

    Uses a real rollout on a clone, so it obeys the actual obstacle dynamics.
    Also reports the analytic constant-velocity estimate for comparison - the
    cheap number a planner can afford every frame.
    """
    from bullet_sim.physics.relative import (
        closing_speed,
        obstacle_positions,
        obstacle_velocities,
        relative_velocity,
        time_to_impact,
        to_player_frame,
    )

    world = _as_world(world)
    snapshot = world.get_state()
    frame = to_player_frame(snapshot)
    idx = frame.bullets.active_indices()
    analytic: float | None = None
    closing = 0.0
    if idx.size:
        pos = obstacle_positions(frame.bullets, idx)
        vel = obstacle_velocities(frame.bullets, idx)
        me = np.array([frame.player.x, frame.player.y])
        tti = time_to_impact(pos, vel, me, radius=float(frame.player.radius))
        finite = tti[np.isfinite(tti)]
        analytic = float(finite.min()) if finite.size else None
        closing = float(np.max(closing_speed(pos, vel, me)))

    roll = world.clone()
    steps = int(np.ceil(horizon / world.spec.dt))
    first: float | None = None
    for i in range(steps):
        res = roll.step(action)
        if res.info.get("collision_event"):
            first = (i + 1) * world.spec.dt
            break
    return {
        "time_to_collision": first,
        "collides_within_horizon": first is not None,
        "horizon": float(horizon),
        "analytic_time_to_impact": analytic,
        "max_closing_speed": closing,
        "action": action if isinstance(action, (int, float)) else str(action),
    }


# --------------------------------------------------------------------------
# the main verdict
# --------------------------------------------------------------------------


def episode_reachability(
    world: Any,
    config: SafetyConfig | None = None,
    *,
    spec: Any = None,
    max_samples: int = 120,
) -> tuple[ReachabilityResult, FreeSpaceSequence]:
    """One-pass, whole-episode reachability from ``S_0``.

    This is the correct feasibility question: *does a survivable trajectory
    exist for the episode*, not *is the player's spawn point still free*.
    The reachable set evolves together with the obstacle field, so a wall
    sweeping across the spawn point is fine as long as the player could have
    been somewhere else by then - which is exactly the situation the planner has
    to deal with.
    """
    config = config or SafetyConfig()
    spec = spec if spec is not None else world.spec
    inflate = _inflate_for(spec, config)
    resolution = _resolution_for(spec, config)
    dt = float(spec.dt)
    total = max(1, int(spec.total_steps))
    samples = int(min(max(2, max_samples), max(2, int(np.ceil(config.horizon / config.sample_dt)))))
    # Resolution of the *time* axis: never coarser than needed to resolve one
    # grid cell of obstacle motion, never finer than the configured sample_dt.
    window = (
        float(config.validation_window)
        if config.validation_window is not None
        else min(total * dt, float(config.max_window))
    )
    horizon = min(window, total * dt)
    sample_dt = max(horizon / samples, config.sample_dt * 0.5)
    steps_per_sample = max(1, int(round(sample_dt / dt)))

    roll = world.clone()
    grids: list[FreeSpaceGrid] = []
    times: list[float] = []
    peak_obstacles = 0
    peak_relative_speed = 0.0
    t = 0.0
    for k in range(samples + 1):
        pool = roll.state.bullets
        peak_obstacles = max(peak_obstacles, int(pool.count))
        #: Measured relative motion ``|v_obstacle - v_player|`` at this instant.
        #: This is where the difficulty score's speed term comes from - it is
        #: measured, never assumed from the requested parameters.
        idx = pool.active_indices()
        if idx.size:
            d = pool.data
            px = float(roll.state.player.vx)
            py = float(roll.state.player.vy)
            mag = np.hypot(
                np.asarray(d["vx"][idx], dtype=np.float64) - px,
                np.asarray(d["vy"][idx], dtype=np.float64) - py,
            )
            peak_relative_speed = max(peak_relative_speed, float(mag.max()))
        grids.append(
            rasterize_free_space(
                roll.state.bullets,
                field_w=spec.field_w, field_h=spec.field_h,
                inflate=inflate, resolution=resolution,
            )
        )
        times.append(t)
        if k == samples:
            break
        for _ in range(steps_per_sample):
            if roll.state.env.step_index >= spec.total_steps:
                break
            roll.step(0)
        t = roll.state.env.step_index * dt

    seq = FreeSpaceSequence(grids=grids, times=times, dt=sample_dt)
    grid = grids[0]
    speed = float(config.max_speed if config.max_speed is not None else spec.player_speed)
    start_cell = grid.world_to_cell(float(world.player[0]), float(world.player[1]))
    accel = config.max_accel
    if accel == 0.0:
        accel = None
    reach = reachable_set(
        [g.free for g in grids[1:]] or [grid.free],
        start_cell=start_cell,
        dt=sample_dt,
        max_speed=speed,
        cell_w=grid.cell_w,
        cell_h=grid.cell_h,
        max_accel=accel,
    )
    reach.peak_obstacles = peak_obstacles  # type: ignore[attr-defined]
    reach.peak_relative_speed = peak_relative_speed  # type: ignore[attr-defined]
    return reach, seq


def validate_scenario(
    built_or_world: Any,
    *,
    spec: Any = None,
    config: SafetyConfig | None = None,
    strategy: str = "generic",
    attempts: int = 1,
    checkpoints: int | None = None,
    max_samples: int = 120,
) -> SafetyReport:
    """Decide whether a scenario has at least one feasible safe path.

    Feasibility = *there exists a trajectory that survives the whole validated
    window*: every sampled instant is outside the inflated obstacle field, and
    every displacement is within ``max_speed`` / ``max_accel``.  Nothing is
    accepted merely because the scene happens to be empty at ``t = 0``.
    """
    world = _as_world(built_or_world, spec=spec)
    spec = spec if spec is not None else world.spec
    config = config or SafetyConfig(
        margin=float(spec.safety_margin), horizon=float(spec.safety_horizon)
    )
    inflate = _inflate_for(spec, config)
    reach, seq = episode_reachability(world, config, spec=spec, max_samples=max_samples)
    grid = seq.grids[0]
    profile = seq.free_fraction_profile()
    peak_obstacles = int(getattr(reach, "peak_obstacles", 0) or world.state.bullets.count)

    report = SafetyReport(
        strategy=strategy,
        strategy_note=STRATEGY_NOTES.get(strategy, STRATEGY_NOTES["generic"]),
        config=config,
        free_fraction=min(profile) if profile else 0.0,
        min_free_fraction=config.min_free_fraction,
        inflated_by=inflate,
        obstacle_count=peak_obstacles,
        horizon=seq.times[-1] if seq.times else 0.0,
        samples=len(seq.grids),
        attempts=attempts,
        reachable_cells=reach.reachable_cells,
        reachable_fraction=reach.reachable_fraction,
    )
    relative_speed = float(getattr(reach, "peak_relative_speed", 0.0) or 0.0)

    def finish() -> SafetyReport:
        _attach_complexity(
            report,
            spec=spec,
            config=config,
            relative_speed=relative_speed,
            spawn_distance_hint=min(float(spec.field_w), float(spec.field_h)),
        )
        return report

    # -- verdict -------------------------------------------------------
    if not reach.start_free:
        report.reason = "玩家出生点已被膨胀后的障碍覆盖（生成即碰撞）"
        report.suggestions = ["把玩家出生点移到自由空间，或减小 safety_margin"]
        return finish()
    if min(profile) < config.min_free_fraction:
        report.reason = (
            f"自由空间最低仅 {min(profile):.1%}，低于下限 "
            f"{config.min_free_fraction:.1%}"
        )
        report.suggestions = _suggestions_for(spec, "free", report)
        return finish()
    if not reach.feasible or reach.reachable_cells < config.min_reachable_cells:
        report.reason = reach.reason or "可达自由空间不足"
        report.suggestions = _suggestions_for(spec, "reach", report)
        return finish()

    layers = [g.free for g in seq.grids[1:]] or [grid.free]
    path = backtrack_path(
        reach, layers, dt=seq.dt or config.sample_dt, grid=grid
    )
    if path is None:
        report.reason = "存在自由空间但找不到满足运动约束的连续路径"
        report.suggestions = _suggestions_for(spec, "path", report)
        return finish()

    report.feasible = True
    report.reason = (
        f"验证窗口 {report.horizon:.2f}s（{report.samples} 个采样）内存在可行安全路径"
    )
    report.path_length = path.length
    report.path_displacement = path.displacement()
    report.min_clearance = path.min_clearance
    return finish()


def _attach_complexity(
    report: SafetyReport,
    *,
    spec: Any,
    config: SafetyConfig,
    relative_speed: float,
    spawn_distance_hint: float,
) -> None:
    """Attach the *measured* complexity score to a report.

    Requirement: difficulty is a function of the parameter space, not a label.
    Every input here is either measured from the built scenario / free-space
    analysis or explicitly configured - nothing is a hand-tuned level name.
    """
    from bullet_sim.scenarios.params import complexity_from_measurements

    player_r = float(spec.player_radius)
    score = complexity_from_measurements(
        obstacle_count=int(report.obstacle_count),
        free_fraction=float(report.free_fraction),
        reachable_fraction=float(report.reachable_fraction),
        min_clearance=float(report.min_clearance),
        player_diameter=2.0 * player_r,
        relative_speed=float(relative_speed),
        player_speed=float(spec.player_speed),
        prediction_horizon=float(config.horizon),
        reaction_time=None,
        spawn_distance_hint=float(spawn_distance_hint),
    )
    report.complexity = float(score.complexity)
    report.complexity_label = score.label
    report.complexity_terms = dict(score.terms)
    report.complexity_advice = list(score.advice)


def is_scenario_valid(
    built_or_world: Any, *, spec: Any = None, config: SafetyConfig | None = None
) -> bool:
    """Boolean shorthand for :func:`validate_scenario`."""
    return validate_scenario(built_or_world, spec=spec, config=config).feasible


# --------------------------------------------------------------------------
# closed loop: generate -> check -> adjust -> regenerate
# --------------------------------------------------------------------------


def generate_valid_scenario(
    scenario: ObstacleScenario,
    *,
    config: SafetyConfig | None = None,
    max_attempts: int = 8,
    strict: bool = True,
    adjust: bool = True,
) -> tuple[ObstacleScenario, SafetyReport, int]:
    """Regenerate/adjust until the scene passes the safety check.

    Requirement: *do not* give up after one attempt.  Each failed attempt makes
    a targeted parameter change (wider gaps, fewer/slower/smaller obstacles,
    later start) and retries.  ``strict=False`` returns the last (infeasible)
    attempt together with its report instead of raising.
    """
    from bullet_sim.scenarios.builder import build_from_spec

    current = scenario
    report: SafetyReport | None = None
    for attempt in range(1, max(1, int(max_attempts)) + 1):
        spec = current.to_spec()
        built = build_from_spec(spec)
        report = validate_scenario(
            built, spec=spec, config=config, strategy=_scenario_strategy(current),
            attempts=attempt,
        )
        if report.feasible:
            return current, report, attempt
        if not adjust:
            break
        current = relax_scenario(current, report)
    assert report is not None
    if strict:
        raise ConfigError(
            "无法在 "
            f"{max_attempts} 次调整内生成可行场景：{report.reason}\n"
            + "\n".join(f"  - {s}" for s in report.suggestions)
        )
    return current, report, int(max_attempts)


def relax_scenario(
    scenario: ObstacleScenario, report: SafetyReport, *, factor: float = 0.8
) -> ObstacleScenario:
    """One targeted relaxation step driven by *why* the scene failed."""
    from dataclasses import replace

    spawns = []
    free = report.free_fraction < report.min_free_fraction
    for spawn in scenario.obstacles:
        changes: dict[str, Any] = {}
        if spawn.count:
            changes["count"] = max(1, int(round(spawn.count * factor)))
        if spawn.speed:
            changes["speed"] = max(10.0, float(spawn.speed) * factor)
        if spawn.size:
            changes["size"] = max(0.05, float(spawn.size) * factor)
        if spawn.gap_width:
            changes["gap_width"] = float(spawn.gap_width) / max(factor, 1e-6)
        if spawn.corridor_width:
            changes["corridor_width"] = float(spawn.corridor_width) / max(factor, 1e-6)
        if free and spawn.extra.get("spawn") is None:
            changes["start_time"] = (spawn.start_time or 0.6) + 0.4
        spawns.append(replace(spawn, **changes))
    return replace(
        scenario,
        obstacles=spawns,
        # a denser scene is allowed a slightly smaller margin before giving up
        safety_margin=float(scenario.safety_margin),
    )


def _scenario_strategy(scenario: ObstacleScenario) -> str:
    """Pick the check that matters most for this composition (requirement 11)."""
    strategies = [
        CATALOG[k].safety_strategy for k in scenario.type_keys if k in CATALOG
    ]
    for pick in ("gap", "corridor", "local_free", "bypass"):
        if pick in strategies:
            return pick
    return "generic"


def _suggestions_for(spec: Any, kind: str, report: SafetyReport) -> list[str]:
    """Concrete parameter changes, in the order most likely to help."""
    out: list[str] = []
    if kind == "free":
        out += [
            "减少障碍数量 obstacle_count",
            "减小障碍尺寸 obstacle_size",
            "加大缺口 gap_width / 通道 corridor_width",
        ]
    if kind in ("reach", "path"):
        out += [
            "降低障碍速度 obstacle_speed（提高相对反应时间 reaction_time）",
            "增大 gap_width 或 corridor_width",
            "缩短预测时窗 safety_horizon（只在更短时间内要求可行）",
            "减小 safety_margin",
        ]
    out.append("或调用 generate_valid_scenario(...) 让平台自动调参重采样")
    return out


def _min_clearance(world: Any, inflate: float) -> float:
    clearances = getattr(world.collision, "last_clearances", None)
    if clearances is None or len(clearances) == 0:
        return 0.0
    return float(np.min(clearances))


def _as_world(built_or_world: Any, *, spec: Any = None) -> Any:
    from bullet_sim.scenarios.builder import BuiltScenario
    from bullet_sim.simulator.world import World

    if isinstance(built_or_world, World):
        return built_or_world
    if isinstance(built_or_world, BuiltScenario):
        return World(built_or_world, reward="zero", terminate_on_collision=False)
    if spec is not None:
        return World(spec, reward="zero", terminate_on_collision=False)
    return World(built_or_world, reward="zero", terminate_on_collision=False)


__all__ = [
    "SafetyConfig",
    "SafetyReport",
    "SafeRegion",
    "STRATEGY_NOTES",
    "compute_safe_region",
    "find_safe_path",
    "predict_collision",
    "validate_scenario",
    "is_scenario_valid",
    "generate_valid_scenario",
    "relax_scenario",
    "free_space_sequence",
]
