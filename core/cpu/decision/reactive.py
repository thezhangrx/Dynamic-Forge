"""ReactiveGap: one-frame, sector-based reactive planner (v0.5.0-ReactiveGap).

# [v0.5.0-DualPlanner]
# PURPOSE:
#   Provide a strong, representative *reactive* baseline that is definitionally
#   distinct from the predictive planner: it sees the CURRENT frame only, builds
#   an 8-sector immediate safety picture, and picks a direction from it.  It
#   never predicts ``p(t+dt)``, never rolls a candidate forward, never uses FAR,
#   FES, risk_near/mid/far or an adaptive future horizon.
#
# OPEN-SOURCE REFERENCE:
#   F1TENTH Follow-the-Gap (nearest-obstacle bubble -> free gap -> steering),
#   F1TENTH iTTC (current distance + closing speed as an immediate safety layer),
#   PythonRobotics potential field (goal attraction idea), Nav2 velocity
#   deadband / behavior-critic anti-wobble idea.  Structure only; no code copied.
#
# ALGORITHM:
#   current frame -> sectorize -> per-sector
#       immediate_margin = clearance - (base_buffer + closing_rate * reaction_time)
#       gap_score        = clearance[d] + w*(clearance[d-1] + clearance[d+1])
#       goal_alignment   = (1 + dot(dir[d], unit(goal)))/2
#   levels: margin < 0 -> unsafe (infeasible); margin < critical_margin ->
#   critical (penalty); else normal.  Score is positional
#   (critical > gap > goal > side), then safety-first hysteresis, then a
#   threat-aware speed scale (with STOP gated by its own frame safety check).
#
# FPGA MAPPING:
#   The board can stream sector_clearance[8], sector_closing[8] and
#   sector_density[8]; the CPU does gap/goal/hysteresis/action.  No horizon.
#
# CPU ROLE:
#   All selection (gap, goal, hysteresis, stop gating) stays on the CPU.
#
# COMPLEXITY:
#   O(N) geometry + O(8) scoring per frame; no future samples, no rollout.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from cpu.decision.build import BUILD_ID, REACTIVE_VERSION
from cpu.decision.immediate import (
    SECTOR_DEGREES,
    SECTOR_DIRECTIONS,
    SectorSummary,
    gap_scores,
    radial_closing_rate,
    sectorize,
    surface_clearance,
)
from cpu.decision.models import Action, SceneState

__all__ = [
    "ReactiveConfig",
    "ReactiveDiagnostics",
    "ReactiveGapPlanner",
    "REACTIVE_VERSION",
]

_EPS = 1e-9


@dataclass(frozen=True)
class ReactiveConfig:
    """ReactiveGap parameters (current-frame only; configurable, not tuned)."""

    #: constant buffer added on top of surface contact.
    base_buffer: float = 0.0
    #: seconds of current closing speed converted into an immediate buffer
    #: (same reaction-time scale as the existing repulsion baseline).
    reaction_time: float = 0.45
    #: margin band that counts as critical.
    critical_margin: float = 4.0
    #: neighbour weight of the circular gap score.
    gap_neighbor_weight: float = 0.5
    #: clearance is capped at ``max_speed * reaction_time * this`` for scoring.
    clearance_cap_multiplier: float = 2.0
    #: positional tier weights (critical > field > closing > gap > goal > side).
    critical_weight: float = 1000.0
    #: potential-field preference tier (repulsion + goal attraction projected on
    #: the 8 sectors); this is the primary steering term and is what keeps a
    #: reactive controller from locking onto an empty-but-useless direction.
    field_weight: float = 300.0
    #: continuous immediate-threat (iTTC) tier: high closing inside the reaction
    #: time is penalised even while the geometric clearance is still positive.
    closing_weight: float = 50.0
    gap_weight: float = 20.0
    goal_weight: float = 10.0
    side_weight: float = 1.0
    #: obstacle count at which the density term saturates.
    density_ref: float = 4.0
    density_weight: float = 0.25
    #: potential-field shape (PythonRobotics-style repulsion + goal attraction).
    repel_weight: float = 1.0
    attract_weight: float = 0.15
    min_influence: float = 12.0
    #: if the realised speed is below this fraction of max speed while a moving
    #: direction was commanded, that direction is treated as blocked this frame
    #: (a reactive anti-pinning guard; no prediction involved).
    blocked_speed_frac: float = 0.05
    #: margin at (or above) which full speed is allowed.
    speed_full_margin: float = 20.0
    #: below this speed scale a STOP may be considered (only if STOP is safe).
    stop_scale_threshold: float = 0.15
    #: STOP is allowed only when its own immediate margin is at least this.
    stop_safety_margin: float = 0.0
    #: safety-first action hysteresis.
    hysteresis_enabled: bool = True
    #: a new direction must beat the held one by more than this to switch.
    switch_score_margin: float = 15.0
    #: a clearly better goal alignment may override the hold (cosine gain).
    goal_switch_threshold: float = 0.25

    def __post_init__(self) -> None:
        tiers = [
            self.critical_weight,
            self.field_weight,
            self.closing_weight,
            self.gap_weight,
            self.goal_weight,
            self.side_weight,
        ]
        if min(tiers) < 0.0:
            raise ValueError("reactive tier weights must be non-negative")
        for i, weight in enumerate(tiers[:-1]):
            if weight <= sum(tiers[i + 1:]):
                raise ValueError(
                    "each reactive tier must dominate all lower tiers"
                )
        if self.critical_margin <= 0.0:
            raise ValueError("critical_margin must be > 0")
        if self.reaction_time < 0.0 or self.base_buffer < 0.0:
            raise ValueError("reaction_time and base_buffer must be non-negative")
        if not 0.0 <= self.density_weight <= 1.0:
            raise ValueError("density_weight must be in [0, 1]")


@dataclass
class ReactiveDiagnostics:
    """Inspectable record of the last reactive decision (v0.5.0-ReactiveGap)."""

    version: str = REACTIVE_VERSION
    build_id: str = ""
    chosen_action: Action | None = None
    sector: int = -1
    chosen_clearance: float = float("inf")
    chosen_closing_rate: float = 0.0
    chosen_margin: float = float("inf")
    chosen_gap_score: float = 0.0
    chosen_goal_score: float = 0.0
    chosen_speed_scale: float = 0.0
    action_switched: bool = False
    emergency_active: bool = False
    stopped: bool = False
    sector_clearance: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sector_closing: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sector_density: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sector_margin: np.ndarray = field(default_factory=lambda: np.zeros(0))
    sector_score: np.ndarray = field(default_factory=lambda: np.zeros(0))


class ReactiveGapPlanner:
    """Pick a current-frame direction from an 8-sector immediate safety picture."""

    def __init__(self, config: ReactiveConfig | None = None) -> None:
        self.config = config or ReactiveConfig()
        self.last_diagnostics: ReactiveDiagnostics | None = None
        self._last_choice: int | None = None  # 0..7 sector, 8 = STOP

    def reset(self) -> None:
        self._last_choice = None
        self.last_diagnostics = None

    # ------------------------------------------------------------------
    def plan(self, scene: SceneState) -> Action:
        """Return a current-frame :class:`Action` (deterministic; never None).

        # [v0.5.0-DualPlanner]
        # PURPOSE:
        #   Decide with the current frame only, so the reactive baseline is a
        #   genuine one-frame method and the benchmark comparison stays honest.
        #
        # OPEN-SOURCE REFERENCE:
        #   F1TENTH Follow-the-Gap + iTTC; PythonRobotics potential field;
        #   Nav2 anti-wobble deadband.  Structure only.
        #
        # ALGORITHM:
        #   1. r, v_rel -> sectorize -> per-sector clearance / closing
        #   2. immediate_margin = clearance - (base_buffer + closing*reaction_time)
        #   3. unsafe -> excluded; critical -> penalty; else normal
        #   4. score = critical*crit + gap*gap + goal*goal + side*side
        #   5. safety-first hysteresis on the direction
        #   6. threat-aware speed scale; STOP only if its own margin allows
        #   7. all sectors unsafe -> deterministic emergency direction
        #
        # FPGA MAPPING:
        #   Consumes the fixed sector vectors; no future sample, no candidate.
        #
        # CPU ROLE:
        #   Gap selection, goal alignment, hysteresis and the action are CPU-side.
        #
        # COMPLEXITY:
        #   O(N) + O(8).
        # """
        cfg = self.config
        agent = scene.agent
        max_speed = float(agent.max_speed)

        positions, velocities, radii = self._obstacle_arrays(scene)
        n = radii.shape[0]
        if n:
            r = positions - agent.position[None, :]
            v_rel = velocities - agent.velocity[None, :]
            radius_sum = agent.radius + radii
        else:
            r = np.zeros((0, 2), dtype=np.float64)
            v_rel = np.zeros((0, 2), dtype=np.float64)
            radius_sum = np.zeros(0, dtype=np.float64)

        summary = sectorize(r, v_rel, radius_sum)              # (8,) each
        clearance = summary.nearest_clearance
        closing = summary.closing_rate
        margin = clearance - (cfg.base_buffer + closing * cfg.reaction_time)

        cap = max(max_speed * cfg.reaction_time * cfg.clearance_cap_multiplier, _EPS)
        clearance_capped = np.minimum(clearance, cap)
        gap = gap_scores(clearance_capped, cfg.gap_neighbor_weight)
        gap_norm = np.clip(
            gap / max(cap * (1.0 + 2.0 * cfg.gap_neighbor_weight), _EPS), 0.0, 1.0
        )
        side_norm = np.clip(
            np.minimum(np.roll(clearance_capped, 1), np.roll(clearance_capped, -1))
            / cap,
            0.0,
            1.0,
        )
        goal_norm = self._goal_alignment(scene, agent.position)

        # ---- potential-field preference (repulsion + goal attraction) --------
        # A pure "clearest gap" score can lock onto an empty but useless direction;
        # projecting the classic repulsion/goal field onto the 8 sectors restores
        # the smooth steering a one-frame controller needs (PythonRobotics
        # potential-field idea; still current-frame only, no rollout).
        influence = max(cfg.min_influence, max_speed * cfg.reaction_time)
        if n:
            dist = np.sqrt(np.maximum((r * r).sum(axis=-1), _EPS * _EPS))
            # falloff on CENTRE distance (like the reference potential field), so
            # the influence radius is exactly `influence` and does not grow with
            # the obstacle radius.
            falloff = np.clip(1.0 - dist / max(influence, _EPS), 0.0, 1.0) ** 2
            unit = r / np.maximum(dist, _EPS)[:, None]
            repel = -(unit * falloff[:, None]).sum(axis=0)
        else:
            repel = np.zeros(2, dtype=np.float64)
        if scene.goal is not None:
            toward = np.asarray(scene.goal, dtype=np.float64) - agent.position
            toward_dist = float(np.hypot(toward[0], toward[1]))
            attract = (
                (toward / toward_dist) * cfg.attract_weight
                if toward_dist > _EPS
                else np.zeros(2, dtype=np.float64)
            )
        else:
            attract = np.zeros(2, dtype=np.float64)
        field = SECTOR_DIRECTIONS @ (cfg.repel_weight * repel + attract)
        field_norm = field / max(float(np.abs(field).max()), _EPS)

        unsafe = margin < 0.0
        # Anti-pinning guard: if the previously commanded moving direction did
        # not translate into motion (realised speed ~ 0), the agent is blocked
        # (e.g. against the field edge); treat that direction as unusable now.
        realised_speed = float(np.hypot(agent.velocity[0], agent.velocity[1]))
        if (
            self._last_choice is not None
            and self._last_choice < 8
            and realised_speed < cfg.blocked_speed_frac * max_speed
        ):
            unsafe = unsafe.copy()
            unsafe[int(self._last_choice)] = True
        critical = (~unsafe) & (margin < cfg.critical_margin)
        critical_score = np.where(
            unsafe,
            1.0,
            np.clip(
                (cfg.critical_margin - margin) / max(cfg.critical_margin, _EPS),
                0.0,
                1.0,
            ),
        )
        # Continuous immediate threat: how much of the reaction-time buffer the
        # current closing speed eats.  This restores the smooth "repulsion"
        # behaviour a one-frame controller needs, without any rollout.
        buffer = closing * cfg.reaction_time
        threat_norm = np.clip(buffer / np.maximum(clearance, _EPS), 0.0, 1.0)
        density_norm = np.clip(
            summary.density / max(cfg.density_ref, _EPS), 0.0, 1.0
        )
        threat_score = np.clip(
            threat_norm + cfg.density_weight * density_norm, 0.0, 1.0
        )

        # score is a COST (argmin): criticality and threat add, gap/goal/side are
        # goodness terms and therefore subtract.  Positional tiers keep safety
        # dominant over gap, which stays dominant over goal.
        score = (
            cfg.critical_weight * critical_score
            + cfg.closing_weight * threat_score
            - cfg.field_weight * field_norm
            - cfg.gap_weight * gap_norm
            - cfg.goal_weight * goal_norm
            - cfg.side_weight * side_norm
        )

        feasible = ~unsafe
        emergency = not bool(feasible.any())
        if emergency:
            # deterministic least-bad escape: max margin, then min closing, then index
            best = int(
                min(
                    range(8),
                    key=lambda d: (-float(margin[d]), float(closing[d]), d),
                )
            )
        else:
            best = int(np.flatnonzero(feasible)[int(np.argmin(score[feasible]))])

        # ---- safety-first direction hysteresis ---------------------------
        if (
            cfg.hysteresis_enabled
            and not emergency
            and self._last_choice is not None
            and self._last_choice < 8
            and self._last_choice != best
        ):
            held = int(self._last_choice)
            held_is_safe = not bool(unsafe[held]) and not bool(critical[held])
            marginal = float(score[held] - score[best]) <= cfg.switch_score_margin
            goal_gain = float(goal_norm[best] - goal_norm[held])
            # Hold only a *safe* direction whose advantage is marginal AND whose
            # goal alignment is not clearly worse; goal pursuit is never
            # suppressed by the anti-wobble rule.
            if held_is_safe and marginal and goal_gain <= cfg.goal_switch_threshold:
                best = held

        # ---- threat-aware speed + STOP gating -----------------------------
        best_margin = float(margin[best])
        if not np.isfinite(best_margin):
            speed_scale = 1.0
        else:
            speed_scale = float(np.clip(best_margin / max(cfg.speed_full_margin, _EPS), 0.0, 1.0))

        stop_margin = self._stop_margin(n, r, velocities, radius_sum, cfg)
        stop_allowed = bool(stop_margin >= cfg.stop_safety_margin)
        stopped = (
            not emergency
            and stop_allowed
            and speed_scale <= cfg.stop_scale_threshold
        )
        if emergency:
            speed_scale = 1.0

        if stopped:
            action = Action.stop()
            final_choice = 8
        else:
            speed = max_speed * max(speed_scale, 1e-3)
            action = Action(speed, float(np.deg2rad(SECTOR_DEGREES[best])))
            final_choice = best

        switched = self._last_choice is not None and final_choice != self._last_choice
        self._last_choice = final_choice

        self.last_diagnostics = ReactiveDiagnostics(
            version=REACTIVE_VERSION,
            build_id=BUILD_ID,
            chosen_action=action,
            sector=best,
            chosen_clearance=float(clearance[best]),
            chosen_closing_rate=float(closing[best]),
            chosen_margin=best_margin,
            chosen_gap_score=float(gap_norm[best]),
            chosen_goal_score=float(goal_norm[best]),
            chosen_speed_scale=float(speed_scale),
            action_switched=bool(switched),
            emergency_active=bool(emergency),
            stopped=bool(stopped),
            sector_clearance=clearance,
            sector_closing=closing,
            sector_density=summary.density,
            sector_margin=margin,
            sector_score=score,
        )
        return action

    # ------------------------------------------------------------------
    @staticmethod
    def _obstacle_arrays(
        scene: SceneState,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        obstacles = scene.obstacles
        if not obstacles:
            return (
                np.zeros((0, 2), dtype=np.float64),
                np.zeros((0, 2), dtype=np.float64),
                np.zeros(0, dtype=np.float64),
            )
        positions = np.stack([o.position for o in obstacles]).astype(np.float64, copy=False)
        velocities = np.stack([o.velocity for o in obstacles]).astype(np.float64, copy=False)
        radii = np.asarray([o.radius for o in obstacles], dtype=np.float64)
        return positions, velocities, radii

    @staticmethod
    def _goal_alignment(scene: SceneState, position: np.ndarray) -> np.ndarray:
        """``(1 + cos(angle to goal)) / 2`` per sector; zeros when no goal."""
        if scene.goal is None:
            return np.zeros(len(SECTOR_DEGREES), dtype=np.float64)
        toward = np.asarray(scene.goal, dtype=np.float64) - position
        dist = float(np.hypot(toward[0], toward[1]))
        if dist <= _EPS:
            return np.zeros(len(SECTOR_DEGREES), dtype=np.float64)
        return (1.0 + SECTOR_DIRECTIONS @ (toward / dist)) / 2.0

    @staticmethod
    def _stop_margin(
        n: int,
        r: np.ndarray,
        obstacle_velocities: np.ndarray,
        radius_sum: np.ndarray,
        cfg: ReactiveConfig,
    ) -> float:
        """Immediate margin of the STOP action (agent velocity = 0).

        STOP must pass the same current-frame safety check as any direction,
        otherwise the controller could "brake into" an incoming obstacle.
        """
        if n == 0:
            return float("inf")
        closing_stop = np.maximum(radial_closing_rate(r, obstacle_velocities), 0.0)
        margins = surface_clearance(r, radius_sum) - (
            cfg.base_buffer + closing_stop * cfg.reaction_time
        )
        return float(margins.min())
