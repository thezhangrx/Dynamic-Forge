"""Long-Term Viability (LTV): coarse future-mobility evaluation (v0.5.1).

# [v0.5.1-LongTermViability]
# PURPOSE:
#   Answer a question v0.4.1's short FAR cannot: after executing candidate ``a``
#   and *reaching* the future, is there still enough escape space to keep moving?
#   Candidate A can look safer for the next 0.3 s yet steer into a dead end;
#   candidate B can look slightly worse yet keep a corridor open.  LTV extracts
#   that long-term information at a few anchors, WITHOUT ever building a full
#   0.8 s trajectory (no 17 x 96 x N rollout).
#
# OPEN-SOURCE REFERENCE (structure only; no code is copied):
#   PythonRobotics DWA  - finite-horizon candidate rollout / trajectory scoring
#   Nav2 MPPI           - trajectory evaluation + critic separation
#   TEB                 - "different candidates may belong to different local
#                         route classes" + finite class count + switch hysteresis
#                         (NOT a homotopy graph, NOT an optimizer, no ROS)
#   CommonRoad          - feasibility separated from utility
#   F1TENTH             - gap / free-direction representation, iTTC ideas
#   TinyMPC             - fixed-size, resource-constrained, embedded-friendly
#   RVO2 / ORCA         - relative-velocity / velocity-space idea only
#
# ALGORITHM:
#   anchors = clamp(ratios * scene_scale, H_short, max_long_horizon)   (S = 3)
#   for each anchor t and shortlisted candidate a:
#       p_agent(a,t) = p_agent + v_candidate(a) * t        # future AGENT
#       p_obstacle(t) = p_obstacle + v_obstacle * t        # future OBSTACLE
#       r(a,t) = p_obstacle(t) - p_agent(a,t); v_rel(a) = v_obstacle - v_candidate
#       sectorize into 8 fixed sectors -> clearance / closing
#       dynamic_buffer = min(base_buffer + max(0,closing)*margin_time,
#                            max_dynamic_buffer)
#       free[d] = clearance[d] - dynamic_buffer[d] >= mobility_margin
#       safe_sector_count, max_continuous_safe_sector (circular),
#       corridor_min_clearance (over the widest circular free run)
#   mobility = (safe_count/8 + max_run/8 + clip(corridor_min_clearance*inv_ref,0,1))/3
#   viability(a) = min(mobility(t1), mobility(t2), mobility(t3))
#
# FPGA MAPPING:
#   candidate (K<=4) x anchor (S=3) x obstacle (N): relative position, distance^2,
#   clearance, blocked/free, 8-sector min/max/count, 8-step circular run scan,
#   min clearance.  add/multiply/square/compare/min/max only.  The clearance
#   normalisation uses a *compile-time reciprocal constant* (inv_clearance_ref)
#   or a threshold/piecewise step - no runtime division in the core kernel.
#
# CPU ROLE:
#   This module only produces features (mobility / viability / corridor).
#   Shortlist choice, viability-vs-risk arbitration, FES, behaviour, task,
#   action hysteresis and the final argmin stay on the CPU.
#
# COMPLEXITY:
#   O(K * S * N + S * 8) per plan, K = 4, S = 3.  NOT O(17 * 96 * N).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from cpu.decision.immediate import SECTOR_DIRECTIONS
from cpu.decision.models import Action, SceneState

__all__ = [
    "ViabilityConfig",
    "ViabilityContext",
    "FutureMobility",
    "ViabilityProfile",
    "LongTermViabilityEvaluator",
    "direction_class",
]

_EPS = 1e-9


@dataclass(frozen=True)
class ViabilityConfig:
    """Long-Term Viability parameters (v0.5.1 frozen defaults)."""

    # -- shortlist ---------------------------------------------------------
    top_k: int = 4
    leaders: int = 3
    diversity: int = 1
    shortlist_margin: float = 0.25
    # -- anchors -----------------------------------------------------------
    anchors: tuple[float, float, float] | None = None
    anchor_ratios: tuple[float, float, float] = (0.125, 0.229, 0.333)
    max_long_horizon: float = 0.8          # v0.5.1 locked
    anchor_eps: float = 1e-3
    # -- mobility ----------------------------------------------------------
    mobility_margin: float = 0.0
    #: Long-Term Mobility spatial normalisation scale -- this is ``escape_radius``
    #: (=50).  It is NOT a reaction distance, NOT a TTC threshold, NOT a
    #: collision probability.
    clearance_ref: float = 50.0
    max_dynamic_buffer: float = 20.0       # [v0.5.1] LTV-only buffer cap
    # -- arbitration -------------------------------------------------------
    risk_band_width: float = 0.05          # bucket width (NOT a collision risk)
    # -- corridor ----------------------------------------------------------
    corridor_min_width: int = 2
    corridor_switch_threshold: float = 0.10
    # -- low-threat fast path (skip LTV only if ALL hold) ------------------
    fast_risk_band_max: int = 0
    fast_margin_min: float = 4.0
    fast_openness_min: float = 0.5
    fast_decision_gap: float = 0.05
    # -- override guard (v0.5.2-LTVGuarded) --------------------------------
    # PURPOSE:
    #   Make LTV a *secondary* mechanism: it may only change the action when it
    #   is a clear, same-risk-band improvement over the v0.4.1 short-term best.
    #   Without this, LTV re-ranked candidates on small viability differences and
    #   the v0.5.1 run regressed (collision 61.7% -> 70.4%, override 18.96%).
    #
    # ALGORITHM:
    #   A candidate may replace the short-term best only if ALL hold:
    #     A. feasible (enforced upstream);
    #     B. critical band not worse than the reference;
    #     C. risk band *equal* to the reference (never cross a risk bucket);
    #     D. viability gain over the reference >= viability_improvement_threshold;
    #     E. not long-term degraded (no narrower escape corridor than the
    #        reference by more than long_term_degraded_margin);
    #     F. not an EmergencyFallback (enforced upstream).
    #   Otherwise the v0.4.1 short-term best is kept.
    #
    #: Master switch for the override guard (default ON for v0.5.2).
    guard_enabled: bool = True
    #: Minimum viability gain required to justify an override (condition D).
    viability_improvement_threshold: float = 0.10
    #: A candidate whose escape-corridor clearance is below the reference's by
    #: more than this margin counts as long-term degraded (condition E).
    long_term_degraded_margin: float = 0.0

    def __post_init__(self) -> None:
        if self.top_k < 1:
            raise ValueError("top_k must be >= 1")
        if self.leaders < 0 or self.diversity < 0:
            raise ValueError("leaders/diversity must be non-negative")
        if self.leaders + self.diversity > self.top_k:
            raise ValueError("leaders + diversity must be <= top_k")
        if self.max_long_horizon <= 0.0:
            raise ValueError("max_long_horizon must be > 0")
        if self.clearance_ref <= 0.0:
            raise ValueError("clearance_ref must be > 0")
        if self.risk_band_width <= 0.0:
            raise ValueError("risk_band_width must be > 0")
        if self.max_dynamic_buffer < 0.0 or self.mobility_margin < 0.0:
            raise ValueError("buffers must be non-negative")
        if self.corridor_min_width < 1:
            raise ValueError("corridor_min_width must be >= 1")
        if self.viability_improvement_threshold < 0.0:
            raise ValueError("viability_improvement_threshold must be >= 0")
        if self.long_term_degraded_margin < 0.0:
            raise ValueError("long_term_degraded_margin must be >= 0")

    @property
    def inv_clearance_ref(self) -> float:
        """Compile-time reciprocal used instead of a runtime division."""
        return 1.0 / self.clearance_ref


@dataclass(frozen=True)
class ViabilityContext:
    """Explicit, immutable per-frame context (replaces any mutable setter).

    ``scene_scale`` is ``T_scene = min(field_w, field_h) / max_speed`` and is
    supplied by the adapter/controller that knows the field.  ``None`` falls back
    to ratios scaled by ``max_long_horizon``.  ``base_buffer`` / ``margin_time``
    mirror the planner's ASE ``SafetyEnvelopeConfig`` so LTV reuses the same
    dynamic-buffer semantics.
    """

    short_horizon: float
    scene_scale: float | None = None
    base_buffer: float = 0.0
    margin_time: float = 0.04


@dataclass(frozen=True)
class FutureMobility:
    """Per-anchor mobility features (arrays are length B, one entry per candidate)."""

    anchor: float
    safe_sector_count: np.ndarray
    max_continuous_safe_sector: np.ndarray
    corridor_min_clearance: np.ndarray
    mobility: np.ndarray
    corridor_id: np.ndarray
    corridor_width: np.ndarray
    corridor_direction: np.ndarray


@dataclass(frozen=True)
class ViabilityProfile:
    """Long-Term Viability result for one planning frame."""

    viability: np.ndarray          # (B,) in [0,1]; 0 for non-shortlist
    shortlist: np.ndarray          # (B,) bool
    viability_cost: np.ndarray     # (B,) 1-viability inside, 1+margin outside
    anchors: np.ndarray            # (S,)
    per_anchor: tuple[FutureMobility, ...]
    corridor_id: np.ndarray        # (B,) t1 representative, -1 when not evaluated
    corridor_width: np.ndarray
    corridor_direction: np.ndarray
    corridor_min_clearance: np.ndarray
    openness_now: float
    evaluated: int
    skipped: bool = False
    skip_reason: str = ""
    run_reason: str = ""


def direction_class(action: Action) -> int:
    """8-way direction class of a candidate heading (``-1`` for a stop action).

    Used by the Top-K diversity rule.  It depends only on the candidate action --
    it never reads mobility / viability / corridor, so diversity cannot "peek"
    at the long-term evaluation it is supposed to seed.
    """
    if action.speed <= 0.0:
        return -1
    return int(round(math.degrees(action.steering_angle) / 45.0)) % 8


def _circular_distance(a: int, b: int) -> int:
    d = abs(a - b) % 8
    return min(d, 8 - d)


def _circular_max_run(free: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Longest circular run of True per row; returns ``(length, start)``.

    Vectorised over rows (candidates) with a fixed 2S-step scan, so the FPGA
    version is a bounded compare/counter chain (DIR7 -> DIR0 handled by doubling).
    """
    k, s = free.shape
    if s == 0:
        return np.zeros(k, dtype=np.int64), np.zeros(k, dtype=np.int64)
    doubled = np.concatenate([free, free], axis=1).astype(np.int64)   # (K, 2S)
    runs = np.zeros_like(doubled)
    runs[:, 0] = doubled[:, 0]
    for j in range(1, 2 * s):
        runs[:, j] = np.where(doubled[:, j] > 0, runs[:, j - 1] + 1, 0)
    max_len = np.minimum(runs.max(axis=1), s)
    end = runs.argmax(axis=1)
    start = (end - max_len + 1) % s
    return max_len.astype(np.int64), start.astype(np.int64)


def _sector_summary_batch(
    r: np.ndarray, v_rel: np.ndarray, radius_sum: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Batched 8-sector reduction over ``(K, N)`` candidate/obstacle pairs."""
    k = r.shape[0]
    n = r.shape[1]
    s = SECTOR_DIRECTIONS.shape[0]
    nearest = np.full((k, s), np.inf, dtype=np.float64)
    closing = np.zeros((k, s), dtype=np.float64)
    density = np.zeros((k, s), dtype=np.float64)
    if k == 0 or n == 0:
        return nearest, closing, density, np.full((k, n), np.inf, dtype=np.float64)

    d = np.sqrt(np.maximum((r * r).sum(axis=-1), _EPS * _EPS))        # (K, N)
    clearance = d - np.asarray(radius_sum, dtype=np.float64)[None, :]  # (K, N)
    dot = (r * v_rel).sum(axis=-1)                                     # (K, N)
    close = np.maximum(-dot, 0.0) / d                                  # (K, N)
    idx = np.argmax(r @ SECTOR_DIRECTIONS.T, axis=-1)                  # (K, N)

    kk = np.repeat(np.arange(k), n)
    flat = idx.ravel()
    np.minimum.at(nearest, (kk, flat), clearance.ravel())
    np.maximum.at(closing, (kk, flat), close.ravel())
    np.add.at(density, (kk, flat), 1.0)
    return nearest, closing, density, clearance


def _mobility_from_summary(
    nearest: np.ndarray,
    closing: np.ndarray,
    base_buffer: float,
    margin_time: float,
    cfg: ViabilityConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """``(mobility, safe_count, max_run, corridor_min_clearance, corridor_dir)``."""
    if nearest.shape[0] == 0:
        z = np.zeros(0, dtype=np.float64)
        return z, np.zeros(0, np.int64), np.zeros(0, np.int64), z.copy(), np.zeros(0, np.int64)
    closing_pos = np.maximum(closing, 0.0)
    dynamic = np.minimum(
        float(base_buffer) + closing_pos * float(margin_time), cfg.max_dynamic_buffer
    )
    free = (nearest - dynamic) >= cfg.mobility_margin
    safe_count = free.sum(axis=1).astype(np.int64)
    max_run, start = _circular_max_run(free)
    s = free.shape[1]

    sector_pos = np.arange(s)[None, :]
    offset = (sector_pos - start[:, None]) % s
    in_corridor = offset < max_run[:, None]
    corridor_clear = np.where(
        in_corridor & np.isfinite(nearest), nearest, np.inf
    ).min(axis=1)
    corridor_clear = np.where(max_run > 0, corridor_clear, 0.0)

    clear_term = np.clip(corridor_clear * cfg.inv_clearance_ref, 0.0, 1.0)
    mobility = (safe_count / s + max_run / s + clear_term) / 3.0
    direction = np.where(
        max_run <= 0,
        -1,
        np.where(max_run >= s, 0, (start + (max_run - 1) // 2) % s),
    ).astype(np.int64)
    return mobility, safe_count, max_run, corridor_clear, direction


class LongTermViabilityEvaluator:
    """Stateless (pure) Long-Term Viability evaluator.

    # [v0.5.1-LongTermViability]
    # PURPOSE:
    #   Produce the long-term mobility / viability / corridor features for the
    #   Top-K shortlist only, plus a low-threat skip decision.  No mutable state:
    #   the scene scale is passed explicitly through :class:`ViabilityContext`.
    #
    # OPEN-SOURCE REFERENCE:
    #   TEB route-class idea + hysteresis; Nav2 critic separation; CommonRoad
    #   feasibility/utility split; F1TENTH gap; TinyMPC fixed-size.  Structure
    #   only; no code copied.
    #
    # ALGORITHM:
    #   anchors -> shortlist (leaders + diversity) -> per-anchor 8-sector
    #   mobility (future agent + future obstacles) -> viability = min over anchors.
    #
    # FPGA MAPPING:
    #   candidate x anchor x obstacle kernels; see module docstring.
    #
    # CPU ROLE:
    #   Feature extraction only; the planner owns arbitration and the action.
    #
    # COMPLEXITY:
    #   O(K * S * N).
    """

    def __init__(self, config: ViabilityConfig | None = None) -> None:
        self.config = config or ViabilityConfig()

    # ------------------------------------------------------------------
    def anchors(self, context: ViabilityContext) -> np.ndarray:
        """Clamped, monotonic anchor times (S = 3)."""
        cfg = self.config
        if cfg.anchors is not None:
            raw = [float(t) for t in cfg.anchors]
        elif context.scene_scale is not None and context.scene_scale > 0.0:
            raw = [c * float(context.scene_scale) for c in cfg.anchor_ratios]
        else:
            last = cfg.anchor_ratios[-1]
            raw = [c / last * cfg.max_long_horizon for c in cfg.anchor_ratios]

        short = float(context.short_horizon)
        hi = cfg.max_long_horizon
        eps = cfg.anchor_eps
        # Reserve room for strict ordering even when every raw anchor saturates.
        short = min(short, hi - 2.0 * eps)
        values = sorted(min(max(v, short), hi) for v in raw)
        values[0] = min(max(values[0], short), hi - 2.0 * eps)
        values[1] = min(max(values[1], values[0] + eps), hi - eps)
        values[2] = min(max(values[2], values[1] + eps), hi)
        return np.asarray(values, dtype=np.float64)

    # ------------------------------------------------------------------
    def current_openness(self, scene: SceneState, context: ViabilityContext) -> float:
        """Current-frame free-sector openness ``max_run / 8`` (no future model).

        Used by the planner's low-threat fast path.  It is a current-frame proxy
        and never reads anchor-level mobility.
        """
        positions, velocities, radii = _obstacle_arrays(scene)
        n = radii.shape[0]
        if n == 0:
            return 1.0
        r = positions - scene.agent.position[None, :]
        v_rel = velocities - scene.agent.velocity[None, :]
        nearest, closing, _density, _clearance = _sector_summary_batch(
            r[None, :, :], v_rel[None, :, :], scene.agent.radius + radii
        )
        _mob, _safe, max_run, _cmin, _cdir = _mobility_from_summary(
            nearest, closing, context.base_buffer, context.margin_time, self.config
        )
        return float(max_run[0] / nearest.shape[1])

    # ------------------------------------------------------------------
    def shortlist(
        self,
        actions: Sequence[Action],
        short_term_cost: np.ndarray,
        critical_band: np.ndarray,
        feasible: np.ndarray,
    ) -> np.ndarray:
        """Top-K = leaders + one diversity candidate (diversity decided first)."""
        cfg = self.config
        actions = tuple(actions)
        b = len(actions)
        mask = np.zeros(b, dtype=bool)
        feas = np.flatnonzero(feasible)
        if feas.size == 0:
            return mask

        risk_band = np.floor(
            np.asarray(short_term_cost, dtype=np.float64) / cfg.risk_band_width
        ).astype(np.int64)
        cb = np.asarray(critical_band, dtype=np.int64)
        order = sorted(feas.tolist(), key=lambda i: (int(cb[i]), int(risk_band[i]), i))

        leaders = order[: cfg.leaders]
        for i in leaders:
            mask[i] = True

        if cfg.diversity > 0:
            remaining = [i for i in order if not mask[i]]
            if remaining:
                classes = {direction_class(actions[i]) for i in leaders}
                uncovered = [i for i in remaining if direction_class(actions[i]) not in classes]
                if uncovered:
                    diversity = uncovered[0]
                else:
                    rank = {i: r for r, i in enumerate(order)}

                    def min_angle(i: int) -> int:
                        di = direction_class(actions[i])
                        if di < 0:
                            return 8
                        return min(
                            _circular_distance(di, direction_class(actions[j]))
                            for j in leaders
                        )

                    diversity = max(
                        remaining, key=lambda i: (min_angle(i), -rank[i], -i)
                    )
                mask[diversity] = True
        return mask

    # ------------------------------------------------------------------
    def evaluate(
        self,
        scene: SceneState,
        actions: Sequence[Action],
        short_term_cost: np.ndarray,
        critical_band: np.ndarray,
        feasible: np.ndarray,
        *,
        context: ViabilityContext,
    ) -> ViabilityProfile:
        """Evaluate mobility/viability for the shortlist (stateless)."""
        cfg = self.config
        actions = tuple(actions)
        b = len(actions)
        anchors = self.anchors(context)
        openness = self.current_openness(scene, context)
        shortlist = self.shortlist(actions, short_term_cost, critical_band, feasible)

        viability = np.zeros(b, dtype=np.float64)
        corridor_id = np.full(b, -1, dtype=np.int64)
        corridor_width = np.zeros(b, dtype=np.int64)
        corridor_direction = np.full(b, -1, dtype=np.int64)
        corridor_clear = np.zeros(b, dtype=np.float64)
        viability_cost = np.full(b, 1.0 + cfg.shortlist_margin, dtype=np.float64)
        per_anchor: list[FutureMobility] = []
        evaluated = int(shortlist.sum())

        if evaluated == 0:
            return ViabilityProfile(
                viability=viability,
                shortlist=shortlist,
                viability_cost=viability_cost,
                anchors=anchors,
                per_anchor=(),
                corridor_id=corridor_id,
                corridor_width=corridor_width,
                corridor_direction=corridor_direction,
                corridor_min_clearance=corridor_clear,
                openness_now=openness,
                evaluated=0,
            )

        positions, velocities, radii = _obstacle_arrays(scene)
        n = radii.shape[0]
        agent = scene.agent
        idx = np.flatnonzero(shortlist)
        v_cand = np.stack([actions[i].velocity_vector for i in idx])       # (K,2)
        if n:
            radius_sum = agent.radius + radii
            v_rel = velocities[None, :, :] - v_cand[:, None, :]           # (K,N,2)
        else:
            radius_sum = np.zeros(0, dtype=np.float64)
            v_rel = np.zeros((idx.size, 0, 2), dtype=np.float64)

        mobility_stack: list[np.ndarray] = []
        for t in anchors:
            t = float(t)
            p_agent = agent.position[None, :] + v_cand * t                 # (K,2)
            if n:
                p_obs = positions[None, :, :] + velocities[None, :, :] * t  # (1,N,2)
            else:
                p_obs = np.zeros((1, 0, 2), dtype=np.float64)
            r = p_obs - p_agent[:, None, :]                                # (K,N,2)
            nearest, closing, _density, _clearance = _sector_summary_batch(
                r, v_rel, radius_sum
            )
            mob, safe, run, cmin, cdir = _mobility_from_summary(
                nearest, closing, context.base_buffer, context.margin_time, cfg
            )
            mobility_stack.append(mob)

            safe_full = np.zeros(b, dtype=np.int64)
            run_full = np.zeros(b, dtype=np.int64)
            cmin_full = np.zeros(b, dtype=np.float64)
            cdir_full = np.full(b, -1, dtype=np.int64)
            safe_full[idx] = safe
            run_full[idx] = run
            cmin_full[idx] = cmin
            cdir_full[idx] = cdir
            per_anchor.append(
                FutureMobility(
                    anchor=t,
                    safe_sector_count=safe_full,
                    max_continuous_safe_sector=run_full,
                    corridor_min_clearance=cmin_full,
                    mobility=_scatter(mob, idx, b),
                    corridor_id=cdir_full.copy(),
                    corridor_width=run_full.copy(),
                    corridor_direction=cdir_full.copy(),
                )
            )

        mobility_stack_arr = np.stack(mobility_stack)                      # (S,K)
        viability_sub = mobility_stack_arr.min(axis=0)                     # (K,)
        viability[idx] = viability_sub
        viability_cost[idx] = 1.0 - viability_sub

        first = per_anchor[0]
        corridor_id[idx] = first.corridor_id[idx]
        corridor_width[idx] = first.corridor_width[idx]
        corridor_direction[idx] = first.corridor_direction[idx]
        corridor_clear[idx] = first.corridor_min_clearance[idx]

        return ViabilityProfile(
            viability=viability,
            shortlist=shortlist,
            viability_cost=viability_cost,
            anchors=anchors,
            per_anchor=tuple(per_anchor),
            corridor_id=corridor_id,
            corridor_width=corridor_width,
            corridor_direction=corridor_direction,
            corridor_min_clearance=corridor_clear,
            openness_now=openness,
            evaluated=evaluated,
        )


def _scatter(values: np.ndarray, idx: np.ndarray, size: int) -> np.ndarray:
    out = np.zeros(size, dtype=np.float64)
    out[idx] = values
    return out


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
