"""Action selection: FAR + ASE + guarded Long-Term Viability (v0.5.2-LTVGuarded).

The candidate action space is expressed only through ``speed`` and
``steering_angle`` (see :class:`~cpu.decision.models.Action`), so a
bullet-hell player ("LEFT / RIGHT") and a 2D cart share the exact same planner.

History
-------
v0.3-4 fixed the candidate action space (full-circle, 8 headings x 2 speeds +
stop = 17).  v0.3-5 added safety-gated feasible-only ranking plus a
deterministic emergency fallback.  v0.4.0 added Future Action Risk (candidate
conditioned relative velocity, continuous closest approach, risk trend and a
strict critic hierarchy) but the 1000-episode run showed the failure budget still
dominated by ``side_hit`` and a rise in action changes.

v0.4.1-SideHit-ASE keeps every v0.4.0 mechanism and adds, in order::

    candidate-specific lateral closing rate      (far.future_action_risk_batch)
        -> adaptive safety margin / envelope      (LEVEL 0 normal / 1 critical / 2 unsafe)
        -> side-hit critical critic               (far.critical_risk_critic)
        -> high-risk local action refinement      (<= 8 extra candidates, danger only)
        -> safety-first action hysteresis         (hold a safe action unless clearly worse)
        -> hierarchical argmin                    (safety > critical > future risk
                                                   > FES > behaviour > task)

The CPU always owns the final action; the candidate x obstacle kernels are the
ones a future FPGA would parallelise.

v0.5.1-LongTermViability adds, without touching any v0.4.1 mechanism::

    Top-K shortlist (K=4 = 3 short-term leaders + 1 LTV-blind diversity)
        -> Long-Term Viability at 3 anchors (future agent x future obstacles)
        -> bucketed arbitration (critical band > risk band > viability > FES ...)
        -> corridor hysteresis, then action hysteresis

Diversity is chosen before and independently of LTV: it may use the candidate
direction and the current frame, never mobility / viability / corridor.

v0.5.2-LTVGuarded makes that arbitration *advisory* rather than authoritative.
The v0.5.1 run regressed against the v0.4.1 reference (collision 61.7% -> 70.4%,
LTV override 18.96%) because LTV re-ranked candidates across risk buckets and on
small viability differences.  v0.5.2 keeps the whole LTV pipeline (shortlist,
anchors, mobility, bucketed arbitration, corridor + action hysteresis) exactly
as it was, but the pipeline's proposal is only *applied* when it is a clear
improvement over the v0.4.1 short-term best::

    short-term best (FAR + ASE + hard feasibility + v0.4.1 hysteresis)
        -> LTV proposal (unchanged v0.5.1 chain)
        -> OVERRIDE GUARD: same critical band, SAME risk band,
           viability gain >= viability_improvement_threshold,
           not long-term degraded, not emergency
        -> otherwise fall back to the short-term best

FAR/ASE remain the primary decision; LTV is a secondary tie-break that only
avoids obvious long-term dead ends among short-term-equivalent candidates.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from cpu.decision.build import BUILD_ID, VERSION
from cpu.decision.corridor_selector import (
    FSC_SELECTOR_ENABLED,
    LEVEL_HARD_SAFETY,
    build_future_space_table,
    select_corridor_candidate,
)
from cpu.decision.cost import SafetyMetrics, TrajectoryCost, UtilityTerms
from cpu.decision.future_corridor import SafeCorridorShadow
from cpu.decision.far import (
    ASE_BASE_MARGIN,
    ASE_CRITICAL_MARGIN,
    ASE_LATERAL_RATE_SCALE_FRAC,
    ASE_LATERAL_RATE_THRESHOLD_FRAC,
    ASE_MARGIN_TIME,
    SafetyEnvelope,
    adaptive_safety_envelope,
    candidate_safety_batch,
    critical_risk_critic,
    future_action_risk_batch,
    future_risk_critic,
)
from cpu.decision.models import Action, SceneState
from cpu.decision.predictor import (
    LinearPredictor,
    Predictor,
    Trajectory,
    TrajectoryBatch,
)
from cpu.decision.viability import (
    LongTermViabilityEvaluator,
    ViabilityConfig,
    ViabilityContext,
    ViabilityProfile,
)

__all__ = [
    "ActionPlanner",
    "HierarchicalCritics",
    "SafetyEnvelopeConfig",
    "RefinementConfig",
    "HysteresisConfig",
    "PlanDiagnostics",
    "SafetyGate",
    "VERSION",
    "BUILD_ID",
    "ENABLE_LTV",
    "LTV_DEBUG_DIAGNOSTICS",
    "LTV_SHADOW_ONLY",
    "LTV_OVERRIDE_GUARD",
    "default_candidate_actions",
]

_EPS = 1e-9

#: Experimental switch (observability round, no algorithm change):
#: ``False`` makes the planner follow the strict v0.4.1 decision path - LTV is
#: never evaluated and the v0.4.1 positional ranking + v0.4.1 action hysteresis
#: decide.  Used to separate "LTV changed the decision" from "something else
#: regressed".  Override with the ``DF_ENABLE_LTV`` environment variable.
ENABLE_LTV: bool = os.environ.get("DF_ENABLE_LTV", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}

#: Diagnostics switch: when ``True`` the planner prints one observability line
#: per decision (override reason, deltas, corridor before/after).  It never
#: changes control logic.  Override with ``DF_LTV_DEBUG``.
LTV_DEBUG_DIAGNOSTICS: bool = os.environ.get("DF_LTV_DEBUG", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}

#: Shadow switch (v0.5.2-LTVGuarded): when ``True`` the full LTV pipeline still
#: runs and fills every diagnostic field, but it may never change the action --
#: the emitted action is exactly the strict v0.4.1 short-term best.  Used to
#: replay "what LTV wanted" against "what v0.4.1 did" without any control risk.
#: Override with the ``DF_LTV_SHADOW`` environment variable.
LTV_SHADOW_ONLY: bool = os.environ.get("DF_LTV_SHADOW", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}

#: Override-guard switch (v0.5.2-LTVGuarded): ``False`` restores the raw v0.5.1
#: arbitration (LTV may freely re-rank) for A/B comparison only.  The guard is
#: ON by default because the raw v0.5.1 path regressed against v0.4.1.
#: Override with the ``DF_LTV_GUARD`` environment variable.
LTV_OVERRIDE_GUARD: bool = os.environ.get("DF_LTV_GUARD", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}

#: Moving speed levels as a fraction of the agent's max speed (stop is added
#: separately, once, instead of being duplicated across every heading).
SPEED_LEVELS: tuple[float, ...] = (0.5, 1.0)
#: Full-circle headings in degrees (CCW from +x): 0, 45, ..., 315.
STEERING_DEGREES: tuple[int, ...] = tuple(range(0, 360, 45))


def default_candidate_actions(max_speed: float) -> tuple[Action, ...]:
    """Stop plus every full-circle heading at each moving speed level (17 actions).

    The base set stays fixed at 17 (8 headings x 2 speeds + stop); v0.4.1 only
    ever adds a *temporary* <= 8 local refinement set under high risk.  Headings
    coincide with the 8 non-zero directions of the 9-way discrete table.
    """
    s = float(max_speed)
    moving = tuple(
        Action(s * level, math.radians(deg))
        for deg in STEERING_DEGREES
        for level in SPEED_LEVELS
    )
    return (Action.stop(),) + moving


class SafetyGate:
    """Three-layer feasibility gate built on the cost's clearance / TTC model.

    * layer 1 - **true predicted collision** (``min_clearance < collision_threshold``):
      the candidate is *infeasible* and stays out of normal cost ranking;
    * layer 2 - **near collision** (``collision_threshold <= min_clearance <
      near_miss_threshold``): still feasible, but the normal cost receives a
      critical penalty that grows as clearance shrinks;
    * layer 3 - **normal clearance**: only the existing multi-objective cost applies.

    v0.4.1 keeps this gate unchanged and runs the new Adaptive Safety Envelope
    *before* it; both feed the final critical tier.
    """

    def __init__(
        self,
        collision_threshold: float = 0.0,
        near_miss_threshold: float = 10.0,
        critical_weight: float = 4.0,
    ) -> None:
        self.collision_threshold = float(collision_threshold)
        self.near_miss_threshold = float(near_miss_threshold)
        self.critical_weight = float(critical_weight)
        if self.near_miss_threshold <= self.collision_threshold:
            raise ValueError("near_miss_threshold must be > collision_threshold")
        if self.critical_weight < 0.0:
            raise ValueError("critical_weight must be non-negative")

    def infeasible(self, metrics: SafetyMetrics) -> np.ndarray:
        """Boolean ``(B,)`` mask of candidates at/inside the collision threshold."""
        return metrics.min_clearance < self.collision_threshold

    def critical_penalty(self, metrics: SafetyMetrics) -> np.ndarray:
        """``(B,)`` near-collision penalty in ``[0, critical_weight]``.

        Zero at/above ``near_miss_threshold`` and maximal at the collision
        threshold; the linear ramp stays monotone in clearance, so a slightly
        closer trajectory is never cheaper than a farther one.
        """
        span = self.near_miss_threshold - self.collision_threshold
        closeness = (self.near_miss_threshold - metrics.min_clearance) / span
        return self.critical_weight * np.clip(closeness, 0.0, 1.0)


@dataclass(frozen=True)
class SafetyEnvelopeConfig:
    """Adaptive Safety Envelope parameters (v0.4.1-SideHit-ASE).

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Define the adaptive buffer that makes a fast lateral approach critical
    #   before the geometric clearance reaches zero (the v0.4.0 side_hit mode).
    #
    # OPEN-SOURCE REFERENCE:
    #   F1TENTH iTTC safety layer + Nav2 MPPI near-collision critic; structure
    #   only, no implementation copied.
    #
    # ALGORITHM:
    #   dynamic_margin = base_margin + closing_rate * margin_time
    #   min_margin     = min over samples of clearance - dynamic_margin
    #
    # FPGA MAPPING:
    #   One multiply-add per (candidate, obstacle) plus a min-reduce.
    #
    # CPU ROLE:
    #   Supplies the Level-1/Level-2 boundary; the CPU still decides the action.
    #
    # COMPLEXITY:
    #   O(1) given the FAR kernel.
    """

    base_margin: float = ASE_BASE_MARGIN
    margin_time: float = ASE_MARGIN_TIME
    critical_margin: float = ASE_CRITICAL_MARGIN
    lateral_rate_threshold_frac: float = ASE_LATERAL_RATE_THRESHOLD_FRAC
    lateral_rate_scale_frac: float = ASE_LATERAL_RATE_SCALE_FRAC

    def __post_init__(self) -> None:
        if self.critical_margin <= 0.0:
            raise ValueError("critical_margin must be > 0")
        if self.base_margin < 0.0 or self.margin_time < 0.0:
            raise ValueError("base_margin and margin_time must be non-negative")
        if self.lateral_rate_scale_frac <= 0.0:
            raise ValueError("lateral_rate_scale_frac must be > 0")


@dataclass(frozen=True)
class RefinementConfig:
    """High-risk local action refinement parameters (v0.4.1-SideHit-ASE).

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Under danger only, probe a few headings around the provisional best so
    #   the 45-degree base grid is not the limiting factor for an evasive turn.
    #   The base action space stays 17; normal risk never pays for this.
    #
    # OPEN-SOURCE REFERENCE:
    #   DWA dynamic-window sampling (local control refinement) and Nav2 MPPI's
    #   local trajectory perturbation - reduced to <= 8 fixed offsets, no
    #   stochastic sampling, no recursion.
    #
    # ALGORITHM:
    #   trigger = feasible_count <= feasible_threshold
    #             OR best_risk_peak > risk_threshold
    #             OR best_min_margin < critical_margin
    #   extra   = offsets(-15,-7.5,+7.5,+15 deg) x 2 speed levels, capped at max_extra
    #
    # FPGA MAPPING:
    #   Bounded by max_extra, so the worst case is a fixed 25-candidate kernel.
    #
    # CPU ROLE:
    #   The CPU decides *whether* to refine (data-dependent branch) and still
    #   owns the final argmin; refinement never recurses.
    #
    # COMPLEXITY:
    #   O(B * S * N) with B <= 25, and only in danger.
    """

    enabled: bool = True
    risk_threshold: float = 0.5
    feasible_threshold: int = 4
    max_extra: int = 8
    offsets_deg: tuple[float, ...] = (-15.0, -7.5, 7.5, 15.0)

    def __post_init__(self) -> None:
        if self.max_extra < 0:
            raise ValueError("max_extra must be non-negative")
        if self.feasible_threshold < 0:
            raise ValueError("feasible_threshold must be non-negative")


@dataclass(frozen=True)
class HysteresisConfig:
    """Safety-first action hysteresis parameters (v0.4.1-SideHit-ASE).

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Stop the planner from chattering between near-equivalent safe actions
    #   (v0.4.0 action changes = 150.5) without ever holding a dangerous action.
    #
    # OPEN-SOURCE REFERENCE:
    #   Common industrial hysteresis / Nav2 behavior-tree "goal still valid"
    #   checks; the safety override priority comes from the layered critic
    #   design (Nav2 MPPI), not from a smoothing rule.
    #
    # ALGORITHM:
    #   hold the previous action iff it is feasible, level 0, and its risk trend
    #   is not worsening, and the new best is not *clearly* safer:
    #     switch if held unsafe/critical, OR held risk_trend > trend_threshold,
    #     OR critical gain > critical_advantage, OR risk gain > risk_advantage,
    #     OR total gain > switch_margin.  Otherwise hold.
    #
    # FPGA MAPPING:
    #   A handful of compares against the previous candidate index.
    #
    # CPU ROLE:
    #   CPU-only bookkeeping; safety always overrides the hold.
    #
    # COMPLEXITY:
    #   O(1).
    """

    enabled: bool = True
    switch_margin: float = 30.0
    critical_advantage: float = 0.02
    risk_advantage: float = 0.05
    trend_threshold: float = 0.15


@dataclass(frozen=True)
class HierarchicalCritics:
    """Tier weights for the lexicographic critic hierarchy (v0.4.1-SideHit-ASE).

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Guarantee the required ordering
    #       safety > critical dynamic risk > future risk > FES > behaviour > task
    #   so a high-progress (or extra-smooth) action can never buy back a
    #   dangerous one.  Each tier is bounded to [0, 1] and the weights are
    #   positional (a positional number system, not unbounded weight inflation):
    #   every tier dominates the sum of all lower tiers by construction.
    #
    # OPEN-SOURCE REFERENCE:
    #   Nav2 MPPI critic layering; DWA's separated obstacle / speed / goal groups.
    #
    # FPGA MAPPING:
    #   Fixed-point positional weighting: one multiply-accumulate per tier.
    #
    # CPU ROLE:
    #   The CPU evaluates the tiers and picks argmin; no lower layer may choose.
    #
    # COMPLEXITY:
    #   O(1) per candidate given the tier scores.
    """

    #: Level 2 - critical dynamic safety (dominant scored tier).
    critical: float = 10000.0
    #: Level 3 - future risk.
    risk: float = 1000.0
    #: Level 4 - future escape space (FES / trap).
    fes: float = 100.0
    #: Level 5 - behaviour quality (smooth + dwell).
    behavior: float = 10.0
    #: Level 6 - task objective (progress).
    task: float = 1.0
    #: Gain on the (positive) risk trend inside the future-risk tier.
    trend_gain: float = 1.0
    #: Gain on the threat-aware speed coupling inside the future-risk tier.
    speed_risk_gain: float = 1.0

    def __post_init__(self) -> None:
        tiers = [self.critical, self.risk, self.fes, self.behavior, self.task]
        if min(tiers) < 0.0:
            raise ValueError("critic tier weights must be non-negative")
        for i, weight in enumerate(tiers[:-1]):
            if weight <= sum(tiers[i + 1:]):
                raise ValueError(
                    "each critic tier must dominate all lower tiers "
                    "(lexicographic ordering)"
                )


@dataclass
class PlanDiagnostics:
    """Deterministic, inspectable record of the last :meth:`ActionPlanner.plan`.

    Keeps the v0.3-5 field names (``used_emergency`` / ``n_feasible`` /
    ``chosen_index`` / ``infeasible``) for API compatibility and adds the
    v0.4.1 ASE attribution needed to explain a side_hit after the fact.
    """

    used_emergency: bool
    n_feasible: int
    chosen_index: int
    infeasible: np.ndarray  # (B,) bool mask

    version: str = ""
    build_id: str = ""
    chosen_action: Action | None = None
    chosen_risk_peak: float = 0.0
    chosen_risk_trend: float = 0.0
    chosen_min_ttc: float = float("inf")
    chosen_min_clearance: float = float("inf")
    chosen_fes: float = 0.0
    critics: dict = field(default_factory=dict)

    # -- v0.4.1-SideHit-ASE -------------------------------------------------
    critical_count: int = 0
    chosen_safety_margin: float = float("inf")
    chosen_closing_rate: float = 0.0
    chosen_action_switched: bool = False
    switch_reason: str = ""
    refinement_used: bool = False
    refinement_count: int = 0

    # -- v0.5.1-LongTermViability -------------------------------------------
    viability_skipped: bool = False
    skip_reason: str = ""
    viability_run_reason: str = ""
    viability_evaluated: int = 0
    long_anchors: np.ndarray | None = None
    chosen_viability: float = 0.0
    chosen_safe_sector_count: int = 0
    chosen_corridor_min_clearance: float = 0.0
    chosen_corridor_width: int = 0
    chosen_corridor_direction: int = -1
    chosen_corridor_persistence: int = 0
    short_term_us: float = 0.0
    viability_us: float = 0.0
    total_us: float = 0.0

    # -- v0.5.1 observability (diagnostics round; NO control effect) --------
    #: v0.4.1 reference: FAR+ASE+hard feasibility + v0.4.1 action hysteresis, no LTV.
    short_term_best_index: int = -1
    short_term_best_action: Action | None = None
    short_term_best_risk: float = 0.0
    short_term_best_fes: float = 0.0
    #: Full v0.5.1 planner choice.
    final_chosen_index: int = -1
    final_chosen_action: Action | None = None
    #: True when the LTV pipeline changed the v0.4.1 short-term decision.
    ltv_override: bool = False
    ltv_override_reason: str = ""
    short_term_best_viability: float = 0.0
    final_chosen_viability: float = 0.0
    viability_override_risk_delta: float = 0.0
    viability_override_viability_delta: float = 0.0
    corridor_before: int = -1
    corridor_after: int = -1
    corridor_switched: bool = False
    corridor_switch_kind: str = ""
    #: Deterministic emergency fallback (excluded from the normal override count).
    emergency: bool = False
    enable_ltv: bool = True
    decision_reason: str = ""

    # -- v0.5.2-LTVGuarded override guard -----------------------------------
    #: Shadow mode: LTV is computed but can never change the action.
    ltv_shadow: bool = False
    #: Whether the same-risk-band / minimum-viability guard was active.
    ltv_guard_enabled: bool = True
    #: What the raw LTV pipeline *wanted* before the guard / shadow veto.
    ltv_proposed_index: int = -1
    #: Chosen risk bucket minus the reference risk bucket (0 = same bucket).
    risk_band_delta: int = 0
    #: The LTV proposal was long-term degraded (narrower corridor than the ref).
    long_term_degraded: bool = False
    #: Which guard condition vetoed the proposal ("" = none / override allowed).
    override_guard_reason: str = ""
    #: Diagnostics: how many LTV-eligible alternatives were examined and why the
    #: guard rejected them (per-frame counts).
    override_candidate_count: int = 0
    override_blocked_by_risk_band: int = 0
    override_blocked_by_viability_margin: int = 0
    override_blocked_by_long_term_degraded: int = 0
    override_blocked_by_critical_band: int = 0
    override_blocked_by_no_reference: int = 0

    # -- v0.6.0-SafeCorridor-Shadow read-only observation -------------------
    #: The *real* Top-K shortlist this frame's planner actually used (3 short
    #: term leaders + 1 diversity, in planner order).  Observation only: these
    #: fields are never read by any control expression, so the emitted action is
    #: bit-identical whether or not anyone looks at them.  Empty when the LTV
    #: pipeline did not run (low-threat fast path, or ENABLE_LTV=False).
    fsc_candidate_indices: tuple[int, ...] = ()
    #: The shortlist actions themselves (indices refer to the planner's own
    #: candidate array, which may include the <=8 refinement actions).
    fsc_candidate_actions: tuple[Action, ...] = ()
    #: ``"leader"`` / ``"diversity"`` per shortlist entry.
    fsc_candidate_sources: tuple[str, ...] = ()
    #: Why the shortlist is empty / where it came from:
    #: ``"ltv_shortlist"`` | ``"ltv_skipped"`` | ``"ltv_disabled"``.
    fsc_candidate_source: str = ""

    # -- v0.6.0-PredictiveCorridorSelector ----------------------------------
    #: Predictive corridor selection over the already-safe candidate set.
    #: OFF by default (``DF_ENABLE_FSC_SELECTOR``); when off the action path is
    #: exactly v0.5.2-LTVGuarded and none of the corridor machinery runs.
    fsc_selector_enabled: bool = False
    fsc_selector_applied: bool = False
    #: The v0.5.2 (pre-selector) choice, always recoverable.
    fsc_selector_base_index: int = -1
    fsc_selector_index: int = -1
    #: Which lexicographic level produced the change ("" when unchanged).
    fsc_selector_level: str = ""
    fsc_selector_reason: str = ""
    #: Level-1 output: candidates that passed the existing hard safety filter.
    fsc_hard_survivors: int = 0
    #: Candidates that also passed the critical-band floor.
    fsc_eligible: int = 0
    #: UNKNOWN-at-t3 candidates that still competed (never an infinite penalty).
    fsc_unknown_candidates: int = 0
    fsc_triggered_reachability: bool = False
    fsc_triggered_persistent: bool = False
    fsc_triggered_max_run: bool = False
    fsc_triggered_clearance: bool = False
    fsc_triggered_existing_tiers: bool = False
    #: Raw corridor facts of the v0.5.2 choice vs the winner (diagnostics).
    fsc_base_reachable: bool = False
    fsc_winner_reachable: bool = False
    fsc_base_persistent: int = 0
    fsc_winner_persistent: int = 0
    fsc_base_max_run_t3: int = 0
    fsc_winner_max_run_t3: int = 0
    #: Cost of materialising the corridor table (0 when the selector is off).
    fsc_table_us: float = 0.0

    @property
    def override_reason(self) -> str:
        """Alias of :attr:`ltv_override_reason` (v0.5.2 naming)."""
        return self.ltv_override_reason

    @property
    def emergency_used(self) -> bool:
        """Alias of :attr:`used_emergency` (v0.4.0 naming)."""
        return self.used_emergency

    @property
    def feasible_count(self) -> int:
        """Alias of :attr:`n_feasible` (v0.4.0 naming)."""
        return self.n_feasible


class ActionPlanner:
    """Pick the candidate action with the best FAR / ASE hierarchical score."""

    def __init__(
        self,
        predictor: Predictor | None = None,
        cost: TrajectoryCost | None = None,
        *,
        safety_gate: SafetyGate | None = None,
        critics: HierarchicalCritics | None = None,
        envelope: SafetyEnvelopeConfig | None = None,
        refinement: RefinementConfig | None = None,
        hysteresis: HysteresisConfig | None = None,
        viability: ViabilityConfig | None = None,
        enable_ltv: bool | None = None,
        ltv_debug: bool | None = None,
        ltv_shadow: bool | None = None,
        fsc_selector: bool | None = None,
        candidate_actions: Sequence[Action] | None = None,
        horizon: float | None = None,
        danger_horizon: float = 0.6,
        danger_threshold: float = 1.0,
        smooth_danger_threshold: float = 0.5,
        smooth_danger_weight: float = 0.1,
        debug: bool = False,
    ) -> None:
        self.predictor = predictor or LinearPredictor()
        self.cost = cost or TrajectoryCost()
        #: Feasibility gate (configurable via :class:`SafetyGate`).
        self.safety_gate = safety_gate or SafetyGate()
        #: Hierarchical critic weights (see :class:`HierarchicalCritics`).
        self.critics = critics or HierarchicalCritics()
        #: Adaptive Safety Envelope parameters.
        self.envelope_cfg = envelope or SafetyEnvelopeConfig()
        #: High-risk local refinement parameters.
        self.refinement_cfg = refinement or RefinementConfig()
        #: Safety-first hysteresis parameters.
        self.hysteresis_cfg = hysteresis or HysteresisConfig()
        #: Long-Term Viability parameters (see :class:`ViabilityConfig`).
        self.viability_cfg = viability or ViabilityConfig(
            guard_enabled=LTV_OVERRIDE_GUARD
        )
        #: Stateless LTV evaluator; the scene scale arrives via ViabilityContext.
        self.viability = LongTermViabilityEvaluator(self.viability_cfg)
        #: Experimental switch: False -> strict v0.4.1 path (LTV never evaluated).
        self.enable_ltv = ENABLE_LTV if enable_ltv is None else bool(enable_ltv)
        #: Diagnostics-only switch; it never changes control logic.
        self.ltv_debug = LTV_DEBUG_DIAGNOSTICS if ltv_debug is None else bool(ltv_debug)
        #: Shadow switch: run all of LTV but never let it change the action.
        self.ltv_shadow = LTV_SHADOW_ONLY if ltv_shadow is None else bool(ltv_shadow)
        #: v0.6.0 predictive corridor selector; OFF by default (bit-identical
        #: v0.5.2).  The corridor evaluator is only created when it is ON.
        self.fsc_selector_enabled = (
            FSC_SELECTOR_ENABLED if fsc_selector is None else bool(fsc_selector)
        )
        self._fsc_shadow = None
        #: Filled by :meth:`plan`; ``None`` before the first call.
        self.last_diagnostics: PlanDiagnostics | None = None
        #: Print a one-line summary per decision when ``True``.
        self.debug = bool(debug)
        self._candidates: tuple[Action, ...] | None = (
            tuple(candidate_actions) if candidate_actions is not None else None
        )
        #: Base lookahead (``None`` -> the scene's own horizon).
        self.horizon = horizon
        #: Lookahead used while in danger (min TTC below ``danger_threshold``).
        self.danger_horizon = float(danger_horizon)
        self.danger_threshold = float(danger_threshold)
        #: Below this min-TTC the smoothness weight is lowered to allow sharp turns.
        self.smooth_danger_threshold = float(smooth_danger_threshold)
        self.smooth_danger_weight = float(smooth_danger_weight)
        self._last_action: Action | None = None
        #: Previous discrete local escape corridor class (corridor hysteresis).
        self._last_corridor_id: int | None = None

    def candidate_actions(self, scene: SceneState) -> tuple[Action, ...]:
        if self._candidates is not None:
            return self._candidates
        return default_candidate_actions(scene.agent.max_speed)

    def horizon_for(self, scene: SceneState, ttc_min: float | None = None) -> float:
        """Dynamic horizon: extend the lookahead when a threat is imminent."""
        base = self.horizon if self.horizon is not None else scene.horizon
        if ttc_min is None:
            ttc_min = self._estimate_min_ttc(scene)
        if ttc_min < self.danger_threshold:
            return max(base, self.danger_horizon)
        return base

    # ------------------------------------------------------------------
    def plan(
        self,
        scene: SceneState,
        *,
        context: ViabilityContext | None = None,
    ) -> Action:
        """Return the chosen candidate action for ``scene`` (deterministic).

        # [v0.5.1-LongTermViability]
        # PURPOSE:
        #   Add a dynamic safety envelope, a side-hit critical tier, a high-risk
        #   local refinement and a safety-first hysteresis on top of FAR, so the
        #   side_hit failure mode and the action chattering are both attacked
        #   without weakening any existing mechanism.
        #
        # OPEN-SOURCE REFERENCE:
        #   DWA (candidate rollout, collision candidates discarded), Nav2 MPPI
        #   (batched critics, true-collision vs near-collision separation,
        #   fallback), CommonRoad (feasibility separated from utility), F1TENTH
        #   (iTTC safety layer), RVO2/ORCA (relative-velocity geometry idea only),
        #   TinyMPC (fixed-size, embedded-friendly).  Structure only.
        #
        # ALGORITHM:
        #   1. base 17 candidates -> FAR safety + risk profile + ASE envelope;
        #   2. if the provisional best is critical / high risk, add <= 8 local
        #      refinement candidates and recompute FAR on the combined set;
        #   3. LEVEL 1 hard feasibility = envelope.unsafe;
        #   4. low-threat fast path (ALL of 5 conditions) -> skip LTV and use the
        #      v0.4.1 positional ranking unchanged;
        #   5. otherwise Long-Term Viability on the K=4 shortlist, then bucketed
        #      arbitration (critical band > risk band > viability > FES > ...);
        #   6. corridor hysteresis, then safety-first action hysteresis;
        #   7. no feasible candidate -> deterministic emergency fallback.
        #
        # FPGA MAPPING:
        #   Levels 1-3 are candidate x (sample x obstacle) kernels in far.py
        #   (<= 25 candidates worst case); levels 4-6 are per-candidate scalars.
        #
        # CPU ROLE:
        #   The CPU decides whether to refine, applies hysteresis and owns the
        #   final argmin/fallback; no lower module may substitute an action.
        #
        # COMPLEXITY:
        #   Low risk: O(17 * H * N).  Danger: O(25 * H * N), one refinement level.
        # """
        t_start = time.perf_counter()
        base_candidates = self.candidate_actions(scene)
        state_ttc = self._estimate_min_ttc(scene)
        horizon = self.horizon_for(scene, state_ttc)
        n_steps = max(1, int(round(horizon / scene.dt)))

        smooth_weight = (
            self.smooth_danger_weight
            if state_ttc < self.smooth_danger_threshold
            else self.cost.w_smooth
        )

        # ---- FAR + adaptive envelope on the base set --------------------------
        safety = candidate_safety_batch(scene, base_candidates, horizon)
        profile = self._far(scene, base_candidates, horizon, safety)
        envelope = adaptive_safety_envelope(profile)
        critical_score = self._critical_score(profile, envelope)
        risk_score = self._risk_score(profile, base_candidates, scene.agent.max_speed)

        # ---- danger-only local refinement (never recursive) -------------------
        refinement_used = False
        refinement_count = 0
        candidates = base_candidates
        if self._needs_refinement(
            profile, envelope, base_candidates, critical_score, risk_score
        ):
            provisional = self._provisional_index(
                envelope, critical_score, risk_score, base_candidates
            )
            extra = self._refinement_actions(
                base_candidates, provisional, scene.agent.max_speed
            )
            if extra:
                candidates = base_candidates + extra
                safety = candidate_safety_batch(scene, candidates, horizon)
                profile = self._far(scene, candidates, horizon, safety)
                envelope = adaptive_safety_envelope(profile)
                critical_score = self._critical_score(profile, envelope)
                risk_score = self._risk_score(
                    profile, candidates, scene.agent.max_speed
                )
                refinement_used = True
                refinement_count = len(extra)

        # ---- LEVEL 1: hard safety feasibility ---------------------------------
        infeasible = envelope.unsafe.copy()
        if self.safety_gate.collision_threshold > 0.0:
            infeasible = infeasible | (
                profile.min_clearance < self.safety_gate.collision_threshold
            )
        feasible = np.flatnonzero(~infeasible)

        batch = self.predictor.predict_batch(scene, candidates, n_steps)
        goal = scene.goal
        start_position = scene.agent.position
        previous_velocity = (
            self._last_action.velocity_vector if self._last_action is not None else None
        )

        t_short = time.perf_counter()

        # ---- Long-Term Viability (shortlist only; low-threat fast path) -------
        critical_band = (envelope.level == 1).astype(np.int64)
        ctx = context or self._default_context(scene)
        risk_band = np.floor(
            risk_score / self.viability_cfg.risk_band_width
        ).astype(np.int64)
        ltv: ViabilityProfile | None = None
        viability_skipped = False
        skip_reason = ""
        run_reason = ""
        if feasible.size:
            if not self.enable_ltv:
                # Experimental reference path: LTV is never evaluated.
                viability_skipped = True
                skip_reason = "ltv_disabled"
            else:
                viability_skipped, skip_reason, run_reason = self._fast_path(
                    scene, candidates, feasible, envelope, risk_score, critical_band, ctx
                )
                if not viability_skipped:
                    ltv = self.viability.evaluate(
                        scene, candidates, risk_score, critical_band, ~infeasible,
                        context=ctx,
                    )

        t_ltv = time.perf_counter()

        # ---- v0.6.0 observation: expose the REAL Top-K shortlist ---------------
        # Reproduces LongTermViabilityEvaluator.shortlist's traversal order (the
        # same (critical_band, risk_band, index) key) but only *reads* the mask
        # the evaluator returned.  No control expression consumes these values,
        # so the emitted action is unchanged.
        if ltv is not None:
            order = sorted(
                np.flatnonzero(~infeasible).tolist(),
                key=lambda i: (int(critical_band[i]), int(risk_band[i]), i),
            )
            short_idx = [int(i) for i in order if bool(ltv.shortlist[i])]
            fsc_candidate_indices = tuple(short_idx)
            fsc_candidate_actions = tuple(candidates[i] for i in short_idx)
            fsc_candidate_sources = tuple(
                "leader" if pos < int(self.viability_cfg.leaders) else "diversity"
                for pos in range(len(short_idx))
            )
            fsc_candidate_source = "ltv_shortlist"
        else:
            fsc_candidate_indices = ()
            fsc_candidate_actions = ()
            fsc_candidate_sources = ()
            if not feasible.size:
                fsc_candidate_source = "no_feasible_candidate"
            elif not self.enable_ltv:
                fsc_candidate_source = "ltv_disabled"
            else:
                fsc_candidate_source = "ltv_skipped"

        # ---- LEVELS 2-6 + arbitration -----------------------------------------
        total = np.full(len(candidates), math.inf, dtype=np.float64)
        fes_arr = np.zeros(len(candidates), dtype=np.float64)
        smooth_arr = np.zeros(len(candidates), dtype=np.float64)
        dwell_arr = np.zeros(len(candidates), dtype=np.float64)
        task_arr = np.zeros(len(candidates), dtype=np.float64)

        corridor_before = self._last_corridor_id
        corridor_switched = False
        corridor_kind = ""
        short_term_best_index = -1
        #: What the raw LTV pipeline proposed before the v0.5.2 guard / shadow veto.
        ltv_proposed_index = -1
        guard_reason = ""
        guard_counts = {
            "candidates": 0,
            "critical_band": 0,
            "risk_band": 0,
            "viability_margin": 0,
            "long_term_degraded": 0,
            "no_reference": 0,
        }

        if feasible.size:
            w = self.critics
            for b in feasible:
                b = int(b)
                terms = self.cost.utility_terms(
                    self._trajectory(batch, b),
                    goal=goal,
                    start_position=start_position,
                    previous_velocity=previous_velocity,
                    smooth_weight=smooth_weight,
                )
                fes_n, smooth_n, dwell_n, task_n = self._tier_scores(terms)
                fes_arr[b], smooth_arr[b] = fes_n, smooth_n
                dwell_arr[b], task_arr[b] = dwell_n, task_n
                total[b] = (
                    w.critical * float(critical_score[b])
                    + w.risk * float(risk_score[b])
                    + w.fes * fes_n
                    + w.behavior * 0.5 * (smooth_n + dwell_n)
                    + w.task * task_n
                )

            # v0.4.1 reference (pure, no state mutation): argmin(total) plus the
            # v0.4.1 action hysteresis.  This is "what v0.4.1 would have chosen".
            st_raw = int(feasible[int(np.argmin(total[feasible]))])
            short_term_best_index, _st_sw, st_reason = self._apply_hysteresis(
                candidates, total, critical_score, risk_score, profile, st_raw
            )

            if viability_skipped or ltv is None:
                # v0.4.1 path, unchanged (low-threat fast path or LTV disabled)
                chosen_index = short_term_best_index
                reason = st_reason
                ltv_proposed_index = int(short_term_best_index)
            else:
                # v0.5.1 bucketed arbitration: viability breaks only small risk
                # differences; corridor hysteresis then stabilises the route class.
                behav_arr = 0.5 * (smooth_arr + dwell_arr)
                bucketed_index = self._bucketed_select(
                    feasible, critical_band, risk_band, ltv.viability_cost,
                    fes_arr, behav_arr, task_arr,
                )
                after_hysteresis, _sw, act_reason = self._apply_viability_hysteresis(
                    candidates, ~infeasible, critical_band, risk_band,
                    ltv.viability_cost, envelope, bucketed_index,
                )
                proposed, corridor_switched, corridor_kind = (
                    self._corridor_hysteresis(feasible, after_hysteresis, ltv)
                )
                ltv_proposed_index = int(proposed)
                parts: list[str] = []
                if bucketed_index != short_term_best_index:
                    parts.append("bucketed")
                if after_hysteresis != bucketed_index:
                    parts.append(f"action_hysteresis:{act_reason}")
                if corridor_switched:
                    parts.append(f"corridor:{corridor_kind}")
                proposal_reason = "+".join(parts) if parts else act_reason

                # ---- v0.5.2-LTVGuarded: LTV is advisory, not authoritative ----
                if self.ltv_shadow:
                    # Shadow: everything computed, recorded, and then discarded.
                    chosen_index = short_term_best_index
                    reason = "ltv_shadow"
                    guard_reason = "shadow"
                    self._last_corridor_id = corridor_before
                    corridor_switched = False
                    corridor_kind = ""
                elif not self.viability_cfg.guard_enabled:
                    # Guard disabled -> exact pre-v0.5.2 behaviour (A/B only).
                    chosen_index = ltv_proposed_index
                    reason = proposal_reason
                else:
                    chosen_index, guard_reason, guard_counts = self._guard_override(
                        feasible, critical_band, risk_band, ltv,
                        short_term_best_index, ltv_proposed_index,
                    )
                    if int(chosen_index) == int(ltv_proposed_index):
                        # Override allowed (or LTV agreed with v0.4.1 anyway).
                        reason = (
                            proposal_reason
                            if int(chosen_index) != int(short_term_best_index)
                            else st_reason
                        )
                    else:
                        # Vetoed: keep short-term safety first and undo the
                        # corridor-hysteresis memory the veto invalidated.
                        self._last_corridor_id = corridor_before
                        corridor_switched = False
                        corridor_kind = ""
                        reason = f"ltv_guard:{guard_reason}" if guard_reason else st_reason
            used_emergency = False
        else:
            best_index = self._emergency_index(
                profile.min_ttc, profile.min_clearance
            )
            chosen_index = best_index
            short_term_best_index = best_index
            ltv_proposed_index = best_index
            used_emergency = True
            reason = "emergency_fallback"

        corridor_after = self._last_corridor_id

        t_end = time.perf_counter()
        held = (
            self._find_action(candidates, self._last_action)
            if self._last_action is not None
            else None
        )
        switched = held is None or int(chosen_index) != int(held)

        chosen = candidates[chosen_index]
        self._last_action = chosen

        chosen_viability = float(ltv.viability[chosen_index]) if ltv is not None else 0.0
        chosen_safe = (
            int(ltv.per_anchor[0].max_continuous_safe_sector[chosen_index])
            if ltv is not None and ltv.per_anchor
            else 0
        )
        chosen_cmin = (
            float(ltv.corridor_min_clearance[chosen_index]) if ltv is not None else 0.0
        )
        chosen_cwidth = int(ltv.corridor_width[chosen_index]) if ltv is not None else 0
        chosen_cdir = int(ltv.corridor_direction[chosen_index]) if ltv is not None else -1
        persistence = 0
        if ltv is not None and chosen_cdir >= 0:
            persistence = sum(
                1
                for fm in ltv.per_anchor
                if int(fm.corridor_id[chosen_index]) == chosen_cdir
            )

        # ---- observability: did the LTV pipeline override v0.4.1? -------------
        # NOTE (v0.5.2): the definition is unchanged -- an override is
        # "final_chosen != short_term_best and not EmergencyFallback".  With the
        # guard on, such an override can only survive the same-risk-band +
        # minimum-viability conditions.  Shadow mode never produces one.
        ltv_override = bool(
            (not used_emergency)
            and self.enable_ltv
            and (not self.ltv_shadow)
            and ltv is not None
            and int(chosen_index) != int(short_term_best_index)
        )
        risk_band_delta = 0
        long_term_degraded = False
        if ltv is not None and short_term_best_index >= 0:
            risk_band_delta = int(risk_band[chosen_index]) - int(
                risk_band[short_term_best_index]
            )
            if int(ltv_proposed_index) >= 0:
                long_term_degraded = self._long_term_degraded(
                    ltv, int(ltv_proposed_index), int(short_term_best_index)
                )
        short_term_best_action = (
            candidates[short_term_best_index] if short_term_best_index >= 0 else None
        )
        short_term_best_risk = (
            float(risk_score[short_term_best_index])
            if short_term_best_index >= 0
            else 0.0
        )
        short_term_best_fes = (
            float(fes_arr[short_term_best_index]) if short_term_best_index >= 0 else 0.0
        )
        short_term_best_viability = (
            float(ltv.viability[short_term_best_index])
            if ltv is not None and short_term_best_index >= 0
            else 0.0
        )
        final_chosen_viability = (
            float(ltv.viability[chosen_index]) if ltv is not None else 0.0
        )
        risk_delta = (
            float(risk_score[chosen_index]) - short_term_best_risk
            if short_term_best_index >= 0
            else 0.0
        )
        viability_delta = final_chosen_viability - short_term_best_viability
        corridor_before_int = int(corridor_before) if corridor_before is not None else -1
        corridor_after_int = int(corridor_after) if corridor_after is not None else -1

        # ---- v0.6.0 Predictive Corridor Selector ------------------------------
        # The v0.5.2 safety decision above is finished and preserved.  The
        # selector only re-orders the candidates that *already* passed the hard
        # safety filter (feasible) and the existing critical tier, using future
        # corridor structure.  OFF by default: then nothing below runs at all.
        base_index_for_selector = int(chosen_index)
        if self.fsc_selector_enabled and feasible.size:
            if self._fsc_shadow is None:
                self._fsc_shadow = SafeCorridorShadow(enabled=True)
            t_table = time.perf_counter()
            table = build_future_space_table(
                self._fsc_shadow,
                scene,
                candidates,
                feasible.tolist(),
                ctx,
                anchors=(ltv.anchors if ltv is not None else None),
            )
            table_us = (time.perf_counter() - t_table) * 1e6
        else:
            table = None
            table_us = 0.0
        selector = select_corridor_candidate(
            enabled=bool(self.fsc_selector_enabled),
            feasible=feasible.tolist(),
            base_index=base_index_for_selector,
            critical_band=critical_band,
            table=table,
            fes=fes_arr,
            behav=0.5 * (smooth_arr + dwell_arr),
            task=task_arr,
            risk_band=risk_band,
        )
        if selector.applied:
            chosen_index = int(selector.index)
            chosen = candidates[chosen_index]
            self._last_action = chosen
            switched = held is None or int(chosen_index) != int(held)
            chosen_viability = (
                float(ltv.viability[chosen_index]) if ltv is not None else 0.0
            )
            final_chosen_viability = chosen_viability
            reason = (
                f"{reason}+fsc:{selector.level}" if reason else f"fsc:{selector.level}"
            )

        self.last_diagnostics = PlanDiagnostics(
            used_emergency=used_emergency,
            n_feasible=int(feasible.size),
            chosen_index=int(chosen_index),
            infeasible=infeasible,
            version=VERSION,
            build_id=BUILD_ID,
            chosen_action=chosen,
            chosen_risk_peak=float(profile.risk_peak[chosen_index]),
            chosen_risk_trend=float(profile.risk_trend[chosen_index]),
            chosen_min_ttc=float(profile.min_ttc[chosen_index]),
            chosen_min_clearance=float(profile.min_clearance[chosen_index]),
            chosen_fes=float(fes_arr[chosen_index]),
            critics={
                "safety": 0.0,
                "critical": self.critics.critical * float(critical_score[chosen_index]),
                "future_risk": self.critics.risk * float(risk_score[chosen_index]),
                "fes": self.critics.fes * float(fes_arr[chosen_index]),
                "smooth": self.critics.behavior * 0.5 * float(smooth_arr[chosen_index]),
                "dwell": self.critics.behavior * 0.5 * float(dwell_arr[chosen_index]),
                "progress": self.critics.task * float(task_arr[chosen_index]),
            },
            critical_count=int((envelope.level == 1).sum()),
            chosen_safety_margin=float(profile.min_margin[chosen_index]),
            chosen_closing_rate=float(profile.closing_rate[chosen_index]),
            chosen_action_switched=bool(switched),
            switch_reason=reason,
            refinement_used=refinement_used,
            refinement_count=refinement_count,
            viability_skipped=bool(viability_skipped),
            skip_reason=skip_reason,
            viability_run_reason=run_reason,
            viability_evaluated=int(ltv.evaluated) if ltv is not None else 0,
            long_anchors=(ltv.anchors if ltv is not None else None),
            chosen_viability=chosen_viability,
            chosen_safe_sector_count=chosen_safe,
            chosen_corridor_min_clearance=chosen_cmin,
            chosen_corridor_width=chosen_cwidth,
            chosen_corridor_direction=chosen_cdir,
            chosen_corridor_persistence=int(persistence),
            short_term_us=(t_short - t_start) * 1e6,
            viability_us=(t_ltv - t_short) * 1e6,
            total_us=(t_end - t_start) * 1e6,
            short_term_best_index=int(short_term_best_index),
            short_term_best_action=short_term_best_action,
            short_term_best_risk=short_term_best_risk,
            short_term_best_fes=short_term_best_fes,
            final_chosen_index=int(chosen_index),
            final_chosen_action=chosen,
            ltv_override=ltv_override,
            ltv_override_reason=(reason if ltv_override else ""),
            short_term_best_viability=short_term_best_viability,
            final_chosen_viability=final_chosen_viability,
            viability_override_risk_delta=risk_delta,
            viability_override_viability_delta=viability_delta,
            corridor_before=corridor_before_int,
            corridor_after=corridor_after_int,
            corridor_switched=bool(corridor_switched),
            corridor_switch_kind=corridor_kind,
            emergency=bool(used_emergency),
            enable_ltv=bool(self.enable_ltv),
            decision_reason=reason,
            ltv_shadow=bool(self.ltv_shadow),
            ltv_guard_enabled=bool(self.viability_cfg.guard_enabled),
            ltv_proposed_index=int(ltv_proposed_index),
            risk_band_delta=int(risk_band_delta),
            long_term_degraded=bool(long_term_degraded),
            override_guard_reason=guard_reason,
            override_candidate_count=int(guard_counts["candidates"]),
            override_blocked_by_risk_band=int(guard_counts["risk_band"]),
            override_blocked_by_viability_margin=int(guard_counts["viability_margin"]),
            override_blocked_by_long_term_degraded=int(
                guard_counts["long_term_degraded"]
            ),
            override_blocked_by_critical_band=int(guard_counts["critical_band"]),
            override_blocked_by_no_reference=int(guard_counts["no_reference"]),
            fsc_candidate_indices=fsc_candidate_indices,
            fsc_candidate_actions=fsc_candidate_actions,
            fsc_candidate_sources=fsc_candidate_sources,
            fsc_candidate_source=fsc_candidate_source,
            fsc_selector_enabled=bool(selector.enabled),
            fsc_selector_applied=bool(selector.applied),
            fsc_selector_base_index=int(selector.base_index),
            fsc_selector_index=int(selector.index),
            fsc_selector_level=selector.level,
            fsc_selector_reason=selector.reason,
            fsc_hard_survivors=int(selector.hard_survivors),
            fsc_eligible=int(selector.eligible),
            fsc_unknown_candidates=int(selector.unknown_candidates),
            fsc_triggered_reachability=bool(selector.triggered_reachability),
            fsc_triggered_persistent=bool(selector.triggered_persistent),
            fsc_triggered_max_run=bool(selector.triggered_max_run),
            fsc_triggered_clearance=bool(selector.triggered_clearance),
            fsc_triggered_existing_tiers=bool(selector.triggered_existing_tiers),
            fsc_base_reachable=bool(selector.base_reachable),
            fsc_winner_reachable=bool(selector.winner_reachable),
            fsc_base_persistent=int(selector.base_persistent),
            fsc_winner_persistent=int(selector.winner_persistent),
            fsc_base_max_run_t3=int(selector.base_max_run_t3),
            fsc_winner_max_run_t3=int(selector.winner_max_run_t3),
            fsc_table_us=float(table_us),
        )

        if self.ltv_debug:  # pragma: no cover - diagnostics only
            print(
                f"[LTV] enable={self.enable_ltv} shadow={self.ltv_shadow}"
                f" guard={self.viability_cfg.guard_enabled}"
                f" skipped={viability_skipped}"
                f"({skip_reason or run_reason}) short_best={short_term_best_index}"
                f" proposed={ltv_proposed_index} final={chosen_index}"
                f" override={ltv_override} reason={reason}"
                f" guard_reason={guard_reason or '-'}"
                f" topk={fsc_candidate_indices}({fsc_candidate_source})"
                f" risk_delta={risk_delta:+.4f} band_delta={risk_band_delta:+d}"
                f" viability_delta={viability_delta:+.4f}"
                f" degraded={long_term_degraded}"
                f" corridor={corridor_before_int}->{corridor_after_int}({corridor_kind})"
                f" emergency={used_emergency}"
            )

        if self.debug:  # pragma: no cover - debug aid
            print(
                f"[{VERSION} | build {BUILD_ID.split('-')[-1]}] "
                f"chosen={chosen_index} emergency={used_emergency} "
                f"feasible={feasible.size}/{len(candidates)} "
                f"level={int(envelope.level[chosen_index])} "
                f"margin={profile.min_margin[chosen_index]:.2f} "
                f"closing={profile.closing_rate[chosen_index]:.1f} "
                f"risk_peak={profile.risk_peak[chosen_index]:.3f} "
                f"trend={profile.risk_trend[chosen_index]:+.3f} "
                f"refine={refinement_count} switch={reason}"
            )

        return chosen

    # ------------------------------------------------------------------
    def _far(
        self,
        scene: SceneState,
        candidates: Sequence[Action],
        horizon: float,
        safety,
    ):
        cfg = self.envelope_cfg
        return future_action_risk_batch(
            scene,
            candidates,
            horizon,
            ttc_reference=self.cost.ttc_reference,
            safety=safety,
            base_margin=cfg.base_margin,
            margin_time=cfg.margin_time,
            critical_margin=cfg.critical_margin,
            lateral_rate_threshold_frac=cfg.lateral_rate_threshold_frac,
            lateral_rate_scale_frac=cfg.lateral_rate_scale_frac,
            trend_gain=self.critics.trend_gain,
        )

    def _critical_score(
        self, profile, envelope: SafetyEnvelope
    ) -> np.ndarray:
        """LEVEL 2 critical dynamic safety score (envelope + gate penalty)."""
        gate_metrics = SafetyMetrics(
            min_clearance=profile.min_clearance, ttc_min=profile.min_ttc
        )
        penalty = self.safety_gate.critical_penalty(gate_metrics)
        cw = self.safety_gate.critical_weight
        penalty_normalised = penalty / cw if cw > 0.0 else np.zeros_like(penalty)
        return critical_risk_critic(
            profile, envelope, critical_penalty=penalty_normalised
        )

    def _risk_score(
        self, profile, candidates: Sequence[Action], max_speed: float
    ) -> np.ndarray:
        """LEVEL 3 future-risk score (candidate-conditioned)."""
        return future_risk_critic(
            profile,
            candidates,
            max_speed,
            ttc_reference=self.cost.ttc_reference,
            near_miss_threshold=self.safety_gate.near_miss_threshold,
            trend_gain=self.critics.trend_gain,
            speed_risk_gain=self.critics.speed_risk_gain,
        )

    def _needs_refinement(
        self, profile, envelope: SafetyEnvelope, candidates, critical_score, risk_score
    ) -> bool:
        """Danger-only trigger for local refinement (low risk returns False)."""
        cfg = self.refinement_cfg
        if not cfg.enabled or cfg.max_extra == 0:
            return False
        feasible = np.flatnonzero(~envelope.unsafe)
        if feasible.size == 0:
            return False
        provisional = self._provisional_index(
            envelope, critical_score, risk_score, candidates
        )
        if provisional is None:
            return False
        if int(feasible.size) <= cfg.feasible_threshold:
            return True
        if float(risk_score[provisional]) > cfg.risk_threshold:
            return True
        return (
            float(envelope.min_margin[provisional]) < self.envelope_cfg.critical_margin
        )

    @staticmethod
    def _provisional_index(
        envelope: SafetyEnvelope,
        critical_score: np.ndarray,
        risk_score: np.ndarray,
        candidates: Sequence[Action],
    ) -> int | None:
        """Best *non-stop* feasible direction, for choosing where to refine.

        Ordered by the real critic tiers (critical, then future risk) and only
        then by the *largest* safety margin, so refinement is centred on the
        genuinely best escape direction - not on the most dangerous one.  Cheap
        O(B) scan; no utility cost and no trajectory rollout.
        """
        best: int | None = None
        best_key: tuple | None = None
        for b in np.flatnonzero(~envelope.unsafe):
            b = int(b)
            if candidates[b].is_stop:
                continue
            key = (
                float(critical_score[b]),
                float(risk_score[b]),
                -float(envelope.min_margin[b]),  # largest margin first
                int(b),
            )
            if best_key is None or key < best_key:
                best_key, best = key, b
        return best

    def _refinement_actions(
        self,
        candidates: Sequence[Action],
        provisional: int | None,
        max_speed: float,
    ) -> tuple[Action, ...]:
        """<= ``max_extra`` temporary actions around the provisional heading."""
        if provisional is None:
            return ()
        base = candidates[provisional]
        base_deg = math.degrees(base.steering_angle)
        speeds = []
        for level in SPEED_LEVELS:
            speed = float(max_speed) * level
            if abs(speed - base.speed) > 1e-9:
                speeds.append(speed)
        speeds.append(base.speed)  # refine the chosen speed first
        extra: list[Action] = []
        for offset in self.refinement_cfg.offsets_deg:
            for speed in speeds:
                if len(extra) >= self.refinement_cfg.max_extra:
                    return tuple(extra)
                extra.append(Action(speed, math.radians(base_deg + offset)))
        return tuple(extra)

    def _apply_hysteresis(
        self,
        candidates: Sequence[Action],
        total: np.ndarray,
        critical_score: np.ndarray,
        risk_score: np.ndarray,
        profile,
        best_index: int,
    ) -> tuple[int, bool, str]:
        """Safety-first hysteresis: hold the previous action unless clearly worse.

        Returns ``(chosen_index, switched, reason)``.
        """
        cfg = self.hysteresis_cfg
        if not cfg.enabled or self._last_action is None:
            return best_index, True, "no_hysteresis"
        held = self._find_action(candidates, self._last_action)
        if held is None:
            return best_index, True, "held_not_in_set"
        if held == best_index:
            return held, False, "same_as_held"
        if not np.isfinite(total[held]):
            return best_index, True, "held_infeasible"
        if profile.envelope_level[held] >= 2:
            return best_index, True, "held_unsafe"
        if profile.envelope_level[held] == 1:
            return best_index, True, "held_critical"
        if float(profile.risk_trend[held]) > cfg.trend_threshold:
            return best_index, True, "held_trend_worsening"
        if float(critical_score[held]) - float(critical_score[best_index]) > cfg.critical_advantage:
            return best_index, True, "clear_critical_gain"
        if float(risk_score[held]) - float(risk_score[best_index]) > cfg.risk_advantage:
            return best_index, True, "clear_risk_gain"
        if float(total[held]) - float(total[best_index]) > cfg.switch_margin:
            return best_index, True, "clear_gain"
        return held, False, "hysteresis_hold"

    # ------------------------------------------------------------------
    def _default_context(self, scene: SceneState) -> ViabilityContext:
        """Fallback context when the caller supplies none.

        ``scene_scale=None`` makes anchors fall back to ratios scaled by
        ``max_long_horizon``; the adapter/controller should pass a real scale.
        """
        return ViabilityContext(
            short_horizon=scene.horizon,
            scene_scale=None,
            base_buffer=self.envelope_cfg.base_margin,
            margin_time=self.envelope_cfg.margin_time,
        )

    def _fast_path(
        self,
        scene: SceneState,
        candidates: Sequence[Action],
        feasible: np.ndarray,
        envelope: SafetyEnvelope,
        risk_score: np.ndarray,
        critical_band: np.ndarray,
        context: ViabilityContext,
    ) -> tuple[bool, str, str]:
        """Low-threat fast path: skip LTV only if ALL five conditions hold."""
        cfg = self.viability_cfg
        ranked = sorted(feasible.tolist(), key=lambda i: (float(risk_score[i]), i))
        best = int(ranked[0])
        second = int(ranked[1]) if len(ranked) > 1 else None
        openness = self.viability.current_openness(scene, context)
        diff = (
            float("inf")
            if second is None
            else float(risk_score[second]) - float(risk_score[best])
        )
        checks = (
            (
                "low_risk",
                int(math.floor(float(risk_score[best]) / cfg.risk_band_width))
                <= cfg.fast_risk_band_max,
            ),
            ("no_critical", int(envelope.level[best]) == 0),
            ("margin", float(envelope.min_margin[best]) >= cfg.fast_margin_min),
            ("openness", float(openness) >= cfg.fast_openness_min),
            ("clear_choice", diff >= cfg.fast_decision_gap),
        )
        for name, ok in checks:
            if not ok:
                return False, "", f"fast_path_unmet:{name}"
        return True, "fast_path:low_risk+no_critical+margin+openness+clear_choice", ""

    @staticmethod
    def _bucketed_select(
        feasible: np.ndarray,
        critical_band: np.ndarray,
        risk_band: np.ndarray,
        viability_cost: np.ndarray,
        fes_arr: np.ndarray,
        behav_arr: np.ndarray,
        task_arr: np.ndarray,
    ) -> int:
        """Lexicographic bucket key: critical band > risk band > viability > ..."""
        best = int(feasible[0])
        best_key: tuple | None = None
        for b in feasible:
            b = int(b)
            key = (
                int(critical_band[b]),
                int(risk_band[b]),
                float(viability_cost[b]),
                float(fes_arr[b]),
                float(behav_arr[b]),
                float(task_arr[b]),
                b,
            )
            if best_key is None or key < best_key:
                best_key, best = key, b
        return best

    def _apply_viability_hysteresis(
        self,
        candidates: Sequence[Action],
        feasible_mask: np.ndarray,
        critical_band: np.ndarray,
        risk_band: np.ndarray,
        viability_cost: np.ndarray,
        envelope: SafetyEnvelope,
        best_index: int,
    ) -> tuple[int, bool, str]:
        """Safety-first hysteresis inside the bucketed (LTV-active) path."""
        cfg = self.hysteresis_cfg
        if not cfg.enabled or self._last_action is None:
            return best_index, True, "no_hysteresis"
        held = self._find_action(candidates, self._last_action)
        if held is None:
            return best_index, True, "held_not_in_set"
        if held == best_index:
            return held, False, "same_as_held"
        if not bool(feasible_mask[held]):
            return best_index, True, "held_infeasible"
        if int(envelope.level[held]) >= 2:
            return best_index, True, "held_unsafe"
        if int(envelope.level[held]) == 1:
            return best_index, True, "held_critical"
        if int(critical_band[held]) != int(critical_band[best_index]):
            return best_index, True, "held_band_worse"
        if int(risk_band[held]) != int(risk_band[best_index]):
            return best_index, True, "held_risk_band_worse"
        gain = float(viability_cost[held]) - float(viability_cost[best_index])
        if gain > cfg.risk_advantage:
            return best_index, True, "clear_viability_gain"
        return held, False, "hysteresis_hold"

    def _corridor_hysteresis(
        self,
        feasible: np.ndarray,
        chosen_index: int,
        ltv: ViabilityProfile,
    ) -> tuple[int, bool, str]:
        """Stabilise the discrete local escape corridor class (DLEC).

        Switch only when the new corridor's viability exceeds the previous
        corridor's by ``corridor_switch_threshold``; if the previous corridor has
        no feasible candidate (hard unsafe / gone) the switch is immediate.
        Safety > hysteresis.

        Returns ``(index, switched, kind)`` where ``kind`` is one of
        ``""`` (same corridor), ``"hold"`` (switch refused),
        ``"switch_viability"`` or ``"switch_hard_unsafe"``.
        """
        cfg = self.viability_cfg
        cid = int(ltv.corridor_id[chosen_index])
        prev = self._last_corridor_id
        if prev is None or cid < 0 or cid == prev:
            self._last_corridor_id = cid if cid >= 0 else prev
            return chosen_index, False, ""
        cand_prev: int | None = None
        best_key: tuple | None = None
        for b in feasible:
            b = int(b)
            if not bool(ltv.shortlist[b]):
                continue
            if int(ltv.corridor_id[b]) != prev:
                continue
            key = (float(ltv.viability_cost[b]), b)
            if best_key is None or key < best_key:
                best_key, cand_prev = key, b
        if cand_prev is None:
            self._last_corridor_id = cid
            return chosen_index, True, "switch_hard_unsafe"
        if (
            float(ltv.viability[chosen_index]) - float(ltv.viability[cand_prev])
            > cfg.corridor_switch_threshold
        ):
            self._last_corridor_id = cid
            return chosen_index, True, "switch_viability"
        self._last_corridor_id = prev
        return cand_prev, False, "hold"

    # ------------------------------------------------------------------
    def _long_term_degraded(
        self, ltv: ViabilityProfile, index: int, base: int
    ) -> bool:
        """Condition E: is ``index`` worse than ``base`` in the long term?

        A candidate is long-term degraded when it does not improve the aggregate
        viability, or when its escape corridor has less clearance than the
        reference's by more than ``long_term_degraded_margin``.  Such a candidate
        may not win a decision on one lucky anchor.
        """
        idx, ref = int(index), int(base)
        if float(ltv.viability[idx]) + _EPS < float(ltv.viability[ref]):
            return True
        margin = float(self.viability_cfg.long_term_degraded_margin)
        return float(ltv.corridor_min_clearance[idx]) + margin < float(
            ltv.corridor_min_clearance[ref]
        )

    def _guard_override(
        self,
        feasible: np.ndarray,
        critical_band: np.ndarray,
        risk_band: np.ndarray,
        ltv: ViabilityProfile,
        base_index: int,
        proposed_index: int,
    ) -> tuple[int, str, dict]:
        """v0.5.2 override guard: LTV may not freely overturn v0.4.1.

        Conditions (A and F are enforced upstream -- the proposal is always a
        feasible candidate and never an emergency):

            B. critical band not worse than the reference;
            C. risk band *equal* to the reference (never cross a risk bucket);
            D. viability gain over the reference >= improvement threshold;
            E. not long-term degraded.

        Returns ``(index, blocked_reason, counts)``.  ``blocked_reason == ""``
        means the override stands and ``index`` is the LTV proposal; otherwise
        the proposal was vetoed and ``index`` is the v0.4.1 short-term best.
        """
        counts = {
            "candidates": 0,
            "critical_band": 0,
            "risk_band": 0,
            "viability_margin": 0,
            "long_term_degraded": 0,
            "no_reference": 0,
        }
        base = int(base_index)
        if base < 0:
            return int(proposed_index), "", counts
        # Without an LTV evaluation of the v0.4.1 choice there is no "clear
        # advantage" to establish, so nothing is allowed to override.
        if not bool(ltv.shortlist[base]):
            counts["no_reference"] = 1
            return base, "no_ltv_reference", counts

        base_critical = int(critical_band[base])
        base_band = int(risk_band[base])
        base_viability = float(ltv.viability[base])
        threshold = float(self.viability_cfg.viability_improvement_threshold)

        # Diagnostics only: classify each LTV-eligible alternative by the first
        # guard condition it fails (association statistics, never control input).
        for b in feasible:
            b = int(b)
            if b == base or not bool(ltv.shortlist[b]):
                continue
            counts["candidates"] += 1
            if int(critical_band[b]) > base_critical:
                counts["critical_band"] += 1
                continue
            if int(risk_band[b]) != base_band:
                counts["risk_band"] += 1
                continue
            if self._long_term_degraded(ltv, b, base):
                counts["long_term_degraded"] += 1
                continue
            if float(ltv.viability[b]) - base_viability < threshold:
                counts["viability_margin"] += 1
                continue

        proposed = int(proposed_index)
        if proposed == base:
            return base, "", counts
        if int(critical_band[proposed]) > base_critical:
            return base, "critical_band", counts
        if int(risk_band[proposed]) != base_band:
            return base, "risk_band", counts
        if self._long_term_degraded(ltv, proposed, base):
            return base, "long_term_degraded", counts
        if float(ltv.viability[proposed]) - base_viability < threshold:
            return base, "viability_margin", counts
        return proposed, "", counts

    @staticmethod
    def _find_action(candidates: Sequence[Action], action: Action) -> int | None:
        """Index of an equivalent candidate (speed + heading within tolerance)."""
        for i, candidate in enumerate(candidates):
            if (
                abs(candidate.speed - action.speed) <= 1e-9
                and abs(candidate.steering_angle - action.steering_angle) <= 1e-9
            ):
                return i
        return None

    def _tier_scores(self, terms: UtilityTerms) -> tuple[float, float, float, float]:
        """Normalise utility terms into ``[0, 1]`` scores (FES/smooth/dwell/task)."""
        fes = float(np.clip(terms.trap / max(self.cost.w_trap, _EPS), 0.0, 1.0))
        smooth = float(
            np.clip(terms.smooth / max(2.0 * self.cost.w_smooth, _EPS), 0.0, 1.0)
        )
        dwell = float(np.clip(terms.dwell / max(self.cost.w_dwell, _EPS), 0.0, 1.0))
        if self.cost.w_progress == 0.0:
            task = 0.0
        else:
            progress_fraction = terms.progress / self.cost.w_progress  # in [-1, 1]
            task = float(np.clip(0.5 * (1.0 - progress_fraction), 0.0, 1.0))
        return fes, smooth, dwell, task

    @staticmethod
    def _trajectory(batch: TrajectoryBatch, index: int) -> Trajectory:
        """View candidate ``index`` of a batch as a single :class:`Trajectory`."""
        return Trajectory(
            agent_positions=batch.agent_positions[index],
            agent_velocity=batch.agent_velocities[index],
            agent_max_speed=batch.agent_max_speed,
            obstacle_positions=batch.obstacle_positions,
            obstacle_velocities=batch.obstacle_velocities,
            obstacle_radii=batch.obstacle_radii,
            agent_radius=batch.agent_radius,
        )

    @staticmethod
    def _emergency_index(ttc_min: np.ndarray, min_clearance: np.ndarray) -> int:
        """Deterministic safest colliding candidate (used when none is feasible).

        Primary: largest predicted minimum TTC (``inf`` counts as largest);
        secondary: largest minimum clearance; final: lowest candidate index.
        """
        return min(
            range(len(ttc_min)),
            key=lambda i: (-float(ttc_min[i]), -float(min_clearance[i]), i),
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _estimate_min_ttc(scene: SceneState) -> float:
        """Minimum time-to-collision at the current state (``inf`` = no threat).

        Pure numpy, same sign convention as :mod:`cpu.decision.cost`
        (``closing > 0`` means the obstacle is approaching).  This state-level
        value only drives the adaptive horizon / adaptive smoothness; the FAR
        critic uses candidate-conditioned quantities instead.
        """
        obstacles = scene.obstacles
        if not obstacles:
            return float("inf")

        agent = scene.agent
        positions = np.stack([o.position for o in obstacles])   # (N, 2)
        velocities = np.stack([o.velocity for o in obstacles])  # (N, 2)
        radii = np.array([o.radius for o in obstacles])         # (N,)

        delta = agent.position[None, :] - positions             # (N, 2) agent - obstacle
        dist = np.hypot(delta[:, 0], delta[:, 1])               # (N,)
        clearance = dist - (agent.radius + radii)               # (N,)
        rel_vel = velocities - agent.velocity[None, :]          # (N, 2)
        unit = delta / np.maximum(dist, 1e-6)[:, None]          # (N, 2) obstacle -> agent
        closing = np.maximum((rel_vel * unit).sum(axis=-1), 0.0)  # (N,)

        if float(closing.max()) <= 0.0:
            return float("inf")
        ttc = np.maximum(clearance, 0.0) / np.where(closing > 0.0, closing, np.inf)
        return float(np.where(closing > 0.0, ttc, np.inf).min())
