"""Unified CPU decision facade.

This is the single entry point the rest of the system uses::

    state: SceneState  ->  CpuDecisionLayer.decide(state)  ->  Action

``state`` is the *abstract* scene (see :mod:`cpu.decision.models`);
the game-specific mapping lives in
:mod:`cpu.adapters.game_adapter`, so the decision core never
imports the simulator or the world.
"""

from __future__ import annotations

from typing import Any

from cpu.decision.cost import TrajectoryCost
from cpu.decision.models import Action, SceneState
from cpu.decision.planner import BUILD_ID, VERSION, ActionPlanner
from cpu.decision.predictor import LinearPredictor, Predictor
from cpu.decision.viability import ViabilityContext

__all__ = ["CpuDecisionLayer"]


class CpuDecisionLayer:
    """Pure decision core: abstract scene in, abstract action out.

    Internally it is a thin wrapper over :class:`ActionPlanner`, which composes
    a :class:`Predictor` (linear, for now), the FAR/ASE safety stack,
    :class:`TrajectoryCost` (FES / behaviour / task), the v0.5.1 Long-Term
    Viability evaluator, a :class:`SafetyGate` and a dynamic horizon.
    """

    name = "cpu-decision"

    def __init__(
        self,
        predictor: Predictor | None = None,
        cost: TrajectoryCost | None = None,
        planner: ActionPlanner | None = None,
    ) -> None:
        self.predictor = predictor or LinearPredictor()
        self.cost = cost or TrajectoryCost()
        self.planner = planner or ActionPlanner(self.predictor, self.cost)

    def decide(
        self, state: SceneState, *, context: ViabilityContext | None = None
    ) -> Action:
        """Return the best :class:`Action` for ``state``.

        ``context`` is the optional Long-Term Viability context (short horizon +
        scene scale); the adapter/controller that knows the field supplies it.
        """
        return self.planner.plan(state, context=context)

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "version": VERSION,
            "build_id": BUILD_ID,
            "enable_ltv": self.planner.enable_ltv,
            "ltv_debug": self.planner.ltv_debug,
            "ltv_shadow": self.planner.ltv_shadow,
            "predictor": type(self.predictor).__name__,
            "cost": type(self.cost).__name__,
            "weights": {
                "collision": self.cost.w_collision,
                "ttc": self.cost.w_ttc,
                "smooth": self.cost.w_smooth,
                "progress": self.cost.w_progress,
                "dwell": self.cost.w_dwell,
                "trap": self.cost.w_trap,
            },
            "safety_gate": {
                "collision_threshold": self.planner.safety_gate.collision_threshold,
                "near_miss_threshold": self.planner.safety_gate.near_miss_threshold,
                "critical_weight": self.planner.safety_gate.critical_weight,
            },
            "critics": {
                "critical": self.planner.critics.critical,
                "risk": self.planner.critics.risk,
                "fes": self.planner.critics.fes,
                "behavior": self.planner.critics.behavior,
                "task": self.planner.critics.task,
                "trend_gain": self.planner.critics.trend_gain,
                "speed_risk_gain": self.planner.critics.speed_risk_gain,
            },
            "envelope": {
                "base_margin": self.planner.envelope_cfg.base_margin,
                "margin_time": self.planner.envelope_cfg.margin_time,
                "critical_margin": self.planner.envelope_cfg.critical_margin,
                "lateral_rate_threshold_frac": (
                    self.planner.envelope_cfg.lateral_rate_threshold_frac
                ),
                "lateral_rate_scale_frac": (
                    self.planner.envelope_cfg.lateral_rate_scale_frac
                ),
            },
            "refinement": {
                "enabled": self.planner.refinement_cfg.enabled,
                "risk_threshold": self.planner.refinement_cfg.risk_threshold,
                "feasible_threshold": self.planner.refinement_cfg.feasible_threshold,
                "max_extra": self.planner.refinement_cfg.max_extra,
            },
            "hysteresis": {
                "enabled": self.planner.hysteresis_cfg.enabled,
                "switch_margin": self.planner.hysteresis_cfg.switch_margin,
                "critical_advantage": self.planner.hysteresis_cfg.critical_advantage,
                "risk_advantage": self.planner.hysteresis_cfg.risk_advantage,
                "trend_threshold": self.planner.hysteresis_cfg.trend_threshold,
            },
            "viability": {
                "top_k": self.planner.viability_cfg.top_k,
                "leaders": self.planner.viability_cfg.leaders,
                "diversity": self.planner.viability_cfg.diversity,
                "anchor_ratios": list(self.planner.viability_cfg.anchor_ratios),
                "max_long_horizon": self.planner.viability_cfg.max_long_horizon,
                "clearance_ref": self.planner.viability_cfg.clearance_ref,
                "max_dynamic_buffer": self.planner.viability_cfg.max_dynamic_buffer,
                "risk_band_width": self.planner.viability_cfg.risk_band_width,
                "corridor_switch_threshold": (
                    self.planner.viability_cfg.corridor_switch_threshold
                ),
                "fast_risk_band_max": self.planner.viability_cfg.fast_risk_band_max,
                "fast_margin_min": self.planner.viability_cfg.fast_margin_min,
                "fast_openness_min": self.planner.viability_cfg.fast_openness_min,
                "fast_decision_gap": self.planner.viability_cfg.fast_decision_gap,
            },
            "horizon": self.planner.horizon,
            "danger_horizon": self.planner.danger_horizon,
            "danger_threshold": self.planner.danger_threshold,
            "ltv_override_guard": {
                "enabled": self.planner.viability_cfg.guard_enabled,
                "viability_improvement_threshold": (
                    self.planner.viability_cfg.viability_improvement_threshold
                ),
                "long_term_degraded_margin": (
                    self.planner.viability_cfg.long_term_degraded_margin
                ),
                "policy": "same risk band + critical not worse + viability margin",
            },
        }
