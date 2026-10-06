"""CPU decision architecture.

    SceneState --CpuDecisionLayer.decide()--> Action

The decision core is game-agnostic; the simulator-side mapping (a
``WorldSnapshot`` into a :class:`SceneState`) lives in
:mod:`cpu.adapters.game_adapter`.
"""

from cpu.decision.build import (
    BUILD_ID,
    PREDICTIVE_VERSION,
    REACTIVE_VERSION,
    STACK_VERSION,
    VERSION,
)
from cpu.decision.core import CpuDecisionLayer
from cpu.decision.cost import SafetyMetrics, TrajectoryCost, UtilityTerms
from cpu.decision.far import (
    CandidateRiskProfile,
    CandidateSafety,
    SafetyEnvelope,
    adaptive_safety_envelope,
    candidate_safety_batch,
    critical_risk_critic,
    future_action_risk_batch,
    future_risk_critic,
)
from cpu.decision.immediate import (
    SECTOR_DEGREES,
    SectorSummary,
    gap_scores,
    radial_closing_rate,
    sectorize,
    surface_clearance,
)
from cpu.decision.models import Action, AgentState, Obstacle, SceneState
from cpu.decision.planner import (
    ENABLE_LTV,
    LTV_DEBUG_DIAGNOSTICS,
    LTV_OVERRIDE_GUARD,
    LTV_SHADOW_ONLY,
    ActionPlanner,
    HierarchicalCritics,
    HysteresisConfig,
    PlanDiagnostics,
    RefinementConfig,
    SafetyEnvelopeConfig,
    SafetyGate,
    default_candidate_actions,
)
from cpu.decision.corridor_selector import (
    FSC_SELECTOR_ENABLED,
    FutureSpaceTable,
    SelectorDecision,
    select_corridor_candidate,
)
from cpu.decision.predictor import LinearPredictor, Predictor
from cpu.decision.reactive import (
    ReactiveConfig,
    ReactiveDiagnostics,
    ReactiveGapPlanner,
)
from cpu.decision.risk import RiskEvaluator
from cpu.decision.viability import (
    FutureMobility,
    LongTermViabilityEvaluator,
    ViabilityConfig,
    ViabilityContext,
    ViabilityProfile,
    direction_class,
)

__all__ = [
    "CpuDecisionLayer",
    "AgentState",
    "Obstacle",
    "Action",
    "SceneState",
    "Predictor",
    "LinearPredictor",
    "RiskEvaluator",
    "TrajectoryCost",
    "SafetyMetrics",
    "UtilityTerms",
    "ActionPlanner",
    "SafetyGate",
    "HierarchicalCritics",
    "SafetyEnvelopeConfig",
    "RefinementConfig",
    "HysteresisConfig",
    "PlanDiagnostics",
    "CandidateSafety",
    "CandidateRiskProfile",
    "SafetyEnvelope",
    "candidate_safety_batch",
    "future_action_risk_batch",
    "future_risk_critic",
    "critical_risk_critic",
    "adaptive_safety_envelope",
    "default_candidate_actions",
    "SECTOR_DEGREES",
    "SectorSummary",
    "sectorize",
    "gap_scores",
    "radial_closing_rate",
    "surface_clearance",
    "ReactiveGapPlanner",
    "ReactiveConfig",
    "ReactiveDiagnostics",
    "LongTermViabilityEvaluator",
    "ViabilityConfig",
    "ViabilityContext",
    "ViabilityProfile",
    "FutureMobility",
    "direction_class",
    "ENABLE_LTV",
    "LTV_DEBUG_DIAGNOSTICS",
    "LTV_SHADOW_ONLY",
    "LTV_OVERRIDE_GUARD",
    "FSC_SELECTOR_ENABLED",
    "FutureSpaceTable",
    "SelectorDecision",
    "select_corridor_candidate",
    "STACK_VERSION",
    "REACTIVE_VERSION",
    "PREDICTIVE_VERSION",
    "VERSION",
    "BUILD_ID",
]
