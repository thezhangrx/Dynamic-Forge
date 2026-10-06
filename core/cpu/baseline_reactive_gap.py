"""ReactiveGap controller: observation -> one-frame sector decision -> action.

# [v0.5.0-DualPlanner]
# PURPOSE:
#   Expose the game-agnostic :class:`ReactiveGapPlanner` to the observation
#   interface.  This is the *reactive* half of the dual planner: current frame
#   only, no future rollout, no FAR/FES, no candidate-conditioned prediction.
#
# OPEN-SOURCE REFERENCE:
#   F1TENTH Follow-the-Gap / iTTC, PythonRobotics potential field, Nav2
#   anti-wobble deadband - structure only, no code copied.
#
# ALGORITHM:
#   observation -> SceneState (adapter) -> ReactiveGapPlanner.plan -> discrete
#   action.  The adapter is the only game-aware piece; the planner is not.
#
# FPGA MAPPING:
#   The board may stream sector_clearance[8] / sector_closing[8] / density[8];
#   the CPU turns them into the action.
#
# CPU ROLE:
#   Gap selection, goal alignment, hysteresis and the final action are CPU-side.
#
# COMPLEXITY:
#   O(N) per frame (no horizon, no candidate rollout).
"""

from __future__ import annotations

from typing import Any, Mapping

from bullet_sim.ai.base import (
    BaseAutonomousController,
    ControllerSpec,
    direction_to_action,
)
from cpu.adapters.game_adapter import observation_to_scene
from cpu.decision.reactive import (
    REACTIVE_VERSION,
    ReactiveGapPlanner,
)

__all__ = ["ReactiveGapController"]

#: The reactive controller holds no horizon; SceneState only needs a valid dt.
DEFAULT_DT = 1.0 / 120.0


class ReactiveGapController(BaseAutonomousController):
    """One-frame 8-sector Follow-the-Gap controller (observation-only)."""

    SPEC = ControllerSpec(
        name="reactive-gap",
        kind="baseline",
        description=(
            f"{REACTIVE_VERSION}: one-frame 8-sector gap + immediate safety "
            "envelope (no rollout / no FAR / no FES)"
        ),
        access="observation",
        cost="cheap",
        notes="reactive baseline: current frame only; predictive planner is separate",
    )

    def __init__(
        self,
        planner: ReactiveGapPlanner | None = None,
        *,
        dt: float = DEFAULT_DT,
    ) -> None:
        super().__init__()
        self.planner = planner or ReactiveGapPlanner()
        self.dt = float(dt)
        #: Filled by :meth:`act` (a :class:`ReactiveDiagnostics`).
        self.last_diagnostics: Any = None

    def reset(
        self,
        observation: Mapping[str, Any] | None = None,
        info: Mapping[str, Any] | None = None,
    ) -> None:
        super().reset(observation, info)
        self.planner.reset()
        self.last_diagnostics = None

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        self._calls += 1
        # horizon=0.0 is deliberate: the reactive planner has no lookahead.
        scene = observation_to_scene(
            observation, info, horizon=0.0, dt=self.dt, prune=False
        )
        action = self.planner.plan(scene)
        self.last_diagnostics = self.planner.last_diagnostics
        v = action.velocity_vector
        return direction_to_action(float(v[0]), float(v[1]))
