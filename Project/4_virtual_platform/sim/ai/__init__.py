"""Autonomous controllers: ``Observation -> Action`` without a human.

This package is the second control mode of the platform:

    Mode A  manual   Keyboard      -> Action -> Simulator
    Mode B  autonomous  Observation -> Controller -> Action -> Simulator
    Mode C  hardware Board         -> Adapter -> Action -> Simulator

Every controller here is an ``ActionProvider`` (see
``bullet_sim.interface.controller``), so it plugs into the same
``ControllerSource -> Action -> World.step`` path as a human or a board.  No
controller may write environment state directly.

Three tiers are deliberately kept apart, and the README must not blur them:

======================  ==============================================
``Baseline``            rule-based / predictive, ships with the repo
``Model``               a *trained* policy; ``[MODEL_NOT_AVAILABLE_YET]``
                        until someone actually trains one
``Hardware``            development board, ``[HARDWARE_INTERFACE_TBD]``
======================  ==============================================
"""

from bullet_sim.ai.base import AutonomousController, ControllerSpec
from bullet_sim.ai.factory import PlaySource, make_auto_source, make_play_source
from bullet_sim.ai.baseline import (
    BASELINE_CONTROLLERS,
    IdleController,
    RandomWalkController,
    RepulsionController,
    RolloutPlanner,
    ThreatFieldController,
    make_controller,
)
from bullet_sim.ai.policy import (
    MODEL_NOT_AVAILABLE_YET,
    ModelNotAvailable,
    PolicyRegistry,
    TrainedPolicyController,
    available_policies,
    load_policy,
    register_policy,
)

__all__ = [
    "AutonomousController",
    "ControllerSpec",
    "BASELINE_CONTROLLERS",
    "IdleController",
    "RandomWalkController",
    "RepulsionController",
    "ThreatFieldController",
    "RolloutPlanner",
    "make_controller",
    "PlaySource",
    "make_auto_source",
    "make_play_source",
    "MODEL_NOT_AVAILABLE_YET",
    "ModelNotAvailable",
    "PolicyRegistry",
    "TrainedPolicyController",
    "available_policies",
    "load_policy",
    "register_policy",
]
