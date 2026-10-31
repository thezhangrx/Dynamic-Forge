"""Bullet-Hell Simulator Platform.

A reusable 2D dynamic-environment simulator for:
  virtual training -> CPU/FPGA heterogeneous deployment -> real 2D robot validation.

Design priorities: correctness > reproducibility > modularity > data interface >
performance > UI aesthetics.

The core is deliberately free of any rendering, AI, or game-specific logic.
"""

from bullet_sim.core.version import (
    DATASET_SCHEMA_VERSION,
    STATE_PROTOCOL_VERSION,
    __version__,
)
from bullet_sim.core.actions import (
    DISCRETE_ACTION_NAMES,
    ActionCodec,
    ActionContext,
    AccelerationActionCodec,
    DiscreteActionCodec,
    VelocityActionCodec,
    codec_from_spec,
)
from bullet_sim.core.state import WorldSnapshot
from bullet_sim.scenarios.presets import scenario, stress
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.world import World

__all__ = [
    "__version__",
    "STATE_PROTOCOL_VERSION",
    "DATASET_SCHEMA_VERSION",
    # core
    "WorldSnapshot",
    "ActionCodec",
    "ActionContext",
    "DiscreteActionCodec",
    "VelocityActionCodec",
    "AccelerationActionCodec",
    "codec_from_spec",
    "DISCRETE_ACTION_NAMES",
    # scenarios
    "ScenarioSpec",
    "scenario",
    "stress",
    # simulation
    "World",
    "BulletHellEnv",
]
