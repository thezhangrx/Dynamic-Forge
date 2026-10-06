"""Adapters between simulator (game) state and the abstract decision models.

    WorldSnapshot --snapshot_to_scene()--> SceneState
    Action       --to_game_action()-----> simulator Action
"""

from cpu.adapters.game_adapter import (
    action_to_velocity,
    observation_to_scene,
    snapshot_to_scene,
    to_game_action,
)

__all__ = [
    "snapshot_to_scene",
    "observation_to_scene",
    "action_to_velocity",
    "to_game_action",
]
