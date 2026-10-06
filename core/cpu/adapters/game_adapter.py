"""Map simulator (game) state <-> the abstract decision models.

The simulator's :class:`~bullet_sim.core.state.WorldSnapshot` (with its SoA
``BulletPool``, the pixel field and the discrete key table) is game-specific;
the decision core only understands :class:`SceneState` / :class:`AgentState` /
:class:`Obstacle` / :class:`Action`.  This module is the **only** place that
knows both, so the decision core stays free of any game import.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bullet_sim.core.state import WorldSnapshot
from cpu.decision.models import (
    Action,
    AgentState,
    Obstacle,
    SceneState,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    from bullet_sim.action.types import Action as GameAction

__all__ = [
    "snapshot_to_scene",
    "observation_to_scene",
    "action_to_velocity",
    "to_game_action",
]

#: Prediction horizon used when the snapshot carries no horizon of its own.
DEFAULT_HORIZON = 0.75

#: Extra surface clearance kept when pruning far obstacles (world units).  An
#: obstacle is kept when it can come within this margin of the agent inside the
#: prediction horizon; everything else is too far to matter within that window.
DANGER_MARGIN = 100.0


def _keep_mask(
    agent_pos: np.ndarray,
    agent_speed: float,
    agent_radius: float,
    positions: np.ndarray,
    velocities: np.ndarray,
    radii: np.ndarray,
    horizon: float,
    margin: float,
) -> np.ndarray:
    """Boolean mask of obstacles that can approach within ``margin`` of the agent.

    ``center - (agent_r + obs_r) - (agent_speed + obs_speed) * horizon < margin``
    keeps every obstacle whose *fastest possible* approach leaves it inside the
    danger margin within the horizon - conservative by construction.
    """
    if positions.size == 0:
        return np.zeros(0, dtype=bool)
    center = np.hypot(positions[:, 0] - agent_pos[0], positions[:, 1] - agent_pos[1])
    o_speed = np.hypot(velocities[:, 0], velocities[:, 1])
    clearance0 = center - (agent_radius + radii)
    approach = (agent_speed + o_speed) * horizon
    return (clearance0 - approach) < margin


def snapshot_to_scene(
    snapshot: WorldSnapshot, *, horizon: float = DEFAULT_HORIZON, prune: bool = False
) -> SceneState:
    """Convert a simulator snapshot into a game-agnostic :class:`SceneState`."""
    p = snapshot.player
    agent = AgentState(
        position=(p.x, p.y),
        velocity=(p.vx, p.vy),
        radius=p.radius,
        max_speed=p.speed,
    )

    idx = snapshot.bullets.active_indices()
    obstacles: list[Obstacle] = []
    if idx.size:
        b = snapshot.bullets.data
        positions = np.stack([b["x"][idx], b["y"][idx]], axis=1).astype(np.float64)
        velocities = np.stack([b["vx"][idx], b["vy"][idx]], axis=1).astype(np.float64)
        radii = b["radius"][idx].astype(np.float64)
        if prune:
            keep = _keep_mask(
                agent.position, agent.max_speed, agent.radius,
                positions, velocities, radii, horizon, DANGER_MARGIN,
            )
            positions, velocities, radii = positions[keep], velocities[keep], radii[keep]
        obstacles = [
            Obstacle(
                position=(float(positions[i, 0]), float(positions[i, 1])),
                velocity=(float(velocities[i, 0]), float(velocities[i, 1])),
                radius=float(radii[i]),
            )
            for i in range(len(radii))
        ]

    t = snapshot.target
    goal = (t.x, t.y) if t is not None else None
    return SceneState(
        agent=agent,
        obstacles=tuple(obstacles),
        goal=goal,
        dt=snapshot.env.dt,
        horizon=horizon,
    )


def observation_to_scene(
    observation: dict,
    info: dict | None = None,
    *,
    horizon: float = DEFAULT_HORIZON,
    dt: float | None = None,
    prune: bool = False,
) -> SceneState:
    """Convert an observation dict (as a controller sees it) into a :class:`SceneState`.

    This is the observation-only counterpart of :func:`snapshot_to_scene`: a
    controller that must stay deployable (no world access) builds its abstract
    scene from the observation it already receives.

    Expected observation layout (see ``interface/space.py``):
    ``player[6] = x,y,vx,vy,radius,speed``, ``target[4] = x,y,radius,dist``,
    ``bullets[n,7] = x,y,vx,vy,radius,ttl,type``.
    """
    player = np.asarray(observation["player"], dtype=np.float64)
    agent = AgentState(
        position=player[0:2],
        velocity=player[2:4],
        radius=float(player[4]),
        max_speed=float(player[5]),
    )

    bullets = observation.get("bullets")
    if bullets is None:
        rows = np.zeros((0, 7), dtype=np.float64)
    else:
        rows = np.asarray(bullets, dtype=np.float64).reshape(-1, 7)
    positions = rows[:, 0:2]
    velocities = rows[:, 2:4]
    radii = rows[:, 4]
    if prune:
        keep = _keep_mask(
            agent.position, agent.max_speed, agent.radius,
            positions, velocities, radii, horizon, DANGER_MARGIN,
        )
        positions, velocities, radii = positions[keep], velocities[keep], radii[keep]
    obstacles = tuple(
        Obstacle(
            position=(float(positions[i, 0]), float(positions[i, 1])),
            velocity=(float(velocities[i, 0]), float(velocities[i, 1])),
            radius=float(radii[i]),
        )
        for i in range(len(radii))
    )

    target = np.asarray(observation["target"], dtype=np.float64)
    goal = (float(target[0]), float(target[1]))
    if dt is None:
        dt = float((info or {}).get("dt") or 1.0 / 120.0)
    return SceneState(
        agent=agent,
        obstacles=obstacles,
        goal=goal,
        dt=float(dt),
        horizon=horizon,
    )


def action_to_velocity(action: Action) -> np.ndarray:
    """Abstract :class:`Action` -> world-frame velocity vector (2D)."""
    return action.velocity_vector


def to_game_action(action: Action) -> "GameAction":
    """Abstract :class:`Action` -> the simulator's unified ``Action``.

    The model's ``speed`` is an absolute speed; the simulator's ``Action`` is a
    unit ``direction`` + ``magnitude`` in ``[0, 1]`` (re-scaled by the player's
    max speed at the codec).  A zero-speed model action becomes ``stay``.
    """
    from bullet_sim.action.types import Action as GameAction

    v = action.velocity_vector
    if float(np.hypot(v[0], v[1])) <= 0.0:
        return GameAction.zero()
    return GameAction.from_vector(v)
