"""Trajectory predictors.

The first milestone is deliberately a **linear** extrapolation only: obstacles
move at constant velocity, and the agent moves at the candidate action's
constant velocity.  Kalman filters, learned dynamics or higher-order models
plug into the same :class:`Predictor` interface later without touching the
planner or the risk evaluator.

Predictions are vectorised (numpy arrays, no per-obstacle Python objects), and
a :class:`TrajectoryBatch` lets the planner evaluate *all* candidate actions in
one pass - the obstacle trajectory is shared, only the agent trajectories vary.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from cpu.decision.models import Action, SceneState

__all__ = ["Predictor", "LinearPredictor", "Trajectory", "TrajectoryBatch"]


@dataclass
class Trajectory:
    """Vectorised future of a scene under one candidate action."""

    agent_positions: np.ndarray      # (H, 2)
    agent_velocity: np.ndarray       # (2,)
    agent_max_speed: float
    obstacle_positions: np.ndarray   # (H, N, 2)
    obstacle_velocities: np.ndarray  # (N, 2)
    obstacle_radii: np.ndarray       # (N,)
    agent_radius: float


@dataclass
class TrajectoryBatch:
    """Vectorised future of a scene under ``B`` candidate actions.

    ``obstacle_*`` is computed once and shared; ``agent_*`` has a leading batch
    axis of size ``B``.
    """

    agent_positions: np.ndarray      # (B, H, 2)
    agent_velocities: np.ndarray     # (B, 2)
    agent_max_speed: float
    obstacle_positions: np.ndarray   # (H, N, 2)
    obstacle_velocities: np.ndarray  # (N, 2)
    obstacle_radii: np.ndarray       # (N,)
    agent_radius: float


class Predictor(ABC):
    """Roll a :class:`SceneState` forward in time under candidate actions."""

    @abstractmethod
    def predict(self, scene: SceneState, action: Action, n_steps: int) -> Trajectory:
        """Return the ``n_steps`` future trajectory (step 1 .. n_steps) under ``action``."""
        raise NotImplementedError

    def predict_batch(self, scene: SceneState, actions: Sequence[Action], n_steps: int) -> TrajectoryBatch:
        """Return a batch of trajectories; default loops :meth:`predict`."""
        trajs = [self.predict(scene, a, n_steps) for a in actions]
        first = trajs[0] if trajs else self.predict(scene, Action.stop(), n_steps)
        return TrajectoryBatch(
            agent_positions=np.stack([t.agent_positions for t in trajs]),
            agent_velocities=np.stack([t.agent_velocity for t in trajs]),
            agent_max_speed=first.agent_max_speed,
            obstacle_positions=first.obstacle_positions,
            obstacle_velocities=first.obstacle_velocities,
            obstacle_radii=first.obstacle_radii,
            agent_radius=first.agent_radius,
        )


class LinearPredictor(Predictor):
    """Constant-velocity (first-order / linear) extrapolation."""

    def predict(self, scene: SceneState, action: Action, n_steps: int) -> Trajectory:
        n = max(0, int(n_steps))
        dt = scene.dt
        agent = scene.agent
        agent_vel = action.velocity_vector

        t = (np.arange(n, dtype=np.float64) + 1.0) * dt
        agent_positions = agent.position[None, :] + agent_vel[None, :] * t[:, None]

        obstacles = scene.obstacles
        if not obstacles:
            obstacle_positions = np.zeros((n, 0, 2), dtype=np.float64)
            obstacle_velocities = np.zeros((0, 2), dtype=np.float64)
            obstacle_radii = np.zeros((0,), dtype=np.float64)
        else:
            obs_pos = np.stack([o.position for o in obstacles])  # (N, 2)
            obs_vel = np.stack([o.velocity for o in obstacles])  # (N, 2)
            obstacle_positions = obs_pos[None, :, :] + obs_vel[None, :, :] * t[:, None, None]
            obstacle_velocities = obs_vel
            obstacle_radii = np.array([o.radius for o in obstacles], dtype=np.float64)

        return Trajectory(
            agent_positions=agent_positions,
            agent_velocity=agent_vel,
            agent_max_speed=agent.max_speed,
            obstacle_positions=obstacle_positions,
            obstacle_velocities=obstacle_velocities,
            obstacle_radii=obstacle_radii,
            agent_radius=agent.radius,
        )

    def predict_batch(self, scene: SceneState, actions: Sequence[Action], n_steps: int) -> TrajectoryBatch:
        n = max(0, int(n_steps))
        dt = scene.dt
        agent = scene.agent

        velocities = np.stack([a.velocity_vector for a in actions])  # (B, 2)
        t = (np.arange(n, dtype=np.float64) + 1.0) * dt
        agent_positions = (
            agent.position[None, None, :] + velocities[:, None, :] * t[None, :, None]
        )  # (B, H, 2)

        obstacles = scene.obstacles
        if not obstacles:
            obstacle_positions = np.zeros((n, 0, 2), dtype=np.float64)
            obstacle_velocities = np.zeros((0, 2), dtype=np.float64)
            obstacle_radii = np.zeros((0,), dtype=np.float64)
        else:
            obs_pos = np.stack([o.position for o in obstacles])  # (N, 2)
            obs_vel = np.stack([o.velocity for o in obstacles])  # (N, 2)
            obstacle_positions = obs_pos[None, :, :] + obs_vel[None, :, :] * t[:, None, None]
            obstacle_velocities = obs_vel
            obstacle_radii = np.array([o.radius for o in obstacles], dtype=np.float64)

        return TrajectoryBatch(
            agent_positions=agent_positions,
            agent_velocities=velocities,
            agent_max_speed=agent.max_speed,
            obstacle_positions=obstacle_positions,
            obstacle_velocities=obstacle_velocities,
            obstacle_radii=obstacle_radii,
            agent_radius=agent.radius,
        )
