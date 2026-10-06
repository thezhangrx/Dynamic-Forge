"""Collision risk evaluation.

v0.2 risk model (weighted sum)::

    risk = w_distance * distance_risk
         + w_closing  * closing_risk
         + w_ttc      * time_to_collision_risk

* ``distance_risk``     - ``1 / (d + ε)`` clamped to ``[0, 1]``; ``d`` is the
  surface-to-surface clearance.
* ``closing_risk``      - normalised approach speed ``min(1, closing / agent_max_speed)``;
  ``closing = (v_obs - v_agent) · unit(obstacle -> agent)`` (positive = approaching).
* ``time_to_collision_risk`` - ``ttc_ref / (ttc + ttc_ref)``; ``ttc = clearance / closing``.
  Small ``ttc`` -> high risk.

``w_ttc`` is the highest weight: a fast, imminent obstacle dominates the score,
so the planner dodges the thing that is actually about to hit it.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from cpu.decision.models import AgentState, Obstacle
from cpu.decision.predictor import Trajectory, TrajectoryBatch

__all__ = ["RiskEvaluator"]


class RiskEvaluator:
    """Distance / closing-speed / time-to-collision risk in ``[0, 1]``."""

    def __init__(
        self,
        epsilon: float = 0.1,
        w_distance: float = 0.2,
        w_closing: float = 0.3,
        w_ttc: float = 0.5,
        ttc_reference: float = 1.0,
    ) -> None:
        self.epsilon = float(epsilon)
        self.w_distance = float(w_distance)
        self.w_closing = float(w_closing)
        self.w_ttc = float(w_ttc)
        self.ttc_reference = float(ttc_reference)
        if self.epsilon <= 0.0:
            raise ValueError("epsilon must be > 0 to keep risk finite")
        if self.ttc_reference <= 0.0:
            raise ValueError("ttc_reference must be > 0")
        if min(self.w_distance, self.w_closing, self.w_ttc) < 0.0:
            raise ValueError("risk weights must be non-negative")

    # -- single-frame primitives (distance only) ---------------------------
    def clearance(self, agent: AgentState, obstacles: Sequence[Obstacle]) -> float:
        """Smallest surface-to-surface distance to any obstacle (``>= 0``)."""
        if not obstacles:
            return float("inf")
        ap = agent.position
        ar = agent.radius
        best = float("inf")
        for o in obstacles:
            center = float(np.hypot(*(ap - o.position)))
            d = center - (ar + o.radius)
            if d < best:
                best = d
        return max(0.0, best)

    def risk(self, agent: AgentState, obstacles: Sequence[Obstacle]) -> float:
        """Distance-only risk in ``[0, 1]``: ``1 / (clearance + ε)``, clamped."""
        d = self.clearance(agent, obstacles)
        if d == float("inf"):
            return 0.0
        return float(min(1.0, 1.0 / (d + self.epsilon)))

    # -- trajectory risk (v0.2 weighted model) -----------------------------
    def risk_trajectory(self, trajectory: Trajectory) -> float:
        """Worst combined risk over a single :class:`Trajectory`."""
        batch = TrajectoryBatch(
            agent_positions=trajectory.agent_positions[None, :, :],
            agent_velocities=trajectory.agent_velocity[None, :],
            agent_max_speed=trajectory.agent_max_speed,
            obstacle_positions=trajectory.obstacle_positions,
            obstacle_velocities=trajectory.obstacle_velocities,
            obstacle_radii=trajectory.obstacle_radii,
            agent_radius=trajectory.agent_radius,
        )
        return float(self.risk_batch(batch)[0])

    def risk_batch(self, batch: TrajectoryBatch) -> np.ndarray:
        """Per-candidate worst risk, vectorised over ``(B, H, N)``.

        Returns a ``(B,)`` array; entry ``b`` is the worst combined risk of
        candidate ``b`` over the whole trajectory.
        """
        b = batch.agent_positions.shape[0]
        h = batch.obstacle_positions.shape[0]
        n = batch.obstacle_positions.shape[1]
        if b == 0 or h == 0 or n == 0:
            return np.zeros(b, dtype=np.float64)

        # (B, H, N, 2) relative position, agent -> obstacle
        delta = batch.agent_positions[:, :, None, :] - batch.obstacle_positions[None, :, :, :]
        dist = np.hypot(delta[..., 0], delta[..., 1])  # (B, H, N)
        radii = batch.agent_radius + batch.obstacle_radii[None, None, :]  # (1, 1, N)
        clearance = dist - radii
        clearance_pos = np.maximum(clearance, 0.0)

        # relative velocity v_obs - v_agent, constant over the linear trajectory
        rel_vel = (
            batch.obstacle_velocities[None, None, :, :]
            - batch.agent_velocities[:, None, None, :]
        )  # (B, 1, N, 2)
        safe_dist = np.maximum(dist, 1e-6)
        unit = delta / safe_dist[..., None]  # (B, H, N, 2)
        closing = np.maximum((rel_vel * unit).sum(axis=-1), 0.0)  # (B, H, N)

        distance_risk = np.minimum(1.0 / (clearance_pos + self.epsilon), 1.0)
        max_speed = max(float(batch.agent_max_speed), 1e-6)
        closing_risk = np.minimum(closing / max_speed, 1.0)
        ttc = clearance_pos / (closing + 1e-6)
        ttc_risk = self.ttc_reference / (ttc + self.ttc_reference)

        combined = (
            self.w_distance * distance_risk
            + self.w_closing * closing_risk
            + self.w_ttc * ttc_risk
        )
        worst = combined.max(axis=(1, 2))  # (B,)
        return np.minimum(worst, 1.0)
