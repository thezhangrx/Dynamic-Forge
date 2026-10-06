"""Multi-objective trajectory cost (v0.3-3).

Replaces the single "minimum risk" objective with a weighted trajectory cost::

    total = w_collision * collision_cost
          + w_ttc       * ttc_cost
          + w_smooth    * smooth_cost
          + w_progress  * progress_cost
          + w_dwell     * dwell_cost
          + w_trap      * trap_cost

v0.3-3 adds the **trap cost**: at the terminal position of a candidate
trajectory the agent probes three escape directions (forward / left / right);
each direction blocked by a nearby obstacle lowers the available-escape count,
so ``trap_cost = 1 - available / 3`` penalises steering into a dead end.

The whole evaluation runs on **numpy arrays** (structure-of-arrays layout), never
on per-obstacle Python objects, so the same arithmetic maps directly onto a
future FPGA BRAM / DSP datapath.

v0.3-5 exposes :meth:`TrajectoryCost.safety_metrics_batch`: the same
clearance / closing-speed / TTC arithmetic, returned as per-candidate scalars so
the planner's safety gate can classify feasibility **without** introducing a
second collision model.  The weighted cost terms themselves are unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cpu.decision.predictor import Trajectory, TrajectoryBatch

__all__ = ["TrajectoryCost", "SafetyMetrics", "UtilityTerms"]


@dataclass(frozen=True)
class SafetyMetrics:
    """Per-candidate feasibility primitives shared by the cost and the safety gate.

    ``min_clearance`` is the signed surface clearance of the closest predicted
    approach (negative = penetration); ``ttc_min`` is the smallest predicted
    time-to-collision (``inf`` = nothing ever closes).  Both are ``(B,)`` arrays,
    one entry per candidate, and are produced by the *same* arithmetic as
    :meth:`TrajectoryCost.evaluate`, so the codebase keeps exactly one collision
    model.
    """

    min_clearance: np.ndarray  # (B,) signed surface distance
    ttc_min: np.ndarray        # (B,) time to first predicted contact


@dataclass(frozen=True)
class UtilityTerms:
    """FES / behaviour / task cost components for one trajectory (v0.4.0-FAR).

    Kept separate from collision / TTC so the planner can rank them in distinct
    hierarchy tiers (CommonRoad Drivability Checker separates feasibility from
    utility).  Every field is already weighted exactly as in :meth:`evaluate`.
    """

    trap: float      # level 3: future escape space (FES)
    smooth: float    # level 4: behaviour quality
    dwell: float     # level 4: behaviour quality
    progress: float  # level 5: task objective

#: cos(45°) - a direction is "blocked" by an obstacle inside this half-angle cone.
_COS45 = float(np.cos(np.deg2rad(45.0)))


class TrajectoryCost:
    """Weighted sum of collision / TTC / smooth / progress / dwell / trap costs."""

    def __init__(
        self,
        w_collision: float = 10.0,
        w_ttc: float = 5.0,
        w_smooth: float = 1.0,
        w_progress: float = -0.5,
        w_dwell: float = 1.2,
        w_trap: float = 2.0,
        ttc_reference: float = 1.0,
        escape_radius: float = 50.0,
    ) -> None:
        self.w_collision = float(w_collision)
        self.w_ttc = float(w_ttc)
        self.w_smooth = float(w_smooth)
        self.w_progress = float(w_progress)
        self.w_dwell = float(w_dwell)
        self.w_trap = float(w_trap)
        self.ttc_reference = float(ttc_reference)
        self.escape_radius = float(escape_radius)
        if self.ttc_reference <= 0.0:
            raise ValueError("ttc_reference must be > 0")
        if self.escape_radius <= 0.0:
            raise ValueError("escape_radius must be > 0")

    def evaluate(
        self,
        trajectory: Trajectory,
        *,
        goal: np.ndarray | None = None,
        start_position: np.ndarray | None = None,
        previous_velocity: np.ndarray | None = None,
        smooth_weight: float | None = None,
        min_clearance: float | None = None,
        ttc_min: float | None = None,
    ) -> float:
        """Total cost of one candidate trajectory (pure function of numpy arrays).

        ``smooth_weight`` optionally overrides ``w_smooth`` for this evaluation
        (the planner lowers it when a threat is imminent, to allow sharp turns).

        ``min_clearance`` / ``ttc_min`` may be supplied by a caller that already
        computed them (the planner's safety gate does so in one batched pass);
        they only avoid recomputing identical arithmetic, never change its meaning.
        """
        agent_positions = trajectory.agent_positions          # (H, 2)
        agent_velocity = trajectory.agent_velocity            # (2,)
        max_speed = trajectory.agent_max_speed
        obstacle_positions = trajectory.obstacle_positions    # (H, N, 2)
        obstacle_velocities = trajectory.obstacle_velocities  # (N, 2)
        obstacle_radii = trajectory.obstacle_radii            # (N,)
        agent_radius = trajectory.agent_radius

        cost = 0.0
        h = obstacle_positions.shape[0]
        n = obstacle_positions.shape[1]

        # ---- collision + TTC + trap, vectorised over every (step, obstacle) ----
        if h > 0 and n > 0:
            # Reuse the batched scalars when provided, so clearance / closing /
            # TTC physics is computed exactly once per planning step.
            if min_clearance is None or ttc_min is None:
                mc, tm = self.clearance_and_ttc(
                    agent_positions, agent_velocity, obstacle_positions,
                    obstacle_velocities, obstacle_radii, agent_radius,
                )
                if min_clearance is None:
                    min_clearance = mc
                if ttc_min is None:
                    ttc_min = tm

            # collision: a huge penalty once the agent penetrates any obstacle
            if min_clearance < 0.0:
                cost += self.w_collision * (1.0 + (-min_clearance))

            # ttc: smaller time-to-collision -> higher cost
            if np.isfinite(ttc_min):
                cost += self.w_ttc * (
                    self.ttc_reference / (ttc_min + self.ttc_reference)
                )

            # trap: dead-end penalty at the terminal position
            cost += self.w_trap * self._trap_cost(
                agent_positions, agent_velocity, previous_velocity,
                obstacle_positions, obstacle_radii, agent_radius,
            )

        # ---- smoothness: penalise deviating from the previous action ----
        if previous_velocity is not None:
            change = float(np.linalg.norm(agent_velocity - previous_velocity))
            w = self.w_smooth if smooth_weight is None else float(smooth_weight)
            cost += w * (change / max(max_speed, 1e-6))

        # ---- dwell: charge standing still (idle agents get hit) ----
        speed = float(np.hypot(agent_velocity[0], agent_velocity[1]))
        cost += self.w_dwell * (1.0 - min(speed / max(max_speed, 1e-6), 1.0))

        # ---- progress: reward moving toward the goal (negative weight) ----
        if (
            goal is not None
            and start_position is not None
            and agent_positions.shape[0] > 0
        ):
            start = float(np.hypot(*(start_position - goal)))
            end = float(np.hypot(*(agent_positions[-1] - goal)))
            progress = (start - end) / max(start, 1e-6)
            cost += self.w_progress * progress

        return float(cost)

    # ------------------------------------------------------------------
    @staticmethod
    def clearance_and_ttc(
        agent_positions: np.ndarray,
        agent_velocity: np.ndarray,
        obstacle_positions: np.ndarray,
        obstacle_velocities: np.ndarray,
        obstacle_radii: np.ndarray,
        agent_radius: float,
    ) -> tuple[float, float]:
        """Shared feasibility scalars for one trajectory: ``(min_clearance, ttc_min)``.

        This is the single clearance / closing-speed / TTC model in the decision
        core: :meth:`evaluate` and :meth:`safety_metrics_batch` both derive from
        it, so the safety gate can never disagree with the cost about a collision.
        """
        delta = agent_positions[:, None, :] - obstacle_positions       # (H, N, 2)
        dist = np.hypot(delta[..., 0], delta[..., 1])                  # (H, N)
        clearance = dist - (agent_radius + obstacle_radii[None, :])    # (H, N) signed
        min_clearance = float(clearance.min())

        rel_vel = obstacle_velocities[None, :, :] - agent_velocity[None, None, :]
        unit = delta / np.maximum(dist, 1e-6)[..., None]
        closing = np.maximum((rel_vel * unit).sum(axis=-1), 0.0)       # (H, N)
        if float(closing.max()) <= 0.0:
            return min_clearance, float("inf")
        ttc = np.maximum(clearance, 0.0) / (closing + 1e-6)
        ttc_min = float(np.where(closing > 0.0, ttc, np.inf).min())
        return min_clearance, ttc_min

    def safety_metrics_batch(self, batch: TrajectoryBatch) -> SafetyMetrics:
        """Per-candidate feasibility primitives for a whole candidate batch.

        Vectorised over ``(B, H, N)`` so the safety gate costs one pass, not one
        pass per candidate, and expressed with the exact same arithmetic as
        :meth:`clearance_and_ttc`.
        """
        b = batch.agent_positions.shape[0]
        if b == 0:
            return SafetyMetrics(
                min_clearance=np.zeros(0, dtype=np.float64),
                ttc_min=np.zeros(0, dtype=np.float64),
            )
        h = batch.obstacle_positions.shape[0]
        n = batch.obstacle_positions.shape[1]
        if h == 0 or n == 0:
            # no obstacle can ever collide -> every candidate is trivially clear
            inf = np.full(b, np.inf, dtype=np.float64)
            return SafetyMetrics(min_clearance=inf, ttc_min=inf.copy())

        delta = (
            batch.agent_positions[:, :, None, :]
            - batch.obstacle_positions[None, :, :, :]
        )  # (B, H, N, 2)
        dist = np.hypot(delta[..., 0], delta[..., 1])                 # (B, H, N)
        clearance = dist - (
            batch.agent_radius + batch.obstacle_radii[None, None, :]
        )                                                             # (B, H, N)

        rel_vel = (
            batch.obstacle_velocities[None, None, :, :]
            - batch.agent_velocities[:, None, None, :]
        )  # (B, 1, N, 2)
        unit = delta / np.maximum(dist, 1e-6)[..., None]
        closing = np.maximum((rel_vel * unit).sum(axis=-1), 0.0)      # (B, H, N)
        ttc = np.where(
            closing > 0.0,
            np.maximum(clearance, 0.0) / (closing + 1e-6),
            np.inf,
        )

        return SafetyMetrics(
            min_clearance=clearance.min(axis=(1, 2)),
            ttc_min=ttc.min(axis=(1, 2)),
        )

    # ------------------------------------------------------------------
    def utility_terms(
        self,
        trajectory: Trajectory,
        *,
        goal: np.ndarray | None = None,
        start_position: np.ndarray | None = None,
        previous_velocity: np.ndarray | None = None,
        smooth_weight: float | None = None,
    ) -> UtilityTerms:
        """FES + behaviour + task terms, with **no** collision / TTC work.

        # [v0.4.0-FAR]
        # PURPOSE:
        #   Let the planner rank feasibility (FAR) and utility in separate
        #   hierarchy tiers without recomputing clearance/TTC, which the FAR
        #   safety filter already owns.
        #
        # OPEN-SOURCE REFERENCE:
        #   CommonRoad Drivability Checker: collision checking and trajectory
        #   utility stay conceptually separate; Nav2 MPPI critic layering.
        #
        # ALGORITHM:
        #   Reuses the exact weighted trap (FES), smoothness, dwell and progress
        #   terms of :meth:`evaluate`; only the collision/TTC block is omitted.
        #
        # FPGA MAPPING:
        #   trap is candidate x obstacle compare/reduce; smooth/dwell/progress
        #   are scalar per candidate.
        #
        # CPU ROLE:
        #   Supplies the lower tiers; the CPU combines them with the risk tier.
        #
        # COMPLEXITY:
        #   O(N) per candidate for trap, O(1) otherwise.
        # """
        agent_positions = trajectory.agent_positions
        agent_velocity = trajectory.agent_velocity
        max_speed = trajectory.agent_max_speed
        obstacle_positions = trajectory.obstacle_positions
        obstacle_radii = trajectory.obstacle_radii
        agent_radius = trajectory.agent_radius

        trap = 0.0
        if obstacle_positions.shape[0] > 0 and obstacle_positions.shape[1] > 0:
            trap = self.w_trap * self._trap_cost(
                agent_positions, agent_velocity, previous_velocity,
                obstacle_positions, obstacle_radii, agent_radius,
            )

        smooth = 0.0
        if previous_velocity is not None:
            change = float(np.linalg.norm(agent_velocity - previous_velocity))
            w = self.w_smooth if smooth_weight is None else float(smooth_weight)
            smooth = w * (change / max(max_speed, 1e-6))

        speed = float(np.hypot(agent_velocity[0], agent_velocity[1]))
        dwell = self.w_dwell * (1.0 - min(speed / max(max_speed, 1e-6), 1.0))

        progress = 0.0
        if (
            goal is not None
            and start_position is not None
            and agent_positions.shape[0] > 0
        ):
            start = float(np.hypot(*(start_position - goal)))
            end = float(np.hypot(*(agent_positions[-1] - goal)))
            progress = self.w_progress * ((start - end) / max(start, 1e-6))

        return UtilityTerms(trap=trap, smooth=smooth, dwell=dwell, progress=progress)

    # ------------------------------------------------------------------
    def _trap_cost(
        self,
        agent_positions: np.ndarray,
        agent_velocity: np.ndarray,
        previous_velocity: np.ndarray | None,
        obstacle_positions: np.ndarray,
        obstacle_radii: np.ndarray,
        agent_radius: float,
    ) -> float:
        """``1 - available/3`` for the terminal escape space (forward/left/right).

        A direction counts as blocked when an obstacle surface lies within
        ``escape_radius`` and inside a 45-degree cone around that direction.
        """
        term_pos = agent_positions[-1]                      # (2,)
        term_obs = obstacle_positions[-1]                   # (N, 2)
        if term_obs.shape[0] == 0:
            return 0.0

        heading = self._heading(agent_velocity, previous_velocity)
        # forward, left (+90deg CCW), right (-90deg CW)
        directions = (
            heading,
            np.array([-heading[1], heading[0]]),
            np.array([heading[1], -heading[0]]),
        )

        delta = term_obs - term_pos                          # (N, 2)
        dist = np.hypot(delta[:, 0], delta[:, 1])            # (N,)
        safe_dist = np.maximum(dist, 1e-6)
        clearance = dist - (agent_radius + obstacle_radii)   # (N,)

        available = 0
        for d in directions:
            cosang = (delta @ d) / safe_dist                 # (N,)
            blocked = bool(np.any((clearance < self.escape_radius) & (cosang > _COS45)))
            if not blocked:
                available += 1
        return 1.0 - available / 3.0

    @staticmethod
    def _heading(
        agent_velocity: np.ndarray, previous_velocity: np.ndarray | None
    ) -> np.ndarray:
        """Unit heading: the candidate's velocity, else previous, else +x."""
        speed = float(np.hypot(agent_velocity[0], agent_velocity[1]))
        if speed > 1e-6:
            return agent_velocity / speed
        if previous_velocity is not None:
            pv = float(np.hypot(previous_velocity[0], previous_velocity[1]))
            if pv > 1e-6:
                return previous_velocity / pv
        return np.array([1.0, 0.0], dtype=np.float64)
