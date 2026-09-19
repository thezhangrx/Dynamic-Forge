"""Rollout-based future simulation and candidate-action evaluation.

Two complementary capabilities, both built strictly on ``World.clone()`` +
``World.step()`` (the kernel exposes nothing prediction-specific):

* :class:`FutureSimulator` - "copy the current state and push it forward":
  ``S_t -> S_{t+1} -> ... -> S_{t+H}`` for a given action or action sequence.
* :func:`evaluate_candidates` - roll out every candidate action and score it
  with a pluggable objective, which is exactly what an MPC / risk-aware planner
  needs from the environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

from bullet_sim.core.state import WorldSnapshot
from bullet_sim.prediction.base import TrajectoryForecast
from bullet_sim.prediction.ballistic import BallisticPredictor

# --------------------------------------------------------------------------
# geometry helpers usable on any snapshot
# --------------------------------------------------------------------------


def clearance(snapshot: WorldSnapshot) -> float:
    """Signed distance from the player surface to the nearest bullet surface.

    ``inf`` when the field is empty, negative when overlapping.  This is the
    canonical scalar risk signal for planning and for reward shaping.
    """
    pool = snapshot.bullets
    if pool.count == 0:
        return float("inf")
    idx = pool.active_indices()
    if idx.size == 0:
        return float("inf")
    d = pool.data
    p = snapshot.player
    dx = d["x"][idx] - p.x
    dy = d["y"][idx] - p.y
    dist = np.sqrt(dx * dx + dy * dy)
    return float(np.min(dist - (d["radius"][idx] + p.radius)))


def time_to_collision(snapshot: WorldSnapshot, max_seconds: float | None = None) -> float:
    """Analytic first-impact time for the *player at rest* (a risk heuristic).

    Uses the quadratic ``|p + v t - q| = r_p + r_b`` per bullet and takes the
    smallest non-negative root.  Returns ``inf`` when nothing hits.
    """
    pool = snapshot.bullets
    if pool.count == 0:
        return float("inf")
    idx = pool.active_indices()
    d = pool.data
    p = snapshot.player
    dx = d["x"][idx] - p.x
    dy = d["y"][idx] - p.y
    vx = d["vx"][idx]
    vy = d["vy"][idx]
    r = d["radius"][idx] + p.radius
    a = vx * vx + vy * vy
    b = 2.0 * (dx * vx + dy * vy)
    c = dx * dx + dy * dy - r * r
    safe_a = np.where(a > 1e-12, a, 1.0)
    disc = b * b - 4.0 * safe_a * c
    hit = (disc >= 0.0) & (a > 1e-12) & (c > 0.0)
    if not hit.any():
        return float("inf")
    sq = np.sqrt(disc[hit])
    t1 = (-b[hit] - sq) / (2.0 * safe_a[hit])
    t2 = (-b[hit] + sq) / (2.0 * safe_a[hit])
    cand = np.where(t1 >= 0.0, t1, t2)
    cand = cand[cand >= 0.0]
    if cand.size == 0:
        return float("inf")
    t = float(cand.min())
    if max_seconds is not None and t > max_seconds:
        return float("inf")
    return t


# --------------------------------------------------------------------------
# rollout
# --------------------------------------------------------------------------


@dataclass
class RolloutResult:
    """One candidate action rolled forward ``H`` steps."""

    action: Any
    states: list[WorldSnapshot]
    rewards: list[float]
    terminated_at: int | None
    objective: float
    min_clearance: float
    final_target_distance: float
    info: dict[str, Any] = field(default_factory=dict)

    @property
    def survived(self) -> bool:
        return self.terminated_at is None


class FutureSimulator:
    """'Clone current state and advance it' - with zero side effects on the world."""

    def __init__(self, world: Any, *, ballistic: BallisticPredictor | None = None) -> None:
        self.world = world
        self.ballistic = ballistic or BallisticPredictor()

    # ------------------------------------------------------------------
    def rollout(
        self,
        horizon: int,
        action: Any = 0,
        *,
        stride: int = 1,
        stop_on_termination: bool = False,
    ) -> RolloutResult:
        """Simulate ``S_{t+1..t+H}`` under a constant action on a clone."""
        result = self.rollout_sequence(
            [action] * int(horizon), stride=stride, stop_on_termination=stop_on_termination
        )
        # Report the *original* action, not the expanded sequence.
        result.action = action
        return result

    def rollout_sequence(
        self,
        actions: Sequence[Any],
        *,
        stride: int = 1,
        stop_on_termination: bool = False,
    ) -> RolloutResult:
        """Simulate the exact action sequence on a clone."""
        acts = list(actions)
        if not acts:
            raise ValueError("actions must not be empty")
        rollout = self.world.clone()
        states: list[WorldSnapshot] = []
        rewards: list[float] = []
        terminated_at: int | None = None
        for i, a in enumerate(acts):
            res = rollout.step(a)
            rewards.append(float(res.reward))
            if (i + 1) % max(1, stride) == 0:
                states.append(rollout.get_state())
            if res.terminated:
                terminated_at = i
                if stop_on_termination:
                    break
        return RolloutResult(
            action=acts[0] if len(acts) == 1 else list(acts),
            states=states,
            rewards=rewards,
            terminated_at=terminated_at,
            objective=float(np.sum(rewards)),
            min_clearance=min((clearance(s) for s in states), default=float("inf")),
            final_target_distance=(
                states[-1].target.distance_to(states[-1].player.x, states[-1].player.y)
                if states
                else float("inf")
            ),
            info={"steps": len(acts)},
        )

    # ------------------------------------------------------------------
    def simulate_future(
        self,
        horizon: int,
        action: Any = 0,
        *,
        stride: int = 1,
        include_initial: bool = True,
    ) -> list[WorldSnapshot]:
        """Thin convenience wrapper returning only the state trajectory."""
        result = self.rollout(horizon, action, stride=stride)
        states = list(result.states)
        if include_initial:
            states.insert(0, self.world.get_state())
        return states

    # ------------------------------------------------------------------
    def ballistic_forecast(self, horizon: int, dt: float | None = None) -> TrajectoryForecast:
        """Closed-form bullet forecast (no stepping) for comparison studies."""
        return self.ballistic.predict(self.world.get_state(), horizon, dt)


# --------------------------------------------------------------------------
# candidate evaluation
# --------------------------------------------------------------------------

Objective = Callable[[RolloutResult], float]


def survival_objective(result: RolloutResult) -> float:
    """Prefer long survival, then large clearance."""
    survived = len(result.rewards) if result.terminated_at is None else result.terminated_at
    return float(survived) + 0.001 * min(result.min_clearance, 1e3)


def clearance_objective(result: RolloutResult) -> float:
    return float(min(result.min_clearance, 1e3))


def target_objective(result: RolloutResult) -> float:
    return -float(result.final_target_distance)


OBJECTIVES: dict[str, Objective] = {
    "survival": survival_objective,
    "clearance": clearance_objective,
    "target": target_objective,
}


def evaluate_candidates(
    future: FutureSimulator,
    candidates: Sequence[Any],
    horizon: int,
    *,
    objective: str | Objective = "survival",
    stride: int = 1,
    stop_on_termination: bool = False,
) -> list[RolloutResult]:
    """Roll every candidate action forward and score it.

    This is the primitive behind candidate-action evaluation, MPC, and
    risk-aware decision making.  Nothing here is specific to any policy.
    """
    fn: Objective
    if isinstance(objective, str):
        try:
            fn = OBJECTIVES[objective]
        except KeyError as exc:
            raise ValueError(
                f"unknown objective {objective!r}; available: {sorted(OBJECTIVES)}"
            ) from exc
    else:
        fn = objective
    out: list[RolloutResult] = []
    for action in candidates:
        res = future.rollout(horizon, action, stride=stride, stop_on_termination=stop_on_termination)
        res.objective = float(fn(res))
        out.append(res)
    return out


def best_candidate(results: Sequence[RolloutResult]) -> RolloutResult | None:
    """Highest-objective candidate; ties broken by the lowest action index."""
    if not results:
        return None
    best = 0
    for i in range(1, len(results)):
        if results[i].objective > results[best].objective:
            best = i
    return results[best]


def describe_results(results: Sequence[RolloutResult]) -> list[dict[str, Any]]:
    return [
        {
            "action": r.action,
            "objective": r.objective,
            "survived": r.survived,
            "terminated_at": r.terminated_at,
            "min_clearance": r.min_clearance,
            "final_target_distance": r.final_target_distance,
        }
        for r in results
    ]


__all__ = [
    "FutureSimulator",
    "RolloutResult",
    "clearance",
    "time_to_collision",
    "evaluate_candidates",
    "best_candidate",
    "describe_results",
    "OBJECTIVES",
]
