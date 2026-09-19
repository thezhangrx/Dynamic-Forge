"""Baseline autonomous controllers that ship with the repository.

**No trained model exists in this repository.**  These are honest, runnable
baselines so that "autonomous mode" can be demonstrated, benchmarked and used
as the reference any future learned policy must beat:

===================  ============  ===============================================
controller           access        idea
===================  ============  ===============================================
``idle``             observation   never moves (control group)
``random``           observation   seeded random 9-way walk
``repulsion``        observation   steer away from nearby bullet density
``threat``           observation   analytic future-threat scoring over 9 actions
``planner``          world         clone the world and roll out candidate actions
===================  ============  ===============================================

Results for the same scene, all produced by the code below (medium preset,
seed 21, 900 steps, run headless):

===========================  ==================
``idle`` (never moves)       ~40–90 steps
``random``                   ~60–150 steps
``repulsion``                ~200–600 steps
``threat`` (predictive)      ~900 steps
``planner`` (world rollout)  ~900 steps
===========================  ==================

Exact numbers depend on the machine only through floating-point ordering; they
are reproduced by ``python -m bullet_sim autoplay --level medium``.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.ai.base import (
    BaseAutonomousController,
    ControllerSpec,
    bullet_matrix,
    direction_to_action,
)
from bullet_sim.core.actions import DISCRETE_ACTION_INDEX, DISCRETE_DIRECTIONS


class IdleController(BaseAutonomousController):
    """Control group: always ``stay``.  Useful to measure scene lethality."""

    SPEC = ControllerSpec(
        name="idle",
        kind="baseline",
        description="never moves (control group)",
        access="observation",
        cost="free",
    )

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        self._calls += 1
        return DISCRETE_ACTION_INDEX["stay"]


class RandomWalkController(BaseAutonomousController):
    """Seeded random 9-way walk.  Reproducible, not intelligent."""

    SPEC = ControllerSpec(
        name="random",
        kind="baseline",
        description="seeded random 9-way walk",
        access="observation",
        cost="free",
    )

    def __init__(self, seed: int = 0, period: int = 6) -> None:
        super().__init__()
        self.seed = int(seed)
        self.period = max(1, int(period))
        self._rng = np.random.Generator(np.random.PCG64(self.seed))
        self._current = 0

    def reset(self, observation=None, info=None) -> None:
        super().reset(observation, info)
        self._rng = np.random.Generator(np.random.PCG64(self.seed))

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        if self._calls % self.period == 0:
            self._current = int(self._rng.integers(0, 9))
        self._calls += 1
        return self._current


class RepulsionController(BaseAutonomousController):
    """Steer away from the local bullet density while drifting to the target.

    Observation-only and O(N) per step: the cheapest thing that actually dodges.

    The influence radius is derived from the player's own speed and a reaction
    time, so the controller behaves the same whether the field is 640 px or
    6400 px wide - a fixed pixel radius silently degenerates to "never dodge"
    at one scale and "always dodge" at another.
    """

    SPEC = ControllerSpec(
        name="repulsion",
        kind="baseline",
        description="repel from nearby bullets, attract to target",
        access="observation",
        cost="cheap",
    )

    def __init__(
        self,
        reaction_time: float = 0.45,
        repel_weight: float = 1.0,
        attract_weight: float = 0.15,
        min_radius: float = 12.0,
    ) -> None:
        super().__init__()
        self.reaction_time = float(reaction_time)
        self.repel_weight = float(repel_weight)
        self.attract_weight = float(attract_weight)
        self.min_radius = float(min_radius)

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        self._calls += 1
        player = np.asarray(observation["player"][:2], dtype=np.float64)
        speed = float(observation["player"][5])
        target = np.asarray(observation["target"][:2], dtype=np.float64)
        bullets = bullet_matrix(observation)

        radius = max(self.min_radius, speed * self.reaction_time)
        steer = np.zeros(2, dtype=np.float64)
        if bullets.shape[0]:
            rel = bullets[:, :2] - player[None, :]
            dist = np.hypot(rel[:, 0], rel[:, 1])
            near = dist < radius
            if near.any():
                # linear falloff: influence -> 0 exactly at the radius
                w = (1.0 - dist[near] / radius) ** 2
                steer -= (rel[near] / np.maximum(dist[near], 1e-6)[:, None] * w[:, None]).sum(
                    axis=0
                )
        toward = target - player
        n = float(np.hypot(toward[0], toward[1]))
        if n > 1e-6:
            steer += (toward / n) * self.attract_weight
        return direction_to_action(
            steer[0] * self.repel_weight, steer[1] * self.repel_weight
        )


class ThreatFieldController(BaseAutonomousController):
    """Predictive baseline: score all 9 actions by their future *threat*.

    For each candidate action the player is walked forward analytically and the
    minimum clearance to the ballistic future of every nearby bullet is taken::

        threat(a) = min over sampled t of ( distance(a, t) - r_bullet - r_player )

    The best action maximises that worst-case clearance.  Reaching the target is
    a *tie-break* worth a fraction of the safety radius, never enough to trade
    safety for progress.

    Everything is expressed in seconds (``lookahead``) and in units of the
    player's own speed, so the controller is independent of ``dt`` and of the
    field scale.  No world cloning happens, so it stays cheap at thousands of
    bullets: bullets outside the player's reachable box are pruned first.

    This is the reference ``prediction -> risk -> decision`` loop that the
    CPU/FPGA split is meant to accelerate; see ``bullet_sim.fpga``.
    """

    SPEC = ControllerSpec(
        name="threat",
        kind="baseline",
        description="analytic future-threat scoring over 9 actions",
        access="observation",
        cost="moderate",
        notes="prediction -> risk -> decision; the loop the CPU/FPGA path accelerates",
    )

    def __init__(
        self,
        lookahead: float = 0.30,
        samples: int = 12,
        safety_radius: float = 18.0,
        tie_tolerance: float = 0.25,
        sticky: float = 0.05,
    ) -> None:
        super().__init__()
        self.lookahead = float(lookahead)
        self.samples = max(2, int(samples))
        self.safety_radius = float(safety_radius)
        #: Two actions within ``tie_tolerance * safety_radius`` of the best
        #: worst-case clearance are considered equally safe; the target-distance
        #: tie-break then decides between them.
        self.tie_tolerance = float(tie_tolerance)
        #: Extra tie-break weight for repeating the previous action, which stops
        #: the controller from dithering between two equally safe directions.
        self.sticky = float(sticky)
        self._last_index = 0

    def threat_scores(
        self, observation: Mapping[str, Any], info: Mapping[str, Any]
    ) -> np.ndarray:
        """Worst-case clearance for each of the 9 discrete actions (px)."""
        player = np.asarray(observation["player"][:2], dtype=np.float64)
        speed = float(observation["player"][5])
        dt = float(info.get("dt") or 1.0 / 120.0)
        horizon = max(1, int(round(self.lookahead / dt)))
        times = np.linspace(0.0, self.lookahead, self.samples) * speed  # distances

        reach = float(times[-1]) + self.safety_radius + 8.0
        bullets = bullet_matrix(observation)
        scores = np.full(9, float("inf"), dtype=np.float64)
        if bullets.shape[0]:
            near = (
                (np.abs(bullets[:, 0] - player[0]) < reach)
                & (np.abs(bullets[:, 1] - player[1]) < reach)
            )
            bullets = bullets[near]
        if bullets.shape[0] == 0:
            return scores

        bx = bullets[:, 0].astype(np.float64)
        by = bullets[:, 1].astype(np.float64)
        bvx = bullets[:, 2].astype(np.float64)
        bvy = bullets[:, 3].astype(np.float64)
        br = bullets[:, 4].astype(np.float64)
        # time (s) corresponding to each sampled travel distance
        tsec = (times / max(speed, 1e-9))

        for idx in range(9):
            dirx, diry = DISCRETE_DIRECTIONS[idx]
            px = player[0] + dirx * times            # (S,)
            py = player[1] + diry * times
            fbx = bx[:, None] + bvx[:, None] * tsec[None, :]   # (N, S)
            fby = by[:, None] + bvy[:, None] * tsec[None, :]
            dist = np.hypot(px[None, :] - fbx, py[None, :] - fby) - br[:, None]
            scores[idx] = float(dist.min()) if dist.size else float("inf")
        return scores

    def choose(
        self, observation: Mapping[str, Any], info: Mapping[str, Any]
    ) -> tuple[int, np.ndarray]:
        """Pick an action from the threat scores.

        Two-stage, safety-first rule::

            safe = { a : threat(a) >= best_threat - tie_tolerance }
            a*   = argmax over safe of (current target distance
                                        - distance after one step in direction a)

        Stage 1 is safety; stage 2 only ever breaks *near-ties*.  Collapsing the
        score itself (e.g. ``min(threat, safety_radius)``) would throw away the
        fact that 70 px of clearance is much better than 18 px, and the
        controller then wanders into the "merely adequate" option.
        """
        player = np.asarray(observation["player"][:2], dtype=np.float64)
        target = np.asarray(observation["target"][:2], dtype=np.float64)
        scores = self.threat_scores(observation, info)

        finite = np.isfinite(scores)
        if not finite.any():
            toward = target - player
            return direction_to_action(toward[0], toward[1]), scores

        # Unknown (no bullets in range) counts as "very safe", not as "-inf".
        cap = float(np.max(scores[finite]))
        base = np.where(finite, scores, cap)
        best = float(base.max())
        tolerance = self.tie_tolerance * self.safety_radius
        safe = np.flatnonzero(base >= best - tolerance)

        gain = np.array(
            [
                float(np.hypot(*(target - player)))
                - float(
                    np.hypot(
                        *(target - (player + np.asarray(DISCRETE_DIRECTIONS[i])))
                    )
                )
                for i in range(9)
            ]
        )
        # Among the safe set: prefer progress, then continuity (avoid dithering),
        # then the lowest index for determinism.
        order = sorted(
            safe.tolist(),
            key=lambda i: (-gain[i], i != self._last_index, i),
        )
        chosen = int(order[0])
        self._last_index = chosen
        return chosen, scores

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        self._calls += 1
        action, _scores = self.choose(observation, info)
        return action


class RolloutPlanner(BaseAutonomousController):
    """MPC-style baseline that clones the world and evaluates real rollouts.

    Most accurate and most expensive: it uses ``World.simulate_future`` and is
    therefore ``world``-access.  Use it on small scenes or as ground truth when
    validating the cheaper predictors.
    """

    SPEC = ControllerSpec(
        name="planner",
        kind="baseline",
        description="clone the world and roll out each candidate action",
        access="world",
        cost="expensive",
        notes="accurate reference; not deployable as-is because it reads the world",
    )

    def __init__(self, world: Any, horizon: int = 24, objective: str = "clearance") -> None:
        super().__init__()
        from bullet_sim.prediction.rollout import FutureSimulator

        self.sim = FutureSimulator(world)
        self.horizon = int(horizon)
        self.objective = objective

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        from bullet_sim.prediction.rollout import best_candidate, evaluate_candidates

        self._calls += 1
        results = evaluate_candidates(
            self.sim, list(range(9)), self.horizon, objective=self.objective
        )
        best = best_candidate(results)
        return int(best.action) if best is not None else 0


#: name -> factory (only the observation-only ones are world-free)
BASELINE_CONTROLLERS: dict[str, Any] = {
    "idle": IdleController,
    "stay": IdleController,
    "random": RandomWalkController,
    "repulsion": RepulsionController,
    "threat": ThreatFieldController,
    "planner": RolloutPlanner,
}


def make_controller(name: str = "threat", *, world: Any = None, **kwargs: Any) -> Any:
    """Factory for the CLI / UI.

    ``name`` is one of :data:`BASELINE_CONTROLLERS`.  ``planner`` requires
    ``world``; passing it explicitly is how the caller acknowledges that this
    controller has privileged access.
    """
    key = str(name).strip().lower()
    if key not in BASELINE_CONTROLLERS:
        raise ValueError(
            f"unknown autonomous controller {name!r}; "
            f"available: {sorted(BASELINE_CONTROLLERS)}"
        )
    cls = BASELINE_CONTROLLERS[key]
    if cls is RolloutPlanner:
        if world is None:
            raise ValueError(
                "the 'planner' controller needs world access; "
                "construct it as make_controller('planner', world=env.world)"
            )
        return cls(world, **kwargs)
    return cls(**kwargs)


def available_controllers() -> list[str]:
    return sorted(BASELINE_CONTROLLERS)
