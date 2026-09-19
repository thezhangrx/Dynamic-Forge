"""Example 4 - plugging in an external controller.

Shows the three supported integration styles:

1. a plain function (rule-based / anytime policy),
2. an ``ActionProvider`` class (stateful controller),
3. a "policy-like" object with ``predict(obs)`` (e.g. an ONNX/CPU/FPGA wrapper).

None of them require touching simulator code.

    python bullet_sim/examples/04_external_controller.py
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from typing import Any, Mapping

import numpy as np

from bullet_sim.interface.controller import ActionProvider
from bullet_sim.prediction.rollout import FutureSimulator, best_candidate, evaluate_candidates
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


# --------------------------------------------------------------------------
# 1. a stateless rule-based controller written as a plain function
# --------------------------------------------------------------------------
def dodge_towards_target(observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
    p = observation["player"]
    t = observation["target"]
    bullets = observation["bullets"]
    # crude: step away from the local bullet density, but keep drifting to target
    if bullets.shape[0]:
        d = np.hypot(bullets[:, 0] - p[0], bullets[:, 1] - p[1])
        near = bullets[d < 0.06]
        if near.shape[0]:
            away = np.array([p[0] - near[:, 0].mean(), p[1] - near[:, 1].mean()])
        else:
            away = np.zeros(2)
    else:
        away = np.zeros(2)
    to_target = np.array([t[0] - p[0], t[1] - p[1]])
    v = to_target * 0.6 + away * 1.4
    angle = np.arctan2(v[1], v[0])
    sector = int(round((angle % (2 * np.pi)) / (np.pi / 4))) % 8
    return 1 + sector  # action 1..8 are the 8 directions


# --------------------------------------------------------------------------
# 2. a stateful ActionProvider
# --------------------------------------------------------------------------
class AlternatingController(ActionProvider):
    """Alternates between two escape directions every ``period`` steps."""

    def __init__(self, period: int = 25, a: int = 1, b: int = 2) -> None:
        self.period = int(period)
        self.a, self.b = a, b
        self.t = 0

    def reset(self, observation, info) -> None:
        self.t = 0

    def act(self, observation, info) -> int:
        self.t += 1
        return self.a if (self.t // self.period) % 2 == 0 else self.b


# --------------------------------------------------------------------------
# 3. a one-step-lookahead planner built purely on public API
# --------------------------------------------------------------------------
class LookaheadPlanner(ActionProvider):
    """Picks the action whose 20-step rollout keeps the largest clearance."""

    def __init__(self, world: Any, horizon: int = 20) -> None:
        self.sim = FutureSimulator(world)
        self.horizon = int(horizon)

    def reset(self, observation, info) -> None:
        return None

    def act(self, observation, info) -> int:
        results = evaluate_candidates(
            self.sim, list(range(9)), self.horizon, objective="clearance"
        )
        best = best_candidate(results)
        return int(best.action) if best is not None else 0


def main() -> None:
    spec = scenario_for_level("medium", seed=21, duration=25.0)

    for label, controller in (
        ("function controller", dodge_towards_target),
        ("ActionProvider", AlternatingController()),
    ):
        env = BulletHellEnv(spec, seed=21)
        result = env.run(controller, steps=900, seed=21)
        env.close()
        print(
            f"{label:<20} survived {result['steps']:>4} steps  "
            f"return={result['return']:>7.1f}  collisions={result['collisions']}"
        )

    # The planner needs the World, so build the env first.
    env = BulletHellEnv(spec, seed=21, terminate_on_collision=True)
    planner = LookaheadPlanner(env.world, horizon=20)
    result = env.run(planner, steps=300, seed=21)
    env.close()
    print(
        f"{'lookahead planner':<20} survived {result['steps']:>4} steps  "
        f"return={result['return']:>7.1f}  collisions={result['collisions']}"
    )


if __name__ == "__main__":
    main()
