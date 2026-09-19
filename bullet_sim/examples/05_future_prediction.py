"""Example 5 - future simulation, danger fields and candidate evaluation.

    python bullet_sim/examples/05_future_prediction.py
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.prediction.ballistic import BallisticPredictor
from bullet_sim.prediction.danger_field import compute_danger_field, danger_from_forecast
from bullet_sim.prediction.rollout import (
    FutureSimulator,
    best_candidate,
    clearance,
    describe_results,
    evaluate_candidates,
    time_to_collision,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


def main() -> None:
    spec = scenario_for_level("hard", seed=4, duration=30.0)
    env = BulletHellEnv(spec, seed=4, reward="zero", terminate_on_collision=False)
    env.reset(seed=4)
    for _ in range(300):
        env.step(0)

    snapshot = env.get_state()
    horizon = 90
    print(f"current state : step={snapshot.env.step_index} bullets={snapshot.bullet_count}")
    print(f"clearance     : {clearance(snapshot):.3f} px")
    print(f"time-to-hit   : {time_to_collision(snapshot):.3f} s (player at rest)")

    # --- 1. closed-form ballistic forecast: O(1) per bullet, no stepping -------
    forecast = BallisticPredictor().predict(snapshot, horizon, spec.dt)
    print(f"\nballistic forecast: {forecast.summary()}")

    # --- 2. full rollout: S_t -> S_t+H on a clone -----------------------------
    sim = FutureSimulator(env.world)
    states = sim.simulate_future(horizon, action=0, include_initial=True)
    print(f"rollout states: {len(states)} (S_t .. S_t+{horizon})")
    print(f"bullets at t+H: {states[-1].bullet_count}")

    # --- 3. candidate action evaluation (the MPC primitive) -------------------
    results = evaluate_candidates(sim, list(range(9)), horizon=60, objective="survival")
    print("\ncandidate actions (60-step rollout, survival objective):")
    for row in describe_results(results):
        print(
            f"  action={row['action']}  objective={row['objective']:>8.3f}  "
            f"survived={str(row['survived']):<5} terminated_at={row['terminated_at']}  "
            f"min_clearance={row['min_clearance']:>8.3f}"
        )
    best = best_candidate(results)
    print(f"  -> best action: {best.action}")

    # --- 4. future danger field (2D risk map per horizon step) ----------------
    danger = compute_danger_field(states, resolution=16.0, safety_radius=24.0)
    p = snapshot.player
    print(f"\ndanger field : grid={danger.grid.shape} resolution={danger.resolution}")
    print(f"  risk now at player({p.x:.0f},{p.y:.0f}) : {danger.risk_at(p.x, p.y, 0):.3f}")
    print(f"  max risk over horizon      : {float(danger.max_over_time().max()):.3f}")
    free_now = float(danger.safe_mask(0.01)[0].mean())
    print(f"  safe-cell fraction at t    : {free_now * 100:.1f}%")

    # same field, but rasterised straight from the analytic forecast
    fast = danger_from_forecast(
        forecast, field_w=spec.field_w, field_h=spec.field_h, resolution=16.0,
        safety_radius=24.0,
    )
    print(f"  |forecast field - rollout field| max = {np.abs(fast.grid[0] - danger.grid[0]).max():.3f}")

    env.close()


if __name__ == "__main__":
    main()
