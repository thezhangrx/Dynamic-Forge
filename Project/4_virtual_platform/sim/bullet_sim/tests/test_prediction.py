"""Future simulation, predictive baselines and danger fields."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.prediction.ballistic import BallisticPredictor
from bullet_sim.prediction.danger_field import compute_danger_field, danger_from_forecast
from bullet_sim.prediction.rollout import (
    FutureSimulator,
    best_candidate,
    clearance,
    evaluate_candidates,
    time_to_collision,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.world import World
from bullet_sim.tests.helpers import make_world


def _env(level="medium", seed=9, **kw):
    spec = scenario_for_level(level, seed=seed)
    return BulletHellEnv(spec, seed=seed, reward="zero", terminate_on_collision=False, **kw)


# --------------------------------------------------------------------------
# clone / simulate_future
# --------------------------------------------------------------------------


def test_simulate_future_returns_horizon_plus_initial():
    env = _env()
    env.reset(seed=9)
    for _ in range(60):
        env.step(0)
    before = env.state_hash()
    states = env.simulate_future(1, horizon=40)
    assert len(states) == 41
    # the live world must be untouched
    assert env.state_hash() == before
    env.close()


def test_simulate_future_matches_direct_stepping():
    env = _env()
    env.reset(seed=9)
    for _ in range(30):
        env.step(0)
    future = env.world.simulate_future([1] * 25, horizon=25, include_initial=False)
    clone = env.world.clone()
    direct = []
    for _ in range(25):
        clone.step(1)
        direct.append(clone.state_hash())
    assert [s.state_hash() for s in future] == direct
    env.close()


def test_clone_state_is_a_private_copy():
    env = _env()
    env.reset(seed=9)
    for _ in range(20):
        env.step(2)
    snap = env.clone_state()
    count = snap.bullet_count
    for _ in range(40):
        env.step(2)
    assert snap.bullet_count == count
    env.close()


def test_world_rollout_final_reports_rewards():
    world = World(scenario_for_level("medium", seed=9), reward="zero",
                  terminate_on_collision=False)
    state, rewards = world.rollout_final([0] * 30, horizon=30)
    assert len(rewards) == 30
    assert state.env.step_index == 30


# --------------------------------------------------------------------------
# ballistic predictor
# --------------------------------------------------------------------------


def test_ballistic_forecast_matches_actual_bullet_motion():
    world = make_world(field_w=1e6, field_h=1e6, player_x=5e5, player_y=5e5,
                       cull_margin=1e6)
    world.spawn_bullets(
        x=100.0, y=100.0, vx=50.0, vy=-20.0, ax=10.0, ay=4.0, radius=2.0, ttl=1e6
    )
    snapshot = world.get_state()
    horizon, dt = 40, world.spec.dt
    forecast = BallisticPredictor().predict(snapshot, horizon, dt)
    assert forecast.positions.shape == (1, horizon, 2)
    for _ in range(horizon):
        world.step(0)
    actual = world.get_state()
    pool = actual.bullets
    idx = pool.active_indices()[0]
    assert forecast.positions[0, -1, 0] == pytest.approx(pool.data["x"][idx], abs=1e-8)
    assert forecast.positions[0, -1, 1] == pytest.approx(pool.data["y"][idx], abs=1e-8)


def test_ballistic_forecast_respects_ttl_and_bounds():
    spec = make_world(field_w=200.0, field_h=200.0, cull_margin=0.0)
    spec.spawn_bullets(x=10.0, y=10.0, vx=100.0, vy=0.0, radius=2.0, ttl=0.2)
    forecast = BallisticPredictor().predict(spec.get_state(), 60, spec.spec.dt)
    assert forecast.valid[0, 0]
    assert not forecast.valid[0, -1]  # expired and/or out of bounds
    assert forecast.valid.sum() < 60
    summary = forecast.summary()
    assert summary["method"] == "ballistic"
    assert summary["n_bullets"] == 1


def test_future_simulator_ballistic_matches_step_rollout_for_linear_bullets():
    world = World(scenario_for_level("easy", seed=3), reward="zero",
                  terminate_on_collision=False)
    for _ in range(30):
        world.step(0)
    sim = FutureSimulator(world)
    forecast = sim.ballistic_forecast(30)
    states = sim.simulate_future(30, action=0, include_initial=False)
    final = states[-1].get_state() if hasattr(states[-1], "get_state") else states[-1]
    pool = final.bullets
    idx = pool.active_indices()
    # match by slot id
    by_slot = {int(s): i for i, s in enumerate(forecast.indices)}
    checked = 0
    for k, slot in enumerate(idx):
        row = by_slot.get(int(slot))
        if row is None:
            continue
        if not forecast.valid[row, -1]:
            continue
        assert forecast.positions[row, -1, 0] == pytest.approx(pool.data["x"][slot], abs=1e-6)
        assert forecast.positions[row, -1, 1] == pytest.approx(pool.data["y"][slot], abs=1e-6)
        checked += 1
    assert checked > 0


# --------------------------------------------------------------------------
# risk primitives
# --------------------------------------------------------------------------


def test_clearance_and_time_to_collision_are_analytic():
    world = make_world(field_w=1e5, field_h=1e5, player_x=0.0, player_y=0.0,
                       cull_margin=1e5)
    world.spawn_bullets(x=50.0, y=0.0, vx=-10.0, vy=0.0, radius=2.0, ttl=1e6)
    snap = world.get_state()
    assert clearance(snap) == pytest.approx(50.0 - 2.0 - snap.player.radius)
    # player at rest: reaches the player surface after (50 - r_b - r_p)/10 seconds
    distance = 50.0 - 2.0 - snap.player.radius
    assert time_to_collision(snap) == pytest.approx(distance / 10.0, rel=1e-9)


def test_time_to_collision_infinite_when_moving_away():
    world = make_world(field_w=1e5, field_h=1e5, player_x=0.0, player_y=0.0,
                       cull_margin=1e5)
    world.spawn_bullets(x=50.0, y=0.0, vx=10.0, vy=0.0, radius=2.0, ttl=1e6)
    assert time_to_collision(world.get_state()) == float("inf")


def test_evaluate_candidates_and_best_selection():
    env = _env()
    env.reset(seed=9)
    for _ in range(90):
        env.step(0)
    sim = FutureSimulator(env.world)
    results = evaluate_candidates(sim, list(range(9)), horizon=30, objective="survival")
    assert len(results) == 9
    best = best_candidate(results)
    assert best is not None
    assert best.objective == max(r.objective for r in results)
    env.close()


def test_env_evaluate_actions_helper():
    env = _env()
    env.reset(seed=9)
    for _ in range(60):
        env.step(0)
    out = env.evaluate_actions([0, 1, 2], horizon=15)
    assert len(out) == 3
    assert all("final_state" in item for item in out)
    env.close()


# --------------------------------------------------------------------------
# danger field
# --------------------------------------------------------------------------


def test_danger_field_from_rollout():
    env = _env()
    env.reset(seed=9)
    for _ in range(120):
        env.step(0)
    states = env.simulate_future(0, horizon=20)
    danger = compute_danger_field(states, resolution=16.0)
    assert danger.horizon == 21
    assert danger.grid.shape == (21, danger.rows, danger.cols)
    assert 0.0 <= float(danger.grid.min()) and float(danger.grid.max()) <= 1.0
    assert danger.safe_mask(0.0).shape == danger.grid.shape
    p = states[0].player
    assert danger.risk_at(p.x, p.y, 0) >= 0.0
    info = danger.to_dict()
    assert info["rows"] == danger.rows and info["cols"] == danger.cols
    env.close()


def test_danger_field_marks_bullet_cell_as_dangerous():
    world = make_world(field_w=200.0, field_h=200.0, player_x=10.0, player_y=10.0,
                       cull_margin=0.0)
    world.spawn_bullets(x=100.0, y=100.0, vx=0.0, vy=0.0, radius=2.0, ttl=1e6)
    states = [world.get_state()]
    danger = compute_danger_field(states, resolution=10.0, safety_radius=10.0)
    assert danger.risk_at(100.0, 100.0, 0) == pytest.approx(1.0)
    assert danger.risk_at(10.0, 10.0, 0) < 1.0


def test_danger_from_forecast_without_stepping():
    world = make_world(field_w=200.0, field_h=200.0, player_x=10.0, player_y=10.0,
                       cull_margin=0.0)
    world.spawn_bullets(x=0.0, y=100.0, vx=100.0, vy=0.0, radius=2.0, ttl=1e6)
    forecast = BallisticPredictor().predict(world.get_state(), 20, world.spec.dt)
    danger = danger_from_forecast(
        forecast, field_w=200.0, field_h=200.0, resolution=10.0, safety_radius=10.0
    )
    assert danger.horizon == 20
    assert danger.method == "forecast_splat_dilate"
    # at the last horizon step the bullet is near x = 0 + 100*20*dt
    x_end = 100.0 * 20 * world.spec.dt
    assert danger.risk_at(x_end, 100.0, 19) > 0.0
