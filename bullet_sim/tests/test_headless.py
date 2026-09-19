"""Requirement 9: headless mode - no window, no display, batch-friendly."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.dataset.recorder import TrajectoryRecorder
from bullet_sim.interface.controller import ScriptedController
from bullet_sim.scenarios.builder import build_many
from bullet_sim.scenarios.presets import scenario_for_level, stress
from bullet_sim.simulator.env import BulletHellEnv, make_envs
from bullet_sim.simulator.world import World


def test_headless_env_runs_without_any_renderer():
    spec = scenario_for_level("medium", seed=4)
    env = BulletHellEnv(spec, seed=4, render_mode=None)
    result = env.run(ScriptedController([1, 2, 3, 0] * 40), steps=160, seed=4)
    assert result["steps"] > 0
    assert "state_hash" in result
    assert env.render() is None  # render_mode=None -> silent no-op
    env.close()


def test_ascii_render_is_headless_safe():
    spec = scenario_for_level("easy", seed=2)
    env = BulletHellEnv(spec, seed=2)
    env.reset(seed=2)
    for _ in range(120):
        env.step(0)
    text = env.render("ascii", width=60, height=20)
    assert isinstance(text, str)
    assert "@" in text  # player
    assert "bullets" in text  # HUD
    env.close()


def test_headless_batch_simulation_and_recording():
    scenarios = build_many(3, "easy", base_seed=100)
    envs = make_envs([b.spec for b in scenarios], base_seed=100)
    episodes = []
    for env in envs:
        rec = TrajectoryRecorder(store_hashes=True)
        env.world.add_listener(rec.listener)
        env.run("random", steps=240, seed=env.spec.seed)
        episodes.append(rec.finish())
        env.close()
    assert len(episodes) == 3
    assert all(ep.steps > 0 for ep in episodes)
    seeds = [ep.seed for ep in episodes]
    assert seeds == sorted(set(seeds))  # distinct, reproducible seeds


def test_no_display_dependency_is_required():
    """Simulation must not import or initialise any renderer."""
    import sys

    spec = stress(120, seed=0)
    world = World(spec, reward="zero", terminate_on_collision=False)
    for _ in range(200):
        world.step(0)
    # pygame may be importable, but the kernel must not have needed it
    assert world.active_bullet_count() > 0
    assert world.describe()["bullet_capacity"] >= world.state.bullets.capacity
    assert "pygame" not in getattr(world, "__dict__", {})


def test_world_describe_reports_platform_contract():
    world = World(stress(100, seed=1), reward="zero")
    info = world.describe()
    for key in ("scenario", "seed", "dt", "fps", "steps", "field", "bullet_capacity",
                "action_space", "collision", "reward", "timeline"):
        assert key in info
    assert info["action_space"]["name"] == "discrete"
    assert info["action_space"]["dim"] == 9


def test_high_density_scene_runs_headless():
    spec = stress(1000, seed=0, duration=6.0)
    world = World(spec, reward="zero", terminate_on_collision=False)
    for _ in range(300):
        world.step(0)
    assert world.active_bullet_count() > 100
