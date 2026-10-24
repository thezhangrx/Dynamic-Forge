"""Requirement 1 & 2: single-bullet uniform and accelerated motion."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.entities.player import P_X, P_Y
from bullet_sim.tests.helpers import DT, make_world


def test_single_bullet_uniform_motion():
    """p(t) = p0 + v*t for a single constant-velocity bullet."""
    world = make_world()
    world.spawn_bullets(x=100.0, y=200.0, vx=60.0, vy=-30.0, radius=2.0, ttl=1000.0)
    assert world.active_bullet_count() == 1

    n = 30
    for _ in range(n):
        world.step(0)

    pool = world.state.bullets
    assert pool.count == 1
    idx = pool.active_indices()[0]
    t = n * DT
    assert pool.data["x"][idx] == pytest.approx(100.0 + 60.0 * t, abs=1e-9)
    assert pool.data["y"][idx] == pytest.approx(200.0 - 30.0 * t, abs=1e-9)
    assert pool.data["vx"][idx] == pytest.approx(60.0)
    assert pool.data["vy"][idx] == pytest.approx(-30.0)
    assert pool.data["age"][idx] == pytest.approx(t)


def test_single_bullet_accelerated_motion_matches_closed_form():
    """Constant acceleration must match p0 + v0*t + 0.5*a*t^2 exactly."""
    world = make_world()
    world.spawn_bullets(
        x=0.0, y=0.0, vx=10.0, vy=0.0, ax=120.0, ay=-60.0, radius=2.0, ttl=1000.0
    )
    n = 45
    for _ in range(n):
        world.step(0)
    pool = world.state.bullets
    idx = pool.active_indices()[0]
    t = n * DT
    assert pool.data["x"][idx] == pytest.approx(10.0 * t + 0.5 * 120.0 * t * t, abs=1e-8)
    assert pool.data["y"][idx] == pytest.approx(-0.5 * 60.0 * t * t, abs=1e-8)
    assert pool.data["vx"][idx] == pytest.approx(10.0 + 120.0 * t, abs=1e-9)
    assert pool.data["vy"][idx] == pytest.approx(-60.0 * t, abs=1e-9)


def test_advance_bullets_n_equals_repeated_steps():
    """The closed-form fast-forward must equal N explicit integrations."""
    a = make_world()
    b = make_world()
    for w in (a, b):
        w.spawn_bullets(x=5.0, y=7.0, vx=30.0, vy=-20.0, ax=5.0, ay=2.0, radius=1.0, ttl=1e9)
    for _ in range(17):
        a.step(0)
    b.fast_forward_bullets(17)
    pa = a.state.bullets
    pb = b.state.bullets
    ia = pa.active_indices()[0]
    ib = pb.active_indices()[0]
    assert pa.data["x"][ia] == pytest.approx(pb.data["x"][ib], abs=1e-9)
    assert pa.data["y"][ia] == pytest.approx(pb.data["y"][ib], abs=1e-9)
    assert pa.data["vx"][ia] == pytest.approx(pb.data["vx"][ib], abs=1e-9)


def test_player_discrete_action_kinematics():
    """Discrete 'right' moves the player at exactly player.speed for dt."""
    world = make_world(field_w=10000.0, field_h=10000.0)
    x0 = float(world.player[P_X])
    y0 = float(world.player[P_Y])
    world.step(2)  # 'right'
    assert float(world.player[P_X]) == pytest.approx(x0 + world.spec.player_speed * DT)
    assert float(world.player[P_Y]) == pytest.approx(y0)


def test_diagonal_action_is_normalised():
    """Diagonal actions must not be sqrt(2) times faster than axis actions."""
    world = make_world(field_w=10000.0, field_h=10000.0)
    from bullet_sim.core.actions import DISCRETE_ACTION_INDEX

    x0 = float(world.player[P_X])
    y0 = float(world.player[P_Y])
    world.step(DISCRETE_ACTION_INDEX["up_right"])
    dx = float(world.player[P_X]) - x0
    dy = float(world.player[P_Y]) - y0
    assert np.hypot(dx, dy) == pytest.approx(world.spec.player_speed * DT)
    assert dx == pytest.approx(dy)


def test_fixed_timestep_is_independent_of_fps():
    """Distance travelled over 1s must not depend on the step size."""
    for fps in (30.0, 60.0, 120.0, 240.0):
        world = make_world(dt=1.0 / fps, duration=5.0)
        world.spawn_bullets(x=0.0, y=0.0, vx=90.0, vy=0.0, radius=1.0, ttl=1e6)
        for _ in range(int(round(fps))):
            world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()[0]
        assert pool.data["x"][idx] == pytest.approx(90.0, abs=1e-6)


def test_bullet_expires_at_ttl():
    world = make_world()
    world.spawn_bullets(x=0.0, y=0.0, vx=1.0, vy=0.0, radius=1.0, ttl=0.1)
    for _ in range(5):
        world.step(0)
    assert world.active_bullet_count() == 1  # 5/60 s < 0.1 s
    for _ in range(3):
        world.step(0)
    assert world.active_bullet_count() == 0


def test_bullets_are_culled_out_of_bounds():
    world = make_world(field_w=100.0, field_h=100.0, cull_margin=0.0)
    world.spawn_bullets(x=50.0, y=50.0, vx=1000.0, vy=0.0, radius=1.0, ttl=1e6)
    for _ in range(10):
        world.step(0)
    assert world.active_bullet_count() == 0
