"""Collision shapes: circle / point / rectangle hitboxes and the SDF contract.

The specification asks for a first-version 2D geometric model that can later
grow to circle / rectangle / point / custom hitboxes.  These tests pin down the
shape math against analytic answers and against the brute-force circle backend.
"""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.collision.base import risk_from_clearance
from bullet_sim.collision.circle import CircleCollision
from bullet_sim.collision.grid import make_collision_model
from bullet_sim.collision.shapes import (
    CircleHitbox,
    PointHitbox,
    RectHitbox,
    ShapedCollision,
    make_hitbox,
)
from bullet_sim.entities.bullet import BulletPool
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.world import World


def _pool(points, radius=2.0) -> BulletPool:
    pool = BulletPool(max(1, len(points)))
    for x, y in points:
        pool.spawn(x=x, y=y, vx=0.0, vy=0.0, radius=radius, ttl=1e9)
    return pool


# --------------------------------------------------------------------------
# signed distance functions
# --------------------------------------------------------------------------


def test_circle_hitbox_sdf():
    hb = CircleHitbox(3.0)
    d = hb.sdf(0.0, 0.0, np.array([5.0]), np.array([0.0]))
    assert float(d[0]) == pytest.approx(2.0)
    d = hb.sdf(0.0, 0.0, np.array([1.0]), np.array([0.0]))
    assert float(d[0]) == pytest.approx(-2.0)


def test_point_hitbox_sdf():
    hb = PointHitbox()
    d = hb.sdf(0.0, 0.0, np.array([3.0, 4.0]), np.array([0.0, 0.0]))
    assert float(d[0]) == pytest.approx(3.0)
    assert float(d[1]) == pytest.approx(4.0)


def test_rect_hitbox_sdf_outside_inside_and_on_the_edge():
    hb = RectHitbox(2.0, 1.0)
    bx = np.array([5.0, 1.0, 0.0, 3.0, 4.0])
    by = np.array([0.0, 0.0, 4.0, 1.0, 3.0])
    d = hb.sdf(0.0, 0.0, bx, by)
    assert float(d[0]) == pytest.approx(3.0)          # 5 - half_w on the x axis
    assert float(d[1]) == pytest.approx(-1.0)         # inside, 1 from the x face
    assert float(d[2]) == pytest.approx(3.0)          # 4 - half_h on the y axis
    assert float(d[3]) == pytest.approx(1.0)          # just past the x face
    assert float(d[4]) == pytest.approx(np.hypot(2.0, 2.0))  # diagonal corner


def test_make_hitbox_from_names_and_mappings():
    assert make_hitbox("circle", radius=5.0).radius == 5.0
    assert isinstance(make_hitbox("point"), PointHitbox)
    assert isinstance(make_hitbox("rect"), RectHitbox)
    assert make_hitbox({"shape": "rect", "half_w": 3.0, "half_h": 4.0}).half_h == 4.0
    assert make_hitbox(None).shape == "circle"
    assert make_hitbox(CircleHitbox(1.0)).radius == 1.0
    with pytest.raises(ValueError):
        make_hitbox("hexagon")


# --------------------------------------------------------------------------
# shaped collision backend
# --------------------------------------------------------------------------


def test_circle_hitbox_matches_the_brute_force_backend():
    rng = np.random.default_rng(0)
    n = 300
    pool = BulletPool(n)
    pool.spawn(
        x=rng.uniform(0, 640, n), y=rng.uniform(0, 480, n),
        vx=0.0, vy=0.0, radius=rng.uniform(1.0, 4.0, n), ttl=1e9,
    )
    shaped = ShapedCollision(CircleHitbox(3.0))
    brute = CircleCollision(strict=True)
    for _ in range(60):
        px, py = rng.uniform(0, 640), rng.uniform(0, 480)
        assert (
            shaped.query_hitbox(pool, CircleHitbox(3.0), px, py).hit
            == brute.query(pool, px, py, 3.0).hit
        )


def test_point_hitbox_is_stricter_than_circle_hitbox():
    pool = _pool([(4.0, 0.0)], radius=2.0)
    model = ShapedCollision()
    assert model.query_hitbox(pool, CircleHitbox(3.0), 0.0, 0.0).hit is True
    assert model.query_hitbox(pool, PointHitbox(), 0.0, 0.0).hit is False
    # a point hitbox only triggers once the bullet surface covers the point
    assert model.query_hitbox(pool, PointHitbox(), 2.5, 0.0).hit is True


def test_rect_hitbox_matches_an_analytic_corner_case():
    pool = _pool([(3.0, 3.0)], radius=1.0)
    model = ShapedCollision(RectHitbox(2.0, 2.0))
    # corner at (2,2): distance from (3,3) to the rect is sqrt(2) > 1 -> miss
    assert model.query_hitbox(pool, RectHitbox(2.0, 2.0), 0.0, 0.0).hit is False
    # move the bullet closer: (2.5, 2.5) -> sqrt(0.5) < 1 -> hit
    pool2 = _pool([(2.5, 2.5)], radius=1.0)
    assert model.query_hitbox(pool2, RectHitbox(2.0, 2.0), 0.0, 0.0).hit is True


def test_shaped_backend_counts_multiple_overlaps():
    pool = _pool([(0.0, 0.0), (1.0, 0.0), (50.0, 0.0)], radius=0.5)
    result = ShapedCollision(CircleHitbox(3.0)).query_hitbox(
        pool, CircleHitbox(3.0), 0.0, 0.0, frame=5
    )
    assert result.count == 2
    assert result.hit and result.frame == 5


def test_shaped_clearance_helper():
    pool = _pool([(10.0, 0.0)], radius=2.0)
    model = ShapedCollision()
    assert model.clearance_at(pool, CircleHitbox(3.0), 0.0, 0.0) == pytest.approx(5.0)
    assert model.clearance_at(pool, PointHitbox(), 0.0, 0.0) == pytest.approx(8.0)


def test_risk_from_clearance_is_bounded():
    assert risk_from_clearance(-1.0) == 1.0
    assert risk_from_clearance(0.0) == 1.0
    assert risk_from_clearance(16.0, 32.0) == pytest.approx(0.5)
    assert risk_from_clearance(1e9, 32.0) == 0.0
    assert 0.0 <= risk_from_clearance(3.0) <= 1.0


def test_collision_factory_exposes_the_shaped_backend():
    assert make_collision_model("shaped").name == "shaped"
    assert "rect" in make_collision_model("shaped").describe()["supported_shapes"]


# --------------------------------------------------------------------------
# world integration
# --------------------------------------------------------------------------


def test_world_upgrades_the_backend_for_a_non_circle_hitbox():
    # A scene without rectangle obstacles uses the shaped backend for a
    # non-circle hitbox; a scene *with* them uses the obstacle backend, which
    # is hitbox-capable as well.
    plain = scenario_for_level("easy", seed=1, duration=6.0).replaced(patterns=[])
    world = World(plain, player_hitbox="point", reward="zero",
                  terminate_on_collision=False)
    assert world.collision.name == "shaped"
    assert world.collision.describe()["hitbox"]["shape"] == "point"

    world_rect = World(plain, player_hitbox={"shape": "rect", "half_w": 6.0, "half_h": 3.0},
                       collision="circle", reward="zero", terminate_on_collision=False)
    assert world_rect.collision.name == "shaped"
    assert world_rect.collision.describe()["hitbox"]["shape"] == "rect"

    # the obstacle backend accepts a non-circle hitbox too
    obstacles = scenario_for_level("easy", seed=1, duration=6.0)
    world_obs = World(obstacles, player_hitbox="point", reward="zero",
                      terminate_on_collision=False)
    assert world_obs.collision.name in ("obstacle", "shaped")


def test_point_hitbox_survives_longer_than_circle_hitbox():
    """A smaller hitbox must be strictly safer - the classic danmaku tradeoff."""
    spec = scenario_for_level("medium", seed=4, duration=20.0)

    circle = World(spec, player_hitbox="circle", reward="zero",
                   terminate_on_collision=True)
    point = World(spec, player_hitbox="point", reward="zero",
                  terminate_on_collision=True)
    c_steps = p_steps = 0
    for _ in range(1500):
        if not circle.step(0).terminated:
            c_steps += 1
        else:
            break
    for _ in range(1500):
        if not point.step(0).terminated:
            p_steps += 1
        else:
            break
    assert p_steps >= c_steps
    assert point.active_bullet_count() > 0


def test_hitbox_shape_does_not_change_bullet_dynamics():
    """The hitbox is a collision concern only: it must not alter S_t otherwise."""
    spec = scenario_for_level("easy", seed=6, duration=8.0)
    a = World(spec, player_hitbox="circle", reward="zero", terminate_on_collision=False)
    b = World(spec, player_hitbox="point", reward="zero", terminate_on_collision=False)
    for _ in range(120):
        a.step(1)
        b.step(1)
    assert a.active_bullet_count() == b.active_bullet_count()
    pa = np.array([a.player[0], a.player[1]])
    pb = np.array([b.player[0], b.player[1]])
    assert np.allclose(pa, pb)
    ba = a.state.bullets.to_protocol_matrix()
    bb = b.state.bullets.to_protocol_matrix()
    assert np.allclose(ba, bb)


def test_env_reports_hitbox_in_describe():
    from bullet_sim.simulator.env import BulletHellEnv

    env = BulletHellEnv(scenario_for_level("easy", seed=1, duration=5.0),
                        player_hitbox="rect")
    assert env.describe()["world"]["collision"]["hitbox"]["shape"] == "rect"
    env.close()
