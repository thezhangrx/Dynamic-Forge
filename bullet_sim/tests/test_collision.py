"""Requirement 6: collision detection correctness and backend agreement."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.collision.base import NullCollision
from bullet_sim.collision.circle import CircleCollision, DistanceCollision
from bullet_sim.collision.grid import UniformGridCollision, make_collision_model
from bullet_sim.entities.bullet import BulletPool


def _pool(bullets) -> BulletPool:
    pool = BulletPool(max(1, len(bullets)))
    for x, y, r in bullets:
        pool.spawn(x=x, y=y, vx=0.0, vy=0.0, radius=r, ttl=1e9)
    return pool


def test_circle_collision_hit_and_miss():
    model = CircleCollision()
    pool = _pool([(4.0, 0.0, 2.0)])
    assert model.query(pool, 0.0, 0.0, 3.0).hit is True
    pool2 = _pool([(6.0, 0.0, 2.0)])
    assert model.query(pool2, 0.0, 0.0, 3.0).hit is False


def test_circle_collision_touching_is_not_a_hit():
    model = CircleCollision(strict=True)
    pool = _pool([(5.0, 0.0, 2.0)])
    res = model.query(pool, 0.0, 0.0, 3.0)
    assert res.hit is False
    lenient = CircleCollision(strict=False)
    assert lenient.query(pool, 0.0, 0.0, 3.0).hit is True


def test_circle_collision_counts_all_overlaps_and_returns_lowest_slot():
    model = CircleCollision()
    pool = _pool([(1.0, 0.0, 2.0), (50.0, 0.0, 2.0), (0.0, 2.0, 2.0)])
    res = model.query(pool, 0.0, 0.0, 3.0)
    assert res.hit
    assert res.count == 2
    assert res.index == 0
    assert res.position == pytest.approx((1.0, 0.0))
    assert res.min_dist2 == pytest.approx(1.0)


def test_collision_empty_pool():
    model = CircleCollision()
    assert model.query(BulletPool(4), 0.0, 0.0, 3.0).hit is False


def test_null_collision_never_hits():
    model = NullCollision()
    pool = _pool([(0.0, 0.0, 10.0)])
    assert model.query(pool, 0.0, 0.0, 10.0).hit is False


def test_distance_collision_reports_clearance():
    model = DistanceCollision()
    pool = _pool([(10.0, 0.0, 2.0)])
    assert model.clearance(pool, 0.0, 0.0, 3.0) == pytest.approx(5.0)
    pool2 = _pool([(4.0, 0.0, 2.0)])
    assert model.clearance(pool2, 0.0, 0.0, 3.0) == pytest.approx(-1.0)


def test_grid_matches_brute_force_on_random_data():
    rng = np.random.default_rng(0)
    n = 400
    pool = BulletPool(n)
    pool.spawn(
        x=rng.uniform(0, 640, n),
        y=rng.uniform(0, 480, n),
        vx=0.0,
        vy=0.0,
        radius=rng.uniform(1.0, 5.0, n),
        ttl=1e9,
    )
    brute = CircleCollision()
    grid = UniformGridCollision()
    grid.configure(640.0, 480.0, 0.0)
    grid.prepare(pool)
    for _ in range(50):
        px, py, pr = rng.uniform(0, 640), rng.uniform(0, 480), 3.0
        assert grid.query(pool, px, py, pr).hit == brute.query(pool, px, py, pr).hit


def test_grid_batch_query_matches_individual_queries():
    rng = np.random.default_rng(1)
    n = 500
    pool = BulletPool(n)
    pool.spawn(
        x=rng.uniform(0, 640, n),
        y=rng.uniform(0, 480, n),
        vx=0.0,
        vy=0.0,
        radius=3.0,
        ttl=1e9,
    )
    grid = UniformGridCollision()
    grid.configure(640.0, 480.0, 0.0)
    grid.prepare(pool)
    px = rng.uniform(-20, 660, 40)
    py = rng.uniform(-20, 500, 40)
    pr = np.full(40, 3.0)
    batch = grid.query_many(pool, px, py, pr)
    brute = CircleCollision()
    single = np.array([brute.query(pool, float(x), float(y), 3.0).hit for x, y in zip(px, py)])
    assert np.array_equal(batch, single)


def test_circle_query_many_matches_single():
    rng = np.random.default_rng(2)
    pool = BulletPool(200)
    pool.spawn(
        x=rng.uniform(0, 100, 200), y=rng.uniform(0, 100, 200),
        vx=0.0, vy=0.0, radius=2.0, ttl=1e9,
    )
    model = CircleCollision()
    px = rng.uniform(0, 100, 25)
    py = rng.uniform(0, 100, 25)
    pr = np.full(25, 3.0)
    batch = model.query_many(pool, px, py, pr)
    single = np.array([model.query(pool, float(x), float(y), 3.0).hit for x, y in zip(px, py)])
    assert np.array_equal(batch, single)


def test_world_reports_collision_and_terminates():
    from bullet_sim.tests.helpers import make_world

    world = make_world(field_w=1000.0, field_h=1000.0, player_x=100.0, player_y=100.0,
                       cull_margin=1000.0)
    world.collision = CircleCollision()
    world.terminate_on_collision = True
    world.spawn_bullets(x=100.0, y=100.0, vx=0.0, vy=0.0, radius=5.0, ttl=1e9)
    result = world.step(0)
    assert result.terminated is True
    assert result.info["collision"]["hit"] is True


def test_make_collision_model_factory():
    assert make_collision_model("circle").name == "circle"
    assert make_collision_model("grid").name == "grid"
    assert make_collision_model("null").name == "null"
    with pytest.raises(ValueError):
        make_collision_model("nope")
