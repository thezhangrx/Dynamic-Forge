"""Bullet data-structure and motion extensions: id, angle, angular_velocity.

Covers the specification's bullet fields plus the two motion primitives:

* straight / accelerated bullets stay analytically exact,
* curved bullets (``angular_velocity != 0``) follow the exact circular arc,
* ``angle`` always tracks the velocity, and
* ``id`` is stable across slot recycling and ``compact()``.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from bullet_sim.collision.circle import CircleCollision
from bullet_sim.collision.shapes import PointHitbox, ShapedCollision
from bullet_sim.entities.bullet import PROTOCOL_FIELDS, BulletPool
from bullet_sim.physics.motion import advance_bullets_n, integrate_bullets
from bullet_sim.tests.helpers import DT, make_world

# --------------------------------------------------------------------------
# bullet fields
# --------------------------------------------------------------------------


def test_bullet_pool_exposes_every_required_field():
    required = {
        "id", "x", "y", "vx", "vy", "ax", "ay", "angle", "angular_velocity",
        "radius", "ttl", "type_id", "group_id",
    }
    pool = BulletPool(4)
    assert required <= set(pool.data)
    assert "alive" in pool.__slots__  # active mask
    assert "age" in pool.data


def test_ids_are_monotonic_and_unique():
    pool = BulletPool(8)
    a = pool.spawn(x=[0.0, 1.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    b = pool.spawn(x=[2.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    ids = pool.data["id"]
    assert ids[a].tolist() == [1, 2]
    assert ids[b].tolist() == [3]
    assert len(set(ids[: pool.count].tolist())) == pool.count


def test_ids_survive_release_and_slot_reuse():
    pool = BulletPool(4)
    slots = pool.spawn(x=[0.0, 1.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    first_id = int(pool.data["id"][slots[0]])
    pool.release(slots[:1])
    new = pool.spawn(x=[9.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    assert int(new[0]) == int(slots[0])  # same slot ...
    assert int(pool.data["id"][new[0]]) != first_id  # ... new identity


def test_ids_survive_compact():
    pool = BulletPool(6)
    pool.spawn(x=[10.0, 20.0, 30.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    pool.release(np.array([1], dtype=np.int32))
    before = pool.data["id"][pool.active_indices()].tolist()
    pool.compact()
    after = pool.data["id"][: pool.count].tolist()
    assert before == after


def test_id_counter_roundtrips_through_serialisation():
    pool = BulletPool(4)
    pool.spawn(x=[0.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    restored = BulletPool.from_dict(pool.to_dict())
    assert restored.next_id == pool.next_id
    # and the restored pool keeps handing out fresh ids
    idx = restored.spawn(x=[1.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    assert int(restored.data["id"][idx[0]]) == pool.next_id


def test_pool_reset_restarts_the_id_sequence():
    pool = BulletPool(4)
    pool.spawn(x=[0.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    assert pool.next_id == 2
    pool.clear()
    assert pool.next_id == 1
    idx = pool.spawn(x=[0.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    assert int(pool.data["id"][idx[0]]) == 1


def test_protocol_fields_include_the_new_columns():
    assert PROTOCOL_FIELDS[:8] == (
        "x", "y", "vx", "vy", "ax", "ay", "angle", "angular_velocity"
    )
    assert "id" in PROTOCOL_FIELDS
    # v3 appends the DynamicObstacle geometry
    assert PROTOCOL_FIELDS[-4:] == ("shape", "half_w", "half_h", "rotation")


# --------------------------------------------------------------------------
# angle bookkeeping
# --------------------------------------------------------------------------


def test_angle_tracks_velocity_every_step():
    pool = BulletPool(2)
    idx = pool.spawn(x=0.0, y=0.0, vx=1.0, vy=0.0, ax=0.0, ay=5.0,
                     radius=1.0, ttl=1e9)
    assert pool.data["angle"][idx[0]] == pytest.approx(0.0)
    for _ in range(10):
        integrate_bullets(pool, DT)
    slot = int(idx[0])
    expected = math.atan2(float(pool.data["vy"][slot]), float(pool.data["vx"][slot]))
    assert float(pool.data["angle"][slot]) == pytest.approx(expected, abs=1e-12)


def test_angle_is_updated_by_the_world_step():
    world = make_world(field_w=1e5, field_h=1e5, player_x=5e4, player_y=5e4,
                       cull_margin=1e5)
    world.spawn_bullets(x=0.0, y=0.0, vx=0.0, vy=0.0, ax=10.0, ay=0.0,
                        radius=1.0, ttl=1e9)
    for _ in range(30):
        world.step(0)
    slot = int(world.state.bullets.active_indices()[0])
    assert float(world.state.bullets.data["angle"][slot]) == pytest.approx(0.0)


# --------------------------------------------------------------------------
# curved motion (angular_velocity)
# --------------------------------------------------------------------------


def test_curved_bullet_follows_an_exact_circular_arc():
    """omega != 0 must produce the closed-form arc, not an Euler approximation."""
    v = 50.0
    omega = 2.0  # rad/s
    radius = v / omega
    steps = 60
    pool = BulletPool(1)
    idx = pool.spawn(x=0.0, y=0.0, vx=v, vy=0.0, radius=1.0, ttl=1e9,
                     angular_velocity=omega)
    total_t = 0.0
    for _ in range(steps):
        integrate_bullets(pool, DT)
        total_t += DT
    slot = int(idx[0])
    theta = omega * total_t
    # exact circle: start at origin heading +x, centre at (0, R)
    expected_x = radius * math.sin(theta)
    expected_y = radius * (1.0 - math.cos(theta))
    assert float(pool.data["x"][slot]) == pytest.approx(expected_x, abs=1e-9)
    assert float(pool.data["y"][slot]) == pytest.approx(expected_y, abs=1e-9)
    # speed is preserved and the heading advanced by theta
    assert math.hypot(float(pool.data["vx"][slot]), float(pool.data["vy"][slot])) == pytest.approx(v, abs=1e-9)
    assert float(pool.data["angle"][slot]) == pytest.approx(theta, abs=1e-9)


def test_curved_motion_keeps_a_constant_radius_from_the_centre():
    v, omega = 80.0, 1.5
    radius = v / omega
    pool = BulletPool(1)
    idx = pool.spawn(x=10.0, y=-5.0, vx=0.0, vy=v, radius=1.0, ttl=1e9,
                     angular_velocity=omega)
    centre = np.array([10.0 - radius, -5.0])
    for _ in range(120):
        integrate_bullets(pool, DT)
    slot = int(idx[0])
    d = np.hypot(pool.data["x"][slot] - centre[0], pool.data["y"][slot] - centre[1])
    assert d == pytest.approx(radius, rel=1e-9)


def test_zero_angular_velocity_is_bit_identical_to_the_straight_path():
    a = BulletPool(1)
    b = BulletPool(1)
    for pool, omega in ((a, 0.0), (b, 0.0)):
        pool.spawn(x=1.0, y=2.0, vx=30.0, vy=-10.0, ax=2.0, ay=1.0,
                   radius=1.0, ttl=1e9, angular_velocity=omega)
    for _ in range(30):
        integrate_bullets(a, DT)
        integrate_bullets(b, DT)
    for name in ("x", "y", "vx", "vy", "angle"):
        assert a.data[name][0] == b.data[name][0]


def test_curved_and_straight_bullets_coexist_in_one_batch():
    pool = BulletPool(2)
    idx = pool.spawn(
        x=[0.0, 0.0], y=[0.0, 0.0], vx=[10.0, 10.0], vy=[0.0, 0.0],
        radius=1.0, ttl=1e9, angular_velocity=[0.0, 1.0],
    )
    for _ in range(20):
        integrate_bullets(pool, DT)
    straight, curved = int(idx[0]), int(idx[1])
    assert float(pool.data["y"][straight]) == pytest.approx(0.0)
    assert float(pool.data["y"][curved]) > 0.0
    assert float(pool.data["angle"][straight]) == pytest.approx(0.0)
    assert float(pool.data["angle"][curved]) == pytest.approx(1.0 * 20 * DT)


def test_advance_bullets_n_matches_repeated_steps_for_curved_bullets():
    a = BulletPool(1)
    b = BulletPool(1)
    for pool in (a, b):
        pool.spawn(x=0.0, y=0.0, vx=40.0, vy=0.0, radius=1.0, ttl=1e9,
                   angular_velocity=2.5)
    for _ in range(15):
        integrate_bullets(a, DT)
    advance_bullets_n(b, DT, 15)
    for name in ("x", "y", "vx", "vy", "angle"):
        assert a.data[name][0] == pytest.approx(b.data[name][0], abs=1e-12)


def test_advance_bullets_n_still_closed_form_for_straight_bullets():
    a = BulletPool(1)
    b = BulletPool(1)
    for pool in (a, b):
        pool.spawn(x=0.0, y=0.0, vx=10.0, vy=5.0, ax=1.0, ay=-0.5,
                   radius=1.0, ttl=1e9)
    for _ in range(40):
        integrate_bullets(a, DT)
    advance_bullets_n(b, DT, 40)
    for name in ("x", "y", "vx", "vy"):
        assert a.data[name][0] == pytest.approx(b.data[name][0], abs=1e-9)


def test_spawn_pattern_emits_curved_bullets():
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.world import World

    PLAYER_SCALE = 1.0

    world = World(scenario_for_level("easy", seed=1, duration=6.0),
                  reward="zero", terminate_on_collision=False)
    world.reset()
    for _ in range(30):
        world.step(0)
    slots = world.spawn_pattern(
        {"kind": "obstacle", "params": {"layout": "small", "size": PLAYER_SCALE}},
        count=1, speed=60.0, angular_velocity=3.0,
    )
    assert len(slots) == 1
    assert float(world.state.bullets.data["angular_velocity"][slots[0]]) == pytest.approx(3.0)
    angle0 = float(world.state.bullets.data["angle"][slots[0]])
    for _ in range(10):
        world.step(0)
    angle1 = float(world.state.bullets.data["angle"][slots[0]])
    assert abs(angle1 - angle0) > 0.1  # the shot curves


# --------------------------------------------------------------------------
# collision reports the new identifiers
# --------------------------------------------------------------------------


def test_collision_reports_bullet_id_frame_distance_and_risk():
    pool = BulletPool(3)
    pool.spawn(x=[50.0, 4.0, 60.0], y=0.0, vx=0.0, vy=0.0, radius=2.0, ttl=1e9)
    model = CircleCollision()
    result = model.query(pool, 0.0, 0.0, 3.0, frame=42)
    assert result.hit
    assert result.bullet_id == int(pool.data["id"][1])
    assert result.frame == 42
    assert result.min_distance == pytest.approx(4.0 - 2.0 - 3.0)
    assert result.risk == 1.0
    payload = result.to_dict()
    for key in ("hit", "bullet_id", "frame", "min_distance", "risk", "min_dist2"):
        assert key in payload


def test_collision_reports_proximity_risk_below_the_hit_threshold():
    pool = BulletPool(1)
    pool.spawn(x=[20.0], y=0.0, vx=0.0, vy=0.0, radius=2.0, ttl=1e9)
    result = CircleCollision(risk_radius=32.0).query(pool, 0.0, 0.0, 3.0, frame=1)
    assert not result.hit
    assert result.min_distance == pytest.approx(15.0)
    assert 0.0 < result.risk < 1.0
    assert result.risk == pytest.approx(1.0 - 15.0 / 32.0)


def test_world_info_exposes_collision_identity():
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.world import World

    world = World(scenario_for_level("easy", seed=2, duration=6.0),
                  collision="circle", reward="zero", terminate_on_collision=True)
    world.reset()
    for _ in range(60):
        world.step(0)
    world.spawn_bullets(x=float(world.player[0]), y=float(world.player[1]),
                        vx=0.0, vy=0.0, radius=4.0, ttl=1e9)
    result = world.step(0)
    info = result.info["collision"]
    assert info["hit"] is True
    assert info["bullet_id"] > 0
    assert info["frame"] == result.info["step_index"]
    assert info["min_distance"] < 0.0


def test_shaped_backend_reports_the_same_identifiers():
    pool = BulletPool(2)
    pool.spawn(x=[0.5, 40.0], y=0.0, vx=0.0, vy=0.0, radius=2.0, ttl=1e9)
    model = ShapedCollision(PointHitbox())
    result = model.query_hitbox(pool, PointHitbox(), 0.0, 0.0, frame=9)
    assert result.hit and result.bullet_id == int(pool.data["id"][0])
    assert result.frame == 9
    assert result.min_distance < 0.0
    assert result.risk == 1.0
