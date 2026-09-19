"""Bullet pool storage semantics and the action-codec contract."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.core.actions import (
    AccelerationActionCodec,
    ActionContext,
    DISCRETE_ACTION_NAMES,
    DiscreteActionCodec,
    VelocityActionCodec,
    codec_from_spec,
    make_codec,
)
from bullet_sim.core.errors import ConfigError
from bullet_sim.entities.bullet import BulletPool, PROTOCOL_FIELDS


def test_alloc_returns_ascending_slots():
    pool = BulletPool(8)
    assert list(pool.alloc(3)) == [0, 1, 2]
    assert list(pool.alloc(2)) == [3, 4]
    assert pool.count == 5
    assert pool.free_count == 3


def test_release_then_alloc_reuses_slots_deterministically():
    pool = BulletPool(8)
    pool.alloc(4)
    pool.release(np.array([1, 3], dtype=np.int32))
    assert list(pool.alloc(2)) == [3, 1]
    assert pool.count == 4


def test_kill_mask_only_kills_live_slots():
    pool = BulletPool(6)
    pool.spawn(x=np.arange(4.0), y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    mask = np.array([True, False, True, False, True, False])
    assert pool.kill_mask(mask) == 2
    assert pool.count == 2
    assert list(pool.active_indices()) == [1, 3]


def test_compact_preserves_slot_order_and_density():
    pool = BulletPool(8)
    pool.spawn(x=[10.0, 20.0, 30.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    pool.release(np.array([1], dtype=np.int32))
    assert list(pool.active_indices()) == [0, 2]
    assert pool.compact() == 2
    assert list(pool.active_indices()) == [0, 1]
    assert list(pool.data["x"][:2]) == [10.0, 30.0]
    assert pool.alive[:2].all() and not pool.alive[2:].any()


def test_pool_grows_transparently():
    pool = BulletPool(2)
    pool.spawn(x=[1.0, 2.0, 3.0, 4.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    assert pool.capacity >= 4
    assert pool.count == 4
    assert sorted(pool.active_indices().tolist()) == [0, 1, 2, 3]


def test_pool_clear_resets_free_order():
    pool = BulletPool(4)
    pool.spawn(x=[1.0, 2.0], y=0.0, vx=0.0, vy=0.0, radius=1.0, ttl=1e9)
    pool.clear()
    assert pool.count == 0
    assert list(pool.alloc(2)) == [0, 1]


def test_pool_protocol_matrix_field_order():
    pool = BulletPool(2)
    pool.spawn(x=[1.0, 2.0], y=[3.0, 4.0], vx=[5.0, 6.0], vy=[7.0, 8.0], radius=1.0, ttl=9.0)
    mat = pool.to_protocol_matrix()
    assert mat.shape == (2, len(PROTOCOL_FIELDS))
    assert mat[0, 0] == 1.0 and mat[0, 1] == 3.0 and mat[0, 2] == 5.0 and mat[0, 3] == 7.0


def test_pool_dict_roundtrip():
    pool = BulletPool(4)
    pool.spawn(x=[1.0, 2.0], y=[3.0, 4.0], vx=[0.0, 0.0], vy=[0.0, 0.0], radius=2.0, ttl=5.0)
    restored = BulletPool.from_dict(pool.to_dict())
    for name in pool.data:
        assert np.allclose(pool.data[name][: pool.count], restored.data[name][: restored.count])
    assert restored.count == pool.count


def test_pool_copy_is_independent():
    pool = BulletPool(4)
    pool.spawn(x=[1.0], y=[0.0], vx=[0.0], vy=[0.0], radius=1.0, ttl=1e9)
    other = pool.copy()
    other.data["x"][0] = 99.0
    assert pool.data["x"][0] == 1.0


def test_pool_free_count_and_len():
    pool = BulletPool(5)
    assert len(pool) == 0
    pool.alloc(3)
    assert len(pool) == 3 and pool.free_count == 2


# --------------------------------------------------------------------------
# actions
# --------------------------------------------------------------------------


def test_discrete_codec_roundtrips_every_action():
    codec = DiscreteActionCodec()
    ctx = ActionContext(player_speed=200.0)
    assert codec.dim == 9
    assert codec.names == DISCRETE_ACTION_NAMES
    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        v = codec.encode(i, ctx)
        assert codec.decode(v) == i, name
        assert codec.normalize(name) == i
        assert codec.normalize(i) == i


def test_discrete_codec_rejects_out_of_range():
    codec = DiscreteActionCodec()
    with pytest.raises(ConfigError):
        codec.normalize(9)
    with pytest.raises(ConfigError):
        codec.normalize("jump")


def test_velocity_codec_clamps_to_player_speed():
    codec = VelocityActionCodec(limit_to_player_speed=True)
    ctx = ActionContext(player_speed=10.0)
    v = codec.encode([100.0, 0.0], ctx)
    assert np.linalg.norm(v) == pytest.approx(10.0)
    loose = VelocityActionCodec(limit_to_player_speed=False)
    assert np.allclose(loose.encode([100.0, 0.0], ctx), [100.0, 0.0])


def test_acceleration_codec_integrates_velocity():
    codec = AccelerationActionCodec(max_accel=100.0)
    ctx = ActionContext(player_speed=50.0, vx=10.0, vy=0.0, dt=1.0)
    v = codec.encode([20.0, 0.0], ctx)
    assert np.allclose(v, [30.0, 0.0])


def test_codec_factory_and_config_forms():
    assert isinstance(make_codec("discrete"), DiscreteActionCodec)
    assert isinstance(make_codec("velocity"), VelocityActionCodec)
    assert isinstance(make_codec("acceleration"), AccelerationActionCodec)
    assert isinstance(codec_from_spec(None), DiscreteActionCodec)
    assert isinstance(codec_from_spec("velocity"), VelocityActionCodec)
    assert isinstance(codec_from_spec({"kind": "acceleration"}), AccelerationActionCodec)
    with pytest.raises(ConfigError):
        make_codec("telepathy")
