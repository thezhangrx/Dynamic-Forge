"""Gymnasium-style interface, spaces, observation encoding and external controllers."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.core.actions import (
    AccelerationActionCodec,
    ActionContext,
    DiscreteActionCodec,
    VelocityActionCodec,
)
from bullet_sim.interface.controller import (
    CallableController,
    ConstantController,
    PolicyAdapter,
    RandomController,
    ScriptedController,
    controller_from,
)
from bullet_sim.interface.space import Box, Discrete, ObservationEncoder
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.rewards import make_reward


def _env(**kw):
    spec = scenario_for_level("medium", seed=12)
    return BulletHellEnv(spec, seed=12, **kw)


def test_reset_and_step_return_gymnasium_signature():
    env = _env()
    obs, info = env.reset(seed=12)
    assert isinstance(obs, dict) and isinstance(info, dict)
    obs, reward, terminated, truncated, info = env.step(1)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)
    assert info["step_index"] == 1
    env.close()


def test_observation_blocks_and_flattening():
    env = _env(normalize=True, max_bullets=4096)
    obs, _ = env.reset(seed=12)
    assert obs["player"].shape == (6,)
    assert obs["target"].shape == (4,)
    assert obs["env"].shape == (6,)
    assert obs["bullets"].shape[1] == 7
    assert obs["bullets_mask"].shape[0] == obs["bullets"].shape[0]
    flat = env.encoder.flatten(obs)
    assert flat.shape == (env.encoder.flat_dim,)
    assert flat.dtype == np.float32
    # bullet count is the last element of the flat vector
    assert int(flat[-1]) == obs["bullets"].shape[0]
    env.close()


def test_flatten_is_fixed_size_as_bullets_grow():
    env = _env(max_bullets=512)
    obs, _ = env.reset(seed=12)
    sizes = {env.encoder.flatten(obs).shape[0]}
    for _ in range(200):
        obs, *_ = env.step(0)
        sizes.add(env.encoder.flatten(obs).shape[0])
    assert len(sizes) == 1
    env.close()


def test_action_space_matches_discrete_contract():
    env = _env()
    space = env.action_space
    assert isinstance(space, Discrete)
    assert space.n == 9
    assert space.names[0] == "stay"
    assert space.contains(8) and not space.contains(9)
    env.close()


def test_box_space_for_velocity_actions():
    env = _env(codec="velocity")
    assert isinstance(env.action_space, Box)
    obs, _ = env.reset(seed=12)
    v0 = env.world.player.copy()
    env.step(np.array([10.0, 0.0]))
    assert env.world.player[0] > v0[0]
    env.close()


def test_external_controllers_drive_the_env():
    env = _env()
    env.reset(seed=12)

    scripted = ScriptedController([1, 2, 3, 4])
    assert [scripted.act({}, {}) for _ in range(4)] == [1, 2, 3, 4]
    assert scripted.act({}, {}) == 0  # default once exhausted
    scripted.reset({}, {})

    const = ConstantController(3)
    assert const.act({}, {}) == 3

    rnd_a = RandomController(env.action_space, seed=7)
    rnd_b = RandomController(env.action_space, seed=7)
    assert [rnd_a.act({}, {}) for _ in range(20)] == [rnd_b.act({}, {}) for _ in range(20)]

    callable_ctrl = CallableController(lambda obs, info: 2)
    assert callable_ctrl.act({}, {}) == 2
    env.close()


def test_policy_adapter_accepts_predict_style_policies():
    class Policy:
        def __init__(self):
            self.n = 0

        def predict(self, obs, deterministic=True):
            self.n += 1
            return 4, None

    adapter = PolicyAdapter(Policy())
    assert adapter.act({"player": np.zeros(6)}, {}) == 4
    assert adapter.policy.n == 1


def test_controller_from_config_forms():
    assert isinstance(controller_from(None), ConstantController)
    assert isinstance(controller_from("random", space=Discrete(9)), RandomController)
    assert isinstance(controller_from({"kind": "scripted", "actions": [1, 2]}), ScriptedController)
    assert controller_from("constant:5").action == 5


def test_env_run_with_external_controller_and_diagnostics():
    env = _env(terminate_on_collision=False)
    calls = {"n": 0}

    def policy(obs, info):
        calls["n"] += 1
        return int(calls["n"] % 9)

    result = env.run(policy, steps=100, seed=12)
    assert result["steps"] == 100
    assert calls["n"] == 100
    assert "return" in result
    env.close()


def test_get_state_and_state_hash_are_exposed():
    env = _env()
    env.reset(seed=12)
    for _ in range(30):
        env.step(1)
    snap = env.get_state()
    assert snap.bullet_count == env.active_bullet_count()
    assert isinstance(env.state_hash(), str) and len(env.state_hash()) == 32
    env.close()


def test_env_context_manager_closes():
    with _env() as env:
        env.reset(seed=12)
        env.step(0)
    assert env._closed is True
    with pytest.raises(RuntimeError):
        env.step(0)


def test_observation_encoder_normalisation_bounds():
    spec = scenario_for_level("medium", seed=12)
    env = BulletHellEnv(spec, seed=12, normalize=True)
    obs, _ = env.reset(seed=12)
    for _ in range(60):
        obs, *_ = env.step(0)
    assert 0.0 <= float(obs["player"][0]) <= 1.0
    assert 0.0 <= float(obs["player"][1]) <= 1.0
    env.close()


def test_reward_functions_are_pluggable():
    spec = scenario_for_level("easy", seed=1)
    env = BulletHellEnv(spec, seed=1, reward="collision_only",
                        terminate_on_collision=False)
    env.reset(seed=1)
    total = 0.0
    for _ in range(50):
        _, r, *_ = env.step(0)
        total += r
    zero_env = BulletHellEnv(spec, seed=1, reward="zero", terminate_on_collision=False)
    zero_env.reset(seed=1)
    _, r, *_ = zero_env.step(0)
    assert r == 0.0
    assert make_reward("survival").name == "survival"
    env.close()
    zero_env.close()


def test_action_codecs_emit_desired_velocity():
    ctx = ActionContext(player_speed=200.0, vx=10.0, vy=0.0, dt=0.5)
    d = DiscreteActionCodec()
    assert np.allclose(d.encode(0, ctx), [0.0, 0.0])
    assert np.allclose(d.encode("right", ctx), [200.0, 0.0])
    v = VelocityActionCodec(limit_to_player_speed=False)
    assert np.allclose(v.encode([5.0, -5.0], ctx), [5.0, -5.0])
    a = AccelerationActionCodec()
    assert np.allclose(a.encode([10.0, 0.0], ctx), [15.0, 0.0])
