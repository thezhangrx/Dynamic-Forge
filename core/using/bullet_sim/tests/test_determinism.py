"""Requirements 7 & 8: fixed-seed reproducibility and clone-then-continue consistency."""

from __future__ import annotations

import json
import subprocess
import sys

import numpy as np
import pytest

from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.world import World
from bullet_sim.tests.helpers import make_world


def _action_sequence(n: int, seed: int = 0) -> list[int]:
    rng = np.random.Generator(np.random.PCG64(seed))
    return [int(a) for a in rng.integers(0, 9, n)]


def _run_hashes(spec, seed: int, actions) -> list[str]:
    env = BulletHellEnv(spec, seed=seed, collision="null", reward="zero",
                        terminate_on_collision=False)
    env.reset(seed=seed)
    out = [env.state_hash()]
    for a in actions:
        env.step(a)
        out.append(env.state_hash())
    env.close()
    return out


# --------------------------------------------------------------------------
# 7. reproducibility
# --------------------------------------------------------------------------


def test_same_seed_same_actions_same_trajectory():
    spec = scenario_for_level("hard", seed=2024)
    actions = _action_sequence(200, seed=1)
    a = _run_hashes(spec, 2024, actions)
    b = _run_hashes(spec, 2024, actions)
    assert a == b
    assert len(a) == 201


def test_different_actions_diverge():
    spec = scenario_for_level("hard", seed=2024)
    a = _run_hashes(spec, 2024, _action_sequence(120, seed=1))
    b = _run_hashes(spec, 2024, _action_sequence(120, seed=2))
    assert a != b


def test_different_seed_changes_a_random_scene():
    spec = scenario_for_level("hard", seed=1)
    a = _run_hashes(spec, 1, [0] * 60)
    b = _run_hashes(spec, 2, [0] * 60)
    assert a != b


def test_reproducibility_survives_a_fresh_interpreter():
    """The strongest form: identical hashes across two separate processes."""
    code = (
        "import json,numpy as np;"
        "from bullet_sim.scenarios.presets import scenario_for_level;"
        "from bullet_sim.simulator.env import BulletHellEnv;"
        "spec=scenario_for_level('hard',seed=99);"
        "env=BulletHellEnv(spec,seed=99,collision='null',reward='zero',"
        "terminate_on_collision=False);"
        "env.reset(seed=99);"
        "rng=np.random.Generator(np.random.PCG64(5));"
        "h=[env.state_hash()];"
        "[h.append((env.step(int(a)),env.state_hash())[1]) for a in rng.integers(0,9,300)];"
        "print(json.dumps(h))"
    )
    runs = [
        subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
        for _ in range(2)
    ]
    first = json.loads(runs[0].stdout.strip().splitlines()[-1])
    second = json.loads(runs[1].stdout.strip().splitlines()[-1])
    assert first == second


def test_state_hash_ignores_dead_slots_and_capacity():
    """The digest depends on the live bullet set, not on pool capacity/holes."""
    a = make_world()
    b = make_world()
    bullets = dict(x=[1.0, 2.0, 3.0], y=[0.0, 0.0, 0.0], vx=[1.0, 1.0, 1.0],
                   vy=[0.0, 0.0, 0.0], radius=1.0, ttl=1e9)
    a.spawn_bullets(**bullets)
    b.spawn_bullets(**bullets)
    # b additionally spawns and then kills a fourth bullet -> identical live set,
    # different internal hole/capacity bookkeeping.
    extra = b.spawn_bullets(x=[9.0], y=[9.0], vx=[0.0], vy=[0.0], radius=1.0, ttl=1e9)
    b.state.bullets.release(extra)
    assert a.state.bullets.capacity == b.state.bullets.capacity
    assert a.active_bullet_count() == b.active_bullet_count() == 3
    assert a.state_hash() == b.state_hash()


# --------------------------------------------------------------------------
# 8. clone / restore
# --------------------------------------------------------------------------


def test_clone_is_independent_and_divergence_is_clean():
    spec = scenario_for_level("medium", seed=5)
    env = BulletHellEnv(spec, seed=5, collision="null", reward="zero",
                        terminate_on_collision=False)
    env.reset(seed=5)
    for _ in range(40):
        env.step(1)
    before = env.state_hash()

    clone = env.world.clone()
    # driving the clone must not disturb the original
    for _ in range(30):
        clone.step(3)
    assert env.state_hash() == before
    assert clone.state_hash() != before


def test_clone_continue_equals_direct_simulation():
    spec = scenario_for_level("hard", seed=13)
    actions = _action_sequence(150, seed=3)

    env = BulletHellEnv(spec, seed=13, collision="circle", reward="zero",
                        terminate_on_collision=False)
    env.reset(seed=13)
    for a in actions[:50]:
        env.step(a)

    clone = env.world.clone()
    hashes_clone = []
    for a in actions[50:]:
        clone.step(a)
        hashes_clone.append(clone.state_hash())

    hashes_direct = []
    for a in actions[50:]:
        env.step(a)
        hashes_direct.append(env.state_hash())

    assert hashes_clone == hashes_direct
    env.close()


def test_clone_state_snapshot_restore_roundtrip():
    spec = scenario_for_level("medium", seed=8)
    env = BulletHellEnv(spec, seed=8, reward="zero", terminate_on_collision=False)
    env.reset(seed=8)
    for _ in range(70):
        env.step(2)

    snapshot = env.clone_state()
    h0 = env.state_hash()

    for _ in range(25):
        env.step(4)
    h1 = env.state_hash()
    assert h1 != h0

    env.restore_state(snapshot)
    assert env.state_hash() == h0
    env.close()


def test_snapshot_serialization_roundtrip_is_lossless():
    spec = scenario_for_level("medium", seed=21)
    env = BulletHellEnv(spec, seed=21, reward="zero", terminate_on_collision=False)
    env.reset(seed=21)
    for _ in range(90):
        env.step(5)
    snap = env.get_state()
    payload = json.loads(json.dumps(snap.to_dict()))
    from bullet_sim.core.state import WorldSnapshot

    restored = WorldSnapshot.from_dict(payload)
    assert restored.state_hash() == snap.state_hash()
    assert restored.bullet_count == snap.bullet_count
    env.close()
