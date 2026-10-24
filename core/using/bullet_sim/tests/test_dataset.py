"""Requirement 10: dataset export / import consistency across all formats."""

from __future__ import annotations

import json

import numpy as np
import pytest

from bullet_sim.dataset.readers import load_episode, verify_roundtrip
from bullet_sim.dataset.recorder import TrajectoryRecorder, record_episode
from bullet_sim.dataset.schema import ARRAY_KEYS, TRANSITION_FIELDS
from bullet_sim.dataset.writers import save_episode, supported_formats
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


def _record(tmp_path, **kwargs):
    spec = scenario_for_level("easy", seed=17)
    rec = TrajectoryRecorder(store_hashes=True, **kwargs)
    env = BulletHellEnv(spec, seed=17, record=rec)
    env.run("random", steps=300, seed=17)
    episode = rec.finish()
    env.close()
    return spec, episode


def test_recorder_captures_full_transition_schema(tmp_path):
    spec, ep = _record(tmp_path)
    assert ep.steps > 0
    for key in ARRAY_KEYS:
        assert key in ep.arrays, f"missing array {key}"
    tr = ep.transition(0)
    for field in TRANSITION_FIELDS:
        assert field in tr, f"missing transition field {field}"
    assert tr["scenario_id"] == ep.meta["scenario_id"]
    assert tr["seed"] == 17
    assert isinstance(tr["next_state"]["bullets"], list)


@pytest.mark.parametrize("fmt", ["json", "npz", "bin"])
def test_lossless_roundtrip(tmp_path, fmt):
    spec, ep = _record(tmp_path)
    path = save_episode(ep, tmp_path / f"ep.{fmt}")
    assert verify_roundtrip(ep, path)
    loaded = load_episode(path)
    assert loaded.steps == ep.steps
    assert loaded.seed == ep.seed
    assert loaded.scenario_id == ep.scenario_id
    for key in ("player", "target", "env", "bullets", "rewards", "actions"):
        assert np.allclose(
            np.asarray(loaded.arrays[key], dtype=np.float64),
            np.asarray(ep.arrays[key], dtype=np.float64),
        )
    assert list(loaded.arrays["state_hashes"]) == list(ep.arrays["state_hashes"])


def test_csv_roundtrip_preserves_scalar_fields(tmp_path):
    spec, ep = _record(tmp_path)
    path = save_episode(ep, tmp_path / "ep.csv")
    loaded = load_episode(path)
    assert loaded.steps == ep.steps
    assert np.allclose(loaded.arrays["rewards"], ep.arrays["rewards"], atol=1e-6)
    assert np.allclose(loaded.arrays["action_vectors"], ep.arrays["action_vectors"], atol=1e-6)
    # CSV carries no S_0 block, so bullet counts are compared from S_1 onwards.
    assert np.array_equal(
        np.asarray(loaded.arrays["bullet_counts"][1:], dtype=np.int64),
        np.asarray(ep.arrays["bullet_counts"][1:], dtype=np.int64),
    )


def test_states_are_self_consistent(tmp_path):
    spec, ep = _record(tmp_path)
    s0 = ep.state(0)
    s1 = ep.state(1)
    assert s0["step_index"] == 0
    assert s1["step_index"] == 1
    assert len(s0["bullets"]) == s0["bullet_count"]
    # the initial state must match the recorded S_0
    assert ep.initial_state["player"] == pytest.approx(s0["player"])
    # bullet rows for state t must start where the offsets say they do
    for t in range(min(ep.steps + 1, 25)):
        lo = int(ep.arrays["bullets_offsets"][t])
        n = int(ep.arrays["bullet_counts"][t])
        rows = ep.arrays["bullets"][lo : lo + n]
        assert rows.shape[0] == n


def test_json_export_matches_arrays(tmp_path):
    spec, ep = _record(tmp_path)
    path = save_episode(ep, tmp_path / "ep.json", )
    payload = json.loads(open(path, encoding="utf-8").read())
    assert payload["meta"]["seed"] == 17
    assert len(payload["transitions"]) == ep.steps
    assert payload["transitions"][0]["reward"] == pytest.approx(float(ep.arrays["rewards"][0]))


def test_bullet_stride_reduces_rows_but_keeps_counts(tmp_path):
    spec = scenario_for_level("medium", seed=3)
    full = TrajectoryRecorder()
    env = BulletHellEnv(spec, seed=3, record=full)
    env.run("stay", steps=400, seed=3)
    ep_full = full.finish()
    env.close()

    sparse = TrajectoryRecorder(bullet_stride=10)
    env = BulletHellEnv(spec, seed=3, record=sparse)
    env.run("stay", steps=400, seed=3)
    ep_sparse = sparse.finish()
    env.close()

    assert ep_sparse.arrays["bullets"].shape[0] < ep_full.arrays["bullets"].shape[0]
    assert np.array_equal(ep_sparse.arrays["bullet_counts"], ep_full.arrays["bullet_counts"])
    assert ep_sparse.arrays["bullet_sampled"].sum() < ep_sparse.steps + 1


def test_load_nonexistent_dataset_raises(tmp_path):
    from bullet_sim.core.errors import DatasetError

    with pytest.raises(DatasetError):
        load_episode(tmp_path / "nope.npz")


def test_supported_formats_listed():
    assert set(supported_formats()) >= {"json", "npz", "csv", "bin"}


def test_record_episode_helper():
    from bullet_sim.scenarios.presets import scenario_for_level as sfl

    env = BulletHellEnv(sfl("easy", seed=6), seed=6)
    ep = record_episode(env, "random", steps=150, seed=6, store_hashes=True)
    env.close()
    assert ep.steps > 0
    assert ep.arrays["state_hashes"].size == ep.steps + 1


def test_episode_embeds_env_policy_and_replays_faithfully(tmp_path):
    """A dataset must carry the policy that produced it, and replay exactly."""
    from bullet_sim.simulator.replay import verify_episode

    spec = scenario_for_level("medium", seed=17, duration=20.0)
    rec = TrajectoryRecorder(store_hashes=True)
    env = BulletHellEnv(spec, seed=17, record=rec, terminate_on_collision=False)
    env.run("random", steps=500, seed=17)
    episode = rec.finish()
    env.close()

    policy = episode.meta.get("env_policy", {})
    assert policy.get("terminate_on_collision") is False
    assert episode.meta["env"]["policy"]["collision"] == "circle"
    assert episode.meta["env"]["scenario"]["seed"] == 17

    comparison = verify_episode(episode)
    assert comparison.ok, comparison.reason
    assert comparison.first_mismatch == -1
    assert comparison.expected_steps == comparison.actual_steps == episode.steps


def test_wrong_policy_is_reported_not_silently_accepted():
    """Overriding the policy away from the recording must be *detected*."""
    from bullet_sim.simulator.replay import verify_episode

    spec = scenario_for_level("medium", seed=17, duration=20.0)
    rec = TrajectoryRecorder(store_hashes=True)
    env = BulletHellEnv(spec, seed=17, record=rec, terminate_on_collision=False)
    env.run("random", steps=500, seed=17)
    episode = rec.finish()
    env.close()
    assert episode.arrays["collisions"].any(), "scene must produce a collision"

    wrong = verify_episode(episode, terminate_on_collision=True)
    assert wrong.ok is False
    assert wrong.actual_steps < wrong.expected_steps
    assert "policy" in wrong.reason
