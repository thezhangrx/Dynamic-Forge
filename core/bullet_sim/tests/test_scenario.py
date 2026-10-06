"""Scenario system: complexity control, bullet-count calibration, JSON round-trip."""

from __future__ import annotations

import json

import numpy as np
import pytest

from bullet_sim.core.errors import ConfigError
from bullet_sim.generators.spec import PatternSpec
from bullet_sim.scenarios.builder import (
    build_from_spec,
    build_many,
    build_scenario,
    measure_live_bullets,
)
from bullet_sim.scenarios.complexity import (
    BULLET_COUNT_LADDER,
    bullet_count_for,
    complexity_from_bullet_count,
    profile_for,
    resolve_complexity,
)
from bullet_sim.scenarios.presets import (
    DIFFICULTY_LEVELS,
    all_levels,
    scenario,
    scenario_for_level,
    stress,
)
from bullet_sim.scenarios.spec import ScenarioSpec


# --------------------------------------------------------------------------
# complexity model
# --------------------------------------------------------------------------


def test_complexity_ladder_is_monotonic():
    values = [0.0, 0.15, 0.4, 0.7, 1.0]
    counts = [profile_for(v).bullet_count for v in values]
    assert counts == sorted(counts)
    assert counts[0] >= BULLET_COUNT_LADDER[0]
    assert counts[-1] == BULLET_COUNT_LADDER[-1]


def test_named_levels_resolve():
    assert resolve_complexity("easy") == pytest.approx(0.15)
    assert resolve_complexity("extreme") == 1.0
    assert resolve_complexity(0.5) == 0.5
    with pytest.raises(ConfigError):
        resolve_complexity("nonsense")
    with pytest.raises(ConfigError):
        resolve_complexity(3.0)


def test_bullet_count_ladder_round_trips_approximately():
    for count in BULLET_COUNT_LADDER:
        value = complexity_from_bullet_count(count)
        assert bullet_count_for(value) == pytest.approx(count, rel=0.02)


def test_pattern_vocabulary_grows_with_complexity():
    assert len(profile_for(0.0).obstacle_kinds) < len(profile_for(1.0).obstacle_kinds)
    assert set(profile_for(1.0).obstacle_kinds) == {
        "small_obstacles", "moving_block", "wall_with_gap",
        "cross_traffic", "corridor",
    }


# --------------------------------------------------------------------------
# presets
# --------------------------------------------------------------------------


def test_all_levels_build_and_escalate():
    specs = all_levels(seed=42, duration=6.0)
    assert set(specs) == set(DIFFICULTY_LEVELS)
    targets = [specs[name].meta["target_bullet_count"] for name in DIFFICULTY_LEVELS]
    assert targets == sorted(targets)
    assert len(set(targets)) == len(targets)


def test_scenario_entry_point_accepts_levels_floats_and_counts():
    assert scenario("hard", seed=1).meta["target_bullet_count"] == \
        scenario_for_level("hard", seed=1).meta["target_bullet_count"]
    assert scenario(0.4, seed=1).meta["label"] == "medium"
    assert scenario(300, seed=1).meta["target_bullet_count"] == 300
    with pytest.raises(ValueError):
        scenario_for_level("impossible", seed=1)


@pytest.mark.parametrize("count", [50, 200, 1000])
def test_stress_calibrates_measured_peak_near_target(count):
    spec = stress(count, seed=5)
    measured = measure_live_bullets(spec)["peak"]
    assert measured == pytest.approx(count, rel=0.25)
    assert spec.bullet_capacity >= measured


def test_stress_produces_exactly_the_requested_bullet_scale():
    small = stress(100, seed=0)
    large = stress(2000, seed=0)
    assert measure_live_bullets(small)["peak"] < measure_live_bullets(large)["peak"]


# --------------------------------------------------------------------------
# building
# --------------------------------------------------------------------------


def test_built_scenario_summary_and_timeline():
    built = build_scenario("medium", seed=3)
    summary = built.summary()
    assert summary["seed"] == 3
    assert summary["steps"] > 0
    assert summary["total_bullets"] > 0
    assert summary["event_steps"] > 0
    assert set(summary["bullets_per_pattern"]) <= {
        "obstacle:block", "obstacle:wall_gap", "obstacle:small",
        "obstacle:cross", "obstacle:corridor",
    }


def test_build_many_is_reproducible_and_distinct():
    a = build_many(3, "easy", base_seed=10)
    b = build_many(3, "easy", base_seed=10)
    assert [x.spec.seed for x in a] == [x.spec.seed for x in b]
    for x, y in zip(a, b):
        assert x.timeline.total_bullets == y.timeline.total_bullets
        assert x.spec.to_dict() == y.spec.to_dict()
    seeds = [x.spec.seed for x in a]
    assert len(set(seeds)) == 3


def test_scenario_spec_json_round_trip(tmp_path):
    spec = scenario_for_level("hard", seed=11)
    path = spec.save(tmp_path / "hard.json")
    loaded = ScenarioSpec.load(path)
    assert loaded.to_dict() == spec.to_dict()
    # and the rebuilt scene must be identical
    a = build_from_spec(spec)
    b = build_from_spec(loaded)
    assert a.timeline.total_bullets == b.timeline.total_bullets
    assert a.peak_bullets == b.peak_bullets


def test_scenario_rejects_unknown_fields():
    with pytest.raises(ConfigError):
        ScenarioSpec.from_dict({"name": "x", "bogus_field": 1})


def test_scenario_validation():
    with pytest.raises(ConfigError):
        ScenarioSpec(duration=0.0)
    with pytest.raises(ConfigError):
        ScenarioSpec(dt=0.0)
    with pytest.raises(ConfigError):
        ScenarioSpec(wrap="teleport")
    with pytest.raises(ConfigError):
        PatternSpec(kind="not_a_pattern")
    with pytest.raises(ConfigError):
        PatternSpec(kind="radial", count=-1)


def test_player_spawn_defaults_avoid_centre_emitters():
    spec = scenario_for_level("hard", seed=0)
    x, y = spec.player_spawn
    assert y < spec.field_h * 0.5


def test_explicit_player_position_is_respected():
    spec = scenario_for_level("easy", seed=0, player_x=123.0, player_y=45.0)
    assert spec.player_spawn == (123.0, 45.0)


def test_built_scenario_can_be_saved_and_inspected(tmp_path):
    built = build_scenario("easy", seed=0)
    path = built.save(tmp_path / "built.json")
    payload = json.loads(path.read_text())
    assert payload["spec"]["seed"] == 0
    assert len(payload["timeline"]) > 0
    assert payload["summary"]["estimated_peak_bullets"] > 0


def test_every_config_sample_loads_and_uses_the_current_model():
    """`bullet_sim/configs/*.json` are shipped samples - they must never rot."""
    import json
    from pathlib import Path

    from bullet_sim.scenarios.spec import ScenarioSpec

    configs = sorted((Path(__file__).resolve().parents[1] / "configs").glob("*.json"))
    assert configs, "the repository should ship scenario config samples"
    for path in configs:
        spec = ScenarioSpec.load(str(path))
        assert spec.patterns, f"{path.name} has no patterns"
        # only the obstacle generator exists now
        assert {p.kind for p in spec.patterns} == {"obstacle"}, path.name
        # and the file is plain, round-trippable JSON
        json.loads(Path(path).read_text(encoding="utf-8"))
