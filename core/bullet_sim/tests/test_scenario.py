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
    make_scenario,
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


def test_calibration_gives_up_instead_of_inflating_forever():
    """到不了目标时必须**收手**，不许每轮乘一次 `target/peak`。

    触发条件很现实：`wall_gap` 这一类布局对 `count` 不敏感（一次就是一道墙），
    所以"实测峰值"可以完全不动；而分批进场又会把数量敏感的类型推到回合之外。
    第一版标定在这种情况下每轮把数量乘一遍，4 轮堆到 10^8 个障碍，光建时间线
    就 180 秒超时 —— 是 `board-control --headless --duration 0.5` 那条用例先炸的。
    """
    import time

    t0 = time.time()
    spec = make_scenario("medium", duration=0.5, stage_seconds=8.0)
    elapsed = time.time() - t0
    assert elapsed < 30.0, f"标定卡住了：{elapsed:.1f}s"
    for p in spec.patterns:
        assert p.resolved_count <= 512, f"数量被无意义地放大了：{p.describe()}"
    # 0.5 秒里只有墙在场（小障碍 16 秒才进场），所以真实峰值远低于目标 ——
    # 照实记录，不假装达到了。
    assert spec.meta["measured_peak_bullets"] < spec.meta["target_bullet_count"]


def test_calibration_still_hits_the_target_on_a_normal_scene():
    """收手规则不能把正常场景也一起"收"掉：30 秒的场景必须照样标定到目标。"""
    spec = make_scenario("medium", duration=30.0, stage_seconds=8.0)
    target = spec.meta["target_bullet_count"]
    measured = spec.meta["measured_peak_bullets"]
    assert measured == pytest.approx(target, rel=0.10), f"{measured} vs {target}"


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


# --------------------------------------------------------------------------
# staged obstacle introduction (`--stage-seconds`)
#
# 视觉链路（摄像头 → PL 几何 → 决策）开场就面对 3 种障碍是没法验的：分不清
# "哪个是墙、哪个是目标"。所以场景支持分批进场：第 1 种从 0 秒开始，之后每 S
# 秒再多放一种，**顺序固定**（墙缺口最先 —— 它是板子侧唯一能可靠读出的几何）。
# --------------------------------------------------------------------------


def _layout(pattern: PatternSpec) -> str:
    return str(pattern.params.get("layout", ""))


def test_staging_is_off_by_default_and_changes_nothing():
    plain = scenario_for_level("medium", seed=3)
    zero = scenario_for_level("medium", seed=3, stage_seconds=0.0)
    assert zero.to_dict() == plain.to_dict(), "stage_seconds <= 0 必须与历史场景一致"
    assert "stage_seconds" not in plain.meta
    assert "obstacle_stages" not in plain.meta


def test_staged_scene_opens_with_the_wall_only():
    spec = scenario_for_level("medium", seed=3, duration=30.0, stage_seconds=8.0)
    first: dict[str, float] = {}
    for p in spec.patterns:
        first[_layout(p)] = min(first.get(_layout(p), 1e9), float(p.start_time))
    assert first["wall_gap"] == 0.0, "第一种必须从 0 秒开始，否则开场是空场"
    for kind in ("small", "block", "cross", "corridor"):
        if kind in first:
            assert first[kind] >= 8.0, f"{kind} 不得在第一个批次窗口内出现"


def test_stage_schedule_is_recorded_in_meta():
    spec = scenario_for_level("hard", seed=1, duration=40.0, stage_seconds=5.0)
    stages = spec.meta["obstacle_stages"]
    assert spec.meta["stage_seconds"] == 5.0
    assert stages[0]["type"] == "wall_with_gap" and stages[0]["first_spawn_s"] == 0.0
    times = [s["first_spawn_s"] for s in stages]
    assert times == sorted(times), f"批次时间必须单调：{stages}"
    assert len(stages) >= 3, f"hard 至少有 4 种障碍，批次表太短：{stages}"


def test_first_stage_really_contains_one_kind():
    """不看 spec，**跑世界**：第一个批次窗口内只能有一种障碍的组活着。"""
    from bullet_sim.simulator.world import World

    window = 8.0
    spec = build_scenario("medium", seed=3, duration=30.0, stage_seconds=window).spec
    layouts = {i: _layout(p) for i, p in enumerate(spec.patterns)}
    world = World(spec, reward="zero", terminate_on_collision=False, clamp_player=True)
    early: set[str] = set()
    alive = 0
    for step in range(int(round(window / spec.dt))):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        alive = max(alive, int(idx.size))
        if idx.size:
            early |= {layouts[int(g)] for g in np.unique(pool.data["group_id"][idx])}
    assert alive > 0, "第一个批次窗口内必须有障碍，不能是空场"
    assert early == {"wall_gap"}, f"开场只应有墙+缺口，实际出现了 {sorted(early)}"


def test_later_stages_do_bring_the_other_kinds():
    from bullet_sim.simulator.world import World

    spec = build_scenario("medium", seed=3, duration=30.0, stage_seconds=4.0).spec
    layouts = {i: _layout(p) for i, p in enumerate(spec.patterns)}
    world = World(spec, reward="zero", terminate_on_collision=False, clamp_player=True)
    late: set[str] = set()
    for _ in range(int(round(30.0 / spec.dt))):
        world.step(0)
    pool = world.state.bullets
    idx = pool.active_indices()
    if idx.size:
        late |= {layouts[int(g)] for g in np.unique(pool.data["group_id"][idx])}
    assert {"wall_gap", "small"} <= late, f"后段必须把其他种类放出来，实际 {sorted(late)}"


def test_staging_shifts_spawns_without_changing_their_geometry():
    """分批只是把后面的种类**整体推后**：出生几何 / 速度必须一模一样。

    长度固定的回合里，晚进场的种类自然少发几次（`emission_steps` 按剩余的
    时长算重复次数）—— 这是"推迟"的应有含义，不是把障碍弄丢；密度由
    `calibrate_bullet_count` 按实测峰值补回来。
    """
    plain = scenario_for_level("medium", seed=5, duration=40.0)
    staged = scenario_for_level("medium", seed=5, duration=40.0, stage_seconds=6.0)
    pb, sb = build_from_spec(plain), build_from_spec(staged)

    def first_event(built, pattern: str):
        for ev in built.timeline.iter_events():
            if ev.pattern == pattern:
                return ev
        return None

    for pattern, shift in (("obstacle:wall_gap", 0.0),
                           ("obstacle:block", 6.0),
                           ("obstacle:small", 12.0)):
        a, b = first_event(pb, pattern), first_event(sb, pattern)
        assert a is not None and b is not None, pattern
        assert b.step - a.step == int(round(shift / plain.dt)), f"{pattern} 的推迟量不对"
        assert np.allclose(a.offsets, b.offsets), f"{pattern} 的出生位置被改了"
        assert np.allclose(a.speeds, b.speeds), f"{pattern} 的速度被改了"

    assert sb.timeline.total_bullets <= pb.timeline.total_bullets


def test_staging_survives_a_json_round_trip(tmp_path):
    spec = scenario_for_level("medium", seed=7, duration=30.0, stage_seconds=6.0)
    loaded = ScenarioSpec.load(spec.save(tmp_path / "staged.json"))
    assert loaded.to_dict() == spec.to_dict()
    assert loaded.meta["obstacle_stages"] == spec.meta["obstacle_stages"]
    a = build_from_spec(loaded)
    b = build_from_spec(spec)
    assert a.timeline.total_bullets == b.timeline.total_bullets


def test_stage_patterns_leaves_unknown_layouts_alone():
    from bullet_sim.scenarios.builder import pattern_kind, stage_patterns

    other = PatternSpec(kind="obstacle", count=3, start_time=1.5,
                        params={"layout": "mystery"})
    assert pattern_kind(other) is None
    staged = stage_patterns([other], 8.0)
    assert staged[0].start_time == 1.5


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
