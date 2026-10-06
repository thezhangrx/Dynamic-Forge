"""Per-episode logging + paired analysis tests - v0.5.2-LTVGuarded diagnostics.

Covers the episode-record schema, the JSONL writer, the seed join and the
override-timing analysis.  Nothing here changes control: the tests assert that
logging is pure and that the emitted action sequence is untouched.
"""

from __future__ import annotations

import json
import pathlib
import tempfile

import pytest

import benchmark_decision as bench
from tools import ltv_regression_diagnostics as tool
from tools.ltv_regression_diagnostics import (
    FAILURE_TYPES,
    TEMPORAL_BINS,
    _episode_paths,
    _mode_label,
    load_episodes,
    paired_episode_analysis,
    temporal_bin,
)

#: A stop-action controller collides deterministically for this seed.
COLLISION_SEED = 42


# --------------------------------------------------------------------------
# stubs: report diagnostics without influencing the emitted action
# --------------------------------------------------------------------------
class _StubDiagnostics:
    def __init__(self, override: bool, guard_reason: str = "") -> None:
        self.ltv_override = override
        self.viability_evaluated = 0
        self.viability_skipped = True
        self.corridor_switched = False
        self.corridor_switch_kind = ""
        self.emergency = False
        self.short_term_best_index = 0
        self.ltv_proposed_index = 0
        self.override_guard_reason = guard_reason
        self.override_candidate_count = 0
        self.override_blocked_by_risk_band = 0
        self.override_blocked_by_viability_margin = 0
        self.override_blocked_by_long_term_degraded = 0
        self.override_blocked_by_critical_band = 0
        self.override_blocked_by_no_reference = 0


class _StubPlanner:
    def __init__(self) -> None:
        self.last_diagnostics = None


class _StubLayer:
    def __init__(self) -> None:
        self.planner = _StubPlanner()


class _OverrideController:
    """Stops in place; reports an *applied* override only on chosen steps.

    Diagnostics never affect the action, so the trajectory (and the collision
    step) is identical to a plain stop controller.
    """

    def __init__(self, override_steps=(), veto_steps=()) -> None:
        self.override_steps = set(override_steps)
        self.veto_steps = set(veto_steps)
        self.layer = _StubLayer()
        self.i = 0

    def reset(self) -> None:
        self.i = 0
        self.layer.planner.last_diagnostics = None

    def act(self, obs, info):
        is_veto = self.i in self.veto_steps
        diag = _StubDiagnostics(self.i in self.override_steps, "risk_band" if is_veto else "")
        self.layer.planner.last_diagnostics = diag
        self.i += 1
        return 0


_COLLISION_STEP: int | None = None


def _collision_step() -> int:
    """Collision step of the deterministic stop-action episode (cached)."""
    global _COLLISION_STEP
    if _COLLISION_STEP is None:
        row = bench.run_episode(COLLISION_SEED, _OverrideController())
        assert row["collision"] is True
        _COLLISION_STEP = int(row["collision_step"])
    return _COLLISION_STEP


# --------------------------------------------------------------------------
# 1-3. seed / collision_step
# --------------------------------------------------------------------------
def test_run_episode_returns_the_seed():
    row = bench.run_episode(COLLISION_SEED, _OverrideController(), max_steps=3)
    assert row["seed"] == COLLISION_SEED
    assert row["seed"] == 42


def test_collision_step_matches_the_fatal_step():
    row = bench.run_episode(COLLISION_SEED, _OverrideController())
    assert row["collision"] is True
    assert isinstance(row["collision_step"], int)
    assert row["collision_step"] == row["survived"]
    assert row["collision_step"] == _collision_step()


def test_collision_step_is_null_without_collision():
    row = bench.run_episode(COLLISION_SEED, _OverrideController(), max_steps=3)
    assert row["collision"] is False
    assert row["collision_step"] is None
    assert row["failure"] is None
    assert row["override_steps"] == []


# --------------------------------------------------------------------------
# 4-5. override_steps / last_override_step
# --------------------------------------------------------------------------
def test_override_steps_records_only_applied_overrides():
    row = bench.run_episode(
        COLLISION_SEED, _OverrideController(override_steps=(0, 1, 2))
    )
    assert row["override_steps"] == [0, 1, 2]
    assert row["override_seen"] is True
    assert row["override_steps"] == sorted(row["override_steps"])


def test_guard_veto_and_proposal_are_not_recorded_as_overrides():
    # steps 0/1 are applied overrides; steps 2/3 are guard vetoes (not applied)
    row = bench.run_episode(
        COLLISION_SEED,
        _OverrideController(override_steps=(0, 1), veto_steps=(2, 3)),
    )
    assert row["override_steps"] == [0, 1]
    assert row["ltv"]["override"] == 2
    assert row["ltv"]["guard_veto"] == 2


def test_last_override_step_is_the_final_applied_override():
    row = bench.run_episode(
        COLLISION_SEED, _OverrideController(override_steps=(0, 5, 17))
    )
    assert row["last_override_step"] == 17
    assert row["last_override_step"] == row["override_steps"][-1]

    empty = bench.run_episode(COLLISION_SEED, _OverrideController())
    assert empty["last_override_step"] is None
    assert empty["override_seen"] is False


# --------------------------------------------------------------------------
# 6-8. override timing relative to the fatal collision
# --------------------------------------------------------------------------
def test_last_override_before_collision_uses_strictly_earlier_steps():
    row = bench.run_episode(
        COLLISION_SEED, _OverrideController(override_steps=(0, 1, 2))
    )
    assert row["last_override_step"] == 2
    assert row["last_override_step_before_collision"] == 2


def test_no_prior_override_gives_null_timing_fields():
    collision_step = _collision_step()
    # an override exactly AT the fatal step is applied but is not "before" it
    at_step = bench.run_episode(
        COLLISION_SEED, _OverrideController(override_steps=(collision_step,))
    )
    assert at_step["override_steps"] == [collision_step]
    assert at_step["last_override_step"] == collision_step
    assert at_step["last_override_step_before_collision"] is None
    assert at_step["steps_since_last_override_before_collision"] is None
    assert at_step["seconds_since_last_override_before_collision"] is None

    none_at_all = bench.run_episode(COLLISION_SEED, _OverrideController())
    assert none_at_all["last_override_step_before_collision"] is None
    assert none_at_all["steps_since_last_override_before_collision"] is None
    assert none_at_all["seconds_since_last_override_before_collision"] is None


def test_temporal_delta_is_steps_over_120hz():
    collision_step = _collision_step()
    row = bench.run_episode(COLLISION_SEED, _OverrideController(override_steps=(0, 7)))
    steps = row["steps_since_last_override_before_collision"]
    assert steps == collision_step - 7
    assert bench.CONTROL_HZ == 120.0
    assert row["seconds_since_last_override_before_collision"] == pytest.approx(
        steps / 120.0
    )


# --------------------------------------------------------------------------
# 9. JSONL is one valid JSON object per line
# --------------------------------------------------------------------------
def test_write_episode_jsonl_is_one_json_object_per_line(tmp_path):
    rows = [
        bench.run_episode(seed, _OverrideController(), max_steps=3)
        for seed in (42, 43, 44)
    ]
    path = bench.write_episode_jsonl(tmp_path / "run.episodes.jsonl", rows)

    lines = path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == len(rows)
    seeds = []
    for line, row in zip(lines, rows):
        obj = json.loads(line)                      # valid JSON per line
        assert isinstance(obj, dict)
        assert obj["seed"] == row["seed"]
        seeds.append(obj["seed"])
        for field in bench.EPISODE_JSONL_FIELDS:
            assert field in obj
        assert "decision_times" not in obj          # no per-frame trajectories
    assert seeds == [42, 43, 44]                    # stable order
    assert load_episodes(path) == [json.loads(line) for line in lines]


def test_episode_record_is_pure_and_json_safe():
    row = bench.run_episode(42, _OverrideController(override_steps=(0,)), max_steps=3)
    snapshot = dict(row)
    record = bench.episode_record(row)
    assert row == snapshot                          # no mutation
    json.dumps(record)                              # serializable
    assert record["override_count"] == 1
    assert record["decisions"] == 3


def test_off_and_guarded_logs_never_share_a_file():
    off, guarded = _episode_paths("/tmp/run7")
    assert off.name == "run7.off.episodes.jsonl"
    assert guarded.name == "run7.guarded.episodes.jsonl"
    assert off != guarded


# --------------------------------------------------------------------------
# 10-14. paired analysis on synthetic logs
# --------------------------------------------------------------------------
def _ep(seed, collision, failure=None, override_steps=(), seconds=None, decisions=10):
    return {
        "seed": seed,
        "collision": bool(collision),
        "collision_step": 100 if collision else None,
        "failure": failure,
        "override_seen": bool(override_steps),
        "override_steps": list(override_steps),
        "last_override_step": (override_steps[-1] if override_steps else None),
        "last_override_step_before_collision": None,
        "steps_since_last_override_before_collision": None,
        "seconds_since_last_override_before_collision": seconds,
        "survived": 0,
        "changes": 0,
        "override_count": len(override_steps),
        "decisions": decisions,
    }


def _write(path, records):
    path.write_text(
        "".join(json.dumps(r) + "\n" for r in records), encoding="utf-8"
    )
    return path


def _paired_fixture(tmp_path):
    """6 episodes with known 4-cell / failure / temporal values."""
    off = [
        _ep(1, True, "side_hit"),                     # -> guarded survive
        _ep(2, False),                                # -> guarded front_hit
        _ep(3, True, "side_hit"),                     # -> guarded corner_trap
        _ep(4, False),
        _ep(5, False),
        _ep(6, True, "corner_trap"),                  # -> guarded survive
    ]
    guarded = [
        _ep(1, False, override_steps=(1,), seconds=0.09),   # <0.10, prior override
        _ep(2, True, "front_hit", seconds=0.30),            # 0.25-0.50, no override
        _ep(3, True, "corner_trap", override_steps=(2, 4), seconds=None),  # none
        _ep(4, False, override_steps=(3,)),                 # override, survived
        _ep(5, False),
        _ep(6, False),
    ]
    return _write(tmp_path / "off.jsonl", off), _write(tmp_path / "g.jsonl", guarded)


def test_paired_analysis_four_cell_table_and_identities(tmp_path):
    off, guarded = _paired_fixture(tmp_path)
    a = paired_episode_analysis(off, guarded)

    assert a["episodes"] == 6
    assert a["both_collision"] == 1
    assert a["both_survive"] == 2
    assert a["off_survive_on_collision"] == 1
    assert a["off_collision_on_survive"] == 2
    total = (
        a["both_collision"]
        + a["both_survive"]
        + a["off_survive_on_collision"]
        + a["off_collision_on_survive"]
    )
    assert total == a["episodes"]

    assert a["on_gain"] == a["off_collision_on_survive"] == 2
    assert a["on_loss"] == a["off_survive_on_collision"] == 1
    assert a["net_episode_delta"] == 1
    assert a["net_episode_delta"] == (
        a["off_collision_total"] - a["guarded_collision_total"]
    )
    assert (a["off_collision_total"], a["guarded_collision_total"]) == (3, 2)


def test_failure_type_paired_analysis_uses_final_failure(tmp_path):
    off, guarded = _paired_fixture(tmp_path)
    a = paired_episode_analysis(off, guarded)
    pairs = a["failure_pairs"]

    assert pairs["side_hit"]["off_failure_on_survive"] == 1     # seed 1
    assert pairs["side_hit"]["off_survive_on_failure"] == 0
    assert pairs["corner_trap"]["off_failure_on_survive"] == 1  # seed 6
    assert pairs["front_hit"]["off_survive_on_failure"] == 1    # seed 2
    assert pairs["front_hit"]["off_failure_on_survive"] == 0
    assert pairs["other"]["off_failure_on_survive"] == 0
    for name in FAILURE_TYPES:
        assert name in pairs


def test_override_outcome_split_and_rate(tmp_path):
    off, guarded = _paired_fixture(tmp_path)
    a = paired_episode_analysis(off, guarded)
    assert a["override_episodes"] == 3                       # seeds 1, 3, 4
    assert a["override_episode_collided"] == 1               # seed 3 only
    assert a["override_episode_survived"] == 2               # seeds 1, 4
    assert (
        a["override_episode_survived"] + a["override_episode_collided"]
        == a["override_episodes"]
    )
    assert a["overrides_total"] == 4                         # 1 + 2 + 1
    assert a["total_decisions"] == 60
    assert a["override_rate_per_decision"] == pytest.approx(4 / 60)


def test_paired_analysis_rejects_count_mismatch(tmp_path):
    off = _write(tmp_path / "off.jsonl", [_ep(1, True)])
    guarded = _write(tmp_path / "g.jsonl", [_ep(1, True), _ep(2, False)])
    with pytest.raises(ValueError, match="count mismatch"):
        paired_episode_analysis(off, guarded)


def test_paired_analysis_rejects_unmatched_seed(tmp_path):
    off = _write(tmp_path / "off.jsonl", [_ep(1, True), _ep(2, False)])
    guarded = _write(tmp_path / "g.jsonl", [_ep(1, True), _ep(3, False)])
    with pytest.raises(ValueError, match="unmatched seeds"):
        paired_episode_analysis(off, guarded)


def test_paired_analysis_rejects_duplicate_seed(tmp_path):
    off = _write(tmp_path / "off.jsonl", [_ep(1, True), _ep(1, False)])
    guarded = _write(tmp_path / "g.jsonl", [_ep(1, True)])
    with pytest.raises(ValueError, match="duplicate seed"):
        paired_episode_analysis(off, guarded)


def test_paired_analysis_rejects_invalid_jsonl(tmp_path):
    bad = tmp_path / "bad.jsonl"
    bad.write_text('{"seed": 1}\nnot json\n', encoding="utf-8")
    good = _write(tmp_path / "g.jsonl", [_ep(1, True)])
    with pytest.raises(ValueError, match="invalid JSON line"):
        paired_episode_analysis(bad, good)


# --------------------------------------------------------------------------
# 15. temporal bins
# --------------------------------------------------------------------------
@pytest.mark.parametrize(
    "value,expected",
    [
        (None, "none"),
        (0.0, "<0.10"),
        (0.0999, "<0.10"),
        (0.10, "0.10-0.25"),
        (0.2499, "0.10-0.25"),
        (0.25, "0.25-0.50"),
        (0.4999, "0.25-0.50"),
        (0.50, "0.50-1.00"),
        (0.9999, "0.50-1.00"),
        (1.00, "1.00-2.00"),
        (2.00, "1.00-2.00"),
        (2.0001, ">2.00"),
        (10.0, ">2.00"),
    ],
)
def test_temporal_bin_boundaries(value, expected):
    assert temporal_bin(value) == expected


def test_temporal_analysis_counts_and_bins(tmp_path):
    off, guarded = _paired_fixture(tmp_path)
    a = paired_episode_analysis(off, guarded)

    # guarded collision episodes are seeds 2 and 3
    assert a["guarded_collision_episodes_with_prior_override"] == 1   # seed 2 @0.30
    assert a["guarded_collision_episodes_without_prior_override"] == 1  # seed 3 None
    bins = a["temporal_bins"]
    assert bins["0.25-0.50"] == 1
    assert bins["none"] == 1
    assert sum(bins.values()) == 2
    assert set(bins) == set(TEMPORAL_BINS)

    per_failure = a["temporal_by_failure"]
    assert per_failure["front_hit"]["collision_episodes"] == 1
    assert per_failure["front_hit"]["prior_override_count"] == 1
    assert per_failure["front_hit"]["no_prior_override_count"] == 0
    assert per_failure["corner_trap"]["collision_episodes"] == 1
    assert per_failure["corner_trap"]["prior_override_count"] == 0
    assert per_failure["corner_trap"]["no_prior_override_count"] == 1
    assert per_failure["corner_trap"]["bins"]["none"] == 1
    for name in FAILURE_TYPES:
        assert name in per_failure
        assert set(per_failure[name]["bins"]) == set(TEMPORAL_BINS)


# --------------------------------------------------------------------------
# 16. legacy summary compatibility + new outcome split
# --------------------------------------------------------------------------
def test_summary_keeps_legacy_keys_and_adds_outcome_split():
    rows = [
        bench.run_episode(seed, _OverrideController(override_steps=(0,)), max_steps=3)
        for seed in (42, 43)
    ]
    summary = bench.summarize("t", rows)

    for key in (
        "name",
        "survival_mean",
        "survival_std",
        "collision_rate",
        "collisions",
        "episodes",
        "decision_us_mean",
        "changes_total",
        "total_decisions",
        "failures",
        "ltv",
    ):
        assert key in summary
    ltv = summary["ltv"]
    for key in (
        "steps",
        "eval",
        "skip",
        "override",
        "override_rate",
        "proposed_override",
        "guard_veto",
        "guard_blocked_risk_band",
        "corridor_switch",
        "emergency",
        "override_episodes",
        "override_then_collision",
        "override_then",
    ):
        assert key in ltv
    assert ltv["override_episode_survived"] + ltv["override_episode_collided"] == (
        ltv["override_episodes"]
    )
    assert ltv["override_episodes"] == 2           # both episodes had an override
    assert ltv["override_episode_survived"] == 2   # neither collided
    assert ltv["override_then"] == {
        "front_hit": 0,
        "side_hit": 0,
        "corner_trap": 0,
        "other": 0,
    }


# --------------------------------------------------------------------------
# 17. stale label is gone; labels come from the runtime version
# --------------------------------------------------------------------------
def test_mode_label_is_runtime_derived():
    assert _mode_label("v9.9.9-Test", ltv_on=True) == "LTV ON  (v9.9.9-Test)"
    assert "v9.9.9-Test" in _mode_label("v9.9.9-Test", ltv_on=False)
    source = pathlib.Path(tool.__file__).read_text(encoding="utf-8")
    assert "v0.5.1" not in source            # no stale hard-coded version
    assert "LTV ON  ({stack_version})" in source


def test_tool_imports_the_real_benchmark_versions():
    assert bench.STACK_VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert bench.PREDICTIVE_VERSION == "v0.6.0-PredictiveCorridorSelector"


# --------------------------------------------------------------------------
# 18. episode logging must not change control
# --------------------------------------------------------------------------
def test_episode_logging_does_not_change_control():
    first = bench.run_episode(42, _OverrideController(override_steps=(0, 1)))
    with tempfile.TemporaryDirectory() as tmp:
        bench.write_episode_jsonl(pathlib.Path(tmp) / "x.jsonl", [first])
    second = bench.run_episode(42, _OverrideController(override_steps=(0, 1)))
    for key in ("survived", "collision", "collision_step", "changes", "override_steps"):
        assert first[key] == second[key]

    # and with the real planner: serializing a run does not perturb the next one
    real_a = bench.run_episode(42, bench.PredictiveController(), max_steps=150)
    with tempfile.TemporaryDirectory() as tmp:
        bench.write_episode_jsonl(pathlib.Path(tmp) / "y.jsonl", [real_a])
    real_b = bench.run_episode(42, bench.PredictiveController(), max_steps=150)
    assert real_a["survived"] == real_b["survived"]
    assert real_a["override_steps"] == real_b["override_steps"]
    assert real_a["collision"] == real_b["collision"]


def test_episode_stub_does_not_touch_the_planner():
    """Sanity: the stub only feeds diagnostics, exactly like the real harness."""
    controller = _OverrideController(override_steps=(0,))
    row = bench.run_episode(42, controller, max_steps=4)
    assert row["changes"] == 0                       # constant stop action
    assert row["ltv"]["steps"] == 4
