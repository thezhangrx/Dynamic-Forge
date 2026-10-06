"""LTV override-guard tests - v0.5.2-LTVGuarded.

Covers the nine required behaviours:

1. the v0.4.1 short-term best is kept by default;
2. a small viability difference cannot override;
3. the same risk band *plus* a clear viability gain may override;
4. a different risk band can never be crossed by LTV;
5. a long-term degraded candidate cannot win on a high LTV score alone;
6. EmergencyFallback is not counted as a normal LTV override;
7. shadow mode never changes the action (it equals strict v0.4.1);
8. diagnostics never change the action;
9. BUILD_ID is a real source SHA-256 over the fingerprinted files.
"""

from __future__ import annotations

import pathlib
import re
import tempfile
from types import SimpleNamespace

import numpy as np

from benchmark_decision import _accumulate_ltv, _new_ltv_counts
from cpu.decision import (
    BUILD_ID,
    ENABLE_LTV,
    LTV_SHADOW_ONLY,
    VERSION,
    Action,
    ActionPlanner,
    AgentState,
    Obstacle,
    SceneState,
    ViabilityConfig,
    ViabilityContext,
    ViabilityProfile,
)

CTX = ViabilityContext(short_horizon=0.3, scene_scale=2.4)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=10.0) -> AgentState:
    return AgentState(position=(x, y), velocity=(0.0, 0.0), radius=radius, max_speed=max_speed)


def _scene(obstacles=(), goal=(50.0, 0.0)) -> SceneState:
    return SceneState(
        agent=_agent(),
        obstacles=tuple(obstacles),
        goal=np.asarray(goal, dtype=np.float64),
        dt=0.1,
        horizon=0.3,
    )


def _obstacle(x, y, vx=0.0, vy=0.0, radius=0.8) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


def _threat_scenes():
    return [
        _scene([_obstacle(6.0, 0.0, vx=-8.0)]),
        _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)]),
        _scene([_obstacle(5.0, 1.0), _obstacle(7.0, -1.0)]),
        _scene([_obstacle(4.0, 0.0, vx=-2.0)]),
    ]


def _profile(
    viability,
    viability_cost,
    shortlist,
    corridor_min_clearance=None,
    corridor_width=None,
    corridor_id=None,
) -> ViabilityProfile:
    """Minimal ViabilityProfile for the guard unit checks."""
    b = len(viability)
    clearance = (
        [5.0] * b if corridor_min_clearance is None else list(corridor_min_clearance)
    )
    width = [4] * b if corridor_width is None else list(corridor_width)
    cid = list(range(b)) if corridor_id is None else list(corridor_id)
    return ViabilityProfile(
        viability=np.asarray(viability, dtype=np.float64),
        shortlist=np.asarray(shortlist, dtype=bool),
        viability_cost=np.asarray(viability_cost, dtype=np.float64),
        anchors=np.array([0.3, 0.55, 0.8]),
        per_anchor=(),
        corridor_id=np.asarray(cid, dtype=np.int64),
        corridor_width=np.asarray(width, dtype=np.int64),
        corridor_direction=np.asarray(cid, dtype=np.int64),
        corridor_min_clearance=np.asarray(clearance, dtype=np.float64),
        openness_now=1.0,
        evaluated=int(np.sum(shortlist)),
    )


def _guard_call(planner, profile, base=0, proposed=1, critical=(0, 0), band=(1, 1)):
    return planner._guard_override(
        np.array([0, 1]),
        np.asarray(critical, dtype=np.int64),
        np.asarray(band, dtype=np.int64),
        profile,
        base,
        proposed,
    )


# --------------------------------------------------------------------------
# 1. the v0.4.1 short-term best is kept by default
# --------------------------------------------------------------------------
def test_default_keeps_short_term_best_unless_the_guard_allows():
    planner = ActionPlanner(horizon=0.3)
    for scene in _threat_scenes():
        planner.plan(scene, context=CTX)
        diag = planner.last_diagnostics
        if diag.ltv_override:
            # A surviving override must satisfy every guard condition.
            assert diag.ltv_guard_enabled is True
            assert diag.risk_band_delta == 0
            assert diag.viability_override_viability_delta >= 0.10 - 1e-9
            assert diag.long_term_degraded is False
            assert diag.emergency is False
        else:
            assert diag.final_chosen_index == diag.short_term_best_index
        assert diag.final_chosen_action in planner.candidate_actions(scene)


def test_guard_returns_reference_when_ltv_proposes_the_same_candidate():
    planner = ActionPlanner(horizon=0.3)
    prof = _profile([0.8, 0.2], [0.2, 0.8], [True, True])
    index, blocked, _counts = _guard_call(planner, prof, base=0, proposed=0)
    assert index == 0
    assert blocked == ""


# --------------------------------------------------------------------------
# 2. a small viability difference cannot override
# --------------------------------------------------------------------------
def test_small_viability_difference_cannot_override():
    planner = ActionPlanner(horizon=0.3)
    # +0.05 gain < 0.10 threshold, same band, same corridor clearance
    prof = _profile([0.50, 0.55], [0.50, 0.45], [True, True])
    index, blocked, counts = _guard_call(planner, prof)
    assert index == 0
    assert blocked == "viability_margin"
    assert counts["candidates"] == 1
    assert counts["viability_margin"] == 1


# --------------------------------------------------------------------------
# 3. same risk band + clear viability advantage may override
# --------------------------------------------------------------------------
def test_clear_advantage_in_the_same_risk_band_can_override():
    planner = ActionPlanner(horizon=0.3)
    # +0.35 gain, same band, and a *better* escape corridor (not degraded)
    prof = _profile(
        [0.40, 0.75],
        [0.60, 0.25],
        [True, True],
        corridor_min_clearance=[5.0, 6.0],
        corridor_width=[4, 5],
    )
    index, blocked, counts = _guard_call(planner, prof)
    assert (index, blocked) == (1, "")
    assert counts["candidates"] == 1


# --------------------------------------------------------------------------
# 4. a different risk band can never be crossed by LTV
# --------------------------------------------------------------------------
def test_different_risk_band_can_never_override():
    planner = ActionPlanner(horizon=0.3)
    # arbitrarily large viability gain, but the risk bucket differs
    prof = _profile([0.10, 0.99], [0.90, 0.01], [True, True])
    index, blocked, counts = _guard_call(planner, prof, band=(1, 2))
    assert index == 0
    assert blocked == "risk_band"
    assert counts["risk_band"] == 1


def test_worse_critical_band_can_never_override():
    planner = ActionPlanner(horizon=0.3)
    prof = _profile([0.10, 0.99], [0.90, 0.01], [True, True])
    index, blocked, counts = _guard_call(planner, prof, critical=(0, 1), band=(1, 1))
    assert index == 0
    assert blocked == "critical_band"
    assert counts["critical_band"] == 1


def test_missing_ltv_reference_blocks_every_override():
    planner = ActionPlanner(horizon=0.3)
    # the v0.4.1 choice itself is not on the LTV shortlist -> no comparison
    prof = _profile([0.0, 0.99], [1.0, 0.01], [False, True])
    index, blocked, counts = _guard_call(planner, prof)
    assert index == 0
    assert blocked == "no_ltv_reference"
    assert counts["no_reference"] == 1


# --------------------------------------------------------------------------
# 5. long-term degraded cannot win on a high LTV score alone
# --------------------------------------------------------------------------
def test_long_term_degraded_cannot_override():
    planner = ActionPlanner(horizon=0.3)
    # +0.65 viability gain, but a strictly narrower escape corridor
    prof = _profile(
        [0.30, 0.95],
        [0.70, 0.05],
        [True, True],
        corridor_min_clearance=[6.0, 3.0],
        corridor_width=[5, 2],
    )
    assert planner._long_term_degraded(prof, 1, 0) is True
    index, blocked, counts = _guard_call(planner, prof)
    assert index == 0
    assert blocked == "long_term_degraded"
    assert counts["long_term_degraded"] == 1


def test_long_term_degraded_margin_is_configurable():
    prof = _profile(
        [0.30, 0.95],
        [0.70, 0.05],
        [True, True],
        corridor_min_clearance=[6.0, 5.5],
        corridor_width=[5, 4],
    )
    strict = ActionPlanner(horizon=0.3)  # margin 0.0 -> degraded
    assert strict._long_term_degraded(prof, 1, 0) is True
    tolerant = ActionPlanner(
        horizon=0.3, viability=ViabilityConfig(long_term_degraded_margin=1.0)
    )
    assert tolerant._long_term_degraded(prof, 1, 0) is False


# --------------------------------------------------------------------------
# 6. EmergencyFallback is not counted as a normal LTV override
# --------------------------------------------------------------------------
def test_emergency_is_not_counted_as_a_normal_override():
    # obstacle centred on the agent -> every candidate unsafe -> emergency
    scene = _scene([_obstacle(0.0, 0.0, radius=5.0)])
    planner = ActionPlanner(horizon=0.3)
    planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics
    assert diag.emergency is True
    assert diag.used_emergency is True
    assert diag.ltv_override is False
    assert diag.override_reason == ""
    assert diag.final_chosen_index == diag.short_term_best_index


# --------------------------------------------------------------------------
# 7. shadow mode never changes the action (equals strict v0.4.1)
# --------------------------------------------------------------------------
def test_shadow_mode_never_changes_the_action():
    reference = ActionPlanner(horizon=0.3, enable_ltv=False)
    shadow = ActionPlanner(horizon=0.3, ltv_shadow=True)
    for scene in _threat_scenes():
        a_ref = reference.plan(scene, context=CTX)
        a_shadow = shadow.plan(scene, context=CTX)
        assert a_ref == a_shadow
        diag = shadow.last_diagnostics
        assert diag.ltv_shadow is True
        assert diag.final_chosen_index == diag.short_term_best_index
        assert diag.ltv_override is False
        assert diag.override_guard_reason == "shadow"
        # the LTV pipeline still ran and is replayable
        assert diag.viability_evaluated >= 0
        assert diag.ltv_proposed_index >= 0


def test_shadow_and_enable_ltv_false_agree_on_every_frame():
    strict = ActionPlanner(horizon=0.3, enable_ltv=False)
    shadow = ActionPlanner(horizon=0.3, ltv_shadow=True)
    for scene in _threat_scenes():
        assert strict.plan(scene, context=CTX) == shadow.plan(scene, context=CTX)
        assert (
            strict.last_diagnostics.final_chosen_index
            == shadow.last_diagnostics.final_chosen_index
        )


# --------------------------------------------------------------------------
# 7b. guard disabled -> raw LTV proposal is applied (A/B switch only)
# --------------------------------------------------------------------------
def test_guard_disabled_applies_the_raw_proposal():
    planner = ActionPlanner(
        horizon=0.3, viability=ViabilityConfig(guard_enabled=False)
    )
    for scene in _threat_scenes():
        planner.plan(scene, context=CTX)
        diag = planner.last_diagnostics
        assert diag.ltv_guard_enabled is False
        assert diag.final_chosen_index == diag.ltv_proposed_index


# --------------------------------------------------------------------------
# 8. diagnostics never change the action
# --------------------------------------------------------------------------
def test_diagnostics_do_not_change_the_action_result():
    off = ActionPlanner(horizon=0.3, ltv_debug=False)
    on = ActionPlanner(horizon=0.3, ltv_debug=True)
    for scene in _threat_scenes():
        a_off = off.plan(scene, context=CTX)
        a_on = on.plan(scene, context=CTX)
        assert a_off == a_on
        assert off.last_diagnostics.chosen_index == on.last_diagnostics.chosen_index
        assert off.last_diagnostics.ltv_override == on.last_diagnostics.ltv_override
        assert (
            off.last_diagnostics.ltv_proposed_index
            == on.last_diagnostics.ltv_proposed_index
        )


def test_short_term_best_and_final_chosen_are_recorded():
    planner = ActionPlanner(horizon=0.3)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics

    assert diag.short_term_best_index >= 0
    assert isinstance(diag.short_term_best_action, Action)
    assert diag.final_chosen_index == diag.chosen_index
    assert diag.final_chosen_action == diag.chosen_action
    assert diag.final_chosen_action in planner.candidate_actions(scene)
    assert diag.enable_ltv is True
    assert isinstance(diag.risk_band_delta, int)
    assert isinstance(diag.viability_override_viability_delta, float)
    assert isinstance(diag.long_term_degraded, bool)


# --------------------------------------------------------------------------
# corridor switch kinds + benchmark guard counters
# --------------------------------------------------------------------------
def test_corridor_switch_kinds_and_benchmark_counters():
    planner = ActionPlanner()
    # hold: a different corridor is proposed but refused
    planner._last_corridor_id = 2
    profile = _profile([0.60, 0.56], [0.40, 0.44], [True, True], corridor_id=[5, 2])
    idx, switched, kind = planner._corridor_hysteresis(np.array([0, 1]), 0, profile)
    assert (idx, switched, kind) == (1, False, "hold")

    # hard-unsafe: previous corridor has no feasible candidate
    planner._last_corridor_id = 7
    profile2 = _profile([0.90, 0.20], [0.10, 0.80], [True, True], corridor_id=[5, 2])
    idx2, switched2, kind2 = planner._corridor_hysteresis(np.array([0, 1]), 0, profile2)
    assert (idx2, switched2, kind2) == (0, True, "switch_hard_unsafe")

    # benchmark accumulator maps the diagnostics fields to counters
    counts = _new_ltv_counts()
    _accumulate_ltv(
        counts,
        SimpleNamespace(
            viability_evaluated=4,
            viability_skipped=False,
            ltv_override=True,
            corridor_switched=True,
            corridor_switch_kind="switch_hard_unsafe",
            emergency=False,
            short_term_best_index=0,
            ltv_proposed_index=2,
            override_guard_reason="",
            override_candidate_count=3,
            override_blocked_by_risk_band=1,
            override_blocked_by_viability_margin=1,
            override_blocked_by_long_term_degraded=1,
            override_blocked_by_critical_band=0,
            override_blocked_by_no_reference=0,
        ),
    )
    _accumulate_ltv(
        counts,
        SimpleNamespace(
            viability_evaluated=0,
            viability_skipped=True,
            ltv_override=False,
            corridor_switched=False,
            corridor_switch_kind="hold",
            emergency=True,
            short_term_best_index=1,
            ltv_proposed_index=1,
            override_guard_reason="",
        ),
    )
    assert counts["steps"] == 2
    assert counts["eval"] == 1 and counts["skip"] == 1
    assert counts["override"] == 1
    assert counts["corridor_switch"] == 1
    assert counts["corridor_unsafe_switch"] == 1
    assert counts["corridor_hold"] == 1
    assert counts["emergency"] == 1
    assert counts["proposed_override"] == 1
    assert counts["guard_candidates"] == 3
    assert counts["guard_blocked_risk_band"] == 1
    assert counts["guard_blocked_viability_margin"] == 1
    assert counts["guard_blocked_long_term_degraded"] == 1
    assert counts["guard_veto"] == 0  # no veto recorded in these stubs


# --------------------------------------------------------------------------
# ENABLE_LTV=false runs the strict v0.4.1 path (no viability evaluation)
# --------------------------------------------------------------------------
def test_enable_ltv_false_never_evaluates_viability():
    planner = ActionPlanner(horizon=0.3, enable_ltv=False)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])

    calls = {"n": 0}

    def _must_not_run(*args, **kwargs):  # pragma: no cover - defensive
        calls["n"] += 1
        raise AssertionError("viability.evaluate must not run when ENABLE_LTV=False")

    planner.viability.evaluate = _must_not_run
    planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics

    assert calls["n"] == 0
    assert diag.viability_evaluated == 0
    assert diag.viability_skipped is True
    assert diag.skip_reason == "ltv_disabled"
    assert diag.enable_ltv is False
    assert diag.ltv_override is False


# --------------------------------------------------------------------------
# 9. BUILD_ID is a real source SHA-256
# --------------------------------------------------------------------------
def test_build_id_is_a_real_source_sha256():
    from cpu.decision.build import FINGERPRINT_FILES, fingerprint

    assert re.fullmatch(r"cpudec-[0-9a-f]{12}", BUILD_ID)
    assert "cpu/decision/build.py" in FINGERPRINT_FILES
    assert "cpu/decision/planner.py" in FINGERPRINT_FILES
    # reproducible over the real checkout
    assert fingerprint() == BUILD_ID

    root = pathlib.Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory() as tmp:
        copy = pathlib.Path(tmp)
        for name in FINGERPRINT_FILES:
            dst = copy / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes((root / name).read_bytes())
        # identical bytes -> identical id
        assert fingerprint(copy) == BUILD_ID
        # a single appended byte changes the id
        target = copy / "cpu/decision/planner.py"
        target.write_bytes(target.read_bytes() + b"\n")
        assert fingerprint(copy) != BUILD_ID


def test_version_and_flags_exported():
    assert VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert isinstance(ENABLE_LTV, bool)
    assert isinstance(LTV_SHADOW_ONLY, bool)
    assert BUILD_ID.startswith("cpudec-")


def test_guard_config_is_frozen_and_validated():
    cfg = ViabilityConfig()
    assert cfg.guard_enabled is True
    assert cfg.viability_improvement_threshold == 0.10
    assert cfg.long_term_degraded_margin == 0.0
    with np.testing.assert_raises(ValueError):
        ViabilityConfig(viability_improvement_threshold=-0.1)
    with np.testing.assert_raises(ValueError):
        ViabilityConfig(long_term_degraded_margin=-0.1)
