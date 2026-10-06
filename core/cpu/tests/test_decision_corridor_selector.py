"""Predictive Corridor Selector tests - v0.6.0-PredictiveCorridorSelector.

The thirteen required cases: OFF equivalence, the four preference levels
(reachability / persistent / max_run / clearance), UNKNOWN is not DEAD_END, hard
unsafe can never win, EmergencyFallback is never overridden, STOP handling is
unchanged, action hysteresis still works, determinism, real candidate data, and
no shadow proxy in the final selector.
"""

from __future__ import annotations

import numpy as np
import pytest

from cpu.decision import ActionPlanner, ViabilityContext
from cpu.decision import planner as planner_mod
from cpu.decision.corridor_selector import (
    FSC_SELECTOR_ENABLED,
    LEVEL_CLEARANCE,
    LEVEL_EXISTING_TIERS,
    LEVEL_HARD_SAFETY,
    LEVEL_MAX_RUN,
    LEVEL_NONE,
    LEVEL_PERSISTENT,
    LEVEL_REACHABILITY,
    FutureSpaceTable,
    select_corridor_candidate,
)
from cpu.decision.future_corridor import SafeCorridorShadow
from cpu.decision.models import Action, AgentState, Obstacle, SceneState

CTX = ViabilityContext(short_horizon=0.3, scene_scale=2.4)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=10.0) -> AgentState:
    return AgentState(position=(x, y), velocity=(0.0, 0.0), radius=radius, max_speed=max_speed)


def _scene(obstacles=(), goal=(50.0, 0.0)) -> SceneState:
    return SceneState(
        agent=_agent(),
        obstacles=tuple(obstacles),
        goal=np.asarray(goal, dtype=np.float64),
        dt=1.0 / 120.0,
        horizon=0.3,
    )


def _obstacle(x, y, vx=0.0, vy=0.0, radius=0.8) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


def _table(
    n: int,
    *,
    reachable=(),
    persistent=(),
    max_run=(),
    clearance=(),
    rect_valid=(),
    corridor_type=(),
) -> FutureSpaceTable:
    def arr(values, dtype, fill):
        vals = list(values) if values else [fill] * n
        assert len(vals) == n
        return np.asarray(vals, dtype=dtype)

    reach = arr([bool(v) for v in reachable], bool, False) if reachable else np.zeros(n, bool)
    return FutureSpaceTable(
        reach_12=reach.copy(),
        reach_23=reach.copy(),
        persistent_sectors=arr(persistent, np.int64, 0),
        max_run_t3=arr(max_run, np.int64, 4),
        corridor_clearance_t3=arr(clearance, np.float64, 5.0),
        safe_rect_valid_t3=arr([bool(v) for v in rect_valid], bool, True)
        if rect_valid
        else np.ones(n, bool),
        corridor_type=tuple(corridor_type) if corridor_type else tuple("" for _ in range(n)),
        evaluated=tuple(range(n)),
    )


def _select(n=3, *, feasible=None, base=0, critical=None, **table_kwargs):
    return select_corridor_candidate(
        enabled=True,
        feasible=list(feasible) if feasible is not None else list(range(n)),
        base_index=base,
        critical_band=list(critical) if critical else [0] * n,
        table=_table(n, **table_kwargs),
        fes=[0.0] * n,
        behav=[0.0] * n,
        task=[0.0] * n,
    )


# ==========================================================================
# 1. selector OFF == v0.5.2
# ==========================================================================
def test_selector_off_is_a_total_no_op():
    decision = select_corridor_candidate(
        enabled=False,
        feasible=[0, 1, 2],
        base_index=1,
        critical_band=[0, 0, 0],
        table=_table(3, max_run=[9, 9, 9]),
        fes=[0.0] * 3,
        behav=[0.0] * 3,
        task=[0.0] * 3,
    )
    assert decision.applied is False
    assert decision.index == decision.base_index == 1
    assert decision.level == LEVEL_HARD_SAFETY
    assert decision.reason == "disabled"
    assert FSC_SELECTOR_ENABLED is False
    assert ActionPlanner(horizon=0.3).fsc_selector_enabled is False


def test_selector_off_planner_matches_the_v052_path():
    scenes = [
        _scene([_obstacle(6.0, 0.0, vx=-8.0)]),
        _scene([_obstacle(5.0, 1.0), _obstacle(7.0, -1.0)]),
        _scene([_obstacle(0.0, 0.0, radius=5.0)]),          # emergency
    ]
    off = ActionPlanner(horizon=0.3, fsc_selector=False)
    for _ in range(5):
        for scene in scenes:
            action = off.plan(scene, context=CTX)
            diag = off.last_diagnostics
            assert diag.fsc_selector_enabled is False
            assert diag.fsc_selector_applied is False
            assert diag.fsc_selector_index == diag.fsc_selector_base_index
            assert diag.fsc_table_us == 0.0
            assert diag.chosen_action == action


# ==========================================================================
# 2. reachable candidate beats an unreachable one
# ==========================================================================
def test_reachability_level_prefers_reachable():
    # base 0 unreachable, candidate 1 reachable, everything else identical
    decision = _select(2, base=0, reachable=[False, True], persistent=[9, 1])
    assert decision.applied is True
    assert decision.index == 1
    assert decision.level == LEVEL_REACHABILITY
    assert decision.triggered_reachability is True
    assert decision.base_reachable is False and decision.winner_reachable is True


def test_unreachable_candidate_never_beats_a_reachable_one():
    # candidate 1 has a far wider corridor but is unreachable
    decision = _select(
        2, base=0, reachable=[True, False], persistent=[1, 9], max_run=[2, 8]
    )
    assert decision.index == 0
    assert decision.applied is False


# ==========================================================================
# 3. persistent corridor preference
# ==========================================================================
def test_persistent_level_prefers_more_persistent_sectors():
    decision = _select(
        2, base=0, reachable=[True, True], persistent=[1, 4], max_run=[8, 3]
    )
    assert decision.applied is True
    assert decision.index == 1
    assert decision.level == LEVEL_PERSISTENT
    assert decision.triggered_persistent is True


# ==========================================================================
# 4. max_run preference
# ==========================================================================
def test_max_run_level_prefers_the_wider_corridor():
    decision = _select(
        2, base=0, reachable=[True, True], persistent=[3, 3], max_run=[3, 7]
    )
    assert decision.applied is True
    assert decision.index == 1
    assert decision.level == LEVEL_MAX_RUN
    assert decision.triggered_max_run is True


# ==========================================================================
# 5. clearance tie-break
# ==========================================================================
def test_clearance_level_breaks_the_tie():
    decision = _select(
        2,
        base=0,
        reachable=[True, True],
        persistent=[3, 3],
        max_run=[5, 5],
        clearance=[2.0, 9.0],
    )
    assert decision.applied is True
    assert decision.index == 1
    assert decision.level == LEVEL_CLEARANCE
    assert decision.triggered_clearance is True


def test_existing_tiers_are_the_last_level():
    decision = select_corridor_candidate(
        enabled=True,
        feasible=[0, 1],
        base_index=0,
        critical_band=[0, 0],
        table=_table(
            2, reachable=[True, True], persistent=[3, 3], max_run=[5, 5], clearance=[5.0, 5.0]
        ),
        fes=[1.0, 0.1],           # candidate 1 wins on FES, same risk band
        behav=[0.0, 0.0],
        task=[0.0, 0.0],
        risk_band=[1, 1],
    )
    assert decision.index == 1
    assert decision.level == LEVEL_EXISTING_TIERS
    assert decision.triggered_existing_tiers is True


def test_level_six_alone_may_not_worsen_the_risk_band():
    decision = select_corridor_candidate(
        enabled=True,
        feasible=[0, 1],
        base_index=0,
        critical_band=[0, 0],
        table=_table(
            2, reachable=[True, True], persistent=[3, 3], max_run=[5, 5], clearance=[5.0, 5.0]
        ),
        fes=[1.0, 0.1],
        behav=[0.0, 0.0],
        task=[0.0, 0.0],
        risk_band=[1, 3],         # the FES winner is riskier -> keep v0.5.2
    )
    assert decision.index == 0
    assert decision.applied is False


def test_identical_candidates_keep_the_v052_choice():
    decision = _select(3, base=2, reachable=[True, True, True], persistent=[3, 3, 3],
                       max_run=[5, 5, 5], clearance=[5.0, 5.0, 5.0])
    assert decision.applied is False
    assert decision.index == 2
    assert decision.level == LEVEL_NONE


# ==========================================================================
# 6. UNKNOWN is not DEAD_END
# ==========================================================================
def test_unknown_is_not_an_infinite_penalty():
    # every candidate is UNKNOWN at t3 (rect invalid -> both reach flags false).
    # The selector must still rank them by the raw corridor metrics.
    decision = _select(
        2,
        base=0,
        reachable=[False, False],
        persistent=[0, 3],
        max_run=[2, 7],
        rect_valid=[False, False],
        corridor_type=["UNKNOWN", "UNKNOWN"],
    )
    assert decision.applied is True
    assert decision.index == 1                     # not excluded, not penalised
    assert decision.unknown_candidates == 2
    assert decision.level in (LEVEL_PERSISTENT, LEVEL_MAX_RUN)


def test_corridor_type_is_not_a_selector_input():
    """UNKNOWN vs STABLE labels must not change the winner."""
    left = _select(2, base=0, reachable=[True, True], persistent=[3, 3],
                   max_run=[5, 5], clearance=[5.0, 5.0],
                   corridor_type=["STABLE", "STABLE"])
    right = _select(2, base=0, reachable=[True, True], persistent=[3, 3],
                    max_run=[5, 5], clearance=[5.0, 5.0],
                    corridor_type=["DEAD_END", "UNKNOWN"])
    assert left.index == right.index == 0
    assert left.level == right.level == LEVEL_NONE


def test_mixed_unknown_loses_only_the_reachability_level():
    decision = _select(
        2,
        base=0,
        reachable=[False, True],        # candidate 1 has a proven corridor
        persistent=[3, 1],
        max_run=[8, 2],
        rect_valid=[False, True],
        corridor_type=["UNKNOWN", "STABLE"],
    )
    assert decision.index == 1
    assert decision.level == LEVEL_REACHABILITY


# ==========================================================================
# 7. hard unsafe candidates can never win
# ==========================================================================
def test_hard_unsafe_candidate_can_never_win():
    # index 2 has the widest corridor but is NOT in the feasible set
    decision = _select(
        3,
        feasible=[0, 1],
        base=0,
        reachable=[True, False, True],
        persistent=[1, 0, 9],
        max_run=[2, 0, 8],
    )
    assert decision.index in (0, 1)
    assert decision.index != 2
    assert decision.hard_survivors == 2


def test_critical_candidate_cannot_be_promoted():
    # candidate 1 is far wider but is in the critical tier while base is not
    decision = _select(
        2,
        base=0,
        critical=[0, 1],
        reachable=[True, True],
        persistent=[0, 9],
        max_run=[2, 8],
    )
    assert decision.applied is False
    assert decision.index == 0
    assert decision.eligible == 1


# ==========================================================================
# 8. EmergencyFallback is never overridden
# ==========================================================================
def test_emergency_fallback_is_never_overridden():
    scene = _scene([_obstacle(0.0, 0.0, radius=5.0)])     # nothing is feasible
    planner = ActionPlanner(horizon=0.3, fsc_selector=True)
    action = planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics
    assert diag.emergency is True and diag.used_emergency is True
    assert diag.fsc_selector_applied is False
    assert diag.fsc_selector_index == diag.fsc_selector_base_index
    assert diag.chosen_action == action

    # and the pure selector refuses an empty feasible set
    decision = select_corridor_candidate(
        enabled=True,
        feasible=[],
        base_index=3,
        critical_band=[0] * 5,
        table=_table(5),
        fes=[0.0] * 5,
        behav=[0.0] * 5,
        task=[0.0] * 5,
    )
    assert decision.applied is False and decision.index == 3


# ==========================================================================
# 9. STOP handling is unchanged
# ==========================================================================
def test_stop_gets_no_special_treatment():
    # STOP (index 0) wins on the raw levels -> the choice is kept
    keeps = _select(2, base=0, reachable=[True, False], persistent=[5, 0],
                    max_run=[6, 1])
    assert keeps.index == 0 and keeps.applied is False
    # STOP loses on the raw levels -> normal selector behaviour, no special rule
    loses = _select(2, base=0, reachable=[False, True], persistent=[0, 5],
                    max_run=[1, 6])
    assert loses.index == 1 and loses.level == LEVEL_REACHABILITY


def test_stop_action_is_still_a_candidate_and_emergency_path_is_intact():
    planner = ActionPlanner(horizon=0.3, fsc_selector=True)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    candidates = planner.candidate_actions(scene)
    assert candidates[0].is_stop is True
    assert len(candidates) == 17                     # candidate set unchanged
    planner.plan(scene, context=CTX)
    assert planner.last_diagnostics.used_emergency is False


# ==========================================================================
# 10. action hysteresis still works
# ==========================================================================
def test_action_hysteresis_state_is_maintained():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    for selector in (False, True):
        planner = ActionPlanner(horizon=0.3, fsc_selector=selector)
        for _ in range(8):
            action = planner.plan(scene, context=CTX)
            diag = planner.last_diagnostics
            # the hysteresis state always tracks the emitted action
            assert planner._last_action == action == diag.chosen_action
            assert diag.chosen_index == diag.final_chosen_index


def test_hysteresis_hold_still_occurs_without_the_selector():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    planner = ActionPlanner(horizon=0.3, fsc_selector=False)
    reasons = []
    for _ in range(10):
        planner.plan(scene, context=CTX)
        reasons.append(planner.last_diagnostics.decision_reason)
    assert any("hysteresis" in r for r in reasons)


# ==========================================================================
# 11. determinism
# ==========================================================================
def test_determinism_pure_selector():
    kwargs = dict(
        enabled=True, feasible=[0, 1, 2], base_index=0, critical_band=[0, 0, 0],
        fes=[0.0] * 3, behav=[0.0] * 3, task=[0.0] * 3,
    )
    table = _table(3, reachable=[True, True, False], persistent=[2, 5, 9],
                   max_run=[4, 6, 8])
    first = select_corridor_candidate(table=table, **kwargs)
    second = select_corridor_candidate(table=table, **kwargs)
    assert first == second
    assert first.to_dict() == second.to_dict()


def test_determinism_across_planner_instances():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    a = ActionPlanner(horizon=0.3, fsc_selector=True)
    b = ActionPlanner(horizon=0.3, fsc_selector=True)
    for _ in range(10):
        assert a.plan(scene, context=CTX) == b.plan(scene, context=CTX)
        da, db = a.last_diagnostics, b.last_diagnostics
        assert da.fsc_selector_index == db.fsc_selector_index
        assert da.fsc_selector_level == db.fsc_selector_level
        assert da.fsc_hard_survivors == db.fsc_hard_survivors


# ==========================================================================
# 12. the selector consumes real planner candidate data
# ==========================================================================
def test_selector_uses_the_real_feasible_set(monkeypatch):
    captured: dict = {}
    real = planner_mod.build_future_space_table

    def spy(shadow, scene, actions, indices, context, **kwargs):
        captured["indices"] = list(indices)
        captured["n_actions"] = len(actions)
        table = real(shadow, scene, actions, indices, context, **kwargs)
        captured["table"] = table
        return table

    monkeypatch.setattr(planner_mod, "build_future_space_table", spy)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    planner = ActionPlanner(horizon=0.3, fsc_selector=True)
    planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics

    assert captured, "the selector must build a corridor table when enabled"
    assert captured["indices"] == list(np.flatnonzero(~diag.infeasible))
    assert captured["n_actions"] >= len(planner.candidate_actions(scene))
    assert diag.fsc_hard_survivors == diag.n_feasible
    # every table entry the selector read belongs to a real planner candidate
    assert set(captured["table"].evaluated) <= set(range(captured["n_actions"]))


def test_selector_table_is_only_built_when_enabled(monkeypatch):
    calls = {"n": 0}
    real = planner_mod.build_future_space_table

    def spy(*args, **kwargs):
        calls["n"] += 1
        return real(*args, **kwargs)

    monkeypatch.setattr(planner_mod, "build_future_space_table", spy)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    ActionPlanner(horizon=0.3, fsc_selector=False).plan(scene, context=CTX)
    assert calls["n"] == 0                     # OFF costs nothing at all
    ActionPlanner(horizon=0.3, fsc_selector=True).plan(scene, context=CTX)
    assert calls["n"] == 1


# ==========================================================================
# 13. no shadow proxy in the final selector
# ==========================================================================
def test_no_shadow_proxy_in_the_final_selector(monkeypatch):
    def _forbidden(*args, **kwargs):  # pragma: no cover - must never run
        raise AssertionError("the selector must not use the shadow Top-K proxy")

    monkeypatch.setattr(SafeCorridorShadow, "select_shadow_candidates", _forbidden)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    planner = ActionPlanner(horizon=0.3, fsc_selector=True)
    action = planner.plan(scene, context=CTX)
    assert action == planner.last_diagnostics.chosen_action
