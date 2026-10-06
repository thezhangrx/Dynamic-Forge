"""Long-Term Viability mechanism tests (A-N) - v0.5.1 LTV under v0.5.2-LTVGuarded."""

from __future__ import annotations

import math

import numpy as np
import pytest

from cpu.decision import (
    BUILD_ID,
    VERSION,
    Action,
    ActionPlanner,
    AgentState,
    LongTermViabilityEvaluator,
    Obstacle,
    SafetyEnvelope,
    SceneState,
    ViabilityConfig,
    ViabilityContext,
    ViabilityProfile,
)
from cpu.decision.viability import (
    _mobility_from_summary,
    _sector_summary_batch,
)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=10.0) -> AgentState:
    return AgentState(position=(x, y), velocity=(0.0, 0.0), radius=radius, max_speed=max_speed)


def _obstacle(x, y, vx=0.0, vy=0.0, radius=0.8) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


def _scene(obstacles=(), goal=(50.0, 0.0), horizon=0.3) -> SceneState:
    g = None if goal is None else np.asarray(goal, dtype=np.float64)
    return SceneState(
        agent=_agent(), obstacles=tuple(obstacles), goal=g, dt=0.1, horizon=horizon
    )


def _ctx(scene_scale=2.4, short=0.3) -> ViabilityContext:
    return ViabilityContext(short_horizon=short, scene_scale=scene_scale)


# --------------------------------------------------------------------------
# A. A/B long-term decision
# --------------------------------------------------------------------------
def test_A_evaluator_prefers_open_future_over_enclosed_future():
    # A (heading 0) runs into a pinched box around its t2/t3 anchors;
    # B (heading 45) keeps open space.
    walls = [
        (5.5, 1.2), (5.5, -1.2), (6.5, 1.2), (6.5, -1.2), (8.0, 1.2), (8.0, -1.2)
    ]
    scene = _scene([_obstacle(x, y) for x, y in walls])
    evaluator = LongTermViabilityEvaluator(
        ViabilityConfig(top_k=2, leaders=2, diversity=0)
    )
    actions = (Action(10.0, 0.0), Action(10.0, math.radians(45.0)))
    profile = evaluator.evaluate(
        scene,
        actions,
        np.array([0.0, 0.02]),
        np.array([0, 0]),
        np.array([True, True]),
        context=_ctx(),
    )
    assert bool(profile.shortlist[0]) and bool(profile.shortlist[1])
    assert profile.viability[0] < profile.viability[1]
    assert profile.viability[0] == pytest.approx(
        min(float(fm.mobility[0]) for fm in profile.per_anchor)
    )


def test_A_arbitration_allows_open_candidate_to_beat_slightly_safer_dead_end():
    # Both are hard-safe and land in the SAME risk band (0): A has slightly
    # lower short-term risk but worse viability, B is slightly worse short-term
    # but clearly better long-term.  The bucketed rule must allow B to win.
    planner = ActionPlanner()
    feasible = np.array([0, 1])
    critical_band = np.array([0, 0])
    risk_band = np.array([0, 0])          # same bucket -> viability decides
    viability_cost = np.array([0.8, 0.2])  # A worse long-term
    zeros = np.zeros(2)
    chosen = planner._bucketed_select(
        feasible, critical_band, risk_band, viability_cost, zeros, zeros, zeros
    )
    assert chosen == 1


def test_A_cross_band_safety_still_wins_over_viability():
    planner = ActionPlanner()
    chosen = planner._bucketed_select(
        np.array([0, 1]),
        np.array([0, 0]),
        np.array([0, 1]),            # A clearly safer (lower risk band)
        np.array([0.9, 0.0]),        # B much better long-term
        np.zeros(2),
        np.zeros(2),
        np.zeros(2),
    )
    assert chosen == 0               # obvious short-term danger cannot be bought back


# --------------------------------------------------------------------------
# B. hard-unsafe long-term candidate
# --------------------------------------------------------------------------
def test_B_hard_unsafe_candidate_is_eliminated_even_if_viable():
    # obstacle dead ahead: the "straight" candidate is hard unsafe; the open
    # backward direction is viable.  Safety must win.
    scene = _scene([_obstacle(3.0, 0.0, radius=1.0)])
    planner = ActionPlanner(horizon=0.3)
    planner.plan(scene, context=_ctx())
    diag = planner.last_diagnostics
    straight = planner._find_action(planner.candidate_actions(scene), Action(10.0, 0.0))
    assert straight is not None
    assert bool(diag.infeasible[straight]) is True
    assert diag.chosen_index != straight


# --------------------------------------------------------------------------
# C. midway dead-end (must not look only at the last anchor)
# --------------------------------------------------------------------------
def test_C_midway_dead_end_lowers_viability():
    # only the middle anchor is pinched: 6 obstacles ring A's t2 position (leaving
    # a narrow forward gap) plus one further ahead to shrink that gap's clearance.
    angles = [a for a in range(45, 360, 45) if a != 180]
    obstacles = [
        _obstacle(5.5 + 1.25 * math.cos(math.radians(a)), 1.25 * math.sin(math.radians(a)))
        for a in angles
    ]
    obstacles.append(_obstacle(7.5, 0.0))
    scene = _scene(obstacles)
    evaluator = LongTermViabilityEvaluator(ViabilityConfig(top_k=1, leaders=1, diversity=0))
    profile = evaluator.evaluate(
        scene,
        (Action(10.0, 0.0),),
        np.array([0.0]),
        np.array([0]),
        np.array([True]),
        context=_ctx(),
    )
    per_anchor = [float(fm.mobility[0]) for fm in profile.per_anchor]
    assert profile.viability[0] == pytest.approx(min(per_anchor))
    assert per_anchor[1] < per_anchor[0]
    assert per_anchor[1] < per_anchor[2]


# --------------------------------------------------------------------------
# D/E/F/K. mobility building blocks
# --------------------------------------------------------------------------
def _free_from_mask(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    nearest = np.where(mask, 10.0, -10.0)[None, :].astype(np.float64)
    closing = np.zeros_like(nearest)
    return nearest, closing


def test_D_complete_dead_end_gives_zero_mobility():
    nearest, closing = _free_from_mask(np.zeros(8, dtype=bool))
    mobility, safe, run, cmin, _dir = _mobility_from_summary(
        nearest, closing, 0.0, 0.04, ViabilityConfig()
    )
    assert float(mobility[0]) == 0.0
    assert int(safe[0]) == 0 and int(run[0]) == 0 and float(cmin[0]) == 0.0


def test_E_multiple_corridors_detected():
    mask = np.array([1, 1, 0, 0, 1, 1, 0, 0], dtype=bool)   # two separate runs
    nearest, closing = _free_from_mask(mask)
    mobility, safe, run, _cmin, _dir = _mobility_from_summary(
        nearest, closing, 0.0, 0.04, ViabilityConfig()
    )
    assert int(safe[0]) == 4
    assert int(run[0]) == 2        # longest continuous corridor
    assert 0.0 <= float(mobility[0]) <= 1.0


def test_E_circular_run_wraps_dir7_to_dir0():
    mask = np.array([1, 1, 0, 0, 0, 0, 0, 1], dtype=bool)   # 7,0,1 is one circular run
    nearest, closing = _free_from_mask(mask)
    _mob, safe, run, _cmin, direction = _mobility_from_summary(
        nearest, closing, 0.0, 0.04, ViabilityConfig()
    )
    assert int(safe[0]) == 3
    assert int(run[0]) == 3
    assert int(direction[0]) == 0


def test_F_same_corridor_candidates_share_class():
    scene = _scene([])  # fully open -> canonical full corridor
    evaluator = LongTermViabilityEvaluator(ViabilityConfig(top_k=2, leaders=2, diversity=0))
    profile = evaluator.evaluate(
        scene,
        (Action(10.0, 0.0), Action(10.0, math.radians(45.0))),
        np.array([0.0, 0.0]),
        np.array([0, 0]),
        np.array([True, True]),
        context=_ctx(),
    )
    assert int(profile.corridor_id[0]) == int(profile.corridor_id[1])


def test_K_mobility_is_bounded_in_unit_interval():
    rng = np.random.default_rng(0)
    for _ in range(50):
        nearest = rng.uniform(-5.0, 30.0, size=(3, 8))
        closing = rng.uniform(0.0, 40.0, size=(3, 8))
        mobility, safe, run, cmin, _dir = _mobility_from_summary(
            nearest, closing, 0.0, 0.04, ViabilityConfig()
        )
        assert np.all(mobility >= 0.0) and np.all(mobility <= 1.0)
        assert np.all(safe >= 0) and np.all(safe <= 8)
        assert np.all(run >= 0) and np.all(run <= 8)
        assert np.all(cmin >= 0.0)


# --------------------------------------------------------------------------
# G. dynamic obstacles use future positions
# --------------------------------------------------------------------------
def test_G_dynamic_obstacle_future_position_changes_mobility():
    evaluator = LongTermViabilityEvaluator(ViabilityConfig(top_k=1, leaders=1, diversity=0))
    actions = (Action(10.0, 0.0),)
    args = (actions, np.array([0.0]), np.array([0]), np.array([True]))
    # An obstacle sitting at x=12 far away, versus one moving left so that it
    # reaches the candidate's t2 future position: viability must drop.
    static = evaluator.evaluate(
        _scene([_obstacle(12.0, 0.0, radius=1.0)]), *args, context=_ctx()
    )
    moving = evaluator.evaluate(
        _scene([_obstacle(12.0, 0.0, vx=-10.0, radius=1.0)]), *args, context=_ctx()
    )
    assert float(moving.viability[0]) < float(static.viability[0])


# --------------------------------------------------------------------------
# H. Top-K diversity
# --------------------------------------------------------------------------
def test_H_top_k_diversity_admits_long_term_best_outside_short_term_top3():
    evaluator = LongTermViabilityEvaluator()   # K=4, leaders=3, diversity=1
    actions = (
        Action(10.0, 0.0),                 # class 0
        Action(10.0, math.radians(45.0)),  # class 1
        Action(10.0, math.radians(90.0)),  # class 2
        Action(10.0, math.radians(135.0)), # class 3 -> diversity
    )
    # the 4th candidate is ranked last by short-term risk (still same band)
    short_term = np.array([0.0, 0.0, 0.0, 0.01])
    mask = evaluator.shortlist(actions, short_term, np.array([0, 0, 0, 0]), np.ones(4, bool))

    assert bool(mask[3]) is True          # enters LTV despite not being top-3
    assert int(mask.sum()) <= 4           # base action space is never expanded


def test_H_diversity_does_not_peek_at_viability():
    # direction_class only depends on the action; it cannot read mobility etc.
    from cpu.decision import direction_class

    assert direction_class(Action(10.0, 0.0)) == 0
    assert direction_class(Action(10.0, math.radians(45.0))) == 1
    assert direction_class(Action.stop()) == -1


# --------------------------------------------------------------------------
# I. corridor hysteresis
# --------------------------------------------------------------------------
def _profile(corridor_id, viability, viability_cost, shortlist) -> ViabilityProfile:
    b = len(corridor_id)
    return ViabilityProfile(
        viability=np.asarray(viability, dtype=np.float64),
        shortlist=np.asarray(shortlist, dtype=bool),
        viability_cost=np.asarray(viability_cost, dtype=np.float64),
        anchors=np.array([0.3, 0.55, 0.8]),
        per_anchor=(),
        corridor_id=np.asarray(corridor_id, dtype=np.int64),
        corridor_width=np.full(b, 4, dtype=np.int64),
        corridor_direction=np.asarray(corridor_id, dtype=np.int64),
        corridor_min_clearance=np.full(b, 5.0),
        openness_now=1.0,
        evaluated=int(np.sum(shortlist)),
    )


def test_I_corridor_hysteresis_holds_and_switches():
    planner = ActionPlanner()
    # candidate 0 is the new bucket-best (corridor 5); candidate 1 is the
    # previous corridor (2) with only slightly lower viability -> hold corridor 2.
    profile = _profile([5, 2], [0.60, 0.56], [0.40, 0.44], [True, True])
    planner._last_corridor_id = 2
    index, switched, kind = planner._corridor_hysteresis(np.array([0, 1]), 0, profile)
    assert index == 1 and switched is False and kind == "hold"
    assert planner._last_corridor_id == 2

    # clear viability gain -> allow the corridor switch
    profile2 = _profile([5, 2], [0.90, 0.20], [0.10, 0.80], [True, True])
    planner._last_corridor_id = 2
    index2, switched2, kind2 = planner._corridor_hysteresis(np.array([0, 1]), 0, profile2)
    assert index2 == 0 and switched2 is True and kind2 == "switch_viability"

    # previous corridor has no feasible candidate (hard unsafe) -> immediate switch
    planner._last_corridor_id = 7
    index3, switched3, kind3 = planner._corridor_hysteresis(np.array([0, 1]), 0, profile2)
    assert index3 == 0 and switched3 is True and kind3 == "switch_hard_unsafe"
    assert planner._last_corridor_id == 5


# --------------------------------------------------------------------------
# J. anchor clamp
# --------------------------------------------------------------------------
def test_J_anchor_clamp_and_monotonicity():
    evaluator = LongTermViabilityEvaluator(ViabilityConfig(max_long_horizon=0.8))
    for scale in (0.1, 0.5, 2.4, 32.0, None):
        anchors = evaluator.anchors(ViabilityContext(short_horizon=0.3, scene_scale=scale))
        assert anchors.shape == (3,)
        assert anchors[0] >= 0.3 - 1e-9
        assert anchors[0] < anchors[1] < anchors[2]
        assert anchors[2] <= 0.8 + 1e-9
    medium = evaluator.anchors(ViabilityContext(short_horizon=0.3, scene_scale=2.4))
    assert np.allclose(medium, [0.30, 0.55, 0.80], atol=0.02)


# --------------------------------------------------------------------------
# L/M. low-threat fast path + diagnostics
# --------------------------------------------------------------------------
def _envelope(level, margin) -> SafetyEnvelope:
    n = len(level)
    return SafetyEnvelope(
        level=np.asarray(level, dtype=np.int64),
        min_margin=np.asarray(margin, dtype=np.float64),
        closing_rate=np.zeros(n),
        critical=np.asarray(level, dtype=np.int64) == 1,
        unsafe=np.asarray(level, dtype=np.int64) >= 2,
        lateral_critical_risk=np.zeros(n),
    )


def test_L_low_threat_fast_path_requires_all_five_conditions():
    planner = ActionPlanner()
    scene = _scene([])                       # openness_now = 1.0
    feasible = np.array([0, 1])
    candidates = (Action(10.0, 0.0), Action(10.0, math.radians(45.0)))
    envelope = _envelope([0, 0], [50.0, 20.0])

    skip, reason, run = planner._fast_path(
        scene, candidates, feasible, envelope, np.array([0.0, 0.20]), np.array([0, 0]), _ctx()
    )
    assert skip is True and reason.startswith("fast_path") and run == ""

    # a near-tie in short-term risk is NOT a clear choice -> must run LTV
    skip2, _r2, run2 = planner._fast_path(
        scene, candidates, feasible, envelope, np.array([0.0, 0.01]), np.array([0, 0]), _ctx()
    )
    assert skip2 is False and run2 == "fast_path_unmet:clear_choice"

    # critical / low margin also forbid the fast path
    skip3, _r3, run3 = planner._fast_path(
        scene, candidates, feasible, _envelope([1, 0], [1.0, 20.0]),
        np.array([0.0, 0.20]), np.array([1, 0]), _ctx()
    )
    assert skip3 is False and run3.startswith("fast_path_unmet:")


def test_M_diagnostics_record_viability_fields():
    planner = ActionPlanner(horizon=0.3)
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    planner.plan(scene, context=_ctx())
    diag = planner.last_diagnostics
    assert isinstance(diag.viability_skipped, bool)
    assert isinstance(diag.skip_reason, str)
    assert isinstance(diag.viability_run_reason, str)
    assert diag.viability_evaluated >= 0
    assert diag.chosen_viability >= 0.0
    if not diag.viability_skipped:
        assert diag.viability_evaluated <= planner.viability_cfg.top_k
        assert diag.long_anchors is not None and diag.long_anchors.shape == (3,)
        assert diag.viability_run_reason.startswith("fast_path_unmet:")


# --------------------------------------------------------------------------
# N. performance / allocations
# --------------------------------------------------------------------------
def test_N_ltv_evaluates_at_most_k_and_records_timings():
    cfg = ViabilityConfig(top_k=4, leaders=3, diversity=1)
    evaluator = LongTermViabilityEvaluator(cfg)
    scene = _scene([_obstacle(6.0, y * 1.5) for y in (-2, -1, 0, 1, 2)])
    actions = tuple(Action(10.0, math.radians(d)) for d in range(0, 360, 45))
    actions = actions + tuple(Action(5.0, math.radians(d)) for d in range(0, 360, 45))
    profile = evaluator.evaluate(
        scene,
        actions,
        np.linspace(0.0, 0.4, len(actions)),
        np.zeros(len(actions), dtype=np.int64),
        np.ones(len(actions), dtype=bool),
        context=_ctx(),
    )
    assert profile.evaluated <= cfg.top_k
    assert profile.viability.shape == (len(actions),)
    assert np.all(profile.viability >= 0.0) and np.all(profile.viability <= 1.0)

    planner = ActionPlanner(horizon=0.3)
    planner.plan(scene, context=_ctx())
    diag = planner.last_diagnostics
    assert diag.short_term_us >= 0.0
    assert diag.viability_us >= 0.0
    assert diag.total_us >= diag.short_term_us


def test_version_and_build_id():
    from cpu.decision import build as build_mod

    assert VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert BUILD_ID == build_mod.fingerprint()
    assert any(f.endswith("viability.py") for f in build_mod.FINGERPRINT_FILES)
