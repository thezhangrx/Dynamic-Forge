"""Adaptive Safety Envelope / side-hit tests - v0.4.1-SideHit-ASE.

Covers the required cases A-O: candidate-specific closing rate, lateral critical
risk (raised by a real lateral approach, silent without closing), safety margin
and critical levels, adaptive refinement trigger/cap, safety-first hysteresis,
threat-aware speed selection, FES regression, SafetyGate, emergency fallback and
the source-fingerprint build id.
"""

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
    LinearPredictor,
    Obstacle,
    SafetyEnvelopeConfig,
    SafetyGate,
    SceneState,
    adaptive_safety_envelope,
    candidate_safety_batch,
    default_candidate_actions,
    future_action_risk_batch,
    future_risk_critic,
)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=10.0) -> AgentState:
    return AgentState(position=(x, y), velocity=(0.0, 0.0), radius=radius, max_speed=max_speed)


def _obstacle(x, y, vx=0.0, vy=0.0, radius=0.5) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


# --------------------------------------------------------------------------
# A. closing_rate is candidate-specific
# --------------------------------------------------------------------------
def test_A_closing_rate_is_candidate_specific():
    scene = SceneState(
        agent=_agent(), obstacles=(_obstacle(6.0, 0.0, vx=-2.0),), dt=0.1, horizon=0.3
    )
    toward = Action(10.0, 0.0)
    away = Action(10.0, math.pi)
    lateral = Action(10.0, math.pi / 2.0)
    profile = future_action_risk_batch(scene, [toward, away, lateral], scene.horizon)

    assert profile.closing_rate[0] > profile.closing_rate[2] > profile.closing_rate[1]
    assert profile.closing_rate[1] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# B. lateral critical: a real lateral approach raises risk
# --------------------------------------------------------------------------
def test_B_fast_lateral_approach_raises_lateral_critical_risk():
    # agent moves +y while an obstacle closes from +x: purely lateral closing
    scene = SceneState(
        agent=_agent(), obstacles=(_obstacle(6.0, 0.0, vx=-8.0),), dt=0.1, horizon=0.3
    )
    lateral_move = Action(10.0, math.pi / 2.0)   # crosses the incoming obstacle path
    receding = Action(10.0, math.pi)             # runs with it: no closing at all
    profile = future_action_risk_batch(scene, [lateral_move, receding], scene.horizon)

    assert profile.closing_rate[0] > 5.0
    assert profile.lateral_critical_risk[0] > 0.25
    assert profile.lateral_critical_risk[0] > profile.lateral_critical_risk[1]
    assert profile.lateral_critical_risk[1] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# C. lateral non-threat: no closing -> no false positive
# --------------------------------------------------------------------------
def test_C_no_closing_means_no_lateral_critical_risk():
    # the obstacle recedes; a lateral move must not be flagged
    scene = SceneState(
        agent=_agent(), obstacles=(_obstacle(6.0, 0.0, vx=8.0),), dt=0.1, horizon=0.3
    )
    lateral_move = Action(10.0, math.pi / 2.0)
    profile = future_action_risk_batch(scene, [lateral_move], scene.horizon)

    assert profile.closing_rate[0] == pytest.approx(0.0)
    assert profile.lateral_critical_risk[0] == pytest.approx(0.0)
    assert adaptive_safety_envelope(profile).level[0] == 0


# --------------------------------------------------------------------------
# D. negative safety margin -> unsafe (level 2), even without a collision
# --------------------------------------------------------------------------
def test_D_negative_safety_margin_is_unsafe():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(12.0, 0.0, vx=-10.0),),
        dt=0.1,
        horizon=0.3,
    )
    heading = Action(10.0, 0.0)
    safety = candidate_safety_batch(scene, [heading], scene.horizon)
    profile = future_action_risk_batch(
        scene,
        [heading],
        scene.horizon,
        safety=safety,
        base_margin=5.0,
        margin_time=0.1,
        critical_margin=4.0,
    )

    assert bool(safety.collides[0]) is False       # no geometric collision
    assert profile.min_margin[0] < 0.0             # but the adaptive buffer is violated
    assert profile.envelope_level[0] == 2
    assert bool(profile.feasible[0]) is False


# --------------------------------------------------------------------------
# E. critical margin: critical != collision
# --------------------------------------------------------------------------
def test_E_critical_candidate_is_not_a_collision():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(4.0, 0.0, vx=-2.0),),
        dt=0.1,
        horizon=0.3,
    )
    escape = Action(10.0, math.pi / 2.0)           # move up, away from the threat
    safety = candidate_safety_batch(scene, [escape], scene.horizon)
    profile = future_action_risk_batch(scene, [escape], scene.horizon, safety=safety)
    envelope = adaptive_safety_envelope(profile)

    assert bool(safety.collides[0]) is False
    assert envelope.level[0] == 1                  # critical
    assert bool(envelope.unsafe[0]) is False       # still feasible
    assert bool(profile.feasible[0]) is True
    assert profile.min_margin[0] < 4.0


# --------------------------------------------------------------------------
# F/G. adaptive refinement trigger + cap
# --------------------------------------------------------------------------
def test_F_refinement_not_triggered_at_low_risk():
    empty = SceneState(agent=_agent(), obstacles=(), dt=0.1, horizon=0.3)
    planner = ActionPlanner(horizon=0.3)
    planner.plan(empty)
    assert planner.last_diagnostics.refinement_used is False
    assert planner.last_diagnostics.refinement_count == 0

    far = SceneState(
        agent=_agent(), obstacles=(_obstacle(80.0, 0.0),), dt=0.1, horizon=0.3
    )
    planner2 = ActionPlanner(horizon=0.3)
    planner2.plan(far)
    assert planner2.last_diagnostics.refinement_used is False


def test_G_refinement_triggers_at_high_risk_with_at_most_eight_extra():
    # a close but non-colliding obstacle: the best escape is still critical
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(2.0, 0.0, vx=-2.0),),
        dt=0.1,
        horizon=0.3,
    )
    planner = ActionPlanner(horizon=0.3)
    planner.plan(scene)
    diag = planner.last_diagnostics

    assert diag.refinement_used is True
    assert 0 < diag.refinement_count <= 8
    assert planner.refinement_cfg.max_extra == 8


# --------------------------------------------------------------------------
# H/I. action hysteresis (safety-first)
# --------------------------------------------------------------------------
class _ProfileStub:
    """Minimal profile stub for the hysteresis unit tests."""

    def __init__(self, level, trend):
        self.envelope_level = np.asarray(level, dtype=np.int64)
        self.risk_trend = np.asarray(trend, dtype=np.float64)


def test_H_hysteresis_holds_when_the_new_action_is_only_slightly_better():
    planner = ActionPlanner()
    candidates = (Action(10.0, 0.0), Action(10.0, math.radians(7.5)))
    planner._last_action = candidates[0]
    profile = _ProfileStub([0, 0], [0.0, 0.0])
    total = np.array([100.0, 95.0])                # only 5 better (< switch_margin 30)
    critical = np.array([0.0, 0.0])
    risk = np.array([0.0, 0.0])

    index, switched, reason = planner._apply_hysteresis(
        candidates, total, critical, risk, profile, best_index=1
    )
    assert index == 0
    assert switched is False
    assert reason == "hysteresis_hold"


def test_I_hysteresis_switches_immediately_when_held_is_dangerous():
    planner = ActionPlanner()
    candidates = (Action(10.0, 0.0), Action(10.0, math.radians(7.5)))
    planner._last_action = candidates[0]
    critical = np.array([0.0, 0.0])
    risk = np.array([0.0, 0.0])
    total = np.array([0.0, 100.0])                 # held *looks* cheaper

    # held unsafe -> must switch
    index, switched, reason = planner._apply_hysteresis(
        candidates, total, critical, risk, _ProfileStub([2, 0], [0.0, 0.0]), 1
    )
    assert (index, switched, reason) == (1, True, "held_unsafe")

    # held critical -> must switch
    index, switched, reason = planner._apply_hysteresis(
        candidates, total, critical, risk, _ProfileStub([1, 0], [0.0, 0.0]), 1
    )
    assert (index, switched, reason) == (1, True, "held_critical")

    # held trend worsening -> must switch
    index, switched, reason = planner._apply_hysteresis(
        candidates, total, critical, risk, _ProfileStub([0, 0], [0.5, 0.0]), 1
    )
    assert (index, switched, reason) == (1, True, "held_trend_worsening")

    # clear safety gain -> switch
    index, switched, reason = planner._apply_hysteresis(
        candidates, total, np.array([0.10, 0.0]), risk, _ProfileStub([0, 0], [0.0, 0.0]), 1
    )
    assert (index, switched, reason) == (1, True, "clear_critical_gain")

    # clear total gain -> switch
    index, switched, reason = planner._apply_hysteresis(
        candidates,
        np.array([1000.0, 0.0]),
        critical,
        risk,
        _ProfileStub([0, 0], [0.0, 0.0]),
        1,
    )
    assert (index, switched, reason) == (1, True, "clear_gain")


# --------------------------------------------------------------------------
# J/K. threat-aware speed selection
# --------------------------------------------------------------------------
def test_J_fast_penalty_increases_with_threat():
    low = SceneState(agent=_agent(), obstacles=(_obstacle(60.0, 0.0),), dt=0.1, horizon=0.3)
    high = SceneState(
        agent=_agent(), obstacles=(_obstacle(6.0, 0.0, vx=-8.0),), dt=0.1, horizon=0.3
    )
    fast, slow = Action(10.0, 0.0), Action(5.0, 0.0)

    low_score = future_risk_critic(
        future_action_risk_batch(low, [fast, slow], low.horizon),
        [fast, slow],
        10.0,
        near_miss_threshold=10.0,
    )
    high_score = future_risk_critic(
        future_action_risk_batch(high, [fast, slow], high.horizon),
        [fast, slow],
        10.0,
        near_miss_threshold=10.0,
    )

    assert low_score[0] == pytest.approx(0.0)
    assert high_score[0] > low_score[0]
    assert high_score[0] >= high_score[1]          # FAST is not cheaper under threat


def test_K_fast_stays_available_when_threat_is_low():
    scene = SceneState(agent=_agent(), obstacles=(_obstacle(60.0, 0.0),), dt=0.1, horizon=0.3)
    fast, slow = Action(10.0, 0.0), Action(5.0, 0.0)
    profile = future_action_risk_batch(scene, [fast, slow], scene.horizon)
    score = future_risk_critic(profile, [fast, slow], 10.0)

    assert score[0] == pytest.approx(0.0)
    assert score[1] == pytest.approx(0.0)
    assert ActionPlanner(horizon=0.3).plan(scene).speed > 0.0


# --------------------------------------------------------------------------
# L. FES regression: corner trap still penalised
# --------------------------------------------------------------------------
def test_L_fes_still_penalises_a_corner_trap():
    obstacles = (
        _obstacle(8.0, -2.0, radius=1.0),
        _obstacle(8.0, 0.0, radius=1.0),
        _obstacle(8.0, 2.0, radius=1.0),
        _obstacle(10.0, 3.0, radius=1.0),
        _obstacle(10.0, -3.0, radius=1.0),
    )
    scene = SceneState(
        agent=_agent(radius=1.0),
        obstacles=obstacles,
        goal=np.array([20.0, 0.0]),
        dt=0.1,
        horizon=0.3,
    )
    planner = ActionPlanner(horizon=0.3)
    chosen = planner.plan(scene)
    assert planner.last_diagnostics.critics["fes"] >= 0.0

    def trap_of(action: Action) -> float:
        batch = LinearPredictor().predict_batch(scene, [action], scene.n_steps)
        return planner.cost.utility_terms(
            planner._trajectory(batch, 0),
            goal=scene.goal,
            start_position=scene.agent.position,
        ).trap

    straight_trap = trap_of(Action(10.0, 0.0))
    assert straight_trap > 0.0
    assert trap_of(chosen) <= straight_trap + 1e-9


# --------------------------------------------------------------------------
# M/N. SafetyGate + emergency fallback preserved
# --------------------------------------------------------------------------
def test_M_safety_gate_is_still_applied():
    scene = SceneState(
        agent=_agent(radius=1.0),
        obstacles=(_obstacle(3.0, 0.0, radius=1.0),),
        dt=0.1,
        horizon=0.3,
    )
    planner = ActionPlanner(
        safety_gate=SafetyGate(collision_threshold=1e9, near_miss_threshold=1e9 + 1.0),
        horizon=0.3,
    )
    planner.plan(scene)
    assert planner.last_diagnostics.used_emergency is True


def test_N_emergency_fallback_is_deterministic():
    scene = SceneState(
        agent=_agent(radius=1.0, max_speed=1.0),
        obstacles=(_obstacle(0.0, 0.0, radius=5.0),),
        dt=0.1,
        horizon=0.3,
    )
    planner = ActionPlanner(horizon=0.3)
    first = planner.plan(scene)
    diag = planner.last_diagnostics
    assert diag.used_emergency is True
    assert diag.n_feasible == 0
    assert diag.chosen_action == first
    assert first.is_stop is False
    assert ActionPlanner(horizon=0.3).plan(scene) == first


# --------------------------------------------------------------------------
# O. version + build id
# --------------------------------------------------------------------------
def test_O_build_id_is_a_source_fingerprint(tmp_path):
    from cpu.decision import build as build_mod

    assert VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert BUILD_ID.startswith("cpudec-") and len(BUILD_ID) == len("cpudec-") + 12
    assert BUILD_ID == build_mod.fingerprint()
    assert any(f.endswith("build.py") for f in build_mod.FINGERPRINT_FILES)
    assert any(f.endswith("reactive_gap.py") for f in build_mod.FINGERPRINT_FILES)

    (tmp_path / "m.py").write_text("a = 1\n", encoding="utf-8")
    first = build_mod.fingerprint(tmp_path, ("m.py",))
    (tmp_path / "m.py").write_text("a = 2\n", encoding="utf-8")
    second = build_mod.fingerprint(tmp_path, ("m.py",))
    assert first != second
    assert build_mod.fingerprint(tmp_path, ("m.py",)) == second


def test_O_candidate_space_stays_17_and_configs_default():
    from cpu.decision import (
        HierarchicalCritics,
        HysteresisConfig,
        RefinementConfig,
    )

    assert len(default_candidate_actions(200.0)) == 17
    envelope = SafetyEnvelopeConfig()
    assert envelope.critical_margin > 0.0 and envelope.margin_time >= 0.0
    assert 0 <= RefinementConfig().max_extra <= 8
    assert HysteresisConfig().enabled is True
    tiers = HierarchicalCritics()
    assert tiers.critical > tiers.risk > tiers.fes > tiers.behavior > tiers.task
