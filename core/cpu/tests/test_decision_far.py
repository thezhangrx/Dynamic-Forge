"""Future Action Risk (FAR) tests - v0.4.0-FAR.

Covers the required cases A-L: candidate-conditioned relative velocity, risk
trend up/down, continuous closest approach, candidate collision / passing,
threat-aware speed selection, FES + SafetyGate integration, emergency fallback
and the source-fingerprint build id.
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
    SafetyGate,
    SceneState,
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
# A. candidate-specific relative velocity
# --------------------------------------------------------------------------
def test_A_candidate_conditioned_relative_velocity_differs_by_action():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(3.0, 0.0, vx=-2.0),),
        dt=0.1,
        horizon=0.3,
    )
    toward = Action(10.0, 0.0)          # v_rel = -12 x
    away = Action(10.0, math.pi)        # v_rel = +8 x
    safety = candidate_safety_batch(scene, [toward, away], scene.horizon)
    profile = future_action_risk_batch(
        scene, [toward, away], scene.horizon, safety=safety
    )

    # the two actions really do see different relative motion / risk
    assert safety.d_ca[0, 0] < safety.d_ca[1, 0]
    assert profile.current_risk[0] > profile.current_risk[1]
    assert profile.risk_peak[0] > profile.risk_peak[1]


# --------------------------------------------------------------------------
# B. future risk trend: worsening
# --------------------------------------------------------------------------
def test_B_worsening_action_has_positive_risk_trend():
    scene = SceneState(
        agent=_agent(), obstacles=(_obstacle(8.0, 0.0),), dt=0.1, horizon=0.3
    )
    toward = Action(10.0, 0.0)
    away = Action(10.0, math.pi)
    profile = future_action_risk_batch(scene, [toward, away], scene.horizon)

    assert profile.risk_far[0] > profile.risk_near[0]
    assert profile.risk_trend[0] > 0.0          # future is deteriorating
    assert profile.risk_trend[1] == pytest.approx(0.0)  # moving away: flat


# --------------------------------------------------------------------------
# C. risk decreasing action
# --------------------------------------------------------------------------
def test_C_passing_action_has_negative_risk_trend():
    # a fast candidate passes an obstacle with a lateral offset: risk is high
    # before the closest approach and falls to zero once it is receding
    scene = SceneState(
        agent=_agent(radius=0.1, max_speed=10.0),
        obstacles=(_obstacle(1.5, 0.5, radius=0.1),),
        dt=0.1,
        horizon=0.3,
    )
    passing = Action(10.0, 0.0)
    safety = candidate_safety_batch(scene, [passing], scene.horizon)
    profile = future_action_risk_batch(scene, [passing], scene.horizon, safety=safety)

    assert bool(safety.collides[0]) is False     # a clean pass, not a hit
    assert profile.risk_near[0] > profile.risk_far[0]
    assert profile.risk_trend[0] < 0.0           # future is improving
    # the critic only penalises the positive part of the trend
    score = future_risk_critic(profile, [passing], 10.0)
    assert score[0] < 1.0


# --------------------------------------------------------------------------
# D. continuous closest approach
# --------------------------------------------------------------------------
def test_D_closest_approach_matches_dense_sampling():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(5.0, 2.0, vx=-1.0, vy=-0.5),),
        dt=0.01,
        horizon=0.5,
    )
    action = Action(3.0, math.radians(30.0))
    safety = candidate_safety_batch(scene, [action], 0.5)

    r0 = scene.obstacles[0].position - scene.agent.position
    v_rel = scene.obstacles[0].velocity - action.velocity_vector
    ts = np.linspace(0.0, 0.5, 5001)
    d2 = ((r0[None, :] + v_rel[None, :] * ts[:, None]) ** 2).sum(axis=-1)
    t_star = float(ts[int(np.argmin(d2))])

    assert safety.t_ca[0, 0] == pytest.approx(t_star, abs=0.01)
    assert safety.d_ca[0, 0] ** 2 == pytest.approx(float(d2.min()), rel=1e-6, abs=1e-9)


def test_D_stationary_relative_velocity_is_nan_safe():
    # candidate velocity exactly matches the obstacle velocity -> |v_rel| = 0
    scene = SceneState(
        agent=_agent(),
        obstacles=(_obstacle(4.0, 0.0, vx=10.0),),
        dt=0.1,
        horizon=0.3,
    )
    match = Action(10.0, 0.0)
    safety = candidate_safety_batch(scene, [match], scene.horizon)
    assert np.isfinite(safety.t_ca[0, 0])
    assert np.isfinite(safety.d_ca[0, 0])
    assert safety.t_ca[0, 0] == pytest.approx(0.0)
    assert safety.d_ca[0, 0] == pytest.approx(4.0)


# --------------------------------------------------------------------------
# E. candidate collision
# --------------------------------------------------------------------------
def test_E_candidate_collision_is_infeasible():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(3.0, 0.0, radius=0.5),),
        dt=0.1,
        horizon=0.3,
    )
    straight = Action(10.0, 0.0)
    safety = candidate_safety_batch(scene, [straight], scene.horizon)
    profile = future_action_risk_batch(scene, [straight], scene.horizon, safety=safety)

    assert bool(safety.collides[0]) is True
    assert bool(profile.feasible[0]) is False
    assert np.isfinite(profile.min_ttc[0])          # strict first-contact time
    assert profile.min_ttc[0] >= 0.0


# --------------------------------------------------------------------------
# F. non-collision passing trajectory
# --------------------------------------------------------------------------
def test_F_near_but_passing_trajectory_is_not_infeasible():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(8.0, 0.0, radius=0.5),),
        dt=0.1,
        horizon=0.3,
    )
    passing = Action(5.0, 0.0)
    safety = candidate_safety_batch(scene, [passing], scene.horizon)
    profile = future_action_risk_batch(scene, [passing], scene.horizon, safety=safety)

    assert bool(safety.collides[0]) is False
    assert bool(profile.feasible[0]) is True
    assert safety.d_ca[0, 0] > scene.agent.radius + scene.obstacles[0].radius
    assert profile.min_clearance[0] > 0.0


# --------------------------------------------------------------------------
# G/H. threat-aware speed selection
# --------------------------------------------------------------------------
def test_G_high_speed_candidate_is_penalised_under_threat():
    scene = SceneState(
        agent=_agent(radius=0.5),
        obstacles=(_obstacle(12.0, 0.0, radius=0.5),),
        dt=0.1,
        horizon=0.3,
    )
    fast = Action(10.0, 0.0)
    slow = Action(5.0, 0.0)
    safety = candidate_safety_batch(scene, [fast, slow], scene.horizon)
    profile = future_action_risk_batch(scene, [fast, slow], scene.horizon, safety=safety)
    score = future_risk_critic(profile, [fast, slow], 10.0, near_miss_threshold=10.0)

    assert profile.risk_peak[0] > profile.risk_peak[1]
    assert score[0] > score[1]


def test_H_low_risk_environment_does_not_ban_fast():
    scene = SceneState(agent=_agent(radius=0.5), obstacles=(), dt=0.1, horizon=0.3)
    fast = Action(10.0, 0.0)
    slow = Action(5.0, 0.0)
    safety = candidate_safety_batch(scene, [fast, slow], scene.horizon)
    profile = future_action_risk_batch(scene, [fast, slow], scene.horizon, safety=safety)
    score = future_risk_critic(profile, [fast, slow], 10.0)

    assert score[0] == pytest.approx(0.0)
    assert score[1] == pytest.approx(0.0)
    # the planner still selects a fast candidate when nothing threatens it
    chosen = ActionPlanner(horizon=0.3).plan(scene)
    assert chosen.speed > 0.0


# --------------------------------------------------------------------------
# I. FES integration
# --------------------------------------------------------------------------
def test_I_far_coexists_with_fes():
    wall = tuple(
        _obstacle(5.0, y, radius=1.0) for y in np.linspace(-8.0, 8.0, 5)
    )
    scene = SceneState(
        agent=_agent(radius=1.0),
        obstacles=wall,
        goal=np.array([50.0, 0.0]),
        dt=0.1,
        horizon=0.3,
    )
    planner = ActionPlanner(horizon=0.3)
    chosen = planner.plan(scene)

    diag = planner.last_diagnostics
    assert diag is not None
    assert diag.critics["fes"] >= 0.0
    assert diag.n_feasible >= 1

    # the straight-in candidate has a non-zero escape-space penalty
    batch = LinearPredictor().predict_batch(scene, [Action(10.0, 0.0)], scene.n_steps)
    terms = planner.cost.utility_terms(
        planner._trajectory(batch, 0),
        goal=scene.goal,
        start_position=scene.agent.position,
    )
    assert terms.trap > 0.0
    assert chosen is not None


# --------------------------------------------------------------------------
# J. SafetyGate integration
# --------------------------------------------------------------------------
def test_J_far_coexists_with_safety_gate():
    scene = SceneState(
        agent=_agent(radius=1.0),
        obstacles=(_obstacle(3.0, 0.0, radius=1.0),),
        dt=0.1,
        horizon=0.3,
    )
    # a configurable clearance margin can make every candidate infeasible
    planner = ActionPlanner(
        safety_gate=SafetyGate(collision_threshold=1e9, near_miss_threshold=1e9 + 1.0),
        horizon=0.3,
    )
    planner.plan(scene)
    assert planner.last_diagnostics is not None
    assert planner.last_diagnostics.used_emergency is True

    # the default gate keeps candidates feasible
    default = ActionPlanner(horizon=0.3)
    default.plan(scene)
    assert default.last_diagnostics.n_feasible >= 1


# --------------------------------------------------------------------------
# K. emergency fallback
# --------------------------------------------------------------------------
def test_K_emergency_fallback_is_deterministic():
    scene = SceneState(
        agent=_agent(radius=1.0, max_speed=1.0),
        obstacles=(_obstacle(0.0, 0.0, radius=5.0),),
        dt=0.1,
        horizon=0.3,
    )
    first_planner = ActionPlanner(horizon=0.3)
    first = first_planner.plan(scene)
    diag = first_planner.last_diagnostics
    assert diag is not None
    assert diag.used_emergency is True
    assert diag.n_feasible == 0
    assert diag.chosen_action == first
    assert first.is_stop is False            # escape beats standing still
    assert ActionPlanner(horizon=0.3).plan(scene) == first


# --------------------------------------------------------------------------
# L. version + source fingerprint build id
# --------------------------------------------------------------------------
def test_L_version_and_source_fingerprint_build_id(tmp_path):
    from cpu.decision import build as build_mod

    assert VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert BUILD_ID.startswith("cpudec-")
    assert len(BUILD_ID) == len("cpudec-") + 12
    # the exported id really is the source fingerprint of this checkout
    assert BUILD_ID == build_mod.fingerprint()

    # a byte change in any fingerprinted source changes the id
    (tmp_path / "a.py").write_text("x = 1\n", encoding="utf-8")
    first = build_mod.fingerprint(tmp_path, ("a.py",))
    (tmp_path / "a.py").write_text("x = 2\n", encoding="utf-8")
    second = build_mod.fingerprint(tmp_path, ("a.py",))
    assert first != second
    assert build_mod.fingerprint(tmp_path, ("a.py",)) == second  # stable


def test_decision_version_surfaces():
    from cpu.decision import CpuDecisionLayer

    desc = CpuDecisionLayer().describe()
    assert desc["version"] == "v0.6.0-PredictiveCorridorSelector"
    assert desc["build_id"] == BUILD_ID
    assert desc["critics"]["risk"] > desc["critics"]["fes"] + desc["critics"]["behavior"]
    assert desc["critics"]["critical"] > desc["critics"]["risk"]
    assert desc["safety_gate"]["collision_threshold"] == 0.0
    assert len(default_candidate_actions(200.0)) == 17
