"""ReactiveGap tests - v0.5.0-DualPlanner.

Covers the shared one-frame geometry primitives and the reactive planner:
sectorization, current-frame closing rate, immediate safety margin, gap scoring,
goal-aware selection, emergency, safety-first hysteresis, STOP gating,
threat-aware speed, diagnostics and the reactive/predictive boundary.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest

from cpu.baseline_reactive_gap import ReactiveGapController
from cpu.decision import (
    BUILD_ID,
    PREDICTIVE_VERSION,
    REACTIVE_VERSION,
    STACK_VERSION,
    Action,
    AgentState,
    Obstacle,
    ReactiveConfig,
    ReactiveGapPlanner,
    SceneState,
)
from cpu.decision.immediate import (
    SECTOR_DIRECTIONS,
    gap_scores,
    radial_closing_rate,
    sectorize,
)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=10.0, vx=0.0, vy=0.0) -> AgentState:
    return AgentState(
        position=(x, y), velocity=(vx, vy), radius=radius, max_speed=max_speed
    )


def _obstacle(x, y, vx=0.0, vy=0.0, radius=0.5) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


def _scene(obstacles=(), goal=(50.0, 0.0), agent=None) -> SceneState:
    g = None if goal is None else np.asarray(goal, dtype=np.float64)
    return SceneState(
        agent=agent or _agent(),
        obstacles=tuple(obstacles),
        goal=g,
        dt=1.0 / 120.0,
        horizon=0.0,          # reactive: no horizon at all
    )


# --------------------------------------------------------------------------
# A. sectorization
# --------------------------------------------------------------------------
def test_A_sectorize_maps_directions_and_clearance():
    r = np.array([[10.0, 0.0], [0.0, 10.0], [-5.0, 0.0], [0.0, -7.0]])
    v_rel = np.zeros((4, 2))
    summary = sectorize(r, v_rel, np.ones(4))

    assert summary.sector_of_obstacle.tolist() == [0, 2, 4, 6]
    assert summary.density.tolist() == [1, 0, 1, 0, 1, 0, 1, 0]
    assert summary.nearest_clearance[0] == pytest.approx(9.0)
    assert summary.nearest_clearance[2] == pytest.approx(9.0)
    assert summary.nearest_clearance[4] == pytest.approx(4.0)
    assert summary.nearest_clearance[6] == pytest.approx(6.0)
    assert summary.nearest_clearance[1] == np.inf


def test_A_sectorize_aggregates_two_obstacles_in_one_sector():
    r = np.array([[10.0, 0.0], [6.0, 0.0]])
    summary = sectorize(r, np.zeros((2, 2)), np.ones(2))
    assert summary.sector_of_obstacle.tolist() == [0, 0]
    assert summary.density[0] == pytest.approx(2.0)
    assert summary.nearest_clearance[0] == pytest.approx(5.0)  # the closer one


# --------------------------------------------------------------------------
# B. current-frame closing rate
# --------------------------------------------------------------------------
def test_B_closing_rate_sign_is_approach_dependent():
    r = np.array([[10.0, 0.0]])
    approaching = radial_closing_rate(r, np.array([[-5.0, 0.0]]))
    receding = radial_closing_rate(r, np.array([[5.0, 0.0]]))
    tangential = radial_closing_rate(r, np.array([[0.0, 5.0]]))

    assert approaching[0] > 0.0
    assert receding[0] < 0.0
    assert tangential[0] == pytest.approx(0.0)

    summary = sectorize(r, np.array([[-5.0, 0.0]]), np.ones(1))
    assert summary.closing_rate[0] == pytest.approx(5.0)
    summary_back = sectorize(r, np.array([[5.0, 0.0]]), np.ones(1))
    assert summary_back.closing_rate[0] == pytest.approx(0.0)  # never negative


# --------------------------------------------------------------------------
# C. immediate safety margin
# --------------------------------------------------------------------------
def test_C_closing_rate_reduces_the_immediate_margin():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    planner = ReactiveGapPlanner()
    planner.plan(scene)
    diag = planner.last_diagnostics

    # geometric clearance in sector 0 is 5.0, but the current closing rate 8
    # produces a much smaller (critical) immediate margin
    assert diag.sector_clearance[0] == pytest.approx(5.0)
    assert diag.sector_closing[0] == pytest.approx(8.0)
    assert diag.sector_margin[0] == pytest.approx(5.0 - 8.0 * 0.45)
    assert diag.sector_margin[0] < 4.0


# --------------------------------------------------------------------------
# D. gap scoring
# --------------------------------------------------------------------------
def test_D_gap_score_uses_both_neighbours():
    clearance = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0])
    gap = gap_scores(clearance, 0.5)
    assert gap[0] == pytest.approx(1.0 + 0.5 * (10.0 + 0.0))
    assert gap[6] == pytest.approx(0.0 + 0.5 * (0.0 + 10.0))


# --------------------------------------------------------------------------
# E. goal-aware selection
# --------------------------------------------------------------------------
def test_E_clear_scene_follows_the_goal_direction():
    planner = ReactiveGapPlanner()
    action = planner.plan(_scene([], goal=(50.0, 0.0)))
    assert planner.last_diagnostics.sector == 0
    assert action.steering_angle == pytest.approx(0.0)

    action_up = planner.plan(_scene([], goal=(0.0, 50.0)))
    assert planner.last_diagnostics.sector == 2
    assert action_up.steering_angle == pytest.approx(math.pi / 2)


# --------------------------------------------------------------------------
# F. safety filtering + emergency
# --------------------------------------------------------------------------
def test_F_critical_direction_is_avoided_but_not_infeasible():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)], goal=(50.0, 0.0))
    planner = ReactiveGapPlanner()
    planner.plan(scene)
    diag = planner.last_diagnostics
    # +x carries the critical penalty, so the controller does not drive into it
    assert diag.sector != 0
    assert not diag.emergency_active


def test_F_emergency_when_every_sector_is_unsafe():
    obstacles = [
        _obstacle(float(np.cos(a) * 1.0), float(np.sin(a) * 1.0), radius=1.0)
        for a in [np.deg2rad(d) for d in range(0, 360, 45)]
    ]
    planner = ReactiveGapPlanner()
    action = planner.plan(_scene(obstacles, goal=None))
    diag = planner.last_diagnostics
    assert diag.emergency_active is True
    assert isinstance(action, Action)
    assert action.speed > 0.0          # emergency escapes, it does not freeze


# --------------------------------------------------------------------------
# G. safety-first hysteresis
# --------------------------------------------------------------------------
def test_G_hysteresis_holds_a_safe_direction_that_is_slightly_worse():
    planner = ReactiveGapPlanner()
    planner._last_choice = 1
    # goal exactly between sectors 0 and 1 -> the two are (near) tied
    angle = math.radians(22.5)
    goal = (50.0 * math.cos(angle), 50.0 * math.sin(angle))
    moving = _agent(vx=5.0, vy=5.0)   # the held 45 deg command really executed
    planner.plan(_scene([], goal=goal, agent=moving))
    diag = planner.last_diagnostics
    assert diag.sector == 1
    assert diag.action_switched is False


def test_G_hysteresis_switches_immediately_when_held_is_critical():
    planner = ReactiveGapPlanner()
    planner._last_choice = 0           # critically threatened by the incoming bullet
    planner.plan(_scene([_obstacle(6.0, 0.0, vx=-8.0)], goal=(50.0, 0.0)))
    diag = planner.last_diagnostics
    assert diag.sector != 0
    assert diag.action_switched is True


# --------------------------------------------------------------------------
# H. STOP gating
# --------------------------------------------------------------------------
def test_H_stop_is_only_allowed_when_its_own_margin_is_safe():
    planner = ReactiveGapPlanner()
    cfg = ReactiveConfig()
    # obstacle closing fast: STOP would have a negative immediate margin
    unsafe_stop = planner._stop_margin(
        1,
        np.array([[1.0, 0.0]]),
        np.array([[-10.0, 0.0]]),
        np.array([1.0]),
        cfg,
    )
    # obstacle stationary: STOP is safe
    safe_stop = planner._stop_margin(
        1,
        np.array([[3.0, 0.0]]),
        np.array([[0.0, 0.0]]),
        np.array([1.0]),
        cfg,
    )
    assert unsafe_stop < 0.0
    assert safe_stop > 0.0


def test_H_surrounded_scene_slows_to_a_safe_stop():
    obstacles = [
        _obstacle(float(np.cos(a) * 3.0), float(np.sin(a) * 3.0), radius=1.0)
        for a in [np.deg2rad(d) for d in range(0, 360, 45)]
    ]
    planner = ReactiveGapPlanner()
    planner.plan(_scene(obstacles, goal=(50.0, 0.0)))
    diag = planner.last_diagnostics
    assert diag.chosen_speed_scale < 0.5
    assert diag.stopped is True        # best margin small and STOP is safe


# --------------------------------------------------------------------------
# I. threat-aware speed
# --------------------------------------------------------------------------
def test_I_speed_scale_drops_with_threat_and_recovers_when_clear():
    clear = ReactiveGapPlanner()
    clear.plan(_scene([], goal=(50.0, 0.0)))
    assert clear.last_diagnostics.chosen_speed_scale == pytest.approx(1.0)
    assert clear.last_diagnostics.stopped is False

    surrounded = [
        _obstacle(float(np.cos(a) * 3.0), float(np.sin(a) * 3.0), radius=1.0)
        for a in [np.deg2rad(d) for d in range(0, 360, 45)]
    ]
    threatened = ReactiveGapPlanner()
    threatened.plan(_scene(surrounded, goal=(50.0, 0.0)))
    assert threatened.last_diagnostics.chosen_speed_scale < 1.0


# --------------------------------------------------------------------------
# J. diagnostics + versions
# --------------------------------------------------------------------------
def test_J_diagnostics_and_versions():
    planner = ReactiveGapPlanner()
    planner.plan(_scene([], goal=(50.0, 0.0)))
    diag = planner.last_diagnostics

    assert diag.version == REACTIVE_VERSION
    assert diag.build_id == BUILD_ID
    assert diag.sector_clearance.shape == (8,)
    assert diag.sector_score.shape == (8,)
    assert isinstance(diag.chosen_action, Action)

    assert STACK_VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert REACTIVE_VERSION == "v0.5.0-ReactiveGap"
    assert PREDICTIVE_VERSION == "v0.6.0-PredictiveCorridorSelector"


# --------------------------------------------------------------------------
# K. reactive/predictive boundary
# --------------------------------------------------------------------------
def test_K_reactive_contains_no_future_machinery():
    import ast

    import cpu.decision.reactive as reactive_mod

    source = pathlib.Path(reactive_mod.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    identifiers = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    identifiers |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}

    for forbidden in (
        "risk_near",
        "risk_far",
        "risk_peak",
        "candidate_safety_batch",
        "future_action_risk_batch",
        "horizon",
        "predictor",
    ):
        assert forbidden not in identifiers, forbidden

    planner = ReactiveGapPlanner()
    assert not hasattr(planner, "horizon")
    assert not hasattr(planner, "predictor")


# --------------------------------------------------------------------------
# L. controller wrapper (synthetic observation, no episode)
# --------------------------------------------------------------------------
def test_L_controller_wrapper_returns_a_valid_discrete_action():
    observation = {
        "player": np.array([0.0, 0.0, 0.0, 0.0, 0.5, 10.0, 1.0]),
        "target": np.array([50.0, 0.0, 0.5, 50.0]),
        "bullets": np.zeros((0, 7)),
    }
    controller = ReactiveGapController()
    controller.reset()
    action = controller.act(observation, {})

    assert action in range(9)
    assert action != 0                      # clear scene: move toward the target
    assert controller.last_diagnostics is not None
    assert controller.last_diagnostics.version == REACTIVE_VERSION
    assert controller.spec().access == "observation"
