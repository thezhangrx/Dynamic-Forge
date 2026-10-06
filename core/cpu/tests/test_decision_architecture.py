"""The refactored, game-agnostic CPU decision architecture.

Covers the ``cpu/decision`` package (models / predictor / risk / planner /
core) and the ``cpu/adapters`` bridge, including an end-to-end drive of the
real simulator.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest

from cpu.adapters.game_adapter import (
    snapshot_to_scene,
    to_game_action,
)
from cpu.decision import (
    Action,
    ActionPlanner,
    AgentState,
    CpuDecisionLayer,
    LinearPredictor,
    Obstacle,
    RiskEvaluator,
    SceneState,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.world import World


# --------------------------------------------------------------------------
# models
# --------------------------------------------------------------------------


def test_models_are_game_agnostic_and_validated():
    agent = AgentState(position=(1, 2), velocity=(0.5, 0.0), radius=1.0, max_speed=2.0)
    assert agent.x == 1.0 and agent.y == 2.0
    assert agent.speed == pytest.approx(0.5)

    obs = Obstacle(position=(5, 5), velocity=(-1, 0), radius=0.5)
    assert obs.speed == pytest.approx(1.0)

    action = Action(speed=2.0, steering_angle=np.pi / 2)
    v = action.velocity_vector
    assert np.allclose(v, [0.0, 2.0], atol=1e-9)
    assert action.is_stop is False
    assert Action.stop().is_stop is True

    with pytest.raises(ValueError):
        AgentState(position=(1, 2, 3), velocity=(0, 0))
    with pytest.raises(ValueError):
        Action(speed=-1.0, steering_angle=0.0)


# --------------------------------------------------------------------------
# predictor (linear only, first milestone)
# --------------------------------------------------------------------------


def test_linear_predictor_extrapolates_constant_velocity():
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=1.0)
    obstacle = Obstacle(position=(4.0, 0.0), velocity=(1.0, 0.0), radius=0.5)
    scene = SceneState(agent=agent, obstacles=(obstacle,), dt=0.1, horizon=1.0)

    pred = LinearPredictor()
    traj = pred.predict(scene, Action(1.0, 0.0), n_steps=3)

    assert traj.agent_positions.shape == (3, 2)
    assert traj.obstacle_positions.shape == (3, 1, 2)
    # agent moved +x under the action's velocity (steps 1,2,3 -> x=0.1,0.2,0.3)
    assert traj.agent_positions[2, 0] == pytest.approx(0.3)
    # obstacle moved +x under its own constant velocity
    assert traj.obstacle_positions[2, 0, 0] == pytest.approx(4.3)


# --------------------------------------------------------------------------
# risk
# --------------------------------------------------------------------------


def test_risk_is_distance_based_and_bounded():
    risk = RiskEvaluator(epsilon=0.1)
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0)

    assert risk.risk(agent, ()) == 0.0  # no obstacles -> no risk

    # overlapping: surface distance 0 -> risk clamps to 1.0
    overlap = Obstacle(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=0.5)
    assert risk.risk(agent, (overlap,)) == pytest.approx(1.0)

    near = Obstacle(position=(1.0, 0.0), velocity=(0.0, 0.0), radius=0.0)
    far = Obstacle(position=(100.0, 0.0), velocity=(0.0, 0.0), radius=0.0)
    assert risk.risk(agent, (near,)) > risk.risk(agent, (far,))
    assert 0.0 < risk.risk(agent, (far,)) < 0.1


# --------------------------------------------------------------------------
# planner
# --------------------------------------------------------------------------


def test_planner_avoids_driving_into_an_obstacle():
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=1.0)
    obstacle = Obstacle(position=(3.0, 0.0), velocity=(0.0, 0.0), radius=0.5)
    scene = SceneState(agent=agent, obstacles=(obstacle,), dt=0.1, horizon=1.0)

    planner = ActionPlanner()
    chosen = planner.plan(scene)

    forward = Action(1.0, 0.0)  # straight into the obstacle

    def worst_risk(action: Action) -> float:
        pred = LinearPredictor()
        r = RiskEvaluator()
        traj = pred.predict(scene, action, scene.n_steps)
        return r.risk_trajectory(traj)

    # the planner must pick an action no worse than driving straight ahead
    assert worst_risk(chosen) <= worst_risk(forward)
    # ...and driving straight ahead must actually be dangerous in this scene
    assert worst_risk(forward) > 0.5
    assert worst_risk(forward) > worst_risk(Action.stop())


def test_planner_default_action_space_covers_the_full_circle():
    from cpu.decision.planner import default_candidate_actions

    actions = default_candidate_actions(200.0)
    # stop + 8 headings x 2 moving speeds, within the 21-candidate budget
    assert len(actions) == 17
    assert sum(1 for a in actions if a.is_stop) == 1

    speeds = sorted({round(a.speed, 6) for a in actions if a.speed > 0.0})
    assert speeds == [100.0, 200.0]

    angles = sorted(
        {round(math.degrees(a.steering_angle)) % 360 for a in actions if a.speed > 0.0}
    )
    assert angles == list(range(0, 360, 45))


def test_planner_can_escape_leftward_when_the_right_is_blocked():
    """Regression (v0.3-4): the old absolute +/-45 grid could never go left."""
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    obstacles = (Obstacle(position=(5.0, 0.0), velocity=(0.0, 0.0), radius=3.0),)
    scene = SceneState(
        agent=agent, obstacles=obstacles, goal=np.array([-50.0, 0.0]), dt=0.1, horizon=1.0
    )

    chosen = ActionPlanner().plan(scene)
    assert chosen.speed > 0.0
    assert chosen.velocity_vector[0] < 0.0


# --------------------------------------------------------------------------
# safety-gated planning (v0.3-5-SafetyGate)
# --------------------------------------------------------------------------


def _batch_for(scene, actions, n_steps):
    return LinearPredictor().predict_batch(scene, actions, n_steps)


def test_safety_gate_marks_a_colliding_trajectory_infeasible():
    from cpu.decision import SafetyGate, TrajectoryCost

    cost = TrajectoryCost()
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    obstacle = Obstacle(position=(5.0, 0.0), velocity=(0.0, 0.0), radius=1.0)
    scene = SceneState(agent=agent, obstacles=(obstacle,), dt=0.1, horizon=1.0)

    straight_in = _batch_for(scene, [Action(10.0, 0.0)], scene.n_steps)  # drives through it
    metrics = cost.safety_metrics_batch(straight_in)
    gate = SafetyGate()

    assert metrics.min_clearance[0] < 0.0
    assert bool(gate.infeasible(metrics)[0]) is True


def test_safety_gate_collision_threshold_is_configurable():
    from cpu.decision import SafetyGate, SafetyMetrics

    metrics = SafetyMetrics(min_clearance=np.array([0.5]), ttc_min=np.array([1.0]))

    assert bool(SafetyGate(collision_threshold=0.0).infeasible(metrics)[0]) is False
    assert bool(SafetyGate(collision_threshold=1.0).infeasible(metrics)[0]) is True

    with pytest.raises(ValueError):
        SafetyGate(collision_threshold=1.0, near_miss_threshold=1.0)


def test_safety_gate_picks_the_safe_trajectory_over_a_higher_progress_collision():
    from cpu.decision import TrajectoryCost

    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    obstacle = Obstacle(position=(5.0, 0.0), velocity=(0.0, 0.0), radius=1.0)
    goal = np.array([20.0, 0.0])  # straight ahead, behind the obstacle
    scene = SceneState(agent=agent, obstacles=(obstacle,), goal=goal, dt=0.1, horizon=1.0)

    cost = TrajectoryCost()
    straight = Action(10.0, 0.0)
    straight_metrics = cost.safety_metrics_batch(_batch_for(scene, [straight], scene.n_steps))
    assert straight_metrics.min_clearance[0] < 0.0  # the greedy candidate collides

    def progress(action: Action) -> float:
        traj = LinearPredictor().predict(scene, action, scene.n_steps)
        start = float(np.hypot(*(agent.position - goal)))
        end = float(np.hypot(*(traj.agent_positions[-1] - goal)))
        return (start - end) / start

    chosen = ActionPlanner().plan(scene)
    chosen_metrics = cost.safety_metrics_batch(_batch_for(scene, [chosen], scene.n_steps))

    # the chosen action is feasible...
    assert chosen_metrics.min_clearance[0] >= 0.0
    # ...even though the colliding straight-ahead candidate makes more progress
    assert progress(straight) > progress(chosen)


def test_safety_gate_emergency_fallback_when_every_candidate_collides():
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=1.0)
    # a large obstacle sitting on the agent: every candidate penetrates immediately
    obstacle = Obstacle(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=5.0)
    scene = SceneState(agent=agent, obstacles=(obstacle,), dt=0.1, horizon=0.3)

    planner = ActionPlanner()
    first = planner.plan(scene)
    diag = planner.last_diagnostics
    assert diag is not None
    assert diag.used_emergency is True
    assert diag.n_feasible == 0
    assert bool(diag.infeasible.all()) is True
    # the fallback never returns None and prefers a moving escape over standing still
    assert isinstance(first, Action)
    assert first.is_stop is False

    # deterministic: the same scene always yields the same action
    assert ActionPlanner().plan(scene) == first


def test_emergency_fallback_orders_by_ttc_then_clearance_then_index():
    idx = ActionPlanner._emergency_index

    # primary: the largest minimum TTC wins
    assert idx(np.array([1.0, 2.0]), np.array([0.0, 0.0])) == 1
    # secondary: on a TTC tie, the largest minimum clearance wins
    assert idx(np.array([2.0, 2.0]), np.array([0.5, 3.0])) == 1
    # tertiary: on a full tie, the lowest candidate index wins
    assert idx(np.array([2.0, 2.0, 2.0]), np.array([1.0, 1.0, 1.0])) == 0
    # an infinite TTC is the maximum and wins on the primary key
    assert idx(np.array([1.0, np.inf]), np.array([100.0, 0.0])) == 1


def test_safety_gate_coexists_with_fes_dwell_and_adaptive_horizon():
    from cpu.decision import SafetyGate, TrajectoryCost

    cost = TrajectoryCost()
    gate = SafetyGate(collision_threshold=0.0, near_miss_threshold=9.0, critical_weight=4.0)
    # the agent is already closing on the obstacle: ttc_min = 9 / 10 = 0.9s
    agent = AgentState(position=(0.0, 0.0), velocity=(5.0, 0.0), radius=1.0, max_speed=10.0)
    obstacle = Obstacle(position=(11.0, 0.0), velocity=(-5.0, 0.0), radius=1.0)
    scene = SceneState(
        agent=agent, obstacles=(obstacle,), goal=np.array([40.0, 0.0]), dt=0.1, horizon=0.3
    )
    planner = ActionPlanner(
        safety_gate=gate, horizon=0.3, danger_horizon=0.6, danger_threshold=1.0
    )

    # adaptive horizon still engages alongside the gate
    assert planner._estimate_min_ttc(scene) == pytest.approx(0.9)
    assert planner.horizon_for(scene) == pytest.approx(0.6)

    # near-collision layer is active (some penalised, some clean)...
    n_steps = max(1, int(round(0.6 / scene.dt)))
    metrics = cost.safety_metrics_batch(
        _batch_for(scene, planner.candidate_actions(scene), n_steps)
    )
    penalties = gate.critical_penalty(metrics)
    assert penalties.min() == pytest.approx(0.0)
    assert penalties.max() > 0.0
    # ...while FES/trap and dwell remain part of the ranking cost
    assert planner.cost.w_trap > 0.0
    assert planner.cost.w_dwell > 0.0

    chosen = planner.plan(scene)
    chosen_metrics = cost.safety_metrics_batch(_batch_for(scene, [chosen], n_steps))
    assert chosen_metrics.min_clearance[0] >= gate.collision_threshold
    assert planner.last_diagnostics.n_feasible >= 1


def test_decision_version_and_stable_build_id():
    from cpu.decision import BUILD_ID, VERSION, CpuDecisionLayer
    from cpu.decision import build as build_mod

    assert VERSION == "v0.6.0-PredictiveCorridorSelector"
    assert BUILD_ID.startswith("cpudec-") and len(BUILD_ID) == len("cpudec-") + 12
    # the id is a real source-content fingerprint, not a version hash
    assert BUILD_ID == build_mod.fingerprint()

    desc = CpuDecisionLayer().describe()
    assert desc["version"] == VERSION
    assert desc["build_id"] == BUILD_ID
    assert desc["safety_gate"]["collision_threshold"] == 0.0
    assert desc["critics"]["risk"] > desc["critics"]["fes"]


def test_weighted_risk_prioritizes_closing_obstacles():
    risk = RiskEvaluator()
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    static = Obstacle(position=(5.0, 0.0), velocity=(0.0, 0.0), radius=0.5)
    closing = Obstacle(position=(5.0, 0.0), velocity=(-5.0, 0.0), radius=0.5)

    pred = LinearPredictor()
    scene_static = SceneState(agent=agent, obstacles=(static,), dt=0.1, horizon=1.0)
    scene_closing = SceneState(agent=agent, obstacles=(closing,), dt=0.1, horizon=1.0)

    r_static = risk.risk_trajectory(pred.predict(scene_static, Action.stop(), 1))
    r_closing = risk.risk_trajectory(pred.predict(scene_closing, Action.stop(), 1))
    assert r_closing > r_static


# --------------------------------------------------------------------------
# cost (v0.3-1 multi-objective)
# --------------------------------------------------------------------------


def test_cost_penalizes_collision_and_rewards_progress():
    from cpu.decision.cost import TrajectoryCost

    cost = TrajectoryCost()
    pred = LinearPredictor()
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    goal = np.array([10.0, 0.0])

    safe_scene = SceneState(
        agent=agent, obstacles=(Obstacle((20.0, 0.0), (0.0, 0.0), 0.5),),
        goal=goal, dt=0.1, horizon=1.0,
    )
    safe = cost.evaluate(pred.predict(safe_scene, Action.stop(), safe_scene.n_steps),
                         goal=goal, start_position=agent.position)

    danger_scene = SceneState(
        agent=agent, obstacles=(Obstacle((3.0, 0.0), (0.0, 0.0), 0.5),),
        goal=goal, dt=0.1, horizon=1.0,
    )
    danger = cost.evaluate(pred.predict(danger_scene, Action(10.0, 0.0), danger_scene.n_steps),
                           goal=goal, start_position=agent.position)

    assert danger > safe
    # collision weight (10.0) dominates once the trajectory penetrates
    assert danger >= 10.0


def test_cost_rewards_progress_toward_goal():
    from cpu.decision.cost import TrajectoryCost

    cost = TrajectoryCost()
    pred = LinearPredictor()
    agent = AgentState(position=(0.0, 0.0), velocity=(0.0, 0.0), radius=1.0, max_speed=10.0)
    goal = np.array([10.0, 0.0])
    scene = SceneState(agent=agent, obstacles=(), goal=goal, dt=0.1, horizon=1.0)

    toward = cost.evaluate(pred.predict(scene, Action(5.0, 0.0), scene.n_steps),
                           goal=goal, start_position=agent.position)
    away = cost.evaluate(pred.predict(scene, Action(5.0, math.pi), scene.n_steps),
                         goal=goal, start_position=agent.position)
    assert toward < away  # progress_cost (negative) rewards moving toward the goal


# --------------------------------------------------------------------------
# adapter
# --------------------------------------------------------------------------


def test_game_adapter_converts_a_snapshot_into_abstract_state():
    world = World(scenario_for_level("easy", seed=3, duration=25.0), reward="zero",
                  terminate_on_collision=False)
    for _ in range(120):
        world.step(0)

    scene = snapshot_to_scene(world.get_state())

    snap = world.get_state()
    assert scene.agent.position.tolist() == [snap.player.x, snap.player.y]
    assert scene.agent.max_speed == snap.player.speed
    assert len(scene.obstacles) == snap.bullet_count
    assert scene.goal is not None and scene.goal.tolist() == [snap.target.x, snap.target.y]
    assert scene.dt == snap.env.dt


# --------------------------------------------------------------------------
# core facade + end-to-end drive
# --------------------------------------------------------------------------


def test_core_decide_returns_an_action():
    scene = SceneState(agent=AgentState(position=(0, 0), velocity=(0, 0), radius=1.0))
    action = CpuDecisionLayer().decide(scene)
    assert isinstance(action, Action)
    assert action.speed >= 0.0
    assert "cpu-decision" == CpuDecisionLayer.name


def test_decision_core_never_imports_the_world():
    pkg = pathlib.Path(__file__).resolve().parents[1] / "decision"   # 测试与模块同层：core/cpu/tests -> core/cpu
    for path in pkg.glob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "bullet_sim.simulator" not in text, path.name
        # the core may *document* the adapter, but it must never import it
        assert "from cpu.adapters" not in text, path.name
        assert "import cpu.adapters" not in text, path.name


def test_decision_drives_the_world_end_to_end():
    from bullet_sim.action.base import action_to_codec_input
    from bullet_sim.entities.player import P_SPEED

    world = World(scenario_for_level("easy", seed=7, duration=25.0), reward="zero",
                  terminate_on_collision=False)
    for _ in range(60):
        world.step(0)

    layer = CpuDecisionLayer()
    for _ in range(10):
        scene = snapshot_to_scene(world.get_state())
        action = layer.decide(scene)
        game_action = to_game_action(action)
        codec_input = action_to_codec_input(game_action, world.codec, float(world.player[P_SPEED]))
        before = world.state.env.step_index
        world.step(codec_input)
        assert world.state.env.step_index == before + 1
