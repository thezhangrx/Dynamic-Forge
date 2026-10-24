"""Collision is a **penalty event**, not an episode/process ending event.

Covers the ten required checks:

1.  single collision does not close the window / stop the run
2.  many collisions in one episode keep it running
3.  ``collision_count`` accumulates correctly
4.  one continuous overlap is counted once (enter/hold/exit/re-enter)
5.  the reward deducts the configured penalty
6.  autonomous mode keeps receiving observations after a collision
7.  manual mode keeps running after a collision
8.  headless mode keeps running after a collision
9.  ``terminated`` does not flip to True on an ordinary collision
10. a genuinely configured end condition still ends the episode
"""

from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from bullet_sim.ai import make_controller  # noqa: E402
from bullet_sim.action import Action, ManualInputSource  # noqa: E402
from bullet_sim.collision.events import (  # noqa: E402
    COUNT_MODES,
    ContactTracker,
    validate_count_mode,
)
from bullet_sim.core.state import WorldSnapshot  # noqa: E402
from bullet_sim.entities.player import P_X, P_Y  # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402
from bullet_sim.simulator.rewards import SurvivalReward, make_reward  # noqa: E402
from bullet_sim.simulator.world import World  # noqa: E402

DT = 1.0 / 60.0


def _bare_spec(**kw) -> ScenarioSpec:
    """No emitters: tests inject exactly the bullets they want."""
    params = dict(
        name="bare",
        seed=0,
        duration=5.0,
        dt=DT,
        field_w=1000.0,
        field_h=1000.0,
        player_x=500.0,
        player_y=500.0,
        patterns=[],
        cull_margin=1e6,
    )
    params.update(kw)
    return ScenarioSpec(**params)


def _world(*, spec_kwargs: dict | None = None, **world_kw) -> World:
    """World over a bare scenario; ``spec_kwargs`` go to the ScenarioSpec."""
    world_kw.setdefault("reward", "survival")
    world_kw.setdefault("terminate_on_collision", False)
    return World(_bare_spec(**(spec_kwargs or {})), **world_kw)


def _hit(world: World, *, n: int = 1) -> np.ndarray:
    """Spawn ``n`` stationary bullets exactly on the player."""
    x = float(world.player[P_X])
    y = float(world.player[P_Y])
    return world.spawn_bullets(
        x=np.full(n, x), y=np.full(n, y), vx=0.0, vy=0.0, radius=3.0, ttl=1e6
    )


# --------------------------------------------------------------------------
# the state machine itself
# --------------------------------------------------------------------------


def test_contact_tracker_counts_one_event_per_new_bullet():
    t = ContactTracker("per_contact")
    assert t.update((), 0).events == 0
    assert t.update((7,), 1).events == 1
    assert t.collision_count == 1
    # holding the same bullet is not a new event
    for frame in range(2, 20):
        assert t.update((7,), frame).events == 0
    assert t.collision_count == 1
    assert t.overlap_frames == 19
    # a second bullet joins -> one more event
    assert t.update((7, 9), 20).events == 1
    assert t.collision_count == 2
    # it leaves, then comes back -> a new event, as required
    assert t.update((), 21).events == 0
    assert t.update((9,), 22).events == 1
    assert t.collision_count == 3
    assert t.summary(frame=22, dt=DT)["seconds_since_event"] == 0.0


def test_contact_tracker_per_step_counts_once_per_contact_episode():
    t = ContactTracker("per_step")
    assert t.update((1, 2, 3), 0).events == 1
    for frame in range(1, 5):
        assert t.update((1, 2, 3), frame).events == 0
    # a new bullet while still in contact does NOT open a new episode
    assert t.update((1, 2, 3, 4), 5).events == 0
    assert t.update((), 6).events == 0
    assert t.update((5,), 7).events == 1
    assert t.collision_count == 2


def test_contact_tracker_per_frame_is_the_legacy_behaviour():
    t = ContactTracker("per_frame")
    for frame in range(6):
        t.update((1,), frame)
    assert t.collision_count == 6


def test_count_mode_validation():
    assert validate_count_mode("PER_CONTACT") == "per_contact"
    with pytest.raises(ValueError):
        validate_count_mode("whenever")
    assert set(COUNT_MODES) == {"per_contact", "per_step", "per_frame"}


# --------------------------------------------------------------------------
# 3 + 4: counting
# --------------------------------------------------------------------------


def test_collision_count_accumulates_over_separate_contacts():
    world = _world()
    counts = []
    for expected in (1, 2, 3):
        slots = _hit(world)
        world.step(0)                      # enter
        counts.append(world.collision_count)
        for _ in range(5):                 # hold: must not increase
            world.step(0)
            counts.append(world.collision_count)
        world.state.bullets.release(slots)  # exit
        for _ in range(3):
            world.step(0)
            counts.append(world.collision_count)
        assert world.collision_count == expected
    assert counts == sorted(counts)
    assert world.collision_count == 3
    assert world.contacts.overlap_frames == 3 * 6
    assert world.contacts.entered_total == 3


def test_two_bullets_at_once_count_as_two_contacts():
    world = _world()
    _hit(world, n=2)
    result = world.step(0)
    assert result.info["collision_events"] == 2
    assert result.info["collision_count"] == 2
    assert world.collision_count == 2


def test_per_step_mode_groups_a_simultaneous_hit_into_one_event():
    world = _world(collision_count_mode="per_step")
    _hit(world, n=3)
    world.step(0)
    assert world.collision_count == 1
    assert world.contacts.entered_total == 3


# --------------------------------------------------------------------------
# 1, 2, 9: nothing ends just because of a collision
# --------------------------------------------------------------------------


def test_single_collision_does_not_terminate_or_stop_the_loop():
    world = _world()
    _hit(world)
    result = world.step(0)
    assert result.info["collision_event"] is True
    assert result.terminated is False
    assert result.truncated is False
    # ... and the world keeps producing steps
    for _ in range(50):
        r = world.step(0)
        assert r.terminated is False
    assert world.state.env.step_index == 51
    assert world.collision_count >= 1


def test_many_collisions_keep_the_episode_running():
    world = _world(spec_kwargs={"duration": 20.0})
    for _ in range(8):
        slots = _hit(world)
        world.step(0)
        world.state.bullets.release(slots)
        world.step(0)
    assert world.collision_count == 8
    assert world.state.env.step_index == 16
    metrics = world.episode_metrics()
    assert metrics["total_collision_count"] == 8
    assert metrics["collision_terminates_episode"] is False
    assert metrics["collision_rate_per_step"] == pytest.approx(0.5)


def test_collision_does_not_set_terminated_in_the_env_tuple():
    env = BulletHellEnv(_bare_spec(), seed=0, terminate_on_collision=False)
    try:
        env.reset(seed=0)
        _hit(env.world)
        _obs, _r, terminated, truncated, info = env.step(0)
        assert info["collision_event"] is True
        assert terminated is False and truncated is False
        for _ in range(30):
            _obs, _r, terminated, truncated, info = env.step(0)
            assert terminated is False
    finally:
        env.close()


def test_mode_b_restores_survival_semantics_when_asked():
    """Experiment Mode B: opt in, do not edit the kernel."""
    world = _world(terminate_on_collision=True)
    _hit(world)
    result = world.step(0)
    assert result.terminated is True
    assert world.episode_metrics()["collision_terminates_episode"] is True

    spec = _bare_spec(collision_terminates_episode=True)
    world2 = World(spec, reward="survival")
    assert world2.terminate_on_collision is True
    _hit(world2)
    assert world2.step(0).terminated is True


# --------------------------------------------------------------------------
# 5: penalty
# --------------------------------------------------------------------------


def test_reward_deducts_the_configured_penalty_once_per_event():
    world = _world(spec_kwargs={"collision_penalty": 2.5})
    slots = _hit(world)
    first = world.step(0)
    assert first.reward == pytest.approx(1.0 - 2.5)
    assert world.cumulative_reward == pytest.approx(-1.5)
    # holding contact must not be billed again
    for _ in range(10):
        r = world.step(0)
        assert r.reward == pytest.approx(1.0)
    world.state.bullets.release(slots)
    for _ in range(5):
        world.step(0)
    # reward for 16 clean steps + the one penalty
    assert world.cumulative_reward == pytest.approx(16 * 1.0 - 2.5)


def test_penalty_comes_from_configuration_not_from_code():
    default = World(_bare_spec(), reward="survival")
    assert default.reward.collision_penalty == pytest.approx(1.0)

    spec = _bare_spec(collision_penalty=7.0)
    configured = World(spec, reward="survival")
    assert configured.reward.collision_penalty == pytest.approx(7.0)
    _hit(configured)
    assert configured.step(0).reward == pytest.approx(1.0 - 7.0)

    # and it survives a JSON round trip
    from bullet_sim.scenarios.spec import ScenarioSpec as Spec

    again = Spec.from_dict(spec.to_dict())
    assert again.collision_penalty == pytest.approx(7.0)
    assert again.collision_count_mode == spec.collision_count_mode
    assert again.collision_terminates_episode is False


def test_reward_zero_is_unaffected_by_the_injected_penalty():
    world = World(_bare_spec(), reward="zero")
    _hit(world)
    assert world.step(0).reward == 0.0


def test_collision_only_reward_charges_per_event():
    reward = make_reward("collision_only", collision_penalty=3.0)
    world = _world(reward=reward)
    _hit(world)
    assert world.step(0).reward == pytest.approx(-3.0)
    assert world.step(0).reward == pytest.approx(0.0)


def test_survival_reward_falls_back_when_info_has_no_event_count():
    """Custom harnesses that do not populate ``info`` keep the old behaviour."""
    reward = SurvivalReward(collision_penalty=1.0)
    snap = WorldSnapshot.initial(field_w=100.0, field_h=100.0, dt=DT)

    class _Hit:
        hit = True

    assert reward(snap, snap, _Hit(), {}) == pytest.approx(0.0)
    assert reward(snap, snap, _Hit(), {"collision_events": 0}) == pytest.approx(1.0)


# --------------------------------------------------------------------------
# 6, 7, 8: every mode keeps running
# --------------------------------------------------------------------------


def test_headless_mode_keeps_running_through_many_collisions():
    spec = _bare_spec(duration=30.0, collision_penalty=1.0)
    env = BulletHellEnv(spec, seed=0)
    try:
        result = env.run(make_controller("idle"), steps=300, seed=0)
        # no scene emitters -> no collisions, but the loop must not end early
        assert result["steps"] == 300
        assert result["terminated"] is False
        assert result["collision_terminates_episode"] is False
    finally:
        env.close()


def _headless_steps_with_pinned_obstacle(env, steps: int):
    """Reset, pin a real obstacle on the player, then step with an idle controller.

    ``env.run`` resets internally, so the obstacle is injected after the reset
    and the loop is driven by hand - the property under test (a collision never
    ends the episode, and events are counted separately from overlap frames) is
    unchanged, but now deterministic.
    """
    from bullet_sim.ai import make_controller
    from bullet_sim.interface.controller import controller_from

    obs, info = env.reset(seed=21)
    env.world.state.bullets.spawn(
        env.world.player[0], env.world.player[1], 0.0, 0.0,
        radius=20.0, ttl=1e9, group_id=99,
    )
    ctrl = make_controller("idle")
    ctrl.reset(obs, info)
    rewards = []
    events = 0
    terminated = False
    for _ in range(int(steps)):
        action = ctrl.act(obs, info)
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(float(reward))
        events += int(info["collision_count"])
        if terminated or truncated:
            break
    return {
        "steps": len(rewards),
        "return": float(np.sum(rewards)),
        "collisions": int(env.world.collision_count),
        "collision_count": events,
        "terminated": bool(terminated),
    }


def test_headless_scene_with_real_collisions_runs_to_the_step_budget():
    from bullet_sim.scenarios.presets import scenario_for_level

    spec = scenario_for_level("medium", seed=21, duration=20.0)
    env = BulletHellEnv(spec, seed=21, terminate_on_collision=False, reward="survival")
    try:
        result = _headless_steps_with_pinned_obstacle(env, 400)
        assert result["steps"] == 400
        assert result["collisions"] >= 1, "the pinned obstacle must collide"
        assert result["terminated"] is False
        # reward = clean steps - penalties
        assert result["return"] == pytest.approx(400 - result["collisions"])
        assert env.world.episode_metrics()["overlap_frames"] > result["collisions"]
    finally:
        env.close()


def test_autonomous_controller_keeps_receiving_observations_after_a_collision():
    # Deterministic collision: a static obstacle is injected on top of the
    # player, then the controller must keep getting observations.
    spec = _bare_spec(duration=6.0)
    env = BulletHellEnv(spec, seed=21, terminate_on_collision=False, codec="action")
    try:
        obs, info = env.reset(seed=21)
        ctrl = make_controller("threat")
        ctrl.reset(obs, info)
        env.world.state.bullets.spawn(
            env.world.player[0], env.world.player[1], 0.0, 0.0,
            radius=20.0, ttl=10.0, group_id=99,
        )
        seen_after = 0
        collided = False
        for _ in range(120):
            action = ctrl.act(obs, info)
            obs, reward, terminated, truncated, info = env.step(action)
            assert not terminated, "a collision must not stop the controller"
            if info["collision_event"]:
                collided = True
            elif collided:
                seen_after += 1
                assert "player" in obs and "bullets" in obs
        assert collided, "the injected obstacle must register a collision"
        assert seen_after > 0
        assert info["collision_count"] >= 1
    finally:
        env.close()


def test_manual_mode_keeps_running_after_a_collision():
    spec = _bare_spec(duration=20.0)
    env = BulletHellEnv(spec, seed=0, terminate_on_collision=False, codec="action")
    try:
        env.reset(seed=0)
        source = ManualInputSource()
        source.open()
        source.press("right")
        x0 = float(env.world.player[P_X])
        hits = 0
        for step in range(120):
            if step in (10, 40, 70):
                _hit(env.world)
            action = source.poll(DT)
            _obs, _r, terminated, truncated, info = env.step(action)
            assert not terminated
            if info["collision_event"]:
                hits += 1
        assert hits == 3
        assert float(env.world.player[P_X]) > x0, "manual control must keep working"
        assert env.world.collision_count == 3
    finally:
        env.close()


# --------------------------------------------------------------------------
# 10: real end conditions still work
# --------------------------------------------------------------------------


def test_time_limit_still_ends_the_episode_as_truncated():
    world = _world(spec_kwargs={"duration": 0.5})  # 30 steps at 60 Hz
    for _ in range(40):
        result = world.step(0)
        if result.truncated:
            break
    assert result.truncated is True
    assert result.terminated is False
    assert world.state.env.step_index == 30


def test_explicit_termination_request_ends_the_episode():
    world = _world()
    assert world.step(0).terminated is False
    world.request_termination()
    assert world.step(0).terminated is True
    assert world.episode_metrics()["steps"] == 2


def test_env_terminate_helper():
    env = BulletHellEnv(_bare_spec(), seed=0)
    try:
        env.reset(seed=0)
        env.terminate()
        _obs, _r, terminated, _tr, _info = env.step(0)
        assert terminated is True
    finally:
        env.close()


def test_success_termination_is_configurable_and_off_by_default():
    world = _world()  # target defaults to the field centre, player is there too
    assert world.step(0).terminated is False, "success end must be opt-in"

    spec = _bare_spec(success_terminates_episode=True)
    world2 = World(spec, reward="survival")
    assert world2.step(0).terminated is True


def test_env_run_reports_the_episode_metrics():
    from bullet_sim.scenarios.presets import scenario_for_level

    spec = scenario_for_level("medium", seed=21, duration=20.0)
    env = BulletHellEnv(spec, seed=21, terminate_on_collision=False)
    try:
        result = env.run(make_controller("idle"), steps=200, seed=21)
        for key in (
            "collision_count",
            "collision_events",
            "collision_rate_per_step",
            "collision_rate_per_second",
            "survival_time",
            "episode_metrics",
        ):
            assert key in result
        metrics = result["episode_metrics"]
        for key in (
            "total_collision_count",
            "collision_count_mode",
            "collision_rate_per_step",
            "survival_time",
            "cumulative_reward",
            "overlap_frames",
        ):
            assert key in metrics
        assert metrics["survival_time"] == pytest.approx(200 * spec.dt)
    finally:
        env.close()


# --------------------------------------------------------------------------
# visualisation must survive collisions
# --------------------------------------------------------------------------


def test_window_stays_open_and_reports_collisions():
    pygame = pytest.importorskip("pygame")
    from bullet_sim.render.overlays import hud_lines, is_collision_flash
    from bullet_sim.render.pygame_view import PygameRenderer

    spec = _bare_spec(duration=20.0)
    env = BulletHellEnv(spec, seed=0, terminate_on_collision=False, codec="action")
    renderer = PygameRenderer(mode="rgb_array", scale=0.4)
    try:
        env.reset(seed=0)
        flashed = False
        for step in range(60):
            if step == 5:
                _hit(env.world)
            _obs, _r, terminated, truncated, _info = env.step(Action.zero())
            renderer.render(env.world)
            assert renderer.closed is False, "a collision must not close the window"
            assert terminated is False
            if is_collision_flash(env.world):
                flashed = True
        assert flashed, "the UI must visibly flag a fresh collision"
        text = "\n".join(hud_lines(env.world, fps=60.0))
        assert "collisions: 1" in text
        assert "reward" in text
        assert "GAME OVER" not in text.upper()
    finally:
        renderer.close()
        env.close()


def test_escape_still_closes_the_window_on_request():
    pygame = pytest.importorskip("pygame")
    from bullet_sim.render.pygame_view import PygameRenderer

    spec = _bare_spec()
    env = BulletHellEnv(spec, seed=0, terminate_on_collision=False, codec="action")
    renderer = PygameRenderer(mode="rgb_array", scale=0.4)
    try:
        env.reset(seed=0)
        _hit(env.world)
        env.step(Action.zero())
        renderer.render(env.world)
        assert renderer.closed is False
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        renderer.render(env.world)
        assert renderer.closed is True, "the user can always quit explicitly"
    finally:
        renderer.close()
        env.close()


# --------------------------------------------------------------------------
# replay / dataset carry the collision policy
# --------------------------------------------------------------------------


def test_recorded_episode_stores_events_separately_from_overlap_frames():
    from bullet_sim.dataset.recorder import TrajectoryRecorder
    from bullet_sim.scenarios.presets import scenario_for_level

    spec = scenario_for_level("medium", seed=21, duration=20.0)
    rec = TrajectoryRecorder()
    env = BulletHellEnv(spec, seed=21, record=rec, terminate_on_collision=False)
    _headless_steps_with_pinned_obstacle(env, 300)
    episode = rec.finish()
    env.close()

    events = int(episode.arrays["collision_events"].sum())
    frames = int(episode.arrays["collisions"].sum())
    assert events >= 1
    assert frames >= events
    policy = episode.meta["env_policy"]
    assert policy["collision_terminates_episode"] is False
    assert policy["collision_count_mode"] == "per_contact"
    assert "collision_penalty" in policy


def test_episode_replays_exactly_with_the_recorded_collision_policy():
    from bullet_sim.dataset.recorder import TrajectoryRecorder
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.replay import verify_episode

    spec = scenario_for_level("medium", seed=21, duration=20.0)
    rec = TrajectoryRecorder(store_hashes=True)
    env = BulletHellEnv(spec, seed=21, record=rec, terminate_on_collision=False)
    env.run(make_controller("idle"), steps=300, seed=21)
    episode = rec.finish()
    env.close()

    comparison = verify_episode(episode)
    assert comparison.ok, comparison.reason
    assert comparison.expected_steps == 300
