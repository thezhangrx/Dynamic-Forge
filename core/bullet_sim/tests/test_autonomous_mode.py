"""Autonomous mode (no human) and runtime mode switching.

Covers the project's "Test 2" (start -> do not touch the keyboard -> the avatar
moves by itself) and "Test 3" (Manual <-> Autonomous) checklists.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from bullet_sim.action import (  # noqa: E402
    Action,
    ManualInputSource,
    SwitchableSource,
    keyboard_action_from_keys,
)
from bullet_sim.action.base import ControllerSource  # noqa: E402
from bullet_sim.ai import (  # noqa: E402
    BASELINE_CONTROLLERS,
    ControllerSpec,
    make_auto_source,
    make_controller,
    make_play_source,
)
from bullet_sim.ai.base import bullet_matrix, direction_to_action  # noqa: E402
from bullet_sim.ai.policy import (  # noqa: E402
    MODEL_NOT_AVAILABLE_YET,
    ModelNotAvailable,
    TrainedPolicyController,
    available_policies,
    load_policy,
    register_policy,
)
from bullet_sim.scenarios.presets import scenario_for_level  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402

import pytest  # noqa: E402  (kept for the fallback runner's shim)


def _env(level: str = "eas y".replace(" ", ""), seed: int = 3, **kw) -> BulletHellEnv:
    return BulletHellEnv(
        scenario_for_level(level, seed=seed, duration=25.0),
        seed=seed,
        terminate_on_collision=True,
        **kw,
    )


# --------------------------------------------------------------------------
# Test 2: nobody touches the keyboard
# --------------------------------------------------------------------------


def test_autonomous_controller_moves_the_player_without_input():
    env = _env(seed=5)
    try:
        obs, info = env.reset(seed=5)
        start = np.array([float(env.world.player[0]), float(env.world.player[1])])
        ctrl = make_controller("threat")
        ctrl.reset(obs, info)
        action = 0
        for _ in range(120):
            action = ctrl.act(obs, info)
            obs, _r, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break
        moved = np.array([float(env.world.player[0]), float(env.world.player[1])]) - start
        assert np.hypot(*moved) > 1.0, "an autonomous controller must actually move"
        assert action != 0 or moved.any()
    finally:
        env.close()


@pytest.mark.parametrize("name", sorted(BASELINE_CONTROLLERS))
def test_every_baseline_controller_runs_headless(name):
    env = _env(seed=7)
    try:
        kwargs = {"world": env.world} if name == "planner" else {}
        if name == "random":
            kwargs["seed"] = 7
        ctrl = make_controller(name, **kwargs)
        result = env.run(ctrl, steps=120, seed=7)
        assert result["steps"] > 0
    finally:
        env.close()



def test_predictive_baselines_beat_the_do_nothing_control():
    """A dodge controller that is worse than 'never move' would be a bug."""
    scores: dict[str, int] = {}
    for name in ("idle", "repulsion", "threat"):
        env = _env(level="medium", seed=21)
        try:
            result = env.run(make_controller(name), steps=600, seed=21)
            scores[name] = result["steps"]
        finally:
            env.close()
    assert scores["repulsion"] > scores["idle"]
    assert scores["threat"] > scores["idle"]


def test_controllers_are_deterministic_given_a_seed():
    def run(name: str, seed: int) -> list[str]:
        env = _env(seed=seed)
        try:
            ctrl = make_controller(name, seed=seed) if name == "random" else make_controller(name)
            env.reset(seed=seed)
            hashes = [env.state_hash()]
            for _ in range(80):
                env.step(ctrl.act(env._last_obs, env._last_info))
                hashes.append(env.state_hash())
            return hashes
        finally:
            env.close()

    for name in ("random", "repulsion", "threat"):
        assert run(name, 11) == run(name, 11)


def test_controller_self_description_is_available_for_the_hud():
    for name in ("idle", "random", "repulsion", "threat"):
        spec = make_controller(name).spec()
        assert isinstance(spec, ControllerSpec)
        assert spec.kind == "baseline"
        assert spec.access in ("observation", "world")
        assert spec.hud_line().startswith("ai        :")


def test_only_the_planner_needs_world_access():
    for name in ("idle", "random", "repulsion", "threat"):
        assert make_controller(name).spec().access == "observation"
    with pytest.raises(ValueError, match="world access"):
        make_controller("planner")  # must be explicit about privileged access


def test_planner_requires_the_world_and_works_with_it():
    env = _env(seed=2)
    try:
        ctrl = make_controller("planner", world=env.world, horizon=12)
        assert ctrl.spec().access == "world"
        result = env.run(ctrl, steps=120, seed=2)
        assert result["steps"] > 0
    finally:
        env.close()


# --------------------------------------------------------------------------
# observation-only helpers
# --------------------------------------------------------------------------


def test_bullet_matrix_and_direction_mapping():
    obs = {"bullets": np.zeros((3, 7), dtype=np.float32)}
    assert bullet_matrix(obs).shape == (3, 7)
    assert bullet_matrix({}).shape == (0, 7)
    assert direction_to_action(1.0, 0.0) == 2      # right
    assert direction_to_action(-1.0, 1.0) == 5     # up-left
    assert direction_to_action(0.0, 0.0) == 0      # stay
    assert direction_to_action(float("nan"), 0.0) == 0


def test_threat_scores_have_one_entry_per_action():
    env = _env(seed=4)
    try:
        obs, info = env.reset(seed=4)
        ctrl = make_controller("threat")
        obs, *_ = env.step(0)
        scores = ctrl.threat_scores(obs, info)
        assert scores.shape == (9,)
        action, _ = ctrl.choose(obs, info)
        assert 0 <= action <= 8
    finally:
        env.close()


# --------------------------------------------------------------------------
# trained models: honest absence
# --------------------------------------------------------------------------


def test_no_trained_model_is_available_and_it_says_so():
    assert MODEL_NOT_AVAILABLE_YET == "[MODEL_NOT_AVAILABLE_YET]"
    with pytest.raises(ModelNotAvailable, match=r"\[MODEL_NOT_AVAILABLE_YET\]"):
        load_policy(None)
    with pytest.raises(ModelNotAvailable, match=r"\[MODEL_NOT_AVAILABLE_YET\]"):
        load_policy("definitely-missing-policy.npz")


def test_vendor_model_formats_are_marked_tbd_not_guessed():
    from bullet_sim.ai.policy import MODEL_DEPLOYMENT_TBD

    with pytest.raises(ModelNotAvailable, match=r"\[MODEL_DEPLOYMENT_TBD\]"):
        load_policy("policy.onnx")


def test_a_registered_policy_can_drive_the_simulator(tmp_path):
    """The plug-in path a real model will use, exercised with a stand-in."""
    W = np.zeros((9, 4), dtype=np.float32)
    W[2, 0] = 1.0  # 'right' whenever the player x-feature is positive
    np.savez(tmp_path / "tiny.npz", W1=W, b1=np.zeros(9, dtype=np.float32))

    policy = load_policy(tmp_path / "tiny.npz")
    assert policy(np.array([1.0, 0.0, 0.0, 0.0])) == 2

    from bullet_sim.ai.policy import policy_registry

    register_policy("unit-test-policy", lambda _path=None: policy, kind="npz")
    try:
        assert "unit-test-policy" in available_policies()
        env = _env(seed=6)
        try:
            # obs_dim pins the input width the policy was trained for
            ctrl = TrainedPolicyController(policy, name="unit-test-policy", obs_dim=4)
            result = env.run(ctrl, steps=60, seed=6)
            assert result["steps"] > 0
        finally:
            env.close()
    finally:
        policy_registry().unregister("unit-test-policy")
    assert "unit-test-policy" not in available_policies()


# --------------------------------------------------------------------------
# Test 3: switching modes at runtime
# --------------------------------------------------------------------------


def test_switchable_source_toggles_and_clears_held_keys():
    manual = ManualInputSource()
    manual.open()
    manual.press("up")
    auto_source, _ = make_auto_source("threat")
    sw = SwitchableSource(
        {"manual": manual, "auto": auto_source}, manual_names=["manual"]
    )
    assert sw.mode == "manual"
    assert sw.poll(0.01) is not None

    assert sw.toggle() == "auto"
    # a key held while leaving manual must not keep steering
    assert manual.pressed_keys == frozenset()
    assert sw.toggle() == "manual"
    assert sw.switch_count == 2
    assert [(e.from_mode, e.to_mode) for e in sw.history] == [
        ("manual", "auto"),
        ("auto", "manual"),
    ]


def test_switching_does_not_disturb_the_simulation_state():
    """A switch changes who produces the Action - never the world."""
    env = _env(seed=8)
    try:
        manual = ManualInputSource()
        manual.open()
        auto_source, _ = make_auto_source("threat")
        sw = SwitchableSource({"manual": manual, "auto": auto_source},
                              manual_names=["manual"])
        obs, info = env.reset(seed=8)
        sw.open()
        before = env.state_hash()
        sw.switch_to("auto", step=env.world.state.env.step_index)
        assert env.state_hash() == before
        sw.switch_to("manual")
        assert env.state_hash() == before
        assert env.world.state.env.step_index == 0
    finally:
        env.close()


def test_env_run_drives_a_switchable_source_and_reports_modes():
    env = _env(seed=9)
    try:
        manual = ManualInputSource()
        auto_source, _ = make_auto_source("threat")
        sw = SwitchableSource({"manual": manual, "auto": auto_source},
                              manual_names=["manual"])
        r1 = env.run(input_source=sw, steps=40, seed=9)
        assert r1["input_mode"] == "manual"
        sw.switch_to("auto")
        r2 = env.run(input_source=sw, steps=40, seed=9)
        assert r2["input_mode"] == "auto"
        assert r2["mode_switches"] == 1
        assert r2["steps"] == 40
    finally:
        env.close()


def test_tab_key_in_the_renderer_switches_modes():
    pygame = pytest.importorskip("pygame")
    from bullet_sim.render.pygame_view import PygameRenderer

    env = _env(seed=1)
    manual = ManualInputSource()
    auto_source, ctrl = make_auto_source("threat")
    sw = SwitchableSource({"manual": manual, "auto": auto_source},
                          manual_names=["manual"])
    renderer = PygameRenderer(mode="rgb_array", input_source=sw, controller=ctrl,
                              scale=0.4)
    try:
        renderer.render(env.world)
        assert sw.mode == "manual"
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_TAB))
        renderer.render(env.world)
        assert sw.mode == "auto"
        assert "AUTO" in renderer.mode_banner
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_TAB))
        renderer.render(env.world)
        assert sw.mode == "manual"
    finally:
        renderer.close()
        env.close()


# --------------------------------------------------------------------------
# the factory that the CLI uses
# --------------------------------------------------------------------------


def test_play_source_factory_covers_every_documented_mode():
    env = _env(seed=1)
    try:
        for mode, expect in (
            ("manual", "switch:manual"),
            ("keyboard", "switch:manual"),
            ("auto", "switch:auto"),
            ("ai", "switch:auto"),
            ("random", "random"),
            ("null", "null"),
        ):
            play = make_play_source(mode, env=env, seed=1)
            assert play.name == expect, mode
        with pytest.raises(ModelNotAvailable):
            make_play_source("model", env=env)
    finally:
        env.close()


def test_manual_and_auto_produce_identical_trajectories_for_identical_actions():
    """The two modes differ only in who produces the Action, not in the physics."""
    spec = scenario_for_level("easy", seed=12, duration=10.0)
    actions = [keyboard_action_from_keys(["right"])] * 60

    env = BulletHellEnv(spec, seed=12, terminate_on_collision=False, codec="action")
    env.reset(seed=12)
    for a in actions:
        env.step(a)
    hash_manual = env.state_hash()
    env.close()

    class Fixed:
        def __init__(self, acts):
            self.acts = acts
            self.i = 0

        def reset(self, *a):
            self.i = 0

        def act(self, *a):
            value = self.acts[min(self.i, len(self.acts) - 1)]
            self.i += 1
            return value

    env = BulletHellEnv(spec, seed=12, terminate_on_collision=False, codec="action")
    env.run(ControllerSource(Fixed(actions)), steps=60, seed=12)
    assert env.state_hash() == hash_manual
    env.close()
