"""Manual control (mode A): keyboard -> Action -> Simulator, verified end to end.

These tests drive a **real pygame renderer** under the dummy SDL video driver,
post genuine ``KEYDOWN``/``KEYUP`` events and check that

* the key reaches the manual source as a device-neutral key name,
* the composed action is the expected 9-way direction with normalised speed,
* the **player actually moves in the world** (not just that an object changed),
* no keyboard code writes environment state directly.

They are the automated form of the project's "Test 1" checklist.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

pygame = pytest.importorskip("pygame")

from bullet_sim.action import ManualInputSource, keyboard_action_from_keys  # noqa: E402
from bullet_sim.action.base import action_to_codec_input  # noqa: E402
from bullet_sim.entities.player import P_SPEED, P_X, P_Y  # noqa: E402
from bullet_sim.render.pygame_view import PygameRenderer, key_name_for  # noqa: E402
from bullet_sim.scenarios.presets import scenario_for_level  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402


def _pair(seed: int = 1, level: str = "easy"):
    env = BulletHellEnv(
        scenario_for_level(level, seed=seed, duration=30.0),
        seed=seed,
        terminate_on_collision=False,
        # same codec the CLI uses: the unified Action can express magnitude
        codec="action",
    )
    source = ManualInputSource()
    renderer = PygameRenderer(mode="rgb_array", input_source=source, scale=0.4)
    env.reset(seed=seed)
    source.open()
    renderer.render(env.world)
    return env, source, renderer


def _press(renderer, env, key):
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=key))
    renderer.render(env.world)


def _release(renderer, env, key):
    pygame.event.post(pygame.event.Event(pygame.KEYUP, key=key))
    renderer.render(env.world)


def test_key_names_are_device_neutral():
    assert key_name_for(pygame.K_UP) == "up"
    assert key_name_for(pygame.K_w) == "w"
    assert key_name_for(pygame.K_a) == "a"
    assert key_name_for(pygame.K_s) == "s"
    assert key_name_for(pygame.K_d) == "d"
    assert key_name_for(pygame.K_SPACE) == "space"
    assert key_name_for(pygame.K_LSHIFT) == "shift"
    # pygame constants never leak into the input layer: no pygame *import* may
    # exist (docstrings are allowed to mention it, imports are not).
    import bullet_sim.action.manual as manual

    source_text = open(manual.__file__, encoding="utf-8").read()
    assert "import pygame" not in source_text
    assert "from pygame" not in source_text


def test_keyboard_moves_the_player_in_every_direction():
    """Test 1: start -> press a key -> the player really moves that way."""
    env, source, renderer = _pair()
    try:
        for key, expect in (
            (pygame.K_RIGHT, (+1, 0)),
            (pygame.K_LEFT, (-1, 0)),
            (pygame.K_UP, (0, +1)),
            (pygame.K_DOWN, (0, -1)),
        ):
            env.reset(seed=1)
            source.clear()
            start = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])])
            _press(renderer, env, key)
            assert source.current_action().as_discrete() != 0
            for _ in range(20):
                action = source.poll(env.spec.dt)
                raw = action_to_codec_input(
                    action, env.world.codec, float(env.world.player[P_SPEED])
                )
                env.step(raw)
            moved = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])]) - start
            if expect[0]:
                assert np.sign(moved[0]) == expect[0] and moved[0] != 0.0
            if expect[1]:
                assert np.sign(moved[1]) == expect[1] and moved[1] != 0.0
            _release(renderer, env, key)
    finally:
        renderer.close()
        env.close()


def test_wasd_and_arrow_keys_are_equivalent():
    assert keyboard_action_from_keys(["w"]) == keyboard_action_from_keys(["up"])
    assert keyboard_action_from_keys(["a"]) == keyboard_action_from_keys(["left"])
    assert keyboard_action_from_keys(["s"]) == keyboard_action_from_keys(["down"])
    assert keyboard_action_from_keys(["d"]) == keyboard_action_from_keys(["right"])


def test_diagonal_is_not_faster_than_axis():
    env, source, renderer = _pair()
    try:
        speeds = {}
        for keys, label in (([pygame.K_UP], "axis"), ([pygame.K_UP, pygame.K_RIGHT], "diag")):
            env.reset(seed=1)
            source.clear()
            start = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])])
            for k in keys:
                _press(renderer, env, k)
            for _ in range(20):
                action = source.poll(env.spec.dt)
                env.step(
                    action_to_codec_input(
                        action, env.world.codec, float(env.world.player[P_SPEED])
                    )
                )
            moved = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])]) - start
            speeds[label] = float(np.hypot(*moved))
            for k in keys:
                _release(renderer, env, k)
        assert speeds["diag"] == pytest.approx(speeds["axis"], rel=1e-6)
    finally:
        renderer.close()
        env.close()


def test_focus_modifier_slows_the_player():
    env, source, renderer = _pair()
    try:
        travelled = {}
        for keys, label in (([pygame.K_RIGHT], "normal"), ([pygame.K_RIGHT, pygame.K_LSHIFT], "focus")):
            env.reset(seed=1)
            source.clear()
            start = float(env.world.player[P_X])
            for k in keys:
                _press(renderer, env, k)
            for _ in range(20):
                env.step(
                    action_to_codec_input(
                        source.poll(env.spec.dt), env.world.codec,
                        float(env.world.player[P_SPEED]),
                    )
                )
            travelled[label] = float(env.world.player[P_X]) - start
            for k in keys:
                _release(renderer, env, k)
        assert 0 < travelled["focus"] < travelled["normal"]
    finally:
        renderer.close()
        env.close()


def test_releasing_keys_stops_the_player():
    env, source, renderer = _pair()
    try:
        env.reset(seed=1)
        source.clear()
        _press(renderer, env, pygame.K_RIGHT)
        for _ in range(10):
            env.step(action_to_codec_input(source.poll(env.spec.dt), env.world.codec,
                                           float(env.world.player[P_SPEED])))
        _release(renderer, env, pygame.K_RIGHT)
        assert source.current_action().is_zero
    finally:
        renderer.close()
        env.close()


def test_space_is_an_explicit_stop():
    assert keyboard_action_from_keys(["space"]).is_zero
    # SPACE overrides a held direction instead of being ignored by combine()
    assert keyboard_action_from_keys(["space", "up"]).is_zero
    assert keyboard_action_from_keys(["up"]).as_discrete() != 0
    assert keyboard_action_from_keys(["0", "right"]).is_zero


def test_escape_and_q_request_shutdown():
    env, source, renderer = _pair()
    try:
        assert renderer.closed is False
        _press(renderer, env, pygame.K_ESCAPE)
        assert renderer.closed is True
    finally:
        renderer.close()
        env.close()

    env, source, renderer = _pair()
    try:
        _press(renderer, env, pygame.K_q)
        assert renderer.closed is True
    finally:
        renderer.close()
        env.close()


def test_h_toggles_the_hud():
    env, source, renderer = _pair()
    try:
        before = renderer.show_hud
        _press(renderer, env, pygame.K_h)
        assert renderer.show_hud is not before
    finally:
        renderer.close()
        env.close()


def test_manual_input_never_touches_the_world_directly():
    """Architectural check: the input layer only produces Actions."""
    import bullet_sim.action.base as base
    import bullet_sim.action.manual as manual
    import bullet_sim.action.types as types

    for module in (base, manual, types):
        text = open(module.__file__, encoding="utf-8").read()
        # no real import of the kernel, only prose in docstrings
        assert "from bullet_sim.simulator" not in text, module.__name__
        assert "import bullet_sim.simulator" not in text, module.__name__


# --------------------------------------------------------------------------
# manual control through a SwitchableSource (the CLI's default wiring)
# --------------------------------------------------------------------------


def test_movement_keys_work_through_a_switchable_source():
    """Regression: the UI delivers key events to the *active* source.

    ``play`` now wraps manual input in a SwitchableSource so TAB can hand over
    to an autonomous controller.  The renderer must still be able to press and
    release keys, and the player must still move.
    """
    from bullet_sim.action import SwitchableSource
    from bullet_sim.action.base import ControllerSource
    from bullet_sim.ai import make_controller

    env = BulletHellEnv(
        scenario_for_level("medium", seed=21, duration=30.0),
        seed=21, terminate_on_collision=False, codec="action",
    )
    manual = ManualInputSource()
    auto = ControllerSource(make_controller("threat"))
    source = SwitchableSource({"manual": manual, "auto": auto}, manual_names=["manual"])
    renderer = PygameRenderer(mode="rgb_array", input_source=source, scale=0.4)
    try:
        env.reset(seed=21)
        source.open()
        renderer.render(env.world)

        start = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])])
        _press(renderer, env, pygame.K_RIGHT)
        assert sorted(source.pressed_keys) == ["right"]

        term = trunc = False
        for _ in range(120):
            action = source.poll_observation(env._last_obs, env._last_info)
            _obs, _r, term, trunc, _info = env.step(action)
            renderer.render(env.world)
            assert renderer.closed is False
        moved = np.array([float(env.world.player[P_X]), float(env.world.player[P_Y])]) - start
        assert moved[0] > 10.0, "manual keys must actually move the player"
        assert not term and not trunc
    finally:
        renderer.close()
        env.close()


def test_keys_pressed_while_autonomous_do_not_steer_after_switching_back():
    from bullet_sim.action import SwitchableSource
    from bullet_sim.action.base import ControllerSource
    from bullet_sim.ai import make_controller

    env = BulletHellEnv(
        scenario_for_level("easy", seed=3, duration=30.0),
        seed=3, terminate_on_collision=False, codec="action",
    )
    manual = ManualInputSource()
    auto = ControllerSource(make_controller("threat"))
    source = SwitchableSource({"manual": manual, "auto": auto}, manual_names=["manual"])
    renderer = PygameRenderer(mode="rgb_array", input_source=source, scale=0.4)
    try:
        env.reset(seed=3)
        source.open()
        renderer.render(env.world)
        source.switch_to("auto")
        _press(renderer, env, pygame.K_RIGHT)      # ignored while AUTO is active
        assert manual.pressed_keys == frozenset()
        source.switch_to("manual")
        assert source.pressed_keys == frozenset(), "no queued key may take over"
    finally:
        renderer.close()
        env.close()
