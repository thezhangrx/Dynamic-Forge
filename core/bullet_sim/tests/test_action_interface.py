"""Unified Action interface and input sources (mode A: manual, mode C: program)."""

from __future__ import annotations

import numpy as np
import pytest

from bullet_sim.action import (
    Action,
    ControllerSource,
    ManualInputSource,
    NullActionSource,
    RandomActionSource,
    ScriptedSource,
    coerce_action,
    combine,
    keyboard_action_from_keys,
    source_from,
)
from bullet_sim.action.base import FULL_DEFLECTION_SECONDS, action_to_codec_input
from bullet_sim.core.actions import (
    AccelerationActionCodec,
    ActionContext,
    ActionObjectCodec,
    DiscreteActionCodec,
    VelocityActionCodec,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


# --------------------------------------------------------------------------
# Action type
# --------------------------------------------------------------------------


def test_action_normalises_direction_and_clamps_magnitude():
    a = Action((3.0, 4.0), 0.5)
    assert a.direction == pytest.approx((0.6, 0.8))
    assert a.magnitude == pytest.approx(0.5)
    assert Action((1.0, 0.0), 5.0).magnitude == 1.0  # clamped
    assert Action((0.0, 0.0), 1.0).is_zero


def test_action_rejects_non_finite_or_negative():
    with pytest.raises(ValueError):
        Action((float("nan"), 0.0))
    with pytest.raises(ValueError):
        Action((1.0, 0.0), -1.0)
    with pytest.raises(ValueError):
        Action((1.0, 2.0, 3.0))  # type: ignore[arg-type]


def test_action_constructors_and_conversions():
    up = Action.from_discrete("up")
    assert up.direction == pytest.approx((0.0, 1.0))
    assert Action.from_discrete(0).is_zero
    assert Action.from_angle(90.0).direction == pytest.approx((0.0, 1.0), abs=1e-12)
    assert Action.from_vector([3.0, 4.0]).direction == pytest.approx((0.6, 0.8))

    v = Action.from_discrete("right").to_velocity(200.0)
    assert np.allclose(v, [200.0, 0.0])
    assert Action.from_discrete("up_right").as_discrete() == 6
    assert Action.zero().as_discrete() == 0
    assert Action.from_angle(90.0).as_discrete() == 3


def test_action_roundtrips_through_dict():
    a = Action.from_discrete("down_left")
    assert Action.from_dict(a.to_dict()) == a


def test_combine_normalises_simultaneous_directions():
    diag = combine(Action.from_discrete("up"), Action.from_discrete("right"))
    assert float(np.hypot(*diag.direction)) == pytest.approx(1.0)
    assert diag.magnitude == pytest.approx(1.0)
    assert diag.as_discrete() == 6
    # opposing keys cancel
    assert combine(Action.from_discrete("left"), Action.from_discrete("right")).is_zero
    assert combine(Action.zero(), Action.zero()).is_zero


# --------------------------------------------------------------------------
# manual input (mode A)
# --------------------------------------------------------------------------


def test_manual_source_composes_pressed_keys():
    src = ManualInputSource()
    src.open()
    assert src.current_action().is_zero
    src.press("up")
    assert src.current_action().as_discrete() == 3
    src.press("right")
    a = src.current_action()
    assert a.as_discrete() == 6  # up_right, not sqrt(2) faster
    assert float(np.hypot(*a.direction)) == pytest.approx(1.0)
    src.release("right")
    assert src.current_action().as_discrete() == 3
    src.release("up")
    assert src.current_action().is_zero


def test_manual_source_w_a_s_d_and_arrows_agree():
    assert keyboard_action_from_keys(["w"]) == keyboard_action_from_keys(["up"])
    assert keyboard_action_from_keys(["a", "s"]) == keyboard_action_from_keys(["left", "down"])
    assert keyboard_action_from_keys(["space"]).is_zero


def test_manual_source_focus_modifier_scales_magnitude():
    focus = keyboard_action_from_keys(["shift", "up"])
    normal = keyboard_action_from_keys(["up"])
    assert focus.direction == pytest.approx(normal.direction)
    assert 0.0 < focus.magnitude < normal.magnitude


def test_manual_source_analog_axis_overrides_keys():
    src = ManualInputSource()
    src.open()
    src.press("up")
    src.set_axis(1.0, 0.0)
    a = src.current_action()
    assert a.direction == pytest.approx((1.0, 0.0))
    src.set_axis(0.0, 0.0)  # deadzone releases analog, keys take over again
    assert src.current_action().as_discrete() == 3


def test_manual_source_keeps_held_keys_across_open_but_clears_on_close():
    src = ManualInputSource()
    src.press("left")
    src.open()  # a held key is real device state - opening must not fake a release
    assert src.current_action().as_discrete() == 1
    src.close()
    assert src.current_action().is_zero
    assert src.pressed_keys == frozenset()


def test_manual_source_poll_reports_changes():
    src = ManualInputSource()
    src.open()
    src.clear()
    assert src.poll(0.01).is_zero
    src.press("down")
    polled = src.poll(0.01)
    assert polled.as_discrete() == 4
    assert src.stats.polls == 2 and src.stats.changes == 1


def test_manual_source_rebind():
    src = ManualInputSource()
    src.set_binding("j", "up_left")
    src.open()
    src.press("j")
    assert src.current_action().as_discrete() == 5
    assert "j" in src.describe()["bindings"]


def test_manual_input_module_does_not_touch_the_simulator():
    """The input layer must only translate device state into Actions."""
    import bullet_sim.action.manual as manual

    source_text = open(manual.__file__, encoding="utf-8").read()
    assert "bullet_sim.simulator" not in source_text
    assert "World" not in source_text.split("def ")[0]


# --------------------------------------------------------------------------
# other sources
# --------------------------------------------------------------------------


def test_scripted_and_random_sources_are_reproducible():
    actions = [0, 1, 2, 3]
    a = ScriptedSource(actions)
    b = ScriptedSource(actions)
    a.open(); b.open()
    assert [a.poll(0.01) for _ in range(6)] == [b.poll(0.01) for _ in range(6)]

    r1 = RandomActionSource(seed=3)
    r2 = RandomActionSource(seed=3)
    r1.open(); r2.open()
    assert [r1.poll(0.01) for _ in range(30)] == [r2.poll(0.01) for _ in range(30)]


def test_null_source_always_zero():
    src = NullActionSource()
    src.open()
    assert src.poll(0.01).is_zero


def test_source_from_config_forms():
    assert isinstance(source_from(None), NullActionSource)
    assert isinstance(source_from("keyboard"), ManualInputSource)
    assert isinstance(source_from("random", seed=1), RandomActionSource)
    assert isinstance(source_from({"kind": "scripted", "actions": [1, 2]}), ScriptedSource)
    assert isinstance(source_from(lambda obs, info: 3), ControllerSource)
    with pytest.raises(ValueError):
        source_from("telepathy")


def test_coerce_action_accepts_all_representations():
    assert coerce_action(3).as_discrete() == 3
    assert coerce_action("up").as_discrete() == 3
    assert coerce_action([1.0, 0.0]).direction == pytest.approx((1.0, 0.0))
    assert coerce_action(None).is_zero
    assert coerce_action(Action.from_discrete("down")).as_discrete() == 4
    with pytest.raises(ValueError):
        coerce_action([1, 2, 3, 4])


# --------------------------------------------------------------------------
# Action -> codec bridge
# --------------------------------------------------------------------------


def test_action_to_codec_input_maps_each_codec():
    a = Action.from_discrete("up_right")
    assert isinstance(action_to_codec_input(a, ActionObjectCodec(), 200.0), Action)
    assert action_to_codec_input(a, DiscreteActionCodec(), 200.0) == 6
    assert np.allclose(
        action_to_codec_input(a, VelocityActionCodec(), 200.0),
        a.to_velocity(200.0),
    )
    accel = action_to_codec_input(a, AccelerationActionCodec(), 200.0)
    assert np.linalg.norm(accel) == pytest.approx(200.0 / FULL_DEFLECTION_SECONDS)
    # low magnitude falls back to 'stay' for the discrete codec
    assert action_to_codec_input(Action((1.0, 0.0), 0.1), DiscreteActionCodec(), 200.0) == 0


def test_action_object_codec_drives_the_world():
    spec = scenario_for_level("easy", seed=5, duration=8.0)
    env = BulletHellEnv(spec, seed=5, codec="action", terminate_on_collision=False)
    obs, _ = env.reset(seed=5)
    x0 = float(env.world.player[0])
    env.step(Action.from_discrete("right"))
    assert float(env.world.player[0]) > x0
    env.close()


# --------------------------------------------------------------------------
# end-to-end: manual / board share one loop
# --------------------------------------------------------------------------


def test_all_input_modes_share_the_environment_loop():
    spec = scenario_for_level("easy", seed=7, duration=10.0)
    env = BulletHellEnv(spec, seed=7, terminate_on_collision=False)

    manual = ManualInputSource()
    manual.open()
    manual.press("right")
    manual.press("up")
    r_manual = env.run(input_source=manual, steps=80, seed=7)  # keys held across the episode
    assert r_manual["input_source"] == "keyboard"
    assert r_manual["last_action"]["direction"] == pytest.approx(
        Action.from_discrete("up_right").to_dict()["direction"]
    )

    r_script = env.run(input_source=ScriptedSource([1, 2, 3, 4], loop=True), steps=80, seed=7)
    assert r_script["steps"] == 80

    r_ctrl = env.run("random", steps=80, seed=7)
    assert r_ctrl["input_source"] == "controller"
    env.close()


def test_manual_and_equivalent_discrete_actions_produce_identical_state():
    """mode A and the discrete controller path must be bit-identical."""
    spec = scenario_for_level("easy", seed=9, duration=10.0)

    env = BulletHellEnv(spec, seed=9, terminate_on_collision=False)
    env.reset(seed=9)
    for _ in range(60):
        env.step(2)  # 'right' via the discrete codec
    hash_discrete = env.state_hash()
    env.close()

    manual = ManualInputSource()
    manual.open()
    manual.press("right")
    env2 = BulletHellEnv(spec, seed=9, terminate_on_collision=False, codec="action")
    env2.reset(seed=9)
    for _ in range(60):
        env2.step(manual.current_action())
    assert env2.state_hash() == hash_discrete
    env2.close()
