"""Hardware input interface: placeholders, TBD discipline, board -> Action loop.

These tests assert two things:

1. the pipeline ``Adapter -> Action -> Simulator`` is fully wired and works
   today (via the loopback harness), and
2. **nothing guesses the board protocol** - every unknown stays ``None`` and
   requests for it fail loudly instead of returning an invented value.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from bullet_sim.action import Action, ScriptedSource
from bullet_sim.hardware_interface import (
    HARDWARE_INPUT_INTERFACE_TBD,
    ActionFrameFormat,
    CANDIDATE_TRANSPORTS,
    HardwareInputAdapter,
    HardwareInputSource,
    HardwareInterfaceNotConfigured,
    LoopbackHardwareInputAdapter,
    PlannedTransport,
    PlaceholderHardwareInputAdapter,
    RecordedHardwareInputAdapter,
    make_hardware_input_adapter,
    make_hardware_input_source,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


# --------------------------------------------------------------------------
# nothing is guessed
# --------------------------------------------------------------------------


def test_planned_transport_is_entirely_unknown():
    t = PlannedTransport()
    assert t.kind is None
    assert t.baud_rate is None
    assert t.byte_order is None
    assert t.packet_bytes is None
    assert t.register_base is None
    assert t.payload_format is None
    assert t.endpoint is None
    assert t.handshake is None
    assert t.timeout_ms is None
    assert t.configured is False
    assert set(t.missing()) >= {
        "kind", "baud_rate", "byte_order", "packet_bytes", "register_base",
        "payload_format", "endpoint", "handshake", "timeout_ms",
    }


def test_describe_marks_everything_as_tbd_and_is_json_serialisable():
    payload = PlannedTransport().describe()
    assert payload["marker"] == HARDWARE_INPUT_INTERFACE_TBD
    assert payload["configured"] is False
    json.dumps(payload)  # must survive a round trip into a report


def test_require_raises_for_unknown_fields_only():
    t = PlannedTransport()
    with pytest.raises(HardwareInterfaceNotConfigured) as exc:
        t.require("baud_rate")
    assert HARDWARE_INPUT_INTERFACE_TBD in str(exc.value)
    t.kind = "uart"
    t.require("kind")  # now decided -> no error
    with pytest.raises(HardwareInterfaceNotConfigured):
        t.require("kind", "register_base")


def test_candidate_transports_are_options_not_decisions():
    assert "uart" in CANDIDATE_TRANSPORTS and "mmio" in CANDIDATE_TRANSPORTS
    # listing options must not select one
    assert PlannedTransport().kind is None


def test_action_frame_format_is_unknown_too():
    fmt = ActionFrameFormat()
    assert fmt.fields is None and fmt.semantics is None and fmt.bits_per_field is None
    assert fmt.describe()["marker"] == HARDWARE_INPUT_INTERFACE_TBD


def test_placeholder_adapter_refuses_instead_of_inventing_actions():
    adapter = PlaceholderHardwareInputAdapter()
    assert adapter.transport == HARDWARE_INPUT_INTERFACE_TBD
    with pytest.raises(HardwareInterfaceNotConfigured):
        adapter.open()
    with pytest.raises(HardwareInterfaceNotConfigured):
        adapter.read_payload()
    with pytest.raises(HardwareInterfaceNotConfigured):
        adapter.decode_payload(b"\x00\x01")
    assert "not configured" in adapter.describe()["status"]


def test_placeholder_source_fails_loudly_through_the_env_loop():
    env = BulletHellEnv(scenario_for_level("easy", seed=1, duration=5.0))
    source = make_hardware_input_source(adapter="placeholder")
    with pytest.raises(HardwareInterfaceNotConfigured):
        env.run(input_source=source, steps=10, seed=1)
    env.close()


# --------------------------------------------------------------------------
# the pipeline works with a harness
# --------------------------------------------------------------------------


def test_loopback_adapter_is_a_test_harness_not_a_protocol():
    adapter = LoopbackHardwareInputAdapter()
    assert "harness" in adapter.transport
    assert adapter.planned.payload_format == "loopback-harness"
    # it deliberately does NOT select any real transport
    assert adapter.planned.kind is None
    assert adapter.planned.configured is False


def test_loopback_adapter_drives_the_simulator_end_to_end():
    spec = scenario_for_level("easy", seed=3, duration=8.0)
    env = BulletHellEnv(spec, seed=3, terminate_on_collision=False, codec="action")

    adapter = LoopbackHardwareInputAdapter()
    adapter.open()
    x0 = float(env.world.player[0])
    y0 = float(env.world.player[1])

    source = HardwareInputSource(adapter)
    source.open()
    for action in (Action.from_discrete("right"),) * 20 + (Action.from_discrete("up"),) * 20:
        adapter.push(action)
    env.reset(seed=3)
    for _ in range(40):
        act = source.poll(spec.dt)
        if act is not None:
            env.step(act)
    assert float(env.world.player[0]) > x0
    assert float(env.world.player[1]) > y0
    env.close()


def test_recorded_adapter_replays_a_board_session():
    actions = [Action.from_discrete(i % 9) for i in range(25)]
    adapter = RecordedHardwareInputAdapter(actions)
    adapter.open()
    got = []
    while True:
        payload = adapter.read_payload()
        if payload is None:
            break
        got.append(adapter.decode_payload(payload))
    assert got == actions


def test_hardware_source_is_an_actionsource():
    from bullet_sim.action import ActionSource

    source = HardwareInputSource(LoopbackHardwareInputAdapter())
    assert isinstance(source, ActionSource)
    assert source.name.startswith("board:")


def test_hardware_and_manual_sources_are_interchangeable():
    """The whole point: mode A and mode B converge on the same loop."""
    spec = scenario_for_level("easy", seed=11, duration=8.0)
    actions = [Action.from_discrete("right")] * 50

    adapter = LoopbackHardwareInputAdapter(list(actions))
    env_board = BulletHellEnv(spec, seed=11, terminate_on_collision=False,
                              codec="action")
    env_board.reset(seed=11)
    src = HardwareInputSource(adapter)
    src.open()
    for _ in range(50):
        a = src.poll(spec.dt)
        env_board.step(a if a is not None else Action.zero())
    board_hash = env_board.state_hash()
    env_board.close()

    env_manual = BulletHellEnv(spec, seed=11, terminate_on_collision=False, codec="action")
    env_manual.reset(seed=11)
    for _ in range(50):
        env_manual.step(Action.from_discrete("right"))
    assert env_manual.state_hash() == board_hash
    env_manual.close()


def test_adapter_factory_and_errors():
    assert isinstance(make_hardware_input_adapter("loopback"), LoopbackHardwareInputAdapter)
    assert isinstance(make_hardware_input_adapter("tbd"), PlaceholderHardwareInputAdapter)
    with pytest.raises(ValueError):
        make_hardware_input_adapter("carrier-pigeon")


def test_describe_reports_the_tbd_contract():
    adapter = PlaceholderHardwareInputAdapter()
    info = adapter.describe()
    assert info["marker"] == HARDWARE_INPUT_INTERFACE_TBD
    assert info["planned_transport"]["configured"] is False
    assert info["action_frame_format"]["marker"] == HARDWARE_INPUT_INTERFACE_TBD


def test_state_link_protocol_is_reachable_from_the_hardware_package():
    from bullet_sim.hardware_interface import state_link

    for name in ("encode_state", "decode_state", "LoopbackLink", "HardwareLink"):
        assert hasattr(state_link, name)
