"""CPU / FPGA boundary: frames, the reference prediction block, the CPU decision.

These tests exercise the *whole* hardware loop in software:

    World -> state frame -> FPGA reference -> prediction frame -> CPU -> Action -> World

and pin down what is implemented versus what is still ``[HARDWARE_INTERFACE_TBD]``.
"""

from __future__ import annotations

import struct

import numpy as np
import pytest

from bullet_sim.core.errors import ProtocolError
from cpu import FrameCpuDecisionLayer, CpuFpgaController, CpuFpgaPipeline
from bullet_sim.fpga import (
    HARDWARE_INTERFACE_TBD,
    PREDICTION_MAGIC,
    PREDICTION_PROTOCOL_VERSION,
    SOFTWARE_REFERENCE_ONLY,
    FpgaConfig,
    FpgaPredictionReference,
    decode_prediction_frame,
    describe_prediction_protocol,
    encode_prediction_frame,
    prediction_frame_size,
)
from bullet_sim.hardware_interface.tbd import HARDWARE_INPUT_INTERFACE_TBD
from bullet_sim.interface.protocol import (
    decode_state,
    describe_protocol,
    encode_state,
    frame_size,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.world import World


def _world(level: str = "easy", seed: int = 3, steps: int = 120) -> World:
    world = World(
        scenario_for_level(level, seed=seed, duration=25.0),
        reward="zero",
        terminate_on_collision=False,
    )
    for _ in range(steps):
        world.step(0)
    return world


# --------------------------------------------------------------------------
# 1. Simulator -> FPGA: the state frame
# --------------------------------------------------------------------------


def test_simulator_gives_fpga_a_self_contained_state_frame():
    world = _world()
    snapshot = world.get_state()
    frame = encode_state(snapshot)
    assert len(frame) == frame_size(snapshot.bullet_count)
    decoded = decode_state(frame)
    assert decoded.bullet_count == snapshot.bullet_count
    # everything the FPGA needs to predict the future is inside the frame
    for attr in ("env", "player", "target"):
        assert hasattr(decoded, attr)
    assert set(
        ("x", "y", "vx", "vy", "ax", "ay", "angle", "angular_velocity",
         "radius", "ttl", "shape", "half_w", "half_h", "rotation")
    ) <= set(decoded.bullets.data)
    info = describe_protocol()
    assert info["version"] == 3
    assert info["readable_versions"] == [1, 2, 3]


# --------------------------------------------------------------------------
# 2. the prediction frame contract
# --------------------------------------------------------------------------


def test_prediction_frame_round_trip():
    grid = np.linspace(0, 1, 4 * 3 * 5, dtype=np.float32).reshape(4, 3, 5)
    frame = encode_prediction_frame(
        grid, source_step=11, dt=1 / 120, cell_w=16.0, cell_h=16.0
    )
    assert len(frame) == prediction_frame_size(4, 3, 5)
    back, header = decode_prediction_frame(frame)
    assert header.magic == PREDICTION_MAGIC
    assert header.version == PREDICTION_PROTOCOL_VERSION
    assert header.source_step == 11
    assert (header.horizon, header.rows, header.cols) == (4, 3, 5)
    assert np.allclose(back, grid)


def test_prediction_frame_is_validated():
    grid = np.zeros((2, 2, 2), dtype=np.float32)
    frame = encode_prediction_frame(grid, source_step=0, dt=0.01, cell_w=1.0, cell_h=1.0)
    bad = bytearray(frame)
    struct.pack_into("<I", bad, 0, 0xDEADBEEF)
    with pytest.raises(ProtocolError, match="magic"):
        decode_prediction_frame(bytes(bad))

    bad = bytearray(frame)
    struct.pack_into("<H", bad, 4, 99)
    with pytest.raises(ProtocolError, match="version"):
        decode_prediction_frame(bytes(bad))

    corrupt = bytearray(frame)
    corrupt[-8] ^= 0xFF
    with pytest.raises(ProtocolError, match="CRC"):
        decode_prediction_frame(bytes(corrupt))

    with pytest.raises(ProtocolError):
        decode_prediction_frame(frame[:-1])


def test_prediction_protocol_description_is_explicit_about_what_is_open():
    info = describe_prediction_protocol()
    assert info["header_bytes"] == 44
    assert info["transport"] == HARDWARE_INTERFACE_TBD
    assert "grid" in info["layout"]


# --------------------------------------------------------------------------
# 3. the FPGA reference block
# --------------------------------------------------------------------------


def test_fpga_reference_is_a_pure_function_of_its_input_bytes():
    world = _world(level="medium", steps=180)
    frame = encode_state(world.get_state())
    fpga = FpgaPredictionReference(FpgaConfig(horizon=12, rows=16, cols=20))
    a = fpga.predict(frame)
    b = fpga.predict(frame)
    assert a == b, "the golden model must be bit-reproducible for HDL verification"


def test_fpga_reference_output_shape_and_range():
    world = _world(level="medium", steps=200)
    cfg = FpgaConfig(horizon=10, rows=16, cols=20)
    fpga = FpgaPredictionReference(cfg)
    frame = fpga.predict(encode_state(world.get_state()))
    grid, header = decode_prediction_frame(frame)
    assert grid.shape == (cfg.horizon, cfg.rows, cfg.cols)
    assert 0.0 <= float(grid.min()) and float(grid.max()) <= 1.0
    assert header.rows == cfg.rows and header.cols == cfg.cols
    assert header.source_step == world.state.env.step_index


def test_fpga_reference_marks_danger_where_bullets_will_be():
    world = _world(level="easy", steps=60)
    fpga = FpgaPredictionReference(FpgaConfig(horizon=8, rows=24, cols=32))
    grid, header = decode_prediction_frame(fpga.predict(encode_state(world.get_state())))
    assert grid.max() > 0.0
    # danger is not uniform: the field must carry a gradient for the CPU to use
    assert float(grid.std()) > 0.0
    snapshot = world.get_state()
    cx = int(snapshot.player.x / header.cell_w)
    cy = int(snapshot.player.y / header.cell_h)
    assert 0 <= cy < header.rows and 0 <= cx < header.cols


def test_fpga_reference_describes_itself_as_a_software_reference():
    info = FpgaPredictionReference().describe()
    assert info["status"] == SOFTWARE_REFERENCE_ONLY
    assert info["transport"] == HARDWARE_INTERFACE_TBD
    assert info["resource_estimates"] == HARDWARE_INTERFACE_TBD
    assert "state frame" in info["input"]


# --------------------------------------------------------------------------
# 4. FPGA result -> CPU -> Action
# --------------------------------------------------------------------------


def test_cpu_decision_layer_produces_an_action_from_frames_only():
    world = _world(level="medium", steps=150)
    state_frame = encode_state(world.get_state())
    fpga = FpgaPredictionReference(FpgaConfig(horizon=15, rows=24, cols=32))
    prediction_frame = fpga.predict(state_frame)

    cpu = FrameCpuDecisionLayer()
    action, diag = cpu.decide(state_frame, prediction_frame)
    assert hasattr(action, "direction") and hasattr(action, "magnitude")
    assert 0.0 <= action.magnitude <= 1.0
    assert "risk" in diag and "risk_vector" in diag
    assert len(diag["risk_vector"]) == 9


def test_cpu_decision_layer_never_imports_the_world():
    """The CPU layer must work from bytes so real silicon can replace the rest."""
    import cpu.pipeline as pipeline

    text = open(pipeline.__file__, encoding="utf-8").read()
    assert "from bullet_sim.simulator" not in text


# --------------------------------------------------------------------------
# 5. the whole loop
# --------------------------------------------------------------------------


def test_full_cpu_fpga_loop_runs_and_measures_latency():
    world = World(
        scenario_for_level("easy", seed=5, duration=25.0),
        reward="zero",
        terminate_on_collision=False,
    )
    pipeline = CpuFpgaPipeline(
        FpgaPredictionReference(FpgaConfig(horizon=12, rows=20, cols=24))
    )
    for _ in range(60):
        pipeline.iterate(world)
    summary = pipeline.latency.summary()
    assert summary["iterations"] == 60
    assert summary["cpu_decide_ms"] > 0.0
    assert summary["fpga_predict_ms"] > 0.0
    assert 0.0 <= summary["fpga_share_pct"] <= 100.0


def test_pipeline_works_as_a_normal_controller():
    env = BulletHellEnv(
        scenario_for_level("easy", seed=6, duration=25.0),
        seed=6,
        terminate_on_collision=True,
    )
    try:
        ctrl = CpuFpgaController(
            env.world, FpgaPredictionReference(FpgaConfig(horizon=12, rows=20, cols=24))
        )
        result = env.run(ctrl, steps=200, seed=6)
        assert result["steps"] > 0
        assert ctrl.spec().access == "world"
    finally:
        env.close()


def test_pipeline_documents_exactly_what_is_implemented_and_what_is_tbd():
    info = CpuFpgaPipeline().describe()
    assert set(info) >= {
        "simulator -> fpga",
        "fpga -> cpu",
        "cpu -> simulator",
        "transport",
        "interface_status",
    }
    status = info["interface_status"]
    assert status["state_frame"] == "implemented"
    assert status["prediction_frame"] == "implemented"
    assert status["cpu_decision_layer"] == "implemented"
    assert status["cpu_fpga_transport"] == HARDWARE_INTERFACE_TBD
    assert "TBD" in status["fpga_prediction_block"]


# --------------------------------------------------------------------------
# 6. the board input half stays honest
# --------------------------------------------------------------------------


def test_board_input_is_still_explicitly_undecided():
    from bullet_sim.hardware_interface import (
        HardwareInterfaceNotConfigured,
        PlaceholderHardwareInputAdapter,
        PlannedTransport,
    )

    planned = PlannedTransport()
    assert planned.configured is False
    assert planned.kind is None and planned.baud_rate is None
    assert planned.byte_order is None and planned.register_base is None
    assert planned.packet_bytes is None
    with pytest.raises(HardwareInterfaceNotConfigured):
        PlaceholderHardwareInputAdapter().open()
    assert HARDWARE_INPUT_INTERFACE_TBD != ""
