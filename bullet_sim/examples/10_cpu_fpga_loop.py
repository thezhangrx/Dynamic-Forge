"""Example 10 - the CPU / FPGA boundary, end to end, over real byte frames.

    python -m bullet_sim.examples.10_cpu_fpga_loop
    python bullet_sim/examples/10_cpu_fpga_loop.py

Runs the whole hardware loop **in software**:

    World -> state frame -> FPGA reference -> prediction frame -> CPU -> Action -> World

and prints the frame sizes, the decision, the latency breakdown, and an explicit
list of what is implemented versus what is still ``[HARDWARE_INTERFACE_TBD]``.

No board, no driver, no transport is assumed.  Replacing the Python reference
with real RTL only requires the prediction frame to come out byte-identical.
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/10_cpu_fpga_loop.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.cpu import CpuDecisionLayer, CpuFpgaPipeline
from bullet_sim.fpga import (
    HARDWARE_INTERFACE_TBD,
    SOFTWARE_REFERENCE_ONLY,
    FpgaConfig,
    FpgaPredictionReference,
    decode_prediction_frame,
    describe_prediction_protocol,
)
from bullet_sim.interface.protocol import (
    decode_state,
    describe_protocol,
    encode_state,
    frame_size,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.world import World


def section(title: str) -> None:
    print()
    print("-" * 74)
    print(title)
    print("-" * 74)


def main() -> None:
    spec = scenario_for_level("medium", seed=21, duration=25.0)
    world = World(spec, reward="zero", terminate_on_collision=False)
    for _ in range(240):
        world.step(0)

    section("1. Simulator -> FPGA : the state frame")
    protocol = describe_protocol()
    print(f"  protocol        : {protocol['name']} v{protocol['version']}")
    print(f"  layout          : {protocol['frame_size']}")
    print(f"  bullet stride   : {protocol['bullet_stride_floats']} floats")
    print(f"  bullet fields   : {', '.join(protocol['bullet_fields'])}")

    state_frame = encode_state(world.get_state())
    header = decode_state(state_frame)
    print(f"  live bullets    : {header.bullet_count}")
    print(f"  frame bytes     : {len(state_frame)}  (expected {frame_size(header.bullet_count)})")

    section("2. FPGA reference block : state frame -> prediction frame")
    cfg = FpgaConfig(horizon=20, rows=30, cols=40)
    fpga = FpgaPredictionReference(cfg)
    print(f"  status          : {SOFTWARE_REFERENCE_ONLY}")
    print(f"  grid            : horizon={cfg.horizon} rows={cfg.rows} cols={cfg.cols}")
    print(f"  output frame    : {describe_prediction_protocol()['frame_size']}")

    prediction_frame = fpga.predict(state_frame)
    assert fpga.predict(state_frame) == prediction_frame, "golden model must be pure"
    grid, ph = decode_prediction_frame(prediction_frame)
    print(f"  frame bytes     : {len(prediction_frame)}")
    print(f"  source_step     : {ph.source_step}  (matches the state frame)")
    print(f"  grid shape      : {grid.shape}, range [{grid.min():.3f}, {grid.max():.3f}]")
    print(f"  reproducible    : True (same input bytes -> identical output bytes)")

    section("3. FPGA -> CPU : the decision layer reads only bytes")
    cpu = CpuDecisionLayer()
    action, diag = cpu.decide(state_frame, prediction_frame)
    print(f"  chosen action   : dir=({action.direction[0]:+.3f},{action.direction[1]:+.3f}) "
          f"mag={action.magnitude:.2f}")
    print(f"  worst risk      : {diag['risk']:.4f}")
    risks = np.asarray(diag["risk_vector"], dtype=float)
    pretty = " ".join(f"{v:+.2f}" for v in risks)
    print(f"  risk per action : {pretty}")
    print(f"  (order: stay left right up down up_left up_right down_left down_right)")

    section("4. the full loop, with latency breakdown")
    pipeline = CpuFpgaPipeline(fpga, cpu)
    for _ in range(300):
        pipeline.iterate(world)
    desc = pipeline.describe()
    for key in ("simulator -> fpga", "fpga -> cpu", "cpu -> simulator"):
        print(f"  {key:<18}: {desc[key]}")
    latency = desc["latency"]
    print(f"  iterations      : {latency['iterations']}")
    print(f"  total / iter    : {latency['total_ms']:.2f} ms")
    print(f"    encode_state  : {latency['encode_state_ms']:.3f} ms")
    print(f"    fpga_predict  : {latency['fpga_predict_ms']:.3f} ms")
    print(f"    cpu_decide    : {latency['cpu_decide_ms']:.3f} ms")
    print(f"    apply_action  : {latency['apply_action_ms']:.3f} ms")
    print(f"  FPGA share      : {latency['fpga_share_pct']:.1f} %  <- the part RTL will own")
    print(f"  transport       : {desc['transport']}")

    section("5. what is real and what is still TBD")
    for key, value in desc["interface_status"].items():
        print(f"  {key:<24}: {value}")
    print()
    print(f"  [HARDWARE_INTERFACE_TBD] = {HARDWARE_INTERFACE_TBD}")
    print("  Not decided, therefore never guessed:")
    print("    transport / baud / byte order / packet length / register map /")
    print("    handshake / LUT-FF-BRAM-DSP / timing closure")
    print()
    print("  To go to real hardware: keep the two frame layouts, replace")
    print("  FpgaPredictionReference with your RTL, and implement a")
    print("  HardwareInputAdapter once the board link is chosen.")
    print("  Nothing above the byte boundary needs to change.")


if __name__ == "__main__":
    main()
