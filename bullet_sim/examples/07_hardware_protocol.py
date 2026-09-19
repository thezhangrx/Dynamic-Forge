"""Example 7 - the CPU/FPGA State-Action protocol.

Demonstrates the deployment path:

    Virtual Simulator --encode_state--> CPU --> FPGA input buffer
                      <--prediction--- FPGA output buffer
                      ----action------> CPU --> FPGA / actuators

and shows that switching to ``Physical Sensor -> CPU -> FPGA`` only replaces the
producer of the state frame; the protocol and the CPU/FPGA side are unchanged.

    python bullet_sim/examples/07_hardware_protocol.py
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.interface.hardware import LoopbackLink
from bullet_sim.interface.protocol import (
    bullet_matrix,
    decode_state,
    describe_protocol,
    encode_action_velocity,
    encode_state,
    frame_size,
    read_header,
)
from bullet_sim.scenarios.presets import stress
from bullet_sim.simulator.world import World


def fake_fpga_accelerator(state_frame: bytes) -> bytes:
    """Stand-in for the FPGA: echoes the bullet count back as a 4-byte payload."""
    header = read_header(state_frame)
    return header.n_bullets.to_bytes(4, "little")


def main() -> None:
    print("== protocol ==")
    for key, value in describe_protocol().items():
        print(f"  {key:<20}: {value}")

    world = World(stress(500, seed=2, duration=8.0), reward="zero",
                  terminate_on_collision=False)
    for _ in range(240):
        world.step(0)

    snapshot = world.get_state()
    frame = encode_state(snapshot)
    header = read_header(frame)
    print(f"\n== state frame ==")
    print(f"  bullets            : {header.n_bullets}")
    print(f"  frame bytes        : {len(frame)}  (expected {frame_size(header.n_bullets)})")
    print(f"  bytes per bullet   : 40")
    print(f"  step / sim_time    : {header.step_index} / {header.sim_time:.4f}s")

    # Zero-copy view of the bullet block, exactly what a DMA descriptor targets.
    matrix = bullet_matrix(frame)
    print(f"  bullet matrix view : {matrix.shape} {matrix.dtype}")
    print(f"  first bullet row   : {np.round(matrix[0], 3).tolist()}")

    # Full decode back into a WorldSnapshot (used by the CPU-side consumer).
    restored = decode_state(frame)
    print(f"  decoded bullets    : {restored.bullet_count}")

    print("\n== link round-trip (CPU <-> accelerator) ==")
    link = LoopbackLink(accelerator=fake_fpga_accelerator)
    link.open()
    prediction = link.roundtrip(frame)
    link.send_action(np.array([12.5, 0.0]))
    print(f"  FPGA baseline reply : {int.from_bytes(prediction, 'little')} bullets")
    print(f"  total frame up      : {link.stats.bytes_up} B")
    print(f"  round-trip latency  : {link.stats.summary()['download_latency']}")

    print("\n== action frame ==")
    payload = encode_action_velocity(30.0, -12.0)
    print(f"  velocity action     : {payload.hex()}  ({len(payload)} bytes)")

    print(
        "\nSwapping `Virtual Simulator` for `Physical Sensor` only changes who\n"
        "produces the state frame - the protocol, the CPU decision layer and the\n"
        "FPGA prediction block stay identical."
    )


if __name__ == "__main__":
    main()
