"""Example 9 - the two control modes on one environment loop.

    python -m bullet_sim.examples.09_input_modes
    python bullet_sim/examples/09_input_modes.py

Demonstrates the specification's central requirement: a manual keyboard source
and a development-board source are *the same kind of object* and drive exactly
the same simulator code.

    人工输入  -> Action -> Simulator
    开发板    -> Action -> Simulator   (transport = [HARDWARE_INPUT_INTERFACE_TBD])

Nothing below opens a window, so it runs headless in CI.
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python core/bullet_sim/examples/09_input_modes.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.action import (
    Action,
    ManualInputSource,
    ScriptedSource,
    combine,
    keyboard_action_from_keys,
    source_from,
)
from bullet_sim.hardware_interface import (
    HARDWARE_INPUT_INTERFACE_TBD,
    HardwareInputSource,
    HardwareInterfaceNotConfigured,
    LoopbackHardwareInputAdapter,
    PlannedTransport,
    PlaceholderHardwareInputAdapter,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


def _env(seed: int = 3, duration: float = 20.0) -> BulletHellEnv:
    return BulletHellEnv(
        scenario_for_level("easy", seed=seed, duration=duration),
        seed=seed,
        terminate_on_collision=False,
    )


def section(title: str) -> None:
    print()
    print("-" * 74)
    print(title)
    print("-" * 74)


def main() -> None:
    section("1. Action = { direction, magnitude }")
    print("  discrete int      ->", Action.from_discrete("up_right"))
    print("  heading in degrees->", Action.from_angle(210.0))
    print("  raw 2D vector     ->", Action.from_vector([3.0, 4.0]))
    print("  up + right (norm) ->", combine(Action.from_discrete("up"),
                                           Action.from_discrete("right")))
    print("  opposite keys     ->", combine(Action.from_discrete("left"),
                                           Action.from_discrete("right")))

    section("2. Mode A - manual keyboard input (device-neutral key names)")
    print("  keys are names, not pygame constants, so any backend can drive it:")
    for keys in (["up"], ["up", "right"], ["shift", "up"], ["a", "s"], ["space"]):
        a = keyboard_action_from_keys(keys)
        print(f"    {str(keys):<22} -> {a}")

    manual = ManualInputSource()
    manual.open()
    manual.press("right")
    manual.press("up")
    manual.press("shift")
    print("  held {right, up, shift} ->", manual.current_action(), "(focus = slower)")
    manual.clear()

    section("3. Mode B - development board input (transport NOT decided)")
    planned = PlannedTransport()
    print(f"  marker            : {HARDWARE_INPUT_INTERFACE_TBD}")
    print(f"  transport kind    : {planned.kind}")
    print(f"  baud / byte order : {planned.baud_rate} / {planned.byte_order}")
    print(f"  register base     : {planned.register_base}")
    print(f"  configured        : {planned.configured}")
    print(f"  still unknown     : {', '.join(planned.missing())}")
    try:
        PlaceholderHardwareInputAdapter().open()
    except HardwareInterfaceNotConfigured as exc:
        print(f"  placeholder.open()-> HardwareInterfaceNotConfigured")
        print(f"    {str(exc)[:70]}...")
    print("  (the platform refuses to invent a protocol)")

    section("4. Both modes run the SAME environment loop")

    # --- mode A: manual -----------------------------------------------------
    env = _env(seed=3)
    manual = ManualInputSource()
    manual.open()
    manual.press("right")
    r_manual = env.run(input_source=manual, steps=400, seed=3)
    print(f"  mode A manual   : {r_manual['steps']:>4} steps, "
          f"collisions={r_manual['collisions']}, source={r_manual['input_source']}")
    print(f"                    final action={r_manual['last_action']}")

    # --- mode B: board (via the loopback test harness) ----------------------
    adapter = LoopbackHardwareInputAdapter()
    adapter.hold(Action.from_discrete("right"))
    r_board = env.run(input_source=HardwareInputSource(adapter), steps=400, seed=3)
    print(f"  mode B board    : {r_board['steps']:>4} steps, "
          f"collisions={r_board['collisions']}, source={r_board['input_source']}")
    print(f"                    adapter={adapter.describe()['adapter']}")

    print(f"  identical result: {r_manual['state_hash'] == r_board['state_hash']}")
    print(f"  state hash      : {r_manual['state_hash']}")

    # --- mode C: external program / policy ----------------------------------
    r_script = env.run(input_source=ScriptedSource([1, 2, 3, 0], loop=True), steps=400, seed=3)
    print(f"  mode C scripted : {r_script['steps']:>4} steps, "
          f"collisions={r_script['collisions']}, source={r_script['input_source']}")
    env.close()

    section("5. Adding the real board later changes NOTHING in the environment")
    print("  1. fill PlannedTransport in hardware_interface/tbd.py")
    print("  2. implement HardwareInputAdapter.read_payload/decode_payload")
    print("  3. env.run(input_source=HardwareInputSource(MyBoardAdapter()))")
    print("  the Action -> Simulator half is already finished and tested.")


if __name__ == "__main__":
    main()
