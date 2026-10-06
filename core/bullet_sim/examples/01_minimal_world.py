"""Example 1 - the smallest possible world.

Shows the Phase-1 contract: a continuous 2D field, one player, one bullet,
a fixed timestep, and exact analytic motion.

    python bullet_sim/examples/01_minimal_world.py
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python core/bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.world import World


def main() -> None:
    dt = 1.0 / 120.0
    spec = ScenarioSpec(
        name="one_bullet",
        seed=0,
        duration=2.0,
        dt=dt,
        field_w=640.0,
        field_h=480.0,
        player_x=320.0,
        player_y=100.0,
        patterns=[],              # no emitters: we inject the bullet by hand
    )
    world = World(spec, collision="null", reward="zero", terminate_on_collision=False)

    # One bullet with constant acceleration.
    world.spawn_bullets(x=0.0, y=240.0, vx=120.0, vy=0.0, ax=10.0, ay=0.0,
                        radius=3.0, ttl=5.0)

    n = 120
    for _ in range(n):
        world.step(0)             # action 0 = 'stay'

    pool = world.state.bullets
    idx = pool.active_indices()[0]
    t = n * dt
    predicted_x = 0.0 + 120.0 * t + 0.5 * 10.0 * t * t

    print(f"dt            : {dt:.6f} s  ({1 / dt:.0f} Hz)")
    print(f"steps         : {n}  ({t:.3f} s of simulation)")
    print(f"bullet x      : {pool.data['x'][idx]:.9f}")
    print(f"analytic x    : {predicted_x:.9f}")
    print(f"match         : {np.isclose(pool.data['x'][idx], predicted_x)}")
    print(f"bullet vx     : {pool.data['vx'][idx]:.9f} (analytic {120.0 + 10.0 * t:.9f})")
    print(f"player        : x={world.player[0]:.3f} y={world.player[1]:.3f}")
    print(f"state hash    : {world.state_hash()}")


if __name__ == "__main__":
    main()
