"""Example 2 - the complexity continuum and the obstacle scenes it builds.

    python bullet_sim/examples/02_patterns_and_scenarios.py

Difficulty is a number over a parameter space, and every level is built out of
*real* dynamic obstacles (moving block / wall with gap / small obstacles /
cross traffic / corridor).  Decorative centre-fired patterns no longer exist.
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python core/bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.obstacles.scenario import scenario_from_types
from bullet_sim.scenarios.builder import measure_live_bullets
from bullet_sim.scenarios.complexity import BULLET_COUNT_LADDER, profile_for
from bullet_sim.scenarios.presets import all_levels, stress


def main() -> None:
    print("== complexity continuum ==")
    for value in (0.0, 0.15, 0.4, 0.7, 1.0):
        p = profile_for(value)
        print(
            f"  complexity={value:.2f}  {p.label:<8} objects~{p.bullet_count:<5} "
            f"speed_scale={p.speed_scale:.2f}  obstacles={','.join(p.obstacle_kinds)}"
        )

    print("\n== named levels: which obstacle types they unlock ==")
    for name, spec in all_levels(seed=7).items():
        layouts = [p.params.get("layout") for p in spec.patterns]
        print(
            f"  {name:<8} target={spec.meta['target_bullet_count']:<5} "
            f"measured={spec.meta.get('measured_peak_bullets', 0):<7.0f} "
            f"player_r={spec.player_radius:<5.1f} layouts={layouts}"
        )

    print("\n== a hand-composed obstacle scene (each type keeps its parameters) ==")
    scene = scenario_from_types(
        [
            {"type": "moving_block", "size": 2.0, "speed": 90.0, "entry_span": 0.6},
            {"type": "wall_with_gap", "gap_width": 4.0 * 20.0, "gap_motion": "sweep"},
            {"type": "small_obstacles", "count": 12, "size": 0.35},
            {"type": "corridor", "corridor_width": 90.0, "motion": "rotate_same",
             "change": 18.0},
        ],
        seed=11, duration=20.0, player_hitbox_radius=10.0,
    )
    spec = scene.to_spec()
    print(f"  obstacle types : {scene.type_keys}")
    print(f"  layouts        : {[p.params.get('layout') for p in spec.patterns]}")
    print(f"  player circle  : r={spec.player_radius} "
          f"(diameter {spec.player_diameter}, visible == collision)")
    measured = measure_live_bullets(spec)
    print(f"  measured peak  : {measured['peak']:.0f} simultaneous obstacles")

    print("\n== 'run exactly N obstacles' (headless calibration) ==")
    for count in BULLET_COUNT_LADDER[:4]:
        s = stress(count, seed=3)
        measured = measure_live_bullets(s)["peak"]
        layouts = [p.params.get("layout") for p in s.patterns]
        print(f"  requested {count:<5} -> measured peak {measured:>6.0f} "
              f"({measured / count * 100:.1f}%)  layouts={layouts}")


if __name__ == "__main__":
    main()
