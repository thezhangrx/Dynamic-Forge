"""Example 11 - the dynamic-obstacle environment and its safety guarantee.

    python bullet_sim/examples/11_obstacle_environment.py

Shows, end to end:

* the nine real-world obstacle types and their documented semantics;
* a single type on its own, then several types composed;
* relative motion ``v_rel = v_obs - v_player``;
* the safety layer that makes "有挑战 ≠ 必死" checkable:
  free space -> feasible path -> verdict -> automatic re-sampling.
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from bullet_sim.obstacles.catalog import CATALOG, GENERATABLE
from bullet_sim.obstacles.scenario import scenario_from_types
from bullet_sim.physics.relative import (
    closing_speed,
    obstacle_positions,
    obstacle_velocities,
    to_player_frame,
)
from bullet_sim.safety import (
    SafetyConfig,
    compute_safe_region,
    find_safe_path,
    generate_valid_scenario,
    is_scenario_valid,
    validate_scenario,
)
from bullet_sim.simulator.world import World

FIELD = (640.0, 480.0)


def _world(scenario, *, duration: float = 8.0):
    spec = scenario.to_spec().replaced(duration=duration)
    return World(spec, reward="zero", terminate_on_collision=False)


def section(title: str) -> None:
    print(f"\n== {title} ==")


def main() -> None:
    section("the obstacle catalogue (what each type stands for)")
    for key in GENERATABLE:
        t = CATALOG[key]
        print(f"  {key:<24} {t.label_zh}")
        print(f"      simulates : {t.simulates}")
        print(f"      safety    : {t.safety_strategy}")

    section("one type alone: moving_block")
    world = _world(scenario_from_types(["moving_block"], seed=7))
    peak = 0
    shapes: set[int] = set()
    half_w: list[float] = []
    for _ in range(480):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        peak = max(peak, int(pool.count))
        if idx.size:
            shapes.update(np.asarray(pool.data["shape"][idx]).tolist())
            half_w.extend(np.asarray(pool.data["half_w"][idx]).tolist())
    print(f"  peak obstacles: {peak}")
    print(f"  shapes        : {sorted(shapes)}  (0=circle, 1=rect)")
    if half_w:
        print(f"  half_w range  : {min(half_w):.1f} .. {max(half_w):.1f}")
    print(f"  player circle : r={world.spec.player_radius:.1f} "
          f"(diameter {world.spec.player_diameter:.1f}, visible == collision)")

    section("several types at once (each keeps its own parameters)")
    mixed = scenario_from_types(
        [
            {"type": "moving_block", "size": 2.0, "speed": 90.0},
            {"type": "wall_with_gap", "gap_width": 180.0},
            {"type": "small_obstacles", "count": 10, "size": 0.35},
        ],
        seed=11,
    )
    print(f"  type keys     : {mixed.type_keys}")
    print(f"  patterns      : {[p.params.get('layout') for p in mixed.to_spec().patterns]}")
    w2 = _world(mixed)
    for _ in range(300):
        w2.step(0)
    print(f"  live obstacles: {w2.active_bullet_count()}")

    section("relative motion v_rel = v_obs - v_player")
    from bullet_sim.action import Action

    w3 = World(
        scenario_from_types(["cross_traffic"], seed=5).to_spec().replaced(duration=8.0),
        reward="zero",
        terminate_on_collision=False,
        codec="action",
    )
    action = Action.from_discrete("right")
    for _ in range(120):
        w3.step(action)
    snap = w3.get_state()
    frame = to_player_frame(snap)
    idx = frame.bullets.active_indices()
    if idx.size:
        pos = obstacle_positions(frame.bullets, idx)
        vel = obstacle_velocities(frame.bullets, idx)
        me = np.array([frame.player.x, frame.player.y])
        close = closing_speed(pos, vel, me)
        nearest = int(np.argmax(close))
        print(f"  player velocity        : ({snap.player.vx:.1f}, {snap.player.vy:.1f})")
        print(f"  obstacles              : {idx.size}")
        print(f"  max closing speed      : {close.max():.1f} world units/s")
        print(f"  nearest obstacle offset: "
              f"({pos[nearest, 0] - me[0]:.1f}, {pos[nearest, 1] - me[1]:.1f})")

    section("safety: a usable gap is feasible, a too-tight one is not")
    config = SafetyConfig(horizon=8.0, sample_dt=0.1, resolution=16.0, max_window=8.0)
    player_d = 2.0 * 10.0     # the default player circle diameter
    for gap in (200.0, player_d):
        sc = scenario_from_types(
            [{"type": "wall_with_gap", "gap_width": gap, "speed": 200.0, "repetitions": 1}],
            seed=1, player_hitbox_radius=10.0,
        )
        w = _world(sc, duration=8.6)
        report = validate_scenario(w, config=config)
        verdict = "feasible" if report.feasible else "INFEASIBLE"
        print(f"  gap_width={gap:6.1f} -> {verdict:<10} "
              f"free={report.free_fraction:5.1%} reachable={report.reachable_cells:4d} "
              f"| {report.reason}")
        if report.feasible:
            path = find_safe_path(w, config)
            region = compute_safe_region(w, config)
            print(f"      safe path : {path.length} waypoints, "
                  f"min clearance {path.min_clearance:.1f}")
            print(f"      safe region: {len(region.safe_points(limit=8))} sample points, "
                  f"reachable fraction {region.reachable_fraction:.1%}")
        else:
            for s in report.suggestions[:3]:
                print(f"      suggestion: {s}")

    section("closed loop: generate -> check -> adjust -> regenerate")
    hard = scenario_from_types(
        [{"type": "wall_with_gap", "gap_width": player_d, "speed": 200.0, "repetitions": 1}],
        seed=1, player_hitbox_radius=10.0,
    )
    fixed, report, attempts = generate_valid_scenario(
        hard, config=config, max_attempts=10, strict=False
    )
    print(f"  attempts      : {attempts}")
    print(f"  feasible      : {report.feasible}")
    print(f"  gap_width     : {hard.obstacles[0].gap_width} -> "
          f"{fixed.obstacles[0].gap_width}")
    print(f"  obstacle speed: {hard.obstacles[0].speed} -> {fixed.obstacles[0].speed}")
    print(f"  is_scenario_valid(fixed): {is_scenario_valid(fixed.to_spec(), config=config)}")


if __name__ == "__main__":
    main()
