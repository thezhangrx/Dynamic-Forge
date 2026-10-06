#!/usr/bin/env python3
"""Bullet-Hell Simulator - single-file runnable entry point.

No installation, no pygame, no pytest required: NumPy is the only dependency.

    python3 bullet_sim_run_demo.py            # fast guided tour (default, ~15 s)
    python3 bullet_sim_run_demo.py quick      # Phase 1: minimal world, exact motion
    python3 bullet_sim_run_demo.py patterns   # Phase 3: patterns + difficulty ladder
    python3 bullet_sim_run_demo.py record     # Phase 4: headless -> dataset -> replay verify
    python3 bullet_sim_run_demo.py predict    # Phase 6: future rollouts + danger field
    python3 bullet_sim_run_demo.py bench      # Phase 7: performance / stress ladder
    python3 bullet_sim_run_demo.py ascii      # Phase 8: visual debug without a display
    python3 bullet_sim_run_demo.py all        # every demo above

Everything is also exposed as a library:

    from bullet_sim import BulletHellEnv, stress
    env = BulletHellEnv(stress(1000, seed=7))
    obs, info = env.reset(seed=7)
    obs, reward, terminated, truncated, info = env.step(1)
"""

from __future__ import annotations

import os
import pathlib
import sys
import tempfile
import time

# Make the package importable no matter where this file is invoked from.
# All module source lives under core/, so that is what goes on sys.path.
ROOT = pathlib.Path(__file__).resolve().parent / "core"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def banner(title: str) -> None:
    print()
    print("=" * 78)
    print(f"  {title}")
    print("=" * 78)


def _np():
    try:
        import numpy as np
    except ImportError:  # pragma: no cover - environment guard
        print(
            "NumPy is required but not installed.\n"
            "  pip install numpy\n"
            "or use the bundled environment:  Env/bin/python bullet_sim_run_demo.py"
        )
        raise SystemExit(2)
    return np


# ---------------------------------------------------------------------------
# 1. minimal world: fixed timestep + exact analytic motion
# ---------------------------------------------------------------------------


def demo_quick() -> None:
    np = _np()
    from bullet_sim.simulator.world import World
    from bullet_sim.scenarios.spec import ScenarioSpec

    banner("1/6  Minimal world - one player, one bullet, fixed dt")

    dt = 1.0 / 120.0
    spec = ScenarioSpec(
        name="one_bullet", seed=0, duration=2.0, dt=dt,
        field_w=640.0, field_h=480.0, player_x=320.0, player_y=100.0,
        patterns=[],
    )
    world = World(spec, collision="null", reward="zero", terminate_on_collision=False)
    world.spawn_bullets(x=0.0, y=240.0, vx=120.0, vy=0.0, ax=10.0, ay=0.0,
                        radius=3.0, ttl=5.0)

    steps = 120
    for _ in range(steps):
        world.step(0)

    pool = world.state.bullets
    slot = pool.active_indices()[0]
    t = steps * dt
    analytic_x = 120.0 * t + 0.5 * 10.0 * t * t

    print(f"  dt                 : {dt:.6f} s  ({1 / dt:.0f} Hz)")
    print(f"  simulated          : {steps} steps = {t:.3f} s")
    print(f"  bullet x  (sim)    : {pool.data['x'][slot]:.9f}")
    print(f"  bullet x  (analytic): {analytic_x:.9f}")
    print(f"  exact match        : {np.isclose(pool.data['x'][slot], analytic_x)}")
    print(f"  bullet vx (analytic): {120.0 + 10.0 * t:.9f}")
    print(f"  state hash         : {world.state_hash()}")


# ---------------------------------------------------------------------------
# 2. patterns + difficulty
# ---------------------------------------------------------------------------


def demo_patterns() -> None:
    _np()
    from bullet_sim.generators.registry import build_timeline
    from bullet_sim.obstacles.scenario import scenario_from_types
    from bullet_sim.scenarios.complexity import profile_for
    from bullet_sim.scenarios.presets import all_levels

    banner("2/6  Obstacle scenes and the complexity continuum")

    print("  complexity continuum (which real obstacle types it unlocks):")
    for value in (0.0, 0.15, 0.40, 0.70, 1.00):
        p = profile_for(value)
        print(f"    c={value:.2f}  {p.label:<8} objects~{p.bullet_count:<5} "
              f"speed_scale={p.speed_scale:.2f}  obstacles={','.join(p.obstacle_kinds)}")

    print("\n  named presets (calibrated by real measurement):")
    for name, spec in all_levels(seed=7).items():
        print(f"    {name:<8} target={spec.meta['target_bullet_count']:<5} "
              f"measured_peak={spec.meta.get('measured_peak_bullets', 0):<7.0f} "
              f"capacity={spec.bullet_capacity:<6} duration={spec.duration:.1f}s "
              f"layouts={[p.params.get('layout') for p in spec.patterns]}")

    print("\n  composed scene: moving block + wall with gap + small obstacles + corridor")
    scene = scenario_from_types(
        [
            {"type": "moving_block", "size": 2.0, "speed": 90.0},
            {"type": "wall_with_gap", "gap_width": 180.0, "gap_motion": "sweep"},
            {"type": "small_obstacles", "count": 12, "size": 0.35},
            {"type": "corridor", "corridor_width": 90.0, "motion": "rotate_same",
             "change": 18.0},
        ],
        seed=11, duration=20.0, player_hitbox_radius=10.0,
    )
    spec = scene.to_spec()
    print(f"    obstacle types : {scene.type_keys}")
    print(f"    player circle  : r={spec.player_radius} (visible == collision)")
    from bullet_sim.scenarios.builder import measure_live_bullets

    print(f"    measured peak  : {measure_live_bullets(spec)['peak']:.0f} obstacles")


# ---------------------------------------------------------------------------
# 3. headless recording, export, reload, exact replay
# ---------------------------------------------------------------------------


def demo_record() -> None:
    _np()
    from bullet_sim.dataset.readers import load_episode, verify_roundtrip
    from bullet_sim.dataset.recorder import TrajectoryRecorder
    from bullet_sim.dataset.writers import save_episode
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.env import BulletHellEnv
    from bullet_sim.simulator.replay import verify_episode

    banner("3/6  Headless run -> training dataset -> reload -> exact replay")

    out = pathlib.Path(tempfile.mkdtemp(prefix="bullet_sim_demo_"))
    spec = scenario_for_level("medium", seed=17, duration=20.0)

    rec = TrajectoryRecorder(store_hashes=True, extra_meta={"scenario": spec.to_dict()})
    env = BulletHellEnv(spec, seed=17, record=rec, terminate_on_collision=False)
    result = env.run("random", steps=800, seed=17)
    episode = rec.finish()
    env.close()

    print(f"  headless run      : {result['steps']} steps, "
          f"{result['collisions']} collision(s), return={result['return']:.1f}")
    print(f"  dataset           : {episode.summary()}")
    print(f"  arrays            :")
    for key in ("player", "target", "env", "bullets", "action_vectors",
                "rewards", "collisions", "done", "state_hashes"):
        arr = episode.arrays[key]
        print(f"    {key:<16} {str(arr.shape):<14} {arr.dtype}")

    tr = episode.transition(0)
    print(f"  one transition    : S_0 bullets={tr['state_t']['bullet_count']} "
          f"-> A_0={tr['action_t']} -> r={tr['reward']} -> "
          f"S_1 bullets={tr['next_state']['bullet_count']}")

    print("\n  export / re-import:")
    for fmt in ("npz", "json", "bin", "csv"):
        written = save_episode(episode, out / f"ep.{fmt}")
        ok = verify_roundtrip(episode, written)
        size = pathlib.Path(written).stat().st_size
        print(f"    {fmt:<5} {size:>9} B   lossless round-trip = {ok}")

    comparison = verify_episode(episode)
    print(f"\n  replay from (scenario, seed, actions): reproduced = {comparison.ok} "
          f"({comparison.reason}; checked {comparison.expected_steps} per-step "
          f"state hashes)")
    print(f"  replay policy     : {comparison.policy}")

    loaded = load_episode(out / "ep.npz")
    print(f"  reloaded episodes : {loaded.steps} transitions, seed={loaded.seed}, "
          f"scenario={loaded.scenario_id}")
    print(f"  artifacts kept in : {out}")


# ---------------------------------------------------------------------------
# 4. future simulation, risk, candidate action evaluation
# ---------------------------------------------------------------------------


def demo_predict() -> None:
    _np()
    from bullet_sim.prediction.ballistic import BallisticPredictor
    from bullet_sim.prediction.danger_field import compute_danger_field
    from bullet_sim.prediction.rollout import (
        FutureSimulator, best_candidate, clearance, evaluate_candidates,
        time_to_collision,
    )
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.env import BulletHellEnv

    banner("4/6  Future simulation, danger field, candidate-action evaluation")

    spec = scenario_for_level("hard", seed=4, duration=30.0)
    env = BulletHellEnv(spec, seed=4, reward="zero", terminate_on_collision=False)
    env.reset(seed=4)
    for _ in range(300):
        env.step(0)

    snap = env.get_state()
    print(f"  now               : step={snap.env.step_index} "
          f"bullets={snap.bullet_count} t={snap.timestamp:.2f}s")
    print(f"  clearance         : {clearance(snap):.3f} px  "
          f"(<0 means overlapping)")
    print(f"  time-to-impact    : {time_to_collision(snap):.3f} s (player at rest)")

    horizon = 90
    forecast = BallisticPredictor().predict(snap, horizon, spec.dt)
    print(f"\n  analytic forecast : {forecast.summary()}")

    sim = FutureSimulator(env.world)
    states = sim.simulate_future(horizon, action=0, include_initial=True)
    print(f"  clone rollout     : {len(states)} states, "
          f"bullets at t+{horizon} = {states[-1].bullet_count} "
          f"(live world untouched: {env.state_hash() == snap.state_hash()})")

    results = evaluate_candidates(sim, list(range(9)), horizon=60, objective="survival")
    print("\n  candidate actions (60-step rollout, survival objective):")
    for r in results:
        print(f"    action={r.action}  objective={r.objective:>8.3f}  "
              f"survived={str(r.survived):<5} min_clearance={r.min_clearance:>8.3f}")
    best = best_candidate(results)
    print(f"    -> best action = {best.action}")

    danger = compute_danger_field(states, resolution=16.0, safety_radius=24.0)
    p = snap.player
    print(f"\n  danger field      : grid={danger.grid.shape} "
          f"resolution={danger.resolution}px")
    print(f"    risk at player  : {danger.risk_at(p.x, p.y, 0):.3f}")
    print(f"    peak risk       : {float(danger.max_over_time().max()):.3f}")
    print(f"    safe cells now  : {float(danger.safe_mask(0.01)[0].mean()) * 100:.1f}%")
    env.close()


# ---------------------------------------------------------------------------
# 5. benchmark
# ---------------------------------------------------------------------------


def demo_bench() -> None:
    _np()
    from bullet_sim.benchmark.metrics import format_table
    from bullet_sim.benchmark.suites import bullet_ladder_suite, collision_backend_suite

    banner("5/6  Performance benchmark and high-density stress ladder")

    print("  running ladder 50 / 200 / 1000 bullets "
          "(full ladder: python3 -m bullet_sim benchmark) ...\n")
    report = bullet_ladder_suite((50, 200, 1000), steps=150, seed=0)
    print()
    print(format_table(report["rows"]))

    print("\n  collision backend comparison (batch players):")
    print(format_table(collision_backend_suite(bullet_count=1000, seed=0)["rows"]))
    print("\n  tip: 'circle' wins for 1 player, 'grid' wins for >= 8 players.")


# ---------------------------------------------------------------------------
# 6. visual debug without a display
# ---------------------------------------------------------------------------


def demo_inputs() -> None:
    _np()
    from bullet_sim.action import Action, ManualInputSource, keyboard_action_from_keys
    from bullet_sim.hardware_interface import (
        HARDWARE_INPUT_INTERFACE_TBD,
        HardwareInputSource,
        HardwareInterfaceNotConfigured,
        LoopbackHardwareInputAdapter,
        PlaceholderHardwareInputAdapter,
        PlannedTransport,
    )
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.env import BulletHellEnv

    banner("7/7  Two control modes, one environment loop")

    print("  Action = { direction, magnitude }")
    for keys in (["up"], ["up", "right"], ["shift", "up"], ["space"]):
        print(f"    keys {str(keys):<16} -> {keyboard_action_from_keys(keys)}")

    print("\n  mode B transport is NOT decided:")
    planned = PlannedTransport()
    print(f"    marker={HARDWARE_INPUT_INTERFACE_TBD} kind={planned.kind} "
          f"baud={planned.baud_rate} byte_order={planned.byte_order}")
    try:
        PlaceholderHardwareInputAdapter().open()
    except HardwareInterfaceNotConfigured as exc:
        print(f"    placeholder.open() -> HardwareInterfaceNotConfigured")
        print(f"      {str(exc)[:64]}...")

    spec = scenario_for_level("easy", seed=3, duration=20.0)
    env = BulletHellEnv(spec, seed=3, terminate_on_collision=False)

    manual = ManualInputSource()
    manual.open()
    manual.press("right")
    r_manual = env.run(input_source=manual, steps=400, seed=3)

    adapter = LoopbackHardwareInputAdapter()
    adapter.hold(Action.from_discrete("right"))
    r_board = env.run(input_source=HardwareInputSource(adapter), steps=400, seed=3)
    env.close()

    print(f"\n  mode A manual : {r_manual['steps']} steps  source={r_manual['input_source']}")
    print(f"  mode B board  : {r_board['steps']} steps  source={r_board['input_source']}")
    print(f"  identical state hash: {r_manual['state_hash'] == r_board['state_hash']} "
          f"({r_manual['state_hash']})")


def demo_ascii() -> None:
    _np()
    from bullet_sim.scenarios.presets import scenario_for_level
    from bullet_sim.simulator.env import BulletHellEnv

    banner("6/6  ASCII visual debug (no window, no pygame)")

    spec = scenario_for_level("medium", seed=31)
    env = BulletHellEnv(spec, seed=31)
    env.reset(seed=31)

    for i in range(360):
        env.step(0)
        if i % 120 == 0:
            print(env.render("ascii", width=76, height=22))
            print()
    env.close()
    print("  legend: @ player   * bullet   O target   . danger   "
          "(windowed view: python3 -m bullet_sim play --level medium --prediction)")


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

DEMOS = {
    "quick": demo_quick,
    "patterns": demo_patterns,
    "record": demo_record,
    "predict": demo_predict,
    "bench": demo_bench,
    "ascii": demo_ascii,
    "inputs": demo_inputs,
}
TOUR = ["quick", "record", "predict", "inputs", "ascii"]


def main(argv: list[str] | None = None) -> int:
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    argv = list(sys.argv[1:] if argv is None else argv)
    what = argv[0].lower() if argv else "tour"

    if what in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    if what == "all":
        names = list(DEMOS)
    elif what == "tour":
        names = TOUR
    elif what in DEMOS:
        names = [what]
    else:
        print(f"unknown demo {what!r}\n")
        print(__doc__)
        return 2

    print(f"bullet_sim - running: {', '.join(names)}")
    print(f"python   : {sys.executable}")
    started = time.perf_counter()
    for name in names:
        DEMOS[name]()
    print(f"\nDone. {len(names)} demo(s) in {time.perf_counter() - started:.1f}s.")
    if what == "tour":
        print("Full docs: README.md, core/bullet_sim/docs/ARCHITECTURE.md, "
              "core/bullet_sim/docs/USAGE.md")
    return 0


if __name__ == "__main__":
    # Same per-launch debug log as the CLI: data/bullet_sim/debug_<date stamp>.log
    from bullet_sim.debug_log import start as _start_debug_log

    _log = _start_debug_log("demo", sys.argv[1:])
    if _log is None:
        raise SystemExit(main())
    with _log:
        _log.exit_code = main()
    raise SystemExit(_log.exit_code)
