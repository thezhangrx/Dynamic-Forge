"""Benchmark runners.

Five measurements requested by the specification:

==========================  ==================================================
simulation FPS              simulation steps per wall-clock second
step latency                per-step wall time (mean / p50 / p99)
bullet count                simultaneously live bullets at the peak
collision check throughput  bullet-vs-player candidate tests per second
scenario generation speed   scenes (and bullets) built per second
==========================  ==================================================
"""

from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from bullet_sim.benchmark.metrics import BenchmarkResult, Stats, format_table, timeit
from bullet_sim.entities.bullet import BulletPool
from bullet_sim.scenarios.builder import build_from_spec, build_scenario, measure_live_bullets
from bullet_sim.prediction.ballistic import BallisticPredictor
from bullet_sim.prediction.danger_field import compute_danger_field
from bullet_sim.scenarios.presets import stress
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.world import World


def auto_warmup(scenario: ScenarioSpec, *, cap: int = 1500, floor: int = 60) -> int:
    """Steps needed to reach a representative scene density.

    Scenes build up over time, so measuring from step 0 would report an
    artificially cheap step.  A quarter of the scenario duration is enough to
    reach (or pass) the dense phase for every pattern family.
    """
    return int(min(max(floor, scenario.total_steps // 4), cap))


def _fresh_world(
    scenario: ScenarioSpec,
    *,
    collision: str = "circle",
    collision_kwargs: Mapping[str, Any] | None = None,
    warmup_steps: int | None = None,
) -> World:
    """Build a world and let the scene reach a representative density."""
    steps = auto_warmup(scenario) if warmup_steps is None else int(warmup_steps)
    world = World(
        scenario,
        collision=collision,
        collision_kwargs=collision_kwargs,
        reward="zero",
        terminate_on_collision=False,
    )
    for _ in range(steps):
        world.step(0)
    return world


def benchmark_step_latency(
    scenario: ScenarioSpec,
    *,
    steps: int = 400,
    warmup: int | None = None,
    collision: str = "circle",
    collision_kwargs: Mapping[str, Any] | None = None,
    action: Any = 0,
) -> BenchmarkResult:
    """Per-step wall time of the full kernel (player + spawn + motion + coll + reward)."""
    auto = auto_warmup(scenario)
    world = _fresh_world(
        scenario,
        collision=collision,
        collision_kwargs=collision_kwargs,
        warmup_steps=warmup if warmup is not None else auto,
    )
    samples: list[float] = []
    bullets: list[int] = []
    import time

    for _ in range(int(steps)):
        t0 = time.perf_counter()
        world.step(action)
        samples.append(time.perf_counter() - t0)
        bullets.append(world.active_bullet_count())
    stats = Stats.from_samples(samples[5:])  # drop cache-warming outliers
    mean_bullets = float(np.mean(bullets)) if bullets else 0.0
    window_peak = int(np.max(bullets)) if bullets else 0
    scene_peak = int(scenario.meta.get("measured_peak_bullets", window_peak) or window_peak)
    metrics = stats.to_dict("ms")
    metrics.update(
        {
            "simulation_fps": stats.rate(),
            "mean_bullets": mean_bullets,
            "window_peak_bullets": window_peak,
            "scene_peak_bullets": scene_peak,
            "warmup_steps": int(warmup if warmup is not None else auto),
            "dt": float(scenario.dt),
            "realtime_factor": stats.rate() * float(scenario.dt),
            "us_per_bullet_step": (
                (stats.mean * 1e6) / mean_bullets if mean_bullets > 0 else float("nan")
            ),
            "collision_model": collision,
        }
    )
    return BenchmarkResult(
        name=f"step_latency[{scenario.name}]",
        metrics=metrics,
        notes={"seed": scenario.seed, "duration": scenario.duration},
    )


def benchmark_collision_throughput(
    scenario: ScenarioSpec,
    *,
    queries: int = 300,
    warmup: int | None = None,
    collision: str = "circle",
    players: int = 1,
) -> BenchmarkResult:
    """Candidate bullet tests per second for the collision backend."""
    world = _fresh_world(scenario, collision=collision, warmup_steps=warmup)
    model = world.collision
    pool = world.state.bullets
    model.prepare(pool)
    n = pool.count
    px, py, pr = 100.0, 100.0, 3.0
    if players > 1:
        rng = np.random.default_rng(0)
        xs = rng.uniform(0, scenario.field_w, players)
        ys = rng.uniform(0, scenario.field_h, players)
        rs = np.full(players, 3.0)
        fn: Callable[[], Any] = lambda: model.query_many(pool, xs, ys, rs)  # type: ignore[attr-defined]
        tests_per_call = n * players
    else:
        fn = lambda: model.query(pool, px, py, pr)
        tests_per_call = n
    stats = timeit(fn, repeat=queries, warmup=5)
    metrics = stats.to_dict("us")
    metrics.update(
        {
            "bullets": n,
            "players_per_query": players,
            "tests_per_query": tests_per_call,
            "tests_per_second": stats.rate(per=float(tests_per_call)),
            "queries_per_second": stats.rate(),
            "collision_model": collision,
        }
    )
    return BenchmarkResult(name=f"collision_throughput[{scenario.name}]", metrics=metrics)


def benchmark_scenario_generation(
    level: Any = "hard",
    *,
    seed: int = 0,
    repeats: int = 5,
    bullet_count: int | None = None,
) -> BenchmarkResult:
    """Scenario construction cost (complexity -> calibrated spec -> timeline)."""
    scenarios: list[ScenarioSpec] = []

    def build() -> None:
        s = stress(bullet_count, seed=seed) if bullet_count else build_scenario(level, seed=seed).spec
        scenarios.append(s)

    stats = timeit(build, repeat=repeats, warmup=1)
    last = scenarios[-1] if scenarios else None
    emitted = 0
    if last is not None:

        emitted = build_from_spec(last).timeline.total_bullets
    metrics = stats.to_dict("ms")
    metrics.update(
        {
            "scenarios_per_second": stats.rate(),
            "emitted_bullets": emitted,
            "bullets_per_second": stats.rate(per=float(emitted)),
            "seed": seed,
        }
    )
    return BenchmarkResult(name="scenario_generation", metrics=metrics)


def benchmark_raw_motion(
    bullet_count: int = 5000, *, steps: int = 400, repeat: int = 3
) -> BenchmarkResult:
    """Isolated motion-integration throughput (the FPGA-bound inner loop)."""
    pool = BulletPool(bullet_count)
    rng = np.random.default_rng(0)
    idx = pool.alloc(bullet_count)
    d = pool.data
    d["x"][idx] = rng.uniform(0, 640, bullet_count)
    d["y"][idx] = rng.uniform(0, 480, bullet_count)
    d["vx"][idx] = rng.uniform(-100, 100, bullet_count)
    d["vy"][idx] = rng.uniform(-100, 100, bullet_count)
    d["radius"][idx] = 3.0
    d["ttl"][idx] = 1e9
    from bullet_sim.physics.motion import integrate_bullets

    def run() -> None:
        for _ in range(int(steps)):
            integrate_bullets(pool, 1.0 / 120.0)

    stats = timeit(run, repeat=repeat, warmup=1)
    per_step = stats.mean / steps
    metrics = {
        "bullets": bullet_count,
        "steps_per_run": steps,
        f"total_ms": stats.total * 1e3,
        "mean_step_us": per_step * 1e6,
        "bullet_updates_per_second": bullet_count / per_step if per_step > 0 else float("inf"),
    }
    return BenchmarkResult(name="raw_motion", metrics=metrics)


def benchmark_prediction_latency(
    scenario: ScenarioSpec,
    *,
    horizons: Sequence[int] = (10, 30, 90),
    resolution: float = 16.0,
    collision: str = "circle",
    warmup: int | None = None,
    repeat: int = 5,
) -> BenchmarkResult:
    """Latency of the future-prediction stack, per horizon.

    Three separately-measured stages, because they have very different hardware
    profiles and the CPU/FPGA split depends on which dominates:

    ``ballistic``   closed-form forecast over every bullet (pure arithmetic)
    ``rollout``     clone + H real simulation steps (the expensive reference)
    ``danger``      rasterise the rollout into an H x rows x cols risk grid
    """
    world = _fresh_world(scenario, collision=collision, warmup_steps=warmup)
    snapshot = world.get_state()
    predictor = BallisticPredictor()
    rows: list[dict[str, Any]] = []
    for horizon in horizons:
        h = int(horizon)
        ballistic = timeit(
            lambda: predictor.predict(snapshot, h, scenario.dt), repeat=repeat, warmup=1
        )
        states = world.simulate_future(0, horizon=h, include_initial=True)

        def roll() -> Any:
            return world.simulate_future(0, horizon=h, include_initial=False)

        rollout = timeit(roll, repeat=max(2, repeat // 2), warmup=1)
        danger = timeit(
            lambda: compute_danger_field(states, resolution=resolution),
            repeat=max(2, repeat // 2),
            warmup=1,
        )
        rows.append(
            {
                "horizon": h,
                "horizon_ms": h * scenario.dt * 1e3,
                "bullets": snapshot.bullet_count,
                "ballistic_ms": ballistic.mean * 1e3,
                "rollout_ms": rollout.mean * 1e3,
                "danger_ms": danger.mean * 1e3,
                "ballistic_us_per_bullet": (ballistic.mean * 1e6) / max(snapshot.bullet_count, 1),
            }
        )
    worst = max(rows, key=lambda r: r["rollout_ms"]) if rows else {}
    metrics: dict[str, Any] = {
        "bullets": snapshot.bullet_count,
        "collision_model": collision,
        "horizons": list(horizons),
    }
    if worst:
        metrics.update(
            {
                "ballistic_ms": worst["ballistic_ms"],
                "rollout_ms": worst["rollout_ms"],
                "danger_ms": worst["danger_ms"],
                "ballistic_us_per_bullet": worst["ballistic_us_per_bullet"],
            }
        )
    return BenchmarkResult(
        name=f"prediction_latency[{scenario.name}]",
        metrics=metrics,
        notes={"rows": rows},
    )


def benchmark_memory(
    scenario: ScenarioSpec,
    *,
    steps: int = 400,
    collision: str = "circle",
    warmup: int | None = None,
) -> BenchmarkResult:
    """Memory footprint and - most importantly - steady-state allocation.

    The kernel is designed to allocate *nothing* while stepping (preallocated
    SoA pools, no per-bullet objects).  ``alloc_per_step_bytes`` measures that
    claim directly with ``tracemalloc`` instead of asserting it in prose.
    """
    import resource
    import tracemalloc

    world = _fresh_world(scenario, collision=collision, warmup_steps=warmup)
    pool = world.state.bullets
    state_bytes = int(sum(a.nbytes for a in pool.data.values()) + pool.alive.nbytes)
    live = max(pool.count, 1)

    # measure a *frozen* world: no spawns, only integration + collision
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    for _ in range(int(steps)):
        world.step(0)
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    allocated = max(0, current - base)
    rss_kb = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    metrics = {
        "bullets_alive": int(pool.count),
        "bullet_capacity": int(pool.capacity),
        "state_bytes": state_bytes,
        "state_mib": state_bytes / (1024 * 1024),
        "bytes_per_bullet": state_bytes / live,
        "alloc_bytes": allocated,
        "alloc_per_step_bytes": allocated / max(int(steps), 1),
        "tracemalloc_peak_kib": peak / 1024,
        "process_rss_mib": rss_kb / 1024,
        "collision_model": collision,
    }
    return BenchmarkResult(name=f"memory[{scenario.name}]", metrics=metrics)


def benchmark_scene(level: Any = "medium", *, steps: int = 400, **kwargs: Any) -> BenchmarkResult:
    """End-to-end: build + simulate a named difficulty for ``steps`` steps."""
    spec = build_scenario(level, **kwargs).spec
    return benchmark_step_latency(spec, steps=steps)


__all__ = [
    "auto_warmup",
    "benchmark_step_latency",
    "benchmark_prediction_latency",
    "benchmark_memory",
    "benchmark_collision_throughput",
    "benchmark_scenario_generation",
    "benchmark_raw_motion",
    "benchmark_scene",
    "format_table",
    "measure_live_bullets",
]
