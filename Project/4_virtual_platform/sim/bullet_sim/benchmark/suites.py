"""Standard benchmark suites, including the high-density stress ladder."""

from __future__ import annotations

import time
from typing import Any, Callable, Mapping, Sequence


from bullet_sim.benchmark.metrics import format_table
from bullet_sim.benchmark.runner import (
    benchmark_collision_throughput,
    benchmark_memory,
    benchmark_prediction_latency,
    benchmark_raw_motion,
    benchmark_scenario_generation,
    benchmark_step_latency,
)
from bullet_sim.scenarios.complexity import BULLET_COUNT_LADDER
from bullet_sim.scenarios.presets import stress

#: Bullet counts the platform must handle (specification §15):
#: 100 / 500 / 1000 / 2000 / 5000 / 10000 dynamic objects.
DEFAULT_LADDER: tuple[int, ...] = BULLET_COUNT_LADDER


def bullet_ladder_suite(
    counts: Sequence[int] = DEFAULT_LADDER,
    *,
    steps: int = 300,
    seed: int = 0,
    collision: str = "circle",
    collision_kwargs: Mapping[str, Any] | None = None,
    include_build: bool = True,
    verbose: bool = False,
    progress: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Run the full stress ladder and return a structured report."""
    log = progress or (print if verbose else (lambda _m: None))
    rows: list[dict[str, Any]] = []
    started = time.time()
    for count in counts:
        log(f"[ladder] building scene for {count} bullets ...")
        t0 = time.perf_counter()
        spec = stress(int(count), seed=seed)
        build_s = time.perf_counter() - t0
        result = benchmark_step_latency(
            spec,
            steps=steps,
            collision=collision,
        )
        m = result.metrics
        row = {
            "target_bullets": int(count),
            "scene_peak_bullets": m["scene_peak_bullets"],
            "window_peak_bullets": m["window_peak_bullets"],
            "mean_bullets": round(float(m["mean_bullets"]), 1),
            "step_mean_ms": round(float(m["mean_ms"]), 4),
            "step_p99_ms": round(float(m["p99_ms"]), 4),
            "us_per_bullet_step": round(float(m["us_per_bullet_step"]), 5),
            "sim_fps": round(float(m["simulation_fps"]), 1),
            "realtime_factor": round(float(m["realtime_factor"]), 2),
            "scene_build_s": round(build_s, 3),
        }
        if include_build:
            coll = benchmark_collision_throughput(
                spec, queries=max(50, steps // 3), collision=collision
            )
            row["collision_mean_us"] = round(float(coll.metrics["mean_us"]), 3)
            row["collision_tests_per_s"] = round(
                float(coll.metrics["tests_per_second"]), 0
            )
        rows.append(row)
        log(
            f"[ladder] {count:>5} bullets -> scene peak {row['scene_peak_bullets']:>5}, "
            f"window mean {row['mean_bullets']:>7.1f}, "
            f"{row['sim_fps']:>8.1f} sim FPS, {row['realtime_factor']:>6.2f}x realtime"
        )
    return {
        "kind": "bullet_ladder",
        "counts": [int(c) for c in counts],
        "steps_per_scene": int(steps),
        "seed": int(seed),
        "collision": collision,
        "rows": rows,
        "wall_time_s": round(time.time() - started, 3),
    }


def difficulty_suite(
    levels: Sequence[str] = ("easy", "medium", "hard", "extreme"),
    *,
    steps: int = 300,
    seed: int = 0,
    collision: str = "circle",
) -> dict[str, Any]:
    """Same measurement across the four qualitative difficulty presets."""
    from bullet_sim.scenarios.presets import scenario_for_level

    rows: list[dict[str, Any]] = []
    for level in levels:
        spec = scenario_for_level(level, seed=seed)
        res = benchmark_step_latency(spec, steps=steps, collision=collision)
        m = res.metrics
        rows.append(
            {
                "level": level,
                "target_bullets": spec.meta.get("target_bullet_count"),
                "scene_peak_bullets": m["scene_peak_bullets"],
                "mean_bullets": round(float(m["mean_bullets"]), 1),
                "step_mean_ms": round(float(m["mean_ms"]), 4),
                "sim_fps": round(float(m["simulation_fps"]), 1),
                "realtime_factor": round(float(m["realtime_factor"]), 2),
                "duration_s": spec.duration,
            }
        )
    return {"kind": "difficulty", "rows": rows, "collision": collision, "seed": int(seed)}


def collision_backend_suite(
    bullet_count: int = 2000, *, players: Sequence[int] = (1, 8, 64), seed: int = 0
) -> dict[str, Any]:
    """Compare collision backends / batch widths - drives the FPGA offload decision."""
    spec = stress(int(bullet_count), seed=seed)
    rows: list[dict[str, Any]] = []
    for model in ("circle", "grid"):
        for p in players:
            if p > 1 and model == "circle":
                pass  # CircleCollision also supports query_many
            res = benchmark_collision_throughput(spec, collision=model, players=int(p))
            m = res.metrics
            rows.append(
                {
                    "model": model,
                    "players": int(p),
                    "bullets": m["bullets"],
                    "query_mean_us": round(float(m["mean_us"]), 3),
                    "tests_per_s": round(float(m["tests_per_second"]), 0),
                }
            )
    return {"kind": "collision_backends", "bullet_count": int(bullet_count), "rows": rows}


def prediction_suite(
    counts: Sequence[int] = (500, 2000, 10000),
    *,
    seed: int = 0,
    horizons: Sequence[int] = (10, 30, 90),
    resolve_uncalibrated: bool = True,
) -> dict[str, Any]:
    """Prediction-stack latency at increasing densities (specification §15)."""
    rows: list[dict[str, Any]] = []
    for count in counts:
        spec = stress(int(count), seed=seed)
        result = benchmark_prediction_latency(spec, horizons=horizons)
        row = {"target_bullets": int(count), "bullets": result.metrics["bullets"]}
        row.update(
            {
                "ballistic_ms": round(float(result.metrics["ballistic_ms"]), 4),
                "rollout_ms": round(float(result.metrics["rollout_ms"]), 4),
                "danger_ms": round(float(result.metrics["danger_ms"]), 4),
                "ballistic_us_per_bullet": round(
                    float(result.metrics["ballistic_us_per_bullet"]), 4
                ),
            }
        )
        rows.append(row)
    return {
        "kind": "prediction",
        "rows": rows,
        "horizons": list(horizons),
        "counts": [int(c) for c in counts],
    }


def memory_suite(
    counts: Sequence[int] = DEFAULT_LADDER,
    *,
    seed: int = 0,
    steps: int = 200,
) -> dict[str, Any]:
    """Memory footprint + steady-state allocation per bullet count."""
    rows: list[dict[str, Any]] = []
    for count in counts:
        spec = stress(int(count), seed=seed)
        metrics = benchmark_memory(spec, steps=steps).metrics
        rows.append(
            {
                "target_bullets": int(count),
                "bullets_alive": metrics["bullets_alive"],
                "bullet_capacity": metrics["bullet_capacity"],
                "state_mib": round(float(metrics["state_mib"]), 4),
                "bytes_per_bullet": round(float(metrics["bytes_per_bullet"]), 1),
                "alloc_per_step_bytes": round(float(metrics["alloc_per_step_bytes"]), 2),
                "process_rss_mib": round(float(metrics["process_rss_mib"]), 1),
            }
        )
    return {"kind": "memory", "rows": rows, "steps": int(steps)}


def full_suite(
    *,
    counts: Sequence[int] = DEFAULT_LADDER,
    steps: int = 300,
    seed: int = 0,
    verbose: bool = True,
    progress: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Everything: ladder + difficulty + raw motion + scenario generation."""
    log = progress or (print if verbose else (lambda _m: None))
    report: dict[str, Any] = {"seed": int(seed), "steps": int(steps)}
    report["bullet_ladder"] = bullet_ladder_suite(
        counts, steps=steps, seed=seed, verbose=verbose, progress=progress
    )
    report["difficulty"] = difficulty_suite(steps=steps, seed=seed)
    report["raw_motion"] = benchmark_raw_motion(max(counts)).to_dict()
    report["scenario_generation"] = benchmark_scenario_generation(
        "hard", seed=seed
    ).to_dict()
    report["collision_backends"] = collision_backend_suite(
        bullet_count=min(2000, max(counts)), seed=seed
    )
    report["prediction"] = prediction_suite(seed=seed)
    report["memory"] = memory_suite(counts, seed=seed)
    log("[suite] done")
    return report


def format_report(report: Mapping[str, Any]) -> str:
    """Render a suite report as a readable text table."""
    kind = report.get("kind")
    if report.get("rows") and kind not in ("bullet_ladder", "difficulty", "collision_backends"):
        return format_table(list(report["rows"]))
    if kind == "bullet_ladder":
        return format_table(list(report["rows"]))
    if kind == "difficulty":
        return format_table(list(report["rows"]))
    if kind == "collision_backends":
        return format_table(list(report["rows"]))
    parts = []
    for key in ("bullet_ladder", "difficulty", "collision_backends",
                "prediction", "memory"):
        if key in report:
            parts.append(f"== {key} ==")
            parts.append(format_report(report[key]))
    if "raw_motion" in report:
        parts.append("== raw_motion ==")
        parts.append(format_table([{**report["raw_motion"]["metrics"]}]))
    if "scenario_generation" in report:
        parts.append("== scenario_generation ==")
        parts.append(format_table([{**report["scenario_generation"]["metrics"]}]))
    return "\n".join(parts)


def report_rows(report: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Flatten a suite report to rows for CSV/JSON output."""
    if "rows" in report:
        return list(report["rows"])
    return []


__all__ = [
    "DEFAULT_LADDER",
    "bullet_ladder_suite",
    "prediction_suite",
    "memory_suite",
    "difficulty_suite",
    "collision_backend_suite",
    "full_suite",
    "format_report",
    "report_rows",
]
