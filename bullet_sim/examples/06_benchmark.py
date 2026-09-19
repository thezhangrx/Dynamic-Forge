"""Example 6 - performance benchmarks and the high-density stress ladder.

    python bullet_sim/examples/06_benchmark.py            # quick ladder
    python bullet_sim/examples/06_benchmark.py --full     # full ladder + raw motion
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import json
import sys
import time
from pathlib import Path

from bullet_sim.benchmark.metrics import format_table
from bullet_sim.benchmark.runner import benchmark_raw_motion, benchmark_scenario_generation
from bullet_sim.benchmark.suites import bullet_ladder_suite, collision_backend_suite, format_report


def main() -> None:
    full = "--full" in sys.argv
    counts = (50, 100, 200, 500, 1000, 2000, 5000) if full else (50, 200, 1000)
    steps = 300 if full else 150

    t0 = time.perf_counter()
    report = {
        "bullet_ladder": bullet_ladder_suite(counts, steps=steps, seed=0, verbose=True),
    }
    report["collision_backends"] = collision_backend_suite(bullet_count=2000, seed=0)
    if full:
        report["raw_motion"] = benchmark_raw_motion(5000).to_dict()
        report["scenario_generation"] = benchmark_scenario_generation("hard", seed=0).to_dict()

    print()
    print(format_report(report))
    out = Path("benchmarks_output")
    out.mkdir(exist_ok=True)
    path = out / ("full_report.json" if full else "quick_report.json")
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nwrote {path}  (total {time.perf_counter() - t0:.1f}s)")


if __name__ == "__main__":
    main()
