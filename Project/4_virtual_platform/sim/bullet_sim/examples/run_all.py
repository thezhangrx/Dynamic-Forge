"""Run every example in sequence (smoke test for the whole surface).

    python bullet_sim/examples/run_all.py
"""

from __future__ import annotations

import importlib
import io
import sys
import traceback
from contextlib import redirect_stdout

EXAMPLES = [
    "01_minimal_world",
    "02_patterns_and_scenarios",
    "03_headless_dataset",
    "04_external_controller",
    "05_future_prediction",
    "06_benchmark",
    "07_hardware_protocol",
    "08_render_debug",
    "09_input_modes",
    "10_cpu_fpga_loop",
    "11_obstacle_environment",
]

# arguments for examples that need a mode
ARGS = {"08_render_debug": ["ascii"]}


def main() -> int:
    failures = 0
    for name in EXAMPLES:
        sys.argv = [name, *ARGS.get(name, [])]
        module = importlib.import_module(name)
        buffer = io.StringIO()
        print(f"=== {name} " + "=" * (60 - len(name)))
        try:
            with redirect_stdout(buffer):
                module.main()
            out = buffer.getvalue()
            lines = out.strip().splitlines()
            print("\n".join(lines[:8]))
            if len(lines) > 8:
                print(f"    ... ({len(lines) - 8} more lines)")
        except Exception:
            failures += 1
            print("FAILED")
            traceback.print_exc()
        print()
    print(f"{len(EXAMPLES) - failures}/{len(EXAMPLES)} examples ran")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
