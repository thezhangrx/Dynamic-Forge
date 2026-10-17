#!/usr/bin/env python
"""**不猜**：直接打印"按某组 CLI 参数构建出来的场景里，每种障碍的速度"。

用途：验证 `--obstacle-speed` 到底有没有进到场景里。

    Env/bin/python core/board/tools/scene_speed.py play --input board
    Env/bin/python core/board/tools/scene_speed.py play --input board --obstacle-speed 200
    Env/bin/python core/board/tools/scene_speed.py board-control
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core"))

from bullet_sim import cli as B  # noqa: E402


def main(argv) -> int:
    if not argv:
        print(__doc__)
        return 2
    args = B.build_parser().parse_args(argv)
    spec = B._spec_from_args(args)
    print(f"命令: {' '.join(argv)}")
    print(f"  obstacle_speed 参数 = {getattr(args, 'obstacle_speed', None)}")
    rows = []
    for pat in getattr(spec, "patterns", ()):
        rows.append((str(getattr(pat, "shape", "?")),
                     str(getattr(pat, "params", {}).get("layout", "")),
                     float(getattr(pat, "speed", 0.0))))
    for shape, layout, speed in rows:
        print(f"  {shape:<6} {layout:<12} speed={speed:.2f}")
    walls = [r for r in rows if r[1] == "wall_gap"]
    print(f"  → 墙速: {walls[0][2]:.2f}" if walls else "  → 场景里没有墙")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
