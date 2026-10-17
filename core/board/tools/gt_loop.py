#!/usr/bin/env python
"""**绕过视觉的闭环**：仿真真值 → 决策 → @MOVE 回仿真。

用户的要求：先把"视觉"和"决策"分开验。这条链路里没有任何像素：

    bullet_sim --state-log state.jsonl（真值，场地单位）
         ↓ 本脚本逐帧读新行，转成 @GT 行
    gt_decision_cli（板子同一套 gap_avoid）
         ↓ 每帧一行决策
    @MOVE 写入仿真的 stdin（FIFO）

用法：
    mkfifo /tmp/move.fifo
    Env/bin/python -m bullet_sim board-control --stdin --nostop --duration 60 \
        --fullscreen --state-log /tmp/gt.jsonl < /tmp/move.fifo &
    Env/bin/python core/board/tools/gt_loop.py /tmp/gt.jsonl /tmp/move.fifo
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

#: 9 个离散动作（与平台动作码一致）：0 停，1..8 依次
def action_from_cmd(vx: float, vy: float, dead: float = 0.25) -> int:
    import math
    if abs(vx) < dead and abs(vy) < dead:
        return 0
    ang = math.degrees(math.atan2(-vy, vx)) % 360.0     # 屏幕上 y 向下
    return int(round(ang / 45.0)) % 8 + 1


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__)
        return 2
    state_path, fifo_path = Path(argv[0]), argv[1]
    decide = Path("/tmp/gt_decide")
    if not decide.exists():
        print("先编译：g++ -std=c++17 -O2 -I core/cpu/cpp -o /tmp/gt_decide "
              "core/board/tools/gt_decision_cli.cpp core/cpu/cpp/gap_avoid.cpp")
        return 1
    proc = subprocess.Popen([str(decide)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            text=True, bufsize=1)
    fifo = open(fifo_path, "w", buffering=1)
    sent = 0
    t0 = time.time()
    while time.time() - t0 < 600:
        try:
            lines = state_path.read_text(encoding="utf-8").splitlines()
        except FileNotFoundError:
            time.sleep(0.05)
            continue
        while sent < len(lines):
            try:
                d = json.loads(lines[sent])
            except Exception:
                sent += 1
                continue
            sent += 1
            pl = d.get("player") or [0, 0, 10]
            obs = [o for o in (d.get("obstacles") or []) if len(o) >= 5 and o[2] > o[3]]
            parts = [f"@GT {pl[0]:.2f} {pl[1]:.2f} {pl[2]:.2f} {len(obs)}"]
            for x, y, hw, hh, rot in obs:
                parts.append(f"{x:.2f} {y:.2f} {hw:.2f} {hh:.2f} {rot:.3f}")
            proc.stdin.write(" ".join(parts) + "\n")
            proc.stdin.flush()
            out = proc.stdout.readline().strip()
            if out:
                # 决策文本也打出来（用户要"看得见决策"）
                sys.stdout.write(out + "\n")
                sys.stdout.flush()
                vx = vy = 0.0
                for tok in out.split():
                    if tok.startswith("cmd=("):
                        vx, vy = (float(t) for t in tok[5:-1].split(","))
                fifo.write(f"@MOVE {action_from_cmd(vx, vy)}\n")
                fifo.flush()
        time.sleep(0.02)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
