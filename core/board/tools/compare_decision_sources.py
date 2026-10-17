#!/usr/bin/env python
"""同一批真值帧，**板子决策** vs **本地决策** 逐帧对比。

用户的思路：把两边在同一时刻的输出摆在一起比，差异立刻暴露问题在哪。

板子侧来自中继记录的 `@DBG ...`（那是板子 `gt_decide_board` 的原样输出）；
本地侧现在用同一份 `gt_decision_cli` 在 PC 上重跑同一条 @GT 序列。

用法：
    Env/bin/python core/board/tools/compare_decision_sources.py /tmp/gt.jsonl /tmp/mv.txt
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

INJ = re.compile(r"\[注入|@DBG frame\s+(\d+)\s+tier=(\S+)\s+cmd=\(([-+0-9.]+),([-+0-9.]+)\)")
DBG = re.compile(r"^@DBG frame\s+(\d+)\s+tier=(\S+)\s+cmd=\(([-+0-9.]+),([-+0-9.]+)\)")


def gt_line(obj: dict) -> str:
    pl = obj.get("player") or [0.0, 0.0, 10.0]
    obs = [o for o in (obj.get("obstacles") or []) if len(o) >= 5 and float(o[2]) > float(o[3])]
    parts = [f"@GT {pl[0]:.2f} {pl[1]:.2f} {pl[2]:.2f} {len(obs)}"]
    for x, y, hw, hh, rot in obs:
        parts.append(f"{x:.2f} {y:.2f} {hw:.2f} {hh:.2f} {rot:.3f}")
    return " ".join(parts)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__)
        return 2
    state, moves = Path(argv[0]), Path(argv[1])
    board = {}
    for line in moves.read_text(encoding="utf-8", errors="replace").splitlines():
        m = DBG.match(line.strip())
        if m:
            board[int(m.group(1))] = (m.group(2), float(m.group(3)), float(m.group(4)))
    objs = []
    for line in state.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            objs.append(json.loads(line))
        except Exception:
            continue
    lines = [gt_line(o) for o in objs]
    local = subprocess.run(["/tmp/gt_decide"], input="\n".join(lines) + "\n",
                           capture_output=True, text=True).stdout.splitlines()
    DEC = re.compile(r"frame\s+(\d+)\s+tier=(\S+)\s+cmd=\(([-+0-9.]+),([-+0-9.]+)\)")
    diff = 0
    same = 0
    shown = 0
    for out in local:
        m = DEC.search(out)
        if not m:
            continue
        frame, tier = int(m.group(1)), m.group(2)
        vx, vy = float(m.group(3)), float(m.group(4))
        b = board.get(frame)
        if b is None:
            continue
        if b[0] != tier or abs(b[1] - vx) > 0.02 or abs(b[2] - vy) > 0.02:
            diff += 1
            if shown < 5:
                shown += 1
                print(f"帧 {frame}: 板子 {b}  vs  本地 {(tier, vx, vy)}")
        else:
            same += 1
    print(f"对比完成：一致 {same} 帧，不一致 {diff} 帧（共 {len(board)} 条板子决策）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
