#!/usr/bin/env python
"""**逐帧对照**：真值 vs 板子识别 vs 板子指令。

不做"多少轮"的统计，直接排成一张表看每一帧：

    帧 | 真值角色(x,y) | 识别角色(x,y) | Δ | 真值缺口(cx,w) | 识别缺口(cx,w) | Δ | 指令

对齐方式：板子的 `[注入#k f=<几何帧号>]` 与仿真状态日志的 `board_actions`
（仿真在该帧时已收到的板子行数）配对 —— 两者同源，不依赖时钟。

用法：
    Env/bin/python core/board/tools/compare_frames.py state.jsonl moves.txt [--rows 30]
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

INJ = re.compile(r"\[注入#(?P<idx>[0-9]+) f=(?P<seq>[0-9]+)\] 角色=\((?P<px>[-+0-9.]+),(?P<py>[-+0-9.]+)\) "
                 r"r=(?P<pr>[0-9.]+) \| 门=\((?P<cx>[-+0-9.]+),(?P<cy>[-+0-9.]+)\) 宽=(?P<w>[0-9.]+)")
MOVE = re.compile(r"@MOVE ([0-8])")


def truth_frames(path: Path) -> list:
    out = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            d = json.loads(line)
        except Exception:
            continue
        walls = [o for o in (d.get("obstacles") or []) if len(o) >= 5 and o[2] > o[3]]
        bands = {}
        for x, y, hw, hh, _ in walls:
            key = round(y)
            bands.setdefault(key, []).append((x - hw, x + hw))
        gaps = []
        for key, segs in bands.items():
            segs.sort()
            for a, b in zip(segs, segs[1:]):
                if b[0] - a[1] > 1.0:
                    gaps.append((0.5 * (a[1] + b[0]), b[0] - a[1], float(key)))
        out.append({"ba": int(d.get("board_actions", -1)),
                    "player": d.get("player") or [0, 0, 0], "gaps": gaps})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("state")
    ap.add_argument("board")
    ap.add_argument("--rows", type=int, default=24)
    args = ap.parse_args(argv)
    truth = truth_frames(Path(args.state))
    text = Path(args.board).read_text(encoding="utf-8", errors="replace")
    inj = [m.groupdict() for m in INJ.finditer(text)]
    moves = [int(m.group(1)) for m in MOVE.finditer(text)]
    if not truth or not inj:
        print(f"数据不足：真值 {len(truth)} 帧，注入 {len(inj)} 条")
        return 1
    cand = [t for t in truth if t["ba"] >= 0]

    def nearest(seq: int):
        # 板子的几何帧号与"仿真已收到的行数"不是严格相等（差一两帧），取最近的
        return min(cand, key=lambda t: abs(t["ba"] - seq)) if cand else None
    print(f"{'帧':>5} {'真值角色':>13} {'识别角色':>13} {'Δ角色':>6} "
          f"{'真值缺口(cx,w,y)':>20} {'识别缺口(cx,w)':>15} {'Δ缺口':>7} 指令")
    rows = 0
    for r in inj:
        seq = int(r["seq"])
        t = nearest(seq)
        if t is None or abs(t["ba"] - seq) > 4:
            continue
        px, py = float(r["px"]), float(r["py"])
        dpx, dpy = px - t["player"][0], py - t["player"][1]
        if t["gaps"]:
            g = min(t["gaps"], key=lambda q: abs(q[0] - float(r["cx"])))
            gx, gw, gy = g
            dg = float(r["cx"]) - gx
            gt = f"{gx:5.0f},{gw:4.0f},{gy:4.0f}"
        else:
            gt, dg = "(无缺口)", float("nan")
        cmd = moves[seq] if 0 <= seq < len(moves) else -1
        print(f"{seq:>5} ({t['player'][0]:5.0f},{t['player'][1]:4.0f}) "
              f"({px:5.0f},{py:4.0f}) ({dpx:+4.0f},{dpy:+4.0f}) "
              f"{gt:>20} ({float(r['cx']):5.0f},{float(r['w']):4.0f}) {dg:+7.0f} {cmd}")
        rows += 1
        if rows >= args.rows:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
