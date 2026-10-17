#!/usr/bin/env python
"""**绕过视觉的闭环中继**：仿真真值 → 决策 → 回写动作。

链路（没有任何像素）：

    bullet_sim --state-log /tmp/gt.jsonl     ← 每帧真值（场地单位）
        │  本脚本按**字节偏移**增量读新行
        ▼
    /tmp/gt_decide（板子同一套 gap_avoid::plan）
        │  每帧一行：frame N tier=... cmd=(vx,vy) gap=... rects=...
        ▼
    /tmp/mv.txt（追加 `@DBG <决策>` + `@MOVE <n>`）
        │  `tail -f /tmp/mv.txt` 保证仿真的 stdin 永不 EOF
        ▼
    仿真（--stdin）执行动作

用法：
    touch /tmp/mv.txt
    ( tail -f /tmp/mv.txt | Env/bin/python -m bullet_sim board-control \
        --stdin --nostop --duration 60 --fullscreen --state-log /tmp/gt.jsonl ) &
    echo '@MOVE 0' >> /tmp/mv.txt        # 先喂一行，仿真才会开始跑
    Env/bin/python core/board/tools/gt_loop.py --state /tmp/gt.jsonl --out /tmp/mv.txt
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path


#: 平台 9 个离散动作（`bullet_sim.core.actions.DISCRETE_ACTION_NAMES`）：
#:   0 stay, 1 left, 2 right, 3 up, 4 down, 5 up_left, 6 up_right, 7 down_left, 8 down_right
#: 注意世界的 **+y 是 up**（不是屏幕向下），所以方位角直接用 `atan2(vy, vx)`。
_ACTION_BY_OCTANT = {0: 2, 45: 6, 90: 3, 135: 5, 180: 1, 225: 7, 270: 4, 315: 8}


def action_from_cmd(vx: float, vy: float, dead: float = 0.25) -> int:
    """(vx, vy) → 9 个离散动作之一。

    踩过的坑：一开始按 `atan2(-vy, vx)` 再 `%8+1` 映射，等价于把"下"当成"上"、
    把动作表当成逆时针排 —— 结果纯真值闭环里平台一直被指向错误的方位
    （决策算的是"向下穿门"，平台却在往西走）。
    """
    if abs(vx) < dead and abs(vy) < dead:
        return 0
    ang = math.degrees(math.atan2(vy, vx)) % 360.0
    octant = int(round(ang / 45.0)) % 8 * 45
    return _ACTION_BY_OCTANT[octant]


def target_point(obj: dict):
    """真值里"角色正对着的那堵墙"的缺口中心（场地单位）。

    只用于**显示**（HUD 准星），不参与决策 —— 决策仍完全由 `gt_decision_cli` 算。
    """
    obs = [o for o in (obj.get("obstacles") or []) if len(o) >= 5 and float(o[2]) > float(o[3])]
    if not obs:
        return None
    pl = obj.get("player") or [0.0, 0.0, 10.0]
    bands = {}
    for x, y, hw, hh, _rot in obs:
        bands.setdefault(round(y), []).append((x - hw, x + hw))
    best = None
    for key, segs in bands.items():
        segs.sort()
        for a, b in zip(segs, segs[1:]):
            if b[0] - a[1] <= 1.0:
                continue
            cx = 0.5 * (a[1] + b[0])
            d = abs(cx - pl[0]) + abs(key - pl[1])
            if best is None or d < best[0]:
                best = (d, cx, float(key))
    return None if best is None else (best[1], best[2])


def gt_line(obj: dict) -> str:
    """一行真值 → `@GT px py pr n x y hw hh rot ...`（只取矩形障碍）。"""
    pl = obj.get("player") or [0.0, 0.0, 10.0]
    obs = [o for o in (obj.get("obstacles") or [])
           if len(o) >= 5 and float(o[2]) > float(o[3])]
    parts = [f"@GT {pl[0]:.2f} {pl[1]:.2f} {pl[2]:.2f} {len(obs)}"]
    for x, y, hw, hh, rot in obs:
        parts.append(f"{x:.2f} {y:.2f} {hw:.2f} {hh:.2f} {rot:.3f}")
    return " ".join(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", default="/tmp/gt.jsonl")
    ap.add_argument("--out", default="/tmp/mv.txt")
    ap.add_argument("--decide", default="/tmp/gt_decide")
    ap.add_argument("--ssh-key", default=None,
                    help="给了就在**板子上**跑决策（真·板子决策）："
                         "ssh -i KEY root@192.168.137.242 /root/tools/gt_decide_board")
    ap.add_argument("--ssh-host", default="root@192.168.137.242")
    ap.add_argument("--max-seconds", type=float, default=120.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    if not Path(args.decide).exists():
        print(f"找不到 {args.decide}。先编译：\n"
              "  g++ -std=c++17 -O2 -I core/cpu/cpp -o /tmp/gt_decide \\\n"
              "      core/board/tools/gt_decision_cli.cpp core/cpu/cpp/gap_avoid.cpp",
              file=sys.stderr)
        return 1

    if args.ssh_key:
        # **板子自己决策**：把 @GT 通过 ssh 喂给板子上的同一个可执行文件，
        # 决策在板子上算（不是 PC 上的副本），输出再回传给仿真。
        cmd = ["ssh", "-i", args.ssh_key, "-o", "StrictHostKeyChecking=no",
               "-o", "UserKnownHostsFile=/dev/null", "-o", "ConnectTimeout=8",
               args.ssh_host, "/root/tools/gt_decide_board"]
        print(f"决策在板子上跑：{args.ssh_host}:/root/tools/gt_decide_board", flush=True)
    else:
        cmd = [args.decide]
    decide = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, text=True, bufsize=1)
    state_path, out_path = Path(args.state), Path(args.out)
    state_path.touch(exist_ok=True)
    out_path.touch(exist_ok=True)
    pos = 0
    n = 0
    t0 = time.time()
    while time.time() - t0 < args.max_seconds:
        if args.max_frames and n >= args.max_frames:
            break
        try:
            data = state_path.read_text(encoding="utf-8", errors="replace")
        except FileNotFoundError:
            time.sleep(0.05)
            continue
        new = data[pos:]
        if not new.strip():
            time.sleep(0.02)
            continue
        pos = len(data)
        for line in new.splitlines():
            try:
                obj = json.loads(line)
            except Exception:
                continue
            decide.stdin.write(gt_line(obj) + "\n")
            decide.stdin.flush()
            dec = decide.stdout.readline().strip()
            if not dec:
                continue
            vx = vy = 0.0
            for tok in dec.split():
                if tok.startswith("cmd=("):
                    try:
                        vx, vy = (float(t) for t in tok[5:-1].split(","))
                    except Exception:
                        vx = vy = 0.0
            act = action_from_cmd(vx, vy)
            with out_path.open("a", encoding="utf-8") as fh:
                tgt = None
                for tok in dec.split():
                    if tok.startswith("aim=("):
                        try:
                            ax, ay = (float(v) for v in tok[5:-1].split(","))
                            if ax == ax and ay == ay:      # 不是 NaN
                                tgt = (ax, ay)
                        except Exception:
                            tgt = None
                if tgt is not None:
                    fh.write(f"@TGT {tgt[0]:.1f} {tgt[1]:.1f}\n")
                fh.write(f"@DBG {dec} | 动作 {act}\n")
                fh.write(f"@MOVE {act}\n")
            n += 1
            if not args.quiet and (n <= 5 or n % 50 == 0):
                print(f"#{n} {dec} → 动作 {act}", flush=True)
    print(f"中继结束：共发出 {n} 帧决策", flush=True)
    decide.stdin.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
