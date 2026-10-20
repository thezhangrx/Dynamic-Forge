#!/usr/bin/env python3
"""检查每次 @STATE 里的墙行，是否都在角色越过前足够早地进了板子。

用法：Env/bin/python core/board/tools/check_wall_coverage.py <run_dir> [scenario.json]
"""
import glob
import json
import sys

run = sys.argv[1] if len(sys.argv) > 1 else sorted(
    glob.glob("/home/zhang/FPGA/data/fpga/loop_logs/full_loop_*_step4*"))[-1]
cfg_path = sys.argv[2] if len(sys.argv) > 2 else "/home/zhang/FPGA/configs/20_walls_alt_24_dy140.json"
cfg = json.load(open(cfg_path))
truth = [round(y) for y in cfg["meta"]["walls_y"]]
first_seen = {}
cross = {}
for line in open(run + "/state.txt"):
    if not line.startswith("@STATE"):
        continue
    p = line.split()
    try:
        py = float(p[3])
        n = int(p[5])
    except (IndexError, ValueError):
        continue
    try:
        vals = [float(v) for v in p[6:6 + 5 * n]]
    except ValueError:
        continue
    if len(vals) < 5 * n:
        continue
    rows = {vals[5 * i + 1] for i in range(n)}
    for t in truth:
        if t in first_seen:
            continue
        for r in rows:
            if abs(r - t) <= 20:
                first_seen[t] = py
                break
    for t in truth:
        if py >= t:
            cross.setdefault(t, py)
print(f"{run}")
bad = 0
for t in truth:
    fs = first_seen.get(t)
    if fs is None:
        print(f"  row {t:6d}: 从未进入 @STATE")
        bad += 1
        continue
    lead = t - fs
    flag = "OK " if lead >= 60 else "LATE"
    if lead < 60:
        bad += 1
    print(f"  row {t:6d}: 进 @STATE 时 py={fs:7.1f}  提前 {lead:7.1f} 单位  {flag}")
print(f"late/missing rows: {bad}/{len(truth)}")
