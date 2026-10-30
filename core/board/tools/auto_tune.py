#!/usr/bin/env python3
"""auto_tune.py —— 用**测出来的噪声**反推决策阈值，不再手写魔法数字。

输入：一次标定抓帧目录（含 board_vt.jsonl + board_events.jsonl）
输出：data/fpga/params.json

推导规则（k 都是保守取值，宁可多停车一次）：
  σ_vy      同一道墙在静止 40 帧里的行抖动（px）
  σ_gap     缺口宽度误差的 1σ（px）
  row_tol   = clamp(4·σ_vy, 6, 40)          墙行归并容差
  tol_vy    = clamp(3·σ_vy, 6, 30)          多帧一致容差
  consensus = clamp(ceil((σ_vy/1.0)^2), 4, 12)   使中位标准误 <1 px 的最小帧数
  inflate   = clamp(ceil(2·σ_gap), 4, 30)   缺口侧安全余量
  settle    = max(0.6, 3·step_ms/1000)      step_ms 由 --step-ms 给（阶跃响应实测），缺省沿用 0.65

用法：
  python3 core/board/tools/auto_tune.py <run_dir> [--step-ms 400]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/fpga/params.json"


def load(run: Path):
    bv = [json.loads(l) for l in open(run / "board_vt.jsonl")]
    ev = [json.loads(l) for l in (run / "board_events.jsonl").read_text(errors="ignore").splitlines()
          if '"vt_tick"' in l]
    py_of = {e["frame"]: e.get("py_used") for e in ev if e.get("py_used") is not None}
    calib = {}
    p = ROOT / "data/fpga/calib_y.json"
    if p.exists():
        calib = json.loads(p.read_text())
    return bv, py_of, calib


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="+")
    ap.add_argument("--step-ms", type=float, default=None)
    a = ap.parse_args()

    bv, py_of, cal = [], {}, {}
    for d in a.run:
        b, p, c = load(Path(d))
        bv += b; py_of.update(p); cal = cal or c
    sc = float(cal.get("scale", 1.0)) or 1.0
    bi = float(cal.get("bias", 0.0))

    # 把每帧的墙行映射到"世界 y"，按最近真值行聚类，统计同一行的抖动
    rows = []
    for r in bv:
        py = py_of.get(r["frame"])
        if py is None:
            continue
        for w in (r.get("walls") or []):
            rows.append((py - 240.0) + sc * float(w["y"]) + bi)

    if len(rows) < 16:
        print("样本太少（%d < 16），无法统计噪声" % len(rows))
        return 2

    worlds = []
    for r in bv:
        py = py_of.get(r["frame"])
        if py is None:
            continue
        for w in (r.get("walls") or []):
            worlds.append(((py - 240.0) + sc * float(w["y"]) + bi, w["gr"] - w["gl"]))
    ys = np.array([w[0] for w in worlds])
    ws = np.array([w[1] for w in worlds])

    # 用 20 px 网格聚类（场景墙间距 140，不会串）
    keys = np.round(ys / 20.0) * 20.0
    sig = []
    for k in np.unique(keys):
        v = ys[keys == k]
        if len(v) >= 8:
            sig.append(np.std(v))
    sigma_vy = float(np.median(sig)) if sig else float(np.std(ys))
    sigma_gap = float(np.std(ws)) if len(ws) > 8 else 4.0

    row_tol = float(min(40, max(6, 4 * sigma_vy)))
    tol_vy = float(min(30, max(6, 3 * sigma_vy)))
    consensus = int(min(12, max(4, math.ceil((sigma_vy / 1.0) ** 2))))
    inflate = int(min(30, max(4, math.ceil(2 * sigma_gap))))
    settle = 0.65 if a.step_ms is None else max(0.6, 3 * a.step_ms / 1000.0)

    # 起步窗口：第一帧"同时有墙+角色"出现的时刻（12 fps）→ 再加 1 s 余量
    warmup = 4.0
    for i, r in enumerate(bv):
        if r.get("walls") and r.get("player"):
            warmup = max(4.0, i / 12.0 + 1.0)
            break

    out = {
        "sigma_vy": round(sigma_vy, 2),
        "sigma_gap": round(sigma_gap, 2),
        "row_tol": row_tol, "tol_vy": tol_vy, "consensus_frames": consensus,
        "inflate": inflate, "settle": round(settle, 2), "warmup_s": round(warmup, 2),
        "samples": int(len(ys)), "source_run": ";".join(str(d) for d in a.run),
    }
    OUT.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print("σ_vy=%.2f px  σ_gap=%.2f px  样本 %d" % (sigma_vy, sigma_gap, len(ys)))
    print("推导：row_tol=%.0f  tol_vy=%.0f  consensus=%d  inflate=%d  settle=%.2fs  warmup=%.1fs"
          % (row_tol, tol_vy, consensus, inflate, settle, warmup))
    print("已写", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
