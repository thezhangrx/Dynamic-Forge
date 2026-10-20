#!/usr/bin/env python3
"""VT / 视觉 ↔ 仿真真值 对账（v2 静态墙）。

回答三个问题：
  1. **墙的转换**：VT 冻结墙的 y / 缺口左右端 vs 真值墙（静态，与时间无关）；
  2. **角色的转换**：视觉绝对 `player_field`、VT `vt_xy` vs 真值角色；
  3. **几何是否可见**：`field_valid` / `player_field_valid` 的比例与原因。

用法（先跑一轮相机闭环，拿到两块日志）：
    python3 core/board/tools/reconcile_vt_truth.py \
        --diag /tmp/vt_diag.jsonl --truth /tmp/vt_truth.jsonl \
        --out /tmp/vt_reconcile.json

真值日志由仿真 `--state-log` 产出；diag 由板子 `--diag-log` 产出。
**静态平台（角色停在出生点）时不需要时间对齐**，直接比坐标最干净。
"""
from __future__ import annotations
import argparse, json, math, statistics as st


def load(path):
    out = []
    try:
        f = open(path, encoding="utf-8")
    except FileNotFoundError:
        return out
    with f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def truth_walls(truth):
    """从真值帧里抽墙：同一 y 的两段拼成一道墙 → (y, gl, gr)。"""
    if not truth:
        return []
    # 用最后一帧（墙是静态的；最后一帧最全）
    r = truth[-1]
    segs = []
    for o in r.get("obstacles", []):
        x, y, hw, hh = o[0], o[1], o[2], o[3]
        if hw <= hh:
            continue
        segs.append((y, x - hw, x + hw))
    segs.sort()
    walls = []
    i = 0
    while i < len(segs):
        y = segs[i][0]
        l = segs[i][1]; rgt = segs[i][2]
        j = i + 1
        while j < len(segs) and abs(segs[j][0] - y) < 1e-6:
            l = min(l, segs[j][1]); rgt = max(rgt, segs[j][2]); j += 1
        walls.append((y, l, rgt))
        i = j
    return walls


def pct(v, q):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag", required=True)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--out", default="/tmp/vt_reconcile.json")
    a = ap.parse_args()

    diag = load(a.diag)
    truth = load(a.truth)
    if not diag:
        print("diag 为空（板子没出帧，或 --diag-log 没写）")
        return 2
    if not truth:
        print("truth 为空（仿真 --state-log 没写）")
        return 2

    tw = truth_walls(truth)
    tp = truth[-1].get("player", [0, 0, 0])
    # diag 有效性
    n_valid_field = sum(1 for r in diag if r.get("field_valid"))
    n_valid_player = sum(1 for r in diag if r.get("player_field_valid"))
    n_locked = sum(1 for r in diag if r.get("vt_locked"))
    reasons = {}
    for r in diag:
        key = (r.get("field_src", "?"), r.get("geo_why", "?"))
        reasons[key] = reasons.get(key, 0) + 1

    # 墙：每个 VT 墙找最近的 truth 墙
    vt_walls = []
    for r in diag:
        if r.get("vt_walls"):
            vt_walls = r["vt_walls"]  # 用最后一帧有墙的
    wall_err = []
    wall_rows = []
    for v in vt_walls:
        if not tw:
            break
        t = min(tw, key=lambda w: abs(w[0] - v["y"]))
        dy = v["y"] - t[0]
        dgl = v["gl"] - t[1]
        dgr = v["gr"] - t[2]
        dcen = 0.5 * (v["gl"] + v["gr"]) - 0.5 * (t[1] + t[2])
        wall_rows.append({"vt_y": v["y"], "truth_y": t[0], "dy": dy,
                          "vt_gap": [v["gl"], v["gr"]], "truth_gap": [t[1], t[2]],
                          "d_left": dgl, "d_right": dgr, "d_center": dcen})
        wall_err.append(abs(dy)); wall_err.append(abs(dgl)); wall_err.append(abs(dgr))

    # 角色：视觉绝对与 VT（不依赖时间，静态平台最干净；动态时取最后一帧）
    last = diag[-1]
    vis = last.get("player_field") if last.get("player_field_valid") else None
    vt = last.get("vt_xy") if last.get("vt_locked") else None
    if last.get("player_field_valid"):
        # 最后一帧的有效值
        for r in reversed(diag):
            if r.get("player_field_valid"):
                vis = r["player_field"]; break
    if last.get("vt_locked"):
        for r in reversed(diag):
            if r.get("vt_locked"):
                vt = r["vt_xy"]; break
    res = {
        "diag_frames": len(diag),
        "truth_frames": len(truth),
        "field_valid": [n_valid_field, len(diag)],
        "player_field_valid": [n_valid_player, len(diag)],
        "vt_locked": [n_locked, len(diag)],
        "field_reasons": [{"src": k[0], "why": k[1], "n": v} for k, v in
                          sorted(reasons.items(), key=lambda kv: -kv[1])[:8]],
        "truth_player": tp[:2],
        "vision_player": vis,
        "vt_player": vt,
        "vision_dxdy": None if vis is None else [vis[0] - tp[0], vis[1] - tp[1]],
        "vt_dxdy": None if vt is None else [vt[0] - tp[0], vt[1] - tp[1]],
        "truth_walls": [list(w) for w in tw],
        "vt_walls_last": vt_walls,
        "wall_rows": wall_rows,
        "wall_abs_err_p50": pct(wall_err, 0.50),
        "wall_abs_err_p95": pct(wall_err, 0.95),
    }
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"帧：diag {len(diag)} / truth {len(truth)}")
    print(f"field_valid {n_valid_field}/{len(diag)}   player_field_valid {n_valid_player}/{len(diag)}"
          f"   vt_locked {n_locked}/{len(diag)}")
    print("几何失败原因：")
    for k, v in sorted(reasons.items(), key=lambda kv: -kv[1])[:5]:
        print(f"  {k[0]:8s} {k[1]:16s} {v}")
    if vis is not None:
        print(f"视觉角色 {vis}  vs 真值 {tp[:2]}  → Δ=({vis[0]-tp[0]:+.1f},{vis[1]-tp[1]:+.1f})")
    if vt is not None:
        print(f"VT 角色   {vt}  vs 真值 {tp[:2]}  → Δ=({vt[0]-tp[0]:+.1f},{vt[1]-tp[1]:+.1f})")
    if wall_rows:
        print(f"墙的 |Δ| p50 {pct(wall_err,0.5):.1f} / p95 {pct(wall_err,0.95):.1f} 场地单位")
        for w in wall_rows:
            print(f"  VT y={w['vt_y']:.1f} truth={w['truth_y']:.1f} Δy={w['dy']:+.1f} "
                  f"缺口中心 Δ={w['d_center']:+.1f}（左 Δ={w['d_left']:+.1f} 右 Δ={w['d_right']:+.1f}）")
    print(f"→ {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
