#!/usr/bin/env python3
"""虚拟真值策略的**离线 A/B**：只在初始化阶段用视觉，之后全靠积分。

策略（对齐 §14）：
  · 角色：前 1.0 s（或前 30 帧）视觉样本取稳健均值当 p0；之后 p += u(@MOVE) × v_board × dt，
    视觉不再参与；只有连续 5 帧偏差 >60 单位才重锚。
  · 墙：视觉每报出一道新墙（y 与已有 track 差 >40 单位）→ 取前 3 帧的 y/gap/vy 中位数固定，
    之后 y -= vy·dt；视觉不再更新它。

对照基线：同一条链路上"每帧都用视觉位置"（就是现在板子上的做法）。

用法：
    python3 virt_core.py --diag /tmp/crit4/diag_crit4.jsonl --truth /tmp/crit4/truth.jsonl \
            --speed 156.6 [--rate 1.09]
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics as st
import sys
from collections import defaultdict

DIRS = {0: (0.0, 0.0), 1: (-1.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 1.0), 4: (0.0, -1.0),
        5: (-0.70710678, 0.70710678), 6: (0.70710678, 0.70710678),
        7: (-0.70710678, -0.70710678), 8: (0.70710678, -0.70710678)}


def load(p):
    out = []
    for line in open(p, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(q * (len(s) - 1)))]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag", required=True)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--speed", type=float, default=156.6, help="单位/板秒（boot_cal 实测）")
    ap.add_argument("--rate", type=float, default=1.09, help="板子时间基速率（板秒/真实秒）")
    ap.add_argument("--init-s", type=float, default=1.0)
    ap.add_argument("--wall-init-n", type=int, default=3)
    ap.add_argument("--wall-thick", type=float, default=8.0)
    ap.add_argument("--every", type=float, default=0.0,
                    help="每隔多少**板秒**输入一次视觉真值来修正角色（0 = 永不）")
    ap.add_argument("--blend", type=float, default=1.0,
                    help="修正强度：1.0 = 硬覆盖，0.3 = 只拉 30%（软校正）")
    a = ap.parse_args(argv)

    diag = load(a.diag)
    truth = load(a.truth)
    if not diag or not truth:
        print("缺日志", file=sys.stderr)
        return 2

    # ---- 时钟对齐：用真值自己的 (unix→t) 表 + 搜一个 skew（与 vision_accuracy 同口径）----
    us = sorted((float(r["unix"]), float(r["t"])) for r in truth if "unix" in r)
    uu = [x[0] for x in us]
    tt = [x[1] for x in us]

    def t_at_wall(w):
        i = bisect.bisect_left(uu, w)
        if i <= 0 or i >= len(uu) or uu[i] - uu[i - 1] > 0.5:
            return None
        f = (w - uu[i - 1]) / (uu[i] - uu[i - 1])
        return tt[i - 1] + f * (tt[i] - tt[i - 1])

    ts = [float(r["t"]) for r in truth]
    ps = [[float(r["player"][0]), float(r["player"][1])] for r in truth]

    def truth_pos(t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(ts):
            return None
        f = (t - ts[i - 1]) / max(1e-9, ts[i] - ts[i - 1])
        return [ps[i - 1][0] + f * (ps[i][0] - ps[i - 1][0]),
                ps[i - 1][1] + f * (ps[i][1] - ps[i - 1][1])]

    obs = [(float(r["host_t"]), r) for r in diag if r.get("host_t")]
    if len(obs) < 20:
        print("缺 host_t", file=sys.stderr)
        return 2

    def score(skew):
        errs = []
        for w, r in obs[::3]:
            t = t_at_wall(w + skew)
            if t is None or not r.get("player_field_valid"):
                continue
            p = truth_pos(t)
            if p:
                errs.append(abs(p[0] - r["player_field"][0]) + abs(p[1] - r["player_field"][1]))
        return (st.median(errs), len(errs)) if len(errs) > 20 else (1e18, 0)

    best = min(((score(k * 0.02), k * 0.02) for k in range(-100, 101)))
    skew = best[1]
    print(f"对齐：skew {skew:+.2f} s（中位 |Δ| {best[0][0]:.1f}，{best[0][1]} 帧）")

    # ---- 积分器 ----
    t0 = obs[0][0]
    init = []
    p = [0.0, 0.0]
    locked = False
    last_seed = 0.0
    t_prev = None
    reinit = 0
    bad_run = 0
    err_virt, err_vis = [], []
    walls = []            # [{y, gap, vy, buf:[(t,y)], frozen}]
    wall_err = []
    for w, r in obs:
        # 板秒：host_t 是板子墙钟（速率 1.09），除以 rate 换成真实秒再乘回——
        # 积分用板秒，所以直接用 host_t 的差即可（rate 只影响速度单位的口径）。
        if t_prev is None:
            t_prev = w
        dt = max(0.0, min(0.5, w - t_prev))
        t_prev = w
        act = int(r.get("action") or 0)
        ux, uy = DIRS.get(act, (0.0, 0.0))

        if not locked:
            if r.get("player_field_valid") and (w - t0) <= a.init_s:
                init.append((r["player_field"][0], r["player_field"][1]))
            if (w - t0) > a.init_s:
                if init:
                    p = [st.median([q[0] for q in init]), st.median([q[1] for q in init])]
                    locked = True
                else:
                    # 没拿到样本：退回视觉（并标记）
                    if r.get("player_field_valid"):
                        p = [r["player_field"][0], r["player_field"][1]]
        else:
            p[0] += ux * a.speed * dt
            p[1] += uy * a.speed * dt
            p[0] = max(1.0, min(639.0, p[0]))
            p[1] = max(1.0, min(479.0, p[1]))
            # **周期性输入真值**：隔 `--every` 板秒把视觉位置融进来一次
            if a.every > 0 and (w - last_seed) >= a.every and r.get("player_field_valid"):
                last_seed = w
                p[0] = (1.0 - a.blend) * p[0] + a.blend * r["player_field"][0]
                p[1] = (1.0 - a.blend) * p[1] + a.blend * r["player_field"][1]
            # 重锚例外
            if r.get("player_field_valid"):
                d = math.hypot(r["player_field"][0] - p[0], r["player_field"][1] - p[1])
                bad_run = bad_run + 1 if d > 60.0 else 0
                if bad_run >= 5:
                    p = [r["player_field"][0], r["player_field"][1]]
                    bad_run = 0
                    reinit += 1

        t = t_at_wall(w + skew)
        if t is not None:
            tp = truth_pos(t)
            if tp:
                if locked:
                    err_virt.append(math.hypot(p[0] - tp[0], p[1] - tp[1]))
                if r.get("player_field_valid"):
                    err_vis.append(math.hypot(r["player_field"][0] - tp[0],
                                              r["player_field"][1] - tp[1]))

        # ---- 墙：新建/初始化/冻结/积分 ----
        wy = r.get("wall_y_units")
        gx = r.get("gap_units")
        if r.get("walls") and wy:
            tr = None
            for wl in walls:
                if abs(wl["y"] - wy) <= 40.0:
                    tr = wl
                    break
            if tr is None:
                walls.append({"y": wy, "gap": gx, "vy": 128.0, "buf": [(w, wy)], "frozen": False})
                tr = walls[-1]
            elif not tr["frozen"]:
                tr["buf"].append((w, wy))
                if len(tr["buf"]) >= a.wall_init_n:
                    ys = [y for _, y in tr["buf"]]
                    tr["y"] = st.median(ys)
                    if gx:
                        tr["gap"] = gx
                    # 斜率：板秒 → 仿真秒要乘 rtf≈0.75；这里直接从样本估，不做假设
                    dtb = tr["buf"][-1][0] - tr["buf"][0][0]
                    if dtb > 0.05:
                        tr["vy"] = (ys[-1] - ys[0]) / dtb      # 单位/板秒
                    tr["frozen"] = True
        for wl in walls:
            if wl["frozen"]:
                # 墙往下压 → 样本斜率本身就是负的（单位/板秒），所以是 **加**：
                # 写成 `-=` 会把"向下"翻成"向上"，实测墙 y 误差直接到 689 单位。
                wl["y"] += wl["vy"] * dt
                # 与真值比较：找真值里 y 最近的墙
                if r.get("walls") and t is not None:
                    k = min(range(len(truth)), key=lambda i: abs(truth[i]["t"] - t))
                    obs_t = truth[k].get("obstacles") or []
                    if obs_t:
                        ty = min(obs_t, key=lambda o: abs(float(o[1]) - wl["y"]))[1]
                        wall_err.append(abs(wl["y"] - float(ty)))

    def rep(name, v):
        if v:
            print(f"  {name:14s} n={len(v):5d}  p50 {st.median(v):6.1f}  p90 {pct(v, .9):6.1f}  "
                  f"p95 {pct(v, .95):6.1f}  ≤20 单位 {100*sum(1 for x in v if x <= 20)/len(v):5.1f}%")

    print("\n角色位置误差（离真值，含时间对齐残差）")
    rep("视觉（基线）", err_vis)
    rep("虚拟真值(积分)", err_virt)
    print(f"  重锚次数 {reinit}（触发 = 积分与现实分叉 >60 单位 × 5 帧）")
    print("\n墙 y 误差（已冻结的 track 与真值最近墙比较）")
    rep("虚拟真值(积分)", wall_err)
    print(f"  墙 track 数 {len(walls)}，其中已冻结 {sum(1 for w in walls if w['frozen'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
