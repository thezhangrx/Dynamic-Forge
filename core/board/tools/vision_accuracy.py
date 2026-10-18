#!/usr/bin/env python3
"""视觉精度对账：把板子的诊断日志和仿真真值按**同一时基**对齐后逐项量误差。

为什么需要它（以及为什么以前的数字是错的）：

1. 板子与主机的**系统时钟不同步** —— 实测板子慢 1.9 s 且会漂。以前用 `host_t`
   （板子墙钟）去和仿真的 `unix`（主机墙钟）配对，等于把所有几何误差都算成了
   "平台在被测那一刻已经移动了 1.9 s × 166 单位/秒 ≈ 300 单位"。
2. 诊断日志里 `t_ns` 是**板子单调时钟**（每帧采集时刻），和系统时钟无关。
   所以正确的对齐方式是：用 `t_ns` 和一个偏移量去拟合仿真时间 `t`
   （仿真 `t` 也是它自己的单调时间），偏移量由**角色轨迹**最小二乘/中位数搜出来。
   这既不依赖两块板子的墙钟，也不依赖 NTP。

对齐过程（只用独立于被测对象的量）：真值里的角色位置由"指令 → 物理积分"决定，
诊断里的 `player_field` 由"屏幕 → 相机 → 检测 → 单应"决定。两者只有时间偏移不同，
所以扫一遍偏移量，取**中位距离**最小的那个（中位数抗错检帧）。

报出的四类误差（都是**场地单位**，p50/p90/p95 + 偏置）：

    · 角色：`player_field` vs 真值 player
    · 缺口：`gap_units`  vs 真值缺口中心 x
    · 墙 y：`wall_y_units` vs 真值选中那道墙的 y（**同一道墙**，按最近的 y 配对）
    · 覆盖率：`field_valid` / 有墙帧 / 有角色帧 / 最长观测空档

用法：
    python3 core/board/tools/vision_accuracy.py --dir /tmp/crit4
    python3 core/board/tools/vision_accuracy.py --diag D --truth T --json out.json
"""

from __future__ import annotations

import argparse
import bisect
import json
import os
import statistics as st
import sys
from typing import Any


def load(path: str | None) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if not path:
        return out
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
                continue   # 被 -INT 掐断时最后一行可能是半条
    return out


def pct(v: list[float], q: float) -> float:
    if not v:
        return float("nan")
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def stats(v: list[float]) -> dict[str, float]:
    if not v:
        return {}
    return {
        "n": len(v),
        "median": st.median(v),
        "p90": pct(v, 0.90),
        "p95": pct(v, 0.95),
        "bias": st.median(v),           # 有符号：判断是不是系统性偏
        "within20": sum(1 for x in v if abs(x) <= 20.0) / len(v),
    }


def truth_index(truth: list[dict[str, Any]]) -> tuple[list[float], list[list[float]]]:
    ts = [float(r["t"]) for r in truth]
    ps = [[float(r["player"][0]), float(r["player"][1])] for r in truth]
    return ts, ps


def sample(ts: list[float], ps: list[list[float]], t: float):
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    if t1 - t0 > 0.5:      # 仿真时间有断档：不外推
        return None
    w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    a, b = ps[i - 1], ps[i]
    return [a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1])]


def estimate_offset(diag: list[dict[str, Any]], ts: list[float], ps: list[list[float]],
                    truth: list[dict[str, Any]]) -> tuple[float, float, int, str]:
    """估计"板子墙钟 → 仿真时间"的偏移。

    为什么必须有个偏移、而且**不能**直接拿两块板子的墙钟配对：

    * 板子的系统时钟和主机不同步（实测慢 1.9 s 且会漂）；
    * 仿真时间 `t` 比真实时间慢（实测 rtf≈0.75，负载越高越慢）。

    但仿真自己每步都记了墙钟 `unix`，所以 (unix → t) 是一张现成的表，
    **仿真变慢这件事已经被这张表吸收了**。于是只需要一个常数偏移：
    `sim_t = interp(host_t + skew)`。少一个自由度，也就少一个反推不准的量。

    偏移用**角色轨迹**搜出来（中位距离最小）：角色位置由"指令 → 物理积分"决定，
    视觉位置由"屏幕 → 相机 → 检测 → 单应"决定，两者只差时间。
    """
    obs = [(float(r["host_t"]), float(r["player_field"][0]), float(r["player_field"][1]))
           for r in diag
           if r.get("player_field_valid") and r.get("host_t")]
    if len(obs) < 8:
        raise SystemExit(
            "诊断日志里没有可用的角色观测（或缺少 host_t）—— "
            "需要 `cam_live.sh` 传 `--host-epoch`（本仓库的脚本都会传）")
    if len(obs) > 300:
        k = len(obs) / 300.0
        obs = [obs[int(i * k)] for i in range(300)]

    # (unix → t) 表：仿真真值自己给的映射
    pairs = sorted((float(r["unix"]), float(r["t"])) for r in truth if "unix" in r)
    us = [p[0] for p in pairs]
    tvals = [p[1] for p in pairs]

    def sim_t_of_wall(w: float):
        i = bisect.bisect_left(us, w)
        if i <= 0 or i >= len(us):
            return None
        w0, w1 = us[i - 1], us[i]
        if w1 - w0 > 0.5:
            return None
        f = (w - w0) / (w1 - w0) if w1 > w0 else 0.0
        return tvals[i - 1] + f * (tvals[i] - tvals[i - 1])

    def score(skew: float) -> tuple[float, int]:
        errs: list[float] = []
        for wt, x, y in obs:
            tv = sim_t_of_wall(wt + skew)
            if tv is None:
                continue
            p = sample(ts, ps, tv)
            if p is None:
                continue
            errs.append(abs(p[0] - x) + abs(p[1] - y))
        if len(errs) < max(8, len(obs) // 2):
            return (float("inf"), 0)
        return (st.median(errs), len(errs))

    best = (0.0, float("inf"), 0)
    # 粗扫 ±6 s，10 ms 一步（166 单位/秒 → 1.7 单位/步，足够细）
    for i in range(-600, 601):
        sk = i * 0.01
        m, cnt = score(sk)
        if m < best[1]:
            best = (sk, m, cnt)
    if best[1] == float("inf"):
        raise SystemExit("对齐失败：没有任何偏移量能给出有效样本")
    c0 = best[0]
    for i in range(-50, 51):
        sk = c0 + i * 0.001
        m, cnt = score(sk)
        if m < best[1]:
            best = (sk, m, cnt)
    return best[0], best[1], best[2], "host_t+skew → 真值 (unix→t) 插值"

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="视觉精度对账（按单调时钟对齐）")
    ap.add_argument("--dir", default=None, help="放 diag_*.jsonl / truth.jsonl 的目录")
    ap.add_argument("--diag", default=None)
    ap.add_argument("--truth", default=None)
    ap.add_argument("--json", default=None, help="把结果写成 JSON")
    ap.add_argument("--table", type=int, default=0, help="打印逐帧残差表的前 N 行")
    a = ap.parse_args(argv)

    dpath = a.diag or (os.path.join(a.dir, "diag_crit4.jsonl") if a.dir else None)
    tpath = a.truth or (os.path.join(a.dir, "truth.jsonl") if a.dir else None)
    diag = load(dpath)
    truth = load(tpath)
    if not diag:
        print(f"诊断日志为空：{dpath}", file=sys.stderr)
        return 2
    if not truth:
        print(f"真值日志为空：{tpath}（没有真值就只能看覆盖率，看不到误差）", file=sys.stderr)
        return 2

    ts, ps = truth_index(truth)
    skew, merr, nfit, how = estimate_offset(diag, ts, ps, truth)
    #: 对齐口径：`sim_t = interp(host_t + skew)`，插值表由真值自己的 (unix, t) 给。
    pairs_u = sorted((float(r["unix"]), float(r["t"])) for r in truth if "unix" in r)
    us = [p[0] for p in pairs_u]
    tv_by_unix = [p[1] for p in pairs_u]

    def wall_to_sim(w: float):
        i = bisect.bisect_left(us, w)
        if i <= 0 or i >= len(us):
            return None
        w0, w1 = us[i - 1], us[i]
        if w1 - w0 > 0.5:
            return None
        f = (w - w0) / (w1 - w0) if w1 > w0 else 0.0
        return tv_by_unix[i - 1] + f * (tv_by_unix[i] - tv_by_unix[i - 1])

    def sim_t(r: dict[str, Any]):
        return wall_to_sim(float(r["host_t"]) + skew) if r.get("host_t") else None

    # ---- 逐项误差 ----
    e_px: list[float] = []   # 角色 x 有符号误差
    e_py: list[float] = []
    e_gap: list[float] = []
    e_wy: list[float] = []
    d_px: list[float] = []   # 角色绝对距离（时间对齐后）
    d_traj: list[float] = []  # 角色离真值轨迹的最近距离（不含时间对齐残差）
    n_valid = n_wall = n_player = 0
    t_last_valid = None
    max_gap_ms = 0.0
    detail = []
    for r in diag:
        tv = sim_t(r)
        if tv is None:
            continue
        if r.get("field_valid"):
            n_valid += 1
            if t_last_valid is not None:
                max_gap_ms = max(max_gap_ms, (float(r["t_ns"]) - t_last_valid) * 1e-6)
            t_last_valid = float(r["t_ns"])
        if r.get("player_field_valid"):
            n_player += 1
            p = sample(ts, ps, tv)
            if p is not None:
                dx = r["player_field"][0] - p[0]
                dy = r["player_field"][1] - p[1]
                e_px.append(dx)
                e_py.append(dy)
                d_px.append((dx * dx + dy * dy) ** 0.5)
                # **离轨迹距离**：在真值轨迹上找最近点（±2 s 窗口），
                # 这个数不含"时间对齐残差"，只反映"视觉点是否落在平台真正走过
                # 的路径上" —— 平台在动的时候，它就是纯几何精度。
                lo = bisect.bisect_left(ts, tv - 2.0)
                hi = bisect.bisect_right(ts, tv + 2.0)
                best_d = min(((r["player_field"][0] - q[0]) ** 2 +
                              (r["player_field"][1] - q[1]) ** 2) ** 0.5
                             for q in ps[lo:hi]) if hi > lo else None
                if best_d is not None:
                    d_traj.append(best_d)
                detail.append(("player", tv, dx, dy))
        walls_true = None
        if r.get("walls"):
            n_wall += 1
        # 缺口 / 墙 y：只在两者都有值、且真值里确实有墙时比较
        if r.get("gap_units") is not None and r.get("wall_y_units") and r.get("walls"):
            tr = truth[min(len(truth) - 1, bisect.bisect_left(ts, tv))]
            obs = tr.get("obstacles") or []
            if obs:
                # 真值的墙：按 y 找最近的
                best = min(obs, key=lambda o: abs(float(o[1]) - float(r["wall_y_units"])))
                e_wy.append(float(r["wall_y_units"]) - float(best[1]))
                # 缺口中心：选中的那道墙的两段之间（只对 wall_with_gap 有意义）
                ys = [o for o in obs if abs(float(o[1]) - float(best[1])) < 1e-6]
                if len(ys) == 2:
                    right = [float(o[0]) + float(o[2]) for o in ys]   # cx + half_w
                    left = [float(o[0]) - float(o[2]) for o in ys]
                    g = 0.5 * (min(right) + max(left))                # 两段之间的中点
                    e_gap.append(float(r["gap_units"]) - g)
                    detail.append(("gap", tv, float(r["gap_units"]) - g, 0.0))

    out: dict[str, Any] = {
        "align": {"skew_s": skew, "median_abs_sum": merr, "n": nfit,
                  "note": how},
        "frames": len(diag),
        "clock_skew_s": skew,
        "align_note": how,
        "coverage": {
            "field_valid": n_valid / len(diag),
            "has_wall": n_wall / len(diag),
            "has_player": n_player / len(diag),
            "max_observe_gap_ms": max_gap_ms,
        },
    }
    if d_px:
        out["player"] = {
            "dist": stats(d_px),
            "dx": stats(e_px),
            "dy": stats(e_py),
        }
        if d_traj:
            out["player"]["off_trajectory"] = stats(d_traj)
        # **静止帧单独看**：平台不动时的误差才是"纯几何误差"；动着的误差里混着
        # 时间对齐残差（对齐偏移是全局最优，但每帧的流水线延迟有 ±60 ms 抖动，
        # 在 166 单位/秒下就是 ±10 单位）。这两个数要分开报，否则永远说不清
        # "到底是几何不准，还是时钟没对齐"。
        rest = []
        for kind, tv, dx, dy in detail:
            if kind != "player":
                continue
            a0 = sample(ts, ps, tv - 0.25)
            a1 = sample(ts, ps, tv + 0.25)
            if a0 is None or a1 is None:
                continue
            if ((a1[0] - a0[0]) ** 2 + (a1[1] - a0[1]) ** 2) ** 0.5 < 3.0:
                rest.append((dx * dx + dy * dy) ** 0.5)
        if rest:
            out["player"]["at_rest"] = stats(rest)
        out["player"]["note"] = "dist 含时间对齐残差；静止帧误差见 at_rest"
    if e_gap:
        out["gap"] = stats(e_gap)
    if e_wy:
        out["wall_y"] = stats(e_wy)

    print("=" * 72)
    print("视觉精度对账")
    print("=" * 72)
    print(f"  对齐：板子墙钟偏移 {skew:+.3f} s → 仿真时间，"
          f"中位 |Δx|+|Δy| = {merr:.1f} 单位，{nfit} 帧参与")
    if merr > 40:
        print("  ⚠ 对齐残差大：这一轮的角色可能大量靠预测/遮挡，结论要谨慎")
    print(f"  时钟：板子墙钟比主机 {'快' if skew > 0 else '慢'} {abs(skew):.2f} s "
          f"（含固定流水线延迟；这就是不能拿 host_t 直接配对的原因）")
    c = out["coverage"]
    print(f"  帧数 {len(diag)}  几何有效 {c['field_valid']*100:.1f}%  "
          f"有墙 {c['has_wall']*100:.1f}%  有角色 {c['has_player']*100:.1f}%  "
          f"最长观测空档 {c['max_observe_gap_ms']:.0f} ms")
    for key, label in (("player", "角色距离"), ("gap", "缺口 x"), ("wall_y", "墙 y")):
        s = out.get(key)
        if not s:
            continue
        s = s["dist"] if key == "player" else s
        print(f"  {label}: |err| p50 {abs(s['median']):.1f}  p90 {s['p90']:.1f}  "
              f"p95 {s['p95']:.1f}  偏置 {s['bias']:+.1f}  ≤20 单位 {s['within20']*100:.0f}%  (n={s['n']})")
    if out.get("player", {}).get("off_trajectory"):
        ot = out["player"]["off_trajectory"]
        print(f"    离轨迹距离（动起来的纯几何误差）: p50 {ot['median']:.1f}  p90 {ot['p90']:.1f}  "
              f"p95 {ot['p95']:.1f}  ≤20 单位 {ot['within20']*100:.0f}%  (n={ot['n']})")
    if out.get("player", {}).get("at_rest"):
        ar = out["player"]["at_rest"]
        print(f"    静止帧（平台不动，纯几何误差）: p50 {ar['median']:.1f}  p90 {ar['p90']:.1f}  "
              f"p95 {ar['p95']:.1f}  ≤20 单位 {ar['within20']*100:.0f}%  (n={ar['n']})")
    if "dx" in out.get("player", {}):
        sx, sy = out["player"]["dx"], out["player"]["dy"]
        print(f"    分轴偏置：dx {sx['bias']:+.1f}（p90 {sx['p90']:.1f}）  "
              f"dy {sy['bias']:+.1f}（p90 {sy['p90']:.1f}）")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        print(f"  结果写入 {a.json}")
    if a.table:
        for kind, tv, dx, dy in detail[: a.table]:
            print(f"    {kind:7s} t={tv:7.2f}  dx={dx:+7.1f} dy={dy:+7.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
