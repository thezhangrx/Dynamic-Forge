#!/usr/bin/env python3
"""开场标定分析：从 `--cal` 那一轮的数据里量出三个数。

    python3 boot_cal.py --diag /tmp/cal/diag.jsonl --truth /tmp/cal/truth.jsonl \
            --out /tmp/cal/calib.json

为什么要单独一个脚本，而不是"看着日志估"：虚拟真值（把 `@MOVE` 积分成平台
位置）只依赖**三个**输入 —— 初始位置、平台速度、指令→动作延迟。这三样只要有一个
是拍的，积分就会系统性偏离真值，然后"用视觉去修积分"看起来永远像视觉坏了。
标定窗口（板子 `--cal N` + 仿真 `--obstacle-delay N`）就是为量这三个数设计的：

    · 停住那 1 s            → 初始位置（多帧稳健均值 + 离散度）
    · 8 个 2 s 匀速段       → 速度（沿指令方向的位移/时间）
    · 段起点→位移起点        → 指令延迟（直线外推回"位移为零"的时刻）

两条独立证据：

    · **视觉侧**（`--diag`）：板子自己认出来的角色位置 → 量的是"整条视觉链路"的
      延迟与速度，含相机曝光/传输/处理；
    · **真值侧**（`--truth`，仿真 `--state-log`）：仿真物理里的角色位置与 `applied`
      速度 → 量的是"平台本身"的速度与延迟。两者相减就是**视觉链路的滞后**。

对账口径：诊断日志的 `host_t` 是**墙钟**（`--host-epoch` 传入），`cal` 是标定窗口内
的秒数，于是 `墙钟 = host_t` 与 `cal` 同源，`unix - (host_t - cal)` 就是常数 ——
用它能把真值行映射到"标定窗口内的第几秒"，进而按指令段分组。
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics as st
import sys
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# 动作号 → 单位方向（**世界坐标，+y 向上**，和 `bullet_sim/core/actions.py` 一致）
# ---------------------------------------------------------------------------
DIRS = {
    0: (0.0, 0.0),
    1: (-1.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 1.0), 4: (0.0, -1.0),
    5: (-0.70710678, 0.70710678), 6: (0.70710678, 0.70710678),
    7: (-0.70710678, -0.70710678), 8: (0.70710678, -0.70710678),
}
AXIS = {1: "x", 2: "x", 3: "y", 4: "y", 5: "diag", 6: "diag", 7: "diag", 8: "diag"}


def segments(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """从诊断日志里**切出指令段**：相邻且 `cal_step` 相同的帧属于同一段。

    为什么从日志切、而不是在脚本里再抄一份动作表：板子的标定计划是**运行时**按
    视觉位置生成的（要不要回中、回几段，取决于角色出生在哪），抄一份表就必然对不上。
    日志里有 `cal_step`（段号、单调递增）和 `action`（那一段实际在发什么），
    这就是唯一真相 —— 分析脚本只认它。
    """
    out: list[dict[str, Any]] = []
    for r in rows:
        step = int(r.get("cal_step", -1))
        if step < 0:
            continue
        t = float(r["cal"])
        if not out or out[-1]["step"] != step:
            out.append({"step": step, "action": int(r.get("action", 0)),
                        "t0": t, "t1": t, "host_t0": float(r.get("host_t") or 0.0),
                        "rows": []})
        out[-1]["t1"] = t
        out[-1]["rows"].append(r)
    return out


def load(path: str | None) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if not path:
        return out
    p = Path(path)
    if not p.exists():
        return out
    with p.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue   # 被 -INT 掐断时最后一行可能是半条
    return out


def _median(v: list[float]) -> float:
    return st.median(v) if v else 0.0


def robust_line(ts: list[float], vs: list[float]) -> tuple[float, float, int]:
    """稳健直线拟合 ``v = a + b·t``，返回 ``(a, b, 用掉的点数)``。

    先最小二乘，再按 MAD 剔离群点重拟合一次 —— 视觉偶尔会给一个跳帧的错位置，
    不剔掉的话它会把斜率拉歪（这正是"取平均值"和"稳健平均"的区别）。
    """
    if len(ts) < 3:
        return 0.0, 0.0, len(ts)
    n = len(ts)
    mt, mv = sum(ts) / n, sum(vs) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    if sxx <= 1e-12:
        return mv, 0.0, n
    b = sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / sxx
    a = mv - b * mt
    resid = sorted(abs(v - (a + b * t)) for t, v in zip(ts, vs))
    mad = resid[len(resid) // 2] * 1.4826
    if mad < 1e-9:
        return a, b, n
    keep = [(t, v) for t, v in zip(ts, vs) if abs(v - (a + b * t)) <= 3.0 * mad]
    if len(keep) < 3:
        return a, b, n
    ts2 = [t for t, _ in keep]
    vs2 = [v for _, v in keep]
    n2 = len(ts2)
    mt2, mv2 = sum(ts2) / n2, sum(vs2) / n2
    sxx2 = sum((t - mt2) ** 2 for t in ts2)
    if sxx2 <= 1e-12:
        return a, b, n
    b2 = sum((t - mt2) * (v - mv2) for t, v in zip(ts2, vs2)) / sxx2
    a2 = mv2 - b2 * mt2
    return a2, b2, n2


# ---------------------------------------------------------------------------
# 视觉侧：用板子自己认出来的角色位置
# ---------------------------------------------------------------------------
def analyze_vision(diag: list[dict[str, Any]]) -> dict[str, Any]:
    rows = [r for r in diag if isinstance(r.get("cal"), (int, float)) and r["cal"] >= 0]
    if not rows:
        return {"ok": False, "why": "诊断日志里没有 cal>=0 的帧（板子没开 --cal？）"}

    # -- 初始位置：前 1 s 停住那段的稳健均值 --
    init = [r for r in rows if r["cal"] < 1.0 and r.get("player_field_valid")]
    ix = [r["player_field"][0] for r in init]
    iy = [r["player_field"][1] for r in init]
    initial = [_median(ix), _median(iy)]
    # 用 MAD 剔除后再取均值：比"直接平均"更不容易被一两帧错位置带偏
    def trimmed(v: list[float]) -> tuple[float, float]:
        if len(v) < 3:
            return (_median(v), 0.0)
        m = _median(v)
        mad = _median([abs(x - m) for x in v]) * 1.4826
        keep = [x for x in v if mad < 1e-9 or abs(x - m) <= 3.0 * mad] or v
        return (sum(keep) / len(keep), mad)
    mx, sdx = trimmed(ix)
    my, sdy = trimmed(iy)

    # -- 按段分析：速度 + 延迟（沿指令方向投影成标量后拟合）--
    seglist = segments(rows)
    detail: list[dict[str, Any]] = []
    speeds: list[float] = []
    delays: list[float] = []
    for g in seglist:
        act = g["action"]
        pts = []
        for r in g["rows"]:
            if not r.get("player_field_valid"):
                continue
            # x 轴用**采集时刻**：位置描述的是那一帧的画面，不是处理完那一刻。
            # 用处理时刻会把每帧的流水线年龄（实测 ~60 ms）算成延迟。
            age = float(r.get("frame_age_ms") or 0.0) * 1e-3
            pts.append((float(r["cal"]) - age, float(r["player_field"][0]),
                        float(r["player_field"][1])))
        if act == 0 or len(pts) < 5:
            continue
        ux, uy = DIRS.get(act, (0.0, 0.0))
        t0, dur = g["t0"], max(0.02, g["t1"] - g["t0"])
        # 只取**中段**：起步有延迟、收尾有减速，两端都不参与斜率
        mid = [p for p in pts if t0 + 0.25 * dur <= p[0] <= t0 + 0.85 * dur]
        if len(mid) < 4:
            continue
        ts = [p[0] for p in mid]
        ss = [p[1] * ux + p[2] * uy for p in mid]
        a_, b_, used = robust_line(ts, ss)
        if b_ == 0.0:
            continue
        speed = b_      # 投影到单位方向上的速度 = |v|
        speeds.append(speed)
        # 延迟：把拟合直线外推回"段起点那一刻的位置"，比段起点晚多少就是延迟
        p0 = (sum(p[1] for p in pts[:2]) / len(pts[:2])) * ux + \
             (sum(p[2] for p in pts[:2]) / len(pts[:2])) * uy
        t_go = t0 + (p0 - a_) / b_
        delay = t_go - t0
        if not (-0.5 <= delay <= 1.5):
            delay = float("nan")
        else:
            delays.append(delay)
        detail.append({
            "step": g["step"], "action": act, "axis": AXIS.get(act, "?"),
            "t0": round(t0, 3), "dur": round(dur, 3), "n_mid": used,
            "speed": round(speed, 1),
            "delay_s": (None if math.isnan(delay) else round(delay, 3)),
        })

    # -- 角度方向上的速度差点：对角若也接近同一模长，说明指令表没写错 --
    axis_speed: dict[str, float] = {}
    for d in detail:
        axis_speed.setdefault(d["axis"], []).append(d["speed"])   # type: ignore[arg-type]
    axis_speed = {k: round(_median(v), 1) for k, v in axis_speed.items()}

    # -- 整段路径拟合：速度 + 延迟一起解 -------------------------------
    # 段级拟合在**脉冲只有 0.8 s** 时不可靠：视觉看到的运动比指令晚 ~1 s（屏幕渲染 +
    # 相机 + 处理 + 仿真 0.75× 实时），中段窗口里经常装的是"上一段"的运动。
    # 换一个用满全部数据的估计：
    #   · 把指令方向按时间积分成 S(t) = ∫u(t)dt（"指令让你走了多远"）；
    #   · 观察到的位置应该等于 p0 + v·S(t − lag)；
    #   · 网格搜 lag（0~2 板秒），每个 lag 下用最小二乘解 (p0, v)，
    #     取残差最小的那个。v 就是"每板秒多少场地单位"，lag 就是端到端延迟。
    path = None
    try:
        cmd_pts = [(float(r["cal"]), DIRS.get(int(r.get("action", 0)), (0.0, 0.0)))
                   for r in rows]
        # S(t)：按 cal 轴累积；dt 用相邻帧差（板秒）
        sts: list[float] = [cmd_pts[0][0]]
        sx: list[float] = [0.0]
        sy: list[float] = [0.0]
        for i in range(1, len(cmd_pts)):
            t0, u0 = cmd_pts[i - 1]
            t1, u1 = cmd_pts[i]
            dt = max(0.0, t1 - t0)
            ux, uy = u1
            sx.append(sx[-1] + ux * dt)
            sy.append(sy[-1] + uy * dt)
            sts.append(t1)
        obs = [(float(r["cal"]) - float(r.get("frame_age_ms") or 0.0) * 1e-3,
                float(r["player_field"][0]), float(r["player_field"][1]))
               for r in rows if r.get("player_field_valid")]
        if len(obs) > 30:
            def S_at(t: float) -> tuple[float, float]:
                i = bisect.bisect_left(sts, t)
                i = min(max(i, 1), len(sts) - 1)
                t0, t1 = sts[i - 1], sts[i]
                f = 0.0 if t1 <= t0 else max(0.0, min(1.0, (t - t0) / (t1 - t0)))
                return (sx[i - 1] + f * (sx[i] - sx[i - 1]),
                        sy[i - 1] + f * (sy[i] - sy[i - 1]))
            best = None
            for k in range(0, 101):          # lag 0..2.0 板秒，步长 0.02
                lag = k * 0.02
                # 最小二乘解 p = p0 + v·S
                n = len(obs)
                mSx = mSy = mpX = mpY = 0.0
                ptss: list[tuple[float, float, float, float]] = []
                for t, x, y in obs:
                    a, b = S_at(t - lag)
                    ptss.append((a, b, x, y))
                    mSx += a; mSy += b; mpX += x; mpY += y
                mSx /= n; mSy /= n; mpX /= n; mpY /= n
                sxx = sxy = syy = sxpx = sypx = 0.0
                for a, b, x, y in ptss:
                    da, db = a - mSx, b - mSy
                    sxx += da * da; sxy += da * db; syy += db * db
                    sxpx += da * (x - mpX); sypx += db * (y - mpY)
                det = sxx * syy - sxy * sxy
                if abs(det) < 1e-9:
                    continue
                vx = (sxpx * syy - sypx * sxy) / det
                vy = (sypx * sxx - sxpx * sxy) / det
                v = 0.5 * (vx + vy)
                resid = []
                for a, b, x, y in ptss:
                    px = mpX + vx * (a - mSx)
                    py = mpY + vy * (b - mSy)
                    resid.append(math.hypot(x - px, y - py))
                r50 = _median(resid)
                if best is None or r50 < best[0]:
                    best = (r50, lag, v, vx, vy)
            if best is not None:
                path = {"speed": round(best[2], 1), "delay_s": round(best[1], 3),
                        "resid_median": round(best[0], 1),
                        "vx": round(best[3], 1), "vy": round(best[4], 1)}
    except Exception as exc:                     # 标定失败不能让整个脚本挂掉
        path = {"error": repr(exc)}

    return {
        "ok": bool(speeds),
        "path_fit": path,
        "n_frames": len(rows),
        "initial": [round(mx, 1), round(my, 1)],
        "initial_median": [round(initial[0], 1), round(initial[1], 1)],
        "initial_spread": [round(sdx, 1), round(sdy, 1)],
        "speed": round(_median(speeds), 1),
        "speed_spread": round((max(speeds) - min(speeds)) if speeds else 0.0, 1),
        "speed_by_axis": axis_speed,
        "delay_s": round(_median(delays), 3) if delays else None,
        "delay_s_spread": (round(max(delays) - min(delays), 3) if delays else None),
        "segments": detail,
    }


# ---------------------------------------------------------------------------
# 真值侧：用仿真的角色位置与 `applied`
# ---------------------------------------------------------------------------
def analyze_truth(truth: list[dict[str, Any]], diag: list[dict[str, Any]]) -> dict[str, Any]:
    """真值侧：平台**本身**的速度与延迟（不受视觉影响）。

    段边界从诊断日志切（板子那一侧的真实指令时序），真值只提供"角色实际怎么动"。
    两边的时间都用墙钟：`host_t`（板子）与 `unix`（仿真），前提是两块板子时钟对齐
    （`sync_clock.sh`；不对齐时本项目所有对账都用 `vision_accuracy.py` 按轨迹搜偏移）。
    """
    if not truth:
        return {"ok": False, "why": "没有真值日志（仿真没开 --state-log？）"}

    rows = [r for r in diag if isinstance(r.get("cal"), (int, float)) and r["cal"] >= 0]
    seglist = [g for g in segments(rows) if g["action"] != 0]
    if not seglist:
        return {"ok": False, "why": "诊断里没有非零指令段"}

    # 真值按墙钟索引
    ts_u = [float(r["unix"]) for r in truth]

    def truth_at(w: float):
        i = bisect.bisect_left(ts_u, w)
        if i <= 0 or i >= len(truth):
            return None
        return truth[i]

    # -- 速度：applied 非零且**段内中段**的 |Δplayer|/Δt（不夹边界钳位）--
    sp: list[float] = []
    for p, q in zip(truth, truth[1:]):
        ap = q.get("applied")
        if not ap or abs(ap[0]) + abs(ap[1]) < 1e-9:
            continue
        dt = float(q["unix"]) - float(p["unix"])
        if dt <= 1e-4:
            continue
        d = math.dist(p["player"][:2], q["player"][:2])
        # 钳位检测：位移明显小于"速度×时间"就说明撞到边界了，不参与标定
        sp.append(d / dt)
    speed = _median(sp) if sp else 0.0

    # -- 逐段的延迟与速度 --
    seg_delay: list[float] = []
    seg_speed: list[dict[str, Any]] = []
    for g in seglist:
        act = g["action"]
        ux, uy = DIRS.get(act, (0.0, 0.0))
        w0 = g["host_t0"]
        if not w0:
            continue
        # 段起点之后，真值里 applied 第一次真正变成这个方向
        on = None
        i = bisect.bisect_left(ts_u, w0)
        for j in range(max(0, i - 2), min(len(truth), i + 400)):
            ap = truth[j].get("applied") or [0.0, 0.0]
            if abs(ap[0] - ux) < 0.05 and abs(ap[1] - uy) < 0.05 and j > i - 1:
                on = float(truth[j]["unix"])
                break
        if on is not None:
            d = on - w0
            if -0.2 <= d <= 1.5:
                seg_delay.append(d)
        # 段内稳态速度（用仿真时间 `t`，不受墙钟抖动影响）
        steady = []
        for r in truth[max(0, i): min(len(truth), i + 400)]:
            tv = float(r["unix"])
            if w0 + 0.2 <= tv <= w0 + 0.55 and (abs((r.get("applied") or [0, 0])[0] - ux) < 0.05
                                               and abs((r.get("applied") or [0, 0])[1] - uy) < 0.05):
                steady.append(r)
        if len(steady) >= 3:
            ss = [float(r["player"][0]) * ux + float(r["player"][1]) * uy for r in steady]
            tt = [float(r["t"]) for r in steady]
            _a, b, used = robust_line(tt, ss)
            if b:
                seg_speed.append({"step": g["step"], "action": act,
                                  "axis": AXIS.get(act, "?"), "speed": round(b, 1), "n": used})

    axis_speed: dict[str, list[float]] = {}
    for d in seg_speed:
        axis_speed.setdefault(d["axis"], []).append(d["speed"])
    return {
        "ok": bool(seg_speed) or bool(sp),
        "n_rows": len(truth),
        "speed": round(speed, 1),
        "speed_spread": round((max(sp) - min(sp)) if sp else 0.0, 1),
        "speed_by_axis": {k: round(_median(v), 1) for k, v in axis_speed.items()},
        "delay_s": round(_median(seg_delay), 3) if seg_delay else None,
        "delay_s_spread": (round(max(seg_delay) - min(seg_delay), 3) if seg_delay else None),
        "n_delay_samples": len(seg_delay),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="开场标定：量平台速度、指令延迟、初始位置")
    ap.add_argument("--diag", required=True, help="板子逐帧诊断（含 cal/cal_step）")
    ap.add_argument("--truth", default=None, help="仿真真值（--state-log）")
    ap.add_argument("--out", default="/tmp/cal/calib.json", help="输出标定结果")
    ap.add_argument("--board-rate", type=float, default=1.0, metavar="R",
                    help="板子的时间基速率（板秒/真实秒）。这块板子实测 **1.09**："
                         "`sleep 20` 只花 18.35 真实秒，`CLOCK_MONOTONIC` 快 9%%。"
                         "给 1.09 会把速度换算成真实单位/秒并打印出来")
    a = ap.parse_args(argv)

    diag = load(a.diag)
    if not diag:
        print(f"诊断日志为空或不存在：{a.diag}", file=sys.stderr)
        return 2
    truth = load(a.truth)
    vis = analyze_vision(diag)
    tru = analyze_truth(truth, diag)
    rate = float(a.board_rate or 1.0)
    if rate <= 0.0:
        rate = 1.0

    print("=" * 70)
    print(f"开场标定结果（板子时间基速率 {rate:.4f} 板秒/真实秒）")
    print("=" * 70)
    if vis.get("ok"):
        print(f"  [视觉侧] 帧数 {vis['n_frames']}")
        print(f"    初始位置   {vis['initial']}   离散度 σ≈{vis['initial_spread']}（场地单位）")
        print(f"    平台速度   {vis['speed']:.1f} 单位/**板秒** = "
              f"{vis['speed'] / rate:.1f} 单位/**真实秒**   "
              f"（段间差 {vis['speed_spread']:.1f}，分方向 {vis['speed_by_axis']}）")
        if vis.get("path_fit") and "speed" in vis["path_fit"]:
            pf = vis["path_fit"]
            print(f"    路径拟合   速度 {pf['speed']:.1f} 单位/板秒 = "
                  f"{pf['speed'] / rate:.1f} 单位/真实秒，端到端延迟 {pf['delay_s']:.2f} 板秒，"
                  f"残差中位 {pf['resid_median']:.1f} 单位（vx {pf['vx']:.1f} / vy {pf['vy']:.1f}）")
        if vis.get("delay_s") is not None:
            print(f"    指令延迟   {vis['delay_s']:.3f} s     （段间差 {vis['delay_s_spread']}）")
        else:
            print("    指令延迟   段级估不出来（脉冲 0.8 s < 端到端延迟）—— 看上面的路径拟合")
    else:
        print(f"  [视觉侧] 不可用：{vis.get('why')}")
    if tru.get("ok"):
        print(f"  [真值侧] 行数 {tru['n_rows']}  帧对 {tru['n_delay_samples']}")
        print(f"    平台速度   {tru['speed']:.1f} 单位/真实秒（分方向 {tru['speed_by_axis']}；"
              f"契约 200 单位/**仿真秒** × 仿真实时率 ≈ 真实秒速度）")
        if tru.get("delay_s") is not None:
            print(f"    指令延迟   {tru['delay_s']:.3f} s")
        if vis.get("ok") and vis.get("delay_s") is not None and tru.get("delay_s") is not None:
            print(f"    → 视觉链路滞后 ≈ {vis['delay_s'] - tru['delay_s']:+.3f} s"
                  "（视觉延迟 − 真值延迟 = 相机+处理多出来的那段）")
    else:
        print(f"  [真值侧] 不可用：{tru.get('why')}")

    # 合并：**速度/延迟优先用真值**（它量的是平台本身）；真值不可用时退回视觉。
    # ⚠ 口径必须分清：
    #   · 视觉路径拟合给的是**每板秒多少场地单位** —— 虚拟真值在板子上积分就用它；
#     （换算成真实秒要**除以** rate：1 板秒 = 1/1.09 真实秒，板秒更短 → 每板秒更多单位）
    #   · 真值侧给的是**每仿真秒 200 单位**（契约），换算成板秒要乘 rtf/速率，
    #     而 rtf 每次都不一样（0.75~0.83），所以**不能**直接拿来积分。
    path_speed = ((vis.get("path_fit") or {}).get("speed")
                  if vis.get("ok") else None)
    best_speed = path_speed if path_speed else (tru.get("speed") if tru.get("ok") else None)
    # 延迟：真值侧给的是"平台本身"的（0.02~0.13 s），视觉侧给的是端到端（含相机/处理）
    best_delay = (tru.get("delay_s") if tru.get("ok") else None)
    if best_delay is None and vis.get("ok"):
        best_delay = vis.get("delay_s")
    vis_lat = ((vis.get("path_fit") or {}).get("delay_s") if vis.get("ok") else None)
    calib = {
        #: **虚拟真值要用的速度**：单位/**板秒**（板子自己的时钟）。它在板子上积分，
        #: 所以不能换成真实秒 —— 板子的时间基快 9%，换过去积分就会慢 9%。
        "speed_units_s": best_speed,
        "speed_units_per_board_s": best_speed,
        "speed_units_real_s": (round(best_speed / rate, 1) if best_speed else None),
        "board_clock_rate": rate,
        #: 指令→平台真正开始移动（真值侧，单位：**秒**；板秒 = ×rate）
        "cmd_delay_s": best_delay,
        "cmd_delay_board_s": (round(best_delay * rate, 3) if best_delay else None),
        #: **端到端**延迟（板子发指令 → 相机看到它动），单位：板秒。
        #: 决策/预测要用这个数，不是上面那个：视觉看到的是 0.5 s 前的世界。
        "vision_latency_board_s": vis_lat,
        "initial_spread": vis.get("initial_spread") if vis.get("ok") else None,
        "initial_xy": vis.get("initial") if vis.get("ok") else None,
        "source": {"speed": "vision_path_fit" if path_speed else ("truth" if tru.get("ok") else None),
                   "delay": "truth" if tru.get("ok") and tru.get("delay_s") is not None else "vision"},
        "vision": vis,
        "truth": tru,
    }
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(calib, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n  标定写入 {out}（速度 {best_speed}，延迟 {best_delay}，"
          f"来源 {calib['source']}）")
    return 0 if best_speed else 1


if __name__ == "__main__":
    raise SystemExit(main())
