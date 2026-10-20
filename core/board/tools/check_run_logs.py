#!/usr/bin/env python3
"""把一次跑的 run 目录读成"证据报告"：轨迹 / 板子信号 / 视觉 / 图形 / 真值对照。

用法：
    Env/bin/python core/board/tools/check_run_logs.py <run_dir> [--json]

它只读 build 出来的日志（`run_<ts>_<tag>/`），不跑仿真、不碰板子。所有结论都
带数字，方便判断下一次该改哪里：

  1 跑概述       到达/碰撞/最小间隙(位置)/仿真与墙钟/页面 fps
  2 轨迹         每 100 单位的最小间隙、动作时长分布、停走时长
  3 板子信号     @MOVE 数/变化次数/间隔、@POSE 数、板内各阶段耗时
  4 视觉处理     VT 帧率、每帧墙数、缺口宽度分布、VT 帧→入库延迟
  5 图形处理     页面渲染 p50/p95、帧间隔、热力图帧数
  6 真值对照     板端每道墙 vs 虚拟真值：Δy、Δgap 左边/右边、左右侧是否一致
  7 判据          自动判定哪条规则被触发（对应该修哪一块）
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


# ----------------------------------------------------------------------
def read_jsonl(path: Path, limit: int | None = None) -> list[dict]:
    out: list[dict] = []
    if not path.exists():
        return out
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
            if limit is not None and len(out) >= limit:
                break
    return out


def pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    i = min(len(s) - 1, max(0, int(round((len(s) - 1) * q))))
    return s[i]


def parse_state(path: Path) -> list[dict]:
    """板端 @STATE：seq px py r n [x y hw hh]*n —— 板子"看到"的墙。"""
    rows: list[dict] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.startswith("@STATE"):
            continue
        p = line.split()
        try:
            seq = int(p[1]); px = float(p[2]); py = float(p[3]); n = int(p[5])
        except (IndexError, ValueError):
            continue
        try:
            vals = [float(v) for v in p[6:6 + 5 * n]]
        except ValueError:
            continue
        if len(vals) < 5 * n:
            continue
        walls_px: list[tuple[float, float]] = []
        for i in range(n):
            x, y = vals[5 * i], vals[5 * i + 1]
            walls_px.append((x, y))
        rows.append({"seq": seq, "px": px, "py": py, "walls": walls_px})
    return rows


def board_wall_rows(states: list[dict]) -> dict[float, dict]:
    """从 @STATE 流里还原"板子认定的墙行"。

    @STATE 的 obs 是 `x*0.5 y hw hh v`：左段折到 [0,320)，右段折到 (320,640]。
    所以 左段右边界 gl = 2*max(x_left)，右段左边界 gr = 2*min(x_right)-640。
    """
    rows: dict[float, dict] = {}
    for st in states:
        by_y: dict[float, list[float]] = {}
        for x, y in st["walls"]:
            by_y.setdefault(round(float(y), 1), []).append(float(x))
        for y, xs in by_y.items():
            left = [v for v in xs if v <= 320.0]
            right = [v for v in xs if v > 320.0]
            gl = round(2.0 * max(left), 1) if left else None
            gr = round(2.0 * min(right) - 640.0, 1) if right else None
            d = rows.setdefault(y, {"y": y, "gls": [], "grs": [],
                                    "first_py": st["py"], "first_seq": st["seq"],
                                    "frames": 0})
            d["frames"] += 1
            if gl is not None:
                d["gls"].append(gl)
            if gr is not None:
                d["grs"].append(gr)
    for d in rows.values():
        d["gap_gl"] = round(statistics.median(d["gls"]), 1) if d["gls"] else None
        d["gap_gr"] = round(statistics.median(d["grs"]), 1) if d["grs"] else None
        d["gap_w"] = (round(d["gap_gr"] - d["gap_gl"], 1)
                      if d["gap_gl"] is not None and d["gap_gr"] is not None else None)
        d.pop("gls", None)
        d.pop("grs", None)
    return rows


# ----------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="读 run 目录出证据报告")
    ap.add_argument("run_dir")
    ap.add_argument("--json", action="store_true", help="顺带打印机器可读摘要")
    ap.add_argument("--tol-y", type=float, default=20.0)
    a = ap.parse_args(argv)

    run = Path(a.run_dir)
    if not run.exists():
        print(f"没有这个目录：{run}", file=sys.stderr)
        return 2

    meta = json.loads((run / "meta.json").read_text(encoding="utf-8")) \
        if (run / "meta.json").exists() else {}
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8")) \
        if (run / "summary.json").exists() else {}
    timeline = read_jsonl(run / "timeline.jsonl")
    board_io = read_jsonl(run / "board_io.jsonl")
    render = read_jsonl(run / "render.jsonl")
    truth_rows = read_jsonl(run / "truth_rows.jsonl")
    local_truth = read_jsonl(run / "local_truth.jsonl")
    board_events = read_jsonl(run / "board_events.jsonl")
    board_vt_all = read_jsonl(run / "board_vt.jsonl")
    states = parse_state(run / "board_state.txt")
    vtruth = {round(float(r["y"]), 1): r for r in (meta.get("virtual_truth") or [])}

    print(f"== run: {run.name} ==")
    print(f"  场景 {meta.get('scenario_name')}  "
          f"场地 {meta.get('field_w')}x{meta.get('field_h')}  "
          f"起点 {meta.get('spawn')}  终点 {meta.get('goal')}")

    # ---------------------------------------------------------------- 1
    print("\n[1] 跑概述")
    print(f"  到达={summary.get('reached')}  碰撞={summary.get('collisions')}  "
          f"t_sim={summary.get('t_sim_s')}s  墙钟={summary.get('wall_s')}s  "
          f"最终=({(summary.get('final_pos') or [None])[0]},"
          f"{(summary.get('final_pos') or [None, None])[1]})")
    print(f"  最小间隙={summary.get('min_clearance')}  "
          f"位置={summary.get('min_clearance_at')}  "
          f"页面 fps={summary.get('page_fps')}  render={summary.get('render_ms')} ms")
    print(f"  日志行数 {summary.get('logs')}")

    # ---------------------------------------------------------------- 2
    print("\n[2] 角色轨迹")
    if timeline:
        acts: dict[int, int] = {}
        for r in timeline:
            acts[int(r.get("act", 0))] = acts.get(int(r.get("act", 0)), 0) + 1
        total = max(1, len(timeline))
        moving = sum(v for k, v in acts.items() if k != 0)
        print(f"  轨迹点 {len(timeline)} 个  仿真时长 {timeline[-1]['t_sim']:.2f}s")
        print("  动作分布 " + "  ".join(
            f"{k}:{100.0 * v / total:.1f}%" for k, v in sorted(acts.items())))
        print(f"  动 {100.0 * moving / total:.1f}% / 停 {100.0 * (total - moving) / total:.1f}%")
        #: 每 100 单位的真值最小间隙
        buckets: dict[int, float] = {}
        for r in timeline:
            b = int(float(r.get("py", 0.0)) // 100)
            c = r.get("clear")
            if c is None:
                continue
            buckets[b] = min(buckets.get(b, 1e9), float(c))
        bad = [(b * 100, round(v, 1)) for b, v in sorted(buckets.items()) if v < 2.0]
        print(f"  间隙<2 的百米段: {bad if bad else '无'}")
        print(f"  全程最小 {min(buckets.values()):.2f}" if buckets else "  无间隙数据")

    # ---------------------------------------------------------------- 3
    print("\n[3] 板子信号与时间")
    poses = [r for r in board_io if r.get("kind") == "pose_sent"]
    moves = [r for r in board_io if r.get("kind") == "move_recv"]
    print(f"  @POSE 发出 {len(poses)} 条   @MOVE 收到 {len(moves)} 条")
    if moves:
        act_seq = [int(r["act"]) for r in moves]
        changes = sum(1 for i in range(1, len(act_seq)) if act_seq[i] != act_seq[i - 1])
        ts = [float(r["t_wall"]) for r in moves]
        gaps = [1000.0 * (ts[i] - ts[i - 1]) for i in range(1, len(ts))]
        print(f"  动作变化 {changes} 次   间隔 p50={pct(gaps, 0.5):.0f}ms "
              f"p95={pct(gaps, 0.95):.0f}ms max={max(gaps) if gaps else 0:.0f}ms")
    if poses:
        print(f"  首条 @POSE t={poses[0].get('t_wall')}s  末条 t={poses[-1].get('t_wall')}s")
    if board_events:
        cnt: dict[str, int] = {}
        for e in board_events:
            cnt[e.get("ev", "?")] = cnt.get(e.get("ev", "?"), 0) + 1
        print("  板端事件 " + "  ".join(f"{k}:{v}" for k, v in sorted(cnt.items())))
        #: 板内阶段耗时（同一时钟，不需要对齐）
        pauses = [float(e["t"]) for e in board_events if e.get("ev") == "pause"]
        resumes = [float(e["t"]) for e in board_events if e.get("ev") == "resume"]
        dur = []
        for i, p in enumerate(pauses):
            nxt = [r for r in resumes if r > p]
            if nxt:
                dur.append(nxt[0] - p)
        if dur:
            print(f"  暂停时长 p50={1000 * statistics.median(dur):.0f}ms "
                  f"max={1000 * max(dur):.0f}ms  共 {len(dur)} 次")
        #: VT 帧 → 入表 的延迟（板端同一时钟）
        vt_t = {int(e["vt_seq"]): float(e["t"]) for e in board_events
                if e.get("ev") == "vt_tick" and e.get("vt_seq") is not None}
        lag = [float(e["t"]) - vt_t[int(e["vt_seq"])] for e in board_events
               if e.get("ev") == "add_wall" and int(e.get("vt_seq", -1)) in vt_t]
    #: 跨机器时钟对齐：用 pose_seq 把"主机发出该位姿"和"板端带着该位姿干活"
    #: 配起来，估计 board_t → host_t 的偏移（25 Hz 位姿 → 量化 ~40 ms）
    host_pose = {int(r["seq"]): float(r["t_wall"]) for r in poses
                 if r.get("seq") is not None}
    board_pose: dict[int, float] = {}
    for e in board_events:
        if e.get("pose_seq") and int(e["pose_seq"]) > 0 and e.get("t") is not None:
            board_pose.setdefault(int(e["pose_seq"]), float(e["t"]))
    pairs = [(host_pose[seq_], bt) for seq_, bt in board_pose.items() if seq_ in host_pose]
    clock_off = None
    if len(pairs) >= 5:
        diffs = sorted(h - b for h, b in pairs)
        clock_off = statistics.median(diffs)
        print(f"  时钟对齐：配对 {len(pairs)} 组，board_t→host_t 偏移 "
              f"{clock_off:+.3f}s（p10 {diffs[len(diffs)//10]:+.3f} "
              f"p90 {diffs[9*len(diffs)//10]:+.3f}）")

    # ---------------------------------------------------------------- 4
    print("\n[4] 视觉处理（板端 VT）")
    #: /tmp/vt.jsonl 是累积文件，拷回来时带着上一轮的帧；只保留本轮 vt_tick 覆盖的帧号
    vt_ticks = [e for e in board_events if e.get("ev") == "vt_tick"]
    frames_used = [int(e["frame"]) for e in vt_ticks if e.get("frame") is not None]
    if frames_used:
        lo_f, hi_f = min(frames_used), max(frames_used)
        board_vt = [r for r in board_vt_all
                    if lo_f - 2 <= int(r.get("frame", -1)) <= hi_f + 2]
    else:
        board_vt = board_vt_all
    if board_vt:
        frames = [int(r.get("frame", 0)) for r in board_vt]
        n_walls = [len(r.get("walls") or []) for r in board_vt]
        geom = sum(1 for r in board_vt if r.get("frame_geom"))
        print(f"  VT 帧 {len(board_vt)} 个（frame {frames[0]}→{frames[-1]}） "
              f"几何有效 {100.0 * geom / max(1, len(board_vt)):.0f}%  "
              f"每帧墙数 mean={statistics.mean(n_walls):.2f} max={max(n_walls)}")
        gaps = []
        for r in board_vt:
            for w in r.get("walls") or []:
                g = float(w.get("gr", 0)) - float(w.get("gl", 0))
                if g > 5:
                    gaps.append(g)
        if gaps:
            print(f"  VT 缺口宽度 p50={statistics.median(gaps):.0f} "
                  f"min={min(gaps):.0f} max={max(gaps):.0f}（真值 90~110）")
        ys = [float(w["y"]) for r in board_vt for w in (r.get("walls") or [])]
        if ys:
            print(f"  VT 墙行 y ∈ [{min(ys):.0f},{max(ys):.0f}]（帧内像素坐标）")
        if vt_ticks:
            t_first = float(vt_ticks[0].get("t", 0.0))
            t_last = float(vt_ticks[-1].get("t", 0.0))
            if t_last > t_first:
                print(f"  VT 帧率 ≈ {len(vt_ticks) / max(1e-6, t_last - t_first):.2f} 帧/s"
                      f"（本轮 {len(vt_ticks)} 帧 / {t_last - t_first:.1f}s）")
        #: 几何自检：VT 角色 x 与主机 px 的偏差（页面把角色画在它的场地 x 上）
        dxs = [abs(float(e["player_dx"])) for e in vt_ticks
               if e.get("player_dx") is not None]
        rej = sum(1 for e in board_events if e.get("ev") == "geom_reject")
        if dxs:
            print(f"  VT 角色 x 偏差 |dx|: p50={statistics.median(dxs):.1f} "
                  f"p95={pct(dxs, 0.95):.1f} max={max(dxs):.1f}"
                  f"  → 几何坏帧 {rej}/{len(vt_ticks)}")
        #: 候选→入表：暂停到入表用了多久、跨了几帧 VT
        pauses_ev = [e for e in board_events if e.get("ev") == "pause"]
        adds_ev = [e for e in board_events if e.get("ev") == "add_wall"]
        dts, nframes = [], []
        for add in adds_ev:
            prev = [p for p in pauses_ev if float(p.get("t", 0)) <= float(add.get("t", 0))]
            if not prev:
                continue
            pz = prev[-1]
            dts.append(1000.0 * (float(add["t"]) - float(pz["t"])))
            if pz.get("vt_seq") is not None and add.get("vt_seq") is not None:
                nframes.append(int(add["vt_seq"]) - int(pz["vt_seq"]))
        if dts:
            print(f"  暂停→入表: p50={statistics.median(dts):.0f}ms "
                  f"max={max(dts):.0f}ms  跨 VT 帧 p50="
                  f"{statistics.median(nframes) if nframes else 0:.0f}")

    # ---------------------------------------------------------------- 5
    print("\n[5] 图形处理（主机渲染）")
    page = [r for r in render if r.get("kind") == "page"]
    field = [r for r in render if r.get("kind") == "field"]
    if page:
        ms = [float(r.get("ms", 0.0)) for r in page]
        gap = [float(r.get("gap_ms", 0.0)) for r in page if "gap_ms" in r]
        print(f"  页面渲染 {len(page)} 帧  ms p50={pct(ms, 0.5):.2f} p95={pct(ms, 0.95):.2f}")
        if gap:
            print(f"  页面帧间隔 p50={pct(gap, 0.5):.1f}ms p95={pct(gap, 0.95):.1f}ms "
                  f"→ 有效 {1000.0 / max(1e-6, pct(gap, 0.5)):.1f} fps")
    if field:
        print(f"  热力图帧 {len(field)} 个  排队丢弃={field[-1].get('dropped', 0)}")

    # ---------------------------------------------------------------- 6
    print("\n[6] 真值对照（板端认的墙 vs 虚拟真值）")
    ticks = [(int(e["seq"]), float(e["t"])) for e in board_events
             if e.get("ev") == "state_tick" and e.get("seq") is not None]
    state_t0 = None
    if ticks:
        ticks.sort()
        state_t0 = ticks[0][1]
        rate = (ticks[-1][0] - ticks[0][0]) / max(1e-6, ticks[-1][1] - ticks[0][1])
        print(f"  @STATE 时间锚点 {len(ticks)} 个：seq {ticks[0][0]}→{ticks[-1][0]}，"
              f"≈{rate:.0f} 条/s（板端时钟起点 t={state_t0:.2f}s）")
    bw = board_wall_rows(states)
    print(f"  板端 @STATE 里出现过 {len(bw)} 行道墙；虚拟真值 {len(vtruth)} 行")
    matched: list[dict] = []
    for y, d in sorted(bw.items()):
        #: 找最近的真值行
        cand = min(vtruth.values(), key=lambda r: abs(float(r["y"]) - y)) if vtruth else None
        if cand is None:
            continue
        dy = y - float(cand["y"])
        rec = {"board_y": y, "true_y": float(cand["y"]), "dy": round(dy, 1),
               "first_py": d.get("first_py"),
               "gap_gl_board": d.get("gap_gl"), "gap_gr_board": d.get("gap_gr"),
               "gap_w_board": d.get("gap_w")}
        if d.get("gap_gl") is not None and cand.get("gap_gl") is not None:
            rec["d_gl"] = round(float(d["gap_gl"]) - float(cand["gap_gl"]), 1)
        if d.get("gap_gr") is not None and cand.get("gap_gr") is not None:
            rec["d_gr"] = round(float(d["gap_gr"]) - float(cand["gap_gr"]), 1)
        if cand.get("gap_side"):
            rec["side"] = cand["gap_side"]
            rec["true_center"] = cand.get("gap_center")
        if abs(dy) <= a.tol_y:
            matched.append(rec)
    for rec in matched[:30]:
        extra = " ".join(f"{k}={rec[k]}" for k in ("d_gl", "d_gr", "side")
                         if k in rec)
        if rec.get("gap_w_board") is not None:
            extra += f" 板端缺口宽={rec['gap_w_board']:.0f}"
        print(f"    y={rec['board_y']:7.1f}  真值 {rec['true_y']:6.0f}  "
              f"Δy={rec['dy']:+6.1f}  首次见 py={rec.get('first_py')}  {extra}")
    widths = [(r.get("gap_w_board"), float(vtruth[round(r["true_y"], 1)]["gap_width"]))
              for r in matched
              if r.get("gap_w_board") is not None
              and vtruth.get(round(r["true_y"], 1), {}).get("gap_width")]
    if widths:
        diffs = [w - tw for (w, tw) in widths if w is not None]
        print(f"  板端缺口宽度 - 真值宽度: 平均 {statistics.mean(diffs):+.1f} "
              f"（真值宽度中位数 {statistics.median([tw for _w, tw in widths]):.0f}）")
    if matched:
        dys = [abs(r["dy"]) for r in matched]
        print(f"  匹配 {len(matched)} 行  |Δy| 平均 {statistics.mean(dys):.1f} "
              f"max {max(dys):.1f}   命中率 {100.0 * len(matched) / max(1, len(vtruth)):.0f}%")
    #: 同侧缺口是否"被量化"：同侧的 gl/gr 本应随每道墙不同
    for side in ("L", "R"):
        gls = [r["gap_gl_board"] for r in matched
               if r.get("side") == side and r.get("gap_gl_board") is not None]
        grs = [r["gap_gr_board"] for r in matched
               if r.get("side") == side and r.get("gap_gr_board") is not None]
        if len(gls) >= 3:
            ug = len({(v // 5) * 5 for v in gls})
            ur = len({(v // 5) * 5 for v in grs}) if grs else 0
            flag = "  ← 量化嫌疑（同一侧所有墙测成同一缺口）" if max(ug, ur) <= 2 else ""
            print(f"  {side} 侧板端缺口: gl {ug} 种取值 "
                  f"({min(gls):.0f}~{max(gls):.0f})  gr {ur} 种取值"
                  f"{flag}")

    # ---------------------------------------------------------------- 7
    print("\n[7] 判据")
    flags: list[str] = []
    mc = summary.get("min_clearance")
    mca = summary.get("min_clearance_at") or []
    if mc is not None and mc < 0:
        flags.append(f"穿墙/擦墙：min_clearance={mc} @ {mca}")
    if matched:
        mean_dy = statistics.mean([abs(r["dy"]) for r in matched])
        if mean_dy > 15:
            flags.append(f"墙 y 映射误差偏大：|Δy| 平均 {mean_dy:.1f} > 15")
        if len(matched) / max(1, len(vtruth)) < 0.8:
            flags.append(f"真值覆盖率低：{len(matched)}/{len(vtruth)}")
    if board_vt:
        gaps_all = [float(w.get("gr", 0)) - float(w.get("gl", 0))
                    for r in board_vt for w in (r.get("walls") or [])]
        if gaps_all and statistics.median(gaps_all) < 80:
            flags.append(f"VT 缺口宽度偏窄：p50={statistics.median(gaps_all):.0f} < 80")
    if board_events and vt_ticks:
        dxs = [abs(float(e["player_dx"])) for e in vt_ticks
               if e.get("player_dx") is not None]
        if dxs and statistics.median(dxs) > 30:
            flags.append(f"VT 几何坏：角色 x 与主机 px 偏差 p50={statistics.median(dxs):.0f}"
                         f"（白框/四边形检测问题，不是策略问题）")
    edge = 0
    for r in board_vt:
        for w in r.get("walls") or []:
            if float(w.get("gl", 999)) < 20 or float(w.get("gr", 0)) > 620:
                edge += 1
    if board_vt and edge > 0.2 * max(1, len(board_vt)):
        flags.append(f"缺口贴边帧多：{edge} 帧（相机可能没看页面）")
    if summary.get("page_fps") is not None and float(summary["page_fps"]) < 0.9 * 30:
        flags.append(f"页面 fps 偏低：{summary['page_fps']}")
    if not flags:
        print("  未触发告警")
    for f in flags:
        print(f"  ⚠ {f}")

    if a.json:
        print("\n[JSON]")
        print(json.dumps({
            "run": run.name,
            "matched_rows": len(matched),
            "truth_rows": len(vtruth),
            "mean_abs_dy": round(statistics.mean([abs(r["dy"]) for r in matched]), 2)
            if matched else None,
            "flags": flags,
        }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
