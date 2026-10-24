#!/usr/bin/env python3
"""audit_crossings.py —— 逐墙审计：每一次"穿过缺口"到底是**凭信息穿过去的**，
还是**碰巧没撞**。

为什么需要它（用户要求）
------------------------
`min_clearance ≥ 0`、`碰撞 0` 只说明这一轮没撞上，不能说明"行为是对的"：
板子完全可能在**不认识那道墙**的情况下直着开过去，靠真值缺口恰好比较宽而侥幸过关。
这种偶然性同样不可接受。所以每次穿墙都要能回答三个问题：

  1. 穿之前，这道墙在板端 @STATE 里吗？（不认识 = 盲穿 = 不可接受）
  2. 穿的时候，角色的 x 在**板端自己认为**的缺口里吗？（不在 = 靠真值宽松侥幸 = 不可接受）
  3. 板端按自己的缺口算，穿行余量是多少？和真值余量差多少？（衡量"识别精度"）

判据（verdict）
---------------
  blind   ：穿之前这道墙不在 @STATE 里 → 没有任何信息就穿过去了
  lucky   ：知道墙，但角色 x 落在板端缺口之外（靠真值更宽才没撞）
  justified：知道墙，且角色 x 在板端缺口内（按测量数据走过去的）
另外报 `gap_err`：板端缺口左右边界 vs 真值（px，正=偏右），以及真值余量。

用法
----
    Env/bin/python core/board/tools/audit_crossings.py data/fpga/loop_logs/run_xxx \
        [--scenario configs/20_walls_alt_24_dy140.json] [--json]

退出码：0 = 全部 justified；1 = 有 blind/lucky。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="逐墙审计穿缺口是否'凭信息'")
    ap.add_argument("run_dir")
    ap.add_argument("--scenario", default=str(ROOT / "configs/20_walls_alt_24_dy140.json"))
    ap.add_argument("--tol-y", type=float, default=20.0,
                    help="板端记的墙 y 与真值行相差多少内算同一道墙")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    run = Path(a.run_dir)
    cfg = json.loads(Path(a.scenario).read_text(encoding="utf-8"))
    meta = cfg.get("meta", {})
    pr = float(cfg.get("player_radius") or 10.0)
    truth = [(float(y), float(g) - float(w) / 2.0, float(g) + float(w) / 2.0)
             for y, g, w in zip(meta.get("walls_y") or [],
                                meta.get("gaps_x") or [],
                                meta.get("gap_widths") or [])]
    if not truth:
        print("场景没有 meta.walls_y/gaps_x/gap_widths，无法审计", file=sys.stderr)
        return 2

    events = read_jsonl(run / "board_events.jsonl")
    timeline = read_jsonl(run / "timeline.jsonl")
    if not timeline:
        print(f"{run} 没有 timeline.jsonl，无法审计", file=sys.stderr)
        return 2

    # ---- 板端"什么时候知道了哪道墙"：按事件顺序，把 add_wall 与当时的 py 配对 ----
    cur_py: float | None = None
    added: list[dict] = []
    for e in events:
        ev = e.get("ev")
        if ev in ("state_tick", "vt_tick") and e.get("py") is not None:
            cur_py = float(e["py"])
        elif ev == "add_wall":
            added.append({"y": float(e.get("y", 0.0)),
                          "gl": float(e.get("gl", 0.0)),
                          "gr": float(e.get("gr", 0.0)),
                          "py_when_added": cur_py})

    def board_wall(row_y: float) -> dict | None:
        """真值行 row_y 对应板端记的哪一条（取最早加入的、且在穿之前加入的）。"""
        cands = [d for d in added if abs(d["y"] - row_y) <= a.tol_y]
        if not cands:
            return None
        before = [d for d in cands
                  if d["py_when_added"] is not None and d["py_when_added"] < row_y - 5.0]
        return (before[0] if before else cands[0])

    # ---- 每个真值行的穿越时刻 / 穿越时的 px ----
    rows = []
    for (wy, tgl, tgr) in truth:
        crossing = next((p for p in timeline if float(p["py"]) >= wy + pr), None)
        if crossing is None:
            rows.append({"row": wy, "verdict": "not_crossed", "note": "本轮没走到这道墙"})
            continue
        px_at = float(crossing["px"])
        py_at = float(crossing["py"])
        clear_t = min((float(p["clear"]) for p in timeline
                       if abs(float(p["py"]) - wy) <= 6.0), default=float("nan"))
        bw = board_wall(wy)
        rec = {"row": wy, "px_at": round(px_at, 1), "py_at": round(py_at, 1),
               "truth_gap": [round(tgl, 1), round(tgr, 1)],
               "truth_clear": round(clear_t, 2)}
        if bw is None:
            rec["verdict"] = "blind"
        else:
            known_before = (bw["py_when_added"] is not None
                            and bw["py_when_added"] < wy - 5.0)
            rec.update({"board_y": round(bw["y"], 1),
                        "board_gap": [round(bw["gl"], 1), round(bw["gr"], 1)],
                        "known_at_py": (None if bw["py_when_added"] is None
                                        else round(bw["py_when_added"], 1)),
                        "gap_err": [round(bw["gl"] - tgl, 1), round(bw["gr"] - tgr, 1)]})
            inside = bw["gl"] + pr <= px_at <= bw["gr"] - pr
            pred = min(px_at - bw["gl"], bw["gr"] - px_at) - pr
            rec["pred_clear"] = round(pred, 2)
            if not known_before:
                rec["verdict"] = "blind"
            elif not inside:
                rec["verdict"] = "lucky"
            else:
                rec["verdict"] = "justified"
        rows.append(rec)

    counts: dict[str, int] = {}
    for r in rows:
        counts[r["verdict"]] = counts.get(r["verdict"], 0) + 1
    verdict = 0 if not (counts.get("blind") or counts.get("lucky")) else 1

    if a.json:
        print(json.dumps({"rows": rows, "counts": counts}, ensure_ascii=False, indent=2))
        return verdict

    print(f"== 逐墙穿越审计：{run.name} ==")
    print(f"  角色半径 {pr}  真值墙 {len(truth)} 行")
    for r in rows:
        if r["verdict"] == "not_crossed":
            print(f"  y={r['row']:6.0f}  {'未走到':8s} {r['note']}")
            continue
        g = r.get("board_gap")
        ge = r.get("gap_err")
        tag = {"justified": "✓ 凭信息", "blind": "✗ 盲穿", "lucky": "✗ 侥幸"}[r["verdict"]]
        extra = ""
        if g:
            extra = (f" 板端缺口 {g[0]:.0f}-{g[1]:.0f} (边界误差 {ge[0]:+.0f}/{ge[1]:+.0f})"
                     f" 预测余量 {r['pred_clear']:+.1f}")
        print(f"  y={r['row']:6.0f}  {tag}  px={r['px_at']:6.1f} "
              f"真值缺口 {r['truth_gap'][0]:.0f}-{r['truth_gap'][1]:.0f} "
              f"真值余量 {r['truth_clear']:+.1f}{extra}")
    print(f"  小计：{counts}")
    if counts.get("blind"):
        print("  ⚠ 有盲穿：那道墙在穿之前从未进入板端 @STATE —— 属于'碰巧没撞'，不可接受")
    if counts.get("lucky"):
        print("  ⚠ 有侥幸：角色 x 落在板端自己认为的缺口之外，靠真值更宽才没撞")
    return verdict


if __name__ == "__main__":
    sys.exit(main())
