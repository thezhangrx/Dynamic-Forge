#!/usr/bin/env python3
"""grade_run.py —— 一键验收：把"到达 / 碰撞 / 最小间隙 / 知识覆盖率"合成 PASS/FAIL。

判据（来自 `docs/闭环标准配置.md` 与 `虚拟真值约定.md`）：
  reached        必须 True
  collisions     必须 0
  min_clearance  ≥ +2.0
  覆盖率         学到/真值 必须 24/24（少 1 行记为 WARN，少 2 行以上 FAIL）
  |Δy| 平均      ≤ 5 px（板端墙行 vs 真值行，仅统计匹配上的行）

覆盖率算法与 auto_calib2 一致：用当前 `data/fpga/calib_y.json` 把板端墙行映射到世界 y，
再和场景真值行比对（15 px 内算命中）。

用法：
  python3 core/board/tools/grade_run.py data/fpga/loop_logs/run_xxx [--scenario ...]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def truth_rows(scenario: str):
    d = json.load(open(ROOT / scenario))
    m = d.get("meta", {})
    return np.array(sorted(float(y) for y in m.get("walls_y", [])))


def coverage(run: Path, scen: str, tol: float = 15.0):
    cal = json.loads((ROOT / "data/fpga/calib_y.json").read_text())
    sc, bi = float(cal.get("scale", 1.0)), float(cal.get("bias", 0.0))
    tys = truth_rows(scen)
    if len(tys) == 0:
        return 0, len(tys), 0.0, 0
    bv = [json.loads(l) for l in open(run / "board_vt.jsonl")]
    ev = [json.loads(l) for l in (run / "board_events.jsonl").read_text(errors="ignore").splitlines()
          if '"vt_tick"' in l]
    py_of = {e["frame"]: e.get("py_used") for e in ev if e.get("py_used") is not None}
    hit, diffs = set(), []
    for r in bv:
        py = py_of.get(r["frame"])
        if py is None:
            continue
        for w in (r.get("walls") or []):
            y = (py - 240.0) + sc * float(w["y"]) + bi
            j = int(np.argmin(np.abs(tys - y)))
            d = abs(float(tys[j]) - y)
            if d <= tol:
                hit.add(round(float(tys[j]), 1))
                diffs.append(d)
    return len(hit), len(tys), (float(np.mean(diffs)) if diffs else 99.0), len(diffs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--scenario", default=None)
    a = ap.parse_args()
    run = Path(a.run)
    summ_p = run / "summary.json"
    meta_p = run / "meta.json"
    if not summ_p.exists():
        print("缺 %s" % summ_p); return 2
    s = json.loads(summ_p.read_text())
    scen = a.scenario
    if scen is None and meta_p.exists():
        scen = json.loads(meta_p.read_text()).get("scenario")
    n_hit, n_all, dy, n_samp = coverage(run, scen)

    reached = bool(s.get("reached"))
    coll = int(s.get("collisions") or 0)
    clear = float(s.get("min_clearance", -99))
    cov_ok = n_hit >= n_all
    cov_warn = n_hit >= n_all - 1

    # 运动帧占比（信息项，不参与判定）：静止帧才可靠，这个数说明一轮里有多少时间是"动着的"
    try:
        bv = [json.loads(l) for l in open(run / "board_vt.jsonl")]
        ev = [json.loads(l) for l in (run / "board_events.jsonl").read_text(errors="ignore").splitlines()
              if '"vt_tick"' in l]
        pos = {e["frame"]: (e.get("px"), e.get("py_used")) for e in ev}
        pts = [pos.get(r["frame"]) for r in bv]
        n_move = 0
        for i in range(1, len(pts)):
            a, b = pts[i - 1], pts[i]
            if a and b and a[0] is not None and b[0] is not None:
                if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 0.5:
                    n_move += 1
        motion = n_move / max(len(pts) - 1, 1)
    except Exception:                     # noqa: BLE001
        motion = -1.0
    rows = [
        ("到达", reached, "%s" % reached),
        #: **碰撞口径 = 圆与矩形相交**（角色圆盘 vs 墙矩形），不是仿真器那个
        #: `collision_events` 计数器。实测两边会打架：fresh5/fresh7 的
        #: `min_clearance = −10.00`（圆已经压进墙 10 单位）却报"碰撞 0"——
        #: 因为仿真器按点判、不按圆判（评分标准差距清单 表 D 早就点名过）。
        #: 这里以 min_clearance < 0 为准，仿真器的计数只当信息打印。
        ("碰撞", clear >= 0.0,
         "%d 次（仿真计数 %d；口径：最小间隙<0 即算）"
         % (0 if clear >= 0.0 else 1, coll)),
        ("最小间隙", clear >= 2.0, "%+.2f" % clear),
        ("墙覆盖率", cov_ok, "%d/%d" % (n_hit, n_all)),
        ("|Δy| 平均", dy <= 5.0, "%.1f px（%d 行样本）" % (dy, n_samp)),
    ]
    info = ("运动帧占比", "%.0f%%（只作参考，静止帧才可靠）" % (100 * motion)) if motion >= 0 else None
    print("== 验收：%s ==" % run.name)
    for name, ok, val in rows:
        print("  %-10s %-8s %s" % (name, "✅" if ok else "❌", val))
    if info:
        print("  %-10s --       %s" % info)
    passed = all(ok for _, ok, _ in rows)
    if not passed and cov_warn:
        print("  （覆盖率少一行，按 WARN 处理但整体仍判 FAIL）")
    print()
    print("结果：", "✅ PASS" if passed else "❌ FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
