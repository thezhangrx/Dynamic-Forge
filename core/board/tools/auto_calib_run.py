#!/usr/bin/env python3
"""auto_calib_run.py —— **多姿态标定**：直接用一次完整闭环跑的数据做拟合。

为什么不用"冻结页面换 spawn"：冻结页面不跟随视口（已验证，见
`视觉侧自动化.md` §六），换 spawn 画面不变。而**一次真实运行**里 py 从 40 走到 3400，
天然覆盖几十个姿态、几十道墙行 —— 而且不需要额外跑实验。

做法：
  1. 读 run 的 `board_vt.jsonl` + `board_events.jsonl`（vt_tick 给每帧 py、px）；
  2. **只取静止帧**（相邻帧位移 < --static，默认 0.5 px）：运动帧有 0.5 s 管线延时；
  3. 粗网格初始化（不依赖上一版标定）→ 迭代最小二乘；
  4. **交叉验证**：按 py 分桶，偶数桶拟合、奇数桶验证，报告跨姿态残差；
  5. 验收：跨姿态 rmse ≤ 1.5 px、不同真值行 ≥ 4、x p90 ≤ 6 px；通过才写文件。

用法：
  python3 core/board/tools/auto_calib_run.py data/fpga/loop_logs/run_xxx \
      --scenario configs/20_walls_alt_24_dy140.json [--push]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CALIB = ROOT / "data/fpga/calib_y.json"
BOARD = "root@192.168.137.242"
KEY = str(ROOT / "data/fpga/board_ssh/board_key")


def truth_rows(scenario: str):
    d = json.load(open(ROOT / scenario))
    m = d.get("meta", {})
    ys = [float(y) for y in m.get("walls_y", [])]
    gx = [float(x) for x in m.get("gaps_x", [])]
    gw = [float(x) for x in m.get("gap_widths", [])]
    rows = {}
    for i, y in enumerate(ys):
        if i < len(gx) and i < len(gw):
            rows[round(y, 1)] = (gx[i] - gw[i] / 2, gx[i] + gw[i] / 2)
    return rows


def load_static(run: Path, static: float):
    bv = [json.loads(l) for l in open(run / "board_vt.jsonl")]
    ev = [json.loads(l) for l in (run / "board_events.jsonl").read_text(errors="ignore").splitlines()
          if '"vt_tick"' in l]
    pos = {e["frame"]: (e.get("px"), e.get("py_used")) for e in ev}
    xs, ys = [], []
    for r in bv:
        p = pos.get(r["frame"])
        xs.append(float("nan") if not p or p[0] is None else p[0])
        ys.append(float("nan") if not p or p[1] is None else p[1])
    xs, ys = np.array(xs), np.array(ys)
    v = np.full(len(xs), np.inf)
    v[1:] = np.hypot(np.diff(xs), np.diff(ys))
    keep = np.zeros(len(xs), bool)
    keep[1:] = v[1:] < static
    out = []
    for i, r in enumerate(bv):
        if keep[i] and r.get("walls"):
            out.append((float(ys[i]), r["walls"]))
    return out


def associate(frames, tys, scale, bias, tol):
    s, g = [], []
    for py, walls in frames:
        for w in walls:
            y = (py - 240.0) + scale * w["y"] + bias
            j = int(np.argmin(np.abs(tys - y)))
            if abs(tys[j] - y) <= tol:
                s.append((w["y"], tys[j] - (py - 240.0), py, tys[j]))
                g.append((w["gl"], w["gr"], tys[j]))
    return (np.array(s) if s else np.zeros((0, 4))), g


def fit(samples):
    A = np.c_[samples[:, 0], np.ones(len(samples))]
    (sc, bi), *_ = np.linalg.lstsq(A, samples[:, 1], rcond=None)
    r = A @ np.array([sc, bi]) - samples[:, 1]
    return float(sc), float(bi), float(np.sqrt(np.mean(r ** 2)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--scenario", default="configs/20_walls_alt_24_dy140.json")
    ap.add_argument("--static", type=float, default=0.5)
    ap.add_argument("--push", action="store_true")
    a = ap.parse_args()

    rows = truth_rows(a.scenario)
    tys = np.array(sorted(rows))
    frames = load_static(Path(a.run), a.static)
    print("静止帧（有墙）：%d" % len(frames))
    if len(frames) < 30:
        print("❌ FAIL：静止帧太少（<30），换个更长的 run")
        return 2
    pys = sorted({round(py) for py, _ in frames})
    print("覆盖 py：%d .. %d（%d 个不同位置）" % (pys[0], pys[-1], len(pys)))

    # ---- 粗网格初始化 ----
    best = (0, 1.0, 0.0, 1e9)
    for sc0 in np.arange(0.85, 1.15, 0.01):
        for b0 in np.arange(-120, 121, 4):
            cnt, res = 0, 0.0
            for py, walls in frames:
                for w in walls:
                    y = (py - 240.0) + sc0 * w["y"] + b0
                    d = float(np.abs(tys - y).min())
                    if d <= 20.0:
                        cnt += 1; res += d
            if cnt and (cnt > best[0] or (cnt == best[0] and res / cnt < best[3])):
                best = (cnt, sc0, b0, res / cnt)
    if best[0] < 20:
        print("❌ FAIL：粗网格找不到初值（命中 %d）" % best[0])
        return 2
    scale, bias = float(best[1]), float(best[2])
    print("粗网格初值 scale=%.2f bias=%.1f（命中 %d，均残差 %.1f px）" % (scale, bias, best[0], best[3]))

    for it in range(5):
        s, _ = associate(frames, tys, scale, bias, 25.0)
        if len(s) < 12:
            print("❌ FAIL：关联样本不足（%d）" % len(s)); return 2
        n1, n2 = scale, bias
        scale, bias, rmse = fit(s)
        # 稳健：丢掉 |残差| > max(6, 3·MAD) 的样本后重拟合
        for _ in range(3):
            pred = s[:, 0] * scale + bias
            r = np.abs(pred - s[:, 1])
            mad = np.median(np.abs(r - np.median(r))) + 1e-6
            keep = r <= max(6.0, 3.0 * mad)
            if keep.sum() < 12 or keep.sum() == len(s):
                break
            s = s[keep]
            scale, bias, rmse = fit(s)
        print("迭代 %d：样本 %d，scale=%.5f bias=%.2f rmse=%.2f px" % (it, len(s), scale, bias, rmse))
        if abs(scale - n1) < 1e-4 and abs(bias - n2) < 0.02:
            break

    s, gaps = associate(frames, tys, scale, bias, 20.0)
    if len(s) < 12:
        print("❌ FAIL：最终样本不足"); return 2
    scale, bias, rmse = fit(s)
    distinct = len({round(float(y), 1) for y in s[:, 3]})
    span = float(max(s[:, 3]) - min(s[:, 3])) if distinct > 1 else 0.0

    # ---- 跨姿态交叉验证：按 py 分桶，奇偶桶互验 ----
    buckets = (np.round(s[:, 2] / 200.0) * 200.0).astype(float)  # s[:,2] = py
    uniq = sorted(set(buckets.tolist()))
    res_cv = []
    if len(uniq) >= 4:
        for parity in (0, 1):
            tr = np.isin(buckets % 400, [0 if parity == 0 else 200])
            if tr.sum() < 8 or (~tr).sum() < 8:
                continue
            sc2, bi2, _ = fit(s[tr])
            pred = s[~tr, 0] * sc2 + bi2
            res_cv.append(float(np.sqrt(np.mean((pred - s[~tr, 1]) ** 2))))
    cv_rmse = float(np.mean(res_cv)) if res_cv else rmse
    print("不同真值行 %d，行跨度 %.0f px；跨姿态验证 rmse=%.2f px（%d 折）"
          % (distinct, span, cv_rmse, len(res_cv)))

    # ---- x 仿射 ----
    src, dst = [], []
    for gl, gr, y in gaps:
        tr = rows.get(round(float(y), 1))
        if tr is None:
            continue
        src += [gl, gr]; dst += [tr[0], tr[1]]
    xa, xc, xp90 = 1.0, 0.0, 99.0
    if len(src) >= 12:
        src = np.array(src, float); dst = np.array(dst, float)
        for _ in range(4):
            A = np.c_[src, np.ones(len(src))]
            (xa, xc), *_ = np.linalg.lstsq(A, dst, rcond=None)
            r = np.abs(A @ np.array([xa, xc]) - dst)
            keep = r <= max(8.0, 2.5 * np.median(r))
            if keep.sum() < 12:
                break
            src, dst = src[keep], dst[keep]
        xp90 = float(np.percentile(np.abs(np.c_[src, np.ones(len(src))] @ np.array([xa, xc]) - dst), 90))

    ok = (distinct >= 4 and span >= 300 and cv_rmse <= 1.5 and xp90 <= 6.0
          and 0.3 < abs(scale) < 3.0)
    print()
    print("结果：scale=%.5f bias=%.3f  x:a=%.4f c=%.2f(p90 %.1f px)"
          % (scale, bias, xa, xc, xp90))
    print("判据：不同行≥4 且跨度≥300 且 跨姿态 rmse≤1.5 且 x p90≤6")
    if not ok:
        print("❌ FAIL：多姿态标定没通过验收，不写文件")
        return 1

    prior = json.loads(CALIB.read_text()) if CALIB.exists() else {}
    prior.update({"scale": scale, "bias": bias, "x_bias_a": xa, "x_bias": xa * 320 + xc - 320,
                  "x_bias_b": xc, "rmse": cv_rmse, "samples": int(len(s)),
                  "rows_matched": int(distinct), "poses": len(uniq),
                  "mode": "multi-pose(from run)", "run": str(a.run), "scenario": a.scenario})
    CALIB.write_text(json.dumps(prior, ensure_ascii=False, indent=2), encoding="utf-8")
    print("已写", CALIB)
    if a.push:
        r = subprocess.run(["scp", "-q", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                            str(CALIB), "%s:/tmp/calib_y.json" % BOARD])
        subprocess.run(["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no", BOARD,
                        "cp -f /tmp/calib_y.json /root/vision_state/ 2>/dev/null || true"])
        print("推送板端：", "ok" if r.returncode == 0 else "失败")
    print("✅ PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
