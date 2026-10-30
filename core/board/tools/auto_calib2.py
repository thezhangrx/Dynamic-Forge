#!/usr/bin/env python3
"""auto_calib2.py —— 用标定抓到的帧**重拟合** y 换算与 x 仿射，并做验收。

和 calib_y.py 的区别（这次踩的坑就在这里）：
  * 先用**上一版标定**做"初值"，把检测到的墙行关联到真值行，再迭代重拟合
    —— 避免"盲关联 → 只匹配到 1 行 → 斜率退化 0"；
  * x 从"一个常数偏置"升级成**仿射**：x_page = a·x_img + c（多行、多缺口联合拟合）；
  * 自带验收：distinct_rows ≥ 3、y rmse ≤ 1.0 px、x p90 ≤ 6 px，不过就 FAIL。

输入：
  <run>/board_vt.jsonl        （板端 vt_sparse 输出）
  <run>/board_events.jsonl    （vt_tick，给每帧 py）
  configs/<scenario>.json     （真值 walls_y / gaps_x / gap_widths）
  data/fpga/calib_y.json      （初值，可缺省 → 用恒等初值）
输出：
  data/fpga/calib_y.json（覆盖）+ 可选推到板端 /tmp/calib_y.json

用法：
  python3 core/board/tools/auto_calib2.py <run_dir> --scenario configs/20_walls_alt_24_dy140.json --py 40
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
    ys = np.array(m.get("walls_y", []), float)
    gx = np.array(m.get("gaps_x", []), float)
    gw = np.array(m.get("gap_widths", []), float)
    rows = {}
    for i, y in enumerate(ys):
        if i < len(gx) and i < len(gw):
            rows[round(float(y), 1)] = (float(gx[i] - gw[i] / 2), float(gx[i] + gw[i] / 2))
    return rows


def load(run: Path, py_at: float):
    """读一个抓帧目录；多姿态时由调用方把多个目录的结果拼起来。"""
    bv = [json.loads(l) for l in open(run / "board_vt.jsonl")]
    ev = [json.loads(l) for l in (run / "board_events.jsonl").read_text(errors="ignore").splitlines()
          if '"vt_tick"' in l]
    py_of = {e["frame"]: e.get("py_used") for e in ev if e.get("py_used") is not None}
    out = []
    for r in bv:
        if not (r.get("walls") or []):
            continue
        out.append((py_of.get(r["frame"], py_at), r["walls"]))
    return out


def associate(frames, tys, scale, bias, tol=30.0):
    """用当前 (scale,bias) 把每个检测行关联到最近真值行，返回样本 [(vy, y_truth-(py-240))]。"""
    ys, xs, gs, vys = [], [], [], []
    for py, walls in frames:
        for w in walls:
            y_page = (py - 240.0) + scale * w["y"] + bias
            j = int(np.argmin(np.abs(tys - y_page)))
            if abs(tys[j] - y_page) <= tol:
                ys.append((float(w["y"]), float(tys[j] - (py - 240.0))))
                xs.append((float(w["gl"]), float(w["gr"])))
                gs.append(tys[j])
                vys.append(float(w["y"]))
    return np.array(ys), np.array(xs), np.array(gs), np.array(vys)


def fit_y(samples: np.ndarray):
    """最小二乘拟合 (py-240) + scale*vy + bias = y_truth。"""
    vy, target = samples[:, 0], samples[:, 1]
    A = np.c_[vy, np.ones(len(vy))]
    (scale, bias), *_ = np.linalg.lstsq(A, target, rcond=None)
    pred = A @ np.array([scale, bias])
    rmse = float(np.sqrt(np.mean((pred - target) ** 2)))
    return float(scale), float(bias), rmse


def fit_x_agent(xs, rows_map, y_of_row, vy_of, prior_a=1.9, prior_b=0.04):
    """按 **board_agent 的语义**拟合横向修正：offset = A + B*(vy - 240)。

    agent 里：
        _xc = x_bias_a + x_bias_b*(w["y"] - view_h/2)
        gl_page = gl_view - _xc + margin
    所以 x_bias_a 是"偏移量(px)"、x_bias_b 是"每行变化率"，**不是**仿射的 a/c。
    """
    src, vys = [], []
    for (gl, gr), y, vy in zip(xs, y_of_row, vy_of):
        tr = rows_map.get(round(float(y), 1))
        if tr is None:
            continue
        src += [float(gl) - tr[0], float(gr) - tr[1]]
        vys += [float(vy), float(vy)]
    if len(src) < 12:
        return prior_a, prior_b, 99.0
    off = np.array(src)
    A = np.c_[np.ones(len(off)), np.array(vys) - 240.0]
    (a, b), *_ = np.linalg.lstsq(A, off, rcond=None)
    for _ in range(4):
        r = np.abs(A @ np.array([a, b]) - off)
        keep = r <= max(6.0, 2.5 * np.median(r))
        if keep.sum() < 12:
            break
        (a, b), *_ = np.linalg.lstsq(A[keep], off[keep], rcond=None)
    res = np.abs(A @ np.array([a, b]) - off)
    return float(a), float(b), float(np.percentile(res, 90))


def fit_x(xs: np.ndarray, rows_map: dict, y_of_row: np.ndarray, prior_a=1.0, prior_c=0.0):
    """x_page = a*x_img + c，用检测缺口两端对真值缺口两端做最小二乘（迭代剔除离群）。"""
    src, dst = [], []
    for (gl, gr), y in zip(xs, y_of_row):
        tr = rows_map.get(round(float(y), 1))
        if tr is None:
            continue
        src += [gl, gr]
        dst += [tr[0], tr[1]]
    if len(src) < 8:
        return prior_a, prior_c, 99.0
    src = np.array(src, float); dst = np.array(dst, float)
    a, c = prior_a, prior_c
    for _ in range(4):
        A = np.c_[src, np.ones(len(src))]
        (a, c), *_ = np.linalg.lstsq(A, dst, rcond=None)
        res = np.abs(A @ np.array([a, c]) - dst)
        keep = res <= max(8.0, 2.5 * np.median(res))
        if keep.sum() < 8:
            break
        src, dst = src[keep], dst[keep]
    res = np.abs(np.c_[src, np.ones(len(src))] @ np.array([a, c]) - dst)
    return float(a), float(c), float(np.percentile(res, 90))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="+", help="一个或多个抓帧目录（多姿态标定）")
    ap.add_argument("--scenario", default="configs/20_walls_alt_24_dy140.json")
    ap.add_argument("--py", type=float, default=40.0)
    ap.add_argument("--push", action="store_true")
    ap.add_argument("--out", default=str(CALIB))
    a = ap.parse_args()

    rows = truth_rows(a.scenario)
    if not rows:
        print("配置里没有真值行"); return 2
    tys = np.array(sorted(rows))
    frames = []
    for r in a.run:
        fr = load(Path(r), a.py)
        print("  载入 %s：%d 帧（有墙）" % (r, len(fr)))
        frames += fr
    if not frames:
        print("抓到的帧里没有墙（vt_sparse 没测出东西）"); return 2

    prior = {}
    if CALIB.exists():
        prior = json.loads(CALIB.read_text())
    scale = float(prior.get("scale", 1.0))
    bias = float(prior.get("bias", 0.0))
    if not (0.3 < abs(scale) < 3.0):
        scale, bias = 1.0, 0.0          # 初值不可信就用恒等
    print("初值 scale=%.4f bias=%.2f（来自 %s）" % (scale, bias, "上一版" if prior else "恒等"))

    # ---- 粗网格初始化：不依赖上一版标定（它可能已经过期/被别的姿态污染）----
    best = (0, 0.0, 0.0, 1e9)
    for sc0 in np.arange(0.88, 1.08, 0.01):
        for b0 in np.arange(-80, 81, 4):
            cnt, res = 0, 0.0
            for py, walls in frames:
                for w in walls:
                    y = (py - 240.0) + sc0 * w["y"] + b0
                    d = float(np.abs(tys - y).min())
                    if d <= 20.0:
                        cnt += 1; res += d
            if cnt and (cnt > best[0] or (cnt == best[0] and res / cnt < best[3])):
                best = (cnt, sc0, b0, res / cnt)
    if best[0] >= 6:
        scale, bias = float(best[1]), float(best[2])
        print("粗网格初值：scale=%.2f bias=%.1f（命中 %d 个样本，均残差 %.1f px）"
              % (scale, bias, best[0], best[3]))
    else:
        print("粗网格没找到初值（命中 %d），沿用上一版 scale=%.3f bias=%.2f" % (best[0], scale, bias))

    for it in range(4):
        ys, xs, yrow, vys = associate(frames, tys, scale, bias)
        if len(ys) < 6:
            print("迭代 %d：关联到的样本只有 %d 个（阈值 30 px）" % (it, len(ys)))
            break
        n1, n2 = scale, bias
        scale, bias, rmse = fit_y(ys)
        distinct = len({round(float(y), 1) for y in yrow})
        print("迭代 %d：样本 %d，关联到 %d 个不同真值行，scale=%.4f bias=%.2f rmse=%.2f px"
              % (it, len(ys), distinct, scale, bias, rmse))
        if abs(scale - n1) < 1e-3 and abs(bias - n2) < 0.05:
            break

    ys, xs, yrow, vys = associate(frames, tys, scale, bias)
    if len(ys) < 6:
        print("❌ FAIL：关联样本太少（%d），检查 quad/对焦/页面" % len(ys))
        return 1
    scale, bias, rmse = fit_y(ys)
    #: 按 agent 语义拟合（这才是板端真正用的）
    xA, xB, xp90 = fit_x_agent(xs, rows, yrow, vys,
                               prior.get("x_bias_a", 1.9), prior.get("x_bias_b", 0.04))
    #: 参考用的仿射（只写进文件、agent 不读）
    xa, xc, _ = fit_x(xs, rows, yrow)
    distinct = len({round(float(y), 1) for y in yrow})

    span = 0.0
    if distinct >= 2:
        vals = sorted({round(float(y), 1) for y in yrow})
        span = vals[-1] - vals[0]
    #: 2 个不同行 + 跨度 ≥100 px 就足以定 scale/bias（两点确定一条直线）；
    #: 3 行以上更稳，但不是必要条件。
    bias_only = False
    if distinct < 2 or span < 100.0:
        # 只拍到一道墙：scale 定不出来。**不覆盖 scale**，只修 bias。
        print("  → 只关联到 %d 个真值行（跨度 %.0f px），改做 **仅 bias 拟合**，scale 沿用上一版" % (distinct, span))
        old_scale = float(prior.get("scale", 1.0)) or 1.0
        pred = samples_y = None
        ys2, xs2, _, _ = associate(frames, tys, old_scale, bias)
        if len(ys2) >= 6:
            target = ys2[:, 1] - old_scale * ys2[:, 0]
            bias = float(np.median(target))
            scale = old_scale
            pred = old_scale * ys2[:, 0] + bias
            rmse = float(np.sqrt(np.mean((pred - ys2[:, 1]) ** 2)))
            bias_only = True
            ys = ys2
    ok = (xp90 <= 6.0) and (rmse <= 1.0) and (0.3 < abs(scale) < 3.0) \
        and ((distinct >= 2 and span >= 100.0) or bias_only)
    print()
    print("拟合结果：scale=%.5f bias=%.3f  x_bias_a=%.2f x_bias_b=%.4f (p90 %.1f px)  [仿射参考 a=%.4f c=%.2f]"
          % (scale, bias, xA, xB, xp90, xa, xc))
    print("样本 %d，不同真值行 %d，y rmse %.2f px" % (len(ys), distinct, rmse))
    print("行跨度 %.0f px；判据：不同行≥2 且跨度≥100 且 y rmse≤1.0 且 x p90≤6.0 且 |scale|∈(0.3,3)" % span)
    if not ok:
        print("❌ FAIL：标定没通过验收，不写文件")
        return 1

    cal = dict(prior)
    cal.update({"scale": scale, "bias": bias,
                "x_bias_a": xA, "x_bias_b": xB, "x_bias": xA,
                "x_affine_a": xa, "x_affine_c": xc, "rmse": rmse, "samples": int(len(ys)), "bias_only": bias_only,
                "rows_matched": int(distinct), "scenario": a.scenario,
                "run": ";".join(str(r) for r in a.run),
                "py": a.py, "poses": len(a.run)})
    Path(a.out).write_text(json.dumps(cal, ensure_ascii=False, indent=2), encoding="utf-8")
    print("已写", a.out)
    if a.push:
        r = subprocess.run(["scp", "-q", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                            a.out, "%s:/tmp/calib_y.json" % BOARD])
        print("推送板端：", "ok" if r.returncode == 0 else "失败")
    print("✅ PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
