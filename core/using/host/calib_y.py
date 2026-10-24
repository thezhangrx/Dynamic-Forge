#!/usr/bin/env python3
"""自动标定板端 VT 的 y 换算系数（scale / bias）—— 不再手动调。

原理
----
页面把角色画在视口正中，所以板端 VT 看到的每一行像素 ``vy``（jsonl 里的 wall y，
从可见带下沿往上数）与场地坐标是线性关系：

    world = (py - half) + scale * vy + bias

其中 ``py`` 是当时的角色位姿（板端收到的 @POSE，日志里逐帧有）。

把真值墙行 ``y_true`` 代进来，就得到一组线性方程：

    scale * vy + bias = y_true - py + half

用多帧、多道墙做**鲁棒最小二乘**（迭代两轮：先用当前系数把墙配到最近的真值行，
剔除残差 > tol 的离群点，再拟合），即可自动解出 ``scale/bias``。

调用时机（用户要求）
------------------
- 相机/窗口一动（`--quad` 指纹变化超过阈值）→ 重新标定；
- 还没有标定结果时 → 标定；
- 标定好且相机没动 → **不再调用**，直接用上次的系数。

用法
----
    Env/bin/python core/board/tools/calib_y.py <run_dir> \
        --scenario configs/20_walls_alt_24_dy140.json \
        --out data/fpga/calib_y.json [--apply /tmp/calib_y.json]

``<run_dir>`` 需要有：
  board_events.jsonl  （vt_tick 事件里的 frame / py）
  board_vt.jsonl      （每帧 walls[{y,gl,gr}]）
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


def read_jsonl(path: Path) -> list[dict]:
    out: list[dict] = []
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def collect_pairs(run_dir: Path) -> list[tuple[float, float, int]]:
    """返回 [(vy, py, frame)]：每帧里每道墙的像素行 + 当时的角色 y。"""
    ev = read_jsonl(run_dir / "board_events.jsonl")
    frame_py: dict[int, float] = {}
    for e in ev:
        if e.get("ev") == "vt_tick" and e.get("frame") is not None:
            #: **必须用"拍摄时刻的 py"**（py_used，由帧时间戳插值得到），
            #: 而不是读帧那一刻的 py —— 两者相差一个管道延迟（~0.3~0.5 s），
            #: 用错的那个会让拟合系数系统性地偏，表现为所有墙的 Δy 同号同量。
            _pu = e.get("py_used")
            if _pu is None:          # 老日志没记 py_used，退回读帧时刻的 py
                _pu = e.get("py", 0.0)
            frame_py[int(e["frame"])] = float(_pu)
    vt = read_jsonl(run_dir / "board_vt.jsonl")
    pairs: list[tuple[float, float, int]] = []
    for rec in vt:
        fr = int(rec.get("frame", -1))
        if fr not in frame_py:
            continue
        py = frame_py[fr]
        for w in rec.get("walls") or []:
            vy = float(w.get("y", 0.0))
            if 1.0 < vy < 479.0:
                pairs.append((vy, py, fr))
    return pairs


def fit_quad(pairs: list[tuple[float, float, int]], truth_y: list[float], half: float,
             scale0: float = 1.0, bias0: float = 0.0, tol: float = 40.0,
             rounds: int = 4) -> dict:
    """y 的**二次**模型：world = (py-half) + a + b*vy + c*vy²。

    为什么线性不够：页面四角只给单应 8 自由度，广角畸变让"同一像素行在不同高度上
    对应的场地 y"不是线性的。实测一轮闭环里 16 行墙的 y 残差随 vy 呈弧形（679 应
    是 640、836 应是 780、1578 应是 1620），线性拟合只能折中，残差 30~100 →
    板子的世界模型里出现"错位的墙" → 规划器以为走廊被堵死，直接停住不动
    （demo3/demo4/demo5 卡死都是这个）。
    """
    a, b = float(bias0), float(scale0)     # a 常数项、b 一次项
    c = 0.0                                 # 二次项
    for _ in range(max(1, rounds)):
        rows: list[tuple[float, float, float]] = []
        for (vy, py, _fr) in pairs:
            world = (py - half) + a + b * vy + c * vy * vy
            t = min(truth_y, key=lambda v: abs(v - world))
            if abs(t - world) <= tol:
                rows.append((vy, t - py + half, world - t))
        if len(rows) < 12:
            break
        # 最小二乘（正规方程 3x3）
        import itertools
        basis = [(1.0, v, v * v) for (v, _y, _r) in rows]
        rhs = [y for (_v, y, _r) in rows]
        m = [[sum(bi[i] * bi[j] for bi in basis) for j in range(3)] for i in range(3)]
        v = [sum(bi[i] * r for bi, r in zip(basis, rhs)) for i in range(3)]
        for i in range(3):                     # 高斯消元
            p = max(range(i, 3), key=lambda k: abs(m[k][i]))
            m[i], m[p] = m[p], m[i]
            v[i], v[p] = v[p], v[i]
            if abs(m[i][i]) < 1e-12:
                break
            for k in range(i + 1, 3):
                f = m[k][i] / m[i][i]
                for j in range(i, 3):
                    m[k][j] -= f * m[i][j]
                v[k] -= f * v[i]
        if abs(m[2][2]) < 1e-12:
            break
        c = v[2] / m[2][2]
        b = (v[1] - m[1][2] * c) / m[1][1]
        a = (v[0] - m[0][1] * b - m[0][2] * c) / m[0][0]
        del itertools
    res: list[float] = []
    for (vy, py, _fr) in pairs:
        world = (py - half) + a + b * vy + c * vy * vy
        t = min(truth_y, key=lambda v: abs(v - world))
        if abs(t - world) <= tol:
            res.append(abs(world - t))
    res.sort()
    return {"q_a": round(a, 4), "q_b": round(b, 6), "q_c": round(c, 9),
            "q_rmse": round(res[len(res) // 2], 3) if res else None,
            "q_samples": len(res),
            "scale": round(b, 5), "bias": round(a, 3)}   # 兼容旧字段（取一次项）


def fit_x_bias(vt_rows: list[dict], frame_py: dict[int, float], truth_gaps: list[tuple],
               truth_y: list[float], scale: float, bias: float,
               half: float) -> tuple[float, float, int]:
    """VT 缺口横向偏差 **随手电平台 y 线性变化**的两个系数。

    为什么不是常数：页面四角只给单应 8 个自由度，广角头的桶形畸变让"同一列像素
    在不同高度上横向偏移不同"。实测（40 帧静态）：wall@280 偏 +0.9 px，
    wall@420 偏 +8.4 px —— 差 7.5 px，用单一常数最多只能折中。

    模型：  x_corr(y) = x_a + x_b * (y - half)
    用所有已配对的行做最小二乘（每行先取该行的中位数，避免贴边行权重过大）。
    返回 (x_a, x_b, 样本行数)；行数 < 2 时 x_b = 0。
    """
    import statistics
    #: 自变量必须是**帧内页面坐标 vy**（畸变是"在画面里的位置"的函数），
    #: 不能用绝对场地 y：用 y 拟合再代到 y≈3160 的墙上，会算出一个 126 px 的
    #: 荒谬修正（实测 s2a：缺口整体被平移 −126，直接穿墙）。
    per_row: dict[float, list[float]] = {}
    for rec in vt_rows:
        py = frame_py.get(int(rec.get("frame", -1)))
        if py is None:
            continue
        for w in rec.get("walls") or []:
            world = (py - half) + scale * float(w.get("y", 0.0)) + bias
            ty = min(truth_y, key=lambda v: abs(v - world))
            if abs(ty - world) > 30.0:
                continue
            g = [gg for gg in truth_gaps if abs(gg[0] - ty) < 1.0]
            if not g:
                continue
            _, tgl, tgr = g[0]
            dx = 0.5 * ((float(w["gl"]) - tgl) + (float(w["gr"]) - tgr))
            per_row.setdefault(round(float(w.get("y", 0.0)) / 20.0) * 20.0,
                               []).append(dx)
    if len(per_row) < 1:
        return 0.0, 0.0, 0
    pts = [(vy, statistics.median(v)) for vy, v in per_row.items() if v]
    if len(pts) == 1:
        return float(pts[0][1]), 0.0, 1
    n = float(len(pts))
    sx = sum(y - half for y, _ in pts)
    sy = sum(d for _, d in pts)
    sxx = sum((y - half) ** 2 for y, _ in pts)
    sxy = sum((y - half) * d for y, d in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-9:
        return float(sy / n), 0.0, len(pts)
    xb = (n * sxy - sx * sy) / den
    xa = (sy - xb * sx) / n
    return float(xa), float(xb), len(pts)


def fit(pairs: list[tuple[float, float, int]], truth_y: list[float], half: float,
        scale0: float = 1.0, bias0: float = 0.0, tol: float = 40.0,
        rounds: int = 4) -> dict:
    """鲁棒最小二乘：world = (py-half) + scale*vy + bias。

    步骤：用当前 (scale,bias) 把每道墙映射成 world → 配最近真值行 →
    保留 |残差| ≤ tol 的样本 → 对 (vy → y_true - py + half) 做最小二乘。
    """
    a, b = float(scale0), float(bias0)
    used: list[tuple[float, float, float]] = []   # (vy, target, resid)
    for _ in range(max(1, rounds)):
        samples: list[tuple[float, float, float]] = []
        for vy, py, _fr in pairs:
            world = (py - half) + a * vy + b
            t = min(truth_y, key=lambda v: abs(v - world))
            resid = world - t
            if abs(resid) <= tol:
                samples.append((vy, t - py + half, resid))
        if len(samples) < 6:
            break
        n = float(len(samples))
        sx = sum(s[0] for s in samples)
        sy = sum(s[1] for s in samples)
        sxx = sum(s[0] * s[0] for s in samples)
        sxy = sum(s[0] * s[1] for s in samples)
        den = n * sxx - sx * sx
        if abs(den) < 1e-9:
            break
        a = (n * sxy - sx * sy) / den
        b = (sy - a * sx) / n
        used = samples
    resid = [s[2] for s in used]
    return {
        "scale": round(a, 5),
        "bias": round(b, 3),
        "samples": len(used),
        "rows_matched": len({round(s[1] + s[0] * 0 - s[0] * 0, 1) for s in used}),
        "rmse": round(math.sqrt(sum(r * r for r in resid) / len(resid)), 2) if resid else None,
        "max_abs": round(max((abs(r) for r in resid), default=0.0), 2),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="自动标定板端 VT y 换算系数")
    ap.add_argument("run_dir")
    ap.add_argument("--scenario", default="configs/20_walls_alt_24_dy140.json")
    ap.add_argument("--view-h", type=float, default=480.0)
    ap.add_argument("--tol", type=float, default=40.0)
    ap.add_argument("--out", default="data/fpga/calib_y.json")
    ap.add_argument("--apply", default="", help="同时写到这个路径（通常是 /tmp/calib_y.json）")
    ap.add_argument("--quad", default="", help="当前白框四角指纹（字符串），用于判断相机是否移动")
    a = ap.parse_args(argv)

    cfg = json.loads(Path(a.scenario).read_text(encoding="utf-8"))
    truth_y = [float(v) for v in (cfg.get("meta", {}).get("walls_y") or [])]
    if not truth_y:
        print("场景里没有 meta.walls_y，无法标定", flush=True)
        return 2
    pairs = collect_pairs(Path(a.run_dir))
    if len(pairs) < 10:
        print(f"样本不足（{len(pairs)}），本轮跳过标定", flush=True)
        return 3

    lin = fit(pairs, truth_y, 0.5 * a.view_h, tol=a.tol)
    res = fit_quad(pairs, truth_y, 0.5 * a.view_h,
                   scale0=float(lin.get("scale") or 1.0),
                   bias0=float(lin.get("bias") or 0.0), tol=a.tol)
    res.update({"lin_rmse": lin.get("rmse"), "lin_max": lin.get("max_abs")})

    #: 同一批样本顺带拟合横向固定偏差（页面四角单应挡不住桶形畸变）
    meta = cfg.get("meta", {})
    truth_gaps = [(float(y), float(g) - float(w) / 2.0, float(g) + float(w) / 2.0)
                  for y, g, w in zip(meta.get("walls_y") or [],
                                     meta.get("gaps_x") or [],
                                     meta.get("gap_widths") or [])]
    vt_rows = read_jsonl(Path(a.run_dir) / "board_vt.jsonl")
    ev = read_jsonl(Path(a.run_dir) / "board_events.jsonl")
    frame_py = {}
    for e in ev:
        if e.get("ev") == "vt_tick" and e.get("frame") is not None:
            _pu = e.get("py_used")
            if _pu is None:
                _pu = e.get("py", 0.0)
            frame_py[int(e["frame"])] = float(_pu)
    x_a, x_b, x_n = (0.0, 0.0, 0)
    if truth_gaps:
        x_a, x_b, x_n = fit_x_bias(vt_rows, frame_py, truth_gaps, truth_y,
                                   float(res["scale"]), float(res["bias"]),
                                   0.5 * a.view_h)
    res.update({"x_bias_a": round(x_a, 3), "x_bias_b": round(x_b, 5),
                "x_bias": round(x_a, 2),   # 兼容旧字段：常数部分
                "x_rows": x_n,
                "run": str(a.run_dir), "scenario": a.scenario,
                "quad": a.quad or None, "pairs": len(pairs)})
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    if a.apply:
        Path(a.apply).write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
