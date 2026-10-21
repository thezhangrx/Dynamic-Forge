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


def fit_x_bias(vt_rows: list[dict], frame_py: dict[int, float], truth_gaps: list[tuple],
               truth_y: list[float], scale: float, bias: float,
               half: float) -> tuple[float, int]:
    """VT 缺口左右边界相对真值的**固定横向平移**（中位数）。

    为什么需要：页面四角只给单应 8 个自由度，广角头的桶形畸变会让整幅画面
    在中部整体偏几个像素（实测 +7~+11 px）。这不是随机误差，是系统性偏差：
    在 y 上我们用 scale/bias 拟掉了，x 上一直没有对应的一项，于是板子总是
    朝真实缺口中心右侧 7~11 px 走 —— 90~110 宽的缺口里，这吃掉约 1/3 余量。
    这里同样**用真值标定一次**（和 y 的做法一致），把结果写进 calib 文件。
    """
    import statistics
    dxs: list[float] = []
    for rec in vt_rows:
        py = frame_py.get(int(rec.get("frame", -1)))
        if py is None:
            continue
        for w in rec.get("walls") or []:
            world = (py - half) + scale * float(w.get("y", 0.0)) + bias
            ty = min(truth_y, key=lambda v: abs(v - world))
            if abs(ty - world) > 40.0:
                continue
            # 找该真值行对应的缺口
            g = [gg for gg in truth_gaps if abs(gg[0] - ty) < 1.0]
            if not g:
                continue
            _, tgl, tgr = g[0]
            dxs.append(0.5 * ((float(w["gl"]) - tgl) + (float(w["gr"]) - tgr)))
    if len(dxs) < 10:
        return 0.0, len(dxs)
    return float(statistics.median(dxs)), len(dxs)


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

    res = fit(pairs, truth_y, 0.5 * a.view_h, tol=a.tol)

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
    x_bias, x_n = (0.0, 0)
    if truth_gaps:
        x_bias, x_n = fit_x_bias(vt_rows, frame_py, truth_gaps, truth_y,
                                 float(res["scale"]), float(res["bias"]),
                                 0.5 * a.view_h)
    res.update({"x_bias": round(x_bias, 2), "x_samples": x_n,
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
