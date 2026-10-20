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
            frame_py[int(e["frame"])] = float(e.get("py", 0.0))
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
    res.update({"run": str(a.run_dir), "scenario": a.scenario,
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
