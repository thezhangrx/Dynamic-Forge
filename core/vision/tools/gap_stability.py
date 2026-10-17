#!/usr/bin/env python
"""缺口检出**稳定性台架**：同一张真实场地图，压上各种相机失真，看同一个几何量抖多少。

它回答的是那句最要命的问题：「你的判据是稳的，还是这个相机/这挡曝光的巧合？」

做法
----
1. 在一张**干净**的真实场地帧上，用**独立方法**量出缺口的真值：
   墙所在的几行上，暖色列（`r-b > 100`，干净图上毫无争议）的**最长背景连续段**。
2. 把同一帧压上相机退化（曝光 / 白平衡 / 失焦 / 噪声 / MJPG），再跑
   `relative.best_wall_band` + `relative.best_gap`。
3. 比较 `cx` / `width` 与真值的偏差 —— 这就是"稳不稳"的量。

用法
----
    Env/bin/python core/vision/tools/gap_stability.py
    Env/bin/python core/vision/tools/gap_stability.py --frames data/fpga/run4_720to360 --picks 0,30,59
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))

from vision import camera as C  # noqa: E402
from vision import primitives as P  # noqa: E402
from vision import relative as R  # noqa: E402


def load_ppm(path: Path) -> P.Frame:
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    if len(parts) < 4 or parts[0][:2] != b"P6":
        raise SystemExit(f"不是 P6 PPM: {path}")
    w, h = (int(v) for v in parts[1].split())
    px = parts[3][: w * h * 3]
    return P.Frame.from_rgb(w, h, px)


def truth_gap(frame: P.Frame, y0: int, y1: int, *, warm: int = 100) -> tuple:
    """**独立真值**：只在**墙自己的支撑区间内**量缺口。

    这里踩过一个坑：第一版把"整行最长的背景段"当缺口，于是量到了**场地外的黑边**
    （左边 0..80 列），真值中心报 64、宽 128，而正确答案是 224 / 45。
    所以支撑区间必须由墙本身定：从第一段暖色列到最后一段暖色列。
    """
    w = frame.width
    cols = [False] * w
    for x in range(w):
        hit = 0
        for y in range(y0, y1 + 1):
            i = y * w + x
            if frame.r[i] - frame.b[i] > warm:
                hit += 1
        cols[x] = hit * 2 > (y1 - y0 + 1)
    runs = []
    x = 0
    while x < w:
        if not cols[x]:
            x += 1
            continue
        s = x
        while x < w and cols[x]:
            x += 1
        runs.append((s, x - 1))
    if len(runs) < 2:
        return 0, 0.0
    lo, hi = runs[0][0], runs[-1][1]
    best = (0, lo, lo)               # (长度, 起点, 终点)，只在 [runs[0].start, runs[-1].end] 里找
    for k in range(len(runs) - 1):
        a, b = runs[k][1] + 1, runs[k + 1][0] - 1     # 两段墙之间的空隙
        if b - a + 1 > best[0]:
            best = (b - a + 1, a, b)
    if best[0] <= 0:
        return 0, 0.0
    if lo > best[1] or hi < best[2]:
        return 0, 0.0
    return best[2] - best[1] + 1, 0.5 * (best[1] + best[2])


@dataclass(frozen=True)
class Preset:
    name: str
    params: C.CameraParams


PRESETS = [
    Preset("干净（基准）", C.CameraParams()),
    Preset("曝光暗 gain.55", C.CameraParams(gain=0.55, offset=-8)),
    Preset("曝光亮 gain1.6", C.CameraParams(gain=1.6, offset=45)),
    Preset("白平衡偏暖", C.CameraParams(wb=(1.25, 0.95, 0.80))),
    Preset("白平衡偏冷", C.CameraParams(wb=(0.80, 1.00, 1.30))),
    Preset("失焦 blur=2", C.CameraParams(blur=2)),
    Preset("噪声 σ=5", C.CameraParams(noise=5.0, seed=7)),
    Preset("MJPG q=50", C.CameraParams(jpeg=50)),
    Preset("叠加（暗+偏暖+失焦+噪+MJPG）",
           C.CameraParams(wb=(1.2, 0.95, 0.85), gain=0.7, offset=10,
                          blur=1, noise=4.0, jpeg=60, seed=3)),
]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", default="data/fpga/run4_720to360")
    ap.add_argument("--picks", default="0,30,59", help="用哪几帧（逗号分隔）")
    args = ap.parse_args(argv)
    d = Path(args.frames)
    picks = [int(v) for v in args.picks.split(",") if v.strip()]
    files = sorted(d.glob("*.ppm"))
    if not files:
        raise SystemExit(f"{d} 里没有 ppm")
    bad = 0
    for pick in picks:
        path = files[min(pick, len(files) - 1)]
        clean = load_ppm(path)
        band = R.best_wall_band(clean)
        if not band["found"]:
            print(f"{path.name}: 干净图都找不到墙带，跳过")
            continue
        y0, y1 = band["y0"], band["y1"]
        tw, tcx = truth_gap(clean, y0, y1)
        print(f"\n=== {path.name}  墙带 y={y0}..{y1} (SNR {band['snr']:.1f})"
              f"  真值: 中心 {tcx:.0f}, 宽 {tw}")
        print(f"{'退化':<26}{'带SNR':>7}{'缺口SNR':>9}{'中心':>7}{'宽':>6}"
              f"{'中心误差':>9}{'宽误差':>8}")
        for pre in PRESETS:
            f = C.apply(clean, pre.params)
            b = R.best_wall_band(f)
            gg = R.best_gap(f, b["y0"], b["y1"]) if b["found"] else {"found": False}
            if not gg.get("found"):
                print(f"{pre.name:<26}{'-':>7}{'未找到':>9}")
                bad += 1
                continue
            ecx = gg["cx"] - tcx
            ew = gg["width"] - tw
            if abs(ecx) > 6 or abs(ew) > 8:
                bad += 1
            print(f"{pre.name:<26}{b['snr']:7.1f}{gg['snr']:9.1f}"
                  f"{gg['cx']:7.0f}{gg['width']:6.0f}{ecx:+9.0f}{ew:+8.0f}")
    print(f"\n超差（|中心误差|>6 px 或 |宽误差|>8 px）次数: {bad}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
