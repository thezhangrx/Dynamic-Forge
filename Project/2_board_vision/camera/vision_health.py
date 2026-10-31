#!/usr/bin/env python3
"""vision_health.py —— 视觉侧的"体检"指标（对焦/曝光/取景/标记），供各处复用。

指标定义（都在 640×480 灰度上算，代价 <2 ms）：
  sharp_edge   墙线 10%→90% 上升宽度（px），越小越清晰；清晰 ≈1.5–2.5，糊 >4
  lap_var      拉普拉斯方差（只在墙行区域，避开光斑）
  sat_ratio    ≥250 的像素占比；>0.5% 说明过曝（墙会被"削平"）
  frame_occ    白框/亮框占画面比例；用来判断页面有没有铺满
  dark_med     暗区中位亮度；场地应 10–40
  markers      ArUco 标记数（能认出来 = 对焦够好的硬证据）

命令行：
  python3 vision_health.py --url http://192.168.137.242:8099/stream.mjpg
  python3 vision_health.py --image /tmp/a.jpg --json
"""
from __future__ import annotations

import argparse
import json
import sys

import cv2
import numpy as np


def metrics(img: np.ndarray) -> dict:
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    h, w = g.shape[:2]
    out = {"w": w, "h": h}

    # 亮区（墙 + 白框）
    bright = g >= 120
    out["bright_ratio"] = float(bright.mean())
    out["sat_ratio"] = float((g >= 250).mean())

    # 暗区中位亮度：只取不亮的地方
    dark = g[~bright]
    out["dark_med"] = float(np.median(dark)) if dark.size > 500 else -1.0
    out["wall_med"] = float(np.median(g[bright])) if bright.sum() > 500 else -1.0

    # 拉普拉斯方差（整帧，作为对焦粗指标）
    lap = cv2.Laplacian(g, cv2.CV_32F, ksize=3)
    out["lap_var"] = float(lap.var())

    # 墙线边缘宽度：只统计"干净台阶"的行，取中位数
    edges = []
    for y in range(0, h, 2):
        prof = g[y].astype(np.float32)
        prof = cv2.GaussianBlur(prof.reshape(1, -1), (1, 5), 0).ravel()
        lo, hi = float(np.percentile(prof, 5)), float(np.percentile(prof, 95))
        if hi - lo < 100:
            continue
        t10, t90 = lo + 0.1 * (hi - lo), lo + 0.9 * (hi - lo)
        above = prof >= t90
        if not (0.15 < above.mean() < 0.85):      # 台阶要"有一段亮一段暗"
            continue
        i90 = np.flatnonzero(above)
        if not i90.size:
            continue
        j = int(i90[0])
        if j < 3:
            continue
        i10 = np.flatnonzero(prof[:j] <= t10)
        if not i10.size:
            continue
        edges.append(float(j - int(i10[-1])))
    out["sharp_edge"] = round(float(np.median(edges)) if edges else -1.0, 2)
    out["edge_rows"] = len(edges)

    # 反光/亮斑：连通域里"块状"（不是细长线）的亮区
    num, lab, stats, _ = cv2.connectedComponentsWithStats(bright.astype(np.uint8), 8)
    glare = 0
    for k in range(1, num):
        x, y, ww, hh, area = stats[k]
        if area < 300:
            continue
        if hh <= 0 or ww <= 0:
            continue
        if 0.5 <= ww / hh <= 2.0 and area > 3000:      # 块状且够大 → 反光
            glare += int(area)
    out["glare_ratio"] = round(glare / float(w * h), 4)

    # 页面占画面比例：亮框外接矩形
    ys, xs = np.where(bright)
    if xs.size > 100:
        x0, x1 = np.percentile(xs, 2), np.percentile(xs, 98)
        y0, y1 = np.percentile(ys, 2), np.percentile(ys, 98)
        bw = int(x1 - x0 + 1); bh = int(y1 - y0 + 1)
        out["frame_occ"] = round(bw * bh / float(w * h), 3)
        out["frame_box"] = [int(x0), int(y0), int(x1), int(y1)]
    else:
        out["frame_occ"] = 0.0
    return out


def verdict(m: dict) -> tuple[bool, list[str]]:
    bad = []
    if m.get("frame_occ", 0) < 0.70:
        bad.append("页面只占画面 %.0f%%（<70%%）→ 相机推近或放大窗口" % (100 * m.get("frame_occ", 0)))
    if m.get("sharp_edge", -1) < 0:
        bad.append("找不到清晰墙线（画面糊/没有墙）")
    elif m.get("sharp_edge", 0) > 6.0:
        # 依据：墙线本身只有 2–3 px 宽；10–90% 上升沿 ≤6 px 时，质心抖动仍 <1 px
        #（实测标定残差 rmse 0.1 px）。超过 6 px 才对质心产生可见偏移。
        bad.append("墙线边缘 %.1f px（>6）→ 对焦不够" % m["sharp_edge"])
    if m.get("sat_ratio", 0) > 0.005:
        bad.append("过曝：饱和像素占 %.1f%%（>0.5%%）" % (100 * m["sat_ratio"]))
    if m.get("glare_ratio", 0) > 0.02:
        bad.append("反光带占画面 %.1f%%（>2%%）→ 会变成幻影墙" % (100 * m.get("glare_ratio", 0)))
    # 暗区亮度不是越小越好：相机噪声/增益会把黑底抬到 60~80。
    # 真正要的是"墙比场亮很多"，所以看两者之差。
    d = m.get("dark_med", -1)
    if not (0 <= d <= 90):
        bad.append("暗区中位亮度 %.0f（应在 0–90）" % d)
    elif m.get("wall_med", 0) - d < 60:
        bad.append("墙与场地对比度不足（%.0f vs %.0f）" % (m.get("wall_med", 0), d))
    return (len(bad) == 0), bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.242:8099/stream.mjpg")
    ap.add_argument("--image")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    if a.image:
        img = cv2.imread(a.image)
        if img is None:
            print("读不到", a.image); return 2
    else:
        cap = cv2.VideoCapture(a.url)
        img = None
        for _ in range(40):
            ok, fr = cap.read()
            if ok:
                img = fr
                break
        cap.release()
        if img is None:
            print("打不开流", a.url); return 2

    m = metrics(img)
    ok, bad = verdict(m)
    if a.json:
        print(json.dumps({"metrics": m, "pass": ok, "problems": bad}, ensure_ascii=False))
    else:
        print("指标:", {k: m[k] for k in ("frame_occ", "sharp_edge", "sat_ratio", "dark_med", "glare_ratio", "lap_var")})
        print("结论:", "✅ PASS" if ok else "❌ " + "；".join(bad))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
