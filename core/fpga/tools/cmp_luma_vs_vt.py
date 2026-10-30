#!/usr/bin/env python3
"""相机域对拍：`vt_sparse`（CPU 基线） vs 「正射 + PL 判据」（C 参考 luma 模式）。

为什么要有它（2026-10-25）
-------------------------
P1a 把 `vt_sparse` 的自适应亮度判据搬进 PL 之后，必须证明**在真实相机帧上**
两边给的是同一个世界。这条脚本就是那个证据生成器：
    相机 640×480 ──┬─ vt_sparse ────────────────→ 页面坐标下的墙/缺口（基线）
                   └─ 正射(640×360) → vision_ref --set luma_mode=1 → 页面坐标

坐标：PL 帧是"页面按 0.75 等比放进 640×360"（左右各 80 px 边带），所以
    x_page = (x_pl - 80) / 0.75      y_page = (360 - y_pl) / 0.75

判据（与 PL_P0_P1_MIGRATION.md §4 第 3 关一致）
    * 墙 Δy 中位 ≤ 2 px、p90 ≤ 4 px
    * 缺口左右边界 |Δ| 中位 ≤ 3 px
    * 条数一致率 ≥ 95%

用法
    python3 cmp_luma_vs_vt.py --mkv <cam_rec.mkv> --quad data/fpga/quad.json \
        --vt /tmp/vt_local --pl /tmp/pl_vision_local --ref /tmp/visref \
        --frames 300 [--start-frac 0.25] [--jobs 4]
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile

import cv2
import numpy as np

K_PL_W, K_PL_H = 640, 360
PAGE_W, PAGE_H = 640.0, 480.0
FIT = min(K_PL_W / PAGE_W, K_PL_H / PAGE_H)          # 0.75
OFF_X = (K_PL_W - PAGE_W * FIT) * 0.5                # 80
Q = 64.0


def extract_frames(mkv: str, count: int, start_frac: float, out_raw: str) -> int:
    cap = cv2.VideoCapture(mkv)
    if not cap.isOpened():
        raise SystemExit(f"打不开录像：{mkv}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if total <= 0:
        total = 100000
    start = int(total * start_frac)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    n = 0
    with open(out_raw, "wb") as fh:
        while n < count:
            ok, f = cap.read()
            if not ok:
                break
            if f.shape[1] != 640 or f.shape[0] != 480:
                f = cv2.resize(f, (640, 480))
            fh.write(cv2.cvtColor(f, cv2.COLOR_BGR2RGB).tobytes())
            n += 1
    cap.release()
    return n


def parse_geom(path: str):
    b = open(path, "rb").read()
    if len(b) < 76:
        return None
    magic, ver, flags, seq, nobs, ngaps, w, h, stamp = struct.unpack_from("<IHHIIIIHQ", b, 0)
    if magic != 0x31564842:
        return None
    off = 32
    player = struct.unpack_from("<4i", b, off)
    off += 16 + 24
    obs, gaps = [], []
    for _ in range(nobs):
        obs.append(struct.unpack_from("<9i", b, off))
        off += 36
    for _ in range(ngaps):
        gaps.append(struct.unpack_from("<9i", b, off))
        off += 36
    return dict(player=player, obs=obs, gaps=gaps)


def pl_to_page_y(y_pl: float) -> float:
    return (K_PL_H - y_pl) / FIT


def pl_to_page_x(x_pl: float) -> float:
    return (x_pl - OFF_X) / FIT


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mkv", required=True)
    ap.add_argument("--quad", required=True)
    ap.add_argument("--vt", required=True, help="vt_sparse 可执行文件（基线）")
    ap.add_argument("--pl", required=True, help="pl_vision 可执行文件（--ortho-only）")
    ap.add_argument("--ref", required=True, help="vision_ref 可执行文件（--set luma_mode=1）")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--start-frac", type=float, default=0.25)
    ap.add_argument("--ytol", type=float, default=40.0, help="墙 y 配对容差（页面单位）")
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="cmpluma_")
    raw = os.path.join(tmp, "cam.raw")
    n = extract_frames(args.mkv, args.frames, args.start_frac, raw)
    if n == 0:
        print("没取到帧")
        return 1

    # 1) 基线：vt_sparse 直接吃相机帧
    vt_jsonl = os.path.join(tmp, "vt.jsonl")
    with open(raw, "rb") as fh:
        subprocess.run([args.vt, "--w", "640", "--h", "480",
                        "--quad-file", args.quad, "--jsonl", vt_jsonl],
                       stdin=fh, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    vt = [json.loads(x) for x in open(vt_jsonl) if x.strip()]

    # 2) 正射 → 640×360 页面图
    ppm_pre = os.path.join(tmp, "o", "f")
    os.makedirs(os.path.dirname(ppm_pre), exist_ok=True)
    with open(raw, "rb") as fh:
        subprocess.run([args.pl, "--w", "640", "--h", "480", "--quad-file", args.quad,
                        "--ortho-only", "--no-camera-domain", "--dump-ppm", ppm_pre],
                       stdin=fh, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    ppms = sorted(os.listdir(os.path.dirname(ppm_pre)))
    ppms = [p for p in ppms if p.endswith(".ppm")]

    # 3) PL 判据（C 参考 luma 模式）→ 几何帧
    dy, dgl, dgr = [], [], []
    n_vt_walls = n_pl_walls = 0
    cnt_agree = 0
    per_frame = []
    # ⚠ 必须用**文件名里的帧号**去索引 vt，不能用列表下标：
    #   pl_vision 在"几何没锁到"的帧会先写 jsonl 再 continue（**不落 PPM**），
    #   于是 PPM 编号有洞；按 enumerate 下标取 vt 会整体错位一帧，
    #   表现成"某几帧 PL 突然一道墙都没有"（2026-10-25 踩过）。
    vt_by_frame = {int(v.get("frame", 0)): v for v in vt}
    for ppm in ppms:
        i = int(ppm.split("_")[-1].split(".")[0]) - 1
        bins = os.path.join(tmp, "g.bin")
        subprocess.run([args.ref, os.path.join(os.path.dirname(ppm_pre), ppm),
                        bins, "--set", "luma_mode=1"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        g = parse_geom(bins)
        if g is None or (i + 1) not in vt_by_frame:
            continue
        v = vt_by_frame[i + 1]
        vwalls = [(w["y"], w["gl"], w["gr"]) for w in (v.get("walls") or [])]
        # PL 的"缺口记录"直接给出 (cy, cx, width)，换算到页面坐标就是 (y, gl, gr)
        pwalls = []
        for gp in g["gaps"]:
            cy, cx, wdt = gp[2] / Q, gp[1] / Q, gp[3] / Q
            pwalls.append((pl_to_page_y(cy),
                           pl_to_page_x(cx - wdt / 2.0),
                           pl_to_page_x(cx + wdt / 2.0)))
        n_vt_walls += len(vwalls)
        n_pl_walls += len(pwalls)
        if len(vwalls) == len(pwalls):
            cnt_agree += 1
        used = set()
        for (vy, vgl, vgr) in vwalls:
            best = None
            for k, (py, pgl, pgr) in enumerate(pwalls):
                if k in used:
                    continue
                d = abs(py - vy)
                if d <= args.ytol and (best is None or d < best[0]):
                    best = (d, k, py, pgl, pgr)
            if best is None:
                continue
            used.add(best[1])
            dy.append(abs(best[2] - vy))
            dgl.append(abs(best[3] - vgl))
            dgr.append(abs(best[4] - vgr))
        per_frame.append((i + 1, len(vwalls), len(pwalls)))
        if len(vwalls) != len(pwalls) and len(per_frame) <= 400:
            only_vt = [round(v[0], 1) for v in vwalls]
            only_pl = [round(p[0], 1) for p in pwalls]
            print(f"  [帧 {i+1}] vt={only_vt}  PL={only_pl}")

    def stat(a):
        return (float(np.median(a)), float(np.percentile(a, 90))) if a else (float("nan"), float("nan"))

    print(f"帧数 {len(ppms)}（起点 {args.start_frac:.2f}）")
    print(f"vt 墙数合计 {n_vt_walls} / PL 墙数合计 {n_pl_walls}；条数完全一致的帧 {cnt_agree}/{len(per_frame)}")
    print(f"配对墙 {len(dy)} 道")
    print(f"  |Δy|  中位 {stat(dy)[0]:.2f} px  p90 {stat(dy)[1]:.2f}")
    print(f"  |Δgl| 中位 {stat(dgl)[0]:.2f} px  p90 {stat(dgl)[1]:.2f}")
    print(f"  |Δgr| 中位 {stat(dgr)[0]:.2f} px  p90 {stat(dgr)[1]:.2f}")
    agree = cnt_agree / max(1, len(per_frame))
    ok = (stat(dy)[0] <= 2.0 and stat(dy)[1] <= 4.0 and
          stat(dgl)[0] <= 3.0 and stat(dgr)[0] <= 3.0 and agree >= 0.95)
    print(f"判定：{'PASS' if ok else 'FAIL'}"
          f"（门槛：Δy 中位≤2/p90≤4、缺口边界中位≤3、条数一致率≥95%）")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
