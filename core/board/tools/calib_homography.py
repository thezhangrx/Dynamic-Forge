#!/usr/bin/env python3
"""用 OpenCV 的透视变换做视觉标定：白框四角 → 单应矩阵 → 正射图 → 换算系数。

为什么这么做（对照之前的问题）
---------------------------
之前板端 VT 用"手调 scale/bias"把像素行换算成场地坐标，那两个数是拿错四边形
拟合出来的，属于**用系数补几何错误**。正确做法是：先用透视变换把页面拉正成
640×480（此时页面 1 像素 = 场地 1 单位，`y = 480 - row` 就是恒等映射），
再把"仍然残留的一点线性偏差"用最小二乘解出来当 scale/bias——这样系数应该接近
(1.0, 0.0)，偏得多就说明四边形没找准，而不是去调系数。

参考：`docs/repo/image_process/opencv-python-tutorials/15_image_transformations/`

用法
----
    # 从板子的 MJPEG 流取帧标定（PC 只读流，不参与板端决策，符合 R1）
    Env/bin/python core/board/tools/calib_homography.py \
        --url http://192.168.137.242:8099/stream.mjpg \
        --scenario configs/20_walls_alt_24_dy140.json \
        --frames 12 --out data/fpga/calib_homography.json \
        --save-overlay data/fpga/calib/overlay.png
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

FW, FH = 640, 480          # 正射图尺寸（= 页面尺寸 = 场地宽度）


# ----------------------------------------------------------------------
def grab_frames(url: str, n: int, timeout: float = 20.0) -> list[np.ndarray]:
    """从 MJPEG 流抓 n 帧（也可以传视频文件路径）。"""
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        return []
    frames: list[np.ndarray] = []
    t0 = time.time()
    while len(frames) < n and time.time() - t0 < timeout:
        ok, fr = cap.read()
        if not ok or fr is None:
            time.sleep(0.05)
            continue
        frames.append(fr)
    cap.release()
    return frames


def order_quad(pts: np.ndarray) -> np.ndarray:
    """把 4 个点排成 左上/右上/右下/左下。"""
    pts = pts.reshape(4, 2).astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)
    out = np.zeros((4, 2), np.float32)
    out[0] = pts[np.argmin(s)]
    out[1] = pts[np.argmin(d)]      # x - y 最小 → 右上
    out[2] = pts[np.argmax(s)]
    out[3] = pts[np.argmax(d)]      # x - y 最大 → 左下
    return out


def find_page_quad(img: np.ndarray) -> tuple[np.ndarray | None, str]:
    """找页面白框的四角（粗白框 → 阈值 + 形态学 + 四边形逼近）。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    best = None
    best_area = 0.0
    for thr in (200, 185, 170, 155):
        m = (gray > thr).astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 0.12 * img.shape[0] * img.shape[1]:
                continue
            peri = cv2.arcLength(c, True)
            for eps in (0.02, 0.03, 0.05):
                ap = cv2.approxPolyDP(c, eps * peri, True)
                if len(ap) != 4 or not cv2.isContourConvex(ap):
                    continue
                q = order_quad(ap)
                w = (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[3])) / 2
                h = (np.linalg.norm(q[3] - q[0]) + np.linalg.norm(q[2] - q[1])) / 2
                if w < 80 or h < 60:
                    continue
                ar = w / max(1e-6, h)
                if not (1.05 <= ar <= 1.75):      # 页面 4:3 ≈ 1.33
                    continue
                if area > best_area:
                    best_area, best = area, q
        if best is not None:
            return best, f"thr={thr}"
    return None, "not_found"


def warp_page(img: np.ndarray, quad: np.ndarray) -> np.ndarray:
    dst = np.array([[0, 0], [FW - 1, 0], [FW - 1, FH - 1], [0, FH - 1]], np.float32)
    M = cv2.getPerspectiveTransform(quad, dst)
    return cv2.warpPerspective(img, M, (FW, FH), flags=cv2.INTER_LINEAR)


def detect_player(warped: np.ndarray) -> tuple[float, float] | None:
    """白点 = 内区最亮的圆形小连通域（排除边框/墙）。"""
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    m = (gray > 215).astype(np.uint8) * 255
    m[:14, :] = 0
    m[-14:, :] = 0
    m[:, :14] = 0
    m[:, -14:] = 0
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if not (60 <= area <= 1500):
            continue
        if w < 6 or h < 6 or w > 40 or h > 40:
            continue
        if abs(w - h) > max(4, 0.5 * max(w, h)):      # 近似圆
            continue
        d = math.hypot(cents[i][0] - FW / 2, cents[i][1] - FH / 2)
        cand = (d, cents[i][0], cents[i][1])
        if best is None or cand < best:
            best = cand
    return None if best is None else (float(best[1]), float(best[2]))


def detect_walls(warped: np.ndarray, bright_frac: float = 0.45,
                 min_gap: int = 20) -> list[dict]:
    """墙行 + 缺口：亮行成带，带内找最长暗段当缺口（口径与板端 VT 一致）。

    返回每道墙：``{"y": 480-band_cy, "gl": 缺口左, "gr": 缺口右}``（y 用 VT 口径，
    即从可见带下沿往上数的像素）。
    """
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    inner = gray[10:FH - 10, 10:FW - 10]
    bright = (inner > 150).astype(np.uint8)
    frac = bright.mean(axis=1)
    rows = np.where(frac > bright_frac)[0]
    if rows.size == 0:
        return []
    bands: list[list[int]] = []
    for r in rows:
        if bands and r - bands[-1][-1] <= 2:
            bands[-1].append(int(r))
        else:
            bands.append([int(r)])
    out: list[dict] = []
    for b in bands:
        if len(b) < 2:                    # 1~2 px 的多半是网格线
            continue
        r0, r1 = b[0], b[-1]
        band = bright[r0:r1 + 1].max(axis=0)      # 带内每列是否亮
        #: 缺口 = 最长暗段
        dark = (band == 0).astype(np.int8)
        best_len, best_s = 0, -1
        s = -1
        for x, v in enumerate(dark):
            if v and s < 0:
                s = x
            if (not v) and s >= 0:
                if x - s > best_len:
                    best_len, best_s = x - s, s
                s = -1
        if s >= 0 and len(dark) - s > best_len:
            best_len, best_s = len(dark) - s, s
        gl = gr = None
        if best_len >= min_gap and best_s > 5 and best_s + best_len < len(dark) - 5:
            gl = best_s + 10
            gr = best_s + best_len + 10
        cy = 10 + 0.5 * (r0 + r1)
        out.append({"y": round(FH - cy, 1), "gl": gl, "gr": gr,
                    "band": [10 + r0, 10 + r1]})
    return out


def match_rows(walls: list[dict], truth_y: list[float], tol: float = 25.0) -> list[dict]:
    out = []
    for w in walls:
        t = min(truth_y, key=lambda v: abs(v - w["y"]))
        if abs(t - w["y"]) <= tol:
            out.append({"measured_y": w["y"], "true_y": t, "dy": round(w["y"] - t, 1),
                        "gl": w["gl"], "gr": w["gr"]})
    return out


def robust_line(xs: list[float], ys: list[float], iters: int = 3,
                tol: float = 30.0) -> tuple[float, float, int, float]:
    """y = a*x + b 的鲁棒拟合，返回 (a, b, n, rmse)。"""
    a, b = 1.0, 0.0
    idx = list(range(len(xs)))
    for _ in range(iters):
        if len(idx) < 3:
            break
        sel = [i for i in idx if abs((a * xs[i] + b) - ys[i]) <= tol] or idx
        A = np.array([[xs[i], 1.0] for i in sel])
        Y = np.array([ys[i] for i in sel])
        sol, *_ = np.linalg.lstsq(A, Y, rcond=None)
        a, b = float(sol[0]), float(sol[1])
        idx = sel
    err = [(a * xs[i] + b) - ys[i] for i in idx]
    rmse = math.sqrt(sum(e * e for e in err) / max(1, len(err)))
    return a, b, len(idx), rmse


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="OpenCV 透视变换标定（白框→单应→系数）")
    ap.add_argument("--url", default="http://192.168.137.242:8099/stream.mjpg")
    ap.add_argument("--scenario", default="configs/20_walls_alt_24_dy140.json")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--out", default="data/fpga/calib_homography.json")
    ap.add_argument("--save-overlay", default="data/fpga/calib/overlay.png")
    ap.add_argument("--save-warped", default="data/fpga/calib/warped.png")
    a = ap.parse_args(argv)

    cfg = json.loads(Path(a.scenario).read_text(encoding="utf-8"))
    truth_y = [float(v) for v in (cfg.get("meta", {}).get("walls_y") or [])]
    if not truth_y:
        print("场景没有 meta.walls_y，无法标定")
        return 2

    frames = grab_frames(a.url, a.frames)
    print(f"抓到 {len(frames)} 帧（{a.url}）")
    if not frames:
        return 3

    ok_quad = 0
    all_walls: list[dict] = []
    warped_best = None
    quad_best = None
    players: list[tuple[float, float]] = []
    for i, fr in enumerate(frames):
        quad, how = find_page_quad(fr)
        if quad is None:
            print(f"  帧{i}: 没找到白框（{how}）")
            continue
        ok_quad += 1
        wp = warp_page(fr, quad)
        walls = detect_walls(wp)
        pl = detect_player(wp)
        if pl:
            players.append(pl)
        if warped_best is None or len(walls) > len(all_walls):
            warped_best = wp
            quad_best = quad
            all_walls = walls
    print(f"白框识别 {ok_quad}/{len(frames)} 帧；最多一帧里测到 {len(all_walls)} 道墙"
          f"；角色 {len(players)} 帧")
    if warped_best is None:
        return 4

    matched = match_rows(all_walls, truth_y)
    print(f"与真值行匹配上 {len(matched)} 道（|Δy|≤25）：")
    for m in matched:
        print(f"   实测 y={m['measured_y']:7.1f}  真值 {m['true_y']:6.0f}  "
              f"Δy={m['dy']:+6.1f}  gap {m['gl']}-{m['gr']}")

    #: 恒等映射（正射图 1 px = 1 单位）下的残差系数
    coef = {"scale": 1.0, "bias": 0.0, "samples": 0, "rmse": None}
    if len(matched) >= 3:
        xs = [m["measured_y"] for m in matched]
        ys = [m["true_y"] for m in matched]
        sc, bi, n, rmse = robust_line(xs, ys)
        coef = {"scale": round(sc, 5), "bias": round(bi, 3),
                "samples": n, "rmse": round(rmse, 2)}
        print(f"残差拟合：true = {sc:.5f}*measured + {bi:.3f}  "
              f"(n={n}, rmse={rmse:.2f})")
        print("  说明：scale≈1、bias≈0 表示四边形已经找对；偏离大说明白框/透视还没对准")

    res = {
        "quad": None if quad_best is None else [[round(float(x), 1), round(float(y), 1)]
                                                for x, y in quad_best],
        "scale": coef["scale"], "bias": coef["bias"],
        "rows_matched": len(matched), "fit": coef,
        "players": [[round(p[0], 1), round(p[1], 1)] for p in players[:5]],
        "walls": all_walls,
        "matched": matched,
        "url": a.url, "scenario": a.scenario,
    }
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"写出 {a.out}")

    if warped_best is not None:
        Path(a.save_warped).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(a.save_warped, warped_best)
        ov = warped_best.copy()
        for w in all_walls:
            cv2.line(ov, (0, int(FH - w["y"])), (FW - 1, int(FH - w["y"])), (0, 220, 0), 1)
            if w["gl"] is not None:
                cv2.rectangle(ov, (int(w["gl"]), int(FH - w["y"]) - 6),
                              (int(w["gr"]), int(FH - w["y"]) + 6), (0, 160, 255), 1)
        for t in truth_y:
            r = int(FH - t)
            if 0 <= r < FH:
                cv2.line(ov, (0, r), (FW - 1, r), (0, 255, 255), 1)
        for p in players[:1]:
            cv2.circle(ov, (int(p[0]), int(p[1])), 12, (255, 0, 255), 2)
        cv2.putText(ov, f"scale={coef['scale']:.3f} bias={coef['bias']:.2f} "
                        f"rows={len(matched)}", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imwrite(a.save_overlay, ov)
        print(f"写出 overlay {a.save_overlay} / warped {a.save_warped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
