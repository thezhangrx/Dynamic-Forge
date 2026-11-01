#!/usr/bin/env python3
"""auto_quad.py —— 从当前相机画面重算"页面四角"（不依赖 ArUco 一定可见）。

三条路，按可靠性从高到低依次尝试：

  1. **ArUco**（复用 push_quad.py 的检测）—— 有标记就用它，绝对度量，最准；
  2. **先验细化**（默认）—— 用现有 quad 当先验，沿四条边找亮线、拟合直线、求交点；
     相机只是挪了一点/缩放变了一点时非常稳（实测 3 次迭代收敛，偏差 ~6 px）；
  3. 都没成功就报错，并给出"把页面标记放大 / 把相机推近"的具体建议。

坐标口径：输出的四角是 **相机像素**（640×480），vt_sparse 吃 640×480 输入，
所以 --in-scale 默认 1.0（用 320×240 输入时才需要 0.5）。

用法：
    python3 core/board/tools/auto_quad.py [--prior data/fpga/quad.json] [--push]
        [--board root@192.168.137.242] [--key ...] [--url http://.../stream.mjpg]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

# --- 工程内路径引导（保证从工程任意位置启动都能定位依赖）---
_PRJ_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for _p in (os.path.join(_PRJ_ROOT, "lib"),
           os.path.join(_PRJ_ROOT, "4_virtual_platform", "sim"),
           os.path.dirname(os.path.abspath(__file__))):
    if _p not in sys.path:
        sys.path.insert(0, _p)


import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URL = "http://192.168.137.242:8099/stream.mjpg"


def grab(url: str, n: int = 6, timeout: float = 8.0) -> np.ndarray | None:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        return None
    t0 = time.time()
    best = None
    got = 0
    while time.time() - t0 < timeout and got < n:
        ok, fr = cap.read()
        if ok and fr is not None:
            best = fr
            got += 1
        else:
            time.sleep(0.05)
    cap.release()
    return best


def try_aruco(img: np.ndarray, marker_side: float) -> np.ndarray | None:
    """调用 push_quad.py 的检测（不推送），成功返回相机像素四角。"""
    tmp = ROOT / "Env" / "npu_build" / "quad_try.json"
    cmd = [str(ROOT / "Env/bin/python"), str(ROOT / "core/board/tools/push_quad.py"),
           "--url", DEFAULT_URL, "--no-push", "--marker-side", str(marker_side),
           "--in-scale", "1.0", "--local", str(tmp)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
    except Exception:
        return None
    if r.returncode != 0 or not tmp.exists():
        return None
    vals = [float(x) for x in tmp.read_text().split()]
    if len(vals) != 8:
        return None
    return np.array(vals, np.float32).reshape(4, 2)


def edge_points(g: np.ndarray, p, q, half=22, step=18, min_v=150.0):
    p = np.asarray(p, float); q = np.asarray(q, float)
    d = q - p
    L = float(np.linalg.norm(d))
    if L < 30:
        return None
    u = d / L
    n = np.array([-u[1], u[0]])
    pts = []
    for t in np.linspace(0.12, 0.88, max(6, int(L / step))):
        c = p + t * d
        best = None
        for s in np.linspace(-half, half, 2 * half + 1):
            x, y = c + n * s
            xi, yi = int(round(x)), int(round(y))
            if not (0 <= xi < g.shape[1] and 0 <= yi < g.shape[0]):
                continue
            v = g[yi, xi]
            if best is None or v > best[0]:
                best = (v, x, y)
        if best and best[0] >= min_v:
            pts.append((best[1], best[2]))
    return np.array(pts, np.float32) if len(pts) >= 8 else None


def refine(img: np.ndarray, prior: np.ndarray, iters: int = 3, half: int = 22):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    quad = prior.astype(np.float32).copy()
    for it in range(iters):
        lines = []
        for i in range(4):
            pts = edge_points(g, quad[i], quad[(i + 1) % 4], half=half)
            if pts is None:
                return None, f"第 {it} 次迭代：边 {i} 找不到足够亮点"
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            lines.append((np.array([x0, y0], float), np.array([vx, vy], float)))
        new = np.zeros((4, 2), np.float32)
        for i in range(4):
            p1, d1 = lines[(i - 1) % 4]
            p2, d2 = lines[i]
            A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
            try:
                t = np.linalg.solve(A, p2 - p1)
            except np.linalg.LinAlgError:
                return None, "边平行，求交点失败"
            new[i] = p1 + t[0] * d1
        shift = float(np.max(np.linalg.norm(new - quad, axis=1)))
        quad = new
        if shift < 0.5:
            break
    return quad, "ok"


def sane(q: np.ndarray, w: int, h: int) -> tuple[bool, str]:
    if q is None:
        return False, "无结果"
    if q.min() < -5 or q[:, 0].max() > w + 5 or q[:, 1].max() > h + 5:
        return False, "四角超出画面"
    area = 0.5 * abs(np.dot(q[:, 0], np.roll(q[:, 1], -1)) - np.dot(q[:, 1], np.roll(q[:, 0], -1)))
    if area < 0.12 * w * h:
        return False, "页面面积过小（%.0f px²，<12%% 画面）" % area
    if area > 0.97 * w * h:
        return False, "页面几乎占满整幅（%.0f%%）——多半是锁到了屏幕边框，不是页面白框" % (100 * area / (w * h))
    # 上边应比下边短（透视），允许小偏差
    top = np.linalg.norm(q[1] - q[0]); bot = np.linalg.norm(q[2] - q[3])
    if not (0.6 < top / max(bot, 1e-6) < 1.6):
        return False, "上下边长度比异常 %.2f" % (top / max(bot, 1e-6))
    return True, "面积 %.0f px²（占画面 %.0f%%）" % (area, 100 * area / (w * h))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--prior", default=str(ROOT / "data/fpga/quad.json"))
    ap.add_argument("--local", default=str(ROOT / "data/fpga/quad.json"))
    ap.add_argument("--board", default="root@192.168.137.242")
    ap.add_argument("--key", default=str(ROOT / "data/fpga/board_ssh/board_key"))
    ap.add_argument("--remote", default="/tmp/quad.json")
    ap.add_argument("--in-scale", type=float, default=1.0)
    ap.add_argument("--marker-side", type=float, default=28.0)
    ap.add_argument("--push", action="store_true")
    a = ap.parse_args()

    img = grab(a.url)
    if img is None:
        print("打不开画面流 %s（先确认 mjpeg_server 在跑）" % a.url)
        return 2
    h, w = img.shape[:2]
    print("画面 %dx%d" % (w, h))

    quad, how = None, ""
    q = try_aruco(img, a.marker_side)
    if q is not None:
        quad, how = q, "ArUco"
    else:
        print("ArUco 没认到标记 → 改用「先验细化」")
        prior_txt = Path(a.prior).read_text().split() if Path(a.prior).exists() else []
        if len(prior_txt) == 8:
            prior = np.array([float(v) for v in prior_txt], np.float32).reshape(4, 2) / a.in_scale
            quad, msg = refine(img, prior)
            how = "先验细化"
            print("  细化结果：", msg)
        else:
            print("  没有先验 quad，无法细化")
            return 4

    if quad is None:
        print("  先验细化失败：%s" % msg)
        print("建议：把 ArUco 标记放大（--fiducial-side ≥ 60）或把相机推近，再重试。")
        return 5
    ok, why = sane(quad, w, h)
    print("候选四角：", [tuple(round(float(v), 1) for v in p) for p in quad])
    print("合理性：", "通过" if ok else "不通过", "—", why)
    if not ok:
        print("建议：把页面上的 ArUco 标记放大（--fiducial-side ≥ 60）或把相机推近，再重试。")
        return 5

    out = quad * a.in_scale
    text = " ".join("%.2f" % v for v in out.reshape(-1)) + "\n"
    Path(a.local).write_text(text, encoding="utf-8")
    print("已写本地 %s" % a.local)
    if a.push:
        r = subprocess.run(["ssh", "-i", a.key, "-o", "StrictHostKeyChecking=no",
                            a.board, "cat > %s" % a.remote],
                           input=text, text=True, capture_output=True)
        if r.returncode != 0:
            print("推送到板子失败：", r.stderr.strip()[:200])
            return 6
        print("已推送 %s:%s（%s）" % (a.board, a.remote, how))
    return 0


if __name__ == "__main__":
    sys.exit(main())
