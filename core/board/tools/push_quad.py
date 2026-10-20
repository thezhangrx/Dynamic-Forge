#!/usr/bin/env python3
"""用现有 ArUco 标记算出**页面四角**，下发给板端 VT 当正射四边形。

为什么需要
----------
板端 `vt_board` 自己的白框检测在真机相机上会退化成"整帧当页面"
（实测 `/tmp/qb.txt` = (1,1)-(639,479)），于是墙的场地 y 系统性偏 80~130、
缺口配错行。这里用仓库现成的 `standard.fiducial` + `vision.fiducial`
（ArUco 检测 + 单应求解）算出页面的真实四角，写进板端 `/tmp/quad.json`，
`vt_board --quad-file` 会优先用它 --- 只下发**几何参数**，不下发图像、不下发真值。

用法
----
    Env/bin/python core/board/tools/push_quad.py \
        --url http://192.168.137.242:8099/stream.mjpg \
        --board root@192.168.137.242 --key data/fpga/board_ssh/board_key \
        --marker-side 48 --in-scale 0.5
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core"))

from vision.primitives import Frame            # noqa: E402
from vision import fiducial as Fid             # noqa: E402
from standard.fiducial import FiducialLayout   # noqa: E402


def grab(url: str, want: int = 1, tries: int = 40) -> np.ndarray | None:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        return None
    best = None
    best_n = -1
    for _ in range(tries):
        ok, fr = cap.read()
        if not ok or fr is None:
            continue
        best = fr if best is None else best
        if want <= 1:
            break
    cap.release()
    return best


def to_frame(img: np.ndarray) -> Frame:
    return Frame(width=img.shape[1], height=img.shape[0],
                 r=img[:, :, 2].tobytes(), g=img[:, :, 1].tobytes(),
                 b=img[:, :, 0].tobytes())


def page_quad(frames: list[np.ndarray], marker_side: float,
              min_markers: int) -> tuple[np.ndarray | None, dict]:
    """多帧尝试，取标记最多/残差最小的一次；返回页面四角（输入图像坐标）。"""
    best = None
    for fr in frames:
        obs = Fid.detect_markers(to_frame(fr))
        if len(obs) < min_markers:
            continue
        layout = FiducialLayout(field_w=640.0, field_h=480.0,
                                marker_side=marker_side)
        try:
            calib, rep = Fid.calibrate_from_fiducials(obs, layout,
                                                   (fr.shape[1], fr.shape[0]),
                                                    min_markers=min_markers)
        except Exception as exc:                            # noqa: BLE001
            print(f"    标定失败：{type(exc).__name__}: {exc}")
            continue
        H = np.array(calib.H, dtype=float).reshape(3, 3)
        Hi = np.linalg.inv(H)
        #: 页面(0..639, 0..479) → 相机像素（H 的世界是 band 局部坐标，
        #: 只差一个 y 平移，对"页面四角"没有影响）
        #: 顺序必须与 vt_board 的 Quad 一致：图像里的 左上→右上→右下→左下。
        #: H 是 图像→世界(y 向上)，所以对应的世界四角是：
        #:   左上=(0,479) 右上=(639,479) 右下=(639,0) 左下=(0,0)
        page = np.array([[0, 479], [639, 479], [639, 0], [0, 0]], np.float32)
        cam = cv2.perspectiveTransform(page.reshape(-1, 1, 2), Hi).reshape(-1, 2)
        rms = float(getattr(rep, "rms_px", 1e9))
        if best is None or rms < best[1]["rms_px"]:
            best = (cam, {"rms_px": rms, "n_markers": len(obs),
                          "ids": sorted(o.id for o in obs)})
    return (None, {}) if best is None else best


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="用 ArUco 标记算页面四角并下发板端")
    ap.add_argument("--url", default="http://192.168.137.242:8099/stream.mjpg")
    ap.add_argument("--board", default="root@192.168.137.242")
    ap.add_argument("--key", default=str(ROOT / "data/fpga/board_ssh/board_key"))
    ap.add_argument("--marker-side", type=float, default=48.0)
    ap.add_argument("--min-markers", type=int, default=2)
    ap.add_argument("--tries", type=int, default=8)
    ap.add_argument("--in-scale", type=float, default=0.5,
                    help="相机帧 → 板端 VT 输入坐标的缩放（ffmpeg 640×480→320×240 即 0.5）")
    ap.add_argument("--remote", default="/tmp/quad.json")
    ap.add_argument("--local", default=str(ROOT / "data/fpga/quad.json"))
    ap.add_argument("--no-push", action="store_true")
    a = ap.parse_args(argv)

    #: 单连接持续读（每次重连都会拿到缓冲里的旧帧，会一直失败）：
    #: 最多试 tries 帧、每帧间隔 0.25 s，一旦能解出标定就停。
    cap = cv2.VideoCapture(a.url)
    if not cap.isOpened():
        print(f"打不开流 {a.url}")
        return 2
    cam, info = None, {}
    for i in range(max(1, a.tries * 4)):
        ok, fr = cap.read()
        if not ok or fr is None:
            time.sleep(0.05)
            continue
        cam, info = page_quad([fr], a.marker_side, a.min_markers)
        if cam is not None:
            break
        time.sleep(0.25)
    cap.release()
    if cam is None:
        print("标记不足，算不出页面四角（先确认页面开着标记、相机能看到）")
        return 4
    print(f"标记 {info['n_markers']} 枚 {info['ids']}  单应重投影 rms={info['rms_px']:.2f}px")
    quad_input = cam * float(a.in_scale)
    print("页面四角(相机像素):",
          [tuple(round(float(v), 1) for v in p) for p in cam])
    print(f"下发四边形(输入 {a.in_scale:g}×):",
          [tuple(round(float(v), 1) for v in p) for p in quad_input])

    text = " ".join(f"{v:.2f}" for v in quad_input.reshape(-1)) + "\n"
    Path(a.local).parent.mkdir(parents=True, exist_ok=True)
    Path(a.local).write_text(text, encoding="utf-8")
    if not a.no_push:
        try:
            res = subprocess.run(
                ["ssh", "-i", a.key, "-o", "StrictHostKeyChecking=no",
                 a.board, f"cat > {a.remote}"],
                input=text.encode(), capture_output=True, timeout=20)
            print(f"已写到板端 {a.remote} (rc={res.returncode})")
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"下发失败：{exc!r}")
            return 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
