#!/usr/bin/env python3
"""focus_check.py —— 开跑前的"对焦/取景"交互检查（键盘可调）。

背景：这颗相机**没有** V4L2 focus 控制（`v4l2-ctl --list-ctrls` 里 0 个 focus 项），
所以"对焦"只能拧镜头；这个程序负责**把画面和指标摆在眼前**，让你边拧边看，
并且能顺手调几项软件参数。

按键：
  [ / ]   相机 sharpness -1 / +1（软件锐化，0–7）
  - / +   数字变焦（中心裁切放大，1.0–2.0）
  1 / 2   曝光 -10 / +10（绝对曝光值，自动曝光下无效）
  w       切换白平衡自动/手动
  s       存一张快照 + 打印指标
  p       重新判定 PASS/FAIL
  q/ESC   退出（打印最终结论并写 data/fpga/focus.json）

用法：
  python3 core/board/tools/focus_check.py
  python3 core/board/tools/focus_check.py --url http://192.168.137.242:8099/stream.mjpg
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core/board/tools"))
from vision_health import metrics, verdict   # noqa: E402

BOARD = "root@192.168.137.242"
KEY = str(ROOT / "data/fpga/board_ssh/board_key")


def v4l2(args: str) -> str:
    cmd = ["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no", BOARD,
           "v4l2-ctl -d /dev/video0 " + args]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        return (r.stdout or r.stderr).strip()
    except Exception as exc:      # noqa: BLE001
        return "ssh 失败: %r" % (exc,)


def get_ctrl(name: str) -> int | None:
    txt = v4l2("--get-ctrl=%s" % name)
    try:
        return int(txt.split(":")[1].strip())
    except Exception:             # noqa: BLE001
        return None


def set_ctrl(name: str, val: int) -> str:
    return v4l2("--set-ctrl=%s=%d" % (name, val))


def draw(img: np.ndarray, m: dict, ok: bool, bad: list[str], hud: str) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]
    color = (0, 200, 0) if ok else (0, 0, 255)
    cv2.rectangle(out, (0, 0), (w - 1, 58), (0, 0, 0), -1)
    cv2.putText(out, "edge %.2fpx  lap %.0f  sat %.2f%%  occ %.0f%%  dark %.0f" % (
        m.get("sharp_edge", -1), m.get("lap_var", 0), 100 * m.get("sat_ratio", 0),
        100 * m.get("frame_occ", 0), m.get("dark_med", -1)),
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, ("PASS" if ok else "FAIL: " + "; ".join(bad))[:90],
                (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if hud:
        cv2.putText(out, hud, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1, cv2.LINE_AA)
    box = m.get("frame_box")
    if box:
        cv2.rectangle(out, (box[0], box[1]), (box[2], box[3]), (0, 180, 255), 1)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.242:8099/stream.mjpg")
    ap.add_argument("--no-window", action="store_true")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.url)
    if not cap.isOpened():
        print("打不开画面流 %s（先起板端 mjpeg_server）" % a.url)
        return 2

    zoom = 1.0
    sharp = get_ctrl("sharpness")
    expo = get_ctrl("exposure_absolute")
    wb_auto = get_ctrl("white_balance_temperature_auto")
    hud = "sharp=%s expo=%s wb_auto=%s zoom=%.1f  [ ]/-/+ 1/2 w s p q" % (
        sharp, expo, wb_auto, zoom)
    final = {"pass": False, "problems": [], "metrics": {}, "ts": time.time()}

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("取帧失败"); break
        if zoom > 1.0:
            h, w = frame.shape[:2]
            cw, ch = int(w / zoom), int(h / zoom)
            x0, y0 = (w - cw) // 2, (h - ch) // 2
            frame = cv2.resize(frame[y0:y0 + ch, x0:x0 + cw], (w, h))
        m = metrics(frame)
        good, bad = verdict(m)
        final.update({"pass": good, "problems": bad, "metrics": m})
        if a.no_window:
            print(json.dumps({"metrics": m, "pass": good, "problems": bad}, ensure_ascii=False))
            break
        cv2.imshow("focus_check", draw(frame, m, good, bad, hud))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("["):
            sharp = max(0, (sharp or 0) - 1); print(set_ctrl("sharpness", sharp)); hud = hud.split(" zoom")[0] + " zoom=%.1f" % zoom
        elif k == ord("]"):
            sharp = min(7, (sharp or 0) + 1); print(set_ctrl("sharpness", sharp))
        elif k == ord("-"):
            zoom = max(1.0, round(zoom - 0.1, 1))
        elif k == ord("+"):
            zoom = min(2.0, round(zoom + 0.1, 1))
        elif k == ord("1"):
            expo = max(10, (expo or 156) - 10); print(set_ctrl("exposure_absolute", expo))
        elif k == ord("2"):
            expo = min(2500, (expo or 156) + 10); print(set_ctrl("exposure_absolute", expo))
        elif k == ord("w"):
            wb_auto = 0 if wb_auto else 1
            print(set_ctrl("white_balance_temperature_auto", wb_auto))
        elif k == ord("s"):
            p = ROOT / "data/fpga/focus_%d.jpg" % int(time.time())
            cv2.imwrite(str(p), frame)
            print("快照", p, json.dumps(m, ensure_ascii=False))
        hud = "sharp=%s expo=%s wb_auto=%s zoom=%.1f  [ ]/-/+ 1/2 w s p q" % (
            sharp, expo, wb_auto, zoom)

    cap.release()
    cv2.destroyAllWindows()
    out = ROOT / "data/fpga/focus.json"
    out.write_text(json.dumps(final, ensure_ascii=False, indent=2), encoding="utf-8")
    print(("✅ PASS" if final["pass"] else "❌ FAIL: " + "；".join(final["problems"])))
    print("指标已写", out)
    return 0 if final["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
