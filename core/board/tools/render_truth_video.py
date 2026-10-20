#!/usr/bin/env python3
"""把仿真 `--state-log`（JSONL 真值）渲染成干净的平台视频。

不是势场图：只有场地白框、墙、角色、目标绿圈 —— 用来和"相机+视觉检测"视频并排看。
    python3 render_truth_video.py --state /tmp/truth.jsonl --out /tmp/truth.mp4 --fps 20
"""
from __future__ import annotations
import argparse, json, math
import cv2
import numpy as np

FIELD_W, FIELD_H = 640.0, 480.0
GOAL = (320.0, 470.0)
GOAL_R = 24.0
R_P = 10.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--out", default="/tmp/truth.mp4")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--every", type=int, default=3, help="每 N 条真值取一帧")
    a = ap.parse_args()
    rows = []
    for line in open(a.state, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    if not rows:
        print("真值日志为空")
        return 2
    vw = None
    n = 0
    for i in range(0, len(rows), max(1, a.every)):
        r = rows[i]
        img = np.zeros((480, 640, 3), np.uint8)
        img[:] = (34, 40, 55)
        cv2.rectangle(img, (1, 1), (638, 478), (255, 255, 255), 2)
        for o in r.get("obstacles", []):
            x, y, hw, hh = o[0], o[1], o[2], o[3]
            if hw <= hh:
                continue
            x0 = int(round(x - hw)); x1 = int(round(x + hw))
            y0 = int(round(FIELD_H - (y + hh))); y1 = int(round(FIELD_H - (y - hh)))
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 120, 255), -1)  # BGR 橙（与平台一致）
        p = r.get("player", [0, 0, R_P])
        cv2.circle(img, (int(round(p[0])), int(round(FIELD_H - p[1]))), int(R_P),
                   (255, 255, 255), -1)
        cv2.circle(img, (int(GOAL[0]), int(FIELD_H - GOAL[1])), int(GOAL_R), (60, 230, 120), 2)
        cv2.putText(img, f"sim truth  t={r.get('t', 0):.2f}s  frame={r.get('frame', i)}",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        if vw is None:
            vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                 float(a.fps), (640, 480))
        vw.write(img)
        n += 1
    if vw:
        vw.release()
    print(f"输出 {a.out}  帧 {n}  fps {a.fps}")


if __name__ == "__main__":
    raise SystemExit(main())
