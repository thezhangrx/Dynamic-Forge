#!/usr/bin/env python3
"""把 `vt_from_vision --jsonl`（视觉转换出的 VT）渲染成干净视频。"""
from __future__ import annotations
import argparse, json
import cv2
import numpy as np

FW, FH = 640.0, 480.0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", required=True)
    ap.add_argument("--out", default="/tmp/vision_vt.mp4")
    ap.add_argument("--fps", type=float, default=25.0)
    a = ap.parse_args()
    rows = []
    for line in open(a.jsonl, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    vw = None
    for i, r in enumerate(rows):
        img = np.zeros((480, 640, 3), np.uint8)
        img[:] = (34, 40, 55)
        cv2.rectangle(img, (1, 1), (638, 478), (255, 255, 255), 2)
        walls = r.get("walls", [])
        for w in walls:
            y = w["y"]; gl = w["gl"]; gr = w["gr"]
            y0 = int(round(FH - (y + 4))); y1 = int(round(FH - (y - 4)))
            if gl > 1:
                cv2.rectangle(img, (0, y0), (int(round(gl)), y1), (0, 120, 255), -1)
            if FW - gr > 1:
                cv2.rectangle(img, (int(round(gr)), y0), (639, y1), (0, 120, 255), -1)
        p = r.get("player")
        if p:
            cv2.circle(img, (int(round(p[0])), int(round(FH - p[1]))), 10, (255, 255, 255), -1)
        cv2.circle(img, (320, int(FH - 470)), 24, (60, 230, 120), 2)
        cv2.putText(img, f"vision->VT  frame={r.get('frame')}  walls={len(walls)}  "
                         f"markers={r.get('markers')} extent={r.get('extent')}  player={p}",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        if vw is None:
            vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                 float(a.fps), (640, 480))
        vw.write(img)
    if vw:
        vw.release()
    print(f"输出 {a.out} 帧 {len(rows)}")

if __name__ == "__main__":
    raise SystemExit(main())
