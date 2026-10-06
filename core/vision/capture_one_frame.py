"""抓拍一帧外接 USB 摄像头图像，保存到 ``data/vision/``。

    python core/vision/capture_one_frame.py
    python core/vision/capture_one_frame.py --device 0 --width 1280 --height 720
    python core/vision/capture_one_frame.py --out data/vision/mine.jpg

设备说明（``lsusb`` 实测）：
    0c45:64ab "Integrated Camera"  = 外接 USB 摄像头 -> /dev/video0（采集）
    1bcf:0b17 "USB Camera"         = 笔记本内置摄像头（名字有误导性）

输出文件名默认带日期戳（``frame_YYYYmmdd_HHMMSS.jpg``），不会覆盖上一张。
"""

from __future__ import annotations

import argparse
import pathlib
import time

import cv2

# core/vision/capture_one_frame.py -> parents[2] == <repo>
_REPO = pathlib.Path(__file__).resolve().parents[2]
_DEFAULT_DIR = _REPO / "data" / "vision"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="抓拍一帧并存到 data/vision/")
    ap.add_argument("--device", type=int, default=0, help="V4L2 设备号（默认 0）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--warmup", type=int, default=25,
                    help="先丢弃多少帧，等自动曝光/白平衡收敛")
    ap.add_argument("--out", type=pathlib.Path, default=None,
                    help="输出路径；默认 data/vision/frame_<日期戳>.jpg")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out = args.out or _DEFAULT_DIR / f"frame_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    # MJPG 下多数 USB 摄像头才能跑到 60fps；顺序必须在设置宽高之前。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[FAIL] 打不开 /dev/video{args.device}")
        return 2

    for _ in range(max(0, args.warmup)):
        cap.grab()
    ok, frame = cap.read()
    cap.release()

    if not ok or frame is None:
        print("[FAIL] 打开成功但读不到帧")
        return 3
    if not cv2.imwrite(str(out), frame):
        print(f"[FAIL] 写文件失败: {out}")
        return 4

    h, w = frame.shape[:2]
    print(f"[ OK ] {w}x{h} -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
