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


def frame_is_usable(bgr, *, min_mean: float = 8.0, max_mean: float = 250.0) -> bool:
    """这一帧是不是"能用的画面"。

    判据来自实测的两个真实故障：

    * **全白**：相机的自动曝光被拉满（或链路出问题）时，整帧饱和成纯白，
      均值 ~254。此时任何阈值检测都会把整幅图当成目标。
    * **全黑**：镜头被挡住 / 隐私快门关闭（笔记本内置摄像头实测如此）。

    这只挡"整帧废掉"的情况，不做内容判断 —— 内容判断属于具体算法。
    """
    import numpy as np
    a = np.asarray(bgr)
    if a.size == 0:
        return False
    mean = float(a.mean())
    return min_mean <= mean <= max_mean


def capture_bgr(device: int = 0, width: int = 640, height: int = 480,
                warmup: int = 25, *, tries: int = 5):
    """抓一帧 BGR 图像（``numpy`` 数组）；失败返回 ``None``。

    单独抽出来是为了让 `vision_calibrate.py --capture` 复用同一条采集路径 ——
    采集参数（MJPG 必须在宽高之前设、warmup 帧数）只在这里定义一次。

    ``tries`` 次里挑第一帧"画面正常、且和上一帧不同"的：只读一帧的话，
    可能拿到自动曝光还没收敛的画面，或者链路卡住重复吐出的同一帧
    （实测出现过整帧纯白、连续 12 帧逐字节相同的情况）。
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    # MJPG 下多数 USB 摄像头才能跑到 60fps；顺序必须在设置宽高之前。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        cap.release()
        return None

    for _ in range(max(0, warmup)):
        cap.grab()
    prev = None
    good = None
    for _ in range(max(1, tries)):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        if frame_is_usable(frame):
            good = frame
            # 再确认画面在变：卡住的链路会重复吐同一帧。
            if prev is not None and not (frame == prev).all():
                break
        prev = frame
    cap.release()
    return good


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out = args.out or _DEFAULT_DIR / f"frame_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)

    frame = capture_bgr(args.device, args.width, args.height, args.warmup)
    if frame is None:
        print(f"[FAIL] 打不开 /dev/video{args.device} 或读不到帧")
        return 2
    if not cv2.imwrite(str(out), frame):
        print(f"[FAIL] 写文件失败: {out}")
        return 4

    h, w = frame.shape[:2]
    print(f"[ OK ] {w}x{h} -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
