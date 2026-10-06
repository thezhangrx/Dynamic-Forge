#!/usr/bin/env python3
"""实时视频调用入口 —— 连续采集摄像头并在窗口里显示。

    python vision_live.py                                  # /dev/video0，640x480
    python vision_live.py --device 0 --width 1280 --height 720
    python vision_live.py --headless --seconds 5           # 不开窗，只测帧率
    python vision_live.py --fourcc YUYV                    # 换格式（免 JPEG 解码）

按仓库约定，根目录只放"调用代码"，实现全在 ``core/vision/live_view.py``。

按键：`ESC` / `q` 退出，`h` 切换左上角的帧率读数。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

# 实现都在 core/ 下，先把 core/ 放进 sys.path
_CORE = pathlib.Path(__file__).resolve().parent / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from vision.live_view import DEFAULT_WINDOW, CameraSource, run  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="实时视频：连续采集 + 窗口显示")
    ap.add_argument("--device", type=int, default=0, help="V4L2 设备号（默认 0）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=None, help="请求帧率（不填由驱动决定）")
    ap.add_argument("--fourcc", default="MJPG", help='像素格式（MJPG / YUYV / "" 表示不动）')
    ap.add_argument("--window", default=DEFAULT_WINDOW, help="窗口标题")
    ap.add_argument("--headless", action="store_true", help="不开窗，只采集并打印帧率")
    ap.add_argument("--no-overlay", action="store_true", help="不画左上角读数")
    ap.add_argument("--seconds", type=float, default=None, help="跑这么多秒后自动退出")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    source = CameraSource(args.device, width=args.width, height=args.height,
                          fps=args.fps, fourcc=args.fourcc)
    try:
        source.open()
    except RuntimeError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2

    print(f"已打开：{source.describe()}")
    if args.fourcc:
        print(f"  请求 {args.fourcc} {args.width}x{args.height}"
              + (f" @{args.fps}fps" if args.fps else ""))
    if not args.headless:
        print("  按 ESC 或 q 退出，h 切换读数")

    try:
        return run(source, window=args.window, headless=args.headless,
                   overlay=not args.no_overlay, seconds=args.seconds)
    finally:
        source.close()


if __name__ == "__main__":
    raise SystemExit(main())
