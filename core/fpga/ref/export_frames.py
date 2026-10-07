#!/usr/bin/env python3
"""把真实录像的若干帧导出成 **P5 PGM（8bit 灰度）**，作为 P2 的仿真激励。

为什么用真实录像而不是合成图
----------------------------
合成图不会有：压缩噪声、屏幕网格线的抗锯齿、相机/录屏的伽马、以及
真实场景里"墙被洗白到多少灰度"。用合成图做逐位比对，只能证明"我的 RTL 和我的
合成器一致"，证明不了"它对真实输入的行为和 Python 一致"。

用法::

    python core/fpga/ref/export_frames.py \
        --video data/vision/run4/screen.mp4 --out data/fpga/stim --frames 300 \
        [--start 0] [--stride 2] [--width 640]

产物::

    <out>/000000.pgm ...        灰度激励
    <out>/manifest.json         每帧的来源（视频、帧号、时间戳），可复现

**灰度还是彩色**：P2 的第一轮用灰度（体积 1/3、便于先打通），但灰度会让
`classify()` 的颜色判据（wall 的 R-B、green 的 G-max）退化 —— 这一点在
manifest 里显式标出，避免把"灰度下的指标"当成"算法在真实输入下的指标"。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys


def main(argv: list[str] | None = None) -> int:
    import cv2

    ap = argparse.ArgumentParser(description="导出灰度 PGM 作为 FPGA 仿真激励")
    ap.add_argument("--video", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--frames", type=int, default=300, help="导出多少帧")
    ap.add_argument("--start", type=int, default=0, help="起始原始帧号")
    ap.add_argument("--stride", type=int, default=1, help="每隔几帧取一帧")
    ap.add_argument("--width", type=int, default=640,
                    help="缩放到该宽度（保持比例）；0 = 原尺寸")
    ap.add_argument("--color", action="store_true",
                    help="导出彩色 PPM(P6)。**验墙必须要彩色** —— wall 判据是 R-B>40，"
                         "灰度下恒为 0，墙的列投影与缺口都测不出来")
    ap.add_argument("--gray", action="store_true",
                    help="强制灰度（默认跟 --color 走：不给 --color 就是灰度）")
    args = ap.parse_args(argv)

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.start:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start)

    args.out.mkdir(parents=True, exist_ok=True)
    manifest = {"video": str(args.video), "fps": fps, "total_frames": total,
                "start": args.start, "stride": args.stride,
                "mode": "color8" if args.color and not args.gray else "gray8",
                "frames": []}

    written = 0
    idx = args.start
    while written < args.frames:
        ok, bgr = cap.read()
        if not ok or bgr is None:
            break
        if (idx - args.start) % max(1, args.stride) == 0:
            want_color = bool(args.color) and not bool(args.gray)
            img = bgr if want_color else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            if args.width and img.shape[1] > args.width:
                s = args.width / float(img.shape[1])
                img = cv2.resize(img, (args.width, int(round(img.shape[0] * s))),
                                 interpolation=cv2.INTER_AREA)
            if want_color:
                name = f"{written:06d}.ppm"
                rgb = img[:, :, ::-1]              # OpenCV 是 BGR，PPM 是 RGB
                with open(args.out / name, "wb") as fp:
                    fp.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]))
                    fp.write(rgb.tobytes())
            else:
                name = f"{written:06d}.pgm"
                with open(args.out / name, "wb") as fp:
                    fp.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
                    fp.write(img.tobytes())
            manifest["frames"].append({
                "file": name, "src_frame": idx, "t": round(idx / fps, 4),
                "w": int(img.shape[1]), "h": int(img.shape[0]),
            })
            written += 1
        idx += 1
    cap.release()

    (args.out / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"导出 {written} 帧 -> {args.out}  ({args.width or 'orig'} 宽, "
          f"{'彩色' if args.color and not args.gray else '灰度'})")
    if written < args.frames:
        print(f"⚠ 只导出了 {written}/{args.frames} 帧（视频到末尾了）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
