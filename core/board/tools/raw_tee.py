#!/usr/bin/env python3
"""把 `cam_live.sh` 里 **喂给板子程序的同一份字节流** 旁路存成 PPM。

为什么需要它：之前我用自己的 ffmpeg 抓帧做离线分析，结果那批帧比 live 差很多
（噪声 17~19 vs 干净 2.0），于是"离线分析说不行、live 其实能跑"。这个 tee 放在
**管道中间**，存下来的就是板子真正看到的帧，离线结论才对得上 live。

用法（cam_live 里由 DUMP_DIR 触发）：
    ffmpeg ... -f rawvideo - | raw_tee.py --dir /tmp/dump --w 640 --h 360 | APP ...
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=360)
    ap.add_argument("--every", type=int, default=10, help="每 N 帧存一张")
    ap.add_argument("--max", type=int, default=20, help="最多存多少张")
    args = ap.parse_args(argv)

    out = Path(args.dir)
    out.mkdir(parents=True, exist_ok=True)
    frame = args.w * args.h * 3
    buf = bytearray()
    n = 0
    saved = 0
    stdout = sys.stdout.buffer
    while True:
        chunk = sys.stdin.buffer.read(frame)
        if len(chunk) < frame:
            break
        stdout.write(chunk)
        stdout.flush()
        if n % args.every == 0 and saved < args.max:
            path = out / f"{n:06d}.ppm"
            with path.open("wb") as fh:
                fh.write(b"P6\n%d %d\n255\n" % (args.w, args.h))
                fh.write(chunk)
            saved += 1
        n += 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
