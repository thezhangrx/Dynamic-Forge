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
    #: 存一张"帧序号 → 墙钟时刻"的表。**这是离线对账的钥匙**：
    #: 有了它，存下来的每张 PPM 都能和仿真真值（`unix` 字段）按时间对上，
    #: 于是"角色/缺口误差"可以在本地几秒一轮地迭代，而不必每次上板跑 90 秒。
    ap.add_argument("--stamp-file", default=None, help="写入 index<TAB>unix 的 TSV")
    args = ap.parse_args(argv)

    out = Path(args.dir)
    out.mkdir(parents=True, exist_ok=True)
    frame = args.w * args.h * 3
    buf = bytearray()
    n = 0
    saved = 0
    stamps = []
    stdout = sys.stdout.buffer
    # ⚠ 写 stamp 必须放在 `finally` 里：tee 夹在管道中间，下游一关（程序处理够帧数退出）
    #   我们写 stdout 就会 BrokenPipeError —— 原来的写法直接异常退出，**时间戳一个都没写**，
    #   于是离线语料没法按时间对真值（这一版之前就是这么静默失败的）。
    # ⚠ 被 `timeout` 杀时是 SIGTERM：不装处理函数的话 Python 直接没了，`finally`
    #   也不会跑 → 时间戳又白写（这一版之前踩过两次：第一次是 BrokenPipe，
    #   第二次是 SIGTERM）。装上处理器 + **每帧 append 并 flush**，这样最坏只丢一行。
    import signal as _signal

    def _on_term(_signum, _frame):
        raise SystemExit(0)

    for _sig in (_signal.SIGTERM, _signal.SIGINT):
        try:
            _signal.signal(_sig, _on_term)
        except (ValueError, OSError):
            pass

    stamp_fh = open(args.stamp_file, "a", encoding="utf-8") if args.stamp_file else None
    try:
        _pump(frame, args, out, stdout, stamps, stamp_fh)
    except BrokenPipeError:
        pass
    finally:
        if stamp_fh is not None:
            stamp_fh.close()
    return 0


def _pump(frame, args, out, stdout, stamps, stamp_fh=None):  # noqa: PLR0913
    n = 0
    saved = 0
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
            if stamp_fh is not None:
                import time as _time
                _ts = _time.time()
                stamps.append((n, _ts))
                stamp_fh.write(f"{n}\t{_ts:.6f}\n")
                stamp_fh.flush()
        n += 1


if __name__ == "__main__":
    raise SystemExit(main())
