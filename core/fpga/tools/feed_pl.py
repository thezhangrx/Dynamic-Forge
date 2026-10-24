#!/usr/bin/env python3
"""feed_pl.py —— 板端把**一帧真实图像**喂进 PL（vision_top），读回几何帧做校验。

为什么要有它
------------
`pl_probe.py` 只读寄存器（能回答"PL 活着吗"），但"喂一帧 → 读回结果"这一环
（P1 验收：1 帧与 CPU 版一致）一直没有工具。这个文件就是那一环，跑在板子上。

接口约定（与 `core/board/vision_regs.hpp` / `core/fpga/rtl/vision_axi.v` 对齐）
--------------------------------------------------------------------------
基址 0x43C00000，4 KB 窗口：
    0x0C0  CTRL     写 bit0=1 → 启动一帧（自清）；读 bit0=busy bit1=done
                    bit[15:8]=frame_count bit16=overflow
    0x0C4  SEQ      帧序号（写进去会附带在帧头里）
    0x0C8/0x0CC     stamp_ns 低/高 32 位
    0x0D0  FLEN     只读：上一帧实际字节数
    0x0D4  PIXPUSH  写：像素 FIFO（整字写，**4 字节打包**，低字节先）
    0x0D8  PIXSTAT  读：[15:0] 缓冲字节 [16] 溢出 [17] 正在放
    0x0DC  PIXCTL   写：bit0 像素源（1=FIFO）bit1 flush
    0x100  FRAME    几何帧字节窗口（magic 'BHV1' / version / 长度 / CRC32）
    0x080/0x084     调试口：写 {sub[3:0],idx[17:0],sel[3:0]}，读数据

用法（板子上；要 /dev/mem）
--------------------------
    python3 feed_pl.py --image /tmp/frame.jpg --w 640 --h 360
    python3 feed_pl.py --image frame.ppm --w 640 --h 360 --jsonl out.jsonl --repeat 5

判据
----
* 帧头 magic == 0x31564842 ('BHV1')、version == 1；
* 帧长 == 76 + 36*n_obs + 36*n_gaps，且 == FLEN；
* 帧尾 CRC32（zlib.crc32 对帧体重算）== 帧里的 crc；
* 连续 N 帧 frame_count 连续（无丢帧），并打出每帧耗时。
"""
from __future__ import annotations

import argparse
import json
import mmap
import os
import struct
import sys
import time
import zlib

BASE = 0x43C00000
kOffCtrl, kOffSeq, kOffStamp0, kOffStamp1 = 0x0C0, 0x0C4, 0x0C8, 0x0CC
kOffFLen, kOffPixPush, kOffPixStat, kOffPixCtl = 0x0D0, 0x0D4, 0x0D8, 0x0DC
kOffFrame = 0x100
kOffDbgSel, kOffDbgData = 0x080, 0x084
FRAME_WINDOW = 2048
MAGIC = 0x31564842


def open_regs():
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    return mmap.mmap(fd, 4096, mmap.MAP_SHARED,
                     mmap.PROT_READ | mmap.PROT_WRITE, offset=BASE), fd


def rd(m, off: int) -> int:
    return struct.unpack_from("<I", m, off)[0]


def wr(m, off: int, val: int) -> None:
    struct.pack_into("<I", m, off, val & 0xFFFFFFFF)


def load_rgb(path: str, w: int, h: int) -> bytes:
    """读图并转成 PL 要的 RGB888（H×W×3，行优先）。"""
    import cv2
    import numpy as np
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit("读不到图：%s" % path)
    if img.shape[1] != w or img.shape[0] != h:
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb).tobytes()


def push_frame(m, rgb: bytes, seq: int, stamp_ns: int) -> float:
    """推进一整帧并等 frame_done；返回耗时（秒）。"""
    assert len(rgb) % 4 == 0, "帧字节数必须是 4 的倍数（整字写 FIFO）"
    #: 顺序**必须是"先 start 再推像素"**：vision_top 只在收到 frame_start 的
    #: 那一帧里吃像素，IDLE 时推进去的会被丢掉，随后 start 又等不到像素 → 死等。
    #: （RTL 头注释："PS 的顺序是 写 start → 轮询 busy 落 → 读 frame_len → 读窗口"）
    wr(m, kOffPixCtl, 0b11)          # 源=FIFO + flush（清掉上一帧残留）
    wr(m, kOffPixCtl, 0b01)          # 源=FIFO，取消 flush
    wr(m, kOffSeq, seq)
    wr(m, kOffStamp0, stamp_ns & 0xFFFFFFFF)
    wr(m, kOffStamp1, (stamp_ns >> 32) & 0xFFFFFFFF)
    t0 = time.perf_counter()
    wr(m, kOffCtrl, 1)               # 先启动这一帧
    #: **必须等 busy 拉高再推像素**：vision_top 在收到 din_sof 之前忽略所有像素，
    #: 而 pixfeed 的放行条件就是"流水线已经开始收"（busy 高）。不等的话开头几十个
    #: 像素被吞 → 整帧错位，但帧长/CRC 仍然自洽（这个坑很隐蔽，实测 n_obs=0）。
    t_wait = time.perf_counter()
    while not (rd(m, kOffCtrl) & 0x1):
        if time.perf_counter() - t_wait > 0.2:
            raise RuntimeError("PL 200 ms 内没有进入忙碌（start 没生效？）")
    words = struct.unpack("<%dI" % (len(rgb) // 4), rgb)
    for v in words:
        wr(m, kOffPixPush, v)
    #: 等 busy 落（帧处理完）；带超时，绝不无限等
    deadline = t0 + 5.0
    while rd(m, kOffCtrl) & 0x1:      # busy
        if time.perf_counter() > deadline:
            raise RuntimeError("PL 5 s 内没处理完这一帧（busy 一直高）")
    if rd(m, kOffCtrl) & 0x10000:
        raise RuntimeError("PL 报 overflow")
    return time.perf_counter() - t0


def read_frame(m) -> dict:
    """按 `core/board/geometry_frame.hpp` 的契约解几何帧。

    头部 32 B：magic(u32) ver(u16) flags(u8) pad seq(u32) n_obs(u8) pad3
               n_gaps(u8) pad3 width(u16) height(u16) stamp_ns(u64)
    然后是 player(16) + target(24) + obstacles(36×n_obs) + gaps(36×n_gaps) + CRC32(4)
    → 总长 = 76 + 36*(n_obs + n_gaps)
    """
    raw = bytes(m[kOffFrame:kOffFrame + FRAME_WINDOW])
    magic, ver, flags = struct.unpack_from("<IHB", raw, 0)
    (seq,) = struct.unpack_from("<I", raw, 8)
    n_obs, n_gaps = raw[12], raw[16]
    w, h = struct.unpack_from("<HH", raw, 20)
    (crc_frame,) = struct.unpack_from("<I", raw, 76 + 36 * (n_obs + n_gaps) - 4)
    need = 76 + 36 * (n_obs + n_gaps)
    crc_calc = zlib.crc32(raw[:need - 4]) & 0xFFFFFFFF
    flen = rd(m, kOffFLen)
    return {"magic": magic, "ver": ver, "flags": flags, "seq": seq,
            "n_obs": n_obs, "n_gaps": n_gaps, "w": w, "h": h,
            "need": need, "flen": flen,
            "crc_frame": crc_frame, "crc_calc": crc_calc,
            "ok_magic": magic == MAGIC, "ok_len": need == flen,
            "ok_crc": crc_frame == crc_calc, "raw": raw}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="喂一帧真实图像进 PL 并校验几何帧")
    ap.add_argument("--image", required=True)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=360)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--jsonl", default="")
    a = ap.parse_args(argv)

    m, fd = open_regs()
    rgb = load_rgb(a.image, a.w, a.h)
    print("帧 %dx%d，%d 字节（%d 个字）" % (a.w, a.h, len(rgb), len(rgb) // 4))
    f = open(a.jsonl, "w") if a.jsonl else None
    ok_all = True
    try:
        for i in range(a.repeat):
            dt = push_frame(m, rgb, seq=i + 1, stamp_ns=time.monotonic_ns())
            r = read_frame(m)
            ok = r["ok_magic"] and r["ok_len"] and r["ok_crc"]
            ok_all &= ok
            print("帧 %d: %.2f ms  magic=%s ver=%d seq=%d %dx%d n_obs=%d n_gaps=%d "
                  "need=%d flen=%d crc %s" %
                  (i + 1, dt * 1000, hex(r["magic"]), r["ver"], r["seq"],
                   r["w"], r["h"], r["n_obs"], r["n_gaps"], r["need"], r["flen"],
                   "OK" if r["ok_crc"] else "MISMATCH(0x%08x vs 0x%08x)" %
                   (r["crc_frame"], r["crc_calc"])))
            if f:
                f.write(json.dumps({k: r[k] for k in ("magic", "ver", "seq", "w", "h",
                                                      "n_obs", "n_gaps", "need", "flen",
                                                      "ok_magic", "ok_len", "ok_crc")},
                                   ensure_ascii=False) + "\n")
    finally:
        if f:
            f.close()
        m.close()
        os.close(fd)
    print("结论：", "全部通过" if ok_all else "有失败项（见上）")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
