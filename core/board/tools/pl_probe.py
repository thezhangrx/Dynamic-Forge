#!/usr/bin/env python3
"""pl_probe.py —— 板端 PL（vision_top）**接口层探针**：只读/只写寄存器，不改 RTL。

为什么要有这个文件
------------------
"PL 能接手什么"里最便宜的一类是**控制面**：参数表、几何帧契约、帧状态/时序、
统计汇总。这些东西不值得每验一次就现写脚本 —— 这里固化成一个探针，随时能回答
"PL 还活着吗、帧对不对、统计能不能读出来"。

它跑在**板子上**（要 /dev/mem），用法：

    ssh root@192.168.137.242 'python3 /tmp/pl_probe.py'            # 全量自检
    ssh root@192.168.137.242 'python3 /tmp/pl_probe.py --cadence 2'  # 只测节拍

判据（与 `core/board/vision_regs.hpp` / `core/fpga/rtl/vision_pkg.vh` 对齐）
------------------------------------------------------------------------
* 参数表：21 个配置寄存器读回值必须与驱动写下去的一致（驱动侧报告"不符 0"）；
* 帧契约：magic='BHV1'(0x31564842)、version=1、长度 = 76+36*n_obs+36*n_gaps
  必须等于 FRAME_LEN，且帧尾 CRC32 必须等于对帧体重算的 zlib.crc32；
* 状态/时序：CTRL 的 busy/frame_done/frame_count 位；帧计数必须连续（无丢帧）；
* 统计：调试口 sel=7 的 n_bands/x_min/x_max/n_obs/n_gaps/n_labels/n_segs，
  以及 sel=3 行直方图、sel=4 列投影、sel=5/6 行带上下沿。

实测基线（2026-10-21，真实相机帧 640×360）
------------------------------------------
    参数表     21 项读回一致
    帧契约     need=472 == FRAME_LEN=472，CRC32 双方均为 0x781475B6
    节拍       mean 29.96 ms / min 29.93 / max 30.06 / sd 0.03 ms，无丢帧
    控制面     单次寄存器访问 3.6 µs；读 2048 B 帧窗口 1.74 ms
    统计       n_obs=8 / n_gaps=3 与帧头一致；行直方图/列投影/行带均可读
"""
from __future__ import annotations

import argparse
import mmap
import os
import struct
import time
import zlib

BASE = 0x43C00000
kOffCtrl, kOffSeq, kOffStamp0, kOffStamp1 = 0x0C0, 0x0C4, 0x0C8, 0x0CC
kOffFLen, kOffPixStat = 0x0D0, 0x0D8
kOffDbgSel, kOffDbgData = 0x080, 0x084
kOffFrame = 0x100
kFrameWindowBytes = 2048
kCfgCount = 21


def open_regs():
    f = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)   # 只读打不开，必须 O_RDWR
    return mmap.mmap(f, 0x2000, offset=BASE)


def rd(m, off: int) -> int:
    return struct.unpack_from("<I", m, off)[0]


def wr(m, off: int, val: int) -> None:
    struct.pack_into("<I", m, off, val & 0xFFFFFFFF)


def dbg(m, sel: int, idx: int = 0, sub: int = 0) -> int:
    wr(m, kOffDbgSel, ((sub & 0xF) << 22) | ((idx & 0x3FFFF) << 4) | (sel & 0xF))
    return rd(m, kOffDbgData)


def show_stats(m) -> dict:
    names = {0: "n_bands", 1: "x_min", 2: "x_max", 3: "n_obs",
             4: "n_gaps", 5: "n_labels", 6: "n_segs"}
    out = {n: dbg(m, 7, 0, s) for s, n in names.items()}
    for k, v in out.items():
        print(f"  {k:<9s} = {v}")
    print("  row_count idx=100/180/260/340 :", [dbg(m, 3, i) for i in (100, 180, 260, 340)])
    print("  span_count idx=100/320/500    :", [dbg(m, 4, i) for i in (100, 320, 500)])
    print("  band y0/y1 idx=0,1            :",
          [dbg(m, 5, 0), dbg(m, 6, 0), dbg(m, 5, 1), dbg(m, 6, 1)])
    return out


def show_frame(m) -> bool:
    flen = rd(m, kOffFLen)
    buf = b"".join(struct.pack("<I", rd(m, kOffFrame + 4 * j))
                   for j in range(kFrameWindowBytes // 4))
    magic, ver, flags, seq, n_obs, n_gaps, iw, ih, stamp = struct.unpack_from(
        "<IHHIIIHHQ", buf, 0)
    need = 76 + 36 * n_obs + 36 * n_gaps
    ok = (magic == 0x31564842 and need == flen and need <= len(buf))
    crc_calc = zlib.crc32(buf[:need - 4]) & 0xFFFFFFFF if ok else 0
    crc_frame = struct.unpack_from("<I", buf, need - 4)[0] if ok else 0
    ok = ok and crc_calc == crc_frame
    print(f"  magic=0x{magic:08X} ver={ver} flags={flags} seq={seq} "
          f"n_obs={n_obs} n_gaps={n_gaps} img={iw}x{ih} stamp={stamp}")
    print(f"  need={need} FLEN={flen} crc_calc=0x{crc_calc:08X} "
          f"crc_frame=0x{crc_frame:08X} -> {'OK' if ok else 'MISMATCH'}")
    return ok


def show_ctrl(m) -> int:
    c = rd(m, kOffCtrl)
    print(f"  CTRL=0x{c:08X} busy={c & 1} done={(c >> 1) & 1} "
          f"fcount={(c >> 8) & 0xFFFF} overflow={(c >> 24) & 1}")
    ps = rd(m, kOffPixStat)
    print(f"  PIX_STAT=0x{ps:08X} buffered={ps & 0xFFFF} "
          f"overflow={(ps >> 16) & 1} filling={(ps >> 17) & 1}")
    return (c >> 8) & 0xFFFF


def cadence(m, seconds: float) -> None:
    last = (rd(m, kOffCtrl) >> 8) & 0xFFFF
    events: list[tuple[float, int]] = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        n = (rd(m, kOffCtrl) >> 8) & 0xFFFF
        if n != last:
            events.append((time.monotonic(), n - last))
            last = n
    if len(events) < 2:
        print(f"  {seconds:g}s 内只看到 {len(events)} 帧完成（PL 没在跑？）")
        return
    d = [(events[i + 1][0] - events[i][0]) * 1e3 for i in range(len(events) - 1)]
    mean = sum(d) / len(d)
    sd = (sum((x - mean) ** 2 for x in d) / len(d)) ** 0.5
    print(f"  {seconds:g}s 内 {len(events)} 帧，相邻帧间隔 ms: "
          f"mean={mean:.2f} min={min(d):.2f} max={max(d):.2f} sd={sd:.2f}")
    print(f"  帧计数增量合计 = {sum(e[1] for e in events)}（应等于帧数，> 说明中间丢帧）")
    print(f"  最终 fcount = {last}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cadence", type=float, default=0.0,
                    help="只测帧节拍，跑这么多秒（需要板端有进程在推帧）")
    a = ap.parse_args()
    m = open_regs()
    print("== 参数表（21 个配置寄存器）==")
    print(" ", [rd(m, 4 * i) for i in range(kCfgCount)])
    if a.cadence > 0:
        print("== 帧节拍 ==")
        cadence(m, a.cadence)
        return 0
    print("== 帧状态/时序 ==")
    show_ctrl(m)
    print(f"  seq={rd(m, kOffSeq)} stamp_ns="
          f"{rd(m, kOffStamp0) | (rd(m, kOffStamp1) << 32)}")
    print("== 帧契约（几何帧窗口）==")
    ok_frame = show_frame(m)
    print("== 统计汇总（调试口）==")
    st = show_stats(m)
    print("== 自检结论 ==")
    ok_cfg = all(v != 0 for v in (rd(m, 4 * i) for i in range(kCfgCount)))
    print(f"  帧契约 {'OK' if ok_frame else 'FAIL'}；"
          f"配置寄存器{'有非零值（已写过的样子）' if ok_cfg else '全 0（没写过？）'}；"
          f"n_obs={st['n_obs']} n_gaps={st['n_gaps']}")
    t0, N = time.monotonic(), 20000
    for _ in range(N):
        rd(m, kOffFLen)
    print(f"  控制面单次访问 {(time.monotonic() - t0) / N * 1e6:.3f} µs")
    return 0 if ok_frame else 2


if __name__ == "__main__":
    raise SystemExit(main())
