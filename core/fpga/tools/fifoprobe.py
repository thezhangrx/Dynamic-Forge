#!/usr/bin/env python3
"""fifoprobe.py —— 直接读像素 FIFO 的**内部状态与内容**（板上跑，要 /dev/mem）

为什么要有它：板上"推了一整帧、帧长/CRC 自洽，但 n_obs=0"，而灰度图取证只能看到
最终结果。要分清是"没写进去 / 读指针跑到前面 / 蓄水池丢了字节"，只能把内部量读出来。

用到的寄存器（2026-10-24 第二批诊断口，vision_axi.v / vision_pixfeed.v）：
    0x0DC PIXCTL  写 bit0=源(1=FIFO) bit1=flush
    0x0E0 PIXDBG0 已接收字节数（每次 push +4）
    0x0E4 PIXDBG1 异或累加
    0x0E8 PIXDBG2 和累加
    0x0EC PIXZSTRB 被忽略的 WSTRB=0 写次数
    0x0F0 PIXMEM  写：FIFO 字地址；读：该地址的字（与 FIFO 消费者同一条读路径）
    0x0F4 PIXPTR  {16'b0, FIFO 字数 level[11:0], 蓄水池字节数 nb[3:0]}
    0x0F8 PIXCOMMIT 打到 PIXPUSH 的写次数（含被字节合并掉的）

判据（推 N 个已知字之后）：
    * commits == N（每笔 store 一次提交）；
    * 0x0E0 == 4*N（只有 N 个字真的进了 FIFO）；
    * 0x0F4 的 level == N（没被消费时），nb == 0；
    * mem[i] == 第 i 个字 —— **这一条最关键**：它是消费者真正会读到的内容。
"""
import mmap
import os
import struct
import sys

BASE = 0x43C00000
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
m = mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=BASE)

rd = lambda off: struct.unpack_from("<I", m, off)[0]
wr = lambda off, v: struct.pack_into("<I", m, off, v & 0xFFFFFFFF)

PIXPUSH, PIXSTAT, PIXCTL = 0x0D4, 0x0D8, 0x0DC
DBG_BYTES, DBG_XOR, DBG_SUM, ZSTRB = 0x0E0, 0x0E4, 0x0E8, 0x0EC
PIXMEM, PIXPTR, PIXCOMMIT = 0x0F0, 0x0F4, 0x0F8

def snapshot(tag):
    ptr = rd(PIXPTR)
    print("%-14s commits=%-6d bytes=%-7d level=%-4d nb=%d zstrb=%d ovf=%d"
          % (tag, rd(PIXCOMMIT), rd(DBG_BYTES), ptr & 0xFFF, (ptr >> 12) & 0xF,
             rd(ZSTRB), (rd(PIXSTAT) >> 16) & 1))

def mem_read(addr):
    wr(PIXMEM, addr)
    return rd(PIXMEM)

def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    print("=== 复位（FIFO 源 + flush）")
    wr(PIXCTL, 0x3)
    wr(PIXCTL, 0x1)
    snapshot("flush 后")

    print("=== 推 %d 个已知字（不跑帧，没人消费）" % n)
    words = [(0x11000000 + i * 0x01010101) & 0xFFFFFFFF for i in range(n)]
    for w in words:
        wr(PIXPUSH, w)
    snapshot("推完后")

    print("=== 回读 FIFO 内容（消费者看到的那个字）")
    mem = [mem_read(i) for i in range(n)]
    bad = 0
    for i, w in enumerate(words):
        ok = mem[i] == w
        if not ok:
            bad += 1
        print("  mem[%2d] = 0x%08x  期望 0x%08x  %s" % (i, mem[i], w, "✓" if ok else "✗"))

    print("=== 再 flush，恢复原状")
    wr(PIXPUSH, 0xDEADBEEF)      # 让最后一笔也可查
    snapshot("flush 前")
    wr(PIXCTL, 0x3)
    wr(PIXCTL, 0x1)
    wr(PIXCTL, 0x0)
    snapshot("flush 后")

    print("\n结论：", "FIFO 写入与读出口一致" if bad == 0 else "%d/%d 个字不一致 —— 问题在 FIFO 写/读通路" % (bad, n))
    return 0 if bad == 0 else 1

if __name__ == "__main__":
    raise SystemExit(main())
