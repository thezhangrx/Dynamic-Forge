#!/usr/bin/env python3
"""bit2bin.py —— 把 Vivado 的 .bit 转成板端 fpga_manager 能吃的 .bin（等价 bootgen）

为什么需要它
    Windows 上那条路是 `bootgen -image bit2bin.bif -arch zynq -process_bitstream bin`，
    要开 Vivado 2018.3、要 .bif、要 cmd。板子上的自启动服务（pl-autoload.service）
    只认 `/lib/firmware/*.bin`，所以每次重编位流都得转一次。
    这个脚本纯 Python，不用 Vivado。

算法（2026-10-29 用 bd_vision_r8.bit 反推并**逐字节验证**过）
    1. 解析 .bit 头：magic 13 B 之后是 a/b/c/d/e 五个字段，e = 4 字节大端"载荷长度"
    2. 取载荷 payload = bit[off : off+len]
    3. 以 4 字节为单位把每个字**字节序反转**（Zynq devcfg 要的就是这个顺序）
    4. 结尾补 5 个 `00 00 00 20`（bootgen 也补，缺了 DMA 长度对不上）

验证：`python3 bit2bin.py --selftest` 会用仓库里那份 r8 的 .bit/.bin 对拍，
      两者 md5 必须完全一致（3258d6f4…），否则说明算法被改坏了。

用法
    python3 core/board/tools/bit2bin.py bd_vision_p0.bit                 # 生成同名 .bin
    python3 core/board/tools/bit2bin.py a.bit -o /tmp/a.bin
    python3 core/board/tools/bit2bin.py --selftest
    # 生成后拷到板子（自启动服务每次启动都会加载这个文件）：
    #   scp -i data/fpga/board_ssh/board_key a.bin root@192.168.137.242:/lib/firmware/bd_vision.bin
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys

TAIL = b"\x00\x00\x00\x20" * 5
R8_BIT = "data/fpga/pl_bitstream/bd_vision_r8.bit"
R8_BIN = "data/fpga/pl_bitstream/bd_vision_r8.bit.bin"
R8_BIN_MD5 = "3258d6f441cb7a031f0c3b0dc963c0ab"


def parse_bit(bit: bytes):
    """返回 (payload, header_len)。header_len 是从文件头到载荷起点。"""
    if bit[:4] != b"\x00\x09\x0f\xf0":
        raise SystemExit("不是 .bit 文件（magic 不是 00 09 0f f0）")
    i = 13
    while i < 4096:
        key = bit[i:i + 1]
        if key == b"e":
            n = int.from_bytes(bit[i + 1:i + 5], "big")
            off = i + 5
            if off + n > len(bit):
                raise SystemExit("e 字段长度 %d 超出文件（offset %d, size %d）"
                                 % (n, off, len(bit)))
            return bit[off:off + n], off
        if key in (b"a", b"b", b"c", b"d"):
            n = int.from_bytes(bit[i + 1:i + 3], "big")
            i += 3 + n
        else:
            raise SystemExit("解析 .bit 头失败：偏移 %d 处不是 a/b/c/d/e（%s）"
                             % (i, key.hex()))
    raise SystemExit("解析 .bit 头失败：没找到 e 字段")


def bit2bin(bit: bytes) -> bytes:
    payload, off = parse_bit(bit)
    n = len(payload) // 4 * 4
    if n != len(payload):
        raise SystemExit("载荷长度 %d 不是 4 的倍数，不敢转" % len(payload))
    sw = b"".join(payload[j:j + 4][::-1] for j in range(0, n, 4))
    return sw + TAIL


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bit", nargs="?", help="输入 .bit")
    ap.add_argument("-o", "--out", help="输出 .bin（默认同名 .bin）")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        for p in (R8_BIT, R8_BIN):
            if not os.path.exists(p):
                raise SystemExit("自检需要仓库里这两个文件：%s" % p)
        got = bit2bin(open(R8_BIT, "rb").read())
        ref = open(R8_BIN, "rb").read()
        md5 = hashlib.md5(got).hexdigest()
        print("由 %s 生成 %d B md5 %s" % (R8_BIT, len(got), md5))
        print("仓库现有 %s %d B md5 %s" % (R8_BIN, len(ref), hashlib.md5(ref).hexdigest()))
        ok = got == ref and md5 == R8_BIN_MD5
        print("PASS（算法与 bootgen 逐字节一致）" if ok else "FAIL（算法不对，别用）")
        return 0 if ok else 1

    if not a.bit:
        ap.error("要么给一个 .bit，要么 --selftest")
    bit = open(a.bit, "rb").read()
    out = bit2bin(bit)
    dst = a.out or (os.path.splitext(a.bit)[0] + ".bin")
    with open(dst, "wb") as f:
        f.write(out)
    payload, off = parse_bit(bit)
    print("%s (%d B) → %s (%d B, md5 %s)"
          % (a.bit, len(bit), dst, len(out), hashlib.md5(out).hexdigest()))
    print("  .bit 头 %d B，载荷 %d B，已按 4 字节换序 + 补 %d B 尾" % (off, len(payload), len(TAIL)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
