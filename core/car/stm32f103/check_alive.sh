#!/usr/bin/env bash
# 用 SWD 读 RAM 判断固件有没有在跑（不需要串口）
#
#   ./check_alive.sh
#
# 原理：main.c 里 g_ms 由 SysTick 每 1 ms 加 1。读两次，间隔 1 秒：
#   - 增 ≈1000  -> 主循环在跑
#   - 不变      -> 没跑（没复位 / 时钟没起来 / 程序卡死）
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ELF="$HERE/build/car.elf"
TC="${TC:-/home/zhang/FPGA/Env/toolchain/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi}"

[ -f "$ELF" ] || { echo "没有 $ELF，先 make" >&2; exit 1; }

# 从符号表取地址和大小
read -r ADDR SIZE <<<"$("${TC}-nm" -S "$ELF" | awk '$4=="g_ms"{print $1, $2}')"
[ -n "${ADDR:-}" ] || { echo "找不到 g_ms 符号" >&2; exit 1; }
echo "g_ms 位于 0x$ADDR（$((16#$SIZE)) 字节）"

TMPA=$(mktemp); TMPB=$(mktemp)
trap 'rm -f "$TMPA" "$TMPB"' EXIT

echo "== 第一次读 =="
st-flash read "$TMPA" "0x$ADDR" "$((16#$SIZE))" 2>&1 | tail -2
sleep 1
echo "== 第二次读（1 秒后）=="
st-flash read "$TMPB" "0x$ADDR" "$((16#$SIZE))" 2>&1 | tail -2

python3 - "$TMPA" "$TMPB" <<'PY'
import struct, sys
a = open(sys.argv[1], 'rb').read()
b = open(sys.argv[2], 'rb').read()
if len(a) < 4 or len(b) < 4:
    sys.exit("回读数据不足：%d / %d 字节" % (len(a), len(b)))
ma = struct.unpack('<I', a[:4])[0]
mb = struct.unpack('<I', b[:4])[0]
d = (mb - ma) & 0xFFFFFFFF
print("g_ms: %d -> %d   (增加 %d)" % (ma, mb, d))
if ma == 0 and mb == 0:
    print("结果：g_ms 一直是 0 —— 固件没跑起来（检查是否真的复位了）")
elif 100 <= d <= 5000:
    print("结果：OK，主循环在跑，SysTick 计数正常（约 %d 次/秒）" % d)
elif d == 0:
    print("结果：g_ms 卡住 —— 程序可能卡在某处（或芯片没复位）")
else:
    print("结果：增量异常（%d），但至少说明程序在东" % d)
PY
