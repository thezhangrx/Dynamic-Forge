#!/usr/bin/env bash
# 用 SWD 读外设寄存器，验证 D153C 引脚重映射 / 定时器配置（不需要串口）
#
#   ./check_hw.sh
#
# 读：TIM3（PWM）、TIM2/TIM4（编码器计数）、GPIOB（引脚模式）、RCC（外设时钟）
# 用法：先跑一次，记住 TIM2/TIM4 的 CNT；用手转一下轮子再跑一次，CNT 应该变化。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ELF="$HERE/build/car.elf"
TC="${TC:-/home/zhang/FPGA/Env/toolchain/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi}"
TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT

[ -f "$ELF" ] || { echo "没有 $ELF，先 make" >&2; exit 1; }

# 一次性把需要的寄存器读成一整块：0x40000000 ~ 0x40000C00 覆盖 TIM2/TIM3/TIM4
BASE=0x40000000
SIZE=0x0C00
echo "== 读外设寄存器 0x$BASE + 0x$SIZE =="
st-flash read "$TMP" "0x$BASE" "$SIZE" 2>&1 | tail -1

# 另外单独读 GPIOB 和 RCC
GPIOB=$(mktemp); RCC=$(mktemp); RAM=$(mktemp)
trap 'rm -f "$TMP" "$GPIOB" "$RCC" "$RAM"' EXIT
st-flash read "$GPIOB" 0x40010C00 0x10 2>&1 | tail -1
st-flash read "$RCC"   0x40021000 0x20 2>&1 | tail -1

# 程序内部状态（g_ms / g_enable / g_target / g_pose）
read -r A_MS _ <<<"$("${TC}-nm" -S "$ELF" | awk '$4=="g_ms"{print $1, $2}')"
st-flash read "$RAM" "0x$A_MS" 4 2>&1 | tail -1

python3 - "$TMP" "$GPIOB" "$RCC" "$RAM" <<'PY'
import struct, sys

tim = open(sys.argv[1], 'rb').read()
gpiob = open(sys.argv[2], 'rb').read()
rcc = open(sys.argv[3], 'rb').read()
ram = open(sys.argv[4], 'rb').read()

def u32(buf, off): return struct.unpack('<I', buf[off:off+4])[0]
def timreg(n, off): return u32(tim, (n - 2) * 0x400 + off)   # TIM2/3/4 基址 0x40000000/400/800

base = 0x40000000
print()
print("=== TIM3（PWM：A=CH4/PB1，B=CH3/PB0）===")
cr1, arr, psc = timreg(3, 0x00), timreg(3, 0x2C), timreg(3, 0x28)
ccer, ccmr1, ccmr2 = timreg(3, 0x20), timreg(3, 0x18), timreg(3, 0x1C)
ccr3, ccr4 = timreg(3, 0x3C), timreg(3, 0x40)
print("  CR1   = 0x%04X  %s%s" % (cr1, "CEN(已启动) " if cr1 & 1 else "CEN=0(没启动!) ",
                                      "ARPE " if cr1 & 0x80 else ""))
print("  ARR=%d PSC=%d  -> 频率 %.1f kHz" % (arr, psc, 72000.0 / ((arr + 1) * (psc + 1))))
print("  CCER  = 0x%04X  %s%s" % (ccer,
    "CC3E " if ccer & (1 << 8) else "CC3E=0 ", "CC4E " if ccer & (1 << 12) else "CC4E=0 "))
print("  CCMR1 = 0x%04X (应为 0x0000，CH1/CH2 不用)  CCMR2 = 0x%04X (应为 0x6868)"
      % (ccmr1, ccmr2))
print("        CCMR2 拆开：OC3M=%d(期望6) OC3PE=%d  OC4M=%d(期望6) OC4PE=%d"
      % ((ccmr2 >> 4) & 7, (ccmr2 >> 3) & 1, (ccmr2 >> 12) & 7, (ccmr2 >> 11) & 1))
print("  CCR3=%d  CCR4=%d   %s" % (ccr3, ccr4,
      "(停机状态，符合预期)" if ccr3 == 0 and ccr4 == 0 else "(有输出)"))

print()
print("=== 编码器计数（手转轮子会变）===")
for n, name in ((2, "TIM2 = B 电机编码器 (PA0/PA1)"), (4, "TIM4 = A 电机编码器 (PB6/PB7)")):
    cr1 = timreg(n, 0x00); smcr = timreg(n, 0x04); cnt = timreg(n, 0x24); arr = timreg(n, 0x2C)
    print("  %-34s CR1=0x%04X %s  SMCR=0x%04X(SMS=%d, 期望 3=编码器模式) CNT=%d ARR=%d"
          % (name, cr1, "CEN" if cr1 & 1 else "CEN=0!", smcr, smcr & 7, cnt, arr))

print()
print("=== GPIOB 模式（应为 PB0/PB1=AF推挽, PB12..15=推挽输出）===")
crl, crh, odr, idr = u32(gpiob, 0), u32(gpiob, 4), u32(gpiob, 0xC), u32(gpiob, 8)
pb01 = (crl >> 0) & 0xFF
pb1215 = (crh >> 16) & 0xFFFF
print("  CRL=0x%08X  PA/PB0-1 模式=0x%02X （期望 0xBB = 两个都 AF推挽50MHz）" % (crl, pb01))
print("  CRH=0x%08X  PB12-15 模式=0x%04X （期望 0x3333 = 四个都推挽输出50MHz）" % (crh, pb1215))
print("  ODR=0x%08X  IDR=0x%08X" % (odr, idr))

print()
print("=== 外设时钟 RCC ===")
apb1, apb2 = u32(rcc, 0x1C), u32(rcc, 0x18)
print("  APB1ENR=0x%08X  TIM2=%d TIM3=%d TIM4=%d" % (apb1, (apb1>>0)&1, (apb1>>1)&1, (apb1>>2)&1))
print("  APB2ENR=0x%08X  IOPB=%d USART1=%d" % (apb2, (apb2>>3)&1, (apb2>>14)&1))

g_ms = struct.unpack('<I', ram[:4])[0]
print()
print("=== 程序状态 ===")
print("  g_ms = %d ms" % g_ms)
PY
