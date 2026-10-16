#!/usr/bin/env bash
# ===========================================================================
# soak_loop.sh —— 长跑：板子上关门跑 N 分钟，同时**只读**采样寄存器与进程资源
#
#     bash soak_loop.sh [分钟数,默认30] [帧数上限,默认0=不限]
#
# 盯四件事（都是"跑一会儿才看得出来"的）：
#   1. 像素 FIFO 溢出粘滞位（PIXSTAT bit16）—— 短跑不出，长跑可能出；
#   2. 帧计数有没有停住（最长连续不涨秒数）—— 相机抽风/流水线卡死；
#   3. 进程内存 VmRSS 与句柄数漂移 —— 泄漏；
#   4. 决策/动作行数是否与帧数对得上、有没有失败帧。
#
# ⚠ 停程序一律 `pkill -INT -x vision_board_devmem`（按**进程名**精确匹配，不用 -f；
#   ffmpeg 也只用 -INT，**绝不 -9** —— 被 -9 的流式 ffmpeg 会把 V4L2 卡在 busy）。
# ===========================================================================
set -u
MIN=${1:-30}
FRAMES=${2:-0}
OUTDIR=/root/soak
mkdir -p "$OUTDIR"
STAMP=$(date +%Y%m%d_%H%M%S)
APP_LOG="$OUTDIR/app_$STAMP.log"
SMP_LOG="$OUTDIR/sampler_$STAMP.log"

echo "[soak] 开始：$MIN 分钟，帧数上限 $FRAMES；日志 $APP_LOG / $SMP_LOG"
echo "[soak] PL 状态：$(cat /sys/class/fpga_manager/fpga0/state 2>/dev/null)"

cd /root/tools || exit 1
# 先把可能残留的进程清掉（不然相机被占着，这一轮会"跑不起来"）
pkill -INT -x ffmpeg 2>/dev/null
pkill -INT -x vision_board_devmem 2>/dev/null
sleep 2

./cam_live.sh --frames "$FRAMES" --via-pl --keyboard-dev stdout >"$APP_LOG" 2>&1 &
# ⚠ 不能固定 sleep 几秒就 pidof：相机要热身（打开设备 + 丢前 10 帧自动曝光），
#   程序可能要好几秒才出现 —— 第一次长跑就是这么"程序 PID=未起来"跑完 25 分钟、
#   采样器一次没启动的。改成**轮询等它出现**。
PID=""; waited=0
for _ in $(seq 1 60); do
    PID=$(pidof vision_board_devmem | awk '{print $1}')
    [ -n "$PID" ] && break
    sleep 2; waited=$((waited + 2))
done
if [ -z "$PID" ]; then
    echo "[soak] ✗ 60 秒内没等到 vision_board_devmem —— 程序没起来，先看日志尾部："
    tail -12 "$APP_LOG"
    pkill -INT -x ffmpeg 2>/dev/null
    exit 1
fi
echo "[soak] 程序 PID=$PID（等了 ${waited} 秒才出现；相机热身要几秒是正常的）"

if [ -n "${PID:-}" ] && [ -x /root/tools/soak_sampler ]; then
    /root/tools/soak_sampler --seconds $((MIN * 60)) --pid "$PID" >"$SMP_LOG" 2>&1 &
    SMP=$!
else
    echo "[soak] 没有 soak_sampler（或程序没起来），只跑程序不采样"; SMP=""
fi

sleep $((MIN * 60))

echo "[soak] 时间到，停程序（SIGINT，不是 -9）"
pkill -INT -x vision_board_devmem 2>/dev/null
pkill -INT -x ffmpeg 2>/dev/null
sleep 2
[ -n "${SMP:-}" ] && { kill "$SMP" 2>/dev/null; sleep 1; }

echo "[soak] ---- 程序侧汇总 ----"
grep -E "成功|失败|帧/秒|决策 |键盘桥|Error|error|溢出|Broken" "$APP_LOG" | tail -8
echo "[soak] 动作行数：$(grep -c '^@MOVE' "$APP_LOG")"
echo "[soak] ---- 采样汇总 ----"
tail -12 "$SMP_LOG" 2>/dev/null || echo "  (没有采样日志)"
echo "[soak] 完成，日志在 $OUTDIR/"
