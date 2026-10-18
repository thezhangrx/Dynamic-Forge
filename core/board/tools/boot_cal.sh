#!/bin/bash
# ===========================================================================
# boot_cal.sh —— 开场标定（**板上实测**）：量平台速度、指令延迟、初始位置
#
#     bash core/board/tools/boot_cal.sh [标定秒数，默认 45]
#
# 为什么需要这一轮：虚拟真值把 `@MOVE` 积分成平台位置，只依赖三个输入 ——
# 初始位置、平台速度、指令→动作延迟。以前速度是 158.7（从离线语料反推）、
# 延迟只知道总数 0.457 s（串口 + 键盘桥 + 采集混在一起）。这两个数错一点，
# 积分就系统性偏，而"用视觉修积分"看起来永远像视觉有问题 —— 所以先标定。
#
# 两边配合（缺一不可）：
#   · 仿真 `--obstacle-delay N`：开场 N 秒**场上没有任何障碍**
#     （没有墙压着角点、没有墙挡住角色，几何和角色都是干净的）；
#   · 板子 `--cal N`：这段窗口里**决策被旁路**，改发已知动作序列
#     停 1 s → 左/右/下/上/左上/右上/左下/右下 各 2 s（段间停 0.4 s），20.2 s 一轮。
#
# 产出的 `/tmp/cal/calib.json` 直接喂给 `virt_truth.py --calib`。
#
# ⚠ 屏幕必须让仿真窗口在最前面（相机看的是物理屏幕）。
# ⚠ 停程序一律 `-INT`，**绝不 `-9`**：被 -9 的流式 ffmpeg 会把 V4L2 卡在 busy。
# ===========================================================================
set -u
cd "$(dirname "$0")/../../.." || exit 1
CAL=${1:-45}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
OUT=${OUT:-/tmp/cal}
PY=Env/bin/python
#: 仿真侧的"无墙窗口"= 标定窗口 + 板子启动那一段（实测 8~15 s，给 25 s 余量）。
#: 不够大就会出现"板子还在标定、墙上来了"，几何被污染，这一轮只能作废。
DELAY=$((CAL + 25))
R() { ssh -n -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

mkdir -p "$OUT"
# 先把两块板子的时钟对齐（板子没有 NTP，实测慢 1.9 s；不对齐会把 1.9 s 的假延迟
# 算进"角色/墙的误差"里，166 单位/秒下就是 300 单位的假误差）
bash core/board/tools/sync_clock.sh 2>&1 | sed 's/^/[标定] /'

# ---- 量板子的时间基速率（**这块板子快 9%**）--------------------------------
# 实测：在板子上 `sleep 20`，主机单调钟只走了 18.35 s —— 也就是板子的
# `CLOCK_MONOTONIC`（以及 steady_clock、`sleep`、所有帧时间戳）比真实时间**快 9%**。
# clocksource 只有一个 `arch_sys_counter`（ARM 架构定时器），这是**硬件/固件层面**
# 的频率标定错误，用户态改不了。所以：
#   · 板子内部的时长（cal/t_ns/frame_age）都要乘 1/rate 换算成真实秒；
#   · 反过来，虚拟真值在板子上积分，用的就是"**每板秒**多少单位"（= 实测 155），
#     **不能**用契约的 200 —— 那 9% 会被自动吸收进标定值里。
RATE=$("$PY" - <<'PYR'
import subprocess, time
KEY = 'data/fpga/board_ssh/board_key'
cmd = ['ssh', '-i', KEY, '-o', 'StrictHostKeyChecking=no',
       '-o', 'ControlPath=/tmp/ssh_mux_board', '-n', 'root@192.168.137.242', 'sleep 20']
m0 = time.monotonic()
subprocess.run(cmd, capture_output=True)
m1 = time.monotonic()
print(f'{20.0 / (m1 - m0):.4f}')
PYR
)
echo "[标定] 板子时间基速率 = $RATE（1.09 = 板子的秒比真实秒快 9%）"
rm -f "$OUT"/diag.jsonl "$OUT"/truth.jsonl "$OUT"/collision.jsonl "$OUT"/calib.json
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
sleep 1
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x vision_board_devmem 2>/dev/null; \
   pkill -INT -f "[c]am_live.sh" 2>/dev/null; sleep 1; echo cleaned' >/dev/null 2>&1

: > /tmp/cal_stream.txt
date +%s.%N > "$OUT/sim_start_epoch.txt"
echo "[标定] 起仿真：无墙窗口 ${DELAY}s，标定 ${CAL}s，真值+碰撞日志 → $OUT"
(setsid sh -c "tail -f /tmp/cal_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((DELAY + CAL + 60)) --fullscreen --no-hud \
   --obstacle-delay $DELAY \
   --state-log '$OUT/truth.jsonl' --collision-log '$OUT/collision.jsonl'" \
   > "$OUT/sim.log" 2>&1 &)
sleep 6
echo '@MOVE 0' >> /tmp/cal_stream.txt
# 窗口必须在前台：相机拍的是屏幕，被别的窗口盖住时连四标记都找不到
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[标定] /'
sleep 2

# ---- 就绪门：等相机真的看到**场地方框**（标定窗口里没有墙，只能用四标记判）----
# `wall_gap_probe` 在无墙窗口里什么也看不到，拿它当判据会让这一轮永远"没就绪"。
# 用 `rectify_probe`（P1 的离线体检器）：每个帧报 `标记=4/4`、单应是否成立。
PROBE=/tmp/rect_probe
if [ ! -x "$PROBE" ]; then
    g++ -std=c++17 -O2 -I core/board -o "$PROBE" \
        core/board/tools/rectify_probe.cpp core/board/ppm_reader.cpp 2>/dev/null
fi
echo "[标定] 等场地方框就绪（最多 40 s）…"
READY=0
for attempt in $(seq 1 20); do
    R 'rm -f /tmp/rdy_*.ppm; ffmpeg -hide_banner -loglevel error -f v4l2 \
        -input_format mjpeg -video_size 640x480 -framerate 60 -i /dev/video0 \
        -vf scale=640:360 -pix_fmt rgb24 -frames:v 3 -y /tmp/rdy_%02d.ppm 2>/dev/null' >/dev/null 2>&1
    rm -rf "$OUT/ready"; mkdir -p "$OUT/ready"
    for i in 01 02 03; do
        scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            "$HOST:/tmp/rdy_$i.ppm" "$OUT/ready/" 2>/dev/null
    done
    if [ -x "$PROBE" ]; then
        # 汇总行形如：`四标记齐全：find_markers 3/3 = 100.0%；find_markers4 3/3 = 100.0%`
        N=$("$PROBE" --ppm-dir "$OUT/ready" 2>/dev/null \
            | grep -o 'find_markers4 [0-9]*/[0-9]*' | awk '{print $2}' | cut -d/ -f1)
    else
        N=0
    fi
    N=${N:-0}
    echo "  第 $attempt 次：3 帧里 $N 帧四标记齐全"
    if [ "${N:-0}" -ge 2 ]; then READY=1; break; fi
    sleep 2
done
if [ "$READY" != "1" ]; then
    echo "[标定] 场地一直没就绪（相机没对着全屏仿真窗口？）—— 这一轮不记录，退出"
    bash core/board/tools/kill_sim.sh >/dev/null 2>&1
    exit 1
fi
echo "[标定] 场地就绪，开始标定（板子旁路决策 ${CAL}s）"

date +%s.%N > "$OUT/board_start_epoch.txt"
R "cd /root/tools && rm -f /tmp/diag_cal.jsonl; ./cam_live.sh --frames 0 --via-pl \
   --keyboard-dev stdout --structural-gap --cal $CAL --diag-log /tmp/diag_cal.jsonl" \
   2>/dev/null | tee -a /tmp/cal_stream.txt > /dev/null &
BPID=$!
sleep $((CAL + 6))
kill "$BPID" 2>/dev/null
R 'pkill -INT -x vision_board_devmem 2>/dev/null; pkill -INT -x ffmpeg 2>/dev/null; \
   pkill -INT -f "[c]am_live.sh" 2>/dev/null; sleep 1' >/dev/null 2>&1
bash core/board/tools/kill_sim.sh >/dev/null 2>&1

scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "$HOST:/tmp/diag_cal.jsonl" "$OUT/" 2>/dev/null

echo
# ⚠ scp 到目录时保留原文件名（`diag_cal.jsonl`），别写成 `diag.jsonl`
"$PY" core/board/tools/boot_cal.py --diag "$OUT/diag_cal.jsonl" \
    --truth "$OUT/truth.jsonl" --board-rate "${RATE:-1.0}" --out "$OUT/calib.json"
