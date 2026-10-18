#!/bin/bash
# 把**板子的系统时钟**对上主机（用 NTP 式的四次取样，取最小 RTT 的那次）。
#
# 为什么必须做：板子没有 NTP，实测它的系统时钟比主机**慢 1.9 s 且会漂**。
# 而诊断日志里的 `host_t` 是用 `date +%s.%N`（板子时钟）打出来的 —— 一旦拿它
# 去和仿真的 `unix`（主机时钟）配对，就相当于给每一帧加了 1.9 s 的假延迟：
# 平台 166 单位/秒 → **300 场地单位**的假误差。以前"角色误差中位 52 单位、
# 墙 y 偏置 −33"这些结论就是这么来的，不是视觉的问题。
#
# 做法（不需要 root 之外的权限，也不需要网络）：
#   1. 主机记 t0 → ssh 到板子读 `date +%s.%N` → 主机记 t1；
#      板子的读数约等于 (t0+t1)/2 那一刻（RTT 对称假设）；
#   2. 取 RTT 最小的一次作为偏移估计（排队最少 = 最准）；
#   3. 在板子上 `date -s @<主机当前时刻 + 偏移>`，再复测一次并报残差。
#
# 用法：
#   bash core/board/tools/sync_clock.sh        # 同步并复测
#   bash core/board/tools/sync_clock.sh --check  # 只看偏差，不改
set -u
cd "$(dirname "$0")/../../.." || exit 1
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1
# ⚠ 必须走**连接复用**（ControlMaster）：不复用时每次 ssh 要 0.3~1.5 s 建会话，
#   而"板子读到的时刻"只能靠 RTT 中点估计 —— 实测单次测量误差有 **0.8 s**，
#   迭代补偿会来回震荡（+0.83 → −0.81 → +0.85）。复用之后 RTT 降到 22~28 ms，
#   中点估计的误差就只有十几毫秒（实测三次读数差 3 ms）。
CP=${CP:-/tmp/ssh_mux_board}
_master_up() {
    ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ControlMaster=auto -o ControlPath="$CP" -o ControlPersist=300 \
        -n -o ConnectTimeout=8 "$HOST" 'true' 2>/dev/null
}
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ControlPath="$CP" -n -o ConnectTimeout=8 "$HOST" "$@"; }
_master_up

measure() {
    # 返回 "offset rtt"（板子时钟 − 主机时钟，取中点估计）
    local best_off=0 best_rtt=999 t0 t1 b off rtt
    for _ in 1 2 3 4 5; do
        t0=$(date +%s.%N)
        b=$(R 'date +%s.%N' 2>/dev/null | tr -d '\r')
        t1=$(date +%s.%N)
        [ -z "$b" ] && continue
        off=$(Env/bin/python -c "print(f'{$b - ($t0 + $t1) / 2:.6f}')")
        rtt=$(Env/bin/python -c "print(f'{$t1 - $t0:.6f}')")
        # 取 RTT 最小的那次
        if Env/bin/python -c "import sys; sys.exit(0 if $rtt < $best_rtt else 1)"; then
            best_off=$off; best_rtt=$rtt
        fi
    done
    echo "$best_off $best_rtt"
}

read -r OFF RTT <<<"$(measure)"
echo "[clock] 板子时钟 − 主机时钟 = $(Env/bin/python -c "print(f'{$OFF:+.3f}')") s" \
     "（最小 RTT ${RTT}s）"
if [ "$CHECK" = "1" ]; then
    exit 0
fi
# 单次设定 + 一次复核。为什么不迭代到底：ssh 的"命令真正执行时刻"只能靠 RTT 中点估，
# 迭代会把中点估计的**偏差**当成真实残差去补，反而来回震荡（实测 ±1.27 s 抖动）。
# 单次设定的残差就是 RTT/2（20~50 ms），已经够用（166 单位/秒 → <9 单位）。
set_clock() {
    local target
    target=$(Env/bin/python -c "print(f'{$(date +%s.%N) + $1 / 2:.6f}')")
    R "date -s @$target" >/dev/null 2>&1
}
if ! Env/bin/python -c "import sys; sys.exit(0 if abs($OFF) < 0.05 else 1)"; then
    set_clock "$RTT"
    sleep 0.5
    read -r OFF2 RTT2 <<<"$(measure)"
    echo "[clock] 已写入，复核残差 $(Env/bin/python -c "print(f'{$OFF2:+.3f}')") s（RTT ${RTT2}s）"
    OFF=$OFF2
fi
if Env/bin/python -c "import sys; sys.exit(0 if abs($OFF) < 0.08 else 1)"; then
    echo "[clock] ✓ 两块板子时间已对齐（残差 <80 ms；对账另有 vision_accuracy.py 兜底）"
else
    echo "[clock] ⚠ 残差 ${OFF}s：对账请用 vision_accuracy.py（按轨迹搜偏移，不依赖墙钟）"
fi
