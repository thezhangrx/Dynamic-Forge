#!/usr/bin/env bash
# ===========================================================================
# run_loop.sh —— 闭环**常驻**：【摄像头 → PL → 决策】在板子上，【按键进平台】在 PC
#
#     bash core/board/tools/run_loop.sh              # 一直跑（Ctrl-C 停）
#     bash core/board/tools/run_loop.sh --keys wasd  # 平台用 WASD
#     bash core/board/tools/run_loop.sh --dry-run    # 只打印要按的键，不注入
#
# 链路（一条命令，全程不用手工挂任何串口）：
#
#     板子: 摄像头 → PL 视觉 → 解码 → 推速度 → 决策 → "@MOVE n" 到 stdout
#       │  ssh（已经在用的通道）
#      PC:  pc_key_driver.py --stdin --arm → PowerShell SendInput → 平台
#
# 为什么要有这个脚本（而不是一条 ssh | python 就完了）：
#   * **上电顺序**：先自检（板子通不通 / PL 在不在 / 相机在不在 / 注入器能不能起）
#     再开始按键 —— 否则你会对着一个黑屏猜是不是自己没配好；
#   * **焦点**：合成按键打到**当前焦点窗口**。开始前会报出焦点窗口标题、并留 5 秒
#     让你把平台点出来（`--delay-s 0` 可跳过）；
#   * **断了会自己接上**：SSH 掉线/相机抽风时自动重启（间隔 2 秒，连续失败退避到
#     10 秒），不会静默变成"什么都没发生"；
#   * **收尾干净**：Ctrl-C 时先松开所有键（ALLUP），再断开 SSH —— 不会把方向键
#     卡在按住状态让角色一直跑；
#   * **留档**：全程 tee 到 data/fpga/loop_logs/loop_<时间>.log，事后能查。
#
# ⚠ 别在 `--arm` 跑着的时候把焦点切走（比如点终端看输出）—— 那之后的按键会打到
#   新焦点窗口上（方向键还会在 shell 里翻历史）。
# ===========================================================================
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

KEYS=arrows
DELAY_S=5
DRY=0
RESTART=1
FRAMES=0                 # 0 = 不限帧数，一直跑
BOARD=root@192.168.137.242
KEYFILE="data/fpga/board_ssh/board_key"
DRIVER="core/rp2040_keyboard/pc_key_driver.py"
PY="$ROOT/Env/bin/python"

while [ $# -gt 0 ]; do
    case "$1" in
        --keys)      KEYS=$2; shift 2 ;;
        --delay-s)   DELAY_S=$2; shift 2 ;;
        --frames)    FRAMES=$2; shift 2 ;;
        --board)     BOARD=$2; shift 2 ;;
        --dry-run)   DRY=1; shift ;;
        --no-restart) RESTART=0; shift ;;
        -h|--help)   sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
    esac
done

SSH="ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ServerAliveInterval=5 -o ServerAliveCountMax=3"
LOG="data/fpga/loop_logs/loop_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$LOG")"

say() { printf '%s\n' "$*" | tee -a "$LOG"; }

# ---------------------------------------------------------------------------
# 自检：四件事都过了才动手
# ---------------------------------------------------------------------------
say "=== 闭环常驻 ===  日志：$LOG"
say "[1/4] 板子通不通 …"
if ! $SSH "$BOARD" 'echo ok' >/dev/null 2>&1; then
    say "  ✗ 连不上 $BOARD —— 先确认板子起来了（ssh 手动试一次）"
    exit 1
fi
PL=$($SSH "$BOARD" 'cat /sys/class/fpga_manager/fpga0/state 2>/dev/null' 2>/dev/null | tr -d '\r')
say "  ✓ 板子在线，PL 状态：${PL:-未知}"
if [ "${PL:-}" != "operating" ]; then
    say "  ⚠ PL 不是 operating：视觉跑不起来。板子上先跑 install_pl_autoload.sh status 看服务"
fi

say "[2/4] 相机在不在 …"
CAM=$($SSH "$BOARD" 'ls /dev/video* 2>/dev/null | tr "\n" " "' 2>/dev/null)
if [ -z "$CAM" ]; then
    say "  ✗ 板子上没有 /dev/video* —— 相机没插/没认到"
    exit 1
fi
say "  ✓ 相机节点：$CAM（cam_live.sh 会自己挑支持 MJPG 的那个）"

say "[3/4] 注入器能不能起（顺便报出当前焦点窗口）…"
if ! "$PY" "$DRIVER" --probe 2>&1 | tee -a "$LOG" | grep -q "已就绪"; then
    say "  ✗ 注入器起不来 —— 先单独跑：$PY $DRIVER --probe"
    exit 1
fi
say "  ✓ 注入器可用"

say "[4/4] 准备开跑：键位=$KEYS  帧数=$([ "$FRAMES" -eq 0 ] && echo 不限 || echo "$FRAMES")  $([ "$DRY" -eq 1 ] && echo '【干跑，不按键】' || echo '【真注入】')"
if [ "$DRY" -eq 0 ] && [ "$DELAY_S" -gt 0 ]; then
    say "  → 现在把**平台窗口**点成焦点，$DELAY_S 秒后开始按键（Ctrl-C 可取消）"
    sleep "$DELAY_S"
fi
if [ "$DRY" -eq 1 ]; then
    ARM=""
else
    ARM="--arm"
fi

# ---------------------------------------------------------------------------
# 主循环：断了自动接上
# ---------------------------------------------------------------------------
stop=0
#: 停掉板子上的相机链路。用 `pkill -x`（**按进程名精确匹配**）而不是 `-f`：
#: `pkill -f` 会连"命令行里含这个字符串的 shell"一起杀 —— 那个坑在 WSL 上踩过
#: 一次（把自己的会话杀了）。ffmpeg 一律用 -INT，**绝不 -9**：被 -9 的流式 ffmpeg
#: 会把 V4L2 设备卡在 "Device or resource busy"，只能靠 sysfs unbind/bind 救回来。
stop_remote() {
    $SSH "$BOARD" 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x vision_board_devmem 2>/dev/null; true' \
        >/dev/null 2>&1
}
on_stop() {
    stop=1
    echo
    echo "[run_loop] 收到停止信号：松开按键 → 停掉板子上的相机链路 → 断开"
    stop_remote
}
trap on_stop INT TERM
fails=0
round=0

while [ "$stop" -eq 0 ]; do
    round=$((round + 1))
    t_start=$(date +%s)
    say "---- 第 $round 轮  $(date +%H:%M:%S) ----"
    # ⚠ ssh 的 stderr **不能**并进管道：那会把 ssh 自己的报错当成"板子的输出"喂给
    #   驱动（有前缀过滤不至于误按键，但日志会很乱）。所以 ssh 的 stderr 直接进日志。
    # 驱动读到 EOF（SSH 断开/程序退出）会自己收工并把键全松（finally 里 ALLUP+QUIT）。
    {
        $SSH "$BOARD" "cd /root/tools && ./cam_live.sh --frames $FRAMES --via-pl --keyboard-dev stdout" 2>>"$LOG" \
            | "$PY" "$DRIVER" --stdin $ARM --keys "$KEYS" 2>&1 \
            | tee -a "$LOG"
    }
    rc=${PIPESTATUS[0]}
    break_at=$(date +%s)
    [ "$stop" -eq 1 ] && break
    # 跑够 30 秒就算"这次是好的"，把连续失败计数清掉（否则偶尔掉一次会累积成永久退避）
    [ $((break_at - t_start)) -ge 30 ] && fails=0

    if [ "$rc" -eq 0 ]; then
        # 正常结束只有一种情况：你给了有限的 --frames。这不是故障，别"重连"。
        say "[run_loop] 正常结束（跑完 $FRAMES 帧）"
        break
    fi
    if [ "$RESTART" -eq 0 ]; then
        say "[run_loop] 退出（rc=$rc），--no-restart 所以不重启"
        break
    fi
    fails=$((fails + 1))
    if [ "$fails" -le 3 ]; then
        say "[run_loop] 断了（rc=$rc），2 秒后重连（连续第 $fails 次）"
        sleep 2
    else
        say "[run_loop] 连续断 $fails 次了 —— 退避到 10 秒。常见原因：相机被别的程序占着、" \
            "pl-autoload 没起来、或者平台抢了焦点"
        sleep 10
    fi
done

stop_remote
say "[run_loop] 结束（共 $round 轮）。日志：$LOG"
