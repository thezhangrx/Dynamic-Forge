#!/usr/bin/env bash
# ===========================================================================
# loop_watchdog.sh —— 跑闭环 + 自动验收 + 失败重跑（看门狗）
#
# 为什么要有：跑一轮要 5–10 分钟，中途崩了（相机被占、页面没起来、板子掉线）
# 以前得人盯着重开。这个脚本按"跑 → 验收 → 不过就重跑"的顺序自动重试，
# 并把每轮的验收结论记进日志。
#
# 用法：
#   bash core/board/tools/loop_watchdog.sh                 # 最多 3 次
#   bash core/board/tools/loop_watchdog.sh --tries 5 --tag pg
#
# 判据：以 grade_run.py 的结果为准（到达/碰撞/最小间隙/覆盖率/|Δy|）。
# 一旦 PASS 就停；连续失败到上限就停并打印最后一轮的失败项。
# ===========================================================================
set -u
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

TRIES=3
TAG="wd"
while [ $# -gt 0 ]; do
    case "$1" in
        --tries) TRIES=$2; shift 2 ;;
        --tag)   TAG=$2; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

PY="$ROOT/Env/bin/python"
LOG=/tmp/watchdog.log
: > "$LOG"
echo "[watchdog] $(date '+%H:%M:%S') 开始，最多 $TRIES 轮" | tee -a "$LOG"

for i in $(seq 1 "$TRIES"); do
    TAGI="${TAG}$i"
    echo "[watchdog] $(date '+%H:%M:%S') 第 $i 轮 → tag=$TAGI" | tee -a "$LOG"
    bash core/board/tools/loop_standard.sh --tag "$TAGI" >>"$LOG" 2>&1
    RUN=$(ls -dt data/fpga/loop_logs/run_*"$TAGI"* 2>/dev/null | head -1)
    if [ -z "$RUN" ]; then
        echo "[watchdog] 第 $i 轮没产生 run 目录（启动就失败了）" | tee -a "$LOG"
        continue
    fi
    if "$PY" core/board/tools/grade_run.py "$RUN" >>"$LOG" 2>&1; then
        echo "[watchdog] ✅ 第 $i 轮 PASS：$RUN" | tee -a "$LOG"
        "$PY" core/board/tools/grade_run.py "$RUN" | sed 's/^/[watchdog] /'
        exit 0
    fi
    echo "[watchdog] ❌ 第 $i 轮 FAIL：$RUN" | tee -a "$LOG"
    "$PY" core/board/tools/grade_run.py "$RUN" 2>/dev/null | sed 's/^/[watchdog] /' | tail -7
done

echo "[watchdog] ❌ 连续 $TRIES 轮都没通过，停止（日志：$LOG）" | tee -a "$LOG"
exit 1
