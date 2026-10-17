#!/bin/bash
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-60}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
PY=Env/bin/python
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$HOST" "$@"; }

bash core/board/tools/kill_sim.sh >/dev/null 2>&1
: > /tmp/bv_stream.txt

echo "[演示] 起仿真（全屏 + 势场 + HUD），${SECS}s ..."
(setsid sh -c "tail -f /tmp/bv_stream.txt | $PY -u -m bullet_sim board-control --stdin --nostop --duration $((SECS + 30)) --fullscreen --no-hud" > /tmp/bv_sim.log 2>&1 &)
sleep 4
echo '@MOVE 0' >> /tmp/bv_stream.txt
sleep 1

echo "[演示] 板子视觉闭环开始（相机 -> PL -> 决策 -> @MOVE）..."
R 'cd /root/tools && ./cam_live.sh --frames 0 --via-pl --keyboard-dev stdout --structural-gap --verbose' 2>/dev/null | tee -a /tmp/bv_stream.txt >/dev/null &
BOARD_PID=$!
sleep "$SECS"
kill "$BOARD_PID" 2>/dev/null
R 'pkill -f cam_live.sh; pkill -f vision_board_devmem' >/dev/null 2>&1
echo "[演示] 结束"
echo "决策样例:"
grep -a '@DBG' /tmp/bv_stream.txt | tail -3
echo "动作行数: $(grep -ac '@MOVE' /tmp/bv_stream.txt)"
echo "HUD:"
grep -a 'board     :' /tmp/bv_sim.log | tail -1
