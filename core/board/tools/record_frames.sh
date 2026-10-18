#!/bin/bash
# 录一批"相机看到的真实帧" + 同时刻的真值状态，供离线算法开发用。
#
# 为什么要专门有个录制脚本：视觉那条链上的问题（标记丢了、角色漂了、速度不对）
# 在实时环路里**根本看不出来** —— 一帧 80 ms，日志只有几行数字，没有画面。
# 而"改一次阈值上板跑一轮"要好几分钟。把帧和真值一起录下来，之后所有算法试验
# 都在本地用 `contour_probe` / `overlay_band_gap.py` 在同一批帧上做，几秒一轮，
# 而且**同一批帧可以互相比较**（换了算法再跑一遍，差异是算法的，不是画面的）。
#
# 用法:
#   bash core/board/tools/record_frames.sh [秒数] [每隔几帧存一张]
#   OUT_DIR=/tmp/vis/rec bash core/board/tools/record_frames.sh 20 2
#
# 产物（本地）：
#   $OUT_DIR/ppm/*.ppm   相机→缩放后 640x360 原始帧（和板子程序吃的是同一份像素）
#   $OUT_DIR/board.log   板子日志（每帧的识别结果、阈值、标记数）
#   $OUT_DIR/truth.jsonl 仿真真值状态包（@STATE 逐帧）
#
# ⚠ 录的时候仿真窗口必须在前台、别被别的窗口挡住 —— 挡住的那几帧会被录进来，
#   离线复现的时候会被当成"算法错"。脚本结束会打印"看不到场地"的帧数，先看这个数。
set -u
cd "$(dirname "$0")/../../.." || exit 1

SECS=${1:-20}
EVERY=${2:-2}
OUT_DIR=${OUT_DIR:-/tmp/vis/rec}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
PY=Env/bin/python
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

mkdir -p "$OUT_DIR/ppm"
rm -f "$OUT_DIR/ppm"/*.ppm "$OUT_DIR/board.log" "$OUT_DIR/truth.jsonl"

bash core/board/tools/kill_sim.sh >/dev/null 2>&1
rm -f /tmp/rc_mv.txt; touch /tmp/rc_mv.txt

echo "[录制] 起仿真 ${SECS}s（不显示 HUD —— HUD 的字会被相机一起拍进去，算噪声）..."
(setsid sh -c "$PY -u -m bullet_sim board-control --stdin --nostop \
    --duration $((SECS + 40)) --fullscreen --no-hud \
    --state-log '$OUT_DIR/truth.jsonl' < /tmp/rc_mv.txt" > /tmp/rc_sim.log 2>&1 &)
sleep 4
echo '@MOVE 0' >> /tmp/rc_mv.txt
sleep 1

echo "[录制] 板子开始采（每 ${EVERY} 帧存一张）..."
R "cd /root/tools && rm -rf /tmp/rec && DUMP_DIR=/tmp/rec DUMP_EVERY=$EVERY DUMP_MAX=400 \
   ./cam_live.sh --frames 0 --via-pl --keyboard-dev stdout --structural-gap --verbose" \
  > "$OUT_DIR/board.log" 2>/dev/null &
BPID=$!
sleep "$SECS"
kill "$BPID" 2>/dev/null
R 'pkill -f cam_live.sh; pkill -f vision_board_devmem' >/dev/null 2>&1
sleep 1

echo "[录制] 取回帧..."
R 'ls /tmp/rec/*.ppm 2>/dev/null' | while read -r f; do
    [ -n "$f" ] && scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        "$HOST:$f" "$OUT_DIR/ppm/" 2>/dev/null
done
bash core/board/tools/kill_sim.sh >/dev/null 2>&1

N=$(ls "$OUT_DIR/ppm"/*.ppm 2>/dev/null | wc -l)
echo "[录制] 完成：$N 帧 → $OUT_DIR/ppm"
echo "[录制] 板子日志里看不到场地的帧数：$(grep -ac '看不到场地边框' "$OUT_DIR/board.log")"
echo "[录制] 标记=4 的比例：$(grep -ao '标记=4 的帧：[0-9]* / [0-9]*' "$OUT_DIR/board.log" | tail -1)"
echo "[录制] 真值帧数：$(wc -l < "$OUT_DIR/truth.jsonl" 2>/dev/null || echo 0)"
