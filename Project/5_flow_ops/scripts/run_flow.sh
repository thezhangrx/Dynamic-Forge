#!/bin/sh
# run_flow.sh —— 板端**一键跑全流程**（只调用现有程序，不改任何一行逻辑）
#
#   相机设置 → 相机管线(ffmpeg|pl_vision) → board_agent(--stdio) → agent_bridge → 小车
#
# 用法（板子上）：
#   sh /root/tools/run_flow.sh --tag t1 [--seconds 60] [--car /dev/ttyUSB0] [--pose "320 40 0"]
#
# ⚠ 不改 board_agent.py / pl_vision / field_cli_board，只用它们现成的参数与 stdio 接口。
set -u
TAG="flow"; SECS=60; CAR=""; POSE="320 40 0"; QUAD=/tmp/quad.json
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG=$2; shift 2;;
    --seconds) SECS=$2; shift 2;;
    --car) CAR=$2; shift 2;;
    --pose) POSE=$2; shift 2;;
    --quad) QUAD=$2; shift 2;;
    *) echo "未知参数 $1"; exit 2;;
  esac
done
RUN=/root/runs/$TAG
mkdir -p "$RUN" || exit 3
echo "== [$TAG] 目录 $RUN"

echo "-- 1) 相机设置（调用现成脚本）"
[ -f /root/vision_state/cam_settings.sh ] && sh /root/vision_state/cam_settings.sh > "$RUN/cam_settings.log" 2>&1 \
  && echo "   cam_settings ok" || echo "   ⚠ 没找到 cam_settings.sh（跳过）"

echo "-- 2) 相机管线（调用现成 start_cam.sh，感知源换成 pl_vision）"
VT_BIN=/root/tools/pl_vision CAM_FPS=12 CAM_W=640 CAM_H=240 \
  nohup sh /root/tools/start_cam.sh "$QUAD" > "$RUN/cam_pipe.log" 2>&1 &
echo "   cam pipe pid=$!"
i=0
while [ $i -lt 25 ]; do
  [ -s /tmp/vt.jsonl ] && break
  i=$((i+1)); sleep 1
done
echo "   vt.jsonl 行数=$(wc -l < /tmp/vt.jsonl 2>/dev/null || echo 0)"

echo "-- 3) 桥接 agent（--stdio，逻辑一行不改）"
nohup python3 /root/tools/agent_bridge.py \
    --tag "$TAG" --car-port "$CAR" --pose-static "$POSE" \
    --log "$RUN/agent_bridge.jsonl" > "$RUN/bridge.out" 2>&1 &
echo "   bridge pid=$!"

echo "-- 4) 跑 $SECS 秒"
sleep "$SECS"

echo "-- 5) 收尾（只停本次起的进程，不 kill -9）"
for p in $(ps -eo pid,args | awk "/agent_bridge\.py --tag $TAG/ {print \$1}"); do kill -TERM $p 2>/dev/null; done
for p in $(ps -eo pid,args | awk "/[p]l_vision --w 640/ {print \$1}"); do kill -TERM $p 2>/dev/null; done
for p in $(ps -eo pid,args | awk "/[f]fmpeg .*video0/ {print \$1}"); do kill -TERM $p 2>/dev/null; done
sleep 1
cp /tmp/vt.jsonl "$RUN/vt.jsonl" 2>/dev/null
echo "== 完成：$RUN"
ls -la "$RUN" | awk '{print "  ", $5, $9}'
