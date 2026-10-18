#!/bin/bash
# 量"命令进仿真 → 物理真的动"的延迟。单独成文件是因为 pkill -f 的模式会匹配到
# 调用它的那个 shell 自己的命令行（自伤），在 bash -c 里写会把自己杀掉。
set -u
cd /home/zhang/FPGA
pkill -f "bullet_sim board-control" 2>/dev/null
sleep 0.5
: > /tmp/lat_stream.txt
rm -f /tmp/lat_truth.jsonl
setsid sh -c "tail -f /tmp/lat_stream.txt | Env/bin/python -u -m bullet_sim board-control --stdin --nostop --duration 40 --fullscreen --no-hud --obstacle-delay 35 --state-log /tmp/lat_truth.jsonl" > /tmp/lat_sim.log 2>&1 &
sleep 8
for step in 1 3 2 0; do
  echo "@MOVE $step" >> /tmp/lat_stream.txt
  Env/bin/python -c "import time;print(f'emit $step at {time.time():.3f}')"
  sleep 3
done
sleep 1
pkill -f "bullet_sim board-control" 2>/dev/null
sleep 0.3
echo done
