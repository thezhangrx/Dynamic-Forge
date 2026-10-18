#!/bin/bash
# 量 `tail -f 文件 | 消费进程` 这条链路的延迟：写进去的行多久能被读到。
# 这条链路是"板子 @MOVE → ssh → tee 文件 → tail -f → 仿真 stdin"的中间段。
set -u
cd /home/zhang/FPGA
: > /tmp/pipe_test.txt
setsid sh -c "tail -f /tmp/pipe_test.txt | Env/bin/python -u -c '
import sys, time
for line in sys.stdin:
    print(f\"recv {time.time():.3f} {line.strip()}\", flush=True)
'" > /tmp/pipe_test.log 2>&1 &
sleep 1
for i in 1 2 3 4 5; do
  echo "@MOVE $i" >> /tmp/pipe_test.txt
  Env/bin/python -c "import time;print(f'send {time.time():.3f} @MOVE $i')"
  sleep 1
done
sleep 1
pkill -f "tail -f /tmp/pipe_test.txt" 2>/dev/null
sleep 0.3
echo ---
cat /tmp/pipe_test.log
