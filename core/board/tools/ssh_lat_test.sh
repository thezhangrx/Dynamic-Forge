#!/bin/bash
# 量 ssh 这一段：板子上 `date` 打点 → 主机读到那一刻，差多少。
# 板子会把**它自己的发送时刻**写进行里，主机把**到达时刻**写在行首，二者相减就是延迟。
set -u
cd /home/zhang/FPGA
KEY=data/fpga/board_ssh/board_key
HOST=root@192.168.137.242
ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -n "$HOST" \
    'for i in 1 2 3 4 5 6; do echo "@MOVE 1 sent=$(date +%s.%N)"; sleep 1; done' 2>/dev/null \
 | Env/bin/python -u -c '
import sys, time
for line in sys.stdin:
    print(f"recv_at={time.time():.3f} {line.strip()}", flush=True)
' 2>&1 | Env/bin/python -u -c '
import sys
for line in sys.stdin:
    parts = dict(p.split("=", 1) for p in line.split() if "=" in p)
    if "recv_at" in parts and "sent" in parts:
        d = float(parts["recv_at"]) - float(parts["sent"])
        print("  ssh 传输延迟 %+.3f s   (%s)" % (d, line.strip()))
    else:
        print(line.rstrip())
'
