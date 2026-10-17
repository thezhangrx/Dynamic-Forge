#!/bin/bash
# 只起仿真（带状态包），供性能/延迟测量用。参数原样透传给 bullet_sim。
# 单独成文件的原因：在 bash -c 里直接写仿真命令行，会让 kill_sim.sh 的 pkill
# 匹配到调用者自己（自伤），这个坑踩过两次。
set -u
cd "$(dirname "$0")/../../.." || exit 1
OUT=${OUT:-/tmp/sp.pkt}
SECS=${SECS:-10}
rm -f "$OUT"; touch "$OUT"
(setsid sh -c "tail -f /tmp/mv.txt | Env/bin/python -u -m bullet_sim board-control \
    --stdin --nostop --duration $SECS --state-pipe $OUT $*" > /tmp/sim_only.log 2>&1 &)
sleep 2
echo '@MOVE 0' >> /tmp/mv.txt
