#!/bin/bash
# 板子一回来就跑这个：推 main.cpp → 重编 → 跑一轮闭环（原样场景）→ 打关键统计
set -u
KEY=/home/zhang/FPGA/data/fpga/board_ssh/board_key
SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 root@192.168.137.242"
echo "== 1) 板子状态 =="
$SSH 'uptime; cat /sys/class/fpga_manager/fpga0/state; for d in /dev/video*; do timeout 5 v4l2-ctl -d $d --list-formats-ext 2>/dev/null | grep -q MJPG && echo "camera=$d"; done' || exit 1
echo "== 2) 推 main.cpp 并重编 =="
scp -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    /home/zhang/FPGA/core/board/main.cpp root@192.168.137.242:/root/board_src/core/board/main.cpp || exit 1
$SSH 'cd /root/board_src/build && make -B vision_board_devmem -j3 2>&1 | tail -2; md5sum vision_board_devmem' || exit 1
echo "== 3) 跑一轮闭环（20 秒，原样场景；PC 侧窗口请先起来） =="
(sleep 4; timeout 90 $SSH 'cd /root/tools && ./cam_live.sh --frames 250 --via-pl --keyboard-dev stdout --verbose --track-log /tmp/tv.txt' > /tmp/verify_board.log 2>&1) \
  | timeout 90 /home/zhang/FPGA/Env/bin/python -m bullet_sim board-control --stdin --max-seconds 18 > /tmp/verify_sim.log 2>&1
scp -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@192.168.137.242:/tmp/tv.txt /tmp/tv.txt >/dev/null 2>&1
echo "== 4) 统计 =="
/home/zhang/FPGA/Env/bin/python - <<'PY'
import re, collections
dec=[l for l in open('/tmp/verify_board.log') if '决策:' in l]
print(' 决策帧', len(dec))
print(' tier:', dict(collections.Counter(re.search(r'tier=(\S+)', l).group(1) for l in dec)))
print(' 矩形个数:', dict(sorted(collections.Counter(int(re.search(r'矩形=(\d+)', l).group(1)) for l in dec).items())))
print(' 角色落在标定标记上的帧:', sum(1 for l in dec if '标定标记' in l))
p=[l.split() for l in open('/tmp/tv.txt') if l.startswith('P ')]
xs=[float(f[1]) for f in p]
print(' 角色 x 范围: %.0f..%.0f' % (min(xs), max(xs)) if xs else ' 无角色行')
cmd=[]
for l in dec:
    m=re.search(r'指令=\(([-+0-9.]+),\s*([-+0-9.]+)\)', l)
    if m: cmd.append(float(m.group(1)))
same=sum(1 for i in range(1,len(p)) if abs(cmd[i-1])>0.3 and abs(float(p[i][1])-float(p[i-1][1]))>0.05 and (cmd[i-1]>0)==(float(p[i][1])>float(p[i-1][1])))
tot=sum(1 for i in range(1,len(p)) if abs(cmd[i-1])>0.3 and abs(float(p[i][1])-float(p[i-1][1]))>0.05)
print(' 指令方向与位移一致: %d/%d' % (same, tot))
PY
