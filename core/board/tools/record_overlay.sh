#!/bin/bash
# 录一段**板子相机视角 + 检测叠加**的视频（mp4），落在板子上，供拉回本地剪辑。
#
#   bash core/board/tools/record_overlay.sh [秒数=60] [输出=/tmp/boardview.mp4]
#
# 画面（`wall_gap_view` 画的）：绿=墙行带 红=缺口 橙=缺口中心 青=角色候选
#
# 为什么要单独一个脚本：`cam_live.sh` 跑的是**决策程序**（没有画面输出），
# 而 `wall_gap_view` 是**标注器**（有画面、不发 @MOVE）。两者不能同时开摄像头。
# 所以录"看得见的画面"时，平台的运动由**脚本化指令**驱动（见 MOTION=1），
# 这样录像里有真实运动，便于对着看检测跟不跟得上。
#
# MOTION=1（默认）：每 3 s 换一个方向推进平台；MOTION=0：平台静止。
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-60}
OUT_MP4=${2:-/tmp/boardview.mp4}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
CP=${CP:-/tmp/ssh_mux_board}
PY=Env/bin/python
FRAMES=$((SECS * 15))          # 标注链路实测 ~15~18 fps，按 15 估
MOTION=${MOTION:-1}
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ControlPath="$CP" -n "$HOST" "$@"; }

# 连接复用（建会话 0.3~1.5 s，反复 ssh 时不复用会拖慢）
ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    -o ControlMaster=auto -o ControlPath="$CP" -o ControlPersist=120 -n "$HOST" 'true' 2>/dev/null

: > /tmp/rec_stream.txt
date +%s.%N > /tmp/rec_start_epoch.txt
echo "[录] 起仿真（全屏），无板子决策、平台由脚本驱动 ..."
(setsid sh -c "tail -f /tmp/rec_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((SECS + 40)) --fullscreen --state-log /tmp/rec_truth.jsonl" \
   > /tmp/rec_sim.log 2>&1 &)
sleep 6
echo '@MOVE 0' >> /tmp/rec_stream.txt
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[录] /'

# 脚本化运动（让画面里有真实位移；不动也可以，墙自己会动）
if [ "$MOTION" = "1" ]; then
    ( for i in $(seq 1 60); do
          for a in 1 2 3 4; do
              echo "@MOVE $a" >> /tmp/rec_stream.txt
              sleep 3
          done
      done ) >/dev/null 2>&1 &
    MPID=$!
fi

echo "[录] 板子开始录 ${SECS}s → $OUT_MP4（标注：绿=墙带 红=缺口 橙=瞄准 青=角色）"
R "sh -c '/root/tools/v4l2_mjpeg_pipe --dev /dev/video0 --bufs 2 --w 640 --h 480 \
  | ffmpeg -hide_banner -loglevel error -f mjpeg -i - -vf scale=640:360 -pix_fmt rgb24 -f rawvideo - \
  | /root/tools/wall_gap_view --w 640 --h 360 --skip 10 --frames $FRAMES \
  | ffmpeg -hide_banner -loglevel error -f rawvideo -pix_fmt rgb24 -s 640x360 -r 15 -i - \
           -c:v libx264 -preset veryfast -pix_fmt yuv420p -y $OUT_MP4'"
R "ls -la $OUT_MP4"

[ "$MOTION" = "1" ] && kill "$MPID" 2>/dev/null
pkill -f "[b]ullet_sim board-control" 2>/dev/null
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x wall_gap_view 2>/dev/null; pkill -INT -x v4l2_mjpeg_pipe 2>/dev/null; true' >/dev/null 2>&1
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" \
    -o UserKnownHostsFile=/dev/null "$HOST:$OUT_MP4" /tmp/boardview.mp4 2>/dev/null
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" \
    -o UserKnownHostsFile=/dev/null "$HOST:/tmp/rec_truth.jsonl" /tmp/ 2>/dev/null
cp -f /tmp/boardview.mp4 /mnt/c/Users/Public/boardview.mp4 2>/dev/null && \
    echo "[录] 已放到 Windows：C:\\Users\\Public\\boardview.mp4"
cp -f /tmp/rec_truth.jsonl /mnt/c/Users/Public/boardview_truth.jsonl 2>/dev/null && \
    echo "[录] 真值：C:\\Users\\Public\\boardview_truth.jsonl（每行 unix + player + obstacles）"
ls -la /mnt/c/Users/Public/boardview.mp4 2>/dev/null
