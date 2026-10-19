#!/bin/bash
# 录板子相机视角（**先开录制，10 秒后才起平台**）。
#
# 为什么这个顺序：之前是"先起仿真、6 秒后才开录制"，结果录像开头错过了墙刚生成
# 的那几秒（正好是要看的窗口）。现在改成：录制先跑起来 → 等 10 s（让相机/编码器
# 稳定）→ 再起仿真窗口 → 平台开始动，整段过程都在录像里。
#
#   bash core/board/tools/record_overlay_first.sh [秒数=60] [输出=/tmp/boardview.mp4]
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-60}
OUT_MP4=${2:-/tmp/boardview.mp4}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
CP=${CP:-/tmp/ssh_mux_board}
PY=Env/bin/python
LEAD=${LEAD:-0}                   # 开录制的同时就起平台（这轮按你的要求：立刻）
FRAMES=$(((SECS + LEAD) * 15))
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ControlPath="$CP" -n "$HOST" "$@"; }

ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    -o ControlMaster=auto -o ControlPath="$CP" -o ControlPersist=300 -n "$HOST" 'true' 2>/dev/null

echo "[录] ① 先开板子录制（${FRAMES} 帧 ≈ $((FRAMES / 15)) s）→ $OUT_MP4"
R "nohup sh -c '/root/tools/v4l2_mjpeg_pipe --dev /dev/video0 --bufs 2 --w 640 --h 480 \
  | ffmpeg -hide_banner -loglevel error -f mjpeg -i - -vf scale=640:360 -pix_fmt rgb24 -f rawvideo - \
  | /root/tools/wall_gap_view --w 640 --h 360 --skip 5 --frames $FRAMES \
  | ffmpeg -hide_banner -loglevel error -f rawvideo -pix_fmt rgb24 -s 640x360 -r 15 -i - \
           -c:v libx264 -preset veryfast -pix_fmt yuv420p -y $OUT_MP4' \
  >/tmp/rec_inner.log 2>&1 &" >/dev/null 2>&1
sleep 3
echo "[录] ② 录制已在跑；等 ${LEAD} s 再起平台（这样墙生成的那一刻也在录像里）"
sleep "$LEAD"

: > /tmp/rec_stream.txt
date +%s.%N > /tmp/rec_start_epoch.txt
(setsid sh -c "tail -f /tmp/rec_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((SECS + 60)) --fullscreen --state-log /tmp/rec_truth.jsonl" \
   > /tmp/rec_sim.log 2>&1 &)
sleep 6
echo '@MOVE 0' >> /tmp/rec_stream.txt
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[录] /'
echo "[录] ③ 平台已起，开始脚本化运动（每 3 s 换向）"
( for i in $(seq 1 40); do
      for a in 1 2 3 4; do
          echo "@MOVE $a" >> /tmp/rec_stream.txt
          sleep 3
      done
  done ) >/dev/null 2>&1 &
MPID=$!

sleep $((SECS - 8))
kill "$MPID" 2>/dev/null
echo '@MOVE 0' >> /tmp/rec_stream.txt
sleep 8     # 让录像录到"平台停下"的收尾
pkill -f "[b]ullet_sim board-control" 2>/dev/null
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x wall_gap_view 2>/dev/null; pkill -INT -x v4l2_mjpeg_pipe 2>/dev/null; sleep 1; ls -la '"$OUT_MP4" >/dev/null 2>&1
ping -c1 -W1 127.0.0.1 >/dev/null 2>&1

scp -q -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" \
    -o UserKnownHostsFile=/dev/null "$HOST:$OUT_MP4" /tmp/boardview.mp4 2>/dev/null
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" \
    -o UserKnownHostsFile=/dev/null "$HOST:/tmp/rec_truth.jsonl" /tmp/ 2>/dev/null
M=$(wslpath -w /tmp/boardview.mp4 2>/dev/null)
T=$(wslpath -w /tmp/rec_truth.jsonl 2>/dev/null)
if [ -n "$M" ]; then
    /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -NoProfile -Command \
      "Copy-Item '$M' 'C:\Users\Public\1_board_camera.mp4' -Force; Copy-Item '$T' 'C:\Users\Public\1_board_camera_truth.jsonl' -Force" \
      >/dev/null 2>&1
    echo "[录] 已放到 C:\\Users\\Public\\1_board_camera.mp4（+ _truth.jsonl）"
fi
ls -la /tmp/boardview.mp4
