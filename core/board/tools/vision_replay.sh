#!/bin/bash
# ===========================================================================
# vision_replay.sh —— 视觉离线对账（相机在板子上，**视频留在板子、只回传日志**）
#
#     bash core/board/tools/vision_replay.sh [秒数]
#
# 为什么这么分工（回答"本地处理还是板子处理"）：
#   * 相机是 UVC 设备，**只有板子能打开**（PC 拿不到 /dev/video0）；
#   * 视频 100 MB 级，来回传很慢；几何日志只有几十 KB；
#   * 同一段视频要**反复跑**（改判据 → 重跑），留在板子上最省事。
#   所以：**板子上录、板子上处理（板子自己的视觉程序）、只把日志传回本地对账**。
#   需要看某几帧画面时，再单独取帧（几 MB），不必整段传。
#
# 步骤：
#   1) 平台上跑起来 + 录一段（板子）/tmp/vis/video.mkv，同时真值写本地 state.jsonl
#   2) 板子上抽帧 → vision_board_devmem --ppm-dir（PS 参考）与 --via-pl（PL）各跑
#   3) 只把两个日志 scp 回来，与真值按时间对账
# ===========================================================================
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-30}
OUT=${OUT:-/tmp/vis}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
PY=Env/bin/python
FPS=${FPS:-30}
mkdir -p "$OUT"

R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$HOST" "$@"; }

echo "[1/3] 平台上跑起来 + 板子录 ${SECS}s ..."
if [ "${SKIP_RECORD:-0}" = "1" ] && [ -f "$OUT/state.jsonl" ]; then
    echo "    跳过录制与真值（SKIP_RECORD=1，复用已有视频+真值）"
elif [ "${REUSE:-0}" = "1" ] && R 'test -f /tmp/vis_video.mkv' 2>/dev/null; then
    echo "    复用板子上已有视频（REUSE=1）"
else
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
rm -f /tmp/mv.txt "$OUT/state.jsonl"; touch /tmp/mv.txt
(setsid sh -c "tail -f /tmp/mv.txt | $PY -u -m bullet_sim play --input auto --obstacle-type wall_with_gap --stage-seconds 0 \
    --fullscreen --duration $((SECS + 25)) --state-log $OUT/state.jsonl" \
    > "$OUT/sim.log" 2>&1 &)
sleep 4
R 'ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 1280x720 -i /dev/video0 -t '"$SECS"' -r '"$FPS"' -y /tmp/vis_video.mkv; ls -l /tmp/vis_video.mkv' || exit 1
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
fi
echo "    真值帧: $(wc -l < "$OUT/state.jsonl" 2>/dev/null || echo 0)"

echo "[2/3] 板子抽帧 + 视觉处理 ..."
R 'rm -rf /tmp/vis_ppm && mkdir -p /tmp/vis_ppm && ffmpeg -hide_banner -loglevel error -i /tmp/vis_video.mkv -vf "scale=640:360" -start_number 0 /tmp/vis_ppm/%06d.ppm && ls /tmp/vis_ppm | wc -l'
N=$(R 'ls /tmp/vis_ppm | wc -l')
echo "    抽帧 $N"
R "/root/board_src/build/vision_board_devmem --ppm-dir /tmp/vis_ppm --frames $N --structural-gap --verbose 2>&1" > "$OUT/board_ps.log" 2>&1
echo "    注入行: $(grep -c '注入#' "$OUT/board_ps.log" 2>/dev/null || echo 0)"

echo "[3/3] 与真值对账（本地）..."
$PY core/board/tools/compare_frames.py "$OUT/state.jsonl" "$OUT/board_ps.log" --rows 12 || true
