#!/bin/sh
# 板端相机管线 —— **板端决策的唯一合法输入来源（真实相机）**。
#
#   /dev/video0 ──ffmpeg(mjpeg→320×240 rgb24)──> vt_board
#        └─同时写 /tmp/cam.jpg（给主机取快照/看画面）
#
# 用法：  sh /root/tools/start_cam.sh [quad.json]
#   quad.json = 主机用 ArUco 标记算出的页面四角（8 个数，320×240 输入坐标）；
#               存在则 vt_board 优先用它正射（`quad_ext=1`），否则自己找白框。
set -u
QUAD="${1:-/tmp/quad.json}"
VT_BIN="${VT_BIN:-/root/tools/vt_board}"   # A/B：vt_board（稠密）或 vt_sparse
CAM_FPS="${CAM_FPS:-2}"                    # 相机→VT 的帧率；决策新鲜度靠它
echo camera > /tmp/vt_source
pkill -f "[v]t_board" 2>/dev/null
pkill -f "[v]t_sparse" 2>/dev/null
pkill -f "[f]fmpeg" 2>/dev/null
sleep 0.5
rm -f /tmp/vt.jsonl /tmp/cam.jpg

exec ffmpeg -hide_banner -loglevel error \
    -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 \
    -f v4l2 -input_format mjpeg \
    -video_size 640x480 -i /dev/video0 \
    -filter_complex "fps=${CAM_FPS},scale=320:240,split=2[a][b]" \
    -map "[a]" -f rawvideo -pix_fmt rgb24 - \
    -map "[b]" -update 1 -q:v 5 -y /tmp/cam.jpg 2>/dev/null \
  | "$VT_BIN" --w 320 --h 240 --frame-roi 0 0 320 240 \
        --quad-file "$QUAD" --quad-state /tmp/qb.txt --jsonl /tmp/vt.jsonl > /dev/null
