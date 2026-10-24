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
CAM_W="${CAM_W:-320}"                      # VT 输入宽（640 = 页面原始分辨率，精度更好）
CAM_H="${CAM_H:-240}"
echo camera > /tmp/vt_source
pkill -f "[v]t_board" 2>/dev/null
pkill -f "[v]t_sparse" 2>/dev/null
pkill -f "[f]fmpeg" 2>/dev/null
sleep 0.5
rm -f /tmp/vt.jsonl /tmp/cam.jpg
#: **锁死相机曝光/白平衡**（第 3 项精度措施）。场景是固定屏幕 + 一个很亮的灯斑，
#: 自动曝光会逐帧微调增益，墙的辉光厚度跟着变，墙行中心就漂 ±8 px。锁死之后
#: 亮度只由场景决定。若现场光变了导致画面过暗/过亮，改 EXPOSURE_ABS 即可。
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 \
             --set-ctrl=exposure_absolute="${EXPOSURE_ABS:-200}" \
             --set-ctrl=white_balance_temperature_auto=0 \
             --set-ctrl=white_balance_temperature="${WB_TEMP:-4000}" \
             --set-ctrl=gain="${GAIN:-4}" 2>/dev/null || true
fi

exec ffmpeg -hide_banner -loglevel error \
    -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 \
    -f v4l2 -input_format mjpeg \
    -video_size 640x480 -i /dev/video0 \
    -filter_complex "fps=${CAM_FPS},scale=${CAM_W}:${CAM_H},split=2[a][b]" \
    -map "[a]" -f rawvideo -pix_fmt rgb24 - \
    -map "[b]" -update 1 -q:v 5 -y /tmp/cam.jpg 2>/dev/null \
  | "$VT_BIN" --w "$CAM_W" --h "$CAM_H" --frame-roi 0 0 "$CAM_W" "$CAM_H" \
        --quad-file "$QUAD" --quad-state /tmp/qb.txt --jsonl /tmp/vt.jsonl > /dev/null
