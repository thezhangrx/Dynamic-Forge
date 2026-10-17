#!/bin/bash
# ===========================================================================
# vision_replay.sh —— 视觉离线对账（相机在板子上：**直接录 PPM，不落 mkv**）
#
#     bash core/board/tools/vision_replay.sh [秒数]
#     SKIP_RECORD=1 bash ...        # 复用板上已有 ppm 序列与真值，只重跑处理+对账
#
# 为什么直接录 PPM：
#   * 先录 mkv 再抽帧，实测画面布满红紫噪声 —— 这块 UVC 的 MJPG 带非标准 APP 字段，
#     "先存流再解码"这条路会坏；
#   * cam_live 那条路（ffmpeg 直接输出 rgb24）是验证过可用的，所以这里也用同一种解码，
#     只是把结果落成 PPM 序列。
#
# 录制前**先自检**：拍一帧跑一遍板子视觉，确认能看到场地（标记数=4）。
# 看不到就直接报错退出 —— 之前多次是"相机对着浏览器在跑视觉"，白跑。
# ===========================================================================
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-12}
OUT=${OUT:-/tmp/vis}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
PY=Env/bin/python
FPS=${FPS:-30}
mkdir -p "$OUT"
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$HOST" "$@"; }

echo "[1/3] 起平台 + 自检 + 录 PPM（${SECS}s）..."
if [ "${SKIP_RECORD:-0}" != "1" ]; then
  bash core/board/tools/kill_sim.sh >/dev/null 2>&1
  rm -rf /tmp/vp; R 'rm -rf /tmp/vp && mkdir -p /tmp/vp'
  rm -f /tmp/mv.txt "$OUT/state.jsonl"; touch /tmp/mv.txt
  (setsid sh -c "tail -f /tmp/mv.txt | $PY -u -m bullet_sim play --input auto --obstacle-type wall_with_gap --stage-seconds 0 --fullscreen --duration $((SECS + 25)) --state-log $OUT/state.jsonl" > "$OUT/sim.log" 2>&1 &)
  sleep 8

  # **相机控制要和 cam_live 一致**：之前我自己用裸 ffmpeg 采集，没设 gain/brightness，
  # 相机用自动增益 → 彩噪 17~19（干净基准 2.0）。cam_live 一直设 gain=4+brightness=70，
  # 这条链路实测标记 100% 能找到。教训：采集参数不同，测出来的"相机坏了"就是假的。
  R 'v4l2-ctl -d /dev/video0 -c exposure_auto=3 -c white_balance_temperature_auto=1 -c gain=4 -c brightness=70 -c saturation=100 -c contrast=32 -c gamma=120 -c backlight_compensation=0' >/dev/null 2>&1

  # ---- 录制前自检：拍一帧，看板子能不能认出场地 ----
  # 取**多帧、留最后一帧**：第 1 帧常常是相机旧缓冲（实测前两帧完全相同），
  # 单帧自检会误判"没有场地"。
  R 'ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 -frames:v 12 -vf "scale=640:360" -y /tmp/pre_%02d.ppm; cp /tmp/pre_12.ppm /tmp/pre.ppm'
  R '/root/board_src/build/vision_board_devmem --ppm /tmp/pre.ppm --frames 1 --structural-gap --verbose 2>&1' > "$OUT/precheck.log" 2>&1
  # 自检条件：**必须凑齐四角标记**（单应才对；只量到场地/门但没标记时，
  # 识别坐标会飞出去 —— 上一轮就是因为这个把 (804,764) 这种坐标当成了角色）
  if ! grep -aq "标记=4" "$OUT/precheck.log"; then
      echo "!! 自检失败：板子在这一帧里没看到场地（标记数≠4）。"
      echo "   常见原因：仿真窗口不在最前 / 相机没对着屏幕。"
      echo "   —— 先解决这个再录，否则录到的是无效素材。"
      grep -a "标记=" "$OUT/precheck.log" | head -2
      bash core/board/tools/kill_sim.sh >/dev/null 2>&1
      exit 2
  fi
  echo "    自检通过（四角标记齐全）"

  R "ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 -t $SECS -r $FPS -vf scale=640:360 -start_number 0 /tmp/vp/%06d.ppm"
  bash core/board/tools/kill_sim.sh >/dev/null 2>&1
else
  echo "    SKIP_RECORD=1：复用板上已有 ppm 序列与真值"
fi
N=$(R 'ls /tmp/vp | wc -l')
echo "    帧数 $N，真值帧 $(wc -l < "$OUT/state.jsonl" 2>/dev/null || echo 0)"

echo "[2/3] 板子视觉处理（PS 参考模型）..."
R "/root/board_src/build/vision_board_devmem --ppm-dir /tmp/vp --frames $N --structural-gap --verbose 2>&1" > "$OUT/board_ps.log" 2>&1
echo "    注入行: $(grep -ac '注入#' "$OUT/board_ps.log" 2>/dev/null || echo 0)"
echo "    标记=4 的帧: $(grep -ao '标记=4' "$OUT/board_ps.log" | wc -l) / $(grep -ac '结构\]' "$OUT/board_ps.log" 2>/dev/null || echo 0)"

echo "[3/3] 与真值对账..."
$PY core/board/tools/compare_frames.py "$OUT/state.jsonl" "$OUT/board_ps.log" --time-scale "$(echo "60/$FPS" | bc)" --rows 10 || true
