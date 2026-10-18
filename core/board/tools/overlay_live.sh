#!/bin/bash
# 实时看"板子此刻识别到什么" —— 窗口直接开在 Windows 屏幕（不录像）。
#
#    bash core/board/tools/overlay_live.sh                 # 只看识别叠加
#    bash core/board/tools/overlay_live.sh --decide        # 同时让板子发动作（板子真的在开车）
#    bash core/board/tools/overlay_live.sh --local         # 只看本机仿真 HUD，不连板子
#
# 画面含义（`wall_gap_view` 画的）：
#    绿框 = 检出的墙行带      红框 = 缺口
#    橙竖线 = 缺口中心（决策要瞄的点）   青框 = 角色候选
#
# 为什么是"板子画、主机显示"：相机挂在板子上，而**主机没有 ffmpeg**（实测），
# 所以链路是  ssh 拉板子的原始 RGB → 本机 pygame 窗口。
# 前提：仿真窗口必须在最前（相机拍的是屏幕）→ 脚本会自动 focus_sim.sh。
set -u
cd "$(dirname "$0")/../../.." || exit 1
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
CP=${CP:-/tmp/ssh_mux_board}
PY=Env/bin/python
SCALE=${SCALE:-1.6}
DECIDE=0
LOCAL=0
for arg in "$@"; do
    case "$arg" in
        --decide) DECIDE=1 ;;
        --local) LOCAL=1 ;;
        *) echo "未知参数 $arg"; exit 2 ;;
    esac
done

if [ "$LOCAL" = "1" ]; then
    echo "[看] 本机仿真 HUD（板子那行显示收到多少 @MOVE、最后方向、丢过期数）"
    exec $PY -m bullet_sim board-control --stdin --nostop --duration 600 --fullscreen
fi

# 板子侧：采集 → 画检测结果 → 原始 RGB 出到 stdout
KB=""
[ "$DECIDE" = "1" ] && KB="--keyboard-dev stdout"

# 连接复用：反复 ssh 时不会有 0.3~1.5 s 的建会话开销
ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    -o ControlMaster=auto -o ControlPath="$CP" -o ControlPersist=120 \
    -n "$HOST" 'true' 2>/dev/null

echo "[看] 板子链路：相机 → 检测叠加（绿=墙带 红=缺口 橙=缺口中心 青=角色）"
echo "     窗口里：ESC/q 退出，+/- 缩放"
if [ "$DECIDE" = "1" ]; then
    # 板子确实在发 @MOVE：需要一个仿真在跑才会动，这里给出提示而不是偷偷起一个
    echo "[看] --decide：板子会把 @MOVE 发到 stdout，但**这条链路是给显示用的**。"
    echo "     要真开车请另开一个终端跑：bash core/board/tools/board_vision_demo.sh 120"
    KB=""
fi

ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    -o ControlPath="$CP" -n "$HOST" \
    '/root/tools/v4l2_mjpeg_pipe --dev /dev/video0 --bufs 2 --w 640 --h 480 \
     | ffmpeg -hide_banner -loglevel error -f mjpeg -i - -vf scale=640:360 \
              -pix_fmt rgb24 -f rawvideo - \
     | /root/tools/wall_gap_view --w 640 --h 360 --skip 10 --verbose-every 30' \
    2>/tmp/overlay_live.err \
  | $PY core/board/tools/view_raw_rgb.py --w 640 --h 360 --scale "$SCALE" \
        --title "board view (green=wall red=gap orange=aim cyan=player)"

rc=$?
ssh -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" -n "$HOST" \
    'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x v4l2_mjpeg_pipe 2>/dev/null; pkill -INT -x wall_gap_view 2>/dev/null; true' 2>/dev/null
[ "$rc" -ne 0 ] && echo "[看] 显示端退出码 $rc（板子侧日志：/tmp/overlay_live.err）"
exit 0
