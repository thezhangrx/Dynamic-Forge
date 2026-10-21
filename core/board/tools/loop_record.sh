#!/usr/bin/env bash
# ===========================================================================
# loop_record.sh —— 录音像的一轮闭环（固定参数，别手敲）
#
#   bash core/board/tools/loop_record.sh [--tag r1] [--seconds 200] [--check]
#
# 这一轮 = fix5 的配置（已到过终点、0 碰撞、24/24 墙全对）**加 --cam-record**，
# 目的只有一个：拿到一段"板子自己看、自己决策、自己过缺口"的完整录像。
#
# ── 2026-10-21 录像版（r11 / run_20261021_164153_r11）实测 ──────────────
#    到达=True  碰撞=0  学到 24/24 道墙  audit_crossings: 24/24"凭信息"（0 盲穿）
#    check_wall_rows: 静止帧墙行误差 median -0.3 px / p90 +0.9 px（|e|>20 占 0.5%）
#    用时 82.7 s 仿真，最小间隙 +8.61，相机 12 fps
#    → 视频：data/fpga/loop_logs/run_20261021_164153_r11/videos/
#      camera_vt.mp4（左=真实相机 右=板端 VT 自己的输出 + 暂停横幅）
#      field.mp4（虚拟真值渲染的势场+轨迹）  truth.mp4（纯真值）
#
# 这一版和 fix5 的差别（每一条都是上面那些卡死/盲穿的根因，别再改回去）：
#   1) gamma 固定 100：这台相机的自动曝光会收敛到"整幅发灰"，墙行会被配到灯斑上。
#   2) 角色检测改成"圆盘−环带"匹配滤波 + 形状复核：不再依赖绝对亮度阈值。
#   3) VT 丢掉**过高的亮行带**（>34 行）：屏幕反光是 60+ 行的大块亮区，会在页面
#      坐标里固定不动，被当成"正前方新墙"，最后把终点挡在墙后（r4/r5/r10 都这样）。
#   4) 暂停期间**持续发 0** 压住平台；settle 2.5 s、warmup 4 s。
#   5) 墙行同一性容差 12σ_y：同一道墙单次测量偶尔偏 27~29 px，容差太小就记成两道。
#   6) 录像**不进滤波图**（-map 0:v -c:v copy）：以前 mpeg4 编码拖住整张图，
#      喂给 VT 的是几秒前的画面。
#
#   * decision_source 必须是 camera：相机 → 板端 vt_sparse → 板端决策 → @MOVE
#   * --field-hz 0：热力图窗口会抢 pygame 的那一块 surface，相机就拍黑屏
#   * 录像在板端 /tmp/cam_rec.mkv，跑完用 make_full_videos.py 出成对视频
#
# 前置：屏幕上没有别的 page_view / platform_host 窗口盖住页面；
#       相机没被 mjpeg_server / calib 占着（本脚本会先清掉残留）。
# ===========================================================================
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

PY="$ROOT/Env/bin/python"
BOARD="root@192.168.137.242"
KEY="data/fpga/board_ssh/board_key"
QUAD_LOCAL="data/fpga/quad.json"
CALIB_LOCAL="data/fpga/calib_y.json"
SCENARIO="configs/20_walls_alt_24_dy140.json"
CHECK=0
TAG="rec"
SECONDS_SIM=200

while [ $# -gt 0 ]; do
    case "$1" in
        --tag)     TAG=$2; shift 2 ;;
        --seconds) SECONDS_SIM=$2; shift 2 ;;
        --check)   CHECK=1; shift ;;
        -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=8 $BOARD"

echo "== 1. 清残留（渲染窗口 / 相机占用）=="
pkill -x platform_host 2>/dev/null
pkill -x page_view 2>/dev/null
pkill -x ffmpeg 2>/dev/null
pkill -x vt_sparse 2>/dev/null
$SSH 'pkill -x ffmpeg; pkill -x vt_sparse; pkill -x mjpeg_server; rm -f /tmp/cam_rec.mkv; true' 2>/dev/null
sleep 1
LEFT=$(ps -eo pid,cmd | grep -aE "[p]latform_host" | wc -l)
echo "  本机 platform_host 残留 = $LEFT"

echo "== 1b. 相机回到自动曝光（任何手动实验之后都必须归位）=="
#: 这台相机的手动曝光只能比自动**更亮**，一旦被锁成手动（exposure_auto=1），
#: 下一轮拍到的就是一片全白 255（实测 r3：40 帧里 0 墙 0 角色）。
#: 所以每轮开跑前显式把"自动曝光 + 默认对比度/亮度"写回去，不靠上一次的状态。
$SSH 'v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto_priority=1 2>/dev/null;
      v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto=3 2>/dev/null;
      v4l2-ctl -d /dev/video0 --set-ctrl contrast=32 2>/dev/null;
      v4l2-ctl -d /dev/video0 --set-ctrl brightness=128 2>/dev/null;
      v4l2-ctl -d /dev/video0 --set-ctrl gamma=100 2>/dev/null;
      v4l2-ctl -d /dev/video0 --get-ctrl exposure_auto,exposure_auto_priority 2>/dev/null |
        sed "s/^/  /"' || true

echo "== 2. 板子与二进制 =="
$SSH 'ls -l /root/tools/vt_sparse /root/tools/board_agent.py /root/tools/field_cli_board 2>/dev/null |
        awk "{print \"  \", \$5, \$9}";
      echo "  quad: $(cat /tmp/quad.json 2>/dev/null)";
      echo "  calib: $(cat /tmp/calib_y.json 2>/dev/null | tr -d "\n " | cut -c1-90)"' || exit 3

[ "$CHECK" = "1" ] && { echo "== --check 结束 =="; exit 0; }

echo "== 3. 推 quad / 标定 =="
[ -f "$QUAD_LOCAL" ]  && scp -q -i "$KEY" -o StrictHostKeyChecking=no "$QUAD_LOCAL"  "$BOARD:/tmp/quad.json"
[ -f "$CALIB_LOCAL" ] && scp -q -i "$KEY" -o StrictHostKeyChecking=no "$CALIB_LOCAL" "$BOARD:/tmp/calib_y.json"

echo "== 4. 开跑（fix5 配置 + 相机录像）=="
exec "$PY" -u core/board/tools/platform_host.py \
    --scenario "$SCENARIO" \
    --port 8518 --tcp 9026 --speed 0.3 \
    --max-seconds "$SECONDS_SIM" --render-hz 30 --field-hz 0 --follow-scale 1 \
    --tag "$TAG" --hold 2 \
    --wall-rgb 255,240,200 --field-rgb 12,14,20 --bg-rgb 0,0,0 \
    --frame-px 2 --field-every 12 \
    --board-ssh "$BOARD" --ssh-key "$KEY" \
    --board-agent /root/tools/board_agent.py --board-python python3 \
    --board-extra "--from-end --verbose --consensus-frames 8 --tol-vy 16 --settle 2.5 --warmup 4.0 --pause-timeout 3.5" \
    --connect-timeout 90 \
    --calib "$CALIB_LOCAL" --calib-board /tmp/calib_y.json --quad-tol 6 \
    --viewer --viewer-fullscreen --viewer-pos 320,60 --viewer-size 1280x960 \
    --viewer-margin 0 \
    --cam-vt-bin /root/tools/vt_sparse --cam-fps 12 --cam-w 640 --cam-h 480 \
    --cam-record
