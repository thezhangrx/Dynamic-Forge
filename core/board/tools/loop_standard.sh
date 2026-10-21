#!/usr/bin/env bash
# ===========================================================================
# loop_standard.sh —— 闭环**标准配置**（2026-10-21 起：std7/std9）
#
#   相机 → 板端 vt_sparse（12 fps）→ 板端决策 → @MOVE → 主机渲染
#
#   同一套参数重复两轮（std7 / std9），两轮都：
#      到达=True   碰撞=0   最小间隙 +18.2 / +16.0
#      学到 24/24 道墙   |Δy| 平均 2.7 / 3.1 px   VT 12.01 帧/s
#      t_sim 64.8 s / 63.0 s（比旧的 sparse6 慢 22 s：多停车换安全）
#      证据：data/fpga/loop_logs/run_20261021_074202_std7/
#            data/fpga/loop_logs/run_20261021_074639_std9/
#
#   2026-10-20 的旧标准（sparse6）只学到 18/24、擦墙一次，已不再作标准。
#
# 这个脚本就是"标准"本身：所有参数都在这里，**不要**再手敲一长串 --foo。
#
#     bash core/board/tools/loop_standard.sh                 # 跑一轮（tag=std）
#     bash core/board/tools/loop_standard.sh --tag try2      # 换个 tag
#     bash core/board/tools/loop_standard.sh --seconds 40    # 缩短仿真时长
#     bash core/board/tools/loop_standard.sh --check         # 只检查板子/标定，不开跑
#
# 前置条件（脚本会检查）：
#   * 板子 192.168.137.242 可 ssh（key: data/fpga/board_ssh/board_key）
#   * 板上有 /root/tools/vt_sparse、/root/tools/board_agent.py、/tmp/quad.json
#   * data/fpga/calib_y.json 存在（y 换算系数，自动标定产物，不要手改）
#   * 屏幕上**没有**别的 page_view/platform_host 窗口压在平台上面
#     （历史坑：残留的 --hold 渲染进程把冻结页面挂在最前面，相机就拍它）
#
# 已知坏点（别开）：
#   * --collect-moving 机制已修好，但**不如停车法**（std4：11/24、|Δy| 6.0），
#     标准配置不启用它。sparse7 那次 90 Hz resume 空转的假"到达"已作废。
#   * 按缺口身份预过滤候选（std8 试过）会把真墙滤掉（24/24→17/24），已回退。
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
TAG="std"
SECONDS_SIM=95

while [ $# -gt 0 ]; do
    case "$1" in
        --tag)      TAG=$2; shift 2 ;;
        --seconds)  SECONDS_SIM=$2; shift 2 ;;
        --check)    CHECK=1; shift ;;
        -h|--help)  sed -n '2,34p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
    esac
done

SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=8 $BOARD"

echo "== 1. 板子与二进制 =="
$SSH 'ls -l /root/tools/vt_sparse /root/tools/board_agent.py /root/tools/field_cli_board 2>/dev/null |
        awk "{print \"  \", \$5, \$9}";
      echo "  vt_source 源码 md5: $(md5sum /root/board_src/vt/vt_sparse.cpp | cut -c1-12)";
      echo "  quad: $(cat /tmp/quad.json 2>/dev/null)";
      echo "  calib: $(cat /tmp/calib_y.json 2>/dev/null | tr -d "\n " | cut -c1-80)"' || exit 3

echo "== 2. 相机快照新鲜度（R5：快照过期就不能当'当前画面'）=="
$SSH 'python3 - <<EOF
import os, time
p = "/tmp/cam.jpg"
try:
    print("  cam.jpg 年龄 = %.1f s" % (time.time() - os.path.getmtime(p)))
except OSError:
    print("  cam.jpg 不存在（本轮由管线重新生成）")
EOF'

echo "== 3. 残留渲染窗口（必须在跑之前清掉，否则会拍到冻结页面）=="
LEFT=$(ps -eo pid,cmd | grep -aE "[p]latform_host" | wc -l)
echo "  本机 platform_host 进程数 = $LEFT"
if [ "$LEFT" -gt 0 ]; then
    echo "  ⚠ 有残留：先 kill -TERM <pid>（现在装过协作退出，能收信号）"
fi

[ "$CHECK" = "1" ] && { echo "== --check 结束 =="; exit 0; }

echo "== 4. 推 quad / 标定到板子 =="
[ -f "$QUAD_LOCAL" ]  && scp -q -i "$KEY" -o StrictHostKeyChecking=no "$QUAD_LOCAL"  "$BOARD:/tmp/quad.json"  && echo "  quad  → /tmp/quad.json"
[ -f "$CALIB_LOCAL" ] && scp -q -i "$KEY" -o StrictHostKeyChecking=no "$CALIB_LOCAL" "$BOARD:/tmp/calib_y.json" && echo "  calib → /tmp/calib_y.json"

echo "== 5. 开跑（标准参数）=="
# 参数对照（改任何一条都要重跑并留 run log 作证据，见 AGENTS.md R4）：
#   场景     20 道墙、dy=140、seed 固定
#   速度     0.5×（1 s 仿真 = 2 s 墙钟）
#   相机     ffmpeg 640x480 → fps=12 → **640x480**（不再降到 320：页面 1:1，
#            墙从 3 px 高变 6 px 高，行定位抖动砍半）+ 锁曝光/白平衡
#   共识     8 帧取中位数，tol-vy 16（实测静止重复性 1σ≈8，卡 7 会剔掉一半好帧）
#   标定     data/fpga/calib_y.json（scale 0.9632 / bias 5.266）
#   决策     板端 field_cli_board（Dijkstra），障碍只来自已学到的墙
#   暂停     settle 0.65 s / 单次暂停上限 1.5 s
exec "$PY" -u core/board/tools/platform_host.py \
    --scenario "$SCENARIO" \
    --port 8498 --tcp 9006 --speed 0.5 \
    --max-seconds "$SECONDS_SIM" --render-hz 30 --field-hz 1 --follow-scale 1 \
    --tag "$TAG" --hold 2 \
    --wall-rgb 255,240,200 --field-rgb 12,14,20 --bg-rgb 0,0,0 \
    --fiducial-side 28 --fiducial-anchor band --frame-px 2 --field-every 5 \
    --board-ssh "$BOARD" --ssh-key "$KEY" \
    --board-agent /root/tools/board_agent.py --board-python python3 \
    --board-extra "--from-end --verbose --consensus-frames 8 --tol-vy 16 --settle 1.0 --pause-timeout 3.5" \
    --connect-timeout 90 \
    --calib "$CALIB_LOCAL" --calib-board /tmp/calib_y.json --quad-tol 6 \
    --viewer --viewer-pos 320,60 --viewer-size 1280x960 --viewer-margin 0 \
    --cam-vt-bin /root/tools/vt_sparse --cam-fps 12 --cam-w 640 --cam-h 480
