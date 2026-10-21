#!/usr/bin/env bash
# ===========================================================================
# make_videos.sh —— 从一轮 run_*/ 生成四段视频（全部离线，不重跑闭环）
#
#     bash core/board/tools/make_videos.sh data/fpga/loop_logs/run_xxx [--out DIR]
#
# 产出（默认写到 <run_dir>/videos/）：
#   1. camera.mp4        相机原始画面（板端管线录的 /tmp/cam_rec.mkv；
#                        没录就退回 camera_last.jpg 单帧提示）
#   2. vision_vt.mp4     视觉处理后的结果（由 board_vt.jsonl 重绘：墙/缺口/角色）
#   3. truth.mp4         虚拟真值（仿真真值的干净渲染：场地/墙/角色/目标）
#   4. path_potential.mp4 板子自己的世界：势场（到障碍距离）+ 真实轨迹折线 + 动作
#
# 数据依赖：run 目录里的 board_vt.jsonl / local_truth.jsonl / timeline.jsonl /
# board_state.txt（闭环跑完都会留）。
# ===========================================================================
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

RUN=${1:-}
[ -z "$RUN" ] && { echo "用法：bash $0 <run_dir> [--out DIR]" >&2; exit 2; }
shift || true
OUT="$RUN/videos"
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT=$2; shift 2 ;;
        *) echo "未知参数 $1" >&2; exit 2 ;;
    esac
done
mkdir -p "$OUT"
PY="$ROOT/Env/bin/python"

echo "== 1/4 相机原始画面 =="
BOARD=root@192.168.137.242
KEY=data/fpga/board_ssh/board_key
if scp -q -o StrictHostKeyChecking=no -i "$KEY" "$BOARD:/tmp/cam_rec.mkv" "$OUT/camera_raw.mp4" 2>/dev/null; then
    #: 板端录的是 mpeg4/mp4v，浏览器能放；顺手转成 h264 更通用（失败就留原始）
    if ffmpeg -hide_banner -loglevel error -y -i "$OUT/camera_raw.mp4" \
              -c:v libx264 -pix_fmt yuv420p -crf 23 "$OUT/camera.mp4" 2>/dev/null; then
        rm -f "$OUT/camera_raw.mp4"
    else
        mv "$OUT/camera_raw.mp4" "$OUT/camera.mp4"
    fi
    echo "  → $OUT/camera.mp4（板端 /tmp/cam_rec.mkv）"
else
    echo "  ⚠ 板端没有 /tmp/cam_rec.mkv（本轮没开 --cam-record）"
    [ -f "$RUN/camera_last.jpg" ] && cp "$RUN/camera_last.jpg" "$OUT/camera_last.jpg" \
        && echo "  → 退回单帧 $OUT/camera_last.jpg"
fi

echo "== 2/4 视觉处理后的录像 =="
if [ -f "$RUN/board_vt.jsonl" ]; then
    "$PY" core/board/tools/render_vt_jsonl.py --jsonl "$RUN/board_vt.jsonl" \
        --out "$OUT/vision_vt.mp4" --fps 20 && echo "  → $OUT/vision_vt.mp4"
else
    echo "  ⚠ 缺 board_vt.jsonl"
fi

echo "== 3/4 虚拟真值视频 =="
if [ -f "$RUN/local_truth.jsonl" ]; then
    "$PY" core/board/tools/render_truth_video.py --state "$RUN/local_truth.jsonl" \
        --out "$OUT/truth.mp4" --fps 20 --every 1 && echo "  → $OUT/truth.mp4"
else
    echo "  ⚠ 缺 local_truth.jsonl"
fi

echo "== 4/4 决策路径 + 势场 =="
"$PY" core/board/tools/render_path_potential.py "$RUN" \
    --out "$OUT/path_potential.mp4" --fps 20 && echo "  → $OUT/path_potential.mp4"

echo
echo "== 产出清单 =="
ls -lh "$OUT" 2>/dev/null | awk 'NR>1 {printf "  %-24s %s\n", $9, $5}'
