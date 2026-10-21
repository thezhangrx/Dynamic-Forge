#!/usr/bin/env bash
# ===========================================================================
# calib_static.sh —— **静态标定**：固定画面 → 多帧 → 拟合 y 换算系数 + 横向偏差
#
# 为什么要有独立的一步（而不是跑完一轮顺手标）
# ------------------------------------------
# 整轮数据里混着"贴边墙、远景墙（畸变最大处）"和运动帧，拟合 rmse 8~12 px；
# 固定画面 40 帧 + 真值的标定能到 **rmse 0.1 px**。以前每轮结束都用整轮数据
# 覆盖标定，导致下一轮 |Δy| 从 1.4 跳到 8.8（实测 s1b/s1c）。所以：
#   * 标定 = 这个脚本（只在相机/窗口动过之后跑一次）
#   * 闭环 = loop_standard.sh（只**用**标定，不再改它）
#
# 它做什么
# --------
#  1. 在板上抓 40 帧 640×480 原始帧（相机独占；先停预览/VT/闭环）
#  2. 用板端 vt_sparse 跑一遍，得到每帧的墙行 vy 与缺口 gl/gr
#  3. 用**主机真值**（configs 的 walls_y/gaps_x/gap_widths）拟合：
#        world = (py-240) + scale*vy + bias        （y 换算）
#        x_bias = median(VT 缺口边界 - 真值边界)   （横向固定偏差）
#  4. 写 data/fpga/calib_y.json + 推到板端 /tmp/calib_y.json
#
# 前提：页面上有 ArUco 标记吗？**不要求** —— 这个脚本只做 y/x 拟合，quad 用现有的
#       （quad 变了要先用 push_quad.py 重推，脚本会检查 quad 是否为空）。
#
# 用法：
#     bash core/board/tools/calib_static.sh                 # 用当前屏幕上那张页面
#     bash core/board/tools/calib_static.sh --frames 60
#     bash core/board/tools/calib_static.sh --py 40         # 页面当前停在哪个 py
# ===========================================================================
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

BOARD="root@192.168.137.242"
KEY="data/fpga/board_ssh/board_key"
SCENARIO="configs/20_walls_alt_24_dy140.json"
FRAMES=40
PY_AT=40
SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=8 $BOARD"

while [ $# -gt 0 ]; do
    case "$1" in
        --frames) FRAMES=$2; shift 2 ;;
        --py)     PY_AT=$2; shift 2 ;;
        -h|--help) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

echo "== 1. 让相机独占（停预览 / VT / 闭环） =="
$SSH 'pkill -f "[m]jpeg_server.py"; pkill -x vt_sparse; pkill -x vt_board; pkill -x ffmpeg; sleep 1; echo ok' >/dev/null 2>&1
LEFT=$(ps -eo cmd | grep -c "[p]latform_host.*--board-ssh")
echo "  正在跑的闭环数 = $LEFT（标定期间不能有闭环抢相机）"
[ "$LEFT" -gt 0 ] && { echo "  ⚠ 先停掉闭环再标定" >&2; exit 3; }
PAGE=$(ps -eo cmd | grep -c "[p]latform_host.*--scenario")
echo "  静态页面显示进程 = $PAGE（标定必须有画面，0 = 相机拍不到东西）"
[ "$PAGE" -eq 0 ] && { echo "  ⚠ 先起一个页面显示（--speed 0 --viewer --viewer-fullscreen）" >&2; exit 3; }

echo "== 2. 检查 quad =="
QB=$($SSH 'cat /tmp/quad.json 2>/dev/null' 2>/dev/null | tr -d '\r')
echo "  /tmp/quad.json = ${QB:-（空！）}"
[ -z "$QB" ] && { echo "  ⚠ 没有 quad：先跑 push_quad.py（需要页面上有 ArUco）" >&2; exit 3; }

echo "== 3. 锁曝光/白平衡后抓 $FRAMES 帧 640x480 =="
#: 与闭环管线用同一套相机参数，否则"标定时是自动曝光、跑的时候也是自动曝光"，
#: 两边的亮度分布都对不上（实测没锁时 40 帧里只有 17 帧能测出墙）。
#: 曝光保持自动（与闭环一致；固定曝光的坑见 platform_host.py 注释）; then         v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1                  --set-ctrl=exposure_absolute=200                  --set-ctrl=white_balance_temperature_auto=0                  --set-ctrl=white_balance_temperature=4000                  --set-ctrl=gain=4 2>/dev/null || true; fi' >/dev/null 2>&1
$SSH "rm -f /tmp/calib.rgb; ffmpeg -hide_banner -loglevel error -f v4l2 \
      -input_format mjpeg -video_size 640x480 -i /dev/video0 \
      -frames:v $FRAMES -f rawvideo -pix_fmt rgb24 -y /tmp/calib.rgb 2>/dev/null;
      echo bytes=\$(stat -c %s /tmp/calib.rgb)" 2>/dev/null | tail -1

echo "== 4. 板端 vt_sparse 跑一遍 =="
$SSH "rm -f /tmp/calib.jsonl; /root/tools/vt_sparse --w 640 --h 480 \
      --frame-roi 0 0 640 480 --quad-file /tmp/quad.json \
      --jsonl /tmp/calib.jsonl < /tmp/calib.rgb > /dev/null; \
      echo lines=\$(wc -l < /tmp/calib.jsonl)" 2>/dev/null | tail -1

echo "== 5. 拟合（用主机真值，一次性） =="
TMP=$(mktemp -d)
scp -q -o StrictHostKeyChecking=no -i "$KEY" "$BOARD:/tmp/calib.jsonl" "$TMP/board_vt.jsonl"
Env/bin/python - "$TMP" "$PY_AT" <<'EOF'
import json, sys
from pathlib import Path
tmp, py_at = Path(sys.argv[1]), float(sys.argv[2])
rows = [json.loads(l) for l in (tmp / "board_vt.jsonl").read_text().splitlines() if l.strip()]
with open(tmp / "board_events.jsonl", "w") as f:
    for r in rows:
        f.write(json.dumps({"ev": "vt_tick", "frame": r["frame"],
                            "py": py_at, "py_used": py_at}) + "\n")
print(f"  帧数 {len(rows)}  有墙帧 {sum(1 for r in rows if r.get('walls'))}  "
      f"有角色帧 {sum(1 for r in rows if r.get('player'))}")
EOF
QUAD=$($SSH 'cat /tmp/quad.json' 2>/dev/null | tr -d '\r' | tr -s ' ' | tr ' ' ',' )
Env/bin/python core/board/tools/calib_y.py "$TMP" --scenario "$SCENARIO" \
    --out data/fpga/calib_y.json --apply /tmp/calib_static_board.json \
    --quad "$QUAD" 2>&1 | tail -2
rm -rf "$TMP"

echo "== 6. 推到板端 =="
scp -q -o StrictHostKeyChecking=no -i "$KEY" data/fpga/calib_y.json "$BOARD:/tmp/calib_y.json" \
    && echo "  → /tmp/calib_y.json"
Env/bin/python -c "
import json
c=json.load(open('data/fpga/calib_y.json'))
print('  scale=%.5f bias=%.3f x_bias=%.2f rmse=%.2f samples=%s'
      % (c['scale'], c['bias'], c.get('x_bias',0), c.get('rmse',-1), c.get('samples')))
print('  验收线：rmse ≤ 1.0 px；超过说明页面没对准/画面糊了，别拿去跑闭环')
"
