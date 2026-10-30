#!/usr/bin/env bash
# ===========================================================================
# auto_calib.sh —— 视觉侧一键自动化（v2）：体检 → 相机整定 → 四角 → 标定 → 阈值
#
#   1. 起冻结页面（**开 ArUco 标记** --fiducials）
#   2. 视觉体检（vision_health：取景/清晰度/过曝/暗区）——不过标 FAIL 但继续
#   3. 曝光/白平衡整定（auto_cam.py：扫参收敛，写 cam_profile.json）
#   4. 页面四角：auto_quad（ArUco 优先 → 先验细化兜底）→ 推送板端
#   5. 抓 N 帧 → 板端 vt_sparse → auto_calib2.py（上一版当初值重拟合 + x 仿射）
#   6. auto_tune.py：用测到的噪声反推 row_tol / tol_vy / consensus / inflate / settle
#   7. PASS/FAIL 汇总
#
# 用法：
#   bash core/board/tools/auto_calib.sh
#   bash core/board/tools/auto_calib.sh --keep-page --frames 60
#   bash core/board/tools/auto_calib.sh --page-running --skip-cam
# ===========================================================================
set -u
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
cd "$ROOT"

BOARD="root@192.168.137.242"
KEY="data/fpga/board_ssh/board_key"
PORT=8099
PAGE_PORT=8511
FRAMES=40
PY_AT=40
PAGE_RUNNING=0
KEEP_PAGE=0
SKIP_CAM=0
POSES="40"
PAGE_PID=""

while [ $# -gt 0 ]; do
    case "$1" in
        --page-running) PAGE_RUNNING=1; shift ;;
        --keep-page)    KEEP_PAGE=1; shift ;;
        --frames)       FRAMES=$2; shift 2 ;;
        --py)           PY_AT=$2; shift 2 ;;
        --skip-cam)     SKIP_CAM=1; shift ;;
        --poses)        POSES="$2"; shift 2 ;;
        -h|--help)      sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=8 $BOARD"
PY="$ROOT/Env/bin/python"
STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$ROOT/data/fpga/loop_logs/run_${STAMP}_calib"
FAILED=0
fail() { echo; echo "❌ FAIL：$*"; exit 1; }

echo "== 0. 前置检查 =="
$SSH 'true' 2>/dev/null || fail "板子 $BOARD 连不上"
[ -x "$PY" ] || fail "缺 $PY"
#: 清掉我们自己的冻结页面残留（tag=autocalib 或 页面端口 8499/8500/8501）
if [ "$PAGE_RUNNING" = "0" ]; then
"$PY" - <<'PYEOF'
import os, signal, glob
killed = []
for d in glob.glob('/proc/[0-9]*'):
    try:
        cmd = open(d + '/cmdline', 'rb').read().replace(b'\0', b' ').decode('utf-8', 'ignore')
    except OSError:
        continue
    if 'platform_host.py' not in cmd or '--scenario' not in cmd:
        continue
    if '--tag autocalib' in cmd or any(('--port %d' % p) in cmd for p in (8499, 8500, 8501)):
        pid = int(d.split('/')[-1])
        if pid == os.getpid():
            continue
        try:
            os.kill(pid, signal.SIGINT); killed.append(pid)
        except OSError:
            pass
print("  清理旧页面:", killed or "无")
PYEOF
else
    echo "  （--page-running：不动现有页面）"
fi
sleep 2
[ "$PAGE_RUNNING" = "0" ] && "$PY" - <<'PYEOF'
import os, signal, glob
for d in glob.glob('/proc/[0-9]*'):
    try:
        cmd = open(d + '/cmdline', 'rb').read().replace(b'\0', b' ').decode('utf-8', 'ignore')
    except OSError:
        continue
    if 'platform_host.py' in cmd and ('--tag autocalib' in cmd or any(('--port %d' % p) in cmd for p in (8499, 8500, 8501))):
        try:
            os.kill(int(d.split('/')[-1]), signal.SIGKILL)
        except OSError:
            pass
PYEOF
sleep 1

LEFTRUN=$(ps -eo cmd | grep -c "[p]latform_host.*--board-ssh")
[ "$LEFTRUN" -gt 0 ] && fail "还有闭环在跑（$LEFTRUN 个），先停掉再标定"

# ---- 1. 冻结页面（带 ArUco）----------------------------------------------
if [ "$PAGE_RUNNING" = "0" ] && [ "$(echo "$POSES" | tr -cd ',' | wc -c)" = "0" ]; then
    echo "== 1. 起冻结页面（py=$PY_AT，开 ArUco 标记）=="
    nohup env DISPLAY=${DISPLAY:-:0} "$PY" -u core/board/tools/platform_host.py \
        --scenario configs/20_walls_alt_24_dy140.json \
        --port "$PAGE_PORT" --tcp 9009 --speed 0.5 --max-seconds 3600 \
        --render-hz 30 --field-hz 0 --follow-scale 1 --tag autocalib --hold 9999 \
        --wall-rgb 255,240,200 --field-rgb 12,14,20 --bg-rgb 0,0,0 \
        --fiducials --fiducial-side 48 --fiducial-anchor band \
        --frame-px 2 --field-every 5 \
        --viewer --viewer-fullscreen --viewer-pos 320,60 --viewer-size 1280x960 \
        --viewer-margin 0 --no-start-camera-pipe > /tmp/autocalib_page.log 2>&1 &
    PAGE_PID=$!
    sleep 6
    grep -q "查看器已起" /tmp/autocalib_page.log || fail "页面没起来，见 /tmp/autocalib_page.log"
    echo "  页面 pid=$PAGE_PID，等画面稳定…"
    #: 等页面真的渲染出来（取景占比 > 0.5）再往下走，否则会拍到上一屏
    start_preview >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
        OCC=$("$PY" core/board/tools/vision_health.py --url "http://192.168.137.242:${PORT:-8099}/stream.mjpg" --json 2>/dev/null \
              | "$PY" -c "import sys,json;print(json.load(sys.stdin)['metrics'] .get('frame_occ',0))" 2>/dev/null || echo 0)
        case "$OCC" in 0.5*|0.6*|0.7*|0.8*|0.9*|1.0*) echo "  画面已稳定（取景 ${OCC}）"; break;; esac
        sleep 1
    done
elif [ "$(echo "$POSES" | tr -cd ',' | wc -c)" != "0" ]; then
    echo "== 1. 多姿态标定：页面由第 5 步按 pose 逐个启动 =="
else
    echo "== 1. 复用屏幕上已有的冻结页面 =="
fi

pick_port() {
    # 找一个板端可用的预览端口（8099 被占就往后找）
    for p in 8099 8100 8101 8102 8103 8104; do
        if $SSH "python3 -c \"import socket,sys; s=socket.socket(); s.settimeout(0.3); sys.exit(0 if (s.connect_ex(('127.0.0.1',$p))!=0) else 1)\"" 2>/dev/null; then
            echo "$p"; return 0
        fi
    done
    echo 8099
}

wait_stream() {   # $1=port，等到真的能出帧（最多 15 s）
    "$PY" - "$1" <<'EOF'
import sys, time, urllib.request
port = sys.argv[1]
url = "http://192.168.137.242:%s/stream.mjpg" % port
t0 = time.time()
while time.time() - t0 < 15:
    try:
        f = urllib.request.urlopen(url, timeout=3)
        if f.read(1024):
            print("  stream ok on %s" % port); sys.exit(0)
    except Exception:
        time.sleep(1)
sys.exit(1)
EOF
}

start_preview() {
    stop_preview >/dev/null 2>&1
    bash core/board/tools/cam_lock.sh release >/dev/null 2>&1
    sleep 1
    for cand in $(pick_port) 8100 8101 8102 8103 8104 8105; do
        PORT=$cand
        $SSH 'for p in /proc/[0-9]*; do c=$(tr "\0" " " < $p/cmdline 2>/dev/null); case "$c" in *"mjpeg_server.py '"$PORT"'"*) kill ${p#/proc/} 2>/dev/null;; esac; done; sleep 1' 2>/dev/null
        $SSH 'cd /root/tools && (setsid nohup python3 mjpeg_server.py '"$PORT"' >/tmp/mjpeg.log 2>&1 </dev/null &); sleep 4; tail -1 /tmp/mjpeg.log' 2>/dev/null
        if wait_stream "$PORT"; then return 0; fi
    done
    echo "  ⚠ 预览起不来（端口都被占？）"
    return 1
}

stop_preview() {
    $SSH 'for p in /proc/[0-9]*; do c=$(tr "\0" " " < $p/cmdline 2>/dev/null); case "$c" in *"mjpeg_server.py"*) kill ${p#/proc/} 2>/dev/null;; esac; done; sleep 1' 2>/dev/null
}

# ---- 2. 曝光/白平衡 ------------------------------------------------------
if [ "$SKIP_CAM" = "0" ]; then
    echo "== 2. 曝光/白平衡整定（相机独占）=="
    stop_preview >/dev/null 2>&1
    bash core/board/tools/cam_lock.sh release >/dev/null 2>&1
    sleep 1
    if "$PY" core/board/tools/auto_cam.py 2>&1 | sed 's/^/  /'; then
        echo "  相机整定 PASS"
    else
        echo "  ⚠ 相机整定未过"; FAILED=1
    fi
else
    echo "== 2. 跳过曝光/白平衡（--skip-cam）=="
fi

echo "== 3. 视觉体检（整定后复测）=="
start_preview
sleep 2
if ! "$PY" core/board/tools/vision_health.py --url "http://192.168.137.242:$PORT/stream.mjpg" | sed 's/^/  /'; then
    #: 体检是**提示性**的：它不过不代表标定不能用（标定自己有 rmse 验收）。
    echo "  ⚠ 体检有告警（对焦/反光），不阻断；标定仍会用 rmse 自证"
fi

# ---- 4. 页面四角 ---------------------------------------------------------
echo "== 4. 页面四角 =="
QUAD_OUT=$("$PY" core/board/tools/auto_quad.py --url "http://192.168.137.242:$PORT/stream.mjpg" \
    --board "$BOARD" --key "$KEY" --push --in-scale 1.0 --marker-side 48 2>&1)
echo "$QUAD_OUT" | sed 's/^/  /'
if ! echo "$QUAD_OUT" | grep -q "已推送"; then
    stop_preview
    fail "页面四角没算出来"
fi
stop_preview

# ---- 5. 抓帧（多姿态）+ 标定拟合 -----------------------------------------
echo "== 5. 抓帧（poses=$POSES）+ 板端 vt_sparse + 重拟合 =="
mkdir -p "$RUN"
RUNS=()
FIRST_POSE=$(echo "$POSES" | cut -d, -f1)

start_pose_page() {   # $1 = py
    local py="$1"
    "$PY" - "$ROOT/configs/20_walls_alt_24_dy140.json" "$ROOT/data/fpga/pose_${py}.json" "$py" <<'EOF'
import json, sys
src, dst, py = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = json.load(open(src))
d["player_x"] = 320.0
d["player_y"] = py
d.setdefault("meta", {})["spawn"] = [320.0, py]
json.dump(d, open(dst, "w"), ensure_ascii=False)
print("pose 场景 ->", dst)
EOF
    nohup env DISPLAY=${DISPLAY:-:0} "$PY" -u core/board/tools/platform_host.py \
        --scenario "data/fpga/pose_${py}.json" \
        --port "$PAGE_PORT" --tcp 9009 --speed 0.5 --max-seconds 3600 \
        --render-hz 30 --field-hz 0 --follow-scale 1 --tag "pose${py}" --hold 9999 \
        --wall-rgb 255,240,200 --field-rgb 12,14,20 --bg-rgb 0,0,0 \
        --fiducials --fiducial-side 48 --fiducial-anchor band \
        --frame-px 2 --field-every 5 \
        --viewer --viewer-fullscreen --viewer-pos 320,60 --viewer-size 1280x960 \
        --viewer-margin 0 --no-start-camera-pipe > /tmp/pose_${py}.log 2>&1 &
    POSE_PID=$!
    sleep 9
}

stop_pose_page() {
    [ -n "${POSE_PID:-}" ] && kill -INT "$POSE_PID" 2>/dev/null && sleep 1
    [ -n "${POSE_PID:-}" ] && kill -TERM "$POSE_PID" 2>/dev/null
    POSE_PID=""
}

for PY_NOW in $(echo "$POSES" | tr ',' ' '); do
    echo "  -- pose py=$PY_NOW --"
    if [ "$(echo "$POSES" | tr -cd ',' | wc -c)" != "0" ] || [ "$PAGE_RUNNING" = "0" ]; then
        start_pose_page "$PY_NOW"
    fi
    POSE_RUN="$ROOT/data/fpga/loop_logs/run_${STAMP}_calib_py${PY_NOW}"
    mkdir -p "$POSE_RUN"
    $SSH "pkill -x ffmpeg 2>/dev/null; pkill -x vt_sparse 2>/dev/null; sleep 1;
          rm -f /tmp/calib.rgb;
          ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 640x480 \
            -i /dev/video0 -frames:v $FRAMES -f rawvideo -pix_fmt rgb24 -y /tmp/calib.rgb 2>/dev/null;
          echo bytes=\$(stat -c %s /tmp/calib.rgb)" 2>/dev/null | tail -1
    $SSH "rm -f /tmp/calib.jsonl; /root/tools/vt_sparse --w 640 --h 480 \
          --frame-roi 0 0 640 480 --quad-file /tmp/quad.json --jsonl /tmp/calib.jsonl \
          < /tmp/calib.rgb > /dev/null; echo lines=\$(wc -l < /tmp/calib.jsonl)" 2>/dev/null | tail -1
    scp -q -i "$KEY" -o StrictHostKeyChecking=no "$BOARD:/tmp/calib.jsonl" "$POSE_RUN/board_vt.jsonl" \
        || fail "取回标定帧失败（py=$PY_NOW）"
    "$PY" - "$POSE_RUN" "$PY_NOW" <<'EOF'
import json, sys
from pathlib import Path
run, py_at = Path(sys.argv[1]), float(sys.argv[2])
rows = [json.loads(l) for l in (run / "board_vt.jsonl").read_text().splitlines() if l.strip()]
with open(run / "board_events.jsonl", "w") as f:
    for r in rows:
        f.write(json.dumps({"ev": "vt_tick", "frame": r["frame"], "py": py_at, "py_used": py_at}) + "\n")
print("  帧数 %d，有墙 %d" % (len(rows), sum(1 for r in rows if r.get("walls"))))
EOF
    RUNS+=("$POSE_RUN")
    if [ "$PAGE_RUNNING" = "0" ]; then stop_pose_page; fi
done
PAGE_PID=""
#: 多姿态有效性检查：冻结页面如果不跟随视口移动，三个 pose 拍到的是同一画面。
if [ "${#RUNS[@]}" -gt 1 ]; then
    SAME=$("$PY" - "${RUNS[@]}" <<'EOF'
import json, sys
sets = []
for d in sys.argv[1:]:
    ys = set()
    for l in open(d + "/board_vt.jsonl"):
        r = json.loads(l)
        for w in (r.get("walls") or []):
            ys.add(round(w["y"] / 5) * 5)
    sets.append(ys)
same = all(s == sets[0] for s in sets)
print("1" if same else "0")
EOF
)
    if [ "$SAME" = "1" ]; then
        echo "  ⚠ 三个 pose 拍到的墙行完全一样 —— 冻结页面**不跟随视口**，多姿态拿不到新信息"
        echo "     退回单姿态（只用第一个），标定仍然有效"
        RUNS=("${RUNS[0]}")
    fi
fi
if "$PY" core/board/tools/auto_calib2.py "${RUNS[@]}" --scenario configs/20_walls_alt_24_dy140.json \
        --py "$FIRST_POSE" --push 2>&1 | sed 's/^/  /'; then
    echo "  标定 PASS"
else
    echo "  ⚠ 标定未过"; FAILED=1
fi

# ---- 6. 阈值反推 ---------------------------------------------------------
echo "== 6. 用噪声反推决策阈值 =="
if "$PY" core/board/tools/auto_tune.py "${RUNS[@]}" 2>&1 | sed 's/^/  /'; then
    echo "  阈值 PASS"
else
    echo "  ⚠ 阈值推导未过"; FAILED=1
fi

# ---- 7. 收尾 -------------------------------------------------------------
if [ "$KEEP_PAGE" = "0" ] && [ -n "$PAGE_PID" ]; then
    kill -INT "$PAGE_PID" 2>/dev/null; sleep 1
    kill -TERM "$PAGE_PID" 2>/dev/null
    echo "  已关闭冻结页面"
fi
stop_preview

# ---- 7b. 固化到板端 /root/vision_state（重启后 rc.local 会自动恢复）----
if [ "$FAILED" -eq 0 ]; then
    $SSH 'mkdir -p /root/vision_state' 2>/dev/null
    scp -q -i "$KEY" -o StrictHostKeyChecking=no data/fpga/quad.json data/fpga/calib_y.json \
        "$BOARD:/root/vision_state/" 2>/dev/null && echo "  已固化到 /root/vision_state（重启自动恢复）"
fi

echo
if [ "$FAILED" -eq 0 ]; then
    echo "✅ PASS：quad + 标定 + 阈值 全部更新"
    echo "   产物：data/fpga/quad.json、data/fpga/calib_y.json、data/fpga/params.json、data/fpga/cam_profile.json"
    echo "   下一步：bash core/board/tools/loop_standard.sh --tag <名字>"
    exit 0
fi
echo "❌ 有步骤没通过（见上面标 FAIL 的地方），不要拿这版去跑闭环"
exit 1
