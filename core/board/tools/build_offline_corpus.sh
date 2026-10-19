#!/bin/bash
# 建一个**离线动态语料**：仿真在跑 + 板子相机抓帧 + 每帧带墙钟 + 真值带墙钟。
#
# 为什么值得单独做：在板上做一次实验要 90 秒，而且"角色检测准不准"这种问题需要
# 反复改判据、反复看同一批帧。有了这个语料，之后所有检测器实验都在本地几秒一轮，
# 而且**同一批帧**可以横向比较（改了判据再跑一遍，差异只来自判据）。
#
# 产物（本地 $OUT）：
#   ppm/NNNNNN.ppm        板子相机真正喂给程序的帧（640x360）
#   stamps.tsv            帧序号 ↔ 墙钟（raw_tee 写入）
#   truth.jsonl           仿真真值（每帧一行，含 unix 墙钟）
#   truth_frames.json     {"frame":N,"x":..,"y":..}（场地单位，y 朝上）—— 已按墙钟配对
#   meta.txt              本轮参数（种子、时长、速度）
#
# 用法：
#   bash core/board/tools/build_offline_corpus.sh [秒数=20] [速度=0=场景默认]
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-20}
SPEED=${2:-0}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
OUT=${OUT:-/tmp/offline_corpus}
PY=Env/bin/python
R() { ssh -n -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

rm -rf "$OUT"; mkdir -p "$OUT/ppm"
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
sleep 1
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -f "[c]am_live" 2>/dev/null; sleep 1' >/dev/null 2>&1

SPEED_ARG=""
[ "$SPEED" != "0" ] && SPEED_ARG="--obstacle-speed $SPEED"
: > /tmp/oc_stream.txt
(setsid sh -c "tail -f /tmp/oc_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((SECS + 60)) --fullscreen --no-hud $SPEED_ARG \
   --state-log '$OUT/truth.jsonl'" > "$OUT/sim.log" 2>&1 &)
sleep 6
echo '@MOVE 0' >> /tmp/oc_stream.txt
# 窗口必须在最前：相机拍的是物理屏幕，被浏览器盖住时"就绪检查"会一直 0/3（实测过）
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[语料] /'

# `LOOP=board`：让**板子自己闭环开车**（这样语料里的帧和真实闭环完全一致，
# 而不是"脚本化动作"那种偏简单的场景）。默认 `LOOP=script`（脚本化运动）。
LOOP=${LOOP:-script}
if [ "$LOOP" = "board" ]; then
    echo "[语料] LOOP=board：板子闭环驱动（帧与真实闭环同分布）"
else
# 用一个**脚本化的动作**让平台动起来（左右往复 + 偶尔上），这样语料里既有横向运动
# 也有纵向运动，检测器在两种情况下都能被检验。动作用主机侧脚本发，不依赖板子。
(setsid sh -c "for i in \$(seq 1 $((SECS * 2))); do
     case \$((i % 6)) in
       0) echo '@MOVE 2' >> /tmp/oc_stream.txt;;
       1|2) echo '@MOVE 0' >> /tmp/oc_stream.txt
# 窗口必须在最前：相机拍的是物理屏幕，被浏览器盖住时"就绪检查"会一直 0/3（实测过）
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[语料] /';;
       3) echo '@MOVE 1' >> /tmp/oc_stream.txt;;
       4|5) echo '@MOVE 0' >> /tmp/oc_stream.txt
# 窗口必须在最前：相机拍的是物理屏幕，被浏览器盖住时"就绪检查"会一直 0/3（实测过）
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[语料] /';;
     esac
     sleep 0.5
   done" > /dev/null 2>&1 &)
fi

# 就绪门：抓 3 帧判"看得到墙和标记"
READY=0
for attempt in $(seq 1 20); do
    R 'rm -f /tmp/rdy_*.ppm; ffmpeg -hide_banner -loglevel error -f v4l2 \
        -input_format mjpeg -video_size 640x480 -framerate 60 -i /dev/video0 \
        -vf scale=640:360 -pix_fmt rgb24 -frames:v 3 -y /tmp/rdy_%02d.ppm 2>/dev/null' >/dev/null 2>&1
    rm -rf /tmp/oc_ready; mkdir -p /tmp/oc_ready
    for i in 01 02 03; do
        scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            "$HOST:/tmp/rdy_$i.ppm" /tmp/oc_ready/ 2>/dev/null
    done
    N=$(g++ -std=c++17 -O2 -I core/board -o /tmp/wg_probe core/board/tools/wall_gap_probe.cpp \
            core/board/ppm_reader.cpp 2>/dev/null && \
        /tmp/wg_probe --ppm-dir /tmp/oc_ready 2>/dev/null | grep -o '有墙 [0-9]*' | awk '{print $2}')
    echo "  就绪检查第 $attempt 次：$N/3 帧有墙"
    if [ "${N:-0}" -ge 2 ]; then READY=1; break; fi
    sleep 2
done
if [ "$READY" != "1" ]; then
    echo "[语料] 场地没就绪，放弃（相机没对着全屏仿真窗口？）"
    bash core/board/tools/kill_sim.sh >/dev/null 2>&1
    exit 1
fi

echo "[语料] 采集中（${SECS}s，每帧存、最多 400 张）…"
if [ "$LOOP" = "board" ]; then
    # 板子闭环：它的 @MOVE 直接进仿真 stdin（和验收跑同一条链路）
    R "cd /root/tools && rm -rf /tmp/oc_dump /tmp/oc_stamps.tsv; \
       DUMP_DIR=/tmp/oc_dump DUMP_EVERY=2 DUMP_MAX=400 DUMP_STAMP=/tmp/oc_stamps.tsv \
       timeout $((SECS + 10)) ./cam_live.sh --frames 0 --via-pl --keyboard-dev stdout \
       --structural-gap" 2>/dev/null | tee -a /tmp/oc_stream.txt > /dev/null
    sleep 1
else
    R "cd /root/tools && rm -rf /tmp/oc_dump /tmp/oc_stamps.tsv; \
       DUMP_DIR=/tmp/oc_dump DUMP_EVERY=2 DUMP_MAX=400 DUMP_STAMP=/tmp/oc_stamps.tsv \
       timeout $((SECS + 10)) ./cam_live.sh --frames 0 --via-pl --keyboard-dev discard \
       --structural-gap >/dev/null 2>&1; ls /tmp/oc_dump | wc -l" 2>/dev/null | tail -1
fi

scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "$HOST:/tmp/oc_dump/*.ppm" "$OUT/ppm/" 2>/dev/null
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "$HOST:/tmp/oc_stamps.tsv" "$OUT/stamps.tsv" 2>/dev/null
bash core/board/tools/kill_sim.sh >/dev/null 2>&1

"$PY" - "$OUT" <<'PY'
import json, sys, os
out = sys.argv[1]
truth = [json.loads(l) for l in open(os.path.join(out, 'truth.jsonl')) if l.strip()]
ut = [r['unix'] for r in truth]
import bisect
def at(u):
    i = bisect.bisect_left(ut, u)
    return truth[min(i, len(truth) - 1)]
frames = sorted(f for f in os.listdir(os.path.join(out, 'ppm')) if f.endswith('.ppm'))
stamps = {}
try:
    for line in open(os.path.join(out, 'stamps.tsv')):
        a, b = line.split()
        stamps[int(a)] = float(b)
except FileNotFoundError:
    pass
pairs = 0
with open(os.path.join(out, 'truth_frames.json'), 'w') as fh:
    for idx, name in enumerate(frames):
        key = int(name.split('.')[0])
        ts = stamps.get(key)
        if ts is None:
            # 没时间戳就退化成"按顺序均匀分布"（不精确，仅作兜底）
            ts = ut[0] + (ut[-1] - ut[0]) * idx / max(1, len(frames) - 1)
        t = at(ts)['player']
        fh.write(json.dumps({"frame": idx, "x": round(t[0], 2), "y": round(t[1], 2)},
                            separators=(",", ":")) + "\n")
        pairs += 1
if not stamps:
    print('[语料] ⚠ 没有 stamps.tsv（raw_tee 没写），真值只能按顺序均匀分布——不精确！')
print(f'[语料] 帧 {len(frames)}，真值配对 {pairs} → {os.path.join(out, "truth_frames.json")}')
PY
cat > "$OUT/meta.txt" <<EOF
secs=$SECS speed=$SPEED
EOF
echo "[语料] 完成：$OUT"
echo "[语料] 用法： /tmp/pp --ppm-dir $OUT/ppm --truth-jsonl $OUT/truth_frames.json --tol 20"
