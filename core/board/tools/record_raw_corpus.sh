#!/bin/bash
# 录一段**原始相机录像**（不做决策、不发 @MOVE），供离线对账用。
#
#   bash core/board/tools/record_raw_corpus.sh [秒数=30]
#
# 流程（顺序按用户要求）：
#   1. **先起平台**（仿真全屏）——不要等录制；
#   2. 立刻开始录板子相机的原始帧（`raw_tee.py` 落 PPM + 每帧墙钟戳）；
#   3. 录完之后**裁掉平台出现之前 / 消失之后的部分**：
#      判据是"这一帧里能看到场地"（暗切阈值 + 场地矩形），曝光没稳定/桌面画面
#      一律算作"平台不存在"，从第一个满足的帧开始、到最后一个连续满足的帧结束；
#   4. 产出 `frames/NNNNN.ppm + stamps.tsv + truth.jsonl + meta.txt`，本地可直接
#      反复跑检测器（不用再开板子）。
#
# 产物落在 `$OUT`（默认 data/fpga/corpus/raw_<时间戳>，不进 git）。
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-30}
EVERY=${EVERY:-3}                      # 每 N 帧落一帧（3 ≈ 5 fps，够对账、体积可控）
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-data/fpga/corpus/raw_$STAMP}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
CP=${CP:-/tmp/ssh_mux_board}
PY=Env/bin/python
R() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ControlPath="$CP" -n "$HOST" "$@"; }

mkdir -p "$OUT"
ssh -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    -o ControlMaster=auto -o ControlPath="$CP" -o ControlPersist=300 -n "$HOST" 'true' 2>/dev/null

# ---- 1) 先起平台（立刻）-----------------------------------------------------
: > /tmp/raw_stream.txt
date +%s.%N > "$OUT/sim_start_epoch.txt"
echo "[原始录像] ① 起平台（全屏，不做决策）…"
(setsid sh -c "tail -f /tmp/raw_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((SECS + 40)) --fullscreen --state-log '$OUT/truth.jsonl'" \
   > "$OUT/sim.log" 2>&1 &)
sleep 5
echo '@MOVE 0' >> /tmp/raw_stream.txt        # 只发 stay：平台不动，画面干净
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[原始录像] /'

# ---- 2) 立刻开始录原始帧（不做检测、不做决策）-------------------------------
echo "[原始录像] ② 开始录 ${SECS}s 原始帧（每 $EVERY 帧落一张 PPM）…"
R "rm -rf /tmp/rawcorp; mkdir -p /tmp/rawcorp; nohup sh -c 'timeout $((SECS + 5)) \
   /root/tools/v4l2_mjpeg_pipe --dev /dev/video0 --bufs 2 --w 640 --h 480 \
   | ffmpeg -hide_banner -loglevel error -f mjpeg -i - -vf scale=640:360 \
            -pix_fmt rgb24 -f rawvideo - \
   | python3 /root/tools/raw_tee.py --dir /tmp/rawcorp --w 640 --h 360 --every $EVERY \
            --max 100000 --stamp-file /tmp/rawcorp/stamps.tsv' >/tmp/raw_tee.log 2>&1 &" \
   >/dev/null 2>&1
sleep $((SECS + 2))
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x v4l2_mjpeg_pipe 2>/dev/null; \
   pkill -INT -f "[r]aw_tee.py" 2>/dev/null; true' >/dev/null 2>&1
pkill -f "[b]ullet_sim board-control" 2>/dev/null
date +%s.%N > "$OUT/sim_end_epoch.txt"

# ---- 3) 拉回来 ---------------------------------------------------------------
echo "[原始录像] ③ 取回帧与真值 …"
rm -rf "$OUT/raw"; mkdir -p "$OUT/raw"
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o ControlPath="$CP" -o UserKnownHostsFile=/dev/null \
    -r "$HOST:/tmp/rawcorp/*" "$OUT/raw/" 2>/dev/null
ls "$OUT/raw" | head -3
echo "  帧数: $(ls "$OUT/raw"/*.ppm 2>/dev/null | wc -l)   真值行数: $(wc -l < "$OUT/truth.jsonl" 2>/dev/null)"

# ---- 4) 裁掉"平台不存在"的部分 ----------------------------------------------
"$PY" - "$OUT" <<'PY'
import json, os, shutil, sys
import numpy as np
from PIL import Image

out = sys.argv[1]
raw = os.path.join(out, "raw")
frames = sorted(f for f in os.listdir(raw) if f.endswith(".ppm")) if os.path.isdir(raw) else []
if not frames:
    print("  没有帧可裁（录制失败）"); raise SystemExit(0)

def platform_visible(path):
    """这一帧里有没有"一个亮的大矩形场地"（曝光没稳定 / 桌面画面都算没有）。"""
    a = np.asarray(Image.open(path).convert("RGB")).astype(int)
    lum = (77 * a[:, :, 0] + 150 * a[:, :, 1] + 29 * a[:, :, 2]) >> 8
    p50 = float(np.percentile(lum, 50))
    if p50 < 70:                      # 整体太暗：曝光还没起来 / 屏幕黑
        return False
    thr = max(40.0, min(120.0, 0.55 * p50))
    m = lum >= thr
    col = m.sum(0); row = m.sum(1)
    if col.max() < 0.3 * lum.shape[0]:
        return False
    xs = np.nonzero(col >= 0.5 * col.max())[0]
    ys = np.nonzero(row >= 0.6 * row.max())[0]
    if len(xs) < 2 or len(ys) < 2:
        return False
    w = xs.max() - xs.min(); h = ys.max() - ys.min()
    return w >= 0.30 * lum.shape[1] and h >= 0.30 * lum.shape[0]

flags = [platform_visible(os.path.join(raw, f)) for f in frames]
if not any(flags):
    print("  所有帧都看不到场地（相机没对着屏幕？）"); raise SystemExit(0)
first = flags.index(True)
last = len(flags) - 1 - flags[::-1].index(True)
keep = frames[first:last + 1]
print(f"  平台可见窗口: 第 {first}..{last} 帧（共 {len(frames)} 帧，保留 {len(keep)} 帧）")
if first > 0:
    print(f"  裁掉开头 {first} 帧（曝光未稳定/桌面画面）")

# 复制被保留的帧 + 对应的戳
fd = os.path.join(out, "frames"); os.makedirs(fd, exist_ok=True)
for i, f in enumerate(keep):
    shutil.copy(os.path.join(raw, f), os.path.join(fd, f"{i:05d}.ppm"))
stamp_path = os.path.join(raw, "stamps.tsv")
kept_names = set(keep)
if os.path.exists(stamp_path):
    rows = [ln for ln in open(stamp_path) if ln.split("\t")[0].strip() in
            {str(int(f.split('.')[0])) for f in kept_names}]
    with open(os.path.join(out, "stamps.tsv"), "w") as fh:
        fh.writelines(rows)
    print(f"  帧戳: {len(rows)} 行")
json.dump({"frames": len(keep), "first": first, "last": last, "every": 0},
          open(os.path.join(out, "trim.json"), "w"))
PY
echo "[原始录像] 完成 → $OUT"
ls -la "$OUT" | head -8
