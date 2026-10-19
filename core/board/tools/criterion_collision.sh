#!/bin/bash
# 判据④（120 s 内 0 碰撞）的验收跑：仿真在环，板子用相机自己开车。
#
# 与 `soak_loop.sh` 的区别：那个看的是 PL 寄存器和资源漂移（"跑久了会不会卡"），
# 这个看的是**行为**：撞没撞、离墙多近、板子这一帧到底有没有可信几何。
#
# 判定口径（写死，避免事后解释）：
#   · 碰撞 = 仿真 `--collision-log` 里的 `collision_event`（真值来自物理，不由视觉算）
#   · 有效跑 = `field_valid` 的帧占比 ≥ 60%（低了说明相机被挡/场地没锁住，这一轮不算）
#   · 最小间隙 = `clearance` 的最小值（负 = 重叠）
#
# 用法：
#   bash core/board/tools/criterion_collision.sh [秒数,默认125] [速度,默认0=场景默认]
#
# ⚠ 跑的时候**屏幕必须让仿真窗口在最前面**：相机看的是物理屏幕，被别的窗口盖住
#   就等于把相机蒙上了（那样的帧会记成 field_valid=false，跑完会被判"无效跑"）。
# ⚠ 停程序一律用 `-INT` 而不是 `-9`：被 -9 的流式 ffmpeg 会把 V4L2 卡在 busy，
#   下一轮会报"Device or resource busy"，很容易误判成相机坏了。
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-125}
SPEED=${2:-0}
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
OUT=${OUT:-/tmp/crit4}
PY=Env/bin/python
R() { ssh -n -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

mkdir -p "$OUT"
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
sleep 1
R 'pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -x vision_board_devmem 2>/dev/null; pkill -INT -f "[c]am_live.sh" 2>/dev/null; sleep 1; echo cleaned' >/dev/null 2>&1

SPEED_ARG=""
[ "$SPEED" != "0" ] && SPEED_ARG="--obstacle-speed $SPEED"
: > /tmp/crit4_stream.txt

# 记录仿真的**启动墙钟时刻**：板子诊断里也有 unix_ms，两边同源 → 直接对齐
date +%s.%N > "$OUT/sim_start_epoch.txt"
echo "[④] 起仿真（全屏、无 HUD、带真值+碰撞日志），$((SECS+40))s 上限"
(setsid sh -c "tail -f /tmp/crit4_stream.txt | $PY -u -m bullet_sim board-control --stdin \
   --nostop --duration $((SECS + 40)) --fullscreen --no-hud $SPEED_ARG \
   --state-log '$OUT/truth.jsonl' --collision-log '$OUT/collision.jsonl'" \
   > "$OUT/sim.log" 2>&1 &)
sleep 6
echo '@MOVE 0' >> /tmp/crit4_stream.txt
# 把仿真窗口提到最前：相机拍的是物理屏幕，浏览器/终端压在上面时 `field_valid`
# 会掉到 50~60%，看起来像"视觉退化"，其实屏幕上根本没有场地（截屏验证过一次）。
bash core/board/tools/focus_sim.sh 2>&1 | sed 's/^/[④] /'
sleep 1

# 板子循环的启动墙钟（和 sim_start 同一台主机同一个时钟）：
# 板子诊断里有 `t_ns`（它自己的单调时钟），两者结合就能把每一帧映射到仿真时间，
# 不依赖板子的墙钟是否与主机同步（实测板子时钟与主机差 1.5 s，且会漂）。
date +%s.%N > "$OUT/board_start_epoch.txt"
# `host_t` 由 cam_live.sh 在**启动 app 的那一刻**用 `date +%s.%N` 打点传入，
# 误差是毫秒级；这里不再额外传（早先在这里传的话，会把 ssh + 启动耗时算成"时间偏移"，
# 实测偏差 1~2 s —— 在 200 场地单位/秒的横移下就是 200~400 单位的假误差）。
# ---- 就绪门：等相机真的看到场地 ------------------------------------------
# 第 11 轮的教训：`sleep 6` 不够（pygame 切全屏更慢），开头十几秒拍的是桌面，
# 那段的真值 `obstacles=[]` 而视觉报出"两道墙"—— 数据被污染且看起来像算法错。
# 这里用**生产同一个检测器**（`wall_gap_probe`）判 3 帧里有 ≥2 帧检出带缺口的墙。
PROBE=/tmp/wg_probe
[ -x "$PROBE" ] || g++ -std=c++17 -O2 -I core/board -o "$PROBE" \
    core/board/tools/wall_gap_probe.cpp core/board/ppm_reader.cpp 2>/dev/null
echo "[④] 等场地就绪（最多 40 s）…"
READY=0
for attempt in $(seq 1 20); do
    R 'rm -f /tmp/rdy_*.ppm; ffmpeg -hide_banner -loglevel error -f v4l2 \
        -input_format mjpeg -video_size 640x480 -framerate 60 -i /dev/video0 \
        -vf scale=640:360 -pix_fmt rgb24 -frames:v 3 -y /tmp/rdy_%02d.ppm 2>/dev/null' >/dev/null 2>&1
    rm -rf "$OUT/ready"; mkdir -p "$OUT/ready"
    for i in 01 02 03; do
        scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            "$HOST:/tmp/rdy_$i.ppm" "$OUT/ready/" 2>/dev/null
    done
    if [ -x "$PROBE" ]; then
        N=$("$PROBE" --ppm-dir "$OUT/ready" 2>/dev/null | grep -o '有墙 [0-9]*' | awk '{print $2}')
        N=${N:-0}
    else
        N=0
    fi
    echo "  第 $attempt 次：3 帧里 $N 帧有墙"
    if [ "${N:-0}" -ge 2 ]; then READY=1; break; fi
    sleep 2
done
if [ "$READY" != "1" ]; then
    echo "[④] 场地起见一直没就绪（相机没对着全屏仿真窗口？）—— 这一轮不记录，退出"
    bash core/board/tools/kill_sim.sh >/dev/null 2>&1
    exit 1
fi
echo "[④] 场地就绪，开始正式记录"

echo "[④] 板子闭环开跑（相机 → PL → 决策 → @MOVE）…"
# DUMP=1 时**同时把相机原始帧存盘**：这样三段视频（相机/决策/虚拟真值）来自**同一次运行**，
# 帧号能严格对应。不存盘时行为完全不变。
DUMP_ENV=""
[ "${DUMP:-0}" = "1" ] && DUMP_ENV="DUMP_DIR=/tmp/cl_frames DUMP_EVERY=${DUMP_EVERY:-3} DUMP_MAX=100000 DUMP_STAMP=/tmp/cl_frames/stamps.tsv"
R "cd /root/tools && rm -rf /tmp/cl_frames; mkdir -p /tmp/cl_frames; rm -f /tmp/diag_crit4.jsonl; $DUMP_ENV ./cam_live.sh --frames 0 --via-pl \
   --keyboard-dev stdout --structural-gap --diag-log /tmp/diag_crit4.jsonl ${EXTRA:-}" \
   2>/dev/null | tee -a /tmp/crit4_stream.txt > /dev/null &
BPID=$!
sleep "$SECS"
kill "$BPID" 2>/dev/null
R 'pkill -INT -x vision_board_devmem 2>/dev/null; pkill -INT -x ffmpeg 2>/dev/null; pkill -INT -f "[c]am_live.sh" 2>/dev/null; sleep 1' >/dev/null 2>&1
bash core/board/tools/kill_sim.sh >/dev/null 2>&1

scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "$HOST:/tmp/diag_crit4.jsonl" "$OUT/" 2>/dev/null

"$PY" - "$OUT" "$SECS" <<'PY'
import json, sys, os, statistics as st
out, secs = sys.argv[1], float(sys.argv[2])
def load(p):
    # 被 -INT 掐断时最后一行可能是半条 JSON —— 跳过坏行，不要因此判"没跑起来"
    out = []
    try:
        f = open(p)
    except FileNotFoundError:
        return out
    with f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out
diag = load(os.path.join(out, 'diag_crit4.jsonl'))
coll = load(os.path.join(out, 'collision.jsonl'))
if not diag:
    print('[④] 没有诊断记录 —— 这一轮没有跑起来（查相机/仿真窗口）')
    sys.exit(1)
n = len(diag)
dur = (diag[-1]['t_ns'] - diag[0]['t_ns']) / 1e9 if n > 1 else secs
valid = sum(1 for r in diag if r['field_valid'])
ps = sum(1 for r in diag if r.get('player_src') == 'ps_new')
act = sum(1 for r in diag if r['action'] != 0)
wraw = [r.get('walls_raw', 0) for r in diag]
wfin = [len(r['walls']) for r in diag]
print(f'[④] 视觉：{n} 帧 / {dur:.1f}s = {n/dur:.1f} fps；field_valid {100*valid/n:.0f}%；'
      f'ps_new 角色 {100*ps/n:.0f}%；非 stay 动作 {act}')
print(f'[④] 门候选：去重前 p50 {st.median(wraw):.0f} / 最大 {max(wraw)}；'
      f'去重后 p50 {st.median(wfin):.0f} / 最大 {max(wfin)}')
if coll:
    ev = [r for r in coll if r.get('collision_event')]
    clr = [r['clearance'] for r in coll if r.get('clearance') is not None]
    fin = coll[-1].get('collision_count', len(ev))
    print(f'[④] 仿真：{len(coll)} 步 = {coll[-1]["t"]:.1f}s；碰撞事件 {len(ev)}（计数 {fin}）')
    if clr:
        s = sorted(clr)
        print(f'[④] 最小间隙 {s[0]:.1f}，p1 {s[max(0,len(s)//100)]:.1f}，中位 {s[len(s)//2]:.1f}')
    verdict = '有效跑' if valid / n >= 0.60 else '**无效跑**（field_valid 不足 60%）'
    print(f'[④] 结论：{verdict}；120 s 内碰撞 = {len(ev)}'
          f'（{"达标" if (len(ev)==0 and valid/n>=0.6) else "不达标"}）')
else:
    print('[④] 没有碰撞日志 —— 仿真没起来（查 sim.log）')
PY
