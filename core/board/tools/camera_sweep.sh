#!/bin/bash
# 相机噪声巡检：一次性把"为什么拍出来全是噪点"量成数，而不是靠眼睛看。
#
# 背景：板子相机拍仿真画面时，画面里到处是紫/绿的彩色斑点，物体边缘被撕碎，
# 二值化之后墙 fill 只有 0.25（本该 ≈1）。可能的成因有好几个（增益、曝光时间、
# 锐化、色彩增强、50Hz 防闪烁、MJPG 压缩、屏幕太暗），**必须一个个量**。
#
# 本脚本对每种相机配置各拍几帧（同一静止画面），然后报告三个数：
#   σ_L     亮度噪声：画面内 12x12 小块里 L 的标准差的中位数（越小越好）
#   σ_色    色度噪声：同块内 (R-B) 的标准差的中位数（越小越好）
#   SNR     (亮度 99 分位 - 中位) / σ_L —— 物体和场地的反差相当于几倍噪声
#           **这个才是"能不能识别"的真正指标**：SNR < 3 时任何阈值分割都会碎。
#
# 用法：
#   bash core/board/tools/camera_sweep.sh            # 跑默认配置表
#   CFGS="60 30 15" bash core/board/tools/camera_sweep.sh
#
# ⚠ 跑之前仿真窗口要在屏幕上（画面得有内容），脚本会自己起一个静止的仿真。
# ⚠ 每个配置都会改相机控制项，脚本结束会把控制项恢复成 cam_live 的默认值。
set -u
cd "$(dirname "$0")/../../.." || exit 1
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
OUT=${OUT:-/tmp/cam_sweep}
PY=Env/bin/python
# ⚠ `-n` 是**必需的**：不带时 ssh 会把本地 stdin 读走，而这里的循环是用
#   `echo "$CONFIGS" | while read ...` 喂的 —— ssh 一读，剩下的配置就全没了，
#   表现是"巡检只跑第一个配置"，而且不报错。
R() { ssh -n -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

mkdir -p "$OUT"
# 放开相机：上一轮如果没退干净，`/dev/video0` 会 "Device or resource busy"，
# 而 ffmpeg 的报错这里会被吞掉 —— 表现成"一帧都没取到"，很容易误判成相机坏了。
# ⚠ pkill 的模式里**必须带方括号**：不带的话模式会匹配到执行 pkill 的那个 shell
#   自己的命令行，先把 shell 杀了（自伤），目标进程反而留着。这个坑踩过好几次。
R 'pkill -f "[c]am_live.sh"; pkill -f "[v]ision_board_devmem"; pkill -f "[f]fmpeg"; sleep 1' >/dev/null 2>&1
bash core/board/tools/kill_sim.sh >/dev/null 2>&1
rm -f /tmp/cs_mv.txt; touch /tmp/cs_mv.txt
(setsid sh -c "$PY -u -m bullet_sim board-control --stdin --nostop --duration 900 \
    --fullscreen --no-hud < /tmp/cs_mv.txt" > /tmp/cs_sim.log 2>&1 &)
sleep 4
echo '@MOVE 0' >> /tmp/cs_mv.txt
sleep 1

# 配置表：名字|格式|尺寸|帧率|相机控制项
#
# ⚠ 这台相机**只支持离散的几档**（`v4l2-ctl --list-formats-ext`）：
#     MJPG 640x480@60    MJPG 1280x720@60    YUYV 640x480@30    YUYV 1280x720@10
#   没有 15/20/24 fps，MJPG 也没有 30 fps —— 想要 30 帧只能走**未压缩的 YUYV**。
#   这一点很关键：曝光时间上限 = 1/帧率，帧率越高能用的曝光越短、只能靠增益，
#   噪声就是这么来的。所以"能不能把帧率降下来"直接决定噪声大小。
CONFIGS=${CFGS:-"
基线 MJPG60|mjpeg|640x480|60|saturation=100 brightness=70 sharpness=2
MJPG60+降锐化降色|mjpeg|640x480|60|saturation=60 brightness=70 sharpness=0
YUYV30（未压缩）|yuyv422|640x480|30|saturation=100 brightness=70 sharpness=2
YUYV30+降锐化降色|yuyv422|640x480|30|saturation=60 brightness=70 sharpness=0
YUYV30+60Hz防闪|yuyv422|640x480|30|saturation=60 brightness=70 sharpness=0 power_line_frequency=2
MJPG60+手动曝光160|mjpeg|640x480|60|exposure_auto=1 exposure_absolute=160
YUYV30+手动曝光320|yuyv422|640x480|30|exposure_auto=1 exposure_absolute=320
"}

i=0
echo "$CONFIGS" | while IFS='|' read -r name fmt size fps ctrls; do
    [ -z "${name:-}" ] && continue
    i=$((i+1))
    safe=$(echo "$name" | tr -c 'a-zA-Z0-9' '_')
    echo "[巡检] $name  ($fmt $size @$fps, $ctrls)"
    # 手动曝光要先切 exposure_auto=1 才能写 exposure_absolute，所以顺序按控制项给的顺序执行
    out=$(R "for c in $ctrls; do v4l2-ctl -d /dev/video0 -c \$c; done 2>&1; \
       rm -rf /tmp/sw_$safe; mkdir -p /tmp/sw_$safe; \
       ffmpeg -hide_banner -loglevel error -f v4l2 -framerate $fps -input_format $fmt \
         -video_size $size -i /dev/video0 -frames:v 6 -y /tmp/sw_$safe/%02d.ppm" 2>&1)
    [ -n "$out" ] && echo "    ↳ $out" | head -3
    # 恢复自动曝光，免得下一个配置被上一个的手动设置带偏
    R 'v4l2-ctl -d /dev/video0 -c exposure_auto=3 >/dev/null 2>&1' >/dev/null 2>&1
    scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        "$HOST:/tmp/sw_$safe/*.ppm" "$OUT/" 2>/dev/null
    # 只挪"这一轮刚下回来的"（纯数字文件名）—— 用 `*.ppm` 会把上一轮已经改好名的
    # 文件再挪一次，最后所有配置都被贴上同一个前缀，表格里只剩一行。
    for f in "$OUT"/[0-9][0-9].ppm; do
        [ -e "$f" ] || continue
        # 分隔符用 `#` 而不是 `__`：配置名里有连续下划线时 `__` 会把前缀切空，
        # 结果所有配置挤成一行（这个坑一眼看不出来，表格只是"少了几行"）。
        mv "$f" "$OUT/${safe}#$(basename "$f")"
    done
done

R 'v4l2-ctl -d /dev/video0 -c saturation=100 -c brightness=70 -c sharpness=2 \
    -c power_line_frequency=1 -c exposure_auto=3 >/dev/null 2>&1' >/dev/null 2>&1
bash core/board/tools/kill_sim.sh >/dev/null 2>&1

"$PY" - "$OUT" <<'PY'
import sys, glob, os
import numpy as np
from PIL import Image

d = sys.argv[1]
files = sorted(glob.glob(os.path.join(d, '*.ppm')))
if not files:
    print('没有取到帧'); sys.exit(1)

def metrics(path):
    a = np.asarray(Image.open(path)).astype(np.float32)
    L = (77*a[:,:,0] + 150*a[:,:,1] + 29*a[:,:,2]) / 256.0
    C = a[:,:,0] - a[:,:,2]
    h, w = L.shape
    b = 12
    hh, ww = h//b*b, w//b*b
    bl = L[:hh,:ww].reshape(hh//b, b, ww//b, b).transpose(0,2,1,3).reshape(-1, b*b)
    bc = C[:hh,:ww].reshape(hh//b, b, ww//b, b).transpose(0,2,1,3).reshape(-1, b*b)
    # 用 mean 而不是 median：画面里如果大部分是平坦区域，median 会退化成 0，
    # 于是 SNR 被算成天文数字（曾经算出来 119053，纯粹是统计量选错了）。
    sig_l = bl.std(axis=1).mean()
    sig_c = bc.std(axis=1).mean()
    p50, p99 = np.percentile(L, [50, 99])
    snr = (p99 - p50) / max(sig_l, 1e-6)
    return sig_l, sig_c, p50, p99, snr

rows = {}
for f in files:
    cfg = os.path.basename(f).split('#')[0]
    rows.setdefault(cfg, []).append(metrics(f))

print()
print(f"{'配置':26s} {'σ_L':>6s} {'σ_色':>6s} {'L中位':>6s} {'L99':>6s} {'SNR':>6s}")
print('-'*64)
for cfg, vals in rows.items():
    v = np.array(vals).mean(axis=0)
    print(f'{cfg[:26]:26s} {v[0]:6.1f} {v[1]:6.1f} {v[2]:6.1f} {v[3]:6.1f} {v[4]:6.1f}')
print()
print('说明：σ_L 是亮度噪声（越小越好），σ_色 是彩色斑点强度；')
print('      SNR = (L99 - L中位)/σ_L，物体与场地的反差相当于几倍噪声，<3 基本没法分割。')
PY
