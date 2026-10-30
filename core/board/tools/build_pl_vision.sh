#!/bin/sh
# build_pl_vision.sh —— 在**板子上**编译 pl_vision（L2 的感知源）并装到 /root/tools
#
# 为什么要脚本固定编译选项：正射是逐像素的活，`-O3 -funroll-loops` 与 `-O2` 差 37.7→25.7 ms
# （同一帧、同一台板子实测）。编译选项飘了，闭环帧率就跟着飘，所以写死在这里。
#
# 实测（2026-10-25，JFMQL30TAI676H，640×360 输出 / 640×480 输入）：
#   -O2 双线性 37.7 ms | -O3 双线性 25.7 ms | -O2/-O3 最近邻 14.8 ms
#   推字 17.3 ms + PL 22.5 ms → **端到端 65.5 ms/帧**（12 fps 只需 ≤83 ms）
set -e
SRC=${SRC:-/root/board_src/vt}
OUT=${OUT:-/root/tools/pl_vision}
cd "$SRC"
g++ -std=c++17 -O3 -funroll-loops -I . -o "$OUT" pl_vision.cpp geometry_frame.cpp
echo "已编译：$OUT（$(stat -c%s "$OUT") 字节）"
"$OUT" --w 640 --h 480 --frames 1 < /dev/null >/dev/null 2>&1 || true   # 冒烟（没 /dev/mem 时会退出非 0）
echo "用法：ffmpeg … -f rawvideo -pix_fmt rgb24 - | $OUT --w 640 --h 480 \\"
echo "        --quad-file /tmp/quad.json --jsonl /tmp/vt.jsonl --stats"
