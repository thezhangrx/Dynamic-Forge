#!/bin/bash
# 把板子侧源码**整体**同步过去并重建。
#
# 为什么需要：单文件 scp 改 header 时极易漏传 —— 实测踩过一次：本地改了
# `geometry.hpp`（把标记顺序从 TL,TR,BL,BR 改成 TL,TR,BR,BL），只同步了
# `main.cpp` 和另外两个 header，板子上编译**照样成功**（旧 header 语法没问题），
# 于是运行时所有帧都判 `marker_far_from_corner`、`field_valid` 恒 0，
# 看起来像"算法坏了"，其实是构建用了陈旧的头文件。
#
# 规矩：**header 与 main.cpp 一起传，然后整体重建**。这个脚本就是那条规矩。
set -u
cd "$(dirname "$0")/../../.." || exit 1
KEY=${KEY:-data/fpga/board_ssh/board_key}
HOST=${HOST:-root@192.168.137.242}
SRC=core/board
DST=/root/board_src/core/board
R() { ssh -n -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=8 "$HOST" "$@"; }

echo "[sync] 源码 → $HOST:$DST"
scp -q -i "$KEY" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "$SRC"/*.cpp "$SRC"/*.hpp "$HOST:$DST/" || { echo "[sync] 源码上传失败"; exit 1; }

# 工具（只看在板子上要编的那几个）
for t in wall_gap_view gt_decision_cli; do
    [ -f "$SRC/tools/$t.cpp" ] && scp -q -i "$KEY" -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null "$SRC/tools/$t.cpp" "$HOST:/root/tools/" 2>/dev/null
done

echo "[sync] 重建"
R 'cd /root/board_src/build && make -B vision_board_devmem -j3 2>&1 | grep -E "error:|warning: .*\[-Wformat" | head -20; echo "[sync] 错误行：$(cd /root/board_src/build && make vision_board_devmem -j3 2>&1 | grep -c "^.*error:")"'

echo "[sync] 工具重建"
R 'cd /root/tools && for t in wall_gap_view gt_decision_cli; do [ -f $t.cpp ] && g++ -std=c++17 -O2 -I /root/board_src/core/board -I /root/board_src/core/cpu/cpp -o $t $t.cpp 2>&1 | head -3; done; ls -la /root/board_src/build/vision_board_devmem'
