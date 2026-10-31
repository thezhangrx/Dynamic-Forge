#!/usr/bin/env bash
# cam_lock.sh —— 相机独占锁：预览 / VT / 标定只能有一个在跑。
#
#   bash core/board/tools/cam_lock.sh status     # 看谁占着相机
#   bash core/board/tools/cam_lock.sh release    # 释放（停预览/VT/闭环残留）
#   bash core/board/tools/cam_lock.sh preview [port]  # 起预览
#
# 用 /proc 扫 cmdline（板端没有 ss/lsof），不依赖 pgrep 的模式匹配，
# 避免"自己把自己 kill 了"的坑（R5）。
set -u
BOARD="root@192.168.137.242"
KEY="data/fpga/board_ssh/board_key"
ssh_cmd() { ssh -i "$KEY" -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$BOARD" "$1"; }

scan() {
    ssh_cmd 'for p in /proc/[0-9]*; do c=$(tr "\0" " " < $p/cmdline 2>/dev/null); case "$c" in
        *mjpeg_server*|*vt_sparse*|*vt_board*|*ffmpeg\ -hide_banner\ -loglevel\ error\ -f\ v4l2*)
            echo "${p#/proc/}: $(echo $c | cut -c1-70)";; esac; done' 2>/dev/null
}

case "${1:-status}" in
    status)
        echo "== 相机占用 =="; scan; echo "（空 = 没人占）";;
    release)
        echo "== 释放相机 =="
        ssh_cmd 'for p in /proc/[0-9]*; do c=$(tr "\0" " " < $p/cmdline 2>/dev/null); case "$c" in
            *mjpeg_server*|*vt_sparse*|*vt_board*|*ffmpeg\ -hide_banner*) kill ${p#/proc/} 2>/dev/null;; esac; done; sleep 1; echo done'
        ;;
    preview)
        PORT="${2:-8100}"
        ssh_cmd "cd /root/tools && (setsid nohup python3 mjpeg_server.py $PORT >/tmp/mjpeg.log 2>&1 </dev/null &); sleep 5; tail -1 /tmp/mjpeg.log"
        echo "预览：http://192.168.137.242:$PORT/"
        ;;
    *) echo "用法: $0 status|release|preview [port]" >&2; exit 2 ;;
esac
