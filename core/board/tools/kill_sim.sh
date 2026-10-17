#!/bin/bash
# 收尾脚本：杀残留的仿真/中继。放成文件是因为在 bash -c 里直接写 pkill 的模式
# 会匹配到自己的命令行（自伤），这个坑踩过好几次。
pkill -f "bullet_sim board-control" 2>/dev/null
pkill -f "gt_loop.py" 2>/dev/null
sleep 0.3
echo "cleaned: $(pgrep -cf 'bullet_sim board-control') sim(s)"
