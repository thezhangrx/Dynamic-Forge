#!/bin/bash
# ===========================================================================
# gt_demo.sh —— **一条命令**：真值闭环 + 决策可视化（没有相机）
#
#     bash core/board/tools/gt_demo.sh [秒数]
#
# 起三样东西：
#   1. 仿真（全屏，带 --danger 分层势图），stdin 由 `tail -f` 供（永不 EOF）
#   2. 中继 gt_loop.py：真值 → gt_decision_cli → @TGT/@DBG/@MOVE
#   3. 屏幕上应当看到：分层色势图、黑边墙、黄色准星（决策瞄准点）、HUD 决策文本
#
# 先修掉"上一次没退干净"的进程——不然新窗口会被旧窗口盖住，看起来像"没改"。
# ===========================================================================
set -u
cd "$(dirname "$0")/../../.." || exit 1
SECS=${1:-120}
PY=Env/bin/python
DEC=/tmp/gt_decide

pkill -f "bullet_sim board-contro[l]" 2>/dev/null
pkill -f "gt_loo[p].py" 2>/dev/null
sleep 1
rm -f /tmp/mv.txt /tmp/gt.jsonl
touch /tmp/mv.txt

echo "[gt_demo] 编译决策 CLI ..."
g++ -std=c++17 -O2 -I core/cpu/cpp -o "$DEC" \
    core/board/tools/gt_decision_cli.cpp core/cpu/cpp/gap_avoid.cpp || exit 1

echo "[gt_demo] 起仿真（--danger，分层势图）..."
(setsid sh -c "tail -f /tmp/mv.txt | $PY -u -m bullet_sim board-control \
    --stdin --nostop --duration $SECS --fullscreen \
    --potential --state-log /tmp/gt.jsonl" > /tmp/sim_gt.log 2>&1 &)
sleep 5
echo '@MOVE 0' >> /tmp/mv.txt      # 喂一行，仿真才会开始跑
sleep 2

echo "[gt_demo] 起中继（真值 → 决策 → 动作）..."
# 决策**在板子上**跑（真·板子决策）：@GT 经 ssh 喂给 /root/tools/gt_decide_board
KEY=${KEY:-data/fpga/board_ssh/board_key}
"$PY" core/board/tools/gt_loop.py --state /tmp/gt.jsonl --out /tmp/mv.txt \
      --max-seconds "$SECS" --decide "$DEC" --ssh-key "$KEY"
