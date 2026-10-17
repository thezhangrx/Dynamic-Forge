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
#: 障碍速度：**不传**，与 `python -m bullet_sim play` 完全一致（用场景自己的速度）。
#: 踩过的坑：之前这里默认传了 --obstacle-speed 120，但用户实测 SPEED=580 时
#: 墙速也不变 —— 说明 board-control 这条路上那个参数没有生效（场景构建时另有一套
#: 速度），所以最稳妥的做法是**不覆盖**，让它和 play 一样。
SPEED=${SPEED:-}
PY=Env/bin/python
DEC=/tmp/gt_decide

pkill -f "bullet_sim board-contro[l]" 2>/dev/null
pkill -f "gt_loo[p].py" 2>/dev/null
sleep 1
rm -f /tmp/mv.txt /tmp/gt.jsonl /tmp/state.pkt /tmp/local_dec.txt
touch /tmp/mv.txt /tmp/state.pkt

echo "[gt_demo] 编译决策 CLI ..."
g++ -std=c++17 -O2 -I core/cpu/cpp -o "$DEC" \
    core/board/tools/gt_decision_cli.cpp core/cpu/cpp/gap_avoid.cpp || exit 1

# **先把生效的墙速打出来**（不靠肉眼判断快慢）：同一组参数走一遍场景构建。
EXTRA_ARGS=""
if [ -n "$SPEED" ]; then EXTRA_ARGS="--obstacle-speed $SPEED"; fi
echo "[gt_demo] 场景速度自检："
"$PY" core/board/tools/scene_speed.py play --input board $EXTRA_ARGS 2>&1 | sed 's/^/    /'

echo "[gt_demo] 起仿真（--danger，分层势图）..."
(setsid sh -c "tail -f /tmp/mv.txt | $PY -u -m bullet_sim board-control \
    --stdin --nostop --duration $SECS --fullscreen \
    $EXTRA_ARGS \
    --potential --state-pipe /tmp/state.pkt --state-log /tmp/gt.jsonl" > /tmp/sim_gt.log 2>&1 &)
sleep 3
echo '@MOVE 0' >> /tmp/mv.txt      # 喂一行，仿真才会开始跑（不等太久）
sleep 1

echo "[gt_demo] 起中继（真值 → 决策 → 动作）..."
# 决策**在板子上**跑（真·板子决策）：@GT 经 ssh 喂给 /root/tools/gt_decide_board
KEY=${KEY:-data/fpga/board_ssh/board_key}
"$PY" core/board/tools/gt_loop.py --packets /tmp/state.pkt --out /tmp/mv.txt --record-local /tmp/local_dec.txt \
      --max-seconds "$SECS" --decide "$DEC" --ssh-key "$KEY"
