#!/usr/bin/env bash
# ===========================================================================
# sync.sh —— 把"acc9 全流程在用的轮子"从仓库原处同步到 core/using/
#
#   bash core/using/sync.sh            # 拷一遍（原件 → using）
#   bash core/using/sync.sh --check    # 只对账：逐字节比，不一致就报出来
#
# 为什么要有这个脚本：`core/using/` 只放**拷贝**，原件仍是唯一真相。没有对账，
# 过两周就分不清"using 里这份是不是最新的"——那正是这个仓库最容易踩的坑。
#
# 2026-10-24 用户口径：这一版 using/ 是 **acc9 快照**（跑出 run_20261024_110028_acc9r3
# 的那一套）。所以对照表= acc9 那轮实际加载的文件；码带/GPS/闸门那些**不在**这里。
# 其中 board_agent.py / vt_sparse.cpp / start_cam.sh 三件是 `0806d81` 原版，
# 其余按 2026-10-24 工作树状态（platform_host.py 另含相机设备自动探测）。
# ===========================================================================
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
cd "$ROOT"

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

# 表：原件路径 → using 子目录
FILES="
core/board/tools/platform_host.py|host
core/board/tools/page_view.py|host
core/board/tools/mjpeg_client.py|host
core/board/tools/full_loop.py|host
core/board/tools/run_log.py|host
core/board/tools/field_loop.py|host
core/board/tools/calib_y.py|host
core/board/tools/loop_standard.sh|host
core/board/tools/grade_run.py|host
core/board/tools/check_run_logs.py|host
core/board/tools/audit_crossings.py|host
core/board/tools/make_task.py|host
core/board/tools/board_agent.py|board
core/board/tools/vt_sparse.cpp|board
core/board/tools/start_cam.sh|board
core/board/tools/field_decision_cli.cpp|board
core/board/field_m1.hpp|board
data/fpga/quad.json|calib
data/fpga/calib_y.json|calib
configs/20_walls_alt_24_dy140.json|configs
"

bad=0; n=0
while IFS='|' read -r src dst; do
    [ -z "${src:-}" ] && continue
    n=$((n + 1))
    if [ ! -f "$src" ]; then
        echo "  ✗ 原件不存在：$src"; bad=$((bad + 1)); continue
    fi
    if [ "$CHECK" = "1" ]; then
        if ! cmp -s "$src" "$HERE/$dst/$(basename "$src")"; then
            echo "  ✗ 不一致：$src  vs  using/$dst/$(basename "$src")"; bad=$((bad + 1))
        fi
    else
        mkdir -p "$HERE/$dst"
        cp -f "$src" "$HERE/$dst/"
    fi
done <<< "$FILES"

# 两个包按目录同步（排除缓存；测试保留，因为 acc9 快照就是全量拷贝）
for pair in "core/bullet_sim|bullet_sim" "core/standard|standard"; do
    src="${pair%%|*}"; dst="${pair##*|}"
    if [ "$CHECK" = "1" ]; then
        if ! diff -rq --exclude '__pycache__' "$src" "$HERE/$dst" >/tmp/_using_diff.txt 2>&1; then
            echo "  ✗ $dst 不一致："; sed 's/^/      /' /tmp/_using_diff.txt | head -20
            bad=$((bad + 1))
        fi
    else
        mkdir -p "$HERE/$dst"
        rsync -a --exclude '__pycache__/' "$src/" "$HERE/$dst/"
    fi
done

if [ "$CHECK" = "1" ]; then
    if [ "$bad" = "0" ]; then echo "  ✓ core/using 与原件逐字节一致（$n 个文件 + bullet_sim + standard）"
    else echo "  ✗ 有 $bad 处不一致 —— 跑一次 bash core/using/sync.sh 同步"; fi
    exit $([ "$bad" = "0" ] && echo 0 || echo 1)
else
    echo "  已同步 $n 个文件 + bullet_sim + standard → core/using/"
fi
