#!/usr/bin/env bash
# ===========================================================================
# pl_check.sh —— PL（vision_top）接口层一键自检：从主机一条命令跑完
#
#     bash core/board/tools/pl_check.sh              # 全量自检（参数/帧契约/统计/控制面耗时）
#     bash core/board/tools/pl_check.sh --cadence 2  # 只测帧节拍（需板端有进程在推帧）
#
# 它做的事：把 pl_probe.py 推到板子上执行，然后**按判据给结论**：
#   * 参数表：21 个配置寄存器有非零值（说明这一轮写过、不是刚上电的空壳）；
#   * 帧契约：magic='BHV1'、长度 = 76+36*n_obs+36*n_gaps == FRAME_LEN、帧尾 CRC32 对得上；
#   * 统计：n_obs/n_gaps 与帧头一致（调试口的统计与帧体交叉核对）；
#   * 控制面：单次寄存器访问耗时（ps 级还是 µs 级，决定 PS 轮询策略）。
#
# 前置：板子 ssh 可达；PL 位流已加载（fpga_manager state=operating）；
#       /dev/mem 可访问。**只读/只写 PL 寄存器，不动相机、不动闭环。**
#
# 实测基线（2026-10-21，真实相机帧 640×360）：
#   参数 21 项一致；帧 need=472 == FLEN=472、CRC 双方一致；
#   节拍 mean 29.96 ms / sd 0.03 ms；PS 触发→done 12.96 ms；单次寄存器 2.9~3.6 µs
# ===========================================================================
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
BOARD="root@192.168.137.242"
KEY="$ROOT/data/fpga/board_ssh/board_key"
PROBE="$ROOT/core/board/tools/pl_probe.py"

CADENCE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --cadence) CADENCE="--cadence $2"; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

echo "== 推探针到板子 =="
scp -q -i "$KEY" -o StrictHostKeyChecking=no "$PROBE" "$BOARD:/tmp/pl_probe.py" || {
    echo "  ✗ scp 失败（板子不可达？）" >&2; exit 3; }

echo "== PL 接口层自检 =="
ssh -i "$KEY" -o StrictHostKeyChecking=no "$BOARD" \
    "python3 /tmp/pl_probe.py $CADENCE"
RC=$?

echo
if [ "$RC" = "0" ]; then
    echo "== 结论：接口层自检通过（帧契约 OK、统计与帧头一致）=="
else
    echo "== 结论：自检失败（rc=$RC）——看上面哪一项是 MISMATCH/全 0 =="
    echo "   常见原因：位流没加载完 / 寄存器契约与 RTL 不一致 / 这一帧是空帧（n_obs=0 也算正常）"
fi
exit "$RC"
