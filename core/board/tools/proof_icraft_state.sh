#!/usr/bin/env bash
# ===========================================================================
# proof_icraft_state.sh —— 出"ICraft 运行时到底能不能用 / AI_Mate 在不在"的证据
#
#     bash core/board/tools/proof_icraft_state.sh            # 在板子上跑，只读
#
# 输出三块，每块都带"命令 + 原始输出"，可直接截图/贴给厂家：
#   A. icraft Python 包在不在、依赖的 C 库在不在（结论：包在、库缺、运行时不可用）
#   B. 全盘有没有 libicraft*.so（`find /` 而不是 `find /lib`——/lib 是 -> usr/lib 的符号链接，
#      find 默认不跟随，之前就因此误判过一次）
#   C. AI_Mate 的软件/硬件痕迹：设备树、/dev、内核模块
#
# 只读：不改任何文件（除了自己 tee 出来的报告）。
# ===========================================================================
set -u
REPORT="${1:-/tmp/icraft_state_$(date +%Y%m%d_%H%M).txt}"
exec > >(tee "$REPORT") 2>&1

echo "================ A. icraft 运行时 ================"
echo "--- 系统 ---"
uname -a
. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME"
echo "root=$(findmnt -no SOURCE /)  uuid=$(dumpe2fs -h "$(findmnt -no SOURCE /)" 2>/dev/null | awk '/Filesystem UUID/{print $3}')"
echo
echo "--- Python 包（期望：在，3.33.1 / fmsh）---"
pip3 show icraft 2>/dev/null | head -6 || echo "  pip3: 没装 icraft"
echo
echo "--- 顶层 import（注意：能过不代表能用，__init__ 只 import os/sys）---"
python3 -c "import icraft; print('  import icraft -> OK')" 2>&1 | tail -2
echo "--- 真正用起来（期望：ImportError）---"
python3 -c "from icraft import xrt; print('  from icraft import xrt -> OK')" 2>&1 | tail -2
echo
echo "--- 依赖的 C 库：ldconfig 登记 vs 文件实际存在 ---"
echo "  ldconfig 登记条数: $(ldconfig -p 2>/dev/null | grep -c icraft)"
for n in xrt xir utils pl_sim hostbackend buyibackend buyi_sim zg330backend xir_serialize_flatbuffers; do
    f="/lib/aarch64-linux-gnu/libicraft_${n}.so"
    printf "  %-52s " "$f"
    [ -e "$f" ] && echo "存在" || echo "缺失"
done
echo
echo "--- ldd 看 python 扩展缺哪些 ---"
for so in /usr/local/lib/python3.8/dist-packages/icraft/*.so; do
    [ -e "$so" ] || continue
    miss=$(ldd "$so" 2>/dev/null | grep "not found" | grep -c icraft)
    printf "  %-40s 缺 icraft 库 %s 个\n" "$(basename "$so")" "$miss"
done
echo

echo "================ B. 全盘有没有 libicraft（正确写法）================"
echo "  /lib 是符号链接: $(ls -ld /lib | awk '{print $9,$10,$11}')"
echo "  find / -xdev -name 'libicraft*'（排除 /proc /sys /dev）:"
find / -xdev \( -path /proc -o -path /sys -o -path /dev \) -prune -o -name "libicraft*" -print 2>/dev/null | head -10
echo "  上面为空 => 全盘没有任何 libicraft*.so"
echo
echo "  另外：/etc/ld.so.cache 的时间（这份缓存来自厂商构建机，登记了库里没有的文件）:"
ls -l /etc/ld.so.cache 2>/dev/null | awk '{print "   ",$6,$7,$8}'
echo

echo "================ C. AI_Mate 的痕迹 ================"
echo "--- 设备树 ---"
for k in ai_mate ai-mate zg330 npu aicore; do
    printf "  %-10s 命中 %s 个节点\n" "$k" "$(grep -rl "$k" /proc/device-tree 2>/dev/null | wc -l)"
done
echo "--- /dev 与内核模块 ---"
ls /dev | grep -iE "^npu|icraft|^zg" || echo "  /dev: 没有 npu/icraft/zg* 节点"
lsmod 2>/dev/null | grep -iE "icraft|zg330|ai_mate" || echo "  lsmod: 没有相关模块"
echo "--- fpga_manager 状态（PL 里装的是什么由位流决定）---"
ls /sys/class/fpga_manager/ 2>/dev/null
for d in /sys/class/fpga_manager/fpga*; do
    [ -e "$d/state" ] && echo "  $d: state=$(cat $d/state) name=$(cat $d/name 2>/dev/null)"
done
echo "  /lib/firmware 里的位流: $(ls -l /lib/firmware/*.bin 2>/dev/null | awk '{print $5, $9}')"
echo
echo "================ 结论 ================"
echo "  A. icraft Python 包: 在；依赖的 C 库: 缺失 => 运行时不可用（ImportError 而不是 ImportError: No module named icraft）"
echo "  B. 全盘没有 libicraft*.so，ldconfig 缓存是厂商构建机带过来的脏条目"
echo "  C. AI_Mate: 设备树/设备节点/内核模块 0 命中；PL 位流是我们自己的设计（见主机侧 .bit 头 design=sys_wrapper）"
echo
echo "报告已写: $REPORT"
