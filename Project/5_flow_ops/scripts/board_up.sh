#!/bin/bash
# ===========================================================================
# board_up.sh —— 板子起没起来、PL 里是哪版位流，一条命令看清楚
#
#     bash core/board/tools/board_up.sh            # 等板子上线（默认 120 s）
#     bash core/board/tools/board_up.sh --check    # 只看一眼，不等
#     bash core/board/tools/board_up.sh -t 300     # 等 5 分钟
#
# 为什么需要：2026-10-29 现场前发现**冷上电（真断电再上电）PS 有时起不来**，
# 但 `reboot` / JTAG `rst -system` 这种**热复位**一按就起来 —— 典型是 SD 卡
# 在冷上电那一下没初始化成功。这个脚本把"到底是没起来 / 起来了但 PL 不对"
# 分开报出来，并给出下一步该按哪个键。
#
# 判据（都要给数字）：
#   1. ssh 通不通（3 s 一次）
#   2. PL state 是不是 operating
#   3. /lib/firmware/bd_vision.bin 的 md5 是不是我们部署的那份（默认 P0 c6180da2）
# ===========================================================================
set -u

BOARD="${BOARD:-root@192.168.137.242}"
KEY="${KEY:-data/fpga/board_ssh/board_key}"
WANT_MD5="${WANT_MD5:-c6180da2fab6aa917121521d03df2502}"   # P0 修复版
EXPECT_BOOT="${EXPECT_BOOT:-6.0}"                            # 正常 ~9 s 到多用户
T=120
CHECK=0
while [ $# -gt 0 ]; do
    case "$1" in
        --check) CHECK=1; shift ;;
        -t|--timeout) T=$2; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数 $1" >&2; exit 2 ;;
    esac
done

SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=3 -o BatchMode=yes $BOARD"

probe() {
    # 输出：state|md5|uptime   失败返回 1
    out=$($SSH 'echo "$(cat /sys/class/fpga_manager/fpga0/state)|$(md5sum /lib/firmware/bd_vision.bin | cut -d" " -f1)|$(cut -d. -f1 /proc/uptime)"' 2>/dev/null) || return 1
    [ -n "$out" ] || return 1
    echo "$out"
    return 0
}

report_ok() {
    local state md5 up
    state=${1%%|*}; md5=$(echo "$1" | cut -d'|' -f2); up=$(echo "$1" | cut -d'|' -f3)
    echo "板子已上线（uptime ${up}s）"
    echo "  PL state      : $state   $([ "$state" = operating ] && echo '✓' || echo '✗ 没配好')"
    echo "  bd_vision.bin : $md5"
    if [ "$md5" = "$WANT_MD5" ]; then
        echo "  → PL 里就是部署的那份位流 ✓"
    else
        echo "  → ⚠ 和期望的 $WANT_MD5 不一致："
        echo "      bash core/board/tools/deploy_pl_autoload.sh data/fpga/pl_bitstream/bd_vision_p0.bit"
    fi
    [ "$state" = operating ] || echo "  → ⚠ state 不是 operating：systemctl restart pl-autoload 看一眼"
}

if [ "$CHECK" = "1" ]; then
    if r=$(probe); then report_ok "$r"; exit 0; fi
    echo "板子不通（ssh $BOARD 失败）"; exit 1
fi

echo "等板子上线（最多 ${T}s，每 3s 试一次）..."
t0=$(date +%s)
while :; do
    if r=$(probe); then
        dt=$(( $(date +%s) - t0 ))
        echo "等了 ${dt}s 起来"
        report_ok "$r"
        exit 0
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$T" ]; then break; fi
    sleep 3
done

cat <<'EOF'

板子 ${T}s 内没上线。按顺序做（**先试 1，不用 PC 也不用 JTAG**）：

  1. 按板子上的**复位键**，然后再跑一次 `bash core/board/tools/board_up.sh`
     （2026-10-29 实测：冷上电常起不来、一按复位就起来 —— SD 卡冷启动没初始化成功）
  2. 还不行：确认供电独立、电流够（别和 USB/其它负载共用），断电 5 s 再上电，重复 1
  3. 还不行：走 JTAG 兜底
       D:\Vivado\Vivado\2018.3\bin\xsdb.bat \\wsl$\Ubuntu\home\zhang\FPGA\core\board\tools\jtag_boot.tcl
     （现在链上能看到 xc7z030；若又报 NO_ZYNQ_ON_CHAIN，查 JTAG 排线/供电）
  4. 起来以后**别再断电**：要重启用 ssh 里的 `reboot`，那是热复位，稳。
EOF
exit 1
