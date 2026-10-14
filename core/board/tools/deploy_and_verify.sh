#!/bin/bash
# ===========================================================================
# deploy_and_verify.sh —— 位流一好就跑完整条板上验证（**在 WSL 侧跑**）
#
#     bash core/board/tools/deploy_and_verify.sh
#
# 它按"从最安全到最危险"的顺序排，每步的结论都单独打出来：
#
#   0) 查位流产物 + 记录 md5（说清楚板子上到底是哪一份）
#   1) 送 bd_vision.bit.bin 到板子 /lib/firmware/bd_vision.bin
#   2) 经 fpga_manager 加载，确认 state
#   3) `read_frame --watch`（**纯读，不写任何寄存器**）
#        —— 顺带证明 AXI 读通路通、地址被译码；预期结果是"帧计数不动"，
#           因为 tpg_source 只送像素、不启动几何帧，只有写 CTRL bit0 才启动。
#           如果这一步能拿到帧，说明有别的源在自动启动帧（值得追一下）。
#   4) `axi_soak`（**会写**，这一步才是 AXI 写死锁的真正判据）
#        Phase B 的金丝雀会先做**单次**配置寄存器写：要挂就挂在那一行，
#        失败点最清楚；过了就说明写通路修好了，后面才是压测与端到端帧。
#
# ⚠ 如果位流还是旧的那一份（没有 AXI 写修复），第 4 步会把 CPU 挂死，
#   只能**彻底断电**恢复（复位键无效）。所以第 3 步特意放在前面：
#   它是只读的，能在不冒任何风险的前提下先把"位流到底加载成功没有"确认掉。
# ===========================================================================
set -u

REPO=$(cd "$(dirname "$0")/../../.." && pwd)
KEY=$REPO/data/fpga/board_ssh/board_key
BOARD=${BOARD:-192.168.137.242}
BIN=${BIN:-/mnt/d/FPGAWORKS/vivado_impl/bd_vision.bit.bin}
SOAK_ROUNDS=${SOAK_ROUNDS:-3000}
SOAK_FRAMES=${SOAK_FRAMES:-8}

SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 -o BatchMode=yes root@$BOARD"
SCP="scp -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o BatchMode=yes"

echo "########## 0) 位流产物 ##########"
if [ ! -f "$BIN" ]; then
    echo "！！找不到 $BIN —— 位流还没建好（跑 core/fpga/synth/bd_pack.tcl 再 bd_vision.tcl）"
    exit 1
fi
ls -la "$BIN"
MD5=$(md5sum "$BIN" | cut -d' ' -f1)
echo "待烧位流 md5: $MD5"

# ⚠ 防"烧了旧位流"：D: 上会一直留着上一轮的 .bit/.bit.bin，光看"文件存在"
#   会把旧的烧上去 —— 而旧位流正好含 AXI 写死锁，会把板子挂死。
#   位流必须**比它所依赖的 RTL 新**，否则说明重建没跑到这一步。
RTL=$REPO/core/fpga/rtl/vision_axi.v
if [ -f "$RTL" ] && [ "$BIN" -ot "$RTL" ]; then
    echo "！！位流比 $RTL 还旧 —— 这是上一轮的旧产物，重建没完成，拒绝烧写"
    echo "   bin: $(date -r "$BIN" '+%F %T')    RTL: $(date -r "$RTL" '+%F %T')"
    exit 1
fi
if [ -n "${EXPECT_MD5:-}" ] && [ "$MD5" != "$EXPECT_MD5" ]; then
    echo "！！md5 与 EXPECT_MD5 不符：期望 $EXPECT_MD5，实际 $MD5"
    exit 1
fi

echo
echo "########## 1) 板子可达性 + 当前 PL 状态 ##########"
$SSH 'echo -n "  uptime: "; uptime | sed "s/.*up //;s/,.*//"
      echo -n "  当前 fpga_manager state: "; cat /sys/class/fpga_manager/fpga0/state
      echo -n "  板上现有 bin md5: "; md5sum /lib/firmware/bd_vision.bin 2>/dev/null | cut -d" " -f1' || {
    echo "！！板子不可达，先确认网线和 IP（静态 192.168.137.242 由 rc.local 配）"; exit 1; }

echo
echo "########## 2) 送位流并加载 ##########"
$SCP "$BIN" root@$BOARD:/lib/firmware/bd_vision.bin
$SSH 'sync
      echo bd_vision.bin > /sys/class/fpga_manager/fpga0/firmware
      sleep 1
      echo -n "  加载后 state: "; cat /sys/class/fpga_manager/fpga0/state
      echo "  --- 内核消息 ---"; dmesg | grep -iE "fpga|devcfg" | tail -4'

echo
echo "########## 3) 只读探针：read_frame --watch（不写任何寄存器，零风险）##########"
$SSH 'cd /root/tools && ./read_frame 1 --watch'; rc=$?
echo "  （退出码 $rc；预期是\"帧计数都没动\"—— 那说明必须显式写 CTRL 启动）"

echo
echo "########## 4) AXI 写通路加压 + 端到端帧（会写！这一步才是判据）##########"
echo "  ⚠ 若位流没含 AXI 写修复，这一步会挂死 CPU，需要彻底断电"
$SSH "cd /root/tools && ./axi_soak $SOAK_ROUNDS --frames $SOAK_FRAMES"; soak=$?

echo
echo "########## 汇总 ##########"
echo "位流 md5            : $MD5"
echo "只读探针退出码      : $rc   (0=帧正常自由运行；2=没有源在启动帧，属预期)"
echo "写通路加压退出码    : $soak  (0=全过；2=有失配；3=地址没译码)"
if [ "$soak" = "0" ]; then
    echo "⇒ AXI 写死锁 + 字节选通两项修复在真硅上确认生效。"
else
    echo "⇒ 未通过，看上面的 ✗ 明细。若整条 SSH 掉线=写挂死了，需彻底断电冷启动。"
fi
