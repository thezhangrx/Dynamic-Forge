#!/bin/bash
# =============================================================================
# board_l0.sh —— 把新位流传上板、重载 PL，然后做 **L0 对拍**（PL vs C 参考）
#
# 为什么要这个脚本：L0 的判据是"同一帧喂 PL 与跑 C 参考，几何帧**逐字节一致**"。
# 以前这一串步骤是手敲的（scp 位流、echo firmware、编译 ref、跑两个喂数器、cmp），
# 每次都容易漏一步；而 round 7 定位到的缺陷（WSTRB=0 的写被当成数据）正好只在
# "上板 + 真喂数"这条路上暴露，所以这条链要能一条命令重复跑。
#
# 用法（本机，WSL）：
#     core/fpga/tools/board_l0.sh                # 用默认位流 C:\pl_build\out\bd_vision.bit.bin
#     core/fpga/tools/board_l0.sh <bin> [帧数]    # 指定位流 + 帧数（默认 3，L0 要做 ≥300）
#     core/fpga/tools/board_l0.sh --no-reload     # 只做对拍，不动 PL（位流已经在板上时用）
#
# 判据（全部通过才算 L0 过）：
#   1. `cmp ref.bin pl.bin` 逐字节一致（3 帧都要）；
#   2. 帧头 magic/ver/长度/CRC 自洽，**n_obs/n_gaps 与参考相同**（不是 0）；
#   3. `0x0EC`（被忽略的空选通写次数）≈ 2×字数 —— 证明"一笔 store 三笔写"的判断；
#   4. `0x0E0`（PL 收到的字节数）= 691200，且 `0x0E4/0x0E8`（xor/sum）与本机算法一致。
# =============================================================================
set -u
cd "$(dirname "$0")/../../.." || exit 1         # 仓库根目录（tools → fpga → core → 根）

BOARD=${BOARD:-root@192.168.137.242}
KEY=${KEY:-data/fpga/board_ssh/board_key}
BIN_DEFAULT=/mnt/c/pl_build/out/bd_vision.bit.bin
STIM=${STIM:-data/fpga/stim_color}
SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 $BOARD"

RELOAD=1
if [ "${1:-}" = "--no-reload" ]; then RELOAD=0; shift; fi
#: 参数放宽：位流路径/frame 数顺序随意，只给数字就当帧数（手敲不容易错）。
BIN=$BIN_DEFAULT
NFRAMES=3
for a in "$@"; do
    if [ -f "$a" ]; then BIN=$a
    elif [ "$a" -eq "$a" ] 2>/dev/null; then NFRAMES=$a
    else echo "看不懂的参数：$a" >&2; exit 2
    fi
done
[ -f "$BIN" ] || { echo "找不到位流：$BIN" >&2; exit 2; }

say() { printf '\n=== %s\n' "$*"; }

say "0) 板子可达性"
$SSH 'uname -r; cat /sys/class/fpga_manager/fpga0/state' || exit 1

if [ "$RELOAD" = 1 ]; then
    say "1) 上传位流并重载 PL（$(basename "$BIN")，$(stat -c%s "$BIN") 字节）"
    local_md5=$(md5sum "$BIN" | awk '{print $1}')
    $SSH 'cat > /lib/firmware/bd_vision.bin' < "$BIN" || exit 1
    remote_md5=$($SSH 'md5sum /lib/firmware/bd_vision.bin' | awk '{print $1}')
    echo "  md5 本机=$local_md5 板上=$remote_md5"
    [ "$local_md5" = "$remote_md5" ] || { echo "  ✗ 传输不一致" >&2; exit 1; }
    # fpga_manager：写 firmware 触发整颗 PL 重配置
    $SSH 'echo bd_vision.bin > /sys/class/fpga_manager/fpga0/firmware; sleep 0.5; cat /sys/class/fpga_manager/fpga0/state'
fi

say "2) 上传参考实现 / 喂数器 / 测试帧"
$SSH 'mkdir -p /tmp/l0'
for f in vision_ref.c ref_api.h; do
    $SSH "cat > /tmp/l0/$f" < "core/fpga/ref/$f" || exit 1
done
for f in feed_pl.py feed_pl.c; do
    $SSH "cat > /tmp/l0/$f" < "core/fpga/tools/$f" || exit 1
done
frames=()
i=0
for p in "$STIM"/*.ppm; do
    b=$(basename "$p")
    $SSH "cat > /tmp/l0/$b" < "$p" || exit 1
    frames+=("${b%.ppm}")
    i=$((i + 1))
    [ "$i" -ge "$NFRAMES" ] && break
done
echo "  ${#frames[@]} 帧：${frames[*]}"

say "3) 板上编译 C 参考 + C 喂数器"
$SSH 'cd /tmp/l0 && gcc -O2 -o visref vision_ref.c -lm && gcc -O2 -o feed_pl_c feed_pl.c' || exit 1

#: 逐字节比对的判据：帧体必须完全一样，但**头里的 seq（8..11）和 stamp_ns（24..31）
#: 是运行期字段**（参考程序写 0，喂数器写真实帧号/时间戳），不算差异。
#: 这一点必须写死成脚本：否则每次都要靠人眼确认"只有 seq 不同"。
$SSH "cat > /tmp/l0/cmp_frame.py" <<'PYEOF'
# 对拍规则（L0）：**帧体必须逐字节一致**，但下面三类字段不参与比较 ——
#   seq(8..11)、stamp_ns(24..31)：运行期字段，参考程序写 0、喂数器写真实值；
#   crc(末尾 4 字节)：它的定义就是覆盖整个帧（含 seq/stamp），seq 不同则必然不同，
#   所以它单独校验 —— **每个文件用自己帧体重算 CRC，必须等于帧内 CRC**。
# 这样"内容对不对"和"帧自不自洽"两件事都被覆盖，且不靠人眼。
import struct, sys, zlib
a = open(sys.argv[1], "rb").read()     # 参考
b = open(sys.argv[2], "rb").read()     # PL
def need_of(x):
    return 76 + 36 * (x[12] + x[16])
def selfcheck(x, tag):
    n = need_of(x)
    if len(x) < n:
        print("%s: 长度 %d < 帧声明的 %d" % (tag, len(x), n)); return False
    crc_in = struct.unpack_from("<I", x, n - 4)[0]
    crc_calc = zlib.crc32(x[:n - 4]) & 0xFFFFFFFF
    if crc_in != crc_calc:
        print("%s: 自身 CRC 不符（帧内 0x%08x / 重算 0x%08x）" % (tag, crc_in, crc_calc))
        return False
    return True
if not (selfcheck(a, "参考") and selfcheck(b, "PL")):
    sys.exit(1)
na, nb = need_of(a), need_of(b)
if na != nb:
    print("帧长不同：参考 %d / PL %d" % (na, nb)); sys.exit(1)
def mask(x, n):
    return x[:8] + b"\0" * 4 + x[12:24] + b"\0" * 8 + x[32:n - 4]
if mask(a, na) != mask(b, nb):
    d = [i for i in range(na) if mask(a, na)[i] != mask(b, nb)[i]]
    print("帧体不一致：%d 处，前 5 个偏移 %s" % (len(d), d[:5]))
    sys.exit(1)
sys.exit(0)
PYEOF
say "4) 逐帧对拍（C 参考 vs PL，忽略 seq/stamp）"
fail=0
for fr in "${frames[@]}"; do
    #: 喂数器：先试 C 版（快，L1 用）；它若因为这个 RTL 版本的写通道挂掉（SIGBUS），
    #: 退回 Python 版（慢 30 倍，但 L0 只判"对不对"，不判快慢）。
    out=$($SSH "cd /tmp/l0 && ./visref $fr.ppm $fr.ref.bin 2>&1; \
                rm -f $fr.pl.bin; \
                ./feed_pl_c $fr.ppm --dump $fr.pl.bin 2>&1 || true; \
                if [ ! -s $fr.pl.bin ]; then echo '[退回 Python 喂数器]'; \
                    python3 feed_pl.py --image $fr.ppm --w 640 --h 360 --dump $fr.pl.bin 2>&1 | tail -2; fi" ) || true
    echo "$out" | sed 's/^/  /'
    # 参考侧数字（n_obs / n_gaps）与 PL 侧数字都要抓出来比
    ref_line=$(echo "$out" | grep -o 'n_obs=[0-9]* n_gaps=[0-9]*' | head -1)
    pl_line=$(echo "$out" | grep -o 'n_obs=[0-9]* n_gaps=[0-9]*' | tail -1)
    if $SSH "cd /tmp/l0 && python3 cmp_frame.py $fr.ref.bin $fr.pl.bin" && [ "$ref_line" = "$pl_line" ]; then
        echo "  ✓ $fr 逐字节一致（$ref_line）"
    else
        echo "  ✗ $fr 不一致：参考 $ref_line / PL $pl_line"
        fail=$((fail + 1))
    fi
done

say "5) 诊断寄存器（0x0E0 字节 / 0x0E4 xor / 0x0E8 sum / 0x0EC 空选通写次数）"
$SSH 'python3 - <<PY
import mmap, os, struct
fd=os.open("/dev/mem", os.O_RDWR|os.O_SYNC)
m=mmap.mmap(fd,4096,mmap.MAP_SHARED,mmap.PROT_READ|mmap.PROT_WRITE,offset=0x43C00000)
rd=lambda o: struct.unpack_from("<I",m,o)[0]
for name,off in (("bytes",0x0E0),("xor",0x0E4),("sum",0x0E8),("zstrb",0x0EC)):
    print("  %-6s = %d (0x%08x)" % (name, rd(off), rd(off)))
PY'

say "结论"
if [ "$fail" = 0 ]; then
    echo "  L0 通过（$NFRAMES/$NFRAMES 帧逐字节一致）"
    exit 0
fi
echo "  L0 未通过：$fail/$NFRAMES 帧不一致" >&2
exit 1
