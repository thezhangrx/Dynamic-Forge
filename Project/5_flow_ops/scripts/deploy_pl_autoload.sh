#!/bin/bash
# ===========================================================================
# deploy_pl_autoload.sh —— 一条命令：把新位流变成"板子上电自动加载"
#
#     bash core/board/tools/deploy_pl_autoload.sh [bit 路径]
#     # 默认用 data/fpga/pl_bitstream/bd_vision_p0.bit
#
# 做四步（每步都有输出，最后给判定）：
#   1. .bit → .bin（纯 Python 版 bootgen，见 bit2bin.py；与 bootgen 逐字节一致）
#   2. scp 到板子 /lib/firmware/bd_vision.bin（旧的备份成 bd_vision_prev.bin）
#   3. systemctl restart pl-autoload  → fpga_manager 立刻重配 PL
#   4. 读 0x43C00958（FRMDIAG，**只有 P0 位流才有这个寄存器**）验证 PL 到底是哪版
#
# 为什么不用 JTAG：板子上电时 systemd 的 pl-autoload.service 就会
#   echo bd_vision.bin > /sys/class/fpga_manager/fpga0/firmware
# 所以只要这个文件是新的，**板子自己就是对的**，xsdb/JTAG 只当兜底。
# ===========================================================================
set -euo pipefail

BOARD="${BOARD:-root@192.168.137.242}"
KEY="${KEY:-data/fpga/board_ssh/board_key}"
BIT="${1:-data/fpga/pl_bitstream/bd_vision_p0.bit}"
SSH="ssh -i $KEY -o StrictHostKeyChecking=no -o ConnectTimeout=8 $BOARD"

[ -f "$BIT" ] || { echo "找不到位流：$BIT" >&2; exit 1; }

echo "== 1/4 .bit → .bin"
TMP_BIN=$(mktemp /tmp/pl_XXXXXX.bin)
python3 core/board/tools/bit2bin.py "$BIT" -o "$TMP_BIN"
MD5=$(md5sum "$TMP_BIN" | cut -d' ' -f1)

echo "== 2/4 传上去（旧文件备份成 bd_vision_prev.bin）"
scp -i "$KEY" -o StrictHostKeyChecking=no -q "$TMP_BIN" "$BOARD:/lib/firmware/bd_vision_new.bin"
$SSH 'set -e; cd /lib/firmware; [ -f bd_vision.bin ] && cp -f bd_vision.bin bd_vision_prev.bin; mv -f bd_vision_new.bin bd_vision.bin; md5sum bd_vision.bin'
rm -f "$TMP_BIN"

echo "== 3/4 重启 autoload 服务（立刻重配 PL）"
$SSH 'systemctl restart pl-autoload; sleep 1; echo "  state=$(cat /sys/class/fpga_manager/fpga0/state)"'

echo "== 4/4 推 3 帧合成画面，读 FLEN/FRMDIAG 验证 PL 里是哪版设计"
# 注意：0x958 在**空闲时是 0**（sof_cnt/环指针都归零），所以必须先推几帧再看，
# 不能像一开始那样空读一下就下结论。
SKIP_VERIFY=${SKIP_VERIFY:-0}
if [ "$SKIP_VERIFY" = "1" ]; then
    echo "  (SKIP_VERIFY=1，跳过)"
else
    $SSH 'cd /tmp
if [ -x /root/tools/pl_vision ] && command -v ffmpeg >/dev/null 2>&1; then
  timeout 40 sh -c "ffmpeg -hide_banner -loglevel error -f lavfi -i testsrc=size=640x480:rate=12 \
     -frames:v 3 -f rawvideo -pix_fmt rgb24 - | /root/tools/pl_vision --w 640 --h 480 \
     --quad-file /root/vision_state/quad.json --frames 3 --jsonl /tmp/vt_plcheck.jsonl" \
     >/tmp/plcheck.log 2>&1 || echo "  (推帧没跑成，看 /tmp/plcheck.log)"
else
  echo "  (板上没有 pl_vision 或 ffmpeg，跳过推帧)"
fi
python3 - <<PY
import mmap, os, struct
fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
m = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=0x43C00000)
g = lambda o: struct.unpack("<I", m[o:o+4])[0]
flen, diag = g(0x0D0), g(0x958)
print("  FLEN(0x0D0)     = 0x%08X  %s" % (flen, "(有几何帧)" if flen else "(没有帧)"))
print("  FRMDIAG(0x958)  = 0x%08X  %s" % (diag, "(P0 位流才有这个寄存器)" if diag else ""))
print("  => %s" % ("P0 位流在线，PL 正常出帧" if (flen and diag) else
                   ("PL 有响应但没出帧：看 /tmp/plcheck.log" if diag else
                    "0x958 是 0：PL 里可能不是 P0 位流（或推帧没跑）")))
PY'
fi

echo
echo "完成：/lib/firmware/bd_vision.bin md5 = $MD5（来自 $BIT）"
echo "以后板子上电即生效；JTAG 只是兜底。要回滚："
echo "  $SSH 'cd /lib/firmware && cp -f bd_vision_prev.bin bd_vision.bin && systemctl restart pl-autoload'"
