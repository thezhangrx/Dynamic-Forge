#!/usr/bin/env bash
# 烧录 STM32F103C8T6（MiniBalance 一体板）
#
# 用法：
#   ./flash.sh              # 先备份原固件，再烧 build/car.bin，最后回读校验
#   ./flash.sh --restore    # 把最近一次备份写回去
#
# 前置：ST-Link 必须已经挂进 WSL（Windows 管理员 PowerShell 里
#       usbipd list → usbipd bind --busid X → usbipd attach --wsl --busid X），
#       且 `st-info --probe` 能看到 1 个 programmer。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/build"
FLASH_BASE=0x08000000
FLASH_SIZE=0x10000          # C8T6 = 64 KB

need_tool() {
    command -v "$1" >/dev/null 2>&1 || { echo "缺少 $1" >&2; exit 1; }
}

need_tool st-info
need_tool st-flash

echo "== 探测调试器 =="
if ! st-info --probe 2>&1 | grep -q "Found 1"; then
    echo "没找到 ST-Link（当前：$(st-info --probe 2>&1 | head -1)）" >&2
    echo "把 ST-Link 挂进 WSL 后再试，或者用 Keil 在 Windows 上烧。" >&2
    exit 2
fi
st-info --probe

latest_backup() {
    ls -t "$BUILD"/backup_orig_*.bin 2>/dev/null | head -1
}

backup() {
    mkdir -p "$BUILD"
    local out="$BUILD/backup_orig_$(date +%Y%m%d_%H%M%S).bin"
    echo "== 备份原固件 -> $out =="
    st-flash read "$out" $FLASH_BASE $FLASH_SIZE
    if [ ! -s "$out" ]; then
        echo "备份失败（文件为空）" >&2
        exit 3
    fi
    # 全 0xFF 说明芯片是空的，没有东西可备份
    if [ "$(xxd -p "$out" | tr -d '\n' | tr -d 'f' | wc -c)" -eq 0 ]; then
        echo "  注意：原固件是全 0xFF（空片），没什么可备份的"
        rm -f "$out"
        return 0
    fi
    echo "  已保存 $(stat -c %s "$out") 字节"
}

restore() {
    local b
    b="$(latest_backup)"
    [ -n "$b" ] || { echo "找不到备份文件（$BUILD/backup_orig_*.bin）" >&2; exit 4; }
    echo "== 恢复 $b =="
    st-flash write "$b" $FLASH_BASE
    st-flash reset
    echo "恢复完成"
}

case "${1:-}" in
    --restore) restore; exit 0 ;;
    "") ;;
    *) echo "用法: $0 [--restore]" >&2; exit 1 ;;
esac

BIN="$BUILD/car.bin"
[ -f "$BIN" ] || { echo "没有 $BIN，先 make" >&2; exit 5; }

backup

echo "== 烧录 $BIN =="
st-flash --reset write "$BIN" $FLASH_BASE

echo "== 回读校验 =="
TMP="$(mktemp)"
st-flash read "$TMP" $FLASH_BASE "$(stat -c %s "$BIN")"
if cmp -s "$BIN" "$TMP"; then
    echo "  校验通过：写入的 $(stat -c %s "$BIN") 字节与文件一致"
else
    echo "  校验失败！回读内容与文件不同" >&2
    rm -f "$TMP"
    exit 6
fi
rm -f "$TMP"

st-flash reset
echo
echo "烧录完成。串口接 USART1（PA9/PA10，115200）时应看到 @READY 和每 200 ms 的 @STATE/@ODOM。"
