#!/usr/bin/env bash
# ===========================================================================
# capture_boot.sh —— 在 **Windows 侧**抓板子控制台（启动日志），存进 data/
#
# 为什么在 Windows 侧：控制台是板载 CP210x，插在 Windows 主机上（COM3）。
# WSL 里要看它得先 `usbipd attach`（需要管理员），而且 attach 之后 Windows
# 那边就看不到了 —— 而要看启动日志的场合，往往正是 WSL 都连不上的时候。
#
#   bash core/board/tools/capture_boot.sh 180        # 抓 180 秒
#   PORT=COM5 bash core/board/tools/capture_boot.sh  # 换串口
#
# ⚠ UART **没有回放缓冲**：要抓启动日志，必须在给板子上电**之前**就开始抓。
#   所以顺序是：先跑这条命令 → 再给板子断电/上电（或按复位）。
# ⚠ 一个串口同一时刻只能被一个程序打开：先关掉 MobaXterm / 串口助手。
# ===========================================================================
set -u
SECS=${1:-120}
PORT=${PORT:-COM3}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT="$ROOT/data/fpga/boot_logs/boot_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$OUT")"

# 退出时**必须**清掉 Windows 侧那个 PowerShell：job 被 kill（Ctrl-C / 超时 / 后台任务
# 取消）时 Windows 子进程不一定跟着死，它会一直占着 COMx —— 下一次抓取就打不开端口，
# 而失败信息又可能被缓冲住，表现成"日志 0 字节、看着像板子什么都没打印"。
# 这个坑真踩过：清掉遗留进程后一切正常。
cleanup() {
    powershell.exe -NoProfile -Command \
      "Get-CimInstance Win32_Process -Filter \"Name='powershell.exe'\" | Where-Object { \$_.CommandLine -like '*win_console.ps1*' -and \$_.ProcessId -ne \$PID } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force -ErrorAction SilentlyContinue }" \
      >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

echo "串口 $PORT，抓 $SECS 秒 → ${OUT#$ROOT/}"
echo "现在给板子**断电再上电**（或按复位）—— 先抓后上电才拿得到启动日志。"
powershell.exe -NoProfile -ExecutionPolicy Bypass \
    -File "$(wslpath -w "$HERE/win_console.ps1")" -Port "$PORT" -Seconds "$SECS" 2>&1 \
  | tee "$OUT"
# ⚠ 这里**故意没有** `tr -d '\r'`：tr 输出到管道/文件时全缓冲，会把板子的输出
#   卡在它缓冲区里（日志文件一直 0 字节，看着像"板子什么都没打印"）。
#   CR 已经在 win_console.ps1 里剥掉了。
echo "已存：${OUT#$ROOT/}"
