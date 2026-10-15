param(
    [string]$Port = "COM3",
    [int]$Baud = 115200,
    [int]$Seconds = 20,
    [string]$Send = "",
    [switch]$Dtr
)
# ===========================================================================
# win_console.ps1 —— 在 **Windows 侧**读板子控制台（CP210x / COMx）
#
# 为什么要在 Windows 侧读：CP210x 插在 Windows 主机上，WSL 里要看它得先
# `usbipd attach`（需要管理员），而且 attach 之后 Windows 那边就看不到了。
# 要看的偏偏是"WSL 都连不上"时的启动日志 —— 那就得从 Windows 直接读。
#
#   powershell -ExecutionPolicy Bypass -File win_console.ps1 -Port COM3 -Seconds 20
#   ... -Send "root"      顺便往控制台发一行（`\r` 自动带上）
#
# ⚠ 一个串口同一时刻只能被一个程序打开：先关掉 MobaXterm / 串口助手，
#   否则报"拒绝访问"。
# ⚠ DTR/RTS **默认不动**（有些板子把它们接到复位/BOOT 上，拉一下就把板子按住
#   或复位了），确实需要时再加 -Dtr。
# ⚠ 本文件必须是 **UTF-8 with BOM**：Windows PowerShell 5.1 对没有 BOM 的脚本按
#   系统 ANSI 代码页解码，中文注释会被解坏、把语法拆掉（症状是起来就 ParserError）。
# ===========================================================================
$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

function Write-Line([string]$s) { [Console]::Out.WriteLine($s); [Console]::Out.Flush() }

# 注意：不能用 `New-Object System.IO.Ports.SerialPort $Port, $Baud, [..]::None, ...`
# —— 那种写法里 PowerShell 不解析 `::` 静态成员（会当成字符串），必须分步赋值。
$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName = $Port
$sp.BaudRate = $Baud
$sp.Parity = [System.IO.Ports.Parity]::None
$sp.DataBits = 8
$sp.StopBits = [System.IO.Ports.StopBits]::One
$sp.ReadTimeout = 200
$sp.WriteTimeout = 1000
if ($Dtr) { $sp.DtrEnable = $true; $sp.RtsEnable = $true }

try { $sp.Open() } catch {
    Write-Line "打开 $Port 失败：$($_.Exception.Message)"
    Write-Line "（多半是别的程序占着这个串口：先关掉 MobaXterm / 串口助手）"
    exit 1
}
Write-Line "---- 读 $Port @ $Baud，$Seconds 秒 ----"

if ($Send -ne "") {
    $sp.Write($Send + "`r")
    Write-Line "---- 已发送：$Send ----"
}

$end = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $end) {
    try {
        $s = $sp.ReadExisting()
        if ($s.Length -gt 0) { [Console]::Out.Write($s); [Console]::Out.Flush() }
    } catch { }
    Start-Sleep -Milliseconds 50
}
$sp.Close()
Write-Line ""
Write-Line "---- 结束 ----"
