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
function Open-Port {
    try {
        $p = New-Object System.IO.Ports.SerialPort
        $p.PortName = $Port
        $p.BaudRate = $Baud
        $p.Parity = [System.IO.Ports.Parity]::None
        $p.DataBits = 8
        $p.StopBits = [System.IO.Ports.StopBits]::One
        $p.ReadTimeout = 200
        $p.WriteTimeout = 1000
        if ($Dtr) { $p.DtrEnable = $true; $p.RtsEnable = $true }
        $p.Open()
        return $p
    } catch {
        return $null
    }
}

# 为什么要有重连：给板子**断电重上电**时，如果 USB 也跟着断了，CP210x 会从
# Windows 上消失再出现（COMx 短暂不存在）。只"打开一次"的抓取会在那一刻失效，
# 而我们要抓的偏偏就是上电那一瞬间的日志。
$deadline = (Get-Date).AddSeconds($Seconds)
$sp = $null
$retries = 0
$sent = $false

while ((Get-Date) -lt $deadline) {
    if ($null -eq $sp -or -not $sp.IsOpen) {
        if ($null -ne $sp) { try { $sp.Close() } catch { } }
        $sp = Open-Port
        if ($null -eq $sp) {
            # 端口在系统里但打不开 = 多半被别的程序占着 → 快速失败，别让人干等
            $exists = ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port)
            if (-not $exists) {
                if ($retries -eq 0) { Write-Line "---- $Port 现在不在（设备可能刚被断电）—— 持续等待重连 ----" }
                $retries++
                Start-Sleep -Milliseconds 400
                continue
            }
            Write-Line "打开 $Port 失败：端口存在但打不开（多半被 MobaXterm / 串口助手占着）"
            exit 1
        }
        if ($retries -gt 0) { Write-Line "---- $Port 重新打开成功（等了 $retries 次）----" }
        else { Write-Line "---- 读 $Port @ $Baud，$Seconds 秒 ----" }
    }

    try {
        # 在 PowerShell 里就把 CR 去掉：**不能**交给外面的 `tr -d '\r'` ——
        # tr 输出到管道/文件时是全缓冲的，板子的输出会卡在它缓冲区里，
        # 日志文件和管道都看不到实时内容（抓启动日志时这个坑最致命）。
        $s = $sp.ReadExisting()
        if ($s.Length -gt 0) {
            [Console]::Out.Write($s.Replace("`r", ""))
            [Console]::Out.Flush()
        }
    } catch { }
    if ($Send -ne "" -and -not $sent) {
        try { $sp.Write($Send + "`r"); Write-Line ""; Write-Line "---- 已发送：$Send ----"; $sent = $true } catch { }
    }
    Start-Sleep -Milliseconds 50
}
if ($null -ne $sp) { try { $sp.Close() } catch { } }
Write-Line ""
Write-Line "---- 结束 ----"
