# ===========================================================================
# win_sendinput.ps1 —— 常驻的按键注入器（Windows 侧），从 stdin 读命令
#
# 为什么常驻：每次按键都起一个 PowerShell 进程要几十毫秒，30 fps 的决策跟本
# 跟不上。这个脚本起来一次、`Add-Type` 一次，然后逐行读命令。
#
# 命令（每行一条）：
#     DN <vk>      按下虚拟键码（十进制或 0x 十六进制）
#     UP <vk>      松开
#     ALLUP        松开所有（看门狗用）
#     PING         回一行 PONG（探活）
#     QUIT         退出
#
# 虚拟键码：←37 ↑38 →39 ↓40，A=65 W=87 S=83 D=68
#
# ⚠ 用 keybd_event 而不是 SendInput：keybd_event 更短，对**普通窗口程序**够用。
#   两者都走同一条"合成输入"的路，**用 DirectInput/原始输入的游戏会忽略它们** ——
#   那种情况只能靠 RP2040 那样的真 HID 设备（这也是那个方案存在的理由）。
#   所以先用这个脚本**实测平台收不收**，收不了再回去做 RP2040。
# ===========================================================================
param([switch]$Quiet)

Add-Type -Namespace W -Name Kb -MemberDefinition @'
[DllImport("user32.dll", SetLastError=true)]
public static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, UIntPtr dwExtraInfo);
'@

$KEYUP = 0x0002

function Do-Dn([int]$vk) { [W.Kb]::keybd_event([byte]$vk, 0, 0, [UIntPtr]::Zero) }
function Do-Up([int]$vk) { [W.Kb]::keybd_event([byte]$vk, 0, $KEYUP, [UIntPtr]::Zero) }

# 记住按过谁，ALLUP 时逐个松开（keybd_event 没有"全松"）
$script:held = New-Object System.Collections.Generic.HashSet[int]

if (-not $Quiet) { Write-Output "READY" }

while ($true) {
    $line = [Console]::In.ReadLine()
    if ($null -eq $line) { break }                 # stdin 关了
    $line = $line.Trim()
    if ($line -eq "") { continue }

    $parts = $line -split '\s+'
    $cmd = $parts[0].ToUpper()
    try {
        switch ($cmd) {
            "DN" {
                $vk = [Convert]::ToInt32($parts[1], 0)
                if ($script:held.Add($vk)) { Do-Dn $vk }
                if (-not $Quiet) { Write-Output "DN $vk" }
            }
            "UP" {
                $vk = [Convert]::ToInt32($parts[1], 0)
                if ($script:held.Remove($vk)) { Do-Up $vk }
                if (-not $Quiet) { Write-Output "UP $vk" }
            }
            "ALLUP" {
                foreach ($vk in @($script:held)) { Do-Up $vk }
                $script:held.Clear()
                if (-not $Quiet) { Write-Output "ALLUP" }
            }
            "PING" { Write-Output "PONG" }
            "QUIT" { foreach ($vk in @($script:held)) { Do-Up $vk }; exit 0 }
            default { if (-not $Quiet) { Write-Output "ERR unknown $cmd" } }
        }
    } catch {
        if (-not $Quiet) { Write-Output "ERR $($_.Exception.Message)" }
    }
}
foreach ($vk in @($script:held)) { Do-Up $vk }
