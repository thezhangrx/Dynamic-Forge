#!/bin/bash
# 把仿真窗口**提到最前面**（相机拍的是物理屏幕，被浏览器盖住就等于把相机蒙上）。
#
# 为什么需要：仿真跑的是 `--fullscreen`，但在 Windows/WSLg 下别的窗口（浏览器、终端）
# 完全可能压在上面。相机看到的就是最上层那一个窗口 —— 实测一次闭环跑里
# `field_valid` 只有 59%、`ps_new` 只找到 6% 的角色，看似"视觉退化"，
# 截屏一看是浏览器盖在上面（屏幕上根本没有场地）。
#
# 做法：用 PowerShell 枚举顶层窗口，按标题找到 bullet_sim 的窗口，SetForegroundWindow
# 并 ShowWindow(SW_MAXIMIZE)。找不到就报出来（让人去点一下）。
#
# 用法：
#   bash core/board/tools/focus_sim.sh            # 提到前台
#   bash core/board/tools/focus_sim.sh --check    # 只报有没有、在不在前台
set -u
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
if [ ! -x "$PS" ]; then
    echo "[focus] 找不到 powershell.exe（不是 WSL？）—— 手动把仿真窗口点到最前面"
    exit 1
fi
SCRIPT=$(cat <<'PS1'
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Win {
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, System.Text.StringBuilder s, int n);
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
}
"@
$found = $null
$cb = [Win+EnumProc]{
  param($h, $l)
  if ([Win]::IsWindowVisible($h)) {
    $sb = New-Object System.Text.StringBuilder 512
    [void][Win]::GetWindowText($h, $sb, 512)
    $t = $sb.ToString()
    if ($t -like "*bullet_sim*") { $script:found = $h; Write-Output ("TITLE " + $t) }
  }
  return $true
}
[void][Win]::EnumWindows($cb, [IntPtr]::Zero)
if ($script:found -eq $null) { Write-Output "NO_WINDOW"; exit 0 }
$fg = [Win]::GetForegroundWindow()
if ($fg -eq $script:found) { Write-Output "ALREADY_FOREGROUND" }
if ("$env:FOCUS_ACTION" -eq "raise") {
  # ⚠ 用 SW_RESTORE(9) **不是** SW_MAXIMIZE(3)：全屏窗口再"最大化"会被改尺寸，
  #   场地的四个角就被推出屏幕外（实测截屏里只剩左上角一个标记，相机自然找不到
  #   其余三个）。只要恢复 + 提到前台就够了。
  [void][Win]::ShowWindow($script:found, 9)   # SW_RESTORE
  [void][Win]::SetForegroundWindow($script:found)
  Start-Sleep -Milliseconds 300
  $fg2 = [Win]::GetForegroundWindow()
  if ($fg2 -eq $script:found) { Write-Output "RAISED_OK" } else { Write-Output "RAISE_MAYBE_BLOCKED" }
}
PS1
)
if [ "$CHECK" = "1" ]; then
    FOCUS_ACTION=check "$PS" -NoProfile -Command "$SCRIPT" 2>&1 | sed 's/^/[focus] /'
else
    FOCUS_ACTION=raise "$PS" -NoProfile -Command "$SCRIPT" 2>&1 | sed 's/^/[focus] /'
fi
