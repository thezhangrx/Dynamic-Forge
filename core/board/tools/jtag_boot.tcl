# ===========================================================================
# jtag_boot.tcl —— 板子卡死/不启动时，从 JTAG 把它"注入启动"
#
# **必须在接着 JTAG 线的那台机器上跑**（本仓库环境是 WSL2：没有 USB、
#  D 盘的 Vivado 是 Windows 版；详见下面"为什么不在 WSL 里跑"）。
#
#     D:\Vivado\Vivado\2018.3\bin\xsdb.bat D:\FPGAWORKS\board_bits\jtag_boot.tcl
#
#   也可以直接把 WSL 里的这份喂给它（UNC 路径）：
#     D:\Vivado\Vivado\2018.3\bin\xsdb.bat \\wsl$\<发行版>\home\zhang\FPGA\core\board\tools\jtag_boot.tcl
#
# 做三件事，四步都打日志：
#   1. 看 JTAG 链上有没有 xc7z030 / arm_dap（没有 → 线没插好 / 板子没上电）
#   2. （可关）把 PL 位流烧进去
#   3. **系统复位** `rst -system`：让 PS 的 BootROM 重跑一遍 —— 这才是"启动"，
#      PS 按启动模式（SD/QSPI）读 BOOT.bin，再拉起 Linux。
#   4. 复位后 5 s 再看一次 targets，报告 PS 有没有活过来。
#
# 纯 JTAG 注一个完整 Linux（FSBL → u-boot → kernel 分段 dow）不在本文件范围，
# 本板的正常启动方式就是 SD 卡里的 BOOT.bin（见 core/fpga/docs/块设计.md）。
#
# 板子起来之后（SSH 192.168.137.242 恢复）：
#     bash core/board/tools/loop_standard.sh --check
# ===========================================================================

set DO_PL 1
set BIT_FILE D:/FPGAWORKS/board_bits/BIT/30tai_lite_detpost/ai7030_top_disable_icap.bit

proc banner {s} { puts "=== $s ===" }

puts "@@@BEGIN@@@"
if {[catch {connect} e]} {
    puts "CONNECT_FAIL: $e"
    exit 2
}
puts "CONNECT_OK"

# ---- 1. 扫链 ---------------------------------------------------------------
if {[catch {set jt [jtag targets]} e]} {
    puts "JTAG_FAIL: $e"
    exit 3
}
puts "JTAG_TARGETS: $jt"

set nz 0
foreach t $jt { if {[string match "*xc7z030*" $t]} { incr nz } }
if {$nz == 0} {
    puts "NO_ZYNQ_ON_CHAIN: 链上没有 xc7z030 —— 检查 JTAG 线/板子供电/启动模式"
    exit 4
}
puts "ZYNQ_ON_CHAIN_OK ($nz)"

# ---- 2. 烧 PL（可选）-------------------------------------------------------
if {$DO_PL} {
    banner "PROGRAM PL: $BIT_FILE"
    jtag targets -set -filter {name =~ "*xc7z030*"}
    if {[catch {fpga -f $BIT_FILE} e]} {
        puts "PROGRAM_PL_FAIL: $e"
        #: 烧 PL 失败不阻塞复位：PS 复位启动不依赖 PL（除非 FSBL 里有 PL 依赖）
    } else {
        puts "PROGRAM_PL_OK"
    }
}

# ---- 3. 找 PS / 系统复位 ---------------------------------------------------
jtag targets -set -filter {name =~ "*xc7z030*"}
if {[catch {set tl [targets]} e]} {
    puts "TARGETS_FAIL: $e"
    set tl {}
} else {
    foreach x $tl { puts "  TARGET: $x" }
}

set a9 ""
foreach x $tl { if {[string match "*Cortex-A9*" $x]} { set a9 $x; break } }

banner "RST -SYSTEM"
if {$a9 ne ""} {
    puts "  选中 PS 目标：$a9"
    catch {targets -set -filter {name =~ "*Cortex-A9*0*"}}
} else {
    puts "  ⚠ 没看到 Cortex-A9（PS 可能是被真正复位过/没起来）：仍尝试 rst -system"
}
if {[catch {rst -system} e]} {
    puts "RST_SYSTEM_FAIL: $e"
    if {[catch {rst -ps} e2]} { puts "RST_PS_FAIL: $e2" } else { puts "RST_PS_OK" }
} else {
    puts "RST_SYSTEM_OK"
}

# ---- 4. 复位后观察 ---------------------------------------------------------
after 5000
catch {con}
if {[catch {set t2 [targets]} e]} {
    puts "POST_TARGETS_FAIL: $e"
} else {
    foreach x $t2 { puts "  POST_TARGET: $x" }
}
puts "@@@END@@@"
exit 0
