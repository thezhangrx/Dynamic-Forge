# =============================================================================
# vision_axi 综合 / 面积诊断
#
# 用法（Vivado 2018.3 batch）：
#   vivado -mode batch -source synth.tcl -tclargs <rtl_dir> <out_dir> <tag> [directive]
#
# 例：
#   vivado -mode batch -source synth.tcl -tclargs \
#       D:/FPGAWORKS/vivado_impl/rtl D:/FPGAWORKS/vivado_impl diag_sg
#
# 为什么参数化：`docs/帧存改造.md` §13/§14 的每一步改动都要在同一把尺子下量，
# 否则数字不可比。固定 part / 固定顶层 / 固定约束，只换 RTL 目录和输出前缀。
#
# 产物（都在 <out_dir>，即 data/ 侧，不进版本库）：
#   <tag>.rpt            总体利用率
#   <tag>_hier.rpt       层次利用率
#   <tag>_ram.rpt        推断出的每一块 RAM/LUTRAM（含尺寸与位置）
#   <tag>.log / .jou
# =============================================================================
set rtl_dir [lindex $argv 0]
set out_dir [lindex $argv 1]
set tag     [lindex $argv 2]
set directive [lindex $argv 3]

read_verilog [glob $rtl_dir/*.v]
read_xdc $out_dir/vision.xdc

if {$directive eq "" || $directive eq "none"} {
    synth_design -top vision_axi -part xc7z030ffg676-2 -include_dirs $rtl_dir
} else {
    synth_design -top vision_axi -part xc7z030ffg676-2 -include_dirs $rtl_dir \
        -directive $directive
}

report_utilization     -file $out_dir/$tag.rpt
report_utilization -hierarchical -file $out_dir/${tag}_hier.rpt
# 这份最关键：它把每一块被推断成 RAM 的数组连**位宽×深度**一起列出来，
# 没进这张表的数组就是被拆成了寄存器。见 §14.2。
report_ram_utilization -file $out_dir/${tag}_ram.rpt

puts "@@@DONE $tag@@@"
exit
