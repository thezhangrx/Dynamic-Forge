# =============================================================================
# vision_axi 实现（综合 → opt → place → route → 比特流）
#
# 用法（Vivado 2018.3 batch）：
#   vivado -mode batch -source impl.tcl -log ... -journal ... \
#       -tclargs <rtl_dir> <out_dir> <tag>
#
# 产物（都在 <out_dir>，即 data/ 侧，不进版本库）：
#   <tag>_util.rpt      实现后的利用率（这才是能报给板子的数）
#   <tag>_timing.rpt    时序摘要（WNS / WHS / 是否收敛）
#   <tag>_clk.rpt       时钟利用率
#   <tag>_drc.rpt       DRC 报告
#   <tag>_routed.dcp    布线后的检查点 —— 补上引脚约束后**不用重跑 12 分钟**
#   <tag>.bit           比特流（**只有引脚约束齐了才生成**，见下）
#
# 为什么带 tag：`docs/帧存改造.md` 里每一步改动都要能拿同一把尺子复现，
# 输出不能互相覆盖。
#
# ⚠ 时钟约束见 out_dir/vision.xdc：**20 ns（50 MHz）是推导出来的，不是拍的**。
#   协同仿真实测每帧约 5.6e5 拍，30 fps 只需要 ~16.7 MHz，50 MHz 有 3 倍余量。
#
# -----------------------------------------------------------------------------
# ⚠⚠ 比特流为什么需要一个额外的引脚约束文件（2026-10-13 实测）
#
# 只跑到 route_design 是**成功**的：
#     Slice LUTs 20792 (26.45%)、Block RAM 82.5 (31.13%)、DSP 46、
#     WNS +0.910 ns / WHS +0.068 ns，91577 个端点 0 失败
#     → "All user specified timing constraints are met"
#
# 但 `write_bitstream` 会被 DRC 挡下：
#     ERROR: [DRC NSTD-1] Unspecified I/O Standard: 135 out of 135 logical ports
#     ERROR: [DRC UCIO-1] Unconstrained Logical Port: 135 out of 135
#     （`ZPS7-1 PS7 block required` 只是 WARNING）
#
# **不要**为了让比特流生成就把 NSTD-1 / UCIO-1 降级成 Warning。
# 本设计的 135 个端口是 AXI-Lite 从口 + 视频输入（`din_*`），真实部署里它们
# **来自 PS7**（PS 跑 Linux，读相机，经 AXI 把帧送进 PL），**不是外部引脚**。
# 降级 DRC 等于让 FPGA 去驱动 135 个电平未定义的引脚 —— **可能烧板子**。
#
# 正确的两条路（二选一，都还没做）：
#   A. 做 **PS7 块设计**：PS7 + AXI 互联 + 本模块，135 个端口变成内部连接，
#      外部只剩板子自己的引脚（DDR/MIO，由 PS7 与厂商预设给出）。
#      素材在 `D:\FPGAWORKS\board_bits\fpai_demo_fpga`（`ps_ai.bd`、
#      `ps_ai_wrapper.sv`、`ip_user_files/mem_init_files/ps7_init.tcl`、
#      `constrs_1/impl_constraints.xdc` 里的板级引脚表），
#      **但那套 .bd 是 Vivado 2020.2 的，与手上的 2018.3 不兼容**，需要重建。
#   B. 做 **PL-only 上板自检顶层**：内部接一个固定的测试图案源 + 只用板子
#      已知的引脚（`sys_clk_p/n` = C8/C7、`led[0/1]` = J1/M6，IOSTANDARD 见
#      `impl_constraints.xdc`），把帧流做 CRC/心跳输出到 LED。
#      端口少、全部有约束 ⇒ 比特流能出，且**无争用风险**，
#      可以在真硅上验证"综合后的电路真的跑得完一帧"。
#
# 本脚本的做法：如果 <out_dir>/board_pins.xdc 存在就读进来（路 A/B 都会产出它），
# 否则跳过比特流、只出检查点，并显式打印原因 —— **不静默失败，也不偷偷降级 DRC**。
# -----------------------------------------------------------------------------
# =============================================================================
set rtl_dir [lindex $argv 0]
set out_dir [lindex $argv 1]
set tag     [lindex $argv 2]

read_verilog [glob $rtl_dir/*.v]
read_xdc $out_dir/vision.xdc

set pin_xdc [file join $out_dir board_pins.xdc]
if {[file exists $pin_xdc]} {
    puts "@@@READING_PIN_CONSTRAINTS $pin_xdc@@@"
    read_xdc $pin_xdc
} else {
    puts "@@@NO_PIN_CONSTRAINTS：只做实现与检查点，不生成比特流@@@"
}

synth_design -top vision_axi -part xc7z030ffg676-2 -include_dirs $rtl_dir
opt_design
place_design
route_design

report_utilization       -file $out_dir/${tag}_util.rpt
report_utilization -hierarchical -file $out_dir/${tag}_util_hier.rpt
report_timing_summary    -file $out_dir/${tag}_timing.rpt
report_clock_utilization -file $out_dir/${tag}_clk.rpt

# 布线后的检查点：补引脚约束后只要 `open_checkpoint` + `write_bitstream`，
# 不必重跑综合与布线。
write_checkpoint -force $out_dir/${tag}_routed.dcp

if {[file exists $pin_xdc]} {
    report_drc -file $out_dir/${tag}_drc.rpt
    write_bitstream -force $out_dir/$tag.bit
    puts "@@@BITSTREAM_OK $tag@@@"
} else {
    puts "@@@IMPL_DONE_NO_BITSTREAM $tag@@@"
}
exit
