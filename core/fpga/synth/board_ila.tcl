# =============================================================================
# 带 ILA 的板级位流（**项目 + Runs 流程**）
#
# 为什么绕了三圈才走到这里 —— 三条路的实测结果
# ---------------------------------------------
#   ① `create_ip` + `synth_design`（in-memory 项目）
#      → ERROR: [Synth 8-439] module 'ila_0' not found（IP 没被带上）
#   ② 综合后插 `create_debug_core` + `create_debug_port`
#      → ERROR: [Vivado 12-711] exceeds the maximum '1' allowed debug ports
#        想改探针数：ERROR: [Vivado 12-4444] Invalid property name
#        C_NUM_OF_PROBES（2018.3 的 debug_core 上没这个属性）
#   ③ 真项目 + create_ip + **直接调 synth_design/opt_design/place/route**
#      → 位流能出，但板上**找不到 ILA**：
#           WARNING: [Labtools 27-3361] The debug hub core was not detected.
#        因为 **dbg_hub（JTAG↔ILA 的桥）的插入是 Runs 流程做的**，
#        直接调用 opt_design 不会插。
#
# 所以正路是：**落盘项目 + create_ip + launch_runs synth_1/impl_1**。
#
# RTL 侧的 ILA 例化用 `` `ifdef VISION_BOARD_ILA `` 隔开，普通构建
# （board.tcl）不需要任何 IP。
#
# 探针：probe0 frame_done(1) probe1 ok_mask(8) probe2 cur_frame_idx(16)
#       probe3 nb_now(16)   probe4 tl_now(32)
# =============================================================================
set rtl_dir   [lindex $argv 0]
set board_dir [lindex $argv 1]
set out_dir   [lindex $argv 2]
set tag       [lindex $argv 3]

set prj $out_dir/ila_prj
file delete -force $prj
create_project -force ila_prj $prj -part xc7z030ffg676-2
set_property target_language Verilog [current_project]

add_files -norecurse [glob $rtl_dir/*.v]
add_files -norecurse [glob $board_dir/*.v]
add_files -fileset constrs_1 -norecurse $out_dir/board_pins.xdc

# 顶层与综合选项
set_property top vision_board_top [current_fileset]
set_property verilog_define {VISION_BOARD_ILA} [current_fileset]
set_property include_dirs "$rtl_dir $board_dir" [current_fileset]

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_0
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES      {5} \
    CONFIG.C_PROBE0_WIDTH       {1} \
    CONFIG.C_PROBE1_WIDTH       {8} \
    CONFIG.C_PROBE2_WIDTH       {16} \
    CONFIG.C_PROBE3_WIDTH       {16} \
    CONFIG.C_PROBE4_WIDTH       {32} \
    CONFIG.C_DATA_DEPTH         {1024} \
    CONFIG.C_INPUT_PIPE_STAGES  {1} \
    CONFIG.C_EN_STRG_QUAL       {0} \
] [get_ips ila_0]
generate_target all [get_ips ila_0]
puts "@@@ ILA IP 已生成 @@"

# ---- Runs 流程：dbg_hub 由它插 --------------------------------------------
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    puts "@@@ SYNTH_FAIL @@"; exit 1
}
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    puts "@@@ IMPL_FAIL @@"; exit 1
}

open_run impl_1
report_utilization    -file $out_dir/${tag}_util.rpt
report_timing_summary -file $out_dir/${tag}_timing.rpt
write_bitstream    -force $out_dir/$tag.bit
write_debug_probes -force $out_dir/$tag.ltx
puts "@@@ILA_BITSTREAM_OK $tag@@@"
exit
