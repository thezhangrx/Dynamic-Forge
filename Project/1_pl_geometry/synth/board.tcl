# =============================================================================
# 板级 bring-up 位流：vision_board_top（只用一个时钟 + 两个 LED + 一个 ILA）
#
# 用法：
#   vivado -mode batch -source board.tcl -log ... -journal ... \
#       -tclargs <rtl_dir> <rtl_board_dir> <out_dir> <tag>
#
# 为什么不用 impl.tcl：`vision_axi` 的 135 个端口没有 LOC/IOSTANDARD
# （真实部署里它们接 PS7，不是外部引脚），出不了比特流；把 DRC 降级又会
# 让 FPGA 驱动电平未定义的引脚。这个顶层把端口收敛到板子上**已知且已约束**的
# 4 个（sys_clk_p/n = C8/C7、led = J1/M6），并且里面跑的就是那套被逐位对账过
# 的 vision_selftest 流水线。
#
# ILA 是这里**唯一能看见的观测手段**：上位机看不到板子的 LED，只有把
# frame_done / busy / cur_frame_idx / dout_vld / {locked,rst_n} 抓出来读，
# 才能回答"综合后的电路在真硅上到底跑没跑"。
# =============================================================================
set rtl_dir   [lindex $argv 0]
set board_dir [lindex $argv 1]
set out_dir   [lindex $argv 2]
set tag       [lindex $argv 3]

# ---- 不放 ILA（见 rtl_board/vision_board_top.v 里的说明）------------------
# in-memory 项目里 `create_ip` 生成的 IP 模块**进不了 synth_design**：
#   ERROR: [Synth 8-439] module 'ila_0' not found
# 需要片上观测时改用综合后插入的 create_debug_core + connect_debug_port。
# 这一版只要两个 LED 就够（人能直接看板子）。

# ---- 读 RTL 与引脚约束 ----------------------------------------------------
read_verilog [glob $rtl_dir/*.v]
read_verilog [glob $board_dir/*.v]
read_xdc $out_dir/board_pins.xdc

synth_design -top vision_board_top -part xc7z030ffg676-2 \
    -include_dirs "$rtl_dir $board_dir"
opt_design
place_design
route_design

report_utilization       -file $out_dir/${tag}_util.rpt
report_timing_summary    -file $out_dir/${tag}_timing.rpt
report_drc               -file $out_dir/${tag}_drc.rpt

write_checkpoint -force $out_dir/${tag}_routed.dcp
write_bitstream -force $out_dir/$tag.bit
write_debug_probes -force $out_dir/$tag.ltx

puts "@@@BOARD_BITSTREAM_OK $tag@@@"
exit
