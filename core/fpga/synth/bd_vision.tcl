# =============================================================================
# 块设计：PS7 + AXI 互联 + vision_axi_ip + tpg_source
#
# 这一步解决的是**根本问题**：`vision_axi` 的 135 个端口（AXI-Lite 从口 +
# 视频输入）真实部署里来自 PS7，不是外部引脚 —— 所以裸模块出不了比特流
# （DRC NSTD-1/UCIO-1），而**降级 DRC 会让 FPGA 驱动电平未定义的引脚、可能烧板**。
# 把 PS7 例化进来之后，这些端口变成**内部连接**，外部只剩 PS 自己的
# DDR/MIO（**专用引脚，由 PS7 自动约束**）—— 于是 DRC 自然通过。
#
# 结构：
#
#     PS7 ──FCLK_CLK0(50MHz)──┬──────────────────► 所有模块的时钟
#      │                      └──FCLK_RESET0_N──► 所有模块的复位
#      └──M_AXI_GP0──► AXI 互联 ──► vision_axi_ip.s (AXI-Lite 从口)
#                                          ▲
#                        tpg_source ───────┘ din_*（像素）
#
# 像素源先用片上图案发生器（见 rtl_bd/tpg_source.v 的说明）：它的几何是手算得
# 出来的，所以 PS 一能读回几何帧，立刻就能判断真硅上这条链对不对。
#
# ⚠ vision_axi_ip 是独立打包出来的 IP（bd_pack.tcl），**内部是原封不动的
#   vision_axi** —— 只改了时钟/复位名好让接口推断认得。功能一个字节没动。
#
# 用法：
#   vivado -mode batch -source bd_vision.tcl -log ... -tclargs \
#       <rtl_dir> <rtl_bd_dir> <out_dir> <tag>
#
# 产物：<tag>.bit / <tag>_util.rpt / <tag>_timing.rpt / <tag>_addr.rpt
# =============================================================================
set rtl_dir [lindex $argv 0]
set bd_dir  [lindex $argv 1]
set out_dir [lindex $argv 2]
set tag     [lindex $argv 3]

set ip_dir  $out_dir/ip_vision
if {![file exists $ip_dir/component.xml]} {
    puts "@@@ 找不到打包好的 IP（$ip_dir/component.xml），先跑 bd_pack.tcl @@"
    exit 1
}

# ---- 项目 ------------------------------------------------------------------
set prj $out_dir/bd_prj
file delete -force $prj
create_project -force bd_prj $prj -part xc7z030ffg676-2
set_property target_language Verilog [current_project]
# IP 仓库根的**父目录**：ip_vision/ 下就是 component.xml
set_property ip_repo_paths [list $out_dir] [current_project]
update_ip_catalog -rebuild

add_files -norecurse [glob $rtl_dir/*.v]
add_files -norecurse [glob $bd_dir/*.v]
set_property include_dirs "$rtl_dir $bd_dir" [current_fileset]
puts "@@@ 已加入 IP 仓库 $out_dir @@"

# ---- 块设计 ----------------------------------------------------------------
create_bd_design "sys"

# ① PS7，套用从厂商 demo 移植过来的配置
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_0
source {//wsl.localhost/Ubuntu/home/zhang/FPGA/core/fpga/synth/ps7_params.tcl}
set failed [apply_vendor_ps7 [get_bd_cells ps7_0]]
puts "@@@ PS7 参数失败 [llength $failed] 项 @@"

# ② AXI 互联：1 主 1 从
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] [get_bd_cells axi_ic]

# ③ 我们的视觉模块（独立打包的 IP）
create_bd_cell -type ip -vlnv user:ip:vision_axi_ip:1.0 vision_0
puts "@@@ vision_axi_ip 已实例化 @@"

# ④ 像素源（普通 RTL 模块）
create_bd_cell -type module -reference tpg_source tpg_0
puts "@@@ tpg_source 已实例化 @@"

# ---- 时钟 / 复位 -----------------------------------------------------------
set CLK     [get_bd_pins ps7_0/FCLK_CLK0]
set RST_RAW [get_bd_pins ps7_0/FCLK_RESET0_N]

# ⚠ PS7 **自己那些 AXI 端口的时钟**也必须驱动，否则 validate_bd_design 报
#   `BD 41-758 ... /ps7_0/M_AXI_GP0_ACLK /ps7_0/S_AXI_HP0_ACLK ... not connected`
#   （踩过一次：只连了互联和从设备的时钟，漏了 PS7 侧那一排。）
foreach p {ps7_0/M_AXI_GP0_ACLK ps7_0/M_AXI_GP1_ACLK \
           ps7_0/S_AXI_HP0_ACLK ps7_0/S_AXI_HP1_ACLK \
           ps7_0/S_AXI_HP2_ACLK ps7_0/S_AXI_HP3_ACLK} {
    connect_bd_net $CLK [get_bd_pins $p]
}

# ⚠ `FCLK_RESET0_N` 是**异步**复位，直接给 AXI 互联/从设备会报
#   `CRITICAL WARNING: [BD 41-1348] Reset pin ... connected to asynchronous
#    reset source ... may prevent design from meeting timing`
#   标准做法（每个 Zynq 块设计都这么干）是过一级 `proc_sys_reset`，
#   产生与 FCLK_CLK0 **同步**的复位。
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_0
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_1
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {1}] [get_bd_cells const_1]
connect_bd_net [get_bd_pins const_1/dout] [get_bd_pins rst_0/dcm_locked]
connect_bd_net $CLK     [get_bd_pins rst_0/slowest_sync_clk]
connect_bd_net $RST_RAW [get_bd_pins rst_0/ext_reset_in]

set RST [get_bd_pins rst_0/peripheral_aresetn]
foreach p {axi_ic/ACLK axi_ic/S00_ACLK axi_ic/M00_ACLK vision_0/s_axi_aclk tpg_0/clk} {
    connect_bd_net $CLK [get_bd_pins $p]
}
foreach p {axi_ic/ARESETN axi_ic/S00_ARESETN axi_ic/M00_ARESETN \
           vision_0/s_axi_aresetn tpg_0/rst_n} {
    connect_bd_net $RST [get_bd_pins $p]
}
puts "@@@ 时钟/复位已连（含 PS7 侧 AXI 端口时钟 + proc_sys_reset 同步复位）@@@"

# ---- AXI -------------------------------------------------------------------
connect_bd_intf_net [get_bd_intf_pins ps7_0/M_AXI_GP0] [get_bd_intf_pins axi_ic/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_ic/M00_AXI]  [get_bd_intf_pins vision_0/s]
# ⚠ 打包出来的 IP 默认把 AXI 接口声明成 100 MHz，而 FCLK_CLK0 是 50 MHz
#   （**时序就是在 50 MHz 上验过的**，改时钟等于把余量从 3 倍压到 1.5 倍，
#   没必要）。不匹配会报：
#     BD 41-237 Bus Interface property FREQ_HZ does not match between
#     /vision_0/s(100000000) and /axi_ic/.../M_AXI(50000000)
#   所以在 BD 侧把该 IP 实例的接口与时钟频率改成 50 MHz。
set_property CONFIG.FREQ_HZ {50000000} [get_bd_intf_pins vision_0/s]
catch {set_property CONFIG.FREQ_HZ {50000000} [get_bd_pins vision_0/s_axi_aclk]}
puts "@@@ AXI 已连（接口频率对齐到 50 MHz）@@"

# ---- 像素流 ----------------------------------------------------------------
foreach s {din_vld din_sof din_eol din_r din_g din_b} {
    connect_bd_net [get_bd_pins tpg_0/$s] [get_bd_pins vision_0/$s]
}
# ⚠ **不加任何额外的 PL 端口**。
#
# 原因：加了端口就要有 LOC/IOSTANDARD，而板子上我**只知道两个 LED 的引脚**
# （J1/M6，from 厂商 impl_constraints.xdc）。多出来的端口没处接，DRC 又会挡。
#
# PS 不需要这些信号也能判断链在跑：`vision_axi` 的控制/状态寄存器
# （0x0C0 的 bit0 busy / bit1 frame_done）就够 —— 帧在完成就说明
# tpg_source → vision_axi 这条像素路是通的。
#
# 于是本设计的外部端口**只有 PS7 自己的 DDR/FIXED_IO**，
# 它们是**专用引脚、由 PS7 自动约束**，DRC 自然通过。
puts "@@@ 像素流已连（不引出任何额外 PL 端口）@@"

# ---- 地址分配（自动）-------------------------------------------------------
assign_bd_address
set segs [get_bd_addr_segs]
puts "@@@ 地址段 @@"
foreach s $segs { puts "    $s  offset=[get_property offset $s]  range=[get_property range $s]" }
# ⚠ `report_bd_address` 是更新版本才有的命令，2018.3 里没有
#   （踩过：`invalid command name "report_bd_address"`，整个脚本在这里停住）。
#   地址已经在上面用 get_property 逐段打出来了，写进日志就够了。
#
# 记下基地址：**vision_axi 的寄存器基址 = 0x43C00000，窗口 64 KB**。
# PS 侧驱动（core/board/vision_driver）用的就是"基址 + 寄存器偏移"这套。

# ---- 包装与构建 ------------------------------------------------------------
# ⚠ **PS 的 DDR / FIXED_IO 必须显式引到顶层**。
#
# 踩过：不写这两句时 `make_wrapper` 产出的是 `module sys_wrapper ();`
# —— **一个端口都没有**。那样 DRC 反而"干净"（根本没有端口可违规），
# 但 PS 的 DDR/MIO 没有被设计 claim，很可能让 PS 拿不到 DDR、Linux 起不来。
# 厂商自己的 `ps_ai_wrapper.sv` 里是有 `DDR_*` / `FIXED_IO_*` 的。
if {[catch {make_bd_intf_pins_external [get_bd_intf_pins ps7_0/DDR]} e]} {
    puts "@@@ DDR 引外部失败（可能已经是外部）: $e @@"
}
if {[catch {make_bd_intf_pins_external [get_bd_intf_pins ps7_0/FIXED_IO]} e]} {
    puts "@@@ FIXED_IO 引外部失败（可能已经是外部）: $e @@"
}
puts "@@@ 顶层端口: [get_bd_ports] @@"

regenerate_bd_layout
validate_bd_design
save_bd_design
make_wrapper -files [get_files sys.bd] -top
# make_wrapper 把 wrapper 写在 **.srcs** 下（不是 .gen），glob 写死 .gen 会
# 报 `no files matched glob pattern`（踩过）。两种都试一下。
set wr [lindex [glob -nocomplain \
    $prj/*/sources_1/bd/sys/hdl/sys_wrapper.v \
    $prj/*.gen/sources_1/bd/sys/hdl/sys_wrapper.v] 0]
if {$wr eq ""} { puts "@@@ 找不到 sys_wrapper.v @@"; exit 1 }
add_files -norecurse $wr
set_property top sys_wrapper [current_fileset]
puts "@@@ wrapper: $wr @@"

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { puts "@@@ SYNTH_FAIL @@"; exit 1 }
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { puts "@@@ IMPL_FAIL @@"; exit 1 }

open_run impl_1
report_utilization    -file $out_dir/${tag}_util.rpt
report_timing_summary -file $out_dir/${tag}_timing.rpt
report_drc            -file $out_dir/${tag}_drc.rpt
write_bitstream -force $out_dir/$tag.bit
puts "@@@BD_BITSTREAM_OK $tag@@@"
exit
