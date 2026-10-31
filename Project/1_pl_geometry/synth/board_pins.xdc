# 板级 pin/电平约束 —— 只用于 vision_board_top（bring-up 顶层）
#
# 来源：厂商 demo `board_bits/fpai_demo_fpga/constrs_1/impl_constraints.xdc`
#   sys_clk_p/n = C8 / C7, DIFF_SSTL15, 200 MHz（周期 5.000 ns）
#   led[0]      = J1, LVCMOS15
#   led[1]      = M6, LVCMOS15
#
# 本设计只有这 4 个端口，**全部有 LOC 与 IOSTANDARD** —— 这正是
# `vision_axi`（135 个端口、真实部署里接 PS7）出不了比特流的原因。
# 不做 DRC 降级：那会让 FPGA 驱动电平未定义的引脚。

set_property PACKAGE_PIN C8 [get_ports sys_clk_p]
set_property PACKAGE_PIN C7 [get_ports sys_clk_n]
set_property IOSTANDARD DIFF_SSTL15 [get_ports sys_clk_p]
set_property IOSTANDARD DIFF_SSTL15 [get_ports sys_clk_n]

set_property PACKAGE_PIN J1 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS15 [get_ports {led[0]}]
set_property PACKAGE_PIN M6 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS15 [get_ports {led[1]}]

create_clock -name sys_clk -period 5.000 [get_ports sys_clk_p]
