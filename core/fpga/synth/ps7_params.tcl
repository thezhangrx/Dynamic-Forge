# =============================================================================
# 30TAI（JFMQL30TAI / 悟净 PSOC Lite）的 PS7 配置 —— 从厂商 demo 移植到 2018.3
#
# 源：fpai_demo_fpga/.../bd/ps_ai/ps_ai.bd（**Vivado 2020.2**）
# 目标：Vivado 2018.3 的 processing_system7 v5_5
#
# 用法：
#   create_ip -name processing_system7 -vendor xilinx.com -library ip \
#             -module_name ps7_0
#   source ps7_params.tcl
#   apply_vendor_ps7 [get_ips ps7_0]
#
# 逐条 set_property 并 catch：两个版本的参数集不完全一致，不这么写的话
# 一条失败就会让后面全部报 'failed due to earlier errors'，看不出真正原因。
#
# 实测（2026-10-14）：595 项里 556 项直接可用；剔掉 38 项只读的
# `PCW_MIO_*_IOTYPE`（v5_5 里由外设分配推出，不可设）后只剩
# `PCW_UART1_PERIPHERAL_ENABLE` 一条失败 —— UART1 走 EMIO，
# 要在块设计里补，不影响 MIO 上的 UART0 控制台（MIO 50..51）。
#
# 关键项已复核读回一致：
#   DDR  MT41K256M16 RE-125 / 32 Bit / 534 MHz      APU 800 MHz  晶振 33.333 MHz
#   M_AXI_GP0 = 1（PS 主控 PL）   S_AXI_HP0..3 = 1（PL 写 DDR）
#   FCLK0..3 = 50 MHz（FCLK0 正好给 vision_axi 当 clk）
#   ENET0 = 1 / MIO 16..27 / MDIO MIO 52..53 / 1000 Mbps
#   QSPI MIO 1..6、SD0 MIO 40..45、UART0 MIO 50..51
# =============================================================================
proc apply_vendor_ps7 {ip} {
    set failed {}
    set n 0
    if {[catch {set_property CONFIG.PCW_ACT_APU_PERIPHERAL_FREQMHZ "800.000000" $ip} e]} { lappend failed "PCW_ACT_APU_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_CAN0_PERIPHERAL_FREQMHZ "23.8095" $ip} e]} { lappend failed "PCW_ACT_CAN0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_CAN1_PERIPHERAL_FREQMHZ "23.8095" $ip} e]} { lappend failed "PCW_ACT_CAN1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_CAN_PERIPHERAL_FREQMHZ "100.000000" $ip} e]} { lappend failed "PCW_ACT_CAN_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_DCI_PERIPHERAL_FREQMHZ "10.158730" $ip} e]} { lappend failed "PCW_ACT_DCI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_ENET0_PERIPHERAL_FREQMHZ "125.000000" $ip} e]} { lappend failed "PCW_ACT_ENET0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_ENET1_PERIPHERAL_FREQMHZ "10.000000" $ip} e]} { lappend failed "PCW_ACT_ENET1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ "50.000000" $ip} e]} { lappend failed "PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ "10.000000" $ip} e]} { lappend failed "PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ "10.000000" $ip} e]} { lappend failed "PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_FPGA3_PERIPHERAL_FREQMHZ "10.000000" $ip} e]} { lappend failed "PCW_ACT_FPGA3_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_I2C_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_ACT_I2C_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_PCAP_PERIPHERAL_FREQMHZ "200.000000" $ip} e]} { lappend failed "PCW_ACT_PCAP_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_QSPI_PERIPHERAL_FREQMHZ "200.000000" $ip} e]} { lappend failed "PCW_ACT_QSPI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_SDIO_PERIPHERAL_FREQMHZ "50.000000" $ip} e]} { lappend failed "PCW_ACT_SDIO_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_SMC_PERIPHERAL_FREQMHZ "10.000000" $ip} e]} { lappend failed "PCW_ACT_SMC_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_SPI_PERIPHERAL_FREQMHZ "166.666672" $ip} e]} { lappend failed "PCW_ACT_SPI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TPIU_PERIPHERAL_FREQMHZ "200.000000" $ip} e]} { lappend failed "PCW_ACT_TPIU_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC0_CLK0_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC0_CLK0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC0_CLK1_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC0_CLK1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC0_CLK2_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC0_CLK2_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC1_CLK0_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC1_CLK0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC1_CLK1_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC1_CLK1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC1_CLK2_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_TTC1_CLK2_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_TTC_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_ACT_TTC_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_UART_PERIPHERAL_FREQMHZ "100.000000" $ip} e]} { lappend failed "PCW_ACT_UART_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_USB0_PERIPHERAL_FREQMHZ "60" $ip} e]} { lappend failed "PCW_ACT_USB0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_USB1_PERIPHERAL_FREQMHZ "60" $ip} e]} { lappend failed "PCW_ACT_USB1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ACT_WDT_PERIPHERAL_FREQMHZ "133.333344" $ip} e]} { lappend failed "PCW_ACT_WDT_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_APU_CLK_RATIO_ENABLE "6:2:1" $ip} e]} { lappend failed "PCW_APU_CLK_RATIO_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_APU_PERIPHERAL_FREQMHZ "800" $ip} e]} { lappend failed "PCW_APU_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_BASEADDR "0xE0008000" $ip} e]} { lappend failed "PCW_CAN0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_CAN0_IO "MIO 14 .. 15" $ip} e]} { lappend failed "PCW_CAN0_CAN0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_GRP_CLK_ENABLE "0" $ip} e]} { lappend failed "PCW_CAN0_GRP_CLK_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_HIGHADDR "0xE0008FFF" $ip} e]} { lappend failed "PCW_CAN0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_PERIPHERAL_CLKSRC "External" $ip} e]} { lappend failed "PCW_CAN0_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_CAN0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN1_PERIPHERAL_CLKSRC "External" $ip} e]} { lappend failed "PCW_CAN1_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN1_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_CAN1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_CAN_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN_PERIPHERAL_FREQMHZ "100" $ip} e]} { lappend failed "PCW_CAN_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CAN_PERIPHERAL_VALID "1" $ip} e]} { lappend failed "PCW_CAN_PERIPHERAL_VALID :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CLK0_FREQ "50000000" $ip} e]} { lappend failed "PCW_CLK0_FREQ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CLK1_FREQ "10000000" $ip} e]} { lappend failed "PCW_CLK1_FREQ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CLK2_FREQ "10000000" $ip} e]} { lappend failed "PCW_CLK2_FREQ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CLK3_FREQ "10000000" $ip} e]} { lappend failed "PCW_CLK3_FREQ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CORE0_FIQ_INTR "0" $ip} e]} { lappend failed "PCW_CORE0_FIQ_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CORE0_IRQ_INTR "0" $ip} e]} { lappend failed "PCW_CORE0_IRQ_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CORE1_FIQ_INTR "0" $ip} e]} { lappend failed "PCW_CORE1_FIQ_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CORE1_IRQ_INTR "0" $ip} e]} { lappend failed "PCW_CORE1_IRQ_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CPU_CPU_6X4X_MAX_RANGE "800" $ip} e]} { lappend failed "PCW_CPU_CPU_6X4X_MAX_RANGE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CPU_PERIPHERAL_CLKSRC "ARM PLL" $ip} e]} { lappend failed "PCW_CPU_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ "33.333333" $ip} e]} { lappend failed "PCW_CRYSTAL_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DCI_PERIPHERAL_CLKSRC "DDR PLL" $ip} e]} { lappend failed "PCW_DCI_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DCI_PERIPHERAL_FREQMHZ "10.159" $ip} e]} { lappend failed "PCW_DCI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DDR_PERIPHERAL_CLKSRC "DDR PLL" $ip} e]} { lappend failed "PCW_DDR_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DDR_RAM_BASEADDR "0x00100000" $ip} e]} { lappend failed "PCW_DDR_RAM_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DDR_RAM_HIGHADDR "0x3FFFFFFF" $ip} e]} { lappend failed "PCW_DDR_RAM_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DM_WIDTH "4" $ip} e]} { lappend failed "PCW_DM_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DQS_WIDTH "4" $ip} e]} { lappend failed "PCW_DQS_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_DQ_WIDTH "32" $ip} e]} { lappend failed "PCW_DQ_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_BASEADDR "0xE000B000" $ip} e]} { lappend failed "PCW_ENET0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_ENET0_IO "MIO 16 .. 27" $ip} e]} { lappend failed "PCW_ENET0_ENET0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_GRP_MDIO_ENABLE "1" $ip} e]} { lappend failed "PCW_ENET0_GRP_MDIO_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_GRP_MDIO_IO "MIO 52 .. 53" $ip} e]} { lappend failed "PCW_ENET0_GRP_MDIO_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_HIGHADDR "0xE000BFFF" $ip} e]} { lappend failed "PCW_ENET0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_ENET0_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_ENET0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_PERIPHERAL_FREQMHZ "1000 Mbps" $ip} e]} { lappend failed "PCW_ENET0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_RESET_ENABLE "1" $ip} e]} { lappend failed "PCW_ENET0_RESET_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET0_RESET_IO "MIO 49" $ip} e]} { lappend failed "PCW_ENET0_RESET_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET1_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_ENET1_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET1_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_ENET1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET_RESET_ENABLE "1" $ip} e]} { lappend failed "PCW_ENET_RESET_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET_RESET_POLARITY "Active Low" $ip} e]} { lappend failed "PCW_ENET_RESET_POLARITY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_ENET_RESET_SELECT "Share reset pin" $ip} e]} { lappend failed "PCW_ENET_RESET_SELECT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_4K_TIMER "0" $ip} e]} { lappend failed "PCW_EN_4K_TIMER :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CAN0 "1" $ip} e]} { lappend failed "PCW_EN_CAN0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CAN1 "0" $ip} e]} { lappend failed "PCW_EN_CAN1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLK0_PORT "1" $ip} e]} { lappend failed "PCW_EN_CLK0_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLK1_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLK1_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLK2_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLK2_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLK3_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLK3_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLKTRIG0_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLKTRIG0_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLKTRIG1_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLKTRIG1_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLKTRIG2_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLKTRIG2_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_CLKTRIG3_PORT "0" $ip} e]} { lappend failed "PCW_EN_CLKTRIG3_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_DDR "1" $ip} e]} { lappend failed "PCW_EN_DDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_CAN0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_CAN0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_CAN1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_CAN1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_CD_SDIO0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_CD_SDIO0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_CD_SDIO1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_CD_SDIO1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_ENET0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_ENET0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_ENET1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_ENET1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_GPIO "1" $ip} e]} { lappend failed "PCW_EN_EMIO_GPIO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_I2C0 "1" $ip} e]} { lappend failed "PCW_EN_EMIO_I2C0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_I2C1 "1" $ip} e]} { lappend failed "PCW_EN_EMIO_I2C1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_MODEM_UART0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_MODEM_UART0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_MODEM_UART1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_MODEM_UART1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_PJTAG "1" $ip} e]} { lappend failed "PCW_EN_EMIO_PJTAG :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_SDIO0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_SDIO0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_SDIO1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_SDIO1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_SPI0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_SPI0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_SPI1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_SPI1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_SRAM_INT "0" $ip} e]} { lappend failed "PCW_EN_EMIO_SRAM_INT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_TRACE "0" $ip} e]} { lappend failed "PCW_EN_EMIO_TRACE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_TTC0 "1" $ip} e]} { lappend failed "PCW_EN_EMIO_TTC0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_TTC1 "1" $ip} e]} { lappend failed "PCW_EN_EMIO_TTC1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_UART0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_UART0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_UART1 "1" $ip} e]} { lappend failed "PCW_EN_EMIO_UART1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_WDT "1" $ip} e]} { lappend failed "PCW_EN_EMIO_WDT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_WP_SDIO0 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_WP_SDIO0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_EMIO_WP_SDIO1 "0" $ip} e]} { lappend failed "PCW_EN_EMIO_WP_SDIO1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_ENET0 "1" $ip} e]} { lappend failed "PCW_EN_ENET0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_ENET1 "0" $ip} e]} { lappend failed "PCW_EN_ENET1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_GPIO "1" $ip} e]} { lappend failed "PCW_EN_GPIO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_I2C0 "1" $ip} e]} { lappend failed "PCW_EN_I2C0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_I2C1 "1" $ip} e]} { lappend failed "PCW_EN_I2C1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_MODEM_UART0 "0" $ip} e]} { lappend failed "PCW_EN_MODEM_UART0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_MODEM_UART1 "0" $ip} e]} { lappend failed "PCW_EN_MODEM_UART1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_PJTAG "1" $ip} e]} { lappend failed "PCW_EN_PJTAG :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_PTP_ENET0 "0" $ip} e]} { lappend failed "PCW_EN_PTP_ENET0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_PTP_ENET1 "0" $ip} e]} { lappend failed "PCW_EN_PTP_ENET1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_QSPI "1" $ip} e]} { lappend failed "PCW_EN_QSPI :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_RST0_PORT "1" $ip} e]} { lappend failed "PCW_EN_RST0_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_RST1_PORT "0" $ip} e]} { lappend failed "PCW_EN_RST1_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_RST2_PORT "0" $ip} e]} { lappend failed "PCW_EN_RST2_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_RST3_PORT "0" $ip} e]} { lappend failed "PCW_EN_RST3_PORT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_SDIO0 "1" $ip} e]} { lappend failed "PCW_EN_SDIO0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_SDIO1 "0" $ip} e]} { lappend failed "PCW_EN_SDIO1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_SMC "0" $ip} e]} { lappend failed "PCW_EN_SMC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_SPI0 "0" $ip} e]} { lappend failed "PCW_EN_SPI0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_SPI1 "1" $ip} e]} { lappend failed "PCW_EN_SPI1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_TRACE "0" $ip} e]} { lappend failed "PCW_EN_TRACE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_TTC0 "1" $ip} e]} { lappend failed "PCW_EN_TTC0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_TTC1 "1" $ip} e]} { lappend failed "PCW_EN_TTC1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_UART0 "1" $ip} e]} { lappend failed "PCW_EN_UART0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_UART1 "1" $ip} e]} { lappend failed "PCW_EN_UART1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_USB0 "1" $ip} e]} { lappend failed "PCW_EN_USB0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_USB1 "0" $ip} e]} { lappend failed "PCW_EN_USB1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_EN_WDT "1" $ip} e]} { lappend failed "PCW_EN_WDT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FCLK0_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_FCLK0_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FCLK1_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_FCLK1_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FCLK2_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_FCLK2_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FCLK3_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_FCLK3_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FCLK_CLK0_BUF "TRUE" $ip} e]} { lappend failed "PCW_FCLK_CLK0_BUF :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_FPGA0_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_FPGA1_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_FPGA2_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FPGA3_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_FPGA3_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_FPGA_FCLK0_ENABLE "1" $ip} e]} { lappend failed "PCW_FPGA_FCLK0_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP0_EN_MODIFIABLE_TXN "1" $ip} e]} { lappend failed "PCW_GP0_EN_MODIFIABLE_TXN :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP0_NUM_READ_THREADS "4" $ip} e]} { lappend failed "PCW_GP0_NUM_READ_THREADS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP0_NUM_WRITE_THREADS "4" $ip} e]} { lappend failed "PCW_GP0_NUM_WRITE_THREADS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP1_EN_MODIFIABLE_TXN "1" $ip} e]} { lappend failed "PCW_GP1_EN_MODIFIABLE_TXN :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP1_NUM_READ_THREADS "4" $ip} e]} { lappend failed "PCW_GP1_NUM_READ_THREADS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GP1_NUM_WRITE_THREADS "4" $ip} e]} { lappend failed "PCW_GP1_NUM_WRITE_THREADS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_BASEADDR "0xE000A000" $ip} e]} { lappend failed "PCW_GPIO_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE "1" $ip} e]} { lappend failed "PCW_GPIO_EMIO_GPIO_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_EMIO_GPIO_IO "64" $ip} e]} { lappend failed "PCW_GPIO_EMIO_GPIO_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH "64" $ip} e]} { lappend failed "PCW_GPIO_EMIO_GPIO_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_HIGHADDR "0xE000AFFF" $ip} e]} { lappend failed "PCW_GPIO_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_MIO_GPIO_ENABLE "1" $ip} e]} { lappend failed "PCW_GPIO_MIO_GPIO_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_MIO_GPIO_IO "MIO" $ip} e]} { lappend failed "PCW_GPIO_MIO_GPIO_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_GPIO_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_GPIO_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C0_BASEADDR "0xE0004000" $ip} e]} { lappend failed "PCW_I2C0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C0_HIGHADDR "0xE0004FFF" $ip} e]} { lappend failed "PCW_I2C0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C0_I2C0_IO "EMIO" $ip} e]} { lappend failed "PCW_I2C0_I2C0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_I2C0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C1_BASEADDR "0xE0005000" $ip} e]} { lappend failed "PCW_I2C1_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C1_HIGHADDR "0xE0005FFF" $ip} e]} { lappend failed "PCW_I2C1_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C1_I2C1_IO "EMIO" $ip} e]} { lappend failed "PCW_I2C1_I2C1_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C1_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_I2C1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C_PERIPHERAL_FREQMHZ "133.333328" $ip} e]} { lappend failed "PCW_I2C_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C_RESET_ENABLE "0" $ip} e]} { lappend failed "PCW_I2C_RESET_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_I2C_RESET_POLARITY "Active Low" $ip} e]} { lappend failed "PCW_I2C_RESET_POLARITY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_IMPORT_BOARD_PRESET "None" $ip} e]} { lappend failed "PCW_IMPORT_BOARD_PRESET :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_INCLUDE_ACP_TRANS_CHECK "0" $ip} e]} { lappend failed "PCW_INCLUDE_ACP_TRANS_CHECK :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_IRQ_F2P_INTR "1" $ip} e]} { lappend failed "PCW_IRQ_F2P_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_IRQ_F2P_MODE "DIRECT" $ip} e]} { lappend failed "PCW_IRQ_F2P_MODE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_0_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_0_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_0_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_0_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_10_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_10_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_10_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_10_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_11_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_11_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_11_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_11_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_12_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_12_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_12_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_12_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_13_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_13_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_13_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_13_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_14_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_14_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_14_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_14_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_15_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_15_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_15_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_15_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_16_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_16_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_16_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_16_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_17_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_17_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_17_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_17_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_18_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_18_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_18_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_18_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_19_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_19_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_19_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_19_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_1_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_1_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_1_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_1_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_20_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_20_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_20_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_20_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_21_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_21_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_21_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_21_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_22_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_22_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_22_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_22_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_23_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_23_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_23_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_23_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_24_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_24_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_24_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_24_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_25_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_25_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_25_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_25_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_26_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_26_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_26_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_26_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_27_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_27_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_27_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_27_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_28_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_28_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_28_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_28_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_29_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_29_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_29_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_29_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_2_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_2_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_30_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_30_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_30_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_30_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_31_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_31_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_31_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_31_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_32_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_32_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_32_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_32_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_33_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_33_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_33_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_33_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_34_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_34_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_34_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_34_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_35_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_35_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_35_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_35_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_36_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_36_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_36_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_36_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_37_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_37_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_37_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_37_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_38_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_38_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_38_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_38_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_39_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_39_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_39_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_39_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_3_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_3_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_40_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_40_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_40_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_40_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_41_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_41_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_41_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_41_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_42_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_42_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_42_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_42_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_43_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_43_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_43_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_43_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_44_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_44_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_44_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_44_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_45_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_45_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_45_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_45_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_46_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_46_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_46_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_46_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_47_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_47_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_47_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_47_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_48_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_48_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_48_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_48_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_49_PULLUP "disabled" $ip} e]} { lappend failed "PCW_MIO_49_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_49_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_49_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_4_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_4_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_50_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_50_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_50_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_50_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_51_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_51_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_51_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_51_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_52_PULLUP "disabled" $ip} e]} { lappend failed "PCW_MIO_52_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_52_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_52_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_53_PULLUP "disabled" $ip} e]} { lappend failed "PCW_MIO_53_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_53_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_53_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_5_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_5_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_6_SLEW "fast" $ip} e]} { lappend failed "PCW_MIO_6_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_7_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_7_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_8_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_8_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_9_PULLUP "enabled" $ip} e]} { lappend failed "PCW_MIO_9_PULLUP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_9_SLEW "slow" $ip} e]} { lappend failed "PCW_MIO_9_SLEW :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_MIO_PRIMITIVE "54" $ip} e]} { lappend failed "PCW_MIO_PRIMITIVE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP0_ENABLE_STATIC_REMAP "0" $ip} e]} { lappend failed "PCW_M_AXI_GP0_ENABLE_STATIC_REMAP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP0_ID_WIDTH "12" $ip} e]} { lappend failed "PCW_M_AXI_GP0_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP0_SUPPORT_NARROW_BURST "0" $ip} e]} { lappend failed "PCW_M_AXI_GP0_SUPPORT_NARROW_BURST :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP0_THREAD_ID_WIDTH "12" $ip} e]} { lappend failed "PCW_M_AXI_GP0_THREAD_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP1_ENABLE_STATIC_REMAP "0" $ip} e]} { lappend failed "PCW_M_AXI_GP1_ENABLE_STATIC_REMAP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP1_ID_WIDTH "12" $ip} e]} { lappend failed "PCW_M_AXI_GP1_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP1_SUPPORT_NARROW_BURST "0" $ip} e]} { lappend failed "PCW_M_AXI_GP1_SUPPORT_NARROW_BURST :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_M_AXI_GP1_THREAD_ID_WIDTH "12" $ip} e]} { lappend failed "PCW_M_AXI_GP1_THREAD_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_AR "1" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_AR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_CLR "1" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_CLR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_RC "11" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_RC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_REA "1" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_REA :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_RR "1" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_RR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_WC "11" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_WC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NAND_CYCLES_T_WP "1" $ip} e]} { lappend failed "PCW_NAND_CYCLES_T_WP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_CEOE "1" $ip} e]} { lappend failed "PCW_NOR_CS0_T_CEOE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_PC "1" $ip} e]} { lappend failed "PCW_NOR_CS0_T_PC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_RC "11" $ip} e]} { lappend failed "PCW_NOR_CS0_T_RC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_TR "1" $ip} e]} { lappend failed "PCW_NOR_CS0_T_TR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_WC "11" $ip} e]} { lappend failed "PCW_NOR_CS0_T_WC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_T_WP "1" $ip} e]} { lappend failed "PCW_NOR_CS0_T_WP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS0_WE_TIME "0" $ip} e]} { lappend failed "PCW_NOR_CS0_WE_TIME :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_CEOE "1" $ip} e]} { lappend failed "PCW_NOR_CS1_T_CEOE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_PC "1" $ip} e]} { lappend failed "PCW_NOR_CS1_T_PC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_RC "11" $ip} e]} { lappend failed "PCW_NOR_CS1_T_RC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_TR "1" $ip} e]} { lappend failed "PCW_NOR_CS1_T_TR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_WC "11" $ip} e]} { lappend failed "PCW_NOR_CS1_T_WC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_T_WP "1" $ip} e]} { lappend failed "PCW_NOR_CS1_T_WP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_CS1_WE_TIME "0" $ip} e]} { lappend failed "PCW_NOR_CS1_WE_TIME :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_CEOE "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_CEOE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_PC "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_PC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_RC "11" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_RC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_TR "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_TR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_WC "11" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_WC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_T_WP "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_T_WP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS0_WE_TIME "0" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS0_WE_TIME :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_CEOE "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_CEOE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_PC "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_PC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_RC "11" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_RC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_TR "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_TR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_WC "11" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_WC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_T_WP "1" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_T_WP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_NOR_SRAM_CS1_WE_TIME "0" $ip} e]} { lappend failed "PCW_NOR_SRAM_CS1_WE_TIME :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_OVERRIDE_BASIC_CLOCK "0" $ip} e]} { lappend failed "PCW_OVERRIDE_BASIC_CLOCK :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_CAN0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_CAN0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_ENET0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_ENET0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_GPIO_INTR "0" $ip} e]} { lappend failed "PCW_P2F_GPIO_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_I2C0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_I2C0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_I2C1_INTR "0" $ip} e]} { lappend failed "PCW_P2F_I2C1_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_QSPI_INTR "0" $ip} e]} { lappend failed "PCW_P2F_QSPI_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_SDIO0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_SDIO0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_SPI1_INTR "0" $ip} e]} { lappend failed "PCW_P2F_SPI1_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_UART0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_UART0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_UART1_INTR "0" $ip} e]} { lappend failed "PCW_P2F_UART1_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_P2F_USB0_INTR "0" $ip} e]} { lappend failed "PCW_P2F_USB0_INTR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY0 "0.096" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_BOARD_DELAY0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY1 "0.102" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_BOARD_DELAY1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY2 "0.100" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_BOARD_DELAY2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY3 "0.090" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_BOARD_DELAY3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_0 "-0.054" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_1 "-0.040" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_2 "-0.041" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_3 "-0.010" $ip} e]} { lappend failed "PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PACKAGE_NAME "ffg676" $ip} e]} { lappend failed "PCW_PACKAGE_NAME :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PCAP_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_PCAP_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PCAP_PERIPHERAL_FREQMHZ "200" $ip} e]} { lappend failed "PCW_PCAP_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PERIPHERAL_BOARD_PRESET "None" $ip} e]} { lappend failed "PCW_PERIPHERAL_BOARD_PRESET :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PJTAG_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_PJTAG_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PJTAG_PJTAG_IO "EMIO" $ip} e]} { lappend failed "PCW_PJTAG_PJTAG_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PLL_BYPASSMODE_ENABLE "0" $ip} e]} { lappend failed "PCW_PLL_BYPASSMODE_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PRESET_BANK0_VOLTAGE "LVCMOS 3.3V" $ip} e]} { lappend failed "PCW_PRESET_BANK0_VOLTAGE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PRESET_BANK1_VOLTAGE "LVCMOS 1.8V" $ip} e]} { lappend failed "PCW_PRESET_BANK1_VOLTAGE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_PS7_SI_REV "PRODUCTION" $ip} e]} { lappend failed "PCW_PS7_SI_REV :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE "0" $ip} e]} { lappend failed "PCW_QSPI_GRP_FBCLK_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_GRP_IO1_ENABLE "0" $ip} e]} { lappend failed "PCW_QSPI_GRP_IO1_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE "1" $ip} e]} { lappend failed "PCW_QSPI_GRP_SINGLE_SS_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_GRP_SINGLE_SS_IO "MIO 1 .. 6" $ip} e]} { lappend failed "PCW_QSPI_GRP_SINGLE_SS_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_GRP_SS1_ENABLE "0" $ip} e]} { lappend failed "PCW_QSPI_GRP_SS1_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_INTERNAL_HIGHADDRESS "0xFCFFFFFF" $ip} e]} { lappend failed "PCW_QSPI_INTERNAL_HIGHADDRESS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_QSPI_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_QSPI_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ "200" $ip} e]} { lappend failed "PCW_QSPI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_QSPI_QSPI_IO "MIO 1 .. 6" $ip} e]} { lappend failed "PCW_QSPI_QSPI_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD0_GRP_CD_ENABLE "0" $ip} e]} { lappend failed "PCW_SD0_GRP_CD_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD0_GRP_POW_ENABLE "0" $ip} e]} { lappend failed "PCW_SD0_GRP_POW_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD0_GRP_WP_ENABLE "0" $ip} e]} { lappend failed "PCW_SD0_GRP_WP_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_SD0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD0_SD0_IO "MIO 40 .. 45" $ip} e]} { lappend failed "PCW_SD0_SD0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SD1_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_SD1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SDIO0_BASEADDR "0xE0100000" $ip} e]} { lappend failed "PCW_SDIO0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SDIO0_HIGHADDR "0xE0100FFF" $ip} e]} { lappend failed "PCW_SDIO0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SDIO_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_SDIO_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_SDIO_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SDIO_PERIPHERAL_VALID "1" $ip} e]} { lappend failed "PCW_SDIO_PERIPHERAL_VALID :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SINGLE_QSPI_DATA_MODE "x4" $ip} e]} { lappend failed "PCW_SINGLE_QSPI_DATA_MODE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T0 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T1 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T2 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T3 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T4 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T4 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T5 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T5 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_CYCLE_T6 "NA" $ip} e]} { lappend failed "PCW_SMC_CYCLE_T6 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_SMC_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SMC_PERIPHERAL_VALID "0" $ip} e]} { lappend failed "PCW_SMC_PERIPHERAL_VALID :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI0_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_SPI0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_BASEADDR "0xE0007000" $ip} e]} { lappend failed "PCW_SPI1_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_GRP_SS1_ENABLE "0" $ip} e]} { lappend failed "PCW_SPI1_GRP_SS1_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_GRP_SS2_ENABLE "0" $ip} e]} { lappend failed "PCW_SPI1_GRP_SS2_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_HIGHADDR "0xE0007FFF" $ip} e]} { lappend failed "PCW_SPI1_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_SPI1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI1_SPI1_IO "MIO 10 .. 15" $ip} e]} { lappend failed "PCW_SPI1_SPI1_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_SPI_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI_PERIPHERAL_FREQMHZ "166.666666" $ip} e]} { lappend failed "PCW_SPI_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_SPI_PERIPHERAL_VALID "1" $ip} e]} { lappend failed "PCW_SPI_PERIPHERAL_VALID :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP0_DATA_WIDTH "64" $ip} e]} { lappend failed "PCW_S_AXI_HP0_DATA_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP0_ID_WIDTH "6" $ip} e]} { lappend failed "PCW_S_AXI_HP0_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP1_DATA_WIDTH "64" $ip} e]} { lappend failed "PCW_S_AXI_HP1_DATA_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP1_ID_WIDTH "6" $ip} e]} { lappend failed "PCW_S_AXI_HP1_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP2_DATA_WIDTH "64" $ip} e]} { lappend failed "PCW_S_AXI_HP2_DATA_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP2_ID_WIDTH "6" $ip} e]} { lappend failed "PCW_S_AXI_HP2_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP3_DATA_WIDTH "64" $ip} e]} { lappend failed "PCW_S_AXI_HP3_DATA_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_S_AXI_HP3_ID_WIDTH "6" $ip} e]} { lappend failed "PCW_S_AXI_HP3_ID_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TPIU_PERIPHERAL_CLKSRC "External" $ip} e]} { lappend failed "PCW_TPIU_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TRACE_INTERNAL_WIDTH "2" $ip} e]} { lappend failed "PCW_TRACE_INTERNAL_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TRACE_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_TRACE_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_BASEADDR "0xE0104000" $ip} e]} { lappend failed "PCW_TTC0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK0_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC0_CLK0_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK0_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC0_CLK0_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK1_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC0_CLK1_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK1_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC0_CLK1_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK2_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC0_CLK2_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_CLK2_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC0_CLK2_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_HIGHADDR "0xE0104fff" $ip} e]} { lappend failed "PCW_TTC0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_TTC0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC0_TTC0_IO "EMIO" $ip} e]} { lappend failed "PCW_TTC0_TTC0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_BASEADDR "0xE0105000" $ip} e]} { lappend failed "PCW_TTC1_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK0_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC1_CLK0_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK0_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC1_CLK0_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK1_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC1_CLK1_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK1_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC1_CLK1_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK2_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_TTC1_CLK2_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_CLK2_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_TTC1_CLK2_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_HIGHADDR "0xE0105fff" $ip} e]} { lappend failed "PCW_TTC1_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_TTC1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC1_TTC1_IO "EMIO" $ip} e]} { lappend failed "PCW_TTC1_TTC1_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_TTC_PERIPHERAL_FREQMHZ "50" $ip} e]} { lappend failed "PCW_TTC_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_BASEADDR "0xE0000000" $ip} e]} { lappend failed "PCW_UART0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_BAUD_RATE "115200" $ip} e]} { lappend failed "PCW_UART0_BAUD_RATE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_GRP_FULL_ENABLE "0" $ip} e]} { lappend failed "PCW_UART0_GRP_FULL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_HIGHADDR "0xE0000FFF" $ip} e]} { lappend failed "PCW_UART0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_UART0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART0_UART0_IO "MIO 50 .. 51" $ip} e]} { lappend failed "PCW_UART0_UART0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_BASEADDR "0xE0001000" $ip} e]} { lappend failed "PCW_UART1_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_BAUD_RATE "115200" $ip} e]} { lappend failed "PCW_UART1_BAUD_RATE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_GRP_FULL_ENABLE "0" $ip} e]} { lappend failed "PCW_UART1_GRP_FULL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_HIGHADDR "0xE0001FFF" $ip} e]} { lappend failed "PCW_UART1_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_UART1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART1_UART1_IO "EMIO" $ip} e]} { lappend failed "PCW_UART1_UART1_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART_PERIPHERAL_CLKSRC "IO PLL" $ip} e]} { lappend failed "PCW_UART_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART_PERIPHERAL_FREQMHZ "100" $ip} e]} { lappend failed "PCW_UART_PERIPHERAL_FREQMHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UART_PERIPHERAL_VALID "1" $ip} e]} { lappend failed "PCW_UART_PERIPHERAL_VALID :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_ACT_DDR_FREQ_MHZ "533.333374" $ip} e]} { lappend failed "PCW_UIPARAM_ACT_DDR_FREQ_MHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_ADV_ENABLE "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_ADV_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_AL "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_AL :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BL "8" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BL :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 "0.521" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BOARD_DELAY0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 "0.636" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BOARD_DELAY1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2 "0.54" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BOARD_DELAY2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3 "0.621" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BOARD_DELAY3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH "32 Bit" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_BUS_WIDTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_0_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_0_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PACKAGE_LENGTH "80.0515" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_0_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_0_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_1_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_1_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PACKAGE_LENGTH "80.0515" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_1_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_1_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_2_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_2_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PACKAGE_LENGTH "80.0515" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_2_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_2_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_3_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_3_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PACKAGE_LENGTH "80.0515" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_3_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_3_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_CLOCK_STOP_EN "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_CLOCK_STOP_EN :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_0_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_0_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_0_PACKAGE_LENGTH "133.8645" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_0_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_0_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_0_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_1_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_1_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_1_PACKAGE_LENGTH "119.997" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_1_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_1_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_1_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_2_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_2_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_2_PACKAGE_LENGTH "121.146" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_2_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_2_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_2_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_3_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_3_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_3_PACKAGE_LENGTH "90.125" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_3_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_3_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_3_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 "0.226" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 "0.278" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 "0.184" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 "0.309" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_0_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_0_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_0_PACKAGE_LENGTH "111.7605" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_0_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_0_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_0_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_1_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_1_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_1_PACKAGE_LENGTH "123.5525" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_1_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_1_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_1_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_2_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_2_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_2_PACKAGE_LENGTH "120.714" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_2_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_2_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_2_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_3_LENGTH_MM "0" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_3_LENGTH_MM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_3_PACKAGE_LENGTH "100.836" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_3_PACKAGE_LENGTH :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_DQ_3_PROPOGATION_DELAY "160" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_DQ_3_PROPOGATION_DELAY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_ENABLE "1" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_FREQ_MHZ "534" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_FREQ_MHZ :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_HIGH_TEMP "Normal (0-85)" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_HIGH_TEMP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_MEMORY_TYPE "DDR 3" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_MEMORY_TYPE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_PARTNO "MT41K256M16 RE-125" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_PARTNO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE "1" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_TRAIN_DATA_EYE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE "1" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_TRAIN_READ_GATE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL "1" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF "1" $ip} e]} { lappend failed "PCW_UIPARAM_DDR_USE_INTERNAL_VREF :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_UIPARAM_GENERATE_SUMMARY "NA" $ip} e]} { lappend failed "PCW_UIPARAM_GENERATE_SUMMARY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB0_BASEADDR "0xE0102000" $ip} e]} { lappend failed "PCW_USB0_BASEADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB0_HIGHADDR "0xE0102fff" $ip} e]} { lappend failed "PCW_USB0_HIGHADDR :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB0_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_USB0_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB0_USB0_IO "MIO 28 .. 39" $ip} e]} { lappend failed "PCW_USB0_USB0_IO :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB1_PERIPHERAL_ENABLE "0" $ip} e]} { lappend failed "PCW_USB1_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB_RESET_ENABLE "0" $ip} e]} { lappend failed "PCW_USB_RESET_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USB_RESET_POLARITY "Active Low" $ip} e]} { lappend failed "PCW_USB_RESET_POLARITY :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_AXI_FABRIC_IDLE "0" $ip} e]} { lappend failed "PCW_USE_AXI_FABRIC_IDLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_AXI_NONSECURE "0" $ip} e]} { lappend failed "PCW_USE_AXI_NONSECURE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_CORESIGHT "0" $ip} e]} { lappend failed "PCW_USE_CORESIGHT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_CROSS_TRIGGER "0" $ip} e]} { lappend failed "PCW_USE_CROSS_TRIGGER :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_CR_FABRIC "1" $ip} e]} { lappend failed "PCW_USE_CR_FABRIC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DDR_BYPASS "0" $ip} e]} { lappend failed "PCW_USE_DDR_BYPASS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DEBUG "0" $ip} e]} { lappend failed "PCW_USE_DEBUG :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DMA0 "0" $ip} e]} { lappend failed "PCW_USE_DMA0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DMA1 "0" $ip} e]} { lappend failed "PCW_USE_DMA1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DMA2 "0" $ip} e]} { lappend failed "PCW_USE_DMA2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_DMA3 "0" $ip} e]} { lappend failed "PCW_USE_DMA3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_EXPANDED_IOP "0" $ip} e]} { lappend failed "PCW_USE_EXPANDED_IOP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_FABRIC_INTERRUPT "1" $ip} e]} { lappend failed "PCW_USE_FABRIC_INTERRUPT :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_HIGH_OCM "0" $ip} e]} { lappend failed "PCW_USE_HIGH_OCM :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_M_AXI_GP0 "1" $ip} e]} { lappend failed "PCW_USE_M_AXI_GP0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_M_AXI_GP1 "1" $ip} e]} { lappend failed "PCW_USE_M_AXI_GP1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_PROC_EVENT_BUS "0" $ip} e]} { lappend failed "PCW_USE_PROC_EVENT_BUS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_PS_SLCR_REGISTERS "0" $ip} e]} { lappend failed "PCW_USE_PS_SLCR_REGISTERS :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_ACP "0" $ip} e]} { lappend failed "PCW_USE_S_AXI_ACP :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_GP0 "0" $ip} e]} { lappend failed "PCW_USE_S_AXI_GP0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_GP1 "0" $ip} e]} { lappend failed "PCW_USE_S_AXI_GP1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_HP0 "1" $ip} e]} { lappend failed "PCW_USE_S_AXI_HP0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_HP1 "1" $ip} e]} { lappend failed "PCW_USE_S_AXI_HP1 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_HP2 "1" $ip} e]} { lappend failed "PCW_USE_S_AXI_HP2 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_S_AXI_HP3 "1" $ip} e]} { lappend failed "PCW_USE_S_AXI_HP3 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_USE_TRACE "0" $ip} e]} { lappend failed "PCW_USE_TRACE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_VALUE_SILVERSION "3" $ip} e]} { lappend failed "PCW_VALUE_SILVERSION :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_WDT_PERIPHERAL_CLKSRC "CPU_1X" $ip} e]} { lappend failed "PCW_WDT_PERIPHERAL_CLKSRC :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_WDT_PERIPHERAL_DIVISOR0 "1" $ip} e]} { lappend failed "PCW_WDT_PERIPHERAL_DIVISOR0 :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_WDT_PERIPHERAL_ENABLE "1" $ip} e]} { lappend failed "PCW_WDT_PERIPHERAL_ENABLE :: $e" } else { incr n }
    if {[catch {set_property CONFIG.PCW_WDT_WDT_IO "EMIO" $ip} e]} { lappend failed "PCW_WDT_WDT_IO :: $e" } else { incr n }
    puts "PS7: 成功 $n 项，失败 [llength $failed] 项"
    foreach f $failed { puts "  PS7_FAIL $f" }
    return $failed
}
