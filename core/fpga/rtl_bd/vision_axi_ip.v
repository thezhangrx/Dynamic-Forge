// ===========================================================================
// vision_axi_ip.v —— `vision_axi` 的**块设计包装**
//
// 为什么要有这一层
// ----------------
// 块设计（IP Integrator）要消费一个 Verilog 模块，需要能**认出它的总线接口**：
// AXI-Lite 从口的端口名（`s_awaddr` / `s_wvalid` / `s_rdata` …）本来就符合
// 约定，但**时钟与复位必须叫 `s_axi_aclk` / `s_axi_aresetn`**，否则
// `ipx::infer_bus_interfaces` 认不出这是 AXI，块设计里就没有接口引脚可连，
// 只能一个信号一个信号地手工接。
//
// 所以这里**只做端口改名**，一个逻辑都不加、不改：
//
//     clk     → s_axi_aclk
//     aresetn → s_axi_aresetn
//
// 其余端口（`s_*` 总线、`din_*` 视频输入）原样透传。
//
// ⚠ 这一层**不改变任何行为**：内部例化的就是原封不动的 `vision_axi`，
//   而那套逻辑是被 320 帧合成 + 32 帧真实照片逐位对账过的。
//   改名只影响**接口推断**，不影响功能 —— 这一点很重要，
//   因为"为了接进块设计而改了被验证过的 RTL"是不可接受的。
// ===========================================================================
`timescale 1ns / 1ps

module vision_axi_ip #(
    parameter integer WIDTH       = 640,
    parameter integer HEIGHT      = 360,
    parameter integer ADDR_W      = 12,
    parameter integer FRAME_BYTES = 2048
) (
    // AXI-Lite 从口（时钟/复位按 AXI 约定命名，好让接口推断认得）
    input  wire                  s_axi_aclk,
    input  wire                  s_axi_aresetn,

    input  wire [ADDR_W-1:0]     s_awaddr,
    input  wire                  s_awvalid,
    output wire                  s_awready,
    input  wire [31:0]           s_wdata,
    input  wire [3:0]            s_wstrb,
    input  wire                  s_wvalid,
    output wire                  s_wready,
    output wire [1:0]            s_bresp,
    output wire                  s_bvalid,
    input  wire                  s_bready,
    input  wire [ADDR_W-1:0]     s_araddr,
    input  wire                  s_arvalid,
    output wire                  s_arready,
    output wire [31:0]           s_rdata,
    output wire [1:0]            s_rresp,
    output wire                  s_rvalid,
    input  wire                  s_rready,

    // 视频输入（像素流）
    input  wire                  din_vld,
    input  wire                  din_sof,
    input  wire                  din_eol,
    input  wire [7:0]            din_r,
    input  wire [7:0]            din_g,
    input  wire [7:0]            din_b
);

    vision_axi #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT),
        .ADDR_W(ADDR_W), .FRAME_BYTES(FRAME_BYTES)
    ) u_core (
        .clk(s_axi_aclk), .aresetn(s_axi_aresetn),

        .s_awaddr(s_awaddr), .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata),   .s_wstrb(s_wstrb),     .s_wvalid(s_wvalid),
        .s_wready(s_wready), .s_bresp(s_bresp),     .s_bvalid(s_bvalid),
        .s_bready(s_bready),
        .s_araddr(s_araddr), .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rdata(s_rdata),   .s_rresp(s_rresp),     .s_rvalid(s_rvalid),
        .s_rready(s_rready),

        .din_vld(din_vld), .din_sof(din_sof), .din_eol(din_eol),
        .din_r(din_r), .din_g(din_g), .din_b(din_b)
    );

endmodule
