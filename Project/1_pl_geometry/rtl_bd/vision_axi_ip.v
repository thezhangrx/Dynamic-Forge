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
// 所以这里**只做端口改名 + 把主口补成完整 AXI4**，一个逻辑都不加、不改：
//
//     clk     → s_axi_aclk
//     aresetn → s_axi_aresetn
//     从 `vision_axi` 的 m_araddr/m_rdata/... 补出标准 `m_axi_*` 全套
//
// ⚠ 为什么要把读主口补成"完整 AXI4"（2026-10-25，P0）：
//   正射要从 DDR 读相机帧，必须有一条 AXI 主口接 PS 的 `S_AXI_HP0`
//   （64 位 HP 口峰值 400 MB/s；PS 逐字推只有 4.4 M 字/s）。
//   块设计/`ipx::package_project` 只认**成套**的 aximm 信号
//   （aw*/w*/b*/ar*/r*），读到只有 ar*/r* 的半套会认不出接口。
//   本模块只**读**，所以写通道按"永不发起"接死：
//       awvalid=0, wvalid=0, bready=1
//   读通道的常量（AXI4 规定）：
//       arsize=3（8 字节/beat）、arburst=1（INCR）、arcache=4'b0011（Normal
//       Non-cacheable Bufferable）、arprot=3'b010（非安全、数据）、arid=0
//   ⚠ 这些常量**不是随手填的**：`arburst` 必须是 INCR（一次 240 beat 的连续行），
//     `arsize` 必须等于数据位宽（64 位 → 3）；写成别的值 PS 侧会拆成多次传输
//     或直接报 DECERR。
//
// ⚠ 这一层**不改变任何行为**：内部例化的就是原封不动的 `vision_axi`，
//   而那套逻辑是被 320 帧合成 + 32 帧真实照片逐位对账过的。
//   改名/补常量只影响**接口推断**，不影响功能 —— 这一点很重要，
//   因为"为了接进块设计而改了被验证过的 RTL"是不可接受的。
// ===========================================================================
`timescale 1ns / 1ps

module vision_axi_ip #(
    parameter integer WIDTH       = 640,
    parameter integer HEIGHT      = 360,
    parameter integer ADDR_W      = 12,
    parameter integer FRAME_BYTES = 2048,
    //: AXI 主口 ID 位宽：Zynq-7000 的 S_AXI_HP0 是 **6 位 ID 的 AXI3**，所以这里
    //: 给出 6 位；块设计里经 `axi_protocol_converter`（AXI4→AXI3）接 HP0。
    parameter integer MID_W       = 6
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

    // ---- AXI4 读主口（接 HP0；写通道接死）-------------------------------
    // AW
    output wire [MID_W-1:0]      m_axi_awid,
    output wire [31:0]           m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire [0:0]            m_axi_awlock,
    output wire [3:0]            m_axi_awcache,
    output wire [2:0]            m_axi_awprot,
    output wire [3:0]            m_axi_awqos,
    output wire                  m_axi_awvalid,
    input  wire                  m_axi_awready,
    // W
    output wire [63:0]           m_axi_wdata,
    output wire [7:0]            m_axi_wstrb,
    output wire                  m_axi_wlast,
    output wire                  m_axi_wvalid,
    input  wire                  m_axi_wready,
    // B
    input  wire [MID_W-1:0]      m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output wire                  m_axi_bready,
    // AR
    output wire [MID_W-1:0]      m_axi_arid,
    output wire [31:0]           m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire [0:0]            m_axi_arlock,
    output wire [3:0]            m_axi_arcache,
    output wire [2:0]            m_axi_arprot,
    output wire [3:0]            m_axi_arqos,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,
    // R
    input  wire [MID_W-1:0]      m_axi_rid,
    input  wire [63:0]           m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready,

    // 视频输入（像素流；P0 用不到，保留给 TPG/老路径）
    input  wire                  din_vld,
    input  wire                  din_sof,
    input  wire                  din_eol,
    input  wire [7:0]            din_r,
    input  wire [7:0]            din_g,
    input  wire [7:0]            din_b
);

    // ---- 核心只给这两组线，其余按 AXI4 规定填常量 -------------------------
    //: ⚠ 声明必须写在连续赋值之前（Verilog 的隐式网规则会悄悄建 1 位网）。
    wire [31:0] core_araddr;
    wire [7:0]  core_arlen;
    wire        core_arvalid;
    wire        core_arready;
    wire [63:0] core_rdata;
    wire        core_rlast;
    wire        core_rvalid;
    wire        core_rready;

    //: 读地址通道常量（理由见文件头）：id=0、size=3（8 字节）、INCR、
    //: cache=0011（Normal Non-cacheable Bufferable）、prot=010（非安全/数据）
    assign m_axi_arid    = {MID_W{1'b0}};
    assign m_axi_araddr  = core_araddr;
    assign m_axi_arlen   = core_arlen;
    assign m_axi_arsize  = 3'd3;
    assign m_axi_arburst = 2'd1;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b010;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_arvalid = core_arvalid;

    assign m_axi_rready  = core_rready;
    assign core_rdata    = m_axi_rdata;
    assign core_rlast    = m_axi_rlast;
    assign core_rvalid   = m_axi_rvalid;
    assign core_arready  = m_axi_arready;

    //: ⚠ `m_axi_rid`/`m_axi_rresp`/`m_axi_b*` 是**输入**，不需要（也不能）赋值：
    //:   本设计只有一条 outstanding、从不发写，所以 ID/RESP 没有语义。
    //:   留成"未使用的输入口"是刻意的 —— 谁要真想用，必须先想清楚乱序问题。

    // ---- 写通道：永不发起 ---------------------------------------------------
    assign m_axi_awid    = {MID_W{1'b0}};
    assign m_axi_awaddr  = 32'd0;
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'd3;
    assign m_axi_awburst = 2'd1;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b010;
    assign m_axi_awqos   = 4'd0;
    assign m_axi_awvalid = 1'b0;
    assign m_axi_wdata   = 64'd0;
    assign m_axi_wstrb   = 8'd0;
    assign m_axi_wlast   = 1'b0;
    assign m_axi_wvalid  = 1'b0;
    assign m_axi_bready  = 1'b1;              // 永远"能收 B"（不会有人给）

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
        .din_r(din_r), .din_g(din_g), .din_b(din_b),

        .m_araddr(core_araddr), .m_arlen(core_arlen), .m_arvalid(core_arvalid),
        .m_arready(core_arready),
        .m_rdata(core_rdata), .m_rlast(core_rlast), .m_rvalid(core_rvalid),
        .m_rready(core_rready)
    );

endmodule
