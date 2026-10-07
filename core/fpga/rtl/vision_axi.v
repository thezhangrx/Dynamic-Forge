// ===========================================================================
// vision_axi.v —— 视觉链路的 **PS 面向接口**（AXI4-Lite 从机 + 几何帧窗口）
//
// 这一层解决"PS 怎么把像素喂进来、怎么把几何帧读出去、怎么调阈值"。
// 它不碰算法：算法全在 vision_top / vision_ccl 里，且已被 320 帧逐位验证。
//
// 地址映射（字对齐；ADDR_W=12 → 4 KB 窗口）
// ---------------------------------------------------------------------------
//   0x000 + 4*i   (i = 0..20)  配置寄存器；i 就是 vision_top 的 cfg_addr
//                              （阈值/检测参数，字段顺序见 ref_api.h 的 VrRefParams）
//   0x080         写：调试选择 {4'b0, sub[3:0], idx[17:0], sel[3:0]}
//   0x084         读：调试数据（对应 vision_top 的 dbg_data）
//   0x0C0         读写控制/状态
//                 读：bit0 busy，bit1 上一帧已完成，[23:8] 完成帧计数，bit24 帧缓冲溢出
//                 写：bit0 = 1 → 启动一帧（自清）
//   0x0C4         写：seq（进几何帧头）
//   0x0C8/0x0CC   写：stamp_ns 的低/高 32 位
//   0x0D0         读：上一帧的**实际字节数**（PS 照这个长度读窗口）
//   0x100 + 4*j   读：几何帧字节窗口，第 j 个字（小端）
//
// 为什么用"BRAM 窗口"而不是 AXI-Stream + DMA：第一个 bring-up 的目标是
// **能看见东西**。窗口方案不依赖 DMA 描述符、不依赖中断、不依赖 Stream 握手，
// PS 侧就是"写寄存器 → 轮询 → 读窗口"。等链路点亮再换 DMA（见移植规划 §3）。
//
// **不会撕裂**：vision_top 每收到一次 frame_start 只处理一帧，然后回 IDLE。
// PS 的顺序是"写 start → 轮询 busy 落 → 读 frame_len → 读窗口"，读的时候
// 引擎已经停住，帧缓冲不再被写。这条是窗口方案能成立的前提，别改成连续采集。
//
// 时钟域：**单时钟**。像素输入必须与 clk 同步；真实相机的像素时钟与 clk
// 不同源时，跨时钟域 FIFO 在外层 —— 本模块不假装自己有 CDC。
//
// ⚠ 读写串行化：本从机在对方事务未完成时压低自己的 ready。裸机驱动按
// "写寄存器 → 轮询 → 读窗口"的顺序访问不受影响；接 DMA / 多主机时要改成
// 真正的双通道（AXI 允许并发读写，只是本实现没做）。
// ===========================================================================

module vision_axi #(
    parameter integer WIDTH       = 640,
    parameter integer HEIGHT      = 360,
    parameter integer ADDR_W      = 12,
    // 几何帧窗口大小。上限 2048：帧窗口下标用 addr[FW_BITS+1:2] 整段选取，
    // 窗口必须落在 0x100..0xFFF 这一段里（实测最长帧 1804 字节，够用）。
    parameter integer FRAME_BYTES = 2048
) (
    input  wire                  clk,
    input  wire                  aresetn,

    // ---- AXI4-Lite 从机 ----
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

    // ---- 像素输入（必须与 clk 同步；CDC 在外层）----
    input  wire                  din_vld,
    input  wire                  din_sof,
    input  wire                  din_eol,
    input  wire [7:0]            din_r,
    input  wire [7:0]            din_g,
    input  wire [7:0]            din_b
);

    localparam integer FB_BITS = $clog2(FRAME_BYTES);       // 2048 → 11
    localparam integer FW_BITS = FB_BITS - 2;               // 32 bit 字数 → 9

    localparam [11:0] ADDR_CONFIG = 12'h000;   // 0x000..0x07F 配置寄存器块
    localparam [11:0] ADDR_DBG    = 12'h080;   // 调试选择（写）
    localparam [11:0] ADDR_DBGDAT = 12'h084;   // 调试数据（读）
    localparam [11:0] ADDR_CTRL   = 12'h0C0;   // 控制/状态
    localparam [11:0] ADDR_SEQ    = 12'h0C4;
    localparam [11:0] ADDR_STAMP0 = 12'h0C8;
    localparam [11:0] ADDR_STAMP1 = 12'h0CC;
    localparam [11:0] ADDR_FLEN   = 12'h0D0;   // 只读：上一帧字节数
    localparam [11:0] ADDR_FRAME  = 12'h100;   // 几何帧窗口
    localparam [11:0] ADDR_FRAME_END = ADDR_FRAME + FRAME_BYTES[11:0];
    localparam [FB_BITS:0] FB_MAX    = FRAME_BYTES[FB_BITS:0];
    localparam [FW_BITS-1:0] FRAME_W0 = ADDR_FRAME[FW_BITS+1:2];   // 0x100>>2 = 64

    // ---- AXI 侧寄存器 ---------------------------------------------------
    reg              aw_v, w_v, b_v;
    reg [ADDR_W-1:0] aw_a;
    reg [31:0]       w_d;
    reg [3:0]        w_s;

    reg              ar_v, r_v;
    reg [ADDR_W-1:0] ar_a;
    reg [31:0]       rdata_q;

    // ---- vision_top 的配置口 -------------------------------------------
    reg        reg_we;
    reg [5:0]  reg_addr;
    reg [31:0] reg_wdata;
    reg        start_pulse;
    wire [5:0] cfg_addr;
    wire [31:0] cfg_rdata;

    // ---- 调试口 / 帧头 --------------------------------------------------
    reg [3:0]   dbg_sel;
    reg [17:0]  dbg_idx;
    reg [3:0]   dbg_sub;
    wire [31:0] dbg_data;
    reg [31:0]  cfg_seq_r;
    reg [63:0]  cfg_stamp_r;

    // ---- 几何帧缓冲（按 32 bit 字存，字节写用位选，读一次给一个字）------
    reg [31:0]      frame_mem [0:FRAME_BYTES/4-1];
    reg [FB_BITS:0] frame_wptr;
    reg [FB_BITS:0] frame_total;      // 不因窗口溢出而停，用来报真实帧长
    reg [FB_BITS:0] frame_len;
    reg             frame_ovf;

    wire        busy;
    wire        frame_done;
    wire [7:0]  dout_data;
    wire        dout_vld;
    wire        dout_last;

    // =====================================================================
    vision_top #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_vision (
        .pix_clk(clk), .rst_n(aresetn),
        .din_vld(din_vld), .din_sof(din_sof), .din_eol(din_eol),
        .din_r(din_r), .din_g(din_g), .din_b(din_b),
        .frame_start(start_pulse),
        .cfg_seq(cfg_seq_r), .cfg_stamp_ns(cfg_stamp_r),
        .frame_done(frame_done), .busy(busy),
        .cfg_we(reg_we), .cfg_addr(cfg_addr), .cfg_wdata(reg_wdata),
        .cfg_rdata(cfg_rdata),
        .dout_vld(dout_vld), .dout_data(dout_data),
        .dout_last(dout_last), .dout_ready(1'b1),
        .dbg_sel(dbg_sel), .dbg_idx(dbg_idx), .dbg_sub(dbg_sub),
        .dbg_data(dbg_data)
    );

    // =====================================================================
    // 握手：读写串行化
    // =====================================================================
    wire idle_all = !aw_v && !w_v && !b_v && !ar_v && !r_v;

    assign s_awready = idle_all;
    assign s_wready  = idle_all;
    assign s_arready = idle_all;
    assign s_bvalid  = b_v;
    assign s_bresp   = 2'b00;
    assign s_rvalid  = r_v;
    assign s_rresp   = 2'b00;
    assign s_rdata   = rdata_q;

    wire wr_commit = (aw_v && w_v && !b_v);
    wire ar_commit = (ar_v && !r_v);

    // 配置口地址：读数据那一拍用读地址；否则用写提交的地址。两者不并发。
    // CTRL(0x0C0) 的状态位住在 vision_top 的 CFG_STATUS(31) 里，读它的时候
    // 把 cfg_addr 借过去，不另外在本地再数一份帧计数（两份计数迟早会漂）。
    wire rd_cycle  = (ar_v || r_v);
    wire rd_conf   = rd_cycle && (ar_a < ADDR_CONFIG + 12'h080);   // 0x000..0x07F
    wire rd_status = rd_cycle && (ar_a == ADDR_CTRL);
    assign cfg_addr = rd_conf   ? {1'b0, ar_a[6:2]}
                    : rd_status ? 6'd31                  // CFG_STATUS
                    :             reg_addr;

    // AXI 字节写选通
    function [31:0] wstrb_merge(input [31:0] old, input [31:0] newd, input [3:0] strb);
        begin
            wstrb_merge[7:0]   = strb[0] ? newd[7:0]   : old[7:0];
            wstrb_merge[15:8]  = strb[1] ? newd[15:8]  : old[15:8];
            wstrb_merge[23:16] = strb[2] ? newd[23:16] : old[23:16];
            wstrb_merge[31:24] = strb[3] ? newd[31:24] : old[31:24];
        end
    endfunction

    // 几何帧窗口：一次给一个字（4 字节，小端）。
    // 下标直接用地址位段 a[FW_BITS+1:2] 减去窗口起始字号：正好 FW_BITS 位，
    // 既满足数组下标位宽，也不需要把 12 位地址整个塞进一个窄变量。
    function [31:0] frame_word(input [FW_BITS+1:2] a_word);
        begin
            frame_word = frame_mem[a_word - FRAME_W0];
        end
    endfunction

    // CTRL 读：借 cfg_rdata 拿 vision_top 的 [23:8]=帧计数 和 bit1=上一帧完成，
    // bit0 直接用 busy，bit24 是本地的窗口溢出。
    wire [31:0] ctrl_rd = {7'b0, frame_ovf, cfg_rdata[23:8], 6'b0, cfg_rdata[1], busy};

    always @(posedge clk) begin
        if (!aresetn) begin
            aw_v <= 0; w_v <= 0; b_v <= 0; aw_a <= 0; w_d <= 0; w_s <= 0;
            ar_v <= 0; r_v <= 0; ar_a <= 0; rdata_q <= 0;
            reg_we <= 0; reg_addr <= 0; reg_wdata <= 0; start_pulse <= 0;
            dbg_sel <= 0; dbg_idx <= 0; dbg_sub <= 0;
            cfg_seq_r <= 0; cfg_stamp_r <= 0;
            frame_wptr <= 0; frame_total <= 0; frame_len <= 0; frame_ovf <= 0;
        end else begin
            reg_we      <= 1'b0;      // 默认：写使能只持续一拍
            start_pulse <= 1'b0;

            // ---------------- 写通道 ----------------
            if (s_awvalid && s_awready) begin aw_a <= s_awaddr; aw_v <= 1'b1; end
            if (s_wvalid  && s_wready ) begin w_d  <= s_wdata;  w_s <= s_wstrb; w_v <= 1'b1; end

            if (wr_commit) begin
                aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1;
                if (aw_a < ADDR_CONFIG + 12'h080) begin
                    // 0x000..0x07F：配置寄存器块 → 透给 vision_top
                    reg_we    <= 1'b1;
                    reg_addr  <= {1'b0, aw_a[6:2]};
                    reg_wdata <= wstrb_merge(32'd0, w_d, w_s);
                end else case (aw_a)
                    ADDR_DBG: begin
                        dbg_sel <= w_d[3:0];
                        dbg_idx <= w_d[21:4];
                        dbg_sub <= w_d[25:22];
                    end
                    ADDR_CTRL:   if (w_d[0]) start_pulse <= 1'b1;
                    ADDR_SEQ:    cfg_seq_r          <= wstrb_merge(cfg_seq_r, w_d, w_s);
                    ADDR_STAMP0: cfg_stamp_r[31:0]  <= wstrb_merge(cfg_stamp_r[31:0],  w_d, w_s);
                    ADDR_STAMP1: cfg_stamp_r[63:32] <= wstrb_merge(cfg_stamp_r[63:32], w_d, w_s);
                    default: ;                     // 未映射地址：忽略
                endcase
            end else if (b_v && s_bready) begin
                b_v <= 1'b0;
            end

            // ---------------- 读通道 ----------------
            // 一拍收地址、下一拍给数据：配置寄存器的读回是组合的，地址必须稳住。
            if (s_arvalid && s_arready) begin
                ar_a <= s_araddr; ar_v <= 1'b1;
            end else if (ar_commit) begin
                ar_v <= 1'b0; r_v <= 1'b1;
                if (ar_a < ADDR_CONFIG + 12'h080)   rdata_q <= cfg_rdata;
                else case (ar_a)
                    ADDR_DBGDAT: rdata_q <= dbg_data;
                    ADDR_CTRL:   rdata_q <= ctrl_rd;
                    ADDR_FLEN:   rdata_q <= {{(32-FB_BITS-1){1'b0}}, frame_len};
                    ADDR_SEQ:    rdata_q <= cfg_seq_r;
                    ADDR_STAMP0: rdata_q <= cfg_stamp_r[31:0];
                    ADDR_STAMP1: rdata_q <= cfg_stamp_r[63:32];
                    default:     rdata_q <= (ar_a >= ADDR_FRAME &&
                                             ar_a <  ADDR_FRAME_END)
                                            ? frame_word(ar_a[FW_BITS+1:2]) : 32'd0;
                endcase
            end else if (r_v && s_rready) begin
                r_v <= 1'b0;
            end

            // ---------------- 几何帧缓冲 ----------------
            // frame_done 比 dout_last 晚一拍（vision_top 在 S_DONE 拉高），
            // 所以这里 frame_wptr 已经是整帧字节数。
            if (frame_done) frame_wptr <= 0;     // 收完一帧，窗口从头开始
            if (start_pulse) frame_ovf <= 1'b0;
            // 真实帧长在**最后一个字节**那一拍定：frame_total 不管窗口够不够
            // 都在数，所以窗口太小时 PS 也能看出"长度超了窗口"。
            if (dout_vld && dout_last) frame_len <= frame_total + 1'b1;
            if (dout_vld) begin
                frame_total <= dout_last ? {FB_BITS+1{1'b0}} : (frame_total + 1'b1);
                if (frame_wptr < FB_MAX) begin
                    case (frame_wptr[1:0])
                        2'd0: frame_mem[frame_wptr[FB_BITS-1:2]][7:0]   <= dout_data;
                        2'd1: frame_mem[frame_wptr[FB_BITS-1:2]][15:8]  <= dout_data;
                        2'd2: frame_mem[frame_wptr[FB_BITS-1:2]][23:16] <= dout_data;
                        2'd3: frame_mem[frame_wptr[FB_BITS-1:2]][31:24] <= dout_data;
                    endcase
                    frame_wptr <= frame_wptr + 1'b1;
                end else begin
                    frame_ovf <= 1'b1;         // 窗口太小：整帧被截断，PS 能看到
                end
            end
        end
    end

endmodule
