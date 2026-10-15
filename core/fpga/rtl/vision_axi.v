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
//   0x0D4         写：像素 FIFO 推入（每个 32 位字 = 4 个打包字节，**整字写**）
//   0x0D8         读：[15:0] 还在缓冲的字节数，[16] 溢出（粘滞），[17] 这一帧还在放
//   0x0DC         写：bit0 = 像素源（0 = din_* 外部/TPG，1 = PS 的 FIFO）
//                       bit1 = flush（清空 FIFO 与 ovf）
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
    // ---- PS 喂像素（真摄像头）：见 vision_pixfeed.v ----------------------
    localparam [11:0] ADDR_PIXPUSH = 12'h0D4;  // 写：推一个 32 位字（4 个打包字节）
    localparam [11:0] ADDR_PIXSTAT = 12'h0D8;  // 读：{ovf, level[15:0]}
    localparam [11:0] ADDR_PIXCTL  = 12'h0DC;  // 写：bit0=像素源(0=din_*外部,1=FIFO) bit1=flush
    localparam [11:0] ADDR_FRAME  = 12'h100;   // 几何帧窗口
    localparam [11:0] ADDR_FRAME_END = ADDR_FRAME + FRAME_BYTES[11:0];
    localparam [FB_BITS:0] FB_MAX    = FRAME_BYTES[FB_BITS:0];

    // ---- 字地址（地址 >> 2）-------------------------------------------------
    // ⚠ 译码必须按**字地址**，不能拿完整字节地址去比字对齐的常量。
    //
    // AXI 允许窄传输的地址不对齐：主控对 0x0C5 做一次字节写，给的是
    // `AWADDR=0x0C5, AWSIZE=0, WSTRB=0x2`（合法且常见 —— Zynq 的 GP 口对
    // `*(volatile uint8_t*)` 就是这么发的）。原来 `case (aw_a)` 拿 0x0C5 去比
    // `ADDR_SEQ=0x0C4`，比不中 → 落进 `default` → **整笔写被静默丢弃**。
    // 读侧同样：读 0x0C5 会掉进 default 返回 0，而不是 CTRL 的值。
    //
    // 配置块（0x000..0x07F）反而一直是对的：它是**范围判断 + aw_a[6:2]**，
    // 天然忽略了低 2 位。所以同一个从口上两种译码语义并存。
    //
    // 2026-10-15 上板第一次真跑才暴露（axi_soak 的 Phase D）：
    //   配置块 0x044 逐字节/半字写 → 全对；
    //   SEQ 0x0C4 / STAMP0 0x0C8 / STAMP1 0x0CC → 只有 lane0 写得进去。
    wire [ADDR_W-3:0] aw_word = aw_a[ADDR_W-1:2];
    wire [ADDR_W-3:0] ar_word = ar_a[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_DBG      = ADDR_DBG[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_DBGDAT   = ADDR_DBGDAT[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_CTRL     = ADDR_CTRL[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_SEQ      = ADDR_SEQ[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_STAMP0   = ADDR_STAMP0[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_STAMP1   = ADDR_STAMP1[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FLEN     = ADDR_FLEN[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXPUSH  = ADDR_PIXPUSH[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXSTAT  = ADDR_PIXSTAT[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXCTL   = ADDR_PIXCTL[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRAME    = ADDR_FRAME[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRAME_END = ADDR_FRAME_END[ADDR_W-1:2];
    // 配置块的窗口边界保持用**字节地址**判（ADDR_CONFIG+0x080 是字对齐的，
    // 对任何窄地址都等价于"字地址 < 32"），这样 aw_a/ar_a 的低 2 位也有用途。

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
    reg [31:0] reg_wdata;      // 原始 s_wdata（不做选通合并）
    reg [3:0]  reg_strb;       // 随路锁存的 s_wstrb
    reg        start_pulse;
    wire [5:0] cfg_addr;
    wire [31:0] cfg_rdata;
    wire [31:0] cfg_wdata;     // = wstrb_merge(cfg_rdata, reg_wdata, reg_strb)

    // ---- 调试口 / 帧头 --------------------------------------------------
    reg [3:0]   dbg_sel;
    reg [17:0]  dbg_idx;
    reg [3:0]   dbg_sub;
    wire [31:0] dbg_data;
    reg [31:0]  cfg_seq_r;
    reg [63:0]  cfg_stamp_r;

    // ---- PS 喂像素（真摄像头那条路）--------------------------------------
    reg         pix_src;          // 0 = din_*（外部/TPG），1 = PS 的 FIFO
    reg         pix_push;         // 一拍脉冲
    reg         pix_flush;        // 一拍脉冲
    wire [15:0] pix_buffered;
    wire        pix_ovf;
    wire        pf_vld, pf_sof, pf_eol;
    wire [7:0]  pf_r, pf_g, pf_b;
    wire        pix_busy;

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
    // PS 喂像素：真摄像头采到的帧在 PS 手上，经 AXI 写进 FIFO，这里还原成
    // 像素时序。见 vision_pixfeed.v（含"为什么不需要 PS 等 FIFO 不满"）。
    // =====================================================================
    wire pf_sel = pix_src;      // 0 = 用 din_*（外部/TPG），1 = 用 PS 的 FIFO

    vision_pixfeed #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_pixfeed (
        .clk(clk), .rst_n(aresetn),
        .push(pix_push), .push_data(w_masked32),
        // `level`（FIFO 里的字数）是纯诊断量，对外报的是 `buffered`（字节数，
        // 含蓄水池），所以这里不接 —— 留着悬空，避免"声明了却没人用"的 lint 告警。
        .flush(pix_flush), .buffered(pix_buffered), .ovf(pix_ovf),
        .sink_ready(busy),      // 见 vision_pixfeed.v：busy 恰在进入 S_RECV 时拉高
        .dout_vld(pf_vld), .dout_sof(pf_sof), .dout_eol(pf_eol),
        .dout_r(pf_r), .dout_g(pf_g), .dout_b(pf_b),
        .busy(pix_busy)
    );

    // 像素源二选一（**逐条对应**：vld/sof/eol/r/g/b 必须整体切换，不能混）
    wire        src_vld = pf_sel ? pf_vld : din_vld;
    wire        src_sof = pf_sel ? pf_sof : din_sof;
    wire        src_eol = pf_sel ? pf_eol : din_eol;
    wire [7:0]  src_r   = pf_sel ? pf_r   : din_r;
    wire [7:0]  src_g   = pf_sel ? pf_g   : din_g;
    wire [7:0]  src_b   = pf_sel ? pf_b   : din_b;
    vision_top #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_vision (
        .pix_clk(clk), .rst_n(aresetn),
        .din_vld(src_vld), .din_sof(src_sof), .din_eol(src_eol),
        .din_r(src_r), .din_g(src_g), .din_b(src_b),
        .frame_start(start_pulse),
        .cfg_seq(cfg_seq_r), .cfg_stamp_ns(cfg_stamp_r),
        .frame_done(frame_done), .busy(busy),
        .cfg_we(reg_we), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata(cfg_rdata),
        .dout_vld(dout_vld), .dout_data(dout_data),
        .dout_last(dout_last), .dout_ready(1'b1),
        .dbg_sel(dbg_sel), .dbg_idx(dbg_idx), .dbg_sub(dbg_sub),
        .dbg_data(dbg_data)
    );

    // =====================================================================
    // 握手：AW 与 W 必须能**分开**接收
    //
    // ⚠ 原实现把三个 ready 全绑在 `idle_all` 上，会**死锁**：
    //     主控给出 AW → `s_awready = idle_all = 1` → `aw_v <= 1`
    //     下一拍 `idle_all` 立刻变 0 → `s_wready` 跟着掉
    //     → `w_v` **永远捕捉不到** → `wr_commit = aw_v && w_v && !b_v`
    //       永不成立 → `bvalid` 永不返回 → **AXI 写总线挂死、CPU 卡住**
    //
    //   AXI 协议**允许**主控先给 AW、再给 W（合法且常见），所以这不是
    //   "主控不规范"，是实现错了。
    //
    //   为什么协同仿真没抓到：`cosim/tb_axi.cpp` 的 `axi_write()` 恰好总是
    //   **同一拍**把 AW 和 W 一起给出 —— 走不到这条路径，11490 项全绿也说明
    //   不了问题。**这是把"主控行为"理想化造成的结构性盲区**，
    //   已给 tb 加 `--w-delay`（默认 1 拍）覆盖，见 tb_axi.cpp。
    //
    //   2026-10-14 在真硅上暴露：读通路完全正常（vision_axi 的配置寄存器读出
    //   默认值 40/110/60/195/45/10/90/5，逐项与 VR_DEFAULT_REF_PARAMS 一致），
    //   **一写就挂**。见 core/fpga/docs/板上验证.md §4。
    //
    // 修法：AW/W 各自只看自己那一半；都不与读并发（保持原串行化意图）。
    //   **没有循环依赖**：aw_v/w_v 只依赖自己的 ready，ar_v 依赖全部。
    // =====================================================================
    wire rd_idle = !ar_v && !r_v;
    wire wr_idle = !b_v;

    assign s_awready = !aw_v && wr_idle && rd_idle;
    assign s_wready  = !w_v  && wr_idle && rd_idle;
    assign s_arready = !ar_v && rd_idle && !aw_v && !w_v && !b_v;
    assign s_bvalid  = b_v;
    assign s_bresp   = 2'b00;
    assign s_rvalid  = r_v;
    assign s_rresp   = 2'b00;
    assign s_rdata   = rdata_q;

    wire wr_commit = (aw_v && w_v && !b_v);
    wire ar_commit = (ar_v && !r_v);

    // 配置口地址：写提交那一拍**写地址优先**；读数据那一拍用读地址。
    //
    // ⚠ 第四个"测试台把主控理想化"的盲区（前三个：AW/W 同拍、WSTRB 恒 0xF、
    //   地址恒字对齐）。tb 一次只发一笔事务，所以从没走到**读写同拍**这条路。
    //   AXI **允许**主控在同一拍同时给出 AWVALID 和 ARVALID（从口可以节流，
    //   但**不能写错寄存器**）。而 s_awready/s_arready 只看**已寄存的** aw_v/ar_v，
    //   所以两者同拍时都会被接收。后果（`reg_we=1` 那一拍 rd_cycle 仍为真）：
    //     · 写配置寄存器 A + 同时读配置寄存器 B(≠A) → cfg_addr 被读地址抢走
    //       → **写静默落进了 B**（上板/仿真都测得到）；
    //     · 写配置寄存器 + 同时读 CTRL → cfg_addr=31(CFG_STATUS) → **写被丢弃**；
    //     · 窄写时 cfg_wdata 的 WSTRB 合并还会读到**错寄存器的旧值**。
    //   注意写**非配置块**寄存器（SEQ/STAMP/CTRL…）不会拉高 cfg_we，
    //   所以那种组合本来就无害 —— 这一点是反向验证时确认的。
    //   修法：reg_we 那一拍无条件用写地址。读数据是在 ar_commit 那一拍采的，
    //   那时 reg_we 还是 0（reg_we 比 commit 晚一拍），所以读不受影响。
    // CTRL(0x0C0) 的状态位住在 vision_top 的 CFG_STATUS(31) 里，读它的时候
    // 把 cfg_addr 借过去，不另外在本地再数一份帧计数（两份计数迟早会漂）。
    wire rd_cycle  = (ar_v || r_v);
    wire rd_conf   = rd_cycle && (ar_a < ADDR_CONFIG + 12'h080);   // 0x000..0x07F
    wire rd_status = rd_cycle && (ar_word == W_CTRL);
    assign cfg_addr = reg_we    ? reg_addr               // 写提交那一拍：写优先
                    : rd_conf   ? {1'b0, ar_a[6:2]}
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

    // ---- 写数据的两条选通路径 -------------------------------------------
    //
    // ⚠ 这里修的是**第二个**"主控行为被理想化"的盲区（第一个是 AW/W 时序，见上）。
    //   `wstrb_merge(32'd0, ...)` 会把**未被选通的字节变成 0**。配置寄存器块
    //   （0x000..0x07F）在 vision_top 里是**整字写**（`cfg_addr` 选中的那个寄存器
    //   整体被 `cfg_wdata` 覆盖），所以只要主控用字节/半字写配置寄存器，其余
    //   三条字节就被静默清零 —— 例如对 0x000 写 1 字节，`r_wall_rb` 之外的
    //   `r_wall_min_r/g` 会一起变 0。SEQ/STAMP 那三个寄存器是就地 `wstrb_merge`，
    //   反而是对的，于是**同一个 AXI-Lite 从口上两种字节语义并存**。
    //
    //   修法：写提交时只锁存原始 `w_d`/`w_s`，下一拍（`reg_we=1` 那一拍）
    //   先读该寄存器的**旧值**再合并：
    //       cfg_wdata = wstrb_merge(cfg_rdata, reg_wdata, reg_strb)
    //   没有循环依赖（cfg_rdata ← cfg_addr ← 寄存器；cfg_wdata 不被 cfg_rdata
    //   之外的组合逻辑回头依赖），也没有时序竞争：仲裁保证读写**严格串行**
    //   （s_arready 要求 !aw_v&&!w_v&&!b_v，s_awready/s_wready 要求 rd_idle），
    //   所以 `reg_we=1` 那一拍不可能有读在飞，cfg_addr 必然等于 reg_addr、
    //   cfg_rdata 必然是该寄存器的旧值。
    //   整字写（strb=4'hF）下 `wstrb_merge` 恒等，行为与原来完全一致。
    //
    // DBG/CTRL 的写数据只用到低 26 位（sel[3:0] / idx[21:4] / sub[25:22]），
    // 高 6 位保留。单独做一个 26 位宽的选通合并，免得 -Wall 的 UNUSEDSIGNAL
    // 拿"w_masked[31:26] 没被读"报错。
    function [25:0] wstrb_merge26(input [25:0] old, input [25:0] newd, input [3:0] strb);
        begin
            wstrb_merge26[7:0]   = strb[0] ? newd[7:0]   : old[7:0];
            wstrb_merge26[15:8]  = strb[1] ? newd[15:8]  : old[15:8];
            wstrb_merge26[23:16] = strb[2] ? newd[23:16] : old[23:16];
            wstrb_merge26[25:24] = strb[3] ? newd[25:24] : old[25:24];
        end
    endfunction
    // 32 位选通掩码版：PIXPUSH 要用**全 32 位**（一个像素的 4 个打包字节都在这儿）
    wire [31:0] w_masked32 = wstrb_merge(32'd0, w_d, w_s);
    wire [25:0] w_masked   = w_masked32[25:0];
    assign cfg_wdata = wstrb_merge(cfg_rdata, reg_wdata, reg_strb);

    // 几何帧窗口：一次给一个字（4 字节，小端）。
    // 下标直接用地址位段 a[FW_BITS+1:2] 减去窗口起始字号：正好 FW_BITS 位，
    // 既满足数组下标位宽，也不需要把 12 位地址整个塞进一个窄变量。
    // 参数是**帧内字号**（不是绝对字地址）：这样调用处写 `ar_word - W_FRAME`，
    // 不必对一个 10 位的字地址做 [FW_BITS+1:2] 这种越界位选（-Wall 会报 SELRANGE）。
    function [31:0] frame_word(input [FW_BITS-1:0] widx);
        begin
            frame_word = frame_mem[widx];
        end
    endfunction

    // CTRL 读：借 cfg_rdata 拿 vision_top 的 [23:8]=帧计数 和 bit1=上一帧完成，
    // bit0 直接用 busy，bit24 是本地的窗口溢出。
    wire [31:0] ctrl_rd = {7'b0, frame_ovf, cfg_rdata[23:8], 6'b0, cfg_rdata[1], busy};

    always @(posedge clk) begin
        if (!aresetn) begin
            aw_v <= 0; w_v <= 0; b_v <= 0; aw_a <= 0; w_d <= 0; w_s <= 0;
            ar_v <= 0; r_v <= 0; ar_a <= 0; rdata_q <= 0;
            reg_we <= 0; reg_addr <= 0; reg_wdata <= 0; reg_strb <= 0; start_pulse <= 0;
            dbg_sel <= 0; dbg_idx <= 0; dbg_sub <= 0;
            cfg_seq_r <= 0; cfg_stamp_r <= 0;
            pix_src <= 1'b0; pix_push <= 1'b0; pix_flush <= 1'b0;
            frame_wptr <= 0; frame_total <= 0; frame_len <= 0; frame_ovf <= 0;
        end else begin
            reg_we      <= 1'b0;      // 默认：写使能只持续一拍
            start_pulse <= 1'b0;
            pix_push    <= 1'b0;
            pix_flush   <= 1'b0;

            // ---------------- 写通道 ----------------
            if (s_awvalid && s_awready) begin aw_a <= s_awaddr; aw_v <= 1'b1; end
            if (s_wvalid  && s_wready ) begin w_d  <= s_wdata;  w_s <= s_wstrb; w_v <= 1'b1; end

            if (wr_commit) begin
                aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1;
                if (aw_a < ADDR_CONFIG + 12'h080) begin
                    // 0x000..0x07F：配置寄存器块 → 透给 vision_top。
                    // 只锁存原始数据/选通，字节合并留给下一拍的 cfg_wdata
                    // （见 w_masked/cfg_wdata 处的说明）。
                    reg_we    <= 1'b1;
                    reg_addr  <= {1'b0, aw_a[6:2]};
                    reg_wdata <= w_d;
                    reg_strb  <= w_s;
                end else case (aw_word)
                    W_DBG: begin
                        dbg_sel <= w_masked[3:0];
                        dbg_idx <= w_masked[21:4];
                        dbg_sub <= w_masked[25:22];
                    end
                    W_CTRL:   if (w_masked[0]) start_pulse <= 1'b1;
                    W_SEQ:    cfg_seq_r          <= wstrb_merge(cfg_seq_r, w_d, w_s);
                    W_STAMP0: cfg_stamp_r[31:0]  <= wstrb_merge(cfg_stamp_r[31:0],  w_d, w_s);
                    W_STAMP1: cfg_stamp_r[63:32] <= wstrb_merge(cfg_stamp_r[63:32], w_d, w_s);
                    W_PIXPUSH: begin
                        // 推一个 32 位字进像素 FIFO。**必须整字写**（strb=4'hF）
                        // —— FIFO 没有"按字节合并"的语义，部分字节写没有意义。
                        pix_push <= 1'b1;
                    end
                    W_PIXCTL: begin
                        pix_src   <= w_masked[0];
                        pix_flush <= w_masked[1];
                    end
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
                else case (ar_word)
                    W_DBGDAT: rdata_q <= dbg_data;
                    W_CTRL:   rdata_q <= ctrl_rd;
                    // [15:0] 还在缓冲的**字节**数；[16] 溢出（粘滞）；[17] 这一帧还在放
                    W_PIXSTAT: rdata_q <= {14'b0, pix_busy, pix_ovf, pix_buffered};
                    W_FLEN:   rdata_q <= {{(32-FB_BITS-1){1'b0}}, frame_len};
                    W_SEQ:    rdata_q <= cfg_seq_r;
                    W_STAMP0: rdata_q <= cfg_stamp_r[31:0];
                    W_STAMP1: rdata_q <= cfg_stamp_r[63:32];
                    default:  rdata_q <= (ar_word >= W_FRAME &&
                                          ar_word <  W_FRAME_END)
                                         ? frame_word(ar_word[FW_BITS-1:0]
                                                      - W_FRAME[FW_BITS-1:0]) : 32'd0;
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
