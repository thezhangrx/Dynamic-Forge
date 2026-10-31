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
    parameter integer FRAME_BYTES = 2048,
    //: 本地实验用：1 = 把正射的 `hold` 强制接 0（拿掉"等 vision_top 收像素"的门控）。
    //: 默认 0（正常行为）。只为在 cosim 里把"hold 是否影响 miss→填→命中"这条时间线分开，
    //: **不需要重建位流**（综合时它是常数）。
    parameter integer HOLD_BYPASS = 0
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
    input  wire [7:0]            din_b,

    // ---- AXI4 读主口（P0：正射从 DDR 取相机帧；BD 里接 ps7 S_AXI_HP0）----
    //: 为什么要在这一层引出：源帧在 DDR（PS 用 udmabuf 放），PL 必须自己按行去读
    //: （PS 逐字推只有 4.4 M 字/s = 17.3 ms/帧，64 位 HP 口峰值 400 MB/s = 2.3 ms/帧）。
    output wire [31:0]           m_araddr,
    output wire [7:0]            m_arlen,
    output wire                  m_arvalid,
    input  wire                  m_arready,
    input  wire [63:0]           m_rdata,
    input  wire                  m_rlast,
    input  wire                  m_rvalid,
    output wire                  m_rready
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
    //: 诊断（2026-10-24）：PS 推入的字节数 / 异或 / 和（只读，flush 清零）
    localparam [11:0] ADDR_PIXDBG0 = 12'h0E0;
    localparam [11:0] ADDR_PIXDBG1 = 12'h0E4;
    localparam [11:0] ADDR_PIXDBG2 = 12'h0E8;
    localparam [11:0] ADDR_PIXZSTRB = 12'h0EC;   // 被忽略的 WSTRB=0 写次数
    localparam [11:0] ADDR_PIXMEM   = 12'h0F0;   // 写：FIFO 字地址；读：该字内容
    localparam [11:0] ADDR_PIXPTR   = 12'h0F4;   // {16'b0, level[11:0], nb[3:0]}
    localparam [11:0] ADDR_PIXCOMMIT= 12'h0F8;   // 打到 PIXPUSH 的写次数（含被合并的）
    // ---- P0（正射进 PL）寄存器块：0x900..0x95F --------------------------
    //: 为什么放在 0x900：0x000..0x07F 是 vision_top 的配置块，0x080..0x0FF 是调试/控制，
    //: 0x100..0x8FF 是几何帧窗口（FRAME_BYTES=2048），所以 0x900 起是空的。
    localparam [11:0] ADDR_FRMCTL  = 12'h900;  // 写: bit0 像素源(1=正射) bit1 start bit2 清完成
                                               // 读: {fills[15:0], 13'b0, done, busy, src}
    localparam [11:0] ADDR_FRMBASE = 12'h904;  // 相机帧物理基址（字节，4 字节对齐）
    localparam [11:0] ADDR_FRMC0C1 = 12'h908;  // {c1[15:0], c0[15:0]}  页面列范围
    localparam [11:0] ADDR_FRMR0R1 = 12'h90C;  // {r1[15:0], r0[15:0]}  页面行范围
    localparam [11:0] ADDR_FRM_H0  = 12'h910;  // h0..h8：0x910 + 4*i
    localparam [11:0] ADDR_FRM_PY16= 12'h934;  // PY0_16（Q16.16 行基址，int32 够）
    localparam [11:0] ADDR_FRM_PS16= 12'h938;  // PS_16
    localparam [11:0] ADDR_FRM_PY24L=12'h93C;  // PY0_24 低 32 位
    localparam [11:0] ADDR_FRM_PY24H=12'h940;  // PY0_24[39:32]（Q8.24 常数 8.04e9 > 2^32）
    localparam [11:0] ADDR_FRM_PS24= 12'h944;  // PS_24
    localparam [11:0] ADDR_FRMDBG0= 12'h948;   // {src_line[15:0], src_off[10:0], hit, miss, 3'b0}
    localparam [11:0] ADDR_FRMDBG1= 12'h94C;   // {tag[15:0], fill_line[15:0]}
    localparam [11:0] ADDR_FRMDBG2= 12'h950;   // {fills[15:0], flags[4:0], rd_idx[10:0]}
    localparam [11:0] ADDR_FRMLINE= 12'h954;
    //: 0x958 只读诊断：{2'b0, sof_cnt, sof_wp, sof_rp, cnt, wp}（各 6 位）
    //: 目的：判断 camadapt 的缓冲在**帧首**是不是带着上一帧的残留（cnt>0）。
    localparam [11:0] ADDR_FRMDIAG= 12'h958;   // {miss_line[15:0], 16'b0}（最近一次未命中行）
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
    localparam [ADDR_W-3:0] W_PIXDBG0  = ADDR_PIXDBG0[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXDBG1  = ADDR_PIXDBG1[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXDBG2  = ADDR_PIXDBG2[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXZSTRB = ADDR_PIXZSTRB[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXMEM   = ADDR_PIXMEM[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXPTR   = ADDR_PIXPTR[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_PIXCOMMIT= ADDR_PIXCOMMIT[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRAME    = ADDR_FRAME[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRAME_END = ADDR_FRAME_END[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMCTL   = ADDR_FRMCTL[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMBASE  = ADDR_FRMBASE[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMC0C1  = ADDR_FRMC0C1[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMR0R1  = ADDR_FRMR0R1[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_H0   = ADDR_FRM_H0[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_PY16 = ADDR_FRM_PY16[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_PS16 = ADDR_FRM_PS16[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_PY24L= ADDR_FRM_PY24L[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_PY24H= ADDR_FRM_PY24H[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRM_PS24 = ADDR_FRM_PS24[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMDBG0  = ADDR_FRMDBG0[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMDBG1  = ADDR_FRMDBG1[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMDBG2  = ADDR_FRMDBG2[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMLINE  = ADDR_FRMLINE[ADDR_W-1:2];
    localparam [ADDR_W-3:0] W_FRMDIAG  = ADDR_FRMDIAG[ADDR_W-1:2];
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
    //: **WSTRB=0 的写不是数据**（2026-10-24 真硅取证确认）。
    //:
    //: 现象：板上喂一整帧后 n_obs=0、帧长/CRC 却自洽；灰度图呈**严格周期 4**的
    //: `[0,0,x,y]`（一半像素是 0）；白点标志从 idx 6700 漂到 20102（≈3×）。
    //: 取证：`PIXSTAT.buffered` 是"边录边排空"的活计数（读的时机不同数不同），
    //: 不能当计数器用；改用**整张灰度图与参考做相关**才看清形状。
    //: 结论：GP 口对**一次 32 位 store** 实际发 **3 笔 AXI 写**（1 笔带数据 +
    //: 2 笔 WSTRB=0）。原实现按"一笔写 = 一个字"处理，把 3 笔都压进 FIFO，
    //: 于是像素流变成 12 字节/字（4 数据 + 8 个零）→ 整幅图错位 → 判不出墙。
    //: cosim 一直没暴露：TB 的 `axi_write()` 只发**一笔** strb=0xF（已补
    //: `--mimic-gp` 覆盖这个主控行为，可本地复现 149 处不符）。
    //: 修法：按 AXI 语义 —— **WSTRB=0 的写不携带任何字节，不压**；部分字节写
    //: 先按字节合并，等 4 个字节选通都到齐再压一个字（整字写仍是一拍压一次，
    //: 快路径与 cosim 行为不变）。
    reg         pix_flush;        // 一拍脉冲
    reg  [31:0] pix_acc;          // 部分字节写时攒着的字节（见下面的说明）
    reg  [3:0]  pix_got;          // 已经到位的字节选通
    reg  [31:0] pix_zstrb_cnt;    // 被忽略的 WSTRB=0 写次数（诊断用，0x0EC 只读）
    reg  [31:0] pix_commit_cnt;   // 打到 PIXPUSH 的写次数（诊断用，0x0F8 只读）
    reg  [10:0] pix_mem_addr_q;   // 要回读的 FIFO 字地址（0x0F0 写入）

    // ---- P0（正射进 PL）寄存器 -------------------------------------------
    //: 系数由 PS 按 `core/fpga/tools/ortho_fixed_ref.cpp` 的同一个式子算好写进来
    //: （PL 不做浮点、不做单应求逆）。PY0_24 是 Q8.24 的 479.33*2^24 = 8.04e9 > 2^32，
    //: 所以拆成两个 32 位寄存器（低 32 位 + [39:32]）；其余常数 int32 都装得下。
    reg  signed [31:0] k_h0, k_h1, k_h2, k_h3, k_h4, k_h5, k_h6, k_h7, k_h8;
    reg  signed [31:0] k_py16, k_ps16, k_ps24;
    reg         [31:0] k_py24_lo;
    reg         [7:0]  k_py24_hi;
    reg  [9:0]  k_c0, k_c1, k_r0, k_r1;
    reg  [31:0] frm_base;
    reg         frm_src;          // 0 = 原路径（din_*/FIFO），1 = 正射
    reg         frm_start;        // 一拍脉冲：同时启动 vision_top 与正射
    reg         frm_done_lat;     // 粘滞的"正射这帧做完了"（PS 轮询会错过单拍脉冲）
    //: 正射子系统对外信号
    wire        of_vld, of_sof, of_eol, of_busy, of_done;
    wire [7:0]  of_r, of_g, of_b;
    wire [15:0] of_fills;
    wire [15:0] of_dbg_line, of_dbg_fill_line, of_dbg_tag;
    //: miss_line 的低 2 位在 FRMDBG 里不单独暴露（FRMLINE 用 [15:2]），留空不影响取证
    wire [9:0]  of_dbg_px;
    wire [7:0]  of_fill_state;
    wire [15:0] of_axi_left;
    wire [1:0]  of_axi_state;
    wire        of_blk;
    wire        of_dbg_hit, of_dbg_miss;
    wire [14:0] of_dbg_rd_idx;
    wire [5:0]  of_dbg_flags;
    wire [31:0] pix_dbg_ptr;      // pixfeed 的 {level, nb}（0x0F4）
    wire [31:0] pix_dbg_mem;      // pixfeed 的 mem[addr]（0x0F0）
    wire [15:0] pix_buffered;
    wire        pix_ovf;
    wire        pf_vld, pf_sof, pf_eol;
    wire [7:0]  pf_r, pf_g, pf_b;
    wire        pix_busy;
    wire [31:0] pix_dbg_bytes, pix_dbg_xor, pix_dbg_sum;

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
    // AXI 写数据的两种选通掩码（**必须在使用之前声明**）
    //
    // ⚠ 踩过：这段原来放在文件后面，而 pixfeed 的例化（`.push_data(w_masked32)`）
    //   在它前面。Verilog 对"先用后声明"会先建一个 **1 位隐式网络** ——
    //   仿真器没报（它按整模块解析），**综合器报了**：
    //     WARNING: [Synth 8-992] w_masked32 is already implicitly declared earlier
    //   真按 1 位处理的话，推像素的数据就只有 1 位了。**声明一律放使用之前。**
    //
    // ⚠ 另一条踩过的：**注释别以仿真器的名字开头**。`// Verilator ...`（大小写
    //   不敏感、连写成 VerilatorX 也算）会被它当成**元注释**去解析，后面的词不是
    //   合法指令就报 `syntax error, unexpected '/'` —— 而报错指向注释那一行，
    //   极容易朝"注释怎么会语法错"这个方向查。加个括号或换个说法即可。
    //   有测试守着这条（core/fpga/tests/test_cosim.py）。
    // =====================================================================
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
    // 26 位选通掩码（只给调试选择/配置用）。
    // ⚠ 原来这里还留了一个 32 位的 `w_masked32`，但 PIXPUSH 的字节合并走的是
    //   `pix_acc`（见下面的状态机），它没有任何读者 —— Verilator -Wall 的
    //   UNUSEDSIGNAL 因此把 test_rtl_lint_clean 打成红。删掉，不留悬空网。
    wire [25:0] w_masked = wstrb_merge26(26'd0, w_d[25:0], w_s);

    // =====================================================================
    // PS 喂像素：真摄像头采到的帧在 PS 手上，经 AXI 写进 FIFO，这里还原成
    // 像素时序。见 vision_pixfeed.v（含"为什么不需要 PS 等 FIFO 不满"）。
    // =====================================================================
    wire pf_sel = pix_src;      // 0 = 用 din_*（外部/TPG），1 = 用 PS 的 FIFO

    vision_pixfeed #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_pixfeed (
        .clk(clk), .rst_n(aresetn),
        .push(pix_push), .push_data(pix_acc),
        // `level`（FIFO 里的字数）是纯诊断量，对外报的是 `buffered`（字节数，
        // 含蓄水池），所以这里不接 —— 留着悬空，避免"声明了却没人用"的 lint 告警。
        .flush(pix_flush), .buffered(pix_buffered), .ovf(pix_ovf),
        .sink_ready(busy),      // 见 vision_pixfeed.v：busy 恰在进入 S_RECV 时拉高
        .dout_vld(pf_vld), .dout_sof(pf_sof), .dout_eol(pf_eol),
        .dout_r(pf_r), .dout_g(pf_g), .dout_b(pf_b),
        .busy(pix_busy),
        .dbg_bytes(pix_dbg_bytes), .dbg_xor(pix_dbg_xor), .dbg_sum(pix_dbg_sum),
        .dbg_mem_addr(pix_mem_addr_q), .dbg_mem_data(pix_dbg_mem), .dbg_ptr(pix_dbg_ptr)
    );

    // =====================================================================
    // P0：正射（DDR 里的 640×480 相机帧 → 640×360 页面像素）
    //
    // 数据流：PS 把相机帧 memcpy 进 udmabuf → 写 FRMBASE → 写 FRMCTL.start
    //        → 本模块按行从 HP0 读（64 位 burst）→ 行缓存 32 行 → 正射
    //        → 直接当 vision_top 的 din_*（不走 PIXPUSH，省掉 17.3 ms）
    //
    // ⚠ `hold = ~busy`：vision_top 没有背压，且每帧先进 S_HCLR 准备直方图
    //   （约 257 拍 busy=0）。正射必须冻在那儿等，否则头几百个像素被静默丢掉。
    // =====================================================================
    wire [63:0] k_py24    = {{24{k_py24_hi[7]}}, k_py24_hi, k_py24_lo};
    wire [39:0] k_py16_40 = {{8{k_py16[31]}}, k_py16};
    wire [39:0] k_ps16_40 = {{8{k_ps16[31]}}, k_ps16};
    wire [63:0] k_ps24_64 = {{32{k_ps24[31]}}, k_ps24};

    vision_ortho_sys #(
        .PL_W(WIDTH), .PL_H(HEIGHT), .SRC_W(640), .SRC_H(480)
    ) u_ortho (
        .clk(clk), .rst_n(aresetn),
        .start(frm_start),
        .hold(HOLD_BYPASS != 0 ? 1'b0 : ~busy),
        .busy(of_busy), .done(of_done),
        .h0(k_h0), .h1(k_h1), .h2(k_h2), .h3(k_h3), .h4(k_h4),
        .h5(k_h5), .h6(k_h6), .h7(k_h7), .h8(k_h8),
        .PY0_16(k_py16_40), .PS_16(k_ps16_40),
        .PY0_24(k_py24), .PS_24(k_ps24_64),
        .c0(k_c0), .c1(k_c1), .r0(k_r0), .r1(k_r1),
        .frm_base(frm_base),
        .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rdata(m_rdata), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready),
        .out_vld(of_vld), .out_sof(of_sof), .out_eol(of_eol),
        .out_r(of_r), .out_g(of_g), .out_b(of_b),
        .dbg_src_line(of_dbg_line), .dbg_src_px(of_dbg_px), .dbg_miss(of_dbg_miss),
        .dbg_fill_line(of_dbg_fill_line), .dbg_tag(of_dbg_tag), .dbg_hit(of_dbg_hit),
        .dbg_rd_idx(of_dbg_rd_idx), .dbg_fills(of_fills), .dbg_flags(of_dbg_flags),
        .dbg_fill_state(of_fill_state), .dbg_blk(of_blk),
        .dbg_axi_left(of_axi_left), .dbg_axi_state(of_axi_state)
    );

    // ---- 相机域适配（P3）：正射之后、vision_top 之前 ------------------------
    //: 为什么必须在 PL 里做：PL 的墙判据是**绝对阈值 + R−B**，而相机把暖白墙拍成中性偏蓝；
    //: PS 侧的 `camera_domain_adapt()` 原来在正射之后重染色，P0 之后 PS 手上没有页面图了。
    //: `flush`：一帧最后一像素进缓冲后还剩 <=41 个没吐，正交 done 后要把它们放完
    //: （vision_top 按 640×360 计数，少一个都不收口）。
    wire        ca_vld, ca_sof, ca_eol;
    wire [7:0]  ca_r, ca_g, ca_b;
    wire [31:0] ca_diag;   //: camadapt 只读诊断（0x958）
    reg         frm_flush;
    always @(posedge clk) begin
        if (!aresetn) frm_flush <= 1'b0;
        else if (frm_start) frm_flush <= 1'b0;
        else if (of_done)   frm_flush <= 1'b1;
    end
    vision_camadapt #(.WIDTH(WIDTH)) u_camadapt (
        .clk(clk), .rst_n(aresetn), .flush(frm_flush),
        .in_vld(of_vld), .in_sof(of_sof), .in_eol(of_eol),
        .in_r(of_r), .in_g(of_g), .in_b(of_b),
        .out_vld(ca_vld), .out_sof(ca_sof), .out_eol(ca_eol),
        .out_r(ca_r), .out_g(ca_g), .out_b(ca_b),
        .dbg_diag(ca_diag)
    );

    // 像素源三选一（**逐条对应**：vld/sof/eol/r/g/b 必须整体切换，不能混）
    //   frm_src=1 → 正射；否则按原口径（pix_src=1 → PS 的 FIFO，0 → 外部/TPG）
    wire        src_vld = frm_src ? ca_vld : (pf_sel ? pf_vld : din_vld);
    wire        src_sof = frm_src ? ca_sof : (pf_sel ? pf_sof : din_sof);
    wire        src_eol = frm_src ? ca_eol : (pf_sel ? pf_eol : din_eol);
    wire [7:0]  src_r   = frm_src ? ca_r   : (pf_sel ? pf_r   : din_r);
    wire [7:0]  src_g   = frm_src ? ca_g   : (pf_sel ? pf_g   : din_g);
    wire [7:0]  src_b   = frm_src ? ca_b   : (pf_sel ? pf_b   : din_b);
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
            pix_acc <= 32'd0; pix_got <= 4'd0; pix_zstrb_cnt <= 32'd0;
            pix_commit_cnt <= 32'd0; pix_mem_addr_q <= 11'd0;
            frame_wptr <= 0; frame_total <= 0; frame_len <= 0; frame_ovf <= 0;
            k_h0 <= 0; k_h1 <= 0; k_h2 <= 0; k_h3 <= 0; k_h4 <= 0;
            k_h5 <= 0; k_h6 <= 0; k_h7 <= 0; k_h8 <= 0;
            k_py16 <= 0; k_ps16 <= 0; k_ps24 <= 0; k_py24_lo <= 0; k_py24_hi <= 0;
            k_c0 <= 10'd0; k_c1 <= 10'd0; k_r0 <= 10'd0; k_r1 <= 10'd0;
            frm_base <= 32'd0; frm_src <= 1'b0; frm_start <= 1'b0; frm_done_lat <= 1'b0;
        end else begin
            reg_we      <= 1'b0;      // 默认：写使能只持续一拍
            start_pulse <= 1'b0;
            frm_start   <= 1'b0;   //: ⚠ 必须在这里清（case 之前）——写在 case 之后会盖掉
                                   //:   W_FRMCTL 里 `frm_start <= 1'b1`（非阻塞，后写的赢），
                                   //:   于是正射永远收不到 start（板上实测：fills=0、
                                   //:   vision_top 卡在等像素，CTRL busy=1 不落）。
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
                    W_PIXMEM:   pix_mem_addr_q <= w_masked[10:0];
                    W_PIXPUSH: begin
                        // 推一个 32 位字进像素 FIFO。**按字节选通判定**（见文件头
                        // 那段"WSTRB=0 的写不是数据"）：真硅上一次 32 位 store 会
                        // 拆成 1 笔 strb=0xF + 2 笔 strb=0，后者必须丢掉。
                        pix_commit_cnt <= pix_commit_cnt + 32'd1;
                        if (w_s == 4'h0) begin
                            pix_zstrb_cnt <= pix_zstrb_cnt + 32'd1;   // 空选通：不压
                        end else if ((pix_got | w_s) == 4'hF) begin
                            pix_push <= 1'b1;                        // 4 字节到齐：压一个字
                            pix_got  <= 4'd0;
                            pix_acc  <= wstrb_merge(pix_acc, w_d, w_s);
                        end else begin
                            pix_got  <= pix_got | w_s;               // 部分字节：先攒着
                            pix_acc  <= wstrb_merge(pix_acc, w_d, w_s);
                        end
                    end
                    W_PIXCTL: begin
                        pix_src   <= w_masked[0];
                        pix_flush <= w_masked[1];
                    end
                    // ---- P0：正射进 PL --------------------------------------
                    W_FRMCTL: begin
                        frm_src <= w_masked[0];
                        //: bit1 同时启动 vision_top（start_pulse）与正射：两者必须
                        //: 同一次写里起，否则"帧边界"对不上（正射会比 vision_top
                        //: 早/晚一帧，像素整幅平移）。
                        if (w_masked[1]) begin
                            frm_start    <= 1'b1;
                            start_pulse  <= 1'b1;
                            frm_done_lat <= 1'b0;
                        end
                        if (w_masked[2]) frm_done_lat <= 1'b0;
                    end
                    W_FRMBASE:  frm_base  <= wstrb_merge(frm_base, w_d, w_s);
                    //: 10 位（c1=560 装不进 9 位，见 vision_ortho 端口处的说明）
                    W_FRMC0C1:  begin k_c0 <= w_masked[9:0]; k_c1 <= w_masked[25:16]; end
                    W_FRMR0R1:  begin k_r0 <= w_masked[9:0]; k_r1 <= w_masked[25:16]; end
                    W_FRM_H0:   k_h0 <= wstrb_merge(k_h0, w_d, w_s);
                    W_FRM_H0+1: k_h1 <= wstrb_merge(k_h1, w_d, w_s);
                    W_FRM_H0+2: k_h2 <= wstrb_merge(k_h2, w_d, w_s);
                    W_FRM_H0+3: k_h3 <= wstrb_merge(k_h3, w_d, w_s);
                    W_FRM_H0+4: k_h4 <= wstrb_merge(k_h4, w_d, w_s);
                    W_FRM_H0+5: k_h5 <= wstrb_merge(k_h5, w_d, w_s);
                    W_FRM_H0+6: k_h6 <= wstrb_merge(k_h6, w_d, w_s);
                    W_FRM_H0+7: k_h7 <= wstrb_merge(k_h7, w_d, w_s);
                    W_FRM_H0+8: k_h8 <= wstrb_merge(k_h8, w_d, w_s);
                    W_FRM_PY16: k_py16  <= wstrb_merge(k_py16, w_d, w_s);
                    W_FRM_PS16: k_ps16  <= wstrb_merge(k_ps16, w_d, w_s);
                    W_FRM_PY24L:k_py24_lo <= wstrb_merge(k_py24_lo, w_d, w_s);
                    W_FRM_PY24H:k_py24_hi <= w_masked[7:0];
                    W_FRM_PS24: k_ps24  <= wstrb_merge(k_ps24, w_d, w_s);
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
                    W_PIXDBG0: rdata_q <= pix_dbg_bytes;
                    W_PIXDBG1: rdata_q <= pix_dbg_xor;
                    W_PIXDBG2: rdata_q <= pix_dbg_sum;
                    W_PIXZSTRB: rdata_q <= pix_zstrb_cnt;
                    W_PIXMEM:   rdata_q <= pix_dbg_mem;
                    W_PIXPTR:   rdata_q <= pix_dbg_ptr;
                    W_PIXCOMMIT: rdata_q <= pix_commit_cnt;
                    W_FLEN:   rdata_q <= {{(32-FB_BITS-1){1'b0}}, frame_len};
                    W_SEQ:    rdata_q <= cfg_seq_r;
                    W_STAMP0: rdata_q <= cfg_stamp_r[31:0];
                    W_STAMP1: rdata_q <= cfg_stamp_r[63:32];
                    W_FRMCTL: rdata_q <= {of_fills, 13'd0, frm_done_lat, of_busy, frm_src};
                    W_FRMBASE: rdata_q <= frm_base;
                    W_FRMC0C1: rdata_q <= {6'b0, k_c1, 6'b0, k_c0};
                    W_FRMR0R1: rdata_q <= {6'b0, k_r1, 6'b0, k_r0};
                    //: 系数回读（R71：探针读回全 0 是因为这里没写读译码，不是硬件错）
                    W_FRM_H0:   rdata_q <= k_h0;
                    W_FRM_H0+1: rdata_q <= k_h1;
                    W_FRM_H0+2: rdata_q <= k_h2;
                    W_FRM_H0+3: rdata_q <= k_h3;
                    W_FRM_H0+4: rdata_q <= k_h4;
                    W_FRM_H0+5: rdata_q <= k_h5;
                    W_FRM_H0+6: rdata_q <= k_h6;
                    W_FRM_H0+7: rdata_q <= k_h7;
                    W_FRM_H0+8: rdata_q <= k_h8;
                    W_FRM_PY16: rdata_q <= k_py16;
                    W_FRM_PS16: rdata_q <= k_ps16;
                    W_FRM_PY24L:rdata_q <= k_py24_lo;
                    W_FRM_PY24H:rdata_q <= {24'b0, k_py24_hi};
                    W_FRM_PS24: rdata_q <= k_ps24;
                    W_FRMDBG0: rdata_q <= {of_dbg_line, 1'b0, of_dbg_px, 2'b0, of_blk, of_dbg_hit, of_dbg_miss};
                    W_FRMDBG1: rdata_q <= {of_dbg_tag, of_fill_state, of_dbg_flags, of_dbg_fill_line[1:0]};
                    //: {flags, rd_idx[15:0], fills[10:0]} —— 全 16 位 rd_idx 都露出来
                    //: （fills 的完整 16 位在 FRMCTL 里，不用重复）
                    W_FRMDBG2: rdata_q <= {of_dbg_flags, of_dbg_rd_idx, of_fills[10:0]};
                    //: 打包（32 位）：{miss_line[15:2], axi_left[15:0], axi_state[1:0]} + miss_line[1:0] 补满 32 位
//:   tb 解码：left=(d>>4)&0xFFFF? —— 不用猜：tb 里按 `(d>>4)` 取不到时以本注释为准
                    W_FRMLINE: rdata_q <= {of_dbg_fill_line[15:2], of_axi_left[15:0], of_axi_state};
                    W_FRMDIAG: rdata_q <= ca_diag;
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
            //: 正射做完这帧 → 粘滞置位，PS 轮询 FRMCTL 不会漏掉单拍 done
            if (of_done) frm_done_lat <= 1'b1;
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
