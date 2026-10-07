// ===========================================================================
// vision_top.v —— 视觉链路的 PL 实现（像素流进 → 几何帧字节流出）
//
//   像素流 → [灰度+分类] → [掩膜+行直方图] → [行带] → [场地 x 范围]
//          → [逐列投影+持续台阶 → 墙段+缺口] → [扫描线 CCL 角色]
//          → [打记录 + CRC32] → 几何帧字节流
//
// 这份 RTL 的**比对对手是 core/fpga/ref/vision_ref.c**，不是 Python：
// 两边跑同一批真实帧，逐位比较掩膜、行带、墙段、缺口、角色和最终字节流。
// 协同仿真台 core/fpga/cosim/tb_vision.cpp，pytest 入口
// core/fpga/tests/test_cosim.py。
//
// 设计取舍（与 C 参考模型逐条对应）
// ---------------------------------
// * **多遍**：行带要先知道"哪些行墙够多"，场地 x 范围要先知道"哪些列墙够多"，
//   墙列判据又在这两者之后。C 是两遍（先存整帧掩膜，再按需回读），RTL 是
//   同一批遍、顺序执行。存整帧是允许的：PL BRAM 9.3 Mb ≈ 1190 KB，
//   灰度 300 KB + 两张 1 bit 掩膜 77 KB ≈ 32%（见移植规划 §2.1）。
// * **逐列投影**：C 的列投影是 `for x { for y }`，RTL 照抄列主序。好处是
//   列内的 Σy 与持续台阶各只要**深度为 k 的移位寄存器**，不需要 640 深的
//   行延迟线。
// * **几何量全是整数定标**（Q6 = 1/64），角度走 vision_ops.vh 的定点 atan2。
//   C 侧也改成同一串整数运算 —— 逐位一致是构造出来的，不是测出来的。
//
// 时钟域：本模块只有像素时钟一个域。AXI / PS 的对接在外层 wrapper，不在
// 本文件（工具链与 IP 未定，见移植规划 §0.5 待确认项）。
// ===========================================================================

`include "vision_pkg.vh"
`include "vision_ops.vh"

// ---------------------------------------------------------------------------
// 编译检查（Verilator）的警告取舍，逐个说明，不是一刀切静音
//   WIDTHEXPAND   — 设计里到处是 `窄寄存器 < 32'd常数`（检测参数写成了
//                   integer）。扩宽保值，不影响功能。
//   WIDTHTRUNC    — vision_ops.vh 里两处除法结果落到 32 位：vo_scale_round
//                   的结果是坐标（≤ 480*256）；atan2 的 r = a·2^15/b 上界
//                   32768。两处都已在现场注明，其余截断已经清掉。
//   BLKSEQ        — 定点函数内部必须用阻塞赋值（就是组合逻辑）；Verilator 把
//                   "被时序块调用的函数"也归进时序块，属误报。
//   UNUSEDSIGNAL  — green 掩膜按协议算出来但本阶段下游不用；地址加法器的高位
//                   在合法地址范围内恒为 0。
// 其余警告（SELRANGE / LATCH / …）保持打开 —— 它们报的都是真问题。
// ---------------------------------------------------------------------------
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off BLKSEQ */
/* verilator lint_off UNUSEDSIGNAL */

module vision_top #(
    parameter integer WIDTH  = 640,
    parameter integer HEIGHT = 360
) (
    input  wire        pix_clk,
    input  wire        rst_n,

    // ---- 像素输入（RGB 8 bit；灰度化在内部做，和 C 参考模型一致）--------
    input  wire        din_vld,
    input  wire        din_sof,
    input  wire        din_eol,
    input  wire [7:0]  din_r,
    input  wire [7:0]  din_g,
    input  wire [7:0]  din_b,

    // ---- 帧级控制 --------------------------------------------------------
    input  wire        frame_start,
    input  wire [31:0] cfg_seq,
    input  wire [63:0] cfg_stamp_ns,
    output reg         frame_done,
    output wire        busy,

    // ---- 运行时可配参数（AXI-Lite 写进来；地址映射见 vision_axi.v）------
    // 为什么必须有这一层：wall/bright 的判据是**绝对阈值**，真机上光照一变
    // 就会假阳性（docs/vision/adaptive.md）。所以阈值必须是寄存器，不是参数。
    input  wire        cfg_we,
    input  wire [5:0]  cfg_addr,
    input  wire [31:0] cfg_wdata,
    output reg  [31:0] cfg_rdata,

    // ---- 几何帧输出（valid/ready 字节流）--------------------------------
    output reg         dout_vld,
    output wire [7:0]  dout_data,
    output wire        dout_last,
    input  wire        dout_ready,

    // ---- 调试/回读口（PS 看中间量，也是协同仿真的观测窗）---------------
    input  wire [3:0]  dbg_sel,
    input  wire [17:0] dbg_idx,
    input  wire [3:0]  dbg_sub,
    output reg  [31:0] dbg_data
);

    // =====================================================================
    // 0. 参数（**必须等于** core/fpga/ref/vision_ref.c 的 VR_DEFAULT_*）
    //    跨语言测试逐项核对：core/fpga/tests/test_rtl_contract.py
    // =====================================================================
    localparam integer W_BITS = $clog2(WIDTH);
    localparam integer H_BITS = $clog2(HEIGHT);
    localparam integer AW     = $clog2(WIDTH * HEIGHT);

    localparam integer MAX_BANDS  = 64;
    localparam integer MAX_OBS    = 32;
    localparam integer MAX_GAPS   = 16;
    localparam integer MAX_SEGS   = 384;      // 一带最多 WIDTH/2 段
    localparam integer MAX_LABELS = 256;
    localparam integer MAX_RUNS   = 384;

    // 唯一还是 localparam 的检测参数：持续台阶的滞后行数 k。
    // 它决定 S_COL_SCAN 里移位寄存器的**级数**（结构），运行时不改。
    localparam integer EDGE_AVG = 2;

    // 除 EDGE_AVG 之外的检测参数与阈值都是**配置寄存器**，
    // 复位值 = core/fpga/ref/vision_ref.c 的 VR_DEFAULT_*（有测试钉住）。
    // 地址映射见 vision_axi.v / ref_api.h 的 VrRefParams。
    localparam [5:0] CFG_WALL_RB     = 6'd0,
                     CFG_WALL_MIN_R  = 6'd1,
                     CFG_WALL_MIN_G  = 6'd2,
                     CFG_BRIGHT_MIN  = 6'd3,
                     CFG_BRIGHT_SPRD = 6'd4,
                     CFG_GREEN_GM    = 6'd5,
                     CFG_GREEN_MIN   = 6'd6,
                     CFG_BAND_PCT    = 6'd7,
                     CFG_BAND_JOIN   = 6'd8,
                     CFG_BAND_MINR   = 6'd9,
                     CFG_FIELD_COL   = 6'd10,
                     CFG_WALL_PCT    = 6'd11,
                     CFG_WALL_COLMIN = 6'd12,
                     CFG_EDGE_MIN    = 6'd13,
                     CFG_EDGE_COLS   = 6'd14,
                     CFG_MIN_SEG     = 6'd15,
                     CFG_MIN_GAP     = 6'd16,
                     CFG_P_MIN_AREA  = 6'd17,
                     CFG_P_MAX_AREA  = 6'd18,
                     CFG_P_FILL      = 6'd19,
                     CFG_P_ASPECT    = 6'd20,
                     CFG_STATUS      = 6'd31;

    reg [7:0]  r_wall_rb, r_wall_min_r, r_wall_min_g;
    reg [7:0]  r_bright_min, r_bright_spread, r_green_gm, r_green_min;
    reg [15:0] r_band_row_pct, r_band_join_rows, r_band_min_rows, r_field_col_min;
    reg [15:0] r_wall_col_pct, r_wall_col_min, r_edge_min, r_edge_min_cols;
    reg [15:0] r_min_seg_px, r_min_gap_px;
    reg [31:0] r_player_min_area, r_player_max_area, r_player_min_fill_pct, r_player_max_aspect_pct;

    localparam [7:0] WIDTH_LO  = WIDTH[7:0];
    localparam [7:0] WIDTH_HI  = WIDTH[15:8];
    localparam [7:0] HEIGHT_LO = HEIGHT[7:0];
    localparam [7:0] HEIGHT_HI = HEIGHT[15:8];

    // EDGE_AVG 不是 2 时下面那两级移位寄存器不够用。宁可在展开时炸掉，
    // 也不要静默算错。
    initial begin
        if ($test$plusargs("dumpseg")) dbg_dump_seg = 1'b1;
        // 帧尺寸超过协议上限时，下面那些 AW/W_BITS 定宽地址就放不下了 ——
        // 展开阶段就报错，别等到板上地址回绕。
        if (WIDTH > `VISION_MAX_W || HEIGHT > `VISION_MAX_H)
            $error("vision_top: 帧尺寸 %0dx%0d 超过协议上限 %0dx%0d",
                   WIDTH, HEIGHT, `VISION_MAX_W, `VISION_MAX_H);
        if (EDGE_AVG != 2)
            $error("vision_top: 持续台阶固定 k=2（两级移位寄存器），改参数必须同步改代码");
    end

    // =====================================================================
    // 1. 帧存与直方图
    // =====================================================================
    reg [7:0]  gray_mem   [0:WIDTH*HEIGHT-1];
    reg        wall_mem   [0:WIDTH*HEIGHT-1];
    reg        bright_mem [0:WIDTH*HEIGHT-1];
    reg [15:0] row_count  [0:HEIGHT-1];
    reg [15:0] span_count [0:WIDTH-1];

    reg [5:0]        n_bands;
    reg [H_BITS-1:0] band_y0 [0:MAX_BANDS-1];
    reg [H_BITS-1:0] band_y1 [0:MAX_BANDS-1];
    reg [W_BITS-1:0] x_min, x_max;

    reg [15:0] col_cnt  [0:WIDTH-1];
    reg [31:0] col_sumy [0:WIDTH-1];
    reg [7:0]  col_step [0:WIDTH-1];
    reg        flags    [0:WIDTH-1];

    reg [8:0]        n_segs;
    reg [W_BITS-1:0] seg_x0 [0:MAX_SEGS-1];
    reg [W_BITS-1:0] seg_x1 [0:MAX_SEGS-1];
    reg [8:0]        seg_id [0:MAX_SEGS-1];
    reg [31:0]       seg_cy [0:MAX_SEGS-1];

    reg [7:0]  n_obs, n_gaps;
    reg [31:0] obs_words [0:511];
    reg [31:0] gap_words [0:511];

    reg [AW-1:0] rd_addr;
    wire [7:0] gray_rd   = gray_mem[rd_addr];
    wire       wall_rd   = wall_mem[rd_addr];

    // =====================================================================
    // 2. 第一遍：像素流
    // =====================================================================
    reg [W_BITS-1:0] wr_col;
    reg [H_BITS-1:0] wr_row;
    reg [15:0]       rc_run;
    wire [31:0]      wr_addr_w = din_sof ? 32'd0 : (wr_row * WIDTH + wr_col);
    wire [AW-1:0]    wr_addr   = wr_addr_w[AW-1:0];

    wire [7:0] gray_w = vo_gray(din_r, din_g, din_b);
    wire [2:0] cls_w  = vo_classify(din_r, din_g, din_b,
                                    r_wall_rb, r_wall_min_r, r_wall_min_g,
                                    r_bright_min, r_bright_spread,
                                    r_green_gm, r_green_min);
    wire is_wall = cls_w[0];

    // =====================================================================
    // 3. 一行/一带/一段的小工具
    // =====================================================================
    function [7:0] abs_diff(input [7:0] a, input [7:0] b);
        begin
            abs_diff = (a > b) ? (a - b) : (b - a);
        end
    endfunction

    function [AW-1:0] base_of(input [H_BITS-1:0] yy);
        reg [31:0] t;
        begin
            t = yy * WIDTH;
            base_of = t[AW-1:0];
        end
    endfunction

    function [15:0] band_limit;
        reg [31:0] t;
        begin
            t = (WIDTH * r_band_row_pct) / 100;
            if (t < 32'd1) t = 32'd1;
            band_limit = t[15:0];
        end
    endfunction

    function [15:0] wall_limit(input [15:0] h);
        reg [31:0] l;
        begin
            l = (h * r_wall_col_pct) / 100;
            if (l < r_wall_col_min) l = r_wall_col_min;
            wall_limit = l[15:0];
        end
    endfunction

    function [W_BITS:0] span_of(input [15:0] len);
        reg [31:0] t;
        begin
            t = len / 16'd4;
            if (t > 32'd20) t = 32'd20;
            if (t < 32'd1)  t = 32'd1;
            span_of = t[W_BITS:0];
        end
    endfunction

    // =====================================================================
    // 4. 主状态机
    // =====================================================================
    localparam [4:0] S_IDLE = 5'd0,
                       S_RECV       = 5'd1,
                       S_BAND       = 5'd2,
                       S_SPAN_CLR   = 5'd3,
                       S_SPAN       = 5'd4,
                       S_SPAN_FIND  = 5'd5,
                       S_COL_CLEAR  = 5'd6,
                       S_COL_SCAN   = 5'd7,
                       S_COL_RUNS   = 5'd8,
                       S_COL_RF     = 5'd9,
                       S_COL_RF2    = 5'd18,
                       S_SEG        = 5'd10,
                       S_SEG_X      = 5'd11,
                       S_SEG_FIN    = 5'd12,
                       S_GAP        = 5'd13,
                       S_CCL        = 5'd14,
                       S_PACK_INIT  = 5'd15,
                       S_PACK       = 5'd16,
                     S_DONE       = 5'd17;

    reg [4:0] state;
    // 调试钩子：`+dumpseg` 打开后每个墙段落一行中间量（与 C 侧
    // `VR_DEBUG_SEG=1` 的格式刻意做成一样，diff 就能定位分叉点）。
    reg       dbg_dump_seg = 1'b0;

    // 通用
    reg [15:0] ctr;

    // 行带扫描
    reg [H_BITS:0] bi;
    reg [1:0]      band_phase;
    reg            band_in_run;
    reg [H_BITS-1:0] band_run_a, m_a, m_b;
    reg            m_vld;

    // 场地范围
    reg [5:0]      band_i;
    reg [15:0]     sp_rows_left;
    reg [W_BITS:0] sp_cx;
    reg [W_BITS:0] sf_i;
    reg            span_seen;

    // 列投影
    reg [W_BITS:0]   cx;
    reg [H_BITS-1:0] cy;
    reg [15:0]       acc_cnt;
    reg [31:0]       acc_sumy;
    reg [7:0]        acc_step, sr0, sr1;

    // 列游程
    reg [W_BITS:0]   ri;
    reg              run_in;
    reg [W_BITS-1:0] run_a;

    // 段
    reg [8:0] seg_i, seg_limit;
    reg [W_BITS-1:0] sg_x0, sg_x1;
    reg [W_BITS:0]   sg_span;
    reg [W_BITS:0]   sx;
    reg [15:0]       sg_total;
    reg [31:0]       sg_sumy;
    reg [15:0]       sg_thick;
    reg [31:0]       sg_ls, sg_rs;
    reg [15:0]       sg_ln, sg_rn;

    // 缺口
    reg [8:0]  gi;
    reg [15:0] gp_best;
    reg [8:0]  gp_bi;
    reg        gp_vld;

    // CCL
    reg         ccl_start;
    wire        ccl_done;
    wire [17:0] ccl_addr;
    wire        ccl_pv;
    wire [31:0] ccl_px, ccl_py, ccl_pr, ccl_area;
    wire [9:0]  ccl_bx0, ccl_bx1, ccl_by0, ccl_by1;
    wire [8:0]  ccl_nlabel;

    // 角色记录
    reg        player_valid;
    reg [31:0] player_x, player_y, player_r;
    reg [9:0]  blob_x0, blob_x1, blob_y0, blob_y1;
    reg [31:0] blob_area;
    reg [8:0]  n_labels;

    // 打包
    reg [1:0]  pk_phase;
    reg [15:0] pk_off, pk_len;
    reg [31:0] pk_crc;
    reg [1:0]  pk_tx;
    reg [8:0]  pk_widx;
    reg [1:0]  pk_byi;
    reg [1:0]  hdr_flags;
    reg        ccl_started;

    // ---------------------------------------------------------------------
    vision_ccl #(.WIDTH(WIDTH), .HEIGHT(HEIGHT),
                 .MAX_LABELS(MAX_LABELS), .MAX_RUNS(MAX_RUNS)) u_ccl (
        .clk(pix_clk), .rst_n(rst_n), .start(ccl_start), .done(ccl_done),
        .rd_addr(ccl_addr), .bright_rd(bright_mem[ccl_addr]),
        .min_area(r_player_min_area), .max_area(r_player_max_area),
        .min_fill_pct(r_player_min_fill_pct), .max_aspect_pct(r_player_max_aspect_pct),
        .player_valid(ccl_pv), .player_x(ccl_px), .player_y(ccl_py), .player_r(ccl_pr),
        .blob_x0(ccl_bx0), .blob_x1(ccl_bx1), .blob_y0(ccl_by0), .blob_y1(ccl_by1),
        .blob_area(ccl_area), .n_labels(ccl_nlabel)
    );

    // ---------------------------------------------------------------------
    // 配置寄存器（AXI-Lite 写进来；复位值 = C 参考模型的默认参数）
    // 读回与写用同一套地址；状态寄存器只读。
    // ---------------------------------------------------------------------
    reg        frame_done_lat;
    reg [15:0] frame_count;

    always @(posedge pix_clk) begin
        if (!rst_n) begin
            r_wall_rb       <= 8'd40;   r_wall_min_r  <= 8'd110;
            r_wall_min_g    <= 8'd60;
            r_bright_min    <= 8'd195;  r_bright_spread <= 8'd45;
            r_green_gm      <= 8'd10;   r_green_min   <= 8'd90;
            r_band_row_pct   <= 16'd5;   r_band_join_rows <= 16'd4;  r_band_min_rows <= 16'd5;
            r_field_col_min  <= 16'd5;
            r_wall_col_pct   <= 16'd25;  r_wall_col_min <= 16'd2;
            r_edge_min   <= 16'd12;  r_edge_min_cols  <= 16'd1;
            r_min_seg_px    <= 16'd8;   r_min_gap_px    <= 16'd8;
            r_player_min_area <= 32'd60;  r_player_max_area <= 32'd1500;
            r_player_min_fill_pct     <= 32'd45;  r_player_max_aspect_pct   <= 32'd160;
        end else if (cfg_we) begin
            case (cfg_addr)
                CFG_WALL_RB:     r_wall_rb       <= cfg_wdata[7:0];
                CFG_WALL_MIN_R:  r_wall_min_r    <= cfg_wdata[7:0];
                CFG_WALL_MIN_G:  r_wall_min_g    <= cfg_wdata[7:0];
                CFG_BRIGHT_MIN:  r_bright_min    <= cfg_wdata[7:0];
                CFG_BRIGHT_SPRD: r_bright_spread <= cfg_wdata[7:0];
                CFG_GREEN_GM:    r_green_gm      <= cfg_wdata[7:0];
                CFG_GREEN_MIN:   r_green_min     <= cfg_wdata[7:0];
                CFG_BAND_PCT:    r_band_row_pct    <= cfg_wdata[15:0];
                CFG_BAND_JOIN:   r_band_join_rows   <= cfg_wdata[15:0];
                CFG_BAND_MINR:   r_band_min_rows   <= cfg_wdata[15:0];
                CFG_FIELD_COL:   r_field_col_min   <= cfg_wdata[15:0];
                CFG_WALL_PCT:    r_wall_col_pct    <= cfg_wdata[15:0];
                CFG_WALL_COLMIN: r_wall_col_min <= cfg_wdata[15:0];
                CFG_EDGE_MIN:    r_edge_min    <= cfg_wdata[15:0];
                CFG_EDGE_COLS:   r_edge_min_cols   <= cfg_wdata[15:0];
                CFG_MIN_SEG:     r_min_seg_px     <= cfg_wdata[15:0];
                CFG_MIN_GAP:     r_min_gap_px     <= cfg_wdata[15:0];
                CFG_P_MIN_AREA:  r_player_min_area  <= cfg_wdata;
                CFG_P_MAX_AREA:  r_player_max_area  <= cfg_wdata;
                CFG_P_FILL:      r_player_min_fill_pct      <= cfg_wdata;
                CFG_P_ASPECT:    r_player_max_aspect_pct    <= cfg_wdata;
                default: ;                       /* 未映射地址写进来当没看见 */
            endcase
        end
    end

    always @* begin
        case (cfg_addr)
            CFG_WALL_RB:     cfg_rdata = {24'b0, r_wall_rb};
            CFG_WALL_MIN_R:  cfg_rdata = {24'b0, r_wall_min_r};
            CFG_WALL_MIN_G:  cfg_rdata = {24'b0, r_wall_min_g};
            CFG_BRIGHT_MIN:  cfg_rdata = {24'b0, r_bright_min};
            CFG_BRIGHT_SPRD: cfg_rdata = {24'b0, r_bright_spread};
            CFG_GREEN_GM:    cfg_rdata = {24'b0, r_green_gm};
            CFG_GREEN_MIN:   cfg_rdata = {24'b0, r_green_min};
            CFG_BAND_PCT:    cfg_rdata = {16'b0, r_band_row_pct};
            CFG_BAND_JOIN:   cfg_rdata = {16'b0, r_band_join_rows};
            CFG_BAND_MINR:   cfg_rdata = {16'b0, r_band_min_rows};
            CFG_FIELD_COL:   cfg_rdata = {16'b0, r_field_col_min};
            CFG_WALL_PCT:    cfg_rdata = {16'b0, r_wall_col_pct};
            CFG_WALL_COLMIN: cfg_rdata = {16'b0, r_wall_col_min};
            CFG_EDGE_MIN:    cfg_rdata = {16'b0, r_edge_min};
            CFG_EDGE_COLS:   cfg_rdata = {16'b0, r_edge_min_cols};
            CFG_MIN_SEG:     cfg_rdata = {16'b0, r_min_seg_px};
            CFG_MIN_GAP:     cfg_rdata = {16'b0, r_min_gap_px};
            CFG_P_MIN_AREA:  cfg_rdata = r_player_min_area;
            CFG_P_MAX_AREA:  cfg_rdata = r_player_max_area;
            CFG_P_FILL:      cfg_rdata = r_player_min_fill_pct;
            CFG_P_ASPECT:    cfg_rdata = r_player_max_aspect_pct;
            // 状态（只读）：bit0 busy，bit1 最近一帧已完成，[23:8] 完成帧计数
            CFG_STATUS:      cfg_rdata = {8'b0, frame_count, 6'b0, frame_done_lat, busy};
            default:         cfg_rdata = 32'd0;
        endcase
    end

    // ---------------------------------------------------------------------
    // 行带工具
    // ---------------------------------------------------------------------
    task band_push(input [H_BITS-1:0] a, input [H_BITS-1:0] b);
        begin
            if ((b - a + 1'b1 >= r_band_min_rows) && (n_bands < MAX_BANDS)) begin
                band_y0[n_bands] <= a;
                band_y1[n_bands] <= b;
                n_bands <= n_bands + 1'b1;
            end
        end
    endtask

    task band_raw_close(input [H_BITS-1:0] b);
        begin
            if (m_vld && (band_run_a - m_b - 1'b1 <= r_band_join_rows)) begin
                m_b <= b;
            end else begin
                if (m_vld) band_push(m_a, m_b);
                m_a   <= band_run_a;
                m_b   <= b;
                m_vld <= 1'b1;
            end
        end
    endtask

    // 列游程工具
    task seg_push(input [W_BITS-1:0] a, input [W_BITS-1:0] b);
        begin
            if ((b - a + 1'b1 >= r_min_seg_px) && (n_segs < MAX_SEGS)) begin
                seg_x0[n_segs] <= a + x_min;
                seg_x1[n_segs] <= b + x_min;
                n_segs <= n_segs + 1'b1;
            end
        end
    endtask

    // ---------------------------------------------------------------------
    // 几何小工具（带符号）
    // ---------------------------------------------------------------------
    function [31:0] thick_or_h(input [15:0] t, input [15:0] h);
        begin
            thick_or_h = (t > 16'd0) ? {16'b0, t} : {16'b0, h};
        end
    endfunction

    // ---------------------------------------------------------------------
    // 打包：头字节
    // ---------------------------------------------------------------------
    function [7:0] hdr_byte(input [4:0] o);
        begin
            case (o)
                5'd0: hdr_byte = 8'h42;   // 'BHV1' 小端 = 0x31564842
                5'd1: hdr_byte = 8'h48;
                5'd2: hdr_byte = 8'h56;
                5'd3: hdr_byte = 8'h31;
                5'd4: hdr_byte = 8'h01;   // version = 1 (u16)
                5'd5: hdr_byte = 8'h00;
                5'd6: hdr_byte = {6'b0, hdr_flags};
                5'd7: hdr_byte = 8'h00;
                5'd8:  hdr_byte = cfg_seq[7:0];
                5'd9:  hdr_byte = cfg_seq[15:8];
                5'd10: hdr_byte = cfg_seq[23:16];
                5'd11: hdr_byte = cfg_seq[31:24];
                5'd12: hdr_byte = n_obs;
                5'd13: hdr_byte = 8'h00;
                5'd14: hdr_byte = 8'h00;
                5'd15: hdr_byte = 8'h00;
                5'd16: hdr_byte = n_gaps;
                5'd17: hdr_byte = 8'h00;
                5'd18: hdr_byte = 8'h00;
                5'd19: hdr_byte = 8'h00;
                5'd20: hdr_byte = WIDTH_LO;
                5'd21: hdr_byte = WIDTH_HI;
                5'd22: hdr_byte = HEIGHT_LO;
                5'd23: hdr_byte = HEIGHT_HI;
                default: hdr_byte = cfg_stamp_ns[8 * (o - 5'd24) +: 8];
            endcase
        end
    endfunction

    function [7:0] byte_of(input [31:0] w, input [1:0] sel);
        begin
            case (sel)
                2'd0: byte_of = w[7:0];
                2'd1: byte_of = w[15:8];
                2'd2: byte_of = w[23:16];
                default: byte_of = w[31:24];
            endcase
        end
    endfunction

    reg [7:0] pk_byte_r;
    wire [31:0] crc_fin = ~pk_crc;

    always @* begin
        if (pk_off < 16'd32) begin
            pk_byte_r = hdr_byte(pk_off[4:0]);
        end else if (pk_off < 16'd48) begin
            case (pk_off[3:2])
                2'd0: pk_byte_r = byte_of(player_x, pk_off[1:0]);
                2'd1: pk_byte_r = byte_of(player_y, pk_off[1:0]);
                2'd2: pk_byte_r = byte_of(player_r, pk_off[1:0]);
                default: pk_byte_r = byte_of({31'b0, player_valid}, pk_off[1:0]);
            endcase
        end else if (pk_off < 16'd72) begin
            pk_byte_r = 8'h00;                     // target 记录：本模型不产出
        end else if (pk_off < 16'd72 + 36 * n_obs) begin
            pk_byte_r = byte_of(obs_words[pk_widx], pk_byi);
        end else begin
            pk_byte_r = byte_of(gap_words[pk_widx], pk_byi);
        end
    end

    assign dout_data = (pk_phase == 2'd0) ? pk_byte_r : crc_fin[8 * pk_tx +: 8];
    // 整帧的**最后**一个字节 = CRC 的第 4 个字节。
    // ⚠ 早先写成"payload 最后一字节 **或** CRC 最后一字节"，于是 dout_last
    // 每帧拉高**两次**。纯字节流看不出来（消费方只数 dout_vld），但下游要是
    // 用 dout_last 定帧长（AXI 包装层就是这么做的），帧长会变成 4 ——
    // 这个 bug 是被 AXI 路径的协同仿真抓到的，tb_vision 一路没发现。
    assign dout_last = dout_vld && (pk_phase == 2'd1) && (pk_tx == 2'd3);
    assign busy      = (state != S_IDLE);

    // =====================================================================
    // 5. 主时序
    // =====================================================================
    // ---- 组合：当前带高 / 列判据门槛 / 段几何 --------------------------
    wire [15:0] bh      = band_y1[band_i] - band_y0[band_i] + 1'b1;
    wire [15:0] lim_now = wall_limit(bh);
    wire [15:0] el_now  = r_edge_min * EDGE_AVG[15:0];
    wire [31:0] sg_cyq_c = (sg_total != 0)
        ? vo_scale_round({16'b0, sg_sumy}, {16'b0, sg_total}, 16'd64)
        : ({16'b0, band_y0[band_i]} + {16'b0, band_y1[band_i]}) * 32'd32;
    // C 的退化分支是 `*cy_q * 4`（用**量化后**的 cy），不是重新量化 cy*256。
    wire [31:0] sg_lq_c = (sg_ln != 0)
        ? vo_scale_round({16'b0, sg_ls}, {16'b0, sg_ln}, 16'd256) : (sg_cyq_c * 32'd4);
    wire [31:0] sg_rq_c = (sg_rn != 0)
        ? vo_scale_round({16'b0, sg_rs}, {16'b0, sg_rn}, 16'd256) : (sg_cyq_c * 32'd4);
    wire signed [31:0] sg_dx_c = $signed({16'b0, sg_x1 - sg_x0 + 1'b1}) + 32'sd1
                                 - $signed({16'b0, sg_span}) * 32'sd2;
    // 倾角先算到一根**组合 wire** 上，再拿去写记录数组。
    // 不是风格问题：把 `vo_atan2_q6(...)` 直接写进"数组元素的非阻塞赋值 RHS"
    // 时（同一函数在别处还有一次调用），Verilator 5.020 -O2 落库的值会算错 ——
    // 现象是 $display 打印的中间量全对、只有存进数组的字节不对。
    // 提到 wire 上就对了；这条已在 rtl/README.md 的"踩过的坑"里记下。
    wire signed [31:0] sg_ang_c = (sg_dx_c > 0)
        ? vo_atan2_q6($signed(sg_rq_c - sg_lq_c), sg_dx_c <<< 8) : 32'sd0;
    wire [7:0]  cur_gray = gray_rd;
    wire [7:0]  cur_step = (({1'b0, cy} + EDGE_AVG) <= {1'b0, band_y1[band_i]})
                           ? abs_diff(gray_rd, sr1) : 8'd0;
    wire [7:0]  step_next = (cur_step > acc_step) ? cur_step : acc_step;
    wire [15:0] next_cnt  = acc_cnt + (wall_rd ? 16'd1 : 16'd0);
    wire [31:0] next_sumy = acc_sumy + (wall_rd ? {16'b0, cy} : 32'd0);
    wire [W_BITS:0]   cx_rel  = cx - x_min;
    wire        flag_now  = (next_cnt >= lim_now) ||
                            ((next_cnt >= r_edge_min_cols) && ({8'b0, step_next} >= el_now));

    always @(posedge pix_clk) begin
        if (!rst_n) begin
            state <= S_IDLE; ctr <= 0;
            dout_vld <= 1'b0; frame_done <= 1'b0;
            frame_done_lat <= 1'b0; frame_count <= 16'd0;
            wr_col <= 0; wr_row <= 0; rc_run <= 0;
            n_bands <= 0; n_obs <= 0; n_gaps <= 0; n_segs <= 0;
            player_valid <= 0; ccl_start <= 0; ccl_started <= 0;
        end else begin
            frame_done <= 1'b0;
            ccl_start  <= 1'b0;
            dout_vld   <= (state == S_PACK);

            case (state)
            // =============================================================
            S_IDLE: begin
                if (frame_start) begin
                    frame_done_lat <= 1'b0;
                    wr_col <= 0; wr_row <= 0; rc_run <= 0;
                    n_bands <= 0; n_obs <= 0; n_gaps <= 0; n_segs <= 0;
                    player_valid <= 1'b0; ccl_started <= 1'b0;
                    bi <= 0; band_phase <= 0; band_in_run <= 0; m_vld <= 0;
                    state <= S_RECV;
                end
            end
            // =============================================================
            S_RECV: begin
                if (din_vld) begin
                    gray_mem[wr_addr]   <= gray_w;
                    wall_mem[wr_addr]   <= is_wall;
                    bright_mem[wr_addr] <= cls_w[1];
                    if (din_sof) begin
                        // 帧首像素：行/列必须**在这里**归零。上一帧结束把 wr_row
                        // 加到了 HEIGHT，不归零的话第二帧的地址会整体偏一帧。
                        wr_row <= 0;
                        wr_col <= 1;
                        rc_run <= is_wall ? 16'd1 : 16'd0;
                    end else if (din_eol) begin
                        row_count[wr_row] <= rc_run + (is_wall ? 16'd1 : 16'd0);
                        rc_run <= 0; wr_col <= 0;
                        wr_row <= wr_row + 1'b1;
                        if (wr_row == HEIGHT[H_BITS-1:0] - 1'b1) state <= S_BAND;
                    end else begin
                        rc_run <= rc_run + (is_wall ? 16'd1 : 16'd0);
                        wr_col <= wr_col + 1'b1;
                    end
                end
            end
            // =============================================================
            // 行带
            S_BAND: begin
                if (band_phase == 2'd0) begin
                    if (bi < HEIGHT) begin
                        if (row_count[bi[H_BITS-1:0]] >= band_limit()) begin
                            if (!band_in_run) begin
                                band_in_run <= 1'b1;
                                band_run_a  <= bi[H_BITS-1:0];
                            end
                        end else if (band_in_run) begin
                            band_in_run <= 1'b0;
                            band_raw_close(bi[H_BITS-1:0] - 1'b1);
                        end
                        bi <= bi + 1'b1;
                    end else begin
                        band_phase <= 2'd1;
                    end
                end else if (band_phase == 2'd1) begin
                    if (band_in_run) begin
                        band_in_run <= 1'b0;
                        band_raw_close(HEIGHT[H_BITS-1:0] - 1'b1);
                    end
                    band_phase <= 2'd2;
                end else begin
                    if (m_vld) band_push(m_a, m_b);
                    m_vld <= 1'b0;
                    ctr   <= 0;
                    state <= S_SPAN_CLR;
                end
            end
            // =============================================================
            S_SPAN_CLR: begin
                if (ctr < WIDTH) begin
                    span_count[ctr[W_BITS-1:0]] <= 16'd0;
                    ctr <= ctr + 1'b1;
                end else if (n_bands == 0) begin
                    sf_i <= 0; span_seen <= 1'b0;
                    state <= S_SPAN_FIND;
                end else begin
                    band_i       <= 0;
                    sp_cx        <= 0;
                    sp_rows_left <= band_y1[0] - band_y0[0] + 1'b1;
                    rd_addr      <= base_of(band_y0[0]);
                    state        <= S_SPAN;
                end
            end
            // =============================================================
            S_SPAN: begin
                if (band_i >= n_bands) begin
                    sf_i <= 0; span_seen <= 1'b0;
                    state <= S_SPAN_FIND;
                end else if (sp_rows_left == 16'd0) begin
                    band_i <= band_i + 1'b1;
                    if (band_i + 1'b1 < n_bands) begin
                        sp_rows_left <= band_y1[band_i+1'b1] - band_y0[band_i+1'b1] + 1'b1;
                        rd_addr      <= base_of(band_y0[band_i+1'b1]);
                        sp_cx        <= 0;
                    end
                end else begin
                    if (wall_rd) span_count[sp_cx[W_BITS-1:0]] <=
                                     span_count[sp_cx[W_BITS-1:0]] + 16'd1;
                    rd_addr <= rd_addr + 1'b1;
                    if (sp_cx == WIDTH - 1) begin
                        sp_cx <= 0;
                        sp_rows_left <= sp_rows_left - 1'b1;
                    end else begin
                        sp_cx <= sp_cx + 1'b1;
                    end
                end
            end
            // =============================================================
            S_SPAN_FIND: begin
                if (sf_i < WIDTH) begin
                    if (span_count[sf_i[W_BITS-1:0]] >= r_field_col_min) begin
                        if (!span_seen) begin x_min <= sf_i[W_BITS-1:0]; span_seen <= 1'b1; end
                        x_max <= sf_i[W_BITS-1:0];
                    end
                    sf_i <= sf_i + 1'b1;
                end else begin
                    if (!span_seen) begin x_min <= 0; x_max <= WIDTH[W_BITS-1:0] - 1'b1; end
                    span_seen <= 1'b0;
                    band_i <= 0;
                    ctr    <= 0;
                    state  <= S_COL_CLEAR;
                end
            end
            // =============================================================
            S_COL_CLEAR: begin
                if (band_i >= n_bands) begin
                    state <= S_CCL;
                end else if (ctr < WIDTH) begin
                    col_cnt[ctr[W_BITS-1:0]]  <= 16'd0;
                    col_sumy[ctr[W_BITS-1:0]] <= 32'd0;
                    col_step[ctr[W_BITS-1:0]] <= 8'd0;
                    ctr <= ctr + 1'b1;
                end else begin
                    cx      <= x_min;
                    cy      <= band_y1[band_i];
                    rd_addr <= base_of(band_y1[band_i]) + x_min;
                    acc_cnt <= 0; acc_sumy <= 0; acc_step <= 0;
                    sr0 <= 8'd0; sr1 <= 8'd0;
                    n_segs <= 0;
                    state  <= S_COL_SCAN;
                end
            end
            // =============================================================
            // 逐列（列内自下而上；持续台阶只要两级移位寄存器）
            S_COL_SCAN: begin
                acc_cnt  <= next_cnt;
                acc_sumy <= next_sumy;
                acc_step <= step_next;
                sr0 <= cur_gray; sr1 <= sr0;

                if (cy == band_y0[band_i]) begin
                    col_cnt[cx[W_BITS-1:0]]  <= next_cnt;
                    col_sumy[cx[W_BITS-1:0]] <= next_sumy;
                    col_step[cx[W_BITS-1:0]] <= step_next;
                    flags[cx_rel[W_BITS-1:0]] <= flag_now;
                    if (cx == x_max) begin
                        ri <= 0; run_in <= 1'b0;
                        state <= S_COL_RUNS;
                    end else begin
                        cx      <= cx + 1'b1;
                        cy      <= band_y1[band_i];
                        rd_addr <= base_of(band_y1[band_i]) + cx + 1'b1;
                        acc_cnt <= 0; acc_sumy <= 0; acc_step <= 0;
                        sr0 <= 8'd0; sr1 <= 8'd0;
                    end
                end else begin
                    cy      <= cy - 1'b1;
                    rd_addr <= rd_addr - WIDTH[AW-1:0];
                end
            end
            // =============================================================
            S_COL_RUNS: begin
                if (ri < (x_max - x_min + 1'b1)) begin
                    if (flags[ri[W_BITS-1:0]]) begin
                        if (!run_in) begin run_in <= 1'b1; run_a <= ri[W_BITS-1:0]; end
                    end else if (run_in) begin
                        run_in <= 1'b0;
                        seg_push(run_a, ri[W_BITS-1:0] - 1'b1);
                    end
                    ri <= ri + 1'b1;
                end else begin
                    state <= S_COL_RF;
                end
            end
            // 收尾那一段：**先推、再结算**。
            // 早先这里写的是 `seg_limit <= run_in ? n_segs + 1 : n_segs;` —— 而
            // seg_push 只在"段长 >= min_seg_px"时才真的把 n_segs 加一。于是
            // "最后一段比 min_seg_px 还短"时，seg_limit 比真实段数大 1，
            // S_SEG 会去读一条**没写过**的 seg_x0/seg_x1（残留值），
            // 吐出一个 x=[0,0]、ln=0 的假段，n_obs 和帧长都跟着错。
            // 这一拍把 n_segs（已经提交）结算成 seg_limit，是必须的。
            S_COL_RF: begin
                if (run_in) begin
                    run_in <= 1'b0;
                    seg_push(run_a, x_max - x_min);
                end
                state <= S_COL_RF2;
            end
            S_COL_RF2: begin
                seg_limit <= n_segs;
                seg_i     <= 0;
                state     <= S_SEG;
            end
            // =============================================================
            // 每个墙段：一次扫过它的 x 范围，算 cy/厚度/倾角
            S_SEG: begin
                if ((seg_i >= seg_limit) || (n_obs >= MAX_OBS)) begin
                    seg_limit <= seg_i;
                    gi <= 0; gp_best <= 0; gp_bi <= 0; gp_vld <= 0;
                    state <= S_GAP;
                end else begin
                    sg_x0 <= seg_x0[seg_i];
                    sg_x1 <= seg_x1[seg_i];
                    sg_span <= span_of(seg_x1[seg_i] - seg_x0[seg_i] + 1'b1);
                    sx <= seg_x0[seg_i];
                    sg_total <= 0; sg_sumy <= 0; sg_thick <= 0;
                    sg_ls <= 0; sg_rs <= 0; sg_ln <= 0; sg_rn <= 0;
                    state <= S_SEG_X;
                end
            end
            S_SEG_X: begin
                if (col_cnt[sx[W_BITS-1:0]] > sg_thick[15:0])
                    sg_thick <= col_cnt[sx[W_BITS-1:0]];
                sg_total <= sg_total + col_cnt[sx[W_BITS-1:0]];
                sg_sumy  <= sg_sumy + col_sumy[sx[W_BITS-1:0]];
                if (sx <= sg_x0 + sg_span - 1'b1) begin
                    sg_ls <= sg_ls + col_sumy[sx[W_BITS-1:0]];
                    sg_ln <= sg_ln + col_cnt[sx[W_BITS-1:0]];
                end
                // C 的 x >= x1-span+1 在 x1 < span-1 时下溢成空 —— 改成
                // x+span >= x1+1 的等价形式；此时区间应当退化成 [x0, x1] 全长。
                if (sx + sg_span >= {1'b0, sg_x1} + 1'b1) begin
                    sg_rs <= sg_rs + col_sumy[sx[W_BITS-1:0]];
                    sg_rn <= sg_rn + col_cnt[sx[W_BITS-1:0]];
                end
                if (sx == sg_x1) state <= S_SEG_FIN;
                else             sx <= sx + 1'b1;
            end
            S_SEG_FIN: begin
                obs_words[n_obs*9 + 0] <= {24'b0, n_obs} + 32'd1;
                obs_words[n_obs*9 + 1] <= 32'd1;                    // VSHAPE_RECT
                obs_words[n_obs*9 + 2] <= 32'd2;                    // VTYPE_WALL_WITH_GAP
                obs_words[n_obs*9 + 3] <= ({16'b0, sg_x0} + {16'b0, sg_x1}) * 32'd32;
                obs_words[n_obs*9 + 4] <= sg_cyq_c;
                obs_words[n_obs*9 + 5] <= 32'd0;                    // radius：矩形没有
                obs_words[n_obs*9 + 6] <= ({16'b0, sg_x1} - {16'b0, sg_x0} + 32'd1) * 32'd32;
                obs_words[n_obs*9 + 7] <= thick_or_h(sg_thick, bh) * 32'd32;
                obs_words[n_obs*9 + 8] <= sg_ang_c;
                if (dbg_dump_seg)
                    $display("V  seg x=[%0d,%0d] span=%0d ls=%0d ln=%0d rs=%0d rn=%0d lq=%0d rq=%0d dx=%0d ang=%0d",
                             sg_x0, sg_x1, sg_span, sg_ls, sg_ln, sg_rs, sg_rn,
                             sg_lq_c, sg_rq_c, sg_dx_c, sg_ang_c);
                seg_id[seg_i] <= n_obs + 1'b1;
                seg_cy[seg_i] <= sg_cyq_c;
                n_obs <= n_obs + 1'b1;
                seg_i <= seg_i + 1'b1;
                state <= S_SEG;
            end
            // =============================================================
            S_GAP: begin
                if (gi + 1'b1 >= seg_limit) begin
                    if (gp_vld && (n_gaps < MAX_GAPS)) begin
                        gap_words[n_gaps*9 + 0] <= {24'b0, n_gaps} + 32'd1;
                        gap_words[n_gaps*9 + 1] <= ({16'b0, seg_x1[gp_bi]} +
                                                    {16'b0, seg_x0[gp_bi+1'b1]}) * 32'd32;
                        gap_words[n_gaps*9 + 2] <= (seg_cy[gp_bi] + seg_cy[gp_bi+1'b1] + 32'd1) >> 1;
                        gap_words[n_gaps*9 + 3] <= {16'b0, gp_best} * 32'd64;
                        gap_words[n_gaps*9 + 4] <= 32'd0;
                        gap_words[n_gaps*9 + 5] <= seg_id[gp_bi];
                        gap_words[n_gaps*9 + 6] <= seg_id[gp_bi+1'b1];
                        gap_words[n_gaps*9 + 7] <= 32'd2;
                        gap_words[n_gaps*9 + 8] <= 32'd1;              // VGF_RELIABLE
                        n_gaps <= n_gaps + 1'b1;
                    end
                    gp_vld <= 1'b0;
                    band_i <= band_i + 1'b1;
                    ctr    <= 0;
                    state  <= S_COL_CLEAR;
                end else begin
                    if ((seg_x0[gi+1'b1] - seg_x1[gi] - 1'b1 >= r_min_gap_px) &&
                        (seg_x0[gi+1'b1] - seg_x1[gi] - 1'b1 > gp_best)) begin
                        gp_best <= seg_x0[gi+1'b1] - seg_x1[gi] - 1'b1;
                        gp_bi   <= gi;
                        gp_vld  <= 1'b1;
                    end
                    gi <= gi + 1'b1;
                end
            end
            // =============================================================
            S_CCL: begin
                if (!ccl_started) begin
                    ccl_start   <= 1'b1;
                    ccl_started <= 1'b1;
                end else if (ccl_done) begin
                    ccl_started  <= 1'b0;
                    player_valid <= ccl_pv;
                    player_x     <= ccl_px;
                    player_y     <= ccl_py;
                    player_r     <= ccl_pr;
                    blob_x0      <= ccl_bx0;
                    blob_x1      <= ccl_bx1;
                    blob_y0      <= ccl_by0;
                    blob_y1      <= ccl_by1;
                    blob_area    <= ccl_area;
                    n_labels     <= ccl_nlabel;
                    state        <= S_PACK_INIT;
                end
            end
            // =============================================================
            S_PACK_INIT: begin
                if (dbg_dump_seg)
                    $display("V  store obs8=%0d obs17=%0d n_obs=%0d", obs_words[8], obs_words[17], n_obs);
                pk_len   <= 16'd72 + 16'd36 * n_obs + 16'd36 * n_gaps;
                pk_off   <= 0;
                pk_phase <= 2'd0;
                pk_crc   <= 32'hFFFFFFFF;
                pk_widx  <= 0;
                pk_byi   <= 0;
                pk_tx    <= 0;
                hdr_flags<= player_valid ? 2'd2 : 2'd0;
                state    <= S_PACK;
            end
            S_PACK: begin
                if (dout_vld && dout_ready) begin
                    if (pk_phase == 2'd0) begin
                        pk_crc <= vo_crc_step(pk_crc, pk_byte_r);
                        if (pk_off + 1'b1 >= pk_len) begin
                            pk_phase <= 2'd1;
                            pk_tx    <= 2'd0;
                        end else begin
                            pk_off <= pk_off + 1'b1;
                            if (pk_off >= 16'd72) begin
                                if (pk_byi == 2'd3) begin
                                    pk_byi <= 2'd0;
                                    // 障碍区走完（正好 9*n_obs 个字）时把字下标归零，
                                    // 否则缺口区会从 gap_words[9*n_obs + …] 取数 ——
                                    // 那里是空的，帧尾整段会错。
                                    if (pk_off + 1'b1 == 16'd72 + 36 * n_obs) pk_widx <= 9'd0;
                                    else pk_widx <= pk_widx + 1'b1;
                                end else pk_byi <= pk_byi + 1'b1;
                            end
                        end
                    end else if (pk_tx == 2'd3) begin
                        // dout_vld 在块首是 `state == S_PACK`，这一拍必须显式拉低 ——
                        // 否则 S_PACK→S_DONE 那一拍还会再吐一个字节（帧长 +1）。
                        dout_vld <= 1'b0;
                        state    <= S_DONE;
                    end else begin
                        pk_tx <= pk_tx + 1'b1;
                    end
                end
            end
            // =============================================================
            S_DONE: begin
                frame_done      <= 1'b1;
                frame_done_lat  <= 1'b1;
                frame_count     <= frame_count + 1'b1;
                state           <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    // =====================================================================
    // 6. 调试/回读口（组合）
    // =====================================================================
    always @* begin
        case (dbg_sel)
            4'd0:  dbg_data = {24'b0, gray_mem[dbg_idx[AW-1:0]]};
            4'd1:  dbg_data = {31'b0, wall_mem[dbg_idx[AW-1:0]]};
            4'd2:  dbg_data = {31'b0, bright_mem[dbg_idx[AW-1:0]]};
            4'd3:  dbg_data = {16'b0, row_count[dbg_idx[H_BITS-1:0]]};
            4'd4:  dbg_data = {16'b0, span_count[dbg_idx[W_BITS-1:0]]};
            4'd5:  dbg_data = {{(32-H_BITS){1'b0}}, band_y0[dbg_idx[5:0]]};
            4'd6:  dbg_data = {{(32-H_BITS){1'b0}}, band_y1[dbg_idx[5:0]]};
            4'd7:  case (dbg_sub)
                       4'd0: dbg_data = {26'b0, n_bands};
                       4'd1: dbg_data = {{(32-W_BITS){1'b0}}, x_min};
                       4'd2: dbg_data = {{(32-W_BITS){1'b0}}, x_max};
                       4'd3: dbg_data = {24'b0, n_obs};
                       4'd4: dbg_data = {24'b0, n_gaps};
                       4'd5: dbg_data = {23'b0, n_labels};
                       4'd6: dbg_data = {23'b0, n_segs};
                       default: dbg_data = 32'd0;
                   endcase
            4'd8:  dbg_data = obs_words[dbg_idx[8:0]];
            4'd9:  dbg_data = gap_words[dbg_idx[8:0]];
            4'd10: case (dbg_sub)
                       4'd0: dbg_data = player_x;
                       4'd1: dbg_data = player_y;
                       4'd2: dbg_data = player_r;
                       4'd3: dbg_data = {31'b0, player_valid};
                       default: dbg_data = 32'd0;
                   endcase
            4'd11: case (dbg_sub)
                       4'd0: dbg_data = {22'b0, blob_x0};
                       4'd1: dbg_data = {22'b0, blob_x1};
                       4'd2: dbg_data = {22'b0, blob_y0};
                       4'd3: dbg_data = {22'b0, blob_y1};
                       4'd4: dbg_data = blob_area;
                       default: dbg_data = 32'd0;
                   endcase
            4'd12: dbg_data = {16'b0, col_cnt[dbg_idx[W_BITS-1:0]]};
            4'd13: dbg_data = col_sumy[dbg_idx[W_BITS-1:0]];
            4'd14: dbg_data = {24'b0, col_step[dbg_idx[W_BITS-1:0]]};
            4'd15: dbg_data = {31'b0, dout_vld};
            default: dbg_data = 32'd0;
        endcase
    end

endmodule
