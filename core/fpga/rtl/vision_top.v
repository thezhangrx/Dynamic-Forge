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
// 注意 `vision_ops.vh` 不在这里 include —— 它里面是**函数**，放在模块外会落到
// 根作用域，Verilog-2001 不允许（Vivado 报 [Synth 8-2644] root scope
// declaration is not allowed）。它在模块体里 include，见下面。

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

    // 纯组合算子（灰度/分类/定点 atan2 表/CRC）。**必须 include 在模块体内** ——
    // 放模块外会落到根作用域，Verilog-2001 不允许。见 vision_ops.vh 顶部说明。
    `include "vision_ops.vh"

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
    // 一带最多几个墙段。**上界可以证出来，不是拍的**：
    //   * `seg_push` 只接受 `长度 >= MIN_SEG_TO_STANDARD (=24)` 的段，
    //     而 24 是**硬常量**，不受运行时配置影响（`r_min_seg_px` 可以配得更小，
    //     但 24 那道永远在）；
    //   * 被接受的段之间至少隔 1 个 `flags == 0` 的列（它们是 flags 的游程）；
    //   * 索引范围 `x_max - x_min + 1 <= WIDTH = 640`。
    //   ⇒ 24k + (k-1) <= 640  ⇒  k <= 25
    // 取 32 留余量。**缩小这个常量不改变任何行为** —— 由 620 帧逐位对账守着。
    //
    // 为什么值得缩：实测 `MAX_SEGS=384` 时，`seg_*` 四个数组是**寄存器**实现的
    // （被组合变址读，推论不成 RAM），值约 20 800 个 LUT + 23 424 个 FF。
    // 384→96 的二分实验省下 15 592 LUT + 15 192 FF。见 docs/帧存改造.md §13。
    localparam integer MAX_SEGS   = 32;
    localparam integer MAX_LABELS = 256;
    localparam integer MAX_RUNS   = 384;

    // ---------------------------------------------------------------------
    // `to_standard` 的结构性过滤阈值：段长短于它就丢掉。
    //
    // **必须等于** core/vision/to_standard.py 的 DEFAULT_MIN_SEGMENT_PX = 24，
    // 也等于 vision_ref.c 里那个 VR_TO_STANDARD_MIN_SEG_PX。
    // 检测器为了找缺口会把墙切成碎片（实测出现过 8 px、17 px 的），
    // 它们不是真障碍，还会破坏 `half_w >= half_h` 的不变量。
    //
    // 注意它与 `r_min_seg_px`（默认 8，**运行时可配的检测参数**）是两回事：
    // 那道在检测层，这道在"转标准结构"层，**两道都要**。
    // ---------------------------------------------------------------------
    localparam integer MIN_SEG_TO_STANDARD = 24;

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
    // 三张帧存**必须**落进 BRAM，不能落 LUTRAM —— 这是实测出来的硬约束：
    //   640×360 三张图共 2 304 000 bit。器件里的分布式 RAM 容量只有
    //   26600 个 LUT，最多装 1.7 Mbit，**物理上装不下**；而 BRAM 有 9.3 Mb。
    //   早先这里没有 ram_style，又用异步读，Vivado 只能推断成分布式 RAM ——
    //   综合出来 "LUT as Distributed RAM 124092 / 26600 = 466%"，整个设计
    //   超器件容量 331%，位流根本生成不了。
    // ram_style="block" 是**必要条件但不是充分条件**：实测单独一个 1-bit×
    //   230400 的异步读数组，加了 ram_style=block 照样落 LUTRAM、BRAM 用 0 个。
    //   必须**同步读**（见下面的 mem_ra / gray_rd），两者齐备才落 BRAM。
    (* ram_style = "block" *) reg [7:0]  gray_mem   [0:WIDTH*HEIGHT-1];
    (* ram_style = "block" *) reg        wall_mem   [0:WIDTH*HEIGHT-1];
    (* ram_style = "block" *) reg        bright_mem [0:WIDTH*HEIGHT-1];
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
    // **单写口**记录存储。写进程见 §5.4（一个数组只有一个 `mem[a] <= d`）。
    (* ram_style = "distributed" *) reg [31:0] obs_words [0:511];
    (* ram_style = "distributed" *) reg [31:0] gap_words [0:511];

    reg [AW-1:0] rd_addr;

    // ---------------------------------------------------------------------
    // 帧存回读：**同步读 + 单一读地址总线**
    //
    // 为什么要同步读：BRAM 只有同步读口，异步读会被强制落成分布式 RAM。
    //   实测（见 core/fpga/docs/工具链现状.md）：三张图 2 304 000 bit，而器件的
    //   分布式 RAM 只有 26600 个 LUT、最多装 1.7 Mbit —— **物理上装不下**。
    //   加了 ram_style="block" 也救不了异步读（实测 BRAM 仍是 0 个）。
    //
    // 为什么用"下一拍地址" rd_nxt 而不是 rd_addr 本身：
    //   BRAM 的地址也要打一拍，输出天然比地址晚一拍。若直接把 rd_addr 喂进去，
    //   读回来的数据就比状态机期望的**晚一拍**，两个扫描器全错。
    //   把"状态机本拍将要赋给 rd_addr 的那个值"提前一拍喂给 BRAM，就有
    //       gray_rd(t) = gray_mem[rd_nxt(t-1)] = gray_mem[rd_addr(t)]
    //   —— 与原来的异步读**逐拍对齐**，状态机一个字节都不用改。
    //   这条思路就是 prefetch（ZipCPU《Building a prefetch module》），
    //   参考出处见 core/fpga/docs/参考项目.md。
    //   代价：rd_nxt 必须与状态机对 rd_addr 的 6 处赋值**逐字一致**。
    //   这条由 620 帧逐位对账守着 —— 对不上会当场报差异，不会静默通过。
    //
    // 为什么三张图共用一个读地址：
    //   行带/列扫描器、CCL、调试口**从不同时**要数据。BRAM 只有两个口，给每张图
    //   各配独立读口会逼 Vivado 复制 BRAM（实测 1 写 + 2 读 → BRAM 数翻倍到 160/265），
    //   共用则只要 80/265。仲裁见下面的 mem_ra。
    // ---------------------------------------------------------------------
    reg [AW-1:0] rd_nxt;        // 预读地址（= 本拍将要赋给 rd_addr 的值）
    reg [7:0]    gray_rd;       // 下面三个都是 **寄存输出**，不是组合
    reg          wall_rd;
    reg          bright_rd;

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

    // 带高上限 = WIDTH 的 pct%。**必须带输入参数** ——
    // Verilog-2001 不接受"无输入的函数"。Verilator 容忍，
    // 但 Vivado 默认模式会直接报 [Synth 8-1549] a function must have
    // at least one input —— 这一条只有综合能发现。
    function [15:0] band_limit(input [15:0] pct);
        reg [31:0] t;
        begin
            t = (WIDTH * pct) / 100;
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
                       S_SG_CYQ     = 5'd19,   // 多周期几何：算 cyq
                       S_SG_LQ      = 5'd20,   //   算 lq
                       S_SG_RQ      = 5'd21,   //   算 rq
                       S_SG_ANG     = 5'd22,   //   算倾角（内部一次除法）
                       // 带末后处理：整堵墙共用一个 rotation（见 S_ANG_INIT 的注释）
                       S_ANG_INIT   = 5'd23,
                       S_ANG_DIV    = 5'd24,
                       S_ANG_WR     = 5'd25,
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
    //: **压缩写指针**：退化段（half_w < half_h）会被丢掉，被保留的段要写到
    //: 紧凑的下标上，否则 `seg_x0/seg_x1/seg_id/seg_cy` 与 `seg_limit` 会错位，
    //: S_GAP 读到的缺口两端就不是相邻的两段了。
    //: 恒有 `seg_k <= seg_i`，所以回写 `seg_x0[seg_k]` 时那一格早已被读过，安全。
    reg [8:0] seg_k;
    // ---- 整堵墙共用一个朝向（带末后处理用）---------------------------------
    //: 本带第一条障碍在 obs_words 里的下标。带内的障碍在 obs_words 里**连续**
    //: （S_SEG 一条接一条写），所以后处理只要 [band_obs0, n_obs) 这一段。
    reg [8:0] band_obs0;
    //: 算出来的墙朝向（Q6），回写用。
    reg [31:0] ang_wall;
    //: 回写游标。
    reg [8:0] ang_i;
    reg [W_BITS-1:0] sg_x0, sg_x1;
    reg [W_BITS:0]   sg_span;
    reg [W_BITS:0]   sx;
    // ⚠ 累加器位宽必须能装下**最坏情况**，C 侧是 `long total/sum_y` 不会溢出。
    //
    // `sg_total` 原来是 16 位（上限 65535），而一段最多能覆盖
    // `WIDTH × HEIGHT = 640 × 360 = 230400` 个墙像素 —— **会回绕**。
    // 回绕后它当了除数，`cyq = (sumy*128+total)/(2*total)` 直接算错。
    //
    // 实测（data/fpga/stim_photo 真实照片）：帧 11 的 `[0,603]` 段
    // （len=604、thick=323，几乎整幅图）算出 cyq=28998 ⇒ y=453，
    // 而图像只有 360 高 —— 物理上不可能。32 张里 2 张触发。
    // 合成语料 320 帧全绿：它的段都很小，远远碰不到 65535。
    // 见 core/fpga/docs/帧存改造.md §14.4。
    //
    // 上界：total ≤ 230400（18 位）、sumy ≤ 230400×359 ≈ 8.3e7（27 位）、
    //       ln/rn ≤ 20×360 = 7200（13 位）。32 位都有很大余量。
    reg [31:0]       sg_total;
    reg [31:0]       sg_sumy;
    reg [15:0]       sg_thick;
    reg [31:0]       sg_ls, sg_rs;
    reg [31:0]       sg_ln, sg_rn;

    // ---------------------------------------------------------------------
    // 墙段几何：**多周期除法**
    //
    // 原先是三个组合的 `vo_scale_round` + 一个 `vo_atan2_q6`（里面还有一次
    // 除法），串起来成了整份设计的关键路径：438 级逻辑、404 个 CARRY4、
    // WNS −85.9 ns。现在改用迭代除法器 vision_div，按
    //     cyq → lq → rq → ang
    // 的顺序一拍一位地算，状态机在 S_SG_* 几个新状态里等它。
    //
    // 每段多花 ~40×4 = 160 拍；一帧 23 万拍，几百个墙段，完全够。
    // 结果与组合除法**逐位相同**（都是截断整除），由 620 帧逐位对账守着。
    // ---------------------------------------------------------------------
    reg        sgr_started;                 // 本步是否已发起
    reg        div_start;
    reg [39:0] div_num;
    reg [19:0] div_den;
    wire [39:0] div_quo;
    wire        div_busy, div_done;

    reg [31:0] sg_cyq_r;                    // 三个归约结果（原 sg_cyq_c/lq_c/rq_c）
    reg [31:0] sg_lq_r;
    reg [31:0] sg_rq_r;
    reg [31:0] sg_ang_r;                    // 倾角（原 sg_ang_c）

    vision_div #(.WN(40), .WD(20)) u_sgdiv (
        .clk(pix_clk), .rst_n(rst_n), .start(div_start),
        .num(div_num), .den(div_den),
        .quo(div_quo), .busy(div_busy), .done(div_done)
    );

    // 缺口
    reg [8:0]  gi;
    reg [15:0] gp_best;
    reg [8:0]  gp_bi;
    reg        gp_vld;

    // CCL
    reg         ccl_start;
    wire        ccl_done;
    wire [17:0] ccl_addr;
    wire [17:0] ccl_addr_nxt;   // CCL 的预读地址（见 vision_ccl 与下面的 mem_ra）
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
        .rd_addr(ccl_addr), .rd_addr_nxt(ccl_addr_nxt), .bright_rd(bright_rd),
        .min_area(r_player_min_area), .max_area(r_player_max_area),
        .min_fill_pct(r_player_min_fill_pct), .max_aspect_pct(r_player_max_aspect_pct),
        .player_valid(ccl_pv), .player_x(ccl_px), .player_y(ccl_py), .player_r(ccl_pr),
        .blob_x0(ccl_bx0), .blob_x1(ccl_bx1), .blob_y0(ccl_by0), .blob_y1(ccl_by1),
        .blob_area(ccl_area), .n_labels(ccl_nlabel)
    );

    // =====================================================================
    // 帧存读口：单一读地址总线 + 寄存输出
    //
    // 三个使用者在**不同时刻**要数据，所以共用一个读口（BRAM 只有两个口，
    // 分开给会逼 Vivado 复制 BRAM，实测 80 → 160 个）：
    //   * S_SPAN / S_COL_SCAN 的两个扫描器 → rd_nxt（预读地址）
    //   * S_CCL 的 CCL                      → ccl_addr_nxt（预读地址）
    //   * 调试口 sel=0/1/2 读掩膜像素       → dbg_idx
    //
    // 调试口为什么要让到帧间：它的读是**随机**地址，与扫描器无法共存于同一拍。
    // 帧间（IDLE/DONE）扫描器不动，这时把口让给调试口，两边都不受影响。
    // 代价：**掩膜像素的调试读只在帧间有效** —— 这是为了让设计装得下 030
    // 而做的取舍，已在 core/fpga/docs/工具链现状.md 记明。
    // =====================================================================
    wire dbg_mask_sel  = (dbg_sel == 4'd0) || (dbg_sel == 4'd1) || (dbg_sel == 4'd2);
    wire frame_idle    = (state == S_IDLE) || (state == S_DONE);
    wire dbg_owns_read = dbg_mask_sel && frame_idle;
    wire ccl_owns_read = (state == S_CCL);

    wire [AW-1:0] mem_ra = dbg_owns_read ? dbg_idx[AW-1:0]
                         : ccl_owns_read ? ccl_addr_nxt[AW-1:0]
                         : rd_nxt;

    always @(posedge pix_clk) begin
        gray_rd   <= gray_mem[mem_ra];
        wall_rd   <= wall_mem[mem_ra];
        bright_rd <= bright_mem[mem_ra];
    end

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
            // 两道长度门槛：`r_min_seg_px` 是运行时可配的**检测参数**，
            // MIN_SEG_TO_STANDARD 是"转标准结构"的**结构性过滤**（见常量声明处）。
            if ((b - a + 1'b1 >= r_min_seg_px) &&
                (b - a + 1'b1 >= MIN_SEG_TO_STANDARD) &&
                (n_segs < MAX_SEGS)) begin
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
    // 段几何的**组合部分**（不含除法）。三个 vo_scale_round 与 atan2 里的那次
    // 除法已移到 S_SG_CYQ/LQ/RQ/ANG 四个多周期状态里，结果是 sg_cyq_r/lq_r/rq_r/ang_r。
    wire signed [31:0] sg_dx_c = $signed({16'b0, sg_x1 - sg_x0 + 1'b1}) + 32'sd1
                                 - $signed({16'b0, sg_span}) * 32'sd2;
    wire signed [31:0] sg_ang_y = sg_rq_r - sg_lq_r;

    // atan2 前半段的组合部分：取 |y| 与 x，必要时对调（保证 a <= b）。
    // 与 C 的 `neg = y<0; a=|y|; b=x; if (a>b) swap, inv=1;` 一致。
    // ⚠ C 的调用是 vr_atan2_q6(rq-lq, dx << 8) —— 第二个参数是 **x_q8**，
    //   不是裸的 dx。这里漏过 `<< 8` 一次，症状是角度整体算错（v 落在错的
    //   表区间），而 cyq/lq/rq 全对，所以只有帧里的角度字段不符。
    wire signed [31:0] an_x8_w = sg_dx_c <<< 8;          // x_q8
    wire        an_neg_w = (sg_ang_y < 32'sd0);
    wire [31:0] an_ay_w  = an_neg_w ? -sg_ang_y : sg_ang_y;
    wire        an_inv_w = (an_ay_w > an_x8_w);
    wire [31:0] an_a_w   = an_inv_w ? an_x8_w : an_ay_w;
    wire [31:0] an_b_w   = an_inv_w ? an_ay_w : an_x8_w;

    // ---------------------------------------------------------------------
    // 后处理用的 atan2 组合部分：**整堵墙的朝向**。
    //
    // 与 C 参考模型这段逐字对应（vision_ref.c 的 vr_extract_walls）：
    //     dx = ((keep[last].a + keep[last].b) - (keep[0].a + keep[0].b)) * (VR_Q/2)
    //     dy = kcy_q[last] - kcy_q[0]
    //     ang_wall = (dx != 0) ? vr_atan2_q6(dy, dx) : kang_q[0]
    //
    // `seg_x0/seg_x1` 含 x_min、C 的 keep[].a/.b 不含 —— 但这里只用**差值**，
    // x_min 自己抵消，所以两边等价。
    //
    // 量级：x 中点用 (x0+x1)*32（≤ 639*2*32 ≈ 4.1e4），cy 是 Q6，两者同一标度。
    // vr_atan2_q6 只看比值，标度不影响结果；而且内部 `a<<15` 不会溢出
    // （C 侧注释要求输入 ≤ 480*256，这里远小于它）。注意这里**不做 `<<8`** ——
    // 与逐段那次的 `<<8` 不同，因为两边给的本来就是 Q6 量级，再移位会溢出。
    // ---------------------------------------------------------------------
    wire signed [31:0] aw_dy_w = seg_cy[seg_k - 9'd1] - seg_cy[9'd0];
    wire signed [31:0] aw_dx_w = (({16'b0, seg_x1[seg_k - 9'd1]} + {16'b0, seg_x0[seg_k - 9'd1]}) -
                                  ({16'b0, seg_x1[9'd0]}          + {16'b0, seg_x0[9'd0]})) * 32'sd32;
    wire        aw_neg_w = (aw_dy_w < 32'sd0);
    wire [31:0] aw_ay_w  = aw_neg_w ? -aw_dy_w : aw_dy_w;
    wire [31:0] aw_x_w   = aw_dx_w[31:0];
    wire        aw_inv_w = (aw_ay_w > aw_x_w);
    wire [31:0] aw_a_w   = aw_inv_w ? aw_x_w : aw_ay_w;
    wire [31:0] aw_b_w   = aw_inv_w ? aw_ay_w : aw_x_w;

    // =====================================================================
    // 5.4 `obs_words` / `gap_words`：**单写口**记录存储
    //
    // 为什么必须单写口：Vivado 只要在一个 always 块里看到**多条** `mem[..] <= ..`
    // 就判成"多写口 RAM"（WARNING: [Synth 8-4767] "RAM has multiple writes via
    // different ports in same process"），于是拒绝推断，把 512×32 拆成
    // **16 384 个寄存器 + 一片 512:1 的读出多路器**。两个数组合计约
    // 11 000 个 LUT + 32 768 个 FF —— 这是"装不下"的第二大来源
    // （第一大是 MAX_SEGS=384 的 seg_*，已按证明收到 32）。见 docs/帧存改造.md §13/§14。
    //
    // 改法：**逐字段写**。每个数组只留一个写进程、一个写口；状态机在一个小计数器
    // 上把 9 个字段一拍一个写进去。
    //
    // 读口**一个都没动**：打包器的 `obs_words[pk_widx]` / `gap_words[pk_widx]`
    // 和调试口的 `obs_words[dbg_idx]` 都是**组合读**，而
    // `ram_style = "distributed"` 的 LUTRAM 天生支持异步读 ——
    // 所以流水线不用加级、协同仿真测试台一行都不用改。
    //
    // 代价：每条记录 9 拍。段数有上界 25（见 MAX_SEGS 处的证明）、缺口 ≤ 16，
    // 每帧最多多几百拍，相对实测 521 230 拍/帧可忽略。
    // =====================================================================
    reg  [3:0]  wf      = 4'd0;   // S_SEG_FIN 的字段计数 0..8
    reg  [3:0]  gw_f    = 4'd0;   // S_GAP     的字段计数 0..8
    reg         gw_busy = 1'b0;   // S_GAP 正在逐字段写（期间不再搜缺口）

    // ---------------- obs_words ----------------
    // 丢掉 `half_w < half_h` 的段时**一个字段都不写**（见 S_SEG_FIN 的三条要求）。
    wire        sg_drop = (({16'b0, sg_x1} - {16'b0, sg_x0} + 32'd1) <
                           thick_or_h(sg_thick, bh));
    // 两个写来源：S_SEG_FIN 写整条新记录；S_ANG_WR 只回写 rotation 那一格。
    wire        ow_ang  = (state == S_ANG_WR);
    wire [8:0]  ow_ix   = ow_ang ? {4'b0, ang_i} : {4'b0, n_obs};
    wire [11:0] ow_b12  = {3'b0, ow_ix} + {ow_ix, 3'b0};       // = ow_ix * 9
    wire [8:0]  ow_ad   = ow_b12[8:0] + (ow_ang ? 9'd8 : {5'b0, wf});
    wire        ow_en   = ow_ang ? (ang_i < n_obs)
                                 : ((state == S_SEG_FIN) && !sg_drop);
    reg  [31:0] ow_fld;
    always @* case (wf)
        4'd0: ow_fld = {24'b0, n_obs} + 32'd1;
        4'd1: ow_fld = 32'd1;                                  // VSHAPE_RECT
        4'd2: ow_fld = 32'd2;                                  // VTYPE_WALL_WITH_GAP
        4'd3: ow_fld = ({16'b0, sg_x0} + {16'b0, sg_x1}) * 32'd32;
        4'd4: ow_fld = sg_cyq_r;
        4'd5: ow_fld = 32'd0;                                  // radius：矩形没有
        4'd6: ow_fld = ({16'b0, sg_x1} - {16'b0, sg_x0} + 32'd1) * 32'd32;
        4'd7: ow_fld = thick_or_h(sg_thick, bh) * 32'd32;
        default: ow_fld = sg_ang_r;
    endcase
    wire [31:0] ow_da = ow_ang ? ang_wall : ow_fld;

    always @(posedge pix_clk)
        if (ow_en) obs_words[ow_ad] <= ow_da;                  // **唯一**写口

    // ---------------- gap_words ----------------
    wire [11:0] gw_b12 = {4'b0, n_gaps} + {{1{1'b0}}, n_gaps, 3'b0};  // = n_gaps * 9
    wire [8:0]  gw_ad  = gw_b12[8:0] + {5'b0, gw_f};
    wire        gw_en  = gw_busy;
    reg  [31:0] gw_fld;
    always @* case (gw_f)
        4'd0: gw_fld = {24'b0, n_gaps} + 32'd1;
        4'd1: gw_fld = ({16'b0, seg_x1[gp_bi]} +
                        {16'b0, seg_x0[gp_bi + 1'b1]}) * 32'd32;
        4'd2: gw_fld = (seg_cy[gp_bi] + seg_cy[gp_bi + 1'b1] + 32'd1) >> 1;
        4'd3: gw_fld = {16'b0, gp_best} * 32'd64;
        4'd4: gw_fld = 32'd0;
        4'd5: gw_fld = seg_id[gp_bi];
        4'd6: gw_fld = seg_id[gp_bi + 1'b1];
        4'd7: gw_fld = 32'd2;
        default: gw_fld = 32'd1;                               // VGF_RELIABLE
    endcase

    always @(posedge pix_clk)
        if (gw_en) gap_words[gw_ad] <= gw_fld;                 // **唯一**写口

    // atan2 的后半段：拿到 r 之后的查表 + 线性插值。
    // 与 C 参考模型 vr_atan2_q6 的后半段逐位一致；两处 `/4096`、`/4` 换成
    // 算术右移 —— 被除数恒非负（AT 单调递增、frac ≥ 0、v 的符号分支已拆开），
    // 所以 `>>>` 与整除等价。
    function signed [31:0] atan2_post(input [39:0] r_in, input inv, input neg);
        reg signed [31:0] v, dv;
        reg [3:0]  idx;
        reg [31:0] frac;
        begin
            if (r_in >= 40'd32768) v = 32'sd201;
            else begin
                idx  = r_in[15:12];
                frac = {20'b0, r_in[11:0]};
                dv   = $signed({16'b0, vo_at(idx + 4'd1)}) - $signed({16'b0, vo_at(idx)});
                v = $signed({16'b0, vo_at(idx)}) + ((dv * $signed({1'b0, frac})) >>> 12);
            end
            if (inv) v = 32'sd402 - v;
            if (neg) v = -v;
            atan2_post = (v >= 0) ? ((v + 32'sd2) >>> 2) : -(((-v) + 32'sd2) >>> 2);
        end
    endfunction
    wire [7:0]  cur_gray = gray_rd;
    wire [7:0]  cur_step = (({1'b0, cy} + EDGE_AVG) <= {1'b0, band_y1[band_i]})
                           ? abs_diff(gray_rd, sr1) : 8'd0;
    wire [7:0]  step_next = (cur_step > acc_step) ? cur_step : acc_step;
    wire [15:0] next_cnt  = acc_cnt + (wall_rd ? 16'd1 : 16'd0);
    wire [31:0] next_sumy = acc_sumy + (wall_rd ? {16'b0, cy} : 32'd0);
    wire [W_BITS:0]   cx_rel  = cx - x_min;
    wire        flag_now  = (next_cnt >= lim_now) ||
                            ((next_cnt >= r_edge_min_cols) && ({8'b0, step_next} >= el_now));

    // =====================================================================
    // 帧存预读地址 rd_nxt
    //
    // 与状态机原来对 rd_addr 的 **6 处** 赋值逐字对应（那 6 处已全部搬到这里，
    // 状态机里只剩一句 `rd_addr <= rd_nxt`）。同一个值一拍两用：
    //   * 进 BRAM 地址口 —— **提前一拍**，正好抵消 BRAM 的读延迟
    //   * 进 rd_addr 寄存器 —— 时序与改动前完全一致
    // 于是
    //     gray_rd(t) = gray_mem[rd_nxt(t-1)] = gray_mem[rd_addr(t)]
    // 与原来的异步读**逐拍等价**，两个扫描器的判据一个字都不用改。
    //
    // ⚠ 下面这 6 条条件必须与状态机里的一模一样。写错了**不会静默通过**：
    //   core/fpga/cosim 那 620 帧、4.3 亿次逐位对账会当场报差异。
    // =====================================================================
    always @* begin
        rd_nxt = rd_addr;                        // 默认：保持
        case (state)
            // (1) 行带扫描：进入第一条带时定位到带首
            S_SPAN_CLR: begin
                if (!(ctr < WIDTH) && !(n_bands == 0))
                    rd_nxt = base_of(band_y0[0]);
            end
            // (2)(3) 行带扫描：换带时跳到新带首，否则逐像素前进
            S_SPAN: begin
                if (!(band_i >= n_bands)) begin
                    if (sp_rows_left == 16'd0) begin
                        if (band_i + 1'b1 < n_bands)
                            rd_nxt = base_of(band_y0[band_i + 1'b1]);
                    end else begin
                        rd_nxt = rd_addr + 1'b1;
                    end
                end
            end
            // (4) 列扫描：进入时定位到首列的带底
            S_COL_CLEAR: begin
                if (!(band_i >= n_bands) && !(ctr < WIDTH))
                    rd_nxt = base_of(band_y1[band_i]) + x_min;
            end
            // (5)(6) 列扫描：列内自下而上（-WIDTH），列末跳到下一列的带底
            S_COL_SCAN: begin
                if (cy == band_y0[band_i]) begin
                    if (cx != x_max)
                        rd_nxt = base_of(band_y1[band_i]) + cx + 1'b1;
                end else begin
                    rd_nxt = rd_addr - WIDTH[AW-1:0];
                end
            end
            default: ;
        endcase
    end

    always @(posedge pix_clk) begin
        if (!rst_n) begin
            state <= S_IDLE; ctr <= 0;
            dout_vld <= 1'b0; frame_done <= 1'b0;
            frame_done_lat <= 1'b0; frame_count <= 16'd0;
            wr_col <= 0; wr_row <= 0; rc_run <= 0;
            n_bands <= 0; n_obs <= 0; n_gaps <= 0; n_segs <= 0;
            player_valid <= 0; ccl_start <= 0; ccl_started <= 0;
            sgr_started <= 1'b0; div_start <= 1'b0;
            wf <= 4'd0; gw_f <= 4'd0; gw_busy <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            ccl_start  <= 1'b0;
            dout_vld   <= (state == S_PACK);

            // 帧存回读地址：统一由下面的 rd_nxt 组合块给出"下一拍的地址"。
            // 状态机里**不再**直接写 rd_addr —— 原来的 6 处赋值全部搬进了 rd_nxt，
            // 于是同一个值既进 BRAM 地址口（提前一拍，补偿 BRAM 的读延迟），
            // 又进 rd_addr 寄存器（时序与原来完全一致）。
            rd_addr <= rd_nxt;

            case (state)
            // =============================================================
            S_IDLE: begin
                if (frame_start) begin
                    frame_done_lat <= 1'b0;
                    wr_col <= 0; wr_row <= 0; rc_run <= 0;
                    n_bands <= 0; n_obs <= 0; n_gaps <= 0; n_segs <= 0;
                    player_valid <= 1'b0; ccl_started <= 1'b0;
                    bi <= 0; band_phase <= 0; band_in_run <= 0; m_vld <= 0;
                    wf <= 4'd0; gw_f <= 4'd0; gw_busy <= 1'b0;
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
                        if (row_count[bi[H_BITS-1:0]] >= band_limit(r_band_row_pct)) begin
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
                        sp_cx        <= 0;
                    end
                end else begin
                    if (wall_rd) span_count[sp_cx[W_BITS-1:0]] <=
                                     span_count[sp_cx[W_BITS-1:0]] + 16'd1;
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
                        acc_cnt <= 0; acc_sumy <= 0; acc_step <= 0;
                        sr0 <= 8'd0; sr1 <= 8'd0;
                    end
                end else begin
                    cy      <= cy - 1'b1;
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
                seg_k     <= 0;          // 压缩写指针归零（见 seg_k 的声明）
                band_obs0 <= n_obs;      // 本带第一条障碍的下标（带末后处理用）
                state     <= S_SEG;
            end
            // =============================================================
            // 每个墙段：一次扫过它的 x 范围，算 cy/厚度/倾角
            S_SEG: begin
                if ((seg_i >= seg_limit) || (n_obs >= MAX_OBS)) begin
                    // 结算成**压缩后**的段数：丢掉的退化段不能再让 S_GAP 看到，
                    // 否则它会拿"被丢掉的段"当缺口的一端（缺口宽度与两端 id 全错）。
                    seg_limit <= seg_k;
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
                if (sx == sg_x1) state <= S_SG_CYQ;   // 累加完 → 多周期几何
                else             sx <= sx + 1'b1;
            end
            // =============================================================
            // 多周期几何：cyq → lq → rq → ang，每步一次迭代除法（约 40 拍）
            //
            // 每一步的套路一样：
            //   * 进入本状态的第一拍（!sgr_started）：分母为 0 就走退化分支
            //     直接给结果；否则把除法器要的 num/den 摆好并 start
            //   * 之后等 div_done，把商锁存下来，交棒给下一步
            // 退化分支与原来组合写法**完全一致**（含 `sg_cyq_r * 4` 用的是
            // **量化后**的 cyq，不是重新量化 cy*256）。
            // =============================================================
            S_SG_CYQ: begin
                if (!sgr_started) begin
                    sgr_started <= 1'b1;
                    if (sg_total == 32'd0) begin
                        sg_cyq_r    <= ({16'b0, band_y0[band_i]} + {16'b0, band_y1[band_i]}) * 32'd32;
                        sgr_started <= 1'b0;   // 直接跳走：别留下"已发起"，否则下一状态死等 div_done
                        state       <= S_SG_LQ;
                    end else begin
                        // 加宽后必须一并改窄化切片：`total` 上界 230400 需要 18 位
                        div_num   <= {8'b0, sg_sumy} * 40'd128 + {8'b0, sg_total};
                        div_den   <= sg_total[18:0] * 20'd2;
                        div_start <= 1'b1;
                    end
                end else begin
                    div_start <= 1'b0;
                    if (div_done) begin
                        sg_cyq_r    <= div_quo[31:0];
                        sgr_started <= 1'b0;
                        state       <= S_SG_LQ;
                    end
                end
            end
            S_SG_LQ: begin
                if (!sgr_started) begin
                    sgr_started <= 1'b1;
                    if (sg_ln == 32'd0) begin
                        sg_lq_r     <= sg_cyq_r * 32'd4;
                        sgr_started <= 1'b0;
                        state       <= S_SG_RQ;
                    end else begin
                        div_num   <= {8'b0, sg_ls} * 40'd512 + {8'b0, sg_ln};
                        div_den   <= sg_ln[18:0] * 20'd2;
                        div_start <= 1'b1;
                    end
                end else begin
                    div_start <= 1'b0;
                    if (div_done) begin
                        sg_lq_r     <= div_quo[31:0];
                        sgr_started <= 1'b0;
                        state       <= S_SG_RQ;
                    end
                end
            end
            S_SG_RQ: begin
                if (!sgr_started) begin
                    sgr_started <= 1'b1;
                    if (sg_rn == 32'd0) begin
                        sg_rq_r     <= sg_cyq_r * 32'd4;
                        sgr_started <= 1'b0;
                        state       <= S_SG_ANG;
                    end else begin
                        div_num   <= {8'b0, sg_rs} * 40'd512 + {8'b0, sg_rn};
                        div_den   <= sg_rn[18:0] * 20'd2;
                        div_start <= 1'b1;
                    end
                end else begin
                    div_start <= 1'b0;
                    if (div_done) begin
                        sg_rq_r     <= div_quo[31:0];
                        sgr_started <= 1'b0;
                        state       <= S_SG_ANG;
                    end
                end
            end
            S_SG_ANG: begin
                if (!sgr_started) begin
                    sgr_started <= 1'b1;
                    // 与 C 的 vr_atan2_q6 前半段一致：
                    //   x_q8 <= 0 或 y_q8 == 0 → 0；否则取 |y|、x，必要时对调
                    if (sg_dx_c <= 32'sd0 || sg_ang_y == 32'sd0) begin
                        sg_ang_r    <= 32'sd0;
                        sgr_started <= 1'b0;
                        state       <= S_SEG_FIN;
                    end else begin
                        div_num   <= {6'b0, an_a_w} <<< 15;   // (uint32)a << 15
                        div_den   <= an_b_w[19:0];
                        div_start <= 1'b1;
                    end
                end else begin
                    div_start <= 1'b0;
                    if (div_done) begin
                        sg_ang_r    <= atan2_post(div_quo, an_inv_w, (sg_ang_y < 0));
                        sgr_started <= 1'b0;
                        state       <= S_SEG_FIN;
                    end
                end
            end
            S_SEG_FIN: begin
                // ------------------------------------------------------------------
                // 不变量 `half_w >= half_h`（half_w 是长边）。
                //
                // 段短于厚度时 half_w 会小于 half_h，PS 侧
                // `decode_vision_frame().validate()` 会**当场抛异常，整条链断**。
                // 实测真实照片 32 张里 6 张触发（如 hw=5.0 < hh=8.5）；合成图永远
                // 产不出这种段，所以 620 帧逐位对账全绿也照不出来。
                //
                // 处理办法与 core/vision/to_standard.py 的 `_segment_obstacle` 一致：
                // **丢掉**（返回 None），而不是交换长短边 —— 交换会把一个碎片
                // 说成"一堵竖直的墙"，那是编造出来的几何。
                //
                // 丢掉时要**同时**做三件事，缺一不可：
                //   ① 不写 obs_words（不占障碍名额）；
                //   ② `seg_k` 不前进（保留下来的段才占紧凑下标）；
                //   ③ `seg_i` 照常前进（继续处理下一条）。
                // 而且 `seg_x0/seg_x1` 要**回写到 seg_k**，让 S_GAP 看到的是
                // 压缩后的相邻关系。三条漏一条，缺口的两端就会错位。
                // ------------------------------------------------------------------
                if (sg_drop) begin
                    if (dbg_dump_seg)
                        $display("V  seg x=[%0d,%0d] **丢掉**（half_w < half_h）",
                                 sg_x0, sg_x1);
                    seg_i <= seg_i + 1'b1;
                    wf    <= 4'd0;
                    state <= S_SEG;
                end else if (wf == 4'd8) begin
                    // 第 8 个字段在本拍已经提交（§5.4 的 ow_en/ow_ad）。收尾必须与
                    // 最后一个字段**同拍**：`ow_ad` 组合自**当前**（未更新的）n_obs，
                    // 所以"先写后加"的顺序天然成立。
                    seg_x0[seg_k] <= sg_x0;
                    seg_x1[seg_k] <= sg_x1;
                    seg_id[seg_k] <= n_obs + 1'b1;
                    seg_cy[seg_k] <= sg_cyq_r;
                    seg_k <= seg_k + 1'b1;
                    n_obs <= n_obs + 1'b1;
                    seg_i <= seg_i + 1'b1;
                    wf    <= 4'd0;
                    state <= S_SEG;
                end else begin
                    if (dbg_dump_seg && (wf == 4'd0))
                        $display("V  seg x=[%0d,%0d] span=%0d ls=%0d ln=%0d rs=%0d rn=%0d lq=%0d rq=%0d dx=%0d ang=%0d",
                                 sg_x0, sg_x1, sg_span, sg_ls, sg_ln, sg_rs, sg_rn,
                                 sg_lq_r, sg_rq_r, sg_dx_c, sg_ang_r);
                    wf <= wf + 1'b1;
                end
            end
            // =============================================================
            // 带末后处理：**整堵墙共用一个朝向**
            //
            // 为什么必须有：决策的 `group_walls` 分桶键是
            // `(int(round(rotation*1000)), int(round(band/2)))` —— 把角度差异
            // **放大 1000 倍**。逐段各自拟合的角差 0.01 rad（PL 的定点 atan2 是
            // Q6，一格就 0.0156 rad）会让**同一堵墙的两段落进不同的桶**，
            // `plan()` 于是永远重建不出缺口、一直喊 NO_GAP。
            //
            // 实测（合成场景 16 s、48 个决策时刻）：逐段角度 → 与 Python 的 tier
            // **48/48 全不同**；换成"整堵墙共用" → **0/48**。这是分水岭。
            //
            // 与 core/vision/to_standard.py 的 `_wall_rotation` 同义：段心连线的
            // 总体最小二乘主方向。本场景实测每堵墙**恰好 2 段**（38/38 个墙带），
            // 两点情形的主方向就是两点连线，所以"首段心 → 末段心"与 Python 的 TLS
            // **精确等价**。
            //
            // `seg_k <= 1` 时什么都不用做：单段本来就用它自己的角（C 侧同样是
            // `kang_q[0]`），而 S_SEG_FIN 已经那么写了。
            //
            // ⚠ `aw_dx_w <= 0` 这一支**不可达**：keep[] 保序、段自左向右，
            //   所以末段心的 x 恒 > 首段心。C 侧 `dx != 0` 也走不到。这里显式
            //   挡住只是为了别在 RTL 里出现除零，不改变可达行为。
            // =============================================================
            S_ANG_INIT: begin
                if ((seg_k <= 9'd1) || (aw_dx_w <= 32'sd0)) begin
                    band_i <= band_i + 1'b1;
                    ctr    <= 0;
                    state  <= S_COL_CLEAR;
                end else begin
                    div_num   <= {22'b0, aw_a_w[17:0]} <<< 15;
                    div_den   <= aw_b_w[19:0];
                    div_start <= 1'b1;
                    state     <= S_ANG_DIV;
                end
            end
            S_ANG_DIV: begin
                div_start <= 1'b0;
                if (div_done) begin
                    ang_wall <= atan2_post(div_quo, aw_inv_w, aw_neg_w);
                    ang_i    <= band_obs0;
                    state    <= S_ANG_WR;
                end
            end
            S_ANG_WR: begin
                if (ang_i >= n_obs) begin
                    band_i <= band_i + 1'b1;
                    ctr    <= 0;
                    state  <= S_COL_CLEAR;
                end else begin
                    // `obs_words[ang_i*9 + 8] <= ang_wall` 交给 §5.4 的单写口：
                    // ow_ang = (state == S_ANG_WR)，ow_en 另加 ang_i < n_obs。
                    ang_i <= ang_i + 1'b1;
                end
            end
            // =============================================================
            S_GAP: begin
                if (gw_busy) begin
                    // 逐字段写记录（§5.4）：gw_en = gw_busy，gw_ad = n_gaps*9 + gw_f。
                    // 这 9 拍里 gp_bi / gp_vld / seg_* 都保持不变，所以 gw_fld 的
                    // 组合读数稳定。
                    if (gw_f == 4'd8) begin
                        gw_f    <= 4'd0;
                        gw_busy <= 1'b0;
                        n_gaps  <= n_gaps + 1'b1;      // 第 8 个字段已用旧 n_gaps 写完
                        gp_vld  <= 1'b0;
                        state   <= S_ANG_INIT;
                    end else begin
                        gw_f <= gw_f + 1'b1;
                    end
                end else if (gi + 1'b1 >= seg_limit) begin
                    if (gp_vld && (n_gaps < MAX_GAPS)) begin
                        gw_f    <= 4'd0;
                        gw_busy <= 1'b1;               // 下一拍起逐字段写
                    end else begin
                        gp_vld <= 1'b0;
                        // 先做**整堵墙朝向**的后处理，再进下一个带（见 S_ANG_INIT）。
                        // 从前这里直接 `band_i <= band_i+1; state <= S_COL_CLEAR;`
                        state  <= S_ANG_INIT;
                    end
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
    // 6. 调试/回读口
    //
    // 除掩膜像素外都是**组合**读（row_count / band / seg / obs / gap / player …）。
    //
    // sel=0/1/2 读三张掩膜的像素**走寄存读**，而且**只在帧间（IDLE/DONE）有效**：
    //   BRAM 没有组合读口，若在这里写 `gray_mem[dbg_idx]`，Vivado 就只能把整张
    //   帧存退回分布式 RAM —— 那正是本设计装不下 030 的原因。所以这三项复用
    //   扫描器的读口（见上面 mem_ra 的仲裁），代价是帧内读回来的是扫描器的数据。
    //   调试用足够：一帧跑完（frame_done）之后再读即可。
    // =====================================================================
    always @* begin
        case (dbg_sel)
            4'd0:  dbg_data = {24'b0, gray_rd};
            4'd1:  dbg_data = {31'b0, wall_rd};
            4'd2:  dbg_data = {31'b0, bright_rd};
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
