// ===========================================================================
// vision_ortho.v —— 正射主体：源像素（相机帧）→ 页面 640×360 像素流
//
// 算术与 `core/fpga/tools/ortho_fixed_ref.cpp` **逐位一致**（R16 定稿）：
//   每行：au/bu、av/bv（Q16.16）、ad/bd（Q8.24）
//   每像素：nd = ad*c + bd;  nd==0 或 |nd|<1024 → 输黑
//           u  = ((au*c+bu) << 8) * 65536 / nd  → 再 -0.5 取整（Q16.16 → 像素）
//           v  同理；钳到 [0, w-1] / [0, h-1]
//
// 结构（2026-10-25 重写，理由见 R62 的数：旧版 137 拍/像素 ≈ 380 ms/帧）：
//
//   行系数（每行一次，组合 + 寄存）
//        │
//        ▼
//   ISSUE  ：每拍算 nu/nv/nd，取绝对值，送进**两台流水线除法器**（II=1）
//        │      c 从 0 数到 PL_W-1：c∉[C0,C1) 的像素直接标黑（不取源像素）
//        ▼
//   DIV×2  ：vision_div_pipe（34 位商，17 级 radix-4），与 meta 同步下移
//        ▼
//   FETCH  ：算 ui/vi → 钳位 → 向行缓存发 (line, off)，等 src_valid
//        │      命中：2 拍/像素；标黑：1 拍/像素；miss：冻结到填行完成
//        ▼
//   输出：每行 PL_W 个像素，out_sof=行首、out_eol=行尾（与 vision_top 的 din_* 同口径）
//
// ⚠ 为什么输出**整行 640 个**（不是只吐 [C0,C1) 的 480 个）：下游 `vision_top` 收
//   640×360 帧、靠 din_sof/din_eol 定位行首行尾，没有任何背压。左右黑边带由本模块
//   自己补成黑像素，下游就不用再写一个"插黑边"的状态机（少一层没人验过的逻辑）。
//   代价是每帧多 5.76 万个 1 拍的像素（≈1.2 ms @50 MHz），换掉一层胶水，划算。
// ===========================================================================
module vision_ortho #(
    //: 页面（输出）尺寸。参数必须被**引用**，否则 lint 报 UNUSEDPARAM；
    //: 这里用它做输出列/行范围的兜底编译期检查。
    parameter integer PL_W = 640,
    parameter integer PL_H = 360,
    parameter integer SRC_W = 640,     // **相机帧**尺寸（钳位与地址步长都用它）
    parameter integer SRC_H = 480
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    //: 下游"这一拍还不收像素"（接 vision_top 的 busy 取反）：冻结整条流水线。
    //: 为什么必须真有这个口：`vision_top` **没有背压**，像素到门口就得收下；
    //: 它上电/每帧先进 S_HCLR 准备直方图（约 257 拍，busy=0），这段时间吐出去的
    //: 像素会被静默丢掉 → 整帧平移。冻结比"吐出去再补"简单，也不会错。
    input  wire        hold,
    output reg         busy,
    output reg         done,

    // ---- 配置（PS 写）----
    input  wire signed [31:0] h0, h1, h2,
    input  wire signed [31:0] h3, h4, h5,
    input  wire signed [31:0] h6, h7, h8,
    input  wire signed [39:0] PY0_16, PS_16,
    input  wire signed [63:0] PY0_24, PS_24,
    //: ⚠ 列范围必须 **10 位**：c1 = 560 > 511（9 位装不下）。写成 [8:0] 时
    //:   560 被截成 48 → `band=(c<C0)||(c>=C1)` 恒真 → **整帧发黑**，而画布尺寸、
    //:   帧长、CRC 全都自洽（板上实测：4.62 ms "跑完"一帧、fills=0、FLEN 有值）。
    //:   这是"看着像跑通了，其实一个像素都没取"的典型，靠**回读寄存器**抓出来的。
    input  wire        [9:0]  c0, c1,          // 页面列范围（范围外输黑）
    input  wire        [9:0]  r0, r1,          // 页面行范围

    // ---- 源像素读口（相机帧，行主序）----
    //: 源像素请求（给行缓存）：行号 + 行内**像素列** + 请求（**电平**，不是单拍脉冲）。
    //: 未命中时行缓存要先去 DDR 填行（几百拍），所以请求必须**保持到 src_valid 回来**。
    output wire [15:0] src_line,
    output wire [9:0]  src_px,
    output wire        src_req,
    input  wire [23:0] src_rgb,
    input  wire        src_valid,              // 与 src_rgb 同拍；命中后 1 拍

    // ---- 输出像素流（给 vision_top 的 din_*）----
    output reg         out_vld,
    output reg         out_sof,
    output reg         out_eol,
    output reg  [7:0]  out_r, out_g, out_b,

    //: 只读调试：本拍发出的是"黑"（边带或 |nd| 退化）而不是取源像素
    output wire        dbg_blk
);
    // =====================================================================
    // 0. 参数检查
    // =====================================================================
    // synthesis translate_off
    initial begin
        if (PL_W > SRC_W || PL_H > SRC_H)
            $error("vision_ortho: 页面尺寸 %0dx%0d 超过相机尺寸 %0dx%0d", PL_W, PL_H, SRC_W, SRC_H);
    end
    // synthesis translate_on

    localparam integer NST = 17;                       // 除法流水级数（与 vision_div_pipe 一致）
    localparam [63:0]  QSAT = 64'd17179869183;         // 2^34-1：商饱和值（=最大可表示 u*2^24）

    // =====================================================================
    // 1. 行系数（与 oracle 同一串运算；R7「由点及面」：全部显式符号扩展）
    // =====================================================================
    reg  [9:0] r;                                      // 当前**输出行**（与 r1 同为 10 位）
    reg  [9:0] c;                                      // 当前输出列（0..PL_W-1）

    wire signed [63:0] H0 = {{32{h0[31]}}, h0};
    wire signed [63:0] H1 = {{32{h1[31]}}, h1};
    wire signed [63:0] H2 = {{32{h2[31]}}, h2};
    wire signed [63:0] H3 = {{32{h3[31]}}, h3};
    wire signed [63:0] H4 = {{32{h4[31]}}, h4};
    wire signed [63:0] H5 = {{32{h5[31]}}, h5};
    wire signed [63:0] H6 = {{32{h6[31]}}, h6};
    wire signed [63:0] H7 = {{32{h7[31]}}, h7};
    wire signed [63:0] H8 = {{32{h8[31]}}, h8};

    wire [9:0] C0 = c0;
    wire [9:0] C1 = c1;
    wire signed [63:0] R    = {{54{1'b0}}, r};   // r 是 10 位 → 54+10 = 64
    wire signed [63:0] PY16 = {{24{PY0_16[39]}}, PY0_16};
    wire signed [63:0] PS16 = {{24{PS_16[39]}}, PS_16};
    wire signed [63:0] py16 = PY16 - PS16 * R;
    wire signed [63:0] py24 = PY0_24 - PS_24 * R;
    wire signed [63:0] au64 = (H0 * 64'sd87381) >>> 16;
    //: 把 bu 的三项拆开（R7：先取证。之前只打 bu 一个数，看不出是哪个操作数错）
    wire signed [63:0] bu_t1 = (H0 * -64'sd6946816) >>> 16;
    wire signed [63:0] bu_t2 = (H1 * py16) >>> 16;
    wire signed [63:0] bu_t3 = H2;
    wire signed [63:0] bu64 = bu_t1 + bu_t2 + bu_t3;
    wire signed [63:0] av64 = (H3 * 64'sd87381) >>> 16;
    wire signed [63:0] bv64 = ((H3 * -64'sd6946816) >>> 16) + ((H4 * py16) >>> 16) + H5;
    wire signed [63:0] ad64 = (H6 * 64'sd22369621) >>> 24;
    wire signed [63:0] bd64 = ((H6 * -64'sd1778384896) >>> 24) + ((H7 * py24) >>> 24) + H8;

    reg signed [63:0] au_r, bu_r, av_r, bv_r, ad_r, bd_r;

    // =====================================================================
    // 2. ISSUE：每拍一个像素
    // =====================================================================
    reg  [2:0] st;
    localparam [2:0] S_IDLE = 3'd0, S_SETUP = 3'd1, S_RUN = 3'd2, S_DRAIN = 3'd3;
    //: ⚠ start 必须**在 ce 之外**记录：`hold=~vision_top.busy` 时，vision_top 每帧
    //: 先进 S_HCLR（≈257 拍 busy=0）→ 那段时间 ce=0，整段 FSM 冻结；如果 start 的
    //: 单拍脉冲也在 ce 里面采样，它就被吞掉了（表现为"板上按了 start 但一帧都不出"）。
    //: 所以先登记 start_pend（不受 ce 限制），S_IDLE 里再消费它。
    reg        start_pend;

    wire signed [63:0] Cs = {{54{1'b0}}, c};
    wire signed [63:0] nu_c = au_r * Cs + bu_r;
    wire signed [63:0] nv_c = av_r * Cs + bv_r;
    wire signed [63:0] nd_c = ad_r * Cs + bd_r;
    //: 黑边带：不取源像素（把三个中间量换成 0/0/1024，除法结果干净且必然"不坏"）
    wire        band  = (c < C0) || (c >= C1);
    wire signed [63:0] nu_i = band ? 64'sd0 : nu_c;
    wire signed [63:0] nv_i = band ? 64'sd0 : nv_c;
    wire signed [63:0] nd_i = band ? 64'sd1024 : nd_c;

    wire        nd_bad = (nd_i < 64'sd1024) && (nd_i > -64'sd1024);
    wire        u_sign = (nu_i < 0) ^ (nd_i < 0);
    wire        v_sign = (nv_i < 0) ^ (nd_i < 0);
    wire [63:0] u_abs  = (nu_i < 0) ? -nu_i : nu_i;     // 先取绝对值再移位（与 C 的截断整除同义）
    wire [63:0] v_abs  = (nv_i < 0) ? -nv_i : nv_i;
    wire [63:0] d_abs  = (nd_i < 0) ? -nd_i : nd_i;

    //: 除法器的输入：Q = |nu|·2^24 / |nd|
    wire [63:0] num_u = u_abs << 24;
    wire [63:0] num_v = v_abs << 24;

    wire       iss_vld = (st == S_RUN);
    //: meta 沿流水线一起下移：{黑边, nd坏, u符号, v符号}
    wire [3:0] meta_in = {band, nd_bad, u_sign, v_sign};

    // =====================================================================
    // 3. 两台流水线除法器（u 与 v 并行；II=1）
    // =====================================================================
    wire [33:0] quo_u, quo_v;
    wire        ovf_u, ovf_v;
    //: 两台除法器输入完全一样（同 in_vld / 同 ce），valid 必然同拍 —— 所以两者相与：
    //: 既让第二台的 out_vld 有落点（悬空口会被 lint 的 PINCONNECTEMPTY 挡），
    //: 又顺手把"两台不同步"变成一个不会静默通过的错。
    wire        div_vld_u, div_vld_v, div_vld;
    assign      div_vld = div_vld_u & div_vld_v;

    vision_div_pipe #(.WN(64), .WQ(34)) u_du (
        .clk(clk), .rst_n(rst_n), .ce(ce),
        .in_vld(iss_vld), .num(num_u), .den(d_abs),
        .quo(quo_u), .ovf(ovf_u), .out_vld(div_vld_u)
    );
    vision_div_pipe #(.WN(64), .WQ(34)) u_dv (
        .clk(clk), .rst_n(rst_n), .ce(ce),
        .in_vld(iss_vld), .num(num_v), .den(d_abs),
        .quo(quo_v), .ovf(ovf_v), .out_vld(div_vld_v)
    );

    //: meta 延迟线（级数与除法器一致：输入级 1 + NST 级）
    reg [3:0] meta_p [0:NST];
    integer   mi;

    //: 时钟使能：FETCH 在等源数据时冻结整条流水线（像素顺序不能乱）。
    //: ⚠ 声明必须在除法器实例之前（Verilog 的隐式网规则在这里会悄悄建一个 1 位网，
    //:   到综合/仿真才发现"我的 ce 没接上"）。
    wire ce;

    // =====================================================================
    // 4. FETCH / 输出：算 ui/vi、发请求、等数据、出像素
    // =====================================================================
    localparam [1:0] F_IDLE = 2'd0, F_WAIT = 2'd1;
    reg  [1:0]  fst;
    reg  [9:0]  ocol;                                  // 输出列（0..PL_W-1）
    reg  [9:0]  orow;                                  // 输出行（r0..r1）
    reg  [15:0] line_h;
    reg  [9:0]  px_h;

    //: 饱和：商超 34 位 ⇒ |u| ≥ 1024 ⇒ 一定被钳到边界（与"精确算完再钳"等价）
    wire [63:0] uq_u = ovf_u ? QSAT : {30'd0, quo_u};
    wire [63:0] vq_v = ovf_v ? QSAT : {30'd0, quo_v};
    wire signed [63:0] uq = meta_p[NST][1] ? -$signed(uq_u) : $signed(uq_u);
    wire signed [63:0] vq = meta_p[NST][0] ? -$signed(vq_v) : $signed(vq_v);
    wire signed [63:0] ui_c0 = (uq - 64'sd32768) >>> 16;
    wire signed [63:0] vi_c0 = (vq - 64'sd32768) >>> 16;
    //: 钳位后的结果**直接收窄**成像素宽度：SRC_W≤1024 时 u 只可能是 0..SRC_W-1。
    //: 若保留 64 位 wire，高 53 位没人用（lint UNUSEDSIGNAL 会报，而且那是真话：
    //: 下游确实只用低位，宽了只是浪费布线）。
    wire signed [63:0] ui_lim = {{32{1'b0}}, SRC_W} - 64'sd1;
    wire signed [63:0] vi_lim = {{32{1'b0}}, SRC_H} - 64'sd1;
    //: u 的钳位结果 ≤ SRC_W-1 = 639 → 10 位就够（11 位会留一位没人用，lint 会报）
    wire [9:0]  ui_px = (ui_c0 < 0) ? 10'd0 : (ui_c0 > ui_lim) ? ui_lim[9:0] : ui_c0[9:0];
    wire [15:0] vi_px = (vi_c0 < 0) ? 16'd0 : (vi_c0 > vi_lim) ? vi_lim[15:0] : vi_c0[15:0];

    //: 行缓存按**像素**存（1 写口 1 读口，BRAM 可推断），所以这里直接给像素列；
    //: 不再有 `off = ui*3` —— 那个字节偏移是"按字节存"那版的遗产。
    wire [15:0] line_x = vi_px;
    wire [9:0]  px_x   = ui_px;
    wire        blk_x  = meta_p[NST][3] || meta_p[NST][2];

    assign dbg_blk  = div_vld && blk_x;
    assign src_line = (fst == F_IDLE) ? line_x : line_h;
    assign src_px   = (fst == F_IDLE) ? px_x   : px_h;
    assign src_req  = (fst == F_IDLE) ? (div_vld && !blk_x) : 1'b1;

    //: ⚠ ce 只跟"FETCH 是否在取走流水线末级"挂钩：F_IDLE 时取，F_WAIT 时**不取**。
    //:   另外 hold=1（下游还没准备好）时一律不取。
    //:   第一版写成 `(fst==F_IDLE) || src_valid` —— 于是在 F_WAIT 那一拍流水线也前进，
    //:   末级的下一个像素被顶掉（每帧正好丢 1 个像素，表现为 ocol 停在 639、done 永不拉高）。
    assign ce = (fst == F_IDLE) && !hold;

    always @(posedge clk) begin
        if (!rst_n) begin
            r <= 10'd0; c <= 10'd0; st <= S_IDLE;
            au_r <= 0; bu_r <= 0; av_r <= 0; bv_r <= 0; ad_r <= 0; bd_r <= 0;
            busy <= 1'b0; done <= 1'b0;
            fst <= F_IDLE; ocol <= 10'd0; orow <= 10'd0; start_pend <= 1'b0;
            line_h <= 16'd0; px_h <= 10'd0;
            out_vld <= 1'b0; out_sof <= 1'b0; out_eol <= 1'b0;
            out_r <= 8'd0; out_g <= 8'd0; out_b <= 8'd0;
            for (mi = 0; mi <= NST; mi = mi + 1) meta_p[mi] <= 4'd0;
        end else begin
            done    <= 1'b0;
            out_vld <= 1'b0;
            //: start 登记**不受 ce 限制**（见 start_pend 处的说明）
            if (start) start_pend <= 1'b1;

            // ---- meta 延迟线（与除法流水同步；ce=0 时冻结）----
            if (ce) begin
                meta_p[0] <= meta_in;
                for (mi = 0; mi < NST; mi = mi + 1) meta_p[mi+1] <= meta_p[mi];
            end

            // ---- FETCH / 输出 ----
            if (fst == F_IDLE) begin
                if (div_vld) begin
                    if (blk_x) begin
                        //: 黑边带 / 退化分母：不取源像素，1 拍出
                        out_vld <= 1'b1; out_r <= 8'd0; out_g <= 8'd0; out_b <= 8'd0;
                        out_sof <= (ocol == 10'd0) && (orow == r0);
                        out_eol <= (ocol == PL_W[9:0] - 10'd1);
                        ocol <= (ocol == PL_W[9:0] - 10'd1) ? 10'd0 : ocol + 10'd1;
                        if (ocol == PL_W[9:0] - 10'd1) begin
                            if (orow == r1) begin done <= 1'b1; busy <= 1'b0; end
                            else orow <= orow + 10'd1;
                        end
                    end else begin
                        //: 请求这一拍就发出去（组合），下一拍数据就回来
                        line_h <= line_x; px_h <= px_x;
                        fst <= F_WAIT;
                    end
                end
            end else begin
                if (src_valid) begin
                    out_vld <= 1'b1; out_sof <= (ocol == 10'd0) && (orow == r0);
                    out_eol <= (ocol == PL_W[9:0] - 10'd1);
                    out_r <= src_rgb[23:16]; out_g <= src_rgb[15:8]; out_b <= src_rgb[7:0];
                    ocol <= (ocol == PL_W[9:0] - 10'd1) ? 10'd0 : ocol + 10'd1;
                    if (ocol == PL_W[9:0] - 10'd1) begin
                        if (orow == r1) begin done <= 1'b1; busy <= 1'b0; end
                        else orow <= orow + 10'd1;
                    end
                    fst <= F_IDLE;
                end
            end

            // ---- ISSUE ----
            if (ce) begin
                case (st)
                S_IDLE: if (start_pend) begin
                    start_pend <= 1'b0;
                    r <= r0; c <= 10'd0; busy <= 1'b1; st <= S_SETUP;
                    //: 输出侧的列/行也必须从这里对齐：r0 不一定是 0
                    orow <= r0; ocol <= 10'd0; fst <= F_IDLE;
                end
                S_SETUP: begin
                    //: 这里读的是组合链（用的是**当前** r），所以必须先定 r 再进 SETUP
                    au_r <= au64; bu_r <= bu64;
                    av_r <= av64; bv_r <= bv64;
                    ad_r <= ad64; bd_r <= bd64;
                    st   <= S_RUN;
                end
                S_RUN: begin
                    if (c == PL_W[9:0] - 10'd1) begin
                        c <= 10'd0;
                        if (r == r1) st <= S_DRAIN;
                        else begin r <= r + 10'd1; st <= S_SETUP; end
                    end else c <= c + 10'd1;
                end
                S_DRAIN: if (fst == F_IDLE && div_vld == 1'b0) begin
                    st <= S_IDLE;         //: 最后一个像素已经出完（done 在输出侧拉高）
                end
                default: st <= S_IDLE;
                endcase
            end
        end
    end
endmodule
