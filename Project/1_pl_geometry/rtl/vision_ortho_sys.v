// ===========================================================================
// vision_ortho_sys.v —— P0 正射子系统（行缓存 + AXI 读主口 + 填充控制 + 正射主体）
//
// 这一层是**可上板的最小闭环**：源帧放在 DDR（PS 用 udmabuf），PL 通过一个 AXI4
// 读主口按行取数（一行 1920 B = 240 个 64 位字），4 行窗口命中就不再读 DDR，
// 正射主体把 (line, px) 交给行缓存、拿回 24 位像素，输出页面像素流。
//
// 为什么 4 行窗口够（数出来的，不是猜的）：
//   v 沿一页行扫过去每行只走几个像素（实测 Δy 中位数 0.35 px），
//   而 v 的变化又主要跟着扫描行号走 —— 所以"同一输出行里用到的相机行"是连续的
//   少数几行；4 槽正好覆盖连续的 4 行（slot = line[1:0]）。
//   如果真的跨了 4 行以外，只会表现为**多几次 miss/填充**（功能仍对），不会错位。
//
// ⚠ seq 依赖：`start` 必须等 `done`（或 busy 拉低）后再给，本层不做请求排队。
// ===========================================================================
module vision_ortho_sys #(
    parameter integer PL_W = 640,
    parameter integer PL_H = 360,
    parameter integer SRC_W = 640,
    parameter integer SRC_H = 480
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    //: 下游还没准备好收像素（接 vision_top 的 busy 取反）；见 vision_ortho 的口径说明。
    input  wire        hold,
    output wire        busy,
    output wire        done,

    // ---- 配置（PS 写；系数由 core/fpga/tools/ortho_fixed_ref.cpp 同一个式子给出）----
    input  wire signed [31:0] h0, h1, h2,
    input  wire signed [31:0] h3, h4, h5,
    input  wire signed [31:0] h6, h7, h8,
    input  wire signed [39:0] PY0_16, PS_16,
    input  wire signed [63:0] PY0_24, PS_24,
    input  wire        [9:0]  c0, c1,
    input  wire        [9:0]  r0, r1,

    //: 相机帧在 DDR 里的**物理基址**（PS 写 FRMBASE 进来）。行内偏移由 linefill 给，
    //: 真正的 AXI 地址 = frm_base + line*1920。
    //: ⚠ 踩过：忘了加这个基址，PL 就从 DDR 地址 0（而不是帧起点）读 —— 板上表现为
    //:   "row_count 全 0（看不见任何墙）、几何帧短一截"，而本地 cosim 因为 FRMBASE=0 全对。
    input  wire [31:0] frm_base,

    // ---- AXI4 读主口（接 BD 的 HP0）----
    output wire [31:0] m_araddr,
    output wire [7:0]  m_arlen,
    output wire        m_arvalid,
    input  wire        m_arready,
    input  wire [63:0] m_rdata,
    input  wire        m_rlast,
    input  wire        m_rvalid,
    output wire        m_rready,

    // ---- 输出像素流（页面 640×360，只吐 c0..c1-1 / r0..r1 内的像素）----
    output wire        out_vld,
    output wire        out_sof,
    output wire        out_eol,
    output wire [7:0]  out_r,
    output wire [7:0]  out_g,
    output wire [7:0]  out_b,

    // ---- 只读调试口（R7：没有中间量就只能猜）----
    output wire [15:0] dbg_src_line,
    output wire [9:0]  dbg_src_px,
    output wire        dbg_miss,
    output wire [15:0] dbg_fill_line,
    output wire [15:0] dbg_tag,
    output wire        dbg_hit,
    output wire [14:0] dbg_rd_idx,
    output wire [15:0] dbg_fills,
    output wire [7:0]  dbg_fill_state,
    output wire [15:0] dbg_axi_left,
    output wire [1:0]  dbg_axi_state,
    output wire        dbg_blk,   // {行缓存填充状态[2:0], 已写像素数[2:0], 相位[1:0]}
    output wire [5:0]  dbg_flags     // {行缓存填充忙, 填充完成脉冲, master 忙, 填充状态机}
);
    // ---- ortho ↔ 行缓存 ----
    wire [15:0] o_line;
    wire [9:0]  o_px;
    wire        o_req;
    wire [23:0] lc_data;
    wire        lc_valid;

    // ---- 行缓存 ↔ 填充控制 ----
    wire [15:0] lc_line;
    wire [9:0]  lc_px;
    wire        lc_req;
    wire        lc_fill_ready;
    wire        lc_miss;
    wire [15:0] lc_miss_line;
    wire        f_start;
    wire [63:0] f_data;
    wire        f_valid;
    wire        f_last;

    // ---- 填充控制 ↔ AXI 主口 ----
    wire        m_start;
    wire [31:0] m_base;
    wire [15:0] m_beats;
    wire        m_done;
    //: ⚠ 行缓存要的是 **AXI 主口**的 m_*（已按 S_DATA 状态门控），
    //:   不能直连裸的 rdata/rvalid —— 那样 rvalid 在非数据相位也会被当成有效字。
    wire [63:0] m_vo_data;
    wire        m_vo_valid;
    wire        m_vo_last;
    //: 这四位只给调试/统计（不让它们悬空：悬空口会让 lint 报 PINCONNECTEMPTY，
    //: 而 lint 门禁是 -Wall 全过，所以"留着看"的信号必须真的接出来）。
    wire        lc_fill_busy, lc_fill_done, m_busy;
    wire [1:0]  f_state;
    wire [2:0]  lc_fstate, lc_ip;
    wire [1:0]  lc_ph;

    //: {行缓存填充忙, 填充完成脉冲, master 忙, master 完成脉冲, 填充状态机}
    assign dbg_flags = {lc_fill_busy, lc_fill_done, m_busy, m_done, f_state};

    vision_ortho #(
        .PL_W(PL_W), .PL_H(PL_H), .SRC_W(SRC_W), .SRC_H(SRC_H)
    ) u_ortho (
        .clk(clk), .rst_n(rst_n),
        .start(start), .hold(hold), .busy(busy), .done(done),
        .h0(h0), .h1(h1), .h2(h2), .h3(h3), .h4(h4), .h5(h5), .h6(h6), .h7(h7), .h8(h8),
        .PY0_16(PY0_16), .PS_16(PS_16), .PY0_24(PY0_24), .PS_24(PS_24),
        .c0(c0), .c1(c1), .r0(r0), .r1(r1),
        .src_line(o_line), .src_px(o_px), .src_req(o_req),
        .src_rgb(lc_data), .src_valid(lc_valid),
        .out_vld(out_vld), .out_sof(out_sof), .out_eol(out_eol),
        .out_r(out_r), .out_g(out_g), .out_b(out_b),
        .dbg_blk(dbg_blk)
    );

    vision_linecache #(
        .PLW(640), .NLINES(32), .IDXW(15)
    ) u_lc (
        .clk(clk), .rst_n(rst_n),
        .line(lc_line), .px(lc_px), .req(lc_req),
        .data(lc_data), .data_valid(lc_valid),
        .miss(lc_miss), .miss_line(lc_miss_line),
        .fill_start(f_start), .fill_data(f_data), .fill_valid(f_valid), .fill_last(f_last),
        .fill_busy(lc_fill_busy), .fill_done(lc_fill_done),
        .fill_ready(lc_fill_ready),
        .dbg_slot(o_line[4:0]), .dbg_tag(dbg_tag), .dbg_hit(dbg_hit), .dbg_rd_idx(dbg_rd_idx),
        .dbg_fstate(lc_fstate), .dbg_ip(lc_ip), .dbg_ph(lc_ph)
    );

    vision_linefill #(
        .LINE_BYTES(1920)
    ) u_fill (
        .clk(clk), .rst_n(rst_n),
        .line(o_line), .px(o_px), .req(o_req),
        .lc_line(lc_line), .lc_px(lc_px), .lc_req(lc_req),
        .lc_miss(lc_miss), .lc_miss_line(lc_miss_line),
        .fill_start(f_start), .fill_data(f_data), .fill_valid(f_valid), .fill_last(f_last),
        .m_start(m_start), .m_base(m_base), .m_beats(m_beats),
        .m_busy(m_busy), .m_data(m_vo_data), .m_valid(m_vo_valid), .m_last(m_vo_last),
        .fill_done(lc_fill_done),
        .dbg_fills(dbg_fills), .dbg_state(f_state)
    );

    vision_axi_master #(.AW(32), .DW(64)) u_master (
        .clk(clk), .rst_n(rst_n),
        .start(m_start), .base(m_base + frm_base), .beats(m_beats),
        .busy(m_busy), .done(m_done),
        .m_valid(m_vo_valid), .m_data(m_vo_data), .m_last(m_vo_last),
        .dbg_left(dbg_axi_left), .dbg_state(dbg_axi_state),
        .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rdata(m_rdata), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready),
        .r_en(lc_fill_ready)
    );

    assign dbg_fill_state = {lc_fstate, lc_ip, lc_ph};
    assign dbg_src_line  = o_line;
    assign dbg_src_px    = o_px;
    assign dbg_miss      = lc_miss;
    assign dbg_fill_line = lc_miss_line;
endmodule
