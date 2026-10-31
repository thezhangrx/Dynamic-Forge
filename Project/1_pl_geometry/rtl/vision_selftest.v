// ===========================================================================
// vision_selftest.v —— **不接相机、不接 PS 也能跑**的 PL 自检顶
//
//   vision_tpg（合成图案）→ vision_top（视觉流水）→ 几何帧字节流
//                                              ↘ 调试口（PS 回读）
//
// 用途：P3 上板 bring-up 的第一步。相机通路（USB/SDI/MIPI）通常最后才通，
// 但"PL 综合出来的东西到底跑没跑对"必须能提前回答。这个顶只要一个时钟
// 就能自转：
//
//   * `auto_run = 1`：内部按冷却计数自动一帧接一帧地跑，`frame_idx` 自增；
//   * `auto_run = 0`：由外部给 `start`（协同仿真 tb 走这条）。
//
// 自检图案的几何是**手算得出来的**（见 vision_tpg.v 与 cosim/tb_tpg.cpp），
// 所以上板后 PS 只要读调试口就能判断"PL 对不对"，不需要任何相机或标定。
// ===========================================================================

module vision_selftest #(
    parameter integer WIDTH  = 640,
    parameter integer HEIGHT = 360,
    parameter integer COOLDOWN = 200000      // auto_run 时两帧之间的空拍数
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        auto_run,        // 1 = 自己按内部节拍连跑
    input  wire        start,           // auto_run=0 时：一拍启动一帧
    input  wire [15:0] frame_idx,       // auto_run=0 时：场景编号

    output wire        tpg_busy,
    output wire        tpg_done,        // 图案像素发完
    output wire        frame_done,      // 来自 vision_top：几何帧已发完
    output wire        busy,

    output wire [7:0]  dout_data,
    output wire        dout_vld,
    output wire        dout_last,

    input  wire [3:0]  dbg_sel,
    input  wire [17:0] dbg_idx,
    input  wire [3:0]  dbg_sub,
    output wire [31:0] dbg_data,
    output wire [15:0] cur_frame_idx,
    output wire [31:0] cfg_rdata_o     // 配置口读回（本顶没用 AXI，导出看状态）
);

    // ---- 自动节拍 -------------------------------------------------------
    reg [15:0] auto_idx;
    reg [31:0] cooldown;
    reg        auto_start;

    always @(posedge clk) begin
        if (!rst_n) begin
            auto_idx   <= 16'd0;
            cooldown   <= COOLDOWN[31:0];
            auto_start <= 1'b0;
        end else begin
            auto_start <= 1'b0;
            if (auto_run) begin
                if (cooldown != 32'd0) cooldown <= cooldown - 32'd1;
                else if (!busy && !tpg_busy) begin
                    auto_start <= 1'b1;                 // 一拍
                    auto_idx   <= auto_idx + 16'd1;
                    cooldown   <= COOLDOWN[31:0];
                end
            end else begin
                cooldown <= COOLDOWN[31:0];
            end
        end
    end

    // 必须是**一拍脉冲**：auto_run 时若直接把 auto_run 当 start，vision_top
    // 一回到 IDLE 就又启动，等于永远在重开帧。
    wire        run       = auto_run ? auto_start : start;
    wire [15:0] idx_now   = auto_run ? auto_idx : frame_idx;
    assign cur_frame_idx  = idx_now;

    // ---- 图案 → 流水 ----------------------------------------------------
    wire        t_vld, t_sof, t_eol;
    wire [7:0]  t_r, t_g, t_b;

    vision_tpg #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_tpg (
        .clk(clk), .rst_n(rst_n),
        .start(run), .frame_idx(idx_now[2:0]),
        .busy(tpg_busy), .frame_done(tpg_done),
        .din_vld(t_vld), .din_sof(t_sof), .din_eol(t_eol),
        .din_r(t_r), .din_g(t_g), .din_b(t_b)
    );

    // vision_top 的配置口不写（跑复位默认值 = C 参考模型的默认参数）；
    // 读回导出到 cfg_rdata_o。要调阈值就把这个顶换成 vision_axi，那边有完整寄存器。
    vision_top #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_vision (
        .pix_clk(clk), .rst_n(rst_n),
        .din_vld(t_vld), .din_sof(t_sof), .din_eol(t_eol),
        .din_r(t_r), .din_g(t_g), .din_b(t_b),
        .frame_start(run),
        .cfg_seq(32'd0), .cfg_stamp_ns(64'd0),
        .frame_done(frame_done), .busy(busy),
        .cfg_we(1'b0), .cfg_addr(6'd0), .cfg_wdata(32'd0),
        .cfg_rdata(cfg_rdata_o),
        .dout_vld(dout_vld), .dout_data(dout_data),
        .dout_last(dout_last), .dout_ready(1'b1),
        .dbg_sel(dbg_sel), .dbg_idx(dbg_idx), .dbg_sub(dbg_sub),
        .dbg_data(dbg_data)
    );
endmodule
