// ===========================================================================
// tpg_source.v —— 块设计里的**像素源**：让 `vision_tpg` 自己连帧跑
//
// 为什么先做这个，而不是"PS 把帧推进来"
// --------------------------------------
// 块设计第一版的目标是**把 PS 和视觉模块连起来、并且在真硅上跑通整条链**。
// 这条链要闭环，需要一个像素源。三个选项：
//
//   ① 相机（MIPI/SDI）—— 硬件还没通，而且要多写一个前端；
//   ② PS 把帧写进 PL 的 BRAM，PL 侧 FSM 读出来 —— 需要 BRAM 控制器 + 读 FSM；
//   ③ **片上图案发生器**（这个文件）—— 复用协同仿真里已经在用的 `vision_tpg`。
//
// 选 ③ 的理由：`vision_tpg` 的几何是**手算得出来的**（见 vision_tpg.v 与
// cosim/tb_tpg.cpp，403 项比对），所以 PS 一旦能读回几何帧，
// **立刻就能拿手算值判断"真硅上这条链对不对"** —— 不需要相机、不需要标定、
// 不需要等 PS 的送帧通路做好。
//
// 换句话说：这一版验证的是**"PS ↔ PL 的 AXI 通路 + 视觉流水线"**，
// 把"像素从哪来"这个变量先固定住。等这条通了，再把 tpg_source 换成
// "PS 送帧"或"相机"，每次只动一个变量。
//
// 行为：复位后等 COOLDOWN 拍，然后一帧接一帧地发（frame_idx 0..7 循环），
// 与 `vision_selftest` 的 auto_run 一致 —— 但**不含 vision_top**，
// 因为块设计里的流水线就是 `vision_axi` 本身。
// ===========================================================================
`timescale 1ns / 1ps

module tpg_source #(
    parameter integer WIDTH    = 640,
    parameter integer HEIGHT   = 360,
    parameter integer COOLDOWN = 200000
) (
    input  wire       clk,
    input  wire       rst_n,

    output wire       din_vld,
    output wire       din_sof,
    output wire       din_eol,
    output wire [7:0] din_r,
    output wire [7:0] din_g,
    output wire [7:0] din_b,

    output wire [2:0] frame_idx,     // 当前图案号（= tpg 用的那个）
    output wire       tpg_busy,
    output wire       tpg_done       // 一帧像素发完（不等于几何帧发完）
);

    reg [15:0] idx  = 16'd0;
    reg [31:0] cool = COOLDOWN[31:0];
    reg        start = 1'b0;

    // 与 vision_selftest 的 auto 节拍同构：冷却计数到 0 且 tpg 不忙 → 一拍 start
    always @(posedge clk) begin
        if (!rst_n) begin
            idx   <= 16'd0;
            cool  <= COOLDOWN[31:0];
            start <= 1'b0;
        end else begin
            start <= 1'b0;
            if (cool != 32'd0) begin
                cool <= cool - 32'd1;
            end else if (!tpg_busy) begin
                start <= 1'b1;
                idx   <= idx + 16'd1;
                cool  <= COOLDOWN[31:0];
            end
        end
    end

    vision_tpg #(.WIDTH(WIDTH), .HEIGHT(HEIGHT)) u_tpg (
        .clk(clk), .rst_n(rst_n),
        .start(start), .frame_idx(idx[2:0]),
        .busy(tpg_busy), .frame_done(tpg_done),
        .din_vld(din_vld), .din_sof(din_sof), .din_eol(din_eol),
        .din_r(din_r), .din_g(din_g), .din_b(din_b)
    );

    assign frame_idx = idx[2:0];

endmodule
