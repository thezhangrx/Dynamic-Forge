// ===========================================================================
// vision_tpg.v —— PL 内的**测试图案发生器**（bring-up 用，无相机也能验通路）
//
// 生成一个几何完全已知的合成场地：
//
//     y ↑
//       │        ┌──────────────┐  ← 墙：暖色，R-B 大、R/G 够亮
//       │        │   ...        │
//       │        └───┐      ┌───┘  ← 缺口（gap_x 左右各 30 px）
//       │            └──────┘
//       │
//       │            ●              ← 角色：纯白，min(R,G,B) > 195
//       └──────────────────────────→ x
//
// 为什么值得有：P3 上板 bring-up 时**相机通路往往还没通**（USB/SDI/MIPI 都
// 要驱动），但"PL 到底跑没跑对"必须能单独回答。自检图案把这个问题变成
// **有确定答案**的题：墙带在哪几行、缺口中心在哪个 x、角色质心在哪 ——
// 全部可以手算。见 cosim/tb_tpg.cpp 里的解析期望值。
//
// 场景随 frame_idx 变化（都是 2 的幂取模，避免除法器）：
//     wall_y  = 60  + (idx[1:0]) * 40     → 60/100/140/180
//     gap_x   = 200 + (idx[1:0]) * 50     → 200/250/300/350
//     p_px    = 300 + (idx[2:0]) * 20     → 300..440
//     p_py    = 280 + (idx[1:0]) * 20     → 280/300/320/340
//
// ⚠ 图案取值是**刻意选过**的：
//   * 背景 (16,16,24)：wall 判据 R-B>40 不成立、bright 判据 min>195 不成立；
//   * 墙 (200,140,60)：wall 成立（R-B=140, R=200, G=140），bright 不成立（min=60）；
//   * 角色 (255,255,255)：bright 成立，wall 不成立（R-B=0）。
//   三张掩膜互不污染，解析期望值才好算。
//
// 像素输出是**组合**的（直接从当前 x/y 算），不是寄存的 —— 寄存会与 x/y
// 差一拍，din_sof 就会标在 (1,0) 上。
// ===========================================================================

module vision_tpg #(
    parameter integer WIDTH    = 640,
    parameter integer HEIGHT   = 360,
    parameter integer BAR_X0   = 80,
    parameter integer BAR_X1   = 559,
    parameter integer BAR_H    = 24,
    parameter integer GAP_W    = 60,
    parameter integer PLAYER_R = 10
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,          // 一拍：开始输出一帧
    // 场景以 8 帧为周期重复（wall_y/gap_x/p_px/p_py 的周期分别是 4/4/8/4），
    // 所以只要低 3 位 —— 端口就开 3 位，省得留一堆用不上的位。
    input  wire [2:0]  frame_idx,
    output reg         busy,
    output reg         frame_done,     // 发完一帧像素（在最后一拍之后的一拍）

    output wire        din_vld,
    output wire        din_sof,
    output wire        din_eol,
    output wire [7:0]  din_r,
    output wire [7:0]  din_g,
    output wire [7:0]  din_b
);

    localparam [7:0] BG_R  = 8'd16,  BG_G  = 8'd16,  BG_B  = 8'd24;
    localparam [7:0] W_R   = 8'd200, W_G   = 8'd140, W_B   = 8'd60;
    localparam [7:0] P_R   = 8'd255, P_G   = 8'd255, P_B   = 8'd255;
    localparam integer HALF_GAP = GAP_W / 2;

    // ---- 场景参数（组合；tb 的期望值按同一组公式算）--------------------
    wire [8:0]  wall_y = 9'd60  + {7'b0, frame_idx[1:0]} * 9'd40;    // 60/100/140/180
    wire [9:0]  gap_x  = 10'd200 + {8'b0, frame_idx[1:0]} * 10'd50;  // 200/250/300/350
    wire [9:0]  p_px   = 10'd300 + {7'b0, frame_idx[2:0]} * 10'd20;  // 300..440
    wire [8:0]  p_py   = 9'd280 + {7'b0, frame_idx[1:0]} * 9'd20;    // 280/300/320/340

    reg [9:0] x;      // 列
    reg [8:0] y;      // 行

    // ---- 像素判定 -------------------------------------------------------
    wire signed [11:0] dx = $signed({2'b0, x}) - $signed({2'b0, p_px});
    wire signed [11:0] dy = $signed({3'b0, y}) - $signed({3'b0, p_py});
    wire signed [23:0] d2 = dx * dx + dy * dy;

    localparam [7:0]  PL_R = PLAYER_R[7:0];
    localparam [23:0] R2   = PL_R * PL_R;

    wire in_bar  = (x >= BAR_X0[9:0]) && (x <= BAR_X1[9:0]);
    wire in_gap  = (x >= gap_x - HALF_GAP[9:0]) && (x < gap_x + HALF_GAP[9:0]);
    wire on_wall = (y >= wall_y) && (y < wall_y + BAR_H[8:0]) && in_bar && !in_gap;
    wire on_play = (d2 <= R2);

    assign din_vld = busy;
    assign din_sof = busy && (x == 10'd0) && (y == 9'd0);
    assign din_eol = busy && (x == WIDTH[9:0] - 10'd1);

    assign din_r = on_play ? P_R : (on_wall ? W_R : BG_R);
    assign din_g = on_play ? P_G : (on_wall ? W_G : BG_G);
    assign din_b = on_play ? P_B : (on_wall ? W_B : BG_B);

    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0; frame_done <= 1'b0; x <= 0; y <= 0;
        end else begin
            frame_done <= 1'b0;
            if (start) begin
                busy <= 1'b1; x <= 10'd0; y <= 9'd0;
            end else if (busy) begin
                if (x == WIDTH[9:0] - 10'd1) begin
                    x <= 10'd0;
                    if (y == HEIGHT[8:0] - 9'd1) begin
                        busy       <= 1'b0;
                        frame_done <= 1'b1;
                    end else begin
                        y <= y + 9'd1;
                    end
                end else begin
                    x <= x + 10'd1;
                end
            end
        end
    end

endmodule
