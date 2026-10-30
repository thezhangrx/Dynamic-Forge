// ===========================================================================
// vision_div_pipe.v —— 流水线无符号除法器（II=1，每拍可进一个新操作数）
//
// 为什么再写一个（不是改 vision_div）：`vision_div` 是 1 位/拍的**迭代**除法器，
// 给 vision_top 的墙段几何用（每帧几百个，多花几万拍无所谓）。正射是**每帧 23 万像素**，
// 每像素两次 64 位除法 → 实测 137 拍/像素、380 ms/帧（R62 的数）。要进 35 ms，
// 除法必须变成"每拍进一个数"的流水线。两者用途不同，所以并存、互不影响。
//
// 算法仍是**恢复余数法**（radix-4，每级出 2 位商），所以结果与 C 的截断整除逐位一致 ——
// 只是把 34 次迭代展开成 17 级流水。
//
// 位宽（不是随手拍的，是几何上界）：
//   调用方算的是 Q = (|nu|<<24)/|nd| = |u|*2^24。实测 |u| < 1024（页面全部落在相机图内，
//   若超了就该钳到边界）→ Q < 1024*2^24 = 2^34，所以 WQ=34 位够。
//   若 Q ≥ 2^34（异常单应/退化分母），`ovf` 拉高，调用方按"饱和到最大"处理 ——
//   与"先用 64 位精确算再钳"**等价**：Q≥2^34 ⟹ |u|≥1024 ⟹ 一定被钳到 639/479。
//   ⚠ 改 WQ 必须同步改调用方的饱和逻辑，否则靠近边界的像素会静默出错。
//
// 时序：in_vld 那一拍给 (num, den)，NST+1 拍后 out_vld 那拍出 (quo, ovf)。
//       ce=0 时整条流水线冻结（像素源被行缓存 miss 卡住时用）。
// ===========================================================================
module vision_div_pipe #(
    parameter integer WN = 64,    // 被除数/除数位宽（无符号）
    parameter integer WQ = 34     // 商位宽（必须偶数：每级 2 位）
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          ce,      // 时钟使能：0 = 冻结
    input  wire          in_vld,
    input  wire [WN-1:0] num,
    input  wire [WN-1:0] den,
    output wire [WQ-1:0] quo,
    output wire          ovf,
    output wire          out_vld
);
    localparam integer NST = WQ / 2;          // 级数（每级 2 位商）
    //: 余数本身 < den ≤ 2^WN−1，所以**存 WN 位**；
    //: 而 trial = 4*rem + 2 位被除数 需要 WN+2 位（这就是 RW）。
    //: ⚠ 一开始把 rem 也存成 RW 位，于是 {rem, nb} 是 68 位被截成 66 位 → 商全错
    //:   （tb_divpipe 第一跑：20 万对里有 16.9 万对错）。
    localparam integer RW  = WN + 2;
    localparam integer NH  = WN - WQ;         // 高 WN-WQ 位先并进余数

    // ---- 输入级：把 num 的高 (WN-WQ) 位先装进余数，并判"商是否超宽" ----
    //: rem 初值 = num >> WQ；若它已经 ≥ den，则商 ≥ 2^WQ → ovf
    wire [NH-1:0]   num_hi   = num[WN-1:WQ];
    wire [WN-1:0]   rem_init = {{(WN - NH){1'b0}}, num_hi};
    wire            ovf_init = (rem_init >= den);

    reg [WN-1:0] rem_p [0:NST];
    reg [WN-1:0] den_p [0:NST];
    //: ⚠ 被除数的低 WQ 位必须**跟着流水线走**（每级各存一份）。
    //:   第一版让 17 级都直接读输入口 `num` —— 于是每级拿到的是"当前这一拍"的
    //:   被除数，而不是自己那一拍进队的像素。单测（喂一个数、等 25 拍）恰好看到
    //:   正确值，连续喂就全部输出成**最后一个像素**的结果（tb_divpipe 20 万对里 16.9 万错）。
    reg [WQ-1:0] num_p [0:NST];
    reg [WQ-1:0] q_p   [0:NST];
    reg          ovf_p [0:NST];
    reg          vld_p [0:NST];

    always @(posedge clk) begin
        if (!rst_n) begin
            rem_p[0] <= {WN{1'b0}};
            den_p[0] <= {WN{1'b0}};
            num_p[0] <= {WQ{1'b0}};
            q_p[0]   <= {WQ{1'b0}};
            ovf_p[0] <= 1'b0;
            vld_p[0] <= 1'b0;
        end else if (ce) begin
            rem_p[0] <= rem_init;
            den_p[0] <= den;
            num_p[0] <= num[WQ-1:0];
            q_p[0]   <= {WQ{1'b0}};
            ovf_p[0] <= ovf_init;
            vld_p[0] <= in_vld;
        end
    end

    // ---- 17 级：trial = 4*rem + 2 位被除数；q2 = trial / den（取值 0..3）----
    genvar gi;
    generate
        for (gi = 0; gi < NST; gi = gi + 1) begin : g_st
            wire [RW-1:0] d1 = {{(RW - WN){1'b0}}, den_p[gi]};
            wire [RW-1:0] d2 = d1 + d1;
            wire [RW-1:0] d3 = d2 + d1;
            //: 被除数按**高位在前**每级吃 2 位（读本级的寄存器副本，不是输入口）
            wire [1:0]    nb = num_p[gi][WQ-1-2*gi -: 2];
            wire [RW-1:0] tr = {rem_p[gi], nb};
            wire [1:0]    q2 = (tr >= d3) ? 2'd3 : (tr >= d2) ? 2'd2 : (tr >= d1) ? 2'd1 : 2'd0;
            wire [RW-1:0] rem_nx = tr - ((q2 == 2'd3) ? d3
                                       : (q2 == 2'd2) ? d2
                                       : (q2 == 2'd1) ? d1 : {RW{1'b0}});
            always @(posedge clk) begin
                if (!rst_n) begin
                    rem_p[gi+1] <= {WN{1'b0}};
                    den_p[gi+1] <= {WN{1'b0}};
                    num_p[gi+1] <= {WQ{1'b0}};
                    q_p[gi+1]   <= {WQ{1'b0}};
                    ovf_p[gi+1] <= 1'b0;
                    vld_p[gi+1] <= 1'b0;
                end else if (ce) begin
                    rem_p[gi+1] <= rem_nx[WN-1:0];   //: 减完必然 < den，装得进 WN 位
                    den_p[gi+1] <= den_p[gi];
                    num_p[gi+1] <= num_p[gi];
                    //: 商是**高位先出**：每级左移 2 位把本级 2 位放到低位，
                    //: 走完 17 级就正好是 34 位商（第一级那 2 位落在最高位）。
                    q_p[gi+1]   <= {q_p[gi][WQ-3:0], q2};
                    //: 高位那 2 位理论上恒 0（rem_nx < den < 2^WN）；真被置起来说明
                    //: 这条不变量被破坏了 → 也按溢出处理（宁可变饱和值，不输出错像素）
                    ovf_p[gi+1] <= ovf_p[gi] | (|rem_nx[RW-1:WN]);
                    vld_p[gi+1] <= vld_p[gi];
                end
            end
        end
    endgenerate

    //: 末级余数还 ≥ den ⇒ 真实商还需要更多位 → 也按溢出处理
    wire rem_ge_den = (rem_p[NST] >= den_p[NST]);

    assign quo     = q_p[NST];
    assign ovf     = ovf_p[NST] || rem_ge_den;
    assign out_vld = vld_p[NST];
endmodule
