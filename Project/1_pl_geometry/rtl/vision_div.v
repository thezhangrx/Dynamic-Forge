// ===========================================================================
// vision_div —— 迭代（多周期）无符号除法器
//
// 为什么不能直接用 `/`：
//   Verilog 的 `/` 综合出来是**一拍算完的组合阵列除法器**。实测本设计的关键
//   路径就是它 —— `sg_rn → 除法 → atan2（又一次除法）→ obs_words`，
//   438 级逻辑、其中 **404 个 CARRY4 串联**，WNS = −85.9 ns（≈11 MHz），
//   而目标周期是 5 ns。见 core/fpga/docs/帧存改造.md §7。
//
// 改法：恢复余数法，**每拍处理 1 位**。WN=34 → 34 拍出一个结果。
//   帧内有 23 万拍余量；每帧最多几百个墙段，多花几万拍完全可以接受。
//
// 为什么结果仍然逐位一致：
//   两者都是**截断整除**（C 的 `/`、Verilog 的 `/`、恢复余数法，对无符号数
//   都给出 floor(num/den)）。所以 620 帧逐位对账（core/fpga/cosim/）仍然守得住。
//
// 端口约定：
//   start  ：一拍脉冲，装载 num/den 并开始
//   busy   ：start 后拉高，算完那拍拉低
//   done   ：算完那拍拉高一个脉冲（quo 同时有效）
//   den==0 ：quo = 0，一拍就 done —— 与 C 参考模型 vr_scale_round 的
//            `if (den <= 0) return 0;` 一致。调用点其实都已先判过分母非零，
//            这里是兜底。
// ===========================================================================
module vision_div #(
    parameter integer WN = 34,   // 被除数位宽
    parameter integer WD = 18    // 除数位宽
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [WN-1:0] num,
    input  wire [WD-1:0] den,
    output reg  [WN-1:0] quo,
    output reg           busy,
    output reg           done
);
    localparam integer KB = $clog2(WN + 1);

    reg [WN-1:0] q;      // 商的移位寄存器（初值 = num，逐位把高位吃掉）
    reg [WD-1:0] rem;    // 余数；减法后 < den ≤ 2^WD−1，装得下 WD 位
    reg [WD-1:0] d;
    reg [KB-1:0] k;

    // 当前这一步：把 q 的最高位并进余数（临时扩到 WD+1 位做比较与相减）
    wire [WD:0]   trial    = {rem, q[WN-1]};
    wire          ge       = (trial >= {1'b0, d});
    wire [WD-1:0] sub      = trial[WD-1:0] - d;   // 只在 ge 时采用；差值 < 2^WD
    // 相减后必然 < d < 2^WD，最高位恒 0，所以只留低 WD 位
    wire [WD-1:0] next_rem = ge ? sub : trial[WD-1:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            quo  <= {WN{1'b0}};
            q    <= {WN{1'b0}};
            rem  <= {WD{1'b0}};
            d    <= {WD{1'b0}};
            k    <= {KB{1'b0}};
        end else begin
            done <= 1'b0;
            if (start) begin
                if (den == {WD{1'b0}}) begin
                    // 除零：一拍给 0，与 C 一致
                    quo  <= {WN{1'b0}};
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    q    <= num;
                    rem  <= {WD{1'b0}};
                    d    <= den;
                    k    <= WN[KB-1:0];
                    busy <= 1'b1;
                end
            end else if (busy) begin
                if (k == {KB{1'b0}}) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    quo  <= q;
                end else begin
                    rem <= next_rem;
                    q   <= {q[WN-2:0], ge};
                    k   <= k - 1'b1;
                end
            end
        end
    end
endmodule
