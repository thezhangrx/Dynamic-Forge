// ===========================================================================
// vision_ops.vh —— 纯组合算子（必须与 core/fpga/ref/vision_ref.c 逐位一致）
//
// 这里只有函数，没有状态。每一个函数都是 C 参考模型里同名函数的**同一串
// 整数运算**：
//
//   vo_gray()        ←→  vr_rgb_to_gray()
//   vo_classify()    ←→  vr_classify()
//   vo_scale_round() ←→  vr_scale_round()
//   vo_atan2_q6()    ←→  vr_atan2_q6()
//   vo_crc_step()    ←→  vr_crc32() 的一步
//
// 为什么角度要在硬件里"重新发明" atan2：RTL 里没有 libm，而对账要求逐位一致。
// 所以两边都用**同一张 9 点表 + 同一次整数除法**；真实精度参照仍在 Python 侧。
// 这张表在 C 侧和这里必须一模一样 —— 有一条测试逐项核对
// （core/fpga/tests/test_rtl_contract.py::test_ops_table_matches_c）。
//
// ⚠ 只对**非负** num 调用 vo_scale_round（C 里也只这么用：坐标、面积、矩）。
// ===========================================================================

`ifndef VISION_OPS_VH

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
`define VISION_OPS_VH

// ---- 灰度化：BT.601 整数权重 77/150/29，+128 四舍五入 --------------------
// C: (uint8_t)((77*r + 150*g + 29*b + 128) >> 8)
function [7:0] vo_gray(input [7:0] r, input [7:0] g, input [7:0] b);
    reg [17:0] s;
    begin
        s = 18'd77 * r + 18'd150 * g + 18'd29 * b + 18'd128;
        vo_gray = s[15:8];
    end
endfunction

// ---- 三路分类，位掩码 [0]=wall [1]=bright [2]=green ---------------------
// **逐条等于** core/vision/primitives.py: classify()（默认参数）：
//   wall  : r-b > wall_rb      且 r > wall_min_r 且 g > wall_min_g
//   bright: min > bright_min   且 max-min < bright_spread
//   green : g-max(r,b) > green_gm 且 g > green_min
// C 的 (r - b) > 阈值 是带符号比较；阈值 > 0，所以等价于 r > b + 阈值，
// 这里用这个形式，避免有符号运算。
function [2:0] vo_classify(
    input [7:0] r, input [7:0] g, input [7:0] b,
    input [7:0] wall_rb, input [7:0] wall_min_r, input [7:0] wall_min_g,
    input [7:0] bright_min, input [7:0] bright_spread,
    input [7:0] green_gm, input [7:0] green_min
);
    reg [2:0] m;
    reg [7:0] rb_max, mx, mn;
    begin
        m = 3'b000;
        if (({1'b0, r} > {1'b0, b} + {1'b0, wall_rb}) &&
            (r > wall_min_r) && (g > wall_min_g))
            m[0] = 1'b1;
        mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
        mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
        // 高饱和色不算"白"：少了 hi-lo 上限，橙墙的 min 到 196 就会被同时
        // 判成亮色，"角色连通域"就凭空冒出来。
        if ((mn > bright_min) && ((mx - mn) < bright_spread))
            m[1] = 1'b1;
        rb_max = (r > b) ? r : b;
        if (({1'b0, g} > {1'b0, rb_max} + {1'b0, green_gm}) && (g > green_min))
            m[2] = 1'b1;
        vo_classify = m;
    end
endfunction

// ---- round(num*scale/den)，num >= 0，den > 0 ----------------------------
// 用 2n+den 的形式，保证"恰好半个单位"向上 —— 与 C 的 `v*64 + 0.5` 截断一致。
function [31:0] vo_scale_round(input [47:0] num, input [31:0] den, input [15:0] scale);
    reg [63:0] n, d;
    begin
        d = {32'b0, den};
        n = {16'b0, num} * {48'b0, scale};
        if (d == 64'd0) vo_scale_round = 32'd0;
        else            vo_scale_round = (n * 64'd2 + d) / (d * 64'd2);
    end
endfunction

// AT[i] = round(atan(i/8) * 256)，i = 0..8（与 C 侧的表逐项相同）
function [15:0] vo_at(input [3:0] i);
    begin
        case (i)
            4'd0: vo_at = 16'd0;
            4'd1: vo_at = 16'd32;
            4'd2: vo_at = 16'd63;
            4'd3: vo_at = 16'd92;
            4'd4: vo_at = 16'd119;
            4'd5: vo_at = 16'd143;
            4'd6: vo_at = 16'd165;
            4'd7: vo_at = 16'd184;
            default: vo_at = 16'd201;
        endcase
    end
endfunction

// ---- 定点 atan2：返回 round(atan2(y_q8, x_q8) * 64)，x_q8 > 0 -----------
function signed [31:0] vo_atan2_q6(input signed [31:0] y_q8, input signed [31:0] x_q8);
    reg signed [31:0] a, b, t, v, frac;
    reg [47:0] num;
    reg [31:0] r;
    reg [3:0]  idx;
    reg        neg, inv;
    begin
        if (x_q8 <= 0 || y_q8 == 0) begin
            vo_atan2_q6 = 32'sd0;
        end else begin
            neg = (y_q8 < 0);
            a   = neg ? -y_q8 : y_q8;
            b   = x_q8;
            inv = (a > b);
            if (inv) begin t = a; a = b; b = t; end
            // r = a/b 的 Q15；a <= b < 2^18（x_q8 = dx<<8 ≤ 640*256）
            num = {30'b0, a[17:0]} * 48'd32768;
            r   = num / {14'b0, b[17:0]};
            if (r >= 32'd32768) begin
                v = 32'sd201;
            end else begin
                idx  = r[15:12];
                frac = $signed({20'b0, r[11:0]});
                v = vo_at(idx) + (($signed({16'b0, vo_at(idx + 4'd1)}) -
                                   $signed({16'b0, vo_at(idx)})) * frac) / 32'sd4096;
            end
            if (inv) v = 32'sd402 - v;      // 402 = round(pi/2 * 256)
            if (neg) v = -v;
            // Q8 → Q6，半数远离零
            vo_atan2_q6 = (v >= 0) ? ((v + 32'sd2) / 32'sd4) : -(((-v) + 32'sd2) / 32'sd4);
        end
    end
endfunction

// ---- CRC32 一步（多项式 0xEDB88320 反转）-------------------------------
// C: crc ^= byte; 8 次 { crc = (crc>>1) ^ (0xEDB88320 & -(crc&1)) }
function [31:0] vo_crc_step(input [31:0] crc, input [7:0] data);
    reg [31:0] x;
    integer k;
    begin
        x = crc ^ {24'b0, data};
        for (k = 0; k < 8; k = k + 1)
            x = (x >> 1) ^ (32'hEDB88320 & {32{x[0]}});
        vo_crc_step = x;
    end
endfunction

`endif // VISION_OPS_VH

/* verilator lint_on WIDTHEXPAND */
/* verilator lint_on WIDTHTRUNC */
/* verilator lint_on BLKSEQ */
/* verilator lint_on UNUSEDSIGNAL */
