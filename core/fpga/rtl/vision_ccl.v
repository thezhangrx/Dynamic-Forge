// ===========================================================================
// vision_ccl.v —— 亮掩膜上的**扫描线连通域**（8 邻接）+ 一阶矩 → 角色
//
// 比对对手：core/fpga/ref/vision_ref.c 的 vr_extract_player()。
// 算法逐条对应：
//   * 逐行找游程，只留**上一行的游程**在手里（WIDTH/2 = 320 是理论上限）；
//   * 与上一行重叠判 8 邻接：`cur.x0 <= prev.x1+1 && cur.x1 >= prev.x0-1`。
//     注意 C 里 prev.x0-1 在 x0=0 时是 -1（有符号），所以这里写成
//     `cur.x1+1 >= prev.x0` 的等价形式，避免无符号下溢；
//   * 并查集，union 时 **parent[b] = a**（a 是当前游程找到的根），并**把 b 的
//     历史矩/包围盒搬进 a** —— 不搬的话"先分后合"的连通域面积与质心会偏小
//     （C 侧修过同一个 bug，见 vision_ref.c 的 uf_union 注释）；
//   * 过滤 `100*n >= 45*w*h` 且 `100*max(w,h) <= 160*min(w,h)`，
//     用 `n²/(w·h)` 的**交叉相乘**比较选最优（不落浮点，同分顺序与 C 一致）。
//
// 并查集是 1 拍 1 跳的**顺序查找**而不是组合展开：链长最坏到标签表深度，
// 组合展开就是 256 级级联比较器；顺序走只多花几拍，面积小一个量级。
// ===========================================================================

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

module vision_ccl #(
    parameter integer WIDTH      = 640,
    parameter integer HEIGHT     = 360,
    parameter integer MAX_LABELS = 256,
    parameter integer MAX_RUNS   = 384
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         done,

    output reg  [17:0] rd_addr,          // 掩膜回读地址（指向 bright_mem）
    input  wire        bright_rd,

    input  wire [31:0] min_area,
    input  wire [31:0] max_area,
    input  wire [31:0] min_fill_pct,
    input  wire [31:0] max_aspect_pct,

    output reg         player_valid,
    output reg  [31:0] player_x, player_y, player_r,
    output reg  [9:0]  blob_x0, blob_x1, blob_y0, blob_y1,
    output reg  [31:0] blob_area,
    output reg  [8:0]  n_labels
);

    localparam integer W_BITS = $clog2(WIDTH);
    localparam integer H_BITS = $clog2(HEIGHT);
    localparam integer RB     = $clog2(MAX_RUNS + 1);      // 游程计数
    localparam integer LB     = $clog2(MAX_LABELS);        // 标签索引
    localparam integer NB     = LB + 1;                    // 标签计数（能到 256）

    localparam [4:0] C_IDLE   = 5'd0,
                     C_INIT   = 5'd1,
                     C_ROW    = 5'd2,
                     C_MATCH  = 5'd3,
                     C_FIND   = 5'd4,
                     C_SETI   = 5'd5,
                     C_UFINDB = 5'd6,
                     C_UDO    = 5'd7,
                     C_CREATE = 5'd8,
                     C_ACC    = 5'd9,
                     C_ACCDO  = 5'd10,
                     C_COPY   = 5'd11,
                     C_SEL    = 5'd12,
                     C_DONE   = 5'd13;

    reg [4:0]    state, find_ret;
    reg [8:0]    init_k;
    reg [LB-1:0] find_target, find_res;
    reg [LB-1:0] ra_tmp;

    reg          in_run;
    reg [W_BITS-1:0] r_a;
    reg [H_BITS-1:0] y;
    reg [W_BITS:0]   x;

    reg [RB-1:0] n_cur, n_prev, i_cur, j_prev;
    reg [NB-1:0] n_label;
    reg [LB-1:0] sel_i;

    reg [W_BITS-1:0] cur_x0 [0:MAX_RUNS-1];
    reg [W_BITS-1:0] cur_x1 [0:MAX_RUNS-1];
    reg [W_BITS-1:0] prv_x0 [0:MAX_RUNS-1];
    reg [W_BITS-1:0] prv_x1 [0:MAX_RUNS-1];
    reg              cur_lv [0:MAX_RUNS-1];
    reg [LB-1:0]     cur_lb [0:MAX_RUNS-1];
    reg [LB-1:0]     prv_lb [0:MAX_RUNS-1];

    reg [LB-1:0]     parent [0:MAX_LABELS-1];
    reg [31:0]       cmp_n  [0:MAX_LABELS-1];
    reg [31:0]       cmp_sx [0:MAX_LABELS-1];
    reg [31:0]       cmp_sy [0:MAX_LABELS-1];
    reg [W_BITS-1:0] cmp_x0 [0:MAX_LABELS-1];
    reg [W_BITS-1:0] cmp_x1 [0:MAX_LABELS-1];
    reg [H_BITS-1:0] cmp_y0 [0:MAX_LABELS-1];
    reg [H_BITS-1:0] cmp_y1 [0:MAX_LABELS-1];

    reg [LB-1:0] best_i;
    reg          best_vld;
    reg [63:0]   best_num, best_den;

    // 地址：y*WIDTH 用 32 位算再截到 18 位（HEIGHT*WIDTH < 2^18 恒成立）
    function [17:0] row_base(input [H_BITS-1:0] yy);
        reg [31:0] t;
        begin
            t = yy * WIDTH;
            row_base = t[17:0];
        end
    endfunction

    // ---- 组合：重叠判定与矩增量 ------------------------------------------
    wire [W_BITS:0] a_prev = prv_x1[j_prev];
    wire [W_BITS:0] b_prev = prv_x0[j_prev];
    wire [W_BITS:0] a_cur  = cur_x0[i_cur];
    wire [W_BITS:0] b_cur  = cur_x1[i_cur];
    wire overlap = (a_cur <= a_prev + 1'b1) && (b_cur + 1'b1 >= b_prev);

    wire [31:0] acc_len   = b_cur - a_cur + 1'b1;
    wire [31:0] acc_sxadd = (a_cur + b_cur) * acc_len / 2;
    wire [31:0] acc_syadd = y * acc_len;

    // ---- 组合：选择判据（全整数，不落浮点）-------------------------------
    wire [31:0] w_c   = cmp_x1[sel_i] - cmp_x0[sel_i] + 1'b1;
    wire [31:0] h_c   = cmp_y1[sel_i] - cmp_y0[sel_i] + 1'b1;
    wire [31:0] wh_c  = w_c * h_c;
    wire [31:0] asp_max = (w_c > h_c) ? w_c : h_c;
    wire [31:0] asp_min = (w_c > h_c) ? h_c : w_c;
    wire        area_ok = (cmp_n[sel_i] >= min_area) && (cmp_n[sel_i] <= max_area);
    wire        edge_ok = (cmp_x0[sel_i] > 0) && (cmp_y0[sel_i] > 0) &&
                          (cmp_x1[sel_i] < WIDTH - 1) && (cmp_y1[sel_i] < HEIGHT - 1);
    wire        fill_ok = (100 * cmp_n[sel_i] >= min_fill_pct * wh_c);
    wire        asp_ok  = (100 * asp_max <= max_aspect_pct * asp_min);
    wire [63:0] score_num = {32'b0, cmp_n[sel_i]} * {32'b0, cmp_n[sel_i]};

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= C_IDLE;
            done         <= 1'b0;
            player_valid <= 1'b0;
            n_labels     <= 9'd0;
            in_run       <= 1'b0;
            rd_addr      <= 18'd0;
        end else begin
            done <= 1'b0;
            case (state)
            // -------------------------------------------------------------
            C_IDLE: begin
                if (start) begin
                    init_k <= 9'd0;
                    state  <= C_INIT;
                end
            end
            // 标签表复位：一拍一项。Verilator 不支持在 for 循环里对数组做
            // 非阻塞赋值（BLKLOOPINIT），顺序复位也更省布线。
            C_INIT: begin
                parent[init_k[LB-1:0]] <= init_k[LB-1:0];
                cmp_n [init_k[LB-1:0]] <= 32'd0;
                cmp_sx[init_k[LB-1:0]] <= 32'd0;
                cmp_sy[init_k[LB-1:0]] <= 32'd0;
                cmp_x0[init_k[LB-1:0]] <= {W_BITS{1'b1}};
                cmp_x1[init_k[LB-1:0]] <= {W_BITS{1'b0}};
                cmp_y0[init_k[LB-1:0]] <= {H_BITS{1'b1}};
                cmp_y1[init_k[LB-1:0]] <= {H_BITS{1'b0}};
                if (init_k + 9'd1 >= MAX_LABELS) begin
                    n_cur <= {RB{1'b0}}; n_prev <= {RB{1'b0}}; n_label <= {NB{1'b0}};
                    y     <= {H_BITS{1'b0}};
                    x     <= {W_BITS+1{1'b0}};
                    in_run<= 1'b0;
                    player_valid <= 1'b0;
                    best_vld <= 1'b0;
                    best_num <= 64'd0; best_den <= 64'd1;
                    rd_addr  <= 18'd0;
                    state    <= C_ROW;
                end else begin
                    init_k <= init_k + 9'd1;
                end
            end
            // -------------------------------------------------------------
            // 扫一行，收集游程
            C_ROW: begin
                if (x < WIDTH) begin
                    if (bright_rd) begin
                        if (!in_run) begin in_run <= 1'b1; r_a <= x[W_BITS-1:0]; end
                    end else if (in_run) begin
                        in_run <= 1'b0;
                        cur_x0[n_cur] <= r_a;
                        cur_x1[n_cur] <= x[W_BITS-1:0] - 1'b1;
                        cur_lv[n_cur] <= 1'b0;
                        n_cur <= n_cur + 1'b1;
                    end
                    x <= x + 1'b1;
                    if (x + 1'b1 < WIDTH) rd_addr <= row_base(y) + {7'b0, x} + 18'd1;
                end else begin
                    if (in_run) begin
                        in_run <= 1'b0;
                        cur_x0[n_cur] <= r_a;
                        cur_x1[n_cur] <= WIDTH[W_BITS-1:0] - 1'b1;
                        cur_lv[n_cur] <= 1'b0;
                        n_cur <= n_cur + 1'b1;
                    end
                    i_cur <= {RB{1'b0}};
                    j_prev<= {RB{1'b0}};
                    state <= C_MATCH;
                end
            end
            // -------------------------------------------------------------
            C_MATCH: begin
                if (i_cur >= n_cur) begin
                    i_cur <= {RB{1'b0}};
                    state <= C_ACC;
                end else if (j_prev >= n_prev) begin
                    state <= C_CREATE;
                end else if (overlap) begin
                    if (!cur_lv[i_cur]) begin
                        find_target <= prv_lb[j_prev];
                        find_ret    <= C_SETI;
                        state       <= C_FIND;
                    end else begin
                        find_target <= cur_lb[i_cur];
                        find_ret    <= C_UFINDB;
                        state       <= C_FIND;
                    end
                end else begin
                    j_prev <= j_prev + 1'b1;
                end
            end
            C_FIND: begin
                if (parent[find_target] == find_target) begin
                    find_res <= find_target;
                    state    <= find_ret;
                end else begin
                    find_target <= parent[find_target];
                end
            end
            C_SETI: begin
                cur_lb[i_cur] <= find_res;
                cur_lv[i_cur] <= 1'b1;
                j_prev <= j_prev + 1'b1;
                state  <= C_MATCH;
            end
            C_UFINDB: begin
                ra_tmp      <= find_res;
                find_target <= prv_lb[j_prev];
                find_ret    <= C_UDO;
                state       <= C_FIND;
            end
            C_UDO: begin
                if (ra_tmp != find_res) begin
                    parent[find_res] <= ra_tmp;
                    cmp_n[ra_tmp]  <= cmp_n[ra_tmp]  + cmp_n[find_res];
                    cmp_sx[ra_tmp] <= cmp_sx[ra_tmp] + cmp_sx[find_res];
                    cmp_sy[ra_tmp] <= cmp_sy[ra_tmp] + cmp_sy[find_res];
                    if (cmp_x0[find_res] < cmp_x0[ra_tmp]) cmp_x0[ra_tmp] <= cmp_x0[find_res];
                    if (cmp_x1[find_res] > cmp_x1[ra_tmp]) cmp_x1[ra_tmp] <= cmp_x1[find_res];
                    if (cmp_y0[find_res] < cmp_y0[ra_tmp]) cmp_y0[ra_tmp] <= cmp_y0[find_res];
                    if (cmp_y1[find_res] > cmp_y1[ra_tmp]) cmp_y1[ra_tmp] <= cmp_y1[find_res];
                end
                j_prev <= j_prev + 1'b1;
                state  <= C_MATCH;
            end
            // -------------------------------------------------------------
            C_CREATE: begin
                if (!cur_lv[i_cur] && (n_label < MAX_LABELS)) begin
                    cmp_n [n_label[LB-1:0]] <= 32'd0;
                    cmp_sx[n_label[LB-1:0]] <= 32'd0;
                    cmp_sy[n_label[LB-1:0]] <= 32'd0;
                    cmp_x0[n_label[LB-1:0]] <= cur_x0[i_cur];
                    cmp_x1[n_label[LB-1:0]] <= cur_x1[i_cur];
                    cmp_y0[n_label[LB-1:0]] <= y;
                    cmp_y1[n_label[LB-1:0]] <= y;
                    cur_lb[i_cur]   <= n_label[LB-1:0];
                    cur_lv[i_cur]   <= 1'b1;
                    n_label         <= n_label + 1'b1;
                end
                j_prev <= {RB{1'b0}};
                i_cur  <= i_cur + 1'b1;
                state  <= C_MATCH;
            end
            // -------------------------------------------------------------
            C_ACC: begin
                if (i_cur >= n_cur) begin
                    i_cur <= {RB{1'b0}};
                    state <= C_COPY;
                end else if (!cur_lv[i_cur]) begin
                    i_cur <= i_cur + 1'b1;
                end else begin
                    find_target <= cur_lb[i_cur];
                    find_ret    <= C_ACCDO;
                    state       <= C_FIND;
                end
            end
            C_ACCDO: begin
                cmp_n[find_res]  <= cmp_n[find_res]  + acc_len;
                cmp_sx[find_res] <= cmp_sx[find_res] + acc_sxadd;
                cmp_sy[find_res] <= cmp_sy[find_res] + acc_syadd;
                if (cur_x0[i_cur] < cmp_x0[find_res]) cmp_x0[find_res] <= cur_x0[i_cur];
                if (cur_x1[i_cur] > cmp_x1[find_res]) cmp_x1[find_res] <= cur_x1[i_cur];
                if (y > cmp_y1[find_res]) cmp_y1[find_res] <= y;
                i_cur <= i_cur + 1'b1;
                state <= C_ACC;
            end
            // -------------------------------------------------------------
            C_COPY: begin
                if (i_cur < n_cur) begin
                    prv_x0[i_cur] <= cur_x0[i_cur];
                    prv_x1[i_cur] <= cur_x1[i_cur];
                    prv_lb[i_cur] <= cur_lb[i_cur];
                    i_cur <= i_cur + 1'b1;
                end else begin
                    n_prev <= n_cur;
                    n_cur  <= {RB{1'b0}};
                    i_cur  <= {RB{1'b0}};
                    x      <= {W_BITS+1{1'b0}};
                    in_run <= 1'b0;
                    if (y + 1'b1 >= HEIGHT) begin
                        sel_i <= {LB{1'b0}};
                        state <= C_SEL;
                    end else begin
                        y       <= y + 1'b1;
                        rd_addr <= row_base(y + 1'b1);
                        state   <= C_ROW;
                    end
                end
            end
            // -------------------------------------------------------------
            C_SEL: begin
                if ({1'b0, sel_i} >= n_label) begin
                    state <= C_DONE;
                end else begin
                    sel_i <= sel_i + 1'b1;
                    if (parent[sel_i] == sel_i) begin
                        if (area_ok && edge_ok && fill_ok && asp_ok) begin
                            if (score_num * best_den > best_num * {32'b0, wh_c}) begin
                                best_num <= score_num;
                                best_den <= {32'b0, wh_c};
                                best_i   <= sel_i;
                                best_vld <= 1'b1;
                            end
                        end
                    end
                end
            end
            // -------------------------------------------------------------
            C_DONE: begin
                if (best_vld) begin
                    player_valid <= 1'b1;
                    player_x <= vo_scale_round({16'b0, cmp_sx[best_i]}, cmp_n[best_i], 16'd64);
                    player_y <= vo_scale_round({16'b0, cmp_sy[best_i]}, cmp_n[best_i], 16'd64);
                    player_r <= (cmp_x1[best_i] - cmp_x0[best_i] + 1'b1 +
                                 cmp_y1[best_i] - cmp_y0[best_i] + 1'b1) * 32'd16;
                    blob_x0  <= cmp_x0[best_i];
                    blob_x1  <= cmp_x1[best_i];
                    blob_y0  <= cmp_y0[best_i];
                    blob_y1  <= cmp_y1[best_i];
                    blob_area<= cmp_n[best_i];
                end else begin
                    player_valid <= 1'b0;
                end
                n_labels <= n_label[8:0];
                done     <= 1'b1;
                state    <= C_IDLE;
            end
            default: state <= C_IDLE;
            endcase
        end
    end
endmodule
