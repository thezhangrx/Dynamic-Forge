// ===========================================================================
// vision_camadapt.v —— 相机域适配（把 pl_vision.cpp:403 那段搬进 PL）
//
// 为什么必须搬（2026-10-25，R64 里读代码读出来的）：
//   相机把暖白墙拍成**中性偏蓝**（R−B 从 +55 变 −5），所以 PL 的墙判据（绝对阈值 +
//   R−B）在相机原色上直接失效。PS 侧的 `camera_domain_adapt()` 在**正射之后**把图重染色：
//   长亮段的像素染成"暖白"（B 减 60），短亮斑抬到 210 保持中性。P0 把正射搬进 PL 之后，
//   PS 手上不再有页面图 ⇒ 这段必须跟着进 RTL，否则 P1/P0 那套"相机域 300/300 逐字节一致"
//   的结论直接作废（而闭环⑤要比基线，靠的就是它）。
//
// 算法（与 `core/fpga/tools/camadapt_ref.py` 逐字对应，常数来自 pl_vision.cpp）：
//     gray = (77R + 150G + 29B) >> 8
//     逐行：gray >= 150 的**连续段**，长度 >= 40 → 整段算"墙"
//     逐像素：墙        → R=G=v, B = v>60 ? v-60 : 0     （暖白：让 R−B 判据认出来）
//             v >= 150  → R=G=B = 210                   （短亮斑=角色/反光，保持中性）
//             else      → R=G=B = v
//   注意输出**只由 gray 与 wall 决定**（原 RGB 被丢掉），所以延迟线只需存 8 位 gray + 1 位 wall。
//
// 怎么做到"逐行连续段"而不存整行（关键技巧）：
//   一段连续亮像素**长度达到 40 的那一刻**，它这 40 个像素正好就是最近 40 个输入 ——
//   而它们都还在 41 深的延迟缓冲里。于是：
//     * 长度 == 40 那一拍：把缓冲里**除最老那个**以外的 40 格 wall 位置 1（常数掩码，一次 OR）；
//     * 长度 >  40 的每一拍：新进的那个像素直接 wall=1。
//   这样不需要"回头改"，也不需要行缓冲/游程表。
//
// ⚠ 缓冲是 41 深而不是 40：长度到 40 时，缓冲里除最老那格外正好装着这 40 个像素。
// ⚠ `flush`：一帧最后一个像素进缓冲后，缓冲里还剩 <=41 个没吐出去，必须由外部（正交 done 后）
//   拉高 flush 把它们吐完，否则 vision_top 会永远少最后几十个像素（它按 640×360 计数）。
// ===========================================================================
//: ⚠ 数组名不能叫 `buf` —— 那是 Verilog 的**门原语**（buf/not/and…），
//:   直接报 `syntax error, unexpected buf`。（同一个坑在 vision_linecache 里也踩过：cmem。）
module vision_camadapt #(
    parameter integer WIDTH = 640,
    parameter integer DEPTH = 41          // 必须 > RUNMIN，见上面说明
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        flush,             // 帧末把缓冲里剩下的像素吐完

    input  wire        in_vld,
    input  wire        in_sof,
    input  wire        in_eol,
    input  wire [7:0]  in_r,
    input  wire [7:0]  in_g,
    input  wire [7:0]  in_b,

    output reg         out_vld,
    output reg         out_sof,
    output reg         out_eol,
    output reg  [7:0]  out_r,
    output reg  [7:0]  out_g,
    output reg  [7:0]  out_b,

    //: **只读诊断**（2026-10-26）：
    //:   帧首（in_vld&&in_sof）那一拍的 {cnt,wp,rp} —— 判断缓冲是否带着上一帧
    //:   残留（真硅上若 cnt>0，就说明 flush 没排干净 ⇒ 整帧右移那么多拍）。
    //:   再带上当前 {cnt,wp}，打包成 32 位给 0x958。
    output wire [31:0] dbg_diag
);
    localparam [7:0]  DARK   = 8'd150;    // >= 它算"亮"
    localparam integer RUNMIN = 40;       // 水平连续亮段至少这么长才算"墙"
    localparam [7:0]  TINT   = 8'd60;     // 暖白重建的 B 减量（判据要 R−B>40）
    localparam [7:0]  BRIGHT = 8'd210;    // 非墙亮斑抬到的亮度

    localparam integer DW = $clog2(DEPTH);        // 指针位宽
    localparam integer CW = $clog2(DEPTH + 2);    // 计数位宽
    //: 常量都写成与比较对象同宽，避免整串 WIDTHEXPAND（lint 门禁要 0 告警）
    localparam [8:0]  RUNMIN9 = RUNMIN[8:0];
    localparam [CW-1:0] DEPTH_C = DEPTH[CW-1:0];

    // ---- 延迟缓冲：每格 {wall, gray} --------------------------------------
    reg [8:0] dbuf [0:DEPTH-1];
    reg [DW-1:0]  wp, rp;
    reg [CW-1:0]  cnt;
    reg [9:0]     ocol;                  // 输出列（0..WIDTH-1）
    //: ⚠ `out_sof` 必须是**整帧第一像素**（vision_top 的 din_sof 是"帧开始"，
    //: 收到它会 wr_row<=0；做成"每行行首"就会每行被重置 —— 这个 bug 在正射那边踩过）。
    //: 每帧一次的 sof 请求（2026-10-26 修）：`row0_done` 只置不清，导致
    //: `out_sof` 复位后只响一次；真硅上第一帧像素数差 37 拍就永久右移 37 列。
    //: 改成"帧首置位、第一个输出像素清位"，保证**每帧都有且只有一次** sof。
    reg           sof_pending;
    //: 诊断锁存：帧首那拍的 cnt/wp/rp（只在 sof 更新，别的都不碰）
    reg [CW-1:0]  sof_cnt;
    reg [DW-1:0]  sof_wp, sof_rp;
    reg [7:0]     run_len;
    integer       i;

    // ---- 灰化（与 vo_gray / camadapt_ref.py 同一式子）---------------------
    //: 只要高 8 位（>>8）。低 8 位**确实没人用** —— 那不是 bug，是这条式子本身：
    //: 与其为了消警把式子写成两半（更容易抄错），不如显式关掉这一条。
    // verilator lint_off UNUSEDSIGNAL
    wire [15:0] gsum = 16'd77 * in_r + 16'd150 * in_g + 16'd29 * in_b;
    wire [7:0]  gray = gsum[15:8];
    // verilator lint_on UNUSEDSIGNAL
    wire        bright = (gray >= DARK);

    wire [8:0]  rl_next = bright ? ({1'b0, run_len} + 9'd1) : 9'd0;
    wire [8:0]  rl_sat  = (rl_next > 9'd255) ? 9'd255 : rl_next;

    wire        wall_in = (rl_sat >= RUNMIN9);
    wire        mark    = (rl_sat == RUNMIN9);    // 刚好满 40：把此前 39 个一起标上

    wire        emit = (cnt >= DEPTH_C) || (flush && (cnt != {CW{1'b0}}));
    //: 标记时要排除**两格**：wp（最老的那个 = 亮段前一个像素，正在被本次写入覆盖）
    //: 和 wp+1（第二老的 = 亮段**前一个**像素）。
    //: ⚠ 第一版只排除了 wp，于是把亮段前一个像素也标成了墙（实测 36 个像素错、
    //:   最大差正好 = TINT(60)，即"该中性却被减了 60"）。差一是这类窗口的经典坑。
    wire [DW-1:0] wp1 = (wp == DEPTH[DW-1:0] - 1'b1) ? {DW{1'b0}} : (wp + 1'b1);
    wire [8:0]  rd   = dbuf[rp];

    // ---- 输出重染色（只由 {wall, gray} 决定）-----------------------------
    wire [7:0]  ov = (rd[8]) ? rd[7:0] : ((rd[7:0] >= DARK) ? BRIGHT : rd[7:0]);
    wire [7:0]  ob = (rd[8]) ? ((rd[7:0] > TINT) ? (rd[7:0] - TINT) : 8'd0) : ov;

    always @(posedge clk) begin
        if (!rst_n) begin
            wp <= {DW{1'b0}}; rp <= {DW{1'b0}}; cnt <= {CW{1'b0}};
            ocol <= 10'd0; run_len <= 8'd0; sof_pending <= 1'b0;
            sof_cnt <= {CW{1'b0}}; sof_wp <= {DW{1'b0}}; sof_rp <= {DW{1'b0}};
            out_vld <= 1'b0; out_sof <= 1'b0; out_eol <= 1'b0;
            out_r <= 8'd0; out_g <= 8'd0; out_b <= 8'd0;
            for (i = 0; i < DEPTH; i = i + 1) dbuf[i] <= 9'd0;
        end else begin
            out_vld <= 1'b0;

            // ---- 诊断锁存（只读，不参与任何逻辑）----
            if (in_vld && in_sof) begin
                sof_cnt <= cnt; sof_wp <= wp; sof_rp <= rp;
            end

            // ---- 每帧一次的 sof 请求（与数据同拍发出，见下）----
            if (in_vld && in_sof) sof_pending <= 1'b1;
            else if (out_vld)     sof_pending <= 1'b0;

            // ---- 输入侧：写缓冲 + 游程判定 + 满 40 那一拍的"回填" ----
            if (in_vld) begin
                dbuf[wp] <= {wall_in, gray};
                if (mark) begin
                    //: 临时自检（R7 探针）：这条亮段的 40 个像素应当**全在**缓冲里且都亮。
                    //: 打出来看"有没有暗像素被圈进来" —— 一眼分清是 run 判定错还是窗口错。
                    // synthesis translate_off
                    if ($test$plusargs("dbgca")) begin
                        for (i = 0; i < DEPTH; i = i + 1)
                            if ((i[31:0] != {26'd0, wp}) && (i[31:0] != {26'd0, wp1}) &&
                                (dbuf[i][7:0] < DARK))
                                $display("CA_MARK_BAD i=%0d wp=%0d cnt=%0d g=%0d", i, wp, cnt, dbuf[i][7:0]);
                        $display("CA_MARK wp=%0d cnt=%0d gray=%0d", wp, cnt, gray);
                    end
                    // synthesis translate_on
                    //: 除最老那格（= wp，它正好要被这次写入覆盖）之外，把 wall 置 1。
                    //: 这 40 格就是这条亮段的 40 个像素（长度刚好到 40 时它们全在缓冲里）。
                    for (i = 0; i < DEPTH; i = i + 1)
                        if (i[31:0] != {26'd0, wp} && i[31:0] != {26'd0, wp1})
                            dbuf[i][8] <= 1'b1;
                end
                wp <= (wp == DEPTH[DW-1:0] - 1'b1) ? {DW{1'b0}} : (wp + 1'b1);
            end

            // ---- 游程计数（行边界清零）----
            if (in_vld) run_len <= in_eol ? 8'd0 : rl_sat[7:0];
            else if (in_sof) run_len <= 8'd0;
            if (!in_vld && in_sof) run_len <= 8'd0;

            // ---- 输出侧 ----
            if (emit) begin
                out_vld <= 1'b1;
                out_sof <= sof_pending;
                out_eol <= (ocol == WIDTH[9:0] - 10'd1);
                out_r   <= ov;
                out_g   <= ov;
                out_b   <= ob;
                ocol    <= (ocol == WIDTH[9:0] - 10'd1) ? 10'd0 : (ocol + 10'd1);
                rp      <= (rp == DEPTH[DW-1:0] - 1'b1) ? {DW{1'b0}} : (rp + 1'b1);
            end

            // ---- 计数（同拍 push/emit 相互抵消）----
            case ({in_vld, emit})
                2'b10:   cnt <= cnt + {{(CW-1){1'b0}}, 1'b1};
                2'b01:   cnt <= cnt - {{(CW-1){1'b0}}, 1'b1};
                default: cnt <= cnt;
            endcase
        end
    end
    //: 只读诊断打包：{2'b0, sof_cnt[6], sof_wp[6], sof_rp[6], cnt[6], wp[6]}
    assign dbg_diag = {2'b0, sof_cnt, sof_wp, sof_rp, cnt[6-1:0], wp[6-1:0]};

endmodule
