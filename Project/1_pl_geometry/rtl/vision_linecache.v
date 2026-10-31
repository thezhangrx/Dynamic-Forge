// ===========================================================================
// vision_linecache.v —— 32 行窗口（按行填充、按像素随机读）
//
// 2026-10-25 重写（原因：Vivado 综合报 `Synth 8-3391` 推断不出 BRAM）。
// 旧版把行数据按**字节**存（`reg [7:0] cmem[0:61439]`），填充时一拍写 8 个
// **不同地址**的字节 —— 那是 8 个写口，而 BRAM 只有 1 个写口。Verilator 不在乎，
// 真综合直接失败。新版改存**像素**（24 位）：
//
//     reg [23:0] pmem [0:NLINES*640-1];     // 1 写口 + 1 读口，BRAM 可推断
//
// 顺带把 R62 那个"24 位字拼反"的坑从根上消掉：像素在这里就是 {R,G,B}，
// `[23:16]` 一定是 R（填充时按字节序组装，读出来就是原样）。
//
// 填充（240 个 64 位 beat = 1920 B = 640 像素）需要**重排**：8 字节 = 2⅔ 像素，
// 所以相位只在 {0,1,2} 里循环（是 3 选 1，不是桶形移位器）：
//
//     相位 0（字节 0 起） ：出像素 p0,p1（字节 0-2 / 3-5）；carry 留字节 6,7
//     相位 1（字节 8 起） ：出 p2(carry,字节8)、p3(9-11)、p4(12-14)；carry 留字节 15
//     相位 2（字节 16 起）：出 p5(carry,16,17)、p6(18-20)、p7(21-23)；不留
//     每 3 个 beat 正好出 8 个像素 → 240 beat = 640 像素
//
// 一拍只写 1 个像素，一个 beat 要占 2~3 拍 → 填充期间必须**反压** AXI 主口：
// `fill_ready` 给 vision_linefill，再由它接 master 的 `r_en`（master 只在
// `rvalid && rready` 同拍才吃掉一个 beat）。不反压会静默丢 beat（整行错位）。
//
// 代价：一行填充 ≈ 720 拍（旧版 240 拍），但填充只有 ~376 次/帧（R63），
// 合计仍远小于 35 ms 预算。
// ===========================================================================
module vision_linecache #(
    parameter integer PLW    = 640,     // 一行输出像素数
    parameter integer NLINES = 32,      // 窗口行数
    parameter integer IDXW   = 15       // 32*640 = 20480 → 15 位
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- 读（来自 vision_ortho）：行号 + 行内像素列 ----
    input  wire [15:0] line,
    input  wire [9:0]  px,
    input  wire        req,
    output reg  [23:0] data,
    output reg         data_valid,
    output reg         miss,
    output reg  [15:0] miss_line,

    // ---- 填充（来自 vision_axi_master 的 64 位字流）----
    input  wire        fill_start,
    input  wire [63:0] fill_data,
    input  wire        fill_valid,
    input  wire        fill_last,
    output reg         fill_busy,
    output reg         fill_done,
    output wire        fill_ready,      //: 这一拍能接一个 beat（接 master 的 r_en）

    // ---- 只读调试口（R7：没有中间量就只能猜）----
    input  wire [4:0]  dbg_slot,
    output wire [15:0] dbg_tag,
    output wire        dbg_hit,
    output wire [IDXW-1:0] dbg_rd_idx,
    output wire [2:0]  dbg_fstate,
    output wire [2:0]  dbg_ip,
    output wire [1:0]  dbg_ph
);
    //: 一行的像素数用于**编译期检查**（参数必须被引用，否则 lint 报 UNUSEDPARAM）
    // synthesis translate_off
    initial begin
        if (PLW != 640) $error("vision_linecache: PLW 必须是 640（填充重排按 8 字节/beat 硬算的）");
    end
    // synthesis translate_on

    (* ram_style = "block" *) reg [23:0] pmem [0:NLINES*PLW-1];
    reg [15:0] tag [0:NLINES-1];
    assign dbg_tag = tag[dbg_slot];

    reg [4:0]      slot;
    reg [15:0]     cur_line;
    reg [IDXW-1:0] wid;                 // 填充像素地址（+1/像素）
    integer        ti;

    // ---- 填充序列化状态 ----
    localparam [2:0] F_IDLE = 3'd0, F_RX = 3'd1, F_EMIT = 3'd2;
    reg [2:0]  fst;
    reg [63:0] beat;
    reg [1:0]  ph;                      // 相位 0/1/2
    reg [2:0]  np;                      // 本 beat 要写几个像素（2 或 3）
    reg [2:0]  ip;                      // 已写几个
    reg [15:0] carry;                   // 跨 beat 的残留字节（carry[7:0] 在前，即更早的字节）
    //: ⚠ `fill_last` 必须**随 beat 一起锁存**：它是上游 m_last，只在上游给数那一拍有效。
    //:   本模块一个 beat 要写 2~3 拍，等最后一个写拍再去看 fill_last 时它已经掉了 ——
    //:   表现是"tag 永远不写、fill_done 永不拉高、整条链永远 miss"（tb_linecache 实测 72/72 全错）。
    reg        last_q;

    wire [4:0] ask_slot = line[4:0];
    wire       hit      = (tag[ask_slot] == line);
    //: 槽基址 = slot*640 = (slot<<9) + (slot<<7)（640 = 512 + 128）。
    //: 两次移位一次加法：不进乘法器，也不需要 640 这个常数乘。
    //: ⚠ 第一版写成 (s<<10)-(s<<7)-(s<<7) = s*768（640 ≠ 768）—— 读地址直接越界到
    //:   pmem 之外（line 59 → 20775 而不是 17319），读回全 0，表现为"整片像素变黑"。
    //: 中间量按 16 位算再截到 IDXW：直接赋 15 位会报 WIDTHTRUNC（移位结果是 16 位）。
    //: 最大 31*640 = 19840 < 2^15，截位不丢数据。
    wire [14:0] s9 = {10'd0, ask_slot} << 9;      // 31*512  = 15872
    wire [14:0] s7 = {10'd0, ask_slot} << 7;      // 31*128  = 3968  → 31*640 = 19840
    wire [IDXW-1:0] slot_base = s9 + s7;
    wire [IDXW-1:0] rd_idx    = slot_base + {5'd0, px};

    assign dbg_hit    = hit;
    assign dbg_rd_idx = rd_idx;
    assign dbg_fstate = fst;
    assign dbg_ip     = ip;
    assign dbg_ph     = ph;
    assign fill_ready = (fst == F_RX);

    // ---- beat 的 8 个字节（小端：beat[7:0] 是行内更早的那个字节）----
    wire [7:0] b0 = beat[7:0],   b1 = beat[15:8],  b2 = beat[23:16], b3 = beat[31:24];
    wire [7:0] b4 = beat[39:32], b5 = beat[47:40], b6 = beat[55:48], b7 = beat[63:56];

    //: 三个待写像素（按相位选）。像素按字节序组装：行内第一个字节 → [23:16]（R）。
    wire [23:0] px0 = (ph == 2'd0) ? {b0, b1, b2}
                    : (ph == 2'd1) ? {carry[7:0], carry[15:8], b0}
                    :                {carry[7:0], b0, b1};
    wire [23:0] px1 = (ph == 2'd0) ? {b3, b4, b5}
                    : (ph == 2'd1) ? {b1, b2, b3}
                    :                {b2, b3, b4};
    wire [23:0] px2 = (ph == 2'd1) ? {b4, b5, b6} : {b5, b6, b7};
    wire [23:0] pxw = (ip == 3'd0) ? px0 : (ip == 3'd1) ? px1 : px2;

    always @(posedge clk) begin
        if (!rst_n) begin
            data <= 24'd0; data_valid <= 1'b0; miss <= 1'b0; miss_line <= 16'd0;
            fill_busy <= 1'b0; fill_done <= 1'b0;
            slot <= 5'd0; cur_line <= 16'd0; wid <= {IDXW{1'b0}};
            fst <= F_IDLE; beat <= 64'd0; ph <= 2'd0; np <= 3'd0; ip <= 3'd0; carry <= 16'd0;
            last_q <= 1'b0;
            //: tag 复位成无效值（行号最大 479）：否则上电后 tag 是随机数，
            //: 可能"假装命中"某一行 → 读到 pmem 里的垃圾。
            for (ti = 0; ti < NLINES; ti = ti + 1) tag[ti] <= 16'hFFFF;
        end else begin
            fill_done  <= 1'b0;
            miss       <= 1'b0;
            data_valid <= 1'b0;

            // ---- 读口（命中才出数；1 拍延迟）----
            if (req) begin
                if (hit) begin
                    data_valid <= 1'b1;
                    data       <= pmem[rd_idx];
                end else begin
                    miss      <= 1'b1;
                    miss_line <= line;
                end
            end

            // ---- 填充 ----
            case (fst)
            F_IDLE: begin
                if (fill_start) begin
                    slot      <= line[4:0];
                    cur_line  <= line;
                    wid       <= slot_base;
                    ph        <= 2'd0;
                    carry     <= 16'd0;
                    fill_busy <= 1'b1;
                    fst       <= F_RX;
                end
            end
            F_RX: begin
                if (fill_valid) begin
                    beat   <= fill_data;
                    last_q <= fill_last;
                    np     <= (ph == 2'd0) ? 3'd2 : 3'd3;
                    ip     <= 3'd0;
                    fst    <= F_EMIT;
                end
            end
            F_EMIT: begin
                pmem[wid] <= pxw;
                wid       <= wid + {{(IDXW-1){1'b0}}, 1'b1};
                if (ip == np - 3'd1) begin
                    //: 本 beat 最后一个写拍：carry 与相位一起更新（px0/1/2 都已定型）
                    carry <= (ph == 2'd0) ? {b7, b6}
                           : (ph == 2'd1) ? {8'd0, b7}
                           :                16'd0;
                    ph        <= (ph == 2'd2) ? 2'd0 : (ph + 2'd1);
                    fill_busy <= 1'b0;
                    if (last_q) begin
                        tag[slot] <= cur_line;
                        fill_done <= 1'b1;
                        fst       <= F_IDLE;
                    end else begin
                        fst <= F_RX;
                    end
                end else begin
                    ip <= ip + 3'd1;
                end
            end
            default: fst <= F_IDLE;
            endcase
        end
    end
endmodule
