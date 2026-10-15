// ===========================================================================
// vision_pixfeed.v —— 让 **PS 把一帧像素喂进 PL 的流水线**
//
// 用途：真摄像头采到的帧在 PS 手上（V4L2），而 `vision_top` 的像素来自 PL 侧。
// 这个模块把 PS 通过 AXI 写进来的字节流，还原成 `din_vld/sof/eol/r/g/b` 时序。
//
//     PS: 写 0x0D4（每个 32 位字 = 4 个打包字节）
//          ──▶ 同步 FIFO ──▶ 字节蓄水池 ──▶ 每拍 1 个像素 ──▶ vision_top
//
//     PS 侧那帧的字节布局就是**连续的 RGB 三元组**（R,G,B,R,G,B,…），字节序
//     小端：写进 0x0D4 的 bit[7:0] 是先到的那个字节。所以 PS 那边只要把
//     PPM 的像素区**原样按 32 位字推**即可，不需要每像素补齐到 4 字节。
//
// ## 为什么不需要"PS 等 FIFO 不满"
//
// 消费者（流水线）每拍吃 1 个像素 = 3 字节/拍 = **150 MB/s**（50 MHz 下），
// 而 PS 实测只能写 ~40 MB/s（非缓存写，见 core/board/tools/axi_bw.c）。
// 消费者快 3.75 倍，FIFO 只会被抽空、不会被塞满，所以 PS 可以**盲写**，
// 不需要每写一个字都回读状态（那样每次都要一个 AXI 往返，慢得多）。
// 真溢出也不会丢帧：`ovf` 是粘滞位，PS 读 PIXSTAT 就能发现。
//
// ## 为什么必须等 `sink_ready`（接 `vision_top` 的 `busy`）
//
// `vision_top` 在收到 `din_sof` 之前会**忽略**所有像素，而且 S_RECV 里从不回压。
// 所以放行条件是"流水线已经开始收"（`busy` 恰在进 S_RECV 那一拍拉高），
// 否则开头几个像素被吞掉、整帧错位 —— 而且错得很隐蔽（几何帧长度可能照样对）。
//
// 不额外给 `vision_top` 加一个"当前在 S_RECV"的输出端口：那会让所有例化它的
// 地方（`vision_selftest` 等）都要跟着改，而 `busy` + "一次只放一帧"已经够 ——
// 这个不变量由 `tb_axi --fifo` 的逐字节比对直接证明。
//
// ## 吞吐：为什么是 1 像素/拍而不是 3 拍/像素
//
// 3 字节/像素、4 字节/字，所以字边界与像素边界**每拍都在错位**。这里用一个
// 8 字节的蓄水池吸收错位：只要池里够 3 字节就吐一个像素、同时再补一个字，
// 两者在**同一拍**做完，于是稳态就是 1 像素/拍（与 `vision_tpg` 的节拍完全一致，
// 流水线是按那个节拍验过的）。
// ===========================================================================

module vision_pixfeed #(
    parameter integer WIDTH  = 640,
    parameter integer HEIGHT = 360,
    parameter integer AW     = 11          // FIFO 深度 = 2^AW 个 32 位字（8 KB）
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- AXI 侧（PS 推字节）----
    input  wire        push,               // 一拍推一个 32 位字
    input  wire [31:0] push_data,
    input  wire        flush,              // 清空 FIFO 与 ovf
    output wire [15:0] buffered,           // **还在缓冲的字节数**（FIFO + 蓄水池）
    output reg         ovf,                // 溢出（粘滞，flush 清）

    // ---- 视频侧（给 vision_top）----
    //: 下游（流水线）本拍能吃像素 —— 接 `vision_top` 的 `busy`。
    //:
    //: 为什么 `busy` 就够：它恰好在状态机进入 **S_RECV** 那一拍拉高；而 S_RECV
    //: 只在收满 `WIDTH*HEIGHT` 个像素后才离开。本模块每次"放一帧"也正好放
    //: WIDTH*HEIGHT 个像素，于是两边**严格同步** —— 不会多送（多送的会被丢弃），
    //: 也不会少送（少送会让流水线永远卡在 S_RECV）。
    //: 这条不变量由协同仿真的 `tb_axi --fifo` 直接证明（32 张真照片逐字节一致）。
    input  wire        sink_ready,
    output reg         dout_vld,
    output reg         dout_sof,
    output reg         dout_eol,
    output reg  [7:0]  dout_r,
    output reg  [7:0]  dout_g,
    output reg  [7:0]  dout_b,
    output reg         busy                // 本帧还没放完
);

    // -----------------------------------------------------------------------
    // 同步 FIFO（同一条时钟：AXI 与像素都是 clk，本模块不假装自己有 CDC）
    // -----------------------------------------------------------------------
    reg [31:0] mem [0:(1 << AW) - 1];
    reg [AW:0] wptr, rptr;                 // 多一位用来区分空/满

    wire        empty = (wptr == rptr);
    wire        full  = (wptr[AW-1:0] == rptr[AW-1:0]) && (wptr[AW] != rptr[AW]);
    wire [31:0] dout  = mem[rptr[AW-1:0]];
    wire        pop   = !empty && can_load;

    // 内部量：FIFO 里的字数（不对外 —— 对外报的是下面的 buffered，
    // 因为"FIFO 占用"不含蓄水池预取的那几个字，拿它当接口契约会误导）。
    // 显式零扩展到 16 位：避免"11 位减法赋给 16 位"的 WIDTHEXPAND
    // （-Wall 下是错误级；隐式扩展的写法还容易在改 AW 时静默出错）
    // 宽度按实际需要给：level ≤ 2^AW = 2048 → 12 位足矣（给 16 位会留下
    // 未用的高位，-Wall 的 UNUSEDSIGNAL 照样拦）
    wire [11:0] level = {1'b0, wptr[AW-1:0]} - {1'b0, rptr[AW-1:0]};

    //: 还在缓冲的**字节**数 = FIFO 里的字 * 4 + 蓄水池里的字节。
    //
    //  为什么报这个而不是只报 FIFO 占用：蓄水池会**预取**最多 2 个字（8 字节），
    //  所以"FIFO 占用"比"推入的总量"少几个字 —— 那个差值是实现细节，
    //  拿它做接口契约会让测试变得脆弱（真写测试时就踩到了）。
    //  "还在缓冲的字节数"才是对 PS 有意义的可观测量：推入 N 个字节、还没被消费时
    //  它就该等于 N。
    //  宽度逐段算清楚（这里错过一次：`{2'b0, level, 2'b00}` 是 2+16+2=**20** 位，
    //  不是 18 —— Verilator 把位宽告警当错误，构建直接失败）：
    //    level ≤ 2048（AW=11）→ 12 位够；×4 ≤ 8192 → 14 位
    wire [13:0] lvl4 = {level, 2'b00};                       // 12 + 2 = 14
    assign buffered  = {2'b0, lvl4} + {12'b0, nb};            // 16 + 16 → 16


    // -----------------------------------------------------------------------
    // 字节蓄水池：acc 低位是下一个要吐的字节
    // -----------------------------------------------------------------------
    reg [63:0] acc;
    reg [3:0]  nb;                         // acc 里的有效字节数（0..7）

    wire [23:0] pix      = acc[23:0];      // R,G,B（低位先到）
    wire        can_emit = (nb >= 4'd3) && busy && sink_ready;
    wire        can_load = (nb <= 4'd4);   // 补 4 字节后不超过 8

    wire [5:0]  sh_load  = {2'b0, nb} * 6'd8;         // 补字时的左移量
    wire [5:0]  sh_emit  = ({2'b0, nb} - 6'd3) * 6'd8; // 吐像素后补字的左移量

    // -----------------------------------------------------------------------
    // 像素坐标与 sof/eol（与 vision_tpg 的约定**逐条一致**：
    //   sof 只在 (0,0)、eol 在每行最后一列、数据与 vld/sof/eol 同一拍）
    // -----------------------------------------------------------------------
    reg [15:0] px, py;

    always @(posedge clk) begin
        if (!rst_n) begin
            wptr <= 0; rptr <= 0;
            acc  <= 64'd0; nb <= 4'd0;
            px <= 16'd0; py <= 16'd0;
            ovf  <= 1'b0; busy <= 1'b0;
            dout_vld <= 1'b0; dout_sof <= 1'b0; dout_eol <= 1'b0;
            dout_r <= 8'd0; dout_g <= 8'd0; dout_b <= 8'd0;
        end else begin
            // ---- FIFO 写 ----
            if (flush) begin
                wptr <= 0; rptr <= 0; ovf <= 1'b0;
                acc  <= 64'd0; nb <= 4'd0;
                px <= 16'd0; py <= 16'd0; busy <= 1'b0;
            end else begin
                if (push) begin
                    if (full) ovf <= 1'b1;      // 粘滞：PS 读 PIXSTAT 能看到
                    else begin
                        mem[wptr[AW-1:0]] <= push_data;
                        wptr <= wptr + 1'b1;
                    end
                end
                if (pop) rptr <= rptr + 1'b1;
            end

            // ---- 蓄水池 / 放像素 ----
            if (!flush) begin
                if (can_emit && pop) begin
                    // 同一拍：吐一个像素 + 补一个字
                    acc <= (acc >> 24)
                         | ({32'd0, dout} << sh_emit);
                    nb  <= nb + 4'd1;           // -3 +4
                end else if (can_emit) begin
                    acc <= acc >> 24;
                    nb  <= nb - 4'd3;
                end else if (pop) begin
                    acc <= acc | ({32'd0, dout} << sh_load);
                    nb  <= nb + 4'd4;
                end
            end

            // ---- 输出时序 ----
            dout_vld <= can_emit;
            dout_sof <= can_emit && (px == 16'd0) && (py == 16'd0);
            dout_eol <= can_emit && (px == WIDTH[15:0] - 16'd1);
            if (can_emit) begin
                dout_r <= pix[7:0];
                dout_g <= pix[15:8];
                dout_b <= pix[23:16];
                if (px == WIDTH[15:0] - 16'd1) begin
                    px <= 16'd0;
                    if (py == HEIGHT[15:0] - 16'd1) begin
                        py   <= 16'd0;
                        busy <= 1'b0;           // 一帧放完
                    end else begin
                        py <= py + 16'd1;
                    end
                end else begin
                    px <= px + 16'd1;
                end
            end

            // ---- 什么时候开始放：流水线要一帧，且池里有货 ----
            // 用 CTRL 的 start 触发（PS 那边顺序：flush → start → 推像素）。
            // 这里只要"没在忙 且 池里够一个像素"就可以起 —— 但必须等 din_ready，
            // 否则 dout_sof 会落在流水线还没开始收的时候，整帧错位。
            if (!busy && sink_ready && (nb >= 4'd3)) busy <= 1'b1;
        end
    end

endmodule
