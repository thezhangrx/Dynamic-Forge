// ===========================================================================
// vision_linefill.v —— 行缓存未命中时的填充控制（ortho 请求 → 行缓存 / AXI 读口）
//
// 为什么单独做成 RTL（R7「整体意识」）：这段"谁去 DDR 取那一行"的胶水如果写在
// 测试台里，上板时必须再写一遍 —— 那就等于**没验证过的东西直接上板**。
// 所以它可综合、进 lint 门禁、由 tb 实际驱动。
//
// 握手（时序在这里写死，不靠"应该差不多"）：
//   ortho 每像素给 (line, px, req)，并在**拿到 src_valid 之前保持 req**。
//   IDLE ：把 req 透给行缓存；命中 → 行缓存 1 拍后直接把 data/data_valid 给 ortho
//          （数据不经过本模块）；未命中 → 记下 miss_line，进 GO
//   GO   ：一拍 fill_start（行缓存据此选槽/记 tag）+ 一拍 m_start
//          （base = line*1920，beats = 240 个 64 位字）
//   FILL ：把 master 的 data/valid/last 直通给行缓存；m_done → 回 IDLE
//          此时 tag 已经写好，ortho 那个还没被满足的 req 下一次就命中
//
// ⚠ 行缓存的 `line` 端口同时用于"读命中判断"和"fill_start 时选槽/记 tag"：
//   所以填充期间必须把它切到**锁存的 fill_line**。实测 ortho 在等待期间确实
//   一直停在同一像素（line 不变），但让 RTL 依赖这个巧合是隐患 —— 锁存一次不值钱。
// ===========================================================================
module vision_linefill #(
    parameter integer LINE_BYTES = 1920
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- ortho 的源请求（读侧）----
    input  wire [15:0] line,
    input  wire [9:0]  px,
    input  wire        req,

    // ---- 行缓存 ----
    output wire [15:0] lc_line,
    output wire [9:0]  lc_px,
    output wire        lc_req,
    input  wire        lc_miss,
    input  wire [15:0] lc_miss_line,
    output wire        fill_start,
    output wire [63:0] fill_data,
    output wire        fill_valid,
    output wire        fill_last,

    // ---- AXI 读主口 ----
    output wire        m_start,
    output wire [31:0] m_base,
    output wire [15:0] m_beats,
    input  wire        m_busy,        //: 主口在跑 burst 时**不许**发新 start（否则被丢）
    input  wire [63:0] m_data,
    input  wire        m_valid,
    input  wire        m_last,
    //: 行缓存的"填完了"脉冲（**唯一的完成判据**，不用主口的 m_done）：
    //: 主口把最后一个 beat 交出去就拉 m_done，而行缓存还要 2~3 拍把它写成像素、
    //: 才写 tag。若在 m_done 就回 IDLE，ortho 那个还没被满足的 req 会立刻再报一次
    //: miss（tag 还没写）→ 同一行填两遍、还把上一遍的尾巴写坏
    //: （实测：752 次填充 = 2×376，输出 8.2 万个字节不符、最大差 220）。
    //: ⚠ 两个脉冲**不是同拍**（m_done 早 2~3 拍），所以不能写成 `m_done && fill_done`
    //:   —— 那样永远等不到（实测直接挂死超时）。
    input  wire        fill_done,

    // ---- 只读调试口（填充次数：一帧该等于"新出现的相机行数"）----
    output wire [15:0] dbg_fills,
    output wire [1:0]  dbg_state
);
    localparam [1:0]  S_IDLE = 2'd0, S_GO = 2'd1, S_FILL = 2'd2;
    //: 一行 1920 B = 240 个 64 位字（AXI arlen = 239）
    //: ⚠ 位宽：`LINE_BYTES/8` 求值是 32 位（整数参数），直接赋 16 位会报 WIDTHTRUNC；
    //:   所以先按 32 位收下、再显式截到 16 位（不是"随手加的括号"，是过 lint 门禁的写法）。
    localparam integer BEATS_I = LINE_BYTES / 8;

    reg [1:0]  st;
    reg [15:0] fill_line;
    reg [15:0] nfill;

    wire idle = (st == S_IDLE);

    assign lc_req  = idle && req;
    assign lc_line = idle ? line : fill_line;
    assign lc_px   = px;

    assign fill_start = (st == S_GO);
    assign fill_data  = m_data;
    assign fill_valid = m_valid;
    assign fill_last  = m_last;

    assign m_start = (st == S_GO);
    assign m_beats = BEATS_I[15:0];
    //: base = line*1920 = (line<<11) - (line<<7) —— 与行缓存槽基址是**同一个式子**，
    //: 别处一旦改成乘 1920 就会多一个乘法器（这条注释是给未来的自己看的）。
    wire [31:0] fl32 = {16'd0, fill_line};
    assign m_base = (fl32 << 11) - (fl32 << 7);

    assign dbg_fills = nfill;
    assign dbg_state = st;

    always @(posedge clk) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            fill_line <= 16'd0;
            nfill     <= 16'd0;
        end else begin
            case (st)
            S_IDLE: begin
                //: ⚠ 必须等主口空闲再发 start：`m_start` 只有一拍，主口在 S_DATA/S_AR 时不认，
                //: 那一拍就永久丢了 —— 表现是行缓存进 F_RX 等 beat、主口 idle，**死锁**
                //: （tb_p0 集成 cosim 第 169 次填充处复现：line=94/px=36 卡死、fills 不再涨）。
                //: 被挡住时 `lc_req` 仍为 1（本态），下一拍会再判一次 miss，不会漏。
                if (req && lc_miss && !m_busy) begin
                    fill_line <= lc_miss_line;
                    st        <= S_GO;
                end
            end
            S_GO: st <= S_FILL;
            S_FILL: begin
                //: m_done 在这里只表示"主口把最后一个 beat 交完了"，不表示"写进内存了"

                //: ⚠ 只在**行缓存填完**（fill_done）时回 IDLE：提前回会让"最后一拍还没
                //: 写成像素"就重新请求（见端口注释里的实测数字）。
                if (fill_done) begin
                    st    <= S_IDLE;
                    nfill <= nfill + 16'd1;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
