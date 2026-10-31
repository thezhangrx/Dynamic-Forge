// ===========================================================================
// vision_axi_master.v —— 极简 AXI4 读主口（一次一行，INCR burst）
//
// 为什么需要（P0 的第二半）：源帧在 DDR 里（PS 用 udmabuf 放），PL 要把
// 640×480×3 = 921,600 B/帧读进来；用 mmap 逐字推只有 4.4 M 字/s（17.3 ms/帧），
// 而 64 位 HP 口峰值 400 MB/s（2.3 ms/帧）。
//
// 这一版是**功能优先**的极简实现（R7：先把功能跑通，再加行缓存/多 outstanding）：
//   * 一次只处理一条 burst（无 outstanding）；
//   * `base + line*stride` 起，连续读 `beats` 个 64 位字；
//   * 数据按拍从 `m_data/m_valid` 流出（行缓存负责攒）。
//
// ⚠ AXI4 读通道的握手（R7：这里最容易错，所以把不变量写成注释）
//   AR：arvalid & arready 同拍才算发出；发出后不再改 ar*。
//   R ：rvalid & rready 同拍吃掉一个 beat；rlast 与最后一拍同拍。
//       本模块**永远**在能收的时候才拉高 rready（不丢 beat）。
// ===========================================================================
module vision_axi_master #(
    parameter integer AW = 32,
    parameter integer DW = 64
) (
    input  wire          clk,
    input  wire          rst_n,

    // ---- 请求（一次一行）----
    input  wire          start,        // 一拍脉冲：开始读
    input  wire [AW-1:0] base,         // 该行起始字节地址
    input  wire [15:0]   beats,        // 本行要读 64 位字数（640×3/8 = 240）
    output reg           busy,
    output reg           done,         // 读满 beats 个 beat 后一拍脉冲

    // ---- 数据流出（给行缓存）----
    output wire          m_valid,
    output wire [DW-1:0] m_data,
    output wire          m_last,
    //: 下游能不能接这一拍（1 = 能）。接行缓存的 `fill_ready`：
    //: 行缓存一拍只能写 1 个像素、一个 beat 要占 2~3 拍，所以必须反压。
    //: ⚠ 反压时必须**同时**把 rready 拉低并把计数改成按握手算，
    //:   否则"没被接走的那一拍"也被计进 beats（整行少一个字，帧就错位）。
    input  wire          r_en,

    // ---- AXI4 读地址通道 ----
    output reg  [AW-1:0] m_araddr,
    output reg  [7:0]    m_arlen,      // beats-1（AXI 的 len 是"拍数-1"）
    output wire          m_arvalid,
    input  wire          m_arready,
    // ---- 只读调试（R80：主口与从机 beat 记账对不上时，必须能看它以为还剩几拍）----
    output wire [15:0] dbg_left,
    output wire [1:0]  dbg_state,

    // ---- AXI4 读数据通道 ----
    input  wire [DW-1:0] m_rdata,
    input  wire          m_rlast,
    input  wire          m_rvalid,
    output wire          m_rready
);
    localparam [1:0] S_IDLE = 2'd0, S_AR = 2'd1, S_DATA = 2'd2;
    reg [1:0]  st;
    reg [15:0] left;          // 还剩几拍没吃
    reg        ar_done;

    assign m_valid = (st == S_DATA) && m_rvalid;
    assign m_data  = m_rdata;
    assign m_last  = m_rlast;
    assign m_arvalid = (st == S_AR) && !ar_done;
    //: 只在"主口在数据相位 && 下游能接"时收；否则 rready=0（AXI 语义允许一直不收）
    assign m_rready  = (st == S_DATA) && r_en;
    assign dbg_left  = left;
    assign dbg_state = st;

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; done <= 1'b0;
            m_araddr <= 0; m_arlen <= 0; left <= 0; ar_done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)
            S_IDLE: begin
                if (start) begin
                    m_araddr <= base;
                    //: AXI 的 arlen = beats-1；上限 256 拍，240 拍合法
                    m_arlen  <= (beats == 16'd0) ? 8'd0 : (beats[7:0] - 8'd1);
                    left     <= beats;
                    ar_done  <= 1'b0;
                    busy     <= 1'b1;
                    st       <= S_AR;
                end else busy <= 1'b0;
            end
            S_AR: begin
                if (m_arvalid && m_arready) begin
                    ar_done <= 1'b1;
                    st <= S_DATA;
                end
            end
            S_DATA: begin
                if (m_rvalid && m_rready) begin
                    if (left != 16'd0) left <= left - 1'b1;
                    if (m_rlast || left == 16'd1) begin
                        busy <= 1'b0; done <= 1'b1; st <= S_IDLE;
                    end
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
