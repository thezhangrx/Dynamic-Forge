// ===========================================================================
// vision_board_top.v —— 30TAI 板级 bring-up 顶层（**只用一个时钟 + 两个 LED**）
//
// 为什么需要这一层
// ----------------
// `vision_axi` 的 135 个端口（AXI-Lite 从口 + `din_*` 视频输入）真实部署里
// **来自 PS7**，不是外部引脚。直接拿它出比特流会被 DRC 挡下：
//
//     ERROR: [DRC NSTD-1] 135 out of 135 logical ports 无 IOSTANDARD
//     ERROR: [DRC UCIO-1] 135 out of 135 logical ports 无 LOC
//
// 把这两条降级成 Warning 能骗过一个比特流，但那等于让 FPGA 去驱动 135 个
// 电平未定义的引脚 —— **可能烧板**。所以不绕，改成把端口收敛到板子上
// **已知且已约束**的几个：
//
//     sys_clk_p/n  C8/C7   DIFF_SSTL15  200 MHz   （板级晶振）
//     led[0]       J1      LVCMOS15
//     led[1]       M6      LVCMOS15
//
// 引脚与电平抄自厂商 demo 的 `constrs_1/impl_constraints.xdc`。
//
// 里面放什么
// ----------
// **原封不动的 `vision_selftest`**（`auto_run=1` 自转）。也就是说这条比特流里
// 跑的就是那套被 320 帧合成 + 32 帧真实照片逐位对过账的流水线，
// 只是把"帧从哪儿来"换成了片上图案发生器。它不依赖相机、不依赖 PS、不需要标定。
//
// 两个 LED 是**可分辨的**诊断信号：
//
//     led[0]  约 2 Hz 闪烁   = 时钟 / MMCM / 逻辑活着
//     led[1]  每 32 帧翻转一次 ≈ 1 Hz 闪烁（帧率约 66 fps）
//                            = **一帧真的从头跑完了**（frame_done）
//
// 两者一起看就能分开两类故障：
//   * led[0] 不闪 → 时钟或 MMCM 没起来（跟流水线无关）
//   * led[0] 闪、led[1] 不闪 → 时钟没问题，**流水线卡住了** ——
//     而"卡在哪"在协同仿真里已经能复现（参见 tb_vision 的 dump_stuck()）
//
// 时钟：板上晶振是 200 MHz 差分，MMCM 4×/16× 分出 50 MHz。
// 50 MHz 是**推导出来的**：协同仿真实测每帧约 5.6e5 拍，30 fps 只需 16.7 MHz。
// ===========================================================================
`timescale 1ns / 1ps

module vision_board_top (
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    output wire [1:0] led
);

    // ---- 200 MHz 差分输入 -------------------------------------------------
    wire clk200;
    IBUFDS #(.DIFF_TERM("TRUE"), .IOSTANDARD("DIFF_SSTL15")) u_ibufds (
        .I(sys_clk_p), .IB(sys_clk_n), .O(clk200)
    );

    // ---- MMCM：200 MHz → 50 MHz ------------------------------------------
    // VCO = 200 × 4 = 800 MHz（-2 速度等级允许 600..1200）
    // CLKOUT0 = 800 / 16 = 50 MHz
    wire clk50_raw, clkfb_raw, clkfb, locked;
    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKFBOUT_MULT_F    (4.000),
        .CLKFBOUT_PHASE     (0.0),
        .CLKIN1_PERIOD      (5.000),
        .CLKOUT0_DIVIDE_F   (16.000),
        .CLKOUT0_DUTY_CYCLE (0.5),
        .CLKOUT0_PHASE      (0.0),
        .DIVCLK_DIVIDE      (1),
        .REF_JITTER1        (0.0),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKOUT0  (clk50_raw),
        .CLKFBOUT (clkfb_raw),
        .LOCKED   (locked),
        .CLKIN1   (clk200),
        .PWRDWN   (1'b0),
        .RST      (1'b0),
        .CLKFBIN  (clkfb)
    );
    BUFG u_bufg_fb  (.I(clkfb_raw),  .O(clkfb));
    BUFG u_bufg_clk (.I(clk50_raw),  .O(clk));

    // ---- 复位：等 MMCM 锁定后再数 1023 拍 ---------------------------------
    // 不用外部 `sys_rst`：它的有效电平没在厂商约束里写明，猜错会导致
    // "复位一直有效 / 一直无效"两种相反的故障，而且都看不出来。
    // 上电复位不需要外部引脚，行为完全确定。
    reg [9:0] rst_cnt = 10'd0;
    reg       rst_n   = 1'b0;
    always @(posedge clk) begin
        if (!locked) begin
            rst_cnt <= 10'd0;
            rst_n   <= 1'b0;
        end else if (rst_cnt != 10'd1023) begin
            rst_cnt <= rst_cnt + 10'd1;
            rst_n   <= 1'b0;
        end else begin
            rst_n   <= 1'b1;
        end
    end

    // ---- 被验证过的那套自检流水（原封不动，auto_run 自转）----------------
    wire        frame_done, busy, tpg_busy, tpg_done;
    wire [7:0]  dout_data;
    wire        dout_vld, dout_last;
    wire [31:0] dbg_data, cfg_rdata;
    wire [15:0] cur_frame_idx;

    vision_selftest #(.WIDTH(640), .HEIGHT(360), .COOLDOWN(200000)) u_selftest (
        .clk(clk), .rst_n(rst_n),
        .auto_run(1'b1),
        .start(1'b0),
        .frame_idx(16'd0),
        .tpg_busy(tpg_busy), .tpg_done(tpg_done),
        .frame_done(frame_done), .busy(busy),
        .dout_data(dout_data), .dout_vld(dout_vld), .dout_last(dout_last),
        .dbg_sel(4'd0), .dbg_idx(18'd0), .dbg_sub(4'd0),
        .dbg_data(dbg_data), .cur_frame_idx(cur_frame_idx),
        .cfg_rdata_o(cfg_rdata)
    );

    // ---- LED -------------------------------------------------------------
    // led[0]：自由计数器的最高位，约 2 Hz（50e6 / 2^24 ≈ 3 Hz）
    // led[1]：每 32 个 frame_done 翻转一次 —— 帧率约 66 fps 时约 1 Hz
    //         它闪 == 一帧真的从头跑完了；它不闪而 led[0] 闪 == 流水线卡住
    reg [23:0] hb = 24'd0;
    reg [4:0]  frame_div = 5'd0;
    reg        frame_tgl = 1'b0;

    always @(posedge clk) begin
        if (!rst_n) begin
            hb        <= 24'd0;
            frame_div <= 5'd0;
            frame_tgl <= 1'b0;
        end else begin
            hb <= hb + 24'd1;
            if (frame_done) begin
                if (frame_div == 5'd31) begin
                    frame_div <= 5'd0;
                    frame_tgl <= ~frame_tgl;
                end else begin
                    frame_div <= frame_div + 5'd1;
                end
            end
        end
    end

    assign led[0] = hb[23];

    // ---- 片上逐位自检：把"帧跑完了"升级成"帧的字节和协同仿真一样" ----------
    //
    // 关键简化：**几何帧的最后 4 字节本身就是 vision_top 算的 CRC32**（小端）。
    // 所以板上不用再实现一遍 CRC —— 只要把每帧的「字节数 + 这 4 个字节」和
    // 协同仿真里提取的常数比一比就行。CRC32 撞上不同内容的概率是 2^-32。
    //
    // 常数由 `cosim/tb_tpg.cpp` 打印（`EXPECT_SCENE`），来源是
    // `Vvision_selftest` 逐场景跑出来的帧 —— 与硬件 `auto_run` 的图案序列
    // **逐字节相同**（两边都只由 start 脉冲与 frame_idx[2:0] 决定输出）。
    //
    //  图案   0        1        2        3        4        5        6        7
    //  帧长   184（八个图案都一样）
    //  CRC   A9670ED9 621801BE 1EEBDE91 0BC5C6CA 76D2BFA9 C71A351A A5440B40 AEC7F26E
    //
    // 8 个图案全部对上 → led[1] **常亮**；只跑完但没全对 → 仍按帧率闪。
    // 这样"常亮"和"在闪"是两种含义，不会混淆。
    reg [15:0] fbytes    = 16'd0;   // 本帧已收到的字节数
    reg [31:0] ftail     = 32'd0;   // 末 4 字节（最新字节进最高位）
    reg [7:0]  ok_mask   = 8'd0;    // 哪些图案比对通过了

    // 本拍若正在送字节就得算进去 —— `frame_done` 可能与最后一个 `dout_vld`
    // **同拍**（tb 就是这么收的：先收字节，再看 frame_done）。
    wire [15:0] nb_now = dout_vld ? (fbytes + 16'd1) : fbytes;
    wire [31:0] tl_now = dout_vld ? {dout_data, ftail[31:8]} : ftail;

    // ⚠ **按图案号索引，不要按"完成帧计数"索引**。
    //
    // `vision_selftest` 里 `auto_start <= 1` 与 `auto_idx <= auto_idx+1` 是**同一拍**，
    // 而 `run = auto_start`、`idx_now = auto_idx` —— 于是 tpg 在 start 那拍看到的
    // 是 **k+1**：硬件跑的第一帧用的是**图案 1**，不是图案 0。
    // 用 `done_cnt[2:0]` 索引会整体错一位，8 个图案一个都对不上。
    //
    // `cur_frame_idx` 就是 `idx_now`，而且在 `frame_done` 时**稳定**
    // （下一个 start 要等 cooldown），所以直接拿它当图案号最稳 ——
    // 不依赖"第几帧 = 哪个图案"这个推断。
    wire [2:0] pat = cur_frame_idx[2:0];

    reg [31:0] exp_crc;
    always @* case (pat)
        3'd0: exp_crc = 32'hA9670ED9;
        3'd1: exp_crc = 32'h621801BE;
        3'd2: exp_crc = 32'h1EEBDE91;
        3'd3: exp_crc = 32'h0BC5C6CA;
        3'd4: exp_crc = 32'h76D2BFA9;
        3'd5: exp_crc = 32'hC71A351A;
        3'd6: exp_crc = 32'hA5440B40;
        default: exp_crc = 32'hAEC7F26E;
    endcase

    wire frame_ok = (nb_now == 16'd184) && (tl_now == exp_crc);

    always @(posedge clk) begin
        if (!rst_n) begin
            fbytes  <= 16'd0;
            ftail   <= 32'd0;
            ok_mask <= 8'd0;
        end else begin
            if (dout_vld) begin
                fbytes <= fbytes + 16'd1;
                ftail  <= {dout_data, ftail[31:8]};
            end
            if (frame_done) begin
                fbytes <= 16'd0;
                ftail  <= 32'd0;
                if (frame_ok) ok_mask <= ok_mask | (8'd1 << pat);
            end
        end
    end

    // led[1]：八个图案全部逐位对上 = **常亮**；否则按帧率闪
    assign led[1] = (ok_mask == 8'hFF) ? 1'b1 : frame_tgl;

    // ---- 可选：片上探针（读回用，默认不编）--------------------------------
    //
    // 两个 LED 只够回答"跑没跑完 / 字节对不对"，而且要**有人看着板子**。
    // 远程操作时需要一个能从 JTAG 读回来的观测手段，所以留一个开关：
    //
    //   vivado ... synth_design -verilog_define VISION_BOARD_ILA -top vision_board_top
    //
    // 走**真项目 + create_ip**（见 synth/board_ila.tcl）—— 非工程模式下
    // `create_ip` 生成的 IP 进不了 synth_design（`Synth 8-439 ila_0 not found`），
    // 而综合后插 `create_debug_core` 那条路在 2018.3 上 `C_NUM_OF_PROBES`
    // 不是合法属性（实测 `Vivado 12-4444`/`12-711`）。真项目这条是官方路。
    //
    //   probe0 frame_done(1)  probe1 ok_mask(8)  probe2 cur_frame_idx(16)
    //   probe3 nb_now(16)     probe4 tl_now(32)
    //
    // 默认不定义这个宏，所以 `board.tcl` 那条正常构建**不需要**任何 IP。
`ifdef VISION_BOARD_ILA
    ila_0 u_ila (
        .clk    (clk),
        .probe0 (frame_done),
        .probe1 (ok_mask),
        .probe2 (cur_frame_idx),
        .probe3 (nb_now),
        .probe4 (tl_now)
    );
`endif

    wire _unused = &{1'b0, tpg_busy, tpg_done, busy, dout_data,
                     dout_last, dbg_data, cfg_rdata, cur_frame_idx};

endmodule
