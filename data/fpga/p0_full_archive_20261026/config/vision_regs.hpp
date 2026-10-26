// ===========================================================================
// vision_regs.hpp —— PS 与 PL 之间的**寄存器契约**
//
// 这是 PS 侧唯一需要"记住"PL 长什么样的地方。它必须与
// `core/fpga/rtl/vision_axi.v` 的地址映射**逐字一致**：
//
//   0x000 + 4*i   i = 0..20   配置寄存器（阈值/检测参数）
//   0x080         写：调试选择 {4'b0, sub[3:0], idx[17:0], sel[3:0]}
//   0x084         读：调试数据
//   0x0C0         读写：控制/状态
//   0x0C4         写：seq（进几何帧头）
//   0x0C8/0x0CC   写：stamp_ns 低/高 32 位
//   0x0D0         读：上一帧实际字节数
//   0x100 + 4*j   读：几何帧字节窗口，第 j 个字（小端）
//
// ⚠ **为什么把这张表单独拎出来**：PS 和 PL 是两个人在改的两份代码，
// 地址错一个字节，行为就静默错掉（读回来的是别的寄存器）。所以：
//   1. 映射只在这一个文件里定义一次；
//   2. 有一条测试**直接解析 vision_axi.v 的 localparam** 与本文件核对
//      （core/board/tests/test_regs_match_rtl.py）——漂移当场报错。
//
// 基址 0x400C0000 是复旦微 AI_Mate 的 **user_op 窗口**（见 开发板架构.md §4.2）。
// ===========================================================================
#ifndef VISION_REGS_HPP
#define VISION_REGS_HPP

#include <cstdint>

namespace vision_board {

// ---------------------------------------------------------------------------
// 基址
// ---------------------------------------------------------------------------
//: AI_Mate 的 user_op 窗口起点。vendor 示例 adder_demo.cpp 用的也是这个。
constexpr uint32_t kRegBase = 0x43C00000u;

// ---------------------------------------------------------------------------
// 配置寄存器：0x000 + 4*i，i 就是 vision_top 的 cfg_addr
//
// 字段顺序**必须**等于 core/fpga/ref/ref_api.h 的 VrRefParams，
// 而 VrRefParams 又必须等于 core/vision/wall_with_gap.py 的默认值 ——
// 三层各有测试守着，见 core/fpga/tests/test_rtl_contract.py。
// ---------------------------------------------------------------------------
enum CfgAddr : uint32_t {
    // ① 逐像素分类
    kCfgWallRb       = 0,   // R - B 下限（橙墙）
    kCfgWallMinR     = 1,   // 墙：R 下限
    kCfgWallMinG     = 2,   // 墙：G 下限
    kCfgBrightMin    = 3,   // 亮：min(R,G,B) 下限
    kCfgBrightSpread = 4,   // 亮：max-min 上限（高饱和色不算白）
    kCfgGreenGm      = 5,   // 绿：G - max(R,B) 下限
    kCfgGreenMin     = 6,   // 绿：G 下限
    // ②..⑥ 检测参数
    kCfgBandRowPct   = 7,
    kCfgBandJoinRows = 8,
    kCfgBandMinRows  = 9,
    kCfgFieldColMin  = 10,
    kCfgWallColPct   = 11,
    kCfgWallColMin   = 12,
    kCfgEdgeMin      = 13,
    kCfgEdgeMinCols  = 14,
    kCfgMinSegPx     = 15,
    kCfgMinGapPx     = 16,
    kCfgPlayerMinArea   = 17,
    kCfgPlayerMaxArea   = 18,
    kCfgPlayerMinFill   = 19,
    kCfgPlayerMaxAspect = 20,
    kCfgCount           = 21,   // 共 21 个（0..20）
};

//: P1a 的自适应亮度参数（cfg 21..28，见 vision_top.v 的 CFG_LUMA_*）。
//: **故意不进 kCfgCount**：那张表是"21 个检测参数"，有测试钉住数量；这 8 个是
//: 另一套判据（luma_mode=1）的参数，默认全关。它们同样会**粘住**，所以驱动开机
//: 要把默认值写一遍，否则上一次谁开过 luma_mode=1，这一次就静默换了一套判据。
enum LumaCfgAddr : uint32_t {
    kCfgLumaMode      = 21,   // 0 = RGB 判据（默认）/ 1 = 自适应亮度判据
    kCfgLumaPct       = 22,   // 亮度分位（默认 88）
    kCfgLumaBias      = 23,   // 阈值 = 分位点 - bias（默认 20）
    kCfgLumaMin       = 24,   // 阈值下限（默认 90）
    kCfgLumaMax       = 25,   // 阈值上限（默认 200）
    kCfgLumaPlayerMin = 26,   // 角色亮度下限（默认 165）
    kCfgLumaEdgeRows  = 27,   // 上下各排除多少行（默认 8）
    kCfgLumaRunMin    = 28,   // 行内亮游程 ≥ 它才算墙（默认 40）
};

//: 配置寄存器块的基址。写第 i 个就是 kRegBase + kOffConfig + cfg_offset(i)。
//: （kOffConfig 就是 0，单列出来是为了与 vision_axi.v 的 ADDR_CONFIG 一一对应，
//:   有一条测试逐项核对两边。）
constexpr uint32_t kOffConfig = 0x000u;

//: 配置寄存器的字节偏移。写第 i 个就是 kRegBase + cfg_offset(i)。
constexpr inline uint32_t cfg_offset(uint32_t i) { return 4u * i; }

// ---------------------------------------------------------------------------
// 调试口
// ---------------------------------------------------------------------------
constexpr uint32_t kOffDbgSel  = 0x080u;   // 写：{4'b0, sub[3:0], idx[17:0], sel[3:0]}
constexpr uint32_t kOffDbgData = 0x084u;   // 读：调试数据

//: 组装 dbg_sel 寄存器的字。字段位置照 vision_axi.v 的 w_d 切片。
constexpr inline uint32_t pack_dbg_sel(uint32_t sel, uint32_t idx, uint32_t sub) {
    return ((sub & 0xFu) << 22) | ((idx & 0x3FFFFu) << 4) | (sel & 0xFu);
}

// ---------------------------------------------------------------------------
// 控制 / 状态
// ---------------------------------------------------------------------------
constexpr uint32_t kOffCtrl   = 0x0C0u;
constexpr uint32_t kOffSeq    = 0x0C4u;
constexpr uint32_t kOffStamp0 = 0x0C8u;   // stamp_ns 低 32 位
constexpr uint32_t kOffStamp1 = 0x0CCu;   // stamp_ns 高 32 位
constexpr uint32_t kOffFLen   = 0x0D0u;   // 只读：上一帧实际字节数
constexpr uint32_t kOffPixPush = 0x0D4u;   // 写：像素 FIFO 推入（整字写，4 字节打包）
constexpr uint32_t kOffPixStat = 0x0D8u;   // 读：[15:0]缓冲字节 [16]溢出 [17]还在放
constexpr uint32_t kOffPixCtl  = 0x0DCu;   // 写：bit0 像素源(0=外部/TPG,1=FIFO) bit1 flush
constexpr uint32_t kOffFrame  = 0x100u;   // 几何帧字节窗口

// ---------------------------------------------------------------------------
// 像素 FIFO 诊断口（0x0E0..0x0F8，只读）
//
// 只给**离线取证的驱动**用（`pl_vision` 的推入校验、`fifoprobe.py`）：
// 0x0E0/0x0E4/0x0E8 是"PS 推进来的字节数 / 异或 / 和"，用来证明"PL 原样收到了
// 我这一帧"（错位后的几何帧自己 CRC 自洽，外部无从发现，所以必须校验）。
// 之所以要把它们写进契约：`pl_vision.cpp` 原来直接拿 0x0E0 这种裸数字去读，
// 一旦 RTL 改地址就静默读到别的寄存器。
// ---------------------------------------------------------------------------
constexpr uint32_t kOffPixDbg0 = 0x0E0u;   // 读：推入字节数（flush 清零）
constexpr uint32_t kOffPixDbg1 = 0x0E4u;   // 读：推入内容异或
constexpr uint32_t kOffPixDbg2 = 0x0E8u;   // 读：推入内容和
constexpr uint32_t kOffPixZStrb = 0x0ECu;  // 读：被忽略的 WSTRB=0 写次数
constexpr uint32_t kOffPixMem  = 0x0F0u;   // 写：FIFO 字地址；读：该字内容
constexpr uint32_t kOffPixPtr  = 0x0F4u;   // 读：{level[11:0], nb[3:0]}
constexpr uint32_t kOffPixCommit = 0x0F8u; // 读：打到 PIXPUSH 的写次数（含被合并的）

// ---------------------------------------------------------------------------
// P0（正射进 PL）寄存器块：0x900..0x95F
//
// 0x000..0x07F 是 vision_top 的配置块，0x080..0x0FF 是调试/控制，0x100..0x8FF 是
// 几何帧窗口（2048 B）——所以 0x900 起是空的，P0 就放在这里。
//
// 为什么这 9 个系数在 PS 算、不在 PL 算：单应求逆要浮点除法，PS 上是一次性的
// （换标定才重算），PL 里只做"整数乘加 + 一次除法/像素"，两者逐位一致由构造保证
// （`core/fpga/tools/ortho_fixed_ref.cpp` 是同一个式子）。
// ---------------------------------------------------------------------------
constexpr uint32_t kOffFrmCtl   = 0x900u;  // 写: bit0 像素源(1=正射) bit1 start bit2 清完成
                                           // 读: {fills[15:0], 13'b0, done, busy, src}
constexpr uint32_t kOffFrmBase  = 0x904u;  // 相机帧物理基址（字节，4 字节对齐）
constexpr uint32_t kOffFrmC0C1  = 0x908u;  // {c1[15:0], c0[15:0]}  页面列范围
constexpr uint32_t kOffFrmR0R1  = 0x90Cu;  // {r1[15:0], r0[15:0]}  页面行范围
constexpr uint32_t kOffFrmH0    = 0x910u;  // h0..h8 = 0x910 + 4*i（h0..h5 Q16.16 / h6..h8 Q8.24）
constexpr uint32_t kOffFrmPy16  = 0x934u;  // PY0_16（Q16.16 行基址）
constexpr uint32_t kOffFrmPs16  = 0x938u;  // PS_16
constexpr uint32_t kOffFrmPy24L = 0x93Cu;  // PY0_24 低 32 位
constexpr uint32_t kOffFrmPy24H = 0x940u;  // PY0_24[39:32]（Q8.24：479.33*2^24 = 8.04e9 > 2^32）
constexpr uint32_t kOffFrmPs24  = 0x944u;  // PS_24
constexpr uint32_t kOffFrmDbg0  = 0x948u;  // {src_line[15:0], src_off[10:0], hit, miss, 3'b0}
constexpr uint32_t kOffFrmDbg1  = 0x94Cu;  // {tag[15:0], fill_state[7:0], flags[5:0], fill_line[1:0]}
constexpr uint32_t kOffFrmDbg2  = 0x950u;  // {flags[5:0], rd_idx[14:0], fills[10:0]}
constexpr uint32_t kOffFrmLine  = 0x954u;  // {fill_line[15:2], axi_left[15:0], axi_state[1:0]}
//: 0x958 只读诊断（2026-10-26）：{2'b0, sof_cnt[5:0], sof_wp[5:0], sof_rp[5:0], cnt[5:0], wp[5:0]}
//: sof_* = camadapt 在**帧首那一拍**看到的缓冲状态（cnt>0 ⇒ 上一帧没排空 ⇒ 整帧右移）。
constexpr uint32_t kOffFrmDiag  = 0x958u;

//: FRMCTL 的位定义 —— **写和读的语义不同**（混用会得到假数）。
//: 实测踩过：把读的 bit1（正射 busy）当成 done，于是第一帧"0.00 ms"就返回了。
constexpr uint32_t kFrmSrc     = 1u << 0;   // 写：像素源 = 正射
constexpr uint32_t kFrmStart   = 1u << 1;   // 写：启动一帧（同时起 vision_top 与正射）
constexpr uint32_t kFrmClrDone = 1u << 2;   // 写：清掉粘滞的 done
constexpr uint32_t kFrmStatusBusy  = 1u << 1;   // 读：正射在跑
constexpr uint32_t kFrmStatusDone  = 1u << 2;   // 读：正射这帧做完了（粘滞）
constexpr uint32_t kFrmFillsShift  = 16;        // 读：[31:16] = 行填充次数

//: 写 kOffCtrl 时置 bit0 = 1 启动一帧（自清）。
constexpr uint32_t kCtrlStart = 1u << 0;

//: 读 kOffCtrl 的位定义（与 vision_axi.v 的 ctrl_rd 一致）。
constexpr uint32_t kStatusBusy        = 1u << 0;
constexpr uint32_t kStatusFrameDone   = 1u << 1;
constexpr uint32_t kStatusFrameCountShift = 8;    // [23:8]
constexpr uint32_t kStatusFrameCountMask  = 0xFFFFu;
constexpr uint32_t kStatusFrameOverflow   = 1u << 24;

//: 几何帧窗口字节数。**必须等于 vision_axi.v 的 FRAME_BYTES 默认值**。
//: 窗口下标用 addr[FW_BITS+1:2] 整段选取，上限 2048。
constexpr uint32_t kFrameWindowBytes = 2048u;
constexpr uint32_t kFrameWindowWords = kFrameWindowBytes / 4u;

// ---------------------------------------------------------------------------
// 便捷换算
// ---------------------------------------------------------------------------
//: 寄存器地址 = 基址 + 偏移。
constexpr inline uint32_t reg(uint32_t off) { return kRegBase + off; }

//: 几何帧窗口第 j 个字的地址。
constexpr inline uint32_t frame_word_addr(uint32_t j) {
    return kRegBase + kOffFrame + 4u * j;
}

//: 把 stamp 的纳秒值拆成 (低 32, 高 32)。
constexpr inline void split_stamp(uint64_t ns, uint32_t& lo, uint32_t& hi) {
    lo = static_cast<uint32_t>(ns & 0xFFFFFFFFull);
    hi = static_cast<uint32_t>(ns >> 32);
}

}   // namespace vision_board

#endif   // VISION_REGS_HPP
