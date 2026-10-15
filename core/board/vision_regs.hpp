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
constexpr uint32_t kOffFrame  = 0x100u;   // 几何帧字节窗口

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
