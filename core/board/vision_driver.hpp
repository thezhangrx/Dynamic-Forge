// ===========================================================================
// vision_driver.hpp —— PS 侧读视觉链路的驱动
//
// 板上这条链（`core/fpga/rtl/` 里的 `vision_axi`）对 PS 只暴露一件事：
//
//     写配置 → 写 seq/stamp → 写 start → 轮询 busy 落 → 读 frame_len → 读窗口
//
// 本驱动就把这串动作封成 :meth:`VisionDriver::run_frame`，返回**几何帧原始字节**。
// 解码（v3 线协议）不在这里 —— 见 `geometry_frame.hpp`。本文件把它 include 进来，
// 所以 `#include "vision_driver.hpp"` + `parse_header()` 的老用法照样可用
// （`FrameHeader` 的字段没变，只是搬了个家）。
//
// 为什么用"轮询 + BRAM 窗口"而不是 DMA/中断：第一个 bring-up 的目标是
// **能看见东西**。窗口方案不依赖 DMA 描述符、不依赖中断、不依赖 Stream 握手。
// 等链路点亮再换 DMA（`vision_axi.v` 的文件头也这么写）。
//
// **不会撕裂**：`vision_top` 每收到一次 start 只处理一帧就回 IDLE。PS 的顺序是
// "写 start → 轮询 busy 落 → 读 frame_len → 读窗口"，读的时候引擎已经停住、
// 帧缓冲不再被写。**这条是窗口方案能成立的前提，别改成连续采集。**
// ===========================================================================
#ifndef VISION_DRIVER_HPP
#define VISION_DRIVER_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_frame.hpp"
#include "vision_regs.hpp"

namespace vision_board {

// ---------------------------------------------------------------------------
// 寄存器读写抽象
//
// **为什么要有这层**：驱动真正的逻辑是"写 start → 轮询 busy 落 → 读 frame_len
// → 读窗口"这个**顺序**，而它跟"寄存器到底怎么访问"无关。把访问抽出来之后：
//   * 板子上用 icraft 后端（`icraft_reg_io()`）；
//   * 本机单元测试用假后端（`MemoryRegIo`），**顺序逻辑不用上板就能测**；
//   * 想更真一点，可以让假后端去调 C 参考模型。
// 没有这一层，"驱动对不对"就只能等板子到了才知道。
// ---------------------------------------------------------------------------
class RegIo {
public:
    virtual ~RegIo() = default;
    //: 读 32 位寄存器（``off`` 是相对 :const:`kRegBase` 的字节偏移）。
    virtual uint32_t read(uint32_t off) const = 0;
    //: 写 32 位寄存器。
    virtual void write(uint32_t off, uint32_t value) = 0;
};

//: 面向测试的假后端：一块 4 KB 的寄存器文件。
class MemoryRegIo : public RegIo {
public:
    static constexpr uint32_t kWindow = 0x1000;   // ADDR_W = 12
    MemoryRegIo() : mem_(kWindow, 0u) {}
    uint32_t read(uint32_t off) const override;
    void write(uint32_t off, uint32_t value) override;
    //: 供测试直接摆弄寄存器文件（模拟 PL 的行为）。
    uint32_t& at(uint32_t off) { return mem_.at(off & (kWindow - 1)); }
    uint32_t at(uint32_t off) const { return mem_.at(off & (kWindow - 1)); }
    std::vector<uint32_t>& raw() { return mem_; }
    const std::vector<uint32_t>& raw() const { return mem_; }

private:
    std::vector<uint32_t> mem_;
};

//: 板上用的 icraft 后端。**只有它需要 icraft SDK**，所以单独一个 .cpp，
//: 本机不编它也能跑测试。
//: 声明放在这里，实现见 vision_driver_icraft.cpp。
RegIo* icraft_reg_io(const std::string& device_url);

// ---------------------------------------------------------------------------
// 21 个配置寄存器。字段顺序**必须**等于 ref_api.h 的 VrRefParams。
// 默认值取自 `core/vision/wall_with_gap.py` 的 WallWithGapParams（语义真值），
// 并且 C 参考模型 `vr_ref_default_params()` 会给同一组 —— 三方一致有测试守着。
// ---------------------------------------------------------------------------
struct VisionParams {
    // ① 逐像素分类
    int wall_rb       = 40;    // R - B 下限（橙墙）
    int wall_min_r    = 110;   // 墙：R 下限
    int wall_min_g    = 60;    // 墙：G 下限
    int bright_min    = 195;   // 亮：min(R,G,B) 下限
    int bright_spread = 45;    // 亮：max-min 上限
    int green_gm      = 10;    // 绿：G - max(R,B) 下限
    int green_min     = 90;    // 绿：G 下限
    // ②..⑥ 检测参数
    int band_row_pct    = 12;
    int band_join_rows  = 2;
    int band_min_rows   = 4;
    int field_col_min   = 8;
    int wall_col_pct    = 50;
    int wall_col_min    = 3;
    int edge_min        = 20;
    int edge_min_cols   = 2;
    int min_seg_px      = 8;
    int min_gap_px      = 12;
    int player_min_area = 120;
    int player_max_area = 6000;
    int player_min_fill = 30;
    int player_max_aspect = 250;

    //: 按 :enum:`CfgAddr` 的顺序摊平成 21 个数（写寄存器时用它）。
    std::vector<int32_t> to_regs() const;
};

// ---------------------------------------------------------------------------
// 几何帧头（FrameHeader）与解码在 geometry_frame.hpp —— 那边才是线协议的归属，
// 本文件只管寄存器。已经 include 进来了，这里不再重复声明。
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 驱动
// ---------------------------------------------------------------------------
class VisionDriver {
public:
    //: ``io`` 的生命周期由调用方管（板子上是 :func:`icraft_reg_io` 的返回值，
    //: 测试里是自己栈上的 :class:`MemoryRegIo`）。
    explicit VisionDriver(RegIo& io);
    ~VisionDriver();

    VisionDriver(const VisionDriver&) = delete;
    VisionDriver& operator=(const VisionDriver&) = delete;

    // -- 寄存器原语 --------------------------------------------------------
    uint32_t read_reg(uint32_t off) const;
    void     write_reg(uint32_t off, uint32_t value);

    // -- 配置 --------------------------------------------------------------
    //: 写全部 21 个配置寄存器。
    void configure(const VisionParams& p);

    //: 写一个配置寄存器（按 :enum:`CfgAddr` 的序号）。
    void set_cfg(CfgAddr a, int32_t value);

    // -- 取帧 --------------------------------------------------------------
    //: 跑一帧，返回**几何帧原始字节**。
    //: ``timeout_ms`` 内没等到 busy 落就抛 std::runtime_error。
    std::vector<uint8_t> run_frame(uint32_t seq, uint64_t stamp_ns,
                                   int timeout_ms = 2000);

    //: 上一帧的实际字节数（读 FLEN）。
    uint32_t last_frame_len() const;

    // -- 状态 --------------------------------------------------------------
    //: 读 CTRL。位定义见 vision_regs.hpp 的 kStatus*。
    uint32_t status() const;
    bool busy() const;
    uint32_t frame_count() const;
    bool overflow() const;

    // -- 调试口 ------------------------------------------------------------
    uint32_t dbg_read(uint32_t sel, uint32_t idx, uint32_t sub = 0);

    //: 自检：读配置块的一个已知寄存器（0xC0 处的版本号在厂商 demo 里有，
    //: 我们这里读回一个配置寄存器验证"写得进、读得出"）。
    bool self_test(std::string* why = nullptr);

private:
    RegIo& io_;
};

}   // namespace vision_board

#endif   // VISION_DRIVER_HPP
