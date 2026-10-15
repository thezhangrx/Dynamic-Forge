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

//: 板上用的 **icraft** 后端。只有它需要 icraft SDK，所以单独一个 .cpp，
//: 本机不编它也能跑测试。实现见 vision_driver_icraft.cpp。
//:
//: ⚠ 注意 `defaultRegRegion()` 是**另一个窗口**（0x400C0000，NPU 那边），
//:   不是我们这块 PL（0x43C00000）。板上跑我们的设计请用 devmem 后端。
RegIo* icraft_reg_io(const std::string& device_url);

//: 板上用的 **/dev/mem** 后端：直接 mmap 到 PL 的寄存器窗口。
//:
//: URL 形如 `devmem://0x43C00000`；写 `devmem` 则用契约里的 :const:`kRegBase`。
//: 板子自带 g++，所以"直接 /dev/mem + 在板上编"是最短路径，不依赖 icraft SDK。
//: 实现见 vision_driver_devmem.cpp。
RegIo* devmem_reg_io(const std::string& device_url);

//: 按 URL 选后端（见 vision_driver_factory.cpp）。`main.cpp` 只用这个，
//: 不认识任何具体后端。
RegIo* open_reg_io(const std::string& device_url);

// ---------------------------------------------------------------------------
// 21 个配置寄存器。字段顺序**必须**等于 ref_api.h 的 VrRefParams。
// 默认值取自 `core/vision/wall_with_gap.py` 的 WallWithGapParams（语义真值），
// 并且 C 参考模型 `vr_ref_default_params()` 会给同一组 —— 三方一致有测试守着。
// ---------------------------------------------------------------------------
// ⚠ **这些默认值必须与 C 参考模型 `vr_ref_default_params()` 逐项相同**
//   （也就等于 `core/vision/wall_with_gap.py` 的 WallWithGapParams，那份是语义真值）。
//   踩过：这里曾经有 13 项和参考模型不一样（band_row_pct 12 vs 5、edge_min 20 vs 12、
//   player_max_area 6000 vs 1500 …），于是 PS 写进 PL 的阈值与**整条验证链**
//   （RTL 复位值 / C 参考模型 / Python 真值）用的那一套不同 —— 表现是"板上几何帧
//   障碍个数对、坐标不对"，而且**完全不报错**。已由
//   `tests/test_params_defaults_match_reference.py` 逐项钉住。
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
    int band_row_pct    = 5;
    int band_join_rows  = 4;
    int band_min_rows   = 5;
    int field_col_min   = 5;
    int wall_col_pct    = 25;
    int wall_col_min    = 2;
    int edge_min        = 12;
    int edge_min_cols   = 1;
    int min_seg_px      = 8;
    int min_gap_px      = 8;
    int player_min_area = 60;
    int player_max_area = 1500;
    int player_min_fill = 45;
    int player_max_aspect = 160;

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
    // -- 真摄像头那条路：PS 把像素推进 PL ----------------------------------
    //
    //  真摄像头采到的帧在 PS 手上（V4L2），而 `vision_top` 的像素来自 PL 侧。
    //  这三件套把帧推给 PL 的像素 FIFO（见 fpga/rtl/vision_pixfeed.v）：
    //
    //      set_pixel_source(true) → flush_pixels() → run_frame_push(...)
    //
    //  **它必须与 :meth:`run_frame` 给出逐字节相同的结果** —— 协同仿真的
    //  `tb_axi --fifo` 就是在证这一条（同一批真实照片，两条像素通路）。

    //: 上一帧的分段耗时（毫秒）。实时那条路上"到底慢在哪"必须实测，不能猜 ——
    //: ffmpeg 单独跑 720p MJPG 有 54 帧/秒，而端到端曾经只有 8.5 帧/秒，
    //: 差的那 ~100ms 全在这两段里（推字 / 等流水线）。
    struct FrameTiming {
        double push_ms = 0.0;   // 把 691200 字节按 32 位字推给 PL
        double wait_ms = 0.0;   // 等 busy 拉高再落 + 读 FLEN + 读窗口
    };
    const FrameTiming& last_timing() const { return timing_; }

    //: 像素源：``false`` = 外部 ``din_*``（板上是 tpg_source），``true`` = PS 的 FIFO。
    void set_pixel_source(bool from_fifo);
    //: 清空像素 FIFO 与溢出标志（换源/开新帧之前调）。
    void flush_pixels();
    //: 读 PIXSTAT：``[15:0]`` 还在缓冲的字节数，``[16]`` 溢出（粘滞），``[17]`` 还在放。
    uint32_t pixel_status() const;

    //: 把一帧 RGB（连续三元组，``w*h*3`` 字节，就是 PPM 的像素区）推进 PL，
    //: 然后取回几何帧字节。**调用方负责先 :meth:`set_pixel_source` 与
    //: :meth:`flush_pixels`。** 返回前会把像素源切回外部。
    std::vector<uint8_t> run_frame_push(const uint8_t* rgb, size_t bytes,
                                        uint32_t seq, uint64_t stamp_ns,
                                        int timeout_ms);

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

    //: 从"已经写了 start"开始：等 busy 拉高再落，按 FLEN 读窗口。
    //: 两条像素通路（外部 din_* / PS 推 FIFO）共用。
    std::vector<uint8_t> finish_frame(int timeout_ms);

    FrameTiming timing_;
};

}   // namespace vision_board

#endif   // VISION_DRIVER_HPP
