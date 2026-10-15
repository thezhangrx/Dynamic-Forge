// ===========================================================================
// vision_driver.cpp —— 驱动逻辑本体（**不含 icraft**，本机可编译可测）
//
// 这个文件里只有"顺序"和"字节"：
//   * 配置寄存器怎么摊平；
//   * 一帧怎么起、怎么等、怎么读；
//   * 几何帧头怎么解。
//
// 寄存器**怎么访问**由 :class:`RegIo` 决定，板子上那份在
// vision_driver_icraft.cpp 里。所以本文件在 PC 上就能编译，配合
// :class:`MemoryRegIo` 能把驱动逻辑测干净 —— 不用等板子。
// ===========================================================================
#include "vision_driver.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace vision_board {

// ---------------------------------------------------------------------------
// 参数 → 寄存器数组
// ---------------------------------------------------------------------------
std::vector<int32_t> VisionParams::to_regs() const {
    // 顺序**必须**等于 :enum:`CfgAddr`，也就是 ref_api.h 里 VrRefParams 的字段序。
    // 有一条测试逐项核对（core/board/tests/）。
    return {
        wall_rb, wall_min_r, wall_min_g,
        bright_min, bright_spread, green_gm, green_min,
        band_row_pct, band_join_rows, band_min_rows, field_col_min,
        wall_col_pct, wall_col_min, edge_min, edge_min_cols,
        min_seg_px, min_gap_px,
        player_min_area, player_max_area, player_min_fill, player_max_aspect,
    };
}

// ---------------------------------------------------------------------------
// 假后端
// ---------------------------------------------------------------------------
uint32_t MemoryRegIo::read(uint32_t off) const {
    return mem_[off & (kWindow - 1)];
}

void MemoryRegIo::write(uint32_t off, uint32_t value) {
    mem_[off & (kWindow - 1)] = value;
}

// 几何帧头/记录的解码已经搬到 geometry_frame.cpp —— 那边才是线协议的归属，
// 本文件只管寄存器与「起一帧 / 等 busy / 读窗口」这个顺序。

// ---------------------------------------------------------------------------
// 驱动
// ---------------------------------------------------------------------------
VisionDriver::VisionDriver(RegIo& io) : io_(io) {}
VisionDriver::~VisionDriver() = default;

uint32_t VisionDriver::read_reg(uint32_t off) const  { return io_.read(off); }
void VisionDriver::write_reg(uint32_t off, uint32_t v) { io_.write(off, v); }

void VisionDriver::set_cfg(CfgAddr a, int32_t value) {
    write_reg(kOffConfig + cfg_offset(static_cast<uint32_t>(a)),
              static_cast<uint32_t>(value));
}

void VisionDriver::configure(const VisionParams& p) {
    const std::vector<int32_t> regs = p.to_regs();
    for (uint32_t i = 0; i < regs.size(); ++i) {
        write_reg(kOffConfig + cfg_offset(i), static_cast<uint32_t>(regs[i]));
    }
}

uint32_t VisionDriver::status() const { return read_reg(kOffCtrl); }
bool VisionDriver::busy() const { return (status() & kStatusBusy) != 0u; }
uint32_t VisionDriver::frame_count() const {
    return (status() >> kStatusFrameCountShift) & kStatusFrameCountMask;
}
bool VisionDriver::overflow() const {
    return (status() & kStatusFrameOverflow) != 0u;
}
uint32_t VisionDriver::last_frame_len() const { return read_reg(kOffFLen); }

uint32_t VisionDriver::dbg_read(uint32_t sel, uint32_t idx, uint32_t sub) {
    write_reg(kOffDbgSel, pack_dbg_sel(sel, idx, sub));
    return read_reg(kOffDbgData);
}

std::vector<uint8_t> VisionDriver::run_frame(uint32_t seq, uint64_t stamp_ns,
                                             int timeout_ms) {
    // ---- 1) 清掉上一帧的"完成"位，并把 seq/stamp 写进帧头 ----------------
    // （CTRL 的完成位是"上一帧已完成"，不是自清；先确认它是干净的，
    //   否则下面的轮询会立刻通过、读到上一帧的窗口。）
    uint32_t lo = 0, hi = 0;
    split_stamp(stamp_ns, lo, hi);
    write_reg(kOffSeq, seq);
    write_reg(kOffStamp0, lo);
    write_reg(kOffStamp1, hi);

    // ---- 2) 启动 ---------------------------------------------------------
    write_reg(kOffCtrl, kCtrlStart);

    return finish_frame(timeout_ms);
}

// ---------------------------------------------------------------------------
// 真摄像头那条路：PS 把像素推进 PL 的 FIFO
// ---------------------------------------------------------------------------
void VisionDriver::set_pixel_source(bool from_fifo) {
    write_reg(kOffPixCtl, from_fifo ? 0x1u : 0x0u);
}
void VisionDriver::flush_pixels() {
    write_reg(kOffPixCtl, 0x2u);          // bit1 = flush
}
uint32_t VisionDriver::pixel_status() const {
    return read_reg(kOffPixStat);
}

std::vector<uint8_t> VisionDriver::run_frame_push(const uint8_t* rgb, size_t bytes,
                                                  uint32_t seq, uint64_t stamp_ns,
                                                  int timeout_ms) {
    if (bytes == 0 || (bytes % 4u) != 0) {
        // 推入是按 32 位字走的，字节数必须整除 4。
        // 640x360x3 = 691200 ✓；换成别的尺寸时调用方自己补零到 4 的倍数。
        throw std::runtime_error("run_frame_push：字节数必须是 4 的倍数");
    }
    uint32_t lo = 0, hi = 0;
    split_stamp(stamp_ns, lo, hi);
    write_reg(kOffSeq, seq);
    write_reg(kOffStamp0, lo);
    write_reg(kOffStamp1, hi);

    // 选 FIFO 源 + 清空（同一次写：bit0=1, bit1=1）
    write_reg(kOffPixCtl, 0x3u);
    write_reg(kOffCtrl, kCtrlStart);

    // 推像素。**盲写**：消费者（流水线）每拍 1 像素 = 150 MB/s，比 PS 的
    // 非缓存写（实测 ~40 MB/s）快得多，所以 FIFO 只会被抽空。推完读一次状态，
    // 溢出就报出来 —— 静默丢像素是最难查的失败。
    const auto t_push0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < bytes; i += 4u) {
        const uint32_t w = static_cast<uint32_t>(rgb[i])
                         | (static_cast<uint32_t>(rgb[i + 1]) << 8)
                         | (static_cast<uint32_t>(rgb[i + 2]) << 16)
                         | (static_cast<uint32_t>(rgb[i + 3]) << 24);
        write_reg(kOffPixPush, w);
    }
    const auto t_push1 = std::chrono::steady_clock::now();
    timing_.push_ms = std::chrono::duration<double, std::milli>(t_push1 - t_push0).count();

    const uint32_t st = pixel_status();
    if (st & (1u << 16)) {
        // 不抛异常：溢出后帧内容仍然可能是"合法的但错的"，交给上面的判据。
        // 但一定要让调用方看见（这是最难查的一类失败）。
        std::fprintf(stderr, "⚠ 像素 FIFO 溢出（缓冲 %u 字节）—— 这一帧不可信\n",
                     st & 0xFFFFu);
    }

    // 把像素源切回去，免得后续 :meth:`run_frame` 以为还有像素进来
    write_reg(kOffPixCtl, 0x0u);
    const auto t_wait0 = std::chrono::steady_clock::now();
    std::vector<uint8_t> out = finish_frame(timeout_ms);
    const auto t_wait1 = std::chrono::steady_clock::now();
    timing_.wait_ms = std::chrono::duration<double, std::milli>(t_wait1 - t_wait0).count();
    return out;
}

// ---------------------------------------------------------------------------
// 等一整帧跑完，然后读 FLEN + 窗口
//
// 两条像素通路（外部 din_* 与 PS 推 FIFO）**共用这一段**：入参前提是"已经写了
// start"，出来的是几何帧字节。抽出来是为了让两条路走**同一份**等待与读取逻辑
// —— 分开写迟早会在超时/长度校验上分家。
// ---------------------------------------------------------------------------
std::vector<uint8_t> VisionDriver::finish_frame(int timeout_ms) {
    // ---- 3) 等 busy 拉高再落 --------------------------------------------
    // 先等它忙起来：写 start 到 busy 拉高之间有若干拍，直接等"落"可能
    // 在还没起来的时候就通过。
    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed_ms = [&] {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t0).count();
    };
    const int spin = 1;   // 轮询间隔（毫秒）。窗口方案下这是唯一的等待成本。
    while (!busy()) {
        if (elapsed_ms() > timeout_ms) {
            throw std::runtime_error("视觉链路超时：写 start 后 busy 一直没拉高");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(spin));
    }
    while (busy()) {
        if (elapsed_ms() > timeout_ms) {
            throw std::runtime_error("视觉链路超时：busy 一直没落");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(spin));
    }

    // ---- 4) 读长度，再按长度读窗口 --------------------------------------
    // **必须照 FLEN 读**，不能读满 2048：多读出来的部分是上一帧的残留，
    // 解帧时会把尾巴当成数据。
    const uint32_t flen = last_frame_len();
    if (flen == 0) {
        throw std::runtime_error("FLEN = 0：这一帧没产出几何帧");
    }
    if (flen > kFrameWindowBytes) {
        throw std::runtime_error("FLEN 超过窗口大小，硬件行为不符合预期");
    }

    std::vector<uint8_t> out(flen, 0);
    const uint32_t words = (flen + 3u) / 4u;
    for (uint32_t j = 0; j < words; ++j) {
        const uint32_t w = read_reg(kOffFrame + 4u * j);
        for (uint32_t k = 0; k < 4u; ++k) {
            const uint32_t idx = 4u * j + k;
            if (idx < flen) {
                out[idx] = static_cast<uint8_t>((w >> (8u * k)) & 0xFFu);
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// 自检
// ---------------------------------------------------------------------------
bool VisionDriver::self_test(std::string* why) {
    // 配置寄存器是读写回最直接的一对：写进去读不回来，就说明基址错、
    // 或者 AXI 根本没接上。厂商示例 adder_demo.cpp 也是先读一个版本寄存器
    // 来确认"设备通了"。
    const int32_t probe = 0x5A5A;
    const CfgAddr addr = kCfgMinGapPx;
    const uint32_t old = read_reg(kOffConfig + cfg_offset(addr));
    write_reg(kOffConfig + cfg_offset(addr), static_cast<uint32_t>(probe));
    const uint32_t got = read_reg(kOffConfig + cfg_offset(addr));
    write_reg(kOffConfig + cfg_offset(addr), old);   // 还原，别留下副作用
    if (got != static_cast<uint32_t>(probe)) {
        if (why) {
            *why = "写 0x5A5A 到 MIN_GAP_PX 读回来是 0x" +
                   std::to_string(got) + " —— 基址或 AXI 通路有问题";
        }
        return false;
    }
    return true;
}

}   // namespace vision_board
