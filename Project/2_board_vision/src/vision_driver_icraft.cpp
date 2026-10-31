// ===========================================================================
// vision_driver_icraft.cpp —— 板上那份 :class:`RegIo`（**需要 icraft SDK**）
//
// 单独一个编译单元，理由：只有这个文件依赖 `icraft-xrt` / `icraft-utils`。
// 本机（PC）没有 aarch64 的 icraft 运行时，所以 CMake 里把它列成可选目标；
// 不编它，驱动的顺序逻辑照样能在 PC 上测（用 MemoryRegIo）。
//
// API 用法照厂商示例 `fpai_demo_app/examples/.../src/adder_demo.cpp`：
//
//     auto device = Device::Open("axi://zg330aiu?npu=0x40000000&dma=0x80000000");
//     auto fpai  = device.cast<FPAIDevice>();
//     fpai.defaultRegRegion().write(reg_base + 0x4, reg0, false);
//     uint32_t v = fpai.defaultRegRegion().read(reg_base + 0x80, false);
//
// 注意 `false` 那个参数是 `relative` —— 我们**始终用绝对地址**，
// 因为契约里写的就是绝对偏移 + 基址，用相对地址会让两边对不上。
// ===========================================================================
#include "vision_driver.hpp"

#include <icraft-xrt/core/session.h>   // Device, RegRegion
#include "et_device.hpp"               // FPAIDevice（icraft-utils）

#include <string>

using namespace icraft::xrt;
using namespace fpai;

namespace vision_board {
namespace {

//: 把 icraft 的寄存器区包成 :class:`RegIo`。
//: 地址一律 = kRegBase + off，与 vision_regs.hpp 的契约一致。
class IcraftRegIo final : public RegIo {
public:
    explicit IcraftRegIo(std::string url) : url_(std::move(url)) {
        device_ = Device::Open(url_.c_str());
        fpai_   = device_.cast<FPAIDevice>();
    }
    ~IcraftRegIo() override {
        // 与厂商示例一致：用完显式关掉，别指望析构顺序。
        Device::Close(device_);
    }

    uint32_t read(uint32_t off) const override {
        return static_cast<uint32_t>(
            fpai_.defaultRegRegion().read(kRegBase + off, false));
    }
    void write(uint32_t off, uint32_t value) override {
        fpai_.defaultRegRegion().write(kRegBase + off, value, false);
    }

private:
    std::string url_;
    Device device_;
    FPAIDevice fpai_;
};

}   // namespace

RegIo* icraft_reg_io(const std::string& device_url) {
    return new IcraftRegIo(device_url);
}

}   // namespace vision_board
