// ===========================================================================
// vision_driver_devmem.cpp —— 板上那份 :class:`RegIo`（**不需要 icraft**）
//
// 直接 `/dev/mem` + `mmap` 到 PL 的寄存器窗口。
//
// 为什么要有这个后端
// ------------------
// 我们这块板子上，PL 的寄存器窗口（`0x43C00000`，见 `vision_regs.hpp` 的
// `kRegBase` 与 `core/fpga/synth/bd_vision.tcl` 里显式钉死的那一段）用
// **`/dev/mem` 就能读写** —— `core/board/tools/` 里的 read_frame / axi_soak
// 已经在真硅上把读写都验过了。
//
// 而 icraft 那条路（`vision_driver_icraft.cpp`）：
//   * `defaultRegRegion()` 是**另一个窗口**（`0x400C0000`），不是我们的 PL；
//   * 链它要交叉编译 + 解开 deb 做 sysroot，多一个依赖。
//
// 板子上自带 g++/cmake，所以"直接 /dev/mem + 在板上编"是最短路径，
// 也少一个依赖。两个后端都实现同一个 :class:`RegIo`，驱动逻辑与决策完全共用。
//
// ⚠ 地址一律**绝对**：`read(off)` 访问 `base + off`，off 是相对窗口的字节偏移，
//   与 `vision_regs.hpp` 的契约一致（不要在这里悄悄改成相对地址）。
// ===========================================================================
#include "vision_driver.hpp"

#include <cerrno>
#include <csetjmp>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace vision_board {
namespace {

//: 地址窗口大小 —— 与 `vision_regs.hpp` 的 ADDR_W=12 一致（4 KB）。
//: 几何帧窗口（0x100..0x8FF）在它里面，所以一次 mmap 就够。
constexpr uint32_t kWindowBytes = 0x1000;

//: PL 没被配置时，访问未译码的地址会吃 SIGBUS。不接的话进程直接崩，
//: 看不出是"位流没加载"。接住它，给一句能查的话。
sigjmp_buf g_probe_jb;
void on_bus(int) { siglongjmp(g_probe_jb, 1); }

uint32_t parse_base(const std::string& url) {
    // 支持 `devmem`、`devmem://`、`devmem://0x43C00000`
    const std::string scheme = "devmem";
    if (url.compare(0, scheme.size(), scheme) != 0) {
        throw std::runtime_error("不是 devmem URL：" + url);
    }
    std::string rest = url.substr(scheme.size());
    while (!rest.empty() && (rest[0] == ':' || rest[0] == '/')) rest.erase(0, 1);
    if (rest.empty()) return kRegBase;                 // 省略 → 用契约基址
    return static_cast<uint32_t>(std::stoul(rest, nullptr, 0));
}

class DevMemRegIo final : public RegIo {
public:
    explicit DevMemRegIo(uint32_t base) : base_(base) {
        fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error(std::string("打开 /dev/mem 失败：") +
                                     std::strerror(errno) + "（需要 root）");
        }
        void* p = ::mmap(nullptr, kWindowBytes, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd_, static_cast<off_t>(base_));
        if (p == MAP_FAILED) {
            const std::string e = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("mmap " + std::to_string(base_) + " 失败：" + e);
        }
        map_ = static_cast<volatile uint8_t*>(p);

        // 第一次访问当探针：位流没加载时给一句明确的错，而不是崩在 SIGBUS 上。
        struct sigaction sa;
        std::memset(&sa, 0, sizeof(sa));
        sa.sa_handler = on_bus;
        ::sigaction(SIGBUS, &sa, nullptr);
        ::sigaction(SIGSEGV, &sa, nullptr);
        if (sigsetjmp(g_probe_jb, 1)) {
            ::munmap(const_cast<uint8_t*>(map_), kWindowBytes);
            map_ = nullptr;
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "访问 " + std::to_string(base_) + " 触发总线错误 —— 该地址没有被 PL 译码。\n"
                "  多半是位流没加载：看 /sys/class/fpga_manager/fpga0/state，\n"
                "  再确认烧的是含 vision_axi 的 bd_vision.bin（不是厂商那份）。");
        }
        (void)*(volatile uint32_t*)(map_ + kOffCtrl);
        // 探针过了就把处理恢复成默认，免得真出问题时把栈跳乱。
        sa.sa_handler = SIG_DFL;
        ::sigaction(SIGBUS, &sa, nullptr);
        ::sigaction(SIGSEGV, &sa, nullptr);
    }

    ~DevMemRegIo() override {
        if (map_) ::munmap(const_cast<uint8_t*>(map_), kWindowBytes);
        if (fd_ >= 0) ::close(fd_);
    }

    uint32_t read(uint32_t off) const override {
        return *reinterpret_cast<volatile uint32_t*>(map_ + (off & (kWindowBytes - 1)));
    }
    void write(uint32_t off, uint32_t value) override {
        *reinterpret_cast<volatile uint32_t*>(map_ + (off & (kWindowBytes - 1))) = value;
    }

    uint32_t base() const { return base_; }

private:
    int fd_ = -1;
    volatile uint8_t* map_ = nullptr;
    uint32_t base_;
};

}   // namespace

RegIo* devmem_reg_io(const std::string& device_url) {
    return new DevMemRegIo(parse_base(device_url));
}

}   // namespace vision_board
