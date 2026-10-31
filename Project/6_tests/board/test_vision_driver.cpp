// ===========================================================================
// test_vision_driver.cpp —— 驱动逻辑的**本机**测试（不需要板子、不需要 icraft）
//
// 靠 :class:`FakePl` 模拟 `vision_axi` 的行为：写 CTRL 的 start 位之后，
// 第一次读 CTRL 报 busy，第二次报完成并给出 FLEN 与帧窗口。
//
// 这样"写 start → 等 busy 起 → 等 busy 落 → 按 FLEN 读窗口 → 拼字节"这条
// **顺序**在本机就能验，不用等板子。到板子上剩下的变量只有"寄存器地址对不对"
// —— 那个由 core/board/tests/test_regs_match_rtl.py 对着 RTL 逐项核。
//
// 编译：见同目录的 Makefile（或 python -m pytest core/board/tests）
// ===========================================================================
#include "../vision_driver.hpp"
#include "../vision_regs.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace vision_board;

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        ++g_checks;                                                        \
        if (!(cond)) {                                                     \
            ++g_fail;                                                      \
            std::printf("  \xe2\x9c\x97 %s\n", (msg));                     \
        }                                                                  \
    } while (0)

// ---------------------------------------------------------------------------
// 模拟的 PL
// ---------------------------------------------------------------------------
class FakePl final : public RegIo {
public:
    //: 要"产出"的几何帧字节。
    std::vector<uint8_t> frame;
    //: 读 PIXSTAT 时回什么：bit16 = 像素 FIFO 溢出粘滞位，低 16 位 = 剩余字节数。
    uint32_t pixstat = 0;

    uint32_t read(uint32_t off) const override {
        if ((off & 0xFFFu) == kOffPixStat) return pixstat;   // 像素 FIFO 状态由测试摆
        // CTRL 的读有副作用：模拟"帧算完了"。真硬件里这是异步的，
        // 驱动靠轮询等它 —— 这里用"第一次读报忙、第二次读报完"来还原那个时序。
        if ((off & 0xFFFu) == kOffCtrl && started_) {
            ++polls_;
            if (polls_ == 1) {
                return kStatusBusy;                       // 忙
            }
            return kStatusFrameDone | (1u << kStatusFrameCountShift);
        }
        return mem_[off & (MemoryRegIo::kWindow - 1)];
    }

    void write(uint32_t off, uint32_t value) override {
        mem_[off & (MemoryRegIo::kWindow - 1)] = value;
        if ((off & 0xFFFu) == kOffCtrl && (value & kCtrlStart)) {
            started_ = true;
            polls_ = 0;
            // 把 FLEN 与帧窗口摆好（真硬件是算法算出来写进去的）
            mem_[kOffFLen] = static_cast<uint32_t>(frame.size());
            for (size_t j = 0; j < frame.size(); ++j) {
                const uint32_t word = kOffFrame + 4u * static_cast<uint32_t>(j / 4);
                mem_[word] |= static_cast<uint32_t>(frame[j]) << (8u * (j % 4));
            }
        }
    }

    const std::vector<uint32_t>& raw() const { return mem_; }

private:
    std::vector<uint32_t> mem_ = std::vector<uint32_t>(MemoryRegIo::kWindow, 0u);
    mutable bool started_ = false;
    mutable int polls_ = 0;
};

//: 造一个最小但合法的几何帧：头 32 B + 尾 4 B（无障碍、无缺口）。
static std::vector<uint8_t> make_frame(uint32_t seq, uint64_t stamp_ns) {
    std::vector<uint8_t> b(36, 0);
    auto put32 = [&](size_t at, uint32_t v) {
        b[at]     = static_cast<uint8_t>(v & 0xFF);
        b[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        b[at + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
        b[at + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    };
    auto put16 = [&](size_t at, uint16_t v) {
        b[at]     = static_cast<uint8_t>(v & 0xFF);
        b[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    put32(0, 0x31564842u);        // magic 'BHV1'
    put16(4, 1);                  // version
    put16(6, 0);                  // flags
    put32(8, seq);
    put32(12, 0);                 // n_obs
    put32(16, 0);                 // n_gaps
    put16(20, 640);               // img_w
    put16(22, 360);               // img_h
    put32(24, static_cast<uint32_t>(stamp_ns & 0xFFFFFFFFu));
    put32(28, static_cast<uint32_t>(stamp_ns >> 32));
    return b;
}

int main() {
    std::printf("=== vision_driver 本机测试 ===\n");

    // ---- 1) 参数摊平的顺序 ------------------------------------------------
    {
        VisionParams p;
        const auto r = p.to_regs();
        CHECK(r.size() == static_cast<size_t>(kCfgCount),
              "to_regs() 的个数必须等于 kCfgCount");
        // 抽查几个有辨识度的默认值，顺序错了就会在这里露馅
        CHECK(r[kCfgWallRb] == p.wall_rb, "第 0 个字段必须是 wall_rb");
        CHECK(r[kCfgGreenMin] == p.green_min, "第 6 个字段必须是 green_min");
        CHECK(r[kCfgBandRowPct] == p.band_row_pct, "第 7 个字段必须是 band_row_pct");
        CHECK(r[kCfgMinGapPx] == p.min_gap_px, "第 16 个字段必须是 min_gap_px");
        CHECK(r[kCfgPlayerMaxAspect] == p.player_max_aspect,
              "第 20 个字段必须是 player_max_aspect");
    }

    // ---- 2) 配置真的写进了正确的偏移 -------------------------------------
    {
        MemoryRegIo io;
        VisionDriver drv(io);
        VisionParams p;
        drv.configure(p);
        for (uint32_t i = 0; i < kCfgCount; ++i) {
            const uint32_t want = static_cast<uint32_t>(p.to_regs()[i]);
            const uint32_t got = io.at(kOffConfig + cfg_offset(i));
            CHECK(got == want, "配置寄存器写错了偏移或值");
        }
    }

    // ---- 3) 自检 ----------------------------------------------------------
    {
        MemoryRegIo io;
        VisionDriver drv(io);
        std::string why;
        CHECK(drv.self_test(&why), "自检应当通过（读写回同一块寄存器）");
    }

    // ---- 4) 取一帧：字节必须**逐位**拼回来 -------------------------------
    {
        FakePl pl;
        pl.frame = make_frame(/*seq=*/7, /*stamp_ns=*/123456789012345ull);
        VisionDriver drv(pl);
        const auto got = drv.run_frame(7, 123456789012345ull);

        CHECK(got.size() == pl.frame.size(), "读回来的长度必须等于 FLEN");
        CHECK(got == pl.frame, "读回来的字节必须与 PL 产出的一模一样");

        const FrameHeader h = parse_header(got);
        CHECK(h.seq == 7, "帧头 seq 应为 7");
        CHECK(h.stamp_ns == 123456789012345ull, "帧头 stamp_ns 应原样带出");
        CHECK(h.img_w == 640 && h.img_h == 360, "帧头尺寸应原样带出");
    }

    // ---- 5) 写进去的 seq/stamp 落在正确的寄存器上 ------------------------
    {
        FakePl pl;
        pl.frame = make_frame(1, 0);
        VisionDriver drv(pl);
        drv.run_frame(/*seq=*/0xAABBCCDDu, /*stamp_ns=*/0x1122334455667788ull);
        CHECK(pl.raw()[kOffSeq] == 0xAABBCCDDu, "seq 应写到 kOffSeq");
        CHECK(pl.raw()[kOffStamp0] == 0x55667788u, "stamp 低 32 位应写到 kOffStamp0");
        CHECK(pl.raw()[kOffStamp1] == 0x11223344u, "stamp 高 32 位应写到 kOffStamp1");
    }

    // ---- 6) 反面控制：magic 不对必须抛，不能静默返回 ---------------------
    {
        FakePl pl;
        pl.frame = make_frame(1, 0);
        pl.frame[0] = 0x00;                        // 破坏 magic
        VisionDriver drv(pl);
        const auto bad = drv.run_frame(1, 0);
        bool threw = false;
        try {
            (void)parse_header(bad);
        } catch (const std::exception&) {
            threw = true;
        }
        CHECK(threw, "magic 不对时 parse_header 必须抛，不能静默通过");
    }

    // ---- 7) 反面控制：FLEN 为 0 必须抛 -----------------------------------
    {
        MemoryRegIo io;
        VisionDriver drv(io);
        // 直接摆一个 busy=0、FLEN=0 的寄存器文件
        io.at(kOffCtrl) = kStatusFrameDone;
        bool threw = false;
        try {
            (void)drv.run_frame(1, 0, /*timeout_ms=*/50);
        } catch (const std::exception&) {
            threw = true;
        }
        CHECK(threw, "FLEN = 0 时必须抛，不能返回空帧当成功");
    }

    // ---- 8) 像素 FIFO 溢出必须被调用方**看见**（而不是只打一行日志）--------
    //
    // 为什么这条重要：溢出时 PL 收到的像素不完整，但产出的几何帧依然"自洽"
    // （CRC 按实际产出算、长度也对）—— 校验过得去、几何是错的。调用方若不知道，
    // 就会拿一张残缺图去决策并发动作（静默错的一类）。
    {
        FakePl pl;
        pl.frame = make_frame(/*seq=*/1, /*stamp_ns=*/0);
        VisionDriver drv(pl);
        const std::vector<uint8_t> rgb(64u * 4u * 4u, 0x80u);   // 4 的倍数即可
        (void)drv.run_frame_push(rgb.data(), rgb.size(), 1, 0, /*timeout_ms=*/200);
        CHECK(!drv.last_push_overflowed(), "正常情况下不该报溢出");
    }
    {
        FakePl pl;
        pl.frame = make_frame(/*seq=*/1, /*stamp_ns=*/0);
        pl.pixstat = (1u << 16) | 12345u;          // 粘滞位置起 + 还剩 12345 字节
        VisionDriver drv(pl);
        const std::vector<uint8_t> rgb(64u * 4u * 4u, 0x80u);
        (void)drv.run_frame_push(rgb.data(), rgb.size(), 1, 0, /*timeout_ms=*/200);
        CHECK(drv.last_push_overflowed(),
              "PIXSTAT 的溢出位必须暴露给调用方（否则只能靠人看日志）");
        CHECK(drv.last_push_buffered() == 12345u,
              "剩余字节数也要带出来，便于定位");
    }

    std::printf("比对项 %d，不符 %d\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
