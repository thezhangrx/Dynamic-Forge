// ===========================================================================
// test_rgb_source.cpp —— `StdinPump` 的**采集时间戳**必须是真的
//
// 为什么单独立一条：几何帧的 `stamp_ns` 是"采集时刻"，而速度 = Δ位置 / Δstamp。
// 主循环以前每帧固定 `+= 33.3 ms`（按 30fps 估），可相机实测交付 40~48 fps ——
// 于是推出来的速度被系统性缩小 (30/实际fps) 倍（40fps 时低 28%，60fps 时低一半），
// 而决策的 `t_arr = 距离 / v_n` 直接吃这个数。
//
// 这里不测整个 app（那要 40 秒编译），只测 `StdinPump` 这一件事：
// 从管道喂帧、中间 sleep，看 `last_stamp_ns()` 的差是否**跟着真实间隔走**。
//
// 编译（pytest 里做）：
//   g++ -std=c++17 -O2 -Wall -Wextra -Werror -Icore/board
//       tests/test_rgb_source.cpp rgb_source.cpp -o test_rgb_source -lpthread
// ===========================================================================
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>
#include <unistd.h>

#include "rgb_source.hpp"

static int g_checks = 0, g_fail = 0;

static void CHECK(bool ok, const char* what)
{
    ++g_checks;
    if (!ok) { ++g_fail; std::printf("  ✗ %s\n", what); }
    else     { std::printf("  ✓ %s\n", what); }
}

static constexpr int W = 640, H = 360;
static constexpr size_t FRAME = static_cast<size_t>(W) * H * 3u;

//: 往写端喂 n 帧，帧间 sleep_ms 毫秒。跑在单独线程里，免得把管道塞满后死锁。
static void feed(int fd, int n, int sleep_ms)
{
    std::vector<uint8_t> frame(FRAME, 0x40);
    for (int k = 0; k < n; ++k) {
        size_t off = 0;
        while (off < FRAME) {
            const ssize_t w = ::write(fd, frame.data() + off, FRAME - off);
            if (w <= 0) goto done;               // 读端关了
            off += static_cast<size_t>(w);
        }
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }
done:
    ::close(fd);
}

//: 跑一轮：喂 n 帧、帧间隔 sleep_ms，返回相邻两帧 stamp 的差（纳秒）。
static std::vector<uint64_t> deltas(int n, int sleep_ms, bool* ok)
{
    int fds[2];
    if (::pipe(fds) != 0) { *ok = false; return {}; }
    const int saved_stdin = ::dup(0);
    ::dup2(fds[0], 0);                            // StdinPump 从 fd 0 读
    ::close(fds[0]);

    std::thread th(feed, fds[1], n, sleep_ms);

    vision_board::StdinPump pump(W, H, /*skip=*/0, /*max_frames=*/n);
    std::vector<uint64_t> ts;
    std::vector<uint8_t> buf;
    std::string why;
    while (static_cast<int>(ts.size()) < n && pump.get(&buf, &why)) {
        ts.push_back(pump.last_stamp_ns());
    }
    th.join();
    ::dup2(saved_stdin, 0);                       // 还原
    ::close(saved_stdin);

    *ok = ts.size() >= 3;
    std::vector<uint64_t> d;
    for (size_t i = 1; i < ts.size(); ++i) d.push_back(ts[i] - ts[i - 1]);
    return d;
}

int main()
{
    std::printf("=== StdinPump 采集时间戳 ===\n");
    bool ok = false;

    // ---- 1) 帧间隔 40 ms：stamp 差应当也是 ~40 ms，而不是固定的 33.3 ms ----
    {
        const auto d = deltas(/*n=*/5, /*sleep_ms=*/40, &ok);
        CHECK(ok, "慢速喂帧应当至少拿到 3 帧");
        if (ok) {
            uint64_t mn = d[0], mx = d[0];
            for (uint64_t x : d) { if (x < mn) mn = x; if (x > mx) mx = x; }
            std::printf("    间隔：min %.1f ms  max %.1f ms\n", mn / 1e6, mx / 1e6);
            CHECK(mn > 25'000'000ull, "40ms 间隔下，stamp 差不该小于 25ms");
            CHECK(mx > 30'000'000ull && mx < 200'000'000ull,
                  "40ms 间隔下，stamp 差应当在 30~200ms 之间（跟着真实节奏走）");
        }
    }

    // ---- 2) 全速喂：stamp 差应当**远小于** 33.3ms（证明不是固定步长）----
    {
        const auto d = deltas(/*n=*/8, /*sleep_ms=*/0, &ok);
        CHECK(ok, "全速喂帧应当至少拿到 3 帧");
        if (ok) {
            uint64_t mx = 0;
            for (uint64_t x : d) if (x > mx) mx = x;
            std::printf("    全速最大间隔 %.3f ms\n", mx / 1e6);
            CHECK(mx < 33'000'000ull,
                  "全速喂时 stamp 差必须小于 33.3ms（等于就说明还在用固定步长）");
        }
    }

    std::printf("比对项 %d，不符 %d\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
