// 静态帧检查器：一帧图进去，看它能不能找出"墙"和"缺口"，并把结果画出来。
//
// 为什么要有它：动态的问题（速度、追踪、延迟）在单帧上根本看不出来，
// 而单帧如果都找不对，动态一定不对。所以先用它把**静态**这一关过了：
//   · 找出来的墙，是不是真的墙（y、厚度、左右端点）
//   · 缺口的位置，是不是真的缺口（中心、宽度）
// 判据不靠眼睛：`--expect-gap CX` 可以直接断言"这一帧的缺口中心应该在 CX±t 内"。
//
// 用法：
//   g++ -std=c++17 -O2 -I core/board -o /tmp/wg core/board/tools/wall_gap_probe.cpp core/board/ppm_reader.cpp
//   /tmp/wg --ppm frame.ppm --overlay /tmp/wg_overlay.ppm
//   /tmp/wg --ppm-dir DIR --limit 20 --expect-gap 300 --tol 20
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>

#include "wall_gap.hpp"
#include "ppm_reader.hpp"

namespace {

using vision_board::wall_gap::Wall;
using vision_board::wall_gap::Params;

struct Args {
    std::string ppm, dir, overlay;
    int limit = 0;
    int expect_gap = -1;
    double tol = 20.0;
    Params p;
    bool verbose = false;
};

//: 在 PPM 上画：墙的行带（绿框）、缺口（红框）、缺口中心（红竖线）。
bool draw_overlay(const std::string& in, const std::string& out,
                  const std::vector<Wall>& walls, int pick) {
    int w = 0, h = 0;
    std::vector<uint8_t> rgb;
    std::string why;
    if (!vision_board::read_ppm_p6(in, &w, &h, &rgb, &why)) return false;

    auto set = [&](int x, int y, int r, int g, int b) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        uint8_t* p = rgb.data() + (static_cast<size_t>(y) * w + x) * 3;
        p[0] = static_cast<uint8_t>(r); p[1] = static_cast<uint8_t>(g); p[2] = static_cast<uint8_t>(b);
    };
    auto hline = [&](int y, int xa, int xb, int r, int g, int b, int th = 1) {
        for (int t = 0; t < th; ++t)
            for (int x = xa; x <= xb; ++x) set(x, y + t, r, g, b);
    };
    auto vline = [&](int x, int ya, int yb, int r, int g, int b, int th = 1) {
        for (int t = 0; t < th; ++t)
            for (int y = ya; y <= yb; ++y) set(x + t, y, r, g, b);
    };
    for (size_t i = 0; i < walls.size(); ++i) {
        const auto& wl = walls[i];
        const bool sel = (static_cast<int>(i) == pick);
        const int r = sel ? 0 : 0, g = sel ? 255 : 140, b = sel ? 0 : 0;
        hline(wl.y0, wl.x0, wl.x1, r, g, b, sel ? 2 : 1);
        hline(wl.y1, wl.x0, wl.x1, r, g, b, sel ? 2 : 1);
        vline(wl.x0, wl.y0, wl.y1, r, g, b, 2);
        vline(wl.x1, wl.y0, wl.y1, r, g, b, 2);
        if (wl.has_gap) {
            // 缺口：红框 + 中心竖线
            hline(wl.y0 - 2, wl.gap_x0, wl.gap_x1, 255, 0, 0, 1);
            hline(wl.y1 + 2, wl.gap_x0, wl.gap_x1, 255, 0, 0, 1);
            vline(wl.gap_x0, wl.y0 - 6, wl.y1 + 6, 255, 0, 0, 1);
            vline(wl.gap_x1, wl.y0 - 6, wl.y1 + 6, 255, 0, 0, 1);
            vline(static_cast<int>(wl.gap_cx), wl.y0 - 8, wl.y1 + 8, 255, 128, 0, 1);
        }
    }
    FILE* f = std::fopen(out.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    std::fwrite(rgb.data(), 1, rgb.size(), f);
    std::fclose(f);
    return true;
}

}   // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--ppm") a.ppm = next();
        else if (k == "--ppm-dir") a.dir = next();
        else if (k == "--limit") a.limit = std::atoi(next().c_str());
        else if (k == "--overlay") a.overlay = next();
        else if (k == "--expect-gap") a.expect_gap = std::atoi(next().c_str());
        else if (k == "--tol") a.tol = std::atof(next().c_str());
        else if (k == "--l-bright") a.p.l_bright = std::atoi(next().c_str());
        else if (k == "--row-frac") a.p.row_bright_frac = std::atof(next().c_str());
        else if (k == "--col-frac") a.p.col_bright_frac = std::atof(next().c_str());
        else if (k == "--min-gap") a.p.min_gap_px = std::atoi(next().c_str());
        else if (k == "--verbose") a.verbose = true;
        else if (k == "-h" || k == "--help") {
            std::printf("用法: wall_gap_probe --ppm F | --ppm-dir D [--limit N]\n"
                        "       [--overlay OUT.ppm] [--expect-gap CX --tol T]\n"
                        "       [--l-bright N] [--row-frac F] [--col-frac F] [--min-gap N]\n");
            return 0;
        } else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
    }

    std::vector<std::string> files;
    if (!a.ppm.empty()) files.push_back(a.ppm);
    else if (!a.dir.empty()) {
        const std::string cmd = "ls -1 '" + a.dir + "'/*.ppm 2>/dev/null";
        if (FILE* p = ::popen(cmd.c_str(), "r")) {
            char buf[4096];
            while (std::fgets(buf, sizeof buf, p)) {
                std::string s(buf);
                while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
                if (!s.empty()) files.push_back(s);
            }
            ::pclose(p);
        }
        std::sort(files.begin(), files.end());
    }
    if (files.empty()) { std::fprintf(stderr, "没有可读的帧\n"); return 2; }
    if (a.limit > 0 && static_cast<int>(files.size()) > a.limit)
        files.resize(static_cast<size_t>(a.limit));

    int n_ok = 0, n_wall = 0, n_gap = 0, n_gap_ok = 0;
    double sum_ms = 0;
    for (size_t fi = 0; fi < files.size(); ++fi) {
        int w = 0, h = 0;
        std::vector<uint8_t> rgb;
        std::string why;
        if (!vision_board::read_ppm_p6(files[fi], &w, &h, &rgb, &why)) {
            std::fprintf(stderr, "读不了 %s: %s\n", files[fi].c_str(), why.c_str());
            continue;
        }
        int lb = 0;
        const auto t0 = std::chrono::steady_clock::now();
        const auto walls = vision_board::wall_gap::find_walls(rgb.data(), w, h, a.p, &lb);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        sum_ms += ms; ++n_ok;
        const int pick = vision_board::wall_gap::pick_wall_for_drawing(walls, h);
        if (!walls.empty()) ++n_wall;
        std::printf("帧 %3zu %dx%d 亮阈=%d 墙=%zu 用时=%.2fms", fi, w, h, lb, walls.size(), ms);
        if (pick >= 0) {
            const auto& wl = walls[static_cast<size_t>(pick)];
            std::printf(" | 选中: 行 %d..%d (中心 %.0f, 厚 %d) x=%d..%d 亮列占比=%.2f",
                        wl.y0, wl.y1, wl.cy, wl.y1 - wl.y0 + 1, wl.x0, wl.x1,
                        wl.bright_frac);
            if (wl.has_gap) {
                ++n_gap;
                std::printf(" 缺口 x=%d..%d 中心=%.0f 宽=%d 个数=%d",
                            wl.gap_x0, wl.gap_x1, wl.gap_cx, wl.gap_w, wl.n_gaps);
                if (a.expect_gap >= 0) {
                    const double err = std::fabs(wl.gap_cx - a.expect_gap);
                    const bool ok = err <= a.tol;
                    if (ok) ++n_gap_ok;
                    std::printf("  ← 期望 %d，误差 %.0f%s", a.expect_gap, err, ok ? " ✓" : " ✗");
                }
            } else {
                std::printf(" 缺口=无");
                if (a.expect_gap >= 0) std::printf("  ← 期望 %d ✗", a.expect_gap);
            }
        } else {
            std::printf(" | 没有检出墙");
        }
        std::printf("\n");
        if (a.verbose) {
            for (size_t i = 0; i < walls.size(); ++i) {
                const auto& wl = walls[i];
                std::printf("       墙#%zu y=%d..%d x=%d..%d 亮列=%.2f 缺口=%s(%d)\n", i,
                            wl.y0, wl.y1, wl.x0, wl.x1, wl.bright_frac,
                            wl.has_gap ? "有" : "无", wl.n_gaps);
            }
        }
        if (!a.overlay.empty()) {
            const std::string out = files.size() == 1
                ? a.overlay
                : a.overlay + "." + std::to_string(fi) + ".ppm";
            draw_overlay(files[fi], out, walls, pick);
        }
    }
    std::printf("---- 汇总：%d 帧，有墙 %d，有缺口 %d", n_ok, n_wall, n_gap);
    if (a.expect_gap >= 0) std::printf("，缺口位置正确 %d", n_gap_ok);
    if (n_ok > 0) std::printf("，平均 %.2f ms/帧（%.0f 帧/秒）", sum_ms / n_ok, 1000.0 * n_ok / sum_ms);
    std::printf(" ----\n");
    return 0;
}
