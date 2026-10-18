// 板子上的"墙 + 缺口"实时标注器：从 stdin 吃原始 RGB，把检测结果画回画面，
// 再从 stdout 吐原始 RGB（交给 ffmpeg 编码）。
//
// 用途：把板子**实时**看到的画面和它自己的检测结果叠在一起录成视频 ——
// 光看日志里的数字永远说不清"到底是识别错了，还是输出了但没生效"。
//
// 和 `wall_gap_probe` 的区别：那个是离线单帧检查（读 PPM），这是实时流。
// 检测算法完全一样（同一个 `wall_gap.hpp`），所以视频里看到的就是板子的判断。
//
// 画法：
//   绿框 = 检出的墙行带（上下两条边 + 左右端点）
//   红框 = 缺口
//   橙竖线 = 缺口中心（决策要瞄的位置）
//   青框 = 角色候选（亮、紧凑、近似圆的连通域，只画一个最像的）
//
// 编译（板子上）：
//   g++ -std=c++17 -O2 -I /root/board_src/core/board -o /root/tools/wall_gap_view \
//       /root/tools/wall_gap_view.cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <unistd.h>

#include "wall_gap.hpp"
#include "contour.hpp"
#include "geometry.hpp"
#include "marker_refine.hpp"
#include "player.hpp"

namespace {

using vision_board::contour::Comp;
using vision_board::wall_gap::Wall;

struct Args {
    int w = 640, h = 360;
    int verbose_every = 30;   // 每 N 帧往 stderr 打一行
    int skip = 0;             // 丢掉开头几帧（自动曝光收敛）
    //: 处理够这么多帧就退出（0 = 读到输入结束）。
    //: 为什么要它：录像时如果用 `ffmpeg -t` 截断，编码器的 stdin 是被上游掐断的，
    //: 尾部写不出来（实测报 "Error writing trailer ... Broken pipe"）。
    //: 让**本程序**先退，管道自然 EOF，编码器就能正常收尾。
    long frames = 0;
    double fps_hint = 0;      // 只用于打印
};

inline void put(std::vector<uint8_t>* img, int w, int h, int x, int y,
                int r, int g, int b) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t* p = img->data() + (static_cast<size_t>(y) * w + x) * 3;
    p[0] = static_cast<uint8_t>(r); p[1] = static_cast<uint8_t>(g); p[2] = static_cast<uint8_t>(b);
}
inline void hline(std::vector<uint8_t>* img, int w, int h, int y, int xa, int xb,
                  int r, int g, int b, int th = 2) {
    for (int t = 0; t < th; ++t)
        for (int x = xa; x <= xb; ++x) put(img, w, h, x, y + t, r, g, b);
}
inline void vline(std::vector<uint8_t>* img, int w, int h, int x, int ya, int yb,
                  int r, int g, int b, int th = 2) {
    for (int t = 0; t < th; ++t)
        for (int y = ya; y <= yb; ++y) put(img, w, h, x + t, y, r, g, b);
}
inline void box(std::vector<uint8_t>* img, int w, int h, int x0, int y0, int x1, int y1,
                int r, int g, int b, int th = 2) {
    hline(img, w, h, y0, x0, x1, r, g, b, th);
    hline(img, w, h, y1 - th + 1, x0, x1, r, g, b, th);
    vline(img, w, h, x0, y0, y1, r, g, b, th);
    vline(img, w, h, x1 - th + 1, y0, y1, r, g, b, th);
}

}   // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--w") a.w = std::atoi(next().c_str());
        else if (k == "--h") a.h = std::atoi(next().c_str());
        else if (k == "--skip") a.skip = std::atoi(next().c_str());
        else if (k == "--log-every") a.verbose_every = std::atoi(next().c_str());
        else if (k == "--frames") a.frames = std::atol(next().c_str());
        else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
    }

    const size_t need = static_cast<size_t>(a.w) * a.h * 3;
    std::vector<uint8_t> src(need), out(need);
    vision_board::contour::Params cp;              // 角色候选用（亮、紧凑、近似圆）
    vision_board::wall_gap::Params wp;

    const auto t_start = std::chrono::steady_clock::now();
    long frame = 0, n_wall = 0, n_gap = 0;
    double sum_ms = 0;
    while (true) {
        size_t got = 0;
        while (got < need) {
            const ssize_t r = ::read(0, src.data() + got, need - got);
            if (r < 0) { std::perror("read"); return 1; }
            if (r == 0) return 0;                    // 输入结束
            got += static_cast<size_t>(r);
        }
        ++frame;
        if (frame <= a.skip) continue;                // 热身帧：不标注、不输出

        const auto t0 = std::chrono::steady_clock::now();
        int lb = 0;
        const auto walls = vision_board::wall_gap::find_walls(src.data(), a.w, a.h, wp, &lb);
        const int pick = vision_board::wall_gap::pick_wall_for_drawing(walls, a.h);
        // 角色：**用主循环同一套**（四标记单应 → 场地 ROI 内找实心圆），
        // 这样视频里看到的就是板子真正在用的那个位置，而不是另一套近似判断。
        vision_board::player::Player pl;
        {
            std::vector<rectify::Marker> mk = rectify::find_markers4(src.data(), a.w, a.h);
            if (mk.size() == 4) {
                std::sort(mk.begin(), mk.end(), [](const rectify::Marker& p,
                                                   const rectify::Marker& q) {
                    return (p.x + p.y) < (q.x + q.y);
                });
                rectify::Marker tl = mk[0], br = mk[3];
                rectify::Marker o1 = mk[1], o2 = mk[2];
                rectify::Marker tr = (o1.x >= o2.x) ? o1 : o2;
                rectify::Marker bl = (o1.x >= o2.x) ? o2 : o1;
                const double span = 0.5 * (std::hypot(mk[1].x - mk[0].x, mk[1].y - mk[0].y) +
                                           std::hypot(mk[2].x - mk[3].x, mk[2].y - mk[3].y));
                vision_board::markers::Params rp;
                if (span > 40.0) {
                    const double expect = span * 48.0 / 640.0;
                    rp.min_side_px = static_cast<int>(0.5 * expect);
                    rp.max_side_px = static_cast<int>(2.0 * expect);
                }
                bool rok = false;
                tl = vision_board::markers::refine(src.data(), a.w, a.h, tl, &rok, rp);
                tr = vision_board::markers::refine(src.data(), a.w, a.h, tr, &rok, rp);
                br = vision_board::markers::refine(src.data(), a.w, a.h, br, &rok, rp);
                bl = vision_board::markers::refine(src.data(), a.w, a.h, bl, &rok, rp);
                const std::array<std::array<double, 2>, 4> img = {
                    {{bl.x, bl.y}, {br.x, br.y}, {tr.x, tr.y}, {tl.x, tl.y}}};
                const rectify::Homography h =
                    rectify::solve(img, rectify::contract_markers(640.0, 480.0));
                const auto geo = vision_board::geometry::judge(h, {bl, br, tr, tl},
                                                              a.w, a.h, 640, 480);
                if (geo.valid) {
                    pl = vision_board::player::find(src.data(), a.w, a.h, h, {bl, br, tr, tl},
                                                    640.0, 480.0, 10.0);
                }
            }
        }
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        sum_ms += ms;

        out = src;
        for (size_t i = 0; i < walls.size(); ++i) {
            const auto& wl = walls[i];
            const bool sel = (static_cast<int>(i) == pick);
            // ⚠ 颜色要一次对上人眼直觉：**绿=选中的墙**、暗绿=其它墙、红=缺口、
            //   橙=缺口中心、品红=角色候选。第一版把选中墙画成 (0,255,255)，
            //   那是青色，和"角色"的框一样，看视频时根本分不清哪个是哪个。
            if (sel) box(&out, a.w, a.h, wl.x0, wl.y0 - 3, wl.x1, wl.y1 + 3, 0, 255, 0, 2);
            else     box(&out, a.w, a.h, wl.x0, wl.y0 - 3, wl.x1, wl.y1 + 3, 0, 130, 0, 1);
            if (wl.has_gap) {
                ++n_gap;
                box(&out, a.w, a.h, wl.gap_x0, wl.y0 - 8, wl.gap_x1, wl.y1 + 8, 255, 0, 0);
                vline(&out, a.w, a.h, static_cast<int>(wl.gap_cx), wl.y0 - 14, wl.y1 + 14,
                      255, 128, 0);
            }
        }
        if (!walls.empty()) ++n_wall;
        if (pl.found) {
            const int cx = static_cast<int>(pl.x), cy = static_cast<int>(pl.y);
            const int rr = std::max(6, static_cast<int>(0.5 * std::max(pl.w, pl.h)) + 3);
            box(&out, a.w, a.h, cx - rr, cy - rr, cx + rr, cy + rr, 255, 0, 255);
            vline(&out, a.w, a.h, cx, cy - rr - 5, cy + rr + 5, 255, 0, 255, 1);
            hline(&out, a.w, a.h, cy, cx - rr - 5, cx + rr + 5, 255, 0, 255, 1);
        }

        std::fwrite(out.data(), 1, need, stdout);
        std::fflush(stdout);

        if (a.frames > 0 && frame - a.skip >= a.frames) {
            const double secs = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            std::fprintf(stderr,
                         "---- 共 %ld 帧（跳过 %d 热身帧），用时 %.2f 秒 → %.1f 帧/秒；"
                         "平均检测 %.2f ms ----\n",
                         frame - a.skip, a.skip, secs, (frame - a.skip) / secs,
                         sum_ms / std::max<long>(1, frame - a.skip));
            return 0;
        }
        if (a.verbose_every > 0 && frame % a.verbose_every == 0) {
            std::fprintf(stderr, "帧 %ld 亮阈=%d 墙=%zu 用时=%.1fms",
                         frame, lb, walls.size(), ms);
            if (pick >= 0) {
                const auto& wl = walls[static_cast<size_t>(pick)];
                std::fprintf(stderr, " | 墙行 %.0f 缺口中心 %.0f 宽 %d",
                             wl.cy, wl.gap_cx, wl.gap_w);
            }
            if (pl.found) std::fprintf(stderr, " | 角色 (%.0f,%.0f)", pl.x, pl.y);
            std::fprintf(stderr, "\n");
        }
    }
}
