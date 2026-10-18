// 场地坐标系（P1）的离线体检器：逐帧统计"四标记能否定位"和"映射误差"。
//
// 为什么先要它：P1 的退出条件是"场地定位成功率、角点误差、映射误差都有统计结果"，
// 而在此之前所有结论都只有定性描述（"标记基本都在"）。这个工具把三件事一次量清：
//
//   1. `find_markers` / `find_markers4` 各自能给出几个标记、四个都齐的帧占多少
//   2. 用四标记 + 已知场地坐标（contract: 32/608/32/448）解出的单应，**重投影误差**是多少
//      （把场地四角映射回图像，和检出的标记中心比 —— 这是自洽性下限，不是真值精度）
//   3. 单应给出的比例尺/场地包围盒，和 `field_extent`（现有的亮度台阶法）差多少
//
// 用法（在仓库根目录）：
//   g++ -std=c++17 -O2 -I core/board -o /tmp/rp core/board/tools/rectify_probe.cpp core/board/ppm_reader.cpp
//   /tmp/rp --ppm-dir /tmp/vis/rec/ppm --jsonl /tmp/p1.jsonl
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>

#include "band_gap.hpp"
#include "rectify.hpp"
#include "ppm_reader.hpp"

using rectify::Homography;
using rectify::Marker;
using rectify::contract_markers;
using vision_board::read_ppm_p6;

namespace {

struct Args {
    std::string ppm, dir, jsonl;
    int limit = 0;
    double field_w = 640.0, field_h = 480.0;
};

//: 单应的重投影误差：场地四角（已知）→ 像素，与检出的标记中心比。单位：像素。
//:
//: ⚠ `solve(img, field)` 得到的是 **图像→场地**（主循环就是这么用的：拿它把像素映射成
//: 场地单位）。所以重投影必须走**逆变换**：场地角点 → 像素。第一版直接拿 H 去作用场地
//: 坐标，量出来"中位误差 256 px"，那其实是把 H 用错了方向 —— 这种错必须自己先抓住，
//: 否则会被当成"标记定位很烂"去改根本不坏的东西。
double reproj_error(const Homography& H, const std::vector<Marker>& mk) {
    if (!H.ok || mk.size() != 4) return -1.0;
    const Homography inv = invert(H);
    if (!inv.ok) return -1.0;
    const auto c = contract_markers(640.0, 480.0);
    // 约定与 MarkerCache 一致：img = [m0, m1, m3, m2] ↔ contract = [TL, TR, BR, BL]
    const std::array<std::array<double, 2>, 4> img = {{
        {mk[0].x, mk[0].y}, {mk[1].x, mk[1].y}, {mk[3].x, mk[3].y}, {mk[2].x, mk[2].y}}};
    double sum = 0;
    for (int i = 0; i < 4; ++i) {
        const auto p = inv.apply(c[static_cast<size_t>(i)][0], c[static_cast<size_t>(i)][1]);
        sum += std::hypot(p[0] - img[static_cast<size_t>(i)][0],
                          p[1] - img[static_cast<size_t>(i)][1]);
    }
    return sum / 4.0;
}

//: 用四标记解单应（和主循环同一个顺序/同一组契约坐标）。
Homography solve_from_markers(const std::vector<Marker>& mk, double fw, double fh) {
    Homography bad;
    if (mk.size() != 4) return bad;
    const std::array<std::array<double, 2>, 4> img = {{
        {mk[0].x, mk[0].y}, {mk[1].x, mk[1].y}, {mk[3].x, mk[3].y}, {mk[2].x, mk[2].y}}};
    return rectify::solve(img, contract_markers(fw, fh));
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
        else if (k == "--jsonl") a.jsonl = next();
        else if (k == "--field-w") a.field_w = std::atof(next().c_str());
        else if (k == "--field-h") a.field_h = std::atof(next().c_str());
        else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
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

    FILE* jf = a.jsonl.empty() ? nullptr : std::fopen(a.jsonl.c_str(), "w");
    int n = 0, n_m4 = 0, n_m4q = 0, n_homo = 0, n_extent = 0;
    std::vector<double> errs, sx_m, sx_e, sy_m, sy_e;
    double sum_ms = 0;
    for (size_t fi = 0; fi < files.size(); ++fi) {
        int w = 0, h = 0;
        std::vector<uint8_t> rgb;
        std::string why;
        if (!read_ppm_p6(files[fi], &w, &h, &rgb, &why)) continue;
        const auto t0 = std::chrono::steady_clock::now();
        const auto m_basic = rectify::find_markers(rgb.data(), w, h);
        const auto m_quad = rectify::find_markers4(rgb.data(), w, h);
        const auto ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        sum_ms += ms;
        ++n;

        const Homography H = solve_from_markers(m_quad, a.field_w, a.field_h);
        const double re = reproj_error(H, m_quad);
        int fx0 = 0, fx1 = 0, fy0 = 0, fy1 = 0;
        bool bbox_ok = false;
        if (H.ok) bbox_ok = rectify::field_bbox(H, a.field_w, a.field_h, w, h,
                                                     &fx0, &fx1, &fy0, &fy1);
        // 单应给出的比例尺（场地单位/像素）：用 bbox 反推
        double sx_h = 0, sy_h = 0;
        if (bbox_ok && fx1 > fx0 && fy1 > fy0) {
            sx_h = a.field_w / (fx1 - fx0);
            sy_h = a.field_h / (fy1 - fy0);
        }
        const auto e = band_gap::field_extent(
            rgb.data(), w, h, a.field_w, a.field_h, 5.0, 3, 0.75, 0.0);

        if (m_basic.size() == 4) ++n_m4;
        if (m_quad.size() == 4) ++n_m4q;
        if (H.ok) ++n_homo;
        if (e.ok) ++n_extent;
        if (re >= 0) errs.push_back(re);
        if (sx_h > 0) { sx_m.push_back(sx_h); sy_m.push_back(sy_h); }
        if (e.ok && e.scale_x > 0) { sx_e.push_back(e.scale_x); sy_e.push_back(e.scale_y); }

        std::printf("帧 %3zu %dx%d 标记=%zu/%zu 单应=%d 重投影=%.2fpx 单应bbox=(%d..%d,%d..%d) "
                    "比例尺单应=(%.3f,%.3f) 亮度台阶=%d", fi, w, h, m_basic.size(),
                    m_quad.size(), H.ok ? 1 : 0, re, fx0, fx1, fy0, fy1, sx_h, sy_h,
                    e.ok ? 1 : 0);
        if (e.ok) std::printf(" 其比例尺=(%.3f,%.3f)", e.scale_x, e.scale_y);
        std::printf(" 用时=%.1fms\n", ms);
        if (jf) {
            std::fprintf(jf,
                         "{\"frame\":%zu,\"w\":%d,\"h\":%d,\"markers_basic\":%zu,"
                         "\"markers_quad\":%zu,\"homography_ok\":%s,\"reproj_px\":%.3f,"
                         "\"bbox\":[%d,%d,%d,%d],\"scale_homo\":[%.4f,%.4f],"
                         "\"extent_ok\":%s,\"scale_extent\":[%.4f,%.4f],\"ms\":%.2f}\n",
                         fi, w, h, m_basic.size(), m_quad.size(), H.ok ? "true" : "false", re,
                         fx0, fx1, fy0, fy1, sx_h, sy_h, e.ok ? "true" : "false",
                         e.scale_x, e.scale_y, ms);
        }
    }
    auto stat = [](const std::vector<double>& v, const char* name) {
        if (v.empty()) { std::printf("  %s: 无数据\n", name); return; }
        std::vector<double> s = v;
        std::sort(s.begin(), s.end());
        auto q = [&](double f) { return s[static_cast<size_t>(f * (s.size() - 1))]; };
        std::printf("  %s: 中位 %.3f  p90 %.3f  最小 %.3f  最大 %.3f（n=%zu）\n",
                    name, q(0.5), q(0.9), s.front(), s.back(), s.size());
    };
    std::printf("---- 汇总（%d 帧，平均 %.1f ms/帧）----\n", n, n > 0 ? sum_ms / n : 0.0);
    std::printf("  四标记齐全：find_markers %d/%d = %.1f%%；find_markers4 %d/%d = %.1f%%\n",
                n_m4, n, n ? 100.0 * n_m4 / n : 0.0, n_m4q, n, n ? 100.0 * n_m4q / n : 0.0);
    std::printf("  单应有效 %d/%d = %.1f%%；field_extent 有效 %d/%d = %.1f%%\n",
                n_homo, n, n ? 100.0 * n_homo / n : 0.0, n_extent, n,
                n ? 100.0 * n_extent / n : 0.0);
    stat(errs, "重投影误差(px)");
    stat(sx_m, "单应比例尺 x (单位/px)");
    stat(sx_e, "亮度台阶比例尺 x (单位/px)");
    if (jf) std::fclose(jf);
    return 0;
}
