// 角色检测的离线评估器（P2 退出条件：precision / recall / 位置误差）。
//
// 关键设计：**"期望的像素位置"不来自角色检测器**，而是
//     仿真真值（场地单位） --单应逆变换--> 图像像素
// 单应来自四角标记（与角色无关），所以这条链路是独立的。真值另外打印出来，
// 便于人工核对（不变式 I5：真值不能来自被测对象）。
//
// 用法：
//   g++ -std=c++17 -O2 -I core/board -o /tmp/pp core/board/tools/player_probe.cpp core/board/ppm_reader.cpp
//   /tmp/pp --ppm-dir DIR --truth 320,105.6 [--tol 8] [--jsonl out.jsonl]
//   `/tmp/pp --ppm-dir DIR --truth-jsonl sim.jsonl`（动态：按墙的 y 对齐后逐帧真值）
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include "band_gap.hpp"
#include "geometry.hpp"
#include "marker_refine.hpp"
#include "player.hpp"
#include "wall_gap.hpp"
#include "ppm_reader.hpp"
#include "rectify.hpp"

using vision_board::read_ppm_p6;

namespace {

struct Args {
    std::string dir, ppm, truth, truth_jsonl, jsonl;
    int limit = 0;
    double tol_px = 10.0;
    double field_w = 640.0, field_h = 480.0;
    double player_radius = 10.0;
    bool quiet = false;
    vision_board::player::Params pp;
    bool no_band_filter = false;
};

//: 从"已经按几何排序"的标记解单应（与主循环同一套约定）。
rectify::Homography solve_ordered(const std::vector<rectify::Marker>& mk,
                                  double fw, double fh) {
    rectify::Homography bad;
    if (mk.size() != 4) return bad;
    const std::array<std::array<double, 2>, 4> img = {{
        {mk[0].x, mk[0].y}, {mk[1].x, mk[1].y}, {mk[2].x, mk[2].y}, {mk[3].x, mk[3].y}}};
    return rectify::solve(img, rectify::contract_markers(fw, fh));
}

}   // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--ppm") a.ppm = next();
        else if (k == "--ppm-dir") a.dir = next();
        else if (k == "--truth") a.truth = next();
        else if (k == "--truth-jsonl") a.truth_jsonl = next();
        else if (k == "--jsonl") a.jsonl = next();
        else if (k == "--limit") a.limit = std::atoi(next().c_str());
        else if (k == "--tol") a.tol_px = std::atof(next().c_str());
        else if (k == "--player-radius") a.player_radius = std::atof(next().c_str());
        else if (k == "--quiet") a.quiet = true;
        else if (k == "--max-run-k") a.pp.max_run_k = std::atof(next().c_str());
        else if (k == "--no-band-filter") a.no_band_filter = true;
        else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
    }
    // 真值（场地单位）：静态场景下是不变的
    double tx = 0, ty = 0;
    bool have_truth = false;
    if (!a.truth.empty()) {
        if (std::sscanf(a.truth.c_str(), "%lf,%lf", &tx, &ty) == 2) have_truth = true;
    }

    //: 逐帧真值：`{"frame":N,"x":...,"y":...}`（场地单位，y 朝上）。
    //: 由 `build_offline_corpus.sh` 用"帧序号→墙钟→仿真真值"生成。
    struct TruthPt { double x, y; bool ok; };
    std::vector<TruthPt> truths;
    if (!a.truth_jsonl.empty()) {
        FILE* f = std::fopen(a.truth_jsonl.c_str(), "r");
        if (!f) { std::fprintf(stderr, "打不开 %s\n", a.truth_jsonl.c_str()); return 2; }
        char line[512];
        while (std::fgets(line, sizeof line, f)) {
            int idx = -1; double tx = 0, ty = 0;
            if (std::sscanf(line, "{\"frame\":%d,\"x\":%lf,\"y\":%lf}", &idx, &tx, &ty) == 3) {
                if (static_cast<size_t>(idx) >= truths.size()) truths.resize(static_cast<size_t>(idx) + 1);
                truths[static_cast<size_t>(idx)] = TruthPt{tx, ty, true};
            }
        }
        std::fclose(f);
        std::printf("逐帧真值: %zu 条\n", truths.size());
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

    int n = 0, n_geo = 0;
    int old_hit = 0, old_wrong = 0;    // 旧检测器
    int new_hit = 0, new_miss = 0, new_false = 0;
    std::vector<double> old_err, new_err;
    std::vector<int> miss_reasons;

    for (size_t fi = 0; fi < files.size(); ++fi) {
        int w = 0, h = 0;
        std::vector<uint8_t> rgb;
        std::string why;
        if (!read_ppm_p6(files[fi], &w, &h, &rgb, &why)) continue;
        ++n;
        if (!truths.empty() && fi < truths.size() && truths[fi].ok) {
            tx = truths[fi].x; ty = truths[fi].y; have_truth = true;
        }

        // ---- 几何：标记（几何排序）→ 单应 → 可信性判定 ----
        std::vector<rectify::Marker> mk = rectify::find_markers4(rgb.data(), w, h);
        if (mk.size() == 4) {
            std::sort(mk.begin(), mk.end(), [](const rectify::Marker& p, const rectify::Marker& q) {
                return (p.x + p.y) < (q.x + q.y);
            });
            rectify::Marker tl = mk[0], br = mk[3];
            rectify::Marker o1 = mk[1], o2 = mk[2];
            rectify::Marker tr = (o1.x >= o2.x) ? o1 : o2;
            rectify::Marker bl = (o1.x >= o2.x) ? o2 : o1;
            // 世界口径（y 朝上）：contract 的 (32,32) 是**世界左下**，对应图像左下标记。
            // 所以喂给 solve 的顺序是 [BL, BR, TR, TL]。
            mk = {bl, br, tr, tl};
        }
        // 标记中心精修（方块 bbox 中心），再做单应
        int n_ref = 0;
        // 契约：标记方块边长 48 场地单位，场地宽 640 → 边长像素 ≈ 跨度 × 48/640
        double span_px = 0;
        if (mk.size() == 4) {
            const double top = std::hypot(mk[1].x - mk[0].x, mk[1].y - mk[0].y);
            const double bot = std::hypot(mk[2].x - mk[3].x, mk[2].y - mk[3].y);
            span_px = 0.5 * (top + bot);
        }
        vision_board::markers::Params rp;
        if (span_px > 40.0) {
            const double expect = span_px * 48.0 / 640.0;
            rp.min_side_px = static_cast<int>(0.5 * expect);
            rp.max_side_px = static_cast<int>(2.0 * expect);
        }
        for (auto& m : mk) {
            bool ok = false;
            const rectify::Marker r = vision_board::markers::refine(rgb.data(), w, h, m, &ok, rp);
            if (ok) { m = r; ++n_ref; }
        }
        const rectify::Homography H = solve_ordered(mk, a.field_w, a.field_h);
        const auto geo = vision_board::geometry::judge(H, mk, w, h, a.field_w, a.field_h);
        const bool geo_ok = geo.valid;
        (void)n_ref;
        if (geo_ok) ++n_geo;

        // ---- 期望像素位置：真值（场地）→ 单应逆变换 ----
        double ex = 0, ey = 0;
        bool have_expect = false;
        if (geo_ok && have_truth) {
            const rectify::Homography inv = rectify::invert(H);
            if (inv.ok) {
                // ⚠ **坐标约定**：仿真的真值/`@STATE` 里 y 轴朝上（+y = 画面里往上），
                //   而 `rectify::contract_markers()` 定义的场地坐标 y 朝下（TL 是 (32,32)）。
                //   两者差一个 `y -> field_h - y` 的翻转。不翻的话，"期望位置"会跑到
                //   画面另一头（实测偏 173 px），看起来像"角色检测偏了"，其实是量尺错了。
                const auto q = inv.apply(tx, ty);   // 现在场地坐标已是 y 朝上，与真值同口径
                ex = q[0]; ey = q[1];
                have_expect = true;
            }
        }

        // ---- 旧检测器 vs 新检测器 ----
        const auto oldp = band_gap::find_player(rgb.data(), w, h);
        std::vector<std::pair<int, int>> bands;
        {
            const auto wl = vision_board::wall_gap::find_walls(rgb.data(), w, h);
            for (const auto& w2 : wl) bands.emplace_back(w2.y0, w2.y1);
        }
        auto newp = vision_board::player::find(rgb.data(), w, h, H, mk,
                                               a.field_w, a.field_h, a.player_radius, a.pp,
                                               a.no_band_filter ? nullptr : &bands);
        // 全局失败 → 用上一帧位置做局部精修（角色贴墙时会被并进墙的大块里）
        static vision_board::player::Player prev;
        if (!newp.found && prev.found && geo_ok) {
            const double unit_per_px = a.field_w / std::max(1.0, static_cast<double>(
                geo.bbox[1] - geo.bbox[0]));
            const double r_px = a.player_radius / std::max(1e-6, unit_per_px);
            const auto loc = vision_board::player::refine_near(rgb.data(), w, h, prev.x, prev.y,
                                                              r_px, 45.0, 12.0);
            bool near_marker = false;
            for (const auto& m : mk) {
                if (std::hypot(loc.x - m.x, loc.y - m.y) < 2.0 * std::max(m.w, m.h)) {
                    near_marker = true;
                    break;
                }
            }
            if (loc.found && !near_marker) {
                newp.found = true;
                newp.x = loc.x; newp.y = loc.y;
                newp.why = "local";
                newp.conf = loc.contrast / 100.0;
            }
        }
        if (newp.found) prev = newp;
        double old_e = -1, new_e = -1;
        if (have_expect) {
            if (oldp.found) {
                old_e = std::hypot(oldp.x - ex, oldp.y - ey);
                if (old_e <= a.tol_px) ++old_hit; else ++old_wrong;
                old_err.push_back(old_e);
            }
            if (newp.found) {
                new_e = std::hypot(newp.x - ex, newp.y - ey);
                if (new_e <= a.tol_px) ++new_hit; else ++new_false;
                new_err.push_back(new_e);
            } else {
                ++new_miss;
                miss_reasons.push_back(0);
            }
        }
        if (!a.quiet) {
            std::printf("帧 %3zu 几何=%s 期望=(%.1f,%.1f) | 旧:%s(%.1f,%.1f) e=%.1f | "
                        "新:%s(%.1f,%.1f) e=%.1f conf=%.2f %s\n",
                        fi, geo_ok ? "ok" : geo.why, ex, ey,
                        oldp.found ? "有" : "无", oldp.x, oldp.y, old_e,
                        newp.found ? "有" : "无", newp.x, newp.y, new_e, newp.conf,
                        newp.found ? "" : newp.why);
        }
        if (jf) {
            std::fprintf(jf,
                         "{\"frame\":%zu,\"geo_ok\":%s,\"geo_why\":\"%s\","
                         "\"expect_px\":[%.1f,%.1f],\"old_found\":%s,\"old_px\":[%.1f,%.1f],"
                         "\"old_err\":%.1f,\"new_found\":%s,\"new_px\":[%.1f,%.1f],"
                         "\"new_err\":%.1f,\"new_conf\":%.2f,\"new_why\":\"%s\"}\n",
                         fi, geo_ok ? "true" : "false", geo.why, ex, ey,
                         oldp.found ? "true" : "false", oldp.x, oldp.y, old_e,
                         newp.found ? "true" : "false", newp.x, newp.y, new_e, newp.conf,
                         newp.why);
        }
    }

    auto stat = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        if (v.empty()) return std::string("无数据");
        auto q = [&](double f) { return v[static_cast<size_t>(f * (v.size() - 1))]; };
        char buf[160];
        std::snprintf(buf, sizeof buf, "中位 %.1f px  p90 %.1f  最大 %.1f（n=%zu）",
                      q(0.5), q(0.9), v.back(), v.size());
        return std::string(buf);
    };
    std::printf("---- 汇总：%d 帧，几何有效 %d（%.1f%%）----\n", n, n_geo,
                n ? 100.0 * n_geo / n : 0.0);
    const int denom = n_geo > 0 ? n_geo : 1;
    std::printf("  旧 find_player ：命中 %d/%d = %.1f%%，错位 %d\n",
                old_hit, denom, 100.0 * old_hit / denom, old_wrong);
    std::printf("  新 player::find：命中 %d/%d = %.1f%%，漏检 %d，错位 %d\n",
                new_hit, denom, 100.0 * new_hit / denom, new_miss, new_false);
    std::printf("  旧 位置误差 %s\n", stat(old_err).c_str());
    std::printf("  新 位置误差 %s\n", stat(new_err).c_str());
    if (jf) std::fclose(jf);
    (void)miss_reasons;
    return 0;
}
