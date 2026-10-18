// 轮廓分割的离线探针：喂一帧（或一整个目录）原始 RGB，看它切出什么、判成什么。
//
// 为什么先做探针而不是直接接进主循环：主循环一帧 80 ms，改一次要上传、上板、
// 起仿真、看日志，一轮好几分钟；而且"结果对不对"在实时链路上根本看不清。
// 离线探针能在**同一批真实帧**上把每种阈值的表现一次列全，几秒一轮。
//
// 用法：
//   contour_probe --ppm frame.ppm --dump-mask /tmp/m.pgm
//   contour_probe --ppm-dir /tmp/dump --track --limit 40
//
// 输出（每帧一行摘要 + 可选逐连通域）：
//   帧 12: 深=7 亮=1  墙=3 标记=2 角色=1  场地=(x0,y0)-(x1,y1) 尺度=(sx,sy) 用时=8.1ms
//   墙(120,240) 60x14 fill=0.94 L=31  v=(-210,0) 场地单位/秒
//
// ⚠ 编译（在仓库根目录）：
//   g++ -std=c++17 -O2 -I core/board -o /tmp/contour_probe core/board/tools/contour_probe.cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>

#include "band_gap.hpp"
#include "contour.hpp"
#include "ppm_reader.hpp"

using namespace vision_board;

namespace {

struct Args {
    std::string ppm_dir, ppm_file, dump_mask, dump_labels;
    int limit = 0;
    bool verbose = false;
    bool track = false;
    bool json = false;
    contour::Params cp;
    contour::ClassifyParams kp;
    double field_w = 640.0, field_h = 480.0;
    double squash = 0.75;
};

void usage() {
    std::printf(
        "用法: contour_probe [--ppm FILE | --ppm-dir DIR [--limit N]]\n"
        "         [--l-dark N] [--l-bright N] [--min-area N]\n"
        "         [--track] [--verbose] [--json]\n"
        "         [--dump-mask FILE.pgm] [--dump-labels FILE.pgm]\n"
        "         [--field-w 640] [--field-h 480] [--squash 0.75]\n");
}

bool read_ppm(const std::string& path, std::vector<uint8_t>* rgb, int* w, int* h) {
    std::string why;
    return read_ppm_p6(path, w, h, rgb, &why);
}

void write_pgm(const std::string& path, const uint8_t* pix, int w, int h) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fprintf(f, "P5\n%d %d\n255\n", w, h);
    std::fwrite(pix, 1, static_cast<size_t>(w) * h, f);
    std::fclose(f);
}

struct FrameResult {
    int n_dark = 0, n_bright = 0;
    int n_wall = 0, n_marker = 0, n_player = 0, n_line = 0;
    double ms = 0;
    bool extent_ok = false;
    band_gap::Extent ext;
};

}   // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--ppm") a.ppm_file = next();
        else if (k == "--ppm-dir") a.ppm_dir = next();
        else if (k == "--limit") a.limit = std::atoi(next().c_str());
        else if (k == "--l-dark") a.cp.l_dark = std::atoi(next().c_str());
        else if (k == "--l-bright") a.cp.l_bright = std::atoi(next().c_str());
        else if (k == "--min-area") a.cp.min_area = std::atoi(next().c_str());
        else if (k == "--track") a.track = true;
        else if (k == "--verbose") a.verbose = true;
        else if (k == "--json") a.json = true;
        else if (k == "--dump-mask") a.dump_mask = next();
        else if (k == "--dump-labels") a.dump_labels = next();
        else if (k == "--field-w") a.field_w = std::atof(next().c_str());
        else if (k == "--field-h") a.field_h = std::atof(next().c_str());
        else if (k == "--squash") a.squash = std::atof(next().c_str());
        else if (k == "-h" || k == "--help") { usage(); return 0; }
        else { std::fprintf(stderr, "未知参数: %s\n", k.c_str()); usage(); return 2; }
    }

    // ---- 收集帧 ----
    std::vector<std::string> files;
    if (!a.ppm_file.empty()) {
        files.push_back(a.ppm_file);
    } else if (!a.ppm_dir.empty()) {
        std::string cmd = "ls -1 '" + a.ppm_dir + "'/*.ppm 2>/dev/null";
        FILE* p = ::popen(cmd.c_str(), "r");
        if (!p) { std::fprintf(stderr, "列不出 %s\n", a.ppm_dir.c_str()); return 2; }
        char buf[4096];
        while (std::fgets(buf, sizeof buf, p)) {
            std::string s(buf);
            while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
            if (!s.empty()) files.push_back(s);
        }
        ::pclose(p);
        std::sort(files.begin(), files.end());
    } else {
        usage();
        return 2;
    }
    if (files.empty()) { std::fprintf(stderr, "没有可读的帧\n"); return 2; }
    if (a.limit > 0 && static_cast<int>(files.size()) > a.limit) files.resize(static_cast<size_t>(a.limit));

    contour::Tracker tracker;
    double prev_ms = 0;
    double sum_ms = 0;
    int n_frames = 0;
    int n_player_frames = 0, n_wall_frames = 0;

    std::vector<uint8_t> rgb;
    std::vector<contour::Comp> dark, bright;
    std::vector<uint8_t> mask;
    std::vector<int> labels;

    for (size_t fi = 0; fi < files.size(); ++fi) {
        int w = 0, h = 0;
        if (!read_ppm(files[fi], &rgb, &w, &h)) {
            std::fprintf(stderr, "读不了 %s\n", files[fi].c_str());
            continue;
        }
        // 时间戳：目录里的 PPM 没有时间戳，用 poppler 命名顺序 + 固定帧率估算 dt；
        // 单帧（--ppm）就没有 dt，速度一律打 0（别拿它当结论）。
        const double dt = (files.size() > 1) ? (1.0 / 12.0) : 0.0;

        const auto t0 = std::chrono::steady_clock::now();
        mask.clear();
        if (!a.dump_mask.empty()) mask.resize(static_cast<size_t>(w) * h);
        labels.clear();
        contour::Thresholds th;
        contour::components(rgb.data(), w, h, a.cp, &dark, &bright,
                            a.dump_mask.empty() ? nullptr : mask.data(),
                            a.dump_labels.empty() ? nullptr : &labels, &th);
        (void)labels;
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        sum_ms += ms; ++n_frames;

        band_gap::Extent ext = band_gap::field_extent(rgb.data(), w, h, a.field_w, a.field_h,
                                                      5.0, 3, a.squash, 0.0);
        contour::Scale scale;
        if (ext.ok) scale = contour::scale_from_bbox(ext.x0, ext.y0, ext.x1, ext.y1,
                                                     a.field_w, a.field_h);

        // ---- 分类 ----
        std::vector<contour::Comp> comps;
        std::vector<contour::Kind> kinds;
        for (const auto& c : dark) {
            comps.push_back(c);
            kinds.push_back(contour::classify(c, false, a.kp));
        }
        for (const auto& c : bright) {
            comps.push_back(c);
            kinds.push_back(contour::classify(c, true, a.kp));
        }
        int n_wall = 0, n_marker = 0, n_player = 0, n_line = 0;
        for (auto k : kinds) {
            switch (k) {
                case contour::Kind::Wall: ++n_wall; break;
                case contour::Kind::Marker: ++n_marker; break;
                case contour::Kind::Player: ++n_player; break;
                case contour::Kind::Line: ++n_line; break;
                default: break;
            }
        }
        if (n_player > 0) ++n_player_frames;
        if (n_wall > 0) ++n_wall_frames;

        std::vector<contour::Track> tracks;
        if (a.track) tracker.update(comps, kinds, dt, contour::TrackerParams{}, &tracks);

        if (a.json) {
            std::printf("{\"frame\":%zu,\"w\":%d,\"h\":%d,\"ms\":%.1f,\"dark\":%d,\"bright\":%d,"
                        "\"wall\":%d,\"marker\":%d,\"player\":%d,\"line\":%d,"
                        "\"extent_ok\":%s,\"boxes\":[",
                        fi, w, h, ms, static_cast<int>(dark.size()), static_cast<int>(bright.size()),
                        n_wall, n_marker, n_player, n_line, ext.ok ? "true" : "false");
            bool first = true;
            for (size_t i = 0; i < comps.size(); ++i) {
                if (kinds[i] == contour::Kind::Line || kinds[i] == contour::Kind::Unknown) continue;
                std::printf("%s{\"kind\":\"%s\",\"cx\":%.1f,\"cy\":%.1f,\"w\":%.0f,\"h\":%.0f,"
                            "\"area\":%d,\"fill\":%.2f,\"L\":%d}",
                            first ? "" : ",", contour::kind_name(kinds[i]),
                            comps[i].cx, comps[i].cy, comps[i].w, comps[i].h,
                            comps[i].area, comps[i].fill, comps[i].mean_l);
                first = false;
            }
            std::printf("]}\n");
        } else {
            std::printf("帧 %3zu: %dx%d 阈值(深≤%d 亮≥%d 主峰=%d) 深=%zu 亮=%zu | "
                        "墙=%d 标记=%d 角色=%d 线=%d | "
                        "场地=%s 尺度=(%.3f,%.3f)单位/px 用时=%.1fms\n",
                        fi, w, h, th.dark, th.bright, th.mode,
                        dark.size(), bright.size(), n_wall, n_marker, n_player, n_line,
                        ext.ok ? "ok" : "--",
                        ext.ok ? a.field_w / (ext.x1 - ext.x0) : 0.0,
                        ext.ok ? a.field_h / (ext.y1 - ext.y0) : 0.0, ms);
            if (a.verbose) {
                for (size_t i = 0; i < comps.size(); ++i) {
                    const auto& c = comps[i];
                    if (kinds[i] == contour::Kind::Line) continue;
                    std::printf("        %-4s (%6.1f,%6.1f) %3.0fx%-3.0f area=%-5d fill=%.2f "
                                "L=%-3d sx=%.1f sy=%.1f%s\n",
                                contour::kind_name(kinds[i]), c.cx, c.cy, c.w, c.h, c.area,
                                c.fill, c.mean_l, c.sx, c.sy,
                                c.inside_image(w, h) ? "" : "  [贴边]");
                }
                for (const auto& t : tracks) {
                    const double fvx = contour::to_field_dx(t.vx, scale);
                    const double fvy = contour::to_field_dy(t.vy, scale);
                    std::printf("        轨迹#%d %s (%6.1f,%6.1f) v=(%+.0f,%+.0f) 场地单位/秒\n",
                                t.id, contour::kind_name(t.kind), t.cx, t.cy, fvx, -fvy);
                }
            }
        }
        if (!a.dump_mask.empty()) write_pgm(a.dump_mask, mask.data(), w, h);
        if (!a.dump_labels.empty()) {
            std::vector<uint8_t> lab(static_cast<size_t>(w) * h, 0);
            for (size_t i = 0; i < lab.size(); ++i)
                lab[i] = static_cast<uint8_t>((labels[i] * 37) % 251 + 1);
            write_pgm(a.dump_labels, lab.data(), w, h);
        }
        prev_ms = ms;
        (void)prev_ms;
    }

    if (n_frames > 0) {
        std::fprintf(stderr,
                     "---- 汇总：%d 帧，平均 %.1f ms/帧（%.1f 帧/秒），有角色 %d 帧，有墙 %d 帧 ----\n",
                     n_frames, sum_ms / n_frames, 1000.0 / (sum_ms / n_frames),
                     n_player_frames, n_wall_frames);
    }
    return 0;
}
