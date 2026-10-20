// 从**相机视觉**直接构建虚拟真值（VT）并渲染成干净视频。
//
// 输入：stdin 原始 RGB（640×360，cam_live 的流水线帧）。
// 输出：stdout 原始 RGB（640×480，VT 的场地：墙/缺口/角色/目标）。
//       同时 --jsonl 写每帧的 VT 数值（墙的场地 y/缺口、角色场地坐标）。
//
// 关键点：
//   * **所有墙**都从 `wall_gap::find_walls` 取（不是只取选中的那一道）；
//   * 坐标用四角标记的单应（透视不变）；标记不足时退回 `field_extent` 的仿射；
//   * 渲染的是"视觉转换出来的 VT"，不是相机原图、也不是仿真真值。
//
// 编译（板上）：
//   g++ -std=c++17 -O2 -I /root/board_src/core/board -o /root/tools/vt_from_vision \
//       /root/tools/vt_from_vision.cpp
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>

#include "band_gap.hpp"
#include "contour.hpp"
#include "field_frame.hpp"
#include "frame_geom.hpp"
#include "marker_refine.hpp"
#include "player.hpp"
#include "rectify.hpp"
#include "wall_gap.hpp"

namespace {

constexpr double FW = 640.0, FH = 480.0;

//: 去白框、并把同一道墙被相机高光/阴影切成两三条的行带合并回一条。
struct WallOut {
    double y = 0.0;
    double gl = 0.0, gr = FW;
    bool gap = false;
};

void put(std::vector<uint8_t>* im, int x, int y, int r, int g, int b) {
    if (x < 0 || y < 0 || x >= 640 || y >= 480) return;
    uint8_t* p = im->data() + (static_cast<size_t>(y) * 640 + x) * 3;
    p[0] = static_cast<uint8_t>(r); p[1] = static_cast<uint8_t>(g); p[2] = static_cast<uint8_t>(b);
}

void rect(std::vector<uint8_t>* im, double fx0, double fy0, double fx1, double fy1,
          int r, int g, int b) {
    int x0 = static_cast<int>(std::lround(std::min(fx0, fx1)));
    int x1 = static_cast<int>(std::lround(std::max(fx0, fx1)));
    int sy0 = static_cast<int>(std::lround(FH - std::max(fy0, fy1)));
    int sy1 = static_cast<int>(std::lround(FH - std::min(fy0, fy1)));
    for (int y = sy0; y <= sy1; ++y)
        for (int x = x0; x <= x1; ++x) put(im, x, y, r, g, b);
}

void line(std::vector<uint8_t>* im, int x0, int y0, int x1, int y1,
          int r, int g, int b) {
    const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put(im, x0, y0, r, g, b); put(im, x0 + 1, y0, r, g, b); put(im, x0, y0 + 1, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void disc(std::vector<uint8_t>* im, double fx, double fy, double rad,
          int r, int g, int b, bool ring = false) {
    const int cx = static_cast<int>(std::lround(fx)), cy = static_cast<int>(std::lround(FH - fy));
    const int R = static_cast<int>(std::lround(rad));
    for (int y = -R; y <= R; ++y)
        for (int x = -R; x <= R; ++x) {
            const double d = std::hypot(x, y);
            if (ring ? (d > R - 2.0 && d <= R) : (d <= R)) put(im, cx + x, cy + y, r, g, b);
        }
}

bool marker_homography(const uint8_t* rgb, int w, int h, rectify::Homography* out) {
    auto mk = rectify::find_markers4(rgb, w, h);
    if (mk.size() != 4) return false;
    std::sort(mk.begin(), mk.end(), [](const rectify::Marker& p, const rectify::Marker& q) {
        return (p.x + p.y) < (q.x + q.y);
    });
    rectify::Marker tl = mk[0], br = mk[3];
    rectify::Marker o1 = mk[1], o2 = mk[2];
    rectify::Marker tr = (o1.x >= o2.x) ? o1 : o2;
    rectify::Marker bl = (o1.x >= o2.x) ? o2 : o1;
    //: ⚠ **必须精修**：find_markers4 给的是"亮内图"质心，不是方块中心；
    //: 不精修时四点的偏差不一致，单应会把 rectified 场地拧成斜的/转的
    //: （这就是这张样例 rectify 后墙还是斜的的原因）。
    const double span = 0.5 * (std::hypot(mk[1].x - mk[0].x, mk[1].y - mk[0].y) +
                               std::hypot(mk[2].x - mk[3].x, mk[2].y - mk[3].y));
    vision_board::markers::Params rp;
    if (span > 40.0) {
        const double expect = span * 28.0 / 640.0;
        rp.min_side_px = static_cast<int>(0.5 * expect);
        rp.max_side_px = static_cast<int>(2.0 * expect);
    }
    bool ok = false;
    tl = vision_board::markers::refine(rgb, w, h, tl, &ok, rp);
    tr = vision_board::markers::refine(rgb, w, h, tr, &ok, rp);
    br = vision_board::markers::refine(rgb, w, h, br, &ok, rp);
    bl = vision_board::markers::refine(rgb, w, h, bl, &ok, rp);
    const std::array<std::array<double, 2>, 4> img = {
        {{bl.x, bl.y}, {br.x, br.y}, {tr.x, tr.y}, {tl.x, tl.y}}};
    *out = rectify::solve(img, rectify::contract_markers(FW, FH));
    return out->ok;
}

bool extent_homography(const band_gap::Extent& e, rectify::Homography* out) {
    if (!e.ok) return false;
    const std::array<std::array<double, 2>, 4> img = {{
        {{static_cast<double>(e.x0), static_cast<double>(e.y1)}},
        {{static_cast<double>(e.x1), static_cast<double>(e.y1)}},
        {{static_cast<double>(e.x1), static_cast<double>(e.y0)}},
        {{static_cast<double>(e.x0), static_cast<double>(e.y0)}},
    }};
    const std::array<std::array<double, 2>, 4> world = {{
        {{0.0, 0.0}}, {{FW, 0.0}}, {{FW, FH}}, {{0.0, FH}},
    }};
    *out = rectify::solve(img, world);
    return out->ok;
}

}  // namespace

int main(int argc, char** argv) {
    int w = 640, h = 360, skip = 6;
    long frames = 0;
    double squash = 1.0;
    std::string jsonl, save_rect, overlay_out, quad_state;
    bool use_frame = false;
    int fr_x0 = 0, fr_y0 = 0, fr_x1 = 0, fr_y1 = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--w") w = std::atoi(next().c_str());
        else if (k == "--h") h = std::atoi(next().c_str());
        else if (k == "--skip") skip = std::atoi(next().c_str());
        else if (k == "--frames") frames = std::atol(next().c_str());
        else if (k == "--squash") squash = std::atof(next().c_str());
        else if (k == "--jsonl") jsonl = next();
        else if (k == "--quad-state") quad_state = next();
        else if (k == "--save-rect") save_rect = next();
        else if (k == "--overlay-out") overlay_out = next();
        else if (k == "--frame") use_frame = true;
        else if (k == "--frame-roi") {
            fr_x0 = std::atoi(next().c_str()); fr_y0 = std::atoi(next().c_str());
            fr_x1 = std::atoi(next().c_str()); fr_y1 = std::atoi(next().c_str());
            use_frame = true;
        }
        else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
    }
    FILE* jf = nullptr;
    if (!jsonl.empty()) { jf = std::fopen(jsonl.c_str(), "w"); }
    FILE* of = nullptr;
    if (!overlay_out.empty()) { of = std::fopen(overlay_out.c_str(), "wb"); }
    const size_t need = static_cast<size_t>(w) * h * 3;
    std::vector<uint8_t> src(need);
    std::vector<uint8_t> out(640 * 480 * 3);
    long frame = 0;
    while (true) {
        size_t got = 0;
        while (got < need) {
            const ssize_t r = ::read(0, src.data() + got, need - got);
            if (r < 0) { std::perror("read"); return 1; }
            if (r == 0) return 0;
            got += static_cast<size_t>(r);
        }
        ++frame;
        if (frame <= skip) continue;

        //: 白框路径：只认场地白框（`frame_geom`），不依赖四角标记。
        //: 相机固定在窗口上时，窗口外沿/标题栏会干扰白框扫描，所以先裁一个
        //: **固定 ROI**（把桌面/标题栏/VS Code 挡在外面），再在 ROI 里找白框。
        std::vector<uint8_t> crop;
        const uint8_t* base = src.data();
        int bw = w, bh = h;
        bool frame_ok = false;
        rectify::Homography Hf;
        vision_board::field_frame::Quad fq_used;
        int roi_x = 0, roi_y = 0;
        if (use_frame) {
            int x0 = std::max(0, std::min(fr_x0, w - 1));
            int y0 = std::max(0, std::min(fr_y0, h - 1));
            int x1 = std::max(x0 + 8, std::min(fr_x1, w));
            int y1 = std::max(y0 + 8, std::min(fr_y1, h));
            bw = x1 - x0; bh = y1 - y0;
            roi_x = x0; roi_y = y0;
            crop.resize(static_cast<size_t>(bw) * bh * 3);
            for (int yy = 0; yy < bh; ++yy)
                std::memcpy(&crop[static_cast<size_t>(yy) * bw * 3],
                            &src[(static_cast<size_t>(y0 + yy) * w + x0) * 3],
                            static_cast<size_t>(bw) * 3);
            base = crop.data();
            //: 首选：场地暗区 + 最小外接旋转矩形 → 白框**四角**透视单应。
            //: 这一步把 bbox 仿射换成四角透视；rectified 后墙才是水平的。
            auto fq = vision_board::field_frame::find(base, bw, bh);
            if (fq.ok && !quad_state.empty()) {
                //: 四角**时间平滑**：一帧一个子进程，所以把上一帧的四角存在文件里。
                //: 相机/屏幕固定时四角只该有 1~2 px 抖动；不平滑时墙线会左右跳，
                //: 视频里看就是"墙在乱飘"。新解与上一帧差 >12 px 视为换了场景，直接采用。
                std::array<double, 8> prev{};
                bool have = false;
                if (FILE* qf = std::fopen(quad_state.c_str(), "r")) {
                    if (std::fscanf(qf, "%lf %lf %lf %lf %lf %lf %lf %lf",
                                    &prev[0], &prev[1], &prev[2], &prev[3],
                                    &prev[4], &prev[5], &prev[6], &prev[7]) == 8)
                        have = true;
                    std::fclose(qf);
                }
                if (have) {
                    double dmax = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        const double dx = fq.p[i][0] - prev[2 * i];
                        const double dy = fq.p[i][1] - prev[2 * i + 1];
                        dmax = std::max(dmax, std::hypot(dx, dy));
                    }
                    if (dmax <= 12.0) {
                        constexpr double a = 0.6;      // 上一帧权重
                        for (int i = 0; i < 4; ++i) {
                            fq.p[i][0] = a * prev[2 * i] + (1.0 - a) * fq.p[i][0];
                            fq.p[i][1] = a * prev[2 * i + 1] + (1.0 - a) * fq.p[i][1];
                        }
                    }
                }
                if (FILE* qf = std::fopen(quad_state.c_str(), "w")) {
                    std::fprintf(qf, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                                 fq.p[0][0], fq.p[0][1], fq.p[1][0], fq.p[1][1],
                                 fq.p[2][0], fq.p[2][1], fq.p[3][0], fq.p[3][1]);
                    std::fclose(qf);
                }
                Hf = vision_board::field_frame::homography(fq, FW, FH);
                frame_ok = Hf.ok;
                fq_used = fq;
            }
            if (!frame_ok) {
                //: 退化：仍用旧的 bbox 仿射（至少不崩）。
                vision_board::frame_geom::Params fp;
                fp.inside_darker_ratio = 0.0;
                const auto fr = vision_board::frame_geom::find(base, bw, bh, fp);
                if (fr.ok) {
                    Hf = vision_board::frame_geom::homography(fr, FW, FH);
                    frame_ok = Hf.ok;
                }
            }
        }

        const auto ext = band_gap::field_extent(base, bw, bh, FW, FH, 5.0, 3, squash);
        rectify::Homography H = Hf;
        const bool markers = !frame_ok && marker_homography(base, bw, bh, &H);

        //: **有标记就先 rectify 成 640×480 俯视图再检测**。
        //: 这样墙是水平的、场地是正矩形、亮背景被挡在场地外 —— 同时解决
        //: "相机歪"和"墙边界往亮背景延伸"两个问题。
        std::vector<uint8_t> rect_img;
        const uint8_t* det = src.data();
        int det_w = w, det_h = h;
        bool rectified = false;
        rectify::Homography idh;
        idh.ok = true;
        if (frame_ok || markers) {
            const rectify::Homography inv = rectify::invert(H);
            if (inv.ok) {
                rect_img.assign(640u * 480u * 3u, 0);
                for (int yy = 0; yy < 480; ++yy) {
                    const double fy = 480.0 - (yy + 0.5);   // 俯视图第 0 行 = 场地顶
                    for (int xx = 0; xx < 640; ++xx) {
                        const auto q = inv.apply(xx + 0.5, fy);
                        const int u = static_cast<int>(std::lround(q[0]));
                        const int v = static_cast<int>(std::lround(q[1]));
                        if (u < 0 || v < 0 || u >= bw || v >= bh) continue;
                        std::memcpy(&rect_img[(static_cast<size_t>(yy) * 640 + xx) * 3],
                                    &base[(static_cast<size_t>(v) * bw + u) * 3], 3);
                    }
                }
                det = rect_img.data(); det_w = 640; det_h = 480; rectified = true;
                if (!save_rect.empty()) {
                    FILE* rf = std::fopen(save_rect.c_str(), "wb");
                    if (rf) {
                        std::fprintf(rf, "P6\n640 480\n255\n");
                        std::fwrite(rect_img.data(), 1, rect_img.size(), rf);
                        std::fclose(rf);
                    }
                }
            }
        }

        vision_board::wall_gap::Params wp;
        wp.keep_no_gap = true;       // 多道墙：没量到缺口的行带也保留
        wp.all_bands = true;         // 所有行带都输出
        wp.min_band_rows = 3;        // 斜拍时顶部墙只有 3~4 px 厚
        wp.min_bright_frac = 0.0;
        if (rectified) {
            //: 俯视图最外圈就是白框本身，不能让 find_walls 把它当墙。
            wp.clip_x0 = 4; wp.clip_x1 = 636; wp.clip_y0 = 4; wp.clip_y1 = 476;
        } else if (ext.ok) {
            wp.clip_x0 = ext.x0 + 4; wp.clip_x1 = ext.x1 - 4;
            wp.clip_y0 = ext.y0 + 4; wp.clip_y1 = ext.y1 - 4;
        }
        //: 光照归一：white top-hat（31×31）把低频的屏亮度梯度扣掉，
        //: 墙/缺口只保留"局部亮"这一项。没有这一步时，左侧/下侧墙偏暗，
        //: find_walls 会在 dim 的那一侧判成"无墙"，缺口位置就跟着错。
        std::vector<uint8_t> hat;
        const uint8_t* wall_src = det;
        if (rectified) {
            const int n = det_w * det_h;
            std::vector<uint8_t> gray(static_cast<size_t>(n), 0), er(static_cast<size_t>(n), 0),
                di(static_cast<size_t>(n), 0);
            for (int i = 0; i < n; ++i) {
                const uint8_t* p = det + static_cast<size_t>(i) * 3;
                gray[static_cast<size_t>(i)] = static_cast<uint8_t>(
                    (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
            }
            // opening = erode -> dilate
            vision_board::field_frame::detail::morph_rect(gray.data(), er.data(), det_w, det_h, 31, 31, false);
            vision_board::field_frame::detail::morph_rect(er.data(), di.data(), det_w, det_h, 31, 31, true);
            hat.assign(static_cast<size_t>(n) * 3, 0);
            for (int i = 0; i < n; ++i) {
                int v = static_cast<int>(gray[static_cast<size_t>(i)]) - static_cast<int>(di[static_cast<size_t>(i)]);
                v = std::min(255, std::max(0, v));
                hat[static_cast<size_t>(i) * 3 + 0] = static_cast<uint8_t>(v);
                hat[static_cast<size_t>(i) * 3 + 1] = static_cast<uint8_t>(v);
                hat[static_cast<size_t>(i) * 3 + 2] = static_cast<uint8_t>(v);
            }
            wall_src = hat.data();
        }
        auto walls = vision_board::wall_gap::find_walls(wall_src, det_w, det_h, wp);

        //: `find_walls` 自带的缺口逻辑是按"整行亮段"判的；在白墙+光照不均时
        //: 会把 dim 的一侧算成墙、把真实缺口算成整墙。这里在 top-hat 上按**墙带
        //: 列亮**重算一次：一条墙带里，某列在带内（上下各放宽几行）完全不亮，
        //: 就认为这一列是缺口；取最长的连续缺口段。
        //:
        //: ⚠ 必须"放宽几行"：白框单应是仿射的（最小外接矩形四角），相机有俯仰时
        //:   rectified 里墙还残留 5~10 px 的斜率；只按 `find_walls` 给的窄行带数，
        //:   墙在 dim 的那一端会掉出行带 → 整段被判成缺口（实测底墙缺口端点每帧
        //:   在 x=8 和 x=79 之间跳）。改成"列在 ±4 行内取最大 luma"后与斜率无关。
        if (rectified && !hat.empty()) {
            for (auto& wl : walls) {
                const int y0 = std::max(0, wl.y0), y1 = std::min(det_h - 1, wl.y1);
                if (y1 < y0) continue;
                const int ry0 = std::max(0, y0 - 4);
                const int ry1 = std::min(det_h - 1, y1 + 4);
                std::vector<int> mx(static_cast<size_t>(det_w), 0);
                for (int y = ry0; y <= ry1; ++y) {
                    for (int x = 0; x < det_w; ++x) {
                        const uint8_t* p = hat.data() + (static_cast<size_t>(y) * det_w + x) * 3;
                        const int l = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
                        if (l > mx[static_cast<size_t>(x)]) mx[static_cast<size_t>(x)] = l;
                    }
                }
                int best_len = 0, best_s = 0, best_e = 0;
                int s = -1;
                for (int x = 8; x < det_w - 8; ++x) {
                    const bool off = mx[static_cast<size_t>(x)] < 30;
                    if (off && s < 0) s = x;
                    if (!off && s >= 0) {
                        if (x - s > best_len) { best_len = x - s; best_s = s; best_e = x - 1; }
                        s = -1;
                    }
                }
                if (s >= 0 && (det_w - 8) - s > best_len) {
                    best_len = (det_w - 8) - s; best_s = s; best_e = det_w - 9;
                }
                if (best_len >= 10) {
                    wl.has_gap = true;
                    wl.gap_x0 = best_s;
                    wl.gap_x1 = best_e;
                }
            }
        }

        vision_board::player::Player pl;
        {
            std::vector<std::pair<int, int>> bands;
            std::vector<std::array<int, 4>> gapw;
            for (const auto& wl : walls) {
                bands.emplace_back(wl.y0, wl.y1);
                if (wl.has_gap) gapw.push_back(std::array<int, 4>{wl.y0, wl.y1, wl.gap_x0, wl.gap_x1});
            }
            if (rectified) {
                //: 俯视图里四角标记的位置由契约给出（32/608, 32/448），不依赖
                //: 检测；把标记作为排除项传给 player，否则它会把角落标记的亮块
                //: 当成角色（实测抓到右下标记）。
                std::vector<rectify::Marker> ex;
                for (const auto& c : rectify::contract_markers(FW, FH)) {
                    rectify::Marker m;
                    m.x = c[0]; m.y = c[1]; m.w = 24; m.h = 24;
                    ex.push_back(m);
                }
                vision_board::player::Params pp;
                //: 出生点离第一道墙只有 20 场地单位；白框单应还有 ~10px 的 y 误差，
                //: 默认 12px 的墙带余量会把**真角色**当墙端头排掉。缩到 6px。
                pp.wall_band_margin = 6;
                if (const char* lb = std::getenv("VT_LBRIGHT")) pp.l_bright = std::atoi(lb);
                if (const char* cf = std::getenv("VT_CELLFRAC")) pp.cell_frac = std::atof(cf);
                if (const char* mc = std::getenv("VT_MINCELLS")) pp.min_cells = std::atoi(mc);
                pl = vision_board::player::find(det, det_w, det_h, idh, ex, FW, FH, 10.0,
                                                pp, &bands, &gapw);
            } else if (markers) {
                pl = vision_board::player::find(base, bw, bh, H, {}, FW, FH, 10.0,
                                                vision_board::player::Params{}, &bands, &gapw);
            }

            //: 目标环中心也是一个**白点**：`player::find` 在亮屏录制里偶尔会把
            //: 它当成角色（实测第 43 帧抓到的正是 (311,466.7)，紧贴目标环）。
            //: 目标环在场地坐标里固定在 (320,470)，离它太近就判否，交给下面的
            //: 白点兜底（那里已排除目标环附近）。
            if (pl.found && (rectified || markers)) {
                double gx = pl.x, gy = pl.y;
                if (rectified) {
                    gy = FH - pl.y;            // rectified 图 y 向下 → 场地 y 朝上
                } else {
                    const auto q = H.apply(pl.x, pl.y);
                    gx = q[0]; gy = q[1];
                }
                if (std::hypot(gx - 320.0, gy - 470.0) < 40.0) {
                    pl.found = false;
                    pl.why = "near_goal";
                }
            }

            //: 兜底：`player::find` 在"角色贴底墙 + 远离四角"时经常 no_blob。
            //: 场地里只有一个**实心白圆**（目标环是绿的、墙是长条、框在边上），
            //: 所以直接在 rectified 图上找小的、近圆的、孤立的白色连通块；排除
            //: 墙带、目标环、四角标记位置即可。实测这张图白点在 (320,420)。
            if (!pl.found && rectified) {
                const int n = det_w * det_h;
                std::vector<uint8_t> wm(static_cast<size_t>(n), 0);
                for (int i = 0; i < n; ++i) {
                    const uint8_t* p = det + static_cast<size_t>(i) * 3;
                    const int l = (77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8;
                    const int mx = std::max(p[0], std::max(p[1], p[2]));
                    const int mn = std::min(p[0], std::min(p[1], p[2]));
                    const int g = p[1], r = p[0];
                    if (l >= 175 && (mx - mn) < 45 && g < r + 40) wm[static_cast<size_t>(i)] = 1;
                }
                std::vector<int> lab(static_cast<size_t>(n), 0);
                std::vector<int> st;
                int next = 0;
                double best_conf = -1.0;
                for (int y = 0; y < det_h; ++y) {
                    for (int x = 0; x < det_w; ++x) {
                        const size_t idx = static_cast<size_t>(y) * det_w + x;
                        if (!wm[idx] || lab[idx]) continue;
                        ++next;
                        st.clear(); st.push_back(static_cast<int>(idx)); lab[idx] = next;
                        int area = 0, x0 = x, x1 = x, y0 = y, y1 = y;
                        long sx = 0, sy = 0;
                        while (!st.empty()) {
                            const int cur = st.back(); st.pop_back();
                            const int cy = cur / det_w, cx = cur % det_w;
                            ++area; sx += cx; sy += cy;
                            x0 = std::min(x0, cx); x1 = std::max(x1, cx);
                            y0 = std::min(y0, cy); y1 = std::max(y1, cy);
                            const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
                            for (int k = 0; k < 4; ++k) {
                                const int nx = cx + dx[k], ny = cy + dy[k];
                                if (nx < 0 || ny < 0 || nx >= det_w || ny >= det_h) continue;
                                const size_t ni = static_cast<size_t>(ny) * det_w + nx;
                                if (wm[ni] && !lab[ni]) { lab[ni] = next; st.push_back(static_cast<int>(ni)); }
                            }
                        }
                        const int bw = x1 - x0 + 1, bh = y1 - y0 + 1;
                        if (area < 60 || area > 1600) continue;
                        if (bw < 6 || bh < 6 || bw > 60 || bh > 60) continue;
                        const double aspect = static_cast<double>(std::max(bw, bh)) / std::max(1, std::min(bw, bh));
                        if (aspect > 1.7) continue;
                        const double fill = static_cast<double>(area) / (bw * bh);
                        if (fill < 0.40) continue;
                        const double cx = sx / static_cast<double>(area);
                        const double cy = sy / static_cast<double>(area);
                        // 排除目标环（顶部中心附近）和四角标记契约位置
                        if (std::hypot(cx - 320.0, cy - 10.0) < 35.0) continue;
                        bool near_marker = false;
                        for (const auto& c : rectify::contract_markers(FW, FH)) {
                            if (std::hypot(cx - c[0], cy - c[1]) < 45.0) { near_marker = true; break; }
                        }
                        if (near_marker) continue;
                        // 排除墙带（含 margin）
                        bool on_wall = false;
                        for (const auto& b : bands) {
                            if (cy >= b.first - 6 && cy <= b.second + 6) { on_wall = true; break; }
                        }
                        if (on_wall) continue;
                        const double conf = fill / 0.79 / std::max(1.0, aspect - 1.0 + 1.0);
                        if (conf > best_conf) {
                            best_conf = conf;
                            pl.found = true; pl.x = cx; pl.y = cy;
                            pl.w = bw; pl.h = bh; pl.area = area;
                            pl.fill = fill; pl.conf = conf; pl.why = "white_dot";
                        }
                    }
                }
            }
        }

        //: 后处理：白框本身/同一道墙的多条行带。
        //: 实测 rectified 图里 find_walls 会把顶部白框当成一条 y≈471 的墙，并把
        //: y≈320 的墙拆成 308/322 两条。这里剔掉贴近 0/480 的框线，再把中心相差
        //: <16 的行带合并（保留有缺口的那条）。
        std::vector<WallOut> show;
        if (rectified || markers) {
            for (const auto& wl : walls) {
                double fy = 0, gl = 0, gr = FW;
                bool gap = wl.has_gap;
                if (rectified) {
                    fy = FH - wl.cy;
                    if (gap) { gl = wl.gap_x0; gr = wl.gap_x1; }
                } else {
                    const auto c = H.apply(0.5 * (wl.x0 + wl.x1), wl.cy);
                    fy = c[1];
                    if (gap) {
                        gl = H.apply(static_cast<double>(wl.gap_x0), wl.cy)[0];
                        gr = H.apply(static_cast<double>(wl.gap_x1), wl.cy)[0];
                    }
                }
                if (fy < 14.0 || fy > 466.0) continue;      // 框线不是墙
                if (gl < 0) gl = 0; if (gr > FW) gr = FW;
                if (gap && gr - gl < 1.0) { gl = 0; gr = FW; gap = false; }
                WallOut o{fy, gl, gr, gap};
                bool merged = false;
                for (auto& e : show) {
                    if (std::fabs(e.y - fy) >= 16.0) continue;
                    e.y = 0.5 * (e.y + fy);
                    if (!e.gap && gap) { e.gap = true; e.gl = gl; e.gr = gr; }
                    merged = true;
                    break;
                }
                if (!merged) show.push_back(o);
            }
        }

        //: 相机视角叠加：把白框四角、检测到的墙/缺口、角色、目标画回**原始相机帧**。
        //: 这是给"平台视角处理"视频用的；VT 本身仍是 stdout 的干净渲染。
        if (of != nullptr) {
            std::vector<uint8_t> ov(src.begin(), src.end());
            const rectify::Homography inv = rectify::invert(Hf);
            const bool has_h = frame_ok && inv.ok;
            auto f2c = [&](double fx, double fy) -> std::pair<int, int> {
                if (!has_h) return {0, 0};
                const auto q = inv.apply(fx, fy);
                return {static_cast<int>(std::lround(q[0])) + roi_x,
                        static_cast<int>(std::lround(q[1])) + roi_y};
            };
            if (fq_used.ok) {
                for (int i = 0; i < 4; ++i) {
                    const int j = (i + 1) % 4;
                    line(&ov, static_cast<int>(std::lround(fq_used.p[i][0])) + roi_x,
                         static_cast<int>(std::lround(fq_used.p[i][1])) + roi_y,
                         static_cast<int>(std::lround(fq_used.p[j][0])) + roi_x,
                         static_cast<int>(std::lround(fq_used.p[j][1])) + roi_y,
                         0, 255, 0);
                }
            }
            if (has_h) {
                for (const auto& wl : show) {
                    // 墙两段
                    if (wl.gl > 1.0) {
                        auto a = f2c(0.0, wl.y), b = f2c(wl.gl, wl.y);
                        line(&ov, a.first, a.second, b.first, b.second, 255, 140, 0);
                    }
                    if (wl.gr < FW - 1.0) {
                        auto a = f2c(wl.gr, wl.y), b = f2c(FW, wl.y);
                        line(&ov, a.first, a.second, b.first, b.second, 255, 140, 0);
                    }
                    if (wl.gap) {
                        auto a = f2c(wl.gl, wl.y), b = f2c(wl.gr, wl.y);
                        line(&ov, a.first, a.second, b.first, b.second, 0, 230, 255);
                    }
                }
                // 目标环（固定 320,470）
                auto g = f2c(320.0, 470.0);
                for (int dy = -26; dy <= 26; ++dy)
                    for (int dx = -26; dx <= 26; ++dx) {
                        const int d2 = dx * dx + dy * dy;
                        if (d2 >= 576 && d2 <= 676) put(&ov, g.first + dx, g.second + dy, 60, 230, 120);
                    }
            }
            std::fwrite(ov.data(), 1, ov.size(), of);
            std::fflush(of);
        }

        std::fill(out.begin(), out.end(), 0);
        for (size_t i = 0; i < out.size(); i += 3) {
            out[i] = 34; out[i + 1] = 40; out[i + 2] = 55;
        }
        for (int y = 0; y < 480; ++y)
            for (int x = 0; x < 640; ++x)
                if (x < 2 || x >= 638 || y < 2 || y >= 478) put(&out, x, y, 255, 255, 255);
        if (jf) {
            std::fprintf(jf, "{\"frame\":%ld,\"markers\":%d,\"frame_geom\":%d,\"extent\":%d,\"walls\":[",
                         frame, markers ? 1 : 0, frame_ok ? 1 : 0, ext.ok ? 1 : 0);
        }
        for (size_t i = 0; i < show.size(); ++i) {
            const auto& wl = show[i];
            const double fy = wl.y, gl = wl.gl, gr = wl.gr;
            // 墙：两段（0..gl 与 gr..FW），厚 8
            if (gl > 1.0) rect(&out, 0.0, fy - 4.0, gl, fy + 4.0, 255, 120, 0);
            if (FW - gr > 1.0) rect(&out, gr, fy - 4.0, FW, fy + 4.0, 255, 120, 0);
            if (jf) {
                std::fprintf(jf, "%s{\"y\":%.1f,\"gl\":%.1f,\"gr\":%.1f}",
                             i ? "," : "", fy, gl, gr);
            }
        }
        if (jf) std::fprintf(jf, "],\"player\":");
        double px = 0, py = 0; bool pok = false;
        if (pl.found && (rectified || markers)) {
            if (rectified) { px = pl.x; py = FH - pl.y; }
            else { const auto q = H.apply(pl.x, pl.y); px = q[0]; py = q[1]; }
            pok = true;
        }
        if (pok) disc(&out, px, py, 10.0, 255, 255, 255);
        disc(&out, 320.0, 470.0, 24.0, 60, 230, 120, true);
        if (jf) {
            if (pok) std::fprintf(jf, "[%.1f,%.1f],\"n\":%zu,\"player_why\":\"%s\"}\n", px, py, show.size(), pl.why);
            else std::fprintf(jf, "null,\"n\":%zu,\"player_why\":\"%s\"}\n", show.size(), pl.why);
            std::fflush(jf);
        }
        std::fwrite(out.data(), 1, out.size(), stdout);
        std::fflush(stdout);
        if (frames > 0 && frame - skip >= frames) break;
    }
    if (jf) std::fclose(jf);
    if (of) std::fclose(of);
    return 0;
}
