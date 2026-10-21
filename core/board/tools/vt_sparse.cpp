// vt_sparse —— 稀疏采样版视觉前端（方案 A）
//
// 为什么重写：旧 `vt_board` 每帧对 640×480 每个像素做一次反投影（307k 次矩阵乘），
// 再做两次 31×31 形态学，单帧 ~0.7 s（板端只有 1.4 Hz）。所有"暂停→等帧→再走"
// 的策略都是为了迁就这个速度。
//
// 但这一场景的信息本质是**一维**的：
//   * 墙 = 水平亮条 → 只要一条"行剖面"就能定位（每行稀疏采 K 列取最大亮度）
//   * 缺口 = 该行上的最长暗段 → 只对"已判定为墙"的那几行做全宽扫描
//   * 角色 = 小而亮的圆点 → 行剖面里"只有一列亮"的行就是角色所在行
//
// 采样量：480 行 × K(24) 列 ≈ 11.5k，加 墙行×640（约 3 行）+ 角色行 640 ≈ 14k，
// 比 307k 少 20 倍；且**完全不做形态学**。目标：板端 20~60 Hz。
//
// 输出与 vt_board 完全同构的 jsonl（frame/quad_ext/walls[y,gl,gr]/player/n），
// 所以落盘、报告、agent 侧代码都不用改。
//
// 用法（与 vt_board 对齐）：
//   vt_sparse --w 320 --h 240 --frame-roi 0 0 320 240 \
//             --quad-file /tmp/quad.json --jsonl /tmp/vt.jsonl [--render out.rgb]
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

#include "field_frame.hpp"
#include "rectify.hpp"

namespace {

constexpr int FW = 640, FH = 480;

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct WallOut {
    double y = 0, gl = 0, gr = 0;
    bool gap = false;
};

//: 双线性采样（图像坐标可为小数）
inline void sample_bilinear(const uint8_t* img, int w, int h, double x, double y,
                            int* r, int* g, int* b) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > w - 1) x = w - 1;
    if (y > h - 1) y = h - 1;
    const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    const int x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
    const double fx = x - x0, fy = y - y0;
    const uint8_t* p00 = img + (static_cast<size_t>(y0) * w + x0) * 3;
    const uint8_t* p10 = img + (static_cast<size_t>(y0) * w + x1) * 3;
    const uint8_t* p01 = img + (static_cast<size_t>(y1) * w + x0) * 3;
    const uint8_t* p11 = img + (static_cast<size_t>(y1) * w + x1) * 3;
    const double w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy);
    const double w01 = (1 - fx) * fy, w11 = fx * fy;
    *r = static_cast<int>(w00 * p00[0] + w10 * p10[0] + w01 * p01[0] + w11 * p11[0]);
    *g = static_cast<int>(w00 * p00[1] + w10 * p10[1] + w01 * p01[1] + w11 * p11[1]);
    *b = static_cast<int>(w00 * p00[2] + w10 * p10[2] + w01 * p01[2] + w11 * p11[2]);
}

}  // namespace

//: 板端单调时钟（秒）。板端 board_agent 用 Python 的 time.monotonic()，在 Linux 上
//: 就是同一个 CLOCK_MONOTONIC —— 有了这个时间戳，agent 就能把"这一帧是什么时候
//: 拍的"和它收到的 @POSE 历史对上，**不用再猜**帧有多旧（跟随视口里角色恒在
//: 页面正中，靠角色位置是判断不出新鲜度的，这一层以前一直是在猜）。
static double mono_now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

int main(int argc, char** argv) {
    int w = 640, h = 360;
    int fr_x0 = 0, fr_y0 = 0, fr_x1 = 0, fr_y1 = 0;
    bool use_frame = false;
    std::string quad_file, jsonl, render_out;
    double wall_thr = 0.0;      // 0 = 自适应
    int cols = 24;
    bool player_free = false;   // true = 不要求角色在页面正中（算法单测用）
    bool dbg_player = false;

    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        const auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (k == "--w") w = std::atoi(next().c_str());
        else if (k == "--h") h = std::atoi(next().c_str());
        else if (k == "--frame-roi") {
            fr_x0 = std::atoi(next().c_str()); fr_y0 = std::atoi(next().c_str());
            fr_x1 = std::atoi(next().c_str()); fr_y1 = std::atoi(next().c_str());
            use_frame = true;
        } else if (k == "--quad-file") quad_file = next();
        //: 与 vt_board 的命令行**完全兼容**：切换实现时不用改 start_cam.sh 的参数。
        //: quad-state/save-rect/overlay/bands-jsonl/skip 收下但不使用（只读 quad）。
        else if (k == "--quad-state") (void)next();
        else if (k == "--save-rect") (void)next();
        else if (k == "--overlay") (void)next();
        else if (k == "--bands-jsonl") (void)next();
        else if (k == "--skip") (void)next();
        else if (k == "--no-gap-recompute") (void)0;
        else if (k == "--jsonl") jsonl = next();
        else if (k == "--render") render_out = next();
        else if (k == "--wall-thr") wall_thr = std::atof(next().c_str());
        else if (k == "--cols") cols = std::max(8, std::atoi(next().c_str()));
        else if (k == "--player-free") player_free = true;
        else if (k == "--dbg-player") dbg_player = true;
        else { std::fprintf(stderr, "unknown arg %s\n", k.c_str()); return 2; }
    }
    if (cols < 8) cols = 8;

    FILE* jf = jsonl.empty() ? nullptr : std::fopen(jsonl.c_str(), "w");
    const size_t need = static_cast<size_t>(w) * h * 3;
    std::vector<uint8_t> src(need);
    std::vector<uint8_t> crop;
    std::vector<uint8_t> render(static_cast<size_t>(FW) * FH * 3);

    long frame = 0;
    while (true) {
        size_t got = 0;
        while (got < need) {
            const ssize_t r = ::read(0, src.data() + got, need - got);
            if (r < 0) { std::perror("read"); return 1; }
            if (r == 0) return 0;                       // 流结束
            got += static_cast<size_t>(r);
        }
        ++frame;
        const double t_cap = mono_now();   // 这一帧"读进来"的时刻（给 agent 配对位姿）

        // ---- ROI 裁剪 ---------------------------------------------------
        const uint8_t* base = src.data();
        int bw = w, bh = h;
        if (use_frame) {
            const int x0 = std::max(0, std::min(fr_x0, w - 1));
            const int y0 = std::max(0, std::min(fr_y0, h - 1));
            const int x1 = std::max(x0 + 8, std::min(fr_x1, w));
            const int y1 = std::max(y0 + 8, std::min(fr_y1, h));
            bw = x1 - x0; bh = y1 - y0;
            crop.resize(static_cast<size_t>(bw) * bh * 3);
            for (int yy = 0; yy < bh; ++yy)
                std::memcpy(&crop[static_cast<size_t>(yy) * bw * 3],
                            &src[(static_cast<size_t>(y0 + yy) * w + x0) * 3],
                            static_cast<size_t>(bw) * 3);
            base = crop.data();
        }

        // ---- 四边形 → 单应 ----------------------------------------------
        bool quad_ext = false;
        vision_board::field_frame::Quad fq;
        if (!quad_file.empty()) {
            if (FILE* xf = std::fopen(quad_file.c_str(), "r")) {
                double v[8] = {0};
                if (std::fscanf(xf, "%lf %lf %lf %lf %lf %lf %lf %lf",
                                &v[0], &v[1], &v[2], &v[3],
                                &v[4], &v[5], &v[6], &v[7]) == 8) {
                    bool sane = true;
                    double area = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        const double px = v[2 * i], py = v[2 * i + 1];
                        if (px < -8 || py < -8 || px > bw + 8 || py > bh + 8) sane = false;
                        const int j = (i + 1) % 4;
                        area += px * v[2 * j + 1] - v[2 * j] * py;
                    }
                    if (std::fabs(area) * 0.5 < 0.10 * bw * bh) sane = false;
                    if (sane) {
                        for (int i = 0; i < 4; ++i) {
                            fq.p[i][0] = v[2 * i];
                            fq.p[i][1] = v[2 * i + 1];
                        }
                        fq.ok = true;
                        quad_ext = true;
                    }
                }
                std::fclose(xf);
            }
        }
        if (!fq.ok) fq = vision_board::field_frame::find(base, bw, bh);
        const auto H = vision_board::field_frame::homography(fq, FW, FH);
        const auto inv = rectify::invert(H);
        if (!fq.ok || !H.ok || !inv.ok) {
            if (jf) {
                std::fprintf(jf, "{\"frame\":%ld,\"t\":%.6f,\"frame_geom\":0,"
                                 "\"quad_ext\":%d,\"walls\":[],\"player\":null,\"n\":0}\n",
                             frame, t_cap, quad_ext ? 1 : 0);
                std::fflush(jf);
            }
            continue;
        }

        // ---- 行剖面：每行稀疏采 cols 列 ---------------------------------
        const double y_page[FH] = {};
        (void)y_page;
        std::vector<int> row_bright(FH, 0), row_maxl(FH, 0);
        std::vector<int> col_x(cols);
        for (int i = 0; i < cols; ++i)
            col_x[i] = static_cast<int>(std::lround(10.0 + i * (FW - 20.0) / (cols - 1)));
        long p90_hist[256] = {0};
        std::vector<int> lat(static_cast<size_t>(FH) * cols, 0);
        for (int r = 0; r < FH; ++r) {
            const double py_page = FH - (r + 0.5);        // 页面上该行的世界 y（y 向上）
            for (int i = 0; i < cols; ++i) {
                const auto q = inv.apply(col_x[i] + 0.5, py_page);
                int rr, gg, bb;
                sample_bilinear(base, bw, bh, q[0], q[1], &rr, &gg, &bb);
                const int L = luma(rr, gg, bb);
                lat[static_cast<size_t>(r) * cols + i] = L;
                if (L > row_maxl[r]) row_maxl[r] = L;
                ++p90_hist[std::min(255, L)];
            }
        }
        //: 自适应阈值：取所有采样点亮度的 90 分位，墙/背景之间通常差很多
        long total = static_cast<long>(FH) * cols, acc = 0;
        int thr = 140;
        for (int v = 0; v < 256; ++v) {
            acc += p90_hist[v];
            if (acc >= total * 88 / 100) { thr = std::max(90, std::min(200, v - 20)); break; }
        }
        if (wall_thr > 0) thr = static_cast<int>(wall_thr);

        //: 每行"亮度超出阈值多少"的累加，作为墙行质心的权重。
        //: 只用硬阈值数亮列（row_bright）时，带子的上下沿会随阈值抖动 2~8 行；
        //: 用连续权重做质心，行的定位就变成亚像素、且对阈值不敏感。
        std::vector<double> row_ev(FH, 0.0);
        for (int r = 0; r < FH; ++r) {
            int n = 0;
            double ev = 0.0;
            for (int i = 0; i < cols; ++i) {
                const int L = lat[static_cast<size_t>(r) * cols + i];
                if (L >= thr) { ++n; ev += static_cast<double>(L - thr); }
            }
            row_bright[r] = n;
            row_ev[r] = ev;
        }

        // ---- 墙行：亮列占比高；成带后取带中心 ---------------------------
        const int band_need = std::max(3, static_cast<int>(cols * 0.55));
        std::vector<std::array<int, 2>> bands;
        for (int r = 0; r < FH; ++r) {
            if (row_bright[r] < band_need) continue;
            if (!bands.empty() && r - bands.back()[1] <= 2) bands.back()[1] = r;
            else bands.push_back({r, r});
        }

        // ---- 每道墙：全宽扫该行，找最长暗段当缺口 ----------------------
        std::vector<WallOut> walls;
        std::vector<double> no_gap_y;   //: 有墙但量不出缺口的行（下游据此"不许前进"）
        for (const auto& b : bands) {
            if (b[1] - b[0] + 1 < 4) continue;            // 单像素/白框线多半是网格线
            //: **太高的"亮行带"不是墙，是屏幕反光/灯斑**（2026-10-21 实测）：
            //: 页面上的墙是 6 px 厚，经相机辉光后在 640×480 里是 19~26 行高；
            //: 而屏幕上的反光带是 60+ 行高的大块亮区。不丢它的话，反光带会被
            //: 当成一道墙，而且它在**页面坐标里是固定的**（反光贴在屏幕上），
            //: 于是随平台前进不断被记成"正前方 50~150 单位处的新墙"：
            //: r4/r5 的 842/861/876、r10 的 3356.9（真墙 3300，偏 +57，把终点
            //: 挡在墙后 → 规划器永远回 0）都是这么来的。墙上限取 34 行，
            //: 真墙 26 行留了余量，反光 60+ 行必丢。
            if (b[1] - b[0] + 1 > 34) continue;
            //: 亚像素质心：带子 [b0-2, b1+2] 内按 row_ev 加权（连续、抗阈值抖动）
            double wsum = 0.0, wrow = 0.0;
            for (int r = std::max(0, b[0] - 2); r <= std::min(FH - 1, b[1] + 2); ++r) {
                const double e = row_ev[static_cast<size_t>(r)];
                if (e <= 0.0) continue;
                wsum += e;
                wrow += e * static_cast<double>(r);
            }
            const double rc_frac = (wsum > 1e-9)
                    ? (wrow / wsum)
                    : 0.5 * (b[0] + b[1]);
            const int rc = static_cast<int>(std::lround(rc_frac));
            if (rc < 10 || rc > FH - 11) continue;        // 页面白框本身不是墙
            const double py_page = FH - (rc_frac + 0.5);
            std::vector<int> row(static_cast<size_t>(FW), 0);
            for (int x = 0; x < FW; ++x) {
                const auto q = inv.apply(x + 0.5, py_page);
                int rr, gg, bb;
                sample_bilinear(base, bw, bh, q[0], q[1], &rr, &gg, &bb);
                row[static_cast<size_t>(x)] = luma(rr, gg, bb);
            }
            int best_len = 0, best_s = -1;
            int s = -1;
            for (int x = 6; x < FW - 6; ++x) {
                const bool dark = row[static_cast<size_t>(x)] < thr;
                if (dark && s < 0) s = x;
                if ((!dark) && s >= 0) {
                    if (x - s > best_len) { best_len = x - s; best_s = s; }
                    s = -1;
                }
            }
            if (s >= 0 && (FW - 6) - s > best_len) { best_len = (FW - 6) - s; best_s = s; }
            //: **缺口两侧必须真的是亮墙**：只有"暗段左右各有 ≥8px 的亮像素"
            //: 才算缺口。否则暗段只是"墙在那一块没被拍到/太暗"（灯光暗角、
            //: 白框内边缘、被角色挡住），把这种暗段当缺口会把角色引到墙里
            //: （实测：y=220 那道墙配到 gl≈29 的假缺口，min_clearance=-10）。
            bool shoulder = false;
            if (best_len > 0) {
                int lb = 0, rb = 0;
                for (int x = std::max(2, best_s - 8); x < best_s; ++x)
                    if (row[static_cast<size_t>(x)] >= thr) ++lb;
                for (int x = best_s + best_len + 1;
                     x < std::min(FW - 1, best_s + best_len + 9); ++x)
                    if (row[static_cast<size_t>(x)] >= thr) ++rb;
                shoulder = (lb >= 6 && rb >= 6);
            }
            WallOut o;
            o.y = py_page;
            //: 缺口宽度必须落在场景真值范围内（90~110，正射后按页面 px 略有伸缩）；
            //: 整行全黑（best_len≈620）= 这一行根本没墙，不能当缺口报出去。
            if (shoulder && best_len >= 45 && best_len <= 200 && best_s > 4
                    && best_s + best_len < FW - 4) {
                o.gap = true;
                o.gl = best_s;
                o.gr = best_s + best_len;
            }
            walls.push_back(o);
            if (!o.gap) no_gap_y.push_back(o.y);
        }

        // ---- 角色：圆盘-环带匹配滤波（不依赖绝对亮度）--------------------
        //: 以前的判据是"绝对亮度 ≥ max(Otsu,165)"。曝光/环境光一变（这台相机的
        //: AE 会收敛到"场地中位数 150+"的状态），整个场地都过阈值，白点被并进
        //: 一大块连通域里 → 角色恒为 null（实测 r2：30/40 帧有墙、有角色 0 帧，
        //: 闸门直接拒跑）。白点的**结构**特征跟亮度无关：一个 ~20px 的圆盘，
        //: 四周一圈是暗场地。所以用"圆盘均值 − 环带均值"的匹配滤波来量：
        //:   * 墙是长条，端点虽然也有响应，但只有白点的 1/2（实测 36 vs 68）；
        //:   * 角色恒在页面正中（跟随视口），搜索窗只需要 ±96 px。
        //: 判据是**对比度**（disc−ring > 25 灰度），不是绝对亮度 —— 面板亮、
        //: 场地被 AE 抬到 150 时照样能量出来。
        bool pfound = false;
        double px = 0, py = 0;
        {
            constexpr double R_D = 9.0;        // 白点半径（页面 px）
            constexpr double R0 = 15.0, R1 = 22.0;   // 环带 = 场地
            constexpr double MIN_RESP = 25.0;  // 对比度门槛（灰度）
            struct Off { double x, y; };
            std::vector<Off> disc, ring;
            for (int dy = -22; dy <= 22; dy += 2) {
                for (int dx = -22; dx <= 22; dx += 2) {
                    const double r = std::sqrt(static_cast<double>(dx * dx + dy * dy));
                    if (r <= R_D) disc.push_back({static_cast<double>(dx), static_cast<double>(dy)});
                    else if (r >= R0 && r <= R1) ring.push_back({static_cast<double>(dx), static_cast<double>(dy)});
                }
            }
            auto resp = [&](double cx, double cy) -> double {
                double sd = 0.0, sr = 0.0;
                for (const auto& o : disc) {
                    int r_, g_, b_;
                    sample_bilinear(base, bw, bh, cx + o.x, cy + o.y, &r_, &g_, &b_);
                    sd += luma(r_, g_, b_);
                }
                for (const auto& o : ring) {
                    int r_, g_, b_;
                    sample_bilinear(base, bw, bh, cx + o.x, cy + o.y, &r_, &g_, &b_);
                    sr += luma(r_, g_, b_);
                }
                return sd / static_cast<double>(disc.size())
                     - sr / static_cast<double>(ring.size());
            };
            const auto c_img = inv.apply(FW * 0.5, FH * 0.5);
            const double WIN = 96.0;
            double best = -1e9, bx = c_img[0], by = c_img[1];
            //: 粗扫 4 px，再在最优附近细扫 1 px
            for (double oy = -WIN; oy <= WIN; oy += 4.0) {
                for (double ox = -WIN; ox <= WIN; ox += 4.0) {
                    const double v = resp(c_img[0] + ox, c_img[1] + oy);
                    if (v > best) { best = v; bx = c_img[0] + ox; by = c_img[1] + oy; }
                }
            }
            const double fx = bx, fy = by;
            for (double oy = -4.0; oy <= 4.0; oy += 1.0) {
                for (double ox = -4.0; ox <= 4.0; ox += 1.0) {
                    const double v = resp(fx + ox, fy + oy);
                    if (v > best) { best = v; bx = fx + ox; by = fy + oy; }
                }
            }
            //: 形状复核：墙的**端点**也是一个亮团，端点响应实测 36（白点 68）。
            //: 白点 ~20px 见方；墙段只有 6~10 px 高、几十~上百 px 长。所以量一下
            //: 亮团的宽高（阈值取圆盘与环带均值的中点），宽高都得像"点"才认。
            bool shape_ok = false;
            if (best >= MIN_RESP) {
                int r_, g_, b_;
                double sd = 0.0, sr = 0.0;
                for (const auto& o : disc) {
                    sample_bilinear(base, bw, bh, bx + o.x, by + o.y, &r_, &g_, &b_);
                    sd += luma(r_, g_, b_);
                }
                for (const auto& o : ring) {
                    sample_bilinear(base, bw, bh, bx + o.x, by + o.y, &r_, &g_, &b_);
                    sr += luma(r_, g_, b_);
                }
                const double dm = sd / static_cast<double>(disc.size());
                const double rm = sr / static_cast<double>(ring.size());
                const double lvl = 0.5 * (dm + rm);
                auto bright = [&](double x, double y) -> bool {
                    int a_, c_, d_;
                    sample_bilinear(base, bw, bh, x, y, &a_, &c_, &d_);
                    return luma(a_, c_, d_) >= lvl;
                };
                int wl = 0, wr = 0, wu = 0, wd = 0;
                while (wl < 30 && bright(bx - wl - 1, by)) ++wl;
                while (wr < 30 && bright(bx + wr + 1, by)) ++wr;
                while (wu < 30 && bright(bx, by - wu - 1)) ++wu;
                while (wd < 30 && bright(bx, by + wd + 1)) ++wd;
                const int wpx = wl + wr + 1, hpx = wu + wd + 1;
                const double ar = static_cast<double>(wpx) / static_cast<double>(hpx);
                shape_ok = (wpx >= 12 && wpx <= 44 && hpx >= 12 && hpx <= 44
                            && ar >= 0.5 && ar <= 2.0);
                if (dbg_player)
                    std::fprintf(stderr, "player_shape w=%d h=%d lvl=%.0f\n", wpx, hpx, lvl);
            }
            if (shape_ok) {
                const auto pg = H.apply(bx, by);   // 图像坐标 → 页面坐标
                pfound = true;
                px = pg[0];
                py = pg[1];
            }
            if (dbg_player)
                std::fprintf(stderr, "player_resp=%.1f at img(%.0f,%.0f)\n",
                             best, bx, by);
        }


        // ---- 输出 -------------------------------------------------------
        if (jf) {
            //: **只报量到了缺口的墙**。没有缺口的行（缺口跑到画面外、被角色挡住、
            //: 或压根不是墙）以前会被写成 gl=0/gr=640，下游一律按"贴边"丢掉，
            //: 反而在报告里制造大量"缺口贴边帧"。宁可少报一行，也不要报假缺口。
            int emitted = 0;
            std::fprintf(jf, "{\"frame\":%ld,\"t\":%.6f,\"frame_geom\":1,"
                             "\"quad_ext\":%d,\"walls\":[",
                         frame, t_cap, quad_ext ? 1 : 0);
            for (const auto& o : walls) {
                if (!o.gap) continue;
                std::fprintf(jf, "%s{\"y\":%.1f,\"gl\":%.1f,\"gr\":%.1f}",
                             emitted ? "," : "", o.y, o.gl, o.gr);
                ++emitted;
            }
            std::fprintf(jf, "],\"no_gap_y\":[");
            for (size_t i = 0; i < no_gap_y.size(); ++i)
                std::fprintf(jf, "%s%.1f", i ? "," : "", no_gap_y[i]);
            std::fprintf(jf, "],\"player\":");
            if (pfound) std::fprintf(jf, "[%.1f,%.1f]", px, py);
            else std::fprintf(jf, "null");
            std::fprintf(jf, ",\"n\":%d,\"no_gap\":%zu,\"fast\":1}\n",
                         emitted, no_gap_y.size());
            std::fflush(jf);
        }
        if (!render_out.empty() || true) {
            //: 干净渲染（给录像用）：深色底 + 白墙 + 缺口留空 + 白点
            std::fill(render.begin(), render.end(), 0);
            for (size_t i = 0; i < render.size(); i += 3) {
                render[i] = 20; render[i + 1] = 24; render[i + 2] = 34;
            }
            auto put = [&](int x, int y, int r, int g, int b) {
                if (x < 0 || y < 0 || x >= FW || y >= FH) return;
                uint8_t* p = &render[(static_cast<size_t>(y) * FW + x) * 3];
                p[0] = static_cast<uint8_t>(r);
                p[1] = static_cast<uint8_t>(g);
                p[2] = static_cast<uint8_t>(b);
            };
            for (const auto& o : walls) {
                const int r = static_cast<int>(std::lround(FH - o.y));
                for (int dy = -3; dy <= 3; ++dy) {
                    for (int x = 0; x < FW; ++x) {
                        if (o.gap && x >= o.gl - 1 && x <= o.gr + 1) continue;
                        put(x, r + dy, 240, 230, 200);
                    }
                }
            }
            if (pfound) {
                const int cx = static_cast<int>(std::lround(px));
                const int cy = static_cast<int>(std::lround(FH - py));
                for (int dy = -9; dy <= 9; ++dy)
                    for (int dx = -9; dx <= 9; ++dx)
                        if (dx * dx + dy * dy <= 81) put(cx + dx, cy + dy, 255, 255, 255);
            }
            if (!render_out.empty()) {
                FILE* rf = std::fopen(render_out.c_str(), "wb");
                if (rf) {
                    std::fprintf(rf, "P6\n%d %d\n255\n", FW, FH);
                    std::fwrite(render.data(), 1, render.size(), rf);
                    std::fclose(rf);
                }
            } else {
                std::fwrite(render.data(), 1, render.size(), stdout);
                std::fflush(stdout);
            }
        }
    }
    if (jf) std::fclose(jf);
    return 0;
}
