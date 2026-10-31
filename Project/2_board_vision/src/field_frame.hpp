// 白框几何（无 marker 路径）——**四角透视单应**。
//
// 为什么不用 `frame_geom::homography` 的 bbox：
//   bbox 是轴对齐的，相机一旦有轻微旋转/俯仰，bbox 四角就不是白框四角；
//   仿射单应会把场地拧斜，缺口行带随之错位（实测 4 道墙只剩 3 道、缺口全丢）。
//
// 这里换一种更稳的取法：
//   1. 场地是"中间亮度"，白框/墙是亮的 → 用 luma ∈ (lo, hi) 取"场地内部"；
//   2. 竖/横两个方向的闭运算把墙、角色、目标环挖出的洞填上（墙是横线、框是外沿）；
//   3. 取最大连通域，求**最小外接旋转矩形**（rotating calipers 的暴力角度版）；
//   4. 四个角点就是白框四角，解 4 点透视单应。
//
// 亮度带的两套口径（暗屏 / 亮屏）：
//   * 默认带 (25,145) 是按 `vision_sample6` 这种**暗屏场地**定的（背景黑、场地
//     25~70、白框/墙 ~200）；这是基准图，行为必须保持；
//   * 亮屏场地（平台页面本身是灰蓝：背景桌面 40~90、场地 100~180、白框/墙 ~250）
//     下，默认带只圈住桌面 → 最大连通域是整幅图、四角贴边、单应完全错。此时
//     `find` 用直方图三簇自动定带再跑一遍（见 `auto_band`），并用 `plausible`
//     拒掉"面积 > 90% / 四角全贴边"的解。
//
// 这比 bbox 多出来的就是**旋转 + 透视**；墙会真正水平，缺口不会因为行带错位被抹掉。
//
// 单位：像素。输出四角顺序固定 TL/TR/BR/BL（图像坐标）。
#ifndef VISION_BOARD_FIELD_FRAME_HPP
#define VISION_BOARD_FIELD_FRAME_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "rectify.hpp"

namespace vision_board {
namespace field_frame {

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: 场地内部的 luma 区间。下界挡桌面/黑边，上界挡白框/墙/角色/目标。
    int lo = 25;
    int hi = 145;
    //: 闭运算核（竖长条）把横墙、角色、目标环的洞填上。
    int close_w = 5;
    int close_h = 21;
    //: 开运算去掉孤立噪点。
    int open_w = 5;
    int open_h = 5;
    //: 最小面积占比（相对图像）。太小说明没锁到场。
    double min_area_frac = 0.15;
    //: 第一遍（固定 lo/hi）给出的四角**贴满画面**时，用直方图自动分带再试一遍。
    //: 只为兼容"亮屏场地"（场地比背景亮）的录制；暗屏场地（sample6）第一遍就过，
    //: 不会进这个分支，行为不变。
    bool auto_retry = true;
};

struct Quad {
    bool ok = false;
    std::array<std::array<double, 2>, 4> p{};   // TL, TR, BR, BL（图像）
    double cx = 0.0, cy = 0.0;                  // 最小外接矩形中心
    double w = 0.0, h = 0.0, angle = 0.0;       // 最小外接矩形（w>=h）
    int why = 0;                                // 0=ok 1=no_mask 2=too_small 3=no_quad
    int lo = 0, hi = 0;                         // 实际用的 luma 带（诊断）
    int retried = 0;                            // 1=固定带贴满画面，已用自动带重试
    const char* why_str() const {
        switch (why) {
            case 0: return "ok";
            case 1: return "no_mask";
            case 2: return "too_small";
            case 3: return "no_quad";
            default: return "unknown";
        }
    }
};

namespace detail {

//: 可分离矩形 max/min 滤波；dilate=true 取 max，false 取 min。
inline void morph_sep(const uint8_t* src, uint8_t* dst, int w, int h,
                      int kw, int kh, bool dilate) {
    std::vector<uint8_t> tmp(static_cast<size_t>(w) * h, 0);
    const int rx = std::max(1, kw / 2), ry = std::max(1, kh / 2);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = src + static_cast<size_t>(y) * w;
        uint8_t* out = &tmp[static_cast<size_t>(y) * w];
        for (int x = 0; x < w; ++x) {
            uint8_t v = dilate ? 0 : 255;
            for (int dx = -rx; dx <= rx; ++dx) {
                const int xx = std::min(w - 1, std::max(0, x + dx));
                const uint8_t s = row[xx];
                v = dilate ? std::max(v, s) : std::min(v, s);
            }
            out[x] = v;
        }
    }
    for (int y = 0; y < h; ++y) {
        uint8_t* out = dst + static_cast<size_t>(y) * w;
        for (int x = 0; x < w; ++x) {
            uint8_t v = dilate ? 0 : 255;
            for (int dy = -ry; dy <= ry; ++dy) {
                const int yy = std::min(h - 1, std::max(0, y + dy));
                const uint8_t s = tmp[static_cast<size_t>(yy) * w + x];
                v = dilate ? std::max(v, s) : std::min(v, s);
            }
            out[x] = v;
        }
    }
}

inline void morph_rect(const uint8_t* src, uint8_t* dst, int w, int h,
                       int kw, int kh, bool dilate) {
    morph_sep(src, dst, w, h, kw, kh, dilate);
}

struct Pt { double x, y; };

inline std::vector<Pt> convex_hull(std::vector<Pt> pts) {
    if (pts.size() < 3) return pts;
    std::sort(pts.begin(), pts.end(), [](const Pt& a, const Pt& b) {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    auto cross = [](const Pt& o, const Pt& a, const Pt& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    std::vector<Pt> hull(2 * pts.size());
    size_t k = 0;
    for (const Pt& p : pts) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) --k;
        hull[k++] = p;
    }
    for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        const Pt& p = pts[i - 1];
        while (k >= t && cross(hull[k - 2], hull[k - 1], p) <= 0) --k;
        hull[k++] = p;
    }
    hull.resize(k > 1 ? k - 1 : k);
    return hull;
}

inline bool min_area_rect(const std::vector<Pt>& hull, double* cx, double* cy,
                          double* w, double* h, double* ang,
                          std::array<std::array<double, 2>, 4>* corners) {
    if (hull.size() < 3) return false;
    double best = 1e300, bca = 1, bsa = 0;
    double bu0 = 0, bu1 = 0, bv0 = 0, bv1 = 0;
    const int steps = 720;   // 0.25°
    for (int i = 0; i < steps; ++i) {
        const double a = i * M_PI / steps;
        const double ca = std::cos(a), sa = std::sin(a);
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Pt& p : hull) {
            const double u = p.x * ca + p.y * sa;
            const double v = -p.x * sa + p.y * ca;
            u0 = std::min(u0, u); u1 = std::max(u1, u);
            v0 = std::min(v0, v); v1 = std::max(v1, v);
        }
        const double area = (u1 - u0) * (v1 - v0);
        if (area < best) {
            best = area; bca = ca; bsa = sa;
            bu0 = u0; bu1 = u1; bv0 = v0; bv1 = v1;
        }
    }
    auto inv = [&](double u, double v) -> std::pair<double, double> {
        return {u * bca - v * bsa, u * bsa + v * bca};
    };
    const auto p0 = inv(bu0, bv0);
    const auto p1 = inv(bu1, bv0);
    const auto p2 = inv(bu1, bv1);
    const auto p3 = inv(bu0, bv1);
    (*corners)[0] = {p0.first, p0.second};
    (*corners)[1] = {p1.first, p1.second};
    (*corners)[2] = {p2.first, p2.second};
    (*corners)[3] = {p3.first, p3.second};
    *cx = 0.5 * (bu0 + bu1) * bca - 0.5 * (bv0 + bv1) * bsa;
    *cy = 0.5 * (bu0 + bu1) * bsa + 0.5 * (bv0 + bv1) * bca;
    *w = bu1 - bu0;
    *h = bv1 - bv0;
    *ang = std::atan2(bsa, bca) * 180.0 / M_PI;
    return true;
}

//: 把最小外接矩形的四个旋转角点整理成 TL/TR/BR/BL（图像 y 向下）。
inline void order_tl_tr_br_bl(std::array<std::array<double, 2>, 4>* c) {
    auto& v = *c;
    std::sort(v.begin(), v.end(), [](const std::array<double, 2>& a,
                                     const std::array<double, 2>& b) {
        return (a[0] + a[1]) < (b[0] + b[1]);
    });
    const auto tl = v[0];
    const auto br = v[3];
    auto o1 = v[1], o2 = v[2];
    const auto tr = (o1[0] >= o2[0]) ? o1 : o2;
    const auto bl = (o1[0] >= o2[0]) ? o2 : o1;
    v = {tl, tr, br, bl};
}

//: 1D 三均值（暗背景 / 场地 / 亮白框+墙）→ 场地的 luma 带。
//: 固定带 (25,145) 是按"暗屏场地"（sample6）定的；录制的平台页面是**亮屏**：
//: 场地 ~100..180、桌面/窗框 ~40..90、白框/墙 ~250。这时固定带反而只圈住了桌面，
//: 最大连通域变成整幅图（四角贴边）。直方图三簇能同时适配两种亮度。
inline void auto_band(const uint8_t* rgb, int w, int h, int* lo, int* hi) {
    int hist[256] = {0};
    const int step = (static_cast<long>(w) * h > 150000) ? 2 : 1;
    for (int y = 0; y < h; y += step) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; x += step) ++hist[luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2])];
    }
    auto pct = [&](double q) {
        double total = 0;
        for (int i = 0; i < 256; ++i) total += hist[i];
        double acc = 0;
        for (int i = 0; i < 256; ++i) { acc += hist[i]; if (acc >= q * total) return static_cast<double>(i); }
        return 255.0;
    };
    double c[3] = {pct(0.10), pct(0.50), pct(0.90)};
    for (int it = 0; it < 20; ++it) {
        double s[3] = {0, 0, 0}, n[3] = {0, 0, 0};
        for (int l = 0; l < 256; ++l) {
            if (!hist[l]) continue;
            int k = 0;
            double best = 1e18;
            for (int j = 0; j < 3; ++j) {
                const double d = std::fabs(static_cast<double>(l) - c[j]);
                if (d < best) { best = d; k = j; }
            }
            s[k] += static_cast<double>(l) * hist[l];
            n[k] += hist[l];
        }
        for (int j = 0; j < 3; ++j) if (n[j] > 0) c[j] = s[j] / n[j];
    }
    std::sort(c, c + 3);
    int a = static_cast<int>((c[0] + c[1]) * 0.5);
    int b = static_cast<int>((c[1] + c[2]) * 0.5);
    if (a < 6) a = 6;
    if (b > 250) b = 250;
    if (b < a + 24) { a = static_cast<int>(c[1]) - 30; b = static_cast<int>(c[1]) + 30; }
    if (a < 6) a = 6;
    if (b > 252) b = 252;
    *lo = a;
    *hi = b;
}

//: 四角多边形的面积（像素²）。
inline double quad_area(const std::array<std::array<double, 2>, 4>& p) {
    double a = 0.0;
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        a += p[i][0] * p[j][1] - p[j][0] * p[i][1];
    }
    return std::fabs(a) * 0.5;
}

//: 四角是否像"白框"而不是"整幅图"：面积不能贴近 1，四角不能全都贴边。
inline bool plausible(const Quad& q, int w, int h) {
    if (!q.ok) return false;
    const double a = quad_area(q.p);
    if (a < 0.05 * w * h) return false;
    if (a > 0.90 * w * h) return false;
    int touch = 0;
    for (const auto& c : q.p) {
        if (c[0] <= 2.0 || c[0] >= w - 3.0 || c[1] <= 2.0 || c[1] >= h - 3.0) ++touch;
    }
    if (touch >= 4) return false;
    if (q.w < 0.20 * w || q.h < 0.20 * h) return false;
    return true;
}

//: 给定 luma 带做一遍检测。prefer_enclosed=true 时优先取**不贴画面边**的连通域
//: （亮屏录制里，画面外沿的桌面/窗框也会落进同一个带，取最大域会被它们带偏）。
inline Quad find_band(const uint8_t* rgb, int w, int h, int lo, int hi,
                      int close_w, int close_h, int open_w, int open_h,
                      double min_area_frac, bool prefer_enclosed) {
    Quad q;
    q.lo = lo;
    q.hi = hi;
    if (!rgb || w < 32 || h < 32) { q.why = 1; return q; }

    std::vector<uint8_t> mask(static_cast<size_t>(w) * h, 0);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; ++x) {
            const int l = luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
            mask[static_cast<size_t>(y) * w + x] = (l > lo && l < hi) ? 1 : 0;
        }
    }
    std::vector<uint8_t> a(mask.size(), 0), b(mask.size(), 0);
    // close = dilate -> erode（核 <=1 表示跳过：亮屏录制里大核会把白框线两侧桥接起来）
    if (close_w > 1 && close_h > 1) {
        morph_rect(mask.data(), a.data(), w, h, close_w, close_h, true);
        morph_rect(a.data(), b.data(), w, h, close_w, close_h, false);
    } else {
        b = mask;
    }
    // open = erode -> dilate
    if (open_w > 1 && open_h > 1) {
        morph_rect(b.data(), a.data(), w, h, open_w, open_h, false);
        morph_rect(a.data(), b.data(), w, h, open_w, open_h, true);
    }

    // 连通域（4 连通），同时记录"是否碰到画面边"。
    std::vector<int> label(mask.size(), 0);
    std::vector<int> stack;
    int best_label = 0, best_size = 0, next = 0;
    int enc_label = 0, enc_size = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t idx = static_cast<size_t>(y) * w + x;
            if (!b[idx] || label[idx]) continue;
            ++next;
            int size = 0;
            bool touch = false;
            stack.clear();
            stack.push_back(static_cast<int>(idx));
            label[idx] = next;
            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                ++size;
                const int cy = cur / w, cx = cur % w;
                if (cx == 0 || cy == 0 || cx == w - 1 || cy == h - 1) touch = true;
                const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = cx + dx[k], ny = cy + dy[k];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    const size_t ni = static_cast<size_t>(ny) * w + nx;
                    if (b[ni] && !label[ni]) { label[ni] = next; stack.push_back(static_cast<int>(ni)); }
                }
            }
            if (size > best_size) { best_size = size; best_label = next; }
            if (!touch && size > enc_size) { enc_size = size; enc_label = next; }
        }
    }
    const int need = static_cast<int>(min_area_frac * w * h);
    int use_label = best_label, use_size = best_size;
    if (prefer_enclosed && enc_label != 0 && enc_size >= need) {
        use_label = enc_label;
        use_size = enc_size;
    }
    if (use_label == 0 || use_size < need) {
        q.why = 2;
        return q;
    }
    std::vector<Pt> pts;
    pts.reserve(static_cast<size_t>(use_size));
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (label[static_cast<size_t>(y) * w + x] == use_label) pts.push_back({x + 0.5, y + 0.5});
        }
    }
    const auto hull = convex_hull(std::move(pts));
    std::array<std::array<double, 2>, 4> corners{};
    if (!min_area_rect(hull, &q.cx, &q.cy, &q.w, &q.h, &q.angle, &corners)) {
        q.why = 3;
        return q;
    }
    // 长边/短边换成正的 w/h；角度保留。
    if (q.h > q.w) {
        std::swap(q.w, q.h);
        q.angle += 90.0;
        while (q.angle >= 180.0) q.angle -= 180.0;
    }
    order_tl_tr_br_bl(&corners);
    q.p = corners;
    q.ok = true;
    q.why = 0;
    return q;
}

}   // namespace detail

//: 在整幅图（或裁剪 ROI）里找白框四角。
//:
//: 两遍：
//:   1. 固定带 `p.lo..p.hi`（暗屏场地基准，保持 sample6 口径不变）；
//:   2. 只有第 1 遍的四角**不可信**（贴满画面 / 面积 > 0.9）时，才用直方图三簇
//:      自动定带重试 —— 亮屏录制靠这一步把桌面/窗框挡在场外。
inline Quad find(const uint8_t* rgb, int w, int h, const Params& p = Params{}) {
    Quad q = detail::find_band(rgb, w, h, p.lo, p.hi, p.close_w, p.close_h,
                               p.open_w, p.open_h, p.min_area_frac, false);
    if (q.ok && detail::plausible(q, w, h)) return q;
    if (!p.auto_retry) return q;

    int lo = 0, hi = 0;
    detail::auto_band(rgb, w, h, &lo, &hi);
    if (hi <= lo + 16) return q;
    // 自动带下不用大闭运算：亮屏录制里白框线只有 2~4 px，21 高的核会把框外并进来。
    Quad q2 = detail::find_band(rgb, w, h, lo, hi, 1, 1, 3, 3, p.min_area_frac, true);
    if (q2.ok && detail::plausible(q2, w, h)) {
        q2.retried = 1;
        return q2;
    }
    return q;
}

//: 白框四角 → 像素→场地单应。世界角点就是场地 0..fw / 0..fh。
//: 图像 TL/TR/BR/BL ↔ 场地 (0,fh),(fw,fh),(fw,0),(0,0)（场地 y 朝上）。
inline rectify::Homography homography(const Quad& q, double fw = 640.0, double fh = 480.0) {
    if (!q.ok) return {};
    const std::array<std::array<double, 2>, 4> img = {{
        {{q.p[3][0], q.p[3][1]}},   // BL
        {{q.p[2][0], q.p[2][1]}},   // BR
        {{q.p[1][0], q.p[1][1]}},   // TR
        {{q.p[0][0], q.p[0][1]}},   // TL
    }};
    const std::array<std::array<double, 2>, 4> world = {{
        {{0.0, 0.0}}, {{fw, 0.0}}, {{fw, fh}}, {{0.0, fh}},
    }};
    return rectify::solve(img, world);
}

}   // namespace field_frame
}   // namespace vision_board

#endif   // VISION_BOARD_FIELD_FRAME_HPP
