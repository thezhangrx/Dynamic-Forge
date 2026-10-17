// ===========================================================================
// rectify.hpp —— 用**四角标定标记**把图像坐标映射到场地坐标（单应变换）
//
// 为什么需要：现在用的是"轴对齐比例 + 剪切角"的近似。它能吃掉小旋转，但吃不掉
// **透视**（近大远小），而真机位是斜视的。实测结果就是：门宽误差看着只有 -26，
// 门的位置却和真值差 130 场地单位（宽度是差值，原点/透视误差全落在位置上）。
//
// 四角标定标记给了 4 组已知对应（平台契约：边长 48、离边 8 → 中心在
// (32,32) (608,32) (32,448) (608,448)）。4 点足以解出**完整单应**
// （8 个自由度），旋转和透视同时被吃掉，而且这 4 点还直接给出场地的四边形
// 掩膜 —— 落在四边形外的几何一律不可信。
//
// 这里只放**数学**（不碰像素），方便单独测：
//     Quad q_img, q_fld; H = solve(q_img, q_fld); H.apply(x,y)
// 找标记本身在 `find_markers()`（下一层）。
// ===========================================================================
#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <vector>

namespace rectify {

/// 场地四角（平台契约）：左上、右上、右下、左下 的**标记中心**。
inline std::array<std::array<double, 2>, 4> contract_markers(double fw = 640.0,
                                                             double fh = 480.0) {
    const double s = 32.0;              // 边长 48、离边 8 → 中心 8+24 = 32
    return {{{s, s}, {fw - s, s}, {fw - s, fh - s}, {s, fh - s}}};
}

/// 8 参数单应：`[x', y', w'] = H * [x, y, 1]`，再除以 w'。
struct Homography {
    std::array<double, 9> h{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    bool ok = false;

    std::array<double, 2> apply(double x, double y) const {
        const double w = h[6] * x + h[7] * y + h[8];
        if (std::fabs(w) < 1e-12) return {0.0, 0.0};
        return {(h[0] * x + h[1] * y + h[2]) / w,
                (h[3] * x + h[4] * y + h[5]) / w};
    }
    /// 映射后是否落在场地里（含 `margin` 的容差；负 margin 表示往里收）。
    bool inside(double x, double y, double fw = 640.0, double fh = 480.0,
                double margin = 0.0) const {
        if (!ok) return false;
        const auto p = apply(x, y);
        return p[0] >= margin && p[0] <= fw - margin &&
               p[1] >= margin && p[1] <= fh - margin;
    }
};

/// 逆变换（3x3 伴随矩阵；单应可逆的假设在四点不共线时成立）。
inline Homography invert(const Homography& H) {
    Homography out;
    if (!H.ok) return out;
    const double* a = H.h.data();
    const double det = a[0] * (a[4] * a[8] - a[5] * a[7])
                     - a[1] * (a[3] * a[8] - a[5] * a[6])
                     + a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (std::fabs(det) < 1e-12) return out;
    out.h[0] = (a[4] * a[8] - a[5] * a[7]) / det;
    out.h[1] = (a[2] * a[7] - a[1] * a[8]) / det;
    out.h[2] = (a[1] * a[5] - a[2] * a[4]) / det;
    out.h[3] = (a[5] * a[6] - a[3] * a[8]) / det;
    out.h[4] = (a[0] * a[8] - a[2] * a[6]) / det;
    out.h[5] = (a[2] * a[3] - a[0] * a[5]) / det;
    out.h[6] = (a[3] * a[7] - a[4] * a[6]) / det;
    out.h[7] = (a[1] * a[6] - a[0] * a[7]) / det;
    out.h[8] = (a[0] * a[4] - a[1] * a[3]) / det;
    out.ok = true;
    return out;
}

/// 场地四角在**图像**里的包围盒（用单应的逆变换）。
///
/// 为什么用它代替"亮度边缘"：实测把相机录下来的帧回放时，墙上亮段的两端
/// 会产生比场地边界更强的竖直亮度台阶，于是"取两条最强台阶"量出来的宽度只有
/// 67 px（真值 544）。而四角标记是**已知场地坐标**的强特征，用它们反解出来的
/// 包围盒不依赖"哪条边更亮"。
inline bool field_bbox(const Homography& H, double field_w, double field_h, int w, int hh,
                       int* x0, int* x1, int* y0, int* y1) {
    const Homography inv = invert(H);
    if (!inv.ok) return false;
    const double corners[4][2] = {{0.0, 0.0}, {field_w, 0.0}, {field_w, field_h}, {0.0, field_h}};
    double mnx = 1e18, mxx = -1e18, mny = 1e18, mxy = -1e18;
    for (auto& c : corners) {
        const auto q = inv.apply(c[0], c[1]);
        if (q[0] < mnx) mnx = q[0];
        if (q[0] > mxx) mxx = q[0];
        if (q[1] < mny) mny = q[1];
        if (q[1] > mxy) mxy = q[1];
    }
    *x0 = static_cast<int>(std::lround(mnx));
    *x1 = static_cast<int>(std::lround(mxx));
    *y0 = static_cast<int>(std::lround(mny));
    *y1 = static_cast<int>(std::lround(mxy));
    (void)w; (void)hh;
    return true;
}

/// 4 点 → 单应（DLT，直接解 8×8 线性方程组；点序必须一致）。
///
/// 为什么不用 SVD：只有 8 个未知数、8 个方程，高斯消元（带部分主元）就够，
/// 而且没有外部依赖。退化情形（三点共线等）会把 `ok` 置 false，由调用方降级。
inline Homography solve(const std::array<std::array<double, 2>, 4>& src,
                        const std::array<std::array<double, 2>, 4>& dst) {
    Homography H;
    // 方程：对每个点
    //   x*h0 + y*h1 + h2 - x'*x*h6 - x'*y*h7 = x'
    //   x*h3 + y*h4 + h5 - y'*x*h6 - y'*y*h7 = y'
    double a[8][9] = {{0}};
    for (int i = 0; i < 4; ++i) {
        const double x = src[i][0], y = src[i][1];
        const double u = dst[i][0], v = dst[i][1];
        double* r0 = a[2 * i];
        double* r1 = a[2 * i + 1];
        r0[0] = x; r0[1] = y; r0[2] = 1.0;
        r0[6] = -u * x; r0[7] = -u * y; r0[8] = u;
        r1[3] = x; r1[4] = y; r1[5] = 1.0;
        r1[6] = -v * x; r1[7] = -v * y; r1[8] = v;
    }
    // 高斯消元（部分主元）
    for (int col = 0; col < 8; ++col) {
        int piv = col;
        for (int r = col + 1; r < 8; ++r)
            if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
        if (std::fabs(a[piv][col]) < 1e-12) return H;      // 退化
        if (piv != col)
            for (int c = col; c < 9; ++c) std::swap(a[piv][c], a[col][c]);
        const double d = a[col][col];
        for (int c = col; c < 9; ++c) a[col][c] /= d;
        for (int r = 0; r < 8; ++r) {
            if (r == col) continue;
            const double f = a[r][col];
            if (f == 0.0) continue;
            for (int c = col; c < 9; ++c) a[r][c] -= f * a[col][c];
        }
    }
    for (int i = 0; i < 8; ++i) H.h[static_cast<size_t>(i)] = a[i][8];
    H.h[8] = 1.0;
    H.ok = true;
    return H;
}


// ===========================================================================
// 找四角标定标记（只用"亮 + 方"，不用颜色/尺寸常数）
// ===========================================================================
// 标记 = 黑底 + 白色角块，所以它在图上是一组**小而方的亮块**，位置就在场地四角。
// 判据：阈值取本帧最亮像素的 0.75（与 `find_player` 同一把尺，跨曝光），
// 连通域取 `aspect < 2`、面积在 [16, 4000]，然后按**象限**取四个角上最亮的那个。
struct Marker {
    double x = 0.0, y = 0.0;
    int w = 0, h = 0;
};

/// **场地亮区**的包围盒：把画面按亮度二值化（阈值取 (p10+p90)/2，与配色无关），
/// 取最大连通区的包围盒。
///
/// 为什么需要它：`find_markers` 原来按"每个象限里离画面角最近的最亮块"选，
/// 实测会选到**屏幕边框/键盘反光**（照片里四角附近确实有更亮的东西），
/// 于是四角标记拼出来的单应把场地映射到画面外（bbox 宽 745 px > 画面 640）。
/// 先用亮区把"屏幕/场地"圈出来，再从里面挑标记，就不会跑到画面外的杂物上。
inline bool field_region_bbox(const uint8_t* rgb, int w, int h,
                              int* x0, int* x1, int* y0, int* y1) {
    if (!rgb || w < 32 || h < 32) return false;
    std::vector<int> hist(256, 0);
    for (int i = 0; i < w * h; ++i) {
        const int o = i * 3;
        int l = rgb[o];
        if (rgb[o + 1] > l) l = rgb[o + 1];
        if (rgb[o + 2] > l) l = rgb[o + 2];
        ++hist[static_cast<size_t>(l)];
    }
    const long n = static_cast<long>(w) * h;
    auto q = [&](double f) {
        const double want = f * static_cast<double>(n);
        long acc = 0;
        for (size_t v = 0; v < hist.size(); ++v) {
            acc += hist[v];
            if (static_cast<double>(acc) >= want) return static_cast<int>(v);
        }
        return 255;
    };
    const int thr = (q(0.10) + q(0.90)) / 2;
    // 列/行的"亮像素占比"：场地是成片亮的，边框/杂物只占少数列
    std::vector<int> colcnt(static_cast<size_t>(w), 0), rowcnt(static_cast<size_t>(h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int o = (y * w + x) * 3;
            int l = rgb[o];
            if (rgb[o + 1] > l) l = rgb[o + 1];
            if (rgb[o + 2] > l) l = rgb[o + 2];
            if (l > thr) { ++colcnt[static_cast<size_t>(x)]; ++rowcnt[static_cast<size_t>(y)]; }
        }
    }
    auto span = [](const std::vector<int>& cnt, int limit) {
        int lo = -1, hi = -1;
        for (int i = 0; i < static_cast<int>(cnt.size()); ++i) {
            if (cnt[static_cast<size_t>(i)] * 2 >= limit) { if (lo < 0) lo = i; hi = i; }
        }
        return std::pair<int, int>{lo, hi};
    };
    const auto cx = span(colcnt, h);
    const auto cy = span(rowcnt, w);
    if (cx.first < 0 || cy.first < 0) return false;
    *x0 = cx.first; *x1 = cx.second; *y0 = cy.first; *y1 = cy.second;
    return (*x1 - *x0) > w / 3 && (*y1 - *y0) > h / 3;
}


//: 用**暗框**找四角标记（黑底 + 白图案里的黑框），失败时调用方退回亮块检测。
//:
//: 为什么需要第二条路：真机位照片上，标记的白图案和墙的高光混在一起，
//: "亮块"检测只能稳定找到 2~3 个，而且会选中墙端（实测四角拼出来的单应把场地
//: 映射到画面外，宽 745 px > 640）。标记的**黑框**是场上最暗的东西，反而更干净。
inline std::vector<Marker> find_markers_dark(const uint8_t* rgb, int w, int h) {
    std::vector<Marker> res;
    if (!rgb || w < 32 || h < 32) return res;
    int rx0 = 0, rx1 = w - 1, ry0 = 0, ry1 = h - 1;
    if (!field_region_bbox(rgb, w, h, &rx0, &rx1, &ry0, &ry1)) return res;
    std::vector<int> hh(256, 0);
    for (int i = 0; i < w * h; ++i) {
        const int o = i * 3;
        int l = rgb[o];
        if (rgb[o + 1] > l) l = rgb[o + 1];
        if (rgb[o + 2] > l) l = rgb[o + 2];
        ++hh[static_cast<size_t>(l)];
    }
    const long total = static_cast<long>(w) * h;
    auto qq = [&](double f) {
        const double want = f * static_cast<double>(total);
        long acc = 0;
        for (size_t v = 0; v < hh.size(); ++v) {
            acc += hh[v];
            if (static_cast<double>(acc) >= want) return static_cast<int>(v);
        }
        return 255;
    };
    const int thr = (qq(0.10) + qq(0.50)) / 2;
    struct Comp { int x0, y0, x1, y1; long area, sx, sy; };
    std::vector<Comp> comps;
    struct Run { int a, b, id; };
    std::vector<Run> prev;
    for (int y = ry0; y <= ry1; ++y) {
        std::vector<Run> cur;
        int x = rx0;
        while (x <= rx1) {
            const int o = (y * w + x) * 3;
            int l = rgb[o];
            if (rgb[o + 1] > l) l = rgb[o + 1];
            if (rgb[o + 2] > l) l = rgb[o + 2];
            if (l > thr) { ++x; continue; }
            const int s0 = x;
            while (x <= rx1) {
                const int k = (y * w + x) * 3;
                int v = rgb[k];
                if (rgb[k + 1] > v) v = rgb[k + 1];
                if (rgb[k + 2] > v) v = rgb[k + 2];
                if (v > thr) break;
                ++x;
            }
            int id = -1;
            for (const Run& r : prev)
                if (r.a <= x - 1 && r.b >= s0) { id = r.id; break; }
            if (id < 0) {
                comps.push_back(Comp{s0, y, x - 1, y, 0, 0, 0});
                id = static_cast<int>(comps.size()) - 1;
            }
            Comp& c = comps[static_cast<size_t>(id)];
            if (s0 < c.x0) c.x0 = s0;
            if (x - 1 > c.x1) c.x1 = x - 1;
            c.y1 = y;
            c.area += x - s0;
            c.sx += static_cast<long>(s0 + x - 1) * (x - s0) / 2;
            c.sy += static_cast<long>(y) * (x - s0);
            cur.push_back(Run{s0, x - 1, id});
        }
        prev = cur;
    }
    const double mx = 0.5 * (rx0 + rx1), my = 0.5 * (ry0 + ry1);
    std::vector<Marker> quad(4);
    std::vector<double> best(4, 1e18);
    for (const Comp& c : comps) {
        const int bw = c.x1 - c.x0 + 1, bh = c.y1 - c.y0 + 1;
        if (c.area < 200 || c.area > 12000) continue;      // 标记的黑框是"块"，不是细线
        const double asp = static_cast<double>(bw > bh ? bw : bh) /
                           static_cast<double>(bw < bh ? bw : bh);
        if (asp >= 2.0) continue;
        const double cx = static_cast<double>(c.sx) / c.area + 0.5;
        const double cy = static_cast<double>(c.sy) / c.area + 0.5;
        const int q = (cx < mx ? 0 : 1) + (cy < my ? 0 : 2);
        const double tx = (q % 2 == 0) ? static_cast<double>(rx0) : static_cast<double>(rx1);
        const double ty = (q / 2 == 0) ? static_cast<double>(ry0) : static_cast<double>(ry1);
        const double d = std::hypot(cx - tx, cy - ty);
        if (d < best[static_cast<size_t>(q)]) {
            best[static_cast<size_t>(q)] = d;
            quad[static_cast<size_t>(q)] = Marker{cx, cy, bw, bh};
        }
    }
    for (int i = 0; i < 4; ++i)
        if (best[static_cast<size_t>(i)] < 1e17) res.push_back(quad[static_cast<size_t>(i)]);
    return res;
}

inline std::vector<Marker> find_markers(const uint8_t* rgb, int w, int h) {
    std::vector<Marker> out;
    if (!rgb || w < 32 || h < 32) return out;
    int lmax = 0;
    for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
        for (int x = 0; x < w; ++x) {
            const size_t i = base + static_cast<size_t>(x);
            int l = rgb[i * 3];
            if (rgb[i * 3 + 1] > l) l = rgb[i * 3 + 1];
            if (rgb[i * 3 + 2] > l) l = rgb[i * 3 + 2];
            if (l > lmax) lmax = l;
        }
    }
    const int thr = static_cast<int>(0.75 * static_cast<double>(lmax));
    if (lmax < 48) return out;                 // 整帧没有亮物
    // **暗框优先**：标定标记是"黑底 + 白图案"，它的黑框是场上最暗的东西；
    // 而"亮块"在真机位上会和墙的高光混在一起（实测只找到 3 个、且有一个在墙上）。
    // 阈值取全场亮度的 (p10+p50)/2 —— 相对量，不含相机常数。
    int thr_dark = 0;
    {
        std::vector<int> hh(256, 0);
        for (int y = 0; y < h; ++y) {
            const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
            for (int x = 0; x < w; ++x) {
                const size_t k = base + static_cast<size_t>(x);
                int l = rgb[k * 3];
                if (rgb[k * 3 + 1] > l) l = rgb[k * 3 + 1];
                if (rgb[k * 3 + 2] > l) l = rgb[k * 3 + 2];
                ++hh[static_cast<size_t>(l)];
            }
        }
        const long total = static_cast<long>(w) * h;
        auto qq = [&](double f) {
            const double want = f * static_cast<double>(total);
            long acc = 0;
            for (size_t v = 0; v < hh.size(); ++v) {
                acc += hh[v];
                if (static_cast<double>(acc) >= want) return static_cast<int>(v);
            }
            return 255;
        };
        thr_dark = (qq(0.10) + qq(0.50)) / 2;
    }

    struct Comp { int x0, y0, x1, y1; long area, sx, sy; int peak; };
    std::vector<Comp> comps;
    struct Run { int a, b, id; };
    std::vector<Run> prev;
    for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
        std::vector<Run> cur;
        int x = 0;
        while (x < w) {
            const size_t idx = base + static_cast<size_t>(x);
            int l = rgb[idx * 3];
            if (rgb[idx * 3 + 1] > l) l = rgb[idx * 3 + 1];
            if (rgb[idx * 3 + 2] > l) l = rgb[idx * 3 + 2];
            if (l <= thr) { ++x; continue; }
            const int s0 = x;
            int peak = l;
            while (x < w) {
                const size_t k = base + static_cast<size_t>(x);
                int v = rgb[k * 3];
                if (rgb[k * 3 + 1] > v) v = rgb[k * 3 + 1];
                if (rgb[k * 3 + 2] > v) v = rgb[k * 3 + 2];
                if (v <= thr) break;
                if (v > peak) peak = v;
                ++x;
            }
            int id = -1;
            for (const Run& r : prev)
                if (r.a <= x - 1 && r.b >= s0) { id = r.id; break; }
            if (id < 0) {
                comps.push_back(Comp{s0, y, x - 1, y, 0, 0, 0, peak});
                id = static_cast<int>(comps.size()) - 1;
            }
            Comp& c = comps[static_cast<size_t>(id)];
            if (s0 < c.x0) c.x0 = s0;
            if (x - 1 > c.x1) c.x1 = x - 1;
            c.y1 = y;
            c.area += x - s0;
            c.sx += static_cast<long>(s0 + x - 1) * (x - s0) / 2;
            c.sy += static_cast<long>(y) * (x - s0);
            if (peak > c.peak) c.peak = peak;
            cur.push_back(Run{s0, x - 1, id});
        }
        prev = cur;
    }
    for (const Comp& c : comps) {
        const int bw = c.x1 - c.x0 + 1, bh = c.y1 - c.y0 + 1;
        if (c.area < 16 || c.area > 4000) continue;
        const double asp = static_cast<double>(bw > bh ? bw : bh) /
                           static_cast<double>(bw < bh ? bw : bh);
        if (asp >= 2.0) continue;
        out.push_back(Marker{static_cast<double>(c.sx) / c.area + 0.5,
                             static_cast<double>(c.sy) / c.area + 0.5, bw, bh});
    }
    // 每个象限取**离场地亮区四角最近**的候选（标记就在四角）。
    // 用亮区包围盒而不是画面边界：画面四角附近常有屏幕边框/键盘反光。
    int rx0 = 0, rx1 = w - 1, ry0 = 0, ry1 = h - 1;
    if (!field_region_bbox(rgb, w, h, &rx0, &rx1, &ry0, &ry1)) {
        rx0 = 0; rx1 = w - 1; ry0 = 0; ry1 = h - 1;
    }
    const double mx = 0.5 * (rx0 + rx1), my = 0.5 * (ry0 + ry1);
    std::vector<Marker> quad(4);
    std::vector<double> best(4, 1e18);
    for (const Marker& m : out) {
        if (m.x < rx0 || m.x > rx1 || m.y < ry0 || m.y > ry1) continue;
        const int q = (m.x < mx ? 0 : 1) + (m.y < my ? 0 : 2);
        const double cx = (q % 2 == 0) ? static_cast<double>(rx0) : static_cast<double>(rx1);
        const double cy = (q / 2 == 0) ? static_cast<double>(ry0) : static_cast<double>(ry1);
        const double d = std::hypot(m.x - cx, m.y - cy);
        if (d < best[static_cast<size_t>(q)]) {
            best[static_cast<size_t>(q)] = d;
            quad[static_cast<size_t>(q)] = m;
        }
    }
    std::vector<Marker> res;
    for (int i = 0; i < 4; ++i)
        if (best[static_cast<size_t>(i)] < 1e17) res.push_back(quad[static_cast<size_t>(i)]);
    return res;
}

}  // namespace rectify
