// ===========================================================================
// band_gap.hpp —— **结构判据**：从一帧 RGB 里直接找"带缺口的横墙"（PS 侧）
//
// 为什么不用逐像素阈值（无论自动还是固定）：真机位实测，这台相机的
//   * 背景自己的 d=r-b 就有 +24±19，而墙只有 +10~+30 —— 分布**重叠**；
//   * 屏幕辉光把整幅图抬成一个宽中灰（亮度 Otsu 谷值落在 88~142，"亮像素"83%）。
// 所以"比背景暖几个 σ""亮不亮"这类**全局**判据在这台相机上从根上不成立
// （试过，见 main.cpp 里 --auto-thresholds 的实测：零矩形从 2% 变成 100%）。
//
// 结构判据只问一件事：**某一行带里，比带外暖得是否显著**。
// 曝光/白平衡是整幅图一起变（增益+偏置），带内带外**同时**变，差分就抵消掉了；
// 而墙是"横的一条"，带内外的差分在墙所在的行上必然出现台阶。于是：
//
//     ev[y]        = 该行 r-b 的均值（行内平均把噪声压到 1/sqrt(W)）
//     带(y0,y1)     = max [ mean_inside(ev) - mean_outside(ev) ]，再非极大值抑制
//     列剖面 prof[x] = 带内均值 - 带外均值（同一列）
//     缺口          = max [ min(左肩, 右肩) - 缺口内 ]（用 min：一侧是背景就掉分）
//     宽度          = 从缺口中心向两侧滞回扩到 (墙电平+背景电平)/2
//     门的判据       = min(左墙长, 右墙长) / 缺口宽 >= 1（边框上的角标洞过不了这一关）
//
// 与 `core/vision/relative.py` 的 `wall_bands/best_gap/best_door` 是同一套算法，
// 有一条测试拿同一张合成图对两边逐项比（容差 1 px），保证 C++ 与 Python 不分家。
// ===========================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace band_gap {

struct Door {
    bool found = false;
    int  y0 = 0, y1 = 0;      // 墙带的行范围
    int  x0 = 0, x1 = 0;      // 缺口列范围（含两端）
    int  cx = 0;              // 缺口中心列
    int  width = 0;           // 缺口宽度（列）
    float snr = 0.0f;         // 缺口显著性（分数/σ）
    float support = 0.0f;     // min(左墙长, 右墙长) / 缺口宽
    int  n_bands = 0;         // 找到几条墙带（诊断）
};

namespace detail {

constexpr float kMadToSigma = 1.4826f;
constexpr float kQuantSigma = 0.29f;    // 8 bit 量化噪声下限

inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    const size_t m = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(m), v.end());
    double a = v[m];
    if (v.size() % 2 == 0) {
        std::nth_element(v.begin(), v.begin() + static_cast<long>(m - 1), v.end());
        a = 0.5 * (a + v[m - 1]);
    }
    return a;
}

inline double mad(const std::vector<double>& v, double med) {
    std::vector<double> d(v.size());
    for (size_t i = 0; i < v.size(); ++i) d[i] = std::fabs(v[i] - med);
    return median(std::move(d));
}

//: 逐行"暖度"：r-b 的行内均值，再 3 行平滑（抑制单行噪声）。
inline std::vector<double> row_profile(const uint8_t* rgb, int w, int h) {
    std::vector<double> ev(static_cast<size_t>(h), 0.0);
    for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
        double s = 0.0;
        for (int x = 0; x < w; ++x) {
            const size_t i = base + static_cast<size_t>(x);
            s += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
        }
        ev[static_cast<size_t>(y)] = s / static_cast<double>(w);
    }
    std::vector<double> out(ev.size(), 0.0);
    for (size_t y = 0; y < ev.size(); ++y) {
        const size_t a = y > 0 ? y - 1 : 0;
        const size_t b = y + 1 < ev.size() ? y + 1 : ev.size() - 1;
        out[y] = (ev[a] + ev[y] + ev[b]) / 3.0;
    }
    return out;
}

//: 同一列的"带内 - 带外"暖度差。
inline std::vector<double> col_profile(const uint8_t* rgb, int w, int h,
                                       int y0, int y1, int pad) {
    std::vector<double> prof(static_cast<size_t>(w), 0.0);
    const int ya = y0 - pad < 0 ? 0 : y0 - pad;
    const int yb = (y1 + pad < h) ? y1 + pad : h - 1;
    const int n_in = y1 - y0 + 1;
    for (int x = 0; x < w; ++x) {
        double inside = 0.0;
        for (int y = y0; y <= y1; ++y) {
            const size_t i = (static_cast<size_t>(y) * w + static_cast<size_t>(x));
            inside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
        }
        inside /= static_cast<double>(n_in);
        double outside = 0.0;
        int n_out = 0;
        for (int y = ya; y < y0; ++y) {
            const size_t i = (static_cast<size_t>(y) * w + static_cast<size_t>(x));
            outside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
            ++n_out;
        }
        for (int y = y1 + 1; y <= yb; ++y) {
            const size_t i = (static_cast<size_t>(y) * w + static_cast<size_t>(x));
            outside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
            ++n_out;
        }
        if (n_out > 0) outside /= static_cast<double>(n_out);
        prof[static_cast<size_t>(x)] = inside - outside;
    }
    return prof;
}

}  // namespace detail

//: 主入口：在当前帧里找"带缺口的横墙"，返回最像门的那一处。
inline Door find(const uint8_t* rgb, int w, int h, int min_rows = 6,
                 int max_rows = 40, int pad = 6, float k_sigma = 5.0f) {
    Door out;
    if (!rgb || w < 32 || h < min_rows + 2 * pad + 4) return out;
    const std::vector<double> ev = detail::row_profile(rgb, w, h);
    const int hn = static_cast<int>(ev.size());
    // σ：行剖面一阶差分的 MAD（缓慢的光照梯度被差分消掉）
    std::vector<double> diffs;
    diffs.reserve(ev.size());
    for (int y = 0; y + 1 < hn; ++y) diffs.push_back(ev[y + 1] - ev[y]);
    float sigma = static_cast<float>(detail::mad(diffs, detail::median(diffs)) * detail::kMadToSigma);
    if (sigma < detail::kQuantSigma) sigma = detail::kQuantSigma;

    auto mean_of = [&ev](int a, int b) {
        if (b < a) return 0.0;
        double s = 0.0;
        for (int y = a; y <= b; ++y) s += ev[static_cast<size_t>(y)];
        return s / static_cast<double>(b - a + 1);
    };

    // ---- ① 所有横墙带（非极大值抑制）------------------------------------
    struct Band { double contrast; int y0, y1; };
    std::vector<Band> cands;
    for (int y0 = pad; y0 < hn - min_rows - pad; ++y0) {
        const int ymax = (y0 + max_rows + 1 < hn - pad) ? y0 + max_rows + 1 : hn - pad;
        for (int y1 = y0 + min_rows; y1 < ymax; ++y1) {
            const double c = mean_of(y0, y1) - 0.5 * (mean_of(y0 - pad, y0 - 1) +
                                                      mean_of(y1 + 1, y1 + pad));
            cands.push_back(Band{c, y0, y1});
        }
    }
    std::sort(cands.begin(), cands.end(),
              [](const Band& a, const Band& b) { return a.contrast > b.contrast; });
    std::vector<char> taken(static_cast<size_t>(hn), 0);
    std::vector<Band> bands;
    for (const Band& b : cands) {
        if (bands.size() >= 4) break;
        if (b.contrast / sigma < k_sigma) break;
        bool hit = false;
        for (int y = b.y0; y <= b.y1; ++y) if (taken[static_cast<size_t>(y)]) { hit = true; break; }
        if (hit) continue;
        bands.push_back(b);
        for (int y = b.y0 > 1 ? b.y0 - 1 : 0; y <= b.y1 + 1 && y < hn; ++y) taken[static_cast<size_t>(y)] = 1;
    }
    out.n_bands = static_cast<int>(bands.size());

    // ---- ② 每条带找缺口，取"支撑比"最大的那扇门 -------------------------
    for (const Band& b : bands) {
        const std::vector<double> prof = detail::col_profile(rgb, w, h, b.y0, b.y1, pad);
        const int wn = static_cast<int>(prof.size());
        const int thick = b.y1 - b.y0 + 1;
        int min_gap = thick < 4 ? 4 : thick;       // 几何先验：门洞不会比墙还窄
        int shoulder = thick < 4 ? 4 : thick;
        if (wn < min_gap + 2 * shoulder + 2) continue;
        std::vector<double> pre(static_cast<size_t>(wn) + 1, 0.0);
        for (int x = 0; x < wn; ++x) pre[x + 1] = pre[x] + prof[x];
        auto mean_col = [&pre](int a, int b) {
            if (b < a) return 0.0;
            return (pre[b + 1] - pre[a]) / static_cast<double>(b - a + 1);
        };
        std::vector<double> cd;
        cd.reserve(prof.size());
        for (int x = 0; x + 1 < wn; ++x) cd.push_back(prof[x + 1] - prof[x]);
        float csigma = static_cast<float>(detail::mad(cd, detail::median(cd)) * detail::kMadToSigma);
        if (csigma < detail::kQuantSigma) csigma = detail::kQuantSigma;

        double best = -1e18;
        int ba = 0, bb = 0;
        double wall_level = 0.0, bg_level = 0.0;
        const int wmax = 240;
        for (int a = shoulder; a + shoulder + min_gap <= wn; ++a) {
            const int lo = (a + wmax < wn - shoulder) ? wmax : (wn - shoulder - a);
            for (int width = min_gap; width <= lo; ++width) {
                const int b2 = a + width - 1;
                const double left = mean_col(a - shoulder, a - 1);
                const double right = mean_col(b2 + 1, b2 + shoulder);
                const double inside = mean_col(a, b2);
                const double l2 = left < right ? left : right;
                const double score = l2 - inside;
                if (score > best) {
                    best = score;
                    ba = a; bb = b2;
                    wall_level = left > right ? left : right;
                    bg_level = inside;
                }
            }
        }
        if (best <= 0.0) continue;
        const double half = 0.5 * (wall_level + bg_level);
        int cxx = (ba + bb) / 2;
        int x0 = cxx, x1 = cxx;
        while (x0 - 1 >= 0 && prof[static_cast<size_t>(x0 - 1)] < half) --x0;
        while (x1 + 1 < wn && prof[static_cast<size_t>(x1 + 1)] < half) ++x1;
        const int width = x1 - x0 + 1;
        if (width < min_gap) continue;
        int left_wall = 0, right_wall = 0;
        for (int x = x0 - 1; x >= 0 && prof[static_cast<size_t>(x)] >= half; --x) ++left_wall;
        for (int x = x1 + 1; x < wn && prof[static_cast<size_t>(x)] >= half; ++x) ++right_wall;
        const float support = static_cast<float>(left_wall < right_wall ? left_wall : right_wall) /
                              static_cast<float>(width);
        const float snr = static_cast<float>(best) / csigma;
        if (snr < k_sigma) continue;
        if (support < 1.0f) continue;
        if (!out.found || support > out.support) {
            out.found = true;
            out.support = support;
            out.snr = snr;
            out.y0 = b.y0; out.y1 = b.y1;
            out.x0 = x0;   out.x1 = x1;
            out.cx = (x0 + x1) / 2;
            out.width = width;
        }
    }
    return out;
}

}  // namespace band_gap
