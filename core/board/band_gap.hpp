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
    float angle_deg = 0.0f;   // 使带对比最大的剪切角（≈ 相机相对场地的旋转）
    //: 该角度下**墙带**本身的对比（以 σ 为单位）。
    //: 角度搜索必须按它来选，而不是按"这角度下最好的门有多宽" ——
    //: 队头角度上一条假窄缝就可能比正确角度下的真缺口更"宽"，
    //: 于是搜索会停在错的角度（实测旋转 6° 时停在 0°）。
    float band_score = 0.0f;
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

//: 某列的斜向偏移：把行沿角度"拉直"用（小角度下 shear 与 rotate 等价，
//: 而且**不用重采样整幅图** —— 只是每列查表时换一个 y）。
inline int shear_dy(int x, int w, double tan_a) {
    const double dx = static_cast<double>(x) - 0.5 * static_cast<double>(w);
    return static_cast<int>(std::lround(dx * tan_a));
}

//: 逐行"暖度"（可剪切）：`ev[y] = mean_x d(x, y - dy(x))`，再 3 行平滑。
//:
//: 为什么要剪切：真机位画面相对场地**有旋转**（实测 ~5~10°）。一条斜墙在"水平
//: 行带"里每行只剩很短一段 → 左右肩被带截断 → 支撑比永远过不了 1（实测 0.05~0.17）。
//: 沿角度把行拉直之后，墙才重新变成"一横条"。
inline std::vector<double> row_profile(const uint8_t* rgb, int w, int h,
                                        double tan_a = 0.0) {
    std::vector<double> ev(static_cast<size_t>(h), 0.0);
    std::vector<double> acc(static_cast<size_t>(h), 0.0);
    std::vector<int> cnt(static_cast<size_t>(h), 0);
    for (int x = 0; x < w; ++x) {
        const int dy = shear_dy(x, w, tan_a);
        for (int y = 0; y < h; ++y) {
            int yy = y + dy;
            if (yy < 0) yy = 0;
            if (yy >= h) yy = h - 1;
            const size_t i = static_cast<size_t>(yy) * static_cast<size_t>(w) +
                             static_cast<size_t>(x);
            acc[static_cast<size_t>(y)] +=
                static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
            ++cnt[static_cast<size_t>(y)];
        }
    }
    for (int y = 0; y < h; ++y)
        ev[static_cast<size_t>(y)] = cnt[static_cast<size_t>(y)] > 0
            ? acc[static_cast<size_t>(y)] / static_cast<double>(cnt[static_cast<size_t>(y)])
            : 0.0;
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
                                       int y0, int y1, int pad,
                                       double tan_a = 0.0) {
    std::vector<double> prof(static_cast<size_t>(w), 0.0);
    const int ya = y0 - pad < 0 ? 0 : y0 - pad;
    const int yb = (y1 + pad < h) ? y1 + pad : h - 1;
    const int n_in = y1 - y0 + 1;
    for (int x = 0; x < w; ++x) {
        const int dy = shear_dy(x, w, tan_a);
        auto at = [&](int y) {
            int yy = y + dy;
            if (yy < 0) yy = 0;
            if (yy >= h) yy = h - 1;
            return static_cast<size_t>(yy) * static_cast<size_t>(w) + static_cast<size_t>(x);
        };
        double inside = 0.0;
        for (int y = y0; y <= y1; ++y) {
            const size_t i = at(y);
            inside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
        }
        inside /= static_cast<double>(n_in);
        double outside = 0.0;
        int n_out = 0;
        for (int y = ya; y < y0; ++y) {
            const size_t i = at(y);
            outside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
            ++n_out;
        }
        for (int y = y1 + 1; y <= yb; ++y) {
            const size_t i = at(y);
            outside += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
            ++n_out;
        }
        if (n_out > 0) outside /= static_cast<double>(n_out);
        prof[static_cast<size_t>(x)] = inside - outside;
    }
    return prof;
}

}  // namespace detail

//: 在**给定剪切角**下找门（对 `find()` 而言是一次内层调用）。
inline Door find_at(const uint8_t* rgb, int w, int h, double tan_a,
                    int min_rows = 6, int max_rows = 40, int pad = 6,
                    float k_sigma = 5.0f, std::vector<Door>* all = nullptr,
                    int min_gap_px = 0) {
    Door out;
    if (!rgb || w < 32 || h < min_rows + 2 * pad + 4) return out;
    const std::vector<double> ev = detail::row_profile(rgb, w, h, tan_a);
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
    for (const Band& b : bands) {
        const float sc = static_cast<float>(b.contrast / sigma);
        if (sc > out.band_score) out.band_score = sc;
    }

    // ---- ② 每条带找缺口，取"支撑比"最大的那扇门 -------------------------
    for (const Band& b : bands) {
        const std::vector<double> prof =
            detail::col_profile(rgb, w, h, b.y0, b.y1, pad, tan_a);
        const int wn = static_cast<int>(prof.size());
        const int thick = b.y1 - b.y0 + 1;
        // 缺口宽度下限：**可通过性**（调用方按"角色直径 × 1.6"给），
        // 没给时退回几何先验"门洞不会比墙还窄"。
        int min_gap = thick < 4 ? 4 : thick;
        if (min_gap_px > min_gap) min_gap = min_gap_px;
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
        // 选门：先比**宽度**，同宽再看支撑比。
        // 教训：真机位帧上门洞宽 24~58 px，而角标洞/反光缝只有 10~18 px；
        // 按支撑比挑会选中那些窄缝（它们的左右肩比值反而更高），
        // 于是决策拿到一个"只有 10 px 的门" → NO_GAP。
        if (all) {
            Door each;
            each.found = true;
            each.support = support;
            each.snr = snr;
            each.y0 = b.y0; each.y1 = b.y1;
            each.x0 = x0;   each.x1 = x1;
            each.cx = (x0 + x1) / 2;
            each.width = width;
            all->push_back(each);
        }
        if (!out.found || width > out.width ||
            (width == out.width && support > out.support)) {
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

//: 把所有**门**都拿出来（每个带一扇；角度用使带对比最大的那个）。
//:
//: 为什么需要"全部"：场景里常有 2 条横墙（各自缺口位置不同）+ 场地边框，
//: 只挑最宽的那扇，会丢掉玩家正对着的那堵墙 —— 而 `gap_avoid` 要的是
//: "我正在朝它走的那个缺口"，不是"全场最宽的缺口"。选择权交给决策层，
//: 判据层只负责把几何报准。
inline std::vector<Door> find_all(const uint8_t* rgb, int w, int h,
                                  int min_rows = 6, int max_rows = 40, int pad = 6,
                                  float k_sigma = 5.0f, float max_angle_deg = 10.0f,
                                  float step_deg = 2.5f, int min_gap_px = 0) {
    std::vector<Door> best_doors;
    int best_n = -1;
    for (float ang = -max_angle_deg; ang <= max_angle_deg + 0.001f; ang += step_deg) {
        const double tan_a = std::tan(ang * 3.14159265358979 / 180.0);
        std::vector<Door> doors;
        const Door one = find_at(rgb, w, h, tan_a, min_rows, max_rows, pad, k_sigma,
                                 &doors, min_gap_px);
        if (!doors.empty() && static_cast<int>(doors.size()) > best_n) {
            best_n = static_cast<int>(doors.size());
            for (Door& d : doors) d.angle_deg = ang;
            best_doors = doors;
            if (!one.found) continue;
        }
    }
    return best_doors;
}

//: 主入口：在 ±`kMaxAngleDeg` 里搜"使墙带对比最大"的剪切角，再在该角下找门。
//:
//: 为什么要搜角度而不是先矫正：4 角标 homography 是**正确**的解法（CAR_MIGRATION
//: C1），但它需要先可靠地找到 4 个角标；而这里只要求"相机大致正对场地、有一个
//: 小旋转"。搜角度用的是同一把尺（带对比），不需要额外标定，也不改变场景契约。
inline Door find(const uint8_t* rgb, int w, int h, int min_rows = 6,
                 int max_rows = 40, int pad = 6, float k_sigma = 5.0f,
                 float max_angle_deg = 10.0f, float step_deg = 2.5f) {
    Door best;
    // 先按 0° 跑一遍（绝大多数情况就是 0°），再按步长扫两侧
    for (float ang = -max_angle_deg; ang <= max_angle_deg + 0.001f; ang += step_deg) {
        const double tan_a = std::tan(ang * 3.14159265358979 / 180.0);
        Door d = find_at(rgb, w, h, tan_a, min_rows, max_rows, pad, k_sigma);
        if (d.n_bands == 0) continue;
        d.angle_deg = ang;
        // 选角度看**墙带是否被拉平**（band_score），不看某个门的宽窄：
        // 否则角度 0 上的一条假窄缝会把搜索钉死在 0°。
        if (!best.found || d.band_score > best.band_score ||
            (d.band_score == best.band_score && d.found && d.support > best.support)) {
            best = d;
        }
    }
    return best;
}

// ===========================================================================
// 场地范围（= 几何标定的**比例尺**来源）
// ===========================================================================
// 为什么需要它：`gap_avoid` 里的 `player_speed / edge_margin / player_radius`
// 都是**场地单位**（640x480），而视觉给的是**图像像素**。把像素直接当场地单位
// 等于假设相机正好 1:1 正对着场地 —— 实测不是（场地在 640x360 的画面里只占
// 约 560x340）。差的那 ~14% 会直接进"到达时间/边界余量"这些量里。
//
// 怎么量而不用标定：场地边框是**暖色**、而且沿 y 连续（一整列都是它），
// 所以列剖面上边框那两列比任何横墙都强得多（横墙只在一两行里出现）。
// 于是：
//     colwarm[x] = mean_y d(x,y)        // 一整列的平均暖度
//     边框 = 左右两侧**最强的两个持续峰**（要求连续若干列都超过背景 + kσ）
// 同样的做法量出行方向，得到高度的比例尺。两个比例尺应当接近（各向同性近似），
// 差得太多就说明透视/旋转明显 —— 报 `anisotropy` 交给上层决定信不信。
struct Extent {
    bool ok = false;
    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;   // 场地边框在图像里的包围盒（y 仅诊断）
    double width_px = 0.0, height_px = 0.0;
    double scale_x = 1.0, scale_y = 1.0;  // 场地单位/像素（field_w/width_px ...）
    double anisotropy = 1.0;              // scale_x / scale_y（≈1 才说明各向同性）
};

namespace detail {

//: 1-D：把所有"持续峰"按强度排序 —— 峰值位置用于定位边框，强度用于挑选。
struct Run {
    int a = 0, b = 0;      // 列的闭区间
    double peak = 0.0;     // 段内最大暖度
};

inline std::vector<Run> runs_above_background(const std::vector<double>& prof, double k,
                                              int min_run) {
    const int n = static_cast<int>(prof.size());
    std::vector<Run> out;
    if (n < 8) return out;
    std::vector<double> diffs;
    diffs.reserve(prof.size());
    for (int i = 0; i + 1 < n; ++i) diffs.push_back(prof[i + 1] - prof[i]);
    const double med = median(diffs);
    double sd = mad(diffs, med) * kMadToSigma;
    if (sd < 1.0) sd = 1.0;
    double bg = 0.0;
    for (double v : prof) bg += v;
    bg /= static_cast<double>(n);
    const double thr = bg + k * sd;
    for (int i = 0; i < n; ++i) {
        if (prof[static_cast<size_t>(i)] <= thr) continue;
        int j = i;
        double peak = prof[static_cast<size_t>(i)];
        while (j + 1 < n && prof[static_cast<size_t>(j + 1)] > thr) {
            ++j;
            if (prof[static_cast<size_t>(j)] > peak) peak = prof[static_cast<size_t>(j)];
        }
        if (j - i + 1 >= min_run) out.push_back(Run{i, j, peak});
        i = j;
    }
    return out;
}

inline std::vector<double> row_means(const uint8_t* rgb, int w, int h) {
    std::vector<double> prof(static_cast<size_t>(w), 0.0);
    for (int x = 0; x < w; ++x) {
        double s = 0.0;
        for (int y = 0; y < h; ++y) {
            const size_t i = static_cast<size_t>(y) * static_cast<size_t>(w) +
                             static_cast<size_t>(x);
            s += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
        }
        prof[static_cast<size_t>(x)] = s / static_cast<double>(h);
    }
    return prof;
}

inline std::vector<double> col_means(const uint8_t* rgb, int w, int h) {
    std::vector<double> prof(static_cast<size_t>(h), 0.0);
    for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
        double s = 0.0;
        for (int x = 0; x < w; ++x) {
            const size_t i = base + static_cast<size_t>(x);
            s += static_cast<double>(rgb[i * 3]) - static_cast<double>(rgb[i * 3 + 2]);
        }
        prof[static_cast<size_t>(y)] = s / static_cast<double>(w);
    }
    return prof;
}

}  // namespace detail

//: 从画面里量出场地边框的范围 → 场地单位/像素的比例尺（field_w/h 由调用方给）。
/// `squash`：画面纵向被压缩的比例（4:3 视野 → 16:9 流水线时 = 0.75；裁剪时 = 1.0）。
///
/// **纵向绝不能"量"**：实测场地边框的下边恒在被截断的画面上（`y1 == h-1`），
/// 于是 `oy` 与 `scale_y` 一起错，真值对照直接把门报低了 140 个场地单位
/// （真值 y≈479，板子报 340~380），闭环里就是照着错的行对齐然后擦墙。
/// 所以纵向反过来推：
///     scale_y = scale_x / squash          // 相机各向同性，压缩多少就补多少
///     oy      = (h - field_h/scale_y)/2   // 场地在画面里居中
inline Extent field_extent(const uint8_t* rgb, int w, int h, double field_w = 640.0,
                           double field_h = 480.0, double k = 5.0, int min_run = 3,
                           double squash = 0.75, double tan_a = 0.0) {
    Extent e;
    if (!rgb || w < 32 || h < 32) return e;
    (void)k;
    (void)min_run;

    // -------------------------------------------------------------------
    // 用**亮度的边缘**量场地范围，而不是"找暖色的边框列"。
    //
    // 踩过的坑（由无相机台架 test_band_gap_units_cpp.py 抓出）：仿真把场地边框
    // 画成冷色 (60,70,95)，而这里原来只找**暖色列**。于是所谓"最强的两条暖色列"
    // 其实是墙端/反光 —— 比例尺与原点全错，门的位置偏出约 200 场地单位
    // （宽度是差值，原点抵消，所以只有位置错、宽度看着还对，非常具有迷惑性）。
    //
    // 场地与场外的区别在**亮度**上（场地底色 vs 黑边），这跟配色无关：
    // 取逐列/逐行亮度均值，找两条最强的**亮度台阶**，它们就是场地的左右/上下边。
    // -------------------------------------------------------------------
    // 沿**剪切后的坐标轴**取剖面：`tan_a != 0` 时，竖直方向不再沿图像列，
    // 而是沿"去剪切"后的竖直线（每列按 shear_dy 平移）。不这么做的话，
    // 旋转过的场地用轴对齐外接框去量，宽度和原点会被一起污染
    // （实测旋转 6° 时宽度由 544 变 501，门的位置偏 150 场地单位）。
    auto luminance_profile = [&](bool vertical) {
        const int n = vertical ? w : h;
        const int m = vertical ? h : w;
        std::vector<double> prof(static_cast<size_t>(n), 0.0);
        for (int i = 0; i < n; ++i) {
            const int dshift = (vertical && tan_a != 0.0)
                ? static_cast<int>(std::lround((static_cast<double>(i) - 0.5 * static_cast<double>(w)) * tan_a))
                : 0;
            double acc = 0.0;
            for (int j = 0; j < m; ++j) {
                int x = vertical ? i : j;
                int y = vertical ? j - dshift : i;
                if (x < 0) x = 0;
                if (x >= w) x = w - 1;
                if (y < 0) y = 0;
                if (y >= h) y = h - 1;
                const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(w) +
                                   static_cast<size_t>(x);
                int l = rgb[idx * 3];
                if (rgb[idx * 3 + 1] > l) l = rgb[idx * 3 + 1];
                if (rgb[idx * 3 + 2] > l) l = rgb[idx * 3 + 2];
                acc += static_cast<double>(l);
            }
            prof[static_cast<size_t>(i)] = acc / static_cast<double>(m);
        }
        return prof;
    };
    //: 两条最强台阶的位置（台阶 = 相邻列的亮度差）。
    auto two_edges = [](const std::vector<double>& prof, int* lo, int* hi, double* step) {
        const int n = static_cast<int>(prof.size());
        if (n < 8) return false;
        double best1 = -1e18, best2 = -1e18;
        int i1 = -1, i2 = -1;
        for (int i = 1; i < n; ++i) {
            const double g = std::fabs(prof[static_cast<size_t>(i)] -
                                       prof[static_cast<size_t>(i - 1)]);
            if (g > best1) { best2 = best1; i2 = i1; best1 = g; i1 = i; }
            else if (g > best2) { best2 = g; i2 = i; }
        }
        if (i1 < 0 || i2 < 0) return false;
        const int a = i1 < i2 ? i1 : i2;
        const int b = i1 < i2 ? i2 : i1;
        if (b - a < n / 2) return false;          // 两条边必须横跨大半幅画面
        *lo = a;
        *hi = b;
        *step = best2;
        return true;
    };

    const std::vector<double> colL = luminance_profile(true);
    int xa = 0, xb = 0;
    double step_x = 0.0;
    if (!two_edges(colL, &xa, &xb, &step_x)) return e;
    e.x0 = xa;
    e.x1 = xb;
    e.width_px = static_cast<double>(e.x1 - e.x0 + 1);
    if (e.width_px < 32.0) return e;
    e.scale_x = field_w / e.width_px;

    if (squash <= 0.0) squash = 1.0;
    e.scale_y = e.scale_x / squash;

    // 纵向：优先用**亮度的行边缘**（上下边框）；量不到就按"居中等高"推。
    // 为什么不再相信"量出来的下边框"：它常常落在画面之外（被裁掉），
    // 于是 y 会整体偏 100 多场地单位。亮度边缘只在两侧都清楚时才算数。
    const std::vector<double> rowL = luminance_profile(false);
    int ya = 0, yb = 0;
    double step_y = 0.0;
    if (two_edges(rowL, &ya, &yb, &step_y) && step_y > 0.25 * step_x) {
        e.y0 = ya;
        e.y1 = yb;
        e.scale_y = field_h / static_cast<double>(e.y1 - e.y0 + 1);
    } else {
        const double field_h_px = field_h / e.scale_y;
        e.y0 = static_cast<int>(std::lround(0.5 * (static_cast<double>(h) - field_h_px)));
        e.y1 = static_cast<int>(std::lround(0.5 * (static_cast<double>(h) + field_h_px))) - 1;
    }
    if (e.y0 < 0) e.y0 = 0;
    if (e.y1 >= h) e.y1 = h - 1;
    e.height_px = static_cast<double>(e.y1 - e.y0 + 1);

    // 显著性：两条边的台阶要明显高于"列剖面自身的抖动"（噪声挡镜头过不了）
    {
        std::vector<double> dif;
        dif.reserve(colL.size());
        for (size_t i = 1; i < colL.size(); ++i) dif.push_back(colL[i] - colL[i - 1]);
        const double med = detail::median(dif);
        double sd = detail::mad(dif, med) * detail::kMadToSigma;
        if (sd < 0.5) sd = 0.5;
        if (step_x / sd < 6.0) return e;
    }

    e.anisotropy = e.scale_x > 0.0 ? e.scale_x / e.scale_y : 1.0;
    e.ok = true;
    return e;
}

}  // namespace band_gap
