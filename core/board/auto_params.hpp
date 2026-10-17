// ===========================================================================
// auto_params.hpp —— 每帧从**本帧统计**推导视觉阈值（不再用固定常量）
//
// 为什么必须这样：真机位实测，橙色墙在自动曝光下被拍成淡粉白，
//     (r-b) 只剩 10~30，而 r/g/b 都被抬到 200 附近。
// 固定阈值 `(r-b)>40 && r>110` 于是有两个互相矛盾的方向：调亮 → r 够但 r-b 没了；
// 调暗 → r-b 够但 r 不够。结果同一堵墙在相邻帧之间来回翻转（实测 43% 的帧零矩形）。
//
// 判据改成「比本帧背景暖多少」：
//     d    = r - b                       // 等量偏置直接抵消
//     med  = 中位数(d)                    // 511 桶直方图，O(N) 且无排序
//     mad  = 中位数(|d - med|)
//     sd   = 1.4826 * mad                 // 稳健标准差
//     wall_rb = med + k_warm * sd         // 只比背景暖 k 个稳健标准差
// 增益与偏置都只影响 med/sd 本身，不影响「z 分数」，所以换曝光不用重调。
//
// 这不是「标定出来的常数」：k_warm 是**统计显著性**（几个 σ），
// 不是「这台相机上墙有多暖」。
// ===========================================================================
#pragma once

#include <cstdint>
#include <cstdlib>
#include <vector>

namespace auto_params {

/// 刻意**不**依赖 vision_driver.hpp：这里只出数值，由调用方填进 VisionParams。
/// 这样本头文件可以脱离板子驱动单独编译、单独测（`g++ -x c++ -` 就能验）。
struct Derived {
    int wall_rb = 0;
    int wall_min_r = 0;
    int wall_min_g = 0;
    int bright_min = 0;
    //: 诊断用：本帧的分割点（亮度谷值、亮像素内部暖/中性的分割点）
    int t_l = 0, t_d = 0;
    float bright_frac = 0.0f;   // 亮像素占比
};

/// `d = r - b` 的取值范围：[-255, 255] → 511 个桶
constexpr int kDBins = 511;
constexpr int kDBase = 255;

struct Stats {
    int   med = 0;        // 中位数 d（诊断用）
    float mad = 0.0f;     // 中位数 |d - med|
    float sd  = 0.0f;     // 1.4826 * mad（稳健标准差）
    float bright_med = 0.0f;   // 亮度 L = max(r,g,b) 的中位数
    float bright_sd  = 0.0f;
    int   t_l = 0;        // 亮度 Otsu 谷值：背景 / 亮物的分界
    int   t_d = 0;        // 亮像素内部 d 的 Otsu 分割：暖（墙）/ 中性（角色）
    float bright_frac = 0.0f;  // 亮像素占比（判"这一帧到底有没有亮物"）
    bool  ok = false;     // 像素数够不够
};

/// 一次扫描同时出 `d` 与 `L` 的稳健统计（两个 511/256 桶直方图，O(N)）。
inline Stats measure(const std::vector<uint8_t>& rgb, int pw, int ph,
                     float k_warm = 4.0f) {
    (void)k_warm;
    Stats s;
    const size_t n = static_cast<size_t>(pw) * static_cast<size_t>(ph);
    if (n < 256 || rgb.size() < n * 3) return s;

    std::vector<int> hist_d(kDBins, 0);
    std::vector<int> hist_l(256, 0);
    for (size_t i = 0; i < n; ++i) {
        const int r = rgb[i * 3 + 0], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        const int d = r - b;
        ++hist_d[d + kDBase];
        int l = r > g ? r : g;
        if (b > l) l = b;
        ++hist_l[l];
    }
    auto quantile = [](const std::vector<int>& h, size_t total, double q) {
        const double want = q * static_cast<double>(total);
        size_t acc = 0;
        for (size_t i = 0; i < h.size(); ++i) {
            acc += static_cast<size_t>(h[i]);
            if (static_cast<double>(acc) >= want) return static_cast<int>(i);
        }
        return static_cast<int>(h.size()) - 1;
    };
    s.med = quantile(hist_d, n, 0.5) - kDBase;
    s.bright_med = static_cast<float>(quantile(hist_l, n, 0.5));

    // MAD：把 |d - med| 也做一次直方图（区间 [0, 255] 足够，d 只有 8 bit）
    std::vector<int> hist_a(256, 0);
    for (size_t i = 0; i < n; ++i) {
        int dd = static_cast<int>(rgb[i * 3 + 0]) - static_cast<int>(rgb[i * 3 + 2]) - s.med;
        if (dd < 0) dd = -dd;
        if (dd > 255) dd = 255;
        ++hist_a[dd];
    }
    // ---- ② 亮度 Otsu 谷值（背景 vs 亮物）-------------------------------
    // 为什么用 Otsu 而不是「中位数 + kσ」：这台相机的色偏让**背景本身**的 r-b
    // 就已经是 +24±19，而墙只有 +10~+30 —— 两个分布**重叠**，任何「比背景暖几 σ」
    // 的判据都会把墙排除掉（实测自动阈值 wall_rb=101，一帧都检不出）。
    // 亮的墙和亮的角色才是"另一类"，所以第一刀必须切在**亮度**上。
    auto otsu = [](const std::vector<int>& h) {
        const int nb = static_cast<int>(h.size());
        double total = 0, sum = 0;
        for (int i = 0; i < nb; ++i) { total += h[i]; sum += static_cast<double>(i) * h[i]; }
        if (total <= 0) return 0;
        double w0 = 0, s0 = 0, best = -1.0;
        int best_t = 0;
        for (int i = 0; i < nb; ++i) {
            w0 += h[i];
            if (w0 <= 0) continue;
            s0 += static_cast<double>(i) * h[i];
            const double w1 = total - w0;
            if (w1 <= 0) break;
            const double m0 = s0 / w0, m1 = (sum - s0) / w1;
            const double v = w0 * w1 * (m0 - m1) * (m0 - m1);
            if (v > best) { best = v; best_t = i; }
        }
        return best_t;
    };
    s.t_l = otsu(hist_l);

    // ---- ③ 亮像素内部的暖/中性分割（墙 vs 角色）-------------------------
    std::vector<int> hist_dbright(kDBins, 0);
    size_t n_bright = 0;
    for (size_t i = 0; i < n; ++i) {
        const int r = rgb[i * 3 + 0], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        int l = r > g ? r : g;
        if (b > l) l = b;
        if (l >= s.t_l) { ++hist_dbright[r - b + kDBase]; ++n_bright; }
    }
    s.bright_frac = static_cast<float>(n_bright) / static_cast<float>(n);
    s.t_d = otsu(hist_dbright) - kDBase;

    s.mad = static_cast<float>(quantile(hist_a, n, 0.5));
    // ⚠ 合成图/干净图上 MAD 可以是 0（只有几种颜色，中位绝对偏差为 0），
    // 这时 med + kσ 会退化成 med，墙阈值塌到地板 → 什么都被判成墙。
    // 所以 σ 还必须**不低于本帧实测的传感器噪声**：用水平相邻像素差分估
    // （缓慢的光照/色温梯度被差分消掉，剩下的才是噪声）。RTL 侧不需要知道这件事。
    float noise = 0.0f;
    {
        std::vector<int> hist_noise(256, 0);
        size_t cnt = 0;
        for (int y = 0; y < ph; ++y) {
            const size_t row = static_cast<size_t>(y) * static_cast<size_t>(pw);
            for (int x = 1; x < pw; ++x) {
                const size_t i = row + static_cast<size_t>(x);
                int dd = (static_cast<int>(rgb[i * 3 + 0]) - static_cast<int>(rgb[i * 3 + 2]))
                       - (static_cast<int>(rgb[(i - 1) * 3 + 0]) - static_cast<int>(rgb[(i - 1) * 3 + 2]));
                if (dd < 0) dd = -dd;
                if (dd > 255) dd = 255;
                ++hist_noise[dd];
                ++cnt;
            }
        }
        if (cnt > 0) {
            // 差分噪声的 MAD 再除以 √2（两个独立样本之差）
            const float mad_n = static_cast<float>(quantile(hist_noise, cnt, 0.5));
            noise = 1.4826f * mad_n / 1.4142f;
        }
    }
    s.sd = 1.4826f * s.mad;
    if (noise > s.sd) s.sd = noise;
    s.ok = true;
    return s;
}

/// 统计 → 寄存器值。全部**由本帧数据决定**，没有任何「这台相机」的常数。
inline Derived derive(const Stats& s, float k_warm = 4.0f, float k_bright = 5.0f) {
    Derived p;
    const float sd = s.sd > 0.5f ? s.sd : 0.5f;   // 量化下限：σ 至少 0.5 个计数
    if (!s.ok) return p;

    // 亮像素太少 = 这一帧没有亮物（镜盖挡着 / 全黑）→ 回到稳健 z 的保守值，
    // 让下游"看不见就沿上一条指令"，而不是把噪声当墙。
    if (s.bright_frac < 0.02f) {
        int wrb = static_cast<int>(s.med + k_warm * sd + 0.5f);
        if (wrb < 4) wrb = 4;
        if (wrb > 200) wrb = 200;
        p.wall_rb = wrb;
        int wmr = static_cast<int>(s.bright_med);
        if (wmr < 24) wmr = 24;
        p.wall_min_r = wmr;
        p.wall_min_g = static_cast<int>(0.6f * static_cast<float>(wmr));
        int bm = static_cast<int>(s.bright_med + k_bright * sd + 0.5f);
        if (bm < 120) bm = 120;
        if (bm > 250) bm = 250;
        p.bright_min = bm;
        p.t_l = s.t_l;
        p.t_d = s.t_d;
        p.bright_frac = s.bright_frac;
        return p;
    }

    // 墙 = **亮且偏暖**：亮度过 Otsu 谷（t_l）且 d 过亮像素内部的 Otsu 分割（t_d）。
    // 两个分割点都是本帧直方图的谷，不含任何"这台相机"的常数。
    int wall_rb = s.t_d;
    if (wall_rb < 2) wall_rb = 2;
    if (wall_rb > 200) wall_rb = 200;
    p.wall_rb = wall_rb;
    int t_l = s.t_l;
    if (t_l < 16) t_l = 16;
    p.wall_min_r = t_l;
    p.wall_min_g = static_cast<int>(0.6f * static_cast<float>(t_l));
    // 角色 = 亮但**中性**（由 bright_spread 那一条区分），亮度门限同为 t_l
    int bright_min = t_l;
    if (bright_min < 100) bright_min = 100;
    if (bright_min > 250) bright_min = 250;
    p.bright_min = bright_min;
    p.t_l = s.t_l;
    p.t_d = s.t_d;
    p.bright_frac = s.bright_frac;
    return p;
}

}  // namespace auto_params
