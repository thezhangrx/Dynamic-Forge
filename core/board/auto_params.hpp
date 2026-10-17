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

// 刻意**不**依赖 vision_driver.hpp：这里只出数值，由调用方填进 VisionParams。
// 这样本头文件可以脱离板子驱动单独编译、单独测（`g++ -x c++ -` 就能验）。
struct Derived {
    int wall_rb = 0;
    int wall_min_r = 0;
    int wall_min_g = 0;
    int bright_min = 0;
};

namespace auto_params {

/// `d = r - b` 的取值范围：[-255, 255] → 511 个桶
constexpr int kDBins = 511;
constexpr int kDBase = 255;

struct Stats {
    int   med = 0;        // 中位数 d
    float mad = 0.0f;     // 中位数 |d - med|
    float sd  = 0.0f;     // 1.4826 * mad（稳健标准差）
    float bright_med = 0.0f;   // 亮度 L = max(r,g,b) 的中位数
    float bright_sd  = 0.0f;
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
    // 墙：比本帧背景暖 k_warm 个稳健标准差（统计显著性，不是色度绝对值）
    int wall_rb = static_cast<int>(s.med + k_warm * sd + 0.5f);
    if (wall_rb < 4) wall_rb = 4;
    if (wall_rb > 200) wall_rb = 200;
    p.wall_rb = wall_rb;
    // 亮度门限同样按本帧 L 的中位数定：只要「不比背景暗」即可（去掉固定 110）
    int wall_min_r = static_cast<int>(s.bright_med);
    if (wall_min_r < 24) wall_min_r = 24;
    p.wall_min_r = wall_min_r;
    p.wall_min_g = static_cast<int>(0.6f * static_cast<float>(wall_min_r));
    // 角色：白，比本帧亮度中位数高 k_bright 个稳健标准差
    int bright_min = static_cast<int>(s.bright_med + k_bright * sd + 0.5f);
    if (bright_min < 120) bright_min = 120;
    if (bright_min > 250) bright_min = 250;
    p.bright_min = bright_min;
    return p;
}

}  // namespace auto_params
