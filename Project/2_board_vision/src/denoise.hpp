// ===========================================================================
// denoise.hpp —— 相机彩噪的**可分离方框滤波**（推像素前做，PS 侧）
//
// 为什么需要：真机位实测相机稳态输出有很强的**彩噪**（相邻像素差中位 17~19，
// 干净基准 2.0）。判据用的是 `d = r-b` 这类**色度**量，彩噪直接把色度淹掉：
// 同一帧上去噪前只能找到 3 个标记（且位置不对），方框滤波 r=1 后**4 个全对**。
//
// 为什么用方框滤波：它是**线性、无参数、无边缘假设**的最小干预；半径是要显式给的
// 物理量（像素），不是"调出来的阈值"。可分离实现每像素 2r+1 次加法 ×2 遍，
// 板子上 640x360 r=1 约十几毫秒。
// ===========================================================================
#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace denoise {

inline void box_blur_channel(std::vector<uint8_t>& c, int w, int h, int r,
                             std::vector<int>& tmp) {
    if (r <= 0) return;
    // 滑动窗口求均值：每像素 O(1)（不是每像素 2r+1 次加法）。
    // 之前逐像素重算窗口，板子上 r=1 就吃掉约 40 ms/帧（帧率 10.7 → 5.9）。
    const int win = 2 * r + 1;
    tmp.resize(static_cast<size_t>(w) * static_cast<size_t>(h));
    for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * static_cast<size_t>(w);
        int sum = 0;
        for (int k = -r; k <= r; ++k) {
            const int xx = std::min(std::max(k, 0), w - 1);
            sum += c[base + static_cast<size_t>(xx)];
        }
        for (int x = 0; x < w; ++x) {
            tmp[base + static_cast<size_t>(x)] = sum / win;
            const int out_x = std::min(std::max(x - r, 0), w - 1);
            const int in_x = std::min(std::max(x + r + 1, 0), w - 1);
            sum += static_cast<int>(c[base + static_cast<size_t>(in_x)])
                 - static_cast<int>(c[base + static_cast<size_t>(out_x)]);
        }
    }
    for (int x = 0; x < w; ++x) {
        int sum = 0;
        for (int k = -r; k <= r; ++k) {
            const int yy = std::min(std::max(k, 0), h - 1);
            sum += tmp[static_cast<size_t>(yy) * static_cast<size_t>(w) + static_cast<size_t>(x)];
        }
        for (int y = 0; y < h; ++y) {
            c[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)] =
                static_cast<uint8_t>(sum / win);
            const int out_y = std::min(std::max(y - r, 0), h - 1);
            const int in_y = std::min(std::max(y + r + 1, 0), h - 1);
            sum += tmp[static_cast<size_t>(in_y) * static_cast<size_t>(w) + static_cast<size_t>(x)]
                 - tmp[static_cast<size_t>(out_y) * static_cast<size_t>(w) + static_cast<size_t>(x)];
        }
    }
}

/// 对交错 RGB 缓冲做三通道方框滤波（原地）。`r` 是半径（像素）。
inline void box_blur_rgb(std::vector<uint8_t>& rgb, int w, int h, int r) {
    if (r <= 0 || w <= 0 || h <= 0) return;
    const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
    if (rgb.size() < n * 3) return;
    std::vector<uint8_t> ch(n);
    static std::vector<int> tmp;      // 复用：每帧都 new 230k 个 int 会明显拖慢
    for (int c = 0; c < 3; ++c) {
        for (size_t i = 0; i < n; ++i) ch[i] = rgb[i * 3 + static_cast<size_t>(c)];
        box_blur_channel(ch, w, h, r, tmp);
        for (size_t i = 0; i < n; ++i) rgb[i * 3 + static_cast<size_t>(c)] = ch[i];
    }
}

}  // namespace denoise
