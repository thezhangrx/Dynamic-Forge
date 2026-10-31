// 标记中心精修 —— 单应准不准，几乎全看这一步。
//
// 为什么需要
// ----------
// 契约里标记的坐标是**方块中心** `(32,32)` 这类点，但检测器给出的是
// "亮内部图案"的质心。仿真画的标记是"深方块 + 亮内图"，内图不是对称的
// （本来就是给编码用的），所以它的质心**偏离方块中心几个像素**。
//
// 实测后果：单应整体平移了约 8~15 px —— 场地宽 550 px，也就是约 1.5%~3% 的比例，
// 折算成场地单位是 10~17。这个量级正好吃掉 P2 的验收预算（≤20 场地单位），
// 表现为"角色检测器明明看对了，误差却有 16 px"。
//
// 做法：以检测到的亮块为中心，向外找**包住它的那个深色方块**，用方块的
// 包围盒中心当标记中心。方块是矩形且与场地网格对齐，包围盒中心比"内部图案质心"
// 稳得多（不受图案不对称影响），也不需要任何编码知识。
#ifndef VISION_BOARD_MARKER_REFINE_HPP
#define VISION_BOARD_MARKER_REFINE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "rectify.hpp"

namespace vision_board {
namespace markers {

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: 搜索窗口相对标记边长的倍数。方块比内图大，2.0 足够（实测 24 vs 14 像素）。
    double window_k = 2.0;
    //: **契约尺寸窗口**（像素）：标记方块边长 = 48 场地单位，换算成像素约等于
    //: 场地宽度像素 × 48/640。调用方用"未精修的四标记跨度"估出这个尺度传进来。
    //: ⚠ 没有这个约束会出事：实测精修会吞掉画面边缘的**大片暗区**（背景/边框），
    //: 把标记中心拽到 (81,6.7)、尺寸 186×161，接着"离标记 2×边长"的排除半径
    //: 覆盖半个画面，把正确的角色候选全否掉（表现为"漏检 7 帧"）。
    //: 传 0 表示不做该方向的约束（仅调试用）。
    int min_side_px = 0;
    int max_side_px = 0;
    //: 深色阈值：窗口内 (p10+p50)/2，与曝光无关。
    //: 取两个分位的中间，是因为窗口里一定同时有"深方块"和"亮内图"两类像素。
    bool auto_dark = true;
    int l_dark = 0;
};

//: 把标记中心精修到"包住它的深色方块"的中心。失败（窗口里没有像样的深块）时
//: 原样返回，并由 `*refined` 告知调用方 —— 失败不报错、不改变几何，只是回到旧行为。
inline rectify::Marker refine(const uint8_t* rgb, int w, int h,
                              const rectify::Marker& m, bool* refined = nullptr,
                              const Params& p = Params{}) {
    if (refined) *refined = false;
    if (!rgb || w < 32 || h < 32) return m;
    const int side = std::max(4, std::max(m.w, m.h));
    const int half = static_cast<int>(p.window_k * side) + 2;
    const int cx = static_cast<int>(std::lround(m.x));
    const int cy = static_cast<int>(std::lround(m.y));
    const int x0 = std::max(0, cx - half), x1 = std::min(w - 1, cx + half);
    const int y0 = std::max(0, cy - half), y1 = std::min(h - 1, cy + half);
    if (x1 - x0 < 6 || y1 - y0 < 6) return m;

    // 窗口内的深色阈值
    int T = p.l_dark;
    if (p.auto_dark || T <= 0) {
        double hist[256];
        for (int i = 0; i < 256; ++i) hist[i] = 0;
        double total = 0;
        for (int y = y0; y <= y1; ++y) {
            const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = x0; x <= x1; ++x) {
                hist[luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2])] += 1.0;
                total += 1.0;
            }
        }
        auto pct = [&](double q) {
            const double need = q * total;
            double acc = 0;
            for (int i = 0; i < 256; ++i) { acc += hist[i]; if (acc >= need) return i; }
            return 255;
        };
        if (total <= 0) return m;
        T = (pct(0.10) + pct(0.50)) / 2;
        T = std::max(1, std::min(250, T));
    }

    // 以"离标记中心最近的那块深色连通域"为准：
    // 用两遍扫描 + 并查集在**小窗口**上做，成本可忽略（窗口一般 < 60×60）。
    const int ww = x1 - x0 + 1, hh = y1 - y0 + 1;
    std::vector<uint8_t> mask(static_cast<size_t>(ww) * hh, 0);
    for (int y = 0; y < hh; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y + y0) * w * 3;
        for (int x = 0; x < ww; ++x) {
            if (luma(row[(x + x0) * 3], row[(x + x0) * 3 + 1], row[(x + x0) * 3 + 2]) <= T) {
                mask[static_cast<size_t>(y) * ww + x] = 1;
            }
        }
    }
    std::vector<int> label(static_cast<size_t>(ww) * hh, 0);
    std::vector<int> stack;
    int next = 1;
    int best = 0, best_px = 0;
    int bx0 = 0, bx1 = -1, by0 = 0, by1 = -1;
    for (int y = 0; y < hh; ++y) {
        for (int x = 0; x < ww; ++x) {
            const size_t idx = static_cast<size_t>(y) * ww + x;
            if (!mask[idx] || label[idx]) continue;
            stack.assign(1, static_cast<int>(idx));
            label[idx] = next;
            int px = 0, mnx = x, mxx = x, mny = y, mxy = y;
            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                const int cyy = cur / ww, cxx = cur % ww;
                ++px;
                mnx = std::min(mnx, cxx); mxx = std::max(mxx, cxx);
                mny = std::min(mny, cyy); mxy = std::max(mxy, cyy);
                const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = cxx + dx[k], ny = cyy + dy[k];
                    if (nx < 0 || ny < 0 || nx >= ww || ny >= hh) continue;
                    const size_t ni = static_cast<size_t>(ny) * ww + nx;
                    if (mask[ni] && !label[ni]) { label[ni] = next; stack.push_back(static_cast<int>(ni)); }
                }
            }
            ++next;
            // 只要"够大、且包住标记中心"的块
            const double bcx = 0.5 * (mnx + mxx) + x0;
            const double bcy = 0.5 * (mny + mxy) + y0;
            const bool contains = (cx >= mnx + x0 - 1 && cx <= mxx + x0 + 1 &&
                                   cy >= mny + y0 - 1 && cy <= mxy + y0 + 1);
            const int bw = mxx - mnx + 1, bh = mxy - mny + 1;
            bool plausible = true;
            if (p.min_side_px > 0) plausible = plausible && bw >= p.min_side_px && bh >= p.min_side_px;
            if (p.max_side_px > 0) plausible = plausible && bw <= p.max_side_px && bh <= p.max_side_px;
            if (p.min_side_px <= 0 && p.max_side_px <= 0) {
                // 没给契约尺寸时的保守退化：只接受"和亮块同量级"的方块
                plausible = bw >= side && bh >= side && bw <= 3 * side && bh <= 3 * side;
            }
            if (!contains || !plausible) continue;
            if (px > best_px) {
                best_px = px;
                best = next - 1;
                bx0 = mnx + x0; bx1 = mxx + x0; by0 = mny + y0; by1 = mxy + y0;
                (void)bcx; (void)bcy;
            }
        }
    }
    if (best_px <= 0) return m;
    rectify::Marker out = m;
    out.x = 0.5 * (bx0 + bx1);
    out.y = 0.5 * (by0 + by1);
    out.w = bx1 - bx0 + 1;
    out.h = by1 - by0 + 1;
    if (refined) *refined = true;
    return out;
}

}   // namespace markers
}   // namespace vision_board

#endif   // VISION_BOARD_MARKER_REFINE_HPP
