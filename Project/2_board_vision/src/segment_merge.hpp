// ===========================================================================
// segment_merge.hpp —— 把"一堵墙被切成好几段"在**矩形层**合回一段
//
// 参考（`docs/repo/fpga/` 下两个工程 + OpenCV）：
//   * OpenCV 形态学教程（`MORPH_CLOSE` = 先膨胀后腐蚀，"去掉小洞"）—— 掩膜层
//     的修复已经落在 `core/vision/mask_repair.py`（PC 参考实现，将来进 RTL）。
//     这里是它的**矩形层近似**：一堵墙被切成 N 段细长条时，把共线、间隔很小的
//     相邻段并回一段。好处是**不用重编位流**，板侧后处理就能生效；
//   * `fpga-edge-detection-accelerator` 的验收方式：先有 PC 参考语义，再谈硬件。
//     本头文件的判据与 `core/vision/mask_repair.merge_segments` 逐条对应
//     （共线 + 间隔上限 + 取较长段几何），并有 C++ 自测钉住。
//
// ⚠ **绝不许把真缺口封死**：`max_gap` 必须明显小于真实通道宽度。实测（真板子、
//   相机拍屏幕、pipeline 坐标）真缺口宽 ~58 px，默认取 6 px —— 10 倍余量。
// ===========================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "gap_avoid.hpp"

namespace segment_merge {

/// 合并参数（板侧后处理，单位 = pipeline 像素 / 场地单位，与 Rect 一致）。
struct Params {
    //: 允许跨越的最大横向空隙（<= 这个值才并）。
    double max_gap = 6.0;
    //: 允许的最大纵向差（两个段要算"共线"）。
    double max_dy = 4.0;
    //: 合并后至少要比原来长多少才值得并（0 = 只要够近就并）。
    double min_gain = 0.0;
};

/// 是否把 `a`（左）与 `b`（右，x 更大）并成一段。
inline bool can_merge(const gap_avoid::Rect& a, const gap_avoid::Rect& b,
                      const Params& p) {
    const double dy = std::fabs(b.y - a.y);
    if (!(dy <= p.max_dy)) return false;
    // 段的横向范围：中心 ± half_w
    const double a_right = a.x + a.half_w;
    const double b_left = b.x - b.half_w;
    const double gap = b_left - a_right;
    if (!(gap <= p.max_gap)) return false;          // NaN 也走这里（比较为假）
    if (gap < 0.0) return true;                     // 重叠：一定并
    const double a_len = 2.0 * a.half_w, b_len = 2.0 * b.half_w;
    return (a_len + b_len) > p.min_gain;
}

/// 合并两个矩形：横向取并集、纵向/厚度/朝向取**较长那一段**的
/// （合并本来就是为了"补断"，不该让碎段主导几何；速度也一样，碎段的速度是噪声）。
inline gap_avoid::Rect merge_two(const gap_avoid::Rect& a, const gap_avoid::Rect& b) {
    const gap_avoid::Rect& longer = (a.half_w >= b.half_w) ? a : b;
    const double left = std::min(a.x - a.half_w, b.x - b.half_w);
    const double right = std::max(a.x + a.half_w, b.x + b.half_w);
    gap_avoid::Rect out = longer;
    out.x = 0.5 * (left + right);
    out.half_w = 0.5 * (right - left);
    return out;
}

/// 得到合并结果 + 实际合并了几次（合并次数写进日志，方便"到底生效没有"）。
inline std::vector<gap_avoid::Rect> merge_collinear(std::vector<gap_avoid::Rect> rects,
                                                    const Params& p, int* n_merged) {
    if (n_merged) *n_merged = 0;
    std::sort(rects.begin(), rects.end(), [](const gap_avoid::Rect& l,
                                             const gap_avoid::Rect& r) {
        return (l.x - l.half_w) < (r.x - r.half_w);
    });
    std::vector<gap_avoid::Rect> out;
    out.reserve(rects.size());
    for (const gap_avoid::Rect& r : rects) {
        if (!out.empty() && can_merge(out.back(), r, p)) {
            out.back() = merge_two(out.back(), r);
            if (n_merged) ++(*n_merged);
            continue;
        }
        out.push_back(r);
    }
    return out;
}

}  // namespace segment_merge
