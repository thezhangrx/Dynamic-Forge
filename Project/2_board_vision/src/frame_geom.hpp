// 几何新路径：**只认场地白框**（不再依赖四角标记）。
//
// 为什么换：四角小标记有三个绕不过去的问题 ——
//   1. 墙生成时会被标记遮住（缺口位置就在标记旁边时，缺口直接"消失"）；
//   2. 标记的内亮块很小，精修与排序都容易错（历史上出过上下镜像、标记离角很远）；
//   3. 标记要求场景里必须有四个"白底黑图"的方块，而平台方框本身**就是**场地边界。
//
// 白框的好处：它的四条边**就是**场地四边，四个角**就是**场地四角 (0,0)(w,0)(w,h)(0,h)，
// 解单应是"已知世界坐标的四点 → 像素四点"，不需要任何近似；而且框线比标记大一个
// 数量级，遮不住、也不怕糊。
//
// 检测思路（单帧、只扫两遍，实测 <1 ms）：
//   1. 每列的亮像素数 → 左右两条竖边（框高 ≈ 场地高，一支墙只有 7 px 厚，差一个量级）；
//   2. 在左右边之间每行的亮像素数 → 上下两条横边（框是**最外侧**满足"几乎整行都亮"的
//      那两行，墙一定在框内，所以取首/末即可）；
//   3. 可信性：四条边都够长、框的面积占比合理、框内确实比框外暗（挡住/白屏时会被否掉）。
#ifndef VISION_BOARD_FRAME_GEOM_HPP
#define VISION_BOARD_FRAME_GEOM_HPP

#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

#include "rectify.hpp"
#include "geometry.hpp"   // 复用 Geometry 结构（bbox/scale/why 同一套口径）

namespace vision_board {
namespace frame_geom {

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: 亮阈值。<=0 → 按本帧直方图自动（中位与 99.5 分位的中点，和 wall_gap 同口径）。
    int l_bright = 0;
    //: 竖边的判据：这一列亮像素 ≥ 这个比例 × 框高（框本身至少要 min_side_frac × 图高）。
    //: 竖边判据：该列亮像素 ≥ 这个比例 × 最高列。
    //: ⚠ 试过收紧到 0.80（想排除"场外余光"）：缺口 x 的准确率反而从 72% 掉到 58%，
    //: 已回退。场地边界的误检不是缺口误差的来源，别在这里调。
    double col_bright_frac = 0.55;
    //: 横边的判据：这一行（左右边之间）亮像素 ≥ 这个比例。
    double row_bright_frac = 0.80;
    //: 框的最小尺寸（相对图像）。
    double min_side_frac = 0.30;
    //: 框内平均亮度必须**低于**框外的这个比例（挡住镜头/白屏时会不满足）。
    double inside_darker_ratio = 1.05;
};

struct Frame {
    bool ok = false;
    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;   // 白框内沿（含）
    int T = 0;                            // 实际用的亮度阈值（诊断）
    int why = 0;                          // 0=ok 1=no_frame 2=too_small 3=not_darker
    const char* why_str() const {
        switch (why) {
            case 0:  return "ok";
            case 1:  return "no_frame";
            case 2:  return "frame_too_small";
            case 3:  return "inside_not_darker";
            default: return "unknown";
        }
    }
};

//: 阈值 = "场地 vs 外界"的分界，**不是**亮阈值。
//: 实测（关灯后的相机画面）：外界 luma 4~7、场地 106~144、白框只比场地亮 10~20、
//: 墙 ~128 —— 用"亮阈值"根本分不出白框；但"外界是黑的"这条极稳，所以取
//: `0.55 × 中位`（≈58）就能把整个场地从黑背景里切出来，框的位置由场地矩形的
//: 外沿给出（白框就是画在场地边上的，外沿即世界 0..640 / 0..480）。
inline int auto_bright(const uint8_t* rgb, int w, int h, int step = 2) {
    if (!rgb || w < 8 || h < 8) return 170;
    double hist[256] = {0};
    double total = 0;
    for (int y = 0; y < h; y += step) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; x += step) {
            hist[luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2])] += 1.0;
            total += 1.0;
        }
    }
    if (total <= 0) return 170;
    auto pct = [&](double q) {
        const double need = q * total;
        double acc = 0;
        for (int i = 0; i < 256; ++i) { acc += hist[i]; if (acc >= need) return i; }
        return 255;
    };
    int T = static_cast<int>(0.55 * static_cast<double>(pct(0.50)));
    if (T < 40) T = 40;                 // 全场都黑（挡住镜头）：会给 no_frame 兜住
    if (T > 120) T = 120;
    return T;
}

//: 找白框。`out` 里给的是**内沿**（检测用的就是那条亮线，稍后由调用方收缩几个像素）。
inline Frame find(const uint8_t* rgb, int w, int h, const Params& p = Params{}) {
    Frame f;
    if (!rgb || w < 64 || h < 48) { f.why = 1; return f; }
    const int T = p.l_bright > 0 ? p.l_bright : auto_bright(rgb, w, h);
    f.T = T;

    // ---- 1) 每列亮像素数：左右两条竖边 ----
    std::vector<int> col(h > 0 ? w : 0, 0);
    int max_col = 0;
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; ++x) {
            if (luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]) >= T) {
                if (++col[static_cast<size_t>(x)] > 0 && col[static_cast<size_t>(x)] > max_col)
                    max_col = col[static_cast<size_t>(x)];
            }
        }
    }
    if (max_col < static_cast<int>(p.min_side_frac * h)) { f.why = 1; return f; }
    const int col_need = static_cast<int>(p.col_bright_frac * max_col);
    int x0 = -1, x1 = -1;
    for (int x = 0; x < w; ++x) {
        if (col[static_cast<size_t>(x)] >= col_need) {
            if (x0 < 0) x0 = x;
            x1 = x;
        }
    }
    if (x0 < 0 || (x1 - x0) < static_cast<int>(p.min_side_frac * w)) { f.why = 1; return f; }

    // ---- 2) 左右边之间每行的亮像素数：上下两条横边 ----
    std::vector<int> rcnt(static_cast<size_t>(h), 0);
    for (int y = 0; y < h; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        int c = 0;
        for (int x = x0; x <= x1; ++x)
            if (luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]) >= T) ++c;
        rcnt[static_cast<size_t>(y)] = c;
    }
    const int row_need = static_cast<int>(p.row_bright_frac * (x1 - x0 + 1));
    int y0 = -1, y1 = -1;
    for (int y = 0; y < h; ++y)
        if (rcnt[static_cast<size_t>(y)] >= row_need) { y0 = y; break; }
    for (int y = h - 1; y >= 0; --y)
        if (rcnt[static_cast<size_t>(y)] >= row_need) { y1 = y; break; }
    if (y0 < 0 || y1 <= y0) { f.why = 1; return f; }
    if ((y1 - y0) < static_cast<int>(p.min_side_frac * h)) { f.why = 2; return f; }

    // ---- 3) 可信性：框内应当比框外暗（挡住镜头/白屏时框内会被照亮）----
    auto mean_in_rect = [&](int ax, int ay, int bx, int by) {
        if (bx <= ax || by <= ay) return 0.0;
        double s = 0; long n = 0;
        for (int y = ay; y < by; ++y) {
            const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = ax; x < bx; ++x) {
                s += luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
                ++n;
            }
        }
        return n ? s / static_cast<double>(n) : 0.0;
    };
    const int inset = 6;
    const double inside = mean_in_rect(x0 + inset, y0 + inset, x1 - inset, y1 - inset);
    // 框外：取框上方的窄条（相机画面里框外通常是黑的桌面）
    const double outside = mean_in_rect(x0 + inset, std::max(0, y0 - 12), x1 - inset,
                                        std::max(1, y0 - 2));
    // 框内必须**亮于**框外（外界是黑的）。反过来（框外比框内亮）说明相机没对着
    // 场地、或者有个窗口盖在上面 —— 这时宁可 no_frame（决策走 stay）。
    if (!(inside > outside * p.inside_darker_ratio)) { f.why = 3; return f; }

    f.ok = true;
    f.x0 = x0; f.x1 = x1; f.y0 = y0; f.y1 = y1;
    f.why = 0;
    return f;
}

//: 白框四角 → 单应（像素 → 场地单位，**y 朝上**）。
//: 顺序与 `main.cpp` 里标记路径一致：`[BL, BR, TR, TL]`（图像）↔ 世界左下/右下/右上/左上。
inline rectify::Homography homography(const Frame& f, double field_w, double field_h) {
    const std::array<std::array<double, 2>, 4> img = {{
        {{static_cast<double>(f.x0), static_cast<double>(f.y1)}},   // BL
        {{static_cast<double>(f.x1), static_cast<double>(f.y1)}},   // BR
        {{static_cast<double>(f.x1), static_cast<double>(f.y0)}},   // TR
        {{static_cast<double>(f.x0), static_cast<double>(f.y0)}},   // TL
    }};
    const std::array<std::array<double, 2>, 4> world = {{
        {{0.0, 0.0}}, {{field_w, 0.0}}, {{field_w, field_h}}, {{0.0, field_h}},
    }};
    return rectify::solve(img, world);
}

//: 白框 → `geometry::Geometry`（和标记路径**同一套下游口径**：bbox/scale/why）。
//: 可信性用"白框自己的"判据，不是标记那套（标记要求四点离 bbox 角很近、尺寸一致，
//: 白框路径没有标记，硬套只会误判）：
//:   · 框的宽高比要接近 (field_w/field_h)/squash = 1.778（±20%，容忍一定倾角）；
//:   · 框要占画面足够大（宽 ≥28%、高 ≥28%），又不能超过画面；
//:   · 比例尺落在物理区间（0.6~4.0 场地单位/像素）。
inline geometry::Geometry as_geometry(const Frame& f, double field_w, double field_h,
                                      const geometry::Params& gp = geometry::Params{},
                                      const rectify::Homography* homo = nullptr) {
    geometry::Geometry g;
    g.src = geometry::Source::Frame;
    if (!f.ok) { g.why = f.why_str(); return g; }
    const int bw = f.x1 - f.x0 + 1, bh = f.y1 - f.y0 + 1;
    g.bbox[0] = f.x0; g.bbox[1] = f.x1; g.bbox[2] = f.y0; g.bbox[3] = f.y1;
    g.aspect = bh > 0 ? static_cast<double>(bw) / bh : 0.0;
    g.scale_x = static_cast<double>(bw) / field_w;
    g.scale_y = static_cast<double>(bh) / field_h;
    if (homo != nullptr) g.homo = *homo;
    const double frac_w = static_cast<double>(bw) / 640.0;
    const double frac_h = static_cast<double>(bh) / 360.0;
    if (g.aspect < gp.aspect_nominal * 0.80 || g.aspect > gp.aspect_nominal * 1.20) {
        g.why = "bad_aspect_frame"; return g;
    }
    if (frac_w < gp.min_frac_w || frac_w > gp.max_frac_w ||
        frac_h < gp.min_frac_h || frac_h > gp.max_frac_h) {
        g.why = "bad_size_frame"; return g;
    }
    if (g.scale_x < gp.min_scale || g.scale_x > gp.max_scale ||
        g.scale_y < gp.min_scale || g.scale_y > gp.max_scale) {
        g.why = "bad_scale_frame"; return g;
    }
    g.valid = true;
    g.why = "ok";
    return g;
}

}   // namespace frame_geom
}   // namespace vision_board

#endif   // VISION_BOARD_FRAME_GEOM_HPP
