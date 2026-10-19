// 场地几何的**可信性判定** —— 单应解出来了，不等于它解对了。
//
// 为什么必须有这一层（实测教训）
// ------------------------------
// 四标记单应的机制是好的（268 帧语料 97.8% 能解），但"解出来了"和"解对了"是两件事。
// 实测：相机对着代码编辑器时，UI 里任意四个亮块也能凑成一组"标记"，单应有效、
// `field_bbox` 也"有效"，于是 `field_valid=true`、14 处假墙、**并且真的发出了移动指令**。
// 这类失败不报错、不自洽性检查也抓不到（4 点单应的重投影误差恒为 0），
// 所以必须用**与求解无关的约束**去否掉它：
//
//   1. 形状：场地是 4:3，流水线又做了 `squash` 纵向压缩，所以画面里场地的宽高比
//      应当是一个**可预测的常数**（契约 640x480 + squash 0.75 → 1.778）。
//      实测正常帧 1.768，误差 0.6%；UI 凑出来的四边形几乎不可能落在这个窗口里。
//   2. 尺寸：场地要占画面相当一部分（我们拍的是屏幕），太小说明锁错了目标。
//   3. 位置：四角标记应当贴近 bbox 的四角（单应把 bbox 定义成场地外框，这一点必然成立，
//      但**反过来**检查"标记离 bbox 角有多远"能抓出"bbox 被一个远离标记的点撑大"的情况）。
//   4. 尺度：由 bbox 反推的 `单位/像素` 必须落在一个物理上可能的区间里。
//
// 判据全部无量纲/相对量，没有"这一帧的亮度/曝光"这种会被 AE 带跑的常数。
#ifndef VISION_BOARD_GEOMETRY_HPP
#define VISION_BOARD_GEOMETRY_HPP

#include <cmath>
#include <cstdio>

#include "rectify.hpp"

namespace vision_board {
namespace geometry {

//: 几何来源。`Stale` 表示"这一帧没有可信几何，但上一次可用还不算太久"，
//: 由调用方决定要不要继续用（本工程的口径：超过 150 ms 就必须停）。
enum class Source { None = 0, Markers, Frame, Extent, Stale };

inline const char* source_name(Source s) {
    switch (s) {
        case Source::Markers: return "markers";
        case Source::Frame:   return "frame";
        case Source::Extent:  return "extent";
        case Source::Stale:   return "stale";
        default:              return "none";
    }
}

//: 判定参数。默认值来自契约几何 + 实测，不是逐帧拟合出来的。
struct Params {
    //: 场地宽高比的理论值 = (field_w/field_h) / squash。
    //: 640/480/0.75 = 1.7778；实测正常帧 1.768。
    double aspect_nominal = 1.7778;
    //: 允许的相对偏差（±12%）。实测正常帧偏差 0.6%，而 UI 凑出的四边形通常偏 >30%。
    //: 留这么宽是因为相机有透视：场地被斜着拍时宽高比会变，±12% 能容忍约 ±10° 的俯仰。
    double aspect_tol = 0.12;
    //: 场地在画面里的最小/最大占比（相对画面宽/高）。
    //: 下限 0.28：再小就说明相机没在看场地（实测 UI 误锁时场地"宽"只有 372/640 = 0.58，
    //: 这个下限抓不到它 —— 所以真正起作用的是宽高比 + 标记宽度一致性，见下）。
    double min_frac_w = 0.28, max_frac_w = 1.02;
    double min_frac_h = 0.28, max_frac_h = 1.02;
    //: 每个标记离 bbox 对应角的距离上限（相对 bbox 对角线）。
    //: 单应是用这四个点算出来的，它们本来就该在 bbox 的四角上；离得远说明 bbox 被撑大了。
    double max_marker_corner_frac = 0.20;
    //: 比例尺的物理区间（场地单位/像素）。0.6~4.0 对应"场地宽 160~1067 px"。
    double min_scale = 0.6, max_scale = 4.0;
};

struct Geometry {
    bool valid = false;
    Source src = Source::None;
    rectify::Homography homo;   // 像素 → 场地
    int bbox[4] = {0, 0, 0, 0}; // x0, x1, y0, y1（图像像素）
    double scale_x = 0.0, scale_y = 0.0;   // 场地单位/像素（= field/bbox 尺寸）
    double aspect = 0.0;         // bbox 宽/高
    const char* why = "none";    // 失效原因（可读字符串，直接进诊断日志）
};

//: 由四标记 + 单应判定几何是否可信。
//:
//: `markers` 用主循环里同一组（`find_markers4` 的输出），顺序由 `rectify` 决定；
//: 这里只做**可信性**判定，不重新求解。
inline Geometry judge(const rectify::Homography& homo, const std::vector<rectify::Marker>& markers,
                      int img_w, int img_h, double field_w, double field_h,
                      const Params& p = Params{}) {
    Geometry g;
    g.homo = homo;
    if (markers.size() != 4) { g.why = "no_markers"; return g; }
    if (!homo.ok) { g.why = "bad_homography"; return g; }

    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    if (!rectify::field_bbox(homo, field_w, field_h, img_w, img_h, &x0, &x1, &y0, &y1)) {
        g.why = "bad_bbox";
        return g;
    }
    g.bbox[0] = x0; g.bbox[1] = x1; g.bbox[2] = y0; g.bbox[3] = y1;
    const double bw = x1 - x0, bh = y1 - y0;
    if (bw <= 1.0 || bh <= 1.0) { g.why = "empty_bbox"; return g; }

    // 1) 宽高比（4:3 场地经纵向压缩后的理论值）
    g.aspect = bw / bh;
    const double dev = std::fabs(g.aspect - p.aspect_nominal) / p.aspect_nominal;
    if (dev > p.aspect_tol) { g.why = "bad_aspect"; return g; }

    // 2) 尺寸占比
    if (bw < p.min_frac_w * img_w || bw > p.max_frac_w * img_w ||
        bh < p.min_frac_h * img_h || bh > p.max_frac_h * img_h) {
        g.why = "bad_size";
        return g;
    }

    // 3) 标记应贴着 bbox 四角（离得远说明 bbox 被别的点撑大了）
    const double diag = std::hypot(bw, bh);
    // 顺序：**世界口径（y 朝上）**的 [左下, 右下, 右上, 左上]。
    // 场地坐标系必须和 `@STATE` / `gap_avoid` 一致（y 朝上），否则 4 点单应虽然
    // 数学上成立，下游的"前方/身后/边界余量"全部上下镜像 —— 实测表现是水平方向
    // 95% 正确、但纵向指令与几何相反，撞墙次数不随其它改进下降。
    const double cx[4] = {static_cast<double>(x0), static_cast<double>(x1),
                          static_cast<double>(x1), static_cast<double>(x0)};
    const double cy[4] = {static_cast<double>(y1), static_cast<double>(y1),
                          static_cast<double>(y0), static_cast<double>(y0)};
    for (int i = 0; i < 4; ++i) {
        const double d = std::hypot(markers[static_cast<size_t>(i)].x - cx[i],
                                    markers[static_cast<size_t>(i)].y - cy[i]);
        if (d > p.max_marker_corner_frac * diag) { g.why = "marker_far_from_corner"; return g; }
    }

    // 4) 比例尺物理区间
    const double sx = field_w / bw, sy = field_h / bh;
    if (sx < p.min_scale || sx > p.max_scale || sy < p.min_scale || sy > p.max_scale) {
        g.why = "bad_scale";
        return g;
    }
    g.scale_x = sx;
    g.scale_y = sy;
    g.valid = true;
    g.src = Source::Markers;
    g.why = "ok";
    return g;
}

}   // namespace geometry
}   // namespace vision_board

#endif   // VISION_BOARD_GEOMETRY_HPP
