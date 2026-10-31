// 静态帧的"墙 + 缺口"识别 —— 不做任何位置先验，只看这一帧的亮度结构。
//
// 为什么单独写这一层
// ------------------
// `band_gap.hpp` 那条路是"先假设场地边框在哪，再用行带/匹配滤波去验证"。它的
// 前提一旦不成立（场地只占画面一小块、画面有透视、有噪点），给出的就是**自洽但
// 错的数**，而且不报错。这一层反过来，只用三条和场景无关的事实：
//
//   1. 墙是**横条**：一整行里亮像素占比很高（角色是圆点、标记是方块，都占不满一行）
//   2. 墙是**连通的一条**：同一行带里，亮列是连续的一段（缺口把它断开）
//   3. 缺口是墙上的**中断**：墙的行带里，某一列区间几乎不亮
//
// 于是识别过程是：先按行找"亮像素很多"的行 → 聚成行带（就是墙）→ 在行带里按列
// 找亮/不亮的分布 → 连续不亮的内部区间就是缺口。全程没有"场地在哪""缺口应该多宽"
// 这种假设，量出来的就是这一帧真实的样子。
//
// 为什么这比匹配滤波稳：它对**行方向**是积分（噪声互相抵消），只有列方向要求
// "这一列在整条墙的厚度上都亮"，噪声很难在 8~20 行上同时骗过 50% 的判据。
//
// 单位：全部是**像素**。换成场地单位由调用方按 `band_gap::Extent` 的 scale_x/scale_y 做。
#ifndef VISION_BOARD_WALL_GAP_HPP
#define VISION_BOARD_WALL_GAP_HPP

#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cmath>

#include "band_gap.hpp"   // Door 是 band_gap 里的结构（去重/选墙都用它）

namespace vision_board {
namespace wall_gap {

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: 亮像素阈值。<=0 表示按本帧直方图自动定（见 `auto_bright`）。
    //: 实测（板子拍仿真窗口，saturation=60/sharpness=0）：场地 L≈110~150，
    //: 墙 L≈180~215，角色 L≈195~215，背景 L≈20~90。
    int l_bright = 0;
    //: 一行里亮像素占比 ≥ 这个值，就算"墙所在的候选行"。
    //: 取 0.20：一道横墙至少横跨 40% 画面宽度；角色圆点只占 3%~10%。
    double row_bright_frac = 0.20;
    //: 列判据：在该行带里，某一列"亮"的行数占比 ≥ 这个值 → 这一列是墙。
    double col_bright_frac = 0.55;
    //: 列的"不亮"判据（占比 ≤ 这个值）→ 这一列是缺口。
    double col_gap_frac = 0.25;
    //: 行带最少几行才算墙。**这条必须够严**：实测场地网格线/边框只有 2~3 px 厚，
    //: 却同样满足"整行很多亮像素"，于是被当成墙 —— 再经"按 y 排序取第一扇"，
    //: 板子就锁到了画面顶部那条网格线（实测墙 y 偏差 180 场地单位，缺口自然全错）。
    //: 真墙在 640x360 下 8~20 px 厚，取 5 既排除网格线又留出余量。
    int min_band_rows = 5;
    //: 行带最多几行（更厚的是窗口边缘/桌面亮块，不是墙）。
    int max_band_rows = 60;
    //: 缺口最小/最大宽度（像素）。太窄是噪声，太宽说明这一行带根本不是墙。
    int min_gap_px = 8;
    int max_gap_px = 240;
    //: 墙的 x 跨度至少要占画面宽度这个比例（否则是局部的亮块，不是墙）。
    double min_span_frac = 0.25;
    //: 行带内"亮列占比"下限：真墙是一条连续亮条（实测 0.9~1.0），网格线 0.5 左右。
    double min_bright_frac = 0.72;
    //: **VT/标注用**：没有缺口也保留这条行带（默认 false = 只保留"带缺口的墙"）。
    //: 相机斜拍/墙被洗白时，部分墙的缺口判据会失败；但"同时看到多道墙"需要把它们
    //: 都保留下来，VT 渲染/决策再按 gap 是否已知分别处理。
    bool keep_no_gap = false;
    //: **VT/标注用**：把所有满足"整行亮"的行带都当成墙输出（不要求 gap/跨度/亮列占比）。
    //: 相机斜拍时部分墙的缺口判据失败，但"同时看到多道墙"需要保留全部行带。
    bool all_bands = false;

    // ---- 颜色门（P2）------------------------------------------------------
    //: 墙像素的"暖度"下限：``r - b >= min_warmth``。**只加这一个门**，就够把
    //: 网格线/外框/标记/角色全部排掉 —— 唯一依据是**墙是暖色**：
    //:
    //:   墙（`WALL_COLOR` (255,120,0)）：相机里 r−b 实测 118~200
    //:   网格线 / 场地边框 / 四角标记 / 角色：都是中性（r−b ≈ 0~15，角色偏粉 ~57）
    //:
    //: 为什么不用亮度门：亮度**随曝光整体漂**（自动曝光第 3~4 帧会跳一档），
    //: 而"暖度"是色差、与亮度近似无关；实测空场地上亮度判据 100% 的帧都报出
    //: 两道"墙"（网格线），加了这道门之后为 0。
    //:
    //: 兜底：整幅暖像素太少（< 0.2%）时**退回亮度判据** —— 万一以后把墙改成中性色，
    //: 这里不会静默变成"永远没有墙"，而是保持老行为（日志里 `wall_px` 会掉到 0）。
    //: **场地掩膜**：只在 [x0,x1]×[y0,y1] 里找墙/缺口；场外（黑色无边界区）
    //: 一律当"非墙、非缺口"。为什么必须有：场地没有边界线，墙带一贴到画面边缘，
    //: 场外黑区就被当成"墙上的缺口"（实测缺口中心被拉到场地左边缘 x=98，
    //: 而真实缺口在别处）→ 决策把角色往平台外带。
    int clip_x0 = 0, clip_x1 = 0, clip_y0 = 0, clip_y1 = 0;   // x1<=x0 表示不裁剪
    //: 缺口两端离场地边缘至少留这么多像素（防止"贴边黑区"冒充缺口）
    int gap_edge_margin = 6;

    int min_warmth = 0;   // 结构门接管后默认关（相机把橙色拉成白灰，色相不可靠）
    //: 整幅画面里"墙像素"少于这个比例就认为"这一帧没有暖色墙"，退回亮度判据。
    double warm_fallback_frac = 0.002;
};

//: 墙像素判据：**暖**（r−b 大）。不加亮度条件：暖度本身已经足够区分，
//: 再叠亮度只会把"墙在阴影里/过曝"两种极端重新变成漏检。
inline bool is_wall_pixel(int r, int g, int b, int min_warmth) {
    (void)g;
    return (r - b) >= min_warmth;
}

//: 一帧里的一道墙（含它的缺口）。
struct Wall {
    int y0 = 0, y1 = 0;          // 行带（含端点）
    int x0 = 0, x1 = 0;          // 墙的 x 跨度（含端点）
    double cy = 0;               // 行带中心（像素）
    double bright_frac = 0;      // 行带里亮的列占比（诊断用）
    bool has_gap = false;
    int gap_x0 = 0, gap_x1 = 0;  // 缺口（含端点），has_gap 为假时无意义
    double gap_cx = 0;           // 缺口中心
    int gap_w = 0;
    //: 缺口两侧的墙段跨度（决策要知道"往哪边挪"）。
    int left_x0 = 0, left_x1 = -1;
    int right_x0 = -1, right_x1 = 0;
    int n_gaps = 0;              // 找到的缺口个数（可能 1 个以上）
    int T_local = 0;             // 行带内实际用的亮度阈值（诊断）
    int cnt_need = 0;            // 判"这列是墙"所需的亮行数（诊断）
};

//: 自动亮阈值：**中位与高分的中间**，不靠"找谷底"。
//:
//: 踩过的坑：一开始用"从直方图主峰往右找谷底"，结果在整幅画面的帧上主峰落在了
//: 抗锯齿/黑边那一档（mode≈5），谷底被判到 8 —— 阈值=9，于是**整幅画面都是"亮"**，
//: 检出来的"墙"是屏幕边框。这类自动阈值一旦被一个占面积很大的暗区带偏，
//: 后面全错而且不报错，所以换成"分位数"：中位（场地/背景的水平）与 99.5 分位
//: （亮物体）取中点。场地占多数时中位就是场地，亮物体再少也抬得动 99.5 分位。
//:
//: 两个夹紧条件保证它不会退化成极端值：
//:   · 至少比中位高 20（否则"亮"和"场地"没有可分性，硬分只会分出一堆噪声）
//:   · 至少比 99.5 分位低 8（否则一个亮点都进不来）
inline int auto_bright(const uint8_t* rgb, int w, int h, int step = 2) {
    if (!rgb || w < 8 || h < 8) return 170;
    if (step < 1) step = 1;
    double hist[256];
    for (int i = 0; i < 256; ++i) hist[i] = 0;
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
        for (int i = 0; i < 256; ++i) {
            acc += hist[i];
            if (acc >= need) return i;
        }
        return 255;
    };
    const int med = pct(0.50);
    const int hi = pct(0.995);
    int T = (med + hi) / 2;
    if (T < med + 20) T = med + 20;
    if (T > hi - 8) T = hi - 8;
    if (T < 1) T = 1;
    if (T > 254) T = 254;
    return T;
}

//: 找这一帧里所有的墙和缺口（按 y 从小到大）。
inline std::vector<Wall> find_walls(const uint8_t* rgb, int w, int h, const Params& p = Params{},
                                    int* out_lbright = nullptr) {
    std::vector<Wall> out;
    if (!rgb || w < 32 || h < 32) return out;
    const int T = p.l_bright > 0 ? p.l_bright : auto_bright(rgb, w, h);
    if (out_lbright) *out_lbright = T;

    // ---- 0) 颜色门：整幅暖像素够不够多？不够就退回亮度判据（见 Params::min_warmth）----
    long n_warm = 0;
    if (p.min_warmth > 0) {
        for (int y = 0; y < h; y += 2) {
            const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = 0; x < w; x += 2) {
                if (is_wall_pixel(row[x * 3], row[x * 3 + 1], row[x * 3 + 2], p.min_warmth))
                    n_warm += 4;      // 每 2x2 采样点代表 4 个像素
            }
        }
    }
    const bool use_colour = (p.min_warmth > 0) &&
                            (n_warm >= p.warm_fallback_frac * static_cast<double>(w) * h);
    const int min_warmth = use_colour ? p.min_warmth : 0;

    // ---- 1) 每行的"墙像素"个数 ----
    // 颜色门：计数的是**暖**像素；回退：计数的是**亮**像素（老行为）。
    std::vector<int> row_cnt(static_cast<size_t>(h), 0);
    const bool clipped = (p.clip_x1 > p.clip_x0);
    const int cx0 = clipped ? std::max(0, p.clip_x0) : 0;
    const int cx1 = clipped ? std::min(w - 1, p.clip_x1) : w - 1;
    const int cy0 = clipped ? std::max(0, p.clip_y0) : 0;
    const int cy1 = clipped ? std::min(h - 1, p.clip_y1) : h - 1;
    for (int y = cy0; y <= cy1; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        int c = 0;
        for (int x = cx0; x <= cx1; ++x) {
            const int r = row[x * 3], g = row[x * 3 + 1], b = row[x * 3 + 2];
            if (min_warmth > 0 ? is_wall_pixel(r, g, b, min_warmth) : (luma(r, g, b) >= T)) ++c;
        }
        row_cnt[static_cast<size_t>(y)] = c;
    }

    // ---- 2) 聚成行带 ----
    const double row_need = p.row_bright_frac * w;
    std::vector<std::pair<int, int>> bands;   // [y0, y1]
    int y = 0;
    while (y < h) {
        if (row_cnt[static_cast<size_t>(y)] >= row_need) {
            const int y0 = y;
            while (y < h && row_cnt[static_cast<size_t>(y)] >= row_need) ++y;
            bands.emplace_back(y0, y - 1);
        } else {
            ++y;
        }
    }

    // ---- 3) 行带内按列判"墙/缺口" ----
    //
    // ⚠ 这里**必须用"列内最大值"而不是"列内均值"**。踩过的坑：行带往往比墙本身
    // 高（墙的亮边/光晕会把上下相邻几行也顶过阈值），用均值会被场地那一半稀释 ——
    // 墙的列只有 120 左右、缺口 70 左右，两者挨得太近，缺口右边界因此少算 20 多像素。
    // 换成"列内最大值"：墙的列 ≈ 205、缺口的列 ≈ 90（场地水平），差得很开。
    for (const auto& band : bands) {
        Wall wl;
        wl.y0 = band.first;
        wl.y1 = band.second;
        const int rows = wl.y1 - wl.y0 + 1;
        if (rows < p.min_band_rows || rows > p.max_band_rows) continue;
        wl.cy = 0.5 * (wl.y0 + wl.y1);

        // 每列在行带里有几行是"墙那么亮"。
        //
        // 为什么用**计数**而不是列均值/列最大值：
        //   · 列均值被行带里的空白行稀释（墙只占行带的一部分行），墙和缺口拉不开；
        //   · 列最大值被**单个噪声像素**打穿 —— 缺口里一个亮点就等于"这列是墙"，
        //     于是缺口被撕成好几段（实测间隙被切成 3 段、中心偏 20 像素）。
        // 计数对两者都稳：墙的列有 6~8 行亮，缺口只有 0~2 行亮。
        std::vector<int> cnt(static_cast<size_t>(w), 0);
        std::vector<int> cmax(static_cast<size_t>(w), 0);
        for (int yy = wl.y0; yy <= wl.y1; ++yy) {
            const uint8_t* row = rgb + static_cast<size_t>(yy) * w * 3;
            for (int x = 0; x < w; ++x) {
                const int L = luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
                if (L > cmax[static_cast<size_t>(x)]) cmax[static_cast<size_t>(x)] = L;
            }
        }
        // 行带内的"亮"水平：p75 与 p25 的中点（p25 就是场地/缺口水平）
        std::vector<int> sorted = cmax;
        std::sort(sorted.begin(), sorted.end());
        const int n = static_cast<int>(sorted.size());
        const int p75 = sorted[static_cast<size_t>(n * 3 / 4)];
        const int p25 = sorted[static_cast<size_t>(n / 4)];
        int Tloc = (p75 + p25) / 2;
        if (Tloc < p25 + 12) Tloc = p25 + 12;
        wl.T_local = Tloc;
        for (int yy = wl.y0; yy <= wl.y1; ++yy) {
            const uint8_t* row = rgb + static_cast<size_t>(yy) * w * 3;
            for (int x = 0; x < w; ++x) {
                const int r = row[x * 3], g = row[x * 3 + 1], b = row[x * 3 + 2];
                const bool hit = (min_warmth > 0)
                    ? is_wall_pixel(r, g, b, min_warmth)
                    : (luma(r, g, b) >= Tloc);
                if (hit) cnt[static_cast<size_t>(x)] += 1;
            }
        }
        // 墙的列应该有"墙那么厚"的行数；取各列计数的 p75 当作墙厚度估计
        std::vector<int> csorted = cnt;
        std::sort(csorted.begin(), csorted.end());
        const int cnt_p75 = csorted[static_cast<size_t>(n * 3 / 4)];
        const int cnt_need = std::max(2, static_cast<int>(0.5 * cnt_p75));
        wl.cnt_need = cnt_need;

        // 逐列的"是墙"判断，再做一次**孤点清理**：
        // 一个孤立的"墙列"（左右都不是墙）几乎一定是噪声竖条 —— 实测缺口正中间
        // 就有这么一列 cnt=10，它把缺口切成两段，于是"取最宽那一段"只拿到左半，
        // 中心偏 12 px。反过来，墙中间一个孤立的"缺口列"也要补回去。
        std::vector<uint8_t> is_wall(static_cast<size_t>(w), 0);
        for (int x = 0; x < w; ++x) {
            is_wall[static_cast<size_t>(x)] = cnt[static_cast<size_t>(x)] >= cnt_need ? 1 : 0;
        }
        for (int pass = 0; pass < 2; ++pass) {
            for (int x = 1; x + 1 < w; ++x) {
                const int a = is_wall[static_cast<size_t>(x - 1)];
                const int b = is_wall[static_cast<size_t>(x)];
                const int c = is_wall[static_cast<size_t>(x + 1)];
                if (b && !a && !c) is_wall[static_cast<size_t>(x)] = 0;
                if (!b && a && c) is_wall[static_cast<size_t>(x)] = 1;
            }
        }
        int x0 = -1, x1 = -1;
        for (int x = 0; x < w; ++x) {
            if (is_wall[static_cast<size_t>(x)]) {
                if (x0 < 0) x0 = x;
                x1 = x;
            }
        }
        if (x0 < 0) {
            if (!p.all_bands) continue;
            x0 = 0; x1 = w - 1;
        }
        wl.x0 = x0; wl.x1 = x1;
        if (!p.all_bands && (x1 - x0 + 1) < p.min_span_frac * w) continue;

        int n_bright_cols = 0;
        for (int x = x0; x <= x1; ++x) {
            if (is_wall[static_cast<size_t>(x)]) ++n_bright_cols;
        }
        wl.bright_frac = static_cast<double>(n_bright_cols) / (x1 - x0 + 1);

        // ---- 4) 缺口：跨度内"明显低于墙"的内部连续区间 ----
        //
        // 判据用**双阈值**：低于 Tloc*0.85 才算缺口（迟滞的下限），这样墙端部
        // 一两个像素的过渡不会被撕成好几个假缺口。
        int best_w = 0, bx0 = 0, bx1 = -1, n_gaps = 0, cur0 = -1;
        for (int x = x0; x <= x1 + 1; ++x) {
            const bool is_gap = (x <= x1) && !is_wall[static_cast<size_t>(x)];
            if (is_gap) {
                if (cur0 < 0) cur0 = x;
            } else if (cur0 >= 0) {
                const int gx0 = cur0, gx1 = x - 1;
                const int gw = gx1 - gx0 + 1;
                cur0 = -1;
                const bool inside = (gx0 > x0) && (gx1 < x1);
                if (inside && gw >= p.min_gap_px && gw <= p.max_gap_px &&
                    (!clipped || (gx0 > cx0 + p.gap_edge_margin &&
                                  gx1 < cx1 - p.gap_edge_margin))) {
                    ++n_gaps;
                    if (gw > best_w) { best_w = gw; bx0 = gx0; bx1 = gx1; }
                }
            }
        }
        wl.n_gaps = n_gaps;
        if (n_gaps > 0) {
            wl.has_gap = true;
            wl.gap_x0 = bx0; wl.gap_x1 = bx1; wl.gap_w = best_w;
            wl.gap_cx = 0.5 * (bx0 + bx1);
            wl.left_x0 = x0; wl.left_x1 = bx0 - 1;
            wl.right_x0 = bx1 + 1; wl.right_x1 = x1;
        }
        // ---- 结构门（唯一一道）------------------------------------------
        // 为什么不再用颜色：实测板子相机把墙的 (255,120,0) 拍成 (130,128,124)
        // （r−b=7，纯中性），而屏幕外那圈背景反而更"暖"（r−b=43）→ 颜色门会把
        // 背景认成墙（绿框跑到平台外）。亮度对比也只有 ~10 个灰阶，靠不住。
        // 稳定的是几何：**横贯场地宽度 + 带内有缺口 + 厚度 5~20 行**。
        // 网格线连续（没缺口）、外框太薄、角色/标记太小 —— 都过不了这三条。
        if (!p.all_bands) {
            if (!wl.has_gap && !p.keep_no_gap) continue;  // 默认：没有缺口 = 不是 wall_with_gap
            if (wl.gap_w < p.min_gap_px || wl.gap_w > p.max_gap_px) continue;
            if (wl.bright_frac < p.min_bright_frac) continue;
        } else if (!wl.has_gap) {
            wl.x0 = 0; wl.x1 = w - 1;   // 没量到缺口：整条行带当墙
        }
        out.push_back(wl);
    }
    return out;
}

// ---------------------------------------------------------------------------
// 门候选去重：同一道墙的多个候选只留一个
// ---------------------------------------------------------------------------
// 为什么必须做（实测）：`find_at(..., &all, ...)` 会把每一条墙带上**所有**可能的
// 缺口位置都当成候选输出。一次实测里 1 帧输出了 **94 扇门** —— 而 `decide()` 会把
// 每一扇都当成一段墙注入规划器，于是同一道墙被叠了十几遍、位置还各差几个像素。
// 规划器看到的是"墙糊成一片"，指令自然乱；这类错在单帧画面上完全看不出来。
//
// 判据：两条候选如果**行带重叠**且**缺口中心接近**，就是同一扇门的重复观测，
// 只保留"证据最强"的那条（先比 snr，再比缺口宽度）。
struct DedupParams {
    //: 行带中心容差（像素）：小于"半条墙厚 + 2"就认为是同一道墙。
    double y_tol_px = 0.0;     // 0 → 用两条候选的半厚之和
    //: 缺口中心容差（像素）：取两条候选里较窄缺口的一半。
    double x_tol_px = 0.0;
};

inline std::vector<band_gap::Door> dedup_doors(const std::vector<band_gap::Door>& in,
                                     const DedupParams& p = DedupParams{}) {
    using Door = band_gap::Door;
    std::vector<Door> out;
    // 先按证据强度排序（snr 大的优先；snr 全 0 时按缺口宽度）
    std::vector<const Door*> order;
    order.reserve(in.size());
    for (const auto& d : in) order.push_back(&d);
    std::sort(order.begin(), order.end(), [](const Door* a, const Door* b) {
        if (a->snr != b->snr) return a->snr > b->snr;
        return a->width > b->width;
    });
    for (const Door* d : order) {
        const double yc = 0.5 * (static_cast<double>(d->y0) + d->y1);
        const double hh = 0.5 * (static_cast<double>(d->y1 - d->y0) + 1.0);
        bool dup = false;
        for (const Door& k : out) {
            const double kyc = 0.5 * (static_cast<double>(k.y0) + k.y1);
            const double khh = 0.5 * (static_cast<double>(k.y1 - k.y0) + 1.0);
            const double ytol = p.y_tol_px > 0 ? p.y_tol_px : (hh + khh + 2.0);
            const double xtol = p.x_tol_px > 0
                                    ? p.x_tol_px
                                    : 0.5 * std::min(static_cast<double>(d->width),
                                                     static_cast<double>(k.width));
            if (std::fabs(yc - kyc) <= ytol && std::fabs(d->cx - k.cx) <= std::max(4.0, xtol)) {
                dup = true;
                break;
            }
        }
        if (!dup) out.push_back(*d);
    }
    std::sort(out.begin(), out.end(), [](const Door& a, const Door& b) { return a.y0 < b.y0; });
    return out;
}

//: 每道墙**只留一扇门**：按行带聚类，每类里保留证据最强的那条候选。
//:
//: 为什么必须做：`find_at(..., &all, ...)` 输出的是每条墙带上**所有**可能的缺口位置
//: （实测一次 98~153 条）。而 `decide()` 会把每一条都当成一段墙注入规划器 ——
//: 同一道墙被叠几十遍、位置各差几像素，规划器看到的墙"糊成一片"，
//: 实测 120 s 里 18~20 次碰撞，最小间隙 -13.9 场地单位（重叠）。
//:
//: 语义上"一道墙只有一个缺口"是**场景契约**（`wall_with_gap` 一次只开一个口），
//: 所以这里不是启发式去噪，而是把检测器的候选列表收敛回契约允许的形状。
inline std::vector<band_gap::Door> one_gap_per_band(const std::vector<band_gap::Door>& in,
                                                    double prev_cx = -1e9) {
    using Door = band_gap::Door;
    std::vector<Door> out;
    // 排序依据（按重要性）：
    //   1. **跨帧连续性**：和上一帧选中缺口中心相差 < 40 px 的候选优先。
    //      为什么最重要：`find_at` 对每条墙带会吐出**许多**候选缺口（实测单帧 27 个，
    //      最多 153 个），而其中只有 10% 带非零 snr —— 只按宽度/支撑度选，很容易
    //      选到同一道墙上的**另一个**候选，于是平台朝错误的缺口开过去
    //      （实测碰撞前一刻：板子瞄 136、真值该道墙的缺口在 550，方向直接反了）。
    //   2. snr（有就信）
    //   3. 缺口宽度
    //   4. 左右墙段支撑度
    auto score = [&](const Door* d) {
        double s = 0.0;
        if (prev_cx > -1e8 && std::fabs(static_cast<double>(d->cx) - prev_cx) < 40.0) s += 1000.0;
        s += static_cast<double>(d->snr) * 10.0;
        s += static_cast<double>(d->width) * 0.1;
        s += static_cast<double>(d->support);
        return s;
    };
    std::vector<const Door*> order;
    order.reserve(in.size());
    for (const auto& d : in) order.push_back(&d);
    std::sort(order.begin(), order.end(),
              [&](const Door* a, const Door* b) { return score(a) > score(b); });
    for (const Door* d : order) {
        const double yc = 0.5 * (static_cast<double>(d->y0) + d->y1);
        const double hh = 0.5 * (static_cast<double>(d->y1 - d->y0) + 1.0);
        bool same_band = false;
        for (const Door& k : out) {
            const double kyc = 0.5 * (static_cast<double>(k.y0) + k.y1);
            const double khh = 0.5 * (static_cast<double>(k.y1 - k.y0) + 1.0);
            if (std::fabs(yc - kyc) <= hh + khh + 2.0) { same_band = true; break; }
        }
        if (!same_band) out.push_back(*d);
    }
    std::sort(out.begin(), out.end(), [](const Door& a, const Door& b) { return a.y0 < b.y0; });
    return out;
}

// ---------------------------------------------------------------------------
// P3：当前"威胁墙"的选择（场地单位，纯逻辑，可脱离图像做单元测试）
// ---------------------------------------------------------------------------
// 为什么不能按"图像里最靠下/最近"来选
// ----------------------------------
// 位置先验在图像坐标里没有物理意义：相机稍微移动，"最靠下的墙"就换人；而且
// 角色可能已经**越过**某道墙，那道墙不再威胁它，图像坐标却完全看不出来。
//
// 正确的定义只有一条：**角色前进方向上、第一道挡路的墙**。
//   · 角色在场地坐标里朝 +y 走（场地口径 +y = 前进方向）
//   · 威胁墙 = 满足 `墙中线 y - 角色 y > 0`（在角色前方）里，距离最小的那道
//   · 没有"前方"的墙（全都已越过）→ 返回 -1，调用方按"无威胁"处理（不是随便选一道）
//   · 迟滞：已经锁定的那道墙，只要它仍在**同一侧**且距离仍在前几名之内，就继续锁，
//     避免两墙距离接近时逐帧来回翻目标（生产代码里 `door_locked` 就是干这个的）
struct WallInField {
    int id = 0;
    double y_center = 0.0;   // 墙中线（场地单位，+y 朝前）
    double half_h = 0.0;     // 墙的半厚
    double gap_cx = 0.0;     // 缺口中心（场地单位 x）
    double gap_w = 0.0;      // 缺口宽度
    bool has_gap = false;
};

struct ThreatParams {
    //: 迟滞：候选墙的距离不超过"已锁定墙距离 + 这个容差"时才允许切换。
    //: 取 15 场地单位 ≈ 角色直径的 1.5 倍 —— 比它更近的新墙才值得改目标。
    double switch_tol_units = 15.0;
};

//: 选出当前威胁墙的下标（找不到返回 -1）。
//: `locked_y` 是上一帧锁定的墙中线（没有就传 NaN / 用 `locked_id` < 0 表示）。
inline int select_threat_wall(const std::vector<WallInField>& walls, double player_y,
                              const ThreatParams& p = ThreatParams{},
                              int locked_id = -1, double locked_y = 0.0) {
    int best = -1;
    double best_dist = 1e18;
    for (size_t i = 0; i < walls.size(); ++i) {
        const double d = walls[i].y_center - player_y;
        if (d <= 0.0) continue;                    // 已经越过：不再威胁
        if (!walls[i].has_gap) continue;           // 没缺口就不是"可通过的威胁"
        if (d < best_dist) { best_dist = d; best = static_cast<int>(i); }
    }
    if (best < 0) return -1;

    // 迟滞：锁定目标还在前方、且不比最优差太多 → 继续锁
    if (locked_id >= 0) {
        for (size_t i = 0; i < walls.size(); ++i) {
            if (walls[i].id != locked_id) continue;
            const double d_lock = walls[i].y_center - player_y;
            if (d_lock > 0.0 && d_lock <= best_dist + p.switch_tol_units) {
                return static_cast<int>(i);
            }
            break;
        }
    }
    (void)locked_y;
    return best;
}

//: **仅用于可视化/诊断**（在标注视频里挑一道画出来）。决策不要用它 —— 见
//: 上面的 `select_threat_wall`：图像坐标里"最靠下"不等于"角色正前方"。
inline int pick_wall_for_drawing(const std::vector<Wall>& walls, int h) {
    int best = -1;
    double best_score = -1e18;
    for (size_t i = 0; i < walls.size(); ++i) {
        const Wall& w = walls[i];
        double score = w.has_gap ? 1000.0 : 0.0;
        score += (static_cast<double>(h) - w.cy);
        if (score > best_score) { best_score = score; best = static_cast<int>(i); }
    }
    return best;
}

}   // namespace wall_gap
}   // namespace vision_board

#endif   // VISION_BOARD_WALL_GAP_HPP
