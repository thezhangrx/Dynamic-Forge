// 角色检测（P2）—— 只在场地里找，且**与四角标记解耦**。
//
// 为什么要重写
// ------------
// 旧的 `band_gap::find_player` 是"全图找最亮的圆"。实测两个致命问题：
//   1. **四角标记**也是"小而紧凑的亮块"（深底 + 亮内图），形状判据和角色重叠，
//      结果品红框每次都落在右上角标记上（视频里一眼可见）；
//   2. 全图搜索没有任何约束：编辑器文字、网格线交点都能凑出候选。
// 实测 102 帧里 PS 侧角色一次都没被采用（`player_src` 全是 rtl / none）。
//
// 这一版的做法（三段，都是廉价操作）：
//   A. **先有几何**：场地 ROI 由四标记单应反投影得到（没有几何就不找角色，
//      直接 `no_geometry` —— 不猜）。
//   B. **粗到细**：把 ROI 按 8×8 像元压成"每格亮像素占比"，在粗网格上找连通块
//      （墙会横跨几十格 → 面积超限被否；网格线只占格内 1/8 → 过不了 50% 阈值），
//      再对留下的少数候选回到像素级求质心/包围盒/填充率。
//   C. **按契约验形**：面积要对得上契约半径（20 场地单位直径）换算出来的像素面积，
//      长宽比、填充率、圆度都要像一个**实心圆**；并且离任何一个标记中心都足够远。
//
// 单位：全部像素；面积阈值由调用方用 `Scale`（场地单位/像素）从契约半径换算，
// 不写死"直径 16 px"这种会被相机距离带跑的数。
#ifndef VISION_BOARD_PLAYER_HPP
#define VISION_BOARD_PLAYER_HPP

#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

#include "rectify.hpp"

namespace vision_board {
namespace player {

inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: 亮阈值。<=0 → 自动：ROI 内 (p50 + p995)/2，与曝光无关（整幅一起变）。
    int l_bright = 0;
    //: 粗网格的格子边长（像素）。8 是权衡：格子再大墙和角色会粘在一起，再小则
    //: 噪声把格子填满、连通块碎掉。
    int cell = 8;
    //: 一个格子算"亮"的门槛：格内亮像素占比。
    double cell_frac = 0.50;
    //: 候选连通块允许的格子数范围（对应面积范围，仅用于快速初筛）。
    int min_cells = 2;
    int max_cells = 120;
    //: 面积（像素）与"契约面积"的比值允许范围。契约面积 = π r_px²，r_px 由契约半径
    //: 10 场地单位 / scale 得到。实测角色直径 16 px → 面积约 200；给足模糊余量。
    double min_area_ratio = 0.25;
    double max_area_ratio = 4.0;
    //: 形状判据
    double min_fill = 0.50;      // 圆 fill ≈ 0.79
    double max_aspect = 1.7;
    double max_roundness = 1.8;  // 二阶矩两轴之比
    //: 离标记中心的排除半径（相对标记边长）。标记只露出一部分时也要排除掉，
    //: 否则标记的内亮块会被当成角色。
    double marker_exclude_k = 2.0;
    //: **窗口内亮像素总量上限**（相对"一个圆盘该有的像素数"）。
    //: 这是"错块识别"的关键判据：角色是**孤立实心圆**，±3r 的方窗里亮像素 ≈ 圆盘
    //: 面积；而墙段上任意一点，整个窗口都是亮的（≈9× 圆盘面积）。
    //:
    //: ⚠ 一开始用的是"横向亮段长度"（圆 2r vs 墙几百像素），但**场地网格线**
    //:   亮度与角色同级（实测 180~200），角色压在网格线上时横向亮段会一直延伸，
    //:   于是**真角色被误杀**（全局检测几乎全灭，系统退化成靠局部精修漂移）。
    //:   改成"窗口亮像素计数"后：网格线只多贡献 ~2×窗宽 个像素，能过；
    //:   墙段是整个窗口全亮，被拒。
    double max_window_fill_k = 2.5;
    //: 横向亮段长度上限（相对圆盘半径）。墙段 >20r，角色压在网格线上可能到 6~10r。
    //: 1000 = 关闭该判据（对照实验用）。
    double max_run_k = 12.0;
    //: **连通块窗口**（相对圆盘半径）。做法：在 ±k·r 的窗口里对亮掩膜做 1 px 腐蚀
    //: （细网格线会被腐蚀掉），再从候选中心做 4 连通填充；**填充碰到窗口边界就判否**
    //: （说明它连着一根长亮条 —— 墙段的端头正是这种情况）。
    //: 这条能分开"墙端头"与"角色"：端头必与墙身相连，角色是孤立的圆盘。
    double comp_window_k = 6.0;
    //: 判定"落在墙带上"的 y 余量（像素）。候选中心离任何墙带 < 这个值就判为"在墙上"。
    //: **为什么需要这条**：墙段的**端头**是 ~15×10 px 的亮块，和角色圆盘在面积、
    //: 填充率、圆度、横向亮段上**几乎完全一样** —— 实测闭环语料里 100/221 个错帧
    //: 全是这种"墙端头被当成角色"。只有借助**墙带几何**才分得开：
    //: 端头的行号落在墙带里，而角色的行号在墙带外的场地灰上。
    int wall_band_margin = 12;
};

struct Player {
    bool found = false;
    double x = 0.0, y = 0.0;   // 质心（像素）
    int w = 0, h = 0;          // 包围盒
    int area = 0;
    double fill = 0.0;
    double roundness = 1.0;
    double conf = 0.0;         // 0~1，越大越像（诊断/多候选排序用）
    const char* why = "none";  // 未找到的原因：no_geometry / no_blob / bad_shape / near_marker
};

namespace detail {

//: ROI 内亮像素的 (p50, p995)，用于自动阈值。抽样：每 2 像素一次足够。
inline void roi_levels(const uint8_t* rgb, int w, int x0, int x1, int y0, int y1,
                       int* out_p50, int* out_p995) {
    double hist[256];
    for (int i = 0; i < 256; ++i) hist[i] = 0;
    double total = 0;
    for (int y = y0; y <= y1; y += 2) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = x0; x <= x1; x += 2) {
            hist[luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2])] += 1.0;
            total += 1.0;
        }
    }
    auto pct = [&](double q) {
        const double need = q * total;
        double acc = 0;
        for (int i = 0; i < 256; ++i) {
            acc += hist[i];
            if (acc >= need) return i;
        }
        return 255;
    };
    if (total <= 0) { *out_p50 = 0; *out_p995 = 255; return; }
    *out_p50 = pct(0.50);
    *out_p995 = pct(0.995);
}

}   // namespace detail

//: 在场地 ROI 内找角色。
//:
//: `markers` 是**已经按几何排序**的 TL/TR/BL/BR 四标记（`main.cpp` 里排好后传进来），
//: `h` 是图像→场地的单应（用它反投影出场地的图像包围盒）。
//: 没有可信几何（`h.ok == false`）时直接返回 `no_geometry`：不猜。
//: `wall_bands`：已知墙带的行区间（含端点），可为空。给了就排除"落在墙带上"的候选 ——
//: 墙端头与角色圆盘在形状上不可分，必须靠这层几何约束（见 `wall_band_margin` 注释）。
inline Player find(const uint8_t* rgb, int w, int hh, const rectify::Homography& h,
                   const std::vector<rectify::Marker>& markers,
                   double field_w, double field_h, double player_radius_units,
                   const Params& p = Params{},
                   const std::vector<std::pair<int, int>>* wall_bands = nullptr) {
    Player out;
    if (!rgb || w < 32 || hh < 32) return out;
    if (!h.ok) { out.why = "no_geometry"; return out; }

    // ---- A. 场地 ROI（由单应反投影，比"整幅图"小且与桌面内容无关）----
    int bx0 = 0, bx1 = 0, by0 = 0, by1 = 0;
    if (!rectify::field_bbox(h, field_w, field_h, w, hh, &bx0, &bx1, &by0, &by1)) {
        out.why = "no_geometry";
        return out;
    }
    const int x0 = std::max(0, bx0 - 4), x1 = std::min(w - 1, bx1 + 4);
    const int y0 = std::max(0, by0 - 4), y1 = std::min(hh - 1, by1 + 4);
    if (x1 - x0 < 16 || y1 - y0 < 16) { out.why = "no_roi"; return out; }

    // ---- 阈值：ROI 内分位数中点（曝光无关）----
    int T = p.l_bright;
    if (T <= 0) {
        int p50 = 0, p995 = 255;
        detail::roi_levels(rgb, w, x0, x1, y0, y1, &p50, &p995);
        T = (p50 + p995) / 2;
        if (T < p50 + 20) T = p50 + 20;
        if (T > p995 - 8) T = p995 - 8;
        T = std::max(1, std::min(254, T));
    }

    // ---- B. 粗网格：每格亮像素占比 ----
    const int cell = std::max(2, p.cell);
    const int gw = (x1 - x0 + 1 + cell - 1) / cell;
    const int gh = (y1 - y0 + 1 + cell - 1) / cell;
    std::vector<int> cnt(static_cast<size_t>(gw) * gh, 0);
    for (int y = y0; y <= y1; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        const int gy = (y - y0) / cell;
        for (int x = x0; x <= x1; ++x) {
            if (luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]) >= T) {
                cnt[static_cast<size_t>(gy) * gw + (x - x0) / cell] += 1;
            }
        }
    }
    const double cell_area = static_cast<double>(cell) * cell;
    std::vector<uint8_t> on(static_cast<size_t>(gw) * gh, 0);
    for (size_t i = 0; i < cnt.size(); ++i) {
        on[i] = (cnt[i] >= p.cell_frac * cell_area) ? 1 : 0;
    }

    // ---- 粗网格上的 4 连通标记（网格很小，直接 BFS）----
    std::vector<int> label(on.size(), 0);
    std::vector<int> stack;
    int next = 1;
    struct Cluster { int gx0, gx1, gy0, gy1, cells; };
    std::vector<Cluster> clusters;
    for (int gy = 0; gy < gh; ++gy) {
        for (int gx = 0; gx < gw; ++gx) {
            const size_t idx = static_cast<size_t>(gy) * gw + gx;
            if (!on[idx] || label[idx]) continue;
            Cluster c{gx, gx, gy, gy, 0};
            stack.clear();
            stack.push_back(static_cast<int>(idx));
            label[idx] = next;
            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                const int cy = cur / gw, cx = cur % gw;
                c.cells++;
                c.gx0 = std::min(c.gx0, cx); c.gx1 = std::max(c.gx1, cx);
                c.gy0 = std::min(c.gy0, cy); c.gy1 = std::max(c.gy1, cy);
                const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = cx + dx[k], ny = cy + dy[k];
                    if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                    const size_t ni = static_cast<size_t>(ny) * gw + nx;
                    if (on[ni] && !label[ni]) { label[ni] = next; stack.push_back(static_cast<int>(ni)); }
                }
            }
            clusters.push_back(c);
            ++next;
        }
    }

    // ---- 契约面积：由"场地单位/像素"换算，交给调用方给的尺度 ----
    // 用 bbox 宽反推单位/像素（和 geometry::judge 同一口径）
    const double unit_per_px = field_w / static_cast<double>(bx1 - bx0);
    const double r_px = (player_radius_units > 0 && unit_per_px > 0)
                        ? player_radius_units / unit_per_px : 8.0;
    const double area_expect = 3.14159265358979 * r_px * r_px;

    double best = -1.0;
    for (const Cluster& c : clusters) {
        if (c.cells < p.min_cells || c.cells > p.max_cells) continue;
        const int px0 = x0 + c.gx0 * cell, px1 = std::min(x1, x0 + (c.gx1 + 1) * cell - 1);
        const int py0 = y0 + c.gy0 * cell, py1 = std::min(y1, y0 + (c.gy1 + 1) * cell - 1);
        // 像素级精修：这一小片区域一般只有几十×几十
        long n = 0;
        double sx = 0, sy = 0, sxx = 0, syy = 0;
        int mminx = 1 << 30, mmaxx = -1, mminy = 1 << 30, mmaxy = -1;
        for (int y = py0; y <= py1; ++y) {
            const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
            for (int x = px0; x <= px1; ++x) {
                if (luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]) < T) continue;
                ++n; sx += x; sy += y; sxx += static_cast<double>(x) * x;
                syy += static_cast<double>(y) * y;
                mminx = std::min(mminx, x); mmaxx = std::max(mmaxx, x);
                mminy = std::min(mminy, y); mmaxy = std::max(mmaxy, y);
            }
        }
        if (n < 20) continue;
        // **二次精修（去窗口偏置）**：上面这一趟是在**按 8×8 网格对齐**的窗口里算质心的，
        // 圆一旦靠近窗口边缘，就会有一块被切在外面 → 质心被拉向包含的那一半，
        // 抖动可达 ±4~8 px（实测平台静止时检测位置每帧还跳 7 px）。
        // 这里以第一趟的质心为中心、用圆盘半径开一个小窗重算一次，把网格对齐的偏置去掉。
        {
            double cx0 = sx / n, cy0 = sy / n;
            const int rr = static_cast<int>(r_px * 1.4) + 3;
            const int qx0 = std::max(0, static_cast<int>(std::lround(cx0)) - rr);
            const int qx1 = std::min(w - 1, static_cast<int>(std::lround(cx0)) + rr);
            const int qy0 = std::max(0, static_cast<int>(std::lround(cy0)) - rr);
            const int qy1 = std::min(hh - 1, static_cast<int>(std::lround(cy0)) + rr);
            long n2 = 0;
            double sx2 = 0, sy2 = 0, sxx2 = 0, syy2 = 0;
            int ax0 = 1 << 30, ax1 = -1, ay0 = 1 << 30, ay1 = -1;
            for (int y = qy0; y <= qy1; ++y) {
                const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
                for (int x = qx0; x <= qx1; ++x) {
                    if (luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]) < T) continue;
                    ++n2; sx2 += x; sy2 += y;
                    sxx2 += static_cast<double>(x) * x;
                    syy2 += static_cast<double>(y) * y;
                    ax0 = std::min(ax0, x); ax1 = std::max(ax1, x);
                    ay0 = std::min(ay0, y); ay1 = std::max(ay1, y);
                }
            }
            // 只有"重算后仍然像同一团亮块"才采用（面积变化不超过 3 倍）
            if (n2 >= 20 && n2 <= 3 * n && ax1 > ax0 && ay1 > ay0) {
                n = n2; sx = sx2; sy = sy2; sxx = sxx2; syy = syy2;
                mminx = ax0; mmaxx = ax1; mminy = ay0; mmaxy = ay1;
            }
        }
        // ⚠ 试过"均值漂移（圆盘内迭代求质心）"来对抗"角色贴墙时质心被拉偏"：
        //   实测反而更差（脚本语料 62.7% → 44.3%，闭环语料 19.9% → 14.5%）——
        //   因为小圆盘内的 argmax 会收敛到**最亮的那片**（角色高光/墙的高亮边），
        //   不是几何中心。已回退。贴墙问题的正确解法见文档 §9.4（先剔除已知墙带）。
        const double cw = mmaxx - mminx + 1, ch = mmaxy - mminy + 1;
        const double fill = static_cast<double>(n) / (cw * ch);
        const double aspect = std::max(cw, ch) / std::max(1.0, std::min(cw, ch));
        const double cx = sx / n, cy = sy / n;
        const double varx = sxx / n - cx * cx, vary = syy / n - cy * cy;
        const double sdx = std::sqrt(std::max(0.0, varx));
        const double sdy = std::sqrt(std::max(0.0, vary));
        const double roundness = (sdx > 0 && sdy > 0)
            ? (sdx > sdy ? sdx / sdy : sdy / sdx) : 1e9;
        const double area_ratio = static_cast<double>(n) / std::max(1.0, area_expect);

        if (area_ratio < p.min_area_ratio || area_ratio > p.max_area_ratio) continue;

        // **十字长度检验**：这是把"角色"与"网格线 / 墙段"分开的可靠判据。
        //   角色（实心圆）：竖直亮段 ≈ 直径（2r），水平亮段 ≈ 直径；
        //   网格线（细）：竖直亮段只有 1~2 px → 被下界挡掉；
        //   墙段（长条）：水平亮段几百像素 → 被上界挡掉。
        // 之前用 fill/aspect 来分，实测在有网格线穿过窗口时 bbox 被拉大到整个窗口，
        // fill 只剩 0.16、aspect 也跟着失真 —— 真角色被误杀（离线语料漏检 53%）。
        {
            const int cx_i = static_cast<int>(std::lround(cx));
            const int cy_i = static_cast<int>(std::lround(cy));
            const int lim = static_cast<int>(p.max_run_k * std::max(2.0, r_px));
            int vrun = 1, hrun = 1;
            for (int y = cy_i - 1; y >= 0 && cy_i - y <= lim; --y) {
                const uint8_t* rp = rgb + static_cast<size_t>(y) * w * 3;
                if (luma(rp[cx_i * 3], rp[cx_i * 3 + 1], rp[cx_i * 3 + 2]) < T) break;
                ++vrun;
            }
            for (int y = cy_i + 1; y < hh && y - cy_i <= lim; ++y) {
                const uint8_t* rp = rgb + static_cast<size_t>(y) * w * 3;
                if (luma(rp[cx_i * 3], rp[cx_i * 3 + 1], rp[cx_i * 3 + 2]) < T) break;
                ++vrun;
            }
            const uint8_t* rowp = rgb + static_cast<size_t>(cy_i) * w * 3;
            for (int x = cx_i - 1; x >= 0 && cx_i - x <= lim; --x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < T) break;
                ++hrun;
            }
            for (int x = cx_i + 1; x < w && x - cx_i <= lim; ++x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < T) break;
                ++hrun;
            }
            if (vrun < 0.8 * r_px) continue;              // 太薄 → 网格线
            if (vrun > 3.2 * r_px) continue;              // 太厚 → 大块/墙粘连
            if (hrun > lim) continue;                     // 太长 → 墙段
        }
        if (fill < 0.30) continue;                        // 松一点的兜底（网格线会把 bbox 拉大）

        // **腐尸+填充检验（墙端头判别）**：在 ±k·r 窗口里对亮掩膜腐蚀 1 px（细网格线
        // 会被腐蚀掉、圆盘保留），从候选中心 4 连通填充；**填到窗口边界 = 它连着一根
        // 长亮条 → 是墙的端头**。角色（哪怕压在网格线上）填充不会碰到边界。
        {
            const int R = std::max(8, static_cast<int>(p.comp_window_k * r_px));
            const int cx_i = static_cast<int>(std::lround(cx));
            const int cy_i = static_cast<int>(std::lround(cy));
            const int W = 2 * R + 1;
            std::vector<uint8_t> m(static_cast<size_t>(W) * W, 0), e(static_cast<size_t>(W) * W, 0);
            for (int yy = 0; yy < W; ++yy) {
                const int y = cy_i - R + yy;
                if (y < 0 || y >= hh) continue;
                const uint8_t* rp = rgb + static_cast<size_t>(y) * w * 3;
                for (int xx = 0; xx < W; ++xx) {
                    const int x = cx_i - R + xx;
                    if (x < 0 || x >= w) continue;
                    if (luma(rp[x * 3], rp[x * 3 + 1], rp[x * 3 + 2]) >= T) {
                        m[static_cast<size_t>(yy) * W + xx] = 1;
                    }
                }
            }
            // 腐蚀 1（4 邻域）
            for (int yy = 1; yy + 1 < W; ++yy) {
                for (int xx = 1; xx + 1 < W; ++xx) {
                    const size_t i0 = static_cast<size_t>(yy) * W + xx;
                    if (!m[i0]) continue;
                    if (m[i0 - 1] && m[i0 + 1] && m[i0 - W] && m[i0 + W]) e[i0] = 1;
                }
            }
            // 种子：腐蚀后中心附近找最近的亮像素
            int sx = -1, sy = -1;
            for (int rr = 0; rr <= 2 && sx < 0; ++rr) {
                for (int dy2 = -rr; dy2 <= rr && sx < 0; ++dy2) {
                    for (int dx2 = -rr; dx2 <= rr && sx < 0; ++dx2) {
                        const int xx = R + dx2, yy = R + dy2;
                        if (xx < 0 || yy < 0 || xx >= W || yy >= W) continue;
                        if (e[static_cast<size_t>(yy) * W + xx]) { sx = xx; sy = yy; }
                    }
                }
            }
            if (sx >= 0) {
                std::vector<int> stack{static_cast<int>(sy) * W + sx};
                std::vector<uint8_t> seen(static_cast<size_t>(W) * W, 0);
                seen[static_cast<size_t>(sy) * W + sx] = 1;
                bool touches = false;
                while (!stack.empty() && !touches) {
                    const int cur = stack.back();
                    stack.pop_back();
                    const int yy = cur / W, xx = cur % W;
                    if (xx == 0 || yy == 0 || xx == W - 1 || yy == W - 1) { touches = true; break; }
                    const int ddx[4] = {1, -1, 0, 0}, ddy[4] = {0, 0, 1, -1};
                    for (int k2 = 0; k2 < 4; ++k2) {
                        const int nx = xx + ddx[k2], ny = yy + ddy[k2];
                        if (nx < 0 || ny < 0 || nx >= W || ny >= W) continue;
                        const size_t ni = static_cast<size_t>(ny) * W + nx;
                        if (e[ni] && !seen[ni]) { seen[ni] = 1; stack.push_back(static_cast<int>(ni)); }
                    }
                }
                if (touches) continue;   // 连着一根长亮条 → 墙端头/墙身
            }
        }

        // **亮段长度（错块识别）**：角色是孤立实心圆，横向亮段 ≈ 直径；墙段上任意
        // 一点横向亮段有几百像素。限值取 12r（≈96 px）：墙段（≥300 px）必被拒，
        // 而"角色压在一条网格线上"时亮段会延伸（实测过 6r 就把真角色误杀了），
        // 12r 能放它过去 —— 这正是本轮踩过的坑：判据过严 → 全局检测几乎全灭 →
        // 系统退化成靠局部精修漂移，反而更差。
        {
            const int cx_i = static_cast<int>(std::lround(cx));
            const int cy_i = static_cast<int>(std::lround(cy));
            const int lim = static_cast<int>(p.max_run_k * std::max(2.0, r_px));
            int run = 1;
            const uint8_t* rowp = rgb + static_cast<size_t>(cy_i) * w * 3;
            for (int x = cx_i - 1; x >= 0 && cx_i - x <= lim; --x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < T) break;
                ++run;
            }
            for (int x = cx_i + 1; x < w && x - cx_i <= lim; ++x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < T) break;
                ++run;
            }
            if (run > lim) continue;
        }

        // **落在墙带上 → 是墙的端头/亮边，不是角色**
        if (wall_bands != nullptr && !wall_bands->empty()) {
            const int cy_i = static_cast<int>(std::lround(cy));
            bool on_wall = false;
            for (const auto& b : *wall_bands) {
                if (cy_i >= b.first - p.wall_band_margin && cy_i <= b.second + p.wall_band_margin) {
                    on_wall = true;
                    break;
                }
            }
            if (on_wall) continue;
        }

        // 离标记太近 → 是标记的内亮块，不是角色
        bool near_marker = false;
        for (const rectify::Marker& mk : markers) {
            const double half = p.marker_exclude_k * std::max(mk.w, mk.h);
            if (std::hypot(cx - mk.x, cy - mk.y) < half) { near_marker = true; break; }
        }
        if (near_marker) continue;

        // 打分：形状越像圆、面积越接近契约，分越高
        const double conf = fill / 0.79 * (1.0 - std::min(1.0, std::fabs(area_ratio - 1.0) * 0.5)) /
                            std::max(1.0, roundness);
        if (conf > best) {
            best = conf;
            out.found = true;
            out.x = cx; out.y = cy;
            out.w = static_cast<int>(cw); out.h = static_cast<int>(ch);
            out.area = static_cast<int>(n);
            out.fill = fill;
            out.roundness = roundness;
            out.conf = conf;
            out.why = "ok";
        }
    }
    if (!out.found) out.why = "no_blob";
    return out;
}

//: 局部精修：在 (x0,y0) 附近用"盘内亮、环带暗"的对比度重新定位圆心。
//:
//: 为什么需要它：角色贴着墙时，亮阈值会把"角色 + 墙"连成一大块，面积判据直接否掉
//: （实测 7/33 帧 `no_blob`）。但**局部**看，角色仍是一个亮盘、周围是场地灰或
//: （贴墙那一侧）亮墙 —— 盘内平均亮度显著高于环带平均亮度，这个对比度不依赖
//: "角色是独立连通域"这个前提。
//:
//: `r_px` 用契约半径换算得到（不是常数）；返回值 `contrast` 是盘内与环带的平均亮度差
//: （灰阶），越小越不可信，调用方用阈值兜住。
struct Local {
    bool found = false;
    double x = 0.0, y = 0.0;
    double contrast = 0.0;
};

inline Local refine_near(const uint8_t* rgb, int w, int hh, double x0, double y0,
                         double r_px, double search_px = 40.0,
                         double min_contrast = 12.0) {
    Local out;
    if (!rgb || w < 32 || hh < 32 || r_px <= 1.0) return out;
    const double rin = std::max(2.0, r_px * 0.7);      // 盘内采样半径
    const double r_lo = r_px * 1.25, r_hi = r_px * 1.9;  // 环带
    if (r_hi < r_lo + 1.0) return out;
    const int step = 2;
    int best_ix = -1, best_iy = -1;
    double best = -1e9;
    //: 次优但**离种子更近**的候选：角色移动慢，"附近的高对比度"比"全场最高的"
    //: 更可信（全场最高的往往是对面那道墙/标记的内亮块）。
    double best_near = -1e9;
    int near_ix = -1, near_iy = -1;
    const double near_radius = std::min(search_px, std::max(6.0, r_px * 2.0));
    const int cx0 = static_cast<int>(std::lround(x0));
    const int cy0 = static_cast<int>(std::lround(y0));
    const int R = static_cast<int>(search_px);
    for (int dy = -R; dy <= R; dy += step) {
        const int cy = cy0 + dy;
        if (cy < static_cast<int>(r_hi) || cy >= hh - static_cast<int>(r_hi)) continue;
        for (int dx = -R; dx <= R; dx += step) {
            const int cx = cx0 + dx;
            if (cx < static_cast<int>(r_hi) || cx >= w - static_cast<int>(r_hi)) continue;
            // 盘内：稀疏采样（每 2 像素）
            double s_in = 0, n_in = 0;
            for (int yy = -static_cast<int>(rin); yy <= static_cast<int>(rin); yy += 2) {
                for (int xx = -static_cast<int>(rin); xx <= static_cast<int>(rin); xx += 2) {
                    if (xx * xx + yy * yy > rin * rin) continue;
                    const uint8_t* px = rgb + (static_cast<size_t>(cy + yy) * w + (cx + xx)) * 3;
                    s_in += luma(px[0], px[1], px[2]);
                    n_in += 1;
                }
            }
            if (n_in <= 0) continue;
            // 环带：8 个方向，但用**下四分位**而不是平均值。
            // 为什么：角色贴着墙时，环带的半边是亮墙（亮度接近角色）→ 平均值被抬高，
            // 对比度被抹平（实测 7/33 帧因此漏检）。取"最暗的那一侧"就绕开了被遮住的
            // 方向 —— 只要角色周围**还有一部分**是场地灰，对比度就成立。
            int outv[8];
            int n_out = 0;
            for (int k = 0; k < 8; ++k) {
                const double a = k * 3.14159265358979 / 4.0;
                const double rr = 0.5 * (r_lo + r_hi);
                const int xx = static_cast<int>(std::lround(rr * std::cos(a)));
                const int yy = static_cast<int>(std::lround(rr * std::sin(a)));
                const uint8_t* px = rgb + (static_cast<size_t>(cy + yy) * w + (cx + xx)) * 3;
                outv[n_out++] = luma(px[0], px[1], px[2]);
            }
            if (n_out <= 0) continue;
            std::sort(outv, outv + n_out);
            const double out_dark = outv[n_out / 4];   // 下四分位
            const double contrast = s_in / n_in - out_dark;
            if (contrast > best) { best = contrast; best_ix = cx; best_iy = cy; }
            if (std::hypot(dx, dy) <= near_radius && contrast > best_near) {
                best_near = contrast; near_ix = cx; near_iy = cy;
            }
        }
    }
    // 优先取"离种子近且对比度够高"的那个；没有再退回全场最高
    if (near_ix >= 0 && best_near >= 0.7 * std::max(min_contrast, best)) {
        out.found = true; out.x = near_ix; out.y = near_iy; out.contrast = best_near;
        return out;
    }
    if (best_ix < 0 || best < min_contrast) return out;

    // **四侧检验**：一个实心圆在上下左右四个方向都应该"出了圆就暗下来"。
    // 墙段不满足：它左右一直亮（墙很长），只有上下会暗。
    // 实测教训：没有这一条时，局部精修会把**亮的墙段**当成角色（对上一个种子位置
    // 是"附近最高对比度"），表现为对账里误差双峰——p25 只有 9 场地单位（对的），
    // 中位却是 65（错的），而且错的那些会直接把平台往错误方向带。
    // 贴墙的角色只有一侧被墙盖住，仍然有 ≥3 侧是暗的，所以这条不会误伤。
    {
        const int cx = best_ix, cy = best_iy;
        const double rr = std::max(2.0, r_px * 1.6);
        int dark_sides = 0;
        const int dx4[4] = {1, -1, 0, 0};
        const int dy4[4] = {0, 0, 1, -1};
        const double disc = [&] {
            double s = 0; int n = 0;
            const int ri = std::max(1, static_cast<int>(r_px * 0.7));
            for (int yy = -ri; yy <= ri; ++yy)
                for (int xx = -ri; xx <= ri; ++xx) {
                    if (xx * xx + yy * yy > ri * ri) continue;
                    const uint8_t* px = rgb + (static_cast<size_t>(cy + yy) * w + (cx + xx)) * 3;
                    s += luma(px[0], px[1], px[2]); ++n;
                }
            return n ? s / n : 0.0;
        }();
        for (int k = 0; k < 4; ++k) {
            const int xx = cx + static_cast<int>(std::lround(dx4[k] * rr));
            const int yy = cy + static_cast<int>(std::lround(dy4[k] * rr));
            if (xx < 0 || yy < 0 || xx >= w || yy >= hh) { ++dark_sides; continue; }
            const uint8_t* px = rgb + (static_cast<size_t>(yy) * w + xx) * 3;
            if (luma(px[0], px[1], px[2]) < disc - 12.0) ++dark_sides;
        }
        if (dark_sides < 3) return out;

        // 亮段长度（同 `find` 里那条，限值 12r）：局部精修很容易沿墙段滑。
        {
            const int lim = static_cast<int>(12.0 * std::max(2.0, r_px));
            int run = 1;
            const uint8_t* rowp = rgb + static_cast<size_t>(cy) * w * 3;
            for (int x = cx - 1; x >= 0 && cx - x <= lim; --x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < disc - 25.0) break;
                ++run;
            }
            for (int x = cx + 1; x < w && x - cx <= lim; ++x) {
                if (luma(rowp[x * 3], rowp[x * 3 + 1], rowp[x * 3 + 2]) < disc - 25.0) break;
                ++run;
            }
            if (run > lim) return out;
        }
    }

    out.found = true;
    out.x = best_ix;
    out.y = best_iy;
    out.contrast = best;
    return out;
}

}   // namespace player
}   // namespace vision_board

#endif   // VISION_BOARD_PLAYER_HPP
