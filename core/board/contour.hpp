// 轮廓（连通域）分割 —— 用"形状"而不是"匹配滤波"来定角色和障碍。
//
// 为什么要多这一条路
// ------------------
// 之前那条路（`band_gap.hpp`）是**按已知几何去假设、再验证**：先假设场地边框在
// 哪、再用"行带 + 匹配滤波"去找缺口、用纹理能量找标记。它的好处是省，坏处是
// 一旦画面模糊/有噪声/"墙不在假设的位置上"，它给出的是**一个自洽但错的数**——
// 不报错，只是位置偏。这正是"识别不出、反复调参"的根子：错的方式没有形状可言。
//
// 轮廓这条路反过来：不假设任何东西，先按亮度把画面切成"深/亮"两类，各自连成
// 连通域，然后**按形状分类**：
//   · 角色 —— 亮、紧凑、近似圆（bbox 填充率高、短长轴比接近 1）
//   · 墙   —— 深、大、矩形（bbox 填充率很高、横或竖的长条）
//   · 场线 —— 深、细（bbox 填充率很低或单维 ≤ 阈值）→ 直接丢掉
//   · 标记 —— 深、小方块，数量少、在场地四角
// 每个连通域有一阶矩（质心）和二阶矩（尺寸/朝向），所以位置和速度都是**量出来**
// 的，不是拟合出来的。速度用**质心在成像平面里的帧间位移**除以 `dt`，再按
// (px/场地单位) 换算 —— 这正是"板子看到的东西动得多快"，不依赖任何先验轨迹。
//
// 代价：每帧要扫一遍 640×360（一次分块 + 一次并查集），比匹配滤波贵一点，
// 但它是**一遍过、无搜索**，不会像角度搜索那样突然贵 5 倍。
//
// 单位纪律：本文件里所有坐标都是**像素**（成像平面）。换成场地单位由调用方
// 用 `Scale`（每轴 px per 场地单位）做 —— 缩放/压缩之后的画面 x、y 两个尺度
// 不同（`--image-squash`），必须分开换，否则速度会按轴偏。
#ifndef VISION_BOARD_CONTOUR_HPP
#define VISION_BOARD_CONTOUR_HPP

#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>

namespace vision_board {
namespace contour {

// ---------------------------------------------------------------------------
// 亮度
// ---------------------------------------------------------------------------
// 用整数近似 BT.601（77/150/29 除以 256），和 `band_gap.hpp` 里那份保持一致 ——
// 两套阈值必须能互相解释，否则"为什么同一张图两边不一样"要查很久。
inline int luma(int r, int g, int b) { return (77 * r + 150 * g + 29 * b) >> 8; }

struct Params {
    //: ≤ 这个亮度算"深色"（墙、场线、定位标记）。**<=0 表示自动**（见 `otsu3`）：
    //: 拍屏幕时场地灰 ≈ 105~135、墙 ≈ 20~50、角色 ≈ 190~230；但参考语料里的
    //: 合成帧是"深底亮标记"，同一个绝对值在两种画面上必然有一边是错的。
    //: 所以默认走自动阈值，`--l-dark/--l-bright` 只用于标定时钉死。
    int l_dark = 0;
    //: ≥ 这个亮度算"亮色"（角色）。**<=0 表示自动**。
    int l_bright = 0;
    //: 小于这个像素数的连通域当噪声（实测噪声斑点到不了 40 px）。
    int min_area = 40;
    //: 最多允许多少个连通域（防止整幅图都是噪声时算出几千个，把时间搞爆）。
    int max_comps = 400;
};

// ---------------------------------------------------------------------------
// 自动阈值：三分类 Otsu
// ---------------------------------------------------------------------------
// 为什么不用固定阈值：这同一条代码要同时吃"拍屏幕的实拍帧"和"参考语料里的合成帧"，
// 两者亮度分布完全相反（实拍：灰场地 + 深墙；合成：深底 + 亮标记）。写死一个数，
// 换一种画面就整幅判错，而且**不报错**。
//
// 三分类 Otsu 的假设只有一条：直方图里有三个峰（深物体 / 场地 / 亮物体）。
// 三峰越清楚，阈值越稳；三峰不清楚（比如整幅都是场地）时阈值会落在场地内部，
// 那时"深/亮"两类里只有很小一块 —— 表现为连通域极少，调用方按数量就能看出来。
struct Thresholds {
    int dark = 0;     // 亮度 ≤ dark → 深
    int bright = 255; // 亮度 ≥ bright → 亮
    int mode = 0;     // 直方图主峰（场地亮度），诊断用
    bool ok = false;
};

//: 三分类 Otsu（在亮度直方图上穷举两个阈值）。`step` 是抽样步长：
//: 阈值只要几个灰度级的精度，没必要为它扫全图（A7 上一次全扫 ≈ 1~2 ms）。
inline Thresholds otsu3(const uint8_t* rgb, int w, int h, int step = 2) {
    Thresholds th;
    if (!rgb || w < 8 || h < 8) return th;
    if (step < 1) step = 1;
    double hist[256] = {0};
    double total = 0;
    for (int y = 0; y < h; y += step) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        for (int x = 0; x < w; x += step) {
            const int L = luma(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
            hist[L] += 1.0;
            total += 1.0;
        }
    }
    if (total <= 0) return th;

    // 前缀：概率 P(t) = Σ_{l<=t} h(l)/N，一阶矩 M(t) = Σ l·h(l)/N
    double P[257] = {0}, M[257] = {0};
    for (int i = 0; i < 256; ++i) {
        P[i + 1] = P[i] + hist[i] / total;
        M[i + 1] = M[i] + hist[i] * i / total;
    }
    const double mT = M[256];
    int best_t1 = 63, best_t2 = 191;
    double best = -1;
    // 类 0: [0, t1]  类 1: [t1+1, t2]  类 2: [t2+1, 255]
    for (int t1 = 0; t1 < 255; ++t1) {
        const double w0 = P[t1 + 1];
        if (w0 <= 0) continue;
        const double m0 = M[t1 + 1] / w0;
        for (int t2 = t1 + 1; t2 < 255; ++t2) {
            const double w1 = P[t2 + 1] - P[t1 + 1];
            if (w1 <= 0) continue;
            const double w2 = 1.0 - P[t2 + 1];
            const double m1 = (M[t2 + 1] - M[t1 + 1]) / w1;
            const double var = w0 * (m0 - mT) * (m0 - mT) + w1 * (m1 - mT) * (m1 - mT);
            if (w2 > 0) {
                const double m2 = (mT - M[t2 + 1]) / w2;
                const double v = var + w2 * (m2 - mT) * (m2 - mT);
                if (v > best) { best = v; best_t1 = t1; best_t2 = t2; }
            }
        }
    }
    // 主峰（场地亮度）：直方图最大值那个灰度级，打印出来便于人眼判断
    int mode = 0;
    double hm = -1;
    for (int i = 0; i < 256; ++i) {
        if (hist[i] > hm) { hm = hist[i]; mode = i; }
    }
    th.dark = best_t1;
    th.bright = best_t2 + 1;
    th.mode = mode;
    th.ok = true;
    // 万一 t1/t2 撞到极端（0 或 255），说明三峰不存在：退化成"只分亮/不亮"。
    if (th.bright > 255) th.bright = 255;
    if (th.dark >= th.bright) th.dark = th.bright - 1;
    return th;
}

// ---------------------------------------------------------------------------
// 一个连通域
// ---------------------------------------------------------------------------
struct Comp {
    int label = 0;
    int area = 0;
    double cx = 0, cy = 0;      // 质心（一阶矩）
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;   // bbox，含端点
    double w = 0, h = 0;         // bbox 宽高（像素）
    double fill = 0;             // area / (w*h)：矩形 ≈ 1，圆形 ≈ 0.79，细线很低
    double aspect = 1;           // max(w,h)/min(w,h)
    int mean_l = 0;              // 平均亮度
    int mean_r = 0, mean_g = 0, mean_b = 0;
    //: 二阶中心矩（归一化到面积：就是方差，开方≈标准差）。
    //: 圆的 σx=σy 且 ≈ r/2；长条的 σ 沿长轴大。用它判"圆不圆"比 aspect 稳。
    double sx = 0, sy = 0;

    //: 近似半径（用面积反推，圆的面积 = πr²）。
    double radius() const { return std::sqrt(static_cast<double>(area) / 3.14159265358979); }
    //: 圆的判据：两轴标准差接近（<=1.6 倍），且半径对得上 bbox。
    bool roundish(double tol = 1.6) const {
        if (sx <= 0 || sy <= 0) return false;
        const double r = sx > sy ? sx / sy : sy / sx;
        return r <= tol;
    }
    bool inside_image(int w, int h) const { return x0 >= 0 && y0 >= 0 && x1 < w && y1 < h; }
};

// ---------------------------------------------------------------------------
// 分类
// ---------------------------------------------------------------------------
enum class Kind {
    Unknown = 0,
    Player,      // 亮、紧凑、近似圆
    Wall,        // 深、大、矩形（横或竖的长条/方块）
    Marker,      // 深、小方块（四角定位标记）
    Line,        // 深、细（场线、边框线）—— 不是障碍
};

inline const char* kind_name(Kind k) {
    switch (k) {
        case Kind::Player: return "角色";
        case Kind::Wall:   return "墙";
        case Kind::Marker: return "标记";
        case Kind::Line:   return "线";
        default:           return "?";
    }
}

//: 分类规则。**只用形状**，不用位置先验，也**不假定墙比场地暗**。
//:
//: 为什么不能用"亮=角色、暗=墙"：实拍屏上墙是**亮的**（白墙，L≈175），场地是
//: 中灰（L≈130），背景是深蓝紫（L≈85）。只看亮度会把墙判成角色、把背景判成墙 ——
//: 而这两类都在阈值附近，错得还很稳（不报错）。形状才是稳的那一维。
//:
//: 三条判据（按顺序）：
//:   1. 细（短边 ≤ 3 px）→ 场线/边框，不是障碍
//:   2. 长条（长宽比 ≥ elongated）或大块（面积 ≥ big_area 且填充高）→ 墙
//:   3. 紧凑、近似圆（填充 0.45~0.92、长宽比 ≤ round_aspect）→ 角色
//: 其余算 Unknown（不参与决策，但会打印出来 —— 漏掉的东西必须看得见）。
struct ClassifyParams {
    // 角色（紧凑、近似圆）
    double player_min_area = 40.0;     // px
    double player_max_area = 1600.0;   // 再大就不是角色（半径 ~22 px 以上）
    double player_min_fill = 0.45;     // 圆是 0.79，模糊/噪声会低一些
    double player_max_aspect = 1.9;
    // 墙（长条或大块）
    double wall_elongated_aspect = 2.2;
    double wall_min_short_side = 4.0;
    double wall_min_area = 150.0;
    double wall_big_area = 1200.0;     // 大块方块（贴脸的墙）也算墙
    double wall_big_min_fill = 0.6;
    // 场线（细）
    double line_max_short_side = 3.0;
};

inline Kind classify(const Comp& c, bool /*from_bright*/, const ClassifyParams& cp) {
    const double short_side = std::min(c.w, c.h);
    const double long_side = std::max(c.w, c.h);
    // 1) 细 → 线
    if (short_side <= cp.line_max_short_side) return Kind::Line;
    // 2) 墙：长条，或大块
    if (c.aspect >= cp.wall_elongated_aspect && short_side >= cp.wall_min_short_side &&
        c.area >= cp.wall_min_area) {
        return Kind::Wall;
    }
    if (c.area >= cp.wall_big_area && c.fill >= cp.wall_big_min_fill) return Kind::Wall;
    // 3) 角色：紧凑、近似圆
    if (c.area >= cp.player_min_area && c.area <= cp.player_max_area &&
        c.fill >= cp.player_min_fill && c.fill <= 0.92 &&
        c.aspect <= cp.player_max_aspect && c.roundish()) {
        return Kind::Player;
    }
    if (long_side >= cp.wall_min_short_side && c.area >= cp.wall_min_area &&
        c.aspect >= cp.wall_elongated_aspect) {
        return Kind::Wall;
    }
    return Kind::Unknown;
}

// ---------------------------------------------------------------------------
// 连通域标记（按行 run + 并查集，一遍扫描）
// ---------------------------------------------------------------------------
namespace detail {

//: 并查集（路径减半 + 按秩合并的简化版：路径压缩就够）。
struct Dsu {
    std::vector<int> p;
    void init(size_t n) { p.resize(n); for (size_t i = 0; i < n; ++i) p[i] = static_cast<int>(i); }
    int find(int x) {
        while (p[static_cast<size_t>(x)] != x) {
            p[static_cast<size_t>(x)] = p[static_cast<size_t>(p[static_cast<size_t>(x)])];
            x = p[static_cast<size_t>(x)];
        }
        return x;
    }
    void unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return;
        // 总是把大的挂到小的上（标号小的赢），这样根的标号仍是"第一个像素"的标号，
        // 结果对扫描顺序稳定 —— 调试时同一张图每次跑出来的 label 一样。
        if (a < b) p[static_cast<size_t>(b)] = a;
        else p[static_cast<size_t>(a)] = b;
    }
};

struct Run {
    int y = 0, xa = 0, xb = 0;   // 含端点
    int label = 0;               // 初始标号 = run 序号
    int root = 0;                // 并查集根（最终）
};

}   // namespace detail

//: 把一个颜色平面（rgb 交错的 24bpp）按亮度阈值分成两类，返回各自的连通域。
//:
//: `out_mask` 可选：传进来一个 w*h 的 uint8 缓冲，会写入 0=场地/1=深/2=亮，
//: 方便把"板子看到的二值图"整幅 dump 出来看（这类问题光看数字永远查不清）。
inline void components(const uint8_t* rgb, int w, int h, const Params& p,
                       std::vector<Comp>* dark_out, std::vector<Comp>* bright_out,
                       uint8_t* out_mask = nullptr,
                       std::vector<int>* label_image = nullptr,
                       Thresholds* th_out = nullptr) {
    dark_out->clear();
    bright_out->clear();
    if (!rgb || w <= 0 || h <= 0) return;
    // 阈值：给了就用给的，没给（<=0）就按本帧直方图自动定
    int T_dark = p.l_dark;
    int T_bright = p.l_bright;
    if (T_dark <= 0 || T_bright <= 0) {
        const Thresholds th = otsu3(rgb, w, h);
        if (th.ok) {
            if (T_dark <= 0) T_dark = th.dark;
            if (T_bright <= 0) T_bright = th.bright;
        }
        if (th_out) *th_out = th;
    } else if (th_out) {
        th_out->dark = T_dark; th_out->bright = T_bright; th_out->ok = true;
    }
    if (T_dark < 0) T_dark = 0;
    if (T_bright > 255) T_bright = 255;

    // ---- 1) 按行找 run（两种极性一起找）----
    std::vector<detail::Run> runs;
    runs.reserve(static_cast<size_t>(w) * 4);
    // 每行 run 的起止（供相邻行做 4 连通合并）
    struct Span { int run_a = 0, run_b = 0; };   // [run_a, run_b) 该行的 run 下标
    std::vector<Span> row_spans(static_cast<size_t>(h));

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = rgb + static_cast<size_t>(y) * w * 3;
        int prev_state = 0;   // 0 场地 / 1 深 / 2 亮
        int xa = 0;
        for (int x = 0; x <= w; ++x) {
            int state = 0;
            if (x < w) {
                const int r = row[x * 3 + 0], g = row[x * 3 + 1], b = row[x * 3 + 2];
                const int L = luma(r, g, b);
                state = (L <= T_dark) ? 1 : ((L >= T_bright) ? 2 : 0);
            }
            if (state != prev_state) {
                if (prev_state != 0 && x > xa) {
                    detail::Run rn;
                    rn.y = y; rn.xa = xa; rn.xb = x - 1;
                    rn.label = static_cast<int>(runs.size());
                    rn.root = rn.label;
                    runs.push_back(rn);
                }
                xa = x;
                prev_state = state;
            }
        }
    }

    // 重新按行记录 run 区间（上面一趟里没存，这里用 label 连续的性质先算边界）
    {
        size_t i = 0;
        for (int y = 0; y < h; ++y) {
            const size_t a = i;
            while (i < runs.size() && runs[i].y == y) ++i;
            row_spans[static_cast<size_t>(y)].run_a = static_cast<int>(a);
            row_spans[static_cast<size_t>(y)].run_b = static_cast<int>(i);
        }
        if (i != runs.size()) { /* 不可能：runs 是按 y 递增造的 */ }
    }

    // ---- 2) 相邻行做 4 连通合并（同类极性才合并）----
    detail::Dsu dsu;
    dsu.init(runs.size());
    // 极性表：run -> 1 深 / 2 亮。为了省一次扫描，用"该 run 的第一个像素亮度"判。
    std::vector<uint8_t> pol(runs.size(), 0);
    for (size_t i = 0; i < runs.size(); ++i) {
        const detail::Run& rn = runs[i];
        const uint8_t* px = rgb + (static_cast<size_t>(rn.y) * w + rn.xa) * 3;
        pol[i] = static_cast<uint8_t>(luma(px[0], px[1], px[2]) <= T_dark ? 1 : 2);
    }
    for (int y = 1; y < h; ++y) {
        const Span& up = row_spans[static_cast<size_t>(y - 1)];
        const Span& cu = row_spans[static_cast<size_t>(y)];
        int i = up.run_a, j = cu.run_a;
        while (i < up.run_b && j < cu.run_b) {
            const detail::Run& a = runs[static_cast<size_t>(i)];
            const detail::Run& b = runs[static_cast<size_t>(j)];
            if (a.xb < b.xa) { ++i; continue; }
            if (b.xb < a.xa) { ++j; continue; }
            // 有重叠（4 连通只需要 x 区间相交，因为 y 相差 1）
            if (pol[static_cast<size_t>(i)] == pol[static_cast<size_t>(j)]) {
                dsu.unite(a.label, b.label);
            }
            // 继续推进：谁先结束推谁（两个都还没结束就推 i，等内层处理完）
            if (a.xb < b.xb) ++i; else if (b.xb < a.xb) ++j; else { ++i; ++j; }
        }
    }

    // ---- 3) 每个 run 的矩：按 run 的长度一次算完 ----
    //
    // 一阶矩/二阶矩都有闭式（等差数列求和），不用逐像素累加，只有颜色必须逐像素。
    // 这样热点就只剩"读像素"本身。
    const size_t nroot = runs.size();
    struct Acc {
        int area = 0;
        double sx = 0, sy = 0, sxx = 0, syy = 0;   // 未减质心
        long long sl = 0, sr = 0, sg = 0, sb = 0;
        int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
        uint8_t pol = 0;
    };
    std::vector<Acc> acc(nroot);
    for (size_t i = 0; i < nroot; ++i) {
        const detail::Run& rn = runs[i];
        const int root = dsu.find(rn.label);
        runs[i].root = root;
        const int len = rn.xb - rn.xa + 1;
        // ∑_{k=0}^{len-1} (xa+k)   = len*xa + len(len-1)/2
        // ∑_{k=0}^{len-1} (xa+k)²  = len*xa² + xa*len(len-1) + len(len-1)(2len-1)/6
        const double L = len;
        const double xa = rn.xa;
        const double sum_x = L * xa + L * (L - 1.0) * 0.5;
        const double sum_x2 = L * xa * xa + xa * L * (L - 1.0) +
                              L * (L - 1.0) * (2.0 * L - 1.0) / 6.0;
        Acc& a = acc[static_cast<size_t>(root)];
        a.area += len;
        a.sx += sum_x;
        a.sy += static_cast<double>(rn.y) * L;
        a.sxx += sum_x2;
        a.syy += static_cast<double>(rn.y) * rn.y * L;
        a.pol = pol[i];
        const uint8_t* px = rgb + (static_cast<size_t>(rn.y) * w + rn.xa) * 3;
        for (int x = 0; x < len; ++x) {
            a.sl += luma(px[x * 3], px[x * 3 + 1], px[x * 3 + 2]);
            a.sr += px[x * 3]; a.sg += px[x * 3 + 1]; a.sb += px[x * 3 + 2];
        }
        a.x0 = std::min(a.x0, rn.xa); a.x1 = std::max(a.x1, rn.xb);
        a.y0 = std::min(a.y0, rn.y);  a.y1 = std::max(a.y1, rn.y);
    }

    // ---- 4) 输出 ----
    if (label_image) {
        label_image->assign(static_cast<size_t>(w) * h, 0);
        for (size_t i = 0; i < nroot; ++i) {
            const detail::Run& rn = runs[i];
            int* row = label_image->data() + static_cast<size_t>(rn.y) * w;
            for (int x = rn.xa; x <= rn.xb; ++x) row[x] = rn.root + 1;
        }
    }
    if (out_mask) {
        std::fill(out_mask, out_mask + static_cast<size_t>(w) * h, 0);
        for (size_t i = 0; i < nroot; ++i) {
            const detail::Run& rn = runs[i];
            uint8_t* row = out_mask + static_cast<size_t>(rn.y) * w;
            const uint8_t v = pol[i];
            for (int x = rn.xa; x <= rn.xb; ++x) row[x] = v;
        }
    }

    dark_out->reserve(64);
    bright_out->reserve(64);
    for (size_t i = 0; i < nroot; ++i) {
        const Acc& a = acc[i];
        if (a.area < p.min_area) continue;
        if (static_cast<int>(dark_out->size() + bright_out->size()) >= p.max_comps) break;
        Comp c;
        c.label = static_cast<int>(i);
        c.area = a.area;
        c.cx = a.sx / a.area;
        c.cy = a.sy / a.area;
        c.x0 = a.x0; c.y0 = a.y0; c.x1 = a.x1; c.y1 = a.y1;
        c.w = a.x1 - a.x0 + 1;
        c.h = a.y1 - a.y0 + 1;
        c.fill = static_cast<double>(a.area) / (c.w * c.h);
        c.aspect = std::max(c.w, c.h) / std::max(1.0, std::min(c.w, c.h));
        c.mean_l = static_cast<int>(a.sl / a.area);
        c.mean_r = static_cast<int>(a.sr / a.area);
        c.mean_g = static_cast<int>(a.sg / a.area);
        c.mean_b = static_cast<int>(a.sb / a.area);
        const double varx = a.sxx / a.area - c.cx * c.cx;
        const double vary = a.syy / a.area - c.cy * c.cy;
        c.sx = std::sqrt(std::max(0.0, varx));
        c.sy = std::sqrt(std::max(0.0, vary));
        if (a.pol == 1) dark_out->push_back(c);
        else if (a.pol == 2) bright_out->push_back(c);
    }
}

// ---------------------------------------------------------------------------
// 每轴尺度：像素 → 场地单位
// ---------------------------------------------------------------------------
// 为什么必须每轴分开：`--image-squash 0.75` 把 4:3 的画面纵向压成 16:9，
// 于是 x 和 y 的 px/单位 差 25%。用一个标量会让纵向速度系统性偏小 25%。
//: 单位约定**跟 `band_gap::Extent` 一致**：存的是"场地单位/像素"（乘上去就换算）。
//: 两处约定必须一样，否则一个乘一个除，速度会差 (比例)² —— 这种错最难看出来了。
struct Scale {
    double unit_per_px_x = 0, unit_per_px_y = 0;
    bool ok() const { return unit_per_px_x > 0 && unit_per_px_y > 0; }
};

//: 从"场地边框在画面里的位置"推尺度（不需要标记，只要有 bbox）。
inline Scale scale_from_bbox(int x0, int y0, int x1, int y1, double field_w, double field_h) {
    Scale s;
    if (x1 > x0 && y1 > y0 && field_w > 0 && field_h > 0) {
        s.unit_per_px_x = field_w / (x1 - x0);
        s.unit_per_px_y = field_h / (y1 - y0);
    }
    return s;
}

//: 成像平面上的位移 → 场地单位。
inline double to_field_dx(double dpx, const Scale& s) { return s.ok() ? dpx * s.unit_per_px_x : 0.0; }
inline double to_field_dy(double dpy, const Scale& s) { return s.ok() ? dpy * s.unit_per_px_y : 0.0; }

// ---------------------------------------------------------------------------
// 跟踪：质心帧间位移 → 速度
// ---------------------------------------------------------------------------
// 用户的原话是"根据轮廓在成像平面内的帧偏移量来计算速度"——这里就是那个。
//
// 做法：贪心最近邻关联（障碍之间至少隔几十像素，不需要匈牙利算法），
// 速度用指数平滑：v ← (1-α)v + α·Δp/Δt。α 取 0.5 是因为在 12 帧/秒下
// "两帧平均"正好把量化误差（±0.5 px）压到一半，又不至于把加减速抹平。
struct Track {
    int id = 0;
    Kind kind = Kind::Unknown;
    double cx = 0, cy = 0;         // 当前质心（像素）
    double vx = 0, vy = 0;         // 速度（px/s，成像平面）
    int area = 0;
    double w = 0, h = 0;
    int age = 1;                   // 连续命中帧数
    int misses = 0;                // 连续丢失帧数
};

struct TrackerParams {
    double max_match_dist = 40.0;  // px：超过这个距离就不认为是同一个物体
    double alpha = 0.5;            // 速度平滑系数
    int max_misses = 4;            // 连续 4 帧没配上就删掉
    bool walls_only = false;
};

class Tracker {
public:
    void reset() { tracks_.clear(); next_id_ = 1; }

    //: 输入是**已经分类好**的连通域（像素坐标），返回更新后的轨迹表。
    void update(const std::vector<Comp>& comps, const std::vector<Kind>& kinds, double dt,
                const TrackerParams& p, std::vector<Track>* out) {
        out->clear();
        std::vector<char> used(comps.size(), 0);
        for (auto& t : tracks_) {
            // 找最近的同类连通域
            int best = -1;
            double bestd = p.max_match_dist;
            for (size_t i = 0; i < comps.size(); ++i) {
                if (used[i]) continue;
                if (kinds[i] != t.kind) continue;
                const double dx = comps[i].cx - t.cx, dy = comps[i].cy - t.cy;
                const double d = std::sqrt(dx * dx + dy * dy);
                if (d < bestd) { bestd = d; best = static_cast<int>(i); }
            }
            if (best >= 0) {
                used[static_cast<size_t>(best)] = 1;
                const Comp& c = comps[static_cast<size_t>(best)];
                if (dt > 1e-6) {
                    const double inst_vx = (c.cx - t.cx) / dt;
                    const double inst_vy = (c.cy - t.cy) / dt;
                    t.vx = (1.0 - p.alpha) * t.vx + p.alpha * inst_vx;
                    t.vy = (1.0 - p.alpha) * t.vy + p.alpha * inst_vy;
                }
                t.cx = c.cx; t.cy = c.cy;
                t.area = c.area; t.w = c.w; t.h = c.h;
                ++t.age;
                t.misses = 0;
            } else {
                ++t.misses;
            }
        }
        // 删掉丢太久的
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [&](const Track& t) { return t.misses > p.max_misses; }),
                      tracks_.end());
        // 新出现的
        for (size_t i = 0; i < comps.size(); ++i) {
            if (used[i]) continue;
            if (kinds[i] == Kind::Unknown || kinds[i] == Kind::Line) continue;
            if (p.walls_only && kinds[i] != Kind::Wall) continue;
            Track t;
            t.id = next_id_++;
            t.kind = kinds[i];
            t.cx = comps[i].cx; t.cy = comps[i].cy;
            t.area = comps[i].area; t.w = comps[i].w; t.h = comps[i].h;
            tracks_.push_back(t);
        }
        *out = tracks_;
    }

    const std::vector<Track>& tracks() const { return tracks_; }

private:
    std::vector<Track> tracks_;
    int next_id_ = 1;
};

}   // namespace contour
}   // namespace vision_board

#endif   // VISION_BOARD_CONTOUR_HPP
