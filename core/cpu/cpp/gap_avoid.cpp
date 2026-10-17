// ===========================================================================
// gap_avoid.cpp —— `core/cpu/gap_avoid.py` 的逐行移植
//
// 逐段对照：每个函数头都写着它在 Python 里的对应函数名。改动任何一处之前，
// 先读 core/cpu/docs/CPP_PORT.md §2 的"三个会改变行为的陷阱"。
//
// 关于浮点：本文件**不**做任何代数化简。Python 写 `a*b + c*d` 就照抄
// `a*b + c*d`，不合并、不提取公因式、不换结合顺序 —— IEEE-754 下
// `(a+b)+c != a+(b+c)`，任何"优化"都会让 `tier` 在边界上分家。
// 编译器**可能**做 FMA 收缩（把 `a*b+c` 合成一条 fma），那会改变舍入结果。
// 因此构建时用 `-ffp-contract=off`（见 README 与测试里的编译命令）。
// ===========================================================================
#include "gap_avoid.hpp"

#include <algorithm>
#include <cstring>

namespace gap_avoid {

// ===========================================================================
// 0. 小工具
// ===========================================================================

double dot(Vec2 a, Vec2 b) {
    // Python: `float(a[0])*float(b[0]) + float(a[1])*float(b[1])`
    return a.x * b.x + a.y * b.y;
}

double wrap_pi(double angle) {
    // =========================================================================
    // Python: `(angle + math.pi) % (2.0*math.pi) - math.pi`
    //
    // ⚠ 陷阱 1（CPP_PORT.md §2.1）：Python 的 `%` 对负数返回**非负**结果，
    //   C++ 的 `std::fmod` 保留被除数的符号。差别只在 `angle < -pi` 时暴露，
    //   而 `a_new - a_old` 的范围是 `[-2pi, 2pi]`，所以**一定**会遇到：
    //
    //       angle = -4*pi
    //       Python  : (-3pi) % 2pi - pi = pi - pi = 0
    //       误解     : fmod(-3pi, 2pi) - pi = -pi - pi = -2pi   ← 错
    //
    //   必须补一次 `if (a < 0) a += 2pi`。
    // =========================================================================
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

/// Python `int(round(x))`。
/// ⚠ 陷阱 3（CPP_PORT.md 没写、移植时新发现的）：Python 的 `round(float)` 是
///   **四舍六入五成双**（banker's rounding）：`round(0.5)==0`、`round(1.5)==2`、
///   `round(2.5)==2`。C++ 的 `std::llround` 是**五入**（away from zero）：
///   `llround(0.5)==1`、`llround(2.5)==3`。两者在 .5 上给出**不同的分桶键**，
///   于是一堵墙可能被拆进两个桶 —— 正是这次要修的那个症状。
///
///   `std::nearbyint` 在默认舍入模式（FE_TONEAREST）下就是五成双，与 Python 一致。
///   不要换成 `std::round` / `std::llround` / `(int)(x+0.5)`。
static inline int py_round_int(double x) {
    return static_cast<int>(std::nearbyint(x));
}

// ===========================================================================
// 1. 几何纯函数
// ===========================================================================

double half_extent_along(const Rect& r, Vec2 d) {
    // Python: `_half_extent_along`
    const double ct = std::cos(r.rotation), st = std::sin(r.rotation);
    const Vec2 aw{ct, st};      // 局部 x 轴 = half_w 方向
    const Vec2 ah{-st, ct};     // 局部 y 轴 = half_h 方向
    return r.half_w * std::fabs(dot(d, aw)) + r.half_h * std::fabs(dot(d, ah));
}

double rect_sdf(const Rect& r, double px, double py, Vec2* grad) {
    // Python: `_rect_sdf`
    const double ct = std::cos(r.rotation), st = std::sin(r.rotation);
    const double dx = px - r.x, dy = py - r.y;
    const double lx = dx * ct + dy * st;          // 世界 -> 局部
    const double ly = -dx * st + dy * ct;
    const double qx = std::fabs(lx) - r.half_w;
    const double qy = std::fabs(ly) - r.half_h;
    const double outside = std::hypot(std::fmax(qx, 0.0), std::fmax(qy, 0.0));
    const double inside = std::fmin(std::fmax(qx, qy), 0.0);
    const double sdf = outside + inside;

    if (grad != nullptr) {
        const double sx = (lx >= 0.0) ? 1.0 : -1.0;
        const double sy = (ly >= 0.0) ? 1.0 : -1.0;
        double gx_l, gy_l;
        if (qx > 0.0 && qy > 0.0) {               // 角点区
            gx_l = sx * qx;
            gy_l = sy * qy;
            double n = std::hypot(gx_l, gy_l);
            if (n == 0.0) n = 1.0;                // Python 的 `or 1.0`
            gx_l /= n;
            gy_l /= n;
        } else if (qx > 0.0) {
            gx_l = sx; gy_l = 0.0;
        } else if (qy > 0.0) {
            gx_l = 0.0; gy_l = sy;
        } else {                                  // 在矩形内部：沿最浅的轴推出
            if (qx > qy) { gx_l = sx; gy_l = 0.0; }
            else         { gx_l = 0.0; gy_l = sy; }
        }
        grad->x = gx_l * ct - gy_l * st;
        grad->y = gx_l * st + gy_l * ct;
    }
    return sdf;
}

// ===========================================================================
// 2. group_walls
// ===========================================================================

namespace {

struct BucketEntry {
    Rect rect;
};

}  // namespace

int group_walls(const Rect* rects, size_t n, double px, double py,
                const AlgoConfig& cfg, Wall* out) {
    // Python: `group_walls`
    //
    // **不能用 group_id**：本场景里所有墙段的 group_id 都是 0。改用几何分组：
    // 同一堵墙的各段在墙厚方向上坐标几乎相同，把该坐标量化到 2 个世界单位一档
    // 作为分组键（相邻墙之间 >= 100 单位，不会误并）。
    if (n > MAX_RECTS) n = MAX_RECTS;
    const int nr = static_cast<int>(n);

    // 第一遍：给每条矩形算分桶键（保持输入顺序）。
    // ⚠ 陷阱 3：必须用 py_round_int，不能用 llround —— 两者对 .5 的处理不同
    //   （Python 五成双 / llround 五入），会给出不同的桶，进而把一堵墙拆开。
    struct Key { int k0, k1; };
    Key key_of[MAX_RECTS];
    for (int i = 0; i < nr; ++i) {
        const Rect& r = rects[i];
        const double nx = -std::sin(r.rotation);
        const double ny = std::cos(r.rotation);
        const double band = r.x * nx + r.y * ny;             // 墙厚方向的坐标
        key_of[i].k0 = py_round_int(r.rotation * 1000.0);
        key_of[i].k1 = py_round_int(band / 2.0);
    }

    // 第二遍：把**去重后**的键按元组字典序排好 —— 这就是 `sorted(buckets)`。
    //
    // ⚠ 陷阱 2（CPP_PORT.md §2.2）：**必须显式排序**。
    //   原因链：
    //       buckets 的遍历顺序 → walls 的顺序 → cands 的顺序
    //                                                 → min(cands, key=(prio,key_d))
    //   Python 的 `min` 在**并列时返回第一个**，而 Python 的 `dict` 迭代顺序 =
    //   插入顺序，`sorted(buckets)` 把它变成确定的全序。
    //   C++ 的 `std::unordered_map` 迭代顺序与插入顺序**无关** → 同一输入可能选出
    //   不同的墙。这里用数组 + `std::sort`，与 `sorted(buckets)` 语义对齐。
    Key uniq[MAX_RECTS];
    int n_uniq = 0;
    for (int i = 0; i < nr; ++i) {
        bool seen = false;
        for (int j = 0; j < n_uniq; ++j) {
            if (uniq[j].k0 == key_of[i].k0 && uniq[j].k1 == key_of[i].k1) { seen = true; break; }
        }
        if (!seen) uniq[n_uniq++] = key_of[i];
    }
    std::sort(uniq, uniq + n_uniq, [](const Key& a, const Key& b) {
        if (a.k0 != b.k0) return a.k0 < b.k0;
        return a.k1 < b.k1;                                  // 元组字典序
    });

    int n_walls = 0;
    // 一个桶的成员缓冲（≤ MAX_RECTS 条，约 5 KB 栈），**不是**每桶一个 ——
    // 64 个桶各带 64 条会是 330 KB 栈，嵌入式上直接爆栈。
    static thread_local BucketEntry m[MAX_RECTS];
    for (int bi = 0; bi < n_uniq && n_walls < MAX_WALLS; ++bi) {
        // 按**输入顺序**收集本桶成员 —— 与 Python 的 `dict` + `append` 一致，
        // 这样后面 `stable_sort(by x)` 的并列行为才相同。
        int cnt = 0;
        for (int i = 0; i < nr; ++i) {
            if (key_of[i].k0 == uniq[bi].k0 && key_of[i].k1 == uniq[bi].k1) {
                m[cnt++].rect = rects[i];
            }
        }
        if (cnt == 0) continue;

        // Python 的 `sorted(..., key=...)` 是**稳定排序**；`r.x` 相等时必须保持
        // 输入顺序，否则 `members[0].rotation`（û 的来源）会换人。
        std::stable_sort(m, m + cnt,
                         [](const BucketEntry& a, const BucketEntry& c) {
                             return a.rect.x < c.rect.x;
                         });

        const double ct = std::cos(m[0].rect.rotation);
        const double st = std::sin(m[0].rect.rotation);
        Vec2 u_hat{ct, st};                          // half_w 轴 = 墙的跨度方向
        // 规范化 û 让它大致指向 +x，这样 e_lo/e_hi、"左侧/右侧"才有稳定含义
        if (u_hat.x < -EPS || (std::fabs(u_hat.x) <= EPS && u_hat.y < 0.0)) {
            u_hat.x = -u_hat.x; u_hat.y = -u_hat.y;
        }
        Vec2 n_hat{-u_hat.y, u_hat.x};

        // ⚠ 均值用的是**整桶**（不只是前 2 段）—— `members[:2]` 只发生在建 Wall 时
        double mx = 0.0, my = 0.0;
        for (int i = 0; i < cnt; ++i) { mx += m[i].rect.x; my += m[i].rect.y; }
        mx /= static_cast<double>(cnt);
        my /= static_cast<double>(cnt);
        const Vec2 to_player{px - mx, py - my};

        if (cfg.has_forward) {
            // 固定行驶轴：n̂ 从墙指向车，恒等于 -forward。**必须这样钉死**：
            // 静态墙的法向速度是 0，"车在墙哪一侧"在车正好走到墙线上时是退化的
            // （点积 ≈ 0），每帧重判的 n̂ 会来回翻，"前进"跟着隔帧反号，车原地抖。
            n_hat.x = -cfg.forward.x;
            n_hat.y = -cfg.forward.y;
        } else if (std::fabs(dot(n_hat, to_player)) > TOL) {
            if (dot(n_hat, to_player) < 0.0) { n_hat.x = -n_hat.x; n_hat.y = -n_hat.y; }
        } else {
            // 退化：玩家正好落在墙所在的那条线上，用墙自己的速度定向
            double vx = 0.0, vy = 0.0;
            for (int i = 0; i < cnt; ++i) { vx += m[i].rect.vx; vy += m[i].rect.vy; }
            vx /= static_cast<double>(cnt);
            vy /= static_cast<double>(cnt);
            if (dot(Vec2{vx, vy}, n_hat) < 0.0) { n_hat.x = -n_hat.x; n_hat.y = -n_hat.y; }
        }

        Wall& w = out[n_walls++];
        w.n_rects = (cnt < 2) ? cnt : 2;             // `members[:2]`
        for (int i = 0; i < w.n_rects; ++i) w.rects[i] = m[i].rect;   // 按值拷贝
        w.u_hat = u_hat;
        w.n_hat = n_hat;
    }
    return n_walls;
}

// ===========================================================================
// 3. GapMemory
// ===========================================================================

void Ring::push(double ti, double si) {
    // = `deque(maxlen=24).append(...)`：满了先丢最旧的
    const int i = (head + n) % RING_CAP;
    t[i] = ti;
    s[i] = si;
    if (n < RING_CAP) n += 1;
    else head = (head + 1) % RING_CAP;
}

WallMemory& GapMemory::get(int key) {
    // = Python `self.walls.setdefault(key, WallMemory())`
    for (int i = 0; i < n_slots; ++i) {
        if (keys[i] == key) return slots[i];
    }
    const int i = n_slots++;                          // 上界由 MAX_WALLS 保证
    keys[i] = key;
    slots[i] = WallMemory{};
    return slots[i];
}

void GapMemory::match_keys(const double* bands, int n, int* out, double tol) {
    // = Python `GapMemory.match_keys`
    PrevEntry prev_local[MAX_KEYS];
    int n_prev_local = n_prev;
    if (n_prev_local > MAX_KEYS) n_prev_local = MAX_KEYS;
    std::memcpy(prev_local, prev, sizeof(PrevEntry) * static_cast<size_t>(n_prev_local));

    if (n > MAX_KEYS) n = MAX_KEYS;
    for (int j = 0; j < n; ++j) {
        const double band = bands[j];
        int pick_i = -1;
        double pick_d = 0.0;
        int pick_key = 0;
        for (int i = 0; i < n_prev_local; ++i) {
            const double dd = std::fabs(prev_local[i].band - band);
            if (pick_i < 0 || dd < pick_d) { pick_d = dd; pick_i = i; pick_key = prev_local[i].key; }
        }
        if (pick_i >= 0 && pick_d <= tol) {
            out[j] = pick_key;
            // 一堵旧墙只能被认领一次：Python 用的是 `prev.pop(pick[1])`
            for (int i = pick_i; i + 1 < n_prev_local; ++i) prev_local[i] = prev_local[i + 1];
            n_prev_local -= 1;
        } else {
            out[j] = next_key;
            next_key += 1;
        }
    }
    // `self._prev = list(zip(bands, keys))`
    n_prev = n;
    for (int j = 0; j < n; ++j) { prev[j].band = bands[j]; prev[j].key = out[j]; }
}

int GapMemory::single_segment_frames() const {
    int s = 0;
    for (int i = 0; i < n_slots; ++i) s += slots[i].n_single_frames;
    return s;
}

// ===========================================================================
// 4. reconstruct_aperture
// ===========================================================================

Aperture reconstruct_aperture(const Wall& wall, WallMemory& mem, double r_p,
                              double margin, double field_w,
                              bool has_global_gap, double global_gap) {
    // Python: `reconstruct_aperture`
    //
    // 两段都在：e_lo/e_hi = 两条内缘取 min/max，gap = e_hi - e_lo。
    // 只有一段可见（实测会发生）：用"别处见过"的缺口宽度补出来 ——
    //   1. 优先用这堵墙自己历史上量到过的宽度；
    //   2. 否则用任意一堵墙量到过的宽度（同一场景 gap_width 是同一个参数）；
    //   3. 缺口在可见段的哪一侧：有本墙记忆就按记忆方位，没记忆就看哪一侧放得下。
    const Rect* r_sorted[2] = {nullptr, nullptr};
    const int nr = (wall.n_rects < 2) ? wall.n_rects : 2;
    for (int i = 0; i < nr; ++i) r_sorted[i] = &wall.rects[i];
    // Python: `sorted(wall.rects, key=lambda r: _dot((r.x, r.y), wall.u_hat))`
    // 元素数 <= 2，手写插入排序即可（保持稳定：相等时不动）
    for (int i = 1; i < nr; ++i) {
        const Rect* key = r_sorted[i];
        const double kd = dot(Vec2{key->x, key->y}, wall.u_hat);
        int j = i - 1;
        while (j >= 0 && dot(Vec2{r_sorted[j]->x, r_sorted[j]->y}, wall.u_hat) > kd) {
            r_sorted[j + 1] = r_sorted[j];
            j -= 1;
        }
        r_sorted[j + 1] = key;
    }
    const Vec2 u_hat = wall.u_hat;

    double e_lo, e_hi, gap;
    bool complete;
    if (nr >= 2) {
        const Rect& rl = *r_sorted[0];
        const Rect& rr = *r_sorted[1];
        const double inner_a = dot(Vec2{rl.x, rl.y}, u_hat) + half_extent_along(rl, u_hat);
        const double inner_b = dot(Vec2{rr.x, rr.y}, u_hat) - half_extent_along(rr, u_hat);
        e_lo = (inner_a < inner_b) ? inner_a : inner_b;
        e_hi = (inner_a < inner_b) ? inner_b : inner_a;
        gap = e_hi - e_lo;
        if (gap > 0.0) {
            mem.has_gap_width = true;
            mem.gap_width_seen = gap;
            mem.has_gap_center = true;
            mem.gap_center_seen = 0.5 * (e_lo + e_hi);
        }
        complete = true;
    } else {
        mem.n_single_frames += 1;
        const Rect& r = *r_sorted[0];
        const double proj = dot(Vec2{r.x, r.y}, u_hat);
        const double half = half_extent_along(r, u_hat);
        const bool have_g = mem.has_gap_width || has_global_gap;
        const double g_mem = mem.has_gap_width ? mem.gap_width_seen : global_gap;
        if (!have_g || g_mem <= 0.0) {
            Aperture ap;
            ap.e_lo = 0.0; ap.e_hi = 0.0; ap.gap_raw = 0.0; ap.center = 0.0;
            ap.usable_half = -INFINITY; ap.complete = false;
            return ap;
        }

        bool on_left;
        if (mem.has_gap_center) {
            on_left = proj < mem.gap_center_seen;
        } else {
            const double free_left = proj - half;
            const double free_right = field_w - (proj + half);
            if (free_left + TOL >= g_mem && free_left >= free_right) {
                on_left = false;                 // 缺口在左边 => 可见的是右段
            } else if (free_right + TOL >= g_mem) {
                on_left = true;                  // 缺口在右边 => 可见的是左段
            } else {
                Aperture ap;
                ap.e_lo = 0.0; ap.e_hi = 0.0; ap.gap_raw = 0.0; ap.center = 0.0;
                ap.usable_half = -INFINITY; ap.complete = false;
                return ap;
            }
        }

        if (on_left) { e_lo = proj + half; e_hi = e_lo + g_mem; }
        else         { e_hi = proj - half; e_lo = e_hi - g_mem; }
        gap = e_hi - e_lo;
        complete = false;
    }

    const double lo = std::fmax(e_lo, 0.0);
    const double hi = std::fmin(e_hi, field_w);
    const double usable_w = hi - lo;

    Aperture ap;
    ap.e_lo = lo;
    ap.e_hi = hi;
    ap.gap_raw = gap;
    ap.center = 0.5 * (lo + hi);
    ap.usable_half = 0.5 * usable_w - r_p - margin;
    ap.complete = complete;
    return ap;
}

// ===========================================================================
// 5. plan
// ===========================================================================

namespace {

Vec2 band_point(const Wall& wall, double lateral, double band) {
    // Python: `_band_point`。P = lateral*û + band*n̂
    return Vec2{lateral * wall.u_hat.x + band * wall.n_hat.x,
                lateral * wall.u_hat.y + band * wall.n_hat.y};
}

}  // namespace

Decision plan(const WorldView& world, GapMemory& mem, const AlgoConfig& cfg,
              double horizon_remaining) {
    // Python: `plan`
    const double* p = world.player;
    const double px = p[P_X], py = p[P_Y];
    const double pvx = p[P_VX], pvy = p[P_VY];
    const double r_p = p[P_RADIUS];
    const double v_max = p[P_SPEED];
    const double dt = world.dt;
    const int64_t step = world.step_index;
    const double field_w = world.field_w;
    mem.step_index = step;

    Wall walls[MAX_WALLS];
    const int n_walls = group_walls(world.rects, world.n_rects, px, py, cfg, walls);

    Decision d;
    if (n_walls == 0) {
        mem.match_keys(nullptr, 0, nullptr);
        d.tier = "IDLE";
        d.reason = "no_wall";
        return d;
    }

    double bands[MAX_WALLS];
    for (int i = 0; i < n_walls; ++i) {
        double s = 0.0;
        for (int j = 0; j < walls[i].n_rects; ++j) {
            const Rect& r = walls[i].rects[j];
            s += dot(Vec2{r.x, r.y}, walls[i].n_hat);
        }
        bands[i] = s / static_cast<double>(walls[i].n_rects);
    }
    int keys[MAX_WALLS];
    mem.match_keys(bands, n_walls, keys);

    // --- 选最近的那堵正在逼近的墙；每一堵都要重建缺口（学习全局缺口宽度）---
    struct Cand {
        const Wall* wall;
        int    key;
        double t_arr, clearance, v_n, front, thick;
        Aperture ap;
        int    prio;
        double key_d;
    };
    Cand cands[MAX_WALLS];
    int n_cands = 0;

    for (int i = 0; i < n_walls; ++i) {
        const Wall& w = walls[i];
        const int key = keys[i];

        double front = -INFINITY;
        for (int j = 0; j < w.n_rects; ++j) {
            const Rect& r = w.rects[j];
            const double v = dot(Vec2{r.x, r.y}, w.n_hat) + half_extent_along(r, w.n_hat);
            if (v > front) front = v;
        }
        double hmax = -INFINITY;
        for (int j = 0; j < w.n_rects; ++j) {
            const double v = half_extent_along(w.rects[j], w.n_hat);
            if (v > hmax) hmax = v;
        }
        const double thick = 2.0 * hmax;
        const double back = front - thick;
        const double pn = dot(Vec2{px, py}, w.n_hat);

        double vn_wall = 0.0;
        for (int j = 0; j < w.n_rects; ++j) {
            const Rect& r = w.rects[j];
            vn_wall += dot(Vec2{r.vx, r.vy}, w.n_hat);
        }
        vn_wall /= static_cast<double>(w.n_rects);
        const double v_n = vn_wall - dot(Vec2{pvx, pvy}, w.n_hat);

        // 必须按玩家半径膨胀墙带、分三段判断
        const double near_ = front + r_p;
        const double far_ = back - r_p;
        double t_arr;
        int prio;
        double key_d;
        if (pn > near_) {
            t_arr = (v_n > EPS) ? (pn - near_) / v_n : INFINITY;
            prio = 1; key_d = pn - near_;        // 还在前面：越近越优先
        } else if (pn >= far_) {
            t_arr = 0.0;                          // 玩家的圆正压在墙带上
            prio = 0; key_d = 0.0;                // 正压着它 => 必须继续盯住这一堵
        } else {
            t_arr = INFINITY;                     // 整面墙已经在身后
            prio = 2; key_d = 0.0;                // 身后 => 不再是约束
        }
        //
        // 选目标墙**必须按位置**（prio/key_d），不能按 t_arr：
        // t_arr = d_n / v_n 里含**车自己的速度**。静态墙场景下，车只要往后动一点点
        // （v_n <= 0），所有墙的 t_arr 同时变成 inf，min() 就会随便挑一堵 ——
        // 目标在两道缺口之间来回跳（实测 ±175px），输出的速度方向隔帧反号。

        double sdf = 0.0;
        for (int j = 0; j < w.n_rects; ++j) {
            const double v = rect_sdf(w.rects[j], px, py, nullptr);
            if (j == 0 || v < sdf) sdf = v;
        }
        const double clearance = sdf - r_p - cfg.safety_margin;

        WallMemory& wmem = mem.get(key);
        const Aperture ap_w = reconstruct_aperture(w, wmem, r_p, cfg.safety_margin,
                                                   field_w, mem.has_global_gap,
                                                   mem.global_gap_width);
        // ⚠ 这一步在**循环里**：本帧靠后的一堵墙能看到本帧靠前那堵刚学到的宽度。
        //   顺序 = `sorted(buckets)` 的顺序，由上面的显式排序保证。别挪出循环。
        if (ap_w.complete && ap_w.gap_raw > 0.0) {
            mem.has_global_gap = true;
            mem.global_gap_width = ap_w.gap_raw;
        }

        Cand& c = cands[n_cands++];
        c.wall = &w; c.key = key; c.t_arr = t_arr; c.clearance = clearance;
        c.v_n = v_n; c.front = front; c.thick = thick; c.ap = ap_w;
        c.prio = prio; c.key_d = key_d;
    }

    // Python: `min(cands, key=lambda c: (c[8], c[9]))`
    // ⚠ 陷阱 2 的落点：并列时返回**第一个** —— 只有 cands 的填充顺序两侧一致，
    //   这里的 `strictly less` 才与 Python 的 `min` 等价。
    int pick = 0;
    for (int i = 1; i < n_cands; ++i) {
        const Cand& a = cands[i];
        const Cand& b = cands[pick];
        bool better;
        if (a.prio != b.prio) better = a.prio < b.prio;
        else                  better = a.key_d < b.key_d;
        if (better) pick = i;
    }
    const Wall& wall = *cands[pick].wall;
    const int key = cands[pick].key;
    const double t_arr = cands[pick].t_arr;
    const double clearance = cands[pick].clearance;
    const Aperture& ap = cands[pick].ap;
    const double front = cands[pick].front;

    WallMemory& wmem = mem.get(key);
    const double lat = dot(Vec2{px, py}, wall.u_hat);

    if (ap.usable_half <= 0.0) {
        // 推不出缺口 / 膨胀后装不下人：朝远离最近那段墙的方向横移兜底，
        // 别把自己顶到场地边界上。
        int nearest = 0;
        double best = 0.0;
        for (int j = 0; j < wall.n_rects; ++j) {
            const double v = rect_sdf(wall.rects[j], px, py, nullptr);
            if (j == 0 || v < best) { best = v; nearest = j; }
        }
        const Rect& r_near = wall.rects[nearest];
        const double proj_near = dot(Vec2{r_near.x, r_near.y}, wall.u_hat);
        double sgn = (lat < proj_near) ? 1.0 : -1.0;
        if (!(0.0 <= lat + sgn * 40.0 && lat + sgn * 40.0 <= field_w)) sgn = -sgn;

        d.wx = sgn * wall.u_hat.x;
        d.wy = sgn * wall.u_hat.y;
        d.tier = "NO_GAP";
        d.reason = "aperture_unknown_flee";
        d.t_arr = t_arr;
        d.clearance = clearance;
        d.gap = ap.gap_raw;
        d.usable_half = ap.usable_half;
        d.e_lo = ap.e_lo;
        d.e_hi = ap.e_hi;
        d.n_rects = wall.n_rects;
        d.gap_line[0] = band_point(wall, ap.e_lo, front);
        d.gap_line[1] = band_point(wall, ap.e_hi, front);
        return d;
    }

    // --- 缺口轨迹：按墙分别估计（线性拟合 + 残差包络）---
    wmem.ring.push(static_cast<double>(step), ap.center);
    double s_hat = 0.0, resid = 0.0;
    if (wmem.ring.size() >= 2) {
        const double t0 = wmem.ring.t_at(0);
        const int n = wmem.ring.size();
        double st = 0.0, stt = 0.0, ss = 0.0, sts = 0.0;
        for (int i = 0; i < n; ++i) {
            const double tt = (wmem.ring.t_at(i) - t0) * dt;
            const double si = wmem.ring.s_at(i);
            st += tt;
            stt += tt * tt;
            ss += si;
            sts += tt * si;
        }
        const double den = n * stt - st * st;
        s_hat = (std::fabs(den) < EPS) ? 0.0 : (n * sts - st * ss) / den;
        const double b = (ss - s_hat * st) / n;
        for (int i = 0; i < n; ++i) {
            const double tt = (wmem.ring.t_at(i) - t0) * dt;
            const double v = std::fabs(wmem.ring.s_at(i) - (b + s_hat * tt));
            if (v > resid) resid = v;
        }
    }

    // Python 里这三个是闭包；这里直接写成立即数 / lambda 无捕获形式
    struct Ctx { double center, s_hat, resid, lat, v_max, usable_half; };
    const Ctx cx{ap.center, s_hat, resid, lat, v_max, ap.usable_half};
    auto s_lo = [&cx](double t) {
        return cx.center + std::fmin(0.0, cx.s_hat) * t - cx.resid * t;
    };
    auto s_hi = [&cx](double t) {
        return cx.center + std::fmax(0.0, cx.s_hat) * t + cx.resid * t;
    };
    auto reach = [&cx](double t) {
        return cx.v_max * std::fmax(0.0, t);           // README 判据 ②（不开加速度）
    };

    // Python 这里还有一行 `H = min(float(horizon_remaining), cfg.max_window)`，
    // **但它在整个 plan() 里从未被使用**（grep 可验）。这里保留同名变量只为
    // 对照时一眼能看出"确实没用"，不要据此以为有窗口逻辑。
    const double H = std::fmin(horizon_remaining, cfg.max_window);
    (void)H;

    // --- 瞄准与判定 ------------------------------------------------------
    // 关键判据（README :300）：墙到达我这条线时，我必须已经在缺口里。
    //
    // 瞄准时刻不取 t_arr 而取 min(t_arr, aim_lookahead)：缺口会动时，用线性拟合
    // 外推 t_arr（开局可达 4 秒）得到的位置毫无意义；短视距 + 每步重算 = 追踪。
    // 瞄准点用**拦截时间**求，而不是固定前瞻：
    //     T 满足 |s(T) - lat| = v_max * T
    double t_aim = std::fabs(0.5 * (s_lo(0.0) + s_hi(0.0)) - cx.lat)
                   / std::fmax(cx.v_max, EPS);
    for (int it = 0; it < 4; ++it) {
        const double cand = std::fabs(0.5 * (s_lo(t_aim) + s_hi(t_aim)) - cx.lat)
                            / std::fmax(cx.v_max, EPS);
        t_aim = std::isfinite(t_arr) ? std::fmin(t_arr, cand) : cand;
    }
    const double aim_lat = 0.5 * (s_lo(t_aim) + s_hi(t_aim));
    const double d_lat = aim_lat - cx.lat;
    const double need = std::fmax(0.0, std::fabs(d_lat) - cx.usable_half);
    const double sgn_lat = (d_lat >= 0.0) ? 1.0 : -1.0;

    const double band = front;
    const Vec2 gl[2] = {band_point(wall, ap.e_lo, band),
                        band_point(wall, ap.e_hi, band)};

    // 横向分量（分级）。不直接 return，因为下面还要叠加前后分量。
    double u_mag;
    const char* tier;
    const char* reason;
    if (std::fabs(d_lat) <= cfg.dead_frac * cx.usable_half) {
        tier = "HOLD"; reason = "gap_will_pass_me"; u_mag = 0.0;
    } else if (std::fabs(d_lat) <= cfg.align_frac * cx.usable_half) {
        u_mag = std::fmin(1.0, std::fabs(d_lat) / std::fmax(cx.usable_half, EPS));
        tier = "TRIM"; reason = "recenter";
    } else {
        u_mag = 1.0; tier = "ALIGN"; reason = "move_to_gap";
    }

    // 纯横向来不来得及。前后可控时"减速/停住"能把这条救回来，所以不判 PANIC。
    if (std::isfinite(t_arr) && need > reach(t_arr) && !cfg.front_back) {
        u_mag = 1.0; tier = "PANIC"; reason = "unreachable_before_arrival";
    }

    // --- 前后分量 -------------------------------------------------------
    // n̂ 从墙指向车 => 前进（朝墙）= −n̂，后退（远离墙）= +n̂。
    const Vec2 fwd{-wall.n_hat.x, -wall.n_hat.y};
    const double d_n = std::fmax(0.0, dot(Vec2{px, py}, wall.n_hat) - (front + r_p));

    const double u_speed = u_mag * cx.v_max;
    const double w_cap = std::sqrt(std::fmax(0.0, cx.v_max * cx.v_max - u_speed * u_speed));
    double w;
    if (cfg.front_back) {
        const double t_align = (need > 1e-6) ? need / std::fmax(u_speed, EPS) : 0.0;
        const double room = std::fmax(0.0, d_n - cfg.hold_clearance);
        const double t_reach = room / std::fmax(w_cap, EPS);
        if (t_reach >= t_align) {
            w = w_cap;                                       // 时间够：全速通过
            if (std::strcmp(tier, "PANIC") == 0) { tier = "ALIGN"; reason = "move_to_gap"; }
        } else {
            // 时间不够：减速到"恰好对齐完成时抵达"，必要时直接停住等
            w = std::fmin(w_cap, room / std::fmax(t_align, EPS));
            tier = (w > 1e-6) ? "SLOW" : "STOP";
            reason = "hold_to_align";
        }
    } else {
        // 不允许前后控制：**油门是固定的**（车不能停、不能减速），横向只能用剩下的预算。
        w = std::fmin(cfg.fwd_speed * cx.v_max, cx.v_max);
        const double u_cap = std::sqrt(std::fmax(0.0, cx.v_max * cx.v_max - w * w));
        u_mag = std::fmin(u_mag, u_cap / std::fmax(cx.v_max, EPS));
        if (cfg.fwd_speed > 0.0) { tier = "CRUISE"; }
    }

    // ⚠ 契约：v_max > 0（见 hpp 前置条件）。Python 在 v_max == 0 时这里会
    //   抛 ZeroDivisionError；C++ 不能抛，于是显式定义成 0（不可达路径）。
    const double w_frac = (cx.v_max != 0.0) ? w / cx.v_max : 0.0;
    double wx = u_mag * sgn_lat * wall.u_hat.x + w_frac * fwd.x;
    double wy = u_mag * sgn_lat * wall.u_hat.y + w_frac * fwd.y;

    // 转向限速：把这一帧的方向相对上一帧的夹角限住。
    if (cfg.max_turn_rate > 0.0 && mem.has_last_dir) {
        const double mag = std::hypot(wx, wy);
        if (mag > EPS) {
            double a_new = std::atan2(wy, wx);
            const double a_old = std::atan2(mem.last_dir.y, mem.last_dir.x);
            const double dd = wrap_pi(a_new - a_old);        // 见 wrap_pi 的注释
            // ⚠ `math.radians(x)` == `x * (M_PI/180.0)`（CPython 的 degToRad），
            //   不是 `x/180.0*M_PI` —— 两种写法末位可能不同。
            const double lim = (M_PI / 180.0) * cfg.max_turn_rate * dt;
            if (std::fabs(dd) > lim) {
                a_new = a_old + std::copysign(lim, dd);
                wx = mag * std::cos(a_new);
                wy = mag * std::sin(a_new);
            }
        }
    }
    // 边界余量：只掐"朝外"的分量（Python 同步实现，见 limit_edges）
    bool edge_limited = false;
    if (cfg.edge_margin > 0.0) {
        const double wx0 = wx, wy0 = wy;
        limit_edges(wx, wy, world.player, world.field_w, world.field_h, cfg.edge_margin);
        edge_limited = (wx != wx0) || (wy != wy0);
    }

    if (std::hypot(wx, wy) > EPS) {
        mem.has_last_dir = true;
        mem.last_dir = Vec2{wx, wy};
    }

    d.edge_limited = edge_limited;
    d.wx = wx; d.wy = wy;
    d.tier = tier; d.reason = reason;
    d.t_arr = t_arr;
    d.clearance = clearance;
    d.gap = ap.gap_raw;
    d.usable_half = ap.usable_half;
    d.e_lo = ap.e_lo;
    d.e_hi = ap.e_hi;
    d.aim_lat = aim_lat;
    d.s_hat = s_hat;
    d.n_rects = wall.n_rects;
    d.gap_line[0] = gl[0];
    d.gap_line[1] = gl[1];
    d.aim_world = band_point(wall, aim_lat, band);
    d.wall_band = band;
    return d;
}

void limit_edges(double& wx, double& wy, const double* player,
                 double field_w, double field_h, double margin) {
    // Python: `limit_edges` —— 逐行照抄，运算顺序不许变（逐位对账）
    if (margin <= 0.0 || field_w <= 0.0 || field_h <= 0.0) return;
    double lim = std::min(margin, 0.5 * field_w);
    lim = std::min(lim, 0.5 * field_h);
    if (lim <= 0.0) return;
    const double x = player[P_X];
    const double y = player[P_Y];
    const double r = player[P_RADIUS];
    if (wx < 0.0) {
        wx *= std::max(0.0, std::min(1.0, (x - r) / lim));
    } else if (wx > 0.0) {
        wx *= std::max(0.0, std::min(1.0, ((field_w - r) - x) / lim));
    }
    if (wy < 0.0) {
        wy *= std::max(0.0, std::min(1.0, (y - r) / lim));
    } else if (wy > 0.0) {
        wy *= std::max(0.0, std::min(1.0, ((field_h - r) - y) / lim));
    }
}

Vec2 to_command(const Decision& d) {
    // Python: `to_command`
    if (std::hypot(d.wx, d.wy) < EPS) return Vec2{0.0, 0.0};
    return Vec2{d.wx, d.wy};
}

}  // namespace gap_avoid
