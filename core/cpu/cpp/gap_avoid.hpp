// ===========================================================================
// gap_avoid.hpp —— `core/cpu/gap_avoid.py` 的 C++ 移植（PS 侧跑的那一份）
//
// 契约：**同一输入 → 同一行为**。验收口径（见 core/cpu/docs/CPP_PORT.md 开头）：
//   * `tier` / 分支走法      —— 逐位一致（全是比较与整数逻辑）
//   * `+ - * /`、`sqrt`      —— 逐位一致（IEEE-754 要求正确舍入）
//   * `hypot`/`atan2`/`cos`/`sin` —— **不能要求逐位**，这些函数不要求正确舍入；
//                              口径是相对误差 < 1e-9
//
// 本文件**不含任何平台 / 仿真器依赖**：`WorldView` 是唯一的输入口。
// 移植范围与陷阱清单见 `core/cpu/docs/CPP_PORT.md`；这里只挑出必须照抄语义的
// 三处（每一处都在 .cpp 里有对应的注释）：
//   1. `wrap_pi()`  —— Python `%` 对负数返回非负，C++ `fmod` 保留符号；
//   2. `group_walls()` 的分桶**显式排序**，否则 argmin 并列时的胜负依赖
//      容器迭代顺序（`unordered_map` 与 Python dict 不同）；
//   3. `int(round(x))` —— Python 的 `round()` 是**四舍六入五成双**，
//      C++ 的 `std::llround` 是五入，两者对 .5 给出不同结果。
//
// 前置条件（越界即未定义，调用方保证）：
//   * `world.player[P_SPEED] > 0` —— Python 在 `w_frac = w / v_max` 处会抛
//     `ZeroDivisionError`，C++ 不能抛，所以这里把 v_max == 0 视为**调用方违约**。
//   * `world.n_rects <= MAX_RECTS`。
//
// 稳态零分配：`plan()` 只用调用方持有的定长缓冲，不 new / 不 malloc。
// ===========================================================================
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace gap_avoid {

// 玩家状态向量下标（与 Python 侧 `P_X..P_ALIVE` 一致）
enum : int { P_X = 0, P_Y = 1, P_VX = 2, P_VY = 3, P_RADIUS = 4, P_SPEED = 5, P_ALIVE = 6 };

constexpr int    SHAPE_RECT = 1;
constexpr double EPS = 1e-9;
//: 几何比较容差。缺口宽度会算成 200.00000000000003 而自由空间是 200.0，
//: 直接用 >= 会因为这个 3e-14 的误差判成"放不下"，算法整帧摆烂。
constexpr double TOL = 1e-6;

// 定长缓冲上界。`MAX_RECTS` 取平台障碍池容量量级；实测墙数是个位数。
constexpr int MAX_RECTS = 64;
constexpr int MAX_WALLS = 64;
constexpr int RING_CAP  = 24;   // = Python 的 `deque(maxlen=24)`
//: ⚠ Python 的 `GapMemory._prev` 是**普通 list**（没有 maxlen），长度 = 墙数
//: <= 矩形数。这里按矩形数上界给。
constexpr int MAX_KEYS  = MAX_RECTS;

struct Vec2 {
    double x = 0.0, y = 0.0;
};

struct Rect {
    int32_t ent_id = 0;
    double  x = 0.0, y = 0.0;
    double  vx = 0.0, vy = 0.0;
    double  half_w = 0.0, half_h = 0.0;
    double  rotation = 0.0;
    double  angular_velocity = 0.0;
};

struct WorldView {
    const double* player = nullptr;      // 长度 7：[x, y, vx, vy, radius, speed, alive]
    const Rect*   rects = nullptr;
    size_t        n_rects = 0;
    double        dt = 0.0;
    int64_t       step_index = 0;
    double        field_w = 0.0;
    double        field_h = 0.0;
};

struct AlgoConfig {
    double safety_margin = 6.0;
    double max_window = 8.0;
    double align_frac = 0.6;
    double dead_frac = 0.05;
    bool   allow_cross = false;
    double aim_lookahead = 0.25;
    bool   front_back = false;
    double fwd_speed = 0.0;
    double hold_clearance = 10.0;
    //: Python 是 `Optional[tuple]`；这里用 `has_forward` 显式区分"没给"和"(0,0)"。
    bool   has_forward = false;
    Vec2   forward{0.0, 0.0};
    double max_turn_rate = 0.0;
};

/// 一堵缺口墙：最多 2 段共线矩形 + 它的墙坐标系。
/// `u_hat` = 跨度方向（缺口沿它排布）；`n_hat` = 厚度方向，**从墙指向玩家**。
///
/// ⚠ `rects` 是**按值持有**的副本（= Python 的 `tuple(members[:2])`）。
///   早先写成 `const Rect*` 指向 `group_walls()` 的局部数组，函数一返回就悬垂 ——
///   这种 bug 在 -O0 下"看起来能跑"，换个优化级别就随机崩。
struct Wall {
    Rect  rects[2];
    int   n_rects = 0;
    Vec2  u_hat{0.0, 0.0};
    Vec2  n_hat{0.0, 0.0};
};

struct Aperture {
    double e_lo = 0.0;
    double e_hi = 0.0;
    double gap_raw = 0.0;
    double center = 0.0;
    double usable_half = -INFINITY;
    bool   complete = false;
};

struct Decision {
    double wx = 0.0, wy = 0.0;
    const char* tier = "IDLE";
    const char* reason = "";
    double t_arr = INFINITY;
    double clearance = INFINITY;
    double gap = NAN;
    double usable_half = NAN;
    double e_lo = NAN, e_hi = NAN;
    double aim_lat = NAN;
    double s_hat = 0.0;
    int    n_rects = 0;
    Vec2   gap_line[2]{{NAN, NAN}, {NAN, NAN}};
    Vec2   aim_world{NAN, NAN};
    double wall_band = NAN;
};

/// 定长环形缓冲，语义 = `collections.deque(maxlen=RING_CAP)`：
/// 迭代顺序恒为**从最旧到最新**，满了以后丢最旧的。
struct Ring {
    double t[RING_CAP]{};
    double s[RING_CAP]{};
    int n = 0;        // 当前元素个数（<= RING_CAP）
    int head = 0;     // 最旧元素的下标

    void push(double ti, double si);
    int  size() const { return n; }
    double t_at(int i) const { return t[(head + i) % RING_CAP]; }
    double s_at(int i) const { return s[(head + i) % RING_CAP]; }
};

/// 每堵墙的跨帧记忆。`has_gap_width/has_gap_center` 对应 Python 的 `is not None`
/// —— **不能用 0/NaN 代替**：0 是合法量到的宽度，语义不同。
struct WallMemory {
    Ring   ring;
    bool   has_gap_width = false;
    double gap_width_seen = 0.0;
    bool   has_gap_center = false;
    double gap_center_seen = 0.0;
    int    n_single_frames = 0;
};

/// 跨帧记忆。墙身份按"上一帧最近的墙"关联（不能用 min(ent_id)：
/// 本场景所有墙段的 group_id 都是 0，而 ent_id 每帧会变）。
struct GapMemory {
    // 墙身份表：定长数组 + 线性查找（墙数量个位数，不用 unordered_map）
    int        keys[MAX_WALLS]{};
    WallMemory slots[MAX_WALLS]{};
    int        n_slots = 0;

    bool   has_global_gap = false;
    double global_gap_width = 0.0;
    int64_t step_index = -1;
    int    next_key = 1;

    struct PrevEntry { double band; int key; };
    PrevEntry prev[MAX_KEYS]{};
    int       n_prev = 0;

    bool has_last_dir = false;
    Vec2 last_dir{0.0, 0.0};

    /// = Python `GapMemory.get(key)`：取不到就建一个。
    WallMemory& get(int key);

    /// = Python `GapMemory.match_keys(bands, tol)`。`out` 必须能放 `n` 个 int。
    void match_keys(const double* bands, int n, int* out, double tol = 24.0);

    /// = Python `GapMemory.single_segment_frames` 属性。
    int single_segment_frames() const;
};

// ---------------------------------------------------------------------------
// 纯函数（移植顺序 §5 的第 1 步：这四个最容易对）
// ---------------------------------------------------------------------------

/// 归一化到 `[-pi, pi)`。**半开区间**：`wrap_pi(pi) == -pi`。
/// ⚠ 必须照抄 Python 的 `%` 语义（对负数返回非负），不能用裸 `fmod`。
double wrap_pi(double angle);
double dot(Vec2 a, Vec2 b);

/// 旋转矩形在单位方向 d 上的半投影长度（精确支撑函数，不是小角度近似）。
double half_extent_along(const Rect& r, Vec2 d);

/// 旋转矩形 SDF。`grad` 可为 nullptr（`plan()` 只用距离那一项）。
double rect_sdf(const Rect& r, double px, double py, Vec2* grad = nullptr);

/// 把矩形聚成"一堵墙"。返回墙数；`out` 必须能放 `n` 个 `Wall`。
/// **必须显式排序**：见 .cpp 里的长注释。
int group_walls(const Rect* rects, size_t n, double px, double py,
                const AlgoConfig& cfg, Wall* out);

/// 从可见的矩形反算缺口。
Aperture reconstruct_aperture(const Wall& wall, WallMemory& mem, double r_p,
                              double margin, double field_w,
                              bool has_global_gap, double global_gap);

// ---------------------------------------------------------------------------
// 主入口
// ---------------------------------------------------------------------------

/// 每帧调用一次：世界状态 -> Decision。只读 world，不改；只改 `mem`。
Decision plan(const WorldView& world, GapMemory& mem, const AlgoConfig& cfg,
              double horizon_remaining);

/// Decision -> 归一化速度指令。零向量 = 不动。
Vec2 to_command(const Decision& d);

}  // namespace gap_avoid
