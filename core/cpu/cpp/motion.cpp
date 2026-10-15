// ===========================================================================
// motion.cpp —— `core/vision/motion.py` 的 C++ 移植，逐句对应 `Tracker.update()`
//
// 照抄语义的三处（每一处改"朴素写法"都会与 Python 分家）：
//
//   1. **预测后再算残差**。α-β 的残差是 `z − x̂`（预测落点），不是 `z − x_old`。
//      用后者会让速度估计偏向"位置一阶差分"，在匀速目标上收敛到错的值。
//   2. **贪心最近邻，按轨迹顺序**，门限内取**最近**（`best_d` 逐步收缩）。
//      并列时取下标小的那个 —— 顺序变了配的对就变了。
//   3. **外推的轨迹不更新 `stamp`**、`age` 照加；`misses > max_misses` 才删。
// ===========================================================================
#include "motion.hpp"

#include <algorithm>

namespace motion {

bool Tracker::update(const Point* dets, int n_dets, double stamp) {
    // ---- dt：必须在改动任何状态**之前**判定 ----
    double dt = 0.0;
    if (have_last_stamp_) {
        dt = stamp - last_stamp_;
        // `!(dt >= 0)` 同时挡住负数与 NaN；`!isfinite` 再挡 ±inf。
        // Python 那边是抛 ValueError，见 hpp 顶部的说明。
        if (!(dt >= 0.0) || !std::isfinite(dt)) return false;
    }
    // 第一帧、或间隔太大：不预测，直接落位、速度置零（避免跨大间隔乱推）
    const bool fresh = !have_last_stamp_ || dt > cfg_.max_dt;
    last_stamp_ = stamp;
    have_last_stamp_ = true;

    const int n_trk = static_cast<int>(tracks_.size());

    // ---- 1. 预测：把每条轨迹推到当前时刻 ----
    std::vector<Point> pred(static_cast<size_t>(n_trk));
    for (int i = 0; i < n_trk; ++i) {
        const Track& t = tracks_[static_cast<size_t>(i)];
        pred[static_cast<size_t>(i)] =
            (fresh || dt <= 0.0) ? Point{t.x, t.y} : t.predict(dt);
    }

    // ---- 2. 关联：贪心最近邻 + 自适应门限 ----
    std::vector<char> used(static_cast<size_t>(std::max(0, n_dets)), 0);
    std::vector<int> match(static_cast<size_t>(n_trk), -1);
    for (int ti = 0; ti < n_trk; ++ti) {
        const Track& t = tracks_[static_cast<size_t>(ti)];
        // 跑得快的目标给更大的搜索窗（一次乘加，仍然可流水）
        const double gate = cfg_.gate +
            cfg_.gate_speed_frac * t.speed() * (dt > 0.0 ? dt : 0.0);
        int best_j = -1;
        double best_d = gate;
        for (int j = 0; j < n_dets; ++j) {
            if (used[static_cast<size_t>(j)]) continue;
            const Point& z = dets[j];
            const Point& p = pred[static_cast<size_t>(ti)];
            const double d = std::hypot(z.x - p.x, z.y - p.y);
            if (d <= best_d) {          // `<=`：恰好落在门限上也算命中
                best_j = j;
                best_d = d;
            }
        }
        if (best_j >= 0) {
            used[static_cast<size_t>(best_j)] = 1;
            match[static_cast<size_t>(ti)] = best_j;
        }
    }

    // ---- 3. 滤波：α-β 更新 ----
    std::vector<Track> survivors;
    std::vector<int> assignment;
    survivors.reserve(static_cast<size_t>(n_trk + std::max(0, n_dets)));
    assignment.reserve(survivors.capacity());

    for (int ti = 0; ti < n_trk; ++ti) {
        Track t = tracks_[static_cast<size_t>(ti)];      // 拷贝后改，原数组只读
        if (match[static_cast<size_t>(ti)] < 0) {
            t.misses += 1;
            if (t.misses <= cfg_.max_misses) {
                if (fresh || dt <= 0.0) {
                    t.vx = t.vy = 0.0;                   // 间隔太大，速度不可信
                } else {
                    t.x = pred[static_cast<size_t>(ti)].x;   // 惯性外推
                    t.y = pred[static_cast<size_t>(ti)].y;
                }
                t.age += 1;
                survivors.push_back(t);
                assignment.push_back(-1);                // 本轮没有对应的检测
            }
            continue;
        }

        const int j = match[static_cast<size_t>(ti)];
        const Point& z = dets[j];
        if (fresh || dt <= 0.0) {
            // 第一帧或大间隔：直接落位，速度保持 0
            t.x = z.x;
            t.y = z.y;
            t.vx = t.vy = 0.0;
        } else {
            const Point& p = pred[static_cast<size_t>(ti)];
            const double rx = z.x - p.x;
            const double ry = z.y - p.y;
            if (std::hypot(rx, ry) > cfg_.max_residual &&
                t.suspect + 1 < cfg_.confirm_frames) {
                // 可疑帧：**位置照跟，速度这一帧不认**（保留上一次的估计，
                // 而不是清零 —— 清零等于"当它静止"，那正是要修的老行为）。
                ++t.suspect;
                t.x = z.x;
                t.y = z.y;
                ++n_relocalized_;
            } else {
                // 正常帧，或者**连续可疑**（那说明是真运动）：照常 α-β
                t.suspect = 0;
                t.x = p.x + cfg_.alpha * rx;
                t.y = p.y + cfg_.alpha * ry;
                t.vx += (cfg_.beta / dt) * rx;
                t.vy += (cfg_.beta / dt) * ry;
            }
        }
        t.stamp = stamp;
        t.hits += 1;
        t.misses = 0;
        t.age += 1;
        survivors.push_back(t);
        assignment.push_back(j);
    }

    // ---- 4. 新目标：没有被任何轨迹认领的检测 ----
    for (int j = 0; j < n_dets; ++j) {
        if (used[static_cast<size_t>(j)]) continue;
        Track nt;
        nt.id = next_id_++;
        nt.x = dets[j].x;
        nt.y = dets[j].y;
        nt.stamp = stamp;
        survivors.push_back(nt);
        assignment.push_back(j);
    }

    tracks_.swap(survivors);
    assignment_.swap(assignment);
    return true;
}

int attach_velocities(const std::vector<Track>& tracks,
                      const std::vector<int>& assignment,
                      int n_dets, double* out_vx, double* out_vy) {
    if (n_dets <= 0 || out_vx == nullptr || out_vy == nullptr) return 0;
    for (int j = 0; j < n_dets; ++j) {
        out_vx[j] = 0.0;                 // 没被认领的检测：速度未知 = 0
        out_vy[j] = 0.0;
    }
    const size_t n = std::min(tracks.size(), assignment.size());
    int attached = 0;
    for (size_t i = 0; i < n; ++i) {
        const int j = assignment[i];
        if (j < 0 || j >= n_dets) continue;
        out_vx[j] = tracks[i].vx;
        out_vy[j] = tracks[i].vy;
        ++attached;
    }
    return attached;
}

bool track_one(Tracker& tr, const Point& p, double stamp,
               double* out_vx, double* out_vy) {
    if (out_vx != nullptr) *out_vx = 0.0;
    if (out_vy != nullptr) *out_vy = 0.0;
    if (!tr.update(&p, 1, stamp)) return false;
    const std::vector<Track>& ts = tr.tracks();
    if (ts.empty()) return false;
    if (out_vx != nullptr) *out_vx = ts[0].vx;
    if (out_vy != nullptr) *out_vy = ts[0].vy;
    return true;
}

}   // namespace motion
