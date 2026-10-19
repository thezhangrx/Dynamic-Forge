// 虚拟真值（VirtPlayer + VirtWall）：视觉只负责**初始化**和**发现新墙**，
// 之后位置/速度由积分维持；视觉**缺失或漂移**时不喂真值（用户口径）。
//
// 三条规矩（都来自实测，不是设计洁癖）：
//   1. **角色**：开头几十帧（平台静止、视觉稳定）取稳健均值锁初始位置；之后
//      `p += 单位方向 × 速度/板秒 × dt`，速度取 `boot_cal` 的 156.6 单位/板秒
//      （已含"板子时间基快 9%"的补偿）。视觉只在"有检出 且 残差 < gate"时做
//      慢校正（α=0.25）；连续 5 帧残差超限才重锚（那说明积分真的分叉了）。
//      → 墙经过角色时视觉会**丢帧**，这时**不喂**、只积分，位置不会跟着跳。
//   2. **墙**：队列（默认最多 2 个墙位，画面里最多同时两道）。新墙先进缓冲区，
//      攒够 `stable_n` 帧取中位数把 `y / 缺口两端 / vy` **冻结**；**冻结后不再被
//      视觉刷新**（否则墙走到画面底部时会被一直重测、看起来乱跳）。出场地即释放。
//   3. 视觉在这些情况下**一律不采信**（保持积分）：没检出、落在墙带里被排除、
//      残差超 `gate`（漂移）。
#ifndef VISION_BOARD_VIRT_TRUTH_HPP
#define VISION_BOARD_VIRT_TRUTH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace vision_board {
namespace virt {

inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    const size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(n), v.end());
    return v[n];
}

// ---------------------------------------------------------------------------
// 角色
// ---------------------------------------------------------------------------
struct PlayerState {
    bool locked = false;
    double x = 0.0, y = 0.0;
    std::vector<std::array<double, 2>> init;   // 初始位置的样本
    int n_init_need = 8;                       // 攒够这么多帧才锁
    int n_reinit = 0;                          // 重锚次数（正常应该 0）
    int bad_run = 0;                           // 连续"残差超限"帧数（漂移判定）
    double gate = 60.0;                        // 残差门（场地单位）
    double alpha = 0.25;                       // 慢校正比例
    //: **行为一致性门**：相邻两次被采信的视觉样本之间的位移，必须和"上一条指令
    //: 允许的位移"对得上。两种漂移都靠它挡：
    //:   · 板子发的是 stay（或已停），视觉却报角色动了 > `still_eps`；
    //:   · 板子发的是某方向，视觉位移方向与之相反、或幅度超过 `step_ratio` 倍。
    //: 触发就当"这一帧视觉不可信"，**不更新真值**（只继续积分）。
    double still_eps = 10.0;                   // 静止判定的位移容差（场地单位）
    double step_gate = 45.0;                   // 单步位移与期望之差的上限
    double step_ratio = 1.8;                   // 幅度相对"速度×dt"的倍数上限
    bool have_prev_vis = false;
    double pvx = 0.0, pvy = 0.0, pvt = 0.0;    // 上一次**被采信**的视觉样本
    int n_reject_missing = 0, n_reject_drift = 0;

    void observe_init(double px, double py) {
        if (!locked) init.push_back({px, py});
    }

    //: 攒够样本就锁；返回是否刚锁定。
    bool try_lock() {
        if (locked || static_cast<int>(init.size()) < n_init_need) return false;
        std::vector<double> xs, ys;
        for (const auto& p : init) { xs.push_back(p[0]); ys.push_back(p[1]); }
        x = median(xs);
        y = median(ys);
        // 样本必须一致：抖动大的初始样本说明检到了别的东西（墙端头/框线）
        const double sx = *std::max_element(xs.begin(), xs.end()) -
                          *std::min_element(xs.begin(), xs.end());
        const double sy = *std::max_element(ys.begin(), ys.end()) -
                          *std::min_element(ys.begin(), ys.end());
        if (sx > 20.0 || sy > 20.0) { init.clear(); return false; }
        locked = true;
        return true;
    }

    //: 积分一步（`ux,uy` 是上一帧**实际发出**的单位方向；dt 是板秒）。
    void integrate(double ux, double uy, double dt, double speed_board,
                   double field_w, double field_h, double radius) {
        if (!locked) return;
        if (dt > 0.0 && dt < 0.5) {
            x += ux * speed_board * dt;
            y += uy * speed_board * dt;
        }
        const double m = std::max(1.0, radius + 1.0);
        x = std::min(std::max(x, m), field_w - m);
        y = std::min(std::max(y, m), field_h - m);
    }

    //: 视觉是否可信。三道门，任一不过就**不采信**（只积分，不更新真值）：
    //:   ① 必须有检出（调用方保证：`player_ok == false` 时根本不调用）；
    //:   ② **行为一致性**：这一次视觉位移要和"上一条指令允许的位移"对得上
    //:      —— 挡"板子没动角色却在动"和"动得太厉害"；
    //:   ③ 与积分位置的残差在 `gate` 内（挡漂移/错块）。
    bool vision_ok(double vx, double vy, double t, double ux, double uy,
                   double speed_board, double dt) {
        if (!locked) return false;

        // ② 行为一致性
        if (have_prev_vis) {
            const double sx = vx - pvx, sy = vy - pvy;
            const double moved = std::hypot(sx, sy);
            const bool cmd_stay = (std::fabs(ux) + std::fabs(uy)) < 1e-6;
            if (cmd_stay && moved > still_eps) {
                ++n_reject_drift;                    // 没发指令却动了 → 漂移
                pvx = vx; pvy = vy; pvt = t;         // 只更新参考点，免得永远拒下去
                // ⚠ 拒绝也要计入"连续异常"：否则一旦锁定错了位置，行为门会永久
                //   拒掉正确的观测，`bad_run` 永远不涨 → **永远纠正不回来**。
                //   实测就是这样：板子以为角色在右边界（x=627），真值在左边界（x=10），
                //   于是一直发"向左"，把角色死死压在左边界（78.7% 的帧）。
                if (++bad_run >= 5) { x = vx; y = vy; bad_run = 0; ++n_reinit; }
                return false;
            }
            if (!cmd_stay) {
                const double expect = speed_board * std::max(dt, 1e-3);
                const double along = sx * ux + sy * uy;      // 沿指令方向的分量
                const bool backwards = along < -still_eps;   // 往反方向动
                const bool too_far = moved > step_ratio * expect + step_gate;
                if (backwards || too_far) {
                    ++n_reject_drift;
                    pvx = vx; pvy = vy; pvt = t;
                    if (++bad_run >= 5) { x = vx; y = vy; bad_run = 0; ++n_reinit; }
                    return false;
                }
            }
            pvx = vx; pvy = vy; pvt = t;
        } else {
            pvx = vx; pvy = vy; pvt = t;
            have_prev_vis = true;
        }

        // ③ 与积分位置的残差
        const double d = std::hypot(vx - x, vy - y);
        if (d <= gate) { bad_run = 0; return true; }
        if (++bad_run >= 5) {               // 连续超限 → 判定积分分叉，重锚
            x = vx; y = vy; bad_run = 0; ++n_reinit;
        }
        return false;
    }

    void correct(double vx, double vy) {
        x += alpha * (vx - x);
        y += alpha * (vy - y);
    }
};

// ---------------------------------------------------------------------------
// 墙（队列）
// ---------------------------------------------------------------------------
struct WallSlot {
    double y = 0.0, gap_l = 0.0, gap_r = 0.0;
    double vy = 0.0;                 // 场地单位/板秒（向下为负）
    double t_last = 0.0;
    bool frozen = false;
    int age = 0;
    std::vector<std::array<double, 4>> buf;   // {t, y, gap_l, gap_r}
};

struct WallQueue {
    std::vector<WallSlot> slots;
    int cap = 2;                     // 画面里最多同时两道墙
    int stable_n = 4;                // 攒够几帧冻结
    double similar = 40.0;           // 判定"同一面墙"的 y 距离（场地单位）
    int n_admitted = 0, n_released = 0;

    //: 喂一次视觉观测（`have` = 这一帧视觉是否给出了墙）。冻结的墙**不会被刷新**。
    void update(double t, bool have, double y, double gap_l, double gap_r) {
        if (!have) return;
        for (auto& s : slots) {
            const double ref = s.frozen ? s.y : (s.buf.empty() ? s.y : s.buf.back()[1]);
            if (std::fabs(ref - y) <= similar) {
                if (!s.frozen) {
                    s.buf.push_back({t, y, gap_l, gap_r});
                    // 冻结前必须满足：① 攒够帧；② y 稳定（极差小）；③ **速度像墙**
                    // （落在 vy_min..vy_max）。③ 是用户看到的"墙爬得慢、卡在队列里"的根：
                    // `wall_y_units` 是"选中的那扇门"的 y，门一换就混进另一面墙，
                    // 估出来的速度偏小，冻结后这面墙几乎不动、永远出不了场地。
                    if (static_cast<int>(s.buf.size()) >= stable_n) {
                        const double ymax = s.buf.back()[1], ymin0 = s.buf.front()[1];
                        const double dtb = s.buf.back()[0] - s.buf.front()[0];
                        const double vy_est = (dtb > 0.05) ? (ymax - ymin0) / dtb : 0.0;
                        const bool stable = std::fabs(ymax - ymin0) <= 20.0;
                        const bool plausible = (std::fabs(vy_est) >= vy_min &&
                                                std::fabs(vy_est) <= vy_max);
                        if (stable && plausible) freeze(s);
                        else if (static_cast<int>(s.buf.size()) > 16) freeze(s);  // 兜底
                    }
                }
                return;                 // 命中就不用再找别的墙位
            }
        }
        if (static_cast<int>(slots.size()) >= cap) return;   // 队列满：等旧的离场
        WallSlot s;
        s.y = y; s.gap_l = gap_l; s.gap_r = gap_r; s.t_last = t;
        s.buf.push_back({t, y, gap_l, gap_r});
        slots.push_back(s);
        ++n_admitted;
    }

    //: 墙速合理区间（场地单位/**板秒**）。契约 128 单位/仿真秒 ≈ 156 单位/板秒。
    double vy_min = 60.0, vy_max = 260.0;

    static void freeze(WallSlot& s) {
        std::vector<double> ys, gl, gr;
        for (const auto& b : s.buf) { ys.push_back(b[1]); gl.push_back(b[2]); gr.push_back(b[3]); }
        s.y = median(ys);
        s.gap_l = median(gl);
        s.gap_r = median(gr);
        const double dt = s.buf.back()[0] - s.buf.front()[0];
        s.vy = (dt > 0.05) ? (s.buf.back()[1] - s.buf.front()[1]) / dt : 0.0;
        s.frozen = true;
    }

    //: 冻结的墙按自己的 vy 匀速积分（视觉不再参与）。
    void advance(double t) {
        for (auto& s : slots) {
            if (s.frozen) {
                const double dt = std::max(0.0, std::min(0.5, t - s.t_last));
                s.y += s.vy * dt;
            }
            s.t_last = t;
            ++s.age;
        }
    }

    //: 出场地即释放（上方超界也算，缺口不会再回来）。
    void purge(double field_h) {
        slots.erase(std::remove_if(slots.begin(), slots.end(),
                                   [&](const WallSlot& s) {
                                       return s.y < -8.0 || s.y > field_h + 8.0;
                                   }),
                    slots.end());
    }

    int frozen_count() const {
        int n = 0;
        for (const auto& s : slots) if (s.frozen) ++n;
        return n;
    }
};

}   // namespace virt
}   // namespace vision_board

#endif   // VISION_BOARD_VIRT_TRUTH_HPP
