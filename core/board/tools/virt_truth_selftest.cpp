// `virt_truth.hpp` 的 v2 自检：静态墙冻结、相对观测、角色零校正/安全重锚。
//
//   g++ -std=c++17 -O2 -I core/board -o /tmp/vt_selftest \
//       core/board/tools/virt_truth_selftest.cpp && /tmp/vt_selftest
//
// 这些断言就是"视觉只做对照、虚拟真值说了算"的可执行版本：
//   * 相对观测 (墙−角色) + 虚拟真值角色位姿 → 墙的场地坐标；
//   * 静态世界冻结后**不随时间移动**（跟随视口的表观运动不能进积分）；
//   * 角色位置只由指令积分决定，视觉不校正；
//   * 连续 5 帧残差超门才重锚（安全阀）。
#include <cmath>
#include <cstdio>

#include "virt_truth.hpp"

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

void check_near(double got, double want, double tol, const char* what) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("%s  %s（got %.3f, want %.3f ±%.3f）\n",
                ok ? "PASS" : "FAIL", what, got, want, tol);
    if (!ok) ++g_fail;
}

}  // namespace

int main() {
    using namespace vision_board::virt;

    // ---- 1) 角色：锁定 + 纯积分（视觉不校正）------------------------------
    {
        PlayerState p;
        for (int i = 0; i < 8; ++i) p.observe_init(200.0, 60.0);
        check(p.try_lock(), "角色：8 帧样本后锁定");
        p.integrate(0.0, 1.0, 0.4, 100.0, 640.0, 480.0, 10.0);
        check_near(p.y, 100.0, 1e-6, "角色：向上积分 0.4 s（100 单位/秒）");
        // 视觉给一个只差 10 单位的位置：零校正 → 位置不能变
        const bool ok = p.vision_ok(200.0, 110.0, 1.0, 0.0, 1.0, 100.0, 0.1);
        check(ok, "角色：残差在门内，视觉判为可信（仅统计）");
        check_near(p.x, 200.0, 1e-6, "角色：视觉可信时位置仍由积分决定（x）");
        check_near(p.y, 100.0, 1e-6, "角色：视觉可信时位置仍由积分决定（y）");
    }

    // ---- 2) 角色：连续 5 帧残差超门 → 安全重锚 ----------------------------
    {
        PlayerState p;
        for (int i = 0; i < 8; ++i) p.observe_init(100.0, 100.0);
        p.try_lock();
        for (int i = 0; i < 5; ++i) {
            (void)p.vision_ok(300.0, 100.0, 1.0 + 0.1 * i, 0.0, 0.0, 100.0, 0.1);
        }
        check_near(p.x, 300.0, 1e-6, "角色：连续 5 帧残差 >60 → 重锚到视觉");
        check(p.n_reinit == 1, "角色：重锚计数 = 1");
    }

    // ---- 3) 墙：相对观测 → 虚拟真值场地坐标 -------------------------------
    {
        WallQueue q;
        check(q.static_world, "墙：默认 static_world（平台 v2）");
        check(!q.require_motion, "墙：静止世界不做 vy 门");
        check(q.cap >= 4, "墙：容量至少 4（全幅可见）");

        // 视觉角色 (200,300)，墙 y=350 → 相对 +50；虚拟真值角色 (100,100) → 墙 y=150
        // 缺口 [260,330] → 相对 [60,130] → 虚拟真值 [160,230]
        for (int i = 0; i < 4; ++i) {
            q.update_relative(0.01 * i, true, 200.0, 300.0, 100.0, 100.0,
                              350.0, 260.0, 330.0);
        }
        check(q.slots.size() == 1, "墙：同一条观测只占一个槽");
        check(q.slots[0].frozen, "墙：4 帧稳定后冻结");
        check_near(q.slots[0].y, 150.0, 1e-6, "墙：y 用相对量放回虚拟真值");
        check_near(q.slots[0].gap_l, 160.0, 1e-6, "墙：缺口左端用相对量");
        check_near(q.slots[0].gap_r, 230.0, 1e-6, "墙：缺口右端用相对量");
        q.advance(10.0);
        check_near(q.slots[0].y, 150.0, 1e-6, "静态世界：冻结后 advance 不改变 y");
    }

    // ---- 4) 墙：旧移动墙模式仍可回退（integrate）--------------------------
    {
        WallQueue q;
        q.static_world = false;
        q.require_motion = true;
        // 每 0.05 s 向上 5 单位（=100 单位/秒，落在 60..260 门内）
        for (int i = 0; i < 4; ++i) {
            q.update(0.05 * i, true, 200.0 + 5.0 * i, 100.0, 200.0);
        }
        check(q.slots[0].frozen, "移动墙：4 帧稳定 + 速度合理 → 冻结");
        const double y0 = q.slots[0].y;
        q.advance(0.65);                 // t_last=0.15 → dt=0.5
        check_near(q.slots[0].y, y0 + 50.0, 1e-6, "移动墙：advance 按 vy 积分 0.5 s");
    }

    // ---- 5) 墙：四道墙各自独立，不被相似门吞掉 ---------------------------
    {
        WallQueue q;
        const double ys[4] = {80.0, 200.0, 320.0, 440.0};
        for (int k = 0; k < 4; ++k) {
            for (int i = 0; i < 4; ++i) {
                q.update(0.01 * i, true, ys[k], 100.0, 200.0);
            }
        }
        check(q.slots.size() == 4, "墙：4 道墙占 4 个槽（v2 全幅）");
        check(q.frozen_count() == 4, "墙：4 道墙全部冻结");
    }

    std::printf("\n%s（%d 项失败）\n", g_fail ? "FAILED" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
