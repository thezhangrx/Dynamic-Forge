/* ===========================================================================
 * gt_decision_cli.cpp —— **绕过视觉**，直接用平台真值跑决策
 *
 * 用户的要求：先把"视觉"和"决策"分开验。这个工具只吃**场地单位**的真值，
 * 不碰像素、不碰相机：
 *
 *     @GT <px> <py> <pr> <n>  [x y half_w half_h rot]*   ← 一行一帧（场地单位）
 *
 * 输出（每帧一行）：
 *     decision tier=<...> cmd=(<vx>,<vy>) gap=? rects=<n>      以及动作号
 *
 * 这样只要平台的真值正确，决策对不对就能一眼看出来；决策对了，剩下的问题
 * 才属于视觉。
 *
 * 编译： g++ -std=c++17 -O2 -I core/cpu/cpp -o /tmp/gt_decide \
 *            core/board/tools/gt_decision_cli.cpp core/cpu/cpp/gap_avoid.cpp
 * 运行： /tmp/gt_decide < gt.txt
 * =========================================================================== */
#include "gap_avoid.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

int main() {
    // 行缓冲：这个工具是**管道里的一环**，不 flush 的话下游 readline 会一直等
    // （踩过：stdout 是管道时是块缓冲，中继读不到任何决策，看起来像死循环）
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    gap_avoid::AlgoConfig cfg;
    cfg.front_back = true;          // 平台 9 个动作里有前后
    gap_avoid::GapMemory mem;
    char line[4096];
    int frame = 0;
    while (std::fgets(line, sizeof(line), stdin)) {
        double px = 0, py = 0, pr = 10;
        int n = 0;
        if (std::sscanf(line, "@GT %lf %lf %lf %d", &px, &py, &pr, &n) != 4) continue;
        std::vector<gap_avoid::Rect> rects;
        const char* p = std::strchr(line, ' ');
        for (int k = 0; k < 4; ++k) {           // 跳过 @GT px py pr n
            if (!p) break;
            p = std::strchr(p + 1, ' ');
        }
        for (int i = 0; i < n && p; ++i) {
            double x = 0, y = 0, hw = 0, hh = 0, rot = 0;
            if (std::sscanf(p, " %lf %lf %lf %lf %lf", &x, &y, &hw, &hh, &rot) != 5) break;
            gap_avoid::Rect r;
            r.ent_id = i + 1;
            r.x = x; r.y = y; r.half_w = hw; r.half_h = hh; r.rotation = rot;
            rects.push_back(r);
            for (int t = 0; t < 5; ++t) {
                p = std::strchr(p + 1, ' ');
                if (!p) break;
            }
        }
        const double player[7] = {px, py, 0.0, 0.0, pr, 200.0, 1.0};
        gap_avoid::WorldView w;
        w.player = player;
        w.rects = rects.data();
        w.n_rects = rects.size();
        w.dt = 1.0 / 25.0;
        w.step_index = frame;
        w.field_w = 640.0;
        w.field_h = 480.0;
        const gap_avoid::Decision d = gap_avoid::plan(w, mem, cfg, 2.0);
        const gap_avoid::Vec2 c = gap_avoid::to_command(d);
        // `aim_world` 是**决策自己选定的瞄准点**（墙上的缺口方向），
        // 显示端必须用它，而不是另算一个"最近的缺口" —— 后者会跟真缺口脱节。
        std::printf("frame %4d tier=%-7s cmd=(%+.2f,%+.2f) gap=%+.1f rects=%zu "
                    "aim=(%.1f,%.1f) band=%.1f\n",
                    frame, d.tier, c.x, c.y, d.gap, rects.size(),
                    d.aim_world.x, d.aim_world.y, d.wall_band);
        ++frame;
    }
    return 0;
}
