// ===========================================================================
// motion_cli.cpp —— **只为交叉验证存在**：吃文本序列、吐轨迹与贴好的速度。
// 不是上板程序（上板入口是 `core/board/main.cpp`）。
//
// 输入（每行一帧，`#` 开头是注释）：
//
//     FRAME <stamp> <n_dets> <x0> <y0> <x1> <y1> ...
//
// 输出（每帧）：
//
//     TRK <n_tracks>
//     T <id> <x> <y> <vx> <vy> <stamp> <hits> <misses> <age> <assign>
//     ...                                （n_tracks 行，顺序即 survivors 顺序）
//     ATT <n_dets> <vx0> <vy0> <vx1> <vy1> ...   （贴回检测的速度）
//     ERR stamp                          （stamp 不单调/非有限；Python 侧抛异常）
//
// 浮点一律 `%.17g`：验证脚本要按相对误差 1e-9 对账，位数不够会看不出问题。
// ===========================================================================
#include "motion.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

int main() {
    motion::Tracker tr;
    std::vector<motion::Point> dets;
    char tag[32];
    while (std::scanf("%31s", tag) == 1) {
        if (tag[0] == '#') {                 // 注释行：吃掉整行
            int c;
            while ((c = std::getchar()) != '\n' && c != EOF) { }
            continue;
        }
        if (std::string(tag) != "FRAME") {
            std::fprintf(stderr, "未知指令：%s\n", tag);
            return 2;
        }
        double stamp = 0.0;
        int n = 0;
        if (std::scanf("%lf %d", &stamp, &n) != 2 || n < 0) {
            std::fprintf(stderr, "FRAME 行格式错\n");
            return 2;
        }
        dets.assign(static_cast<size_t>(n), motion::Point{});
        for (int i = 0; i < n; ++i) {
            if (std::scanf("%lf %lf", &dets[static_cast<size_t>(i)].x,
                           &dets[static_cast<size_t>(i)].y) != 2) {
                std::fprintf(stderr, "检测坐标读不全\n");
                return 2;
            }
        }

        if (!tr.update(dets.data(), n, stamp)) {
            std::printf("ERR %.17g\n", stamp);
            continue;
        }
        const std::vector<motion::Track>& ts = tr.tracks();
        const std::vector<int>& asg = tr.assignment();
        std::printf("TRK %zu\n", ts.size());
        for (size_t i = 0; i < ts.size(); ++i) {
            const motion::Track& t = ts[i];
            std::printf("T %d %.17g %.17g %.17g %.17g %.17g %d %d %d %d\n",
                        t.id, t.x, t.y, t.vx, t.vy, t.stamp,
                        t.hits, t.misses, t.age,
                        i < asg.size() ? asg[i] : -1);
        }
        std::vector<double> vx(static_cast<size_t>(n)), vy(static_cast<size_t>(n));
        motion::attach_velocities(ts, asg, n, vx.data(), vy.data());
        std::printf("ATT %d", n);
        for (int j = 0; j < n; ++j) {
            std::printf(" %.17g %.17g", vx[static_cast<size_t>(j)],
                        vy[static_cast<size_t>(j)]);
        }
        std::printf("\n");
    }
    return 0;
}
