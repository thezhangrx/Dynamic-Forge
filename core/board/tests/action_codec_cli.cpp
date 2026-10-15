// ===========================================================================
// action_codec_cli.cpp —— 给差分测试用的极小 CLI（也方便手工试）
//
// 从 stdin 逐行读 `wx wy`，对每行打印动作号。`core/board/tests/
// test_action_codec_matches_python.py` 正是用它把**同一批速度**同时喂给
// C++ 与 Python 两份实现，断言结果逐点相同。
//
// 单独一个 CLI 而不是让测试链库：与 `geometry_frame_dump.cpp` 同一套路 ——
// Python 侧只要能起一个进程就能对账，不用碰构建系统。
// ===========================================================================
#include "action_codec.hpp"

#include <cstdio>

int main() {
    double wx = 0.0, wy = 0.0;
    while (std::scanf("%lf %lf", &wx, &wy) == 2) {
        std::printf("%d\n", vision_board::action_from_velocity(wx, wy));
    }
    return 0;
}
