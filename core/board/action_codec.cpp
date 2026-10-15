// ===========================================================================
// action_codec.cpp —— 离散动作量化 + 串口行编码（见 action_codec.hpp 的说明）
//
// 与 core/rp2040_keyboard/action_sender.py 是**同一算法**的两份实现，
// 差分测试逐点对账。
// ===========================================================================
#include "action_codec.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace vision_board {

//: 1/√2。**必须照 Python 的算法算，不能抄它的十进制值。**
//:
//: ⚠ 踩过的坑：原来写的是字面量 `0.70710678118654752440`。1/√2 的真值
//:   0.707106781186547524400844… 恰好落在两个相邻 double 的中间偏上，于是
//:   round-to-nearest 得到 `0x1.6a09e667f3bcdp-1`；而 Python 侧
//:   `float(1.0 / np.sqrt(2.0))` 是"先算 sqrt 再取倒数"，得到的是
//:   `0x1.6a09e667f3bccp-1` —— **差 1 个 ulp**。
//:
//:   为什么这 1 个 ulp 要命：45° 边界上相邻两个方向的点积**本来就相等**
//:   （选向就是比点积最大），这 1 个 ulp 足以让两边破到不同的方向。
//:   实测 527 组速度里有 2 组分家（C++ 选对角线、Python 选轴向），
//:   角色会朝偏 45° 的方向走 —— 而且不会报错。
//:   差分测试 `test_action_codec_matches_python.py` 就是为此存在的。
//:
//:   所以这里按 Python 的**算法**写：先 sqrt 再取倒数。
const double kInvSqrt2 = 1.0 / std::sqrt(2.0);

const char* const kDiscreteActionNames[kNumDiscreteActions] = {
    "stay", "left", "right", "up", "down",
    "up_left", "up_right", "down_left", "down_right",
};

const double kDiscreteDirections[kNumDiscreteActions][2] = {
    {  0.0,        0.0       },   // 0 stay
    { -1.0,        0.0       },   // 1 left
    {  1.0,        0.0       },   // 2 right
    {  0.0,        1.0       },   // 3 up
    {  0.0,       -1.0       },   // 4 down
    { -kInvSqrt2,  kInvSqrt2 },   // 5 up_left
    {  kInvSqrt2,  kInvSqrt2 },   // 6 up_right
    { -kInvSqrt2, -kInvSqrt2 },   // 7 down_left
    {  kInvSqrt2, -kInvSqrt2 },   // 8 down_right
};

int action_from_velocity(double wx, double wy, double deadzone) {
    // 与 Python 完全一致：先判 |v| < deadzone → stay；否则在 1..8 里取点积最大。
    if (std::sqrt(wx * wx + wy * wy) < deadzone) return 0;
    int best_i = 0;
    double best_d = -std::numeric_limits<double>::infinity();
    for (int i = 1; i < kNumDiscreteActions; ++i) {
        const double d = kDiscreteDirections[i][0] * wx
                       + kDiscreteDirections[i][1] * wy;
        if (d > best_d) { best_i = i; best_d = d; }
    }
    return best_i;
}

const char* action_name(int a) {
    static thread_local char buf[16];
    if (a >= 0 && a < kNumDiscreteActions) return kDiscreteActionNames[a];
    std::snprintf(buf, sizeof(buf), "?%d", a);
    return buf;
}

int format_action_line(int a, char* buf, int cap) {
    const int n = std::snprintf(buf, static_cast<size_t>(cap), "%s%d\n",
                                kActionPrefix, a);
    if (n < 0 || n >= cap) return -1;
    return n;
}

}   // namespace vision_board
