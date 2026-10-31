// ===========================================================================
// action_codec.hpp —— 把决策的**连续速度指令**量化成**离散动作号**，再编成串口行
//
// 这一层是 PS 与 RP2040 键盘桥之间的**协议边界**：
//
//     几何帧 → 决策（gap_avoid）→ 期望速度 (wx, wy)
//        → action_from_velocity() → 动作号 0..8
//        → format_action_line()   → "@MOVE <n>\n" → 板子控制台 TX
//        → RP2040（GP1 接在控制台 TX 上）→ USB HID 方向键 → 平台里的角色
//
// ⚠ **动作号的顺序是公开协议，只能追加、不能重排**。
//   真值在 `core/bullet_sim/core/actions.py` 的 `DISCRETE_ACTION_NAMES`
//   （源码注释：*Order is part of the public protocol — never reorder, only append*）。
//   本文件是它在 C++ 侧的一份镜像，**不是**第二份真值 ——
//   `core/board/tests/test_action_codec_matches_python.py` 会：
//     ① 解析本文件的表，和 Python 的 `DISCRETE_ACTION_NAMES` /
//        `DISCRETE_DIRECTIONS` 逐项比对；
//     ② 用**同一批速度**分别喂 C++ 与 Python 的 `action_from_velocity`，
//        断言动作号逐点相同（差分对账，和 gap_avoid 的 tier 对账同一套路）。
// ===========================================================================
#ifndef VISION_BOARD_ACTION_CODEC_HPP
#define VISION_BOARD_ACTION_CODEC_HPP

namespace vision_board {

//: 9 个离散动作（stay + 8 个方向）。
inline constexpr int kNumDiscreteActions = 9;

//: 动作名，顺序 = 公开协议。
extern const char* const kDiscreteActionNames[kNumDiscreteActions];

//: 9 个单位方向（世界系，y 向上）。第 0 项是 stay，不参与选向。
extern const double kDiscreteDirections[kNumDiscreteActions][2];

//: |v| 小于它就当 stay。与 `action_sender.py` 的 DEFAULT_DEADZONE 一致。
inline constexpr double kDefaultDeadzone = 1e-3;

//: 串口行前缀。这条线是**共享的控制台**（RP2040 会看到全部内核日志），
//: 所以必须带一个正常日志里不会出现的前缀才能挑出指令。
inline constexpr const char* kActionPrefix = "@MOVE ";

//: 把世界系期望速度量化成最近的方向（点积最大）。
//: 与 `core/rp2040_keyboard/action_sender.py` 的 `action_from_velocity` 同一算法；
//: 有差分测试保证两边逐点一致。
int action_from_velocity(double wx, double wy,
                         double deadzone = kDefaultDeadzone);

//: 动作名（越界返回 "?<n>" 的静态串）。
const char* action_name(int a);

//: 编成 "@MOVE <n>\n"。返回写入的字节数（不含结尾 '\0'），容量不够返回 -1。
//: `buf` 至少要 16 字节。
int format_action_line(int a, char* buf, int cap);

}   // namespace vision_board

#endif   // VISION_BOARD_ACTION_CODEC_HPP
