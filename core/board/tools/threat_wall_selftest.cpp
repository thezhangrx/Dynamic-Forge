// P3 的单元测试：威胁墙选择。纯逻辑，不碰相机/图像，秒级可跑。
//
// 为什么必须脱离图像测：选墙的错法是"在多墙场景瞄错墙"，而在实拍里这个错**看不出**——
// 画面、日志都正常，只是平台朝错误的缺口开过去。只有把"墙的场地位置 + 角色位置"
// 直接喂进去，才能对每一种几何排布断言"应该选哪一道"。
//
// 用例按 P3 的退出条件覆盖五类：单墙 / 双墙 / 上下顺序互换 / 已越过 / 多缺口+迟滞。
//
// 编译运行：
//   g++ -std=c++17 -O2 -I core/board -o /tmp/tw core/board/tools/threat_wall_selftest.cpp && /tmp/tw
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "wall_gap.hpp"

using vision_board::wall_gap::select_threat_wall;
using vision_board::wall_gap::ThreatParams;
using vision_board::wall_gap::WallInField;

namespace {

int g_fail = 0;
int g_pass = 0;

void check(bool ok, const std::string& name, const std::string& detail = "") {
    if (ok) {
        ++g_pass;
        std::printf("  ✓ %s\n", name.c_str());
    } else {
        ++g_fail;
        std::printf("  ✗ %s   %s\n", name.c_str(), detail.c_str());
    }
}

WallInField W(int id, double y, double gap_cx, bool has_gap = true, double half = 4.0) {
    WallInField w;
    w.id = id;
    w.y_center = y;
    w.half_h = half;
    w.gap_cx = gap_cx;
    w.gap_w = 70.0;
    w.has_gap = has_gap;
    return w;
}

//: 场地 y 朝前（+y = 角色前进方向）；角色从 y=0 出发。
void test_single_wall() {
    std::printf("[1] 单墙：只有一道墙在前方\n");
    std::vector<WallInField> walls = {W(1, 200.0, 320.0)};
    const int s = select_threat_wall(walls, 60.0);
    check(s == 0, "角色 y=60，墙 y=200 → 选中该墙",
          "got " + std::to_string(s));
    // 没有缺口的墙不是"可通过的威胁"
    std::vector<WallInField> solid = {W(1, 200.0, 320.0, /*has_gap=*/false)};
    check(select_threat_wall(solid, 60.0) == -1, "无缺口的墙不算威胁", "");
    // 空集合
    check(select_threat_wall({}, 60.0) == -1, "没有墙 → -1（不是乱选一道）", "");
}

void test_two_walls_nearest() {
    std::printf("[2] 双墙：取前方最近的那道\n");
    std::vector<WallInField> walls = {W(1, 300.0, 100.0), W(2, 180.0, 500.0)};
    const int s = select_threat_wall(walls, 60.0);
    check(s == 1, "墙在 y=300/180，角色 y=60 → 选 y=180 的那道",
          "got " + std::to_string(s));
}

void test_order_independent() {
    std::printf("[3] 上下顺序互换：结果只由相对位置决定，与存储顺序无关\n");
    std::vector<WallInField> a = {W(1, 180.0, 100.0), W(2, 300.0, 500.0)};
    std::vector<WallInField> b = {W(2, 300.0, 500.0), W(1, 180.0, 100.0)};
    const int sa = select_threat_wall(a, 60.0);
    const int sb = select_threat_wall(b, 60.0);
    const int ida = sa >= 0 ? a[static_cast<size_t>(sa)].id : -1;
    const int idb = sb >= 0 ? b[static_cast<size_t>(sb)].id : -1;
    check(ida == 1 && idb == 1, "两种顺序都选中 id=1（y=180）",
          "got " + std::to_string(ida) + "/" + std::to_string(idb));
}

void test_passed_wall() {
    std::printf("[4] 已越过：身后的墙不再是威胁\n");
    std::vector<WallInField> walls = {W(1, 40.0, 100.0), W(2, 220.0, 500.0)};
    const int s = select_threat_wall(walls, 60.0);
    check(s == 1, "角色 y=60，墙 y=40 已在身后 → 选 y=220",
          "got " + std::to_string(s));
    std::vector<WallInField> only_behind = {W(1, 40.0, 100.0)};
    check(select_threat_wall(only_behind, 60.0) == -1, "只剩身后的墙 → -1",
          "got " + std::to_string(select_threat_wall(only_behind, 60.0)));
    // 角色正好在墙上（距离 0）也算"不再前方"
    std::vector<WallInField> on_wall = {W(1, 60.0, 100.0)};
    check(select_threat_wall(on_wall, 60.0) == -1, "角色与墙中线重合 → -1", "");
}

void test_hysteresis() {
    std::printf("[5] 多缺口 + 迟滞：两墙接近时不逐帧翻目标\n");
    // 锁定 y=200（id=1）；另一道 y=196（id=2）比它更近 4 单位
    std::vector<WallInField> walls = {W(1, 200.0, 100.0), W(2, 196.0, 500.0)};
    ThreatParams p;
    p.switch_tol_units = 15.0;
    const int kept = select_threat_wall(walls, 60.0, p, /*locked_id=*/1);
    check(kept == 0, "新墙只近 4 单位（< 容差 15）→ 继续锁 id=1",
          "got " + std::to_string(kept));
    // 新目标明显更近（近 40 单位）→ 应当切换
    std::vector<WallInField> walls2 = {W(1, 200.0, 100.0), W(2, 160.0, 500.0)};
    const int sw = select_threat_wall(walls2, 60.0, p, /*locked_id=*/1);
    check(sw == 1, "新墙近 40 单位（> 容差）→ 切到 id=2",
          "got " + std::to_string(sw));
    // 锁定的墙被越过 → 必须换目标（不能因为"锁定"就继续瞄身后）
    std::vector<WallInField> walls3 = {W(1, 40.0, 100.0), W(2, 220.0, 500.0)};
    const int after = select_threat_wall(walls3, 60.0, p, /*locked_id=*/1);
    check(after == 1, "锁定的墙已被越过 → 改选前方 id=2",
          "got " + std::to_string(after));
    // 锁定 id 不在集合里 → 退回普通选择
    std::vector<WallInField> walls4 = {W(7, 180.0, 300.0)};
    check(select_threat_wall(walls4, 60.0, p, /*locked_id=*/1) == 0,
          "锁定 id 不存在 → 退回最近前方墙", "");
}

}   // namespace

int main() {
    std::printf("威胁墙选择 —— 单元测试（P3）\n");
    test_single_wall();
    test_two_walls_nearest();
    test_order_independent();
    test_passed_wall();
    test_hysteresis();
    std::printf("---- 通过 %d，失败 %d ----\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
