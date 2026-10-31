// ===========================================================================
// test_fiducial_zone.cpp —— 四角标定标记 footprint 判据的本机自测
//
// 为什么单独测：板子可能不在手边（掉电/在别人那里），但"哪些位置会被当成标记、
// 哪些不会"这条判据必须**随时能验**。判据错了的后果很具体：真角色在角落附近时被
// 当成标记（于是只发 stay，动作发不出去），或者标记没被排除（角色被推到边上）。
//
// 约定与仓库里其它 C++ 自测一致：跑完打印 "比对项 N，不符 0"，由 Python 侧包装成
// pytest 用例；"0 项" 也要报错，防止"一个检查都没跑"也报通过。
// ===========================================================================
#include "fiducial_zone.hpp"

#include <cstdio>
#include <cmath>

static int g_total = 0;
static int g_bad = 0;

static void expect(bool cond, const char* what) {
    ++g_total;
    if (!cond) {
        ++g_bad;
        std::printf("  ✗ %s\n", what);
    }
}

int main() {
    // 平台契约默认：640x480 场地、标记边长 48、离边 8、额外留白 12
    fiducial_zone::Zone z;

    // 标记中心：应判为标记
    expect(z.contains(32.0, 32.0), "左下角标记中心应命中");
    expect(z.contains(608.0, 32.0), "右下角标记中心应命中");
    expect(z.contains(32.0, 448.0), "左上角标记中心应命中");
    expect(z.contains(608.0, 448.0), "右上角标记中心应命中");

    // 标记外沿（半边长 = 48/2 + 12 = 36）：边界内命中、边界外不命中
    expect(z.contains(32.0 + 36.0, 32.0), "标记 footprint 边界（内侧）应命中");
    expect(!z.contains(32.0 + 36.5, 32.0), "标记 footprint 之外 0.5 单位不应命中");
    expect(z.contains(32.0, 32.0 + 36.0), "标记 footprint 边界（y 方向内侧）应命中");
    expect(!z.contains(32.0, 32.0 + 36.5), "y 方向边界外不应命中");

    // 场地中央：绝不能命中（否则角色永远"没检出"）
    expect(!z.contains(320.0, 240.0), "场地正中不应命中");
    expect(!z.contains(320.0, 32.0), "下边中点不应命中");
    expect(!z.contains(32.0, 240.0), "左边中点不应命中");

    // 只靠近一个角、另一个坐标近中心：不应命中
    expect(!z.contains(320.0, 36.0), "x 在中间时哪怕 y 贴边也不应命中");

    // 贴边但远离角（场地边中段）：不应命中 —— 角色在边上不该被当成标记
    expect(!z.contains(160.0, 20.0), "下边中段贴边不应命中");

    // 角与角之间距离足够远：一个角不命中另一个角
    expect(!z.contains(608.0, 448.0) == false, "右上角标记中心必须命中（反向写法）");

    // 场地尺寸/留白可配置：换成 320x240 的小场地，坐标整体缩放
    fiducial_zone::Zone small;
    small.field_w = 320.0;
    small.field_h = 240.0;
    expect(small.contains(16.0, 16.0), "小场地左下角标记中心应命中");
    expect(small.contains(304.0, 224.0), "小场地右上角标记中心应命中");
    expect(!small.contains(160.0, 120.0), "小场地正中不应命中");

    // 留白加大 → footprint 变大（单调性）
    fiducial_zone::Zone wider = z;
    wider.slack = 40.0;
    expect(wider.half() > z.half(), "留白加大后 footprint 必须更大");
    // 默认 half = 24 + 12 = 36；留白 40 时 half = 64。取 50 单位：前者不命中、后者命中。
    expect(!z.contains(32.0 + 50.0, 32.0), "默认留白下 50 单位外不命中（half=36）");
    expect(wider.contains(32.0 + 50.0, 32.0), "留白 40 时 50 单位处应命中（half=64）");

    // 中心内缩量 = margin + side/2（便于和 Python 侧契约对齐）
    expect(std::fabs(z.centre_inset() - 32.0) < 1e-9, "中心内缩应为 margin + side/2");

    std::printf("比对项 %d，不符 %d\n", g_total, g_bad);
    return g_bad == 0 ? 0 : 1;
}
