// ===========================================================================
// test_segment_merge.cpp —— 矩形层"碎片合并"的本机自测
//
// 最关键的一条是**反面**用例：真缺口（实测 ~58 px）绝不能被并掉 —— 对避障来说
// "把通道封死"比"墙断裂"危险得多。其余是几何契约（取较长段的厚度/朝向/速度、
// 不同行带不并、重叠必并、输入乱序也按左边界扫）。
// 约定与仓库其它 C++ 自测一致：打印 "比对项 N，不符 0"，由 Python 侧包成 pytest。
// ===========================================================================
#include "segment_merge.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

static int g_total = 0, g_bad = 0;
static void expect(bool c, const char* what) {
    ++g_total;
    if (!c) { ++g_bad; std::printf("  ✗ %s\n", what); }
}

static gap_avoid::Rect R(double x, double half_w, double y = 100.0,
                         double half_h = 8.0, double vx = 0.0) {
    gap_avoid::Rect r;
    r.x = x; r.y = y; r.half_w = half_w; r.half_h = half_h;
    r.vx = vx; r.vy = 0.0; r.rotation = 0.0; r.angular_velocity = 0.0; r.ent_id = 0;
    return r;
}

int main() {
    segment_merge::Params p;          // 默认 max_gap=6, max_dy=4
    int n = 0;

    // ---- 1) 两段共线小缝：并成一段（横向取并集）----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20), R(144, 20)};   // 缝 4 px
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "共线 + 4px 缝应当并成一段");
        expect(std::fabs(out[0].x - 122.0) < 1e-9, "合并后中心 = 并集中点");
        expect(std::fabs(out[0].half_w - 42.0) < 1e-9, "合并后 half_w = 并集一半");
        expect(n == 1, "合并计数应为 1");
    }

    // ---- 2) 真缺口（58 px）：绝不许并 ----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20), R(198, 20)};   // 缝 58 px
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 2, "真缺口 58px 必须保留两段（不许封死通道）");
        expect(n == 0, "不该发生合并");
    }

    // ---- 3) 恰好等于 max_gap：并（<= 语义）----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20), R(146, 20)};   // 缝 6 px
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "缝恰好等于上限应当并");
    }
    // ---- 4) 超过 max_gap 一点：不并 ----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20), R(146.5, 20)}; // 缝 6.5 px
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 2, "缝超过上限不应并");
    }

    // ---- 5) 不同行带（Δy 大）：不并 ----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20, 100.0), R(110, 20, 200.0)};
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 2, "不同行带的两堵墙不许并（Δy > max_dy）");
    }

    // ---- 6) 三段串起来：并成一段 ----
    {
        std::vector<gap_avoid::Rect> in{R(100, 10), R(122, 10), R(145, 10)};
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "三段小缝应并成一段");
        expect(n == 2, "三段应发生 2 次合并");
    }

    // ---- 7) 几何/速度：取**较长**那一段 ----
    {
        gap_avoid::Rect shorty = R(100, 8, 100.0, 3.0, -50.0);
        gap_avoid::Rect longy  = R(118, 40, 104.0, 12.0, 99.0);
        std::vector<gap_avoid::Rect> in{shorty, longy};
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "应当并");
        expect(std::fabs(out[0].half_h - 12.0) < 1e-9, "厚度取较长段");
        expect(std::fabs(out[0].y - 104.0) < 1e-9, "行带 y 取较长段");
        expect(std::fabs(out[0].vx - 99.0) < 1e-9, "速度取较长段（碎段速度是噪声）");
    }

    // ---- 8) 重叠：必并 ----
    {
        std::vector<gap_avoid::Rect> in{R(100, 20), R(110, 20)};
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "重叠的两段应当并");
    }

    // ---- 9) 空 / 单个：原样返回，不崩 ----
    {
        std::vector<gap_avoid::Rect> empty;
        auto o1 = segment_merge::merge_collinear(empty, p, &n);
        expect(o1.empty() && n == 0, "空输入原样返回");
        std::vector<gap_avoid::Rect> one{R(100, 20)};
        auto o2 = segment_merge::merge_collinear(one, p, &n);
        expect(o2.size() == 1 && std::fabs(o2[0].x - 100.0) < 1e-9, "单段原样返回");
    }

    // ---- 10) 乱序输入：按左边界扫，仍能并 ----
    {
        std::vector<gap_avoid::Rect> in{R(144, 20), R(100, 20)};
        auto out = segment_merge::merge_collinear(in, p, &n);
        expect(out.size() == 1, "乱序输入也应按左边界并成一段");
        expect(std::fabs((out[0].x - out[0].half_w) - 80.0) < 1e-9,
               "合并后的左边界必须等于最左边那段的左边界 80");
    }

    // ---- 11) 关掉合并（max_gap=0）：只并重叠，不并任何缝 ----
    {
        segment_merge::Params off; off.max_gap = 0.0;
        std::vector<gap_avoid::Rect> in{R(100, 20), R(139, 20)};   // 缝 -1（刚好相接）
        auto out = segment_merge::merge_collinear(in, off, &n);
        expect(out.size() == 1, "相接（gap=-1）在 max_gap=0 下也算并");
        std::vector<gap_avoid::Rect> in2{R(100, 20), R(142, 20)};  // 缝 2
        auto out2 = segment_merge::merge_collinear(in2, off, &n);
        expect(out2.size() == 2, "max_gap=0 时 2px 缝不并");
    }

    std::printf("比对项 %d，不符 %d\n", g_total, g_bad);
    return g_bad == 0 ? 0 : 1;
}
