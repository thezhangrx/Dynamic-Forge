// ===========================================================================
// fiducial_zone.hpp —— "这个位置是不是四角标定标记" 的纯函数
//
// 为什么要有这个文件：平台的四角 ArUco 标记是**已知位置**的亮块
// （`core/standard/fiducial.py`：黑边外沿边长 48 场地单位、离场地边 8、外围还有
// 一圈留白 8）。它们的填充度实测会偶尔越过视觉侧 `player_min_fill_pct = 45`
// 这条线，于是几何帧里的"角色"会突然跳到某个角落 —— 板子若照着那个位置发指令，
// 真角色就被推着往边上钻。
//
// 判据本身是纯几何、无状态：给定场地尺寸 + 标记参数，判断 (x, y) 是否落在任一
// 标记的 footprint 里。抽成头文件是为了**能在 PC 上直接测**（板子掉电/不在手边
// 时也能验），板上的 `main.cpp` 只是调用它。
//
// 坐标约定：与视觉帧一致（场地坐标系，单位是场地单位；四角就是四种符号组合，
// 所以 y 轴朝上还是朝下都不影响结论）。
// ===========================================================================
#pragma once

#include <cmath>

namespace fiducial_zone {

struct Zone {
    double field_w = 640.0;
    double field_h = 480.0;
    //: 标记黑边外沿边长（场地单位），平台契约默认 48。
    double side = 48.0;
    //: 黑边外沿到场地边的距离，平台契约默认 8。
    double margin = 8.0;
    //: 额外留白（场地单位）：标记外面还有一圈留白（默认 8），再加上相机透视/
    //: 量化误差。默认 12。
    double slack = 12.0;

    /// 标记中心到场地边的最小距离（= margin + side/2），方便外部核对。
    double centre_inset() const { return margin + side / 2.0; }

    /// 判据用的半边长：半个标记 + 留白。实测真角色（半径 10）不会长期贴角。
    double half() const { return side / 2.0 + slack; }

    /// (x, y) 是否落在四个标记中的任意一个 footprint 里。
    bool contains(double x, double y) const {
        const double h = half();
        const double cx[2] = {centre_inset(), field_w - centre_inset()};
        const double cy[2] = {centre_inset(), field_h - centre_inset()};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                if (std::fabs(x - cx[i]) <= h && std::fabs(y - cy[j]) <= h) return true;
            }
        }
        return false;
    }
};

}  // namespace fiducial_zone
