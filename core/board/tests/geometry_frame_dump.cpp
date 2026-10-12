// ===========================================================================
// geometry_frame_dump —— 把几何帧字节解成**文本**，给 Python 侧对账用
//
// 存在的唯一理由：`core/standard/vision/vision_frame.py` 是线协议的**真值**，
// `core/board/geometry_frame.cpp` 是板上那份。两边必须逐字段一致，而"读一遍源码
// 觉得一样"不算数 —— 这个程序让 Python 能把编码好的字节喂进来，再把解出来的字段
// 逐项比回去。
//
// 输入（stdin）：连续的一帧帧，每帧前面 4 字节小端长度，然后是帧字节。
// 输出（stdout）：每帧一行，空格分隔：
//     FRAME <seq> <stamp_ns> <flags> <img_w> <img_h>
//           <player_present> <px> <py> <pr> <p_occluded>
//           <n_obs> <n_gaps>
//     OBS <id> <shape> <type_id> <x> <y> <radius> <half_w> <half_h> <rotation>
//     GAP <id> <cx> <cy> <width> <axis> <ba> <bb> <bt> <reliable> <occluded>
//
// 解析失败时输出一行 `ERR <说明>`，**继续**处理后面的帧（Python 侧要靠它测
// "坏帧必须被拒绝"）。浮点用 `%.17g`，Python 的 `float()` 能逐位还原。
// ===========================================================================
#include "geometry_frame.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <vector>

using namespace vision_board;

namespace {

uint32_t rd_u32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

void dump(const GeometryFrame& f) {
    std::printf("FRAME %u %llu %u %u %u %d %.17g %.17g %.17g %d %u %u\n",
                f.header.seq,
                static_cast<unsigned long long>(f.header.stamp_ns),
                static_cast<unsigned>(f.header.flags),
                static_cast<unsigned>(f.header.img_w),
                static_cast<unsigned>(f.header.img_h),
                f.player.present ? 1 : 0, f.player.x, f.player.y, f.player.radius,
                f.player.occluded ? 1 : 0,
                static_cast<unsigned>(f.obstacles.size()),
                static_cast<unsigned>(f.gaps.size()));
    for (const GeometryObstacle& o : f.obstacles) {
        std::printf("OBS %d %d %d %.17g %.17g %.17g %.17g %.17g %.17g\n",
                    o.id, o.shape, o.type_id, o.x, o.y, o.radius,
                    o.half_w, o.half_h, o.rotation);
    }
    for (const GeometryGap& g : f.gaps) {
        std::printf("GAP %d %.17g %.17g %.17g %.17g %d %d %d %d %d\n",
                    g.id, g.cx, g.cy, g.width, g.axis,
                    g.blocker_a, g.blocker_b, g.blocked_by_type,
                    g.reliable ? 1 : 0, g.occluded ? 1 : 0);
    }
    std::fflush(stdout);
}

}  // namespace

int main() {
    for (;;) {
        uint8_t lenb[4];
        const size_t got = std::fread(lenb, 1, 4, stdin);
        if (got == 0) break;                        // 正常结束
        if (got != 4) { std::printf("ERR 长度前缀不全\n"); break; }
        const uint32_t n = rd_u32le(lenb);
        if (n == 0 || n > (1u << 22)) { std::printf("ERR 长度前缀不合理：%u\n", n); break; }
        std::vector<uint8_t> bytes(n);
        if (std::fread(bytes.data(), 1, n, stdin) != n) {
            std::printf("ERR 帧字节不全（声明 %u）\n", n);
            break;
        }
        try {
            dump(parse_geometry_frame(bytes));
        } catch (const std::exception& e) {
            std::printf("ERR %s\n", e.what());
            std::fflush(stdout);
        }
    }
    return 0;
}
