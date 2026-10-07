// ===========================================================================
// tb_tpg.cpp —— **PL 自检图案**的解析验收（不依赖相机，也不依赖 C 参考模型）
//
// vision_tpg 生成的场地几何是手算得出来的，所以这一份测试**不用 C 模型当
// 参照**：它直接把手算的墙带/缺口/角色和 RTL 的输出比。这才是独立的检查 ——
// tb_vision / tb_axi 比的是"RTL 等于 C 模型"，两者一起错是测不出来的。
//
// 场景（见 vision_tpg.v；idx = frame_idx & 7）：
//     wall_y = 60 + (idx&3)*40         墙带占 24 行
//     gap_x  = 200 + (idx&3)*50        缺口左右各 30 px
//     p_px   = 300 + (idx&7)*20        角色圆盘 r = 10
//     p_py   = 280 + (idx&3)*20
//     墙 x 范围 [80, 559]
//
// 用法:  ./Vvision_selftest <帧数> [--verbose]
// ===========================================================================

#include <verilated.h>
#include "Vvision_selftest.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>

static Vvision_selftest *top = nullptr;
static void step()
{
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    top->clk = 0; top->eval();
}

static long g_checks = 0, g_fail = 0;
static bool g_verbose = false;

static void chk(const char *what, long long idx, long long got, long long want)
{
    g_checks++;
    if (got == want) return;
    g_fail++;
    if (g_fail <= 30) printf("  ✗ %s[%lld]  RTL=%lld  手算=%lld\n", what, idx, got, want);
}

// ---- 场景参数（**必须与 vision_tpg.v 同一组公式**）-----------------------
struct Scene {
    int wall_y, gap_x, p_px, p_py;
    static const int BAR_X0 = 80, BAR_X1 = 559, BAR_H = 24, GAP_W = 60, R = 10;
    int seg0_x0() const { return BAR_X0; }
    int seg0_x1() const { return gap_x - GAP_W / 2 - 1; }
    int seg1_x0() const { return gap_x + GAP_W / 2; }
    int seg1_x1() const { return BAR_X1; }
    int cy_q() const { return wall_y * 64 + 736; }      // 24 行的均值 = wall_y + 11.5
};

static Scene scene_of(int idx)
{
    Scene s;
    s.wall_y = 60 + (idx & 3) * 40;
    s.gap_x  = 200 + (idx & 3) * 50;
    s.p_px   = 300 + (idx & 7) * 20;
    s.p_py   = 280 + (idx & 3) * 20;
    return s;
}

static int32_t rd32(const uint8_t *p, size_t off)
{
    return (int32_t)((uint32_t)p[off] | ((uint32_t)p[off+1] << 8) |
                     ((uint32_t)p[off+2] << 16) | ((uint32_t)p[off+3] << 24));
}

// 调试口读（组合）
static uint32_t dbg(int sel, int idx, int sub = 0)
{
    top->dbg_sel = (uint8_t)sel;
    top->dbg_idx = (uint32_t)idx;
    top->dbg_sub = (uint8_t)sub;
    top->eval();
    return (uint32_t)top->dbg_data;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    int frames = (argc >= 2) ? atoi(argv[1]) : 8;
    for (int i = 2; i < argc; i++) if (!strcmp(argv[i], "--verbose")) g_verbose = true;

    top = new Vvision_selftest;
    top->clk = 0;
    top->rst_n = 0;
    top->auto_run = 0; top->start = 0; top->frame_idx = 0;
    top->dbg_sel = 0; top->dbg_idx = 0; top->dbg_sub = 0;
    for (int i = 0; i < 8; i++) step();
    top->rst_n = 1;
    step();

    // 角色圆盘的格点个数（手算）
    int disc_area = 0;
    for (int dy = -Scene::R; dy <= Scene::R; dy++)
        for (int dx = -Scene::R; dx <= Scene::R; dx++)
            if (dx * dx + dy * dy <= Scene::R * Scene::R) disc_area++;

    for (int f = 0; f < frames; f++) {
        Scene sc = scene_of(f);
        top->frame_idx = (uint16_t)f;
        top->start = 1; step(); top->start = 0;

        std::vector<uint8_t> bytes;
        bool done = false;
        int last_pulses = 0;
        for (long g = 0; g < 40000000L && !done; g++) {
            if (top->dout_vld) {
                bytes.push_back((uint8_t)top->dout_data);
                if (top->dout_last) last_pulses++;
            }
            step();
            if (top->frame_done) done = true;
        }
        chk("frame_done", f, done ? 1 : 0, 1);
        if (!done) continue;
        chk("dout_last 次数", f, last_pulses, 1);
        chk("帧长", f, (long long)bytes.size(), 76 + 36 * 2 + 36 * 1);

        // ---- 帧头 ----------------------------------------------------
        chk("magic", f, (int32_t)((uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                                  ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24)),
            0x31564842);
        chk("version", f, bytes[4], 1);
        chk("flags.player", f, (bytes[6] >> 1) & 1, 1);
        chk("seq", f, rd32(bytes.data(), 8), 0);
        chk("n_obs", f, rd32(bytes.data(), 12), 2);
        chk("n_gaps", f, rd32(bytes.data(), 16), 1);
        chk("width", f, rd32(bytes.data(), 20) & 0xFFFF, 640);
        chk("height", f, rd32(bytes.data(), 22) & 0xFFFF, 360);

        // ---- 角色 ----------------------------------------------------
        chk("player.x", f, rd32(bytes.data(), 32), sc.p_px * 64);
        chk("player.y", f, rd32(bytes.data(), 36), sc.p_py * 64);
        chk("player.r", f, rd32(bytes.data(), 40), (2 * Scene::R + 1) * 32);
        chk("player.flags", f, rd32(bytes.data(), 44), 1);

        // ---- 两个墙段 -------------------------------------------------
        const size_t o0 = 72, o1 = 72 + 36;
        chk("obs0.id", f, rd32(bytes.data(), o0 + 0), 1);
        chk("obs0.shape", f, rd32(bytes.data(), o0 + 4), 1);
        chk("obs0.type", f, rd32(bytes.data(), o0 + 8), 2);
        chk("obs0.x", f, rd32(bytes.data(), o0 + 12), (sc.seg0_x0() + sc.seg0_x1()) * 32);
        chk("obs0.y", f, rd32(bytes.data(), o0 + 16), sc.cy_q());
        chk("obs0.radius", f, rd32(bytes.data(), o0 + 20), 0);
        chk("obs0.half_w", f, rd32(bytes.data(), o0 + 24),
            (sc.seg0_x1() - sc.seg0_x0() + 1) * 32);
        chk("obs0.half_h", f, rd32(bytes.data(), o0 + 28), Scene::BAR_H * 32);
        chk("obs0.rot", f, rd32(bytes.data(), o0 + 32), 0);

        chk("obs1.id", f, rd32(bytes.data(), o1 + 0), 2);
        chk("obs1.x", f, rd32(bytes.data(), o1 + 12), (sc.seg1_x0() + sc.seg1_x1()) * 32);
        chk("obs1.y", f, rd32(bytes.data(), o1 + 16), sc.cy_q());
        chk("obs1.half_w", f, rd32(bytes.data(), o1 + 24),
            (sc.seg1_x1() - sc.seg1_x0() + 1) * 32);
        chk("obs1.half_h", f, rd32(bytes.data(), o1 + 28), Scene::BAR_H * 32);
        chk("obs1.rot", f, rd32(bytes.data(), o1 + 32), 0);

        // ---- 缺口 ----------------------------------------------------
        const size_t g0 = 72 + 72;
        chk("gap.id", f, rd32(bytes.data(), g0 + 0), 1);
        chk("gap.cx", f, rd32(bytes.data(), g0 + 4),
            (sc.seg0_x1() + sc.seg1_x0()) * 32);
        chk("gap.cy", f, rd32(bytes.data(), g0 + 8), sc.cy_q());
        chk("gap.width", f, rd32(bytes.data(), g0 + 12), Scene::GAP_W * 64);
        chk("gap.axis", f, rd32(bytes.data(), g0 + 16), 0);
        chk("gap.ba", f, rd32(bytes.data(), g0 + 20), 1);
        chk("gap.bb", f, rd32(bytes.data(), g0 + 24), 2);
        chk("gap.type", f, rd32(bytes.data(), g0 + 28), 2);
        chk("gap.flags", f, rd32(bytes.data(), g0 + 32), 1);

        // ---- 中间量（调试口）-----------------------------------------
        chk("n_bands", f, dbg(7, 0, 0), 1);
        chk("band_y0", f, dbg(5, 0), sc.wall_y);
        chk("band_y1", f, dbg(6, 0), sc.wall_y + Scene::BAR_H - 1);
        chk("x_min", f, dbg(7, 0, 1), Scene::BAR_X0);
        chk("x_max", f, dbg(7, 0, 2), Scene::BAR_X1);
        chk("n_labels", f, dbg(7, 0, 5), 1);
        chk("blob_x0", f, dbg(11, 0, 0), sc.p_px - Scene::R);
        chk("blob_x1", f, dbg(11, 0, 1), sc.p_px + Scene::R);
        chk("blob_y0", f, dbg(11, 0, 2), sc.p_py - Scene::R);
        chk("blob_y1", f, dbg(11, 0, 3), sc.p_py + Scene::R);
        chk("blob_area", f, dbg(11, 0, 4), disc_area);

        if (g_verbose)
            printf("场景 %d (wall_y=%d gap_x=%d player=(%d,%d)): %zu 字节，全部与手算一致\n",
                   f, sc.wall_y, sc.gap_x, sc.p_px, sc.p_py, bytes.size());
    }

    // ---- 自由运行模式（上板就是这条：auto_run=1，不需要任何外部激励）----
    // 板上没人给 start，所以这一条必须单独验：自动节拍能不能一帧接一帧地跑，
    // 而且每帧的 dout_last 还是恰好一次。
    {
        long n_done = 0, n_last = 0, n_bytes = 0;
        bool prev_done = false;
        top->auto_run = 1;
        top->start = 0;
        for (long i = 0; i < 40000000L; i++) {
            if (top->dout_vld) {
                n_bytes++;
                if (top->dout_last) n_last++;
            }
            step();
            bool d = top->frame_done;
            if (d && !prev_done) n_done++;
            prev_done = d;
            if (n_done >= 3) break;
        }
        chk("auto: 完成帧数 >= 3", 0, n_done >= 3 ? 1 : 0, 1);
        chk("auto: dout_last 次数 == 完成帧数", 0, n_last, n_done);
        chk("auto: 每帧 184 字节", 0, n_bytes, n_done * 184);
        if (g_verbose) printf("自由运行：%ld 帧，%ld 字节，dout_last %ld 次\n",
                              n_done, n_bytes, n_last);
    }

    printf("\n=== 自检图案结果 ===\n");
    printf("比对项 %ld，不符 %ld\n", g_checks, g_fail);
    delete top;
    return g_fail == 0 ? 0 : 1;
}
