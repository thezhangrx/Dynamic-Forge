// ===========================================================================
// tb_axi.cpp —— 走 **PS 面向接口**（AXI4-Lite）的协同仿真
//
// tb_vision.cpp 直接驱动 vision_top 的引脚；这一份走真实的上板路径：
//
//   1. AXI 写配置寄存器（阈值）→ 读回来核对
//   2. AXI 写 CTRL 启动一帧 → 灌像素 → 轮询状态
//   3. AXI 读 FLEN（上一帧字节数）→ 读几何帧窗口 → 与 C 参考模型逐字节比
//   4. AXI 读调试口 → 与 C 参考模型的行直方图/行带比
//
// 为什么值得单独写一份：`vision_top` 逐位对是 P2 的验收，但**上板时 PS 碰到的
// 是 AXI 口**。地址译码错、握手错、窗口读错、CTRL 位定义错 —— 这些在
// tb_vision.cpp 里全都测不到，而在板上表现为"读回来全是 0"或"卡死"。
//
// 用法:  ./Vvision_axi <stim_dir> <frames> [--verbose]
// ===========================================================================

#include <verilated.h>
#include "Vvision_axi.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "ref_api.h"
}

static const int MAX_W = 640;
static const int MAX_H = 480;

// ---- 地址映射（**必须与 rtl/vision_axi.v 一致**，契约测试会核对）---------
enum {
    A_CFG_WALL_RB      = 0x000,   // 4*0
    A_CFG_WALL_MIN_G   = 0x008,   // 4*2
    A_CFG_BRIGHT_MIN   = 0x00C,   // 4*3
    A_CFG_BRIGHT_SPRD  = 0x010,   // 4*4
    A_CFG_MIN_GAP      = 0x040,   // 4*16
    A_CFG_P_MIN_AREA   = 0x044,   // 4*17
    A_DBG_SEL        = 0x080,
    A_DBG_DATA       = 0x084,
    A_CTRL           = 0x0C0,
    A_SEQ            = 0x0C4,
    A_STAMP0         = 0x0C8,
    A_STAMP1         = 0x0CC,
    A_FLEN           = 0x0D0,
    A_FRAME          = 0x100,
    A_CFG_STATUS     = 0x07C    // 4*31：vision_top 的 CFG_STATUS
};

static Vvision_axi *top = nullptr;
static void step();

static long g_checks = 0, g_fail = 0;
static bool g_verbose = false;

static void cmp_u32(const char *what, long long idx, uint32_t got, uint32_t want)
{
    g_checks++;
    if (got == want) return;
    g_fail++;
    if (g_fail <= 30) printf("  ✗ %s[%lld]  AXI=%u  期望=%u\n", what, idx, got, want);
}

// ===========================================================================
// 一个够用的 AXI4-Lite 主机模型
// 注意：从机把读写**串行化**（对方事务未完成时压低自己的 ready），
// 所以这里也必须一次只发一个事务 —— 这正是裸机驱动的访问方式。
// ===========================================================================
static void axi_idle()
{
    top->s_awvalid = 0; top->s_wvalid = 0; top->s_bready = 0;
    top->s_arvalid = 0; top->s_rready = 0;
    top->eval();
}

static bool axi_write(uint32_t addr, uint32_t data)
{
    top->s_awaddr  = addr;  top->s_awvalid = 1;
    top->s_wdata   = data;  top->s_wstrb   = 0xF;  top->s_wvalid = 1;
    top->s_bready  = 1;
    bool done = false;
    for (int i = 0; i < 500 && !done; i++) {
        bool aw_fire = top->s_awvalid && top->s_awready;
        bool w_fire  = top->s_wvalid  && top->s_wready;
        bool b_fire  = top->s_bvalid  && top->s_bready;
        step();
        if (aw_fire) top->s_awvalid = 0;
        if (w_fire)  top->s_wvalid  = 0;
        if (b_fire)  done = true;
    }
    axi_idle();
    return done;
}

static bool axi_read(uint32_t addr, uint32_t *out)
{
    top->s_araddr  = addr; top->s_arvalid = 1; top->s_rready = 1;
    bool done = false;
    for (int i = 0; i < 500 && !done; i++) {
        bool ar_fire = top->s_arvalid && top->s_arready;
        if (top->s_rvalid && top->s_rready) { *out = (uint32_t)top->s_rdata; done = true; }
        step();
        if (ar_fire) top->s_arvalid = 0;
    }
    axi_idle();
    return done;
}

// ===========================================================================
static void step()
{
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    top->clk = 0; top->eval();
}

static bool load_ppm(const std::string &path, int *w, int *h, std::vector<uint8_t> &rgb)
{
    FILE *fp = fopen(path.c_str(), "rb");
    if (!fp) return false;
    char magic[3] = {0};
    if (fscanf(fp, "%2s", magic) != 1 || strcmp(magic, "P6") != 0) { fclose(fp); return false; }
    int vals[2] = {0, 0}, n = 0, maxv = 0, c, seen = 0;
    while (n < 2 && (c = fgetc(fp)) != EOF) {
        if (c == '#') { while ((c = fgetc(fp)) != EOF && c != '\n') {} continue; }
        if (c >= '0' && c <= '9') vals[n] = vals[n] * 10 + (c - '0');
        else if (vals[n] > 0) n++;
    }
    while ((c = fgetc(fp)) != EOF) {
        if (c >= '0' && c <= '9') maxv = maxv * 10 + (c - '0');
        else if (maxv > 0) { seen = 1; break; }
    }
    if (!seen || maxv != 255) { fclose(fp); return false; }
    *w = vals[0]; *h = vals[1];
    rgb.resize((size_t)(*w) * (*h) * 3);
    if (fread(rgb.data(), 1, rgb.size(), fp) != rgb.size()) { fclose(fp); return false; }
    fclose(fp);
    return true;
}

#define CHECK(cond, msg) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("  ✗ %s\n", msg); } } while (0)

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    if (argc < 3) {
        fprintf(stderr, "用法: %s <stim_dir> <frames> [--verbose] [--bright N]\n", argv[0]);
        return 2;
    }
    std::string dir = argv[1];
    int frames = atoi(argv[2]);
    VrRefParams params;
    vr_ref_default_params(&params);
    uint32_t rtl_only_addr = 0xFFFFFFFFu, rtl_only_val = 0;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--verbose")) g_verbose = true;
        else if (!strcmp(argv[i], "--bright") && i + 1 < argc) params.bright_min = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bright-spread") && i + 1 < argc) params.bright_spread = atoi(argv[++i]);
        // 反面控制：只改 RTL 的寄存器、不改 C 的参数 → AXI 路径必须报不一致
        else if (!strcmp(argv[i], "--rtl-only") && i + 2 < argc) {
            rtl_only_addr = (uint32_t)strtoul(argv[++i], nullptr, 0);
            rtl_only_val  = (uint32_t)strtoul(argv[++i], nullptr, 0);
        }
    }
    bool rtl_only = (rtl_only_addr != 0xFFFFFFFFu);

    top = new Vvision_axi;

    // 复位 + AXI 空闲
    top->aresetn = 0;
    top->din_vld = 0; top->din_sof = 0; top->din_eol = 0;
    top->din_r = top->din_g = top->din_b = 0;
    axi_idle();
    for (int i = 0; i < 8; i++) step();
    top->aresetn = 1;
    step();

    std::vector<uint8_t> rgb;
    std::vector<uint8_t> gray_g(MAX_W * MAX_H), wall_g(MAX_W * MAX_H), bright_g(MAX_W * MAX_H);
    std::vector<uint16_t> rowc_g(MAX_H);
    std::vector<uint8_t> frame_g(8192);

    // ---- 1) 配置寄存器：写进去再读回来 --------------------------------
    uint32_t v = 0;
    for (int f = 0; f < frames; f++) {
        char name[64];
        snprintf(name, sizeof(name), "%06d.ppm", f);
        int w = 0, h = 0;
        if (!load_ppm(dir + "/" + name, &w, &h, rgb)) { printf("激励读取失败：%s\n", name); break; }

        // 每帧都重写一遍配置（顺便测配置寄存器的读写）
        CHECK(axi_write(A_CFG_WALL_RB,    (uint32_t)params.wall_rb), "写 WALL_RB 超时");
        CHECK(axi_write(A_CFG_WALL_MIN_G, (uint32_t)params.wall_min_g), "写 WALL_MIN_G 超时");
        CHECK(axi_write(A_CFG_BRIGHT_MIN, (uint32_t)params.bright_min), "写 BRIGHT_MIN 超时");
        CHECK(axi_write(A_CFG_BRIGHT_SPRD,(uint32_t)params.bright_spread), "写 BRIGHT_SPREAD 超时");
        CHECK(axi_write(A_CFG_MIN_GAP,    (uint32_t)params.min_gap_px), "写 MIN_GAP 超时");
        CHECK(axi_write(A_CFG_P_MIN_AREA, (uint32_t)params.player_min_area), "写 P_MIN_AREA 超时");
        CHECK(axi_write(A_SEQ, (uint32_t)f), "写 SEQ 超时");
        if (rtl_only) axi_write(rtl_only_addr, rtl_only_val);
        if (f == 0) {
            CHECK(axi_read(A_CFG_WALL_RB, &v)    && v == (uint32_t)params.wall_rb,   "读回 WALL_RB 不符");
            CHECK(axi_read(A_CFG_BRIGHT_MIN, &v) && v == (uint32_t)params.bright_min,"读回 BRIGHT_MIN 不符");
            CHECK(axi_read(A_CFG_BRIGHT_SPRD, &v) && v == (uint32_t)params.bright_spread, "读回 BRIGHT_SPREAD 不符");
            CHECK(axi_read(A_CFG_MIN_GAP, &v)    && v == (uint32_t)params.min_gap_px,"读回 MIN_GAP 不符");
            CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == (uint32_t)params.player_min_area, "读回 P_MIN_AREA 不符");
            CHECK(axi_read(A_SEQ, &v)            && v == 0,                          "读回 SEQ 不符");
        }

        // ---- 2) 启动一帧，然后灌像素 ---------------------------------
        VrRefResult res;
        memset(&res, 0, sizeof(res));
        if (vr_ref_process_ex(rgb.data(), w, h, 1, gray_g.data(), wall_g.data(),
                              bright_g.data(), rowc_g.data(), &res,
                              frame_g.data(), frame_g.size(), &params,
                              (uint32_t)f, 0) != 0) return 1;

        CHECK(axi_write(A_CTRL, 1), "写 CTRL 启动超时");
        for (int i = 0; i < 4; i++) step();      // 让状态机进 S_RECV

        size_t npix = (size_t)w * h;
        for (size_t i = 0; i < npix; i++) {
            int x = (int)(i % (size_t)w);
            top->din_vld = 1;
            top->din_sof = (i == 0) ? 1 : 0;
            top->din_eol = (x == w - 1) ? 1 : 0;
            top->din_r = rgb[i * 3 + 0];
            top->din_g = rgb[i * 3 + 1];
            top->din_b = rgb[i * 3 + 2];
            step();
        }
        top->din_vld = 0; top->din_sof = 0; top->din_eol = 0;

        // ---- 3) 轮询到帧完成 -----------------------------------------
        bool ok = false;
        for (long i = 0; i < 20000000L; i++) {
            step();
            // 直接读 CTRL 太重（AXI 事务若干拍），每 256 拍看一次
            if ((i & 0xFF) == 0) {
                uint32_t st = 0;
                if (!axi_read(A_CTRL, &st)) break;
                if ((st & 0x2) && !(st & 0x1)) { ok = true; break; }
            }
        }
        CHECK(ok, "轮询帧完成超时");
        if (!ok) continue;

        // ---- 4) FLEN + 帧窗口：与 C 参考模型逐字节比 ------------------
        uint32_t flen = 0;
        CHECK(axi_read(A_FLEN, &flen), "读 FLEN 超时");
        cmp_u32("frame_len", f, flen, (uint32_t)res.frame_bytes);

        size_t nbytes = flen < res.frame_bytes ? flen : res.frame_bytes;
        for (size_t i = 0; i < nbytes; i += 4) {
            uint32_t word = 0;
            if (!axi_read(A_FRAME + (uint32_t)i, &word)) { g_checks++; g_fail++;
                printf("  ✗ 读帧窗口 [%zu] 超时\n", i); break; }
            for (size_t k = 0; k < 4 && i + k < nbytes; k++)
                cmp_u32("frame", (long long)(i + k), (word >> (8 * k)) & 0xFFu, frame_g[i + k]);
        }
        uint32_t st = 0;
        axi_read(A_CTRL, &st);
        cmp_u32("frame_ovf", f, (st >> 24) & 1u, 0u);

        // ---- 5) 调试口：行直方图（和 C 参考模型比）-------------------
        if (f == 0) {
            // 写调试选择：{sub, idx[17:0], sel}
            CHECK(axi_write(A_DBG_SEL, (0u << 22) | (0u << 4) | 3u), "写 DBG_SEL 超时");
            for (int y = 0; y < h; y++) {
                CHECK(axi_write(A_DBG_SEL, ((uint32_t)y << 4) | 3u), "写 DBG_SEL 超时");
                CHECK(axi_read(A_DBG_DATA, &v), "读 DBG_DATA 超时");
                cmp_u32("row_count", y, v & 0xFFFFu, rowc_g[y]);
            }
            // 读状态寄存器（vision_top 的 CFG_STATUS：bit0 busy, bit1 done, [23:8] 计数）
            CHECK(axi_read(A_CFG_STATUS, &v), "读 CFG_STATUS 超时");
            cmp_u32("status.done", 0, (v >> 1) & 1u, 1u);
            cmp_u32("status.frames", 0, (v >> 8) & 0xFFFFu, 1u);
        }

        if (g_verbose)
            printf("帧 %d: AXI 路径一致（%u 字节, %d 障碍, %d 缺口, 角色 %d）\n",
                   f, flen, res.n_obstacles, res.n_gaps, res.player_valid);
    }

    printf("\n=== AXI 协同仿真结果 ===\n");
    printf("比对项 %ld，不符 %ld\n", g_checks, g_fail);
    delete top;
    return g_fail == 0 ? 0 : 1;
}
