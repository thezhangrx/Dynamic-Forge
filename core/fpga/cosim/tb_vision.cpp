// ===========================================================================
// tb_vision.cpp —— RTL ↔ C 参考模型的**逐位协同仿真台**
//
// 同一帧同一批像素同时喂给：
//   * Verilator 编出来的 vision_top（RTL）
//   * core/fpga/ref/vision_ref.c（C 参考模型，-DVR_NO_MAIN 编成 .o 链进来）
// 然后逐位比对：灰度/墙/亮三张掩膜、行直方图、行带、场地 x 范围、墙段记录、
// 缺口记录、角色、以及**最终几何帧的全部字节**（含 CRC32）。
//
// 为什么同一个进程里比，而不是两边各写文件再 diff：中间量不必发明一套 dump
// 格式，出问题的第一现场可以直接打印（哪一帧、哪一类、第几个、期望/实际）。
//
// 用法:  ./Vvision_top <stim_dir> <frames> [--verbose]
//   激励目录里是 export_frames.py --color 产出的 000000.ppm ...
// ===========================================================================

#include <verilated.h>
#include "Vvision_top.h"
#include "Vvision_top___024root.h"

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

static Vvision_top *top = nullptr;
static void step();          // 定义在下面；配置寄存器读写要先看到它

// 配置寄存器地址（**必须与 rtl/vision_top.v 的 CFG_* 一致**，
// 契约测试 core/fpga/tests/test_rtl_contract.py 会核对）
// 显式写出数值：test_rtl_contract.py 会把这份表与 vision_top.v 的 CFG_*
// 逐项核对，隐式自增的枚举解析起来太脆。
enum {
    CFG_WALL_RB      = 0,
    CFG_WALL_MIN_R   = 1,
    CFG_WALL_MIN_G   = 2,
    CFG_BRIGHT_MIN   = 3,
    CFG_BRIGHT_SPRD  = 4,
    CFG_GREEN_GM     = 5,
    CFG_GREEN_MIN    = 6,
    CFG_BAND_PCT     = 7,
    CFG_BAND_JOIN    = 8,
    CFG_BAND_MINR    = 9,
    CFG_FIELD_COL    = 10,
    CFG_WALL_PCT     = 11,
    CFG_WALL_COLMIN  = 12,
    CFG_EDGE_MIN     = 13,
    CFG_EDGE_COLS    = 14,
    CFG_MIN_SEG      = 15,
    CFG_MIN_GAP      = 16,
    CFG_P_MIN_AREA   = 17,
    CFG_P_MAX_AREA   = 18,
    CFG_P_FILL       = 19,
    CFG_P_ASPECT     = 20,
    CFG_STATUS       = 31
};

static void cfg_write(int addr, uint32_t v)
{
    top->cfg_we = 1; top->cfg_addr = (uint8_t)addr; top->cfg_wdata = v;
    step();
    top->cfg_we = 0; top->cfg_wdata = 0;
    top->eval();
}

static uint32_t cfg_read(int addr)
{
    top->cfg_we = 0; top->cfg_addr = (uint8_t)addr;
    top->eval();
    return (uint32_t)top->cfg_rdata;
}

// 把一组参数写进 RTL 的配置寄存器。默认值 = C 参考模型的 VR_DEFAULT_*。
static void cfg_apply(const VrRefParams &p)
{
    cfg_write(CFG_WALL_RB,    (uint32_t)p.wall_rb);
    cfg_write(CFG_WALL_MIN_R, (uint32_t)p.wall_min_r);
    cfg_write(CFG_WALL_MIN_G, (uint32_t)p.wall_min_g);
    cfg_write(CFG_BRIGHT_MIN, (uint32_t)p.bright_min);
    cfg_write(CFG_BRIGHT_SPRD,(uint32_t)p.bright_spread);
    cfg_write(CFG_GREEN_GM,   (uint32_t)p.green_gm);
    cfg_write(CFG_GREEN_MIN,  (uint32_t)p.green_min);
    cfg_write(CFG_BAND_PCT,   (uint32_t)p.band_row_pct);
    cfg_write(CFG_BAND_JOIN,  (uint32_t)p.band_join_rows);
    cfg_write(CFG_BAND_MINR,  (uint32_t)p.band_min_rows);
    cfg_write(CFG_FIELD_COL,  (uint32_t)p.field_col_min);
    cfg_write(CFG_WALL_PCT,   (uint32_t)p.wall_col_pct);
    cfg_write(CFG_WALL_COLMIN,(uint32_t)p.wall_col_min);
    cfg_write(CFG_EDGE_MIN,   (uint32_t)p.edge_min);
    cfg_write(CFG_EDGE_COLS,  (uint32_t)p.edge_min_cols);
    cfg_write(CFG_MIN_SEG,    (uint32_t)p.min_seg_px);
    cfg_write(CFG_MIN_GAP,    (uint32_t)p.min_gap_px);
    cfg_write(CFG_P_MIN_AREA, (uint32_t)p.player_min_area);
    cfg_write(CFG_P_MAX_AREA, (uint32_t)p.player_max_area);
    cfg_write(CFG_P_FILL,     (uint32_t)p.player_min_fill_pct);
    cfg_write(CFG_P_ASPECT,   (uint32_t)p.player_max_aspect_pct);
}

// ---- 时钟 ----------------------------------------------------------------
static void step()
{
    top->pix_clk = 0; top->eval();
    top->pix_clk = 1; top->eval();
    top->pix_clk = 0; top->eval();
}

// ---- P6 PPM 读取（export_frames.py 写的就是这个）-------------------------
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

// ---- 计数器 --------------------------------------------------------------
static long g_checks = 0, g_fail = 0;
static int  g_fail_frames = 0;
static bool g_verbose = false;
// 整轮几何帧字节的 FNV-1a 折叠：给"这组参数到底有没有改变输出"一个廉价判据。
// 只比"两边一致"是不够的 —— 两边都没变也是"一致"，那样配置寄存器接错也发现不了。
static uint64_t g_hash = 1469598103934665603ull;

static void cmp_u32(const char *what, long long idx, uint32_t rtl, uint32_t gold)
{
    g_checks++;
    if (rtl == gold) return;
    g_fail++;
    if (g_fail <= 30)
        printf("  ✗ %s[%lld]  RTL=%u  C=%u\n", what, idx, rtl, gold);
}

static void cmp_bytes(const char *what, const uint8_t *rtl, const uint8_t *gold, size_t n)
{
    size_t first = n;
    for (size_t i = 0; i < n; i++) {
        g_checks++;
        if (rtl[i] != gold[i]) {
            g_fail++;
            if (first == n) first = i;
        }
    }
    if (first != n)
        printf("  ✗ %s 长度 %zu，首个不符在 [%zu]: RTL=%u C=%u\n",
               what, n, first, rtl[first], gold[first]);
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
    if (argc < 3) {
        fprintf(stderr, "用法: %s <stim_dir> <frames> [--verbose]\n", argv[0]);
        return 2;
    }
    std::string dir = argv[1];
    int frames = atoi(argv[2]);
    VrRefParams params;
    vr_ref_default_params(&params);
    int rtl_only_addr = -1; uint32_t rtl_only_val = 0;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--verbose")) g_verbose = true;
        // 改一个参数（两边一起改）：用来验证"配置寄存器真的接到了那个判据上"。
        // 只改 RTL 不改 C，或者改了却接错寄存器，这一条立刻挂。
        else if (!strcmp(argv[i], "--bright") && i + 1 < argc) params.bright_min = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wall-rb") && i + 1 < argc) params.wall_rb = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wall-min-g") && i + 1 < argc) params.wall_min_g = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bright-spread") && i + 1 < argc) params.bright_spread = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--min-seg") && i + 1 < argc) params.min_seg_px = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--min-gap") && i + 1 < argc) params.min_gap_px = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--p-min-area") && i + 1 < argc) params.player_min_area = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--band-pct") && i + 1 < argc) params.band_row_pct = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wall-min-g") && i + 1 < argc) params.wall_min_g = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bright-spread") && i + 1 < argc) params.bright_spread = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--p-fill") && i + 1 < argc) params.player_min_fill_pct = atoi(argv[++i]);
        // **反面控制**：只改 RTL 的寄存器、不改 C 的参数。这样两边必须**不一致** ——
        // 用它来证明"配置寄存器真的参与了判据"，而不是碰巧两边都没变。
        else if (!strcmp(argv[i], "--rtl-only") && i + 2 < argc) {
            rtl_only_addr = atoi(argv[++i]);
            rtl_only_val  = (uint32_t)atoi(argv[++i]);
        }
    }

    top = new Vvision_top;

    // 复位
    top->rst_n = 0;
    top->frame_start = 0; top->din_vld = 0; top->din_sof = 0; top->din_eol = 0;
    top->din_r = top->din_g = top->din_b = 0;
    top->dout_ready = 0;
    top->cfg_seq = 0; top->cfg_stamp_ns = 0;
    top->cfg_we = 0; top->cfg_addr = 0; top->cfg_wdata = 0;
    top->dbg_sel = 0; top->dbg_idx = 0; top->dbg_sub = 0;
    for (int i = 0; i < 8; i++) step();
    top->rst_n = 1;
    step();
    // 把参数（默认或命令行改过的）写进配置寄存器 —— 走一遍真实的 PS 写路径
    cfg_apply(params);
    if (rtl_only_addr >= 0) cfg_write(rtl_only_addr, rtl_only_val);

    std::vector<uint8_t> rgb;
    std::vector<uint8_t> gray_g(MAX_W * MAX_H), wall_g(MAX_W * MAX_H), bright_g(MAX_W * MAX_H);
    std::vector<uint16_t> rowc_g(MAX_H);
    std::vector<uint8_t> frame_g(8192);
    std::vector<uint8_t> frame_rtl;

    for (int f = 0; f < frames; f++) {
        char name[64];
        snprintf(name, sizeof(name), "%06d.ppm", f);
        std::string path = dir + "/" + name;
        int w = 0, h = 0;
        if (!load_ppm(path, &w, &h, rgb)) {
            printf("激励读取失败：%s（到此为止）\n", path.c_str());
            break;
        }
        if (w > MAX_W || h > MAX_H) { printf("帧 %dx%d 超过上限\n", w, h); break; }

        // ---- 金标准 ----
        VrRefResult res;
        memset(&res, 0, sizeof(res));
        if (vr_ref_process_ex(rgb.data(), w, h, 1,
                              gray_g.data(), wall_g.data(), bright_g.data(), rowc_g.data(),
                              &res, frame_g.data(), frame_g.size(), &params, 0, 0) != 0) {
            printf("C 参考模型内存不足\n");
            return 1;
        }

        // ---- 驱动 RTL ----
        top->frame_start = 1;
        step();
        top->frame_start = 0;

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

        // ---- 收几何帧字节流 ----
        frame_rtl.clear();
        top->dout_ready = 1;
        bool done = false;
        int last_pulses = 0;
        long guard = 64L * 1000 * 1000;
        while (!done && guard-- > 0) {
            if (top->dout_vld) {
                frame_rtl.push_back((uint8_t)top->dout_data);
                if (top->dout_last) last_pulses++;
            }
            step();
            if (top->frame_done) done = true;
        }
        // dout_last 每帧必须**恰好**一次，而且是整帧的最后一字节。
        // 早先它同时挂在"payload 最后一字节"和"CRC 最后一字节"上，每帧拉两次；
        // 纯字节流看不出来，但下游拿它定帧长就会算成 4 个字节。
        cmp_u32("dout_last 次数", f, (uint32_t)last_pulses, 1u);
        top->dout_ready = 0;
        if (!done) { printf("帧 %d: RTL 超时未出帧（guard 用尽）\n", f); g_fail_frames++; continue; }

        // ---- 比对 ----
        long fail_before = g_fail;
        // 1) 掩膜与行直方图（直接读 RTL 内部帧存）
        for (size_t i = 0; i < npix; i++) {
            cmp_u32("gray",   (long long)i, (uint32_t)top->rootp->vision_top__DOT__gray_mem[i],   gray_g[i]);
            cmp_u32("wall",   (long long)i, (uint32_t)top->rootp->vision_top__DOT__wall_mem[i],   wall_g[i]);
            cmp_u32("bright", (long long)i, (uint32_t)top->rootp->vision_top__DOT__bright_mem[i], bright_g[i]);
        }
        for (int y = 0; y < h; y++)
            cmp_u32("row_count", y, (uint32_t)top->rootp->vision_top__DOT__row_count[y], rowc_g[y]);

        // 2) 行带 / 场地范围 / 计数
        cmp_u32("n_bands", 0, dbg(7, 0, 0), (uint32_t)res.n_bands);
        cmp_u32("x_min",   0, dbg(7, 0, 1), (uint32_t)res.x_min);
        cmp_u32("x_max",   0, dbg(7, 0, 2), (uint32_t)res.x_max);
        cmp_u32("n_obs",   0, dbg(7, 0, 3), (uint32_t)res.n_obstacles);
        cmp_u32("n_gaps",  0, dbg(7, 0, 4), (uint32_t)res.n_gaps);
        cmp_u32("n_labels",0, dbg(7, 0, 5), (uint32_t)res.n_labels);
        for (int b = 0; b < res.n_bands; b++) {
            cmp_u32("band_y0", b, dbg(5, b), (uint32_t)res.band_y0[b]);
            cmp_u32("band_y1", b, dbg(6, b), (uint32_t)res.band_y1[b]);
        }

        // 3) 障碍/缺口记录
        if (g_verbose) {
            for (int r = 0; r < res.n_obstacles; r++) {
                printf("    obs%-2d RTL:", r);
                for (int c = 0; c < 9; c++) printf(" %11d", (int)dbg(8, r * 9 + c));
                printf("\n         C  :");
                for (int c = 0; c < 9; c++) printf(" %11d", res.obstacles[r][c]);
                printf("\n");
            }
        }
        for (int r = 0; r < res.n_obstacles; r++)
            for (int c = 0; c < 9; c++)
                cmp_u32("obs", r * 9 + c, dbg(8, r * 9 + c), (uint32_t)res.obstacles[r][c]);
        for (int r = 0; r < res.n_gaps; r++)
            for (int c = 0; c < 9; c++)
                cmp_u32("gap", r * 9 + c, dbg(9, r * 9 + c), (uint32_t)res.gaps[r][c]);

        // 4) 角色
        cmp_u32("player_valid", 0, dbg(10, 0, 3), (uint32_t)res.player_valid);
        for (int c = 0; c < 3; c++)
            cmp_u32("player", c, dbg(10, 0, c), (uint32_t)res.player[c]);
        cmp_u32("blob_x0", 0, dbg(11, 0, 0), (uint32_t)res.blob_x0);
        cmp_u32("blob_x1", 0, dbg(11, 0, 1), (uint32_t)res.blob_x1);
        cmp_u32("blob_y0", 0, dbg(11, 0, 2), (uint32_t)res.blob_y0);
        cmp_u32("blob_y1", 0, dbg(11, 0, 3), (uint32_t)res.blob_y1);
        cmp_u32("blob_area", 0, dbg(11, 0, 4), (uint32_t)res.blob_area);

        // 4b) 状态寄存器必须真的在动（否则 PS 侧无法判断帧好了没有）
        if (f == 0) {
            cmp_u32("status.frame_done", 0, (cfg_read(CFG_STATUS) >> 1) & 1u, 1u);
            cmp_u32("status.frames", 0, (cfg_read(CFG_STATUS) >> 8) & 0xFFFFu, 1u);
        }

        // 5) 最终几何帧字节流
        if (frame_rtl.size() != res.frame_bytes) {
            g_checks++; g_fail++;
            printf("  ✗ 帧长不符：RTL %zu 字节，C %zu 字节\n", frame_rtl.size(), res.frame_bytes);
        } else {
            cmp_bytes("frame", frame_rtl.data(), frame_g.data(), res.frame_bytes);
        }

        for (size_t i = 0; i < frame_rtl.size(); i++) {
            g_hash ^= frame_rtl[i];
            g_hash *= 1099511628211ull;
        }

        if (g_fail != fail_before) {
            g_fail_frames++;
            printf("帧 %d (%s): 不一致（累计 %ld 处）\n", f, name, g_fail - fail_before);
        } else if (g_verbose) {
            printf("帧 %d: 逐位一致 (%zu 字节, %d 障碍, %d 缺口, 角色 %d)\n",
                   f, frame_rtl.size(), res.n_obstacles, res.n_gaps, res.player_valid);
        }
    }

    printf("\n=== 协同仿真结果 ===\n");
    printf("比对项 %ld，不符 %ld，有差异的帧 %d\n", g_checks, g_fail, g_fail_frames);
    printf("几何帧指纹 %016llx\n", (unsigned long long)g_hash);
    delete top;
    return g_fail == 0 ? 0 : 1;
}
