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
//: RTL 固定的画面尺寸（vision_axi.v 的 WIDTH/HEIGHT 默认值）。激励尺寸必须与
//  它**相等**：RTL 既不会报错也不会截断，只会从第 HEIGHT 行起整帧对不上，
//  打出来满屏"不符"，看着像 RTL 回归。2026-10-14 被 /tmp/stim_real 里一个
//  640x480 的旧目录误伤过（262 项"不符"，全是几何不符造成的）。
static const int RTL_W = 640;
static const int RTL_H = 360;

// ---- 地址映射（**必须与 rtl/vision_axi.v 一致**，契约测试会核对）---------
enum {
    A_CFG_WALL_RB      = 0x000,   // 4*0
    A_CFG_WALL_MIN_G   = 0x008,   // 4*2
    A_CFG_BRIGHT_MIN   = 0x00C,   // 4*3
    A_CFG_BRIGHT_SPRD  = 0x010,   // 4*4
    A_CFG_MIN_GAP      = 0x040,   // 4*16
    A_CFG_P_MIN_AREA   = 0x044,   // 4*17
    A_CFG_P_MAX_AREA   = 0x048,   // 4*18
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
static int  g_frames_done = 0;   // 真正跑完比对的帧数（假通过防线用）
static int  g_stim_err    = 0;   // 激励读不出来 / 超限的次数
static bool g_verbose = false;
//: AW 之后隔几拍才给 W。
//
// ⚠ **默认 1，不是 0**：AXI 允许主控先给 AW、再给 W，而 `vision_axi` 原来把
//   三个 ready 全绑在一个 `idle_all` 上，**AW 一被接收 wready 就掉**，于是
//   W 永远进不来、bvalid 永不返回 —— 总线死锁。
//   原测试台恰好总是**同拍**给出 AW 和 W，所以 11490 项全绿也抓不到这个 bug，
//   最后是拿真硬件（2026-10-14）才暴露的。默认 1 拍之后，
//   "AW/W 不同拍"这条路每次跑测试都会被走到。
//   `--w-delay 0` 可以退回同拍的老行为（两个都要覆盖）。
static int g_w_delay = 1;

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

static bool axi_write(uint32_t addr, uint32_t data);

//: 带字节选通的写。**老 tb 把 s_wstrb 恒定为 0xF**，于是 AXI-Lite 的另一条
//  合法主控行为（只写部分字节）从没被走到过 —— 和 AW/W 同拍一样，属于
//  "把主控行为理想化"造成的结构性盲区（见下方 1.5 节的自检）。
static bool axi_write_s(uint32_t addr, uint32_t data, uint32_t strb)
{
    top->s_awaddr  = addr;  top->s_awvalid = 1;
    top->s_wdata   = data;  top->s_wstrb   = (uint8_t)strb;
    // g_w_delay == 0 时与老行为一致（同拍给 W）；>0 时等几拍再给 —— 后者才
    // 能覆盖"AW 已接收、W 还没来"那一段，正是原来死锁的地方。
    top->s_wvalid  = (g_w_delay == 0) ? 1 : 0;
    top->s_bready  = 1;
    bool done = false;
    int n = 0;
    for (int i = 0; i < 500 && !done; i++) {
        bool aw_fire = top->s_awvalid && top->s_awready;
        bool w_fire  = top->s_wvalid  && top->s_wready;
        bool b_fire  = top->s_bvalid  && top->s_bready;
        step(); n++;
        if (aw_fire) top->s_awvalid = 0;
        if (w_fire)  top->s_wvalid  = 0;
        if (b_fire)  done = true;
        if (!top->s_wvalid && g_w_delay > 0 && n > g_w_delay) top->s_wvalid = 1;
    }
    axi_idle();
    return done;
}

static bool axi_write(uint32_t addr, uint32_t data)
{
    return axi_write_s(addr, data, 0xF);
}

//: 模拟真实主控的**窄写**：地址**不对齐**（带上字节通道偏移），选通位随之落在
//  对应通道上。这正是 Zynq GP 口对 `*(volatile uint8_t*)p` / `*(volatile
//  uint16_t*)p` 的作法 —— AWADDR 就是那个字节地址，WSTRB 只有对应那几位。
//
//  ⚠ **这是第三个"测试台把主控理想化"的盲区**（前两个：AW/W 同拍、只发 0xF）。
//    `axi_write_s(A_SEQ, 0x0000BB00, 0x2)` 发的是"**字对齐**地址 + 选通掩码"，
//    那也是合法的 AXI 行为（稀疏写），RTL 处理得了；但真实主控对 0x0C5 做的
//    字节写发的是 `AWADDR=0x0C5, WSTRB=0x2` —— 地址不对齐。
//    旧 RTL 用 `case (aw_a)` 拿完整字节地址比字对齐常量，0x0C5 比不中就掉进
//    `default` 被静默丢弃。**协同仿真全绿、真硅上却挂**，就是这么来的。
//    两种合法行为都要覆盖，所以窄写和上面的稀疏写各测一遍。
static bool axi_write_narrow(uint32_t byte_addr, uint32_t data, int nbytes)
{
    uint32_t strb = (((1u << nbytes) - 1u) << (byte_addr & 3u)) & 0xFu;
    return axi_write_s(byte_addr, data, strb);
}

//: 读写**同一拍**发出：AWVALID+WVALID+ARVALID 一起给。
//  AXI **允许**主控这么做（从口可以节流，但**不能写错寄存器**）。tb 原来一次
//  只发一笔事务，所以从没走到这条路 —— 这是第四个"把主控理想化"的盲区。
static bool axi_write_read_same_cycle(uint32_t waddr, uint32_t wdata,
                                      uint32_t raddr, uint32_t *rout)
{
    top->s_awaddr  = waddr; top->s_awvalid = 1;
    top->s_wdata   = wdata; top->s_wstrb   = 0xF; top->s_wvalid = 1;
    top->s_araddr  = raddr; top->s_arvalid = 1;
    top->s_bready  = 1;     top->s_rready  = 1;
    bool wdone = false, rdone = false;
    for (int i = 0; i < 500 && !(wdone && rdone); i++) {
        bool aw_fire = top->s_awvalid && top->s_awready;
        bool w_fire  = top->s_wvalid  && top->s_wready;
        bool ar_fire = top->s_arvalid && top->s_arready;
        bool b_fire  = top->s_bvalid  && top->s_bready;
        if (top->s_rvalid && top->s_rready) { *rout = (uint32_t)top->s_rdata; rdone = true; }
        step();
        if (aw_fire) top->s_awvalid = 0;
        if (w_fire)  top->s_wvalid  = 0;
        if (ar_fire) top->s_arvalid = 0;
        if (b_fire)  wdone = true;
    }
    axi_idle();
    return wdone && rdone;
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
        fprintf(stderr, "用法: %s <stim_dir> <frames> [--verbose] [--bright N] [--w-delay N]\n", argv[0]);
        return 2;
    }
    std::string dir = argv[1];
    int frames = atoi(argv[2]);
    VrRefParams params;
    vr_ref_default_params(&params);
    uint32_t rtl_only_addr = 0xFFFFFFFFu, rtl_only_val = 0;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--verbose")) g_verbose = true;
        else if (!strcmp(argv[i], "--w-delay") && i + 1 < argc) g_w_delay = atoi(argv[++i]);
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

    // ---- 1.5) 字节选通自检（覆盖第二个"主控行为被理想化"的盲区）--------
    //
    // 配置寄存器块（0x000..0x07F）在 vision_top 里是**整字写**（cfg_addr 选中的
    // 寄存器被 cfg_wdata 整体覆盖），所以从口必须自己把 strb 未选中的字节并回去。
    // 原实现 `reg_wdata <= wstrb_merge(32'd0, w_d, w_s)` 会**把未选中的字节清零**
    // —— 对 0x044 写 1 字节会把其余 3 字节写成 0；而同一个从口上 SEQ/STAMP 却是
    // 就地 wstrb_merge，同一个从口两种字节语义。
    // 下面的 oracle 直接照 AXI-Lite 的字节使能定义独立算一遍，不复用 RTL 的函数。
    uint32_t v = 0;
    {
        auto merge = [](uint32_t old, uint32_t data, uint32_t strb) {
            uint32_t r = old;
            for (int b = 0; b < 4; b++)
                if (strb & (1u << b))
                    r = (r & ~(0xFFu << (8 * b))) | (data & (0xFFu << (8 * b)));
            return r;
        };
        const uint32_t LANE_DATA[6] = { 0x000000AAu, 0x0000BB00u, 0x00CC0000u,
                                        0xDD000000u, 0x0000EEEEu, 0xFFFF0000u };
        const uint32_t LANE_STRB[6] = { 0x1u, 0x2u, 0x4u, 0x8u, 0x3u, 0xCu };
        const char    *LANE_NAME[6] = { "lane0 字节", "lane1 字节", "lane2 字节",
                                        "lane3 字节", "低半字",   "高半字" };

        // ⚠ SEQ/STAMP0/STAMP1 **都会进帧头**（SEQ 在字节 8..11，stamp 在 24..31），
        //   所以压完必须复原 —— 否则每帧会有 8 字节对不上，报一堆莫名其妙的"不符"。
        //   （第一版就漏了 STAMP0，被帧比对当场抓出 256/2560 项不符。）
        uint32_t save_area = 0, save_seq = 0, save_st0 = 0, save_st1 = 0;
        CHECK(axi_read(A_CFG_P_MIN_AREA, &save_area), "选通自检：预读 P_MIN_AREA 超时");
        CHECK(axi_read(A_SEQ, &save_seq), "选通自检：预读 SEQ 超时");
        CHECK(axi_read(A_STAMP0, &save_st0), "选通自检：预读 STAMP0 超时");
        CHECK(axi_read(A_STAMP1, &save_st1), "选通自检：预读 STAMP1 超时");

        // ① 32 位配置寄存器：逐字节 / 逐半字，四种 strb 掩码全覆盖
        uint32_t acc = 0x11223344u;
        CHECK(axi_write(A_CFG_P_MIN_AREA, acc), "选通自检：整字写超时");
        CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == acc, "选通自检：整字写读回不符");
        for (int i = 0; i < 6; i++) {
            char msg[80];
            snprintf(msg, sizeof(msg), "选通自检：写 %s 超时", LANE_NAME[i]);
            CHECK(axi_write_s(A_CFG_P_MIN_AREA, LANE_DATA[i], LANE_STRB[i]), msg);
            acc = merge(acc, LANE_DATA[i], LANE_STRB[i]);
            snprintf(msg, sizeof(msg), "选通自检：写 %s 后读回不符", LANE_NAME[i]);
            CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == acc, msg);
        }

        // ② SEQ(0x0C4) 走的是另一条就地合并路径，同样要过
        acc = 0x11223344u;
        CHECK(axi_write(A_SEQ, acc), "选通自检：SEQ 整字写超时");
        CHECK(axi_write_s(A_SEQ, 0x0000BB00u, 0x2u), "选通自检：SEQ lane1 超时");
        acc = merge(acc, 0x0000BB00u, 0x2u);
        CHECK(axi_read(A_SEQ, &v) && v == acc, "选通自检：SEQ lane1 读回不符");

        // ③ 窄写（**地址不对齐**）：真实主控对 *(volatile uint8_t/uint16_t*) 的作法。
        //    旧 RTL 只认 lane0（case(aw_a) 拿字节地址比对齐常量），这一步就是判据。
        const uint32_t OFF[2] = { A_CFG_P_MIN_AREA, A_SEQ };
        const char    *NM[2]  = { "P_MIN_AREA(配置块)", "SEQ(就地合并)" };
        for (int k = 0; k < 2; k++) {
            uint32_t off = OFF[k];
            uint32_t acc2 = 0x11223344u;
            CHECK(axi_write(off, acc2), "窄写基准整字写超时");
            for (int lane = 0; lane < 4; lane++) {
                uint32_t d = (uint32_t)(0xB0u + lane) << (8 * lane);
                CHECK(axi_write_narrow(off + (uint32_t)lane, d, 1), "窄写(字节)超时");
                acc2 = merge(acc2, d, 1u << lane);
                char nm[64];
                snprintf(nm, sizeof(nm), "窄写 %s +%d 字节读回不符", NM[k], lane);
                CHECK(axi_read(off, &v) && v == acc2, nm);
            }
            {   // 低半字（地址对齐到 +0，WSTRB=0x3）
                uint32_t d = 0x0000CAFEu;
                CHECK(axi_write_narrow(off, d, 2), "窄写(低半字)超时");
                acc2 = merge(acc2, d, 0x3u);
                char nm[64]; snprintf(nm, sizeof(nm), "窄写 %s 低半字读回不符", NM[k]);
                CHECK(axi_read(off, &v) && v == acc2, nm);
            }
            {   // 高半字（地址 +2，WSTRB=0xC）
                uint32_t d = 0xFEED0000u;
                CHECK(axi_write_narrow(off + 2, d, 2), "窄写(高半字)超时");
                acc2 = merge(acc2, d, 0xCu);
                char nm[64]; snprintf(nm, sizeof(nm), "窄写 %s 高半字读回不符", NM[k]);
                CHECK(axi_read(off, &v) && v == acc2, nm);
            }
            uint32_t fin = 0; axi_read(off, &fin);
            printf("    %-20s 窄写末值 0x%08X（期望 0x%08X）%s\n", NM[k], fin, acc2,
                   fin == acc2 ? " ✓" : " ✗");
        }

        // ④ 窄**读**：地址不对齐时也必须命中同一个字。
        //    旧 RTL 的读侧 `case (ar_a)` 同样拿字节地址比 —— 读 0x0C5 会掉进
        //    default 返回 0，而不是 SEQ 的值。
        {
            uint32_t seqv = 0x5A5A1234u;
            CHECK(axi_write(A_SEQ, seqv), "窄读基准写超时");
            for (int lane = 0; lane < 4; lane++) {
                char nm[64];
                snprintf(nm, sizeof(nm), "窄读 SEQ +%d 字节", lane);
                CHECK(axi_read(A_SEQ + (uint32_t)lane, &v) && v == seqv, nm);
            }
            uint32_t st0 = 0x13572468u;
            CHECK(axi_write(A_STAMP0, st0), "窄读基准写 STAMP0 超时");
            CHECK(axi_read(A_STAMP0 + 3, &v) && v == st0, "窄读 STAMP0 +3 字节");
        }

        // ⑤ 读写**同拍**：写的那笔必须落到**写地址**上，读的那笔必须读到**读地址**。
        //    旧 RTL 在 reg_we=1 那一拍仍让读地址抢占 cfg_addr，于是：
        //      · 写 SEQ + 读配置寄存器 → 写落进了被读的那个配置寄存器；
        //      · 写配置寄存器 + 读 CTRL → cfg_addr=31(CFG_STATUS) → 写被丢弃。
        {
            uint32_t rv = 0;
            // (a) 写配置块寄存器 A、同时读配置块寄存器 B(≠A)
            //     ---- 最有害的一种：写会**静默落到被读的那个寄存器**上。
            //     （写非配置块寄存器（如 SEQ）不会拉高 cfg_we，那种组合本来就
            //       无害 —— 这一点是反向验证时确认的。）
            CHECK(axi_write(A_CFG_P_MIN_AREA, 0x11111111u), "同拍: 置 P_MIN_AREA 超时");
            CHECK(axi_write(A_CFG_P_MAX_AREA, 0x99999999u), "同拍: 置 P_MAX_AREA 超时");
            CHECK(axi_write_read_same_cycle(A_CFG_P_MIN_AREA, 0x22222222u,
                                            A_CFG_P_MAX_AREA, &rv),
                  "同拍: 写 P_MIN_AREA + 读 P_MAX_AREA 超时");
            CHECK(rv == 0x99999999u, "同拍: 读回的不是 P_MAX_AREA 的原值");
            CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == 0x22222222u,
                  "同拍: 写 P_MIN_AREA 没生效");
            CHECK(axi_read(A_CFG_P_MAX_AREA, &v) && v == 0x99999999u,
                  "同拍: 写 P_MIN_AREA 时**落进了被读的那个配置寄存器**");

            // (b) 写 P_MIN_AREA、读 P_MIN_AREA 自己
            CHECK(axi_write(A_CFG_P_MIN_AREA, 0x33333333u), "同拍: 重置 P_MIN_AREA 超时");
            CHECK(axi_write_read_same_cycle(A_CFG_P_MIN_AREA, 0x44444444u,
                                            A_CFG_P_MIN_AREA, &rv),
                  "同拍: 写读同一寄存器超时");
            CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == 0x44444444u,
                  "同拍: 写配置寄存器没生效（被读地址抢了 cfg_addr）");

            // (c) 写配置寄存器、读 CTRL（读侧是借 CFG_STATUS 那条路）
            CHECK(axi_write_read_same_cycle(A_CFG_P_MIN_AREA, 0x55555555u,
                                            A_CTRL, &rv),
                  "同拍: 写配置寄存器 + 读 CTRL 超时");
            CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == 0x55555555u,
                  "同拍: 读 CTRL 时写被丢弃（cfg_addr 被借去 CFG_STATUS）");

            // (d) 写 STAMP0、读 SEQ（两者都不在配置块里，走向量寄存器那条路）
            CHECK(axi_write_read_same_cycle(A_STAMP0, 0x0BADF00Du, A_SEQ, &rv),
                  "同拍: 写 STAMP0 + 读 SEQ 超时");
            CHECK(axi_read(A_STAMP0, &v) && v == 0x0BADF00Du, "同拍: 写 STAMP0 没生效");
        }

        // ③ 复原，免得影响后面的参数写与帧比对
        CHECK(axi_write(A_CFG_P_MIN_AREA, save_area), "选通自检：恢复 P_MIN_AREA 超时");
        CHECK(axi_write(A_SEQ, save_seq), "选通自检：恢复 SEQ 超时");
        CHECK(axi_write(A_STAMP0, save_st0), "选通自检：恢复 STAMP0 超时");
        CHECK(axi_write(A_STAMP1, save_st1), "选通自检：恢复 STAMP1 超时");
        CHECK(axi_read(A_CFG_P_MIN_AREA, &v) && v == save_area, "选通自检：恢复 P_MIN_AREA 不符");
        CHECK(axi_read(A_SEQ, &v) && v == save_seq, "选通自检：恢复 SEQ 不符");
        CHECK(axi_read(A_STAMP0, &v) && v == save_st0, "选通自检：恢复 STAMP0 不符");
        CHECK(axi_read(A_STAMP1, &v) && v == save_st1, "选通自检：恢复 STAMP1 不符");
    }

    // ---- 1) 配置寄存器：写进去再读回来 --------------------------------
    for (int f = 0; f < frames; f++) {
        char name[64];
        snprintf(name, sizeof(name), "%06d.ppm", f);
        int w = 0, h = 0;
        if (!load_ppm(dir + "/" + name, &w, &h, rgb)) {
            printf("激励读取失败：%s\n", name);
            g_stim_err++;
            break;
        }
        if (w != RTL_W || h != RTL_H) {
            printf("!! 激励 %s 是 %dx%d，RTL 固定为 %dx%d —— 几何不符，判为失败\n",
                   name, w, h, RTL_W, RTL_H);
            g_stim_err++;
            break;
        }
        g_frames_done++;

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

    // ---- 假通过防线（与 tb_vision.cpp 同一条规矩，理由见那边）----------------
    int rc = (g_fail == 0) ? 0 : 1;
    if (g_frames_done != frames) {
        printf("!! 只比对了 %d / %d 帧（激励出错 %d 次）—— 判为失败\n",
               g_frames_done, frames, g_stim_err);
        rc = 1;
    }
    if (g_checks == 0) {
        printf("!! 一次比对都没发生 —— 判为失败\n");
        rc = 1;
    }

    delete top;
    return rc;
}
