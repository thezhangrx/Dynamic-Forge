/* ===========================================================================
 * axi_soak.c —— 在板子上**加压复验 AXI 写通路**，专打两个曾经的盲区
 *
 * ⚠⚠ 这个工具会**写** 0x43C00000。在 AXI 写死锁修复之前的位流上，
 *     第一次写就会把 CPU 挂死（只能彻底断电恢复）。所以：
 *         **必须先烧上含修复的 bd_vision 位流，再运行本工具。**
 *     只读的检查请用 read_frame（它带 --watch 只读模式）。
 *
 * ---------------------------------------------------------------------------
 * 为什么需要它（协同仿真已经全绿了，为什么还要上板打）
 * ---------------------------------------------------------------------------
 * 真硅上暴露的两个 bug 都是"**测试台把主控行为理想化**"造成的结构性盲区：
 *
 *   ① AW/W 不同拍 → 写死锁。
 *      tb_axi 原来总是同拍给出 AW 和 W，11490 项全绿也没抓到。
 *      已给 tb 加 --w-delay（默认 1）覆盖。
 *   ② 配置寄存器块的字节选通 → 未选通字节被静默清零。
 *      tb_axi 原来把 s_wstrb 恒定为 0xF，同样从没走到过。
 *      已加 axi_write_s() + 逐字节/半字节自检覆盖。
 *
 * 仿真能覆盖的是"时序组合"，但**真实主控的 WSTRB 到底长什么样**只有 CPU
 * 自己知道（Zynq GP 口对 8/16 位窄访问会产生什么样的 WSTRB，是我们对主控
 * 的假设）。所以这一条必须上板确认：本工具的 Phase D 用真正的 8/16 位
 * volatile 访问去打，oracle 照 AXI-Lite 的字节使能定义在 C 里独立算。
 *
 * ---------------------------------------------------------------------------
 * 阶段（顺序是刻意的：先只读、再单次写、再压测、最后端到端）
 * ---------------------------------------------------------------------------
 *   A  快照 21 个可写配置寄存器 + SEQ/STAMP0/STAMP1（**纯读**，顺带证明地址被译码）
 *   B  金丝雀：**单次**写一个配置寄存器并读回（要挂就挂在这里，失败点最清楚）
 *   C  字级压测：R 轮随机地址/随机数据 写→读回 比对（数据按寄存器位宽掩码）
 *   D  字节/半字选通压测：8/16 位访问打 P_MIN_AREA 与 SEQ 的各个字节通道
 *      —— 这一项就是"WSTRB 修复在位流里到底有没有生效"的判据
 *   E  复原 A 的快照并逐字复核（保证 F 的比对不受污染）
 *   F  端到端：写 CTRL start → 轮询帧计数/busy → 读 flen → CRC32 → 与
 *      Vvision_selftest 的 8 个期望值对账，重复 N 帧
 *   G  存活确认 + 指纹（能打印到这行，就说明整轮写没有挂死）
 *
 * 编译（在板子上直接编）：
 *     gcc -O2 -o axi_soak axi_soak.c
 * 运行：
 *     ./axi_soak                 # 2000 轮字级压测 + 8 帧端到端
 *     ./axi_soak 20000           # 加码到 2 万轮
 *     ./axi_soak 2000 --no-frame # 只压写通路，不启动帧
 *     ./axi_soak 2000 --seed 7 --frames 32
 *
 * 寄存器映射见 core/fpga/rtl/vision_axi.v 与 vision_top.v 的 CFG_* 表。
 * =========================================================================== */
/* 同 read_frame.c：sigaction/sigsetjmp 是 POSIX 的，-std=c11 下必须显式
 * 声明特性宏，否则 <signal.h> 不暴露它们。写在源文件里，不依赖编译选项。 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <setjmp.h>
#include <sys/mman.h>

#define BASE     0x43C00000UL
#define MAP_SIZE 0x1000UL
#define OFF_SEQ    0x0C4
#define OFF_STAMP0 0x0C8
#define OFF_STAMP1 0x0CC
#define OFF_CTRL   0x0C0
#define OFF_FLEN   0x0D0
#define OFF_FRAME  0x100
#define FRAME_MAX  2048

#define EXP_LEN 184
/* 同 read_frame.c：这是帧**内嵌**的 CRC 字段（最后 4 字节小端），不是对整帧
 * 再算的 CRC32。见 cosim/tb_tpg.cpp 里提取期望常数的写法。 */
static const uint32_t EXP_CRC[8] = {
    0xA9670ED9u, 0x621801BEu, 0x1EEBDE91u, 0x0BC5C6CAu,
    0x76D2BFA9u, 0xC71A351Au, 0xA5440B40u, 0xAEC7F26Eu,
};

/* 可写的配置寄存器：偏移 + 读回时有效的位宽。
 * 与 vision_top.v 的 CFG_* localparam 表一一对应（CFG_STATUS=6'd31 是只读的，
 * 不在这里）。索引 21..30 未映射，写进去被忽略，所以**不能**拿它们做读回比对。 */
static const struct { uint32_t off; int bits; const char *name; } CFG[] = {
    { 0x000,  8, "WALL_RB"     }, { 0x004,  8, "WALL_MIN_R"  },
    { 0x008,  8, "WALL_MIN_G"  }, { 0x00C,  8, "BRIGHT_MIN"  },
    { 0x010,  8, "BRIGHT_SPRD" }, { 0x014,  8, "GREEN_GM"    },
    { 0x018,  8, "GREEN_MIN"   }, { 0x01C, 16, "BAND_PCT"    },
    { 0x020, 16, "BAND_JOIN"   }, { 0x024, 16, "BAND_MINR"   },
    { 0x028, 16, "FIELD_COL"   }, { 0x02C, 16, "WALL_PCT"    },
    { 0x030, 16, "WALL_COLMIN" }, { 0x034, 16, "EDGE_MIN"    },
    { 0x038, 16, "EDGE_COLS"   }, { 0x03C, 16, "MIN_SEG"     },
    { 0x040, 16, "MIN_GAP"     }, { 0x044, 32, "P_MIN_AREA"  },
    { 0x048, 32, "P_MAX_AREA"  }, { 0x04C, 32, "P_FILL"      },
    { 0x050, 32, "P_ASPECT"    },
};
#define NCFG ((int)(sizeof(CFG) / sizeof(CFG[0])))

/* 复位默认值，顺序与 CFG[] 一一对应（= vision_top.v 复位块里的值，
 * 也 = C 参考模型的 VR_DEFAULT_REF_PARAMS）。
 *
 * 为什么工具里要带这张表：Phase F 的 8 个期望 CRC 是**在默认配置下**由
 * Vvision_selftest 跑出来的。如果配置不是默认值（比如上一次运行崩在复原之前），
 * 帧内容本来就该不一样 —— 那时报"CRC 不符"是**误导性的失败**。
 * 所以先用这张表判一下"配置是不是默认值"，不是的话明确说清楚。
 *
 * ⚠ 这是同一份知识的第三份拷贝，靠 core/board/tests/test_tools_contract.py
 *   直接解析 vision_top.v 的复位块来守住（漂了会报错）。 */
static const uint32_t DEF[NCFG] = {
    40, 110, 60, 195, 45, 10, 90,            /* 8 位那批 */
    5, 4, 5, 5, 25, 2, 12, 1, 8, 8,          /* 16 位那批 */
    60, 1500, 45, 160,                       /* 32 位那批 */
};

static volatile uint8_t *M;
static uint8_t *snap;                 /* A 阶段的快照 */
static long g_checks = 0, g_fail = 0;
static uint64_t g_fp = 1469598103934665603ULL;   /* FNV-1a，让每轮可复现 */

static uint32_t mask_of(int bits) { return (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u); }

static void w32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(M + off) = v; }
static uint32_t r32(uint32_t off)          { return *(volatile uint32_t *)(M + off); }

static void fp_mix(uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t v[3] = { a, b, c };
    for (int i = 0; i < 3; i++) {
        g_fp ^= v[i];
        g_fp *= 1099511628211ULL;
    }
}

static void expect(const char *what, uint32_t got, uint32_t want, uint32_t off)
{
    g_checks++;
    if (got == want) return;
    g_fail++;
    if (g_fail <= 25)
        printf("  ✗ %s @0x%03X  AXI=0x%08X  期望=0x%08X\n", what, off, got, want);
}

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
    }
    return ~c;
}

/* AXI-Lite 字节使能的**独立** oracle：按定义算，不复用 RTL 的 wstrb_merge。 */
static uint32_t merge_lanes(uint32_t old, uint32_t data, uint32_t strb)
{
    uint32_t r = old;
    for (int b = 0; b < 4; b++)
        if (strb & (1u << b))
            r = (r & ~(0xFFu << (8 * b))) | (data & (0xFFu << (8 * b)));
    return r;
}

static sigjmp_buf g_jb;
static void on_bus(int sig) { (void)sig; siglongjmp(g_jb, 1); }

/* POSIX.1-2008 摘掉了 usleep，用 nanosleep */
static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* 把 setjmp 关在小函数里，免得 -Wclobbered 警告 main 里那堆局部变量
 * （rounds/nframes/seed/...）。装好信号处理，然后做第一次访问当探针。 */
static int probe_mapped(volatile uint8_t *base)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_bus;
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
    if (sigsetjmp(g_jb, 1)) return -1;
    (void)*(volatile uint32_t *)(base + OFF_CTRL);
    return 0;
}

/* xorshift32：确定性，让 --seed 相同的两次跑出完全一样的激励与指纹 */
static uint32_t g_rng;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return g_rng;
}

int main(int argc, char **argv)
{
    long rounds = 2000;
    int nframes = 8, do_frame = 1, seed = 20261014;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--no-frame")) do_frame = 0;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) nframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed")   && i + 1 < argc) seed = atoi(argv[++i]);
        else if (argv[i][0] != '-') rounds = atol(argv[i]);
    }
    if (rounds < 1) rounds = 1;
    if (nframes < 0) nframes = 0;
    g_rng = (uint32_t)seed ? (uint32_t)seed : 1u;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    M = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE);
    if (M == MAP_FAILED) { perror("mmap"); return 1; }

    if (probe_mapped(M) != 0) {
        fprintf(stderr, "！！访问 0x%08lX 触发总线错误 —— 位流没加载或不是 bd_vision\n",
                (unsigned long)BASE);
        return 3;
    }

    snap = calloc(NCFG, 4);
    printf("=== AXI 写通路加压：轮数 %ld，帧数 %s%d，seed %d ===\n",
           rounds, do_frame ? "" : "(跳过) ", do_frame ? nframes : 0, seed);

    /* ---------------- A 快照（纯读）---------------- */
    printf("\n[A] 快照可写配置寄存器 %d 个（纯读）\n", NCFG);
    for (int i = 0; i < NCFG; i++) {
        uint32_t v = r32(CFG[i].off);
        memcpy(snap + 4 * i, &v, 4);
    }
    printf("    ctrl=0x%08X flen=%u seq=0x%08X\n",
           r32(OFF_CTRL), r32(OFF_FLEN), r32(OFF_SEQ));
    /* Phase F 的期望 CRC 前提是"配置处于复位默认值"。刚加载完位流时必然成立，
     * 但如果这一次不是（上一次跑崩在复原之前等），报"CRC 不符"就是误导。
     * 所以这里明确判一次，Phase F 再据此给出提示。 */
    int at_default = 1, ndiff = 0;
    for (int i = 0; i < NCFG; i++) {
        uint32_t v; memcpy(&v, snap + 4 * i, 4);
        v &= mask_of(CFG[i].bits);
        if (v != DEF[i]) {
            at_default = 0;
            if (++ndiff <= 6)
                printf("    ⚠ %s = %u，复位默认值是 %u\n", CFG[i].name, v, DEF[i]);
        }
    }
    printf("    配置%s复位默认值（不符 %d 项）%s\n", at_default ? "是" : "**不是**", ndiff,
           at_default ? " —— Phase F 的期望 CRC 适用" : "");

    /* ---------------- B 金丝雀：单次写 ---------------- */
    printf("\n[B] 金丝雀：**单次**写 P_MIN_AREA 并读回（要挂就挂在这一步）\n");
    {
        uint32_t want = 0x00C0FFEEu;
        w32(0x044, want);
        uint32_t got = r32(0x044);
        expect("金丝雀写读回", got, want, 0x044);
        printf("    写 0x%08X → 读回 0x%08X  %s\n", want, got, got == want ? "✓" : "✗");
    }

    /* ---------------- C 字级压测 ---------------- */
    printf("\n[C] 字级压测 %ld 轮：随机地址/随机数据 写→读回（按位宽掩码）\n", rounds);
    long c_bad = 0;
    for (long r = 0; r < rounds; r++) {
        int i = (int)(rnd() % (uint32_t)NCFG);
        uint32_t m = mask_of(CFG[i].bits);
        uint32_t data = rnd() & m;
        uint32_t before = r32(CFG[i].off) & m;
        w32(CFG[i].off, data);
        uint32_t got = r32(CFG[i].off) & m;
        g_checks++;
        fp_mix(CFG[i].off, data, got);
        if (got != data) {
            g_fail++; c_bad++;
            if (c_bad <= 10)
                printf("  ✗ 第 %ld 轮 %s @0x%03X 写 0x%08X 读回 0x%08X（写前 0x%08X）\n",
                       r, CFG[i].name, CFG[i].off, data, got, before);
        }
    }
    printf("    完成，失配 %ld\n", c_bad);

    /* ---------------- D 字节 / 半字选通压测 ---------------- */
    printf("\n[D] 字节/半字选通：用真正的 8/16 位访问打各字节通道\n");
    printf("    （这是\"WSTRB 修复有没有进这个位流\"的判据）\n");
    {
        /* SEQ(0x0C4) 与 STAMP0/1 是就地 wstrb_merge；P_MIN_AREA(0x044) 走
         * 配置寄存器块的整字写路径（靠 cfg_rdata 读旧值再合并）。两条都要过。 */
        const uint32_t OFF[2] = { 0x044, OFF_SEQ };
        const char *NM[2] = { "P_MIN_AREA(配置块)", "SEQ(就地合并)" };
        for (int k = 0; k < 2; k++) {
            uint32_t off = OFF[k];
            uint32_t base = 0x11223344u, acc = base;
            w32(off, base);
            expect("选通基准整字写", r32(off), base, off);

            /* 8 位：逐通道写，每次只选通 1 字节 */
            for (int lane = 0; lane < 4; lane++) {
                uint32_t d = (uint32_t)(0xA0u + lane) << (8 * lane);
                uint32_t strb = 1u << lane;
                *(volatile uint8_t *)(M + off + lane) = (uint8_t)(0xA0u + lane);
                acc = merge_lanes(acc, d, strb);
                char nm[64]; snprintf(nm, sizeof(nm), "%s lane%d 字节", NM[k], lane);
                expect(nm, r32(off), acc, off);
            }
            /* 16 位：低半字、高半字 */
            {
                uint32_t d = 0x0000BEEFu, strb = 0x3u;
                *(volatile uint16_t *)(M + off + 0) = (uint16_t)d;
                acc = merge_lanes(acc, d, strb);
                char nm[64]; snprintf(nm, sizeof(nm), "%s 低半字", NM[k]);
                expect(nm, r32(off), acc, off);
            }
            {
                uint32_t d = 0xDEAD0000u, strb = 0xCu;
                *(volatile uint16_t *)(M + off + 2) = (uint16_t)(d >> 16);
                acc = merge_lanes(acc, d, strb);
                char nm[64]; snprintf(nm, sizeof(nm), "%s 高半字", NM[k]);
                expect(nm, r32(off), acc, off);
            }
            printf("    %-20s 末值 0x%08X（期望 0x%08X）%s\n", NM[k], r32(off), acc,
                   r32(off) == acc ? " ✓" : " ✗");
        }
        /* STAMP0/STAMP1 也各打一个字节 */
        w32(OFF_STAMP0, 0x11223344u);
        *(volatile uint8_t *)(M + OFF_STAMP0 + 1) = 0xBB;
        expect("STAMP0 lane1 字节", r32(OFF_STAMP0), 0x1122BB44u, OFF_STAMP0);
        w32(OFF_STAMP1, 0x55667788u);
        *(volatile uint8_t *)(M + OFF_STAMP1 + 3) = 0xAA;
        expect("STAMP1 lane3 字节", r32(OFF_STAMP1), 0xAA667788u, OFF_STAMP1);

        // ⚠ 窄**读**：地址不对齐时也必须命中同一个字。
        //   2026-10-15 上板暴露：RTL 的读侧也是 `case (ar_a)` 拿**字节地址**比
        //   字对齐常量，所以读 0x0C5 会掉进 default 返回 0，而不是 SEQ 的值。
        //   写侧修了之后读侧同样要修 —— 这里就是那一半的判据。
        //   （读本身不改状态，但它证明了"窄写测试里的读回"走的是同一条路。）
        {
            uint32_t seqv = 0x5A5A1234u, got = 0;
            w32(OFF_SEQ, seqv);
            for (int lane = 0; lane < 4; lane++) {
                got = r32(OFF_SEQ + (uint32_t)lane);
                char nm[64];
                snprintf(nm, sizeof(nm), "窄读 SEQ +%d 字节", lane);
                expect(nm, got, seqv, OFF_SEQ);
            }
            uint32_t st0 = 0x13572468u;
            w32(OFF_STAMP0, st0);
            expect("窄读 STAMP0 +3 字节", r32(OFF_STAMP0 + 3), st0, OFF_STAMP0);
        }
    }

    /* ---------------- E 复原 ---------------- */
    printf("\n[E] 复原快照并逐字复核（不然后面帧的比对会被污染）\n");
    for (int i = 0; i < NCFG; i++) {
        uint32_t v; memcpy(&v, snap + 4 * i, 4);
        w32(CFG[i].off, v);
    }
    for (int i = 0; i < NCFG; i++) {
        uint32_t v; memcpy(&v, snap + 4 * i, 4);
        uint32_t m = mask_of(CFG[i].bits);
        char nm[64]; snprintf(nm, sizeof(nm), "复原 %s", CFG[i].name);
        expect(nm, r32(CFG[i].off) & m, v & m, CFG[i].off);
    }
    w32(OFF_SEQ, 0); w32(OFF_STAMP0, 0); w32(OFF_STAMP1, 0);
    printf("    已复原 %d 个寄存器\n", NCFG);

    /* ---------------- F 端到端帧 ---------------- */
    if (do_frame && nframes > 0) {
        printf("\n[F] 端到端 %d 帧：写 CTRL start → 等帧 → CRC 对账\n", nframes);
        if (!at_default)
            printf("  ⚠ 配置当前**不是**复位默认值 —— 下面报“不符”是预期的，\n"
                   "    因为 8 个期望 CRC 是在默认配置下跑出来的\n");
        int hit[8] = {0}, ok = 0, bad = 0;
        uint8_t buf[FRAME_MAX];
        for (int f = 0; f < nframes; f++) {
            uint32_t cnt0 = (r32(OFF_CTRL) >> 8) & 0xFFFFu;
            w32(OFF_CTRL, 1u);                       /* bit0 = start_pulse */
            int waited = 0, adv = 0;
            while (waited < 500) {
                uint32_t st = r32(OFF_CTRL);
                if (((st >> 8) & 0xFFFFu) != cnt0) adv = 1;
                if (adv && !(st & 1u)) break;
                sleep_ms(1); waited++;
            }
            if (!adv) { printf("  帧 %d: 500ms 内帧计数未动 —— start 没生效\n", f); bad++; continue; }
            uint32_t len = r32(OFF_FLEN);
            if (len == 0 || len > FRAME_MAX) { printf("  帧 %d: flen=%u 异常\n", f, len); bad++; continue; }
            memcpy(buf, (const void *)(M + OFF_FRAME), len);
            uint32_t emb = (uint32_t)buf[len - 4]
                         | ((uint32_t)buf[len - 3] << 8)
                         | ((uint32_t)buf[len - 2] << 16)
                         | ((uint32_t)buf[len - 1] << 24);
            uint32_t self_crc = crc32(buf, (size_t)(len - 4));
            int pat = -1;
            for (int i = 0; i < 8; i++)
                if (len == EXP_LEN && emb == EXP_CRC[i]) { pat = i; break; }
            if (pat >= 0) { ok++; hit[pat]++; } else { bad++; }
            printf("  帧 %2d: len=%4u 内嵌crc=0x%08X 自洽%s %s\n", f, len, emb,
                   (self_crc == emb) ? "✓" : "✗", pat >= 0 ? "✓" : "✗ **不符**");
        }
        printf("  端到端：一致 %d/%d，不符 %d；图案分布", ok, nframes, bad);
        for (int i = 0; i < 8; i++) printf(" [%d]=%d", i, hit[i]);
        printf("\n");
        if (bad) { g_fail++; g_checks++; }
        else     { g_checks++; }
    }

    /* ---------------- G 存活 + 汇总 ---------------- */
    printf("\n=== 汇总 ===\n");
    printf("比对项 %ld，不符 %ld\n", g_checks, g_fail);
    printf("激励指纹 0x%016llX（同 seed/轮数应完全一致）\n", (unsigned long long)g_fp);
    printf("存活确认：ctrl=0x%08X（能打印到这行说明整轮写没有挂死 CPU）\n", r32(OFF_CTRL));
    if (g_checks == 0) { printf("！！一次比对都没发生\n"); return 1; }
    return g_fail ? 2 : 0;
}
