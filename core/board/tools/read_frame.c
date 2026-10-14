/* ===========================================================================
 * read_frame.c —— 在**板子上**读回几何帧，并和协同仿真的期望值逐位对账
 *
 * 这是"真硅上整条链对不对"的端到端验证：
 *
 *     tpg_source（片上图案）→ vision_axi（视觉流水）→ AXI 窗口 → 本程序 → 对账
 *
 * 期望值从哪来：`core/fpga/cosim/tb_tpg.cpp` 里 Vvision_selftest 逐场景跑出来的
 * 帧长与 CRC32（见 core/fpga/docs/板上验证.md §2.2）。块设计里的图案源就是同一个
 * `vision_tpg`、相同的默认配置，所以这两组常数**应当完全相同**。
 *
 * 编译（在板子上直接编）：
 *     gcc -O2 -o read_frame read_frame.c
 * 运行：
 *     ./read_frame              # 启动 1 帧并对账
 *     ./read_frame 40           # 连续 40 帧，统计命中率
 *     ./read_frame 1 --watch    # **只读不写**：不启动，光等自由运行的帧
 *
 * ---------------------------------------------------------------------------
 * ⚠ 2026-10-14 修掉的一个真 bug（这个工具原来根本跑不通）
 * ---------------------------------------------------------------------------
 *   原版只"等 frame_done 翻转"，**从不写 CTRL 的 start 位**。但
 *   `rtl_bd/tpg_source.v` 只负责按自己的节奏送像素，**不启动几何帧**；
 *   `vision_top` 的 frame_start 唯一来源就是 AXI 的 `ADDR_CTRL` bit0。
 *   也就是说没人启动过帧，frame_done 永远不会翻转 —— 原版必然等满超时、
 *   读到 flen=0，然后报一句含糊的"长度异常"。
 *
 *   正确时序（vision_axi.v 的寄存器注释里也写着）：
 *       写 CTRL bit0 → 轮询 busy 落 → 读 frame_len → 读窗口
 *   `vision_top` 在 S_RECV 里等 `din_sof` 对齐，所以在任意时刻启动都安全，
 *   不必和图案源对相位。
 *
 *   `--watch` 保留原来的"只等不启动"行为：它本身是个有用的**判据** ——
 *   如果 `--watch` 能拿到帧，说明有别的什么在自动启动帧；拿不到才是正常的。
 *   这个模式**完全不写寄存器**，因此在任何位流上都不会挂死 CPU。
 *
 * 寄存器映射见 core/fpga/rtl/vision_axi.v：
 *     基址 + 0x0C0  控制/状态（bit0 busy, bit1 frame_done, bit23:8 帧计数）
 *     基址 + 0x0D0  上一帧字节数（只读）
 *     基址 + 0x100  几何帧窗口
 * =========================================================================== */
/* 在源文件里声明，而不是依赖编译选项：sigaction/sigsetjmp 是 POSIX 的，
 * 用 `-std=c11`（严格 ISO C）时不声明这个宏它们会被 <signal.h> 藏起来，
 * 报 "unknown type name 'sigjmp_buf'" / "storage size of 'sa' isn't known"。
 * 这样写，-std=c11 和 -std=gnu11 都能编。 */
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

#define BASE        0x43C00000UL
#define MAP_SIZE    0x1000UL
#define OFF_CTRL    0x0C0
#define OFF_FLEN    0x0D0
#define OFF_FRAME   0x100
#define FRAME_MAX   2048

/* 一帧最多等多久（毫秒）。实测一帧约 52 万拍 @50MHz ≈ 11 ms，
 * 这里给足余量，超时说明帧根本没启动。 */
#define FRAME_TIMEOUT_MS 500

/* 期望值（来自协同仿真的 Vvision_selftest，图案 0..7） */
static const uint32_t EXP_CRC[8] = {
    0xA9670ED9u, 0x621801BEu, 0x1EEBDE91u, 0x0BC5C6CAu,
    0x76D2BFA9u, 0xC71A351Au, 0xA5440B40u, 0xAEC7F26Eu,
};
#define EXP_LEN 184

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

/* PL 没译码时 /dev/mem 的访问会吃 SIGBUS（PS 侧 SLCR 那种被内核挡掉的会吃
 * SIGSEGV）。不接这个的话程序直接崩，看不出是"位流没加载"。
 *
 * 把 setjmp 关在这个小函数里：放在 main 里的话，-Wclobbered 会警告 main 里
 * 每一个可能被 longjmp 冲掉的局部变量（watch/want/...）。 */
static sigjmp_buf g_jb;
static void on_bus(int sig) { (void)sig; siglongjmp(g_jb, 1); }

/* POSIX.1-2008 把 usleep 摘掉了（-std=c11 + _POSIX_C_SOURCE=200809L 下会
 * 报隐式声明），用 nanosleep。 */
static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* 装好信号处理，然后做**第一次**访问当探针：位流没加载时它立刻跳回来，
 * 而不是把程序崩掉。返回 0 = 地址被译码。 */
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

int main(int argc, char **argv)
{
    int want = 1, watch = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--watch")) watch = 1;
        else want = atoi(argv[i]);
    }
    if (want <= 0) want = 1;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile uint8_t *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, BASE);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }

    volatile uint32_t *ctrl = (volatile uint32_t *)(m + OFF_CTRL);
    volatile uint32_t *flen = (volatile uint32_t *)(m + OFF_FLEN);
    const uint8_t *fp = (const uint8_t *)(m + OFF_FRAME);

    if (probe_mapped(m) != 0) {
        fprintf(stderr, "！！访问 0x%08lX 触发总线错误\n", (unsigned long)BASE);
        fprintf(stderr, "   该地址没有被 PL 译码 —— 说明位流没加载，或加载的不是 bd_vision。\n");
        fprintf(stderr, "   先看 /sys/class/fpga_manager/fpga0/state，再确认 bd_vision.bin 烧过。\n");
        return 3;
    }

    printf("基址 0x%08lX  ctrl=0x%08X  flen=%u  模式=%s\n",
           (unsigned long)BASE, (unsigned)*ctrl, (unsigned)*flen,
           watch ? "只读(--watch，不启动)" : "写 CTRL 启动");

    int ok = 0, bad = 0, to = 0, seen[8] = {0};
    uint8_t buf[FRAME_MAX];

    for (int f = 0; f < want; f++) {
        uint32_t st0 = *ctrl;
        uint32_t cnt0 = (st0 >> 8) & 0xFFFFu;

        if (!watch) {
            *ctrl = 1u;                    /* bit0 = start_pulse，启动一帧 */
        }

        /* 等"帧计数变了"（比只看 frame_done 翻转更可靠），再等 busy 落。
         * 注意 count 在 [23:8]，两种模式下都读得到。 */
        int waited = 0, advanced = 0;
        while (waited < FRAME_TIMEOUT_MS) {
            uint32_t st = *ctrl;
            uint32_t cnt = (st >> 8) & 0xFFFFu;
            if (cnt != cnt0) advanced = 1;
            if (advanced && !(st & 1u)) break;      /* 计数前进且 busy 已落 */
            sleep_ms(1);
            waited++;
        }
        if (!advanced) {
            printf("帧 %d: %s 等了 %d ms 帧计数都没动 —— %s\n", f,
                   watch ? "[--watch]" : "[已写 start]",
                   FRAME_TIMEOUT_MS,
                   watch ? "没有别的源在启动帧（这是正常的）"
                         : "start 写了但帧没起来，查 CTRL/位流");
            to++; bad++;
            continue;
        }

        uint32_t len = *flen;
        if (len == 0 || len > FRAME_MAX) {
            printf("帧 %d: 长度异常 %u（帧计数动了，但 flen 不对）\n", f, len);
            bad++; continue;
        }
        memcpy(buf, (const void *)fp, len);      /* 快照，避免读到一半被改写 */

        uint32_t crc = crc32(buf, len);
        int pat = -1;
        for (int i = 0; i < 8; i++)
            if (len == EXP_LEN && crc == EXP_CRC[i]) { pat = i; break; }

        if (pat >= 0) { ok++; seen[pat]++; }
        else          { bad++; }

        printf("帧 %2d: len=%4u crc=0x%08X  %s\n", f, len, crc,
               pat >= 0 ? "✓ 与协同仿真一致" : "✗ **不符**");

        if (pat < 0 && bad <= 2) {                   /* 头几次不符时打出头部，便于定位 */
            printf("       头 24 字节:");
            for (int i = 0; i < 24 && i < (int)len; i++) printf(" %02X", buf[i]);
            printf("\n");
        }
    }

    printf("\n=== 结果：一致 %d / %d，不符 %d（其中超时 %d）===\n", ok, want, bad, to);
    printf("图案命中分布:");
    for (int i = 0; i < 8; i++) printf(" [%d]=%d", i, seen[i]);
    printf("\n");
    return bad ? 2 : 0;
}
