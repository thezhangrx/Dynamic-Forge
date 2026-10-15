/* ===========================================================================
 * axi_bw.c —— 量一下 PS↔PL 的 AXI 吞吐（决定"帧缓冲放哪"）
 *
 * 为什么需要：真摄像头要进 PL，一帧 640x360x3 = 691,200 字节。两条路：
 *   ① PL 里放帧缓冲，PS 直接写进去（简单，但要写得够快）
 *   ② PS 写 DDR，PL 侧 AXI master 去读（要写 master，但 PS 侧是缓存写，很快）
 * 选哪条取决于"PS 直接写 PL"到底多快 —— 这个数只能实测。
 *
 * ⚠ 只写 0x07C（CFG_STATUS，只读，写进去被忽略）与读 0x0C0（CTRL），
 *   都不改任何状态，可以放心反复打。别拿配置寄存器当靶子（会把阈值写乱）。
 *
 * 编译：gcc -O2 -o axi_bw axi_bw.c
 * 运行：./axi_bw [字节数,默认691200]
 * =========================================================================== */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>

#define BASE     0x43C00000UL
#define MAP_SIZE 0x1000UL
#define OFF_CTRL 0x0C0
#define OFF_RO   0x07C          /* CFG_STATUS：只读，写进去被忽略 */
/* 64 位写要用**8 字节对齐**的地址：0x078 起 8 字节覆盖 0x078 与 0x07C 两个配置槽，
 * 两个都是"未映射/只读"→ 写进去被忽略，安全。
 * （踩过：拿 0x07C 做 64 位写 → 非对齐访问 → **SIGBUS**。Device 内存上
 *   非对齐的 64 位访问在 ARM 上是直接报总线错，不是慢一点。） */
#define OFF_RO64 0x078

static double now_s(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
    size_t bytes = (argc > 1) ? (size_t)strtoull(argv[1], NULL, 0) : 691200UL;
    size_t words = bytes / 4;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile uint8_t *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }
    volatile uint32_t *ro = (volatile uint32_t *)(m + OFF_RO);
    volatile uint32_t *ctrl = (volatile uint32_t *)(m + OFF_CTRL);

    printf("=== PS -> PL AXI 吞吐（%zu 字节 = %zu 个 32 位写）===\n", bytes, words);

    /* ① 32 位写 */
    double t0 = now_s();
    for (size_t i = 0; i < words; i++) *ro = (uint32_t)i;
    double t1 = now_s();
    const double w32_ms = (t1 - t0) * 1e3;
    printf("  32 位逐字写: %.2f ms  → %.1f MB/s\n",
           w32_ms, (double)bytes / (t1 - t0) / 1e6);

    /* ② 64 位写（对齐）：看能不能把事务数减半 */
    double w64_ms = 0.0;
    t0 = now_s();
    {
        volatile uint64_t *ro64 = (volatile uint64_t *)(m + OFF_RO64);
        for (size_t i = 0; i < words / 2; i++) *ro64 = ((uint64_t)i << 32) | i;
    }
    t1 = now_s();
    w64_ms = (t1 - t0) * 1e3;
    printf("  64 位逐字写: %.2f ms  → %.1f MB/s\n",
           w64_ms, (double)bytes / (t1 - t0) / 1e6);

    /* ③ 读（对照） */
    volatile uint32_t sink = 0;
    t0 = now_s();
    for (size_t i = 0; i < words; i++) sink += *ctrl;
    t1 = now_s();
    printf("  32 位逐字读: %.2f ms  → %.1f MB/s\n",
           (t1 - t0) * 1e3, (double)bytes / (t1 - t0) / 1e6);
    (void)sink;

    /* ④ 一帧的预算：30 fps = 33.3 ms/帧。写这一帧要多久，决定架构选型。 */
    printf("\n  一帧 691200B：32 位写 %.1f ms = 33.3ms 预算的 %.0f%%\n",
           w32_ms, 100.0 * w32_ms / 33.3);
    if (w64_ms > 0.0) {
        printf("                64 位写 %.1f ms = 33.3ms 预算的 %.0f%%\n",
               w64_ms, 100.0 * w64_ms / 33.3);
        printf("  ⇒ 64 位写比 32 位快 %.2fx\n", w32_ms / w64_ms);
    }
    return 0;
}
