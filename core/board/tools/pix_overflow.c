/* ===========================================================================
 * pix_overflow.c —— **在真芯片上强制触发一次像素 FIFO 溢出**，验证粘滞位与恢复
 *
 * 为什么需要单独一个工具：`tb_axi --fifo`（Verilator 协同仿真）已经覆盖了 RTL 行为，
 * 驱动侧也有假寄存器测试；但"**真芯片上的粘滞位到底会不会置起、flush 之后能不能
 * 恢复**"只能实测。这条路径平时永远不会走到（PS 非缓存写实测 ~40 MB/s，而流水线
 * 每拍吃 1 像素 = 150 MB/s，3.75 倍余量），所以不主动灌满就永远没验过。
 *
 * 做法（**不启动流水线**，只往 FIFO 里灌）：
 *   1. PIXCTL = bit0（选 FIFO 源）+ bit1（flush，先清干净）；
 *   2. **扫一遍连写字数**：flush 后连写 N 个字，读 PIXSTAT 看粘滞位有没有置起，
 *      找出"连写多少字才开始溢出"—— 这是 PS 盲写的**突发余量**（消费者一直在排空，
 *      所以灌完之后占用反而是低的：溢出发生在突发期间，这才是真实场景）；
 *   3. 粘滞性：flush 之前一直保持；
 *   4. flush 之后再读：占用归零、bit16 清掉（说明能恢复，不会永久卡死）。
 *
 * ⚠ 只碰 0x0D4/0x0D8/0x0DC 三个像素寄存器，不动配置、不写 CTRL 的 start 位，
 *   所以不会把流水线/决策状态搞乱。跑之前确认板上没有程序在用 PL。
 *
 * 编译：gcc -O2 -Wall -o pix_overflow pix_overflow.c
 * 运行：./pix_overflow [灌入字数,默认4096]
 * 退出码 0 = 粘滞位如期置起且能清掉；非 0 = 有断言不成立
 * =========================================================================== */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>

#define BASE       0x43C00000UL
#define MAP_SIZE   0x1000UL
#define OFF_PIXPUSH 0x0D4
#define OFF_PIXSTAT 0x0D8
#define OFF_PIXCTL  0x0DC

#define FIFO_WORDS 2048u          /* RTL: vision_pixfeed.v 的 FIFO 深度（字） */

/* usleep 在 POSIX.1-2008 里被移除了，_POSIX_C_SOURCE=200809L 下不声明
 * （gcc 会给 implicit-declaration 警告）—— 用 nanosleep。 */
static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static int fails;

static void check(int cond, const char *what)
{
    if (!cond) { ++fails; printf("  ✗ %s\n", what); }
    else       { printf("  ✓ %s\n", what); }
}

int main(int argc, char **argv)
{
    unsigned words = (argc > 1) ? (unsigned)strtoul(argv[1], NULL, 0) : 4096u;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile uint8_t *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }

    volatile uint32_t *push = (volatile uint32_t *)(m + OFF_PIXPUSH);
    volatile uint32_t *stat = (volatile uint32_t *)(m + OFF_PIXSTAT);
    volatile uint32_t *ctl  = (volatile uint32_t *)(m + OFF_PIXCTL);

    printf("=== 像素 FIFO 溢出（真芯片）===  灌入 %u 字，FIFO 深度 %u 字\n",
           words, FIFO_WORDS);

    /* 1) 选 FIFO 源 + flush：保证从"空 + 粘滞位清零"开始 */
    *ctl = 0x3u;
    sleep_ms(1);
    uint32_t st = *stat;
    printf("  flush 后：占用 %u 字节，溢出位 %u\n", st & 0xFFFFu, (st >> 16) & 1u);
    check((st & 0xFFFFu) == 0u, "flush 之后占用应为 0");
    check(((st >> 16) & 1u) == 0u, "flush 之后溢出位应为 0");

    /* 2) 扫阈值：flush 之后连写 N 字，看粘滞位什么时候第一次置起。
     *    消费者一直在排空，所以"灌满"是突发期间的事：N 越小越可能不溢出。 */
    printf("\n  连写字数 → 溢出位（每档先 flush）\n");
    unsigned threshold = 0u;
    for (unsigned n = 32u; n <= words; n *= 2u) {
        *ctl = 0x3u;                       /* flush：占用清零、粘滞位清 */
        sleep_ms(1);
        for (unsigned i = 0; i < n; ++i) *push = 0xA5A50000u | (i & 0xFFFFu);
        st = *stat;
        unsigned o = (st >> 16) & 1u, occ_n = st & 0xFFFFu;
        printf("    %6u 字 → 溢出 %u（读时占用 %u 字节）%s\n",
               n, o, occ_n, o ? "  ← 第一次溢出" : "");
        if (o && threshold == 0u) threshold = n;
    }
    check(threshold != 0u, "连写到某档必须出现溢出（粘滞位在真芯片上会置起）");
    if (threshold != 0u) {
        printf("  ⇒ 突发余量：连写 %u 字（%u 字节 ≈ %.1f 行 640x3 图像）之内不丢像素\n",
               threshold / 2u, threshold / 2u * 4u,
               (double)(threshold / 2u * 4u) / (640.0 * 3.0));
    }
    /* 灌到测试上限之后，粘滞位必须是 1（下面接着验"粘滞 + 能清"） */
    *ctl = 0x3u;
    sleep_ms(1);
    for (unsigned i = 0; i < words; ++i) *push = 0x5A5A0000u | (i & 0xFFFFu);
    st = *stat;
    printf("  最后一次连写 %u 字后：占用 %u 字节，溢出位 %u\n",
           words, st & 0xFFFFu, (st >> 16) & 1u);

    /* 3) 再灌一轮：粘滞位应当保持（粘滞 = 一直到 flush 才清） */
    for (unsigned i = 0; i < 64u; ++i) *push = 0x5A5A0000u | i;
    st = *stat;
    check(((st >> 16) & 1u) == 1u, "overflow 是粘滞位：flush 之前一直保持");

    /* 4) flush 恢复 */
    *ctl = 0x3u;
    sleep_ms(1);
    st = *stat;
    printf("  再 flush 后：占用 %u 字节，溢出位 %u\n", st & 0xFFFFu, (st >> 16) & 1u);
    check((st & 0xFFFFu) == 0u, "flush 之后占用应归零（能恢复，不会永久卡死）");
    check(((st >> 16) & 1u) == 0u, "flush 之后溢出位应清掉");

    /* 5) 收尾：把像素源切回 din_*（0），免得后续 run_frame 以为还有像素进来 */
    *ctl = 0x0u;

    printf("----\n%s（不符 %d）\n", fails ? "失败" : "通过", fails);
    munmap((void *)m, MAP_SIZE);
    close(fd);
    return fails ? 1 : 0;
}
