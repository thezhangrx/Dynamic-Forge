/* ===========================================================================
 * soak_sampler.c —— 长跑时**只读**地每秒采一次状态（不影响正在跑的程序）
 *
 * 为什么单独写：板子上没有 devmem，而 `read_frame` 会**写 CTRL 启动**一整帧 ——
 * 拿它采样会把正在跑的闭环搅乱。这里只读三个只读寄存器：
 *
 *     PIXSTAT(0x0D8)  [15:0] 缓冲字节  [16] 像素 FIFO 溢出粘滞  [17] feed busy
 *     CTRL(0x0C0)     bit0 busy  bit1 frame_done  [23:8] 帧计数
 *     FLEN(0x0D0)     上一帧几何帧长度
 *
 * 另外顺带盯**进程资源**（内存/句柄），这两样泄漏在长跑里最容易看漏：
 * 给 `--pid N` 就读 /proc/N/status 的 VmRSS 与 /proc/N/fd 的个数。
 *
 * 编译：gcc -O2 -Wall -o soak_sampler soak_sampler.c
 * 运行：./soak_sampler --seconds 1800 --pid $(pidof vision_board_devmem)
 * =========================================================================== */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/mman.h>

#define BASE       0x43C00000UL
#define MAP_SIZE   0x1000UL
#define OFF_CTRL   0x0C0
#define OFF_FLEN   0x0D0
#define OFF_PIXSTAT 0x0D8

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/*: 读 /proc/<pid>/status 里的 VmRSS（KB）；读不到返回 -1。 */
static long rss_kb(long pid)
{
    if (pid <= 0) return -1;
    char path[64];
    snprintf(path, sizeof(path), "/proc/%ld/status", pid);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256];
    long kb = -1;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "VmRSS:", 6) == 0) { sscanf(line + 6, "%ld", &kb); break; }
    }
    fclose(f);
    return kb;
}

/*: 数 /proc/<pid>/fd 下的条目数（句柄泄漏）；读不到返回 -1。 */
static long n_fds(long pid)
{
    if (pid <= 0) return -1;
    char path[64];
    snprintf(path, sizeof(path), "/proc/%ld/fd", pid);
    DIR *d = opendir(path);
    if (!d) return -1;
    long n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] != '.') ++n;
    }
    closedir(d);
    return n;
}

int main(int argc, char **argv)
{
    long seconds = 1800, pid = -1;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atol(argv[++i]);
        else if (!strcmp(argv[i], "--pid") && i + 1 < argc) pid = atol(argv[++i]);
        else { fprintf(stderr, "用法：%s [--seconds N] [--pid PID]\n", argv[0]); return 2; }
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile uint8_t *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }
    volatile uint32_t *ctrl = (volatile uint32_t *)(m + OFF_CTRL);
    volatile uint32_t *flen = (volatile uint32_t *)(m + OFF_FLEN);
    volatile uint32_t *pstat = (volatile uint32_t *)(m + OFF_PIXSTAT);

    printf("秒  帧计数  busy done 缓冲字节 ovf feedbusy  FLEN  VmRSS(KB) fds\n");
    fflush(stdout);

    unsigned ovf_seen = 0, max_occ = 0, last_cnt = 0, stall_ticks = 0, max_stall = 0;
    long rss0 = rss_kb(pid), rss1 = rss0, fd0 = n_fds(pid), fd1 = fd0;

    for (long t = 1; t <= seconds; ++t) {
        uint32_t c = *ctrl, p = *pstat, L = *flen;
        unsigned cnt = (c >> 8) & 0xFFFFu, occ = p & 0xFFFFu, o = (p >> 16) & 1u;
        if (o) ovf_seen = 1;
        if (occ > max_occ) max_occ = occ;
        if (cnt == last_cnt) { ++stall_ticks; if (stall_ticks > max_stall) max_stall = stall_ticks; }
        else stall_ticks = 0;
        last_cnt = cnt;
        long r = rss_kb(pid), nf = n_fds(pid);
        if (r > 0) rss1 = r;
        if (nf > 0) fd1 = nf;

        printf("%3ld %6u   %u    %u   %6u   %u    %u    %5u  %8ld %4ld\n",
               t, cnt, c & 1u, (c >> 1) & 1u, occ, o, (p >> 17) & 1u, L, r, nf);
        fflush(stdout);
        sleep_ms(1000);
    }

    printf("---- 汇总 ----\n");
    printf("  像素 FIFO 溢出粘滞位: %s（最大占用 %u 字节）\n",
           ovf_seen ? "**出现过**" : "从未置起", max_occ);
    /* ⚠ 这个计数是 vision_top 的**完成帧计数**，PL 会自己背靠背地跑（不需要 start），
     *   所以它远高于应用侧的帧率（实测 ~1000/s）—— 它在这里只当**存活/卡顿**信号用，
     *   别当成"应用的帧率"。应用帧率看程序自己的输出。 */
    printf("  PL 完成帧计数: %u（%u/s 均摊；PL 自己背靠背跑，**不是**应用帧率）\n",
           last_cnt, (unsigned)(last_cnt / (seconds ? seconds : 1)));
    printf("  最长连续不涨秒数: %u%s\n", max_stall,
           max_stall >= 3 ? "  ← 有卡顿，值得看" : "");
    if (pid > 0) {
        printf("  VmRSS: %ld → %ld KB（Δ%+ld KB）\n", rss0, rss1, rss1 - rss0);
        printf("  句柄数: %ld → %ld（Δ%+ld）\n", fd0, fd1, fd1 - fd0);
    }
    munmap((void *)m, MAP_SIZE);
    close(fd);
    return 0;
}
