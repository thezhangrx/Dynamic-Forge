/* feed_pl.c —— 板端把一帧真实图像喂进 PL（vision_top），C 版，目标 L1（≤35 ms/帧）
 *
 * 为什么要有 C 版：Python 版（feed_pl.py）每推一个 32 位字要过一次解释器，
 * 实测 560~600 ms/帧 —— 瓶颈完全在喂数器，不在 PL。C 版把 691200 字节
 * （172800 个字）用紧循环写进映射好的 AXI 窗口，预期 40 MB/s 量级 → ~17 ms。
 *
 * 用法（板上；要 /dev/mem）：
 *     ./feed_pl frame.ppm [--frames N] [--dump out.bin] [--quiet]
 *
 * 判据（与 core/fpga/docs/PL_A_B_TASK.md 一致）：
 *     magic=0x31564842 / ver=1 / 长度 = 76+36*(n_obs+n_gaps) == FLEN / CRC32 对得上
 *
 * 接口（core/board/vision_regs.hpp ↔ core/fpga/rtl/vision_axi.v）：
 *   0x0C0 CTRL 写 bit0=1 启动一帧；读 bit0=busy bit1=done bit16=ovf
 *   0x0C4 SEQ / 0x0C8,0x0CC STAMP_NS
 *   0x0D0 FLEN（只读）  0x0D4 PIXPUSH（整字，4 字节，低字节先）
 *   0x0D8 PIXSTAT      0x0DC PIXCTL（bit0=1 用 FIFO，bit1=flush）
 *   0x100 FRAME（2048 B 窗口）
 *
 * 顺序是**先 start、等 busy、再推像素**：vision_top 在 din_sof 之前忽略像素，
 * 先推会整帧错位（帧长/CRC 仍然自洽，极难发现 —— Python 版踩过一次）。
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define REG_BASE   0x43C00000UL
#define OFF_CTRL   0x0C0
#define OFF_SEQ    0x0C4
#define OFF_STAMP0 0x0C8
#define OFF_STAMP1 0x0CC
#define OFF_FLEN   0x0D0
#define OFF_PIXPUSH 0x0D4
#define OFF_PIXSTAT 0x0D8
#define OFF_PIXCTL  0x0DC
#define OFF_FRAME   0x100
#define FRAME_WINDOW 2048

/* 把帧窗口从设备映射**逐字节**拷到普通内存。
 *
 * 为什么不能直接 `memcpy`/`fwrite` 读映射区：这块内存是 Device 属性，
 * ARM 上**非对齐访问会触发总线错误**（SIGBUS），而 glibc 的 memcpy/fwrite
 * 会按 8/16 字节宽、且可能非对齐地读 —— 实测 `fwrite(raw + 0x100, ...)`
 * 直接 SIGBUS（rc=135），而逐字节读完全正常。 */
static void copy_frame_window(volatile uint8_t *src, uint8_t *dst, size_t n) {
    for (size_t i = 0; i < n; ++i) dst[i] = src[i];
}
#define MAGIC_LE    0x31564842UL   /* 'B''H''V''1' 小端（0x31='1'；曾错写成 0x32 导致永远报 MISMATCH） */

static volatile uint32_t *regs;

static inline void wr(uint32_t off, uint32_t v) { regs[off >> 2] = v; }
static inline uint32_t rd(uint32_t off) { return regs[off >> 2]; }

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

/* 读 PPM(P6)，返回 RGB 连续缓冲；失败返回 NULL */
static uint8_t *load_ppm(const char *path, int *pw, int *ph) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    char magic[3] = {0};
    int w = 0, h = 0, maxv = 0;
    if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P6") != 0) {
        fprintf(stderr, "%s: 不是 P6\n", path); fclose(f); return NULL;
    }
    /* 跳过注释/空白 */
    int c;
    while ((c = fgetc(f)) != EOF && (c == ' ' || c == '\n' || c == '\r' || c == '\t')) {}
    ungetc(c, f);
    if (fscanf(f, "%d %d %d", &w, &h, &maxv) != 3 || w <= 0 || h <= 0) {
        fprintf(stderr, "%s: 头部解析失败\n", path); fclose(f); return NULL;
    }
    fgetc(f);                                  /* 头部后单个空白 */
    size_t n = (size_t)w * h * 3;
    uint8_t *buf = malloc(n);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, n, f) != n) {
        fprintf(stderr, "%s: 像素不足（期望 %zu 字节）\n", path, n); 
        free(buf); fclose(f); return NULL;
    }
    fclose(f);
    *pw = w; *ph = h;
    return buf;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "用法: %s frame.ppm [--frames N] [--dump out.bin] [--quiet]\n", argv[0]);
        return 2;
    }
    const char *img = argv[1];
    int frames = 1, quiet = 0;
    const char *dump = NULL;
    for (int i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dump") && i + 1 < argc) dump = argv[++i];
        else if (!strcmp(argv[i], "--quiet")) quiet = 1;
    }

    int w = 0, h = 0;
    uint8_t *rgb = load_ppm(img, &w, &h);
    if (!rgb) return 1;
    size_t nbytes = (size_t)w * h * 3;
    if (nbytes % 4) {
        fprintf(stderr, "帧字节数 %zu 不是 4 的倍数（整字写 FIFO 要求补齐）\n", nbytes);
        free(rgb); return 1;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); free(rgb); return 1; }
    void *map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)REG_BASE);
    if (map == MAP_FAILED) { perror("mmap"); close(fd); free(rgb); return 1; }
    regs = (volatile uint32_t *)map;

    double t_all0 = now_ms();
    int bad = 0;
    for (int k = 0; k < frames; ++k) {
        double t0 = now_ms();
        wr(OFF_PIXCTL, 0x3);                 /* FIFO 源 + flush */
        wr(OFF_PIXCTL, 0x1);                 /* FIFO 源 */
        wr(OFF_SEQ, (uint32_t)(k + 1));
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        uint64_t ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
        wr(OFF_STAMP0, (uint32_t)ns);
        wr(OFF_STAMP1, (uint32_t)(ns >> 32));
        wr(OFF_CTRL, 1);                     /* 启动 */

        /* 等忙碌（= 流水线进入接收态）；200 ms 超时 */
        double tw = now_ms();
        while (!(rd(OFF_CTRL) & 0x1)) {
            if (now_ms() - tw > 200.0) {
                fprintf(stderr, "PL 200 ms 未进入忙碌\n"); free(rgb); return 1;
            }
        }
        /* 紧循环推字：低字节先 */
        const uint32_t *words = (const uint32_t *)rgb;
        size_t nw = nbytes / 4;
        for (size_t i = 0; i < nw; ++i) wr(OFF_PIXPUSH, words[i]);
        double t_push = now_ms();

        /* 等 busy 落 */
        while (rd(OFF_CTRL) & 0x1) {}
        /* 溢出**不算失败**：PS 推得比流水线吃得快时，帧取满 230400 像素之后
         * 剩下的字会灌满 8 KB FIFO 并置粘滞 ovf —— 那时帧已经完整了。
         * 判"帧对不对"只认 magic/长度/CRC（下面那三道），溢出不参与。
         * （踩过：把 ovf 当错误，320 帧里 91 帧被误判成失败，而它们逐字节一致。） */
        int ovf = (rd(OFF_CTRL) & 0x10000) ? 1 : 0;
        double t1 = now_ms();

        uint32_t magic = rd(OFF_FRAME + 0);
        uint32_t ver   = rd(OFF_FRAME + 4) & 0xFFFF;
        uint32_t seq   = rd(OFF_FRAME + 8);
        uint8_t win[FRAME_WINDOW];           /* 帧窗口的普通内存副本（见 copy_frame_window） */
        copy_frame_window((volatile uint8_t *)map + OFF_FRAME, win, FRAME_WINDOW);
        const uint8_t *raw = win;            /* 之后一律只碰这份副本 */
        uint32_t n_obs = raw[12];
        uint32_t n_gaps = raw[16];
        uint32_t need  = 76 + 36 * (n_obs + n_gaps);
        uint32_t flen  = rd(OFF_FLEN);
        /* CRC32(zlib) 覆盖前面 need-4 字节 */
        uint32_t crc = 0xFFFFFFFFu;
        for (uint32_t i = 0; i < need - 4; ++i) {
            crc ^= raw[i];
            for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
        }
        crc ^= 0xFFFFFFFFu;
        uint32_t crc_frame;
        crc_frame = (uint32_t)raw[need - 4] | ((uint32_t)raw[need - 3] << 8)
                  | ((uint32_t)raw[need - 2] << 16) | ((uint32_t)raw[need - 1] << 24);
        int ok = (magic == MAGIC_LE) && (need == flen) && (crc == crc_frame);
        if (!ok) bad++;
        if (!quiet) {
            printf("帧 %d: 推字 %.2f ms，整帧 %.2f ms | magic=0x%08x ver=%u seq=%u "
                   "n_obs=%u n_gaps=%u need=%u flen=%u crc %s%s\n",
                   k + 1, t_push - t0, t1 - t0, magic, ver, seq, n_obs, n_gaps,
                   need, flen, ok ? "OK" : "MISMATCH",
                   ovf ? "（尾部溢出，帧已完整）" : "");
        }
        if (dump) {
            FILE *o = fopen(dump, k ? "ab" : "wb");
            if (o) { fwrite(raw, 1, need, o); fclose(o); }
        }
    }
    if (!quiet) printf("总耗时 %.2f ms（%d 帧）\n", now_ms() - t_all0, frames);
    munmap(map, 4096);
    close(fd);
    free(rgb);
    return bad ? 1 : 0;
}
