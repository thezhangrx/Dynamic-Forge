/* 直接量"相机采集 → 用户态拿到"的延迟。
 *
 * 为什么必须单独量：板子上的诊断只能量"拿到像素之后"（pump → decide）。
 * 如果驱动里排着几帧，那我们一直在用**过期画面**做决策 —— 表现是"平台好像在三
 * 百毫秒甚至一秒前的位置上做决定"，而所有日志都正常。这是最后一块没量的延迟。
 *
 * 做法：V4L2 的每个 buffer 出队时带 `buf.timestamp`（CLOCK_MONOTONIC，帧**采集完成**
 * 的时刻）。拿它和**出队后立刻读的** CLOCK_MONOTONIC 相减，就是这一帧的年龄。
 * 同时打印队列入/出队的节奏，以及"我们处理得慢时年龄会不会堆起来"。
 *
 * 编译（板子上）：
 *   gcc -O2 -o /root/tools/v4l2_latency v4l2_latency.c
 * 运行：
 *   ./v4l2_latency --bufs 8 --frames 120
 *   ./v4l2_latency --bufs 2 --frames 120      # 对照：减少驱动队列
 *   ./v4l2_latency --bufs 8 --busy-ms 80      # 模拟"处理一帧要 80ms"
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static uint64_t mono_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

struct Buf { void* p; size_t len; };

static int cmp_u64(const void* a, const void* b) {
    const uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

int main(int argc, char** argv) {
    const char* dev = "/dev/video0";
    int nbuf = 8, frames = 120, busy_ms = 0, w = 640, h = 480;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--bufs") && i + 1 < argc) nbuf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--busy-ms") && i + 1 < argc) busy_ms = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i + 1 < argc) w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc) h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dev") && i + 1 < argc) dev = argv[++i];
    }
    if (nbuf < 2) nbuf = 2;

    int fd = open(dev, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) { perror("open"); return 1; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); return 1; }
    printf("[v4l2] 格式 %ux%u fourcc=%c%c%c%c\n", fmt.fmt.pix.width, fmt.fmt.pix.height,
           (fmt.fmt.pix.pixelformat) & 0xff, (fmt.fmt.pix.pixelformat >> 8) & 0xff,
           (fmt.fmt.pix.pixelformat >> 16) & 0xff, (fmt.fmt.pix.pixelformat >> 24) & 0xff);

    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof rb);
    rb.count = nbuf;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0) { perror("REQBUFS"); return 1; }
    printf("[v4l2] 申请 %d 个 buffer，实际拿到 %u\n", nbuf, rb.count);
    struct Buf* bufs = calloc(rb.count, sizeof *bufs);
    for (unsigned i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); return 1; }
        bufs[i].len = b.length;
        bufs[i].p = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (bufs[i].p == MAP_FAILED) { perror("mmap"); return 1; }
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); return 1; }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); return 1; }

    uint64_t* ages = calloc((size_t)frames, sizeof *ages);
    int got = 0;
    uint64_t t_prev = 0;
    while (got < frames) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
            if (errno == EAGAIN) { usleep(200); continue; }
            perror("DQBUF");
            break;
        }
        const uint64_t now = mono_ns();
        const uint64_t ts = (uint64_t)b.timestamp.tv_sec * 1000000000ull +
                            (uint64_t)b.timestamp.tv_usec * 1000ull;
        const uint64_t age = (now > ts) ? (now - ts) : 0;
        ages[got] = age;
        if (got < 6 || got % 30 == 0) {
            printf("[v4l2] 帧 %3d 采集→出队 %6.1f ms%s\n", got, age / 1e6,
                   (t_prev && now > t_prev) ? "" : "");
        }
        t_prev = now;
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF2"); break; }
        ++got;
        if (busy_ms > 0) usleep((useconds_t)busy_ms * 1000);
    }
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) { /* 收尾失败不影响结论 */ }

    if (got > 0) {
        qsort(ages, (size_t)got, sizeof *ages, cmp_u64);
        const uint64_t p50 = ages[got / 2];
        const uint64_t p90 = ages[(int)(got * 0.9)];
        const uint64_t p99 = ages[(int)(got * 0.99)];
        printf("---- 采集→出队延迟：n=%d  p50 %.1f ms  p90 %.1f ms  p99 %.1f ms  max %.1f ms ----\n",
               got, p50 / 1e6, p90 / 1e6, p99 / 1e6, ages[got - 1] / 1e6);
    }
    for (unsigned i = 0; i < rb.count; ++i) munmap(bufs[i].p, bufs[i].len);
    free(bufs);
    free(ages);
    close(fd);
    return 0;
}
