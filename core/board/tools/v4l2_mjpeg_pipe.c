/* 低延迟 MJPG 取帧器：只把**最新**的一帧交出去，绝不让驱动队列堆起来。
 *
 * 为什么需要它（本轮实测的根因）
 * ----------------------------
 * 我们原来是 `ffmpeg -f v4l2 ... | 板子程序`。ffmpeg 的 v4l2 输入会用**驱动默认的
 * 队列深度**（这块板子上是 8 个 buffer），而我们的处理只有 ~13 帧/秒（每帧 75 ms）。
 * 相机 60 帧/秒往里灌，队列永远是满的 → **DQBUF 拿到的是队列里最老的一帧**。
 *
 * 用 `buf.timestamp`（CLOCK_MONOTONIC，采集完成时刻）直接量：
 *     buffer 数   采集→出队（p50，处理 80 ms/帧时）
 *        8              587 ms
 *        4              282 ms
 *        2              122 ms
 * 这正好解释了"按速度方向分解出来的端到端滞后 ≈ 486 ms"。
 *
 * 本程序的策略：**申请尽量少的 buffer + 每次 DQBUF 后把队列里剩下的全部取出并立刻
 * 回收，只把最新的那帧写出去**。这样无论下游多慢，交出去的帧最多只比采集晚一个
 * 相机帧周期（16.7 ms），而不是几百毫秒。
 *
 * 用法（输出的 MJPG 直接喂 ffmpeg 解码，无需 libjpeg 头文件）：
 *   ./v4l2_mjpeg_pipe --bufs 2 --w 640 --h 480 2>/tmp/cap.log \
 *     | ffmpeg -f mjpeg -i - -vf scale=640:360 -pix_fmt rgb24 -f rawvideo - \
 *     | <板子程序>
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
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

int main(int argc, char** argv) {
    const char* dev = "/dev/video0";
    int nbuf = 2, w = 640, h = 480, log_every = 60;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--bufs") && i + 1 < argc) nbuf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i + 1 < argc) w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc) h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--log-every") && i + 1 < argc) log_every = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dev") && i + 1 < argc) dev = argv[++i];
    }
    if (nbuf < 2) nbuf = 2;

    int fd = open(dev, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) { perror("open"); return 1; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = (uint32_t)w;
    fmt.fmt.pix.height = (uint32_t)h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT"); return 1; }

    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof rb);
    rb.count = (uint32_t)nbuf;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0) { perror("REQBUFS"); return 1; }
    fprintf(stderr, "[cap] %ux%u MJPG，申请 %d buffer，实际 %u\n", w, h, nbuf, rb.count);

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

    long n_frame = 0, n_dropped = 0;
    uint64_t sum_age = 0;
    for (;;) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
            if (errno == EAGAIN) {
                struct pollfd pf = {fd, POLLIN, 0};
                poll(&pf, 1, 100);
                continue;
            }
            perror("DQBUF");
            break;
        }
        // **把队列里剩下的全部取出**：只保留最新的这一帧，其余立刻回收。
        for (;;) {
            struct v4l2_buffer b2;
            memset(&b2, 0, sizeof b2);
            b2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b2.memory = V4L2_MEMORY_MMAP;
            if (xioctl(fd, VIDIOC_DQBUF, &b2) < 0) break;   // EAGAIN：队列空了
            if (xioctl(fd, VIDIOC_QBUF, &b) < 0) break;     // 旧的还回去
            b = b2;
            ++n_dropped;
        }
        const uint64_t now = mono_ns();
        const uint64_t ts = (uint64_t)b.timestamp.tv_sec * 1000000000ull +
                            (uint64_t)b.timestamp.tv_usec * 1000ull;
        const uint64_t age = (now > ts) ? (now - ts) : 0;
        sum_age += age;
        ++n_frame;
        if (log_every > 0 && n_frame % log_every == 0) {
            fprintf(stderr, "[cap] 帧 %ld 本帧滞后 %.1f ms，平均 %.1f ms，累计丢旧帧 %ld\n",
                    n_frame, age / 1e6, (double)sum_age / n_frame / 1e6, n_dropped);
        }
        // 把这一帧的 MJPG 字节写出去（UVC 的一个 buffer 就是一张完整的 JPEG）
        const size_t n = b.bytesused;
        const uint8_t* p = (const uint8_t*)bufs[b.index].p;
        size_t off = 0;
        while (off < n) {
            const ssize_t k = write(1, p + off, n - off);
            if (k < 0) {
                if (errno == EINTR) continue;
                fprintf(stderr, "[cap] 下游关闭（%s）\n", strerror(errno));
                xioctl(fd, VIDIOC_STREAMOFF, &type);
                return 0;
            }
            off += (size_t)k;
        }
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF2"); break; }
    }
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    fprintf(stderr, "[cap] 结束：%ld 帧，丢旧帧 %ld，平均滞后 %.1f ms\n",
            n_frame, n_dropped, n_frame ? (double)sum_age / n_frame / 1e6 : 0.0);
    close(fd);
    return 0;
}
