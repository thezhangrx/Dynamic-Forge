// ===========================================================================
// pl_vision.cpp —— 板端感知源：**相机帧 → 正射到页面 → PL 判墙 → vt.jsonl**
//
// 为什么要有它（L2 接线）
// ----------------------
// 到这一步为止，板端决策吃的 `/tmp/vt.jsonl` 是 `vt_sparse`（CPU 版稀疏视觉）
// 写出来的。L0 已经证明 PL 的几何帧与 C 参考 **320/320 帧逐字节一致**、单帧
// 中位 23.4 ms，所以"能不能用 PL 替代 CPU 感知"只剩接线。这个文件就是那条接线，
// 而且**只换感知**：接口（stdin 640×480 RGB24、`--quad-file`、`--jsonl`）与
// `vt_sparse` 完全一致，闭环里只需要把二进制名换掉。
//
// 坐标契约（为什么不拟合、只做等比缩放）
// --------------------------------------
//   * `vt_sparse` 定义的页面空间是 **640×480，y 向上**（行 r ↔ 页面 y = FH-(r+0.5)）。
//   * PL 的输入帧固定 **640×360**（`vision_top` 的 WIDTH/HEIGHT）。本文件把相机帧
//     正射到**同一页面**的 640×360 采样上：列 c ↔ 页面 x = c+0.5（1:1），
//     行 r ↔ 页面 y360 = 360-(r+0.5)。
//   * 于是 PL 报出的行 r 换回页面 y 只是 **×480/360**：
//         y_page = (360 - y_pl) * 480/360 = (360 - y_pl) * 4/3
//     x 则 1:1（两边都是 640 宽）。**没有需要标定的自由参数**。
//   * 正射用 `vision_board::field_frame::homography(quad, 640, 360)` —— 与
//     `vt_sparse` 同一个求解器、同一份 quad，只是目标分辨率不同（不另写一套几何）。
//
// 用法（板上；与 vt_sparse 的调用方式一致）
// ----------------------------------------
//     ffmpeg … -f rawvideo -pix_fmt rgb24 - | pl_vision   （续行参数见下）
//         --w 640 --h 480 --frame-roi 0 0 640 480 \
//         --quad-file /tmp/quad.json --quad-state /tmp/qb.txt --jsonl /tmp/vt.jsonl
//
// 输出 jsonl（**与 vt_sparse 同构**，下游 board_agent 不用改）：
//     {"frame":N,"t":…,"frame_geom":1,"quad_ext":1,
//      "walls":[{"y":…,"gl":…,"gr":…},…],"player":[x,y]|null,"n":K,"no_gap":…,"fast":1}
//
// 诊断
// ----
//   * `--dump-ppm 前缀`：把正射后的 640×360 页面图按帧号落盘（PPM），用于离线对拍；
//   * `--stats`：每 20 帧往 stderr 打一次"正射 ms / 推数 ms / PL 整帧 ms"。
// ===========================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "field_frame.hpp"        // Quad / homography / find
#include "geometry_frame.hpp"     // parse_geometry_frame（长度/magic/版本/CRC 四道校验）
#include "rectify.hpp"            // Homography / invert
#include "vision_regs.hpp"        // 寄存器偏移（与 RTL 契约的唯一来源）

namespace {

constexpr int kCamW = 640, kCamH = 480;    // ffmpeg 送进来的相机帧
constexpr int kPlW = 640, kPlH = 360;      // PL 流水线固定的页面采样分辨率

//: 页面（世界单位）尺寸。**必须与页面渲染/标定一致**（场地 640×480）。
constexpr double kPageW = 640.0, kPageH = 480.0;

//: 页面在 PL 输入帧里的摆放：**等比缩放后居中**（"fit"）。
//:
//: ⚠ 为什么不是"四角正好铺满整幅图"（踩过）：判据靠**画面里的左右白框**定位场地
//: 范围（`span=[x_min,x_max]`），白框若压在图像边缘（列 0 与 639），会被
//: "离边 10 px 不算白框"那条规则剔掉 → `bands=0`，一道墙都报不出；而同一帧用
//: 自适应阈值的 vt_sparse 却能报出墙。语料的几何就是这样摆的：640×480 的页面
//: 按 0.75 放进 640×360，左右各留 80 px。
constexpr double kFitScale = (kPlW / kPageW < kPlH / kPageH) ? (kPlW / kPageW)
                                                            : (kPlH / kPageH);
constexpr double kOffX = (kPlW - kPageW * kFitScale) * 0.5;   // = 80
constexpr double kOffY = (kPlH - kPageH * kFitScale) * 0.5;   // = 0

double now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

// ---------------------------------------------------------------------------
// PL 寄存器窗口
// ---------------------------------------------------------------------------
struct PlRegs {
    volatile uint32_t* r = nullptr;
    void* map = nullptr;

    bool open() {
        int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0) { std::perror("/dev/mem"); return false; }
        map = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, vision_board::kRegBase);
        ::close(fd);
        if (map == MAP_FAILED) { std::perror("mmap"); map = nullptr; return false; }
        r = static_cast<volatile uint32_t*>(map);
        return true;
    }
    uint32_t rd(uint32_t off) const { return r[off >> 2]; }
    //: **必须用单个 32 位 volatile 写**。踩过的坑：Python 的 `struct.pack_into("<I")`
    //: 在真硅上一次 store 会变成 3 笔 AXI 写（FIFO 进 3 份、整幅图错位），而 C 的
    //: `*(volatile uint32_t*)p = v` 是 1 笔 —— 用 `memcpy` 之类也可能被编成多笔。
    void wr(uint32_t off, uint32_t v) { r[off >> 2] = v; }
};

// ---------------------------------------------------------------------------
// P0：PS 只搬帧，正射在 PL 里做
// ---------------------------------------------------------------------------
//: 旧路（r8 位流）是"PS 自己正射（≈25.7 ms）→ 把 640×360 页面图逐个 32 位字写进
//: 0x0D4（实测 4.4~7.8 M 字/s，≈17 ms）→ vision_top 判据"。P0 把这两段一起搬进 PL：
//:
//:     PS：memcpy 640×480×3 → udmabuf → sync_for_device → FRMCTL=src|clr
//:         → FRMCTL=src|start → 每帧前等 busy=0、起跑后等 busy 落 → 读 FLEN 字节
//:     PL：HP0 按行读（1920 B/行）→ 行缓存 32 行 → 正射 → camadapt → vision_top
//:
//: ⚠ 因为**正射与相机域适配都已经在 PL 里做了**，PS 这边不许再调
//:   `Rectifier::sample_rgb()` 和 `camera_domain_adapt()` —— 前者白付 25.7 ms，
//:   后者会把已经适配过的图再染一遍（墙的 R−B 被减两次），两条都是"静默错"。
//:
//: ⚠ 顺序（每一句在真硅上都翻过车，别重排）：
//:   ① 起跑前等 vision_top 的 busy=0：上一帧没结束就写 start 会被 S_IDLE 吞掉，
//:      量出"0.00 ms 就完成"的假数（用的是上一帧残留的 done 位）；
//:   ② memcpy + sync_for_device：u-dma-buf 是 sync_mode=1，CPU 写完必须告诉驱动，
//:      否则 PL 可能读到旧 DRAM；
//:   ③ 写 seq/stamp（进几何帧头，对应 RTL 的 0x0C4 / 0x0C8 / 0x0CC）；
//:   ④ 先 FRMCTL=src|clr 再 FRMCTL=src|start：start 这一位同时起 vision_top **与**
//:      正射，分两次写会让两个引擎的"帧边界"错开一帧 → 整幅平移；
//:   ⑤ 等 busy 落：端到端到此为止（正射自己的 done 只表示"像素喂完了"）；
//:   ⑥ 按 **FLEN（0x0D0）** 读窗口：不要按窗口头里的 n_obs 反算 —— 那条在 P0
//:      路径上可能读到上一帧的残留（frame_len 由 dout_last 那一拍才定）。
//:
//: ⚠ 超时一律按**时间**判（CLOCK_MONOTONIC），不用"空转多少次 MMIO"——后者会量出
//:   5622/6809 ms 那种假数（交接文档 §4 第 8 条）。

//: u-dma-buf：PS 把整帧放进 DDR，PL 走 HP0 自己按行读。
struct Udmabuf {
    uint8_t* p = nullptr;
    uint64_t phys = 0;
    size_t   size = 0;

    bool open(const char* dev = "/dev/udmabuf0",
              const char* phys_path = "/sys/class/u-dma-buf/udmabuf0/phys_addr",
              const char* size_path = "/sys/class/u-dma-buf/udmabuf0/size") {
        if (FILE* f = std::fopen(phys_path, "r")) {
            if (std::fscanf(f, "%llx", reinterpret_cast<unsigned long long*>(&phys)) != 1) phys = 0;
            std::fclose(f);
        }
        if (FILE* f = std::fopen(size_path, "r")) {
            unsigned long long v = 0;
            if (std::fscanf(f, "%llu", &v) == 1) size = static_cast<size_t>(v);
            std::fclose(f);
        }
        if (phys == 0 || size == 0) {
            std::fprintf(stderr, "pl_vision: 读不到 %s / %s\n", phys_path, size_path);
            return false;
        }
        const int fd = ::open(dev, O_RDWR | O_SYNC);
        if (fd < 0) { std::perror(dev); return false; }
        void* m = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) { std::perror("mmap udmabuf"); return false; }
        p = static_cast<uint8_t*>(m);
        return true;
    }
    //: sync_mode=1：写完必须显式告诉驱动"这段交给设备"，否则 PL 可能读旧 DRAM。
    void sync_for_device() const {
        if (FILE* f = std::fopen("/sys/class/u-dma-buf/udmabuf0/sync_for_device", "w")) {
            std::fputs("0\n", f);
            std::fclose(f);
        }
    }
};

//: 从 quad **现算** 13 个正射系数。
//: 与 `core/fpga/tools/pl_p0_probe.cpp::calc_coefs`、`tools/ortho_fixed_ref.cpp`
//: 是同一串式子。**禁止手抄常数**：换标定/换场景时手抄的那份不会跟着变，而它错
//: 一位的表现是"整幅图整体偏一点点"——看起来像相机问题，最难查。
//: 输入是**本帧实际使用的 quad**（`Rectifier::quad_used`），所以万一 quad 被重新
//: 检测到，系数会跟着更新（正常情况只在开机写一次）。
static bool calc_coefs(const vision_board::field_frame::Quad& fq,
                       int64_t out[13], int* c0, int* c1) {
    if (!fq.ok) return false;
    auto H = rectify::invert(vision_board::field_frame::homography(fq, kPageW, kPageH));
    if (!H.ok) {
        std::fprintf(stderr, "pl_vision: 单应求逆失败\n");
        return false;
    }
    double h[9];
    for (int i = 0; i < 9; ++i) h[i] = H.h[i] / H.h[8];
    auto q16 = [](double x) { return static_cast<int64_t>(std::llround(x * 65536.0)); };
    auto q24 = [](double x) { return static_cast<int64_t>(std::llround(x * 16777216.0)); };
    //: 与 sample_rgb 里那对 (ca, cb) 是同一串：px = ca*c + cb
    const double ca = 1.0 / kFitScale;
    const double cb = (0.5 - kOffX) / kFitScale;
    out[0] = q16(h[0]); out[1] = q16(h[1]); out[2] = q16(h[2]);
    out[3] = q16(h[3]); out[4] = q16(h[4]); out[5] = q16(h[5]);
    out[6] = q24(h[6]); out[7] = q24(h[7]); out[8] = q24(h[8]);
    out[9]  = q16((kPlH - 0.5) * ca);    // PY0_16
    out[10] = q16(ca);                   // PS_16
    out[11] = q24((kPlH - 0.5) * ca);    // PY0_24
    out[12] = q24(ca);                   // PS_24
    //: 列范围门控（与 oracle 同一串整数）：px = ca*c + cb ∈ (0, kPageW]
    *c0 = kPlW;
    *c1 = 0;
    for (int c = 0; c < kPlW; ++c) {
        const double px = ca * c + cb;
        if (px > 0.0 && px <= kPageW) { if (c < *c0) *c0 = c; *c1 = c + 1; }
    }
    return true;
}

//: P0 的一帧：搬帧 → start → 等 busy 落 → 读 FLEN 字节。
struct P0Path {
    PlRegs* pl = nullptr;
    Udmabuf buf;

    //: **只读探针**（`--dbg-frm <path>`）：起跑后按固定间隔读 FRMDBG0/1/2/FRMLINE，
    //: 用来判断"正射有没有按正确的相机列采样"。**完全不写任何寄存器**，
    //: 唯一的副作用是多花了点 MMIO 时间（每帧几十 µs 级）。
    std::FILE* dbg_fp = nullptr;
    int        dbg_n = 0;            //: 最多采几拍（0 = 关闭）
    double     dbg_period_us = 2.5;   //: 采样间隔（µs）；2 ms 内 800 拍

    int64_t coefs[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int     c0 = 0, c1 = 0;
    bool    have_coefs = false;
    int     n_coef_writes = 0;      //: 系数重写次数（正常 = 1；>1 说明 quad 变过）

    bool init(PlRegs& regs) {
        pl = &regs;
        if (!buf.open()) return false;
        const size_t need = static_cast<size_t>(kCamW) * kCamH * 3;
        if (buf.size < need) {
            std::fprintf(stderr, "pl_vision: udmabuf 只有 %zu B，装不下 %zu B 的相机帧\n",
                         buf.size, need);
            return false;
        }
        if (buf.phys & 3u) {
            std::fprintf(stderr, "pl_vision: udmabuf phys 0x%llx 不是 4 字节对齐\n",
                         static_cast<unsigned long long>(buf.phys));
            return false;
        }
        //: FRMBASE 只在开机写一次（udmabuf 的 phys 不会变）。**必须回读**：
        //: 板上第一次跑就是 c0/c1 没写进去 → band 恒真 → 整帧发黑 → 4.62 ms 就
        //: "跑完"一帧，帧长/CRC 全自洽，一个像素都没取。
        const uint32_t phys32 = static_cast<uint32_t>(buf.phys);
        pl->wr(vision_board::kOffFrmBase, phys32);
        const uint32_t back = pl->rd(vision_board::kOffFrmBase);
        std::fprintf(stderr, "pl_vision: P0 udmabuf phys=0x%llx size=%zu MB，FRMBASE 回读=0x%08x%s\n",
                     static_cast<unsigned long long>(buf.phys), buf.size >> 20, back,
                     back == phys32 ? "" : " **不符**");
        return back == phys32;
    }

    bool set_coefs(const vision_board::field_frame::Quad& fq) {
        int64_t c[13];
        int a = 0, b = 0;
        if (!calc_coefs(fq, c, &a, &b)) return false;
        bool same = have_coefs && a == c0 && b == c1;
        for (int i = 0; i < 13 && same; ++i) if (c[i] != coefs[i]) same = false;
        if (same) return true;
        std::memcpy(coefs, c, sizeof c);
        c0 = a;
        c1 = b;
        have_coefs = true;
        for (int i = 0; i < 9; ++i)
            pl->wr(vision_board::kOffFrmH0 + 4u * static_cast<uint32_t>(i),
                   static_cast<uint32_t>(coefs[i]));
        pl->wr(vision_board::kOffFrmPy16, static_cast<uint32_t>(coefs[9]));
        pl->wr(vision_board::kOffFrmPs16, static_cast<uint32_t>(coefs[10]));
        pl->wr(vision_board::kOffFrmPy24L,
               static_cast<uint32_t>(static_cast<uint64_t>(coefs[11]) & 0xFFFFFFFFull));
        pl->wr(vision_board::kOffFrmPy24H,
               static_cast<uint32_t>((static_cast<uint64_t>(coefs[11]) >> 32) & 0xFFull));
        pl->wr(vision_board::kOffFrmPs24, static_cast<uint32_t>(coefs[12]));
        pl->wr(vision_board::kOffFrmC0C1,
               (static_cast<uint32_t>(c1) << 16) | static_cast<uint32_t>(c0));
        pl->wr(vision_board::kOffFrmR0R1, (static_cast<uint32_t>(kPlH - 1) << 16) | 0u);
        ++n_coef_writes;
        if (n_coef_writes == 1) {
            std::fprintf(stderr,
                         "pl_vision: P0 系数（quad 现算）h0..h8=%lld %lld %lld %lld %lld %lld"
                         " %lld %lld %lld PY16=%lld PS16=%lld PY24=%lld PS24=%lld c0..c1=%d..%d\n",
                         static_cast<long long>(coefs[0]), static_cast<long long>(coefs[1]),
                         static_cast<long long>(coefs[2]), static_cast<long long>(coefs[3]),
                         static_cast<long long>(coefs[4]), static_cast<long long>(coefs[5]),
                         static_cast<long long>(coefs[6]), static_cast<long long>(coefs[7]),
                         static_cast<long long>(coefs[8]), static_cast<long long>(coefs[9]),
                         static_cast<long long>(coefs[10]), static_cast<long long>(coefs[11]),
                         static_cast<long long>(coefs[12]), c0, c1);
            std::fprintf(stderr,
                         "pl_vision: P0 系数回读 C0C1=0x%08x（期望 0x%08x）PY24L=0x%08x"
                         "（期望 0x%08x）R0R1=0x%08x\n",
                         pl->rd(vision_board::kOffFrmC0C1),
                         (static_cast<uint32_t>(c1) << 16) | static_cast<uint32_t>(c0),
                         pl->rd(vision_board::kOffFrmPy24L),
                         static_cast<uint32_t>(static_cast<uint64_t>(coefs[11]) & 0xFFFFFFFFull),
                         pl->rd(vision_board::kOffFrmR0R1));
        }
        return true;
    }

    //: 起跑前：等 vision_top 空闲（busy=0）。
    bool wait_idle(double limit_ms, const char* when = "") const {
        const double t0 = now_ms();
        while (pl->rd(vision_board::kOffCtrl) & vision_board::kStatusBusy) {
            if (now_ms() - t0 > limit_ms) {
                std::fprintf(stderr, "pl_vision: %s等 vision_top 空闲超时 %.0f ms（CTRL=0x%08x）\n",
                             when, limit_ms, pl->rd(vision_board::kOffCtrl));
                return false;
            }
        }
        return true;
    }
    //: 起跑后：先确认 busy 真的拉起过，再等它落下。
    //: 只看"busy 落"是不够的：start 没生效时 busy 一直是 0，会被当成"瞬间完成"。
    bool wait_done(double limit_ms) const {
        const double t0 = now_ms();
        bool seen = false;
        for (;;) {
            const uint32_t c = pl->rd(vision_board::kOffCtrl);
            const bool busy = (c & vision_board::kStatusBusy) != 0;
            if (busy) {
                seen = true;
            } else if (seen) {
                return true;
            } else if (now_ms() - t0 > 50.0) {
                std::fprintf(stderr, "pl_vision: start 之后 50 ms 内没见 busy 拉起（CTRL=0x%08x）\n",
                             c);
                return false;
            }
            if (now_ms() - t0 > limit_ms) {
                std::fprintf(stderr, "pl_vision: 等 vision_top 完成超时 %.0f ms（CTRL=0x%08x）\n",
                             limit_ms, c);
                return false;
            }
        }
    }

    //: 跑一帧。`raw` 非空时把几何帧原始字节原样带出（逐字节对拍用）。
    bool run(const uint8_t* cam, size_t bytes, uint32_t seq, uint64_t stamp_ns,
             vision_board::GeometryFrame& out, std::vector<uint8_t>* raw,
             double* t_copy, double* t_all, uint32_t* flen_out, uint32_t* fills_out) {
        const double t0 = now_ms();
        if (!wait_idle(200.0, "起跑前")) return false;
        if (bytes > buf.size) {
            std::fprintf(stderr, "pl_vision: 帧 %zu B 超过 udmabuf %zu B\n", bytes, buf.size);
            return false;
        }
        std::memcpy(buf.p, cam, bytes);
        buf.sync_for_device();
        if (t_copy) *t_copy = now_ms() - t0;

        pl->wr(vision_board::kOffSeq, seq);
        pl->wr(vision_board::kOffStamp0, static_cast<uint32_t>(stamp_ns));
        pl->wr(vision_board::kOffStamp1, static_cast<uint32_t>(stamp_ns >> 32));
        pl->wr(vision_board::kOffFrmCtl, vision_board::kFrmSrc | vision_board::kFrmClrDone);
        pl->wr(vision_board::kOffFrmCtl, vision_board::kFrmSrc | vision_board::kFrmStart);
        if (dbg_fp && dbg_n > 0) {
            //: 只读采样：正射的 walker 在跑，把它的 (src_line, src_px) 轨迹录下来。
            //: 判据见 tools/PL 文档：px 应当扫过"页面 0..640 对应的相机列"。
            const double t_p0 = now_ms();
            double t_next = t_p0;
            int i = 0;
            while (i < dbg_n) {
                const uint32_t d0 = pl->rd(vision_board::kOffFrmDbg0);
                const uint32_t d1 = pl->rd(vision_board::kOffFrmDbg1);
                const uint32_t d2 = pl->rd(vision_board::kOffFrmDbg2);
                const uint32_t dl = pl->rd(vision_board::kOffFrmLine);
                const uint32_t dd = pl->rd(vision_board::kOffFrmDiag);
                std::fprintf(dbg_fp,
                             "{\"i\":%d,\"t_ms\":%.3f,\"d0\":%u,\"d1\":%u,\"d2\":%u,"
                             "\"dl\":%u,\"dd\":%u}\n",
                             i, now_ms() - t_p0, d0, d1, d2, dl, dd);
                ++i;
                const double now2 = now_ms();
                if ((now2 - t_p0) > 2.0) break;       //: 只探前 2 ms（一帧 22 ms），不许占掉整帧
                const double gap = dbg_period_us / 1000.0;
                while (now_ms() - t_next < gap) { /* busy */ }
                t_next += gap;
                if (t_next < now2) t_next = now2 + gap;
            }
            std::fflush(dbg_fp);
        }
        if (!wait_done(500.0)) return false;
        if (t_all) *t_all = now_ms() - t0;

        if (fills_out) *fills_out = pl->rd(vision_board::kOffFrmCtl) >> vision_board::kFrmFillsShift;
        const uint32_t flen = pl->rd(vision_board::kOffFLen);
        if (flen_out) *flen_out = flen;
        if (flen < 76u || flen > vision_board::kFrameWindowBytes) {
            std::fprintf(stderr, "pl_vision: FLEN=%u 不合理（窗口 %u）\n", flen,
                         vision_board::kFrameWindowBytes);
            return false;
        }
        //: 窗口：**逐字节**读到普通内存再解析（device 内存上的宽/非对齐读会 SIGBUS）
        static std::vector<uint8_t> win;
        win.resize(flen);
        const volatile uint8_t* src =
            static_cast<const volatile uint8_t*>(pl->map) + vision_board::kOffFrame;
        for (uint32_t i = 0; i < flen; ++i) win[i] = src[i];

        auto le32 = [&](uint32_t off) {
            return static_cast<uint32_t>(win[off]) |
                   (static_cast<uint32_t>(win[off + 1]) << 8) |
                   (static_cast<uint32_t>(win[off + 2]) << 16) |
                   (static_cast<uint32_t>(win[off + 3]) << 24);
        };
        const uint32_t n_obs = le32(12), n_gaps = le32(16);
        const size_t need = 76u + 36u * (static_cast<size_t>(n_obs) + n_gaps);
        if (need > flen) {
            std::fprintf(stderr, "pl_vision: FLEN=%u 装不下声明的 %u obs + %u gaps（需 %zu B）\n",
                         flen, n_obs, n_gaps, need);
            return false;
        }
        if (need != flen) {
            std::fprintf(stderr, "pl_vision: 帧长自洽性告警 FLEN=%u 但 76+36*(%u+%u)=%zu\n",
                         flen, n_obs, n_gaps, need);
        }
        try {
            const std::vector<uint8_t> body(win.begin(), win.begin() + static_cast<long>(need));
            if (raw) *raw = body;
            out = vision_board::parse_geometry_frame(body);   // 长度/magic/版本/CRC 四道校验
        } catch (const std::exception& e) {
            std::fprintf(stderr, "pl_vision: 几何帧解析失败：%s\n", e.what());
            return false;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// 正射：页面 640×360 ← 相机 640×480（逆映射 + 双线性）
// ---------------------------------------------------------------------------
static bool g_nearest = false;      //: 正射采样方式：false=双线性，true=最近邻（测速用）
static bool g_cam_domain = true;    //: 是否做"相机域适配"（见 camera_domain_adapt）

struct Rectifier {
    //: 页面(640×480, y↑) → 相机像素。**与 vt_sparse 用同一个**求解器与目标尺寸。
    rectify::Homography page_to_cam;
    bool ok = false;

    //: quad 8 个数：TL,TR,BR,BL（相机 640×480 像素）。解析规则与 vt_sparse 一致
    //: （越界/面积过小就当没锁到，退回 `field_frame::find` 自己找白框）。
    void load(const std::string& path, const uint8_t* cam, int bw, int bh) {
        vision_board::field_frame::Quad fq;
        bool ext = false;
        if (!path.empty()) {
            if (FILE* xf = std::fopen(path.c_str(), "r")) {
                double v[8] = {0};
                if (std::fscanf(xf, "%lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3],
                                &v[4], &v[5], &v[6], &v[7]) == 8) {
                    bool sane = true;
                    double area = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        const double px = v[2 * i], py = v[2 * i + 1];
                        if (px < -8 || py < -8 || px > bw + 8 || py > bh + 8) sane = false;
                        const int j = (i + 1) % 4;
                        area += px * v[2 * j + 1] - v[2 * j] * py;
                    }
                    if (std::fabs(area) * 0.5 < 0.10 * bw * bh) sane = false;
                    if (sane) {
                        for (int i = 0; i < 4; ++i) {
                            fq.p[i][0] = v[2 * i];
                            fq.p[i][1] = v[2 * i + 1];
                        }
                        fq.ok = true;
                        ext = true;
                    }
                }
                std::fclose(xf);
            }
        }
        if (!fq.ok) fq = vision_board::field_frame::find(cam, bw, bh);
        quad_ext = ext;
        quad_used = fq;
        if (!fq.ok) return;
        //: ⚠ `field_frame::homography()` 的方向是 **相机图 → 场地**（solve(img, world)），
        //: 要"页面 → 相机"必须再倒一次 —— 与 `vt_sparse` 里的 `invert(H)` 同一条路。
        //: 这一处方向搞反的后果：能出图、能出角色，但墙的位置与 vt_sparse 差一大截
        //: （实测同一帧：墙间隔 68 行 vs 期望 124 行）。
        page_to_cam = rectify::invert(vision_board::field_frame::homography(fq, kPageW, kPageH));
        ok = page_to_cam.ok;
    }
    bool quad_ext = false;
    vision_board::field_frame::Quad quad_used;   //: 本帧实际使用的四边形（写 quad-state 用）

    //: 页面 (x, y↑) → 相机 (u, v) → 双线性采样。
    //:
    //: 输出图 = "把页面按 kFitScale 等比放进 640×360 后**正向**采样"：
    //: 输出像素 (c, r) 对应页面点 px=(c+0.5-kOffX)/kFitScale、py=(kPlH-r-0.5-kOffY)/kFitScale，
    //: 再用 page→cam 单应拿到相机坐标做双线性。页面以外（左右 80 px 边带）= 黑，
    //: 与页面渲染时的场地外底色一致。
    //:
    //: 性能：单应 (u,v) 共用同一个分母，而 px 对列号 c 是**一次函数**，所以
    //:   每行只有 6 个系数（u/v 的分子、分母各 2 个），每像素只需 3 次乘加 + **1 次除法**
    //:   （求分母倒数后 u/v 各一次乘法）。逐像素调 `Homography::apply` 要两次除法，
    //:   而且在 -O2 下 230400 像素实测 47 ms，明显不划算。
    inline void sample_rgb(const uint8_t* cam, int cw, int ch, uint8_t* dst) const {
        std::memset(dst, 0, static_cast<size_t>(kPlW) * kPlH * 3);

        //: 列范围与行无关，提到循环外（`px` 只跟列号有关）：
        //:   只保留 px ∈ (0, kPageW] 的列 —— 页面左右各有 kOffX 宽的边带（黑）。
        const double ca = 1.0 / kFitScale;                       // px = ca*c + cb
        const double cb = (0.5 - kOffX) / kFitScale;
        int c0 = kPlW, c1 = 0;
        for (int c = 0; c < kPlW; ++c) {
            const double px = ca * c + cb;
            if (px > 0.0 && px <= kPageW) { if (c < c0) c0 = c; c1 = c + 1; }
        }

        const double* H = page_to_cam.h.data();
        //: 单精度：Cortex-A9 的 VFP **没有** double 硬件除法（软件模拟 ~100+ 拍），
        //: 而有 vdiv.f32。坐标上限 640、双线性权重到 1e-3 足够，所以逐像素一律用 float。
        const float cw_f = static_cast<float>(cw), ch_f = static_cast<float>(ch);
        for (int r = 0; r < kPlH; ++r) {
            const double py = (kPlH - (r + 0.5) - kOffY) / kFitScale;
            if (py <= 0.0 || py > kPageH) continue;              // 页面外（上下边）
            //: 每行 6 个系数（分子/分母对列号都是一次函数）
            const float au = static_cast<float>(H[0] * ca);
            const float bu = static_cast<float>(H[0] * cb + H[1] * py + H[2]);
            const float av = static_cast<float>(H[3] * ca);
            const float bv = static_cast<float>(H[3] * cb + H[4] * py + H[5]);
            const float ad = static_cast<float>(H[6] * ca);
            const float bd = static_cast<float>(H[6] * cb + H[7] * py + H[8]);
            uint8_t* row = dst + static_cast<size_t>(r) * kPlW * 3;
            for (int c = c0; c < c1; ++c) {
                const float cf = static_cast<float>(c);
                const float d = ad * cf + bd;
                if (d > -1e-6f && d < 1e-6f) continue;
                const float inv_d = 1.0f / d;
                float u = (au * cf + bu) * inv_d - 0.5f;
                float v = (av * cf + bv) * inv_d - 0.5f;
                u = u < 0.0f ? 0.0f : (u > cw_f - 1.001f ? cw_f - 1.001f : u);
                v = v < 0.0f ? 0.0f : (v > ch_f - 1.001f ? ch_f - 1.001f : v);
                const int xi = static_cast<int>(u), yi = static_cast<int>(v);
                const float fx = u - static_cast<float>(xi), fy = v - static_cast<float>(yi);
                const int xi1 = xi + 1 < cw ? xi + 1 : cw - 1;
                const int yi1 = yi + 1 < ch ? yi + 1 : ch - 1;
                if (g_nearest) {
                    const uint8_t* p = cam + (static_cast<size_t>(yi) * cw + xi) * 3;
                    uint8_t* out = row + static_cast<size_t>(c) * 3;
                    out[0] = p[0]; out[1] = p[1]; out[2] = p[2];
                    continue;
                }
                const uint8_t* p00 = cam + (static_cast<size_t>(yi) * cw + xi) * 3;
                const uint8_t* p10 = cam + (static_cast<size_t>(yi) * cw + xi1) * 3;
                const uint8_t* p01 = cam + (static_cast<size_t>(yi1) * cw + xi) * 3;
                const uint8_t* p11 = cam + (static_cast<size_t>(yi1) * cw + xi1) * 3;
                uint8_t* out = row + static_cast<size_t>(c) * 3;
                for (int k = 0; k < 3; ++k) {
                    const float top = p00[k] + (p10[k] - p00[k]) * fx;
                    const float bot = p01[k] + (p11[k] - p01[k]) * fx;
                    out[k] = static_cast<uint8_t>(top + (bot - top) * fy + 0.5f);
                }
            }
        }

    }
};

// ---------------------------------------------------------------------------
// 相机域适配：亮度 → 判据域
//
// 为什么需要（2026-10-25 实测）：PL 的判据是从**屏幕录像**语料标定出来的 —— 墙是
// 暖白 (255,240,200)、角色纯白 (255,255,255)，靠 R-B>40 区分。经相机后：
//   · 墙被拍成中性偏蓝：R-B 从 +55 变成 **-5**（饱和度调到 100 反而更负，-37），
//     颜色信息**不可恢复**；
//   · 整幅对比度被压：墙 100~175、场地 ≈3、角色 ≥195，且右侧有暗角，
//     导致判据的"场地范围"只测到半幅（span=[80,364] 而不是 [80,559]）。
// 做法（三步，参数由实测电平给出，不是拟合）：
//   ① 线性拉伸 [kDark, kEdge] → [0,255]：把相机压缩的动态范围还原，
//      并压掉暗角造成的电平差（这一步把 span 恢复成全幅）；
//   ② 亮到 ≥kKeep 的像素保持**中性**（角色是纯白）；
//   ③ 中间亮度按"暖白"重建颜色（R=G=v, B=v-kTint），让墙重新满足 R-B>40。
// 判据本身一个字节都没改，改的只是"喂进去的画面"。
// ---------------------------------------------------------------------------
// 相机域适配：把相机画面"翻译"成判据认得的形态
//
// 为什么需要（实测）：PL 的判据来自**屏幕录像**语料 —— 墙是暖白 (255,240,200)、
// 角色纯白 (255,255,255)，靠 `R-B>40` 区分。经相机后：
//   · 墙被拍成中性偏蓝（R-B 从 +55 → **-5**；饱和度调到 100 反而 -37），
//     颜色信息不可恢复；
//   · 墙的亮度 ≈198、角色 ≈200~255、场地 ≈3、页面白框 ≈255 —— **亮度也不可分**，
//     只差 2 个灰阶（试过按亮度分档：角色被判成墙、或墙被判成角色，两头都踩）。
//
// 唯一可靠的区别是**形状**：墙是长水平条（几十~几百像素），角色是 ~20 px 的亮斑。
// 所以这里按形状重建颜色：
//   ① 每行找"亮像素的连续段"，长度 ≥ kRunMin 的段 → 判为墙 → 染暖白（R-B=kTint）；
//   ② 其它亮像素（短段，通常是角色/反光点）→ 抬到 kBrightLevel 并保持**中性**
//      （判据的 bright = min>195 且 spread<45）；
//   ③ 暗像素原样保留（场地/缺口）。
// 判据本身一个字节没改，改的只是"喂进去的画面"。
//: 阈值 150 的依据（8 帧实测）：墙的亮度稳定在 **196~202**；场地中位 26~32，但
//: p90≈70~80、个别帧 p99 到 178（反光）。原先取 40 —— 把场地大片像素算成"亮"，
//: f1/f2 整幅被染成一堵墙（障碍 hw=320 = 满宽，缺口一个都出不来）。
constexpr int kAdaptLvl = 150;      // ≥ 它的像素才算"亮"（墙或角色）
constexpr int kAdaptDark = kAdaptLvl;   // 兼容旧名
constexpr int kAdaptRunMin = 40;    // 水平连续亮段至少这么长才算"墙"
constexpr int kAdaptTint = 60;      // 暖白重建的 R-B 差（判据要 >40）
constexpr int kAdaptBright = 210;   // 非墙亮斑（角色）抬到的亮度（>195 才算 bright）

inline void camera_domain_adapt(uint8_t* img, int w, int h) {
    std::vector<uint8_t> gray(static_cast<size_t>(w) * h);
    for (size_t i = 0, n = gray.size(); i < n; ++i) {
        const uint8_t* p = img + i * 3;
        gray[i] = static_cast<uint8_t>((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
    }
    std::vector<uint8_t> is_wall(static_cast<size_t>(w) * h, 0);
    for (int r = 0; r < h; ++r) {
        const size_t base = static_cast<size_t>(r) * w;
        int c = 0;
        while (c < w) {
            if (gray[base + c] < kAdaptDark) { ++c; continue; }
            int e = c;
            while (e < w && gray[base + e] >= kAdaptDark) ++e;
            if (e - c >= kAdaptRunMin)
                for (int k = c; k < e; ++k) is_wall[base + k] = 1;
            c = e;
        }
    }
    for (size_t i = 0, n = gray.size(); i < n; ++i) {
        uint8_t* p = img + i * 3;
        const int v = gray[i];
        if (is_wall[i]) {
            p[0] = p[1] = p[2] = static_cast<uint8_t>(v);
            p[2] = static_cast<uint8_t>(v > kAdaptTint ? v - kAdaptTint : 0);
        } else if (v >= kAdaptDark) {
            //: 短亮段：角色/反光点。抬到 kBrightLevel 且保持中性，让 bright 判据认出来。
            p[0] = p[1] = p[2] = static_cast<uint8_t>(kAdaptBright);
        } else {
            p[0] = p[1] = p[2] = static_cast<uint8_t>(v);
        }
    }
}

//: PL 输出 → 页面坐标（640×480，y↑）。**唯一的一处换算**，与上面的摆放严格互逆：
//:   x_page = (x_pl - kOffX) / kFitScale
//:   y_page = (kPlH - y_pl - kOffY) / kFitScale
//: ⚠ **仅在硅上出现的固定列偏移补偿**（默认 0，不改基线行为）。
//: 取证：同帧同 quad 同系数，Verilator 对齐、硅上整幅右移 +37 列（宽度不变、3 帧不累积）。
//: 根因未定位（已排除 quad/正射/camadapt 帧首/DDR 延迟/out_sof 次数/位流过期），
//: 这里只做**可关闭的**运行态补偿，供闭环验证用；默认 0 = 与原来完全一致。
static double g_x_shift_cols = 0.0;      //: 硅上实测 +37
inline double x_to_page_x(double x_pl) { return (x_pl - kOffX) / kFitScale; }
//: vt.jsonl 里的 x 一律走这里（带上补偿）
inline double x_to_page_corrected(double x_pl) {
    return x_to_page_x(x_pl - g_x_shift_cols);
}
inline double row_to_page_y(double y_pl_row) { return (kPlH - y_pl_row - kOffY) / kFitScale; }

//: 读 vision_top 的 row_count[row]（dbg sel=3）。这是 PL 眼里"这一行有多少墙像素"，
//: 用来把"板端与本地逐字节不一致"变成可比较的数字（上一轮记的是 229 vs 197）。
static uint32_t dbg_row_count(PlRegs& pl, uint32_t row) {
    pl.wr(vision_board::kOffDbgSel, vision_board::pack_dbg_sel(3, row, 0));
    return pl.rd(vision_board::kOffDbgData);
}

}  // namespace

int main(int argc, char** argv) {
    std::string quad_file, jsonl, quad_state, dump_ppm, dump_frame, dump_meta;
    std::string dbg_frm;      //: --dbg-frm：只读探针（不写任何寄存器）
    int dbg_n = 800;          //: --dbg-n：最多采样拍数
    bool ortho_only = false;   //: 只做正射并落 PPM，不碰 PL（本机离线对拍用）
    int cw = kCamW, ch = kCamH, frames_limit = 0;
    bool stats = false;
    //: `--pl-param NAME=VALUE`：把阈值写进 PL 的配置寄存器（0x000 起，21 个，
    //: 顺序见 vision_regs.hpp / vision_pkg.vh）。留空 = 用 RTL 默认值。
    //: 为什么需要：相机的色彩响应与屏幕录像语料不同（墙被拍成中性灰），
    //: 所以某些阈值要按"相机域"实测后重设，而不是照抄语料。
    //: `--pl-param NAME=VALUE` 收集成字符串，启动时和默认表一起写（见下）。
    std::vector<std::pair<std::string, std::string>> pl_params_raw;

    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--w") cw = std::atoi(next().c_str());
        else if (k == "--h") ch = std::atoi(next().c_str());
        else if (k == "--quad-file") quad_file = next();
        else if (k == "--quad-state") quad_state = next();   // 收下不用（只读 quad）
        else if (k == "--jsonl") jsonl = next();
        else if (k == "--frames") frames_limit = std::atoi(next().c_str());
        else if (k == "--dump-ppm") dump_ppm = next();
        else if (k == "--dump-frame") dump_frame = next();   //: 把 PL 的原始几何帧落盘（逐字节对拍用）
        else if (k == "--dump-meta") dump_meta = next();     //: 每帧一行 json（FLEN/fills/row_count/耗时），300 帧统计用
        else if (k == "--stats") stats = true;
        else if (k == "--nearest") g_nearest = true;
        else if (k == "--ortho-only") ortho_only = true;   //: 同上
        else if (k == "--dbg-frm") dbg_frm = next();       //: 只读探针落盘（jsonl）
        else if (k == "--dbg-n") dbg_n = std::atoi(next().c_str());
        else if (k == "--x-shift-cols") g_x_shift_cols = std::atof(next().c_str());
        else if (k == "--no-camera-domain") g_cam_domain = false;
        else if (k == "--pl-param") {
            const std::string kv = next();
            const auto eq = kv.find('=');
            if (eq == std::string::npos) {
                std::fprintf(stderr, "pl_vision: --pl-param 要用 NAME=VALUE 形式：%s\n", kv.c_str());
                return 2;
            }
            pl_params_raw.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
        }
        else if (k == "--frame-roi") { next(); next(); next(); next(); }   // 与 vt_sparse 对齐，忽略
    }
    if (cw != kCamW || ch != kCamH) {
        std::fprintf(stderr, "pl_vision: 只支持 %d×%d 输入（PL 的页面采样固定 %d×%d），收到 %d×%d\n",
                     kCamW, kCamH, kPlW, kPlH, cw, ch);
        return 2;
    }

    PlRegs pl;
    if (!pl.open() && !ortho_only) return 1;
    //: `--quad-state` 落盘：闭环启动后会读 /tmp/qb.txt 与开跑时的指纹比（--quad-tol），
    //: 判断"相机或页面窗口有没有被动过"。vt_sparse 每帧写它，这里也必须写，
    //: 否则闭环会看到一个旧指纹而拒跑/误判。写的就是本帧实际使用的 quad。
    auto write_quad_state = [&](const vision_board::field_frame::Quad& q) {
        if (quad_state.empty()) return;
        FILE* f = std::fopen(quad_state.c_str(), "w");
        if (!f) return;
        for (int i = 0; i < 4; ++i) std::fprintf(f, "%.2f %.2f ", q.p[i][0], q.p[i][1]);
        std::fputc('\n', f);
        std::fclose(f);
    };

    // ---- 阈值：**开机把 21 个配置寄存器全写一遍**（0x000 + 4*i）-----------
    //
    //: 为什么必须全写，而不是只写要覆盖的那几个：PL 的配置寄存器**不清零、会粘住**。
    //: 上一次运行留下的 `wall_rb=0` 会让"中性亮像素"（角色）也被算成墙，行带跟着变，
    //: 于是同一条流水线在两张一样的图上给出不同的墙 —— 实测踩过：PL 报墙 y=356，
    //: 参考报 381.6（同一张图、同一像素）。
    //: 表里的默认值必须与 `vision_ref.c` 的 `VR_DEFAULT_*` 逐项一致（契约测试会核对）。
    struct PlParam { const char* name; uint32_t def; };
    static const PlParam kParams[] = {
        {"wall_rb", 40}, {"wall_min_r", 110}, {"wall_min_g", 60},
        {"bright_min", 195}, {"bright_spread", 45}, {"green_gm", 10}, {"green_min", 90},
        {"band_row_pct", 5}, {"band_join_rows", 4}, {"band_min_rows", 5},
        {"field_col_min", 5}, {"wall_col_pct", 25}, {"wall_col_min", 2},
        {"edge_min", 12}, {"edge_min_cols", 1}, {"min_seg_px", 8}, {"min_gap_px", 8},
        {"player_min_area", 60}, {"player_max_area", 1500},
        {"player_min_fill_pct", 45}, {"player_max_aspect_pct", 160},
    };
    static const int kNParams = static_cast<int>(sizeof(kParams) / sizeof(kParams[0]));
    std::map<std::string, std::string> overrides;
    for (const auto& kv : pl_params_raw) {
        overrides[kv.first] = kv.second;
    }
    for (const auto& kv : overrides) {
        bool known = false;
        for (int i = 0; i < kNParams; ++i) if (kv.first == kParams[i].name) known = true;
        if (!known) {
            std::fprintf(stderr, "pl_vision: 不认识的 PL 参数 %s\n", kv.first.c_str());
            return 2;
        }
    }
    for (int i = 0; !ortho_only && i < kNParams; ++i) {
        uint32_t v = kParams[i].def;
        const auto it = overrides.find(kParams[i].name);
        if (it != overrides.end()) v = static_cast<uint32_t>(std::atoi(it->second.c_str()));
        pl.wr(vision_board::kOffConfig + 4u * static_cast<uint32_t>(i), v);
    }
    if (!ortho_only)
    std::fprintf(stderr, "pl_vision: 已写 %d 个 PL 参数%s\n", kNParams,
                 overrides.empty() ? "（全默认）" : "（含命令行覆盖）");

    //: P1a 的 8 个亮度参数（cfg 21..28）**不进** kParams（那张表是"21 个检测参数"，
    //: 有测试钉住数量），但它们的寄存器同样**会粘住**：上一次谁开过 luma_mode=1，
    //: 这一次就静默继承一套完全不同的判据。所以按 `vision_ref.c` 的
    //: VR_DEFAULT_REF_PARAMS 把默认值显式写一遍（默认全是"关"）。
    if (!ortho_only) {
        static const uint32_t kLumaDef[] = {
            0u,    // 21 luma_mode      = 0（RGB 判据，与旧版逐位一致）
            88u,   // 22 luma_pct
            20u,   // 23 luma_bias
            90u,   // 24 luma_min
            200u,  // 25 luma_max
            165u,  // 26 luma_player_min
            8u,    // 27 luma_edge_rows
            40u,   // 28 luma_run_min
        };
        static_assert(sizeof(kLumaDef) / sizeof(kLumaDef[0]) ==
                          vision_board::kCfgLumaRunMin - vision_board::kCfgLumaMode + 1,
                      "luma 参数个数必须与 cfg 21..28 对齐");
        for (uint32_t i = 0; i < sizeof(kLumaDef) / sizeof(kLumaDef[0]); ++i)
            pl.wr(vision_board::kOffConfig + 4u * (vision_board::kCfgLumaMode + i), kLumaDef[i]);
        std::fprintf(stderr, "pl_vision: 已写 8 个 luma 参数（全默认 = 关）\n");
    }

    // ---- P0：udmabuf + FRMBASE（开机一次）--------------------------------
    P0Path p0;
    if (!ortho_only && !p0.init(pl)) return 1;
    if (!ortho_only && !dbg_frm.empty()) {
        p0.dbg_fp = std::fopen(dbg_frm.c_str(), "w");
        p0.dbg_n = dbg_n;
        if (!p0.dbg_fp) {
            std::fprintf(stderr, "pl_vision: --dbg-frm 打不开 %s\n", dbg_frm.c_str());
        } else {
            std::fprintf(stderr, "pl_vision: 只读探针 → %s（最多 %d 拍）\n",
                         dbg_frm.c_str(), p0.dbg_n);
        }
    }

    //: `--dump-meta`：每帧一行 json，给"300 帧一致率"与"已知偏差分布"取证用。
    FILE* meta_f = nullptr;
    if (!dump_meta.empty()) {
        meta_f = std::fopen(dump_meta.c_str(), "w");
        if (!meta_f) { std::perror(dump_meta.c_str()); return 1; }
        std::setvbuf(meta_f, nullptr, _IOLBF, 0);
    }

    FILE* jf = jsonl.empty() ? stdout : std::fopen(jsonl.c_str(), "w");
    if (!jf) { std::perror(jsonl.c_str()); return 1; }
    std::setvbuf(jf, nullptr, _IOLBF, 0);        // 行缓冲：下游按行读

    std::vector<uint8_t> cam(static_cast<size_t>(cw) * ch * 3);
    std::vector<uint8_t> page(static_cast<size_t>(kPlW) * kPlH * 3);
    Rectifier rect;
    long frame = 0;
    double sum_rect = 0, sum_push = 0, sum_all = 0;
    int n_bad = 0, n_retry = 0;

    while (frames_limit == 0 || frame < frames_limit) {
        // ---- 读一整帧（不足就是管道关了）----
        size_t got = 0;
        while (got < cam.size()) {
            const ssize_t n = ::read(0, cam.data() + got, cam.size() - got);
            if (n <= 0) {
                if (got == 0) goto done;              // 正常结束
                std::fprintf(stderr, "pl_vision: 帧读到一半管道断了（%zu/%zu）\n", got, cam.size());
                goto done;
            }
            got += static_cast<size_t>(n);
        }
        ++frame;

        const double t_stamp = now_ms() / 1000.0;
        rect.load(quad_file, cam.data(), cw, ch);
        if (rect.ok) write_quad_state(rect.quad_used);
        if (!rect.ok) {
            // 与 vt_sparse 一致：没锁到页面就报 frame_geom=0（下游据此暂停，不猜）
            std::fprintf(jf, "{\"frame\":%ld,\"t\":%.6f,\"frame_geom\":0,\"quad_ext\":%d,"
                             "\"walls\":[],\"player\":null,\"n\":0}\n",
                         frame, t_stamp, rect.quad_ext ? 1 : 0);
            continue;
        }
        //: ⚠ P0 模式下**正射与相机域适配都在 PL 里**：这里只有"要 dump 页面图"或
        //: `--ortho-only`（本机离线对拍 oracle）时才跑 PS 版软正射。否则不跑 ——
        //: 跑了既白付 25.7 ms，又把已经适配过的图再染一遍（墙的 R−B 会被减两次）。
        double t_rect = 0;
        if (ortho_only || !dump_ppm.empty()) {
            const double t_soft0 = now_ms();
            rect.sample_rgb(cam.data(), cw, ch, page.data());
            if (g_cam_domain) camera_domain_adapt(page.data(), kPlW, kPlH);
            t_rect = now_ms() - t_soft0;
        }

        if (!dump_ppm.empty()) {
            char path[512];
            std::snprintf(path, sizeof path, "%s_%05ld.ppm", dump_ppm.c_str(), frame);
            if (FILE* f = std::fopen(path, "wb")) {
                std::fprintf(f, "P6\n%d %d\n255\n", kPlW, kPlH);
                std::fwrite(page.data(), 1, page.size(), f);
                std::fclose(f);
            }
        }
        if (ortho_only) continue;          //: 只要正射图（本机对拍 oracle，不碰 PL）

        // ---- PL 判墙（P0：相机帧由 PL 自己从 DDR 取、自己正射）----
        vision_board::GeometryFrame gf;
        double t_push = 0, t_all = 0;
        uint32_t flen = 0, fills = 0;
        const uint64_t stamp_ns = static_cast<uint64_t>(t_stamp * 1e9);
        std::vector<uint8_t> raw_frame;
        //: 系数从**本帧实际用的 quad** 现算（禁止手抄）；正常情况下只在第一帧写一次。
        if (!p0.set_coefs(rect.quad_used)) {
            std::fprintf(stderr, "pl_vision: 从 quad 现算系数失败（跳过这一帧）\n");
            ++n_bad;
            continue;
        }
        bool frame_ok = false;
        for (int attempt = 0; attempt < 3 && !frame_ok; ++attempt) {
            frame_ok = p0.run(cam.data(), cam.size(), static_cast<uint32_t>(frame), stamp_ns,
                              gf, dump_frame.empty() ? nullptr : &raw_frame,
                              &t_push, &t_all, &flen, &fills);
            if (!frame_ok) ++n_retry;
        }
        if (!frame_ok) {
            ++n_bad;
            continue;
        }
        sum_rect += t_rect; sum_push += t_push; sum_all += t_all;
        //: row_count（PL 眼里的"每行墙像素数"）：把"板端与本地差多少"变成一行数字，
        //: 不用猜。上一轮记的已知偏差就是 row_count[359]=229 vs 本地 197。
        const uint32_t row_last = dbg_row_count(pl, static_cast<uint32_t>(kPlH - 1));
        const uint32_t fcount =
            (pl.rd(vision_board::kOffCtrl) >> vision_board::kStatusFrameCountShift) &
            vision_board::kStatusFrameCountMask;
        if (stats && frame % 20 == 0) {
            std::fprintf(stderr, "pl_vision: 帧 %ld 搬帧 %.2f ms / PL %.2f ms | FLEN=%u fills=%u"
                                 " frame_count=%u row_count[%d]=%u（n_obs=%u n_gaps=%u）\n",
                         frame, t_push, t_all, flen, fills, fcount, kPlH - 1, row_last,
                         gf.header.n_obs, gf.header.n_gaps);
        }
        if (meta_f) {
            std::fprintf(meta_f, "{\"frame\":%ld,\"flen\":%u,\"fills\":%u,\"frame_count\":%u,"
                                 "\"row_last\":%u,\"n_obs\":%u,\"n_gaps\":%u,\"ms\":%.3f}\n",
                         frame, flen, fills, fcount, row_last,
                         gf.header.n_obs, gf.header.n_gaps, t_all);
        }

        //: 原始几何帧落盘：与 `vision_ref <page.ppm> out.bin` 的输出逐字节比对，
        //: 这是"PL 的判据在相机域也与参考一致"的唯一硬证据（L0 的相机域版本）。
        if (!dump_frame.empty()) {
            char path[512];
            std::snprintf(path, sizeof path, "%s_%05ld.bin", dump_frame.c_str(), frame);
            if (FILE* f = std::fopen(path, "wb")) {
                std::fwrite(raw_frame.data(), 1, raw_frame.size(), f);
                std::fclose(f);
            }
        }

        // ---- 几何帧 → vt.jsonl（与 vt_sparse 同构）----------------------
        //: 墙：一条障碍 = 一行墙；它的 y 是 PL 的行号，换成页面 y（×480/360）。
        //: 缺口：PL 报的是 (cx, cy, width)，vt.jsonl 要 gl/gr —— gl=cx-w/2, gr=cx+w/2。
        //: 配对规则：每条缺口找"y 最近"的那条墙（PL 的缺口本来就带 cy，同一行）。
        std::fprintf(jf, "{\"frame\":%ld,\"t\":%.6f,\"frame_geom\":1,\"quad_ext\":%d,\"walls\":[",
                     frame, t_stamp, rect.quad_ext ? 1 : 0);
        //: 同一道墙可能被报出两个缺口（反光/模糊把墙切成三段，实测 f0：
        //: 一条 247..489（宽 242，明显过宽）与真缺口 239..362 重叠）。决策只认
        //: "这道墙的缺口在哪"，两条会让它以为墙有大段缺口 → 有穿墙风险。
        //: 处理：y 相差 ≤4 px 的缺口视为同一条墙，**保留较窄的那个**（真缺口
        //: 宽度 = 场景的 90~110 页面单位，过宽的那条是合成出来的）。
        //: 先去重（按缺口自己的 cy）……
        int emitted = 0;
        //: 再按**归属到同一道墙**去重：把每条缺口归到墙 y 之后，若两条落在同一条墙上
        //: （|Δy_page| ≤ 2），只保留**较窄**的那条。实测 f0 同一道墙报了两条
        //: （247..489 宽 242 与 239..362），过宽的那条是反光/模糊把墙切成三段合成出来的；
        //: 留着它会让决策以为墙有一段大缺口 → 有穿墙风险。
        std::vector<std::pair<double, std::pair<double, double>>> emitted_list;
        for (const auto& g : gf.gaps) {
            const double cy_img = g.cy;
            const double gl_img = g.cx - g.width * 0.5, gr_img = g.cx + g.width * 0.5;
            //: **输出墙的 y，不是缺口的 cy**。vt.jsonl 里 `walls[].y` 是"这一行墙"的位置，
            //: 缺口只贡献 gl/gr。实测：障碍 y 与 vt 的墙 y 差 ≤2 px，而缺口自带的 cy
            //: 会偏 14~27 px（被模糊的段拉偏）。
            //:
            //: 归属规则：缺口由**两侧的墙段**夹出来，所以按"缺口左右边界与墙段 x 范围
            //: 的距离"找归属，而不是按 y 最近 —— 同一道墙附近常有多余行带（f3 里
            //: y=356.5 与 381.6 两条带，缺口 cy=369 到两者都≈12 px，按 y 最近会选错，
            //: 实测把 y=383 报成 357）。
            double y_use = cy_img;
            {
                //: ⚠ **必须先过"y 一致"这道硬门**（2026-10-26 实测修）。
                //: 原来把 `half_h*1e3` 直接放在加权里，于是"最薄的那条"会赢过
                //: "行对得上但略厚"的那条：同一帧里缺口 cy=69.7（所属墙在 70 行），
                //: 但 288 行那条带更薄（hh 2.5 vs 3.5），加权 2500 < 3500 就把它选走了
                //: —— 缺口被归到页面 y=96（差 291），agent 于是把这道墙算到
                //: `ahead=-117`（身后），第一道墙永远进不了候选，开局直接撞墙。
                //: 实测 A/B：加门之前 30/30 帧归错，加门之后 0/30。
                //: 门限 40：正常偏差 0~27（缺口 cy 被模糊的墙段拉偏），跨墙至少 140。
                constexpr double kAttribMaxDy = 40.0;
                double best = 1e18;
                for (const auto& o : gf.obstacles) {
                    //: 贴页面上下沿的"障碍"是**页面白框**，不当墙也不当归属候选
                    //: （实测 f0 会选中白框那条细带，把整帧的墙都带歪）。
                    const double oy_page = row_to_page_y(o.y);
                    if (oy_page < 6.0 || oy_page > kPageH - 6.0) continue;
                    const double dy = std::fabs(o.y - cy_img);
                    if (dy > kAttribMaxDy) continue;      //: 行对不上就不是这道墙
                    const double a = o.x - o.half_w, b = o.x + o.half_w;
                    const double dx = (b <= gl_img) ? (gl_img - b)
                                    : (a >= gr_img) ? (a - gr_img)
                                    : 0.0;                     // x 有重叠：优先
                    //: 平手时**取更薄的那条**：墙是细长条（实测 hh≈7），反光/模糊会
                    //: 合成胖块（hh≈57）。f3 里缺口左右各有一条 dx=0 的候选，
                    //: 只按 |y-cy| 判是死平手（12.1 vs 12.1），按厚度就能选对。
                    const double d = dx * 1e6 + o.half_h * 1e3 + dy;
                    if (d < best) { best = d; y_use = o.y; }
                }
                //: 一个都没过门（墙太厚/缺口被拉偏太多）：就退回用缺口自己的 cy，
                //: 误差量级 14~27 px，比"归到另一道墙上（上百 px）"好得多。
            }
            //: **角色挡住的"缺口"不是缺口**（实测就是它导致了闭环擦墙）：
            //: 角色的像素是中性亮（判据不当它是墙），于是墙的连续段会被角色从中间切断，
            //: 缺口逻辑就把它当成一个开口。实测 y≈220 那道墙报出的缺口在帧间乱跳
            //: （93…207 / 135…239 / 342…446 / 466…579），**这些 x 正好是角色所在位置**
            //: ——机器人于是瞄准自己脚下那个"缺口"撞上墙（min_clearance=-10）。
            //: 判据：**角色中心落在缺口区间内，且缺口宽度 < 角色直径 + 20 px** 才是
            //: "被角色切出来的假缺口"。只按"区间重叠"会误杀：实测角色站在两个真缺口
            //: 之间时，两侧各重叠 3 px，把两条真缺口一起丢掉（f0/f1/f2/f4/f5 全空）。
            if (gf.player.present) {
                const double pr = (gf.player.radius > 0.0 ? gf.player.radius : 12.0);
                const bool inside = (gf.player.x > gl_img && gf.player.x < gr_img);
                if (inside && (gr_img - gl_img) < (2.0 * pr + 20.0)) continue;
            }
            const double y_page = row_to_page_y(y_use);
            //: **页面白框不是墙**：白框在页面最上/最下沿（页面 y≈0 或 480），它会以
            //: "一条满宽墙 + 缺口"的形式被报出来（实测 f0 多出一条 y≈477 的假墙）。
            //: 但**不能**在图像上把它抹掉 —— 判据正是靠这条满宽墙确定场地范围
            //: （span=[80,559]），抹掉它会让 span 塌缩、检测全乱（试过，f0 的墙直接
            //: 消失）。所以在这里按输出过滤：离页面上下沿 6 px 以内的不当墙。
            if (y_page < 6.0 || y_page > kPageH - 6.0) continue;
            //: **缺口宽度必须落在物理可能范围内**。实测（基线 acc9r16 自己的 1753 帧
            //: vt.jsonl）：缺口宽度 p50=105、min=45、max=162（场景真值 90~110）。
            //: 相机域的模糊/反光会造出 13 px 或 281 px 的"缺口"，而决策会把它当成
            //: 真开口 → 一头撞上去（PL 第一轮闭环就是这样擦墙，min_clearance=-10）。
            //: 所以超出 [60, 140] 页面单位的一律丢掉（场景真值 91~110，留测量余量）：宁可这道墙"没有缺口"（agent 会
            //: 把它当实墙、绕开），也不能报一个不存在的开口。
            {
                const double gwp = x_to_page_corrected(gr_img) - x_to_page_corrected(gl_img);
                if (gwp < 60.0 || gwp > 140.0) continue;
            }
            const double gl_p = x_to_page_corrected(gl_img), gr_p = x_to_page_corrected(gr_img);
            bool merged = false;
            for (auto& kv : emitted_list) {
                if (std::fabs(kv.first - y_page) > 2.0) continue;
                const double w_old = kv.second.second - kv.second.first;
                if ((gr_p - gl_p) < w_old) kv.second = {gl_p, gr_p};   // 更窄的留下
                merged = true;
                break;
            }
            if (!merged) emitted_list.push_back({y_page, {gl_p, gr_p}});
        }
        for (const auto& kv : emitted_list) {
            std::fprintf(jf, "%s{\"y\":%.1f,\"gl\":%.1f,\"gr\":%.1f}",
                         emitted ? "," : "", kv.first, kv.second.first, kv.second.second);
            ++emitted;
        }
        std::fprintf(jf, "],\"player\":");
        if (gf.player.present) {
            std::fprintf(jf, "[%.1f,%.1f]", x_to_page_corrected(gf.player.x), row_to_page_y(gf.player.y));
        } else {
            std::fprintf(jf, "null");
        }
        //: `no_gap` = 有墙但没缺口的行数。PL 的 n_obs 包含"没有缺口的墙"，
        //: 所以 no_gap = n_obs - (与缺口配上的墙数)。这里按"每条缺口对应一条墙"算，
        //: 配不上的墙（缺口在画面外/被角色挡）都计入 no_gap —— 这正是 vt_sparse 的语义。
        const int no_gap = static_cast<int>(gf.obstacles.size()) - emitted;
        std::fprintf(jf, ",\"n\":%d,\"no_gap\":%d,\"fast\":1}\n",
                     emitted, no_gap < 0 ? 0 : no_gap);
        std::fflush(jf);
    }

done:
    if (frame > 0) {
        std::fprintf(stderr, "pl_vision: 共 %ld 帧，失败 %d，重推 %d；平均 正射 %.2f / 推数 %.2f / PL %.2f ms\n",
                     frame, n_bad, n_retry, sum_rect / frame, sum_push / frame, sum_all / frame);
    }
    if (meta_f) std::fclose(meta_f);
    if (jf != stdout) std::fclose(jf);
    (void)quad_state;
    return n_bad == 0 ? 0 : 1;
}
