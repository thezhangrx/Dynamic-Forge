// ===========================================================================
// pl_p0_probe.cpp —— P0（正射在 PL 里）的**上板探针**：换位流后先量这一帧
//
// 为什么先写这个、而不是直接改 pl_vision（R1d / R1c）
// ------------------------------------------------------------------------
// P0 的数据通路是全新的：PS 把相机帧 memcpy 进 udmabuf → 写 FRMBASE → start →
// PL 自己按行从 HP0 读 → 正射 →（本探针阶段还没接相机域适配）→ vision_top → 几何帧。
// 这条路上任何一环错了（udmabuf 没 sync、FRMBASE 没对齐、HP0 没连上、系数写错），
// pl_vision 那边都会表现为"没反应"，而它同时还在做很多别的事（quad、vt.jsonl…），
// 排障面太大。所以先用一个**只做最少事**的探针，把"帧能不能跑完、要多久"量出来。
//
// 它做的事（每一步都打印，成功失败都能看到是哪一步）：
//   1) mmap PL 寄存器窗口（0x43C00000）
//   2) mmap /dev/udmabuf0，读 /sys/.../phys_addr，写一帧（默认合成图；给了 PPM 就用 PPM）
//   3) 写 sync_for_device（u-dma-buf 的 sync_mode=1：CPU 写完必须 sync，否则 PL 可能读旧 DRAM）
//   4) 写 13 个正射系数 + c0/c1/r0/r1 + FRMBASE
//   5) 写 FRMCTL = src=1|start=1，轮询 FRMCTL.done，**用 CLOCK_MONOTONIC 计时**
//   6) 读 CTRL（vision_top 的 frame_done/帧计数）与 FLEN（几何帧字节数），
//      再读一遍 FRMCTL 的 fills（行填充次数：应当 ≈ 相机行数）
//   7) 重复 N 次，打印每次耗时 + 中位数
//
// ⚠ 系数是 **data/fpga/quad.json 的 VR_DBG_HQ 值**（与 ortho_fixed_ref 打印的一致）。
//   换标定/换 quad 必须重算；正式版由 pl_vision 用同一条式子算出来写，这里只为探针简单。
//
// 编译（板上）：
//   g++ -O2 -std=c++17 -I/root/board_src/core/board pl_p0_probe.cpp -o pl_p0_probe
// 用法：
//   ./pl_p0_probe --quad data/fpga/quad.json --frames 30     # **正式用法**：自己算系数
//   ./pl_p0_probe --ppm /tmp/cam.ppm                          # 用真实 640×480 PPM
//   ./pl_p0_probe --dump-frame /tmp/f                         # 把几何帧落盘（与本地 cosim 比）
//   P0_COEF_ONLY=1 ./pl_p0_probe --quad data/fpga/quad.json    # 只算系数（本机可跑，自证）
// 注意：每帧前会等 vision_top 的 busy 落下（P5：不等就会被 S_IDLE 吞掉、给出 0.00 ms 假数）
// ===========================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "vision_regs.hpp"
//: 从 quad.json 自己算 13 个正射系数（与 `tools/ortho_fixed_ref.cpp` 同一串式子）。
//: 正式版（`pl_vision --p0`）走的就是这条路：**PS 算系数、PL 只做像素**。
#include "field_frame.hpp"
#include "rectify.hpp"

namespace {

constexpr int kCamW = 640, kCamH = 480;
constexpr uint32_t kFrameBytes = 640u * 480u * 3u;   // 921600

// ---- P0 寄存器块（与 rtl/vision_axi.v 的 ADDR_FRM* 一一对应）----------------
constexpr uint32_t kOffFrmCtl  = 0x900;
//: ⚠ 同一地址、读写语义不同（实测踩过：把读的 bit1 当 done，于是第一帧"0.00 ms"就返回了）：
//:   W: bit0 = 像素源(1=正射)  bit1 = start(同时起 vision_top)  bit2 = 清 done
//:   R: bit0 = src            bit1 = **of_busy**（正射在跑）    bit2 = done(粘滞态)
//:      [15:3] = 0            [31:16] = 行填充次数
constexpr uint32_t kCtlSrc = 1u << 0, kCtlStart = 1u << 1, kCtlClrDone = 1u << 2;
constexpr uint32_t kStBusy = 1u << 1, kStDone = 1u << 2;
constexpr uint32_t kOffFrmBase = 0x904;
constexpr uint32_t kOffFrmC0C1 = 0x908;
constexpr uint32_t kOffFrmR0R1 = 0x90C;
constexpr uint32_t kOffFrmH0   = 0x910;   // h0..h8 = 0x910 + 4*i
constexpr uint32_t kOffFrmPy16 = 0x934;
constexpr uint32_t kOffFrmPs16 = 0x938;
constexpr uint32_t kOffFrmPy24L= 0x93C;
constexpr uint32_t kOffFrmPy24H= 0x940;
constexpr uint32_t kOffFrmPs24 = 0x944;
constexpr uint32_t kOffFrmDbg0 = 0x948;
constexpr uint32_t kOffFrmDbg1 = 0x94C;
constexpr uint32_t kOffFrmDbg2 = 0x950;
constexpr uint32_t kOffFrmLine = 0x954;

//: 下面这组是 data/fpga/quad.json 的 VR_DBG_HQ 打印值（**兜底默认**）。
//: 给了 `--quad <path>` 就用自己算出来的那组，并与这组对比打印（不一致要当心标定换了）。
constexpr int64_t kH0 = 53183, kH1 = 2476, kH2 = 1584660;
constexpr int64_t kH3 = -1926, kH4 = -48241, kH5 = 27506115;
constexpr int64_t kH6 = -211, kH7 = 3684, kH8 = 16777216;
constexpr int64_t kPy16 = 31413589, kPs16 = 87381;
constexpr int64_t kPy24 = 8041878869LL, kPs24 = 22369621;
constexpr int     kC0 = 80, kC1 = 560, kR0 = 0, kR1 = 359;

//: 把 quad.json 的 8 个数 → 相机→页面单应取逆 → 量化成 RTL 端口的 13 个整数。
//: 与 ortho_fixed_ref.cpp 逐行对应（常数 FIT/OFF_X/PAGE_W/H 必须与视觉链一致）。
static bool calc_coefs(const char* quad_path, int64_t out[13], int* c0, int* c1) {
    constexpr double PAGE_W = 640.0, PAGE_H = 480.0, PL_W = 640.0, PL_H = 360.0;
    constexpr double FIT = 0.75;                       // min(640/640, 360/480)
    constexpr double OFF_X = (PL_W - PAGE_W * FIT) * 0.5;
    vision_board::field_frame::Quad fq;
    FILE* f = std::fopen(quad_path, "r");
    if (!f) { std::perror(quad_path); return false; }
    double v[8] = {0};
    if (std::fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3],
                    &v[4], &v[5], &v[6], &v[7]) != 8) { std::fclose(f); return false; }
    std::fclose(f);
    for (int i = 0; i < 4; i++) { fq.p[i][0] = v[2 * i]; fq.p[i][1] = v[2 * i + 1]; }
    fq.ok = true;
    auto H = rectify::invert(vision_board::field_frame::homography(fq, PAGE_W, PAGE_H));
    if (!H.ok) { std::fprintf(stderr, "单应求逆失败\n"); return false; }
    double h[9];
    for (int i = 0; i < 9; i++) h[i] = H.h[i] / H.h[8];
    auto q16 = [](double x) { return (int64_t)std::llround(x * 65536.0); };
    auto q24 = [](double x) { return (int64_t)std::llround(x * 16777216.0); };
    const double CA = 1.0 / FIT, CB = (0.5 - OFF_X) / FIT;
    out[0] = q16(h[0]); out[1] = q16(h[1]); out[2] = q16(h[2]);
    out[3] = q16(h[3]); out[4] = q16(h[4]); out[5] = q16(h[5]);
    out[6] = q24(h[6]); out[7] = q24(h[7]); out[8] = q24(h[8]);
    out[9] = q16((PL_H - 0.5) * CA);   // PY0_16
    out[10] = q16(CA);                 // PS_16
    out[11] = q24((PL_H - 0.5) * CA);  // PY0_24
    out[12] = q24(CA);                 // PS_24
    //: 列范围门控（与 oracle 同一串整数）：px = CA*c + CB ∈ (0, PAGE_W]
    *c0 = (int)PL_W; *c1 = 0;
    for (int c = 0; c < (int)PL_W; c++) {
        const double px = CA * c + CB;
        if (px > 0.0 && px <= PAGE_W) { if (c < *c0) *c0 = c; *c1 = c + 1; }
    }
    return true;
}

double now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

struct Regs {
    volatile uint32_t* r = nullptr;
    void* map = nullptr;
    bool open() {
        int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0) { std::perror("/dev/mem"); return false; }
        map = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, vision_board::kRegBase);
        ::close(fd);
        if (map == MAP_FAILED) { std::perror("mmap regs"); map = nullptr; return false; }
        r = static_cast<volatile uint32_t*>(map);
        return true;
    }
    uint32_t rd(uint32_t off) const { return r[off >> 2]; }
    //: **单个 32 位 volatile 写**（多笔会被 PL 当多次提交，踩过，见 vision_pixfeed 的注释）
    void wr(uint32_t off, uint32_t v) { r[off >> 2] = v; }
};

struct Udmabuf {
    uint8_t* p = nullptr;
    uint64_t phys = 0;
    size_t   size = 0;
    bool open(const char* dev = "/dev/udmabuf0",
              const char* phys_path = "/sys/class/u-dma-buf/udmabuf0/phys_addr",
              const char* size_path = "/sys/class/u-dma-buf/udmabuf0/size") {
        // ---- 物理地址 / 大小 ----
        if (FILE* f = std::fopen(phys_path, "r")) {
            if (std::fscanf(f, "%llx", (unsigned long long*)&phys) != 1) phys = 0;
            std::fclose(f);
        }
        if (FILE* f = std::fopen(size_path, "r")) {
            unsigned long long v = 0;
            if (std::fscanf(f, "%llu", &v) == 1) size = (size_t)v;
            std::fclose(f);
        }
        if (!phys || !size) { std::fprintf(stderr, "读不到 %s / %s\n", phys_path, size_path); return false; }
        int fd = ::open(dev, O_RDWR | O_SYNC);
        if (fd < 0) { std::perror(dev); return false; }
        p = static_cast<uint8_t*>(::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        ::close(fd);
        if (p == MAP_FAILED) { std::perror("mmap udmabuf"); p = nullptr; return false; }
        return true;
    }
    //: sync_mode=1：CPU 写完必须告诉驱动"交给设备"，否则 PL 可能读到旧 DRAM
    void sync_for_device() {
        if (FILE* f = std::fopen("/sys/class/u-dma-buf/udmabuf0/sync_for_device", "w")) {
            std::fputs("0\n", f);
            std::fclose(f);
        }
    }
};

bool read_ppm(const char* path, std::vector<uint8_t>* out) {
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::perror(path); return false; }
    int w = 0, h = 0, mx = 0;
    if (std::fscanf(f, "P6 %d %d %d", &w, &h, &mx) != 3 || w != kCamW || h != kCamH) {
        std::fprintf(stderr, "PPM 要 640x480 P6，得到 %dx%d\n", w, h);
        std::fclose(f);
        return false;
    }
    std::fgetc(f);
    out->resize(kFrameBytes);
    size_t n = std::fread(out->data(), 1, out->size(), f);
    std::fclose(f);
    if (n != out->size()) { std::fprintf(stderr, "PPM 数据不足\n"); return false; }
    return true;
}

//: 合成图：横向渐变 + 一条亮带（只为"帧能跑完"服务，不对应任何真实场景）
void synth_frame(std::vector<uint8_t>* out) {
    out->resize(kFrameBytes);
    for (int r = 0; r < kCamH; ++r) {
        for (int c = 0; c < kCamW; ++c) {
            uint8_t* p = out->data() + ((size_t)r * kCamW + c) * 3;
            uint8_t v = (uint8_t)((c * 255) / (kCamW - 1));
            if (r > 200 && r < 230) { p[0] = 200; p[1] = 200; p[2] = 200; }
            else { p[0] = (uint8_t)(v / 3); p[1] = (uint8_t)(v / 3); p[2] = (uint8_t)(v / 3); }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    int frames = 10;
    int watch = 0;
    std::string dump_frame;
    std::string quad_path;   //: --quad <data/fpga/quad.json>：自己算 13 个系数（正式用法）   //: --dump-frame <path>：把几何帧取回来（与本地 cosim 逐字节比）          //: --watch N：起一帧后每 100 ms 打一次中间量（定位"慢在哪"）
    std::string ppm;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if (k == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (k == "--ppm" && i + 1 < argc) ppm = argv[++i];
        else if (k == "--watch" && i + 1 < argc) watch = std::atoi(argv[++i]);
        else if (k == "--dump-frame" && i + 1 < argc) dump_frame = argv[++i];
        else if (k == "--quad" && i + 1 < argc) quad_path = argv[++i];
        else { std::fprintf(stderr, "未知参数 %s\n", k.c_str()); return 2; }
    }
    if (frames < 1) frames = 1;
    (void)watch;   //: watch 采样是上一版逐条打点用的；现在改为"每帧前等 busy + 端到端计时"，留着参数不删

    //: `--coef-only`：只算并打印 13 个系数（本机可跑，不碰 /dev/mem）。
    //: 用来证明"PS 侧算出来的系数 = 我们一直用的 VR_DBG_HQ 那组"。
    if (!quad_path.empty() && getenv("P0_COEF_ONLY")) {
        int64_t c[13];
        int a = 0, b = 0;
        if (!calc_coefs(quad_path.c_str(), c, &a, &b)) return 1;
        static const int64_t ref[13] = {kH0, kH1, kH2, kH3, kH4, kH5, kH6, kH7, kH8,
                                        kPy16, kPs16, kPy24, kPs24};
        int nd = 0;
        std::printf("现算：");
        for (int i = 0; i < 13; i++) { std::printf("%lld ", (long long)c[i]); if (c[i] != ref[i]) nd++; }
        std::printf("\nc0..c1=%d..%d（期望 %d..%d）| 与 VR_DBG_HQ 不同 %d 项\n", a, b, kC0, kC1, nd);
        return (nd == 0 && a == kC0 && b == kC1) ? 0 : 1;
    }
    Regs regs;
    if (!regs.open()) return 1;
    Udmabuf buf;
    if (!buf.open()) return 1;
    std::printf("udmabuf: phys=0x%llx size=%zu MB\n", (unsigned long long)buf.phys, buf.size >> 20);
    if (buf.phys & 3u) { std::fprintf(stderr, "phys 不是 4 字节对齐\n"); return 1; }

    std::vector<uint8_t> frame;
    if (!ppm.empty()) {
        if (!read_ppm(ppm.c_str(), &frame)) return 1;
        std::printf("帧来源：%s\n", ppm.c_str());
    } else {
        synth_frame(&frame);
        std::printf("帧来源：合成图\n");
    }
    std::memcpy(buf.p, frame.data(), kFrameBytes);
    buf.sync_for_device();

    int64_t coefs[13] = {kH0, kH1, kH2, kH3, kH4, kH5, kH6, kH7, kH8,
                         kPy16, kPs16, kPy24, kPs24};
    int c0v = kC0, c1v = kC1;
    if (!quad_path.empty()) {
        if (!calc_coefs(quad_path.c_str(), coefs, &c0v, &c1v)) return 1;
        static const int64_t ref[13] = {kH0, kH1, kH2, kH3, kH4, kH5, kH6, kH7, kH8,
                                        kPy16, kPs16, kPy24, kPs24};
        int ndiff = 0;
        for (int i = 0; i < 13; i++) if (coefs[i] != ref[i]) ndiff++;
        std::printf("系数（quad 现算）：%lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld"
                    " | c0..c1=%d..%d | 与 VR_DBG_HQ 兜底值不同的项 %d\n",
                    (long long)coefs[0], (long long)coefs[1], (long long)coefs[2],
                    (long long)coefs[3], (long long)coefs[4], (long long)coefs[5],
                    (long long)coefs[6], (long long)coefs[7], (long long)coefs[8],
                    (long long)coefs[9], (long long)coefs[10], (long long)coefs[11],
                    (long long)coefs[12], c0v, c1v, ndiff);
    }
    // ---- 系数（一次即可；PL 的寄存器会粘住）----
    for (int i = 0; i < 9; i++) regs.wr(kOffFrmH0 + 4 * i, (uint32_t)coefs[i]);
    regs.wr(kOffFrmPy16, (uint32_t)coefs[9]);
    regs.wr(kOffFrmPs16, (uint32_t)coefs[10]);
    regs.wr(kOffFrmPy24L, (uint32_t)((uint64_t)coefs[11] & 0xFFFFFFFFull));
    regs.wr(kOffFrmPy24H, (uint32_t)(((uint64_t)coefs[11] >> 32) & 0xFFu));
    regs.wr(kOffFrmPs24, (uint32_t)coefs[12]);
    regs.wr(kOffFrmC0C1, ((uint32_t)c1v << 16) | (uint32_t)c0v);
    regs.wr(kOffFrmR0R1, ((uint32_t)kR1 << 16) | (uint32_t)kR0);
    regs.wr(kOffFrmBase, (uint32_t)buf.phys);
    std::printf("系数/基址已写：FRMBASE=0x%08x\n", (uint32_t)buf.phys);
    //: **回读**（R7：写完必须验它真的落进去了。板上第一次跑就是 c0/c1 没写进去 →
    //: band 恒真 → 整帧发黑 → 4.62 ms 就"跑完"了，数字看着漂亮其实全错）
    std::printf("回读：FRMBASE=0x%08x C0C1=0x%08x R0R1=0x%08x PY16=%d PS16=%d PY24L=0x%08x PY24H=0x%02x PS24=%d h0=%d h8=%d\n",
                regs.rd(kOffFrmBase), regs.rd(kOffFrmC0C1), regs.rd(kOffFrmR0R1),
                (int32_t)regs.rd(kOffFrmPy16), (int32_t)regs.rd(kOffFrmPs16),
                regs.rd(kOffFrmPy24L), regs.rd(kOffFrmPy24H) & 0xFF, (int32_t)regs.rd(kOffFrmPs24),
                (int32_t)regs.rd(kOffFrmH0), (int32_t)regs.rd(kOffFrmH0 + 4 * 8));

    //: 起跑前先确认 vision_top 是**空闲**的：上一轮如果卡在 S_RECV（busy=1），
    //: 新写的 start 会被 S_IDLE 的判定吞掉，于是"正射在跑、vision_top 在等另一帧"，
    //: 数字会变成几秒级（实测踩过 5622 ms）。这种情况应当先重配 PL 再来。
    {
        long spins = 0;
        while ((regs.rd(vision_board::kOffCtrl) & vision_board::kStatusBusy) && spins < 2000000)
            ++spins;
        if (regs.rd(vision_board::kOffCtrl) & vision_board::kStatusBusy) {
            std::fprintf(stderr, "vision_top 不是空闲（CTRL=0x%08x）：先重配 PL 再跑探针\n",
                         regs.rd(vision_board::kOffCtrl));
            return 1;
        }
    }
    std::vector<double> times;
    for (int i = 0; i < frames; ++i) {
        //: ⚠ P5：上一帧没结束时写 start 会被 vision_top 的 S_IDLE 判定吞掉
        //: （表现：这一帧 0.00 ms 就"完成"，用的是上一帧留下的 done 粘滞位）。
        //: 所以每帧前先等 busy=0，超时就明确报错而不是给假数。
        {
            const double tw0 = now_ms();
            while ((regs.rd(vision_board::kOffCtrl) & vision_board::kStatusBusy) &&
                   now_ms() - tw0 < 500.0) { }
            if (regs.rd(vision_board::kOffCtrl) & vision_board::kStatusBusy) {
                std::fprintf(stderr, "帧 %d 起跑前 busy 不落（CTRL=0x%08x），跳过\n", i,
                             regs.rd(vision_board::kOffCtrl));
                continue;
            }
        }
        regs.wr(kOffFrmCtl, kCtlSrc | kCtlClrDone);   // src=1, 清 done
        const double t_start = now_ms();
        regs.wr(kOffFrmCtl, kCtlSrc | kCtlStart);     // src=1, start
        //: 端到端 = start → **vision_top 的 busy 落下**（正射 done 只是"像素喂完"）
        bool vtop_done = false;
        {
            const double tw0 = now_ms();
            while (now_ms() - tw0 < 500.0) {
                if (!(regs.rd(vision_board::kOffCtrl) & vision_board::kStatusBusy)) {
                    vtop_done = true;
                    break;
                }
            }
        }
        const double t_all = now_ms();
        const uint32_t ctrl = regs.rd(vision_board::kOffCtrl);
        const uint32_t flen = regs.rd(vision_board::kOffFLen);
        const uint32_t fc = regs.rd(kOffFrmCtl);
        if (!vtop_done) {
            std::fprintf(stderr, "帧 %d: vision_top 500 ms 内没结束（CTRL=0x%08x）\n", i, ctrl);
            continue;
        }
        times.push_back(t_all - t_start);
        //: 每帧打几个 row_count（PL 眼里的"每行墙像素数"）：把"输入数据是否一致"
        //: 变成一行数字，不用猜。dbg sel=3, idx=row → row_count[row]。
        {
            std::printf("  row_count[0,60,120,200,300,359] =");
            for (int r : {0, 60, 120, 200, 300, 359}) {
                regs.wr(vision_board::kOffDbgSel, vision_board::pack_dbg_sel(3, (uint32_t)r, 0));
                std::printf(" %u", regs.rd(vision_board::kOffDbgData));
            }
            std::printf("\n");
        }
        if (!dump_frame.empty() && flen > 0 && flen <= 2048) {
            std::vector<uint8_t> buf(flen);
            for (uint32_t w = 0; w < (flen + 3) / 4; ++w) {
                const uint32_t word = regs.rd(vision_board::kOffFrame + w * 4);
                for (int k = 0; k < 4 && w * 4 + k < flen; ++k)
                    buf[w * 4 + k] = (uint8_t)(word >> (8 * k));
            }
            char path[512];
            std::snprintf(path, sizeof path, "%s_%02d.bin", dump_frame.c_str(), i);
            if (FILE* f = std::fopen(path, "wb")) {
                std::fwrite(buf.data(), 1, buf.size(), f);
                std::fclose(f);
                std::printf("  几何帧已落盘：%s（%u 字节）\n", path, flen);
            }
        }
        std::printf("帧 %2d: 端到端 %.2f ms | fills=%u | frame_count=%u FLEN=%u\n",
                    i, t_all - t_start, fc >> 16, (ctrl >> 8) & 0xFFFFu, flen);
    }
    std::sort(times.begin(), times.end());
    const double med = times[times.size() / 2];
    std::printf("PL 单帧：中位 %.2f ms（min %.2f / max %.2f，%zu 帧）\n",
                med, times.front(), times.back(), times.size());
    //: 预算：整条链 ≤35 ms 里，PL 只占"正射+取数+判据"这一段；PS 侧还有 memcpy 与几何帧读取。
    std::printf("判据：PL 段中位 %.2f ms %s 35 ms 预算（含 PS 侧还要再算）\n",
                med, med <= 20.0 ? "明显在" : "接近/超过");
    return 0;
}
