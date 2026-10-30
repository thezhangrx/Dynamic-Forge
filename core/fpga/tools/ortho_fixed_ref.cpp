// ortho_fixed_ref.cpp —— 正射的**定点参考实现**（RTL 的 oracle）
//
// 定稿算术（见 core/fpga/docs/PL_P0_P1_MIGRATION.md 的 R9/R10）：
//   分子 Q16.16： au=(h0*CA)>>16, bu=(h0*CB>>16)+(h1*PY>>16)+h2
//   分母 Q8.24 ： ad=(h6*CA)>>24, bd=(h6*CB>>24)+(h7*PY>>24)+h8
//   每像素     ： u = ((au*c+bu) << 8) / (ad*c+bd)     （像素实数）
//                v 同上；|ad*c+bd| < 1024 → 输黑
// 采样：最近邻（与 pl_vision --nearest 同口径）。RTL 要逐字节等于这个文件。
//
// 用法： ./ortho_fixed_ref --quad data/fpga/quad.json --w 640 --h 480 \
//            --dump /tmp/ox/f  < raw_rgb24            # 输入是连续 640×480 RGB24
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>

#include "field_frame.hpp"
#include "rectify.hpp"

namespace {
constexpr int PL_W = 640, PL_H = 360;
constexpr double PAGE_W = 640.0, PAGE_H = 480.0;
constexpr double FIT = 0.75;                 // min(640/640, 360/480)
constexpr double OFF_X = (PL_W - PAGE_W * FIT) * 0.5;   // 80
constexpr int S16 = 1 << 16;
constexpr int S24 = 1 << 24;

//: ⚠⚠ 定点常数**一律 int64_t**。踩过的坑（R14/R15）：`PY24 = 479.33*2^24 = 8.04e9`
//: 超过 INT32_MAX，用 `int` 存就溢出成负数 → 分母 `bd` 直接错、整幅图错位 1/0.889 倍。
//: 规律：**Q16 常数最大 3.1e7（放得下 int32），Q24 常数最大 8.0e9（必须 int64）**。
//: RTL 侧对应：Q24 的行基址/乘积一律用 64 位累加器，别图省 DSP 用 32 位。
//: 这条不是"某个变量错了"，是**一整类宽度问题**，所以这里连同乘积一起收紧。
int64_t q16(double v) { return std::llround(v * S16); }
int64_t q24(double v) { return std::llround(v * S24); }
int64_t floor_shr(int64_t v, int n) { return v >> n; }   // 与 Verilog 有符号 >>>> 同义

struct Coef {                  // 一行的六个系数
    int64_t au, bu, av, bv;    // Q16.16
    int64_t ad, bd;            // Q8.24
};
}  // namespace

int main(int argc, char** argv) {
    std::string quad_file, dump;
    int cw = 640, ch = 480, frames = 0;
    for (int i = 1; i < argc; i++) {
        std::string k = argv[i];
        if (k == "--quad") quad_file = argv[++i];
        else if (k == "--w") cw = std::atoi(argv[++i]);
        else if (k == "--h") ch = std::atoi(argv[++i]);
        else if (k == "--dump") dump = argv[++i];
        else if (k == "--frames") frames = std::atoi(argv[++i]);
    }
    vision_board::field_frame::Quad fq;
    {
        FILE* f = std::fopen(quad_file.c_str(), "r");
        if (!f) { std::perror("quad"); return 1; }
        double v[8];
        if (std::fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) != 8) {
            std::fprintf(stderr, "quad 文件要 8 个数\n");
            return 1;
        }
        std::fclose(f);
        for (int i = 0; i < 4; i++) { fq.p[i][0] = v[2 * i]; fq.p[i][1] = v[2 * i + 1]; }
        fq.ok = true;
    }
    // cam→page 再取逆（与 pl_vision 同一条路），最后按 H[8] 归一化
    auto H = rectify::invert(vision_board::field_frame::homography(fq, PAGE_W, PAGE_H));
    if (!H.ok) { std::fprintf(stderr, "单应求逆失败\n"); return 1; }
    double h[9];
    for (int i = 0; i < 9; i++) h[i] = H.h[i] / H.h[8];
    //: 宽度审计（每一项都按最大值验过）：
    //:   hq[*]  ≤ 419.71*2^16 = 2.75e7      → int32 够，但仍统一 int64 免踩
    //:   hd[*]  ≤ 1.0*2^24   = 1.68e7       → int32 够
    //:   CA24   = 1.3333*2^24 = 2.24e7      → int32 够
    //:   CB24   = -106*2^24   = -1.78e9     → int32 勉强够（离边界 3.7e8）
    //:   PY24   = 479.33*2^24 = **8.04e9**  → **int32 溢出**（R14 的元凶）
    //:   PS24   = 1.3333*2^24 = 2.24e7      → 够
    //:   py24(r)= PY24-PS24*r，与 PY24 同量级
    const int64_t hq[6] = {q16(h[0]), q16(h[1]), q16(h[2]), q16(h[3]), q16(h[4]), q16(h[5])};
    const int64_t hd[3] = {q24(h[6]), q24(h[7]), q24(h[8])};   // h8 归一化后 = 1.0 → 1<<24
    if (getenv("VR_DBG_H")) {
        for (int i = 0; i < 9; i++) std::fprintf(stderr, "h[%d]=%.9f%s", i, h[i], (i % 3 == 2) ? "\n" : " ");
    }
    const double CA = 1.0 / FIT, CB = (0.5 - OFF_X) / FIT;
    const int64_t CA16 = q16(CA), CB16 = q16(CB);
    const int64_t PY16 = q16((PL_H - 0.5) * CA), PS16 = q16(CA);
    const int64_t CA24 = q24(CA), CB24 = q24(CB);
    const int64_t PY24 = q24((PL_H - 0.5) * CA), PS24 = q24(CA);
    if (getenv("VR_DBG_HQ")) {
        //: 把**量化后**的系数按 RTL 端口顺序打出来，tb 直接拿去当覆盖参数，
        //: 避免"手抄小数"被二次量化（R7：这是一整类坑，不只 h）。
        std::fprintf(stdout, "%d %d %d %d %d %d %d %d %d %lld %lld %lld %lld\n",
                     hq[0], hq[1], hq[2], hq[3], hq[4], hq[5], hd[0], hd[1], hd[2],
                     (long long)PY16, (long long)PS16, (long long)PY24, (long long)PS24);
    }


    //: 页面范围门控（漏了它 = 左右 80 px 黑边带会被采样成灰，实测整幅对不上 2.86%）：
    //:   px = CA*c + CB 必须是 (0, PAGE_W]；py = (PL_H-r-0.5)*CA 必须是 (0, PAGE_H]。
    //: RTL 里就是两个比较器（c0..c1 由 PS 写进寄存器，或按同一串整数算出来）。
    int c0 = PL_W, c1 = 0;
    for (int c = 0; c < PL_W; c++) {
        const double px = CA * c + CB;
        if (px > 0.0 && px <= PAGE_W) { if (c < c0) c0 = c; c1 = c + 1; }
    }
    std::vector<uint8_t> cam(static_cast<size_t>(cw) * ch * 3), page(static_cast<size_t>(PL_W) * PL_H * 3);
    long fr = 0;
    while (frames == 0 || fr < frames) {
        size_t got = 0;
        while (got < cam.size()) {
            size_t n = std::fread(cam.data() + got, 1, cam.size() - got, stdin);
            if (n == 0) break;
            got += n;
        }
        if (got < cam.size()) break;
        fr++;
        // ---- 逐行算系数，再逐像素采样（最近邻）----
        std::memset(page.data(), 0, page.size());
        for (int r = 0; r < PL_H; r++) {
            const double pyf = (PL_H - r - 0.5) * CA;
            if (pyf <= 0.0 || pyf > PAGE_H) continue;          // 页面外（上下边）→ 黑
            const int64_t py16 = PY16 - PS16 * r;
            const int64_t py24 = PY24 - PS24 * r;
            Coef co;
            co.au = floor_shr(hq[0] * CA16, 16);
            co.bu = floor_shr(hq[0] * CB16, 16) +
                    floor_shr(hq[1] * py16, 16) + hq[2];
            co.av = floor_shr(hq[3] * CA16, 16);
            co.bv = floor_shr(hq[3] * CB16, 16) +
                    floor_shr(hq[4] * py16, 16) + hq[5];
            co.ad = floor_shr(hd[0] * CA24, 24);
            co.bd = floor_shr(hd[0] * CB24, 24) +
                    floor_shr(hd[1] * py24, 24) + hd[2];
            if (getenv("VR_DBG_ALL")) {
                std::fprintf(stdout, "%d %lld %lld %lld %lld %lld %lld\n", r,
                             (long long)co.au, (long long)co.bu, (long long)co.av,
                             (long long)co.bv, (long long)co.ad, (long long)co.bd);
            }
            if (getenv("VR_DBG_RC")) {
                int dr = atoi(getenv("VR_DBG_RC"));
                if (r == dr) {
                    int dc = getenv("VR_DBG_C") ? atoi(getenv("VR_DBG_C")) : 320;
                    const int64_t ndd = co.ad * dc + co.bd;
                    std::fprintf(stderr, "dbg r=%d c=%d au=%.4f bu=%.4f av=%.4f bv=%.4f ad=%.8f bd=%.8f\n",
                                 r, dc, co.au / 65536.0, co.bu / 65536.0, co.av / 65536.0,
                                 co.bv / 65536.0, co.ad / 16777216.0, co.bd / 16777216.0);
                    std::fprintf(stderr, "    u=%lld v=%lld  (float 期望见下)\n",
                                 (long long)((((co.au*dc+co.bu)<<8)-(ndd>>1))/ndd),
                                 (long long)((((co.av*dc+co.bv)<<8)-(ndd>>1))/ndd));
                }
            }
            uint8_t* row = page.data() + static_cast<size_t>(r) * PL_W * 3;
            for (int c = c0; c < c1; c++) {
                const int64_t nd = co.ad * c + co.bd;
                if (nd < 1024 && nd > -1024) continue;            // 页面外/退化 → 黑
                // ⚠ 与 pl_vision 同一像素中心约定：先 -0.5 再取整
                //   （float 版是 `u = (...)*inv_d - 0.5f; xi=(int)u`）。
                //   漏掉这个 0.5 时：逐字节相同率只有 2.85%，平均差 34 灰阶。
                //: 先算 Q16.16 的 u/v，再减 0.5 后取整 —— 与 float 版
                //: `u = (...)/d - 0.5f; xi = (int)u` **同序**。
                //: 之前写成 `(...<<8 - nd/2)/nd` 差了不少于 0.5 的取整方向，
                //: 边界像素会差 1 个采样点（实测 99.66% → 目标 ≥99.9%）。
                const int64_t uq = ((co.au * c + co.bu) << 8) * 65536 / nd;
                const int64_t vq = ((co.av * c + co.bv) << 8) * 65536 / nd;
                const int64_t u = (uq - 32768) >> 16;
                const int64_t v = (vq - 32768) >> 16;
                int64_t ui = u < 0 ? 0 : (u > cw - 1 ? cw - 1 : u);
                int64_t vi = v < 0 ? 0 : (v > ch - 1 ? ch - 1 : v);
                if (getenv("VR_DBG_UV")) {
                    int dr = atoi(getenv("VR_DBG_UV"));
                    int dc = getenv("VR_DBG_UV_C") ? atoi(getenv("VR_DBG_UV_C")) : 80;
                    if (r == dr && c == dc)
                        std::fprintf(stderr, "C_UV r=%d c=%d nd=%lld u=%lld v=%lld addr=%lld\n",
                                     r, c, (long long)nd, (long long)u, (long long)v,
                                     (long long)(v * cw + u));
                }
                std::memcpy(row + static_cast<size_t>(c) * 3,
                            cam.data() + (vi * cw + ui) * 3, 3);
            }
        }
        if (!dump.empty()) {
            char path[512];
            std::snprintf(path, sizeof path, "%s_%05ld.ppm", dump.c_str(), fr);
            if (FILE* f = std::fopen(path, "wb")) {
                std::fprintf(f, "P6\n%d %d\n255\n", PL_W, PL_H);
                std::fwrite(page.data(), 1, page.size(), f);
                std::fclose(f);
            }
        }
    }
    std::fprintf(stderr, "ortho_fixed_ref: %ld 帧\n", fr);
    return 0;
}
