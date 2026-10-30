// tb_camadapt.cpp —— vision_camadapt 与参考实现（tools/camadapt_ref.py）逐字节对拍
// 用法: ./Vvision_camadapt in.ppm exp.ppm [rows]
//   in.ppm  = 正射后的页面图（640×360，未适配）—— 一般就是 `ortho_fixed_ref --dump` 的产物
//   exp.ppm = 同一张图过 camadapt_ref.py 之后的样子
// 判据：输出整幅逐字节 0 不符。
//
// ⚠ 为什么要有这条（而不是"看着对"）：这段适配的颜色是**判据的输入**（R−B>40 才算墙、
//   亮度>195 才算角色），一个像素错就可能让一道墙漏报。而它又是从 PS 侧 C 代码搬过来的，
//   抄错常数/抄错分支都"看起来很正常"。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cmath>
#include <cstdint>
#include "Vvision_camadapt.h"
#include "verilated.h"

static Vvision_camadapt* top;
static long long cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

static bool load_ppm(const char* p, int* w, int* h, std::vector<uint8_t>* out) {
    FILE* f = fopen(p, "rb");
    if (!f) return false;
    int mx = 0;
    if (fscanf(f, "P6 %d %d %d", w, h, &mx) != 3) { fclose(f); return false; }
    fgetc(f);
    out->resize((size_t)(*w) * (*h) * 3);
    size_t n = fread(out->data(), 1, out->size(), f);
    fclose(f);
    return n == out->size();
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s in.ppm exp.ppm\n", argv[0]);
        return 2;
    }
    int iw = 0, ih = 0, ew = 0, eh = 0;
    std::vector<uint8_t> in, exp;
    if (!load_ppm(argv[1], &iw, &ih, &in) || !load_ppm(argv[2], &ew, &eh, &exp)) {
        fprintf(stderr, "读 PPM 失败\n");
        return 2;
    }
    if (iw != 640 || ih != 360 || ew != 640 || eh != 360) {
        fprintf(stderr, "要 640x360 的 PPM（得到 in %dx%d / exp %dx%d）\n", iw, ih, ew, eh);
        return 2;
    }

    Verilated::commandArgs(argc, argv);
    top = new Vvision_camadapt;
    top->rst_n = 0; top->flush = 0; top->in_vld = 0; top->in_sof = 0; top->in_eol = 0;
    top->in_r = 0; top->in_g = 0; top->in_b = 0;
    for (int i = 0; i < 4; i++) tick();
    top->rst_n = 1; tick();

    const int W = iw, H = ih;
    std::vector<uint8_t> got((size_t)W * H * 3, 0);

    // ---- 灌一整帧：每行 640 个像素，sof/eol 由列号给 ----
    size_t npix = (size_t)W * H;
    size_t oi = 0;
    for (size_t i = 0; i < npix; i++) {
        int col = (int)(i % W);
        const uint8_t* p = in.data() + i * 3;
        top->in_vld = 1;
        top->in_sof = (col == 0);
        top->in_eol = (col == W - 1);
        top->in_r = p[0]; top->in_g = p[1]; top->in_b = p[2];
        tick();
        if (top->out_vld) {
            //: ⚠ 索引必须是 oi*3（这里是**像素**计数）。第一版写成 got[oi]，
            //: 于是主循环按字节偏移写、flush 段按像素写 —— 整幅错位（首个不符在 (0,26)）。
            if (oi * 3 + 2 < got.size()) {
                got[oi * 3]     = (uint8_t)top->out_r;
                got[oi * 3 + 1] = (uint8_t)top->out_g;
                got[oi * 3 + 2] = (uint8_t)top->out_b;
            }
            oi++;
        }
    }
    // ---- 冲刷：缓冲里还剩 <=41 个像素 ----
    top->in_vld = 0; top->in_sof = 0; top->in_eol = 0;
    top->flush = 1;
    long long guard = 1000;
    while (oi < npix && guard-- > 0) {
        tick();
        if (top->out_vld) {
            if (oi * 3 + 2 < got.size()) {
                got[oi * 3] = (uint8_t)top->out_r; got[oi * 3 + 1] = (uint8_t)top->out_g; got[oi * 3 + 2] = (uint8_t)top->out_b;
            }
            oi++;
        }
    }
    if (oi != npix) { printf("输出像素数不对：%zu != %zu 拍数 %lld\n", oi, npix, cyc); return 1; }

    long bad = 0, first = -1;
    long long maxd = 0;
    for (size_t i = 0; i < got.size() && i < exp.size(); i++) {
        if (got[i] != exp[i]) {
            if (first < 0) first = (long)i;
            bad++;
            maxd = std::max<long long>(maxd, std::abs((int)got[i] - (int)exp[i]));
        }
    }
    if (getenv("DUMP_OUT")) {
        if (FILE* f = fopen(getenv("DUMP_OUT"), "wb")) {
            fprintf(f, "P6\n%d %d\n255\n", W, H);
            fwrite(got.data(), 1, got.size(), f);
            fclose(f);
        }
    }
    printf("拍数 %lld  输出字节 %zu/%zu  不符 %ld  最大差 %lld  首个不符 %ld\n",
           cyc, got.size(), exp.size(), bad, maxd, first);
    if (first >= 0) {
        size_t pi = (size_t)first / 3, ch = (size_t)first % 3;
        printf("  首个不符：像素 (%zu,%zu) 通道 %zu  got=%02x want=%02x\n",
               pi / W, pi % W, ch, got[first], exp[first]);
    }
    bool ok = (bad == 0);
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
