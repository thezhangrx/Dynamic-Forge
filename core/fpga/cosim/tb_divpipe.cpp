// tb_divpipe.cpp —— vision_div_pipe 的对拍（vs C 的截断整除）
// 用法: ./Vvision_div_pipe [对数] [随机种子]
// 断言：①每个输入的 quo == num/den（C 的无符号整除）；②商超 2^WQ 时 ovf=1；
//       ③吞吐 II=1：N 个输入只需 N + 固定延迟拍。
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <random>
#include <vector>
#include "Vvision_div_pipe.h"
#include "verilated.h"

static Vvision_div_pipe* top;
static long long cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

int main(int argc, char** argv) {
    const int N = (argc > 1) ? atoi(argv[1]) : 200000;
    const unsigned seed = (argc > 2) ? (unsigned)atoi(argv[2]) : 12345u;
    Verilated::commandArgs(argc, argv);
    top = new Vvision_div_pipe;
    const uint64_t MAXQ = 1ull << 34;

    top->rst_n = 0; top->ce = 1; top->in_vld = 0; top->num = 0; top->den = 0;
    for (int i = 0; i < 3; i++) tick();
    top->rst_n = 1; tick();

    std::mt19937_64 rng(seed);
    std::vector<std::pair<uint64_t, uint64_t>> in;
    in.reserve(N);
    // ---- 构造输入：大部分是"真实量级"（num<2^50, den≈2^24..2^34），
    //      再加边界（num<den、den=1、2 的幂、num 很大、商正好 2^34-1 / 2^34）----
    for (int i = 0; i < N; i++) {
        uint64_t d, n;
        int k = i % 8;
        if (k == 0) { d = 1024; n = rng() % (1ull << 40); }
        else if (k == 1) { d = 1; n = rng() % (1ull << 34); }
        else if (k == 2) { d = (1ull << 24); n = (1ull << 40) + (rng() % 1000); }
        else if (k == 3) { d = 4096; n = rng() % 700; }                 // 商 0
        else if (k == 4) { d = 33554432ull; n = 1; }                    // 商 0
        else if (k == 5) { d = 7; n = 7 * ((MAXQ - 1) - (rng() % 4)); } // 商 ≈ 2^34-1
        else if (k == 6) { d = 7; n = 7 * (MAXQ + (rng() % 4)); }       // 商 ≥ 2^34 → ovf
        else { d = (rng() % ((1ull << 33) + 1)) | 1024; n = rng() % (1ull << 50); }
        if (d == 0) d = 1;
        in.emplace_back(n, d);
    }

    long long bad = 0, badovf = 0, nout = 0;
    size_t ip = 0;
    bool fin = false;
    long long guard = 4LL * N + 1000;
    while (!fin && guard-- > 0) {
        bool push = ip < in.size();
        top->in_vld = push ? 1 : 0;
        if (push) { top->num = in[ip].first; top->den = in[ip].second; }
        top->eval();
        if (top->out_vld) {
            //: 输出与输入**同序**（II=1 的定点延迟），所以直接用索引对
            if (nout >= (long long)in.size()) { printf("输出多于输入！\n"); return 1; }
            uint64_t n = in[nout].first, d = in[nout].second;
            uint64_t want = n / d;
            uint64_t got = top->quo;
            bool want_ovf = (want >= MAXQ);
            if (want_ovf) { if (!top->ovf) { if (badovf < 3) printf("  缺 ovf #%lld n=%llu d=%llu\n", nout, (unsigned long long)n, (unsigned long long)d); badovf++; } }
            else {
                if (top->ovf) { if (badovf < 3) printf("  多 ovf #%lld n=%llu d=%llu want=%llu\n", nout, (unsigned long long)n, (unsigned long long)d, (unsigned long long)want); badovf++; }
                else if (got != want) {
                    if (bad < 3) printf("  商错 #%lld n=%llu d=%llu got=%llu want=%llu\n",
                                        nout, (unsigned long long)n, (unsigned long long)d,
                                        (unsigned long long)got, (unsigned long long)want);
                    bad++;
                }
            }
            nout++;
        }
        tick();
        if (ip < in.size()) ip++;
        fin = (nout >= (long long)in.size());
    }
    printf("输入 %zu  输出 %lld  商不符 %lld  溢出标志不符 %lld  拍数 %lld\n",
           in.size(), nout, bad, badovf, cyc);
    bool ok = (bad == 0) && (badovf == 0) && (nout == (long long)in.size());
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
