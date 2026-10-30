// tb_axim.cpp —— vision_axi_master 的功能验证（DDR 内存模型）
// 用法: ./Vvision_axi_master [words]     默认一整行：240 个 64 位字
// 断言：①读出的数据与 mem[] 逐字相同；②拍数 == beats；③done 恰好一次。
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cstdint>
#include "Vvision_axi_master.h"
#include "verilated.h"

static Vvision_axi_master* top;
static long long cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

int main(int argc, char** argv) {
    const int beats = (argc > 1) ? atoi(argv[1]) : 240;      // 640*3/8 = 240
    const int base  = (argc > 2) ? atoi(argv[2]) : 0;
    Verilated::commandArgs(argc, argv);
    top = new Vvision_axi_master;

    // DDR 模型：640*480 像素 = 307200 字（这里按 64 位字存，够覆盖一行）
    const size_t N = 4096;
    std::vector<uint64_t> mem(N);
    for (size_t i = 0; i < N; i++) mem[i] = (uint64_t)i;   // 单调：便于肉眼判"差一拍"还是"步长错"

    top->rst_n = 0; top->start = 0; top->m_arready = 0; top->m_rvalid = 0; top->m_rdata = 0; top->m_rlast = 0;
    top->r_en = 1;   //: 下游永远能接（这个 tb 只验主口本身）
    for (int i = 0; i < 4; i++) tick();
    top->rst_n = 1; tick();

    top->base = (uint32_t)base; top->beats = (uint16_t)beats;
    top->start = 1; tick(); top->start = 0;

    long long got = 0, bad = 0, donecnt = 0;
    bool ar_issued = false;
    uint64_t ar_addr = 0; int ar_len = 0;
    long long r_phase = 0;      // 已经给了几拍
    long long guard = 1000000;
    while (guard-- > 0) {
        // AR：收下就记下来
        top->m_arready = ar_issued ? 0 : 1;
        if (top->m_arvalid && top->m_arready) { ar_addr = top->m_araddr; ar_len = top->m_arlen; ar_issued = true; }
        // R：按模型给数据（每拍一个字，rlast 与最后一拍同拍）
        bool give = ar_issued && r_phase < (ar_len + 1) && !top->m_rlast;
        if (give) {
            size_t idx = (size_t)((ar_addr + r_phase * 8) / 8);
            top->m_rdata = idx < N ? mem[idx] : 0;
            top->m_rvalid = 1;
            top->m_rlast  = (r_phase == ar_len);
        } else { top->m_rvalid = 0; top->m_rlast = 0; }
        top->eval();   //: ⚠ 必须"先驱动、再 eval、后检查" —— 少了这一步，检查读到的是上一拍的组合值
                       //:   （实测表现：数据整体差一个 beat，got=0 want=1 …）
        if (top->m_rvalid && top->m_rready) {
            // 检查数据
            size_t idx = (size_t)((ar_addr + r_phase * 8) / 8);
            uint64_t want = (idx < N ? mem[idx] : 0);
            if (top->m_data != want) { if (bad < 3) printf("  mismatch@%lld got=%llu want=%llu\n", r_phase, (unsigned long long)top->m_data, (unsigned long long)want); bad++; }
            got++; r_phase++;
        }
        if (top->done) donecnt++;
        tick();
        if (donecnt && !top->busy) break;
    }
    printf("beats=%d base=%d  arlen=%d  读出 %lld 拍  数据不符 %lld  done 次数 %lld  拍数 %lld\n",
           beats, base, ar_len, got, bad, donecnt, cyc);
    bool ok = (bad == 0) && (got == beats) && (donecnt == 1) && (ar_len == beats - 1);
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
