// tb_ortho_sys.cpp —— P0 全链路对拍：DDR 模型 → AXI4 读主口 → 行缓存 → 正射
//
// 用法: ./Vvision_ortho_sys cam.ppm expected.ppm [h0..h8 PY0_16 PS_16 PY0_24 PS_24 c0 c1 r0 r1]
//
// 与 tb_ortho.cpp 的区别（为什么还要这一个）：tb_ortho 是**直供**源像素（按 src_line/src_off
// 当场查数组），验证的是正射算术；本 tb 把真实的数据通路接起来 ——
//   行缓存（4 槽）+ 填充控制（miss→按行 burst）+ AXI 读主口 + DDR 从机模型，
// 于是"未命中 → 填 240 个 64 位字 → 再命中 → 出像素"这条时序也被逐字节验证。
// 判据：输出与 ortho_fixed_ref --dump 的页面**逐字节 0 不符**。
//
// ⚠ AXI 从机的时序（R7：这里错一次会表现为"数据差一个 beat/差一行"，很难从输出反推）
//   * AR：arvalid & arready 同拍算发出；发出后本 tb 不再拉 arready（一次一条 burst）。
//   * R ：rready 由 master 按状态给；本 tb 只在 ar_issued 且还没送满 beats 时给 rvalid，
//         rlast 与最后一拍同拍；**先驱动、再在 tick 前判定握手**（不能用 tick 之后的
//         m_arvalid，那是"下一拍"的值 —— tb_axim 里踩过差一个 beat）。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cmath>
#include <cstdint>
#include "Vvision_ortho_sys.h"
#include "verilated.h"

static Vvision_ortho_sys* top;
static long long cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

static bool load_ppm(const char* p, int* w, int* h, std::vector<uint8_t>* out) {
    FILE* f = fopen(p, "rb");
    if (!f) return false;
    int mx; if (fscanf(f, "P6 %d %d %d", w, h, &mx) != 3) { fclose(f); return false; }
    fgetc(f);
    out->resize((size_t)(*w) * (*h) * 3);
    size_t n = fread(out->data(), 1, out->size(), f);
    fclose(f);
    return n == out->size();
}

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "用法: %s cam.ppm expected.ppm [h0..h8 PY0_16 PS_16 PY0_24 PS_24 c0 c1 r0 r1]\n", argv[0]); return 2; }
    int cw = 0, ch = 0, ew = 0, eh = 0;
    std::vector<uint8_t> cam, exp;
    if (!load_ppm(argv[1], &cw, &ch, &cam) || !load_ppm(argv[2], &ew, &eh, &exp)) { fprintf(stderr, "读 PPM 失败\n"); return 2; }
    if (cw != 640 || ch != 480) { fprintf(stderr, "相机帧要 640x480，得到 %dx%d\n", cw, ch); return 2; }

    // ---- DDR 从机模型：按 64 位字（小端）存相机帧 ----
    const size_t NW = cam.size() / 8;                  // 921600/8 = 115200
    std::vector<uint64_t> ddr(NW, 0);
    for (size_t w = 0; w < NW; w++)
        for (int k = 0; k < 8; k++) ddr[w] |= (uint64_t)cam[w * 8 + k] << (8 * k);

    Verilated::commandArgs(argc, argv);
    top = new Vvision_ortho_sys;
    //: ⚠ 只是**兜底默认值**（= oracle `VR_DBG_HQ` 对 data/fpga/quad.json 的打印）。
    //:   用例必须把 oracle 打印的 13 个数**原样传进来**（手抄会二次量化，R14/R15）。
    const int64_t def[13] = {53183, 2476, 1584660, -1926, -48241, 27506115, -211, 3684, 16777216,
                             31413589, 87381, 8041878869LL, 22369621};
    int64_t v[18] = {0};
    for (int i = 0; i < 13; i++) v[i] = def[i];
    v[13] = 80; v[14] = 560; v[15] = 0; v[16] = 359;
    for (int i = 3, k = 0; i < argc && k < 17; i++) {
        if (argv[i][0] == '+') continue;   // 只跳 plusarg；负数系数是以 '-' 开头的合法数字
        v[k++] = atoll(argv[i]);
    }

    top->h0 = (int32_t)v[0]; top->h1 = (int32_t)v[1]; top->h2 = (int32_t)v[2];
    top->h3 = (int32_t)v[3]; top->h4 = (int32_t)v[4]; top->h5 = (int32_t)v[5];
    top->h6 = (int32_t)v[6]; top->h7 = (int32_t)v[7]; top->h8 = (int32_t)v[8];
    top->PY0_16 = v[9]; top->PS_16 = v[10]; top->PY0_24 = v[11]; top->PS_24 = v[12];
    top->c0 = (uint16_t)v[13]; top->c1 = (uint16_t)v[14];
    top->r0 = (uint16_t)v[15]; top->r1 = (uint16_t)v[16];
    top->rst_n = 0; top->start = 0; top->hold = 0;
    top->m_arready = 0; top->m_rdata = 0; top->m_rvalid = 0; top->m_rlast = 0;
    for (int i = 0; i < 4; i++) tick();
    top->rst_n = 1; tick();
    //: ⚠ 故意让 start 脉冲落在 **hold=1 期间**（模拟 vision_top 每帧先进 S_HCLR、
    //: busy=0 的那 ~257 拍）：这正是"start 被 ce 吞掉"那个 bug 的现场。
    //: 修法见 vision_ortho 的 start_pend。hold 放开后必须能正常起帧（拍数 +1 拍）。
    top->hold = 1;
    top->start = 1; tick(); top->start = 0;
    for (int i = 0; i < 5; i++) tick();
    top->hold = 0; tick();

    // ---- AXI 从机状态 ----
    bool     ar_issued = false;
    uint32_t ar_addr = 0;
    long long ar_len = 0, r_phase = 0;
    long long ar_cnt = 0, r_beat = 0, bad_beat = 0;

    std::vector<uint8_t> got((size_t)ew * eh * 3, 0);
    int orow = (int)v[15], ocol = 0;   // 整行 640 个（黑边带也吐）
    long long guard = 400LL * 1000 * 1000;
    bool fin = false;
    while (!fin && guard-- > 0) {
        // ---- 从机：AR ----
        top->m_arready = ar_issued ? 0 : 1;
        if (top->m_arvalid && top->m_arready) {
            ar_addr = top->m_araddr; ar_len = (long long)top->m_arlen; r_phase = 0;
            ar_issued = true; ar_cnt++;
            //: 地址必须是 64 位对齐的行首（line*1920/8 = line*240）
            if ((ar_addr % 8) != 0 || ((ar_addr / 8) % 240) != 0)
                printf("  ⚠ AR 地址不对齐行首: %u\n", ar_addr);
        }
        // ---- 从机：R ----
        bool give = ar_issued && (r_phase <= ar_len);
        if (give) {
            size_t wi = (size_t)(ar_addr / 8) + (size_t)r_phase;
            top->m_rdata  = (wi < ddr.size()) ? ddr[wi] : 0;
            top->m_rvalid = 1;
            top->m_rlast  = (r_phase == ar_len);
        } else {
            top->m_rdata = 0; top->m_rvalid = 0; top->m_rlast = 0;
        }
        //: 握手判定用**本拍**（posedge 前）的 m_rready —— tick 之后它已经是下一拍的值
        bool r_hs = top->m_rvalid && top->m_rready;

        // ---- 收输出像素（registered 输出，与 tb_ortho 同一套落位规则）----
        if (top->out_vld) {
            size_t oi = ((size_t)orow * ew + ocol) * 3;
            if (oi + 2 < got.size()) {
                got[oi] = (uint8_t)top->out_r; got[oi+1] = (uint8_t)top->out_g; got[oi+2] = (uint8_t)top->out_b;
            }
            //: 诊断口：前几个不符点打出"像素坐标 + RTL 实际取的相机像素"
            static long dbgn = 0;
            if (dbgn < 8 && getenv("DBG_MIS") && oi + 2 < got.size() && oi + 2 < exp.size()) {
                if (got[oi] != exp[oi] || got[oi+1] != exp[oi+1] || got[oi+2] != exp[oi+2]) {
                    size_t ci = (size_t)top->dbg_src_line * 640u + (size_t)top->dbg_src_px;
                    printf("MIS #%ld row=%d col=%d got=%02x%02x%02x want=%02x%02x%02x "
                           "| src_line=%d src_px=%d cam@src=%02x%02x%02x blk=%d rd_idx=%d want_idx=%d hit=%d\n",
                           dbgn, orow, ocol, got[oi], got[oi+1], got[oi+2],
                           exp[oi], exp[oi+1], exp[oi+2],
                           (int)top->dbg_src_line, (int)top->dbg_src_px,
                           cam[ci*3], cam[ci*3+1], cam[ci*3+2], (int)top->dbg_blk,
                           (int)top->dbg_rd_idx,
                           (int)(((top->dbg_src_line & 31) * 640) + top->dbg_src_px),
                           (int)top->dbg_hit);
                    dbgn++;
                }
            }
            if (++ocol > 639) { ocol = 0; orow++; }   // RTL 现在每行吐满 PL_W=640
        }
        fin = top->done;

        tick();
        if (r_hs) {
            //: 顺带校验 AXI 读回的数据真的来自 DDR 模型（否则"像素对"可能是巧合）
            if (top->m_rdata != ((size_t)(ar_addr / 8) + (size_t)r_phase < ddr.size()
                                 ? ddr[(size_t)(ar_addr / 8) + (size_t)r_phase] : 0)) bad_beat++;
            r_beat++; r_phase++;
            if (r_phase > ar_len) ar_issued = false;
        }
    }
    if (!fin) { printf("超时（%lld 拍）\n", cyc); return 1; }

    if (getenv("DUMP_OUT")) {
        FILE* f = fopen(getenv("DUMP_OUT"), "wb");
        if (f) { fprintf(f, "P6\n%d %d\n255\n", ew, eh); fwrite(got.data(), 1, got.size(), f); fclose(f); }
    }
    long bad = 0, first_bad = -1; long long maxd = 0;
    for (size_t i = 0; i < got.size() && i < exp.size(); i++)
        if (got[i] != exp[i]) { if (first_bad < 0) first_bad = (long)i; bad++; maxd = std::max<long long>(maxd, std::abs((int)got[i] - (int)exp[i])); }
    printf("拍数 %lld  输出字节 %zu/%zu  不符 %ld  最大差 %lld  首个不符 %ld\n",
           cyc, got.size(), exp.size(), bad, maxd, first_bad);
    printf("AXI: burst %lld 读回 %lld 拍（期望 burst×240）数据不符 %lld | 行填充 %d 次\n",
           ar_cnt, r_beat, bad_beat, (int)top->dbg_fills);
    bool ok = (bad == 0) && (bad_beat == 0) && (ar_cnt == (long long)top->dbg_fills) && (ar_cnt > 0);
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
