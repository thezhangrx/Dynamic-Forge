// tb_ortho.cpp —— vision_ortho 与定点 oracle（tools/ortho_fixed_ref --dump）逐字节对拍
// 用法: ./Vvision_ortho <cam.ppm> <expected.ppm>
//   相机帧 640×480 P6 → RTL → 输出 640×360，与 expected 逐字节比。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cmath>
#include <cstdint>
#include "Vvision_ortho.h"
#include "verilated.h"

static Vvision_ortho* top;
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

    Verilated::commandArgs(argc, argv);
    top = new Vvision_ortho;
    // 默认参数（可由 argv[3..] 覆盖：h0..h8 用 Q16.16/Q8.24 整数，其余按十进制）
    //: ⚠ 只是**兜底默认值**（= oracle `VR_DBG_HQ` 对 data/fpga/quad.json 的打印）。
    //:   用例必须把 oracle 打印的 13 个数**原样传进来**（手抄会二次量化，R14/R15）。
    const int64_t def[13] = {53183, 2476, 1584660, -1926, -48241, 27506115, -211, 3684, 16777216,
                             31413589, 87381, 8041878869LL, 22369621};
    int64_t v[18] = {0};
    for (int i = 0; i < 13; i++) v[i] = def[i];
    v[13] = 80; v[14] = 560; v[15] = 0; v[16] = 359;
    // ⚠ 只吃**数字**参数：Verilator 的 plusarg（+dbg 等）也在 argv 里，
    //   第一版把 "+dbg" 当 h0 覆盖了 → h0=0、au=0、整幅错（表现为 bu 少一项 t1）。
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
    top->rst_n = 0; top->start = 0; top->hold = 0; top->src_rgb = 0; top->src_valid = 0;
    for (int i = 0; i < 4; i++) tick();
    top->rst_n = 1; tick();
    top->start = 1; tick(); top->start = 0;

    std::vector<uint8_t> got((size_t)ew * eh * 3, 0);
    int orow = (int)v[15], ocol = 0;   // 输出位置（**整行 640 个**，黑边带也吐）
    long long guard = 200LL * 1000 * 1000;
    uint16_t prev_addr = 0;
    bool first = true;
    // done 与最后一个 out_vld 可能同一拍：允许先收完这一拍再退出，
    // 否则尾部像素会被吞掉（实测尾部 54 行不符）。
    // done 是**一拍脉冲**：必须先收这一拍的 out_vld 再退出，不能只看循环条件
    // （第一版要么漏最后一拍的像素，要么永远退不出去）。
    bool fin = false;
    while (!fin && guard-- > 0) {
        // 源数据：addr 给出后**下一拍**给数据（1 拍延迟）
        //: 源数据**组合**给出（按当前 src_addr）—— 行缓存上板时也是"地址→数据同拍"，
        //: 第一版按"上一拍地址"喂，整体错开一个像素（表现为逐字节全不符、首错在首像素）。
        {
            //: 源请求现在是 (line, off_bytes)；tb 按相机内存布局取：idx = line*640 + off/3
            size_t idx = (size_t)top->src_line * 640u + (size_t)top->src_px;
            if (idx + 2 < cam.size()) {
                top->src_rgb = (uint32_t)cam[idx * 3] << 16 | (uint32_t)cam[idx * 3 + 1] << 8 | cam[idx * 3 + 2];
                top->src_valid = 1;
            } else top->src_valid = 0;
        }
        if (top->out_vld) {
            //: sof/eol 必须严格落在每行首/尾（vision_top 靠它们数行；只比像素值查不出来）
            {
                static long sofbad = 0, eolbad = 0;
                //: sof 是**帧**开始（只有第一行第一列），eol 是每行行尾
                bool want_sof = (ocol == 0) && (orow == (int)v[15]);
                bool want_eol = (ocol == 639);
                if ((bool)top->out_sof != want_sof && sofbad++ < 3)
                    printf("  SOF 不符 row=%d col=%d got=%d want=%d\n", orow, ocol, (int)top->out_sof, (int)want_sof);
                if ((bool)top->out_eol != want_eol && eolbad++ < 3)
                    printf("  EOL 不符 row=%d col=%d got=%d want=%d\n", orow, ocol, (int)top->out_eol, (int)want_eol);
                if (sofbad + eolbad > 0 && sofbad + eolbad < 4) printf("  （sof/eol 不符累计 %ld/%ld）\n", sofbad, eolbad);
            }
            // ⚠ RTL 只输出**页面范围**内的像素（c0..c1-1 / r0..r1），图外是黑。
            //   所以不能"顺序写"，必须按 (row,col) 落位 —— 第一版就是顺序写，
            //   整幅左移 80 列，640776 字节全不符。
            size_t oi = ((size_t)orow * ew + ocol) * 3;
            if (oi + 2 < got.size()) { got[oi] = (uint8_t)top->out_r; got[oi+1] = (uint8_t)top->out_g; got[oi+2] = (uint8_t)top->out_b; }
            if (++ocol > 639) { ocol = 0; orow++; }   // RTL 现在每行吐满 PL_W=640
        }
        fin = top->done;
        tick();
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
    delete top;
    return bad == 0 ? 0 : 1;
}
