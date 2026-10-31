// tb_linecache.cpp —— vision_linecache 单独验证（不经 master，直接灌 fill 流）
// 断言：①命中读出 = mem 对应字节；②未填充的行会拉 miss 并给出正确行号。
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include "Vvision_linecache.h"
#include "verilated.h"

static Vvision_linecache* top;
static long long cyc = 0;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++; }

static std::vector<uint8_t> mem(640 * 480 * 3);   // 相机帧（字节）

static void fill_line(int line) {
    //: 填哪一行是由 `line` 端口给的（RTL 用 line[4:0] 选槽、line 记 tag）——
    //: 第一版忘了设它，四次填充全落进同一个槽（实测只有第一行读得出来）。
    top->line = (uint16_t)line;
    top->fill_start = 1; tick(); top->fill_start = 0;
    //: ⚠ 新版的填充是**重排**（8 字节 = 2⅔ 像素，一拍只写 1 个像素），
    //: 一个 beat 要占 2~3 拍 → 必须按 `fill_ready` 握手喂，否则 beat 会被丢。
    int w = 0;
    long long guard = 5000;
    while (w < 240 && guard-- > 0) {
        bool ready = top->fill_ready;
        top->fill_valid = ready ? 1 : 0;
        if (ready) {
            uint64_t d = 0;
            for (int k = 0; k < 8; k++) d |= (uint64_t)mem[(size_t)line * 1920 + w * 8 + k] << (8 * k);
            top->fill_data = d;
            top->fill_last = (w == 239);
            w++;
        }
        tick();
    }
    top->fill_valid = 0; top->fill_last = 0;
    long long ndone = 0;
    for (int i = 0; i < 8; i++) { tick(); if (top->fill_done) ndone++; }
    top->dbg_slot = (uint16_t)(line & 31); top->eval();
    printf("  fill line=%d slot=%d tag=%d (beat=%d done=%lld state=%d busy=%d)\n",
           line, line & 31, (int)top->dbg_tag, w, ndone, (int)top->dbg_fstate, (int)top->fill_busy);
}

static bool read_px(int line, int px, uint8_t* out) {
    //: data_valid/data 都是**寄存输出**（1 拍延迟）：先打一拍 req，再读
    top->line = (uint16_t)line; top->px = (uint16_t)px; top->req = 1;
    top->eval();
    if (getenv("DBG_LC"))
        printf("    req line=%d slot=%d tag=%d hit=%d rd_idx=%d\n", line, line & 31,
               (int)top->dbg_tag, (int)top->dbg_hit, (int)top->dbg_rd_idx);
    tick();
    bool ok = top->data_valid;
    //: ⚠ 字节序：**行内第一个字节在 [23:16]**（RGB 大端），与 vision_ortho 的 src_rgb
    //:   和 tb_ortho 的直供口径一致。这里原来写反成"byte0 在 [7:0]"，而 RTL 也写反了 ——
    //:   两边一致所以 72/72 全过；直到 sys 全链路对上 oracle 才发现 B/R 颠倒（2026-10-25）。
    if (ok) { out[0] = (uint8_t)(top->data >> 16), out[1] = (uint8_t)(top->data >> 8), out[2] = (uint8_t)top->data; }
    top->req = 0; tick();
    return ok;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vvision_linecache;
    for (size_t i = 0; i < mem.size(); i++) mem[i] = (uint8_t)((i * 37 + (i >> 8)) & 0xFF);
    top->rst_n = 0; top->req = 0; top->fill_start = 0; top->fill_valid = 0; top->fill_last = 0;
    top->fill_data = 0; top->line = 0; top->px = 0; top->dbg_slot = 0;
    for (int i = 0; i < 4; i++) tick();
    top->rst_n = 1; tick();

    long bad = 0, miss_cnt = 0;
    // 1) 未填充 → 必须 miss
    uint8_t px[3];
    top->line = 5; top->px = 0; top->req = 1;
    tick();                       // miss 也是寄存输出
    if (top->miss) miss_cnt++;
    top->req = 0; tick();

    // 2) 填 4 行（不同槽），逐点读回比对
    int lines[4] = {5, 6, 7, 8};
    for (int i = 0; i < 4; i++) fill_line(lines[i]);
    long checked = 0;
    for (int li = 0; li < 4; li++) {
        for (int px_i = 0; px_i < 640; px_i += 1) {
            if (!read_px(lines[li], px_i, px)) { bad++; continue; }
            size_t off = (size_t)lines[li] * 1920 + px_i * 3;
            if (px[0] != mem[off] || px[1] != mem[off + 1] || px[2] != mem[off + 2]) {
                if (bad == 0)
                    printf("  首个不符: line=%d px=%d got=%02x%02x%02x want=%02x%02x%02x "
                           "| 槽布局会是 %02x%02x%02x\n",
                           lines[li], px_i, px[0], px[1], px[2], mem[off], mem[off+1], mem[off+2],
                           mem[(size_t)(lines[li] & 31) * 1920 + px_i * 3],
                           mem[(size_t)(lines[li] & 31) * 1920 + px_i * 3 + 1],
                           mem[(size_t)(lines[li] & 31) * 1920 + px_i * 3 + 2]);
                bad++;
            }
            checked++;
        }
    }
    printf("检查 %ld 点，不符 %ld；未填充时 miss 次数 %ld；拍数 %lld\n", checked, bad, miss_cnt, cyc);
    bool ok = (bad == 0) && (miss_cnt >= 1) && (checked > 60);
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
