// tb_p0.cpp —— **P0 本地集成 cosim**：DDR 模型 → 寄存器 → 正射 → vision_top → 几何帧
//
// 为什么要有这个 tb（R77 P4：结构性根因）
// ---------------------------------------
// P0 路径（PS 写 FRMBASE/start → PL 自己从 DDR 读 → 正射 → vision_top）此前**只**在真硅上试过，
// 每次集成 bug 的代价是"重建位流 20 min + 上板一轮"，而且板上只能看到几个寄存器。
// 这个 tb 把同一整条路搬到本地：一旦它跑通/跑不通，数字立刻有，且能看到 vision_top 卡在哪一态。
//
// 它做什么
//  1) 读 `<cam.ppm>`（640×480 相机帧）→ 装进 DDR 模型（64 位字，小端）
//  2) 读 `<page.ppm>`（640×360 页面图 = 正射 oracle 的输出，RTL 正射与它逐字节一致）
//     → 用 C 参考 `vr_ref_process_ex` 算出**期望几何帧**
//  3) 通过 AXI-Lite 写：13 个正射系数、c0/c1/r0/r1、FRMBASE=0
//  4) 写 FRMCTL = src=1|clr_done，再写 src=1|start → 轮询 CTRL.busy
//  5) 收齐后读几何帧窗口（0x100，2048 B）与期望逐字节比
//  6) 若超时：用 vision_top 的 dbg（sel=15 sub=1/2/3）打 state / {wr_row,wr_col} / frame_count
//
// 用法：
//   ./Vvision_axi_p0 <cam.ppm> <page.ppm> [--max-cycles N]
//
// ⚠ 系数用 `VR_DBG_HQ` 那组（data/fpga/quad.json），与 ortho_fixed_ref/pl_vision 同一条式子。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <cmath>

#include "Vvision_axi.h"
#include "verilated.h"
#include "ref_api.h"

static Vvision_axi* top;
static long long cyc = 0;

// ---- P0 寄存器（与 rtl/vision_axi.v 的 ADDR_FRM* 一一对应）------------------
static const uint32_t A_FRMCTL   = 0x900;
static const uint32_t A_FRMBASE  = 0x904;
static const uint32_t A_FRMC0C1  = 0x908;
static const uint32_t A_FRMR0R1  = 0x90C;
static const uint32_t A_FRM_H0   = 0x910;
static const uint32_t A_FRM_PY16 = 0x934;
static const uint32_t A_FRM_PS16 = 0x938;
static const uint32_t A_FRM_PY24L= 0x93C;
static const uint32_t A_FRM_PY24H= 0x940;
static const uint32_t A_FRM_PS24 = 0x944;
static const uint32_t A_CTRL     = 0x0C0;
static const uint32_t A_FLEN     = 0x0D0;
static const uint32_t A_FRAME    = 0x100;
static const uint32_t A_DBGSEL   = 0x080;
static const uint32_t A_DBGDAT   = 0x084;

// ---- DDR 模型 + AXI 读从机 ------------------------------------------------
static std::vector<uint64_t> ddr;      // 相机帧（64 位字）
static uint32_t FRM_BASE_G = 0x10000000u;   // tb 用的相机帧基址（非零，见 main）
static bool     ar_issued = false;
static uint32_t ar_addr = 0;
static long long ar_len = 0, r_phase = 0, ar_cnt = 0, r_beats = 0;

//: AXI 从机：每拍按当前状态驱动 m_*；tick 之后结算握手（与 tb_ortho_sys 同一套写法）
//: ⚠ 只**驱动**组合输入，**不推进任何状态**。
//: 踩过的坑（R81 死锁的真身）：原来这个函数里还顺手做了"AR 握手捕获 + r_phase 清零"，
//: 而它会在"轮询寄存器"的路径里被同一拍调用两次（外层一次 + axi_read 的 step 里一次），
//: 于是同一拍把同一个 AR 捕获两次、r_phase 被重复清零 → 主口/从机 beat 记账错位 → 死锁。
//: 现在：驱动=slave_comb()（可多次调用，无副作用）；握手结算=只在 tick() 里做一次。
static int g_rd_delay = 0;          //: --rd-delay N：AR 之后过 N 拍才给第一个 R 拍
static long long ar_cyc = 0;
static void slave_comb() {
    top->m_arready = 1;          //: 主口一次只发一个 AR，永远接受
    bool give = ar_issued && (r_phase <= ar_len)
                && (g_rd_delay <= 0 || (cyc - ar_cyc) >= g_rd_delay);
    if (give) {
        size_t wi = (size_t)((ar_addr - FRM_BASE_G) / 8) + (size_t)r_phase;
        top->m_rdata  = (wi < ddr.size()) ? ddr[wi] : 0;
        top->m_rvalid = 1;
        top->m_rlast  = (r_phase == ar_len);
    } else {
        top->m_rdata = 0; top->m_rvalid = 0; top->m_rlast = 0;
    }
}
static void slave_settle(bool r_hs) {
    if (r_hs) {
        r_beats++; r_phase++;
        if (getenv("TRACE_AR") && r_phase > ar_len) printf("  AR 完 cyc=%lld r_beats=%lld\n", cyc, r_beats);
        if (r_phase > ar_len) ar_issued = false;
    }
}
//: 一拍：驱动 → 结算 AR 握手 → 上下沿 → 结算 R 握手。**每次 tick 只结算一次**。
static void tick() {
    slave_comb();
    if (top->m_arvalid && top->m_arready) {
        ar_addr = top->m_araddr;
        ar_len  = (long long)top->m_arlen;
        r_phase = 0;
        ar_issued = true;
        ar_cyc = cyc;
        ar_cnt++;
        if (getenv("TRACE_AR")) printf("  AR#%lld addr=0x%x arlen=%lld cyc=%lld\n",
                                      ar_cnt, ar_addr, ar_len, cyc);
        slave_comb();      //: AR 捕获后这一拍就该开始给拍子（与真实 AXI 从机一致）
    }
    bool r_hs = top->m_rvalid && top->m_rready;
    top->clk = 0; top->eval(); top->clk = 1; top->eval(); cyc++;
    slave_settle(r_hs);
}
static void step() { tick(); }
static void idle_bus() {
    top->s_awvalid = 0; top->s_wvalid = 0; top->s_bready = 1;
    top->s_arvalid = 0; top->s_rready = 1;
}

static bool axi_write(uint32_t addr, uint32_t data) {
    top->s_awaddr = addr; top->s_awvalid = 1;
    top->s_wdata  = data; top->s_wstrb   = 0xF; top->s_wvalid = 1;
    top->s_bready = 1;
    bool done = false;
    for (int i = 0; i < 200 && !done; i++) {
        bool aw_fire = top->s_awvalid && top->s_awready;
        bool w_fire  = top->s_wvalid  && top->s_wready;
        bool b_fire  = top->s_bvalid  && top->s_bready;
        step();
        if (aw_fire) top->s_awvalid = 0;
        if (w_fire)  top->s_wvalid  = 0;
        if (b_fire)  done = true;
    }
    idle_bus();
    return done;
}
static bool axi_read(uint32_t addr, uint32_t* out) {
    top->s_araddr = addr; top->s_arvalid = 1; top->s_rready = 1;
    bool done = false;
    for (int i = 0; i < 200 && !done; i++) {
        bool ar_fire = top->s_arvalid && top->s_arready;
        bool r_fire  = top->s_rvalid  && top->s_rready;
        step();
        if (ar_fire) top->s_arvalid = 0;
        if (r_fire) { *out = top->s_rdata; done = true; }
    }
    idle_bus();
    return done;
}
static bool dbg_read(uint32_t sel, uint32_t sub, uint32_t* out) {
    //: dbg_sel 打包：{4'b0, sub[3:0], idx[17:0], sel[3:0]}
    return axi_write(A_DBGSEL, (sub << 22) | sel) && axi_read(A_DBGDAT, out);
}

//: 相机域适配（与 `tools/camadapt_ref.py` / pl_vision.cpp:403 逐字一致）。
//: RTL 侧现在也会做这一步（vision_axi 里正射 → vision_camadapt → vision_top），
//: 所以 C 参考必须吃**适配后**的页面图，否则比的是两条不同的链。
static void adapt_rgb(std::vector<uint8_t>* img, int w, int h) {
    std::vector<uint8_t> gray((size_t)w * h), wall((size_t)w * h, 0);
    for (int i = 0; i < w * h; i++) {
        const uint8_t* p = img->data() + (size_t)i * 3;
        gray[i] = (uint8_t)((77 * p[0] + 150 * p[1] + 29 * p[2]) >> 8);
    }
    for (int r = 0; r < h; r++) {
        int c = 0;
        while (c < w) {
            if (gray[(size_t)r * w + c] < 150) { c++; continue; }
            int e = c;
            while (e < w && gray[(size_t)r * w + e] >= 150) e++;
            if (e - c >= 40)
                for (int k = c; k < e; k++) wall[(size_t)r * w + k] = 1;
            c = e;
        }
    }
    for (int i = 0; i < w * h; i++) {
        const int v = gray[i], o = i * 3;
        if (wall[i]) {
            (*img)[o] = (*img)[o + 1] = (uint8_t)v;
            (*img)[o + 2] = (uint8_t)(v > 60 ? v - 60 : 0);
        } else if (v >= 150) {
            (*img)[o] = (*img)[o + 1] = (*img)[o + 2] = 210;
        } else {
            (*img)[o] = (*img)[o + 1] = (*img)[o + 2] = (uint8_t)v;
        }
    }
}

static bool load_ppm(const char* p, int* w, int* h, std::vector<uint8_t>* out) {
    FILE* f = fopen(p, "rb");
    if (!f) { std::perror(p); return false; }
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
        fprintf(stderr, "用法: %s cam.ppm(640x480) page.ppm(640x360) [--max-cycles N]\n", argv[0]);
        return 2;
    }
    long long max_cyc = 3000000;
    const char* dump_frame = nullptr;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--max-cycles") && i + 1 < argc) max_cyc = atoll(argv[++i]);
        else if (!strcmp(argv[i], "--dump-frame") && i + 1 < argc) dump_frame = argv[++i];
    }

    int cw = 0, ch = 0, pw = 0, ph = 0;
    std::vector<uint8_t> cam, page;
    if (!load_ppm(argv[1], &cw, &ch, &cam)) return 2;
    if (!load_ppm(argv[2], &pw, &ph, &page)) return 2;
    if (cw != 640 || ch != 480) { fprintf(stderr, "相机帧要 640x480（得到 %dx%d）\n", cw, ch); return 2; }
    if (pw != 640 || ph != 360) { fprintf(stderr, "页面图要 640x360（得到 %dx%d）\n", pw, ph); return 2; }

    // ---- DDR 模型：相机帧按 64 位字（小端）----
    const size_t NW = cam.size() / 8;
    ddr.assign(NW, 0);
    for (size_t w = 0; w < NW; w++)
        for (int k = 0; k < 8; k++) ddr[w] |= (uint64_t)cam[w * 8 + k] << (8 * k);
    printf("DDR 模型：%zu 个 64 位字（%zu 字节）\n", NW, cam.size());

    // ---- C 参考：从**适配后的页面图**算期望几何帧（RTL 链路：正射→camadapt→vision_top）----
    adapt_rgb(&page, pw, ph);
    // ---- C 参考：从**页面图**算期望几何帧 ----
    VrRefParams params;
    vr_ref_default_params(&params);
    VrRefResult res;
    memset(&res, 0, sizeof(res));
    std::vector<uint8_t> gray_g(pw * ph), wall_g(pw * ph), bright_g(pw * ph);
    std::vector<uint16_t> rowc_g(ph);
    std::vector<uint8_t> frame_g(2048);
    if (vr_ref_process_ex(page.data(), pw, ph, 1, gray_g.data(), wall_g.data(),
                          bright_g.data(), rowc_g.data(), &res,
                          frame_g.data(), frame_g.size(), &params, 0, 0) != 0) {
        fprintf(stderr, "C 参考失败\n");
        return 1;
    }
    const size_t exp_len = res.frame_bytes;
    printf("C 参考期望几何帧：%zu 字节（n_obstacles=%d n_gaps=%d）\n", exp_len, res.n_obstacles, res.n_gaps);

    // ---- 仿真 ----
    Verilated::commandArgs(argc, argv);
    top = new Vvision_axi;
    top->aresetn = 0; idle_bus();
    top->din_vld = 0; top->din_sof = 0; top->din_eol = 0;
    top->din_r = 0; top->din_g = 0; top->din_b = 0;
    top->m_arready = 0; top->m_rdata = 0; top->m_rvalid = 0; top->m_rlast = 0;
    for (int i = 0; i < 5; i++) tick();
    top->aresetn = 1; tick();

    // ---- 写系数/基址 ----
    //: ⚠ 系数**不许手抄**（2026-10-26 修）：原来这里硬编码了旧 quad 的一组
    //: `VR_DBG_HQ` 值。一旦 `data/fpga/quad.json` 变了（相机/取景一挪就得重算），
    //: oracle 用新 quad、RTL 用旧系数，门禁就会"红得有道理，但红错了原因"——
    //: 实测就是这样：RTL=292B / 参考=328B、n_obs 4 vs 5、78/328 不符。
    //: 现在改成：`tb_p0 <cam.ppm> <page.ppm> [13 个系数] [--frames N]`，
    //: 由 pytest 从 oracle 的 `VR_DBG_HQ` 打印里取出来喂进来（与 tb_ortho 同一规矩）。
    int64_t coefs[13] = {53183, 2476, 1584660, -1926, -48241, 27506115, -211, 3684, 16777216,
                         31413589, 87381, 8041878869LL, 22369621};
    int ngiven = 0;
    while (3 + ngiven < argc && ngiven < 13) {
        char* e = nullptr;
        const long long v = std::strtoll(argv[3 + ngiven], &e, 10);
        if (e == argv[3 + ngiven] || (e && *e)) break;
        coefs[ngiven] = v;
        ngiven++;
    }
    if (ngiven == 0) {
        printf("⚠ 没给系数（13 个 arg），退回旧 quad 的硬编码值 —— 只在旧 quad 下可信\n");
    } else if (ngiven != 13) {
        fprintf(stderr, "系数必须给 13 个（h0..h8, PY16, PS16, PY24, PS24），收到 %d 个\n", ngiven);
        return 1;
    }
    for (int i = 0; i < 9; i++)
        if (!axi_write(A_FRM_H0 + 4 * i, (uint32_t)coefs[i])) { fprintf(stderr, "写 h%d 失败\n", i); return 1; }
    if (!axi_write(A_FRM_PY16, (uint32_t)coefs[9])) return 1;
    if (!axi_write(A_FRM_PS16, (uint32_t)coefs[10])) return 1;
    if (!axi_write(A_FRM_PY24L, (uint32_t)((uint64_t)coefs[11] & 0xFFFFFFFFull))) return 1;
    if (!axi_write(A_FRM_PY24H, (uint32_t)(((uint64_t)coefs[11] >> 32) & 0xFFull))) return 1;
    if (!axi_write(A_FRM_PS24, (uint32_t)coefs[12])) return 1;
    if (!axi_write(A_FRMC0C1, (560u << 16) | 80u)) return 1;
    if (!axi_write(A_FRMR0R1, (359u << 16) | 0u)) return 1;
    FRM_BASE_G = 0x10000000u;   //: 故意非零：本地也要能抓到"忘了加基址"
    if (!axi_write(A_FRMBASE, FRM_BASE_G)) return 1;

    uint32_t v = 0;
    if (!axi_read(A_FRMC0C1, &v)) return 1;
    printf("回读 C0C1=0x%08x（期望 0x02300050）%s\n", v, v == 0x02300050u ? "✓" : "✗");
    if (!axi_read(A_FRM_PY24L, &v)) return 1;
    printf("回读 PY24L=0x%08x（期望 0xdf555555）%s\n", v, v == 0xdf555555u ? "✓" : "✗");

    int nframes = 1;
    for (int i = 3 + ngiven; i < argc; i++) if (!strcmp(argv[i], "--frames") && i + 1 < argc) nframes = atoi(argv[++i]);
    for (int i = 3 + ngiven; i < argc; i++) if (!strcmp(argv[i], "--rd-delay") && i + 1 < argc) g_rd_delay = atoi(argv[++i]);
    std::vector<std::pair<long,size_t>> run_stats;
    for (int frame_i = 0; frame_i < nframes; frame_i++) {
    // ---- 起一帧（P0 路径）----
    if (!axi_write(A_FRMCTL, 0x1u | 0x4u)) { fprintf(stderr, "清 done 失败\n"); return 1; }
    const long long cyc0 = cyc;
    if (!axi_write(A_FRMCTL, 0x1u | 0x2u)) { fprintf(stderr, "start 失败\n"); return 1; }
    const long long cyc_start = cyc;

    // ---- 轮询 CTRL.busy ----
    //: 轮询 CTRL.busy（每 64 拍一次）。**现在可以放心轮询**：从机模型的握手结算只在
    //: tick() 里做一次，MMIO 读写不会再把 beat 记账搞乱（R81 那个"死锁"的真身就是它）。
    bool busy_seen = false, done = false;
    long long guard = max_cyc;
    long long last_sample = 0;
    while (guard-- > 0) {
        if (((max_cyc - guard) & 63) == 0) {
            uint32_t ctrl = 0;
            if (axi_read(A_CTRL, &ctrl)) {
                if (ctrl & 1u) busy_seen = true;
                if (busy_seen && !(ctrl & 1u)) { done = true; break; }
            }
        }
        if (cyc - last_sample > 200000) {
            last_sample = cyc;
            uint32_t st = 0, rc = 0;
            dbg_read(15, 1, &st); dbg_read(15, 2, &rc);
            printf("  [%lld] vtop state=%u row=%u col=%u | AXI burst %lld 读 %lld 拍\n",
                   cyc, st & 0x1Fu, (rc >> 10) & 0x1FFu, rc & 0x3FFu, ar_cnt, r_beats);
        }
        tick();
    }
    long long cyc_pl = cyc - cyc_start;

    uint32_t ctrl = 0, flen = 0;
    axi_read(A_CTRL, &ctrl);
    axi_read(A_FLEN, &flen);
    printf("帧结束: %s | 拍数 %lld（%.2f ms @50MHz）| CTRL=0x%08x frame_count=%u | FLEN=%u | AXI burst %lld 读 %lld 拍\n",
           done ? "完成" : "**超时**", cyc_pl, cyc_pl / 50000.0,
           ctrl, (ctrl >> 8) & 0xFFFFu, flen, ar_cnt, r_beats);

    if (!done) {
        uint32_t st = 0, rc = 0, fc = 0;
        dbg_read(15, 1, &st);     // state
        dbg_read(15, 2, &rc);     // {wr_row, wr_col}
        dbg_read(15, 3, &fc);     // frame_count
        uint32_t d0 = 0, d1 = 0, d2 = 0, dline = 0;
        axi_read(0x948, &d0); axi_read(0x94C, &d1); axi_read(0x950, &d2); axi_read(0x954, &dline);
        uint32_t dline2 = 0;
        axi_read(0x954, &dline2);
        printf("  AXI 主口: state=%u left=%u（0=IDLE 1=AR 2=DATA）\n",
               dline2 & 3u, (dline2 >> 4) & 0xFFFFu);
        printf("  正射侧: src_line=%u src_px=%u miss=%u hit=%u blk=%u | tag=0x%04x fill_state=%u "
               "flags=0x%02x rd_idx=%u fills=%u | miss_line=%u\n",
               d0 >> 16, (d0 >> 5) & 0x3FFu, d0 & 1u, (d0 >> 1) & 1u, (d0 >> 2) & 1u,
               d1 >> 16, (d1 >> 8) & 0xFFu, d2 >> 26, d2 & 0x7FFFu, d1 & 0xFFu, dline >> 16);
        printf("  vision_top: state=%u wr_row=%u wr_col=%u frame_count=%u"
               "（state: 0=IDLE 1=RECV 2=BAND 10/11/12=SEG 13=GAP 14=CCL 15/16=PACK 17=DONE）\n",
               st & 0x1Fu, (rc >> 10) & 0x1FFu, rc & 0x3FFu, fc);
        printf("FAIL\n");
        delete top;
        return 1;
    }

    // ---- 读几何帧窗口，与 C 参考逐字节比 ----
    std::vector<uint8_t> win(2048, 0);
    for (size_t i = 0; i < 2048 / 4; i++) {
        uint32_t word = 0;
        if (!axi_read(A_FRAME + (uint32_t)i * 4, &word)) { fprintf(stderr, "读帧窗口失败\n"); return 1; }
        win[i * 4 + 0] = (uint8_t)(word);
        win[i * 4 + 1] = (uint8_t)(word >> 8);
        win[i * 4 + 2] = (uint8_t)(word >> 16);
        win[i * 4 + 3] = (uint8_t)(word >> 24);
    }
    long bad = 0, first = -1;
    if (flen != exp_len) printf("  ⚠ 帧长不符：RTL=%u 参考=%zu\n", flen, exp_len);
    for (size_t i = 0; i < exp_len && i < win.size(); i++)
        if (win[i] != frame_g[i]) { if (first < 0) first = (long)i; bad++; }
    printf("几何帧逐字节：不符 %ld / %zu（首个不符 %ld）\n", bad, exp_len, first);
    if (first >= 0) {
        size_t lo = (size_t)first > 8 ? (size_t)first - 8 : 0;
        printf("  RTL : ");
        for (size_t i = lo; i < lo + 16 && i < win.size(); i++) printf("%02x ", win[i]);
        printf("\n  参考: ");
        for (size_t i = lo; i < lo + 16 && i < exp_len; i++) printf("%02x ", frame_g[i]);
        printf("\n");
    }
    if (dump_frame && exp_len > 0) {
        if (FILE* f = fopen(dump_frame, "wb")) { fwrite(win.data(), 1, (size_t)flen, f); fclose(f); printf("几何帧已落盘：%s（%u 字节）\n", dump_frame, flen); }
    }
    {
        printf("  row_count[0,60,120,200,300,359] =");
        const int rows[6] = {0, 60, 120, 200, 300, 359};
        for (int r = 0; r < 6; r++) {
            uint32_t val = 0;
            //: dbg_sel 打包：{4'b0, sub[3:0], idx[17:0], sel[3:0]}
            axi_write(A_DBGSEL, ((uint32_t)rows[r] << 4) | 3u);
            axi_read(A_DBGDAT, &val);
            printf(" %u", val);
        }
        printf("\n");
    }
    run_stats.push_back({bad, (size_t)flen});
    printf("  [帧 %d] FLEN=%u 不符 %ld / %zu\n", frame_i, flen, bad, exp_len);
    // ---- 下一帧：不 reset，接着跑（模拟板上连跑）----
    if (frame_i + 1 < nframes) {
        busy_seen = false; done = false; guard = max_cyc; last_sample = cyc;
        //: 等 busy 落下再起下一帧（探针在板上也是这么做的）
        while (guard-- > 0) {
            if (((max_cyc - guard) & 63) == 0) {
                uint32_t c2 = 0;
                if (axi_read(A_CTRL, &c2) && !(c2 & 1u)) break;
            }
            tick();
        }
    }
    }  // for frame_i
    bool ok = true;
    for (auto& st : run_stats) if (st.first != 0) ok = false;
    printf("%s\n", ok ? "PASS" : "FAIL");
    delete top;
    return ok ? 0 : 1;
}
