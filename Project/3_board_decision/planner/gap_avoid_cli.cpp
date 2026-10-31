// ===========================================================================
// gap_avoid_cli —— 把 C++ 版 `plan()` 包成"吃文本轨迹、吐决策"的进程
//
// 存在的唯一理由：**让 Python 能把同一批输入喂给两边并逐帧对账**。
// 它不是上板程序（上板入口在 core/board/main.cpp），所以刻意保持零依赖 ——
// 只读 stdin、只写 stdout，不用 JSON 库、不用文件参数。
//
// 用法：
//     gap_avoid_cli < trace.txt > decisions.txt
//
// 输入格式（行为单位，空白分隔；`#` 起头是注释）：
//     CFG   <safety_margin> <max_window> <align_frac> <dead_frac> <allow_cross>
//           <aim_lookahead> <front_back> <fwd_speed> <hold_clearance>
//           <has_forward> <fx> <fy> <max_turn_rate>
//     FRAME <dt> <step_index> <field_w> <field_h> <horizon>
//           <n_rects> <px> <py> <pvx> <pvy> <radius> <speed> <alive>
//     RECT  <ent_id> <x> <y> <vx> <vy> <half_w> <half_h> <rotation> <angular_velocity>
//     ...（RECT 紧跟在它所属的 FRAME 后面，条数 = n_rects）
//
// CFG 出现一次（文件头）。每读完一个 FRAME + 它的 RECT，就输出一行：
//     DEC <wx> <wy> <tier> <t_arr> <clearance> <gap> <usable_half> <e_lo> <e_hi>
//         <aim_lat> <s_hat> <n_rects> <wall_band>
//
// 浮点一律 `%.17g`（最短往返表示），Python 侧用 `float()` 读回即可逐位还原。
//
// ⚠ **一个进程 = 一局**：`GapMemory` 在 main 里建一次，跨 FRAME 保持。
//   Python 对照组也必须每局新建 `GapMemory()`，否则跨帧记忆对不上。
// ===========================================================================
#include "gap_avoid.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace gap_avoid;

namespace {

int          g_line = 0;
const char*  g_err = nullptr;

char  g_buf[8192];
char* g_cur = nullptr;      // g_buf 里的当前位置；nullptr = 本行已取完

/// 读一个空白分隔的 token。
/// ⚠ 早先这里把**整行**当成一个 token 返回，于是 `CFG 6 8 0.6 ...` 整行被拿去和
///   "CFG" 比较，直接报"第一行必须是 CFG"。取词必须按空白切，不是按行切。
bool next_tok(char* buf, size_t cap, bool* eof) {
    *eof = false;
    for (;;) {
        if (g_cur != nullptr) {
            while (*g_cur == ' ' || *g_cur == '\t') g_cur += 1;
            if (*g_cur == '#') *g_cur = '\0';          // 注释到行尾
            if (*g_cur != '\0') {
                size_t n = 0;
                while (g_cur[n] != '\0' && g_cur[n] != ' ' && g_cur[n] != '\t') n += 1;
                const size_t m = (n < cap - 1) ? n : cap - 1;
                std::memcpy(buf, g_cur, m);
                buf[m] = '\0';
                g_cur += n;
                return true;
            }
            g_cur = nullptr;
        }
        if (std::fgets(g_buf, static_cast<int>(sizeof(g_buf)), stdin) == nullptr) {
            *eof = true;
            return false;
        }
        g_line += 1;
        size_t n = std::strlen(g_buf);
        while (n > 0 && (g_buf[n - 1] == '\n' || g_buf[n - 1] == '\r')) g_buf[--n] = '\0';
        g_cur = g_buf;
    }
}

double need_double(bool* ok) {
    char buf[512];
    bool eof = false;
    if (!next_tok(buf, sizeof(buf), &eof)) { g_err = "输入在期望处结束"; *ok = false; return 0.0; }
    char* end = nullptr;
    const double v = std::strtod(buf, &end);
    if (end == buf) { *ok = false; g_err = "期望一个实数"; return 0.0; }
    *ok = true;
    return v;
}

long long need_int(bool* ok) {
    char buf[512];
    bool eof = false;
    if (!next_tok(buf, sizeof(buf), &eof)) { g_err = "输入在期望处结束"; *ok = false; return 0; }
    char* end = nullptr;
    const long long v = std::strtoll(buf, &end, 10);
    if (end == buf) { *ok = false; g_err = "期望一个整数"; return 0; }
    *ok = true;
    return v;
}

}  // namespace

int main() {
    AlgoConfig cfg;
    GapMemory  mem;

    // ---- 表头（CFG）----
    {
        char buf[512];
        bool eof = false;
        if (!next_tok(buf, sizeof(buf), &eof)) { std::fprintf(stderr, "空输入\n"); return 2; }
        if (std::strcmp(buf, "CFG") != 0) {
            std::fprintf(stderr, "第 %d 行：第一行必须是 CFG，得到 `%s`\n", g_line, buf);
            return 2;
        }
    }
    bool ok = true;
    cfg.safety_margin = need_double(&ok);
    cfg.max_window = need_double(&ok);
    cfg.align_frac = need_double(&ok);
    cfg.dead_frac = need_double(&ok);
    cfg.allow_cross = need_int(&ok) != 0;
    cfg.aim_lookahead = need_double(&ok);
    cfg.front_back = need_int(&ok) != 0;
    cfg.fwd_speed = need_double(&ok);
    cfg.hold_clearance = need_double(&ok);
    cfg.has_forward = need_int(&ok) != 0;
    cfg.forward.x = need_double(&ok);
    cfg.forward.y = need_double(&ok);
    cfg.max_turn_rate = need_double(&ok);
    cfg.edge_margin = need_double(&ok);
    if (!ok) { std::fprintf(stderr, "CFG 解析失败：%s\n", g_err); return 2; }

    // ---- 逐帧 ----
    for (;;) {
        char buf[512];
        bool eof = false;
        if (!next_tok(buf, sizeof(buf), &eof)) break;      // 正常结束
        if (std::strcmp(buf, "FRAME") != 0) {
            std::fprintf(stderr, "第 %d 行：期望 FRAME，得到 `%s`\n", g_line, buf);
            return 2;
        }

        WorldView w{};
        w.dt = need_double(&ok);
        w.step_index = need_int(&ok);
        w.field_w = need_double(&ok);
        w.field_h = need_double(&ok);
        const double horizon = need_double(&ok);
        const long long n_rects = need_int(&ok);
        double player[7] = {0, 0, 0, 0, 0, 0, 1};
        for (int i = 0; i < 7; ++i) player[i] = need_double(&ok);
        if (!ok) { std::fprintf(stderr, "FRAME 解析失败：%s\n", g_err); return 2; }
        if (n_rects < 0 || n_rects > MAX_RECTS) {
            std::fprintf(stderr, "n_rects=%lld 超出 [0,%d]\n", n_rects, MAX_RECTS);
            return 2;
        }
        // 契约：v_max > 0（见 hpp）。Python 在 v_max == 0 时会抛 ZeroDivisionError，
        // C++ 不能抛，所以这里**显式拒绝**而不是悄悄给一个不同答案。
        if (!(player[P_SPEED] > 0.0)) {
            std::fprintf(stderr, "第 %d 行：player[P_SPEED]=%g <= 0，超出 plan() 契约\n",
                         g_line, player[P_SPEED]);
            return 2;
        }

        static Rect rects[MAX_RECTS];
        for (long long i = 0; i < n_rects; ++i) {
            char tag[64];
            bool e2 = false;
            if (!next_tok(tag, sizeof(tag), &e2) || std::strcmp(tag, "RECT") != 0) {
                std::fprintf(stderr, "第 %d 行：期望 RECT\n", g_line);
                return 2;
            }
            Rect& r = rects[i];
            r.ent_id = static_cast<int32_t>(need_int(&ok));
            r.x = need_double(&ok);
            r.y = need_double(&ok);
            r.vx = need_double(&ok);
            r.vy = need_double(&ok);
            r.half_w = need_double(&ok);
            r.half_h = need_double(&ok);
            r.rotation = need_double(&ok);
            r.angular_velocity = need_double(&ok);
            if (!ok) { std::fprintf(stderr, "RECT 解析失败：%s\n", g_err); return 2; }
        }

        w.player = player;
        w.rects = rects;
        w.n_rects = static_cast<size_t>(n_rects);

        const Decision d = plan(w, mem, cfg, horizon);
        std::printf("DEC %.17g %.17g %s %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %d %.17g\n",
                    d.wx, d.wy, d.tier, d.t_arr, d.clearance, d.gap, d.usable_half,
                    d.e_lo, d.e_hi, d.aim_lat, d.s_hat, d.n_rects, d.wall_band);
    }
    return 0;
}
