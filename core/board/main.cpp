// ===========================================================================
// main.cpp —— 板上 bring-up 程序：把视觉链路跑起来并打印几何帧
//
// 这是**上板第一步**要跑的东西，对应移植规划里的 P3。
// 它不依赖相机：`vision_axi` 的像素输入可以由 `vision_tpg`（自检图案）顶替，
// 也可以由真实像素流顶替 —— 本程序只关心 PS 这一侧。
//
// 用法（在板子的 Ubuntu 20.04 上）：
//
//     ./vision_board_app                       # 默认 8 帧
//     ./vision_board_app --frames 200          # 跑 200 帧，报吞吐
//     ./vision_board_app --device-url 'axi://zg330aiu?npu=0x40000000&dma=0x80000000'
//
// 输出每帧：seq / 帧长 / 障碍数 / 缺口数 / 角色是否有效 / 帧头 stamp，
// 以及**这一帧算出来的速度指令**。
//
// 决策已经接上了：`core/cpu/cpp/gap_avoid.*` 是 `core/cpu/gap_avoid.py` 的 C++ 移植
// （验收口径 `tier` 逐位一致 + 浮点量 rel_err < 1e-9，10 条交叉验证用例见
// `core/cpu/tests/test_cpp_gap_avoid.py`）。
// ===========================================================================
#include "gap_avoid.hpp"
#include "geometry_frame.hpp"
#include "vision_driver.hpp"
#include "vision_regs.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace vision_board;

namespace {

struct Args {
    std::string device_url = "axi://zg330aiu?npu=0x40000000&dma=0x80000000";
    int frames = 8;
    int timeout_ms = 2000;
    bool verbose = false;
    // 决策需要的两样东西视觉帧里**没有**，必须来自配置（CPP_PORT.md §6）：
    double field_w = 640.0;         // 场宽（世界单位）
    double field_h = 480.0;         // 场高
    double player_speed = 200.0;    // 速度上限
    double dt_default = 1.0 / 30.0; // 首帧没有上一帧 stamp 时用
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s 缺少取值\n", k.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (k == "--device-url")      a.device_url = next();
        else if (k == "--frames")     a.frames = std::atoi(next());
        else if (k == "--timeout-ms") a.timeout_ms = std::atoi(next());
        else if (k == "--verbose")    a.verbose = true;
        // 决策需要的场参数/速度上限：视觉帧不携带，只能从命令行来
        else if (k == "--field-w")      a.field_w = std::atof(next());
        else if (k == "--field-h")      a.field_h = std::atof(next());
        else if (k == "--player-speed") a.player_speed = std::atof(next());
        else if (k == "--dt-default")   a.dt_default = std::atof(next());
        else if (k == "-h" || k == "--help") {
            std::printf(
                "用法: %s [--device-url URL] [--frames N] [--timeout-ms MS] [--verbose]\n"
                "          [--field-w W] [--field-h H] [--player-speed V] [--dt-default DT]\n"
                "\n"
                "  --field-w/--field-h  场地尺寸（世界单位）。视觉帧**不携带**这两个量\n"
                "                       （标准 §6），所以必须由调用方给。默认 640x480。\n"
                "  --player-speed       玩家速度上限（世界单位/秒）。视觉帧里没有，默认 200。\n"
                "  --dt-default         首帧没有上一帧 stamp 时用的 dt，默认 1/30 s。\n",
                argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "不认识参数 %s（--help 看用法）\n", k.c_str());
            std::exit(2);
        }
    }
    return a;
}

// ---------------------------------------------------------------------------
// 决策：几何帧 -> 速度指令
//
// 数据流（CPP_PORT.md §6 的三条规矩都在这儿）：
//
//     几何帧 v3 字节 --parse_geometry_frame--> 障碍记录（矩形）
//         --只取 shape==rect--> gap_avoid::Rect[]  --plan()--> Decision
//
// ⚠ **dt 取两帧 `stamp` 之差**，不是帧号之差。帧号之差在掉帧时是错的，
//   而 `dt` 会进"缺口扫动速度"的估计。
// ⚠ **vel 一律填 0**：几何帧里**没有**速度字段（v3 线协议只有位置/尺寸/朝向）。
//   对静态墙这是精确的；对移动墙，`plan()` 里的 `v_n` 会退化成"只算车自己的速度"。
//   真正的修法是 PS 侧按 stamp 差分出速度再填进来 —— 那是下一步，不是这里。
// ⚠ **player/field 来自配置**：视觉帧不携带（标准 §6），用了默认值就要报出来。
// ---------------------------------------------------------------------------
struct DecisionState {
    gap_avoid::GapMemory mem;
    gap_avoid::AlgoConfig cfg;
    uint64_t prev_stamp_ns = 0;
    bool have_prev_stamp = false;
    int n_decisions = 0;
    int n_failed = 0;
    int tier_count[8] = {0, 0, 0, 0, 0, 0, 0, 0};   // 见 tier_bucket()
};

/// 把 tier 归到 8 个桶里（只为统计，不改语义）。
int tier_bucket(const char* t) {
    if (std::strcmp(t, "HOLD") == 0) return 0;
    if (std::strcmp(t, "TRIM") == 0) return 1;
    if (std::strcmp(t, "ALIGN") == 0) return 2;
    if (std::strcmp(t, "PANIC") == 0) return 3;
    if (std::strcmp(t, "SLOW") == 0) return 4;
    if (std::strcmp(t, "STOP") == 0) return 5;
    if (std::strcmp(t, "NO_GAP") == 0) return 6;
    return 7;
}

const char* kTierNames[8] = {"HOLD", "TRIM", "ALIGN", "PANIC",
                             "SLOW", "STOP", "NO_GAP", "其它"};

void decide(const GeometryFrame& f, const Args& args, DecisionState& st) {
    // ---- dt：两帧 stamp 之差 ----
    double dt = args.dt_default;
    bool dt_is_default = true;
    if (st.have_prev_stamp && f.header.stamp_ns > st.prev_stamp_ns) {
        dt = static_cast<double>(f.header.stamp_ns - st.prev_stamp_ns) * 1e-9;
        dt_is_default = false;
    }
    st.prev_stamp_ns = f.header.stamp_ns;
    st.have_prev_stamp = true;

    // ---- 角色：几何帧里没有速度，恒 0 ----
    const double player[7] = {
        f.player.present ? f.player.x : args.field_w * 0.5,
        f.player.present ? f.player.y : 0.0,
        0.0, 0.0,
        f.player.present ? f.player.radius : 10.0,
        args.player_speed,
        1.0,
    };

    // ---- 障碍：只取矩形（决策的 group_walls 只认矩形）----
    std::vector<gap_avoid::Rect> rects;
    rects.reserve(f.obstacles.size());
    for (const GeometryObstacle& o : f.obstacles) {
        if (!o.is_rect()) continue;
        gap_avoid::Rect r;
        r.ent_id = o.id;
        r.x = o.x;
        r.y = o.y;
        r.vx = 0.0;              // 见上面的说明：线协议不带速度
        r.vy = 0.0;
        r.half_w = o.half_w;
        r.half_h = o.half_h;
        r.rotation = o.rotation;
        r.angular_velocity = 0.0;
        rects.push_back(r);
    }

    gap_avoid::WorldView world;
    world.player = player;
    world.rects = rects.data();
    world.n_rects = rects.size();
    world.dt = dt;
    world.step_index = static_cast<int64_t>(f.header.seq);
    world.field_w = args.field_w;
    world.field_h = args.field_h;

    const gap_avoid::Decision d = gap_avoid::plan(world, st.mem, st.cfg, 2.0);
    const gap_avoid::Vec2 cmd = gap_avoid::to_command(d);
    ++st.n_decisions;
    st.tier_count[tier_bucket(d.tier)] += 1;

    std::printf("        决策: tier=%-7s 指令=(%+.3f, %+.3f)  "
                "t_arr=%.2fs 间隙=%+.1f 可用半宽=%+.1f 矩形=%d",
                d.tier, cmd.x, cmd.y,
                std::isfinite(d.t_arr) ? d.t_arr : -1.0,
                std::isfinite(d.gap) ? d.gap : -1.0,
                std::isfinite(d.usable_half) ? d.usable_half : -1.0,
                d.n_rects);
    if (rects.empty()) std::printf("  [无矩形障碍]");
    if (!f.player.present) std::printf("  [无角色，按场宽中线/半径 10 兜底]");
    if (dt_is_default) std::printf("  [dt=默认，无上一帧 stamp]");
    std::printf("\n");
}

}   // namespace

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);

    std::printf("=== vision board bring-up ===\n");
    std::printf("设备: %s\n", args.device_url.c_str());

    // ---- 打开 PL ---------------------------------------------------------
    RegIo* io = nullptr;
    try {
        io = icraft_reg_io(args.device_url);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "打开设备失败：%s\n"
                             "  排查：位流烧了吗？user_op 窗口是 0x400C0000 吗？\n",
                     e.what());
        return 1;
    }
    VisionDriver drv(*io);

    // ---- 自检：写得进、读得出 -------------------------------------------
    std::string why;
    if (!drv.self_test(&why)) {
        std::fprintf(stderr, "自检失败：%s\n", why.c_str());
        std::fprintf(stderr,
                     "  这条失败通常意味着：基址不对 / 位流没烧 / AXI 没接上。\n"
                     "  先用 xsct 确认 PL 能认到（jtag targets 应看到 xc7z030）。\n");
        return 1;
    }
    std::printf("自检：通过（配置寄存器读写回一致）\n");

    // ---- 配置 ------------------------------------------------------------
    VisionParams params;          // 默认值 = C 参考模型 / Python 真值
    drv.configure(params);
    std::printf("配置：已写入 %d 个寄存器（阈值用默认值）\n", (int)kCfgCount);

    // ---- 逐帧 ------------------------------------------------------------
    DecisionState st;
    const auto t0 = std::chrono::steady_clock::now();
    int ok = 0, bad = 0;
    uint64_t stamp_ns = 0;
    for (int i = 0; i < args.frames; ++i) {
        std::vector<uint8_t> bytes;
        try {
            bytes = drv.run_frame(static_cast<uint32_t>(i), stamp_ns,
                                  args.timeout_ms);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "第 %d 帧失败：%s\n", i, e.what());
            ++bad;
            continue;
        }
        try {
            const GeometryFrame frame = parse_geometry_frame(bytes);
            const FrameHeader& h = frame.header;
            ++ok;
            std::printf("帧 %4d: seq=%u  长度=%zu B  障碍=%u  缺口=%u  "
                        "尺寸=%ux%u  stamp=%llu ns%s\n",
                        i, h.seq, bytes.size(), h.n_obs, h.n_gaps,
                        h.img_w, h.img_h,
                        static_cast<unsigned long long>(h.stamp_ns),
                        h.flags & 1u ? "  [player_valid]" : "");
            decide(frame, args, st);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "帧 %d 解码失败：%s\n", i, e.what());
            ++bad;
        }
        if (drv.overflow()) {
            std::fprintf(stderr,
                         "⚠ 帧缓冲溢出位置位 —— 上一帧比窗口(2048B)还长。\n"
                         "  查 n_obs/n_gaps 是不是爆了（障碍上限 32、缺口上限 16）。\n");
        }
        stamp_ns += 33'333'333ull;      // 30 fps 的步长
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    std::printf("----\n成功 %d 帧，失败 %d 帧，用时 %lld ms", ok, bad,
                static_cast<long long>(ms));
    if (ms > 0) {
        std::printf("（%.1f 帧/秒）", 1000.0 * ok / static_cast<double>(ms));
    }
    std::printf("\n");
    if (st.n_decisions > 0) {
        std::printf("决策 %d 帧（失败 %d）: ", st.n_decisions, st.n_failed);
        for (int i = 0; i < 8; ++i) {
            if (st.tier_count[i] > 0) {
                std::printf("%s=%d ", kTierNames[i], st.tier_count[i]);
            }
        }
        std::printf("\n");
    }

    delete io;      // icraft 后端是 new 出来的；板子上也可以改成栈上对象
    return bad == 0 ? 0 : 1;
}
