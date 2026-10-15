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
#include "action_codec.hpp"
#include "ppm_reader.hpp"

// C 参考模型（黄金模型）。板子上编它 = 给"真摄像头"一条**不需要 PL** 的通路，
// 同时也是 PL 那侧的参照（协同仿真里同一个函数就是判据）。
extern "C" {
#include "ref_api.h"
}
#include "geometry_frame.hpp"
#include "vision_driver.hpp"
#include "vision_regs.hpp"

#include <chrono>
#include <memory>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

using namespace vision_board;

namespace {

//: PL 流水线的固定尺寸（vision_axi.v 的 WIDTH/HEIGHT 默认值）。
//: 走 `--via-pl` 时输入的图必须**恰好**这么大 —— 不符就早报，别喂进去。
constexpr int kPlWidth  = 640;
constexpr int kPlHeight = 360;

struct Args {
    // 板子上 PL 的寄存器窗口用 /dev/mem 直接访问（板上工具已验证）。
    // icraft 的 defaultRegRegion 是**另一个窗口**，不是我们这块 PL。
    std::string device_url = "devmem://0x43C00000";
    int frames = 8;
    int timeout_ms = 2000;
    bool verbose = false;
    // 决策需要的两样东西视觉帧里**没有**，必须来自配置（CPP_PORT.md §6）：
    double field_w = 640.0;         // 场宽（世界单位）
    double field_h = 480.0;         // 场高
    double player_speed = 200.0;    // 速度上限
    double dt_default = 1.0 / 30.0; // 首帧没有上一帧 stamp 时用
    // 键盘桥：决策算出的速度 → 动作号 → "@MOVE <n>\n" → 板子控制台 TX。
    // RP2040 的 GP1 就接在控制台 TX 上（见 core/rp2040_keyboard/README.md），
    // 它从共享的控制台流里挑出带 @MOVE 前缀的行。空串 = 不发。
    // 几何帧的来源：PL（默认，经 AXI 读窗口）或 PPM（PS 上跑 C 参考模型）。
    // 后者是**真摄像头那条路**的入口：
    //     ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 out.ppm
    //     ./vision_board_devmem --ppm out.ppm
    // 也是 PL 拿不到时的参照实现。
    std::string ppm_file;           // 单张图（每帧重复用它）
    std::string ppm_dir;            // 目录（%06d.ppm，与协同仿真的激励同命名）
    //: `--via-pl`：PPM 的像素**推进 PL 的 FIFO**，几何帧由 PL 产出
    //: （而不是在 PS 上跑黄金模型）。真摄像头要走 FPGA 加速时用这个。
    bool via_pl = false;
    //: `--verify-both`：同一张图**两条路都跑一遍**（PL 推像素 + PS 黄金模型），
    //: 比对几何帧是否**逐字节相同**。这是"PL 吃真图"在真硅上的等价性判据
    //: —— 协同仿真的 `tb_axi --fifo` 证的是同一件事。
    bool verify_both = false;

    std::string keyboard_dev = "/dev/ttyPS0";
    double action_deadzone = kDefaultDeadzone;
    bool echo_action = false;       // 把动作号也打到 stdout（人看）
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
        else if (k == "--ppm")          a.ppm_file = next();
        else if (k == "--via-pl")       a.via_pl = true;
        else if (k == "--verify-both")  a.verify_both = true;
        else if (k == "--ppm-dir")      a.ppm_dir = next();
        else if (k == "--keyboard-dev") a.keyboard_dev = next();
        else if (k == "--no-keyboard")  a.keyboard_dev.clear();
        else if (k == "--action-deadzone") a.action_deadzone = std::atof(next());
        else if (k == "--echo-action")  a.echo_action = true;
        else if (k == "-h" || k == "--help") {
            std::printf(
                "用法: %s [--device-url URL] [--frames N] [--timeout-ms MS] [--verbose]\n"
                "          [--field-w W] [--field-h H] [--player-speed V] [--dt-default DT]\n"
                "\n"
                "  --field-w/--field-h  场地尺寸（世界单位）。视觉帧**不携带**这两个量\n"
                "                       （标准 §6），所以必须由调用方给。默认 640x480。\n"
                "  --player-speed       玩家速度上限（世界单位/秒）。视觉帧里没有，默认 200。\n"
                "  --dt-default         首帧没有上一帧 stamp 时用的 dt，默认 1/30 s。\n"
                "  --keyboard-dev P     把动作号写成 \"@MOVE <n>\\n\" 发到这个串口\n"
                "                       （默认 /dev/ttyPS0 = 板子控制台 TX；RP2040 的 GP1\n"
                "                        就接在这根线上，从控制台流里挑 @MOVE 行）。\n"
                "  --no-keyboard        不发动作（只看视觉+决策）。\n"
                "  --action-deadzone D  速度小于 D 就当 stay，默认 1e-3。\n"
                "  --echo-action        把动作号也打到 stdout。\n"
                "  --ppm FILE           几何帧改由 PS 上跑 C 参考模型产出（单张图，重复用）。\n"
                "                       **真摄像头就走这条**：\n"
                "                         ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 o.ppm\n"
                "                         %s --ppm o.ppm\n"
                "  --ppm-dir DIR        同上，但按 000000.ppm/000001.ppm/... 依次取\n"
                "                       （与 data/fpga/stim_* 的命名一致）。\n"
                "  --via-pl             配上 --ppm/--ppm-dir 时，把像素**推进 PL**、\n"
                "                       几何帧由 PL 产出（真摄像头走 FPGA 加速就走这条）。\n"
                "                       不加则是在 PS 上跑黄金模型。\n"
                "  --verify-both        同一张图两条路都跑，比对几何帧是否逐字节相同\n"
                "                       （真硅上验「PL 吃真图」，与 tb_axi --fifo 同一性质）。\n",
                argv[0], argv[0]);
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

// ---------------------------------------------------------------------------
// PS 侧参考模型前端：PPM → 几何帧字节
//
// 产出的是**和 PL 逐位相同**的几何帧（协同仿真里 RTL 就是对着这个函数比的），
// 所以下游的解码 / 决策 / 键盘一行都不用改 —— 换的只是"帧从哪来"。
//
// 这条路的用途有两个：
//   ① **真摄像头**：板子上有 v4l2-ctl/ffmpeg，采一帧存 PPM 就能直接跑；
//   ② **参照**：PL 那侧出问题时，用同一张图跑一遍 PS 版，就能判断是
//      "算法/决策不对"还是"PL 通路不对"。
// ---------------------------------------------------------------------------
class RefFrameSource {
public:
    explicit RefFrameSource(const Args& args)
        : file_(args.ppm_file), dir_(args.ppm_dir) {
        vr_ref_default_params(&params_);
        rgb_.reserve(1024 * 1024);
        gray_.resize(1024 * 1024);
        wall_.resize(1024 * 1024);
        bright_.resize(1024 * 1024);
        rowc_.resize(4096);
        frame_.resize(8192);
    }

    //: 取第 i 帧的几何帧字节。失败返回 false 并把原因写进 why。
    bool next(int i, uint32_t seq, uint64_t stamp_ns,
              std::vector<uint8_t>* out, std::string* why) {
        std::string path;
        if (!dir_.empty()) {
            char name[32];
            std::snprintf(name, sizeof(name), "%06d.ppm", i);
            path = dir_ + "/" + name;
        } else {
            path = file_;
        }
        if (!vision_board::read_ppm_p6(path, &w_, &h_, &rgb_, why)) return false;
        if (w_ <= 0 || h_ <= 0) { *why = "图尺寸非法"; return false; }
        if (static_cast<size_t>(w_) * static_cast<size_t>(h_) > gray_.size()) {
            *why = "图太大（缓冲只准备了 1M 像素）";
            return false;
        }

        VrRefResult res;
        std::memset(&res, 0, sizeof(res));
        const int rc = vr_ref_process_ex(rgb_.data(), w_, h_, 1,
                                         gray_.data(), wall_.data(),
                                         bright_.data(), rowc_.data(),
                                         &res, frame_.data(), frame_.size(),
                                         &params_, seq, stamp_ns);
        if (rc != 0) { *why = "参考模型返回 " + std::to_string(rc); return false; }
        out->assign(frame_.begin(), frame_.begin() + res.frame_bytes);
        return true;
    }

    int width() const { return w_; }
    int height() const { return h_; }

private:
    std::string file_, dir_;
    VrRefParams params_{};
    std::vector<uint8_t> rgb_, gray_, wall_, bright_, frame_;
    std::vector<uint16_t> rowc_;
    int w_ = 0, h_ = 0;
};

// ---------------------------------------------------------------------------
// 动作发射：把动作号写成 "@MOVE <n>\n" 发到板子控制台 TX
//
// 为什么发到**控制台**而不是另开一个串口：RP2040 的 GP1 就接在控制台 TX 上
// （见 core/rp2040_keyboard/README.md，只两根线：TX + GND）。它从共享的控制台
// 流里挑出带 @MOVE 前缀的行、忽略内核日志与 shell 输出。
//
// O_NONBLOCK 是刻意的：串口拥塞时**宁可丢这一帧的动作**，也不能把决策循环堵住
// —— 键盘桥那边有看门狗（松开所有键），漏一帧不会卡住角色，堵住决策循环会。
// ---------------------------------------------------------------------------
class ActionEmitter {
public:
    explicit ActionEmitter(const std::string& dev) {
        if (dev.empty()) return;                 // 显式关掉
        fd_ = ::open(dev.c_str(), O_WRONLY | O_NONBLOCK | O_NOCTTY);
        if (fd_ < 0) {
            std::fprintf(stderr, "⚠ 打不开键盘串口 %s：%s（这一轮不发动作）\n",
                         dev.c_str(), std::strerror(errno));
        }
    }
    ~ActionEmitter() { if (fd_ >= 0) ::close(fd_); }
    bool ok() const { return fd_ >= 0; }

    //: 返回写出去的字节数；<0 表示这一帧丢了（已打点）。
    int emit(int action, bool noisy) {
        if (fd_ < 0) return -1;
        char buf[16];
        const int n = vision_board::format_action_line(action, buf, sizeof(buf));
        if (n <= 0) return -1;
        const ssize_t w = ::write(fd_, buf, static_cast<size_t>(n));
        if (w != n) {
            ++dropped_;
            if (noisy && dropped_ <= 3) {
                std::fprintf(stderr, "⚠ 动作行没写完（%zd/%d，errno=%s）—— 丢掉这一帧\n",
                             w, n, std::strerror(errno));
            }
            return -1;
        }
        ++sent_;
        return n;
    }
    long sent() const { return sent_; }
    long dropped() const { return dropped_; }

private:
    int fd_ = -1;
    long sent_ = 0, dropped_ = 0;
};

gap_avoid::Vec2 decide(const GeometryFrame& f, const Args& args, DecisionState& st) {
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
    return cmd;
}

}   // namespace

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);

    std::printf("=== vision board bring-up ===\n");
    std::printf("设备: %s\n", args.device_url.c_str());

    const bool have_ppm = !args.ppm_file.empty() || !args.ppm_dir.empty();
    // 走 PL 时**不**用 PS 参考模型：像素推给 PL，几何帧从 PL 读回来
    const bool use_ref = have_ppm && !args.via_pl && !args.verify_both;

    // ---- 打开 PL（只有走 PL 那条路才需要）--------------------------------
    RegIo* io = nullptr;
    if (!use_ref) {
    try {
        io = open_reg_io(args.device_url);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "打开设备失败：%s\n"
                             "  排查：位流烧了吗？PL 的寄存器窗口是不是 0x43C00000？\n"
                             "  （基址的契约：vision_regs.hpp 的 kRegBase 与\n"
                             "    core/fpga/synth/bd_vision.tcl 里显式钉死的那一段）\n",
                     e.what());
        return 1;
    }
    }
    std::unique_ptr<VisionDriver> drv;
    std::unique_ptr<RefFrameSource> ref;
    // ⚠ 这两段必须是**两个独立的 if**，不能写成 if/else：
    //   `--verify-both` 要**同时**建 ref 与 drv（两条路都要跑）。写成 else 的话
    //   只建了 ref，而帧循环里会去调 `drv->run_frame_push(...)` → **空指针段错误**
    //   （实测 exit 139，而且现象是"打印看起来正常、下一行就崩"，不好定位）。
    if (use_ref || args.verify_both) {
        ref.reset(new RefFrameSource(args));
        std::printf("帧来源: PS 参考模型 (%s)\n",
                    args.ppm_dir.empty() ? args.ppm_file.c_str() : args.ppm_dir.c_str());
    }
    if (!use_ref) {
        drv.reset(new VisionDriver(*io));
        if (args.verify_both) {
            std::printf("帧来源: **两条都跑**（PL 推像素 vs PS 黄金模型，逐字节比）\n");
            drv->set_pixel_source(true);
            drv->flush_pixels();
        } else if (have_ppm) {
            std::printf("帧来源: PL（像素由 PS 推入 FIFO，帧取自 %s）\n",
                        args.ppm_dir.empty() ? args.ppm_file.c_str() : args.ppm_dir.c_str());
            drv->set_pixel_source(true);
            drv->flush_pixels();
        } else {
            std::printf("帧来源: PL（外部 din_*，板上是 tpg_source）\n");
        }
    }

    std::string why;
    if (drv) {
        // ---- 自检：写得进、读得出 ---------------------------------------
        if (!drv->self_test(&why)) {
            std::fprintf(stderr, "自检失败：%s\n", why.c_str());
            std::fprintf(stderr,
                         "  这条失败通常意味着：基址不对 / 位流没烧 / AXI 没接上。\n"
                         "  或者用 --ppm/--ppm-dir 走 PS 参考模型那条路。\n");
            return 1;
        }
        std::printf("自检：通过（配置寄存器读写回一致）\n");

        // ---- 配置 --------------------------------------------------------
        VisionParams params;      // 默认值 = C 参考模型 / Python 真值
        drv->configure(params);
        std::printf("配置：已写入 %d 个寄存器（阈值用默认值）\n", (int)kCfgCount);
    }

    // ---- 键盘桥：动作行发到板子控制台 TX（RP2040 就听这根线）--------------
    long cmp_total = 0, cmp_ok = 0, cmp_bad = 0;   // --verify-both 的对账计数

    ActionEmitter kb(args.keyboard_dev);
    if (!args.keyboard_dev.empty()) {
        std::printf("键盘桥: %s（%s）\n", args.keyboard_dev.c_str(),
                    kb.ok() ? "已打开，发 \"@MOVE <n>\" " : "**打不开**，本轮不发动作");
    } else {
        std::printf("键盘桥: 已关闭（--no-keyboard）\n");
    }

    // ---- 逐帧 ------------------------------------------------------------
    DecisionState st;
    const auto t0 = std::chrono::steady_clock::now();
    int ok = 0, bad = 0;
    uint64_t stamp_ns = 0;
    for (int i = 0; i < args.frames; ++i) {
        std::vector<uint8_t> bytes;
        std::vector<uint8_t> bytes_ref;      // --verify-both 时用来对账
        if (args.verify_both) {
            // 先跑 PS 黄金模型（纯计算，不动硬件），再让 PL 跑同一张图。
            // 两张几何帧必须**逐字节相同** —— 不同就说明 PL 那条路有问题，
            // 而且差异会直接指出是哪个字段（这里打前 32 字节）。
            std::string path = args.ppm_dir.empty()
                             ? args.ppm_file
                             : (args.ppm_dir + "/" + [&] {
                                   char n[32]; std::snprintf(n, sizeof(n), "%06d.ppm", i);
                                   return std::string(n);
                               }());
            std::string rwhy;
            if (!ref->next(i, static_cast<uint32_t>(i), stamp_ns, &bytes_ref, &rwhy)) {
                std::fprintf(stderr, "第 %d 帧：参考模型失败：%s\n", i, rwhy.c_str());
                ++bad; break;
            }
            std::vector<uint8_t> rgb;
            int pw = 0, ph = 0;
            std::string why2;
            if (!vision_board::read_ppm_p6(path, &pw, &ph, &rgb, &why2)) {
                std::fprintf(stderr, "第 %d 帧：%s\n", i, why2.c_str());
                ++bad; break;
            }
            try {
                bytes = drv->run_frame_push(rgb.data(), rgb.size(),
                                            static_cast<uint32_t>(i), stamp_ns,
                                            args.timeout_ms);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "第 %d 帧：PL 推像素失败：%s\n", i, e.what());
                ++bad; continue;
            }
            ++cmp_total;
            if (bytes.size() != bytes_ref.size()) {
                ++cmp_bad;
                std::printf("  ✗ 帧 %d 长度不一致：PL=%zu，参考=%zu\n",
                            i, bytes.size(), bytes_ref.size());
            } else if (bytes != bytes_ref) {
                ++cmp_bad;
                size_t k = 0;
                while (k < bytes.size() && bytes[k] == bytes_ref[k]) ++k;
                std::printf("  ✗ 帧 %d 内容不一致（第一个差异在字节 %zu）：\n", i, k);
                std::printf("      PL  :"); for (size_t t = k; t < k + 16 && t < bytes.size(); ++t) std::printf(" %02X", bytes[t]);
                std::printf("\n      参考:"); for (size_t t = k; t < k + 16 && t < bytes_ref.size(); ++t) std::printf(" %02X", bytes_ref[t]);
                std::printf("\n");
            } else {
                ++cmp_ok;
                std::printf("  ✓ 帧 %d：PL 与 PS 黄金模型**逐字节相同**（%zu 字节）\n",
                            i, bytes.size());
            }
        } else if (drv && have_ppm) {
            // 真摄像头那条路：读 PPM → 推给 PL 的像素 FIFO → 从 PL 取几何帧。
            // PL 的流水线尺寸是编译期常量（WIDTH/HEIGHT），所以这里必须**恰好**
            // 是那个尺寸 —— 不符就直接说清楚，别喂进去得到一堆看不懂的错。
            std::string path = args.ppm_dir.empty()
                             ? args.ppm_file
                             : (args.ppm_dir + "/" + [&] {
                                   char n[32]; std::snprintf(n, sizeof(n), "%06d.ppm", i);
                                   return std::string(n);
                               }());
            std::vector<uint8_t> rgb;
            int pw = 0, ph = 0;
            std::string why2;
            if (!vision_board::read_ppm_p6(path, &pw, &ph, &rgb, &why2)) {
                std::fprintf(stderr, "第 %d 帧失败：%s\n", i, why2.c_str());
                ++bad; break;
            }
            if (pw != kPlWidth || ph != kPlHeight) {
                std::fprintf(stderr,
                    "第 %d 帧：图是 %dx%d，而 PL 的流水线固定为 %dx%d —— 尺寸不符\n"
                    "  （PL 的 WIDTH/HEIGHT 是编译期常量；要换尺寸得改 RTL 重建位流）\n",
                    i, pw, ph, kPlWidth, kPlHeight);
                ++bad; break;
            }
            try {
                bytes = drv->run_frame_push(rgb.data(), rgb.size(),
                                            static_cast<uint32_t>(i), stamp_ns,
                                            args.timeout_ms);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "第 %d 帧失败（推像素）：%s\n", i, e.what());
                ++bad; continue;
            }
        } else if (ref) {
            std::string rwhy;
            if (!ref->next(i, static_cast<uint32_t>(i), stamp_ns, &bytes, &rwhy)) {
                std::fprintf(stderr, "第 %d 帧失败（参考模型）：%s\n", i, rwhy.c_str());
                ++bad;
                break;                 // ppm-dir 取完就停，不是错误
            }
        } else {
            try {
                bytes = drv->run_frame(static_cast<uint32_t>(i), stamp_ns,
                                       args.timeout_ms);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "第 %d 帧失败：%s\n", i, e.what());
                ++bad;
                continue;
            }
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
            const gap_avoid::Vec2 cmd = decide(frame, args, st);
            const int action = vision_board::action_from_velocity(
                cmd.x, cmd.y, args.action_deadzone);
            if (args.echo_action) {
                std::printf("        动作: %d (%s)\n", action,
                            vision_board::action_name(action));
            }
            kb.emit(action, args.verbose);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "帧 %d 解码失败：%s\n", i, e.what());
            ++bad;
        }
        if (drv && drv->overflow()) {
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
    if (args.verify_both) {
        // ---- 配置对账：PS 写进去的 21 个寄存器 vs 参考模型的默认参数 ----
        //
        // 为什么单列这一步：**协同仿真测不到它** —— `tb_axi` 两边都用 C 参考模型的
        // 默认值，所以"PS 侧那份结构体的默认值 / 下标映射"从来没被验过。而板上
        // 写进去的是 C++ 的 `VisionParams`。两者只要有一项不同，PL 与模型的
        // 判据就会不同 → 几何帧"个数对、坐标不对"，且**完全不报错**。
        {
            VrRefParams rp;
            vr_ref_default_params(&rp);
            const int32_t want[21] = {
                rp.wall_rb, rp.wall_min_r, rp.wall_min_g, rp.bright_min,
                rp.bright_spread, rp.green_gm, rp.green_min, rp.band_row_pct,
                rp.band_join_rows, rp.band_min_rows, rp.field_col_min,
                rp.wall_col_pct, rp.wall_col_min, rp.edge_min, rp.edge_min_cols,
                rp.min_seg_px, rp.min_gap_px, rp.player_min_area,
                rp.player_max_area, rp.player_min_fill_pct, rp.player_max_aspect_pct,
            };
            const char* nm[21] = {
                "wall_rb", "wall_min_r", "wall_min_g", "bright_min", "bright_spread",
                "green_gm", "green_min", "band_row_pct", "band_join_rows",
                "band_min_rows", "field_col_min", "wall_col_pct", "wall_col_min",
                "edge_min", "edge_min_cols", "min_seg_px", "min_gap_px",
                "player_min_area", "player_max_area", "player_min_fill_pct",
                "player_max_aspect_pct",
            };
            int ncfg_bad = 0;
            for (int j = 0; j < 21; ++j) {
                const uint32_t got = drv->read_reg(kOffConfig + 4u * (uint32_t)j);
                if (static_cast<int32_t>(got) != want[j]) {
                    std::printf("  ✗ 配置第 %2d 项 %-22s PL=%d  参考=%d\n",
                                j, nm[j], static_cast<int>(got), want[j]);
                    ++ncfg_bad;
                }
            }
            std::printf("---- 配置对账：21 项，不符 %d ----\n", ncfg_bad);
            if (ncfg_bad) ++bad;
        }
        std::printf("---- 等价性对账：%ld 帧，逐字节相同 %ld，不符 %ld ----\n",
                    cmp_total, cmp_ok, cmp_bad);
        if (cmp_bad) ++bad;
    }
    if (!args.keyboard_dev.empty()) {
        std::printf("键盘桥: 发出 %ld 行，丢 %ld 行\n", kb.sent(), kb.dropped());
    }
    if (st.n_decisions > 0) {
        std::printf("决策 %d 帧（失败 %d）: ", st.n_decisions, st.n_failed);
        for (int i = 0; i < 8; ++i) {
            if (st.tier_count[i] > 0) {
                std::printf("%s=%d ", kTierNames[i], st.tier_count[i]);
            }
        }
        std::printf("\n");
    }

    delete io;      // icraft/devmem 后端是 new 出来的
    return bad == 0 ? 0 : 1;
}
