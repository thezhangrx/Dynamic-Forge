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
#include "fiducial_zone.hpp"
#include "gap_avoid.hpp"
#include "motion.hpp"
#include "action_codec.hpp"
#include "ppm_reader.hpp"
#include "rgb_source.hpp"

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
#include <netdb.h>
#include <sys/socket.h>
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
    //: 四角标定标记（平台契约，`core/standard/fiducial.py`）：
    //: 黑边外沿边长 + 到场地边的距离。板子用它们**排除"把标记当角色"**。
    double fiducial_side = 48.0;
    double fiducial_margin = 8.0;
    //: 标记周围还要额外留多少（场地单位）才算"这是标记不是角色"。
    //: 相机有透视/量化误差，标记的留白边也比黑边外沿大一圈（48 → 64）。
    double fiducial_slack = 12.0;
    //: 逐像素分类的 7 个阈值**覆盖**（负数 = 用 C 参考模型/Python 真值的默认值）。
    //:
    //: 为什么要有这个：判据是**绝对阈值**（`r-b>40` 这类），而相机拍屏幕会把
    //: 橙墙洗成近中性（实测 r-b 只剩 20~57）、把黑底抬成中灰。平台上只有橙/白/黑
    //: 三种颜色，所以按真实相机**放宽/收紧**这几个数就能救回来 —— 不用重编位流，
    //: 也不动场景（墙色、角色大小、场地尺寸全是契约）。
    int ov_wall_rb = -1, ov_wall_min_r = -1, ov_wall_min_g = -1;
    int ov_bright_min = -1, ov_bright_spread = -1;
    int ov_green_gm = -1, ov_green_min = -1;
    bool vision_calibrated() const {
        return ov_wall_rb >= 0 || ov_wall_min_r >= 0 || ov_wall_min_g >= 0 ||
               ov_bright_min >= 0 || ov_bright_spread >= 0 ||
               ov_green_gm >= 0 || ov_green_min >= 0;
    }
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
    std::string track_log_path;     // --track-log 的路径（main 里开成 FILE*）
    //: 离线帧序列（--ppm/--ppm-dir）合成时间戳用的帧率。
    //: ⚠ 必须与素材的真实帧间隔一致，否则推出来的速度会整体差一个倍数：
    //:   例：60fps 的素材按 30fps 打戳 → 速度算小一半（而决策的 t_arr 直接吃它）。
    //: 实时路径（--stdin-rgb）**不用**这个值 —— 那边用读线程打下的真实采集时刻。
    double ppm_fps = 30.0;
    //: 逐帧轨迹日志（`--track-log FILE`）：原始矩形 + 跟踪器状态。
    //: 只为**验证速度估计**用：光看"推出来的速度"没法区分"东西真在动"和
    //: "检测框在抖" —— 必须把原始位置一起落盘才能离线算差分。
    std::FILE* track_log = nullptr;
    //: `--via-pl`：PPM 的像素**推进 PL 的 FIFO**，几何帧由 PL 产出
    //: （而不是在 PS 上跑黄金模型）。真摄像头要走 FPGA 加速时用这个。
    bool via_pl = false;
    //: `--stdin-rgb`：帧从**标准输入**读原始 RGB24（`W*H*3` 字节一帧、无头）。
    //: 实时采集走这条 —— 一次开设备、连续出帧，不像 `--ppm` 那样每帧重开设备。
    //:    ffmpeg ... -pix_fmt rgb24 -f rawvideo - | ./vision_board_devmem --stdin-rgb
    //: `--timing`：每帧打印分段耗时（读像素 / 推 PL / 等流水线），并在末尾给平均。
    //: 实时那条路必须知道"慢在哪一段"——否则只能靠猜。
    bool timing = false;
    bool stdin_rgb = false;
    //: 丢掉开头几帧：摄像头自动曝光要好几帧才收敛（实测第 5~9 帧才稳，早期抓到过
    //: 全黑的第 1 帧）。**不是可选项** —— 不丢就是拿到黑图。
    int skip_frames = 10;
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
        else if (k == "--fiducial-side")   a.fiducial_side = std::atof(next());
        else if (k == "--fiducial-margin") a.fiducial_margin = std::atof(next());
        else if (k == "--fiducial-slack")  a.fiducial_slack = std::atof(next());
        // 分类阈值覆盖（--wall-rb 等；不给就用默认值）
        else if (k == "--wall-rb")        a.ov_wall_rb = std::atoi(next());
        else if (k == "--wall-min-r")     a.ov_wall_min_r = std::atoi(next());
        else if (k == "--wall-min-g")     a.ov_wall_min_g = std::atoi(next());
        else if (k == "--bright-min")     a.ov_bright_min = std::atoi(next());
        else if (k == "--bright-spread")  a.ov_bright_spread = std::atoi(next());
        else if (k == "--green-gm")       a.ov_green_gm = std::atoi(next());
        else if (k == "--green-min")      a.ov_green_min = std::atoi(next());
        else if (k == "--ppm")          a.ppm_file = next();
        else if (k == "--via-pl")       a.via_pl = true;
        else if (k == "--stdin-rgb")    a.stdin_rgb = true;
        else if (k == "--timing")       a.timing = true;
        else if (k == "--skip-frames")  a.skip_frames = std::atoi(next());
        else if (k == "--verify-both")  a.verify_both = true;
        else if (k == "--ppm-dir")      a.ppm_dir = next();
        else if (k == "--track-log")    a.track_log_path = next();
        else if (k == "--ppm-fps")      a.ppm_fps = std::atof(next());
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
                "  --keyboard-dev T     动作号发到哪儿。三种写法：\n"
                "                         /dev/ttyPS0        （串口，RP2040 的 GP1 听这根线）\n"
                "                         udp://192.168.137.1:9999 （**走网口**，不用接任何线）\n"
                "                         stdout             （打到标准输出，配 SSH 管道：\n"
                "                                             ssh 板子 '... --keyboard-dev stdout'\n"
                "                                               | pc_key_driver.py --stdin）\n"
                "                       （默认 /dev/ttyPS0 = 板子控制台 TX；RP2040 的 GP1\n"
                "                        就接在这根线上，从控制台流里挑 @MOVE 行）。\n"
                "  --no-keyboard        不发动作（只看视觉+决策）。\n"
                "  --action-deadzone D  速度小于 D 就当 stay，默认 1e-3。\n"
                "  --echo-action        把动作号也打到 stdout。\n"
                "  --track-log FILE     逐帧把原始矩形与跟踪器状态写成文本（验证速度估计用：\n"
                "                       只有原始位置落盘，才分得清「真在动」和「框在抖」）。\n"
                "  --ppm-fps F          离线帧序列合成时间戳用的帧率，默认 30。\n"
                "                       ⚠ 要和素材真实帧间隔一致，否则速度整体差一个倍数\n"
                "                       （60fps 素材按 30 打戳 → 速度算小一半）。\n"
                "                       实时路径不用它：那边用读线程打下的真实采集时刻。\n"
                "  --ppm FILE           几何帧改由 PS 上跑 C 参考模型产出（单张图，重复用）。\n"
                "                       **真摄像头就走这条**：\n"
                "                         ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 o.ppm\n"
                "                         %s --ppm o.ppm\n"
                "  --ppm-dir DIR        同上，但按 000000.ppm/000001.ppm/... 依次取\n"
                "                       （与 data/fpga/stim_* 的命名一致）。\n"
                "  --via-pl             配上 --ppm/--ppm-dir 时，把像素**推进 PL**、\n"
                "                       几何帧由 PL 产出（真摄像头走 FPGA 加速就走这条）。\n"
                "                       不加则是在 PS 上跑黄金模型。\n"
                "  --wall-rb N          覆盖「墙」的 R-B 下限（默认 40）。相机把橙墙洗成\n"
                "                       近中性时（实测 r-b 只剩 20~57）把它放宽到 20 左右；\n"
                "                       平台上只有橙/白/黑三种颜色，放宽不会引入别的颜色。\n"
                "  --wall-min-r N       覆盖「墙」的 R 下限（默认 110）\n"
                "  --wall-min-g N       覆盖「墙」的 G 下限（默认 60）\n"
                "  --bright-min N       覆盖「亮（角色）」的 min(RGB) 下限（默认 195）\n"
                "  --bright-spread N    覆盖「亮」的 max-min 上限（默认 45）\n"
                "  --green-gm N         覆盖「绿」的 G-max(R,B) 下限（默认 10）\n"
                "  --green-min N        覆盖「绿」的 G 下限（默认 90）\n"
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


/// 逐像素分类的**有效阈值**：C 参考模型/Python 真值的默认值 + 命令行覆盖。
///
/// 为什么集中在一处：写寄存器（`configure`）和 `--verify-both` 的配置对账必须
/// 用**同一个**来源，否则标定过的阈值会被那条对账判成"不一致"（它原本只比默认值）。
VisionParams effective_vision_params(const Args& a) {
    VisionParams p;
    if (a.ov_wall_rb >= 0)       p.wall_rb = a.ov_wall_rb;
    if (a.ov_wall_min_r >= 0)    p.wall_min_r = a.ov_wall_min_r;
    if (a.ov_wall_min_g >= 0)    p.wall_min_g = a.ov_wall_min_g;
    if (a.ov_bright_min >= 0)    p.bright_min = a.ov_bright_min;
    if (a.ov_bright_spread >= 0) p.bright_spread = a.ov_bright_spread;
    if (a.ov_green_gm >= 0)      p.green_gm = a.ov_green_gm;
    if (a.ov_green_min >= 0)     p.green_min = a.ov_green_min;
    return p;
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
    //: 线协议**不带速度**，所以 PS 侧自己推（见 `core/cpu/cpp/motion.hpp`）。
    //: 角色与障碍各一个跟踪器：关联是"最近邻 + 门限"，两类目标的运动模型
    //: 完全无关，混在一起只会互相抢检测。
    motion::Tracker player_motion;
    motion::Tracker obstacle_motion;
    uint64_t prev_stamp_ns = 0;
    bool have_prev_stamp = false;
    int n_decisions = 0;
    int n_failed = 0;
    //: 有几帧的"角色"落在四角标定标记上（被当没检出）—— 见 decide() 里的说明。
    int n_player_on_fiducial = 0;
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

    //: 用**外部给的**像素跑参考模型（与 :meth:`next` 的唯一区别是"图从哪来"）。
    //:
    //: `--verify-both` 与实时（stdin）那条路都用它 —— 两条路**必须吃同一份像素**，
    //: 否则比的就不是"两条像素通路"，而是"两张图"，那样的对账没有意义。
    bool next_from(const std::vector<uint8_t>& rgb, int w, int h,
                   uint32_t seq, uint64_t stamp_ns,
                   std::vector<uint8_t>* out, std::string* why) {
        if (w <= 0 || h <= 0) { *why = "图尺寸非法"; return false; }
        if (static_cast<size_t>(w) * static_cast<size_t>(h) > gray_.size()) {
            *why = "图太大（缓冲只准备了 1M 像素）";
            return false;
        }
        return run(rgb.data(), w, h, seq, stamp_ns, out, why);
    }

    //: 取第 i 帧的几何帧字节（从文件/目录读图）。
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
        int w = 0, h = 0;
        if (!vision_board::read_ppm_p6(path, &w, &h, &rgb_, why)) return false;
        w_ = w; h_ = h;
        return next_from(rgb_, w, h, seq, stamp_ns, out, why);
    }

private:
    bool run(const uint8_t* rgb, int w, int h, uint32_t seq, uint64_t stamp_ns,
             std::vector<uint8_t>* out, std::string* why) {
        VrRefResult res;
        std::memset(&res, 0, sizeof(res));
        const int rc = vr_ref_process_ex(rgb, w, h, 1,
                                         gray_.data(), wall_.data(),
                                         bright_.data(), rowc_.data(),
                                         &res, frame_.data(), frame_.size(),
                                         &params_, seq, stamp_ns);
        if (rc != 0) { *why = "参考模型返回 " + std::to_string(rc); return false; }
        out->assign(frame_.begin(), frame_.begin() + res.frame_bytes);
        return true;
    }

public:
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
    //: 目标有三种写法：
    //:   `/dev/ttyPS0`        板子控制台串口（RP2040 的 GP1 就听这根线）
    //:   `udp://192.168.137.1:9999`  **走网口**发给 PC —— 不需要接任何线，
    //:                        也不和控制台日志抢那一条串口
    //:   `stdout`（或 `-`）   把 `@MOVE n` 打到标准输出。配 SSH 管道用：
    //:                        `ssh 板子 '... --keyboard-dev stdout' | pc_key_driver.py --stdin`
    //:                        —— 复用**已经在用**的 SSH 通道，既不用接引脚，
    //:                        也不用动 Windows 防火墙/usbipd
    explicit ActionEmitter(const std::string& target) {
        if (target.empty()) return;              // 显式关掉
        if (target == "stdout" || target == "-") { stdout_ = true; return; }
        if (target.rfind("udp://", 0) == 0) { open_udp(target.substr(6)); return; }
        fd_ = ::open(target.c_str(), O_WRONLY | O_NONBLOCK | O_NOCTTY);
        if (fd_ < 0) {
            std::fprintf(stderr, "⚠ 打不开键盘串口 %s：%s（这一轮不发动作）\n",
                         target.c_str(), std::strerror(errno));
        }
    }
    ~ActionEmitter() {
        if (fd_ >= 0) ::close(fd_);
        if (udp_ >= 0) ::close(udp_);
    }
    bool ok() const { return fd_ >= 0 || udp_ >= 0 || stdout_; }
    //: 给启动时打一句用（说清楚发到哪儿）。
    const char* kind() const {
        if (stdout_)   return "标准输出";
        if (udp_ >= 0) return "UDP";
        if (fd_ >= 0)  return "串口";
        return "未打开";
    }

    //: 返回写出去的字节数；<0 表示这一帧丢了（已打点）。
    int emit(int action, bool noisy) {
        if (fd_ < 0 && udp_ < 0 && !stdout_) return -1;
        char buf[16];
        const int n = vision_board::format_action_line(action, buf, sizeof(buf));
        if (n <= 0) return -1;
        if (stdout_) {
            // ⚠ 必须 fflush：stdout 接管道时是**块缓冲**，不刷的话整串动作会卡在
            // 用户态缓冲区里，直到进程退出才吐出来 —— 那就完全不是实时控制了。
            const size_t w = std::fwrite(buf, 1, static_cast<size_t>(n), stdout);
            std::fflush(stdout);
            if (w != static_cast<size_t>(n)) { ++dropped_; return -1; }
            ++sent_;
            return n;
        }
        if (udp_ >= 0) {
            // UDP：**丢了也无所谓**（键盘桥有看门狗），所以不做重传/确认，
            // 也就绝不会把决策循环堵在这里。
            const ssize_t w = ::sendto(udp_, buf, static_cast<size_t>(n), 0,
                                       reinterpret_cast<const sockaddr*>(&dst_),
                                       sizeof(dst_));
            if (w != n) { ++dropped_; return -1; }
            ++sent_;
            return n;
        }
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
    void open_udp(const std::string& hostport) {
        const size_t c = hostport.rfind(':');
        if (c == std::string::npos) {
            std::fprintf(stderr, "⚠ --keyboard-dev 的 udp:// 要写成 udp://主机:端口\n");
            return;
        }
        const std::string host = hostport.substr(0, c);
        const std::string port = hostport.substr(c + 1);
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* res = nullptr;
        if (::getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
            std::fprintf(stderr, "⚠ 解析不了 udp://%s（这一轮不发动作）\n", hostport.c_str());
            return;
        }
        udp_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (udp_ < 0) { ::freeaddrinfo(res); return; }
        std::memcpy(&dst_, res->ai_addr, res->ai_addrlen);
        ::freeaddrinfo(res);
    }

    bool stdout_ = false;
    int fd_ = -1;
    int udp_ = -1;
    sockaddr_in dst_{};
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

    // ---- 速度：线协议不带，PS 侧用相邻帧 stamp 差推 --------------------------
    // `vision_pkg.vh` 里写着"速度由 PS 从相邻帧 stamp 之差推"；决策要它来算
    // "墙是不是在逼近"（`v_n = vn_wall - player_v·n̂`）。没有它，真机上所有障碍
    // 都被当成静止的 —— 而训练环境里它们是有速度的，两边判据就不是一回事了。
    const double t_now = static_cast<double>(f.header.stamp_ns) * 1e-9;
    double pvx = 0.0, pvy = 0.0;
    //: 四角标定标记是**已知位置**的亮块（`standard.fiducial`：边长 48、离边 8，
    //: 留白边还有 8）—— 实测它们的填充度会偶尔越过 `player_min_fill_pct=45`，
    //: 于是几何帧里"角色"会突然跳到某个角落。板子如果照着那个位置发指令，
    //: 真角色就被推着**往边上钻**，然后卡在边上（相机能看见，但检出漂移）。
    //: 所以：落在标记 footprint 里的"角色"记录一律**当没检出**（这一帧 stay），
    //: 宁可不动，也不拿一个已知是标记的位置去发指令。
    fiducial_zone::Zone zone;
    zone.field_w = args.field_w;
    zone.field_h = args.field_h;
    zone.side = args.fiducial_side;
    zone.margin = args.fiducial_margin;
    zone.slack = args.fiducial_slack;
    const bool player_on_fiducial = f.player.present && zone.contains(f.player.x, f.player.y);
    if (player_on_fiducial) ++st.n_player_on_fiducial;
    const bool player_ok = f.player.present && !player_on_fiducial;

    if (player_ok) {
        const motion::Point pp{f.player.x, f.player.y};
        if (!motion::track_one(st.player_motion, pp, t_now, &pvx, &pvy)) {
            // 时间戳坏了（倒流/NaN）：重起时基，这一帧速度为 0。
            // Python 侧此时会抛异常；板子上是实时环，不能崩（见 motion.hpp 顶部）。
            st.player_motion.reset();
            pvx = pvy = 0.0;
        }
    } else {
        pvx = pvy = 0.0;                 // 没角色（或那是标定标记）：不留旧速度
        st.player_motion.reset();
    }

    // ---- 角色 ----
    const double player[7] = {
        player_ok ? f.player.x : args.field_w * 0.5,
        player_ok ? f.player.y : 0.0,
        pvx, pvy,
        player_ok ? f.player.radius : 10.0,
        args.player_speed,
        1.0,
    };

    // ---- 障碍：只取矩形（决策的 group_walls 只认矩形）----
    //
    // ⚠ **丢掉退化矩形**：PL 的游程检测会把屏幕上的**一条细线**（场地外框、
    //   网格线、窗口边、反光边缘）也输出成"矩形" —— 实测相机画面里这些假矩形
    //   宽 3~6 px、half_h ≤ 0，而且**每帧出现在同一个位置**。它们进决策的后果
    //   很具体：几何帧里一会儿 1 个、一会儿 6 个矩形，`gap_avoid` 找错缺口、
    //   跟踪器给墙推出一帧 188 一帧 5 的假速度，平台于是"左右乱跑"。
    //   真正的墙：half_w ≈ 44~96、half_h ≈ 3~16（实测一帧里 3~6 个矩形，其中
    //   一半是 12~16 px 的假货）。这里按**绝对像素尺寸**过滤：half_w < 20 的
    //   一律不要 —— 实测真墙最窄 44，假货最大 16，中间是空的，好切。
    static const double kMinRectHalfW = 20.0;   // px（pipeline 坐标）
    static const double kMinRectHalfH = 1.0;
    std::vector<gap_avoid::Rect> rects;
    rects.reserve(f.obstacles.size());
    int n_degenerate = 0;
    for (const GeometryObstacle& o : f.obstacles) {
        if (!o.is_rect()) continue;
        if (!(o.half_w >= kMinRectHalfW) || !(o.half_h >= kMinRectHalfH)) {
            ++n_degenerate;      // NaN 也走这里（比较为假）
            continue;
        }
        gap_avoid::Rect r;
        // ⚠ `o.id` 是**当帧序号**（RTL: `seg_id <= n_obs + 1`），跨帧不稳定，
        //   所以这里只当标签用；跨帧配速度靠跟踪器的位置关联（见 motion.hpp）。
        r.ent_id = o.id;
        r.x = o.x;
        r.y = o.y;
        r.vx = 0.0;              // 下面由跟踪器填
        r.vy = 0.0;
        r.half_w = o.half_w;
        r.half_h = o.half_h;
        r.rotation = o.rotation;
        r.angular_velocity = 0.0;   // 决策不看自转（gap_avoid 里没用这个字段）
        rects.push_back(r);
    }

    if (!rects.empty()) {
        std::vector<motion::Point> dets;
        dets.reserve(rects.size());
        for (const gap_avoid::Rect& r : rects) dets.push_back(motion::Point{r.x, r.y});
        std::vector<double> ovx(rects.size(), 0.0), ovy(rects.size(), 0.0);
        if (st.obstacle_motion.update(dets.data(), static_cast<int>(dets.size()), t_now)) {
            motion::attach_velocities(st.obstacle_motion.tracks(),
                                      st.obstacle_motion.assignment(),
                                      static_cast<int>(dets.size()),
                                      ovx.data(), ovy.data());
        } else {
            st.obstacle_motion.reset();     // 时间戳坏了：重起时基，这一帧速度为 0
        }
        for (size_t i = 0; i < rects.size(); ++i) {
            rects[i].vx = ovx[i];
            rects[i].vy = ovy[i];
        }
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
    gap_avoid::Vec2 cmd = gap_avoid::to_command(d);
    if (!player_ok) {
        // 角色没检出来（或检出来的位置正好是四角标定标记）时，上面用的位置是
        // **兜底值**（场地中线）—— 拿它发指令等于闭着眼睛开：实测漏检角色时指令会
        // 恒定不变、平台一直朝一个方向走且永不收敛；而"把角落标记当角色"更糟，
        // 它会照着角落把真角色往边上推。所以这一帧只发 stay，并把原因打出来。
        cmd.x = 0.0;
        cmd.y = 0.0;
        if (args.verbose) {
            std::fprintf(stderr, player_on_fiducial
                ? "  ⚠ 本帧的「角色」落在四角标定标记上 → 只发 stay（不拿标记当角色）\n"
                : "  ⚠ 本帧没检出角色 → 只发 stay（不拿兜底位置发指令）\n");
        }
    }
    ++st.n_decisions;
    st.tier_count[tier_bucket(d.tier)] += 1;

    std::printf("        决策: tier=%-7s 指令=(%+.3f, %+.3f)  "
                "t_arr=%.2fs 间隙=%+.1f 可用半宽=%+.1f 矩形=%d",
                d.tier, cmd.x, cmd.y,
                std::isfinite(d.t_arr) ? d.t_arr : -1.0,
                std::isfinite(d.gap) ? d.gap : -1.0,
                std::isfinite(d.usable_half) ? d.usable_half : -1.0,
                d.n_rects);
    if (n_degenerate > 0) std::printf("（丢了 %d 个退化矩形）", n_degenerate);
    // 推出来的速度也打出来：线协议不带它，"到底推没推、推得对不对"只能看这一行。
    // 墙速取**最大**的那个（有多面墙时逐个打会把日志淹掉）。
    double wall_v = 0.0;
    for (const gap_avoid::Rect& r : rects) {
        const double sp = std::hypot(r.vx, r.vy);
        if (sp > wall_v) wall_v = sp;
    }
    std::printf("  速度: 角色=(%+.1f,%+.1f)|v|=%.1f  墙max|v|=%.1f",
                pvx, pvy, std::hypot(pvx, pvy), wall_v);
    if (args.track_log != nullptr) {
        std::fprintf(args.track_log, "F %u %llu %zu %zu %d\n",
                     f.header.seq, static_cast<unsigned long long>(f.header.stamp_ns),
                     rects.size(), st.obstacle_motion.tracks().size(),
                     st.obstacle_motion.n_relocalized());
        for (const gap_avoid::Rect& r : rects) {
            std::fprintf(args.track_log, "R %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                         r.ent_id, r.x, r.y, r.vx, r.vy, r.half_w, r.half_h, r.rotation);
        }
        const std::vector<motion::Track>& ts = st.obstacle_motion.tracks();
        const std::vector<int>& asg = st.obstacle_motion.assignment();
        for (size_t i = 0; i < ts.size(); ++i) {
            std::fprintf(args.track_log, "T %d %.6f %.6f %.6f %.6f %d %d %d %d\n",
                         ts[i].id, ts[i].x, ts[i].y, ts[i].vx, ts[i].vy,
                         ts[i].hits, ts[i].misses, ts[i].age,
                         i < asg.size() ? asg[i] : -1);
        }
        std::fprintf(args.track_log, "P %.6f %.6f %.6f %.6f\n",
                     player_ok ? f.player.x : 0.0,
                     player_ok ? f.player.y : 0.0, pvx, pvy);
        std::fflush(args.track_log);
    }
    if (rects.empty()) std::printf("  [无矩形障碍]");
    if (!player_ok) {
        std::printf(player_on_fiducial
            ? "  [角色落在四角标定标记上，当没检出]"
            : "  [无角色，按场宽中线/半径 10 兜底]");
    }
    if (dt_is_default) std::printf("  [dt=默认，无上一帧 stamp]");
    std::printf("\n");
    return cmd;
}

}   // namespace

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);      // 非 const：下面要挂上轨迹日志的文件

    std::printf("=== vision board bring-up ===\n");
    std::printf("设备: %s\n", args.device_url.c_str());

    // `--track-log` 的文件在这里开（解析期只存路径，不碰 IO）
    if (!args.track_log_path.empty()) {
        args.track_log = std::fopen(args.track_log_path.c_str(), "w");
        if (args.track_log == nullptr) {
            std::fprintf(stderr, "⚠ 打不开轨迹日志 %s：%s\n",
                         args.track_log_path.c_str(), std::strerror(errno));
        } else {
            std::printf("轨迹日志: %s\n", args.track_log_path.c_str());
        }
    }
    const bool have_ppm = !args.ppm_file.empty() || !args.ppm_dir.empty();
    const bool have_src = have_ppm || args.stdin_rgb;
    // 走 PL 时**不**用 PS 参考模型：像素推给 PL，几何帧从 PL 读回来
    const bool use_ref = have_src && !args.via_pl && !args.verify_both;

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
    // 每帧的像素来源（PPM 文件/目录，或 stdin 的原始 RGB）
    //
    // stdin 那条用 :class:`StdinPump`（**读线程**）：实测三段是串行的
    // （等采集 33 + 推PL 17 + 等流水线 21 = 72ms ≈ 14 帧/秒），而读线程能在主线程
    // 推像素/等流水线的时候把下一帧读好，重叠之后接近 max(33, 38) ≈ 40ms。
    std::unique_ptr<RgbSource> src;
    std::unique_ptr<StdinPump> pump;
    if (args.stdin_rgb) {
        pump.reset(new StdinPump(kPlWidth, kPlHeight, args.skip_frames, args.frames));
        std::printf("帧来源: **stdin 原始 RGB**（%dx%d，丢掉开头 %d 帧热身，读线程重叠）\n"
                    "  管道缓冲 = %d 字节（要 ≥1 帧 691200 才谈得上重叠；"
                    "不够就在启动脚本里调 /proc/sys/fs/pipe-max-size）\n",
                    kPlWidth, kPlHeight, args.skip_frames, pump->pipe_bytes());
    } else if (have_ppm) {
        src.reset(new PpmSource(args.ppm_file, args.ppm_dir));
    }
    if (use_ref || args.verify_both) {
        ref.reset(new RefFrameSource(args));
    }
    if (!use_ref) {
        drv.reset(new VisionDriver(*io));
        if (args.verify_both) {
            std::printf("PL: **两条都跑**（PL 推像素 vs PS 黄金模型，逐字节比）\n");
            drv->set_pixel_source(true);
            drv->flush_pixels();
        } else if (have_src) {
            std::printf("PL: 像素由 PS 推入 FIFO（来自 %s）\n",
                        pump ? "stdin 原始 RGB（读线程）" : src->kind());
            drv->set_pixel_source(true);
            drv->flush_pixels();
        } else {
            std::printf("PL: 外部 din_*（板上是 tpg_source）\n");
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
        const VisionParams params = effective_vision_params(args);
        drv->configure(params);
        if (args.vision_calibrated()) {
            std::printf("配置：已写入 %d 个寄存器（**标定过的阈值**：wall_rb=%d "
                        "wall_min_r=%d wall_min_g=%d bright_min=%d bright_spread=%d）\n",
                        (int)kCfgCount, params.wall_rb, params.wall_min_r,
                        params.wall_min_g, params.bright_min, params.bright_spread);
        } else {
            std::printf("配置：已写入 %d 个寄存器（阈值用默认值）\n", (int)kCfgCount);
        }
    }

    // ---- 键盘桥：动作行发到板子控制台 TX（RP2040 就听这根线）--------------
    long cmp_total = 0, cmp_ok = 0, cmp_bad = 0;   // --verify-both 的对账计数

    ActionEmitter kb(args.keyboard_dev);
    if (!args.keyboard_dev.empty()) {
        std::printf("键盘桥: %s → %s（%s）\n", args.keyboard_dev.c_str(), kb.kind(),
                    kb.ok() ? "已打开，发 \"@MOVE <n>\"" : "**打不开**，本轮不发动作");
    } else {
        std::printf("键盘桥: 已关闭（--no-keyboard）\n");
    }

    // ---- 逐帧 ------------------------------------------------------------
    DecisionState st;
    const auto t0 = std::chrono::steady_clock::now();
    int ok = 0, bad = 0;
    uint64_t stamp_ns = 0;
    double sum_read = 0, sum_push = 0, sum_wait = 0, sum_total = 0;
    long n_timed = 0;
    // `--frames 0` = 不限帧数，一直跑到输入结束（实时采集用）
    const long want_frames = args.frames > 0 ? args.frames : 1000000000L;
    for (int i = 0; i < want_frames; ++i) {
        // ---- 取这一帧的像素（统一入口：PPM 或 stdin）------------------
        // 两条下游（PL 推像素 / PS 参考模型）**吃同一份 rgb** ——
        // 否则 --verify-both 比的就不是"两条像素通路"而是"两张图"。
        const auto t_frame0 = std::chrono::steady_clock::now();
        std::vector<uint8_t> rgb;
        int pw = 0, ph = 0;
        if (pump) {
            std::string swhy;
            if (!pump->get(&rgb, &swhy)) {
                if (i > 0) break;
                std::fprintf(stderr, "第一帧就取不到像素：%s\n", swhy.c_str());
                ++bad; break;
            }
            pw = kPlWidth; ph = kPlHeight;
            // 用读线程打下的**真实采集时刻**当 stamp：固定步长会让速度系统性偏小
            // （相机实测 40~48 fps，按 30fps 打戳就低 28%~50%）。
            const uint64_t cam_ts = pump->last_stamp_ns();
            if (cam_ts != 0) stamp_ns = cam_ts;
        } else if (src) {
            std::string swhy;
            if (!src->next(&rgb, &pw, &ph, &swhy)) {
                // 没有更多帧是**正常结束**（ppm-dir 取完了 / 管道关了）
                if (i > 0) break;
                std::fprintf(stderr, "第一帧就取不到像素：%s\n", swhy.c_str());
                ++bad; break;
            }
            if (pw != kPlWidth || ph != kPlHeight) {
                std::fprintf(stderr,
                    "第 %d 帧：图是 %dx%d，而 PL 的流水线固定为 %dx%d —— 尺寸不符\n"
                    "  （PL 的 WIDTH/HEIGHT 是编译期常量；要换尺寸得改 RTL 重建位流）\n",
                    i, pw, ph, kPlWidth, kPlHeight);
                ++bad; break;
            }
        }

        const auto t_read1 = std::chrono::steady_clock::now();
        std::vector<uint8_t> bytes;
        std::vector<uint8_t> bytes_ref;      // --verify-both 时用来对账
        if (args.verify_both) {
            // 先跑 PS 黄金模型（纯计算，不动硬件），再让 PL 跑**同一张图**。
            // 两张几何帧必须逐字节相同 —— 不同就说明 PL 那条路有问题，
            // 而且差异位置能直接指出是哪个字段。
            std::string rwhy;
            if (!ref->next_from(rgb, pw, ph, static_cast<uint32_t>(i), stamp_ns,
                                &bytes_ref, &rwhy)) {
                std::fprintf(stderr, "第 %d 帧：参考模型失败：%s\n", i, rwhy.c_str());
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
            if (drv->last_push_overflowed()) {
                // 像素没喂全 → 几何帧"自洽但错"（CRC 会过）→ **丢掉这一帧**，
                // 不能拿它去决策：对账也就没有意义了。
                std::fprintf(stderr,
                             "第 %d 帧：像素 FIFO 溢出（缓冲 %u 字节），几何不可信，"
                             "本帧丢弃（不对账、不决策）\n",
                             i, drv->last_push_buffered());
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
                if (args.verbose)
                    std::printf("  ✓ 帧 %d：PL 与 PS 黄金模型逐字节相同（%zu 字节）\n",
                                i, bytes.size());
            }
        } else if (drv && (src || pump)) {
            try {
                bytes = drv->run_frame_push(rgb.data(), rgb.size(),
                                            static_cast<uint32_t>(i), stamp_ns,
                                            args.timeout_ms);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "第 %d 帧失败（推像素）：%s\n", i, e.what());
                ++bad; continue;
            }
            if (drv->last_push_overflowed()) {
                // 见上面那条：像素没喂全时几何"自洽但错"，必须当坏帧丢掉，
                // 否则会拿一张残缺图去决策并发动作 —— 静默错的一类。
                std::fprintf(stderr,
                             "第 %d 帧：像素 FIFO 溢出（缓冲 %u 字节），几何不可信，"
                             "本帧丢弃（不决策、不发动作）\n",
                             i, drv->last_push_buffered());
                ++bad; continue;
            }
        } else if (ref) {
            std::string rwhy;
            if (!ref->next_from(rgb, pw, ph, static_cast<uint32_t>(i), stamp_ns,
                                &bytes, &rwhy)) {
                std::fprintf(stderr, "第 %d 帧失败（参考模型）：%s\n", i, rwhy.c_str());
                ++bad; break;
            }
        } else {
            try {
                bytes = drv->run_frame(static_cast<uint32_t>(i), stamp_ns,
                                       args.timeout_ms);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "第 %d 帧失败：%s\n", i, e.what());
                ++bad; continue;
            }
        }
        if (pump) pump->recycle(std::move(rgb));   // 还回空闲池，省一次分配

        if (args.timing) {
            const auto t_frame1 = std::chrono::steady_clock::now();
            const double total = std::chrono::duration<double, std::milli>(t_frame1 - t_frame0).count();
            const double rd    = std::chrono::duration<double, std::milli>(t_read1 - t_frame0).count();
            const auto& tm = drv ? drv->last_timing() : VisionDriver::FrameTiming{};
            sum_read += rd; sum_push += tm.push_ms; sum_wait += tm.wait_ms;
            sum_total += total; ++n_timed;
            std::printf("   [timing] 读像素 %6.1f  推PL %6.1f  等流水线 %6.1f  合计 %6.1f ms\n",
                        rd, tm.push_ms, tm.wait_ms, total);
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
        // 离线序列：按 --ppm-fps 合成步长（实时路径已在上面用真实采集时刻覆盖）
        stamp_ns += static_cast<uint64_t>(1e9 / args.ppm_fps + 0.5);
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    std::printf("----\n成功 %d 帧，失败 %d 帧，用时 %lld ms", ok, bad,
                static_cast<long long>(ms));
    if (ms > 0) {
        std::printf("（%.1f 帧/秒）", 1000.0 * ok / static_cast<double>(ms));
    }
    std::printf("\n");
    if (args.timing && n_timed > 0) {
        const double n = static_cast<double>(n_timed);
        const double mean = sum_total / n;
        std::printf("---- 分段平均（%ld 帧）：读像素 %.1f  推PL %.1f  等流水线 %.1f  合计 %.1f ms"
                    "   → %.1f 帧/秒 ----\n",
                    n_timed, sum_read / n, sum_push / n, sum_wait / n, mean,
                    mean > 0.0 ? 1000.0 / mean : 0.0);
    }
    if (args.verify_both) {
        // ---- 配置对账：PS 写进去的 21 个寄存器 vs 参考模型的默认参数 ----
        //
        // 为什么单列这一步：**协同仿真测不到它** —— `tb_axi` 两边都用 C 参考模型的
        // 默认值，所以"PS 侧那份结构体的默认值 / 下标映射"从来没被验过。而板上
        // 写进去的是 C++ 的 `VisionParams`。两者只要有一项不同，PL 与模型的
        // 判据就会不同 → 几何帧"个数对、坐标不对"，且**完全不报错**。
        {
            // ⚠ 比的是**有效阈值**（默认值或命令行标定过的），不是写死的默认值：
            //   这条对账要抓的是"结构体 → 寄存器下标映射"的错位，而不是标定本身。
            const VisionParams eff = effective_vision_params(args);
            const std::vector<int32_t> regs = eff.to_regs();
            const int32_t want[21] = {
                regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6],
                regs[7], regs[8], regs[9], regs[10], regs[11], regs[12], regs[13],
                regs[14], regs[15], regs[16], regs[17], regs[18], regs[19], regs[20],
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
        if (st.n_player_on_fiducial > 0) {
            std::printf("其中 %d 帧的「角色」落在四角标定标记上（已当没检出，只发 stay）\n",
                        st.n_player_on_fiducial);
        }
    }

    delete io;      // icraft/devmem 后端是 new 出来的
    return bad == 0 ? 0 : 1;
}
