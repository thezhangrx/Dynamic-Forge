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
#include "auto_params.hpp"
#include "band_gap.hpp"
#include "denoise.hpp"
#include "geometry.hpp"
#include "frame_geom.hpp"
#include "virt_truth.hpp"
#include "marker_refine.hpp"
#include "player.hpp"
#include "robust.hpp"
#include "wall_gap.hpp"
#include "rectify.hpp"
#include "fiducial_zone.hpp"
#include "segment_merge.hpp"
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

#include <array>
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
    //: 矩形层"碎片合并"（见 `segment_merge.hpp`）：一堵墙被切成几段时并回一段。
    //: 默认开；`--merge-gap-px 0` 关掉。实测真缺口约 58 px，所以默认 6 px 有 10 倍余量。
    double merge_gap_px = 6.0;
    double merge_dy_px = 4.0;
    //: 决策的**边界余量**（场地单位，见 `gap_avoid.hpp` 的 `limit_edges`）：
    //: 距边界不足这个距离时，指令里朝外的分量会被线性削到 0（切向保留）。
    //: 取值依据：视觉边缘裁切需 >= 1.5 x 角色半径 ≈ 15 + 延迟位移 46ms x 200 ≈ 9，
    //: 合计取 25（场宽 640 的 ~4%）。`--edge-margin 0` 关掉。
    double edge_margin = 25.0;
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
    //: `--auto-thresholds`（**默认关**，见下面的实测）/ `--no-auto-thresholds`。
    //:
    //: 逐像素分类的 7 个阈值**每帧由本帧统计推出**（见 auto_params.hpp）：
    //: 真机上自动曝光把橙墙拍成淡粉白，(r-b) 只剩 10~30，固定 `(r-b)>40 && r>110`
    //: 于是帧间翻转（实测 43% 的帧一个矩形都没有）。
    //:
    //: ⚠ 第一版自动阈值（全局 z 分数 / Otsu）在真机位上**更差**：这一轮实测
    //:   `--auto-thresholds` 250 帧零矩形 100%，关掉只有 1%（ALIGN 218）。
    //: 原因写在 auto_params.hpp 的注释里：整幅图被屏幕辉光抬成一个宽的中灰，
    //: 全局直方图上"亮 83%"、谷值落在 88~142，背景的 d 又比墙还暖 ——
    //: **全局统计量在这台相机上不成立**，必须走局部/结构判据（行带匹配滤波）。
    //: 所以默认关，留作对照实验的开关。
    bool auto_thresholds = false;
    //: `--structural-gap`：用 `band_gap.hpp`（行带匹配滤波 + 缺口滞回）**从像素直接
    //: 重建**"带缺口的墙"，并按场地边框量出的比例尺换算到场地单位，顶掉 RTL 的碎矩形。
    //:
    //: 为什么要这条：RTL 的矩形来自逐像素分类 + 游程/带检测。相机把墙拍成淡粉白时，
    //: 它给出来的是**碎块**（实测一帧 3~8 个，真墙只有 2 条），`gap_avoid` 因此
    //: 重建不出缺口 —— 真机 250 帧里 NO_GAP 181 次、指令几乎全是"不动"。
    //: RTL 一个门都不改（`--verify-both` 照旧逐字节对账），这条全在 PS 侧。
    bool structural_gap = false;
    //: 画面纵向压缩比（`cam_live.sh` 用 scale=640:360 把 4:3 压成 16:9 → 0.75；
    //: 用 CAM_CROP=1 时是 1.0）。纵向换算**靠它推**，不靠被截断的下边框。
    double image_squash = 0.75;
    //: 推像素前的方框滤波半径（0=关）。真机位相机彩噪很强（噪声度量 17~19，
    //: 干净基准 2.0），去噪前标记只能找到 3 个，r=1 后 4 个全对（实测）。
    int denoise_radius = 0;
    //: 取"排队里最旧的那一帧"（默认关：默认只取最新帧，见 StdinPump::set_drop_stale）
    bool keep_stale = false;
    //: 是否还每帧调用亮度台阶法（`field_extent`）。默认关：P1 之后它只为参考，
    //: 而实测每帧要 9.6 ms —— 在 13 fps 的闭环里这是 13% 的帧时间。
    bool extent_fallback = false;
    //: `--no-pl`：不推像素到 PL、不等几何帧。用作对照实验：
    //: PL 那段（推 17.3 + 等 6.4 = 23.7 ms）目前唯一还在用的是它给的角色
    //: （实测 67/1238 帧），而 PS 侧角色已经覆盖 78%+。
    bool no_pl = false;
    //: `--host-epoch <秒>`：由**主机**在启动本进程前写入的墙钟时刻。
    //: 有了它，每帧就能算出"这一帧发生在主机时间的哪一刻"，从而和仿真时间**直接**
    //: 对齐 —— 不用猜偏移、也不依赖板子时钟（实测它与主机差 1.5 s 且会漂）。
    double host_epoch = 0.0;
    //: `--no-player-pred`：关掉"局部精修 + 短时预测"，只用全局检测。
    //: 用作 A/B：局部精修以**上一帧**为种子，一旦种到墙上就会逐帧漂移。
    bool no_player_pred = false;
    //: `--local-refine`：全局检测失败时用"上一帧位置"做局部精修。
    //: **默认关**：离线动态语料（同一批帧）显示全局检测已经能到 96% 检出、误差中位
    //: 10 px；而局部精修在平坦场地上会沿亮带漂移，把它打开反而让命中率从
    //: 离线 62.7% 掉到在线 19%（`ps_new` 里混着一半是局部精修的结果）。
    bool local_refine = false;
    //: `--legacy-gap`：退回旧的 `band_gap::find_at` 门检测（对照用）。
    //: 默认用 `wall_gap::find_walls`（每道墙一扇门、静态 1 px、动态目视验证过）。
    bool legacy_gap = false;
    std::string diag_log;
    //: 契约里的角色半径（场地单位）。用作**尺寸先验**：结构判据抓到的角色，
    //: 其外接框必须在"契约半径 × 比例尺"附近；抓到大块方形亮物（墙角/角标）
    //: 必须丢掉，否则会照着错的位置发指令（实测出现过 r=32.4，契约是 10）。
    double player_radius = 10.0;

    std::string keyboard_dev = "/dev/ttyPS0";
    double action_deadzone = kDefaultDeadzone;
    bool echo_action = false;       // 把动作号也打到 stdout（人看）
    //: `--cal <秒>`：**开场标定窗口**。这段时间里决策层被旁路，改发一段
    //: 已知的动作序列（停 → 左右上下 → 四个对角，各 2 s，段间停 0.4 s），
    //: 同时仿真用 `--obstacle-delay` 保证场上没有障碍。用途：
    //:
    //:   1. 停住那 1 s 给出"角色静止位置"的多帧样本（取稳健均值当初始位置）；
    //:   2. 每个 2 s 的匀速段给出 `位移/时间` = **平台真实速度**
    //:      （以前用的 158.7 是拍的，实际是契约里的 200 还是别的，必须量）；
    //:   3. 段起点到"位移真正开始"的滞后 = **指令→动作延迟**
    //:      （以前只量到过 0.457 s 这个总数，没把它拆成可标定的量）。
    //:
    //: 旁路只在标定窗口内生效：窗口一过立刻恢复决策层，行为和平时完全一样。
    double cal_seconds = 0.0;
    //: `--predict-s S`（P1）：把世界**推进到"这条指令真正生效"的那一刻**再决策。
    //: 实测端到端 0.56 板秒（相机 60 ms + 处理 65 ms + 传输 + 仿真 0.75× 实时），
    //: 而平台按 ~156 场地单位/板秒 走 —— 0.5 s 就是 **78 单位**，而缺口可用半宽
    //: 只有 25 单位。拿"看到的位置"直接决策，等于朝 0.5 s 前该去的地方走。
    //: 0 = 关（默认，保持老行为，便于 A/B）。
    double predict_s = 0.0;
    //: 预测用的平台速度（场地单位/**板秒**）。默认取 `boot_cal` 的实测值
    //: （156.6 ≈ 契约 200 单位/仿真秒 × rtf 0.75 ÷ 板子速率 1.09）。
    double predict_speed = 156.6;
    //: 几何来源：`frame` = 只认场地白框（推荐，不再依赖四角标记）；
    //: `markers` = 老路径；`auto` = 白框优先、失败退标记。
    std::string geom = "auto";
    //: `--virt`（默认开）：墙/角色的位置由**虚拟真值**维持（视觉只做初始化 +
    //: 发现新墙 + 慢校正）。`--no-virt` 回到"每帧都用视觉"，用于 A/B。
    bool virt = true;
    //: 角色慢校正的残差门（场地单位）与比例；漂移超过门 → 该帧视觉**不喂**。
    double virt_gate = 60.0;
    double virt_alpha = 0.25;
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
        else if (k == "--merge-gap-px")    a.merge_gap_px = std::atof(next());
        else if (k == "--merge-dy-px")     a.merge_dy_px = std::atof(next());
        else if (k == "--edge-margin")     a.edge_margin = std::atof(next());
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
        else if (k == "--structural-gap") a.structural_gap = true;
        else if (k == "--image-squash") a.image_squash = std::atof(next());
        else if (k == "--denoise") a.denoise_radius = std::atoi(next());
        else if (k == "--keep-stale") a.keep_stale = true;
        else if (k == "--extent-fallback") a.extent_fallback = true;
        else if (k == "--no-pl") a.no_pl = true;
        else if (k == "--host-epoch") a.host_epoch = std::atof(next());
        else if (k == "--cal") a.cal_seconds = std::atof(next());
        else if (k == "--predict-s") a.predict_s = std::atof(next());
        else if (k == "--predict-speed") a.predict_speed = std::atof(next());
        else if (k == "--geom") a.geom = next();
        else if (k == "--virt") a.virt = true;
        else if (k == "--no-virt") a.virt = false;
        else if (k == "--virt-gate") a.virt_gate = std::atof(next());
        else if (k == "--virt-alpha") a.virt_alpha = std::atof(next());
        else if (k == "--no-player-pred") a.no_player_pred = true;
        else if (k == "--local-refine") a.local_refine = true;
        else if (k == "--legacy-gap") a.legacy_gap = true;
        //: `--diag-log <path>`：每帧一条 JSON 的结构化诊断记录（见 docs/VISION_OVERHAUL.md §4）。
        //: 为什么要它：以前"识别不出"只能靠肉眼看日志里几个数字猜是相机、几何、角色还是
        //: 选墙出了问题；有了这份记录，每一帧的判定链条都能被机读地数出来。
        else if (k == "--diag-log") a.diag_log = next();
        else if (k == "--player-radius") a.player_radius = std::atof(next());
        else if (k == "--auto-thresholds")    a.auto_thresholds = true;
        else if (k == "--no-auto-thresholds") a.auto_thresholds = false;
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
                "  --no-auto-thresholds 关掉每帧自动阈值（对照用；默认开）\n"
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

/// 把「本帧统计推出的阈值」并进有效阈值：**命令行显式给的仍然最优先**
/// （`--wall-rb N` 说明人明确要看那个数，用来做对照实验）。
VisionParams apply_auto_thresholds(const Args& a, VisionParams p,
                                  const auto_params::Derived& d) {
    if (a.ov_wall_rb < 0)       p.wall_rb = d.wall_rb;
    if (a.ov_wall_min_r < 0)    p.wall_min_r = d.wall_min_r;
    if (a.ov_wall_min_g < 0)    p.wall_min_g = d.wall_min_g;
    if (a.ov_bright_min < 0)    p.bright_min = d.bright_min;
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
/// 门的**时序平滑**（"颜色分割 → 掩膜修复 → 形状/连通性 → 时序跟踪"的最后一级）。
///
/// 为什么必须有：`band_gap` 每帧独立重算，真机位上门心的列坐标会抖 ±10 px，
/// 而决策对它做符号判断 —— 于是同一个 tier 里指令左右来回翻（实测翻转里
/// `ALIGN → ALIGN`、`NO_GAP → NO_GAP` 占了一半）。
///
/// 关联/接纳的门槛全部用**门自己的尺寸**表达（宽、带厚），不是像素常数：
/// 门心偏离不到一个门宽、宽度变化不到半个门宽、行偏移不到两个带厚 → 认为是同一扇门，
/// 做指数平滑；否则直接换成新门（走开了就是另一扇）。
struct DoorSmoother {
    bool ok = false;
    double cx = 0.0, width = 0.0, y = 0.0, hh = 0.0;
    //: 门在动时的速度（场地单位/秒）。**必须有**：注入的矩形如果 vx=vy=0，
    //: `gap_avoid` 就认为墙是静止的，不会提前切向；而真平台上的墙是扫动的，
    //: 于是"照着上一帧的缺口走"必然撞 —— 真机闭环 13~14 次碰撞的主要来源。
    double vx = 0.0, vy = 0.0;
    double prev_cx = 0.0, prev_y = 0.0, prev_t = 0.0;
    bool has_prev = false;
    //: 跳变等待状态（见 update() 里"连续两帧才采纳"）
    int jump_pending = 0;
    double pend_cx = 0.0, pend_width = 0.0, pend_y = 0.0, pend_hh = 0.0;

    /// 返回平滑后的 (cx, width, y, hh)，并更新速度估计。
    void update(double ncx, double nwidth, double ny, double nhh, double t_now = 0.0) {
        const double old_cx = cx, old_y = y;
        if (!ok) {
            ok = true;
            cx = ncx; width = nwidth; y = ny; hh = nhh;
            return;
        }
        const bool same = std::fabs(ncx - cx) <= (width > 1.0 ? width : 1.0) &&
                          std::fabs(nwidth - width) <= (0.5 * width + 1.0) &&
                          std::fabs(ny - y) <= (2.0 * hh + 1.0);
        if (!same) {
            // **目标跳变要"连续两帧"才采纳**。
            // 为什么：检测器在同一条墙带上会在几个候选缺口之间来回选，一帧跳到别处、
            // 下一帧又跳回来。原来的实现遇到"不是同一扇门"就**立即采纳**，于是
            // 平滑后的目标也跟着跳 —— 实测这正是"平台左右乱晃"的主因：
            // 缺口跳变 >20 单位的帧里，指令方向翻转率 **29%**，而缺口平稳的帧只有 **8%**。
            // 现在：同一处跳变连续出现 2 帧才认（≈120 ms），单帧毛刺不再改变目标。
            if (jump_pending > 0 &&
                std::fabs(ncx - pend_cx) <= (pend_width > 1.0 ? pend_width : 1.0)) {
                ++jump_pending;
            } else {
                jump_pending = 1;
            }
            pend_cx = ncx; pend_width = nwidth; pend_y = ny; pend_hh = nhh;
            if (jump_pending >= 2) {
                cx = ncx; width = nwidth; y = ny; hh = nhh;
                jump_pending = 0;
            }
            return;
        }
        jump_pending = 0;               // 回到同一扇门：清掉跳变计数
        constexpr double kAlpha = 1.0 / 3.0;   // 三帧等效平均：够快跟手，又能压抖动
        cx += kAlpha * (ncx - cx);
        width += kAlpha * (nwidth - width);
        y += kAlpha * (ny - y);
        hh += kAlpha * (nhh - hh);
        // 速度：用**平滑后**的位置差分（差分放大抖动，平滑后再差分才稳）
        if (has_prev && t_now > prev_t) {
            const double dtv = t_now - prev_t;
            const double k = 0.5;              // 半步低通，够跟手也不抖
            vx += k * ((cx - old_cx) / dtv - vx);
            vy += k * ((y - old_y) / dtv - vy);
        }
        prev_cx = cx; prev_y = y; prev_t = t_now; has_prev = true;
    }
};

/// **带惯性的世界模型**（用户提的方案）：识别不到时按已建模的位置+速度外推。
///
/// 为什么需要：`band_gap` 每帧独立重算，真机位上门/带会间歇性丢（反光、遮挡、
/// 曝光抖动）。直接"丢了就 stay"会让平台一顿一顿；而"丢了就沿上一条指令"
/// 又会在镜头被挡时一直开（这个坑已经踩过）。
///
/// 规矩：
/// * 只桥接**短暂的漏检**，不桥接"看不见场地" —— 后者由 `field_extent` 的显著性
///   门直接判掉（一律 stay），所以这里不需要再设安全超时；
/// * 关联用**墙带自己的厚度**作容差（不是像素常数）；
/// * 外推距离一旦超过**半个缺口宽**，这条带就作废（目标可能已经挪到别处，
///   继续照着旧缺口走就是撞）。
struct BandTrack {
    double y = 0.0, cx = 0.0, w = 0.0, hh = 0.0;
    double vx = 0.0, vy = 0.0;
    int misses = 0;
};

struct WorldModel {
    std::vector<BandTrack> tracks;
    uint64_t last_ns = 0;
    bool has = false;

    /// 用本帧检出的门（场地单位）更新模型；`dt` 用于速度差分。
    void update(const std::vector<std::array<double, 4>>& dets, double dt) {
        std::vector<bool> used(dets.size(), false);
        for (BandTrack& tr : tracks) {
            int best = -1;
            double best_dy = 1e18;
            for (size_t i = 0; i < dets.size(); ++i) {
                if (used[i]) continue;
                const double dy = std::fabs(dets[i][1] - tr.y);
                if (dy <= 2.0 * tr.hh + 1.0 && dy < best_dy) {
                    best_dy = dy;
                    best = static_cast<int>(i);
                }
            }
            if (best < 0) {
                ++tr.misses;
                continue;
            }
            const auto& d = dets[static_cast<size_t>(best)];
            used[static_cast<size_t>(best)] = true;
            tr.misses = 0;
            constexpr double kA = 1.0 / 3.0;
            if (dt > 1e-4) {
                const double nvx = (d[0] - tr.cx) / dt;
                const double nvy = (d[1] - tr.y) / dt;
                tr.vx += 0.5 * (nvx - tr.vx);
                tr.vy += 0.5 * (nvy - tr.vy);
            }
            tr.cx += kA * (d[0] - tr.cx);
            tr.y += kA * (d[1] - tr.y);
            tr.w += kA * (d[2] - tr.w);
            tr.hh += kA * (d[3] - tr.hh);
        }
        for (size_t i = 0; i < dets.size(); ++i) {
            if (used[i]) continue;
            tracks.push_back(BandTrack{dets[i][0], dets[i][1], dets[i][2], dets[i][3], 0, 0, 0});
        }
        last_ns = 0;
        has = true;
    }

    /// 模型外推：`dt` 秒后仍可用的带（超过半个缺口宽的位移 → 作废）。
    std::vector<BandTrack> predict(double dt) {
        std::vector<BandTrack> out;
        for (BandTrack& tr : tracks) {
            const double dx = tr.vx * dt;
            const double dy = tr.vy * dt;
            const double moved = std::hypot(dx, dy);
            if (tr.w > 1.0 && moved > 0.5 * tr.w) continue;   // 外推过头：作废
            BandTrack q = tr;
            q.cx += dx;
            q.y += dy;
            out.push_back(q);
        }
        return out;
    }
};

struct DecisionState {
    gap_avoid::GapMemory mem;
    //: P3 审计：本帧选中的墙（doors 下标）和选择理由。直接进诊断日志 ——
    //: "为什么瞄这扇门"必须是可读的一句话，不能只体现在最终指令里。
    int selected_door = -1;
    const char* select_reason = "none";
    //: `gap_avoid` 本帧的决策档位（"为什么不动"必须能从日志直接看出来）
    const char* last_tier = "none";
    gap_avoid::AlgoConfig cfg;
    //: P1：上一帧**实际发出**的单位方向（预测要用它，不能用帧间差分的"看到的速度"）
    gap_avoid::Vec2 last_cmd{0.0, 0.0};
    //: 虚拟真值：角色（积分 + 慢校正）、墙（队列 + 冻结 + 积分）
    virt::PlayerState vp;
    virt::WallQueue wq;
    //: 这一帧"选中那扇门"的场地量（墙队列的输入）
    double wq_last_y = 0.0, wq_last_gl = 0.0, wq_last_gr = 0.0;
    bool wq_last_seen = false;
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
    //: 矩形层碎片合并的次数（"一堵墙被切成几段"被并回来的总次数）。
    int n_merged_rects = 0;
    //: 有几帧被**边界余量**削过（朝外分量被衰减）—— "别贴边走"到底生效没有。
    int n_edge_limited = 0;
    //: 本帧没检出障碍、于是**沿用上一条有效指令**的帧数（见 decide() 里的说明）
    int n_hold_cmd = 0;
    //: 最后一次"几何有效"（至少有一个矩形）的 stamp。用来给"沿用上一条指令"封顶：
    //: 实测把相机挡住时，`field_extent` 仍会在噪声里"找出"边框（40 帧只有 1 帧报
    //: 看不到场地），角色也会在噪声里被当成亮点 —— 于是先前的"漏检也沿上一条指令"
    //: 会**一直开下去**（实测挡镜头时 40/40 帧都在发 @MOVE 1）。
    //: 规矩：只在看门狗窗口（500 ms，与协议一致）内允许沿用，超时一律 stay。
    uint64_t last_geom_ns = 0;
    //: 用结构判据重建几何的帧数（--structural-gap）
    int n_structural = 0;
    //: 看不到场地（边框显著性不足）的帧数；这些帧一律 stay（见 decide 顶部）
    int n_no_field_frames = 0;
    //: 角色被**夹回场地内**的帧数（说明换算把它算到场外了，必须记账）
    int n_clamped = 0;
    //: 门的时序平滑器（见 DoorSmoother 的注释）
    DoorSmoother door_sm;
    //: 带惯性的世界模型（见 WorldModel 的注释）
    WorldModel world_model;
    //: **目标墙锁定**：一旦选中某条墙带，就盯住它直到角色穿过它。
    //: 为什么必须有：场景常有 2 条横墙，按"离角色最近"选会在地带中间来回跳
    //: （|dy| 几乎相等），实测门的 y 带与真值一致只有 2/12，指令跟着抖。
    //: 锁定规则只用**符号**（角色在墙的哪一侧），没有任何阈值：
    //: 选定时记下 side = sign(角色y − 墙y)；side 翻号 = 已经穿过去了 → 换下一堵。
    bool door_locked = false;
    double lock_y = 0.0;
    double lock_side = 0.0;
    //: 构造时把"前后可控"打开（见构造函数）。
    DecisionState() {
        // ⚠ **这是本轮最关键的一行**：`AlgoConfig::front_back` 默认 false，
        //   而 false 时 `gap_avoid` 把**法向（前进）分量置 0**，只输出切向分量 ——
        //   于是指令永远是 (±1, 0)：贴着墙左右滑，永远不穿缺口，最后撞墙/撞角。
        //   真机闭环 13~14 次碰撞、指令无竖直分量，根因就是这里。
        //   平台的 9 个离散动作里有 N/S（前后），所以"前后可控"对它成立；
        //   这是**平台能力**，不是调参。
        cfg.front_back = true;
    }
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

    //: 让参考模型用**与 PL 完全同一套**阈值 —— `--verify-both` 的前提。
    //: 不设的话参考模型会用默认常量，自动阈值一开，两边比的就不是同一件事了。
    void set_vision_params(const VisionParams& p) {
        params_.wall_rb = p.wall_rb;
        params_.wall_min_r = p.wall_min_r;
        params_.wall_min_g = p.wall_min_g;
        params_.bright_min = p.bright_min;
        params_.bright_spread = p.bright_spread;
        params_.green_gm = p.green_gm;
        params_.green_min = p.green_min;
        params_.band_row_pct = p.band_row_pct;
        params_.band_join_rows = p.band_join_rows;
        params_.band_min_rows = p.band_min_rows;
        params_.field_col_min = p.field_col_min;
        params_.wall_col_pct = p.wall_col_pct;
        params_.wall_col_min = p.wall_col_min;
        params_.edge_min = p.edge_min;
        params_.edge_min_cols = p.edge_min_cols;
        params_.min_seg_px = p.min_seg_px;
        params_.min_gap_px = p.min_gap_px;
        params_.player_min_area = p.player_min_area;
        params_.player_max_area = p.player_max_area;
        params_.player_min_fill_pct = p.player_min_fill;
        params_.player_max_aspect_pct = p.player_max_aspect;
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

//: 延迟分解（板子内部那一段）：像素交到手上的时刻、取帧时的队深、累计丢掉的旧帧数。
//: 光看"采集→决策"一个总数分不清是**排队**还是**算得慢**，而这两者的修法完全不同。
struct FrameLat {
    uint64_t got_ns = 0;
    size_t queued = 0;
    long dropped = 0;
};

gap_avoid::Vec2 decide(GeometryFrame f, const Args& args, DecisionState& st,
                       const std::vector<band_gap::Door>* doors,
                       const band_gap::Extent* ext,
                       const band_gap::Player* splayer,
                       const rectify::Homography* hmap,
                       const FrameLat& lat) {
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

    // ---- 结构判据模式：先把几何**换算到场地单位** --------------------------
    // 视觉给的是像素，而 `gap_avoid` 的 `player_speed / edge_margin / player_radius`
    // 是场地单位（640x480）。比例尺从**场地边框在画面里的宽度**来（`field_extent`），
    // 不是假设 1:1 —— 实测场地只占 564~570 px，比例尺 1.12~1.14。
    // **看不到场地就一律 stay**：`field_extent` 的边框峰要 ≥8σ 显著，
    // 挡镜头/相机没对准时过不了这一关。这时任何几何（包括 RTL 的矩形）都不可信 ——
    // 实测挡镜头时 RTL 仍会吐噪声矩形，于是平台继续开（40/40 帧 @MOVE 1）。
    // 这条是硬规矩，不因为"也许能看见一点"而放宽。
    if (args.structural_gap && ext && !ext->ok) {
        ++st.n_no_field_frames;
        st.last_geom_ns = 0;          // 连"沿用上一条指令"也不许
        st.door_locked = false;
        return gap_avoid::Vec2{0.0, 0.0};
    }
    const bool use_struct = args.structural_gap && ext && ext->ok;
    const double s_scale = use_struct ? ext->scale_x : 1.0;
    const double s_scale_y = use_struct ? ext->scale_y : 1.0;
    if (use_struct) {
        // 角色优先用**PS 侧的结构检测**（最亮的圆，见 `band_gap::find_player`）：
        // RTL 的"亮 + 低饱和"在淡粉白的墙面前会把角色吞掉（实测 28/40 帧无角色），
        // 而板子的规矩是"没角色就 stay"。这里仍然只给像素坐标，下面统一换算。
        if (splayer && splayer->found && !doors->empty()) {
            f.player.present = true;
            if (hmap && hmap->ok) {
                const auto q = hmap->apply(splayer->x, splayer->y);
                f.player.x = q[0];
                f.player.y = q[1];
                // 半径用两点映射量（单应会把圆压成椭圆，取两轴平均）
                const double rr = 0.5 * static_cast<double>(splayer->w);
                const auto qx = hmap->apply(splayer->x + rr, splayer->y);
                const auto qy = hmap->apply(splayer->x, splayer->y + rr);
                const double rx = std::hypot(qx[0] - q[0], qx[1] - q[1]);
                const double ry = std::hypot(qy[0] - q[0], qy[1] - q[1]);
                f.player.radius = 0.5 * (rx + ry);
                // **掩膜**：单应映射后必须在场地四边形内，否则这一帧没有可信角色
                if (!hmap->inside(splayer->x, splayer->y, args.field_w, args.field_h, 0.0)) {
                    f.player.present = false;
                }
            } else {
                f.player.x = splayer->x;
                f.player.y = splayer->y;
                f.player.radius = 0.25 * static_cast<double>(splayer->w + splayer->h);
            }
        } else if (f.player.present) {
            // **PS 角色缺失时退回 RTL 角色 —— 必须换单位**。
            // RTL 几何帧里的角色是**图像像素（y 朝下）**，而下游 `gap_avoid` 要的是
            // 场地单位（y 朝上）。以前这里直接沿用，等于把一个 ~550px 的图像坐标当成
            // 场地坐标喂进规划器（差一个比例尺 + 一个上下镜像），
            // 表现是"PS 一失效就乱开"，而日志里看不出任何异常。
            if (hmap && hmap->ok) {
                const auto q = hmap->apply(f.player.x, f.player.y);
                const double rr = std::max(1.0, f.player.radius);
                const auto qx = hmap->apply(f.player.x + rr, f.player.y);
                const auto qy = hmap->apply(f.player.x, f.player.y + rr);
                const double rx = std::hypot(qx[0] - q[0], qx[1] - q[1]);
                const double ry = std::hypot(qy[0] - q[0], qy[1] - q[1]);
                f.player.x = q[0];
                f.player.y = q[1];
                f.player.radius = 0.5 * (rx + ry);
                if (!hmap->inside(f.player.x, f.player.y, args.field_w, args.field_h, 0.0)) {
                    f.player.present = false;
                }
            } else {
                f.player.present = false;   // 没有几何 → 图像坐标不能当真
            }
        }
        const double ox = static_cast<double>(ext->x0), oy = static_cast<double>(ext->y0);
        if (f.player.present) {
            f.player.x = (f.player.x - ox) * s_scale;
            f.player.y = (f.player.y - oy) * s_scale_y;
            f.player.radius *= 0.5 * (s_scale + s_scale_y);
        }
        // **角色一定在场地内**（平台契约）。越界只可能是换算误差 ——
        // 夹回去并记账，否则 `limit_edges` 以为还没到边，就会一直往框上开。
        if (f.player.present) {
            const double rr = f.player.radius;
            const double xmin = rr, xmax = args.field_w - rr;
            const double ymin = rr, ymax = args.field_h - rr;
            const double cx0 = f.player.x, cy0 = f.player.y;
            if (f.player.x < xmin) f.player.x = xmin;
            if (f.player.x > xmax) f.player.x = xmax;
            if (f.player.y < ymin) f.player.y = ymin;
            if (f.player.y > ymax) f.player.y = ymax;
            if (f.player.x != cx0 || f.player.y != cy0) ++st.n_clamped;
        }
        for (auto& o : f.obstacles) {
            o.x = (o.x - ox) * s_scale;
            o.y = (o.y - oy) * s_scale_y;
            o.half_w *= s_scale;
            o.half_h *= s_scale_y;
        }
    }
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
    // ⚠ **最后的兜底：角色不可能落在场地四角**。
    // 白框的四个角在相机里是"小而亮的紧凑块"，和角色圆盘几乎一样 —— 实测这一轮
    // 100% 的 `player_field_valid` 帧都落在左上角 40×40 像素内（988/991），
    // 于是决策一直以为角色在边上。这条判据放在**最终取值处**，对"新检测器 / 旧亮圆
    // 检测器 / 预测"三条来源一起生效（之前只加在新检测器上，旧检测器漏过去了）。
    bool player_on_corner = false;
    {
        const double pxu = player_ok ? f.player.x : args.field_w * 0.5;
        const double pyu = player_ok ? f.player.y : 0.0;
        const double keep = 24.0 + args.player_radius + 60.0;   // 角标记半边长+半径+余量
        const double cs[4][2] = {{32.0, 32.0}, {args.field_w - 32.0, 32.0},
                                 {32.0, args.field_h - 32.0},
                                 {args.field_w - 32.0, args.field_h - 32.0}};
        for (const auto& c : cs)
            if (std::hypot(pxu - c[0], pyu - c[1]) < keep) { player_on_corner = true; break; }
    }
    const double player[7] = {
        (player_ok && !player_on_corner) ? f.player.x : args.field_w * 0.5,
        (player_ok && !player_on_corner) ? f.player.y : 0.0,
        pvx, pvy,
        player_ok ? f.player.radius : 10.0,
        args.player_speed,
        1.0,
    };

    // ---- 虚拟真值：角色（视觉只做初始化 + 慢校正）------------------------
    // 为什么不再每帧都用视觉：相机端到端 0.56 板秒（= 78 场地单位），而视觉在
    // "墙经过角色"时会丢帧或漂移 —— 那正是最需要它的时刻。改成积分维持后，
    // 丢帧只影响校正，不影响位置。
    if (args.virt) {
        st.vp.gate = args.virt_gate;
        st.vp.alpha = args.virt_alpha;
        if (!st.vp.locked) {
            if (player_ok) st.vp.observe_init(player[0], player[1]);
            st.vp.try_lock();
        } else {
            st.vp.integrate(st.last_cmd.x, st.last_cmd.y, dt, args.predict_speed,
                            args.field_w, args.field_h, player[4]);
            // **只在"有检出 且 没漂移"时喂视觉**；否则这一帧完全不用它
            // **只在"识别到 且 不漂移"时更新真值**：没检出（被墙盖住/贴墙）
            // 或行为不一致（没发指令却动了、动得超出逻辑）→ 这一帧完全不喂。
            // **不做任何校正**（用户口径）：锁定初始位置之后，角色位置**完全**由
            // 指令积分决定；视觉的角色结果一概不进虚拟真值。
            // 仍然调用 `vision_ok` 只为**统计漂移/缺失**（`n_reject_*`）与"严重分叉
            // 时重锚"这一个安全阀；它不再修改位置（原来那句 `correct()` 已删除）。
            if (player_ok) {
                (void)st.vp.vision_ok(player[0], player[1], t_now,
                                      st.last_cmd.x, st.last_cmd.y, args.predict_speed, dt);
            }
        }
    }

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

    // ---- 结构判据：用门的左右两段墙**顶掉**碎矩形 ------------------------
    // 为什么不是"补充"：碎块每帧个数/位置都在变，混进去只会让 `group_walls`
    // 把同一堵墙分成两组。结构判据给出的是**这堵墙的完整几何**（两段 + 缺口），
    // 语义上就是 `gap_avoid` 想要的输入。
    // 结构模式：几何**只**来自结构判据，不回退 RTL 的矩形。
    // 为什么：镜头被挡时 RTL 仍会吐噪声矩形（实测 30/40 帧因此继续发指令），
    // 把它们当兜底等于给噪声开门。没有门 → 没有几何 → 沿用的上限是看门狗 500 ms。
    if (use_struct) {
        rects.clear();
    }
    if (use_struct && !doors->empty()) {
        const double ox = static_cast<double>(ext->x0), oy = static_cast<double>(ext->y0);
        int ent = 0;
        // 选**角色正对着的那扇门**：按"墙带中心与角色的行距离"取最近。
        // 为什么不是全部注入，也不是只取最宽：场景里常有 2 条横墙，各自缺口位置不同；
        // 全部注入时 `gap_avoid` 会在两堵墙之间来回换目标（实测翻转 8%），
        // 只取最宽又会选到玩家不去的那堵。角色所在行是**唯一**能把"我要过哪个门"
        // 定下来的信息，而且它不需要任何阈值。
        // 目标墙锁定（符号判据，见 DecisionState::door_locked 的注释）
        const band_gap::Door* pick = nullptr;
        for (const band_gap::Door& d : *doors) {
            const double wy = 0.5 * ((d.y0 - oy) + (d.y1 - oy)) * s_scale_y;
            const double side = f.player.y - wy;
            if (st.door_locked) {
                const double tol = 2.0 * 0.5 * (static_cast<double>(d.y1 - d.y0) + 1.0) * s_scale_y;
                const bool same_wall = std::fabs(wy - st.lock_y) <= tol + 1.0;
                const bool crossed = (side * st.lock_side) < 0.0;
                if (same_wall && !crossed) {
                    // 还在这堵墙的这一侧：继续盯住它
                    st.lock_y = wy;
                    pick = &d;
                    st.select_reason = "locked";
                    st.selected_door = static_cast<int>(&d - &(*doors)[0]);
                }
            }
        }
        if (pick == nullptr) {
            // **P3**：选"角色前方最近的那道墙"，而不是"图像里最近的那道"。
            // 两者的差别在"角色已经越过某道墙"时才显现：按 |wy - player.y| 选，
            // 刚过身后的那道墙距离最小 → 平台会回头去瞄一扇已经过去的门。
            // 定义与单测在 `wall_gap::select_threat_wall`（12 条断言覆盖单墙/双墙/
            // 顺序互换/已越过/迟滞五种情形）。
            std::vector<wall_gap::WallInField> wf;
            for (size_t di = 0; di < doors->size(); ++di) {
                const band_gap::Door& d = (*doors)[di];
                wall_gap::WallInField w;
                w.id = static_cast<int>(di);
                w.y_center = 0.5 * ((d.y0 - oy) + (d.y1 - oy)) * s_scale_y;
                w.half_h = 0.5 * (static_cast<double>(d.y1 - d.y0) + 1.0) * s_scale_y;
                w.has_gap = d.found && d.width > 0;
                w.gap_cx = 0.5 * ((d.x0 - ox) + (d.x1 - ox)) * s_scale;
                w.gap_w = static_cast<double>(d.width) * s_scale;
                wf.push_back(w);
            }
            const int sel = wall_gap::select_threat_wall(wf, f.player.y);
            if (sel >= 0) {
                pick = &(*doors)[static_cast<size_t>(sel)];
                st.select_reason = "ahead_nearest";
            } else {
                // 没有"前方"的墙（全被越过/全无缺口）→ 退回最近的那道，
                // 而不是乱选；这一步保留旧行为，避免在边缘情形下反而无目标。
                double best_dy = 1e18;
                for (const band_gap::Door& d : *doors) {
                    const double wy = 0.5 * ((d.y0 - oy) + (d.y1 - oy)) * s_scale_y;
                    const double dy = std::fabs(wy - f.player.y);
                    if (dy < best_dy) { best_dy = dy; pick = &d; }
                }
                if (pick != nullptr) st.select_reason = "fallback_nearest";
            }
            if (pick != nullptr) st.selected_door = static_cast<int>(pick - &(*doors)[0]);
            if (pick != nullptr) {
                const double wy = 0.5 * ((pick->y0 - oy) + (pick->y1 - oy)) * s_scale_y;
                st.door_locked = true;
                st.lock_y = wy;
                st.lock_side = f.player.y - wy;
                if (st.lock_side == 0.0) st.lock_side = 1.0;
            }
        }
        if (pick == nullptr) { return gap_avoid::Vec2{0.0, 0.0}; }
        // 先平滑，再用平滑后的几何去铺两段墙（否则门心抖动会直接变成指令抖动）
        {
            const double ncx = 0.5 * ((pick->x0 - ox) + (pick->x1 - ox)) * s_scale;
            const double nwidth = static_cast<double>(pick->width) * s_scale;
            const double ny = 0.5 * ((pick->y0 - oy) + (pick->y1 - oy)) * s_scale_y;
            const double nhh = 0.5 * (static_cast<double>(pick->y1 - pick->y0) + 1.0) * s_scale_y;
            st.door_sm.update(ncx, nwidth, ny, nhh, t_now);
        }
        // **缺口中心也要"范围"而不是"点值"**：缺口帧间跳变 p90 28 场地单位，
        // 而横向可用宽度只有 25 单位 —— 目标抖一点就"左一下右一下"。
        // 在 α-β 平滑之外再套一层时间窗鲁棒均值（截尾 + MAD 剔离群）。
        static robust::Window s_win_gap_cx(0.35, 10);
        static robust::Window s_win_gap_y(0.35, 10);
        s_win_gap_cx.push(st.door_sm.cx, t_now);
        s_win_gap_y.push(st.door_sm.y, t_now);
        const double sm_cx = s_win_gap_cx.enough(3) ? s_win_gap_cx.robust_mean() : st.door_sm.cx;
        const double sm_w = st.door_sm.width;
        const double sm_y = s_win_gap_y.enough(3) ? s_win_gap_y.robust_mean() : st.door_sm.y;
        const double sm_hh = st.door_sm.hh;
        // 注入**所有**墙（每扇门各自带缺口），但把"角色正对着的那扇"用平滑后的几何。
        // 只注入一扇会漏掉另一堵墙 —— 规划器看不见它，于是直直撞上去
        // （真机闭环实测 13 次碰撞，多数是撞在没被注入的那堵墙上）。
        // 其余墙用原始几何（它们不是当前目标，抖动不影响指令）。
        for (const band_gap::Door& d : *doors) {
            const bool near_one = (&d == pick);
            double by, hh;
            if (hmap && hmap->ok) {
                // 带中心行与上下沿都过单应：y 才和 x 在同一个坐标系里
                const double band_row = 0.5 * (static_cast<double>(d.y0) + d.y1);
                const double cx_img = 0.5 * (static_cast<double>(d.x0) + d.x1);
                by = hmap->apply(cx_img, band_row)[1];
                const double y_top = hmap->apply(cx_img, static_cast<double>(d.y0))[1];
                const double y_bot = hmap->apply(cx_img, static_cast<double>(d.y1))[1];
                hh = 0.5 * std::fabs(y_bot - y_top);
                if (near_one) { by = sm_y; hh = sm_hh; }
            } else {
                by = near_one ? sm_y : 0.5 * ((d.y0 - oy) + (d.y1 - oy)) * s_scale_y;
                hh = near_one ? sm_hh
                              : 0.5 * (static_cast<double>(d.y1 - d.y0) + 1.0) * s_scale_y;
            }
            double gl, gr;
            if (hmap && hmap->ok) {
                const double band_row = 0.5 * (static_cast<double>(d.y0) + d.y1);
                gl = hmap->apply(static_cast<double>(d.x0), band_row)[0];
                gr = hmap->apply(static_cast<double>(d.x1), band_row)[0];
                if (near_one) { gl = sm_cx - 0.5 * sm_w; gr = sm_cx + 0.5 * sm_w; }
            } else {
                gl = near_one ? sm_cx - 0.5 * sm_w : (static_cast<double>(d.x0) - ox) * s_scale;
                gr = near_one ? sm_cx + 0.5 * sm_w : (static_cast<double>(d.x1) - ox) * s_scale;
            }
            // 缺口只在**场地内**才有意义（场外不算可通行）
            if (gl < 0.0) gl = 0.0;
            if (gr > args.field_w) gr = args.field_w;
            if (gr - gl < 1.0) continue;    // 整个缺口在场外 → 这扇门不可用
            // 虚拟真值：记下这一帧"选中那扇门"的场地量（y / 缺口两端）
            if (&d == pick) {
                st.wq_last_y = by; st.wq_last_gl = gl; st.wq_last_gr = gr; st.wq_last_seen = true;
            }
            auto push_seg = [&](double a, double b) {
                gap_avoid::Rect r;
                r.ent_id = ++ent;
                r.x = 0.5 * (a + b);
                r.y = by;
                r.half_w = 0.5 * (b - a);
                r.half_h = hh;
                r.rotation = 0.0;
                // 墙在动：把门的速度填进矩形（场地单位/秒）。
                // 只给"角色正对着的那堵"填 —— 它才是当前目标，速度估计也只对它可信。
                r.vx = near_one ? st.door_sm.vx : 0.0;
                r.vy = near_one ? st.door_sm.vy : 0.0;
                r.angular_velocity = 0.0;
                rects.push_back(r);
            };
            if (gl > 1.0) push_seg(0.0, gl);
            if (args.field_w - gr > 1.0) push_seg(gr, args.field_w);
        }
        // ---- 虚拟真值：墙队列（视觉只负责"发现新墙 + 初始化"）----------------
        // 冻结之后**视觉不再刷新这一面墙**：它只按自己测到的 vy 匀速走。
        // 这样墙走到画面底部/贴住角色时，位置不会跟着每次重测去跳。
        if (args.virt) {
            st.wq.update(t_now, st.wq_last_seen, st.wq_last_y, st.wq_last_gl, st.wq_last_gr);
            st.wq.advance(t_now);
            st.wq.purge(args.field_h);
            st.wq_last_seen = false;
            if (st.wq.frozen_count() > 0) {
                // ⚠ **不能直接 clear()**：那样会把"前方还没冻结的新墙"一起清掉，
                // 决策就只剩身后那面已冻结的墙 —— 而它不在前方，于是 select_threat_wall
                // 找不到目标 → `stay` 不动。实测用户看到的"过了缺口就停住"就是这个
                // （用真值直接喂决策时 4718 帧里一次 stay 都没有，说明决策层本身没错）。
                // 正确做法：**只替换与已冻结墙重合的矩形**，其余（新墙/未冻结）保留。
                std::vector<gap_avoid::Rect> keep;
                keep.reserve(rects.size());
                for (const auto& r : rects) {
                    bool matched = false;
                    for (const auto& sl : st.wq.slots) {
                        if (sl.frozen && std::fabs(r.y - sl.y) <= 40.0) { matched = true; break; }
                    }
                    if (!matched) keep.push_back(r);
                }
                rects.swap(keep);
                int vid = 0;
                for (const auto& sl : st.wq.slots) {
                    if (!sl.frozen) continue;
                    auto push = [&](double a, double b) {
                        gap_avoid::Rect r;
                        r.ent_id = ++vid;
                        r.x = 0.5 * (a + b);
                        r.y = sl.y;
                        r.half_w = 0.5 * (b - a);
                        r.half_h = 4.0;            // 契约：墙全厚 8 场地单位
                        r.vx = 0.0;
                        r.vy = sl.vy;              // 逼近速度（决策要用它算"墙在压过来"）
                        r.rotation = 0.0;
                        rects.push_back(r);
                    };
                    if (sl.gap_l > 1.0) push(0.0, sl.gap_l);
                    if (args.field_w - sl.gap_r > 1.0) push(sl.gap_r, args.field_w);
                }
            }
        }
        if (!rects.empty()) ++st.n_structural;
        if (args.verbose && rects.size() >= 2) {
            static long shown = 0;
            if ((++shown % 8) == 1) {
                std::printf("  [注入#%ld f=%llu] 角色=(%.0f,%.0f) r=%.1f | 门=(%.0f,%.0f) 宽=%.0f 带厚=%.1f"
                            " | 左墙 x=%.0f±%.0f 右墙 x=%.0f±%.0f"
                            " | 跨尺 sx=%.4f sy=%.4f oy=%d\n",
                            shown,
                            f.player.x, f.player.y, f.player.radius,
                            sm_cx, sm_y, sm_w, sm_hh,
                            shown, static_cast<unsigned long long>(f.header.seq),
                            f.player.x, f.player.y, f.player.radius,
                            rects[0].x, rects[0].half_w,
                            rects[1].x, rects[1].half_w,
                            s_scale, s_scale_y, ext->y0);
            }
        }
    }

    // ---- 矩形层碎片合并（颜色分割 → 掩膜修复 → **形状/连通性** → 时序跟踪）----
    int n_merged = 0;
    if (args.merge_gap_px > 0.0 && rects.size() > 1) {
        segment_merge::Params mp;
        mp.max_gap = args.merge_gap_px;
        mp.max_dy = args.merge_dy_px;
        rects = segment_merge::merge_collinear(rects, mp, &n_merged);
    }
    if (n_merged > 0) st.n_merged_rects += n_merged;

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
        // ---- P1：障碍按**跟踪器给的速度**前推 ------------------------------
        // 速度来自 `motion::Tracker`（最近邻关联），不是"这一帧和上一帧选同一个带"
        // ——后者在有多道墙时会跳到另一道墙上（实测 |v_y| p90 815 px/s 全是假速度）。
        // 墙在场景里确实在动（实测 128 场地单位/秒向下），0.5 s 就是 64 单位，
        // 比缺口半宽还大，所以"墙 y"必须一起前推才谈得上瞄得准。
        if (args.predict_s > 0.0) {
            const double k = args.predict_s;
            for (size_t i = 0; i < rects.size(); ++i) {
                rects[i].x += rects[i].vx * k;
                rects[i].y += rects[i].vy * k;
            }
        }
    }

    // ---- P1：把角色推进到"这条指令真正生效"的那一刻 ------------------------
    // 为什么用**上一帧发出的指令**而不是帧间差分：平台上看到的位移已经含了
    // 0.5 s 的延迟，用它外推只会把延迟再算一遍；而"我们自己刚发的是什么方向"
    // 是零延迟的已知量。速度取 `boot_cal` 实测的 156.6 单位/板秒。
    double player_pred[7] = {player[0], player[1], player[2], player[3],
                             player[4], player[5], player[6]};
    if (args.virt && st.vp.locked) {
        // 虚拟真值的角色位置是**零延迟**的，直接用它，不再叠 P1 的外推
        player_pred[0] = st.vp.x;
        player_pred[1] = st.vp.y;
        // 速度也用虚拟真值：**发出去什么方向、就以什么速度在动**（单位时间位移
        // 由积分保证），而不是拿视觉帧间差分那个抖动的"看到的速度"。
        player_pred[2] = st.last_cmd.x * args.predict_speed;
        player_pred[3] = st.last_cmd.y * args.predict_speed;
    } else if (args.predict_s > 0.0 && player_ok) {
        const double k = args.predict_s * args.predict_speed;
        player_pred[0] += st.last_cmd.x * k;
        player_pred[1] += st.last_cmd.y * k;
        // 前推后必须留在场地内，否则规划器会把它当成"贴边"
        const double m = std::max(1.0, player_pred[4] + 1.0);
        player_pred[0] = std::min(std::max(player_pred[0], m), args.field_w - m);
        player_pred[1] = std::min(std::max(player_pred[1], m), args.field_h - m);
    }

    gap_avoid::WorldView world;
    world.player = player_pred;
    world.rects = rects.data();
    world.n_rects = rects.size();
    world.dt = dt;
    world.step_index = static_cast<int64_t>(f.header.seq);
    world.field_w = args.field_w;
    world.field_h = args.field_h;

    st.cfg.edge_margin = args.edge_margin;
    // **A 方案**：打开前后控制并给前进一个固定占比（0.6），让指令能"斜着直奔缺口"。
    // 默认 `front_back=false` + `fwd_speed=0` 时，规划器的前进分量恒为 0，只会横着走。
    st.cfg.front_back = true;
    st.cfg.fwd_speed = 0.6;      // 边界余量（0 = 关）
    const gap_avoid::Decision d = gap_avoid::plan(world, st.mem, st.cfg, 2.0);
    if (d.edge_limited) ++st.n_edge_limited;
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
    if (!rects.empty()) st.last_geom_ns = f.header.stamp_ns;

    // 本帧一个障碍都没检出：**不要**当成"该停"。
    // 板子控平台时场景**只有一种障碍**（--obstacle-type 强约束），所以"这帧没检出"
    // 是漏检，不是"墙没了"。实测 43% 的帧零矩形，指令就在"走"与"停"之间来回跳，
    // 角色在出发点附近乱晃。物理事实是墙还在 —— 保持上一条有效指令。
    static gap_avoid::Vec2 last_cmd{0.0, 0.0};
    static bool have_cmd = false;
    //: 沿用的前提：最近 500 ms 内**真的**有过几何（看门狗窗口，与协议一致）。
    //: 没有这个前提，镜头被挡住时会拿着一条旧指令一直开（实测 40/40 帧）。
    const bool geom_fresh = st.last_geom_ns != 0 &&
                            f.header.stamp_ns > st.last_geom_ns &&
                            (f.header.stamp_ns - st.last_geom_ns) < 500ull * 1000ull * 1000ull;
    if (rects.empty() && geom_fresh && have_cmd && player_ok
        && cmd.x == 0.0 && cmd.y == 0.0) {
        cmd = last_cmd;
        ++st.n_hold_cmd;
    } else if (player_ok && (cmd.x != 0.0 || cmd.y != 0.0)) {
        last_cmd = cmd;
        have_cmd = true;
    }
    //: P1 用：本帧最终发出的**单位方向**（没有速度模长的概念，只有方向）
    {
        const double n = std::hypot(cmd.x, cmd.y);
        st.last_cmd = (n > 1e-9) ? gap_avoid::Vec2{cmd.x / n, cmd.y / n}
                                 : gap_avoid::Vec2{0.0, 0.0};
    }
    ++st.n_decisions;
    st.tier_count[tier_bucket(d.tier)] += 1;
    st.last_tier = d.tier;

    // 先把**决策真正用的**角色位置打出来（`f.player` 在 decide 里可能被结构判据覆盖）：
    // 之前这里只打几何帧里的角色，结构判据覆盖后日志里还是 (0,0)，符号校准没法做。
    std::printf("        角色用=(%+.1f,%+.1f) r=%+.1f\n", f.player.x, f.player.y,
                f.player.radius);
    // **端到端延迟（板子内部这一段）**：从相机给帧的 stamp 到本帧决策算完，
    // 用板子自己的单调时钟量，不跨机器、不需要对时。
    {
        const auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        const uint64_t cam_ns = f.header.stamp_ns;
        if (cam_ns != 0 && lat.got_ns > cam_ns && now_ns > cam_ns) {
            const double age = (lat.got_ns > cam_ns) ? (lat.got_ns - cam_ns) * 1e-6 : 0.0;
            const double proc = (now_ns > lat.got_ns) ? (now_ns - lat.got_ns) * 1e-6 : 0.0;
            std::printf("        @LAT 取帧滞后 %.1f + 处理 %.1f = 采集→决策 %.1f ms"
                        "（取帧时队列=%zu，累计丢旧帧=%ld）\n",
                        age, proc, (now_ns - cam_ns) * 1e-6, lat.queued, lat.dropped);
        }
    }
    // 给**平台 HUD** 的一行（`@DBG` 前缀由 board_link 解析）：视觉这条链路上
    // 也能在屏幕上看到板子的决策，和真值链路一致。
    std::printf("@DBG tier=%s cmd=(%.2f,%.2f) gap=%.1f rects=%d "
                "player=(%.0f,%.0f) aim=(%.0f,%.0f)\n",
                d.tier, cmd.x, cmd.y,
                std::isfinite(d.gap) ? d.gap : -1.0, d.n_rects,
                f.player.x, f.player.y,
                d.aim_world.x == d.aim_world.x ? d.aim_world.x : 0.0,
                d.aim_world.y == d.aim_world.y ? d.aim_world.y : 0.0);
    std::printf("决策: tier=%-7s 指令=(%+.3f, %+.3f)  "
                "t_arr=%.2fs 间隙=%+.1f 可用半宽=%+.1f 矩形=%d",
                d.tier, cmd.x, cmd.y,
                std::isfinite(d.t_arr) ? d.t_arr : -1.0,
                std::isfinite(d.gap) ? d.gap : -1.0,
                std::isfinite(d.usable_half) ? d.usable_half : -1.0,
                d.n_rects);
    if (n_degenerate > 0) std::printf("（丢了 %d 个退化矩形）", n_degenerate);
    if (n_merged > 0) std::printf("（并了 %d 段碎墙）", n_merged);
    if (d.edge_limited) std::printf("（边界限速）");
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

// ---------------------------------------------------------------------------
// 开场标定（`--cal <秒>`）：用**已知动作序列**量平台的速度和指令延迟
//
// 为什么必须单独做这一件事：虚拟真值（把 `@MOVE` 积分成平台位置）依赖两个数——
// 平台速度、指令→动作延迟。以前的 158.7 单位/秒是从离线语料反推的，延迟只量到
// 一个总数（0.457 s，混着串口、键盘桥、采集三处）。这两个数一旦错，积分就会
// 系统性偏离真值，而"用视觉去修积分"看起来永远像视觉有问题。
//
// 序列设计（一段 20.2 s，可重复）：
//   * 头 1 s 停住 —— 静止位置样本（多帧取稳健均值）；
//   * 之后 8 个匀速段各 2 s：左、右、下、上、左上、右上、左下、右下，
//     段间停 0.4 s —— 停顿让"上一段减速→下一段起步"的边界清晰，
//     每段的位移/时间就是速度，起步滞后就是延迟；
//   * 顺序上先水平后垂直，覆盖 8 个方向（含对角 0.7071 分量，用来验
//     "对角速度是否也是同一个模"）。
// ---------------------------------------------------------------------------
namespace calib {

struct Step {
    double dur;     // 这一段的时长（秒）
    int    action;  // 这一段的动作号（1 左 2 右 3 上 4 下 5..8 对角，0 停）
};

//: 一轮标定的动作表：**前缀只跑一次**（静止采样 + 回中），**脉冲段循环**。
//:
//: 为什么段长从 2 s 改成 0.6 s：2 s 在 166 单位/秒下就是 **330 单位**位移 ——
//: 场地只有 640×480，平台从 (320,106) 出发往左 2 s 会**撞到左边界被钳住**，
//: 量出来的"速度"是钳位后的平均速度（实测 y 方向 42.6、对角 51.8，全是钳位造成的
//: 假值）。0.6 s = 100 单位，从场地中央出发八个方向都不会碰到边界。
//:
//: 为什么先"回中"：角色出生点在场地**靠下**（实测 (320,106)，场地 -y 方向只剩 96
//: 单位），不回中就没法量向下的速度。回中方向/时长用**视觉给的位置**算（静止时
//: 视觉误差中位 14 单位，够用），速度按契约 200 估（这里只需要走到中央附近，
//: 不要求精确 —— 精确的那个数正是后面脉冲段要量的）。
struct Plan {
    std::vector<Step> prefix;   // 只跑一次（静止采样、回中）
    std::vector<Step> pulses;   // 循环跑（八个方向各一次 + 段间停）
    double prefix_s = 0.0;
    double pulse_s = 0.0;
};

inline double sum_dur(const std::vector<Step>& v) {
    double s = 0.0;
    for (const Step& st : v) s += st.dur;
    return s;
}

//: 用"角色现在在哪"生成标定计划。`have` == false 时跳过回中（位置未知就别乱走）。
inline Plan build(double x, double y, bool have, double field_w, double field_h) {
    Plan pl;
    pl.prefix.push_back({1.0, 0});          // 静止采样（初始位置）
    if (have) {
        const double cx = 0.5 * field_w, cy = 0.5 * field_h;
        // 回中速度按 200 估：只求"走到中央附近"，不精确
        const double dy = cy - y, dx = cx - x;
        double vd = std::min(0.9, std::fabs(dy) / 200.0);
        if (std::fabs(dy) > 60.0 && vd > 0.15) {
            pl.prefix.push_back({vd, dy > 0.0 ? 3 : 4});
            pl.prefix.push_back({0.3, 0});
        }
        double hd = std::min(0.9, std::fabs(dx) / 200.0);
        if (std::fabs(dx) > 60.0 && hd > 0.15) {
            pl.prefix.push_back({hd, dx > 0.0 ? 2 : 1});
            pl.prefix.push_back({0.3, 0});
        }
    }
    // 八个方向各 **0.8 s**，段间停 0.3 s；成对安排让位置大致回到原点。
    // 0.8 s 在 200 单位/秒下约 160 单位 —— **从场地中央出发，八个方向都不会碰边界**
    // （中央到最近边界 230 单位，斜向每轴 113 单位）。1.2 s 就太长了：从中央往下
    // 240 单位会直接撞到底边被钳住，量出来的速度是钳位后的假值（实测出现过
    // 119 单位/秒、分方向 114~126 的荒谬结果）。
    static const int kDirs[] = {1, 2, 4, 3, 5, 8, 6, 7};
    for (int d : kDirs) {
        pl.pulses.push_back({0.8, d});
        pl.pulses.push_back({0.3, 0});
    }
    pl.prefix_s = sum_dur(pl.prefix);
    pl.pulse_s = sum_dur(pl.pulses);
    return pl;
}

//: `t`（标定窗口内的秒数）对应的 (动作号, 段序号, 段内相对时间)。
//: 段序号**单调递增**（脉冲段跨轮次继续编号），分析脚本据此切样本。
struct Point { int action; int step; double t_in_step; };

inline Point at(const Plan& pl, double t) {
    Point p{0, -1, 0.0};
    if (t < 0.0 || pl.pulses.empty()) return p;
    if (t < pl.prefix_s) {
        double x = t;
        for (size_t i = 0; i < pl.prefix.size(); ++i) {
            if (x < pl.prefix[i].dur || i + 1 == pl.prefix.size()) {
                p.action = pl.prefix[i].action;
                p.step = static_cast<int>(i);
                p.t_in_step = x;
                return p;
            }
            x -= pl.prefix[i].dur;
        }
        return p;
    }
    double x = t - pl.prefix_s;
    const int cycle = static_cast<int>(x / pl.pulse_s);
    x -= cycle * pl.pulse_s;
    const int n = static_cast<int>(pl.pulses.size());
    for (int i = 0; i < n; ++i) {
        if (x < pl.pulses[i].dur || i + 1 == n) {
            p.action = pl.pulses[i].action;
            p.step = static_cast<int>(pl.prefix.size()) + cycle * n + i;
            p.t_in_step = x;
            return p;
        }
        x -= pl.pulses[i].dur;
    }
    return p;
}

}   // namespace calib

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
        pump->set_drop_stale(!args.keep_stale);
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

    // ---- 结构化诊断记录（--diag-log）--------------------------------------
    // 只写事实，不写推断：几何是否有效、几个标记、角色在哪、墙/缺口在哪、最后发了什么。
    // `coordinate_error` 之类的真值误差**不在这里算** —— 真值不来自算法自身（不变式 I5）。
    FILE* diag_f = nullptr;
    if (!args.diag_log.empty()) {
        diag_f = std::fopen(args.diag_log.c_str(), "w");
        if (!diag_f) std::fprintf(stderr, "⚠ 打不开诊断日志 %s\n", args.diag_log.c_str());
        else std::printf("诊断记录: %s（每帧一条 JSON）\n", args.diag_log.c_str());
    }

    // ---- 逐帧 ------------------------------------------------------------
    DecisionState st;
    const auto t0 = std::chrono::steady_clock::now();
    //: 进程启动的单调时刻，用来把"主机墙钟起点"外推到每一帧
    const auto t_proc0 = t0;
    if (args.host_epoch > 0.0) {
        std::printf("主机时基: %.3f（诊断日志里的 host_t 可直接与仿真时间对账）\n",
                    args.host_epoch);
    }
    int ok = 0, bad = 0;
    uint64_t stamp_ns = 0;
    double sum_read = 0, sum_push = 0, sum_wait = 0, sum_total = 0;
    long n_timed = 0;
    long n_auto_frames = 0;
    long n_struct_seen = 0;
    //: 判据 ②/③ 的统计：结构判据帧数、标记=4 的帧数（用于验收，不是调试输出）
    long n_struct_all = 0;
    long n_marker4 = 0;
    long n_model_frames = 0;   // 靠世界模型外推给出几何的帧数
    //: 标定窗口的起点：**第一帧处理完**那一刻（不是进程启动）——
    //: 相机起来到出第一帧要 1~3 s，用进程启动会把窗口浪费在还没画面的时间里，
    //: 于是仿真那侧已经"该放墙了"而板子还在发标定动作。以首帧为零点，
    //: 两边就只差仿真启动→板子首帧那一段已知延迟。
    bool cal_started = false;
    auto t_cal0 = t0;
    long n_cal_frames = 0;
    //: 标定计划（首帧时按视觉位置生成一次；之后只读）
    calib::Plan cal_plan;
    //: 场地不可见的帧数（`field_extent` 失败 = 相机没看到场地边框）。
    //: 闭环测试里这个数就是"这一轮算不算数"的判据：窗口被盖住时它会一路涨，
    //: 同时所有指令都是 stay —— 那是环境问题，不是判据问题，必须能一眼看出来。
    long n_no_field = 0;      // 走到结构判据的帧数（打印节流用，i 会因为热身帧不是 0）      // 用本帧统计推出阈值的帧数（--no-auto-thresholds 时为 0）
    // `--frames 0` = 不限帧数，一直跑到输入结束（实时采集用）
    const long want_frames = args.frames > 0 ? args.frames : 1000000000L;
    for (int i = 0; i < want_frames; ++i) {
        // ---- 取这一帧的像素（统一入口：PPM 或 stdin）------------------
        // 两条下游（PL 推像素 / PS 参考模型）**吃同一份 rgb** ——
        // 否则 --verify-both 比的就不是"两条像素通路"而是"两张图"。
        const auto t_frame0 = std::chrono::steady_clock::now();
        auto t_got = t_frame0;   // 像素真正交到手上的时刻（延迟分解用）
        size_t q_before = 0;
        std::vector<uint8_t> rgb;
        int pw = 0, ph = 0;
        if (pump) {
            std::string swhy;
            q_before = pump->queued();
            if (!pump->get(&rgb, &swhy)) {
                if (i > 0) break;
                std::fprintf(stderr, "第一帧就取不到像素：%s\n", swhy.c_str());
                ++bad; break;
            }
            pw = kPlWidth; ph = kPlHeight;
            // 用读线程打下的**真实采集时刻**当 stamp：固定步长会让速度系统性偏小
            // （相机实测 40~48 fps，按 30fps 打戳就低 28%~50%）。
            t_got = std::chrono::steady_clock::now();
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

        // ---- 去噪：推像素前做，PL 与结构判据都吃同一份干净缓冲 ---------------
        if (args.denoise_radius > 0 && !rgb.empty() && pw > 0 && ph > 0) {
            denoise::box_blur_rgb(rgb, pw, ph, args.denoise_radius);
        }

        // ---- 结构判据：**必须在推像素之前**算 -----------------------------
        // 踩过的坑：把这段放到 `decide` 那边（推像素之后）时 `rgb` 已经是空的
        // （像素被 push 消费掉了）→ 结构判据从来没跑过，日志里一行都没有。
        band_gap::Door sg_door;
        band_gap::Extent sg_ext;
        std::vector<band_gap::Door> sg_doors;
        band_gap::Player sg_player;
        int sg_min_gap_px = 0;
        rectify::Homography sg_h;
        size_t sg_marker_count = 0;
        //: 几何可信性判定失败的原因（"ok" = 通过）。直接进诊断日志，
        //: 这样"这一帧为什么 stay"在日志里是**可读的一句话**，不用再靠猜。
        const char* sg_geo_why = "none";
        //: 上一次"可信几何"成功确认距今多少 ms（诊断 / 门控用，见上面 geo_stale）
        double sg_geo_age_ms = 1e9;
        //: 新角色检测器（`player.hpp`）的完整结果，供诊断日志区分来源
        player::Player sg_player_new;
        //: 角色目标的不确定度（时间窗内两轴范围之和，像素）。越小越确定，进日志/门控用。
        double sg_player_unc = 0.0;
        //: 去重前的门候选个数（诊断：这个数远大于 `walls` 的长度就说明重复严重）
        size_t sg_doors_raw = 0;
        //: 结构判据内部的分段耗时（毫秒）。`decide` 只给出总时间，回答不了
        //: "到底哪一步慢" —— 实测总时间 57ms(p50)/125ms(p95)，必须拆开看。
        double ms_markers = 0, ms_doors = 0, ms_player = 0, ms_extent = 0;
        //: 本帧的墙（`wall_gap::find_walls` 的结果）。**在找角色之前就要有**：
        //: 墙的端头在形状上和角色圆盘不可分，只能靠"这个候选是不是落在墙带上"排除。
        std::vector<wall_gap::Wall> sg_walls;
        std::vector<std::pair<int, int>> sg_bands;
        if (args.structural_gap && !rgb.empty() && pw > 0 && ph > 0) {
            // 先用亮度边缘粗量（无标记时的回退）。如果下面用标记解出了单应，
            // 会被 `field_bbox()` 覆盖 —— 那时这次的 1.5 ms 全图扫描就白花了。
            // 真机上它大约占 15 ms 板子时间，先保留（逻辑简单、无标记的帧仍需要它）。
            // ⚠ P1 之后 `field_extent` 只是"参考值"（有效性由四标记单应判定），
            //   但它每帧仍要花 **9.6 ms**（p50，实测）。默认不再调用；
            //   需要对比时才 `--extent-fallback`。
            if (args.extent_fallback) {
                const auto t_ext0 = std::chrono::steady_clock::now();
                sg_ext = band_gap::field_extent(rgb.data(), pw, ph,
                                                args.field_w, args.field_h,
                                                5.0, 3, args.image_squash);
                ms_extent = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_ext0).count();
            }
            // ⚠ **P1：亮度台阶法不再有权单独判定"场地可见"**
            //   实测它在 268 帧上只有 45.9% 有效，且有效时 scale_x 恒为 1.187 ——
            //   它量的是显示器边框（黑边→屏幕的亮度台阶），不是场地；纵向尺度更是
            //   "按 squash 反推 + 场地居中"假设出来的，从来没被量过。
            //   所以这里只保留它的数值作为**兜底参考**，有效性一律由四标记单应 +
            //   可信性判定（geometry.hpp）给出：没有标记 → 没有几何 → INVALID → stay。
            sg_ext.ok = false;
            // 尺寸窗口与可通过性都由**契约 + 本帧比例尺**算出来，不是像素常数
            if (sg_ext.ok && sg_ext.scale_x > 0.0) {
                const double dia_px = 2.0 * args.player_radius / sg_ext.scale_x;
                sg_player = band_gap::find_player(
                    rgb.data(), pw, ph, 0,
                    static_cast<int>(0.5 * dia_px), static_cast<int>(2.0 * dia_px));
                sg_min_gap_px = static_cast<int>(1.6 * dia_px);
            } else {
                // **ROI 跟踪**：以上一帧的角色位置为中心开窗（±120 px），
                // 找不到再退回全图。角色是逐帧连续的小目标，开窗能省掉大部分扫描。
                static double last_px = -1, last_py = -1;
                const int R = 120;
                sg_player = last_px >= 0
                    ? band_gap::find_player(rgb.data(), pw, ph, 0, 0, 0,
                                            static_cast<int>(last_px) - R,
                                            static_cast<int>(last_py) - R,
                                            static_cast<int>(last_px) + R,
                                            static_cast<int>(last_py) + R)
                    : band_gap::find_player(rgb.data(), pw, ph);
                if (!sg_player.found) {
                    sg_player = band_gap::find_player(rgb.data(), pw, ph);
                }
                if (sg_player.found) { last_px = sg_player.x; last_py = sg_player.y; }
            // 四角标记 → 单应（吃掉旋转与透视），并给出场地四边形掩膜
            {
                // 每 5 帧重算一次标记/单应，其余帧复用（相机与平台相对静止）
                //
                // ⚠ 三处刻意的选择，都是被实测逼出来的：
                // 1) 用 `find_markers4`（深色纹理 + 亮块两个检测器合并）而不是
                //    `find_markers`：268 帧语料上四标记齐全率 84.7% vs **97.8%**。
                // 2) **自己按几何排序**再解单应，不依赖检测器的返回顺序。
                //    实测有 20/101 帧出现"标记离 bbox 角很远"，根子就是顺序假设
                //    （原来写死 img=[m0,m1,m3,m2] ↔ [TL,TR,BR,BL]）。
                // 3) 4 点单应对**任何**顺序都能精确解出来（重投影误差恒 0），
                //    所以顺序错了不会报错，只会给出一个歪掉的场地 —— 必须靠几何排序
                //    + 可信性判定（geometry.hpp）一起兜住。
                static std::vector<rectify::Marker> s_mk;
                static rectify::Homography s_homo;
                static long s_tick = 0;
                // `--geom=frame` 时**完全不找标记**：省掉每 5 帧一次 51 ms 的尖峰
                // （实测帧率 11.3 → 16.7 fps 主要就是这个），也不再需要标记排除。
                if (args.geom != "frame" && (s_tick % 5 == 0 || s_mk.size() != 4)) {
                    const auto t_mk0 = std::chrono::steady_clock::now();
                    std::vector<rectify::Marker> found =
                        rectify::find_markers4(rgb.data(), pw, ph);
                    if (found.size() == 4) {
                        // 按"和/差"排成 TL, TR, BR, BL：与成像方向无关，只要不是
                        // 45° 旋转就能唯一确定（45° 那种病态情形由可信性判定否掉）。
                        std::sort(found.begin(), found.end(),
                                  [](const rectify::Marker& a, const rectify::Marker& b) {
                                      return (a.x + a.y) < (b.x + b.y);
                                  });
                        rectify::Marker tl = found[0], br = found[3];
                        rectify::Marker o1 = found[1], o2 = found[2];
                        rectify::Marker tr = (o1.x >= o2.x) ? o1 : o2;   // 靠右的是 TR
                        rectify::Marker bl = (o1.x >= o2.x) ? o2 : o1;
                        // 标记中心精修：检测器给的是"亮内图"的质心，而契约坐标是
                        // **方块中心**；内图不对称会让单应整体平移 8~15 px（实测）。
                        // 精修到"包住内图的深色方块"的 bbox 中心后，角色位置误差中位
                        // 从 16.6 px 降到 8.7 px（同一批静态帧）。
                        bool r_tl = false, r_tr = false, r_br = false, r_bl = false;
                        // 契约：标记方块边长 48 场地单位 → 像素边长 ≈ 四标记跨度 × 48/640。
                        // 给精修一个**尺寸窗口**，否则它会吞掉画面边缘的大片暗区。
                        const double span_px =
                            0.5 * (std::hypot(found[1].x - found[0].x, found[1].y - found[0].y) +
                                   std::hypot(found[2].x - found[3].x, found[2].y - found[3].y));
                        markers::Params rp;
                        if (span_px > 40.0) {
                            const double expect = span_px * 28.0 / 640.0;   // 标记边长已缩到 28 场地单位
                            rp.min_side_px = static_cast<int>(0.5 * expect);
                            rp.max_side_px = static_cast<int>(2.0 * expect);
                        }
                        tl = markers::refine(rgb.data(), pw, ph, tl, &r_tl, rp);
                        tr = markers::refine(rgb.data(), pw, ph, tr, &r_tr, rp);
                        br = markers::refine(rgb.data(), pw, ph, br, &r_br, rp);
                        bl = markers::refine(rgb.data(), pw, ph, bl, &r_bl, rp);
                        // ⚠ 解单应时喂的顺序也必须是"世界口径"的 [左下,右下,右上,左上]。
                        //   只改 `s_mk`（judge 用）而忘了这里，会出现"判定通过、坐标仍然
                        //   上下镜像"的最坏情形：实测同一帧真值 (320,106) 被算成 (319,380)。
                        const std::array<std::array<double, 2>, 4> img = {
                            {{bl.x, bl.y}, {br.x, br.y}, {tr.x, tr.y}, {tl.x, tl.y}}};
                        const rectify::Homography h =
                            rectify::solve(img, rectify::contract_markers(args.field_w,
                                                                         args.field_h));
                        if (h.ok) {
                            // ⚠ 顺序必须与 `rectify::contract_markers()` 一致：
                            //   contract = [TL, TR, BR, BL]（注意 BR 在 BL 前面）。
                            //   写成 TL,TR,BL,BR 会让单应整体**上下镜像** —— 而镜像
                            //   不改变 bbox、不改变宽高比、也不改变"标记到角点距离"，
                            //   所以可信性判定照样通过，只有"真值投影到图像"时才暴露：
                            //   实测把角色真值投成了 y=-33.9（画面外）。
                            // **世界口径（y 朝上）**：contract 的 (32,32) 是世界左下，
                            // 对应图像左下标记 → 喂给 solve 的顺序是 [BL, BR, TR, TL]。
                            // 这样解出来的单应直接给出"y 朝上"的场地坐标，和 `@STATE`、
                            // `gap_avoid`（已验证）同一口径，纵向逻辑不再镜像。
                            s_mk = {bl, br, tr, tl};
                            s_homo = h;
                        } else {
                            s_mk.clear();
                        }
                    } else {
                        s_mk.clear();
                    }
                    ms_markers = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t_mk0).count();
                }
                ++s_tick;
                const std::vector<rectify::Marker>& ms = s_mk;
                sg_marker_count = ms.size();
                ++n_struct_all;
                if (ms.size() == 4) ++n_marker4;
                // 不管找到几个标记，都先把"候选角色贴着标记"的情况排掉
                // （标记只露出 2~3 个时也需要这条；否则会拿角标当角色）
                if (sg_player.found && !ms.empty()) {
                    for (const rectify::Marker& mk : ms) {
                        const double half = 2.0 * std::max(mk.w, mk.h);
                        if (std::hypot(sg_player.x - mk.x, sg_player.y - mk.y) < half) {
                            sg_player.found = false;
                            break;
                        }
                    }
                }
                // ---- 几何来源：白框优先、标记兜底（`--geom=frame|markers|auto`）----
                // 白框路径的四个角**就是**场地四角，不需要任何近似；标记路径保留做兜底，
                // 也用于对照（历史上 `--verify-both` 的对账都基于它）。
                geometry::Geometry geo;
                bool geo_ready = false;
                static rectify::Homography s_frame_homo;
                if (args.geom != "markers") {
                    const auto fr = frame_geom::find(rgb.data(), pw, ph);
                    if (fr.ok) {
                        const rectify::Homography fh =
                            frame_geom::homography(fr, args.field_w, args.field_h);
                        if (fh.ok) {
                            geo = frame_geom::as_geometry(fr, args.field_w, args.field_h,
                                                          geometry::Params{}, &fh);
                            if (geo.valid) { s_frame_homo = fh; geo_ready = true; }
                        }
                    }
                }
                if (!geo_ready && args.geom != "frame" && s_homo.ok) {
                    sg_h = s_homo;
                    geo = geometry::judge(sg_h, ms, pw, ph, args.field_w, args.field_h);
                    geo_ready = true;
                }
                if (!geo_ready) {
                    sg_geo_why = "no_geometry";
                } else {
                    if (geo.src == geometry::Source::Frame) sg_h = s_frame_homo;
                    sg_geo_why = geo.why;
                    // ⚠ 顺序不能反：**先算年龄，再更新**"最后一次成功确认"的时刻。
                    //   反过来的话，本帧刚把自己写成"刚刚成功"，年龄永远是 0/无穷，
                    //   实测表现为每一帧都被判 `geo_stale`（101/101 帧 INVALID）。
                    static uint64_t last_marker_ok_ns = 0;
                    sg_geo_age_ms = (last_marker_ok_ns == 0)
                        ? 0.0
                        : ((stamp_ns >= last_marker_ok_ns)
                               ? static_cast<double>(stamp_ns - last_marker_ok_ns) * 1e-6
                               : 1e9);
                    if (geo.valid) last_marker_ok_ns = stamp_ns;
                    if (geo.valid && sg_geo_age_ms > 600.0) {
                        sg_geo_why = "geo_stale";
                    }
                    if (geo.valid && sg_geo_age_ms <= 600.0) {
                        sg_ext.ok = true;
                        sg_ext.x0 = geo.bbox[0]; sg_ext.x1 = geo.bbox[1];
                        sg_ext.y0 = geo.bbox[2]; sg_ext.y1 = geo.bbox[3];
                        sg_ext.width_px = static_cast<double>(geo.bbox[1] - geo.bbox[0] + 1);
                        sg_ext.height_px = static_cast<double>(geo.bbox[3] - geo.bbox[2] + 1);
                        sg_ext.scale_x = geo.scale_x;
                        sg_ext.scale_y = geo.scale_y;
                        sg_ext.anisotropy = geo.scale_y > 0 ? geo.scale_x / geo.scale_y : 1.0;
                        // **P2：角色检测换成"场地 ROI 内找实心圆"**
                        // 旧 `band_gap::find_player` 是全图找最亮的圆，和四角标记形状重叠，
                        // 实测 PS 侧从没被采用过。新检测器（`player.hpp`）用：
                        //   契约面积（由 10 场地单位半径换算）+ 形状 + 离标记的距离
                        // 并且只在场地 ROI 里找。找到就覆盖旧结果；找不到就保留旧结果
                        // （旧的仍然可能对，只是不可靠，日志里 player_src 会区分）。
                        const auto t_pl0 = std::chrono::steady_clock::now();
                        // 先算墙（同一套检测器、同一帧），给角色检测提供"墙带"约束
                        // 场地掩膜：只在几何给出的场地 bbox 内找墙/缺口，
                        // 场外黑区不许当缺口（否则缺口中心被拉到画面边缘，
                        // 角色会被带出平台）。bbox 来自四标记单应 + 可信性判定。
                        // 裁剪边界 = 场地内沿（收缩 4 px，避开白框线自身）。
                        // 几何这一帧无效时**沿用上一次有效的 bbox**：否则"刚出现墙"
                        // 的那几帧没有裁剪，绿/红框就跑到平台外的黑区去了（实测现象）。
                        static band_gap::Extent s_clip{};
                        if (sg_h.ok && sg_ext.ok) s_clip = sg_ext;
                        wall_gap::Params wp;
                        if (s_clip.ok) {
                            const int inset = 4;
                            wp.clip_x0 = s_clip.x0 + inset;
                            wp.clip_x1 = s_clip.x1 - inset;
                            wp.clip_y0 = s_clip.y0 + inset;
                            wp.clip_y1 = s_clip.y1 - inset;
                        }
                        sg_walls = wall_gap::find_walls(rgb.data(), pw, ph, wp);
                        sg_bands.clear();
                        for (const wall_gap::Wall& w2 : sg_walls)
                            sg_bands.emplace_back(w2.y0, w2.y1);
                        // 缺口的列范围：角色穿缺口时它的行号会落在墙带里，
                        // 必须放行（否则最需要角色的时刻恰好丢帧）。
                        std::vector<std::array<int, 4>> gap_windows;
                        for (const wall_gap::Wall& w2 : sg_walls)
                            if (w2.has_gap) gap_windows.push_back({w2.y0, w2.y1, w2.gap_x0, w2.gap_x1});
                        const auto pl = player::find(rgb.data(), pw, ph, sg_h, ms,
                                                     args.field_w, args.field_h,
                                                     args.player_radius, player::Params{},
                                                     &sg_bands, &gap_windows);
                        ms_player = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_pl0).count();
                        sg_player_new = pl;
                        // ⚠ **角色不可能落在四角标记上**：按世界坐标再排一次。
                        //   实测坑：空场地（没有墙）时 PS 检测器的"角色"稳定落在图像
                        //   (0..31, 0..31)，也就是左上角那个标定标记 —— 于是
                        //   `player_field` 跑到 x=186..551 的乱值，标定窗口的
                        //   "初始位置"直接失效。像素级的标记排除（`marker_exclude_k`
                        //   × 检出尺寸）不够（标记的内亮块本身就小），而**世界坐标**
                        //   是准的：直接按合同角点排最可靠。
                        if (sg_player_new.found && sg_h.ok) {
                            const auto q = sg_h.apply(sg_player_new.x, sg_player_new.y);
                            // 标记半边长 24 + 角色半径 10 + 余量 36 → 70 场地单位
                            const double keep_out = 24.0 + args.player_radius + 36.0;
                            const double corners[4][2] = {
                                {32.0, 32.0},
                                {args.field_w - 32.0, 32.0},
                                {32.0, args.field_h - 32.0},
                                {args.field_w - 32.0, args.field_h - 32.0}};
                            for (const auto& c : corners) {
                                if (std::hypot(q[0] - c[0], q[1] - c[1]) < keep_out) {
                                    sg_player_new.found = false;
                                    break;
                                }
                            }
                        }
                        // 全局没找到时，用**上一帧**位置做局部精修：角色贴墙时
                        // 会被并进墙的大块里（面积判据直接否掉），但"盘内亮、环带暗"
                        // 的对比度仍然成立。只在上一次位置可信时做，避免满场乱找。
                        static player::Player s_prev_player;
                        if (!sg_player_new.found && s_prev_player.found && !args.no_player_pred &&
                            args.local_refine) {
                            const double unit_per_px =
                                args.field_w / std::max(1.0, static_cast<double>(
                                    geo.bbox[1] - geo.bbox[0]));
                            const double r_px = args.player_radius / std::max(1e-6, unit_per_px);
                            auto loc = player::refine_near(rgb.data(), pw, ph,
                                                           s_prev_player.x,
                                                           s_prev_player.y, r_px,
                                                           45.0, 12.0);
                            bool near_mk = false;
                            for (const rectify::Marker& m : ms) {
                                if (std::hypot(loc.x - m.x, loc.y - m.y) <
                                    2.0 * std::max(m.w, m.h)) { near_mk = true; break; }
                            }
                            // **漂移闸**：局部精修以**上一帧**为种子，而角色周围是平坦
                            // 的场地灰 —— 对比度曲面在那里几乎是平的，argmax 每帧挪一两个
                            // 像素，会**自我强化地越走越偏**（实测平台静止不动时，板子的
                            // "角色"在 0.9 s 里从 x=324 漂到 199，而真值一直是 320）。
                            // 所以只允许"一帧能走到的距离"（200 场地单位/秒 × 60ms ≈ 12 px，
                            // 留余量取 14 px），超过就当它没找到，改用预测/全局结果。
                            const double drift_px = std::hypot(loc.x - s_prev_player.x,
                                                               loc.y - s_prev_player.y);
                            if (loc.found && drift_px > 14.0) loc.found = false;
                            if (loc.found && !near_mk) {
                                sg_player_new.found = true;
                                sg_player_new.x = loc.x;
                                sg_player_new.y = loc.y;
                                sg_player_new.why = "local";
                                sg_player_new.conf = loc.contrast / 100.0;
                            }
                        }
                        // **时间窗鲁棒化**（用户口径：宁可要一个大概的范围，也不要一个
                        // 不稳定的点值）。实测：单帧角色误差 p50 35 / p95 159 场地单位，
                        // 而决策要的是稳定目标 —— 目标抖就变成指令抖（缺口跳变 >20 单位
                        // 的帧里指令翻转率 29%，平稳帧 8%）。
                        // 做法：对**像素坐标**做 0.35 s 的截尾均值 + MAD 离群剔除。
                        // 用 MAD 而不是"均值±3σ"，因为均值本身会被错块带跑。
                        {
                            static robust::Window2 s_win_player(0.35, 10);
                            const double t_s = static_cast<double>(stamp_ns) * 1e-9;
                            if (sg_player_new.found) {
                                s_win_player.push(sg_player_new.x, sg_player_new.y, t_s);
                                sg_player_unc = s_win_player.uncertainty();
                                if (s_win_player.enough(3)) {
                                    sg_player_new.x = s_win_player.cx();
                                    sg_player_new.y = s_win_player.cy();
                                }
                            }
                        }

                        // **预测（coasting）**：角色被墙盖住时，相机确实看不见它
                        // （实测动态跑里 42% 的帧没有角色）。物理上"看不见"不等于
                        // "不存在"——平台只需要知道自己大概在哪、往哪走。所以用
                        // 最后几次有效观测估速度，短时间外推（≤300 ms），并把来源
                        // 标成 `ps_pred` 以便审计；超过 300 ms 一律不再外推（stay）。
                        struct PlayerPred {
                            double x = 0, y = 0, vx = 0, vy = 0;
                            uint64_t t_ns = 0;
                            bool have = false;
                        };
                        static PlayerPred s_pred;
                        if (sg_player_new.found && sg_player_new.why != std::string("pred")) {
                            if (s_pred.have && stamp_ns > s_pred.t_ns) {
                                const double dt = static_cast<double>(stamp_ns - s_pred.t_ns) * 1e-9;
                                if (dt > 0.001 && dt < 0.6) {
                                    const double nvx = (sg_player_new.x - s_pred.x) / dt;
                                    const double nvy = (sg_player_new.y - s_pred.y) / dt;
                                    s_pred.vx = 0.5 * s_pred.vx + 0.5 * nvx;
                                    s_pred.vy = 0.5 * s_pred.vy + 0.5 * nvy;
                                }
                            }
                            s_pred.x = sg_player_new.x;
                            s_pred.y = sg_player_new.y;
                            s_pred.t_ns = stamp_ns;
                            s_pred.have = true;
                            s_prev_player = sg_player_new;
                            // ⚠ 试过对角色位置做 α-β 轨迹滤波（+离群闸）：实测
                            //   ≤20 场地单位的比例反而从 19% 降到 14% —— 说明这点抖动
                            //   不是"白噪声"，而是**偶发的错块**（跳到墙/网格线上）。
                            //   滤波对偶发错块无效，只会把正确值一起拖慢，故不保留。
                            //   真正的解法是"错块识别"（形状/上下文），见文档 §7 的下一步。
                            s_prev_player = sg_player_new;
                        } else if (!args.no_player_pred && s_pred.have && stamp_ns > s_pred.t_ns) {
                            const double age = static_cast<double>(stamp_ns - s_pred.t_ns) * 1e-9;
                            if (age <= 0.30) {
                                sg_player_new.found = true;
                                sg_player_new.why = "pred";
                                sg_player_new.x = s_pred.x + s_pred.vx * age;
                                sg_player_new.y = s_pred.y + s_pred.vy * age;
                                sg_player_new.conf = 1.0 - age / 0.30;
                            }
                        }
                        // ⚠ **决策用的 `sg_player` 必须吃下全部三个来源**（全局/局部/预测）。
                        //   之前只写了全局结果 `pl`，局部精修与预测只落在 `sg_player_new`
                        //   （诊断用）→ "角色被墙盖住时靠预测继续走"根本没进决策。
                        if (sg_player_new.found) {
                            sg_player.found = true;
                            sg_player.x = sg_player_new.x;
                            sg_player.y = sg_player_new.y;
                            sg_player.w = sg_player_new.w;
                            sg_player.h = sg_player_new.h;
                            sg_player.fill = sg_player_new.fill;
                        }
                    } else {
                        sg_ext.ok = false;
                    }
                }
            }
            }   // 收掉"无比例尺时的 ROI 角色跟踪"那个 else 分支
            // **性能**：`find_all` 会在 ±10° 里搜 25 个角度（每个角度都要重扫整幅图），
            // 实测是整帧耗时的大头（板子上 ~390 ms/帧）。有了四角标记的单应之后，
            // 旋转/透视都在**坐标变换**里被吃掉了，行带检测不需要再搜角度。
            // 关键细节：判据用"**曾经**有过单应"而不是"本帧有单应" —— 标记偶尔只露出
            // 2~3 个，若这类帧回退到 25 角度搜索，帧率会周期性掉到 1 帧/秒。
            static bool have_h_ever = false;
            if (sg_h.ok) have_h_ever = true;
            const auto t_door0 = std::chrono::steady_clock::now();
            if (!args.legacy_gap) {
                // **已验证的检测器**：`wall_gap::find_walls`。
                // 它的做法是"按行找亮像素多的行 → 聚成行带 → 行带内按列统计亮行数 →
                // 连续不亮的内部区间就是缺口"，每道墙**只给一扇门**。
                // 实测（静态语料）缺口中心误差 **1 px**、0.6 ms/帧；而旧的
                // `band_gap::find_at` 每条墙带吐 27~153 个候选、90% snr=0，
                // 靠启发式挑一个 —— 实测碰撞前一刻缺口误差中位 **147 场地单位**。
                wall_gap::Params wp2;
                wp2.min_gap_px = std::min(sg_min_gap_px > 0 ? sg_min_gap_px : 8, 24);
                // 复用角色检测那一步已经算好的墙（省一次 ~0.5 ms，也保证两者一致）
                const std::vector<wall_gap::Wall>& walls =
                    sg_walls.empty() ? (sg_walls = wall_gap::find_walls(rgb.data(), pw, ph, wp2))
                                     : sg_walls;
                sg_doors.clear();
                for (const wall_gap::Wall& w : walls) {
                    if (!w.has_gap) continue;
                    band_gap::Door d;
                    d.found = true;
                    d.y0 = w.y0; d.y1 = w.y1;
                    d.x0 = w.gap_x0; d.x1 = w.gap_x1;
                    d.cx = static_cast<int>(std::lround(w.gap_cx));
                    d.width = w.gap_w;
                    d.support = static_cast<float>(w.bright_frac);   // 诊断用
                    d.snr = static_cast<float>(w.bright_frac * 10.0);
                    // 可通过性：用契约算出来的最小缺口宽度（`sg_min_gap_px`）过滤，
                    // 与旧路径同一口径，避免"看着有缺口但其实过不去"。
                    if (sg_min_gap_px > 0 && d.width < sg_min_gap_px) continue;
                    sg_doors.push_back(d);
                }
            } else if (sg_h.ok || have_h_ever) {
                std::vector<band_gap::Door> all;
                band_gap::find_at(rgb.data(), pw, ph, 0.0, 6, 40, 6, 5.0f, &all,
                                  sg_min_gap_px);
                sg_doors = all;
            } else {
                sg_doors = band_gap::find_all(rgb.data(), pw, ph, 6, 40, 6, 5.0f, 10.0f, 2.5f,
                                              sg_min_gap_px);
            }
            ms_doors = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_door0).count();
            // **门候选去重**：`find_at` 会把每条墙带上的所有可能缺口都当候选输出
            // （实测 1 帧 94 个），而 `decide` 会把每一扇都注入规划器 —— 同一道墙被叠
            // 十几遍，规划器看到的墙"糊成一片"。这里按"行带重叠 + 缺口中心接近"合并。
            const size_t n_raw_doors = sg_doors.size();
            if (!sg_doors.empty()) sg_doors = wall_gap::dedup_doors(sg_doors);
            sg_doors_raw = n_raw_doors;

            // 场地范围要在**去剪切**的坐标系里量：先由角度搜索定角，再用同一个角
            // 重新量一次边界。顺序不能反（先量框会把旋转的信息烧掉，实测偏 150 单位）。
            {
                float best_ang = 0.0f;
                int best_w = -1;
                for (const band_gap::Door& d : sg_doors) {
                    if (d.width > best_w) { best_w = d.width; best_ang = d.angle_deg; }
                }
                if (best_ang != 0.0f) {
                    const double ta = std::tan(static_cast<double>(best_ang) * 3.14159265358979 / 180.0);
                    sg_ext = band_gap::field_extent(rgb.data(), pw, ph, args.field_w,
                                                    args.field_h, 5.0, 3,
                                                    args.image_squash, ta);
                    // 同 P1：这里只是"按角度再量一次"的参考值，没有资格单独判定有效
                    sg_ext.ok = false;
                }
            }

            // ---- 世界模型：有检出就更新，没检出就按速度外推 ------------------
            // 只在**场地可见**时做（看不见场地由 `field_extent` 的显著性门判掉，
            // 那一帧在 `decide()` 里直接 stay，不进入这里的外推）。
            static uint64_t prev_ns = 0;
            double mdt = 0.0;
            if (prev_ns != 0 && stamp_ns > prev_ns) mdt = (stamp_ns - prev_ns) * 1e-9;
            prev_ns = stamp_ns;
            std::vector<std::array<double, 4>> dets;
            for (const band_gap::Door& d : sg_doors) {
                dets.push_back({0.5 * (d.x0 + d.x1), 0.5 * (d.y0 + d.y1),
                                static_cast<double>(d.width),
                                0.5 * (static_cast<double>(d.y1 - d.y0) + 1.0)});
            }
            if (!dets.empty()) {
                st.world_model.update(dets, mdt);
            } else if (st.world_model.has) {
                const std::vector<BandTrack> pred = st.world_model.predict(mdt);
                sg_doors.clear();
                for (const BandTrack& q : pred) {
                    band_gap::Door d;
                    d.found = true;
                    d.width = static_cast<int>(std::lround(q.w));
                    const int half = static_cast<int>(std::lround(q.hh));
                    d.x0 = static_cast<int>(std::lround(q.cx - 0.5 * q.w));
                    d.x1 = static_cast<int>(std::lround(q.cx + 0.5 * q.w));
                    d.y0 = static_cast<int>(std::lround(q.y - half));
                    d.y1 = static_cast<int>(std::lround(q.y + half));
                    d.cx = static_cast<int>(std::lround(q.cx));
                    d.support = 1.0f;
                    sg_doors.push_back(d);
                }
                if (!sg_doors.empty()) ++n_model_frames;
            }
            // **最终去重 + 上限**（必须在世界模型之后再做一次）：
            // 上面那次去重只作用于"本帧检测到的"候选；世界模型外推会把**每条轨迹**
            // 都吐成一扇门，轨迹测多了就会一次注入几十扇（实测 p50 24、最大 53，
            // 每一扇都被当成一段墙塞给规划器 → 墙糊成一片）。
            // 这里按"行带 + 缺口中心"再去重一次，并限制注入数量（按缺口宽度排序取前 N）。
            {
                const size_t before = sg_doors.size();
                // 先把"同一道墙上的多条候选"收敛成一扇门（契约：一道墙只有一个缺口），
                // 再按 (行带, 缺口中心) 去重，最后限上限。
                // 上一帧选中的缺口中心（像素）：用来在同一条墙带的多个候选里
                // 保持跨帧一致（见 `one_gap_per_band` 的注释）。
                static double s_prev_gap_cx = -1e9;
                if (!sg_doors.empty()) sg_doors = wall_gap::one_gap_per_band(sg_doors, s_prev_gap_cx);
                if (!sg_doors.empty()) s_prev_gap_cx = sg_doors.front().cx;
                if (!sg_doors.empty()) sg_doors = wall_gap::dedup_doors(sg_doors);
                if (sg_doors.size() > 6) {
                    std::sort(sg_doors.begin(), sg_doors.end(),
                              [](const band_gap::Door& a, const band_gap::Door& b) {
                                  return a.width > b.width;
                              });
                    sg_doors.resize(6);
                }
                sg_doors_raw = before;
            }
            // 日志/回退用：仍然报"最宽的那扇门"
            for (const band_gap::Door& d : sg_doors)
                if (!sg_door.found || d.width > sg_door.width) sg_door = d;
        }

        // ---- 每帧自动阈值：用**这一帧自己的统计**推 7 个分类阈值 -----------
        // 必须在推像素**之前**写寄存器：PL 的逐像素比较器读的就是这 21 个数。
        // 统计量全部来自本帧（d=r-b 的中位数/MAD + 实测噪声地板），不是"这台相机"的常数。
        if (args.auto_thresholds && !rgb.empty() && pw > 0 && ph > 0) {
            const auto_params::Stats st_a = auto_params::measure(rgb, pw, ph);
            if (st_a.ok) {
                const auto_params::Derived dp = auto_params::derive(st_a);
                const VisionParams pp =
                    apply_auto_thresholds(args, effective_vision_params(args), dp);
                if (drv) drv->configure(pp);      // --via-pl：写 21 个寄存器
                if (ref) ref->set_vision_params(pp);   // PS 参考模型：同一套阈值
                ++n_auto_frames;
                if (args.verbose && (n_auto_frames <= 3 || n_auto_frames % 50 == 0)) {
                    std::printf("  [自动阈值] 帧 %ld: t_l=%d t_d=%d 亮像素=%.1f%% "
                                "→ wall_rb=%d wall_min_r=%d wall_min_g=%d bright_min=%d\n",
                                (long)i, dp.t_l, dp.t_d, 100.0f * dp.bright_frac,
                                pp.wall_rb, pp.wall_min_r, pp.wall_min_g, pp.bright_min);
                }
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
        } else if (drv && (src || pump) && !args.no_pl) {
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
            // `--no-pl` 时没有几何帧：合成一个"只有时间戳"的空帧。
            // 下游 `decide` 只用它的 `stamp_ns`（dt）和 `player`（RTL 角色），
            // 后者在没有 PL 时本来就没有 —— 于是 PS 侧几何成为唯一来源。
            GeometryFrame frame;
            if (!bytes.empty()) {
                frame = parse_geometry_frame(bytes);
            } else {
                frame.header.stamp_ns = stamp_ns;
                frame.player.present = false;
            }
            const FrameHeader& h = frame.header;
            ++ok;
            std::printf("帧 %4d: seq=%u  长度=%zu B  障碍=%u  缺口=%u  "
                        "尺寸=%ux%u  stamp=%llu ns%s\n",
                        i, h.seq, bytes.size(), h.n_obs, h.n_gaps,
                        h.img_w, h.img_h,
                        static_cast<unsigned long long>(h.stamp_ns),
                        h.flags & 1u ? "  [player_valid]" : "");
            if (args.structural_gap) {
                ++n_struct_seen;
                if (!sg_ext.ok) {
                    ++n_no_field;
                    if (n_struct_seen <= 5) {
                        std::fprintf(stderr,
                            "  ⚠ 第 %ld 帧看不到场地边框（field_extent 失败）："
                            "相机可能没对准平台，或仿真窗口被挡住\n", n_struct_seen);
                    }
                }
                if (args.verbose && (n_struct_seen <= 3 || n_struct_seen % 50 == 0)) {
                    std::printf("  [结构] 标记=%zu 角色=%d(%.0f,%.0f) 门=%d "
                                "带=%.0f..%.0f 缺口=%.0f..%.0f(宽%.0f) "
                                "支撑=%.2f 比例尺=%.3f 场地=%.0fpx h=%d\n",
                                sg_marker_count,
                                sg_player.found ? 1 : 0, sg_player.x, sg_player.y,
                                sg_door.found ? 1 : 0, (double)sg_door.y0, (double)sg_door.y1,
                                (double)sg_door.x0, (double)sg_door.x1, (double)sg_door.width,
                                sg_door.support, sg_ext.scale_x, sg_ext.width_px,
                                sg_h.ok ? 1 : 0);
                }
            }
            FrameLat lat;
            // 只有实时采集（stdin 读线程）才有"真实的取帧时刻"；PPM 回放那条路上
            // stamp 是合成的，量出来的差值没有意义，所以置 0 让打印自己跳过。
            lat.got_ns = pump
                ? static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      t_got.time_since_epoch()).count())
                : 0;
            lat.queued = q_before;
            lat.dropped = pump ? pump->dropped() : 0L;
            const gap_avoid::Vec2 cmd =
                decide(frame, args, st, &sg_doors, &sg_ext, &sg_player, &sg_h, lat);
            int action = vision_board::action_from_velocity(
                cmd.x, cmd.y, args.action_deadzone);
            // ---- 开场标定：窗口内旁路决策，发已知动作序列 --------------------
            // `cal_t` 是**标定窗口内的秒数**（-1 = 不在标定窗口）。它进诊断日志，
            // 于是分析脚本能直接按"指令段"切样本：每段位移/时间 = 速度，
            // 段起点到位移起点的滞后 = 指令延迟。
            double cal_t = -1.0;
            int cal_step = -1;
            if (args.cal_seconds > 0.0) {
                // **等角色被看见再开始标定**：计划要不要回中、往哪个方向回，
                // 取决于角色现在在哪。用"第一帧"当起点会踩坑：第一帧往往还没有
                // 可信几何（标记/单应要几帧才稳），于是拿不到位置 → 不回中 →
                // 出生点靠下时"向下"那一段直接撞边界，量出来的速度就是假的
                // （实测出现过初始位置 (0,0)、速度 119 单位/秒、段间差 148）。
                // 所以：等到"四标记 + 角色"都有，或者等够 5 s（相机被挡的兜底）。
                if (!cal_started) {
                    bool have_pos = sg_player.found && sg_h.ok;
                    const double waited = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    if (!have_pos && waited < 5.0) {
                        // 还没就绪：这一帧什么都不发（保持 stay），窗口也还没开始计时
                        action = 0;
                        if (args.echo_action) std::printf("        动作: 0 (标定等待几何)\n");
                        kb.emit(action, args.verbose);
                        continue;
                    }
                    cal_started = true;
                    t_cal0 = std::chrono::steady_clock::now();
                    double px_u = 0.0, py_u = 0.0;
                    if (have_pos) {
                        const auto q = sg_h.apply(sg_player.x, sg_player.y);
                        px_u = q[0];
                        py_u = q[1];
                    }
                    cal_plan = calib::build(px_u, py_u, have_pos, args.field_w, args.field_h);
                    std::printf("标定窗口: %.1f s（决策旁路；前缀 %.1f s"
                                "（静止采样 + 回中），脉冲一轮 %.1f s）\n",
                                args.cal_seconds, cal_plan.prefix_s, cal_plan.pulse_s);
                    if (!have_pos) {
                        std::printf("  ⚠ 首帧没看到角色：跳过回中，只跑八个方向的脉冲"
                                    "（若出生点靠边，向下/向右那几段可能撞边界）\n");
                    }
                }
                cal_t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_cal0).count();
                if (cal_t < args.cal_seconds) {
                    const calib::Point cp = calib::at(cal_plan, cal_t);
                    action = cp.action;
                    cal_step = cp.step;
                    ++n_cal_frames;
                } else {
                    cal_t = -1.0;   // 窗口结束：后面全是决策层，日志里一眼能分开
                }
            }
            if (args.echo_action) {
                std::printf("        动作: %d (%s)%s\n", action,
                            vision_board::action_name(action),
                            cal_step >= 0 ? "  [标定]" : "");
            }
            kb.emit(action, args.verbose);
            if (diag_f) {
                // 每帧一条：字段固定，方便直接喂给对账脚本（见 docs §4）
                const uint64_t t_now_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                // **共享时基**：板子和仿真机都用 CLOCK_REALTIME（NTP 同源），
                // 记下时间戳就能直接对齐，不必再用"速度序列互相关"这种间接办法
                // （实测位置对齐残差 197 场地单位、速度对齐在长段恒定动作上会给出
                //   多个几乎同样好的偏移）。
                const double host_t = (args.host_epoch > 0.0)
                    ? args.host_epoch + std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - t_proc0).count()
                    : 0.0;
                // **帧交付时刻**（像素真正交到手上的那一刻）的墙钟。
                // 为什么单独记：`host_t` 是"处理完"的时刻，比交付时刻晚一个处理周期
                // （实测 ~65 ms）。在线平台以 200 场地单位/秒往复运动，这 65 ms 就是
                // 13 单位的假误差，而延迟抖动会把它放大到几十单位 —— 这正是在线
                // 数字比离线（按交付时刻配对）差一大截的原因。
                const double host_t_got = (args.host_epoch > 0.0)
                    ? args.host_epoch + std::chrono::duration<double>(t_got - t_proc0).count()
                    : 0.0;
                const auto unix_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
                const double lat_age_ms =
                    (lat.got_ns > frame.header.stamp_ns)
                        ? (lat.got_ns - frame.header.stamp_ns) * 1e-6 : 0.0;
                const double lat_proc_ms =
                    (lat.got_ns != 0 && t_now_ns > lat.got_ns)
                        ? (t_now_ns - lat.got_ns) * 1e-6 : 0.0;
                // 分段耗时：读像素（含等采集）/ 推 PL / 等流水线 —— 和 `--timing` 同一口径，
                // 但每帧都落盘，才能算 p50/p95/p99 而不是只有平均值。
                const auto t_frame0_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        t_frame0.time_since_epoch()).count());
                const double ms_read = (lat.got_ns > t_frame0_ns)
                    ? (lat.got_ns - t_frame0_ns) * 1e-6 : 0.0;
                const auto tm_now = drv ? drv->last_timing() : VisionDriver::FrameTiming{};
                // **场地单位**的角色/缺口坐标（诊断专用）。
                // 为什么必须单独记：`player_xy` 是图像像素，`truth.jsonl` 是场地单位，
                // 两者不能直接对账（第一次做对齐时残差 449 单位，等于没对齐）。
                double player_fx = 0, player_fy = 0;
                bool have_pf = false;
                if (sg_player.found && sg_h.ok) {
                    const auto q = sg_h.apply(sg_player.x, sg_player.y);
                    player_fx = q[0];
                    player_fy = q[1];
                    have_pf = true;
                }
                double gap_units = 0, wall_y_units = 0;
                // ⚠ 必须用**决策实际选中的那扇门**，不能用 `front()`。
                //   `sg_doors` 是按 y0 升序排的，front 是**画面最上面**那道墙（也就是
                //   场地里**最远**的那道），而决策选的是"角色前方最近"的那道。
                //   用 front 记日志会让对账把"板子瞄的缺口"和"真值里另一道墙的缺口"
                //   比较 —— 实测因此得出"缺口误差 230~523 单位"的假结论。
                const band_gap::Door* dsel = (!sg_doors.empty() && sg_h.ok)
                    ? ((st.selected_door >= 0 && st.selected_door < static_cast<int>(sg_doors.size()))
                           ? &sg_doors[static_cast<size_t>(st.selected_door)]
                           : &sg_doors.front())
                    : nullptr;
                if (dsel != nullptr) {
                    const band_gap::Door& d0 = *dsel;
                    const auto q0 = sg_h.apply(0.5 * (d0.x0 + d0.x1),
                                               0.5 * (d0.y0 + d0.y1));
                    gap_units = q0[0];
                    wall_y_units = q0[1];
                }
                const char* src = sg_ext.ok ? "markers" : (sg_h.ok ? "rejected" : "none");
                // 角色来源必须分开记：PS 结构检测（`find_player`）和 RTL 几何帧里的
                // 角色**是两条路**，一个坏了另一个可能还在用。混在一起会让诊断日志说
                // "no_player"而板子其实正在按 RTL 的角色走 —— 实测就踩过（71/101 帧
                // 动作非 0，却被标成"没角色"）。
                const bool ps_player = sg_player.found && !sg_doors.empty();
                const bool rtl_player = frame.player.present;
                const char* player_src = sg_player_new.found
                    ? (std::string(sg_player_new.why) == "pred" ? "ps_pred" : "ps_new")
                    : (ps_player ? "ps_old" : (rtl_player ? "rtl" : "none"));
                const char* stop_reason = !sg_ext.ok ? "no_field"
                    : (sg_doors.empty() ? "no_wall"
                       : ((!ps_player && !rtl_player) ? "no_player" : "none"));
                std::fprintf(diag_f,
                    "{\"frame\":%d,\"t_ns\":%llu,\"unix_ms\":%llu,\"host_t\":%.3f,\"host_t_got\":%.3f,"
                    "\"frame_age_ms\":%.1f,"
                    "\"field_valid\":%s,\"field_src\":\"%s\",\"geo_why\":\"%s\","
                    "\"geo_age_ms\":%.1f,\"markers\":%zu,\"walls_raw\":%zu,"
                    "\"scale\":[%.4f,%.4f],\"player_valid\":%s,\"player_src\":\"%s\","
                    "\"player_xy\":[%.1f,%.1f],"
                    "\"player_px\":[%d,%d],\"player_field\":[%.1f,%.1f],"
                    "\"player_field_valid\":%s,\"player_unc\":%.1f,\"gap_units\":%.1f,\"wall_y_units\":%.1f,\"tier\":\"%s\",\"walls\":[",
                    i, static_cast<unsigned long long>(frame.header.stamp_ns),
                    static_cast<unsigned long long>(unix_ns / 1000000ull), host_t,
                    host_t_got, lat_age_ms,
                    sg_ext.ok ? "true" : "false", src, sg_geo_why, sg_geo_age_ms,
                    sg_marker_count, sg_doors_raw,
                    sg_ext.scale_x, sg_ext.scale_y,
                    sg_player.found ? "true" : "false", player_src, sg_player.x, sg_player.y,
                    sg_player.w, sg_player.h, player_fx, player_fy,
                    have_pf ? "true" : "false", sg_player_unc, gap_units, wall_y_units, st.last_tier);
                for (size_t wi = 0; wi < sg_doors.size(); ++wi) {
                    const band_gap::Door& d = sg_doors[wi];
                    std::fprintf(diag_f,
                                 "%s{\"y0\":%d,\"y1\":%d,\"gap_x0\":%d,\"gap_x1\":%d,"
                                 "\"gap_cx\":%d,\"gap_w\":%d,\"snr\":%.1f}",
                                 wi ? "," : "", d.y0, d.y1, d.x0, d.x1, d.cx, d.width,
                                 static_cast<double>(d.snr));
                }
                std::fprintf(diag_f,
                    "],\"selected_wall\":%d,\"gap_center\":%d,"
                    "\"select_reason\":\"%s\",\"decision\":[%.2f,%.2f],\"action\":%d,"
                    "\"cal\":%.3f,\"cal_step\":%d,"
                    "\"stop_reason\":\"%s\","
                    "\"ms\":{\"read\":%.1f,\"push\":%.1f,\"wait\":%.1f,"
                    "\"decide\":%.1f,\"markers\":%.1f,\"doors\":%.1f,\"player\":%.1f,"
                    "\"extent\":%.1f}}\n",
                    st.selected_door, sg_doors.empty() ? 0 : sg_doors.front().cx,
                    st.select_reason, cmd.x, cmd.y, action, cal_t, cal_step, stop_reason,
                    ms_read, tm_now.push_ms, tm_now.wait_ms, lat_proc_ms,
                    ms_markers, ms_doors, ms_player, ms_extent);
                std::fflush(diag_f);
            }
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

    if (diag_f) { std::fclose(diag_f); diag_f = nullptr; }
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
        if (st.n_edge_limited > 0) {
            std::printf("其中 %d 帧被边界余量限速（朝外分量被削，切向保留）\n",
                        st.n_edge_limited);
        }
        if (n_struct_seen > 0) {
            std::printf("结构判据：%ld 帧里 %ld 帧看不到场地边框（%.0f%%）\n",
                        n_struct_seen, n_no_field,
                        n_struct_seen > 0 ? 100.0 * n_no_field / n_struct_seen : 0.0);
            if (n_no_field * 2 > n_struct_seen) {
                std::printf("  ⚠ 一大半帧看不到场地 —— 这一轮**不能**用来判判据好坏"
                            "（先确认仿真窗口全屏在最前、相机对着它）\n");
            }
        }
        if (n_model_frames > 0) {
            std::printf("其中 %ld 帧的几何来自**世界模型外推**（本帧没检出，按位置+速度推）\n",
                        n_model_frames);
        }
        if (st.n_clamped > 0) {
            std::printf("其中 %d 帧角色被**夹回场地内**（换算把它算到了场外）\n", st.n_clamped);
        }
        if (n_struct_all > 0) {
            std::printf("标记=4 的帧：%ld / %ld = %.0f%%\n",
                        n_marker4, n_struct_all,
                        n_struct_all ? 100.0 * n_marker4 / n_struct_all : 0.0);
        }
        if (st.n_no_field_frames > 0) {
            std::printf("其中 %d 帧**看不到场地**（边框显著性不足）→ 一律 stay\n",
                        st.n_no_field_frames);
        }
        if (st.n_structural > 0) {
            std::printf("其中 %d 帧用**结构判据**重建几何（门 → 左右两段墙，场地单位）\n",
                        st.n_structural);
        }
        if (n_auto_frames > 0) {
            std::printf("其中 %ld 帧用**本帧统计**推出分类阈值（自动阈值）\n", n_auto_frames);
        }
        if (st.n_hold_cmd > 0) {
            std::printf("其中 %d 帧没检出障碍、沿用了上一条有效指令（漏检≠该停）\n",
                        st.n_hold_cmd);
        }
        if (st.n_merged_rects > 0) {
            std::printf("其中矩形层碎片合并 %d 次（一堵墙被切成几段 → 并回一段）\n",
                        st.n_merged_rects);
        }
        if (st.n_player_on_fiducial > 0) {
            std::printf("其中 %d 帧的「角色」落在四角标定标记上（已当没检出，只发 stay）\n",
                        st.n_player_on_fiducial);
        }
    }

    delete io;      // icraft/devmem 后端是 new 出来的
    return bad == 0 ? 0 : 1;
}
