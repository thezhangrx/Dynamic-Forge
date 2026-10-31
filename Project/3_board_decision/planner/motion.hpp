// ===========================================================================
// motion.hpp —— `core/vision/motion.py` 的 C++ 移植（板子 PS 侧跑的那一份）
//
// 为什么 PS 必须自己推速度：几何帧线协议里**没有速度字段** —— 障碍记录是
// `id, shape, type, x, y, radius, half_w, half_h, rotation`（见
// `core/fpga/rtl/vision_pkg.vh`），而决策（`gap_avoid`）要用 `Rect.vx/vy` 算
// "墙是不是在逼近"（`v_n = vn_wall - player_v·n̂`）。协议头里早就写明
// "速度由 PS 从相邻帧 stamp 之差推" —— 本文件就是那句话的实现。
//
// ⚠ 帧里的 `id` **不能**当跨帧身份用：RTL 侧是 `seg_id[seg_k] <= n_obs + 1`，
//   也就是**当帧序号**（按带内顺序 1,2,3…），下一帧可能换人。所以这里和 Python
//   参考实现一样，用**最近邻 + 门限**做关联，不按 id 配对。
//
// 契约（与 `core/cpu/cpp/README.md` 同一套口径）：
//   * 关联结果（哪条轨迹配哪个检测）、丢帧计数、新建轨迹 —— **必须一致**；
//   * `+ - * /` 的浮点结果        —— **逐位一致**；
//   * `hypot`（门限判据用）        —— **不要求逐位**（相对误差 < 1e-9），
//     Python 的 `math.hypot` 与 glibc `std::hypot` 算法不同，末位可差 1 ulp。
//     影响面只有"某次关联恰好卡在门限上"这种零测度情形。
//
// ⚠ 编译时**必须** `-ffp-contract=off`：α-β 更新里有 `t.vx += (beta/dt) * rx`
//   这类"乘加"，GCC 默认会收缩成 FMA（只舍入一次），与 Python 分开算的结果不同。
//
// 与 Python 的**唯一**行为差异（有意为之，且写在注释里）：
//   * stamp 不单调 / 不是有限数时，Python `Tracker.update()` 抛 `ValueError`
//     （"宁可炸"）；这里返回 `false` **且不改动任何状态**，由调用方决定怎么办。
//     板子上是实时控制环，不能因为一个坏时间戳把整条流水线崩掉：调用方
//     （`main.cpp`）选择 `reset()` 重新起时基。
// ===========================================================================
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace motion {

//: ByteTrack 实测的稳态 α-β 增益（与 Python 的 `DEFAULT_ALPHA/BETA` 同源）。
constexpr double kDefaultAlpha = 0.6608;
constexpr double kDefaultBeta = 0.0728;

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Config {
    //: 位置修正增益 / 速度修正增益（越大越跟手）。
    double alpha = kDefaultAlpha;
    double beta = kDefaultBeta;
    //: 关联门限的基础值（世界单位）。24 ≈ 1.2 个玩家直径。
    //: **不能拍脑袋调小**：调小会让每帧都判成新目标 —— 速度恒为 0，
    //: 而位置看着还是对的（Python 侧注释里记着这个坑）。
    double gate = 24.0;
    //: 门限随目标自身速率放宽的比例：`gate + frac * |v| * dt`。
    double gate_speed_frac = 0.5;
    //: 连续丢失多少帧后删除轨迹。
    int max_misses = 5;
    //: 帧间隔超过它就**不做预测**（速度置零重来），避免跨大间隔乱推。
    double max_dt = 0.5;
    //: 残差（预测落点与检测点的距离）超过它就**只落位、不更新速度**。
    //: 依据与实测数据见 `core/vision/motion.py` 里同名字段的注释：
    //: 门限（gate=24）回答"是不是同一个目标"，不能顺便当成"这个位移可信吗"——
    //: 真实素材里带合并一变，段的中心单帧横移 24 px，门限恰好放进来，于是造出
    //: -66 px/s 的假横向速度（真实运动只有 15~30 px/s）。8 是"最快真实步长
    //: （~2 单位/帧）"的 4 倍，又远小于那个 24。
    double max_residual = 8.0;
    //: 「快速起步」：轨迹速度**刚被清零/刚建立**时，先用 `fast_beta` 追几帧再回到
    //: 常规 `beta`。实测依据（30fps，见 core/vision/motion.py 同名字段）：
    //: 常规 β 下"物体出现时就在动"要 19 帧才追到 90%；fast 3 帧 β=0.3 只要 7 帧；
    //: 代价是轨迹头几帧会凭空造出速度（σ=0.5px 抖动下峰值 19 单位/秒，中位 3）。
    int fast_updates = 3;
    //: 快速起步阶段用的 β。
    double fast_beta = 0.3;
    //: 连续多少帧可疑才**改判**成"真运动"（然后照常 α-β 更新）。
    //: ⚠ 必须有：只看单帧残差的话，任何步长 > max_residual 的目标永远拿不到速度
    //: （速度恒 0、位置却看着对 —— 静默错）。这条是既有测试
    //: `core/vision/tests/test_pipeline.py::test_velocity_is_derived_from_stamp_not_frame_index`
    //: 逼出来的：那条测试的轨迹是 10 单位/帧，正好在 8 之上。
    int confirm_frames = 2;
};

struct Track {
    int id = 0;
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    //: 最后一次**被匹配到**的时刻（外推的帧不更新它）。
    double stamp = 0.0;
    int hits = 1;
    int misses = 0;
    int age = 1;
    //: 连续"可疑"（残差超阈值）的帧数；一旦某帧正常就清零。
    int suspect = 0;
    //: 距上次"速度被清零/刚建立"过了几帧（快速起步用）。
    int boot = 0;

    double speed() const { return std::hypot(vx, vy); }
    double heading() const {
        return (vx == 0.0 && vy == 0.0) ? 0.0 : std::atan2(vy, vx);
    }
    Point predict(double dt) const { return Point{x + vx * dt, y + vy * dt}; }
};

class Tracker {
public:
    Tracker() = default;
    explicit Tracker(const Config& cfg) : cfg_(cfg) {}

    //: 喂进这一帧的检测点（世界坐标）。返回 false = stamp 不单调/非有限，
    //: 此时**内部状态一点没动**（调用方可以 `reset()` 后重喂这一帧）。
    //:
    //: `stamp` 是**采集时刻**（秒，不是帧号 —— Python 侧那三个参考项目全在
    //: 这里翻车：ByteTrack 把 dt 写死成 1，速度单位变成"像素/帧"）。
    bool update(const Point* dets, int n_dets, double stamp);

    const std::vector<Track>& tracks() const { return tracks_; }

    //: 与 `tracks()` **等长同序**：每条轨迹对应哪一个检测（下标），
    //: `-1` = 本轮没匹配上（靠惯性外推）。调用方靠它把速度贴回自己的对象。
    const std::vector<int>& assignment() const { return assignment_; }

    void reset() {
        tracks_.clear();
        assignment_.clear();
        have_last_stamp_ = false;
        last_stamp_ = 0.0;
        next_id_ = 1;
        n_relocalized_ = 0;
    }

    bool have_last_stamp() const { return have_last_stamp_; }
    //: 累计触发过多少次"只落位、不更新速度"（诊断用）。
    int n_relocalized() const { return n_relocalized_; }
    const Config& config() const { return cfg_; }

private:
    Config cfg_{};
    std::vector<Track> tracks_;
    std::vector<int> assignment_;
    bool have_last_stamp_ = false;
    double last_stamp_ = 0.0;
    int next_id_ = 1;
    int n_relocalized_ = 0;
};

// ---------------------------------------------------------------------------
// 把跟踪结果贴回调用方自己的对象
//
// 为什么要有这个函数：轨迹的 `id` 是跟踪器自己发的，和帧里的 `id` 不是一回事
// （帧里那个是当帧序号）。没有这份对应关系，就只能"比位置猜哪个速度是谁的"。
//
// `out_vx/out_vy` 长度必须 >= n_dets；**没被任何轨迹认领的检测置 0**
// （新目标第一帧本来就不知道速度，置 0 是诚实值，不是兜底）。
// 返回贴上的条数。
// ---------------------------------------------------------------------------
int attach_velocities(const std::vector<Track>& tracks,
                      const std::vector<int>& assignment,
                      int n_dets, double* out_vx, double* out_vy);

//: 单目标的便捷包装：返回是否拿到速度（`out_vx/out_vy` 未拿到时置 0）。
bool track_one(Tracker& tr, const Point& p, double stamp,
               double* out_vx, double* out_vy);

}   // namespace motion
