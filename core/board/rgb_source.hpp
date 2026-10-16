// ===========================================================================
// rgb_source.hpp —— 一帧 RGB 从哪来（PPM 文件 / 标准输入流）
//
// 为什么要有这层：板上"视觉 → 决策 → 键盘"这个循环的**帧率瓶颈不在 FPGA**
// （PL 侧实测 39.6 帧/秒），而在采集那一环。原来每帧都是
//
//     ffmpeg -frames:v 1 ... out.ppm && ./vision_board_devmem --ppm out.ppm
//
// 也就是**每帧重开一次摄像头设备 + 落一次盘**，实测 1.0～1.25 秒/帧（≈1 帧/秒）。
// 换成"一次开设备、连续出帧、原始 RGB 走管道"之后，上限就回到摄像头本身
// （实测 25.3 帧/秒）。
//
// 所以这里把"帧从哪来"抽出来，两种来源共用下游（推 PL / 参考模型 / 键盘）：
//   * `PpmSource`   —— 文件或目录（`--ppm` / `--ppm-dir`），批处理与回归用；
//   * `StdinSource` —— 从标准输入读**原始 RGB24**（`--stdin-rgb`），实时用。
//
// ⚠ `StdinSource` 读的是 `W*H*3` 字节一帧、没有头。喂它的命令形如
//     ffmpeg ... -pix_fmt rgb24 -f rawvideo -
//   尺寸必须与 PL 的 WIDTH/HEIGHT 一致（PL 那边是编译期常量）。
// ===========================================================================
#ifndef VISION_BOARD_RGB_SOURCE_HPP
#define VISION_BOARD_RGB_SOURCE_HPP

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vision_board {

//: 取下一帧 RGB。`next()` 返回 false 表示**没有更多帧**（正常结束，不是错误），
//: 原因写在 `why` 里。
class RgbSource {
public:
    virtual ~RgbSource() = default;
    virtual bool next(std::vector<uint8_t>* rgb, int* w, int* h,
                      std::string* why) = 0;
    virtual const char* kind() const = 0;
};

//: 文件或目录（`%06d.ppm`）。给批处理与回归用；也是 `--ppm` / `--ppm-dir`。
class PpmSource final : public RgbSource {
public:
    PpmSource(std::string file, std::string dir)
        : file_(std::move(file)), dir_(std::move(dir)) {}
    bool next(std::vector<uint8_t>* rgb, int* w, int* h,
              std::string* why) override;
    const char* kind() const override { return "PPM 文件"; }

private:
    std::string file_, dir_;
    int idx_ = 0;
};

//: 标准输入上的原始 RGB24 流。给实时采集用。
//
//  `skip` 是**丢掉的帧数**：摄像头的自动曝光要好几帧才收敛（实测第 5～9 帧才
//  稳定，之前抓到过全黑的第 1 帧），所以开头几帧必须丢 —— 这不是可选项，
//  是"能不能拿到一张能用的图"的问题。
//
//  `max_frames` <= 0 表示一直读到输入结束（ffmpeg 断了就自然结束）。
class StdinSource final : public RgbSource {
public:
    StdinSource(int width, int height, int skip, long max_frames)
        : w_(width), h_(height), skip_(skip), max_(max_frames) {}
    bool next(std::vector<uint8_t>* rgb, int* w, int* h,
              std::string* why) override;
    const char* kind() const override { return "stdin 原始 RGB"; }
    //: 已经丢掉了多少帧（用于在启动时打一句"热身"说明）。
    int skipped() const { return skipped_; }

private:
    int w_, h_, skip_, skipped_ = 0, pipe_bytes_ = 0;
    long max_, got_ = 0;
};

//: **采集与处理重叠**用的读线程。
//
// 为什么需要：实测实时那条路上三段是**串行**的 ——
//     等采集 33ms + 推 PL 17ms + 等流水线 21ms = 72ms（≈14 帧/秒）
// 而这三段本来可以重叠：主线程在推像素、等流水线的时候，读线程就可以去
// 取下一帧。重叠之后应当接近 `max(33, 38) ≈ 40ms`（≈25 帧/秒）。
//
// 队列深度 2 就够（再多只是把延迟堆高），并且用**移动语义 + 空闲池**，
// 稳态下不分配内存、不多拷一份 691KB。
//
// 线程只做 `read()`；没有别的共享状态，主线程取完帧还会把缓冲还回池里。
class StdinPump {
public:
    StdinPump(int width, int height, int skip, long max_frames);
    ~StdinPump();
    StdinPump(const StdinPump&) = delete;
    StdinPump& operator=(const StdinPump&) = delete;

    //: 取一帧（阻塞直到有帧或结束）。false = 没有更多帧了，原因见 :meth:`why`。
    bool get(std::vector<uint8_t>* out, std::string* why);

    //: 上一帧 `get()` 拿到的像素是**什么时候读进来的**（CLOCK_MONOTONIC 纳秒）。
    //:
    //: 为什么要它：几何帧的 `stamp_ns` 是"采集时刻"，而速度是 `Δ位置 / Δstamp`。
    //: 以前主循环每帧固定 `+= 33.3 ms`（30 fps），可相机实测交付 **40~48 fps** ——
    //: 于是推出来的速度被系统性缩小 (30/实际fps) 倍（40 fps 时低 28%，60 fps 时低一半），
    //: 而决策的 `t_arr = 距离 / v_n` 直接吃这个数。让读线程在读完后打时间戳，
    //: 才是真正的"采集间隔"，也把"处理耗时"排除在 dt 之外（同 `to_standard.py` 的纪律）。
    uint64_t last_stamp_ns() const { return last_stamp_ns_; }
    //: 把用完的缓冲还回空闲池（省一次分配；不还也不影响正确性）。
    void recycle(std::vector<uint8_t>&& buf);
    int skipped() const { return skipped_; }
    //: 实际设成的管道缓冲字节数（0 = 没设成，stdin 不是管道）。
    int pipe_bytes() const { return pipe_bytes_; }

private:
    void run();

    int w_, h_, skip_, skipped_ = 0, pipe_bytes_ = 0;
    long max_, got_ = 0;
    size_t need_;
    std::thread th_;
    std::mutex m_;
    std::condition_variable cv_;
    //: 已读好的帧 + 它的**采集时刻**（读线程读完那一刻，纳秒）
    std::deque<std::pair<std::vector<uint8_t>, uint64_t>> ready_;
    std::deque<std::vector<uint8_t>> free_;    // 可复用的缓冲
    bool done_ = false;
    std::string why_;
    uint64_t last_stamp_ns_ = 0;
};

}   // namespace vision_board

#endif   // VISION_BOARD_RGB_SOURCE_HPP
