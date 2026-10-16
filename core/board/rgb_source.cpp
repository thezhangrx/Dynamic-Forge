// ===========================================================================
// rgb_source.cpp —— 见 rgb_source.hpp
// ===========================================================================
#include "rgb_source.hpp"

#include "ppm_reader.hpp"

#include <cstdio>
#include <fcntl.h>
#include <thread>
#include <unistd.h>

namespace vision_board {

bool PpmSource::next(std::vector<uint8_t>* rgb, int* w, int* h, std::string* why) {
    std::string path;
    if (!dir_.empty()) {
        char name[32];
        std::snprintf(name, sizeof(name), "%06d.ppm", idx_);
        path = dir_ + "/" + name;
    } else {
        path = file_;                 // 单张图：每帧重复用它
    }
    ++idx_;
    if (!read_ppm_p6(path, w, h, rgb, why)) return false;
    return true;
}

bool StdinSource::next(std::vector<uint8_t>* rgb, int* w, int* h, std::string* why) {
    const size_t need = static_cast<size_t>(w_) * static_cast<size_t>(h_) * 3u;

    // 丢弃热身帧（自动曝光收敛用）。**必须真的读完再丢**，不能只 seek ——
    // 这是管道，没有"跳过"的概念。
    std::vector<uint8_t> scratch;
    while (skipped_ < skip_) {
        scratch.resize(need);
        size_t got = 0;
        while (got < need) {
            const ssize_t n = ::read(0, scratch.data() + got, need - got);
            if (n <= 0) {
                *why = n == 0 ? "热身时输入就结束了（ffmpeg 提前退出？）"
                              : "热身时读 stdin 出错";
                return false;
            }
            got += static_cast<size_t>(n);
        }
        ++skipped_;
    }

    if (max_ > 0 && got_ >= max_) {
        *why = "到 --frames 上限了";
        return false;
    }

    rgb->resize(need);
    size_t got = 0;
    while (got < need) {
        const ssize_t n = ::read(0, rgb->data() + got, need - got);
        if (n < 0) { *why = "读 stdin 出错"; return false; }
        if (n == 0) {
            // 管道关了：这是**正常结束**（ffmpeg 停了或我们被 Ctrl-C 了）
            *why = got == 0 ? "输入结束" : "输入在帧中途结束（只读到一部分）";
            return false;
        }
        got += static_cast<size_t>(n);
    }
    ++got_;
    *w = w_;
    *h = h_;
    return true;
}

}   // namespace vision_board

// ===========================================================================
// StdinPump —— 采集与处理重叠（见 rgb_source.hpp 的说明）
// ===========================================================================
namespace vision_board {

StdinPump::StdinPump(int width, int height, int skip, long max_frames)
    : w_(width), h_(height), skip_(skip), max_(max_frames),
      need_(static_cast<size_t>(width) * static_cast<size_t>(height) * 3u) {
    // ⚠ **把管道缓冲放大**，否则"采集与处理重叠"根本不成立。
    //
    // Linux 管道默认只有 64 KB，而一帧原始 RGB 是 691200 B（≈10.5 个管道）。
    // 于是 ffmpeg 写满 64 KB 就阻塞，一直等到我们把这一帧读完 —— 采集和处理
    // **被迫串行**。实测：59.4 ms/帧里 33.5 ms 花在"等采集"上，而摄像头本身
    // 就有 30 fps（33 ms），也就是说重叠本该把它藏掉。
    //
    // 放大到能装 3 帧之后，ffmpeg 可以一直跑在前面，读线程从队列里拿就行。
    // 上限受 /proc/sys/fs/pipe-max-size 限制（root 可调，见 cam_live.sh）。
    const int want = 3 * 1024 * 1024;
    const int got = ::fcntl(0, F_SETPIPE_SZ, want);
    pipe_bytes_ = got > 0 ? got : 0;   // 失败（stdin 不是管道）也不影响正确性
    th_ = std::thread([this] { run(); });
}

StdinPump::~StdinPump() {
    if (th_.joinable()) th_.join();
}

void StdinPump::recycle(std::vector<uint8_t>&& buf) {
    std::lock_guard<std::mutex> lk(m_);
    if (free_.size() < 4) free_.push_back(std::move(buf));
}

//: 单调时钟（纳秒）。用它而不是墙钟：stamp 要的是"帧之间的真实间隔"，
//: 墙钟会被 NTP 跳变污染（跳一次 → 某一帧的 dt 变成负数或巨大）。
uint64_t mono_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool StdinPump::get(std::vector<uint8_t>* out, std::string* why) {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [this] { return !ready_.empty() || done_; });
    if (ready_.empty()) { *why = why_; return false; }
    last_stamp_ns_ = ready_.front().second;
    *out = std::move(ready_.front().first);
    ready_.pop_front();
    cv_.notify_all();                 // 叫醒读线程：腾出位置了
    return true;
}

void StdinPump::run() {
    int local_skipped = 0;
    long local_got = 0;
    std::string err;
    // 丢掉热身帧（自动曝光收敛）—— 这一步在**这条线程**里做，
    // 所以主线程在热身期间就已经在等了，不会额外浪费时间。
    std::vector<uint8_t> buf;
    buf.resize(need_);
    while (local_skipped < skip_) {
        size_t got = 0;
        while (got < need_) {
            const ssize_t n = ::read(0, buf.data() + got, need_ - got);
            if (n <= 0) {
                err = n == 0 ? "热身时输入结束（ffmpeg 提前退出？）" : "热身时读 stdin 出错";
                goto finish;
            }
            got += static_cast<size_t>(n);
        }
        ++local_skipped;
    }
    for (;;) {
        if (max_ > 0 && local_got >= max_) break;
        {
            std::unique_lock<std::mutex> lk(m_);
            // 队列满了就等主线程取走（深度 2：再多只是把延迟堆高）
            cv_.wait(lk, [this] { return ready_.size() < 2 || done_; });
            if (done_) break;
        }
        // 取一个缓冲：优先用空闲池里的；**不在持锁状态下 read()**
        {
            std::lock_guard<std::mutex> lk(m_);
            if (!free_.empty()) { buf = std::move(free_.front()); free_.pop_front(); }
            else buf.resize(need_);
        }
        size_t got = 0;
        while (got < need_) {
            const ssize_t n = ::read(0, buf.data() + got, need_ - got);
            if (n < 0) { err = "读 stdin 出错"; goto finish; }
            if (n == 0) {
                if (got == 0) { err = "输入结束"; goto finish; }
                err = "输入在帧中途结束"; goto finish;
            }
            got += static_cast<size_t>(n);
        }
        ++local_got;
        // ⚠ 时间戳打在这里（**读完这一帧的那一刻**），不是主循环取走它的时候：
        //   主循环取走时已经过了推像素/等流水线，那会把处理耗时算进 dt，速度偏小。
        const uint64_t ts = mono_ns();
        {
            std::lock_guard<std::mutex> lk(m_);
            ready_.emplace_back(std::move(buf), ts);
        }
        cv_.notify_all();
    }
finish:
    {
        std::lock_guard<std::mutex> lk(m_);
        skipped_ = local_skipped;
        got_ = local_got;
        why_ = err.empty() ? "输入结束" : err;
        done_ = true;
    }
    cv_.notify_all();
}

}   // namespace vision_board
