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
    // ⚠ 管道只要**装得下一帧**就够，不能装 4 帧。
    //
    // 原来放 3 MB（≈4.3 帧）：读线程一停（哪怕只停一拍），ffmpeg 就往管道里灌
    // 三四帧**已经过期**的画面，之后我们每次 read() 拿到的都是管道最旧的那一帧。
    // 实测"取帧滞后"因此涨到 100 ms 以上，占整条链路延迟的一半。
    // 而且那 41 MB/s 的原始 RGB 要在 4 个 A7 核上搬来搬去，把 ffmpeg 的解码
    // CPU 也一起拖垮 —— 实测读线程不停下来时，主循环反而饿死（0.3 帧/秒）。
    // 管道 ≈ 一帧 + 一点余量：既保留"读这一帧时 ffmpeg 在解下一帧"的重叠，
    // 又把陈旧度锁死在 1 帧以内。
    const int want = static_cast<int>(need_) + 64 * 1024;
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
    if (drop_stale_) {
        // 只留最新一帧：排队等着的是"过去"，处理不过来时它只会把延迟做大。
        while (ready_.size() > 1) {
            if (free_.size() < 4) free_.push_back(std::move(ready_.front().first));
            ready_.pop_front();
            ++dropped_;
        }
    }
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
        // ⚠ 这里是**整条链路延迟的一半**所在：等，还是丢。
        //
        // 读线程一旦停下来等主线程取走，ffmpeg 就继续往管道里灌。管道有多大，
        // 就会积压多少帧"已经过期"的画面 —— 我们之后每次 read() 拿到的都是
        // 管道里**最旧**的那一帧（实测积压 100 ms 以上）。
        //
        // 反过来，"读线程永不停、只留最新帧"这条路在这块板上走不通：那会逼
        // ffmpeg 满速解码 640×360 MJPG（单核就已经吃掉 94%），41 MB/s 的原始
        // RGB 再从管道搬一遍，CPU 抢光之后主循环反而饿死（实测 0.3 帧/秒，
        // 400 帧里只算出 3 帧）。所以正确做法是两头都收：
        //   · 管道 ≈ 一帧（见构造函数）→ 积压最多 1 帧，还能保留解码/处理重叠；
        //   · 队列深度 1 → 手上有一帧在算时不再排第二帧。
        {
            std::unique_lock<std::mutex> lk(m_);
            // 深度 3：读线程**可以跑到前面**（它读得快、我们算得慢），
            // 但 `get()` 只取最新的那一帧（见 get() 里的 drop_stale 分支）——
            // 于是"排在前面的旧帧"只是读线程的缓冲，永远不会被拿去决策。
            // 深度 1 不行：读线程被迫和主循环锁步，帧在队列里干等一个完整周期
            // （实测取帧滞后 63 ms），而那时它已经是"上一轮的画面"了。
            const size_t depth = drop_stale_ ? 3 : 2;
            cv_.wait(lk, [this, depth] { return ready_.size() < depth || done_; });
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
