// 鲁棒时间窗估计 —— "宁可要一个大概的范围，也不要一个不稳定的点值"。
//
// 为什么需要（用户提出的要求，实测也支持）
// ------------------------------------
// 单帧识别不可避免地有抖动和偶发错块：实测角色单帧误差 p50 35、p95 159 场地单位，
// 缺口帧间跳变 p90 28 单位。而**决策要的是一个稳定目标**：目标抖 → 指令抖 →
// 平台左右乱晃（实测：缺口跳变 >20 单位的帧里，指令方向翻转率 29%，平稳帧只有 8%）。
//
// 所以这一层的口径是：
//   1. **时间窗内取鲁棒统计**（截尾均值 / 中位数），而不是"这一帧是多少就是多少"；
//   2. **离群值丢掉**（用中位绝对偏差 MAD 判，不用均值±3σ —— 均值本身会被离群值带跑）；
//   3. 同时给出**范围**（低/高分位），决策可以按"范围"做事（例如取范围中点、
//      或按范围判断这一帧目标是否足够确定），而不是假精确。
//
// 关键设计点：**窗口按时间而不是按帧数**。帧率会变（13~18 帧/秒），按帧数会让
// 窗口时长飘忽；按时间才能保证"200~300 ms 的惯性"是恒定的。
#ifndef VISION_BOARD_ROBUST_HPP
#define VISION_BOARD_ROBUST_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace vision_board {
namespace robust {

//: 一个标量的时间窗估计器。
class Window {
public:
    explicit Window(double window_s = 0.35, size_t max_n = 12)
        : window_s_(window_s), max_n_(max_n) {}

    //: 压入一个样本（`t_s` 是该样本的**时间**，秒；建议用单调时钟）。
    void push(double v, double t_s) {
        if (!std::isfinite(v)) return;
        s_.push_back({v, t_s});
        trim(t_s);
    }

    void clear() { s_.clear(); }
    bool empty() const { return s_.empty(); }
    size_t size() const { return s_.size(); }

    //: 丢掉超出时间窗的旧样本。
    void trim(double now_s) {
        while (!s_.empty() && now_s - s_.front().t > window_s_) s_.pop_front();
        while (s_.size() > max_n_) s_.pop_front();
    }

    //: 中位数（对离群值最稳）。
    double median() const {
        if (s_.empty()) return 0.0;
        std::vector<double> v = vals();
        const size_t n = v.size();
        std::nth_element(v.begin(), v.begin() + static_cast<long>(n / 2), v.end());
        const double hi = v[n / 2];
        if (n % 2 == 1) return hi;
        std::nth_element(v.begin(), v.begin() + static_cast<long>(n / 2 - 1), v.end());
        return 0.5 * (hi + v[n / 2 - 1]);
    }

    //: 截尾均值：先按 MAD 去掉离群值，再去掉两端各 `trim_frac`，剩下的取平均。
    //: 比中位数更"用到全部信息"，又不像普通均值那样被一个错块带跑。
    double robust_mean(double trim_frac = 0.2, double mad_k = 3.0) const {
        if (s_.empty()) return 0.0;
        std::vector<double> v = vals();
        const double med = median();
        std::vector<double> dev(v.size());
        for (size_t i = 0; i < v.size(); ++i) dev[i] = std::fabs(v[i] - med);
        std::nth_element(dev.begin(), dev.begin() + static_cast<long>(dev.size() / 2), dev.end());
        const double mad = dev[dev.size() / 2] * 1.4826;   // → 等效 σ
        std::vector<double> keep;
        keep.reserve(v.size());
        for (double x : v) {
            if (mad > 1e-9 && std::fabs(x - med) > mad_k * mad) continue;   // 离群
            keep.push_back(x);
        }
        if (keep.empty()) return med;
        std::sort(keep.begin(), keep.end());
        const size_t cut = static_cast<size_t>(trim_frac * static_cast<double>(keep.size()));
        size_t lo = cut, hi = keep.size() - cut;
        if (hi <= lo) { lo = 0; hi = keep.size(); }
        double sum = 0.0;
        for (size_t i = lo; i < hi; ++i) sum += keep[i];
        return sum / static_cast<double>(hi - lo);
    }

    //: 鲁棒范围（低/高分位）。决策可以按范围做事，而不是假精确。
    double lo(double q = 0.15) const { return quantile(q); }
    double hi(double q = 0.85) const { return quantile(q); }
    double spread() const { return hi() - lo(); }

    double quantile(double q) const {
        if (s_.empty()) return 0.0;
        std::vector<double> v = vals();
        std::sort(v.begin(), v.end());
        const double pos = q * static_cast<double>(v.size() - 1);
        const size_t i = static_cast<size_t>(pos);
        const size_t j = std::min(i + 1, v.size() - 1);
        const double f = pos - static_cast<double>(i);
        return v[i] * (1.0 - f) + v[j] * f;
    }

    //: 样本够不够（少于 n 个就不要拿去决策 —— 宁可 say INVALID）。
    bool enough(size_t n = 3) const { return s_.size() >= n; }

private:
    std::vector<double> vals() const {
        std::vector<double> v;
        v.reserve(s_.size());
        for (const auto& x : s_) v.push_back(x.v);
        return v;
    }
    struct S { double v; double t; };
    std::deque<S> s_;
    double window_s_ = 0.35;
    size_t max_n_ = 12;
};

//: 二维点的时间窗（两轴各自独立统计 —— x、y 的噪声特性不一样，混在一起只会互相污染）。
struct Window2 {
    Window x, y;
    Window2(double window_s = 0.35, size_t max_n = 12) : x(window_s, max_n), y(window_s, max_n) {}
    void push(double px, double py, double t) { x.push(px, t); y.push(py, t); }
    void clear() { x.clear(); y.clear(); }
    bool enough(size_t n = 3) const { return x.enough(n) && y.enough(n); }
    double cx() const { return x.robust_mean(); }
    double cy() const { return y.robust_mean(); }
    //: 这个点在窗口里的"不确定度"（两轴范围之和）。决策可以用它判断目标够不够确定。
    double uncertainty() const { return x.spread() + y.spread(); }
};

}   // namespace robust
}   // namespace vision_board

#endif   // VISION_BOARD_ROBUST_HPP
