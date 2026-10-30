// M1 代价场决策内核：**板上 CLI 与视觉主程序共用同一份实现**。
//
// 为什么抽出来：真值在环用的 `field_decision_cli` 和视觉路径的 `main.cpp` 必须
// 走同一套 build/下降/EMA/迟滞，否则"板上验过的 M1"和"视觉路径的决策"又是两份。
//
// 支持**视口**：`build(walls, view_y0, view_h)` 只在视口内建场；墙可以只给当前
// 视角里的（`field_decision_cli --view-h`），也可以给全场的（视觉路径用 VT 的
// 冻结墙 + 全幅）。
#ifndef VISION_BOARD_FIELD_M1_HPP
#define VISION_BOARD_FIELD_M1_HPP

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace field_m1 {

struct Rect {
  double x = 0.0, y = 0.0, hw = 0.0, hh = 0.0;
};

struct Config {
  double field_w = 640.0, field_h = 480.0;
  double goal_x = 320.0, goal_y = 470.0;
  double cell = 10.0;        // 网格边长（场地单位）
  //: **硬阻挡带**（把墙周围一圈判成不可走）。用户 2026-10-23 口径：
  //: **墙周围不要判成禁区** —— 靠斥力的 `1/d` 自己把车推开就行。
  //: 所以本项在"带斥力"的配置里只留 ~1~2（只为把墙线本身那几格判成障碍，
  //: 好让波前仍然绕缺口走），不再是 13。
  double inflate = 13.0;
  double ema = 0.3;
  int hyst = 2;              // 新方向要连续 2 帧才换

  //: **人工斥力（公式 5.5 的 U_rep，Khatib 排斥）** —— 只加在代价场上，
  //: 不新增控制层。口径与 `core/cpu/potential_field.py` 同一套：
  //:
  //:     U_rep(p) = ½·η·(1/d − 1/rr)²      d < rr，否则 0
  //:     F(p)     = travel_到目标(p) + w_rep · rr² · U_rep(p)
  //:
  //: `travel` 是波前测地距离（会绕缺口），`rr²·U_rep` 把 1/长度² 的量纲换回
  //: "等效距离"，所以 `w_rep` 就是"贴墙惩罚最多值多少场地单位"。
  //: `d` = 格中心到**未膨胀**墙矩形表面的距离（chamfer 用 inflate 后的网格算，
  //: 这里用几何精算，避免让膨胀量顺带改变斥力形状）。
  //:
  //: **默认 w_rep = 0 → 逐位等于旧行为**（回归安全）。要"走中间"就把它打开，
  //: 本机 bench 扫出来的稳健区间在 `docs/闭环标准配置.md` 里。
  double repel_range = 120.0;   // rr：作用半径（场地单位），只在 d < rr 内生效
  double repel_gain = 1.0;      // η
  double repel_weight = 0.0;    // w_rep：打开就 > 0（建议从 20 起扫）
};

class Field {
 public:
  void configure(const Config& c) { cfg_ = c; }
  const Config& config() const { return cfg_; }
  bool built() const { return built_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  double view_y0() const { return view_y0_; }
  double view_h() const { return view_h_; }

  //: 建视口 [0, view_y0, field_w, view_h] 内的代价场。
  //: `goal` 在视口内 → 从目标格反向 Dijkstra；
  //: 在视口上方 → 视口顶行自由格作多源种子，种子代价 =
  //:   (goal_y − 视口顶) + |x − goal_x|；下方同理（底行）。
  void build(const std::vector<Rect>& walls, double view_y0, double view_h) {
    const double cell = cfg_.cell;
    view_y0_ = view_y0;
    view_h_ = view_h;
    nx_ = std::max(1, static_cast<int>(std::lround(cfg_.field_w / cell)));
    ny_ = std::max(1, static_cast<int>(std::lround(view_h / cell)));
    const double view_y1 = view_y0 + view_h;
    cost_.assign(static_cast<size_t>(nx_) * ny_, INFINITY);
    std::vector<char> blocked(static_cast<size_t>(nx_) * ny_, 0);

    for (const Rect& r : walls) {
      if (r.y + r.hh < view_y0 - 1e-9 || r.y - r.hh > view_y1 + 1e-9) continue;
      for (int ix = 0; ix < nx_; ++ix) {
        const double x = (ix + 0.5) * cell;
        if (std::fabs(x - r.x) > r.hw + cfg_.inflate) continue;
        for (int iy = 0; iy < ny_; ++iy) {
          const double y = view_y0 + (iy + 0.5) * cell;
          if (std::fabs(y - r.y) <= r.hh + cfg_.inflate) {
            blocked[static_cast<size_t>(iy) * nx_ + ix] = 1;
          }
        }
      }
    }
    auto free_cell = [&](int ix, int iy) {
      return ix >= 0 && iy >= 0 && ix < nx_ && iy < ny_ &&
             !blocked[static_cast<size_t>(iy) * nx_ + ix];
    };

    using P = std::pair<float, std::pair<int, int>>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    auto seed = [&](int ix, int iy, float c) {
      if (!free_cell(ix, iy)) return;
      const size_t k = static_cast<size_t>(iy) * nx_ + ix;
      if (c < cost_[k]) { cost_[k] = c; pq.push({c, {ix, iy}}); }
    };

    if (cfg_.goal_y >= view_y0 && cfg_.goal_y <= view_y1) {
      int bx = -1, by = -1;
      double bd = 1e18;
      for (int iy = 0; iy < ny_; ++iy) {
        for (int ix = 0; ix < nx_; ++ix) {
          if (!free_cell(ix, iy)) continue;
          const double d = std::hypot((ix + 0.5) * cell - cfg_.goal_x,
                                      view_y0 + (iy + 0.5) * cell - cfg_.goal_y);
          if (d < bd) { bd = d; bx = ix; by = iy; }
        }
      }
      if (bx >= 0) seed(bx, by, 0.0f);
    } else if (cfg_.goal_y > view_y1) {
      const float extra = static_cast<float>(cfg_.goal_y - view_y1);
      bool any = false;
      for (int ix = 0; ix < nx_; ++ix) {
        if (!free_cell(ix, ny_ - 1)) continue;
        seed(ix, ny_ - 1, extra + static_cast<float>(
            std::fabs((ix + 0.5) * cell - cfg_.goal_x)));
        any = true;
      }
      if (!any) {
        for (int iy = ny_ - 2; iy >= 0 && !any; --iy) {
          for (int ix = 0; ix < nx_; ++ix) {
            if (!free_cell(ix, iy)) continue;
            seed(ix, iy, extra + static_cast<float>(
                std::fabs((ix + 0.5) * cell - cfg_.goal_x) + (ny_ - 1 - iy) * cell));
            any = true;
          }
        }
      }
    } else {
      const float extra = static_cast<float>(view_y0 - cfg_.goal_y);
      bool any = false;
      for (int ix = 0; ix < nx_; ++ix) {
        if (!free_cell(ix, 0)) continue;
        seed(ix, 0, extra + static_cast<float>(
            std::fabs((ix + 0.5) * cell - cfg_.goal_x)));
        any = true;
      }
      if (!any) {
        for (int iy = 1; iy < ny_ && !any; ++iy) {
          for (int ix = 0; ix < nx_; ++ix) {
            if (!free_cell(ix, iy)) continue;
            seed(ix, iy, extra + static_cast<float>(
                std::fabs((ix + 0.5) * cell - cfg_.goal_x) + iy * cell));
            any = true;
          }
        }
      }
    }

    while (!pq.empty()) {
      const auto top = pq.top(); pq.pop();
      const float c = top.first;
      const int ix = top.second.first, iy = top.second.second;
      if (c > at(ix, iy)) continue;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (!dx && !dy) continue;
          const int jx = ix + dx, jy = iy + dy;
          if (!free_cell(jx, jy)) continue;
          const float nc = c + static_cast<float>(std::hypot(dx, dy) * cell);
          if (nc < at(jx, jy)) {
            cost_[static_cast<size_t>(jy) * nx_ + jx] = nc;
            pq.push({nc, {jx, jy}});
          }
        }
      }
    }
    //: 波前（`travel`）算完再叠斥力项 —— 二者的"零"不同：
    //: travel 以目标为 0 反向传播，斥力只跟"离墙多远"有关，必须在 Dijkstra
    //: **之后**加，否则斥力会被当成波前的种子传播、把测地距离算歪。
    if (cfg_.repel_weight > 0.0 && cfg_.repel_range > 0.0) apply_repulsion(walls);
    built_ = true;
  }

  //: 8 邻域下降 + EMA + 迟滞；返回 0..8 动作码（0 = stay）。
  int step(double px, double py) {
    if (!built_) return 0;
    int ix = static_cast<int>((px - 0.0) / cfg_.cell);
    int iy = static_cast<int>((py - view_y0_) / cfg_.cell);
    ix = std::min(nx_ - 1, std::max(0, ix));
    iy = std::min(ny_ - 1, std::max(0, iy));
    int bx = ix, by = iy;
    float bc = at(ix, iy);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const int jx = ix + dx, jy = iy + dy;
        if (jx < 0 || jy < 0 || jx >= nx_ || jy >= ny_) continue;
        if (at(jx, jy) < bc) { bc = at(jx, jy); bx = jx; by = jy; }
      }
    }
    double vx = 0, vy = 0;
    if (std::isfinite(bc) && (bx != ix || by != iy)) {
      vx = bx - ix; vy = by - iy;
      const double m = std::hypot(vx, vy) || 1.0;
      vx /= m; vy /= m;
    }
    ema_x_ = (1 - cfg_.ema) * ema_x_ + cfg_.ema * vx;
    ema_y_ = (1 - cfg_.ema) * ema_y_ + cfg_.ema * vy;
    const int want = quantize(ema_x_, ema_y_);
    int act;
    if (want == prev_act_) { streak_ = 0; act = prev_act_; }
    else { ++streak_; act = (streak_ >= cfg_.hyst) ? want : prev_act_; }
    prev_act_ = act;
    return act;
  }

  //: 动作码 → 单位方向（给 `gap_avoid::Vec2` 那类接口用）。
  static void direction(int act, double* ux, double* uy) {
    static const double dirs[9][2] = {{0, 0}, {-1, 0}, {1, 0}, {0, 1}, {0, -1},
                                      {-0.70710678, 0.70710678},
                                      {0.70710678, 0.70710678},
                                      {-0.70710678, -0.70710678},
                                      {0.70710678, -0.70710678}};
    const int k = (act >= 0 && act <= 8) ? act : 0;
    *ux = dirs[k][0];
    *uy = dirs[k][1];
  }

  void reset_state() { ema_x_ = ema_y_ = 0.0; prev_act_ = 0; streak_ = 0; }

 private:
  float at(int ix, int iy) const {
    return cost_[static_cast<size_t>(iy) * nx_ + ix];
  }

  //: 点到轴对齐矩形的距离（矩形外 → 表面距离；矩形内 → 0）。
  static double rect_dist(double x, double y, const Rect& r) {
    const double dx = std::max(std::fabs(x - r.x) - r.hw, 0.0);
    const double dy = std::max(std::fabs(y - r.y) - r.hh, 0.0);
    return std::hypot(dx, dy);
  }

  //: F += w_rep·rr²·½η·(1/d − 1/rr)²（仅 d < rr；含墙/均无墙时不动）。
  void apply_repulsion(const std::vector<Rect>& walls) {
    const double rr = cfg_.repel_range;
    const double k = cfg_.repel_weight * rr * rr * 0.5 * cfg_.repel_gain;
    for (int iy = 0; iy < ny_; ++iy) {
      const double y = view_y0_ + (iy + 0.5) * cfg_.cell;
      for (int ix = 0; ix < nx_; ++ix) {
        const size_t idx = static_cast<size_t>(iy) * nx_ + ix;
        if (!std::isfinite(cost_[idx])) continue;      // 膨胀禁区恒为 inf
        const double x = (ix + 0.5) * cfg_.cell;
        double d = 1e18;
        for (const Rect& r : walls) {
          const double dd = rect_dist(x, y, r);
          if (dd < d) d = dd;
          if (d <= 0.0) break;
        }
        //: **`1/d` 做分母 ⇒ 越靠近墙势越大**，所以不需要"禁区"：
        //: 真正贴到墙上的格代价是天文数字，下降自然不会去。这里只夹一个
        //: 半个格的下限（`d>=cell/2`），纯粹为了让浮点数别溢出成 inf。
        d = std::max(d, 0.5 * cfg_.cell);
        if (d >= rr) continue;
        const double u = 1.0 / d - 1.0 / rr;
        cost_[idx] += static_cast<float>(k * u * u);
      }
    }
  }

  static int quantize(double vx, double vy) {
    const double n = std::hypot(vx, vy);
    if (n < 1e-6) return 0;
    const double ux = vx / n, uy = vy / n;
    const double dirs[9][2] = {{0, 0}, {-1, 0}, {1, 0}, {0, 1}, {0, -1},
                               {-0.70710678, 0.70710678},
                               {0.70710678, 0.70710678},
                               {-0.70710678, -0.70710678},
                               {0.70710678, -0.70710678}};
    int best = 0;
    double bestd = -2;
    for (int k = 1; k <= 8; ++k) {
      const double d = ux * dirs[k][0] + uy * dirs[k][1];
      if (d > bestd) { bestd = d; best = k; }
    }
    return best;
  }

  Config cfg_;
  double view_y0_ = 0.0, view_h_ = 0.0;
  int nx_ = 0, ny_ = 0;
  std::vector<float> cost_;
  bool built_ = false;
  double ema_x_ = 0.0, ema_y_ = 0.0;
  int prev_act_ = 0, streak_ = 0;
};

}  // namespace field_m1

#endif  // VISION_BOARD_FIELD_M1_HPP
