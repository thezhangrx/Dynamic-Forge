// M1 决策器（板上运行）：**Dijkstra 代价场 + 8 邻域下降 + EMA + 迟滞**。
//
// 输入：`@STATE <ver> px py pr n [x y hw hh rot]*`（仿真真值，场地单位），一行一帧。
// 输出：`@MOVE <code>`（换向/心跳时打印）；`--echo` 时每个输入再回 `@ACK <ver> <act>`。
//
// 算法本体在 `field_m1.hpp`（与视觉主程序 `main.cpp` 共用同一份实现）。
//
// **视角跟随模式**（`--view-h H`）：只根据当前视口里的墙建势场；目标用 `--goal`
// 告知。目标在视口外时以顶行（或底行）自由格作多源边界条件。
//
// 编译（板上）：
//   g++ -std=c++17 -O2 -I /root/tools -o /root/tools/field_decision_cli
//       /root/tools/field_decision_cli.cpp
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "field_m1.hpp"

namespace {

bool parse_walls(const char* p, int n, std::vector<field_m1::Rect>* out) {
  out->clear();
  const char* r = p;
  for (int i = 0; i < n; ++i) {
    field_m1::Rect q;
    char* e = nullptr;
    q.x = std::strtod(r, &e); if (e == r) break; r = e;
    q.y = std::strtod(r, &e); if (e == r) break; r = e;
    q.hw = std::strtod(r, &e); if (e == r) break; r = e;
    q.hh = std::strtod(r, &e); if (e == r) break; r = e;
    (void)std::strtod(r, &e);            // rotation
    if (e != r) r = e;
    out->push_back(q);
  }
  return static_cast<int>(out->size()) == n;
}

}  // namespace

int main(int argc, char** argv) {
  bool echo = false;
  double field_w = 640.0, field_h = 480.0;
  double goal_x = 320.0, goal_y = 470.0;
  double view_h = 0.0;               // 0 = 全幅（老行为）
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    auto next = [&](double def) -> double {
      return (i + 1 < argc) ? std::atof(argv[++i]) : def;
    };
    if (std::strcmp(a, "--echo") == 0) echo = true;
    else if (std::strcmp(a, "--field-w") == 0) field_w = next(field_w);
    else if (std::strcmp(a, "--field-h") == 0) field_h = next(field_h);
    else if (std::strcmp(a, "--goal") == 0) {
      goal_x = next(goal_x);
      goal_y = next(goal_y);
    } else if (std::strcmp(a, "--view-h") == 0) view_h = next(view_h);
  }
  if (field_w <= 0 || field_h <= 0) {
    std::fprintf(stderr, "field 尺寸必须 > 0\n");
    return 2;
  }
  if (view_h > field_h) view_h = field_h;

  field_m1::Config cfg;
  cfg.field_w = field_w;
  cfg.field_h = field_h;
  cfg.goal_x = goal_x;
  cfg.goal_y = goal_y;
  field_m1::Field field;
  field.configure(cfg);

  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::vector<field_m1::Rect> walls;
  char line[65536];
  int frames = 0, last_act = 0, last_n = -1;
  bool printed_parse = false, printed_build = false, printed_reflex = false;

  while (std::fgets(line, sizeof(line), stdin)) {
    double px = 0, py = 0, pr = 10;
    int n = 0, ver = 0;
    const char* p = line;
    if (std::strncmp(p, "@STATE", 6) == 0) {
      int off = 0;
      if (std::sscanf(p, "@STATE %d %lf %lf %lf %d%n",
                      &ver, &px, &py, &pr, &n, &off) < 5) continue;
      p += off;
    } else if (std::strncmp(p, "@GT", 3) == 0) {
      int off = 0;
      if (std::sscanf(p, "@GT %lf %lf %lf %d%n", &px, &py, &pr, &n, &off) < 4) continue;
      p += off;
      ver = -1;
    } else {
      continue;                    // 忽略日志噪声
    }
    if (!parse_walls(p, n, &walls)) continue;

    const double vh = (view_h > 0.0) ? view_h : field_h;
    double vy0 = 0.0;
    if (view_h > 0.0) {
      vy0 = py - vh * 0.5;
      vy0 = std::min(std::max(vy0, 0.0), std::max(0.0, field_h - vh));
    }
    if (!printed_parse) {
      std::fprintf(stderr,
                   "[field] 解析到 %d 段墙（@STATE 里 n=%d），field %.0f×%.0f，"
                   "goal (%.0f,%.0f)，view-h %.0f\n",
                   static_cast<int>(walls.size()), n, field_w, field_h,
                   goal_x, goal_y, view_h);
      printed_parse = true;
      last_n = static_cast<int>(walls.size());
    }
    const bool need = !field.built() || view_h > 0.0 ||
                      static_cast<int>(walls.size()) != last_n;
    if (need) {
      field.build(walls, vy0, vh);
      last_n = static_cast<int>(walls.size());
      if (!printed_build) {
        std::fprintf(stderr, "[field] 代价场建好：%d×%d，墙 %zu 段\n",
                     field.nx(), field.ny(), walls.size());
        printed_build = true;
      }
    }

    int act = field.step(px, py);

    //: **安全反射**：不管 M1 给出什么，只要角色边缘离最近一段墙 < 2 单位，
    //: 就改成"停"（act=0），别再往里挤。实测相机几何有 ±10 单位的误差，
    //: 偶尔会让 M1 贴着墙角过；这一层是纯板端的兜底，不依赖主机。
    double min_clear = 1e9;
    for (const auto& w : walls) {
      const double dx = std::max(std::abs(px - w.x) - w.hw, 0.0);
      const double dy = std::max(std::abs(py - w.y) - w.hh, 0.0);
      min_clear = std::min(min_clear, std::hypot(dx, dy) - pr);
    }
    if (min_clear < 2.0) {
      //: 已经在墙边/墙里时不能一律停（会僵死）：在 9 个动作里挑"下一步离墙最远"
      //: 的那个；M1 的动作如果也不差就优先它。这样贴近时是往外退/贴着滑，
      //: 不是原地卡住。
      static const double dirs[9][2] = {{0, 0}, {-1, 0}, {1, 0}, {0, 1}, {0, -1},
                                        {-0.70710678, 0.70710678},
                                        {0.70710678, 0.70710678},
                                        {-0.70710678, -0.70710678},
                                        {0.70710678, -0.70710678}};
      const double step = 8.0;
      auto clear_at = [&](double x, double y) {
        double m = 1e9;
        for (const auto& w : walls) {
          const double dx = std::max(std::abs(x - w.x) - w.hw, 0.0);
          const double dy = std::max(std::abs(y - w.y) - w.hh, 0.0);
          m = std::min(m, std::hypot(dx, dy) - pr);
        }
        return m;
      };
      double best = clear_at(px, py);
      int best_act = 0;
      for (int k = 1; k <= 8; ++k) {
        const double c = clear_at(px + dirs[k][0] * step, py + dirs[k][1] * step);
        if (c > best + 1e-9) { best = c; best_act = k; }
      }
      const double c_m1 = clear_at(px + dirs[act][0] * step, py + dirs[act][1] * step);
      if (c_m1 >= best - 0.5) best_act = act;
      act = best_act;
      if (!printed_reflex) {
        std::fprintf(stderr, "[field] 安全反射：clear=%.2f < 2 → 改走 act=%d\n",
                     min_clear, act);
        printed_reflex = true;
      }
    }
    ++frames;
    if (act != last_act || frames % 8 == 1) std::printf("@MOVE %d\n", act);
    last_act = act;
    if (echo) std::printf("@ACK %d %d\n", ver, act);
  }
  return 0;
}
