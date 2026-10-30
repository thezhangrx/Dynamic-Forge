#!/usr/bin/env python3
"""本机离线试"人工势场斥力"（公式 5.5）—— 只调控制器参数，不碰相机。

**这是本机实验，不是闭环证据**（AGENTS.md R1：板子跑得好的结论必须来自相机链路）。
这里用场景真值当障碍，只为回答一个问题：

    给障碍加"只在 d ≤ Q* 内生效"的斥力（公式 5.5 的 U_rep），
    车进缺口时会不会被两侧墙"夹"到缺口中间，而不是贴边切角？

模型与主机逐位一致（`bullet_sim/physics/kinematics.py`）：速度 200、dt=1/120、
9 个离散动作、场地夹取；墙按"一行一条线 + 缺口"建模，厚 8（半厚 4）。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

DIRS = {
    0: (0.0, 0.0), 1: (-1.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 1.0), 4: (0.0, -1.0),
    5: (-math.sqrt(0.5), math.sqrt(0.5)), 6: (math.sqrt(0.5), math.sqrt(0.5)),
    7: (-math.sqrt(0.5), -math.sqrt(0.5)), 8: (math.sqrt(0.5), -math.sqrt(0.5)),
}


def pt_seg(x, y, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(x - x0, y - y0), (x0, y0)
    t = ((x - x0) * dx + (y - y0) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    px, py = x0 + t * dx, y0 + t * dy
    return math.hypot(x - px, y - py), (px, py)


class Scene:
    def __init__(self, cfg_path):
        cfg = json.loads(Path(cfg_path).read_text(encoding="utf-8"))
        m = cfg["meta"]
        self.fw = float(cfg["field_w"])
        self.fh = float(cfg["field_h"])
        self.r = float(cfg["player_radius"])
        self.speed = float(cfg["player_speed"])
        self.dt = float(cfg["dt"])
        self.goal = (float(cfg["target_x"]), float(cfg["target_y"]))
        self.goal_r = float(cfg["target_radius"])
        self.spawn = (float(cfg["player_x"]), float(cfg["player_y"]))
        self.walls = []      # [(y, gl, gr, half_h)]
        for p, y in zip(cfg["patterns"], m["walls_y"]):
            q = p["params"]
            cx, gw = float(q["gap_center"]), float(q["gap_width"])
            self.walls.append((float(y), cx - gw / 2.0, cx + gw / 2.0,
                               float(q.get("thickness", 8.0)) / 2.0))
        self.walls.sort()
        self.thick = self.walls[0][3] if self.walls else 4.0

    # ---- 几何 ---------------------------------------------------------
    def segs(self):
        """所有墙段：每行两段 [0,gl] / [gr,fw]，y=row.y。"""
        for (y, gl, gr, hh) in self.walls:
            if gl > 0.5:
                yield (0.0, y, gl, y, hh)
            if gr < self.fw - 0.5:
                yield (gr, y, self.fw, y, hh)

    def clearance(self, x, y):
        """到最近墙面的距离（减半径；含墙厚）。"""
        best = float("inf")
        for (x0, y0, x1, y1, hh) in self.segs():
            d, _ = pt_seg(x, y, x0, y0, x1, y1)
            c = d - self.r - hh
            if c < best:
                best = c
        return best

    def next_gap(self, py):
        for (y, gl, gr, _hh) in self.walls:
            if y > py + 2.0:
                return (0.5 * (gl + gr), y)
        return (self.goal[0], self.goal[1])


def force(sc, x, y, qstar, eta, krep, katt=1.0, target=None, clamp=5.0):
    """公式 5.5：F = -∇U_att - Σ∇U_rep（斥力只在 d ≤ Q* 内）。"""
    tx, ty = sc.next_gap(y) if target is None else target
    ax, ay = (tx - x), (ty - y)
    n = math.hypot(ax, ay)
    if n > 1e-9:
        ax, ay = ax / n * katt, ay / n * katt
    fx, fy = ax, ay
    dmin = float("inf")
    for (x0, y0, x1, y1, hh) in sc.segs():
        d, (nx, ny) = pt_seg(x, y, x0, y0, x1, y1)
        d = max(1e-3, d - hh)
        dmin = min(dmin, d)
        if d > qstar:
            continue
        mag = eta * (1.0 / d - 1.0 / qstar) / (d * d) * krep
        ux, uy = (x - nx), (y - ny)
        un = math.hypot(ux, uy)
        if un < 1e-9:
            continue
        fx += mag * ux / un
        fy += mag * uy / un
    n = math.hypot(fx, fy)
    if clamp and n > clamp:
        fx, fy = fx / n * clamp, fy / n * clamp
    return fx, fy, dmin


def quantise(fx, fy):
    n = math.hypot(fx, fy)
    if n < 1e-9:
        return 0
    ux, uy = fx / n, fy / n
    best, ba = -9.9, 0
    for a, (dx, dy) in DIRS.items():
        if a == 0:
            continue
        s = ux * dx + uy * dy
        if s > best:
            best, ba = s, a
    return ba


def run(cfg, *, qstar, eta, krep, hold=0.05, tmax=200.0,
        guard=True, inflate=2.5, verbose=False):
    sc = Scene(cfg)
    x, y = sc.spawn
    steps = int(round(tmax / sc.dt))
    hold_n = max(1, int(round(hold / sc.dt)))
    act = 0
    minclear = float("inf")
    cross = []          # 每个缺口穿越时的横向偏移
    passed = set()
    reach = False
    vel = 0.0
    for k in range(steps):
        if k % hold_n == 0:
            fx, fy, dmin = force(sc, x, y, qstar, eta, krep)
            cand = quantise(fx, fy)
            if guard:
                #: 和板上 send_act 同一套判据：这一步会不会更贴墙
                nx = x + DIRS[cand][0] * sc.speed * sc.dt * hold_n
                ny = y + DIRS[cand][1] * sc.speed * sc.dt * hold_n
                c0 = sc.clearance(x, y)
                c1 = sc.clearance(nx, ny)
                if c1 < inflate and c1 < c0 - 1e-9 and cand != 0:
                    for alt in (1, 2, 3, 4, 5, 6, 7, 8):
                        nx2 = x + DIRS[alt][0] * sc.speed * sc.dt * hold_n
                        ny2 = y + DIRS[alt][1] * sc.speed * sc.dt * hold_n
                        if sc.clearance(nx2, ny2) >= c0 - 1e-9:
                            cand = alt
                            break
                    else:
                        cand = 0
            act = cand
        dx, dy = DIRS[act]
        x += dx * sc.speed * sc.dt
        y += dy * sc.speed * sc.dt
        x = min(max(x, sc.r), sc.fw - sc.r)
        y = min(max(y, sc.r), sc.fh - sc.r)
        c = sc.clearance(x, y)
        if c < minclear:
            minclear = c
        #: 穿越行：第一次越过某行时记横向偏移
        for (wy, gl, gr, _hh) in sc.walls:
            if wy not in passed and y > wy + 2.0:
                passed.add(wy)
                cross.append(abs(x - 0.5 * (gl + gr)))
                break
        if math.hypot(x - sc.goal[0], y - sc.goal[1]) <= sc.goal_r:
            reach = True
            break
        vel += 0.0
    crossed = len(cross)
    med = sorted(cross)[len(cross) // 2] if cross else float("nan")
    return {
        "reach": reach, "y": round(y, 1), "x": round(x, 1),
        "min_clear": round(minclear, 2), "crossed": crossed,
        "off_med": None if not cross else round(med, 1),
        "off_max": None if not cross else round(max(cross), 1),
        "t_sim": round(k * sc.dt, 1), "n_wall": len(sc.walls),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="configs/20_walls_alt_24_dy140_y120.json")
    a = ap.parse_args()
    print("%-6s %-5s %-5s %-5s %-6s %-7s %-7s %-8s %-7s %s" % (
        "Q*", "eta", "krep", "hold", "guard", "到终点", "min_clr", "过墙数", "偏移中位", "偏移最大"))
    for qstar in (0.0, 30.0, 45.0, 60.0, 90.0):
        for eta in (0.0, 2e4, 6e4, 1.6e5, 4e5):
            r = run(a.scenario, qstar=qstar, eta=eta, krep=1.0)
            print("%-6.0f %-5.0f %-5.1f %-5.2f %-6s %-7s %-7.2f %-8d %-7s %s" % (
                qstar, eta, 1.0, 0.05, "on", r["reach"], r["min_clear"],
                r["crossed"], r["off_med"], r["off_max"]))


if __name__ == "__main__":
    raise SystemExit(main())
