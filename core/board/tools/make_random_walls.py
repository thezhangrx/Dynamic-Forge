#!/usr/bin/env python3
"""生成"多道静态墙 + **随机缺口**"的场景 JSON。

为什么单独做生成器：`configs/static_wall_40.json` 之类的缺口是
240/320/400/320 循环，视觉/板子看多了会"记住"节奏；联调要的是**每道墙缺口
随机**、但**必然可通行**（相邻缺口横向差有上限），并且**同一个 seed 可复现**。

规则（全部按场地单位）：
  * 墙 y 从 `--y0` 起每 `--dy` 一道，共 `--walls` 道；
  * 缺口中心走**随机游走**：`c_{i+1} = c_i + U(-step, +step)`，夹在 `[cmin, cmax]`；
    相邻缺口横向差 ≤ step，而两道墙间隔 dy、角色速度 200 → 有足够时间横移；
  * 缺口宽 `U(wmin, wmax)`（默认 70~90，角色半径 10、板子膨胀 13，余量够）；
  * 生成的就是普通 `wall_static` 弹型，仿真/板子/视觉都按真值走。

用法：
    Env/bin/python core/board/tools/make_random_walls.py \\
        --walls 24 --seed 20261020 --out configs/20_walls_rand_24.json
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

#: 墙厚（场地单位）——与 static_wall 平台一致
THICK = 8.0


def build(*, walls: int, seed: int, y0: float, dy: float,
          gmin: float, gmax: float, gap_wmin: float, gap_wmax: float,
          step: float, spawn: tuple[float, float],
          goal: tuple[float, float], field_h: float,
          duration: float) -> dict:
    rng = random.Random(int(seed))
    ys: list[float] = []
    gaps: list[float] = []
    widths: list[float] = []
    c = rng.uniform(gmin, gmax)
    for i in range(int(walls)):
        y = y0 + i * dy
        if i > 0:
            c = min(max(c + rng.uniform(-step, step), gmin), gmax)
        ys.append(float(y))
        gaps.append(float(round(c, 1)))
        widths.append(float(round(rng.uniform(gap_wmin, gap_wmax), 1)))

    pats = []
    for y, gx, gw in zip(ys, gaps, widths):
        pats.append({
            "kind": "obstacle", "count": 1, "start_time": 0.0, "interval": 1.0,
            "repetitions": 1, "origin": "field", "speed": 120.0, "speed_var": 0.0,
            "angle": -90.0, "angle_var": 0.0, "angle_spread": 360.0,
            "angular_velocity": 0.0, "accel": 0.0, "accel_direction": None,
            "ttl": 1.0e9, "radius": 3.0, "radius_var": 0.0, "aim": False,
            "jitter_pos": 0.0, "jitter_vel": 0.0, "shape": "rect",
            "half_w": 1.0, "half_h": THICK / 2.0, "rotation": 0.0,
            "min_player_distance": 0.0, "type_id": 9, "group_id": 0,
            "params": {"layout": "wall_static", "y": float(y),
                       "gap_center": float(gx), "gap_width": float(gw),
                       "thickness": THICK},
        })
    return {
        "name": f"20_walls_rand_{walls}",
        "seed": int(seed),
        "duration": float(duration),
        "dt": 1.0 / 120.0,
        "field_w": 640.0,
        "field_h": float(field_h),
        "wrap": "cull",
        "player_x": float(spawn[0]), "player_y": float(spawn[1]),
        "player_radius": 10.0, "player_speed": 200.0,
        "target_x": float(goal[0]), "target_y": float(goal[1]),
        "target_radius": 24.0, "target_shape": "circle",
        "target_half_w": 0.0, "target_half_h": 0.0,
        "bullet_capacity": None,
        "collision_penalty": 1.0,
        "collision_count_mode": "per_contact",
        "collision_terminates_episode": False,
        "success_terminates_episode": False,
        "difficulty": None, "complexity": None,
        "patterns": pats,
        "meta": {"goal": [float(goal[0]), float(goal[1])],
                 "spawn": [float(spawn[0]), float(spawn[1])],
                 "static": True, "follow_h": 240.0,
                 "walls_y": ys, "gaps_x": gaps, "gap_widths": widths},
        "collision": "auto", "safety_margin": 6.0, "safety_horizon": 2.0,
        "require_valid_scenario": True, "action_space": "discrete",
        "cull_margin": 32.0,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="生成多道静态墙 + 随机缺口场景")
    ap.add_argument("--walls", type=int, default=24)
    ap.add_argument("--seed", type=int, default=20261020)
    ap.add_argument("--y0", type=float, default=80.0)
    ap.add_argument("--dy", type=float, default=100.0)
    ap.add_argument("--gap-min", type=float, default=140.0,
                    help="缺口中心允许的最小 x")
    ap.add_argument("--gap-max", type=float, default=500.0,
                    help="缺口中心允许的最大 x")
    ap.add_argument("--gap-w-min", type=float, default=70.0)
    ap.add_argument("--gap-w-max", type=float, default=90.0)
    ap.add_argument("--step", type=float, default=110.0,
                    help="相邻两道墙缺口中心的横向随机步长上限")
    ap.add_argument("--spawn", type=float, nargs=2, default=(320.0, 40.0))
    ap.add_argument("--goal", type=float, nargs=2, default=None,
                    help="默认 = 最后一道墙上方 60")
    ap.add_argument("--field-h", type=float, default=None,
                    help="默认 = 最后一道墙 + 120")
    ap.add_argument("--duration", type=float, default=240.0)
    ap.add_argument("--out", default="configs/20_walls_rand_24.json")
    a = ap.parse_args(argv)

    last_y = a.y0 + (a.walls - 1) * a.dy
    goal = a.goal if a.goal is not None else (float(a.spawn[0]), last_y + 60.0)
    field_h = a.field_h if a.field_h is not None else last_y + 120.0
    cfg = build(walls=a.walls, seed=a.seed, y0=a.y0, dy=a.dy,
                gmin=a.gap_min, gmax=a.gap_max,
                gap_wmin=a.gap_w_min, gap_wmax=a.gap_w_max, step=a.step,
                spawn=tuple(a.spawn), goal=tuple(goal), field_h=field_h,
                duration=a.duration)
    out = Path(a.out)
    if not out.is_absolute():
        out = _ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    print(f"{out}  墙 {a.walls} 道  seed={a.seed}  field 640×{field_h:g}")
    for y, gx, gw in zip(cfg["meta"]["walls_y"], cfg["meta"]["gaps_x"],
                         cfg["meta"]["gap_widths"]):
        print(f"  y={y:7.1f}  gap_center={gx:6.1f}  width={gw:4.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
