#!/usr/bin/env python3
"""网格势场方案调用入口 —— 和 ``core/cpu/`` 同名的根目录入口（约定：前缀 = 模块名）。

    python cpu_field_bench.py                          # 默认：4 种子、难度 0.4、目标=场中央
    python cpu_field_bench.py --seeds 1 2 3 4 5 6 7 8  # 8 个种子
    python cpu_field_bench.py --difficulty 0.6
    python cpu_field_bench.py --weights 0 1 5          # 扫行程代价权重 λ
    python cpu_field_bench.py --cell 16 --descend 4    # 粗网格（快 3 倍）
    python cpu_field_bench.py --nav                    # 绕墙导航测试（本方案的价值所在）

实现全在 ``core/cpu/``：

    potential_field.py   网格势场 + 波前测地距离（纯 Python 零依赖，唯一入口 plan_field）
    field_bench.py       本文件的转发目标：在平台真实场景上评测碰撞与延迟

**为什么要有这个方案**：朴素势场（APF）把"势 + 直线距离"当决策依据，于是
"紧贴墙背后"那块空地（势=0、直线又近）会被选中 —— 角色一头撞进墙。
本方案把"直线距离"换成**测地距离**（波前），墙那边的点代价自然变大。
``--nav`` 子命令就是这条结论的现场演示。

**必须有负面对照**：``原地不动`` 那一行。如果连它都报 0 碰撞，
说明碰撞计数器坏了（这个错真发生过），本方案的任何数字都不可信。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

_CORE = pathlib.Path(__file__).resolve().parent / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))


def _parse(args: argparse.Namespace) -> int:
    from cpu.field_bench import CENTER, compare
    from cpu.potential_field import FieldConfig

    cfg = FieldConfig(cell=args.cell, descend_steps=args.descend,
                      recompute=args.recompute, safety_margin=args.margin,
                      repel_range=args.repel_range)
    goals = {"场中央": CENTER}
    if args.with_no_goal:
        goals["不用目标"] = None
    compare(seeds=tuple(args.seeds), difficulty=args.difficulty,
            goals=goals, configs={"默认": cfg}, weights=tuple(args.weights))
    print("\n读法：『原地不动』是负面对照。本方案的碰撞**没有明显低于它**时，"
          "\n      说明问题不在寻路而在【目标选得不对】—— 见 cpu_field_bench.py --nav。")
    return 0


def _nav(args: argparse.Namespace) -> int:
    from cpu.potential_field import (FieldConfig, FieldMemory, FieldView, plan_field)

    sys.path.insert(0, str(_CORE))
    from cpu.gap_avoid import Rect

    import math

    wy, hh = 266.2, 4.0
    segs = ((0.0, 313.0), (388.6, 640.0))
    rects = tuple(Rect(i, 0.5 * (a + b), wy, 0.0, 0.0, 0.5 * (b - a), hh, 0.0, 0.0)
                  for i, (a, b) in enumerate(segs))

    def view(px: float, py: float, goal):
        return FieldView(player=[px, py, 0, 0, 10.0, 200.0, 1.0], rects=rects,
                         circles=(), field_w=640.0, field_h=480.0, goal=goal)

    def navigate(source, goal, cfg, step=6.0, max_steps=400):
        mem = FieldMemory()
        px, py = source
        traj = [(px, py)]
        for _ in range(max_steps):
            d = plan_field(view(px, py, goal), mem, cfg)
            if math.hypot(d.wx, d.wy) < 1e-9:
                return traj, "STALL"
            px += d.wx * step
            py += d.wy * step
            traj.append((px, py))
            if math.hypot(px - goal[0], py - goal[1]) < 12.0:
                return traj, "ARRIVED"
        return traj, "TIMEOUT"

    def crosses(traj):
        for (x0, y0), (x1, y1) in zip(traj, traj[1:]):
            for s in range(41):
                t = s / 40.0
                x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
                if abs(y - wy) <= hh and any(a <= x <= b for a, b in segs):
                    return True
        return False

    source = (200.0, 105.6)
    goal = (200.0, 450.0)
    print(f"墙 y={wy}，实体段 x∈[0,313]∪[388.6,640]；缺口中心 x=350.8")
    print(f"起点 {source}（实体段正下方） 目标 {goal}（墙后正中）")
    print("必须从缺口绕过去 —— 直线是穿墙的。\n")
    print(f"{'配置':>26} {'结果':>9} {'步数':>5} {'穿墙':>5}   终点")
    print("-" * 70)
    for cell in (8.0, 4.0):
        cfg = FieldConfig(cell=cell, descend_steps=args.descend, recompute=1)
        traj, res = navigate(source, goal, cfg)
        print(f"{f'cell={cell:g} 下降{args.descend}步':>26} {res:>9} {len(traj):>5} "
              f"{str(crosses(traj)):>5}   ({traj[-1][0]:6.1f},{traj[-1][1]:6.1f})")
    print("\n对照：朴素 APF（排斥 + 直线朝目标）会卡在墙前 —— 见 "
          "core/cpu/tests/test_potential_field.py 的"
          "\ntest_straight_line_apf_would_fail_here（那条测试把它锁成了回归）。")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="网格势场 + 波前测地距离方案")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--difficulty", type=float, default=0.4)
    ap.add_argument("--cell", type=float, default=8.0, help="网格边长（世界单位）")
    ap.add_argument("--descend", type=int, default=4, help="沿场下降几步取方向")
    ap.add_argument("--recompute", type=int, default=8, help="每 N 次调用重算一次")
    ap.add_argument("--margin", type=float, default=6.0, help="安全余量")
    ap.add_argument("--repel-range", type=float, default=120.0, dest="repel_range")
    ap.add_argument("--weights", type=float, nargs="+", default=[1.0],
                    help="行程代价权重 λ（可给多个来扫）")
    ap.add_argument("--with-no-goal", action="store_true",
                    help="额外跑一行「不用目标」（负面对照）")
    ap.add_argument("--nav", action="store_true", help="只跑绕墙导航测试")
    args = ap.parse_args(argv)
    return _nav(args) if args.nav else _parse(args)


if __name__ == "__main__":
    raise SystemExit(main())
