#!/usr/bin/env python3
"""**板子链路闭环**的调用入口：世界 → 相机 → PL(视觉) → 几何帧 → 决策 → 世界。

    python board_loop_run.py --episodes 3 --seconds 16
    python board_loop_run.py --episodes 3 --seconds 16 --compare
    python board_loop_run.py --episodes 1 --seconds 16 --policy hold   # 阴性对照

与 `cpu_vision_loop.py`（Python 全流程）的关系
---------------------------------------------
**同一个闭环，只换 ④ 那一步**：

    cpu_vision_loop.py :  相机 → core/vision(Python) → 单应 → 决策 → 世界
    board_loop_run.py  :  相机 → **PL 顶替物** → 几何帧 → 单应 → 决策 → 世界

世界的推进、控制频率、碰撞统计、阴性对照全部复用 `core/harness/loop.py`，
所以两边的指标**可以直接比**。

`--compare` 会把两种视觉跑在同一批 seed 上、并排打印 —— 那是"把 Python 全流程
换成板子链路之后，行为变了没有"最直接的证据。

**本机的"PL"是 `core/fpga/ref/vision_ref.c` 编译出来的共享库**，
它与 Verilog RTL **逐位一致**（620 帧、4.3 亿次比对、零差异）。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from cpu.gap_avoid import AlgoConfig                          # noqa: E402
from harness.board_loop import describe_backend, run_board_episode   # noqa: E402
from harness.board_sim import compile_ref, RefPl              # noqa: E402
from harness.loop import (                                    # noqa: E402
    SIM_DT,
    SyntheticCamera,
    build_env,
    run_episode,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="板子链路闭环：相机 → PL(视觉) → 几何帧 → 决策（对照 Python 全流程）")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=16.0,
                    help="每局时长。**别低于 ~12s**：第一堵墙要 (480-73)/40 ≈ 10.2s "
                         "才够得着角色，更短的局不可能碰撞，'0 碰撞'是空的")
    ap.add_argument("--vary-gap", type=float, default=0.06,
                    help="每个 seed 挪动缺口位置（场宽比例）。必须 >0，否则每局完全一样")
    ap.add_argument("--policy", default="gap_avoid", choices=("gap_avoid", "hold"),
                    help="hold = 阴性对照（角色不动），用来证明碰撞指标是活的")
    ap.add_argument("--gap-width", type=float, default=60.0)
    ap.add_argument("--gap-position", type=float, default=0.28,
                    help="缺口位置（默认 0.28；**不要用 0.5**，角色出生在中线）")
    ap.add_argument("--gap-motion", default="static", choices=("static", "sweep"))
    ap.add_argument("--obstacle-speed", type=float, default=40.0)
    ap.add_argument("--control-hz", type=float, default=30.0)
    ap.add_argument("--margin", type=float, default=6.0)
    ap.add_argument("--forward", type=float, nargs=2, default=(0.0, 1.0))
    ap.add_argument("--blur", type=int, default=1)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--classifier", default="absolute",
                    choices=("absolute", "adaptive"))
    ap.add_argument("--compare", action="store_true",
                    help="**同时跑 Python 视觉**，并排对比 —— 这是"
                         "'换成板子链路后行为变了没有'的直接证据")
    ap.add_argument("--rebuild", action="store_true", help="强制重编 C 参考模型")
    ap.add_argument("--verbose", action="store_true")
    return ap.parse_args(argv)


def _one_episode(args: argparse.Namespace, seed: int, calib, pl, *, python_vision: bool):
    """跑一局。``python_vision=True`` 时走原来的 Python 视觉链（对照组）。"""
    # 每个 seed 把缺口挪一点 —— wall_with_gap 场景本身没有随机性，
    # 只换 seed 会得到**完全相同**的局（实测 seed=1 与 2 逐帧一致）。
    #
    # ⚠ 这里**必须**挡住越界的缺口位置。`pos = gap_position + vary_gap*(seed-1)`
    #   是个无界线性式：`--vary-gap` 给大一点（比如 0.6），seed 0 就是
    #   0.28-0.6 = **-0.32**，seed 3 起又 > 1 —— 缺口被推到画面外，整局变成
    #   "墙上根本没有缺口"。那时两边当然都报 NO_GAP 并撞墙，"两者一致"照样
    #   成立，但**验证的已经不是缺口穿越**，指标全是空的。
    #   实测：`--vary-gap 0.6 --episodes 8` 里只有 seed 1/2 的缺口在场内，
    #   于是 6/8 局撞墙 —— 看起来像视觉坏了，其实是我把参数传错了。
    pos = args.gap_position + args.vary_gap * (seed - 1)
    if not (0.0 <= pos <= 1.0):
        # 给一条**能用**的建议，而不是复述那个越界的区间。
        # seed 0..N-1 的位置落在 [gap_position - vary, gap_position + vary*(N-1)]，
        # 两头都要留在 [0,1]：
        #   下界 gap_position - vary >= 0        → vary <= gap_position
        #   上界 gap_position + vary*(N-1) <= 1  → vary <= (1-gap_position)/(N-1)
        # 两边取小。注意**不是**对称的 (N-1)/2 —— 位置是线性的、单向的。
        n = max(int(getattr(args, "episodes", 1)), 2)
        safe = min(args.gap_position,
                   (1.0 - args.gap_position) / (n - 1))
        raise SystemExit(
            f"缺口位置越界：seed={seed} → gap_position={pos:.3f}，不在 [0,1]。\n"
            f"  gap_position={args.gap_position} + vary_gap={args.vary_gap}*(seed-1)\n"
            f"  本参数下 {n} 局的缺口位置是 "
            f"{args.gap_position - args.vary_gap:.2f} .. "
            f"{args.gap_position + args.vary_gap * (n - 1):.2f}。\n"
            f"  要让 {n} 局全部落在场内，--vary-gap 必须 <= {safe:.4f}"
            f"（默认 0.06）。缺口出画面时整局没有缺口可穿，"
            f"'两边都撞墙'是空结论。")
    env = build_env(seed, args.gap_width, args.seconds,
                    args.obstacle_speed, pos, args.gap_motion)
    camera = SyntheticCamera(blur=args.blur, noise=args.noise)
    cfg = AlgoConfig(safety_margin=args.margin, forward=tuple(args.forward))
    steps = int(round(args.seconds / SIM_DT))
    if python_vision:
        return run_episode(env, camera, calib, steps=steps, cfg=cfg,
                           control_hz=args.control_hz, verbose=args.verbose,
                           classifier=None, policy=args.policy)
    return run_board_episode(env, camera, calib, steps=steps, cfg=cfg, pl=pl,
                             control_hz=args.control_hz, verbose=args.verbose,
                             policy=args.policy)


def _brief(st: dict) -> str:
    coll = "碰撞" if st.get("ever_collided") else "无碰撞"
    surv = st.get("survived_s")
    surv_s = f"{surv:.1f}s" if surv is not None else "—"
    tiers = " ".join(f"{k}={v}" for k, v in sorted(st.get("tiers", {}).items()))
    return (f"{coll}  决策={st.get('decided', 0)}  被拒={st.get('rejected', 0)}  "
            f"存活={surv_s}  档位: {tiers or '—'}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    so = compile_ref(force=args.rebuild)
    pl = RefPl(so)
    print("=" * 74)
    print("板子链路闭环（本机模拟）")
    print(f"  PL 顶替物 : {describe_backend(pl)}")
    print("  几何帧    : v3 线协议字节（与板上同一份格式）")
    print("  单应/决策 : 与 Python 全流程**同一份代码**")
    if args.compare:
        print("  对照组    : Python 视觉链（core/vision）")
    print("=" * 74)
    print(f"C 参考模型: {so}")
    print(f"场景      : wall_with_gap 缺口={args.gap_width:.0f} "
          f"@{args.gap_position:.2f}·场宽({args.gap_motion})  墙速={args.obstacle_speed:.0f}  "
          f"{args.seconds:.0f}s  策略={args.policy}")

    # 标定：**用合成相机自己走一遍真机那条标定流程**（四角 ArUco）。
    # 不拿 data/vision/calibration.json —— 那份的 image_width 是 0，
    # 实测和别的录像对不上（见 board_sim.calibrate_from_video 的文档）。
    probe = build_env(args.seed, args.gap_width, args.seconds,
                      args.obstacle_speed, args.gap_position, args.gap_motion)
    camera0 = SyntheticCamera(blur=args.blur, noise=args.noise)
    calib = camera0.calibrate(probe)
    print(f"标定      : 合成相机自标定  单位={calib.unit}  "
          f"覆盖={calib.marker_coverage * 100:.0f}%")

    board_stats, py_stats = [], []
    for ep in range(args.episodes):
        seed = args.seed + ep
        st = _one_episode(args, seed, calib, pl, python_vision=False)
        board_stats.append(st)
        line = f"局{ep + 1} seed={seed}  **板子链路**  {_brief(st)}"
        if st.get("pl"):
            p = st["pl"]
            line += (f"\n              PL: {p['frames']} 帧  "
                     f"角色命中 {p['player_valid_rate'] * 100:.0f}%  "
                     f"障碍中位 {p['n_obstacles_median']:.0f}  "
                     f"缺口中位 {p['n_gaps_median']:.0f}  "
                     f"帧长中位 {p['frame_bytes_median']:.0f}B  "
                     f"耗时中位 {p['pl_ms_median']:.2f}ms")
        print(line)
        if args.compare:
            st2 = _one_episode(args, seed, calib, pl, python_vision=True)
            py_stats.append(st2)
            print(f"局{ep + 1} seed={seed}  **Python 视觉**  {_brief(st2)}")

    n = len(board_stats)
    nocol = sum(1 for s in board_stats if not s.get("ever_collided"))
    print("-" * 74)
    print(f"板子链路: 无碰撞 {nocol}/{n} = {100.0 * nocol / n:.0f}%")
    if args.compare and py_stats:
        nocol2 = sum(1 for s in py_stats if not s.get("ever_collided"))
        same = sum(1 for a, b in zip(board_stats, py_stats)
                   if bool(a.get("ever_collided")) == bool(b.get("ever_collided")))
        print(f"Python  : 无碰撞 {nocol2}/{n} = {100.0 * nocol2 / n:.0f}%")
        print(f"两者碰撞结论一致 {same}/{n}")
        print("  ⚠ 两者**不要求逐位相同**：C 参考模型是 RTL 的对手，"
              "与 Python 视觉是「同一语义、不同实现」，")
        print("     差异预算见 core/fpga/docs/交接文档.md（缺口中心 ≤0.1 场地单位）。")
    if args.policy == "hold":
        print("⚠ 这是**阴性对照**（角色不动）：0 碰撞才说明碰撞计数器坏了。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
