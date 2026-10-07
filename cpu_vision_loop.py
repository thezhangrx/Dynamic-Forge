#!/usr/bin/env python3
"""摄像头闭环的**调用入口**：平台当真实世界、视觉当唯一传感器、core/cpu 决策。

真正的实现（合成/真相机、单局流程、场景构建）都在 ``core/harness/loop.py``，
本文件只做三件事：解析参数、按策略跑若干局、打印验收小结。

用法
----
    python cpu_vision_loop.py --synthetic --episodes 5 --seconds 16
    python cpu_vision_loop.py --synthetic --episodes 3 --seconds 16 --policy hold
    python cpu_vision_loop.py --camera --calibration data/vision/calibration.json

``--policy hold`` 是**阴性对照**（角色不动）：用来证明碰撞指标是活的。
任何"无碰撞率"结论都必须和它一起看，理由见 docs/vision/closed_loop.md §4.2。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from bullet_sim.entities.player import P_Y                       # noqa: E402
from cpu.gap_avoid import AlgoConfig                             # noqa: E402
from harness.loop import (                                       # noqa: E402
    FIELD,
    SIM_DT,
    RealCamera,
    SyntheticCamera,
    _classifier,
    build_env,
    run_episode,
)
from vision.calibration import FieldCalibration                  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="摄像头 -> 视觉 -> core/cpu -> 角色（闭环）")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--synthetic", action="store_true",
                     help="离屏渲染 + 已知单应当相机（默认，可复现）")
    src.add_argument("--camera", action="store_true", help="真开摄像头")
    ap.add_argument("--calibration", type=pathlib.Path,
                    default=pathlib.Path("data/vision/calibration.json"))
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=16.0,
                    help="每局时长。**别低于 ~12s**：角色在 y≈73，墙从 y=480 以 40 单位/秒"
                         "下来，第一堵墙要 (480-73)/40 ≈ 10.2s 才够得着角色 —— "
                         "比这短的局**不可能发生碰撞**，'0 碰撞'是空的")
    ap.add_argument("--vary-gap", type=float, default=0.06,
                    help="每个 seed 把缺口位置挪动多少（场宽比例）。**必须>0**："
                         "wall_with_gap 的场景本身没有随机性，只用 seed 换种子会得到"
                         "**完全相同**的局（实测 seed=1 与 seed=2 的轨迹逐帧一致），"
                         "那样'N 个 seed 全 0 碰撞'其实只跑了一局")
    ap.add_argument("--policy", default="gap_avoid", choices=("gap_avoid", "hold"),
                    help="gap_avoid=真链路（视觉→决策）；"
                         "hold=**阴性对照**，角色不动，用来证明碰撞指标是活的")
    ap.add_argument("--gap-width", type=float, default=60.0)
    ap.add_argument("--obstacle-speed", type=float, default=40.0)
    ap.add_argument("--gap-motion", default="static", choices=("static", "sweep"),
                    help="缺口动不动。**默认 static**：sweep 会让缺口自己游走到角色身上，"
                         "于是'角色根本没动'也能 0 碰撞，测不出控制有没有生效")
    ap.add_argument("--gap-position", type=float, default=0.28,
                    help="缺口在场地宽度上的位置 0~1（默认 0.28：**不要用 0.5**，"
                         "角色出生在中线，缺口也在中线就测不出控制有没有生效）")
    ap.add_argument("--margin", type=float, default=6.0)
    ap.add_argument("--forward", type=float, nargs=2, default=(0.0, 1.0))
    ap.add_argument("--control-hz", type=float, default=30.0,
                    help="每秒看几次摄像头并决策（默认 30；物理仍按 120Hz 步进）")
    ap.add_argument("--blur", type=int, default=1,
                    help="合成相机的模糊核（1=清晰；真实相机大约是 3~5）")
    ap.add_argument("--noise", type=float, default=0.0,
                    help="合成相机的传感器噪声标准差（灰度级）")
    ap.add_argument("--classifier", default="absolute",
                    choices=("absolute", "adaptive"),
                    help="像素分类层：absolute=原绝对灰度阈值（默认，快）；"
                         "adaptive=自适应+弃权（抗光照变化；纯 Python 分类层约慢 10 倍，"
                         "整条闭环约慢 3 倍）。两者在标准渲染场景上逐像素等价，"
                         "见 docs/vision/adaptive.md")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    use_camera = bool(args.camera)
    camera = (RealCamera(args.device, args.width, args.height) if use_camera
              else SyntheticCamera(blur=args.blur, noise=args.noise))
    cfg = AlgoConfig(safety_margin=args.margin, forward=tuple(args.forward))

    probe = build_env(args.seed, args.gap_width, args.seconds,
                      args.obstacle_speed, args.gap_position, args.gap_motion)
    if use_camera:
        if not args.calibration.exists():
            raise SystemExit(f"没有标定文件 {args.calibration}；先跑 vision_calibrate.py")
        calib = FieldCalibration.load(args.calibration)
        print(f"相机     : /dev/video{args.device}（真实摄像头）")
    else:
        calib = camera.calibrate(probe)
        print(f"相机     : 合成（离屏渲染 + 已知单应）")
    print(f"标定     : 单位={calib.unit} 覆盖={calib.marker_coverage * 100:.0f}%")
    print(f"场景     : wall_with_gap  缺口={args.gap_width:.0f} "
          f"@{args.gap_position:.2f}·场宽({args.gap_motion})  "
          f"墙速={args.obstacle_speed:.0f}  {args.seconds:.0f}s  策略={args.policy}")
    if args.blur > 1 or args.noise > 0:
        print(f"图像退化 : 模糊核={args.blur}  噪声={args.noise}")
        print(f"分类层   : {args.classifier}")

    # 局时长够不够"撞一次"？先算清楚，否则整轮验收是空的。
    # 角色出生在 field_h*0.15（见 ObstacleScenario.player_spawn），墙在场地顶端
    # 生成后以 obstacle_speed 往下走，所以第一堵墙够到角色需要下面这么多秒。
    # 从 env 里**读**出生位置而不是写死常量：写死会在场景参数改动后悄悄失效。
    from bullet_sim.entities.player import P_Y
    spawn_y = float(probe.world.player[P_Y])
    t_first = (FIELD[1] - spawn_y) / max(1e-6, args.obstacle_speed)
    if args.seconds < t_first:
        print(f"\n⚠ 本局 {args.seconds:.0f}s < 第一堵墙够到角色所需的 {t_first:.1f}s"
              f"（角色 y={spawn_y:.0f}，墙从 y={FIELD[1]:.0f} 以 {args.obstacle_speed:.0f}/s 下来）"
              f" —— **这一轮不可能发生碰撞**，报出来的 0 碰撞不构成任何证据。\n"
              f"  要看结果请用 --seconds ≥ {t_first + 4:.0f}。", file=sys.stderr)

    steps = int(args.seconds / SIM_DT)
    rows = []
    for ep in range(int(args.episodes)):
        seed = args.seed + ep
        # **必须动一点场景参数**：wall_with_gap 本身没有随机性，只换 seed
        # 会得到一模一样的局（实测过），N 个 seed 就等于跑了 1 局。
        gap_pos = args.gap_position + args.vary_gap * ((ep % 5) - 2)
        gap_pos = min(0.95, max(0.05, gap_pos))
        env = build_env(seed, args.gap_width, args.seconds,
                        args.obstacle_speed, gap_pos, args.gap_motion)
        t0 = time.time()
        st = run_episode(env, camera, calib, steps=steps, cfg=cfg,
                         control_hz=args.control_hz, verbose=args.verbose,
                         classifier=_classifier(args.classifier), policy=args.policy)
        rows.append((seed, st, time.time() - t0))
        hit = "碰撞 N=%d(首次 %.1fs)" % (st["entered"], st["survived_s"]) \
            if st["ever_collided"] else "无碰撞"
        print(f"  第{ep + 1}局 seed={seed} 缺口@{gap_pos:.2f}: 步数={st['steps']} "
              f"决策={st['decided']} 被拒={st['rejected']} 越界={st['out_of_field']} "
              f"**{hit}** 末档={st['final_tier']} 用时={time.time() - t0:.1f}s")
        if st["tiers"]:
            top = sorted(st["tiers"].items(), key=lambda kv: -kv[1])[:4]
            print(f"    档位分布: " + "  ".join(f"{k}={v}" for k, v in top))
        if st["start"]:
            print(f"    角色（视觉估计）: 起点 ({st['start'][0]:.0f},{st['start'][1]:.0f}) -> "
                  f"终点 ({st['end'][0]:.0f},{st['end'][1]:.0f})  最高 y={st['max_y']:.0f}"
                  f"  **前向进展 {st['forward_gain']:.0f} 单位**")
        if st.get("gap_in_err"):
            e = sorted(st["gap_in_err"])
            print(f"    决策输入误差（视觉 vs 真值）: 缺口 {e[len(e) // 2]:.2f} 场地单位", end="")
        if st.get("player_in_err"):
            e = sorted(st["player_in_err"])
            print(f" | 角色 {e[len(e) // 2]:.2f} 场地单位", end="")
        if st.get("gap_in_err") or st.get("player_in_err"):
            print()
        if args.verbose:
            for row in st["trace"][::max(1, len(st["trace"]) // 12)]:
                print("      t=%5.2f %-7s pos=(%4d,%4d) gap=%-6s dir=%s" % row)

    if use_camera:
        camera.close()
    if any(r[1]["out_of_field"] for r in rows):
        print("\n⚠ 有帧越界 —— 标定很可能已经失效（窗口/相机/屏幕动过），"
              "这一局的结果不可信", file=sys.stderr)
    ok = [r for r in rows if r[1]["decided"] > 0 or args.policy == "hold"]

    # -- 验收小结 ---------------------------------------------------------
    n = len(rows)
    clean = sum(1 for r in rows if not r[1]["ever_collided"])
    print(f"\n小结: {len(ok)}/{n} 局跑通；**无碰撞 {clean}/{n} = {100.0 * clean / max(1, n):.0f}%**")
    surv = [r[1]["survived_s"] for r in rows if r[1]["survived_s"] is not None]
    if surv:
        print(f"      首次碰撞时间: 中位 {sorted(surv)[len(surv) // 2]:.1f}s "
              f"最早 {min(surv):.1f}s（越晚越好）")
    if args.policy == "hold" and clean == n:
        print("      ✗ **阴性对照没有撞到 —— 碰撞指标很可能是坏的**"
              "（角色一动不动却被判无碰撞）。先修计数器，再看 gap_avoid 的数字。",
              file=sys.stderr)
        return 1
    return 0 if len(ok) == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
