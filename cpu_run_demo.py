#!/usr/bin/env python3
"""CPU 决策调用入口 —— 和 ``core/cpu/`` 同名的根目录入口（约定：前缀 = 模块名）。

    python cpu_run_demo.py                     # 缺口墙避障可视化，drive 场景，5 局
    python cpu_run_demo.py gap                 # incoming 场景（墙朝车压过来）
    python cpu_run_demo.py gap --headless -e 5 # 不开窗口，只打印每局统计
    python cpu_run_demo.py gap --scene drive --gap-slide 60 --episodes 20
    python cpu_run_demo.py selftest            # gap_avoid 零依赖自检（不需要 pygame）
    python cpu_run_demo.py paths               # 打印本机解析到的模块路径（排查环境）

实现全在 ``core/cpu/``：

    gap_avoid.py        纯 Python 零依赖的缺口墙避障算法（``plan()`` 是唯一入口，
                        整份可以抄进任何工程；不含 numpy，算子可逐条翻成 Verilog）
    gap_wall_demo.py    该算法的 pygame 可视化 + 5 局回归（本文件转发到它）
    pipeline.py         CPU↔FPGA 帧回路（字节帧进出，换真硬件时上层不变）
    adapters/           ``VisionFrame`` / 仿真池 -> ``gap_avoid.WorldView``

本文件只做参数转发，不含任何算法。``pygame`` 只在 ``gap_wall_demo`` 里按需导入，
所以没有 pygame 也能跑 ``selftest`` / ``paths``。

**``version`` 与 ``bench`` 两个子命令已删除**：它们依赖 ``cpu.decision.build``
（BUILD_ID 指纹）与 ``cpu.benchmark_decision``，而那一整套通用决策架构
（decision/ + benchmark + baseline + tools + game_adapter）已按参赛范围
（只比三种障碍、逐个单独展示、不做混合）整体移除。
"""

from __future__ import annotations

import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

USAGE = __doc__


def _cmd_paths() -> int:
    import bullet_sim
    import cpu
    import cpu.gap_avoid

    for mod in (bullet_sim, cpu, cpu.gap_avoid):
        print(f"{mod.__name__:>14} -> {mod.__file__}")
    print(f"{'core':>14} -> {_CORE}")
    return 0


def _wall_pair(ga, *, y: float, gap_centre: float, gap: float,
               wall_cx: float = 320.0, half_span: float = 180.0,
               half_thick: float = 8.0):
    """把「一块整墙」拆成缺口两侧的两块 —— README 里 ``wall_with_gap`` 的真实构造。

    墙沿 x 展开、法向是 y（``rotation=0``）：整墙本应覆盖
    ``x ∈ [wall_cx ± half_span]``，挖掉 ``[gap_centre ± gap/2]`` 这一段，
    两侧各留一块矩形。所以**缺口中心**才是障碍的"门"。

    注意 ``gap_centre`` 是**场地坐标**（不是相对墙心）—— 算法内部会把重建出的
    孔径按 ``x ∈ [0, field_w]`` 裁剪，用场地坐标才不会像我第一版那样把墙摆到
    场地外面去，白测一轮。
    """
    lo_edge = wall_cx - half_span
    hi_edge = wall_cx + half_span
    g_lo = gap_centre - gap / 2.0
    g_hi = gap_centre + gap / 2.0
    out = []
    for i, (a, b) in enumerate(((lo_edge, g_lo), (g_hi, hi_edge))):
        if b - a <= 0.0:
            continue
        out.append(ga.Rect(ent_id=100 + i, x=(a + b) / 2.0, y=y,
                           vx=0.0, vy=0.0,
                           half_w=(b - a) / 2.0, half_h=half_thick,
                           rotation=0.0, angular_velocity=0.0))
    return out


def _cmd_selftest() -> int:
    """缺口墙算法的零依赖自检 —— 不碰 pygame、不碰仿真器。"""
    import cpu.gap_avoid as ga

    #: 场地 640×480（README 默认），墙横在 y=240，车在 y=120 朝 +y 开
    WALL_Y, CAR_Y, CAR_X = 240.0, 120.0, 320.0

    def run(rects, cfg=None, horizon=2.0):
        view = ga.WorldView(player=[CAR_X, CAR_Y, 0.0, 0.0, 10.0, 200.0, 1.0],
                            rects=rects, dt=1.0 / 120.0, step_index=0,
                            field_w=640.0, field_h=480.0)
        return view, ga.plan(view, ga.GapMemory(),
                             cfg or ga.AlgoConfig(), horizon_remaining=horizon)

    # 1) 空场景：应当 IDLE / 零指令
    _, dec = run([])
    print(f"1) 空场景                  -> {dec.tier:<8} 指令={ga.to_command(dec)}")
    assert dec.tier == "IDLE" and ga.to_command(dec) == (0.0, 0.0)

    cfg = ga.AlgoConfig(forward=(0.0, 1.0), front_back=True, fwd_speed=1.0)

    # 2) 缺口中心在车右侧 60px、宽 200（= 10× 车直径）：横向指令必须朝 +x
    rects = _wall_pair(ga, y=WALL_Y, gap_centre=CAR_X + 60.0, gap=200.0)
    _, dec = run(rects, cfg=cfg)
    print(f"2) 墙(y=240) 缺口 x=380/200 -> {dec.tier:<8} gap={dec.gap:7.2f} "
          f"可用半孔径={dec.usable_half:6.2f} 指令={ga.to_command(dec)}")
    assert dec.gap > 0.0, "缺口应当被重建出来"
    assert dec.wx > 0.0, "缺口在 +x 侧，横向指令必须朝 +x"

    # 3) 缺口挪到车左侧 60px：指令符号必须跟着翻，这是"看得见缺口"的最小证据
    rects = _wall_pair(ga, y=WALL_Y, gap_centre=CAR_X - 60.0, gap=200.0)
    _, dec = run(rects, cfg=cfg)
    print(f"3) 墙(y=240) 缺口 x=260/200 -> {dec.tier:<8} gap={dec.gap:7.2f} "
          f"可用半孔径={dec.usable_half:6.2f} 指令={ga.to_command(dec)}")
    assert dec.gap > 0.0 and dec.wx < 0.0, "缺口在 -x 侧，横向指令必须朝 -x"

    # 4) 缺口 15 < 车直径 20：README 说构造期就该拒绝，算法也绝不能当成门
    rects = _wall_pair(ga, y=WALL_Y, gap_centre=CAR_X, gap=15.0)
    _, dec = run(rects, cfg=cfg)
    print(f"4) 墙(y=240) 缺口 x=320/ 15 -> {dec.tier:<8} gap={dec.gap:7.2f} "
          f"可用半孔径={dec.usable_half:6.2f}")
    assert not (dec.usable_half > 0.0), "15px 的缝放不下 20px 的车，不能算可用"

    print("\n自检通过（4/4）。")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)

    if not args:
        args = ["gap"]

    head, rest = args[0], args[1:]
    if head in ("-h", "--help", "help"):
        print(USAGE)
        return 0
    if head == "gap":
        # 默认给一组「真实车模」参数：车朝静止的墙开过去，缺口会动
        from cpu.gap_wall_demo import main as gap_main

        return gap_main(["--scene", "drive"] + rest if not any(
            a.startswith("--scene") for a in rest) else rest)
    if head == "selftest":
        return _cmd_selftest()
    if head == "paths":
        return _cmd_paths()
    if head == "test":
        import pytest

        return pytest.main([str(_CORE / "cpu" / "tests"), "-q", *rest])
    print(f"未知子命令：{head}\n", file=sys.stderr)
    print(USAGE, file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
