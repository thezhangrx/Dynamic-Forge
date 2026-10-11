"""势场可视化的回归测试。

**全部离屏**（``SDL_VIDEODRIVER=dummy``），不需要窗口、不需要人看。
这一点是刻意的：可视化的正确性必须能用断言表达，
否则"窗口开了没报错"就会被当成"可视化是对的"。

锁住的几条都对应实际踩到的：

1. **叠加层必须画在 ``_draw_hud`` 里** —— ``render()`` 在 ``_draw_hud`` 之后
   就取像素返回了（``rgb_array`` 模式），画在 ``render()`` 之后根本没机会进画面。
2. **``show_text`` 默认是 ``False``**，而 ``render()`` 只在
   ``show_hud and show_text`` 时才调 ``_draw_hud`` —— 不强制打开就没有叠加层。
3. **场的 alpha 不能太低** —— 95 的时候场被世界的暗背景糊成一片深绿/深红，
   看不出漏斗；这是第一次渲出来就发现的问题。
4. **障碍和玩家要在场之后重画一遍** —— 否则半透明的场把它们盖住，
   而"色块对应哪堵墙"正是要看的东西。
5. **``DroidSansFallbackFull`` 缺拉丁字母** —— 用它渲 "plan()" 会出六个方块。
"""

from __future__ import annotations

import math
import os

import pytest

pygame = pytest.importorskip("pygame")

from cpu.field_demo import (FieldOverlay, draw_hud, draw_legend, field_overlay,
                            make_renderer, render_frame, run_demo, save_png)
from cpu.gap_avoid import GapMemory
from cpu.potential_field import FieldConfig, FieldMemory


@pytest.fixture(autouse=True)
def _headless() -> None:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


def _world(seed: int = 1, warm: int = 200, key: str = "wall_with_gap"):
    from bullet_sim.obstacles.scenario import ObstacleScenario
    from bullet_sim.simulator.world import World

    sc = ObstacleScenario(name=key, seed=seed, obstacles=[{"type": key}],
                          require_valid=False)
    built = sc.build(validate=False)
    world = World(built.spec, codec="velocity")
    for _ in range(warm):
        world.step((0.0, 0.0))
    return world


def _renderer(mode: str = "rgb_array"):
    return make_renderer(mode=mode, window_size=(640, 480), show_hitboxes=False,
                         show_world_axes=False, show_fiducials=False)


# ---------------------------------------------------------------------------
# 1. 叠加层真的进了像素
# ---------------------------------------------------------------------------
def test_render_frame_returns_pixels_and_overlay() -> None:
    world = _world()
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    arr, ov, cmd = render_frame(world, FieldMemory(), cfg, _renderer(),
                                GapMemory(), goal_mode="center", step=200)
    assert arr.shape == (480, 640, 3), f"尺寸不对：{arr.shape}"
    assert ov.field is not None, "没有场 —— 叠加层没画上"
    nx, ny, cell, field = ov.field
    assert (nx, ny) == (40, 30) and cell == 16.0
    assert len(field) == nx * ny


def test_overlay_is_actually_drawn_not_just_computed() -> None:
    """**关键**：算出来的场必须真的改了像素。

    只看 ``arr`` 的形状是查不出"叠加层没画"的 —— 底下的世界照样是一张
    合法图片。所以这里拿"有叠加层"和"没叠加层"两张图比，必须不同。
    """
    import numpy as np

    world = _world()
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)

    r1 = _renderer()
    r1.overlay = None
    plain = r1.render(world)

    r2 = _renderer()
    arr, ov, _ = render_frame(world, FieldMemory(), cfg, r2, GapMemory(),
                              goal_mode="center", step=200)
    assert not np.array_equal(plain, arr), "加了叠加层但像素没变 —— 没画上去"


def test_show_text_is_forced_on() -> None:
    """``show_text`` 默认 False，而 ``_draw_hud`` 只在它为真时才被调用。

    不强制打开 → ``render()`` 根本不调 ``_draw_hud`` → 叠加层永远画不出来。
    """
    r = _renderer()
    assert r.show_text is True
    assert r.show_hud is True
    assert r.help_seconds == 0.0, "大块 CONTROLS 面板会挡住场中央"


# ---------------------------------------------------------------------------
# 2. 场/路点/瞄准点是自洽的
# ---------------------------------------------------------------------------
def test_waypoint_and_plan_aim_mostly_agree() -> None:
    """``plan()`` 的缺口瞄准点，和势场沿场下降选出的路点，应当基本一致。

    差得远说明两边至少有一个不对。这条是**跨实现的一致性检查** ——
    它同时约束 `gap_avoid.plan` 和 `potential_field`，任何一边改坏了都会挂。
    容差取 2 个格（``descend_steps`` 决定路点会往前探，不会完全重合）。
    """
    world = _world(warm=430)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    arr, ov, _ = render_frame(world, FieldMemory(), cfg, _renderer(),
                              GapMemory(), goal_mode="gap", step=430)
    assert ov.aim is not None, "plan() 在这帧给不出缺口"
    assert ov.waypoint is not None, "势场没选出路点"
    d = math.hypot(ov.waypoint[0] - ov.aim[0], ov.waypoint[1] - ov.aim[1])
    assert d <= 3.0 * cfg.cell, f"路点 {ov.waypoint} 与瞄准点 {ov.aim} 差 {d:.1f}"


def test_field_gets_warmer_beyond_the_wall() -> None:
    """越过墙以后场值应当明显变大 —— 这是"波前没从墙里漏过去"的可视化判据。

    如果墙两边颜色差不多，说明测地距离没把墙算进去（那就退化成直线距离了）。
    """
    world = _world(warm=430)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    ov = field_overlay(_field_view(world, goal=(320.0, 200.0)),
                       FieldMemory(), cfg)
    nx, ny, cell, field = ov.field
    # 玩家那一带（世界 y≈72）vs 场地顶部（世界 y≈470）
    j_low = int(72.0 / cell)
    j_high = int(470.0 / cell)
    i_mid = int(320.0 / cell)
    low = field[i_mid * ny + j_low]
    high = field[i_mid * ny + j_high]
    assert math.isfinite(low) and math.isfinite(high)
    assert high > low * 1.5, (
        f"墙上方 {high:.0f} 没有明显高于墙下方 {low:.0f} —— "
        "测地距离可能没把墙算进去")


def _field_view(world, goal):
    from cpu.potential_field import field_from_world
    return field_from_world(world, goal=goal, step_index=0)


# ---------------------------------------------------------------------------
# 3. 诊断文本与图例
# ---------------------------------------------------------------------------
def test_hud_rows_mention_the_parameters_you_tune() -> None:
    """调参时最想知道的那几个量必须在诊断面板里。"""
    world = _world()
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1,
                      travel_weight=0.5, repel_range=90.0)
    ov = field_overlay(_field_view(world, goal=(320.0, 240.0)), FieldMemory(), cfg)
    text = "\n".join(ov.rows)
    for token in ("cell", "16", "0.5", "90", "cost", "fallback", "recompute"):
        assert token in text, f"诊断面板缺 {token!r}：\n{text}"


def test_draw_hud_and_legend_do_not_crash_with_cjk() -> None:
    """中文必须能渲（pygame 默认字体没有 CJK，会出一堆方块但不该崩）。"""
    pygame.init()
    surf = pygame.Surface((640, 480))
    ov = FieldOverlay(rows=("目标 = plan() 的缺口", "格数 1200", "λ 1"),
                      waypoint=(100.0, 100.0), aim=(120.0, 120.0))
    draw_hud(surf, ov)
    draw_legend(surf, ov)
    pygame.quit()


# ---------------------------------------------------------------------------
# 4. 存 PNG（"可视化对不对"要能被别人复核）
# ---------------------------------------------------------------------------
def test_save_png_writes_a_readable_file(tmp_path) -> None:
    world = _world()
    arr, _ov, _cmd = render_frame(world, FieldMemory(),
                                  FieldConfig(cell=16.0, descend_steps=4),
                                  _renderer(), GapMemory(), goal_mode="center")
    out = tmp_path / "field.png"
    save_png(out, arr)
    assert out.exists() and out.stat().st_size > 2000
    head = out.read_bytes()[:8]
    assert head == b"\x89PNG\r\n\x1a\n", "写出来的不是 PNG"


# ---------------------------------------------------------------------------
# 5. 导入约定（`python core/cpu/field_demo.py` 曾经报过 ImportError）
# ---------------------------------------------------------------------------
def test_no_relative_imports_in_cpu_package() -> None:
    """**包内不许用相对导入。**

    相对导入只在"以包的身份导入"时成立。直接跑文件时 Python 把它当成
    ``__main__``、没有父包，于是
    ``ImportError: attempted relative import with no known parent package``。

    这条是真踩过的：``field_demo.py`` 一开始写的是 ``from .gap_avoid import ...``，
    用户按最自然的方式 ``python3 core/cpu/field_demo.py`` 就直接报错。
    仓库里其他文件（``__init__.py``、``pipeline.py``）用的都是绝对导入
    （``from cpu.xxx import ...``）。
    """
    import ast
    import pathlib

    root = pathlib.Path(__file__).resolve().parents[1]      # core/cpu/
    bad: list[str] = []
    for f in sorted(root.glob("*.py")):
        tree = ast.parse(f.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and (node.level or 0) > 0:
                bad.append(f"{f.name}:{node.lineno}  from {'.' * node.level}{node.module}")
    assert not bad, (
        "core/cpu/ 里出现了相对导入，直接跑文件会报 "
        f"'attempted relative import with no known parent package'：{bad}")


def test_module_file_is_runnable_as_a_script() -> None:
    """``python core/cpu/field_demo.py --help`` 必须能跑。

    覆盖三件事：① 没有相对导入的 ImportError；② 模块文件自己有 argparse
    （所以 ``--help`` 不会掉进 demo 里跑一整局）；③ 退出码为 0。
    """
    import pathlib
    import subprocess
    import sys as _sys

    script = pathlib.Path(__file__).resolve().parents[1] / "field_demo.py"
    env = dict(os.environ, SDL_VIDEODRIVER="dummy")
    r = subprocess.run([_sys.executable, str(script), "--help"],
                       capture_output=True, text=True, timeout=120, env=env)
    assert r.returncode == 0, f"退出码 {r.returncode}\n{r.stderr[-900:]}"
    assert "usage:" in r.stdout, f"--help 没打出用法：\n{r.stdout[:400]}"


def test_root_script_and_module_agree() -> None:
    """根脚本与模块文件必须走**同一份** argparse 默认值。

    两条入口迟早会漂移 —— 除非它们共用一个 ``build_parser()``。
    """
    from cpu.field_demo import build_parser

    a = build_parser().parse_args([])
    b = build_parser().parse_args([])
    assert vars(a) == vars(b)
    assert a.goal == "auto" and a.cell == 16.0


# ---------------------------------------------------------------------------
# 6. 另外两种障碍类型（移动方块 / 弹幕）
# ---------------------------------------------------------------------------
def test_predict_ghosts_covers_both_shapes() -> None:
    """幽灵必须**两种形状都覆盖** —— 弹幕是圆、方块/墙是矩形。

    每个障碍每个时刻一个幽灵；位置按匀速直行 + 匀角速度外推。
    """
    import math as _m

    from cpu.field_demo import predict_ghosts
    from cpu.potential_field import Circle, FieldView
    from cpu.gap_avoid import Rect

    v = FieldView(player=[0.0, 0.0, 0, 0, 10.0, 200.0, 1.0],
                  rects=(Rect(1, 0.0, 0.0, 10.0, 0.0, 4.0, 2.0, 0.0, 1.0),),
                  circles=(Circle(2, 0.0, 0.0, 3.0, 0.0, -20.0),),
                  field_w=640.0, field_h=480.0, goal=None)
    gs = predict_ghosts(v, horizons=(0.1, 0.2))
    assert len(gs) == 4, f"1 矩形 + 1 圆，2 个时刻 → 4 个幽灵，实得 {len(gs)}"
    kinds = sorted(g[1] for g in gs)
    assert kinds == ["circle", "circle", "rect", "rect"]
    # 矩形：x 方向 10/s，0.2 秒后应当在 x=2.0；角度也推进
    rects = [g for g in gs if g[1] == "rect"]
    assert _m.isclose(max(r[2] for r in rects), 2.0), rects
    assert _m.isclose(max(r[6] for r in rects), 0.2), rects
    # 圆：y 方向 -20/s
    circles = [g for g in gs if g[1] == "circle"]
    assert _m.isclose(min(c[3] for c in circles), -4.0), circles


def test_aim_blind_when_no_rects() -> None:
    """纯圆场景（弹幕）必须标出"``plan()`` 给不出缺口"。

    实测：``small_obstacles`` 和 ``moving_block`` 上 ``plan()`` **100%**
    给不出可用的缺口瞄准点（它只认"带缺口的墙"，弹幕它看不见、单块没有缺口）。
    不标出来的话，图上那个黄叉会被当成真目标。
    """
    world = _world(key="small_obstacles", warm=200)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    ov = field_overlay(_field_view(world, goal=(320.0, 240.0)),
                       FieldMemory(), cfg, aim=None, label="弹幕")
    assert ov.aim_blind is True
    assert ov.mix[1] > 0 and ov.mix[0] == 0, f"应当是纯圆场景：{ov.mix}"
    assert any("给不出缺口" in r for r in ov.rows), ov.rows
    assert any("弹幕" in r or "圆" in r for r in ov.rows)


def test_aim_not_blind_for_wall_with_gap() -> None:
    """反面对照：带缺口的墙上 ``plan()`` 是能给出缺口的，不该误报。"""
    world = _world(key="wall_with_gap", warm=430)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    from cpu.gap_avoid import AlgoConfig, plan
    from cpu.adapters.sim_adapter import view_from_world
    dec = plan(view_from_world(world), GapMemory(), AlgoConfig(),
               horizon_remaining=1e9)
    aim = (float(dec.aim_world[0]), float(dec.aim_world[1]))
    assert all(math.isfinite(v) for v in aim), "墙场景上应当能给出缺口"
    ov = field_overlay(_field_view(world, goal=aim), FieldMemory(), cfg, aim=aim)
    assert ov.aim_blind is False
    assert not any("给不出缺口" in r for r in ov.rows)


@pytest.mark.parametrize("key", ["wall_with_gap", "moving_block", "small_obstacles"])
def test_all_three_types_render(key: str) -> None:
    """三种障碍类型都要能渲出来，而且幽灵图层按类型出现。

    带缺口的墙：没有幽灵也能看（它不动或只扫缺口）。
    移动方块 / 弹幕：**必须**有幽灵，否则看不出运动。
    """
    import numpy as np

    world = _world(key=key, warm=200)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    arr, ov, _cmd = render_frame(world, FieldMemory(), cfg, _renderer(),
                                 GapMemory(), goal_mode="center", step=200)
    assert arr.shape == (480, 640, 3)
    assert ov.mix[0] + ov.mix[1] > 0, f"{key} 场景里没有障碍？"
    if key == "moving_block":
        assert all(g[1] == "rect" for g in ov.ghosts), "方块场景应当只有矩形幽灵"
        assert len(ov.ghosts) > 0
    elif key == "small_obstacles":
        assert all(g[1] == "circle" for g in ov.ghosts), "弹幕场景应当只有圆形幽灵"
        assert len(ov.ghosts) > 0
    assert arr.any(), "渲出来是全黑"


def test_no_ghosts_flag_removes_the_layer() -> None:
    world = _world(key="small_obstacles", warm=200)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    _a, ov, _c = render_frame(world, FieldMemory(), cfg, _renderer(), GapMemory(),
                              goal_mode="center", ghost_horizon=0.0)
    assert ov.ghosts == (), "--no-ghosts 时不该有幽灵"


# ---------------------------------------------------------------------------
# 7. 时间维 + 目标来源（两项都接进可视化后）
# ---------------------------------------------------------------------------
def test_resolve_goal_explicit_wins_over_policy() -> None:
    """**调用方显式给的 goal 绝对优先** —— 策略不该偷偷换掉它。

    一开始把默认策略写成 ``auto``，于是显式传 ``goal=(200,450)`` 被
    ``plan()`` 的缺口覆盖，绕墙导航测试全挂。现在 ``plan_field`` 根本不调
    ``resolve_goal``，而 ``resolve_goal`` 也是"有 goal 就直接返回"。
    """
    from cpu.potential_field import FieldConfig, FieldMemory, resolve_goal

    world = _world(key="wall_with_gap", warm=430)
    v = _field_view(world, goal=(200.0, 450.0))
    for pol in ("auto", "plan", "view", "safest", "none"):
        goal, src, _aim = resolve_goal(v, FieldConfig(goal_policy=pol), FieldMemory())
        assert goal == (200.0, 450.0), f"策略 {pol} 换掉了显式目标：{goal}"
        assert src == "view"


def test_resolve_goal_falls_back_to_target_when_plan_has_no_gap() -> None:
    """弹幕/方块上 ``plan()`` 100% 给不出缺口 → ``auto`` 必须退到平台 target。"""
    from cpu.potential_field import FieldConfig, FieldMemory, resolve_goal

    for key in ("small_obstacles", "moving_block"):
        world = _world(key=key, warm=200)
        v = _field_view(world, goal=None)          # 调用方没给目标
        goal, src, aim = resolve_goal(v, FieldConfig(goal_policy="auto"),
                                      FieldMemory())
        assert goal is None, (
            f"{key}: view.goal=None 时 auto 无处可退（这正说明为什么"
            f"调用方要用平台 target 填 view.goal）")
        assert src == "none"


def test_time_dimension_shows_in_hud() -> None:
    """时间维开了以后必须在面板上看得见 —— 否则调参时不知道它有没有生效。"""
    world = _world(key="moving_block", warm=200)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1,
                      time_steps=6, time_dt=1.0 / 30.0)
    ov = field_overlay(_field_view(world, goal=(320.0, 240.0)), FieldMemory(), cfg)
    text = "\n".join(ov.rows)
    assert "时间维" in text and "K=6" in text, text
    assert "视野" in text


def test_time_dimension_off_says_static() -> None:
    world = _world(key="moving_block", warm=200)
    cfg = FieldConfig(cell=16.0, descend_steps=4, recompute=1)
    ov = field_overlay(_field_view(world, goal=(320.0, 240.0)), FieldMemory(), cfg)
    assert "静态波前" in "\n".join(ov.rows)


@pytest.mark.parametrize("key", ["wall_with_gap", "moving_block", "small_obstacles"])
@pytest.mark.parametrize("steps", [0, 6])
def test_demo_runs_with_and_without_time_dimension(key: str, steps: int) -> None:
    """三种障碍 × 时间维开/关，都要能跑完一局并渲出图。"""
    rc = run_demo(obstacle=key, seed=1, goal="auto", cell=16.0, descend=4,
                  recompute=8, frames=40, save=None, quiet=True,
                  time_steps=steps, ghost_horizon=0.3, width=320, height=240)
    assert rc == 0
