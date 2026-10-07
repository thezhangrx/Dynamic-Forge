"""把"交付标准"写成测试：简洁 / 依赖小 / 可迁移 C++。

这些测试不是验证算法对不对（那是 ``test_decision_*`` 的事），
而是**防止标准被无声破坏**：

* 有人往 ``gap_avoid.py`` 里 import 了 numpy 或平台 → 依赖变重、C++ 移植变难；
* 有人把 ``wrap_pi`` 换回朴素写法 → C++ 语义悄悄改变；
* 有人改了分桶遍历的顺序 → argmin 并列时结果依赖字典顺序。
"""

from __future__ import annotations

import ast
import math
import pathlib

import pytest

from cpu.gap_avoid import (
    AlgoConfig,
    GapMemory,
    Rect,
    WorldView,
    group_walls,
    plan,
    wrap_pi,
)

MODULE = pathlib.Path(__file__).resolve().parents[1] / "gap_avoid.py"

#: ``gap_avoid.py`` 允许的顶层 import —— 只有 stdlib。
ALLOWED_TOP_LEVEL_IMPORTS = {
    "__future__", "math", "collections", "dataclasses", "typing",
}


# ---------------------------------------------------------------------------
# 依赖小 / 可迁移：顶层导入必须只有 stdlib
# ---------------------------------------------------------------------------
def _top_level_imports(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in tree.body:                      # 只看模块级，函数内的懒加载不算
        if isinstance(node, ast.Import):
            names.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                names.add(node.module.split(".")[0])
    return names


def test_gap_avoid_only_imports_stdlib_at_module_level():
    found = _top_level_imports(MODULE)
    extra = found - ALLOWED_TOP_LEVEL_IMPORTS
    assert not extra, (
        f"gap_avoid.py 顶层多出了非 stdlib 依赖 {sorted(extra)}；"
        "它是「要移植到 C++ 的那份」，必须保持零依赖（numpy / 平台类型都不行）"
    )


def test_gap_avoid_has_no_platform_coupling_at_import_time():
    found = _top_level_imports(MODULE)
    bad = {n for n in found if n in {"bullet_sim", "cpu", "vision", "standard"}}
    assert not bad, (
        f"gap_avoid.py 顶层 import 了平台/兄弟模块 {sorted(bad)}；"
        "仿真器胶水在 cpu/adapters/sim_adapter.py，视觉胶水在 "
        "cpu/adapters/vision_adapter.py"
    )


def test_sim_adapter_is_where_the_platform_glue_lives():
    """胶水确实搬家了，而不是被删掉。"""
    adapter = MODULE.parent / "adapters" / "sim_adapter.py"
    src = adapter.read_text(encoding="utf-8")
    for name in ("view_from_world", "observe_rects", "rect_from_pool"):
        assert f"def {name}" in src


def test_legacy_imports_still_work():
    """``from cpu.gap_avoid import view_from_world`` 不能破（PEP 562 懒 re-export）。"""
    import cpu.gap_avoid as ga
    from cpu.adapters.sim_adapter import view_from_world as moved
    assert ga.view_from_world is moved
    assert ga.rect_from_pool is not None and ga.observe_rects is not None


# ---------------------------------------------------------------------------
# C++ 陷阱 1：负数取模（Python % vs C++ fmod）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("angle", [0.0, 0.1, -0.1, 1.0, -3.0, math.pi / 2])
def test_wrap_pi_is_identity_inside_the_range(angle):
    assert wrap_pi(angle) == pytest.approx(angle)


@pytest.mark.parametrize("turns", [-6, -4, -2, -1, 0, 1, 2, 4, 6])
def test_wrap_pi_range_is_half_open(turns):
    for base in (0.3, -0.3, 2.0, -2.0):
        v = wrap_pi(base + turns * 2.0 * math.pi)
        assert -math.pi <= v < math.pi


def test_wrap_pi_differs_from_the_naive_fmod_translation():
    """证明这个陷阱是**真实存在**的，而不是理论担忧。

    朴素 C++ 翻译 ``fmod(a + pi, 2pi) - pi`` 在 ``a < -pi`` 时给出错误结果；
    移植时必须按 ``wrap_pi`` docstring 里的写法补一次 ``if (a < 0) a += 2pi``。
    """
    for angle in (-4.0 * math.pi, -3.5 * math.pi, -7.0, -4.0):
        naive = math.fmod(angle + math.pi, 2.0 * math.pi) - math.pi
        correct = wrap_pi(angle)
        assert correct != pytest.approx(naive), f"angle={angle} 时两者竟然相同"
        assert -math.pi <= correct < math.pi
        assert not (-math.pi <= naive < math.pi), (
            f"angle={angle}: 朴素翻译给出 {naive}，已越界 —— 这正是要防的 bug"
        )


# ---------------------------------------------------------------------------
# C++ 陷阱 2：分桶遍历顺序决定 argmin 并列结果
# ---------------------------------------------------------------------------
def _wall_rects(a=120.0, b=520.0, half=120.0):
    return [
        Rect(1, a, 400.0, 0.0, 0.0, half, 13.0, 0.0, 0.0),
        Rect(2, b, 400.0, 0.0, 0.0, half, 13.0, 0.0, 0.0),
    ]


def test_group_walls_is_independent_of_input_order():
    fwd = (0.0, 1.0)
    forward_walls = group_walls(_wall_rects(), 100.0, 50.0, fwd)
    reverse_walls = group_walls(list(reversed(_wall_rects())), 100.0, 50.0, fwd)
    assert [w.key for w in forward_walls] == [w.key for w in reverse_walls]
    assert [w.u_hat for w in forward_walls] == [w.u_hat for w in reverse_walls]
    assert [w.n_hat for w in forward_walls] == [w.n_hat for w in reverse_walls]


def test_plan_is_independent_of_rect_order():
    """端到端：换一下障碍列表顺序，决策必须逐位相同。"""
    fwd = AlgoConfig(forward=(0.0, 1.0))
    base = _wall_rects()
    orderings = [base, list(reversed(base)),
                 [base[1], base[0]], base[:1] + base[1:]]
    results = []
    for rects in orderings:
        view = WorldView(player=[100.0, 50.0, 0.0, 0.0, 10.0, 200.0, 1.0],
                         rects=list(rects), dt=1.0 / 120.0, step_index=0,
                         field_w=640.0, field_h=480.0)
        dec = plan(view, GapMemory(), fwd, 2.0)
        results.append((dec.wx, dec.wy, dec.tier, dec.gap))
    assert len(set(results)) == 1, f"顺序不同导致决策不同：{results}"


# ---------------------------------------------------------------------------
# 简洁：入口面不能膨胀
# ---------------------------------------------------------------------------
def test_public_surface_stays_small():
    """``__all__`` 之外的公开名字不设限，但核心入口必须是这几个且存在。"""
    import cpu.gap_avoid as ga
    for name in ("WorldView", "Rect", "AlgoConfig", "GapMemory",
                 "Decision", "plan", "to_command"):
        assert hasattr(ga, name), f"核心入口 {name} 不见了"
