"""`--obstacle-speed` 必须真正覆盖默认场景的速度（以前它只走 `--obstacle-type` 那条路）。

用户实测传 580 墙速不变 —— 根因：默认场景的速度由构建器按 level 算
（`wall_speed = (44 + 52·v) · speed_scale`），CLI 的覆盖参数没有传进
`template_patterns()`。现在透传为 `speed_override`：**墙速精确等于它**，
其它障碍按同一比例缩放（保持相对节奏）。
"""

from __future__ import annotations

import pytest

from bullet_sim.scenarios.builder import build_scenario, template_patterns
from bullet_sim.scenarios.complexity import profile_for


def _walls(patterns):
    """墙 = 矩形（`half_w > half_h`）；PatternSpec 用 half_w/half_h 表示形状。"""
    return [p for p in patterns
            if str(getattr(p, "params", {}).get("layout", "")) == "wall_gap"]


def test_override_sets_wall_speed_exactly():
    prof = profile_for("medium")
    base = template_patterns(prof, 640.0, 480.0)
    fast = template_patterns(prof, 640.0, 480.0, speed_override=200.0)
    wb, wf = _walls(base), _walls(fast)
    if not wb or not wf:
        pytest.skip("该 profile 不含 wall_with_gap")
    assert abs(wf[0].speed - 200.0) < 1e-6, "覆盖后墙速必须精确等于给定值"
    assert abs(wf[0].speed - wb[0].speed) > 1.0, "覆盖必须真的改变默认速度"


def test_other_obstacles_scale_with_the_same_ratio():
    prof = profile_for("medium")
    base = template_patterns(prof, 640.0, 480.0)
    fast = template_patterns(prof, 640.0, 480.0, speed_override=200.0)
    wb, wf = _walls(base), _walls(fast)
    if not wb or not wf:
        pytest.skip("该 profile 不含 wall_with_gap")
    ratio = wf[0].speed / wb[0].speed
    for b, f in zip(base, fast):
        if float(getattr(b, "half_w", 0.0)) > float(getattr(b, "half_h", 0.0)):
            continue
        if not getattr(b, "speed", 0.0):
            continue
        assert abs((f.speed / b.speed) - ratio) < 1e-3, (
            f"{b.describe()}: 缩放比 {f.speed / b.speed:.4f} 与墙的 {ratio:.4f} 不一致")


def test_default_matches_play_build_path():
    """不传覆盖时，demo 用的构建路径与 `play` 完全一致（同一个 build_scenario）。"""
    a = build_scenario("medium", seed=0, kinds=("wall_with_gap",))
    b = build_scenario("medium", seed=0, kinds=("wall_with_gap",))
    assert a.spec.to_dict() == b.spec.to_dict(), "同参数两次构建必须完全一致"
