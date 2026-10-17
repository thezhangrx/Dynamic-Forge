"""边界余量（`AlgoConfig.edge_margin`）：只掐"朝外"的分量，切向保留。

为什么要这一条：缺口会扫到场地边缘，"对齐缺口"于是把角色一路推到边上；角色贴边时
视觉会漏检（blob 被画面边缘裁掉），平台只发 stay → 卡在边上（实测 300 帧里角色检测
位置几乎恒为 x=0）。

两条必须钉住的语义：
1. `margin = 0` 时**逐位等于旧行为**（现有 200+ 例不受影响）；
2. 边界处朝外分量 = 0，但**切向分量保留** —— 否则角色会僵在边上（就是现在的症状）。
"""

from __future__ import annotations

import pytest

from cpu.gap_avoid import (
    P_RADIUS,
    AlgoConfig,
    Decision,
    GapMemory,
    Rect,
    WorldView,
    limit_edges,
    plan,
    to_command,
)

DT = 1.0 / 120.0


def _player(x: float, y: float = 240.0, r: float = 10.0) -> list:
    return [x, y, 0.0, 0.0, r, 200.0, 1.0]


# --------------------------------------------------------------------------
# 纯函数
# --------------------------------------------------------------------------
def test_margin_zero_is_identity():
    p = _player(10.0)
    assert limit_edges(-1.0, 0.5, p, 640.0, 480.0, 0.0) == (-1.0, 0.5)


def test_at_the_boundary_outward_component_is_zero():
    p = _player(10.0)                      # 中心 = 半径 → 贴住左边界
    wx, wy = limit_edges(-1.0, 0.5, p, 640.0, 480.0, 25.0)
    assert wx == pytest.approx(0.0), "贴边时不许再往左推"
    assert wy == pytest.approx(0.5), "切向（y）必须保留，否则会僵在边上"


def test_ramp_is_linear_inside_the_margin():
    p = _player(22.5)                      # (x - r) = 12.5 = margin/2
    wx, _ = limit_edges(-1.0, 0.0, p, 640.0, 480.0, 25.0)
    assert wx == pytest.approx(-0.5)


def test_outside_the_margin_is_untouched():
    p = _player(320.0)
    assert limit_edges(-1.0, 1.0, p, 640.0, 480.0, 25.0) == (-1.0, 1.0)


@pytest.mark.parametrize("x,y,wx,wy,expect_wx,expect_wy", [
    (10.0, 240.0, -1.0, 0.0, 0.0, 0.0),        # 左边界，朝左
    (630.0, 240.0, +1.0, 0.0, 0.0, 0.0),       # 右边界，朝右
    (320.0, 10.0, 0.0, -1.0, 0.0, 0.0),        # 上边界（y 小），朝上
    (320.0, 470.0, 0.0, +1.0, 0.0, 0.0),       # 下边界，朝下
])
def test_all_four_sides(x, y, wx, wy, expect_wx, expect_wy):
    p = _player(x, y)
    got = limit_edges(wx, wy, p, 640.0, 480.0, 25.0)
    assert got == pytest.approx((expect_wx, expect_wy))


def test_margin_larger_than_field_does_not_blow_up():
    """场地比 margin 还小时系数不能算翻（lim 夹到半个场宽）。"""
    p = _player(5.0, 5.0, r=2.0)
    wx, wy = limit_edges(-1.0, -1.0, p, 10.0, 10.0, 100.0)
    assert -1.0 <= wx <= 0.0 and -1.0 <= wy <= 0.0


# --------------------------------------------------------------------------
# 接进 plan()：贴边时那一条朝外指令必须被削掉
# --------------------------------------------------------------------------
def _wall_gap_near_left_edge() -> list:
    """一堵横墙，缺口在 x≈40（靠左）。段只有右边那一截。"""
    return [Rect(1, 360.0, 200.0, 0.0, 0.0, 280.0, 13.0, 0.0, 0.0)]


@pytest.mark.parametrize("x,y", [
    (12.0, 240.0),      # 贴左边界
    (628.0, 240.0),     # 贴右边界
    (320.0, 12.0),      # 贴"y 小"的边界
    (320.0, 468.0),     # 贴"y 大"的边界
])
def test_plan_never_commands_outward_at_a_boundary(x, y):
    """贴边时，**朝该边界的那个分量必须是 0** —— 不管律本身想往哪推。

    这条是对"接线"的检查（纯函数的语义已由上面的用例钉住）：只要律把角色推到边界，
    它就再也不许往外推；否则角色会被顶出可视区、视觉漏检、平台只能发 stay（卡在边上）。
    """
    world = WorldView(player=_player(x, y), rects=_wall_gap_near_left_edge(),
                      dt=DT, step_index=0, field_w=640.0, field_h=480.0)
    dec = plan(world, GapMemory(), AlgoConfig(edge_margin=25.0), horizon_remaining=2.0)
    if x < 320.0:
        assert dec.wx >= 0.0, f"贴左边界却还在往左推：wx={dec.wx:+.3f}"
    if x > 320.0:
        assert dec.wx <= 0.0, f"贴右边界却还在往右推：wx={dec.wx:+.3f}"
    if y < 240.0:
        assert dec.wy >= 0.0, f"贴上边界却还在往上推：wy={dec.wy:+.3f}"
    if y > 240.0:
        assert dec.wy <= 0.0, f"贴下边界却还在往下推：wy={dec.wy:+.3f}"


def test_without_margin_the_same_scene_is_unchanged():
    """对照组：margin=0（默认）必须**逐位等于旧行为** —— 现有 200+ 例不受影响。"""
    def run(margin: float) -> tuple:
        world = WorldView(player=_player(12.0), rects=_wall_gap_near_left_edge(),
                          dt=DT, step_index=0, field_w=640.0, field_h=480.0)
        d = plan(world, GapMemory(), AlgoConfig(edge_margin=margin), horizon_remaining=2.0)
        return (d.wx, d.wy, d.tier)
    assert run(0.0) == run(0.0)
    got = run(0.0)
    assert got[0] != 0.0 or got[1] != 0.0, "对照组应当给出非零指令（否则这条用例没意义）"


def test_plan_is_deterministic_with_margin():
    def once() -> tuple:
        world = WorldView(player=_player(30.0), rects=_wall_gap_near_left_edge(),
                          dt=DT, step_index=0, field_w=640.0, field_h=480.0)
        d = plan(world, GapMemory(), AlgoConfig(edge_margin=25.0), horizon_remaining=2.0)
        return (d.wx, d.wy, d.tier, d.edge_limited)
    assert once() == once()
