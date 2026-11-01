"""**真实平台画面**上的决策回归：`data/vision/run4/screen.mp4` 的两帧。

## 数据是怎么来的（可复现）

```bash
Env/bin/python core/fpga/ref/export_frames.py \\
    --video data/vision/run4/screen.mp4 --out data/fpga/run4_seq \\
    --frames 60 --start 0 --stride 1 --width 640 --color
```

然后跑 `vision_board_devmem --ppm-dir data/fpga/run4_seq --frames 60 --track-log …`
（本机，C 参考模型），几何就来自它打印的 `R` 行 —— 也就是说下面这些数字是**平台
真实画面**经过完整视觉通路（分割 → 墙段 → 缺口 → 几何帧 → 解码）之后的产物，
不是手写的合成场景。

## 画面里本来是什么

深色网格场地；**上边两段橙色墙**，中间一个约 44 px 宽的缺口（x≈201.5~245.5）；
白色圆球是角色（x≈413，在缺口的**右**边）；四角是 UI 图标。

## 这两帧要钉住什么

* **帧 0**（一段墙=一个矩形）：缺口 44、可用半宽 9、指令全力向左 —— 角色在缺口右边，
  就该往左对准缺口；
* **帧 30**（同一堵墙被**两条带各检一次** → 4 个矩形）：决策必须把它并成**同样两堵墙**、
  缺口与指令**与帧 0 完全一致**。这条是真实数据里量出来的形态：带合并一变，段的中心
  会单帧横移（正是 `core/vision/motion.py` 里 `max_residual` 守卫要挡的东西），
  所以"并墙之后决策不变"这条不变量值得钉住 —— 它错了不会报错，只会让角色朝错方向走。
"""

from __future__ import annotations

import pytest

from cpu.gap_avoid import AlgoConfig, GapMemory, Rect, WorldView, plan, to_command

#: 帧里角色的**实际**半径（7，不是随手写的 10）：可用半宽 = 44/2 − 7 − 6 = 9，
#: 与程序在真机上打印的 `可用半宽=+9.0` 对得上。写错半径会让断言差 3 而看不出原因。
_PLAYER_R = 7.0


def _rect(i, x, y, vx, vy, hw, hh):
    return Rect(i, x, y, vx, vy, hw, hh, 0.0, 0.0)


def _player(x, y, vx, vy):
    return [x, y, vx, vy, _PLAYER_R, 200.0, 1.0]


#: 帧 0：墙只检出一段一条（y=45.5）。
_FRAME0 = WorldView(
    player=_player(413.0, 112.5, 0.0, 0.0),
    rects=[_rect(1, 164.50, 45.50, 0.0, 0.0, 37.00, 3.00),
           _rect(2, 378.50, 45.50, 0.0, 0.0, 133.00, 3.00)],
    dt=1.0 / 30.0, step_index=0, field_w=640.0, field_h=360.0)

#: 帧 30：同一堵墙被两条带各检一次（y=12.5 与 y=60），而且下边那条更宽
#: —— 这正是会引起"段中心跳变"的形态。速度是跟踪器推出来的（13 左右）。
_FRAME30 = WorldView(
    player=_player(423.7, 127.5, 6.6, 16.7),
    rects=[_rect(1, 164.50, 12.50, 0.0, 12.99, 37.00, 3.00),
           _rect(2, 378.50, 12.50, 0.0, 12.98, 133.00, 3.00),
           _rect(3, 140.50, 60.00, 0.0, 12.66, 61.00, 3.50),
           _rect(4, 402.50, 60.00, 0.0, 12.69, 157.00, 3.50)],
    dt=1.0 / 30.0, step_index=30, field_w=640.0, field_h=360.0)


def _decide(w: WorldView):
    return plan(w, GapMemory(), AlgoConfig(), horizon_remaining=2.0)


def test_frame0_aims_left_at_the_real_gap():
    """帧 0：真实平台画面里角色在缺口右边 → 应当全力向左对准缺口。"""
    d = _decide(_FRAME0)
    cmd = to_command(d)
    assert d.tier == "ALIGN", f"真实画面里这一帧应当是 ALIGN，得到 {d.tier}"
    assert d.gap == pytest.approx(44.0), f"缺口宽度：{d.gap} 应为我从画面量到的 44"
    assert d.usable_half == pytest.approx(9.0), f"可用半宽：{d.usable_half}"
    assert cmd[0] == pytest.approx(-1.0), f"应当全力向左，得到 {cmd}"
    assert cmd[1] == pytest.approx(0.0, abs=1e-9), f"纵向不该有分量：{cmd}"
    assert d.n_rects == 2


def test_band_doubled_wall_gives_the_same_decision():
    """同一堵墙被两条带各检一次（4 个矩形）时，决策必须与单条带时**完全一致**。

    真实数据里这个形态很常见（带合并状态一变就出现），而且是"段中心单帧跳变"的
    来源。决策若把它当成两堵墙/两个缺口，角色就会朝错的方向走 —— 而且不报错。
    """
    a, b = _decide(_FRAME0), _decide(_FRAME30)
    assert b.n_rects == a.n_rects == 2, "4 个矩形应当被并成同样两堵墙"
    assert b.tier == a.tier, f"tier 不一致：{a.tier} vs {b.tier}"
    assert b.gap == pytest.approx(a.gap), f"缺口不一致：{a.gap} vs {b.gap}"
    assert b.usable_half == pytest.approx(a.usable_half)
    assert to_command(b)[0] == pytest.approx(to_command(a)[0]), "指令方向不一致"
