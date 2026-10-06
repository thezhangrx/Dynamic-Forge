"""``wall_with_gap`` 识别的合成图回归测试。

**完全自包含**：不读文件、不用 numpy/opencv、不需要摄像头。合成图直接用纯 Python
往一个 ``bytearray`` 里画，和算法本身一样是可以翻成 Verilog 的东西。

实拍图会随光照、角度、屏幕眩光变化，不能当断言基准；合成图可以。实拍图的作用是
调参与目视验收（``python vision_detect.py``）。
"""

from __future__ import annotations

import pytest

from vision.primitives import Frame
from vision.wall_with_gap import WallWithGapParams, detect

W, H = 640, 480
FIELD = (34, 40, 55)
WALL = (240, 160, 48)
GRID = (44, 52, 65)          # 与场地只差 10 个灰阶：低于 edge_min=12，不该触发边缘
PLAYER = (250, 250, 250)
TARGET = (110, 200, 90)
GLARE = (250, 250, 250)      # 灯反光：把颜色洗成近白


# --------------------------------------------------------------------------
# 纯 Python 画图（只服务测试）
# --------------------------------------------------------------------------
def _put(buf: bytearray, x: int, y: int, rgb: tuple[int, int, int]) -> None:
    if 0 <= x < W and 0 <= y < H:
        i = (y * W + x) * 3
        buf[i:i + 3] = bytes(rgb)


def _rect(buf: bytearray, x0: int, y0: int, x1: int, y1: int,
          rgb: tuple[int, int, int]) -> None:
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            _put(buf, x, y, rgb)


def _disc(buf: bytearray, cx: int, cy: int, r: int,
          rgb: tuple[int, int, int]) -> None:
    for y in range(cy - r, cy + r + 1):
        for x in range(cx - r, cx + r + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                _put(buf, x, y, rgb)


def _ring(buf: bytearray, cx: int, cy: int, r: int, thick: int,
          rgb: tuple[int, int, int]) -> None:
    lo, hi = (r - thick) ** 2, r * r
    for y in range(cy - r, cy + r + 1):
        for x in range(cx - r, cx + r + 1):
            d = (x - cx) ** 2 + (y - cy) ** 2
            if lo <= d <= hi:
                _put(buf, x, y, rgb)


def make_frame(*, rows=((100, 240),), player=(320, 350, 12),
               target=(320, 200, 14), glare=None, grid=True) -> Frame:
    """一行 = ``(y0, gap_x)``：左段 ``0..gap_x``，右段 ``gap_x+60..W-1``。

    ``glare`` = ``(x0, x1, y0, y1)``：在这个矩形里把墙"洗白"（颜色判据失效）。
    """
    buf = bytearray(bytes(FIELD) * (W * H))
    if grid:
        for gx in range(0, W, 100):
            for y in range(H):
                _put(buf, gx, y, GRID)
        for gy in range(0, H, 100):
            for x in range(W):
                _put(buf, x, gy, GRID)
    for y0, gap_x in rows:
        _rect(buf, 0, y0, gap_x, y0 + 26, WALL)
        _rect(buf, gap_x + 60, y0, W - 1, y0 + 26, WALL)
    if glare is not None:
        _rect(buf, *glare, GLARE)
    _disc(buf, player[0], player[1], player[2], PLAYER)
    _ring(buf, target[0], target[1], target[2], 3, TARGET)
    return Frame.from_rgb(W, H, buf)


# --------------------------------------------------------------------------
# 墙 / 缺口
# --------------------------------------------------------------------------
def test_single_wall_gap_is_recovered():
    obs = detect(make_frame(rows=((100, 240),)))
    assert obs.player is not None and obs.target is not None
    assert len(obs.walls) == 1

    wall = obs.walls[0]
    assert len(wall.segments) == 2
    assert wall.gap is not None
    assert wall.gap.cx == pytest.approx(270.0, abs=4.0)      # 缺口 240..300 的中点
    assert wall.gap.width_px == pytest.approx(60.0, abs=6.0)
    assert abs(wall.gap.axis_deg) < 3.0                      # 墙是水平的
    assert wall.thickness_px == pytest.approx(27.0, abs=4.0)
    assert wall.segments[0].x0 == 0
    assert wall.segments[-1].x1 >= W - 2


def test_two_rows_become_two_walls():
    obs = detect(make_frame(rows=((80, 200), (300, 380))))
    assert len(obs.walls) == 2
    gaps = sorted(w.gap.cx for w in obs.walls if w.gap)
    assert len(gaps) == 2
    assert gaps[0] == pytest.approx(230.0, abs=6.0)          # 200..260
    assert gaps[1] == pytest.approx(410.0, abs=6.0)          # 380..440


def test_edge_opening_is_not_an_internal_gap():
    """缺口贴边 → 场上一整段墙，不应报出内部缺口。"""
    buf = bytearray(bytes(FIELD) * (W * H))
    _rect(buf, 0, 200, W - 1, 226, WALL)
    obs = detect(Frame.from_rgb(W, H, buf))
    assert len(obs.walls) == 1
    assert obs.walls[0].gap is None


def test_glare_does_not_widen_the_gap():
    """灯反光把墙的颜色洗白时，靠墙的边缘仍应把缺口定在原地。

    这是实拍图里踩到的坑：缺口左边被眩光吃掉后，纯颜色判据把缺口从 60px
    放大到 110px。加"纵向边缘"判据后应回到 60px 附近。
    """
    # 反光盖住墙的上半部分（y=0..20），留下墙的下边缘 y=21..26 可见
    frame = make_frame(rows=((100, 300),), glare=(240, 300, 95, 120))
    obs = detect(frame)
    wall = obs.walls[0]
    assert wall.gap is not None
    assert wall.gap.width_px == pytest.approx(60.0, abs=8.0), "反光不应把缺口放大"
    assert obs.walls[0].segments[0].x1 >= 295, "左段应延伸到 x≈300，而不是在 x≈240 就断"


def test_grid_lines_do_not_create_false_walls():
    """场地网格线对比度低（ΔG=10 < edge_min=12），不该被当成墙。"""
    obs = detect(make_frame(rows=((100, 240),), grid=True))
    assert len(obs.walls) == 1


# --------------------------------------------------------------------------
# 角色 / 目标
# --------------------------------------------------------------------------
def test_player_and_target_are_located():
    obs = detect(make_frame(player=(200, 400, 15), target=(450, 150, 12)))
    assert obs.player is not None
    assert obs.player.x == pytest.approx(200.0, abs=3.0)
    assert obs.player.y == pytest.approx(400.0, abs=3.0)
    assert obs.player.r == pytest.approx(15.0, abs=2.5)
    assert obs.target is not None
    assert obs.target.x == pytest.approx(450.0, abs=4.0)
    assert obs.target.y == pytest.approx(150.0, abs=4.0)


def test_defaults_are_usable():
    p = WallWithGapParams()
    assert 0 < p.wall_diff <= 60
    assert p.edge_min > 0
    obs = detect(make_frame(), p)
    assert obs.walls and obs.player and obs.target
