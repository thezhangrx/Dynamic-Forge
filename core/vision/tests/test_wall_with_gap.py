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
GRID = (44, 52, 65)          # 低对比网格：与场地只差 10 个灰阶，低于 edge_min=12
#: **平台实际画的网格色**（见 `render/pygame_view.py` 的 `_draw_world_axes`）。
#: 与场地差 42 个灰阶，远高于 `edge_min * edge_avg = 24` —— 会触发"持续台阶"分支。
#: 老测试只用了上面的低对比网格，所以"网格线把缺口切碎"这个问题一直没被暴露。
PLATFORM_GRID = (70, 82, 105)
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
               target=(320, 200, 14), glare=None, grid=True,
               grid_rgb=GRID) -> Frame:
    """一行 = ``(y0, gap_x)``：左段 ``0..gap_x``，右段 ``gap_x+60..W-1``。

    ``glare`` = ``(x0, x1, y0, y1)``：在这个矩形里把墙"洗白"（颜色判据失效）。
    """
    buf = bytearray(bytes(FIELD) * (W * H))
    if grid:
        for gx in range(0, W, 100):
            for y in range(H):
                _put(buf, gx, y, grid_rgb)
        for gy in range(0, H, 100):
            for x in range(W):
                _put(buf, x, gy, grid_rgb)
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


def test_platform_contrast_grid_does_not_fragment_the_gap():
    """回归：**平台真实对比度的网格线**不能把缺口切碎。

    老测试用的网格只差 10 个灰阶，触发不了 `col_step_max`；平台实际画的是
    `(70,82,105)`（差 42），足以让"持续台阶"分支在缺口里的网格线处点亮。
    后果很严重：真实的 36 px 缺口被切成 "6+6+6" 三个小缝，`min_gap_px` 一个都不认，
    **整堵墙被当成实心**，决策层只能一直喊 `NO_GAP`。

    修法是在 `detect_walls` 里要求走边沿分支的列**至少有几个墙色像素**
    （`edge_min_cols`）—— 墙再白也留得下几像素墙色，网格线一像素都没有。
    """
    obs = detect(make_frame(rows=((100, 240),), grid_rgb=PLATFORM_GRID))
    assert len(obs.walls) == 1
    wall = obs.walls[0]
    assert len(wall.segments) == 2, \
        f"网格线把墙切成了 {len(wall.segments)} 段：{[(s.x0, s.x1) for s in wall.segments]}"
    assert wall.gap is not None, "缺口没被认出来（整堵墙被当成实心）"
    assert wall.gap.width_px == pytest.approx(60.0, abs=8.0)


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


# --------------------------------------------------------------------------
# 分辨率无关性
# --------------------------------------------------------------------------
def test_params_rescale_with_frame_size():
    """回归：默认参数是**按 640x480 调的**，换分辨率必须跟着缩放。

    实测：一段 1920x1080 的录屏里玩家圆是 1763 px，而 `player_max_area` 卡在 1500
    —— 玩家被丢掉，检测器退而选了旁边一块被光洗白的区域。现象看起来只是
    "检测不准"，很难反推到"上限是按分辨率写死的"。
    """
    p = WallWithGapParams()
    # s 取宽高比例的**较小**者：1920/640=3.0 但 1080/480=2.25 -> s=2.25。
    # 取小的才保守：宁可把窗口设小，也不要把整幅画面当成目标。
    big = p.rescaled(1920, 1080)
    s = 1080 / 480
    assert big.player_max_area == pytest.approx(1500 * s * s, abs=1)
    assert big.player_min_area == pytest.approx(60 * s * s, abs=1)
    assert big.min_gap_px == round(8 * s)                 # 长度按 s，面积按 s²
    # 灰度阈值与比例**不能**跟着缩放 —— 缩了就变成另一个判据了
    assert big.bright_min == p.bright_min
    assert big.wall_diff == p.wall_diff
    assert big.edge_min == p.edge_min
    assert big.wall_col_frac == p.wall_col_frac


def test_params_rescale_is_a_noop_at_the_reference_size():
    p = WallWithGapParams()
    assert p.rescaled(640, 480) is p           # 零开销，同一个对象
    assert p.rescaled(1280, 960).player_max_area == pytest.approx(1500 * 4)
    assert p.rescaled(1920, 1440).player_max_area == pytest.approx(1500 * 9)


def test_a_high_resolution_frame_still_finds_the_player():
    """整条路径的回归：把图放大 3 倍，缩放机制必须让检测照常工作。

    这条钉的是"参数里不要编码分辨率"这个性质本身。
    """
    small = make_frame(rows=((100, 240),), grid_rgb=PLATFORM_GRID)
    # 3 倍最近邻放大 = 分辨率变了，场景内容一样
    buf = bytearray(1920 * 1440 * 3)
    for y in range(1440):
        src = (y // 3) * 640
        row = y * 1920 * 3
        for x in range(1920):
            i = src + x // 3
            j = row + x * 3
            buf[j] = small.r[i]
            buf[j + 1] = small.g[i]
            buf[j + 2] = small.b[i]
    big = Frame.from_rgb(1920, 1440, buf)
    obs = detect(big)
    assert obs.player is not None, "放大 3 倍后玩家检不出来了 —— 参数又跟分辨率绑上了"
    # 玩家真值 (320, 350) -> 放大后 (960, 1050)
    assert obs.player.x == pytest.approx(960, abs=6)
    assert obs.player.y == pytest.approx(1050, abs=6)
