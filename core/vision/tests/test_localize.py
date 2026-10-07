"""任务一/二回归测试：像素 → 世界坐标、距离、地面反投影。

同样不需要摄像头：先用合成单应造出"图像点"，再验证能不能把世界坐标和长度量回来。

注意这里的参考单应 ``H_TRUE`` 是**强透视**的（第三行 1.5），否则"远近像素长度不同"
这类性质会被压平到看不出来 —— 这是写第一版测试时踩过的坑。
"""

from __future__ import annotations

import math

import pytest

from standard.obstacle.obstacle import Obstacle, ObstacleShape, ObstacleType
from standard.player.player import PlayerState
from standard.vision.vision_output import GapObs, Units, VisionFrame
from vision.calibration import Marker, apply_homography, calibrate_from_marker
from vision.localize import (
    GroundCamera,
    depth_error,
    distance,
    known_size_depth,
    length_scale,
    points_to_world,
    to_world,
)

#: 强透视的合成相机：世界(米) → 图像(px)。Y 越远，图像越靠上、越窄（梯形）。
H_TRUE = (800.0, 0.0, 320.0,
          0.0, -525.0, 430.0,
          0.0, 1.5, 1.0)
MARKER = Marker("test-marker", 0.30, 0.20)


def to_px(X, Y):
    return apply_homography(H_TRUE, X, Y)


def px_len(p, q):
    return math.hypot(q[0] - p[0], q[1] - p[1])


@pytest.fixture
def cal():
    quad = [to_px(X, Y) for X, Y in MARKER.world_corners()]
    return calibrate_from_marker(quad, MARKER, image_size=(640, 480))


# --------------------------------------------------------------------------
# A. 单应映射
# --------------------------------------------------------------------------
def test_points_round_trip(cal):
    world = [(0.05, 0.02), (0.18, 0.09), (0.27, 0.15)]
    got = points_to_world(cal, [to_px(X, Y) for X, Y in world])
    for (X, Y), (X2, Y2) in zip(world, got):
        assert math.hypot(X2 - X, Y2 - Y) < 1e-6


def test_distance_uses_mapping_not_a_single_pixel_scale(cal):
    """透视下像素↔米不是常数比例，所以距离必须先映射再量。"""
    # 两组世界长度都取 0.10 m，但一组在近处（Y=0.02）、一组在远处（Y=0.16）
    near = (to_px(0.02, 0.02), to_px(0.12, 0.02))
    far = (to_px(0.02, 0.16), to_px(0.12, 0.16))
    assert distance(cal, *near) == pytest.approx(0.10, abs=1e-6)
    assert distance(cal, *far) == pytest.approx(0.10, abs=1e-6)
    # 但它们在图上的像素长度明显不同 —— 这正是不能乘单一系数的原因
    assert abs(px_len(*near) - px_len(*far)) > 5.0


def test_length_scale_is_local(cal):
    s_near = length_scale(cal, *((to_px(0.0, 0.0)), to_px(0.05, 0.0)))
    s_far = length_scale(cal, *((to_px(0.25, 0.18)), to_px(0.30, 0.18)))
    assert s_near > 0 and s_far > 0
    assert abs(s_near - s_far) / max(s_near, s_far) > 0.05, "局部尺度应随位置明显变化"
    assert s_far > s_near, "远处一个像素代表的真实距离更大"


# --------------------------------------------------------------------------
# B. 地面反投影
# --------------------------------------------------------------------------
def test_ground_backprojection_pitch_zero_matches_textbook_formula():
    """pitch=0 时必须退化成 Z = fy·h/(v−cy)、X = (u−cx)·Z/fx。"""
    cam = GroundCamera(height_m=1.2, fx=800.0, fy=800.0, cx=320.0, cy=240.0,
                       pitch_deg=0.0)
    for u, v in ((400.0, 400.0), (320.0, 460.0), (500.0, 300.0)):
        Z_text = cam.fy * cam.height_m / (v - cam.cy)
        X_text = (u - cam.cx) * Z_text / cam.fx
        X, Z = cam.to_ground(u, v)
        assert Z == pytest.approx(Z_text, rel=1e-9)
        assert X == pytest.approx(X_text, rel=1e-9)
        assert cam.ground_distance(u, v) == pytest.approx(math.hypot(X, Z))


def test_ground_backprojection_distance_grows_as_point_moves_up():
    """图像里越靠上的地面点越远（地面假设最基本的性质）。"""
    cam = GroundCamera(height_m=1.0, fx=700.0, fy=700.0, cx=320.0, cy=240.0)
    assert cam.ground_distance(320.0, 460.0) < cam.ground_distance(320.0, 360.0)
    assert cam.ground_distance(320.0, 360.0) < cam.ground_distance(320.0, 300.0)


def test_ground_ray_above_horizon_raises():
    cam = GroundCamera(height_m=1.0, fx=700.0, fy=700.0, cx=320.0, cy=240.0)
    with pytest.raises(ValueError, match="不朝下|地平线"):
        cam.to_ground(320.0, 100.0)         # 在地平线以上，与地面无交点


def test_pitching_down_brings_the_same_row_closer():
    """俯视时同一像点对应的地面点**更近** —— 说明 pitch 不是可有可无的参数。"""
    level = GroundCamera(1.0, 700.0, 700.0, 320.0, 240.0, pitch_deg=0.0)
    down = GroundCamera(1.0, 700.0, 700.0, 320.0, 240.0, pitch_deg=20.0)
    assert down.ground_distance(320.0, 360.0) < level.ground_distance(320.0, 360.0)


def test_pitch_90_looks_straight_down():
    """pitch=90° 时画面正中就是正下方：水平距离 0、深度等于安装高度。"""
    cam = GroundCamera(1.5, 700.0, 700.0, 320.0, 240.0, pitch_deg=90.0)
    X, Z = cam.to_ground(320.0, 240.0)
    assert X == pytest.approx(0.0, abs=1e-9)
    assert Z == pytest.approx(0.0, abs=1e-9)
    assert cam.ray(320.0, 240.0)[1] == pytest.approx(-1.0)   # 射线朝正下


# --------------------------------------------------------------------------
# 已知尺寸测距 / 双目误差
# --------------------------------------------------------------------------
def test_known_size_depth_is_inverse_linear():
    assert known_size_depth(700.0, 0.30, 70.0) == pytest.approx(3.0)
    assert known_size_depth(700.0, 0.30, 140.0) == pytest.approx(1.5)
    with pytest.raises(ValueError, match="像素尺寸"):
        known_size_depth(700.0, 0.30, 0.0)


def test_depth_error_matches_the_stereo_formula():
    """δZ = Z²·δd/(f·B)：参考项目实测 f=961.94 px、B=120 mm。"""
    f, b = 961.94, 0.12
    assert depth_error(1.0, f, b) == pytest.approx(1.0 * 0.5 / (f * b))
    assert depth_error(3.0, f, b) == pytest.approx(9.0 * 0.5 / (f * b))
    # 误差随距离平方增长：3 m 处必须是 1 m 处的 9 倍
    assert depth_error(3.0, f, b) == pytest.approx(9 * depth_error(1.0, f, b))
    assert depth_error(3.0, f, b) < 0.05, "该配置在 3 m 处应好于 5 cm"


# --------------------------------------------------------------------------
# 整帧搬到世界坐标
# --------------------------------------------------------------------------
def _wall_frame(*, center_m, half_w_m, half_h_m, player_m, player_r_m,
                gap_center_m, gap_half_m):
    """按**世界坐标**造一帧，再算出它在图像里**看起来**是什么样。

    关键：透视下世界坐标里轴对齐的矩形，在图像里并不轴对齐（有剪切）。
    所以这里不能直接写 ``rotation=0``，而要用映射后的四角算出图像里的
    中心 / 半宽 / 半高 / 朝向 —— 否则测试自己在造假几何。
    """
    w4 = [to_px(center_m[0] + sx * half_w_m, center_m[1] + sy * half_h_m)
          for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    c = (sum(p[0] for p in w4) / 4.0, sum(p[1] for p in w4) / 4.0)
    # 图像里的"等效旋转矩形"要和 to_world 用**同一套定义**，否则测试自己在造
    # 不一致的几何：half_h 必须是两条长边之间的**垂直距离**的一半，
    # 而不是短边的长度 —— 透视剪切下两者差 cos(剪切角)，实测会带来 12.5% 的系统偏差。
    ax = (w4[1][0] - w4[0][0]) + (w4[2][0] - w4[3][0])
    ay = (w4[1][1] - w4[0][1]) + (w4[2][1] - w4[3][1])
    rot = math.atan2(ay, ax)
    nrm = math.hypot(ax, ay)
    ux, uy = ax / nrm, ay / nrm
    proj0 = abs((w4[1][0] - w4[0][0]) * ux + (w4[1][1] - w4[0][1]) * uy)
    proj1 = abs((w4[2][0] - w4[3][0]) * ux + (w4[2][1] - w4[3][1]) * uy)
    hw = (proj0 + proj1) / 4.0
    m0 = ((w4[0][0] + w4[1][0]) / 2.0, (w4[0][1] + w4[1][1]) / 2.0)
    m1 = ((w4[3][0] + w4[2][0]) / 2.0, (w4[3][1] + w4[2][1]) / 2.0)
    hh = abs((m1[0] - m0[0]) * (-uy) + (m1[1] - m0[1]) * ux) / 2.0
    p = to_px(*player_m)
    pr = to_px(player_m[0] + player_r_m, player_m[1])
    g = to_px(*gap_center_m)
    g0 = to_px(gap_center_m[0] - gap_half_m, gap_center_m[1])
    g1 = to_px(gap_center_m[0] + gap_half_m, gap_center_m[1])
    return VisionFrame(
        seq=1, stamp=0.0, units=Units.PX,
        player=PlayerState(x=p[0], y=p[1], radius=px_len(p, pr), vx=30.0, vy=0.0),
        obstacles=[Obstacle(id=2001, type=ObstacleType.WALL_WITH_GAP, type_id=2,
                            shape=ObstacleShape.RECT, x=c[0], y=c[1],
                            half_w=hw, half_h=hh, rotation=rot)],
        gaps=[GapObs(id=1, center=g, width=px_len(g0, g1), blockers=(2001, 2002))],
    )


def test_to_world_converts_units_and_geometry(cal):
    frame = _wall_frame(center_m=(0.10, 0.10), half_w_m=0.05, half_h_m=0.01,
                        player_m=(0.20, 0.05), player_r_m=0.008,
                        gap_center_m=(0.15, 0.10), gap_half_m=0.03)
    out = to_world(frame, cal)
    assert out.units == Units.M
    assert out.metres_per_unit == 1.0

    # 容差 1 mm：透视投影后的矩形是**一般四边形**，而 Obstacle 只能表达
    # 「旋转矩形」（中心+两个半尺寸+一个朝向），把四边形塞进这 4 个参数会丢掉
    # 一点信息（这里约 0.1 mm）。这不是实现错误，是表达能力的边界。
    assert out.player is not None
    assert out.player.x == pytest.approx(0.20, abs=1e-3)
    assert out.player.y == pytest.approx(0.05, abs=1e-3)
    assert out.player.radius == pytest.approx(0.008, abs=1e-3)

    ob = out.obstacles[0]
    assert ob.x == pytest.approx(0.10, abs=1e-3)
    assert ob.y == pytest.approx(0.10, abs=1e-3)
    assert ob.half_w == pytest.approx(0.05, abs=2e-3), "半宽要按四角映射换算"
    assert ob.half_h == pytest.approx(0.01, abs=2e-3)
    assert abs(ob.rotation) < 5e-2, "水平墙在世界系里仍大致水平"

    g = out.gaps[0]
    assert g.center[0] == pytest.approx(0.15, abs=1e-3)
    assert g.width == pytest.approx(0.06, abs=2e-3)


def test_to_world_is_idempotent_in_metres(cal):
    frame = _wall_frame(center_m=(0.10, 0.10), half_w_m=0.05, half_h_m=0.01,
                        player_m=(0.20, 0.05), player_r_m=0.008,
                        gap_center_m=(0.15, 0.10), gap_half_m=0.03)
    once = to_world(frame, cal)
    assert to_world(once, cal) is once, "已经是米单位时不应再转一次"


def test_gap_fits_player_works_after_conversion(cal):
    """任务一的交付意义：换算到世界坐标后，平台硬约束（缺口 ≥ 玩家直径）才可判。"""
    # 缺口 6 cm、玩家半径 4 cm（直径 8 cm）→ 过不去
    frame = _wall_frame(center_m=(0.10, 0.10), half_w_m=0.05, half_h_m=0.01,
                        player_m=(0.20, 0.05), player_r_m=0.04,
                        gap_center_m=(0.15, 0.10), gap_half_m=0.03)
    out = to_world(frame, cal)
    assert out.gaps[0].width == pytest.approx(0.06, abs=2e-3)
    assert out.player.radius == pytest.approx(0.04, abs=2e-3)
    assert out.gap_fits_player(out.gaps[0]) is False

    # 同一个缺口，玩家变小（半径 2 cm）→ 过得去
    frame2 = _wall_frame(center_m=(0.10, 0.10), half_w_m=0.05, half_h_m=0.01,
                         player_m=(0.20, 0.05), player_r_m=0.02,
                         gap_center_m=(0.15, 0.10), gap_half_m=0.03)
    out2 = to_world(frame2, cal)
    assert out2.gap_fits_player(out2.gaps[0]) is True
