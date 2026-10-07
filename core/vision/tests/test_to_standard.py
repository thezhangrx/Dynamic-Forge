"""适配器测试：``wall_with_gap`` 像素域观测 → 标准 ``VisionFrame``。

这一层只做翻译，所以测试也只看**翻译对不对**：字段有没有丢、不变量有没有破、
ID 关系有没有连对。几何（像素 → 世界）是 `localize` 的事，在别的文件里测。
"""

from __future__ import annotations

import pytest

from standard.obstacle.obstacle import ObstacleShape, ObstacleType
from standard.vision.vision_output import Units
from vision.to_standard import frame_to_standard
from vision.wall_with_gap import (
    FrameObservation,
    Gap,
    PlayerObs,
    TargetObs,
    Wall,
    WallSegment,
)


def _wall(y0, y1, segments, thickness=26.0, gap=None, covered=0.0) -> Wall:
    return Wall(y0=y0, y1=y1, segments=tuple(segments),
                thickness_px=thickness, covered_px=covered, gap=gap)


def _seg(x0, x1, cy=213.0, thickness=26.0, angle=0.0) -> WallSegment:
    return WallSegment(x0=x0, x1=x1, cy=cy, thickness=thickness, angle_deg=angle)


def _obs(walls=(), player=None, target=None, w=640, h=480) -> FrameObservation:
    return FrameObservation(image="<test>", width=w, height=h,
                            player=player, target=target, walls=tuple(walls),
                            field=(0, w - 1))


# --------------------------------------------------------------------------
def test_units_stay_pixels_and_metadata_is_carried():
    """适配器**不做几何** —— 单位必须还是像素，否则 to_world 会以为自己不用干活。"""
    obs = _obs(player=PlayerObs(x=100.0, y=200.0, r=9.0, fill=0.75))
    vf = frame_to_standard(obs, seq=3, stamp=1.25)
    assert vf.units == Units.PX
    assert (vf.seq, vf.stamp) == (3, 1.25)
    assert (vf.image_width, vf.image_height) == (640, 480)
    vf.validate()


def test_player_maps_to_player_state():
    obs = _obs(player=PlayerObs(x=100.0, y=200.0, r=9.0, fill=0.75))
    vf = frame_to_standard(obs, seq=0, stamp=0.0)
    p = vf.player
    assert p is not None
    assert (p.x, p.y, p.radius) == (100.0, 200.0, 9.0)
    assert p.score == pytest.approx(0.75)      # 填充率当置信度
    assert p.frame_id == vf.frame_id


def test_missing_player_stays_missing():
    """没看到就是 None，不能补一个 (0,0) 出来 —— "不知道"和"在原点"是两回事。"""
    vf = frame_to_standard(_obs(), seq=0, stamp=0.0)
    assert vf.player is None


def test_gap_wall_becomes_two_rects_sharing_a_group_plus_one_gap():
    """``wall_with_gap`` 的编码：两段 rect 共享 group_id + 一个缺口。"""
    wall = _wall(200, 226,
                 [_seg(40, 300), _seg(380, 600)],
                 thickness=26.0,
                 gap=Gap(cx=340.0, cy=213.0, width_px=79.0, axis_deg=0.0))
    vf = frame_to_standard(_obs(walls=[wall]), seq=0, stamp=0.0)
    vf.validate()

    assert len(vf.obstacles) == 2
    a, b = vf.obstacles
    assert a.shape == ObstacleShape.RECT and b.shape == ObstacleShape.RECT
    assert a.type == ObstacleType.WALL_WITH_GAP == b.type
    assert a.group_id == b.group_id, "同一堵墙的两段必须共享 group_id"

    assert len(vf.gaps) == 1
    g = vf.gaps[0]
    assert g.width == pytest.approx(79.0)
    assert g.center == pytest.approx((340.0, 213.0))
    assert set(g.blockers) == {a.id, b.id}, "缺口的两端必须是夹住它的那两段墙"


def test_rect_half_sizes_and_invariant():
    wall = _wall(200, 226, [_seg(40, 339)], thickness=26.0)
    vf = frame_to_standard(_obs(walls=[wall]), seq=0, stamp=0.0)
    ob = vf.obstacles[0]
    assert ob.half_w == pytest.approx(150.0)     # (339-40+1)/2
    assert ob.half_h == pytest.approx(13.0)      # 厚度的一半
    assert ob.half_w >= ob.half_h                # 标准结构的不变量


def test_short_fragments_are_dropped():
    """检测器为了找缺口会把墙切成碎片；太短的既不是障碍，还会破坏 half_w>=half_h。"""
    wall = _wall(200, 226, [_seg(40, 300), _seg(310, 320), _seg(325, 700)],
                 thickness=26.0,
                 gap=Gap(cx=305.0, cy=213.0, width_px=30.0, axis_deg=0.0))
    vf = frame_to_standard(_obs(walls=[wall]), seq=0, stamp=0.0,
                           min_segment_px=24)
    vf.validate()
    lengths = sorted(round(2 * o.half_w) for o in vf.obstacles)
    assert lengths == [261, 376], f"101 px 的碎片应当被丢掉，得到 {lengths}"


def test_degenerate_segment_is_skipped_not_raised():
    """比厚度还短的段会被丢掉，而不是让 validate() 抛异常炸掉整帧。"""
    wall = _wall(200, 226, [_seg(40, 45, thickness=26.0), _seg(200, 500)],
                 thickness=26.0)
    vf = frame_to_standard(_obs(walls=[wall]), seq=0, stamp=0.0, min_segment_px=1)
    vf.validate()                                 # 不抛
    assert all(o.half_w >= o.half_h for o in vf.obstacles)


def test_solid_wall_without_gap_becomes_one_obstacle():
    wall = _wall(200, 226, [_seg(40, 300), _seg(304, 600)], thickness=26.0)
    vf = frame_to_standard(_obs(walls=[wall]), seq=0, stamp=0.0)
    vf.validate()
    assert len(vf.obstacles) == 1
    assert vf.gaps == []


def test_obstacle_ids_are_unique_across_walls():
    walls = [_wall(100, 126, [_seg(40, 300)]),
             _wall(200, 226, [_seg(40, 300)])]
    vf = frame_to_standard(_obs(walls=walls), seq=0, stamp=0.0)
    vf.validate()                                 # 内部就会查重复 id
    assert len({o.id for o in vf.obstacles}) == 2


def test_target_maps_to_a_circle():
    obs = _obs(target=TargetObs(x=300.0, y=100.0, r=14.0, w=28, h=28))
    vf = frame_to_standard(obs, seq=0, stamp=0.0)
    vf.validate()
    assert vf.target is not None
    assert vf.target.shape == "circle"
    assert vf.target.radius == pytest.approx(14.0)
