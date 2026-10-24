"""三个规范的回归测试。

**纯 stdlib**（只用 `json` / `pathlib` / `pytest`），不依赖 numpy/opencv/摄像头。
把 README 里每一条"设计原则"都变成一条**可执行**的断言——尤其是那些
"必须拦住某种错误用法"的规则，否则规范会慢慢漂移。
"""

from __future__ import annotations

import json
import pathlib

import pytest

from standard.obstacle.obstacle import (
    TYPE_CODES,
    Obstacle,
    ObstacleShape,
    ObstacleType,
)
from standard.player.player import PlayerState
from standard.vision.vision_output import GapObs, Units, VisionFrame

HERE = pathlib.Path(__file__).resolve().parents[1]


def _examples(rel: str) -> list[dict]:
    doc = json.loads((HERE / rel).read_text(encoding="utf-8"))
    return doc["examples"]


# --------------------------------------------------------------------------
# JSON Schema 可解析 + 内嵌样例能喂给 Python 定义
# --------------------------------------------------------------------------
@pytest.mark.parametrize("rel", ["player/player.json", "obstacle/obstacle.json",
                                 "vision/vision_output.json"])
def test_schema_json_is_parsable(rel: str):
    doc = json.loads((HERE / rel).read_text(encoding="utf-8"))
    assert doc["$schema"].endswith("2020-12/schema")
    assert doc["x-standard-version"] == 1
    assert doc["examples"], "每个 schema 至少内嵌一个样例"


def test_player_example_round_trips():
    p = PlayerState.from_dict(_examples("player/player.json")[0])
    p.validate()
    assert PlayerState.from_dict(p.to_dict()).to_dict() == p.to_dict()
    # 派生量：能算的就不存（原则 P1）
    assert p.diameter == pytest.approx(2 * p.radius)
    assert p.speed_measured == pytest.approx((p.vx ** 2 + p.vy ** 2) ** 0.5)


def test_obstacle_examples_round_trip():
    for ex in _examples("obstacle/obstacle.json"):
        o = Obstacle.from_dict(ex)
        o.validate()
        assert Obstacle.from_dict(o.to_dict()).to_dict() == o.to_dict()


def test_vision_example_round_trips():
    vf = VisionFrame.from_dict(_examples("vision/vision_output.json")[0])
    vf.validate()
    assert VisionFrame.from_dict(vf.to_dict()).to_dict() == vf.to_dict()


# --------------------------------------------------------------------------
# 与 BHL1 v3 的线格式互转
# --------------------------------------------------------------------------
def test_bhl1_bullet_is_17_floats_and_round_trips():
    o = Obstacle.from_dict(_examples("obstacle/obstacle.json")[1])   # 矩形墙段
    b = o.to_bhl1_bullet()
    assert len(b) == 17, "BHL1 v3 的 bullet stride 是 17 个 float32"
    back = Obstacle.from_bhl1_bullet(b)
    assert back.id == o.id and back.shape == o.shape
    assert back.half_w == o.half_w and back.half_h == o.half_h
    assert back.rotation == o.rotation
    assert back.to_bhl1_bullet() == pytest.approx(b)


def test_bhl1_player_is_7_floats():
    p = PlayerState.from_dict(_examples("player/player.json")[0])
    vals = p.to_bhl1_player()
    assert len(vals) == 7, "BHL1 player 块是 f32[7]"
    assert vals[4] == p.radius and vals[5] == p.speed


# --------------------------------------------------------------------------
# 规范必须拦住的错误用法（每条对应 README 里的一条设计原则）
# --------------------------------------------------------------------------
def test_rect_invariant_half_w_ge_half_h():
    with pytest.raises(ValueError, match="不变量"):
        Obstacle(id=1, shape=ObstacleShape.RECT, x=0, y=0,
                 half_w=5, half_h=9).validate()


def test_circle_requires_radius():
    with pytest.raises(ValueError, match="radius"):
        Obstacle(id=1, shape=ObstacleShape.CIRCLE, x=0, y=0).validate()


def test_type_id_must_match_type():
    with pytest.raises(ValueError, match="type_id 与 type 不一致"):
        Obstacle(id=1, shape=ObstacleShape.CIRCLE, x=0, y=0, radius=1,
                 type=ObstacleType.CORRIDOR, type_id=1).validate()


def test_angle_must_stay_synced_with_velocity():
    with pytest.raises(ValueError, match="不同步"):
        Obstacle(id=1, shape=ObstacleShape.CIRCLE, x=0, y=0, radius=1,
                 vx=1.0, vy=0.0, angle=3.0).validate()


def test_speed_is_a_cap_not_instantaneous():
    """把上限当瞬时速度用，必须报错（player.md §3.2）。"""
    with pytest.raises(ValueError, match="超过上限"):
        PlayerState(x=0, y=0, radius=1, vx=100.0, vy=0.0, speed=10.0).validate()


def test_occluded_gap_cannot_claim_to_be_reliable():
    """灯反光下缺口边界是推出来的，必须标 reliable=False。"""
    with pytest.raises(ValueError, match="reliable"):
        GapObs(id=1, center=(0, 0), width=10, blockers=(1, 2),
               occluded=True, reliable=True).validate()


def test_gap_blockers_must_reference_obstacles_in_the_same_frame():
    vf = VisionFrame(seq=0, stamp=0.0,
                     obstacles=[Obstacle(id=1, shape=ObstacleShape.CIRCLE,
                                         x=0, y=0, radius=1)],
                     gaps=[GapObs(id=1, center=(0, 0), width=10, blockers=(1, 99))])
    with pytest.raises(ValueError, match="不存在的墙段"):
        vf.validate()


def test_one_frame_one_coordinate_system():
    """一条消息只能有一个坐标系（原则 P2）。"""
    vf = VisionFrame(seq=0, stamp=0.0, frame_id="field",
                     player=PlayerState(x=0, y=0, radius=1, frame_id="camera"))
    with pytest.raises(ValueError, match="只能有一个坐标系"):
        vf.validate()


def test_units_must_be_explicit_and_consistent():
    VisionFrame(seq=0, stamp=0.0, units=Units.PX).validate()
    VisionFrame(seq=0, stamp=0.0, units=Units.M, metres_per_unit=1.0).validate()
    with pytest.raises(ValueError, match="metres_per_unit"):
        VisionFrame(seq=0, stamp=0.0, units=Units.M, metres_per_unit=0.01).validate()


# --------------------------------------------------------------------------
# 语义助手：平台硬约束与时间
# --------------------------------------------------------------------------
def test_gap_fits_player_uses_the_platform_hard_constraint():
    """平台硬约束：缺口宽度不得小于玩家圆直径。"""
    gap = GapObs(id=1, center=(0, 0), width=45.0, blockers=(1, 2))
    assert gap.fits(player_radius=10.0) is True         # 45 >= 20
    assert gap.fits(player_radius=30.0) is False        # 45 < 60


def test_dt_uses_stamp_not_seq():
    a = VisionFrame(seq=0, stamp=1.0)
    b = VisionFrame(seq=9, stamp=1.5)                   # 帧号差 9，真实间隔 0.5 s
    assert b.dt_since(a) == pytest.approx(0.5)
    with pytest.raises(ValueError, match="dt"):
        a.dt_since(b)


def test_type_code_table_is_append_only_and_complete():
    assert TYPE_CODES[None] == 0
    assert sorted(v for v in TYPE_CODES.values() if v) == [1, 2, 3, 4, 5]
    assert len(set(TYPE_CODES.values())) == len(TYPE_CODES), "编码不能重复"
