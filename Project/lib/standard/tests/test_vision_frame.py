"""视觉帧线协议的测试。

测的是**契约**，不是实现细节：往返、定标误差、CRC、版本门、长度、
以及"和 ``describe_protocol()`` 说的完全一致" ——
因为 HDL/testbench 会照着那份自述去对齐字段，说错就是错的开始。
"""

from __future__ import annotations

import struct

import pytest

from standard.obstacle.obstacle import Obstacle, ObstacleShape, ObstacleType
from standard.player.player import PlayerState
from standard.vision.vision_frame import (
    GAP_BYTES,
    OBSTACLE_BYTES,
    VISION_MAGIC,
    VISION_PROTOCOL_VERSION,
    VISION_Q,
    decode_vision_frame,
    describe_vision_protocol,
    encode_vision_frame,
    quantisation_error,
    vision_frame_size,
)
from standard.vision.vision_output import GapObs, TargetObs, Units, VisionFrame


def _frame(**kw) -> VisionFrame:
    base = dict(seq=7, stamp=1.25, units=Units.PX, image_width=640, image_height=480)
    base.update(kw)
    return VisionFrame(**base)


def _full_frame() -> VisionFrame:
    return _frame(
        player=PlayerState(x=320.0, y=72.0, radius=10.0),
        target=TargetObs(x=100.0, y=100.0, shape="rect", half_w=20.0, half_h=10.0),
        obstacles=[
            Obstacle(id=3, shape=ObstacleShape.RECT, type=ObstacleType.WALL_WITH_GAP,
                     x=100.0, y=400.0, half_w=80.0, half_h=5.0, rotation=0.0),
            Obstacle(id=4, shape=ObstacleShape.CIRCLE, type=ObstacleType.MOVING_BLOCK,
                     x=200.0, y=300.0, radius=12.0),
        ],
        gaps=[
            GapObs(id=1, center=(192.0, 412.0), width=60.0, axis=0.0,
                   blockers=(3, 4), blocked_by_type="wall_with_gap"),
        ],
    )


# --------------------------------------------------------------------------
# 一、往返
# --------------------------------------------------------------------------
def test_round_trip_is_exact_for_quantised_values() -> None:
    """值本来就是 1/VISION_Q 的整数倍时，往返必须**逐位精确**。

    这条挡的是"用 float 存整数"那类实现：只要编码路径碰了浮点再回来，
    某些值就会差 1 个 LSB。
    """
    q = float(VISION_Q)
    frame = _frame(
        player=PlayerState(x=320.0, y=72.0, radius=10.0),
        obstacles=[Obstacle(id=1, shape=ObstacleShape.RECT,
                            type=ObstacleType.WALL_WITH_GAP,
                            x=10.0, y=20.0, half_w=5.0, half_h=1.0),
                   Obstacle(id=2, shape=ObstacleShape.RECT,
                            type=ObstacleType.WALL_WITH_GAP,
                            x=90.0, y=20.0, half_w=5.0, half_h=1.0)],
        gaps=[GapObs(id=2, center=(1.0, 2.0), width=3.0, axis=0.0, blockers=(1, 2))],
    )
    for v in (frame.player.x, frame.player.y, frame.gaps[0].center[0]):
        assert (v * q) == round(v * q)          # 前提：确实可精确表示
    back = decode_vision_frame(encode_vision_frame(frame))
    assert back.player is not None
    assert (back.player.x, back.player.y, back.player.radius) == \
        (frame.player.x, frame.player.y, frame.player.radius)
    assert back.obstacles[0].x == frame.obstacles[0].x
    assert back.gaps[0].center == frame.gaps[0].center
    assert back.gaps[0].width == frame.gaps[0].width


def test_round_trip_keeps_arbitrary_values_within_the_quantisation_budget() -> None:
    """任意浮点值往返后，误差必须 ≤ 声明过的 ``quantisation_error()``。

    **这条比"能往返"更重要**：线协议是定标的，误差必须**有名有姓**，
    否则 `core/fpga/docs/移植规划.md` §2.3 的误差预算无从核对。
    """
    eps = quantisation_error()
    frame = _full_frame()
    back = decode_vision_frame(encode_vision_frame(frame))
    assert back.player is not None
    assert abs(back.player.x - frame.player.x) <= eps
    assert abs(back.player.radius - frame.player.radius) <= eps
    for a, b in zip(back.obstacles, frame.obstacles):
        assert abs(a.x - b.x) <= eps and abs(a.y - b.y) <= eps
        # radius 与 half_w/half_h 是联合体：圆形只比 radius，矩形只比半宽半高，
        # 另一半必须解成 None（不许拿 0 冒充"量到了"）
        if b.shape is ObstacleShape.CIRCLE:
            assert a.radius is not None and b.radius is not None
            assert abs(a.radius - b.radius) <= eps
            assert a.half_w is None and a.half_h is None
        else:
            assert a.half_w is not None and b.half_w is not None
            assert abs(a.half_w - b.half_w) <= eps
            assert abs(a.half_h - (b.half_h or 0.0)) <= eps
            assert a.radius is None
    for a, b in zip(back.gaps, frame.gaps):
        assert abs(a.center[0] - b.center[0]) <= eps
        assert abs(a.width - b.width) <= eps


def test_quantisation_error_is_tight_against_the_budget() -> None:
    """定标误差要**明显小于**缺口中心的目标误差（0.1 场地单位）。

    否则整个误差预算会被线协议自己吃掉，而这是最容易被忽略的一项。
    """
    assert quantisation_error() < 0.1 / 4.0


def test_round_trip_preserves_flags_units_and_codes() -> None:
    frame = _full_frame()
    back = decode_vision_frame(encode_vision_frame(frame))
    assert back.seq == frame.seq
    assert back.units == frame.units
    assert back.image_width == 640 and back.image_height == 480
    assert [o.id for o in back.obstacles] == [3, 4]
    assert [o.shape for o in back.obstacles] == [ObstacleShape.RECT, ObstacleShape.CIRCLE]
    assert back.obstacles[0].type == ObstacleType.WALL_WITH_GAP
    assert back.obstacles[0].type_id == 2
    assert back.gaps[0].blockers == (3, 4)
    assert back.gaps[0].blocked_by_type == "wall_with_gap"
    assert back.gaps[0].reliable is True
    assert back.target is not None and back.target.shape == "rect"


def test_stamp_round_trips_exactly_in_nanoseconds() -> None:
    """stamp 走 uint64 纳秒 —— 往返必须精确（硬件时戳是整数计数器）。"""
    for t in (0.0, 1.25, 1234.567891, 1e6 / 1e9):
        back = decode_vision_frame(encode_vision_frame(_frame(stamp=t)))
        assert back.stamp == pytest.approx(t, abs=1e-9)


def test_empty_frame_round_trips() -> None:
    back = decode_vision_frame(encode_vision_frame(_frame()))
    assert back.player is None and back.target is None
    assert back.obstacles == [] and back.gaps == []
    assert len(encode_vision_frame(_frame())) == vision_frame_size(0, 0)


# --------------------------------------------------------------------------
# 二、拒绝坏帧（"绝不猜"）
# --------------------------------------------------------------------------
def test_crc_detects_tampering() -> None:
    raw = bytearray(encode_vision_frame(_full_frame()))
    raw[40] ^= 0x01                      # 踩第一个几何字节
    with pytest.raises(ValueError, match="CRC32"):
        decode_vision_frame(raw)


def test_rejects_wrong_magic() -> None:
    raw = bytearray(encode_vision_frame(_frame()))
    raw[0:4] = struct.pack("<I", 0xDEADBEEF)
    with pytest.raises(ValueError, match="magic"):
        decode_vision_frame(raw)


def test_rejects_unknown_version_instead_of_guessing() -> None:
    """读到不认识的版本必须报错 —— 这是本项目一路坚持的纪律。"""
    raw = bytearray(encode_vision_frame(_frame()))
    raw[4:6] = struct.pack("<H", VISION_PROTOCOL_VERSION + 1)
    with pytest.raises(ValueError, match="版本"):
        decode_vision_frame(raw)


def test_rejects_truncated_and_overlong_frames() -> None:
    raw = encode_vision_frame(_full_frame())
    with pytest.raises(ValueError):
        decode_vision_frame(raw[:-1])
    with pytest.raises(ValueError):
        decode_vision_frame(raw + b"\x00")
    with pytest.raises(ValueError, match="帧太短"):
        decode_vision_frame(b"\x00" * 8)


def test_rejects_unknown_shape_and_type_codes() -> None:
    # 注意：篡改后**必须重算 CRC**，否则先被 CRC 校验拦住（那是设计如此），
    # 就测不到"未知编码要被拒绝"这一层了。
    import zlib
    off = 32 + 16 + 24                   # 第一条障碍的记录起点
    for delta, needle in ((4, "shape"), (8, "type")):
        raw = bytearray(encode_vision_frame(_full_frame()))
        raw[off + delta:off + delta + 4] = struct.pack("<i", 99)
        raw[-4:] = struct.pack("<I", zlib.crc32(bytes(raw[:-4])) & 0xFFFFFFFF)
        with pytest.raises(ValueError, match=needle):
            decode_vision_frame(raw)


def test_encode_rejects_an_invalid_frame() -> None:
    """语义不合法的帧**不许出去**（编码前先 validate）。"""
    bad = _frame(gaps=[GapObs(id=1, center=(0.0, 0.0), width=60.0,
                              blockers=(1, 1))])      # 两段墙是同一 id
    with pytest.raises(ValueError, match="blockers"):
        encode_vision_frame(bad)


def test_non_finite_geometry_is_rejected() -> None:
    with pytest.raises(ValueError, match="有限"):
        encode_vision_frame(_frame(player=PlayerState(x=float("nan"), y=0.0,
                                                     radius=1.0)))


# --------------------------------------------------------------------------
# 三、契约自述与布局必须一致
# --------------------------------------------------------------------------
def test_frame_size_matches_the_real_encoding() -> None:
    for n_obs, n_gaps in ((0, 0), (2, 1), (4, 3)):
        # 约束：每个 gap 的 blockers 必须指回**本帧**存在的障碍 id
        # （VisionFrame.validate 强制）。所以障碍 id 取 0..n_obs-1，
        # 缺口一律引用 (0, 1)。
        f = _frame(
            obstacles=[Obstacle(id=i, shape=ObstacleShape.RECT,
                                type=ObstacleType.WALL_WITH_GAP,
                                x=1.0, y=1.0, half_w=1.0, half_h=1.0)
                       for i in range(n_obs)],
            gaps=[GapObs(id=i, center=(0.0, 0.0), width=60.0, blockers=(0, 1))
                  for i in range(n_gaps)],
        )
        raw = encode_vision_frame(f)
        assert len(raw) == vision_frame_size(n_obs, n_gaps), (n_obs, n_gaps)
        assert len(raw) == 76 + OBSTACLE_BYTES * n_obs + GAP_BYTES * n_gaps


def test_describe_protocol_is_self_consistent() -> None:
    """自述里的每个数与真实实现必须对得上（HDL 会照它对齐）。"""
    d = describe_vision_protocol()
    assert d["magic"] == VISION_MAGIC and d["magic_bytes"] == "BHV1"
    assert d["version"] == VISION_PROTOCOL_VERSION
    assert d["quantisation_scale"] == VISION_Q
    assert d["frame_size"] == "76 + 36*n_obstacles + 36*n_gaps"
    raw = encode_vision_frame(_full_frame())
    assert len(raw) == 76 + 36 * 2 + 36 * 1
    # 自述必须能被 JSON 序列化（它会被写进报告/测试向量）
    import json
    json.dumps(d)


def test_magic_bytes_are_the_documented_ascii() -> None:
    assert struct.pack("<I", VISION_MAGIC) == b"BHV1"


def test_wire_format_carries_no_velocities() -> None:
    """**速度不进线协议** —— 把它钉住，免得以后有人"顺手加上"。

    理由（不是省字节）：``PlayerState.validate()`` 要求
    ``speed_measured <= speed``，而 ``speed`` 是速度上限（配置量），
    PL 不知道。编进帧以后解码出来的 ``PlayerState`` 会**自己校验失败**，
    要绕过就只能编一个 speed —— 那正是本项目一路拒绝的做法。
    速度由 PS 从相邻帧的 ``stamp`` 之差推（见移植规划 §3）。
    """
    from standard.vision import vision_frame as vf
    d = vf.describe_vision_protocol()
    assert "velocity" not in d["frame_size"]
    blob = encode_vision_frame(_full_frame())
    # 角色记录现在只有 4 个字：x, y, radius, flags
    assert vf.PLAYER_BYTES == 4 * 4
    assert len(blob) == 76 + 36 * 2 + 36 * 1
