"""视觉 → 决策 IO 接口的测试。

**全部用合成帧**：不 import cv2、不打开摄像头、不依赖 ``core/vision`` 的实现。
这也顺带验证了适配器的鸭子类型契约——测试里喂的是 **dict**，不是 dataclass。
"""

from __future__ import annotations

import math

import pytest

from cpu.adapters.vision_adapter import (
    VisionAdapterError,
    VisionWorldConfig,
    VisionWorldStream,
    dt_between,
    frame_to_worldview,
    gaps_from_frame,
    player_vector,
    rects_from_frame,
)
from cpu.gap_avoid import AlgoConfig, GapMemory, plan, to_command

# ---------------------------------------------------------------------------
# 合成帧
# ---------------------------------------------------------------------------
WALL_Y = 400.0
WALL_HH = 13.0
GAP_LO, GAP_HI = 240.0, 400.0          # 缺口 [240, 400]，宽 160


def rect(oid, cx, half_w, *, cy=WALL_Y, half_h=WALL_HH, rot=0.0, **kw):
    d = {"id": oid, "shape": "rect", "x": cx, "y": cy,
         "vx": 0.0, "vy": 0.0, "half_w": half_w, "half_h": half_h,
         "rotation": rot, "angular_velocity": 0.0}
    d.update(kw)
    return d


def two_segment_wall(vy=0.0):
    """两段共线矩形 + 中间 160 宽的缺口（和平台 wall_with_gap 的几何一致）。"""
    return [rect(1, 120.0, 120.0, vy=vy), rect(2, 520.0, 120.0, vy=vy)]


def frame(seq=0, stamp=0.0, *, player=(100.0, 50.0, 10.0, 200.0),
          obstacles=(), gaps=(), frame_id="field", units="m", calib=None, **kw):
    if player is None:
        p = None
    else:
        px, py, r, sp = player
        p = {"x": px, "y": py, "vx": 0.0, "vy": 0.0,
             "radius": r, "speed": sp, "alive": True}
    d = {
        "seq": seq, "stamp": stamp, "frame_id": frame_id,
        "units": units, "metres_per_unit": 1.0,
        "player": p,
        "obstacles": list(obstacles),
        "gaps": list(gaps),
        "calibration": calib,
    }
    d.update(kw)
    return d


CFG = VisionWorldConfig(field_w=640.0, field_h=480.0)


# ---------------------------------------------------------------------------
# dt：必须用 stamp，且坏时间戳要报错
# ---------------------------------------------------------------------------
def test_first_frame_uses_configured_default_dt():
    _, meta = frame_to_worldview(frame(stamp=1.0), CFG, prev_frame=None)
    assert meta.used_default_dt is True
    assert meta.dt == pytest.approx(CFG.dt_default)


def test_dt_comes_from_stamp_not_seq():
    f0 = frame(seq=0, stamp=10.0)
    f1 = frame(seq=1, stamp=10.5)        # seq 差 1，但时间差 0.5s
    _, meta = frame_to_worldview(f1, CFG, prev_frame=f0)
    assert meta.used_default_dt is False
    assert meta.dt == pytest.approx(0.5)     # 若是 seq 差会得到 1.0


def test_non_positive_dt_raises():
    with pytest.raises(VisionAdapterError, match="stamp 差必须 > 0"):
        dt_between(frame(stamp=5.0), frame(stamp=5.0), CFG)
    with pytest.raises(VisionAdapterError, match="stamp 差必须 > 0"):
        dt_between(frame(stamp=1.0), frame(stamp=2.0), CFG)   # 时间倒流


def test_nan_stamp_raises():
    with pytest.raises(VisionAdapterError, match="stamp 不是有限数"):
        dt_between(frame(stamp=float("nan")), frame(stamp=1.0), CFG)


# ---------------------------------------------------------------------------
# 契约：坐标系 / 单位 / player
# ---------------------------------------------------------------------------
def test_wrong_frame_id_raises():
    with pytest.raises(VisionAdapterError, match="frame_id"):
        frame_to_worldview(frame(frame_id="camera"), CFG)


def test_uncalibrated_pixels_are_rejected():
    """units='px' 且没有单应 —— 坐标还是像素，必须报错而不是照跑。"""
    with pytest.raises(VisionAdapterError, match="还是\\*\\*像素\\*\\*"):
        frame_to_worldview(frame(units="px", calib=None), CFG)


def test_uncalibrated_pixels_allowed_when_explicitly_disabled():
    cfg = VisionWorldConfig(field_w=640.0, field_h=480.0, require_calibration=False)
    view, _ = frame_to_worldview(frame(units="px", calib=None), cfg)
    assert view.field_w == 640.0


def test_calibrated_pixels_are_accepted():
    calib = {"id": "cam0", "H_field_from_image": [1.0] * 9}
    _, meta = frame_to_worldview(frame(units="px", calib=calib), CFG)
    assert meta.n_rects_kept == 0


def test_missing_player_raises():
    with pytest.raises(VisionAdapterError, match="没有 player"):
        frame_to_worldview(frame(player=None), CFG)


def test_bad_player_geometry_raises():
    with pytest.raises(VisionAdapterError, match="player 位置/半径非法"):
        player_vector(frame(player=(0.0, 0.0, 0.0, 200.0)), CFG)


def test_unknown_speed_falls_back_to_config_and_is_flagged():
    vec, used_default = player_vector(frame(player=(1.0, 2.0, 10.0, 0.0)), CFG)
    assert used_default is True
    assert vec[5] == pytest.approx(CFG.default_speed)
    # 已知 speed 时不应打标记
    _, used_default2 = player_vector(frame(player=(1.0, 2.0, 10.0, 123.0)), CFG)
    assert used_default2 is False


def test_player_vector_layout_matches_p_star_indices():
    from cpu.gap_avoid import P_ALIVE, P_RADIUS, P_SPEED, P_X, P_Y
    vec, _ = player_vector(frame(player=(7.0, 8.0, 9.0, 111.0)), CFG)
    assert vec[P_X] == 7.0 and vec[P_Y] == 8.0
    assert vec[P_RADIUS] == 9.0 and vec[P_SPEED] == 111.0
    assert vec[P_ALIVE] == 1.0


# ---------------------------------------------------------------------------
# 障碍筛选：只留可用矩形，且逐条计数
# ---------------------------------------------------------------------------
def test_only_rectangles_are_kept_and_circles_counted():
    obs = [rect(1, 120.0, 120.0), {"id": 9, "shape": "circle", "x": 10.0, "y": 10.0}]
    meta = _meta()
    rects = rects_from_frame(frame(obstacles=obs), CFG, meta)
    assert [r.ent_id for r in rects] == [1]
    assert meta.dropped_not_rect == 1


def test_invalid_and_occluded_are_dropped_by_default():
    obs = [rect(1, 120.0, 120.0), rect(2, 520.0, 120.0, valid=False),
           rect(3, 300.0, 10.0, occluded=True)]
    meta = _meta()
    rects = rects_from_frame(frame(obstacles=obs), CFG, meta)
    assert [r.ent_id for r in rects] == [1]
    assert meta.dropped_invalid == 1 and meta.dropped_occluded == 1


def test_low_score_dropped_when_threshold_set():
    cfg = VisionWorldConfig(min_score=0.5)
    obs = [rect(1, 120.0, 120.0, score=0.9), rect(2, 520.0, 120.0, score=0.2)]
    meta = _meta()
    rects = rects_from_frame(frame(obstacles=obs), cfg, meta)
    assert [r.ent_id for r in rects] == [1]
    assert meta.dropped_low_score == 1


def test_bad_geometry_is_dropped_not_raised():
    obs = [rect(1, 120.0, 120.0),
           {"id": 2, "shape": "rect", "x": 1.0, "y": 1.0, "half_w": 0.0, "half_h": 5.0},
           {"id": 3, "shape": "rect", "x": float("nan"), "y": 1.0,
            "half_w": 5.0, "half_h": 5.0}]
    meta = _meta()
    rects = rects_from_frame(frame(obstacles=obs), CFG, meta)
    assert [r.ent_id for r in rects] == [1]
    assert meta.dropped_bad_geometry == 2


def test_meta_summary_is_json_friendly():
    meta = _meta()
    rects_from_frame(frame(obstacles=two_segment_wall()), CFG, meta)
    s = meta.summary()
    assert s["rects_kept"] == 2 and isinstance(s["dropped"], dict)


def _meta():
    from cpu.adapters.vision_adapter import VisionMeta
    return VisionMeta()


# ---------------------------------------------------------------------------
# 缺口观测
# ---------------------------------------------------------------------------
def test_gaps_from_frame_returns_center_and_width():
    g = {"id": 1, "center": (320.0, WALL_Y), "width": 160.0,
         "blockers": (1, 2), "reliable": True}
    assert gaps_from_frame(frame(gaps=[g])) == [(320.0, WALL_Y, 160.0)]


def test_gaps_from_frame_skips_malformed():
    assert gaps_from_frame(frame(gaps=[{"id": 1, "width": 0.0},
                                       {"id": 2, "center": (1.0, 2.0)}])) == []


# ---------------------------------------------------------------------------
# 端到端：合成帧 → plan()
# ---------------------------------------------------------------------------
def _view_and_decide(frm, prev=None, cfg=None):
    view, meta = frame_to_worldview(frm, CFG, prev_frame=prev)
    mem = GapMemory()
    dec = plan(view, mem, cfg or AlgoConfig(forward=(0.0, 1.0)), 2.0)
    return view, meta, dec


def test_end_to_end_produces_a_usable_decision():
    view, meta, dec = _view_and_decide(frame(stamp=1.0, obstacles=two_segment_wall()))
    assert meta.n_rects_kept == 2
    assert dec.tier not in ("", None)
    assert math.isfinite(dec.wx) and math.isfinite(dec.wy)
    wx, wy = to_command(dec)
    assert math.hypot(wx, wy) <= 1.0 + 1e-9


def test_end_to_end_is_deterministic_under_rect_reordering():
    """障碍顺序变了，决策必须一模一样（C++ 侧 unordered_map 顺序不可依赖）。"""
    a = two_segment_wall()
    b = list(reversed(a))
    _, _, dec_a = _view_and_decide(frame(stamp=1.0, obstacles=a))
    _, _, dec_b = _view_and_decide(frame(stamp=1.0, obstacles=b))
    assert (dec_a.wx, dec_a.wy, dec_a.tier) == (dec_b.wx, dec_b.wy, dec_b.tier)


def test_wall_reconstruction_matches_the_vision_gap_hint():
    """算法自己重建的缺口宽度，应当和视觉侧直接给的缺口一致（互相校核）。"""
    g = {"id": 1, "center": (320.0, WALL_Y), "width": GAP_HI - GAP_LO,
         "blockers": (1, 2), "reliable": True}
    _, _, dec = _view_and_decide(frame(stamp=1.0, obstacles=two_segment_wall(),
                                       gaps=[g]))
    assert dec.gap == pytest.approx(GAP_HI - GAP_LO, abs=1e-6)


def test_stream_reuses_memory_and_tracks_dt():
    stream = VisionWorldStream(CFG)
    f0 = frame(seq=0, stamp=0.0, obstacles=two_segment_wall())
    f1 = frame(seq=1, stamp=1.0 / 60.0, obstacles=two_segment_wall())
    view0, _ = stream.step(f0, cfg=AlgoConfig(forward=(0.0, 1.0)))
    view1, _ = stream.step(f1, cfg=AlgoConfig(forward=(0.0, 1.0)))
    assert stream.frames == 2
    assert view0.dt == pytest.approx(CFG.dt_default)
    assert view1.dt == pytest.approx(1.0 / 60.0)
    assert stream.last_meta.used_default_dt is False
    # 记忆是同一个对象，没有被重置
    assert stream.memory.step_index == 1


def test_stream_reset_clears_memory_and_prev_frame():
    stream = VisionWorldStream(CFG)
    stream.step(frame(seq=0, stamp=0.0, obstacles=two_segment_wall()))
    stream.reset()
    assert stream.frames == 0 and stream.prev_frame is None
    assert stream.memory.step_index == -1
