"""任务二回归测试：从视频取速度（跨帧关联 + α-β 滤波）。

不需要摄像头：给一条**合成轨迹**（世界坐标 + 时间戳），看能不能把真实速度恢复出来。
"""

from __future__ import annotations

import math

import pytest

from vision.motion import DEFAULT_ALPHA, DEFAULT_BETA, Tracker, track_points


def _uniform(vx, vy, *, fps=30, seconds=1.0, start=(0.2, 0.2)):
    """造一条匀速直线轨迹，返回 ``(每帧点, 每帧时间戳)``。"""
    dt = 1.0 / fps
    n = max(2, int(seconds * fps))
    frames, stamps = [], []
    for k in range(n):
        t = k * dt
        frames.append([(start[0] + vx * t, start[1] + vy * t)])
        stamps.append(t)
    return frames, stamps


# --------------------------------------------------------------------------
# 能不能把速度恢复出来
# --------------------------------------------------------------------------
def test_uniform_motion_velocity_is_recovered():
    frames, stamps = _uniform(0.30, -0.10)
    last = track_points(frames, stamps)[-1][0]
    assert last.vx == pytest.approx(0.30, abs=0.02)
    assert last.vy == pytest.approx(-0.10, abs=0.02)
    assert last.speed == pytest.approx(math.hypot(0.30, 0.10), abs=0.02)


def test_dt_comes_from_stamp_not_frame_index():
    """同样多的帧、不同帧率 → 恢复出的速度必须一致（都是 0.25 m/s）。

    这正是三个参考项目翻车的地方：ByteTrack 的 dt 写死为 1，速度单位变成"像素/帧"。
    """
    got = {}
    for fps in (10, 30, 60):
        dt = 1.0 / fps
        n = int(fps)
        frames = [[(0.25 * k * dt, 0.5)] for k in range(n)]
        stamps = [k * dt for k in range(n)]
        got[fps] = track_points(frames, stamps)[-1][0].vx
    for fps, v in got.items():
        assert v == pytest.approx(0.25, abs=0.10), f"{fps} fps 估出 {v}"
    # 帧率越高，收敛越好 —— 速度估计精度受帧率限制
    assert abs(got[60] - 0.25) < abs(got[10] - 0.25)


def test_two_targets_crossing_keep_separate_ids():
    dt = 1 / 30
    frames, stamps = [], []
    for k in range(24):
        t = k * dt
        frames.append([(0.1 + 0.2 * t, 0.5), (0.9 - 0.2 * t, 0.5)])
        stamps.append(t)
    tracks = track_points(frames, stamps)[-1]
    assert len(tracks) == 2
    assert tracks[0].id != tracks[1].id
    vxs = sorted(t.vx for t in tracks)
    assert vxs[0] == pytest.approx(-0.2, abs=0.05)
    assert vxs[1] == pytest.approx(+0.2, abs=0.05)


def test_id_is_stable_across_frames():
    frames, stamps = _uniform(0.1, 0.0)
    seq = track_points(frames, stamps)
    ids = {t.id for frame in seq for t in frame}
    assert len(ids) == 1, "一个目标自始至终只能有一个 id"


# --------------------------------------------------------------------------
# 边界情况
# --------------------------------------------------------------------------
def test_lost_target_survives_a_few_frames_then_disappears():
    tr = Tracker(max_misses=3)
    for k in range(5):
        tr.update([(0.1 * k, 0.0)], k * 0.1)
    assert len(tr.tracks) == 1
    for k in range(3):                      # 丢 3 帧，还在
        tr.update([], 0.5 + 0.1 * k)
    assert len(tr.tracks) == 1
    tr.update([], 1.0)                      # 第 4 帧超过 max_misses
    assert tr.tracks == []


def test_large_gap_resets_velocity_instead_of_extrapolating():
    """两帧间隔过大时不能拿旧速度乱推。"""
    tr = Tracker(max_dt=0.5)
    tr.update([(0.0, 0.0)], 0.0)
    tr.update([(0.1, 0.0)], 0.1)
    assert tr.tracks[0].vx > 0.0
    tr.update([(0.5, 0.0)], 1.5)            # dt = 1.4 s
    assert tr.tracks[0].vx == 0.0
    assert tr.tracks[0].x == pytest.approx(0.5)


def test_non_monotonic_stamp_is_rejected():
    tr = Tracker()
    tr.update([(0.0, 0.0)], 1.0)
    with pytest.raises(ValueError, match="单调递增"):
        tr.update([(0.1, 0.0)], 0.5)


def test_new_target_gets_zero_velocity_on_first_frame():
    tr = Tracker()
    tracks = tr.update([(0.3, 0.4)], 0.0)
    assert len(tracks) == 1
    assert tracks[0].vx == 0.0 and tracks[0].vy == 0.0
    assert tracks[0].speed == 0.0


def test_gate_rejects_unrelated_detection():
    """门限外的检测不能被当成同一个目标（否则会把不同物体关联到一起）。"""
    tr = Tracker(gate=0.1)
    tr.update([(0.0, 0.0)], 0.0)
    tracks = tr.update([(5.0, 5.0)], 0.1)   # 远在天边
    assert len(tracks) == 2, "应新建一条轨迹，而不是把两条并成一条"


def test_reset_clears_everything():
    tr = Tracker()
    tr.update([(0.0, 0.0)], 0.0)
    tr.reset()
    assert tr.tracks == []
    tr.update([(1.0, 1.0)], 5.0)            # reset 后第一帧不该报"stamp 倒退"
    assert len(tr.tracks) == 1


def test_default_gains_come_from_bytetrack_measurement():
    """默认增益不是拍脑袋：ByteTrack 实测 Kalman 增益与框高无关，可退化成常数。"""
    assert 0.5 < DEFAULT_ALPHA < 0.8
    assert 0.0 < DEFAULT_BETA < 0.2
