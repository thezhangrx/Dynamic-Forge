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


# --------------------------------------------------------------------------
# 速度要能**贴回调用方自己的对象**上（矩形/检测框），不能只留在轨迹里
#
# 板子上的用处就是这件事：线协议带障碍的几何（id/x/y/半宽/半高/转角），
# 但**不带速度**；PS 必须把推出来的速度填回这一帧的那些矩形上，决策才看得见。
# 而轨迹 id 是跟踪器自己发的，和帧里的 id 不是一回事（帧里的 id 是当帧序号，
# 见 vision_top.v 的 `seg_id[seg_k] <= n_obs + 1`），所以只能靠 det→track 的对应表。
# --------------------------------------------------------------------------
def test_last_assignment_aligns_with_returned_tracks():
    frames, stamps = _uniform(0.30, -0.10)
    tr = Tracker()
    for pts, ts in zip(frames, stamps):
        tracks = tr.update(pts, ts)
        asg = tr.last_assignment
        assert len(asg) == len(tracks), "对应表必须与返回的轨迹等长同序"
        assert all(j == 0 for j in asg), "只有一条轨迹，它就应该对应唯一的检测"


def test_last_assignment_is_none_for_extrapolated_track():
    """目标丢了一帧：那条轨迹靠惯性外推 → 对应关系是 None（不是乱指一个检测）。"""
    tr = Tracker()
    tr.update([(0.0, 0.0)], 0.0)
    tr.update([(0.1, 0.0)], 1.0 / 30)
    tr.update([], 2.0 / 30)                       # 这一帧什么都没有
    assert tr.last_assignment == [None], tr.last_assignment


def test_velocity_lands_on_the_right_object():
    """速度必须贴到**正确的那个矩形**上，而且是"匹配到它的那条轨迹"的速度。

    这是这套接口存在的理由：贴错了不会报错，只会让决策以为"那面墙在朝我压过来"。
    帧里的 id 是当帧序号（顺序会变），所以这里每帧把两个检测的顺序翻转，
    逼着关联必须靠位置。
    """
    dt = 1.0 / 30
    tr = Tracker()
    n = 60
    last = None
    for k in range(n):
        t = k * dt
        left = (0.30 * t, 0.0)
        right = (10.0 - 0.20 * t, 0.0)
        dets = [left, right] if k % 2 == 0 else [right, left]   # 顺序每帧翻转
        tracks = tr.update(dets, t)
        asg = tr.last_assignment
        assert len(asg) == len(tracks)

        # ① 贴的是不是"匹配到它的那条轨迹"的值（逐帧精确对账，与收敛无关）
        attached: list[float | None] = [None] * len(dets)
        for trk, j in zip(tracks, asg):
            if j is not None:
                assert attached[j] is None, "一个检测不该被两条轨迹匹配"
                attached[j] = trk.vx
        for j, v in enumerate(attached):
            if k == 0:
                continue                    # 首帧没有历史 → 速度按设计为 0
            assert v is not None, f"第 {k} 帧第 {j} 个检测没被任何轨迹认领"

        # ② 符号从第 2 帧起就该对（关联错的话符号会立刻反）
        if k >= 2:
            for j, det in enumerate(dets):
                want_sign = 1.0 if det is left else -1.0
                assert attached[j] * want_sign > 0.0, (
                    f"第 {k} 帧第 {j} 个检测的速度符号错了：{attached[j]}")
        last = (dets, attached)

    # ③ 跑到收敛：α-β 需要几十帧（≈1 秒），末帧速度要接近真值
    dets, attached = last
    for j, det in enumerate(dets):
        want = 0.30 if det is left else -0.20
        assert attached[j] == pytest.approx(want, abs=0.02), (
            f"末帧第 {j} 个检测速度应为 {want}，得到 {attached[j]}")


# --------------------------------------------------------------------------
# 位置跳变不能变成速度（真实素材上量出来的那个坑）
#
# 实测（data/vision/run4/screen.mp4 前 60 帧，30fps）：带合并一变，段的**中心**
# 单帧横移 24 px。那是"段变宽了"，不是"墙在横着飞"。关联门限 24 恰好放它进来，
# 于是推出来的速度瞬间 -66 px/s，还要十几帧才衰减 —— 真实运动只有 15~30 px/s。
# --------------------------------------------------------------------------
def test_position_jump_does_not_become_velocity():
    tr = Tracker()
    # 前几帧静止（把速度收敛到 ~0）
    for k in range(6):
        tr.update([(100.0, 50.0)], k / 30.0)
    assert tr.tracks[0].vx == pytest.approx(0.0, abs=1e-9)

    # 第 7 帧：中心突然横移 24（=带合并），这是**门限之内**的
    tracks = tr.update([(76.0, 50.0)], 6 / 30.0)
    assert tracks[0].vx == 0.0, f"跳变不该变成速度，得到 vx={tracks[0].vx}"
    assert tracks[0].x == pytest.approx(76.0), "位置还是要跟上（否则轨迹会越飘越远）"
    assert tr.n_relocalized == 1

    # 之后几帧仍在 76：速度继续保持 0，不会"复活"
    for k in range(7, 12):
        tracks = tr.update([(76.0, 50.0)], k / 30.0)
    assert tracks[0].vx == 0.0
    assert tr.n_relocalized == 1, "只有那一帧是跳变"


def test_realistic_fast_motion_is_still_tracked():
    """守卫不能把**真运动**也当成跳变：残差对匀速目标是小的（预测已吃掉速度）。"""
    tr = Tracker()
    for k in range(40):                      # 2 单位/帧 = 60 单位/s，比实测真实运动快
        tr.update([(2.0 * k, 0.0)], k / 30.0)
    last = tr.tracks[0]
    assert last.vx == pytest.approx(60.0, abs=2.0), f"真运动被挡了：vx={last.vx}"
    assert tr.n_relocalized == 0, "整段匀速不该触发守卫"


def test_residual_threshold_sits_between_real_and_fake():
    """8 这个取值要有依据：真实步长（~2/帧）远小于它，假跳变（24）远大于它。"""
    assert Tracker().max_residual == 8.0


def test_fast_target_still_acquires_velocity_after_confirm():
    """步长**大于** ``max_residual`` 的真运动：连续可疑 → 改判 → 拿到速度。

    这条是既有测试（``test_pipeline.py`` 的 10 单位/帧轨迹）逼出来的：
    只写"残差超阈值就不更新速度"会让这种目标**永远**是 0 —— 位置看着还对，
    速度静默错。所以必须有"连续可疑就改判"的计数器。
    """
    tr = Tracker(gate=100.0)
    tracks = []
    for k in range(20):
        tracks = tr.update([(10.0 * k, 0.0)], k * 0.05)
    assert tracks[0].speed > 0, f"10 单位/帧 的真运动不该永远拿不到速度：{tracks[0].speed}"
    assert tr.n_relocalized >= 1, "第一帧应当先被判成可疑（然后改判）"


def test_jump_does_not_corrupt_an_established_velocity():
    """已经建立了速度之后遇到单帧跳变：位置落位，**速度保持**（不清零、不被改写）。"""
    tr = Tracker(gate=100.0)
    for k in range(20):
        tr.update([(2.0 * k, 0.0)], k * 0.05)
    v_before = tr.tracks[0].vx
    assert v_before > 0, "先要有一个正经速度"

    tracks = tr.update([(2.0 * 20 + 24.0, 0.0)], 20 * 0.05)     # 单帧横移 24
    assert tracks[0].x == pytest.approx(2.0 * 20 + 24.0), "位置要落位"
    assert tracks[0].vx == pytest.approx(v_before), \
        f"单帧跳变不该改写已有速度：{v_before} → {tracks[0].vx}"


# --------------------------------------------------------------------------
# 快速起步：物体**出现时就在动**（轨迹刚建立，速度从 0 起）
#
# 实测（30fps）：常规 β=0.0728 下要 19 帧才追到 90%，而决策的 t_arr = 距离/v_n
# 直接吃这个速度 —— 墙刚在以 60/s 逼近时 t_arr 会被高估成 45 s（真 3.3 s）。
# 仿真里速度是真值、没有这个滞后，所以这是**部署与训练不一致**的一处来源。
# --------------------------------------------------------------------------
def test_fast_start_converges_when_object_appears_moving():
    V, DT = 60.0, 1.0 / 30.0

    def frames_to_90(**kw):
        tr, err = Tracker(**kw), []
        for k in range(40):
            err.append(abs(tr.update([(V * k * DT, 0.0)], k * DT)[0].vx - V))
        for k in range(len(err)):
            if err[k] <= 0.1 * V:
                return k
        return None

    slow = frames_to_90(fast_updates=0)
    fast = frames_to_90()
    assert slow is not None and fast is not None
    assert fast <= 8, f"快速起步后应当 8 帧内追到 90%，实测 {fast} 帧"
    assert fast < slow, f"必须比常规 β 快：{fast} vs {slow}"


def test_fast_start_does_not_change_steady_state_noise():
    """代价只在轨迹头几帧；**稳态**噪声必须与常规 β 完全一样（否则就是拿噪声换收敛）。"""
    import random
    def steady(**kw):
        rng = random.Random(7)
        tr, vs = Tracker(**kw), []
        for k in range(150):
            t = tr.update([(100.0 + rng.gauss(0, 0.5), 50.0 + rng.gauss(0, 0.5))],
                          k * 1.0 / 30.0)[0]
            if k >= 40:
                vs.append(abs(t.vx))
        vs.sort()
        return vs[int(len(vs) * 0.9)]
    # ⚠ 不能要求"逐位相同"：快速起步改变了前几帧的状态，两条轨迹是**同一个随机过程的
    #   两次不同实现**，采到的样本自然不同。口径是"稳态噪声量级一致"（都在 1 单位/秒
    #   上下，而真实速度是 15~200）。
    a, b = steady(), steady(fast_updates=0)
    assert abs(a - b) <= 1.0, f"稳态噪声不该被快速起步改变：{a} vs {b}"
