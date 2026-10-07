"""``align`` / ``identity`` / ``reconcile`` 的回归测试。

**完全自包含**：不读文件、不用 numpy/opencv、不需要摄像头。

这里锁住的每一条都对应一次**实测踩到的错**，不是想当然的性质：

1. 时戳倒退必须**炸**，不能静默（队列积压会静默污染速度）；
2. 关联的 ``max_diff`` 硬门限 —— 没有它，"最邻近"会让 `clip.avi`
   那种"声明 25 fps 实际 8.6"的录像把差 116 ms 的帧配对成功；
3. 插值**绝不外推**，且角度要解环绕（从 +179° 到 -179° 是 2° 不是 0°）；
4. Umeyama 必须把**刚体错位**与**残余**分开报 —— 实测同一份识别代码，
   标定错位时误差 261 场地单位、换同机位标定后 1.78，靠这个区分才定位到病因；
5. 时钟偏移判据靠**平移量随速度等比缩放**，不能靠平移方向 ——
   运动近一维时沿运动方向的空间偏移与时间偏移**数学上不可分**；
6. ``IDSW`` 用"上次出现时"的 id，不用"上一帧"（墙在画面边缘进出是常态）；
7. ``IDSW=0`` 与 ``IDF1=1.0`` 可以在身份其实错了的时候同时成立（门限太宽）。
"""

from __future__ import annotations

import math

import pytest

from vision.align import (
    AlignError,
    TimeOffsetEstimator,
    angle_diff,
    associate,
    associate_short_to_long,
    check_monotonic,
    interpolate_angle,
    interpolate_linear,
    umeyama_2d,
)
from vision.identity import (
    evaluate_identity,
    hungarian,
    similarity_from_points,
)
from vision.reconcile import FrameEstimate, FrameTruth, reconcile

# ---------------------------------------------------------------------------
# ① 时戳纪律
# ---------------------------------------------------------------------------
def test_monotonic_rejects_backwards() -> None:
    """时戳倒退是**故障**，不是噪声（kalibr 的 TimeWentBackwardsException）。"""
    with pytest.raises(AlignError, match="倒退"):
        check_monotonic([0.0, 0.1, 0.05], name="cam")


def test_monotonic_rejects_duplicates_by_default() -> None:
    with pytest.raises(AlignError, match="重复"):
        check_monotonic([0.0, 0.1, 0.1], name="cam")


def test_monotonic_allows_duplicates_when_asked() -> None:
    check_monotonic([0.0, 0.1, 0.1], name="cam", strict=False)


def test_monotonic_rejects_non_finite() -> None:
    with pytest.raises(AlignError, match="有限数"):
        check_monotonic([0.0, float("nan")], name="cam")


# ---------------------------------------------------------------------------
# ② 关联：硬门限
# ---------------------------------------------------------------------------
def test_associate_hard_gate_refuses_far_pairs() -> None:
    """门限之外的帧**配不上**，而不是配到"最邻近"。

    实测：`clip.avi` 声明 25 fps 而实际 8.6 fps，相邻帧时差 116 ms。
    没有门限时最邻近会把它们全配上，于是误差里混进的是配对错。
    """
    ref = [0.0, 0.1, 0.2]
    other = [0.5, 0.6, 0.7]                     # 整体偏 0.5s
    with pytest.raises(AlignError, match="一对都配不上"):
        associate(ref, other, max_diff=0.05)


def test_associate_offset_recovers_the_pairs() -> None:
    """给了正确的 offset 就能配上 —— offset 只用于关联。"""
    ref = [0.0, 0.1, 0.2]
    other = [0.5, 0.6, 0.7]
    a = associate(ref, other, max_diff=0.02, offset_other=-0.5)
    assert len(a) == 3
    assert a.idx_other == (0, 1, 2)
    assert max(a.diffs) < 1e-9


def test_associate_short_to_long_keeps_every_short_point() -> None:
    """以**短**的那条为基准：短序列的点一个都不丢（evo: sync.py:99-104）。"""
    short = [0.0, 0.2, 0.4]
    long_ = [i * 0.01 for i in range(50)]
    a, b_longer = associate_short_to_long(short, long_, max_diff=0.02)
    assert b_longer is True
    assert len(a) == len(short)


def test_associate_reports_time_diffs() -> None:
    """配对时差要能读出来 —— 否则"误差大"是识别差还是配对差分不清。"""
    a = associate([0.0, 1.0], [0.0, 1.02], max_diff=0.05)
    assert a.max_diff == pytest.approx(0.02, abs=1e-9)


# ---------------------------------------------------------------------------
# ③ 插值：不外推 + 角度解环绕
# ---------------------------------------------------------------------------
def test_interpolate_clamps_and_never_extrapolates() -> None:
    """目标时戳超出区间就**夹住**，绝不外推（nuscenes.py:354-355）。

    外推出来的真值看起来是有效的，比没有真值更危险。
    """
    stamps = [0.0, 1.0]
    vals = [0.0, 10.0]
    out = interpolate_linear(stamps, vals, [-5.0, 0.5, 99.0])
    assert out == [0.0, 5.0, 10.0]


def test_interpolate_linear_midpoint() -> None:
    assert interpolate_linear([0.0, 1.0, 2.0], [0.0, 10.0, 20.0], [0.5]) == [5.0]


def test_interpolate_angle_wraps() -> None:
    """+179° -> -179° 的实际变化是 **2°**，不是穿过 180°。

    墙有 ``rotation``、缺口有 ``axis``，逐帧比角度必须处理这件事。
    """
    out = interpolate_angle([0.0, 1.0], [math.radians(179.0), math.radians(-179.0)], [0.5])
    assert abs(abs(out[0]) - math.pi) < math.radians(1.0), out


def test_angle_diff_wraps() -> None:
    assert abs(angle_diff(math.radians(179.0), math.radians(-179.0))
               - math.radians(-2.0)) < 1e-9


# ---------------------------------------------------------------------------
# ④ Umeyama：把"标定错"与"识别错"分开
# ---------------------------------------------------------------------------
def _ring(n: int = 30) -> list[tuple[float, float]]:
    return [(320.0 + 50.0 * math.cos(2 * math.pi * i / n),
             240.0 + 40.0 * math.sin(2 * math.pi * i / n)) for i in range(n)]


def test_umeyama_recovers_a_known_transform() -> None:
    th, tx, ty = math.radians(7.0), 12.0, -5.0
    src = _ring()
    dst = [(math.cos(th) * x - math.sin(th) * y + tx,
            math.sin(th) * x + math.cos(th) * y + ty) for x, y in src]
    r = umeyama_2d(src, dst)
    assert math.degrees(r.theta) == pytest.approx(7.0, abs=1e-6)
    # ``origin_translation`` 才是构造时的平移 (tx,ty)；
    # ``translation`` 是**质心位移**，含"旋转 x 质心到原点距离"这一项。
    assert r.origin_translation == pytest.approx(math.hypot(tx, ty), abs=1e-6)
    assert r.translation > r.origin_translation    # 圆环离原点远，杠杆明显
    assert r.rms < 1e-6


def test_umeyama_separates_registration_from_recognition() -> None:
    """纯刚性错位：``rms_before`` 大而 ``rms`` 近 0。

    这是整套工具的**核心判据**：实测用错机位的标定去对账得到 261 场地单位，
    换成同机位降到 1.78，当时只有一个数字看不出病因。
    """
    src = _ring()
    shifted = [(x + 9.0, y + 23.0) for x, y in src]
    r = umeyama_2d(src, shifted)
    assert r.rms_before == pytest.approx(math.hypot(9.0, 23.0), abs=1e-6)
    assert r.rms < 1e-6                       # 全是刚体错位，识别没有误差
    assert r.rigid_part == pytest.approx(r.rms_before, abs=1e-6)


def test_umeyama_detects_recognition_noise() -> None:
    """纯识别噪声：``rms_before`` 与 ``rms`` 几乎一样（刚体解释不了）。"""
    src = _ring()
    noisy = [(x + 2.5 * math.sin(i * 2.3), y + 2.5 * math.cos(i * 1.7))
             for i, (x, y) in enumerate(src)]
    r = umeyama_2d(src, noisy)
    assert r.rms > 0.5 * r.rms_before


def test_umeyama_scale_is_off_by_default() -> None:
    """``with_scale`` 默认关：否则"尺度标定错了"会被吸收掉，
    于是标定错误看起来像识别很好。"""
    src = _ring()
    scaled = [(1.3 * x, 1.3 * y) for x, y in src]
    r_off = umeyama_2d(src, scaled)
    r_on = umeyama_2d(src, scaled, with_scale=True)
    assert r_off.scale == 1.0
    assert r_on.scale == pytest.approx(1.3, rel=1e-6)
    assert r_off.rms > 10.0 and r_on.rms < 1e-6


def test_umeyama_rejects_degenerate_input() -> None:
    with pytest.raises(AlignError, match="重合"):
        umeyama_2d([(1.0, 1.0)] * 3, [(2.0, 2.0), (3.0, 3.0), (4.0, 4.0)])


# ---------------------------------------------------------------------------
# ⑤ 时钟偏移：凸包估计
# ---------------------------------------------------------------------------
def test_time_offset_estimator_recovers_affine_clock() -> None:
    """从"单向时延非负"的点集里把 ``slope/offset`` 估出来（kalibr 的凸包法）。"""
    slope, offset = 1.00035, 2.75
    est = TimeOffsetEstimator()
    for i in range(400):
        remote = i * 0.033
        # 时延非负，且有一部分点接近 0 时延（凸包才有东西可贴）
        delay = 0.020 * ((i * 7919) % 101) / 100.0
        est.add(remote, slope * remote + offset + delay)
    s, o = est.slope_offset()
    assert s == pytest.approx(slope, abs=1e-4)
    assert o == pytest.approx(offset, abs=2e-3)
    assert "正常" in est.health()


def test_time_offset_estimator_rejects_backwards() -> None:
    est = TimeOffsetEstimator()
    est.add(1.0, 1.0)
    with pytest.raises(AlignError, match="倒退"):
        est.add(0.5, 1.0)


def test_time_offset_health_flags_drift() -> None:
    est = TimeOffsetEstimator()
    for i in range(50):
        remote = i * 0.1
        est.add(remote, 1.2 * remote)          # 速率偏 20%
    assert "异常" in est.health()


# ---------------------------------------------------------------------------
# ⑥ 匈牙利与身份指标
# ---------------------------------------------------------------------------
def test_hungarian_finds_the_optimum() -> None:
    cost = [[4, 1, 3], [2, 0, 5], [3, 2, 2]]
    pairs = hungarian(cost)
    assert sum(cost[i][j] for i, j in pairs) == 5


def test_hungarian_handles_rectangular_and_empty() -> None:
    assert hungarian([]) == []
    assert hungarian([[1.0, 2.0]]) == [(0, 0)]
    assert len(hungarian([[1.0], [2.0]])) == 1


def _run(truth_pts, est_pts, truth_ids, est_ids, gate=50.0):
    sims = [similarity_from_points(t, e, gate=gate) for t, e in zip(truth_pts, est_pts)]
    return evaluate_identity(truth_ids, est_ids, sims)


def test_identity_perfect_tracking() -> None:
    p = [[(1.0, 0.0)]] * 3
    r = _run(p, p, [[1]] * 3, [[7]] * 3)
    assert r.IDF1 == pytest.approx(1.0)
    assert r.IDSW == 0 and r.Frag == 0 and r.MT == 1


def test_idsw_uses_last_time_present_not_previous_frame() -> None:
    """漏检一帧后**同一个 id** 回来，不算身份切换。

    墙在画面边缘进出是常态，用"上一帧"比会把正常进出误判成切换
    （TrackEval clear.py:62-64）。
    """
    truth_pts = [[(1.0, 0.0)]] * 3          # 真值**一直在场**
    est_pts = [[(1.0, 0.0)], [], [(1.0, 0.0)]]   # 只有中间一帧漏检
    r = _run(truth_pts, est_pts, [[1]] * 3, [[7], [], [7]])
    assert r.CLR_FN == 1                    # 真值在场却没配上 -> FN
    assert r.IDSW == 0                      # 上次出现是 7，这次还是 7 -> 不算切换
    assert r.Frag == 1                      # 断裂要计数，但不算切换


def test_idsw_counts_a_real_switch() -> None:
    truth_pts = [[(1.0, 0.0)]] * 3
    est_pts = [[(1.0, 0.0)], [], [(1.0, 0.0)]]
    r = _run(truth_pts, est_pts, [[1]] * 3, [[7], [], [9]])
    assert r.IDSW == 1
    assert r.IDF1 == pytest.approx(0.4, abs=1e-9)


def test_identity_and_idsw_can_disagree() -> None:
    """``IDSW=0`` 与 ``IDF1=1.0`` 可以在**身份其实错了**时同时成立。

    门限很宽时，所有配对都过门限，逐帧匹配为了"保住 id"会配错位置，
    于是两个身份指标都完美。**只有位置误差能发现**（MOTP / 我们自己的误差）。
    """
    tp = [[(1.0, 0.0), (2.0, 0.0)]] * 2
    ep = [[(1.0, 0.0), (2.0, 0.0)]] * 2
    r_loose = _run(tp, ep, [[1, 2]] * 2, [[7, 8], [8, 7]], gate=50.0)
    r_tight = _run(tp, ep, [[1, 2]] * 2, [[7, 8], [8, 7]], gate=1.5)
    assert r_loose.IDF1 == pytest.approx(1.0)
    assert r_tight.IDF1 < 0.6
    assert r_tight.IDSW == 2


# ---------------------------------------------------------------------------
# ⑦ 流水线：病因分辨
# ---------------------------------------------------------------------------
def _series(n=240, *, player_speed=190.0, wall_speed=40.0):
    """真值：角色与墙都以恒定速率移动，方向不同。"""
    out = []
    for i in range(n):
        t = i / 120.0
        out.append(FrameTruth(
            stamp=t,
            player=(320.0, 70.0 - player_speed * t, 10.0),
            gaps=((180.0, 60.0, 480.0 - wall_speed * t),)))
    return out


def _estimates(truth, *, shift=(0.0, 0.0), t_offset=0.0, stride=4):
    out = []
    for i in range(0, len(truth), stride):
        f = truth[i]
        out.append(FrameEstimate(
            stamp=f.stamp + t_offset,
            player=(f.player[0] + shift[0], f.player[1] + shift[1], f.player[2]),
            gaps=((f.gaps[0][0] + shift[0], f.gaps[0][1], f.gaps[0][2] + shift[1]),)))
    return out


def test_reconcile_ideal_is_zero() -> None:
    t = _series()
    rep = reconcile(t, _estimates(t))
    assert rep.player.stats()["median"] == pytest.approx(0.0, abs=1e-9)
    assert rep.pairing.unmatched_est == 0
    assert rep.gap.identity is not None and rep.gap.identity.IDF1 == pytest.approx(1.0)


def test_reconcile_names_a_calibration_offset() -> None:
    """同一个刚体位移作用在所有对象上 -> **标定/机位错**，不是时钟偏移。"""
    t = _series()
    rep = reconcile(t, _estimates(t, shift=(9.0, 23.0)))
    r = rep.player.rigid
    assert r is not None
    assert r.rms_before == pytest.approx(math.hypot(9.0, 23.0), abs=1e-6)
    assert r.rms < 1e-6                       # 残余 = 0 -> 不是识别问题
    assert any("别改识别算法" in n for n in rep.notes)
    assert any("不是时钟偏移" in n for n in rep.notes)


def test_reconcile_names_a_clock_offset_even_when_timediff_looks_perfect() -> None:
    """时钟偏移恰好是采样间隔整数倍时，**配对时差是 0**，只能靠"平移量随速度缩放"发现。

    实测：0.5s 偏移 @120Hz 日志 + 30Hz 估计，中位时差 0.0 ms，
    平移方向与运动方向差 180°（看着像标定错）；但角色与墙反推的
    时间偏移都是 ~500 ms（速率 190 vs 40），这才是真判据。
    """
    t = _series()
    rep = reconcile(t, _estimates(t, t_offset=0.5))
    assert any("时钟偏移" in n for n in rep.notes)
    assert any("503 ms" in n or "502 ms" in n or "501 ms" in n for n in rep.notes), rep.notes


def test_reconcile_excludes_unmatched_frames_and_says_so() -> None:
    """配不上的帧要**排除并声明**，不能混进误差统计（那会变成配对错）。"""
    t = _series()
    est = [FrameEstimate(stamp=f.stamp + 5.0,
                         player=f.player, gaps=f.gaps) for f in t[::4]]
    with pytest.raises(AlignError, match="一对都配不上"):
        reconcile(t, est)


def test_reconcile_reports_missing_truth_gaps_as_missing() -> None:
    """真值没有缺口时，报告"一帧都没配上"，**不是**报一堆 0 误差。"""
    t = [FrameTruth(stamp=f.stamp, player=f.player, gaps=()) for f in _series()]
    est = [FrameEstimate(stamp=f.stamp, player=f.player, gaps=()) for f in t[::4]]
    rep = reconcile(t, est)
    assert rep.gap.n_matched == 0
    assert "一帧都没配上" in rep.gap.summary()


def test_reconcile_rejects_empty_input() -> None:
    t = _series(10)
    with pytest.raises(AlignError):
        reconcile([], _estimates(t))
    with pytest.raises(AlignError):
        reconcile(t, [])
