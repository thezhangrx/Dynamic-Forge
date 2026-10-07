"""``core/vision/reconcile.py`` 的回归测试 —— 重点是**配对**这一步。

为什么单独测"配对"
------------------
对账 = 把估计序列配上真值序列，再逐项相减。相减之前必须先保证
**配到的是同一时刻**，否则算出来的是"配对错"，它会伪装成"识别错"，
而且伪装得很像（数字连续变化、看起来只是精度不够）。

这里钉的就是这个：``pair_series`` / ``reconcile`` 在给了非零
``offset_truth`` 时，**关联和插值必须用同一个时间约定**。
"""

from __future__ import annotations

import math
import sys

import pytest

sys.path.insert(0, "core")

from vision.reconcile import (  # noqa: E402
    FrameEstimate,
    FrameTruth,
    pair_series,
    reconcile,
)

# 平台的**角色速度是 200 场地单位/秒**（不是 10 —— 10 是半径）。
# 这个量级是这条 bug 之所以贵的原因，所以测试里按真实值造。
PLAYER_SPEED = 200.0
TRUTH_HZ = 20.0
EST_HZ = 20.0


def _moving_series(n: int, *, offset: float):
    """造一段"真值 + 估计"，估计比真值晚 ``offset`` 秒（同一物理轨迹）。

    物理量：角色沿 x 匀速 200 单位/秒，缺口也一起匀速走（40 单位/秒）。
    ``offset`` 的定义与 ``vision.align.associate`` 一致：
    ``truth_stamp + offset == est_stamp``，即
    ``est(t) = truth(t - offset)``。
    """
    truth, est = [], []
    for k in range(n):
        t_stamp = (k + 1) / TRUTH_HZ
        truth.append(FrameTruth(
            stamp=t_stamp,
            player=(PLAYER_SPEED * t_stamp, 100.0, 10.0),
            gaps=((320.0, 60.0, 40.0 * t_stamp),)))
    for k in range(n):
        e_stamp = k / EST_HZ
        # 视频/估计看到的是 (e_stamp - offset) 时刻的真值
        t = e_stamp - offset
        est.append(FrameEstimate(
            stamp=e_stamp,
            player=(PLAYER_SPEED * t, 100.0, 10.0),
            gaps=((320.0, 60.0, 40.0 * t),)))
    return truth, est


@pytest.mark.parametrize("offset", [0.0, 0.06, -0.06, 0.5, -0.5])
def test_paired_truth_lands_on_the_same_instant(offset: float) -> None:
    """**核心断言**：配对后，插值出来的真值角色位置要和估计几乎重合。

    修之前这里会差 ``|offset| x 200`` 单位（offset=-0.06 时约 12 单位）
    —— 因为关联用了 offset、插值没用，两者自相矛盾。
    """
    truth, est = _moving_series(200, offset=offset)
    rep = reconcile(truth, est, offset_truth=offset, max_diff=0.02, gap_gate=100.0)
    assert rep.player.n_frames > 0
    med = rep.player.stats()["median"]
    # 只允许亚单位级的差（轨迹是精确直线，配对对了就是 0）
    assert med < 0.5, f"offset={offset}: 配对后角色位置误差中位 {med:.2f} 单位，说明插值用错了时刻"


def test_offset_is_honoured_in_both_directions() -> None:
    """正负 offset 都要对 —— 只用一个符号测不出符号错误。"""
    for offset in (0.06, -0.06):
        truth, est = _moving_series(120, offset=offset)
        pairing = pair_series(truth, est, offset_truth=offset, max_diff=0.02)
        # 关联选中的真值帧，其时戳加上 offset 应当就是估计时戳附近
        for ei, ti, d in zip(pairing.est_indices, pairing.truth_indices, pairing.diffs):
            assert abs(truth[ti].stamp + offset - est[ei].stamp) <= 0.02 + 1e-9
            assert d <= 0.02 + 1e-9


def test_interpolated_truth_time_is_offset_corrected() -> None:
    """直接盯住 ``truth_at`` 的时戳：它必须落在"该看的那一刻"。

    ``pair_series`` 把 ``truth_at[k].stamp`` 记成插值目标时刻。这个字段
    平时没人看，但它是**插值到底取在哪一刻**的唯一凭据 —— 上面那条 bug
    在这个字段上表现为整体差一个 ``offset``。
    """
    offset = 0.2
    truth, est = _moving_series(100, offset=offset)
    pairing = pair_series(truth, est, offset_truth=offset, max_diff=0.02)
    for k, ei in enumerate(pairing.est_indices):
        want = est[ei].stamp - offset
        got = pairing.truth_at[k].stamp
        assert got == pytest.approx(want, abs=1e-9), \
            f"插值目标 {got} != est({est[ei].stamp}) - offset({offset}) = {want}"


def test_player_error_is_insensitive_to_truth_sample_rate() -> None:
    """真值采样率变密，配对后误差不该变 —— 变密只该让插值更准。

    这条挡的是"靠吸附最近帧而不是插值"的实现：那样误差会随真值帧率
    呈锯齿（半帧 x 200 单位/秒），而正确实现应当稳定在小量级。
    """
    meds = []
    for hz in (20.0, 60.0):
        truth, est = [], []
        for k in range(int(6 * hz)):
            t = (k + 1) / hz
            truth.append(FrameTruth(stamp=t, player=(PLAYER_SPEED * t, 0.0, 10.0)))
        for k in range(120):
            e = k / 20.0
            t = e + 0.05
            est.append(FrameEstimate(stamp=e, player=(PLAYER_SPEED * t, 0.0, 10.0)))
        rep = reconcile(truth, est, offset_truth=-0.05, max_diff=0.02)
        meds.append(rep.player.stats()["median"])
    assert max(meds) < 0.5, f"不同真值帧率下误差不一致：{meds}"


def test_stationary_player_does_not_crash() -> None:
    """角色不动（平台默认）时刚体变换无定义，必须优雅退化而不是抛异常。"""
    truth = [FrameTruth(stamp=(k + 1) / TRUTH_HZ, player=(320.0, 100.0, 10.0))
             for k in range(50)]
    est = [FrameEstimate(stamp=k / EST_HZ, player=(320.0, 100.0, 10.0))
           for k in range(50)]
    rep = reconcile(truth, est, offset_truth=-0.05, max_diff=0.02)
    assert rep.player.stationary is True
    assert rep.player.rigid is None
    assert rep.player.stats()["median"] == pytest.approx(0.0, abs=1e-9)


def test_far_apart_clocks_raise_instead_of_pairing_garbage() -> None:
    """两边时间完全不重叠时必须报错，不能"最邻近"地硬配上。"""
    from vision.align import AlignError

    truth, est = _moving_series(50, offset=0.0)
    # FrameEstimate 是 frozen 的，所以挪时间要重建，不能就地改
    far = [FrameEstimate(stamp=e.stamp + 100.0, player=e.player, gaps=e.gaps)
           for e in est]
    with pytest.raises(AlignError):
        pair_series(truth, far, offset_truth=0.0, max_diff=0.05)
