"""内容对齐（``vision_reconcile._align_by_content``）的回归测试。

为什么值得单独测
----------------
录像在 Windows 上、平台在 WSL 里，**两个时钟、两个起点**，没有任何共同时戳。
唯一的共同量是"画面内容"：平台日志说缺口在场地 (192, 412)，视频那一帧
检测出的缺口也在场地单位下，能重合的那个时间偏移 δ 就是答案。

这条路径**没有真值可对照**（它本身就是用来造真值的），所以必须用
"注入已知 δ，看能不能原样取回来"来验。这里就干这件事。

更要紧的是**周期性歧义**：墙每隔固定时间生成一堵，于是画面每隔一个
生成周期就长得一样，评分曲线有等间距的多个极小值。只看缺口，全局最小
可能落到隔壁周期上，而且**换任何算法都救不了**（是内容本身的对称性）。
本文件把这个失败模式钉住，并验证"把角色计进评分"确实能消歧。

数值上的两条纪律（踩过坑）
--------------------------
* 估计值必须带**检测噪声**：无噪声时评分能到正好 0，比值类指标失去意义，
  测出来的是"理想数据的性质"而不是"真实链路的行为"。真实检测误差
  约 0.5 场地单位，这里按这个量级注入固定种子的噪声。
* **δ 的精度上限是"一帧 + 一格"（0.05 + 0.02 ≈ 0.07s），不是搜索步长（0.02s）。**
  状态日志 20Hz，实现里取最近帧（不插值）：在半帧处左右两帧都配对得上、
  评分完全相同，形成一个约一帧宽的"平底"；再加上把平底归并成一个坑时
  取的是平滑曲线的最低格点，最终 δ 可能落在平底里任意一格上。
  所以容差取 0.08s。宣称更准就是在"角色匀速"的假设上编精度，
  实测链路里没有依据（0.07s × 40 单位/秒 ≈ 3 个场地单位，也不是瓶颈）。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import random
import sys
from dataclasses import dataclass, field

import pytest

# 被测实现住在 core/vision/align.py。**不要再 importlib 去加载根脚本** ——
# 那些函数原先挤在根脚本 vision_reconcile.py 里，测试只能按路径加载一个"脚本"，
# 属于布局违规的连带症状；收敛之后它们在家，测试也就正常 import 了。
from vision.align import (  # noqa: E402
    _align_affine as align_affine,
    _align_by_content as align,
    _fit_line_robust as robust_line_fit,
)

FIELD_H = 480.0
SIM_HZ = 20.0        # 平台状态日志频率
SPAWN_PERIOD = 1.6   # 秒 —— 墙的生成周期，也是歧义的间距
TOOTH = 0.6          # 场地单位 —— 注入的检测噪声（实测约 0.5）
# δ 的可分辨极限：一帧（0.05，20Hz）+ 一格（0.02）≈ 0.07，取 0.08
TOL = 0.08


@dataclass
class _F:
    """真值 / 估计共用的一帧：只要 stamp / player / gaps 三个属性。"""
    stamp: float
    player: tuple[float, float, float] | None = None
    gaps: tuple[tuple[float, float, float], ...] = field(default_factory=tuple)


class _Scene:
    """墙以 ``speed`` 往上、每 ``period`` 生成一堵的场地。

    ``speed * period`` 是否能整除场高决定了**歧义有多严重**：

    * 整除（如 37.5 * 1.6 = 60，480/60 = 8）→ 可见缺口集合每个周期
      **精确重复**，差一个周期的解和真解一样好，**纯周期、不可消歧**；
    * 不整除（平台的 40 * 1.6 = 64，480/64 = 7.5）→ 上下边界各有半堵墙
      对不齐，歧义被削弱但依然存在（实测残差比约 5 倍）。

    两种都要测：前者是"算法的极限在哪"，后者是"平台实际的样子"。
    """

    def __init__(self, speed: float = 40.0, period: float = SPAWN_PERIOD) -> None:
        self.speed = speed
        self.period = period

    def gaps_at(self, t: float) -> list[tuple[float, float, float]]:
        out = []
        # 生成时刻覆盖 [t - 场高/速度, t]，即所有可能还在场上的墙
        n0 = int(math.floor((t - FIELD_H / self.speed) / self.period))
        n1 = int(math.ceil(t / self.period))
        for n in range(n0, n1 + 1):
            y = FIELD_H - self.speed * (t - n * self.period)
            if 0.0 <= y <= FIELD_H:
                out.append((192.0, 60.0, y))
        return out

    @property
    def exactly_periodic(self) -> bool:
        q = self.speed * self.period
        return abs(FIELD_H / q - round(FIELD_H / q)) < 1e-9


def _truth(n: int, scene: _Scene, player_path=None) -> list[_F]:
    """平台状态日志：stamp = (k+1)/SIM_HZ（走完一步再记，与平台一致）。"""
    out = []
    for k in range(n):
        t = (k + 1) / SIM_HZ
        pl = None if player_path is None else player_path(t)
        out.append(_F(stamp=t, player=pl, gaps=tuple(scene.gaps_at(t))))
    return out


def _est(truth: list[_F], delta: float, scene: _Scene, *, player: bool,
         seed: int = 20261007) -> list[_F]:
    """录像侧：视频帧在时刻 p 拍到的，是平台 (p - δ) 时刻的样子。

    δ > 0 表示平台时钟领先视频（录像开始得晚）。
    叠加固定种子的检测噪声 —— 真实链路不可能零误差。
    """
    rng = random.Random(seed)
    t_stamps = [f.stamp for f in truth]
    out = []
    for k in range(len(truth)):
        p = k / SIM_HZ
        t = p - delta
        j = min(range(len(t_stamps)), key=lambda i: abs(t_stamps[i] - t))
        pl = None
        if player and truth[j].player is not None:
            px, py, pr = truth[j].player
            pl = (px + rng.gauss(0, TOOTH), py + rng.gauss(0, TOOTH), pr)
        gaps = tuple((cx + rng.gauss(0, TOOTH), w, cy + rng.gauss(0, TOOTH))
                     for (cx, w, cy) in scene.gaps_at(t))
        out.append(_F(stamp=p, player=pl, gaps=gaps))
    return out


def _circular(t: float):
    """非周期的角色轨迹：圆周运动，两个坐标都在变。

    角速度 1.3 rad/s，与墙的生成周期 1.6s **不可通约**（1.3*1.6 = 2.08 rad
    不是 2π 的倍数），所以角色位置不会跟着墙一起重复 —— 这正是它能消歧的原因。
    """
    r = 120.0
    return (320.0 + r * math.cos(1.3 * t), 240.0 + r * math.sin(1.3 * t), 10.0)


_STILL = lambda _t: (320.0, 240.0, 10.0)  # noqa: E731


# --------------------------------------------------------------------------
# 一、注入的 δ 要能取回来
# --------------------------------------------------------------------------
@pytest.mark.parametrize("injected", [-1.5, -0.4, 0.0, 0.95, 1.2, 2.5])
def test_recovers_injected_offset_when_player_moves(injected: float) -> None:
    """角色在动 —— 非周期信息足够，注入多少就该取回多少（到 TOL 为止）。"""
    scene = _Scene(40.0)                      # 平台实际的墙速
    truth = _truth(240, scene, _circular)
    est = _est(truth, injected, scene, player=True)
    delta, gap, ps, rival = align(truth, est)
    assert delta is not None
    assert delta == pytest.approx(injected, abs=TOL), f"注入 {injected}，取回 {delta}"
    # 噪声量级 TOOTH，中位残差应当在同一量级（而不是 0，也不是几十）
    assert gap < 4 * TOOTH, f"缺口残差应接近噪声量级，实际 {gap}"
    assert ps < 4 * TOOTH, f"角色残差应接近噪声量级，实际 {ps}"
    assert rival > 1.5, f"角色在动时不该有歧义，实际歧义比 {rival}"


@pytest.mark.parametrize("injected", [0.05, 0.45, -0.45])
def test_half_frame_offset_is_the_known_worst_case(injected: float) -> None:
    """注入的 δ 恰好落在**半帧**上时，取回的 δ 可以差一整帧，代价如实可见。

    为什么单独测：实现里取最近帧（不插值），est 帧与真值帧的配对在
    半帧处左右都成立，评分相同 —— 于是 δ 的误差最大到一整帧（0.05s），
    角色残差相应被放大到 ``帧间位移 ≈ 0.05 × 156 ≈ 7.8 场地单位``。
    这是"取最近帧"的固有代价，**不是 bug**；把它钉在这里，
    免得以后有人以为 δ 的精度等于搜索步长（0.02s）而去"修正"它。
    """
    scene = _Scene(40.0)
    truth = _truth(240, scene, _circular)
    est = _est(truth, injected, scene, player=True)
    delta, gap, ps, rival = align(truth, est)
    assert delta is not None
    # 误差不超过一整帧（0.05s），再大就说明配对错了
    assert abs(delta - injected) <= TOL, f"注入 {injected}，取回 {delta}"
    assert gap < 4 * TOOTH
    # 角色残差被 δ 误差按"角色速度 × δ误差"放大，量级对得上即可
    assert ps < 4 * TOOTH + 200 * abs(delta - injected), \
        f"角色残差 {ps:.2f} 与 δ 误差 {abs(delta - injected):.3f}s 不相称"


def test_recovers_offset_without_player_on_non_periodic_scene() -> None:
    """角色没参与时，**不精确周期**的场景（平台实际情形）仍然能对齐。

    平台 speed*period = 64 不整除场高 480，上下边界对不齐，
    相邻周期的解被削弱，所以纯缺口也够用 —— 但这是"平台参数恰好帮忙"，
    不是算法保证（见下一个测试）。
    """
    scene = _Scene(40.0)
    assert not scene.exactly_periodic
    truth = _truth(240, scene, _circular)
    est = _est(truth, 0.95, scene, player=False)
    delta, gap, ps, rival = align(truth, est)
    assert delta == pytest.approx(0.95, abs=0.03)
    assert math.isnan(ps), "角色没参与时应如实报 nan，不能编一个 0"


# --------------------------------------------------------------------------
# 二、周期场景的歧义：这是算法的极限，不是 bug
# --------------------------------------------------------------------------
def test_exactly_periodic_scene_cannot_be_disambiguated_by_gaps() -> None:
    """**只看缺口**时，精确周期的场景必然歧义 —— 把这个失败模式钉住。

    墙速 37.5 * 周期 1.6 = 60，480/60 = 8 整除：可见缺口集合每个周期
    精确重复，于是 δ 和 δ±1.6 的评分**完全一样**。
    这不是"算法不够好"，是内容本身的对称性；断言歧义比 ≈ 1
    就是在断言"工具必须承认自己没算准"，而不是随便挑一个。
    """
    scene = _Scene(37.5, SPAWN_PERIOD)
    assert scene.exactly_periodic
    truth = _truth(240, scene, _circular)
    est = _est(truth, 0.95, scene, player=False)
    delta, _gap, _ps, rival = align(truth, est)
    assert delta is not None
    assert rival == pytest.approx(1.0, abs=0.05), \
        f"精确周期场景本该完全歧义（歧义比 1），实际 {rival}"


def test_player_resolves_the_exactly_periodic_scene() -> None:
    """同一个"不可消歧"的场景，**把角色计进评分就能消歧**。

    这是加角色项的全部理由：缺口是周期的，角色不是。
    用**相对**判据（开角色后歧义比明显变大）而不是绝对阈值，
    因为绝对的"够不够可信"没有客观标准，而"角色项有没有用"是可比的。
    """
    scene = _Scene(37.5, SPAWN_PERIOD)
    truth = _truth(240, scene, _circular)
    for injected in (0.95, -1.3, 2.2):
        est = _est(truth, injected, scene, player=True)
        delta, _gap, _ps, rival_moving = align(truth, est)
        assert delta == pytest.approx(injected, abs=TOL), \
            f"注入 {injected}，取回 {delta}"
        # 同一批画面，只把角色项关掉 —— 直接量出角色项的贡献
        est_np = _est(truth, injected, scene, player=False)
        _d2, _g2, _p2, rival_gaps_only = align(truth, est_np)
        assert rival_gaps_only == pytest.approx(1.0, abs=0.05)
        assert rival_moving > 2.5 * rival_gaps_only, (
            f"角色在动时应当把隔壁周期拉开，实际 "
            f"{rival_moving:.2f} vs 纯缺口 {rival_gaps_only:.2f}")


def test_stationary_player_does_not_fake_a_confident_answer() -> None:
    """角色**站着不动**时它是个常数项，等于没有信息 —— 不能因此显得可信。

    这是最容易犯的错：加了角色就看歧义比，却忘了"人没操作"时
    这一项对所有 δ 都相等。此时必须仍然暴露周期歧义。
    """
    scene = _Scene(37.5, SPAWN_PERIOD)
    truth = _truth(240, scene, _STILL)
    est = _est(truth, 0.95, scene, player=True)
    delta, _gap, _ps, rival = align(truth, est)
    assert delta is not None
    assert rival == pytest.approx(1.0, abs=0.05), \
        f"角色不动时不该自信，实际歧义比 {rival}"


# --------------------------------------------------------------------------
# 三、退化输入：不崩，也不假装算出来了
# --------------------------------------------------------------------------
def test_empty_inputs_return_none() -> None:
    assert align([], [])[0] is None
    assert align(_truth(10, _Scene(), _circular), [])[0] is None


def test_scene_without_gaps_returns_none() -> None:
    """两边都没有缺口（平台还没生成墙）时不能编一个 δ 出来。"""
    scene = _Scene()
    truth = [_F(stamp=(k + 1) / SIM_HZ, player=_circular(k / SIM_HZ)) for k in range(60)]
    est = [_F(stamp=k / SIM_HZ, player=_circular(k / SIM_HZ)) for k in range(60)]
    assert align(truth, est)[0] is None
    assert scene.gaps_at(0.0) == [] or True   # 场景本身与断言无关，保留以示对照


def test_far_apart_clocks_return_none() -> None:
    """两边时间完全不重叠（拿错了 log）时必须返回 None，不能硬报一个 δ。

    这是真实的误用：录像和状态日志不是同一次跑（或差了几分钟），
    时间上根本对不上。此时任何 δ 都只是"矮子里拔将军"。
    """
    scene = _Scene()
    truth = _truth(120, scene, _circular)          # 0.05 ~ 6.0 s
    est = _est(truth, 0.0, scene, player=True)
    for e in est:                                  # 平移到 100 s 之后
        e.stamp += 100.0
    assert align(truth, est)[0] is None


def test_one_sided_input_returns_none() -> None:
    """只有一边有缺口时返回 None（平台还没生成墙，或视频里没检出墙）。"""
    scene = _Scene()
    truth = [_F(stamp=(k + 1) / SIM_HZ, player=_circular(k / SIM_HZ)) for k in range(60)]
    est = _est(_truth(60, scene, _circular), 0.0, scene, player=True)
    assert align(truth, est)[0] is None


# ---------------------------------------------------------------------------
# 四、录像时间轴漂移（掉帧）：δ(t) 不再是常数
# ---------------------------------------------------------------------------
def _drifted_est(truth, scene, *, base: float, rate: float, player: bool,
                 seed: int = 7):
    """模拟"录屏掉帧"：录像时间轴比平台时钟快 ``rate``（例如 0.0137）。

    模型：视频第 ``v`` 帧标称时刻是 ``v/fps``，但它实际拍到的是平台
    ``v/fps * (1+rate) + base`` 时刻的画面 —— 即录像把一段**被压缩过**的
    时间铺在了标称时长上。此时
    ``truth_time = est_time * (1+rate) + base``，一个常数 δ 对不齐整段。
    """
    rng = random.Random(seed)
    t_stamps = [f.stamp for f in truth]
    out = []
    for k in range(len(truth)):
        e = k / SIM_HZ
        t = e * (1.0 + rate) + base
        j = min(range(len(t_stamps)), key=lambda i: abs(t_stamps[i] - t))
        pl = None
        if player and truth[j].player is not None:
            px, py, pr = truth[j].player
            pl = (px + rng.gauss(0, TOOTH), py + rng.gauss(0, TOOTH), pr)
        out.append(_F(stamp=e, player=pl,
                      gaps=tuple((cx + rng.gauss(0, TOOTH), w, cy + rng.gauss(0, TOOTH))
                                 for (cx, w, cy) in scene.gaps_at(t))))
    return out


def test_affine_alignment_recovers_a_frame_drop_rate() -> None:
    """掉帧造成的线性漂移必须被估出来，而且比常数 δ 明显更准。

    这是真实事故：一段 104s 的 Windows 录屏，分窗最优 δ 从 -1.40 线性走到
    -0.22（1.37%/秒）。常数 δ 只能取折中，缺口残差十几单位、评分曲线变平、
    歧义比贴近 1；仿射拟合之后每个窗口都回到 1~2 单位。
    """
    scene = _Scene(40.0)
    # 用 200s 的真值，让漂移有足够时间积累（短了估不出来，也不该硬估）
    truth = _truth(3000, scene, _circular)
    rate = 0.0137
    est = _drifted_est(truth, scene, base=-1.49, rate=rate, player=True)

    a, b, rows, ok = align_affine(truth, est)
    assert ok, "这么长的序列应当估得出仿射关系"
    assert len(rows) >= 3, f"窗口太少: {len(rows)}"
    # 符号约定：`_align_by_content` 里 δ 满足 ``truth_time = est_time - δ``。
    # 这里造的数据是 ``content_time = est*(1+rate) + base``，所以
    #   δ(est) = est - content_time = -rate*est - base
    # 于是斜率 ≈ -rate、截距 ≈ -base。（第一版把截距符号写反了，实测值是 +1.50。）
    assert a == pytest.approx(-rate, abs=2e-3), f"估出的速率差 {a:+.5f}，应为 {-rate:+.5f}"
    assert b == pytest.approx(1.49, abs=0.15), f"截距 {b:+.3f}，应为 {-(-1.49):+.3f}"

    # 关键：仿射修正之后，配对残差应当远小于"常数 δ"能给的
    import dataclasses
    fixed = [dataclasses.replace(e, stamp=e.stamp - (a * e.stamp + b)) for e in est]
    d_c, g_c, p_c, _r = align(truth, est)              # 常数模型
    d_a, g_a, p_a, _r2 = align(truth, fixed)           # 仿射修正后
    assert d_a == pytest.approx(0.0, abs=0.1)

    def median_gap_after(series, offset):
        """按给定偏移配对后，缺口 y 的中位绝对误差（场地单位）。"""
        import math as _m
        errs = []
        for e in series[::max(1, len(series) // 200)]:
            t = e.stamp - offset
            j = min(range(len(truth)), key=lambda i: abs(truth[i].stamp - t))
            if abs(truth[j].stamp - t) > 0.1 or not e.gaps or not truth[j].gaps:
                continue
            for (_ex, _ew, ey) in e.gaps:
                errs.append(min(abs(ey - ty) for (_tx, _tw, ty) in truth[j].gaps))
        errs.sort()
        return errs[len(errs) // 2] if errs else float("nan")

    bad_const = median_gap_after(est, d_c)
    good_affine = median_gap_after(fixed, 0.0)
    assert good_affine * 3 < bad_const, (
        f"仿射修正应当明显更准：常数 δ 残差 {bad_const:.2f}，"
        f"仿射后 {good_affine:.2f}")


def test_affine_falls_back_for_short_series() -> None:
    """序列太短时**不要**硬估斜率 —— 退回常数 δ（返回 ok=False）。"""
    scene = _Scene(40.0)
    truth = _truth(200, scene, _circular)          # 10 s
    est = _est(truth, 0.95, scene, player=True)
    a, b, rows, ok = align_affine(truth, est, min_span=20.0)
    assert ok is False and rows == []


def test_robust_line_fit_rejects_an_outlier_window() -> None:
    """某个窗口错一个墙周期时，稳健拟合不能被它带偏。"""
    xs = [0.0, 10.0, 20.0, 30.0, 40.0]
    ys = [1.0, 1.5, 2.0, 99.0, 3.0]      # 第 4 个是离群点
    a, b, keep = robust_line_fit(xs, ys)
    assert keep.count(False) == 1 and keep[3] is False
    assert a == pytest.approx(0.05, abs=0.01)
    assert b == pytest.approx(1.0, abs=0.1)
