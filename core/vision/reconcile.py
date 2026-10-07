"""对账流水线：把「真值序列」与「估计序列」配起来，报出**分得清病因**的误差。

一句话目标
----------
输入一段录像的识别结果 + 同一时段的平台状态日志，输出：

* 每个量（角色位置/半径、缺口中心/宽度、墙位置）的误差分布；
* **刚体错位（=标定错）与对齐后残差（=识别错）分开报**；
* 配对质量（时差中位/max、配不上的比例）—— 否则"误差大"是识别差还是配对差分不清；
* 墙/缺口的跨帧身份指标（IDSW / MT,PT,ML / IDF1）。

为什么必须把"标定错"和"识别错"分开
----------------------------------
实测：同一份识别代码，用整屏那张参考物照片的标定去对账紧贴场地的序列，
角色位置误差 **261 场地单位**；换成同机位的标定，**1.78 单位**。算法一行没改。
当时只有一个数字，无法判断病因，只能事后二分。``align.umeyama_2d`` 把这件事
变成两个数（见 ``docs/reference/evo.md``）。

数据形状：逐帧位置化
--------------------
两边都是**等长的逐帧序列**（``stamps[i]`` 与第 i 帧的内容一一对应），
**某一帧没有目标就是空元组，而不是把这一帧删掉**。
删掉会让后续所有帧错位一帧，而错位会伪装成"识别变差"。
这条约定取自 nuscenes 与 TrackEval，见 ``docs/reference/nuscenes-devkit.md``。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from .align import (
    DEFAULT_MAX_DIFF,
    AlignError,
    Rigid2D,
    associate,
    check_monotonic,
    interpolate_linear,
    umeyama_2d,
)
from .identity import (
    DEFAULT_THRESHOLD,
    IdentityReport,
    evaluate_identity,
    similarity_from_points,
)

#: 缺口的跨帧关联门限（场地单位）。超出就当成"另一个缺口"。
DEFAULT_GAP_GATE = 40.0

#: 判定"这一对算配上"的最小相似度（与 identity.DEFAULT_THRESHOLD 一致）。
DEFAULT_MATCH_THRESHOLD = DEFAULT_THRESHOLD


# ---------------------------------------------------------------------------
# 数据形状
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FrameTruth:
    """某一时刻的**平台真值**。

    ``gaps`` 每条是 ``(缺口中心x, 缺口宽, 墙带中心y)``；
    ``player`` 是 ``(x, y, 半径)``；没有就是 ``None`` / 空元组。
    """

    stamp: float
    player: tuple[float, float, float] | None = None
    gaps: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class FrameEstimate:
    """同一时刻的**视觉估计**，字段含义与 :class:`FrameTruth` 完全一致。

    只有字段含义一致，逐个相减才有意义 —— 这是本项目最贵的一课：
    ``track_real.jsonl`` 曾经把场地单位的数值标成 ``units="m"``，
    数值一样、含义不同，属于"跑得通但全错"。
    """

    stamp: float
    player: tuple[float, float, float] | None = None
    gaps: tuple[tuple[float, float, float], ...] = ()


@dataclass
class Pairing:
    """一次配对的结果：``est[i]`` 对应 ``truth_at[i]``。"""

    est_indices: list[int] = field(default_factory=list)
    truth_indices: list[int] = field(default_factory=list)
    #: 每对的时间差（秒）
    diffs: list[float] = field(default_factory=list)
    #: 真值被**插值**到估计时戳之后的结果
    truth_at: list[FrameTruth] = field(default_factory=list)
    #: 没能配上任何真值（超出 max_diff）的估计帧数
    unmatched_est: int = 0

    def __len__(self) -> int:
        return len(self.est_indices)

    def median_diff(self) -> float:
        return _median(self.diffs) if self.diffs else float("nan")

    def max_diff(self) -> float:
        return max(self.diffs) if self.diffs else float("nan")


# ---------------------------------------------------------------------------
# ② 配对：evo 式硬门限 + 真值插值到估计时戳
# ---------------------------------------------------------------------------
def pair_series(truth: Sequence[FrameTruth], est: Sequence[FrameEstimate], *,
                max_diff: float = DEFAULT_MAX_DIFF,
                offset_truth: float = 0.0,
                name: str = "reconcile") -> Pairing:
    """把估计序列的每一帧配上它时戳附近的真值。

    两步（顺序不能反）：

    1. **硬门限关联**（evo ``matching_time_indices``）：每个估计时戳取最邻近的真值时戳，
       只有时差 ``<= max_diff`` 才算配上，**超出的估计帧直接标为未配对**。
       没这道门限，"最邻近"会静默把差好几秒的两帧配到一起，
       于是误差里混进的是"配对错"而不是"识别错"。
    2. **把真值插值到估计时戳上**（而不是吸附到最近的真值采样点）。
       平台日志 120 Hz、相机 9~30 Hz，吸附最多引入半个采样周期；
       但**插值必须先把目标时戳夹在真值区间内**，绝不外推 ——
       外推出来的真值看起来是有效的，比没有真值更危险。
    """
    if not truth:
        raise AlignError(f"{name}: 真值序列为空")
    if not est:
        raise AlignError(f"{name}: 估计序列为空")
    t_stamps = [f.stamp for f in truth]
    e_stamps = [f.stamp for f in est]
    check_monotonic(t_stamps, name=f"{name}.truth.stamp")
    check_monotonic(e_stamps, name=f"{name}.est.stamp")

    assoc = associate(e_stamps, t_stamps, max_diff=max_diff,
                      offset_other=offset_truth,
                      name_ref=f"{name}.est", name_other=f"{name}.truth")

    out = Pairing(unmatched_est=len(e_stamps) - len(assoc))
    for ei, ti, d in zip(assoc.idx_ref, assoc.idx_other, assoc.diffs):
        out.est_indices.append(ei)
        out.truth_indices.append(ti)
        out.diffs.append(d)

    # 真值插值到估计时戳。
    # 只在"真值每一帧都有角色、且至少 2 帧"时才插值；否则**退化成分量缺失**
    # （一个都不填），让下游把那些帧算成"真值没有角色"。
    # **宁可少报，也不要编一个插值出来。**
    #
    # **插值目标必须减掉 offset_truth**，否则和上面的关联**自相矛盾**：
    # `associate` 是拿 `truth_stamp + offset_truth` 去比 `e_stamp` 的
    # （见 ``vision.align.associate``：``shifted = [t + offset_other ...]``），
    # 也就是"est 时刻 e 对应的真值时刻是 e - offset_truth"。
    # 关联用偏移、插值不用，插出来的真值就整体错了一个 offset。
    #
    # 实测这个错有多贵：修之前，一段 240 帧的合成录像（角色 200 单位/秒、
    # offset=-0.06s）报出**角色位置误差中位 5.32 单位**，而逐帧直接比对
    # 只有 **1.10**。0.06s x 200 单位/秒 = 12 单位 —— 误差全是这个时间错。
    #
    # 为什么"缺口看着没事、只有角色离谱"很好认：缺口**不插值**，
    # 是从关联选中的那一帧原样带过来的，用了 offset，所以是准的。
    # 两边一个插值一个不插，误差只出现在插值的那一边 —— 这个不对称
    # 就是这条 bug 的指纹。
    targets = [e_stamps[i] - offset_truth for i in out.est_indices]
    has_player = [f.player is not None for f in truth]
    interp_player: list[tuple[float, float, float] | None] = [None] * len(targets)
    if targets and len(truth) >= 2 and all(has_player):
        xs = interpolate_linear(t_stamps, [f.player[0] for f in truth], targets)  # type: ignore[index]
        ys = interpolate_linear(t_stamps, [f.player[1] for f in truth], targets)  # type: ignore[index]
        rs = interpolate_linear(t_stamps, [f.player[2] for f in truth], targets)  # type: ignore[index]
        interp_player = list(zip(xs, ys, rs))

    # 缺口：真值缺口条数逐帧可能不同，所以**不做插值**（不同条数没法插），
    # 直接把最近的真值帧原样带过来，并把时差记进 diffs 供核查。
    truth_at: list[FrameTruth] = []
    for k, ti in enumerate(out.truth_indices):
        src = truth[ti]
        truth_at.append(FrameTruth(
            stamp=targets[k],
            player=interp_player[k] if interp_player[k] is not None else src.player,
            gaps=src.gaps))
    out.truth_at = truth_at
    return out


# ---------------------------------------------------------------------------
# ③ 各量的误差
# ---------------------------------------------------------------------------
@dataclass
class PlayerReport:
    """角色的误差 + 刚体分离。"""

    n_frames: int = 0
    n_detected: int = 0
    #: 位置误差（场地单位），配对成功且两侧都有角色的帧
    pos_err: list[float] = field(default_factory=list)
    radius_err: list[float] = field(default_factory=list)
    #: 位置误差的矢量（未取模），用于 Umeyama
    vec: list[tuple[tuple[float, float], tuple[float, float]]] = field(default_factory=list)
    rigid: Rigid2D | None = None
    #: 角色全程几乎没动 -> 刚体错位无法定义（点重合）。这不是错误。
    stationary: bool = False

    @property
    def detection_rate(self) -> float:
        return self.n_detected / self.n_frames if self.n_frames else 0.0

    def stats(self) -> dict[str, float]:
        return _err_stats(self.pos_err)

    def summary(self) -> str:
        if not self.pos_err:
            return f"角色: 一次都没检出（配对 {self.n_frames} 帧）"
        s = self.stats()
        rad = ("真值缺失" if not self.radius_err else
               f"中位 {_median([abs(v) for v in self.radius_err]):.2f}")
        line = (f"角色: 检出 {100 * self.detection_rate:.0f}%  位置误差 "
                f"中位 {s['median']:.2f} / p90 {s['p90']:.2f} / max {s['max']:.2f} 场地单位"
                f"  半径误差 {rad}")
        if self.rigid is not None:
            line += "\n      " + self.rigid.summary()
        elif self.stationary:
            line += ("\n      （角色全程没动，刚体错位无法定义 —— "
                     "要看『标定错还是识别错』需要让角色动起来）")
        return line


@dataclass
class GapReport:
    """缺口/墙的误差 + 跨帧身份。"""

    n_frames: int = 0
    n_matched: int = 0
    n_unmatched: int = 0
    #: 本次用的关联门限，只用于把"没配上"解释清楚
    gate: float = 0.0
    cx_err: list[float] = field(default_factory=list)
    width_err: list[float] = field(default_factory=list)
    y_err: list[float] = field(default_factory=list)
    #: 按**跨帧身份 id** 分组的配对点：``{id: [((真值x,真值y), (估计x,估计y)), ...]}``。
    #: 为什么不放在一个平铺列表里：一场可以同时有**多堵墙**（实测 8 秒后场上 3 堵），
    #: 混在一起做刚体拟合会得到毫无意义的变换（实测报出 RMS 332 单位、旋转 90°）。
    #: 刚性错位只对**同一个对象**才有定义。
    vec_by_id: dict[int, list[tuple[tuple[float, float], tuple[float, float]]]] = \
        field(default_factory=dict)
    rigid: Rigid2D | None = None
    #: 拟合刚体所用的那个缺口 id（取帧数最多的一个）
    rigid_id: int | None = None
    #: ``{真值id: [该帧配到的估计id, ...]}``。若同一个真值 id 上出现多个估计 id，
    #: 说明**视觉侧在这个缺口上换过身份**（清单 D6 的表现）。
    est_id_of_truth: dict[int, list[int]] = field(default_factory=dict)
    identity: IdentityReport | None = None

    def summary(self) -> str:
        if not self.cx_err:
            return (f"缺口: 一帧都没配上（配对 {self.n_frames} 帧，"
                    f"未配对 {self.n_unmatched}）。\n"
                    f"      若两侧其实都有缺口，说明偏移超过了 --gap-gate "
                    f"({self.gate:g} 场地单位) —— 调大门限再跑一次")
        s = _err_stats(self.cx_err)
        w = _err_stats(self.width_err)
        y = _err_stats(self.y_err)
        line = (f"缺口: 配到 {self.n_matched}/{self.n_frames} 帧 "
                f"中心x误差 中位 {s['median']:.2f} / p90 {s['p90']:.2f} "
                f"| y误差 中位 {y['median']:.2f} "
                f"| 宽度误差 中位 {w['median']:.2f} / max {w['max']:.2f} 场地单位")
        if self.rigid is not None:
            n_id = len(set(self.est_id_of_truth.get(self.rigid_id, ())))
            line += (f"\n      [刚体拟合于缺口 id={self.rigid_id}（真值），"
                     f"{len(self.vec_by_id.get(self.rigid_id, ()))} 帧，"
                     f"其间估计侧用了 {n_id} 个 id]")
            line += "\n      " + self.rigid.summary()
        if len(self.vec_by_id) > 1:
            line += f"\n      （本段出现过 {len(self.vec_by_id)} 个不同缺口）"
        if self.identity is not None:
            line += "\n      " + self.identity.summary()
        return line


@dataclass
class ReconcileReport:
    """整份对账报告。"""

    pairing: Pairing
    player: PlayerReport
    gap: GapReport
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "pairs": len(self.pairing),
            "unmatched_est": self.pairing.unmatched_est,
            "pair_time_diff_median": self.pairing.median_diff(),
            "pair_time_diff_max": self.pairing.max_diff(),
            "player": {
                "detection_rate": self.player.detection_rate,
                **self.player.stats(),
                "radius_err_median": _median([abs(v) for v in self.player.radius_err])
                if self.player.radius_err else None,
                "rigid": self.player.rigid.to_dict() if self.player.rigid else None,
            },
            "gap": {
                "matched": self.gap.n_matched,
                "unmatched": self.gap.n_unmatched,
                "cx": _err_stats(self.gap.cx_err),
                "width": _err_stats(self.gap.width_err),
                "rigid": self.gap.rigid.to_dict() if self.gap.rigid else None,
                "identity": self.gap.identity.to_dict() if self.gap.identity else None,
            },
            "notes": list(self.notes),
        }

    def summary(self) -> str:
        lines = [
            f"配对: {len(self.pairing)} 对  未配对估计帧 {self.pairing.unmatched_est}  "
            f"时差 中位 {1e3 * self.pairing.median_diff():.1f} ms / "
            f"max {1e3 * self.pairing.max_diff():.1f} ms",
            self.player.summary(),
            self.gap.summary(),
        ]
        for n in self.notes:
            lines.append(f"注: {n}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def reconcile(truth: Sequence[FrameTruth], est: Sequence[FrameEstimate], *,
              max_diff: float = DEFAULT_MAX_DIFF,
              offset_truth: float = 0.0,
              gap_gate: float = DEFAULT_GAP_GATE,
              threshold: float = DEFAULT_MATCH_THRESHOLD,
              with_scale: bool = False,
              ) -> ReconcileReport:
    """跑完整对账。见模块文档头。"""
    pairing = pair_series(truth, est, max_diff=max_diff, offset_truth=offset_truth)

    player = _player_report(pairing, est, with_scale=with_scale)
    gap = _gap_report(pairing, est, gate=gap_gate, threshold=threshold,
                      with_scale=with_scale)

    notes: list[str] = []
    if pairing.unmatched_est:
        notes.append(
            f"有 {pairing.unmatched_est}/{len(est)} 个估计帧在 {max_diff:g}s 内找不到真值，"
            "已排除出误差统计（不是识别错，别混进来）")
    if pairing.max_diff() > 0.05:
        notes.append(f"配对时差 max={1e3 * pairing.max_diff():.0f} ms 偏大；"
                     "墙在移动，时差会直接变成位置误差（40 单位/秒 -> 每 25ms 差 1 单位）")
    if player.rigid is not None and player.rigid.translation > 5.0 \
            and player.rigid.rms < 0.5 * player.rigid.rms_before:
        notes.append("角色误差**几乎全是刚体错位**（对齐后残差远小于对齐前）"
                     "-> 不是识别精度问题，先别改识别算法")
        notes.extend(_diagnose_offset(player, gap, pairing))

    # "IDSW=0 但位置错" 这条只在**没有刚性偏移**时才有意义：
    # 有刚性偏移时误差本来就大，报了就是误报（在标定错位的用例上误报过一次）。
    if (gap.identity is not None and gap.identity.IDSW == 0 and gap.cx_err
            and (gap.rigid is None or gap.rigid.translation < 5.0)
            and _err_stats(gap.cx_err)["median"] > 5.0):
        notes.append("缺口 IDSW=0 但中心误差中位偏大，且没有刚性偏移 -> "
                     "可能是相似度门限太宽：逐帧匹配为了保住 id 而配错了位置"
                     "（IDSW 与 IDF1 都看不出来，只能靠位置误差发现）")
    return ReconcileReport(pairing=pairing, player=player, gap=gap, notes=notes)


def _mean_speed(points: Sequence[tuple[float, float]], stamps: Sequence[float]) -> float:
    """平均速率（单位/秒）。用总路程 / 总时间，不是首末位移 —— 拐弯时后者会低估。"""
    if len(points) < 2:
        return 0.0
    dist = sum(math.hypot(points[i + 1][0] - points[i][0],
                          points[i + 1][1] - points[i][1])
               for i in range(len(points) - 1))
    dt = stamps[-1] - stamps[0]
    return dist / dt if dt > 0 else 0.0


def _diagnose_offset(player: "PlayerReport", gap: "GapReport",
                     pairing: Pairing) -> list[str]:
    """把"刚体错位"进一步归因：**标定/机位错** 还是 **时钟偏移/帧过期**。

    为什么不能靠平移方向判：运动接近一维时，**沿运动方向的空间偏移与时间偏移
    在位置上完全等价** —— 数学上不可分。实测踩过：一个 0.5 s 的时钟偏移
    （@120Hz 日志配 30Hz 估计）平移方向与运动方向差 180°，看起来像纯标定错。

    真正锋利的是**平移量随对象速度怎么变**：

    * **时钟偏移**：``est(t) = truth(t - lag)``，于是平移量 = ``速度 x lag``
      —— **每个对象按自己的速度缩放**；
    * **空间/标定偏移**：同一个刚体变换作用在所有对象上
      —— **与速度无关，所有对象平移量相同**。

    所以分别反推 ``lag = 平移量 / 平均速率``：两个对象给出一致的 lag
    就是时钟偏移（并且顺带把 lag 估出来了）；差别很大就是标定错。
    """
    if player.rigid is None or gap.rigid is None:
        return []
    tp, tg = player.rigid.translation, gap.rigid.translation
    if tp < 5.0:
        return []

    p_pts = [t for t, _ in player.vec]
    p_st = [pairing.truth_at[k].stamp for k in range(len(player.vec))]
    v_p = _mean_speed(p_pts, p_st)
    # 缺口用**刚体拟合的同一个 id 分组**（同一堵墙），不能用平铺列表 ——
    # 一场可能有好几堵墙，混起来算速度没有意义。
    g_vec = gap.vec_by_id.get(gap.rigid_id, []) if gap.rigid_id is not None else []
    if len(g_vec) < 2:
        return [f"  · 角色平移 {tp:.2f} 单位 ≈ {1e3 * (tp / v_p):.0f} ms 的时间偏移"
                "（缺口侧帧数不足，无法交叉验证）"] if v_p > 1e-6 else []
    g_pts = [t for t, _ in g_vec]
    g_st = [pairing.truth_at[k].stamp for k in range(len(g_vec))]
    v_g = _mean_speed(g_pts, g_st)
    if v_p <= 1e-6:
        return [f"  · 角色平移 {tp:.2f} 单位；真值几乎不动，无法区分两类病因"]

    lag_p = tp / v_p
    if v_g <= 1e-6 or tg < 1.0:
        return [f"  · 角色平移 {tp:.2f} 单位 ≈ {1e3 * lag_p:.0f} ms 的时间偏移"
                "（缺口几乎不动，无法交叉验证）"]

    lag_g = tg / v_g
    ratio = max(lag_p, lag_g) / max(1e-9, min(lag_p, lag_g))
    if ratio <= 1.35:
        return [f"  · **两个对象反推的时间偏移一致**：角色 {1e3 * lag_p:.0f} ms、"
                f"缺口 {1e3 * lag_g:.0f} ms（速率 {v_p:.0f} vs {v_g:.0f} 单位/秒）"
                "-> **是时钟偏移 / 帧过期**，不是标定：先查采集时戳与 --offset",
                "    注意：offset 若恰为采样间隔的整数倍，**配对时差会显示 0 ms**，"
                "时差诊断完全看不出来（实测 0.5s @120Hz/30Hz 就是这样）"]
    return [f"  · 两个对象反推的时间偏移**对不上**（角色 {1e3 * lag_p:.0f} ms、"
            f"缺口 {1e3 * lag_g:.0f} ms，差 {ratio:.1f} 倍）"
            "-> **不是时钟偏移**（时钟偏移会按各自速度等比缩放），"
            "更像是标定/机位错：重标定或固定相机"]


def _player_report(pairing: Pairing, est: Sequence[FrameEstimate], *,
                   with_scale: bool) -> PlayerReport:
    rep = PlayerReport(n_frames=len(pairing))
    for k, ei in enumerate(pairing.est_indices):
        t = pairing.truth_at[k].player
        e = est[ei].player
        if e is None:
            continue
        rep.n_detected += 1
        if t is None:
            continue
        rep.pos_err.append(math.hypot(e[0] - t[0], e[1] - t[1]))
        # 真值半径缺失（NaN）时**不记这一项** —— 编一个数出来比缺更危险
        if not (math.isnan(e[2]) or math.isnan(t[2])):
            rep.radius_err.append(e[2] - t[2])
        rep.vec.append(((t[0], t[1]), (e[0], e[1])))
    if len(rep.vec) >= 2:
        try:
            rep.rigid = umeyama_2d([p[0] for p in rep.vec], [p[1] for p in rep.vec],
                                   with_scale=with_scale)
        except AlignError:
            # 角色**没动**时所有点重合，刚性变换没有定义（不是数据错误）。
            # 平台默认就是"没有输入所以不动"，所以这条必须能优雅退化 ——
            # 实测在这里抛异常直接把整份报告弄没了。
            rep.stationary = True
    return rep


def _gap_report(pairing: Pairing, est: Sequence[FrameEstimate], *,
                gate: float, threshold: float, with_scale: bool) -> GapReport:
    rep = GapReport(n_frames=len(pairing), gate=gate)
    truth_ids: list[list[int]] = []
    est_ids: list[list[int]] = []
    sims: list[list[list[float]]] = []
    prev_truth: dict[int, tuple[float, float]] = {}
    prev_est: dict[int, tuple[float, float]] = {}
    next_id = 0

    for k, ei in enumerate(pairing.est_indices):
        tg = pairing.truth_at[k].gaps
        eg = est[ei].gaps
        if not tg or not eg:
            rep.n_unmatched += 1
            truth_ids.append([])
            est_ids.append([])
            sims.append([])
            prev_truth, prev_est = {}, {}
            continue

        # 缺口元组是 ``(中心x, 宽度, 墙带中心y)`` —— **空间位置是 (x, y)**，
        # 不是 (x, width)。早先这里用 ``g[:2]`` 取到了宽度当坐标，于是
        # 同一场景里所有墙的"位置"都是 (180, 60)、彼此重合，
        # 跨帧 id 指派把不同墙混成同一个 id，相似度矩阵也是拿宽度算的
        # （自检时抓到：刚体拟合报 RMS 81 单位）。
        t_xy = [(g[0], g[2]) for g in tg]
        e_xy = [(g[0], g[2]) for g in eg]

        # 逐帧的跨帧 id：这里**由对账工具自己**按最近邻指派，
        # 因为视觉侧目前没有跨帧缺口身份（清单 D6）。等流水线有了真 id，
        # 应当换成读它自己的 id，否则这个指标量的是"几何一致性"而不是"跟踪能力"。
        t_ids, prev_truth, next_id = _assign_ids(t_xy, prev_truth, next_id, gate)
        e_ids, prev_est, next_id = _assign_ids(e_xy, prev_est, next_id, gate)
        truth_ids.append(list(t_ids))
        est_ids.append(list(e_ids))
        sims.append(similarity_from_points(t_xy, e_xy, gate=gate))

        # 逐帧记录**所有**配对（一帧可能有多堵墙），误差统计用全部。
        #
        # 这里用 ``threshold=0.0`` —— 也就是"只要在 gate 之内就算配上"。
        # 为什么和身份指标用**不同**的门限：
        #   * **误差提取**要宽容。从严的话，一个较大的刚性偏移会让相似度
        #     ``1 - d/gate`` 掉到门限以下，于是报表显示"一帧都没配上"，
        #     而"偏移有多大"恰恰是我们最想量出来的东西（实测踩到：
        #     24.7 单位的标定偏移被报成"一帧都没配上"）。
        #   * **身份指标**要严格。门限太宽时逐帧匹配会为了保住 id 而配错位置，
        #     IDSW 与 IDF1 都看不出来（见 docs/vision/identity.md §5.2）。
        pairs = _best_pairs(sims[-1], 0.0)
        if not pairs:
            rep.n_unmatched += 1
            continue
        rep.n_matched += 1
        for i, j in pairs:
            rep.cx_err.append(eg[j][0] - tg[i][0])
            rep.width_err.append(eg[j][1] - tg[i][1])
            rep.y_err.append(eg[j][2] - tg[i][2])
            # 刚体拟合按**真值 id** 分组 —— 那是"同一个物理缺口"的定义。
            # **不能**比较 `t_ids[i] == e_ids[j]`：真值与估计的 id 来自两次独立指派，
            # 共享同一个计数器，两个 id 空间天然不相交，那个判断永远不成立
            # （写错过一次，结果是刚体静默地永远不拟合）。
            rep.vec_by_id.setdefault(t_ids[i], []).append(
                ((tg[i][0], tg[i][2]), (eg[j][0], eg[j][2])))
            rep.est_id_of_truth.setdefault(t_ids[i], []).append(e_ids[j])

    # 刚体只在**帧数最多的那一个缺口**上拟合 —— 跨对象的"刚性错位"没有定义
    if rep.vec_by_id:
        rep.rigid_id = max(rep.vec_by_id, key=lambda k: len(rep.vec_by_id[k]))
        v = rep.vec_by_id[rep.rigid_id]
        if len(v) >= 2:
            try:
                rep.rigid = umeyama_2d([p[0] for p in v], [p[1] for p in v],
                                       with_scale=with_scale)
            except AlignError:
                rep.rigid = None        # 缺口没动（缺口静止 + 墙不动）时同样无定义
    if any(truth_ids):
        rep.identity = evaluate_identity(truth_ids, est_ids, sims, threshold=threshold)
    return rep


def _assign_ids(points: Sequence[tuple[float, float]],
                prev: dict[int, tuple[float, float]], next_id: int,
                gate: float) -> tuple[list[int], dict[int, tuple[float, float]], int]:
    """跨帧最近邻指派稳定 id。超出 ``gate`` 就开一个新 id。"""
    ids: list[int] = []
    used: set[int] = set()
    out: dict[int, tuple[float, float]] = {}
    for p in points:
        best, best_d = None, gate
        for i, q in prev.items():
            if i in used:
                continue
            d = math.hypot(p[0] - q[0], p[1] - q[1])
            if d < best_d:
                best, best_d = i, d
        if best is None:
            best = next_id
            next_id += 1
        used.add(best)
        ids.append(best)
        out[best] = p
    return ids, out, next_id


def _best_pairs(sim: Sequence[Sequence[float]], threshold: float) -> list[tuple[int, int]]:
    from .identity import match_frame
    if not sim or not sim[0]:
        return []
    return match_frame(sim, threshold=threshold,
                       est_ids=list(range(len(sim[0]))))


# ---------------------------------------------------------------------------
def _err_stats(v: Sequence[float]) -> dict[str, float]:
    if not v:
        return {"median": float("nan"), "p90": float("nan"), "max": float("nan"),
                "mean": float("nan"), "n": 0}
    a = sorted(abs(x) for x in v)
    n = len(a)
    return {"median": _median(a), "p90": a[min(n - 1, int(0.9 * (n - 1)))],
            "max": a[-1], "mean": sum(a) / n, "n": n}


def _median(v: list[float]) -> float:
    if not v:
        return float("nan")
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


__all__ = [
    "DEFAULT_GAP_GATE", "DEFAULT_MATCH_THRESHOLD",
    "FrameTruth", "FrameEstimate", "Pairing",
    "PlayerReport", "GapReport", "ReconcileReport",
    "pair_series", "reconcile",
]
