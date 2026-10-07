"""对账的地基：时间关联、真值插值、刚体对齐、时钟偏移估计。

四件事各来自一个开源项目，取的是方法不是代码：

============  ==========================================================
来自          取了什么
============  ==========================================================
kalibr        ``TimeOffsetEstimator``：用**下凸包**从"单向时延非负"估出
              两个时钟的仿射关系 ``(slope, offset)`` —— **不需要共同时钟**。
              以及一条纪律：**时戳不单调递增就抛异常**（队列积压会静默污染速度）。
evo           ``associate``：最邻近 + **硬门限 max_diff** + 显式 offset；
              **以短的那条为基准**，短序列的点一个都不丢。
evo           ``umeyama_2d``：把"刚体错位（=标定错）"与"对齐后残差（=识别错）"
              分开报 —— 实测吃过这个亏：同一份识别代码，标定错位时误差 261 场地单位，
              换成同机位标定后 1.78，而当时无法从单个数字判断问题在哪。
nuscenes      真值挂在关键帧上、传感器读数各有自己的时戳 -> 要真值就插值，
              且**插值前把目标时戳夹到区间内**（防外推）。角度用 slerp 不用线性。
============  ==========================================================

为什么放在 ``core/vision/`` 且**不依赖第三方库**
------------------------------------------------
本模块和 ``primitives`` / ``subpixel`` 一样是**纯 Python**：这些量都要往 Verilog 翻
（关联是新点与轨迹的门限比较，凸包是比较+栈，Umeyama 2D 是累加+一次 atan2）。
实测过依赖 numpy 的代价：`_warp` 逐像素版比整幅版慢两个数量级。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

#: 两个时戳相差超过它就**不配对**。
#:
#: 为什么必须要有：没有门限时"最邻近"会**静默**把差好几秒的两帧配到一起，
#: 于是误差统计里混进了"配对错"而不是"识别错"。实测吃过这个亏：
#: `clip.avi` 声明 25 fps 而实际 8.6 fps，回放时相邻帧时差 116 ms ——
#: 任何小于它的门限都会立刻把这件事暴露成"大量点配不上"。
DEFAULT_MAX_DIFF = 1.0 / 30.0


class AlignError(ValueError):
    """对账地基上的任何"这数据不对"都抛它。**绝不静默返回一个看起来合理的数。**"""


# ---------------------------------------------------------------------------
# ① 时戳纪律：单调性
# ---------------------------------------------------------------------------
def check_monotonic(stamps: Sequence[float], *, name: str = "stamps",
                    strict: bool = True) -> None:
    """时戳必须单调递增，否则抛 :class:`AlignError`。

    来自 kalibr 的 ``TimeWentBackwardsException``：远端时戳倒退被当作**故障**，
    不是"数据里有点噪声"。这条对本项目特别对症 ——
    未设 ``CAP_PROP_BUFFERSIZE=1`` 时，队列积压会让"新拿到的帧其实更早"，
    而现象只是速度算错，**静默**。宁可炸。

    ``strict=False`` 允许相等（同一时刻多帧），只禁止倒退。
    """
    prev = None
    for i, t in enumerate(stamps):
        if not isinstance(t, (int, float)) or not math.isfinite(t):
            raise AlignError(f"{name}[{i}] 不是有限数：{t!r}")
        if prev is not None:
            if t < prev or (strict and t == prev):
                what = "倒退" if t < prev else "重复"
                raise AlignError(
                    f"{name} 在 [{i-1}]->[{i}] 时戳{what}：{prev} -> {t}"
                    f"（队列积压/丢帧/时戳来源不对都会这样；"
                    f"先查采集层，不要在对账里遮掉）")
        prev = t


# ---------------------------------------------------------------------------
# ② 时间关联（evo: matching_time_indices + associate_trajectories）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Association:
    """一次关联的结果。**两侧索引一一对应，等长。**"""

    #: 基准侧（通常是**短**的那条）被用到的索引，升序
    idx_ref: tuple[int, ...]
    #: 另一侧被配到的索引
    idx_other: tuple[int, ...]
    #: 每一对的时差（已取绝对值）
    diffs: tuple[float, ...]

    def __len__(self) -> int:
        return len(self.idx_ref)

    @property
    def median_diff(self) -> float:
        return _median(list(self.diffs)) if self.diffs else float("nan")

    @property
    def max_diff(self) -> float:
        return max(self.diffs) if self.diffs else float("nan")


def associate(stamps_ref: Sequence[float], stamps_other: Sequence[float],
              *, max_diff: float = DEFAULT_MAX_DIFF,
              offset_other: float = 0.0,
              name_ref: str = "ref", name_other: str = "other") -> Association:
    """把 ``stamps_ref`` 的每个时戳配到 ``stamps_other`` 里最接近的一个。

    与 evo ``matching_time_indices`` 同一套做法（``evo/core/sync.py:42-67``）：

    1. 对 ``stamps_other`` 加 ``offset_other`` —— **offset 只用于关联**，
       两边各自保留自己的时间基准（`sync.py:120-124`）；
    2. 每个基准点取最邻近；
    3. **只有时差 <= ``max_diff`` 才算配上**（硬门限）。

    配不上就**跳过**（返回的表里没有它），一个都没配上则抛异常 ——
    分两层：底层"能配多少配多少"，上层"一个都没有就是错了，必须炸"、
    evo 用 ``SyncException``（`sync.py:112`）。

    **调用方应当把短的那条当 ``ref``**，这样短序列的点一个都不丢；
    :func:`associate_short_to_long` 帮你做这件事。
    """
    if max_diff <= 0.0:
        raise AlignError(f"max_diff 必须为正，得到 {max_diff}")
    check_monotonic(stamps_ref, name=name_ref, strict=False)

    shifted = [t + offset_other for t in stamps_other]
    idx_ref: list[int] = []
    idx_other: list[int] = []
    diffs: list[float] = []
    for i, t in enumerate(stamps_ref):
        best_j, best_d = -1, float("inf")
        for j, u in enumerate(shifted):
            d = abs(u - t)
            if d < best_d:
                best_j, best_d = j, d
        if best_j >= 0 and best_d <= max_diff:
            idx_ref.append(i)
            idx_other.append(best_j)
            diffs.append(best_d)

    if not diffs:
        raise AlignError(
            f"在 max_diff={max_diff:g}s、offset={offset_other:g}s 下，"
            f"{name_ref} 与 {name_other} **一对都配不上**。"
            f"（{len(stamps_ref)} vs {len(stamps_other)} 个时戳）"
            "常见原因：① 两边时钟有未修的固定偏移 -> 用 --offset 或 "
            "TimeOffsetEstimator 先估；② 时戳单位不一致（ms 当成了 s）；"
            "③ 帧率声明与实际不符（如 clip.avi 声明 25 fps 实际 8.6）。")
    return Association(tuple(idx_ref), tuple(idx_other), tuple(diffs))


def associate_short_to_long(stamps_a: Sequence[float], stamps_b: Sequence[float],
                            **kw) -> tuple[Association, bool]:
    """以**短的那条**为基准做关联（evo: ``sync.py:99-104``）。

    返回 ``(关联结果, b_is_longer)``。基准侧永远是"点更少"的那条，
    所以它不会有任何点因为"找不到对应"而被丢掉 —— 反过来做就会丢，
    而丢掉的那些恰恰是"估计器多输出了"的证据。
    """
    b_longer = len(stamps_b) > len(stamps_a)
    ref, other = (stamps_a, stamps_b) if b_longer else (stamps_b, stamps_a)
    assoc = associate(ref, other, **kw)
    return assoc, b_longer


# ---------------------------------------------------------------------------
# ③ 真值插值（nuscenes: 夹到区间内 + 角度用 slerp）
# ---------------------------------------------------------------------------
def interpolate_linear(stamps: Sequence[float], values: Sequence[float],
                       targets: Iterable[float]) -> list[float]:
    """在 ``stamps`` 上对 ``values`` 线性插值到 ``targets``。

    **目标时戳先夹到 ``[stamps[0], stamps[-1]]``**（nuscenes 的做法，
    ``nuscenes.py:354-355``，原注释是为了防"数据库时戳有偏差"）。
    外推出来的真值比没有真值更危险 —— 它看起来是有效的。
    """
    if len(stamps) != len(values):
        raise AlignError(f"stamps 与 values 长度不同：{len(stamps)} vs {len(values)}")
    if len(stamps) < 2:
        raise AlignError(f"插值至少要 2 个点，得到 {len(stamps)}")
    check_monotonic(stamps, name="插值基准 stamps", strict=False)
    lo, hi = stamps[0], stamps[-1]
    out: list[float] = []
    for t in targets:
        u = lo if t < lo else (hi if t > hi else t)      # 夹住，绝不外推
        j = _bisect(stamps, u)
        t0, t1 = stamps[j], stamps[j + 1]
        if t1 == t0:
            out.append(values[j])
            continue
        w = (u - t0) / (t1 - t0)
        out.append(values[j] * (1.0 - w) + values[j + 1] * w)
    return out


def interpolate_angle(stamps: Sequence[float], angles: Sequence[float],
                      targets: Iterable[float]) -> list[float]:
    """角度插值：**先解环绕，再线性插值，最后绕回**。

    为什么不直接线性插值：从 +179° 到 -179° 的实际变化是 **2°**，
    线性插值会给出 0°（穿过整个 180°）。本项目里墙有 ``rotation``、
    缺口有 ``axis``，逐帧比角度时必须处理这件事。
    """
    if len(stamps) != len(angles):
        raise AlignError(f"stamps 与 angles 长度不同：{len(stamps)} vs {len(angles)}")
    unwrapped = [angles[0]]
    for a in angles[1:]:
        prev = unwrapped[-1]
        d = _wrap(a - prev)
        unwrapped.append(prev + d)
    return [_wrap(v) for v in interpolate_linear(stamps, unwrapped, targets)]


def _wrap(a: float) -> float:
    """把角度规约到 ``(-pi, pi]``。"""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def angle_diff(a: float, b: float) -> float:
    """两个角度的最小差（已解环绕）。**比角度误差只能用这个。**"""
    return _wrap(a - b)


def _bisect(stamps: Sequence[float], t: float) -> int:
    """返回 ``j`` 使 ``stamps[j] <= t <= stamps[j+1]``，且 ``j <= len-2``。"""
    lo, hi = 0, len(stamps) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if stamps[mid] <= t:
            lo = mid
        else:
            hi = mid - 1
    return min(lo, len(stamps) - 2)


# ---------------------------------------------------------------------------
# ④ 2D 刚体对齐（evo: umeyama_alignment 的二维特例）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Rigid2D:
    """二维相似变换 ``dst ≈ scale * R(theta) @ src + (tx, ty)``。"""

    #: 旋转角（弧度）
    theta: float
    tx: float
    ty: float
    #: 尺度。``with_scale=False`` 时恒为 1.0
    scale: float
    #: 该变换下各点的 RMS 残差（= **对齐后**的误差，也就是"识别误差"）
    rms: float
    #: 对齐**前**的 RMS 误差（含刚体错位，也就是"实测误差"）
    rms_before: float
    n: int
    #: **质心位移**（各点平均移动了多少）。这是诊断该看的量。
    mean_dx: float = 0.0
    mean_dy: float = 0.0

    @property
    def translation(self) -> float:
        """**质心位移**的模长（场地单位）。标定错位的主要表现形式。

        为什么不用 ``hypot(tx, ty)``：那个是**原点处**的平移量，
        混进了"旋转 x 质心距离"这一项。实测真实数据上：质心在 (320,55)、
        旋转只有 -0.60°，``hypot(tx,ty)`` 报 **5.09**，而实际平均位移只有
        **约 1.8** —— 差出来的 3.3 全是旋转杠杆。
        更糟的是这个杠杆**随对象到原点的距离变化**，于是"比较角色与缺口的平移量"
        那条时钟偏移判据会被污染（两者离原点距离不同）。
        """
        return math.hypot(self.mean_dx, self.mean_dy)

    @property
    def origin_translation(self) -> float:
        """``hypot(tx, ty)``：原点处的平移量。仅作参考，**不要用它做诊断**。"""
        return math.hypot(self.tx, self.ty)

    def apply(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.theta) * self.scale, math.sin(self.theta) * self.scale
        return (c * x - s * y + self.tx, s * x + c * y + self.ty)

    def to_dict(self) -> dict:
        return {"theta_deg": math.degrees(self.theta), "tx": self.tx, "ty": self.ty,
                "translation": self.translation,
                "rigid_part": self.rigid_part,
                "origin_translation": self.origin_translation,
                "mean_dx": self.mean_dx, "mean_dy": self.mean_dy,
                "scale": self.scale,
                "rms": self.rms, "rms_before": self.rms_before, "n": self.n}

    @property
    def rigid_part(self) -> float:
        """刚性错位贡献的那部分误差：``sqrt(rms_before^2 - rms^2)``。

        这是**唯一**不会误导的"刚体错位有多大"。为什么不报平移量：
        有旋转时"平移量"取决于你在哪个点量 —— 构造上的平移是 1.803，
        质心位移是 3.130（含"旋转 x 质心距离"），而实际 RMS 误差是 3.131。
        三个数各不相同，只报其中一个就会被误读（实测在真实数据上把
        1.79 的误差报成"平移 5.09"，多出来的全是旋转杠杆）。
        """
        d = self.rms_before * self.rms_before - self.rms * self.rms
        return math.sqrt(d) if d > 0.0 else 0.0

    def summary(self) -> str:
        return (f"刚体: 实测 RMS {self.rms_before:.2f} 单位 "
                f"= 刚体 {self.rigid_part:.2f} + 残余 {self.rms:.2f}（残余=识别误差）\n"
                f"      质心位移 {self.translation:.2f} 单位  "
                f"旋转 {math.degrees(self.theta):+.2f}°  尺度 {self.scale:.4f}"
                f"  (n={self.n})")


def umeyama_2d(src: Sequence[tuple[float, float]],
               dst: Sequence[tuple[float, float]], *,
               with_scale: bool = False) -> Rigid2D:
    """最小二乘求二维相似变换，把 ``src`` 对齐到 ``dst``（Umeyama 1991 的二维形式）。

    二维有闭式解，**不需要 SVD**（三维才需要）：

        theta = atan2( sum(x_i x y_i), sum(x_i . y_i) )        # 中心化之后
        scale = sqrt( sum|y_i|^2 / sum|x_i|^2 )
        t     = mean(dst) - scale * R(theta) @ mean(src)

    返回值里同时带 ``rms_before`` 与 ``rms``：

    * ``rms_before`` 大、``rms`` 小  -> **是标定/配准错位**，识别本身没错；
    * ``rms_before`` 与 ``rms`` 都大 -> **是识别误差**。

    这条区分是实测逼出来的：同一份识别代码，用错机位的标定去对账得到
    261 场地单位误差，换成同机位标定降到 1.78 —— 而当时只有一个数字，看不出病因。

    **``with_scale=True`` 要慎用**：它会把"尺度标定错了"也吸收掉，
    于是标定错误看起来像识别很好。默认关掉，让尺度误差显式暴露。
    """
    n = len(src)
    if n != len(dst):
        raise AlignError(f"src 与 dst 点数不同：{n} vs {len(dst)}")
    if n < 2:
        raise AlignError(f"对齐至少要 2 个点，得到 {n}")

    mx = sum(p[0] for p in src) / n
    my = sum(p[1] for p in src) / n
    ux = sum(p[0] for p in dst) / n
    uy = sum(p[1] for p in dst) / n

    a = b = sx2 = sy2 = 0.0
    for (x, y), (u, v) in zip(src, dst):
        dx, dy = x - mx, y - my
        du, dv = u - ux, v - uy
        a += dx * du + dy * dv          # 点积和
        b += dx * dv - dy * du          # 叉积和
        sx2 += dx * dx + dy * dy
        sy2 += du * du + dv * dv

    if sx2 <= 0.0:
        raise AlignError("src 的所有点重合，无法定旋转（这不是对齐问题，是数据问题）")
    theta = math.atan2(b, a)
    if with_scale:
        scale = math.sqrt(sy2 / sx2) if sx2 > 0.0 else 1.0
    else:
        scale = 1.0

    c, s = math.cos(theta) * scale, math.sin(theta) * scale
    tx = ux - (c * mx - s * my)
    ty = uy - (s * mx + c * my)

    def res2(px, py, qx, qy, cc, ss, ttx, tty):
        ex = cc * px - ss * py + ttx - qx
        ey = ss * px + cc * py + tty - qy
        return ex * ex + ey * ey

    acc = 0.0
    for (x, y), (u, v) in zip(src, dst):
        acc += res2(x, y, u, v, c, s, tx, ty)
    rms = math.sqrt(acc / n)
    acc0 = sum(res2(x, y, u, v, 1.0, 0.0, 0.0, 0.0) for (x, y), (u, v) in zip(src, dst))
    rms_before = math.sqrt(acc0 / n)

    # 质心位移 = 各点平均移动量。``t`` 是原点处的平移，两者差一个"旋转 x 质心"。
    c2, s2 = math.cos(theta) * scale, math.sin(theta) * scale
    mean_dx = (c2 * mx - s2 * my + tx) - mx
    mean_dy = (s2 * mx + c2 * my + ty) - my

    return Rigid2D(theta=theta, tx=tx, ty=ty, scale=scale,
                   rms=rms, rms_before=rms_before, n=n,
                   mean_dx=mean_dx, mean_dy=mean_dy)


# ---------------------------------------------------------------------------
# ⑤ 时钟偏移估计（kalibr: TimestampCorrector 的下凸包）
# ---------------------------------------------------------------------------
@dataclass
class TimeOffsetEstimator:
    """估出两个时钟的仿射关系 ``local = slope * remote + offset``。

    来自 kalibr ``sm_timing/TimestampCorrector.hpp:8-18``
    （Zhang/Liu/Xia, INFOCOM 2002 的单向时间同步凸包算法）。

    物理关系是 ``local = slope * remote + offset + delay``，其中 **delay >= 0**
    （相机：曝光 -> 读出 -> 传输都是正的）。于是散点全部落在那条直线的**上方**，
    它的**下凸包**就是最紧的下界估计 —— **不需要两边共享时钟、不需要同步硬件**。

    与 ``TimestampCorrector`` 的差异（如实记录）：kalibr 用凸包的**中点段**
    （``_midpointSegmentIndex``）取斜率和截距，本实现也取中点段，
    但返回值额外给出 ``hull`` 供人工核查；kalibr 是**在线递推**，
    本实现是**离线整批**（我们的对账是离线场景）。
    """

    #: 点对 ``(remote, local)``，remote 必须单调递增
    points: list[tuple[float, float]] = field(default_factory=list)

    def add(self, remote: float, local: float) -> None:
        if self.points and remote < self.points[-1][0]:
            raise AlignError(
                f"remote 时戳倒退：{self.points[-1][0]} -> {remote}。"
                "kalibr 在同样情况下抛 TimeWentBackwardsException ——"
                "别在这里遮掉，先查采集层为什么给了倒退的时戳")
        self.points.append((float(remote), float(local)))

    def extend(self, remotes: Sequence[float], locals_: Sequence[float]) -> None:
        for r, l in zip(remotes, locals_):
            self.add(r, l)

    @property
    def hull(self) -> list[tuple[float, float]]:
        """``(remote, local)`` 的下凸包（按 remote 升序）。"""
        hull: list[tuple[float, float]] = []
        for p in self.points:
            while len(hull) >= 2 and _cross(hull[-2], hull[-1], p) <= 0.0:
                hull.pop()
            hull.append(p)
        return hull

    def slope_offset(self) -> tuple[float, float]:
        """取凸包**中点段**的斜率和截距（kalibr 的做法）。"""
        h = self.hull
        if len(h) < 2:
            raise AlignError(f"凸包只有 {len(h)} 个点，估不出斜率（至少需要 2 个不同 remote）")
        x_mid = 0.5 * (self.points[0][0] + self.points[-1][0])
        idx = 0
        for i in range(len(h) - 1):
            if h[i][0] <= x_mid:
                idx = i
            else:
                break
        idx = min(idx, len(h) - 2)
        (x0, y0), (x1, y1) = h[idx], h[idx + 1]
        if x1 == x0:
            raise AlignError("凸包中点段的两个 remote 相同，斜率无定义")
        slope = (y1 - y0) / (x1 - x0)
        return slope, y0 - slope * x0

    @property
    def slope(self) -> float:
        return self.slope_offset()[0]

    @property
    def offset(self) -> float:
        return self.slope_offset()[1]

    def to_local(self, remote: float) -> float:
        slope, offset = self.slope_offset()
        return slope * remote + offset

    def health(self) -> str:
        """把估出来的量翻译成一句人话。``slope`` 应≈1，偏离说明丢帧或时戳在编。"""
        if len(self.points) < 2:
            return "点太少，无法评估"
        slope, offset = self.slope_offset()
        n = len(self.points)
        d = slope - 1.0
        flag = "正常" if abs(d) <= 5e-3 else ("**可疑**" if abs(d) <= 5e-2 else "**异常**")
        return (f"slope={slope:.6f}（偏离 1 有 {d:+.2e}，{flag}）"
                f" offset={offset:+.4f}s  凸包点数={len(self.hull)}/{n}")


def _cross(o: tuple[float, float], a: tuple[float, float],
           b: tuple[float, float]) -> float:
    """叉积 ``(a-o) x (b-o)``。正 = 逆时针。"""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _median(v: list[float]) -> float:
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


__all__ = [
    "DEFAULT_MAX_DIFF", "AlignError",
    "check_monotonic",
    "Association", "associate", "associate_short_to_long",
    "interpolate_linear", "interpolate_angle", "angle_diff",
    "Rigid2D", "umeyama_2d",
    "TimeOffsetEstimator",
]


# ---------------------------------------------------------------------------
# 内容对齐（从根脚本 vision_reconcile.py 下沉）
# ---------------------------------------------------------------------------
def _align_by_content(truth, est, *, span: float = 4.0, step: float = 0.02,
                      gate: float = 0.10, sample: int = 80):
    """不共时钟时的对齐：**用画面内容**反推时间偏移。

    视频（你自己在 Windows 录的）与平台状态日志是两个时钟、两个起点，
    没有任何共同的时戳可用。但视频里拍的**就是**平台 —— 于是：
    平台日志说"缺口在场地 (192, 412)"，视频那一帧检测出的缺口也在场地单位下，
    两者在**同一个场地坐标系**里应当重合。扫一遍时间偏移 δ，
    取让两边缺口距离最小的那个 δ。

    用**倒角距离**（每个估计缺口到最近的真值缺口的距离）而不是一一配对：
    场上可能同时有好几堵墙，倒角距离对"多一个/少一个"不敏感。

    **周期场景的歧义，与为什么加上角色**
    ------------------------------------
    墙是每隔固定时间生成一堵的，于是**只看缺口**的话画面每隔一个生成周期
    就长得一样，评分曲线出现等间距的多个极小值（实测相邻极小值 1.66s，
    正是墙的生成周期），全局最小可能落到隔壁周期上。
    单靠缺口**无法**消除这个歧义 —— 这是内容本身的对称性，不是算法不够好。

    解法是引入**非周期**的信息：角色。角色由人驾驶，轨迹不周期；
    真值日志里有角色的逐帧位置，视频里也能检出角色。
    所以评分 = 缺口倒角中位 + 角色位置误差中位（都是场地单位）。
    角色一动，隔壁周期的解立刻被拉开。
    若整段录像角色没动（人没操作），这一项对所有 δ 都是常数，
    消歧能力为零 —— 此时靠 ``rivalry`` 如实报出**歧义比**，
    由调用方告警，而不是假装算准了。

    歧义比 ``rivalry`` = 次优簇的评分 / 最优簇的评分。
    远大于 1 说明落点明确；接近 1 说明两种偏移一样好，**不可信**。

    返回 ``(δ, 缺口残差中位, 角色残差中位, 歧义比)``；
    样本太少或没有缺口时返回 ``(None, nan, nan, nan)``。
    """
    import math
    _nan = float("nan")
    if not truth or not est:
        return None, _nan, _nan, _nan
    t_stamps = [f.stamp for f in truth]
    step_i = max(1, len(est) // max(1, sample))
    probes = est[::step_i]
    if not any(e.gaps for e in probes) or not any(f.gaps for f in truth):
        return None, _nan, _nan, _nan

    def _nearest(target: float) -> int:
        lo, hi = 0, len(t_stamps) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if t_stamps[mid] < target:
                lo = mid + 1
            else:
                hi = mid
        if lo > 0 and abs(t_stamps[lo - 1] - target) <= abs(t_stamps[lo] - target):
            return lo - 1
        return lo

    def _median(xs: list[float]) -> float:
        xs = sorted(xs)
        return xs[len(xs) // 2]

    def _score(delta: float) -> tuple[float, float, float]:
        """-> (缺口残差中位, 角色残差中位, 排序用评分)。

        角色项只在**两边都检出**时计入；一个都没检出就退化成纯缺口评分，
        并在返回值里如实给出 nan（调用方据此知道这一轮没有消歧能力）。

        **取最近帧，不做插值**：状态日志 20Hz（帧间隔 0.05s），
        δ 的搜索步长 0.02s 比它细，所以连续几个 δ 会取到同一帧、
        评分相同 —— δ 因此只能分辨到**半帧（±0.025s）**。
        这一点必须如实承认：插值能算出更细的 δ，但那是在
        "角色匀速"的假设上编出来的精度，实测链路里没有依据；
        0.025s × 40 单位/秒 = 1 个场地单位，本来也不是瓶颈。
        """
        d: list[float] = []
        dp: list[float] = []
        for e in probes:
            j = _nearest(e.stamp - delta)
            if abs(t_stamps[j] - (e.stamp - delta)) > gate:
                continue
            tg = truth[j].gaps
            if e.gaps and tg:
                # 倒角：每个估计缺口 -> 最近的真值缺口
                for (ex, _ew, ey) in e.gaps:
                    d.append(min(math.hypot(ex - tx, ey - ty) for (tx, _tw, ty) in tg))
            tp, ep = truth[j].player, e.player
            if tp is not None and ep is not None:
                dp.append(math.hypot(ep[0] - tp[0], ep[1] - tp[1]))
        if len(d) < 5:
            return float("inf"), _nan, float("inf")
        g = _median(d)
        # 角色项至少要有 3 个样本才敢参与排序，否则噪声说了算。
        p = _median(dp) if len(dp) >= 3 else _nan
        return g, p, g if not math.isfinite(p) else g + p

    scored: list[tuple[float, float, float, float]] = []   # (δ, gap, player, key)
    n_steps = int(round(2 * span / step)) + 1
    for i in range(n_steps):
        delta = -span + i * step
        g, p, key = _score(delta)
        if math.isfinite(key):
            scored.append((delta, g, p, key))
    if not scored:
        return None, _nan, _nan, _nan

    # **次优解 = 另一个"坑"（局部极小）**，不是"出了这个坑之后随便一个点"。
    # 早先的写法是"从最优解往外走，走出坑之后的第一个点"，那样量到的是
    # **坑有多宽**（被自己的阈值卡住，歧义比永远报 ~3），
    # 而不是"次好的解释有多好"。必须找真正的局部极小值。
    #
    # 但逐帧检测噪声会让评分曲线带高频抖动，产生一堆"只差一格"的假极小值。
    # 所以要**先中位滤波再找坑**：窗口 5 格 = 0.1s，足以抹掉抖动，
    # 又远小于真正的周期歧义间距（1.6s），不会把真坑抹平。
    keys = [r[3] for r in scored]
    half = 2
    smooth = []
    for i in range(len(keys)):
        w = keys[max(0, i - half):i + half + 1]
        smooth.append(sorted(w)[len(w) // 2])

    valley_idx = [i for i in range(len(scored))
                  if (i == 0 or smooth[i - 1] >= smooth[i])
                  and (i == len(scored) - 1 or smooth[i + 1] >= smooth[i])]
    # 相邻格点（同一个坑的平底）会有多个，归并成一个坑。
    # 阈值取**平滑窗宽**（2*half+1 格 = 0.10s）：一个坑的平底不会比平滑窗更宽，
    # 而真正的另一个坑至少隔着一个周期（1.6s），不会被误并。
    # **归并只用来判断"有几个坑"**；每个坑的代表点必须回到**原始评分**
    # 的最小格点 —— 平滑只是让坑的个数数得准，拿平滑曲线挑代表点会把
    # 最优解挪走（实测 δ 从 -0.05 挪到 0.00，缺口残差 0.5 变成 2.2）。
    # 归并**只和本坑的第一个格点比**，不要链式累加：平底上的假极小值
    # 一个接一个、每个都距上一个 ≤ merge_w，链式比较会把整个搜索范围
    # 并成一个坑（然后"没有次优解"被误读成"落点唯一"）。
    merge_w = (2 * half + 2) * step          # 6 格 = 0.12s
    groups: list[list[int]] = []
    for i in valley_idx:
        if groups and scored[i][0] - scored[groups[-1][0]][0] <= merge_w + 1e-9:
            groups[-1].append(i)
        else:
            groups.append([i])
    if not groups:                       # 评分曲线单调（理论上不该发生）
        groups = [[i0]]

    # 每个坑的代表 = 该坑范围内**原始评分最小**的格点
    rep = [min(g, key=lambda i: keys[i]) for g in groups]
    vkey = [keys[i] for i in rep]
    best_slot = min(range(len(rep)), key=lambda s: vkey[s])
    best_i = rep[best_slot]
    best_d, best_g, best_p = scored[best_i][0], scored[best_i][1], scored[best_i][2]
    best_key = vkey[best_slot]

    # 次优 = 最优坑**之外**的最小评分。
    #
    # 为什么不只用"其他坑"：评分曲线**完全平**的时候（精确周期的场景就是
    # 这样）根本找不到局部极小，坑的个数是 0 或 1。早先按"只有一个坑 =>
    # 落点唯一 => 可信"处理，正好把**最歧义**的情况报成了最可信 —— 完全反了。
    # 改成在最优坑的邻域之外取最小值：平的时候它 ≈ 最优（歧义比 ≈ 1，如实），
    # 尖的时候它远大于最优（歧义比大，可信）。
    lo = scored[groups[best_slot][0]][0] - merge_w
    hi = scored[groups[best_slot][-1]][0] + merge_w
    outside = [keys[i] for i in range(len(scored))
               if scored[i][0] < lo or scored[i][0] > hi]
    if not outside:
        rivalry = float("inf")           # 整个搜索范围就一个坑：落点唯一
    elif best_key <= 1e-9:
        # 无噪声的理想数据：评分能到正好 0。此时"比值"没有意义 ——
        # 要么次优也是 0（真歧义），要么不是（唯一解）。
        rivalry = 1.0 if min(outside) <= 1e-9 else float("inf")
    else:
        rivalry = min(outside) / best_key
    return best_d, best_g, best_p, rivalry

def _fit_line_robust(xs: list[float], ys: list[float]) -> tuple[float, float, list[bool]]:
    """最小二乘拟合 ``y = a*x + b``，用 MAD 剔一轮离群点。

    为什么需要剔：某个窗口的内容对齐可能整体错一个墙周期（周期场景的老问题），
    那个点会把斜率整个带偏。MAD 只看"离群多远"，不假设噪声分布，比标准差稳。
    返回 ``(a, b, 内点掩码)``。
    """
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0), [True] * n

    def _fit(xx, yy):
        m = len(xx)
        sx, sy = sum(xx), sum(yy)
        sxx = sum(v * v for v in xx)
        sxy = sum(a * b for a, b in zip(xx, yy))
        den = m * sxx - sx * sx
        if abs(den) < 1e-12:
            return 0.0, sy / m
        a = (m * sxy - sx * sy) / den
        return a, (sy - a * sx) / m

    a, b = _fit(xs, ys)
    res = [y - (a * x + b) for x, y in zip(xs, ys)]
    med = sorted(res)[len(res) // 2]
    mad = sorted(abs(r - med) for r in res)[len(res) // 2]
    if mad <= 1e-9:
        return a, b, [True] * n
    keep = [abs(r - med) <= 3.5 * 1.4826 * mad for r in res]
    if sum(keep) >= 2:
        a, b = _fit([x for x, k in zip(xs, keep) if k],
                    [y for y, k in zip(ys, keep) if k])
    return a, b, keep

def _align_affine(truth, est, *, n_win: int = 5, span: float = 4.0,
                  step: float = 0.02, gate: float = 0.10, min_span: float = 20.0):
    """内容对齐的**仿射**版本：``δ(t) = a*t + b``，用来容纳录像时间轴的漂移。

    为什么常数 δ 不够（实测）
    ------------------------
    一段 104s 的 Windows 录屏，分窗各自对齐的结果是::

        窗口  6.7s -> δ=-1.40      40s -> δ=-0.98
             73.0s -> δ=-0.50      93s -> δ=-0.22

    四点几乎完美落在一条直线上（``δ = 0.0137 t - 1.49``，1.37% 的速率差）。
    **每个窗口单独看都很干净**（缺口残差 1~2 单位），但整段用一个常数 δ
    怎么都对不齐 —— 强行取全局最优会得到一个折中的 δ，残差十几单位，
    而且评分曲线变得平坦、歧义比接近 1（工具会告警，但告警也救不回数据）。

    成因：录屏软件掉帧，而封装仍按 60fps 写均匀时戳（实测容器 PTS 是
    严格均匀的 1/60，偏差 0.000 —— 所以**不能靠读 PTS 发现**）。
    平台侧的时钟是真时间（同一份日志里 ``stamp`` 与 ``wall`` 全程只差 8ms），
    所以漂移在录像那一侧。

    做法：按时间分窗，每窗独立求 δ，再稳健拟合一条直线，然后把
    ``est`` 的时戳改写成 ``t - δ(t)``（见调用处）。这样下游
    ``pair_series`` 不用改（它只认常数偏移，此时偏移已经是 0）。

    返回 ``(a, b, rows, ok)``；``ok=False`` 表示样本不足，调用方应退回常数 δ。
    ``rows`` 是每窗的 ``(t_mid, δ, 缺口残差, 角色残差, 歧义比)``，供打印核查。
    """
    _nan = float("nan")
    if not est or not truth:
        return 0.0, 0.0, [], False
    t_lo, t_hi = est[0].stamp, est[-1].stamp
    if (t_hi - t_lo) < min_span or len(est) < 40:
        return 0.0, 0.0, [], False          # 太短，漂移估不出来，交给常数 δ

    edges = [t_lo + (t_hi - t_lo) * k / n_win for k in range(n_win + 1)]
    rows = []
    for k in range(n_win):
        lo, hi = edges[k], edges[k + 1]
        win = [e for e in est if lo <= e.stamp <= hi] if k == n_win - 1 \
            else [e for e in est if lo <= e.stamp < hi]
        if len(win) < 20:
            continue
        d, g, p, r = _align_by_content(truth, win, span=span, step=step, gate=gate)
        if d is None:
            continue
        rows.append((0.5 * (win[0].stamp + win[-1].stamp), d, g, p, r))
    if len(rows) < 2:
        return 0.0, 0.0, rows, False

    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    a, b, keep = _fit_line_robust(xs, ys)
    # 斜率 >= 1 意味着"修正后时间不前进甚至倒退"，那是拟合失败而不是漂移。
    if not (-1.0 < a < 1.0):
        return 0.0, 0.0, rows, False
    return a, b, rows, True
