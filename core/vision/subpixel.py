"""亚像素边缘估计 —— 把"阈值卡在哪"换成"边缘真的在哪"。

问题
----
二值掩膜只能给出**整数**边界，而且阈值一旦定死，边界就落在亮度斜坡的某个位置，
不是真正的边缘位置。实测两个后果：

* 参考物（洋红块）的四角：斜坡宽约 8 px，阈值 90% 卡在斜坡下沿，
  量出来的矩形比真实的小 1~2 px，标定尺度系统性偏 ~3%；
* 角色（白圆）的半径：相机模糊把亮块边缘外扩约 1 px，
  在半径只有 ~7 px 时就是 **+16%**。半径进了"缺口 ≥ 2r"这条硬约束，
  量大了偏保守、量小了我们就把危险判成安全。

做法
----
边缘两侧各有一个**平台**（里面是目标、外面是背景）。取两者的**半高**
``T = (hi + lo) / 2`` 作为判据，再在剖面上下穿的那对相邻采样点之间做
**线性插值**：

    index = i - 1 + (2·v[i-1] - (hi+lo)) / (2·(v[i-1] - v[i]))

这一步是对的：模糊是对称的，半高位置就等于无模糊时的真实边缘位置
（对任何对称的点扩散函数都成立），而阈值位置不是。

硬件映射
--------
* 平台电平 ``hi/lo``：两条累加器 + 比较器；
* 穿越判定：把 ``2v`` 与 ``hi+lo`` 比 —— 一个加法器 + 一个比较器；
* 插值：一次减法 + **一次除法**。除法是这里唯一的"贵"操作，
  和单应映射里的 ``1/W'`` 同级；要省可以换成倒数查找表 + 一次乘法。
* 全程不需要整帧缓存，逐行/逐列扫描即可。
"""

from __future__ import annotations

import math
from typing import Callable, Iterable, Sequence

Point = tuple[float, float]

#: 每像素取一个打分（越大越像目标）。返回 Python int，方便逐位复现。
Scorer = Callable[[int, int, int], int]


def bright_score(r: int, g: int, b: int) -> int:
    """亮度：用于白圆角色（取三通道最大值，等价于"最亮的那个通道"）。"""
    return r if r > g else g if g > b else b


def magenta_score(r: int, g: int, b: int, *, weak_pct: int = 90) -> int:
    """洋红参考物的"像不像"程度：``min(R,B) - G``，被压制时归零。

    直接减会在无彩色处得到正数（R≈B>G 不成立时才是 0），所以再乘一个
    "弱通道确实被压住"的判据 —— 和 `calibration.detect_marker_quad`
    用的是同一条颜色键，保证"掩膜"和"亚像素精修"看的是同一个东西。
    """
    weak = g
    strong = r if r < b else b
    if weak * 100 > strong * weak_pct:
        return 0
    return strong - weak


# --------------------------------------------------------------------------
# 一维：找半高穿越点
# --------------------------------------------------------------------------
def half_level_edge(values: Sequence[float], *, hi: float | None = None,
                    lo: float | None = None) -> float | None:
    """在剖面上找**从高到低**的第一个半高穿越点，返回**分数下标**。

    ``values`` 假定是"从目标内部往背景走"。``hi``/``lo`` 不给就从剖面自身估
    （取前若干点的中位与后若干点的中位），给了就按给定的算 ——
    给固定电平更稳，因为逐帧估计会被噪声带偏。

    找不到穿越点返回 ``None``（剖面整个在目标里或整个在背景里）。
    """
    n = len(values)
    if n < 3:
        return None
    if hi is None or lo is None:
        head = sorted(values[:max(2, n // 8)])
        tail = sorted(values[-max(2, n // 8):])
        hi = head[len(head) // 2] if hi is None else hi
        lo = tail[len(tail) // 2] if lo is None else lo
    if hi - lo <= 0:
        return None
    twice_thr = hi + lo                       # 与 2*v 比较，避免除以 2
    prev = 2 * values[0] - twice_thr
    for i in range(1, n):
        cur = 2 * values[i] - twice_thr
        if prev > 0 >= cur:                   # 高 -> 低
            span = prev - cur
            if span <= 0:
                return float(i - 1)
            # 分数的部分：prev/span ∈ [0,1)
            return (i - 1) + prev / span
        prev = cur
    return None


# --------------------------------------------------------------------------
# 在 Frame 上取剖面
# --------------------------------------------------------------------------
def _sample(frame, x: float, y: float, score: Scorer) -> float:
    """**双线性**采样，坐标是"像素中心"约定（像素 i 的采样位置就是 i）。

    为什么不用最近邻：斜边上的采样点会落在像素之间，最近邻会引入最多半个像素的
    量化误差 —— 那正好把亚像素精修的意义抵消掉。双线性只要一个 2x2 邻域，
    在 FPGA 上就是**两条行缓冲 + 两次线性插值**，成本可接受。

    出界按背景（0）处理，避免边缘跑到画面外还在"外推"。
    """
    w, h = frame.width, frame.height
    x0, y0 = math.floor(x), math.floor(y)
    fx, fy = x - x0, y - y0

    def at(ix: int, iy: int) -> float:
        if 0 <= ix < w and 0 <= iy < h:
            i = iy * w + ix
            return float(score(frame.r[i], frame.g[i], frame.b[i]))
        return 0.0

    top = at(x0, y0) * (1.0 - fx) + at(x0 + 1, y0) * fx
    bot = at(x0, y0 + 1) * (1.0 - fx) + at(x0 + 1, y0 + 1) * fx
    return top * (1.0 - fy) + bot * fy


def _sample_along(frame, x0: float, y0: float, dx: float, dy: float,
                  count: int, score: Scorer) -> list[float]:
    """从 ``(x0,y0)`` 沿 ``(dx,dy)`` 取 ``count`` 个双线性采样。"""
    return [_sample(frame, x0 + dx * k, y0 + dy * k, score) for k in range(count)]


def _line_through(p: Point, q: Point) -> tuple[float, float, float]:
    """两点确定的直线 ``(a, b, c)``，满足 ``a·x + b·y + c = 0``。"""
    a = p[1] - q[1]
    b = q[0] - p[0]
    return a, b, -(a * p[0] + b * p[1])


def _intersect(l1: tuple[float, float, float],
               l2: tuple[float, float, float]) -> Point | None:
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    return ((b1 * c2 - b2 * c1) / det, (a2 * c1 - a1 * c2) / det)


def _centroid(pts: Iterable[Point]) -> Point:
    pts = list(pts)
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


# --------------------------------------------------------------------------
# 四边形：四条边各推一点，再求交
# --------------------------------------------------------------------------
def refine_quad(frame, quad: Sequence[Point], *, score: Scorer = magenta_score,
                samples: int = 7, span: float = 4.0, levels: tuple[int, int] | None = None,
                ) -> list[Point]:
    """把整数四角精修到亚像素。

    对四条边各自：沿边的法线方向取若干条剖面，每条给出一个半高穿越点，
    取**偏移量的中位**当作这条边整体该往外挪多少；最后用挪过的四条边两两求交。

    为什么只做"平移"不做"拟合斜率"：参考物的边在透视下确实是直线，
    但每条边上几个采样点之间的斜率差异远小于平移量（实测平移 ~1.5 px、
    斜率差 <0.2 px），先解决主要矛盾。真要做完整直线拟合也只是多两组累加器
    （Σx, Σy, Σx², Σxy），留作后续。
    """
    if len(quad) != 4:
        raise ValueError(f"需要 4 个角点，得到 {len(quad)}")
    # 坐标约定：进来的四角是**边**约定（像素 i 覆盖 [i, i+1)），
    # 而双线性采样是**中心**约定（像素 i 的采样位置就是 i）。
    # 两者差 0.5，必须显式换 —— 少这一步会稳定偏半个像素，
    # 而半个像素在这个尺度上就是 1~2% 的尺度误差。
    quad = [(p[0] - 0.5, p[1] - 0.5) for p in quad]
    cx, cy = _centroid(quad)
    edges: list[tuple[float, float, float]] = []
    for k in range(4):
        p = quad[k]
        q = quad[(k + 1) % 4]
        ex, ey = q[0] - p[0], q[1] - p[1]
        length = math.hypot(ex, ey)
        if length < 1e-6:
            return list(quad)
        # 外法线：垂直于边、且背离形心
        nx, ny = ey / length, -ex / length
        mx, my = (p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0
        if nx * (mx - cx) + ny * (my - cy) < 0:
            nx, ny = -nx, -ny

        pts: list[Point] = []
        for s in range(samples):
            t = (s + 1) / (samples + 1)
            bx = p[0] + ex * t
            by = p[1] + ey * t
            # 剖面：从边内侧 span 处，向外走到 span 处（步长 1，方向是单位法线）
            start = (bx - nx * span, by - ny * span)
            prof = _sample_along(frame, start[0], start[1], nx, ny,
                                 int(2 * span) + 1, score)
            idx = half_level_edge(prof, hi=None if levels is None else levels[0],
                                  lo=None if levels is None else levels[1])
            if idx is None:
                continue
            # 采样点 k 的位置是 base + n·(k - span)，所以穿越点就是
            # base + n·(idx - span) —— 一个**亚像素的边缘点**
            off = idx - span
            pts.append((bx + nx * off, by + ny * off))
        # **拟合一条直线**，而不是只把原边平移一个中位量。
        # 平移版看不见"边本身有斜度/弯曲"（透视下确实会），
        # 于是四个角只能靠原四边形的方向，尺度会残留几个百分点的偏差。
        edges.append(_fit_line(pts) if len(pts) >= 3 else _line_through(p, q))

    corners: list[Point] = []
    for k in range(4):
        hit = _intersect(edges[(k - 1) % 4], edges[k])
        corners.append(hit if hit is not None else quad[k])
    return [(p[0] + 0.5, p[1] + 0.5) for p in corners]      # 换回边约定


def _fit_line(points: Sequence[Point]) -> tuple[float, float, float]:
    """亚像素边缘点 → 最小二乘直线 ``(a, b, c)``。

    用**总体最小二乘**（点到直线的正交距离平方和最小），也就是 2x2 协方差矩阵的
    主特征向量。直线上点的 x/y 都是带误差的，普通"y 对 x 回归"会把误差全算到 y 上，
    边一斜结果就偏。

    Verilog：五个累加器（Σx, Σy, Σx², Σxy, Σy²）+ 一个 2x2 特征向量。
    特征方向 ``θ = ½·atan2(2Σxy, Σx²−Σy²)`` 需要一次 atan2（上板用 CORDIC）；
    嫌贵可以退回"只平移"的旧做法 —— 精度差几个百分点，但全是加减乘。
    """
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)     # 主方向
    a, b = -math.sin(theta), math.cos(theta)           # 法线
    return (a, b, -(a * mx + b * my))


# --------------------------------------------------------------------------
# 圆：沿多个方向量半径，同时把圆心也修正
# --------------------------------------------------------------------------
def refine_circle(frame, cx: float, cy: float, r: float, *,
                  score: Scorer = bright_score, directions: int = 16,
                  span_frac: float = 0.6, levels: tuple[float, float] | None = None,
                  ) -> tuple[float, float, float]:
    """把圆的 ``(cx, cy, r)`` 精修到亚像素。

    沿 ``directions`` 条射线往外找半高穿越点；**新的圆心取这些边缘点的均值**，
    半径取各点到新圆心的距离均值。这比"用外接框估半径"准得多：
    外接框是整数、而且受单个离群像素支配。
    """
    if r <= 0.0 or directions < 4:
        return cx, cy, r
    start = r * (1.0 - span_frac)
    steps = max(3, int(r * (2.0 * span_frac)) + 1)

    # 先把所有方向的剖面取出来，再**统一**估一次 hi/lo。
    # 逐条射线各自估会被"朝向刻度"带偏：平台在圆里画了一条深色短线，
    # 沿刻度的那条射线头部是暗的，阈值被拉低，穿越点就跑到圆外去了。
    # 统一估还把电平变成两个**寄存器**，正好是硬件上该有的样子。
    profiles: list[tuple[float, float, list[float]]] = []
    for k in range(directions):
        th = 2.0 * math.pi * k / directions
        dx, dy = math.cos(th), math.sin(th)
        profiles.append((dx, dy, _sample_along(
            frame, cx + dx * start, cy + dy * start, dx, dy, steps, score)))
    if levels is None:
        edge_n = max(1, steps // 4)
        head = sorted(v for _, _, p in profiles for v in p[:edge_n])
        tail = sorted(v for _, _, p in profiles for v in p[-edge_n:])
        levels = (head[len(head) // 2], tail[len(tail) // 2])

    hits: list[Point] = []
    for dx, dy, prof in profiles:
        idx = half_level_edge(prof, hi=levels[0], lo=levels[1])
        if idx is None:
            continue
        d = start + idx
        hits.append((cx + dx * d, cy + dy * d))
    if len(hits) < directions // 2:
        return cx, cy, r                    # 边缘被挡住太多，宁可用原值
    ncx, ncy = _centroid(hits)
    radii = sorted(math.hypot(p[0] - ncx, p[1] - ncy) for p in hits)
    # 取中位而不是均值：个别方向被墙/高光打断时中位不受影响
    return ncx, ncy, radii[len(radii) // 2]


__all__ = [
    "Scorer", "bright_score", "magenta_score",
    "half_level_edge", "refine_quad", "refine_circle",
]
