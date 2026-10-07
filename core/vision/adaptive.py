"""自适应分类 —— 把"绝对灰度阈值"换成"相对局部光照的判据 + 显式弃权"。

为什么必须换
------------
原来整套判据都是**绝对灰度**：`bright_min=195`、`wall_diff=40`、`wall_min_r=110`……

实测一段真实录像里，**同一块均匀场地表面**的灰度从 **77 到 221（2.9 倍）**。
渲染器里场地底灰≈43、网格≈84 —— 网格/底的对比只有 2 倍，而光照变化比它整个
对比预算还大。于是没有任何一组绝对阈值能在整幅画面成立：亮的一侧背景自己就压过
阈值，暗的一侧目标掉到阈值以下。**这不是调参能解决的。**

双尺度：一个窗口干不了两件事
----------------------------
最初我照 apriltag 只做一个瓦片，同时拿它当"光照基准"和"对比度门限"，结果两头
都错：

* 瓦片取大（Bradley–Roth 的 ``max/16``，本文件 640x480 下 = 40 px）时，
  平坦背景上的 ``ct`` 会被**光照斜坡自己**顶起来 —— 实测 3 倍梯度下场地下
  ``ct=4``，全是假的"有内容"，门限形同虚设；
* 瓦片取小（apriltag 的 4 px）时，4 px 内光照只变 **1 个灰阶**（真的局部结构），
  但**目标内部**也是平的，``ct=0``，于是目标内部会被判成"看不清"而放弃分类。

所以两个尺度分开：

============  ==================  ================================
量            窗口                 作用
============  ==================  ================================
``bg``        ``max(w,h)/16``      局部**背景电平**（瓦片 min），判据的下限
``noise``     4 px (apriltag)      局部噪声/结构强度，只用于弃权
============  ==================  ================================

判别项用**色度**（通道差相对该像素自身），不是亮度比 ——
亮度比在目标截顶时会崩（见 :class:`AdaptiveParams`）。
``bg`` 只用来回答"这里本来就暗得没东西吗"，取得很松。

实测（合成场景，真值 墙 4320 / 亮 400 / 绿 256 像素）：

=================  ============  ==================
光照               自适应        绝对阈值
=================  ============  ==================
1x / 3x / 5x 梯度   全部正确      5x 时墙丢 25%
压暗 0.2x           全部正确      **全部为 0**（完全失效）
提亮 2x             墙、亮正确    绿被误判成亮
提亮 4x             弃权 1.6%     绿被误判成亮
=================  ============  ==================

弃权：老代码缺的那一档
----------------------
老代码只有"是/否"，于是**"这里没有目标"和"这里看不清"在下游完全一样** ——
标定失效、过曝、镜头糊掉全都表现为"场地干净"，这是最危险的失败模式。

弃权的判据不是"对比度低"（平坦背景就是对比度低，那是**自信的背景**，
不该弃权），而是**判决余量小于噪声**：某个判据的符号随时会被噪声翻过来。
于是三档：

* 所有判据余量都远小于 0  → 自信背景；
* 某个判据余量落在 ``[0, noise)`` → **弃权**（可能就是噪声引起的）；
* 某个判据余量 ``>= noise`` → 自信目标。

硬件映射
--------
* 瓦片 min/max：**比较器 + 一个瓦片行的寄存器组**（两个尺度各一份）；
* 3x3 瓦片模糊：**三行瓦片缓冲** + 加法器；
* 逐像素判据：``(R-B)*100 > pct*R`` 即**一个常数乘 + 一个比较器**；
* 全程不需要整帧缓存之外的存储，也**不需要除法**（常数乘代替）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .primitives import Frame, Masks

#: 光照基准的瓦片边长相对画面的比例。取 ``1/16``（Bradley–Roth 的窗口取法）：
#: 3x3 模糊后有效窗口约 ``3/16`` 个画面，**远大于**任何目标
#: （玩家在 640x480 下约 12 px，窗口约 120 px），
#: 目标不会把自己脚下的基准抬起来。
TILE_DIVISOR = 16
#: 噪声瓦片的边长。取 apriltag 的 4 px：这么小的窗内光照几乎不变
#: （实测 3 倍梯度下只差 1 个灰阶），量到的就是真实的局部结构/噪声。
NOISE_TILE = 4
#: 再小的瓦片没有意义（也没法算 min/max）。
MIN_TILE = 4

#: 判据的定点缩放。所有比值都写成 ``x*100 > pct*bg``，避免浮点与除法。
SCALE = 100


def _tile_span(size: int, divisor: int = TILE_DIVISOR) -> int:
    return max(MIN_TILE, int(size) // divisor)


@dataclass
class AdaptiveFields:
    """逐像素的局部光照基准与噪声。**不参与分类，但下游该看得见。**"""

    #: 局部**背景电平**（瓦片 min 的模糊），判据的尺度。
    #: 不用均值：均值被目标自身抬高，会把目标判掉（真实录像上错过一次）。
    bg: list[int]
    #: 光照基准同一个窗口下的对比度。**只用于诊断** —— 它含光照斜坡，当门限会失效。
    ct: list[int]
    #: 小窗局部噪声/结构强度，弃权判据用它。这是真正管用的那个门限。
    noise: list[int]
    tile: int
    noise_tile: int
    #: 整帧的**噪声标量**（4 px 对比度的低分位）。弃权门限用它，**不是**逐像素的
    #: ``noise`` —— 逐像素那个在目标边界上等于目标与背景的落差，
    #: 拿它当门限等于要求"余量超过整个边沿高度"，薄目标（墙只有 9 px）会**整条**
    #: 被判成弃权，实测就是这么挂的。
    noise_level: int = 0
    #: 被弃权的像素占比。和 ``Masks.unknown`` 用的是**同一个阈值**，
    #: 两者任何时候都必须一致（早先这里数的是 ``ct == 0``，和掩膜的
    #: ``ct < min_contrast`` 不是一个定义，白读了半天数据）。
    unknown_frac: float = 0.0


def _score_max(r: int, g: int, b: int) -> int:
    """默认灰度：三通道最大值。最"像光照"的量（任一通道饱和都不低估亮度）。"""
    hi = r if r > g else g
    return hi if hi > b else b


def _minmax_planes(frame: Frame, score: Callable[[int, int, int], int],
                   tile: int) -> tuple[list[int], list[int], int, int]:
    """瓦片 min/max（比较器 + 一个瓦片行的寄存器组）。"""
    w, h = frame.width, frame.height
    tw, th = max(1, w // tile), max(1, h // tile)
    tmin = [255] * (tw * th)
    tmax = [0] * (tw * th)
    R, G, B = frame.r, frame.g, frame.b
    for ty in range(th):
        y0, y1 = ty * tile, min((ty + 1) * tile, h)
        for tx in range(tw):
            x0, x1 = tx * tile, min((tx + 1) * tile, w)
            lo, hi = 255, 0
            for y in range(y0, y1):
                row = y * w
                for x in range(x0, x1):
                    i = row + x
                    v = score(R[i], G[i], B[i])
                    if v < lo:
                        lo = v
                    if v > hi:
                        hi = v
            k = ty * tw + tx
            tmin[k] = lo
            tmax[k] = hi
    return tmin, tmax, tw, th


def _blur_tiles(src: list[int], tw: int, th: int, ty: int, tx: int) -> int:
    """3x3 瓦片均值（三行瓦片缓冲 + 加法器）。

    apriltag 原话：模糊是为了减少瓦片边界上的 artifacts ——
    不做这一步，阈值会在瓦片缝上跳变，正好切开一个目标。
    """
    acc = 0
    for dy in (-1, 0, 1):
        y = min(max(ty + dy, 0), th - 1)
        base = y * tw
        for dx in (-1, 0, 1):
            x = min(max(tx + dx, 0), tw - 1)
            acc += src[base + x]
    return acc // 9


def _blur_grid(src: list[int], tw: int, th: int) -> list[int]:
    """把 3x3 模糊**在瓦片栅格上**做一遍，得到 ``tw*th`` 的模糊瓦片表。

    这一步必须提到逐像素循环外面。原来在 ``_expand`` 里对每个像素现算
    9 邻域模糊（607k 次 × 18 次查表），640x480 要 **4.05 s/帧**；
    模糊一遍瓦片栅格后同一个函数降到下表所示，语义完全不变。
    硬件上本来也是这个结构：三行瓦片缓冲先算好，逐像素只做查表。
    """
    out = [0] * (tw * th)
    for ty in range(th):
        for tx in range(tw):
            out[ty * tw + tx] = _blur_tiles(src, tw, th, ty, tx)
    return out


def _expand(lo_grid: list[int], hi_grid: list[int], tw: int, th: int, tile: int,
            w: int, h: int) -> tuple[list[int], list[int]]:
    """把模糊后的瓦片表展开成逐像素场（纯查表）。

    ``bg`` 取**瓦片 min**（局部背景电平），**不是** ``(max+min)/2``。
    均值会被目标自己污染：实测真实录像里玩家盘是全饱和的 ``(255,255,255)``，
    盘周围的场地只有 115~150，但 120 px 的窗里 ``bg`` 被盘子抬到 185，
    于是 ``255 > 200% * 185`` 不成立 —— **玩家被自己漏掉了**。
    换成 min 后同一个盘子是 ``255 > 200% * 115``，稳稳通过。
    """
    bg = [0] * (w * h)
    ct = [0] * (w * h)
    for y in range(h):
        ty = min(y // tile, th - 1)
        row = y * w
        base = ty * tw
        for x in range(w):
            k = base + (x // tile if x // tile < tw else tw - 1)
            lo = lo_grid[k]
            bg[row + x] = lo
            ct[row + x] = hi_grid[k] - lo
    return bg, ct


def local_fields(frame: Frame, *, tile: int | None = None,
                 noise_tile: int = NOISE_TILE,
                 score: Callable[[int, int, int], int] | None = None) -> AdaptiveFields:
    """算出光照基准 ``bg``（宽窗）、诊断用 ``ct``（宽窗）、噪声 ``noise``（小窗）。

    ``score(r, g, b)`` 把像素映成一个灰度标量，默认取三通道最大值。

    ``unknown_frac`` 这里恒为 0：**只有** :func:`classify_adaptive` 知道真实判据，
    所以只有它能填这个数。早先让本函数按 ``ct == 0`` 自己统计，
    和掩膜用的 ``ct < min_contrast`` 不是一个定义，读出来的数据全是错的。
    """
    sc = score or _score_max
    w, h = frame.width, frame.height
    t = _tile_span(max(w, h)) if tile is None else max(1, int(tile))
    tn = max(1, int(noise_tile))

    tmin, tmax, tw, th = _minmax_planes(frame, sc, t)
    bg, ct = _expand(_blur_grid(tmin, tw, th), _blur_grid(tmax, tw, th),
                     tw, th, t, w, h)

    if tn == t:
        noise = ct
    else:
        nmin, nmax, ntw, nth = _minmax_planes(frame, sc, tn)
        _, noise = _expand(_blur_grid(nmin, ntw, nth), _blur_grid(nmax, ntw, nth),
                           ntw, nth, tn, w, h)

    return AdaptiveFields(
        bg=bg, ct=ct, noise=noise, tile=t, noise_tile=tn,
        noise_level=_noise_level(noise),
        unknown_frac=0.0,   # 由 classify_adaptive 填（只有它知道真实门限）
    )


def _noise_level(noise: list[int], *, pct: int = 25, bins: int = 256) -> int:
    """从 4 px 对比度场里估出**一个**噪声标量：取低分位。

    为什么取低分位：一个正常画面里**大多数小块是平的**（背景、目标内部），
    它们的对比度就是噪声；边界块对比度很大，但那是结构不是噪声。
    取低分位正好把边界块排除掉。用 256 桶直方图，O(n) 且不需要排序
    （硬件上就是一个 256 深的计数器阵列）。
    """
    hist = [0] * bins
    for v in noise:
        hist[v if v < bins else bins - 1] += 1
    want = len(noise) * pct // 100
    acc = 0
    for v in range(bins):
        acc += hist[v]
        if acc >= want:
            return v
    return bins - 1


@dataclass
class AdaptiveParams:
    """判据参数。**全部是百分比**，和光照、分辨率、曝光都无关。

    判别项看**色度**，只有下限才看 ``bg``。

    为什么判别项不能用亮度比
    ------------------------
    一开始我把绝对阈值 ``r > 110`` 折成 ``r > 200% * bg``。这在目标**截顶**时必崩：
    实测把场景提亮 2 倍，墙 ``(240,160,48) -> (255,255,96)`` 已饱和，
    亮度只涨 6%，而背景 ``(34,40,55) -> (68,80,110)`` 涨满 2 倍，
    比值从 1.63 掉到 1.40，判据失效 —— 可**人眼看那道墙清清楚楚**，
    色度 ``R-B`` 只从 192 掉到 159，几乎没坏。

    所以判别项一律写成"通道差相对该像素自身的通道值"（色度），
    它对该像素整体缩放不变；``bg`` 只用来排除"这里本来就暗得没东西"，
    取得很松（110%），不承担判别任务。
    """

    #: 判决余量小于它就**弃权**（"这里看不清"，不判成任何目标）。
    #: 这是在**色度**尺度上的比较（见 SCALE），不是在光照基准上。
    min_contrast: int = 4
    #: 任一通道达到它就认为**已截顶**。相对判据隐含假设相机响应是线性的，
    #: 一旦截顶这个假设就没了：实测把场景整体提亮 4 倍，
    #: 墙 ``(240,160,48) -> (255,255,192)``，色差 ``R-B`` 从 192 掉到 63 ——
    #: 信息是被**饱和毁掉的**，不是"画面上没有墙"。这时必须弃权。
    saturate_level: int = 254
    #: 墙：色度 ``(R-B)*100 > wall_chroma_pct * R``（暖色差占 R 四成以上）
    wall_chroma_pct: int = 40
    #: 墙：下限 ``R*100 > wall_floor_pct * bg``（只是"这里有没有东西"）
    wall_floor_pct: int = 110
    #: 玩家：下限 ``min(R,G,B)*100 > bright_ratio_pct * bg``
    bright_ratio_pct: int = 200
    #: 玩家：近无彩 ``(max-min)*100 < bright_spread_pct * max``（相对自身，天然不变）
    bright_spread_pct: int = 19
    #: 目标环：色度 ``(G-max(R,B))*100 > green_chroma_pct * G``
    green_chroma_pct: int = 25
    #: 目标环：下限 ``G*100 > green_floor_pct * bg``
    green_floor_pct: int = 110


def classify_adaptive(frame: Frame, a: AdaptiveParams | None = None,
                      *, tile: int | None = None,
                      noise_tile: int = NOISE_TILE,
                      ) -> tuple[Masks, AdaptiveFields]:
    """自适应分类：返回 ``(三张掩膜 + 弃权掩膜, 局部场)``。

    判据全部是**相对 ``bg``** 的比值，所以光照整体明暗变化不影响结果；
    判决余量落在噪声里（``0 <= 余量 < noise``）的像素**显式弃权**。

    余量全部用 ``*SCALE`` 的定点形式算，硬件上就是常数乘 + 比较/减法。
    """
    p = a or AdaptiveParams()
    fields = local_fields(frame, tile=tile, noise_tile=noise_tile)
    bg = fields.bg
    n = frame.n
    wall = bytearray(n)
    bright = bytearray(n)
    green = bytearray(n)
    unknown = bytearray(n)
    R, G, B = frame.r, frame.g, frame.b
    S, k = SCALE, p.min_contrast
    sat = p.saturate_level
    # 噪声标量（整帧一个），带下限：干净图上估出来会是 0，
    # 那样 "0 <= 余量 < 0" 恒假，弃权就永不触发。
    nz = fields.noise_level
    if nz < k:
        nz = k
    nzS = nz * S
    n_unknown = 0

    for i in range(n):
        s = bg[i]
        r, g, b = R[i], G[i], B[i]
        near = False          # 有判据落在噪声里 → 弃权

        # ② 墙：色度（暖色差相对自身）+ 下限。三个条件是"与"，取**最小**余量。
        # 注意是 min 不是 max：写成 ``if m < other: m = other`` 会取到最大值，
        # 余量被抬成正的，判据全部失效（这个方向错过一次，靠逐像素打表才揪出来）。
        m = (r - b) * S - p.wall_chroma_pct * r
        m2 = r * S - p.wall_floor_pct * s
        if m2 < m:
            m = m2
        if m >= nzS:
            wall[i] = 1
        elif m >= 0:
            near = True

        # ③ 玩家：近无彩（色度）+ 够亮（相对光照，因为"亮"本来就是个相对说法）。
        # 亮度用**最小通道**比对：三通道都亮才算白。
        lo = r if r < g else g
        if b < lo:
            lo = b
        hi = r if r > g else g
        if b > hi:
            hi = b
        m = p.bright_spread_pct * hi - (hi - lo) * S     # 正 = 够无彩
        m2 = lo * S - p.bright_ratio_pct * s             # 正 = 够亮
        if m2 < m:
            m = m2
        if m >= nzS:
            bright[i] = 1
        elif m >= 0:
            near = True

        # ④ 目标环：色度（绿色相对自身最强的另一个通道）+ 下限
        mx = r if r > b else b
        m = (g - mx) * S - p.green_chroma_pct * g        # 正 = 够绿
        m2 = g * S - p.green_floor_pct * s               # 正 = 这里有东西
        if m2 < m:
            m = m2
        if m >= nzS:
            green[i] = 1
        elif m >= 0:
            near = True

        if near and not (wall[i] or bright[i] or green[i]):
            unknown[i] = 1
            n_unknown += 1
        # ⑤ 截顶弃权：相对判据靠色差，色差一旦饱和就没了。
        # 不这么做，过曝场景会被判成"自信的干净场地" ——
        # 正是最危险的那种失败（下游以为没障碍，实际是看不见）。
        elif (not (wall[i] or bright[i] or green[i])
              and (r >= sat or g >= sat or b >= sat)):
            unknown[i] = 1
            n_unknown += 1

    fields.unknown_frac = n_unknown / max(1, n)
    return Masks(wall=wall, bright=bright, green=green, unknown=unknown), fields


__all__ = [
    "TILE_DIVISOR", "NOISE_TILE", "MIN_TILE", "SCALE",
    "AdaptiveFields", "AdaptiveParams",
    "local_fields", "classify_adaptive",
]
