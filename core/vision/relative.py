"""**帧内相对判据**：把"绝对阈值 + 调参"换成"与增益/偏置无关的量"。

## 问题（实测，不是推演）

原来的分类是三条**绝对阈值**：

```c
wall   : (r - b) > 40  && r > 110 && g > 60      // 偏红 + 够亮
bright : min(r,g,b) > 195 && (max-min) < 45      // 近白 + 不花
```

它们在"数字原图"上余量巨大（屏幕橙墙 `r-b = 192`、白角色 `min = 235`、黑底 `min = 34`），
但相机拍屏幕时：自动曝光把黑底抬到 **114~180**、把橙墙削顶成近中性（`r-b` 只剩
**20~57**）、白平衡还会旋转色相。于是三个量全都落在阈值边界上 —— 实测
`brightness=70` 有墙、`75` 就一帧都没有（**5 个单位就翻面**）。靠扫参数去救这段，
就是在追相机的变化，**换台相机 / 换个光照就重新失效**。

## 换成什么

**归一化色度 + 帧内分位数**，两者都不含"对这个相机才成立"的常数：

| 量 | 定义 | 为什么稳 |
|---|---|---|
| 暖色度 | `warm = (r - b) / (r + g + b + eps)` | 整体增益 / 偏置**同时**作用在分子分母 → 除掉；相机去饱和是**乘性**的色度增益，墙与背景的 `warm` **比值**被保留（实测 ~6 倍） |
| 亮度 | `L = max(r,g,b)`，阈值取**本帧 L 的分位数** | 曝光变了，分位数跟着变 → 阈值自适应；不需要"195"这种数字 |
| 近中性 | `(L - min) <= neutral * L` | 相对量；白 = 通道几乎相等 |

`eps` 只是**数值安全**（避免除零），不是判据；`warm` / `neutral` 的取值来自
**颜色契约 + 实测分离度**（见 `DEFAULTS` 的注释），不是"调到好看为止"。

## 不变量（有测试钉住）

* **增益不变**：`frame * k` → 三张掩膜**逐位相同**；
* **偏置不变**（未削顶时）：`frame + c` → 掩膜相同；
* 削顶（饱和）后 `warm` 会退化 → 这类像素进 `unknown` 掩膜（"看不清，不判"），
  而不是被硬判成墙或背景。

这三条正是"不需要调参才能好看"的可验证版本：**换相机、改曝光、改白平衡，判据不变**。
"""

from __future__ import annotations

from dataclasses import dataclass

from . import primitives as P

__all__ = ["RelativeParams", "classify_relative", "DEFAULTS"]

#: 帧内亮度分位数的默认值：只取最亮的一小撮（角色是唯一的"亮紧凑块"）。
_DEFAULT_BRIGHT_Q = 0.995
#: "近中性"的默认容差（相对 L）：白圆的通道几乎相等，橙墙/绿物会明显偏。
_DEFAULT_NEUTRAL = 0.35


@dataclass(frozen=True)
class RelativeParams:
    """相对判据的参数：全部是**无量纲的统计量**，没有"对这个相机才成立"的常数。

    实测教训（真板子、相机亮度 40→130 扫了一遍）：只把色度**除法归一化**
    （`(r-b)/(r+g+b)`）**并不够** —— 曝光低时图像接近黑电平，分母很小，
    背景的色度被偏置顶到 0.69，墙和背景的分布直接重叠。也就是说"除法归一化"
    去掉了**增益**，但去不掉**偏置**。

    所以这里改用**稳健 z 分数**（中位数 + MAD）：

    ```text
    d  = r - b                          // 暖色差：等量偏置直接抵消
    med, mad = 中位数(d), 中位数(|d - med|)     // 用本帧自己的背景统计
    z  = (d - med) / (1.4826 * mad + eps)       // 无量纲：增益与偏置都不影响
    ```

    `z` 的含义是"比本帧的背景暖多少个稳健标准差" —— 这个含义换相机、换曝光、
    换白平衡都不变，所以阈值 `k_warm` 不需要重调；`mad` 估计的是本帧背景的
    颜色抖动（噪声 + 光照不均），画面越脏阈值自动越松。
    """

    #: 墙：暖色差要比本帧背景暖 `k_warm` 个稳健标准差。
    #: 这个是**统计显著性**（"明显比背景暖"），不是亮度/色度的绝对值。
    k_warm: float = 4.0
    #: 亮块（角色）：亮度要比本帧背景亮 `k_bright` 个稳健标准差，
    #: 且近中性（相对 L 的通道差 —— 只有"白"才符合）。
    k_bright: float = 5.0
    #: 近中性容差（相对 L）：白圆的通道几乎相等。
    neutral: float = 0.35
    #: 数值安全项（**不是**判据）：避免 MAD 为 0 时除零。
    eps: float = 1e-3
    #: 绿：绿-最大其他通道 的 z 分数阈值（平台上没有绿物，留作扩展）。
    k_green: float = 4.0
    #: 稳健标准差的换算系数（正态分布下 MAD → sigma）。
    mad_to_sigma: float = 1.4826
    #: **行内平滑窗口**（像素）：墙是**沿着长度方向均匀**的长条结构，噪声是逐像素的，
    #: 所以在 x 方向做一次方框平均，信噪比按 sqrt(窗口) 提升（窗口 16 → 4 倍）。
    #:
    #: 这不是"调参"：它用的是**几何先验**（墙 = 长条 + 均匀），窗口大小只取决于
    #: "墙比噪声的相关长度长多少"，与相机的曝光/白平衡无关；玩家是**紧凑**目标，
    #: 所以这一平滑**只作用在墙/绿通道上**，绝不作用在亮块（角色）上。
    row_avg: int = 16


DEFAULTS = RelativeParams()


def _median(values) -> float:
    xs = sorted(values)
    n = len(xs)
    if n == 0:
        return 0.0
    mid = n // 2
    return float(xs[mid]) if n % 2 else 0.5 * (float(xs[mid - 1]) + float(xs[mid]))


def _mad(values, med: float) -> float:
    return _median([abs(v - med) for v in values])


def _quantile(values, q: float) -> float:
    """线性插值分位数（不依赖 numpy，语义与 numpy.percentile 的线性插值一致）。"""
    if not values:
        return 0.0
    xs = sorted(values)
    if q <= 0.0:
        return float(xs[0])
    if q >= 1.0:
        return float(xs[-1])
    pos = q * (len(xs) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(xs) - 1)
    frac = pos - lo
    return float(xs[lo]) * (1.0 - frac) + float(xs[hi]) * frac


def classify_relative(frame: P.Frame, p: RelativeParams | None = None) -> P.Masks:
    """一帧 → 墙 / 亮 / 绿 三张掩膜（**稳健 z 分数**，无相机相关常数）。

    见 :class:`RelativeParams` 的说明：判据是"比本帧背景暖/亮多少个稳健标准差"，
    因此对**增益（曝光）、偏置（黑电平）、白平衡**都不敏感 —— 不需要按相机调参。
    """
    prm = p or DEFAULTS
    n = frame.n
    r_b, g_b, b_b = frame.r, frame.g, frame.b

    d = [0] * n          # 暖色差 r-b
    gr = [0] * n         # 绿差 g-max(r,b)
    lum = [0] * n
    for i in range(n):
        r, g, b = r_b[i], g_b[i], b_b[i]
        d[i] = r - b
        gr[i] = g - (r if r > b else b)
        lum[i] = max(r, g, b)

    # ---- 行内方框平均：墙/绿的"长条先验"（噪声按 sqrt(N) 降，信号不动）----
    w_ = frame.width
    h_ = frame.height
    win = max(1, int(prm.row_avg))
    if win > 1 and w_ >= win and n:
        half = win // 2
        d_s = [0.0] * n
        gr_s = [0.0] * n
        for y in range(h_):
            base = y * w_
            # 前缀和：O(1) 取滑动窗，硬件上就是一个"行缓存 + 累加器"
            pre_d = [0] * (w_ + 1)
            pre_g = [0] * (w_ + 1)
            for x in range(w_):
                pre_d[x + 1] = pre_d[x] + d[base + x]
                pre_g[x + 1] = pre_g[x] + gr[base + x]
            for x in range(w_):
                a = x - half
                b2 = x + half + 1
                a = 0 if a < 0 else a
                b2 = w_ if b2 > w_ else b2
                cnt = b2 - a
                d_s[base + x] = (pre_d[b2] - pre_d[a]) / cnt
                gr_s[base + x] = (pre_g[b2] - pre_g[a]) / cnt
        d_use, gr_use = d_s, gr_s
    else:
        d_use, gr_use = d, gr

    tot = [r_b[i] + g_b[i] + b_b[i] for i in range(n)] if n else []
    med_tot = _median(tot) if tot else 0.0
    dark_thr = 0.15 * med_tot          # 相对量：远暗于本帧中位亮度 → 色度不可信
    med_d, mad_d = _median(d_use), _mad(d_use, _median(d_use))
    med_l, mad_l = _median(lum), _mad(lum, _median(lum))
    med_g, mad_g = _median(gr_use), _mad(gr_use, _median(gr_use))
    sd_d = prm.mad_to_sigma * mad_d + prm.eps
    sd_l = prm.mad_to_sigma * mad_l + prm.eps
    sd_g = prm.mad_to_sigma * mad_g + prm.eps

    wall = bytearray(n)
    bright = bytearray(n)
    green = bytearray(n)
    unknown = bytearray(n)
    for i in range(n):
        r, g, b = r_b[i], g_b[i], b_b[i]
        z_d = (d_use[i] - med_d) / sd_d
        z_l = (lum[i] - med_l) / sd_l
        z_g = (gr_use[i] - med_g) / sd_g
        # 符号谓词：暖 = 红比蓝多。等量偏置直接抵消（(r+c)-(b+c) = r-b），
        # 所以它是**不变量**；而"白/中性"（r≈b）永远不满足 —— 光靠"比背景暖"
        # 会把**白色角色**也判成墙（实测：背景 d=-21、角色 d=-15、墙 d=+192）。
        if d[i] > 0 and z_d > prm.k_warm:
            wall[i] = 1
        if z_l > prm.k_bright and (lum[i] - min(r, g, b)) <= prm.neutral * max(1.0, lum[i]):
            bright[i] = 1
        if gr[i] > 0 and z_g > prm.k_green:
            green[i] = 1
    for i in range(n):
        if tot[i] <= dark_thr:
            unknown[i] = 1
    return P.Masks(wall, bright, green, unknown)


# ==========================================================================
# 第二级证据：**行间台阶**（局部差分），不依赖颜色的绝对/相对阈值
# ==========================================================================
def row_profile(frame: P.Frame, *, window: int = 16) -> list:
    """每行的"暖色证据"：`d = r - b` 先在行内做方框平均（噪声 /sqrt(N)），
    再按 y 做一次 3 行平滑（抑制单行噪声）。

    **为什么是差分而不是阈值**：真实场景里屏幕的亮度/色温在 y 方向是**缓慢变化**的，
    而墙是"在某一行突然开始、突然结束"的**台阶**。差分把缓慢变化直接消掉，
    留下台阶 —— 这一步与曝光、白平衡、色度强弱都无关。
    """
    w, h = frame.width, frame.height
    if w <= 0 or h <= 0:
        return []
    d_row = []
    for y in range(h):
        base = y * w
        s = 0
        for x in range(w):
            s += frame.r[base + x] - frame.b[base + x]
        d_row.append(s / float(w))
    # y 方向 3 行平滑
    out = [0.0] * h
    for y in range(h):
        a = max(0, y - 1)
        b = min(h - 1, y + 1)
        out[y] = sum(d_row[a:b + 1]) / float(b - a + 1)
    _ = window
    return out


def row_steps(profile: list, *, half: int = 3) -> list:
    """行间台阶：`step[y] = ev[y+half] - ev[y-half]`（局部差分，缓慢光照被消掉）。"""
    n = len(profile)
    if n == 0:
        return []
    out = [0.0] * n
    for y in range(n):
        a = max(0, y - half)
        b = min(n - 1, y + half)
        out[y] = profile[b] - profile[a]
    return out


def wall_bands_from_step(frame: P.Frame, *, half: int = 3, k_step: float = 4.0,
                         min_rows: int = 4) -> tuple:
    """用**行间台阶**找出"墙带"（`[(y0, y1), ...]`），返回 `(bands, steps)`。

    判据全是**统计量**：`|step|` 超过 `k_step` 个稳健标准差（MAD）的行算"边"，
    两条边之间就是一条带。没有"墙应该多亮/多暖"的绝对值 —— 这正是为了跨曝光稳定。

    返回的 `steps` 供调用方做诊断（画图 / 打日志）。
    """
    prof = row_profile(frame)
    steps = row_steps(prof, half=half)
    if not steps:
        return ([], steps)
    med = _median(steps)
    mad = _mad(steps, med)
    sd = DEFAULTS.mad_to_sigma * mad + DEFAULTS.eps
    edges = [y for y, s in enumerate(steps) if abs(s - med) > k_step * sd]
    bands: list[tuple[int, int]] = []
    for a, b in zip(edges, edges[1:]):
        if b - a + 1 >= min_rows:
            bands.append((a, b))
    return (bands, steps)


def best_wall_band(frame: P.Frame, *, min_rows: int = 6, max_rows: int = 40,
                   pad: int = 6, k_sigma: float = 5.0) -> dict:
    """**匹配滤波**找横墙：在所有候选行带里挑"带内比带外暖得最多"的那条。

    为什么这样最稳：墙是"两条平行长边 + 均匀内部"的结构，噪声是逐像素、逐行独立的。
    于是把**整个带作为一个整体**去比对（带内均值 - 带外均值），信噪比按
    `sqrt(带宽 * 行长)` 提升；而"挑最大"这一步是**结构匹配**，不需要知道墙有多暖。

    返回 `{found, y0, y1, snr, contrast, sigma}`：`snr` 是"带内比带外暖几个稳健标准差"，
    与曝光/白平衡无关；`k_sigma` 是判"有墙"的统计门槛（对**搜索后的最大值**取 5σ）。
    """
    prof = row_profile(frame)
    h = len(prof)
    out = {"found": False, "y0": 0, "y1": 0, "snr": 0.0, "contrast": 0.0, "sigma": 0.0}
    if h < min_rows + 2 * pad + 2:
        return out
    # 噪声尺度：行剖面的**一阶差分**的 MAD（去掉缓慢趋势后的抖动）
    diffs = [prof[i + 1] - prof[i] for i in range(h - 1)]
    med_d = _median(diffs)
    sigma = DEFAULTS.mad_to_sigma * _mad(diffs, med_d) + DEFAULTS.eps
    best = (-1e18, 0, 0, 0.0)
    for y0 in range(pad, h - min_rows - pad):
        for y1 in range(y0 + min_rows, min(h - pad, y0 + max_rows + 1)):
            inside = sum(prof[y0:y1 + 1]) / float(y1 - y0 + 1)
            before = sum(prof[y0 - pad:y0]) / float(pad)
            after = sum(prof[y1 + 1:y1 + 1 + pad]) / float(pad)
            contrast = inside - 0.5 * (before + after)     # 带内比带外暖多少
            if contrast > best[0]:
                best = (contrast, y0, y1, contrast / sigma)
    contrast, y0, y1, snr = best
    out.update(found=snr >= k_sigma, y0=y0, y1=y1, snr=snr,
               contrast=contrast, sigma=sigma)
    return out


def band_segments(frame: P.Frame, y0: int, y1: int, *,
                  k_col: float = 4.0, min_seg_px: int = 8) -> dict:
    """在已找到的**行带**里，用**列剖面**找墙段与缺口（同样是差分/匹配，不是颜色阈值）。

    带内每列的 `d = r-b` 在 y 方向平均（噪声 /sqrt(带高)），再减去**带外**同一列的
    基线（pad 行）—— 得到"这一列比背景暖多少"的 1 维剖面；然后：

    * `> k_col` 个稳健标准差的列 = 墙列（游程 → 墙段，丢太短的）；
    * 两段之间最宽的间隔 = **缺口**（宽度即缺口像素数）。

    全是 1 维、平均过、差分过的量 → 跨曝光稳定；没有任何"墙该多暖"的绝对值。
    """
    w, h = frame.width, frame.height
    out = {"segments": [], "gap": None, "gap_width": 0.0, "profile": []}
    if w <= 0 or y1 < y0 or y0 < 0 or y1 >= h:
        return out
    pad = 6
    ya = max(0, y0 - pad)
    yb = min(h - 1, y1 + pad)
    prof = []
    for x in range(w):
        inside = 0
        for y in range(y0, y1 + 1):
            i = y * w + x
            inside += frame.r[i] - frame.b[i]
        inside /= float(y1 - y0 + 1)
        outside = 0
        n_out = 0
        for y in range(ya, y0):
            i = y * w + x
            outside += frame.r[i] - frame.b[i]
            n_out += 1
        for y in range(y1 + 1, yb + 1):
            i = y * w + x
            outside += frame.r[i] - frame.b[i]
            n_out += 1
        if n_out:
            outside /= float(n_out)
        prof.append(inside - outside)
    out["profile"] = prof
    # ⚠ 噪声尺度要用**相邻列的差分**来估：列剖面本身带着场景的大尺度结构
    #   （左边界 -13、墙 30+），用它的 MAD 会把"结构"当成"噪声"，门槛高到永远
    #   过不去（实测：中位 8.2、σ 12.0、4σ 门槛 56 —— 而最大值才 33）。
    #   差分把缓慢的结构消掉，只留下抖动 —— 和 `best_wall_band` 里同一套做法。
    diffs = [prof[i + 1] - prof[i] for i in range(len(prof) - 1)]
    dmed = _median(diffs)
    sd = DEFAULTS.mad_to_sigma * _mad(diffs, dmed) + DEFAULTS.eps
    med = _median(prof)
    flags = [1 if (v - med) > k_col * sd else 0 for v in prof]
    # 游程 → 墙段（丢太短的）
    segs = []
    start = None
    for x, f in enumerate(flags):
        if f and start is None:
            start = x
        elif not f and start is not None:
            if x - start >= min_seg_px:
                segs.append((start, x - 1))
            start = None
    if start is not None and w - start >= min_seg_px:
        segs.append((start, w - 1))
    out["segments"] = segs
    # 缺口：相邻两段之间最宽的间隔
    best = (0, None)
    for a, b in zip(segs, segs[1:]):
        gap = b[0] - a[1] - 1
        if gap > best[0]:
            best = (gap, ((a[1] + b[0]) / 2.0))
    out["gap"] = best[1]
    out["gap_width"] = float(best[0])
    return out
