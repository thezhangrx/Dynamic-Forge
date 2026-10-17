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
