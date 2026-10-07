"""``wall_with_gap`` 的图像识别：两段墙 + 一个缺口，外加角色与目标。

**纯 Python 实现**（只有 ``image_io.py`` 碰第三方库），全部算子都能直接翻译成 Verilog。

识别流程（每一步都对应一个硬件部件）
------------------------------------
::

    ① 逐像素分类      三个并行减法器 + 比较器            → 墙掩膜 / 白掩膜 / 绿掩膜
    ② 行直方图        行累加器                            → 墙所在的 y 带（每面墙一条带）
    ③ 场地 x 范围     列累加器（只在墙带上统计）           → 排除屏幕外的背景
    ④ 列判定          列累加器 + 纵向差分（行缓冲）        → 这一列是不是墙
    ⑤ 游程状态机      一维 run-length                     → 墙段；两段之间就是缺口
    ⑥ 扫描线连通域    上一行游程缓存                      → 角色圆 / 目标环

为什么要第 ④ 步的"纵向差分"
---------------------------
灯在屏幕上形成的高光会把墙的颜色**完全洗白**（实测该区域 B≈243, G=R≈255，
``R-B`` 只剩 13，和纯眩光的 13.9 无法区分）。颜色判据在那里必然失效。
但墙的上下边缘仍在：实测墙底边的亮度台阶 ``|ΔG|`` 在被洗白的 x=350 处还有 25.7，
而缺口内部只有 3.3。所以"列是不是墙"用 **颜色或边缘** 两个判据取或——这正是把
第二个缺口从 111px 修回 45px 的原因。
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from dataclasses import field as dc_field
from typing import Any, Callable, ClassVar

from vision import primitives as P
from vision import subpixel as S


# --------------------------------------------------------------------------
# 参数（全部集中在这里，标定后换成世界单位即可）
# --------------------------------------------------------------------------
@dataclass
class WallWithGapParams:
    # -- ① 逐像素分类 -----------------------------------------------------
    wall_diff: int = 40          # R - B，橙墙；40 而不是 45：眩光下 R-B 只剩 41
    wall_min_r: int = 110
    wall_min_g: int = 60
    bright_min: int = 195        # 白色玩家
    bright_spread: int = 45
    green_score: int = 10        # G - max(R, B)，绿色目标环
    green_min_g: int = 90

    # -- ② 行带：墙在哪些行 -----------------------------------------------
    # 5% 而不是 10%：墙有约 1° 的倾角，右段比左段低 20 行；阈值太高会把
    # 远处那半截墙排除在带外（实测 y=30..42 只有 30~46 个墙像素）。
    band_row_frac: float = 0.05
    band_join_rows: int = 4      # 间隔这么近的墙行合并（补网格线断口）
    band_min_rows: int = 5

    # -- ③ 场地 x 范围 ----------------------------------------------------
    field_col_min: int = 5       # 一列在墙带上有这么多墙像素 → 属于场地

    # -- ④ 列判定 ---------------------------------------------------------
    wall_col_frac: float = 0.25  # 列内墙像素 >= 比例 × 带高
    wall_col_min: int = 2
    edge_min: int = 12           # 持续台阶阈值（实际比较 edge_min*k）
    #: 台阶的抽头半径：上 k 行之和 vs 下 k 行之和。k 越大越抗 1 像素宽的网格线
    edge_avg: int = 2
    #: 走**边沿分支**的列至少要有几个墙色像素。见 `detect_walls` 里的说明：
    #: 光靠边沿阈值分不开"洗白的墙"和"缺口里的杂边"（真机上后者一样强），
    #: 但"墙再白也留得下几像素墙色、网格线一像素都没有"这条差别是稳的。
    edge_min_cols: int = 1

    # -- ⑤ 段 / 缺口 ------------------------------------------------------
    #: 墙段短于它就丢掉（像素）。
    #:
    #: **别再靠调大它来滤网格线** —— 曾经因为网格线把缺口切碎而把它从 8 提到 16，
    #: 那只是把症状盖住，而且**会制造幻影缺口**：墙中间一段短段被丢掉，
    #: 左右两段就变成相邻，凭空多出一个宽度等于该短段的缺口。
    #: 网格线已经在 `detect_walls` 里从根上解决（边沿分支要求列内至少有几个
    #: 墙色像素，见 `edge_min_cols`）。
    min_seg_px: int = 8
    min_gap_px: int = 8

    # -- ⑥ 角色（白圆） ---------------------------------------------------
    player_min_area: int = 60
    player_max_area: int = 1500
    # 面积/外接框。注意**小圆达不到理论值 0.785**：pygame 光栅化的填充圆
    # 在半径 10 时实测只有 0.58（半径越大越接近 0.785）。原来卡在 0.60，
    # 结果干净渲染下玩家根本检不出来；真机上只是因为镜头模糊把描边糊进白芯
    # 才勉强过线 —— 那是"靠模糊兜底"，不该依赖。降到 0.45。
    # 形状主要由 aspect + area 两档把关，fill 只负责挡"稀疏/环状"的块。
    player_min_fill: float = 0.45
    player_max_aspect: float = 1.6

    # -- ⑥ 目标（绿环） ---------------------------------------------------
    target_min_area: int = 40
    target_max_area: int = 1500
    target_max_aspect: float = 1.6
    target_roi: tuple[float, float] = (0.90, 0.85)   # 目标只可能在这个比例范围内

    # ------------------------------------------------------------------
    #: 下面这些默认值是**按 640x480 调的**。换分辨率时调用 :meth:`rescaled`，
    #: 不要直接改这些数。
    #:
    #: 为什么要有这个机制：实测拿一段 **1920x1080** 的录屏跑，玩家圆是 1763 px，
    #: 而 `player_max_area` 卡在 1500 —— 玩家被丢掉，检测器退而选了旁边一块
    #: 被光洗白的区域，**看起来只是"检测不准"，很难反推到"上限是按分辨率写死的"**。
    #: 参数里悄悄编码一个分辨率，就等于把"可移植"这个性质丢了。
    # --
    #: 这些默认值对应的参考分辨率。
    REF_WIDTH: ClassVar[int] = 640
    REF_HEIGHT: ClassVar[int] = 480

    def rescaled(self, width: int, height: int) -> "WallWithGapParams":
        """按帧尺寸缩放**长度/面积**类的像素参数，返回一份新参数。

        哪些该缩放、哪些**不该**：

        * **长度**（`min_seg_px`/`min_gap_px`/行带/列数阈值）→ 乘 ``s``；
        * **面积**（玩家/目标的 min/max area）→ 乘 ``s²``；
        * **灰度差**（`wall_diff`/`edge_min`/`bright_min`/…）→ **原样**。
          它们是"颜色/亮度差"，和分辨率无关 —— 缩放它们就变成另一个判据了。
        * **比例**（`wall_col_frac`/`target_roi`）→ **原样**，它们本来就是比例。

        ``s`` 取宽高比例的**较小**者：宁可保守（把窗口设小）也不要放大到
        把整幅画面当成目标。长宽比与参考不同时，这是安全的一侧。
        """
        s = min(width / self.REF_WIDTH, height / self.REF_HEIGHT)
        if abs(s - 1.0) < 1e-9:
            return self                       # 640x480：原样返回，零开销
        import dataclasses
        return dataclasses.replace(
            self,
            wall_col_min=max(1, int(round(self.wall_col_min * s))),
            field_col_min=max(1, int(round(self.field_col_min * s))),
            edge_avg=max(1, int(round(self.edge_avg * s))),
            band_join_rows=max(1, int(round(self.band_join_rows * s))),
            band_min_rows=max(1, int(round(self.band_min_rows * s))),
            min_seg_px=max(1, int(round(self.min_seg_px * s))),
            min_gap_px=max(1, int(round(self.min_gap_px * s))),
            player_min_area=max(1, int(round(self.player_min_area * s * s))),
            player_max_area=max(1, int(round(self.player_max_area * s * s))),
            target_min_area=max(1, int(round(self.target_min_area * s * s))),
            target_max_area=max(1, int(round(self.target_max_area * s * s))),
        )


# --------------------------------------------------------------------------
# 结果类型（字段名与平台 State 帧 v3 对齐）
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class WallSegment:
    x0: int
    x1: int
    cy: float
    thickness: float
    angle_deg: float

    @property
    def length(self) -> int:
        return self.x1 - self.x0 + 1

    def to_dict(self) -> dict[str, Any]:
        return {"x0": self.x0, "x1": self.x1, "cx": (self.x0 + self.x1) / 2.0,
                "cy": self.cy, "length": self.length,
                "thickness": self.thickness, "angle_deg": self.angle_deg}


@dataclass(frozen=True)
class Gap:
    cx: float
    cy: float
    width_px: float
    axis_deg: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class Wall:
    """一面墙 = 一条 y 带上的若干墙段；``gap`` 为 ``None`` 表示缺口贴边。"""

    y0: int
    y1: int
    segments: tuple[WallSegment, ...]
    thickness_px: float
    covered_px: float
    gap: Gap | None

    def to_dict(self) -> dict[str, Any]:
        return {"y0": self.y0, "y1": self.y1,
                "segments": [s.to_dict() for s in self.segments],
                "thickness_px": self.thickness_px,
                "covered_px": self.covered_px,
                "gap": self.gap.to_dict() if self.gap else None}


@dataclass(frozen=True)
class PlayerObs:
    x: float
    y: float
    r: float
    fill: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TargetObs:
    x: float
    y: float
    r: float
    w: int
    h: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class FrameObservation:
    image: str
    width: int
    height: int
    player: PlayerObs | None
    target: TargetObs | None
    walls: tuple[Wall, ...] = ()
    field: tuple[int, int] = (0, 0)
    diagnostics: dict[str, Any] = dc_field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "image": self.image,
            "width": self.width,
            "height": self.height,
            "field_x": list(self.field),
            "player": self.player.to_dict() if self.player else None,
            "target": self.target.to_dict() if self.target else None,
            "walls": [w.to_dict() for w in self.walls],
            "gaps": [w.gap.to_dict() for w in self.walls if w.gap],
            "diagnostics": self.diagnostics,
        }


# --------------------------------------------------------------------------
# ② 行带
# --------------------------------------------------------------------------
def find_wall_bands(wall_mask: bytearray, width: int, height: int,
                    p: WallWithGapParams) -> list[tuple[int, int]]:
    """墙面在画面的哪些行上：行直方图 → 游程 → 合并 → 丢掉太薄的。"""
    counts = P.row_counts(wall_mask, width, 0, height - 1, 0, width - 1)
    limit = max(1, int(p.band_row_frac * width))
    flags = [c >= limit for c in counts]
    bands = P.join_close(P.runs(flags), p.band_join_rows)
    return [b for b in bands if b[1] - b[0] + 1 >= p.band_min_rows]


# --------------------------------------------------------------------------
# ③ 场地 x 范围
# --------------------------------------------------------------------------
def find_field_span(wall_mask: bytearray, width: int, height: int,
                    bands: list[tuple[int, int]], p: WallWithGapParams) -> tuple[int, int]:
    """墙横跨整个场地，所以"墙像素够多的列"就是场地范围，屏幕外的背景被排除。"""
    if not bands:
        return 0, width - 1
    col = [0] * width
    for y0, y1 in bands:
        for x in range(width):
            c = 0
            for y in range(y0, y1 + 1):
                c += wall_mask[y * width + x]
            col[x] += c
    good = [x for x, c in enumerate(col) if c >= p.field_col_min]
    return (good[0], good[-1]) if good else (0, width - 1)


# --------------------------------------------------------------------------
# ⑤ 墙段几何
# --------------------------------------------------------------------------
def _segment_stats(wall_mask: bytearray, width: int, y0: int, y1: int,
                   x0: int, x1: int) -> tuple[float, float, float]:
    """一段墙的 ``(cy, thickness, angle_deg)``。

    厚度 = 该段内单列墙像素的最大值；倾角 = 段左右两端各自的重心 y 连线的斜率。
    """
    total = 0
    sum_y = 0
    thick = 0
    for x in range(x0, x1 + 1):
        col = 0
        for y in range(y0, y1 + 1):
            if wall_mask[y * width + x]:
                col += 1
                total += 1
                sum_y += y
        if col > thick:
            thick = col
    cy = (sum_y / total) if total else 0.5 * (y0 + y1)
    # 倾角：左右各取一小段求重心 y
    span = max(1, min(20, (x1 - x0 + 1) // 4))
    left = _centroid_y(wall_mask, width, y0, y1, x0, x0 + span - 1)
    right = _centroid_y(wall_mask, width, y0, y1, x1 - span + 1, x1)
    dx = (x1 - span + 1) - (x0 + span - 1)
    angle = math.degrees(math.atan2(right - left, dx)) if dx > 0 else 0.0
    return cy, float(thick), angle


def _centroid_y(wall_mask: bytearray, width: int, y0: int, y1: int,
                x0: int, x1: int) -> float:
    total = 0
    sum_y = 0
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            if wall_mask[y * width + x]:
                total += 1
                sum_y += y
    return (sum_y / total) if total else 0.5 * (y0 + y1)


def detect_walls(frame: P.Frame, wall_mask: bytearray,
                 p: WallWithGapParams) -> tuple[tuple[Wall, ...], tuple[int, int]]:
    width, height = frame.width, frame.height
    bands = find_wall_bands(wall_mask, width, height, p)
    x_min, x_max = find_field_span(wall_mask, width, height, bands, p)

    walls: list[Wall] = []
    for y0, y1 in bands:
        h = y1 - y0 + 1
        counts = P.col_counts(wall_mask, width, x_min, x_max, y0, y1)
        # 持续台阶：认出被灯洗白的墙（颜色没了、整块亮度跳变还在）
        edges = P.col_step_max(frame, y0, y1, x_min, x_max, k=p.edge_avg)
        edge_limit = p.edge_min * max(1, p.edge_avg)
        limit = max(p.wall_col_min, int(p.wall_col_frac * h))
        # 边沿分支**必须要求这一列至少有几个墙色像素**（``counts[i] > 0``）。
        #
        # 为什么：光靠边沿阈值分不开"被洗白的墙"和"缺口里的杂边"。实测同一帧里
        #   合成画面 墙列边缘=260，缺口里的网格线=42~74
        #   真实拍摄 墙列边缘 中位 200~256，而**缺口里的列最高到 285**
        # 也就是说真机上缺口处的反光边界/窗框边缘跟墙一样强，任何固定阈值都会误判。
        # 但两者有个物理差别是稳的：**墙再白也总有几像素落在颜色键里，
        # 网格线和窗框边缘则一个都没有**（实测缺口列 counts 恒为 0）。
        # 这一条同时保住了边沿分支的初衷：洗白的墙靠它的少墙色像素 + 强边沿被认出来。
        flags = [counts[i] >= limit
                 or (counts[i] >= p.edge_min_cols and edges[i] >= edge_limit)
                 for i in range(len(counts))]
        seg_runs = P.drop_short(P.runs(flags), p.min_seg_px)

        segments: list[WallSegment] = []
        for a, b in seg_runs:
            sx0, sx1 = a + x_min, b + x_min
            cy, thick, angle = _segment_stats(wall_mask, width, y0, y1, sx0, sx1)
            segments.append(WallSegment(sx0, sx1, cy, thick, angle))
        if not segments:
            continue

        gap = None
        best = 0
        for s0, s1 in zip(segments, segments[1:]):
            w = s1.x0 - s0.x1 - 1
            if w >= p.min_gap_px and w > best:
                best = w
                gap = Gap(cx=(s0.x1 + s1.x0) / 2.0,
                          cy=0.5 * (s0.cy + s1.cy),
                          width_px=float(w),
                          axis_deg=0.5 * (s0.angle_deg + s1.angle_deg))
        thickness = max((s.thickness for s in segments), default=float(h))
        covered = float(sum(s.length for s in segments))
        walls.append(Wall(y0, y1, tuple(segments), thickness, covered, gap))

    walls.sort(key=lambda w: -w.covered_px)
    return tuple(walls), (x_min, x_max)


# --------------------------------------------------------------------------
# ⑥ 角色 / 目标
# --------------------------------------------------------------------------
def _mask_roi(mask: bytearray, width: int, height: int, roi) -> None:
    """把 ROI 之外的掩膜清零 —— **就地改**。

    为什么在掩膜这一层做，而不是在每个检测器里逐个判断：ROI 的语义是
    "场地之外的东西按定义不是场地上的目标"，这是**输入**的约束，不是某个检测器
    的偏好。放在这里，墙带、列投影、连通域全都自动只在场地内找。

    实测：只在 `detect_player` 里加 ROI 时，玩家的假阳性清掉了，
    但**墙**还是被窗口外的文字干扰，算出来的缺口宽度是 270（真值 60）。
    """
    if roi is None:
        return
    x0 = max(0, int(roi[0])); x1 = min(width - 1, int(roi[2]))
    y0 = max(0, int(roi[1])); y1 = min(height - 1, int(roi[3]))
    if x1 < x0 or y1 < y0:
        return
    for y in range(height):
        base = y * width
        if y < y0 or y > y1:
            for x in range(width):
                mask[base + x] = 0
        else:
            for x in range(0, x0):
                mask[base + x] = 0
            for x in range(x1 + 1, width):
                mask[base + x] = 0


def _in_roi(b, roi) -> bool:
    """块心在不在 ROI 里。``roi`` 是 ``(x0, y0, x1, y1)``，``None`` = 不限。"""
    if roi is None:
        return True
    return roi[0] <= b.cx <= roi[2] and roi[1] <= b.cy <= roi[3]


def detect_player(bright_mask: bytearray, width: int, height: int,
                  p: WallWithGapParams, *, frame=None, roi=None,
                  refine: bool = True) -> PlayerObs | None:
    """在亮块里挑出玩家圆。

    打分用 ``fill × area``（也就是"实心像素数"）而**不是**只看 ``fill``：
    只看 fill 会挑中一个又小又圆的高光斑点。实测半径 16 的玩家（面积 804、
    fill 0.70）会被一个 14x16、fill 0.73 的小块挤掉 —— 体积项必须进来。
    反过来只看 area 也不行：一块方方正正的高光面积可能更大。两项都看才对。
    """
    blobs = P.scanline_blobs(bright_mask, width, 0, height - 1, 0, width - 1,
                             min_area=p.player_min_area)
    best: P.Blob | None = None
    best_score = 0.0
    for b in blobs:
        if b.area > p.player_max_area:
            continue
        if not _in_roi(b, roi):
            continue                      # 场地之外的东西按定义不是场地上的目标
        if b.x0 <= 0 or b.y0 <= 0 or b.x1 >= width - 1 or b.y1 >= height - 1:
            continue                      # 贴边的大片高光不要
        if b.fill < p.player_min_fill or b.aspect > p.player_max_aspect:
            continue
        score = b.fill * b.area
        if score > best_score:
            best, best_score = b, score
    if best is None:
        return None
    cx, cy, r = best.cx, best.cy, (best.w + best.h) / 4.0
    if refine and frame is not None:
        # 掩膜只能给整数外接框，而且**相机模糊会把边界往外糊**：
        # 实测模糊核 5 px 时外接框法偏小 12%、模糊核 7 px 时偏小 22%，
        # 而半高法只有 3% 上下。半径进了"缺口 >= 2r"这条硬约束，不能差这么多。
        cx, cy, r = S.refine_circle(frame, cx, cy, r)
    return PlayerObs(cx, cy, r, best.fill)


def detect_target(green_mask: bytearray, width: int, height: int,
                  p: WallWithGapParams, *, roi=None) -> TargetObs | None:
    blobs = P.scanline_blobs(green_mask, width, 0, height - 1, 0, width - 1,
                             min_area=p.target_min_area)
    rx, ry = p.target_roi
    best: P.Blob | None = None
    for b in blobs:
        if b.area > p.target_max_area or b.aspect > p.target_max_aspect:
            continue
        if not _in_roi(b, roi):
            continue
        if b.cx >= width * rx or b.cy >= height * ry:
            continue
        if b.x0 <= 0 or b.y0 <= 0 or b.x1 >= width - 1 or b.y1 >= height - 1:
            continue
        if best is None or b.area > best.area:
            best = b
    if best is None:
        return None
    return TargetObs(best.cx, best.cy, (best.w + best.h) / 4.0, best.w, best.h)


# --------------------------------------------------------------------------
# 总入口
# --------------------------------------------------------------------------
def detect(frame: P.Frame, params: WallWithGapParams | None = None,
           *, name: str = "<frame>", refine: bool = True,
           roi: tuple[float, float, float, float] | None = None,
           classifier: Callable[[P.Frame], P.Masks] | None = None,
           ) -> FrameObservation:
    """一帧 → 结构化识别结果。整条链只有纯 Python 运算。

    ``classifier`` 默认 ``None`` 表示沿用**绝对阈值** :func:`P.classify`
    （行为一字不变）。传入 :func:`vision.adaptive.classify_adaptive` 的偏函数
    就换成**自适应 + 弃权**那条路 —— 换的只是分类层，
    墙带/缺口/连通域/亚像素细分这些下游一个字节都不用动。
    """
    # 参数按帧尺寸缩放：默认值是给 640x480 调的，换分辨率必须跟着换，
    # 否则玩家/目标会被面积上限误杀（实测 1920x1080 上就是这么挂的）。
    p = (params or WallWithGapParams()).rescaled(frame.width, frame.height)
    if classifier is None:
        masks = P.classify(frame,
                           wall_diff=p.wall_diff, wall_min_r=p.wall_min_r,
                           wall_min_g=p.wall_min_g, bright_min=p.bright_min,
                           bright_spread=p.bright_spread, green_score=p.green_score,
                           green_min_g=p.green_min_g)
    else:
        masks = classifier(frame)
    # ROI 作用在**掩膜**上：场地之外的东西按定义不是场地上的目标。
    # 放在这一层，墙带/列投影/连通域全都自动只在场地内找。
    # 实测：只在 `detect_player` 里加 ROI，玩家的假阳性清掉了，但**墙**还是被
    # 窗口外的文字干扰，算出来的缺口宽度是 270（真值 60）。
    if roi is not None:
        for which in ("wall", "bright", "green"):
            _mask_roi(getattr(masks, which), frame.width, frame.height, roi)
    walls, field = detect_walls(frame, masks.wall, p)
    return FrameObservation(
        image=name, width=frame.width, height=frame.height,
        player=detect_player(masks.bright, frame.width, frame.height, p,
                             frame=frame, roi=roi, refine=refine),
        target=detect_target(masks.green, frame.width, frame.height, p, roi=roi),
        walls=walls, field=field,
        diagnostics={"params": asdict(p)},
    )


# --------------------------------------------------------------------------
# 可视化（纯 Python 画图，保持与平台窗口一致的"只有图形、没有文字"风格）
# --------------------------------------------------------------------------
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)


def annotate(frame: P.Frame, obs: FrameObservation) -> P.Frame:
    w, h = frame.width, frame.height
    buf = frame.to_rgb()
    for wall in obs.walls:
        for s in wall.segments:
            P.draw_rect_outline(buf, w, h, s.x0, wall.y0, s.x1, wall.y1, YELLOW)
        if wall.gap is not None:
            half = wall.gap.width_px / 2.0
            t = math.radians(wall.gap.axis_deg)
            dx, dy = math.cos(t), math.sin(t)
            x0 = int(wall.gap.cx - dx * half)
            y0 = int(wall.gap.cy - dy * half)
            x1 = int(wall.gap.cx + dx * half)
            y1 = int(wall.gap.cy + dy * half)
            P.draw_line(buf, w, h, x0, y0, x1, y1, GREEN, 3)
            nx, ny = -dy, dx
            tick = int(max(6.0, wall.thickness_px))
            for px, py in ((x0, y0), (x1, y1)):
                P.draw_line(buf, w, h,
                            int(px - nx * tick), int(py - ny * tick),
                            int(px + nx * tick), int(py + ny * tick), GREEN, 2)
    if obs.player is not None:
        P.draw_circle(buf, w, h, int(obs.player.x), int(obs.player.y),
                      int(round(obs.player.r)), RED, 2)
    if obs.target is not None:
        P.draw_circle(buf, w, h, int(obs.target.x), int(obs.target.y),
                      int(round(obs.target.r)), GREEN, 2)
    return P.Frame.from_rgb(w, h, buf)


__all__ = [
    "FrameObservation",
    "Gap",
    "PlayerObs",
    "TargetObs",
    "Wall",
    "WallSegment",
    "WallWithGapParams",
    "annotate",
    "detect",
    "detect_player",
    "detect_target",
    "detect_walls",
    "find_field_span",
    "find_wall_bands",
]
