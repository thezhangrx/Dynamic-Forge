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
from typing import Any

from vision import primitives as P


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

    # -- ⑤ 段 / 缺口 ------------------------------------------------------
    min_seg_px: int = 8
    min_gap_px: int = 8

    # -- ⑥ 角色（白圆） ---------------------------------------------------
    player_min_area: int = 60
    player_max_area: int = 1500
    player_min_fill: float = 0.60    # 面积/外接框；正圆 ≈ 0.785
    player_max_aspect: float = 1.6

    # -- ⑥ 目标（绿环） ---------------------------------------------------
    target_min_area: int = 40
    target_max_area: int = 1500
    target_max_aspect: float = 1.6
    target_roi: tuple[float, float] = (0.90, 0.85)   # 目标只可能在这个比例范围内


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
        # 持续台阶：认出被灯洗白的墙，同时不把 1 像素宽的网格线当墙
        edges = P.col_step_max(frame, y0, y1, x_min, x_max, k=p.edge_avg)
        edge_limit = p.edge_min * max(1, p.edge_avg)
        limit = max(p.wall_col_min, int(p.wall_col_frac * h))
        flags = [counts[i] >= limit or edges[i] >= edge_limit
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
def detect_player(bright_mask: bytearray, width: int, height: int,
                  p: WallWithGapParams) -> PlayerObs | None:
    blobs = P.scanline_blobs(bright_mask, width, 0, height - 1, 0, width - 1,
                             min_area=p.player_min_area)
    best: P.Blob | None = None
    for b in blobs:
        if b.area > p.player_max_area:
            continue
        if b.x0 <= 0 or b.y0 <= 0 or b.x1 >= width - 1 or b.y1 >= height - 1:
            continue                      # 贴边的大片高光不要
        if b.fill < p.player_min_fill or b.aspect > p.player_max_aspect:
            continue
        if best is None or b.fill > best.fill:
            best = b
    if best is None:
        return None
    return PlayerObs(best.cx, best.cy, (best.w + best.h) / 4.0, best.fill)


def detect_target(green_mask: bytearray, width: int, height: int,
                  p: WallWithGapParams) -> TargetObs | None:
    blobs = P.scanline_blobs(green_mask, width, 0, height - 1, 0, width - 1,
                             min_area=p.target_min_area)
    rx, ry = p.target_roi
    best: P.Blob | None = None
    for b in blobs:
        if b.area > p.target_max_area or b.aspect > p.target_max_aspect:
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
           *, name: str = "<frame>") -> FrameObservation:
    """一帧 → 结构化识别结果。整条链只有纯 Python 运算。"""
    p = params or WallWithGapParams()
    masks = P.classify(frame,
                       wall_diff=p.wall_diff, wall_min_r=p.wall_min_r,
                       wall_min_g=p.wall_min_g, bright_min=p.bright_min,
                       bright_spread=p.bright_spread, green_score=p.green_score,
                       green_min_g=p.green_min_g)
    walls, field = detect_walls(frame, masks.wall, p)
    return FrameObservation(
        image=name, width=frame.width, height=frame.height,
        player=detect_player(masks.bright, frame.width, frame.height, p),
        target=detect_target(masks.green, frame.width, frame.height, p),
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
