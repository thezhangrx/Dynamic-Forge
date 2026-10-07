"""``wall_with_gap`` 的像素域观测 → **标准视觉结构**（``core/standard/vision``）。

为什么要有这一层
----------------
两边各自都是对的，但彼此不认识：

* 检测器（``wall_with_gap.detect``）刻意只依赖 ``primitives``，不知道标准结构
  —— 这样它才能纯粹用"比较器 + 行缓冲 + 累加器"表达，逐条翻成 Verilog；
* ``localize.to_world`` 只认标准 :class:`VisionFrame`，因为它是"发出去的那一帧"
  的格式，必须和 ``core/standard`` 严格对齐。

于是中间必须有一层适配器。**两边都不要为了迁就对方而改** —— 这正是
"模块化 + 数据接口"该有的样子：接缝是显式的、可单独测试的，
而不是让检测器去 import 标准结构、或者让标准结构长出检测器的字段。

这一层只做**翻译**，不做任何几何运算
------------------------------------
坐标还全是**像素**（``units=Units.PX``）。像素 → 世界是
:func:`vision.localize.to_world` 的事。把翻译和几何混在一起，
以后要换检测器就得重测几何 —— 分开之后各自可测。
"""

from __future__ import annotations

import math
from typing import Any

from standard.obstacle.obstacle import Obstacle, ObstacleShape, ObstacleType
from standard.player.player import DEFAULT_FRAME_ID, PlayerState
from standard.vision.vision_output import (
    Calibration,
    GapObs,
    TargetObs,
    Units,
    VisionFrame,
)

from vision.wall_with_gap import FrameObservation, Wall

#: 检测出的墙段至少在图像里有多长才当成一条障碍。
#:
#: 检测器为了找缺口会把墙切成很多小段（实测一帧里出现过 8 px、17 px 的碎片），
#: 它们既不是真障碍，还会破坏标准结构的不变量 ``half_w >= half_h``。
#: 这里把太短的削掉 —— 缺口的两端仍然各自落在真正的长段上，所以缺口不受影响。
DEFAULT_MIN_SEGMENT_PX = 24


def _segment_obstacle(seg: Any, wall_index: int, *, obstacle_id: int,
                      thickness: float, rotation: float,
                      frame_id: str) -> Obstacle | None:
    """一条墙段 → 一个 ``rect`` 障碍；退化（短于厚度）返回 ``None``。

    ``rotation`` 由调用方按**整堵墙**算好再传进来，不用单段自己那个角 ——
    见 :func:`_wall_rotation`。
    """
    length = float(seg.x1 - seg.x0 + 1)
    # 厚度取"这一条墙带"的整体厚度：单段的 thickness 在碎片上会是 0。
    half_h = max(thickness, 1.0) / 2.0
    half_w = length / 2.0
    if half_w < half_h:
        return None                       # 不变量 half_w >= half_h，宁缺勿滥
    cx = (seg.x0 + seg.x1) / 2.0
    return Obstacle(
        id=obstacle_id,
        shape=ObstacleShape.RECT,
        x=cx, y=float(seg.cy),
        type=ObstacleType.WALL_WITH_GAP,
        frame_id=frame_id,
        half_w=half_w, half_h=half_h,
        rotation=rotation,
        # 同一堵墙的两段共用一个 group_id —— 这正是平台 ``wall_with_gap`` 的编码方式
        # （两段 rect 共享 group_id + 一个缺口）。
        group_id=wall_index,
    )


def _wall_rotation(wall: Wall) -> float:
    """整堵墙的朝向（弧度）—— **同一堵墙的所有段必须共用一个**。

    为什么不能各段用自己的：``cpu/gap_avoid.group_walls`` 按
    ``(rotation 量化, 墙厚方向坐标量化)`` 分桶，**旋转差一点点就落到不同桶里**，
    于是一堵两段的墙被拆成两堵单段的墙，``plan()`` 永远重建不出缺口，
    只会一直喊 ``NO_GAP``（实测就是这样：检测出的两段 y 相同、x 对称，
    却因为各自拟合出的角度差了 0.1 度而分家）。

    这里直接从**段心连线**求方向：两段（或更多段）共线，段心连线就是墙的方向，
    比任何单段自己拟合出来的角都稳。只有一段时退回用那一段的角。
    """
    segs = [s for s in wall.segments if s.x1 > s.x0]
    if len(segs) == 1:
        return math.radians(float(segs[0].angle_deg))
    pts = [((s.x0 + s.x1) / 2.0, float(s.cy)) for s in segs]
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    if abs(sxx) < 1e-9 and abs(syy) < 1e-9:
        return math.radians(float(segs[0].angle_deg))
    # 总体最小二乘（2x2 协方差主方向），和 `subpixel._fit_line` 同一套算法
    return 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def _gap_obstacle(wall: Wall, wall_index: int, blockers: tuple[int, int],
                  *, gap_id: int) -> GapObs:
    g = wall.gap
    assert g is not None
    return GapObs(
        id=gap_id,
        center=(float(g.cx), float(g.cy)),
        width=float(g.width_px),
        blockers=blockers,
        axis=math.radians(float(g.axis_deg)),
        blocked_by_type=ObstacleType.WALL_WITH_GAP.value,
        # 缺口贴到画面边缘时，宽度是推出来的而不是看到的 —— 这时不可信。
        reliable=bool(wall.segments) and not _gap_touches_border(wall),
        occluded=False,
    )


def _gap_touches_border(wall: Wall) -> bool:
    """缺口是不是"贴着画面边缘"（那样量到的宽度只是残段之间的空隙）。"""
    g = wall.gap
    if g is None:
        return True
    xs = [s.x0 for s in wall.segments] + [s.x1 for s in wall.segments]
    if not xs:
        return True
    return g.cx <= min(xs) or g.cx >= max(xs)


def frame_to_standard(obs: FrameObservation, *, seq: int, stamp: float,
                      calibration: Calibration | None = None,
                      frame_id: str = DEFAULT_FRAME_ID,
                      min_segment_px: int = DEFAULT_MIN_SEGMENT_PX,
                      obstacle_id_base: int = 100) -> VisionFrame:
    """把一帧检测结果翻成标准 :class:`VisionFrame`（**单位仍是像素**）。

    ``seq`` / ``stamp`` 由调用方给：时间戳必须来自**采集时刻**，
    不能由本函数自己打（那会把处理耗时算进 dt，速度就错了 —— 见 ``motion.py``）。
    """
    obstacles: list[Obstacle] = []
    gaps: list[GapObs] = []
    next_id = obstacle_id_base
    next_gap = 1

    for wi, wall in enumerate(obs.walls):
        rot = _wall_rotation(wall)
        if wall.gap is not None:
            placed: list[tuple[Any, int]] = []
            for seg in wall.segments:
                if (seg.x1 - seg.x0 + 1) < min_segment_px:
                    continue
                ob = _segment_obstacle(seg, wi, obstacle_id=next_id,
                                       thickness=wall.thickness_px,
                                       rotation=rot, frame_id=frame_id)
                if ob is None:
                    continue
                obstacles.append(ob)
                placed.append((seg, ob.id))
                next_id += 1
            # 缺口的两端 = 夹住它的那两段（按 x 排序后相邻的一对）
            left = [p for p in placed if p[0].x1 <= wall.gap.cx]
            right = [p for p in placed if p[0].x0 >= wall.gap.cx]
            if left and right:
                blockers = (max(left, key=lambda p: p[0].x1)[1],
                            min(right, key=lambda p: p[0].x0)[1])
                gaps.append(_gap_obstacle(wall, wi, blockers, gap_id=next_gap))
                next_gap += 1
        else:
            # 没有缺口 => 这是一堵实心墙，整条当一个障碍
            if not wall.segments:
                continue
            x0 = min(s.x0 for s in wall.segments)
            x1 = max(s.x1 for s in wall.segments)
            cy = sum(s.cy for s in wall.segments) / len(wall.segments)
            if (x1 - x0 + 1) < min_segment_px:
                continue
            half_h = max(wall.thickness_px, 1.0) / 2.0
            half_w = (x1 - x0 + 1) / 2.0
            if half_w < half_h:
                continue
            obstacles.append(Obstacle(
                id=next_id, shape=ObstacleShape.RECT,
                x=(x0 + x1) / 2.0, y=cy,
                type=ObstacleType.WALL_WITH_GAP, frame_id=frame_id,
                half_w=half_w, half_h=half_h, rotation=rot, group_id=wi,
            ))
            next_id += 1

    player = None
    if obs.player is not None:
        player = PlayerState(
            x=float(obs.player.x), y=float(obs.player.y),
            radius=float(obs.player.r),
            frame_id=frame_id, stamp=stamp, seq=seq,
            # 填充率是"这个圆有多像实心圆"，正好当检测置信度用
            score=float(obs.player.fill) if 0.0 <= obs.player.fill <= 1.0 else None,
        )

    target = None
    if obs.target is not None:
        target = TargetObs(x=float(obs.target.x), y=float(obs.target.y),
                           shape="circle", radius=float(obs.target.r))

    out = VisionFrame(
        seq=seq, stamp=stamp, frame_id=frame_id,
        units=Units.PX,
        image_width=obs.width, image_height=obs.height,
        calibration=calibration,
        player=player, obstacles=obstacles, gaps=gaps, target=target,
        diagnostics={"image": obs.image, "detector": "wall_with_gap",
                     "min_segment_px": int(min_segment_px),
                     **({"field_x": list(obs.field)} if obs.field else {})},
    )
    out.validate()
    return out


__all__ = ["DEFAULT_MIN_SEGMENT_PX", "frame_to_standard"]
