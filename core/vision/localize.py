"""任务一/二：像素 → 世界坐标 + 距离。

两条路，按有没有参考物选
------------------------
**A. 单应映射**（用 :mod:`vision.calibration` 的标定结果）——**首选**。
平面场景下它是**精确**的（不是近似）；每个点 8 乘 2 加 1 除，最省算力。

**B. 地面反投影**（单相机 + 已知安装高度）——没有参考物时的退路。
把像点变成一条射线，再和地面求交：

```
射线方向（相机系）:      d = ( (u − cx)/fx , (v − cy)/fy , 1 )
绕 pitch 旋转到世界系:   d_w = R(pitch) · d
与地面 y = h 求交:       t = h / d_w.y
世界点:                  P = t · d_w         （原点在相机正下方的地面）
```

当 ``pitch = 0``（光轴水平）时它退化成教科书那两条：

```
Z = fy·h / (v − cy)        X = (u − cx)·Z / fx
```

另外提供**已知尺寸测距**：``Z = f·W / w_px``（物体真实宽度已知 —— 比如墙角距离
就是用"缺口宽度"这个已知量反推的）。

**可迁移性**：A 的每个点 = 6 乘加 + 1 倒数；B 的每个点 = 3 乘加 + 1 除 + 少量三角函数
（pitch 固定时三角函数是常数，能预计算成寄存器）。两者都无分支、无迭代，适合流水线。

参考的开源项目
--------------
* ``opencv-python-tutorials/15_image_transformations`` —— 4 点单应把斜视地面压成俯视图；
* ``Stereo_Vision_Camera/README.md`` —— 针孔模型与 ``Z = f·b/d`` 的推导，以及
  "误差随距离平方增长"的结论（本模块的 :func:`depth_error` 就是那条公式）；
* ``data_standard``（``MapMetaData.origin/resolution``、``CameraInfo.K/P``）—— 字段命名与
  "K 是畸变原图、P 是校正后图，不可混用"的坑。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Iterable, Sequence

from standard.obstacle.obstacle import Obstacle, ObstacleShape
from standard.vision.vision_output import GapObs, TargetObs, Units, VisionFrame

from .calibration import FieldCalibration, Point, apply_homography

#: 世界单位统一用米（标定时已确定 metres_per_unit）。
DEFAULT_WORLD_UNITS = Units.M


# ==========================================================================
# A. 单应映射：像素 → 场地坐标
# ==========================================================================
def points_to_world(calib: FieldCalibration,
                    points: Iterable[Point]) -> list[Point]:
    """一批图像点 → 场地坐标。"""
    return [calib.image_to_field(x, y) for x, y in points]


def distance(calib: FieldCalibration, p1: Point, p2: Point) -> float:
    """两个图像点之间的真实距离（米）。"""
    return calib.distance_metres(p1, p2)


def length_scale(calib: FieldCalibration, p1: Point, p2: Point,
                 *, eps: float = 1e-9) -> float:
    """局部**米/像素**比例（取两点中点处的等效尺度）。

    只是给"心里有个数"用的诊断量：透视下这个比值随位置变化，所以**不要**用它
    去把一串像素长度乘成米 —— 那件事必须走 :func:`distance`（先映射再量）。
    """
    (X1, Y1), (X2, Y2) = calib.image_to_field(*p1), calib.image_to_field(*p2)
    px = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    return calib.unit_to_metres(math.hypot(X2 - X1, Y2 - Y1)) / max(px, eps)


# ==========================================================================
# B. 地面反投影（没有参考物时的退路）
# ==========================================================================
@dataclass(frozen=True)
class GroundCamera:
    """一台架在 ``height_m`` 高、俯仰 ``pitch_deg`` 的相机（针孔模型）。

    ``pitch_deg``：0 = 光轴水平；正值 = **向下俯**（俯视地面）。这是最常见的
    机器人安装方式，而教科书那条 ``Z = fy·h/(v−cy)`` 只在 0 时成立。
    """

    height_m: float
    fx: float
    fy: float
    cx: float
    cy: float
    pitch_deg: float = 0.0

    def ray(self, u: float, v: float) -> tuple[float, float, float]:
        """像点 → 世界系里的射线方向（未归一化）。

        推导：相机系是「x 右、y 下、z 前」（左手系），世界系是「x 右、y 上、z 前」，
        所以先取 ``dy = (v − cy)/fy``（正 = 在光轴**下方**），到世界系要把它翻成
        ``−dy``（下方 = y 负）。再把相机绕 x 轴**下俯** ``pitch``：

        ```
        R_x(p) = [[1,0,0],[0,cos p,−sin p],[0,sin p,cos p]]
        d_world = R_x(p) · (dx, −dy, 1)
                = ( dx, −dy·cos p − sin p, −dy·sin p + cos p )
        ```

        自检：``pitch = 0`` 时 ``d = (dx, −dy, 1)``；``pitch = 90°`` 时
        ``d = (dx, −1, −dy)`` —— 光轴竖直向下，正下方那个点就是最短距离。
        """
        dx = (u - self.cx) / self.fx
        dy = (v - self.cy) / self.fy
        p = math.radians(self.pitch_deg)
        c, s = math.cos(p), math.sin(p)
        return dx, -dy * c - s, -dy * s + c

    def to_ground(self, u: float, v: float) -> tuple[float, float]:
        """像点 → 地面坐标 ``(X, Z)``（原点在相机正下方，X 右、Z 前，单位与 height 相同）。"""
        wx, wy, wz = self.ray(u, v)
        if wy >= -1e-12:
            raise ValueError(
                f"这条射线不朝下（wy={wy:.3f}），与地面无交点："
                "像点在地平线以上，或 pitch 方向反了"
            )
        t = self.height_m / (-wy)
        return t * wx, t * wz

    def ground_distance(self, u: float, v: float) -> float:
        """相机到该地面点的水平距离。"""
        X, Z = self.to_ground(u, v)
        return math.hypot(X, Z)


def known_size_depth(focal_px: float, real_size: float, px_size: float) -> float:
    """已知真实尺寸的物体有多远：``Z = f·W / w``。

    例：墙角宽度 ``W`` 已知（就是缺口宽度），量出它在图像里占 ``w`` 个像素，
    就能反推深度。``focal_px`` 可以从标定或内参拿。
    """
    if px_size <= 0.0:
        raise ValueError(f"像素尺寸必须 > 0，得到 {px_size!r}")
    return focal_px * real_size / px_size


def depth_error(depth: float, focal_px: float, baseline: float,
                disparity_px: float = 0.5) -> float:
    """双目深度误差 ``δZ = Z²·δd / (f·B)`` —— 误差随距离**平方**增长。

    这个函数放在这里是为了让"要不要上双目"这件事可以被算出来，而不是拍脑袋：
    实测参考项目 ``f=961.94 px``、``B=120 mm`` 时，5 cm 精度只能保证到约 3 m。
    """
    if focal_px <= 0.0 or baseline <= 0.0:
        raise ValueError("焦距与基线都必须 > 0")
    return depth * depth * disparity_px / (focal_px * baseline)


# ==========================================================================
# 把视觉输出整帧搬到世界坐标（任务一真正的交付物）
# ==========================================================================
def to_world(frame: VisionFrame, calib: FieldCalibration) -> VisionFrame:
    """把一帧**像素单位**的 :class:`VisionFrame` 换成**世界单位（米）**。

    规则：
    * 中心点 —— 直接过单应；
    * 半径 / 半宽 / 半高 —— **不能乘一个比例**（透视不是线性尺度），而是把
      "中心 + 沿该轴的半长"也映射过去，取两点距离。这是唯一在透视下正确的做法；
    * 朝向 —— 映射轴向量后取 ``atan2``。

    输出的 ``units="m"``、``metres_per_unit=1.0``，并回填 ``calibration`` 信息。
    """
    if frame.units == Units.M:
        return frame                                  # 已经是世界单位，幂等

    def pt(x: float, y: float) -> Point:
        return calib.image_to_field(x, y)

    def extent(cx: float, cy: float, half: float, theta: float) -> float:
        return _world_extent(calib, cx, cy, half, theta)

    # -- 角色 ------------------------------------------------------------
    player = frame.player
    if player is not None:
        X, Y = pt(player.x, player.y)
        # 圆的半径：两个互相垂直的方向各量一次再平均（圆不该有方向偏好）
        r = 0.5 * (extent(player.x, player.y, player.radius, 0.0)
                   + extent(player.x, player.y, player.radius, math.pi / 2))
        player = replace(player, x=X, y=Y, radius=r, frame_id=frame.frame_id)

    # -- 障碍 ------------------------------------------------------------
    obstacles: list[Obstacle] = []
    for ob in frame.obstacles:
        if ob.shape == ObstacleShape.CIRCLE:
            X, Y = pt(ob.x, ob.y)
            r = 0.5 * (extent(ob.x, ob.y, ob.radius or 0.0, 0.0)
                       + extent(ob.x, ob.y, ob.radius or 0.0, math.pi / 2))
            obstacles.append(replace(ob, x=X, y=Y, radius=r,
                                     half_w=r, half_h=r, frame_id=frame.frame_id))
        else:
            (X, Y), hw, hh, rot = _rect_to_world(
                calib, ob.x, ob.y, ob.half_w or 0.0, ob.half_h or 0.0, ob.rotation)
            obstacles.append(replace(ob, x=X, y=Y, half_w=hw, half_h=hh,
                                     rotation=rot, frame_id=frame.frame_id))

    # -- 缺口（派生关系：中心沿轴向的两端都映射过去） ----------------------
    gaps: list[GapObs] = []
    for g in frame.gaps:
        X, Y = pt(*g.center)
        w = 2.0 * extent(g.center[0], g.center[1], g.width / 2.0, g.axis)
        X0, Y0 = pt(g.center[0], g.center[1])
        X1, Y1 = pt(g.center[0] + math.cos(g.axis), g.center[1] + math.sin(g.axis))
        gaps.append(replace(g, center=(X, Y), width=w,
                            axis=math.atan2(Y1 - Y0, X1 - X0)))

    # -- 目标 ------------------------------------------------------------
    target = frame.target
    if target is not None:
        X, Y = pt(target.x, target.y)
        if target.shape == "circle":
            r = 0.5 * (extent(target.x, target.y, target.radius or 0.0, 0.0)
                       + extent(target.x, target.y, target.radius or 0.0, math.pi / 2))
            target = replace(target, x=X, y=Y, radius=r)
        else:
            (X2, Y2), hw, hh, _ = _rect_to_world(
                calib, target.x, target.y,
                target.half_w or 0.0, target.half_h or 0.0, 0.0)
            target = replace(target, x=X2, y=Y2, half_w=hw, half_h=hh)

    return replace(
        frame,
        units=DEFAULT_WORLD_UNITS,
        metres_per_unit=1.0,
        obstacles=obstacles,
        gaps=gaps,
        player=player,
        target=target,
        diagnostics={**(frame.diagnostics or {}),
                     "calib_id": calib.marker,
                     "calib_rms_px": calib.rms_px,
                     "H_field_from_image": list(calib.H)},
    )


def _world_extent(calib: FieldCalibration, cx: float, cy: float,
                  half: float, theta: float) -> float:
    """沿图像方向 ``theta``、以 ``(cx, cy)`` 为中心、半长 ``half``（像素）的**世界半长**。

    取「中心 ∓ 两端」的中点距离再除以 2（**对称**测量）。
    为什么必须对称：透视下两侧的尺度不一样，只量一侧会系统偏大或偏小。
    """
    dx, dy = math.cos(theta), math.sin(theta)
    X0, Y0 = calib.image_to_field(cx - half * dx, cy - half * dy)
    X1, Y1 = calib.image_to_field(cx + half * dx, cy + half * dy)
    return calib.unit_to_metres(math.hypot(X1 - X0, Y1 - Y0)) / 2.0


def _image_corners(cx: float, cy: float, hw: float, hh: float,
                   theta: float) -> list[Point]:
    """图像的旋转矩形 → 4 个角点（顺序：−w−h, +w−h, +w+h, −w+h）。"""
    dx, dy = math.cos(theta), math.sin(theta)
    nx, ny = -dy, dx
    return [(cx + sx * hw * dx + sy * hh * nx, cy + sx * hw * dy + sy * hh * ny)
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def _rect_to_world(calib: FieldCalibration, cx: float, cy: float,
                   hw: float, hh: float, theta: float
                   ) -> tuple[Point, float, float, float]:
    """旋转矩形：**把 4 个角点各自映射过去**，再从世界四角反推中心/半尺寸/朝向。

    为什么不能只映射中心、再把 ``half_w`` 按某个比例缩放：透视下**世界坐标里轴对齐的
    矩形，在图像里并不是轴对齐的**（有剪切），所以"图像长边方向"与"世界长边方向"
    不是同一个角，图像里近似垂直的两条边在世界里也不垂直。把 4 个角点都映射过去，
    这些误差自然消失。

    返回 ``(世界中心, 世界 half_w, 世界 half_h, 世界朝向)``。
    """
    w = [calib.image_to_field(*p) for p in _image_corners(cx, cy, hw, hh, theta)]
    Xc = sum(p[0] for p in w) / 4.0
    Yc = sum(p[1] for p in w) / 4.0

    # 长轴方向：两条长边的方向向量相加（对称，抗噪）
    ax = (w[1][0] - w[0][0]) + (w[2][0] - w[3][0])
    ay = (w[1][1] - w[0][1]) + (w[2][1] - w[3][1])
    rot = math.atan2(ay, ax)
    norm = math.hypot(ax, ay)
    if norm < 1e-12:
        raise ValueError("矩形退化：长边长度为零")
    # half_w = 沿长轴方向的半长（把两条长边投影到长轴上取平均）
    proj0 = abs((w[1][0] - w[0][0]) * ax + (w[1][1] - w[0][1]) * ay) / norm
    proj1 = abs((w[2][0] - w[3][0]) * ax + (w[2][1] - w[3][1]) * ay) / norm
    half_w = calib.unit_to_metres((proj0 + proj1) / 4.0)

    # half_h = 两条长边之间的**垂直距离**的一半 —— 不是短边的长度！
    # 透视下短边映射到世界后是**斜的**，直接用它的长度会把"厚度"算大
    # （实测强透视下能大 30%+）。取垂直距离才是真正的墙厚。
    nx, ny = -ay / norm, ax / norm
    # 两条长边的中点
    m0 = ((w[0][0] + w[1][0]) / 2.0, (w[0][1] + w[1][1]) / 2.0)
    m1 = ((w[3][0] + w[2][0]) / 2.0, (w[3][1] + w[2][1]) / 2.0)
    half_h = calib.unit_to_metres(abs((m1[0] - m0[0]) * nx + (m1[1] - m0[1]) * ny) / 2.0)
    return (Xc, Yc), half_w, half_h, rot


__all__ = [
    "DEFAULT_WORLD_UNITS",
    "GroundCamera", "known_size_depth", "depth_error",
    "points_to_world", "distance", "length_scale", "to_world",
]
