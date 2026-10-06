"""规范三 · 视觉输出（VisionFrame）—— Python 定义。

**纯 stdlib**。载荷里的 ``player`` / ``obstacles`` 直接复用规范一与规范二的定义
（相对导入），本模块只补上「时间、坐标系、单位、标定、观测质量」这些属于**观测**
而不是**实体**的信息。

核心约定（完整理由见同目录 ``vision_output.md``）：

* 一条消息**只有一个坐标系**（``frame_id``，默认场地世界系），消费者不得自行推断；
* ``units`` 必须显式（``"px"`` / ``"m"``），标定前一律 ``"px"``；
* ``stamp`` 是**采集时刻**，测速只用它；``seq`` 只用于去重；
* **缺口是派生关系**（``GapObs`` + ``blockers``），不是障碍实体；
* 视觉输出里 ``Obstacle.id`` 就是**跟踪 ID**，与平台真值 ``id`` 不同空间。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Any, Mapping, Sequence

from ..obstacle.obstacle import Obstacle
from ..player.player import DEFAULT_FRAME_ID, PlayerState

#: 本规范版本，与 vision_output.json 的 ``x-standard-version`` 一致。
STANDARD_VERSION = 1


class Units(str, Enum):
    """载荷单位。标定完成前必须是 ``PX``。"""

    PX = "px"
    M = "m"


# --------------------------------------------------------------------------
@dataclass
class Calibration:
    """用的是哪次标定，以及把像素变成世界坐标的那个映射。

    ``H_field_from_image`` 是**图像 → 场地**的 3×3 单应（行主序）。
    只发布这一条映射，不发布 ``P``（双目校正的产物），避免 ``K``/``P`` 混用那类 bug。
    """

    id: str
    K: tuple[float, ...] | None = None
    D: tuple[float, ...] | None = None
    H_field_from_image: tuple[float, ...] | None = None
    rms_reprojection_px: float | None = None

    def validate(self) -> None:
        for name in ("K", "H_field_from_image"):
            v = getattr(self, name)
            if v is not None and len(v) != 9:
                raise ValueError(f"{name} 必须是 3x3 行主序的 9 个数，得到 {v!r}")
        if self.rms_reprojection_px is not None and self.rms_reprojection_px < 0:
            raise ValueError(f"rms_reprojection_px 不能为负：{self.rms_reprojection_px!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "K": list(self.K) if self.K is not None else None,
            "D": list(self.D) if self.D is not None else None,
            "H_field_from_image": (list(self.H_field_from_image)
                                   if self.H_field_from_image is not None else None),
            "rms_reprojection_px": self.rms_reprojection_px,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Calibration":
        def tup(key: str) -> tuple[float, ...] | None:
            v = data.get(key)
            return None if v is None else tuple(float(x) for x in v)

        return cls(id=str(data["id"]), K=tup("K"), D=tup("D"),
                   H_field_from_image=tup("H_field_from_image"),
                   rms_reprojection_px=data.get("rms_reprojection_px"))


# --------------------------------------------------------------------------
@dataclass
class GapObs:
    """缺口观测——**派生关系**，不是障碍实体。

    ``width`` 与角色半径同单位，因此可以直接与 ``2 * player.radius`` 比较
    （平台的硬约束：缺口不得小于玩家圆直径）。
    """

    id: int
    center: tuple[float, float]
    width: float
    blockers: tuple[int, int]
    axis: float = 0.0
    blocked_by_type: str | None = "wall_with_gap"
    reliable: bool = True
    occluded: bool = False
    score: float | None = None

    def validate(self) -> None:
        if self.width <= 0.0:
            raise ValueError(f"缺口宽度必须 > 0：{self.width!r}")
        if len(self.blockers) != 2:
            raise ValueError(f"blockers 必须是构成缺口的两段墙 id，得到 {self.blockers!r}")
        if self.blockers[0] == self.blockers[1]:
            raise ValueError(f"blockers 的两段墙不能是同一条：{self.blockers!r}")
        if self.occluded and self.reliable:
            raise ValueError("边界被遮挡时 reliable 必须为 False（宽度是推出来的，不是看到的）")

    def fits(self, player_radius: float) -> bool:
        """平台硬约束：缺口宽度是否容得下玩家圆直径。"""
        return self.width >= 2.0 * player_radius

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id, "center": list(self.center), "width": self.width,
            "axis": self.axis, "blockers": list(self.blockers),
            "blocked_by_type": self.blocked_by_type,
            "reliable": self.reliable, "occluded": self.occluded, "score": self.score,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "GapObs":
        return cls(
            id=int(data["id"]),
            center=(float(data["center"][0]), float(data["center"][1])),
            width=float(data["width"]),
            blockers=(int(data["blockers"][0]), int(data["blockers"][1])),
            axis=float(data.get("axis", 0.0)),
            blocked_by_type=data.get("blocked_by_type", "wall_with_gap"),
            reliable=bool(data.get("reliable", True)),
            occluded=bool(data.get("occluded", False)),
            score=None if data.get("score") is None else float(data["score"]),
        )


# --------------------------------------------------------------------------
@dataclass
class TargetObs:
    """目标区域观测；字段与平台 ``target f32[6]`` 对齐，可直写回 State 帧。"""

    x: float
    y: float
    shape: str = "circle"
    radius: float | None = None
    half_w: float | None = None
    half_h: float | None = None
    score: float | None = None
    valid: bool = True
    occluded: bool = False

    def validate(self) -> None:
        if self.shape not in ("circle", "rect"):
            raise ValueError(f"shape 只能是 circle/rect，得到 {self.shape!r}")
        if self.shape == "circle" and self.radius is None:
            raise ValueError("shape=circle 时必须给 radius")
        if self.shape == "rect" and (self.half_w is None or self.half_h is None):
            raise ValueError("shape=rect 时必须给 half_w 与 half_h")
        if self.shape == "rect" and self.half_w is not None and self.half_h is not None \
                and self.half_w < self.half_h:
            raise ValueError("不变量 half_w >= half_h 被破坏")

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x, "y": self.y, "shape": self.shape, "radius": self.radius,
            "half_w": self.half_w, "half_h": self.half_h,
            "score": self.score, "valid": self.valid, "occluded": self.occluded,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TargetObs":
        return cls(
            x=float(data["x"]), y=float(data["y"]),
            shape=str(data.get("shape", "circle")),
            radius=None if data.get("radius") is None else float(data["radius"]),
            half_w=None if data.get("half_w") is None else float(data["half_w"]),
            half_h=None if data.get("half_h") is None else float(data["half_h"]),
            score=None if data.get("score") is None else float(data["score"]),
            valid=bool(data.get("valid", True)),
            occluded=bool(data.get("occluded", False)),
        )


# --------------------------------------------------------------------------
@dataclass
class VisionFrame:
    """一帧视觉输出。"""

    seq: int
    stamp: float
    frame_id: str = DEFAULT_FRAME_ID
    units: Units = Units.PX
    camera_frame_id: str | None = "camera"
    metres_per_unit: float | None = None
    field_origin: tuple[float, float] | None = None
    image_width: int | None = None
    image_height: int | None = None
    calibration: Calibration | None = None
    player: PlayerState | None = None
    obstacles: list[Obstacle] = field(default_factory=list)
    gaps: list[GapObs] = field(default_factory=list)
    target: TargetObs | None = None
    diagnostics: dict[str, Any] | None = None
    standard_version: int = STANDARD_VERSION

    # ------------------------------------------------------------------
    def validate(self) -> None:
        """整帧校验；不合法抛 ``ValueError``。"""
        if self.standard_version != STANDARD_VERSION:
            raise ValueError(f"standard_version 应为 {STANDARD_VERSION}，"
                             f"得到 {self.standard_version!r}")
        if self.seq < 0:
            raise ValueError(f"seq 不能为负：{self.seq!r}")
        if self.units == Units.M and self.metres_per_unit not in (None, 1.0):
            raise ValueError(
                f"units='m' 时 metres_per_unit 必须是 1.0 或 None，得到 {self.metres_per_unit!r}"
            )
        if self.calibration is not None:
            self.calibration.validate()
        if self.player is not None:
            self.player.validate()
            if self.player.frame_id != self.frame_id:
                raise ValueError(
                    f"player.frame_id={self.player.frame_id!r} 与帧头 "
                    f"frame_id={self.frame_id!r} 不一致（一条消息只能有一个坐标系）"
                )
        seen: set[int] = set()
        for ob in self.obstacles:
            ob.validate()
            if ob.frame_id != self.frame_id:
                raise ValueError(
                    f"obstacle id={ob.id} 的 frame_id={ob.frame_id!r} 与帧头 "
                    f"frame_id={self.frame_id!r} 不一致"
                )
            if ob.id in seen:
                raise ValueError(f"同一帧里出现重复的 obstacle id={ob.id}")
            seen.add(ob.id)
        for gap in self.gaps:
            gap.validate()
            missing = [b for b in gap.blockers if b not in seen]
            if missing:
                raise ValueError(
                    f"缺口 id={gap.id} 引用了不存在的墙段 id={missing}（blockers 必须指回本帧的障碍）"
                )
        if self.target is not None:
            self.target.validate()

    # ------------------------------------------------------------------
    @property
    def n_obstacles(self) -> int:
        return len(self.obstacles)

    def obstacle_by_id(self, oid: int) -> Obstacle | None:
        return next((o for o in self.obstacles if o.id == oid), None)

    def gap_fits_player(self, gap: GapObs) -> bool | None:
        """缺口是否容得下角色；没有角色观测时返回 ``None``。"""
        return None if self.player is None else gap.fits(self.player.radius)

    def dt_since(self, earlier: "VisionFrame") -> float:
        """两帧之间的真实时间间隔（秒）——**用 stamp 而不是 seq**。"""
        dt = self.stamp - earlier.stamp
        if dt <= 0.0 or not isfinite(dt):
            raise ValueError(
                f"非法时间间隔 dt={dt!r}（stamp 必须严格递增；"
                "若要退化用帧号，必须另行保证帧率恒定）"
            )
        return dt

    # ------------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {
            "standard_version": self.standard_version,
            "seq": self.seq,
            "stamp": self.stamp,
            "frame_id": self.frame_id,
            "camera_frame_id": self.camera_frame_id,
            "units": self.units.value,
            "metres_per_unit": self.metres_per_unit,
            "field_origin": list(self.field_origin) if self.field_origin else None,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "calibration": self.calibration.to_dict() if self.calibration else None,
            "player": self.player.to_dict() if self.player else None,
            "obstacles": [o.to_dict() for o in self.obstacles],
            "gaps": [g.to_dict() for g in self.gaps],
            "target": self.target.to_dict() if self.target else None,
            "diagnostics": self.diagnostics,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "VisionFrame":
        cal = data.get("calibration")
        tgt = data.get("target")
        return cls(
            seq=int(data["seq"]),
            stamp=float(data["stamp"]),
            frame_id=str(data.get("frame_id", DEFAULT_FRAME_ID)),
            units=Units(data.get("units", "px")),
            camera_frame_id=data.get("camera_frame_id", "camera"),
            metres_per_unit=None if data.get("metres_per_unit") is None
            else float(data["metres_per_unit"]),
            field_origin=None if data.get("field_origin") is None
            else (float(data["field_origin"][0]), float(data["field_origin"][1])),
            image_width=data.get("image_width"),
            image_height=data.get("image_height"),
            calibration=None if cal is None else Calibration.from_dict(cal),
            player=None if data.get("player") is None
            else PlayerState.from_dict(data["player"]),
            obstacles=[Obstacle.from_dict(o) for o in data.get("obstacles", [])],
            gaps=[GapObs.from_dict(g) for g in data.get("gaps", [])],
            target=None if tgt is None else TargetObs.from_dict(tgt),
            diagnostics=data.get("diagnostics"),
            standard_version=int(data.get("standard_version", STANDARD_VERSION)),
        )


__all__ = [
    "STANDARD_VERSION", "Units", "Calibration", "GapObs", "TargetObs", "VisionFrame",
]
