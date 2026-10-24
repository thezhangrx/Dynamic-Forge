"""规范二 · 障碍（Obstacle）—— Python 定义。

**纯 stdlib**（`dataclasses` + `enum` + `math`）。

核心约定（完整理由见同目录 ``obstacle.md``）：

* **一个障碍 = 一个圆 或 一个旋转矩形**，没有第三种原语；
* 矩形不变量 **``half_w >= half_h``**（长边恒为 ``half_w`` 轴）；圆形约定
  ``half_w = half_h = radius``，同一套 clearance 代码通用；
* ``angle`` 是**速度方向**、``rotation`` 是**形状朝向**，两者相差 90°；
* 缺口**不是**实体：``wall_with_gap`` 就是两个矩形（同 ``group_id``），
  缺口由规范三的 ``GapObs`` 以"关系"的形式发布。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import cos, inf, isfinite, sin
from typing import Any, Mapping, Sequence

#: 本规范版本，与 obstacle.json 的 ``x-standard-version`` 一致。
STANDARD_VERSION = 1

#: 默认坐标系：场地世界系（原点在场地左下，+x 向右，+y 向上）。
DEFAULT_FRAME_ID = "field"


# --------------------------------------------------------------------------
# 枚举与编码
# --------------------------------------------------------------------------
class ObstacleShape(str, Enum):
    """形状原语。线协议编码见 :data:`SHAPE_CODES`。"""

    CIRCLE = "circle"
    RECT = "rect"


#: 与平台 ``SHAPE_CODES`` 一致：``{"circle":0,"disc":0,"rect":1,"box":1}``。
SHAPE_CODES: dict[str, int] = {"circle": 0, "rect": 1}


class ObstacleType(str, Enum):
    """5 种现实障碍类型（平台的 ``obstacles/catalog.py`` 就是这 5 个 key）。"""

    MOVING_BLOCK = "moving_block"
    WALL_WITH_GAP = "wall_with_gap"
    SMALL_OBSTACLES = "small_obstacles"
    CORRIDOR = "corridor"
    CROSS_TRAFFIC = "cross_traffic"


#: ``type`` ↔ 线协议 ``type_id``。**只追加、不重排。**
#:
#: 注意：平台当前实现把 ``PatternSpec.type_id`` 默认写成 0（自由标签，不是类型编码），
#: 所以这张表是本项目**权威**的类型编码；落地时需要平台显式设置 ``type_id``。
TYPE_CODES: dict[ObstacleType | None, int] = {
    None: 0,
    ObstacleType.MOVING_BLOCK: 1,
    ObstacleType.WALL_WITH_GAP: 2,
    ObstacleType.SMALL_OBSTACLES: 3,
    ObstacleType.CORRIDOR: 4,
    ObstacleType.CROSS_TRAFFIC: 5,
}

#: 反向表：``type_id`` → ``type``（含未分类 0）。
TYPE_NAMES: dict[int, ObstacleType | None] = {v: k for k, v in TYPE_CODES.items()}

#: BHL1 v3 bullet 块的字段顺序（stride = 17 × float32 = 68 字节）。
BHL1_BULLET_FIELDS: tuple[str, ...] = (
    "x", "y", "vx", "vy", "ax", "ay", "angle", "angular_velocity", "radius",
    "ttl", "type_id", "group_id", "id", "shape", "half_w", "half_h", "rotation",
)


# --------------------------------------------------------------------------
@dataclass
class Obstacle:
    """一个动态障碍。

    必填只有 ``id`` / ``shape`` / ``x`` / ``y``；几何细节按 ``shape`` 二选一：
    圆形给 ``radius``，矩形给 ``half_w`` + ``half_h``。
    """

    # -- 身份与几何（必填） ----------------------------------------------
    id: int
    shape: ObstacleShape
    x: float
    y: float

    # -- 类型 ------------------------------------------------------------
    type: ObstacleType | None = None
    type_id: int | None = None
    frame_id: str = DEFAULT_FRAME_ID

    # -- 运动 ------------------------------------------------------------
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    #: 速度方向（弧度）。线协议里有意冗余，必须与 (vx,vy) 同步。
    angle: float | None = None
    #: 矩形 = 自转；圆 = 速度方向旋转率。弧度/秒。
    angular_velocity: float = 0.0

    # -- 形状尺寸 --------------------------------------------------------
    radius: float | None = None
    half_w: float | None = None
    half_h: float | None = None
    rotation: float = 0.0

    # -- 生命周期与分组 --------------------------------------------------
    ttl: float | None = None
    age: float | None = None
    group_id: int | None = None
    alive: bool = True

    # -- 观测质量 --------------------------------------------------------
    valid: bool = True
    occluded: bool = False
    score: float | None = None
    cov: tuple[float, float, float] | None = None

    # ------------------------------------------------------------------
    # 派生量
    # ------------------------------------------------------------------
    @property
    def is_circle(self) -> bool:
        return self.shape == ObstacleShape.CIRCLE

    @property
    def is_rect(self) -> bool:
        return self.shape == ObstacleShape.RECT

    @property
    def extent_w(self) -> float:
        """外侧常说的"宽"（总尺寸）；结构里只存半尺寸。"""
        return 2.0 * (self.radius if self.is_circle else self.half_w)  # type: ignore[operator]

    @property
    def extent_h(self) -> float:
        return 2.0 * (self.radius if self.is_circle else self.half_h)  # type: ignore[operator]

    def corners(self) -> tuple[tuple[float, float], ...]:
        """矩形四角（世界系）；圆形抛 ``ValueError``。

        ``θ = rotation`` 是长边方向：``中心 ± half_w·(cosθ,sinθ) ± half_h·(−sinθ,cosθ)``
        """
        if not self.is_rect:
            raise ValueError("corners() 只对矩形有意义；圆形请用 radius")
        assert self.half_w is not None and self.half_h is not None
        dx, dy = cos(self.rotation), sin(self.rotation)
        nx, ny = -dy, dx
        hw, hh = self.half_w, self.half_h
        return tuple(
            (self.x + sx * hw * dx + sy * hh * nx, self.y + sx * hw * dy + sy * hh * ny)
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        )

    def angle_matches_velocity(self, tol: float = 1e-3) -> bool:
        """校验 ``angle`` 与 ``(vx, vy)`` 是否同步（相差 ±π 的整倍视为等价）。

        默认容差 1e-3（约 0.06°）而不是 1e-6：JSON 里的角度通常只保留 4 位小数，
        而 float32 线格式本身也只有约 1e-7 的相对精度。想在线协议上做严格校验
        可以显式传更小的 ``tol``。
        """
        if self.angle is None or (self.vx == 0.0 and self.vy == 0.0):
            return True
        from math import atan2, pi
        delta = (self.angle - atan2(self.vy, self.vx)) % (2.0 * pi)
        return min(delta, 2.0 * pi - delta) <= tol

    # ------------------------------------------------------------------
    def validate(self) -> None:
        """就地校验；不合法抛 ``ValueError``。"""
        if self.id < 0:
            raise ValueError(f"id 不能为负：{self.id!r}")
        if self.is_circle:
            if self.radius is None or self.radius <= 0.0:
                raise ValueError(f"shape=circle 时必须给正的 radius，得到 {self.radius!r}")
        else:
            if self.half_w is None or self.half_h is None:
                raise ValueError("shape=rect 时必须给 half_w 与 half_h")
            if self.half_w <= 0.0 or self.half_h <= 0.0:
                raise ValueError(f"half_w/half_h 必须为正：{self.half_w!r}, {self.half_h!r}")
            if self.half_w < self.half_h:
                raise ValueError(
                    f"不变量被破坏：half_w >= half_h 恒成立（长边是 half_w 轴），"
                    f"得到 half_w={self.half_w!r} < half_h={self.half_h!r}"
                )
        if self.ttl is not None and not (isfinite(self.ttl) or self.ttl == inf) :
            raise ValueError(f"ttl 必须为正或 inf：{self.ttl!r}")
        if self.ttl is not None and isfinite(self.ttl) and self.ttl <= 0.0:
            raise ValueError(f"ttl 必须 > 0：{self.ttl!r}")
        if self.score is not None and not (0.0 <= self.score <= 1.0):
            raise ValueError(f"score 必须在 [0, 1]：{self.score!r}")
        if self.cov is not None and len(self.cov) != 3:
            raise ValueError(f"cov 必须是 [σxx, σxy, σyy]，得到 {self.cov!r}")
        if self.type_id is not None and self.type is not None:
            want = TYPE_CODES[self.type]
            if self.type_id != want:
                raise ValueError(
                    f"type_id 与 type 不一致：type={self.type.value!r} 应为 "
                    f"{want}，得到 {self.type_id!r}（编码表见 obstacle.md §3.5）"
                )
        if not self.angle_matches_velocity():
            raise ValueError(
                "angle 与 (vx,vy) 不同步：angle 是速度方向的冗余存储，"
                "必须由速度同步而来、不得独立修改"
            )

    # ------------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "frame_id": self.frame_id,
            "type": self.type.value if self.type is not None else None,
            "type_id": self.type_id,
            "shape": self.shape.value,
            "x": self.x, "y": self.y,
            "vx": self.vx, "vy": self.vy, "ax": self.ax, "ay": self.ay,
            "radius": self.radius,
            "half_w": self.half_w, "half_h": self.half_h,
            "rotation": self.rotation, "angle": self.angle,
            "angular_velocity": self.angular_velocity,
            "ttl": self.ttl, "age": self.age,
            "group_id": self.group_id, "alive": self.alive,
            "valid": self.valid, "occluded": self.occluded,
            "score": self.score,
            "cov": list(self.cov) if self.cov is not None else None,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Obstacle":
        cov = data.get("cov")
        typ = data.get("type")
        return cls(
            id=int(data["id"]),
            shape=ObstacleShape(data["shape"]),
            x=float(data["x"]),
            y=float(data["y"]),
            type=None if typ is None else ObstacleType(typ),
            type_id=None if data.get("type_id") is None else int(data["type_id"]),
            frame_id=str(data.get("frame_id", DEFAULT_FRAME_ID)),
            vx=float(data.get("vx", 0.0)), vy=float(data.get("vy", 0.0)),
            ax=float(data.get("ax", 0.0)), ay=float(data.get("ay", 0.0)),
            angle=None if data.get("angle") is None else float(data["angle"]),
            angular_velocity=float(data.get("angular_velocity", 0.0)),
            radius=None if data.get("radius") is None else float(data["radius"]),
            half_w=None if data.get("half_w") is None else float(data["half_w"]),
            half_h=None if data.get("half_h") is None else float(data["half_h"]),
            rotation=float(data.get("rotation", 0.0)),
            ttl=None if data.get("ttl") is None else float(data["ttl"]),
            age=None if data.get("age") is None else float(data["age"]),
            group_id=None if data.get("group_id") is None else int(data["group_id"]),
            alive=bool(data.get("alive", True)),
            valid=bool(data.get("valid", True)),
            occluded=bool(data.get("occluded", False)),
            score=None if data.get("score") is None else float(data["score"]),
            cov=None if cov is None else (float(cov[0]), float(cov[1]), float(cov[2])),
        )

    # ------------------------------------------------------------------
    # 与 BHL1 v3 线格式互转（stride 17 × float32）
    # ------------------------------------------------------------------
    def to_bhl1_bullet(self) -> tuple[float, ...]:
        """按 ``BHL1_BULLET_FIELDS`` 顺序导出 17 个值。

        矩形会用 ``radius`` 列回填 ``half_w``（平台对圆的约定反过来：
        圆时 ``half_w = half_h = radius``）。这里为保持列语义直白，
        矩形把 ``radius`` 写 0，圆的 ``half_w/half_h`` 写 ``radius``。
        """
        r = self.radius if self.is_circle else 0.0
        hw = self.half_w if self.is_rect else self.radius
        hh = self.half_h if self.is_rect else self.radius
        return (
            self.x, self.y, self.vx, self.vy, self.ax, self.ay,
            self.angle if self.angle is not None else 0.0,
            self.angular_velocity,
            float(r),
            float(self.ttl) if self.ttl is not None else inf,
            float(self.type_id if self.type_id is not None else TYPE_CODES[self.type]),
            float(self.group_id if self.group_id is not None else 0),
            float(self.id),
            float(SHAPE_CODES[self.shape.value]),
            float(hw if hw is not None else 0.0),
            float(hh if hh is not None else 0.0),
            self.rotation,
        )

    @classmethod
    def from_bhl1_bullet(cls, values: Sequence[float], **extra: Any) -> "Obstacle":
        if len(values) != len(BHL1_BULLET_FIELDS):
            raise ValueError(f"BHL1 bullet 块应为 {len(BHL1_BULLET_FIELDS)} 个值，"
                             f"得到 {len(values)}")
        (x, y, vx, vy, ax, ay, angle, ang_vel, radius, ttl,
         type_id, group_id, oid, shape, half_w, half_h, rotation) = (float(v) for v in values)
        is_circle = int(shape) == SHAPE_CODES["circle"]
        tid = int(type_id)
        return cls(
            id=int(oid), shape=ObstacleShape("circle" if is_circle else "rect"),
            x=x, y=y, vx=vx, vy=vy, ax=ax, ay=ay, angle=angle,
            angular_velocity=ang_vel,
            radius=(radius if is_circle else None),
            half_w=(None if is_circle else half_w),
            half_h=(None if is_circle else half_h),
            rotation=rotation, ttl=ttl, type_id=tid,
            type=TYPE_NAMES.get(tid), group_id=int(group_id),
            **extra,
        )


__all__ = [
    "STANDARD_VERSION", "DEFAULT_FRAME_ID",
    "ObstacleShape", "SHAPE_CODES",
    "ObstacleType", "TYPE_CODES", "TYPE_NAMES",
    "BHL1_BULLET_FIELDS", "Obstacle",
]
