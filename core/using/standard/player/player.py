"""规范一 · 角色（Player）—— Python 定义。

**纯 stdlib**（`dataclasses` + `math`），不引入第三方依赖：这份定义要同时给
视觉、CPU 决策、FPGA 侧（通过线协议）共用，谁都不该为了用数据结构而装库。

核心约定（完整理由见同目录 ``player.md``）：

* **角色就是一个圆**：``radius`` 既是碰撞体也是画面大小，没有第二套尺寸；
* ``speed`` 是**运动学上限**，瞬时速率请用派生属性 ``speed_measured``；
* 位置与速度都在 ``frame_id`` 指定的坐标系里（默认 ``"field"`` 场地世界系）；
* 测速只用 ``stamp`` 之差；``seq`` 只用于去重。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot
from typing import Any, Mapping, Sequence

#: 本规范版本，与 player.json 的 ``x-standard-version`` 一致。
STANDARD_VERSION = 1

#: 默认坐标系：场地世界系（原点在场地左下，+x 向右，+y 向上）。
DEFAULT_FRAME_ID = "field"

#: BHL1 v3 ``player`` 块的字段顺序（帧头 32..60 字节，``f32[7]``）。
BHL1_PLAYER_FIELDS: tuple[str, ...] = (
    "x", "y", "vx", "vy", "radius", "speed", "alive",
)


@dataclass
class PlayerState:
    """一个角色的完整状态。

    只有 ``x``/``y``/``radius`` 是必填——它们是"看到/知道这个角色存在"的最小信息；
    速度、置信度、协方差都是可选的，缺省表示"不知道"，而不是"是 0"。
    """

    # -- 几何（必填） ----------------------------------------------------
    x: float
    y: float
    radius: float

    # -- 身份 / 时间 / 参考系 --------------------------------------------
    id: str = "player"
    frame_id: str = DEFAULT_FRAME_ID
    stamp: float | None = None
    seq: int | None = None

    # -- 运动 ------------------------------------------------------------
    vx: float = 0.0
    vy: float = 0.0
    #: 运动学速度**上限**（平台 ScenarioSpec.player_speed），不是瞬时速率。
    speed: float = 0.0
    alive: bool = True

    # -- 观测质量 --------------------------------------------------------
    valid: bool = True
    occluded: bool = False
    score: float | None = None
    #: [σxx, σxy, σyy]；ROS 用 6x6=36 个 float64，这里只留 2D 的独立分量。
    pose_cov: tuple[float, float, float] | None = None
    vel_cov: tuple[float, float, float] | None = None
    is_stationary: bool | None = None

    # ------------------------------------------------------------------
    # 派生量：能算的就不存（避免两个真值源）
    # ------------------------------------------------------------------
    @property
    def heading(self) -> float:
        """速度方向（弧度）。静止时返回 0.0。"""
        return 0.0 if (self.vx == 0.0 and self.vy == 0.0) else atan2(self.vy, self.vx)

    @property
    def speed_measured(self) -> float:
        """瞬时速率 ``hypot(vx, vy)``——**不要**和 ``speed``（上限）混用。"""
        return hypot(self.vx, self.vy)

    @property
    def diameter(self) -> float:
        """平台所有间距（缺口宽度、走廊宽度）都以它为单位。"""
        return 2.0 * self.radius

    # ------------------------------------------------------------------
    def validate(self) -> None:
        """就地校验；不合法抛 ``ValueError``。"""
        if self.radius <= 0.0:
            raise ValueError(f"radius 必须 > 0（角色是圆），得到 {self.radius!r}")
        if self.speed < 0.0:
            raise ValueError(f"speed 是速度上限，不能为负：{self.speed!r}")
        if self.score is not None and not (0.0 <= self.score <= 1.0):
            raise ValueError(f"score 必须在 [0, 1]：{self.score!r}")
        for name in ("pose_cov", "vel_cov"):
            v = getattr(self, name)
            if v is not None and len(v) != 3:
                raise ValueError(f"{name} 必须是 [σxx, σxy, σyy]，得到 {v!r}")
        if self.speed_measured > self.speed + 1e-9:
            raise ValueError(
                f"瞬时速率 {self.speed_measured:.3f} 超过上限 speed={self.speed:.3f}；"
                "若 speed 是瞬时速度，说明字段用错了（见 player.md §3.2）"
            )

    # ------------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        """JSON 友好；元组转列表。"""
        d = {
            "id": self.id,
            "frame_id": self.frame_id,
            "stamp": self.stamp,
            "seq": self.seq,
            "x": self.x,
            "y": self.y,
            "radius": self.radius,
            "vx": self.vx,
            "vy": self.vy,
            "speed": self.speed,
            "alive": self.alive,
            "valid": self.valid,
            "occluded": self.occluded,
            "score": self.score,
            "pose_cov": list(self.pose_cov) if self.pose_cov is not None else None,
            "vel_cov": list(self.vel_cov) if self.vel_cov is not None else None,
            "is_stationary": self.is_stationary,
        }
        return d

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PlayerState":
        def cov(key: str) -> tuple[float, float, float] | None:
            v = data.get(key)
            return None if v is None else (float(v[0]), float(v[1]), float(v[2]))

        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            radius=float(data["radius"]),
            id=str(data.get("id", "player")),
            frame_id=str(data.get("frame_id", DEFAULT_FRAME_ID)),
            stamp=None if data.get("stamp") is None else float(data["stamp"]),
            seq=None if data.get("seq") is None else int(data["seq"]),
            vx=float(data.get("vx", 0.0)),
            vy=float(data.get("vy", 0.0)),
            speed=float(data.get("speed", 0.0)),
            alive=bool(data.get("alive", True)),
            valid=bool(data.get("valid", True)),
            occluded=bool(data.get("occluded", False)),
            score=None if data.get("score") is None else float(data["score"]),
            pose_cov=cov("pose_cov"),
            vel_cov=cov("vel_cov"),
            is_stationary=data.get("is_stationary"),
        )

    # ------------------------------------------------------------------
    # 与 BHL1 v3 线格式互转（帧头 32..60 字节的 f32[7]）
    # ------------------------------------------------------------------
    def to_bhl1_player(self) -> tuple[float, ...]:
        """按 ``BHL1_PLAYER_FIELDS`` 顺序导出 7 个 float32 值。"""
        return (self.x, self.y, self.vx, self.vy, self.radius, self.speed,
                1.0 if self.alive else 0.0)

    @classmethod
    def from_bhl1_player(cls, values: Sequence[float], **extra: Any) -> "PlayerState":
        """从线格式的 7 个值还原；``stamp``/``frame_id`` 等头信息用 ``extra`` 补。"""
        if len(values) != len(BHL1_PLAYER_FIELDS):
            raise ValueError(f"BHL1 player 块应为 {len(BHL1_PLAYER_FIELDS)} 个值，"
                             f"得到 {len(values)}")
        x, y, vx, vy, radius, speed, alive = (float(v) for v in values)
        return cls(x=x, y=y, vx=vx, vy=vy, radius=radius, speed=speed,
                   alive=bool(alive), **extra)


__all__ = ["STANDARD_VERSION", "DEFAULT_FRAME_ID", "BHL1_PLAYER_FIELDS", "PlayerState"]
