"""视觉 → 决策 的 IO 接口 —— 把 ``VisionFrame`` 变成 ``plan()`` 的输入。

    core/vision/  ──识别──▶  VisionFrame(世界坐标)  ──本文件──▶  WorldView / Rect
                                                                      │
                                                                      ▼
                                                          cpu.gap_avoid.plan()

为什么需要它
------------
``gap_avoid.py`` 的数据来源原来只有两个，**都是仿真器专用**：
``rect_from_pool``（读 bullet_sim 的 SoA 池）与 ``view_from_world``（包 world）。
比赛要的是"摄像头 → 图像处理 → 决策 → 小车"，中间**没有连接代码**。
本文件就是那一段连接。

三条硬性约束
------------
1. **不碰摄像头**。本模块是纯数据转换，不 import cv2、不 import vision 的实现、
   不打开任何设备。摄像头不可用时它就是一堆可以用合成帧测的纯函数。
2. **不 import ``standard`` 包**，用鸭子类型读字段（``getattr`` / ``Mapping``）。
   于是它既能吃 ``standard.vision.VisionFrame``，也能吃**任何同名字段的对象或 dict**，
   同时保持"依赖小"（只有 stdlib）。这也避免决策层反向依赖契约包。
3. **不静默兜底**。所有"猜"的地方要么显式抛错、要么走**显式配置的默认值**，
   并把"用了默认值"记在 :class:`VisionMeta` 里。这是 ``safety/``
   "宁可误判不可行"的同一条纪律——坐标错了却继续跑，是最难查的一类 bug。

坐标系与单位（这两条错了，后面所有判据都错）
--------------------------------------------
* 坐标系：``frame_id`` 必须是 ``"field"``（原点左下、+x 右、+y 上），
  且帧里的坐标必须**已经是世界坐标**（``core/standard/vision/vision_output.md`` §4.2/§4.8
  规定坐标只有一套，由 ``units`` 决定）。
  ``units="px"`` 有两种可能：① **世界单位就是像素**（合法，需 ``metres_per_unit``）；
  ② 还没标定（**非法**——像素不是世界坐标）。本模块用
  ``require_calibration=True`` 把②挡在门外：没有 ``H_field_from_image`` 且不是
  ``units="m"`` 就抛错。
* 时间：``dt`` 一律用 **``stamp`` 之差**，绝不用 ``seq`` 之差
  （标准 §4.3：三个参考项目全在这里翻车，``ByteTrack`` 把 ``dt`` 写死成 1）。
* 场地尺寸：``VisionFrame`` **不携带** ``field_w/field_h``（标准 §6 明确写
  "视觉侧不产生"），所以由 :class:`VisionWorldConfig` 提供，不能从帧里猜。

用法
----
单帧（自己管历史）：:

    from cpu.adapters.vision_adapter import VisionWorldConfig, frame_to_worldview

    cfg = VisionWorldConfig(field_w=640.0, field_h=480.0)
    view, meta = frame_to_worldview(frame, cfg, prev_frame=prev)

连续帧（推荐，省掉两个经典 bug）：:

    from cpu.adapters.vision_adapter import VisionWorldStream

    stream = VisionWorldStream(VisionWorldConfig())     # 内部持有 GapMemory
    for frame in frames:
        view, dec = stream.step(frame, cfg=AlgoConfig(forward=(0.0, 1.0)))
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence

from cpu.gap_avoid import AlgoConfig, Decision, GapMemory, Rect, WorldView, plan

#: 帧里的世界坐标系名。与 ``core/standard`` 的 ``DEFAULT_FRAME_ID`` 保持一致。
DEFAULT_FRAME_ID = "field"

__all__ = [
    "DEFAULT_FRAME_ID",
    "VisionAdapterError",
    "VisionWorldConfig",
    "VisionMeta",
    "dt_between",
    "player_vector",
    "rects_from_frame",
    "gaps_from_frame",
    "frame_to_worldview",
    "VisionWorldStream",
]


class VisionAdapterError(ValueError):
    """视觉帧不满足决策层的输入契约（坐标系/单位/字段缺失/数值非法）。"""


# ---------------------------------------------------------------------------
# 鸭子类型取值
# ---------------------------------------------------------------------------
def _get(obj: Any, name: str, default: Any = None) -> Any:
    """从 dataclass 或 Mapping 里取字段；两种都支持。"""
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def _shape_is_rect(shape: Any) -> bool:
    """``ObstacleShape.RECT`` / ``"rect"`` / ``1`` 都算矩形。"""
    value = getattr(shape, "value", shape)
    if value == 1:                       # 平台的 SHAPE_CODES: rect == 1
        return True
    return str(value).lower() == "rect"


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class VisionWorldConfig:
    """决策侧对"场地"与"质量门限"的配置。

    这里每一个值都**不在** ``VisionFrame`` 里（标准 §6），
    所以必须由决策侧给出——不能让算法从像素里猜。
    """

    #: 场地尺寸（世界单位）。默认取平台的默认场地。
    field_w: float = 640.0
    field_h: float = 480.0

    #: 期望的坐标系；不符直接抛错（防止拿像素当世界坐标用）。
    frame_id: str = DEFAULT_FRAME_ID

    #: 首帧没有上一帧可比，用这个 ``dt``（秒）。
    dt_default: float = 1.0 / 120.0

    #: ``player.speed`` 是"运动学速度上限"，视觉侧常常不知道 → 用这个兜底。
    #: 平台 ``ScenarioSpec.player_speed`` 默认 200.0。
    default_speed: float = 200.0

    #: 质量门限：``score`` 低于它丢弃；``None`` = 不看 score。
    min_score: Optional[float] = None
    #: 丢不丢被遮挡 / 标为无效的观测（默认丢，并计数）。
    drop_occluded: bool = True
    drop_invalid: bool = True

    #: 是否要求"已标定"。True 时：``units != "m"`` 且帧里没有
    #: ``calibration.H_field_from_image`` → 抛错（像素不是世界坐标）。
    #: 早期联调（世界单位就是像素）可显式关掉，但**要在日志里说明**。
    require_calibration: bool = True


# ---------------------------------------------------------------------------
# 诊断
# ---------------------------------------------------------------------------
@dataclass
class VisionMeta:
    """这一帧转换过程的诊断量。**不参与决策**，只用于日志与调参。"""

    seq: int = -1
    stamp: float = math.nan
    dt: float = math.nan
    used_default_dt: bool = False
    used_default_speed: bool = False
    n_obstacles_raw: int = 0
    n_rects_kept: int = 0
    dropped_not_rect: int = 0
    dropped_invalid: int = 0
    dropped_occluded: int = 0
    dropped_low_score: int = 0
    dropped_bad_geometry: int = 0
    n_gaps: int = 0
    n_gaps_unreliable: int = 0

    @property
    def n_dropped(self) -> int:
        return (self.dropped_not_rect + self.dropped_invalid + self.dropped_occluded
                + self.dropped_low_score + self.dropped_bad_geometry)

    def summary(self) -> dict[str, Any]:
        return {
            "seq": self.seq, "stamp": self.stamp, "dt": self.dt,
            "used_default_dt": self.used_default_dt,
            "used_default_speed": self.used_default_speed,
            "obstacles_raw": self.n_obstacles_raw,
            "rects_kept": self.n_rects_kept,
            "dropped": {
                "not_rect": self.dropped_not_rect,
                "invalid": self.dropped_invalid,
                "occluded": self.dropped_occluded,
                "low_score": self.dropped_low_score,
                "bad_geometry": self.dropped_bad_geometry,
            },
            "gaps": self.n_gaps, "gaps_unreliable": self.n_gaps_unreliable,
        }


# ---------------------------------------------------------------------------
# 帧 → dt / player / rects
# ---------------------------------------------------------------------------
def dt_between(frame: Any, prev_frame: Any, cfg: VisionWorldConfig) -> tuple[float, bool]:
    """两帧之间的真实时间间隔；返回 ``(dt, 是否用了默认值)``。

    **只用 ``stamp``**。``stamp <= 0``（时钟倒流/坏时间戳）直接抛错，
    因为一个错误的 ``dt`` 会让所有速度判据静默偏。
    """
    if prev_frame is None:
        return float(cfg.dt_default), True
    t_new = float(_get(frame, "stamp", math.nan))
    t_old = float(_get(prev_frame, "stamp", math.nan))
    if not _finite(t_new, t_old):
        raise VisionAdapterError(f"stamp 不是有限数：new={t_new!r} old={t_old!r}")
    dt = t_new - t_old
    if not (dt > 0.0):
        raise VisionAdapterError(
            f"stamp 差必须 > 0，得到 {dt!r}（new={t_new} old={t_old}）。"
            "丢帧/重复帧要在调用方处理，不要在这里猜一个 dt"
        )
    return dt, False


def player_vector(frame: Any, cfg: VisionWorldConfig) -> tuple[list[float], bool]:
    """``player`` → ``[x, y, vx, vy, radius, speed, alive]``（与 ``P_*`` 下标一致）。

    返回 ``(vector, 是否用了默认 speed)``。**没有 player 就抛错**——
    没有角色位置，任何避障判据都没有意义。
    """
    p = _get(frame, "player")
    if p is None:
        raise VisionAdapterError("帧里没有 player：无法决策（不要用占位坐标兜底）")

    x = float(_get(p, "x", math.nan))
    y = float(_get(p, "y", math.nan))
    r = float(_get(p, "radius", math.nan))
    if not _finite(x, y, r) or r <= 0.0:
        raise VisionAdapterError(f"player 位置/半径非法：x={x!r} y={y!r} radius={r!r}")

    vx = float(_get(p, "vx", 0.0) or 0.0)
    vy = float(_get(p, "vy", 0.0) or 0.0)
    speed = _get(p, "speed", 0.0)
    speed = 0.0 if speed is None else float(speed)
    used_default = False
    if not (speed > 0.0):
        # 视觉侧通常不知道"速度上限"（那是底盘参数），用配置兜底并记账。
        speed = float(cfg.default_speed)
        used_default = True
        if not (speed > 0.0):
            raise VisionAdapterError("default_speed 必须 > 0")
    alive = bool(_get(p, "alive", True))
    return [x, y, vx, vy, r, speed, 1.0 if alive else 0.0], used_default


def rects_from_frame(frame: Any, cfg: VisionWorldConfig,
                     meta: VisionMeta) -> list[Rect]:
    """把帧里的**矩形**障碍转成 ``Rect`` 列表，并把丢弃原因记进 ``meta``。

    圆形障碍**不在这里**：``wall_with_gap`` 只用矩形，
    ``small_obstacles`` 走另一条路线（一维间隙），不该混进同一个 rect 列表。
    """
    out: list[Rect] = []
    obstacles = _get(frame, "obstacles", None) or []
    meta.n_obstacles_raw = len(obstacles)

    for ob in obstacles:
        if not _shape_is_rect(_get(ob, "shape", None)):
            meta.dropped_not_rect += 1
            continue
        if cfg.drop_invalid and not bool(_get(ob, "valid", True)):
            meta.dropped_invalid += 1
            continue
        if cfg.drop_occluded and bool(_get(ob, "occluded", False)):
            meta.dropped_occluded += 1
            continue
        if cfg.min_score is not None:
            score = _get(ob, "score", None)
            if score is not None and float(score) < float(cfg.min_score):
                meta.dropped_low_score += 1
                continue

        oid = _get(ob, "id", None)
        x = float(_get(ob, "x", math.nan))
        y = float(_get(ob, "y", math.nan))
        hw = _get(ob, "half_w", None)
        hh = _get(ob, "half_h", None)
        if oid is None or hw is None or hh is None:
            meta.dropped_bad_geometry += 1        # 矩形缺半尺寸 = 无法用
            continue
        hw, hh = float(hw), float(hh)
        if not _finite(x, y, hw, hh) or hw <= 0.0 or hh <= 0.0:
            meta.dropped_bad_geometry += 1
            continue

        rot = float(_get(ob, "rotation", 0.0) or 0.0)
        if not math.isfinite(rot):
            meta.dropped_bad_geometry += 1
            continue

        out.append(Rect(
            ent_id=int(oid), x=x, y=y,
            vx=float(_get(ob, "vx", 0.0) or 0.0),
            vy=float(_get(ob, "vy", 0.0) or 0.0),
            half_w=hw, half_h=hh, rotation=rot,
            angular_velocity=float(_get(ob, "angular_velocity", 0.0) or 0.0),
        ))
    meta.n_rects_kept = len(out)
    return out


def gaps_from_frame(frame: Any) -> list[tuple[float, float, float]]:
    """帧里的缺口观测 → ``[(center_x, center_y, width), ...]``。

    **用途是校核，不是替代算法**：视觉侧已经把缺口算出来了
    （标准 §4.4：缺口是"关系"不是实体，``blockers`` 指回两段墙），
    而 ``gap_avoid.reconstruct_aperture()`` 会从矩形自己再算一遍。
    两个结果应当一致；不一致就说明识别或重建有一方错了，值得报警。

    ``reliable=False`` 的缺口（边界被遮挡、宽度是推出来的）**照样返回**，
    由调用方决定是否采信——本函数不做隐式过滤。
    """
    out: list[tuple[float, float, float]] = []
    for g in _get(frame, "gaps", None) or []:
        center = _get(g, "center", None)
        width = _get(g, "width", None)
        if center is None or width is None:
            continue
        cx, cy = float(center[0]), float(center[1])
        w = float(width)
        if _finite(cx, cy, w) and w > 0.0:
            out.append((cx, cy, w))
    return out


# ---------------------------------------------------------------------------
# 一帧 → WorldView
# ---------------------------------------------------------------------------
def _check_contract(frame: Any, cfg: VisionWorldConfig) -> None:
    """坐标系 / 单位 / 场地尺寸 的前置校验。全部**显式抛错**。"""
    frame_id = _get(frame, "frame_id", None)
    if frame_id is not None and str(frame_id) != str(cfg.frame_id):
        raise VisionAdapterError(
            f"frame_id={frame_id!r} 与决策层期望的 {cfg.frame_id!r} 不一致："
            "坐标不在同一个坐标系里，不能直接用"
        )

    if not (cfg.field_w > 0.0 and cfg.field_h > 0.0):
        raise VisionAdapterError(f"field 尺寸必须 > 0：{cfg.field_w}x{cfg.field_h}")

    if cfg.require_calibration:
        units = getattr(_get(frame, "units", "px"), "value", None) or str(
            _get(frame, "units", "px"))
        calib = _get(frame, "calibration", None)
        h = _get(calib, "H_field_from_image", None) if calib is not None else None
        if str(units).lower() != "m" and h is None:
            raise VisionAdapterError(
                "帧既不是 units='m'，也没有 calibration.H_field_from_image —— "
                "说明坐标还是**像素**，不是世界坐标。"
                "先完成标定（见 docs/坐标系与标定.md），"
                "或确认'世界单位就是像素'后把 require_calibration=False 显式关掉"
            )


def frame_to_worldview(
    frame: Any,
    cfg: VisionWorldConfig,
    *,
    prev_frame: Any = None,
) -> tuple[WorldView, VisionMeta]:
    """一帧视觉输出 → ``(WorldView, VisionMeta)``。

    纯函数：不修改 ``frame``，不持有状态。``dt`` 需要上一帧，
    所以调用方要么传 ``prev_frame``，要么用 :class:`VisionWorldStream`。
    """
    _check_contract(frame, cfg)
    meta = VisionMeta(seq=int(_get(frame, "seq", -1)),
                      stamp=float(_get(frame, "stamp", math.nan)))

    dt, used_default_dt = dt_between(frame, prev_frame, cfg)
    meta.dt, meta.used_default_dt = dt, used_default_dt

    player, used_default_speed = player_vector(frame, cfg)
    meta.used_default_speed = used_default_speed

    rects = rects_from_frame(frame, cfg, meta)

    gaps = _get(frame, "gaps", None) or []
    meta.n_gaps = len(gaps)
    meta.n_gaps_unreliable = sum(
        1 for g in gaps if not bool(_get(g, "reliable", True)))

    view = WorldView(player=player, rects=rects, dt=dt,
                     step_index=int(_get(frame, "seq", 0)),
                     field_w=float(cfg.field_w), field_h=float(cfg.field_h))
    return view, meta


# ---------------------------------------------------------------------------
# 连续帧：把两个"必须跨帧复用"的东西包起来
# ---------------------------------------------------------------------------
@dataclass
class VisionWorldStream:
    """连续视觉帧 → 连续决策。

    包住两个**必须跨帧复用、却最容易被重新 new 出来**的东西：

    * ``GapMemory`` —— 缺口宽度/方位的记忆。每帧新建会让"只有一段墙可见"
      （实测约 4% 的帧）时无法补全缺口；
    * 上一帧 —— ``dt`` 的来源。

    行为等价于手写循环，只是不给写错的机会。::

        stream = VisionWorldStream(VisionWorldConfig())
        view, dec = stream.step(frame, cfg=AlgoConfig(forward=(0.0, 1.0)))
    """

    world_cfg: VisionWorldConfig
    memory: GapMemory = field(default_factory=GapMemory)
    prev_frame: Any = None
    last_meta: Optional[VisionMeta] = None
    frames: int = 0

    def step(self, frame: Any, *,
             cfg: Optional[AlgoConfig] = None,
             horizon_remaining: float = 2.0) -> tuple[WorldView, Decision]:
        """喂一帧，返回 ``(这一帧的 WorldView, 决策)``。"""
        view, meta = frame_to_worldview(frame, self.world_cfg,
                                        prev_frame=self.prev_frame)
        dec = plan(view, self.memory, cfg or AlgoConfig(),
                   horizon_remaining=horizon_remaining)
        self.prev_frame = frame
        self.last_meta = meta
        self.frames += 1
        return view, dec

    def reset(self) -> None:
        """新一局/新一段视频时调用：记忆和上一帧都要清。"""
        self.memory = GapMemory()
        self.prev_frame = None
        self.last_meta = None
        self.frames = 0
