"""规范三的**线协议**：``VisionFrame`` ⇄ bytes（给 FPGA/PL 用）。

为什么要有这个文件
------------------
``vision_output.py`` 定义了视觉输出的**语义**（字段、单位、坐标系），但它只有
``to_dict`` / ``from_dict`` —— 那是 **JSON**。而这条帧的**生产者是 FPGA 的 PL**：
可编程逻辑不可能吐 JSON，也不可能解析浮点。所以"像素 → 几何"这一段要落地，
就必须先有一个**定长、定标、可逐位比对**的二进制帧。

写得跟 ``bullet_sim/interface/protocol.py``（状态帧 v3）**同一套纪律**，
因为那套已经被 CPU↔FPGA 回路验证过：

* 小端、**定长步长**、字段固定；
* **CRC32 覆盖全部前置字节**，收端先校验再解析；
* 版本显式，读到不认识的版本**必须报错**，不许"尽力解析"；
* ``frame_size(n)`` 与 ``describe_protocol()`` 是契约的一部分，供 testbench/HDL 对齐。

两条本文件特有的、必须写明的取舍
--------------------------------
1. **几何量走定点整数，不走 float32。**
   状态帧用 float32 是因为它由 CPU 产生。本帧由 **PL** 产生，
   而 PL 里做浮点要额外逻辑与时序预算；我们的几何量本来就来自
   整数累加器 + 一次除法，所以定标成整数是**更自然**的表达。
   标度 :data:`VISION_Q` = 64（1/64 单位），即

   * 像素坐标：分辨率 1/64 px ≈ 0.016 px —— 亚像素精修（~0.1 px）够用；
   * 场地单位：分辨率 1/64 ≈ 0.016 场地单位 —— 缺口中心目标 ≤0.1 够用。

   代价是**量化**：任意浮点值编码后有 ≤ ``1/(2*VISION_Q)`` 的误差。
   这是有意的，且**可算**（见 ``quantisation_error()``），
   不是"大概差不多"。

2. **本帧只承载 PL 能测的量：位置与几何。速度不进线协议。**
   这不是省字节，是被校验挡回来的：``PlayerState.validate()`` 要求
   ``speed_measured <= speed``，而 ``speed`` 是**速度上限**（配置量），
   PL 不知道它（``CPP_PORT.md`` §6 已经写明"视觉侧常常不知道，用配置默认值"）。
   若把 ``vx/vy`` 编进帧，解码出来的 ``PlayerState`` 会因为 ``speed`` 默认 0
   而**自己校验失败**；要绕过就得**编一个 speed 出来**，那正是本项目
   一路拒绝的做法。速度由 PS 从相邻帧推（对应 §3：运动/身份在 PS 侧），
   那里有 `stamp` 之差可用。

3. **时间是整数纳秒（``uint64``），不是 float 秒。**
   硬件时戳天然是计数器。纳秒在 double 的 15 位有效数字下、几天量程内无损，
   所以 ``stamp``（float 秒）↔ ``stamp_ns``（uint64）的往返是精确的。

帧布局（全部小端）
------------------
::

    offset size  type      field
    ------ ----  --------  --------------------------------------------
    -- 头 32 B --
    0      4     uint32    magic ('BHV1' == 0x31564842)
    4      2     uint16    version
    6      2     uint16    flags   bit0 units=1 表示 "m"，否则 "px"
                                  bit1 player 有效
                                  bit2 target 有效
    8      4     uint32    seq
    12     4     uint32    n_obstacles
    16     4     uint32    n_gaps
    20     2     uint16    image_width   (0 = 未提供)
    22     2     uint16    image_height  (0 = 未提供)
    24     8     uint64    stamp_ns
    -- 角色 16 B（定长，靠 flags 判有效）--
    32     16    int32[4]  x, y, radius, flags(bit0 valid, bit1 occluded)
    -- 目标 24 B（定长，同上）--
    48     24    int32[6]  x, y, radius, half_w, half_h, flags(同角色)
    -- 障碍 n_obstacles × 36 B --
    72     36*n  int32[9]  id, shape, type_id, x, y, radius, half_w, half_h, rotation
                           （radius 与 half_w/half_h 是**联合体**：
                            shape=circle 时只有 radius 有效，
                            shape=rect 时只有 half_w/half_h 有效，
                            另一半解出来是 None —— 与 Obstacle 自身约定一致。
                            早先漏了 radius 槽位，圆形障碍解出来 radius=None，
                            被 Obstacle.validate() 当场拒绝。）
                           （形状/类型编码复用 standard.obstacle 的
                            SHAPE_CODES / TYPE_CODES，**只追加不重排**）
    -- 缺口 n_gaps × 36 B --
    72+32n 36*g  int32[9]  id, center_x, center_y, width, axis,
                           blocker_a, blocker_b, blocked_by_type_code,
                           flags(bit0 reliable, bit1 occluded)
    -- 尾 --
    ..     4     uint32    crc32 of all preceding bytes

总长：``76 + 36*n_obstacles + 36*n_gaps``。

**几何字段的定标**：``x/y/radius/half_w/half_h/center_x/center_y/width/axis``
都是 ``round(值 * VISION_Q)``；``blocker_*`` 与各种 code 是原样整数，
**不乘标度**（它们是 id/枚举，不是几何量）。解码时对称还原。
"""

from __future__ import annotations

import struct
import zlib
from typing import Any, Mapping, Sequence

from ..obstacle.obstacle import SHAPE_CODES, TYPE_CODES, ObstacleShape, ObstacleType
from ..player.player import PlayerState
from .vision_output import GapObs, Units, VisionFrame

__all__ = [
    "VISION_MAGIC", "VISION_PROTOCOL_VERSION", "VISION_Q",
    "HEADER_BYTES", "PLAYER_BYTES", "TARGET_BYTES",
    "OBSTACLE_BYTES", "GAP_BYTES", "TRAILER_BYTES",
    "vision_frame_size", "encode_vision_frame", "decode_vision_frame",
    "describe_vision_protocol", "quantisation_error",
]

#: Magic: ``b'BHV1'`` 小端。
VISION_MAGIC = 0x31564842
#: 本规范的线协议版本。**改动 = 改版本号**，不要就地改含义。
VISION_PROTOCOL_VERSION = 1
#: 几何量的定标：值 = round(真实值 * VISION_Q)。
VISION_Q = 64

HEADER_BYTES = 32
PLAYER_BYTES = 16
TARGET_BYTES = 24
OBSTACLE_BYTES = 36
GAP_BYTES = 36
TRAILER_BYTES = 4

_HEADER = struct.Struct("<IHHIIIHHQ")
_PLAYER = struct.Struct("<4i")
_TARGET = struct.Struct("<6i")
_OBSTACLE = struct.Struct("<9i")
_GAP = struct.Struct("<9i")
_TRAILER = struct.Struct("<I")

#: 角色/目标记录的 flags 位。
_ACTOR_VALID = 1 << 0
_ACTOR_OCCLUDED = 1 << 1
#: 缺口记录的 flags 位。
_GAP_RELIABLE = 1 << 0
_GAP_OCCLUDED = 1 << 1
#: 头里的 flags 位。
_F_UNITS_M = 1 << 0
_F_PLAYER = 1 << 1
_F_TARGET = 1 << 2

_SHAPE_NAMES: dict[int, ObstacleShape] = {v: ObstacleShape(k) for k, v in SHAPE_CODES.items()}
_TYPE_NAMES: dict[int, ObstacleType | None] = {v: k for k, v in TYPE_CODES.items()}


def quantisation_error(q: int = VISION_Q) -> float:
    """一个几何量编码后的**最大**绝对误差（单位与输入相同）。

    四舍五入到最近的 ``1/q``，所以误差 ≤ ``1/(2q)``。
    ``VISION_Q = 64`` → **0.0078**。这个数要拿来对误差预算
    （`core/fpga/docs/移植规划.md` §2.3：缺口中心误差 ≤0.1 场地单位），
    而不是等测出来才发现不够。
    """
    if q <= 0:
        raise ValueError(f"标度必须为正，得到 {q!r}")
    return 1.0 / (2.0 * q)


def vision_frame_size(n_obstacles: int, n_gaps: int) -> int:
    """整帧字节数。**契约的一部分**：HDL/testbench 按它分配与校验。"""
    if n_obstacles < 0 or n_gaps < 0:
        raise ValueError(f"数量不能为负：n_obstacles={n_obstacles!r} n_gaps={n_gaps!r}")
    return (HEADER_BYTES + PLAYER_BYTES + TARGET_BYTES
            + OBSTACLE_BYTES * int(n_obstacles) + GAP_BYTES * int(n_gaps)
            + TRAILER_BYTES)


def _q(value: float) -> int:
    """浮点 → 定标整数。"""
    if value != value or value in (float("inf"), float("-inf")):
        raise ValueError(f"几何量必须有限，得到 {value!r}")
    return int(round(float(value) * VISION_Q))


def _uq(value: int) -> float:
    """定标整数 → 浮点。"""
    return int(value) / float(VISION_Q)


def _shape_code(shape: ObstacleShape | str) -> int:
    key = shape.value if isinstance(shape, ObstacleShape) else str(shape)
    if key not in SHAPE_CODES:
        raise ValueError(f"未知形状 {shape!r}；已登记的编码只有 {sorted(SHAPE_CODES)}")
    return SHAPE_CODES[key]


def _type_code(value: Any) -> int:
    if value is None:
        return TYPE_CODES[None]
    key = value if isinstance(value, ObstacleType) else ObstacleType(str(value))
    return TYPE_CODES[key]


# --------------------------------------------------------------------------
def encode_vision_frame(frame: VisionFrame) -> bytes:
    """``VisionFrame`` -> bytes。编码前先 ``validate()``（不合法的帧不出去）。"""
    frame.validate()                      # 语义校验：非法就抛，不要"尽力编码"

    n_obs = len(frame.obstacles)
    n_gaps = len(frame.gaps)
    flags = 0
    if frame.units == Units.M:
        flags |= _F_UNITS_M
    if frame.player is not None:
        flags |= _F_PLAYER
    if frame.target is not None:
        flags |= _F_TARGET

    img_w = int(frame.image_width or 0)
    img_h = int(frame.image_height or 0)
    if not (0 <= img_w <= 0xFFFF and 0 <= img_h <= 0xFFFF):
        raise ValueError(f"image 尺寸超出 uint16：{img_w}x{img_h}")

    buf = bytearray()
    buf += _HEADER.pack(VISION_MAGIC, VISION_PROTOCOL_VERSION, flags, int(frame.seq),
                        n_obs, n_gaps, img_w, img_h, _stamp_to_ns(frame.stamp))

    # 角色（定长；无效时全 0 + valid 位 = 0）
    p = frame.player
    if p is None:
        buf += _PLAYER.pack(0, 0, 0, 0)
    else:
        aflags = _ACTOR_VALID
        if getattr(p, "occluded", False):
            aflags |= _ACTOR_OCCLUDED
        buf += _PLAYER.pack(_q(p.x), _q(p.y), _q(p.radius), aflags)

    # 目标（定长，同理）
    t = frame.target
    if t is None:
        buf += _TARGET.pack(0, 0, 0, 0, 0, 0)
    else:
        tflags = _ACTOR_VALID if t.valid else 0
        if t.occluded:
            tflags |= _ACTOR_OCCLUDED
        buf += _TARGET.pack(_q(t.x), _q(t.y), _q(t.radius or 0.0),
                            _q(t.half_w or 0.0), _q(t.half_h or 0.0), tflags)

    for ob in frame.obstacles:
        buf += _OBSTACLE.pack(
            int(ob.id), _shape_code(ob.shape), _type_code(ob.type),
            _q(ob.x), _q(ob.y), _q(ob.radius or 0.0),
            _q(ob.half_w or 0.0), _q(ob.half_h or 0.0), _q(ob.rotation))

    for gp in frame.gaps:
        gflags = 0
        if gp.reliable:
            gflags |= _GAP_RELIABLE
        if gp.occluded:
            gflags |= _GAP_OCCLUDED
        buf += _GAP.pack(int(gp.id), _q(gp.center[0]), _q(gp.center[1]),
                         _q(gp.width), _q(gp.axis),
                         int(gp.blockers[0]), int(gp.blockers[1]),
                         _type_code(gp.blocked_by_type), gflags)

    buf += _TRAILER.pack(zlib.crc32(bytes(buf)) & 0xFFFFFFFF)
    return bytes(buf)


def decode_vision_frame(data: bytes | bytearray | memoryview) -> VisionFrame:
    """bytes -> ``VisionFrame``。**任何不一致都抛 ``ValueError``，绝不猜。**"""
    raw = bytes(data)
    if len(raw) < HEADER_BYTES + PLAYER_BYTES + TARGET_BYTES + TRAILER_BYTES:
        raise ValueError(f"帧太短：{len(raw)} 字节（最小 "
                         f"{vision_frame_size(0, 0)}）")

    (magic, version, flags, seq, n_obs, n_gaps, img_w, img_h, stamp_ns) = \
        _HEADER.unpack_from(raw, 0)
    if magic != VISION_MAGIC:
        raise ValueError(f"magic 不对：0x{magic:08X}，应为 0x{VISION_MAGIC:08X}"
                         "（这不是一帧视觉输出）")
    if version != VISION_PROTOCOL_VERSION:
        raise ValueError(
            f"视觉帧版本 {version} 不是本实现支持的 {VISION_PROTOCOL_VERSION}；"
            "不许'尽力解析'—— 未知版本的同名字段含义可能已经变了")

    want = vision_frame_size(n_obs, n_gaps)
    if len(raw) != want:
        raise ValueError(f"长度不符：收到 {len(raw)} 字节，按 n_obstacles={n_obs} "
                         f"n_gaps={n_gaps} 应为 {want} 字节")
    crc_got = _TRAILER.unpack_from(raw, want - TRAILER_BYTES)[0]
    crc_want = zlib.crc32(raw[:want - TRAILER_BYTES]) & 0xFFFFFFFF
    if crc_got != crc_want:
        raise ValueError(f"CRC32 不符：帧里 0x{crc_got:08X}，实算 0x{crc_want:08X}"
                         "（帧被截断/篡改）")

    units = Units.M if flags & _F_UNITS_M else Units.PX
    off = HEADER_BYTES

    px, py, pr, pflags = _PLAYER.unpack_from(raw, off)
    off += PLAYER_BYTES
    player = None
    if flags & _F_PLAYER:
        player = PlayerState(x=_uq(px), y=_uq(py), radius=_uq(pr),
                             occluded=bool(pflags & _ACTOR_OCCLUDED),
                             stamp=stamp_ns / 1e9, seq=int(seq))

    tx, ty, trad, thw, thh, tflags = _TARGET.unpack_from(raw, off)
    off += TARGET_BYTES
    target = None
    if flags & _F_TARGET:
        from .vision_output import TargetObs
        is_rect = thw != 0 or thh != 0
        target = TargetObs(x=_uq(tx), y=_uq(ty),
                           shape="rect" if is_rect else "circle",
                           radius=None if is_rect else _uq(trad),
                           half_w=_uq(thw) if is_rect else None,
                           half_h=_uq(thh) if is_rect else None,
                           valid=bool(tflags & _ACTOR_VALID),
                           occluded=bool(tflags & _ACTOR_OCCLUDED))

    from ..obstacle.obstacle import Obstacle
    obstacles = []
    for _ in range(n_obs):
        (oid, shape_c, type_c, x, y, rad, hw, hh, rot) = \
            _OBSTACLE.unpack_from(raw, off)
        off += OBSTACLE_BYTES
        shape = _SHAPE_NAMES.get(shape_c)
        if shape is None:
            raise ValueError(f"未知 shape 编码 {shape_c}（只允许 "
                             f"{sorted(SHAPE_CODES.values())}）")
        type_name = _TYPE_NAMES.get(type_c)
        if type_c not in _TYPE_NAMES:
            raise ValueError(f"未知 type 编码 {type_c}（只允许 "
                             f"{sorted(TYPE_CODES.values())}）")
        is_circle = (shape == ObstacleShape.CIRCLE)
        obstacles.append(Obstacle(
            id=int(oid), shape=shape, type=type_name, type_id=int(type_c),
            x=_uq(x), y=_uq(y),
            radius=_uq(rad) if is_circle else None,
            half_w=None if is_circle else _uq(hw),
            half_h=None if is_circle else _uq(hh),
            rotation=_uq(rot)))

    gaps = []
    for _ in range(n_gaps):
        gid, cx, cy, w, axis, ba, bb, bt, gflags = _GAP.unpack_from(raw, off)
        off += GAP_BYTES
        if bt not in _TYPE_NAMES:
            raise ValueError(f"缺口 blocked_by_type 编码未知：{bt}")
        blocked = _TYPE_NAMES[bt]
        gaps.append(GapObs(
            id=int(gid), center=(_uq(cx), _uq(cy)), width=_uq(w), axis=_uq(axis),
            blockers=(int(ba), int(bb)),
            blocked_by_type=(blocked.value if blocked is not None else None),
            reliable=bool(gflags & _GAP_RELIABLE),
            occluded=bool(gflags & _GAP_OCCLUDED)))

    frame = VisionFrame(
        seq=int(seq), stamp=stamp_ns / 1e9, units=units,
        image_width=img_w or None, image_height=img_h or None,
        player=player, obstacles=obstacles, gaps=gaps, target=target)
    frame.validate()
    return frame


def _stamp_to_ns(stamp: float) -> int:
    ns = int(round(float(stamp) * 1e9))
    if ns < 0:
        raise ValueError(f"stamp 不能为负：{stamp!r}")
    if ns >= 1 << 64:
        raise ValueError(f"stamp 超出 uint64 纳秒量程：{stamp!r}")
    return ns


def describe_vision_protocol() -> dict:
    """给 testbench / HDL / 文档用的契约自述。**别在别处再抄一份布局。**"""
    return {
        "magic": VISION_MAGIC,
        "magic_bytes": "BHV1",
        "version": VISION_PROTOCOL_VERSION,
        "endianness": "little",
        "quantisation_scale": VISION_Q,
        "quantisation_max_error": quantisation_error(),
        "crc": "crc32 over all preceding bytes",
        "header_bytes": HEADER_BYTES,
        "player_bytes": PLAYER_BYTES,
        "target_bytes": TARGET_BYTES,
        "obstacle_bytes": OBSTACLE_BYTES,
        "gap_bytes": GAP_BYTES,
        "trailer_bytes": TRAILER_BYTES,
        "frame_size": "76 + 36*n_obstacles + 36*n_gaps",
        "stamp_units": "uint64 nanoseconds",
        "shape_codes": dict(SHAPE_CODES),
        "type_codes": {str(k): v for k, v in TYPE_CODES.items()},
        "flags": {
            "header.bit0": "units: 1=m, 0=px",
            "header.bit1": "player record valid",
            "header.bit2": "target record valid",
            "actor.bit0": "valid",
            "actor.bit1": "occluded",
            "gap.bit0": "reliable",
            "gap.bit1": "occluded",
        },
    }
