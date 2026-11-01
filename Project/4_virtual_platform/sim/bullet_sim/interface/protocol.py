"""Flat State/Action protocol for the CPU <-> FPGA link.

A byte-exact, fixed-stride little-endian frame.  It is the *contract* between
the simulator and any hardware or external process, which is why it lives in
``interface/`` rather than inside the simulator.

Frame layout (State Protocol v1)
--------------------------------
::

    offset  size  type       field
    ------  ----  ---------  ------------------------------------------
    0       4     uint32     magic ('BHL1' == 0x314C4842)
    4       2     uint16     version
    6       2     uint16     flags (bit0: bullets are compacted)
    8       4     uint32     step_index
    12      4     uint32     n_bullets
    16      4     float32    sim_time
    20      4     float32    field_w
    24      4     float32    field_h
    28      4     float32    dt
    32      28    float32[7] player: x, y, vx, vy, radius, speed, alive
    60      24    float32[6] target: x, y, radius, shape, half_w, half_h
    84      40*n  float32[n] bullets, stride = 10 floats:
                             x, y, vx, vy, ax, ay, radius, ttl, type_id, group_id
    84+40n  4     uint32     crc32 of all preceding bytes

Total frame size: ``88 + 40 * n_bullets`` bytes.

Action frame: ``float32[2]`` desired world-frame velocity ``(vx, vy)``.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Any

import numpy as np

from bullet_sim.core.errors import ProtocolError
from bullet_sim.core.state import EnvState, WorldSnapshot
from bullet_sim.core.version import (
    STATE_PROTOCOL_VERSION,
    STATE_PROTOCOL_VERSION_MIN_READABLE,
)
from bullet_sim.entities.bullet import (
    PROTOCOL_FIELDS,
    PROTOCOL_FIELDS_V1,
    PROTOCOL_FIELDS_V2,
    BulletPool,
)
from bullet_sim.entities.player import PLAYER_DIM
from bullet_sim.entities.target import TARGET_DIM

MAGIC = 0x314C4842  # b'BHL1' little-endian
_HEADER_STRUCT = struct.Struct("<IHHII4f")
PLAYER_BLOCK = PLAYER_DIM  # 7 floats
TARGET_BLOCK = TARGET_DIM  # 6 floats
#: v2 wire layout (13 floats per bullet).
BULLET_STRIDE = len(PROTOCOL_FIELDS)
#: v1 wire layout kept for reading older captures (10 floats per bullet).
BULLET_STRIDE_V1 = len(PROTOCOL_FIELDS_V1)
#: v2 wire layout (13 floats): adds angle / angular_velocity / id.
BULLET_STRIDE_V2 = len(PROTOCOL_FIELDS_V2)
#: version -> (stride, field order)
_STRIDES: dict[int, tuple[int, tuple[str, ...]]] = {
    1: (BULLET_STRIDE_V1, PROTOCOL_FIELDS_V1),
    2: (BULLET_STRIDE_V2, PROTOCOL_FIELDS_V2),
    STATE_PROTOCOL_VERSION: (BULLET_STRIDE, PROTOCOL_FIELDS),
}
HEADER_BYTES = _HEADER_STRUCT.size + 4 * (PLAYER_BLOCK + TARGET_BLOCK)
TRAILER_BYTES = 4
FLAG_COMPACTED = 0x0001


def stride_for(version: int = STATE_PROTOCOL_VERSION) -> int:
    """Bullet stride (in float32 words) for a protocol version."""
    try:
        return _STRIDES[int(version)][0]
    except KeyError as exc:
        raise ProtocolError(f"unsupported protocol version {version}") from exc


def fields_for(version: int = STATE_PROTOCOL_VERSION) -> tuple[str, ...]:
    try:
        return _STRIDES[int(version)][1]
    except KeyError as exc:
        raise ProtocolError(f"unsupported protocol version {version}") from exc


def frame_size(n_bullets: int, version: int = STATE_PROTOCOL_VERSION) -> int:
    """Total byte size of a state frame carrying ``n_bullets`` bullets."""
    return (
        HEADER_BYTES + stride_for(version) * 4 * int(n_bullets) + TRAILER_BYTES
    )


def frame_size_v1(n_bullets: int) -> int:
    """Byte size of a legacy (protocol v1) frame."""
    return frame_size(n_bullets, 1)


@dataclass(frozen=True)
class FrameHeader:
    magic: int
    version: int
    flags: int
    step_index: int
    n_bullets: int
    sim_time: float
    field_w: float
    field_h: float
    dt: float


# --------------------------------------------------------------------------
# state frames
# --------------------------------------------------------------------------


def encode_state(snapshot: WorldSnapshot, *, compact: bool = True) -> bytes:
    """Serialize ``S_t`` into one fixed-stride hardware frame."""
    pool = snapshot.bullets
    idx = pool.active_indices() if compact else np.arange(pool.capacity)
    n = int(idx.size)
    p = snapshot.player
    t = snapshot.target
    e = snapshot.env

    buf = bytearray(frame_size(n))
    _HEADER_STRUCT.pack_into(
        buf,
        0,
        MAGIC,
        STATE_PROTOCOL_VERSION,
        FLAG_COMPACTED if compact else 0,
        int(e.step_index),
        n,
        float(e.sim_time),
        float(e.field_w),
        float(e.field_h),
        float(e.dt),
    )
    off = _HEADER_STRUCT.size
    _write_floats(
        buf,
        off,
        [p.x, p.y, p.vx, p.vy, p.radius, p.speed, 1.0 if p.alive else 0.0],
    )
    off += 4 * PLAYER_BLOCK
    _write_floats(
        buf,
        off,
        [t.x, t.y, t.radius, 0.0 if t.shape == "circle" else 1.0, t.half_w, t.half_h],
    )
    off += 4 * TARGET_BLOCK
    if n:
        mat = np.empty((n, BULLET_STRIDE), dtype="<f4")
        d = pool.data
        for j, name in enumerate(PROTOCOL_FIELDS):
            mat[:, j] = d[name][idx]
        raw = mat.tobytes()
        buf[off : off + len(raw)] = raw
        off += len(raw)
    crc = zlib.crc32(bytes(buf[:off])) & 0xFFFFFFFF
    struct.pack_into("<I", buf, off, crc)
    return bytes(buf)


def _write_floats(buf: bytearray, offset: int, values) -> None:
    """Little-endian float32 writes into a bytearray (no file-object games)."""
    raw = np.asarray(values, dtype="<f4").tobytes()
    buf[offset : offset + len(raw)] = raw


def decode_state(frame: bytes | bytearray | memoryview, *, dtype: Any = None) -> WorldSnapshot:
    """Parse and validate a hardware state frame."""
    if len(frame) < HEADER_BYTES + TRAILER_BYTES:
        raise ProtocolError(f"frame too short: {len(frame)} bytes")
    (
        magic,
        version,
        flags,
        step_index,
        n_bullets,
        sim_time,
        field_w,
        field_h,
        dt,
    ) = _HEADER_STRUCT.unpack_from(frame, 0)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic 0x{magic:08X}, expected 0x{MAGIC:08X}")
    if version < STATE_PROTOCOL_VERSION_MIN_READABLE or version > STATE_PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol version {version}; readable range is "
            f"[{STATE_PROTOCOL_VERSION_MIN_READABLE}, {STATE_PROTOCOL_VERSION}]"
        )
    stride = stride_for(version)
    fields = fields_for(version)
    expected = frame_size(n_bullets, version)
    if len(frame) != expected:
        raise ProtocolError(
            f"frame size mismatch: got {len(frame)}, expected {expected} for "
            f"{n_bullets} bullets"
        )
    crc_stored = struct.unpack_from("<I", frame, expected - 4)[0]
    crc_calc = zlib.crc32(bytes(memoryview(frame)[: expected - 4])) & 0xFFFFFFFF
    if crc_stored != crc_calc:
        raise ProtocolError(
            f"CRC mismatch: stored 0x{crc_stored:08X}, computed 0x{crc_calc:08X}"
        )

    off = _HEADER_STRUCT.size
    player = np.frombuffer(frame, dtype="<f4", count=PLAYER_BLOCK, offset=off).astype(np.float64)
    off += 4 * PLAYER_BLOCK
    target = np.frombuffer(frame, dtype="<f4", count=TARGET_BLOCK, offset=off).astype(np.float64)
    off += 4 * TARGET_BLOCK

    pool = BulletPool(n_bullets, dtype=dtype)
    if n_bullets:
        mat = np.frombuffer(
            frame, dtype="<f4", count=n_bullets * stride, offset=off
        ).reshape(n_bullets, stride)
        idx = np.arange(n_bullets, dtype=np.int32)
        for j, name in enumerate(fields):
            pool.data[name][idx] = mat[:, j]
        # Older frames carry fewer fields: derive what can be derived so an old
        # capture still decodes into a valid S_t.
        if version < 3:
            # v1/v2 have no shape columns -> every obstacle was a circle.
            pool.data["shape"][idx] = 0
            pool.data["half_w"][idx] = pool.data["radius"][idx]
            pool.data["half_h"][idx] = pool.data["radius"][idx]
            pool.data["rotation"][idx] = 0.0
        if version < 2:
            pool.data["angle"][idx] = np.arctan2(
                pool.data["vy"][idx], pool.data["vx"][idx]
            )
            pool.data["angular_velocity"][idx] = 0.0
            pool.data["id"][idx] = np.arange(1, n_bullets + 1, dtype=np.int32)
        pool.alive[:n_bullets] = True
        pool.count = n_bullets
        pool.next_id = int(pool.data["id"][:n_bullets].max()) + 1 if n_bullets else 1
        free = np.arange(pool.capacity - 1, n_bullets - 1, -1, dtype=np.int32)
        pool._free[: free.size] = free
        pool._free_top = int(free.size)

    env = EnvState(
        field_w=float(field_w),
        field_h=float(field_h),
        dt=float(dt),
        step_index=int(step_index),
        sim_time=float(sim_time),
        wrap="cull",
        bullet_budget=n_bullets,
    )
    return WorldSnapshot(
        player=_player_from(player),
        bullets=pool,
        target=_target_from(target),
        env=env,
        scenario_id="",
        seed=0,
        timestamp=float(sim_time),
    )


def _player_from(arr: np.ndarray):
    from bullet_sim.entities.player import PlayerState

    return PlayerState.from_array(arr)


def _target_from(arr: np.ndarray):
    from bullet_sim.entities.target import TargetState

    return TargetState.from_array(arr)


def read_header(frame: bytes | bytearray | memoryview) -> FrameHeader:
    """Cheap header-only inspection (no CRC check, no allocation)."""
    if len(frame) < _HEADER_STRUCT.size:
        raise ProtocolError("frame too short for a header")
    (
        magic,
        version,
        flags,
        step_index,
        n_bullets,
        sim_time,
        field_w,
        field_h,
        dt,
    ) = _HEADER_STRUCT.unpack_from(frame, 0)
    return FrameHeader(
        magic, version, flags, step_index, n_bullets, sim_time, field_w, field_h, dt
    )


def bullet_matrix(frame: bytes | bytearray | memoryview) -> np.ndarray:
    """Zero-copy ``float32[n, stride]`` view over the bullet block of a frame.

    ``stride`` is 13 for protocol v2 and 10 for v1 frames; use
    :func:`fields_for` with ``read_header(frame).version`` to interpret columns.
    """
    header = read_header(frame)
    stride = stride_for(header.version)
    off = _HEADER_STRUCT.size + 4 * (PLAYER_BLOCK + TARGET_BLOCK)
    return np.frombuffer(
        frame, dtype="<f4", count=header.n_bullets * stride, offset=off
    ).reshape(header.n_bullets, stride)


# --------------------------------------------------------------------------
# action frames
# --------------------------------------------------------------------------


def encode_action_velocity(vx: float, vy: float) -> bytes:
    """Pack a desired world-frame velocity into an action frame."""
    return struct.pack("<2f", float(vx), float(vy))


def decode_action_velocity(frame: bytes | bytearray | memoryview) -> np.ndarray:
    if len(frame) != 8:
        raise ProtocolError(f"action frame must be 8 bytes, got {len(frame)}")
    return np.array(struct.unpack_from("<2f", frame, 0), dtype=np.float64)


def encode_action_discrete(index: int) -> bytes:
    return struct.pack("<I", int(index) & 0xFFFFFFFF)


def decode_action_discrete(frame: bytes | bytearray | memoryview) -> int:
    if len(frame) != 4:
        raise ProtocolError(f"discrete action frame must be 4 bytes, got {len(frame)}")
    return int(struct.unpack_from("<I", frame, 0)[0])


def describe_protocol() -> dict:
    return {
        "name": "BHL1 state/action protocol",
        "version": STATE_PROTOCOL_VERSION,
        "readable_versions": sorted(_STRIDES),
        "bullet_fields_v2": list(PROTOCOL_FIELDS_V2),
        "magic": hex(MAGIC),
        "endianness": "little",
        "header_bytes": HEADER_BYTES,
        "trailer_bytes": TRAILER_BYTES,
        "bullet_stride_floats": BULLET_STRIDE,
        "bullet_fields": list(PROTOCOL_FIELDS),
        "bullet_fields_v1": list(PROTOCOL_FIELDS_V1),
        "frame_size": f"88 + {BULLET_STRIDE * 4} * n_bullets",
        "frame_size_v1": f"88 + {BULLET_STRIDE_V1 * 4} * n_bullets",
        "frame_size_v2": f"88 + {BULLET_STRIDE_V2 * 4} * n_bullets",
        "action_frame": {"velocity": "float32[2]", "discrete": "uint32"},
    }
