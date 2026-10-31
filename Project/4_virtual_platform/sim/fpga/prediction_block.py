"""FPGA-side prediction block: contract + a software reference implementation.

The FPGA is going to compute *something* over the bullet cloud and hand the
result back to the CPU.  This package fixes two things and refuses to fix a
third:

**Fixed here**

1. the **input**: exactly one state frame, as produced by
   ``bullet_sim.interface.protocol.encode_state`` (v2, 88 + 52·n bytes, CRC32);
2. the **output**: exactly one prediction frame, defined in this module
   (magic ``BHP1``, a normalised H x rows x cols risk grid + CRC32).

**Deliberately NOT fixed** - ``[HARDWARE_INTERFACE_TBD]``

3. the transport between CPU and FPGA (DMA / MMIO / interrupt / shared memory /
   PCIe / ...), the register map, the handshake and the timing budget.  None of
   these are decided, so none of them are invented here.  See
   ``bullet_sim/hardware_interface/tbd.py``.

:class:`FpgaPredictionReference` implements the fixed part in pure Python, so
the CPU side, the frame layout and the whole decision pipeline can be built,
tested and benchmarked **today**, and the HDL only has to reproduce a
bit-exact function of its input.

.. note::
   This is a **software reference model, not HDL**, and it makes no claim about
   resource usage or timing.  ``[SOFTWARE_REFERENCE_ONLY]``
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Any

import numpy as np

from bullet_sim.core.errors import ProtocolError
from bullet_sim.interface.protocol import decode_state, encode_state
from bullet_sim.prediction.ballistic import BallisticPredictor

#: Magic for the prediction frame: b'BHP1' little-endian.
PREDICTION_MAGIC = 0x31504842
PREDICTION_PROTOCOL_VERSION = 1
PREDICTION_HEADER = struct.Struct("<IHHIHHHH5fI")
PREDICTION_TRAILER_BYTES = 4
PREDICTION_STATUS_OK = 0

#: The board link is not chosen yet.  Nothing here may assume a transport.
HARDWARE_INTERFACE_TBD = "[HARDWARE_INTERFACE_TBD]"

#: This module is a Python reference, not a synthesisable design.
SOFTWARE_REFERENCE_ONLY = "[SOFTWARE_REFERENCE_ONLY]"


@dataclass(frozen=True)
class PredictionHeader:
    magic: int
    version: int
    flags: int
    source_step: int
    horizon: int
    rows: int
    cols: int
    cell_w: float
    cell_h: float
    dt: float
    origin_x: float
    origin_y: float
    n_cells: int


def prediction_frame_size(horizon: int, rows: int, cols: int) -> int:
    """Total bytes of a prediction frame carrying an ``H x rows x cols`` grid."""
    return (
        PREDICTION_HEADER.size
        + 4 * int(horizon) * int(rows) * int(cols)
        + PREDICTION_TRAILER_BYTES
    )


def encode_prediction_frame(
    grid: np.ndarray,
    *,
    source_step: int,
    dt: float,
    cell_w: float,
    cell_h: float,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    horizon: int | None = None,
) -> bytes:
    """Serialise a risk grid into one FPGA -> CPU prediction frame.

    ``grid`` is ``(H, rows, cols)`` float in ``[0, 1]``; it is flattened
    row-major with row 0 at ``origin_y``.  ``horizon == H`` unless stated.
    """
    g = np.ascontiguousarray(grid, dtype="<f4")
    if g.ndim != 3:
        raise ProtocolError(f"risk grid must be (H, rows, cols), got {g.shape}")
    h, rows, cols = g.shape
    n_cells = h * rows * cols
    buf = bytearray(prediction_frame_size(h, rows, cols))
    PREDICTION_HEADER.pack_into(
        buf, 0, PREDICTION_MAGIC, PREDICTION_PROTOCOL_VERSION, 0,
        int(source_step), int(horizon if horizon is not None else h),
        int(rows), int(cols), 0,
        float(cell_w), float(cell_h), float(dt), float(origin_x), float(origin_y),
        n_cells,
    )
    off = PREDICTION_HEADER.size
    raw = g.tobytes()
    buf[off : off + len(raw)] = raw
    off += len(raw)
    struct.pack_into("<I", buf, off, zlib.crc32(bytes(buf[:off])) & 0xFFFFFFFF)
    return bytes(buf)


def decode_prediction_frame(frame: bytes | bytearray | memoryview) -> tuple[np.ndarray, PredictionHeader]:
    """Parse + validate a prediction frame, returning ``(grid, header)``."""
    if len(frame) < PREDICTION_HEADER.size + PREDICTION_TRAILER_BYTES:
        raise ProtocolError(f"prediction frame too short: {len(frame)} bytes")
    fields = PREDICTION_HEADER.unpack_from(frame, 0)
    # field 7 is `reserved` and is intentionally not part of PredictionHeader
    header = PredictionHeader(
        magic=fields[0], version=fields[1], flags=fields[2], source_step=fields[3],
        horizon=fields[4], rows=fields[5], cols=fields[6],
        cell_w=fields[8], cell_h=fields[9], dt=fields[10],
        origin_x=fields[11], origin_y=fields[12], n_cells=fields[13],
    )
    if header.magic != PREDICTION_MAGIC:
        raise ProtocolError(
            f"bad prediction magic 0x{header.magic:08X}, expected 0x{PREDICTION_MAGIC:08X}"
        )
    if header.version != PREDICTION_PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported prediction protocol version {header.version}"
        )
    expected = prediction_frame_size(header.horizon, header.rows, header.cols)
    if len(frame) != expected:
        raise ProtocolError(
            f"prediction frame size mismatch: got {len(frame)}, expected {expected}"
        )
    stored = struct.unpack_from("<I", frame, expected - 4)[0]
    calc = zlib.crc32(bytes(memoryview(frame)[: expected - 4])) & 0xFFFFFFFF
    if stored != calc:
        raise ProtocolError(
            f"prediction CRC mismatch: stored 0x{stored:08X}, computed 0x{calc:08X}"
        )
    flat = np.frombuffer(
        frame, dtype="<f4", count=header.n_cells, offset=PREDICTION_HEADER.size
    )
    grid = flat.reshape(int(header.horizon), int(header.rows), int(header.cols))
    return grid.astype(np.float32), header


@dataclass
class FpgaConfig:
    """Compile-time knobs the HDL would be parameterised with."""

    rows: int = 30
    cols: int = 40
    horizon: int = 30
    safety_radius: float = 18.0
    #: Per-cell decay of the soft danger field; 1.0 degenerates to a binary map.
    soft_decay: float = 0.82
    #: Fixed-point width the future HDL will use.  ``None`` = float reference.
    fixed_point_bits: int | None = None


class FpgaPredictionReference:
    """Software reference of the FPGA prediction block.

    ``predict(state_frame) -> prediction_frame`` - a pure function of its input
    bytes.  Same input, same output, always; that is what makes it usable as the
    golden model for HDL verification.

    It runs the analytic ballistics of every bullet and rasterises the result
    into an ``H x rows x cols`` risk grid, dilated by the player's safety radius.
    """

    name = "fpga-prediction-reference"
    status = SOFTWARE_REFERENCE_ONLY

    def __init__(
        self,
        config: FpgaConfig | None = None,
        *,
        field_w: float = 640.0,
        field_h: float = 480.0,
    ) -> None:
        self.config = config or FpgaConfig()
        self.field_w = float(field_w)
        self.field_h = float(field_h)
        self._predictor = BallisticPredictor()
        self.frames = 0

    # ------------------------------------------------------------------
    def predict(
        self,
        state_frame: bytes,
        *,
        field_w: float | None = None,
        field_h: float | None = None,
    ) -> bytes:
        """FPGA input buffer -> FPGA result buffer."""
        snapshot = decode_state(state_frame)
        self.frames += 1
        return self.predict_from_snapshot(
            snapshot,
            field_w=field_w if field_w is not None else snapshot.env.field_w,
            field_h=field_h if field_h is not None else snapshot.env.field_h,
        )

    def predict_from_snapshot(
        self, snapshot: Any, *, field_w: float, field_h: float
    ) -> bytes:
        cfg = self.config
        horizon = max(1, int(cfg.horizon))
        rows = max(1, int(cfg.rows))
        cols = max(1, int(cfg.cols))
        dt = float(snapshot.env.dt)

        forecast = self._predictor.predict(snapshot, horizon, dt)
        grid = np.zeros((horizon, rows, cols), dtype=np.float32)

        cell_w = float(field_w) / cols
        cell_h = float(field_h) / rows
        if forecast.n_bullets == 0:
            return encode_prediction_frame(
                grid, source_step=snapshot.env.step_index, dt=dt,
                cell_w=cell_w, cell_h=cell_h,
            )

        # splat every sampled future position, then dilate by the safety radius
        positions = forecast.positions           # (N, H, 2)
        valid = forecast.valid                   # (N, H)
        for step in range(horizon):
            m = valid[:, step]
            if not m.any():
                continue
            p = positions[m, step, :]
            cx = np.clip((p[:, 0] / cell_w).astype(np.int64), 0, cols - 1)
            cy = np.clip((p[:, 1] / cell_h).astype(np.int64), 0, rows - 1)
            np.add.at(grid[step].ravel(), cy * cols + cx, 1.0)
        k = max(1, int(np.ceil(cfg.safety_radius / min(cell_w, cell_h))))
        # Soft dilation (max with geometric decay) instead of a binary one: a
        # binary field saturates at 1.0 over a whole neighbourhood, which makes
        # every "close enough" cell indistinguishable to the decision layer.
        # The decaying field keeps the *gradient* of danger, which is what the
        # CPU needs to choose between two merely-safe directions.
        grid = _dilate(grid, k, axis=1, decay=cfg.soft_decay)
        grid = _dilate(grid, k, axis=2, decay=cfg.soft_decay)
        peak = float(grid.max())
        if peak > 0:
            grid /= peak
        return encode_prediction_frame(
            grid, source_step=snapshot.env.step_index, dt=dt,
            cell_w=cell_w, cell_h=cell_h,
        )

    # ------------------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "input": "state frame (interface.protocol v2/v3): 88 + 4*stride*n bytes",
            "output": f"prediction frame (BHP1 v{PREDICTION_PROTOCOL_VERSION})",
            "grid": {"horizon": self.config.horizon, "rows": self.config.rows,
                     "cols": self.config.cols},
            "safety_radius": self.config.safety_radius,
            "soft_decay": self.config.soft_decay,
            "fixed_point_bits": self.config.fixed_point_bits,
            "transport": HARDWARE_INTERFACE_TBD,
            "resource_estimates": HARDWARE_INTERFACE_TBD,
            "frames": self.frames,
        }


def _dilate(a: np.ndarray, k: int, axis: int, decay: float = 1.0) -> np.ndarray:
    """Separable max-dilation with geometric decay (a cheap soft distance field).

    ``decay == 1.0`` reproduces the classic binary dilation; values below 1 make
    the field fall off with distance, which keeps it informative for the
    decision layer instead of saturating over a whole neighbourhood.
    """
    out = a.copy()
    cur = a
    for _ in range(int(k)):
        shifted = np.roll(cur, 1, axis=axis) * decay
        np.maximum(out, shifted, out=out)
        shifted = np.roll(out, -1, axis=axis) * decay
        np.maximum(out, shifted, out=out)
        cur = out
    return out


def describe_prediction_protocol() -> dict[str, Any]:
    """Frame contract, for the README and for the HDL team."""
    return {
        "name": "BHP1 prediction frame",
        "version": PREDICTION_PROTOCOL_VERSION,
        "magic": hex(PREDICTION_MAGIC),
        "endianness": "little",
        "header_bytes": PREDICTION_HEADER.size,
        "trailer_bytes": PREDICTION_TRAILER_BYTES,
        "cell_type": "float32 (or fixed-point once fixed_point_bits is chosen)",
        "layout": (
            "magic u32 | version u16 | flags u16 | source_step u32 | horizon u16 | "
            "rows u16 | cols u16 | reserved u16 | cell_w f32 | cell_h f32 | dt f32 | "
            "origin_x f32 | origin_y f32 | n_cells u32 | grid f32[n_cells] row-major | crc32 u32"
        ),
        "frame_size": "44 + 4*horizon*rows*cols + 4",
        "transport": HARDWARE_INTERFACE_TBD,
    }


__all__ = [
    "PREDICTION_MAGIC",
    "PREDICTION_PROTOCOL_VERSION",
    "HARDWARE_INTERFACE_TBD",
    "SOFTWARE_REFERENCE_ONLY",
    "PredictionHeader",
    "FpgaConfig",
    "FpgaPredictionReference",
    "prediction_frame_size",
    "encode_prediction_frame",
    "decode_prediction_frame",
    "describe_prediction_protocol",
]
