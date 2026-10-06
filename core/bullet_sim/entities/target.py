"""Target zone - the goal region the agent should reach / hold.

Deliberately generic (circle / rect / point).  For the real 2D-car transfer the
target is simply a physical goal region, no game semantics attached.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

SHAPE_CODES: dict[str, float] = {"circle": 0.0, "point": 0.0, "rect": 1.0}
SHAPE_NAMES: tuple[str, ...] = ("circle", "rect")

TARGET_DIM = 6
TARGET_FIELDS: tuple[str, ...] = ("x", "y", "radius", "shape", "half_w", "half_h")


def target_vector(
    x: float,
    y: float,
    radius: float = 10.0,
    shape: str = "circle",
    half_w: float = 0.0,
    half_h: float = 0.0,
    dtype: Any = None,
) -> np.ndarray:
    from bullet_sim.core.numerics import float_dtype

    return np.array(
        [x, y, radius, SHAPE_CODES.get(shape, 0.0), half_w, half_h],
        dtype=float_dtype(dtype),
    )


@dataclass(frozen=True)
class TargetState:
    """Immutable target-zone snapshot."""

    x: float
    y: float
    radius: float = 10.0
    shape: str = "circle"
    half_w: float = 0.0
    half_h: float = 0.0

    @property
    def rect_half_extents(self) -> tuple[float, float]:
        if self.shape == "rect":
            return self.half_w, self.half_h
        return self.radius, self.radius

    def contains(self, x: float, y: float) -> bool:
        if self.shape == "rect":
            return abs(x - self.x) <= self.half_w and abs(y - self.y) <= self.half_h
        return float(np.hypot(x - self.x, y - self.y)) <= self.radius

    def distance_to(self, x: float, y: float) -> float:
        dx = abs(x - self.x) - (self.half_w if self.shape == "rect" else 0.0)
        dy = abs(y - self.y) - (self.half_h if self.shape == "rect" else 0.0)
        if self.shape == "rect":
            dx = max(0.0, dx)
            dy = max(0.0, dy)
            return float(np.hypot(dx, dy))
        return max(0.0, float(np.hypot(x - self.x, y - self.y)) - self.radius)

    # -- conversion --------------------------------------------------------
    def to_array(self, dtype: Any = None) -> np.ndarray:
        return target_vector(
            self.x, self.y, self.radius, self.shape, self.half_w, self.half_h, dtype=dtype
        )

    @classmethod
    def from_array(cls, arr: Any) -> "TargetState":
        a = np.asarray(arr, dtype=np.float64).reshape(-1)
        shape_code = int(round(float(a[3])))
        name = SHAPE_NAMES[shape_code] if 0 <= shape_code < len(SHAPE_NAMES) else "circle"
        return cls(
            x=float(a[0]),
            y=float(a[1]),
            radius=float(a[2]),
            shape=name,
            half_w=float(a[4]),
            half_h=float(a[5]),
        )

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "radius": self.radius,
            "shape": self.shape,
            "half_w": self.half_w,
            "half_h": self.half_h,
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "TargetState":
        return cls(
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            radius=float(payload.get("radius", 10.0)),
            shape=str(payload.get("shape", "circle")),
            half_w=float(payload.get("half_w", 0.0)),
            half_h=float(payload.get("half_h", 0.0)),
        )
