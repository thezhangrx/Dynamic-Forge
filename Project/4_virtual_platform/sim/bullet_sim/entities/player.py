"""Player state.

The live player is a contiguous ``float64[7]`` vector inside the world - the
same layout that maps onto a CPU/FPGA register block.  :class:`PlayerState` is
an immutable, serializable *view* of that vector used by snapshots, datasets
and observations.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.numerics import float_dtype

# -- indices into the raw player vector ------------------------------------
P_X = 0
P_Y = 1
P_VX = 2
P_VY = 3
P_RADIUS = 4
P_SPEED = 5
P_ALIVE = 6
PLAYER_DIM = 7

PLAYER_FIELDS: tuple[str, ...] = ("x", "y", "vx", "vy", "radius", "speed", "alive")


def player_vector(
    x: float,
    y: float,
    vx: float = 0.0,
    vy: float = 0.0,
    radius: float = 3.0,
    speed: float = 200.0,
    alive: bool = True,
    dtype: Any = None,
) -> np.ndarray:
    """Build the raw player vector used by the physics kernel."""
    v = np.zeros(PLAYER_DIM, dtype=float_dtype(dtype))
    v[P_X] = x
    v[P_Y] = y
    v[P_VX] = vx
    v[P_VY] = vy
    v[P_RADIUS] = radius
    v[P_SPEED] = speed
    v[P_ALIVE] = 1.0 if alive else 0.0
    return v


@dataclass(frozen=True)
class PlayerState:
    """Immutable player snapshot."""

    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 3.0
    speed: float = 200.0
    alive: bool = True

    # -- conversion --------------------------------------------------------
    def to_array(self, dtype: Any = None) -> np.ndarray:
        v = np.array(
            [self.x, self.y, self.vx, self.vy, self.radius, self.speed,
             1.0 if self.alive else 0.0],
            dtype=float_dtype(dtype),
        )
        return v

    @classmethod
    def from_array(cls, arr: Sequence[float]) -> "PlayerState":
        a = np.asarray(arr, dtype=np.float64).reshape(-1)
        return cls(
            x=float(a[P_X]),
            y=float(a[P_Y]),
            vx=float(a[P_VX]),
            vy=float(a[P_VY]),
            radius=float(a[P_RADIUS]),
            speed=float(a[P_SPEED]),
            alive=bool(a[P_ALIVE] > 0.5),
        )

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float64)

    @property
    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy], dtype=np.float64)

    @property
    def speed_norm(self) -> float:
        return float(np.hypot(self.vx, self.vy))

    def distance_to(self, x: float, y: float) -> float:
        return float(np.hypot(self.x - x, self.y - y))

    # -- serialization -----------------------------------------------------
    def to_dict(self) -> dict:
        return {name: getattr(self, name) for name in PLAYER_FIELDS}

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "PlayerState":
        return cls(
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            vx=float(payload.get("vx", 0.0)),
            vy=float(payload.get("vy", 0.0)),
            radius=float(payload.get("radius", 3.0)),
            speed=float(payload.get("speed", 200.0)),
            alive=bool(payload.get("alive", True)),
        )

    def copy(self, **changes: Any) -> "PlayerState":
        from dataclasses import replace

        return replace(self, **changes)
