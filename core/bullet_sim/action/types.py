"""The unified Action type.

    Action = { direction: (dx, dy), magnitude: float }

This is the *single* action representation the platform exposes to the outside
world.  Both control modes produce it and neither can bypass it:

    Keyboard / gamepad      -> Action -> Simulator        (mode A, manual)
    Development board       -> Action -> Simulator        (mode B, hardware)

``direction`` is a unit vector in world coordinates (``0`` = no direction) and
``magnitude`` is a normalised strength in ``[0, 1]``, so an analog stick, a
velocity profile, or a board-side speed level can all be expressed without
changing the environment logic.  A discrete 9-way pad is simply the special case
of unit-axis directions at ``magnitude = 1``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.actions import (
    DISCRETE_ACTION_NAMES,
    DISCRETE_DIRECTIONS,
    DISCRETE_ACTION_INDEX,
)

_EPS = 1e-9


@dataclass(frozen=True)
class Action:
    """A normalised movement command."""

    direction: tuple[float, float] = (0.0, 0.0)
    magnitude: float = 1.0

    def __post_init__(self) -> None:
        d = self.direction
        if len(d) != 2:
            raise ValueError(f"action direction must have 2 components, got {d!r}")
        dx, dy = float(d[0]), float(d[1])
        if not (math.isfinite(dx) and math.isfinite(dy)):
            raise ValueError(f"action direction must be finite, got {d!r}")
        mag = float(self.magnitude)
        if not math.isfinite(mag) or mag < 0.0:
            raise ValueError(f"action magnitude must be finite and >= 0, got {mag!r}")
        norm = math.hypot(dx, dy)
        if norm > _EPS:
            # store a unit vector; magnitude carries the strength
            dx, dy = dx / norm, dy / norm
        else:
            dx, dy = 0.0, 0.0
        object.__setattr__(self, "direction", (dx, dy))
        object.__setattr__(self, "magnitude", min(mag, 1.0))

    # ------------------------------------------------------------------
    # constructors
    # ------------------------------------------------------------------
    @classmethod
    def zero(cls) -> "Action":
        """No movement (the ``stay`` action)."""
        return cls((0.0, 0.0), 0.0)

    @classmethod
    def from_discrete(cls, action: int | str, magnitude: float = 1.0) -> "Action":
        """Build from the canonical 9-way discrete table."""
        idx = cls._discrete_index(action)
        dx, dy = DISCRETE_DIRECTIONS[idx]
        if dx == 0.0 and dy == 0.0:
            return cls.zero()
        return cls((float(dx), float(dy)), magnitude)

    @classmethod
    def from_vector(cls, vector: Sequence[float], *, clamp: bool = True) -> "Action":
        """Build from a 2D vector; ``|vector|`` becomes the magnitude."""
        v = np.asarray(vector, dtype=np.float64).reshape(-1)
        if v.size != 2:
            raise ValueError(f"action vector must have 2 components, got {v.size}")
        norm = float(np.hypot(v[0], v[1]))
        if norm <= _EPS:
            return cls.zero()
        mag = min(norm, 1.0) if clamp else norm
        return cls((float(v[0]) / norm, float(v[1]) / norm), mag)

    @classmethod
    def from_angle(cls, degrees: float, magnitude: float = 1.0) -> "Action":
        """Build from a heading in degrees (0 = +x, counter-clockwise)."""
        rad = math.radians(float(degrees))
        return cls((math.cos(rad), math.sin(rad)), magnitude)

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "Action":
        return cls(
            direction=tuple(payload.get("direction", (0.0, 0.0))),  # type: ignore[arg-type]
            magnitude=float(payload.get("magnitude", 1.0)),
        )

    @staticmethod
    def _discrete_index(action: int | str) -> int:
        if isinstance(action, str):
            key = action.strip().lower()
            if key not in DISCRETE_ACTION_INDEX:
                raise ValueError(
                    f"unknown discrete action {action!r}; expected {DISCRETE_ACTION_NAMES}"
                )
            return DISCRETE_ACTION_INDEX[key]
        idx = int(action)
        if not (0 <= idx < len(DISCRETE_ACTION_NAMES)):
            raise ValueError(
                f"discrete action index {idx} out of range [0, {len(DISCRETE_ACTION_NAMES)})"
            )
        return idx

    # ------------------------------------------------------------------
    # conversions
    # ------------------------------------------------------------------
    @property
    def is_zero(self) -> bool:
        return self.magnitude <= _EPS or (
            abs(self.direction[0]) <= _EPS and abs(self.direction[1]) <= _EPS
        )

    @property
    def angle(self) -> float:
        """Heading in radians (0 for a zero action)."""
        dx, dy = self.direction
        return math.atan2(dy, dx) if not self.is_zero else 0.0

    def to_velocity(self, speed: float) -> np.ndarray:
        """World-frame velocity command ``direction * magnitude * speed``."""
        dx, dy = self.direction
        return np.array([dx * self.magnitude * float(speed),
                         dy * self.magnitude * float(speed)], dtype=np.float64)

    def to_array(self, dim: int = 2) -> np.ndarray:
        """``[dx*m, dy*m]`` - the vector form used by the ActionCodec path."""
        return self.to_velocity(1.0)[:dim]

    def as_discrete(self) -> int:
        """Nearest discrete action index (deterministic tie-break by index)."""
        if self.is_zero:
            return DISCRETE_ACTION_INDEX["stay"]
        unit = np.asarray(self.direction, dtype=np.float64)
        sims = DISCRETE_DIRECTIONS @ unit
        return int(np.argmax(sims))

    def scaled(self, factor: float) -> "Action":
        return Action(self.direction, self.magnitude * float(factor))

    def to_dict(self) -> dict:
        return {
            "direction": [self.direction[0], self.direction[1]],
            "magnitude": self.magnitude,
        }

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return (
            f"Action(dir=({self.direction[0]:+.3f},{self.direction[1]:+.3f}), "
            f"mag={self.magnitude:.3f})"
        )


def coerce_action(value: Any, *, as_discrete: bool = True) -> Action:
    """Best-effort conversion of an external action representation to ``Action``.

    Accepted: ``Action``, ``None``/``"stay"``, an int/str discrete action, a
    2-vector (velocity or direction), or a mapping with ``direction``.
    """
    if isinstance(value, Action):
        return value
    if value is None:
        return Action.zero()
    if isinstance(value, str):
        return Action.from_discrete(value)
    if isinstance(value, dict):
        return Action.from_dict(value)
    arr = np.asarray(value)
    if arr.ndim == 0:
        return Action.from_discrete(int(arr))
    flat = arr.reshape(-1)
    if flat.size == 2:
        return Action.from_vector(flat)
    if flat.size == 1:
        return Action.from_discrete(int(flat[0]))
    raise ValueError(f"cannot interpret {value!r} as an Action")


def combine(*actions: Action) -> Action:
    """Vector-sum several actions, normalising the direction again.

    Used by manual input: pressing ``up`` and ``right`` together must give the
    same speed as pressing either alone, not ``sqrt(2)`` times more.
    """
    vx = vy = 0.0
    mag = 0.0
    for a in actions:
        if a is None or a.is_zero:
            continue
        vx += a.direction[0] * a.magnitude
        vy += a.direction[1] * a.magnitude
        mag = max(mag, a.magnitude)
    if abs(vx) <= _EPS and abs(vy) <= _EPS:
        return Action.zero()
    return Action((vx, vy), mag)


__all__ = ["Action", "combine", "coerce_action"]
