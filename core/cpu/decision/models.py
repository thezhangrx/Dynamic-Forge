"""Domain models for CPU-side decision making.

These are deliberately **game-agnostic**: nothing here knows about pixels, the
640x480 field, the discrete 9-way key table, or the ``BulletPool`` SoA layout.
A bullet-hell player and a real 2D cart both map onto the same abstractions::

    SceneState  = AgentState + obstacles + optional goal + timing
    AgentState  = position + velocity (+ radius, max speed)
    Obstacle    = position + velocity (+ radius)
    Action      = speed + steering_angle

Positions and velocities are plain 2D vectors in some world frame (no unit is
assumed); converting *to* and *from* a concrete simulator is the adapter's job
(see ``cpu.adapters.game_adapter``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

__all__ = ["AgentState", "Obstacle", "Action", "SceneState"]


def _vec2(values: object, *, name: str) -> np.ndarray:
    """Coerce ``values`` to a float64 2-vector."""
    v = np.asarray(values, dtype=np.float64).reshape(-1)
    if v.size != 2:
        raise ValueError(f"{name} must be a 2-vector, got shape {v.shape}")
    return v


def _pos_of(other: object) -> np.ndarray:
    if hasattr(other, "position"):
        return other.position  # type: ignore[union-attr]
    return _vec2(other, name="position")


@dataclass(frozen=True)
class AgentState:
    """A moving agent (robot / cart / player) in a 2D world frame."""

    position: np.ndarray
    velocity: np.ndarray
    radius: float = 1.0
    max_speed: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec2(self.position, name="position"))
        object.__setattr__(self, "velocity", _vec2(self.velocity, name="velocity"))
        if self.radius < 0.0 or self.max_speed < 0.0:
            raise ValueError("radius and max_speed must be non-negative")

    # -- scalar conveniences ------------------------------------------------
    @property
    def x(self) -> float:
        return float(self.position[0])

    @property
    def y(self) -> float:
        return float(self.position[1])

    @property
    def speed(self) -> float:
        return float(np.hypot(self.velocity[0], self.velocity[1]))

    # -- geometry -----------------------------------------------------------
    def distance_to(self, other: object) -> float:
        """Center-to-center distance to an :class:`Obstacle` / ``AgentState`` / vector."""
        p = _pos_of(other)
        return float(np.hypot(*(self.position - p)))

    def advance(self, dt: float) -> "AgentState":
        """Move forward ``dt`` under the current velocity (constant velocity)."""
        return AgentState(
            self.position + self.velocity * dt, self.velocity, self.radius, self.max_speed
        )


@dataclass(frozen=True)
class Obstacle:
    """A dynamic obstacle in a 2D world frame."""

    position: np.ndarray
    velocity: np.ndarray
    radius: float = 0.5

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec2(self.position, name="position"))
        object.__setattr__(self, "velocity", _vec2(self.velocity, name="velocity"))
        if self.radius < 0.0:
            raise ValueError("radius must be non-negative")

    @property
    def x(self) -> float:
        return float(self.position[0])

    @property
    def y(self) -> float:
        return float(self.position[1])

    @property
    def speed(self) -> float:
        return float(np.hypot(self.velocity[0], self.velocity[1]))

    def advance(self, dt: float) -> "Obstacle":
        """Move forward ``dt`` under constant velocity (linear motion)."""
        return Obstacle(self.position + self.velocity * dt, self.velocity, self.radius)


@dataclass(frozen=True)
class Action:
    """A generic motion command, expressed as ``speed`` + ``steering_angle``.

    * ``speed``           - desired speed magnitude (>= 0, world units / s)
    * ``steering_angle``  - heading in radians, counter-clockwise from ``+x``

    There are **no** game keys here (no LEFT / RIGHT / UP): a heading is a
    continuous angle, and a candidate set is just a list of these.  This keeps
    the planner identical for a bullet-hell player and a 2D cart.
    """

    speed: float = 0.0
    steering_angle: float = 0.0

    def __post_init__(self) -> None:
        speed = float(self.speed)
        angle = float(self.steering_angle)
        if not math.isfinite(speed) or speed < 0.0:
            raise ValueError(f"speed must be finite and >= 0, got {speed!r}")
        if not math.isfinite(angle):
            raise ValueError(f"steering_angle must be finite, got {angle!r}")
        object.__setattr__(self, "speed", speed)
        object.__setattr__(self, "steering_angle", angle)

    # -- conversions --------------------------------------------------------
    @property
    def velocity_vector(self) -> np.ndarray:
        """World-frame velocity implied by ``(speed, steering_angle)``."""
        return np.array(
            [
                math.cos(self.steering_angle) * self.speed,
                math.sin(self.steering_angle) * self.speed,
            ],
            dtype=np.float64,
        )

    @property
    def is_stop(self) -> bool:
        return self.speed == 0.0

    @classmethod
    def stop(cls) -> "Action":
        return cls(0.0, 0.0)

    def to_dict(self) -> dict:
        return {"speed": self.speed, "steering_angle": self.steering_angle}


@dataclass(frozen=True)
class SceneState:
    """One planning frame: the agent, the obstacles around it, and timing.

    ``goal`` is optional; when present it is a 2D point the planner may steer
    toward once safety is equal (a soft progress term, off by default).
    """

    agent: AgentState
    obstacles: tuple[Obstacle, ...] = ()
    goal: np.ndarray | None = None
    dt: float = 1.0 / 120.0
    horizon: float = 0.75

    def __post_init__(self) -> None:
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        if self.goal is not None:
            object.__setattr__(self, "goal", _vec2(self.goal, name="goal"))
        if self.dt <= 0.0:
            raise ValueError("dt must be > 0")
        if self.horizon < 0.0:
            raise ValueError("horizon must be >= 0")

    @property
    def n_steps(self) -> int:
        """Number of prediction steps for ``horizon`` at ``dt``."""
        return max(1, int(round(self.horizon / self.dt)))

    def __len__(self) -> int:
        return len(self.obstacles)
