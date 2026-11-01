"""Fixed-timestep clock.

Simulation time is *always* ``step_index * dt``.  Rendering frame rate is
decoupled: a renderer may draw 0, 1 or many simulation steps per displayed
frame, and offline simulation is free to run as fast as the CPU allows.
"""

from __future__ import annotations

from dataclasses import dataclass

from bullet_sim.core.errors import ConfigError

_DEFAULT_FPS = 120.0


@dataclass(frozen=True)
class FixedClock:
    """Immutable fixed-step time base."""

    dt: float
    fps: float

    def __post_init__(self) -> None:
        if not (self.dt > 0.0):
            raise ConfigError(f"dt must be > 0, got {self.dt!r}")
        if not (self.fps > 0.0):
            raise ConfigError(f"fps must be > 0, got {self.fps!r}")

    # -- constructors ------------------------------------------------------
    @classmethod
    def from_fps(cls, fps: float) -> "FixedClock":
        if not (fps > 0.0):
            raise ConfigError(f"fps must be > 0, got {fps!r}")
        return cls(dt=1.0 / float(fps), fps=float(fps))

    @classmethod
    def from_dt(cls, dt: float) -> "FixedClock":
        if not (dt > 0.0):
            raise ConfigError(f"dt must be > 0, got {dt!r}")
        return cls(dt=float(dt), fps=1.0 / float(dt))

    @classmethod
    def make(cls, *, dt: float | None = None, fps: float | None = None) -> "FixedClock":
        """Build from either ``dt`` or ``fps`` (``dt`` wins when both given)."""
        if dt is not None:
            return cls.from_dt(dt)
        if fps is not None:
            return cls.from_fps(fps)
        return cls.from_fps(_DEFAULT_FPS)

    # -- conversions -------------------------------------------------------
    def time_of(self, step_index: int) -> float:
        return float(step_index) * self.dt

    def step_of(self, seconds: float) -> int:
        """First step index at or after ``seconds`` (used by the scheduler)."""
        if seconds <= 0.0:
            return 0
        return int(round(float(seconds) / self.dt))

    def steps_for(self, seconds: float) -> int:
        """Number of steps covering ``seconds`` (at least ``1`` if positive)."""
        if seconds <= 0.0:
            return 0
        return max(1, int(round(float(seconds) / self.dt)))

    def advance(self, step_index: int, n: int = 1) -> int:
        return step_index + n
