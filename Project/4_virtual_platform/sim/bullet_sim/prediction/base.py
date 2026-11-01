"""Prediction interfaces.

Prediction is a *layer on top of* the physics kernel, never part of it.  A
predictor receives an immutable ``WorldSnapshot`` and returns a forecast; it may
be analytic (zero-cost), rollout-based (uses ``World.clone()``), learned, or a
CPU/FPGA accelerator result delivered through ``interface/hardware.py``.

    S_t --predict--> future trajectories --derive--> danger field / safe region
                                              --feed--> planning / decisions
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

import numpy as np

from bullet_sim.core.state import WorldSnapshot


@dataclass
class TrajectoryForecast:
    """Per-bullet future positions produced by any predictor."""

    #: ``(n_bullets, horizon, 2)`` world-frame positions.
    positions: np.ndarray
    #: ``(n_bullets, horizon)`` validity (alive and still inside the field).
    valid: np.ndarray
    #: ``(horizon,)`` times in seconds relative to ``t0``.
    times: np.ndarray
    #: ``(n_bullets,)`` originating slot ids.
    indices: np.ndarray
    #: ``(n_bullets,)`` bullet type ids.
    type_ids: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int16))
    #: ``(n_bullets,)`` bullet radii.
    radii: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dt: float = 0.0
    horizon: int = 0
    method: str = "unknown"

    def at(self, h: int) -> np.ndarray:
        """Positions at horizon index ``h`` for currently-valid bullets."""
        mask = self.valid[:, h]
        return self.positions[mask, h, :]

    def final_positions(self) -> np.ndarray:
        return self.at(self.horizon - 1) if self.horizon else np.zeros((0, 2))

    @property
    def n_bullets(self) -> int:
        return int(self.positions.shape[0])

    def summary(self) -> dict[str, Any]:
        return {
            "method": self.method,
            "n_bullets": self.n_bullets,
            "horizon": int(self.horizon),
            "dt": float(self.dt),
            "horizon_seconds": float(self.horizon * self.dt),
            "mean_valid_fraction": float(self.valid.mean()) if self.valid.size else 0.0,
        }


@runtime_checkable
class Predictor(Protocol):
    """Anything that can forecast the future of a snapshot."""

    name: str

    def predict(
        self, snapshot: WorldSnapshot, horizon: int, dt: float | None = None
    ) -> TrajectoryForecast: ...


def empty_forecast(horizon: int = 0, dt: float = 0.0, method: str = "empty") -> TrajectoryForecast:
    return TrajectoryForecast(
        positions=np.zeros((0, horizon, 2)),
        valid=np.zeros((0, horizon), dtype=bool),
        times=np.arange(horizon, dtype=np.float64) * dt,
        indices=np.zeros(0, dtype=np.int32),
        dt=dt,
        horizon=horizon,
        method=method,
    )


__all__ = ["TrajectoryForecast", "Predictor", "empty_forecast"]
