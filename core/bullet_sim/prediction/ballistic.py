"""Analytic ballistics predictor - the zero-cost forecasting baseline.

Because every bullet obeys ``p(t) = p0 + v0*t + 0.5*a*t^2`` exactly, the future
can be produced without stepping the world at all.  This is both a strong
lower-bound for "how expensive must prediction be?" studies and the reference
any learned predictor must beat.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from bullet_sim.core.state import WorldSnapshot
from bullet_sim.entities.bullet import PROTOCOL_FIELDS
from bullet_sim.prediction.base import TrajectoryForecast, empty_forecast


class BallisticPredictor:
    """Constant-velocity / constant-acceleration extrapolation for every bullet."""

    name = "ballistic"

    def __init__(
        self,
        *,
        respect_ttl: bool = True,
        respect_bounds: bool = True,
        margin: float = 0.0,
    ) -> None:
        self.respect_ttl = bool(respect_ttl)
        self.respect_bounds = bool(respect_bounds)
        self.margin = float(margin)

    def predict(
        self,
        snapshot: WorldSnapshot,
        horizon: int,
        dt: float | None = None,
    ) -> TrajectoryForecast:
        h = int(horizon)
        step = float(dt if dt is not None else snapshot.env.dt)
        if h <= 0 or snapshot.bullet_count == 0:
            return empty_forecast(max(h, 0), step, self.name)

        pool = snapshot.bullets
        idx = pool.active_indices()
        d = pool.data
        n = int(idx.size)

        x0 = d["x"][idx]
        y0 = d["y"][idx]
        vx = d["vx"][idx]
        vy = d["vy"][idx]
        ax = d["ax"][idx]
        ay = d["ay"][idx]
        radii = d["radius"][idx]
        types = d["type_id"][idx]
        remaining = (d["ttl"][idx] - d["age"][idx]) if self.respect_ttl else np.full(n, np.inf)

        times = (np.arange(1, h + 1, dtype=np.float64)) * step

        # (n, h) broadcast of positions
        t = times[None, :]
        px = x0[:, None] + vx[:, None] * t + 0.5 * ax[:, None] * t * t
        py = y0[:, None] + vy[:, None] * t + 0.5 * ay[:, None] * t * t
        positions = np.stack([px, py], axis=-1)

        valid = np.ones((n, h), dtype=bool)
        if self.respect_ttl:
            valid &= t <= remaining[:, None]
        if self.respect_bounds:
            w = snapshot.env.field_w + self.margin
            hgt = snapshot.env.field_h + self.margin
            valid &= (px >= -self.margin) & (px <= w) & (py >= -self.margin) & (py <= hgt)

        return TrajectoryForecast(
            positions=positions,
            valid=valid,
            times=times,
            indices=idx.astype(np.int32),
            type_ids=types.astype(np.int16),
            radii=radii.astype(np.float64),
            dt=step,
            horizon=h,
            method=self.name,
        )

    def predict_player_path(
        self, snapshot: WorldSnapshot, action_vector: np.ndarray, horizon: int, dt: float | None = None
    ) -> np.ndarray:
        """Straight-line player path implied by a constant velocity command."""
        step = float(dt if dt is not None else snapshot.env.dt)
        v = np.asarray(action_vector, dtype=np.float64).reshape(2)
        ts = np.arange(1, int(horizon) + 1, dtype=np.float64)[:, None] * step
        return snapshot.player.position[None, :] + ts * v[None, :]

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "respect_ttl": self.respect_ttl,
            "respect_bounds": self.respect_bounds,
        }


__all__ = ["BallisticPredictor", "PROTOCOL_FIELDS"]
