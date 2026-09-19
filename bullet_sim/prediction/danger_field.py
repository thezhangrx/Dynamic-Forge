"""Future-state danger field.

For each forecast horizon step, bullets are rasterised into a coarse grid and
dilated by a safety radius, producing a per-step *risk map*.  This is the input
for "future danger field" and "future safe region" studies and for risk-aware
planning; it is deliberately kept outside the physics kernel.

Rasterisation is O(N + cells) per step (splat + separable max-dilation), so it
scales to thousands of bullets and hundreds of horizon steps.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from bullet_sim.core.state import WorldSnapshot


@dataclass
class DangerField:
    """``(H, rows, cols)`` risk grid over the simulation field."""

    grid: np.ndarray
    field_w: float
    field_h: float
    rows: int
    cols: int
    dt: float
    resolution: float
    safety_radius: float
    method: str = "splat_dilate"

    @property
    def horizon(self) -> int:
        return int(self.grid.shape[0])

    def at(self, h: int) -> np.ndarray:
        return self.grid[h]

    def max_over_time(self) -> np.ndarray:
        return self.grid.max(axis=0)

    def mean_over_time(self) -> np.ndarray:
        return self.grid.mean(axis=0)

    def safe_mask(self, threshold: float = 1e-6) -> np.ndarray:
        """``(H, rows, cols)`` boolean: cells free of danger at each step."""
        return self.grid <= threshold

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        cw = self.field_w / self.cols
        ch = self.field_h / self.rows
        c = int(np.clip(x / cw, 0, self.cols - 1))
        r = int(np.clip(y / ch, 0, self.rows - 1))
        return r, c

    def risk_at(self, x: float, y: float, h: int) -> float:
        r, c = self.cell_of(x, y)
        return float(self.grid[h, r, c])

    def risk_along(self, path: np.ndarray) -> np.ndarray:
        """Risk along an ``(H, 2)`` world-frame path, one value per step."""
        out = np.empty(len(path), dtype=np.float32)
        for h in range(min(len(path), self.horizon)):
            out[h] = self.risk_at(path[h, 0], path[h, 1], h)
        return out

    def to_dict(self) -> dict[str, Any]:
        return {
            "field_w": self.field_w,
            "field_h": self.field_h,
            "rows": self.rows,
            "cols": self.cols,
            "dt": self.dt,
            "resolution": self.resolution,
            "safety_radius": self.safety_radius,
            "horizon": self.horizon,
            "method": self.method,
        }

    def save(self, path: str) -> str:
        np.savez_compressed(
            path,
            grid=self.grid.astype(np.float32),
            meta=np.asarray(
                [self.field_w, self.field_h, self.rows, self.cols, self.dt,
                 self.resolution, self.safety_radius]
            ),
        )
        return path


def compute_danger_field(
    states: Sequence[WorldSnapshot] | Any,
    *,
    resolution: float = 12.0,
    safety_radius: float | None = None,
    decay: float = 1.0,
    normalize: bool = True,
) -> DangerField:
    """Build a danger field from a sequence of future states (S_t, S_t+1, ...).

    ``states`` is normally the output of ``world.simulate_future(...)`` or a
    :class:`~bullet_sim.prediction.rollout.FutureSimulator` rollout.
    """
    states = list(states)
    if not states:
        raise ValueError("compute_danger_field requires at least one state")
    ref = states[0]
    field_w = float(ref.env.field_w)
    field_h = float(ref.env.field_h)
    res = max(float(resolution), 1e-6)
    cols = max(1, int(np.ceil(field_w / res)))
    rows = max(1, int(np.ceil(field_h / res)))
    cell_w = field_w / cols
    cell_h = field_h / rows

    if safety_radius is None:
        safety_radius = 2.5 * res
    safety_cells = max(1, int(np.ceil(float(safety_radius) / min(cell_w, cell_h))))

    horizon = len(states)
    grid = np.zeros((horizon, rows, cols), dtype=np.float32)
    for h, snap in enumerate(states):
        pool = snap.bullets
        n = pool.count
        if n == 0:
            continue
        idx = pool.active_indices()
        d = pool.data
        x = d["x"][idx]
        y = d["y"][idx]
        cx = np.clip((x / cell_w).astype(np.int64), 0, cols - 1)
        cy = np.clip((y / cell_h).astype(np.int64), 0, rows - 1)
        layer = grid[h]
        np.add.at(layer.ravel(), cy * cols + cx, 1.0)
    # separable max dilation -> "near a bullet" risk, then decay in time
    if safety_cells > 0:
        grid = _dilate_axis(grid, safety_cells, axis=1)
        grid = _dilate_axis(grid, safety_cells, axis=2)
    if decay != 1.0:
        factors = np.power(float(decay), np.arange(horizon, dtype=np.float32))
        grid *= factors[:, None, None]
    if normalize:
        peak = float(grid.max())
        if peak > 0:
            grid = grid / peak
    return DangerField(
        grid=grid.astype(np.float32),
        field_w=field_w,
        field_h=field_h,
        rows=rows,
        cols=cols,
        dt=float(ref.env.dt),
        resolution=res,
        safety_radius=float(safety_radius),
    )


def _dilate_axis(a: np.ndarray, k: int, axis: int) -> np.ndarray:
    """Separable max-dilation by ``k`` cells along one axis (non-destructive)."""
    out = a.copy()
    for d in range(1, int(k) + 1):
        lo = [slice(None)] * a.ndim
        hi = [slice(None)] * a.ndim
        lo[axis] = slice(d, None)
        hi[axis] = slice(None, -d)
        np.maximum(out[tuple(lo)], a[tuple(hi)], out=out[tuple(lo)])
        lo[axis] = slice(None, -d)
        hi[axis] = slice(d, None)
        np.maximum(out[tuple(lo)], a[tuple(hi)], out=out[tuple(lo)])
    return out


def danger_from_forecast(
    forecast: Any,
    *,
    field_w: float,
    field_h: float,
    resolution: float = 12.0,
    safety_radius: float | None = None,
    normalize: bool = True,
) -> DangerField:
    """Rasterise a :class:`TrajectoryForecast` without stepping the world."""
    positions = np.asarray(forecast.positions)
    valid = np.asarray(forecast.valid, dtype=bool)
    rows, cols = _grid_shape(field_w, field_h, resolution)
    res = max(float(resolution), 1e-6)
    cell_w = field_w / cols
    cell_h = field_h / rows
    if safety_radius is None:
        safety_radius = 2.5 * res
    safety_cells = max(1, int(np.ceil(float(safety_radius) / min(cell_w, cell_h))))

    grid = np.zeros((int(forecast.horizon), rows, cols), dtype=np.float32)
    for step in range(int(forecast.horizon)):
        m = valid[:, step]
        if not m.any():
            continue
        p = positions[m, step, :]
        cx = np.clip((p[:, 0] / cell_w).astype(np.int64), 0, cols - 1)
        cy = np.clip((p[:, 1] / cell_h).astype(np.int64), 0, rows - 1)
        np.add.at(grid[step].ravel(), cy * cols + cx, 1.0)
    grid = _dilate_axis(grid, safety_cells, axis=1)
    grid = _dilate_axis(grid, safety_cells, axis=2)
    if normalize and grid.max() > 0:
        grid = grid / float(grid.max())
    return DangerField(
        grid=grid.astype(np.float32),
        field_w=field_w,
        field_h=field_h,
        rows=rows,
        cols=cols,
        dt=float(getattr(forecast, "dt", 0.0)),
        resolution=res,
        safety_radius=float(safety_radius),
        method="forecast_splat_dilate",
    )


def _grid_shape(field_w: float, field_h: float, resolution: float) -> tuple[int, int]:
    res = max(float(resolution), 1e-6)
    cols = max(1, int(np.ceil(field_w / res)))
    rows = max(1, int(np.ceil(field_h / res)))
    return rows, cols


__all__ = [
    "DangerField",
    "compute_danger_field",
    "danger_from_forecast",
]
