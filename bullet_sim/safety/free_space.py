"""Free space / safe region on a grid, with exact obstacle inflation.

The question this module answers is *not* "do the shapes overlap right now".
It is: **"which cells can the player's collision volume occupy without
touching anything?"** - i.e. configuration space, not workspace.

Inflation is exact rather than approximate: a cell is free iff

    sdf(obstacle, cell_centre) >= player_circumradius + safety_margin

for every obstacle, using the same signed-distance functions the collision
system uses.  That guarantees the grid never reports "free" for a cell the
player could not actually occupy, which is the conservative direction safety
analysis needs.

Two analysis levels are provided, as requested:

* **Level 1** - :func:`rasterize_free_space` on one instant (static geometry);
* **Level 2** - :class:`FreeSpaceSequence`, the same grid evaluated over a
  future time window (dynamic free space).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from bullet_sim.collision.obstacles import obstacle_sdf_matrix
from bullet_sim.entities.bullet import BulletPool

#: 8-connected neighbourhood offsets (including "stay").
NEIGHBOURS_8: tuple[tuple[int, int], ...] = (
    (0, 0), (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)

#: 4-connected neighbourhood, used when diagonal squeezing must be forbidden.
NEIGHBOURS_4: tuple[tuple[int, int], ...] = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))


@dataclass
class FreeSpaceGrid:
    """Boolean traversability grid for one instant."""

    free: np.ndarray
    rows: int
    cols: int
    cell_w: float
    cell_h: float
    origin_x: float
    origin_y: float
    inflate: float = 0.0
    occupied_fraction: float = 0.0

    # ------------------------------------------------------------------
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        c = int(np.floor((float(x) - self.origin_x) / self.cell_w))
        r = int(np.floor((float(y) - self.origin_y) / self.cell_h))
        return (
            int(np.clip(r, 0, self.rows - 1)),
            int(np.clip(c, 0, self.cols - 1)),
        )

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        return (
            self.origin_x + (c + 0.5) * self.cell_w,
            self.origin_y + (r + 0.5) * self.cell_h,
        )

    def is_free(self, x: float, y: float) -> bool:
        r, c = self.world_to_cell(x, y)
        return bool(self.free[r, c])

    @property
    def free_fraction(self) -> float:
        return float(self.free.mean()) if self.free.size else 0.0

    @property
    def free_cells(self) -> int:
        return int(self.free.sum())

    # ------------------------------------------------------------------
    def components(self, *, diagonal: bool = True) -> tuple[np.ndarray, int]:
        """Label connected free regions (``0`` = occupied)."""
        labels = np.zeros((self.rows, self.cols), dtype=np.int32)
        offsets = NEIGHBOURS_8 if diagonal else NEIGHBOURS_4
        current = 0
        for r0 in range(self.rows):
            for c0 in range(self.cols):
                if not self.free[r0, c0] or labels[r0, c0]:
                    continue
                current += 1
                stack = [(r0, c0)]
                labels[r0, c0] = current
                while stack:
                    r, c = stack.pop()
                    for dr, dc in offsets:
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < self.rows and 0 <= cc < self.cols:
                            if self.free[rr, cc] and not labels[rr, cc]:
                                labels[rr, cc] = current
                                stack.append((rr, cc))
        return labels, current

    def largest_component(self, *, diagonal: bool = True) -> np.ndarray:
        labels, count = self.components(diagonal=diagonal)
        if count == 0:
            return np.zeros_like(self.free)
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        biggest = int(np.argmax(sizes))
        return labels == biggest

    # ------------------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        return {
            "rows": self.rows,
            "cols": self.cols,
            "resolution": [round(self.cell_w, 3), round(self.cell_h, 3)],
            "inflate": round(self.inflate, 3),
            "free_fraction": round(self.free_fraction, 4),
            "free_cells": self.free_cells,
            "total_cells": int(self.rows * self.cols),
        }


def grid_shape(field_w: float, field_h: float, resolution: float) -> tuple[int, int, float, float]:
    res = max(1.0, float(resolution))
    cols = max(2, int(np.ceil(field_w / res)))
    rows = max(2, int(np.ceil(field_h / res)))
    return rows, cols, field_w / cols, field_h / rows


def cell_centres(
    rows: int, cols: int, cell_w: float, cell_h: float
) -> tuple[np.ndarray, np.ndarray]:
    xs = (np.arange(cols) + 0.5) * cell_w
    ys = (np.arange(rows) + 0.5) * cell_h
    gx, gy = np.meshgrid(xs, ys)
    return gx.reshape(-1), gy.reshape(-1)


def occupancy_mask(
    pool: BulletPool,
    *,
    rows: int,
    cols: int,
    cell_w: float,
    cell_h: float,
    inflate: float,
    index: np.ndarray | None = None,
) -> np.ndarray:
    """``(rows, cols)`` mask of cells that are **not** traversable.

    Exact configuration-space inflation: a cell is blocked iff any obstacle's
    surface is closer than ``inflate`` to the cell centre.
    """
    blocked = np.zeros(rows * cols, dtype=bool)
    if index is None:
        index = pool.active_indices()
    if index.size == 0:
        return blocked.reshape(rows, cols)
    gx, gy = cell_centres(rows, cols, cell_w, cell_h)
    d = pool.data
    ox_all = d["x"][index].astype(np.float64)
    oy_all = d["y"][index].astype(np.float64)
    ext = np.hypot(
        d["half_w"][index].astype(np.float64), d["half_h"][index].astype(np.float64)
    )
    # cheap prefilter: only obstacles whose bounding box can reach the grid
    x0, x1 = gx.min() - float(inflate), gx.max() + float(inflate)
    y0, y1 = gy.min() - float(inflate), gy.max() + float(inflate)
    near = (ox_all + ext >= x0) & (ox_all - ext <= x1) & (oy_all + ext >= y0) & (oy_all - ext <= y1)
    if near.any():
        idx = index[near]
        sdf = obstacle_sdf_matrix(
            gx, gy,
            d["x"][idx].astype(np.float64), d["y"][idx].astype(np.float64),
            d["shape"][idx], d["radius"][idx].astype(np.float64),
            d["half_w"][idx].astype(np.float64), d["half_h"][idx].astype(np.float64),
            d["rotation"][idx].astype(np.float64),
        )
        blocked = np.any(sdf < float(inflate), axis=1)
    return blocked.reshape(rows, cols)


def rasterize_free_space(
    pool: BulletPool,
    *,
    field_w: float,
    field_h: float,
    inflate: float,
    resolution: float = 16.0,
    index: np.ndarray | None = None,
) -> FreeSpaceGrid:
    """Level-1 analysis: traversable cells for the *current* obstacle layout."""
    rows, cols, cw, ch = grid_shape(field_w, field_h, resolution)
    blocked = occupancy_mask(
        pool, rows=rows, cols=cols, cell_w=cw, cell_h=ch,
        inflate=inflate, index=index,
    )
    free = ~blocked
    return FreeSpaceGrid(
        free=free, rows=rows, cols=cols, cell_w=cw, cell_h=ch,
        origin_x=0.0, origin_y=0.0, inflate=float(inflate),
        occupied_fraction=float(blocked.mean()) if blocked.size else 0.0,
    )


@dataclass
class FreeSpaceSequence:
    """Level-2 analysis: free space sampled over a future time window."""

    grids: list[FreeSpaceGrid] = field(default_factory=list)
    times: list[float] = field(default_factory=list)
    dt: float = 0.0

    def __len__(self) -> int:
        return len(self.grids)

    @property
    def rows(self) -> int:
        return self.grids[0].rows if self.grids else 0

    @property
    def cols(self) -> int:
        return self.grids[0].cols if self.grids else 0

    def free_fraction_profile(self) -> list[float]:
        return [g.free_fraction for g in self.grids]

    def intersection(self) -> np.ndarray:
        """Cells free in **every** sampled instant (always-safe cells)."""
        if not self.grids:
            return np.zeros((0, 0), dtype=bool)
        out = self.grids[0].free.copy()
        for g in self.grids[1:]:
            out &= g.free
        return out

    def describe(self) -> dict[str, Any]:
        profile = self.free_fraction_profile()
        return {
            "samples": len(self.grids),
            "dt": self.dt,
            "horizon": self.times[-1] if self.times else 0.0,
            "free_fraction_start": round(profile[0], 4) if profile else 0.0,
            "free_fraction_min": round(min(profile), 4) if profile else 0.0,
            "free_fraction_end": round(profile[-1], 4) if profile else 0.0,
            "always_free_fraction": round(float(self.intersection().mean()), 4)
            if self.grids
            else 0.0,
        }


__all__ = [
    "FreeSpaceGrid",
    "FreeSpaceSequence",
    "NEIGHBOURS_4",
    "NEIGHBOURS_8",
    "cell_centres",
    "grid_shape",
    "occupancy_mask",
    "rasterize_free_space",
]
