"""Time-windowed safe-path search over the free-space grid.

The search is a **time-expanded reachability flood fill**:

* discretise the future into ``n_samples`` slices of ``dt``;
* at each slice, rasterise the *actual* obstacle layout (from the real
  simulator, not a re-implementation) and inflate it by
  ``player_circumradius + safety_margin``;
* propagate the set of cells the player can occupy while respecting
  ``max_speed`` (and ``max_accel`` when given), using a Euclidean movement
  structuring element so a diagonal move may not exceed the distance actually
  travellable in one slice;
* intersect with the free space of that slice;
* backtrack one feasible trajectory.

Because every slice uses the real free space, a path returned here is a path the
player can *actually* fly, and "the scenario is feasible" means "there exists a
sequence of positions satisfying the kinematic limits and avoiding every
obstacle at every sampled instant".

The flood fill is vectorised with numpy shifts, so a 60-slice window over a
32x24 grid costs a few milliseconds - cheap enough to run inside scenario
generation *and* at runtime for the AI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from bullet_sim.safety.free_space import FreeSpaceGrid


@dataclass
class SafePath:
    """A feasible collision-free trajectory in grid and world coordinates."""

    cells: list[tuple[int, int]] = field(default_factory=list)
    times: list[float] = field(default_factory=list)
    clearances: list[float] = field(default_factory=list)
    grid: FreeSpaceGrid | None = None
    strategy: str = "generic"

    @property
    def length(self) -> int:
        return len(self.cells)

    @property
    def world_points(self) -> list[tuple[float, float]]:
        if self.grid is None:
            return []
        return [self.grid.cell_to_world(r, c) for r, c in self.cells]

    @property
    def min_clearance(self) -> float:
        return min(self.clearances) if self.clearances else 0.0

    def displacement(self) -> float:
        pts = self.world_points
        if len(pts) < 2:
            return 0.0
        return float(
            sum(
                float(np.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
                for i in range(len(pts) - 1)
            )
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "length": self.length,
            "cells": [list(c) for c in self.cells],
            "times": [round(t, 4) for t in self.times],
            "min_clearance": round(self.min_clearance, 4),
            "displacement": round(self.displacement(), 4),
            "strategy": self.strategy,
        }


def movement_kernel(
    radius: float,
    cell_w: float,
    cell_h: float,
    *,
    tol: float = 0.5,
    allow_stay: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Offsets reachable within the physical distance ``radius``.

    Returns ``(dr, dc, distance)``.  A half-cell tolerance lets sub-cell
    progress still reach the neighbouring cell *centre*; without it a slice
    shorter than one cell would freeze the flood fill entirely (a modelling
    artefact, not a physical limit).
    """
    radius = max(0.0, float(radius))
    slack = float(tol) * min(cell_w, cell_h)
    r_max = int(np.ceil(radius / max(cell_h, 1e-9))) + 1
    c_max = int(np.ceil(radius / max(cell_w, 1e-9))) + 1
    drs, dcs, dists = [], [], []
    for dr in range(-r_max, r_max + 1):
        for dc in range(-c_max, c_max + 1):
            if dr == 0 and dc == 0:
                continue
            d = float(np.hypot(dr * cell_h, dc * cell_w))
            if d <= radius + slack:
                drs.append(dr)
                dcs.append(dc)
                dists.append(d)
    if allow_stay:
        drs.append(0)
        dcs.append(0)
        dists.append(0.0)
    if not drs:
        drs, dcs, dists = [0], [0], [0.0]
    return (
        np.asarray(drs, dtype=np.int64),
        np.asarray(dcs, dtype=np.int64),
        np.asarray(dists, dtype=np.float64),
    )


def _shift_or(mask: np.ndarray, dr: int, dc: int) -> np.ndarray:
    out = np.zeros_like(mask)
    rows, cols = mask.shape
    r0, r1 = max(0, dr), min(rows, rows + dr)
    c0, c1 = max(0, dc), min(cols, cols + dc)
    if r0 >= r1 or c0 >= c1:
        return out
    out[r0:r1, c0:c1] = mask[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
    return out


def dilate_with_offsets(mask: np.ndarray, drs: np.ndarray, dcs: np.ndarray) -> np.ndarray:
    out = np.zeros_like(mask)
    for dr, dc in zip(drs.tolist(), dcs.tolist()):
        out |= _shift_or(mask, int(dr), int(dc))
    return out


def _shift_min(arr: np.ndarray, dr: int, dc: int) -> np.ndarray:
    """``out[r, c] = arr[r - dr, c - dc]`` (edges = +inf)."""
    out = np.full_like(arr, np.inf)
    rows, cols = arr.shape
    r0, r1 = max(0, dr), min(rows, rows + dr)
    c0, c1 = max(0, dc), min(cols, cols + dc)
    if r0 >= r1 or c0 >= c1:
        return out
    out[r0:r1, c0:c1] = arr[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
    return out


@dataclass
class ReachabilityResult:
    """Outcome of the time-expanded flood fill."""

    reachable_layers: list[np.ndarray] = field(default_factory=list)
    reachable_final: np.ndarray | None = None
    reachable_anytime: np.ndarray | None = None
    times: list[float] = field(default_factory=list)
    start_cell: tuple[int, int] = (0, 0)
    start_free: bool = True
    reason: str = ""
    strategy: str = "generic"
    #: Set by the validator: how many obstacles existed at the densest sample.
    peak_obstacles: int = 0

    @property
    def feasible(self) -> bool:
        return bool(self.start_free and self.reachable_final is not None
                    and self.reachable_final.any())

    @property
    def reachable_fraction(self) -> float:
        if self.reachable_final is None or self.reachable_final.size == 0:
            return 0.0
        return float(self.reachable_final.mean())

    @property
    def reachable_cells(self) -> int:
        return 0 if self.reachable_final is None else int(self.reachable_final.sum())

    def safe_region(self) -> np.ndarray:
        """Cells the player can reach at *some* point in the window."""
        if self.reachable_anytime is None:
            return np.zeros((0, 0), dtype=bool)
        return self.reachable_anytime

    def to_dict(self) -> dict[str, Any]:
        return {
            "feasible": self.feasible,
            "start_free": bool(self.start_free),
            "reachable_cells": self.reachable_cells,
            "reachable_fraction": round(self.reachable_fraction, 4),
            "samples": len(self.reachable_layers),
            "horizon": self.times[-1] if self.times else 0.0,
            "reason": self.reason,
            "strategy": self.strategy,
        }


def reachable_set(
    layers: Sequence[np.ndarray],
    *,
    start_cell: tuple[int, int],
    dt: float,
    max_speed: float,
    cell_w: float,
    cell_h: float,
    max_accel: float | None = None,
    initial_speed: float | None = None,
    passes: int | None = None,
) -> ReachabilityResult:
    """Time-expanded reachability flood fill with kinematic limits.

    Propagates an **arrival time** per cell rather than a boolean, so progress
    smaller than one grid cell is never lost: a player travelling 6 px per slice
    across 16 px cells still reaches the neighbouring cell after three slices.
    A boolean dilation would silently freeze, which is a discretisation bug and
    would make feasible scenes look infeasible.
    """
    if not layers:
        return ReachabilityResult(reason="no free-space layers", start_cell=start_cell)
    rows, cols = layers[0].shape
    r0, c0 = int(start_cell[0]), int(start_cell[1])
    if not layers[0][r0, c0]:
        return ReachabilityResult(
            start_free=False, start_cell=start_cell,
            reason="the player starts inside an inflated obstacle",
        )

    speed = float(max_speed)
    v0 = speed if initial_speed is None else min(float(initial_speed), speed)
    # per-slice travel distance, honouring an acceleration limit
    if max_accel is not None and np.isfinite(max_accel) and max_accel > 0.0:
        v_end = min(speed, v0 + float(max_accel) * dt)
        step = 0.5 * (v0 + v_end) * dt
    else:
        step = speed * dt
    drs, dcs, dists = movement_kernel(step, cell_w, cell_h)
    n_passes = passes if passes is not None else max(2, int(np.ceil(step / min(cell_w, cell_h))) + 2)

    arrival = np.full((rows, cols), np.inf, dtype=np.float64)
    arrival[r0, c0] = 0.0
    reachable_layers: list[np.ndarray] = []
    anytime = np.zeros((rows, cols), dtype=bool)
    times: list[float] = []

    for k, layer in enumerate(layers):
        t_k = (k + 1) * float(dt)
        # A cell that is blocked at t_k cannot be occupied *now*, so it must not
        # act as a source for this slice.  Without this invalidation the wavefront
        # keeps the finite arrival time a cell had in an earlier slice and leaks
        # straight through a moving wall - accepting scenes that are lethal.
        arrival = np.where(layer, arrival, np.inf)
        # cells usable at t_k: free now, and already reached by then
        usable = layer & (arrival <= t_k + 1e-9)
        current = usable.copy()
        for _ in range(int(n_passes)):
            relaxed = arrival.copy()
            for dr, dc, dist in zip(drs.tolist(), dcs.tolist(), dists.tolist()):
                cand = _shift_min(arrival, int(dr), int(dc)) + (dist / max(speed, 1e-9))
                np.minimum(relaxed, cand, out=relaxed)
            # a cell may only be entered if it is free at this instant
            relaxed = np.where(layer, relaxed, np.inf)
            changed = relaxed < arrival - 1e-12
            if not changed.any():
                break
            arrival = np.where(changed, relaxed, arrival)
            current = layer & (arrival <= t_k + 1e-9)
            if not current.any():
                break
        reachable_layers.append(current.copy())
        anytime |= current
        times.append(t_k)
        if not current.any():
            return ReachabilityResult(
                reachable_layers=reachable_layers, reachable_final=current,
                reachable_anytime=anytime, times=times,
                start_cell=start_cell, start_free=True,
                reason=f"the reachable set became empty after {k + 1} slice(s) "
                       "- the corridor closes before the horizon",
            )

    final = reachable_layers[-1] if reachable_layers else np.zeros((rows, cols), dtype=bool)
    return ReachabilityResult(
        reachable_layers=reachable_layers,
        reachable_final=final,
        reachable_anytime=anytime,
        times=times,
        start_cell=start_cell,
        start_free=True,
        reason="feasible path found" if final.any() else "no reachable cell at the horizon",
    )


def backtrack_path(
    reach: ReachabilityResult,
    layers: Sequence[np.ndarray],
    *,
    dt: float,
    target_cell: tuple[int, int] | None = None,
    grid: FreeSpaceGrid | None = None,
) -> SafePath | None:
    """Recover one concrete trajectory from a successful reachability result."""
    if not reach.feasible or not reach.reachable_layers:
        return None
    final = reach.reachable_final
    rows, cols = final.shape
    if target_cell is not None and final[target_cell[0], target_cell[1]]:
        cur = (int(target_cell[0]), int(target_cell[1]))
    else:
        rr, cc = np.nonzero(final)
        # The point of a "safe path" is to make progress: pick the reachable
        # cell farthest (in travel terms) from the start, not the nearest one,
        # which would trivially be the start cell itself.
        r0, c0 = reach.start_cell
        d = np.hypot((rr - r0) * 1.0, (cc - c0) * 1.0)
        j = int(np.argmax(d))
        cur = (int(rr[j]), int(cc[j]))

    path_cells = [cur]
    for k in range(len(reach.reachable_layers) - 1, 0, -1):
        prev_layer = reach.reachable_layers[k - 1]
        pr, pc = np.nonzero(prev_layer)
        if pr.size == 0:
            break
        d = (pr - cur[0]) ** 2 + (pc - cur[1]) ** 2
        j = int(np.argmin(d))
        cur = (int(pr[j]), int(pc[j]))
        path_cells.append(cur)
    path_cells.append(reach.start_cell)
    path_cells.reverse()

    times = [i * dt for i in range(len(path_cells))]
    scale = (grid.cell_w if grid is not None else 1.0)
    clearances: list[float] = []
    for i, (r, c) in enumerate(path_cells):
        # path index 0 is the start (before any slice); index i>=1 sits in
        # reachable layer i-1
        layer = layers[min(max(i - 1, 0), len(layers) - 1)]
        clearances.append(float(_cell_clearance(layer, r, c)) * scale)
    return SafePath(
        cells=path_cells, times=times, clearances=clearances, grid=grid,
        strategy=reach.strategy,
    )


def _cell_clearance(layer: np.ndarray, r: int, c: int, *, max_ring: int = 6) -> int:
    """Chebyshev distance (in cells) from ``(r, c)`` to the nearest blocked cell."""
    rows, cols = layer.shape
    for ring in range(1, max_ring + 1):
        r0, r1 = max(0, r - ring), min(rows, r + ring + 1)
        c0, c1 = max(0, c - ring), min(cols, c + ring + 1)
        window = layer[r0:r1, c0:c1]
        if not window.all():
            return ring
    return max_ring


__all__ = [
    "SafePath",
    "ReachabilityResult",
    "movement_kernel",
    "_shift_min",
    "dilate_with_offsets",
    "reachable_set",
    "backtrack_path",
]
