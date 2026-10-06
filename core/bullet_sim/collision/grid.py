"""Uniform-grid broadphase collision backend.

Purpose
-------
For a *single* player query the brute-force :class:`~bullet_sim.collision.circle.CircleCollision`
is already optimal in NumPy (one vectorised pass over N bullets), so this
backend delegates single queries to it.  The grid pays off for **batch**
queries - many players against the same bullet field (vectorised RL envs,
policy search, multi-agent scenes) - where the CSR bucket layout turns an
``O(P*N)`` problem into ``O(P * bullets_per_cell)``.

The bucket layout is a counting-sort CSR structure: ``order`` holds live slot
indices grouped by cell, ``start`` holds per-cell offsets.  Both are rebuilt
once per simulation step via :meth:`prepare`.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from bullet_sim.collision.base import CollisionModel, CollisionResult, CollisionStats
from bullet_sim.collision.circle import CircleCollision
from bullet_sim.entities.bullet import BulletPool

class UniformGridCollision(CollisionModel):
    name = "grid"

    def __init__(self, cell_size: float | None = None, strict: bool = True) -> None:
        self.cell_size = cell_size
        self.strict = bool(strict)
        self._fallback = CircleCollision(strict=strict)
        self._stats = CollisionStats()
        self._field_w = 0.0
        self._field_h = 0.0
        self._margin = 0.0
        self._cell = 1.0
        self._cols = 1
        self._rows = 1
        self._start = np.zeros(1, dtype=np.int64)
        self._order = np.empty(0, dtype=np.int64)
        self._max_r = 1.0

    # ------------------------------------------------------------------
    def configure(self, field_w: float, field_h: float, margin: float = 0.0) -> None:
        self._field_w = float(field_w)
        self._field_h = float(field_h)
        self._margin = float(margin)
        self._rebuild_dims()

    def _rebuild_dims(self) -> None:
        cell = self.cell_size if self.cell_size else max(4.0 * self._max_r, 1.0)
        self._cell = max(float(cell), 1e-9)
        self._cols = max(1, int(np.ceil((self._field_w + 2 * self._margin) / self._cell)))
        self._rows = max(1, int(np.ceil((self._field_h + 2 * self._margin) / self._cell)))
        self._start = np.zeros(self._cols * self._rows + 1, dtype=np.int64)

    # ------------------------------------------------------------------
    def prepare(self, pool: BulletPool) -> None:
        idx = pool.active_indices()
        n = int(idx.size)
        if self._field_w <= 0.0:
            # Never configured explicitly: derive bounds from the bullets so the
            # backend still behaves correctly when used standalone.
            if n == 0:
                return
            d = pool.data
            pad = float(np.max(d["radius"][idx])) * 4.0 + 1.0
            self.configure(
                float(np.max(d["x"][idx])) + pad, float(np.max(d["y"][idx])) + pad, pad
            )
        if n == 0:
            self._order = np.empty(0, dtype=np.int64)
            self._start[:] = 0
            return
        d = pool.data
        max_r = float(np.max(d["radius"][idx])) if n else 1.0
        if self.cell_size is None and max_r > self._max_r * 1.5:
            self._max_r = max_r
            self._rebuild_dims()
        elif self.cell_size is None:
            self._max_r = max(self._max_r, max_r)
        cells = self._cell_ids(d["x"][idx], d["y"][idx])
        ncells = self._cols * self._rows
        counts = np.bincount(cells, minlength=ncells)
        start = np.zeros(ncells + 1, dtype=np.int64)
        np.cumsum(counts, out=start[1:])
        order = np.empty(n, dtype=np.int64)
        # Counting sort; np.argsort(kind='stable') on cells preserves slot order
        # inside each bucket, which keeps query results deterministic.
        order[:] = idx[np.argsort(cells, kind="stable")]
        self._start = start
        self._order = order

    def _cell_ids(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        cx = np.floor((x + self._margin) / self._cell).astype(np.int64)
        cy = np.floor((y + self._margin) / self._cell).astype(np.int64)
        np.clip(cx, 0, self._cols - 1, out=cx)
        np.clip(cy, 0, self._rows - 1, out=cy)
        return cy * self._cols + cx

    # ------------------------------------------------------------------
    def query(
        self,
        pool: BulletPool,
        px: float,
        py: float,
        pr: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        # Single-query: brute force is cheaper than building/descending buckets.
        self._stats.queries += 1
        res = self._fallback.query(pool, px, py, pr, frame=frame)
        self._stats.candidate_tests += self._fallback.stats().candidate_tests
        return res

    def query_many(
        self, pool: BulletPool, px: np.ndarray, py: np.ndarray, pr: np.ndarray
    ) -> np.ndarray:
        """Vectorised batch query for ``P`` players -> ``bool[P]``."""
        p = np.asarray(px, dtype=np.float64).reshape(-1)
        q = np.asarray(py, dtype=np.float64).reshape(-1)
        r = np.asarray(pr, dtype=np.float64).reshape(-1)
        P = p.size
        if pool.count == 0 or P == 0 or self._order.size == 0:
            return np.zeros(P, dtype=bool)
        self._stats.queries += P
        d = pool.data

        # A player can only touch bullets whose centre is within
        # (pr + max_bullet_radius); with a >= 4*max_r cell size the 3x3
        # neighbourhood is guaranteed to be sufficient.
        base = self._cell_ids(p, q)
        reach = int(np.ceil((float(np.max(r)) + self._max_r * 2.0) / self._cell))

        pid_list = []
        slot_list = []
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                cxi = (base % self._cols) + dx
                cyi = (base // self._cols) + dy
                ok = (cxi >= 0) & (cxi < self._cols) & (cyi >= 0) & (cyi < self._rows)
                if not ok.any():
                    continue
                cells = cyi[ok] * self._cols + cxi[ok]
                pidx = np.flatnonzero(ok)
                counts = self._start[cells + 1] - self._start[cells]
                total = int(counts.sum())
                if total == 0:
                    continue
                reps = np.repeat(pidx, counts)
                offs = np.repeat(self._start[cells], counts)
                within = np.arange(total, dtype=np.int64) - np.repeat(
                    np.cumsum(counts) - counts, counts
                )
                pid_list.append(reps)
                slot_list.append(self._order[offs + within])

        if not pid_list:
            return np.zeros(P, dtype=bool)

        pidx = np.concatenate(pid_list)
        slots = np.concatenate(slot_list)
        self._stats.candidate_tests += int(slots.size)
        ddx = d["x"][slots] - p[pidx]
        ddy = d["y"][slots] - q[pidx]
        d2 = ddx * ddx + ddy * ddy
        rr = d["radius"][slots] + r[pidx]
        hit = d2 < rr * rr if self.strict else d2 <= rr * rr
        out = np.zeros(P, dtype=bool)
        if hit.any():
            out[pidx[hit]] = True
        return out

    # ------------------------------------------------------------------
    def stats(self) -> CollisionStats:
        return self._stats

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "cell_size": self.cell_size,
            "cells": int(self._cols * self._rows),
            "broadphase": "uniform_grid",
            "strict": self.strict,
        }


def make_collision_model(kind: str = "circle", **kwargs: Any) -> CollisionModel:
    """Factory for configs/CLI.

    ``circle`` | ``grid`` | ``null`` | ``circle_distance`` | ``shaped`` |
    ``obstacle``.
    ``shaped`` supports circle / point / rect player hitboxes against circular
    obstacles; ``obstacle`` additionally supports rotated-rectangle obstacles
    and reports relative-motion contact rates.
    """
    from bullet_sim.collision.circle import DistanceCollision
    from bullet_sim.collision.base import NullCollision
    from bullet_sim.collision.obstacles import ObstacleCollision
    from bullet_sim.collision.shapes import ShapedCollision

    table = {
        "circle": CircleCollision,
        "circle_distance": DistanceCollision,
        "grid": UniformGridCollision,
        "shaped": ShapedCollision,
        "obstacle": ObstacleCollision,
        "null": NullCollision,
    }
    try:
        cls = table[kind]
    except KeyError as exc:
        raise ValueError(
            f"unknown collision model {kind!r}; available: {sorted(table)}"
        ) from exc
    return cls(**kwargs)  # type: ignore[arg-type]
