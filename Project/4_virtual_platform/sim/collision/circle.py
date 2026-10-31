"""Vectorised circle-circle collision (default backend).

Cost is a single pass over the live bullets: no Python loop, no allocation of
per-bullet objects, and a real-valued distance computation that is trivially
pipelineable onto CPU SIMD / FPGA DSP blocks.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from bullet_sim.collision.base import (
    DEFAULT_RISK_RADIUS,
    CollisionModel,
    CollisionResult,
    CollisionStats,
    risk_from_clearance,
)
from bullet_sim.entities.bullet import BulletPool


class CircleCollision(CollisionModel):
    """Exact brute-force circle test, vectorised over all live bullets."""

    name = "circle"

    def __init__(
        self, strict: bool = True, risk_radius: float = DEFAULT_RISK_RADIUS
    ) -> None:
        #: ``strict=True`` uses ``d2 < (r_i + r_p)^2`` (touching is *not* a hit).
        self.strict = bool(strict)
        self.risk_radius = float(risk_radius)
        self._stats = CollisionStats()
        self._d2 = np.empty(0, dtype=np.float64)
        self._hit = np.empty(0, dtype=bool)

    # ------------------------------------------------------------------
    def prepare(self, pool: BulletPool) -> None:
        """Pre-size scratch buffers so querying allocates nothing."""
        cap = pool.capacity
        if self._d2.size != cap:
            self._d2 = np.empty(cap, dtype=np.float64)
            self._hit = np.empty(cap, dtype=bool)

    def query(
        self,
        pool: BulletPool,
        px: float,
        py: float,
        pr: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        step = -1 if frame is None else int(frame)
        n = pool.count
        self._stats.queries += 1
        if n == 0:
            return CollisionResult.miss(frame=step)
        self.prepare(pool)
        d = pool.data
        idx = np.flatnonzero(pool.alive)
        m = idx.size
        self._stats.candidate_tests += int(m)
        if m == 0:
            return CollisionResult.miss(frame=step)

        dx = d["x"][idx] - px
        dy = d["y"][idx] - py
        d2 = dx * dx + dy * dy
        dist = np.sqrt(d2)
        clearance = dist - d["radius"][idx] - float(pr)
        min_clearance = float(np.min(clearance))
        risk = risk_from_clearance(min_clearance, self.risk_radius)
        hit = clearance < 0.0 if self.strict else clearance <= 0.0
        if not hit.any():
            return CollisionResult.miss(
                min_distance=min_clearance, risk=risk, frame=step
            )

        hit_slots = idx[hit]
        hit_d2 = d2[hit]
        k = int(np.argmin(hit_d2))
        slot = int(hit_slots[k])
        return CollisionResult(
            hit=True,
            index=slot,
            bullet_id=int(d["id"][slot]),
            hit_ids=tuple(int(v) for v in d["id"][hit_slots]),
            count=int(hit_slots.size),
            min_dist2=float(hit_d2[k]),
            min_distance=min_clearance,
            frame=step,
            risk=1.0,
            position=(float(d["x"][slot]), float(d["y"][slot])),
        )

    # ------------------------------------------------------------------
    def query_many(
        self, pool: BulletPool, px: np.ndarray, py: np.ndarray, pr: np.ndarray
    ) -> np.ndarray:
        """Batch query for ``P`` players -> ``bool[P]`` (used by vectorised envs)."""
        n = pool.count
        p = np.asarray(px, dtype=np.float64).reshape(-1)
        q = np.asarray(py, dtype=np.float64).reshape(-1)
        r = np.asarray(pr, dtype=np.float64).reshape(-1)
        if n == 0 or p.size == 0:
            return np.zeros(p.size, dtype=bool)
        idx = pool.active_indices()
        if idx.size == 0:
            return np.zeros(p.size, dtype=bool)
        d = pool.data
        bx = d["x"][idx][None, :]
        by = d["y"][idx][None, :]
        br = d["radius"][idx][None, :]
        dx = bx - p[:, None]
        dy = by - q[:, None]
        d2 = dx * dx + dy * dy
        rr = br + r[:, None]
        self._stats.queries += p.size
        self._stats.candidate_tests += int(p.size * idx.size)
        return (d2 < rr * rr).any(axis=1) if self.strict else (d2 <= rr * rr).any(axis=1)

    # ------------------------------------------------------------------
    def stats(self) -> CollisionStats:
        return self._stats

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "strict": self.strict,
            "risk_radius": self.risk_radius,
            "broadphase": "none",
        }


class DistanceCollision(CircleCollision):
    """Circle test that also returns a continuous penetration/separation metric.

    Used by risk-aware rewards and by the danger-field predictor.
    """

    name = "circle_distance"

    def clearance(self, pool: BulletPool, px: float, py: float, pr: float) -> float:
        """Signed clearance to the nearest bullet surface (negative = overlap)."""
        if pool.count == 0:
            return float("inf")
        idx = pool.active_indices()
        if idx.size == 0:
            return float("inf")
        d = pool.data
        dx = d["x"][idx] - px
        dy = d["y"][idx] - py
        dist = np.sqrt(dx * dx + dy * dy)
        return float(np.min(dist - (d["radius"][idx] + pr)))

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "strict": self.strict,
            "risk_radius": self.risk_radius,
            "broadphase": "none",
            "clearance": True,
        }
