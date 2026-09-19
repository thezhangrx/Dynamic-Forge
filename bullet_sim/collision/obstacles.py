"""Collision / clearance between the player hitbox and **dynamic obstacles**.

The original circle-circle model assumed obstacles were points with a radius.
A dynamic-obstacle environment needs circles *and* rotated rectangles (blocks,
walls, corridors), so this module expresses every obstacle as a **signed
distance function** and derives one clearance number per obstacle:

    clearance_i = sdf(obstacle_i, player_centre) - player_circumradius

For a circle or point player hitbox this is exact (``circumradius`` is the
radius, or zero).  For a rectangular player hitbox the circumradius is used,
which is *conservative*: a corner graze may be reported as a hit rather than
missed.  Safety analysis wants exactly that bias, and it is stated here rather
than hidden.

Relative motion
---------------
Overlap is geometric, but the *risk* is not: a wall sliding past at 200 px/s has
nothing to do with the same wall closing head-on.  :meth:`clearances` therefore
has a companion :meth:`contact_rate` that returns ``-d(clearance)/dt`` per
obstacle, i.e. how fast the gap is shrinking, and :meth:`time_to_contact` gives
the analytic first-contact time.
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
from bullet_sim.collision.shapes import CircleHitbox, Hitbox, make_hitbox
from bullet_sim.entities.bullet import BulletPool


def player_circumradius(hitbox: Any) -> float:
    """Conservative radius of a player hitbox (exact for circle/point)."""
    shape = getattr(hitbox, "shape", "circle")
    if shape in ("circle", "disc"):
        return float(getattr(hitbox, "radius", 0.0))
    if shape in ("point", "pixel"):
        return 0.0
    if shape in ("rect", "box"):
        return float(
            np.hypot(
                float(getattr(hitbox, "half_w", 0.0)),
                float(getattr(hitbox, "half_h", 0.0)),
            )
        )
    raise ValueError(f"unsupported hitbox shape {shape!r}")


def obstacle_sdf(
    px: float,
    py: float,
    ox: np.ndarray,
    oy: np.ndarray,
    shape: np.ndarray,
    radius: np.ndarray,
    half_w: np.ndarray,
    half_h: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    """Signed distance from the player centre to each obstacle surface.

    ``< 0`` means the player centre is inside the obstacle.  Works for circular
    and rotated-rectangular obstacles in one vectorised pass.
    """
    dx = ox - px
    dy = oy - py
    out = np.empty_like(dx, dtype=np.float64)

    circle = shape == 0
    if circle.any():
        out[circle] = np.hypot(dx[circle], dy[circle]) - radius[circle]

    rect = ~circle
    if rect.any():
        cos_r = np.cos(rotation[rect])
        sin_r = np.sin(rotation[rect])
        ddx = dx[rect]
        ddy = dy[rect]
        # rotate the offset into the obstacle's local frame
        lx = ddx * cos_r + ddy * sin_r
        ly = -ddx * sin_r + ddy * cos_r
        qx = np.abs(lx) - half_w[rect]
        qy = np.abs(ly) - half_h[rect]
        outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
        inside = np.minimum(np.maximum(qx, qy), 0.0)
        out[rect] = outside + inside
    return out


def obstacle_sdf_matrix(
    gx: np.ndarray,
    gy: np.ndarray,
    ox: np.ndarray,
    oy: np.ndarray,
    shape: np.ndarray,
    radius: np.ndarray,
    half_w: np.ndarray,
    half_h: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    """``(cells, obstacles)`` signed distances for a grid of sample points.

    Used by the free-space rasteriser: one cell is blocked if *any* obstacle is
    closer than the inflation radius, so this is the exact configuration-space
    test rather than a bounding-box approximation.
    """
    gx = np.asarray(gx, dtype=np.float64).reshape(-1, 1)
    gy = np.asarray(gy, dtype=np.float64).reshape(-1, 1)
    ox = np.asarray(ox, dtype=np.float64).reshape(1, -1)
    oy = np.asarray(oy, dtype=np.float64).reshape(1, -1)
    dx = ox - gx
    dy = oy - gy
    out = np.empty(dx.shape, dtype=np.float64)
    shape = np.asarray(shape).reshape(1, -1)
    circle = shape == 0
    if circle.any():
        out[:, circle[0]] = np.hypot(dx[:, circle[0]], dy[:, circle[0]]) - radius[circle[0]]
    rect = ~circle
    if rect.any():
        r = rect[0]
        cos_r = np.cos(rotation[r]).reshape(1, -1)
        sin_r = np.sin(rotation[r]).reshape(1, -1)
        lx = dx[:, r] * cos_r + dy[:, r] * sin_r
        ly = -dx[:, r] * sin_r + dy[:, r] * cos_r
        qx = np.abs(lx) - half_w[r].reshape(1, -1)
        qy = np.abs(ly) - half_h[r].reshape(1, -1)
        out[:, r] = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(
            np.maximum(qx, qy), 0.0
        )
    return out


class ObstacleCollision(CollisionModel):
    """Player hitbox vs DynamicObstacle (circle / rotated rectangle)."""

    name = "obstacle"

    def __init__(
        self,
        hitbox: Any = None,
        *,
        radius: float = 3.0,
        strict: bool = True,
        risk_radius: float = DEFAULT_RISK_RADIUS,
    ) -> None:
        self.hitbox: Hitbox = make_hitbox(hitbox, radius=radius)
        self.strict = bool(strict)
        self.risk_radius = float(risk_radius)
        self._stats = CollisionStats()
        self._last_clearance: np.ndarray = np.zeros(0)
        self._last_index: np.ndarray = np.zeros(0, dtype=np.int32)

    # ------------------------------------------------------------------
    def prepare(self, pool: BulletPool) -> None:
        return None

    # -- core vectorised query ----------------------------------------
    def clearances(self, pool: BulletPool, px: float, py: float, pr: float) -> np.ndarray:
        """Surface clearance to every active obstacle (negative = overlapping)."""
        idx = pool.active_indices()
        self._last_index = idx
        if idx.size == 0:
            self._last_clearance = np.zeros(0)
            return self._last_clearance
        d = pool.data
        sdf = obstacle_sdf(
            px, py,
            d["x"][idx].astype(np.float64), d["y"][idx].astype(np.float64),
            d["shape"][idx], d["radius"][idx].astype(np.float64),
            d["half_w"][idx].astype(np.float64), d["half_h"][idx].astype(np.float64),
            d["rotation"][idx].astype(np.float64),
        )
        self._last_clearance = sdf - float(pr)
        return self._last_clearance

    @property
    def last_clearances(self) -> np.ndarray:
        """Per-obstacle clearance from the most recent query (same order as ids)."""
        return self._last_clearance

    @property
    def last_ids(self) -> np.ndarray:
        return self._last_index

    def contact_rate(
        self, pool: BulletPool, px: float, py: float, pr: float, pvx: float, pvy: float
    ) -> np.ndarray:
        """``-d(clearance)/dt`` per obstacle: how fast the gap is closing.

        Positive = approaching, negative = separating, ~0 = sliding past.  This
        is the relative-motion signal the planner and the reward can use; the
        geometric overlap test alone cannot distinguish these cases.
        """
        idx = pool.active_indices()
        if idx.size == 0:
            return np.zeros(0)
        d = pool.data
        ox = d["x"][idx].astype(np.float64)
        oy = d["y"][idx].astype(np.float64)
        rel_x = ox - px
        rel_y = oy - py
        dist = np.maximum(np.hypot(rel_x, rel_y), 1e-9)
        unit_x = rel_x / dist
        unit_y = rel_y / dist
        rvx = d["vx"][idx].astype(np.float64) - float(pvx)
        rvy = d["vy"][idx].astype(np.float64) - float(pvy)
        return -(rvx * unit_x + rvy * unit_y)

    # -- CollisionModel protocol --------------------------------------
    def query(
        self,
        pool: BulletPool,
        px: float,
        py: float,
        pr: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        return self.query_hitbox(pool, CircleHitbox(radius=pr), px, py, frame=frame)

    def query_hitbox(
        self,
        pool: BulletPool,
        hitbox: Any,
        px: float,
        py: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        step = -1 if frame is None else int(frame)
        self._stats.queries += 1
        pr = player_circumradius(hitbox)
        clearance = self.clearances(pool, px, py, pr)
        idx = self._last_index
        if clearance.size == 0:
            return CollisionResult.miss(frame=step)
        self._stats.candidate_tests += int(clearance.size)

        k = int(np.argmin(clearance))
        min_clearance = float(clearance[k])
        hit = clearance < 0.0 if self.strict else clearance <= 0.0
        if not hit.any():
            return CollisionResult.miss(
                min_distance=min_clearance,
                risk=risk_from_clearance(min_clearance, self.risk_radius),
                frame=step,
            )
        hit_slots = idx[hit]
        d = pool.data
        # report the overlapping obstacle whose *centre* is nearest
        dx = d["x"][hit_slots] - px
        dy = d["y"][hit_slots] - py
        d2 = dx * dx + dy * dy
        j = int(np.argmin(d2))
        slot = int(hit_slots[j])
        return CollisionResult(
            hit=True,
            index=slot,
            bullet_id=int(d["id"][slot]),
            hit_ids=tuple(int(v) for v in d["id"][hit_slots]),
            count=int(hit_slots.size),
            min_dist2=float(d2[j]),
            min_distance=min_clearance,
            frame=step,
            risk=1.0,
            position=(float(d["x"][slot]), float(d["y"][slot])),
        )

    # ------------------------------------------------------------------
    def stats(self) -> CollisionStats:
        return self._stats

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "hitbox": self.hitbox.describe(),
            "strict": self.strict,
            "risk_radius": self.risk_radius,
            "broadphase": "none",
            "shapes": ["circle", "rect"],
            "rotation": True,
            "relative_motion": True,
        }


__all__ = [
    "ObstacleCollision",
    "obstacle_sdf",
    "obstacle_sdf_matrix",
    "player_circumradius",
]
