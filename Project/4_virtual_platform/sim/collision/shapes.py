"""Player hitbox shapes and the shaped collision backend.

The first version of the platform used circles only.  The specification asks for
a design that can grow to **circle / rectangle / point / custom hitbox**, so the
overlap test is expressed as a *signed distance function* (SDF) from the bullet
centre to the player hitbox::

    hit  <=>  sdf(hitbox, bullet_centre) < bullet_radius
    clearance = sdf - bullet_radius          (negative  => overlapping)

Circle, point and axis-aligned rectangle all have closed-form SDFs, so the whole
test stays vectorised over the bullet pool and maps cleanly onto fixed-function
hardware.  A custom shape only has to provide ``sdf(x, y)``.

This module is deliberately independent of rendering and of the world: it takes
positions, not snapshots.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

import numpy as np

from bullet_sim.collision.base import (
    DEFAULT_RISK_RADIUS,
    CollisionModel,
    CollisionResult,
    CollisionStats,
    risk_from_clearance,
)
from bullet_sim.entities.bullet import BulletPool

# --------------------------------------------------------------------------
# hitboxes
# --------------------------------------------------------------------------


@runtime_checkable
class Hitbox(Protocol):
    """A player shape supporting a signed distance query."""

    shape: str

    def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
        """Signed distance from each bullet centre to the hitbox surface."""
        ...

    def describe(self) -> dict[str, Any]:
        ...


@dataclass(frozen=True)
class CircleHitbox:
    """Disc of radius ``radius`` centred at the player position."""

    radius: float = 3.0
    shape: str = "circle"

    def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
        dx = bx - px
        dy = by - py
        return np.sqrt(dx * dx + dy * dy) - float(self.radius)

    def describe(self) -> dict[str, Any]:
        return {"shape": self.shape, "radius": float(self.radius)}


@dataclass(frozen=True)
class PointHitbox:
    """The classic danmaku 1-pixel hitbox."""

    shape: str = "point"

    def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
        dx = bx - px
        dy = by - py
        return np.sqrt(dx * dx + dy * dy)

    def describe(self) -> dict[str, Any]:
        return {"shape": self.shape, "radius": 0.0}


@dataclass(frozen=True)
class RectHitbox:
    """Axis-aligned rectangle of half extents ``(half_w, half_h)``."""

    half_w: float = 4.0
    half_h: float = 4.0
    shape: str = "rect"

    def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
        qx = np.abs(bx - px) - float(self.half_w)
        qy = np.abs(by - py) - float(self.half_h)
        outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
        inside = np.minimum(np.maximum(qx, qy), 0.0)
        return outside + inside

    def describe(self) -> dict[str, Any]:
        return {
            "shape": self.shape,
            "half_w": float(self.half_w),
            "half_h": float(self.half_h),
        }


_HITBOXES = {"circle": CircleHitbox, "point": PointHitbox, "rect": RectHitbox}


def make_hitbox(spec: Any = None, *, radius: float = 3.0) -> Hitbox:
    """Build a hitbox from ``None`` / a name / a mapping / a Hitbox instance."""
    if spec is None:
        return CircleHitbox(radius=radius)
    if isinstance(spec, str):
        key = spec.strip().lower()
        if key == "circle":
            return CircleHitbox(radius=radius)
        if key in ("point", "pixel"):
            return PointHitbox()
        if key in ("rect", "rectangle", "aabb", "box"):
            return RectHitbox(half_w=radius, half_h=radius)
        raise ValueError(f"unknown hitbox shape {spec!r}; expected {sorted(_HITBOXES)}")
    if isinstance(spec, Hitbox):
        return spec
    if isinstance(spec, dict):
        params = dict(spec)
        name = str(params.pop("shape", params.pop("kind", "circle"))).lower()
        if name in ("point", "pixel"):
            return PointHitbox()
        if name in ("rect", "rectangle", "aabb", "box"):
            hw = float(params.pop("half_w", params.pop("half_width", radius)))
            hh = float(params.pop("half_h", params.pop("half_height", radius)))
            return RectHitbox(half_w=hw, half_h=hh)
        return CircleHitbox(radius=float(params.get("radius", radius)))
    raise ValueError(f"cannot build a hitbox from {spec!r}")


# --------------------------------------------------------------------------
# shaped collision backend
# --------------------------------------------------------------------------


class ShapedCollision(CollisionModel):
    """Circle/point/rect player hitboxes against circular bullets.

    One vectorised pass computes the SDF for every live bullet, from which all
    reported quantities (hit, count, closest bullet id, clearance, risk) follow.
    """

    name = "shaped"

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

    # ------------------------------------------------------------------
    def prepare(self, pool: BulletPool) -> None:
        return None

    def query(
        self,
        pool: BulletPool,
        px: float,
        py: float,
        pr: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        """Circle convenience wrapper (radius ``pr`` overrides the hitbox)."""
        return self.query_hitbox(pool, CircleHitbox(radius=pr), px, py, frame=frame)

    def query_hitbox(
        self,
        pool: BulletPool,
        hitbox: Hitbox,
        px: float,
        py: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        step = -1 if frame is None else int(frame)
        self._stats.queries += 1
        if pool.count == 0:
            return CollisionResult.miss(frame=step)
        idx = pool.active_indices()
        m = int(idx.size)
        if m == 0:
            return CollisionResult.miss(frame=step)
        self._stats.candidate_tests += m

        d = pool.data
        bx = d["x"][idx]
        by = d["y"][idx]
        br = d["radius"][idx]
        sdf = hitbox.sdf(px, py, bx, by)
        clearance = sdf - br
        min_i = int(np.argmin(clearance))
        min_clearance = float(clearance[min_i])

        hit_mask = clearance < 0.0 if self.strict else clearance <= 0.0
        if not hit_mask.any():
            return CollisionResult.miss(
                min_distance=min_clearance,
                risk=risk_from_clearance(min_clearance, self.risk_radius),
                frame=step,
            )

        hit_slots = idx[hit_mask]
        dx = bx - px
        dy = by - py
        d2 = dx * dx + dy * dy
        # Report the overlapping bullet whose *centre* is closest - that is the
        # one a consumer wants to highlight or attribute the hit to.
        centre_d2 = d2[hit_mask]
        k2 = int(np.argmin(centre_d2))
        slot = int(hit_slots[k2])
        return CollisionResult(
            hit=True,
            index=slot,
            bullet_id=int(d["id"][slot]),
            hit_ids=tuple(int(v) for v in d["id"][hit_slots]),
            count=int(hit_slots.size),
            min_dist2=float(centre_d2[k2]),
            min_distance=min_clearance,
            frame=step,
            risk=1.0,
            position=(float(d["x"][slot]), float(d["y"][slot])),
        )

    def clearance_at(self, pool: BulletPool, hitbox: Hitbox, px: float, py: float) -> float:
        """Signed surface clearance to the nearest bullet (negative = overlap)."""
        if pool.count == 0:
            return float("inf")
        idx = pool.active_indices()
        if idx.size == 0:
            return float("inf")
        d = pool.data
        sdf = hitbox.sdf(px, py, d["x"][idx], d["y"][idx])
        return float(np.min(sdf - d["radius"][idx]))

    def stats(self) -> CollisionStats:
        return self._stats

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "hitbox": self.hitbox.describe(),
            "strict": self.strict,
            "risk_radius": self.risk_radius,
            "broadphase": "none",
            "supported_shapes": sorted(_HITBOXES),
        }


def shaped_collision_for(hitbox: Any = None, **kwargs: Any) -> ShapedCollision:
    return ShapedCollision(hitbox, **kwargs)


__all__ = [
    "Hitbox",
    "CircleHitbox",
    "PointHitbox",
    "RectHitbox",
    "make_hitbox",
    "ShapedCollision",
    "shaped_collision_for",
]
