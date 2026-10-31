"""Collision model protocol.

The world knows nothing about *how* collisions are computed.  It asks a
:class:`CollisionModel` whether the player's hitbox overlaps any live bullet.
Swap in an analytic circle test, a shaped-hitbox test, a spatial-hash grid, a
swept-volume test, or a future FPGA collision block without touching the kernel.

The default model is the classic circle-circle overlap::

    (x_i - x_p)^2 + (y_i - y_p)^2 < (r_i + r_p)^2

Every model reports the specification-required outputs:

======================  ==========================================================
``hit``                 whether a collision happened
``index``/``bullet_id``  which bullet (slot index **and** stable monotonic id)
``frame``               simulation step index at which it was detected
``count``               how many bullets overlap simultaneously
``min_dist2``           closest squared centre distance
``min_distance``        closest surface distance (negative inside the hitbox)
``risk``                normalised proximity risk in ``[0, 1]``
``position``            world position of the reported bullet
======================  ==========================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from bullet_sim.entities.bullet import BulletPool

#: Clearance at which the proximity risk reaches 0 (fully safe).  The risk of a
#: non-overlapping bullet is ``1 - clearance / risk_radius`` clamped to [0, 1].
DEFAULT_RISK_RADIUS = 32.0


def risk_from_clearance(
    clearance: float, risk_radius: float = DEFAULT_RISK_RADIUS
) -> float:
    """Map a signed surface clearance (px) onto a ``[0, 1]`` risk value."""
    if risk_radius <= 0.0:
        return 1.0 if clearance <= 0.0 else 0.0
    if clearance <= 0.0:
        return 1.0
    return float(max(0.0, min(1.0, 1.0 - clearance / risk_radius)))


@dataclass(frozen=True)
class CollisionResult:
    """Outcome of a single player query."""

    hit: bool = False
    #: Lowest live slot index that overlaps (canonical, deterministic).
    index: int = -1
    #: Stable monotonic id of that bullet (survives ``compact()``).
    bullet_id: int = -1
    #: Stable ids of **every** overlapping bullet, ascending.  The contact
    #: state machine (``collision/events.py``) needs the whole set, not just one
    #: representative, to tell "touched a second bullet" from "still touching
    #: the first one".
    hit_ids: tuple[int, ...] = ()
    #: Total number of overlapping bullets (== ``len(hit_ids)``).
    count: int = 0
    #: Smallest squared centre distance found among overlapping bullets.
    min_dist2: float = float("inf")
    #: Smallest signed surface distance to *any* live bullet (negative = overlap).
    min_distance: float = float("inf")
    #: Simulation step index at which the query ran.
    frame: int = -1
    #: Normalised proximity risk in ``[0, 1]`` (1.0 = overlapping).
    risk: float = 0.0
    #: Position of the bullet at :attr:`index` (or ``nan`` when no hit).
    position: tuple[float, float] = (float("nan"), float("nan"))

    @classmethod
    def miss(
        cls,
        *,
        min_distance: float = float("inf"),
        risk: float = 0.0,
        frame: int = -1,
    ) -> "CollisionResult":
        return cls(
            hit=False, index=-1, bullet_id=-1, hit_ids=(), count=0,
            min_dist2=float("inf"),
            min_distance=min_distance, frame=frame, risk=risk,
            position=(float("nan"), float("nan")),
        )

    @property
    def clearance(self) -> float:
        """Signed surface clearance; negative means overlapping."""
        return self.min_distance

    def to_dict(self) -> dict:
        return {
            "hit": self.hit,
            "index": self.index,
            "bullet_id": self.bullet_id,
            "hit_ids": list(self.hit_ids),
            "count": self.count,
            "min_dist2": self.min_dist2,
            "min_distance": self.min_distance,
            "frame": self.frame,
            "risk": self.risk,
            "x": self.position[0],
            "y": self.position[1],
        }


@dataclass
class CollisionStats:
    """Optional counters exposed for benchmarks."""

    queries: int = 0
    candidate_tests: int = 0
    extra: dict = field(default_factory=dict)

    def reset(self) -> None:
        self.queries = 0
        self.candidate_tests = 0
        self.extra.clear()


@runtime_checkable
class CollisionModel(Protocol):
    """Minimal contract every collision backend must satisfy."""

    name: str

    def query(
        self,
        pool: BulletPool,
        px: float,
        py: float,
        pr: float,
        *,
        frame: int | None = None,
    ) -> CollisionResult:
        """Overlap result for a circular player proxy at ``(px, py)`` radius ``pr``."""
        ...

    def prepare(self, pool: BulletPool) -> None:
        """Optional per-step hook (broadphase rebuild). Default: no-op."""
        ...

    def stats(self) -> CollisionStats:
        ...

    def describe(self) -> dict[str, Any]:
        ...


class NullCollision:
    """Disables collision detection (pure dynamics experiments)."""

    name = "null"

    def query(self, pool, px, py, pr, *, frame=None) -> CollisionResult:
        return CollisionResult.miss(frame=-1 if frame is None else int(frame))

    def prepare(self, pool: BulletPool) -> None:
        return None

    def stats(self) -> CollisionStats:
        return CollisionStats()

    def describe(self) -> dict[str, Any]:
        return {"name": self.name}


__all__ = [
    "CollisionModel",
    "CollisionResult",
    "CollisionStats",
    "NullCollision",
    "DEFAULT_RISK_RADIUS",
    "risk_from_clearance",
]
