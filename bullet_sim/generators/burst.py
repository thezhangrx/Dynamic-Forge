"""Spawn events and the deterministic spawn timeline.

A :class:`SpawnEvent` is a *pre-resolved batch* of bullets: positions are stored
as offsets from an origin, directions as angles, and "needs the live player
position" is expressed by two flags (``origin_mode='player'``, ``aim=True``).
That split is what makes stepping RNG-free while still allowing homing-aimed
patterns:

* everything random is resolved **once, at build time** with the scenario seed;
* everything player-dependent is resolved **at spawn time** from the current
  state, which is itself a deterministic function of (S_0, actions, dt).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Iterator, Sequence

import numpy as np

from bullet_sim.core.numerics import TYPE_DTYPE

ORIGIN_FIXED = 0
ORIGIN_PLAYER = 1


@dataclass
class SpawnEvent:
    """One atomic batch of bullets emitted at one fixed simulation step."""

    step: int
    origin: np.ndarray = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64)
    )
    origin_mode: int = ORIGIN_FIXED
    aim: bool = False
    group_id: int = 0
    pattern: str = ""

    offsets: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    angles: np.ndarray = field(default_factory=lambda: np.zeros(0))
    speeds: np.ndarray = field(default_factory=lambda: np.zeros(0))
    accel_mag: np.ndarray = field(default_factory=lambda: np.zeros(0))
    #: ``nan`` means "accelerate along the bullet's own velocity".
    accel_angle: np.ndarray = field(default_factory=lambda: np.zeros(0))
    radius: np.ndarray = field(default_factory=lambda: np.zeros(0))
    ttl: np.ndarray = field(default_factory=lambda: np.zeros(0))
    #: Per-bullet spin rate in rad/s; 0 = straight bullet (curved bullets).
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(0))
    type_id: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=TYPE_DTYPE))
    #: DynamicObstacle geometry: 0 = circle, 1 = rect; extents in world units.
    shape: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=TYPE_DTYPE))
    half_w: np.ndarray = field(default_factory=lambda: np.zeros(0))
    half_h: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rotation: np.ndarray = field(default_factory=lambda: np.zeros(0))
    #: Drop any obstacle that would appear closer than this to the live player.
    min_player_distance: float = 0.0

    def __post_init__(self) -> None:
        self.step = int(self.step)
        self.origin = np.asarray(self.origin, dtype=np.float64).reshape(2)
        self.offsets = np.asarray(self.offsets, dtype=np.float64).reshape(-1, 2)
        n = self.offsets.shape[0]
        for name in (
            "angles",
            "speeds",
            "accel_mag",
            "accel_angle",
            "radius",
            "ttl",
            "angular_velocity",
            "half_w",
            "half_h",
            "rotation",
        ):
            arr = np.asarray(getattr(self, name), dtype=np.float64).reshape(-1)
            if arr.size == 0:
                arr = np.zeros(n, dtype=np.float64)
            elif arr.size == 1 and n != 1:
                arr = np.full(n, float(arr[0]))
            elif arr.size != n:
                raise ValueError(
                    f"SpawnEvent field {name!r} has length {arr.size}, expected {n}"
                )
            setattr(self, name, arr)
        tid = np.asarray(self.type_id, dtype=TYPE_DTYPE).reshape(-1)
        if tid.size == 0:
            tid = np.zeros(n, dtype=TYPE_DTYPE)
        elif tid.size == 1 and n != 1:
            tid = np.full(n, int(tid[0]), dtype=TYPE_DTYPE)
        self.type_id = tid
        shp = np.asarray(self.shape, dtype=TYPE_DTYPE).reshape(-1)
        if shp.size == 0:
            shp = np.zeros(n, dtype=TYPE_DTYPE)
        elif shp.size == 1 and n != 1:
            shp = np.full(n, int(shp[0]), dtype=TYPE_DTYPE)
        self.shape = shp
        # a circle's box is its radius, so downstream code never special-cases
        if n and not self.half_w.any():
            self.half_w = self.radius.copy()
        if n and not self.half_h.any():
            self.half_h = self.radius.copy()

    @property
    def n(self) -> int:
        return int(self.offsets.shape[0])

    def is_empty(self) -> bool:
        return self.n == 0

    # ------------------------------------------------------------------
    def resolve_velocities(
        self, player_xy: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Resolve the batch into world-frame ``(positions, velocities, accelerations)``.

        ``player_xy`` is only consulted for aimed events; passing ``None`` for an
        aimed event is an error (the world always supplies it).
        """
        if self.n == 0:
            z = np.zeros((0, 2), dtype=np.float64)
            return z, z, z
        if self.origin_mode == ORIGIN_PLAYER:
            if player_xy is None:
                raise ValueError("origin_mode='player' requires the live player position")
            origin = np.asarray(player_xy, dtype=np.float64).reshape(2)
        else:
            origin = self.origin
        pos = origin[None, :] + self.offsets

        angles = self.angles
        if self.aim:
            if player_xy is None:
                raise ValueError("aim=True requires the live player position")
            target = np.asarray(player_xy, dtype=np.float64).reshape(2)
            base = np.arctan2(target[1] - pos[:, 1], target[0] - pos[:, 0])
            angles = base + angles

        vel = np.empty_like(pos)
        vel[:, 0] = np.cos(angles) * self.speeds
        vel[:, 1] = np.sin(angles) * self.speeds

        acc = np.zeros_like(pos)
        nz = self.accel_mag != 0.0
        if nz.any():
            along = np.isnan(self.accel_angle)
            both = nz & along
            if both.any():
                acc[both, 0] = np.cos(angles[both]) * self.accel_mag[both]
                acc[both, 1] = np.sin(angles[both]) * self.accel_mag[both]
            fixed = nz & ~along
            if fixed.any():
                acc[fixed, 0] = np.cos(self.accel_angle[fixed]) * self.accel_mag[fixed]
                acc[fixed, 1] = np.sin(self.accel_angle[fixed]) * self.accel_mag[fixed]
        return pos, vel, acc

    def to_dict(self) -> dict:
        return {
            "step": self.step,
            "origin": self.origin.tolist(),
            "origin_mode": int(self.origin_mode),
            "aim": bool(self.aim),
            "group_id": int(self.group_id),
            "pattern": self.pattern,
            "n": self.n,
            "offsets": self.offsets.tolist(),
            "angles": self.angles.tolist(),
            "speeds": self.speeds.tolist(),
            "accel_mag": self.accel_mag.tolist(),
            "accel_angle": self.accel_angle.tolist(),
            "radius": self.radius.tolist(),
            "ttl": self.ttl.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "shape": self.shape.tolist(),
            "half_w": self.half_w.tolist(),
            "half_h": self.half_h.tolist(),
            "rotation": self.rotation.tolist(),
            "min_player_distance": self.min_player_distance,
            "type_id": self.type_id.tolist(),
        }


class SpawnTimeline:
    """Ordered, immutable-by-convention schedule of :class:`SpawnEvent`."""

    __slots__ = ("_by_step", "_steps", "_count", "_window")

    def __init__(self, events: Iterable[SpawnEvent] | None = None) -> None:
        by_step: dict[int, list[SpawnEvent]] = {}
        for ev in events or ():
            by_step.setdefault(int(ev.step), []).append(ev)
        self._by_step = by_step
        self._steps = tuple(sorted(by_step))
        self._count = sum(ev.n for evs in by_step.values() for ev in evs)
        self._window = (
            (self._steps[0], self._steps[-1]) if self._steps else (0, 0)
        )

    # ------------------------------------------------------------------
    def __len__(self) -> int:
        return self._count

    def __bool__(self) -> bool:
        return self._count > 0

    @property
    def total_bullets(self) -> int:
        return self._count

    @property
    def step_window(self) -> tuple[int, int]:
        return self._window

    @property
    def steps(self) -> tuple[int, ...]:
        return self._steps

    def events_at(self, step: int) -> Sequence[SpawnEvent]:
        """Events due at ``step`` (declaration order preserved)."""
        return self._by_step.get(int(step), ())

    def has_event_at(self, step: int) -> bool:
        return int(step) in self._by_step

    def iter_events(self) -> Iterator[SpawnEvent]:
        for step in self._steps:
            yield from self._by_step[step]

    def events_in_range(self, start: int, stop: int) -> list[SpawnEvent]:
        out: list[SpawnEvent] = []
        for step in self._steps:
            if step < start:
                continue
            if step >= stop:
                break
            out.extend(self._by_step[step])
        return out

    def bullet_count_before(self, step: int) -> int:
        """How many bullets this timeline has emitted strictly before ``step``."""
        total = 0
        for s in self._steps:
            if s >= step:
                break
            total += sum(ev.n for ev in self._by_step[s])
        return total

    def summary(self) -> dict[str, Any]:
        per_kind: dict[str, int] = {}
        for step in self._steps:
            for ev in self._by_step[step]:
                per_kind[ev.pattern or "?"] = per_kind.get(ev.pattern or "?", 0) + ev.n
        return {
            "total_bullets": self._count,
            "event_steps": len(self._steps),
            "first_step": self._window[0],
            "last_step": self._window[1],
            "bullets_per_pattern": per_kind,
        }

    def to_list(self) -> list[dict]:
        return [ev.to_dict() for ev in self.iter_events()]


@dataclass
class GenerationContext:
    """Everything a generator may need that is known at build time."""

    field_w: float
    field_h: float
    dt: float
    duration: float
    rng: np.random.Generator
    #: Player spawn position (used for ``origin='player'`` bookkeeping only;
    #: runtime resolution always uses the *live* position).
    player_xy: np.ndarray = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64)
    )
    clock: Any = None

    def origin_xy(self, origin: Any) -> tuple[np.ndarray, int]:
        """Resolve an origin keyword to ``(position, mode)``."""
        if isinstance(origin, str):
            w, h = self.field_w, self.field_h
            table = {
                # "field" is the obstacle layouts' marker; they compute their own
                # boundary position and never read this value.
                "field": (0.5 * w, 0.5 * h),
                "center": (0.5 * w, 0.5 * h),
                "top": (0.5 * w, h),
                "bottom": (0.5 * w, 0.0),
                "left": (0.0, 0.5 * h),
                "right": (w, 0.5 * h),
            }
            if origin == "player":
                return np.asarray(self.player_xy, dtype=np.float64).reshape(2), ORIGIN_PLAYER
            x, y = table[origin]
            return np.array([x, y], dtype=np.float64), ORIGIN_FIXED
        arr = np.asarray(origin, dtype=np.float64).reshape(-1)
        if arr.size != 2:
            raise ValueError(f"origin must be 'field'|'player'|'top'|... or (x,y), got {origin!r}")
        return arr.copy(), ORIGIN_FIXED
