"""Bullet storage: a structure-of-arrays pool with O(1) slot recycling.

Layout rationale
----------------
Bullets are *not* Python objects.  Every attribute lives in one contiguous
``numpy`` array of length ``capacity``; only the first ``count`` *live* slots
carry meaningful data once :meth:`BulletPool.compact` has been called.  This is
what makes the state directly mappable to a fixed-stride FPGA/DMA buffer and
what keeps stepping allocation-free.

Slot semantics
--------------
* slots are allocated in strictly ascending order from a LIFO free stack, so
  allocation order is a pure function of the spawn schedule;
* ``alive`` is the authoritative liveness mask - a slot may be dead while
  higher slots are still live (hole) until ``compact()`` runs;
* the *canonical* ordering used for hashing, export and hardware transfer is
  ascending slot index;
* ``id`` is a monotonic counter, **independent of the slot index**, so it stays
  stable across ``compact()`` and can be reported by the collision system and
  referenced by datasets.

DynamicObstacle
--------------
A row of this pool is a *dynamic obstacle*, not necessarily a bullet: ``shape``
selects circle or rotated rectangle, ``half_w``/``half_h`` carry the rectangle
extents in world units, and ``rotation`` its heading.  A circular obstacle keeps
``half_w == half_h == radius`` so one clearance routine covers every shape.

Motion representation
---------------------
``vx, vy`` and ``angle`` are two views of the same information: ``angle`` is
re-derived from the velocity every step, while a non-zero ``angular_velocity``
rotates the velocity vector (curved bullets).  ``ax, ay`` is the linear
acceleration used when ``angular_velocity`` is zero.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

import numpy as np

from bullet_sim.core.errors import ConfigError, StateError
from bullet_sim.core.numerics import (
    INT_DTYPE,
    TYPE_DTYPE,
    float_dtype,
)

#: (name, kind) where kind is ``f`` (float), ``i`` (int32) or ``s`` (int16).
#: Every field the specification requires is present explicitly:
#: ``id, x, y, vx, vy, ax, ay, angle, angular_velocity, radius, ttl, active, type``.
BULLET_FIELDS: tuple[tuple[str, str], ...] = (
    ("id", "i"),
    ("x", "f"),
    ("y", "f"),
    ("vx", "f"),
    ("vy", "f"),
    ("ax", "f"),
    ("ay", "f"),
    ("angle", "f"),
    ("angular_velocity", "f"),
    ("radius", "f"),
    ("age", "f"),
    ("ttl", "f"),
    ("type_id", "s"),
    ("group_id", "s"),
    # --- DynamicObstacle geometry (protocol v3) ---------------------------
    ("shape", "s"),
    ("half_w", "f"),
    ("half_h", "f"),
    ("rotation", "f"),
)

FLOAT_FIELDS: tuple[str, ...] = tuple(n for n, k in BULLET_FIELDS if k == "f")
INT_FIELDS: tuple[str, ...] = tuple(n for n, k in BULLET_FIELDS if k != "f")

#: Backwards-compatible v1 protocol layout (stride 10 floats).
PROTOCOL_FIELDS_V1: tuple[str, ...] = (
    "x",
    "y",
    "vx",
    "vy",
    "ax",
    "ay",
    "radius",
    "ttl",
    "type_id",
    "group_id",
)

#: Canonical field order for the flat hardware protocol v2 (all float32 on the
#: wire).  ``angle``/``angular_velocity``/``id`` are the fields added by the
#: "virtual bullet-hell environment" specification; see docs/DESIGN.md §10.
PROTOCOL_FIELDS: tuple[str, ...] = (
    "x",
    "y",
    "vx",
    "vy",
    "ax",
    "ay",
    "angle",
    "angular_velocity",
    "radius",
    "ttl",
    "type_id",
    "group_id",
    "id",
)

#: v2 protocol layout (13 floats): adds angle / angular_velocity / id.
PROTOCOL_FIELDS_V2: tuple[str, ...] = PROTOCOL_FIELDS

#: Canonical field order for the flat hardware protocol v3 (17 floats).
#: v3 turns a "bullet" into a *dynamic obstacle*: it can be a rotated rectangle.
PROTOCOL_FIELDS: tuple[str, ...] = PROTOCOL_FIELDS_V2 + (
    "shape",
    "half_w",
    "half_h",
    "rotation",
)

#: Obstacle shape codes used on the wire and in the SoA ``shape`` column.
SHAPE_CODES: dict[str, int] = {"circle": 0, "disc": 0, "rect": 1, "box": 1}
SHAPE_NAMES: tuple[str, ...] = ("circle", "rect")


def shape_code(shape: str | int) -> int:
    """Accept ``"circle"``/``"rect"`` or a raw code."""
    if isinstance(shape, (int, np.integer)):
        code = int(shape)
        if not (0 <= code < len(SHAPE_NAMES)):
            raise ConfigError(f"unknown obstacle shape code {code}")
        return code
    key = str(shape).strip().lower()
    if key not in SHAPE_CODES:
        raise ConfigError(
            f"unknown obstacle shape {shape!r}; expected one of {sorted(SHAPE_CODES)}"
        )
    return SHAPE_CODES[key]


def shape_name(code: int) -> str:
    code = int(code)
    return SHAPE_NAMES[code] if 0 <= code < len(SHAPE_NAMES) else "circle"


#: Alias used by the dataset layer (one column per entry).
BULLET_COLUMNS_V1 = PROTOCOL_FIELDS_V1
BULLET_COLUMNS_V2 = PROTOCOL_FIELDS_V2

_DTYPE_OF = {
    "f": None,  # resolved from the pool's float dtype
    "i": INT_DTYPE,
    "s": TYPE_DTYPE,
}


class BulletPool:
    """Preallocated SoA bullet container with a deterministic free list."""

    __slots__ = (
        "capacity",
        "count",
        "dtype",
        "alive",
        "next_id",
        "_free",
        "_free_top",
        "data",
    )

    def __init__(self, capacity: int, dtype: Any = None) -> None:
        if capacity < 0:
            raise ConfigError(f"bullet capacity must be >= 0, got {capacity}")
        self.capacity = int(capacity)
        self.count = 0
        self.dtype = float_dtype(dtype)
        self.data: Dict[str, np.ndarray] = {}
        for name, kind in BULLET_FIELDS:
            dt = self.dtype if kind == "f" else _DTYPE_OF[kind]
            self.data[name] = np.zeros(self.capacity, dtype=dt)
        self.alive = np.zeros(self.capacity, dtype=bool)
        #: Monotonic bullet id source.  Ids survive ``compact()`` (unlike slot
        #: indices), which is what collision reports and datasets refer to.
        self.next_id = 1
        # Free stack: the *last* element is the next slot to hand out, so the
        # initial list yields ascending slots 0, 1, 2, ...
        self._free = np.arange(self.capacity - 1, -1, -1, dtype=INT_DTYPE)
        self._free_top = self.capacity

    # ------------------------------------------------------------------
    # attribute access
    # ------------------------------------------------------------------
    def __getattr__(self, item: str) -> np.ndarray:
        # Only reached when normal attribute lookup fails; gives `pool.x`
        # sugar without duplicating the arrays.
        try:
            data = object.__getattribute__(self, "data")
        except AttributeError:  # partially constructed / unpickling
            raise AttributeError(item) from None
        if item in data:
            return data[item]
        raise AttributeError(item)

    @property
    def free_count(self) -> int:
        return int(self._free_top)

    def __len__(self) -> int:
        return int(self.count)

    # ------------------------------------------------------------------
    # allocation
    # ------------------------------------------------------------------
    def grow(self, new_capacity: int) -> None:
        """Enlarge the pool in place (keeps existing slots and free order)."""
        if new_capacity <= self.capacity:
            return
        extra = new_capacity - self.capacity
        for name, kind in BULLET_FIELDS:
            dt = self.dtype if kind == "f" else _DTYPE_OF[kind]
            buf = np.zeros(new_capacity, dtype=dt)
            buf[: self.capacity] = self.data[name]
            self.data[name] = buf
        new_alive = np.zeros(new_capacity, dtype=bool)
        new_alive[: self.capacity] = self.alive
        self.alive = new_alive
        # Stack order: the *existing* free slots stay on top (so allocation
        # order is unchanged by growing) and the new slots sit below them,
        # themselves arranged descending so they are handed out ascending.
        new_free = np.empty(new_capacity, dtype=INT_DTYPE)
        keep = self._free_top
        new_free[:extra] = np.arange(
            new_capacity - 1, self.capacity - 1, -1, dtype=INT_DTYPE
        )
        new_free[extra : extra + keep] = self._free[:keep]
        self._free = new_free
        self._free_top = keep + extra
        self.capacity = int(new_capacity)

    def alloc(self, n: int) -> np.ndarray:
        """Reserve ``n`` slots, returning their indices in ascending order."""
        n = int(n)
        if n <= 0:
            return np.empty(0, dtype=INT_DTYPE)
        if n > self._free_top:
            self.grow(max(self.capacity * 2, self.capacity + n))
        top = self._free_top
        idx = self._free[top - n : top][::-1].copy()
        self._free_top = top - n
        self.alive[idx] = True
        self.count += n
        return idx

    def release(self, idx: np.ndarray | Sequence[int]) -> None:
        """Return slots to the free stack (idempotent for dead slots)."""
        arr = np.asarray(idx, dtype=INT_DTYPE).reshape(-1)
        if arr.size == 0:
            return
        arr = arr[self.alive[arr]]
        if arr.size == 0:
            return
        arr = np.sort(arr)
        self.alive[arr] = False
        top = self._free_top
        self._free[top : top + arr.size] = arr
        self._free_top = top + arr.size
        self.count -= int(arr.size)
        if self.count < 0:  # pragma: no cover - invariant guard
            raise StateError("bullet pool count went negative")

    def kill_mask(self, mask: np.ndarray) -> int:
        """Kill every live slot where ``mask`` is true; returns how many died."""
        idx = np.flatnonzero(np.asarray(mask, dtype=bool) & self.alive)
        self.release(idx)
        return int(idx.size)

    def clear(self) -> None:
        """Kill everything and reset the free list to the pristine order."""
        self.alive[:] = False
        self.count = 0
        self.next_id = 1
        self._free[:] = np.arange(self.capacity - 1, -1, -1, dtype=INT_DTYPE)
        self._free_top = self.capacity

    # ------------------------------------------------------------------
    # spawning
    # ------------------------------------------------------------------
    def spawn(
        self,
        x: Any,
        y: Any,
        vx: Any,
        vy: Any,
        *,
        ax: Any = 0.0,
        ay: Any = 0.0,
        radius: Any = 2.0,
        ttl: Any = float("inf"),
        type_id: Any = 0,
        group_id: Any = 0,
        angular_velocity: Any = None,
        shape: Any = 0,
        half_w: Any = None,
        half_h: Any = None,
        rotation: Any = 0.0,
    ) -> np.ndarray:
        """Write ``n`` bullets into freshly allocated slots.

        All array-like inputs must broadcast to a common length ``n``.
        ``age`` is initialised to ``0``.
        """
        n = _broadcast_size(x, y, vx, vy, ax, ay, radius, ttl, type_id, group_id)
        if n == 0:
            return np.empty(0, dtype=INT_DTYPE)
        idx = self.alloc(n)
        d = self.data
        d["x"][idx] = np.broadcast_to(np.asarray(x, dtype=self.dtype), (n,))
        d["y"][idx] = np.broadcast_to(np.asarray(y, dtype=self.dtype), (n,))
        d["vx"][idx] = np.broadcast_to(np.asarray(vx, dtype=self.dtype), (n,))
        d["vy"][idx] = np.broadcast_to(np.asarray(vy, dtype=self.dtype), (n,))
        d["ax"][idx] = np.broadcast_to(np.asarray(ax, dtype=self.dtype), (n,))
        d["ay"][idx] = np.broadcast_to(np.asarray(ay, dtype=self.dtype), (n,))
        d["radius"][idx] = np.broadcast_to(np.asarray(radius, dtype=self.dtype), (n,))
        d["ttl"][idx] = np.broadcast_to(np.asarray(ttl, dtype=self.dtype), (n,))
        d["age"][idx] = 0.0
        d["type_id"][idx] = np.broadcast_to(np.asarray(type_id, dtype=TYPE_DTYPE), (n,))
        d["group_id"][idx] = np.broadcast_to(np.asarray(group_id, dtype=TYPE_DTYPE), (n,))
        # DynamicObstacle geometry: a circle stores half_w == half_h == radius
        # so the same clearance code works for every shape.
        # ``shape`` may be a single name/code or a per-bullet array of codes.
        if isinstance(shape, str):
            code: Any = shape_code(shape)
        elif isinstance(shape, np.ndarray):
            code = shape
        else:
            code = shape_code(int(shape))
        d["shape"][idx] = np.broadcast_to(np.asarray(code, dtype=TYPE_DTYPE), (n,))
        r = d["radius"][idx]
        d["half_w"][idx] = r if half_w is None else np.broadcast_to(
            np.asarray(half_w, dtype=self.dtype), (n,)
        )
        d["half_h"][idx] = r if half_h is None else np.broadcast_to(
            np.asarray(half_h, dtype=self.dtype), (n,)
        )
        d["rotation"][idx] = np.broadcast_to(np.asarray(rotation, dtype=self.dtype), (n,))
        d["id"][idx] = np.arange(self.next_id, self.next_id + n, dtype=INT_DTYPE)
        self.next_id += n
        # ``angle`` starts as the velocity direction and is kept in sync every
        # step; ``angular_velocity`` defaults to 0 (straight bullet).
        d["angle"][idx] = np.arctan2(d["vy"][idx], d["vx"][idx])
        if angular_velocity is not None:
            d["angular_velocity"][idx] = np.broadcast_to(
                np.asarray(angular_velocity, dtype=self.dtype), (n,)
            )
        return idx

    def set_fields(self, idx: np.ndarray, **values: Any) -> None:
        """Overwrite individual fields of existing slots (debug / injection)."""
        for name, val in values.items():
            if name not in self.data:
                raise ConfigError(f"unknown bullet field {name!r}")
            self.data[name][idx] = val

    # ------------------------------------------------------------------
    # views
    # ------------------------------------------------------------------
    def active_indices(self) -> np.ndarray:
        """Ascending indices of live slots (canonical ordering)."""
        return np.flatnonzero(self.alive)

    def active_mask(self) -> np.ndarray:
        return self.alive

    def compact(self) -> int:
        """Move all live bullets into the lowest slots, preserving slot order.

        Returns the new ``count``.  ``self.alive[:count]`` is all ``True``
        afterwards.  Used before export / hardware transfer.
        """
        idx = self.active_indices()
        n = int(idx.size)
        if n == self.count and n > 0 and np.array_equal(idx, np.arange(n)):
            return n
        for name in self.data:
            arr = self.data[name]
            arr[:n] = arr[idx]
        self.alive[:n] = True
        self.alive[n:] = False
        self.count = n
        # Slots [0, n) are live; the rest are free, arranged so that n is the
        # next slot handed out and allocation stays ascending.
        free = np.arange(self.capacity - 1, n - 1, -1, dtype=INT_DTYPE)
        self._free[: free.size] = free
        self._free_top = int(free.size)
        return n

    def field(self, name: str, compact: bool = True) -> np.ndarray:
        """Return a field (optionally only the live prefix) as a copy."""
        if name not in self.data:
            raise ConfigError(f"unknown bullet field {name!r}")
        if not compact:
            return self.data[name].copy()
        idx = self.active_indices()
        return self.data[name][idx]

    def to_arrays(self, compact: bool = True) -> Dict[str, np.ndarray]:
        """Export all fields as a dict of equal-length arrays."""
        idx = self.active_indices() if compact else np.arange(self.capacity)
        return {name: self.data[name][idx] for name in self.data}

    def to_protocol_matrix(self, compact: bool = True) -> np.ndarray:
        """Return a ``float32[N, 10]`` matrix in canonical :data:`PROTOCOL_FIELDS` order."""
        idx = self.active_indices() if compact else np.arange(self.capacity)
        out = np.empty((idx.size, len(PROTOCOL_FIELDS)), dtype=np.float32)
        for j, name in enumerate(PROTOCOL_FIELDS):
            out[:, j] = self.data[name][idx]
        return out

    # ------------------------------------------------------------------
    # copy / serialization
    # ------------------------------------------------------------------
    def copy(self) -> "BulletPool":
        other = BulletPool.__new__(BulletPool)
        other.capacity = self.capacity
        other.count = self.count
        other.dtype = self.dtype
        other.next_id = self.next_id
        other.data = {k: v.copy() for k, v in self.data.items()}
        other.alive = self.alive.copy()
        other._free = self._free.copy()
        other._free_top = self._free_top
        return other

    def to_dict(self, compact: bool = True) -> Dict[str, Any]:
        """JSON-friendly dict (lists).  Set ``compact=True`` for live bullets only."""
        idx = self.active_indices() if compact else np.arange(self.capacity)
        out: Dict[str, Any] = {
            "capacity": self.capacity,
            "count": int(idx.size),
            "next_id": int(self.next_id),
            "fields": {
                name: self.data[name][idx].tolist() for name in self.data
            },
        }
        return out

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any], dtype: Any = None) -> "BulletPool":
        capacity = int(payload.get("capacity", payload.get("count", 0)))
        pool = cls(capacity, dtype=dtype)
        fields = payload["fields"]
        n = int(payload.get("count", len(next(iter(fields.values()), []))))
        idx = np.arange(n, dtype=INT_DTYPE)
        for name in pool.data:
            if name in fields:
                pool.data[name][:n] = np.asarray(fields[name], dtype=pool.data[name].dtype)
        pool.alive[:n] = True
        pool.count = n
        # Restore the id counter: older payloads may not carry it, so fall back
        # to max(id)+1 (or 1 for an empty pool).
        ids = pool.data["id"][:n]
        pool.next_id = int(
            payload.get("next_id", (int(ids.max()) + 1) if n else 1)
        )
        free = np.arange(capacity - 1, n - 1, -1, dtype=INT_DTYPE)
        pool._free[: free.size] = free
        pool._free_top = int(free.size)
        return pool


def obstacle_extent(pool: "BulletPool", idx: np.ndarray | None = None) -> np.ndarray:
    """Circumscribed radius per obstacle: ``radius`` for circles, half-diagonal for rects.

    Used for broad-phase culling and for the free-space inflation, where a
    conservative bound is the correct choice.
    """
    if idx is None:
        idx = pool.active_indices()
    if idx.size == 0:
        return np.zeros(0, dtype=np.float64)
    d = pool.data
    is_rect = d["shape"][idx] != 0
    out = d["radius"][idx].astype(np.float64).copy()
    if is_rect.any():
        hw = d["half_w"][idx][is_rect].astype(np.float64)
        hh = d["half_h"][idx][is_rect].astype(np.float64)
        out[is_rect] = np.hypot(hw, hh)
    return out


def _broadcast_size(*vals: Any) -> int:
    """Common length of a set of broadcastable inputs.

    All-scalar inputs mean "exactly one bullet", which is the natural reading of
    ``spawn(x=1.0, y=2.0, ...)``.
    """
    n = -1
    for v in vals:
        arr = np.asarray(v)
        if arr.ndim == 0:
            continue
        sz = arr.size if arr.ndim == 1 else int(np.prod(arr.shape))
        if n == -1:
            n = sz
        elif sz != 1 and sz != n:
            raise ConfigError(
                f"bullet spawn inputs are not broadcastable: {sz} vs {n}"
            )
    return 1 if n == -1 else n
