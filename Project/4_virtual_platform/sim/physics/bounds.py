"""Bullet lifecycle: boundary handling and lifetime expiry."""

from __future__ import annotations

import numpy as np

from bullet_sim.entities.bullet import BulletPool

#: ``cull``  - delete bullets leaving the field (default, cheapest)
#: ``wrap``  - torus topology (useful for infinite density stress tests)
#: ``bounce``- reflect the velocity component that left the field
BOUNDARY_MODES = ("cull", "wrap", "bounce")


def out_of_bounds_mask(
    pool: BulletPool, field_w: float, field_h: float, margin: float = 0.0
) -> np.ndarray:
    """Live bullets whose centre lies outside the field (+``margin``)."""
    if pool.count == 0:
        return np.zeros(pool.capacity, dtype=bool)
    sel = pool.alive
    x = pool.data["x"]
    y = pool.data["y"]
    bad = np.zeros(pool.capacity, dtype=bool)
    bad[sel] = (
        (x[sel] < -margin)
        | (x[sel] > field_w + margin)
        | (y[sel] < -margin)
        | (y[sel] > field_h + margin)
    )
    return bad


def cull_out_of_bounds(
    pool: BulletPool,
    field_w: float,
    field_h: float,
    margin: float = 0.0,
    mode: str = "cull",
) -> int:
    """Apply the boundary policy; returns the number of bullets removed."""
    if pool.count == 0:
        return 0
    if mode == "cull":
        return pool.kill_mask(out_of_bounds_mask(pool, field_w, field_h, margin))
    sel = pool.alive
    if not sel.any():
        return 0
    d = pool.data
    x, y, vx, vy = d["x"], d["y"], d["vx"], d["vy"]
    if mode == "wrap":
        span_x = field_w + 2.0 * margin
        span_y = field_h + 2.0 * margin
        x[sel] = (x[sel] + margin) % span_x - margin
        y[sel] = (y[sel] + margin) % span_y - margin
        return 0
    if mode == "bounce":
        lo_x, hi_x = -margin, field_w + margin
        lo_y, hi_y = -margin, field_h + margin
        left = sel & (x < lo_x)
        right = sel & (x > hi_x)
        bottom = sel & (y < lo_y)
        top = sel & (y > hi_y)
        x[left] = 2.0 * lo_x - x[left]
        x[right] = 2.0 * hi_x - x[right]
        y[bottom] = 2.0 * lo_y - y[bottom]
        y[top] = 2.0 * hi_y - y[top]
        vx[left | right] *= -1.0
        vy[bottom | top] *= -1.0
        return 0
    raise ValueError(f"unknown boundary mode {mode!r}; expected one of {BOUNDARY_MODES}")


def expire_bullets(pool: BulletPool) -> int:
    """Remove bullets whose age reached their time-to-live."""
    if pool.count == 0:
        return 0
    sel = pool.alive
    if not sel.any():
        return 0
    d = pool.data
    dead = np.zeros(pool.capacity, dtype=bool)
    dead[sel] = d["age"][sel] >= d["ttl"][sel]
    return pool.kill_mask(dead)
