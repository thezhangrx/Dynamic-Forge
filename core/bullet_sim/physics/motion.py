"""Bullet motion integration.

Two motion primitives, both exact over one fixed step:

**Straight / accelerated** (``angular_velocity == 0``)::

    p(t+dt) = p(t) + v(t)*dt + 0.5*a*dt^2
    v(t+dt) = v(t) + a*dt

**Curved** (``angular_velocity != 0``): the velocity vector is rotated by
``theta = omega*dt`` while its magnitude follows the acceleration.  The position
advance is the closed-form arc integral ``integral R(omega*t) v dt``::

    dp    = (sin(theta)/omega) * v_new + ((1 - cos(theta))/omega) * perp(v_new)
    v_rot = R(theta) @ v_new

which reduces to ``v*dt`` as ``omega -> 0``.  A constant-angular-velocity bullet
is therefore analytically testable instead of Euler-approximated.

``angular_velocity`` is interpreted by **shape**, which is what lets a rectangle
be a swinging barrier:

* **circle** - the velocity vector rotates (a curved shot), as described above;
* **rectangle** - the *body* spins about its own centre (``rotation += omega*dt``)
  while the velocity stays straight, so a wall can rotate in place.

Both paths are pure elementwise batch operations over the SoA arrays: no Python
loop over bullets, no allocation, no per-bullet branching.
"""

from __future__ import annotations

import numpy as np

from bullet_sim.entities.bullet import BulletPool

#: Below this |omega| the arc formula is replaced by its linear limit.
_OMEGA_EPS = 1e-12


def integrate_bullets(pool: BulletPool, dt: float, mask: np.ndarray | None = None) -> None:
    """Advance every live bullet by one fixed step (in place)."""
    if pool.count == 0:
        return
    sel = pool.alive if mask is None else (pool.alive & mask)
    if not sel.any():
        return

    d = pool.data
    x, y = d["x"], d["y"]
    vx, vy = d["vx"], d["vy"]
    ax, ay = d["ax"], d["ay"]
    omega = d["angular_velocity"]

    idx = np.flatnonzero(sel)

    # A *rectangle* uses ``angular_velocity`` to spin its **body** about its own
    # centre (a swinging barrier / a corridor wall that rotates); its velocity
    # stays straight.  A *circle* keeps the original meaning: the velocity
    # vector rotates, i.e. a curved shot.  Spinning bodies are therefore
    # excluded from the curved-velocity path below.
    is_rect = np.asarray(d["shape"]) != 0
    spinning = is_rect & (np.abs(omega) >= _OMEGA_EPS)
    if spinning.any():
        rot = d["rotation"]
        rot[spinning] = (
            np.mod(rot[spinning] + omega[spinning] * dt + np.pi, 2.0 * np.pi) - np.pi
        )

    w = omega[idx]
    w = np.where(is_rect[idx], 0.0, w)
    straight = np.abs(w) < _OMEGA_EPS

    if straight.all():
        x[sel] += vx[sel] * dt + ax[sel] * (0.5 * dt * dt)
        y[sel] += vy[sel] * dt + ay[sel] * (0.5 * dt * dt)
        vx[sel] += ax[sel] * dt
        vy[sel] += ay[sel] * dt
    else:
        nvx = vx[idx] + ax[idx] * dt
        nvy = vy[idx] + ay[idx] * dt
        dx = np.empty_like(nvx)
        dy = np.empty_like(nvy)

        s = straight
        if s.any():
            dx[s] = nvx[s] * dt
            dy[s] = nvy[s] * dt

        c = ~straight
        if c.any():
            th = w[c] * dt
            sin_t = np.sin(th)
            cos_t = np.cos(th)
            small = np.abs(th) < 1e-8
            # sin(theta)/omega -> dt  and  (1-cos(theta))/omega -> 0 as theta -> 0
            coef_a = np.where(small, dt, sin_t / np.where(small, 1.0, w[c]))
            coef_b = np.where(small, 0.0, (1.0 - cos_t) / np.where(small, 1.0, w[c]))
            vx_c = nvx[c]
            vy_c = nvy[c]
            dx[c] = coef_a * vx_c - coef_b * vy_c
            dy[c] = coef_a * vy_c + coef_b * vx_c
            nvx[c] = cos_t * vx_c - sin_t * vy_c
            nvy[c] = sin_t * vx_c + cos_t * vy_c

        x[idx] += dx
        y[idx] += dy
        vx[idx] = nvx
        vy[idx] = nvy

    d["age"][sel] += dt
    # Keep the ``angle`` view of the motion consistent with the velocity.
    # NOTE: ``arr[idx] = f(arr[idx])`` writes through; ``np.arctan2(..., out=arr[idx])``
    # would write into a *fancy-index copy* and silently do nothing.
    d["angle"][sel] = np.arctan2(d["vy"][sel], d["vx"][sel])


def advance_bullets_n(pool: BulletPool, dt: float, n: int) -> None:
    """Advance ``n`` steps (offline fast-forward).

    For purely straight/accelerated bullets the whole jump is closed-form.
    Curved bullets (``angular_velocity != 0``) fall back to repeated integration,
    because a rotating velocity has no single closed-form jump that also keeps
    ``angle`` consistent; that fallback is explicit rather than silently wrong.
    """
    if n <= 0 or pool.count == 0:
        return
    sel = pool.alive
    if not sel.any():
        return
    d = pool.data
    if np.any(d["angular_velocity"][sel] != 0.0):
        for _ in range(int(n)):
            integrate_bullets(pool, dt)
        return

    t = float(n) * dt
    d["x"][sel] += d["vx"][sel] * t + 0.5 * d["ax"][sel] * t * t
    d["y"][sel] += d["vy"][sel] * t + 0.5 * d["ay"][sel] * t * t
    d["vx"][sel] += d["ax"][sel] * t
    d["vy"][sel] += d["ay"][sel] * t
    d["age"][sel] += t
    d["angle"][sel] = np.arctan2(d["vy"][sel], d["vx"][sel])


def bullet_positions(pool: BulletPool) -> np.ndarray:
    """Return an ``[N, 2]`` copy of live bullet positions."""
    idx = pool.active_indices()
    return np.stack([pool.data["x"][idx], pool.data["y"][idx]], axis=-1)


def bullet_velocities(pool: BulletPool) -> np.ndarray:
    idx = pool.active_indices()
    return np.stack([pool.data["vx"][idx], pool.data["vy"][idx]], axis=-1)


def bullet_ids(pool: BulletPool) -> np.ndarray:
    """Live bullet ids in ascending slot order (the canonical ordering)."""
    return pool.data["id"][pool.active_indices()]
