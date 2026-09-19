"""Player-frame (relative-motion) representation.

The platform's job is *relative* motion, not "bullets flying":

    v_relative = v_obstacle - v_player

Whether the agent drives forward through a static world or stands still while
the world moves towards it is physically the same situation, and the AI must not
be able to tell the difference.  This module makes that equivalence explicit and
cheap: transform a snapshot into the player's frame, where the player is at rest
and every obstacle carries the relative velocity.

    world frame :  player moves at (vx, vy),  obstacle at (ovx, ovy)
    player frame:  player moves at (0, 0),    obstacle at (ovx - vx, ovy - vy)

Distances, clearances and collision times are invariant under this change of
frame - which is exactly what makes "did I move toward it?" and "did it move
toward me?" the same question.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np

from bullet_sim.core.state import WorldSnapshot
from bullet_sim.entities.bullet import BulletPool


def relative_velocity(
    obstacle_v: np.ndarray, player_v: np.ndarray
) -> np.ndarray:
    """``v_obstacle - v_player`` for ``(..., 2)`` arrays."""
    o = np.asarray(obstacle_v, dtype=np.float64)
    p = np.asarray(player_v, dtype=np.float64)
    return o - p


def closing_speed(
    positions: np.ndarray, velocities: np.ndarray, player_xy: np.ndarray
) -> np.ndarray:
    """Rate at which each obstacle closes on the player (positive = approaching).

    ``-d(dist)/dt`` for straight-line motion.  Zero means "moving parallel or
    moving away"; positive means the gap is shrinking.
    """
    pos = np.asarray(positions, dtype=np.float64).reshape(-1, 2)
    vel = np.asarray(velocities, dtype=np.float64).reshape(-1, 2)
    me = np.asarray(player_xy, dtype=np.float64).reshape(2)
    if pos.size == 0:
        return np.zeros(0, dtype=np.float64)
    rel = pos - me[None, :]
    dist = np.hypot(rel[:, 0], rel[:, 1])
    safe = np.maximum(dist, 1e-9)
    unit = rel / safe[:, None]
    return -np.einsum("ij,ij->i", vel, unit)


def time_to_impact(
    positions: np.ndarray,
    velocities: np.ndarray,
    player_xy: np.ndarray,
    *,
    radius: float = 0.0,
    max_seconds: float | None = None,
) -> np.ndarray:
    """Analytic first-contact time per obstacle for a *stationary* player.

    Assumes constant velocity (the caller should already be in the player frame
    if the player is moving).  ``inf`` when the obstacle never reaches the
    player.
    """
    pos = np.asarray(positions, dtype=np.float64).reshape(-1, 2)
    vel = np.asarray(velocities, dtype=np.float64).reshape(-1, 2)
    me = np.asarray(player_xy, dtype=np.float64).reshape(2)
    if pos.size == 0:
        return np.zeros(0, dtype=np.float64)
    rel = pos - me[None, :]
    a = np.einsum("ij,ij->i", vel, vel)
    b = 2.0 * np.einsum("ij,ij->i", rel, vel)
    c = np.einsum("ij,ij->i", rel, rel) - float(radius) ** 2
    out = np.full(pos.shape[0], np.inf, dtype=np.float64)
    moving = a > 1e-12
    disc = np.where(moving, b * b - 4.0 * a * c, -1.0)
    hit = moving & (disc >= 0.0)
    if hit.any():
        sq = np.sqrt(disc[hit])
        t1 = (-b[hit] - sq) / (2.0 * a[hit])
        t2 = (-b[hit] + sq) / (2.0 * a[hit])
        cand = np.where(t1 >= 0.0, t1, t2)
        cand = np.where(cand >= 0.0, cand, np.inf)
        out[hit] = cand
    else:
        # already overlapping and not moving
        out[moving & (c <= 0.0)] = 0.0
    if max_seconds is not None:
        out = np.where(out <= float(max_seconds), out, np.inf)
    return out


def obstacle_velocities(pool: BulletPool, idx: np.ndarray | None = None) -> np.ndarray:
    if idx is None:
        idx = pool.active_indices()
    if idx.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    d = pool.data
    return np.stack([d["vx"][idx], d["vy"][idx]], axis=-1).astype(np.float64)


def obstacle_positions(pool: BulletPool, idx: np.ndarray | None = None) -> np.ndarray:
    if idx is None:
        idx = pool.active_indices()
    if idx.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    d = pool.data
    return np.stack([d["x"][idx], d["y"][idx]], axis=-1).astype(np.float64)


def to_player_frame(
    snapshot: WorldSnapshot, *, player_velocity: tuple[float, float] | None = None
) -> WorldSnapshot:
    """Copy of ``S_t`` in which the player is at rest.

    Positions are unchanged; the player's velocity is zeroed and subtracted from
    every obstacle's velocity (and acceleration direction is left alone, since
    it is a world-frame quantity).  Use this to evaluate relative motion,
    clearances and time-to-impact without special-casing a moving player.
    """
    vel = (
        np.array(player_velocity, dtype=np.float64)
        if player_velocity is not None
        else np.array([snapshot.player.vx, snapshot.player.vy], dtype=np.float64)
    )
    out = snapshot.copy()
    player = replace(out.player, vx=0.0, vy=0.0)
    out.player = player
    pool = out.bullets
    idx = pool.active_indices()
    if idx.size:
        pool.data["vx"][idx] = pool.data["vx"][idx] - vel[0]
        pool.data["vy"][idx] = pool.data["vy"][idx] - vel[1]
    return out


def relative_motion_stats(snapshot: WorldSnapshot, *, horizon: float = 1.0) -> dict[str, Any]:
    """Summary of how the obstacle field is moving *relative to the player*.

    Numbers the HUD, the dataset and the difficulty score can all use.
    """
    pool = snapshot.bullets
    idx = pool.active_indices()
    me = np.array([snapshot.player.x, snapshot.player.y], dtype=np.float64)
    mine = np.array([snapshot.player.vx, snapshot.player.vy], dtype=np.float64)
    if idx.size == 0:
        return {
            "obstacles": 0,
            "max_closing_speed": 0.0,
            "mean_closing_speed": 0.0,
            "min_time_to_impact": None,
            "player_speed": float(np.hypot(*mine)),
        }
    pos = obstacle_positions(pool, idx)
    vel = obstacle_velocities(pool, idx)
    rel = relative_velocity(vel, mine)
    close = closing_speed(pos, rel, me)
    tti = time_to_impact(pos, rel, me, radius=float(snapshot.player.radius))
    approaching = close > 0.0
    return {
        "obstacles": int(idx.size),
        "player_speed": float(np.hypot(*mine)),
        "max_closing_speed": float(np.max(close)) if close.size else 0.0,
        "mean_closing_speed": float(np.mean(close)) if close.size else 0.0,
        "approaching_fraction": float(np.mean(approaching)) if close.size else 0.0,
        "min_time_to_impact": (
            float(np.min(tti)) if np.isfinite(tti).any() else None
        ),
        "horizon": float(horizon),
    }


__all__ = [
    "relative_velocity",
    "closing_speed",
    "time_to_impact",
    "obstacle_positions",
    "obstacle_velocities",
    "to_player_frame",
    "relative_motion_stats",
]
