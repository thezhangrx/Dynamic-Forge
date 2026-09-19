"""Player kinematics.

The player is a massless point constrained to a maximum speed - intentionally
the simplest model that still maps 1:1 onto a real 2D differential/omni drive
cart (``(vx, vy)`` command, bounded norm, bounded workspace).  Nothing here
depends on a discrete action set: the action -> velocity translation happens in
``core.actions``.
"""

from __future__ import annotations

import numpy as np

from bullet_sim.core.actions import ActionContext, ActionCodec
from bullet_sim.core.numerics import clip_norm
from bullet_sim.entities.player import P_RADIUS, P_SPEED, P_VX, P_VY, P_X, P_Y


def clamp_to_field(
    player: np.ndarray,
    field_w: float,
    field_h: float,
    clamp: bool = True,
) -> None:
    """Keep the player inside the workspace, accounting for its radius."""
    if not clamp:
        return
    r = float(player[P_RADIUS])
    player[P_X] = min(max(float(player[P_X]), r), field_w - r)
    player[P_Y] = min(max(float(player[P_Y]), r), field_h - r)


def apply_velocity_command(
    player: np.ndarray,
    vx: float,
    vy: float,
    dt: float,
    *,
    max_speed: float = float("inf"),
    field_w: float | None = None,
    field_h: float | None = None,
    clamp: bool = True,
) -> None:
    """Integrate an externally supplied velocity command (in place)."""
    v = np.array([vx, vy], dtype=np.float64)
    limit = min(max_speed, float(player[P_SPEED]))
    v = clip_norm(v, limit) if np.isfinite(limit) else v
    player[P_VX] = v[0]
    player[P_VY] = v[1]
    player[P_X] = float(player[P_X]) + v[0] * dt
    player[P_Y] = float(player[P_Y]) + v[1] * dt
    if field_w is not None and field_h is not None:
        clamp_to_field(player, field_w, field_h, clamp=clamp)


def apply_action(
    player: np.ndarray,
    action: object,
    codec: ActionCodec,
    dt: float,
    *,
    field_w: float | None = None,
    field_h: float | None = None,
    clamp: bool = True,
) -> np.ndarray:
    """Encode ``action`` with ``codec`` and integrate the resulting velocity.

    Returns the commanded velocity vector (useful for logging / datasets).
    """
    ctx = ActionContext(
        player_speed=float(player[P_SPEED]),
        vx=float(player[P_VX]),
        vy=float(player[P_VY]),
        dt=float(dt),
    )
    v = codec.encode(action, ctx)
    apply_velocity_command(
        player,
        float(v[0]),
        float(v[1]),
        dt,
        field_w=field_w,
        field_h=field_h,
        clamp=clamp,
    )
    return v
