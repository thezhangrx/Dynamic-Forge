"""Immediate (one-frame) geometry primitives shared by both planners.

# [v0.5.0-DualPlanner]
# PURPOSE:
#   Give the ReactiveGap controller and the predictive FAR module one common,
#   game-agnostic definition of relative position / relative velocity /
#   closing rate / surface clearance / 8-sector summary.  Reactive uses these
#   on the CURRENT frame only; predictive expands the same primitives into
#   candidate x future-sample tensors.  One geometry model, two time models.
#
# OPEN-SOURCE REFERENCE:
#   F1TENTH Follow-the-Gap (nearest-obstacle bubble -> free gap),
#   F1TENTH iTTC (distance + closing speed), PythonRobotics potential field
#   (relative position / repulsion geometry), RVO2 velocity-obstacle geometry
#   (relative velocity).  Structure only; no code is copied.
#
# ALGORITHM:
#   r = p_obstacle - p_agent
#   v_rel = v_obstacle - v_agent
#   closing_rate = -dot(r, v_rel) / max(|r|, eps)     (>0 means approaching)
#   clearance = |r| - (r_agent + r_obstacle)
#   sector = argmax over 8 fixed directions of dot(unit(r), dir)   (no atan2)
#
# FPGA MAPPING:
#   candidate-free kernels over obstacles: subtract, dot, square, one sqrt,
#   one guarded divide, 8-way argmax, and per-sector min/max/add reduction.
#   Produces exactly the simple vectors a board could stream:
#   sector_clearance[8], sector_closing[8], sector_density[8], emergency_flag.
#
# CPU ROLE:
#   The CPU consumes the sector summary and does gap selection, goal alignment,
#   hysteresis and the final action; no geometry decision is delegated.
#
# COMPLEXITY:
#   O(N) per frame for geometry, O(8) for the sector reduction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = [
    "SECTOR_DEGREES",
    "SECTOR_DIRECTIONS",
    "SectorSummary",
    "relative_position",
    "relative_velocity",
    "radial_closing_rate",
    "surface_clearance",
    "sector_index",
    "sectorize",
    "gap_scores",
]

_EPS = 1e-9

#: Fixed 8-sector compass (degrees, CCW from +x).  Order is part of the contract.
SECTOR_DEGREES: tuple[int, ...] = (0, 45, 90, 135, 180, 225, 270, 315)
_ANGLES = np.deg2rad(np.asarray(SECTOR_DEGREES, dtype=np.float64))
#: (8, 2) unit directions of the sectors.
SECTOR_DIRECTIONS: np.ndarray = np.stack(
    [np.cos(_ANGLES), np.sin(_ANGLES)], axis=1
).astype(np.float64)


def _as_vec2(array: object, *, name: str) -> np.ndarray:
    arr = np.asarray(array, dtype=np.float64)
    arr = arr.reshape(-1, 2)
    if arr.ndim != 2:
        raise ValueError(f"{name} must be (..., 2)")
    return arr


def relative_position(positions: object, origin: object) -> np.ndarray:
    """``p_obstacle - p_agent`` for an ``(N, 2)`` position array."""
    return _as_vec2(positions, name="positions") - np.asarray(origin, dtype=np.float64)


def relative_velocity(velocities: object, origin_velocity: object) -> np.ndarray:
    """``v_obstacle - v_agent`` for an ``(N, 2)`` velocity array."""
    return _as_vec2(velocities, name="velocities") - np.asarray(
        origin_velocity, dtype=np.float64
    )


def radial_closing_rate(r: object, v_rel: object) -> np.ndarray:
    """Signed radial closing rate ``-dot(r, v_rel) / max(|r|, eps)``.

    ``> 0`` means the obstacle is approaching; ``<= 0`` means it is not.  This is
    a *current-frame* quantity: it never depends on a candidate action here.
    """
    r = _as_vec2(r, name="r")
    v = _as_vec2(v_rel, name="v_rel")
    d = np.sqrt(np.maximum((r * r).sum(axis=-1), _EPS * _EPS))
    return -(r * v).sum(axis=-1) / d


def surface_clearance(r: object, radius_sum: object) -> np.ndarray:
    """Signed surface-to-surface clearance ``|r| - (r_agent + r_obstacle)``."""
    r = _as_vec2(r, name="r")
    d = np.sqrt(np.maximum((r * r).sum(axis=-1), _EPS * _EPS))
    return d - np.asarray(radius_sum, dtype=np.float64)


def sector_index(r: object) -> np.ndarray:
    """Nearest of the 8 fixed sectors per obstacle, via dot product (no atan2)."""
    r = _as_vec2(r, name="r")
    if r.shape[0] == 0:
        return np.zeros(0, dtype=np.int64)
    return np.argmax(r @ SECTOR_DIRECTIONS.T, axis=1).astype(np.int64)


@dataclass(frozen=True)
class SectorSummary:
    """Per-sector current-frame summary; one entry per sector (S = 8)."""

    nearest_clearance: np.ndarray   # (S,) inf when the sector is empty
    closing_rate: np.ndarray        # (S,) max radial closing rate (>= 0)
    density: np.ndarray             # (S,) obstacle count
    sector_of_obstacle: np.ndarray  # (N,) sector chosen for each obstacle


def sectorize(r: object, v_rel: object, radius_sum: object) -> SectorSummary:
    """Reduce obstacles into the fixed 8 sectors (current frame only).

    # [v0.5.0-DualPlanner]
    # PURPOSE:
    #   Turn the live obstacle set into the small fixed-size vector a reactive
    #   controller (and a future FPGA) works with, so gap selection never needs
    #   the full obstacle list again.
    #
    # OPEN-SOURCE REFERENCE:
    #   F1TENTH Follow-the-Gap bubble/gap construction; structure only.
    #
    # ALGORITHM:
    #   idx = argmax_s dot(unit(r), dir_s)
    #   nearest_clearance[s] = min clearance over obstacles with idx == s
    #   closing_rate[s]      = max closing rate over obstacles with idx == s
    #   density[s]           = count of obstacles with idx == s
    #
    # FPGA MAPPING:
    #   8-way compare tree per obstacle + 8 accumulators (min/max/count); all
    #   add/compare, no atan2, no sort.
    #
    # CPU ROLE:
    #   Supplies features; gap/goal/hysteresis decisions stay on the CPU.
    #
    # COMPLEXITY:
    #   O(N + 8).
    # """
    r = _as_vec2(r, name="r")
    n = r.shape[0]
    s = SECTOR_DIRECTIONS.shape[0]
    nearest = np.full(s, np.inf, dtype=np.float64)
    closing = np.zeros(s, dtype=np.float64)
    density = np.zeros(s, dtype=np.float64)
    if n == 0:
        return SectorSummary(
            nearest_clearance=nearest,
            closing_rate=closing,
            density=density,
            sector_of_obstacle=np.zeros(0, dtype=np.int64),
        )

    idx = sector_index(r)
    clearance = surface_clearance(r, radius_sum)
    close = np.maximum(radial_closing_rate(r, v_rel), 0.0)
    np.minimum.at(nearest, idx, clearance)
    np.maximum.at(closing, idx, close)
    np.add.at(density, idx, 1.0)
    return SectorSummary(
        nearest_clearance=nearest,
        closing_rate=closing,
        density=density,
        sector_of_obstacle=idx,
    )


def gap_scores(clearance: object, neighbor_weight: float = 0.5) -> np.ndarray:
    """Circular gap score ``c[d] + w*(c[d-1] + c[d+1])`` over the 8 sectors.

    Encoding the neighbours stops a single narrow sector from being read as a
    wide gap (F1TENTH Follow-the-Gap idea, discrete 8-way version).
    """
    c = np.asarray(clearance, dtype=np.float64)
    w = float(neighbor_weight)
    return c + w * (np.roll(c, 1) + np.roll(c, -1))
