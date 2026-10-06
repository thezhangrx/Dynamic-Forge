"""Future Safe Corridor (FSC) -- v0.6.0-SafeCorridor-Shadow prototype.

# [v0.6.0-SafeCorridor-Shadow]
# PURPOSE:
#   Test a higher-level hypothesis than v0.5.1/v0.5.2 Long-Term Viability: future
#   safety is not a single scalar per candidate, but a *time-indexed spatial
#   structure*.  If the future safe spaces at t1/t2/t3 are connected by a
#   reachable relation and stay narrow and stable, the current action should
#   pre-position the agent toward that future corridor.
#
#   This module is a SHADOW prototype: it only *reads* a scene and produces
#   diagnostics.  It is never imported by :mod:`planner` and therefore cannot
#   change the emitted action.
#
# OPEN-SOURCE REFERENCE (ideas only; no planner is copied):
#   Nav2 MPPI                - layered collision / critical / preference critics;
#                              a high-level planner must not let a soft
#                              preference override a hard safety constraint.
#   F1TENTH Follow-the-Gap   - free-space gap and *continuous* safe region; do
#                              not only look at the nearest obstacle.
#   Open-SPITE               - cheap axis-aligned box approximations for fast
#                              safety pre-screening.
#   CommonRoad-Reach         - future reachable space as first-class high-level
#                              information (no full reachability solver here).
#   TEB                      - corridor / route persistence and selection
#                              hysteresis; do not switch route on a momentary
#                              score change.
#
# ALGORITHM:
#   per candidate, for each anchor t_k:
#       future agent position  p(t_k) = p0 + v_cand * t_k
#       future obstacle position o(t_k) = o0 + v_o * t_k
#       8-sector reduction -> SafeMask[8] (bit d = 1 <=> sector d is free)
#                           -> max_run, run_start, corridor clearance
#       conservative axis-aligned SafeRect around p(t_k)
#   then: reach_12 = intersect(Expand(R1, v_max*(t2-t1)), R2) != empty
#         reach_23 = intersect(Expand(R2, v_max*(t3-t2)), R3) != empty
#         corridor_type in {NONE, STABLE_SAFE_FUNNEL, DEAD_END_FUNNEL,
#                           UNSTABLE_FUNNEL}
#         future target (R3 centre, only when R3 is provably valid)
#         pre-positioning match p(t_k) in R_k
#
# FPGA MAPPING:
#   add / subtract / multiply / compare / min / max / bit-and only, for:
#   dx/dy, future position, squared distance, clearance, SafeMask, max run,
#   min clearance, rectangle bounds, expand, intersect, reach flag.  The CPU (or
#   a soft core) keeps funnel classification, target policy and the action.
#
# CPU ROLE:
#   Diagnostics only in this phase.  The v0.5.2 planner remains the sole owner
#   of the action; this module cannot emit one.
#
# COMPLEXITY:
#   O(K * S * N) with K = candidates evaluated (<= 4 target), S = 3 anchors,
#   N = obstacles.  No grid, no graph search, no sqrt in the reachability stage.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from cpu.decision.models import Action, SceneState
from cpu.decision.viability import (
    ViabilityContext,
    _circular_max_run,
    _sector_summary_batch,
)

#: Shadow master switch.  DEFAULT OFF: with it off the evaluator computes nothing
#: at all, so a caller can wire it in unconditionally and still get the exact
#: v0.5.2 action.  Override with ``DF_ENABLE_SAFE_CORRIDOR_SHADOW``.
SAFE_CORRIDOR_SHADOW_ENABLED: bool = os.environ.get(
    "DF_ENABLE_SAFE_CORRIDOR_SHADOW", "0"
).strip().lower() in {"1", "true", "yes", "on"}

__all__ = [
    "CORRIDOR_NONE",
    "CORRIDOR_UNKNOWN",
    "CORRIDOR_STABLE",
    "CORRIDOR_DEAD_END",
    "CORRIDOR_UNSTABLE",
    "CORRIDOR_TYPES",
    "CORRIDOR_STABLE_SAFE_FUNNEL",
    "CORRIDOR_DEAD_END_FUNNEL",
    "CORRIDOR_UNSTABLE_FUNNEL",
    "RECT_OK",
    "RECT_NO_OBSTACLES",
    "RECT_CENTRE_BLOCKED",
    "RECT_DIAGONAL_OVERLAP",
    "RECT_DEGENERATE",
    "SAFE_CORRIDOR_SHADOW_ENABLED",
    "AnchorSpace",
    "FutureSpaceDescriptor",
    "FutureCorridorConfig",
    "DSSv6ShadowHeader",
    "DSSv6ShadowGlobalSafety",
    "DSSv6ShadowCandidateRisk",
    "DSSv6Shadow",
    "ShadowFrame",
    "SafeCorridorShadow",
    "expand_rect",
    "rects_intersect",
    "rect_reachable",
    "safe_rect_from_obstacles",
    "mask_to_bool",
    "mask_max_run",
    "mask_direction",
    "mask_hamming",
    "mask_popcount",
    "persistent_mask",
    "circular_sector_distance",
    "classify_corridor",
    "descriptor_from_spaces",
    "bool_to_mask",
]

_EPS = 1e-9

#: Safe corridor classes (v0.6.0 enum).
CORRIDOR_NONE = "NONE"
#: There is future information, but the SafeRect / reachability computation could
#: not prove safety OR a dead end.  "Cannot prove safe" is NOT "proved dead end".
CORRIDOR_UNKNOWN = "UNKNOWN"
CORRIDOR_STABLE = "STABLE"
CORRIDOR_DEAD_END = "DEAD_END"
CORRIDOR_UNSTABLE = "UNSTABLE"
CORRIDOR_TYPES: tuple[str, ...] = (
    CORRIDOR_NONE,
    CORRIDOR_UNKNOWN,
    CORRIDOR_STABLE,
    CORRIDOR_DEAD_END,
    CORRIDOR_UNSTABLE,
)
#: Previous-round names, kept so older callers keep working.
CORRIDOR_STABLE_SAFE_FUNNEL = CORRIDOR_STABLE
CORRIDOR_DEAD_END_FUNNEL = CORRIDOR_DEAD_END
CORRIDOR_UNSTABLE_FUNNEL = CORRIDOR_UNSTABLE

#: Why a conservative SafeRect could not be produced (diagnostics only).
RECT_OK = "ok"
RECT_NO_OBSTACLES = "no_obstacles"
RECT_CENTRE_BLOCKED = "centre_blocked"
RECT_DIAGONAL_OVERLAP = "diagonal_overlap"
RECT_DEGENERATE = "degenerate"

#: Axis-aligned rectangle as ``(xmin, xmax, ymin, ymax)``.
Rect = tuple[float, float, float, float]


# ==========================================================================
# rectangle algebra (add / subtract / min / max / compare only)
# ==========================================================================
def expand_rect(rect: Rect, distance: float) -> Rect:
    """Grow ``rect`` by ``distance`` on every side (Minkowski with a square)."""
    xmin, xmax, ymin, ymax = rect
    d = float(distance)
    return (xmin - d, xmax + d, ymin - d, ymax + d)


def rects_intersect(a: Rect, b: Rect) -> bool:
    """Closed-interval AABB overlap test (touching counts as intersecting)."""
    return (
        a[0] <= b[1] and b[0] <= a[1] and a[2] <= b[3] and b[2] <= a[3]
    )


def rect_reachable(r1: Rect, r2: Rect, v_max: float, dt: float) -> bool:
    """``Expand(R1, v_max*dt)`` intersects ``R2`` (pure arithmetic, no sqrt)."""
    if dt <= 0.0:
        return rects_intersect(r1, r2)
    return rects_intersect(expand_rect(r1, float(v_max) * float(dt)), r2)


def safe_rect_from_obstacles(
    center: object,
    obstacle_positions: np.ndarray,
    obstacle_radii: np.ndarray,
    *,
    agent_radius: float,
    buffer: float = 0.0,
    max_half: float = 50.0,
    min_half: float = 0.5,
) -> tuple[bool, Rect, str]:
    """Conservative obstacle-free axis-aligned box around ``center``.

    Simple rule (no polygon clipping, no hull, no optimiser): for the future agent
    position ``(px, py)`` find the *left / right / down / up* clearance to the
    nearest inflated obstacle box that actually blocks that axis line, then::

        xmin = px - left_clearance      xmax = px + right_clearance
        ymin = py - down_clearance      ymax = py + up_clearance

    Obstacles are inflated by ``radius + agent_radius + buffer`` because the box
    bounds the agent **centre**.  Because the row/column rule cannot see a purely
    diagonal obstacle, the box is then **verified** against every inflated box; if
    that check fails the box is rejected (``valid=False``) rather than silently
    including obstacle area.  Returns ``(valid, rect, invalid_reason)``.
    """
    c = np.asarray(center, dtype=np.float64).reshape(2)
    cx, cy = float(c[0]), float(c[1])
    cap = float(max_half)

    obs = np.asarray(obstacle_positions, dtype=np.float64).reshape(-1, 2)
    radii = np.asarray(obstacle_radii, dtype=np.float64).reshape(-1)
    if obs.shape[0] and obs.shape[0] != radii.shape[0]:
        raise ValueError("obstacle_positions and obstacle_radii must align")

    if obs.shape[0] == 0:
        rect = (cx - cap, cx + cap, cy - cap, cy + cap)
        return True, rect, RECT_NO_OBSTACLES

    inflate = radii + float(agent_radius) + float(buffer)
    bxmin = obs[:, 0] - inflate
    bxmax = obs[:, 0] + inflate
    bymin = obs[:, 1] - inflate
    bymax = obs[:, 1] + inflate

    if bool(((bxmin <= cx) & (bxmax >= cx) & (bymin <= cy) & (bymax >= cy)).any()):
        return False, (cx - cap, cx + cap, cy - cap, cy + cap), RECT_CENTRE_BLOCKED

    # per-direction clearance on the centre's row / column
    y_band = (bymin <= cy) & (bymax >= cy)
    x_band = (bxmin <= cx) & (bxmax >= cx)
    left = cx - bxmax[y_band & (bxmax <= cx)]
    right = bxmin[y_band & (bxmin >= cx)] - cx
    down = cy - bymax[x_band & (bymax <= cy)]
    up = bymin[x_band & (bymin >= cy)] - cy
    left_clearance = float(left.min()) if left.size else cap
    right_clearance = float(right.min()) if right.size else cap
    down_clearance = float(down.min()) if down.size else cap
    up_clearance = float(up.min()) if up.size else cap
    left_clearance = min(left_clearance, cap)
    right_clearance = min(right_clearance, cap)
    down_clearance = min(down_clearance, cap)
    up_clearance = min(up_clearance, cap)

    rect = (
        cx - left_clearance,
        cx + right_clearance,
        cy - down_clearance,
        cy + up_clearance,
    )
    xmin, xmax, ymin, ymax = rect
    if xmax - xmin < 2.0 * min_half or ymax - ymin < 2.0 * min_half:
        return False, rect, RECT_DEGENERATE

    overlap = (bxmin < xmax) & (bxmax > xmin) & (bymin < ymax) & (bymax > ymin)
    if bool(overlap.any()):
        return False, rect, RECT_DIAGONAL_OVERLAP

    return True, (float(xmin), float(xmax), float(ymin), float(ymax)), RECT_OK


# ==========================================================================
# SafeMask algebra (bit operations + bounded circular run scan)
# ==========================================================================
def mask_to_bool(mask: int) -> np.ndarray:
    """8-bit ``SafeMask`` -> ``(8,)`` bool array (bit ``d`` = sector ``d``)."""
    m = int(mask) & 0xFF
    return np.asarray([(m >> d) & 1 for d in range(8)], dtype=bool)


def bool_to_mask(free: np.ndarray) -> int:
    """``(8,)`` bool array -> 8-bit ``SafeMask``."""
    out = 0
    for d, ok in enumerate(np.asarray(free, dtype=bool).reshape(-1)):
        if bool(ok):
            out |= 1 << d
    return out


def mask_max_run(mask: int) -> tuple[int, int]:
    """Longest circular run of set bits -> ``(length, start_sector)``.

    ``start`` is normalised to ``0`` when the mask is empty (no run exists).
    """
    free = mask_to_bool(mask)[None, :]
    run, start = _circular_max_run(free)
    length = int(run[0])
    return (length, int(start[0]) if length > 0 else 0)


def mask_direction(mask: int, max_run: int, run_start: int) -> int:
    """Corridor direction = centre sector of the longest run (-1 when empty)."""
    if max_run <= 0:
        return -1
    if max_run >= 8:
        return 0
    return int((int(run_start) + (int(max_run) - 1) // 2) % 8)


def mask_popcount(mask: int) -> int:
    """Number of set sectors in a SafeMask (bitwise popcount, no float math)."""
    return int(bin(int(mask) & 0xFF).count("1"))


def persistent_mask(*masks: int) -> int:
    """Bitwise AND of the anchor SafeMasks.

    A non-empty result means at least one sector is safe at *every* anchor, i.e.
    a direction that persists across the whole horizon.
    """
    out = 0xFF
    for mask in masks:
        out &= int(mask) & 0xFF
    return out


def mask_hamming(a: int, b: int) -> int:
    """Number of sectors whose safety bit differs between two SafeMasks."""
    return mask_popcount(int(a) ^ int(b))


def circular_sector_distance(a: int, b: int) -> int:
    """Circular distance in sectors between two 8-way directions."""
    if a < 0 or b < 0:
        return 8
    d = abs(int(a) - int(b)) % 8
    return min(d, 8 - d)


# ==========================================================================
# per-anchor future safe space
# ==========================================================================
@dataclass(frozen=True)
class AnchorSpace:
    """Future safe space of one candidate at one anchor time."""

    anchor: float
    safe_mask: int
    max_run: int
    run_start: int
    #: Minimum surface clearance over all obstacles at this anchor.
    min_clearance: float
    #: Minimum clearance *inside* the free corridor (0 when there is none).
    corridor_clearance: float
    safe_rect_valid: bool
    rect: Rect
    #: Why the SafeRect is invalid (see ``RECT_*``); ``RECT_OK``/``no_obstacles``
    #: when valid.  Diagnostics only.
    safe_rect_invalid_reason: str = RECT_OK
    valid: bool = True

    @property
    def direction(self) -> int:
        return mask_direction(self.safe_mask, self.max_run, self.run_start)

    def has_usable_corridor(self, config: "FutureCorridorConfig") -> bool:
        """A corridor of at least ``min_run`` sectors with enough clearance."""
        return (
            self.max_run >= config.min_run
            and self.corridor_clearance >= config.clearance_threshold
        )

    def to_dict(self) -> dict:
        return {
            "anchor": float(self.anchor),
            "safe_mask": int(self.safe_mask),
            "max_run": int(self.max_run),
            "run_start": int(self.run_start),
            "min_clearance": float(self.min_clearance),
            "corridor_clearance": float(self.corridor_clearance),
            "safe_rect_valid": bool(self.safe_rect_valid),
            "safe_rect_invalid_reason": self.safe_rect_invalid_reason,
            "xmin": float(self.rect[0]),
            "xmax": float(self.rect[1]),
            "ymin": float(self.rect[2]),
            "ymax": float(self.rect[3]),
        }


@dataclass(frozen=True)
class FutureCorridorConfig:
    """FSC thresholds.

    Every threshold is configurable here and echoed into the diagnostics, so no
    empirical constant is hidden deep in the code.  The first version uses the
    three fixed anchors.
    """

    anchors: tuple[float, float, float] = (0.30, 0.55, 0.80)
    #: A corridor needs at least this many contiguous safe sectors.
    min_run: int = 2
    #: Every anchor needs at least this much corridor clearance for STABLE.
    clearance_threshold: float = 2.0
    #: t3 clearance below this floor is a dead end (never STABLE).
    t3_clearance_floor: float = 1.0
    #: Sector count in ``mask_t1 & mask_t2 & mask_t3`` needed for STABLE.
    min_persistent_sectors: int = 1
    #: Direction change (sectors) between anchors above which the route is unstable.
    unstable_direction_jump: int = 3
    #: SafeMask Hamming jump between anchors above which the route is unstable.
    unstable_mask_jump: int = 4
    #: Minimum popcount(mask_i & mask_i+1) for the route to count as continuous.
    unstable_min_overlap: int = 1
    #: max_run drop from t1 to t3 that counts as a rapid collapse.
    rapid_shrink_run_drop: int = 2
    max_dynamic_buffer: float = 20.0
    #: SafeRect bounds.
    max_rect_half: float = 50.0
    min_rect_half: float = 0.5

    def __post_init__(self) -> None:
        if len(self.anchors) != 3:
            raise ValueError("FSC first version uses exactly three anchors")
        if not (self.anchors[0] < self.anchors[1] < self.anchors[2]):
            raise ValueError("anchors must be strictly increasing")
        if self.min_run < 1:
            raise ValueError("min_run must be >= 1")
        if self.dead_end_clearance < 0.0 or self.clearance_threshold < 0.0:
            raise ValueError("clearance thresholds must be non-negative")
        if self.min_rect_half < 0.0 or self.max_rect_half <= self.min_rect_half:
            raise ValueError("invalid SafeRect bounds")
        if self.min_persistent_sectors < 0 or self.unstable_min_overlap < 0:
            raise ValueError("sector counts must be non-negative")
        if self.rapid_shrink_run_drop < 0:
            raise ValueError("rapid_shrink_run_drop must be non-negative")

    @property
    def dead_end_clearance(self) -> float:
        """Backwards-compatible alias of :attr:`t3_clearance_floor`."""
        return self.t3_clearance_floor

    def to_dict(self) -> dict:
        """Threshold echo for the diagnostics record."""
        return {
            "anchors": [float(a) for a in self.anchors],
            "min_run": int(self.min_run),
            "clearance_threshold": float(self.clearance_threshold),
            "t3_clearance_floor": float(self.t3_clearance_floor),
            "min_persistent_sectors": int(self.min_persistent_sectors),
            "unstable_direction_jump": int(self.unstable_direction_jump),
            "unstable_mask_jump": int(self.unstable_mask_jump),
            "unstable_min_overlap": int(self.unstable_min_overlap),
            "rapid_shrink_run_drop": int(self.rapid_shrink_run_drop),
            "max_dynamic_buffer": float(self.max_dynamic_buffer),
            "max_rect_half": float(self.max_rect_half),
            "min_rect_half": float(self.min_rect_half),
        }


# ==========================================================================
# classification
# ==========================================================================
def classify_corridor(
    spaces: Sequence[AnchorSpace],
    reach_12: bool,
    reach_23: bool,
    config: FutureCorridorConfig,
    *,
    persistent: int | None = None,
    overlap_12: int | None = None,
    overlap_23: int | None = None,
) -> str:
    """Turn the three future safe spaces into a corridor class.

    Order matters: a hard dead end beats an unstable route, and an unstable route
    beats a stable one.  "The region got smaller" is never by itself a failure --
    only a lost exit, a broken reach relation, a collapsed *persistent* direction
    or a corridor jump is.
    """
    if len(spaces) != 3:
        raise ValueError("classify_corridor expects exactly three anchors")
    s1, s2, s3 = spaces
    if s1.max_run < config.min_run:
        return CORRIDOR_NONE
    persistent = (
        persistent_mask(s1.safe_mask, s2.safe_mask, s3.safe_mask)
        if persistent is None
        else int(persistent)
    )
    overlap_12 = (
        mask_popcount(s1.safe_mask & s2.safe_mask)
        if overlap_12 is None
        else int(overlap_12)
    )
    overlap_23 = (
        mask_popcount(s2.safe_mask & s3.safe_mask)
        if overlap_23 is None
        else int(overlap_23)
    )
    persistent_count = mask_popcount(persistent)

    # A. no evidence at all: there is no corridor to classify yet.
    if s1.max_run < config.min_run:
        return CORRIDOR_NONE
    # B. UNKNOWN: future information exists, but at least one conservative
    #    SafeRect could not be proven, so neither "safe" nor "dead end" is
    #    established.  Failing to *prove* safety is never a proven dead end.
    if not (s1.safe_rect_valid and s2.safe_rect_valid and s3.safe_rect_valid):
        return CORRIDOR_UNKNOWN
    # C. hard dead end: with all rectangles proven, a failed reach or a lost t3
    #    exit is a *proof*.
    if (
        not reach_12
        or not reach_23
        or s3.max_run < config.min_run
        or s3.corridor_clearance < config.t3_clearance_floor
    ):
        return CORRIDOR_DEAD_END
    # D. rapid collapse with no direction surviving all three anchors is a dead
    #    end rather than "safe narrowing".
    if (
        persistent_count < config.min_persistent_sectors
        and (int(s1.max_run) - int(s3.max_run)) >= config.rapid_shrink_run_drop
    ):
        return CORRIDOR_DEAD_END
    # E. unstable route: the corridor jumps or barely overlaps between anchors.
    direction_jump = max(
        circular_sector_distance(s1.direction, s2.direction),
        circular_sector_distance(s2.direction, s3.direction),
    )
    hamming_jump = max(
        mask_hamming(s1.safe_mask, s2.safe_mask),
        mask_hamming(s2.safe_mask, s3.safe_mask),
    )
    if (
        direction_jump >= config.unstable_direction_jump
        or hamming_jump >= config.unstable_mask_jump
        or overlap_12 < config.unstable_min_overlap
        or overlap_23 < config.unstable_min_overlap
    ):
        return CORRIDOR_UNSTABLE
    # F. stable: a direction persists and every anchor keeps a usable corridor.
    if persistent_count < config.min_persistent_sectors:
        return CORRIDOR_NONE
    if min(s.max_run for s in spaces) < config.min_run:
        return CORRIDOR_NONE
    if min(s.corridor_clearance for s in spaces) < config.clearance_threshold:
        return CORRIDOR_NONE
    return CORRIDOR_STABLE


# ==========================================================================
# per-candidate descriptor
# ==========================================================================
@dataclass(frozen=True)
class FutureSpaceDescriptor:
    """FSC result for one candidate (three anchors)."""

    valid: bool
    candidate_id: int
    anchors: np.ndarray
    per_anchor: tuple[AnchorSpace, ...]
    reach_12: bool
    reach_23: bool
    corridor_type: str
    future_target_valid: bool
    future_target_x: float
    future_target_y: float
    future_target_source: str
    corridor_match: tuple[bool, bool, bool]
    corridor_match_count: int
    #: ``mask_t1 & mask_t2 & mask_t3`` -- a direction safe at every anchor.
    persistent_mask: int = 0
    #: popcount(mask_t1 & mask_t2) / popcount(mask_t2 & mask_t3).
    overlap_12: int = 0
    overlap_23: int = 0
    #: Threshold echo (see :meth:`FutureCorridorConfig.to_dict`).
    thresholds: dict = field(default_factory=dict)

    # -- convenience views ------------------------------------------------
    @property
    def persistent_sectors(self) -> int:
        return mask_popcount(self.persistent_mask)

    @property
    def mask_overlap_12(self) -> bool:
        return self.overlap_12 > 0

    @property
    def mask_overlap_23(self) -> bool:
        return self.overlap_23 > 0

    @property
    def safe_masks(self) -> np.ndarray:
        return np.asarray([s.safe_mask for s in self.per_anchor], dtype=np.int64)

    @property
    def widths(self) -> np.ndarray:
        return np.asarray([s.max_run for s in self.per_anchor], dtype=np.int64)

    @property
    def min_clearances(self) -> np.ndarray:
        return np.asarray([s.min_clearance for s in self.per_anchor], dtype=np.float64)

    @property
    def safe_rect_valids(self) -> np.ndarray:
        return np.asarray([s.safe_rect_valid for s in self.per_anchor], dtype=bool)

    @property
    def corridor_match_t1(self) -> bool:
        return self.corridor_match[0]

    @property
    def corridor_match_t2(self) -> bool:
        return self.corridor_match[1]

    @property
    def corridor_match_t3(self) -> bool:
        return self.corridor_match[2]

    def to_dict(self) -> dict:
        return {
            "valid": bool(self.valid),
            "candidate_id": int(self.candidate_id),
            "corridor_type": self.corridor_type,
            "reach_12": bool(self.reach_12),
            "reach_23": bool(self.reach_23),
            "persistent_mask": int(self.persistent_mask),
            "persistent_sectors": int(self.persistent_sectors),
            "overlap_12": int(self.overlap_12),
            "overlap_23": int(self.overlap_23),
            "mask_overlap_12": bool(self.mask_overlap_12),
            "mask_overlap_23": bool(self.mask_overlap_23),
            "thresholds": dict(self.thresholds),
            "future_target_valid": bool(self.future_target_valid),
            "future_target_x": float(self.future_target_x),
            "future_target_y": float(self.future_target_y),
            "future_target_source": self.future_target_source,
            "corridor_match_t1": bool(self.corridor_match[0]),
            "corridor_match_t2": bool(self.corridor_match[1]),
            "corridor_match_t3": bool(self.corridor_match[2]),
            "corridor_match_count": int(self.corridor_match_count),
            "anchors": [float(a) for a in self.anchors],
            "per_anchor": [s.to_dict() for s in self.per_anchor],
        }


def _target_from_spaces(
    spaces: Sequence[AnchorSpace], config: FutureCorridorConfig
) -> tuple[bool, float, float, str]:
    """Future target: R3 centre, only when R3 is provably valid and clear."""
    s3 = spaces[2]
    if s3.has_usable_corridor(config) and s3.safe_rect_valid:
        xmin, xmax, ymin, ymax = s3.rect
        return True, 0.5 * (xmin + xmax), 0.5 * (ymin + ymax), "rect_center"
    return False, 0.0, 0.0, "none"


def descriptor_from_spaces(
    spaces: Sequence[AnchorSpace],
    times: object,
    v_max: float,
    config: FutureCorridorConfig,
    *,
    candidate_id: int = 0,
    future_positions: Sequence[tuple[float, float]] | None = None,
    timing: dict | None = None,
) -> FutureSpaceDescriptor:
    """Reachability + funnel classification + target + pre-positioning match.

    Pure arithmetic on the three anchor spaces, so toy scenarios can drive it
    with synthetic SafeMasks and SafeRects without any obstacle geometry.
    """
    spaces = tuple(spaces)
    if len(spaces) != 3:
        raise ValueError("descriptor_from_spaces expects exactly three anchors")
    times_arr = np.asarray(times, dtype=np.float64)
    if future_positions is None:
        future_positions = tuple(
            (
                0.5 * (s.rect[0] + s.rect[1]),
                0.5 * (s.rect[2] + s.rect[3]),
            )
            for s in spaces
        )
    points = tuple((float(p[0]), float(p[1])) for p in future_positions)

    r1, r2, r3 = spaces[0].rect, spaces[1].rect, spaces[2].rect
    t_reach = time.perf_counter()
    reach_12 = bool(
        spaces[0].safe_rect_valid
        and spaces[1].safe_rect_valid
        and rect_reachable(r1, r2, v_max, float(times_arr[1]) - float(times_arr[0]))
    )
    reach_23 = bool(
        spaces[1].safe_rect_valid
        and spaces[2].safe_rect_valid
        and rect_reachable(r2, r3, v_max, float(times_arr[2]) - float(times_arr[1]))
    )
    persistent = persistent_mask(
        spaces[0].safe_mask, spaces[1].safe_mask, spaces[2].safe_mask
    )
    overlap_12 = mask_popcount(spaces[0].safe_mask & spaces[1].safe_mask)
    overlap_23 = mask_popcount(spaces[1].safe_mask & spaces[2].safe_mask)
    t_corridor = time.perf_counter()
    corridor_type = classify_corridor(
        spaces,
        reach_12,
        reach_23,
        config,
        persistent=persistent,
        overlap_12=overlap_12,
        overlap_23=overlap_23,
    )
    target_valid, tx, ty, source = _target_from_spaces(spaces, config)
    if timing is not None:
        timing["reachability_us"] = timing.get("reachability_us", 0.0) + (
            t_corridor - t_reach
        ) * 1e6
        timing["corridor_us"] = timing.get("corridor_us", 0.0) + (
            time.perf_counter() - t_corridor
        ) * 1e6
    matches = tuple(
        bool(
            spaces[k].safe_rect_valid
            and spaces[k].rect[0] <= points[k][0] <= spaces[k].rect[1]
            and spaces[k].rect[2] <= points[k][1] <= spaces[k].rect[3]
        )
        for k in range(3)
    )
    return FutureSpaceDescriptor(
        valid=True,
        candidate_id=int(candidate_id),
        anchors=times_arr,
        per_anchor=spaces,
        reach_12=reach_12,
        reach_23=reach_23,
        corridor_type=corridor_type,
        future_target_valid=bool(target_valid),
        future_target_x=float(tx),
        future_target_y=float(ty),
        future_target_source=source,
        corridor_match=matches,
        corridor_match_count=int(sum(matches)),
        persistent_mask=int(persistent),
        overlap_12=int(overlap_12),
        overlap_23=int(overlap_23),
        thresholds=config.to_dict(),
    )


# ==========================================================================
# DSS v6.0 shadow container
# ==========================================================================
@dataclass(frozen=True)
class DSSv6ShadowHeader:
    version: str
    build_id: str
    anchors: tuple[float, float, float]
    n_candidates: int
    n_obstacles: int
    horizon: float
    #: Echo of every FSC threshold used for this frame.
    thresholds: dict = field(default_factory=dict)
    shadow_only: bool = True


@dataclass(frozen=True)
class DSSv6ShadowGlobalSafety:
    """Read-only copy of the v0.5.2 safety scalars (never written back)."""

    emergency: bool
    n_feasible: int
    chosen_index: int
    ltv_override: bool
    viability_evaluated: int
    risk_band_delta: int
    chosen_risk: float
    chosen_viability: float


@dataclass(frozen=True)
class DSSv6ShadowCandidateRisk:
    """Per-candidate v0.5.2 risk plus the FSC corridor class."""

    candidate_id: int
    speed: float
    steering_deg: float
    is_stop: bool
    risk: float
    ltv_viability: float
    fsc_corridor_type: str
    fsc_width_t1: int
    fsc_width_t2: int
    fsc_width_t3: int
    fsc_reach_12: bool
    fsc_reach_23: bool
    fsc_persistent_mask: int
    fsc_overlap_12: int
    fsc_overlap_23: int
    fsc_match_count: int
    fsc_target_valid: bool


@dataclass(frozen=True)
class DSSv6Shadow:
    """Shadow decision-support snapshot: header + global + [17] risk + [17][3] space."""

    header: DSSv6ShadowHeader
    global_safety: DSSv6ShadowGlobalSafety
    candidate_risk: tuple[DSSv6ShadowCandidateRisk, ...]
    future_space: tuple[tuple[dict, dict, dict], ...]

    def to_dict(self) -> dict:
        return {
            "header": {
                "version": self.header.version,
                "build_id": self.header.build_id,
                "anchors": list(self.header.anchors),
                "n_candidates": self.header.n_candidates,
                "n_obstacles": self.header.n_obstacles,
                "horizon": self.header.horizon,
                "thresholds": dict(self.header.thresholds),
                "shadow_only": self.header.shadow_only,
            },
            "global_safety": {
                "emergency": self.global_safety.emergency,
                "n_feasible": self.global_safety.n_feasible,
                "chosen_index": self.global_safety.chosen_index,
                "ltv_override": self.global_safety.ltv_override,
                "viability_evaluated": self.global_safety.viability_evaluated,
                "risk_band_delta": self.global_safety.risk_band_delta,
                "chosen_risk": self.global_safety.chosen_risk,
                "chosen_viability": self.global_safety.chosen_viability,
            },
            "candidate_risk": [
                {
                    "candidate_id": c.candidate_id,
                    "speed": c.speed,
                    "steering_deg": c.steering_deg,
                    "is_stop": c.is_stop,
                    "risk": c.risk,
                    "ltv_viability": c.ltv_viability,
                    "fsc_corridor_type": c.fsc_corridor_type,
                    "corridor_type": c.fsc_corridor_type,
                    "fsc_width_t1": c.fsc_width_t1,
                    "fsc_width_t2": c.fsc_width_t2,
                    "fsc_width_t3": c.fsc_width_t3,
                    "fsc_reach_12": c.fsc_reach_12,
                    "fsc_reach_23": c.fsc_reach_23,
                    "fsc_persistent_mask": c.fsc_persistent_mask,
                    "fsc_overlap_12": c.fsc_overlap_12,
                    "fsc_overlap_23": c.fsc_overlap_23,
                    "fsc_match_count": c.fsc_match_count,
                    "fsc_target_valid": c.fsc_target_valid,
                }
                for c in self.candidate_risk
            ],
            "future_space": [[dict(a) for a in row] for row in self.future_space],
        }


@dataclass(frozen=True)
class ShadowFrame:
    """One frame of FSC shadow diagnostics (never an action)."""

    candidate_count: int
    stable_funnel_count: int
    dead_end_funnel_count: int
    unstable_funnel_count: int
    none_funnel_count: int
    unknown_funnel_count: int
    chosen_index: int
    chosen: FutureSpaceDescriptor
    per_candidate: tuple[FutureSpaceDescriptor, ...]
    evaluated_indices: tuple[int, ...]
    safe_rect_us: float
    reachability_us: float
    corridor_us: float
    total_shadow_us: float
    dss: DSSv6Shadow
    #: The planner's real Top-K shortlist (<= 4), in planner order; empty when no
    #: planner shortlist existed and the documented shadow proxy was used.
    topk_indices: tuple[int, ...] = ()
    #: ``"planner_topk"`` | ``"shadow_proxy"``.
    candidate_source: str = ""

    @property
    def fsc_compute_us(self) -> float:
        """Time in the SafeMask/SafeRect phase (previous-round alias)."""
        return self.safe_rect_us

    @property
    def diagnostics_us(self) -> float:
        """Whole-shadow time (previous-round alias)."""
        return self.total_shadow_us

    def to_dict(self) -> dict:
        return {
            "candidate_count": int(self.candidate_count),
            "stable_funnel_count": int(self.stable_funnel_count),
            "dead_end_funnel_count": int(self.dead_end_funnel_count),
            "unstable_funnel_count": int(self.unstable_funnel_count),
            "none_funnel_count": int(self.none_funnel_count),
            "unknown_funnel_count": int(self.unknown_funnel_count),
            "chosen_index": int(self.chosen_index),
            "evaluated_indices": [int(i) for i in self.evaluated_indices],
            "topk_indices": [int(i) for i in self.topk_indices],
            "candidate_source": self.candidate_source,
            "safe_rect_us": float(self.safe_rect_us),
            "reachability_us": float(self.reachability_us),
            "corridor_us": float(self.corridor_us),
            "total_shadow_us": float(self.total_shadow_us),
            "chosen": self.chosen.to_dict(),
            "per_candidate": [d.to_dict() for d in self.per_candidate],
            "dss": self.dss.to_dict(),
        }


# ==========================================================================
# the shadow evaluator
# ==========================================================================
class SafeCorridorShadow:
    """Stateless FSC evaluator.  Produces diagnostics; never an action.

    Off by default (:data:`SAFE_CORRIDOR_SHADOW_ENABLED`, env
    ``DF_ENABLE_SAFE_CORRIDOR_SHADOW``).  While disabled :meth:`observe` returns
    ``None`` and nothing at all is computed, so a caller may wire it in
    unconditionally and still emit exactly the v0.5.2 action.
    """

    def __init__(
        self,
        config: FutureCorridorConfig | None = None,
        *,
        enabled: bool | None = None,
    ) -> None:
        self.config = config or FutureCorridorConfig()
        self.enabled = (
            SAFE_CORRIDOR_SHADOW_ENABLED if enabled is None else bool(enabled)
        )

    # ------------------------------------------------------------------
    def anchors(self, context: ViabilityContext | None = None) -> np.ndarray:
        """The three FSC anchor times (fixed in v0.6.0; scene_scale is not used)."""
        return np.asarray(self.config.anchors, dtype=np.float64)

    # ------------------------------------------------------------------
    def _anchor_spaces(
        self,
        scene: SceneState,
        action: Action,
        context: ViabilityContext,
        anchors: np.ndarray,
    ) -> tuple[tuple[AnchorSpace, ...], tuple[tuple[float, float], ...], float]:
        """SafeMask / max_run / clearance / SafeRect for the three anchors."""
        cfg = self.config
        agent = scene.agent
        v_cand = action.velocity_vector[None, :]                     # (1,2)
        obstacles = scene.obstacles
        if obstacles:
            positions = np.stack([o.position for o in obstacles]).astype(np.float64)
            velocities = np.stack([o.velocity for o in obstacles]).astype(np.float64)
            radii = np.asarray([o.radius for o in obstacles], dtype=np.float64)
            radius_sum = agent.radius + radii
            v_rel = velocities[None, :, :] - v_cand[:, None, :]       # (1,N,2)
        else:
            positions = np.zeros((0, 2), dtype=np.float64)
            velocities = np.zeros((0, 2), dtype=np.float64)
            radii = np.zeros(0, dtype=np.float64)
            radius_sum = np.zeros(0, dtype=np.float64)
            v_rel = np.zeros((1, 0, 2), dtype=np.float64)

        spaces: list[AnchorSpace] = []
        future_positions: list[tuple[float, float]] = []
        rect_us = 0.0
        for t in anchors:
            t = float(t)
            p_agent = agent.position[None, :] + v_cand * t             # (1,2)
            if positions.shape[0]:
                p_obs = positions[None, :, :] + velocities[None, :, :] * t
            else:
                p_obs = np.zeros((1, 0, 2), dtype=np.float64)
            r = p_obs - p_agent[:, None, :]                            # (1,N,2)
            nearest, closing, _density, clearance = _sector_summary_batch(
                r, v_rel, radius_sum
            )
            # identical dynamic-buffer semantics as the v0.5.2 mobility kernel
            closing_pos = np.maximum(closing, 0.0)
            dynamic = np.minimum(
                float(context.base_buffer) + closing_pos * float(context.margin_time),
                float(cfg.max_dynamic_buffer),
            )
            free = (nearest - dynamic) >= 0.0                          # (1,8)
            mask = bool_to_mask(free[0])
            max_run, run_start = mask_max_run(mask)
            if max_run > 0 and nearest.shape[1]:
                sector_pos = np.arange(nearest.shape[1])
                offset = (sector_pos - run_start) % 8
                in_corridor = offset < max_run
                corridor_clear = float(
                    np.where(
                        in_corridor & np.isfinite(nearest[0]), nearest[0], np.inf
                    ).min()
                )
                if not np.isfinite(corridor_clear):
                    # no finite obstacle bound inside the corridor -> fully open
                    corridor_clear = float("inf")
            else:
                corridor_clear = 0.0
            min_clear = (
                float(clearance.min()) if clearance.shape[1] else float("inf")
            )
            t_rect = time.perf_counter()
            rect_valid, rect, rect_reason = safe_rect_from_obstacles(
                p_agent[0],
                p_obs[0],
                radii,
                agent_radius=agent.radius,
                buffer=float(context.base_buffer),
                max_half=cfg.max_rect_half,
                min_half=cfg.min_rect_half,
            )
            rect_us += (time.perf_counter() - t_rect) * 1e6
            spaces.append(
                AnchorSpace(
                    anchor=t,
                    safe_mask=mask,
                    max_run=int(max_run),
                    run_start=int(run_start),
                    min_clearance=min_clear,
                    corridor_clearance=corridor_clear,
                    safe_rect_valid=bool(rect_valid),
                    rect=rect,
                    safe_rect_invalid_reason=str(rect_reason),
                )
            )
            future_positions.append((float(p_agent[0, 0]), float(p_agent[0, 1])))
        return tuple(spaces), tuple(future_positions), float(rect_us)

    # ------------------------------------------------------------------
    def evaluate(
        self,
        scene: SceneState,
        action: Action,
        context: ViabilityContext,
        *,
        candidate_id: int = 0,
        anchors: np.ndarray | None = None,
        timing: dict | None = None,
    ) -> FutureSpaceDescriptor:
        """Full FSC descriptor for one candidate."""
        times = np.asarray(anchors if anchors is not None else self.anchors(context))
        spaces, future_positions, rect_us = self._anchor_spaces(
            scene, action, context, times
        )
        if timing is not None:
            timing["safe_rect_us"] = timing.get("safe_rect_us", 0.0) + rect_us
        return descriptor_from_spaces(
            spaces,
            times,
            float(scene.agent.max_speed),
            self.config,
            candidate_id=candidate_id,
            future_positions=future_positions,
            timing=timing,
        )

    # ------------------------------------------------------------------
    def select_shadow_candidates(
        self, actions: Sequence[Action], chosen_index: int, k: int = 4
    ) -> tuple[int, ...]:
        """Shadow-only candidate subset: the chosen action plus ``k-1`` headings.

        The real v0.5.2 Top-K shortlist lives inside the planner and is NOT read
        here (that would require touching planner.py).  This proxy keeps the FSC
        cost at O(K*3*N) and is documented as a first-version stand-in.
        """
        actions = tuple(actions)
        n = len(actions)
        if n == 0:
            return ()
        chosen = int(chosen_index) if 0 <= int(chosen_index) < n else 0
        picked = [chosen]
        step = max(1, n // max(1, k))
        i = (chosen + step) % n
        while len(picked) < min(k, n):
            if i not in picked:
                picked.append(i)
            i = (i + step) % n
            if len(picked) >= min(k, n):
                break
        return tuple(picked[: min(k, n)])

    # ------------------------------------------------------------------
    def observe(
        self,
        scene: SceneState,
        actions: Sequence[Action],
        context: ViabilityContext,
        *,
        chosen_index: int = 0,
        k: int = 4,
        anchors: np.ndarray | None = None,
        v052_diagnostics=None,
        candidate_indices: Sequence[int] | None = None,
        version: str = "v0.6.0-SafeCorridor-Shadow",
        build_id: str = "",
    ) -> ShadowFrame | None:
        """Frame-level shadow diagnostics.  Returns data only -- no action.

        ``candidate_indices`` should be the planner's *real* Top-K shortlist
        (``PlanDiagnostics.fsc_candidate_indices``); it is preferred over the
        shadow's own proxy.  The chosen candidate is always evaluated as well, so
        ``chosen`` is never a stand-in, but it is not part of the Top-K list.

        Returns ``None`` when the shadow is disabled (the default) without
        computing anything.
        """
        if not self.enabled:
            return None
        actions = tuple(actions)
        t0 = time.perf_counter()
        topk: tuple[int, ...] = ()
        if candidate_indices:
            topk = tuple(
                dict.fromkeys(
                    int(i) for i in candidate_indices if 0 <= int(i) < len(actions)
                )
            )
        if topk:
            candidate_source = "planner_topk"
        else:
            topk = self.select_shadow_candidates(actions, chosen_index, k=k)
            candidate_source = "shadow_proxy"
        subset = tuple(dict.fromkeys(topk + (int(chosen_index),)))
        timing: dict = {}
        descriptors: list[FutureSpaceDescriptor] = []
        for i in subset:
            descriptors.append(
                self.evaluate(
                    scene,
                    actions[i],
                    context,
                    candidate_id=i,
                    anchors=anchors,
                    timing=timing,
                )
            )
        t1 = time.perf_counter()

        counts = {name: 0 for name in CORRIDOR_TYPES}
        for d in descriptors:
            counts[d.corridor_type] = counts.get(d.corridor_type, 0) + 1
        chosen = next(
            (d for d in descriptors if d.candidate_id == int(chosen_index)),
            descriptors[0] if descriptors else None,
        )
        if chosen is None:  # pragma: no cover - defensive
            raise ValueError("observe() needs at least one candidate")

        diag = v052_diagnostics
        header = DSSv6ShadowHeader(
            version=version,
            build_id=build_id,
            anchors=tuple(float(a) for a in (anchors if anchors is not None else self.anchors(context))),
            n_candidates=len(actions),
            n_obstacles=len(scene.obstacles),
            horizon=float(scene.horizon),
            thresholds=self.config.to_dict(),
        )
        global_safety = DSSv6ShadowGlobalSafety(
            emergency=bool(getattr(diag, "emergency", False)),
            n_feasible=int(getattr(diag, "n_feasible", 0) or 0),
            chosen_index=int(chosen_index),
            ltv_override=bool(getattr(diag, "ltv_override", False)),
            viability_evaluated=int(getattr(diag, "viability_evaluated", 0) or 0),
            risk_band_delta=int(getattr(diag, "risk_band_delta", 0) or 0),
            chosen_risk=float(getattr(diag, "chosen_risk_peak", 0.0) or 0.0),
            chosen_viability=float(getattr(diag, "chosen_viability", 0.0) or 0.0),
        )
        by_id = {d.candidate_id: d for d in descriptors}
        candidate_risk = tuple(
            DSSv6ShadowCandidateRisk(
                candidate_id=i,
                speed=float(actions[i].speed),
                steering_deg=float(np.degrees(actions[i].steering_angle)),
                is_stop=bool(actions[i].is_stop),
                # -1.0 = not evaluated by the read-only v0.5.2 diagnostics
                risk=float(getattr(diag, "chosen_risk_peak", 0.0) or 0.0)
                if i == int(chosen_index)
                else -1.0,
                ltv_viability=float(getattr(diag, "chosen_viability", 0.0) or 0.0)
                if i == int(chosen_index)
                else -1.0,
                fsc_corridor_type=by_id[i].corridor_type if i in by_id else "",
                fsc_width_t1=int(by_id[i].widths[0]) if i in by_id else -1,
                fsc_width_t2=int(by_id[i].widths[1]) if i in by_id else -1,
                fsc_width_t3=int(by_id[i].widths[2]) if i in by_id else -1,
                fsc_reach_12=bool(by_id[i].reach_12) if i in by_id else False,
                fsc_reach_23=bool(by_id[i].reach_23) if i in by_id else False,
                fsc_persistent_mask=int(by_id[i].persistent_mask) if i in by_id else 0,
                fsc_overlap_12=int(by_id[i].overlap_12) if i in by_id else 0,
                fsc_overlap_23=int(by_id[i].overlap_23) if i in by_id else 0,
                fsc_match_count=int(by_id[i].corridor_match_count) if i in by_id else -1,
                fsc_target_valid=bool(by_id[i].future_target_valid) if i in by_id else False,
            )
            for i in range(len(actions))
        )
        future_space = tuple(
            tuple(
                (
                    {
                        **by_id[i].per_anchor[k].to_dict(),
                        # the candidate's corridor class, carried on every anchor
                        # row so a consumer never has to join back to CandidateRisk
                        "corridor_type": by_id[i].corridor_type,
                    }
                    if i in by_id
                    else {}
                )
                for k in range(3)
            )
            for i in range(len(actions))
        )
        dss = DSSv6Shadow(
            header=header,
            global_safety=global_safety,
            candidate_risk=candidate_risk,
            future_space=future_space,
        )
        t2 = time.perf_counter()
        return ShadowFrame(
            candidate_count=len(descriptors),
            stable_funnel_count=counts[CORRIDOR_STABLE],
            dead_end_funnel_count=counts[CORRIDOR_DEAD_END],
            unstable_funnel_count=counts[CORRIDOR_UNSTABLE],
            none_funnel_count=counts[CORRIDOR_NONE],
            unknown_funnel_count=counts[CORRIDOR_UNKNOWN],
            chosen_index=int(chosen_index),
            topk_indices=tuple(int(i) for i in topk),
            candidate_source=candidate_source,
            chosen=chosen,
            per_candidate=tuple(descriptors),
            evaluated_indices=tuple(int(i) for i in subset),
            safe_rect_us=float(timing.get("safe_rect_us", 0.0)),
            reachability_us=float(timing.get("reachability_us", 0.0)),
            corridor_us=float(timing.get("corridor_us", 0.0)),
            total_shadow_us=(t2 - t0) * 1e6,
            dss=dss,
        )
