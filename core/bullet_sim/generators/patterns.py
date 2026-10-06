"""Parameterised bullet-pattern generators.

Every generator is a pure function ``PatternSpec -> [SpawnEvent]``.  They run
**once, at scenario build time**, using only the scenario's seeded RNG, and they
never touch the live world.  Results are therefore cacheable, hashable, and
directly portable to a CPU/FPGA emitter table.

Implemented patterns
--------------------
``radial``  n bullets on an arc/ring starting at ``angle``
``spiral``  rotating multi-arm emitter, one emission per interval
``aimed``   fan centred on the direction towards the live player
``burst``   instantaneous ring (optionally offset onto a spawn ring)
``line``    evenly spaced bullets along the axis perpendicular to travel
``wall``    full-span curtain perpendicular to travel
``random``  randomised positions/directions (seeded)
``mixed``   composition of several of the above
"""

from __future__ import annotations

import math
from typing import Callable

import numpy as np

from bullet_sim.core.clock import FixedClock
from bullet_sim.core.errors import ScenarioError
from bullet_sim.core.numerics import TYPE_DTYPE
from bullet_sim.generators.burst import ORIGIN_FIXED, GenerationContext, SpawnEvent
from bullet_sim.entities.bullet import shape_code
from bullet_sim.generators.spec import PATTERN_TYPE_IDS, PatternSpec

_TWO_PI = 2.0 * math.pi
_PI = math.pi
_DEG = math.pi / 180.0
_FULL_RING_EPS = 1e-9


# --------------------------------------------------------------------------
# scheduling
# --------------------------------------------------------------------------


def emission_steps(spec: PatternSpec, ctx: GenerationContext) -> list[int]:
    """Deterministic list of simulation steps at which ``spec`` emits.

    Periodic emitters use an **integer** step period so that N emissions are
    exactly ``N * period`` steps apart regardless of ``dt`` rounding.
    """
    clock = ctx.clock or FixedClock.make(dt=ctx.dt)
    start = clock.step_of(spec.start_time)
    if spec.interval <= 0.0 or spec.repetitions == 1:
        return [start]
    if spec.repetitions > 1:
        n = spec.repetitions
    else:  # repetitions <= 0  ->  emit until the scenario ends
        remaining = max(0.0, ctx.duration - spec.start_time)
        n = int(math.floor(remaining / spec.interval + 1e-9)) + 1
        n = max(n, 1)
    period = max(1, int(round(spec.interval / ctx.dt)))
    return [start + i * period for i in range(n)]


# --------------------------------------------------------------------------
# shared helpers
# --------------------------------------------------------------------------


def _count(spec: PatternSpec) -> int:
    """Resolved bullet count (``None`` -> the pattern's natural default)."""
    return max(0, int(spec.resolved_count))


def _speeds(spec: PatternSpec, ctx: GenerationContext, n: int) -> np.ndarray:
    if n == 0:
        return np.zeros(0)
    if spec.speed_var:
        return spec.speed + ctx.rng.uniform(-spec.speed_var, spec.speed_var, n)
    return np.full(n, float(spec.speed))


def _angle_jitter(spec: PatternSpec, ctx: GenerationContext, n: int) -> np.ndarray:
    if n == 0:
        return np.zeros(0)
    if spec.angle_var:
        return ctx.rng.uniform(-spec.angle_var, spec.angle_var, n) * _DEG
    return np.zeros(n)


def _radii(spec: PatternSpec, ctx: GenerationContext, n: int) -> np.ndarray:
    if n == 0:
        return np.zeros(0)
    if spec.radius_var:
        r = spec.radius + ctx.rng.uniform(-spec.radius_var, spec.radius_var, n)
        return np.maximum(r, 1e-6)
    return np.full(n, float(spec.radius))


def _pos_jitter(spec: PatternSpec, ctx: GenerationContext, shape: tuple[int, int]) -> np.ndarray:
    n = shape[0]
    if spec.jitter_pos and n:
        return ctx.rng.uniform(-spec.jitter_pos, spec.jitter_pos, shape)
    return np.zeros(shape, dtype=np.float64)


def _ttls(spec: PatternSpec, n: int) -> np.ndarray:
    return np.full(n, float(spec.ttl))


def _accel_arrays(
    spec: PatternSpec, n: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(magnitude, angle)``; ``angle = nan`` means "along velocity"."""
    mag = np.full(n, float(spec.accel))
    if spec.accel == 0.0:
        ang = np.full(n, np.nan)
    elif spec.accel_direction is None:
        ang = np.full(n, np.nan)
    else:
        ang = np.full(n, float(spec.accel_direction) * _DEG)
    return mag, ang


def _type_ids(spec: PatternSpec, n: int) -> np.ndarray:
    return np.full(n, int(spec.type_id if spec.type_id is not None else 0), dtype=TYPE_DTYPE)


def _geo(spec: PatternSpec, n: int, radius: np.ndarray) -> dict[str, Any]:
    """Per-bullet geometry arrays for an obstacle batch."""
    from bullet_sim.entities.bullet import shape_code

    hw = radius if spec.half_w is None else np.full(n, float(spec.half_w))
    hh = radius if spec.half_h is None else np.full(n, float(spec.half_h))
    if spec.shape != "circle" and spec.half_w is not None and n and not np.isscalar(spec.half_w):
        hw = np.asarray(spec.half_w, dtype=np.float64)
    return {
        "shape": np.full(n, shape_code(spec.shape), dtype=TYPE_DTYPE),
        "half_w": np.asarray(hw, dtype=np.float64),
        "half_h": np.asarray(hh, dtype=np.float64),
        "rotation": np.full(n, float(spec.rotation)),
    }


def _make_event(
    spec: PatternSpec,
    ctx: GenerationContext,
    step: int,
    offsets: np.ndarray,
    angles: np.ndarray,
    speeds: np.ndarray,
) -> SpawnEvent:
    n = offsets.shape[0]
    origin, mode = ctx.origin_xy(spec.origin)
    accel_mag, accel_angle = _accel_arrays(spec, n)
    return SpawnEvent(
        step=step,
        origin=origin,
        origin_mode=mode,
        aim=bool(spec.aim),
        group_id=int(spec.group_id),
        pattern=spec.kind,
        offsets=offsets,
        angles=angles,
        speeds=speeds,
        accel_mag=accel_mag,
        accel_angle=accel_angle,
        radius=_radii(spec, ctx, n),
        ttl=_ttls(spec, n),
        type_id=_type_ids(spec, n),
        angular_velocity=np.full(n, float(spec.angular_velocity)),
        min_player_distance=float(spec.min_player_distance),
        **_geo(spec, n, _radii(spec, ctx, n)),
    )


def _even_angles(base_rad: float, spread_deg: float, n: int) -> np.ndarray:
    if n <= 0:
        return np.zeros(0)
    spread = float(spread_deg) * _DEG
    full = abs(spread_deg) >= 360.0 - _FULL_RING_EPS
    if n == 1:
        return np.array([base_rad])
    if full:
        return base_rad + np.linspace(0.0, _TWO_PI, n, endpoint=False)
    return base_rad + np.linspace(0.0, spread, n, endpoint=True)


# --------------------------------------------------------------------------
# generators
# --------------------------------------------------------------------------


# --------------------------------------------------------------------------
# dynamic-obstacle layouts
#
# Realism rules every layout obeys:
#   1. an obstacle is born on a boundary (or spans the field as a wall/corridor)
#      - never at the field centre;
#   2. it arrives with an entry direction that points *into* the field;
#   3. every opening (gap, corridor) admits the player circle.
# --------------------------------------------------------------------------

#: Spawn region keywords -> an anchor on the field boundary.
SPAWN_REGIONS: tuple[str, ...] = (
    "left", "right", "top", "bottom", "farthest_side", "nearest_side",
)


def _region_anchor(region: str, ctx: GenerationContext) -> tuple[float, float, float]:
    """``(x, y, inward_angle_rad)`` for a spawn region keyword.

    Angles point *into* the field, so an obstacle spawned on the left edge
    travels rightwards unless the caller overrides it.
    """
    w, h = ctx.field_w, ctx.field_h
    table = {
        "left": (0.0, h * 0.5, 0.0),
        "right": (w, h * 0.5, _PI),
        "bottom": (w * 0.5, 0.0, _PI / 2),
        "top": (w * 0.5, h, -_PI / 2),
    }
    if region in ("farthest_side", "nearest_side"):
        # pick the horizontal side the player is *not* near
        px = float(ctx.player_xy[0])
        near_left = px < w * 0.5
        if region == "nearest_side":
            near_left = not near_left
        return table["left"] if near_left else table["right"]
    return table.get(region, table["left"])


def _edge_point(
    region: str, ctx: GenerationContext, t: float, *, outward: float = 0.0
) -> tuple[float, float, float, float]:
    """A point on the named boundary edge plus its inward unit normal.

    ``t`` slides along the edge; ``outward`` pushes the point *outside* the
    field so the obstacle visibly enters instead of appearing on the border.
    Keeping the edge coordinate (and only randomising along the edge) is what
    guarantees "born on a boundary, never on the centre line".
    """
    x, y, inward = _region_anchor(region, ctx)
    w, h = ctx.field_w, ctx.field_h
    vertical_edge = abs(math.cos(inward)) >= 0.5      # left / right edge
    if vertical_edge:
        px, py = x, float(t) * h
    else:
        px, py = float(t) * w, y
    nx, ny = math.cos(inward), math.sin(inward)
    return px - nx * outward, py - ny * outward, nx, ny


def gen_obstacle_field(spec: PatternSpec, ctx: GenerationContext) -> list[SpawnEvent]:
    """Emit dynamic obstacles according to ``params['layout']``.

    Layouts (each has a real-world reading, see ``bullet_sim/obstacles/catalog.py``):

    ``block``     a large rectangle entering from a *band* of one boundary
    ``wall_gap``  a wall spanning the field with one passable opening
    ``small``     many small circles entering from the edges
    ``cross``     obstacles entering from several sides at once
    ``corridor``  a tilted channel of two long walls, optionally narrowing
    """
    params = spec.params
    layout = str(params.get("layout", "block")).lower()
    if layout not in LAYOUTS:
        raise ScenarioError(
            f"unknown obstacle layout {layout!r}; expected {sorted(LAYOUTS)}"
        )
    out: list[SpawnEvent] = []
    start_step = (ctx.clock or FixedClock.make(dt=ctx.dt)).step_of(spec.start_time)
    for step in emission_steps(spec, ctx):
        elapsed = (step - start_step) * ctx.dt
        builder = _LAYOUT_BUILDERS[layout]
        offsets, angles, speeds, hw, hh, rot, spin, aimed = builder(spec, ctx, elapsed, step)
        n = offsets.shape[0]
        if n == 0:
            continue
        ev = SpawnEvent(
            step=step,
            origin=np.array([0.0, 0.0], dtype=np.float64),
            origin_mode=ORIGIN_FIXED,
            aim=bool(aimed),
            group_id=int(spec.group_id),
            pattern=f"obstacle:{layout}",
            offsets=offsets,
            angles=angles,
            speeds=speeds,
            accel_mag=_accel_arrays(spec, n)[0],
            accel_angle=_accel_arrays(spec, n)[1],
            radius=np.full(n, float(spec.radius)),
            ttl=_ttls(spec, n),
            type_id=_type_ids(spec, n),
            angular_velocity=np.asarray(spin, dtype=np.float64),
            shape=np.full(n, shape_code(spec.shape), dtype=TYPE_DTYPE),
            half_w=np.asarray(hw, dtype=np.float64),
            half_h=np.asarray(hh, dtype=np.float64),
            rotation=np.asarray(rot, dtype=np.float64),
            min_player_distance=float(
                params.get("min_player_distance", spec.min_player_distance)
            ),
        )
        out.append(ev)
    return out


def _layout_block(spec, ctx, elapsed, step):
    """Large rectangles entering from a *band* of the boundary, strictly inward.

    Fixes two things the single-anchor version got wrong:

    * the block is born somewhere along an ``entry_span`` fraction of the edge
      instead of at one point, so several blocks can enter side by side;
    * the velocity is forced to have a positive component along the inward
      normal, so a block can never be emitted running *out* of the workspace.
    """
    region = str(spec.params.get("spawn", "top"))
    span_frac = float(np.clip(spec.params.get("entry_span", 0.6), 0.0, 1.0))
    n = max(1, _count(spec))
    half = float(spec.half_w or 60.0)
    across = float(spec.half_h or half)
    explicit = spec.params.get("entry_angle")
    #: Blocks are placed at *random* positions along the entry band (seeded), so
    #: a scene does not replay the same three lanes every time.
    lo, hi = 0.5 - span_frac / 2.0, 0.5 + span_frac / 2.0
    if n > 1:
        ts = ctx.rng.uniform(lo, hi, n)
    else:
        ts = np.array([ctx.rng.uniform(lo, hi)])
    #: Keep the block away from a wall opening it would otherwise plug: the
    #: generator can be told to avoid a band of the edge (``avoid_span``).
    avoid = spec.params.get("avoid_span")
    if avoid is not None:
        a_lo, a_hi = float(avoid[0]), float(avoid[1])
        for _ in range(8):
            clash = (ts >= a_lo) & (ts <= a_hi)
            if not clash.any():
                break
            ts[clash] = ctx.rng.uniform(lo, hi, int(clash.sum()))
    offs = np.empty((n, 2), dtype=np.float64)
    angs = np.empty(n, dtype=np.float64)
    hws = np.empty(n, dtype=np.float64)
    hhs = np.empty(n, dtype=np.float64)
    rots = np.empty(n, dtype=np.float64)
    for i, t in enumerate(ts):
        px, py, nx, ny = _edge_point(region, ctx, float(t), outward=0.5 * half)
        inward_angle = math.atan2(ny, nx)
        if explicit is None:
            theta = inward_angle
            # a small deterministic fan so a band of blocks is not perfectly parallel
            theta += (i - (n - 1) / 2.0) * 0.04
        else:
            theta = float(explicit) * _DEG
        # never emit a block that runs out of the field
        if nx * math.cos(theta) + ny * math.sin(theta) < 0.1:
            theta = inward_angle
        offs[i] = (px, py)
        angs[i] = theta
        hws[i] = half
        hhs[i] = across
        rots[i] = theta
    spin = np.full(n, float(spec.angular_velocity))
    return offs, angs, np.full(n, float(spec.speed)), hws, hhs, rots, spin, False


def _layout_wall_gap(spec, ctx, elapsed, step):
    """A wall spanning the field with one opening; the opening may sweep."""
    region = str(spec.params.get("spawn", "top"))
    x, y, inward = _region_anchor(region, ctx)
    theta = float(spec.params.get("entry_angle", np.degrees(inward))) * _DEG
    axis = "x" if abs(math.cos(theta)) < 0.5 else "y"   # span across the other axis
    span = ctx.field_w if axis == "x" else ctx.field_h
    thickness = float(spec.params.get("thickness", 26.0))
    gap_w = float(spec.params.get("gap_width", 90.0))
    gap_pos = float(spec.params.get("gap_position", 0.5))
    motion = str(spec.params.get("gap_motion", "static"))
    jitter = float(spec.params.get("gap_jitter", 0.0))
    if motion == "sweep":
        amp = float(spec.params.get("gap_sweep", 0.35))
        speed = float(spec.params.get("gap_speed", 0.6))
        gap_pos = 0.5 + amp * math.sin(2.0 * math.pi * speed * elapsed)
        gap_pos += ctx.rng.uniform(-jitter, jitter)
    elif motion == "random":
        gap_pos = float(ctx.rng.uniform(0.2, 0.8))
    gap_pos = float(np.clip(gap_pos, 0.0, 1.0))
    half_gap = min(gap_w / 2.0, span / 2.0)
    centre = float(np.clip(gap_pos * span, half_gap, span - half_gap))
    segs: list[tuple[float, float]] = []
    if centre - half_gap > 1.0:
        segs.append((0.0, centre - half_gap))
    if centre + half_gap < span - 1.0:
        segs.append((centre + half_gap, span))
    offs, hws, hhs = [], [], []
    for a, b in segs:
        mid = 0.5 * (a + b)
        seg_half = 0.5 * (b - a)
        if axis == "x":
            offs.append([mid, y])
            hws.append(seg_half)
            hhs.append(thickness / 2)
        else:
            offs.append([x, mid])
            hws.append(thickness / 2)
            hhs.append(seg_half)
    n = len(offs)
    if n == 0:
        empty = np.zeros((0, 2))
        return (empty, np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0),
                np.zeros(0), np.zeros(0), False)
    # the wall travels inward, with its long axis perpendicular to the travel
    rot = 0.0 if axis == "x" else _PI / 2
    return (
        np.array(offs, dtype=np.float64),
        np.full(n, theta),
        np.full(n, float(spec.speed)),
        np.array(hws, dtype=np.float64),
        np.array(hhs, dtype=np.float64),
        np.full(n, rot),
        np.full(n, float(spec.angular_velocity)),
        False,
    )


def _layout_small(spec, ctx, elapsed, step):
    """Small circles entering from the edges, each with its own entry angle."""
    n = max(1, _count(spec))
    regions = spec.params.get("regions", ["left", "right", "top", "bottom"])
    m = len(regions)
    idx = ctx.rng.integers(0, m, n)
    offsets = np.empty((n, 2), dtype=np.float64)
    angles = np.empty(n, dtype=np.float64)
    jitter = float(spec.params.get("angle_jitter", 25.0)) * _DEG
    for i in range(n):
        region = str(regions[int(idx[i])])
        t = ctx.rng.uniform(0.05, 0.95)
        px, py, nx, ny = _edge_point(region, ctx, t, outward=1.0)
        offsets[i] = (px, py)
        angles[i] = math.atan2(ny, nx) + ctx.rng.uniform(-jitter, jitter)
    speeds = _speeds(spec, ctx, n)
    size = float(spec.params.get("size", spec.radius))
    return (
        offsets,
        angles,
        speeds,
        np.full(n, size),
        np.full(n, size),
        np.zeros(n),
        np.full(n, float(spec.angular_velocity)),
        False,
    )


def _layout_cross(spec, ctx, elapsed, step):
    """Obstacles crossing the field from several sides at once."""
    n = max(1, _count(spec))
    sides = list(spec.params.get("sides", ["left", "right", "top", "bottom"]))
    per = max(1, n // max(1, len(sides)))
    offs, angs = [], []
    for side in sides:
        for t in np.linspace(0.12, 0.88, per):
            px, py, nx, ny = _edge_point(str(side), ctx, float(t), outward=1.0)
            offs.append((px, py))
            angs.append(math.atan2(ny, nx))
    offs = np.array(offs, dtype=np.float64)
    angs = np.array(angs, dtype=np.float64)
    m = offs.shape[0]
    size = float(spec.params.get("size", spec.radius))
    return (offs, angs, _speeds(spec, ctx, m), np.full(m, size), np.full(m, size),
            np.zeros(m), np.full(m, float(spec.angular_velocity)), False)


def _layout_corridor(spec, ctx, elapsed, step):
    """Two walls **symmetric about the player**, whose centres never translate.

    Design (per the specification):

    * the player is placed at the channel centre and the two walls are mirrored
      about it, so the episode never starts with the player inside a wall;
    * the walls' **centre coordinates stay put** for the rotational motions;
    * the road changes in one of three ways, spread over the whole episode as a
      constant rate so it cannot run away:
      - ``open`` / ``close`` - the walls translate along the channel normal,
        symmetrically, so the width grows / shrinks;
      - ``rotate_same`` - both walls spin the same way, tilting the channel
        while keeping its width exactly constant (a parallelogram);
      - ``rotate_opposite`` - the walls scissor; the total angle has already
        been clamped by ``resolve_spec`` so the *narrowest* point of the
        scissor is still wider than the player circle;
    * ``walls=1`` places a single barrier, for the "one wall is enough" case;
    * a width can never fall below ``CORRIDOR_GAP_MARGIN`` x the player diameter,
      because the per-episode change is clamped before it ever reaches the
      integrator.
    """
    params = spec.params
    cx, cy = float(ctx.player_xy[0]), float(ctx.player_xy[1])
    region = str(params.get("spawn", "left"))
    _, _, inward = _region_anchor(region, ctx)
    tilt = float(params.get("tilt_deg", 0.0)) * _DEG
    explicit = params.get("entry_angle")
    theta = (float(explicit) * _DEG if explicit is not None else inward) + tilt
    d = np.array([math.cos(theta), math.sin(theta)], dtype=np.float64)
    nrm = np.array([-d[1], d[0]], dtype=np.float64)

    width = float(params.get("corridor_width", 140.0))
    wall_t = float(params.get("wall_thickness", 20.0))
    wall_len = float(params.get("wall_length", 6.0 * 2.0 * 20.0))
    walls = int(params.get("walls", 2))
    motion = str(params.get("motion", "static"))
    change = float(params.get("change", 0.0))
    duration = max(float(getattr(ctx, "duration", 1.0) or 1.0), 1e-6)
    # the true player diameter is carried on the spec by ``resolve_spec``
    player_d = float(params.get("player_diameter", 20.0))

    sides = (1.0,) if walls == 1 else (-1.0, 1.0)
    offs, spins = [], []
    for side in sides:
        offs.append(np.array([cx, cy]) + nrm * (side * (width / 2.0 + wall_t / 2.0)))
        if motion == "rotate_same":
            spins.append(math.radians(change) / duration)
        elif motion == "rotate_opposite":
            spins.append(side * math.radians(change) / duration)
        else:
            spins.append(0.0)
    n = len(offs)
    if motion in ("open", "close"):
        sign = 1.0 if motion == "open" else -1.0
        # each wall moves half of the width change, in opposite directions
        speed = 0.5 * change * player_d / duration
        vel = [nrm * (side * sign * speed) for side in sides]
        speeds = [float(np.hypot(v[0], v[1])) for v in vel]
        angles = [math.atan2(v[1], v[0]) if s > 0 else 0.0 for v, s in zip(vel, speeds)]
    else:
        speeds = [0.0] * n
        angles = [0.0] * n
    return (
        np.array(offs, dtype=np.float64),
        np.array(angles, dtype=np.float64),
        np.array(speeds, dtype=np.float64),
        np.full(n, wall_len / 2.0),
        np.full(n, wall_t / 2.0),
        np.full(n, theta),
        np.array(spins, dtype=np.float64),
        False,
    )


#: layout name -> builder
_LAYOUT_BUILDERS = {
    "block": _layout_block,
    "wall_gap": _layout_wall_gap,
    "small": _layout_small,
    "cross": _layout_cross,
    "corridor": _layout_corridor,
}

#: All supported obstacle layouts.
LAYOUTS: tuple[str, ...] = tuple(_LAYOUT_BUILDERS)


GeneratorFn = Callable[[PatternSpec, GenerationContext], list[SpawnEvent]]

#: The environment has exactly one generator: realistic dynamic obstacles.
#: Decorative centre-fired patterns were removed on purpose.
_GENERATORS: dict[str, GeneratorFn] = {
    "obstacle": gen_obstacle_field,
}

#: Kinds that emit continuously (repeat until the scenario ends).
CONTINUOUS_KINDS = ("obstacle",)


def register_generator(kind: str, fn: GeneratorFn, *, override: bool = False) -> None:
    """Extend the pattern vocabulary without touching the core."""
    if kind in _GENERATORS and not override:
        raise ScenarioError(f"generator {kind!r} already registered")
    _GENERATORS[kind] = fn
    PATTERN_TYPE_IDS.setdefault(kind, len(PATTERN_TYPE_IDS))


def available_patterns() -> list[str]:
    return sorted(_GENERATORS)


def generator_for(kind: str) -> GeneratorFn:
    try:
        return _GENERATORS[kind]
    except KeyError as exc:
        raise ScenarioError(
            f"unknown pattern kind {kind!r}; available: {available_patterns()}"
        ) from exc


def generate_pattern(spec: PatternSpec, ctx: GenerationContext) -> list[SpawnEvent]:
    """Dispatch a single (non-composite) spec to its generator."""
    return generator_for(spec.kind)(spec, ctx)


__all__ = [
    "gen_obstacle_field",
    "LAYOUTS",
    "SPAWN_REGIONS",
    "generator_for",
    "generate_pattern",
    "register_generator",
    "available_patterns",
    "emission_steps",
    "CONTINUOUS_KINDS",
]
