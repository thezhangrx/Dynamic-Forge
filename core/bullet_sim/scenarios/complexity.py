"""Continuous difficulty model.

``complexity`` is a single scalar in ``[0, 1]`` from which *every* difficulty
knob is derived: bullet count, speed, direction change, acceleration, pattern
variety and scenario duration.  Presets (easy/medium/hard/extreme) are just
points on this continuum, which is what lets the project study

    scene complexity -> computational budget -> prediction quality
                     -> decision quality  -> real-time performance

without ever hand-tuning two different scene generators.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

from bullet_sim.core.errors import ConfigError

#: Canonical bullet-count ladder from the specification (stress-test levels):
#: 100 / 500 / 1000 / 2000 / 5000 / 10000 simultaneously-live dynamic objects.
BULLET_COUNT_LADDER: tuple[int, ...] = (100, 500, 1000, 2000, 5000, 10000)

#: Named levels and the complexity value they sit at.
LEVEL_VALUES: dict[str, float] = {
    "trivial": 0.0,
    "easy": 0.15,
    "medium": 0.40,
    "hard": 0.70,
    "extreme": 1.00,
}

#: Inclusive lower bounds for the qualitative label of a complexity value.
_LABEL_BOUNDS: tuple[tuple[float, str], ...] = (
    (0.25, "easy"),
    (0.55, "medium"),
    (0.80, "hard"),
    (1.01, "extreme"),
)


@dataclass(frozen=True)
class ComplexityProfile:
    """Fully-resolved difficulty values for one complexity score."""

    value: float
    label: str
    bullet_count: int
    #: Master control multipliers: every level scales *both* how many obstacles
    #: there are and how fast they move, monotonically with complexity.
    count_scale: float
    speed_scale: float
    accel_scale: float
    spread_scale: float
    direction_change: float
    #: Obstacle kinds this complexity unlocks (real dynamic obstacles only).
    obstacle_kinds: tuple[str, ...]
    duration: float

    def to_dict(self) -> dict:
        return {
            "value": self.value,
            "label": self.label,
            "bullet_count": self.bullet_count,
            "count_scale": self.count_scale,
            "speed_scale": self.speed_scale,
            "accel_scale": self.accel_scale,
            "spread_scale": self.spread_scale,
            "direction_change": self.direction_change,
            "obstacle_kinds": list(self.obstacle_kinds),
            "duration": self.duration,
        }


def label_for(value: float) -> str:
    for bound, name in _LABEL_BOUNDS:
        if value < bound:
            return name
    return "extreme"  # pragma: no cover - unreachable, kept for safety


def bullet_count_for(value: float, ladder: Iterable[int] = BULLET_COUNT_LADDER) -> int:
    """Log-spaced interpolation across the canonical bullet-count ladder."""
    counts = sorted(int(c) for c in ladder)
    v = min(max(float(value), 0.0), 1.0)
    if v <= 0.0:
        return counts[0]
    if v >= 1.0:
        return counts[-1]
    pos = v * (len(counts) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(counts) - 1)
    frac = pos - lo
    a, b = counts[lo], counts[hi]
    # geometric (log) interpolation keeps the ladder perceptually even
    return int(round(math.exp(math.log(a) * (1 - frac) + math.log(b) * frac)))


def complexity_from_bullet_count(count: int) -> float:
    """Inverse of :func:`bullet_count_for` (nearest ladder point)."""
    counts = list(BULLET_COUNT_LADDER)
    if count <= counts[0]:
        return 0.0
    if count >= counts[-1]:
        return 1.0
    for i in range(len(counts) - 1):
        a, b = counts[i], counts[i + 1]
        if a <= count <= b:
            frac = (math.log(count) - math.log(a)) / (math.log(b) - math.log(a))
            return (i + frac) / (len(counts) - 1)
    return 1.0  # pragma: no cover


#: Progressive obstacle vocabulary: a harder scene unlocks more kinds of *real*
#: dynamic obstacle.  Decorative centre-fired patterns no longer exist.
_OBSTACLE_LADDER: tuple[str, ...] = (
    "small_obstacles",
    "moving_block",
    "wall_with_gap",
    "cross_traffic",
    "corridor",
)


def obstacle_kinds_for(value: float) -> tuple[str, ...]:
    """Progressive obstacle vocabulary: richer scenes unlock more kinds."""
    v = min(max(float(value), 0.0), 1.0)
    n = max(1, int(round(1 + v * (len(_OBSTACLE_LADDER) - 1))))
    return _OBSTACLE_LADDER[:n]


def profile_for(
    value: float = 0.4,
    *,
    duration: float | None = None,
    bullet_count: int | None = None,
    speed_scale: float | None = None,
    accel_scale: float | None = None,
) -> ComplexityProfile:
    """Build a :class:`ComplexityProfile` from a complexity scalar.

    Any parameter may be overridden explicitly - that is how the "run exactly
    N bullets" and stress-test entry points work.
    """
    v = resolve_complexity(value)
    count = (
        int(bullet_count)
        if bullet_count is not None
        else bullet_count_for(v)
    )
    if count < 0:
        raise ConfigError(f"bullet_count must be >= 0, got {count}")
    dur = float(duration) if duration is not None else round(20.0 + 25.0 * v, 3)
    return ComplexityProfile(
        value=v,
        label=label_for(v),
        bullet_count=count,
        count_scale=0.35 + 1.15 * v,
        speed_scale=float(speed_scale) if speed_scale is not None else 0.65 + 0.85 * v,
        accel_scale=float(accel_scale) if accel_scale is not None else 0.4 + 1.6 * v,
        spread_scale=0.6 + 0.9 * v,
        direction_change=0.15 + 1.35 * v,
        obstacle_kinds=obstacle_kinds_for(v),
        duration=dur,
    )


#: The largest simultaneous-object target a *level* may use.  Measured: a scene
#: with ~1500 peak objects still leaves 21% free space, while ~2500 collapses to
#: 6.5% and becomes infeasible.  Levels therefore stop well below that ceiling -
#: "extreme" must still have a safe region - and ``stress(N)`` is the uncapped
#: path for raw performance work.
LEVEL_OBJECT_CAP: int = 400

#: Fraction of the cap used by the easiest level.
LEVEL_OBJECT_FLOOR: float = 0.20


def level_object_count(value: float) -> int:
    """Simultaneous-object target for a *level*: proportional, safety-capped.

    Grows monotonically (and roughly linearly) with complexity, so the level is
    an intuitive master control, while never exceeding the measured ceiling at
    which free space disappears.
    """
    v = min(max(float(value), 0.0), 1.0)
    frac = LEVEL_OBJECT_FLOOR + (1.0 - LEVEL_OBJECT_FLOOR) * v
    return max(20, int(round(LEVEL_OBJECT_CAP * frac)))


def safety_count_cap(
    field_w: float,
    field_h: float,
    player_radius: float,
    *,
    size_ratio: float = 0.35,
    max_cover: float = 0.35,
) -> int:
    """How many small obstacles still leave a safe region.

    A level preset must never generate a scene without free space, so the
    requested count is capped by a simple area budget: the small obstacles
    (radius = ``size_ratio`` x player diameter / 2) may cover at most
    ``max_cover`` of the field.  ``stress(N)`` bypasses this on purpose.
    """
    r = 0.5 * float(size_ratio) * 2.0 * float(player_radius)
    if r <= 0.0:
        return 0
    area = float(field_w) * float(field_h)
    return max(1, int(max_cover * area / (math.pi * r * r)))


def resolve_complexity(value: Any) -> float:
    """Accept a float in ``[0,1]``, a named level, or an ``int`` bullet count."""
    if isinstance(value, str):
        key = value.strip().lower()
        if key in LEVEL_VALUES:
            return LEVEL_VALUES[key]
        try:
            value = float(key)
        except ValueError as exc:
            raise ConfigError(
                f"unknown difficulty {value!r}; expected a level {sorted(LEVEL_VALUES)} "
                "or a number in [0, 1]"
            ) from exc
    f = float(value)
    if f < 0.0 or f > 1.0:
        raise ConfigError(f"complexity must lie in [0, 1] (or be a named level), got {f}")
    return f


__all__ = [
    "BULLET_COUNT_LADDER",
    "LEVEL_OBJECT_CAP",
    "level_object_count",
    "safety_count_cap",
    "LEVEL_VALUES",
    "ComplexityProfile",
    "profile_for",
    "obstacle_kinds_for",
    "bullet_count_for",
    "complexity_from_bullet_count",
    "resolve_complexity",
    "label_for",
]
