"""Parameterised pattern specification.

A bullet pattern is **data**, never code.  Every knob the brief asks for lives
here: count, initial position, initial velocity, launch angle, angular
velocity, acceleration, lifetime, radius, spawn time and random perturbation.

Angle convention
----------------
* User-facing angles are **degrees**, ``0`` = ``+x``, counter-clockwise.
* ``angle``          base direction (or *bias* added on top of an aim direction)
* ``angle_spread``   total fan width in degrees; ``360`` means a full ring
* ``angular_velocity`` deg/s used by emitters whose base angle rotates over
  time (spiral) or by rings that rotate per emission.

Randomness
----------
``speed_var`` / ``angle_var`` / ``jitter_*`` / ``radius_var`` consume the
scenario's *build-time* RNG, so a given ``(spec, seed)`` always expands to the
same :class:`~bullet_sim.generators.burst.SpawnEvent` list.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Sequence


from bullet_sim.core.errors import ConfigError

ORIGIN_KEYWORDS: tuple[str, ...] = (
    #: "the generator itself decides" - the dynamic-obstacle layouts always
    #: place their object on a field boundary, so they use this instead of
    #: pretending to be a centre emitter.  It is now the default.
    "field",
    "player",
    "top",
    "bottom",
    "left",
    "right",
)

#: Stable integer ids used by datasets / hardware protocol to tag objects.
#: Only realistic dynamic obstacles exist now - decorative centre-fired
#: patterns (radial/spiral/aimed/burst/line/wall/random/single) were removed.
PATTERN_TYPE_IDS: dict[str, int] = {
    "obstacle": 9,
}

PATTERN_KINDS: tuple[str, ...] = tuple(PATTERN_TYPE_IDS)

#: Natural object count per pattern, used when ``count is None``.
PATTERN_DEFAULT_COUNT: dict[str, int] = {
    "obstacle": 12,
}


@dataclass
class PatternSpec:
    """Declarative description of one bullet emitter."""

    kind: str = "radial"
    #: Bullets per emission.  ``None`` means "the pattern's natural default"
    #: (see :data:`PATTERN_DEFAULT_COUNT`) - a ``single`` shot really is one
    #: bullet, a wall spans the field, a spiral is driven by ``params['arms']``.
    count: int | None = None
    start_time: float = 0.0
    interval: float = 0.0
    repetitions: int = 1

    #: Where the pattern is born.  ``"field"`` means "the generator decides" and
    #: is the default: the dynamic-obstacle layouts place objects on a boundary.
    origin: Any = "field"
    speed: float = 120.0
    speed_var: float = 0.0
    angle: float = -90.0
    angle_var: float = 0.0
    angle_spread: float = 360.0
    angular_velocity: float = 0.0

    accel: float = 0.0
    accel_direction: float | None = None
    ttl: float = 20.0
    radius: float = 3.0
    radius_var: float = 0.0

    aim: bool = False
    jitter_pos: float = 0.0
    jitter_vel: float = 0.0

    # --- DynamicObstacle geometry (see entities/bullet.py, protocol v3) ---
    #: ``"circle"`` or ``"rect"``; a wall/corridor/block is a rectangle.
    shape: str = "circle"
    #: Rectangle half extents in world units.  ``None`` means "use radius",
    #: which keeps a circular obstacle's bounding box consistent.
    half_w: float | None = None
    half_h: float | None = None
    #: Body rotation in radians (static for now; velocity rotation is
    #: ``angular_velocity`` above).
    rotation: float = 0.0
    #: Obstacles closer than this to the live player at spawn time are dropped.
    #: This is what makes "never materialise on top of the agent" enforceable
    #: rather than a convention.
    min_player_distance: float = 0.0
    type_id: int | None = None
    group_id: int = 0

    #: Pattern-specific knobs: e.g. ``arms`` (spiral), ``spacing`` (line),
    #: ``span`` (wall), ``edge`` (random), ``rings`` (burst).
    params: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.kind not in PATTERN_TYPE_IDS:
            raise ConfigError(
                f"unknown pattern kind {self.kind!r}; expected one of {list(PATTERN_TYPE_IDS)}"
            )
        if self.count is not None and self.count < 0:
            raise ConfigError(f"pattern count must be >= 0, got {self.count}")
        if self.interval < 0.0:
            raise ConfigError(f"pattern interval must be >= 0, got {self.interval}")
        if self.start_time < 0.0:
            raise ConfigError(f"pattern start_time must be >= 0, got {self.start_time}")
        if isinstance(self.origin, (list, tuple)) and len(self.origin) != 2:
            raise ConfigError("explicit origin must be a 2-element (x, y) sequence")
        if isinstance(self.origin, str) and self.origin not in ORIGIN_KEYWORDS:
            raise ConfigError(
                f"unknown origin {self.origin!r}; expected a keyword {list(ORIGIN_KEYWORDS)} "
                "or an (x, y) pair"
            )
        from bullet_sim.entities.bullet import shape_code

        shape_code(self.shape)  # validates
        if self.type_id is None:
            self.type_id = PATTERN_TYPE_IDS[self.kind]

    # ------------------------------------------------------------------
    @property
    def resolved_count(self) -> int:
        """``count`` with the pattern's natural default substituted."""
        if self.count is not None:
            return int(self.count)
        return int(PATTERN_DEFAULT_COUNT.get(self.kind, 0))

    @property
    def resolves_at_runtime(self) -> bool:
        """True when the emitter needs the *current* player position at spawn."""
        return self.aim or self.origin == "player"

    def is_repeating(self) -> bool:
        return self.interval > 0.0 and self.repetitions != 1

    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "count": self.resolved_count,
            "start_time": self.start_time,
            "interval": self.interval,
            "repetitions": self.repetitions,
            "origin": list(self.origin) if isinstance(self.origin, (list, tuple)) else self.origin,
            "speed": self.speed,
            "speed_var": self.speed_var,
            "angle": self.angle,
            "angle_var": self.angle_var,
            "angle_spread": self.angle_spread,
            "angular_velocity": self.angular_velocity,
            "accel": self.accel,
            "accel_direction": self.accel_direction,
            "ttl": self.ttl,
            "radius": self.radius,
            "radius_var": self.radius_var,
            "aim": self.aim,
            "jitter_pos": self.jitter_pos,
            "jitter_vel": self.jitter_vel,
            "shape": self.shape,
            "half_w": self.half_w,
            "half_h": self.half_h,
            "rotation": self.rotation,
            "min_player_distance": self.min_player_distance,
            "type_id": self.type_id,
            "group_id": self.group_id,
            "params": dict(self.params),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "PatternSpec":
        known = {f for f in cls.__dataclass_fields__}  # type: ignore[attr-defined]
        kwargs = {k: v for k, v in payload.items() if k in known}
        if "params" in kwargs and kwargs["params"] is not None:
            kwargs["params"] = dict(kwargs["params"])
        return cls(**kwargs)

    def replaced(self, **changes: Any) -> "PatternSpec":
        return replace(self, **changes)

    def describe(self) -> str:
        return (
            f"{self.kind}(count={self.resolved_count}, speed={self.speed}, angle={self.angle}, "
            f"spread={self.angle_spread}, t={self.start_time}, every={self.interval})"
        )


def pattern_from(value: Any) -> PatternSpec:
    """Coerce a name / mapping / PatternSpec into a :class:`PatternSpec`.

    ``pattern_from("radial")`` is the shorthand used by
    :meth:`World.spawn_pattern` so a caller can fire a one-off pattern without
    building a full spec object.
    """
    if isinstance(value, PatternSpec):
        return value
    if isinstance(value, str):
        return PatternSpec(kind=value)
    if isinstance(value, Mapping):
        return PatternSpec.from_dict(value)
    raise ConfigError(f"cannot build a PatternSpec from {value!r}")


def patterns_from(values: Sequence[Any]) -> list[PatternSpec]:
    return [pattern_from(v) for v in values]


def lerp(a: float, b: float, t: float) -> float:
    return float(a) + (float(b) - float(a)) * float(t)


__all__ = [
    "PatternSpec",
    "PATTERN_DEFAULT_COUNT",
    "PATTERN_KINDS",
    "PATTERN_TYPE_IDS",
    "ORIGIN_KEYWORDS",
    "pattern_from",
    "patterns_from",
    "lerp",
]
