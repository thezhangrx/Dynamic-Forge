"""Obstacle type system: what a dynamic obstacle *is*, and what it stands for.

A "scenario type" in this platform is **a kind of real-world dynamic obstacle**,
not a difficulty label and not a decorative bullet pattern.  Every type must
document, in code and in the README:

* what it is;
* which real situation it simulates;
* how it moves, and why that motion is physically reasonable;
* what the player has to solve;
* which safety check is the relevant one for it.

That documentation lives in :class:`ObstacleType` so it cannot drift from the
generator: the type *is* the config, and `README`/`describe()` are derived from it.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Sequence

import math

import numpy as np

from bullet_sim.core.errors import ConfigError
from bullet_sim.generators.spec import PatternSpec

#: Characteristic obstacle size presets, expressed as a multiple of the
#: **player body diameter**.  Using a ratio (not pixels) is what keeps the
#: player/obstacle proportion meaningful across field sizes and for the future
#: real cart, where "2x my own width" is the transferable statement.
SIZE_PRESETS: dict[str, float] = {
    "tiny": 0.15,
    "small": 0.35,
    "medium": 0.8,
    "large": 2.0,
    "huge": 4.5,
}

#: Safety strategy names; each obstacle type names the one that fits it.
#: How a corridor may change over the episode; the change is spread over the
#: whole scenario as a constant rate, so it can never "run away".
CORRIDOR_MOTIONS: tuple[str, ...] = (
    "static",             # the channel never changes
    "open",               # the two walls move apart (width grows)
    "close",              # the two walls move together (width shrinks)
    "rotate_same",        # both walls rotate the same way (channel tilts)
    "rotate_opposite",    # the walls scissor in opposite directions
)

#: The smallest channel width ever allowed, as a multiple of the player diameter.
CORRIDOR_GAP_MARGIN = 1.25

#: Cap on a same-direction rotation (a tilted channel keeps its width).
CORRIDOR_MAX_SAME_ANGLE = 45.0

#: How far past the field edge a default corridor wall extends, so the road
#: reads as continuous rather than as two short blocks.
CORRIDOR_SPAN_FACTOR = 1.1

#: Safety strategy names; each obstacle type names the one that fits it.
SAFETY_STRATEGIES: tuple[str, ...] = (
    "bypass",      # is there room to go around the moving block at all?
    "gap",         # is the gap wide enough *and* reachable in time?
    "local_free",  # is there a connected corridor of free space?
    "corridor",    # does the channel stay open long enough to pass?
    "generic",     # full time-windowed free-space + path search
)


@dataclass(frozen=True)
class ObstacleType:
    """Immutable description of one real-world dynamic obstacle class."""

    key: str
    label: str
    label_zh: str
    simulates: str
    motion: str
    player_problem: str
    layout: str
    safety_strategy: str = "generic"
    shape: str = "circle"
    defaults: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "label_zh": self.label_zh,
            "simulates": self.simulates,
            "motion": self.motion,
            "player_problem": self.player_problem,
            "layout": self.layout,
            "shape": self.shape,
            "safety_strategy": self.safety_strategy,
            "defaults": dict(self.defaults),
        }

    def describe(self) -> str:
        return (
            f"{self.label_zh} ({self.key})\n"
            f"  模拟现实 : {self.simulates}\n"
            f"  运动方式 : {self.motion}\n"
            f"  玩家任务 : {self.player_problem}\n"
            f"  安全校验 : {self.safety_strategy}"
        )


@dataclass
class ObstacleSpawn:
    """One configured instance of an :class:`ObstacleType` inside a scenario.

    Parameters are all in **world units / seconds**, except ``size`` which is a
    multiple of the player body diameter (see :data:`SIZE_PRESETS`).
    """

    type_key: str
    count: int | None = None
    size: float | None = None
    speed: float | None = None
    interval: float | None = None
    start_time: float | None = None
    repetitions: int | None = None
    ttl: float | None = None
    region: str | None = None
    gap_width: float | None = None          # world units; >= player diameter
    gap_position: float | None = None
    gap_motion: str | None = None
    gap_speed: float | None = None
    corridor_width: float | None = None     # world units; >= player diameter
    corridor_min_width: float | None = None  # gap floor in world units
    wall_length: float | None = None        # corridor wall length in world units
                                              # (None = extend across the field)
    walls: int | None = None                # 1 = single barrier, 2 = symmetric channel
    motion: str | None = None               # see CORRIDOR_MOTIONS
    change: float | None = None             # width delta (player diameters) or degrees
    tilt_deg: float | None = None           # initial corridor inclination (degrees)
    entry_span: float | None = None         # fraction of the edge a block may use
    spawn_distance: float | None = None
    entry_angle: float | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------------
    def overrides(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for name in (
            "count", "gap_width", "gap_position", "gap_motion", "gap_speed",
            "corridor_width", "corridor_min_width", "wall_length", "walls",
            "motion", "change", "tilt_deg", "entry_span", "spawn_distance",
            "entry_angle",
        ):
            value = getattr(self, name)
            if value is not None:
                out[name] = value
        if self.region is not None:
            out["spawn"] = self.region
        out.update(self.extra)
        return out

    def to_dict(self) -> dict[str, Any]:
        data = {
            "type": self.type_key,
            "count": self.count,
            "size": self.size,
            "speed": self.speed,
            "interval": self.interval,
            "start_time": self.start_time,
            "repetitions": self.repetitions,
            "ttl": self.ttl,
            "region": self.region,
            "gap_width": self.gap_width,
            "gap_position": self.gap_position,
            "gap_motion": self.gap_motion,
            "gap_speed": self.gap_speed,
            "corridor_width": self.corridor_width,
            "corridor_min_width": self.corridor_min_width,
            "wall_length": self.wall_length,
            "walls": self.walls,
            "motion": self.motion,
            "change": self.change,
            "tilt_deg": self.tilt_deg,
            "entry_span": self.entry_span,
            "spawn_distance": self.spawn_distance,
            "entry_angle": self.entry_angle,
            "extra": dict(self.extra),
        }
        return data

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ObstacleSpawn":
        data = dict(payload)
        data["type_key"] = data.pop("type", data.pop("type_key", "moving_block"))
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown obstacle spawn fields: {sorted(unknown)}")
        return cls(**data)

    @classmethod
    def from_value(cls, value: Any) -> "ObstacleSpawn":
        """Accept ``"moving_block"``, a mapping, or an instance."""
        if isinstance(value, ObstacleSpawn):
            return value
        if isinstance(value, str):
            return cls(type_key=value)
        if isinstance(value, Mapping):
            return cls.from_dict(value)
        raise ConfigError(f"cannot build an ObstacleSpawn from {value!r}")


def _corridor_axis_angle(
    params: Mapping[str, Any], spawn: "ObstacleSpawn", field_w: float, field_h: float
) -> float:
    """Channel axis direction, used to decide which field extent the walls span."""
    region = str(params.get("spawn", "left"))
    table = {
        "left": 0.0, "right": math.pi,
        "bottom": math.pi / 2.0, "top": -math.pi / 2.0,
    }
    inward = table.get(region, 0.0)
    if region == "farthest_side":
        inward = table["left"]
    if region == "nearest_side":
        inward = table["right"]
    explicit = spawn.entry_angle
    base = float(explicit) * math.pi / 180.0 if explicit is not None else inward
    return base + float(params.get("tilt_deg", 0.0)) * math.pi / 180.0


def resolve_spec(
    type_spec: ObstacleType,
    spawn: ObstacleSpawn,
    *,
    player_radius: float,
    field_w: float,
    field_h: float,
    group_id: int = 0,
) -> PatternSpec:
    """Turn a type + its configuration into a concrete :class:`PatternSpec`.

    This is the only place where "an obstacle type" becomes "a generator call",
    so the player/obstacle proportion and the boundary-spawn rule are applied
    uniformly to every type.
    """
    params = dict(type_spec.defaults)
    params.update(spawn.overrides())
    params["layout"] = type_spec.layout

    # The player is a circle, so every size is quoted as a multiple of the
    # player *diameter* ("twice my own width" transfers to the real cart).  The
    # generators work in world units, so convert exactly once, here, and never
    # let the raw ratio leak into ``params`` (it would be read as pixels).
    raw_size = spawn.size if spawn.size is not None else type_spec.defaults.get("size", 0.8)
    ratio = float(raw_size)
    params.pop("size", None)
    player_d = 2.0 * float(player_radius)
    characteristic = max(1.0, ratio * player_d)
    half = max(0.5, characteristic / 2.0)
    span = float(field_w if type_spec.layout in ("wall_gap", "corridor") else min(field_w, field_h))
    # a wall must span the field; a block is sized relative to the player circle
    if type_spec.layout == "wall_gap":
        params.setdefault("travel", span)
        params.setdefault("thickness", max(8.0, 0.35 * characteristic))
    if type_spec.layout == "corridor":
        params.setdefault("corridor_width", 3.0 * player_d)
        params.setdefault("wall_thickness", max(8.0, 0.4 * characteristic))
    if type_spec.layout in ("small", "cross"):
        # circle / square half-extent in world units = half the characteristic size
        params["size"] = half
    params.setdefault("min_player_distance", max(1.5 * player_d, 0.25 * min(field_w, field_h)))

    # A gap/corridor must admit the player circle - never a sealed opening.
    # ``gap_width`` / ``corridor_width`` are always world units; the catalogue
    # defaults are ratios of the player diameter and are resolved here.
    min_opening = player_d
    if type_spec.layout == "wall_gap":
        explicit = spawn.gap_width
        gap = float(explicit) if explicit is not None else float(
            params.pop("gap_ratio", 3.0)
        ) * player_d
        if explicit is not None and float(explicit) < min_opening:
            raise ConfigError(
                f"{type_spec.key}: gap_width={explicit} is smaller than the player "
                f"circle diameter ({min_opening:.2f}); the opening must admit the player"
            )
        params["gap_width"] = max(gap, min_opening)
    if type_spec.layout == "corridor":
        explicit = spawn.corridor_width
        width = float(explicit) if explicit is not None else float(
            params.pop("corridor_ratio", 4.0)
        ) * player_d
        if explicit is not None and float(explicit) < min_opening:
            raise ConfigError(
                f"{type_spec.key}: corridor_width={explicit} is smaller than the player "
                f"circle diameter ({min_opening:.2f}); the corridor must admit the player"
            )
        width = max(width, min_opening)

        # The gap floor: explicit ``corridor_min_width`` (world units) or the
        # catalogue ratio, and never below the hard safety floor.  The raw key
        # is consumed here so it never lingers in the generator params.
        floor_explicit = spawn.corridor_min_width
        params.pop("corridor_min_width", None)
        floor = (
            float(floor_explicit)
            if floor_explicit is not None
            else float(params.pop("corridor_min_ratio", CORRIDOR_GAP_MARGIN)) * player_d
        )
        hard_floor = min_opening * CORRIDOR_GAP_MARGIN
        if floor_explicit is not None and float(floor_explicit) < hard_floor:
            raise ConfigError(
                f"{type_spec.key}: corridor_min_width={floor_explicit} is below the "
                f"hard safety floor ({hard_floor:.2f} = {CORRIDOR_GAP_MARGIN} x the "
                "player diameter); the corridor must never be able to crush the player"
            )
        min_gap = max(floor, hard_floor)

        # Walls extend across the whole field by default: the corridor should
        # read as a road, not as two short blocks.  ``wall_length`` (world
        # units) or ``wall_length_ratio`` (x player diameter) override it.
        ratio = params.pop("wall_length_ratio", None)
        if spawn.wall_length is not None:
            wall_len = float(spawn.wall_length)
        elif ratio is not None:
            wall_len = float(ratio) * player_d
        else:
            axis_theta = _corridor_axis_angle(params, spawn, field_w, field_h)
            wall_len = float(field_w if abs(math.cos(axis_theta)) >= abs(math.sin(axis_theta))
                             else field_h) * CORRIDOR_SPAN_FACTOR

        walls = int(spawn.walls if spawn.walls is not None else params.pop("walls", 2))
        if walls not in (1, 2):
            raise ConfigError(f"{type_spec.key}: walls must be 1 or 2, got {walls}")
        motion = str(
            spawn.motion if spawn.motion is not None else params.pop("motion", "static")
        )
        if motion not in CORRIDOR_MOTIONS:
            raise ConfigError(
                f"{type_spec.key}: unknown corridor motion {motion!r}; "
                f"expected {sorted(CORRIDOR_MOTIONS)}"
            )
        change = float(
            spawn.change if spawn.change is not None else params.pop("change", 0.0)
        )
        # Clamp the requested change so the channel can never crush the player.
        if motion == "close":
            change = float(np.clip(change, 0.0, max(0.0, (width - min_gap) / player_d)))
        elif motion == "open":
            room = max(0.0, 0.45 * min(field_w, field_h) - width)
            change = float(np.clip(change, 0.0, room / player_d))
        elif motion == "rotate_same":
            change = float(np.clip(change, 0.0, CORRIDOR_MAX_SAME_ANGLE))
        elif motion == "rotate_opposite":
            # two walls of half-length a pivoting about their centres: the gap
            # at the wall ends is w - 2a*tan(theta), so bound tan(theta).
            a = max(0.5 * wall_len, 1e-9)
            max_tan = max(0.0, (width - min_gap)) / (2.0 * a)
            change = float(np.clip(change, 0.0, math.degrees(math.atan(max_tan))))
        else:  # static
            change = 0.0
        params["corridor_width"] = width
        params["wall_length"] = wall_len
        params["walls"] = walls
        params["motion"] = motion
        params["change"] = change
        params["tilt_deg"] = float(params.get("tilt_deg", 0.0))
        # the walls are *meant* to sit right beside the player
        params["min_player_distance"] = 0.0
        params["player_diameter"] = player_d

    speed = float(spawn.speed if spawn.speed is not None else 90.0)
    interval = float(spawn.interval if spawn.interval is not None else 1.6)
    repetitions = spawn.repetitions if spawn.repetitions is not None else 0
    ttl = float(spawn.ttl if spawn.ttl is not None else 14.0)
    if type_spec.layout == "corridor":
        #: A corridor is **one** persistent pair of walls that changes shape over
        #: the episode.  Re-emitting it every ``interval`` would stack several
        #: pairs on top of each other, each with its own rotation phase, so it
        #: emits exactly once and lives until the scenario ends.
        if spawn.repetitions is None:
            repetitions = 1
        if spawn.ttl is None:
            ttl = float("inf")

    #: A circle keeps ``half_w == half_h == radius`` (the platform-wide
    #: convention) so its collision radius really is half the characteristic
    #: size - i.e. ``radius = size_ratio * player_diameter / 2``.  For rectangles
    #: ``radius`` is unused by the obstacle SDF and stays as a legacy fallback.
    radius = half if type_spec.shape == "circle" else half * 0.5

    return PatternSpec(
        kind="obstacle",
        count=spawn.count if spawn.count is not None else (1 if type_spec.shape == "rect" else 12),
        #: The world already contains obstacles at t=0.  They are born on the
        #: boundary, so this is not an "instant hit on step 0".
        start_time=float(spawn.start_time if spawn.start_time is not None else 0.0),
        interval=interval,
        repetitions=repetitions,
        origin="field",           # obstacles are born on the boundary, never at the centre
        speed=speed,
        angle=float(spawn.entry_angle) if spawn.entry_angle is not None else 0.0,
        ttl=ttl,
        radius=radius,
        shape=type_spec.shape,
        half_w=half,
        half_h=half if type_spec.shape == "rect" else half,
        group_id=group_id,
        type_id=None,
        params=params,
        min_player_distance=float(params.get("min_player_distance", 0.0)),
    )


__all__ = [
    "ObstacleType",
    "ObstacleSpawn",
    "SIZE_PRESETS",
    "SAFETY_STRATEGIES",
    "resolve_spec",
]
