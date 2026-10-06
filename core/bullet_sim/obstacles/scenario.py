"""``ObstacleScenario``: a composable set of dynamic obstacle types.

    ObstacleScenario([moving_block, wall_with_gap, small_obstacles, corridor])

* run **one** type on its own;
* run **several** at once;
* keep the parameters of each type separately (count / speed / size / gap /
  region / timing);
* expand a ``mixed_obstacles`` entry into its constituent types.

The result converts to a :class:`~bullet_sim.scenarios.spec.ScenarioSpec`, so
everything downstream (reproducibility, dataset, benchmark, safety check) works
unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from bullet_sim.core.errors import ConfigError
from bullet_sim.core.rng import SeedManager
from bullet_sim.generators.spec import PatternSpec
from bullet_sim.obstacles.catalog import CATALOG, GENERATABLE, get_obstacle_type
from bullet_sim.obstacles.spec import (
    SIZE_PRESETS,
    ObstacleSpawn,
    ObstacleType,
    resolve_spec,
)

DEFAULT_FIELD_W = 640.0
DEFAULT_FIELD_H = 480.0
DEFAULT_DT = 1.0 / 120.0

#: Types that make a sensible default "mixed" scene.
@dataclass
class ObstacleScenario:
    """A named, seeded composition of dynamic obstacle types."""

    name: str = "obstacle_scenario"
    seed: int = 0
    duration: float = 30.0
    dt: float = DEFAULT_DT
    field_w: float = DEFAULT_FIELD_W
    field_h: float = DEFAULT_FIELD_H

    #: The player is a circle.  This radius is simultaneously the collision
    #: shape and the drawn shape, so visual size always equals hitbox size.
    player_hitbox_radius: float = 10.0
    player_speed: float = 200.0
    player_x: float | None = None
    player_y: float | None = None

    #: The obstacle composition.
    obstacles: list[ObstacleSpawn] = field(default_factory=list)

    #: Safety validation config (see bullet_sim.safety).
    safety_margin: float = 6.0
    safety_horizon: float = 2.0
    require_valid: bool = True

    target_x: float | None = None
    target_y: float | None = None
    meta: dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------------
    def __post_init__(self) -> None:
        self.obstacles = [ObstacleSpawn.from_value(v) for v in self.obstacles]
        for spawn in self.obstacles:
            if spawn.type_key not in GENERATABLE:
                raise ConfigError(
                    f"obstacle type {spawn.type_key!r} cannot be generated directly; "
                    f"available: {list(GENERATABLE)}"
                )
        if self.player_hitbox_radius <= 0:
            raise ConfigError("player_hitbox_radius must be > 0")
        # A gap must admit the player circle; refuse instead of silently
        # generating an impassable wall.
        diameter = 2.0 * float(self.player_hitbox_radius)
        for spawn in self.obstacles:
            if spawn.gap_width is not None and spawn.gap_width < diameter:
                raise ConfigError(
                    f"{spawn.type_key}: gap_width={spawn.gap_width} is smaller than "
                    f"the player circle diameter ({diameter}); the opening must "
                    "admit the player"
                )

    # ------------------------------------------------------------------
    @property
    def type_keys(self) -> list[str]:
        return [s.type_key for s in self.obstacles]

    def types(self) -> list[ObstacleType]:
        return [get_obstacle_type(s.type_key) for s in self.obstacles]

    def player_spawn(self) -> tuple[float, float]:
        """Where the player starts.

        A corridor scene puts the player **at the field centre**: the corridor
        generator mirrors its two walls about the player, so the episode begins
        with the player inside the channel.  Every other scene keeps the
        lower-middle spawn (boundary obstacles enter from far away).
        """
        x = self.field_w * 0.5 if self.player_x is None else float(self.player_x)
        if self.player_y is not None:
            y = float(self.player_y)
        elif any(s.type_key == "corridor" for s in self.obstacles):
            y = self.field_h * 0.5
        else:
            y = self.field_h * 0.15
        return x, y

    # ------------------------------------------------------------------
    def to_patterns(self) -> list[PatternSpec]:
        """Expand every obstacle type into a generator spec (deterministic)."""
        patterns: list[PatternSpec] = []
        for i, spawn in enumerate(self.obstacles):
            type_spec = get_obstacle_type(spawn.type_key)
            patterns.append(
                resolve_spec(
                    type_spec,
                    spawn,
                    player_radius=self.player_hitbox_radius,
                    field_w=self.field_w,
                    field_h=self.field_h,
                    group_id=i,
                )
            )
        return patterns

    def to_spec(self):
        """Build the :class:`ScenarioSpec` this scenario describes."""
        from bullet_sim.scenarios.spec import ScenarioSpec

        px, py = self.player_spawn()
        return ScenarioSpec(
            name=self.name,
            seed=int(self.seed),
            duration=float(self.duration),
            dt=float(self.dt),
            field_w=float(self.field_w),
            field_h=float(self.field_h),
            player_x=px,
            player_y=py,
            player_radius=float(self.player_hitbox_radius),
            player_speed=float(self.player_speed),
            target_x=self.target_x,
            target_y=self.target_y,
            collision="obstacle",
            patterns=self.to_patterns(),
            safety_margin=float(self.safety_margin),
            safety_horizon=float(self.safety_horizon),
            require_valid_scenario=bool(self.require_valid),
            meta={
                "obstacle_scenario": self.name,
                "obstacle_types": self.type_keys,
                "player_hitbox_radius": self.player_hitbox_radius,
                **dict(self.meta),
            },
        )

    def build(self, *, validate: bool | None = None):
        """Build a ready-to-run scenario, running the safety check by default."""
        from bullet_sim.scenarios.builder import build_from_spec

        spec = self.to_spec()
        if validate is None:
            validate = self.require_valid
        if validate:
            built = build_valid_or_raise(spec)
        else:
            built = build_from_spec(spec)
        return built

    # ------------------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        """Everything a组员 needs to understand the scene."""
        return {
            "name": self.name,
            "seed": self.seed,
            "duration": self.duration,
            "dt": self.dt,
            "field": [self.field_w, self.field_h],
            "player": {
                "hitbox_radius": self.player_hitbox_radius,
                "diameter": 2.0 * self.player_hitbox_radius,
                "visible_equals_hitbox": True,
                "speed": self.player_speed,
            },
            "obstacle_types": [t.to_dict() for t in self.types()],
            "obstacle_params": [s.to_dict() for s in self.obstacles],
            "safety": {
                "margin": self.safety_margin,
                "horizon": self.safety_horizon,
                "require_valid": self.require_valid,
            },
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "seed": self.seed,
            "duration": self.duration,
            "dt": self.dt,
            "field_w": self.field_w,
            "field_h": self.field_h,
            "player_hitbox_radius": self.player_hitbox_radius,
            "player_speed": self.player_speed,
            "player_x": self.player_x,
            "player_y": self.player_y,
            "obstacles": [s.to_dict() for s in self.obstacles],
            "safety_margin": self.safety_margin,
            "safety_horizon": self.safety_horizon,
            "require_valid": self.require_valid,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "meta": dict(self.meta),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ObstacleScenario":
        data = dict(payload)
        data["obstacles"] = [ObstacleSpawn.from_value(v) for v in data.get("obstacles", [])]
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown obstacle scenario fields: {sorted(unknown)}")
        return cls(**data)

    def save(self, path: str) -> str:
        import json
        from pathlib import Path

        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        return str(p)

    @classmethod
    def load(cls, path: str) -> "ObstacleScenario":
        import json
        from pathlib import Path

        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


# --------------------------------------------------------------------------
# composition helpers
# --------------------------------------------------------------------------


def scenario_from_types(
    types: Sequence[Any],
    *,
    name: str | None = None,
    seed: int = 0,
    duration: float = 30.0,
    **kwargs: Any,
) -> ObstacleScenario:
    """Convenience constructor: ``scenario_from_types(["moving_block", ...])``."""
    spawns = [ObstacleSpawn.from_value(t) for t in types]
    return ObstacleScenario(
        name=name or "+".join(s.type_key for s in spawns) or "empty",
        seed=seed,
        duration=duration,
        obstacles=spawns,
        **kwargs,
    )


def build_valid_or_raise(spec: Any):
    """Build ``spec`` and refuse to hand back a scene with no feasible path."""
    from bullet_sim.scenarios.builder import build_from_spec
    from bullet_sim.safety.validate import validate_scenario

    built = build_from_spec(spec)
    report = validate_scenario(built, spec=spec)
    built.meta["safety"] = report.to_dict()
    if not report.feasible:
        raise ConfigError(
            "scenario rejected by the safety check: no feasible collision-free "
            f"path found ({report.reason}). Adjust obstacle count / speed / size / "
            "gap_width / spawn_distance, or lower safety_margin. "
            "Use require_valid=False to build it anyway for debugging."
        )
    return built


def default_scenario(type_key: str = "moving_block", **kwargs: Any) -> ObstacleScenario:
    return scenario_from_types([type_key], **kwargs)


def size_preset(name: str) -> float:
    try:
        return SIZE_PRESETS[name]
    except KeyError as exc:
        raise ConfigError(
            f"unknown size preset {name!r}; expected {sorted(SIZE_PRESETS)}"
        ) from exc


__all__ = [
    "ObstacleScenario",
    "scenario_from_types",
    "default_scenario",
    "build_valid_or_raise",
    "size_preset",
    "CATALOG",
]
