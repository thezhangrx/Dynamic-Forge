"""Scenario specification: the *only* thing needed to reproduce a scene.

A scenario is fully described by ``ScenarioSpec`` + ``seed``.  Everything else
(the initial state, the spawn timeline, the derived difficulty parameters) is a
pure function of those two inputs, so persisting a scenario next to a dataset
is sufficient to replay it exactly.
"""

from __future__ import annotations

import json

import numpy as np
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

from bullet_sim.core.actions import codec_from_spec
from bullet_sim.core.errors import ConfigError
from bullet_sim.entities.player import PlayerState
from bullet_sim.entities.target import TargetState
from bullet_sim.generators.spec import PatternSpec, pattern_from

DEFAULT_FIELD_W = 640.0
DEFAULT_FIELD_H = 480.0
DEFAULT_DT = 1.0 / 120.0
DEFAULT_DURATION = 30.0


@dataclass
class ScenarioSpec:
    """Declarative scene definition."""

    name: str = "scenario"
    seed: int = 0
    duration: float = DEFAULT_DURATION
    dt: float = DEFAULT_DT

    field_w: float = DEFAULT_FIELD_W
    field_h: float = DEFAULT_FIELD_H
    wrap: str = "cull"

    player_x: float | None = None
    player_y: float | None = None
    #: The player is a **circle**; this radius is both what you see and what
    #: collides.  There is no separate visible body: enlarging the hitbox
    #: enlarges the drawn circle, so the picture never lies about the physics.
    player_radius: float = 3.0
    player_speed: float = 200.0

    target_x: float | None = None
    target_y: float | None = None
    target_radius: float = 14.0
    target_shape: str = "circle"
    target_half_w: float = 0.0
    target_half_h: float = 0.0

    #: Hard cap on simultaneously live bullets (pool capacity is derived from
    #: this plus a safety factor so steady-state stepping never allocates).
    bullet_capacity: int | None = None

    patterns: list[PatternSpec] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Collision / episode-end configuration (all of it data, none of it code)
    # ------------------------------------------------------------------
    #: Reward subtracted per **collision event** (positive magnitude).
    #: ``reward -= collision_penalty * events``; see ``simulator/rewards.py``.
    collision_penalty: float = 1.0
    #: How overlapping frames become collision events:
    #: ``per_contact`` (one per newly touched bullet, default) |
    #: ``per_step`` (one per contact episode) |
    #: ``per_frame`` (legacy: every overlapping frame counts).
    collision_count_mode: str = "per_contact"
    #: Mode B of the experiment matrix.  ``False`` (default) treats a collision
    #: strictly as a penalty event and keeps the episode running.
    collision_terminates_episode: bool = False
    #: Optional task-success termination (reaching the target zone).
    success_terminates_episode: bool = False

    #: Qualitative difficulty of the scene: ``easy``/``medium``/``hard``/
    #: ``extreme`` or a custom label.  ``None`` for hand-written scenarios.
    difficulty: str | None = None

    #: Normalised complexity in [0, 1] (kept in sync with difficulty when known).
    complexity: float | None = None

    #: Free-form metadata (generator version, tags, measurement results, ...).
    meta: dict[str, Any] = field(default_factory=dict)

    #: Action interface descriptor, e.g. ``{"kind": "discrete"}``.
    action_space: Any = "discrete"

    #: Collision backend hint (``auto`` picks the obstacle-aware one when the
    #: scenario contains rectangular obstacles).
    collision: str = "auto"

    # --- safety / feasibility (see bullet_sim.safety) --------------------
    #: Extra clearance the planner must keep beyond the player hitbox.
    safety_margin: float = 6.0
    #: How far ahead feasibility is proven, in **seconds**.
    safety_horizon: float = 2.0
    #: When True, ``ObstacleScenario.build()`` refuses an infeasible scene.
    require_valid_scenario: bool = True

    #: Boundary margin before culling (0 = cull at the field edge).
    cull_margin: float = 32.0

    def __post_init__(self) -> None:
        self.patterns = [pattern_from(p) for p in self.patterns]
        if self.duration <= 0.0:
            raise ConfigError(f"scenario duration must be > 0, got {self.duration}")
        if self.dt <= 0.0:
            raise ConfigError(f"scenario dt must be > 0, got {self.dt}")
        if self.wrap not in ("cull", "wrap", "bounce"):
            raise ConfigError(f"unknown wrap mode {self.wrap!r}")
        from bullet_sim.collision.events import validate_count_mode

        self.collision_count_mode = validate_count_mode(self.collision_count_mode)
        if not np.isfinite(self.collision_penalty):
            raise ConfigError("collision_penalty must be finite")
        if self.collision_penalty < 0.0:
            raise ConfigError(
                "collision_penalty is a positive magnitude subtracted from the "
                f"reward; got {self.collision_penalty}. Use a negative value only "
                "via a custom reward function."
            )
        if self.field_w <= 0 or self.field_h <= 0:
            raise ConfigError("field dimensions must be positive")
        if self.player_radius <= 0.0:
            raise ConfigError("player_radius must be > 0 (the player is a circle)")
        if self.safety_margin < 0.0:
            raise ConfigError("safety_margin must be >= 0")
        if self.safety_horizon <= 0.0:
            raise ConfigError("safety_horizon must be > 0 seconds")

    # ------------------------------------------------------------------
    @property
    def total_steps(self) -> int:
        return max(1, int(round(self.duration / self.dt)))

    @property
    def player_diameter(self) -> float:
        """Diameter of the player circle - the unit obstacle sizes are quoted in."""
        return 2.0 * float(self.player_radius)

    @property
    def player_spawn(self) -> tuple[float, float]:
        # Default spawn sits below the centre: most emitters live at the field
        # centre or along the top edge, so a centre spawn would be an instant
        # hit on step 0.
        x = self.field_w * 0.5 if self.player_x is None else float(self.player_x)
        y = self.field_h * 0.22 if self.player_y is None else float(self.player_y)
        return x, y

    def player_state(self) -> PlayerState:
        x, y = self.player_spawn
        return PlayerState(
            x=x, y=y, radius=self.player_radius, speed=self.player_speed, alive=True
        )

    def target_state(self) -> TargetState:
        x = self.field_w * 0.5 if self.target_x is None else float(self.target_x)
        y = self.field_h * 0.5 if self.target_y is None else float(self.target_y)
        return TargetState(
            x=x,
            y=y,
            radius=self.target_radius,
            shape=self.target_shape,
            half_w=self.target_half_w,
            half_h=self.target_half_h,
        )

    def make_codec(self):
        return codec_from_spec(self.action_space)

    def pattern_summary(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for p in self.patterns:
            out[p.kind] = out.get(p.kind, 0) + 1
        return out

    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "seed": self.seed,
            "duration": self.duration,
            "dt": self.dt,
            "field_w": self.field_w,
            "field_h": self.field_h,
            "wrap": self.wrap,
            "player_x": self.player_x,
            "player_y": self.player_y,
            "player_radius": self.player_radius,
            "player_speed": self.player_speed,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "target_radius": self.target_radius,
            "target_shape": self.target_shape,
            "target_half_w": self.target_half_w,
            "target_half_h": self.target_half_h,
            "bullet_capacity": self.bullet_capacity,
            "collision_penalty": self.collision_penalty,
            "collision_count_mode": self.collision_count_mode,
            "collision_terminates_episode": self.collision_terminates_episode,
            "success_terminates_episode": self.success_terminates_episode,
            "difficulty": self.difficulty,
            "complexity": self.complexity,
            "patterns": [p.to_dict() for p in self.patterns],
            "meta": dict(self.meta),
            "collision": self.collision,
            "safety_margin": self.safety_margin,
            "safety_horizon": self.safety_horizon,
            "require_valid_scenario": self.require_valid_scenario,
            "action_space": self.action_space
            if isinstance(self.action_space, (str, dict))
            else "discrete",
            "cull_margin": self.cull_margin,
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ScenarioSpec":
        data = dict(payload)
        data["patterns"] = [pattern_from(p) for p in data.get("patterns", [])]
        data["meta"] = dict(data.get("meta", {}))
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        unknown = set(data) - known
        if unknown:
            raise ConfigError(
                f"unknown scenario fields: {sorted(unknown)}; known: {sorted(known)}"
            )
        return cls(**data)

    def replaced(self, **changes: Any) -> "ScenarioSpec":
        return replace(self, **changes)

    # ------------------------------------------------------------------
    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=False)

    @classmethod
    def from_json(cls, text: str) -> "ScenarioSpec":
        return cls.from_dict(json.loads(text))

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(self.to_json(), encoding="utf-8")
        return p

    @classmethod
    def load(cls, path: str | Path) -> "ScenarioSpec":
        return cls.from_json(Path(path).read_text(encoding="utf-8"))


def scenario_from(value: Any) -> ScenarioSpec:
    """Coerce dict / json-path / ScenarioSpec into a ScenarioSpec."""
    if isinstance(value, ScenarioSpec):
        return value
    if isinstance(value, Mapping):
        return ScenarioSpec.from_dict(value)
    if isinstance(value, (str, Path)):
        return ScenarioSpec.load(value)
    raise ConfigError(f"cannot build a ScenarioSpec from {value!r}")
