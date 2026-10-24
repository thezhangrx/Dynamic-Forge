"""The parameter space that replaces Easy / Medium / Hard / Extreme.

Difficulty is **not** a label the user picks; it is a *function* of measurable
environment parameters::

    complexity = f(obstacle_count, relative_speed, geometry_complexity,
                   density, available_free_space, prediction_horizon)

Everything here is either (a) an input parameter a组员 sets, or (b) a quantity
measured from the built scenario / the safety report.  Nothing is a hand-tuned
constant pretending to be a difficulty level, and the legacy level names survive
only as *presets over this parameter space*.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping

import numpy as np

from bullet_sim.core.errors import ConfigError


@dataclass
class ComplexityParams:
    """The input parameters of the environment (all optional, all explicit)."""

    # --- obstacle population ------------------------------------------
    obstacle_count: int = 12
    obstacle_size: float = 0.8          # multiple of the player circle diameter
    obstacle_speed: float = 90.0        # world units / second
    obstacle_density: float | None = None  # area fraction; None = derived

    # --- geometry ------------------------------------------------------
    gap_width: float | None = None
    gap_position: float = 0.5
    gap_motion: str = "static"
    corridor_width: float | None = None
    geometry_complexity: float | None = None  # 0..1, derived if None

    # --- timing / perception -------------------------------------------
    spawn_distance: float | None = None   # boundary -> player distance
    relative_speed: float | None = None   # |v_obs - v_player|; derived if None
    reaction_time: float | None = None    # spawn_distance / relative_speed
    prediction_horizon: float = 2.0       # seconds the agent must look ahead
    field_of_view: float | None = None    # observation radius; None = unlimited

    # --- player --------------------------------------------------------
    player_speed: float = 200.0
    player_radius: float = 10.0
    player_hitbox_radius: float = 4.0

    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.obstacle_count < 0:
            raise ConfigError("obstacle_count must be >= 0")
        if self.obstacle_size <= 0:
            raise ConfigError("obstacle_size must be > 0")
        if self.obstacle_speed < 0:
            raise ConfigError("obstacle_speed must be >= 0")
        if self.prediction_horizon <= 0:
            raise ConfigError("prediction_horizon must be > 0 seconds")
        if self.player_hitbox_radius > self.player_radius:
            raise ConfigError(
                "player_hitbox_radius must not exceed player_radius "
                "(the drawn circle and the collision circle are the same thing)"
            )

    # ------------------------------------------------------------------
    @property
    def player_diameter(self) -> float:
        """Diameter of the player circle: the unit obstacle sizes are quoted in."""
        return 2.0 * float(self.player_radius)

    def resolved(self, *, field_w: float, field_h: float) -> "ComplexityParams":
        """Fill in the derived parameters that are still ``None``."""
        out = replace(self)
        if out.relative_speed is None:
            # worst case: an obstacle closing head-on on the player
            out.relative_speed = float(abs(out.obstacle_speed) + abs(out.player_speed))
        if out.spawn_distance is None:
            out.spawn_distance = float(min(field_w, field_h))
        if out.reaction_time is None:
            out.reaction_time = float(out.spawn_distance) / max(float(out.relative_speed), 1e-9)
        if out.gap_width is None:
            out.gap_width = 3.0 * out.player_diameter
        if out.corridor_width is None:
            out.corridor_width = 3.0 * out.player_diameter
        if out.obstacle_density is None:
            area = max(field_w * field_h, 1e-9)
            size = out.obstacle_size * out.player_diameter
            out.obstacle_density = float(
                min(1.0, out.obstacle_count * size * size / area)
            )
        if out.geometry_complexity is None:
            out.geometry_complexity = geometry_complexity(
                out.obstacle_count, out.gap_width, out.player_diameter
            )
        return out

    def to_dict(self) -> dict[str, Any]:
        return {
            "obstacle_count": self.obstacle_count,
            "obstacle_size": self.obstacle_size,
            "obstacle_speed": self.obstacle_speed,
            "obstacle_density": self.obstacle_density,
            "gap_width": self.gap_width,
            "gap_position": self.gap_position,
            "gap_motion": self.gap_motion,
            "corridor_width": self.corridor_width,
            "geometry_complexity": self.geometry_complexity,
            "spawn_distance": self.spawn_distance,
            "relative_speed": self.relative_speed,
            "reaction_time": self.reaction_time,
            "prediction_horizon": self.prediction_horizon,
            "field_of_view": self.field_of_view,
            "player_speed": self.player_speed,
            "player_radius": self.player_radius,
            "player_hitbox_radius": self.player_hitbox_radius,
            "meta": dict(self.meta),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ComplexityParams":
        known = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        unknown = set(payload) - known
        if unknown:
            raise ConfigError(f"unknown complexity params: {sorted(unknown)}")
        return cls(**dict(payload))


def geometry_complexity(
    obstacle_count: int, gap_width: float, player_diameter: float, *, scale: float = 12.0
) -> float:
    """How much geometric reasoning the scene demands (``0..1``).

    Combines *how many* obstacles there are with *how tight* the passage is
    relative to the player's own width: two large obstacles with a
    barely-passable gap is geometrically harder than ten small ones in the open.
    """
    tightness = float(player_diameter) / max(float(gap_width), 1e-9)
    count_term = 1.0 - float(np.exp(-max(0, int(obstacle_count)) / float(scale)))
    return float(np.clip(0.5 * count_term + 0.5 * min(tightness, 1.0), 0.0, 1.0))


@dataclass
class ComplexityScore:
    """The *measured* difficulty of one concrete scenario."""

    complexity: float
    label: str
    terms: dict[str, float] = field(default_factory=dict)
    advice: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "complexity": round(self.complexity, 4),
            "label": self.label,
            "terms": {k: round(v, 4) for k, v in self.terms.items()},
            "advice": list(self.advice),
        }

    def format(self) -> str:
        terms = " ".join(f"{k}={v:.2f}" for k, v in self.terms.items())
        return f"complexity={self.complexity:.3f} ({self.label})  [{terms}]"


def complexity_from_measurements(
    *,
    obstacle_count: int,
    free_fraction: float,
    reachable_fraction: float,
    min_clearance: float,
    player_diameter: float,
    relative_speed: float,
    player_speed: float,
    prediction_horizon: float,
    reaction_time: float | None,
    spawn_distance_hint: float = 480.0,
    path_count_hint: float = 1.0,
) -> ComplexityScore:
    """Derive a continuous difficulty from what the scene actually looks like.

    Terms (all in ``[0, 1]``, larger = harder):

    ``crowding``    how little free space is left
    ``reach``       how little of the reachable set survives
    ``tightness``   clearance relative to the player's own width
    ``speed``       relative speed versus the player's own speed
    ``anticipation`` required prediction time versus available reaction time
    ``geometry``    obstacle count contribution

    The weights are explicit constants so an experiment can restate the formula,
    and none of them is a re-labelled "extreme".
    """
    body = max(float(player_diameter), 1e-9)
    crowding = float(np.clip(1.0 - float(free_fraction), 0.0, 1.0))
    reach = float(np.clip(1.0 - float(reachable_fraction), 0.0, 1.0))
    tightness = float(np.clip(1.0 - min(float(min_clearance), 4.0 * body) / (4.0 * body), 0.0, 1.0))
    speed = float(
        np.clip(float(relative_speed) / max(float(relative_speed) + float(player_speed), 1e-9), 0.0, 1.0)
    )
    # How much of the available reaction window the required prediction horizon
    # already consumes.  Fast, close obstacles leave no time for a late
    # reaction, so the *shortfall* is the difficulty term.
    span = max(float(spawn_distance_hint), 1e-9)
    time_to_contact = span / max(float(relative_speed), 1e-9)
    anticipation = float(
        np.clip(1.0 - time_to_contact / max(float(prediction_horizon), 1e-9), 0.0, 1.0)
    )
    geometry = float(np.clip(1.0 - np.exp(-max(0, int(obstacle_count)) / 12.0), 0.0, 1.0))
    scarcity = float(np.clip(1.0 / max(float(path_count_hint), 1e-9) - 1.0, 0.0, 1.0))

    terms = {
        "crowding": crowding,
        "reach": reach,
        "tightness": tightness,
        "speed": speed,
        "anticipation": anticipation,
        "geometry": geometry,
        "path_scarcity": scarcity,
    }
    weights = {
        "crowding": 0.22,
        "reach": 0.20,
        "tightness": 0.18,
        "speed": 0.12,
        "anticipation": 0.16,
        "geometry": 0.07,
        "path_scarcity": 0.05,
    }
    total = sum(weights.values())
    score = sum(weights[k] * terms[k] for k in terms) / total
    score = float(np.clip(score, 0.0, 1.0))
    return ComplexityScore(
        complexity=score,
        label=complexity_label(score),
        terms=terms,
        advice=_advice(terms),
    )


def complexity_label(score: float, *, bins: int = 4) -> str:
    """Qualitative *description* of a measured score (never an input)."""
    names = ("easy", "moderate", "hard", "extreme")[: max(1, int(bins))]
    idx = int(np.clip(np.floor(float(score) * len(names)), 0, len(names) - 1))
    return names[idx]


def _advice(terms: Mapping[str, float]) -> list[str]:
    out: list[str] = []
    if terms.get("crowding", 0.0) > 0.6:
        out.append("自由空间偏少：减少障碍数量或尺寸")
    if terms.get("tightness", 0.0) > 0.6:
        out.append("安全余量偏紧：加大 gap_width / corridor_width 或降低 safety_margin")
    if terms.get("anticipation", 0.0) > 0.7:
        out.append("需要的预测时间接近/超过反应时间：降低障碍速度或增大 spawn_distance")
    if terms.get("reach", 0.0) > 0.7:
        out.append("可达空间很小：场景接近'唯一解'，建议放宽一处参数")
    return out


def score_report(
    report: Any, *, params: ComplexityParams, field_w: float = 640.0, field_h: float = 480.0
) -> ComplexityScore:
    """Score a :class:`~bullet_sim.safety.SafetyReport` + its parameters."""
    resolved = params.resolved(field_w=field_w, field_h=field_h)
    return complexity_from_measurements(
        obstacle_count=int(getattr(report, "obstacle_count", 0)),
        free_fraction=float(getattr(report, "free_fraction", 1.0)),
        reachable_fraction=float(getattr(report, "reachable_fraction", 1.0)),
        min_clearance=float(getattr(report, "min_clearance", 0.0)),
        player_diameter=2.0 * float(params.player_radius),
        relative_speed=float(resolved.relative_speed or 0.0),
        player_speed=float(params.player_speed),
        prediction_horizon=float(params.prediction_horizon),
        reaction_time=resolved.reaction_time,
        spawn_distance_hint=float(resolved.spawn_distance or 480.0),
    )


__all__ = [
    "ComplexityParams",
    "ComplexityScore",
    "complexity_from_measurements",
    "complexity_label",
    "geometry_complexity",
    "score_report",
]
