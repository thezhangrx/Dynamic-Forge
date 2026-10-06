"""Named difficulty presets and the "one click" scenario entry points.

    >>> from bullet_sim.scenarios.presets import easy, hard, stress
    >>> s = hard(seed=7)             # a fully-specified ScenarioSpec
    >>> s = stress(2000, seed=7)     # exactly ~2000 simultaneously-live bullets
"""

from __future__ import annotations

from typing import Any

from bullet_sim.scenarios.builder import BuiltScenario, build_from_spec, make_scenario
from bullet_sim.scenarios.complexity import (
    BULLET_COUNT_LADDER,
    LEVEL_VALUES,
    ComplexityProfile,
    complexity_from_bullet_count,
    profile_for,
)
from bullet_sim.scenarios.spec import ScenarioSpec

#: Complexity value behind each named difficulty.
DIFFICULTY_LEVELS: tuple[str, ...] = ("easy", "medium", "hard", "extreme")


def scenario_for_level(level: str, *, seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    """Build one of the four canonical difficulty scenarios."""
    key = level.strip().lower()
    if key not in LEVEL_VALUES:
        raise ValueError(f"unknown level {level!r}; expected one of {DIFFICULTY_LEVELS}")
    return make_scenario(LEVEL_VALUES[key], seed=seed, name=key, **kwargs)


def easy(seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    return scenario_for_level("easy", seed=seed, **kwargs)


def medium(seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    return scenario_for_level("medium", seed=seed, **kwargs)


def hard(seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    return scenario_for_level("hard", seed=seed, **kwargs)


def extreme(seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    return scenario_for_level("extreme", seed=seed, **kwargs)


def stress(bullet_count: int = 1000, *, seed: int = 0, **kwargs: Any) -> ScenarioSpec:
    """Scenario calibrated to ``bullet_count`` simultaneously-live bullets."""
    return make_scenario(
        complexity_from_bullet_count(bullet_count),
        seed=seed,
        bullet_count=int(bullet_count),
        name=f"stress_{int(bullet_count)}",
        **kwargs,
    )


def scenario(
    level: str | float | int = "medium", *, seed: int = 0, **kwargs: Any
) -> ScenarioSpec:
    """Uniform entry point: level name, complexity float, or explicit bullet count."""
    if isinstance(level, str) and level.strip().lower() in LEVEL_VALUES:
        return scenario_for_level(level, seed=seed, **kwargs)
    if isinstance(level, int):
        return stress(level, seed=seed, **kwargs)
    return make_scenario(level, seed=seed, **kwargs)


def built(
    level: str | float | int = "medium", *, seed: int = 0, **kwargs: Any
) -> BuiltScenario:
    spec = scenario(level, seed=seed, **kwargs)
    profile = profile_for(
        spec.meta.get("complexity", 0.5),
        duration=spec.duration,
        bullet_count=spec.meta.get("target_bullet_count"),
    )
    return build_from_spec(spec, profile=profile)


def all_levels(seed: int = 0, **kwargs: Any) -> dict[str, ScenarioSpec]:
    return {name: scenario_for_level(name, seed=seed, **kwargs) for name in DIFFICULTY_LEVELS}


__all__ = [
    "DIFFICULTY_LEVELS",
    "BULLET_COUNT_LADDER",
    "ComplexityProfile",
    "easy",
    "medium",
    "hard",
    "extreme",
    "stress",
    "scenario",
    "built",
    "scenario_for_level",
    "all_levels",
]
