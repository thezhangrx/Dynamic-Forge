"""Real-world dynamic obstacle types and their composition.

The platform's scenario vocabulary is **obstacle types**, not difficulty labels:

    ObstacleScenario([moving_block, wall_with_gap, small_obstacles, corridor])
        -> ScenarioSpec -> World

Each type documents what it is, what real situation it simulates, how it moves,
why that motion is reasonable, and what the player must solve - see
``bullet_sim/obstacles/catalog.py``.

Only obstacles that simulate something real exist: decorative centre-fired
patterns (radial / spiral / aimed / burst / line / wall / random) have been
removed from the platform.  The player is a circle whose drawn size equals its
collision size.
"""

from bullet_sim.obstacles.catalog import (
    CATALOG,
    CORRIDOR,
    CROSS_TRAFFIC,
    GENERATABLE,
    MOVING_BLOCK,
    SMALL_OBSTACLES,
    WALL_WITH_GAP,
    catalogue_summary,
    get_obstacle_type,
)
from bullet_sim.obstacles.scenario import (
    ObstacleScenario,
    build_valid_or_raise,
    default_scenario,
    scenario_from_types,
    size_preset,
)
from bullet_sim.obstacles.spec import (
    SAFETY_STRATEGIES,
    SIZE_PRESETS,
    ObstacleSpawn,
    ObstacleType,
    resolve_spec,
)

__all__ = [
    "ObstacleType",
    "ObstacleSpawn",
    "ObstacleScenario",
    "SIZE_PRESETS",
    "SAFETY_STRATEGIES",
    "resolve_spec",
    "CATALOG",
    "GENERATABLE",
    "MOVING_BLOCK",
    "WALL_WITH_GAP",
    "SMALL_OBSTACLES",
    "CORRIDOR",
    "CROSS_TRAFFIC",
    "get_obstacle_type",
    "catalogue_summary",
    "scenario_from_types",
    "default_scenario",
    "expand_mixed",
    "build_valid_or_raise",
    "size_preset",
]
