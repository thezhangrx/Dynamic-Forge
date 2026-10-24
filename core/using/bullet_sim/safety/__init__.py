"""Safety layer: does a scenario *have* a feasible escape?

    Scenario Generator -> Obstacle Generation -> Free Space Analysis
                       -> Path Feasibility Check -> Valid Scenario

* :mod:`bullet_sim.safety.free_space` - configuration-space free space
  (exact obstacle inflation by the player hitbox + safety margin);
* :mod:`bullet_sim.safety.path_search` - time-expanded reachability with
  ``max_speed`` / ``max_accel`` limits and trajectory backtracking;
* :mod:`bullet_sim.safety.validate` - the public verdict API and the
  generate -> check -> adjust -> regenerate loop.

**有挑战 ≠ 必死**: a hard scene is one where a path exists but requires
prediction and timing, never one where no path exists at all.
"""

from bullet_sim.safety.free_space import (
    FreeSpaceGrid,
    FreeSpaceSequence,
    occupancy_mask,
    rasterize_free_space,
)
from bullet_sim.safety.path_search import (
    ReachabilityResult,
    SafePath,
    backtrack_path,
    reachable_set,
)
from bullet_sim.safety.validate import (
    STRATEGY_NOTES,
    SafeRegion,
    SafetyConfig,
    SafetyReport,
    compute_safe_region,
    find_safe_path,
    free_space_sequence,
    generate_valid_scenario,
    is_scenario_valid,
    predict_collision,
    relax_scenario,
    validate_scenario,
)

__all__ = [
    "FreeSpaceGrid",
    "FreeSpaceSequence",
    "occupancy_mask",
    "rasterize_free_space",
    "SafePath",
    "ReachabilityResult",
    "reachable_set",
    "backtrack_path",
    "SafetyConfig",
    "SafetyReport",
    "SafeRegion",
    "STRATEGY_NOTES",
    "compute_safe_region",
    "find_safe_path",
    "predict_collision",
    "validate_scenario",
    "is_scenario_valid",
    "generate_valid_scenario",
    "relax_scenario",
    "free_space_sequence",
]
