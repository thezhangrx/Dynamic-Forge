"""Pattern registry facade.

Configs and the CLI address patterns purely by name.  ``registry`` re-exports
the lookup/registration surface so that external code never needs to import the
individual generator modules.
"""

from __future__ import annotations

from bullet_sim.generators.composite import build_timeline, expand_patterns
from bullet_sim.generators.patterns import (
    available_patterns,
    generate_pattern,
    generator_for,
    register_generator,
)
from bullet_sim.generators.spec import (
    PATTERN_KINDS,
    PATTERN_TYPE_IDS,
    PatternSpec,
    pattern_from,
    patterns_from,
)

__all__ = [
    "PATTERN_KINDS",
    "PATTERN_TYPE_IDS",
    "PatternSpec",
    "pattern_from",
    "patterns_from",
    "build_timeline",
    "expand_patterns",
    "available_patterns",
    "generate_pattern",
    "generator_for",
    "register_generator",
]
