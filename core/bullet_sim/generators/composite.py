"""Pattern composition and timeline scheduling.

A scenario is a *list* of patterns.  Composition is deliberately trivial - we
just expand every pattern into its events, tag them with a stable group id, and
merge into one :class:`~bullet_sim.generators.burst.SpawnTimeline` - but the
merge is the reason complex scenes (``radial + spiral + aimed + accelerated``)
need no special-casing anywhere in the kernel.

Determinism rules for the merge:
1. each pattern draws from its **own named RNG stream** (``pattern:<i>``), so
   adding/reordering patterns cannot perturb unrelated ones;
2. events are ordered by ``(step, pattern_index, emission_index)``;
3. the timeline is a plain sorted mapping, iterated in ascending step order.
"""

from __future__ import annotations

from dataclasses import MISSING
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.clock import FixedClock
from bullet_sim.core.errors import ScenarioError
from bullet_sim.core.rng import SeedManager
from bullet_sim.generators.burst import GenerationContext, SpawnEvent, SpawnTimeline
from bullet_sim.generators.patterns import generate_pattern
from bullet_sim.generators.spec import PATTERN_KINDS, PatternSpec, pattern_from


def expand_patterns(specs: Sequence[Any]) -> list[PatternSpec]:
    """Flatten ``mixed`` specs into their leaf patterns (recursively)."""
    out: list[PatternSpec] = []
    for raw in specs:
        spec = pattern_from(raw)
        if spec.kind != "mixed":
            out.append(spec)
            continue
        nested = spec.params.get("patterns")
        if not nested:
            raise ScenarioError("a 'mixed' pattern requires params['patterns'] = [...]")
        children = expand_patterns(nested)
        # Inheritance: a mixed wrapper may set shared defaults that children
        # leave unset. Children always win when they specify a value.
        for child in children:
            out.append(_inherit(spec, child))
    return out


def _inherit(parent: PatternSpec, child: PatternSpec) -> PatternSpec:
    """Apply a ``mixed`` wrapper's defaults to a child that kept class defaults.

    Only fields the child left untouched are inherited, so a child can always
    override anything explicitly.
    """
    skip = {"kind", "params", "type_id", "count", "origin", "aim", "group_id"}
    changes: dict[str, Any] = {}
    for name, fdef in PatternSpec.__dataclass_fields__.items():  # type: ignore[attr-defined]
        if name in skip:
            continue
        child_default = fdef.default if fdef.default is not MISSING else None
        parent_val = getattr(parent, name)
        child_val = getattr(child, name)
        if child_val == child_default and parent_val != child_default:
            changes[name] = parent_val
    return child.replaced(**changes) if changes else child


def build_timeline(
    specs: Sequence[Any],
    *,
    field_w: float,
    field_h: float,
    dt: float,
    duration: float,
    seeds: SeedManager,
    player_xy: np.ndarray | None = None,
    clock: FixedClock | None = None,
) -> SpawnTimeline:
    """Expand every pattern into one deterministic spawn timeline."""
    clock = clock or FixedClock.make(dt=dt)
    leaves = expand_patterns(specs)
    events: list[SpawnEvent] = []
    for index, spec in enumerate(leaves):
        # group_id always equals the pattern's declaration index, giving every
        # scenario a stable, reproducible per-pattern bullet grouping.
        spec = spec.replaced(group_id=index)
        ctx = GenerationContext(
            field_w=float(field_w),
            field_h=float(field_h),
            dt=float(dt),
            duration=float(duration),
            rng=seeds.generator(f"pattern:{index}:{spec.kind}"),
            player_xy=(
                np.asarray(player_xy, dtype=np.float64).reshape(2)
                if player_xy is not None
                else np.zeros(2)
            ),
            clock=clock,
        )
        generated = generate_pattern(spec, ctx)
        for emission_index, ev in enumerate(generated):
            ev.group_id = int(spec.group_id)
            events.append(ev)
    # Stable ordering: step, then declaration order (list order is stable in
    # Python, so a plain sort by step preserves declaration order ties).
    events.sort(key=lambda e: e.step)
    return SpawnTimeline(events)


def timeline_from_dict(payload: Mapping[str, Any]) -> SpawnTimeline:
    """Rebuild a timeline from :meth:`SpawnTimeline.to_list` output."""
    events = []
    for item in payload.get("events", []):
        events.append(
            SpawnEvent(
                step=int(item["step"]),
                origin=np.asarray(item.get("origin", [0.0, 0.0]), dtype=np.float64),
                origin_mode=int(item.get("origin_mode", 0)),
                aim=bool(item.get("aim", False)),
                group_id=int(item.get("group_id", 0)),
                pattern=str(item.get("pattern", "")),
                offsets=np.asarray(item.get("offsets", []), dtype=np.float64).reshape(-1, 2),
                angles=np.asarray(item.get("angles", []), dtype=np.float64),
                speeds=np.asarray(item.get("speeds", []), dtype=np.float64),
                accel_mag=np.asarray(item.get("accel_mag", []), dtype=np.float64),
                accel_angle=np.asarray(item.get("accel_angle", []), dtype=np.float64),
                radius=np.asarray(item.get("radius", []), dtype=np.float64),
                ttl=np.asarray(item.get("ttl", []), dtype=np.float64),
                angular_velocity=np.asarray(
                    item.get("angular_velocity", []), dtype=np.float64
                ),
                shape=np.asarray(item.get("shape", []), dtype=np.int16),
                half_w=np.asarray(item.get("half_w", []), dtype=np.float64),
                half_h=np.asarray(item.get("half_h", []), dtype=np.float64),
                rotation=np.asarray(item.get("rotation", []), dtype=np.float64),
                min_player_distance=float(item.get("min_player_distance", 0.0)),
                type_id=np.asarray(item.get("type_id", []), dtype=np.int16),
            )
        )
    return SpawnTimeline(events)


__all__ = ["expand_patterns", "build_timeline", "timeline_from_dict", "PATTERN_KINDS"]
