"""Predictive Corridor Selector -- v0.6.0-PredictiveCorridorSelector.

# [v0.6.0-PredictiveCorridorSelector]
# PURPOSE:
#   Upgrade the Future Safe Corridor from a last-moment tie-break preference into
#   a real *predictive selector* that participates in candidate choice.  The
#   existing v0.5.2 hard-safety stack (FAR / ASE / SafetyGate / EmergencyFallback)
#   still decides which candidates are allowed; the selector only orders the
#   candidates that already passed that filter.
#
# OPEN-SOURCE REFERENCE (structural ideas only, nothing copied):
#   F1TENTH Follow-the-Gap -- safety bubble, continuous free-space gap, max gap,
#       gap goal.  The selector consumes a continuous future gap width.
#   CommonRoad-Reach -- reachable set / future driving corridor.  The selector
#       consumes a future reachability flag.
#   Nav2 MPPI -- hard collision / critical safety separated from ordinary
#       preference, and an ordinary preference may never override collision
#       safety.  Enforced by the Level-1 filter and the critical-band floor.
#
# DECISION ORDER (lexicographic, never one big weighted sum):
#   Level 1  existing hard safety      -- only feasible candidates are eligible
#   Level 1c existing critical tier    -- a critical candidate can never be
#                                         promoted over a non-critical reference
#   Level 2  future reachability       -- reach_12 and reach_23 hold
#   Level 3  persistent safe sectors   -- popcount(mask_t1 & mask_t2 & mask_t3)
#   Level 4  future corridor width     -- max_run at t3
#   Level 5  future corridor clearance -- corridor_clearance at t3
#   Level 6  existing FES / behaviour / task (then index, deterministically)
#
#   ``corridor_type`` is deliberately NOT a selector input: UNKNOWN is not an
#   infinite penalty (it only loses the reachability level), STABLE earns no
#   bonus, and DEAD_END is never manufactured.
#
# FPGA MAPPING:
#   The selector itself is compare / min / max / bitwise / popcount over an
#   already-materialised table.  It performs no geometry: it never computes a
#   SafeRect, never re-runs FAR, and never re-runs the corridor kernels.
#
# CPU ROLE:
#   Bounded lexicographic ordering over the feasible candidate set.  All safety
#   filtering stays where it was: FAR / ASE / SafetyGate / EmergencyFallback.
#
# COMPLEXITY:
#   O(K) with K = number of feasible candidates, integer/float comparisons only,
#   plus one corridor evaluation per eligible candidate (see
#   :func:`build_future_space_table`), never inside the selector loop.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from cpu.decision.models import Action, SceneState
from cpu.decision.viability import ViabilityContext

__all__ = [
    "FSC_SELECTOR_ENABLED",
    "LEVEL_NONE",
    "LEVEL_REACHABILITY",
    "LEVEL_PERSISTENT",
    "LEVEL_MAX_RUN",
    "LEVEL_CLEARANCE",
    "LEVEL_EXISTING_TIERS",
    "LEVEL_HARD_SAFETY",
    "FutureSpaceTable",
    "SelectorDecision",
    "build_future_space_table",
    "select_corridor_candidate",
]

#: Master switch.  DEFAULT OFF: with it off nothing here runs at all and the
#: emitted action is exactly v0.5.2-LTVGuarded.  Override with
#: ``DF_ENABLE_FSC_SELECTOR``.
FSC_SELECTOR_ENABLED: bool = os.environ.get(
    "DF_ENABLE_FSC_SELECTOR", "0"
).strip().lower() in {"1", "true", "yes", "on"}

# -- which level produced the decision (diagnostics) -------------------------
LEVEL_NONE = "unchanged"          # selector ran, winner == v0.5.2 choice
LEVEL_REACHABILITY = "reachability"
LEVEL_PERSISTENT = "persistent"
LEVEL_MAX_RUN = "max_run"
LEVEL_CLEARANCE = "clearance"
LEVEL_EXISTING_TIERS = "existing_tiers"
LEVEL_HARD_SAFETY = "hard_safety"  # selector disabled / not applicable


@dataclass(frozen=True)
class FutureSpaceTable:
    """Already-computed corridor facts, indexed by the planner's candidates.

    Every array is length ``len(actions)``; only the evaluated indices are
    meaningful.  This is the *only* thing the selector reads.
    """

    reach_12: np.ndarray
    reach_23: np.ndarray
    persistent_sectors: np.ndarray
    max_run_t3: np.ndarray
    corridor_clearance_t3: np.ndarray
    safe_rect_valid_t3: np.ndarray
    corridor_type: tuple[str, ...] = ()
    evaluated: tuple[int, ...] = ()

    @property
    def reachable(self) -> np.ndarray:
        return np.asarray(
            self.reach_12 & self.reach_23, dtype=bool
        )

    def to_dict(self) -> dict:
        return {
            "evaluated": [int(i) for i in self.evaluated],
            "reachable": [bool(v) for v in self.reachable],
            "persistent_sectors": [int(v) for v in self.persistent_sectors],
            "max_run_t3": [int(v) for v in self.max_run_t3],
            "corridor_clearance_t3": [
                float(v) for v in self.corridor_clearance_t3
            ],
            "safe_rect_valid_t3": [bool(v) for v in self.safe_rect_valid_t3],
        }


@dataclass(frozen=True)
class SelectorDecision:
    """Outcome of one Predictive Corridor Selector evaluation."""

    enabled: bool
    #: True when the selector actually ordered the feasible set.
    applied: bool
    #: Final candidate index (the v0.5.2 choice when not applied).
    index: int
    #: The v0.5.2 choice, always recoverable.
    base_index: int
    reason: str
    #: Which lexicographic level produced the change ("" when unchanged).
    level: str
    #: Number of feasible candidates that entered the selector (Level 1 output).
    hard_survivors: int
    #: Number of candidates that passed the critical-band floor.
    eligible: int
    #: Candidates that were UNKNOWN at t3 but still allowed to compete.
    unknown_candidates: int = 0
    #: Per-level trigger flags for this frame.
    triggered_reachability: bool = False
    triggered_persistent: bool = False
    triggered_max_run: bool = False
    triggered_clearance: bool = False
    triggered_existing_tiers: bool = False
    #: v0.5.2 base vs winner raw facts (diagnostics).
    base_reachable: bool = False
    winner_reachable: bool = False
    base_persistent: int = 0
    winner_persistent: int = 0
    base_max_run_t3: int = 0
    winner_max_run_t3: int = 0

    @property
    def changed(self) -> bool:
        return int(self.index) != int(self.base_index)

    def to_dict(self) -> dict:
        return {
            "enabled": bool(self.enabled),
            "applied": bool(self.applied),
            "index": int(self.index),
            "base_index": int(self.base_index),
            "changed": bool(self.changed),
            "reason": self.reason,
            "level": self.level,
            "hard_survivors": int(self.hard_survivors),
            "eligible": int(self.eligible),
            "unknown_candidates": int(self.unknown_candidates),
            "triggered_reachability": bool(self.triggered_reachability),
            "triggered_persistent": bool(self.triggered_persistent),
            "triggered_max_run": bool(self.triggered_max_run),
            "triggered_clearance": bool(self.triggered_clearance),
            "triggered_existing_tiers": bool(self.triggered_existing_tiers),
            "base_reachable": bool(self.base_reachable),
            "winner_reachable": bool(self.winner_reachable),
            "base_persistent": int(self.base_persistent),
            "winner_persistent": int(self.winner_persistent),
            "base_max_run_t3": int(self.base_max_run_t3),
            "winner_max_run_t3": int(self.winner_max_run_t3),
        }


def build_future_space_table(
    shadow,
    scene: SceneState,
    actions: Sequence[Action],
    indices: Sequence[int],
    context: ViabilityContext,
    *,
    anchors: object | None = None,
) -> FutureSpaceTable:
    """Materialise the corridor table for ``indices`` (the feasible set).

    Uses the existing Future Safe Corridor evaluator exactly once per candidate;
    nothing here re-derives FAR, ASE or the safety gate.  When the selector is
    OFF the caller must not invoke this at all.
    """
    actions = tuple(actions)
    n = len(actions)
    reach_12 = np.zeros(n, dtype=bool)
    reach_23 = np.zeros(n, dtype=bool)
    persistent = np.zeros(n, dtype=np.int64)
    max_run_t3 = np.zeros(n, dtype=np.int64)
    clearance_t3 = np.zeros(n, dtype=np.float64)
    rect_valid_t3 = np.zeros(n, dtype=bool)
    corridor_type: list[str] = [""] * n
    evaluated: list[int] = []

    for raw in indices:
        i = int(raw)
        if i < 0 or i >= n:
            continue
        descriptor = shadow.evaluate(
            scene, actions[i], context, candidate_id=i, anchors=anchors
        )
        reach_12[i] = bool(descriptor.reach_12)
        reach_23[i] = bool(descriptor.reach_23)
        persistent[i] = int(descriptor.persistent_sectors)
        max_run_t3[i] = int(descriptor.widths[2])
        clearance_t3[i] = float(descriptor.per_anchor[2].corridor_clearance)
        rect_valid_t3[i] = bool(descriptor.safe_rect_valids[2])
        corridor_type[i] = descriptor.corridor_type
        evaluated.append(i)

    return FutureSpaceTable(
        reach_12=reach_12,
        reach_23=reach_23,
        persistent_sectors=persistent,
        max_run_t3=max_run_t3,
        corridor_clearance_t3=clearance_t3,
        safe_rect_valid_t3=rect_valid_t3,
        corridor_type=tuple(corridor_type),
        evaluated=tuple(evaluated),
    )


def _selector_key(
    i: int,
    *,
    reachable: np.ndarray,
    persistent_sectors: np.ndarray,
    max_run_t3: np.ndarray,
    corridor_clearance_t3: np.ndarray,
    fes: np.ndarray,
    behav: np.ndarray,
    task: np.ndarray,
) -> tuple:
    """Lexicographic key: smaller is better.  Levels 2-6, then the index."""
    return (
        0 if bool(reachable[i]) else 1,
        -int(persistent_sectors[i]),
        -int(max_run_t3[i]),
        -float(corridor_clearance_t3[i]),
        float(fes[i]),
        float(behav[i]),
        float(task[i]),
        int(i),
    )


#: Number of key components belonging to the corridor levels (L2..L5).
_CORRIDOR_KEY_LEN = 4
#: Number of key components belonging to the existing tiers (L6).
_EXISTING_KEY_LEN = 3
#: First key component that is only the deterministic index tie-break.
_INDEX_KEY_POS = _CORRIDOR_KEY_LEN + _EXISTING_KEY_LEN


def select_corridor_candidate(
    *,
    enabled: bool,
    feasible: Sequence[int],
    base_index: int,
    critical_band: Sequence[int],
    table: FutureSpaceTable | None,
    fes: Sequence[float],
    behav: Sequence[float],
    task: Sequence[float],
    risk_band: Sequence[int] | None = None,
) -> SelectorDecision:
    """Order the already-safe candidates by future corridor structure.

    Pure, deterministic, allocation-free apart from the key tuples.  When the
    selector is disabled or there is nothing to order, ``index == base_index``
    and the v0.5.2 action is untouched.
    """
    base = int(base_index)
    indices = [int(i) for i in feasible]

    def _off(reason: str, level: str = LEVEL_HARD_SAFETY) -> SelectorDecision:
        return SelectorDecision(
            enabled=bool(enabled),
            applied=False,
            index=base,
            base_index=base,
            reason=reason,
            level=level,
            hard_survivors=len(indices),
            eligible=0,
        )

    # The switch must be a total no-op when off.
    if not enabled:
        return _off("disabled")
    # Level 1: only candidates that already passed the existing hard safety gate.
    if table is None or not indices:
        return _off("no_feasible_candidate")

    reachable = table.reachable
    n = len(table.max_run_t3)
    indices = [i for i in indices if 0 <= i < n]
    if not indices:
        return _off("no_feasible_candidate")

    # Level 1c: existing critical tier is safety, not preference -- a critical
    # candidate may never be promoted over a non-critical reference.
    base_critical = int(critical_band[base]) if 0 <= base < len(critical_band) else 0
    eligible = [i for i in indices if int(critical_band[i]) <= base_critical]
    if not eligible:
        return _off("critical_floor")

    unknown = sum(1 for i in eligible if not bool(table.safe_rect_valid_t3[i]))

    winner = min(
        eligible,
        key=lambda i: _selector_key(
            i,
            reachable=reachable,
            persistent_sectors=table.persistent_sectors,
            max_run_t3=table.max_run_t3,
            corridor_clearance_t3=table.corridor_clearance_t3,
            fes=fes,
            behav=behav,
            task=task,
        ),
    )

    def _key(i: int) -> tuple:
        return _selector_key(
            i,
            reachable=reachable,
            persistent_sectors=table.persistent_sectors,
            max_run_t3=table.max_run_t3,
            corridor_clearance_t3=table.corridor_clearance_t3,
            fes=fes,
            behav=behav,
            task=task,
        )

    # A change needs a *genuine* preference.  If the winner differs from the
    # v0.5.2 choice only by the deterministic index tie-break, there is nothing
    # to prefer and the v0.5.2 action stands (this also protects the action
    # hysteresis hold).
    if winner != base and _key(winner)[:_INDEX_KEY_POS] == _key(base)[:_INDEX_KEY_POS]:
        winner = base
    # Level 6 alone (corridor Levels 2-5 all tied) is the weakest kind of
    # preference, so it may not worsen the existing risk band.
    elif (
        winner != base
        and _key(winner)[:_CORRIDOR_KEY_LEN] == _key(base)[:_CORRIDOR_KEY_LEN]
        and risk_band is not None
        and 0 <= winner < len(risk_band)
        and 0 <= base < len(risk_band)
        and int(risk_band[winner]) > int(risk_band[base])
    ):
        winner = base

    base_reach = bool(reachable[base]) if 0 <= base < n else False
    win_reach = bool(reachable[winner])
    base_persist = int(table.persistent_sectors[base]) if 0 <= base < n else 0
    win_persist = int(table.persistent_sectors[winner])
    base_run = int(table.max_run_t3[base]) if 0 <= base < n else 0
    win_run = int(table.max_run_t3[winner])

    # Attribute the change to the first level at which the winner strictly beats
    # the v0.5.2 choice (diagnostics only).
    t_reach = t_persist = t_run = t_clear = t_tiers = False
    level = LEVEL_NONE
    if winner != base:
        if win_reach and not base_reach:
            level, t_reach = LEVEL_REACHABILITY, True
        elif win_persist > base_persist:
            level, t_persist = LEVEL_PERSISTENT, True
        elif win_run > base_run:
            level, t_run = LEVEL_MAX_RUN, True
        elif float(table.corridor_clearance_t3[winner]) > float(
            table.corridor_clearance_t3[base]
        ):
            level, t_clear = LEVEL_CLEARANCE, True
        else:
            level, t_tiers = LEVEL_EXISTING_TIERS, True

    return SelectorDecision(
        enabled=True,
        applied=bool(winner != base),
        index=int(winner),
        base_index=base,
        reason="selected" if winner != base else "unchanged",
        level=level,
        hard_survivors=len(indices),
        eligible=len(eligible),
        unknown_candidates=int(unknown),
        triggered_reachability=t_reach,
        triggered_persistent=t_persist,
        triggered_max_run=t_run,
        triggered_clearance=t_clear,
        triggered_existing_tiers=t_tiers,
        base_reachable=base_reach,
        winner_reachable=win_reach,
        base_persistent=base_persist,
        winner_persistent=win_persist,
        base_max_run_t3=base_run,
        winner_max_run_t3=win_run,
    )
