"""Versioned dataset schema.

The exported artifact is a real ML dataset, not a game save: every transition
carries ``state_t, action_t, next_state, collision, reward, done`` plus the
identifiers (``scenario_id``, ``seed``, ``timestamp``) needed to reproduce it.

Storage layout (per episode)
----------------------------
============================  ==========================  ==========================
key                           shape                       meaning
============================  ==========================  ==========================
``player``                    ``(T+1, 7)``                x, y, vx, vy, r, speed, alive
``target``                    ``(T+1, 6)``                x, y, radius, shape, hw, hh
``env``                       ``(T+1, 7)``                field_w, field_h, dt, step, time, wrap, budget
``bullets``                   ``(sum(n_t), 10)``          compact live bullets per step
``bullet_counts``             ``(T+1,)``                  bullets recorded at each state
``bullets_offsets``           ``(T+1,)``                  row offset into ``bullets``
``actions``                   ``(T,)``                    raw action (int or vector)
``action_vectors``            ``(T, 2)``                  encoded world-frame velocity
``collisions``                ``(T,)``                    bool
``collision_counts``          ``(T,)``                    simultaneous overlaps (frames)
``collision_events``          ``(T,)``                    NEW collision events per step
``min_dist2``                 ``(T,)``                    closest squared distance
``rewards``                   ``(T,)``                    float32
``terminated``                ``(T,)``                    bool
``truncated``                 ``(T,)``                    bool
``done``                      ``(T,)``                    bool
``timestamps``                ``(T,)``                    simulation time after the step
``step_indices``              ``(T,)``                    simulation step index
``state_hashes``              ``(T+1,)``                  canonical state digests (opt-in)
============================  ==========================  ==========================
``T`` is the number of transitions; ``T+1`` states (S_0 .. S_T) are stored.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from bullet_sim.core.state import WorldSnapshot
from bullet_sim.core.version import DATASET_SCHEMA_VERSION
from bullet_sim.entities.bullet import PROTOCOL_FIELDS

#: One row of ``bullets``.
BULLET_COLUMNS: tuple[str, ...] = tuple(PROTOCOL_FIELDS)

#: Logical transition fields exposed to consumers (JSON / iteration).
TRANSITION_FIELDS: tuple[str, ...] = (
    "state_t",
    "action_t",
    "next_state",
    "collision",
    "collision_events",
    "reward",
    "terminated",
    "truncated",
    "done",
    "scenario_id",
    "seed",
    "timestamp",
    "step_index",
    "bullet_count",
    "state_hash",
)

#: Array keys every episode is expected to contain.
ARRAY_KEYS: tuple[str, ...] = (
    "player",
    "target",
    "env",
    "bullets",
    "bullet_counts",
    "bullets_offsets",
    "bullet_sampled",
    "actions",
    "action_vectors",
    "collisions",
    "collision_counts",
    "collision_events",
    "min_dist2",
    "rewards",
    "terminated",
    "truncated",
    "done",
    "timestamps",
    "step_indices",
    "state_hashes",
)

#: dtypes used when materializing arrays (fixed so files are portable).
ARRAY_DTYPES: dict[str, str] = {
    "player": "float32",
    "target": "float32",
    "env": "float32",
    "bullets": "float32",
    "bullet_counts": "int32",
    "bullets_offsets": "int32",
    "bullet_sampled": "bool",
    "action_vectors": "float32",
    "collisions": "bool",
    "collision_counts": "int32",
    "collision_events": "int32",
    "min_dist2": "float32",
    "rewards": "float32",
    "terminated": "bool",
    "truncated": "bool",
    "done": "bool",
    "timestamps": "float64",
    "step_indices": "int32",
}

META_KEYS: tuple[str, ...] = (
    "schema_version",
    "scenario_id",
    "seed",
    "dt",
    "fps",
    "field_w",
    "field_h",
    "steps",
    "scenario",
    "action_space",
    "collision_model",
    "reward",
    "created_by",
)


@dataclass
class Episode:
    """A recorded trajectory: metadata + fixed-layout arrays."""

    meta: dict[str, Any] = field(default_factory=dict)
    initial_state: dict[str, Any] = field(default_factory=dict)
    arrays: dict[str, np.ndarray] = field(default_factory=dict)

    # ------------------------------------------------------------------
    @property
    def steps(self) -> int:
        return int(self.arrays.get("rewards", np.empty(0)).shape[0])

    @property
    def scenario_id(self) -> str:
        return str(self.meta.get("scenario_id", ""))

    @property
    def seed(self) -> int:
        return int(self.meta.get("seed", 0))

    def __len__(self) -> int:
        return self.steps

    # ------------------------------------------------------------------
    def state(self, t: int) -> dict[str, Any]:
        """Materialize state ``S_t`` (``0 <= t <= steps``) as a plain dict."""
        n_states = self.steps + 1
        if not (0 <= t < n_states):
            raise IndexError(f"state index {t} out of range [0, {n_states})")
        a = self.arrays
        lo = int(a["bullets_offsets"][t])
        hi = lo + int(a["bullet_counts"][t])
        hashes = a.get("state_hashes")
        return {
            # env row ``t`` is state S_t, so it is authoritative for step/time.
            "step_index": int(a["env"][t, 3]),
            "timestamp": float(a["env"][t, 4]),
            "player": a["player"][t].tolist(),
            "target": a["target"][t].tolist(),
            "env": a["env"][t].tolist(),
            "bullets": a["bullets"][lo:hi].tolist(),
            "bullets_recorded": bool(a["bullet_sampled"][t]) if "bullet_sampled" in a else True,
            "bullet_count": int(a["bullet_counts"][t]),
            "state_hash": str(hashes[t]) if hashes is not None and hashes.size > t else "",
            "bullet_fields": list(BULLET_COLUMNS),
        }

    def transition(self, t: int) -> dict[str, Any]:
        """Materialize one training transition (state_t -> next_state)."""
        if not (0 <= t < self.steps):
            raise IndexError(f"transition index {t} out of range [0, {self.steps})")
        a = self.arrays
        return {
            "state_t": self.state(t),
            "action_t": a["actions"][t].tolist(),
            "action_vector": a["action_vectors"][t].tolist(),
            "next_state": self.state(t + 1),
            "collision": bool(a["collisions"][t]),
            "collision_count": int(a["collision_counts"][t]),
            "collision_events": (
                int(a["collision_events"][t]) if "collision_events" in a else 0
            ),
            "min_dist2": float(a["min_dist2"][t]),
            "reward": float(a["rewards"][t]),
            "terminated": bool(a["terminated"][t]),
            "truncated": bool(a["truncated"][t]),
            "done": bool(a["done"][t]),
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "timestamp": float(a["timestamps"][t]),
            "step_index": int(a["step_indices"][t]),
            "bullet_count": int(a["bullet_counts"][t + 1]),
            "state_hash": str(a["state_hashes"][t + 1])
            if "state_hashes" in a and a["state_hashes"].size > t + 1
            else "",
        }

    def to_dict(self, max_transitions: int | None = None) -> dict[str, Any]:
        n = self.steps if max_transitions is None else min(self.steps, int(max_transitions))
        return {
            "meta": dict(self.meta),
            "initial_state": dict(self.initial_state),
            "transitions": [self.transition(t) for t in range(n)],
            "truncated_transitions": self.steps - n,
        }

    def summary(self) -> dict[str, Any]:
        a = self.arrays
        return {
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "steps": self.steps,
            "total_bullet_rows": int(a["bullets"].shape[0]) if "bullets" in a else 0,
            "max_bullets": int(a["bullet_counts"].max()) if self.steps >= 0 and "bullet_counts" in a else 0,
            "mean_bullets": float(a["bullet_counts"].mean()) if "bullet_counts" in a else 0.0,
            "collisions": int(a["collisions"].sum()) if "collisions" in a else 0,
            "total_reward": float(a["rewards"].sum()) if "rewards" in a else 0.0,
            "has_hashes": "state_hashes" in a and a["state_hashes"].size > 0,
        }


def episode_from_snapshot(
    snapshot: WorldSnapshot, meta: Mapping[str, Any] | None = None
) -> Episode:
    """Create an empty episode seeded with ``S_0``."""
    return Episode(meta=dict(meta or {}), initial_state=snapshot.to_dict())


__all__ = [
    "DATASET_SCHEMA_VERSION",
    "BULLET_COLUMNS",
    "TRANSITION_FIELDS",
    "ARRAY_KEYS",
    "ARRAY_DTYPES",
    "META_KEYS",
    "Episode",
    "episode_from_snapshot",
]
