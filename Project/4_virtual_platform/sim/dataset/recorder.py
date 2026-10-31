"""Trajectory recorder - turns a live simulation into a training dataset.

Attaches to a :class:`~bullet_sim.simulator.world.World` as a *listener*, so the
kernel never imports the dataset layer and recording costs nothing when unused:

    rec = TrajectoryRecorder()
    env = BulletHellEnv(scenario, record=rec)
    env.run(controller)
    episode = rec.finish()
    episode.save("data/ep0001.npz")
"""

from __future__ import annotations

import time
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.errors import DatasetError
from bullet_sim.core.state import WorldSnapshot
from bullet_sim.core.version import DATASET_SCHEMA_VERSION
from bullet_sim.dataset.schema import BULLET_COLUMNS, Episode
from bullet_sim.entities.bullet import PROTOCOL_FIELDS


class TrajectoryRecorder:
    """Accumulates ``(S_t, A_t, S_{t+1}, r, done)`` transitions in fixed arrays."""

    def __init__(
        self,
        *,
        store_bullets: bool = True,
        bullet_stride: int = 1,
        store_hashes: bool = False,
        max_steps: int | None = None,
        extra_meta: Mapping[str, Any] | None = None,
    ) -> None:
        self.store_bullets = bool(store_bullets)
        self.bullet_stride = max(1, int(bullet_stride))
        self.store_hashes = bool(store_hashes)
        self.max_steps = None if max_steps is None else int(max_steps)
        self.extra_meta = dict(extra_meta or {})
        self._clear()

    # ------------------------------------------------------------------
    def _clear(self) -> None:
        self.player: list[np.ndarray] = []
        self.target: list[np.ndarray] = []
        self.env: list[np.ndarray] = []
        self.bullet_rows: list[np.ndarray] = []
        self.bullet_counts: list[int] = []
        self.bullet_offsets: list[int] = []
        self.bullet_sampled: list[bool] = []
        self.actions: list[Any] = []
        self.action_vectors: list[np.ndarray] = []
        self.collisions: list[bool] = []
        self.collision_counts: list[int] = []
        self.collision_events: list[int] = []
        self.min_dist2: list[float] = []
        self.rewards: list[float] = []
        self.terminated: list[bool] = []
        self.truncated: list[bool] = []
        self.timestamps: list[float] = []
        self.step_indices: list[int] = []
        self.hashes: list[str] = []
        self.initial: WorldSnapshot | None = None
        self.env_info: dict[str, Any] = {}
        self.meta: dict[str, Any] = {}
        self._total_rows = 0
        self._start_time = time.time()

    # ------------------------------------------------------------------
    # listener API
    # ------------------------------------------------------------------
    def listener(self, event: str, payload: Any) -> None:
        if event == "reset":
            self.on_reset(payload)
        elif event == "step":
            self.on_step(payload)

    def note_env(self, env_info: Mapping[str, Any]) -> None:
        """Remember the environment policy so the episode is self-replaying.

        Called by :class:`~bullet_sim.simulator.env.BulletHellEnv` on every
        reset; without it a replay could use different termination/collision
        settings and silently diverge.
        """
        self.env_info = dict(env_info)

    def on_reset(self, snapshot: WorldSnapshot) -> None:
        self._clear()
        self.initial = snapshot.copy()
        self.meta = {
            "schema_version": DATASET_SCHEMA_VERSION,
            "scenario_id": snapshot.scenario_id,
            "seed": int(snapshot.seed),
            "dt": float(snapshot.env.dt),
            "fps": float(1.0 / snapshot.env.dt) if snapshot.env.dt else 0.0,
            "field_w": float(snapshot.env.field_w),
            "field_h": float(snapshot.env.field_h),
            "created_by": "bullet_sim.dataset.recorder",
            "env": dict(self.env_info),
            "env_policy": dict(self.env_info.get("policy", {})),
            **self.extra_meta,
        }
        if self.store_hashes:
            self.hashes.append(snapshot.state_hash())
        self._append_state(snapshot, force=True)

    def on_step(self, record: Any) -> None:
        if self.initial is None:
            raise DatasetError("on_step called before on_reset")
        if self.max_steps is not None and len(self.rewards) >= self.max_steps:
            return
        info = getattr(record, "info", {}) or {}
        if "action_code" in info and info["action_code"] is not None:
            self.actions.append(info["action_code"])
        else:
            vec = np.asarray(getattr(record, "action_vector", (0.0, 0.0)), dtype=np.float32)
            self.actions.append(vec.copy())
        self.action_vectors.append(
            np.asarray(getattr(record, "action_vector", (0.0, 0.0)), dtype=np.float32)
        )
        col = getattr(record, "collision", None)
        self.collisions.append(bool(getattr(col, "hit", False)))
        self.collision_counts.append(int(getattr(col, "count", 0)))
        self.collision_events.append(int(info.get("collision_events", 0)))
        self.min_dist2.append(float(getattr(col, "min_dist2", np.inf)))
        self.rewards.append(float(record.reward))
        self.terminated.append(bool(record.terminated))
        self.truncated.append(bool(record.truncated))
        self.timestamps.append(float(record.sim_time))
        self.step_indices.append(int(record.step_index))
        if self.store_hashes:
            h = getattr(record, "state_hash", None)
            if h is None:
                h = record.state.state_hash()
            self.hashes.append(str(h))
        self._append_state(record.state, force=False)

    # ------------------------------------------------------------------
    def _append_state(self, snapshot: WorldSnapshot, *, force: bool) -> None:
        self.player.append(snapshot.player_array(np.float32))
        self.target.append(snapshot.target_array(np.float32))
        self.env.append(snapshot.env_array(np.float32))
        sampled = self.store_bullets and (
            force or (snapshot.env.step_index % self.bullet_stride == 0)
        )
        if sampled:
            mat = snapshot.bullets.to_protocol_matrix(compact=True).astype(np.float32)
        else:
            mat = np.empty((0, len(PROTOCOL_FIELDS)), dtype=np.float32)
        self.bullet_offsets.append(self._total_rows)
        self.bullet_counts.append(int(snapshot.bullet_count))
        self.bullet_sampled.append(bool(sampled))
        self.bullet_rows.append(mat)
        self._total_rows += mat.shape[0]

    # ------------------------------------------------------------------
    @property
    def steps(self) -> int:
        return len(self.rewards)

    def finish(self) -> Episode:
        """Materialize every accumulated array into an :class:`Episode`."""
        if self.initial is None:
            raise DatasetError("nothing recorded: reset the environment first")
        arrays: dict[str, np.ndarray] = {
            "player": _stack(self.player, (7,), "float32"),
            "target": _stack(self.target, (6,), "float32"),
            "env": _stack(self.env, (7,), "float32"),
            "bullets": (
                np.concatenate(self.bullet_rows, axis=0).astype(np.float32)
                if self.bullet_rows and self._total_rows
                else np.zeros((0, len(PROTOCOL_FIELDS)), dtype=np.float32)
            ),
            "bullet_counts": np.asarray(self.bullet_counts, dtype=np.int32),
            "bullets_offsets": np.asarray(self.bullet_offsets, dtype=np.int32),
            "bullet_sampled": np.asarray(self.bullet_sampled, dtype=bool),
            "action_vectors": _stack(self.action_vectors, (2,), "float32"),
            "collisions": np.asarray(self.collisions, dtype=bool),
            "collision_counts": np.asarray(self.collision_counts, dtype=np.int32),
            "collision_events": np.asarray(self.collision_events, dtype=np.int32),
            "min_dist2": np.asarray(self.min_dist2, dtype=np.float32),
            "rewards": np.asarray(self.rewards, dtype=np.float32),
            "terminated": np.asarray(self.terminated, dtype=bool),
            "truncated": np.asarray(self.truncated, dtype=bool),
            "done": np.asarray(
                [a or b for a, b in zip(self.terminated, self.truncated)], dtype=bool
            ),
            "timestamps": np.asarray(self.timestamps, dtype=np.float64),
            "step_indices": np.asarray(self.step_indices, dtype=np.int32),
            "state_hashes": np.asarray(self.hashes, dtype="U32"),
        }
        actions = _actions_array(self.actions)
        arrays["actions"] = actions
        meta = dict(self.meta)
        # ``note_env`` runs after ``on_reset``, so merge it here (authoritative).
        if self.env_info:
            meta["env"] = dict(self.env_info)
            meta["env_policy"] = dict(self.env_info.get("policy", {}))
        meta.update(
            {
                "steps": self.steps,
                "bullet_columns": list(BULLET_COLUMNS),
                "store_bullets": self.store_bullets,
                "bullet_stride": self.bullet_stride,
                "store_hashes": self.store_hashes,
                "collision_as_penalty": True,
                "wall_time_s": round(time.time() - self._start_time, 4),
            }
        )
        return Episode(
            meta=meta,
            initial_state=self.state_payload(0),
            arrays=arrays,
        )

    # ------------------------------------------------------------------
    def state_payload(self, t: int) -> dict[str, Any]:
        """State ``S_t`` in the canonical, JSON-round-trippable array form.

        This is the *same* shape returned by :meth:`Episode.state`, so JSON
        export/import is lossless.
        """
        n_states = len(self.player)
        if not (0 <= t < n_states):
            raise IndexError(f"state index {t} out of range [0, {n_states})")
        mat = self.bullet_rows[t]
        return {
            "step_index": int(self.env[t][3]),
            "timestamp": float(self.env[t][4]),
            "player": self.player[t].tolist(),
            "target": self.target[t].tolist(),
            "env": self.env[t].tolist(),
            "bullets": mat.tolist(),
            "bullets_recorded": bool(self.bullet_sampled[t]),
            "bullet_count": int(self.bullet_counts[t]),
            "bullet_fields": list(BULLET_COLUMNS),
            "state_hash": self.hashes[t] if t < len(self.hashes) else "",
        }

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._clear()

    def save(self, path: str, fmt: str | None = None) -> str:
        from bullet_sim.dataset.writers import save_episode

        return save_episode(self.finish(), path, fmt=fmt)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _stack(rows: Sequence[np.ndarray], shape: tuple[int, ...], dtype: str) -> np.ndarray:
    if not rows:
        return np.zeros((0,) + shape, dtype=dtype)
    return np.asarray(rows, dtype=dtype).reshape(len(rows), *shape)


def _actions_array(actions: Sequence[Any]) -> np.ndarray:
    if not actions:
        return np.zeros((0,), dtype=np.int32)
    first = actions[0]
    if isinstance(first, np.ndarray):
        return np.asarray(actions, dtype=np.float32)
    try:
        return np.asarray(actions, dtype=np.int32)
    except (TypeError, ValueError):
        return np.asarray(actions, dtype=np.float32)


def record_episode(
    env: Any,
    controller: Any = None,
    *,
    steps: int | None = None,
    seed: int | None = None,
    store_bullets: bool = True,
    bullet_stride: int = 1,
    store_hashes: bool = False,
) -> Episode:
    """Convenience: run a headless episode and return the recorded dataset."""
    rec = TrajectoryRecorder(
        store_bullets=store_bullets, bullet_stride=bullet_stride, store_hashes=store_hashes
    )
    env.recorder = rec
    env.world.add_listener(rec.listener)
    try:
        env.run(controller, steps=steps, seed=seed)
    finally:
        env.world.remove_listener(rec.listener)
    return rec.finish()


__all__ = ["TrajectoryRecorder", "record_episode", "Episode"]
