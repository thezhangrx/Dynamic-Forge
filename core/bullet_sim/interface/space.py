"""Observation / action space descriptions and the observation encoder.

Deliberately **independent of ``gymnasium``**: the platform must be usable for
supervised learning, imitation learning, planning and model-based control, not
only RL.  The shapes here are compatible with Gymnasium boxes/discretes, so an
adapter is trivial, but nothing forces the dependency.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from bullet_sim.core.actions import ActionCodec, DiscreteActionCodec
from bullet_sim.core.state import WorldSnapshot

#: Per-bullet float features in the observation matrix.
BULLET_OBS_FIELDS: tuple[str, ...] = (
    "x",
    "y",
    "vx",
    "vy",
    "radius",
    "ttl",
    "type_id",
)
BULLET_OBS_DIM = len(BULLET_OBS_FIELDS)

PLAYER_OBS_DIM = 6  # x, y, vx, vy, radius, speed
TARGET_OBS_DIM = 4  # x, y, radius, distance_to_player
ENV_OBS_DIM = 6  # field_w, field_h, sim_time, step_index, density, n_alive


# --------------------------------------------------------------------------
# space descriptors
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Discrete:
    n: int
    names: tuple[str, ...] = ()
    start: int = 0

    def contains(self, x: Any) -> bool:
        try:
            i = int(x)
        except (TypeError, ValueError):
            return False
        return self.start <= i < self.start + self.n

    def sample(self, rng: np.random.Generator) -> int:
        return int(rng.integers(0, self.n))

    def to_dict(self) -> dict:
        return {"type": "discrete", "n": self.n, "names": list(self.names)}


@dataclass(frozen=True)
class Box:
    low: Any
    high: Any
    shape: tuple[int, ...]
    dtype: str = "float32"

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        low = np.broadcast_to(np.asarray(self.low, dtype=np.float64), self.shape)
        high = np.broadcast_to(np.asarray(self.high, dtype=np.float64), self.shape)
        return (low + (high - low) * rng.random(self.shape)).astype(self.dtype)

    def to_dict(self) -> dict:
        return {
            "type": "box",
            "shape": list(self.shape),
            "low": np.asarray(self.low).tolist(),
            "high": np.asarray(self.high).tolist(),
            "dtype": self.dtype,
        }


def action_space_for(codec: ActionCodec) -> Discrete | Box:
    if isinstance(codec, DiscreteActionCodec):
        return Discrete(n=codec.dim, names=codec.names)
    return Box(
        low=-np.inf, high=np.inf, shape=(int(codec.dim),), dtype="float64"
    )


# --------------------------------------------------------------------------
# observation encoding
# --------------------------------------------------------------------------


@dataclass
class ObservationEncoder:
    """Turns ``WorldSnapshot`` into a structured observation dict.

    Two representations are produced from the same data:

    * ``encode``  -> dict with fixed-size player/target/env blocks **and**
      variable-length bullet blocks (for prediction / planning / supervision);
    * ``flatten`` -> a single fixed-size float vector (for RL and for the
      CPU/FPGA input buffer), with ``max_bullets`` padding and a validity mask.
    """

    normalize: bool = False
    max_bullets: int | None = None
    vel_scale: float = 300.0
    ttl_scale: float = 10.0
    radius_scale: float = 8.0
    include_bullets: bool = True
    extra: dict[str, Any] = field(default_factory=dict)

    # -- structured obs ----------------------------------------------------
    def encode(self, snapshot: WorldSnapshot) -> dict[str, Any]:
        p = snapshot.player
        t = snapshot.target
        e = snapshot.env
        sx = e.field_w if self.normalize and e.field_w else 1.0
        sy = e.field_h if self.normalize and e.field_h else 1.0
        sv = self.vel_scale if self.normalize else 1.0
        sr = self.radius_scale if self.normalize else 1.0
        st = self.ttl_scale if self.normalize else 1.0

        player = np.array(
            [p.x / sx, p.y / sy, p.vx / sv, p.vy / sv, p.radius / sr, p.speed / sv],
            dtype=np.float32,
        )
        target = np.array(
            [
                t.x / sx,
                t.y / sy,
                t.radius / sr,
                float(np.hypot(t.x - p.x, t.y - p.y)) / max(np.hypot(e.field_w, e.field_h), 1e-9),
            ],
            dtype=np.float32,
        )
        density = snapshot.bullet_count / max(e.field_w * e.field_h, 1e-9) * 1e4
        env = np.array(
            [
                e.field_w,
                e.field_h,
                e.sim_time,
                float(e.step_index),
                density,
                float(snapshot.bullet_count),
            ],
            dtype=np.float32,
        )

        obs: dict[str, Any] = {
            "player": player,
            "target": target,
            "env": env,
            "reward": np.float32(0.0),
            "collision": False,
            "timestamp": float(snapshot.timestamp),
            "step_index": int(e.step_index),
        }
        if self.include_bullets:
            pool = snapshot.bullets
            idx = pool.active_indices()
            n = int(idx.size)
            mat = np.zeros((n, BULLET_OBS_DIM), dtype=np.float32)
            if n:
                d = pool.data
                mat[:, 0] = d["x"][idx] / sx
                mat[:, 1] = d["y"][idx] / sy
                mat[:, 2] = d["vx"][idx] / sv
                mat[:, 3] = d["vy"][idx] / sv
                mat[:, 4] = d["radius"][idx] / sr
                mat[:, 5] = d["ttl"][idx] / st
                mat[:, 6] = d["type_id"][idx]
            obs["bullets"] = mat
            obs["bullets_mask"] = np.ones(n, dtype=bool)
        obs.update(self.extra)
        return obs

    # -- flat obs ----------------------------------------------------------
    @property
    def flat_dim(self) -> int:
        # player + target + env + (collision, reward) + (bullet count)
        head = PLAYER_OBS_DIM + TARGET_OBS_DIM + ENV_OBS_DIM + 3
        tail = (self.max_bullets or 0) * BULLET_OBS_DIM
        return head + tail

    def flatten(self, obs: Mapping[str, Any]) -> np.ndarray:
        """Fixed-size float32 vector; requires ``max_bullets`` to be set."""
        head = [
            obs["player"],
            obs["target"],
            obs["env"],
            np.array([obs.get("collision", False), obs.get("reward", 0.0)], dtype=np.float32),
        ]
        if self.max_bullets is None:
            return np.concatenate([np.asarray(h, dtype=np.float32).reshape(-1) for h in head])
        bullets = np.asarray(obs.get("bullets", np.zeros((0, BULLET_OBS_DIM))), dtype=np.float32)
        pad = np.zeros((self.max_bullets, BULLET_OBS_DIM), dtype=np.float32)
        n = min(self.max_bullets, bullets.shape[0])
        pad[:n] = bullets[:n]
        head.append(pad.reshape(-1))
        head.append(np.array([n], dtype=np.float32))
        return np.concatenate([np.asarray(h, dtype=np.float32).reshape(-1) for h in head])

    def describe(self) -> dict:
        return {
            "normalize": self.normalize,
            "max_bullets": self.max_bullets,
            "flat_dim": self.flat_dim,
            "bullet_fields": list(BULLET_OBS_FIELDS),
        }


def observation_space_for(encoder: ObservationEncoder) -> dict:
    """Human/JSON-readable observation-space description."""
    return {
        "player": {"shape": [PLAYER_OBS_DIM], "dtype": "float32"},
        "target": {"shape": [TARGET_OBS_DIM], "dtype": "float32"},
        "env": {"shape": [ENV_OBS_DIM], "dtype": "float32"},
        "bullets": {
            "shape": ["n_alive", BULLET_OBS_DIM],
            "fields": list(BULLET_OBS_FIELDS),
            "dtype": "float32",
            "variable_length": True,
        },
        "flat": {"shape": [encoder.flat_dim], "dtype": "float32"},
    }
