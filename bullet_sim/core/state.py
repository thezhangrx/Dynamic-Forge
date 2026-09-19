"""The simulator state S_t and its environment-level metadata.

    S_t = { player_state, bullet_states, target_state, environment_state, timestamp }

The state is *not* a UI structure: it has lossless ``to_dict``/``from_dict``,
a canonical ``state_hash`` for reproducibility assertions, and a flat fixed
stride representation for the CPU/FPGA protocol (see ``interface/protocol.py``).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Dict, Mapping, Optional

import numpy as np

from bullet_sim.core.errors import StateError
from bullet_sim.core.numerics import INT_DTYPE, canonical_hash, float_dtype
from bullet_sim.entities.bullet import PROTOCOL_FIELDS, BulletPool
from bullet_sim.entities.player import PlayerState
from bullet_sim.entities.target import TargetState

ENV_DIM = 7
ENV_FIELDS: tuple[str, ...] = (
    "field_w",
    "field_h",
    "dt",
    "step_index",
    "sim_time",
    "wrap",
    "bullet_budget",
)

#: Boundary handling for bullets leaving the field.
WRAP_MODES: tuple[str, ...] = ("cull", "wrap", "bounce")


@dataclass(frozen=True)
class EnvState:
    """Environment-level (non-entity) state."""

    field_w: float = 640.0
    field_h: float = 480.0
    dt: float = 1.0 / 120.0
    step_index: int = 0
    sim_time: float = 0.0
    wrap: str = "cull"
    bullet_budget: int = 0

    def to_array(self, dtype: Any = None) -> np.ndarray:
        return np.array(
            [
                self.field_w,
                self.field_h,
                self.dt,
                float(self.step_index),
                self.sim_time,
                float(WRAP_MODES.index(self.wrap)) if self.wrap in WRAP_MODES else 0.0,
                float(self.bullet_budget),
            ],
            dtype=float_dtype(dtype),
        )

    @classmethod
    def from_array(cls, arr: Any) -> "EnvState":
        a = np.asarray(arr, dtype=np.float64).reshape(-1)
        wi = int(round(float(a[5])))
        return cls(
            field_w=float(a[0]),
            field_h=float(a[1]),
            dt=float(a[2]),
            step_index=int(round(float(a[3]))),
            sim_time=float(a[4]),
            wrap=WRAP_MODES[wi] if 0 <= wi < len(WRAP_MODES) else "cull",
            bullet_budget=int(round(float(a[6]))),
        )

    def to_dict(self) -> dict:
        d = {
            "field_w": self.field_w,
            "field_h": self.field_h,
            "dt": self.dt,
            "step_index": self.step_index,
            "sim_time": self.sim_time,
            "wrap": self.wrap,
            "bullet_budget": self.bullet_budget,
        }
        return d

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "EnvState":
        return cls(
            field_w=float(payload.get("field_w", 640.0)),
            field_h=float(payload.get("field_h", 480.0)),
            dt=float(payload.get("dt", 1.0 / 120.0)),
            step_index=int(payload.get("step_index", 0)),
            sim_time=float(payload.get("sim_time", 0.0)),
            wrap=str(payload.get("wrap", "cull")),
            bullet_budget=int(payload.get("bullet_budget", 0)),
        )


@dataclass
class WorldSnapshot:
    """Complete, self-contained, serializable simulator state."""

    player: PlayerState
    bullets: BulletPool
    target: TargetState
    env: EnvState
    scenario_id: str = ""
    seed: int = 0
    timestamp: float = 0.0
    rng_state: Optional[Dict[str, Any]] = field(default=None, repr=False)

    # -- convenience -------------------------------------------------------
    @property
    def bullet_count(self) -> int:
        return int(self.bullets.count)

    @property
    def step_index(self) -> int:
        return int(self.env.step_index)

    def player_array(self, dtype: Any = None) -> np.ndarray:
        return self.player.to_array(dtype=dtype)

    def target_array(self, dtype: Any = None) -> np.ndarray:
        return self.target.to_array(dtype=dtype)

    def env_array(self, dtype: Any = None) -> np.ndarray:
        return self.env.to_array(dtype=dtype)

    # -- copying -----------------------------------------------------------
    def copy(self) -> "WorldSnapshot":
        """Deep copy: mutating the copy never affects the original."""
        return WorldSnapshot(
            player=replace(self.player),
            bullets=self.bullets.copy(),
            target=replace(self.target),
            env=replace(self.env),
            scenario_id=self.scenario_id,
            seed=self.seed,
            timestamp=self.timestamp,
            rng_state=(
                None
                if self.rng_state is None
                else {
                    k: (dict(v) if isinstance(v, dict) else v)
                    for k, v in self.rng_state.items()
                }
            ),
        )

    # -- hashing -----------------------------------------------------------
    def state_hash(self) -> str:
        """Canonical digest of the *simulation-relevant* state.

        Only live bullets are hashed, in ascending slot order, so that the
        digest is invariant to dead-slot bookkeeping and to the pool capacity.
        """
        b = self.bullets
        idx = b.active_indices()
        parts: list[tuple[np.ndarray, np.dtype]] = [
            (np.array([self.env.step_index], dtype=INT_DTYPE), INT_DTYPE),
            (np.array([self.timestamp, self.env.sim_time], dtype=np.float64), np.dtype(np.float64)),
            (self.player_array(np.float64), np.dtype(np.float64)),
            (self.target_array(np.float64), np.dtype(np.float64)),
            (self.env_array(np.float64), np.dtype(np.float64)),
            (np.array([idx.size], dtype=INT_DTYPE), INT_DTYPE),
        ]
        if idx.size:
            mat = np.empty((idx.size, len(PROTOCOL_FIELDS)), dtype=np.float64)
            for j, name in enumerate(PROTOCOL_FIELDS):
                mat[:, j] = b.data[name][idx]
            parts.append((mat, np.dtype(np.float64)))
        return canonical_hash(parts)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, WorldSnapshot):
            return NotImplemented
        return self.state_hash() == other.state_hash()

    def __ne__(self, other: object) -> bool:
        eq = self.__eq__(other)
        return NotImplemented if eq is NotImplemented else not eq

    # -- serialization -----------------------------------------------------
    def to_dict(self, include_bullets: bool = True) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "timestamp": self.timestamp,
            "step_index": self.env.step_index,
            "player": self.player.to_dict(),
            "target": self.target.to_dict(),
            "env": self.env.to_dict(),
            "bullet_count": self.bullet_count,
        }
        if include_bullets:
            d["bullets"] = self.bullets.to_dict(compact=True)
        return d

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "WorldSnapshot":
        try:
            bullets = BulletPool.from_dict(payload["bullets"]) if "bullets" in payload else BulletPool(0)
            env = EnvState.from_dict(payload.get("env", {}))
            return cls(
                player=PlayerState.from_dict(payload.get("player", {})),
                bullets=bullets,
                target=TargetState.from_dict(payload.get("target", {})),
                env=env,
                scenario_id=str(payload.get("scenario_id", "")),
                seed=int(payload.get("seed", 0)),
                timestamp=float(payload.get("timestamp", env.sim_time)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise StateError(f"malformed snapshot payload: {exc}") from exc

    # -- constructors ------------------------------------------------------
    @classmethod
    def initial(
        cls,
        *,
        field_w: float,
        field_h: float,
        dt: float,
        player: PlayerState | None = None,
        target: TargetState | None = None,
        bullet_capacity: int = 0,
        wrap: str = "cull",
        bullet_budget: int = 0,
        scenario_id: str = "",
        seed: int = 0,
        dtype: Any = None,
    ) -> "WorldSnapshot":
        env = EnvState(
            field_w=float(field_w),
            field_h=float(field_h),
            dt=float(dt),
            step_index=0,
            sim_time=0.0,
            wrap=wrap,
            bullet_budget=int(bullet_budget),
        )
        return cls(
            player=player or PlayerState(x=field_w * 0.5, y=field_h * 0.5),
            bullets=BulletPool(int(bullet_capacity), dtype=dtype),
            target=target or TargetState(x=field_w * 0.5, y=field_h * 0.5),
            env=env,
            scenario_id=scenario_id,
            seed=int(seed),
            timestamp=0.0,
        )
