"""Unified action interface.

The world never inspects a raw action object: it asks an :class:`ActionCodec`
to translate *any* supported action representation into a desired world-frame
velocity ``(vx, vy)``.  This is the single seam that lets a policy, a rule
controller, an MPC solver, or a CPU/FPGA decision buffer drive the simulator
without touching kernel code.

Built-in codecs
---------------
``discrete``      9-way discrete directions (default, v1 action space)
``velocity``      direct 2D velocity command ``(vx, vy)``
``acceleration``  2D acceleration command integrated against current velocity
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.errors import ConfigError

_INV_SQRT2 = float(1.0 / np.sqrt(2.0))

#: Canonical discrete action table. Index == action id. Order is part of the
#: public protocol - never reorder, only append.
DISCRETE_ACTION_NAMES: tuple[str, ...] = (
    "stay",
    "left",
    "right",
    "up",
    "down",
    "up_left",
    "up_right",
    "down_left",
    "down_right",
)

DISCRETE_DIRECTIONS: np.ndarray = np.array(
    [
        (0.0, 0.0),
        (-1.0, 0.0),
        (1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
        (-_INV_SQRT2, _INV_SQRT2),
        (_INV_SQRT2, _INV_SQRT2),
        (-_INV_SQRT2, -_INV_SQRT2),
        (_INV_SQRT2, -_INV_SQRT2),
    ],
    dtype=np.float64,
)

#: Name -> index lookup (stable, used by configs and the CLI).
DISCRETE_ACTION_INDEX: Mapping[str, int] = {
    name: i for i, name in enumerate(DISCRETE_ACTION_NAMES)
}


@dataclass(frozen=True)
class ActionContext:
    """Everything a codec may need besides the raw action itself."""

    player_speed: float
    vx: float = 0.0
    vy: float = 0.0
    dt: float = 0.0
    max_speed: float = float("inf")


class ActionCodec:
    """Base class: maps an action to a desired world-frame velocity."""

    #: Dimensionality of the *raw* action vector (``0`` for discrete sets).
    dim: int = 0
    name: str = "base"
    #: Whether raw actions are integer indices (affects serialization).
    discrete: bool = False

    def encode(self, action: Any, ctx: ActionContext) -> np.ndarray:
        """Return ``float64[2]`` desired velocity. Must be side-effect free."""
        raise NotImplementedError

    def decode(self, vec: Sequence[float]) -> Any:
        """Best-effort inverse mapping of a velocity vector to a raw action."""
        raise NotImplementedError

    def normalize(self, action: Any) -> Any:
        """Validate/normalize a raw action (used by recorders and replays)."""
        raise NotImplementedError

    def describe(self) -> dict:
        return {"name": self.name, "dim": self.dim, "discrete": self.discrete}


class DiscreteActionCodec(ActionCodec):
    """9-way discrete movement; direction is scaled by the player's speed."""

    discrete = True

    def __init__(self, names: Sequence[str] = DISCRETE_ACTION_NAMES) -> None:
        self.names = tuple(names)
        if self.names != DISCRETE_ACTION_NAMES:
            raise ConfigError("custom discrete action tables are not supported yet")
        self.dim = len(self.names)
        self.name = "discrete"

    def encode(self, action: Any, ctx: ActionContext) -> np.ndarray:
        idx = self._index(action)
        return DISCRETE_DIRECTIONS[idx] * float(ctx.player_speed)

    def decode(self, vec: Sequence[float]) -> int:
        v = np.asarray(vec, dtype=np.float64).reshape(-1)
        if v.size != 2:
            raise ConfigError(f"expected a 2-vector, got shape {v.shape}")
        n = float(np.linalg.norm(v))
        if n < 1e-12:
            return DISCRETE_ACTION_INDEX["stay"]
        unit = v / n
        # Pick the table entry with maximal cosine similarity (deterministic
        # tie-break by lowest index).
        sims = DISCRETE_DIRECTIONS @ unit
        return int(np.argmax(sims))

    def normalize(self, action: Any) -> int:
        return self._index(action)

    def _index(self, action: Any) -> int:
        if isinstance(action, str):
            try:
                return DISCRETE_ACTION_INDEX[action]
            except KeyError as exc:  # pragma: no cover - defensive
                raise ConfigError(f"unknown discrete action name {action!r}") from exc
        arr = np.asarray(action)
        if arr.size != 1:
            raise ConfigError(
                f"discrete action must be a scalar index, got shape {arr.shape}"
            )
        idx = int(arr.reshape(-1)[0])
        if not (0 <= idx < self.dim):
            raise ConfigError(
                f"discrete action index {idx} out of range [0, {self.dim})"
            )
        return idx


class VelocityActionCodec(ActionCodec):
    """Direct world-frame velocity command, optionally speed-limited."""

    dim = 2
    name = "velocity"

    def __init__(self, limit_to_player_speed: bool = True) -> None:
        self.limit_to_player_speed = bool(limit_to_player_speed)

    def encode(self, action: Any, ctx: ActionContext) -> np.ndarray:
        v = np.asarray(action, dtype=np.float64).reshape(-1)
        if v.size != 2:
            raise ConfigError(f"velocity action must have 2 components, got {v.size}")
        lim = ctx.max_speed
        if self.limit_to_player_speed:
            lim = min(lim, float(ctx.player_speed))
        n = float(np.linalg.norm(v))
        if np.isfinite(lim) and n > lim > 0.0:
            v = v * (lim / n)
        return v

    def decode(self, vec: Sequence[float]) -> np.ndarray:
        return np.asarray(vec, dtype=np.float64).reshape(2)

    def normalize(self, action: Any) -> np.ndarray:
        v = np.asarray(action, dtype=np.float64).reshape(-1)
        if v.size != 2:
            raise ConfigError(f"velocity action must have 2 components, got {v.size}")
        return v


class AccelerationActionCodec(ActionCodec):
    """Acceleration command integrated against the current velocity.

    ``v_new = clamp(v + a * dt, max_speed)``.  Requires ``ctx.dt`` and the
    player's current velocity to be populated.
    """

    dim = 2
    name = "acceleration"

    def __init__(self, max_accel: float = float("inf")) -> None:
        self.max_accel = float(max_accel)

    def encode(self, action: Any, ctx: ActionContext) -> np.ndarray:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.size != 2:
            raise ConfigError(f"acceleration action must have 2 components, got {a.size}")
        n = float(np.linalg.norm(a))
        if np.isfinite(self.max_accel) and n > self.max_accel > 0.0:
            a = a * (self.max_accel / n)
        v = np.array([ctx.vx, ctx.vy], dtype=np.float64) + a * float(ctx.dt)
        lim = min(ctx.max_speed, float(ctx.player_speed))
        n = float(np.linalg.norm(v))
        if np.isfinite(lim) and n > lim > 0.0:
            v = v * (lim / n)
        return v

    def decode(self, vec: Sequence[float]) -> np.ndarray:
        return np.asarray(vec, dtype=np.float64).reshape(2)

    def normalize(self, action: Any) -> np.ndarray:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.size != 2:
            raise ConfigError(f"acceleration action must have 2 components, got {a.size}")
        return a


class ActionObjectCodec(ActionCodec):
    """Consume :class:`~bullet_sim.action.types.Action` objects directly.

    ``Action{ direction, magnitude }`` is the platform's unified action type;
    this codec is what lets it be handed to ``World.step`` unchanged.  It is
    opt-in (``codec="action"``) so that existing discrete/velocity codecs keep
    their exact semantics.

    The import of :mod:`bullet_sim.action.types` is lazy to keep the dependency
    one-directional (``action`` depends on ``core``, never the reverse).
    """

    dim = 2
    name = "action"

    def encode(self, action: Any, ctx: ActionContext) -> np.ndarray:
        from bullet_sim.action.types import Action, coerce_action  # local: avoid a cycle

        a = action if isinstance(action, Action) else coerce_action(action)
        return a.to_velocity(float(ctx.player_speed))

    def decode(self, vec: Sequence[float]) -> Any:
        from bullet_sim.action.types import Action

        return Action.from_vector(vec)

    def normalize(self, action: Any) -> Any:
        from bullet_sim.action.types import Action, coerce_action

        a = action if isinstance(action, Action) else coerce_action(action)
        return a.to_dict()


_CODECS = {
    "action": ActionObjectCodec,
    "discrete": DiscreteActionCodec,
    "velocity": VelocityActionCodec,
    "acceleration": AccelerationActionCodec,
}


def make_codec(kind: str = "discrete", **kwargs: Any) -> ActionCodec:
    """Factory used by configs / CLI (``kind`` in ``{'discrete','velocity','acceleration'}``)."""
    try:
        cls = _CODECS[kind]
    except KeyError as exc:
        raise ConfigError(
            f"unknown action codec {kind!r}; available: {sorted(_CODECS)}"
        ) from exc
    return cls(**kwargs)  # type: ignore[arg-type]


def codec_from_spec(spec: Any) -> ActionCodec:
    """Build a codec from a plain config mapping, e.g. ``{"kind": "velocity"}``."""
    if spec is None:
        return DiscreteActionCodec()
    if isinstance(spec, ActionCodec):
        return spec
    if isinstance(spec, str):
        return make_codec(spec)
    if isinstance(spec, Mapping):
        params = dict(spec)
        kind = params.pop("kind", "discrete")
        return make_codec(kind, **params)
    raise ConfigError(f"cannot build an ActionCodec from {spec!r}")
