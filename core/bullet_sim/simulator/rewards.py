"""Reward functions.

Rewards are **pluggable and out of the kernel's critical path**: the world only
calls ``reward(prev, curr, collision, info)``.  This keeps the platform usable
for supervised learning, imitation learning, RL, planning and pure simulation
without preferring any one of them.
"""

from __future__ import annotations

from typing import Any, Mapping, Protocol, runtime_checkable

from bullet_sim.collision.base import CollisionResult
from bullet_sim.core.state import WorldSnapshot


@runtime_checkable
class RewardFunction(Protocol):
    name: str

    def reset(self, snapshot: WorldSnapshot) -> None: ...

    def __call__(
        self,
        prev: WorldSnapshot,
        curr: WorldSnapshot,
        collision: CollisionResult,
        info: dict,
    ) -> float: ...


class ZeroReward:
    """No reward at all - pure dynamics / planning / dataset generation."""

    name = "zero"

    def reset(self, snapshot: WorldSnapshot) -> None:
        return None

    def __call__(self, prev, curr, collision, info) -> float:
        return 0.0


def collision_events(info: Mapping[str, Any], collision: Any) -> int:
    """How many **new** collision events this step contains.

    Falls back to "1 if overlapping" for callers that do not populate ``info``
    (custom harnesses), which is the old behaviour - but the kernel always
    provides the event count, so in normal use the penalty is charged once per
    event, never once per overlapping frame.
    """
    events = info.get("collision_events")
    if events is None:
        return 1 if getattr(collision, "hit", False) else 0
    return int(events)


class SurvivalReward:
    """Survival reward with a **per-collision-event** penalty.

        r_t = step_reward - collision_penalty * events_t + target_bonus - shaping

    ``collision_penalty`` is a positive magnitude (default 1.0) and is normally
    supplied from the scenario config, not from code.  Because it multiplies the
    *event* count, holding contact with one bullet for 40 frames costs one
    penalty, not forty.
    """

    name = "survival"

    def __init__(
        self,
        *,
        step_reward: float = 1.0,
        collision_penalty: float = 1.0,
        target_bonus: float = 0.0,
        proximity_weight: float = 0.0,
    ) -> None:
        self.step_reward = float(step_reward)
        self.collision_penalty = float(collision_penalty)
        self.target_bonus = float(target_bonus)
        self.proximity_weight = float(proximity_weight)
        self._in_target = False

    def reset(self, snapshot: WorldSnapshot) -> None:
        self._in_target = self._check_target(snapshot)

    @staticmethod
    def _check_target(snapshot: WorldSnapshot) -> bool:
        p = snapshot.player
        return snapshot.target.contains(p.x, p.y)

    def __call__(self, prev, curr, collision, info) -> float:
        r = self.step_reward
        r -= self.collision_penalty * collision_events(info, collision)
        in_target = self._check_target(curr)
        if self.target_bonus and in_target and not self._in_target:
            r += self.target_bonus
        self._in_target = in_target
        if self.proximity_weight:
            clearance = info.get("clearance")
            if clearance is not None:
                r -= self.proximity_weight * float(clearance)
        return float(r)


class CollisionOnlyReward:
    """``0`` on a clean step, ``-collision_penalty`` per collision event."""

    name = "collision_only"

    def __init__(self, collision_penalty: float = 1.0, **legacy: Any) -> None:
        # ``penalty=`` was the old keyword; keep accepting it.
        self.collision_penalty = float(legacy.pop("penalty", collision_penalty))

    def reset(self, snapshot: WorldSnapshot) -> None:
        return None

    def __call__(self, prev, curr, collision, info) -> float:
        return -self.collision_penalty * collision_events(info, collision)


_REWARDS = {
    "zero": ZeroReward,
    "survival": SurvivalReward,
    "collision_only": CollisionOnlyReward,
}


def _accepts(cls: Any, kwargs: Mapping[str, Any]) -> dict[str, Any]:
    """Drop kwargs the reward class does not declare.

    ``World`` injects the configured ``collision_penalty`` into every string-based
    reward, and ``ZeroReward`` has no parameters at all; filtering here keeps
    that injection harmless instead of a TypeError.
    """
    import inspect

    params = inspect.signature(cls).parameters
    if any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return dict(kwargs)
    return {k: v for k, v in kwargs.items() if k in params}


def make_reward(kind: str = "survival", **kwargs: Any) -> RewardFunction:
    if kind is None:
        return ZeroReward()
    if isinstance(kind, RewardFunction):
        return kind
    try:
        cls = _REWARDS[kind]
    except KeyError as exc:
        raise ValueError(
            f"unknown reward {kind!r}; available: {sorted(_REWARDS)}"
        ) from exc
    return cls(**_accepts(cls, kwargs))  # type: ignore[arg-type]
