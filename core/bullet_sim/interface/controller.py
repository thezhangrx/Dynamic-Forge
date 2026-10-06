"""External controller interface.

Any decision maker - rule-based, supervised, imitation, RL policy, MPC, or a
CPU/FPGA module - implements :class:`ActionProvider` and can drive the
simulator without the simulator knowing anything about it.

    class MyController:
        def reset(self, observation, info): ...
        def act(self, observation, info): return 3   # discrete 'up'
"""

from __future__ import annotations

from typing import Any, Callable, Mapping, Protocol, Sequence, runtime_checkable

import numpy as np

from bullet_sim.interface.space import Discrete


@runtime_checkable
class ActionProvider(Protocol):
    """Minimal controller contract."""

    def reset(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> None: ...

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Any: ...


class CallableController:
    """Adapt a plain ``fn(observation, info) -> action`` callable."""

    name = "callable"

    def __init__(self, fn: Callable[[Mapping[str, Any], Mapping[str, Any]], Any]) -> None:
        self._fn = fn

    def reset(self, observation, info) -> None:
        return None

    def act(self, observation, info) -> Any:
        return self._fn(observation, info)


class ConstantController:
    """Always returns the same action (useful baselines and benchmarks)."""

    name = "constant"

    def __init__(self, action: Any = 0) -> None:
        self.action = action

    def reset(self, observation, info) -> None:
        return None

    def act(self, observation, info) -> Any:
        return self.action


class RandomController:
    """Uniformly random policy over the action space (seeded, reproducible)."""

    name = "random"

    def __init__(self, space: Discrete | Any, seed: int = 0) -> None:
        self.space = space
        self.seed = int(seed)
        self._rng = np.random.Generator(np.random.PCG64(self.seed))

    def reset(self, observation, info) -> None:
        self._rng = np.random.Generator(np.random.PCG64(self.seed))

    def act(self, observation, info) -> Any:
        if isinstance(self.space, Discrete):
            return int(self._rng.integers(0, self.space.n))
        return self.space.sample(self._rng)


class ScriptedController:
    """Replays a fixed action sequence (used by replays and regression tests)."""

    name = "scripted"

    def __init__(self, actions: Sequence[Any], loop: bool = False, default: Any = 0) -> None:
        if not actions:
            raise ValueError("ScriptedController requires a non-empty action sequence")
        self.actions = list(actions)
        self.loop = bool(loop)
        self.default = default
        self._i = 0

    def reset(self, observation, info) -> None:
        self._i = 0

    def act(self, observation, info) -> Any:
        if self._i < len(self.actions):
            a = self.actions[self._i]
            self._i += 1
            return a
        if self.loop:
            a = self.actions[self._i % len(self.actions)]
            self._i += 1
            return a
        return self.default

    @property
    def exhausted(self) -> bool:
        return self._i >= len(self.actions)


class PolicyAdapter:
    """Adapt objects exposing ``predict``/``forward``/``__call__`` to a provider."""

    name = "policy"

    def __init__(self, policy: Any, method: str | None = None, deterministic: bool = True) -> None:
        self.policy = policy
        self.method = method
        self.deterministic = deterministic

    def reset(self, observation, info) -> None:
        inner = getattr(self.policy, "reset", None)
        if callable(inner):
            inner()

    def act(self, observation, info) -> Any:
        if self.method:
            fn = getattr(self.policy, self.method)
        elif hasattr(self.policy, "predict"):
            fn = self.policy.predict
        elif hasattr(self.policy, "act"):
            fn = self.policy.act
        else:
            fn = self.policy
        try:
            out = fn(observation, deterministic=self.deterministic)
        except TypeError:
            try:
                out = fn(observation)
            except TypeError:
                out = fn(np.asarray(observation, dtype=np.float32))
        if isinstance(out, tuple):
            out = out[0]
        return out


def controller_from(spec: Any, *, space: Any = None, seed: int = 0) -> ActionProvider:
    """Build a controller from a config entry.

    Accepted forms: an ``ActionProvider`` instance, a callable, ``"random"``,
    ``("constant", action)``, or ``{"kind": "scripted", "actions": [...]}``.
    """
    if spec is None:
        return ConstantController(0)
    if isinstance(spec, ActionProvider):
        return spec
    if callable(spec) and not isinstance(spec, type):
        return CallableController(spec)
    if isinstance(spec, str):
        text = spec.strip()
        if text.startswith("constant:"):
            return ConstantController(int(text.split(":", 1)[1]))
        if text.startswith("stay"):
            return ConstantController(0)
        if text == "random":
            return RandomController(space, seed=seed)
        raise ValueError(f"unknown controller spec {spec!r}")
    if isinstance(spec, Mapping):
        params = dict(spec)
        kind = params.pop("kind", "constant")
        if kind == "constant":
            return ConstantController(params.get("action", 0))
        if kind == "random":
            return RandomController(space, seed=int(params.get("seed", seed)))
        if kind == "scripted":
            return ScriptedController(
                params["actions"], loop=bool(params.get("loop", False)),
                default=params.get("default", 0),
            )
        if kind == "policy":
            return PolicyAdapter(params["policy"], method=params.get("method"))
        raise ValueError(f"unknown controller kind {kind!r}")
    raise ValueError(f"cannot build a controller from {spec!r}")


__all__ = [
    "ActionProvider",
    "CallableController",
    "ConstantController",
    "RandomController",
    "ScriptedController",
    "PolicyAdapter",
    "controller_from",
]
