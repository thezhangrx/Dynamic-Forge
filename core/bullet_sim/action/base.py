"""Action sources: the one seam between *input devices* and the simulator.

An :class:`ActionSource` converts whatever a device produces into an
:class:`~bullet_sim.action.types.Action`.  That is all it does:

* it may **not** import or touch ``bullet_sim.simulator`` - no reading or
  writing of environment state;
* it must be a pure producer: ``poll(dt)`` returns an ``Action`` (or ``None``
  when the device has nothing new for this frame);
* the caller (``BulletHellEnv.run`` / the CLI) is what feeds the action into
  ``World.step()``.

Implementations here cover software sources; the hardware board source lives in
``bullet_sim.hardware_interface`` and satisfies the same protocol, which is
exactly why mode A (manual) and mode B (board) share one environment loop.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Protocol, Sequence, runtime_checkable

import numpy as np

from bullet_sim.action.types import Action, coerce_action


@runtime_checkable
class ActionSource(Protocol):
    """Anything that can produce actions from some input device."""

    name: str

    def open(self) -> None: ...

    def close(self) -> None: ...

    def poll(self, dt: float) -> Action | None: ...


# --------------------------------------------------------------------------
# software sources
# --------------------------------------------------------------------------


class NullActionSource:
    """Always returns a zero action (baseline / idle camera)."""

    name = "null"

    def open(self) -> None:
        return None

    def close(self) -> None:
        return None

    def poll(self, dt: float) -> Action:
        return Action.zero()


class ScriptedSource:
    """Replays a fixed list of actions (ints, tuples or ``Action`` objects)."""

    name = "scripted"

    def __init__(
        self,
        actions: Sequence[Any],
        *,
        loop: bool = False,
        default: Action | None = None,
        step_period: int = 1,
    ) -> None:
        if not actions:
            raise ValueError("ScriptedSource requires a non-empty action sequence")
        self.actions = [coerce_action(a) for a in actions]
        self.loop = bool(loop)
        self.default = default or Action.zero()
        self.step_period = max(1, int(step_period))
        self._i = 0
        self._tick = 0

    def open(self) -> None:
        self._i = 0
        self._tick = 0

    close = open

    def poll(self, dt: float) -> Action:
        self._tick += 1
        if self._tick % self.step_period != 0:
            return self.actions[min(self._i, len(self.actions) - 1)]
        if self._i < len(self.actions):
            action = self.actions[self._i]
            self._i += 1
            return action
        if self.loop and self.actions:
            action = self.actions[self._i % len(self.actions)]
            self._i += 1
            return action
        return self.default

    @property
    def exhausted(self) -> bool:
        return self._i >= len(self.actions)


class ControllerSource:
    """Adapt any existing ``ActionProvider`` / callable into an ActionSource.

    This is how a rule-based controller, an imitation policy, an MPC solver or an
    NPU inference wrapper plugs in without the simulator knowing anything about
    it.  The provider still receives the observation dict; its raw output is
    converted to the unified :class:`Action`.
    """

    name = "controller"

    def __init__(self, provider: Any, *, as_discrete: bool = True) -> None:
        from bullet_sim.interface.controller import controller_from

        self.provider = controller_from(provider)
        self.as_discrete = bool(as_discrete)

    def open(self) -> None:
        return None

    def close(self) -> None:
        return None

    def poll_observation(self, observation: dict, info: dict) -> Action:
        raw = self.provider.act(observation, info)
        return coerce_action(raw, as_discrete=self.as_discrete)

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Action:
        """``ActionProvider`` alias for :meth:`poll_observation`.

        Having both makes wrapping idempotent: ``ControllerSource`` is itself a
        valid provider, so passing an already-wrapped source to
        ``env.run(controller=...)`` cannot double-wrap or fail.
        """
        return self.poll_observation(dict(observation), dict(info))

    def poll(self, dt: float) -> Action | None:
        # A controller needs the observation, so the loop calls
        # ``poll_observation`` instead; ``poll`` exists to satisfy the protocol.
        return None

    def reset(self, observation: dict, info: dict) -> None:
        reset = getattr(self.provider, "reset", None)
        if callable(reset):
            reset(observation, info)


class IterableSource:
    """Replay of a recorded dataset's ``actions`` array."""

    name = "dataset"

    def __init__(self, actions: Iterable[Any]) -> None:
        self._inner = ScriptedSource(list(actions))

    def open(self) -> None:
        self._inner.open()

    def close(self) -> None:
        self._inner.close()

    def poll(self, dt: float) -> Action:
        return self._inner.poll(dt)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


#: Time for a full-deflection action to reach the player's top speed when the
#: world is driven by the *acceleration* codec.  Documented constant rather than
#: a hidden magic number.
FULL_DEFLECTION_SECONDS = 0.25


def action_to_codec_input(
    action: Action,
    codec: Any,
    player_speed: float,
    *,
    discrete_threshold: float = 0.5,
) -> Any:
    """Convert a unified :class:`Action` into whatever ``codec`` expects.

    This is the compatibility seam: the input layer always produces ``Action``,
    while a world may still be configured with a discrete / velocity /
    acceleration codec.  The mapping is explicit and documented:

    * ``action`` codec        -> the ``Action`` itself
    * ``discrete`` codec      -> nearest index if ``magnitude >= threshold``
    * ``velocity`` codec      -> ``direction * magnitude * player_speed``
    * ``acceleration`` codec  -> full deflection reaches top speed in
      :data:`FULL_DEFLECTION_SECONDS`
    """
    from bullet_sim.core.actions import (
        AccelerationActionCodec,
        ActionObjectCodec,
        DiscreteActionCodec,
        VelocityActionCodec,
    )

    if isinstance(codec, ActionObjectCodec):
        return action
    if isinstance(codec, DiscreteActionCodec):
        return action.as_discrete() if action.magnitude >= discrete_threshold else 0
    if isinstance(codec, AccelerationActionCodec):
        accel = float(player_speed) / FULL_DEFLECTION_SECONDS
        return np.asarray(action.direction, dtype=np.float64) * action.magnitude * accel
    if isinstance(codec, VelocityActionCodec):
        return action.to_velocity(float(player_speed))
    return action.to_velocity(float(player_speed))


def _filter_kwargs(cls: Any, kwargs: Mapping[str, Any]) -> dict[str, Any]:
    """Drop kwargs a source's constructor does not accept (e.g. a stray ``seed``).

    ``source_from`` is called with a common bag of options from the CLI/env, and
    a ``ManualInputSource`` has no use for ``seed``; passing it through would be a
    confusing TypeError rather than a working default.
    """
    import inspect

    try:
        params = inspect.signature(cls).parameters
    except (TypeError, ValueError):  # pragma: no cover - builtins
        return dict(kwargs)
    if any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return dict(kwargs)
    return {k: v for k, v in kwargs.items() if k in params}


def source_from(spec: Any, **kwargs: Any) -> ActionSource:
    """Build an ActionSource from a name / config mapping / instance.

    Recognised names: ``null``, ``keyboard``, ``manual``, ``random``,
    ``scripted``, ``board`` (hardware - see ``hardware_interface``).
    """
    from bullet_sim.action.manual import ManualInputSource

    if spec is None:
        return NullActionSource()
    if isinstance(spec, ActionSource):
        return spec
    if isinstance(spec, str):
        key = spec.strip().lower()
        if key in ("null", "none", "stay"):
            return NullActionSource()
        if key in ("keyboard", "manual", "human"):
            return ManualInputSource(**_filter_kwargs(ManualInputSource, kwargs))
        if key == "random":
            from bullet_sim.action.manual import RandomActionSource

            return RandomActionSource(**_filter_kwargs(RandomActionSource, kwargs))
        if key == "scripted":
            params = {k: v for k, v in kwargs.items() if k != "actions"}
            return ScriptedSource(kwargs["actions"], **params)
        if key in ("auto", "ai", "autonomous", "baseline"):
            # mode B: an autonomous baseline controller (observation-only unless
            # 'planner' is selected, which needs explicit world access).
            from bullet_sim.ai.baseline import make_controller

            name = kwargs.get("auto") or kwargs.get("controller") or "threat"
            world = kwargs.get("world")
            return ControllerSource(make_controller(str(name), world=world))
        if key in ("model", "policy", "trained"):
            from bullet_sim.ai.policy import TrainedPolicyController, load_policy

            policy = load_policy(kwargs.get("model"))
            return ControllerSource(TrainedPolicyController(policy))
        if key in ("board", "hardware", "fpga"):
            from bullet_sim.hardware_interface import make_hardware_input_source

            return make_hardware_input_source(**kwargs)
        raise ValueError(
            f"unknown action source {spec!r}; try manual|auto|model|random|"
            "scripted|null|board|controller"
        )
    if isinstance(spec, dict):
        params = dict(spec)
        kind = params.pop("kind", params.pop("name", "null"))
        return source_from(kind, **params)
    if callable(spec):
        return ControllerSource(spec)
    raise ValueError(f"cannot build an ActionSource from {spec!r}")


@dataclass
class SourceStats:
    """Simple poll/latency bookkeeping shared by all sources."""

    polls: int = 0
    changes: int = 0
    last_action: Action = field(default_factory=Action.zero)
    source_time_s: float = 0.0

    def record(self, action: Action, elapsed: float) -> None:
        self.polls += 1
        if action != self.last_action:
            self.changes += 1
        self.last_action = action
        self.source_time_s += elapsed

    def summary(self) -> dict[str, Any]:
        return {
            "polls": self.polls,
            "changes": self.changes,
            "source_time_ms": self.source_time_s * 1e3,
            "last_action": self.last_action.to_dict(),
        }


class TimedSource:
    """Wrap a source and record how long each poll takes."""

    def __init__(self, inner: ActionSource) -> None:
        self.inner = inner
        self.name = f"timed:{getattr(inner, 'name', 'source')}"
        self.stats = SourceStats()

    def open(self) -> None:
        self.inner.open()

    def close(self) -> None:
        self.inner.close()

    def poll(self, dt: float) -> Action | None:
        t0 = time.perf_counter()
        action = self.inner.poll(dt)
        self.stats.record(action or Action.zero(), time.perf_counter() - t0)
        return action


__all__ = [
    "ActionSource",
    "NullActionSource",
    "ScriptedSource",
    "ControllerSource",
    "IterableSource",
    "TimedSource",
    "SourceStats",
    "coerce_action",
    "source_from",
    "action_to_codec_input",
    "FULL_DEFLECTION_SECONDS",
]
