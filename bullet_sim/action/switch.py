"""Runtime switching between input sources (Manual <-> Autonomous <-> ...).

The platform must let a组员 start in manual mode, then hand control to a
controller *without restarting* and without touching the simulator.  That is a
property of the input layer only:

    KeyboardSource ─┐
    ControllerSource├─► SwitchableSource ─► Action ─► Simulator
    HardwareSource ─┘        (toggled by TAB in the UI)

Switching changes *who produces the Action*.  It never mutates the world, so
the trajectory stays continuous across a switch and a recording made while
switching replays exactly (the switch is part of the action stream, not of the
physics).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from bullet_sim.action.base import ActionSource, SourceStats
from bullet_sim.action.types import Action


@dataclass
class SwitchEvent:
    """One transition, recorded so a session can be audited/replayed."""

    step: int
    from_mode: str
    to_mode: str


class SwitchableSource:
    """Route ``poll`` to one of several named sources; switch at runtime.

    Parameters
    ----------
    sources:
        ``name -> ActionSource``; the first entry is the initial mode.
    manual_names:
        Sources that own a physical device.  They are *cleared* when control
        leaves them, so a key held while switching does not keep steering the
        avatar after an autonomous controller takes over.
    """

    def __init__(
        self,
        sources: Mapping[str, ActionSource],
        *,
        manual_names: Sequence[str] = (),
        initial: str | None = None,
    ) -> None:
        if not sources:
            raise ValueError("SwitchableSource needs at least one source")
        self._sources: dict[str, ActionSource] = dict(sources)
        self._manual = set(manual_names)
        self._order = list(self._sources)
        self._current = initial or self._order[0]
        if self._current not in self._sources:
            raise ValueError(f"unknown initial mode {self._current!r}")
        self.stats = SourceStats()
        self.history: list[SwitchEvent] = []
        self.switch_count = 0
        self._closed = False

    # ------------------------------------------------------------------
    @property
    def name(self) -> str:
        """``switch:<current>`` so the HUD and datasets show the active source."""
        return f"switch:{self._current}"

    @property
    def mode(self) -> str:
        return self._current

    @property
    def modes(self) -> list[str]:
        return list(self._order)

    @property
    def active(self) -> ActionSource:
        return self._sources[self._current]

    def source_for(self, mode: str) -> ActionSource:
        try:
            return self._sources[mode]
        except KeyError as exc:
            raise ValueError(
                f"unknown input mode {mode!r}; available: {self._order}"
            ) from exc

    # ------------------------------------------------------------------
    def switch_to(self, mode: str, *, step: int = -1) -> bool:
        """Switch to ``mode``.  Returns True if the mode actually changed."""
        source = self.source_for(mode)
        if mode == self._current:
            return False
        previous = self._current
        if previous in self._manual:
            clear = getattr(self._sources[previous], "clear", None)
            if callable(clear):
                clear()
        self._current = mode
        self.switch_count += 1
        self.history.append(SwitchEvent(step=int(step), from_mode=previous, to_mode=mode))
        open_fn = getattr(source, "open", None)
        if callable(open_fn):
            open_fn()
        return True

    def toggle(self, *, step: int = -1) -> str:
        """Cycle to the next mode (what TAB does in the UI)."""
        if len(self._order) == 1:
            return self._current
        index = (self._order.index(self._current) + 1) % len(self._order)
        self.switch_to(self._order[index], step=step)
        return self._current

    # ------------------------------------------------------------------
    # device passthrough
    #
    # The UI delivers key events without knowing which mode is active, so a
    # SwitchableSource must present the same ``press``/``release``/``set_axis``
    # surface a device source does.  Events are routed to the **active** source
    # only: a key pressed while an autonomous controller is driving must not
    # queue up and take effect after switching back to manual.
    # ------------------------------------------------------------------
    def press(self, key: str) -> bool:
        fn = getattr(self.active, "press", None)
        return bool(fn(key)) if callable(fn) else False

    def release(self, key: str) -> bool:
        fn = getattr(self.active, "release", None)
        return bool(fn(key)) if callable(fn) else False

    def set_axis(self, x: float, y: float, **kwargs: Any) -> None:
        fn = getattr(self.active, "set_axis", None)
        if callable(fn):
            fn(x, y, **kwargs)

    def clear(self) -> None:
        """Release every held key, whatever mode is active."""
        for source in self._sources.values():
            clear = getattr(source, "clear", None)
            if callable(clear):
                clear()

    @property
    def pressed_keys(self) -> frozenset:
        keys = getattr(self.active, "pressed_keys", None)
        return frozenset(keys) if keys else frozenset()

    @property
    def supports_device_input(self) -> bool:
        return callable(getattr(self.active, "press", None))

    def open(self) -> None:
        for source in self._sources.values():
            opener = getattr(source, "open", None)
            if callable(opener):
                opener()
        self._closed = False

    def close(self) -> None:
        for source in self._sources.values():
            closer = getattr(source, "close", None)
            if callable(closer):
                closer()
        self._closed = True

    def poll(self, dt: float) -> Action | None:
        return self.active.poll(dt)

    def poll_observation(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Action:
        """Some sources (controllers, models) need the observation instead of dt."""
        source = self.active
        poller = getattr(source, "poll_observation", None)
        if callable(poller):
            return poller(observation, info)
        action = source.poll(float(info.get("dt") or 1.0 / 120.0))
        return action if action is not None else Action.zero()

    @property
    def needs_observation(self) -> bool:
        return callable(getattr(self.active, "poll_observation", None))

    # ------------------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        active = self.active
        return {
            "name": self.name,
            "mode": self._current,
            "modes": self._order,
            "manual_modes": sorted(self._manual),
            "switches": self.switch_count,
            "history": [
                {"step": e.step, "from": e.from_mode, "to": e.to_mode} for e in self.history[-10:]
            ],
            "active_source": getattr(active, "name", type(active).__name__),
            "stats": self.stats.summary(),
        }


__all__ = ["SwitchableSource", "SwitchEvent"]
