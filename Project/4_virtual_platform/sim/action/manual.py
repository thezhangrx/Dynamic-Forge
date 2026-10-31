"""Manual (human) input: ``Input Device -> Action``.

Responsibilities of this module, and nothing more:

1. hold a **key-name -> Action** binding table,
2. accept ``press`` / ``release`` / ``set_axis`` events from whatever device
   backend exists (pygame today, evdev / a web pad / a socket tomorrow),
3. compose the currently held keys into one :class:`Action` per frame.

It never imports the simulator, never reads or writes environment state, and
never decides what the avatar does - it only says "the player is holding up and
right at full deflection".

Diagonal movement falls out of composition rather than being a separate key, so
holding ``up`` + ``right`` gives the same speed as either alone (``sqrt(2)``
normalisation is handled by :func:`~bullet_sim.action.types.combine`).

Key names are device-neutral strings (``"up"``, ``"w"``, ``"space"``); the
pygame backend in ``render/pygame_view.py`` maps ``pygame.K_*`` to them, which
is what keeps this module free of any rendering dependency.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

import numpy as np

from bullet_sim.action.base import SourceStats
from bullet_sim.action.types import Action, combine

#: Default bindings.  Several keys may map to the same action; the canonical 9
#: directions are covered by the arrow keys and by WASD.
DEFAULT_BINDINGS: dict[str, Action] = {
    # arrows
    "up": Action.from_discrete("up"),
    "down": Action.from_discrete("down"),
    "left": Action.from_discrete("left"),
    "right": Action.from_discrete("right"),
    # WASD
    "w": Action.from_discrete("up"),
    "s": Action.from_discrete("down"),
    "a": Action.from_discrete("left"),
    "d": Action.from_discrete("right"),
    # explicit stop
    "space": Action.zero(),
    "0": Action.zero(),
}
# NOTE: "shift"/"ctrl" are *modifiers* (see DEFAULT_MODIFIERS), not bindings:
# a focus key scales the magnitude instead of introducing a new direction.

#: Keys that scale the magnitude instead of contributing a direction.
DEFAULT_MODIFIERS: dict[str, float] = {
    "shift": 0.4,   # focus / slow mode (Touhou-style)
    "ctrl": 0.25,   # very slow, precise positioning
}


@dataclass
class ManualInputSource:
    """Compose held keys into a single ``Action`` every poll."""

    name: str = "keyboard"
    bindings: Mapping[str, Action] = field(default_factory=lambda: dict(DEFAULT_BINDINGS))
    modifiers: Mapping[str, float] = field(default_factory=lambda: dict(DEFAULT_MODIFIERS))
    base_magnitude: float = 1.0
    #: When True, ``poll`` returns the current action even if unchanged.
    emit_when_unchanged: bool = True

    _pressed: set[str] = field(default_factory=set, init=False, repr=False)
    _axis: tuple[float, float] = field(default=(0.0, 0.0), init=False, repr=False)
    _axis_active: bool = field(default=False, init=False, repr=False)
    stats: SourceStats = field(default_factory=SourceStats, init=False)

    # ------------------------------------------------------------------
    # device callbacks (called by the renderer / device backend)
    # ------------------------------------------------------------------
    def press(self, key: str) -> bool:
        """Register a key-down.  Returns True if the binding was recognised."""
        name = str(key).lower()
        self._pressed.add(name)
        return name in self.bindings or name in self.modifiers

    def release(self, key: str) -> bool:
        name = str(key).lower()
        had = name in self._pressed
        self._pressed.discard(name)
        return had

    def set_axis(self, x: float, y: float, *, deadzone: float = 0.08) -> None:
        """Analog stick / trigger input. ``(0, 0)`` releases analog control."""
        ax, ay = float(x), float(y)
        if float(np.hypot(ax, ay)) <= deadzone:
            self._axis, self._axis_active = (0.0, 0.0), False
            return
        self._axis, self._axis_active = (ax, ay), True

    def clear(self) -> None:
        self._pressed.clear()
        self._axis, self._axis_active = (0.0, 0.0), False

    def is_pressed(self, key: str) -> bool:
        return str(key).lower() in self._pressed

    @property
    def pressed_keys(self) -> frozenset[str]:
        return frozenset(self._pressed)

    # ------------------------------------------------------------------
    # composition
    # ------------------------------------------------------------------
    def current_action(self) -> Action:
        """The action implied by the current device state.

        A key bound to :meth:`Action.zero` (SPACE / ``0``) is an **explicit
        stop**: it overrides any simultaneously held direction.  Otherwise it
        would be silently ignored by :func:`combine`, and "press SPACE to stop"
        would be a lie.
        """
        if self._explicit_stop_pressed():
            return Action.zero()
        magnitude = self.base_magnitude
        for key in self._pressed:
            if key in self.modifiers:
                magnitude = min(magnitude, float(self.modifiers[key]))

        action = combine(*(self._action_for(k) for k in self._pressed)).scaled(magnitude)
        if self._axis_active:
            analog = Action.from_vector(self._axis)
            # analog input wins when it is non-trivial, otherwise keep the keys
            if not analog.is_zero:
                action = analog.scaled(magnitude)
        return action

    def _explicit_stop_pressed(self) -> bool:
        for key in self._pressed:
            bound = self.bindings.get(key)
            if bound is not None and bound.is_zero:
                return True
        return False

    def _action_for(self, key: str) -> Action:
        bound = self.bindings.get(key)
        if bound is not None:
            return bound
        return Action.zero()

    # ------------------------------------------------------------------
    # ActionSource protocol
    # ------------------------------------------------------------------
    def open(self) -> None:
        """Attach to the device.

        Deliberately does **not** clear the pressed-key set: a physical key that
        is being held is genuine device state that outlives an episode boundary,
        and clearing it would fabricate a key release the user never performed.
        Use :meth:`clear` explicitly (e.g. when the window loses focus).
        """
        return None

    def close(self) -> None:
        """Detach from the device and forget held keys."""
        self.clear()

    def poll(self, dt: float) -> Action | None:
        action = self.current_action()
        if not self.emit_when_unchanged and action == self.stats.last_action:
            return None
        self.stats.polls += 1
        if action != self.stats.last_action:
            self.stats.changes += 1
        self.stats.last_action = action
        return action

    # ------------------------------------------------------------------
    def set_binding(self, key: str, action: Any) -> None:
        from bullet_sim.action.base import coerce_action

        self.bindings = {**self.bindings, str(key).lower(): coerce_action(action)}

    def describe(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "bindings": {k: v.to_dict() for k, v in sorted(self.bindings.items())},
            "modifiers": dict(self.modifiers),
            "base_magnitude": self.base_magnitude,
            "pressed": sorted(self._pressed),
            "axis": list(self._axis),
        }


class RandomActionSource:
    """Seeded random discrete actions (reproducible baselines)."""

    name = "random"

    def __init__(self, seed: int = 0, n_actions: int = 9, period: int = 1) -> None:
        self.seed = int(seed)
        self.n_actions = int(n_actions)
        self.period = max(1, int(period))
        self._rng = np.random.Generator(np.random.PCG64(self.seed))
        self._current = Action.zero()
        self._tick = 0

    def open(self) -> None:
        self._rng = np.random.Generator(np.random.PCG64(self.seed))
        self._tick = 0
        self._current = Action.zero()

    close = open

    def poll(self, dt: float) -> Action:
        if self._tick % self.period == 0:
            self._current = Action.from_discrete(int(self._rng.integers(0, self.n_actions)))
        self._tick += 1
        return self._current


def keyboard_action_from_keys(keys: Iterable[str], source: ManualInputSource | None = None) -> Action:
    """Pure helper: compose an action from a set of key names.

    Exposed so tests and non-pygame backends can reuse the exact mapping the
    interactive player uses.
    """
    src = source or ManualInputSource()
    src.clear()
    for key in keys:
        src.press(key)
    return src.current_action()


__all__ = [
    "ManualInputSource",
    "RandomActionSource",
    "DEFAULT_BINDINGS",
    "DEFAULT_MODIFIERS",
    "keyboard_action_from_keys",
]
