"""Action layer: the single seam between input devices and the simulator.

    Input Device -> Action -> Simulator

* :mod:`bullet_sim.action.types`  - the ``Action { direction, magnitude }`` type
* :mod:`bullet_sim.action.base`   - the ``ActionSource`` protocol + software sources
* :mod:`bullet_sim.action.manual` - human input (keyboard / analog), mode A

Hardware (development-board) input lives in :mod:`bullet_sim.hardware_interface`
and implements the same ``ActionSource`` protocol, which is what makes mode A
and mode B share one environment loop.
"""

from bullet_sim.action.base import (
    ActionSource,
    ControllerSource,
    IterableSource,
    NullActionSource,
    ScriptedSource,
    SourceStats,
    TimedSource,
    coerce_action,
    source_from,
)
from bullet_sim.action.manual import (
    DEFAULT_BINDINGS,
    DEFAULT_MODIFIERS,
    ManualInputSource,
    RandomActionSource,
    keyboard_action_from_keys,
)
from bullet_sim.action.switch import SwitchEvent, SwitchableSource
from bullet_sim.action.types import Action, combine

__all__ = [
    "Action",
    "combine",
    "ActionSource",
    "NullActionSource",
    "ScriptedSource",
    "ControllerSource",
    "IterableSource",
    "TimedSource",
    "SourceStats",
    "coerce_action",
    "source_from",
    "ManualInputSource",
    "RandomActionSource",
    "DEFAULT_BINDINGS",
    "DEFAULT_MODIFIERS",
    "keyboard_action_from_keys",
    "SwitchableSource",
    "SwitchEvent",
]
