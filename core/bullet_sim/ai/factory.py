"""Build the input source for a run: manual / autonomous / model / board.

One factory, used by both ``python -m bullet_sim play`` (windowed) and
``python -m bullet_sim run`` (headless), so the two paths can never drift apart:

============  ==================================================================
``manual``    ``ManualInputSource`` (mode A - keyboard)
``auto``      an autonomous baseline controller (mode B - no human)
``model``     a *trained* policy, ``[MODEL_NOT_AVAILABLE_YET]`` until one exists
``random``    seeded random walk
``scripted``  a recorded/scripted action sequence
``board``     development board, ``[HARDWARE_INTERFACE_TBD]``
``controller``an explicit ``--controller`` policy
============  ==================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from bullet_sim.action.base import ControllerSource, NullActionSource, source_from
from bullet_sim.action.manual import ManualInputSource
from bullet_sim.action.switch import SwitchableSource
from bullet_sim.ai.baseline import make_controller
from bullet_sim.ai.policy import MODEL_NOT_AVAILABLE_YET, TrainedPolicyController, load_policy

#: Modes that own a physical device and must be cleared when control leaves them.
MANUAL_MODES = ("manual", "keyboard")

MODE_ALIASES = {
    "keyboard": "manual",
    "human": "manual",
    "ai": "auto",
    "autonomous": "auto",
    "baseline": "auto",
    "policy": "model",
    "trained": "model",
    "harness": "board",
}


@dataclass
class PlaySource:
    """The resolved source plus what the UI needs to describe it."""

    source: Any
    mode: str
    controller: Any = None
    manual: Any = None
    note: str = ""

    @property
    def name(self) -> str:
        return getattr(self.source, "name", type(self.source).__name__)


def normalise_mode(mode: str) -> str:
    key = str(mode or "manual").strip().lower()
    return MODE_ALIASES.get(key, key)


def make_auto_source(controller_name: str = "threat", *, world: Any = None, seed: int = 0):
    """Autonomous baseline as an ``ActionSource`` plus the controller itself."""
    ctrl = make_controller(controller_name, world=world)
    return ControllerSource(ctrl), ctrl


def make_play_source(
    mode: str = "manual",
    *,
    env: Any = None,
    auto: str = "threat",
    model: str | None = None,
    model_name: str | None = None,
    controller: Any = None,
    actions: Any = None,
    seed: int = 0,
    switchable: bool = True,
) -> PlaySource:
    """Resolve ``mode`` into a ready-to-use ``ActionSource``.

    When ``switchable`` is set and both a manual source and an autonomous source
    are meaningful, the result is a :class:`SwitchableSource` so the player can
    hand control back and forth at runtime (TAB in the UI).
    """
    key = normalise_mode(mode)

    if key == "manual":
        manual = ManualInputSource()
        if not switchable:
            return PlaySource(manual, "manual", manual=manual)
        auto_source, ctrl = make_auto_source(auto, world=getattr(env, "world", None), seed=seed)
        sw = SwitchableSource(
            {"manual": manual, "auto": auto_source},
            manual_names=list(MANUAL_MODES),
            initial="manual",
        )
        return PlaySource(sw, "manual", controller=ctrl, manual=manual,
                          note=f"TAB switches to autonomous '{auto}'")

    if key == "auto":
        auto_source, ctrl = make_auto_source(auto, world=getattr(env, "world", None), seed=seed)
        if not switchable:
            return PlaySource(auto_source, "auto", controller=ctrl)
        manual = ManualInputSource()
        sw = SwitchableSource(
            {"auto": auto_source, "manual": manual},
            manual_names=list(MANUAL_MODES),
            initial="auto",
        )
        return PlaySource(sw, "auto", controller=ctrl, manual=manual,
                          note="TAB switches to manual keyboard control")

    if key == "model":
        policy = load_policy(model)
        ctrl = TrainedPolicyController(policy, name=model_name or "model")
        return PlaySource(ControllerSource(ctrl), "model", controller=ctrl,
                          note=MODEL_NOT_AVAILABLE_YET)

    if key == "controller":
        src = ControllerSource(controller)
        return PlaySource(src, "controller", controller=controller)

    if key in ("scripted",):
        if actions is None:
            raise ValueError("scripted mode needs an action sequence (--actions)")
        return PlaySource(source_from("scripted", actions=actions), "scripted")

    if key == "null":
        return PlaySource(NullActionSource(), "null")

    # random / board / anything else that source_from already understands
    src = source_from(key, seed=seed)
    ctrl = getattr(src, "provider", None)
    return PlaySource(src, key, controller=ctrl)


__all__ = [
    "PlaySource",
    "MANUAL_MODES",
    "MODE_ALIASES",
    "normalise_mode",
    "make_auto_source",
    "make_play_source",
]
