"""Development-board input: ``Board -> [TBD transport] -> Adapter -> Action``.

    Development Board
       -> [HARDWARE_INPUT_INTERFACE_TBD]        <- UART/SPI/USB/GPIO/TCP/MMIO: NOT decided
       -> HardwareInputAdapter                  <- this module (abstract)
       -> Action                                <- bullet_sim.action.types.Action
       -> Simulator                             <- unchanged environment logic

The adapter is the only place that will ever know the board protocol.  Until the
board is chosen, :class:`PlaceholderHardwareInputAdapter` refuses to produce
actions rather than inventing them, and
:class:`LoopbackHardwareInputAdapter` lets the *rest* of the pipeline
(``Adapter -> Action -> World.step``) be tested today without pretending to know
the wire format.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Deque, Sequence

from bullet_sim.action.base import SourceStats, coerce_action
from bullet_sim.action.types import Action
from bullet_sim.core.errors import BulletSimError
from bullet_sim.hardware_interface.tbd import (
    HARDWARE_INPUT_INTERFACE_TBD,
    ActionFrameFormat,
    PlannedTransport,
)


class HardwareInterfaceNotConfigured(BulletSimError):
    """Raised when board input is requested before the protocol is defined."""


# --------------------------------------------------------------------------
# abstract adapter
# --------------------------------------------------------------------------


class HardwareInputAdapter(ABC):
    """Abstract board -> Action adapter.

    Concrete implementations are written **after** the development board and its
    transport are chosen; nothing here presumes a wire format.
    """

    #: Human-readable transport name; stays at the TBD marker until decided.
    transport: str = HARDWARE_INPUT_INTERFACE_TBD

    def __init__(
        self,
        transport: PlannedTransport | None = None,
        frame_format: ActionFrameFormat | None = None,
    ) -> None:
        self.planned = transport or PlannedTransport()
        # A decided transport kind wins; otherwise keep the class's descriptive
        # label (which is the TBD marker for the unimplemented placeholder).
        self.transport = self.planned.kind or type(self).transport
        self.frame_format = frame_format or ActionFrameFormat()
        self.stats = SourceStats()
        self._open = False

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        self.planned.require("kind", "payload_format")
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    # -- data path ---------------------------------------------------------
    @abstractmethod
    def decode_payload(self, payload: Any) -> Action:
        """Convert one raw board payload into an :class:`Action`.

        Must be implemented once the frame format is known.  Until then,
        implementations should raise :class:`HardwareInterfaceNotConfigured`.
        """
        raise NotImplementedError

    @abstractmethod
    def read_payload(self) -> Any:
        """Read one raw payload from the transport (``None`` if nothing new)."""
        raise NotImplementedError

    def poll(self, dt: float) -> Action | None:
        """Read + decode.  Returns ``None`` when the board has nothing new."""
        t0 = time.perf_counter()
        payload = self.read_payload()
        if payload is None:
            return None
        action = self.decode_payload(payload)
        self.stats.record(action, time.perf_counter() - t0)
        return action

    # -- introspection -----------------------------------------------------
    def describe(self) -> dict[str, Any]:
        return {
            "adapter": type(self).__name__,
            "transport": self.transport,
            "open": self._open,
            "planned_transport": self.planned.describe(),
            "action_frame_format": self.frame_format.describe(),
            "stats": self.stats.summary(),
            "marker": HARDWARE_INPUT_INTERFACE_TBD,
        }


class PlaceholderHardwareInputAdapter(HardwareInputAdapter):
    """Refuses to invent a protocol.  Every operation explains what is missing."""

    transport = HARDWARE_INPUT_INTERFACE_TBD

    def decode_payload(self, payload: Any) -> Action:
        raise HardwareInterfaceNotConfigured(
            f"{HARDWARE_INPUT_INTERFACE_TBD} cannot decode a board payload: the "
            "transport, frame format, byte order and encoding are not decided yet. "
            "Implement HardwareInputAdapter.decode_payload for the chosen board."
        )

    def read_payload(self) -> Any:
        raise HardwareInterfaceNotConfigured(
            f"{HARDWARE_INPUT_INTERFACE_TBD} no transport is configured, so no board "
            "payload can be read. Choose the board link, fill PlannedTransport in "
            "hardware_interface/tbd.py, then implement read_payload()."
        )

    def open(self) -> None:
        raise HardwareInterfaceNotConfigured(
            f"{HARDWARE_INPUT_INTERFACE_TBD} board input is not configured yet"
        )

    def describe(self) -> dict[str, Any]:
        data = super().describe()
        data["status"] = "not configured - intentionally unimplemented"
        return data


# --------------------------------------------------------------------------
# test harnesses (NOT protocol guesses)
# --------------------------------------------------------------------------


class LoopbackHardwareInputAdapter(HardwareInputAdapter):
    """In-process harness: actions are pushed programmatically and polled back.

    This exists so the pipeline ``Adapter -> Action -> World.step`` can be
    exercised and unit-tested *before* real hardware exists.  It is deliberately
    **not** a model of the eventual board protocol.
    """

    transport = "loopback (test harness)"

    def __init__(self, actions: Sequence[Any] | None = None, **kwargs: Any) -> None:
        # NOTE: ``kind`` stays None - this harness deliberately does *not*
        # select a real transport.  Only the payload format is set, and only to
        # label the harness.
        kwargs.setdefault("transport", PlannedTransport(payload_format="loopback-harness"))
        super().__init__(**kwargs)
        self._queue: Deque[Any] = deque(actions or [])
        self._last: Action = Action.zero()
        self._hold = False

    def push(self, action: Any) -> None:
        self._queue.append(action)

    def hold(self, action: Any) -> None:
        """Repeat ``action`` forever instead of consuming the queue."""
        self._last = coerce_action(action)
        self._hold = True

    def decode_payload(self, payload: Any) -> Action:
        return coerce_action(payload)

    def read_payload(self) -> Any:
        if self._queue:
            payload = self._queue.popleft()
            self._last = coerce_action(payload)
            return payload
        return self._last if self._hold else None

    def open(self) -> None:
        self._open = True


class RecordedHardwareInputAdapter(HardwareInputAdapter):
    """Replay a recorded board session (numpy action array) for regression tests."""

    transport = "recorded (offline)"

    def __init__(self, actions: Sequence[Any], **kwargs: Any) -> None:
        kwargs.setdefault("transport", PlannedTransport(payload_format="recorded-actions"))
        super().__init__(**kwargs)
        self._actions = [coerce_action(a) for a in actions]
        self._i = 0

    def decode_payload(self, payload: Any) -> Action:
        return coerce_action(payload)

    def read_payload(self) -> Any:
        if self._i >= len(self._actions):
            return None
        item = self._actions[self._i]
        self._i += 1
        return item

    def open(self) -> None:
        self._i = 0
        self._open = True


# --------------------------------------------------------------------------
# ActionSource bridge
# --------------------------------------------------------------------------


class HardwareInputSource:
    """Expose a :class:`HardwareInputAdapter` as an ``ActionSource``.

    This is the single line that makes mode B (board) use the exact same
    ``-> Action -> Simulator`` path as mode A (manual).
    """

    def __init__(self, adapter: HardwareInputAdapter, *, fallback: Action | None = None) -> None:
        self.adapter = adapter
        self.fallback = fallback or Action.zero()
        self.name = f"board:{adapter.transport}"

    def open(self) -> None:
        self.adapter.open()

    def close(self) -> None:
        self.adapter.close()

    def poll(self, dt: float) -> Action | None:
        action = self.adapter.poll(dt)
        if action is None:
            return None
        return action

    def describe(self) -> dict[str, Any]:
        return {"name": self.name, "adapter": self.adapter.describe()}


def make_hardware_input_adapter(kind: str = "placeholder", **kwargs: Any) -> HardwareInputAdapter:
    """Factory: ``placeholder`` (default, refuses) | ``loopback`` | ``recorded``."""
    table = {
        "placeholder": PlaceholderHardwareInputAdapter,
        "tbd": PlaceholderHardwareInputAdapter,
        "loopback": LoopbackHardwareInputAdapter,
        "recorded": RecordedHardwareInputAdapter,
        "replay": RecordedHardwareInputAdapter,
    }
    try:
        cls = table[str(kind).lower()]
    except KeyError as exc:
        raise ValueError(
            f"unknown hardware input adapter {kind!r}; available: {sorted(table)}"
        ) from exc
    return cls(**kwargs)


def make_hardware_input_source(**kwargs: Any) -> HardwareInputSource:
    kind = kwargs.pop("adapter", kwargs.pop("kind", "placeholder"))
    return HardwareInputSource(make_hardware_input_adapter(kind, **kwargs))


__all__ = [
    "HardwareInterfaceNotConfigured",
    "HardwareInputAdapter",
    "PlaceholderHardwareInputAdapter",
    "LoopbackHardwareInputAdapter",
    "RecordedHardwareInputAdapter",
    "HardwareInputSource",
    "make_hardware_input_adapter",
    "make_hardware_input_source",
]
