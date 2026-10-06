"""Everything about the development board that is **not yet decided**.

The specification is explicit: do not guess the transport, the data format, the
baud rate, the register addresses, the packet length, the byte order, or the
CPU/FPGA handshake.  So none of those are invented here - they are declared as
``None`` with the marker ``[HARDWARE_INPUT_INTERFACE_TBD]`` and must be filled
in only after the real board and its protocol are chosen.

Any code that needs one of these values must read it from
:class:`PlannedTransport` and handle ``None`` (typically by raising
:class:`~bullet_sim.hardware_interface.input_adapter.HardwareInterfaceNotConfigured`).

When the board arrives, fill in this table, implement
``HardwareInputAdapter.decode_payload`` for the chosen frame format, and change
nothing else: the ``Adapter -> Action -> Simulator`` path is already wired.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

#: The single marker used across the code base for "not yet determined".
HARDWARE_INPUT_INTERFACE_TBD = "[HARDWARE_INPUT_INTERFACE_TBD]"

#: Candidate transports.  Listed only to make the decision explicit - **none of
#: them is selected**, and the list must not be read as a recommendation.
CANDIDATE_TRANSPORTS: tuple[str, ...] = (
    "uart",
    "spi",
    "usb",
    "gpio",
    "tcp",
    "shared_memory",
    "mmio",
    "pcie",
    "other",
)


@dataclass
class PlannedTransport:
    """Placeholder for every still-unknown property of the board link.

    All fields default to ``None`` / ``"TBD"`` on purpose.  Nothing in this
    repository may assume a value.
    """

    #: One of :data:`CANDIDATE_TRANSPORTS`, or ``None`` while undecided.
    kind: str | None = None
    #: Serial-style line rate (baud) - unknown.
    baud_rate: int | None = None
    #: Byte order of multi-byte fields ("little"/"big") - unknown.
    byte_order: str | None = None
    #: Fixed packet/frame length in bytes, if the protocol turns out to be fixed.
    packet_bytes: int | None = None
    #: Register / MMIO base address, if the link turns out to be memory mapped.
    register_base: int | None = None
    #: Payload encoding ("raw", "fixed_point", "protobuf", ...) - unknown.
    payload_format: str | None = None
    #: Host device node / endpoint / channel identifier - unknown.
    endpoint: str | None = None
    #: Handshake, timeout and retry policy - unknown.
    handshake: str | None = None
    timeout_ms: float | None = None
    #: Free-form notes to be filled in during bring-up.
    notes: str = HARDWARE_INPUT_INTERFACE_TBD

    @property
    def configured(self) -> bool:
        """True only once the minimum set needed to actually talk to a board is set."""
        return self.kind is not None and self.payload_format is not None

    def missing(self) -> list[str]:
        """Names of the fields that are still unknown."""
        return [k for k, v in asdict(self).items() if v is None]

    def describe(self) -> dict[str, Any]:
        data = asdict(self)
        data["configured"] = self.configured
        data["marker"] = HARDWARE_INPUT_INTERFACE_TBD
        data["missing"] = self.missing()
        return data

    def require(self, *fields: str) -> None:
        """Raise if any of ``fields`` has not been decided yet."""
        from bullet_sim.hardware_interface.input_adapter import (
            HardwareInterfaceNotConfigured,
        )

        unknown = [f for f in fields if getattr(self, f, None) is None]
        if unknown:
            raise HardwareInterfaceNotConfigured(
                f"{HARDWARE_INPUT_INTERFACE_TBD} unknown field(s): {unknown}. "
                "Do not guess the board protocol - fill in PlannedTransport after "
                "the development board and its link are chosen."
            )


#: The project-wide placeholder instance.  Copy it, fill it in, and hand the
#: filled copy to a HardwareInputAdapter implementation.
PLANNED_TRANSPORT = PlannedTransport()


@dataclass
class ActionFrameFormat:
    """How a board side action is encoded - **also undecided**.

    Kept separate from :class:`PlannedTransport` because a link may be fixed
    before the payload is agreed.  Every field defaults to ``None``.
    """

    #: Number of raw fields per action sample (e.g. 1 for a direction byte).
    fields: int | None = None
    #: Bit width per field, if a fixed-point encoding is used.
    bits_per_field: int | None = None
    #: Scale/offset for fixed-point conversion, if applicable.
    scale: float | None = None
    offset: float | None = None
    #: Whether the board sends a discrete index or a continuous 2D vector.
    semantics: str | None = None
    notes: str = HARDWARE_INPUT_INTERFACE_TBD

    def describe(self) -> dict[str, Any]:
        data = asdict(self)
        data["marker"] = HARDWARE_INPUT_INTERFACE_TBD
        data["missing"] = [k for k, v in asdict(self).items() if v is None]
        return data


__all__ = [
    "HARDWARE_INPUT_INTERFACE_TBD",
    "CANDIDATE_TRANSPORTS",
    "PlannedTransport",
    "PLANNED_TRANSPORT",
    "ActionFrameFormat",
]
