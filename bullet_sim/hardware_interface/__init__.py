"""Hardware interface layer (development board + CPU/FPGA).

    Development Board
      -> [HARDWARE_INPUT_INTERFACE_TBD]        <- transport NOT decided
      -> HardwareInputAdapter                  <- input_adapter.py
      -> Action -> Simulator

    Simulator State -> encode_state() -> CPU -> FPGA input   (state_link.py)
    FPGA result -> CPU decision -> Action -> Simulator

**Nothing in this package guesses the board protocol.**  Transports, baud rates,
register addresses, packet lengths, byte order and encodings are all declared as
``None`` in :mod:`bullet_sim.hardware_interface.tbd` and must stay that way until
the real board is selected.

Current status
--------------
============================  ==================================================
:class:`HardwareInputAdapter` abstract base, ready to implement
:class:`PlaceholderHardwareInputAdapter` the default: refuses instead of guessing
:class:`LoopbackHardwareInputAdapter` in-process harness for pipeline tests
:class:`HardwareInputSource` makes a board adapter look like any other ActionSource
``state_link``               the already-frozen state/action byte protocol
============================  ==================================================
"""

from bullet_sim.hardware_interface.input_adapter import (
    HardwareInputAdapter,
    HardwareInputSource,
    HardwareInterfaceNotConfigured,
    LoopbackHardwareInputAdapter,
    PlaceholderHardwareInputAdapter,
    RecordedHardwareInputAdapter,
    make_hardware_input_adapter,
    make_hardware_input_source,
)
from bullet_sim.hardware_interface.tbd import (
    ActionFrameFormat,
    CANDIDATE_TRANSPORTS,
    HARDWARE_INPUT_INTERFACE_TBD,
    PLANNED_TRANSPORT,
    PlannedTransport,
)

__all__ = [
    "HARDWARE_INPUT_INTERFACE_TBD",
    "CANDIDATE_TRANSPORTS",
    "PlannedTransport",
    "PLANNED_TRANSPORT",
    "ActionFrameFormat",
    "HardwareInputAdapter",
    "HardwareInputSource",
    "HardwareInterfaceNotConfigured",
    "PlaceholderHardwareInputAdapter",
    "LoopbackHardwareInputAdapter",
    "RecordedHardwareInputAdapter",
    "make_hardware_input_adapter",
    "make_hardware_input_source",
]
