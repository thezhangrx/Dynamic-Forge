"""State/Action link towards CPU / FPGA.

This module is the *upward* half of the hardware story: the simulator pushes a
flat state frame out, and a prediction result comes back.  The board *input*
half lives in :mod:`bullet_sim.hardware_interface.input_adapter`; together they
express the full loop the project is aiming at::

    Simulator State  --encode_state-->  CPU --> FPGA input buffer
                                          FPGA result --> CPU decision
    Board / CPU action --HardwareInputAdapter--> Action --> Simulator

The concrete bus (DMA / MMIO / interrupt / polling / shared memory) is **not
decided**; see :mod:`bullet_sim.hardware_interface.tbd`.  What is fixed here is
the byte layout, which is the part the FPGA must agree on regardless of bus.
"""

from __future__ import annotations

from bullet_sim.interface.hardware import (
    ActionCodecLink,
    FileLink,
    HardwareLink,
    LinkStats,
    LoopbackLink,
    NullLink,
    SerialLinkStub,
    make_link,
)
from bullet_sim.interface.protocol import (
    decode_state,
    describe_protocol,
    encode_action_discrete,
    encode_action_velocity,
    encode_state,
    frame_size,
    read_header,
)

__all__ = [
    "HardwareLink",
    "LinkStats",
    "NullLink",
    "LoopbackLink",
    "FileLink",
    "SerialLinkStub",
    "ActionCodecLink",
    "make_link",
    "encode_state",
    "decode_state",
    "encode_action_velocity",
    "encode_action_discrete",
    "read_header",
    "frame_size",
    "describe_protocol",
]
