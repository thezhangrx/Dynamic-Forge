"""Hardware interface layer (development board + CPU/FPGA).

    Development Board
      -> @MOVE n over serial / stdout / UDP     <- 协议已定：core/board/action_codec.hpp
      -> BoardInputAdapter                      <- board_link.py
      -> Action -> Simulator

    Simulator State -> encode_state() -> CPU -> FPGA input   (state_link.py)
    FPGA result -> CPU decision -> Action -> Simulator

板子协议**已经定了**（`@MOVE n`，传输三种：串口 / stdout / UDP），实现在
:mod:`bullet_sim.hardware_interface.board_link`；解析规则与键盘桥逐批对账。
`tbd.py` 里的 TBD 标记与 `PlaceholderHardwareInputAdapter` 仍然保留：**没有**给任何
传输参数时它还是默认 —— 拒绝产出，而不是猜一个协议。

Current status
--------------
============================  ==================================================
:class:`BoardInputAdapter`    **真板子**（`@MOVE n`）：stdin / UDP / 串口 + 看门狗回中
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
from bullet_sim.hardware_interface.board_link import (
    MAX_N as BOARD_ACTION_MAX,
    PREFIX as BOARD_ACTION_PREFIX,
    BoardInputAdapter,
    SerialBoardAdapter,
    StdinBoardAdapter,
    UdpBoardAdapter,
    make_board_adapter,
    parse_moves as parse_board_moves,
)
from bullet_sim.hardware_interface.tbd import (
    ActionFrameFormat,
    CANDIDATE_TRANSPORTS,
    HARDWARE_INPUT_INTERFACE_TBD,
    PLANNED_TRANSPORT,
    PlannedTransport,
)

__all__ = [
    "BOARD_ACTION_PREFIX",
    "BOARD_ACTION_MAX",
    "BoardInputAdapter",
    "StdinBoardAdapter",
    "UdpBoardAdapter",
    "SerialBoardAdapter",
    "make_board_adapter",
    "parse_board_moves",
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
