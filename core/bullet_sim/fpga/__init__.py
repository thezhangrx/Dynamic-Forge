"""FPGA boundary: what the FPGA consumes, what it produces, and what is TBD.

    Simulator State  --encode_state()-->  [state frame, protocol v2]
                                          |
                                          v
                                    FPGA prediction block
                                          |
                                          v
                                     [prediction frame, BHP1]
                                          |
                                          v
                                    CPU decision layer  (bullet_sim.cpu)
                                          |
                                          v
                                        Action
                                          |
                                          v
                                      Simulator

**Implemented today**: both frame layouts and a Python reference of the
prediction block (:class:`~bullet_sim.fpga.prediction_block.FpgaPredictionReference`),
so the CPU side and the whole pipeline can be built and tested now.

**Not implemented, not guessed**: the CPU <-> FPGA transport, the register map,
the handshake, resource estimates and timing closure.  Every one of those carries
``[HARDWARE_INTERFACE_TBD]``.  See ``bullet_sim/hardware_interface/tbd.py``.
"""

from bullet_sim.fpga.prediction_block import (
    HARDWARE_INTERFACE_TBD,
    PREDICTION_MAGIC,
    PREDICTION_PROTOCOL_VERSION,
    SOFTWARE_REFERENCE_ONLY,
    FpgaConfig,
    FpgaPredictionReference,
    PredictionHeader,
    decode_prediction_frame,
    describe_prediction_protocol,
    encode_prediction_frame,
    prediction_frame_size,
)

__all__ = [
    "PREDICTION_MAGIC",
    "PREDICTION_PROTOCOL_VERSION",
    "HARDWARE_INTERFACE_TBD",
    "SOFTWARE_REFERENCE_ONLY",
    "PredictionHeader",
    "FpgaConfig",
    "FpgaPredictionReference",
    "prediction_frame_size",
    "encode_prediction_frame",
    "decode_prediction_frame",
    "describe_prediction_protocol",
]
