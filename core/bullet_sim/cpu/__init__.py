"""CPU side of the CPU/FPGA split.

    FPGA result (prediction frame)
        -> CpuDecisionLayer.decide(state_frame, prediction_frame)
        -> Action
        -> Simulator

The CPU layer talks **frames**, never the ``World``, so replacing the Python
reference FPGA with real silicon changes nothing above this boundary.  The
transport itself is ``[HARDWARE_INTERFACE_TBD]``.
"""

from bullet_sim.cpu.decision import (
    CpuDecisionLayer,
    CpuFpgaController,
    CpuFpgaPipeline,
    PipelineLatency,
)

__all__ = [
    "CpuDecisionLayer",
    "CpuFpgaPipeline",
    "CpuFpgaController",
    "PipelineLatency",
]
