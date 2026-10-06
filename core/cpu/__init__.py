"""CPU side of the CPU/FPGA split.

Two complementary halves live here:

* the **abstract decision architecture** (``cpu.decision``):
  ``SceneState -> CpuDecisionLayer.decide(state) -> Action``, built from
  models / predictor / risk / planner and kept game-agnostic;
* the **frame-based hardware loop** (``cpu.pipeline``):
  ``State -> FPGA -> CPU -> Action`` over byte frames, with the legacy
  :class:`FrameCpuDecisionLayer` and :class:`CpuFpgaPipeline`.

The simulator-specific mapping lives in ``cpu.adapters``.
"""

from cpu.decision import CpuDecisionLayer
from cpu.pipeline import (
    CpuFpgaController,
    CpuFpgaPipeline,
    FrameCpuDecisionLayer,
    PipelineLatency,
)

__all__ = [
    "CpuDecisionLayer",
    "FrameCpuDecisionLayer",
    "CpuFpgaPipeline",
    "CpuFpgaController",
    "PipelineLatency",
]
