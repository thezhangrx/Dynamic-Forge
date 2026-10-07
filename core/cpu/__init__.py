"""CPU side of the CPU/FPGA split.

交付内容只有两块：

* **避障算法**（``cpu.gap_avoid``）—— ``wall_with_gap`` 的正式算法，
  纯 stdlib、单入口 ``plan()``，是唯一需要移植成 C++/Verilog 的那份；
* **帧式硬件回路**（``cpu.pipeline``）—— ``State -> FPGA -> CPU -> Action``
  走字节帧，含 :class:`FrameCpuDecisionLayer` 与 :class:`CpuFpgaPipeline`。

与仿真器/视觉的接线在 ``cpu.adapters``。

**已删除的 ``cpu.decision``**：那是一套通用决策架构（models / predictor /
risk / planner + FAR / LTV / FSC / corridor 等六个版本，约 6.4k 行源码 +
5k 行测试），与参赛范围不符（只比三种障碍、逐个单独展示、不做混合），
而且它的默认路径是"6 个手调权重的加权和"，与自述的"绝不加权和"矛盾。
它的整条生态（``benchmark_decision`` / ``baseline_reactive_gap`` /
``tools/`` / ``adapters/game_adapter`` / ``docs/decision_v0*.md``）一并删除。

**``cpu.pipeline`` 与 ``cpu.decision`` 无关** —— 它只依赖 ``bullet_sim.fpga``
的参考预测块、``bullet_sim.interface.protocol`` 的帧编解码和
``bullet_sim.action`` 的动作类型。所以删掉 ``decision`` 后 CPU↔FPGA 的
帧回路与它的测试（``tests/test_cpu_fpga_pipeline.py``）完全不受影响，
这一点在删之前用引用图核过。
"""

from cpu.pipeline import (
    CpuFpgaController,
    CpuFpgaPipeline,
    FrameCpuDecisionLayer,
    PipelineLatency,
)

__all__ = [
    "FrameCpuDecisionLayer",
    "CpuFpgaPipeline",
    "CpuFpgaController",
    "PipelineLatency",
]
