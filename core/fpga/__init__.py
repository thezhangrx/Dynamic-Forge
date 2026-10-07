"""``fpga`` —— 视觉链路往 PL 走的这一层（P0–P5 见 docs/移植规划.md）。

这里**没有 Python 运行时逻辑**，只有三类东西：

* ``rtl/``   可综合的 Verilog（``vision_top.v`` / ``vision_ccl.v`` / ``vision_ops.vh``）
* ``ref/``   C 参考模型（``vision_ref.c``）与激励导出、与 Python 的对账脚本
* ``cosim/`` Verilator 协同仿真台（RTL ↔ C 参考模型逐位对账）

``tests/`` 里的 pytest 负责把这三块串起来：线格式跨语言一致、RTL 常量不漂移、
以及**在真实录像帧上 RTL 与 C 参考模型逐位一致**。
"""
