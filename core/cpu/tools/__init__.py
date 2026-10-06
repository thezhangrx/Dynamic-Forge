"""CPU 决策的离线诊断工具（不属于运行时链路）。

* ``fsc_shadow_probe.py``            —— Future-Safe-Corridor 影子模式探针
* ``ltv_regression_diagnostics.py``  —— LTV（长时域可行性）回归诊断

两个脚本都能直接 ``python core/cpu/tools/<脚本>.py`` 运行；测试里以
``from tools import ltv_regression_diagnostics`` 的形式导入。
"""
