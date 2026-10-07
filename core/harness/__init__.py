"""闭环测试台（集成层）。

把 ``bullet_sim`` 当"真实世界"，``core/vision`` 当唯一传感器，
``core/cpu`` 当决策层，跑完整的一圈。它依赖四个模块：
``bullet_sim`` + ``vision`` + ``cpu`` + ``standard``，所以既不属于
``core/cpu/``（那里不许依赖平台实现），也不该留在仓库根的调用脚本里。

见 :mod:`harness.loop`。
"""
