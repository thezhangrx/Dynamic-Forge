"""Adapters: 数据来源 ⇄ 决策。

两条互不依赖的胶水：

* ``sim_adapter``    —— bullet_sim 的池子 → ``gap_avoid.WorldView``（**仿真专用**）；
* ``vision_adapter`` —— ``VisionFrame``（视觉输出）→ ``gap_avoid.WorldView``
  （**比赛链路用这个**；纯 stdlib、不碰摄像头、可用合成帧测）。

注意：这两个适配器**不在这里 import**，避免 ``cpu.adapters`` 一被导入就拉进
numpy / 平台类型。直接按路径导入即可::

    from cpu.adapters.vision_adapter import VisionWorldStream
    from cpu.adapters.sim_adapter import view_from_world

**已删除的 ``game_adapter``**：它是把仿真器 ``WorldSnapshot`` 转成抽象
``SceneState`` 的胶水，只服务于已删的 ``cpu.decision`` 架构
（`cpu.decision.models`），``sim_adapter`` / ``vision_adapter`` 都与它无关。
"""
