"""图像处理模块（vision）。

分层（与平台一致：依赖只能从专用层流向通用层）

* ``primitives.py``      通用原语：颜色分割 / 形态学 / 轮廓 / 几何拟合 / 路径工具
* ``wall_with_gap.py``   本障碍专用：两段墙 + 缺口，外加角色与目标
* ``capture_one_frame.py``  抓帧，产物写到 ``data/vision/``

调用代码放在仓库根：``vision_detect.py``（前缀 = ``core/`` 下的目录名）。
"""

from __future__ import annotations

__all__ = ["primitives", "wall_with_gap"]
