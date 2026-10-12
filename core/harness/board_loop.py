"""板子的模拟全流程（**闭环**）—— 把 Python 全流程里的"视觉"换成 PL 那一份。

参考实现
--------
`core/harness/loop.py` 的 :func:`run_episode` 是**本项目的 Python 全流程**：

    bullet_sim 世界 → 合成相机 → core/vision(Python) → 单应 → 决策 → 动作 → 世界

本模块是**同一个闭环**，只把 ④ 那一步换成板子上的那条路：

    bullet_sim 世界 → 合成相机 → **PL 顶替物(C 模型/RTL)** → 几何帧字节
                   → 解码 → 单应 → 决策 → 动作 → 世界
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^ 只有这一段不同

为什么要做这件事
----------------
`board_sim_run.py` 是**开环**的：放一段录像，决策的输出不回灌任何东西。
这能验"链路通不通"，但验不了"**闭环**里这套东西行不行"——因为：
  * 录像里的画面不随决策改变；
  * 于是"无碰撞率"这类结论根本不成立。

本模块把 PL 插回闭环，所以指标重新变得有意义：动作 → 世界 → 相机 → PL → 决策。

**关键纪律（沿用 loop.py 的）**：决策层只能看见摄像头。
`plan()` 拿到的永远是**视觉产出的世界坐标帧**；`env.world` 只用来推进物理和渲染，
绝不喂给算法。

为什么复用 :func:`run_episode` 而不是重写一遍
--------------------------------------------
世界的推进、控制频率解耦、碰撞统计、分档统计、越界保护 —— 这些**都与视觉实现无关**。
重写一遍等于把 `loop.py` 里那些踩过坑的逻辑（`_contacts` 的计数器、
`decide_every` 的量化）复制成第二份，然后等着两份漂移。
`run_episode` 早就留了 `vision=` 这个可插拔点，本模块只提供那个回调。
"""

from __future__ import annotations

import pathlib
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Optional

_CORE = pathlib.Path(__file__).resolve().parents[1]
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from cpu.gap_avoid import AlgoConfig                    # noqa: E402
from harness.board_sim import RefPl                     # noqa: E402
from harness.loop import run_episode                    # noqa: E402
from standard.vision.vision_frame import decode_vision_frame   # noqa: E402

#: 几何帧头 32 B（`vision_pkg.vh` / `vision_frame.py` 的 HEADER_BYTES）
HEADER_BYTES = 32
#: 尾 4 B（CRC32）
TRAILER_BYTES = 4


# ===========================================================================
# PL 的统计：板子模拟里"看得见"的那部分指标
# ===========================================================================
@dataclass
class PlStats:
    """PL 这一段的可观测量。

    这些数**只来自 PL 的产物**（几何帧字节），不用真值、不用 Python 视觉 ——
    否则就不是在测 PL 了。
    """

    frames: int = 0                 # PL 处理过几帧
    player_valid: int = 0           # 有几帧检出了角色
    n_obstacles: list = field(default_factory=list)
    n_gaps: list = field(default_factory=list)
    frame_bytes: list = field(default_factory=list)
    t_ms: list = field(default_factory=list)   # PL 每帧耗时

    @property
    def player_valid_rate(self) -> float:
        return self.player_valid / self.frames if self.frames else 0.0

    def _med(self, v: list) -> float:
        if not v:
            return 0.0
        s = sorted(v)
        return float(s[len(s) // 2])

    def summary(self) -> dict[str, Any]:
        return {
            "frames": self.frames,
            "player_valid_rate": round(self.player_valid_rate, 4),
            "n_obstacles_median": self._med(self.n_obstacles),
            "n_gaps_median": self._med(self.n_gaps),
            "frame_bytes_median": self._med(self.frame_bytes),
            "pl_ms_median": round(self._med(self.t_ms), 3),
        }


# ===========================================================================
# 把 PL 包成 run_episode 的 vision 回调
# ===========================================================================
class PlVision:
    """``vision=`` 回调：**像素 → PL → 几何帧字节 → 解码后的像素坐标帧**。

    契约（与 `run_episode` 的文档一致）：
      * 输入是相机给的 `vision.primitives.Frame`（平面式 R/G/B）；
      * 输出必须是**像素坐标**的 ``VisionFrame`` —— **单应由调用方统一施加**，
        这里绝不做单应。让回调自己做会出现"两套标定"，那是最难查的一类错。
    """

    def __init__(self, pl: Any = None, *, stats: Optional[PlStats] = None):
        self.pl = pl if pl is not None else RefPl()
        self.stats = stats if stats is not None else PlStats()

    def __call__(self, frame: Any, *, seq: int, stamp: float,
                 classifier: Any = None) -> Any:
        # 平面 → 交错。`Frame.to_rgb()` 就是干这个的，不自己写一遍。
        rgb = bytes(frame.to_rgb())
        w, h = int(frame.width), int(frame.height)

        t0 = time.perf_counter()
        frame_bytes, res = self.pl.process(
            rgb, w, h, seq=int(seq), stamp_ns=int(stamp * 1e9))
        dt_ms = (time.perf_counter() - t0) * 1e3

        self.stats.frames += 1
        self.stats.frame_bytes.append(len(frame_bytes))
        self.stats.n_obstacles.append(int(getattr(res, "n_obstacles", 0)))
        self.stats.n_gaps.append(int(getattr(res, "n_gaps", 0)))
        self.stats.t_ms.append(dt_ms)
        if int(getattr(res, "player_valid", 0)):
            self.stats.player_valid += 1

        # 这里是 PL 的产物解码 —— 板子上 PS 拿到 AXI 窗口里的字节之后干的也是这件事
        return decode_vision_frame(frame_bytes)


# ===========================================================================
# 一局：板子链路 + 闭环
# ===========================================================================
def run_board_episode(env: Any, camera: Any, calib: Any, *, steps: int,
                      cfg: Optional[AlgoConfig] = None, pl: Any = None,
                      control_hz: float = 30.0, verbose: bool = False,
                      classifier: Any = None, policy: str = "gap_avoid",
                      ) -> dict[str, Any]:
    """跑一局**板子链路**的闭环。

    与 :func:`run_episode` 的差别**只有一处**：④ 那一步走 PL 顶替物。
    其余（世界推进、控制频率、碰撞统计、阴性对照）全部复用，所以指标可比。
    """
    vision = PlVision(pl)
    stats = run_episode(env, camera, calib,
                        steps=steps, cfg=cfg or AlgoConfig(),
                        control_hz=control_hz, verbose=verbose,
                        classifier=classifier, policy=policy,
                        vision=vision)
    stats["pl"] = vision.stats.summary()
    stats["vision_backend"] = getattr(vision.pl, "name", type(vision.pl).__name__)
    return stats


def describe_backend(pl: Any = None) -> str:
    """一句话说明这一局的"PL"是什么 —— 写进日志，别让人猜。"""
    pl = pl if pl is not None else RefPl()
    return getattr(pl, "name", type(pl).__name__)
