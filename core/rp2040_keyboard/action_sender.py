"""把决策层的速度量化成动作号，并写到板子的控制台串口。

为什么单独一个文件
------------------
"动作号"这件事在项目里已经有**公开协议**了 —— `core/bullet_sim/core/actions.py`
的 `DISCRETE_ACTION_NAMES`（源码注释：*Order is part of the public protocol —
never reorder, only append*）与配套的 `DISCRETE_DIRECTIONS`（9 个单位方向）。

所以这里**不另立一套映射**，直接拿那份表做"速度 → 最近的方向"的量化。
两边（板子 / RP2040）共用同一个下标含义，才不会各改各的。

线协议：ASCII 行 ``@MOVE <n>\\n``，n = 0..8。前缀是必需的 —— 这条线是
**共享的控制台**（`ttyPS0`），会混着内核日志和 shell 输出。
"""

from __future__ import annotations

import pathlib

# 复用项目里的公开动作表，不复制一份
import sys

_CORE = pathlib.Path(__file__).resolve().parents[1]
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from bullet_sim.core.actions import DISCRETE_ACTION_NAMES, DISCRETE_DIRECTIONS  # noqa: E402

PREFIX = "@MOVE "
DEFAULT_DEV = "/dev/ttyPS0"          # PS 的控制台 UART0
DEFAULT_DEADZONE = 1e-3              # |v| 小于它就当 stay


def action_from_velocity(wx: float, wy: float,
                         deadzone: float = DEFAULT_DEADZONE) -> int:
    """把世界系期望速度量化成 0..8 的动作号（最近方向）。

    取与 9 个单位方向**点积最大**的那个 —— 对单位向量来说点积就是
    cos(夹角)×|v|，所以这一条同时管了"方向"和"速度大小"：
    |v| 小的时候所有点积都小，于是再用 deadzone 兜一道。

    ⚠ 用点积而不是"比角度"，是因为 8 个方向本来就相差 45°，
    点积最大等价于夹角最小，而点积不需要 atan2（少一次浮点超越函数调用，
    在 PS 上无所谓，但这个函数在主机上也要被测试，越简单越好对账）。
    """
    if (wx * wx + wy * wy) ** 0.5 < deadzone:
        return 0                                    # stay
    best_i, best_d = 0, float("-inf")
    for i, (dx, dy) in enumerate(DISCRETE_DIRECTIONS):
        if i == 0:                                  # 0 是 stay，不参与选向
            continue
        d = dx * wx + dy * wy
        if d > best_d:
            best_i, best_d = i, d
    return best_i


def action_name(a: int) -> str:
    return DISCRETE_ACTION_NAMES[a] if 0 <= a < len(DISCRETE_ACTION_NAMES) else f"?{a}"


def format_line(a: int) -> bytes:
    """要写进串口的字节（含换行）。"""
    return f"{PREFIX}{a}\n".encode("ascii")


def send(action: int, dev: str | pathlib.Path = DEFAULT_DEV) -> None:
    """写到板子的控制台串口。

    ⚠ 往 `/dev/ttyPS0` 写会和内核日志/shell 输出交织 —— 所以 RP2040 侧靠
    `@MOVE ` 前缀挑指令。发之前最好 `dmesg -n 1` 把内核日志压下去，减少噪声。
    """
    with open(dev, "wb", buffering=0) as fp:
        fp.write(format_line(action))
