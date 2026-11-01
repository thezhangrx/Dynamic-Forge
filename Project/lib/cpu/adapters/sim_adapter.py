"""仿真器适配层 —— 把 bullet_sim 的 world/池子包成 ``gap_avoid.plan()`` 的输入。

**这个文件故意和 ``gap_avoid.py`` 分开**：

``gap_avoid.py`` 是"要移植到 C++ 的那份算法"，它必须**零平台耦合**；
本文件是"在本平台上跑起来"的胶水，**移植时整份删掉**。

原来的 ``gap_avoid.py:598-626`` 把这段胶水写在算法文件里，导致：
* 文件头声称"纯 Python、零依赖"，但文件里出现 `bullet_sim` 的池子结构；
* 接收方无法一眼判断"哪一段才是要移植的算法"。

``gap_avoid.py`` 仍然 re-export 这三个名字（PEP 562 懒加载），所以**旧导入不破**：

    from cpu.gap_avoid import view_from_world      # 仍然可用

依赖：只 import ``gap_avoid`` 里的数据类型；不 import numpy、不 import 平台。
"""

from __future__ import annotations

from typing import Any

from cpu.gap_avoid import SHAPE_RECT, Rect, WorldView


def rect_from_pool(pool: Any, i: int) -> Rect:
    """从 SoA 池子里读一个矩形实体（protocol v3 的几何字段）。"""
    d = pool.data
    return Rect(ent_id=int(d["id"][i]),
                x=float(d["x"][i]), y=float(d["y"][i]),
                vx=float(d["vx"][i]), vy=float(d["vy"][i]),
                half_w=float(d["half_w"][i]), half_h=float(d["half_h"][i]),
                rotation=float(d["rotation"][i]),
                angular_velocity=float(d["angular_velocity"][i]))


def observe_rects(pool: Any) -> list[Rect]:
    """把池子里所有矩形读成 Rect 列表（非矩形跳过）。"""
    return [rect_from_pool(pool, i) for i in pool.active_indices()
            if int(pool.data["shape"][i]) == SHAPE_RECT]


def view_from_world(world: Any) -> WorldView:
    """把 bullet_sim 的 world 包成 WorldView。"""
    return WorldView(player=[float(v) for v in world.player],
                     rects=observe_rects(world.state.bullets),
                     dt=float(world.state.env.dt),
                     step_index=int(world.state.env.step_index),
                     field_w=float(world.state.env.field_w),
                     field_h=float(world.state.env.field_h))


__all__ = ["rect_from_pool", "observe_rects", "view_from_world"]
