"""从 ``world.state`` 读出**真值**（给打分用）—— 不经过任何视觉。

为什么单独一个模块
------------------
这两个函数原先住在根目录的 ``vision_reconcile.py`` 里。后来闭环脚本
``cpu_vision_loop.py`` 也要用它们**给存活/误差打分**，于是
``cpu_vision_loop`` -> ``vision_reconcile`` -> ``cpu_vision_loop`` 形成
**循环导入**，直接 ImportError。

放到平台包里是正确的解法，而不是把 import 藏进函数里绕开：这两个函数
读的是**平台自己的状态**（``world.state.player`` / ``world.state.bullets``），
和视觉一点关系都没有，本来就该由平台提供。于是依赖方向变回单向：

    cpu_vision_loop.py ─┐
                        ├─▶ bullet_sim.simulator.truth  （读真值）
    vision_reconcile.py ┘

**纪律：真值只用来打分，绝不参与决策。** 闭环脚本里唯一允许读真值的地方
是统计（存活时间、决策输入误差）；一旦把它喂给 ``plan()``，整个环路就
什么都证明不了了。
"""

from __future__ import annotations

__all__ = ["truth_player", "truth_gaps"]


def truth_player(world) -> tuple[float, float, float]:
    """角色真值 ``(x, y, 半径)``。"""
    p = world.state.player
    return float(p.x), float(p.y), float(p.radius)


def truth_gaps(world, *, band: float = 6.0) -> list[tuple[float, float, float, float]]:
    """**所有**墙的缺口，每条 ``(缺口中心x, 缺口宽, 墙带中心y, 墙带厚)``。

    按 **y 分带**，不按 ``group_id`` —— 因为实测发现 ``group_id`` 根本分不开墙：
    ``wall_with_gap`` 连开两堵墙时，四个矩形**全是** ``group_id=0, type_id=9``。
    分带正是 ``vision.wall_with_gap.detect_walls`` 自己用的办法，所以真值跟着它走，
    两侧才是同一套语义 —— 否则"误差"里混进去的是**定义不一致**，不是识别误差。
    """
    pool = world.state.bullets
    idx = pool.active_indices()
    if idx.size == 0:
        return []
    d = pool.data
    rects: list[tuple[float, float, float, float]] = []
    for i in idx:
        if d["shape"][i] == 0:
            continue                              # 圆不是墙
        rects.append((float(d["x"][i]), float(d["y"][i]),
                      float(d["half_w"][i]), float(d["half_h"][i])))
    rects.sort(key=lambda r: r[1])

    bands: list[list[tuple[float, float, float, float]]] = []
    for r in rects:
        if bands and abs(r[1] - bands[-1][-1][1]) <= band:
            bands[-1].append(r)
        else:
            bands.append([r])

    out: list[tuple[float, float, float, float]] = []
    for b in bands:
        if len(b) != 2:
            continue                              # 缺口墙必须是两段
        a, c = sorted(b, key=lambda r: r[0] - r[2])       # 按左边缘
        left_edge, right_edge = a[0] + a[2], c[0] - c[2]
        if right_edge <= left_edge:
            continue                              # 两段重叠 -> 不是缺口
        out.append((0.5 * (left_edge + right_edge), right_edge - left_edge,
                    0.5 * (a[1] + c[1]), a[3] + c[3]))
    return out
