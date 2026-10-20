"""static-wall 平台 v2：墙全部开局摆好、不生成不移动；4 道墙各一个固定缺口。

为什么单独做这个平台：v1 里"墙在动 + 虚拟真值积分 + 决策抖动"三件事互相耦合，
任何一条出问题都会表现成"角色乱飘/撞墙"。v2 把变量降到最少——
**墙不动、目标唯一（穿过全部缺口）、画面跟着角色 y 滚动**，
于是剩下能出错的只有"看得准不准、瞄得准不准"。

用法：
    python -m bullet_sim play --scenario static_wall --fullscreen
    python -m bullet_sim board-control --stdin --scenario static_wall --fullscreen
"""

from __future__ import annotations

from bullet_sim.generators.spec import PatternSpec
from bullet_sim.scenarios.spec import ScenarioSpec

#: 4 道静态墙的 y（场地单位，y 朝上）
WALL_Y = (120.0, 220.0, 320.0, 420.0)
#: 每道墙的缺口中心 x（各不相同，必须挨个穿过）
GAP_X = (120.0, 480.0, 200.0, 420.0)
#: 缺口宽 / 墙厚（场地单位）
GAP_W = 70.0
WALL_THICK = 8.0
#: 目标点：所有墙之上；角色出生点：底部中央
GOAL = (320.0, 470.0)
#: 到达判定半径（场地单位）：绿圈的半径就是它
GOAL_R = 24.0
SPAWN = (320.0, 60.0)
#: 渲染跟随窗口高度（场地单位），宽度仍是全宽 640
FOLLOW_H = 240.0


def make(seed: int = 0, duration: float = 120.0,
         n_walls: int | None = None, y0: float = 70.0, dy: float = 50.0,
         gap_cycle: tuple[float, ...] | None = None) -> ScenarioSpec:
    """生成静态墙场景。

    `n_walls=None` 用默认 4 道（WALL_Y / GAP_X）；给了 `n_walls` 就**从 y0 起每隔 dy
    放一道**，缺口位置按 `gap_cycle` 循环（默认 120/480/200/420），方便调试"多道墙"。
    """
    if n_walls is not None:
        ys = [y0 + i * dy for i in range(int(n_walls))]
        gxs = [gap_cycle[i % len(gap_cycle)] for i in range(int(n_walls))] if gap_cycle else \
              [GAP_X[i % len(GAP_X)] for i in range(int(n_walls))]
        walls = tuple(ys), tuple(gxs)
    else:
        walls = WALL_Y, GAP_X
    pats: list[PatternSpec] = []
    for y, gx in zip(*walls):
        pats.append(
            PatternSpec(
                kind="obstacle",
                count=1,
                start_time=0.0,
                interval=1.0,
                repetitions=1,
                shape="rect",                  # **矩形**（默认是 circle → 画出来是圆点）
                half_w=1.0,                    # 占位；真实半宽由布局按缺口位置算
                half_h=WALL_THICK / 2.0,
                # **永久有效**：`PatternSpec.ttl` 默认 20 s，到期 `expire_bullets()`
                # 会把障碍整批删掉 —— 静态墙必须活到回合结束（这也是"墙会消失"
                # 最直接的一条代码路径）。
                ttl=1.0e9,
                params={
                    "layout": "wall_static",
                    "y": y,
                    "gap_center": gx,
                    "gap_width": GAP_W,
                    "thickness": WALL_THICK,
                },
            )
        )
    return ScenarioSpec(
        name=f"static_wall_{len(pats)}",
        seed=seed,
        dt=1.0 / 120.0,
        duration=duration,
        field_w=640.0,
        field_h=480.0,
        player_radius=10.0,
        player_speed=200.0,
        player_x=SPAWN[0], player_y=SPAWN[1],
        # ⚠ **必须显式给 target**：`target_x is None` 时默认取场地中心 (320,240)，
        # 角色一开始往上走就"到达目标"→ 回合重置 → 墙全部消失（用户看到的现象）。
        target_x=GOAL[0], target_y=GOAL[1],
        # 到达判定半径：**只要角色中心进入这个圆就算到达**（用户口径）。
        # 画面上就是这个绿圈的半径，所以「看到绿圈 = 到达范围」。
        target_radius=GOAL_R,
        # 到达目标**不结束回合**（我们要的是持续观察，不是通关即重置）
        success_terminates_episode=False,
        patterns=pats,
        meta={"goal": GOAL, "spawn": SPAWN, "static": True,
              "follow_h": FOLLOW_H, "walls_y": tuple(walls[0]), "gaps_x": tuple(walls[1])},
    )
