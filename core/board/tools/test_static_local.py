#!/usr/bin/env python3
"""在**本地 Python** 上先验"纯转向（B 方案）"能不能过静态墙平台。

为什么先在本地跑：板子上一轮闭环要 1~2 分钟，且视觉/虚拟真值/决策三件事耦合；
这里用**真值**直接喂控制器，把"看得准不准"整段拿掉，只看"瞄得准不准 + 会不会抖"，
改一次判据几秒钟就能出结果。

控制器（B 方案）：
    aim = 下一道未穿过的墙的缺口中心（wall.y > player.y + 8 里最近的那道）
    全部穿过后 aim = 目标点
    cmd = normalize(aim − player)      —— 纯转向，不拆横/纵、不"按时抵达"
    可选减速：离目标点很近或前方缺口太窄时降速（默认不减速，先看纯转向）

输出指标（新平台验收口径）：
    碰撞次数 / 缺口瞄准误差 p95 / 总用时 / 左右反转次数
"""
from __future__ import annotations
import argparse, math, statistics as st
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.core.actions import DISCRETE_DIRECTIONS

#: `DISCRETE_DIRECTIONS[code] = (ux, uy)`（numpy 数组，index = 动作码）
ACTION_OF = {}
for code, vec in enumerate(DISCRETE_DIRECTIONS):
    ux, uy = float(vec[0]), float(vec[1])
    n = math.hypot(ux, uy)
    if n > 0:
        ACTION_OF[(ux / n, uy / n)] = code


def pick_action(cmd_x: float, cmd_y: float, deadzone: float = 0.05) -> int:
    n = math.hypot(cmd_x, cmd_y)
    if n < deadzone:
        return 0
    ux, uy = cmd_x / n, cmd_y / n
    best, bestdot = 0, -2.0
    for (vx, vy), code in ACTION_OF.items():
        d = ux * vx + uy * vy
        if d > bestdot:
            best, bestdot = code, d
    return best


def _dist_to_rect(px, py, cx, cy, hw, hh):
    """点到轴对齐矩形的距离（内侧为 0）。用来做**角色圆盘 vs 墙矩形**的碰撞判定。"""
    dx = max(abs(px - cx) - hw, 0.0)
    dy = max(abs(py - cy) - hh, 0.0)
    return math.hypot(dx, dy)


def next_wall(walls, px, py, margin=8.0):
    """下一道未穿过的墙：y > py + margin 里 y 最小的那道。"""
    cand = [w for w in walls if w[1] > py + margin]
    return min(cand, key=lambda w: w[1]) if cand else None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--max-seconds", type=float, default=40.0)
    ap.add_argument("--deadzone", type=float, default=0.05)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)

    spec = ScenarioSpec.load(a.scenario)
    goal = (float(spec.target_x or 320.0), float(spec.target_y or 470.0))
    env = BulletHellEnv(spec, seed=spec.seed)
    obs, _ = env.reset(seed=spec.seed)
    px, py = float(env.world.player[0]), float(env.world.player[1])
    t, collisions, reversals, prev_act = 0.0, 0, 0, 0
    streak = 0
    collided = False
    min_clear = None
    aim_err = []
    passed = []
    while t < a.max_seconds:
        pool = env.world.state.bullets
        walls = []           # (left, right, y, gap_center)
        seg_rows = []
        for i in range(int(pool.count)):
            if not bool(pool.alive[i]):
                continue
            seg_rows.append((float(pool.y[i]), float(pool.x[i]) - float(pool.half_w[i]),
                             float(pool.x[i]) + float(pool.half_w[i])))
        seg_rows.sort()
        ys = sorted({round(r[0], 3) for r in seg_rows})
        for y in ys:
            segs = [(l, r) for (yy, l, r) in seg_rows if abs(yy - y) < 1e-6]
            segs.sort()
            if len(segs) >= 2:
                gl = segs[0][1]; gr = segs[-1][0]
                walls.append((segs[0][1], segs[-1][0], y, 0.5 * (segs[0][1] + segs[-1][0])))
        w = next_wall(walls, px, py)
        # ⚠ 目标点要**越过缺口**（wall.y + 20），不能正好等于缺口中心：
        #   否则角色一到缺口中心 (dx,dy)=(0,0) → 指令 0 → 卡在缺口里不动
        #   （实测就是这个：40 s 后停在 (120,80)，正好是第一道墙的缺口中心）。
        # 目标 y **永远在角色前方**（`max(..., py+20)`）：否则一越过目标点
        # dy 变负、角色往回走，形成"越过-回来"振荡，净前进为 0（实测卡在 121,99）。
        if w is not None:
            aim = (w[3], max(w[2] + 20.0, py + 20.0))
        else:
            aim = (goal[0], max(goal[1], py + 20.0))
        dx, dy = aim[0] - px, aim[1] - py
        want = pick_action(dx, dy, a.deadzone)
        # **动作迟滞**：量化到 8 方向时，cmd 落在 45° 附近会每帧在两个相邻方向间跳
        # （实测 4739 次切换 / 40 s）。要求"新方向连续 2 帧占优"才真正换向。
        if want == prev_act:
            streak = 0
            act = prev_act
        else:
            streak += 1
            act = want if streak >= 2 else prev_act
        if act != prev_act:
            reversals += 1
            prev_act = act
        # 瞄准误差：目标缺口中心的 x 与角色 x 的差（在穿墙前统计）
        if w is not None and abs(py - w[2]) < 60:
            aim_err.append(abs(px - w[3]))
        # **碰撞体积**：角色圆盘半径 r_vs_rect，墙矩形 (cx, cy, hw, hh)。
        # 角色半径默认取合同 10，墙半厚取 half_h（=4）。距离 < 半径 → 碰撞。
        r_p = float(spec.player_radius)
        for (yy, l, rr) in seg_rows:
            cx = 0.5 * (l + rr); hw = 0.5 * (rr - l)
            gap_dist = _dist_to_rect(px, py, cx, yy, hw, 4.0) - r_p
            min_clear = gap_dist if min_clear is None else min(min_clear, gap_dist)
            if gap_dist < 0.0:
                collided = True
        _, _, term, trunc, info = env.step(act)
        t += float(spec.dt)
        px, py = float(env.world.player[0]), float(env.world.player[1])
        c = info.get("collision_events", info.get("collisions", 0)) if isinstance(info, dict) else 0
        collisions = max(collisions, int(c or 0))
        if trunc or term:
            break
        reached = math.hypot(px - goal[0], py - goal[1]) <= float(spec.target_radius or 14)
        if reached:
            break
    print(f"场景 {spec.name}: {len(env.spec.patterns)} 道墙  角色半径 {float(spec.player_radius):.0f}"
          f"  墙半厚 4.0")
    if min_clear is not None:
        print(f"  最小间隙 {min_clear:+.1f} 场地单位 (<0 即碰撞)   碰撞={'是' if collided else '否'}")
    print(f"  用时 {t:5.1f} s   碰撞 {collisions}   动作切换 {reversals}   终点 ({px:.0f},{py:.0f})"
          f"   到达={'是' if math.hypot(px-goal[0], py-goal[1]) <= float(spec.target_radius or 14) else '否'}")
    if aim_err:
        v = sorted(aim_err)
        print(f"  瞄准误差 |Δx|: n={len(v)} 中位 {st.median(v):.1f}  p95 {v[int(0.95*(len(v)-1))]:.1f} 场地单位")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
