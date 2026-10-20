#!/usr/bin/env python3
"""本地 Python 上用**势场**（cpu.potential_field）跑静态墙平台。

与 test_static_local.py 的区别：
  · 目标 = **最终目标点**（绿圈 320,470），不再用"下一道缺口"；
  · 决策 = `plan_field()` 的势场下降方向（和 `cpu_field_demo.py --goal auto` 同一套）；
  · 墙 = 静态矩形（静态墙平台），间距拉大后更容易看出势场的行为。
"""
from __future__ import annotations
import argparse, math, sys
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.env import BulletHellEnv
from cpu.potential_field import FieldConfig, FieldMemory, FieldView, plan_field
from cpu.gap_avoid import Rect
#: `plan_field` 的 `rects` 要的是 `cpu.gap_avoid.Rect`（有 half_w/half_h 等字段的
#: dataclass），不是元组 —— 传元组会在 `_raster_rect` 里报 'tuple' has no half_w。

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from test_static_local import pick_action   # 复用"方向→动作码"的量化


def _dist_rect(px, py, cx, cy, hw, hh):
    return math.hypot(max(abs(px-cx)-hw, 0.0), max(abs(py-cy)-hh, 0.0))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--max-seconds", type=float, default=60.0)
    a = ap.parse_args(argv)
    spec = ScenarioSpec.load(a.scenario)
    goal = (float(spec.target_x or 320.0), float(spec.target_y or 470.0))
    env = BulletHellEnv(spec, seed=spec.seed); env.reset(seed=spec.seed)
    mem = FieldMemory(); cfg = FieldConfig()
    px, py = float(env.world.player[0]), float(env.world.player[1])
    t, prev_act, streak, switches = 0.0, 0, 0, 0
    min_clear = None; collided = False; reached = False
    for step in range(int(a.max_seconds / float(spec.dt))):
        pool = env.world.state.bullets
        rects = []
        for i in range(int(pool.count)):
            if not bool(pool.alive[i]):
                continue
            rects.append(Rect(int(pool.id[i]), float(pool.x[i]), float(pool.y[i]),
                              float(pool.vx[i]), float(pool.vy[i]),
                              float(pool.half_w[i]), float(pool.half_h[i]),
                              float(pool.rotation[i]), 0.0))
        view = FieldView(player=[px, py, 0.0, 0.0, float(spec.player_radius),
                                 float(spec.player_speed), 1.0],
                         rects=rects, field_w=spec.field_w, field_h=spec.field_h,
                         goal=goal, step_index=step, dt=float(spec.dt))
        dec = plan_field(view, mem, cfg)
        want = pick_action(dec.wx, dec.wy, 0.05)
        if want == prev_act:
            streak = 0; act = prev_act
        else:
            streak += 1; act = want if streak >= 2 else prev_act
        if act != prev_act:
            switches += 1; prev_act = act
        r_p = float(spec.player_radius)
        for rr in rects:
            d = _dist_rect(px, py, rr.x, rr.y, rr.half_w, rr.half_h) - r_p
            min_clear = d if min_clear is None else min(min_clear, d)
            if d < 0:
                collided = True
        env.step(act)
        t += float(spec.dt)
        px, py = float(env.world.player[0]), float(env.world.player[1])
        if math.hypot(px-goal[0], py-goal[1]) <= float(spec.target_radius or 14):
            reached = True
            break
    print(f"势场本地测试  场景 {spec.name}（{len(spec.patterns)} 道墙）")
    print(f"  用时 {t:5.1f} s   到达={'是' if reached else '否'}   终点 ({px:.0f},{py:.0f})")
    print(f"  动作切换 {switches}   最小间隙 {min_clear:+.1f} 场地单位   碰撞={'是' if collided else '否'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
