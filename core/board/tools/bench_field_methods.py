#!/usr/bin/env python3
"""多方法对比：M1 Dijkstra 代价场 / M4 原始势场+随机逃逸（M2/M3 待加）。

统一 runner：同一场景、同一指标、多个起点。
指标：到达 / 用时 / 碰撞(最小间隙<0) / 最小间隙 / 动作切换 / 单帧耗时。
"""
from __future__ import annotations
import argparse, heapq, math, random, sys, time
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.env import BulletHellEnv
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from test_static_local import pick_action

CELL = 10.0
INFLATE = 13.0          # 角色半径 10 + 安全余量 3


def _dist_rect(px, py, cx, cy, hw, hh):
    return math.hypot(max(abs(px-cx)-hw, 0.0), max(abs(py-cy)-hh, 0.0))


def walls_of(env):
    pool = env.world.state.bullets
    out = []
    for i in range(int(pool.count)):
        if bool(pool.alive[i]):
            out.append((float(pool.x[i]), float(pool.y[i]),
                        float(pool.half_w[i]), float(pool.half_h[i])))
    return out


def build_cost(walls, fw, fh, goal):
    """Dijkstra：从目标反向松弛，得到每格到目标的代价（不可达 = inf）。"""
    nx, ny = int(round(fw / CELL)), int(round(fh / CELL))
    blocked = [[False] * ny for _ in range(nx)]
    for (cx, cy, hw, hh) in walls:
        for ix in range(nx):
            x = (ix + 0.5) * CELL
            if abs(x - cx) > hw + INFLATE:
                continue
            for iy in range(ny):
                y = (iy + 0.5) * CELL
                if abs(y - cy) <= hh + INFLATE:
                    blocked[ix][iy] = True
    cost = [[float("inf")] * ny for _ in range(nx)]
    gx, gy = int(goal[0] / CELL), int(goal[1] / CELL)
    gx = max(0, min(nx - 1, gx)); gy = max(0, min(ny - 1, gy))
    cost[gx][gy] = 0.0
    pq = [(0.0, gx, gy)]
    while pq:
        c, ix, iy = heapq.heappop(pq)
        if c > cost[ix][iy]:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                jx, jy = ix + dx, iy + dy
                if dx == 0 and dy == 0 or not (0 <= jx < nx and 0 <= jy < ny):
                    continue
                if blocked[jx][jy]:
                    continue
                nc = c + math.hypot(dx, dy) * CELL
                if nc < cost[jx][jy]:
                    cost[jx][jy] = nc
                    heapq.heappush(pq, (nc, jx, jy))
    return cost, nx, ny, blocked


def m1_cmd(cost, nx, ny, px, py, fw, fh):
    ix = max(0, min(nx - 1, int(px / CELL))); iy = max(0, min(ny - 1, int(py / CELL)))
    best, bd = None, float("inf")
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            jx, jy = ix + dx, iy + dy
            if not (0 <= jx < nx and 0 <= jy < ny):
                continue
            c = cost[jx][jy]
            if c < bd:
                bd, best = c, (jx, jy)
    if best is None or bd == float("inf"):
        return 0.0, 0.0, None
    tx, ty = (best[0] + 0.5) * CELL, (best[1] + 0.5) * CELL
    vx, vy = tx - px, ty - py
    n = math.hypot(vx, vy) or 1.0
    return vx / n, vy / n, (tx, ty)


def run(method, spec, start, max_s=60.0, rng=None):
    # **真正的起点注入**：以前用 env.world.set_player（不存在），三次起点结果一样。
    spec = spec.replaced(player_x=float(start[0]), player_y=float(start[1]))
    env = BulletHellEnv(spec, seed=spec.seed); env.reset(seed=spec.seed)
    goal = (float(spec.target_x or 320.0), float(spec.target_y or 470.0))
    px, py = float(env.world.player[0]), float(env.world.player[1])
    walls = walls_of(env)
    cost = nx = ny = None
    if method == "m1":
        cost, nx, ny, _ = build_cost(walls, spec.field_w, spec.field_h, goal)
    prev_act, streak, switches = 0, 0, 0
    ema = (0.0, 0.0)
    min_clear, collided, reached = None, False, False
    t, frame_ms, frames, stuck = 0.0, 0.0, 0, 0
    last_pos = (px, py)
    while t < max_s:
        t0 = time.perf_counter()
        if method == "m1":
            vx, vy, _ = m1_cmd(cost, nx, ny, px, py, spec.field_w, spec.field_h)
        else:                      # m4: 目标方向 + 墙斥力（简化人工势场）
            vx, vy = goal[0] - px, goal[1] - py
            n = math.hypot(vx, vy) or 1.0
            vx, vy = vx / n, vy / n
            for (cx, cy, hw, hh) in walls:
                d = _dist_rect(px, py, cx, cy, hw, hh)
                if 0 < d < 60:
                    ux, uy = px - cx, py - cy
                    m = math.hypot(ux, uy) or 1.0
                    k = (60 - d) / 60 * 2.0
                    vx += k * ux / m; vy += k * uy / m
            # 随机逃逸：连续 20 帧位移很小 → 切向随机推
            if math.hypot(px - last_pos[0], py - last_pos[1]) < 1.0:
                stuck += 1
            else:
                stuck = 0; last_pos = (px, py)
            if stuck >= 20:
                ang = rng.uniform(0, 2 * math.pi)
                vx += 1.2 * math.cos(ang); vy += 1.2 * math.sin(ang)
        ema = (0.7 * ema[0] + 0.3 * vx, 0.7 * ema[1] + 0.3 * vy)
        want = pick_action(ema[0], ema[1], 0.05)
        if want == prev_act:
            streak = 0; act = prev_act
        else:
            streak += 1; act = want if streak >= 2 else prev_act
        if act != prev_act:
            switches += 1; prev_act = act
        for (cx, cy, hw, hh) in walls:
            d = _dist_rect(px, py, cx, cy, hw, hh) - float(spec.player_radius)
            min_clear = d if min_clear is None else min(min_clear, d)
            if d < 0:
                collided = True
        env.step(act)
        frame_ms += (time.perf_counter() - t0) * 1e3; frames += 1
        t += float(spec.dt)
        px, py = float(env.world.player[0]), float(env.world.player[1])
        if math.hypot(px - goal[0], py - goal[1]) <= float(spec.target_radius or 14):
            reached = True; break
    return dict(reached=reached, t=t, collided=collided, clear=min_clear,
                switches=switches, ms=frame_ms / max(1, frames))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--methods", default="m1,m4")
    ap.add_argument("--max-seconds", type=float, default=45.0)
    a = ap.parse_args(argv)
    spec = ScenarioSpec.load(a.scenario)
    starts = [(320.0, 60.0), (80.0, 60.0), (560.0, 60.0)]
    print(f"场景 {spec.name}  墙 {len(spec.patterns)} 道  目标 {spec.target_x, spec.target_y}")
    print(f"{'方法':4s} {'起点':>10s}  {'到达':4s} {'用时':>6s} {'碰撞':4s} {'最小间隙':>8s} {'切换':>5s} {'ms':>6s}")
    for m in a.methods.split(","):
        for s in starts:
            r = run(m, spec, s, a.max_seconds, random.Random(0))
            print(f"{m:4s} {str(s):>10s}  {'是' if r['reached'] else '否':4s} {r['t']:6.1f} "
                  f"{'是' if r['collided'] else '否':4s} {(r['clear'] if r['clear'] is not None else float('nan')):8.1f} "
                  f"{r['switches']:5d} {r['ms']:6.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
