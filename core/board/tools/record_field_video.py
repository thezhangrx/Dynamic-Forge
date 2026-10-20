#!/usr/bin/env python3
"""录制"板子决策 + 势场"视频：从仿真真值流 + 板子动作流出片。

两种模式：
  * 全幅（`--view-h 0`）：640×480，整场墙都在画面里；
  * **视角跟随**（`--view-h H`）：640×H 的视口，随角色 y 滚动；势场只在视口内
    用**可见的墙**重建（和板子 `field_decision_cli --view-h` 同一算法）。

    python3 record_field_video.py --state /tmp/m1_state.txt --actions /tmp/m1_actions.txt \
            --out /tmp/board_field.mp4 --field-h 4100 --view-h 240 --goal 320,4060

画面：
  热力图 = 视口内的代价场（目标在视口外时，以视口顶行为多源边界条件）
  橙色   = 墙        白线 = 轨迹（随视口滚动）     白圈 = 角色
  绿圈   = 目标（在视口内）；视口外则用顶部箭头 + 距离提示
  HUD    = act / 切换次数 / 最小间隙 / 视口 y
"""
from __future__ import annotations
import argparse, heapq, math, time
from pathlib import Path

import numpy as np
import cv2

CELL, INFLATE, R_P = 10.0, 13.0, 10.0


def _seed_goal(cost, blocked, nx, ny, view_y, view_h, goal):
    """把目标/边界条件写进 cost 并返回种子列表（与 C++ 版一致）。"""
    gx, gy = goal
    view_y1 = view_y + view_h
    seeds = []

    def free(ix, iy):
        return 0 <= ix < nx and 0 <= iy < ny and not blocked[ix, iy]

    def seed(ix, iy, c):
        if free(ix, iy) and c < cost[ix, iy]:
            cost[ix, iy] = c
            seeds.append((c, ix, iy))

    if view_y <= gy <= view_y1:
        # 目标在视口内：最近自由格
        bx = by = -1
        bd = float("inf")
        for iy in range(ny):
            for ix in range(nx):
                if not free(ix, iy):
                    continue
                d = math.hypot((ix + 0.5) * CELL - gx,
                               view_y + (iy + 0.5) * CELL - gy)
                if d < bd:
                    bd, bx, by = d, ix, iy
        if bx >= 0:
            seed(bx, by, 0.0)
    elif gy > view_y1:
        extra = gy - view_y1
        iy = ny - 1
        any_seed = False
        for ix in range(nx):
            if free(ix, iy):
                seed(ix, iy, extra + abs((ix + 0.5) * CELL - gx))
                any_seed = True
        if not any_seed:
            for iy in range(ny - 2, -1, -1):
                for ix in range(nx):
                    if free(ix, iy):
                        seed(ix, iy, extra + abs((ix + 0.5) * CELL - gx)
                             + (ny - 1 - iy) * CELL)
                        any_seed = True
                if any_seed:
                    break
    else:
        extra = view_y - gy
        iy = 0
        any_seed = False
        for ix in range(nx):
            if free(ix, iy):
                seed(ix, iy, extra + abs((ix + 0.5) * CELL - gx))
                any_seed = True
        if not any_seed:
            for iy in range(1, ny):
                for ix in range(nx):
                    if free(ix, iy):
                        seed(ix, iy, extra + abs((ix + 0.5) * CELL - gx) + iy * CELL)
                        any_seed = True
                if any_seed:
                    break
    return seeds


def build_cost(walls, fw, view_y, view_h, goal):
    """视口内 Dijkstra 代价场。walls 里视口外的墙自动忽略。"""
    nx, ny = int(fw / CELL), max(1, int(round(view_h / CELL)))
    blocked = np.zeros((nx, ny), bool)
    for (cx, cy, hw, hh) in walls:
        if cy + hh < view_y - 1e-9 or cy - hh > view_y + view_h + 1e-9:
            continue
        for ix in range(nx):
            x = (ix + .5) * CELL
            if abs(x - cx) > hw + INFLATE:
                continue
            for iy in range(ny):
                if abs((view_y + (iy + .5) * CELL) - cy) <= hh + INFLATE:
                    blocked[ix, iy] = True
    cost = np.full((nx, ny), np.inf)
    seeds = _seed_goal(cost, blocked, nx, ny, view_y, view_h, goal)
    pq = [(c, ix, iy) for (c, ix, iy) in seeds]
    heapq.heapify(pq)
    while pq:
        c, ix, iy = heapq.heappop(pq)
        if c > cost[ix, iy]:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not dx and not dy:
                    continue
                jx, jy = ix + dx, iy + dy
                if not (0 <= jx < nx and 0 <= jy < ny) or blocked[jx, jy]:
                    continue
                nc = c + math.hypot(dx, dy) * CELL
                if nc < cost[jx, jy]:
                    cost[jx, jy] = nc
                    heapq.heappush(pq, (nc, jx, jy))
    return cost, nx, ny


def make_base(walls, goal, fw, fh, view_y, view_h):
    cost, nx, ny = build_cost(walls, fw, view_y, view_h, goal)
    # ⚠ cost 是 [ix, iy]（x 在前）；cv2 把 axis0 当行（y），必须转置再画，
    #   否则墙的膨胀带会变成**横着的红矩形竖过来**（之前视频里"闪烁的红色矩形"）。
    cost_xy = cost.T                       # (ny, nx) = (y, x)
    cm = np.flipud(np.where(np.isfinite(cost_xy), cost_xy, np.nan))
    finite = np.isfinite(cm)
    if finite.any():
        mn = float(np.nanmin(cm)); mx = float(np.nanmax(cm))
        denom = (mx - mn) or 1.0
        norm = np.where(finite, (cm - mn) / denom, 1.0)
    else:
        norm = np.ones_like(cm)
    base = cv2.resize(cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET),
                      (int(fw), int(round(view_h))), interpolation=cv2.INTER_NEAREST)
    for (cx, cy, hw, hh) in walls:
        if cy + hh < view_y or cy - hh > view_y + view_h:
            continue
        top = int(round(view_h - (cy - view_y + hh)))
        bot = int(round(view_h - (cy - view_y - hh)))
        cv2.rectangle(base, (int(cx - hw), top), (int(cx + hw), bot), (0, 120, 255), -1)
    # 目标：视口内画绿圈；视口外在上/下边缘画箭头
    if view_y <= goal[1] <= view_y + view_h:
        cv2.circle(base, (int(goal[0]), int(round(view_h - (goal[1] - view_y)))),
                   24, (60, 230, 120), 2)
        cv2.putText(base, "GOAL", (int(goal[0]) - 20, max(12, int(view_h - (goal[1] - view_y)) - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 230, 120), 1)
    elif goal[1] > view_y + view_h:
        x = int(min(max(goal[0], 8), fw - 8))
        cv2.fillPoly(base, [np.array([[x, 4], [x - 10, 24], [x + 10, 24]], np.int32)],
                     (60, 230, 120))
        cv2.putText(base, f"goal up {goal[1] - view_y - view_h:.0f}",
                    (max(8, min(int(goal[0]) + 14, int(fw) - 120)), 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 230, 120), 1)
    else:
        x = int(min(max(goal[0], 8), fw - 8))
        cv2.fillPoly(base, [np.array([[x, int(view_h) - 4], [x - 10, int(view_h) - 24],
                                      [x + 10, int(view_h) - 24]], np.int32)],
                     (60, 230, 120))
    return base


def parse_state(line):
    a = line.split()
    if len(a) < 6 or not a[0].startswith("@STATE"):
        return None
    px, py, pr, n = float(a[2]), float(a[3]), float(a[4]), int(a[5])
    walls = []
    t = a[6:]
    for i in range(n):
        if len(t) < i * 5 + 4:
            break
        walls.append(tuple(float(v) for v in t[i * 5:i * 5 + 4]))
    return px, py, pr, walls


def new_actions(path, n_seen):
    """返回（新增动作码列表, 新的已处理行数）。只消费完整行、按行数增量。"""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except (FileNotFoundError, UnicodeDecodeError):
        return [], n_seen
    cut = text.rfind("\n")
    if cut < 0:
        return [], n_seen
    lines = text[:cut].splitlines()
    out = []
    for ln in lines[n_seen:]:
        if ln.startswith("@MOVE"):
            try:
                out.append(int(ln.split()[1]))
            except (IndexError, ValueError):
                pass
    return out, len(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--actions", required=True)
    ap.add_argument("--out", default="/tmp/board_field.mp4")
    ap.add_argument("--seconds", type=float, default=30.0, help="最长墙钟秒数（到达会提前结束）")
    ap.add_argument("--goal", default="320,470")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--field-h", type=float, default=480.0)
    ap.add_argument("--view-h", type=float, default=0.0,
                    help="0 = 全幅；>0 = 跟随视口高度")
    a = ap.parse_args()
    goal = tuple(float(v) for v in a.goal.split(","))
    field_h = float(a.field_h)
    view_h = float(a.view_h) if a.view_h and a.view_h > 0 else field_h
    follow = bool(a.view_h and a.view_h > 0)

    base = None
    vw = None
    path_world = []
    act, switches, mc = 0, 0, None
    n_act_lines = 0
    n_frames = 0
    view_y = 0.0
    t0 = time.time()
    f = None
    try:
        f = open(a.state, "r", encoding="utf-8")
        while time.time() - t0 < a.seconds:
            pos_before = f.tell()
            line = f.readline()
            if not line:
                time.sleep(0.01)
                continue
            if not line.endswith("\n"):
                f.seek(pos_before)
                time.sleep(0.01)
                continue
            st = parse_state(line)
            if st is None:
                continue
            px, py, _pr, walls = st
            if follow:
                view_y = min(max(py - view_h * 0.5, 0.0), max(0.0, field_h - view_h))
                walls = [w for w in walls
                         if w[1] + w[3] >= view_y and w[1] - w[3] <= view_y + view_h]
                base = make_base(walls, goal, 640.0, field_h, view_y, view_h)
            elif base is None and walls:
                view_y = 0.0
                base = make_base(walls, goal, 640.0, field_h, 0.0, field_h)

            new, n_act_lines = new_actions(a.actions, n_act_lines)
            for cmd in new:
                if cmd != act:
                    switches += 1
                act = cmd

            if base is None:
                continue
            for (cx, cy, hw, hh) in walls:
                d = math.hypot(max(abs(px - cx) - hw, 0), max(abs(py - cy) - hh, 0)) - R_P
                mc = d if mc is None else min(mc, d)
            path_world.append((px, py))
            img = base.copy()
            for i in range(1, len(path_world)):
                x1, y1 = path_world[i - 1]
                x2, y2 = path_world[i]
                sy1 = view_h - (y1 - view_y)
                sy2 = view_h - (y2 - view_y)
                if max(sy1, sy2) < -8 or min(sy1, sy2) > view_h + 8:
                    continue
                cv2.line(img, (int(x1), int(sy1)), (int(x2), int(sy2)), (255, 255, 255), 2)
            psy = view_h - (py - view_y)
            cv2.circle(img, (int(px), int(psy)), int(R_P), (255, 255, 255), -1)
            cv2.putText(img,
                        f"board M1  act={act} sw={switches} clear={mc:+.1f} "
                        f"view_y={view_y:.0f}",
                        (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
            cv2.putText(img, "orange=wall  dark-red=inflate(+13)  white=player  green=goal",
                        (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (225, 225, 225), 1)
            if vw is None:
                vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                     float(a.fps), (img.shape[1], img.shape[0]))
            vw.write(img)
            n_frames += 1
            if math.hypot(px - goal[0], py - goal[1]) <= 24:
                break
    finally:
        if vw is not None:
            vw.release()
        if f is not None:
            f.close()
    print(f"输出 {a.out}  帧 {n_frames}  切换 {switches}  最小间隙 {mc}")


if __name__ == "__main__":
    main()
