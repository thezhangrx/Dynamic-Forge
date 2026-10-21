#!/usr/bin/env python3
"""render_path_potential.py —— 画"板子自己的世界"：势场 + 决策路径折线。

数据来源（全部来自一轮 run_*/ 目录，不需要重跑）：
  board_state.txt   @STATE 逐帧：px py r n_walls [x0 y x1 half 0]*
                    → 板端**已学到**的墙（障碍段）+ 它以为的角色位置
  timeline.jsonl    t_sim px py act dist_goal clear row
                    → 真实轨迹（折线）与每步动作

画什么：
  * 灰白色 = 板端知道的两侧障碍段（每道墙缺口的左右两段）
  * 背景灰度 = 势场（到最近障碍的**距离**，越暗越安全；等价于代价场的定性视图）
  * 青色折线 = 角色真实走过的路径
  * 绿色圆点 = 当前位置；红点 = 目标
  * 动作箭头 + 动作号

用法：
    Env/bin/python core/board/tools/render_path_potential.py <run_dir> \
        --out /tmp/path_potential.mp4 [--fps 20]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

GOAL = (320.0, 3360.0)


def read_state(path: Path) -> list[dict]:
    """@STATE 行 → [{px,py,walls:[(x0,x1,y)]}]（障碍段是水平墙的两段）。"""
    out = []
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        t = line.split()
        if len(t) < 6 or t[0] != "@STATE":
            continue
        try:
            px, py = float(t[2]), float(t[3])
            n = int(t[5])
        except (IndexError, ValueError):
            continue
        vals = t[6:]
        walls = []
        for i in range(n):
            chunk = vals[i * 5:(i + 1) * 5]
            if len(chunk) < 3:
                break
            try:
                x0, y, x1 = float(chunk[0]), float(chunk[1]), float(chunk[2])
            except ValueError:
                continue
            walls.append((min(x0, x1), max(x0, x1), y))
        out.append({"px": px, "py": py, "walls": walls})
    return out


def read_timeline(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="渲染板子世界：势场 + 决策路径")
    ap.add_argument("run_dir")
    ap.add_argument("--out", default="/tmp/path_potential.mp4")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--field-h", type=float, default=3420.0)
    a = ap.parse_args(argv)

    run = Path(a.run_dir)
    states = read_state(run / "board_state.txt")
    tl = read_timeline(run / "timeline.jsonl")
    if not tl:
        print("没有 timeline.jsonl", file=sys.stderr)
        return 2

    W, H = 640, 900                      # 画布：横向 640、纵向压缩到 900（视野跟角色）
    scale = W / 640.0
    half_h = 900.0 / 2.0 / scale         # 可视半高（场地单位）

    writer = None
    n_frames = 0
    #: 用 @STATE 的序号与 timeline 的 t_sim 对齐：@STATE 约 110 条/s、timeline 30 帧/s
    si = 0
    for k in range(0, len(tl), max(1, a.step)):
        row = tl[k]
        py = float(row["py"])
        px = float(row["px"])
        #: 找到同一时刻的 @STATE（用 py 最近者近似对齐，够画图用）
        if states:
            best, bd = si, 1e18
            for j in range(si, min(si + 400, len(states))):
                d = abs(states[j]["py"] - py)
                if d < bd:
                    bd, best = d, j
            si = max(0, best - 5)
            st = states[best]
            walls = st["walls"]
            px_draw, py_draw = st["px"], st["py"]
        else:
            walls, px_draw, py_draw = [], px, py

        #: 视野：纵向以角色为中心
        y0 = py - half_h
        canvas = np.full((H, W, 3), (26, 30, 40), np.uint8)

        def to_px(x: float, y: float) -> tuple[int, int]:
            cx = int(round(x * scale))
            cy = int(round((y - y0) * scale))
            cy = H - 1 - cy                 # 场地上方在画面顶部
            return cx, cy

        # ---- 势场：到最近障碍段的距离（画成灰度）----
        gy, gx = np.mgrid[0:H:8, 0:W:8]
        world_x = gx / scale
        world_y = y0 + (H - 1 - gy) / scale
        dist = np.full(world_x.shape, 1e9, np.float32)
        for (x0, x1, wy) in walls:
            dx = np.maximum(0.0, np.maximum(x0 - world_x, world_x - x1))
            dist = np.minimum(dist, np.hypot(dx, world_y - wy))
        if walls:
            near = np.clip(dist / 120.0, 0.0, 1.0)
            tile = (near * 70 + 18).astype(np.uint8)
            canvas = cv2.resize(tile, (W, H), interpolation=cv2.INTER_NEAREST)
            canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

        # ---- 障碍段（板端已知的墙）----
        for (x0, x1, wy) in walls:
            p0 = to_px(x0, wy)
            p1 = to_px(x1, wy)
            cv2.line(canvas, p0, p1, (200, 190, 160), 3)

        # ---- 真实轨迹 ----
        pts = [to_px(float(r["px"]), float(r["py"])) for r in tl[:k + 1:max(1, a.step)]]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], (230, 200, 60), 2)

        # ---- 角色 / 目标 ----
        c = to_px(px_draw, py_draw)
        cv2.circle(canvas, c, 7, (90, 230, 140), -1)
        g = to_px(GOAL[0], GOAL[1])
        if 0 <= g[1] < H:
            cv2.circle(canvas, g, 9, (80, 80, 230), 2)

        # ---- 动作箭头 ----
        act = int(row.get("act", 0) or 0)
        dirs = {1: (-1, 0), 2: (1, 0), 3: (0, 1), 4: (0, -1),
                5: (-1, 1), 6: (1, 1), 7: (-1, -1), 8: (1, -1)}
        if act in dirs:
            dx, dy = dirs[act]
            tip = (int(c[0] + dx * 26), int(c[1] - dy * 26))
            cv2.arrowedLine(canvas, c, tip, (90, 230, 140), 2, tipLength=0.35)

        txt = (f"t={float(row.get('t_sim', 0)):.1f}s  py={py:6.0f}  act={act}  "
               f"known_rows={len({round(w[2]) for w in walls})}")
        cv2.putText(canvas, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (235, 235, 235), 1, cv2.LINE_AA)

        if writer is None:
            writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                     a.fps, (W, H))
        writer.write(canvas)
        n_frames += 1

    if writer is not None:
        writer.release()
    print(f"写了 {n_frames} 帧 → {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
