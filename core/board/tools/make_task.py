#!/usr/bin/env python3
"""把场景里的"任务 + 自己的信息"抽成板上用的 task.json。

**这不是真值下发**，它是 AGENTS.md R2（2026-10-23 版）明确允许的四类东西：

    spawn / goal / goal_r        任务设定 + 出生点
    field / radius / speed / dt  自己的运动学模型（一步走多远）
    page                         定位基准（白框内的页面是世界坐标里 640×480）

**不含**任何墙/缺口的坐标 —— 那些只能由板子用相机看出来。

用法：
    python3 core/board/tools/make_task.py configs/20_walls_alt_24_dy140_y120.json \
        -o /tmp/task.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def build(scenario: str | Path, page: tuple[float, float] = (640.0, 480.0)) -> dict:
    cfg = json.loads(Path(scenario).read_text(encoding="utf-8"))
    meta = cfg.get("meta") or {}
    spawn = meta.get("spawn") or [cfg.get("player_x"), cfg.get("player_y")]
    goal = meta.get("goal") or [cfg.get("target_x"), cfg.get("target_y")]
    task = {
        "scenario": cfg.get("name"),
        "spawn": [float(spawn[0]), float(spawn[1])],
        "goal": [float(goal[0]), float(goal[1])],
        "goal_r": float(cfg.get("target_radius", 24.0)),
        "field": [float(cfg.get("field_w", 640.0)), float(cfg.get("field_h", 3420.0))],
        "radius": float(cfg.get("player_radius", 10.0)),
        "speed": float(cfg.get("player_speed", 200.0)),
        "dt": float(cfg.get("dt", 1.0 / 120.0)),
        "page": [float(page[0]), float(page[1])],
    }
    #: 硬性自检：**不许**把墙带进去
    for k in ("walls", "walls_y", "gaps_x", "gap_widths", "gap_sides", "patterns"):
        assert k not in task, k
    return task


def main() -> int:
    ap = argparse.ArgumentParser(description="生成板上 task.json（无墙）")
    ap.add_argument("scenario")
    ap.add_argument("-o", "--out", default="/tmp/task.json")
    a = ap.parse_args()
    task = build(a.scenario)
    Path(a.out).write_text(json.dumps(task, ensure_ascii=False, indent=2),
                           encoding="utf-8")
    print(json.dumps(task, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
