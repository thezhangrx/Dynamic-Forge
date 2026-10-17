#!/usr/bin/env python
"""**真值对照打分**：板子报的几何 vs 仿真真值（场地单位）。

为什么需要它：闭环里有 13 次碰撞时，单看板子日志无法判断是"几何报歪了"
还是"决策错了"。这个脚本把两边的数放在**同一个坐标系**里比，并给出归类。

对齐方式（不靠时间戳，靠状态）：板子日志里每条 `[注入]` 都带着它当时用的
角色位置；在仿真真值序列里找**角色位置最近**的那一帧，认为它们说的是同一时刻。
摆拍式的时间对齐在 25 fps vs 12 fps 的双流上不可靠，而角色位置是连续的。

输入：
    sim：`bullet_sim ... --state-log state.jsonl`（每帧一行：角色 + 所有矩形障碍）
    板子：`cam_live.sh ... --verbose` 的 stdout（含 `[注入] ...` 行）
输出：
    角色/门中心/门宽 的误差统计，以及"门的 y 带与真值墙带是否同一条"的归类。
用法：
    Env/bin/python core/board/tools/score_vs_sim.py /tmp/state.jsonl /tmp/mv.txt
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

FIELD_W = 640.0


def load_sim(path: Path) -> list:
    frames = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            d = json.loads(line)
        except Exception:
            continue
        pl = d.get("player") or [0, 0, 0]
        walls = [o for o in (d.get("obstacles") or []) if len(o) >= 5 and o[2] > o[3]]
        frames.append({"player": pl, "walls": walls,
                       "ba": int(d.get("board_actions", -1))})
    return frames


def bands_of(walls: list) -> list:
    """把同一行上的墙段聚成"带"，并算出带的缺口区间（段之间的最大空隙）。"""
    bands = []
    for w in sorted(walls, key=lambda r: r[1]):
        x, y, hw, hh, _rot = w[:5]
        placed = False
        for b in bands:
            if abs(b["y"] - y) <= max(2.0, 0.5 * (b["hh"] + hh)):
                b["segs"].append((x - hw, x + hw))
                b["hh"] = 0.5 * (b["hh"] + hh)
                placed = True
                break
        if not placed:
            bands.append({"y": y, "hh": hh, "segs": [(x - hw, x + hw)]})
    for b in bands:
        b["segs"].sort()
        gaps = []
        for i in range(len(b["segs"]) - 1):
            a, c = b["segs"][i][1], b["segs"][i + 1][0]
            if c - a > 1.0:
                gaps.append((a, c))
        b["gaps"] = gaps
    return bands


INJ = re.compile(r"\[注入#(?P<idx>[0-9]+)\] 角色=\((?P<px>[-+0-9.]+),(?P<py>[-+0-9.]+)\) r=(?P<pr>[0-9.]+) "
                 r"\| 门=\((?P<cx>[-+0-9.]+),(?P<cy>[-+0-9.]+)\) 宽=(?P<w>[0-9.]+) "
                 r"带厚=(?P<hh>[0-9.]+) \| 左墙 x=(?P<lw>[-+0-9.]+)±(?P<lwh>[0-9.]+) "
                 r"右墙 x=(?P<rw>[-+0-9.]+)±(?P<rwh>[0-9.]+)")


def load_board(path: Path) -> list:
    out = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = INJ.search(line)
        if m:
            out.append({k: float(v) for k, v in m.groupdict().items()})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("sim", help="--state-log 产出的 jsonl")
    ap.add_argument("board", help="板子的 verbose 输出")
    args = ap.parse_args(argv)

    sim = load_sim(Path(args.sim))
    rows = load_board(Path(args.board))
    if not sim or not rows:
        print(f"数据不足：sim {len(sim)} 帧，板子 {len(rows)} 条注入")
        return 1

    for f in sim:
        f["bands"] = bands_of(f["walls"])

    errs_cx, errs_w, errs_cy, same_band, no_band = [], [], [], 0, 0
    #: 每条注入行到**任意**真值缺口的最小 2D 距离（场地单位）。
    #: 为什么要有它：上一版只在"最近的那条带"里找缺口，一旦选错带，误差就被算到
    #: 别的墙上（掩盖真实分布）。这个最近距离回答的是"板子报的门，在真值里
    #: 到底有没有对应的缺口" —— 是判据质量，而不是选择质量。
    near2d = []
    for r in rows:
        # 按角色位置找最近的真值帧
        # 配对键优先用"已收到的板子行数"（跨机器不用对时钟）；
        # 老日志没有这个字段时退回"角色位置最近"。
        _cand = [f for f in sim if f.get("ba", -1) >= 0]
        if _cand:
            k = int(r.get("idx", 0))
            best = min(_cand, key=lambda f: abs(f["ba"] - k))
        else:
            best = min(sim, key=lambda f: abs(f["player"][0] - r["px"])
                       + abs(f["player"][1] - r["py"]))
        if not best["bands"]:
            no_band += 1
            continue
        b = min(best["bands"], key=lambda b: abs(b["y"] - r["cy"]))
        # 真值缺口：该带上最靠近板子门心的那个空隙
        if not b["gaps"]:
            no_band += 1
            continue
        gap = min(b["gaps"], key=lambda g: abs(0.5 * (g[0] + g[1]) - r["cx"]))
        tcx = 0.5 * (gap[0] + gap[1])
        tw = gap[1] - gap[0]
        _d = min((((r["cx"] - 0.5 * (g[0] + g[1])) ** 2 +
                   (r["cy"] - b2["y"]) ** 2) ** 0.5)
                 for b2 in best["bands"] for g in b2["gaps"]) if any(b2["gaps"] for b2 in best["bands"]) else 1e9
        near2d.append(_d)
        errs_cx.append(r["cx"] - tcx)
        errs_w.append(r["w"] - tw)
        errs_cy.append(r["cy"] - b["y"])
        if abs(r["cy"] - b["y"]) <= b["hh"] + r["hh"]:
            same_band += 1

    n = max(1, len(errs_cx))
    mean = lambda v: sum(v) / len(v) if v else 0.0
    p95 = lambda v: sorted(v)[min(len(v) - 1, int(0.95 * len(v)))] if v else 0.0
    print(f"配对 {len(errs_cx)} 条（真值帧 {len(sim)}，注入行 {len(rows)}，无可用带 {no_band}）")
    print(f"门中心误差：均值 {mean(errs_cx):+.1f}  中位 {sorted(errs_cx)[len(errs_cx)//2] if errs_cx else 0:+.1f} "
          f"|P95| {abs(p95([abs(v) for v in errs_cx])):.1f}")
    print(f"门宽误差  ：均值 {mean(errs_w):+.1f}  P95 {p95([abs(v) for v in errs_w]):.1f}")
    print(f"门 y 误差  ：均值 {mean(errs_cy):+.1f}  P95 {p95([abs(v) for v in errs_cy]):.1f}")
    ok30 = sum(1 for d in near2d if d <= 30.0)
    print(f"到最近真值缺口的 2D 距离：中位 {sorted(near2d)[len(near2d)//2] if near2d else 0:.1f} "
          f"P95 {p95(near2d):.1f} | ≤30 单位的占 {ok30}/{n} = {100.0*ok30/n:.0f}%")
    print(f"门的 y 带与真值墙带一致：{same_band}/{n} = {100.0*same_band/n:.0f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
