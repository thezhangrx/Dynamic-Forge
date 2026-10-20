#!/usr/bin/env python3
"""一次跑的完整证据日志（JSONL）。

为什么单独做一个模块：之前每次调参只有最后一份 summary（到了没到、撞了几次），
"墙为什么测错、板子什么时候发的动作、VT 用哪一帧的位姿映射的"这些都没落盘，
所以每轮只能靠猜。这里把四类事实分开写，互不污染：

  meta.json          这次跑的配置 + **虚拟真值**（config.meta: walls_y/gaps_x/...）
  timeline.jsonl     角色轨迹：每步 t_sim / 世界时间 / px,py / 动作 / 真值最近间隙
  board_io.jsonl     板子信号与时间：每条 @POSE 发出（带 seq）、每条 @MOVE 收到
  render.jsonl       图形处理：页面/热力图渲染耗时、窗口 fps
  truth_rows.jsonl   本地真值：每行道墙第一次进入视野时的真值缺口（对照用）
  local_truth.jsonl  本地真值快照（2 Hz）：角色附近 400 单位内的墙矩形
  board_state.txt    板端实际收到的 @STATE（板端测到的墙）——跑完从板子拷回
  board_events.jsonl 板端事件（vt_tick / pause / add_wall / fix_gap / nudge / move）
  board_vt.jsonl     板端 VT 每帧原始值（跑完从板子拷回 tail）
  summary.json       汇总（向后兼容原来那份 platform_*.json）

约定：所有 JSONL 每行一个对象，`t_sim` 是仿真时间，`t`/`t_wall` 是
`time.monotonic()` 秒（同进程内可比）；跨机器用 `pose_seq` 对齐。
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class JsonlFile:
    """行缓冲的 JSONL 写入器（异常不抛，日志不能拖垮主循环）。"""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.path.open("w", buffering=1, encoding="utf-8")
        self.lines = 0
        self.bytes = 0

    def write(self, obj: dict[str, Any]) -> None:
        try:
            text = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
            self._f.write(text + "\n")
            self.lines += 1
            self.bytes += len(text) + 1
        except (OSError, TypeError, ValueError):
            pass

    def close(self) -> None:
        try:
            self._f.close()
        except OSError:
            pass


class RunLog:
    """把一次跑的轨迹/信号/渲染/真值都写进 run 目录。"""

    def __init__(self, run_dir: Path, meta: dict[str, Any]) -> None:
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.timeline = JsonlFile(self.run_dir / "timeline.jsonl")
        self.board_io = JsonlFile(self.run_dir / "board_io.jsonl")
        self.render = JsonlFile(self.run_dir / "render.jsonl")
        self.truth_rows = JsonlFile(self.run_dir / "truth_rows.jsonl")
        self.local_truth = JsonlFile(self.run_dir / "local_truth.jsonl")
        self.events = JsonlFile(self.run_dir / "host_events.jsonl")
        self.t0 = time.monotonic()
        self.meta = dict(meta)
        self.meta.setdefault("started", time.strftime("%Y-%m-%dT%H:%M:%S"))
        self._write_meta()

    # ------------------------------------------------------------------
    def _write_meta(self) -> None:
        (self.run_dir / "meta.json").write_text(
            json.dumps(self.meta, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")

    def wall_t(self) -> float:
        return round(time.monotonic() - self.t0, 4)

    # ------------------------------------------------------------------
    def step(self, **kw: Any) -> None:
        kw["t_wall"] = self.wall_t()
        self.timeline.write(kw)

    def io(self, kind: str, **kw: Any) -> None:
        """kind: pose_sent / move_recv。"""
        rec = {"kind": kind, "t_wall": self.wall_t()}
        rec.update(kw)
        self.board_io.write(rec)

    def frame(self, kind: str, **kw: Any) -> None:
        rec = {"kind": kind, "t_wall": self.wall_t()}
        rec.update(kw)
        self.render.write(rec)

    def event(self, name: str, **kw: Any) -> None:
        rec = {"ev": name, "t_wall": self.wall_t()}
        rec.update(kw)
        self.events.write(rec)

    def close(self) -> None:
        for f in (self.timeline, self.board_io, self.render, self.truth_rows,
                  self.local_truth, self.events):
            f.close()

    def counts(self) -> dict[str, int]:
        return {
            "timeline": self.timeline.lines,
            "board_io": self.board_io.lines,
            "render": self.render.lines,
            "truth_rows": self.truth_rows.lines,
            "local_truth": self.local_truth.lines,
            "host_events": self.events.lines,
        }


# ======================================================================
# 虚拟真值 / 本地真值
# ======================================================================
def virtual_truth(spec) -> dict[str, Any]:
    """config.meta 里的墙/缺口定义 —— 这是"虚拟真值"的权威来源。"""
    meta = dict(getattr(spec, "meta", None) or {})
    rows = []
    ys = [float(v) for v in (meta.get("walls_y") or [])]
    gx = [float(v) for v in (meta.get("gaps_x") or [])]
    gw = [float(v) for v in (meta.get("gap_widths") or [])]
    gs = list(meta.get("gap_sides") or [])
    for i, y in enumerate(ys):
        c = gx[i] if i < len(gx) else None
        w = gw[i] if i < len(gw) else None
        side = gs[i] if i < len(gs) else None
        rows.append({
            "y": round(y, 2),
            "gap_center": None if c is None else round(c, 2),
            "gap_width": None if w is None else round(w, 2),
            "gap_side": side,
            "gap_gl": None if (c is None or w is None) else round(c - 0.5 * w, 2),
            "gap_gr": None if (c is None or w is None) else round(c + 0.5 * w, 2),
        })
    return {"rows": rows, "raw_meta": meta}


def local_truth_rows(walls, field_h: float) -> list[dict[str, Any]]:
    """把 env 里的矩形墙按行聚一下，方便和板端"每行一条"的日志对齐。"""
    rows: dict[float, list[tuple[float, float]]] = {}
    for (cx, cy, hw, hh) in walls:
        key = round(float(cy), 1)
        rows.setdefault(key, []).append((float(cx), float(hw)))
    out = []
    for y, segs in sorted(rows.items()):
        segs.sort()
        #: 一行墙通常由左右两段组成，中间那段空白就是缺口
        gl = gr = None
        if len(segs) >= 2:
            left_end = max(cx + hw for cx, hw in segs if cx < 320.0)
            right_beg = min(cx - hw for cx, hw in segs if cx >= 320.0)
            gl, gr = round(left_end, 1), round(right_beg, 1)
        out.append({
            "y": y,
            "segments": [[round(cx - hw, 1), round(cx + hw, 1)] for cx, hw in segs],
            "gap_gl": gl,
            "gap_gr": gr,
        })
    return out
