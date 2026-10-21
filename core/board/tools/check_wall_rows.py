#!/usr/bin/env python3
"""check_wall_rows.py —— 板端"墙行识别"到底准不准（离线，不改任何东西）。

为什么需要
----------
`audit_crossings.py` 只回答"穿墙时这道墙进表没有"，回答不了"进表的那一行
对不对"。实测出现过同一道墙在不同 py 被记成 346/359/398/417/427 五道的情况
（r4/r5 都因此死锁），这种"错行"在覆盖率和碰撞指标上都看不出来。

怎么算
------
板端每一帧的墙行是**页面坐标**（0..480，y 向上，跟随视口把角色钉在正中）。
主机真值知道那一刻平台在哪（`@POSE`）。于是

    世界 y = (py - 240) + scale*vy + bias         （标定系数，与板端一致）

再和 configs 里的真值墙行比对。只统计**静止帧**（py 在 ±0.2 s 内不变）——
运动时画面本身有 0.5 s 级的管线延时，那属于"时序"，不是"行识别"误差。

    Env/bin/python core/board/tools/check_wall_rows.py data/fpga/loop_logs/run_xxx
"""
from __future__ import annotations

import json
import statistics as st
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    run = Path(sys.argv[1])
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    truth = [float(r["y"]) for r in meta.get("virtual_truth") or []]
    if not truth:
        print("meta 里没有 virtual_truth")
        return 2
    calib = {}
    try:
        calib = json.loads(Path(meta["args"]["calib"]).read_text(encoding="utf-8"))
    except (OSError, KeyError, ValueError):
        pass
    scale = float(calib.get("scale", 1.0))
    bias = float(calib.get("y_bias", calib.get("bias", 0.0)))

    vt_path = run / "board_vt.jsonl"
    ev_path = run / "board_events.jsonl"
    if not vt_path.exists() or not ev_path.exists():
        print(f"缺 {vt_path.name} 或 {ev_path.name}")
        return 2

    #: 按**帧号**对齐：vt.jsonl 的 `t` 是板端 CLOCK_MONOTONIC 绝对值，
    #: board_events 的 `t` 是 agent 启动后的相对时间 —— 两者不能直接比。
    #: 事件里的 `frame` 和 vt.jsonl 的 `frame` 是同一个，按它对位姿就行。
    frame_pos: dict[int, tuple[float, float]] = {}
    ordered: list[tuple[int, float]] = []
    for line in ev_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if '"vt_tick"' not in line:
            continue
        try:
            e = json.loads(line)
        except ValueError:
            continue
        f = e.get("frame")
        if f is None:
            continue
        frame_pos[int(f)] = (float(e["py"]), float(e["px"]))
        ordered.append((int(f), float(e["py"])))
    ordered.sort()

    def still(frame: int, back: int = 20) -> float | None:
        """这一帧之前 back 帧内 py 没动 → 返回 py（画面与位姿同一时刻）。"""
        i = next((k for k, (f, _) in enumerate(ordered) if f >= frame), None)
        if i is None or i < back:
            return None
        py = ordered[i][1]
        for k in range(i - back, i + 1):
            if abs(ordered[k][1] - py) > 0.5:
                return None
        return py

    errs: list[float] = []
    rows_seen = 0
    for line in vt_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        fr = r.get("frame")
        if fr is None:
            continue
        py = still(int(fr))
        if py is None:
            continue
        for w in r.get("walls") or []:
            vy = float(w.get("y", 0.0))
            wy = (py - 240.0) + scale * vy + bias
            tgt = min(truth, key=lambda q: abs(q - wy))
            if abs(tgt - wy) > 400.0:      # 视野外的行不参与统计
                continue
            rows_seen += 1
            errs.append(wy - tgt)
    if not errs:
        print("没有可用的静止帧（跑得太短？）")
        return 1
    a = sorted(errs)
    n = len(a)
    big = sum(1 for v in a if abs(v) > 20.0)
    print(f"静止帧墙行样本 {n}（{rows_seen} 次观测）")
    print(f"  误差 median {st.median(a):+.1f} px  "
          f"p10 {a[n // 10]:+.1f}  p90 {a[9 * n // 10]:+.1f}  "
          f"max|e| {max(abs(a[0]), abs(a[-1])):.1f}")
    print(f"  |误差|>20 px 的比例 {100.0 * big / n:.1f}%")
    ok = abs(st.median(a)) <= 6.0 and a[9 * n // 10] <= 20.0
    print("  判定：" + ("行识别可用" if ok else "行识别不合格"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
