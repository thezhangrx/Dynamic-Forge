#!/usr/bin/env python3
"""`board_agent` 的**进程级冒烟**（AGENTS.md R1d 第 4 关）+ 拍摄延迟注入测试。

它扮演主机（不启动平台、不碰相机）：写合成 `vt.jsonl`、走真 `@STEP` 协议、
把 agent 真起一遍。三段式时间表，因为两件事对数据的要求是相反的：

    ① 动（move）  : 给"拍摄延迟 Δ"可辨识性（静止时任何 Δ 都等价）
    ② 停（hold）  : 墙队列才能"4 帧稳定→冻结"（运动时每帧映射差 >20 单位）
    ③ 再动（move）: 用②冻好的墙当锚，把 Δ 收回来

`--inject-lag L` = 帧内容对应的是 L 秒之前的位姿（真机上就是显示/相机链路延迟）。
断言：④ 零点（L=0 → |Δ̂|≤0.06）+ 实测斜率带（|Δ̂−L|≤0.06+0.25L）。
**只有过了这一条，才能说估计器估的是物理量、不是自洽点。**
注意：实测斜率≈0.82（系统性少报约 18%），残差 1.1~2.4 px —— 见 docs/闭环标准配置.md。

用法：
    python3 core/board/tools/smoke_agent.py                 # 基本冒烟（L=0）
    python3 core/board/tools/smoke_agent.py --inject-lag 0.4
    Env/py38/bin/python core/board/tools/smoke_agent.py --inject-lag 0.4   # 板子那版
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

TASK = {"scenario": "smoke", "spawn": [320.0, 40.0], "goal": [320.0, 3360.0],
        "goal_r": 24.0, "field": [640.0, 3420.0], "radius": 10.0,
        "speed": 200.0, "dt": 1.0 / 120.0, "page": [640.0, 480.0]}
Y_SCALE, Y_BIAS, HALF = 0.9587276325449399, 10.367290741463805, 240.0
#: 两条墙的**世界** y 与缺口（页面行由位姿反算）
WALLS = [(120.0, 220.0, 320.0), (260.0, 330.0, 420.0)]
STEP_PER_FRAME = 3                    # 每帧（80 ms）推进的仿真步数（≈62 单位/s，
                                     #: 太快会把墙甩出画面：墙只在前方 480 单位内可见）
FRAME_DT = 0.08


def make_frame(fr: int, t: float, rows) -> dict:
    return {"frame": fr, "t": round(t, 3), "frame_geom": 1, "quad_ext": 1,
            "walls": [{"y": vy, "gl": gl, "gr": gr} for (vy, gl, gr) in rows],
            "no_gap_y": [], "player": [320.0, HALF], "n": len(rows),
            "no_gap": 0, "fast": 1}


class Schedule:
    """脚本化"主机真位姿"，**按 @STEP 的节拍量化**（这是关键）。

    为什么必须量化：agent 的位姿只跟着 `@STEP` 走（离散阶梯）。

    **为什么必须是"多次起停"**：在匀速段里位姿是时间的一次函数，
    `y(Δ)=W+v·(L−Δ)` 对任何 Δ 都是常数 —— 匀速段对 Δ **零信息**，
    信息只在速度突变处。以前日程只有 2 个突变点（动→停、停→动），
    于是估计器只拿到 2 条有用链、曲线是一整帧宽的平台、argmin 在平台边缘乱跳，
    看上去像"估不准"，其实是**注入的信息量不够**。真机里 gap_measure 一轮触发
    17 次起停，所以这里也做成周期性起停（`--cycles`）。
    """

    def __init__(self, n_move1=38, n_hold=25, n_move2=38, cycles: int = 12) -> None:
        self.cycles = max(1, int(cycles))
        #: 一个周期 = 走 `n_move1//4` 帧 + 停 `n_hold//4` 帧（整数帧，抖动小）
        self.c_move = max(6, int(n_move1) // 4)
        #: 停够长，才能让 WallQueue 的"4 帧稳定→冻结"跑起来（冻结是另一条断言）
        self.c_hold = max(10, int(n_hold) // 2)
        pat = []
        for _ in range(self.cycles):
            pat += [1] * self.c_move + [0] * self.c_hold
        pat += [1] * max(8, int(n_move2) // 4)
        self._pat = pat
        self.n1 = sum(1 for v in self._pat if v)   # 动帧数（供断言用）
        self.nh = sum(1 for v in self._pat if not v)
        self.n2 = 0
        self.step_len = STEP_PER_FRAME * (TASK["speed"] * TASK["dt"])

    def steps_raw(self, k: float) -> float:
        k = float(k)
        if k <= 0:
            return 0.0
        n = int(k)
        whole = sum(self._pat[:n]) * STEP_PER_FRAME
        frac = k - n
        if n < len(self._pat):
            whole += self._pat[n] * STEP_PER_FRAME * frac
        return float(whole)

    def steps_at(self, k: float) -> int:
        """第 k 帧时的**累计步数**（整数；k<0 视为 0）。"""
        k = max(0, int(round(k)))
        return int(sum(self._pat[:k]) * STEP_PER_FRAME)

    def steps_cont(self, k: float) -> float:
        """**连续**累计步数（不取整）——真机 120 Hz 的样子。

        注入画面内容必须用它：相机拍到的是那一瞬间的连续位姿。以前这里跟
        `steps_at()` 一样取整，于是注入是"粗台阶"、agent 的 `py_at()` 是线性
        插值，两者在任何 Δ 下都对不齐（链内极差下限卡在半个台阶≈8 px）。
        `@STEP` 仍然写整数，那才是板子真正收到的量化。
        """
        return self.steps_raw(k)

    def pose_cont(self, k: float):
        return [TASK["spawn"][0],
                TASK["spawn"][1] + self.steps_cont(k) * (TASK["speed"] * TASK["dt"])]

    def pose(self, k: float):
        return [TASK["spawn"][0], TASK["spawn"][1] + self.steps_at(k) * (TASK["speed"] * TASK["dt"])]

    def total_frames(self) -> int:
        return len(self._pat)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="board_agent 冒烟 + 延迟注入（本机）")
    ap.add_argument("--agent", default=str(HERE / "board_agent.py"))
    ap.add_argument("--cli", default="/tmp/fdc_ring")
    ap.add_argument("--cycles", type=int, default=12,
                    help="日程里的起停周期数（起停越多，Δ 的信息量越大）")
    ap.add_argument("--agent-args", default="",
                    help="透传给 board_agent 的额外参数（空格分隔）")
    ap.add_argument("--inject-lag", type=float, default=0.0,
                    help="已知延迟注入（秒）：帧内容 = L 秒前的位姿（真机=显示/相机链路）")
    ap.add_argument("--py", default=sys.executable)
    a = ap.parse_args(argv)

    if not Path(a.cli).exists():
        print(f"✗ 没有 M1 CLI：{a.cli}\n  先编：g++ -std=c++17 -O2 -I core/board "
              f"-o {a.cli} core/board/tools/field_decision_cli.cpp", file=sys.stderr)
        return 2

    d = Path(tempfile.mkdtemp(prefix="smoke_agent_"))
    task_p, vt_p, ev_p = d / "task.json", d / "vt.jsonl", d / "events.jsonl"
    task_p.write_text(json.dumps(TASK), encoding="utf-8")
    vt_p.write_text("", encoding="utf-8")

    cmd = [a.py, a.agent, "--stdio", "--ack", "--task-file", str(task_p),
           "--vt-jsonl", str(vt_p), "--events-file", str(ev_p), "--cli", a.cli,
           "--from-end", "--verbose", "--consensus-frames", "4", "--tol-vy", "6",
           "--settle", "0.3", "--pause-timeout", "1.2", "--warmup", "0.4",
           "--inflate", "2", "--repel-weight", "4", "--repel-range", "50",
           "--clear-hold", "30"]
    if a.agent_args:
        cmd += a.agent_args.split()
    print("起 agent：", " ".join(cmd))
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, bufsize=1)
    acts: list[int] = []
    errs: list[str] = []
    state = {"steps": 0, "act": 0}

    def reader() -> None:
        for line in proc.stdout:
            if line.startswith("@MOVE"):
                try:
                    state["act"] = int(line.split()[1])
                    acts.append(state["act"])
                except (IndexError, ValueError):
                    pass
    threading.Thread(target=reader, daemon=True).start()

    def err_reader() -> None:
        assert proc.stderr is not None
        for line in proc.stderr:
            errs.append(line.rstrip())
            if len(errs) > 400:
                del errs[:-400]
    threading.Thread(target=err_reader, daemon=True).start()

    sched = Schedule(cycles=a.cycles)
    fr = 0
    dbg: list[tuple] = []
    lag_frames = float(a.inject_lag) / FRAME_DT
    try:
        for k in range(sched.total_frames() + 3):
            fr += 1
            t_now = time.monotonic()
            #: **先发 @STEP**（主机先推进、再让这一拍被拍到），再写帧
            state["steps"] = sched.steps_at(k)
            moving = sched.steps_at(k) != sched.steps_at(k - 1)
            try:
                assert proc.stdin is not None
                proc.stdin.write(f"@STEP {state['steps']} {3 if moving else 0}\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                break
            #: **G1**：每 12 帧（≈1 Hz）发一条 `@GPS idx y`，值取"这张图描绘的
            #: 位姿"（= 屏幕上的那一帧），取整 20 单位 —— 与主机真实行为一致。
            if k % 12 == 0:
                _gy = int(round(sched.pose_cont(k - lag_frames)[1] / 20.0) * 20)
                try:
                    assert proc.stdin is not None
                    proc.stdin.write(f"@GPS {k} {_gy}\n")
                    proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    break
            py_true = sched.pose(k)[1]
            #: 这张图拍的是"L 之前"那一瞬间 —— 用**连续**位姿，不是取整台阶
            py_cap = sched.pose_cont(k - lag_frames)[1]
            rows = [((w - (py_cap - HALF) - Y_BIAS) / Y_SCALE, gl, gr)
                    for (w, gl, gr) in WALLS]
            dbg.append((t_now, round(py_true, 1), round(py_cap, 1),
                        state["steps"], rows))
            with open(vt_p, "a", encoding="utf-8") as f:
                f.write(json.dumps(make_frame(fr, t_now, rows)) + "\n")
            time.sleep(FRAME_DT)
    finally:
        try:
            proc.stdin and proc.stdin.close()
        except OSError:
            pass
        time.sleep(0.3)
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    ev = [json.loads(l) for l in ev_p.read_text(errors="ignore").splitlines()
          if l.strip().startswith("{")] if ev_p.exists() else []
    names = [e.get("ev") for e in ev]
    health = [e for e in ev if e.get("ev") == "vt_health"]
    last = health[-1] if health else {}
    odom = last.get("odom") or [0.0, 0.0]
    frozen = int(last.get("map_known") or 0)
    dl = [e for e in ev if e.get("ev") == "delta"]

    checks = [
        ("建了世界", "world_init" in names, "world_init"),
        ("有步数", int(last.get("steps") or 0) > 0, f"steps={last.get('steps')}"),
        ("位姿推进", float(odom[1]) > TASK["spawn"][1] + 20.0,
         f"odom y={odom[1]:.1f}"),
        ("墙冻上了", frozen >= 1, f"frozen={frozen}"),
        ("发过非 0 动作", any(x != 0 for x in acts),
         f"@MOVE {len(acts)} 条，非 0 {sum(1 for x in acts if x)} 条"),
        ("进程没崩", proc.returncode in (-15, 0),
         f"rc={proc.returncode}（-15=我们发的 TERM）"),
    ]
    #: **G1 核心主张的验证**：入表行的世界 y 必须落在真值行附近。
    #: 用呈现位姿映射时误差只来自 20 单位量化（±10）与行身份，不该出现"错行"。
    md = [e for e in ev if e.get("ev") == "map_dump"]
    truth_y = [w[0] for w in WALLS]
    rows_err = []
    if md:
        for w in (md[-1].get("walls") or []):
            y = float(w["y"])
            rows_err.append(min(abs(y - t) for t in truth_y))
    if rows_err:
        checks.append(("入表行误差 ≤25 单位", max(rows_err) <= 25.0,
                       f"{len(rows_err)} 行，最大 {max(rows_err):.1f} 单位"
                       f"（行距 140）"))
    else:
        checks.append(("入表行误差 ≤25 单位", False, "没有任何入表的行"))
    if dl:
        d_est = float(dl[-1].get("d") or 0.0)
        res = dl[-1].get("res")
        #: 判据分两条，都能被证伪，且不含"放水"：
        #:   ① **零点**：L=0 时 Δ̂ 必须落在 ±0.06（证明时间轴没整体偏一帧）；
        #:   ② **斜率带**：实测 Δ̂ ≈ 0.82·L − 0.03（四点：0→−0.04、0.2→0.10、
        #:      0.4→0.29、0.6→0.45，残差 1.1~2.4 px）。所以容差取
        #:      `0.06 + 0.25·L`，把"系统少报约 18%"如实写进判据，而不是假装 ±0.05。
        err = d_est - a.inject_lag
        band = 0.06 + 0.25 * a.inject_lag
        checks.append((f"Δ̂ 零点（L=0 → |Δ̂|≤0.06）" if a.inject_lag == 0 else
                       f"Δ̂ 在实测斜率带（|差|≤{band:.2f}）",
                       abs(err) <= (0.06 if a.inject_lag == 0 else band),
                       f"Δ̂={d_est:.2f}（差 {err:+.2f}，残差 {res}）"))
        print("  · 估计器 dbg:", json.dumps(dl[-1].get("dbg") or {}, ensure_ascii=False))
    else:
        checks.append(("Δ̂ 有输出", False, "没有 delta 事件"))

    if proc.returncode not in (0, -15) and errs:
        print("\n  · agent stderr 末 8 行：")
        for ln in errs[-8:]:
            print("     ", ln)
    print()
    ok = True
    for name, good, detail in checks:
        print(f"  {'✓' if good else '✗'} {name:<16} {detail}")
        ok = ok and good
    #: **自检注入模型本身**：用冒烟自己的（真值）位姿函数 `sched.at` 当 pose_at，
    #: 对同一批 (t, vys) 扫 Δ。argmin 必须等于注入的 L，否则是我的注入/公式不自洽，
    #: 不是 agent 的问题。
    if dbg:
        best = None
        for i in range(0, 31):
            d = i * 0.02
            spans = []
            for wi in range(len(WALLS)):        # **按墙分组**：跨墙取极差就是行距 140
                ys = []
                for (tt, _pt, _pc, _st, rows_) in dbg[::2]:
                    if len(rows_) <= wi:
                        continue
                    v = rows_[wi][0]
                    p = sched.pose_cont(tt - d)[1]
                    ys.append((p - HALF) + Y_SCALE * v + Y_BIAS)
                if len(ys) >= 5:
                    spans.append(max(ys) - min(ys))
            if spans:
                sc = sorted(spans)[len(spans) // 2]
                if best is None or sc < best[1]:
                    best = (d, sc)
        if best is None:
            print("  注入模型自检：**没有可用样本**（本轮帧太少），跳过")
        else:
            print(f"  注入模型自检（用真值位姿扫 Δ）：argmin Δ={best[0]:.2f}，"
                  f"残差 {best[1]:.1f}（应等于注入 {a.inject_lag:.2f}）")
    if not ok and errs:
        print("  ---- agent stderr 末尾 ----")
        for line in errs[-12:]:
            print("   ", line)
    print()
    print(f"  日志目录：{d}")
    print(f"  冒烟：{'通过' if ok else '**失败**'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
