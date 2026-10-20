#!/usr/bin/env python3
"""真值在环（不接视觉）：仿真 `@STATE` → 板子 `field_decision_cli` → `@MOVE` → 仿真。

链路（没有任何像素）：

    本机 bullet_sim（无窗口，120 Hz）
        │  每 N 步一行 @STATE（同时写 --state-file，给录像工具）
        ▼
    板子 /root/tools/field_decision_cli --echo
        │  每个输入回一行 @ACK <seq> <act>；换向/心跳另打 @MOVE <act>
        ▼
    本机把最新的 @MOVE 作为下一步动作（保持到下一次更新）

和 `gt_loop.py` 的区别：那条是旧的 `--state-log` JSON + `gt_decision_cli`（gap_avoid）；
这条是 v2 平台的 `--state-pipe` 文本 + M1 代价场，字段和板子上的二进制完全一致。

用法（本机板子二进制先编一份，用于无板自测）：

    g++ -std=c++17 -O2 -o /tmp/field_decision_cli_host \\
        core/board/tools/field_decision_cli.cpp
    python3 core/board/tools/field_loop.py --board local \\
        --local-bin /tmp/field_decision_cli_host --max-seconds 30 --speed 4

真板子（默认）：

    python3 core/board/tools/field_loop.py --max-seconds 30 --state-hz 120 \
        --video-out /tmp/m1_board_field.mp4

产物：
    /tmp/m1_state.txt    @STATE 流（20 Hz，给录像；--record-hz 可调）
    /tmp/m1_actions.txt  @MOVE 流（同上）
    /tmp/m1_board.log    板子的 stdout+stderr（含两行自检）
    /tmp/m1_loop.json    汇总指标（到达/碰撞/间隙/切换/延迟分位）
    --video-out          跑完后用完整日志出势场+轨迹 mp4（不会和实时写抢读）
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from bullet_sim.cli import _state_pipe_line          # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402


def dist_rect(px: float, py: float, cx: float, cy: float, hw: float, hh: float) -> float:
    """点到轴对齐矩形的最短距离（内侧为 0）。"""
    return math.hypot(max(abs(px - cx) - hw, 0.0), max(abs(py - cy) - hh, 0.0))


def walls_of(env) -> list[tuple[float, float, float, float]]:
    """当前活着的矩形障碍（墙）：(x, y, half_w, half_h)。"""
    pool = env.world.state.bullets
    out: list[tuple[float, float, float, float]] = []
    for i in range(int(pool.count)):
        if not bool(pool.alive[i]):
            continue
        hw = float(pool.half_w[i]); hh = float(pool.half_h[i])
        if hw <= hh:                     # 圆不算墙
            continue
        out.append((float(pool.x[i]), float(pool.y[i]), hw, hh))
    return out


def pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))]


def board_command(args, goal, field_w: float, field_h: float) -> list[str]:
    """板子命令 + 视口/目标/场地参数（v2 的 viewport M1 支持这些开关）。"""
    tail = [
        "--echo",
        "--field-w", f"{field_w:g}",
        "--field-h", f"{field_h:g}",
        "--goal", f"{goal[0]:g}", f"{goal[1]:g}",
        "--view-h", f"{max(0.0, float(args.view_h)):g}",
    ]
    #: 板端可调项（A/B 用）：不给就沿用板子二进制的默认。
    for opt, val in (("--ema", getattr(args, "board_ema", None)),
                     ("--hyst", getattr(args, "board_hyst", None)),
                     ("--reflex", getattr(args, "board_reflex", None)),
                     ("--trace", getattr(args, "board_trace", None))):
        if val is not None:
            tail += [opt, f"{val:g}" if isinstance(val, float) else str(val)]
    if args.board == "local":
        return [args.local_bin, *tail]
    return [
        "ssh", "-i", args.ssh_key,
        "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null",
        "-o", "ConnectTimeout=8",
        "-T", args.board_host, args.board_binary, *tail,
    ]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="真值在环：仿真 ↔ 板子 M1 决策")
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--max-seconds", type=float, default=45.0, help="仿真时间上限（秒）")
    ap.add_argument("--state-hz", type=float, default=30.0, help="每板秒发多少条 @STATE")
    ap.add_argument("--record-hz", type=float, default=20.0,
                    help="写 --state-file 的频率（给 record_field_video.py 用；"
                         "默认 20，视频才不会变成慢动作/快放）。0 = 与 --state-hz 相同")
    ap.add_argument("--video-out", default=None,
                    help="可选：跑完用 record_field_video.py 出 mp4（需要 cv2）；"
                         "--view-h>0 时出跟随视口的视频")
    ap.add_argument("--view-h", type=float, default=0.0,
                    help="视角跟随高度（场地单位；0 = 全幅）。>0 时只把当前视口里的墙"
                         "发给板子，板子也只在视口内建势场")
    ap.add_argument("--field-h", type=float, default=None,
                    help="覆盖场景场高（默认取场景）")
    ap.add_argument("--field-w", type=float, default=None,
                    help="覆盖场景场宽（默认取场景）")
    ap.add_argument("--goal", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="覆盖目标点（默认取场景）")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="仿真倍率：1.0 = 与墙钟同速；0 = 不降速（尽量快）")
    ap.add_argument("--board", choices=("ssh", "local"), default="ssh")
    ap.add_argument("--local-bin", default="/tmp/field_decision_cli_host")
    ap.add_argument("--board-binary", default="/root/tools/field_decision_cli")
    ap.add_argument("--ssh-key", default="data/fpga/board_ssh/board_key")
    ap.add_argument("--board-host", default="root@192.168.137.242")
    ap.add_argument("--state-file", default="/tmp/m1_state.txt")
    ap.add_argument("--actions-file", default="/tmp/m1_actions.txt")
    ap.add_argument("--board-log", default="/tmp/m1_board.log")
    ap.add_argument("--summary", default="/tmp/m1_loop.json")
    ap.add_argument("--collision-log", default=None, help="可选：仿真逐帧碰撞/间距 JSONL")
    ap.add_argument("--stall-s", type=float, default=5.0,
                    help="多久收不到板子的 @ACK 就判失联（秒）")
    ap.add_argument("--no-stop-on-arrival", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    spec = ScenarioSpec.load(args.scenario)
    field_w = float(args.field_w if args.field_w is not None else spec.field_w)
    field_h = float(args.field_h if args.field_h is not None else spec.field_h)
    goal = (float(args.goal[0]) if args.goal is not None
            else float(spec.target_x if spec.target_x is not None else 320.0),
            float(args.goal[1]) if args.goal is not None
            else float(spec.target_y if spec.target_y is not None else 470.0))
    goal_r = float(spec.target_radius if spec.target_radius is not None else 24.0)
    dt = float(spec.dt)
    view_h = max(0.0, float(args.view_h))
    if view_h > 0.0:
        view_h = min(view_h, field_h)
    state_every = max(1, int(round(1.0 / (float(args.state_hz) * dt))))
    rec_hz = float(args.record_hz) if float(args.record_hz) > 0 else float(args.state_hz)
    record_every = max(1, int(round(1.0 / (rec_hz * dt))))

    # 板子现在通过 --field-w/--field-h/--goal/--view-h 接收这些；不再写死。
    if view_h > 0.0:
        print(f"  视口：640×{view_h:g}（跟随玩家；目标 {goal}，场 {field_w:g}×{field_h:g}）",
              file=sys.stderr)

    if args.board == "local" and not Path(args.local_bin).exists():
        print(f"本地板子程序不存在：{args.local_bin}\n"
              f"先编：g++ -std=c++17 -O2 -o {args.local_bin} "
              f"core/board/tools/field_decision_cli.cpp", file=sys.stderr)
        return 2

    env = BulletHellEnv(spec, seed=spec.seed, collision_log=args.collision_log)
    env.reset(seed=spec.seed)
    px, py = float(env.world.player[0]), float(env.world.player[1])
    n_walls0 = len(walls_of(env))

    state_f = open(args.state_file, "w", encoding="utf-8")
    actions_f = open(args.actions_file, "w", encoding="utf-8")
    log_f = open(args.board_log, "w", encoding="utf-8")

    cmd = board_command(args, goal, field_w, field_h)
    try:
        proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
    except Exception as exc:
        print(f"起不了板子进程：{exc!r}", file=sys.stderr)
        state_f.close(); actions_f.close(); log_f.close(); env.close()
        return 2

    lock = threading.Lock()
    shared = {
        "act": 0,
        "send_t": {},                 # seq -> monotonic 发送时刻
        "lat": [],                    # @ACK 的端到端延迟（秒）
        "acks": 0,
        "moves": 0,
        "last_ack": None,
        "eof": False,
        "selfcheck": [],
    }

    def reader() -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            now = time.monotonic()
            line = raw.rstrip("\n")
            with lock:
                log_f.write(line + "\n")
                log_f.flush()
                if line.startswith("@MOVE"):
                    shared["moves"] += 1
                    try:
                        act = int(line.split()[1])
                    except (IndexError, ValueError):
                        continue
                    shared["act"] = act
                    shared["last_ack"] = now
                    actions_f.write(line + "\n")
                    actions_f.flush()
                elif line.startswith("@ACK"):
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            seq = int(parts[1]); act = int(parts[2])
                        except ValueError:
                            continue
                        shared["acks"] += 1
                        shared["act"] = act
                        shared["last_ack"] = now
                        t0 = shared["send_t"].get(seq)
                        if t0 is not None:
                            shared["lat"].append(now - t0)
                elif line.startswith("[field]"):
                    shared["selfcheck"].append(line)
        with lock:
            shared["eof"] = True

    th = threading.Thread(target=reader, name="board-reader", daemon=True)
    th.start()

    switches = 0
    prev_act = 0
    collisions = 0
    min_clear: float | None = None
    min_clear_late: float | None = None
    reached = False
    t_sim = 0.0
    steps = 0
    seq = 0
    wall0 = time.monotonic()
    last_progress = 0.0
    error: str | None = None
    exit_code = 0

    try:
        while t_sim < args.max_seconds - 1e-12:
            if proc.poll() is not None and shared["acks"] == 0:
                error = f"板子进程提前退出（rc={proc.returncode}），见 {args.board_log}"
                exit_code = 2
                break
            if shared["eof"]:
                error = f"板子输出流已关闭（见 {args.board_log}）"
                exit_code = 2
                break
            if (seq > 0 and shared["last_ack"] is None
                    and time.monotonic() - wall0 > args.stall_s):
                error = f"板子 {args.stall_s:g} s 内没有任何 @ACK，判失联"
                exit_code = 3
                break
            if (shared["last_ack"] is not None
                    and time.monotonic() - shared["last_ack"] > args.stall_s):
                error = f"板子 {args.stall_s:g} s 没有回 @ACK，判失联"
                exit_code = 3
                break

            # 视角跟随：只把当前视口里的墙发出去（板子看不到视野外的东西）。
            y_min = y_max = None
            if view_h > 0.0:
                py_now = float(env.world.player[1])
                vy0 = min(max(py_now - view_h * 0.5, 0.0), max(0.0, field_h - view_h))
                y_min = vy0 - 20.0
                y_max = vy0 + view_h + 20.0
            if steps % record_every == 0:
                # 录像用：比决策采样慢（默认 20 Hz），否则 mp4 会变成慢动作。
                state_f.write(_state_pipe_line(env, seq=seq, y_min=y_min, y_max=y_max))
                state_f.flush()
            if steps % state_every == 0:
                line = _state_pipe_line(env, seq=seq, y_min=y_min, y_max=y_max)
                with lock:
                    shared["send_t"][seq] = time.monotonic()
                try:
                    assert proc.stdin is not None
                    proc.stdin.write(line)
                    proc.stdin.flush()
                except (BrokenPipeError, OSError) as exc:
                    error = f"写板子 stdin 失败：{exc!r}"
                    exit_code = 2
                    break
                seq += 1

            with lock:
                act = int(shared["act"])
            if act != prev_act:
                switches += 1
                prev_act = act

            _obs, _rew, _term, _trunc, info = env.step(act)
            collisions += int(info.get("collision_events", 0) or 0)
            px, py = float(env.world.player[0]), float(env.world.player[1])

            for (cx, cy, hw, hh) in walls_of(env):
                d = dist_rect(px, py, cx, cy, hw, hh) - float(spec.player_radius)
                min_clear = d if min_clear is None else min(min_clear, d)
                if t_sim >= 1.0:
                    min_clear_late = d if min_clear_late is None else min(min_clear_late, d)

            t_sim += dt
            steps += 1

            if args.speed > 0:
                due = wall0 + t_sim / float(args.speed)
                now = time.monotonic()
                if due > now:
                    time.sleep(due - now)

            if math.hypot(px - goal[0], py - goal[1]) <= goal_r:
                reached = True
                # 给录像补最后一帧（否则 20 Hz 采样可能停在到达前 50 ms，
                # 录像进程看不到"到达"会一直等到 --seconds）。
                state_f.write(_state_pipe_line(env, seq=seq, y_min=y_min, y_max=y_max))
                state_f.flush()
                if not args.no_stop_on_arrival:
                    break

            if t_sim - last_progress >= 5.0 and args.verbose:
                last_progress = t_sim
                with lock:
                    acks = shared["acks"]
                    lat = list(shared["lat"])
                p50 = pct(lat, 0.50)
                print(f"  t={t_sim:5.1f}s pos=({px:.0f},{py:.0f}) act={act} "
                      f"acks={acks} lat_p50={p50 * 1e3 if p50 else float('nan'):.1f}ms",
                      file=sys.stderr)
    except KeyboardInterrupt:
        error = "Ctrl-C"
    finally:
        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()
        th.join(timeout=2.0)
        state_f.close()
        actions_f.close()
        log_f.close()

    wall_s = time.monotonic() - wall0
    with lock:
        lat = list(shared["lat"])
        acks = shared["acks"]
        moves = shared["moves"]
        selfcheck = list(shared["selfcheck"])
    clearance = None if min_clear is None else round(min_clear, 2)
    clearance_late = None if min_clear_late is None else round(min_clear_late, 2)
    summary = {
        "scenario": spec.name,
        "goal": goal,
        "goal_radius": goal_r,
        "reached": bool(reached),
        "t_sim_s": round(t_sim, 3),
        "wall_s": round(wall_s, 3),
        "rtf": round(t_sim / wall_s, 3) if wall_s > 0 else None,
        "final_pos": [round(px, 1), round(py, 1)],
        "collisions": int(collisions),
        "min_clearance": clearance,
        "min_clearance_after_1s": clearance_late,
        "clearance_ok": clearance is not None and clearance >= 2.0,
        "geometric_collision": clearance is not None and clearance < 0.0,
        "action_switches": int(switches),
        "states_sent": int(seq),
        "acks": int(acks),
        "moves": int(moves),
        "latency_ms": {
            "p50": None if not lat else round(pct(lat, 0.50) * 1e3, 1),
            "p95": None if not lat else round(pct(lat, 0.95) * 1e3, 1),
            "max": None if not lat else round(max(lat) * 1e3, 1),
        },
        "board_selfcheck": selfcheck,
        "state_every_steps": state_every,
        "state_hz": args.state_hz,
        "speed": args.speed,
        "error": error,
        "files": {
            "state": args.state_file,
            "actions": args.actions_file,
            "board_log": args.board_log,
            "video": args.video_out,
        },
    }
    Path(args.summary).write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                                  encoding="utf-8")
    env.close()

    # 录像放在**跑完之后**用完整日志出片：不会和实时写入抢读，切换数也不会被
    # 半行/重读污染（之前边跑边录会把 18 次切换记成 35 次）。
    if args.video_out:
        rec_script = Path(__file__).with_name("record_field_video.py")
        rec_cmd = [
            sys.executable, str(rec_script),
            "--state", args.state_file, "--actions", args.actions_file,
            "--out", args.video_out, "--seconds", "20",
            "--fps", f"{rec_hz:g}",
            "--goal", f"{goal[0]:g},{goal[1]:g}",
            "--field-h", f"{field_h:g}",
            "--view-h", f"{view_h:g}",
        ]
        print(f"  录像：{args.video_out}（{rec_hz:g} fps，读完整日志）", file=sys.stderr)
        try:
            subprocess.run(rec_cmd, check=False)
        except Exception as exc:                 # pragma: no cover - 环境缺 cv2 等
            print(f"⚠ 录像失败：{exc!r}", file=sys.stderr)

    print(f"场景 {spec.name}  目标 {goal} 半径 {goal_r:g}  墙 {n_walls0} 段")
    print(f"  到达 {'是' if reached else '否'}   仿真 {t_sim:5.2f} s   墙钟 {wall_s:5.2f} s "
          f"({summary['rtf']}×)   终点 ({px:.0f},{py:.0f})")
    print(f"  仿真碰撞事件 {collisions} 次   最小间隙 {clearance}（1 s 后 {clearance_late}）"
          f"   动作切换 {switches}")
    ok_reach = "是" if reached else "否"
    ok_gap = "是" if (clearance is not None and clearance >= 2.0) else "否"
    ok_sw = "是" if switches < 30 else "否"
    ok_col = "是" if collisions == 0 and not summary["geometric_collision"] else "否"
    print(f"  判据：到达={ok_reach}  间隙≥+2={ok_gap}  切换<30={ok_sw}  0 碰撞={ok_col}")
    print(f"  状态 {seq} 条   @MOVE {moves} 条   @ACK {acks} 条")
    if lat:
        print(f"  端到端延迟 p50 {pct(lat, 0.5) * 1e3:.1f} ms   p95 {pct(lat, 0.95) * 1e3:.1f} ms"
              f"   max {max(lat) * 1e3:.1f} ms   （n={len(lat)}）")
    for line in selfcheck:
        print(f"  板子自检 {line}")
    if error:
        print(f"  ⚠ {error}")
    print(f"  汇总 {args.summary}")
    criteria_ok = (reached and summary["clearance_ok"] and switches < 30
                   and collisions == 0 and not summary["geometric_collision"])
    if not criteria_ok and exit_code == 0:
        exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
