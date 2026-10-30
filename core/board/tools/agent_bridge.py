#!/usr/bin/env python3
"""agent_bridge.py —— **纯脚本层**桥：把现有 board_agent.py(--stdio) 接到小车上

⚠ 不修改任何原程序：
  * board_agent.py 一行不改，仍用它的 `--stdio`（stdin 收 @POSE、stdout 发 @MOVE）；
  * 本脚本只做三件事：喂 @POSE、读 @MOVE、把动作转成 @VEL 发给小车串口。

用法（板子上）：
  python3 agent_bridge.py --car-port /dev/ttyUSB0 --tag t1 \
      [--pose-static "320 40 0"] [--pose-file /tmp/pose.txt] \
      [--agent-args "--from-end --verbose --consensus-frames 8 --tol-vy 16 --settle 1.0 --pause-timeout 3.5"]

动作→速度映射（小车只要沿 x 前后，不需要转向）：
  act 0 → @VEL 0 0        （停）
  act 1 → @VEL -V 0       （-x / 后退）
  act 2 → @VEL +V 0       （+x / 前进）
  act 3/4 → @VEL 0 0      （原 y 轴，现实里交给"平台滚动"，小车不动）
  5..8（对角）→ 取其 x 分量
"""
from __future__ import annotations
import argparse, json, os, subprocess, sys, threading, time

V_DEFAULT = 120.0        # mm/s（对齐用的低速）


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--agent", default="/root/tools/board_agent.py")
    ap.add_argument("--agent-args", default=(
        "--from-end --verbose --consensus-frames 8 --tol-vy 16 "
        "--settle 1.0 --pause-timeout 3.5"))
    ap.add_argument("--car-port", default="")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pose-static", default="320 40 0")
    ap.add_argument("--pose-file", default="")
    ap.add_argument("--pose-hz", type=float, default=25.0)
    ap.add_argument("--v", type=float, default=V_DEFAULT)
    ap.add_argument("--tag", default="board")
    ap.add_argument("--log", default="")
    a = ap.parse_args()

    log_path = a.log or f"/tmp/agent_bridge_{a.tag}.jsonl"
    logf = open(log_path, "a", buffering=1)

    car = None
    if a.car_port:
        import serial
        car = serial.Serial(a.car_port, a.baud, timeout=0.05)
        logf.write(json.dumps({"ev": "car_open", "port": a.car_port, "baud": a.baud}) + "\n")

    cmd = [sys.executable, "-u", a.agent, "--stdio"] + a.agent_args.split()
    logf.write(json.dumps({"ev": "agent_start", "cmd": cmd}) + "\n")
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=open(f"/tmp/agent_{a.tag}.err", "ab"), text=True, bufsize=1)

    stop = threading.Event()

    # ---- 动作 → 小车 ----
    def act_to_vel(act: int) -> float:
        if act in (1, 3, 5, 8):
            if act == 1: return -a.v
            if act in (5, 8): return -a.v * 0.7
            return 0.0
        if act in (2, 4, 6, 7):
            if act == 2: return +a.v
            if act in (6, 7): return +a.v * 0.7
            return 0.0
        return 0.0

    def car_loop():
        """5 Hz 心跳：把当前目标速度持续发给小车（绕过它 1 s 看门狗）"""
        while not stop.is_set():
            v = cur_vel[0]
            if car is not None:
                try:
                    car.write((f"@VEL {int(v)} 0\n").encode()); car.flush()
                    r = car.read(512) or b""
                    if r:
                        for ln in r.split(b"\n"):
                            if ln:
                                logf.write(json.dumps({"ev": "car_rx",
                                                       "s": ln.decode("utf-8", "replace")}) + "\n")
                except Exception as e:      # 串口掉线：记录并停止，不崩
                    logf.write(json.dumps({"ev": "car_err", "e": str(e)[:80]}) + "\n")
                    time.sleep(0.5)
            time.sleep(0.2)

    cur_vel = [0.0]
    threading.Thread(target=car_loop, daemon=True).start()

    # ---- agent stdout → @MOVE ----
    def agent_rx():
        for line in p.stdout:
            line = line.strip()
            if not line.startswith("@MOVE"):
                continue
            try:
                act = int(line.split()[1])
            except Exception:
                continue
            cur_vel[0] = act_to_vel(act)
            logf.write(json.dumps({"ev": "move", "act": act, "v": cur_vel[0]}) + "\n")

    threading.Thread(target=agent_rx, daemon=True).start()

    # ---- 喂 @POSE ----
    seq = 0
    per = 1.0 / max(1.0, a.pose_hz)
    try:
        while p.poll() is None:
            seq += 1
            if a.pose_file and os.path.exists(a.pose_file):
                try:
                    x, y, r = (float(t) for t in open(a.pose_file).read().split()[:3])
                except Exception:
                    x, y, r = (float(t) for t in a.pose_static.split()[:3])
            else:
                x, y, r = (float(t) for t in a.pose_static.split()[:3])
            try:
                p.stdin.write(f"@POSE {seq} {x} {y} {r}\n"); p.stdin.flush()
            except Exception:
                break
            time.sleep(per)
    finally:
        stop.set()
        try:
            p.terminate()
        except Exception:
            pass
        try:
            cur_vel[0] = 0.0
            if car is not None:
                car.write(b"@VEL 0 0\n"); car.flush(); car.close()
        except Exception:
            pass
        logf.write(json.dumps({"ev": "end", "log": log_path}) + "\n")
        logf.close()
    print(log_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
