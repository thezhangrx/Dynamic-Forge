#!/usr/bin/env python3
"""主机侧：**只渲染虚拟平台 + 回放板子的运动信号**。

与 `full_loop.py` 的区别：
  * 不运行 VT、不建 @STATE、不做门控/暂停——这些全部在板子上（`board_agent.py`）；
  * 只做两件事：
      1) 渲染 640×480 跟随视口页面（相机拍它），有势场版给录像；
      2) 通过 TCP 把角色位姿 `@POSE x y r` 发给板子，接收 `@MOVE <act>` 并推进仿真。

协议（一行一条，UTF-8）：
    主机 → 板子： @POSE <x> <y> <r>
    板子 → 主机： @MOVE <act>
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "core"))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import cv2
import numpy as np

sys.path.insert(0, str(ROOT / "core/board/tools"))
from bullet_sim.render.pygame_view import PygameRenderer
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.env import BulletHellEnv
from full_loop import FrameServer, PagePublisher


class Link:
    """TCP 服务端：等板子连上来，收 @MOVE，发 @POSE。"""

    def __init__(self, port: int) -> None:
        self.port = int(port)
        self.conn: socket.socket | None = None
        self.peer = ""
        self._lock = threading.Lock()
        self.act = 0
        self.moves = 0
        self.poses = 0
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", self.port))
        self._srv.listen(1)
        self._stop = False
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        while not self._stop:
            try:
                conn, addr = self._srv.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self.conn = conn
                self.peer = f"{addr[0]}:{addr[1]}"
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn: socket.socket) -> None:
        buf = b""
        while not self._stop:
            try:
                chunk = conn.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("utf-8", "ignore").strip()
                if txt.startswith("@MOVE"):
                    try:
                        a = int(txt.split()[1])
                    except (IndexError, ValueError):
                        continue
                    with self._lock:
                        self.act = a
                        self.moves += 1

    def send_pose(self, x: float, y: float, r: float) -> bool:
        with self._lock:
            conn = self.conn
        if conn is None:
            return False
        try:
            conn.sendall(f"@POSE {x:.2f} {y:.2f} {r:.2f}\n".encode())
            self.poses += 1
            return True
        except OSError:
            with self._lock:
                self.conn = None
            return False

    def stop(self) -> None:
        self._stop = True
        try:
            self._srv.close()
        except OSError:
            pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="主机：只渲染平台 + 回放 @MOVE")
    ap.add_argument("--scenario", default=str(ROOT / "configs/20_walls_alt_24_dy140.json"))
    ap.add_argument("--port", type=int, default=8100, help="页面端口")
    ap.add_argument("--tcp", type=int, default=9000, help="等板子连的 TCP 端口")
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--max-seconds", type=float, default=60.0)
    ap.add_argument("--render-hz", type=float, default=30.0)
    ap.add_argument("--field-hz", type=float, default=5.0)
    ap.add_argument("--follow-scale", type=float, default=1.0)
    ap.add_argument("--out-dir", default=str(ROOT / "data/fpga/loop_logs"))
    ap.add_argument("--tag", default="board_vt")
    ap.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--viewer-pos", default="320,60")
    ap.add_argument("--viewer-size", default="1280x960")
    ap.add_argument("--hold", type=float, default=2.0)
    a = ap.parse_args(argv)

    spec = ScenarioSpec.load(a.scenario)
    env = BulletHellEnv(spec, seed=spec.seed)
    env.reset(seed=spec.seed)
    goal = (float(spec.target_x or 320.0), float(spec.target_y or 470.0))
    goal_r = float(spec.target_radius or 24.0)
    dt = float(spec.dt)
    field_w = float(spec.field_w)
    field_h = float(spec.field_h)

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f"platform_{ts}_{a.tag}"
    follow_mp4 = Path(str(stem) + "_follow.mp4")
    field_mp4 = Path(str(stem) + "_field.mp4")

    def writer(path: Path, fps: float):
        return cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                               float(fps), (640, 480))

    common = dict(mode="rgb_array", window_size=(640, 480), show_hud=False,
                  show_text=False, show_fiducials=False, show_world_axes=False,
                  show_field_border=True, show_hitboxes=False,
                  follow_player=True, follow_h=240.0,
                  follow_scale=float(a.follow_scale),
                  wall_color=(200, 60, 0), background=(8, 10, 14),
                  field_bg=(34, 40, 55), resizable=False)
    page = PygameRenderer(show_potential=False, **common)
    field_view = PygameRenderer(show_potential=True, potential_every=1, **common)

    server = FrameServer(a.port)
    base = page.render(env.world)
    server.publish(base)
    publisher = PagePublisher(server, fps=max(10.0, a.render_hz)).start()
    print(f"  page http://127.0.0.1:{a.port}/   TCP @MOVE 端口 {a.tcp}", file=sys.stderr)

    viewer = None
    if a.viewer:
        import subprocess
        venv = dict(os.environ)
        venv["DISPLAY"] = ":0"
        venv["SDL_VIDEODRIVER"] = "x11"
        vx, vy = a.viewer_pos.split(",")
        vw, vh = a.viewer_size.lower().split("x")
        venv["SDL_VIDEO_WINDOW_POS"] = f"{vx},{vy}"
        viewer = subprocess.Popen(
            [sys.executable, str(ROOT / "core/board/tools/page_view.py"),
             "--url", f"http://127.0.0.1:{a.port}/stream.mjpg",
             "--x", vx, "--y", vy, "--w", vw, "--h", vh,
             "--fps", f"{max(20.0, a.render_hz):g}"],
            env=venv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    fw = writer(follow_mp4, a.render_hz)
    vw_ = writer(field_mp4, a.field_hz)

    link = Link(a.tcp)
    t0 = time.monotonic()
    t_sim = 0.0
    last_page = 0.0
    last_field = 0.0
    reached = False
    collisions = 0
    print("  等板子连 TCP ...", file=sys.stderr)
    while t_sim < a.max_seconds and not reached:
        now = time.monotonic()
        with link._lock:
            act = link.act
        _o, _r, _t, _tr, info = env.step(int(act))
        collisions += int(info.get("collision_events", 0) or 0)
        px, py = float(env.world.player[0]), float(env.world.player[1])
        r = float(env.world.player[4])
        t_sim += dt
        link.send_pose(px, py, r)
        if now - last_page >= 1.0 / max(1.0, a.render_hz):
            last_page = now
            fr = page.render(env.world)
            if fr is not None:
                publisher.submit(fr)
                fw.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
        if now - last_field >= 1.0 / max(0.5, a.field_hz):
            last_field = now
            ff = field_view.render(env.world)
            if ff is not None:
                vw_.write(cv2.cvtColor(ff, cv2.COLOR_RGB2BGR))
        if (px - goal[0]) ** 2 + (py - goal[1]) ** 2 <= goal_r ** 2:
            reached = True
        # 按 speed 节流
        due = t0 + t_sim / max(1e-6, a.speed)
        lag = due - time.monotonic()
        if lag > 0:
            time.sleep(min(lag, 0.01))

    summary = {
        "scenario": spec.name, "reached": reached, "t_sim_s": round(t_sim, 3),
        "wall_s": round(time.monotonic() - t0, 2),
        "final_pos": [round(px, 1), round(py, 1)],
        "collisions": collisions, "moves": link.moves, "poses": link.poses,
        "tcp_peer": link.peer,
        "videos": {"follow": str(follow_mp4), "field": str(field_mp4)},
    }
    (Path(str(stem) + ".json")).write_text(json.dumps(summary, ensure_ascii=False,
                                                      indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2), file=sys.stderr)

    time.sleep(a.hold)
    publisher.stop()
    fw.release()
    vw_.release()
    link.stop()
    if viewer is not None:
        try:
            viewer.send_signal(2)
            viewer.wait(timeout=3.0)
        except Exception:                         # noqa: BLE001
            pass
    server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
