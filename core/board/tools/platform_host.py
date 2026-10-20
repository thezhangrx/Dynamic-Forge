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
import queue
import socket
import subprocess
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
from field_loop import dist_rect, walls_of
from run_log import RunLog, local_truth_rows, virtual_truth
from full_loop import FrameServer, PagePublisher


class VideoWriterThread:
    """录像写盘/编码挪到独立线程：主循环只做一次 BGR 转换 + 入队。

    队列满就丢帧（录像允许掉帧，页面和姿态不能等磁盘）。实测 640×480
    `VideoWriter.write` 在主机上 5~10 ms/帧，放在 30 fps 主循环里直接把它
    压到 ~18 fps；拆出去后主循环回到 28~30 fps。
    """

    def __init__(self, path: Path, fps: float, size: tuple[int, int] = (640, 480),
                 maxlen: int = 8) -> None:
        self.vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                                  float(fps), size)
        self.q: queue.Queue = queue.Queue(maxlen)
        self.written = 0
        self.dropped = 0
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def write(self, rgb) -> None:
        try:
            self.q.put_nowait(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                return
            try:
                self.vw.write(item)
                self.written += 1
            except (cv2.error, OSError):
                pass

    def close(self) -> None:
        try:
            self.q.put_nowait(None)
        except queue.Full:
            try:
                self.q.put(None, timeout=2.0)
            except queue.Full:
                pass
        self._t.join(timeout=10.0)
        self.vw.release()


class FieldRecorder:
    """热力图录像：主循环只提交世界快照（clone 0.07 ms），渲染+写盘在独立线程。

    实测 `show_potential` 每帧要重算势场，一帧 ~80 ms；放在 30 fps 主循环里
    每 200 ms 就吃掉 80 ms，页面 fps 从 30 掉到 18。拆出去后主循环只剩提交。
    """

    def __init__(self, renderer, writer: "VideoWriterThread", hz: float) -> None:
        self.r = renderer
        self.w = writer
        self.hz = max(0.1, float(hz))
        self.q: queue.Queue = queue.Queue(2)
        self.rendered = 0
        self.dropped = 0
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def submit(self, world) -> None:
        try:
            self.q.put_nowait(world.clone())
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while True:
            w = self.q.get()
            if w is None:
                return
            try:
                fr = self.r.render(w)
            except (RuntimeError, cv2.error):
                continue
            if fr is not None:
                self.w.write(fr)
                self.rendered += 1

    def close(self) -> None:
        try:
            self.q.put_nowait(None)
        except queue.Full:
            pass
        self._t.join(timeout=10.0)


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
        self.pose_seq = 0
        self.move_log: list[tuple[float, int]] = []
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
                        self.move_log.append((time.monotonic(), a))

    def drain_moves(self) -> list[tuple[float, int]]:
        with self._lock:
            out = self.move_log
            self.move_log = []
        return out

    def send_pose(self, x: float, y: float, r: float) -> bool:
        with self._lock:
            conn = self.conn
        if conn is None:
            return False
        try:
            with self._lock:
                self.pose_seq += 1
                seq = self.pose_seq
            conn.sendall(f"@POSE {seq} {x:.2f} {y:.2f} {r:.2f}\n".encode())
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


class SshLink:
    """反向通道：主机 ssh 起 board_agent --stdio，stdin 发 @POSE，stdout 收 @MOVE。"""

    def __init__(self, target: str, key: str, agent: str,
                 python: str = "python3", extra: str = "") -> None:
        self.peer = f"ssh:{target}"
        self._lock = threading.Lock()
        self.act = 0
        self.moves = 0
        self.poses = 0
        self.pose_seq = 0
        self.move_log: list[tuple[float, int]] = []
        self.geom_bad = False
        self.geom_bad_msg = ""
        self._stop = False
        cmd = ["ssh", "-i", key, "-o", "StrictHostKeyChecking=no",
               "-o", "ServerAliveInterval=5", target,
               f"{python} {agent} --stdio {extra}".strip()]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._drain_err, daemon=True).start()

    def _drain_err(self) -> None:
        for line in self.proc.stderr:
            txt = line.decode("utf-8", "ignore").rstrip()
            if not txt:
                continue
            if "GEOM_BAD" in txt:
                #: 板端明确说"相机里没有页面"：记下来，主机 8 s 后提前收工，
                #: 别白跑 300 s 再发现一条墙都没学到。
                self.geom_bad = True
                self.geom_bad_msg = txt
            print(f"  [board] {txt}", file=sys.stderr)

    def _read(self) -> None:
        for line in self.proc.stdout:
            txt = line.decode("utf-8", "ignore").strip()
            if txt.startswith("@MOVE"):
                try:
                    a = int(txt.split()[1])
                except (IndexError, ValueError):
                    continue
                with self._lock:
                    self.act = a
                    self.moves += 1
                    self.move_log.append((time.monotonic(), a))

    def ready(self) -> bool:
        return self.proc.poll() is None

    def drain_moves(self) -> list[tuple[float, int]]:
        with self._lock:
            out = self.move_log
            self.move_log = []
        return out

    def send_pose(self, x: float, y: float, r: float) -> bool:
        if self.proc.poll() is not None:
            return False
        try:
            with self._lock:
                self.pose_seq += 1
                seq = self.pose_seq
            self.proc.stdin.write(f"@POSE {seq} {x:.2f} {y:.2f} {r:.2f}\n".encode())
            self.proc.stdin.flush()
            self.poses += 1
            return True
        except (OSError, ValueError):
            return False

    def stop(self) -> None:
        self._stop = True
        try:
            self.proc.stdin.close()
        except (OSError, ValueError):
            pass


    def start_camera_pipe(target: str, key: str, extra: str = "") -> dict:
        """在板子上拉起相机管线：ffmpeg 抓 /dev/video0 → vt_board（VT 在板端）。

        **板端决策的输入只能是这条链路**（用户硬规则）。同时写 /tmp/vt_source=camera，
        板端 agent 会校验这个标记，synthetic 输入会被直接拒绝。
        """
        import subprocess
        cmd = (
            'pkill -f "[v]t_board"; pkill -f "[f]fmpeg"; sleep 0.3; '
            'rm -f /tmp/vt.jsonl /tmp/cam.jpg; '
            'nohup bash -c \'ffmpeg -hide_banner -loglevel error -f v4l2 '
            '-input_format mjpeg -video_size 640x480 -i /dev/video0 '
            '-filter_complex "fps=2,scale=320:240,split=2[a][b]" '
            '-map "[a]" -f rawvideo -pix_fmt rgb24 - '
            '-map "[b]" -update 1 -q:v 5 -y /tmp/cam.jpg '
            '| /root/tools/vt_board --w 320 --h 240 --frame-roi 0 0 320 240 '
            '--quad-state /tmp/qb.txt --jsonl /tmp/vt.jsonl %s\' '
            '>/tmp/campipe.log 2>&1 & '
            'sleep 4; pidof vt_board; wc -l < /tmp/vt.jsonl'
        ).replace("%s", extra)
        try:
            res = subprocess.run(["ssh", "-i", key, "-o", "StrictHostKeyChecking=no",
                                  "-o", "ConnectTimeout=8", target, cmd],
                                 capture_output=True, timeout=60)
            out = (res.stdout or b"").decode("utf-8", "ignore").strip().splitlines()
            return {"ok": res.returncode == 0,
                    "pid": out[0] if out else "",
                    "vt_lines": out[-1] if out else ""}
        except (OSError, subprocess.SubprocessError) as exc:
            return {"ok": False, "error": repr(exc)}


def quad_fingerprint(target: str, key: str) -> str:
    """抓板端白框四角状态当"相机/窗口是否动过"的指纹。"""
    import subprocess
    try:
        res = subprocess.run(["ssh", "-i", key, "-o", "StrictHostKeyChecking=no",
                              "-o", "ConnectTimeout=8", target,
                              "cat /tmp/qb.txt 2>/dev/null"],
                             capture_output=True, timeout=15)
        txt = (res.stdout or b"").decode("utf-8", "ignore")
    except (OSError, subprocess.SubprocessError):
        return ""
    nums = []
    for tok in txt.replace(",", " ").split():
        try:
            nums.append(round(float(tok), 1))
        except ValueError:
            continue
    return ",".join(f"{v:.1f}" for v in nums)


def quad_moved(old: str, new: str, tol: float = 6.0) -> bool:
    """四角平均位移超过 tol 像素 → 认为相机/窗口动过。"""
    if not old or not new:
        return True
    try:
        a = [float(v) for v in old.split(",")]
        b = [float(v) for v in new.split(",")]
    except ValueError:
        return True
    if len(a) != len(b) or not a:
        return True
    return sum(abs(x - y) for x, y in zip(a, b)) / len(a) > tol


def fetch_board_files(target: str, key: str, run_dir: Path,
                      python: str = "python3") -> dict[str, Any]:
    """把板端证据拷进 run 目录：@STATE 流 / 板端事件 / VT 原始帧。"""
    import subprocess
    out: dict[str, Any] = {}
    ssh = ["ssh", "-i", key, "-o", "StrictHostKeyChecking=no",
           "-o", "ConnectTimeout=8", target]
    jobs = {
        "board_state.txt": "cat /tmp/board_state.txt",
        "board_events.jsonl": "cat /tmp/board_events.jsonl",
        "board_vt.jsonl": "tail -n 6000 /tmp/vt.jsonl",
        #: 相机快照必须连带时间戳一起取：/tmp/cam.jpg 只在管线带 jpeg 输出时才刷，
        #: 之前吃过亏 —— 读到的是几分钟前的旧文件，误判成"窗口被盖住"。
        "camera_last.jpg": "cat /tmp/cam.jpg 2>/dev/null; echo; "
                           "stat -c %Y /tmp/cam.jpg 2>/dev/null",
    }
    for name, cmd in jobs.items():
        dest = run_dir / name
        try:
            res = subprocess.run(ssh + [cmd], capture_output=True, timeout=60)
            if res.returncode == 0 and res.stdout:
                body = res.stdout
                if name == "camera_last.jpg":
                    #: 末行是 mtime（秒）；据此算出快照年龄，过期的直接标记
                    parts = body.rsplit(b"\n", 2)
                    mtxt = parts[-1].strip() if len(parts) > 1 else b""
                    body = parts[0] + (b"\n" if len(parts) > 1 else b"")
                    try:
                        age = time.time() - float(mtxt or 0)
                    except ValueError:
                        age = -1.0
                    dest.write_bytes(body)
                    out[name] = {"bytes": len(body), "age_s": round(age, 1),
                                 "stale": age < 0 or age > 5.0}
                else:
                    dest.write_bytes(body)
                    out[name] = {"bytes": len(body),
                                 "lines": body.count(b"\n")}
            else:
                out[name] = {"error": res.returncode}
        except (OSError, subprocess.SubprocessError) as exc:
            out[name] = {"error": repr(exc)}
    return out


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
    ap.add_argument("--wall-rgb", default="255,240,200",
                    help="墙颜色。**必须够亮**：VT 的墙检测是亮度门，实测 200,60,0 "
                         "一堵墙都测不到，换成 255,240,200 后缺口边界误差 ≤1 px")
    ap.add_argument("--field-rgb", default="12,14,20", help="场地底色（要够暗）")
    ap.add_argument("--bg-rgb", default="0,0,0", help="场地外底色")
    ap.add_argument("--viewer-margin", type=int, default=0,
                    help="页面窗口四周留黑边（实测 24 会把黑边算进四边形，默认 0）")
    ap.add_argument("--fiducials", action=argparse.BooleanOptionalAction,
                    default=False,
                    help="画四角 ArUco 标定标记（现有 standard.fiducial 那套）")
    ap.add_argument("--fiducial-side", type=float, default=28.0,
                    help="标记边长（场地单位）；相机离得远/画面模糊时可以加大")
    ap.add_argument("--fiducial-anchor", default="band", choices=("field", "band"),
                    help="标记锚点：band=当前视口四角（跟随时永远可见）")
    ap.add_argument("--calib-markers", action=argparse.BooleanOptionalAction,
                    default=False,
                    help="在视口四角画 4 个纯色标定块（红绿蓝黄），供 OpenCV 单应标定")
    ap.add_argument("--frame-px", type=int, default=2,
                    help="页面白框粗细（相机 VT 靠它定四边形，太细会锁错框）")
    ap.add_argument("--field-every", type=int, default=2,
                    help="热力图抽样：每 N 个格子画一次（1=最细但很贵）")
    ap.add_argument("--profile", action="store_true", help="每 2 s 打一次循环耗时")
    ap.add_argument("--board-ssh", default="",
                    help="板子 ssh 目标，如 root@192.168.137.242（板子连不到主机时用）")
    ap.add_argument("--ssh-key", default=str(ROOT / "data/fpga/board_ssh/board_key"))
    ap.add_argument("--board-agent", default="/root/tools/board_agent.py")
    ap.add_argument("--board-python", default="python3")
    #: 硬规则（用户要求）：板端决策的输入只能是真实相机。
    #: 曾经存在过"PC 渲染帧 → 板端 VT"的通道（--vt-feed），已彻底删除。
    ap.add_argument("--vt-board-extra", default="",
                    help="附加给板端 vt_board 的参数，如 '--no-gap-recompute'")
    ap.add_argument("--board-extra", default="")
    ap.add_argument("--connect-timeout", type=float, default=90.0)
    ap.add_argument("--calib", default=str(ROOT / "data/fpga/calib_y.json"),
                    help="y 换算系数标定文件（自动生成，不手调）")
    ap.add_argument("--calib-board", default="/tmp/calib_y.json",
                    help="板端读取的标定文件路径")
    ap.add_argument("--quad-tol", type=float, default=6.0,
                    help="白框四角平均位移超过这个像素就认为相机/窗口动过，需要重标")
    ap.add_argument("--start-camera-pipe", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="开跑前在板子上拉起相机管线（唯一允许的决策输入来源）")
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

    run_dir = out / f"run_{ts}_{a.tag}"
    vtruth = virtual_truth(spec)
    log = RunLog(run_dir, {
        "tag": a.tag,
        "scenario": a.scenario,
        "scenario_name": spec.name,
        "field_w": field_w, "field_h": field_h, "dt": dt,
        "goal": [goal[0], goal[1]], "goal_r": goal_r,
        "spawn": [float(spec.player_x or 0.0), float(spec.player_y or 0.0)],
        "player_radius": float(spec.player_radius),
        "args": {k: v for k, v in vars(a).items()},
        "videos": {"follow": str(follow_mp4), "field": str(field_mp4)},
        "summary_path": str(stem) + ".json",
        "virtual_truth": vtruth["rows"],
    })

    def writer(path: Path, fps: float):
        return cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                               float(fps), (640, 480))

    def _rgb(text: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
        try:
            parts = [int(v) for v in str(text).replace(" ", "").split(",")]
            if len(parts) == 3:
                return (max(0, min(255, parts[0])), max(0, min(255, parts[1])),
                        max(0, min(255, parts[2])))
        except ValueError:
            pass
        return default

    wall_rgb = _rgb(a.wall_rgb, (255, 240, 200))
    field_rgb = _rgb(a.field_rgb, (12, 14, 20))
    bg_rgb = _rgb(a.bg_rgb, (0, 0, 0))
    common = dict(mode="rgb_array", window_size=(640, 480), show_hud=False,
                  show_text=False, show_fiducials=bool(a.fiducials),
                  fiducial_anchor=str(a.fiducial_anchor),
                  fiducial_side=float(a.fiducial_side),
                  show_world_axes=False,
                  show_field_border=True, frame_px=int(a.frame_px),
                  show_viewport_markers=bool(a.calib_markers),
                  show_hitboxes=False,
                  follow_player=True, follow_h=240.0,
                  follow_scale=float(a.follow_scale),
                  wall_color=wall_rgb, background=bg_rgb,
                  field_bg=field_rgb, resizable=False)
    page = PygameRenderer(show_potential=False, **common)
    field_view = PygameRenderer(show_potential=True,
                                potential_every=max(1, int(a.field_every)),
                                **common)

    server = FrameServer(a.port)
    base = page.render(env.world)
    server.publish(base)
    publisher = PagePublisher(server, fps=max(10.0, a.render_hz)).start()
    if a.board_ssh:
        print(f"  page http://127.0.0.1:{a.port}/   ssh 反向通道 {a.board_ssh}", file=sys.stderr)
    else:
        print(f"  page http://127.0.0.1:{a.port}/   TCP @MOVE 端口 {a.tcp}", file=sys.stderr)

    viewer = None
    if a.viewer:
        #: 先清掉上一轮遗留的僵窗：它们会一直盖在新页面上，VT 读到的是旧画面
        #: （而且新加的"每秒置顶"会让僵窗和新窗互相抢置顶）。
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmd = f.read().replace(b"\0", b" ").decode("utf-8", "ignore")
            except OSError:
                continue
            if "page_view.py" in cmd:
                try:
                    os.kill(int(pid), 15)
                except OSError:
                    pass
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
             "--fps", f"{max(20.0, a.render_hz):g}",
             "--margin", str(int(a.viewer_margin))],
            env=venv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    board_files = {"state": str(run_dir / "board_state.txt"),
                   "events": str(run_dir / "board_events.jsonl"),
                   "vt": str(run_dir / "board_vt.jsonl")}
    fw = VideoWriterThread(follow_mp4, a.render_hz)
    vw_ = VideoWriterThread(field_mp4, a.field_hz)
    field_rec = (FieldRecorder(field_view, vw_, a.field_hz)
                 if a.field_hz >= 0.1 else None)
    render_t = 0.0

    import subprocess as _sp
    calib_prev: dict = {}
    calib_path = Path(a.calib)
    if calib_path.exists():
        try:
            calib_prev = json.loads(calib_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            calib_prev = {}
    quad_now = quad_fingerprint(a.board_ssh, a.ssh_key) if a.board_ssh else ""
    need_calib = (not calib_prev) or quad_moved(calib_prev.get("quad") or "", quad_now,
                                               a.quad_tol)
    print(f"  标定: 已有={bool(calib_prev)} 相机指纹={quad_now!r} "
          f"→ {'本轮跑完重新标定' if need_calib else '沿用现有系数（相机没动）'}",
          file=sys.stderr)

    if a.board_ssh and a.start_camera_pipe:
        _cam = start_camera_pipe(a.board_ssh, a.ssh_key, extra=a.vt_board_extra)
        print(f"  相机管线: {_cam}", file=sys.stderr)
        if not _cam.get("ok"):
            print("  ⚠ 相机管线没起来：板端会没有图像，本轮不会产生有效决策",
                  file=sys.stderr)
    if a.board_ssh:
        #: 开跑前先确认板端 VT 正在出新帧：否则整轮都是"读旧画面"，
        #: 结论会和现实脱节（我自己就被旧 cam.jpg 骗过一次）。
        try:
            _st = _sp.run(["ssh", "-i", a.ssh_key, "-o", "StrictHostKeyChecking=no",
                           "-o", "ConnectTimeout=8", a.board_ssh,
                           'now=$(date +%s); m=$(stat -c %Y /tmp/vt.jsonl 2>/dev/null '
                           '|| echo 0); echo $((now - m))'],
                          capture_output=True, timeout=20)
            _age = int((_st.stdout or b"-1").strip() or -1)
            print(f"  板端 VT 日志年龄 {_age}s", file=sys.stderr)
            if 0 <= _age > 5:
                print("  ⚠ 板端 VT 超过 5 s 没出新帧：先确认相机管线在跑", file=sys.stderr)
        except (OSError, _sp.SubprocessError, ValueError):
            pass
        #: 清场：上一轮残留的 board_agent 会继续写 /tmp/board_state.txt 和
        #: /tmp/vt.jsonl（两次 open("w") 交错 → 文件里混着两轮的数据），
        #: 证据日志就不可信了。用方括号模式避免 pkill 打到自己的命令行。
        import subprocess as _sp
        try:
            _sp.run(["ssh", "-i", a.ssh_key, "-o", "StrictHostKeyChecking=no",
                     "-o", "ConnectTimeout=8", a.board_ssh,
                     'pkill -f "[b]oard_agent.py"; pkill -f "[f]ield_cli_board"; '
                     'sleep 0.2; rm -f /tmp/board_state.txt /tmp/board_events.jsonl'],
                    capture_output=True, timeout=20)
        except (OSError, _sp.SubprocessError):
            pass
        link = SshLink(a.board_ssh, a.ssh_key, a.board_agent,
                       python=a.board_python, extra=a.board_extra)
        t_wait = time.monotonic()
        while link.moves == 0 and time.monotonic() - t_wait < a.connect_timeout:
            if not link.send_pose(*[float(v) for v in (env.world.player[0],
                                                       env.world.player[1],
                                                       env.world.player[4])]):
                time.sleep(0.05)
            time.sleep(0.02)
        print(f"  板子已发首批动作 moves={link.moves}", file=sys.stderr)
        log.event("board_handshake", moves=link.moves, pose_seq=link.pose_seq,
                  waited_s=round(time.monotonic() - t_wait, 2))
    else:
        link = Link(a.tcp)
    t0 = time.monotonic()
    next_page = t0
    next_field = t0
    t_sim = 0.0
    last_page = 0.0
    last_field = 0.0
    last_pose_t = 0.0
    last_page_render_t = t0
    reached = False
    collisions = 0
    min_clear: float | None = None
    min_clear_at: list | None = None
    abort_reason = None
    if not a.board_ssh:
        print("  等板子连 TCP ...", file=sys.stderr)
    prof = {"it": 0, "step": 0.0, "page": 0.0, "field": 0.0, "t": time.monotonic()}
    truth_rows = vtruth["rows"]
    rows_seen: dict[float, bool] = {}
    view_half = 240.0
    last_truth_snap = 0.0
    last_pose_seq = link.pose_seq
    while t_sim < a.max_seconds and not reached:
        now = time.monotonic()
        prof["it"] += 1

        #: ① 板子信号（@MOVE）：带主机时间戳，和 @POSE 的 seq 一起就能算延迟
        for (t_recv, act_recv) in link.drain_moves():
            log.io("move_recv", act=int(act_recv), t_sim=round(t_sim, 3),
                   age_ms=round(1000.0 * (now - t_recv), 2))

        with link._lock:
            act = link.act
        _t_st = time.monotonic()
        _o, _r, _t, _tr, info = env.step(int(act))
        _st_ms = 1000.0 * (time.monotonic() - _t_st)
        collisions += int(info.get("collision_events", 0) or 0)
        px, py = float(env.world.player[0]), float(env.world.player[1])
        r = float(env.world.player[4])
        #: 真值最小间隙（到最近墙面 - 角色半径），衡量"贴着墙走"的程度。
        clear_now: float | None = None
        nearest_row: float | None = None
        nearest_wall: list[float] | None = None
        for (cx, cy, hw, hh) in walls_of(env):
            if (cx - goal[0]) ** 2 + (cy - goal[1]) ** 2 <= (goal_r * 2.5) ** 2:
                continue          # 终点块不是墙，别把"进终点"算成穿墙
            d = dist_rect(px, py, cx, cy, hw, hh) - float(spec.player_radius)
            if clear_now is None or d < clear_now:
                clear_now = float(d)
                nearest_wall = [round(cx, 1), round(cy, 1), round(hw, 1), round(hh, 1)]
                nearest_row = round(float(cy), 1)
            if d < (min_clear if min_clear is not None else 1e9):
                min_clear = float(d)
                min_clear_at = [round(px, 1), round(py, 1), round(t_sim, 2)]
                log.event("min_clearance", value=round(float(d), 2), x=round(px, 1),
                          y=round(py, 1), t_sim=round(t_sim, 2),
                          wall=nearest_wall)
        t_sim += dt

        #: ② 角色轨迹：每步一行（够细才能回放"怎么动的"）
        log.step(t_sim=round(t_sim, 4), px=round(px, 2), py=round(py, 2),
                 r=round(r, 2), act=int(act),
                 dist_goal=round(float(((px - goal[0]) ** 2
                                        + (py - goal[1]) ** 2) ** 0.5), 2),
                 clear=None if clear_now is None else round(clear_now, 2),
                 row=nearest_row)

        #: ③ 虚拟真值行：第一次进屏的那一刻记下来（对照板端什么时候才知道）
        for row in truth_rows:
            ry = float(row["y"])
            if rows_seen.get(ry):
                continue
            if ry <= py + view_half + 1.0:
                rows_seen[ry] = True
                log.truth_rows.write({
                    "t_wall": log.wall_t(),
                    "t_sim": round(t_sim, 3), "py": round(py, 1), "y": ry,
                    "gap_gl": row["gap_gl"], "gap_gr": row["gap_gr"],
                    "gap_side": row["gap_side"], "gap_center": row["gap_center"],
                    "gap_width": row["gap_width"],
                })

        #: ④ 本地真值快照（2 Hz）：角色 ±400 内的墙矩形，便于事后复现"当时场上有哪些墙"
        if t_sim - last_truth_snap >= 0.5:
            last_truth_snap = t_sim
            near = [[round(cx, 1), round(cy, 1), round(hw, 1), round(hh, 1)]
                    for (cx, cy, hw, hh) in walls_of(env) if abs(cy - py) <= 400.0]
            log.local_truth.write({"t_wall": log.wall_t(),
                                   "t_sim": round(t_sim, 3), "px": round(px, 1),
                                   "py": round(py, 1), "walls": near})

        if now - last_pose_t >= 0.04:
            #: 姿态 25 Hz 就够（板端 VT 才 2 Hz），发太快 ssh 通道会把主循环拖慢。
            last_pose_t = now
            ok = link.send_pose(px, py, r)
            if link.pose_seq != last_pose_seq:
                last_pose_seq = link.pose_seq
                log.io("pose_sent", seq=int(link.pose_seq), x=round(px, 2),
                       y=round(py, 2), r=round(r, 2), ok=bool(ok),
                       t_sim=round(t_sim, 3))
        if a.render_hz > 0 and now >= next_page:
            #: 固定节拍推进（不是 now+period）：迭代按 60 步/s 走，now 抖动会让
            #: "两拍"变成"三拍"，30 fps 直接掉到 23。
            next_page += 1.0 / max(1.0, a.render_hz)
            if next_page <= now:
                next_page = now + 1.0 / max(1.0, a.render_hz)
            _rt = time.monotonic()
            fr = page.render(env.world)
            _pg_ms = 1000.0 * (time.monotonic() - _rt)
            render_t += time.monotonic() - _rt
            prof["page"] += _pg_ms
            prof["pg"] = prof.get("pg", 0) + 1
            if fr is not None:
                publisher.submit(fr)
                fw.write(fr)
                log.frame("page", t_sim=round(t_sim, 3), ms=round(_pg_ms, 2),
                          px=round(px, 1), py=round(py, 1),
                          gap_ms=round(1000.0 * (now - last_page_render_t), 2))
                last_page_render_t = now
        if field_rec is not None and now >= next_field:
            next_field = now + 1.0 / max(0.1, a.field_hz)
            _ft = time.monotonic()
            field_rec.submit(env.world)
            _fd_ms = 1000.0 * (time.monotonic() - _ft)
            prof["field"] += _fd_ms
            log.frame("field", t_sim=round(t_sim, 3), ms=round(_fd_ms, 2),
                      queued=field_rec.q.qsize(), dropped=field_rec.dropped)
        if (px - goal[0]) ** 2 + (py - goal[1]) ** 2 <= goal_r ** 2:
            reached = True
        if getattr(link, "geom_bad", False) and t_sim > 8.0:
            abort_reason = "camera_not_page"
            break
        prof["step"] += _st_ms
        if a.profile and time.monotonic() - prof["t"] >= 2.0:
            el = time.monotonic() - prof["t"]
            n = max(1, prof["it"])
            print(f"  [prof] it/s={prof['it'] / el:.1f} step={prof['step'] / n:.2f}ms "
                  f"page={prof['page'] / el:.1f}ms/s field={prof['field'] / el:.1f}ms/s "
                  f"pg/s={prof.get('pg', 0) / el:.1f}",
                  file=sys.stderr, flush=True)
            prof = {"it": 0, "step": 0.0, "page": 0.0, "field": 0.0,
                    "t": time.monotonic()}
        # 按 speed 节流
        due = t0 + t_sim / max(1e-6, a.speed)
        lag = due - time.monotonic()
        if lag > 0:
            #: 一次睡够（上限 50 ms）：分段 10 ms 睡会把循环节拍打散，30 fps 的
            #: 页面渲染就会不断错过 deadline（实测 25 fps）。
            time.sleep(min(lag, 0.05))

    summary = {
        "scenario": spec.name, "reached": reached, "t_sim_s": round(t_sim, 3),
        "wall_s": round(time.monotonic() - t0, 2),
        "final_pos": [round(px, 1), round(py, 1)],
        "collisions": collisions, "moves": link.moves, "poses": link.poses,
        "min_clearance": None if min_clear is None else round(float(min_clear), 2),
        "min_clearance_at": min_clear_at,
        "page_fps": round(publisher.published / max(1e-6, time.monotonic() - t0), 2),
        "page_dropped": publisher.dropped,
        "rec_pages": fw.written, "rec_fields": vw_.written,
        "rec_dropped": [fw.dropped, vw_.dropped],
        "render_ms": round(1000.0 * render_t / max(1, fw.written + vw_.written), 2),
        "peer": link.peer,
        "videos": {"follow": str(follow_mp4), "field": str(field_mp4)},
        "abort_reason": abort_reason,
        "decision_source": "camera",
        "run_dir": str(run_dir),
        "logs": log.counts(),
    }
    if a.board_ssh:
        summary["board_files"] = fetch_board_files(a.board_ssh, a.ssh_key, run_dir)
    #: 自动标定（用户要求：不手调系数；相机/窗口一动就重标；标好后不再调用）
    if need_calib:
        _res = None
        try:
            import calib_y
            _cfg = json.loads(Path(a.scenario).read_text(encoding="utf-8"))
            _truth = [float(v) for v in (_cfg.get("meta", {}).get("walls_y") or [])]
            _pairs = calib_y.collect_pairs(run_dir)
            if not _truth:
                print("  自动标定跳过：场景没有真值墙行", file=sys.stderr)
            elif len(_pairs) < 30:
                print(f"  自动标定跳过：样本不足（{len(_pairs)}）", file=sys.stderr)
            else:
                _res = calib_y.fit(_pairs, _truth, 240.0, tol=30.0, rounds=6)
                _res.update({"quad": quad_now, "run": str(run_dir),
                             "scenario": a.scenario})
                if (_res.get("samples", 0) < 30
                        or (_res.get("rmse") or 999.0) > 20.0):
                    print(f"  自动标定质量不足（samples={_res.get('samples')} "
                          f"rmse={_res.get('rmse')}）：保留旧系数", file=sys.stderr)
                    summary["calib"] = {"rejected": True, **_res}
                    _res = None
        except (ImportError, OSError, ValueError) as exc:
            print(f"  自动标定失败：{exc!r}", file=sys.stderr)

        if _res is not None:
            calib_path.parent.mkdir(parents=True, exist_ok=True)
            calib_path.write_text(json.dumps(_res, ensure_ascii=False, indent=2) + "\n",
                                  encoding="utf-8")
            summary["calib"] = _res
            if a.board_ssh:
                try:
                    import subprocess as _sp2
                    _sp2.run(["ssh", "-i", a.ssh_key, "-o", "StrictHostKeyChecking=no",
                              "-o", "ConnectTimeout=8", a.board_ssh,
                              f"cat > {a.calib_board}"],
                             input=(json.dumps(_res, ensure_ascii=False) + "\n").encode(),
                             capture_output=True, timeout=20)
                except (OSError, _sp2.SubprocessError):
                    pass
            print(f"  自动标定: scale={_res['scale']} bias={_res['bias']} "
                  f"samples={_res['samples']} rmse={_res['rmse']}", file=sys.stderr)
    else:
        summary["calib"] = {"reused": True, "scale": calib_prev.get("scale"),
                            "bias": calib_prev.get("bias"),
                            "quad": calib_prev.get("quad")}

    (Path(str(stem) + ".json")).write_text(json.dumps(summary, ensure_ascii=False,
                                                      indent=2) + "\n")
    (run_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False,
                                                     indent=2) + "\n")
    log.close()
    print(json.dumps(summary, ensure_ascii=False, indent=2), file=sys.stderr)
    print(f"  证据日志 {run_dir}", file=sys.stderr)

    time.sleep(a.hold)
    publisher.stop()
    fw.close()
    vw_.close()
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
