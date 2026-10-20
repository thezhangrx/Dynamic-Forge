#!/usr/bin/env python3
"""完整闭环：**跟随视口 + 板子 M1 + 相机白框视觉（新墙门控）**。

链路（一条时间轴）：

    本机 bullet_sim（static_wall_wide，跟随视口 640×480）
        │  每步 @STATE（只发**已知 VT 墙**）→ ssh → 板子 field_decision_cli
        │  @MOVE ←─────────────────────────────────────────────┘
        │  渲染 640×480 干净平台页（白框/墙/角色，无 marker、无势场）
        ▼
    8100 MJPEG 页面 ──→ WSLg X11 窗口（page_view.py，屏幕 (320,60) 1280×960）
        │  板子相机拍屏幕 → /dev/video0 → 板子 mjpeg_server（8099）
        ▼
    相机白框 VT（vt_from_vision）→ 墙/缺口/角色
        │  新墙第一次稳定出现 → **暂停**（不 step）、等 VT 稳定
        ▼
    更新已知 VT 墙集合 → 恢复板子闭环（新墙从下一帧 @STATE 开始进板子的代价场）

口径（见 `虚拟真值约定.md`）：
  * VT（仿真真值 + 板子指令积分）是**唯一控制真值**；
  * 相机视觉只做两件事：① 初始角色位置（全幅画面，只记不反馈）；
    ② 新墙第一次稳定出现时写 VT 墙集合；
  * 角色**零校正**；安全重锚只在连续 5 帧 |视觉−VT| > 60 时记一次 `n_reinit`。

产物（默认 `data/fpga/loop_logs/`）：
    full_loop_<ts>_follow.mp4     干净跟随视口（相机看到的同一画面）+ 暂停横幅
    full_loop_<ts>_field.mp4      跟随视口 + **势场/代价场**（只录不投屏）
    full_loop_<ts>_camera_vt.mp4  相机 | 视觉 VT 左右并排 + 暂停横幅
    full_loop_<ts>/state.txt      实际发出的 @STATE
    full_loop_<ts>/actions.txt    @MOVE 流
    full_loop_<ts>/board.log      板子 stdout/stderr（含自检两行）
    full_loop_<ts>/events.jsonl   pause/resume/新墙/重锚/初始位置
    full_loop_<ts>/vt_samples.jsonl  相机 VT 逐样本（墙/角色/几何）
    full_loop_<ts>/full_loop.json 汇总指标 + 暂停证据
    full_loop_<ts>/follow_last.png / camera_last.png

⚠ 为什么投屏页**不带势场**：实测把 `show_potential=True` 的画面给相机，白框 VT
  在 rectified 图上检出 **0 道墙**（势场的亮绿底把墙的局部对比翻转了）；干净页
  在同一机位稳定检出 2/3/4 道墙。势场/代价场单独录到 `_field.mp4`。

用法（真板子 + 相机，默认）：

    Env/bin/python core/board/tools/full_loop.py --max-seconds 45 --speed 0.5

本地无相机自测：

    Env/bin/python core/board/tools/full_loop.py --board local \
        --local-bin /tmp/field_decision_cli_host --vt-source none --no-viewer \
        --send-all-walls --speed 2 --max-seconds 20

相机不可用时**明确报错**（不会用真值假装视觉）：

    ... --vt-source none        # 只跑板子闭环，pause 数必为 0
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parents[2]
for _p in (str(_ROOT), str(_HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from field_loop import board_command, dist_rect, pct, walls_of   # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec                # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv                # noqa: E402
from bullet_sim.render.pygame_view import PygameRenderer          # noqa: E402

from mjpeg_client import MjpegClient                              # noqa: E402

import cv2                                                        # noqa: E402

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_RED = (0, 0, 220)
_GREEN = (60, 210, 90)
_YELLOW = (0, 210, 235)
_WHITE = (240, 240, 240)


# ======================================================================
# 8100 页面
# ======================================================================
class FrameServer:
    """线程版 MJPEG 页面：主循环 publish()，浏览器/查看器 /stream.mjpg 取帧。"""

    PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>platform live (follow)</title>
<style>
html,body{margin:0;padding:0;background:#050608;overflow:hidden}
img{display:block;width:100vw;height:100vh;object-fit:contain;background:#050608}
</style></head><body>
<img id="v" src="/stream.mjpg">
</body></html>
"""

    VIEW = """<!doctype html>
<html><head><meta charset="utf-8"><title>platform</title>
<style>html,body{margin:0;background:#000;overflow:hidden}
img{display:block;width:100vw;height:100vh;object-fit:contain}</style>
</head><body><img src="/stream.mjpg"></body></html>
"""

    def __init__(self, port: int, *, quality: int = 85) -> None:
        self.port = int(port)
        self.quality = int(quality)
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = -1
        self._published = 0

        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *args):        # noqa: D401
                pass

            def _html(self, body: str) -> None:
                data = body.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):                          # noqa: N802
                u = self.path.split("?")[0]
                if u in ("/", "/view"):
                    self._html(FrameServer.PAGE if u == "/" else FrameServer.VIEW)
                    return
                if u == "/last.jpg":
                    jpg = owner.jpeg()
                    if not jpg:
                        self.send_error(404)
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(jpg)
                    return
                if u == "/status":
                    body = json.dumps({"seq": owner.seq, "published": owner.published})
                    data = body.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if u == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    last = -1
                    try:
                        while True:
                            got = owner.wait_frame(last + 1, timeout=1.0)
                            if got is None:
                                continue
                            last, jpg = got
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(
                                ("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
                    return
                self.send_error(404)

        self._httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self._httpd.daemon_threads = True
        self._httpd.fs = self
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="page-http", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    def publish(self, rgb: np.ndarray) -> bool:
        if rgb is None:
            return False
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.shape[2] == 3 else rgb
        ok, enc = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return False
        jpg = enc.tobytes()
        with self._cond:
            self._jpeg = jpg
            self._seq += 1
            self._published += 1
            self._cond.notify_all()
        return True

    def wait_frame(self, after: int, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._seq < after and self._jpeg is not None:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)
            if self._jpeg is None or self._seq < after:
                return None
            return self._seq, self._jpeg

    def jpeg(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    @property
    def seq(self) -> int:
        with self._cond:
            return self._seq

    @property
    def published(self) -> int:
        with self._cond:
            return self._published

    def close(self) -> None:
        try:
            self._httpd.shutdown()
        except Exception:                             # noqa: BLE001
            pass


class PagePublisher:
    """把页面帧的 **JPEG 编码 + HTTP 推送** 挪出主循环。

    为什么：640×480 一帧的 `cvtColor+imencode` 实测 3~5 ms，加上 pygame 渲染
    3.3 ms，主循环每帧要付 ~7 ms。15 fps 时占 10%，30 fps 就 20%，60 fps 40%，
    平台一高（24/40 道墙）本来实时率就低，再背这个就更卡。

    现在主循环只产 RGB 帧（`submit()` 覆盖式入槽，不阻塞、旧帧直接丢），
    该线程按 `fps` 编码并 `publish()`。页面可以稳定跑到 30~60 fps，
    主循环的额外成本只有一次 `notify`（微秒级）。

    `pygame.surfarray.array3d()` 返回的是拷贝，所以主线程可以立刻渲染下一帧。
    """

    def __init__(self, server: FrameServer, fps: float = 30.0) -> None:
        self.server = server
        self.fps = max(1.0, float(fps))
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._stop = False
        self.published = 0
        self.dropped = 0
        self._thread = threading.Thread(target=self._loop, name="page-publish",
                                        daemon=True)

    # ------------------------------------------------------------------
    def start(self) -> "PagePublisher":
        self._thread.start()
        return self

    def submit(self, rgb) -> None:
        if rgb is None:
            return
        with self._cond:
            if self._frame is not None:
                self.dropped += 1          # 编码跟不上产帧：只保留最新一帧
            self._frame = rgb
            self._cond.notify()

    def stop(self) -> None:
        with self._cond:
            self._stop = True
            self._cond.notify_all()
        th = self._thread
        if th.is_alive():
            th.join(timeout=2.0)

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        #: **不做 sleep 节流**：产帧频率已由主循环的 --render-hz 控制（30 fps 就是
        #: 每 33 ms 一帧），线程只需要"来一帧编一帧"。之前这里用 time.sleep 对齐
        #: 30 fps，WSL 下 sleep 粒度 5~20 ms，实测只跑到 24 fps；去掉后按产帧节奏
        #: 推送，30 fps 目标能全部送达（编码跟不上时槽位只留最新帧，见 submit）。
        while True:
            with self._cond:
                while self._frame is None and not self._stop:
                    self._cond.wait(0.2)
                if self._stop:
                    return
                frame, self._frame = self._frame, None
            if self.server.publish(frame):
                self.published += 1


# ======================================================================
# 相机 VT
# ======================================================================
class CameraVT:
    """板子相机 MJPEG → 白框 VT（`vt_from_vision` 一帧一个子进程）。

    为什么一帧一个子进程：VT 程序是 stdin 流式的，但我们要"最新一帧"而不是排队
    回放；一帧 640×480 的 VT 约 0.16 s，3 Hz 采样下 CPU 足够。子进程的 stdout 是
    640×480 的 VT 渲染（给对照视频），--jsonl 是数值。
    """

    def __init__(self, url: str, vt_bin: str, hz: float, run_dir: Path,
                 camera_w: int = 640, camera_h: int = 480) -> None:
        self.url = url
        self.vt_bin = vt_bin
        self.hz = max(0.5, float(hz))
        self.run_dir = run_dir
        self.camera_w = int(camera_w)
        self.camera_h = int(camera_h)
        self.client = MjpegClient(url)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._pending: list[dict] = []
        self._seq = 0
        self._log_path = run_dir / "vt_samples.jsonl"
        self._tmp_jsonl = run_dir / "vt_tmp.jsonl"
        self._log_f = None
        self.err = ""
        self.rc_last = 0
        self.samples = 0
        self.vt_ms_last = 0.0
        self.cam_shape = None

    # ------------------------------------------------------------------
    def start(self) -> None:
        try:
            self._log_f = open(self._log_path, "w", encoding="utf-8")
        except OSError:
            self._log_f = None
        self.client.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="camera-vt", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.client.stop()
        th = self._thread
        if th is not None:
            th.join(timeout=5.0)
        self._thread = None
        if self._log_f is not None:
            try:
                self._log_f.close()
            except Exception:                         # noqa: BLE001
                pass
            self._log_f = None

    # ------------------------------------------------------------------
    def latest(self) -> dict | None:
        with self._lock:
            return self._latest

    def latest_after(self, seq: int) -> dict | None:
        with self._lock:
            if self._latest is not None and self._latest["seq"] > seq:
                return self._latest
            return None

    def drain(self) -> list[dict]:
        with self._lock:
            out = self._pending
            self._pending = []
        return out

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        last_frame_seq = -1
        next_due = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_due:
                self._stop.wait(min(0.05, next_due - now))
                continue
            got = self.client.latest(after_seq=last_frame_seq, timeout=0.25)
            if got is None:
                if self.client.errors and self.client.connects == 0:
                    self.err = self.client.last_error
                continue
            last_frame_seq, t_frame, jpg = got
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.err = "camera jpeg decode failed"
                continue
            h, w = img.shape[:2]
            self.cam_shape = (w, h)
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            t0 = time.monotonic()
            cmd = [self.vt_bin, "--w", str(w), "--h", str(h),
                   "--frames", "1", "--skip", "0",
                   "--frame-roi", "0", "0", str(w), str(h),
                   #: 四角跨帧平滑（存在文件里，因为一帧一个子进程）
                   "--quad-state", str(self.run_dir / "quad_state.txt"),
                   "--jsonl", str(self._tmp_jsonl)]
            try:
                proc = subprocess.run(cmd, input=rgb.tobytes(),
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                      timeout=20.0)
            except Exception as exc:                  # noqa: BLE001
                self.err = f"vt subprocess: {exc!r}"
                self._stop.wait(0.2)
                continue
            vt_ms = (time.monotonic() - t0) * 1e3
            self.rc_last = int(proc.returncode)
            if proc.returncode != 0:
                self.err = (proc.stderr or b"")[-300:].decode("utf-8", "replace")
                next_due = time.monotonic() + 1.0 / self.hz
                continue
            rec = None
            try:
                line = self._tmp_jsonl.read_text(encoding="utf-8").strip().splitlines()[-1]
                rec = json.loads(line)
            except Exception as exc:                  # noqa: BLE001
                self.err = f"vt jsonl: {exc!r}"
            vt_bgr = None
            try:
                if len(proc.stdout) == 640 * 480 * 3:
                    vt_rgb = np.frombuffer(proc.stdout, np.uint8).reshape(480, 640, 3)
                    vt_bgr = cv2.cvtColor(vt_rgb, cv2.COLOR_RGB2BGR)
            except Exception:                         # noqa: BLE001
                vt_bgr = None
            with self._lock:
                self._seq += 1
                sample = {
                    "seq": self._seq,
                    "t_wall": t_frame,
                    "t_done": time.monotonic(),
                    "vt_ms": round(vt_ms, 1),
                    "walls": list((rec or {}).get("walls") or []),
                    "player": (rec or {}).get("player"),
                    "frame_geom": int((rec or {}).get("frame_geom") or 0),
                    "markers": int((rec or {}).get("markers") or 0),
                    "extent": int((rec or {}).get("extent") or 0),
                    "n": int((rec or {}).get("n") or 0),
                    "cam_bgr": img,
                    "vt_bgr": vt_bgr,
                    "raw": rec,
                }
                self._latest = sample
                self._pending.append(sample)
                if len(self._pending) > 8:
                    self._pending = self._pending[-8:]
                self.samples += 1
                self.vt_ms_last = vt_ms
            if self._log_f is not None:
                slim = {k: v for k, v in sample.items()
                        if k not in ("cam_bgr", "vt_bgr", "raw")}
                self._log_f.write(json.dumps(slim, ensure_ascii=False) + "\n")
                self._log_f.flush()
            next_due = time.monotonic() + 1.0 / self.hz


# ======================================================================
# 板子链路
# ======================================================================
class BoardLink:
    def __init__(self, cmd: list[str], log_path: Path,
                 actions_f=None) -> None:
        self.log_f = open(log_path, "w", encoding="utf-8")
        self.actions_f = actions_f
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        self.lock = threading.Lock()
        self.act = 0
        self.send_t: dict[int, float] = {}
        self.lat: list[float] = []
        self.acks = 0
        self.moves = 0
        self.last_ack: float | None = None
        self.eof = False
        self.selfcheck: list[str] = []
        self.lines = 0
        self._th = threading.Thread(target=self._read, name="board-reader", daemon=True)
        self._th.start()

    def _read(self) -> None:
        assert self.proc.stdout is not None
        for raw in self.proc.stdout:
            now = time.monotonic()
            line = raw.rstrip("\n")
            with self.lock:
                self.lines += 1
                try:
                    self.log_f.write(line + "\n")
                    self.log_f.flush()
                except Exception:                     # noqa: BLE001
                    pass
                if line.startswith("@MOVE"):
                    self.moves += 1
                    try:
                        self.act = int(line.split()[1])
                    except (IndexError, ValueError):
                        pass
                    self.last_ack = now
                    if self.actions_f is not None:
                        try:
                            self.actions_f.write(line + "\n")
                            self.actions_f.flush()
                        except Exception:             # noqa: BLE001
                            pass
                elif line.startswith("@ACK"):
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            seq = int(parts[1])
                            self.act = int(parts[2])
                        except ValueError:
                            continue
                        self.acks += 1
                        self.last_ack = now
                        t0 = self.send_t.pop(seq, None)
                        if t0 is not None:
                            self.lat.append(now - t0)
                elif line.startswith("[field]"):
                    self.selfcheck.append(line)
        with self.lock:
            self.eof = True

    def send(self, seq: int, line: str) -> None:
        with self.lock:
            self.send_t[seq] = time.monotonic()
        assert self.proc.stdin is not None
        self.proc.stdin.write(line)
        self.proc.stdin.flush()

    def close(self) -> None:
        try:
            if self.proc.stdin is not None:
                self.proc.stdin.close()
        except Exception:                             # noqa: BLE001
            pass
        try:
            self.proc.wait(timeout=3.0)
        except Exception:                             # noqa: BLE001
            try:
                self.proc.kill()
            except Exception:                         # noqa: BLE001
                pass
        self._th.join(timeout=2.0)
        try:
            self.log_f.close()
        except Exception:                             # noqa: BLE001
            pass


# ======================================================================
# 小工具
# ======================================================================
def _text(img: np.ndarray, s: str, xy: tuple[int, int], *,
          color=_WHITE, scale=0.42, thick=1) -> None:
    cv2.putText(img, s, xy, _FONT, scale, color, thick, cv2.LINE_AA)


def _banner(img: np.ndarray, s: str, color=_RED) -> None:
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, h - 26), (w - 1, h - 1), color, -1)
    _text(img, s, (6, h - 8), color=_WHITE, scale=0.48, thick=1)


def _annotate(img: np.ndarray, line1: str, line2: str = "",
              paused: bool = False) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]
    cv2.rectangle(out, (0, 0), (w - 1, 24), (0, 0, 0), -1)
    _text(out, line1, (5, 17), color=(225, 225, 225), scale=0.42)
    if paused:
        _banner(out, "PAUSED: new wall discovered - waiting for stable camera VT")
    elif line2:
        _text(out, line2, (5, h - 8), color=_YELLOW, scale=0.42)
    return out


def _open_writer(path: Path, fps: float, size: tuple[int, int]):
    path.parent.mkdir(parents=True, exist_ok=True)
    wr = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                         max(1.0, float(fps)), size)
    if not wr.isOpened():
        print(f"⚠ 打不开视频写入器：{path}", file=sys.stderr)
        return None
    return wr


def _validate_video(path: Path) -> dict:
    if not path.exists():
        return {"path": str(path), "exists": False, "frames": 0}
    cap = cv2.VideoCapture(str(path))
    ok = cap.isOpened()
    frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if ok else 0
    fps = float(cap.get(cv2.CAP_PROP_FPS)) if ok else 0.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if ok else 0
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if ok else 0
    cap.release()
    return {"path": str(path), "exists": True, "opened": ok, "frames": frames,
            "fps": round(fps, 2), "size": [w, h],
            "bytes": path.stat().st_size}


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    return xs[len(xs) // 2]


def _walls_close(a, b, tol: float = 8.0) -> bool:
    if len(a) != len(b):
        return False
    aa = sorted(float(w["y"]) for w in a)
    bb = sorted(float(w["y"]) for w in b)
    return all(abs(x - y) <= tol for x, y in zip(aa, bb))


def _wall_ys(walls) -> list[float]:
    return sorted(float(w["y"]) for w in walls)


def _shift_match(dets_vy: list[float], known_ys: list[float],
                 py_ref: float, half: float,
                 tol: float = 28.0, window: float = 150.0):
    """估计相机链路带来的**公共平移** δ，并返回 (δ, 已匹配下标集合)。

    为什么需要：VT 帧比"当前仿真位置"旧，`py_at(t - latency)` 只能近似；同一帧里
    所有墙的 y 都有一个**相同的**偏移 δ（≈ 当前 y − 拍这帧时的 y）。用已知墙
    对齐出 δ，再比较，才不会把"已知墙整体平移 50 单位"误判成新墙。

    做法：先取 `det - 最近已知墙期望位置` 的中位数 d0，再在 d0±window 内搜索让
    匹配数最多的 δ（墙间距 120，避免 δ±120 的别名）。
    """
    if not known_ys:
        return 0.0, set()
    expected = [k - (py_ref - half) for k in known_ys]
    diffs: list[float] = []
    for vy in dets_vy:
        e = min(expected, key=lambda q: abs(vy - q))
        if abs(vy - e) < 150.0:
            diffs.append(vy - e)
    d0 = _median(diffs) if diffs else 0.0
    best_score = (-1, -1e9)
    best_delta = d0
    best_matched: set[int] = set()
    steps = int(round(2.0 * window))          # 0.5 单位步长
    for i in range(-steps, steps + 1):
        d = d0 + 0.5 * i
        matched = {j for j, vy in enumerate(dets_vy)
                   if min(abs(vy - (e + d)) for e in expected) <= tol}
        score = (len(matched), -abs(d - d0))
        if score > best_score:
            best_score = score
            best_delta = d
            best_matched = matched
    return best_delta, best_matched


# ======================================================================
# 主流程
# ======================================================================
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="跟随视口 + 板子 M1 + 相机白框 VT 新墙门控")
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--max-seconds", type=float, default=45.0, help="仿真时间上限")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="仿真倍率（默认 0.5：相机 VT 3 Hz、200 单位/秒也能在新墙前停住）")
    ap.add_argument("--state-hz", type=float, default=120.0)
    ap.add_argument("--render-hz", type=float, default=30.0,
                    help="投屏页帧率（编码/推送在线程，主循环不背 JPEG 成本）")
    ap.add_argument("--field-hz", type=float, default=5.0, help="势场录像帧率")
    ap.add_argument("--vt-hz", type=float, default=3.0, help="相机 VT 采样率")
    ap.add_argument("--port", type=int, default=8100)
    ap.add_argument("--out-dir", default="data/fpga/loop_logs")
    ap.add_argument("--tag", default="")
    ap.add_argument("--goal", type=float, nargs=2, default=None, metavar=("X", "Y"))
    ap.add_argument("--field-w", type=float, default=None)
    ap.add_argument("--field-h", type=float, default=None)
    ap.add_argument("--board", choices=("ssh", "local"), default="ssh")
    ap.add_argument("--local-bin", default="/tmp/field_decision_cli_host")
    ap.add_argument("--board-binary", default="/root/tools/field_decision_cli")
    ap.add_argument("--ssh-key", default="data/fpga/board_ssh/board_key")
    ap.add_argument("--board-host", default="root@192.168.137.242")
    ap.add_argument("--board-ema", type=float, default=None,
                    help="板端方向平滑（不给=用二进制默认）")
    ap.add_argument("--board-hyst", type=int, default=None,
                    help="板端换向滞回帧数")
    ap.add_argument("--board-reflex", type=int, default=None,
                    help="1/0 开关板端安全反射")
    ap.add_argument("--board-trace", default=None,
                    help="板端逐帧决策轨迹文件（诊断用）")
    ap.add_argument("--board-inflate", type=float, default=None,
                    help="板端规划膨胀半径（默认 13；调大可防贴墙角切角）")
    ap.add_argument("--board-view-h", type=float, default=0.0,
                    help="板子 --view-h（0 = 全幅 = 场高）")
    ap.add_argument("--stall-s", type=float, default=5.0)
    ap.add_argument("--send-all-walls", action="store_true",
                    help="不按已知 VT 墙过滤 @STATE（默认关闭：新墙要等视觉确认才进板子）")
    ap.add_argument("--cam-latency", type=float, default=0.30,
                    help="相机→页面→VT 的估计延迟（秒），只用于把 VT 坐标映射回场地 y")
    ap.add_argument("--wall-match-tol", type=float, default=55.0,
                    help="VT 墙映射回场地后与已知墙的最大匹配距离")
    ap.add_argument("--new-wall-margin", type=float, default=30.0,
                    help="候选新墙必须比已知最高墙再高多少")
    ap.add_argument("--vt-y-bias", type=float, default=-8.0,
                    help="VT 墙行固定偏移（**在环实测**：−8 时最小间隙转正；"
                         "冻结页标定给 −5.5，但在环相机链路更慢，先用 −8）")
    ap.add_argument("--vt-y-scale", type=float, default=1.0,
                    help="VT 墙行尺度修正（冻结页标定 ~0.98；在环尚未验证，默认 1）")
    ap.add_argument("--gap-min-width", type=float, default=75.0,
                    help="VT 量到的缺口宽度低于此值视为不可信（真缺口 90~110）")
    ap.add_argument("--expect-alternate", type=int, default=1,
                    help="1 = 缺口左右交替场景：新行的缺口必须与上一行不同侧")
    ap.add_argument("--gap-margin", type=float, default=10.0,
                    help="把视觉量到的缺口两边**向外放**多少（实测 VT 偏窄；默认 10）")
    ap.add_argument("--follow-scale", type=float, default=1.0,
                    help="跟随视口额外缩放（<1 看得更远，墙同比例变小）")
    ap.add_argument("--pause-ahead", type=float, default=160.0,
                    help="新墙离角色还有多少场地单位时才暂停（越大越早停）")
    ap.add_argument("--pause-timeout", type=float, default=12.0)
    ap.add_argument("--init-timeout", type=float, default=20.0)
    ap.add_argument("--vt-source", choices=("camera", "none"), default="camera")
    ap.add_argument("--camera-url", default="http://192.168.137.242:8099/stream.mjpg")
    ap.add_argument("--vt-bin", default="/tmp/vt_from_vision")
    ap.add_argument("--viewer", choices=("auto", "none"), default="auto")
    ap.add_argument("--viewer-display", default=":0")
    ap.add_argument("--viewer-pos", default="320,60")
    ap.add_argument("--viewer-size", default="1280x960")
    ap.add_argument("--viewer-wait", type=float, default=2.0)
    ap.add_argument("--viewer-hold", type=float, default=5.0,
                    help="跑完以后再显示多少秒（随后关窗口；8100 页面保持可访问）")
    ap.add_argument("--keep-serving", action=argparse.BooleanOptionalAction, default=True,
                    help="跑完后继续在 8100 服务最后一帧（页面保持可访问）")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    root = _ROOT
    spec = ScenarioSpec.load(args.scenario)
    field_w = float(args.field_w if args.field_w is not None else spec.field_w)
    field_h = float(args.field_h if args.field_h is not None else spec.field_h)
    goal = (float(args.goal[0]) if args.goal is not None
            else float(spec.target_x if spec.target_x is not None else 320.0),
            float(args.goal[1]) if args.goal is not None
            else float(spec.target_y if spec.target_y is not None else 470.0))
    goal_r = float(spec.target_radius if spec.target_radius is not None else 24.0)
    dt = float(spec.dt)
    args.view_h = float(args.board_view_h) if args.board_view_h > 0 else field_h

    state_every = max(1, int(round(1.0 / (float(args.state_hz) * dt))))
    render_every_s = 1.0 / max(1.0, float(args.render_hz))
    field_every_s = 1.0 / max(0.5, float(args.field_hz))

    ts = time.strftime("%Y%m%d_%H%M%S")
    tag = f"_{args.tag}" if args.tag else ""
    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = root / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f"full_loop_{ts}{tag}"
    run_dir = out_dir / f"full_loop_{ts}{tag}"
    run_dir.mkdir(parents=True, exist_ok=True)
    follow_mp4 = out_dir / f"{stem.name}_follow.mp4"
    field_mp4 = out_dir / f"{stem.name}_field.mp4"
    camera_mp4 = out_dir / f"{stem.name}_camera_vt.mp4"
    state_path = run_dir / "state.txt"
    actions_path = run_dir / "actions.txt"
    board_log = run_dir / "board.log"
    events_path = run_dir / "events.jsonl"
    summary_path = run_dir / "full_loop.json"

    events_f = open(events_path, "w", encoding="utf-8")

    def event(rec: dict) -> None:
        rec = dict(rec)
        rec.setdefault("t_wall", round(time.monotonic() - t_start, 3))
        events_f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        events_f.flush()
        if args.verbose:
            print(f"  [event] {rec}", file=sys.stderr)

    t_start = time.monotonic()
    wall_start = t_start

    # ---------------- renderers（投屏用干净版；录像用带势场版） ------------
    common = dict(
        mode="rgb_array", window_size=(640, 480),
        show_hud=False, show_text=False, show_fiducials=False,
        show_world_axes=False, show_field_border=True, show_hitboxes=False,
        follow_player=False, follow_h=240.0,
        wall_color=(200, 60, 0), background=(8, 10, 14), field_bg=(34, 40, 55),
        resizable=False,
    )
    page_view = PygameRenderer(show_potential=False,
                               follow_scale=float(args.follow_scale), **common)
    #: 势场录像：每帧重算（原来 30 帧才更新一次，5 fps 下就是 6 s 一跳，
    #: 看起来像"势在闪"）。
    field_view = PygameRenderer(show_potential=True, potential_every=1,
                                follow_scale=float(args.follow_scale), **common)

    env = BulletHellEnv(spec, seed=spec.seed)
    env.reset(seed=spec.seed)
    px, py = float(env.world.player[0]), float(env.world.player[1])

    # ---------------- 8100 页面 ------------------------------------------
    try:
        server = FrameServer(args.port)
    except OSError as exc:
        print(f"⚠ 端口 {args.port} 不可用（{exc}）；先停掉旧的 8100：\n"
              f"    pkill -INT -f '[s]im_stream.py.*--port {args.port}'\n"
              f"或换 --port", file=sys.stderr)
        env.close()
        return 2
    base_frame = page_view.render(env.world)
    if base_frame is None:
        print("⚠ 渲染首帧失败", file=sys.stderr)
        env.close()
        return 2
    server.publish(base_frame)
    #: 编码/推送线程：主循环只产 RGB 帧，页面自己跑 30~60 fps（见 PagePublisher）。
    publisher = PagePublisher(server, fps=max(10.0, float(args.render_hz))).start()
    print(f"  页面 http://127.0.0.1:{args.port}/   （640×480，页面 {args.render_hz:g} fps，"
          f"编码/推送在线程）", file=sys.stderr)

    # ---------------- 查看器（WSLg X11） ---------------------------------
    viewer_proc = None
    viewer_log = None
    if args.viewer == "auto":
        disp = args.viewer_display
        xnum = disp.split(":")[-1].split(".")[0]
        if not Path(f"/tmp/.X11-unix/X{xnum}").exists():
            print(f"⚠ DISPLAY={disp} 不可用（/tmp/.X11-unix/X{xnum} 不存在）→ "
                  f"相机看不到页面；需要 WSLg 或手动开浏览器", file=sys.stderr)
            if args.vt_source == "camera":
                server.close(); env.close()
                return 3
        else:
            vw, vh = args.viewer_size.lower().split("x")
            vx, vy = args.viewer_pos.split(",")
            vlog = open(run_dir / "viewer.log", "w", encoding="utf-8")
            venv = dict(os.environ)
            venv["DISPLAY"] = disp
            venv["SDL_VIDEODRIVER"] = "x11"
            venv["SDL_VIDEO_WINDOW_POS"] = f"{vx},{vy}"
            cmd = [sys.executable, str(_HERE / "page_view.py"),
                   "--url", f"http://127.0.0.1:{args.port}/stream.mjpg",
                   "--x", str(vx), "--y", str(vy), "--w", str(vw), "--h", str(vh),
                   "--fps", f"{max(20.0, float(args.render_hz)):g}"]
            viewer_proc = subprocess.Popen(cmd, env=venv, stdout=vlog,
                                           stderr=subprocess.STDOUT)
            viewer_log = vlog
            print(f"  查看器 pid {viewer_proc.pid} @ ({vx},{vy}) {vw}×{vh} "
                  f"DISPLAY={disp}", file=sys.stderr)
            time.sleep(max(0.5, args.viewer_wait))
            if viewer_proc.poll() is not None:
                print(f"⚠ 查看器提前退出（rc={viewer_proc.returncode}），见 "
                      f"{run_dir / 'viewer.log'}", file=sys.stderr)
                if args.vt_source == "camera":
                    server.close(); env.close()
                    return 3

    # ---------------- 相机 VT -------------------------------------------
    vt = None
    if args.vt_source == "camera":
        vt_bin = Path(args.vt_bin)
        if not vt_bin.exists():
            print(f"  VT 二进制不存在，编译到 {vt_bin} ...", file=sys.stderr)
            cc = subprocess.run(
                ["g++", "-std=c++17", "-O2", "-I", str(root / "core/board"),
                 "-o", str(vt_bin), str(_HERE / "vt_from_vision.cpp")],
                capture_output=True, text=True)
            if cc.returncode != 0:
                print(f"⚠ 编译 vt_from_vision 失败：\n{cc.stderr[-1500:]}",
                      file=sys.stderr)
                server.close(); env.close()
                return 3
        vt = CameraVT(args.camera_url, str(vt_bin), args.vt_hz, run_dir)
        vt.start()
        print(f"  相机 VT：{args.camera_url} @ {args.vt_hz:g} Hz", file=sys.stderr)

    # ---------------- 录像 ----------------------------------------------
    follow_writer = _open_writer(follow_mp4, args.render_hz, (640, 480))
    field_writer = _open_writer(field_mp4, args.field_hz, (640, 480))
    camera_writer = _open_writer(camera_mp4, args.vt_hz, (1280, 480)) \
        if args.vt_source == "camera" else None

    # ---------------- 状态历史（VT 角色映射用） ---------------------------
    sim_hist: deque[tuple[float, float]] = deque(maxlen=20000)

    def vt_world_y(vy: float, py_ref: float) -> float:
        """VT 整流图的 y（0=带底、240=角色）→ 场地 y。

        标定（冻结页 + 真值，三个角色 y、共 13 道墙）：误差 ≈ +5.5 + 0.02·d
        （d = vy-240）。先按 0.98 收掉尺度，再减 5.5 的固定偏移。
        """
        #: 视口缩放不是 1:1 时（--follow-scale<1，画面更高），整流图 480 px
        #: 对应 2*follow_half 个场地单位，先把像素差换成场地单位再套标定。
        d_px = float(vy) - 240.0
        d_world = d_px * (2.0 * follow_half / 480.0)
        return float(py_ref) + d_world * args.vt_y_scale + args.vt_y_bias

    def note_py(y: float) -> None:
        sim_hist.append((time.monotonic(), float(y)))

    note_py(py)

    def py_at(t: float) -> float:
        for tt, pp in reversed(sim_hist):
            if tt <= t:
                return pp
        return sim_hist[0][1] if sim_hist else py

    # ---------------- 渲染 / 发布 ----------------------------------------
    # 这些计数器在 render_page / write_cam_samples / pause 里都要用，必须先定义。
    t_sim = 0.0
    steps = 0
    run_start = time.monotonic()       # 板子闭环的 paced 起点（不含初始化）
    follow_half = 240.0
    known: list[float] = []
    known_detail: list[dict] = []
    page_state = {"last_page": 0.0, "next_page": 0.0, "last_field": 0.0, "frames": 0,
                  "field_frames": 0, "record": False}
    status = {"phase": "init", "follow": False, "act": 0, "known": 0, "vt_n": 0}
    pause = {"active": False, "t0": 0.0, "target_gy": 0.0, "samples": [],
             "events": [], "last_resume": 0.0, "last_new_y": None,
             #: 一次暂停没确认到墙时，给同一个候选加冷却，否则"帧里那道墙还在"
             #: 会让主循环立刻再暂停一次 —— 实测 t_sim 卡在 7.3 s 反复暂停 32 次。
             "cooldown_until": 0.0, "skip_gy": 1.0e9,
             #: 刚加入的墙的缺口（gl,gr）：冷却期里同缺口的候选不再触发暂停，
             #: 否则"刚加完的墙还在 160 以内"会立刻再停一次。
             "cooldown_gaps": [], "target_gl": 0.0, "target_gr": 640.0}
    all_pauses: list[dict] = []

    def render_page(force: bool = False) -> None:
        now = time.monotonic()
        #: **按截止时间调度**，而不是"距上一帧 ≥ 1/30 s"：主循环按仿真步长走
        #: （120 Hz → 8.33 ms/次），用相对间隔会因取整稳定落到 24 fps（每 5 次）
        #: 或 34.6 fps（打折过头）；按截止时间累加，平均正好是 --render-hz。
        if not force:
            if now < page_state["next_page"]:
                return
            page_state["next_page"] = max(now, page_state["next_page"] + render_every_s)
        else:
            page_state["next_page"] = now + render_every_s
        page_state["last_page"] = now
        frame = page_view.render(env.world)
        if frame is None:
            return
        publisher.submit(frame)          # 编码/推送在 page-publish 线程
        page_state["frames"] += 1
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if follow_writer is not None and page_state["record"]:
            line1 = (f"follow t={t_sim:.2f}s act={status['act']} "
                     f"known={status['known']} vt={status['vt_n']} "
                     f"pos=({px:.0f},{py:.0f})")
            follow_writer.write(_annotate(frame_bgr, line1, paused=pause["active"]))
        if now - page_state["last_field"] >= field_every_s:
            page_state["last_field"] = now
            ff = field_view.render(env.world)
            if ff is not None:
                page_state["field_frames"] += 1
                if field_writer is not None and page_state["record"]:
                    line1 = (f"field t={t_sim:.2f}s act={status['act']} "
                             f"known={status['known']} pos=({px:.0f},{py:.0f})")
                    field_writer.write(_annotate(cv2.cvtColor(ff, cv2.COLOR_RGB2BGR),
                                                 line1, paused=pause["active"]))

    # ---------------- 相机样本 → 对照视频 + 残差 -------------------------
    residual = {"n": 0, "values": [], "consecutive_big": 0, "n_reject": 0,
                "n_reinit": 0, "last_reinit": -1e9, "wall_res": []}

    def write_cam_samples(samples: list[dict]) -> None:
        if not samples:
            return
        for s in samples:
            # 角色对照（只统计，不反馈）。跟随视口里角色应恒在 (px, half)。
            pl = s.get("player")
            if pl and s.get("frame_geom"):
                vx, vy = float(pl[0]), float(pl[1])
                if status["follow"]:
                    exp_x, exp_y = px, follow_half
                    ok_gate = abs(vx - exp_x) < 120 and abs(vy - exp_y) < 80
                else:
                    exp_x, exp_y = px, py
                    ok_gate = abs(vx - exp_x) < 120 and abs(vy - exp_y) < 120
                if ok_gate:
                    res = math.hypot(vx - exp_x, vy - exp_y)
                    residual["n"] += 1
                    residual["values"].append(res)
                    residual["consecutive_big"] = (
                        residual["consecutive_big"] + 1 if res > 60.0 else 0)
                    if (residual["consecutive_big"] >= 5
                            and time.monotonic() - residual["last_reinit"] > 8.0):
                        residual["last_reinit"] = time.monotonic()
                        residual["n_reinit"] += 1
                        residual["consecutive_big"] = 0
                        event({"event": "reinit", "reason": "player_residual>60 x5",
                               "residual": round(res, 1), "vt_seq": s["seq"],
                               "px": round(px, 1), "py": round(py, 1),
                               "n_reinit": residual["n_reinit"]})
                else:
                    residual["n_reject"] += 1
            # 墙对照：已知墙映射回视口后与 VT 检测的差（先对齐公共平移 δ）
            if status["follow"] and known and s.get("frame_geom"):
                py_cap = py_at(s["t_wall"] - args.cam_latency)
                walls = [w for w in (s.get("walls") or [])
                         if 20.0 <= float(w["y"]) <= 460.0]
                dets_vy = [float(w["y"]) for w in walls]
                delta, matched = _shift_match(dets_vy, known, py_cap, follow_half)
                for j in matched:
                    exp_vy = min((k - (py_cap - follow_half) + delta for k in known),
                                 key=lambda e: abs(dets_vy[j] - e))
                    residual["wall_res"].append(abs(dets_vy[j] - exp_vy))
            if camera_writer is not None:
                cam = s["cam_bgr"].copy()
                vtb = s["vt_bgr"].copy() if s.get("vt_bgr") is not None \
                    else np.zeros((480, 640, 3), np.uint8)
                py_cap = py_at(s["t_wall"] - args.cam_latency)
                # 已知墙在视口里的 y → VT 图上的屏幕行
                for k in known:
                    vy = k - (py_cap - follow_half)
                    row = int(round(480 - vy))
                    if 4 <= row < 476:
                        cv2.line(vtb, (0, row), (639, row), (0, 170, 0), 1)
                if pause["active"]:
                    vy = pause["target_gy"] - (py_cap - follow_half)
                    row = int(round(480 - vy))
                    if 0 <= row < 480:
                        cv2.line(vtb, (0, row), (639, row), (0, 0, 255), 2)
                        cv2.putText(vtb, "new wall", (8, max(16, row - 6)),
                                    _FONT, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                out = np.hstack([cam, vtb])
                cv2.rectangle(out, (0, 0), (1279, 22), (0, 0, 0), -1)
                _text(out, f"CAM  t={s['t_wall'] - t_start:6.2f}s  seq={s['seq']}  "
                           f"vt={s['vt_ms']:.0f}ms", (5, 16))
                _text(out, f"VT n={s['n']} geom={s['frame_geom']} "
                           f"player={s.get('player')}", (646, 16), color=(180, 230, 255))
                if pause["active"]:
                    _banner(out, "PAUSED: new wall discovered - waiting for stable camera VT")
                elif time.monotonic() - pause["last_resume"] < 2.0 and pause["last_new_y"]:
                    _banner(out, f"RESUME: VT wall set updated  y={pause['last_new_y']:.1f}",
                            color=(30, 140, 40))
                camera_writer.write(out)

    # ---------------- 初始位置 + 初始墙（相机视觉） ----------------------
    init_player = None
    init_player_res = None
    init_full_walls = None
    init_full_samples = 0
    init_follow_samples = 0
    last_vt_seq = -1

    def wait_samples(seconds: float) -> None:
        """等待时保持页面刷新 + 写相机对照视频。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            render_page()
            if vt is not None:
                write_cam_samples(vt.drain())
            time.sleep(0.01)

    if vt is not None:
        #: 全幅初始化只在“整场一屏能装下”的平台有意义（4 道墙 640×480）。
        #: 24/40 道墙的场地 2500~4100 高，整场 fit 后墙只有 1~2 px，相机 VT
        #: 不可能数出 ≥3 道墙（实测）→ 直接进跟随视口初始化。
        if field_h <= 1200.0:
            status["phase"] = "init_full"
            print("  初始位置：全幅画面等待相机 VT ...", file=sys.stderr)
            good_full: list[dict] = []
            deadline = time.monotonic() + args.init_timeout
            while time.monotonic() < deadline and len(good_full) < 3:
                render_page()
                write_cam_samples(vt.drain())
                s = vt.latest()
                if s is not None and s["seq"] > last_vt_seq:
                    last_vt_seq = s["seq"]
                    if (s["frame_geom"] and len(s["walls"]) >= 3 and s.get("player")):
                        good_full.append(s)
                time.sleep(0.02)
            init_full_samples = len(good_full)
            if good_full:
                init_player = [_median([s["player"][0] for s in good_full]),
                               _median([s["player"][1] for s in good_full])]
                init_player_res = math.hypot(init_player[0] - px, init_player[1] - py)
                init_full_walls = _median([len(s["walls"]) for s in good_full])
                event({"event": "init_full", "vision_player":
                       [round(init_player[0], 1), round(init_player[1], 1)],
                       "vt_player": [round(px, 1), round(py, 1)],
                       "residual": round(init_player_res, 2),
                       "n_walls": init_full_walls, "samples": init_full_samples})
                print(f"  视觉初始角色 ({init_player[0]:.1f},{init_player[1]:.1f})  "
                      f"残差 {init_player_res:.1f}（全幅 {init_full_walls} 道墙，"
                      f"{init_full_samples} 样本）", file=sys.stderr)
                if init_player_res > 60.0:
                    event({"event": "init_rejected",
                           "reason": "residual>60", "residual": round(init_player_res, 1)})
                    print("  ⚠ 视觉初始位置残差 >60，按约定**不写 VT**（只记日志）",
                          file=sys.stderr)
            else:
                print(f"  ⚠ {args.init_timeout:g}s 内没拿到可用的全幅 VT 样本"
                      f"（{vt.err or 'camera stream?'}）", file=sys.stderr)

        # 切跟随视口（干净页面）
        status["phase"] = "init_follow"
        status["follow"] = True
        page_view.follow_player = True
        field_view.follow_player = True
        render_page(force=True)
        follow_switch_t = time.monotonic()
        vp = page_view.viewport
        if vp is not None:
            follow_half = 0.5 * vp.usable_h / max(1e-9, vp.scale)
        # 丢掉切视口之前（全幅）的样本：全幅 y=80..440 也在 [20,460] 里，会被误当
        # 跟随视口的墙，映射到 -100..260。
        fresh = vt.latest()
        if fresh is not None:
            last_vt_seq = fresh["seq"]
        print(f"  切跟随视口（半高 {follow_half:g} 场地单位），等待初始墙 ...",
              file=sys.stderr)
        good_follow: list[dict] = []
        last_valid: dict | None = None
        deadline = time.monotonic() + args.init_timeout
        while time.monotonic() < deadline:
            render_page()
            write_cam_samples(vt.drain())
            s = vt.latest()
            if (s is not None and s["seq"] > last_vt_seq
                    and s["t_wall"] >= follow_switch_t + args.cam_latency):
                last_vt_seq = s["seq"]
                if s["frame_geom"] and 1 <= len(s["walls"]) <= 6:
                    ys = _wall_ys(s["walls"])
                    if all(20.0 <= y <= 460.0 for y in ys):
                        good_follow.append(s)
                        last_valid = s
            if len(good_follow) >= 2 and _walls_close(
                    good_follow[-1]["walls"], good_follow[-2]["walls"], 8.0):
                break
            time.sleep(0.02)
        #: 有些场景（比如出生点上方只有 2 道墙 + 一条画框伪线）偶尔凑不出
        #: "两帧完全一致"的样本；这时退化为**用最近一帧有效样本**，别直接放弃。
        if len(good_follow) < 2 and last_valid is not None:
            good_follow = [last_valid]
        init_follow_samples = len(good_follow)
        if len(good_follow) >= 1:
            base = good_follow[-2:] if len(good_follow) >= 2 else good_follow
            # 同一道墙在两个样本里的 y 先排序配对再取均值
            ys_a = _wall_ys(base[0]["walls"])
            if len(base) >= 2:
                ys_b = _wall_ys(base[1]["walls"])
                ys = [0.5 * (a + b) for a, b in zip(ys_a, ys_b)]
            else:
                ys = list(ys_a)
            py_now = py
            #: 初始化也要按场地范围过滤：画框上下沿会被 VT 当成墙（y<0 或 >场高），
            #: 写进 known 会让后面的"新墙"判断和代价场一开始就错。
            known = sorted(y for y in (vt_world_y(vy, py_now) for vy in ys)
                           if 5.0 < y < field_h - 8.0)
            known_detail = []
            for k in known:
                # 取最接近该已知 y 的 VT 墙（同一个样本里的缺口端点）
                best = None
                bd = 1e9
                for s in base:
                    for w in s["walls"]:
                        gy = vt_world_y(float(w["y"]), py_now)
                        if abs(gy - k) < bd:
                            bd = abs(gy - k)
                            best = w
                known_detail.append({
                    "y": round(k, 1),
                    "gl": round(float(best.get("gl", 0)), 1) if best else 0.0,
                    "gr": round(float(best.get("gr", 640)), 1) if best else 640.0,
                    "vt_y": round(float(best["y"]), 1) if best else None,
                    "source": "camera_vt_init",
                })
            event({"event": "init_follow", "known": [round(k, 1) for k in known],
                   "detail": known_detail, "samples": init_follow_samples,
                   "half": round(follow_half, 2), "py": round(py, 1)})
            print(f"  初始已知 VT 墙（场地 y）：{[round(k, 1) for k in known]}",
                  file=sys.stderr)
        else:
            print(f"  ⚠ 跟随视口内没拿到稳定墙（{len(good_follow)} 样本）",
                  file=sys.stderr)
        if not known:
            print("⚠ 相机视觉初始化失败：看不到白框/墙。按约定**不伪造视觉**。\n"
                  "  排查：查看器窗口是否在屏幕上？板子相机 8099 是否在推流？\n"
                  "  可先用 `--vt-source none --send-all-walls` 只跑板子闭环。",
                  file=sys.stderr)
            server.close()
            if vt is not None:
                vt.stop()
            #: 初始化失败也要关掉查看器窗口，否则它会一直挂在屏幕同一个位置，
            #: 下一次运行的新窗口被它盖住 → 相机拍到旧页面 → VT 永远初始化失败。
            if viewer_proc is not None and viewer_proc.poll() is None:
                try:
                    viewer_proc.send_signal(2)
                    viewer_proc.wait(timeout=3.0)
                except Exception:                 # noqa: BLE001
                    pass
            for w in (follow_writer, field_writer, camera_writer):
                if w is not None:
                    w.release()
            env.close()
            events_f.close()
            return 3
    else:
        status["phase"] = "init_follow"
        status["follow"] = True
        page_view.follow_player = True
        field_view.follow_player = True
        render_page(force=True)
        vp = page_view.viewport
        if vp is not None:
            follow_half = 0.5 * vp.usable_h / max(1e-9, vp.scale)
        known = sorted({float(w[1]) for w in walls_of(env)})
        event({"event": "init_no_vision", "known": [round(k, 1) for k in known]})

    status["phase"] = "run"
    status["follow"] = True
    status["known"] = len(known)
    page_state["record"] = True

    # ---------------- 板子进程 -------------------------------------------
    if args.board == "local" and not Path(args.local_bin).exists():
        print(f"⚠ 本地板子二进制不存在：{args.local_bin}\n"
              f"   g++ -std=c++17 -O2 -o {args.local_bin} "
              f"core/board/tools/field_decision_cli.cpp", file=sys.stderr)
        server.close()
        if vt is not None:
            vt.stop()
        for w in (follow_writer, field_writer, camera_writer):
            if w is not None:
                w.release()
        env.close()
        events_f.close()
        return 2
    cmd = board_command(args, goal, field_w, field_h)
    print(f"  板子：{' '.join(cmd[:4])} ... {' '.join(cmd[-4:])}", file=sys.stderr)

    state_f = open(state_path, "w", encoding="utf-8")
    actions_f = open(actions_path, "w", encoding="utf-8")
    link = BoardLink(cmd, board_log, actions_f=actions_f)

    def wall_filter(y: float) -> bool:
        if args.send_all_walls or not known:
            return True
        return any(abs(y - k) <= args.wall_match_tol for k in known)

    state_seg_counts: list[int] = []
    state_vt_segs: list[int] = []

    def make_state(seq: int) -> str:
        """组装 @STATE。

        **默认（不给 --send-all-walls）：墙的几何也来自相机 VT** ——
        用 `known_detail` 里冻结那一帧量到的缺口 `gl/gr` 直接拼左右两段，
        y 用 VT 确认的墙行（不再是仿真真值）。没被视觉确认的墙一律不发，
        所以板子的决策信息只来自：视觉的墙 + 自身位姿（里程计）。
        `--send-all-walls` 保留旧的全真值行为（只用于对照实验）。
        """
        pl = env.world.player
        obs: list[str] = []
        if args.send_all_walls:
            for (x, y, hw, hh) in walls_of(env):
                if not wall_filter(y):
                    continue
                obs.append(f"{x:.1f} {y:.1f} {hw:.1f} {hh:.1f} 0.0")
        else:
            n_vt = 0
            for d in known_detail:
                if d.get("source") not in ("camera_vt_pause", "camera_vt_init"):
                    continue
                #: 已知墙的 y 在写入 known_detail 时已经过 vt_world_y 标定
                #: （尺度 + 偏置），这里**不能再加一次**，否则会偏两倍。
                y = float(d["y"])
                gl = float(d.get("gl", 0.0)); gr = float(d.get("gr", 640.0))
                if not (5.0 < y < field_h - 5.0) or gr - gl < 40.0:
                    continue
                #: 实测 VT 的缺口**两边都往内收**（左 +0~+12，右 −8~−37），所以
                #: 这里按 `--gap-margin` **向外放**（默认 10），把量窄的缺口补回来；
                #: 量得特别窄（<60）的行会把整行堵死（膨胀 13 后无格可过），
                #: 这时以测量中心向外补到 70 宽——真缺口 ~90，补完仍在真缺口内。
                gl = gl - args.gap_margin
                gr = gr + args.gap_margin
                if gr - gl < 60.0:
                    c = 0.5 * (gl + gr)
                    gl, gr = c - 35.0, c + 35.0
                gl = max(0.0, min(640.0, gl))
                gr = max(0.0, min(640.0, gr))
                if gr - gl < 50.0:
                    continue
                if gl > 4.0:
                    obs.append(f"{gl * 0.5:.1f} {y:.1f} {gl * 0.5:.1f} 4.0 0.0")
                if gr < 636.0:
                    obs.append(f"{(gr + 640.0) * 0.5:.1f} {y:.1f} "
                               f"{(640.0 - gr) * 0.5:.1f} 4.0 0.0")
                n_vt += 1
            state_vt_segs.append(n_vt)
        state_seg_counts.append(len(obs))
        return (f"@STATE {seq} {float(pl[0]):.1f} {float(pl[1]):.1f} "
                f"{float(pl[4]):.1f} {len(obs)} " + " ".join(obs) + "\n")

    # 板子 warmup：先发一帧，等第一个 @ACK/自检
    seq = 0
    line0 = make_state(seq)
    state_f.write(line0)
    state_f.flush()
    link.send(seq, line0)
    seq += 1
    warm_deadline = time.monotonic() + 12.0
    while time.monotonic() < warm_deadline and link.acks == 0:
        render_page()
        if vt is not None:
            write_cam_samples(vt.drain())
        if link.proc.poll() is not None and link.acks == 0:
            break
        time.sleep(0.05)
    if link.acks == 0:
        print(f"⚠ 板子 12 s 内没有 @ACK（见 {board_log}）；串口/ssh/二进制？",
              file=sys.stderr)
        link.close()
        state_f.close()
        actions_f.close()
        server.close()
        if vt is not None:
            vt.stop()
        for w in (follow_writer, field_writer, camera_writer):
            if w is not None:
                w.release()
        env.close()
        events_f.close()
        return 3
    print(f"  板子就绪：@ACK {link.acks} 条，@MOVE {link.moves} 条", file=sys.stderr)
    for ln in link.selfcheck:
        print(f"  板子自检 {ln}", file=sys.stderr)
    # 板子 ready 之后才开 pacing；否则初始化耗时会变成"仿真瞬间追帧"。
    run_start = time.monotonic()

    # ---------------- 主循环 ---------------------------------------------
    collisions = 0
    min_clear: float | None = None
    min_clear_late: float | None = None
    switches = 0
    prev_act = 0
    reached = False
    paused_total = 0.0
    pause_count = 0
    new_wall_events = 0
    error: str | None = None
    exit_code = 0
    last_keepalive = 0.0
    last_status = 0.0

    def pause_fail(cand_gy: float) -> None:
        """暂停没换到新墙：同一候选冷却 4 s（墙集合不变，继续走）。"""
        pause["cooldown_until"] = time.monotonic() + 4.0
        pause["skip_gy"] = float(cand_gy)
        pause["cooldown_gaps"] = []

    def pause_begin(sample: dict, cand_gy: float, cand_w: dict) -> None:
        nonlocal pause_count
        pause_count += 1
        pause["active"] = True
        pause["t0"] = time.monotonic()
        pause["target_gy"] = float(cand_gy)
        pause["target_vy"] = float(cand_w["y"])
        pause["target_gl"] = float(cand_w.get("gl", 0.0))
        pause["target_gr"] = float(cand_w.get("gr", 640.0))
        pause["known_before"] = [round(k, 1) for k in known]
        pause["samples"] = []
        pause["events"] = []
        pause["last_seq"] = sample["seq"]
        info = {
            "event": "pause", "index": pause_count,
            "t_sim": round(t_sim, 3), "step": steps,
            "player": [round(px, 1), round(py, 1)],
            "known_before": [round(k, 1) for k in known],
            "vt_seq": sample["seq"], "vt_n": sample["n"],
            "vt_walls": sample["walls"],
            "wall_view_y": round(float(cand_w["y"]), 1),
            "wall_global_est": round(cand_gy, 1),
        }
        event(info)
        print(f"  ⏸ t={t_sim:.2f}s 发现新墙：view_y={cand_w['y']:.1f} "
              f"→ 场地 y≈{cand_gy:.1f}；暂停等相机 VT 稳定（已知 "
              f"{[round(k,1) for k in known]}）", file=sys.stderr)

    def pause_update() -> bool:
        """暂停中：把**这一屏所有稳定的新墙一次加完**再恢复（批量，不逐道停）。

        稳定性按"每道墙自己在相邻两帧里的世界 y"判，不再对齐某一道 target：
        相机链路有 ~1 s 延迟，暂停瞬间拿到的那一帧其实是"角色还在 ~200 单位前"
        时的画面，target 映射必然对不上（实测 target=556，确认出来是 748，
        然后立刻重暂停 → 8 ms 一次的活锁）。暂停后页面已经冻结，**用当前 py 映射**
        反而一致；两帧都在冻结之后，同一道墙的 gy 自然稳定。
        """
        nonlocal known, known_detail, paused_total, new_wall_events
        s = vt.latest() if vt is not None else None
        if s is None:
            return False
        now = time.monotonic()
        #: 只收暂停后的稳定帧：晚 0.9 s 才开始（板子相机 + MJPEG 打包有延迟）。
        if s["seq"] <= pause.get("last_seq", -1):
            return False
        if s["t_wall"] < pause["t0"] + 0.9:
            return False
        pause["last_seq"] = s["seq"]
        timed_out = (now - pause["t0"]) > args.pause_timeout
        if not s["frame_geom"]:
            return False
        py_now = py
        #: 只信画面**中段**的墙：画框上下沿的墙会被裁掉一截，缺口量出来常常
        #: 宽得离谱（离线测试里出现过 230–593）。60~420 之外的行一律不收。
        walls = [w for w in s["walls"] if 60.0 <= float(w["y"]) <= 480.0]
        dets_vy = [float(w["y"]) for w in walls]
        _delta, matched = _shift_match(dets_vy, known, py_now, follow_half)
        max_known = max(known) if known else -1e9
        extras: list[tuple[float, dict]] = []
        for j, w in enumerate(walls):
            gy = vt_world_y(float(w["y"]), py_now)
            if not (5.0 < gy < field_h - 8.0):
                continue
            gl = float(w.get("gl", 0.0)); gr = float(w.get("gr", 640.0))
            #: 同一道墙被旧帧再看一次才会缺口完全一样；缺口每 4 道墙重复，
            #: 所以必须同时要求 y 接近同一道已知墙。
            if any(abs(float(d.get("gl", 0.0)) - gl) <= 30.0
                   and abs(float(d.get("gr", 640.0)) - gr) <= 30.0
                   and abs(gy - float(d.get("y", -1e9))) <= 120.0
                   for d in known_detail):
                continue
            if gy <= max_known + args.new_wall_margin:
                continue
            #: **可信度检查**（本轮新增）：
            #: 1) 缺口宽度：真缺口 90~110，量到 <75 基本是量歪了（实测有一行
            #:    量成 65 宽、中心偏 300，直接把角色送进墙里）；
            #: 2) 交替场景：新行缺口应与上一行**不同侧**（左/右），同侧就是量错。
            #: 不可信的行先不收，等下一次暂停换一帧再量。
            if gr - gl < args.gap_min_width:
                event({"event": "gap_reject", "t_sim": round(t_sim, 3),
                       "y": round(gy, 1), "gl": round(gl, 1),
                       "gr": round(gr, 1), "why": "width"})
                continue
            if args.expect_alternate and known_detail:
                prev_d = None
                for d in known_detail:
                    if not str(d.get("source", "")).startswith("camera_vt"):
                        continue
                    if prev_d is None or float(d["y"]) > float(prev_d["y"]):
                        prev_d = d
                if prev_d is not None:
                    c_new = 0.5 * (gl + gr)
                    c_prev = 0.5 * (float(prev_d.get("gl", 0.0))
                                    + float(prev_d.get("gr", 640.0)))
                    if (c_new < 320.0) == (c_prev < 320.0):
                        event({"event": "gap_reject", "t_sim": round(t_sim, 3),
                               "y": round(gy, 1), "gl": round(gl, 1),
                               "gr": round(gr, 1), "why": "side",
                               "prev_y": round(float(prev_d["y"]), 1)})
                        continue
            extras.append((gy, w))
        #: 与上一帧的候选逐道比对：y 差 ≤20 **且** 缺口左右边界差 ≤15，
        #: 连续两帧都一致才算稳定（只比 y 会把"同一行但缺口量歪"的帧放进来）。
        prev = pause.get("prev_extras") or []
        def _same(c, q) -> bool:
            gy, w_x = c
            return (abs(gy - q[0]) <= 20.0
                    and abs(float(w_x.get("gl", 0.0)) - q[1]) <= 15.0
                    and abs(float(w_x.get("gr", 640.0)) - q[2]) <= 15.0)
        stable = [c for c in extras if any(_same(c, q) for q in prev)]
        pause["prev_extras"] = [
            (gy, float(w_x.get("gl", 0.0)), float(w_x.get("gr", 640.0)))
            for gy, w_x in extras]
        if not extras:
            #: 这一屏的墙**都已经知道了**（触发它的是刚加过的墙/重复候选）：
            #: 立刻恢复，不等 12 s 超时；按缺口冷却 2 s，别马上再停。
            pause["cooldown_until"] = now + 2.0
            pause["cooldown_gaps"] = (pause["cooldown_gaps"] + [
                (float(pause["target_gl"]), float(pause["target_gr"]))])[-12:]
            paused_total += now - pause["t0"]
            pause["active"] = False
            pause["prev_extras"] = []
            event({"event": "resume_known", "index": pause_count,
                   "t_sim": round(t_sim, 3),
                   "target_gy": round(pause["target_gy"], 1)})
            print(f"  ▶ t={t_sim:.2f}s 恢复：这一屏的墙都已知，立即继续"
                  f"（暂停 {now - pause['t0']:.1f}s）", file=sys.stderr)
            return True
        if stable:
            added = []
            for gy_b, w_b in stable:
                if any(abs(gy_b - k) <= args.wall_match_tol for k in known):
                    continue
                known.append(gy_b)
                known_detail.append({
                    "y": round(gy_b, 1),
                    "gl": round(float(w_b.get("gl", 0.0)), 1),
                    "gr": round(float(w_b.get("gr", 640.0)), 1),
                    "vt_y": round(float(w_b["y"]), 1),
                    "source": "camera_vt_pause"})
                added.append(gy_b)
                new_wall_events += 1
            known = sorted(known)
            info = {
                "event": "resume", "index": pause_count,
                "t_sim": round(t_sim, 3), "step": steps,
                "player": [round(px, 1), round(py, 1)],
                "wall_view_y": round(pause["target_vy"], 1),
                "wall_global_est": round(pause["target_gy"], 1),
                "known_before": pause.get("known_before", []),
                "added": [round(y, 1) for y in added],
                "stable": True, "timeout": bool(timed_out),
                "samples": len(pause.get("prev_extras") or []),
                "pause_wall_s": round(now - pause["t0"], 2),
                "known_after": [round(k, 1) for k in known],
            }
            event(info)
            pause["events"].append(info)
            all_pauses.append(info)
            #: **本屏还没收完就继续等**：只要这一屏（60~460）里还有没匹配到
            #: known 的墙，就继续暂停收集，避免"加了一道就恢复"导致上面那行
            #: 在角色越过前还没进 @STATE（实测 1900 行晚 32、2040 行晚 112）。
            remain = 0
            for gy_r, _w_r in extras:
                if any(abs(gy_r - k) <= args.wall_match_tol for k in known):
                    continue
                remain += 1
            if remain and not timed_out:
                pause["prev_extras"] = []
                return False
            pause["last_resume"] = now
            pause["last_new_y"] = added[-1] if added else None
            #: 刚加入的墙还在"前方 160"里，按缺口冷却 2 s 避免立刻再停。
            pause["cooldown_until"] = now + 2.0
            pause["cooldown_gaps"] = (pause["cooldown_gaps"] + [
                (float(w_b.get("gl", 0.0)), float(w_b.get("gr", 640.0)))
                for _gy, w_b in stable])[-12:]
            print(f"  ▶ t={t_sim:.2f}s 恢复：VT 批量加入 {[round(y,1) for y in added]}"
                  f"（pause={now - pause['t0']:.1f}s）→ "
                  f"{[round(k,1) for k in known]}", file=sys.stderr)
            paused_total += now - pause["t0"]
            pause["active"] = False
            pause["prev_extras"] = []
            return True
        if timed_out:
            ev = {"event": "resume_no_wall", "index": pause_count,
                  "t_sim": round(t_sim, 3), "timeout": True,
                  "samples": len(pause.get("prev_extras") or []),
                  "target_gy": round(pause["target_gy"], 1)}
            event(ev)
            all_pauses.append(ev)
            print(f"  ▶ t={t_sim:.2f}s 恢复：暂停超时且没拿到稳定的新墙，墙集合不变",
                  file=sys.stderr)
            pause_fail(pause["target_gy"])
            paused_total += now - pause["t0"]
            pause["active"] = False
            pause["prev_extras"] = []
            return True
        return False


    try:
        while t_sim < args.max_seconds - 1e-12:
            now = time.monotonic()
            if link.eof and link.acks == 0:
                error = f"板子输出流关闭（见 {board_log}）"
                exit_code = 3
                break
            if link.proc.poll() is not None and not link.eof:
                error = f"板子进程已退出（rc={link.proc.returncode}，见 {board_log}）"
                exit_code = 3
                break
            if (link.acks > 0 and link.last_ack is not None
                    and now - link.last_ack > args.stall_s):
                error = f"板子 {args.stall_s:g}s 没回 @ACK，判失联"
                exit_code = 3
                break

            try:
                if not pause["active"]:
                    if steps % state_every == 0:
                        line = make_state(seq)
                        state_f.write(line)
                        state_f.flush()
                        link.send(seq, line)
                        seq += 1
                    act = int(link.act)
                    status["act"] = act
                    if act != prev_act:
                        switches += 1
                        prev_act = act
                    _obs, _rew, _term, _trunc, info = env.step(act)
                    collisions += int(info.get("collision_events", 0) or 0)
                    px, py = float(env.world.player[0]), float(env.world.player[1])
                    note_py(py)
                    for (cx, cy, hw, hh) in walls_of(env):
                        d = dist_rect(px, py, cx, cy, hw, hh) - float(spec.player_radius)
                        min_clear = d if min_clear is None else min(min_clear, d)
                        if t_sim >= 1.0:
                            min_clear_late = (d if min_clear_late is None
                                              else min(min_clear_late, d))
                    t_sim += dt
                    steps += 1
                    status["known"] = len(known)
                    if vt is not None:
                        s = vt.latest()
                        status["vt_n"] = len(s["walls"]) if s else 0
                        if (s is not None and s["seq"] > last_vt_seq
                                and s["frame_geom"] and not pause["active"]):
                            last_vt_seq = s["seq"]
                            py_cap = py_at(s["t_wall"] - args.cam_latency)
                            walls = [w for w in s["walls"]
                                     if 20.0 <= float(w["y"]) <= 456.0]
                            max_known = max(known) if known else -1e9
                            #: **近距离复核**：VT 偶尔会把某一行量歪（实测 1900 行量成
                            #: 102–167，真值 396–502，偏了 300），之前只靠"两帧一致"
                            #: 挡不住。当已确认的墙进到 100 单位以内、且连续两帧量到的
                            #: 缺口都和已存的差 >40 时，用新测量**纠正**这一行。
                            gap_fix = pause.setdefault("gap_fix", {})
                            for w in walls:
                                gy_w = vt_world_y(float(w["y"]), py)
                                ahead_w = gy_w - py
                                if not (20.0 < ahead_w < 100.0):
                                    continue
                                gl_w = float(w.get("gl", 0.0))
                                gr_w = float(w.get("gr", 640.0))
                                for d in known_detail:
                                    if abs(gy_w - float(d["y"])) > 30.0:
                                        continue
                                    if (abs(gl_w - float(d.get("gl", 0.0))) <= 40.0
                                            and abs(gr_w - float(d.get("gr", 640.0))) <= 40.0):
                                        gap_fix.pop(id(d), None)
                                        break
                                    prev_fix = gap_fix.get(id(d))
                                    if (prev_fix is not None
                                            and abs(prev_fix[0] - gl_w) <= 15.0
                                            and abs(prev_fix[1] - gr_w) <= 15.0):
                                        d["gl"] = round(gl_w, 1)
                                        d["gr"] = round(gr_w, 1)
                                        d["source"] = "camera_vt_fix"
                                        event({"event": "gap_fix",
                                               "y": round(float(d["y"]), 1),
                                               "old": [round(prev_fix[2], 1),
                                                       round(prev_fix[3], 1)],
                                               "new": [round(gl_w, 1), round(gr_w, 1)]})
                                        gap_fix.pop(id(d), None)
                                    else:
                                        gap_fix[id(d)] = (gl_w, gr_w,
                                                          float(d.get("gl", 0.0)),
                                                          float(d.get("gr", 640.0)))
                                    break
                            dets_vy = [float(w["y"]) for w in walls]
                            # 公共平移对齐：已知墙整体偏 δ 不是新墙
                            delta, matched = _shift_match(dets_vy, known,
                                                          py_cap, follow_half)
                            cands = []
                            for j, w in enumerate(walls):
                                #: **不再用 matched 跳过**：δ 估偏时会把"新墙"误判成
                                #: 已知墙，于是不停 → 板子不知道它 → 直接撞上去
                                #: （实测 y=520 那道墙就是这么穿过去的）。已知墙由
                                #: 暂停后的冻结画面判（extras 为空 → 立即继续）。
                                vy = float(w["y"])
                                #: **尽量往前走**：按"这一帧里墙在角色前方多少"判
                                #: （`vy - 半带`，与相机延迟无关），进到 `--pause-ahead`
                                #: 以内才停；0 = 不限。
                                #: 换算成**场地单位**（视口缩放<1 时像素≠场地单位）
                                ahead = (vy - 240.0) * (2.0 * follow_half / 480.0)
                                if args.pause_ahead > 0 and ahead > args.pause_ahead:
                                    continue
                                if ahead < 15.0:
                                    continue
                                #: 与 pause_update 一致：只信画面中段的行，
                                #: 画框上下沿的墙缺口量不准，不为它暂停。
                                if not (60.0 <= vy <= 480.0):
                                    continue
                                #: 这里**不再用** stale 的 py_cap 算出来的 gy 去比
                                #: `max_known`：相机链路 ~1 s 延迟会让 gy 偏低 ~200，
                                #: 曾经因此漏掉终点前最后一道墙（板子靠 send-all-walls
                                #: 直接开过去）。"是不是新墙"交给暂停后冻结页面的
                                #: pause_update 判，那里 py 是准的。
                                gy = vt_world_y(vy, py)
                                # 场地外的"墙"是帧线/反光，不是墙
                                if not (5.0 < gy < field_h - 8.0):
                                    continue
                                #: **只对未确认的行暂停**：映射用当前 py（标定后±4），
                                #: 不再用 stale 的 py_cap；否则已知行会一直是最"近"的
                                #: 候选 → 反复暂停又"全部已知"，把真正的新行饿死
                                #: （实测 53 次暂停、最后一行只提前 4.5 单位）。
                                if any(abs(gy - k) <= 30.0 for k in known):
                                    continue
                                # 目标环正好在 (·,470)：只挡目标上的假墙；
                                # y=440 的最后一道墙离目标只有 30，不能被挡。
                                if abs(gy - goal[1]) < 12.0:
                                    continue
                                gl = float(w.get("gl", 0.0)); gr = float(w.get("gr", 640.0))
                                #: 刚加入的墙（刚批量确认过）在冷却期里不再触发；
                                #: 但**已经贴到 80 单位以内**时不再看冷却——宁可多停
                                #: 一次，也不能让角色越过还没进 @STATE 的行。
                                if (ahead > 80.0 and now < pause["cooldown_until"]
                                        and any(abs(g[0] - gl) <= 30.0
                                                and abs(g[1] - gr) <= 30.0
                                                for g in pause["cooldown_gaps"])):
                                    continue
                                cands.append((gy, w))
                            if now < pause["cooldown_until"]:
                                cands = [c for c in cands
                                         if abs(c[0] - pause["skip_gy"]) > 100.0]
                            if cands:
                                #: **选最近的前方行**（不是最远的）：一次暂停先服务
                                #: 下一行，保证它在角色越过前被确认；远处的行下次再说。
                                cand_gy, cand_w = min(cands, key=lambda c: c[0])
                                pause_begin(s, cand_gy, cand_w)
                else:
                    if now - last_keepalive >= 0.10:
                        line = make_state(seq)
                        state_f.write(line)
                        state_f.flush()
                        link.send(seq, line)
                        seq += 1
                        last_keepalive = now
                    pause_update()
            except (BrokenPipeError, OSError) as exc:
                error = f"写板子 stdin 失败：{exc!r}"
                exit_code = 3
                break

            # 动作流：reader 已经把 @MOVE 写进 actions_f
            render_page()
            if vt is not None:
                write_cam_samples(vt.drain())

            if now - last_status >= 2.0:
                last_status = now
                with link.lock:
                    acks, moves = link.acks, link.moves
                print(f"    t={t_sim:6.2f}s pos=({px:.0f},{py:.0f}) "
                      f"act={link.act} known={len(known)} "
                      f"vt={status['vt_n']} acks={acks} moves={moves} "
                      f"{'PAUSED' if pause['active'] else ''}", file=sys.stderr)

            if math.hypot(px - goal[0], py - goal[1]) <= goal_r:
                reached = True
                render_page(force=True)
                if vt is not None:
                    write_cam_samples(vt.drain())
                break

            if args.speed > 0:
                due = run_start + t_sim / float(args.speed) + paused_total
                lag = due - time.monotonic()
                if lag > 0:
                    time.sleep(min(0.2, lag))
                elif pause["active"]:
                    time.sleep(0.005)
            elif pause["active"]:
                time.sleep(0.005)
    except KeyboardInterrupt:
        error = "Ctrl-C"
        exit_code = 130
    finally:
        pass

    # ---------------- 收尾 -----------------------------------------------
    wall_s = time.monotonic() - run_start
    with link.lock:
        lat = list(link.lat)
        acks = link.acks
        moves = link.moves
        selfcheck = list(link.selfcheck)
    actions_f.close()
    state_f.close()
    link.close()
    if vt is not None:
        vt.stop()
    last_render = page_view.render(env.world)
    server.publish(last_render if last_render is not None else base_frame)
    # 最后一帧存图（给报告）
    try:
        raw = server.jpeg()
        if raw:
            (run_dir / "follow_last.jpg").write_bytes(raw)
        if camera_writer is not None and vt is not None:
            s = vt.latest()
            if s is not None and s.get("cam_bgr") is not None:
                cv2.imwrite(str(run_dir / "camera_last.png"), s["cam_bgr"])
    except Exception:                                 # noqa: BLE001
        pass
    for w in (follow_writer, field_writer, camera_writer):
        if w is not None:
            try:
                w.release()
            except Exception:                         # noqa: BLE001
                pass

    clearance = None if min_clear is None else round(min_clear, 2)
    clearance_late = None if min_clear_late is None else round(min_clear_late, 2)
    vids = {"follow": _validate_video(follow_mp4),
            "field": _validate_video(field_mp4),
            "camera_vt": _validate_video(camera_mp4)}
    pause_list = list(all_pauses)
    summary = {
        "scenario": spec.name,
        "goal": list(goal),
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
        "send_all_walls": bool(args.send_all_walls),
        "state_wall_segments": {
            "first": state_seg_counts[0] if state_seg_counts else None,
            "min": min(state_seg_counts) if state_seg_counts else None,
            "max": max(state_seg_counts) if state_seg_counts else None,
        },
        "pause_count": int(pause_count),
        "pauses": pause_list,
        "new_walls_discovered": int(new_wall_events),
        "known_wall_count": int(len(known)),
        "known_walls": [
            ({"y": round(k, 1), **{kk: vv for kk, vv in
                                   min(known_detail, key=lambda d: abs(d["y"] - k)).items()
                                   if kk != "y"}}
             if known_detail else {"y": round(k, 1)})
            for k in known
        ],
        "vision": {
            "source": args.vt_source,
            "camera_url": args.camera_url if args.vt_source == "camera" else None,
            "vt_hz": args.vt_hz,
            "init_player": None if init_player is None
            else [round(init_player[0], 1), round(init_player[1], 1)],
            "init_player_residual": None if init_player_res is None
            else round(init_player_res, 2),
            "init_full_walls": init_full_walls,
            "init_full_samples": init_full_samples,
            "init_follow_samples": init_follow_samples,
            "follow_half": round(follow_half, 2),
            "player_residual": {
                "n": residual["n"],
                "median": None if not residual["values"]
                else round(_median(residual["values"]), 2),
                "p95": None if not residual["values"]
                else round(pct(residual["values"], 0.95), 2),
                "n_reject_gate": residual["n_reject"],
                "n_reinit": residual["n_reinit"],
            },
            "wall_residual": {
                "n": len(residual["wall_res"]),
                "median": None if not residual["wall_res"]
                else round(_median(residual["wall_res"]), 2),
            },
            "vt_error": vt.err if vt is not None else "",
        },
        "vt_walls_sent": {"min": min(state_vt_segs) if state_vt_segs else None,
                          "max": max(state_vt_segs) if state_vt_segs else None},
        "page": {
            "render_hz": float(args.render_hz),
            "published": int(publisher.published),
            "dropped": int(publisher.dropped),
            "published_fps": (round(publisher.published / wall_s, 2)
                              if wall_s > 0 else None),
        },
        "files": {
            "state": str(state_path),
            "actions": str(actions_path),
            "board_log": str(board_log),
            "events": str(events_path),
            "vt_samples": str(run_dir / "vt_samples.jsonl"),
            "follow_mp4": str(follow_mp4),
            "field_mp4": str(field_mp4),
            "camera_vt_mp4": str(camera_mp4),
            "summary": str(summary_path),
            "run_dir": str(run_dir),
        },
        "videos": vids,
        "error": error,
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
    events_f.close()
    env.close()

    print(f"场景 {spec.name}  目标 {goal} 半径 {goal_r:g}  已知墙 {len(known)} 道")
    print(f"  页面 {args.render_hz:g} fps 目标 → 实际推送 {publisher.published} 帧"
          f"（{publisher.published / wall_s:.1f} fps，丢旧帧 {publisher.dropped}）"
          f"  主循环不背 JPEG")
    print(f"  到达 {'是' if reached else '否'}   仿真 {t_sim:5.2f} s   墙钟 "
          f"{wall_s:5.2f} s ({summary['rtf']}×)   终点 ({px:.0f},{py:.0f})")
    print(f"  仿真碰撞事件 {collisions} 次   最小间隙 {clearance}"
          f"（1 s 后 {clearance_late}）   动作切换 {switches}")
    print(f"  暂停 {pause_count} 次   新墙 {new_wall_events} 道   已知 "
          f"{[round(k, 1) for k in known]}")
    if lat:
        print(f"  端到端延迟 p50 {pct(lat, 0.5) * 1e3:.1f} ms   "
              f"p95 {pct(lat, 0.95) * 1e3:.1f} ms   max {max(lat) * 1e3:.1f} ms")
    for ln in selfcheck:
        print(f"  板子自检 {ln}")
    for p in pause_list:
        if "new_wall" not in p:
            print(f"  暂停证据 #{p.get('index')}: t={p.get('t_sim', 0):.2f}s "
                  f"恢复但无新墙（timeout={p.get('timeout')}）")
            continue
        print(f"  暂停证据 #{p['index']}: 暂停 t={p['t_sim']:.2f}s "
              f"墙 view_y={p.get('wall_view_y', float('nan')):.1f} → "
              f"场地 y={p['new_wall']['y']:.1f}; "
              f"稳定={p['stable']} spread={p['vy_spread']:.1f}; "
              f"暂停 {p['pause_wall_s']:.1f}s 后恢复")
    print(f"  视频 {follow_mp4}")
    print(f"       {field_mp4}")
    print(f"       {camera_mp4}")
    for k, v in vids.items():
        print(f"    {k}: {v.get('frames', 0)} 帧 {v.get('size')} "
              f"{v.get('bytes', 0)} B")
    print(f"  汇总 {summary_path}")
    if error:
        print(f"  ⚠ {error}")
    criteria_ok = (reached and summary["clearance_ok"] and switches < 30
                   and collisions == 0 and not summary["geometric_collision"]
                   and (pause_count >= 1 or args.vt_source != "camera"))
    if not criteria_ok and exit_code == 0:
        exit_code = 1

    # 关查看器（恢复桌面）；8100 页面继续可访问
    if viewer_proc is not None:
        try:
            if args.viewer_hold > 0:
                time.sleep(args.viewer_hold)
        except KeyboardInterrupt:
            pass
        try:
            viewer_proc.send_signal(2)                # SIGINT
            viewer_proc.wait(timeout=3.0)
        except Exception:                             # noqa: BLE001
            try:
                viewer_proc.kill()
            except Exception:                         # noqa: BLE001
                pass
    if viewer_log is not None:
        try:
            viewer_log.close()
        except Exception:                             # noqa: BLE001
            pass

    try:
        publisher.stop()                 # 页面最后仍由 server 的最后一帧服务
    except Exception:                    # noqa: BLE001
        pass
    if args.keep_serving:
        print(f"  页面保持可访问：http://127.0.0.1:{args.port}/  （Ctrl-C 结束）",
              file=sys.stderr, flush=True)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    server.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
