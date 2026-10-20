#!/usr/bin/env python3
"""Headless 平台 → 浏览器 MJPEG 页。

为什么不用 pygame 窗口：WSLg 下窗口置顶不可靠（X11 raise 甚至会把进程搞退）。
这里让 pybullet 以 `rgb_array` 离屏渲染，帧直接送浏览器；浏览器窗口就是"弹窗"，
相机拍浏览器页即可，不再依赖 WSLg 窗口焦点。

用法（本机）：
    python3 core/board/tools/sim_stream.py --scenario configs/static_wall_wide.json \
        --port 8098 --w 1280 --h 960 --no-fiducials

浏览器打开 http://127.0.0.1:8098/ ，点页面上的"打开平台窗口"可弹出一个独立窗口。
"""
from __future__ import annotations

import argparse
import io
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from bullet_sim.render.pygame_view import PygameRenderer  # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec          # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv          # noqa: E402

from PIL import Image  # noqa: E402

_lock = threading.Condition()
_frame = [b""]
_seq = [0]
_stop = threading.Event()


def _jpeg(frame) -> bytes:
    img = Image.fromarray(frame)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _capture(spec_path: str, w: int, h: int, no_fiducials: bool, wall_color: str,
             interval: float) -> None:
    spec = ScenarioSpec.load(spec_path)
    env = BulletHellEnv(spec, seed=spec.seed)
    env.reset(seed=spec.seed)
    view = PygameRenderer(
        mode="rgb_array",
        window_size=(w, h),
        show_hud=False,
        show_text=False,
        show_fiducials=not no_fiducials,
        show_world_axes=False,
        show_field_border=True,
        show_hitboxes=False,
        wall_color=tuple(int(wall_color[i:i + 2], 16) for i in (0, 2, 4)),
        background=(8, 10, 14),
        field_bg=(34, 40, 55),
    )
    try:
        while not _stop.is_set():
            frame = view.render(env.world)
            if frame is not None:
                jpg = _jpeg(frame)
                with _lock:
                    _frame[0] = jpg
                    _seq[0] += 1
                    _lock.notify_all()
            time.sleep(max(0.02, interval))
    finally:
        env.close()


_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>platform %(w)dx%(h)d</title>
<style>
html,body{margin:0;padding:0;background:#050608;color:#ddd;font-family:system-ui,sans-serif}
#bar{padding:8px 12px;background:#11161f;font-size:14px}
button{background:#2d6cdf;color:#fff;border:0;border-radius:4px;padding:6px 10px;cursor:pointer}
img{display:block;width:100vw;height:100vh;object-fit:contain;background:#000}
</style></head><body>
<div id="bar">平台 headless 渲染 · %(w)dx%(h)d · 相机拍这个页面即可
<button onclick="openWin()">打开平台窗口</button></div>
<img src="/stream.mjpg?ts=%(ts)d">
<script>
function openWin(){
  const w=%(w)d,h=%(h)d+40;
  window.open('/view','platform','width='+w+',height='+h+',menubar=no,toolbar=no,location=no,status=no');
}
</script>
</body></html>
"""

_VIEW = """<!doctype html>
<html><head><meta charset="utf-8"><title>platform</title>
<style>html,body{margin:0;background:#000}img{display:block;width:auto;height:auto;width:100vw;height:100vh;object-fit:contain;margin:0}</style>
</head><body><img src="/stream.mjpg?ts=%(ts)d"></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        u = self.path.split("?")[0]
        if u in ("/", "/view"):
            w, h = self.server.stream_size
            body = ((_PAGE if u == "/" else _VIEW) % {"w": w, "h": h, "ts": int(time.time())}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if u == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            try:
                while True:
                    with _lock:
                        while _seq[0] == last:
                            _lock.wait(1.0)
                        last = _seq[0]
                        jpg = _frame[0]
                    if not jpg:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            return
        self.send_error(404)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="configs/static_wall_wide.json")
    ap.add_argument("--port", type=int, default=8098)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=960)
    ap.add_argument("--no-fiducials", action="store_true")
    ap.add_argument("--wall-color", default="C83C00")
    ap.add_argument("--fps", type=float, default=15.0)
    args = ap.parse_args()
    threading.Thread(target=_capture, args=(args.scenario, args.w, args.h,
                                             args.no_fiducials, args.wall_color,
                                             1.0 / max(1.0, args.fps)), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    srv.daemon_threads = True
    srv.stream_size = (args.w, args.h)
    print("serving http://127.0.0.1:%d/  %dx%d" % (args.port, args.w, args.h), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
