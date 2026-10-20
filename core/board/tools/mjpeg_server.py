#!/usr/bin/env python3
"""Board-side MJPEG preview: /dev/video0 -> http://<board>:8099/

为什么需要它：PC 上的相机预览窗口和板子拿到的 UVC 画面可能不是同一个
crop（这颗相机只有 1280x720 / 640x480 两种 MJPG 模式；720p 是 16:9，
640x480 是 4:3）。这里直接把**板子实际拿到的帧**用浏览器显示出来，
用来判断"右上角 marker 到底在不在板子的视野里"。

只依赖 ffmpeg + python3 标准库。用法（板上）：

    python3 mjpeg_server.py [port] [w] [h]

浏览器打开 http://<board-ip>:8099/ ；页面上的链接可以在 640x480 / 1280x720
之间切换（同一时刻只有一个 ffmpeg 打开 /dev/video0）。
"""
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8099
MODES = [(640, 480), (1280, 720)]
mode = [MODES[0][0], MODES[0][1]]
lock = threading.Condition()
frame = [None]
seq = [0]
proc = [None]
stop = threading.Event()


def ffmpeg_cmd(w, h):
    return [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", "%dx%d" % (w, h), "-i", "/dev/video0",
        "-c:v", "copy", "-f", "mjpeg", "-",
    ]


def capture_loop():
    """Keep ffmpeg running; split the MJPEG stream into individual JPEGs."""
    while not stop.is_set():
        w, h = mode
        try:
            p = subprocess.Popen(ffmpeg_cmd(w, h), stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001
            print("spawn ffmpeg failed: %r" % (exc,), flush=True)
            time.sleep(1.0)
            continue
        proc[0] = p
        print("capture started %dx%d pid=%d" % (w, h, p.pid), flush=True)
        buf = b""
        try:
            while not stop.is_set():
                # Mode switch: kill the current ffmpeg, outer loop restarts it.
                if (mode[0], mode[1]) != (w, h):
                    p.kill()
                    break
                chunk = p.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    i = buf.find(b"\xff\xd8")
                    if i < 0:
                        if len(buf) > 4:
                            buf = buf[-4:]
                        break
                    j = buf.find(b"\xff\xd9", i + 2)
                    if j < 0:
                        if i > 0:
                            buf = buf[i:]
                        break
                    jpg = buf[i:j + 2]
                    buf = buf[j + 2:]
                    with lock:
                        frame[0] = jpg
                        seq[0] += 1
                        lock.notify_all()
        finally:
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass
            try:
                p.wait(timeout=2)
            except Exception:  # noqa: BLE001
                pass
            if proc[0] is p:
                proc[0] = None
        if not stop.is_set():
            time.sleep(0.2)


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>board camera %(w)dx%(h)d</title>
<style>body{background:#111;color:#eee;font-family:system-ui,sans-serif;margin:0;padding:16px}
a{color:#6cf} .bar{margin:8px 0 14px} img{width:100%%;max-width:%(w)dpx;background:#000;border:1px solid #444}</style>
</head><body>
<h2>板子相机 /dev/video0 — %(w)dx%(h)d</h2>
<div class="bar">
模式：
<a href="/switch?w=640&h=480">640x480 (4:3)</a> ·
<a href="/switch?w=1280&h=720">1280x720 (16:9)</a>
&nbsp; | &nbsp; 当前：<b>%(w)dx%(h)d</b>
</div>
<img src="/stream.mjpg?ts=%(ts)d">
<p style="color:#888">这是板子 ffmpeg 直接从 /dev/video0 取的 MJPEG 帧；不是 PC 预览窗口的画面。
如果右上角 marker 在这里被切掉，而 PC 预览里有，说明两个模式/裁切不同。</p>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # silence per-frame logs
        pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            w, h = mode
            body = (PAGE % {"w": w, "h": h, "ts": int(time.time())}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if u.path == "/switch":
            q = parse_qs(u.query)
            try:
                w = int(q.get("w", ["640"])[0])
                h = int(q.get("h", ["480"])[0])
            except ValueError:
                w, h = 640, 480
            if (w, h) not in MODES:
                w, h = 640, 480
            with lock:
                mode[0], mode[1] = w, h
                lock.notify_all()
            p = proc[0]
            if p is not None:
                try:
                    p.kill()
                except Exception:  # noqa: BLE001
                    pass
            self.send_response(302)
            self.send_header("Location", "/")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if u.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            try:
                while True:
                    with lock:
                        while seq[0] == last:
                            lock.wait(1.0)
                        last = seq[0]
                        jpg = frame[0]
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    # Cap at 15 fps even if the camera runs 60.
                    time.sleep(1.0 / 15.0)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            return
        self.send_error(404)


def main():
    threading.Thread(target=capture_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    print("serving http://0.0.0.0:%d/ mode=%dx%d" % (PORT, mode[0], mode[1]),
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        p = proc[0]
        if p is not None:
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
