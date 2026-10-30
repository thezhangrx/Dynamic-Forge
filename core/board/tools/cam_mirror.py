#!/usr/bin/env python3
"""cam_mirror.py —— 板载相机画面"镜像"（**不碰相机**，只转发管线写的 /tmp/cam.jpg）

和 `cam_preview.py` 的区别（很重要）：

  cam_preview.py  自己开板端相机（ffmpeg 直读 /dev/videoX）→ 跑平台时会**抢设备**
  cam_mirror.py   只读板端 `/tmp/cam.jpg`（start_cam.sh / platform_host 的管线在写），
                  **不开相机**，所以闭环跑着的时候也能看

用法：
    python3 core/board/tools/cam_mirror.py               # → http://127.0.0.1:8502/
    python3 core/board/tools/cam_mirror.py --port 8503

端点：
    /                一页 HTML（自动刷新，内含 /stream.mjpg）
    /stream.mjpg     motion-jpeg 流
    /snap.jpg        当前最新一帧
    /status.json     真实帧率 / **板端算出来的帧年龄** / 是否卡住

两条硬要求（吃过亏，写死在这里）：
  1. **重复帧不算新帧**。第一版把"cat 到同样的字节"也当一帧，于是管线停了以后
     `/status.json` 还报 `2.4 fps, age 0.3` —— 页面卡死而监控说一切正常。
     现在板端先算 md5，**只有内容变了才发**；主机侧的 fps 只统计"内容不同"的帧。
  2. **帧年龄必须由板端时钟算**（R5）。`/tmp/cam.jpg` 的 mtime 与主机时钟差好几年
     （板子没 RTC，时间停在 2023），主机拿自己的表减它只会得到负数/几千秒。
     所以板端在包头里同时给 `now` 和 `mtime`，age =(now-mtime) 在板端算。
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
DEFAULT_KEY = str(ROOT / "data/fpga/board_ssh/board_key")
DEFAULT_BOARD = "root@192.168.137.242"
SNAP = "/tmp/cam.jpg"

#: 板端发送器：内容变了才发（md5 去重），包头给 now/mtime/size 三个整数。
#: 用 `ssh board python3 -` 把这段喂进去 —— 不走 heredoc，避免引号/转义地狱。
REMOTE = r'''
import hashlib, os, sys, time
P = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cam.jpg"
out = sys.stdout.buffer
last = None
while True:
    try:
        with open(P, "rb") as f:
            data = f.read()
        m = os.stat(P).st_mtime_ns
    except OSError:
        time.sleep(0.2)
        continue
    h = hashlib.md5(data).digest()
    if h != last:
        last = h
        out.write(b"@T %d %d %d\n" % (time.time_ns(), m, len(data)))
        out.write(data)
        out.flush()
    time.sleep(0.12)
'''


class Latest:
    def __init__(self) -> None:
        self.cond = threading.Condition()
        self.seq = 0            # 内容不同的帧计数
        self.jpg = b""
        self.age = -1.0         # 板端算的帧年龄（秒）
        self.fps = 0.0          # 只统计内容不同的帧
        self.t_recv = 0.0
        self.err = ""
        self.connected = False
        self._last = 0.0

    def put(self, jpg: bytes, age: float) -> None:
        with self.cond:
            now = time.time()
            if self._last and now > self._last:
                inst = 1.0 / (now - self._last)
                self.fps = inst if not self.fps else 0.7 * self.fps + 0.3 * inst
            self._last = now
            self.seq += 1
            self.jpg = jpg
            self.age = age
            self.t_recv = now
            self.connected = True
            self.err = ""
            self.cond.notify_all()

    def wait(self, seq: int, timeout: float = 2.0):
        with self.cond:
            self.cond.wait(timeout)
            if self.seq == seq:
                return None
            return self.seq, self.jpg

    def state(self, stale_s: float) -> dict:
        return {"connected": self.connected,
                "fps": round(self.fps, 1),
                "age": round(self.age, 2),
                "board_age": round(self.age, 2),
                "stale": bool(self.age < 0 or self.age > stale_s),
                "stale_s": stale_s,
                "seq": self.seq,
                "last_error": self.err[:200]}


LATEST = Latest()


def _read_exact(stream, n: int):
    buf = b""
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _read_line(stream):
    buf = b""
    while True:
        ch = stream.read(1)
        if not ch:
            return None
        if ch == b"\n":
            return buf
        buf += ch
        if len(buf) > 4096:          # 头行不可能这么长，说明流错位了
            return b""


def reader(board: str, key: str, snap: str) -> None:
    """一条常连 ssh（`python3 -` 喂脚本）；断了就重连。"""
    cmd = ["ssh", "-i", key, "-o", "StrictHostKeyChecking=no",
           "-o", "ServerAliveInterval=5", "-o", "ConnectTimeout=8",
           board, "python3", "-", snap]
    while True:
        try:
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, bufsize=0)
        except OSError as e:
            with LATEST.cond:
                LATEST.connected = False
                LATEST.err = f"ssh 起不来：{e}"
                LATEST.cond.notify_all()
            time.sleep(3.0)
            continue
        try:
            p.stdin.write(REMOTE.encode())
            p.stdin.close()
            assert p.stdout is not None
            while True:
                line = _read_line(p.stdout)
                if line is None:
                    break
                if not line.startswith(b"@T "):
                    continue
                try:
                    _now, mtime, size = [int(v) for v in line.split()[1:4]]
                except ValueError:
                    continue
                data = _read_exact(p.stdout, size)
                if data is None:
                    break
                age = max(0.0, (_now - mtime) / 1e9)
                LATEST.put(data, age)
        except (OSError, ValueError, AssertionError) as e:
            with LATEST.cond:
                LATEST.err = f"读流出错：{e}"
                LATEST.cond.notify_all()
        finally:
            try:
                p.terminate()
            except OSError:
                pass
        with LATEST.cond:
            LATEST.connected = False
            if not LATEST.err:
                LATEST.err = "ssh 断开，重连中"
            LATEST.cond.notify_all()
        time.sleep(2.0)


PAGE = """<!doctype html><meta charset=utf-8><title>board cam</title>
<style>body{margin:0;background:#111;color:#ccc;font:13px/1.5 system-ui}
img{width:100%;max-width:960px;display:block;background:#000}
.bar{display:flex;gap:14px;padding:6px 10px;align-items:center}
#st.ok{color:#7c7} #st.bad{color:#f66;font-weight:600}
</style>
<div class=bar><b>板载相机 /tmp/cam.jpg</b><span id=st class=bad>…</span></div>
<img id=f src="/snap.jpg">
<script>
const el=document.getElementById('st'),f=document.getElementById('f');
async function tick(){const j=await (await fetch('/status.json')).json();
 el.textContent=`${j.fps.toFixed(1)} 帧/s（内容变化）  帧年龄 ${j.age<0?'—':j.age.toFixed(2)+' s'}  `
  +(j.stale?'⚠ 源已停（>'+j.stale_s+' s 没有新画面）':(j.connected?'实时':'离线'))
  +(j.last_error?'  '+j.last_error:'');
 el.className=j.stale?'bad':'ok';}
setInterval(tick,500); tick(); f.src='/stream.mjpg';
</script>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _json(self, obj) -> None:
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        stale_s = getattr(self.server, "stale_s", 5.0)
        if path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path in ("/snap.jpg", "/snap"):
            seq, jpg = LATEST.seq, LATEST.jpg
            if not jpg:
                got = LATEST.wait(seq, timeout=3.0)
                if got:
                    seq, jpg = got
            if not jpg:
                self.send_error(503, "no frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Age", f"{LATEST.age:.2f}")
            self.end_headers()
            self.wfile.write(jpg)
            return
        if path == "/status.json":
            self._json(LATEST.state(stale_s))
            return
        if path in ("/stream.mjpg", "/stream"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seq = -1
            while True:
                got = LATEST.wait(seq, timeout=2.0)
                if got is None:
                    continue
                seq, jpg = got
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            return
        self.send_error(404)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="转发板端 /tmp/cam.jpg 的实时画面（不占相机；重复帧不算新帧）")
    ap.add_argument("--port", type=int, default=8502)
    ap.add_argument("--board", default=DEFAULT_BOARD)
    ap.add_argument("--key", default=DEFAULT_KEY)
    ap.add_argument("--snap", default=SNAP, help="板端快照路径")
    ap.add_argument("--stale-s", type=float, default=5.0,
                    help=f"超过这么多秒没有新画面就标记为卡住（R5：默认 5 s）")
    ap.add_argument("--bind", default="127.0.0.1")
    a = ap.parse_args(argv)

    threading.Thread(target=reader, args=(a.board, a.key, a.snap),
                     daemon=True).start()
    httpd = ThreadingHTTPServer((a.bind, int(a.port)), Handler)
    httpd.daemon_threads = True
    httpd.stale_s = float(a.stale_s)
    print(f"相机镜像： http://{a.bind}:{a.port}/   "
          f"（/snap.jpg /status.json；卡住阈值 {a.stale_s:.0f} s）", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
