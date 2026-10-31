#!/usr/bin/env python3
"""cam_preview.py —— 板载相机的**实时预览**（主机侧中转）。

为什么需要
----------
相机插在板子上（`/dev/video0`），平台页面在主机上。平时要看"相机眼里的画面"，
只能等一轮跑完再从 `cam_rec.mkv` 里取帧 —— 调页面/调曝光时太慢。

做法
----
主机上开一个小 MJPEG 服务：后台用 ssh 在**板上**跑 ffmpeg 读 `/dev/video0`，
把 JPEG 流打到 stdout，这里按 SOI/EOI（`FFD8 … FFD9`）切帧缓存最新一帧，
再以 `multipart/x-mixed-replace` 发给浏览器。

用
--
    python3 core/board/tools/cam_preview.py --port 8500
    # 浏览器打开： http://127.0.0.1:8500/          （实时）
    #              http://127.0.0.1:8500/snap.jpg   （单帧，适合反复刷新/取快照）

链接断了（板子网卡抖动很常见）会自动重连，不会把服务弄死。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE_DEFAULT_SSH = "root@192.168.137.242"
HERE_DEFAULT_KEY = "data/fpga/board_ssh/board_key"

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>board camera (live)</title>
<style>
html,body{margin:0;background:#050608;color:#bbb;font:13px/1.5 monospace}
#bar{padding:6px 10px;display:flex;gap:14px;align-items:center;flex-wrap:wrap}
#v{display:block;width:100vw;height:auto}
a{color:#8cf}
.bad{color:#f88}
</style></head><body>
<div id="bar">
  <b>板载相机 /dev/video0</b>
  <span id="s">连接中…</span>
  <span>· <a href="/snap.jpg">单帧</a></span>
  <span>· <a href="/stream.mjpg">MJPEG 原始流</a>（若浏览器不显示，用本页）</span>
  <span>· <a href="/status.json">/status.json</a></span>
</div>
<img id="v" alt="waiting for first frame">
<script>
let ok = 0, fail = 0, lastUrl = null;
async function tick() {
  try {
    const r = await fetch('/snap.jpg?t=' + Date.now(), {cache: 'no-store'});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const blob = await r.blob();
    if (lastUrl) URL.revokeObjectURL(lastUrl);
    lastUrl = URL.createObjectURL(blob);
    document.getElementById('v').src = lastUrl;
    ok++;
  } catch (e) {
    fail++;
  }
  const s = document.getElementById('s');
  s.textContent = '帧 ' + ok + ' / 失败 ' + fail + '  ' + new Date().toLocaleTimeString();
  s.className = (fail > 20 && ok === 0) ? 'bad' : '';
  if (fail > 20 && ok === 0) s.textContent += '  （服务连不上板子？看 /status.json）';
  setTimeout(tick, 130);
}
tick();
</script>
</body></html>
"""


class Latest:
    """只保留最新一帧 JPEG（预览不需要队列）。"""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpg: bytes | None = None
        self._seq = 0
        self.connected = False
        self.fps = 0.0
        self.last_error = ""
        self._count = 0
        self._t0 = time.monotonic()

    def put(self, jpg: bytes) -> None:
        with self._cond:
            self._jpg = jpg
            self._seq += 1
            self.connected = True
            self._count += 1
            dt = time.monotonic() - self._t0
            if dt >= 2.0:
                self.fps = self._count / dt
                self._count = 0
                self._t0 = time.monotonic()
            self._cond.notify_all()

    def down(self, err: str) -> None:
        with self._cond:
            self.connected = False
            self.last_error = err
            self._cond.notify_all()

    def wait(self, after: int, timeout: float):
        with self._cond:
            if self._seq <= after:
                self._cond.wait(timeout)
            if self._jpg is None or self._seq <= after:
                return None
            return self._seq, self._jpg


LATEST = Latest()


def _split_mjpeg(stream, sink) -> None:
    """从 ffmpeg 的 MJPEG 裸流里按 FFD8…FFD9 切帧。"""
    buf = bytearray()
    while True:
        chunk = stream.read(65536)
        if not chunk:
            return
        buf += chunk
        while True:
            i = buf.find(b"\xff\xd8")
            if i < 0:
                del buf[: max(0, len(buf) - 1)]
                break
            j = buf.find(b"\xff\xd9", i + 2)
            if j < 0:
                del buf[: max(0, i - 1)]
                break
            jpg = bytes(buf[i : j + 2])
            del buf[: j + 2]
            sink(jpg)


def camera_loop(ssh_target: str, key: str, size: str, fps: int,
                set_cam: bool = False, dev: str = "/dev/video0") -> None:
    """ssh 到板上跑 ffmpeg，断线自动重连。"""
    #: 先拼远程命令串：**三元表达式要单独一行**，否则 `A if c else "" + B`
    #: 会被解析成 `A if c else ("" + B)`，开 `--set-cam` 时反而把 ffmpeg 丢掉。
    _pre = ""
    if set_cam:
        _pre = ("v4l2-ctl -d ${DEV:-/dev/video0} --set-ctrl=exposure_auto=1 "
                "--set-ctrl=exposure_absolute=200 "
                "--set-ctrl=white_balance_temperature_auto=0 "
                "--set-ctrl=white_balance_temperature=4000 --set-ctrl=gain=4 "
                "2>/dev/null || true; ")
    #: **设备节点自动探测**：相机拔插后枚举号会变（实测 /dev/video0 → /dev/video1），
    #: 旧进程还抓着 `/dev/video0 (deleted)` 的 fd 继续出帧 —— 表现是"画面不对、
    #: 参数改了没反应"。这里每次重连都重新探一次，重插也能自愈。
    _detect = (
        "DEV=$(v4l2-ctl --list-devices 2>/dev/null | awk '"
        "/^[^ \t]/ { skip = ($0 ~ /platform|mvx/); next } "
        "!skip && match($0, /\\/dev\\/video[0-9]+/) "
        "{ print substr($0, RSTART, RLENGTH); exit }'); "
        f"DEV=${{DEV:-{dev}}}; ")
    #: 顺序很重要：**先探测 DEV，再用 DEV 设控制**（否则 `_pre` 里的 ${DEV} 还是空）
    _remote = _detect + _pre + (
        f"ffmpeg -hide_banner -loglevel error -fflags nobuffer -flags low_delay "
        f"-probesize 32 -analyzeduration 0 -f v4l2 -input_format mjpeg "
        f"-video_size {size} -framerate {int(fps)} -i \"$DEV\" "
        f"-vf fps={int(fps)} -f mjpeg -q:v 5 -")
    cmd = [
        "ssh", "-i", key,
        "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null",
        "-o", "LogLevel=ERROR",
        "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=5",
        ssh_target,
        _remote,
    ]
    while True:
        try:
            #: stderr 必须**要么重定向到文件、要么有人读**：用 PIPE 而不读会把
            #: 管道写满，ffmpeg 就卡在 write 上不再出帧（实测板端 ffmpeg 一直占着
            #: /dev/video0 却一帧都不出来）。
            errlog = open("/tmp/cam_preview_ffmpeg.log", "wb")
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=errlog)
            try:
                _split_mjpeg(proc.stdout, LATEST.put)
            finally:
                try:
                    proc.kill()
                except OSError:
                    pass
                errlog.close()
            try:
                err = open("/tmp/cam_preview_ffmpeg.log", "rb").read()[-300:]
            except OSError:
                err = b""
            LATEST.down(err.decode("utf-8", "ignore").strip() or "stream ended")
        except OSError as exc:
            LATEST.down(str(exc))
        time.sleep(1.5)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # noqa: D401 - 静默
        pass

    def do_GET(self) -> None:  # noqa: N802
        #: **查询串要剥掉**：首页用 `/snap.jpg?t=<时间戳>` 防浏览器缓存，
        #: 直接拿 `self.path` 比对会把它当 404（实测第一版就是这样）。
        path = self.path.split("?", 1)[0]
        self.path = path
        if self.path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ("/snap.jpg", "/snap"):
            got = LATEST.wait(-1, timeout=3.0)
            if got is None:
                self.send_error(503, "no frame yet (board unreachable?)")
                return
            _seq, jpg = got
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
            return
        if self.path in ("/status", "/status.json"):
            body = json.dumps({
                "connected": LATEST.connected,
                "fps": round(LATEST.fps, 1),
                "last_error": LATEST.last_error[:200],
            }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ("/stream.mjpg", "/stream"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seq = -1
            while True:
                got = LATEST.wait(seq, timeout=2.0)
                if got is None:
                    if not LATEST.connected:
                        time.sleep(0.2)
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
    ap = argparse.ArgumentParser(description="板载相机实时预览（主机侧 MJPEG 中转）")
    ap.add_argument("--port", type=int, default=8500)
    ap.add_argument("--ssh", default=HERE_DEFAULT_SSH, help="板子 ssh 目标")
    ap.add_argument("--key", default=HERE_DEFAULT_KEY)
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--dev", default="/dev/video0",
                    help="兜底设备节点（正常靠板端自动探测；探测失败才用它）")
    ap.add_argument("--set-cam", action="store_true",
                    help="先按平台那套设置相机曝光/增益（会覆盖当前设置！跑平台时别开）")
    a = ap.parse_args(argv)

    threading.Thread(target=camera_loop,
                     args=(a.ssh, a.key, a.size, a.fps, a.set_cam, a.dev),
                     daemon=True).start()
    httpd = ThreadingHTTPServer(("127.0.0.1", int(a.port)), Handler)
    httpd.daemon_threads = True
    print(f"预览服务： http://127.0.0.1:{a.port}/   （单帧 /snap.jpg，状态 /status.json）",
          flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
