#!/usr/bin/env python3
"""最小 MJPEG 客户端：读 `multipart/x-mixed-replace` 流里的 JPEG 帧。

板子上的 `mjpeg_server.py`（8099）和本机的平台页（8100）都是这种流。
`cv2.VideoCapture(url)` 在 WSL/OpenCV 5 上会卡在 multipart 解析里（实测 15 s
没有任何帧），所以这里按 JPEG 的 SOI/EOI 自己拆帧；只依赖标准库 + numpy/cv2。

用法（作为一个后台线程）：

    c = MjpegClient("http://127.0.0.1:8100/stream.mjpg")
    c.start()
    seq, t, jpg = c.latest(after_seq=-1, timeout=1.0)   # 阻塞到有新帧
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    c.stop()
"""

from __future__ import annotations

import threading
import time
import urllib.request


class MjpegClient:
    def __init__(self, url: str, *, timeout: float = 8.0, retry_s: float = 0.5) -> None:
        self.url = str(url)
        self.timeout = float(timeout)
        self.retry_s = float(retry_s)
        self._cond = threading.Condition()
        self._seq = -1
        self._t = 0.0
        self._jpeg: bytes | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.connects = 0
        self.errors = 0
        self.frames = 0
        self.last_error = ""

    # ------------------------------------------------------------------
    def start(self) -> "MjpegClient":
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="mjpeg-client",
                                            daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        th = self._thread
        if th is not None:
            th.join(timeout=3.0)
        self._thread = None

    # ------------------------------------------------------------------
    def latest(self, after_seq: int = -1, timeout: float = 1.0):
        """返回 `(seq, monotonic_time, jpeg_bytes)`；超时 / 停止时返回 `None`。"""
        deadline = time.monotonic() + max(0.0, timeout)
        with self._cond:
            while self._seq <= after_seq and not self._stop.is_set():
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)
            if self._stop.is_set() and self._seq <= after_seq:
                return None
            if self._jpeg is None:
                return None
            return self._seq, self._t, self._jpeg

    @property
    def seq(self) -> int:
        with self._cond:
            return self._seq

    # ------------------------------------------------------------------
    def _publish(self, jpg: bytes) -> None:
        with self._cond:
            self._seq += 1
            self._t = time.monotonic()
            self._jpeg = jpg
            self.frames += 1
            self._cond.notify_all()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                resp = urllib.request.urlopen(self.url, timeout=self.timeout)
            except Exception as exc:                     # noqa: BLE001
                self.errors += 1
                self.last_error = f"connect: {exc!r}"
                self._stop.wait(self.retry_s)
                continue
            self.connects += 1
            buf = b""
            try:
                while not self._stop.is_set():
                    chunk = resp.read(65536)
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
                        self._publish(jpg)
            except Exception as exc:                     # noqa: BLE001
                if not self._stop.is_set():
                    self.errors += 1
                    self.last_error = f"read: {exc!r}"
            finally:
                try:
                    resp.close()
                except Exception:                        # noqa: BLE001
                    pass
            if not self._stop.is_set():
                self._stop.wait(self.retry_s)
