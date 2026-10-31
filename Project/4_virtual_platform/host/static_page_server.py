#!/usr/bin/env python3
"""static_page_server.py —— 把一个 PNG 当成"平台页面"用 MJPEG 推给 page_view。

只用于**相机/曝光自检**：让 page_view 显示一张固定页面，板子拍一张照片回来，
量一下"暗场地/亮墙/角色点"的灰度分布 —— 这样不用跑闭环就能判断相机侧出了什么问题。

    Env/bin/python core/board/tools/static_page_server.py <png> --port 8600
"""
from __future__ import annotations

import argparse
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("png")
    ap.add_argument("--port", type=int, default=8600)
    ap.add_argument("--watch", action="store_true",
                    help="文件一变就换画面（测延时用：A/B 切换）")
    a = ap.parse_args()
    img = cv2.imread(a.png)
    if img is None:
        print(f"读不到 {a.png}")
        return 2
    ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if not ok:
        return 2
    state = {"blob": jpg.tobytes(), "mtime": Path(a.png).stat().st_mtime,
             "switched": time.time()}

    def _reload() -> None:
        while a.watch:
            time.sleep(0.05)
            try:
                mt = Path(a.png).stat().st_mtime
            except OSError:
                continue
            if mt <= state["mtime"]:
                continue
            im = cv2.imread(a.png)
            if im is None:
                continue
            ok2, enc = cv2.imencode(".jpg", im,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            if not ok2:
                continue
            state["blob"] = enc.tobytes()
            state["mtime"] = mt
            state["switched"] = time.time()
            print(f"  switched at {time.time():.3f} mtime={mt:.3f}", flush=True)

    if a.watch:
        import threading
        threading.Thread(target=_reload, daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):  # noqa: D401
            pass

        def do_GET(self) -> None:  # noqa: N802
            if not self.path.startswith("/stream.mjpg"):
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    blob = state["blob"]
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(blob)}\r\n\r\n".encode())
                    self.wfile.write(blob)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    time.sleep(1.0 / 30.0)
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer(("127.0.0.1", a.port), H)
    print(f"serving {a.png} at http://127.0.0.1:{a.port}/stream.mjpg", flush=True)
    srv.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
