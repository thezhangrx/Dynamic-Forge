#!/usr/bin/env python3
"""把 8100 的 MJPEG 页面显示到 PC 屏幕上（WSLg X11 窗口），给板子相机拍。

为什么不用 Windows 浏览器：本闭环的相机对着 PC 屏幕；这里不需要任何 Windows
互操作，直接在 WSLg 的 X11 桌面开一个无边框窗口，位置固定为 (320,60)、
1280×960 —— 这正是白框视觉验证过的机位（`whiteframe_*`）。

页面本身是 full_loop.py 在 8100 上提供的 MJPEG；这个进程只是"显示器"：
浏览器仍可以打开 http://127.0.0.1:8100/ ，两边看到的是同一帧。

用法：
    DISPLAY=:0 python3 core/board/tools/page_view.py --url http://127.0.0.1:8100/stream.mjpg

窗口内容 = 640×480 的平台帧放大到 1280×960（smoothscale）。窗口位置用
SDL_VIDEO_WINDOW_POS 固定；实测 WSLg/Weston 会把它放在 (320,60) 的内容区。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# 必须在 import pygame 之前设置。
os.environ.setdefault("SDL_VIDEODRIVER", "x11")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("DISPLAY", ":0")

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pygame  # noqa: E402

from mjpeg_client import MjpegClient  # noqa: E402


class WindowRaiser:
    """把窗口顶到最前（X11 `XRaiseWindow`）。

    为什么要：相机对着 PC 屏幕做 VT，只要别的窗口（比如跑日志的终端）盖在
    页面上，VT 就会把终端里的文字当成"墙行"，整轮数据全废且很难从数值上看出来。
    这台机器没有 wmctrl/xdotool，pygame 这个构建也没有 always_on_top，
    所以直接用 ctypes 调 libX11。
    """

    def __init__(self) -> None:
        self.dpy = None
        self.wid = 0
        self.fn = None
        try:
            import ctypes
            import ctypes.util
            wid = pygame.display.get_wm_info().get("window")
            lib = ctypes.util.find_library("X11")
            if not wid or not lib:
                return
            x11 = ctypes.CDLL(lib)
            x11.XOpenDisplay.restype = ctypes.c_void_p
            dpy = x11.XOpenDisplay(None)
            if not dpy:
                return
            self.dpy = ctypes.c_void_p(dpy)
            self.wid = ctypes.c_ulong(wid)
            self._x11 = x11
            self._raise = x11.XRaiseWindow
            self._raise.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
            self._flush = x11.XFlush
            self._flush.argtypes = [ctypes.c_void_p]
        except Exception:                                # noqa: BLE001
            self.dpy = None

    def raise_now(self) -> None:
        if self.dpy is None:
            return
        try:
            self._raise(self.dpy, self.wid)
            self._flush(self.dpy)
        except Exception:                                # noqa: BLE001
            pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="WSLg 窗口显示 8100 的 MJPEG 页面")
    ap.add_argument("--url", default="http://127.0.0.1:8100/stream.mjpg")
    ap.add_argument("--x", type=int, default=320)
    ap.add_argument("--y", type=int, default=60)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=960)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--hold", action=argparse.BooleanOptionalAction, default=True,
                    help="流断了以后保留最后一帧（默认开）")
    ap.add_argument("--idle-exit", type=float, default=8.0,
                    help="N 秒没新帧就自己退出（避免旧窗口一直盖在新页面上）")
    args = ap.parse_args(argv)

    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{args.x},{args.y}"
    pygame.display.init()
    pygame.font.init()
    try:
        screen = pygame.display.set_mode((args.w, args.h), pygame.NOFRAME)
    except Exception as exc:                             # noqa: BLE001
        print(f"⚠ 开不了 X11 窗口（DISPLAY={os.environ.get('DISPLAY')}）：{exc!r}",
              file=sys.stderr)
        return 2
    pygame.display.set_caption("platform live")
    raiser = WindowRaiser()
    raiser.raise_now()
    font = pygame.font.SysFont("monospace", 18)
    print(f"viewer window {args.w}x{args.h} at ({args.x},{args.y})  url={args.url}",
          flush=True)

    client = MjpegClient(args.url).start()
    last_seq = -1
    last_frame = None
    last_paint = 0.0
    last_raise = 0.0
    waiting_since = time.monotonic()
    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    return 0
                if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    return 0
            got = client.latest(after_seq=last_seq, timeout=0.2)
            if got is not None:
                last_seq, _t, jpg = got
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    last_frame = img
                    waiting_since = time.monotonic()
            now = time.monotonic()
            if now - last_paint >= 1.0 / max(1.0, args.fps):
                last_paint = now
                if last_frame is not None:
                    rgb = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)
                    surf = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
                    if surf.get_size() != (args.w, args.h):
                        surf = pygame.transform.smoothscale(surf, (args.w, args.h))
                    screen.blit(surf, (0, 0))
                elif not args.hold:
                    screen.fill((0, 0, 0))
                else:
                    screen.fill((0, 0, 0))
                    if font is not None:
                        txt = font.render("waiting for platform stream ...", True,
                                          (200, 200, 200))
                        screen.blit(txt, (32, 32))
                    if now - waiting_since > 15.0:
                        print(f"⚠ 15 s 没收到帧：{client.last_error}", file=sys.stderr)
                        waiting_since = now
                pygame.display.flip()
            if args.idle_exit > 0 and now - waiting_since > args.idle_exit:
                print(f"⚠ {args.idle_exit:g} s 没新帧，退出（防止僵窗盖住新页面）",
                      file=sys.stderr, flush=True)
                return 0
            if now - last_raise >= 1.0:
                #: 每秒顶一次，防止终端/编辑器跑上来盖住页面（盖住 = VT 读文字）
                last_raise = now
                raiser.raise_now()
            if not pygame.display.get_init():
                return 0
            time.sleep(0.005)
    except KeyboardInterrupt:
        return 0
    finally:
        client.stop()
        try:
            pygame.quit()
        except Exception:                                # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())
