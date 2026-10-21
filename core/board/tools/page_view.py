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
            #: 光 raise 有时压不住别的窗口（终端抢焦点后又会盖回来），
            #: 再把输入焦点抢过来 —— 实测这样才能稳定待在相机视野里。
            self._focus = x11.XSetInputFocus
            self._focus.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                    ctypes.c_int, ctypes.c_ulong]
        except Exception:                                # noqa: BLE001
            self.dpy = None

    def raise_now(self) -> None:
        if self.dpy is None:
            return
        try:
            #: 每次重新问一遍窗口 id：窗口刚创建时 get_wm_info() 可能还返回旧的
            #: （实测僵持在错误 id 上，页面就一直被终端盖着）
            wid = pygame.display.get_wm_info().get("window")
            if wid:
                self.wid = ctypes.c_ulong(wid)
            self._raise(self.dpy, self.wid)
            try:
                self._focus(self.dpy, self.wid, 1, 0)   # RevertToPointerRoot, CurrentTime
            except Exception:                            # noqa: BLE001
                pass
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
    ap.add_argument("--fullscreen", action="store_true",
                    help="全屏显示。WSLg(RAIL) 下全屏窗口通常会被抬成前台窗口，"
                         "比固定位置的无边框窗口更不容易被别的窗口盖住 —— 相机看不到"
                         "页面时先试这个")
    ap.add_argument("--content", default="",
                    help="全屏时页面内容画多大（如 1280x960），居中留黑边。"
                         "这样窗口是全屏（WSLg 下不会被别的窗口压住），"
                         "而页面仍落在相机调好的那块矩形上")
    ap.add_argument("--hold", action=argparse.BooleanOptionalAction, default=True,
                    help="流断了以后保留最后一帧（默认开）")
    ap.add_argument("--margin", type=int, default=24,
                    help="页面四周留黑边（相机 VT 靠白框定四边形，留边更稳）")
    ap.add_argument("--idle-exit", type=float, default=8.0,
                    help="N 秒没新帧就自己退出（避免旧窗口一直盖在新页面上）")
    args = ap.parse_args(argv)

    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{args.x},{args.y}"
    pygame.display.init()
    pygame.font.init()
    try:
        if args.fullscreen:
            screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            screen = pygame.display.set_mode((args.w, args.h), pygame.NOFRAME)
    except Exception as exc:                             # noqa: BLE001
        print(f"⚠ 开不了 X11 窗口（DISPLAY={os.environ.get('DISPLAY')}）：{exc!r}",
              file=sys.stderr)
        return 2
    pygame.display.set_caption("platform live")
    raiser = None
    font = pygame.font.SysFont("monospace", 18)
    print(f"viewer window {args.w}x{args.h} at ({args.x},{args.y})  url={args.url}",
          flush=True)

    sw, sh = screen.get_size()
    if args.content:
        cw, ch = (int(v) for v in args.content.lower().split("x"))
    else:
        cw, ch = args.w, min(args.h, sh)
    off_x, off_y = (sw - cw) // 2, (sh - ch) // 2
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
                    m = max(0, min(args.margin, min(cw, ch) // 4))
                    inner = (cw - 2 * m, ch - 2 * m)
                    if surf.get_size() != inner:
                        surf = pygame.transform.smoothscale(surf, inner)
                    screen.fill((0, 0, 0))
                    screen.blit(surf, (off_x + m, off_y + m))
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
                if raiser is None:
                    raiser = WindowRaiser()
            if args.idle_exit > 0 and now - waiting_since > args.idle_exit:
                print(f"⚠ {args.idle_exit:g} s 没新帧，退出（防止僵窗盖住新页面）",
                      file=sys.stderr, flush=True)
                return 0
            if raiser is not None and now - last_raise >= 0.5:
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
