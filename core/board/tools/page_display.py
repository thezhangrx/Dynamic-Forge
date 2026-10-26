#!/usr/bin/env python3
"""page_display.py —— 只把**平台页面**放到屏幕上（不跑决策、不需要板子）。

为什么单独做一个
----------------
平时要看"相机眼里的画面"，必须有东西显示在屏幕上；但完整平台会同时跑决策、
开板子、写一堆日志。调页面时只想"把页面摆上去"。

顺带一个用途：`--y-speed` 让角色沿场地慢慢往上走，配合 `cam_preview.py`
可以直接目视检查"相机眼里的页面偏不偏"。

用法
----
    python3 core/board/tools/page_display.py                 # 静态放到出生点
    python3 core/board/tools/page_display.py --y-speed 60    # 慢慢往上爬，循环
    # 另开一个终端：
    python3 core/board/tools/cam_preview.py --port 8500      # 相机实时预览
    # 浏览器： http://127.0.0.1:8500/
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core"))

from bullet_sim.render.pygame_view import PygameRenderer          # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec                # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv                # noqa: E402
from full_loop import FrameServer, PagePublisher                  # noqa: E402


def _rgb(text: str, default):
    """'r,g,b' → tuple，解析失败就用默认值。"""
    try:
        parts = [int(float(x)) for x in str(text).replace(" ", "").split(",")]
        if len(parts) == 3:
            return tuple(parts)
    except (TypeError, ValueError):
        pass
    return default


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="只显示平台页面（不跑决策）")
    ap.add_argument("--scenario",
                    default=str(ROOT / "configs/20_walls_alt_24_dy140_y120.json"))
    ap.add_argument("--port", type=int, default=8490, help="MJPEG 端口（page_view 连它）")
    ap.add_argument("--y", type=float, default=None, help="固定角色 y（不指定就用出生点）")
    ap.add_argument("--y-speed", type=float, default=0.0,
                    help="角色 y 的速度（场地单位/秒，墙钟）；0 = 静止")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-viewer", action="store_true", help="不自己拉全屏查看器")
    #: 平台/标记尺寸必须与**闭环标准配置**一致（`loop_standard.sh` 那两处），
    #: 否则量出来的 quad 根本不是跑的时候那个取景（这条踩过：page_display 默认
    #: margin=24、1280x960，而闭环是 margin=0、1440x1080）。
    ap.add_argument("--content", default="1440x1080",
                    help="查看器把 640×480 页面拉伸到的尺寸（与 --viewer-size 一致）")
    ap.add_argument("--fiducial-side", type=float, default=48.0,
                    help="四角 ArUco 标记边长（场地单位）；与 loop_standard.sh 一致")
    #: 与 loop_standard.sh 完全一致的配色（默认值就是标准配置那组）
    ap.add_argument("--wall-rgb", default="255,240,200")
    ap.add_argument("--field-rgb", default="12,14,20")
    ap.add_argument("--bg-rgb", default="0,0,0")
    a = ap.parse_args(argv)

    spec = ScenarioSpec.load(a.scenario)
    env = BulletHellEnv(spec, seed=spec.seed)
    env.reset()
    y0 = float(env.world.player[1]) if a.y is None else float(a.y)
    env.world.player[1] = y0
    fh = float(env.world.state.env.field_h)

    server = FrameServer(int(a.port))
    pub = PagePublisher(server, fps=max(10.0, a.fps)).start()
    #: **配色必须与闭环标准配置一致**（`loop_standard.sh` 里那一组）：
    #: `--wall-rgb 255,240,200 --field-rgb 12,14,20 --bg-rgb 0,0,0`。
    #: 以前这里没传，用的是渲染器默认的灰蓝底 ⇒ 相机看到的是"另一张平台画面"，
    #: 和实跑时的深底完全不同（会被误认为"平台背景变了"）。
    wall = _rgb(a.wall_rgb, (255, 240, 200))
    field_rgb = _rgb(a.field_rgb, (12, 14, 20))
    bg_rgb = _rgb(a.bg_rgb, (0, 0, 0))
    view = PygameRenderer(mode="rgb_array", follow_player=True, follow_h=240.0,
                          show_potential=False, show_text=False,
                          show_fiducials=True,
                          fiducial_anchor="band",
                          fiducial_side=float(a.fiducial_side),
                          show_world_axes=False, show_field_border=True,
                          frame_px=2, show_hitboxes=False,
                          wall_color=wall, background=bg_rgb, field_bg=field_rgb,
                          window_size=(640, 480))

    viewer = None
    if not a.no_viewer:
        venv = dict(os.environ)
        venv["DISPLAY"] = venv.get("DISPLAY", ":0")
        venv["SDL_VIDEODRIVER"] = "x11"
        viewer = subprocess.Popen(
            [sys.executable, str(ROOT / "core/board/tools/page_view.py"),
             "--url", f"http://127.0.0.1:{int(a.port)}/stream.mjpg",
             "--fullscreen", "--content", str(a.content),
             "--x", "320", "--y", "60", "--w", "1280", "--h", "960",
             "--fps", f"{max(20.0, a.fps):g}", "--margin", "0", "--idle-exit", "0"],
            env=venv, stdout=subprocess.DEVNULL)

    print(f"页面已开： http://127.0.0.1:{int(a.port)}/  （角色 y={y0:.0f}）", flush=True)
    print("相机预览： python3 core/board/tools/cam_preview.py --port 8500", flush=True)
    t0 = time.monotonic()
    y = y0
    try:
        while True:
            now = time.monotonic()
            if viewer is not None and viewer.poll() is not None:
                print("查看器退出了（屏幕/显示问题？）", file=sys.stderr, flush=True)
                viewer = None
            if a.y_speed:
                y = y0 + (now - t0) * float(a.y_speed)
                span = max(1.0, fh - 80.0)
                y = 40.0 + (y - 40.0) % span
                env.world.player[1] = y
            pub.submit(view.render(env.world), float(env.world.player[1]))
            time.sleep(max(0.0, 1.0 / max(1.0, a.fps) - (time.monotonic() - now)))
    except KeyboardInterrupt:
        return 0
    finally:
        pub.stop()
        if viewer is not None:
            try:
                viewer.terminate()
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
