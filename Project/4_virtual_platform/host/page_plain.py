#!/usr/bin/env python3
"""page_plain.py —— **无角色的平台页面**（只画场地/墙/缺口/四角标记 + 量尺）

为什么要它：`page_display.py` 会画一个角色圆点；量"投影长度"的时候那个点会挡在
场地中间、还会让相机把角色当成障碍。这个工具把角色关掉，其余（配色、取景、
四角 ArUco 标记、场地白框）与**闭环标准配置**完全一致，另外在页面底部画两条
**已知长度的量尺**，方便用卷尺/图像量投影尺寸。

用法
    Env/bin/python core/board/tools/page_plain.py                  # 全屏显示
    Env/bin/python core/board/tools/page_plain.py --no-viewer --dump /tmp/plain.png
    Env/bin/python core/board/tools/page_plain.py --ruler 100,500 --y 400

打印出来的几何（量投影就靠这几个数）
    page        640x480 px，视野 = 场地全宽 x (480/scale) 单位
    scale       px/单位（= usable_w / field_w，标准场景 field_w=640 ⇒ 1.0）
    viewer      把 640x480 拉到 --content（标准 1440x1080）⇒ 屏幕放大倍数
    量尺        底部两条白条 = --ruler 给的场地单位长度（默认 100 和 500）
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "core"))
sys.path.insert(0, HERE)

# --- 工程内路径引导（保证从工程任意位置启动都能定位依赖）---
_PRJ_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
for _p in (os.path.join(_PRJ_ROOT, "lib"),
           os.path.join(_PRJ_ROOT, "4_virtual_platform", "sim"),
           os.path.dirname(os.path.abspath(__file__))):
    if _p not in sys.path:
        sys.path.insert(0, _p)



def _rgb(text, default):
    try:
        parts = [int(float(x)) for x in str(text).replace(" ", "").split(",")]
        if len(parts) == 3:
            return tuple(parts)
    except (TypeError, ValueError):
        pass
    return default


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="无角色的平台页面（量投影长度用）")
    ap.add_argument("--scenario", default=os.path.join(ROOT, "configs/20_walls_alt_24_dy140.json"))
    ap.add_argument("--port", type=int, default=8491, help="MJPEG 端口")
    ap.add_argument("--y", type=float, default=None, help="视口对准的场地 y（默认出生点）")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-viewer", action="store_true", help="不拉全屏查看器")
    ap.add_argument("--content", default="1440x1080", help="查看器把页面拉到的尺寸")
    ap.add_argument("--viewer-pos", default="320,60", help="查看器窗口位置（与 loop_standard 一致）")
    ap.add_argument("--viewer-size", default="1440x1080", help="查看器窗口尺寸（与 loop_standard 一致）")
    ap.add_argument("--fiducial-side", type=float, default=48.0)
    ap.add_argument("--wall-rgb", default="255,240,200")
    ap.add_argument("--field-rgb", default="12,14,20")
    ap.add_argument("--bg-rgb", default="0,0,0")
    ap.add_argument("--ruler", default="100,500",
                    help="底部量尺长度（场地单位，逗号分隔；0 = 不画）")
    ap.add_argument("--ruler-rgb", default="255,255,255")
    ap.add_argument("--dump", default="", help="存一张 PNG 后退出（自检用）")
    a = ap.parse_args(argv)

    from bullet_sim.render.pygame_view import PygameRenderer        # noqa: E402
    from bullet_sim.scenarios.spec import ScenarioSpec              # noqa: E402
    from bullet_sim.simulator.env import BulletHellEnv              # noqa: E402
    from full_loop import FrameServer, PagePublisher                # noqa: E402

    spec = ScenarioSpec.load(a.scenario)
    env = BulletHellEnv(spec, seed=spec.seed)
    env.reset()
    y0 = float(env.world.player[1]) if a.y is None else float(a.y)
    env.world.player[1] = y0
    field_w = float(env.world.state.env.field_w)
    field_h = float(env.world.state.env.field_h)

    view = PygameRenderer(mode="rgb_array", follow_player=True, follow_h=240.0,
                          show_potential=False, show_text=False,
                          show_fiducials=True, fiducial_anchor="band",
                          fiducial_side=float(a.fiducial_side),
                          show_world_axes=False, show_field_border=True,
                          frame_px=2, show_hitboxes=False,
                          wall_color=_rgb(a.wall_rgb, (255, 240, 200)),
                          background=_rgb(a.bg_rgb, (0, 0, 0)),
                          field_bg=_rgb(a.field_rgb, (12, 14, 20)),
                          show_player=False,          # ← 无角色
                          window_size=(640, 480))

    #: pygame/SDL 初始化时会接管 SIGTERM（只投递 SDL_QUIT，进程不退出，
    #: 表现就是 kill -TERM 杀不掉、端口一直占着）。这里在渲染器建好之后
    #: 重新装一次：收到 TERM 就按 Ctrl-C 处理，走 finally 收尾。
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    frame = view.render(env.world)
    if frame is None:
        print("渲染失败")
        return 2

    # ---- 底部量尺 ---------------------------------------------------------
    rulers = [float(x) for x in str(a.ruler).split(",") if x.strip()]
    scale = float(view.viewport.scale)          # 页像素 / 场地单位
    try:
        import numpy as np
        img = np.array(frame, dtype=np.uint8) if not hasattr(frame, "shape") else frame
        rgb = _rgb(a.ruler_rgb, (255, 255, 255))
        H, W = img.shape[:2]
        ybar = H - 14
        for k, length in enumerate(rulers):
            if length <= 0:
                continue
            n = int(round(length * scale))
            n = min(n, W - 40)
            x0 = (W - n) // 2
            y = ybar - k * 10
            img[y:y + 3, x0:x0 + n] = rgb          # 横条
            img[y - 4:y + 7, x0:x0 + 2] = rgb      # 左端刻度
            img[y - 4:y + 7, x0 + n - 2:x0 + n] = rgb  # 右端刻度
        frame = img
    except Exception as e:                          # 画量尺失败不影响出画面
        print("量尺没画上：%s" % e)

    mul = 1.0
    try:
        cw, ch = [int(v) for v in str(a.content).lower().split("x")]
        mul = cw / 640.0
    except (ValueError, ZeroDivisionError):
        pass

    print("=== 无角色平台（scenario=%s）===" % os.path.basename(a.scenario))
    print("  场地        : %.0f x %.0f 单位" % (field_w, field_h))
    print("  页         : 640x480 px（margin 0）")
    print("  scale      : %.4f px/单位  ⇒ 视野宽 %.0f 单位，高 %.0f 单位"
          % (scale, 640.0 / scale, 480.0 / scale))
    print("  viewer     : %s（放大 %.3fx）⇒ 屏幕上 1 单位 = %.3f px"
          % (a.content, mul, scale * mul))
    for length in rulers:
        if length > 0:
            print("  量尺 %.0f 单位 → 页 %.0f px → 屏幕 %.0f px"
                  % (length, length * scale, length * scale * mul))
    print("  视口中心 y : %.0f（板带的 y ∈ [%.0f, %.0f]）"
          % (view.viewport.center[1], view._band[0], view._band[1]))

    if a.dump:
        import numpy as np
        from PIL import Image
        Image.fromarray(np.array(frame, dtype=np.uint8)).save(a.dump)
        print("  已存图：%s" % a.dump)
        return 0

    server = FrameServer(int(a.port))
    pub = PagePublisher(server, fps=max(10.0, a.fps)).start()
    print("  页面地址：http://127.0.0.1:%d/" % a.port, flush=True)

    viewer = None
    if not a.no_viewer:
        venv = dict(os.environ)
        venv["DISPLAY"] = venv.get("DISPLAY", ":0")
        venv["SDL_VIDEODRIVER"] = "x11"
        viewer = subprocess.Popen(
            [sys.executable, os.path.join(HERE, "page_view.py"),
             "--url", "http://127.0.0.1:%d/stream.mjpg" % int(a.port),
             "--fullscreen", "--content", str(a.content),
             "--x", str(a.viewer_pos.split(",")[0]), "--y", str(a.viewer_pos.split(",")[1]),
             "--w", str(a.viewer_size.split("x")[0]), "--h", str(a.viewer_size.split("x")[1]),
             "--fps", "%g" % max(20.0, a.fps), "--margin", "0", "--idle-exit", "0"],
            env=venv, stdout=subprocess.DEVNULL)

    try:
        while True:
            pub.submit(frame, y0)          # 静态画面（无角色、不动）
            import time
            time.sleep(1.0 / max(1.0, a.fps))
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
