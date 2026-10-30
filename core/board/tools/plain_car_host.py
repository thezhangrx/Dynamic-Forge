#!/usr/bin/env python3
"""plain_car_host.py —— 无角色平台 + 板端指令"分方向"转发（不改原程序）

用户口径（2026-10-30）
    板子发来的 **前后** 指令  → 平台视角 **前后移**（角色在场地 y 上走，页面跟着滚）
    板子发来的 **左右** 指令  → **小车前后跑**（"平台从左到右"= 小车的前向）
    ⇒ 平台 +x = 小车前向；平台 −x = 小车后退
    平台本身仍然**不画角色**（量投影/看图用），只是视口跟着 y 滚。

怎么做到"不动原程序"
    不碰 `platform_host.py` 的一行代码，只是把它的两个"零件"拿来用：
      * `SshLink`：ssh 起板端 `board_agent.py --stdio`，收 `@MOVE`、发 `@POSE`；
      * `BulletHellEnv` / `PygameRenderer` / `FrameServer+PagePublisher`：跟平台一样。
    然后在**我们这个脚本里**把 act 拆成两半再分别送出去：
      act 1(-x) 2(+x) 3(+y) 4(-y) 5..8(对角)  →  y 分量喂给平台，x 分量喂给小车

用法
    # 真板子（相机管线要先起好，比如 loop_standard.sh 起的那种）+ 真小车
    core/board/tools/plain_car_host.py --board-ssh root@192.168.137.242

    # 不接小车，只看平台怎么滚（小车指令只打印）
    core/board/tools/plain_car_host.py --board-ssh root@192.168.137.242 --no-car

    # 完全离线自测：键盘按 1/2/3/4/0 当板子指令
    core/board/tools/plain_car_host.py --no-board --no-car

按键（自测/应急）：3=前 4=后 1=左 2=右 0=停  q=退出
"""
from __future__ import annotations

import argparse
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "core"))
sys.path.insert(0, HERE)

#: act -> (x 分量, y 分量)，与 car_kin.c 的 k_dirs 一致（页面坐标，y 向下）
#: 1=-x 2=+x 3=+y 4=-y 5=(-1,+1) 6=(+1,+1) 7=(-1,-1) 8=(+1,-1)
DIRS = {0: (0, 0), 1: (-1, 0), 2: (1, 0), 3: (0, 1), 4: (0, -1),
        5: (-1, 1), 6: (1, 1), 7: (-1, -1), 8: (1, -1)}


def split_act(act: int, car_sign: int, v: float):
    """板子 act → (喂给平台的 world_act, 喂给小车的线速度 mm/s)。

    用户口径：y 分量给平台（前后移），x 分量给小车（前后跑，平台 +x = 小车前向）。
    world_act 只取 3/4（避免平台在 x 上乱走）。
    """
    dx, dy = DIRS.get(int(act), (0, 0))
    world_act = 3 if dy > 0 else (4 if dy < 0 else 0)
    car_v = (car_sign * dx * float(v)) if dx else 0.0
    return world_act, car_v


def _rgb(text, default):
    try:
        p = [int(float(x)) for x in str(text).replace(" ", "").split(",")]
        if len(p) == 3:
            return tuple(p)
    except (TypeError, ValueError):
        pass
    return default


class Car:
    """E30 无线链路 → 小车：@MODE 1（差速）+ 10 Hz @VEL 心跳"""

    def __init__(self, port: str, baud: int, v_forward: float, dry: bool):
        self.dry, self.v_forward = dry, float(v_forward)
        self.v, self.w = 0.0, 0.0
        self.s = None
        self._stop = False
        if dry:
            print("[car] dry-run：只打印，不开串口")
        else:
            import serial
            self.s = serial.Serial(port, baud, timeout=0.05)
            self.cmd("@STOP")
            time.sleep(0.05)
            self.cmd("@MODE 1")
        threading.Thread(target=self._hb, daemon=True).start()

    def cmd(self, line: str) -> None:
        if self.s is None:
            return
        try:
            self.s.write((line + "\n").encode())
        except Exception:
            pass

    def _hb(self) -> None:
        last = None
        while not self._stop:
            line = "@VEL %d %d" % (int(round(self.v)), int(round(self.w)))
            if line != last:
                print("[car] %s" % line, flush=True)
                last = line
            if self.s is not None:
                try:
                    self.s.write((line + "\n").encode())
                except Exception:
                    pass
            time.sleep(0.1)

    def set(self, v: float, w: float = 0.0) -> None:
        self.v, self.w = v, w

    def stop_now(self) -> None:
        self.set(0.0, 0.0)
        self.cmd("@VEL 0 0")
        self.cmd("@STOP")

    def close(self) -> None:
        self.stop_now()
        self._stop = True
        time.sleep(0.05)
        if self.s is not None:
            try:
                self.s.close()
            except Exception:
                pass


class Hotkeys:
    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else -1
        self.old = None

    def __enter__(self):
        if self.enabled:
            try:
                self.old = termios.tcgetattr(self.fd)
                tty.setraw(self.fd)
                os.set_blocking(self.fd, False)
            except (termios.error, OSError):
                self.enabled = False
        return self

    def __exit__(self, *exc):
        if self.enabled and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def poll(self) -> str:
        if not self.enabled:
            return ""
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return ""
        try:
            return os.read(self.fd, 1).decode(errors="ignore")
        except (OSError, BlockingIOError):
            return ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="无角色平台 + 板端指令分方向转发")
    ap.add_argument("--scenario", default=os.path.join(ROOT, "configs/20_walls_alt_24_dy140.json"))
    ap.add_argument("--port", type=int, default=8493, help="页面 MJPEG 端口")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--content", default="1440x1080")
    ap.add_argument("--viewer-pos", default="320,60")
    ap.add_argument("--viewer-size", default="1440x1080")
    ap.add_argument("--fiducial-side", type=float, default=48.0)
    ap.add_argument("--wall-rgb", default="255,240,200")
    ap.add_argument("--field-rgb", default="12,14,20")
    ap.add_argument("--bg-rgb", default="0,0,0")
    ap.add_argument("--y", type=float, default=None, help="起始视口中心 y（默认出生点）")
    # 板子
    ap.add_argument("--board-ssh", default="", help="root@192.168.137.242；空=不接板子")
    ap.add_argument("--ssh-key", default=os.path.join(ROOT, "data/fpga/board_ssh/board_key"))
    ap.add_argument("--board-agent", default="/root/tools/board_agent.py")
    ap.add_argument("--board-python", default="python3")
    ap.add_argument("--board-extra", default="")
    ap.add_argument("--no-board", action="store_true", help="用键盘顶替板端指令")
    # 小车
    ap.add_argument("--car-port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    ap.add_argument("--car-baud", type=int, default=115200)
    ap.add_argument("--car-v", type=float, default=130.0, help="小车前后速度 mm/s")
    ap.add_argument("--car-x-sign", type=int, default=1, choices=(1, -1),
                    help="平台 +x 是 1（小车前进）还是 -1（小车后退）")
    ap.add_argument("--no-car", action="store_true", help="不接小车（只打印）")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="跑多久后自己停（0=不限）")
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

    view = PygameRenderer(mode="rgb_array", follow_player=True, follow_h=240.0,
                          show_potential=False, show_text=False, show_fiducials=True,
                          fiducial_anchor="band", fiducial_side=float(a.fiducial_side),
                          show_world_axes=False, show_field_border=True, frame_px=2,
                          show_hitboxes=False,
                          wall_color=_rgb(a.wall_rgb, (255, 240, 200)),
                          background=_rgb(a.bg_rgb, (0, 0, 0)),
                          field_bg=_rgb(a.field_rgb, (12, 14, 20)),
                          show_player=False,                 # 无角色
                          window_size=(640, 480))

    #: 先渲染一帧：viewport 是在第一次 render 里建的（page_plain.py 同款）
    frame0 = view.render(env.world)
    scale = float(view.viewport.scale) if view.viewport is not None else 1.0

    # 板端链接（用 platform_host 里现成的 SshLink，不改它）
    link = None
    if not a.no_board and a.board_ssh:
        import platform_host as ph
        link = ph.SshLink(a.board_ssh, a.ssh_key, a.board_agent,
                          python=a.board_python, extra=a.board_extra)
        print("[board] 已起 board_agent --stdio（%s）" % a.board_ssh)
    else:
        print("[board] 不接板子：用键盘 1/2/3/4/0 当指令")

    car = Car(a.car_port, a.car_baud, a.car_v, dry=(a.no_car or a.no_board))

    server = FrameServer(int(a.port))
    pub = PagePublisher(server, fps=max(10.0, a.fps)).start()
    viewer = None
    if not a.no_viewer:
        venv = dict(os.environ)
        venv["DISPLAY"] = venv.get("DISPLAY", ":0")
        venv["SDL_VIDEODRIVER"] = "x11"
        vx, vy = a.viewer_pos.split(",")
        vw, vh = a.viewer_size.split("x")
        viewer = subprocess.Popen(
            [sys.executable, os.path.join(HERE, "page_view.py"),
             "--url", "http://127.0.0.1:%d/stream.mjpg" % int(a.port),
             "--fullscreen", "--content", str(a.content),
             "--x", vx, "--y", vy, "--w", vw, "--h", vh,
             "--fps", "%g" % max(20.0, a.fps), "--margin", "0", "--idle-exit", "0"],
            env=venv, stdout=subprocess.DEVNULL)

    print("=== 映射（用户口径）===")
    print("  板子 3(+y)/4(-y)  → 平台视角 前/后 移")
    print("  板子 2(+x)/1(-x)  → 小车 前/后 跑（平台 +x = 小车前向，符号 %+d）"
          % a.car_x_sign)
    print("  对角 5..8 拆成两半：y 给平台、x 给小车")
    print("  平台：%.0f 单位宽，scale=%.3f px/单位，视口中心 y=%.0f"
          % (field_w, scale, y0))
    print("  页面 %s  小车 %s" % (f"http://127.0.0.1:{a.port}/",
                                 "dry-run" if (a.no_car or a.no_board) else a.car_port),
          flush=True)

    hk = Hotkeys(not a.no_board)
    hk.__enter__()
    act = 0
    hk_act = None
    t_sim = 0.0
    dt = 1.0 / max(1.0, a.fps)
    last_pose = 0.0
    try:
        while True:
            if link is not None:
                if not link.ready():
                    print("[board] board_agent 退出了，收尾")
                    break
                act = int(link.act)
            ch = hk.poll()
            if ch:
                if ch in "123456780":
                    hk_act = int(ch)
                elif ch in ("q", "Q", "\x03"):
                    break
            if hk_act is not None:
                act = hk_act

            world_act, car_v = split_act(act, a.car_x_sign, a.car_v)
            if world_act:
                env.step(world_act)          # 平台视角前后移
            car.set(car_v, 0.0)              # 小车前后跑

            t_sim += dt
            px, py = float(env.world.player[0]), float(env.world.player[1])
            if link is not None and time.monotonic() - last_pose > 0.04:
                last_pose = time.monotonic()
                link.send_pose(px, py, float(env.world.player[4]))
            frame = view.render(env.world) if t_sim > 0 else frame0
            if frame is not None:
                pub.submit(frame, py)
            time.sleep(dt)
            if a.max_seconds and t_sim > a.max_seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\n收尾……")
        hk.__exit__()
        car.close()
        if link is not None:
            link.stop()
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
        time.sleep(0.3)
        print("  小车已 @STOP；页面/查看器已关；board_agent 已断开")
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        print("act  world_act  car_v(sign=+1)  car_v(sign=-1)   含义")
        ok = True
        for act in range(9):
            w, cv = split_act(act, +1, 130.0)
            w2, cv2 = split_act(act, -1, 130.0)
            print("%2d   %2d        %+7.0f       %+7.0f      %s"
                  % (act, w, cv, cv2,
                     {0: "停", 1: "小车后退", 2: "小车前进", 3: "平台前进",
                      4: "平台后退", 5: "平台前进+小车后退", 6: "平台前进+小车前进",
                      7: "平台后退+小车后退", 8: "平台后退+小车前进"}[act]))
            ok = ok and (w in (0, 3, 4)) and (cv == 0 or abs(cv) == 130.0)
        ok = ok and split_act(3, +1, 130)[0] == 3 and split_act(2, +1, 130)[1] == 130.0
        print("PASS" if ok else "FAIL")
        raise SystemExit(0 if ok else 1)
    raise SystemExit(main())
