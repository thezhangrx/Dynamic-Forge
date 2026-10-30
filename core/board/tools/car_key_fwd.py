#!/usr/bin/env python3
"""car_key_fwd.py —— 键盘 ↑/↓ 按住 = 电机**全速**正转 / 反转（最朴素的遥控）

    ↑（或 w）按住 → @VEL 690 0   两轮全速正转（PWM 6900 已到限幅）
    ↓（或 s）按住 → @VEL -690 0  两轮全速反转
    松开          → @VEL 0 0     停
    空格          → 立刻 @STOP（清速度环 + 停）
    q / Ctrl-C    → 退出（退出前 @VEL 0 0 + @STOP）

为什么是 690：`@MODE 1`（差速）下 `velocity = v(mm/s) × 10` PWM，690 mm/s ⇒
6900 PWM = `max_pwm` 限幅，再加 400 死区就被 `motor_set_pwm` 夹到满占空比。

只发现成协议，固件一行不改：
    @MODE 1   切纯差速（直立环不参与；不然姿态环会盖掉速度指令）
    @VEL v 0  线速度 mm/s
    @STOP     停 + 清速度环积分
车端有 1 s 看门狗，所以这里 10 Hz 心跳一直重发。

用法
    python3 core/board/tools/car_key_fwd.py
    python3 core/board/tools/car_key_fwd.py --v 300      # 不满速（300 mm/s）
    python3 core/board/tools/car_key_fwd.py --dry-run    # 只打印不发串口
    python3 core/board/tools/car_key_fwd.py --no-mode    # 不切差速（保持自平衡）
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from keyboard_drive import HOLD_LOCK_S, parse_events   # noqa: E402  复用解析/去抖

FULL_V = 690.0          # mm/s ⇒ PWM 6900（限幅）


def next_v(cur: float, kind: str, val: bytes, now: float, v_full: float,
           hold_lock: float = HOLD_LOCK_S):
    """按一次事件算新的目标速度（纯逻辑，好测）。

    返回 (v, hold_until 或 None)。hold_until=None 表示"不是按住类事件"。
    """
    if kind == "arrow":
        if val == b"A":
            return +v_full, now + hold_lock
        if val == b"B":
            return -v_full, now + hold_lock
        return cur, None
    ch = val
    if ch in (b"w", b"W"):
        return +v_full, now + hold_lock
    if ch in (b"s", b"S"):
        return -v_full, now + hold_lock
    if ch == b" ":
        return 0.0, 0.0
    return cur, None


class Car:
    def __init__(self, port: str, baud: int, dry: bool):
        self.dry = dry
        self.v = 0.0
        self.last = None
        self.s = None
        if dry:
            print("[car] dry-run：不打开串口，只打印")
        else:
            import serial
            self.s = serial.Serial(port, baud, timeout=0.05)
        self._stop = False
        import threading
        threading.Thread(target=self._hb, daemon=True).start()

    def cmd(self, line: str) -> None:
        print("[car] %s" % line, flush=True)
        if self.s is not None:
            try:
                self.s.write((line + "\n").encode())
            except Exception:
                pass

    def _hb(self) -> None:
        while not self._stop:
            line = "@VEL %d 0" % int(round(self.v))
            if self.s is not None:
                try:
                    self.s.write((line + "\n").encode())
                except Exception:
                    pass
            time.sleep(0.1)

    def set(self, v: float) -> None:
        if int(round(v)) != int(round(self.v)):
            print("[car] @VEL %d 0" % int(round(v)), flush=True)
        self.v = v

    def close(self) -> None:
        self.set(0.0)
        self.cmd("@VEL 0 0")
        self.cmd("@STOP")
        self._stop = True
        time.sleep(0.1)
        if self.s is not None:
            try:
                self.s.close()
            except Exception:
                pass


def main() -> int:
    ap = argparse.ArgumentParser(description="↑/↓ 按住 = 电机全速正/反转")
    ap.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--v", type=float, default=FULL_V, help="mm/s，690=满 PWM（默认）")
    ap.add_argument("--no-mode", action="store_true", help="不切 @MODE 1")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不开串口")
    a = ap.parse_args()

    car = Car(a.port, a.baud, a.dry_run)
    if not a.no_mode:
        car.cmd("@STOP")
        time.sleep(0.05)
        car.cmd("@MODE 1")
        time.sleep(0.2)

    print("就绪（@MODE %s，v=%.0f mm/s ⇒ PWM %.0f）"
          % ("0" if a.no_mode else "1", a.v, min(a.v * 10, 6900)))
    print("  ↑/w 按住 = 全速正转   ↓/s 按住 = 全速反转   空格 = @STOP   q = 退出")

    fd = sys.stdin.fileno()
    import termios
    import tty
    import select
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    os.set_blocking(fd, False)

    buf = b""
    v_hold = 0.0
    hold_until = 0.0
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    data = b""
                if data:
                    buf += data
            events, buf = parse_events(buf)
            for kind, val in events:
                if kind == "ch" and val in (b"q", b"\x03"):
                    raise SystemExit(0)
                nv, nu = next_v(v_hold, kind, val, time.time(), a.v)
                if nu is not None:
                    if kind == "ch" and val == b" ":
                        car.set(0.0)
                        car.cmd("@VEL 0 0")
                        car.cmd("@STOP")     # 空格额外清速度环
                        v_hold, hold_until = 0.0, 0.0
                    else:
                        car.set(nv)
                        v_hold, hold_until = nv, nu
            # 松手判定：超过 HOLD_LOCK_S 没有新的方向键事件
            if v_hold != 0.0 and time.time() > hold_until:
                car.set(0.0)
                v_hold = 0.0
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        car.close()
        print("\n已退出（已发 @VEL 0 0 + @STOP）")
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        V, L = 690.0, HOLD_LOCK_S
        cases = []
        v, until = next_v(0.0, "arrow", b"A", 10.0, V)
        cases.append(("↑ 按下 → 全速正转", v == V and until == 10.0 + L))
        v2, u2 = next_v(v, "arrow", b"A", 10.5, V)          # 自动重复
        cases.append(("0.5 s 后重复 → 还是 +V 且续期", v2 == V and u2 == 10.5 + L))
        cases.append(("松手 0.8 s → 停", 10.5 + L < 11.3))
        v3, u3 = next_v(0.0, "arrow", b"B", 12.0, V)
        cases.append(("↓ 按下 → 全速反转", v3 == -V and u3 == 12.0 + L))
        v4, _ = next_v(-V, "arrow", b"C", 12.1, V)          # ←/→ 不管
        cases.append(("←/→ → 不改速度", v4 == -V))
        v5, u5 = next_v(-V, "ch", b" ", 12.2, V)
        cases.append(("空格 → 0 且立即清 hold", v5 == 0.0 and u5 == 0.0))
        v6, _ = next_v(V, "ch", b"w", 13.0, V)
        cases.append(("w → 全速正转", v6 == V))
        for name, ok in cases:
            print("  %s %s" % ("ok  " if ok else "FAIL", name))
        bad = [n for n, ok in cases if not ok]
        print("PASS" if not bad else "FAIL %s" % bad)
        raise SystemExit(0 if not bad else 1)
    raise SystemExit(main())
