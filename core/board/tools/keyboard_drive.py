#!/usr/bin/env python3
"""keyboard_drive.py —— 用键盘方向键开小车（PC 侧，走 E30 无线链路）

按键
    ←  按一下 = 左转 90°（**两轮都正转**，左轮慢、右轮快 —— 靠差速转）
    →  按一下 = 右转 90°（**两轮都正转**，左轮快、右轮慢）
    ↑  按住  = 往前走（松开 0.7 s 自动停）
    ↓  按住  = 往后走
    f        = 左右对调（打印新的 --turn-sign 值）
    + / -    = 转角 +10° / −10°
    s        = @STOP（停车 + 清速度环，出问题先按它）
    q / Ctrl-C = 退出

只发"现成协议"，固件一行不改
    @MODE 1     纯差速：两轮 = velocity ∓ turn（直立环不参与）
    @VEL v w    v = 线速度 mm/s，w = 转向差速 mrad/s
                两轮同时正转：|turn| < velocity。turn_pwm≈w*840，
                velocity_pwm≈v_mm_s*10（估算）⇒ v=130 / w=600 时两轮都在正转
                （左 796、右 1804 的等效 PWM），差速 504 用来转
    @STOP       停电机 + 清速度环积分（每轮转向前后各发一次：从干净状态起步，
                否则上一轮直行留下的积分会让"两轮都正转"变成贴地跑）

角度怎么闭环
    @ODOM θ 是**编码器里程计**角（5 Hz）。按下方向键后持续发弧线指令，
    |Δθ| 到 目标角度×turn-scale 就停。turn-scale 默认 0.75
    （实测：编码器走 90° ≈ 地面实际 120°，所以只让编码器走 67.5°）。

用法
    python3 keyboard_drive.py
    python3 keyboard_drive.py --turn-sign -1      # 左右对着反了时用它
    python3 keyboard_drive.py --arc-v 0           # 纯原地转（两轮反向）——旧行为
    python3 keyboard_drive.py --deg 120 --turn-scale 0.75
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty

TURN_LOCK_S = 0.80   # 同一次"按住"的续期窗口：必须 > 终端自动重复的首延迟(≈0.5 s)
HOLD_LOCK_S = 0.70   # ↑/↓ 松手判定窗口


class Car:
    """串口 + 10 Hz 心跳 + 里程计 θ 跟踪"""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.v = 0.0          # 当前线速度指令 mm/s
        self.w = 0.0          # 当前角速度指令 mrad/s
        self.theta = None     # 最近一次 @ODOM 的 θ（rad，未展开）
        self.theta_acc = 0.0  # 展开后的累计角（rad）
        self.odom_n = 0
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self._hb, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()

    # ---- 10 Hz 心跳：把当前 (v,w) 一直发给车 ----
    def _hb(self):
        while not self._stop:
            with self._lock:
                v, w = self.v, self.w
            try:
                self.s.write(("@VEL %d %d\n" % (int(round(v)), int(round(w)))).encode())
            except Exception:
                pass
            time.sleep(0.1)

    # ---- 收 @ODOM，维护展开后的 θ ----
    def _rx(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.s.read(4096)
            except Exception:
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                f = line.split()
                if len(f) < 4 or f[0] != b"@ODOM":
                    continue
                try:
                    th = int(f[3]) / 1000.0     # mrad -> rad
                except ValueError:
                    continue
                with self._lock:
                    if self.theta is None:
                        self.theta = th
                    else:
                        d = th - self.theta
                        if d > 3.14159265:
                            d -= 6.28318531
                        elif d < -3.14159265:
                            d += 6.28318531
                        self.theta_acc += d
                        self.theta = th
                    self.odom_n += 1

    def set(self, v: float, w: float):
        with self._lock:
            self.v, self.w = v, w

    def push(self):
        """把当前 (v,w) 立刻发一遍（不等心跳）"""
        with self._lock:
            v, w = self.v, self.w
        self.cmd("@VEL %d %d" % (int(round(v)), int(round(w))))

    def cmd(self, line: str):
        try:
            self.s.write((line + "\n").encode())
        except Exception:
            pass

    def stop_now(self):
        self.set(0.0, 0.0)
        self.cmd("@STOP")

    def angle_deg(self) -> float:
        with self._lock:
            return self.theta_acc * 57.29578

    def odom_count(self) -> int:
        with self._lock:
            return self.odom_n

    def close(self):
        self.stop_now()
        time.sleep(0.1)
        self._stop = True
        try:
            self.s.close()
        except Exception:
            pass


class TurnJob(threading.Thread):
    """一次转向：发弧线指令 → 盯编码器 θ → 到角度就停。后台跑，主线程继续读键盘。"""

    def __init__(self, car: Car, delta_deg: float, w_cmd: float, v_arc: float,
                 timeout: float):
        super().__init__(daemon=True)
        self.car = car
        self.delta = delta_deg
        self.w_cmd = w_cmd
        self.v_arc = v_arc
        self.timeout = timeout
        self.cancel = False
        self.done = False
        self.reported = False
        self.info = {}

    def run(self):
        car = self.car
        car.cmd("@STOP")            # 清速度环积分，从干净状态起步
        time.sleep(0.06)
        a0 = car.angle_deg()
        n0 = car.odom_count()
        t0 = time.time()
        car.set(self.v_arc, self.w_cmd)
        car.push()                  # 立刻发，不等心跳
        hit = False
        lost = False
        while time.time() - t0 < self.timeout:
            if self.cancel:
                break
            if time.time() - t0 > 0.8 and car.odom_count() == n0:
                lost = True         # @ODOM 没数据 → 闭环瞎了，立刻停，别让它一直拱
                break
            if abs(car.angle_deg() - a0) >= abs(self.delta) - 2.0:
                hit = True
                break
            time.sleep(0.02)
        car.set(0.0, 0.0)
        car.cmd("@VEL 0 0")
        self.info = {
            "cmd_v": self.v_arc, "cmd_w": self.w_cmd, "want": self.delta,
            "a0": a0, "a1": car.angle_deg(), "dt": time.time() - t0,
            "hit": hit, "cancel": self.cancel, "lost": lost,
        }
        self.done = True


def fmt_turn(info: dict) -> str:
    d = info["a1"] - info["a0"]
    if info["hit"]:
        tail = ""
    elif info.get("lost"):
        tail = "(没收到 @ODOM，已中止)"
    elif info["cancel"]:
        tail = "(被打断)"
    else:
        tail = "(超时)"
    return ("  转向: @VEL %d %d  目标 %+.1f°  实测编码器 %+.1f°  用时 %.2fs %s"
            % (info["cmd_v"], info["cmd_w"], info["want"], d, info["dt"], tail))


ARROWS = b"ABCD"


def parse_events(buf: bytes):
    """把终端字节流切成事件，返回 (events, rest)。

    event = ("arrow", b"C") 或 ("ch", b"f")；不完整的转义序列留在 rest 里等下一次。
    ↑↓←→ 既认 \\x1b[A 也认 \\x1bOA（application cursor mode）；
    带参数的转义序列（ESC[1;5C 之类）整条丢掉，不把残留字符漏给按键处理。
    """
    ev = []
    while buf:
        if buf[0:1] != b"\x1b":
            ev.append(("ch", buf[0:1]))
            buf = buf[1:]
            continue
        if len(buf) < 2:
            break                       # 只有 ESC，等后续字节
        if buf[1:2] == b"O":            # SS3（application cursor）
            if len(buf) < 3:
                break
            if buf[2:3] in ARROWS:
                ev.append(("arrow", buf[2:3]))
            buf = buf[3:]
            continue
        if buf[1:2] == b"[":            # CSI：扫到终止字节 (0x40..0x7e)
            i = 2
            while i < len(buf) and not (0x40 <= buf[i] <= 0x7E):
                i += 1
            if i >= len(buf):
                break                   # 还没收完
            if i == 2 and buf[i:i + 1] in ARROWS:
                ev.append(("arrow", buf[i:i + 1]))
            buf = buf[i + 1:]
            continue
        buf = buf[1:]                   # 不认识的转义：丢掉 ESC
    return ev, buf


class Edge:
    """方向键的"按下沿"判定（终端 raw 模式**没有 key-up 事件**）。

    实测的终端行为：按下 → 约 0.5 s 静默 → 每 ~40 ms 自动重复一次，直到松开。
    所以不能只看"两次事件的间隔"（那 0.5 s 静默会被误当成新的按下 → 一次按键转两下）。

    做法：**锁定期由每个事件续期**
        new = (now >= lock)     # 距上一次事件超过了 lock 窗口 = 真的新按下
        lock = now + lock_s     # 任何事件都续期
    按住期间事件不断续期 → 永远只算一次；松开超过 lock_s 后的第一个事件 = 新按下。
    """

    def __init__(self, lock_s: float = TURN_LOCK_S):
        self.lock_s = lock_s
        self.lock = 0.0

    def feed(self, now: float) -> bool:
        new = now >= self.lock
        self.lock = now + self.lock_s
        return new

    def rearm(self, now: float) -> None:
        """转完之后把窗口重新推一格（吞掉残留的重复事件）"""
        self.lock = max(self.lock, now + self.lock_s)


def turn_cmd(seq: bytes, deg: float, scale: float, tsign: int, w: float):
    """←/→ 键 → (编码器目标角 delta_deg, 角速度指令 w_mrad_s)。

    C = →（右转）  D = ←（左转）；tsign = -1 时整体对调。
    scale 把"地面角度"换成"编码器角度"（实测编码器 90° ≈ 地面 120° ⇒ 0.75）。
    """
    d = 1.0 if seq == b"C" else -1.0
    return d * deg * scale * tsign, -d * w * tsign


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--v", type=float, default=150.0, help="按住 ↑/↓ 的线速度 mm/s")
    ap.add_argument("--arc-v", type=float, default=130.0,
                    help="转向时两轮共同的前进速度 mm/s（默认 130 = 两轮都正转靠差速转；"
                         "0 = 两轮反向原地转）")
    ap.add_argument("--w", type=float, default=600.0, help="转向差速指令 mrad/s")
    ap.add_argument("--deg", type=float, default=90.0, help="按一下转多少度")
    ap.add_argument("--turn-scale", type=float, default=0.75,
                    help="编码器角→地面角的换算（实测编码器 90°≈地面 120°，故 0.75）")
    ap.add_argument("--turn-sign", type=int, default=1, choices=(1, -1),
                    help="左右整体对调：+1（默认）/-1；运行中按 f 也能翻")
    ap.add_argument("--timeout", type=float, default=5.0, help="单次转向最长时间 s")
    ap.add_argument("--no-mode", action="store_true", help="不切 @MODE 1（保持自平衡）")
    ap.add_argument("--log", default="", help="把每次转向的数字追加写进这个文件")
    a = ap.parse_args()

    car = Car(a.port, a.baud)
    if not a.no_mode:
        car.cmd("@STOP")
        time.sleep(0.05)
        car.cmd("@MODE 1")        # 纯差速
        time.sleep(0.30)

    tsign = a.turn_sign
    deg = a.deg
    logf = open(a.log, "a", buffering=1) if a.log else None

    print("就绪（@MODE 1 纯差速）")
    print("  ← 左转 / → 右转：两轮都正转，靠差速转；每次 %g°（编码器 %g°）"
          % (deg, deg * a.turn_scale))
    print("  ↑ 按住前进（%g mm/s）  ↓ 按住后退   q 退出" % a.v)
    print("  f 左右对调   +/− 转角 ±10°   s @STOP 停车")
    print("  提示：一次只转一次 —— 松开约 0.8 s 再按（终端没有 key-up 事件，"
          "靠这个窗口区分“按住”和“再按一下”）")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    os.set_blocking(fd, False)

    buf = b""
    edge = Edge(TURN_LOCK_S)
    hold_until = 0.0          # ↑/↓ 的松手判定
    v_hold = 0.0
    job = None

    def arrow(seq: bytes, now: float) -> None:
        nonlocal hold_until, v_hold, job, tsign, deg
        is_new = edge.feed(now)
        if seq == b"A":                      # ↑
            v_hold = a.v
            hold_until = now + HOLD_LOCK_S
            car.set(a.v, 0.0)
        elif seq == b"B":                    # ↓
            v_hold = -a.v
            hold_until = now + HOLD_LOCK_S
            car.set(-a.v, 0.0)
        else:                                # C=右 D=左
            v_hold = 0.0
            car.set(0.0, 0.0)
            if job is not None and not job.done:
                return                       # 正在转，只续期，不叠加
            if not is_new:
                return                       # 同一次按住的自动重复
            d = 1.0 if seq == b"C" else -1.0
            delta, wcmd = turn_cmd(seq, deg, a.turn_scale, tsign, a.w)
            job = TurnJob(car, delta, wcmd, a.arc_v, a.timeout)
            job.start()
            print("  → 开始%s转 %.0f°：@VEL %d %d"
                  % ("右" if d > 0 else "左", abs(delta / a.turn_scale),
                     a.arc_v, int(wcmd)))

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
                if kind == "arrow":
                    arrow(val, time.time())
                    continue
                ch = val
                if ch in (b"q", b"\x03"):
                    raise SystemExit(0)
                if ch == b"f":
                    tsign = -tsign
                    print("  f: turn-sign 现在 = %+d" % tsign)
                elif ch in (b"+", b"="):
                    deg = min(180.0, deg + 10.0)
                    print("  +: 每次转 %.0f°" % deg)
                elif ch in (b"-", b"_"):
                    deg = max(10.0, deg - 10.0)
                    print("  -: 每次转 %.0f°" % deg)
                elif ch == b"s":
                    if job is not None and not job.done:
                        job.cancel = True
                    car.stop_now()
                    v_hold = 0.0
                    print("  s: @STOP")

            # 长按 ↑/↓ 松手
            if v_hold != 0.0 and time.time() > hold_until:
                v_hold = 0.0
                car.set(0.0, 0.0)

            # 转向收尾
            if job is not None and job.done and not job.reported:
                job.reported = True
                print(fmt_turn(job.info))
                if logf is not None:
                    logf.write("%.3f %s\n" % (time.time(),
                                              " ".join("%s=%s" % (k, job.info[k])
                                                       for k in sorted(job.info))))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if job is not None and not job.done:
            job.cancel = True
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        car.close()
        if logf is not None:
            logf.close()
        print("\n已退出（已发 @STOP）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
