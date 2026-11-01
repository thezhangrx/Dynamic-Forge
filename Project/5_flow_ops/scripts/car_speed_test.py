#!/usr/bin/env python3
"""car_speed_test.py —— 给小车发一段"全速向前"的定时指令，并记录编码器里程

用法（一段一段来）：

    # 1) 发一段 T 秒的全速前进（默认 v=690mm/s → PWM 已到限幅 6900 = 满占空比）
    python3 car_speed_test.py --t 0.4
    python3 car_speed_test.py --t 0.9
    python3 car_speed_test.py --t 1.6

    # 2) 量完距离，把三段 (时长:距离mm) 给我，用同一份数据算 v_max / 加速度 / 减速度
    python3 car_speed_test.py --fit 0.4:310 0.9:720 1.6:1330

    # 3) 自检（合成数据回代，确认拟合器没写错）
    python3 car_speed_test.py --selftest

模型（为什么三个数就能解出三个量）
    电机是电压驱动、没有速度闭环（@MODE 1 是开环前馈），所以
        加速段  v(t) = v_max * (1 - e^(-t/τ))          τ = 电机+车的机械时间常数
        停指令后 自由滑行  s_coast = v(T) * τ_d        τ_d = 摩擦减速时间常数
        总位移  S(T) = v_max*[T - τ(1-e^(-T/τ))] + v_max*(1-e^(-T/τ))*τ_d
    三个不同的 T 三个方程 → 解 (v_max, τ, τ_d)，再由它们换算：
        起步加速度 a0 = v_max/τ，滑行减速度 = v_max/τ_d，匀速速度 = v_max

安全
    * 发完立刻 @VEL 0 0 + @STOP（自由滑行，不是急刹）
    * 车载看门狗 1 s：脉冲期间按 10 Hz 心跳重发，绝不靠单条指令撑 1 s 以上
    * --max-t 默认 3.0 s，超了直接拒绝；先短后长，别一上来就 3 s
"""
from __future__ import annotations

import argparse
import math
import sys
import time

TURN_PWM_PER_RAD_S = None      # 本工具不转向，留空


def send(port, lines, gap=0.06, n=3):
    """同一条指令连发 n 次（无线丢包严重，重复无害）"""
    for _ in range(n):
        for ln in lines:
            port.write((ln + "\n").encode())
        port.flush()
        time.sleep(gap)


def pulse(port, v_mm_s, t_sec, settle=1.6, start_mm=3):
    """发一段 t 秒的 @VEL v 0。

    **起点用编码器判**：先按 10 Hz 一直发指令，等 @ODOM 真的开始动（|Δx|≥start_mm）
    才把那一刻当 t=0，再继续发 t 秒。这样无线那 0.1~0.5 s 的延迟不会算进时长里。
    等 1.5 s 还没动就判"指令没送到"，直接收工（避免量一个假数据）。
    """
    rx = []
    last_x = [None]

    def drain():
        try:
            data = port.read(4096) or b""
        except Exception:
            return
        if not data:
            return
        rx.append((time.time(), data))
        for line in data.split(b"\n"):
            f = line.split()
            if len(f) >= 4 and f[0] == b"@ODOM":
                last_x[0] = int(f[1])

    send(port, ["@STOP", "@MODE 1"], gap=0.08, n=2)
    t_settle = time.time() + 0.35
    while time.time() < t_settle:
        drain()
        time.sleep(0.02)
    x0 = last_x[0]
    if x0 is None:
        return None, None, None, rx, "上电/链路：连 @ODOM 都没有"

    cmd = "@VEL %d 0" % v_mm_s
    send(port, [cmd, cmd], gap=0.05, n=2)
    t_cmd = time.time()
    moving_since = None
    last_hb = time.time()
    while True:
        if time.time() - last_hb >= 0.1:
            port.write((cmd + "\n").encode())
            last_hb = time.time()
        drain()
        now = time.time()
        if last_x[0] is not None and abs(last_x[0] - x0) >= start_mm:
            moving_since = now
            break
        if now - t_cmd > 1.5:
            send(port, ["@VEL 0 0", "@STOP"], gap=0.05, n=3)
            return None, None, None, rx, "1.5 s 内编码器没动：指令没送到或轮子没转"
        time.sleep(0.01)

    t0 = moving_since
    while time.time() - t0 < t_sec:
        if time.time() - last_hb >= 0.1:
            port.write((cmd + "\n").encode())
            last_hb = time.time()
        drain()
        time.sleep(0.01)
    t1 = time.time()
    send(port, ["@VEL 0 0", "@STOP"], gap=0.05, n=3)
    t2 = time.time()
    while time.time() - t2 < settle:
        drain()
        time.sleep(0.02)
    return t0, t1, t2, rx, None


def parse(rx):
    """从收到的字节流里抽出 @ODOM / @STATE，带主机时间戳"""
    out = []
    buf = b""
    for ts, chunk in rx:
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            f = line.split()
            if not f:
                continue
            if f[0] == b"@ODOM" and len(f) >= 4:
                out.append((ts, "ODOM", [int(x) for x in f[1:4]]))
            elif f[0] == b"@STATE" and len(f) >= 5:
                out.append((ts, "STATE", [float(f[1])] + [int(x) for x in f[2:5]]))
    return out


def run_one(a):
    import serial
    port = serial.Serial(a.port, a.baud, timeout=0.05)
    print("端口 %s 已打开" % port.name)
    print("指令：@MODE 1 + @VEL %d 0 持续 %.2f s（PWM≈%d%s）"
          % (a.v, a.t, min(a.v * 10, 6900),
             "，已到限幅" if a.v * 10 >= 6900 else ""))
    t0, t1, t2, rx, err = pulse(port, a.v, a.t)
    port.close()
    if err:
        print("\n⚠ %s" % err)
        print("   （没发成这一段，别量距离；先确认车有电、E30 对通）")
        return 2

    ev = parse(rx)
    odom = [(ts, v[0]) for ts, k, v in ev if k == "ODOM"]
    print("\n时间线（相对第一条 @ODOM）：")
    print("  t(s)   @ODOM.x   备注")
    base = odom[0][0] if odom else t0
    for ts, x in odom:
        tag = ""
        if t0 <= ts <= t1:
            tag = "推动中"
        elif ts > t1:
            tag = "滑行"
        print("  %5.2f  %8d   %s%s" % (ts - base, x, tag,
                                       "  <- 指令结束" if abs(ts - t1) < 0.06 else ""))
    if odom:
        print("\n编码器总位移 Δx = %d mm（%.0f mm；和卷尺比一比，可以看编码器标定差多少）"
              % (odom[-1][1] - odom[0][1], odom[-1][1] - odom[0][1]))
        moving = [x for ts, x in odom if t0 - 0.2 <= ts <= t1 + 0.05]
        if len(moving) >= 2:
            print("推动期间编码器走了 %d mm，用时约 %.2f s（指令 %.2f s）"
                  % (moving[-1] - moving[0], t1 - t0, a.t))
    else:
        print("\n⚠ 没收到 @ODOM：链路丢包或车没上电（本段数据不可用）")
    print("\n指令墙钟：开始 %.3f  结束 %.3f  时长 %.3f s"
          % (t0 % 100, t1 % 100, t1 - t0))
    print(">>> 请用卷尺量“起点→车最终停下的位置”，把 mm 数告诉我；再把车放回原位。")
    return 0


# ---------------- 拟合 ----------------

def model_S(T, v, tau, taud):
    ramp = T - tau * (1.0 - math.exp(-T / tau))
    coast = (1.0 - math.exp(-T / tau)) * taud
    return v * (ramp + coast)


def fit(points, verbose=True):
    """points = [(T, S), ...]（T: 秒，S: mm）→ (v_max, τ, τ_d) + 残差"""
    n = len(points)
    if n < 3:
        raise SystemExit("至少要 3 段不同时长（现在 %d 段）" % n)
    best = None
    taus = [10 ** (-1.6 + 3.2 * i / 159.0) for i in range(160)]      # 0.025 .. 3.16 s
    tauds = [10 ** (-2.0 + 3.0 * j / 119.0) for j in range(120)]     # 0.01 .. 10 s
    for tau in taus:
        A = [T - tau * (1.0 - math.exp(-T / tau)) for T, _ in points]
        E = [1.0 - math.exp(-T / tau) for T, _ in points]
        for taud in tauds:
            B = [A[i] + taud * E[i] for i in range(n)]
            if any(b <= 1e-6 for b in B):
                continue
            v = sum(points[i][1] / B[i] for i in range(n)) / n
            if v <= 0:
                continue
            res = sum((v * B[i] - points[i][1]) ** 2 for i in range(n))
            if best is None or res < best[0]:
                best = (res, v, tau, taud)
    # 局部细化
    res, v, tau, taud = best
    for _ in range(60):
        improved = False
        for dt in (0.9, 1.1):
            for cand in ((tau * dt, taud), (tau, taud * dt)):
                t2, d2 = cand
                if not (0.005 < t2 < 10.0 and 0.002 < d2 < 30.0):
                    continue
                A = [T - t2 * (1.0 - math.exp(-T / t2)) for T, _ in points]
                E = [1.0 - math.exp(-T / t2) for T, _ in points]
                B = [A[i] + d2 * E[i] for i in range(n)]
                if any(b <= 1e-6 for b in B):
                    continue
                v2 = sum(points[i][1] / B[i] for i in range(n)) / n
                if v2 <= 0:
                    continue
                r2 = sum((v2 * B[i] - points[i][1]) ** 2 for i in range(n))
                if r2 < res - 1e-12:
                    res, v, tau, taud = r2, v2, t2, d2
                    improved = True
        if not improved:
            break
    return v, tau, taud, res


def report(points):
    v, tau, taud, res = fit(points)
    print("=== 输入（时长 s : 距离 mm）===")
    for T, S in points:
        print("  %.2f s : %8.1f mm   模型 %.1f mm   残差 %+.1f mm"
              % (T, S, model_S(T, v, tau, taud), model_S(T, v, tau, taud) - S))
    print("\n=== 拟合结果 ===")
    print("  匀速速度        v_max = %.3f m/s  (%.1f mm/s, %.1f cm/s)" % (v / 1000, v, v / 10))
    print("  加速时间常数    τ     = %.3f s   →  0→95%% 用时 %.2f s" % (tau, 3.0 * tau))
    print("  起步加速度      a0    = v/τ = %.2f m/s²   （0→0.5 m/s 需 %.2f s）"
          % (v / 1000 / tau, 0.5 * tau / (v / 1000) if v else 0))
    print("  平均加速度(加速段) = %.2f m/s²" % (v / 1000 / (3.0 * tau)))
    print("  滑行减速度      = v/τ_d = %.2f m/s²   (τ_d = %.3f s)" % (v / 1000 / taud, taud))
    print("  满速滑行距离    = %.0f mm" % (v * taud))
    rms = math.sqrt(res / len(points))
    print("\n  拟合 RMS 残差 = %.1f mm（< 每段距离的 5%% 就算可信）" % rms)
    print("  各段“到达 v_max 的比例”：")
    for T, S in points:
        print("    %.2f s → %.0f%%" % (T, 100.0 * (1.0 - math.exp(-T / tau))))


def selftest():
    v, tau, taud = 480.0, 0.22, 0.35
    pts = [(0.4, round(model_S(0.4, v, tau, taud), 1)),
           (0.9, round(model_S(0.9, v, tau, taud), 1)),
           (1.6, round(model_S(1.6, v, tau, taud), 1))]
    print("自检：真值 v=%.1f mm/s τ=%.2f τd=%.2f" % (v, tau, taud))
    print("      合成距离 =", pts)
    fv, ft, fd, res = fit(pts)
    print("      拟合     v=%.1f mm/s τ=%.2f τd=%.2f  rms=%.2f mm"
          % (fv, ft, fd, math.sqrt(res / 3)))
    ok = abs(fv - v) / v < 0.02 and abs(ft - tau) < 0.03 and abs(fd - taud) < 0.05
    print("      %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def parse_points(args):
    pts = []
    for s in args:
        try:
            T, S = s.split(":")
            pts.append((float(T), float(S)))
        except ValueError:
            raise SystemExit("格式要是 时长:距离，比如 0.4:310（收到 %r）" % s)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--t", type=float, default=0.4, help="这一段的持续时间 s")
    ap.add_argument("--v", type=int, default=150,
                    help="线速度指令 mm/s（默认 150，和键盘 ↑ 前进一致；690 = PWM 满幅，实车会打滑）")
    ap.add_argument("--max-t", type=float, default=3.0, help="单段最长秒数（安全）")
    ap.add_argument("--fit", nargs="+", metavar="T:S", help="用三段距离算速度/加速度")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if a.fit:
        report(parse_points(a.fit))
        return 0
    if not (0.05 <= a.t <= a.max_t):
        raise SystemExit("--t 要在 0.05..%.1f s 之间（收到 %.2f）" % (a.max_t, a.t))
    return run_one(a)


if __name__ == "__main__":
    raise SystemExit(main())
