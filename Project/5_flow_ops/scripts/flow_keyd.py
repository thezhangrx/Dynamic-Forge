#!/usr/bin/env python3
"""flow_keyd.py —— 板端按键 / 指示灯服务（PS_KEY = PS_MIO47）

    PS_KEY(MIO47) 按一下  → /run/flow/press 计数 +1   （PC 侧轮询它来"开始/停止全流程"）
    /run/flow/run  写 1   → 点亮板上指示灯；写 0 → 熄灭（**由 PC 写**，代表流程真的在跑）

为什么把"灯"跟"run"文件绑而不是跟"按过键"绑：
    灯要表示"全流程现在在跑"，而不是"按过一次"。PC 侧只有在真的拉起
    loop_standard.sh 之后才写 run=1，停止并收尾完才写 run=0 —— 所以灯亮 ⇔ 流程在跑；
    流程自己挂了 PC 那边也会写 0，灯会灭。

GPIO 号（Zynq PS GPIO，4 个 bank 由 DT 固定）：
    MIO 0..31  → base 480      MIO 32..53 → base 458
    EMIO 0..31 → base 426      EMIO 32..63 → base 394
    所以 MIO47 = 458 + (47-32) = **473**（--key-gpio 473，默认就是它）

用法（板上，root）：
    python3 flow_keyd.py --key-gpio 473 --led-gpio -1          # 只测按键
    python3 flow_keyd.py --key-gpio 473 --led-gpio 474         # 连灯一起
    cat /run/flow/press    # 按键计数
    echo 1 > /run/flow/run # 手动点亮（自测用）
装机：见同目录 flow_keyd.service（systemd，开机自启）
"""
from __future__ import annotations

import argparse
import os
import sys
import time

GPIO = "/sys/class/gpio"
RUNDIR = "/run/flow"


def p(n):
    return "%s/gpio%d" % (GPIO, n)


def export(n: int) -> bool:
    if os.path.isdir(p(n)):
        return True
    try:
        with open(GPIO + "/export", "w") as f:
            f.write(str(n))
    except OSError as e:
        print("export %d 失败: %s（可能被驱动占用）" % (n, e), file=sys.stderr)
        return False
    for _ in range(100):
        if os.path.isdir(p(n)):
            return True
        time.sleep(0.01)
    return False


def direction(n: int, d: str) -> None:
    try:
        with open(p(n) + "/direction", "w") as f:
            f.write(d)
    except OSError:
        pass            # 输入脚有些内核不允许改方向，忽略


def read(n: int) -> int:
    try:
        with open(p(n) + "/value") as f:
            return int(f.read().strip())
    except OSError:
        return -1


def write(n: int, v: int) -> bool:
    try:
        with open(p(n) + "/value", "w") as f:
            f.write("1" if v else "0")
        return True
    except OSError as e:
        print("写 gpio%d=%d 失败: %s" % (n, v, e), file=sys.stderr)
        return False


def write_atomic(path: str, text: str) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--key-gpio", type=int, default=473, help="PS_KEY 的 GPIO 号（MIO47=473）")
    ap.add_argument("--key-active-low", type=int, default=1, help="按下读到 0（默认 1）")
    ap.add_argument("--led-gpio", type=int, default=-1, help="指示灯 GPIO 号；-1 = 不用灯")
    ap.add_argument("--led-active-low", type=int, default=1, help="灯亮 = 输出 0（默认 1）")
    ap.add_argument("--poll", type=float, default=0.01, help="按键扫描周期 s")
    ap.add_argument("--rundir", default=RUNDIR)
    a = ap.parse_args()

    os.makedirs(a.rundir, exist_ok=True)
    press_file = os.path.join(a.rundir, "press")
    run_file = os.path.join(a.rundir, "run")
    if not os.path.exists(press_file):
        write_atomic(press_file, "0\n0\n")
    if not os.path.exists(run_file):
        write_atomic(run_file, "0\n")

    if not export(a.key_gpio):
        return 2
    direction(a.key_gpio, "in")
    key_ok = True
    if a.led_gpio >= 0:
        if not export(a.led_gpio):
            print("灯 GPIO 不可用，按键功能照常", file=sys.stderr)
            a.led_gpio = -1
        else:
            direction(a.led_gpio, "out")
            write(a.led_gpio, 1 if a.led_active_low else 0)     # 灭

    print("flow_keyd: key=gpio%d(active_low=%d) led=%s rundir=%s"
          % (a.key_gpio, a.key_active_low,
             ("gpio%d(low=%d)" % (a.led_gpio, a.led_active_low)) if a.led_gpio >= 0 else "无",
             a.rundir))

    # 读初值：按下 = active_low ? 0 : 1
    raw = read(a.key_gpio)
    if raw < 0:
        print("读不到 key GPIO 的值，退出", file=sys.stderr)
        return 3
    pressed_level = 0 if a.key_active_low else 1
    last = raw
    stable = 0
    n_press = 0
    try:
        with open(press_file) as f:
            n_press = int((f.read().split() or ["0"])[0])
    except (OSError, ValueError):
        n_press = 0

    led_state = None
    run_state = 0
    last_run_check = 0.0
    last_led_write = 0.0
    # 去抖：同一电平连续 4 次采样（≈40 ms）才认；只在"稳定到按下电平"那一次计数
    last_stable = raw
    pending = raw
    pending_n = 0
    print("flow_keyd: 就绪，raw=%d press=%d" % (raw, n_press))

    while True:
        raw = read(a.key_gpio)
        if raw >= 0:
            if raw != pending:
                pending = raw
                pending_n = 1
            elif pending_n < 4:
                pending_n += 1
            if pending_n >= 4 and pending != last_stable:
                last_stable = pending
                if pending == pressed_level:
                    n_press += 1
                    write_atomic(press_file, "%d\n%.3f\n" % (n_press, time.time()))
                    print("flow_keyd: PS_KEY 按下 -> press=%d" % n_press)

        # 灯：看 PC 写的 run 文件（每 100 ms 一次就够）
        now = time.time()
        if now - last_run_check > 0.1:
            last_run_check = now
            try:
                with open(run_file) as f:
                    run_state = 1 if f.read().strip().startswith("1") else 0
            except OSError:
                run_state = 0
            if a.led_gpio >= 0:
                on = (0 if a.led_active_low else 1)
                off = (1 if a.led_active_low else 0)
                want = on if run_state else off
                # 变化时写；另外每 1 s 无条件重写一次（自愈：引脚被别人 poke 过/写丢）
                if want != led_state or now - last_led_write > 1.0:
                    ok = write(a.led_gpio, want)
                    last_led_write = now
                    if want != led_state:
                        led_state = want
                        print("flow_keyd: run=%d -> 灯%s%s"
                              % (run_state, "亮" if run_state else "灭",
                                 "" if ok else "（写入失败！）"))
        time.sleep(a.poll)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
