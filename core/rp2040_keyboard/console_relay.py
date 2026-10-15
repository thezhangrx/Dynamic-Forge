#!/usr/bin/env python3
"""把板子控制台上的 `@MOVE` 行转发到 RP2040 的 USB 串口 —— **零接线方案**。

## 为什么需要它

板子的控制台（`ttyPS0`）本来就接到了**板上那颗 CP210x** 上 —— 我们一直用它读
启动日志。也就是说 **PC 上本来就能看到 `@MOVE` 行**。所以除了"从板子引两根线到
RP2040"（见 `code.py` 的 `USE_UART_INPUT`）之外，还有这条路：

    PC: 读板子控制台 (CP210x)  ──挑出 @MOVE 行──▶  RP2040 的 USB 串口 (CDC)
    RP2040: 收指令 ──▶ USB HID 方向键 ──▶ PC 上的平台角色

**RP2040 一根线都不用接到板子上**，它只插在 PC 的 USB 上（同时充当键盘）。
代价是 PC 上要跑这个脚本。

两条路在 RP2040 侧是同一个解析器（`code.py` 的 `feed()`），所以行为一致。

## 用法

    # 列出候选串口（找板子控制台和 RP2040 分别对应哪个）
    python3 console_relay.py --list

    # 转发（板子控制台通常是 CP210x；RP2040 在 CircuitPython 下是 CDC）
    python3 console_relay.py --console /dev/ttyUSB0 --rp2040 /dev/ttyACM0

    # 先干跑不写串口，只看它挑出了什么
    python3 console_relay.py --console /dev/ttyUSB0 --dry-run

需要 `pyserial`（`pip install pyserial`）。板子侧建议先 `dmesg -n 1` 压掉内核日志，
线上会干净很多 —— 不过解析器对噪声是免疫的，不压也不会出错，只是白读。
"""

from __future__ import annotations

import argparse
import sys

PREFIX = b"@MOVE "                 # 与 action_codec.hpp / code.py 必须一致
MAX_N = 8                          # DISCRETE_ACTION_NAMES 的下标上界
_BUF_LIMIT = 4096                  # 半行残渣别无限涨（控制台日志量可能很大）


def parse_moves(data: bytes, buf: bytearray) -> list[int]:
    """从控制台字节流里挑出 `@MOVE <n>` 行，返回动作号列表。

    `buf` 是**跨批次**的半行残渣（就地修改）。纯函数，不碰串口 ——
    所以这一层可以脱机测试（`tests/test_console_relay.py`）。

    规则与 RP2040 侧 `code.py` 的 `feed()` 逐条对应，两边不许分家：
      * 只按 `\\n` 拆行，半行留到下一批；
      * 只认带 `@MOVE ` 前缀的行（这条线是**共享的控制台**，混着内核日志）；
      * 前缀后必须是整数且在 0..8 内，否则**静默丢弃** —— 宁可不动，
        也不能让噪声把键按错；
      * `\\r` 由 `strip()` 去掉（串口常见的 CRLF）。
    """
    out: list[int] = []
    buf.extend(data)
    while b"\n" in buf:
        line, _, rest = buf.partition(b"\n")
        buf[:] = rest
        line = bytes(line).strip()
        if not line.startswith(PREFIX):
            continue
        try:
            n = int(line[len(PREFIX):])
        except ValueError:
            continue
        if not 0 <= n <= MAX_N:
            continue
        out.append(n)
    if len(buf) > _BUF_LIMIT:
        del buf[:]                     # 残渣太多就丢掉，别把内存吃光
    return out


def _open_serial(port: str, baud: int):
    """打开一个串口。

    用 `serial_for_url` 而不是 `Serial`：这样除了设备名（`/dev/ttyUSB0`、`COM3`）
    还能用 URL 形式，比如 `socket://192.168.1.5:5000`（板子控制台在另一台机器上时）
    或 `loop://`。**也让这条转发逻辑可以脱机冒烟测试** —— 本环境没有可用 PTY，
    就用 `socket://` 起一个假控制台来验真串口这条路（不只是验纯解析函数）。
    """
    try:
        import serial                    # pyserial
    except ImportError:
        sys.exit("需要 pyserial：pip install pyserial")
    return serial.serial_for_url(port, baud, timeout=0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="板子的 @MOVE 行 → RP2040 USB 串口 转发")
    ap.add_argument("--console", help="板子控制台串口（CP210x，如 /dev/ttyUSB0）")
    ap.add_argument("--udp", type=int, metavar="PORT",
                    help="改成**收 UDP**（板子用 --keyboard-dev udp://本机IP:PORT 直发）。"
                         "不需要接任何线，也不和控制台日志抢串口")
    ap.add_argument("--rp2040", help="RP2040 的 USB 串口（CDC，如 /dev/ttyACM0）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true", help="只打印挑出的动作，不写串口")
    ap.add_argument("--list", action="store_true", help="列出候选串口后退出")
    args = ap.parse_args()

    if args.list:
        try:
            from serial.tools import list_ports
        except ImportError:
            sys.exit("需要 pyserial：pip install pyserial")
        for p in list_ports.comports():
            print(f"  {p.device}  {p.description}")
        return 0

    if not args.console and args.udp is None:
        ap.error("要么给 --list，要么给 --console / --udp（和 --rp2040）")
    if not args.rp2040 and not args.dry_run:
        ap.error("需要 --rp2040（或者用 --dry-run 只看不写）")

    rp = None if args.dry_run else _open_serial(args.rp2040, args.baud)
    buf = bytearray()
    n_sent = 0

    if args.udp is not None:
        # ---- UDP 源：板子直发（--keyboard-dev udp://...）----
        import socket as _socket
        sk = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        sk.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        sk.bind(("0.0.0.0", args.udp))
        print(f"UDP :{args.udp} → {'(dry-run)' if rp is None else args.rp2040}"
              f"   前缀 {PREFIX!r}   Ctrl-C 退出", flush=True)
        try:
            while True:
                data, _peer = sk.recvfrom(512)
                for n in parse_moves(data, buf):
                    line = PREFIX + str(n).encode("ascii") + b"\n"
                    if rp is not None:
                        rp.write(line)
                    n_sent += 1
                    print(f"  → {line!r}  (累计 {n_sent})", flush=True)
        except KeyboardInterrupt:
            print(f"\n转发 {n_sent} 行后退出")
        finally:
            sk.close()
            if rp is not None:
                rp.close()
        return 0

    con = _open_serial(args.console, args.baud)
    print(f"控制台 {args.console} → {'(dry-run)' if rp is None else args.rp2040}"
          f"   前缀 {PREFIX!r}   Ctrl-C 退出", flush=True)
    try:
        while True:
            chunk = con.read(256)
            if not chunk:
                continue
            for n in parse_moves(chunk, buf):
                line = PREFIX + str(n).encode("ascii") + b"\n"
                if rp is not None:
                    rp.write(line)
                n_sent += 1
                print(f"  → {line!r}  (累计 {n_sent})", flush=True)
    except KeyboardInterrupt:
        print(f"\n转发 {n_sent} 行后退出")
    finally:
        con.close()
        if rp is not None:
            rp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
