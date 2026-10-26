#!/usr/bin/env python3
"""e30_config.py —— E30-433T20D 配置助手（在 WSL 里通过 USB-TTL 跑）

为什么要它：两个 E30 必须配成**完全相同的**信道/地址/空中速率/透明模式，
否则一个发一个不收。这里把"进配置模式 → 读当前参数 → 写参数 → 回读校验"
做成一条命令，避免手抖。

⚠ 硬件前提（模块侧的 M0/M1 是**物理引脚**，USB-TTL 上控制不了）：
   按 E30 手册把 M0/M1 跳到"配置模式"电平（多数 E3x 系列是 M0=1、M1=1），
   配置完再跳回"透明传输"电平（M0=0、M1=0）。跳线不对时模块会把你发的
   指令当业务数据发到空中，命令不会有回显。

用法：
    python3 e30_config.py --port /dev/ttyUSB0 read
    python3 e30_config.py --port /dev/ttyUSB0 write --channel 0x17 --addr 0x0000 \
        --air-rate 9.6k --uart-baud 9600
    python3 e30_config.py --port /dev/ttyUSB0 copy /tmp/e30.json   # 存一份给第二块用
"""
from __future__ import annotations
import argparse, json, time

AIR = {"2.4k": 0, "4.8k": 1, "9.6k": 2, "19.2k": 3, "38.4k": 4, "62.5k": 5}


def open_port(port: str, baud: int):
    import serial  # pyserial
    s = serial.Serial(port, baud, timeout=0.5)
    time.sleep(0.1)
    return s


def cmd(s, line: str, wait: float = 0.6) -> str:
    """发一条配置指令并取回显（E3x 的配置指令都是可打印 ASCII，以 \\r\\n 结尾）。"""
    s.reset_input_buffer()
    s.write(line.encode() + b"\r\n")
    s.flush()
    time.sleep(wait)
    return s.read(512).decode("utf-8", "ignore").strip()


def do_read(s) -> dict:
    out = {}
    for name, c in (("C0", "C0"), ("C1", "C1"), ("C2", "C2"), ("C3", "C3")):
        out[name] = cmd(s, c)
    return out


def do_write(s, ch: int, addr: int, air: int, baud: int) -> dict:
    res = {}
    #: C0：地址(2B) 波特率/校验 空中速率 信道 —— 具体字节含义按手册，这里按 E3x 通用位域拼
    res["C0_set"] = cmd(s, f"C0 00 00 {baud & 0xFF:02X} {baud >> 8:02X} {air:02X} {ch:02X}")
    res["C1_set"] = cmd(s, f"C1 00 00 {ch:02X}")
    res["C0_get"] = cmd(s, "C0")
    res["C1_get"] = cmd(s, "C1")
    res["addr_set"] = cmd(s, f"C2 {addr & 0xFF:02X} {addr >> 8:02X}")
    res["addr_get"] = cmd(s, "C2")
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600, help="配置串口波特率（E3x 默认 9600 8N1）")
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("read")
    w = sub.add_parser("write")
    w.add_argument("--channel", default="0x17")
    w.add_argument("--addr", default="0x0000")
    w.add_argument("--air-rate", default="9.6k", choices=sorted(AIR))
    w.add_argument("--uart-baud", type=int, default=9600)
    c = sub.add_parser("copy")
    c.add_argument("json")
    a = ap.parse_args()

    s = open_port(a.port, a.baud)
    try:
        if a.op == "read":
            print(json.dumps(do_read(s), ensure_ascii=False, indent=1))
        elif a.op == "write":
            res = do_write(s, int(a.channel, 0), int(a.addr, 0), AIR[a.air_rate], a.uart_baud)
            print(json.dumps(res, ensure_ascii=False, indent=1))
        else:
            res = do_read(s)
            open(a.json, "w").write(json.dumps(res, ensure_ascii=False, indent=1))
            print("已保存当前参数：", a.json, "（第二块模块照这份核对）")
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
