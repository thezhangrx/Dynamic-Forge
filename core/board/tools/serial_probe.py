#!/usr/bin/env python3
"""serial_probe.py —— 串口链路体检（E30 配置前先证明"线/口/波特率"是通的）

做三件事：
  1. 被动听 1.5 s：打印收到的任何字节（证明对端有东西发过来）；
  2. 波特率扫描：9600/115200/57600/38400/2400/4800，每档发 C0/C1/C2/C3/AT，
     打印回显（E3x 的配置指令是 ASCII，回显形如 `C0:00 00 1A 17`）；
  3. 环回提示：如果所有波特率都收不到，先做个环回自测。
"""
from __future__ import annotations
import argparse, time
import serial

CMDS = [b"C0\r\n", b"C1\r\n", b"C2\r\n", b"C3\r\n", b"AT\r\n",
        b"C0 00 00 1A 17\r\n"]
BAUDS = [9600, 115200, 57600, 38400, 4800, 2400]


def probe(port: str, baud: int, passive: float = 0.3) -> str:
    try:
        s = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:                                   # noqa: BLE001
        return f"open fail: {e}"
    got = []
    try:
        s.reset_input_buffer()
        time.sleep(passive)
        got += list(s.read(256) or b"")
        for c in CMDS:
            s.reset_input_buffer()
            s.write(c); s.flush()
            time.sleep(0.45)
            got += list(s.read(256) or b"")
    finally:
        s.close()
    return got


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=0, help="只测这一档（默认全扫）")
    a = ap.parse_args()
    bauds = [a.baud] if a.baud else BAUDS
    best = None
    for b in bauds:
        got = probe(a.port, b)
        txt = bytes(got)
        show = txt.decode("utf-8", "ignore").replace("\r", "\\r").replace("\n", "\\n")
        print(f"[{b:6d}] bytes={len(txt):3d}  {show[:120]!r}")
        if len(txt) > len(best[1]) if best else True:
            best = (b, txt)
    print("\n最佳档：", best[0] if best else None, "共", len(best[1]) if best else 0, "字节")
    if not best or not best[1]:
        print("""\n全是 0 字节 ⇒ 按顺序排：\n"""
              """  1) 环回自测：把 USB-TTL 的 TX 和 RX **短接**，再跑一次；\n"""
              """     这一步有回显 ⇒ 适配器+WSL 通，问题在模块侧；没回显 ⇒ 适配器/驱动问题。\n"""
              """  2) 模块 TX/RX 是否交叉（适配器 TX→模块 RXD，适配器 RX→模块 TXD）；\n"""
              """  3) 模块 VCC 是否真有 5V（万用表量，且发射瞬间不塌）；GND 是否共地；\n"""
              """  4) M0/M1 是否真是"配置模式"电平（E3x 一般 M0=1,M1=1）——注意 USB-TTL 的\n"""
              """     DTR/RTS 有时会接到模块的 M0/M1，被串口程序拉低；\n"""
              """  5) 该模块的配置串口波特率可能是**上次被改过**的（不是 9600），全扫一遍。""")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
