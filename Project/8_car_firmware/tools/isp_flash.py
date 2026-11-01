#!/usr/bin/env python3
"""STM32F103 串口 ISP 烧写（AN3155 / USART bootloader）

不需要 ST-Link：只要把 **BOOT0 置 1**、复位，芯片就进入内置 bootloader，
用一路 USB-TTL 接 USART1(PA9/PA10) 就能擦、写、校验。

接线（STM32F103C8T6 核心板）：
    USB-TTL TX  ->  PA10 (USART1_RX)
    USB-TTL RX  ->  PA9  (USART1_TX)
    USB-TTL GND ->  GND
    （板子自己供电；不要把 USB-TTL 的 3V3/5V 和板子电源硬并在一起）

跳线：
    BOOT0 = 1   BOOT1 = 0      ← 进 bootloader
    BOOT0 = 0   BOOT1 = 0      ← 正常跑程序（烧完改成这个再复位）

用法：
    python3 isp_flash.py --port /dev/ttyUSB0 --bin build/car.bin
    python3 isp_flash.py --port /dev/ttyUSB0 --check          # 只握手 + 读 ID
    python3 isp_flash.py --port /dev/ttyUSB0 --bin ... --no-verify

串口参数固定 **115200 8E1**（bootloader 要求偶校验，波特率自适应）。
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：pip3 install pyserial")

ACK = 0x79
NACK = 0x1F

CMD_GET = 0x00
CMD_GET_ID = 0x02
CMD_READ = 0x11
CMD_GO = 0x21
CMD_WRITE = 0x31
CMD_ERASE = 0x43

FLASH_BASE = 0x08000000

# STM32F103 的 PID 表（bootloader 读出来的 2 字节）
PID_NAMES = {
    0x0410: "STM32F10x Medium-density (C8T6 等)",
    0x0414: "STM32F10x High-density",
    0x0412: "STM32F10x Low-density",
    0x0430: "STM32F10x XL-density",
}


def xor_cksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


class Bootloader:
    def __init__(self, port, baud=115200, timeout=2.0):
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- 基础收发 ----
    def _read_exact(self, n):
        data = self.ser.read(n)
        if len(data) != n:
            raise TimeoutError("期望 %d 字节，只收到 %d（%s）" % (n, len(data), data.hex()))
        return data

    def _expect_ack(self, what):
        b = self._read_exact(1)[0]
        if b == ACK:
            return
        if b == NACK:
            raise RuntimeError("%s 收到 NACK" % what)
        raise RuntimeError("%s 收到未知响应 0x%02X" % (what, b))

    def _send(self, data):
        self.ser.write(bytes(data))
        self.ser.flush()

    def _cmd(self, code, what):
        self._send([code])
        self._expect_ack(what)

    # ---- 协议 ----
    def sync(self, tries=3):
        """发 0x7F 进 bootloader；必须在 BOOT0=1 复位后的 1~2 秒内。"""
        last = None
        for i in range(tries):
            self.ser.reset_input_buffer()
            self._send([0x7F])
            try:
                self._expect_ack("SYNC")
                return True
            except (TimeoutError, RuntimeError) as e:
                last = e
                time.sleep(0.3)
        raise RuntimeError(
            "握手失败（%s）。检查：BOOT0 是否为 1、复位过没有、TX/RX 是否交叉、"
            "是否 8E1 115200、串口有没有被别的程序占用" % last)

    def cmd_get(self):
        self._cmd(CMD_GET, "GET")
        n = self._read_exact(1)[0]
        payload = self._read_exact(n + 1)      # 版本号 + N 个命令
        self._expect_ack("GET(结束)")
        ver, cmds = payload[0], payload[1:]
        return ver, cmds

    def cmd_get_id(self):
        self._cmd(CMD_GET_ID, "GET_ID")
        n = self._read_exact(1)[0]
        payload = self._read_exact(n + 1)
        self._expect_ack("GET_ID(结束)")
        pid = (payload[0] << 8) | payload[1]
        return pid

    def cmd_erase_all(self):
        self._cmd(CMD_ERASE, "ERASE")
        self._send([0xFF, 0x00])               # 0xFF = 全片擦除
        self._expect_ack("ERASE(全局)")
        # 全片擦除耗时较长，给足时间
        time.sleep(1.0)
        self.ser.reset_input_buffer()

    def cmd_write(self, addr, data):
        assert 1 <= len(data) <= 256
        self._cmd(CMD_WRITE, "WRITE")
        a = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF])
        self._send(list(a) + [xor_cksum(a)])
        self._expect_ack("WRITE(地址)")
        n = len(data) - 1
        self._send([n, n ^ 0xFF])
        self._expect_ack("WRITE(长度)")
        self._send(list(data) + [xor_cksum(data)])
        self._expect_ack("WRITE(数据)")

    def cmd_read(self, addr, n):
        self._cmd(CMD_READ, "READ")
        a = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF])
        self._send(list(a) + [xor_cksum(a)])
        self._expect_ack("READ(地址)")
        cnt = n - 1
        self._send([cnt, cnt ^ 0xFF])
        self._expect_ack("READ(长度)")
        return self._read_exact(n)

    def cmd_go(self, addr=FLASH_BASE):
        self._cmd(CMD_GO, "GO")
        a = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF])
        self._send(list(a) + [xor_cksum(a)])
        self._expect_ack("GO")


def main():
    ap = argparse.ArgumentParser(description="STM32F103 串口 ISP 烧写")
    ap.add_argument("--port", required=True, help="例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bin", help="要烧的 .bin（从 0x08000000 开始）")
    ap.add_argument("--check", action="store_true", help="只握手/读 ID，不擦不写")
    ap.add_argument("--no-verify", action="store_true", help="跳过回读校验")
    ap.add_argument("--no-go", action="store_true", help="写完不跳转")
    args = ap.parse_args()

    try:
        bl = Bootloader(args.port, args.baud)
    except serial.SerialException as e:
        sys.exit("打不开串口 %s：%s" % (args.port, e))

    try:
        print("== 握手 ==")
        bl.sync()
        print("  OK")

        ver, cmds = bl.cmd_get()
        print("== bootloader 版本 %.1f，支持命令 %d 个 ==" % (ver, len(cmds)))
        need = [CMD_WRITE, CMD_ERASE, CMD_GO]
        missing = [c for c in need if c not in cmds]
        if missing:
            print("  !! 缺少必需命令：%s" % ["0x%02X" % c for c in missing])

        pid = bl.cmd_get_id()
        print("== 芯片 PID 0x%04X  %s ==" % (pid, PID_NAMES.get(pid, "未知")))

        if args.check or not args.bin:
            print("（--check 模式，到此结束）")
            return

        data = open(args.bin, "rb").read()
        if not data:
            sys.exit("bin 文件是空的")
        print("== 固件 %s，%d 字节 ==" % (args.bin, len(data)))

        print("== 全片擦除 ==")
        bl.cmd_erase_all()
        print("  OK")

        print("== 写入 0x%08X ==" % FLASH_BASE)
        off = 0
        t0 = time.time()
        while off < len(data):
            chunk = data[off:off + 256]
            if len(chunk) < 256:                  # 补齐到 4 字节对齐
                pad = (-len(chunk)) % 4
                chunk = chunk + b"\xFF" * pad
            bl.cmd_write(FLASH_BASE + off, chunk)
            off += len(chunk)
            if off % 4096 == 0 or off >= len(data):
                print("  %6d / %d 字节  (%.1fs)" % (min(off, len(data)), len(data), time.time() - t0))

        if not args.no_verify:
            print("== 回读校验 ==")
            bad = 0
            off = 0
            while off < len(data):
                n = min(256, len(data) - off)
                back = bl.cmd_read(FLASH_BASE + off, n)
                if back != data[off:off + n]:
                    bad += 1
                    print("  偏移 0x%X 不一致" % off)
                off += n
            if bad:
                sys.exit("校验失败：%d 个块不一致" % bad)
            print("  全部一致")

        if not args.no_go:
            print("== 跳转到 0x%08X ==" % FLASH_BASE)
            bl.cmd_go(FLASH_BASE)
            print("  OK（程序已开始运行）")

        print("\n完成。记得把 BOOT0 改回 0，再复位一次，之后上电才会正常跑程序。")
    finally:
        bl.close()


if __name__ == "__main__":
    main()
