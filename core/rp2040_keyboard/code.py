# ===========================================================================
# code.py —— RP2040 Zero（CircuitPython）：串口收动作号 → 模拟 USB 键盘方向键
#
# 数据流：
#     板子 PS 的控制台 ttyPS0 ──(TX → GP1)──► 本程序 ──USB HID──► 跑平台的电脑
#
# 协议：ASCII 行 `@MOVE <n>\n`，n = 0..8，就是
#       core/bullet_sim/core/actions.py 的 DISCRETE_ACTION_NAMES 下标。
#       前缀是必需的：这条线是**共享的控制台**，会混着内核日志和 shell 输出，
#       靠前缀才能只挑出指令。
#
# 依赖（拷到 CIRCUITPY 盘的 lib/ 下）：
#     adafruit_hid/
# 固件：CircuitPython（**不是** MicroPython —— 官方 rp2 端口不带 USB HID 设备模式）
#
# 为什么要有看门狗：串口断了 / 板子崩了 / 脚本退出了，如果键一直按着，
# 角色会**一直朝那个方向跑**。比赛现场这是最坏的一种失败，所以超时就全松。
# ===========================================================================
import time

import board
import busio
import usb_hid
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode

# ---- 配置 -----------------------------------------------------------------
BAUDRATE   = 115200          # 与板子 UART0 一致（PS7 配置里 PCW_UART0_BAUD_RATE）
PREFIX     = b"@MOVE "
TIMEOUT_MS = 500             # 超过这么久没收到**有效**指令 → 松开所有键
DEBUG      = False           # True 时把收到的行打到 USB 串口（要 boot.py 里留 CDC）

# ---- 动作号 → 按键 --------------------------------------------------------
# 下标必须与 core/bullet_sim/core/actions.py 的 DISCRETE_ACTION_NAMES 一致。
# 那份顺序是**公开协议**（源码里写明 never reorder, only append），
# 所以这里照抄，不另立一套。
#
# 平台用的键如果不是方向键，改这一张表就行（WASD 就用 Keycode.W / A / S / D）。
KEYS = {
    0: (),                                          # stay
    1: (Keycode.LEFT_ARROW,),                       # left
    2: (Keycode.RIGHT_ARROW,),                      # right
    3: (Keycode.UP_ARROW,),                         # up
    4: (Keycode.DOWN_ARROW,),                       # down
    5: (Keycode.UP_ARROW, Keycode.LEFT_ARROW),      # up_left
    6: (Keycode.UP_ARROW, Keycode.RIGHT_ARROW),     # up_right
    7: (Keycode.DOWN_ARROW, Keycode.LEFT_ARROW),    # down_left
    8: (Keycode.DOWN_ARROW, Keycode.RIGHT_ARROW),   # down_right
}

# ---- HID ------------------------------------------------------------------
# CircuitPython 默认已经把 KEYBOARD 放进 usb_hid.devices；万一没有就显式开。
if usb_hid.Device.KEYBOARD not in usb_hid.devices:
    usb_hid.enable((usb_hid.Device.KEYBOARD,))

keyboard = Keyboard(usb_hid.devices)

# ---- 串口 -----------------------------------------------------------------
# GP0 = 本机 TX（可不接），GP1 = 本机 RX（← 板子的 TX）
uart = busio.UART(board.GP0, board.GP1, baudrate=BAUDRATE, timeout=0)

_held = ()                   # 当前按住的键（元组，便于比较）
_last_ok = time.monotonic()  # 最近一次收到**有效**指令的时刻
_buf = bytearray()


def apply(action: int) -> None:
    """把按键状态切到 `action` 对应的那一组（只动差集，避免抖动）。"""
    global _held
    want = KEYS.get(action, ())
    if want == _held:
        return
    for k in _held:
        if k not in want:
            keyboard.release(k)
    for k in want:
        if k not in _held:
            keyboard.press(k)
    _held = want
    if DEBUG:
        print("action", action, "->", [hex(k) for k in want])


def release_all(reason: str) -> None:
    global _held
    if _held:
        keyboard.release_all()
        _held = ()
        if DEBUG:
            print("release_all:", reason)


while True:
    chunk = uart.read(64)
    if chunk:
        _buf.extend(chunk)
        # 只按 \n 拆行；半行留在缓冲里等下一批
        while b"\n" in _buf:
            line, _, rest = _buf.partition(b"\n")
            _buf = bytearray(rest)
            line = bytes(line).strip()
            if not line.startswith(PREFIX):
                continue                      # 控制台的其它输出，忽略
            try:
                n = int(line[len(PREFIX):])
            except ValueError:
                continue                      # 不是数字，忽略（别让噪声把键卡住）
            if not 0 <= n <= 8:
                continue                      # 序号越界，忽略
            apply(n)
            _last_ok = time.monotonic()
        # 缓冲别无限涨：控制台日志量大时，半行残渣可能越积越多
        if len(_buf) > 256:
            _buf = bytearray()

    # ---- 看门狗：断流就全松 ----
    if _held and (time.monotonic() - _last_ok) * 1000.0 > TIMEOUT_MS:
        release_all("超时 %.0f ms 没收到有效指令" % TIMEOUT_MS)
