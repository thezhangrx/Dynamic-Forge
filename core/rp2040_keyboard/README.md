# `core/rp2040_keyboard/` —— 用 RP2040 把动作号变成键盘按键

> 目标：板子（PS）算出一个动作 → 经串口 → **RP2040 Zero** 模拟 USB 键盘按下方向键
> → 平台里的角色移动。

---

## 1. 为什么是**单向**的（这也是为什么只要两根线）

板子的控制台是 **`ttyPS0`（PS 的 UART0，MIO 50..51，115200）** —— 启动日志里
`Getty on ttyPS0` 就是它。

所以 RP2040 的 GP1 接的正是**板子的串口 TX**，RP2040 **只听不说**：

| 板子 | RP2040 | 必须 |
|---|---|---|
| 串口 TX（控制台） | **GP1**（RX） | ✅ |
| **GND** | **GND** | ✅ |
| （RX） | GP0（TX） | ⬜ 不接也行 |

**电平安全**：UART0 在 MIO 50..51，属 **MIO Bank0 = LVCMOS 3.3V**
（从移植过来的 PS7 配置里读回确认过），与 RP2040 的 3.3V GPIO 同电平，
不需要电平转换。**不要**把它接到 1.8V 的 Bank1 上（厂商配置里 Bank1 是 1.8V）。

## 2. ⚠️ 是 **CircuitPython**，不是 MicroPython

组员给的 `busio.UART(board.GP0, board.GP1, ...)` 是 **CircuitPython** 的写法
（MicroPython 是 `machine.UART`）。这个区别决定成败：

| | USB HID（模拟键盘） |
|---|---|
| 官方 **MicroPython** rp2 端口 | ❌ 不带设备模式 HID，要自己改固件 |
| **CircuitPython** | ✅ 原生 `usb_hid` + `adafruit_hid` |

所以要刷 **CircuitPython 固件**，并把 `adafruit_hid` 库拷到 CIRCUITPY 盘的 `lib/` 下。

## 3. 协议：一个字节 = 一个动作号

动作表**直接复用项目里那份**（`core/bullet_sim/core/actions.py` 的
`DISCRETE_ACTION_NAMES`）—— 那份的顺序本身就是公开协议
（源码注释：*Order is part of the public protocol — never reorder, only append*），
所以不要在两边各写一份自己的映射。

```
0 stay        1 left        2 right       3 up          4 down
5 up_left     6 up_right    7 down_left   8 down_right
```

**线格式**：ASCII 行，`@MOVE <n>\n`。

为什么带前缀：这条线是**共享的控制台**，RP2040 会看到全部内核日志和 shell 提示。
带一个不会在正常日志里出现的 `@MOVE ` 前缀，RP2040 才能只挑出指令、忽略其余。

**为什么用文本而不是裸字节**：控制台里本来就有大量文本，裸字节既没法区分、
也没法人眼检查。文本还能直接 `echo` 手工试：
```sh
echo "@MOVE 1" > /dev/ttyPS0     # 按住左
echo "@MOVE 0" > /dev/ttyPS0     # 松开
```

## 4. ⚠️ 必须有**看门狗**

RP2040 收不到新命令超过 `TIMEOUT_MS` 就**松开所有键**。

理由不是"优雅"，是**安全**：串口断了、板子崩了、脚本退出了，如果键一直按着，
角色会**一直朝那个方向跑**。比赛现场这是最坏的一种失败。

## 5. 板子侧：怎么发

```sh
# 手动试
echo "@MOVE 2" > /dev/ttyPS0

# 从决策层发（伪代码）
#   Decision 在 core/cpu/gap_avoid.py：有 wx / wy（世界系期望速度）和 tier
#   把 (wx, wy) 量化成 0..8 的动作号，再写成一行
```

量化：拿 `DISCRETE_DIRECTIONS`（`core/bullet_sim/core/actions.py`）里 9 个单位方向，
取与 `(wx, wy)` 点积最大的那个；`|(wx,wy)|` 小于死区就当 `stay`（0）。

**注意**：`/dev/ttyPS0` 是**控制台**，往它写会和内核日志/shell 输出交织。所以
* 发之前最好把内核日志级别调低（`dmesg -n 1`），减少噪声；
* 或者以后改用 **UART1（EMIO）** 那条独立的串口 —— 厂商配置里
  `PCW_UART1_UART1_IO = EMIO`，但那需要（a）修好 PS7 里
  `PCW_UART1_PERIPHERAL_ENABLE` 那条应用失败的参数，（b）给 EMIO 分配 PL 引脚。
  **那是更干净的方案，但今天是"先跑通"**。

## 6. 怎么用

1. RP2040 Zero 刷 **CircuitPython** 固件（.uf2 拖到 RPI-RP2 盘）；
2. 把 `adafruit_hid/` 整个目录拷到 CIRCUITPY 盘的 `lib/`；
3. 把 `code.py` 拷到 CIRCUITPY 盘根目录；
4. 接线：板子 TX → GP1，GND → GND；
5. 插到**跑平台的那台电脑**的 USB 上（不是插板子）；
6. 板子上 `echo "@MOVE 1" > /dev/ttyPS0`，看角色是不是往左走。
