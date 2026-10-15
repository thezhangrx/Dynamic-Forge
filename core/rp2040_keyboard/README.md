# `core/rp2040_keyboard/` —— 用 RP2040 把动作号变成键盘按键

> 目标：板子（PS）算出一个动作 → 经串口 → **RP2040 Zero** 模拟 USB 键盘按下方向键
> → 平台里的角色移动。

---

## 1. 两种接法（选一种就行）

板子的控制台是 **`ttyPS0`（PS 的 UART0，MIO 50..51，115200）** —— 启动日志里
`Getty on ttyPS0` 就是它。

### 接法 A：两根线（板子直接说，RP2040 直接听）

RP2040 **只听不说**，所以只要两根线：

| 板子 | RP2040 Zero | 必须 |
|---|---|---|
| 控制台 **TX** | **GP1**（RX） | ✅ |
| **GND** | **GND** | ✅ |
| （RX） | GP0（TX） | ❌ **不要接** |

> ⚠ **不要把 RP2040 的 GP0（TX）接到板子的控制台 RX 上**。板子那颗 CP210x
> 也在驱动同一条 RX，两条线一起推就是**总线争用**。我们只需要单向，所以不接。

**板子这边具体接哪个脚**（仓库里没有原理图，所以按这个办法找）：

板子的控制台**已经接到板上那颗 CP210x**（我们一直用它读启动日志），
所以要接的就是**同一条网络**，从这三处任选一处引：

1. 板上如果有标着 `UART`/`DEBUG`/`CONSOLE` 的 **4 针排针**（GND/TX/RX/VCC），
   用它的 **GND + TX** —— 这是最省事的，优先找这个；
2. 否则找 **CP210x 芯片的 RXD 脚**（它收的就是 PS 的 TX），或那条线上串的
   小电阻的一端，焊一根飞线出来；
3. 再不行就从 **PS 的 MIO50 走线/测试点**上引（MIO50 = UART0 TX）。

**怎么确认找对了**（万用表足够，不必上示波器）：

* 空闲时该线是 **3.3V 高**（UART 空闲电平）；
* 在板子上 `echo hi > /dev/ttyPS0`，该线会抖动一下；开机时它会持续抖动
  （就是一串启动日志）；
* 用 RP2040 先只接 GND、GP1 悬空试：**不接 TX 时键盘永远不该动**；
  接上 TX 后跑 `vision_board_devmem`，串口监视器里就能看到 `@MOVE n`。

**电平安全**：UART0 在 MIO 50..51，属 **MIO Bank0 = LVCMOS 3.3V**
（从移植过来的 PS7 配置里读回确认过），与 RP2040 的 3.3V GPIO 同电平，
不需要电平转换。**不要**把它接到 1.8V 的 Bank1 上（厂商配置里 Bank1 是 1.8V），
也不要把 5V 引到 GP1 上。

### 接法 B：**一根线都不用接**（PC 转发）

板子的控制台**本来就接到板上那颗 CP210x**，而 CP210x 的 USB 是插在 PC 上的 ——
也就是说 **PC 上本来就能看到 `@MOVE` 行**。那就让 PC 转一手：

```
板子(P S) ──控制台──▶ CP210x ──USB──▶ PC ──console_relay.py──▶ RP2040(USB 串口)
                                          └─ 顺带把 HID 键盘发回 PC ─┘
```

RP2040 **只插在 PC 的 USB 上**（同时当键盘），一根线都不接到板子：

```bash
# PC 上（WSL/Windows 都行；需要 pyserial）
python3 core/rp2040_keyboard/console_relay.py --list          # 认出两个串口
python3 core/rp2040_keyboard/console_relay.py \
        --console /dev/ttyUSB0 --rp2040 /dev/ttyACM0          # 开始转发
# 板子上跑（控制台同一时刻只能被一个程序读，所以 PC 那边读的时候别再开 MobaXterm）
./vision_board_devmem --frames 200 --echo-action
```

`code.py` 里 `USE_USB_INPUT = True`（默认）就是为这条路准备的：它除了听
GP1，也听 **USB CDC（`sys.stdin`）**，两条输入共用同一个解析器。
两条都不用时把对应的开关关掉，省得两份输入互相干扰。

串口地址用的是 pyserial 的 URL 形式，所以除了设备名还能跨机：

```bash
# 板子控制台在另一台机器上时
python3 console_relay.py --console socket://192.168.1.5:5000 --rp2040 /dev/ttyACM0
```

> 选哪条：A 更"干净"（PC 不在控制环里，符合"板子当大脑"的形态），但要动板子；
> B 立刻能跑、零风险。**先用 B 把整条链演一遍，再决定要不要做 A。**

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
