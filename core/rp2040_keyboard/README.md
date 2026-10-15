# `core/rp2040_keyboard/` —— 用 RP2040 把动作号变成键盘按键

> 目标：板子（PS）算出一个动作 → 经串口 → **RP2040 Zero** 模拟 USB 键盘按下方向键
> → 平台里的角色移动。

---

## 0. 三条路（第 0 条**不需要 RP2040**）

板子算完决策都会把动作号打到控制台（`@MOVE n`）。**怎么把它变成按键**有三条路：

| # | 做法 | 额外硬件 | 普通窗口程序 | DirectInput/原始输入的游戏 |
|---|---|---|---|---|
| **C** | **PC 上软件注入**（`pc_key_driver.py`） | **无** | ✅ | ❌ 会被忽略 |
| A | RP2040 接板子控制台（两根线，`USE_UART_INPUT`） | RP2040 + 2 线 | ✅ | ✅ 真 HID |
| B | RP2040 插 PC，PC 转发（`console_relay.py`，`USE_USB_INPUT`） | RP2040 | ✅ | ✅ 真 HID |

**先用 C 把闭环跑起来**（零硬件、零风险），顺带就知道了平台到底吃不吃合成输入：

```bash
# 干跑（默认，只打印要按的键，不注入）
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0
# 真注入（先把平台窗口点成焦点！）
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0 --arm
# 平台用 WASD
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0 --arm --keys wasd
# 板子那边同时跑
./vision_board_devmem --frames 500 --echo-action
```

**如果 C 能让角色动** → 闭环已经通了，RP2040 可以当成"更真实的 HID"再补。
**如果 C 完全没反应**（角色不动，但 `pc_key_driver` 明明打印了按键）→
平台用的是原始输入，软件注入进不去，那就必须走 **A 或 B**（真 HID 设备）。

> ⚠ C 会真的往**当前焦点窗口**按键。跑之前先把平台窗口点出来，别让按键打进
> 编辑器。所以脚本默认干跑，`--arm` 才注入。
> ⚠ 控制台同一时刻只能被一个程序读 —— 跑 C 的时候别开着 MobaXterm 读 COM3。

---

## 0.5 资料核对（`docs/resource/keyboard/`）

那份资料里有：

| 内容 | 文件 | 对这件事的用处 |
|---|---|---|
| **引脚图** | `原理图/RP2040-Zero脚位定义.jpg` | ✅ **直接回答"接哪两个脚"**（上面那张表就是照它写的） |
| RP2040-Zero 原理图 / Pico 引脚图 / 两份 datasheet | `原理图/*.pdf` | 需要查电平、供电、USB 走线时用 |
| **MicroPython 固件** | `micropyth固件/rp2-pico-20220618-v1.19.1.uf2` | ⚠ **v1.19.1 太老，做不了 HID 键盘**（见 §2 的表） |
| 厂商例程 | `测试例程/{blink,hellow,WS2812}.py`、`例程/…WS2812例程/` | 与键盘无关，是"板子 + 工具链能不能跑"的冒烟测试。`RP2040-Zero.py` 用 GP16 驱 WS2812、`main.py` 读 GP24 —— 与引脚图一致 |

**结论**：引脚那一半完全对得上（GP1/GND 可直接照做）；**固件那一半要换** ——
不管走 CircuitPython 还是 MicroPython ≥1.23，都得重新刷一次固件。

两条路都可用，按你们更顺手的选：
* **CircuitPython**：`code.py` 已经写好，`usb_hid` + `adafruit_hid`，本目录的测试都在守它；
* **MicroPython ≥1.23**：留在现有工具链里，用 `machine.USBDevice` +
  [micropython-lib 的 `usb.device`](https://github.com/micropython/micropython-lib/tree/master/micropython/usb)（那里有 keyboard 示例）。
  ⚠ 两个坑：① 运行时 USB 设备要配在 **`boot.py`** 里（它在 USB 子系统初始化之前执行）；
  ② **软复位会清掉运行时 USB 设备**，所以在设备还活着的时候 `mpremote run` 会失败一次
  —— 第二次就好了（官方文档明说）。要 `code.py` 的 MicroPython 版我可以照同一套协议写，
  但**得在硬件上验过才算**（现在没有硬件）。

## 1. 接法 A：板子直接说（两根线）

板子的控制台是 **`ttyPS0`（PS 的 UART0，MIO 50..51，115200）** —— 启动日志里
`Getty on ttyPS0` 就是它。

### 接线与找脚

RP2040 **只听不说**，所以只要两根线：

| 板子 | RP2040 Zero | 必须 |
|---|---|---|
| 控制台 **TX** | **GP1**（RX） | ✅ |
| **GND** | **GND** | ✅ |
| （RX） | GP0（TX） | ❌ **不要接** |

> ⚠ **不要把 RP2040 的 GP0（TX）接到板子的控制台 RX 上**。板子那颗 CP210x
> 也在驱动同一条 RX，两条线一起推就是**总线争用**。我们只需要单向，所以不接。

**RP2040-Zero 这边具体接哪个脚**（依据 `docs/resource/keyboard/原理图/RP2040-Zero脚位定义.jpg`）：

| RP2040-Zero 引脚 | 位置 | 接到 |
|---|---|---|
| **GP1**（= UART0 RX） | 右侧排针第 2 个 | 板子控制台 TX ✅ |
| **GND** | 左侧排针第 2 个（5V 下面） | 板子 GND ✅ |
| 3V3 | 左侧排针第 3 个 | 不用接（RP2040-Zero 由 USB-C 供电） |
| GP0（= UART0 TX） | 右侧排针第 1 个 | **不要接**（见上面的总线争用） |

引脚图右侧那列标的就是 `GP1 UART0 RX` —— 和我们用的脚**一致**。
另外图上写明 **GP16 被板载 WS2812 灯占着**（`测试例程` 里也确实是这么用的），
所以别拿 GP16 接串口。

**板子这边具体接哪个脚**（仓库里没有主板的原理图，所以按这个办法找）：

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

## 2. 接法 B：**一根线都不用接**（PC 转发到 RP2040）

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
| **CircuitPython** | ✅ 原生 `usb_hid` + `adafruit_hid`（本目录 `code.py` 用的就是这条） |
| **MicroPython ≥ 1.23**（2024-06 起） | ✅ `machine.USBDevice` + micropython-lib 的 `usb.device`（有 keyboard 示例） |
| **MicroPython ≤ 1.22**（含资料里给的 v1.19.1） | ❌ **不带**设备模式 HID，必须换固件 |

> ⚠ 这条原来写成"官方 MicroPython 的 rp2 端口不带设备模式 HID"，**按版本限定才对**：
> MicroPython **v1.23.0** 的发布说明写着 *"MicroPython now has full support for
> user-defined USB devices... a higher-level USB library in `micropython-lib`...
> with examples for **keyboard**, mouse, MIDI and serial CDC. This feature is
> currently available on **rp2** and samd ports"*。
> 也就是说 rp2 从 1.23 起就能做键盘了，**不必**为了这个换到 CircuitPython。
>
> 唯一的前提是：**资料里那份固件 `rp2-pico-20220618-v1.19.1.uf2` 是 2022 年的
> v1.19.1，做不了** —— 无论走哪条路都要重新刷固件。

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
