# `core/rp2040_keyboard/` —— 用 RP2040 把动作号变成键盘按键

> 目标：板子（PS）算出一个动作 → 经串口 → **RP2040 Zero** 模拟 USB 键盘按下方向键
> → 平台里的角色移动。

---

## 0. 先分清两个角色：**传输** vs **执行**

这两件事经常被混在一起，混了就白买硬件：

* **传输**（板子 → 主机）：板子算完决策把 `@MOVE n` 送出来。走控制台串口，
  或者走 **UDP**（`--keyboard-dev udp://<PC-IP>:9999`，零接线、也不跟控制台日志抢串口）。
* **执行**（决策 → 平台）：把动作号变成平台认的输入。软件注入（`pc_key_driver.py`）
  或真 USB HID 键盘（RP2040）。

UDP 换掉的是**线**，不是**键盘**。键盘这个角色要不要硬件，只取决于一件事：
**平台吃不吃合成按键**。

| # | 做法 | 额外硬件 | 普通窗口程序 | DirectInput/原始输入的游戏 |
|---|---|---|---|---|
| **C** | **PC 上软件注入**（`pc_key_driver.py`） | **无** | ✅ | ❌ 会被忽略 |
| A | RP2040 接板子控制台（两根线，`USE_UART_INPUT`） | RP2040 + 2 线 | ✅ | ✅ 真 HID |
| B | RP2040 插 PC，PC 转发（`console_relay.py --console`） | RP2040 | ✅ | ✅ 真 HID |
| D | RP2040 插 PC，板子 UDP 直发（`console_relay.py --udp`） | RP2040 | ✅ | ✅ 真 HID |

### C 路的动作怎么从板子传到 PC（三种，选一种）

| 传法 | 命令 | 要不要额外条件 |
|---|---|---|
| **SSH 管道**（推荐） | `ssh 板子 '… --keyboard-dev stdout' \| pc_key_driver.py --stdin` | **只要 SSH 通**（已在用） |
| 控制台串口 | `pc_key_driver.py --console /dev/ttyUSB0` | CP210x 要先 `usbipd attach` 到 WSL |
| UDP | `pc_key_driver.py --udp 9999` | ⚠ 见下 |

**推荐 SSH 管道**：它复用已经在用的 SSH 通道，不用接引脚、不用 `usbipd` 挂串口、
也不用改 Windows 防火墙。管道里混着的日志（`帧 3: …`）会被前缀规则自动丢掉。

> ⚠ **UDP 那条在本机当前这套网络下是走不通的**（实测）：板子 ARP 能解析
> `192.168.137.1`，但 ICMP 100% 丢包、UDP 报文也到不了 WSL；而 `9999` 并不在
> Windows 的保留端口段（只有 50000–50059）里，所以原因是 WSL 的 **mirrored
> 网络模式 + Hyper-V 防火墙挡了 LAN→WSL 的入站**（改它要管理员权限）。
> 板子 → Windows 主机方向同理：在 Windows 上 bind 同一端口会 `WSAEACCES`。
> 所以 `--udp` 代码留着（本机自测是通的），但**别把它当成默认路**。

### 先花一分钟判定平台收不收合成按键（不用板子、不用 RP2040）

```bash
# ① 注入这条路通不通？（起一次 PowerShell 注入器，不按任何键）
python3 core/rp2040_keyboard/pc_key_driver.py --probe

# ② 平台收不收？四个方向依次按一遍（跑起来后有 5 秒把平台窗口点成焦点）
python3 core/rp2040_keyboard/pc_key_driver.py --press 1,2,3,4 --arm --delay-s 5
#    平台用 WASD 就换成 --keys wasd；先去掉 --arm 干跑一遍看键位对不对
```

②跑完就退出，不会把键卡住。一次运行能同时回答两件事：
**有方向动了 = 平台收合成按键（RP2040 不用买）**；动的方向和你按的不一样 =
换一套键位（`--keys wasd` / `--keys arrows`）再试；完全不动 = 平台走原始输入。

### 跑闭环

```bash
# 干跑（默认，只打印要按的键，不注入）
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0
# 真注入（先把平台窗口点成焦点！）
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0 --arm
# 零接线：板子直发 UDP，这里收（板子侧 --keyboard-dev udp://<本机IP>:9999）
python3 core/rp2040_keyboard/pc_key_driver.py --udp 9999 --arm
# 平台用 WASD
python3 core/rp2040_keyboard/pc_key_driver.py --console /dev/ttyUSB0 --arm --keys wasd
# 板子那边同时跑
./vision_board_devmem --frames 500 --echo-action
```

**一条命令的完整闭环（SSH 管道，推荐）**——板子在 PL 上跑视觉+决策，动作经
stdout 走 SSH 回来直接注入，PC 上不需要挂任何串口、也不用动防火墙：

```bash
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'cd /root/board_src/build && ./vision_board_devmem --via-pl --frames 0 --keyboard-dev stdout' \
  | python3 core/rp2040_keyboard/pc_key_driver.py --stdin --arm
# 平台用 WASD 就再加 --keys wasd；先去掉 --arm 干跑一遍确认键位对不对
```

> ⚠ `--keyboard-dev stdout` 与 `--stdin` 是**成对**的（同一个协议的两端）。
> 板子侧写 stdout 必须 `fflush`，否则动作会卡在用户态缓冲区里、等进程退出才吐出来
> ——那就不是实时控制了。这点在 `tests/test_pc_key_driver.py` 里钉住了。

**如果 C 能让角色动** → 闭环已经通了，RP2040 可以当成"更真实的 HID"再补。
**如果 C 完全没反应**（角色不动，但 `pc_key_driver` 明明打印了按键）→
平台用的是原始输入，软件注入进不去，那就必须走 **A / B / D**（真 HID 设备）。

> ⚠ C 会真的往**当前焦点窗口**按键。跑之前先把平台窗口点出来，别让按键打进
> 编辑器。所以脚本默认干跑，`--arm` 才注入。
> ⚠ 控制台同一时刻只能被一个程序读 —— 跑 C 的时候别开着 MobaXterm 读 COM3。
> 这也是 `--udp` 存在的意义：板子直发 UDP 时不碰控制台，日志照看。

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

**接法 D：同一个转发，但源改成 UDP**（板子直发，PC 不再去读控制台）：

```bash
# PC 上：收 UDP，转给 RP2040
python3 core/rp2040_keyboard/console_relay.py --udp 9999 --rp2040 /dev/ttyACM0
# 板子上：决策直发 UDP（板子已知 PC 的 IP；WSL 里是 eth3 的 192.168.137.1）
./vision_board_devmem --frames 200 --keyboard-dev udp://192.168.137.1:9999 --echo-action
```

比 B 好在两点：**不占控制台**（MobaXterm 可以继续看日志），以及**不需要
PC 去认哪个 /dev/ttyUSB 是板子**。代价是要通网络（板子 192.168.137.242 ↔ WSL
eth3 192.168.137.1，ICS 已经通了，`ping` 一下即可）。
`--udp` 和 `--console` 二选一，其余行为（解析、看门狗、前缀）完全一致。

> 选哪条：A 更"干净"（PC 不在控制环里，符合"板子当大脑"的形态），但要动板子；
> B/D 立刻能跑、零风险。**先用 B 或 D 把整条链演一遍，再决定要不要做 A。**
> **但只要平台吃合成按键，C 就够用了 —— A/B/D 全部不必做。**

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
