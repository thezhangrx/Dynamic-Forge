# 人工操纵 与 开发板操纵（双模式输入）

本平台必须同时支持两种控制模式，且**两者之后的环境逻辑完全一致**：

```
模式 A  人工操纵    Keyboard / Gamepad  ->  Action  ->  Simulator
模式 B  自主控制    Observation         ->  AI Controller -> Action -> Simulator
模式 C  开发板操纵  Development Board   ->  Action  ->  Simulator
                       └ [HARDWARE_INPUT_INTERFACE_TBD]

运行时可切换：SwitchableSource（UI 里按 TAB），切换只改变"谁产生 Action"。
```

统一点是 `Action`：

```python
Action = { direction: (dx, dy), magnitude: float }   # direction 为单位向量
```

---

## 1. 分层与硬边界

```
action/                    硬件/设备无关的输入层
├── types.py    Action + combine() + coerce_action()
├── base.py     ActionSource 协议 + ScriptedSource / ControllerSource / NullActionSource
└── manual.py   ManualInputSource（键名→Action） + RandomActionSource

hardware_interface/        开发板侧
├── tbd.py             所有"尚未确定"的通信参数（全部 None）
├── input_adapter.py   HardwareInputAdapter / Placeholder / Loopback / Recorded
└── state_link.py      Simulator -> FPGA input 的既有字节协议

render/pygame_view.py      仅负责把 pygame 按键翻译成**键名**并转发
```

**硬边界（有测试断言）**

1. `action/` 下任何模块都**不能** import `bullet_sim.simulator`，也不能读写环境状态；
2. `ManualInputSource` 只回答"当前设备状态下玩家在做什么"，不决定玩家会移动到哪；
3. `World`/`Env` 是唯一调用 `World.step()` 的地方。

---

## 2. 模式 A：人工操纵

### 2.1 键盘绑定（键名与设备无关）

`action/manual.py::DEFAULT_BINDINGS`

| 方向 | 按键 |
|---|---|
| up / down / left / right | ↑ ↓ ← → **和** W S A D |
| 停止 | Space, 0 |
| 聚焦/慢速（magnitude 缩放） | Shift (0.4×), Ctrl (0.25×) |

**斜向不需要专门按键**：同时按住 `up` + `right` 即得 `up_right`，且
`combine()` 会重新归一化，所以斜向速度与正向相同（不会快 √2 倍）。
相反方向同时按下互相抵消。

### 2.2 用法

```python
from bullet_sim.action import ManualInputSource
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.scenarios.presets import scenario_for_level

env = BulletHellEnv(scenario_for_level("hard", seed=3, duration=60.0))
manual = ManualInputSource()
manual.open()
# 设备后端（pygame / evdev / 网页手柄 / socket）只做两件事：
manual.press("up"); manual.release("up"); manual.set_axis(0.7, -0.3)
env.run(input_source=manual, steps=None)      # headless 也可跑
```

### 2.3 命令行

```bash
python -m bullet_sim play --input keyboard              # 窗口 + WASD/方向键
python -m bullet_sim play --input keyboard --prediction # 附带预测轨迹叠加
python -m bullet_sim run  --input keyboard --steps 600  # 无窗口（键盘后端仍需事件循环）
```

`render/pygame_view.py` 中的 `pygame_key_names()` 是 pygame 与输入层唯一的接触点：

```python
renderer = PygameRenderer(input_source=manual)     # KEYDOWN -> press(key_name)
                                                   # KEYUP   -> release(key_name)
```

换成手柄/网页/串口只是换一个"事件 → 键名"的适配器，`ManualInputSource` 不变。

---

## 2.5 模式 B：自主控制（`bullet_sim/ai/`）

不给键盘，由控制器产生 Action。当前仓库提供 4 个**基线**控制器（**没有训练好的模型**）：

| 名称 | 访问级别 | 说明 |
|---|---|---|
| `idle` | observation | 永不移动（对照） |
| `random` | observation | 带种子随机游走 |
| `repulsion` | observation | 逃离附近弹幕密度 |
| `threat` | observation | 解析预测：9 个候选方向的未来最坏间隙 |
| `planner` | **world** | clone 世界做真实 rollout |

```bash
python -m bullet_sim play --level easy --seed 3 --input auto --auto threat --keep-alive
python -m bullet_sim run  --level medium --seed 21 --steps 1200 --input auto
python -m bullet_sim autoplay --level medium --seed 21 --steps 1200
```

训练模型接入：见 `README.md` 第 7.3 节，标记为 `[MODEL_NOT_AVAILABLE_YET]`；
模型部署工具链标记为 `[MODEL_DEPLOYMENT_TBD]`。**两者都未被猜测。**

## 2.6 运行时切换 Manual <-> Autonomous

`SwitchableSource`（`bullet_sim/action/switch.py`）持有多个命名源：

```python
sw = SwitchableSource({"manual": manual, "auto": auto_source}, manual_names=["manual"])
sw.toggle()             # UI 里绑定在 TAB
env.run(input_source=sw, steps=600)
```

* 切换**不修改世界状态**，轨迹连续；
* 离开人工源时自动清空按住的键，避免旧按键继续操纵；
* `env.run` 返回 `input_mode` 与 `mode_switches`，便于审计与数据集标注。

---

## 3. 模式 C：开发板操纵（**协议已定并已实现**）

### 3.1 协议（已经不是 TBD）

板子算完决策就把动作号打到控制台/stdout/UDP：**一行一个** `@MOVE <n>\n`，`n∈0..8`。
定义在 `core/board/action_codec.hpp`（C++ 生产者），规矩与键盘桥完全一致：

* 只认**行首**就是 `@MOVE ` 的行（这条线上混着内核/程序日志）；
* 前缀后必须是 0..8 的整数，否则**静默丢弃**；
* **断流回中**（默认 500 ms）—— 不放的话板子一断，角色会一直朝最后那个方向飞。

解析规则在 `bullet_sim/hardware_interface/board_link.py`；它与
`core/rp2040_keyboard/console_relay.py:parse_moves` 由测试逐批对账
（`tests/test_board_link.py`），差异一处都算失败。

### 3.2 用法：一条命令让**仿真器当平台**

```bash
# 串口（板子控制台 / RP2040 的 USB CDC）
python -m bullet_sim board-control --port /dev/ttyUSB0

# SSH 管道（推荐：不占控制台、不用 usbipd、不动防火墙）
ssh -i <key> root@192.168.137.242 \
    'cd /root/tools && ./cam_live.sh --frames 0 --via-pl --keyboard-dev stdout' \
  | python -m bullet_sim board-control --stdin

# UDP（板子 --keyboard-dev udp://<本机IP>:9999）
python -m bullet_sim board-control --udp 9999

# 无窗口（自测 / 无显示的机器）
python -m bullet_sim board-control --headless --stdin --steps 600 --json
```

`board-control` 就是"输入源换成板子、**默认不自动停**"的 `play`
（`--headless` 时走 `run`）：主循环只有一份，不存在第三套"什么时候停"的规矩。

板子在环里时**开场只放一种障碍**（`--stage-seconds`，`play`/`board-control` 默认 8 秒）：
第一段只有墙+缺口 —— 摄像头 → PL 几何 → `gap_avoid` 唯一能稳定读出的就是它。
开场把墙、移动块、小障碍一次全放上去，画面里分不清"哪个是墙、哪个是目标"，
控制对不对根本看不出来。要回到"全都在场"就加 `--stage-seconds 0`。
（详见根 `README.md` §9.4。）

### 3.3 `--nostop`：平台不许自己停

| 行为 | 默认 | `--nostop`（`board-control` 的默认） |
|---|---|---|
| 回合步数上限（`spec.total_steps`） | 到点就停 | **不设上限**（一直跑，Ctrl-C 停） |
| `terminated` / `truncated` | 结束本轮 | **忽略**，继续跑 |
| 碰撞 | 本来就是惩罚事件、不终止（`--collision-terminates` 才终止） | 同上 |
| 显式 `--max-steps` / `--max-seconds` | 生效 | **仍然生效**（显式永远优先） |

实测对照（回合约 60 步，要求跑 120 步）：`board-control` → `ran 120 steps
truncated=True`；`play --input board`（不带 `--nostop`）→ `ran 60 steps`。

### 3.4 为什么这条链路重要：绕开"平台收不收合成按键"这个未知

真实比赛平台吃不吃软件合成按键（SendInput）一直是链路上唯一没法离线验证的一环。
把**仿真器当平台**之后，板子的决策直接用 `@MOVE n` 驱动仿真，**不需要键盘注入**，
于是"板子→平台"这一段可以现在就闭环验证；真实平台那条路留着按需再验。

### 3.5 怎么验（不需要板子）

`tests/test_board_link.py`（33 条）：解析器与键盘桥逐批对账、stdin/UDP 传输、
**看门狗回中**、CLI 把板子信号喂进仿真、以及 `--nostop` 真的跑过回合终点
（对照组用不带 `--nostop` 的 `run`）。都给的是假信号（管道 / 本地 socket），
所以 CI 里也能跑。

> 历史说明：`tbd.py` 里的 `HARDWARE_INPUT_INTERFACE_TBD` 与
> `PlaceholderHardwareInputAdapter` **保留着** —— 当**没有**给任何传输参数时，
> 它仍然是默认（拒绝产出，而不是猜一个协议）。协议未定那段时间的记录见 §3.1 的
> 实现史与 `git log`。

## 4. 状态上行（Simulator → CPU → FPGA）

见 `docs/PROTOCOL.md` 与 `hardware_interface/state_link.py`。
`interface/hardware.py` 的 `HardwareLink` 抽象了
`upload_state / fetch_prediction / send_action`，具体总线同样未定（TBD）。

完整目标回路：

```
Simulator State --encode_state()--> CPU --> FPGA input buffer
                                    FPGA result --> CPU decision
Board / CPU action --HardwareInputAdapter--> Action --> Simulator
```

---

## 5. 命令行速查

```bash
python -m bullet_sim play --input keyboard     # 模式 A
python -m bullet_sim play --input board        # 模式 B（当前为 TBD 占位，会明确报错）
python -m bullet_sim play --input scripted     # 回放
python -m bullet_sim play --input random       # 随机基线
python -m bullet_sim play --input controller   # 用 --controller 指定的策略（RL/MPC/NPU）
python -m bullet_sim run  --input board --steps 500   # headless，同样的输入源
```
