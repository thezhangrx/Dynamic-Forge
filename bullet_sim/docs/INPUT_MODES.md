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

## 3. 模式 C：开发板操纵（当前为占位）

### 3.1 契约

```python
class HardwareInputAdapter(ABC):
    transport: str = HARDWARE_INPUT_INTERFACE_TBD

    @abstractmethod
    def open(self) -> None: ...
    @abstractmethod
    def close(self) -> None: ...
    @abstractmethod
    def read_payload(self) -> Any: ...        # 从板卡读一帧原始负载
    @abstractmethod
    def decode_payload(self, payload) -> Action: ...
    def poll(self, dt) -> Action | None: ...  # read + decode
```

### 3.2 明确留白的内容

**全部集中在 `hardware_interface/tbd.py`，默认值一律 `None`，不会被任何代码猜测。**

`PlannedTransport`：`kind`（UART/SPI/USB/GPIO/TCP/共享内存/MMIO/PCIe/其它）、
`baud_rate`、`byte_order`、`packet_bytes`、`register_base`、`payload_format`、
`endpoint`、`handshake`、`timeout_ms`。

`ActionFrameFormat`：`fields`、`bits_per_field`、`scale`、`offset`、`semantics`。

未配置时：

```python
>>> PlaceholderHardwareInputAdapter().open()
HardwareInterfaceNotConfigured: [HARDWARE_INPUT_INTERFACE_TBD] board input is not configured yet
>>> adapter.read_payload()
HardwareInterfaceNotConfigured: [HARDWARE_INPUT_INTERFACE_TBD] no transport is configured ...
```

**它拒绝返回动作，而不是编造一个。** 这是有意为之：答辩时"我们还没有确定板卡协议"
是诚实且正确的状态，"我们猜了一个"才是风险。

### 3.3 拿到板卡后要做的三件事

1. 在 `hardware_interface/tbd.py` 里把 `PlannedTransport` / `ActionFrameFormat`
   填成真实值（或新建一个模块，不修改 placeholder 的默认值）；
2. 实现一个 `HardwareInputAdapter` 子类：

```python
class MyBoardAdapter(HardwareInputAdapter):
    transport = "uart"                       # 或实际选择
    def open(self):
        self.planned.require("kind", "baud_rate", "payload_format", "byte_order")
        self._port = ...                     # 只在这里出现真实通信代码
        self._open = True
    def read_payload(self):
        return self._port.read(self.planned.packet_bytes)   # 没新数据返回 None
    def decode_payload(self, payload) -> Action:
        ...                                  # 按真实帧格式解析成 Action
```

3. 接进仿真，**不改任何环境逻辑**：

```python
env.run(input_source=HardwareInputSource(MyBoardAdapter(...)))
```

### 3.4 今天可以验证的链路

在真实板卡到来前，用 loopback harness 验证
`Adapter -> Action -> World.step` 这条链路是通的：

```python
from bullet_sim.hardware_interface import LoopbackHardwareInputAdapter, HardwareInputSource
adapter = LoopbackHardwareInputAdapter()
adapter.hold(Action.from_discrete("up_left"))
env.run(input_source=HardwareInputSource(adapter), steps=1000)
```

`LoopbackHardwareInputAdapter` / `RecordedHardwareInputAdapter` **不是协议猜测**，
它们是测试夹具，`planned.kind` 保持 `None`。

---

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
