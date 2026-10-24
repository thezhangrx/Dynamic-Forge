# CPU/FPGA State & Action 协议 (v3)

> **真值顺序：代码 > 本文档。** 版本与布局的唯一来源是
> `bullet_sim/core/version.py` 的 `STATE_PROTOCOL_VERSION` 与
> `bullet_sim/entities/bullet.py` 的 `PROTOCOL_FIELDS` /
> `PROTOCOL_FIELDS_V1` / `PROTOCOL_FIELDS_V2`。
> 本文档曾长期停留在 v2（stride 13、`88+52n`），与代码不符 —— 已按代码订正。

本文件是模拟器与硬件/外部进程之间的**唯一契约**。它位于 `interface/`，
不依赖模拟器内部实现，因此 `Virtual Simulator → CPU → FPGA` 与
`Physical Sensor → CPU → FPGA` 可以共用同一套解析代码。

## 1. State 帧布局（小端）

| offset | size | type | 字段 |
|---|---|---|---|
| 0 | 4 | uint32 | magic = `'BHL1'` = `0x314C4842` |
| 4 | 2 | uint16 | version = 3（当前值；v1/v2 帧仍可读）|
| 6 | 2 | uint16 | flags（bit0: 弹幕已压缩） |
| 8 | 4 | uint32 | step_index |
| 12 | 4 | uint32 | n_bullets |
| 16 | 4 | float32 | sim_time |
| 20 | 4 | float32 | field_w |
| 24 | 4 | float32 | field_h |
| 28 | 4 | float32 | dt |
| 32 | 28 | float32[7] | player: `x, y, vx, vy, radius, speed, alive` |
| 60 | 24 | float32[6] | target: `x, y, radius, shape, half_w, half_h` |
| 84 | 68·n | float32[n][17] | bullets（固定 stride = 17 floats） |
| 84+68n | 4 | uint32 | 之前所有字节的 crc32 |

**总长度 = 88 + 68 × n_bullets 字节**

弹幕行字段顺序（`stride = 17`）：

```
x, y, vx, vy, ax, ay, angle, angular_velocity, radius, ttl, type_id, group_id, id,
shape, half_w, half_h, rotation
```

> **v3 相对 v2** 追加 `shape`、`half_w`、`half_h`、`rotation` ——
> 一条"子弹"从圆变成一个**可旋转矩形**（即 dynamic obstacle），
> stride 13 → 17，帧长 `88+52n` → `88+68n`。
> `shape` 取值见 `SHAPE_CODES`（0 = circle、1 = rect）；
> 圆形障碍填 `half_w = half_h = radius`、`rotation = 0`。
>
> **v2 相对 v1** 新增 `angle`、`angular_velocity`、`id`。
>
> `decode_state()` **同时接受 v1 / v2 / v3 帧**（`STATE_PROTOCOL_VERSION_MIN_READABLE = 1`）：
> v1 缺失的字段由 `vx,vy` 推导（`angle = atan2(vy, vx)`）、`angular_velocity = 0`、
> `id` 按顺序补 1..n；v2 缺失的四个字段按圆形补默认值。
> `encode_state()` 只产出 v3。带版本号的旧抓包因此不会失效。
>
> ⚠ **`stride` 随版本变，不要写死 17**：按帧内版本查
> `stride_for(h.version)` / `fields_for(h.version)`。

### 为什么这样设计

* **固定 stride**：任意一条弹幕都能以 `base + i*stride` 定位（v3 是 `i*68`），
  天然映射 DMA descriptor / BRAM 行 / 流水线寄存器。
* **header 全部定长**：CPU 侧可只读前 32 字节做路由与合法性检查，无需解析变长部分。
* **CRC32 尾校验**：跨板卡/串口传输时能检出位翻转，避免污染决策。
* **float32**：与 FPGA DSP 常见数据路径对齐；模拟器内部默认 float64，
  导出时降精度，因此**不要**用协议帧做逐位复现断言，复现请用
  `WorldSnapshot.state_hash()`。

## 2. Action 帧

| 用途 | 布局 | 说明 |
|---|---|---|
| 连续速度 | `float32[2]` | 世界坐标系期望速度 `(vx, vy)` |
| 离散动作 | `uint32` | `0..8`，顺序见 `DISCRETE_ACTION_NAMES` |

离散动作表（顺序是协议的一部分，只允许追加，不允许重排）：

```
0 stay  1 left  2 right  3 up  4 down
5 up_left  6 up_right  7 down_left  8 down_right
```

## 3. Python API

```python
from bullet_sim.interface.protocol import (
    encode_state, decode_state, read_header, bullet_matrix,
    encode_action_velocity, decode_action_velocity,
    encode_action_discrete, decode_action_discrete,
)

frame = encode_state(world.get_state())      # bytes
hdr   = read_header(frame)                   # 零拷贝读 header
m     = bullet_matrix(frame)                 # float32[n,stride] 零拷贝视图
snap  = decode_state(frame)                  # 完整解码（含 CRC 校验）
```

所有非法帧（magic / version / 长度 / CRC 错误）都会抛出
`bullet_sim.core.errors.ProtocolError`。

## 3.1 版本兼容表

| version | bullet stride | 帧长 | 字段 | 可读 | 可写 |
|---|---|---|---|---|---|
| 1 | 10 floats (40 B) | `88 + 40n` | 无 angle/angular_velocity/id | ✅ `decode_state` | ❌ |
| 2 | 13 floats (52 B) | `88 + 52n` | + angle/angular_velocity/id | ✅ | ❌ |
| **3** | **17 floats (68 B)** | **`88 + 68n`** | **+ shape/half_w/half_h/rotation** | ✅ | ✅ `encode_state` |

上表（以及本文档的标题、header 表版本号、v3 字段顺序、布局表尺寸）由
**`core/bullet_sim/tests/test_protocol_doc_consistency.py`** 钉住 ——
代码改了而这里没改会红。这正是它之前停在 v2 的原因：`test_protocol.py` 只管代码，
`test_readme_consistency.py` 只管 README，**没有一条测试看这份唯一契约**。

```python
from bullet_sim.interface.protocol import read_header, stride_for, fields_for
h = read_header(frame)
fields = fields_for(h.version)      # 按帧内版本解释列含义
stride = stride_for(h.version)
```

## 4. Hardware Link (HAL)

`bullet_sim.interface.hardware`：

```python
class HardwareLink(ABC):
    def upload_state(self, frame: bytes) -> None      # CPU → FPGA 输入缓冲
    def fetch_prediction(self) -> bytes               # FPGA → CPU 预测输出缓冲
    def send_action(self, action: np.ndarray) -> None  # CPU 决策 → FPGA/执行器
    def stats(self) -> LinkStats                      # 延迟与带宽统计
```

内置实现：

| 实现 | 用途 |
|---|---|
| `NullLink` | 纯 CPU 仿真（丢弃所有帧） |
| `LoopbackLink` | 同进程回环 + 可插拔的 `accelerator(frame)->frame` |
| `FileLink` | 通过文件交换帧，模拟离线 FPGA 批处理流水线 |
| `SerialLinkStub` | 串口传输契约占位（接板后实现） |

```python
link = LoopbackLink(accelerator=my_fpga_model)
link.open()
prediction = link.roundtrip(encode_state(snapshot))
link.send_action(np.array([12.5, 0.0]))
print(link.stats.summary())
```

## 5. 迁移路径

```
[今天] Virtual Simulator --encode_state--> CPU 决策 --> AssumedAction
                                  |
                                  +--> LoopbackLink ("FPGA" 用 Python 建模)

[下一步] Virtual Simulator --encode_state--> CPU --> 真 FPGA 预测 -->
          CPU 决策 --> 真 FPGA/执行器

[最终]  Physical Sensor  --encode_state--> CPU --> 真 FPGA 预测 -->
          CPU 决策 --> 2D 小车执行器
```

只需要替换 frame 的**生产者**，协议、CPU 决策层与 FPGA 预测块保持不变。
