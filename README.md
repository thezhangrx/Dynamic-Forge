# bullet_sim — 二维动态障碍环境模拟器 · 组员开发手册

> 目标链路：**虚拟环境训练 → CPU / FPGA 异构部署 → 现实二维小车验证**。
> 优先级：**正确性 > 可复现性 > 模块化 > 数据接口 > 性能 > UI 美观**。
>
> 平台的核心不是"弹幕游戏"，而是**有尺寸、有来源、有运动规律、并且保证存在可行安全路径的
> 动态障碍环境**。渲染、UI、输入设备都不是核心；Simulator 才是。

---

## 1. 项目简介

| 项 | 值 |
|---|---|
| 包版本 | `0.1.0` |
| 唯一硬依赖 | NumPy |
| 可选依赖 | pygame（可视化）、pytest（测试）、matplotlib |
| 状态协议 | **v3**（`readable_versions: [1, 2, 3]`） |
| 数据集 schema | v1（`bin / csv / dat / json / npz`） |
| 自动化测试 | **383 项**（`python -m pytest bullet_sim/tests -q`） |

三句话概括现状：

1. **角色是一个圆**，画出来的大小就是碰撞大小（没有"看起来大、判定小"的车身）。
2. **环境只有现实障碍**：`moving_block / wall_with_gap / small_obstacles / corridor /
   cross_traffic`，可任意组合；装饰性弹幕模式（radial / spiral / line / wall / …）**已全部删除**。
3. **难度是一个总控**：`--level` 同时、单调地放大障碍速度 / 数量 / 类型数；
   每个 level 构建后都会**实测安全区**，`extreme` 也一定留有可行通道（有挑战 ≠ 必死）。

---

## 2. 架构

### 2.1 分层视图

严格单向依赖，上层可替换、下层零依赖。

```
┌──────────────────────────────────────────────────────────────────────┐
│ L6 应用层      cli  /  examples  /  run_demo.py                       │
├──────────────────────────────────────────────────────────────────────┤
│ L5 工具层      render/     dataset/    benchmark/    prediction/     │
│                (可视化)     (数据集)     (性能)        (预测/危险场)   │
├──────────────────────────────────────────────────────────────────────┤
│ L4 接口层      interface/  (Simulator API · State/Action 协议 · HAL)  │
│                simulator/  (World 确定性内核 · Env · Replay · Reward) │
├──────────────────────────────────────────────────────────────────────┤
│ L3 场景层      scenarios/  (ScenarioSpec · 参数化难度 · 预设 · 构建)  │
│                obstacles/  (ObstacleType 目录 · ObstacleScenario 组合)│
│                safety/     (自由空间 · 可行路径 · 场景可行性校验)      │
├──────────────────────────────────────────────────────────────────────┤
│ L2 机制层      generators/  collision/  physics/                     │
│                (纯函数/纯数据：不持有世界引用，可单测、可移植)          │
├──────────────────────────────────────────────────────────────────────┤
│ L1 核心层      core/  (SoA 状态 · RNG · 时钟 · 动作 · 配置)           │
│                entities/ (Player / BulletPool / Target 的 SoA 视图)   │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.2 关键约束

| 规则 | 说明 |
|---|---|
| 内核无 UI | `simulator/` 及以下不 import 任何渲染库；`render/` 只读快照 |
| 内核无 AI | 算法通过 `ActionProvider` / `ActionSource` 注入，模拟器不 import 策略库 |
| 内核无随机 | 关卡随机在**构建期**用种子展开成确定性 `SpawnEvent` 时间线；步进本身不含 RNG |
| 内核无分配 | 障碍池按容量预分配，稳态步进零动态分配 |
| 预测与物理分离 | `prediction/` 只调用 `world.clone()` / `world.step()`，不修改世界 |
| 障碍不从中心凭空生成 | 生成器只能把障碍放在**边界**上（或作为跨越场地的墙 / 走廊） |

---

## 3. 核心模块

| 目录 | 职责 | 关键文件 |
|---|---|---|
| `bullet_sim/core/` | SoA 状态容器、种子管理、固定时钟、动作编解码、数值策略 | `state.py`, `rng.py`, `clock.py`, `actions.py`, `numerics.py` |
| `bullet_sim/entities/` | Player / BulletPool / Target 的 SoA 读写视图 | `player.py`, `bullet.py`, `target.py` |
| `bullet_sim/physics/` | 精确运动积分（直线 / 圆弧 / **矩形自转**）、玩家运动学、边界剔除、相对运动 | `motion.py`, `kinematics.py`, `bounds.py`, `relative.py` |
| `bullet_sim/collision/` | 可替换碰撞后端 + 接触状态机 | `circle.py`, `grid.py`, `shapes.py`, `obstacles.py`, `events.py` |
| `bullet_sim/generators/` | 构建期实体生成（动态障碍 layout）、`SpawnEvent` / `SpawnTimeline` | `patterns.py`, `burst.py`, `composite.py`, `registry.py` |
| `bullet_sim/obstacles/` | **现实障碍类型目录**与场景组合 | `catalog.py`, `spec.py`, `scenario.py` |
| `bullet_sim/safety/` | 配置空间自由空间、时间窗路径搜索、场景可行性闭环 | `free_space.py`, `path_search.py`, `validate.py` |
| `bullet_sim/scenarios/` | `ScenarioSpec`、参数化难度、level 预设、构建与标定 | `spec.py`, `complexity.py`, `params.py`, `presets.py`, `builder.py` |
| `bullet_sim/simulator/` | 确定性内核、Env、Replay、Reward | `world.py`, `env.py`, `replay.py`, `rewards.py` |
| `bullet_sim/action/` | 统一 `Action`、`ActionSource`、人工输入、运行时切换 | `types.py`, `base.py`, `manual.py`, `switch.py` |
| `bullet_sim/ai/` | 自主基线控制器、模型接入契约 | `baseline.py`, `policy.py`, `factory.py` |
| `bullet_sim/cpu/`, `bullet_sim/fpga/` | CPU 决策层、CPU↔FPGA 回路、预测帧参考实现 | `decision.py`, `prediction_block.py` |
| `bullet_sim/hardware_interface/` | 开发板输入占位（全量 TBD，故障即报错） | `tbd.py` |
| `bullet_sim/interface/` | Obs/Action 空间、扁平协议、控制器契约、HardwareLink | `space.py`, `protocol.py`, `controller.py`, `hardware.py` |
| `bullet_sim/prediction/` | 闭式弹道、rollout、危险场 | `ballistic.py`, `rollout.py`, `danger_field.py` |
| `bullet_sim/dataset/` | 数据集 schema、录制、多格式读写、回放校验 | `schema.py`, `recorder.py`, `writers.py`, `readers.py` |
| `bullet_sim/benchmark/` | 性能与压力套件 | `metrics.py`, `runner.py`, `suites.py` |
| `bullet_sim/render/` | pygame / ASCII 视图、**Viewport 窗口缩放**、HUD、调试叠加 | `viewport.py`, `pygame_view.py`, `ascii_view.py`, `overlays.py` |
| `bullet_sim/tests/` | 自动化测试（含 README 一致性检查） | `test_obstacle_environment.py` 等 |

---

## 4. 数据结构

**以下字段名与代码一致**：`bullet_sim/entities/bullet.py`、`bullet_sim/entities/player.py`、
`bullet_sim/core/state.py`、`bullet_sim/action/types.py` 是唯一权威。

整个状态：`S_t = { player, bullets, target, environment, timestamp }`。

### 4.1 Bullet (`BulletPool`) — SoA 列式存储

每个字段一条连续数组，容量固定、稳态零分配；`id` 单调递增且**跨 `compact()` 稳定**。
一个 Bullet 既可以是"小圆障碍"，也可以是"墙"——区别只在字段解释，不在结构。

| 字段 | dtype | 说明 |
|---|---|---|
| `id` | int32 | 唯一标识，碰撞报告与数据集引用它（不是槽位号） |
| `x` | float | 位置 x |
| `y` | float | 位置 y |
| `vx` | float | 速度 x |
| `vy` | float | 速度 y |
| `ax` | float | 加速度 x |
| `ay` | float | 加速度 y |
| `angle` | float | 当前速度方向（弧度，每步自动同步） |
| `angular_velocity` | float | **按形状解释**：圆 = 速度方向旋转率（曲线）；矩形 = **自转**（绕自身中心，位置不变） |
| `radius` | float | 圆形碰撞半径；矩形由 SDF 使用 `half_w/half_h` |
| `age` | float | 已存活时间 |
| `ttl` | float | 生命周期（走廊墙为 `inf`） |
| `type_id` | int16 | 类型标签（可视化 / 分组） |
| `group_id` | int16 | 发射组（同一 pattern 实例） |
| `shape` | int16 | `0` = circle，`1` = rect（`SHAPE_CODES`） |
| `half_w` | float | 矩形半宽；圆形时 `= radius`（同一套 clearance 代码通用） |
| `half_h` | float | 矩形半高；圆形时 `= radius` |
| `rotation` | float | 矩形朝向（弧度，世界系；可被 `angular_velocity` 驱动） |
| `alive` | bool | 存活掩码 |

> `vx,vy` 与 `angle` 是同一运动的两种视图：`angle` 每步由速度重算。
> `shape / half_w / half_h / rotation` 是 **protocol v3** 新增字段（stride 10 → 13 → 17）；
> 解码旧帧时自动回填 `shape=0, half_w=half_h=radius, rotation=0`。

### 4.2 Player (`PlayerState`, float64[7])

**角色就是一个圆**：半径既是碰撞体，也是屏幕上画出来的圆。

| 字段 | 说明 |
|---|---|
| `x` | 位置 x（世界坐标，原点左下） |
| `y` | 位置 y |
| `vx` | 速度 x |
| `vy` | 速度 y |
| `radius` | 圆的半径（**可视 = 碰撞**） |
| `speed` | 最大速度 |
| `alive` | 存活标志 |

没有 `player_body_radius`、`body_radius`、`body_to_hitbox_ratio`——它们已被删除。
障碍尺寸、缺口宽度、走廊宽度全部以 `player_diameter = 2 × player_radius` 为单位，
因此"两倍我自己的宽度"这种描述到真实小车上依然成立。

### 4.3 Environment (`EnvState`)

| 字段 | 说明 |
|---|---|
| `field_w` | 场地宽 |
| `field_h` | 场地高 |
| `dt` | 固定时间步（= 1 / 仿真频率） |
| `step_index` | 当前步序号 |
| `sim_time` | 仿真时间 = `step_index * dt` |
| `wrap` | 出界策略：`cull` / `wrap` / `bounce` |
| `bullet_budget` | 同时存活实体上限（= 池容量） |

### 4.4 Action（统一动作接口）

| 字段 | 类型 | 说明 |
|---|---|---|
| `direction` | `(float, float)` | 单位向量；零向量表示不移动 |
| `magnitude` | `float` | 强度 `[0, 1]`；0 等价于 stay |

```python
from bullet_sim.action import Action
Action.from_discrete("up_right")   # 9 向之一
Action.from_angle(210.0, 0.5)      # 角度 + 强度
Action.from_vector([3.0, 4.0])     # 任意 2D 向量（自动归一化）
Action.zero()                      # 不移动
```

`Action` 是**对外唯一动作表示**。键盘、AI 控制器、未来开发板都产出同一个 `Action`，
再经 `action_to_codec_input()` 进入 `World.step()`——这条链路不允许被绕过或加分支。

---

## 5. CPU / FPGA Interface

### 5.1 分工

| 侧 | 负责 |
|---|---|
| CPU | 观测编码、决策（`bullet_sim/cpu/decision.py`）、回合管理、数据集、渲染 |
| FPGA | 大规模障碍的**并行预测**（`bullet_sim/fpga/prediction_block.py`：`BHP1` 帧协议 + Python golden model） |
| Simulator | 环境本体，对两侧都只暴露字节协议与 `Action` |

### 5.2 State 帧（v3）

```
Header (84 B, 小端)
  magic u32 'BHL1' | version u16 (=3) | flags u16 | step_index u32 | n_bullets u32
  sim_time f32 | field_w f32 | field_h f32 | dt f32
  player f32[7] (x,y,vx,vy,radius,speed,alive)
  target f32[6] (x,y,radius,shape,half_w,half_h)
Bullet stride = 17 x f32 = 68 B  (v3)
  x, y, vx, vy, ax, ay, angle, angular_velocity, radius, ttl,
  type_id, group_id, id, shape, half_w, half_h, rotation
Trailer: crc32 u32
Total := 88 + 68 * n_bullets 字节   (v1 = 88+40n, v2 = 88+52n)
```

`decode_state()` 同时接受 v1 / v2 / v3；`encode_state()` 只产出 v3。
`describe_protocol()` 上报 `readable_versions: [1, 2, 3]`。详细说明见
`bullet_sim/docs/PROTOCOL.md`。

### 5.3 现在**可以**使用什么

```bash
# 端到端：Simulator -> FPGA 参考块 -> CPU 决策 -> Action -> Simulator
python -m bullet_sim.examples.10_cpu_fpga_loop
```

```python
from bullet_sim.cpu import CpuFpgaPipeline
from bullet_sim.fpga import FpgaPredictionReference
```

### 5.4 现在**不能**使用什么

* 真实板卡通信（串口参数、寄存器地址、时序）——**未接入**
  （`[HARDWARE_INPUT_INTERFACE_TBD]`），占位实现会抛
  `HardwareInterfaceNotConfigured`，而不是猜一个动作；
* FPGA RTL——目前只有 Python golden model 与帧格式（`[HARDWARE_INTERFACE_TBD]`）；
* 量化与上板工具链（`[MODEL_DEPLOYMENT_TBD]`）。

`bullet_sim/hardware_interface/tbd.py` 是唯一记录这些未知量的地方。

---

## 6. Manual Control（人工操纵）

```bash
python -m bullet_sim play --input manual --seed 3          # 等价于 --interactive
python -m bullet_sim play --level easy --seed 3            # play 默认就是 manual
```

| 按键 | 动作 |
|---|---|
| 方向键 / WASD | 移动（支持斜向） |
| `SPACE` | 显式停止（覆盖移动键） |
| `SHIFT` | 聚焦（0.4× 速度） |
| `CTRL` | 慢速（0.25× 速度） |
| `TAB` | 运行时 MANUAL ↔ AUTO 切换 |
| `H` | 切换 HUD |
| `+` / `-` / `0` | 缩放 / 复位视图 |
| 鼠标滚轮 | 缩放 |
| `B` | 切换碰撞轮廓显示 |
| `ESC` | 退出 |

**键盘不会直接改玩家状态**：按键 → `ManualInputSource` → `Action` → `World.step()`。
细节见 `bullet_sim/docs/INPUT_MODES.md`。

---

## 7. Autonomous / AI Control

```bash
python -m bullet_sim play --input auto --auto threat --seed 3 --prediction
python -m bullet_sim autoplay --level medium --seed 21 --steps 1200
```

| 基线 | 特点 |
|---|---|
| `idle` | 不动（对照） |
| `stay` | 显式停住 |
| `random` | 随机方向 |
| `repulsion` | 排斥式躲避（便宜） |
| `threat` | 解析预测式参考基线 |
| `planner` | 真实 rollout（最强，但最贵；有世界访问权） |

`--auto` 与 `--input` 的完整取值见 `python -m bullet_sim play --help`。

**训练好的模型：`[MODEL_NOT_AVAILABLE_YET]`。** 仓库不包含任何策略权重：

```python
from bullet_sim.ai import load_policy      # -> ModelNotAvailable
```

接入契约已就绪（`bullet_sim/ai/policy.py`）；模型需要你们自己训练。

---

## 8. 动态障碍环境（核心）

### 8.1 5 种现实障碍类型

| type key | 现实对应 | 运动 | 玩家任务 | 安全校验 |
|---|---|---|---|---|
| `moving_block` | 迎面 / 横向接近的大型物体（移动机器、货架、车辆） | 从边界**一段区间内的随机位置**进入，速度强制指向场内；默认 2× 玩家直径 | 提前决定从哪一侧绕行 | `bypass` |
| `wall_with_gap` | 围栏 / 隔断上的局部开口 | 两段矩形 + 一个 `gap_width` 缺口（位置随机抖动、可扫动）；缺口**不得小于玩家圆直径** | 定位缺口并判断"能否在墙到达前抵达" | `gap` |
| `small_obstacles` | 树林枝叶 / 人群车流中的细小物体 | 尺寸远小于玩家的圆从各边界进入 | 保持长期连贯的可行通道 | `local_free` |
| `corridor` | 通道 / 路面张开-收窄-偏转 | 角色在两堵墙**正中间**，墙心不动，只通过张开/收缩/旋转模拟道路变化（详见 8.3） | 判断变化后"现在能否通过 / 是否要等" | `corridor` |
| `cross_traffic` | 十字路口 / 多方向来流 | 从多条边界同时进入、方向各异 | 多方向相对运动之间的间隙选择 | `generic` |

`mixed_obstacles` 已删除——**组合就是重复 `--obstacle-type`**（见 8.2）。

### 8.2 组合场景

```python
from bullet_sim.obstacles.scenario import scenario_from_types

sc = scenario_from_types(
    [
        {"type": "moving_block", "size": 2.0, "speed": 90.0, "entry_span": 0.8},
        {"type": "wall_with_gap", "gap_width": 160.0, "gap_motion": "sweep"},
        {"type": "small_obstacles", "count": 12, "size": 0.35},
        {"type": "corridor", "corridor_width": 120.0, "motion": "rotate_same", "change": 16.0},
    ],
    seed=7, duration=30.0, player_hitbox_radius=10.0,
)
spec = sc.to_spec()        # 之后走既有管线：复现 / 数据集 / 基准 / 安全校验
built = sc.build()         # 默认先过安全校验，不可行则 raise
sc.describe()              # 玩家尺寸 + 每个类型的现实语义 + 每个实例的参数
```

```bash
python -m bullet_sim obstacles            # 列出 5 种类型及其现实语义
python -m bullet_sim obstacles --type corridor
python -m bullet_sim run --obstacle-type moving_block --obstacle-type wall_with_gap \
    --gap-width 180 --seed 3 --steps 900
```

### 8.3 corridor 详解（延伸长度与间距控制）

**几何**：角色生成在场地**正中心**，两堵墙以角色为中心**对称分布**，并**沿通道方向一直
延伸到场地两端**（默认跨场：`wall_length = 场地边长 × 1.1`，可用 `--wall-length` 覆盖）。
墙的**中心坐标不随时间平移**（旋转模式下完全不动）。

**三种道路变化**（`--corridor-motion`）：

| motion | 变化方式 | 宽度 | 转向 |
|---|---|---|---|
| `open` | 两墙沿法线**对称向外**平移 | 变宽 | — |
| `close` | 两墙沿法线**对称向内**平移 | 变窄（有下限） | — |
| `rotate_same` | 两墙**同向自转** | **恒定不变** | 都向一边 |
| `rotate_opposite` | 两墙**反向自转**（剪刀口） | 末端变窄（有下限） | 各向一边 |
| `static` | 不变 | — | — |

**间距怎么控制**（互不冲突的几条）：

| 参数 | 作用 | 默认 |
|---|---|---|
| `--player-radius R` | 圆形角色半径；**所有间距都以它的直径 `d = 2R` 为单位** | 10（障碍场景） |
| `--corridor-width W` | **初始间距**（两墙内表面之间的世界单位）；不得小于 `d` | `4d`（=80，R=10 时） |
| `--corridor-change C` | 整段回放的**总变化量**：`open`/`close` 时是宽度变化（单位 `d`），`rotate_*` 时是总角度（度）。分摊成恒定速度 / 角速度 | 0 |
| `--corridor-min-width M` | **间距硬下限**（世界单位）；不得低于 `1.25d`，低于直接报错 | `1.25d` |
| `--wall-length L` | 墙长（世界单位）；不填 = 跨场 | 跨场 |
| `--corridor-walls N` | `2` = 对称双墙，`1` = 只放一堵墙 | 2 |
| `--tilt-deg A` | 整条通道的初始倾角（度） | 0 |

```bash
# 双墙同向旋转：通道整体倾斜，宽度恒定（长墙也能转得很明显）
python -m bullet_sim play --obstacle-type corridor --corridor-motion rotate_same \
       --corridor-change 16 --corridor-width 120 --seed 1

# 双墙反向旋转（剪刀口）：角度会自动限制在几何安全范围内
python -m bullet_sim play --obstacle-type corridor --corridor-motion rotate_opposite \
       --corridor-change 20 --corridor-width 160 --seed 1

# 张开 / 收缩，并直接指定间距下限
python -m bullet_sim play --obstacle-type corridor --corridor-motion close \
       --corridor-width 160 --corridor-min-width 60 --corridor-change 20 --seed 1

# 只放一堵墙
python -m bullet_sim play --obstacle-type corridor --corridor-walls 1 --seed 1
```

**为什么反向旋转的角度看起来不大？** 这是几何必然，不是 bug：两堵墙绕各自中心反向
旋转 `θ` 时，墙**端部**的间距变成 `W − 2a·tanθ`（`a` = 墙半长）。墙延伸得越长，
允许的 `θ` 越小：

```
rotate_opposite, W=80,  墙长 704 → 安全上限约  4.5°
rotate_opposite, W=160, 墙长 704 → 安全上限约 10.9°
rotate_same 没有这个限制（宽度恒定），所以长墙的"路面偏转"请用 rotate_same。
```

`resolve_spec` 会把超限的 `--corridor-change` **自动裁剪**到安全值，因此**参数再离谱
也生成不出能夹死角色的走廊**（安全校验只是第二道保险）。

### 8.4 Pattern（旧的装饰性弹幕模式，已删除）

以下模式**不再存在**（没有生成器、不在 `--level` 预设里、`PatternSpec(kind=...)` 会报错）：

`single`、`radial`、`spiral`、`aimed`、`burst`、`line`、`wall`、`random`。

`available_patterns()` 只返回 `["obstacle"]`。运行时仍可注入单个实体（调试 / 传感器回放）：

```python
world.spawn_pattern(
    {"kind": "obstacle", "params": {"layout": "small", "size": 0.35}},
    count=1, speed=90.0,
)
```

5 个 layout：`block`（边界区间随机 + 强制朝内）、`wall_gap`（缺口 ≥ 玩家直径、位置随机/扫动）、
`small`、`cross`（多方向）、`corridor`（见 8.3）。

---

## 9. 难度：`--level` 是唯一总控 + 实测 complexity

### 9.1 level 单调放大三件事

| 量 | 0.0 → 1.0 | 说明 |
|---|---|---|
| `speed_scale` | 0.65 → 1.50 | 所有障碍速度 |
| `count_scale` | 0.35 → 1.50 | 障碍数量 |
| `obstacle_kinds` | 1 → 5 种 | `small_obstacles` → `+moving_block` → `+wall_with_gap` → `+cross_traffic` → `+corridor` |

`trivial / easy / medium / hard / extreme` 都是这条曲线上的点
（`LEVEL_VALUES = {trivial:0.0, easy:0.15, medium:0.40, hard:0.70, extreme:1.0}`）。
也可以直接给一个 `[0,1]` 的浮点数。

### 9.2 难度是**测出来的**

`validate_scenario()` 把实测到的自由空间、可达集、路径最小余量、峰值相对速度喂给
`complexity_from_measurements()`，结果就在 `SafetyReport` 上：

```python
from bullet_sim.safety import validate_scenario
report = validate_scenario(spec)
print(report.complexity, report.complexity_label, report.complexity_terms)
```

7 个分项与显式权重：`crowding` 0.22 / `reach` 0.20 / `tightness` 0.18 / `speed` 0.12 /
`anticipation` 0.16 / `geometry` 0.07 / `path_scarcity` 0.05。
`complexity_label` 只是给报告分箱，**不参与任何仿真语义**。

### 9.3 每个 level 都会被实测一遍（extreme 也保证有安全区）

`make_scenario()` 构建完 level 场景后会跑一次安全校验：自由空间下限 **15%**、可达区域
下限 **12 格**；不满足就按比例减薄障碍数量再校验，最多 5 次。实测结果（seed=3, 15 s）：

```
level     请求数量   自由空间   可达格   complexity   减薄次数
easy        128      88.2%     248      0.317        0
medium      208      68.4%     198      0.399        0
hard        304      47.8%     116      0.500        0
extreme     400      34.1%      22      0.587        1
```

请求值保存在 `meta['target_bullet_count']`，实际采用值在
`meta['achieved_bullet_target']`，减薄次数在 `meta['level_reduced_by']`，
完整报告在 `meta['safety']`。

**显式的 `--bullets N` / `stress(N)` 不受这把安全锁限制**——那是纯粹的"还能不能跑"
压力测试（100 … 10000）。

---

## 10. Collision（碰撞 = 惩罚事件）

### 10.1 行为约定

**碰撞是惩罚事件，不是进程 / 回合结束事件。**

```
collision_event  = True          # 本步出现了新的接触
collision_count += 1             # 累加（同一接触持续重叠不会重复计）
reward           -= collision_penalty
窗口继续渲染 · AI 继续决策 · 后续障碍继续生成 · step() 继续
terminated       = False         # 除非显式配置了终止条件
```

检测与"事件"是分开的两件事：`collision/*` 负责几何判定与 `ContactTracker`
进入/保持/退出状态机，`simulator/rewards.py` 与 Env 只消费事件。

### 10.2 可配置项

| 项 | 位置 | 默认 |
|---|---|---|
| `collision_penalty` | `ScenarioSpec` / `--collision-penalty` | 1.0（**正数**，每事件扣减；负数会被拒绝） |
| `collision_count_mode` | `per_contact` / `per_step` / `per_frame` | `per_contact` |
| `collision_terminates_episode` | 模式 B 开关 `--collision-terminates` | `False`（碰撞不结束回合） |
| `success_terminates_episode` | 达成目标是否结束 | `False` |

### 10.3 碰撞后端

| 后端 | 适用 |
|---|---|
| `circle` | 单玩家、暴力向量化最快 |
| `grid` | 多玩家 / 批量环境（均匀网格宽相） |
| `circle_distance` | 只关心最小距离（不做事件） |
| `shaped` | 圆 / 点 / 矩形自定义 hitbox（引擎能力；场景默认圆） |
| `obstacle` | **动态障碍**：圆 + 旋转矩形 SDF、相对运动接近率 |
| `null` | 只研究动力学 |

`ScenarioSpec.collision = "auto"` 时由场景自动选择：有矩形障碍 → `obstacle`，否则 `circle`。

---

## 11. 安全校验：有挑战 ≠ 必死

```
Scenario Generator → Obstacle Generation → Free Space Analysis
                   → Path Feasibility Check → Valid Scenario
```

| 公开 API（`bullet_sim/safety/`） | 作用 |
|---|---|
| `is_scenario_valid(...)` | 布尔判定：是否存在可行安全路径 |
| `find_safe_path(...)` | 找一条具体可行的无碰撞轨迹（`SafePath`） |
| `compute_safe_region(...)` | 当前 / 时窗自由空间与可达区域（`SafeRegion`） |
| `predict_collision(world, action, horizon)` | 保持该动作时首次碰撞时间（真实 rollout + 解析估计） |
| `validate_scenario(...)` | 完整报告（`SafetyReport`：原因、数值、**实测难度**、建议） |
| `generate_valid_scenario(...)` | **不"生成一次就失败"**：失败则定向调参重采样 |

判据是**几何 + 动力学 + 时间**，而不是当前帧重叠：

1. 每个采样时刻都位于**膨胀后**的障碍之外，膨胀量 = 玩家真实碰撞体外接圆 +
   `safety_margin`（配置空间 / configuration space）；
2. 相邻位移在 `player_speed`（与可选 `max_accel`）之内；
3. 轨迹从玩家真实出生点出发，覆盖整个验证时窗（默认 `min(回合时长, max_window)`）；
4. 时间展开的**到达时间波前**（arrival-time wavefront）保证小于一格的推进不丢失；
5. 移动 / 旋转的障碍会把"此刻被占据的格"从可传播源中剔除——**波前不会穿过移动的墙**。

```bash
# 可行 / 不可行都会给出理由与建议，不可行时退出码 2
python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 200 --validate --steps 600
python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 200 --validate --safe-path --steps 600
python -m bullet_sim obstacles --scenario scene.json --validate --safe-path
# 缺口小于玩家圆直径：构造期直接拒绝
python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 5 --steps 10
```

**保守偏差是刻意的**：单元格判 blocked 的条件是 `sdf < inflate`；宁可误判"不可行"，
也不放过"必死"场景。已知代价：密集场景单次校验耗时随实体数量增长，适合"生成阶段校验
一次 + 存 JSON 复用"，而不是每帧调用。

---

## 12. Visualization / HUD

```bash
python -m bullet_sim play --level medium --seed 3
python -m bullet_sim play --level easy --seed 3 --input auto --prediction
python -m bullet_sim play --obstacle-type corridor --corridor-motion rotate_same --seed 3
```

**窗口缩放**：`bullet_sim/render/viewport.py` 的单个 `Viewport` 负责世界↔屏幕映射
（等比 letterbox，`scale = fit_scale × zoom`）。放大窗口只改变"看到多大"，
`dt`、物理、碰撞判定与渲染帧率完全解耦。`VIDEORESIZE` 实时 `resize`，`+/-/0` 与滚轮缩放。

**画面元素**

| 你看到的 | 说明 |
|---|---|
| 白色圆 + 短线 | 玩家：半径既是碰撞体也是画面大小（短线只是方向提示） |
| 红色圆环（`B` 切换） | 同一个圆的碰撞轮廓——可视 = 碰撞，正好贴在一起 |
| 彩色小圆 / 矩形（可旋转） | 动态障碍，形状与 `shape/half_w/half_h/rotation` 一致 |
| 浅色网格 + 坐标标注 | 世界坐标轴（100 px 网格），把屏幕像素对应回世界量 |
| 蓝色折线（`--prediction`） | 障碍未来轨迹（解析外推，是**预测叠加**，不是障碍） |
| 红色半透明栅格（`--danger`） | 未来危险场 |
| 红色边框闪烁 | 刚发生碰撞（惩罚事件提示，不中断运行） |
| HUD | 控制模式 / 控制器 / 场景与障碍类型 / FPS / 碰撞与 reward / 实体计数 / 玩家圆半径 / 世界坐标 |
| 黄色横幅 | 切换模式时的即时提示 |

---

## 13. Headless Simulation

内核不依赖渲染：`render_mode="none"`（默认）下可无窗口跑任意规模。

```bash
python -m bullet_sim run --level medium --seed 7 --steps 1200
python -m bullet_sim autoplay --level medium --steps 1200 --seeds 1 2 3
```

```python
from bullet_sim.simulator.env import BulletHellEnv
env = BulletHellEnv(spec, seed=7, render_mode="none")
out = env.run(steps=1200)          # 返回 steps/return/collisions/state_hash/...
```

`SDL_VIDEODRIVER=dummy` + `PYGAME_HIDE_SUPPORT_PROMPT=1` 可在 CI 里跑渲染相关测试。

---

## 14. Dataset

```bash
python -m bullet_sim record --level medium --count 8 --out data/ --format npz --hashes --steps 800
python -m bullet_sim replay --dataset data/ep_00000.npz
```

* schema 版本化（`DATASET_SCHEMA_VERSION = 1`），数组键见 `python -m bullet_sim info`；
* 每条 episode 的 meta 内嵌**完整 `ScenarioSpec`**，因此"seed + actions → 轨迹"可离线精确复现；
* `replay` 会逐步重放并比对 `state_hash`，输出 `reproduced / reason`；
* 支持 `bin / csv / dat / json / npz`，`--bullet-stride` 可对实体采样以降体积。

细节见 `bullet_sim/docs/DATASET.md`。

---

## 15. 项目目录

```
Bullet_Platform/
├── run_demo.py                      # 单文件可运行入口（不需要安装）
├── pyproject.toml                   # 打包；NumPy 是唯一硬依赖
├── README.md                        # 本文件
├── benchmarks_output/               # 基准报告产物（本地生成，不入库）
└── bullet_sim/
    ├── core/                        状态容器 / RNG / 时钟 / 动作编解码 / 数值策略
    ├── entities/                    Player / BulletPool / Target
    ├── physics/                     运动积分（含圆弧与矩形自转）、玩家运动学、边界、相对运动
    ├── collision/                   圆-圆、hitbox 形状、均匀网格、障碍 SDF、接触状态机
    ├── generators/                  动态障碍 layout、SpawnEvent / SpawnTimeline
    ├── obstacles/                   5 种现实障碍目录、ObstacleScenario 组合
    ├── safety/                      自由空间（配置空间膨胀）、到达时间波前、场景可行性
    ├── scenarios/                   ScenarioSpec、level 总控、实测难度参数、构建与标定
    ├── simulator/                   World 确定性内核、Env、Replay、Reward
    ├── action/                      Action 类型、ActionSource、人工输入、运行时切换
    ├── ai/                          自主基线控制器、模型接入契约
    ├── cpu/                         CPU 决策层与 CPU↔FPGA 回路
    ├── fpga/                        预测帧协议与 Python 参考实现
    ├── hardware_interface/          开发板输入占位（TBD）+ 状态链路
    ├── interface/                   Obs/Action 空间、扁平协议、控制器、HardwareLink
    ├── prediction/                  闭式弹道、rollout、危险场
    ├── dataset/                     schema、recorder、json/npz/csv/bin 读写
    ├── benchmark/                   指标、runner、压力测试套件
    ├── render/                      pygame / ASCII 视图、viewport、HUD、调试叠加
    ├── tests/                       自动化测试（含 README 一致性检查）
    ├── examples/                    可运行示例
    ├── configs/                     场景配置样例（可直接 --scenario 加载）
    ├── docs/                        架构、设计、输入模式、数据集、协议、重构分析、开源参考
    └── cli.py                       命令行入口
```

`bullet_sim/docs/` 另有：`bullet_sim/docs/ARCHITECTURE.md`（架构与实测性能）、
`bullet_sim/docs/DESIGN.md`（设计与接口变更记录）、
`bullet_sim/docs/INPUT_MODES.md`（人工 / 自主 / 开发板输入）、
`bullet_sim/docs/PROTOCOL.md`（CPU/FPGA 协议）、
`bullet_sim/docs/DATASET.md`（数据集）、
`bullet_sim/docs/USAGE.md`（API 快速上手）、
`bullet_sim/docs/REFACTOR_ANALYSIS.md`（动态障碍改造的结构分析）、
`bullet_sim/docs/OPEN_SOURCE_REFERENCES.md`（开源参考与许可审查）。

---

## 16. Installation

```bash
cd /home/zhang/Bullet_Platform

# 只要 NumPy（仿真 + 数据集 + 基准 + 测试全部可用）
pip install -e .

# 需要可视化 / 测试工具
pip install -e ".[dev]"
```

`For_Test/` 是一份**本机便利虚拟环境**（内含 torch/pygame 等，约 5.7 GB），
**刻意不入库**（见 `.gitignore`）。克隆仓库后请用上面两条 `pip install` 之一自建环境：

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
```

零安装也可直接跑：`python3 run_demo.py`（自动把仓库根加入 `sys.path`，只需要 NumPy）。

---

## 17. Quick Start

```bash
# 1) 看平台有什么
python -m bullet_sim info
python -m bullet_sim scenarios
python -m bullet_sim obstacles            # 5 种现实障碍类型 + 现实语义

# 2) 无窗口跑一把（最快验证环境是否正常）
python -m bullet_sim run --level easy --seed 1 --steps 600 --input auto

# 3) 组合一个动态障碍场景，并先校验可行性
python -m bullet_sim run --obstacle-type moving_block --obstacle-type small_obstacles \
       --seed 3 --steps 900 --validate --safe-path

# 4) 走廊：角色在两墙正中间，墙沿通道延伸到场地两端
python -m bullet_sim play --obstacle-type corridor --corridor-motion rotate_same \
       --corridor-change 16 --corridor-width 120 --seed 1

# 5) 打开可视化界面（默认人工控制）
python -m bullet_sim play --level medium --seed 3
#    方向键 / WASD 移动，SPACE 停，SHIFT 聚焦，TAB 切自主，B 切轮廓，滚轮缩放，ESC 退出

# 6) 直接以自主模式启动（含预测叠加）
python -m bullet_sim play --level easy --seed 3 --input auto --auto threat --prediction

# 7) 对比所有自主基线（无窗口）
python -m bullet_sim autoplay --level medium --seed 21 --steps 1200

# 8) headless 批量仿真 + 生成训练数据
python -m bullet_sim record --level medium --count 8 --out data/ --format npz --hashes --steps 800

# 9) 验证数据可以精确复现
python -m bullet_sim replay --dataset data/ep_00000.npz

# 10) 两道"必死"防线：缺口太小直接拒绝；刚好能过但被安全余量封死则判不可行
python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 5 --steps 10
python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 20 \
       --obstacle-speed 200 --safety-horizon 8 --validate --steps 300

# 11) 走廊的"绝不压死"：收缩量再离谱也会被裁到安全值
python -m bullet_sim run --obstacle-type corridor --corridor-motion close \
       --corridor-width 100 --corridor-change 99 --steps 300
```

引导式演示（约 30 秒）：

```bash
python3 run_demo.py            # 默认导览
python3 run_demo.py patterns   # 障碍场景与 level 总控
python3 run_demo.py inputs     # 人工 / 自主 / 开发板三种模式对比
python3 run_demo.py --help
```

---

## 18. Test / 验证

### 18.1 自动化测试

```bash
python -m pytest bullet_sim/tests -q                 # 383 项，约 90 秒
python -m bullet_sim.tests.run_tests                 # 无 pytest 时的内置 runner
BULLET_SIM_NO_PYTEST=1 python -m bullet_sim.tests.run_tests
```

重点测试文件：

| 文件 | 覆盖 |
|---|---|
| `bullet_sim/tests/test_obstacle_environment.py` | 圆形角色（可视 = 碰撞）、5 种类型、边界生成、走廊各运动模式与墙心不动、level 总控与 extreme 安全区、安全 API、HUD、CLI |
| `bullet_sim/tests/test_patterns.py` | 5 个 obstacle layout 的几何性质（区间随机、缺口下限、走廊对称/旋转限角、只保留现实形状） |
| `bullet_sim/tests/test_collision_events.py` | 碰撞 = 惩罚事件的 10 项约定 |
| `bullet_sim/tests/test_readme_consistency.py` | **本 README 与代码一致**（路径 / 命令 / 参数 / 字段） |
| `bullet_sim/tests/test_protocol.py` | 协议 v1/v2/v3 编解码与 stride |
| `bullet_sim/tests/test_determinism.py` | 同 seed 跨进程哈希一致 |

### 18.2 手工验收清单

| # | 验证项 | 命令 / 操作 | 期望 |
|---|---|---|---|
| **1** | 人工控制 | `python -m bullet_sim play --level easy --seed 1` 后按 `→` | 角色向右移动；松开即停；`SHIFT` 变慢 |
| **2** | 自主控制 | `python -m bullet_sim play --level easy --seed 3 --input auto` | 手离开键盘，角色自行避障 |
| **3** | 模式切换 | 运行中按 `TAB` | HUD 在 MANUAL / AUTO 间切换，世界状态连续 |
| **4** | 碰撞 = 惩罚事件 | `python -m bullet_sim run --level medium --seed 21 --steps 400 --input auto` | 碰撞不置位 `terminated`，窗口不关 |
| **5** | 模式 B 对照 | 同上加 `--collision-terminates` | 首次碰撞即 `terminated: True` |
| **6** | Headless | `python -m bullet_sim run --level medium --seed 7 --steps 1200` | 无窗口跑完并打印统计与状态哈希 |
| **7** | 复现性 | 同一条命令跑两次 | `state hash` 完全一致 |
| **8** | 数据集 | 第 17 节第 8、9 步 | `reproduced: True`、`reason: exact match` |
| **9** | 动态障碍类型 | `python -m bullet_sim obstacles` | 打印 5 种类型的现实语义、运动、玩家任务、安全策略 |
| **10** | 障碍场景可行 | `python -m bullet_sim run --obstacle-type moving_block --obstacle-type wall_with_gap --gap-width 180 --validate --safe-path --steps 900` | 打印实测难度与安全路径，退出码 0 |
| **11** | 必死场景被拒 | `python -m bullet_sim run --obstacle-type wall_with_gap --gap-width 20 --obstacle-speed 200 --safety-horizon 8 --validate --steps 300` | 报 `INFEASIBLE`、可达 0 格、退出码 2 |
| **12** | 走廊旋转 | `python -m bullet_sim play --obstacle-type corridor --corridor-motion rotate_same --corridor-change 16 --seed 1` | 两墙绕各自中心同向旋转，墙心不动，间距不变 |
| **13** | 窗口缩放 | 拖动窗口 / 滚轮 / `B` | 画面等比缩放不拉伸；角色圆的画面大小 = 碰撞大小 |
| **14** | 走廊延伸与间距 | 第 8.3 节命令，改 `--corridor-width` / `--corridor-min-width` / `--wall-length` | 间距随之变化；低于 `1.25d` 直接报错 |

---

## 19. Current Status

| 能力 | 状态 | 备注 |
|---|---|---|
| Simulator 内核（固定 dt / SoA 池 / 生命周期） | ✅ 已完成 | 稳态零分配、可复现 |
| **Player 是圆：可视 = 碰撞，半径可调** | ✅ 已完成 | `--player-radius`；无"车身 vs hitbox"两套尺寸 |
| **DynamicObstacle（shape/half_w/half_h/rotation，协议 v3）** | ✅ 已完成 | 兼容读 v1/v2 |
| **矩形 `angular_velocity` = 身体自转** | ✅ 已完成 | 走廊"墙心不动只旋转"靠它实现；圆仍是曲线弹 |
| **5 种现实障碍类型 + 目录（含现实语义）** | ✅ 已完成 | `moving_wall` / `closing_gap` / `mixed_obstacles` 已删除；`sparse`+`dense` 合并 |
| **走廊：对称双墙、跨场延伸、张开/收缩/同向/反向旋转** | ✅ 已完成 | 间距由 `--corridor-width` / `--corridor-change` / `--corridor-min-width` 控制 |
| **装饰性弹幕模式（radial/spiral/line/wall/...）** | ✅ **已删除** | `available_patterns() == ["obstacle"]` |
| **level = 单调总控（速度 / 数量 / 类型数）** | ✅ 已完成 | `speed_scale` / `count_scale` / `obstacle_kinds` |
| **每个 level 实测安全区（extreme 也保证）** | ✅ 已完成 | 自由空间 ≥15%、可达 ≥12 格，否则自动减薄 |
| **安全校验（自由空间 + 可行路径 + 自动重采样）** | ✅ 已完成 | 6 个公开 API；不可行时退出码 2 |
| **相对运动 `v_rel = v_obs − v_player`** | ✅ 已完成 | 速度 / 接近率 / 撞击时间 / 玩家系快照 |
| **窗口缩放（viewport，与仿真解耦）** | ✅ 已完成 | 等比 letterbox |
| Collision（圆 / 点 / 矩形 / 网格 / 障碍 SDF） | ✅ 已完成 | 6 个可替换后端 |
| 碰撞 = 惩罚事件（进入/退出状态机 + 可配置惩罚） | ✅ 已完成 | 默认 `collision_terminates_episode=false` |
| Episode 终止条件可配置（时间 / 目标 / 显式 / 模式 B） | ✅ 已完成 | |
| 人工控制（键盘 / WASD / 斜向 / 聚焦 / 停止） | ✅ 已完成 | 有自动化测试 |
| Baseline Autonomous Control | ✅ 已完成 | `threat` 为预测式参考，`planner` 最强 |
| 运行时 Manual ↔ Autonomous 切换 | ✅ 已完成 | `TAB` |
| 可视化 + HUD + 世界坐标 + 碰撞轮廓 | ✅ 已完成 | 碰撞闪红提示 |
| Headless simulation | ✅ 已完成 | 内核不依赖渲染 |
| Dataset（5 种格式 + 自复现 meta + 精确 replay） | ✅ 已完成 | |
| Prediction（弹道 / rollout / 危险场） | ✅ 已完成 | |
| Benchmark（FPS / 延迟 / 碰撞 / 预测 / 内存） | ✅ 已完成 | `python -m bullet_sim benchmark` |
| State 帧协议 v3（兼容读 v1/v2） | ✅ 已完成 | CRC32 + 固定 stride |
| Prediction 帧协议 + FPGA Python 参考块 | ✅ 已完成（参考模型） | `[SOFTWARE_REFERENCE_ONLY]`，**非 HDL** |
| CPU 决策层（只吃字节帧） | ✅ 已完成 | 精度低于直接读实体的基线，属已量化偏差 |
| **开发板真实通信** | ⏳ **尚未接入** | `[HARDWARE_INPUT_INTERFACE_TBD]`（输入适配器）/ `[HARDWARE_INTERFACE_TBD]`（FPGA） |
| **FPGA RTL 实现** | ⏳ **尚未开始** | 只有 Python golden model 与帧格式 |
| **训练好的 AI 模型** | ⏳ **`[MODEL_NOT_AVAILABLE_YET]`** | 接口已就绪，模型需自行训练 |
| **30TAI / Icraft 量化与上板** | ⏳ **`[MODEL_DEPLOYMENT_TBD]`** | 工具链未确定，不猜测 |
| CPU / FPGA 带宽与延迟实测 | ⏳ **`[TBD]`** | 依赖上一条 |
| 现实二维小车映射 | ⏳ 未开始 | 核心状态量与尺寸比例已按可迁移设计 |

**已知不足（不隐藏）**

1. `threat` 基线（解析预测）在密集场景下弱于 `planner`（真实 rollout），差距来自直线路径近似；
2. CPU/FPGA 栅格通路的命中精度低于直接读实体的基线，属**栅格量化偏差**；
3. `collision_penalty` 目前只实现"每次事件固定惩罚"；次数 / 持续时间 / 严重度加权属后续扩展
   （`ContactTracker` 已记录 `overlap_frames` 等原始量，扩展不需要改内核）；
4. **安全校验是保守近似**：格点离散 + 矩形按外接圆，可能把个别实际可行场景判为不可行；
   密集场景单次校验耗时随实体数增长，适合生成期校验一次而非每帧；
5. **相邻实体之间不判碰撞**：障碍彼此可以穿过，只有"玩家 ↔ 障碍"参与判定；
6. **走廊反向旋转的角度受几何限制**：墙越长、允许的剪刀角越小（见 8.3）；
   想要明显的"路面偏转"请用 `rotate_same`；
7. 性能数字随机器与场景变化，请以 `python -m bullet_sim benchmark` 现场生成为准
   （旧的"10000 弹幕"数字来自已删除的装饰性弹幕场景，不可直接对比）。

---

## 20. Future Hardware Integration

拿到真板卡后的推荐顺序：

1. **确定链路**（见第 5 节），把 `bullet_sim/hardware_interface/tbd.py` 填成真实值；
2. 实现 `HardwareInputAdapter` 子类，先用 `LoopbackHardwareInputAdapter` 对照验证 `Action` 语义；
3. 用 `FpgaPredictionReference` 当 golden model 写 HDL 测试向量：同一 state frame 必须得到
   逐位相同的 prediction frame；
4. 把 `CpuFpgaPipeline` 的 `fpga` 参数换成真实链路，CPU 决策层不动；
5. 用 `python -m bullet_sim benchmark` 对比 **CPU 侧时间 / FPGA 侧时间 / 端到端延迟**，
   并记录 LUT / FF / BRAM / DSP；
6. 最后把虚拟环境换成真实传感器输入——state frame 的生产者变了，
   其余（协议、CPU 决策、FPGA 预测）完全不变。

**边界原则**：`Simulator ↔ Action ↔ CPU ↔ FPGA` 之间只通过本文件记录的两帧字节协议与
`Action` 通信；任何一侧的内部实现都可以替换，前提是这两处不破。

---

## 21. 开源参考

`Keiki-master/` 与 `TostEngine-cinder-bullethell-main/` 仅作为**技术思路**来源，
未复制任何代码或数据：

* **TostEngine**（MIT）：`Velocity{linear, angular}` 的角速度概念、空间网格碰撞、
  定长实体池、固定步长主循环、人工操作方式；
* **Keiki**（**未找到 LICENSE 文件** → `[LICENSE_REVIEW_REQUIRED]`）：builder 式弹幕组织、
  极坐标弹幕表、延迟合并列表、未来帧预演。

本项目为独立 Python 实现，并额外提供自己的 State / Action / Dataset / Hardware Interface。
逐条对比与许可证审查见 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md`。
