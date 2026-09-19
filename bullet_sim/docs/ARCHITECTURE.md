# Bullet-Hell Simulator 平台设计文档

> 项目定位：**虚拟环境训练 → CPU/FPGA 异构部署 → 现实二维小车验证** 的公共基础设施。
> 弹幕 = 动态障碍物的一种表现形式，不是游戏内容本体。
> 优先级：**正确性 > 可复现性 > 模块化 > 数据接口 > 性能 > UI 美观**。

---

## 1. 系统架构

### 1.1 分层视图

严格单向依赖，上层可替换、下层零依赖：

```
┌──────────────────────────────────────────────────────────────────────┐
│ L6 应用层      cli  /  examples  /  experiments                       │
├──────────────────────────────────────────────────────────────────────┤
│ L5 工具层      render/     dataset/    benchmark/    prediction/     │
│                (可视化)     (数据集)     (性能)        (预测/危险场)   │
├──────────────────────────────────────────────────────────────────────┤
│ L4 接口层      interface/  (Simulator API · State/Action 协议 · HAL)  │
│                simulator/  (World 确定性内核 · Env Gym 风格 · 快照)   │
├──────────────────────────────────────────────────────────────────────┤
│ L3 场景层      scenarios/  (ScenarioSpec · 参数化难度 · 预设 · 构建)  │
│                obstacles/  (ObstacleType 目录 · ObstacleScenario 组合)│
│                safety/     (自由空间 · 可行路径 · 场景可行性校验)      │
├──────────────────────────────────────────────────────────────────────┤
│ L2 机制层      generators/  collision/  physics/                     │
│                (纯函数/纯数据：不持有世界引用，可单测、可移植)          │
├──────────────────────────────────────────────────────────────────────┤
│ L1 核心层      core/  (SoA 状态 · RNG · 时钟 · 动作 · 配置)           │
│                entities/ (Player/Bullet/Target 的 SoA 视图与语义)     │
└──────────────────────────────────────────────────────────────────────┘
```

**关键约束**

| 规则 | 说明 |
|---|---|
| 内核无 UI | `simulator/` 及以下不 import 任何渲染库；`render/` 只读快照 |
| 内核无 AI | 算法通过 `ActionProvider` 协议注入，模拟器不 import 任何策略库 |
| 内核无随机 | 关卡随机在 **构建期** 用种子展开为确定性的 `SpawnEvent` 时间线；步进本身不含 RNG |
| 内核无分配 | 弹幕池按容量预分配，步进过程中零动态分配（稳态） |
| 预测与物理分离 | `prediction/` 只调用 `world.clone()/step()`，不修改世界；物理无关预测器可替换 |

### 1.2 数据流

```
ScenarioSpec(JSON) + seed
        │
        ▼  build_scenario()
SpawnTimeline  ─┐
                ├─► World.reset() ──► S_0
PlayerInit ─────┘                       │
                                        ▼  World.step(action)   [固定 dt]
                          ┌─────────────┴─────────────┐
                          │ 1 应用动作 (player)        │
                          │ 2 到期 SpawnEvent 落地     │
                          │ 3 弹幕运动积分             │
                          │ 4 边界/寿命剔除            │
                          │ 5 碰撞检测 (可替换模型)    │
                          │ 6 奖励/终止判定            │
                          │ 7 记录 (可选 Recorder)     │
                          └─────────────┬─────────────┘
                                        ▼
                       S_{t+1}, reward, terminated, truncated, info
                                        │
     ┌──────────────────────────────────┼───────────────────────────────┐
     ▼                ▼                 ▼                ▼              ▼
 DatasetWriter   Renderer        Predictor      Benchmark        HardwareLink
 (npz/csv/bin)   (pygame/ascii)  (rollout)      (latency/FPS)    (CPU/FPGA)
```

---

## 2. 模块划分

```
bullet_sim/
├── core/            核心层：与硬件/算法无关的最基础构件
│   ├── config.py        配置 dataclass + JSON/YAML 往返 + 校验
│   ├── state.py         WorldSnapshot / PlayerState / BulletPool / TargetState / EnvState
│   ├── rng.py           SeedManager：主种子 → 命名子流 (PCG64/Philox)，无全局随机
│   ├── clock.py         FixedClock：dt / fps 换算、步数、仿真时间
│   ├── actions.py       ActionCodec：discrete / continuous / vector 统一编码
│   ├── numerics.py      dtype 策略、状态哈希、缓冲复用工具
│   └── errors.py
├── entities/        语义层：状态字段的读写视图（不复制内存）
│   ├── player.py        Player：位置/速度/半径/最大速度
│   ├── bullet.py        BulletPool：SoA + free-list 分配/回收/视图
│   └── target.py        TargetZone：目标区域（矩形/圆/点）
├── physics/         纯运动学
│   ├── motion.py        integrate_bullets(位置/速度/加速度)
│   ├── kinematics.py    apply_discrete_action / apply_velocity_command
│   └── bounds.py        出界与寿命剔除、环绕策略
├── collision/       碰撞模型（可替换）
│   ├── base.py          CollisionModel 协议 + CollisionResult(id/frame/距离/风险)
│   ├── circle.py        圆-圆解析解（向量化）
│   ├── shapes.py        circle / point / rect hitbox（SDF）+ ShapedCollision
│   └── grid.py          均匀网格宽相（超大规模 / 批量环境）
├── generators/      参数化生成器（构建期，无副作用）★ 规格中的 patterns/
│   ├── spec.py          PatternSpec 数据类（所有可配置参数）
│   ├── burst.py         SpawnEvent / SpawnTimeline
│   ├── patterns.py      5 种动态障碍 layout（block/wall_gap/small/cross/corridor）
│   ├── composite.py     多 pattern 叠加、时间线调度、repeat/interval
│   └── registry.py      名字 → 生成器，配置驱动
├── obstacles/       ★ 动态障碍类型系统
│   ├── spec.py          ObstacleType（含现实语义）/ ObstacleSpawn / SIZE_PRESETS（相对玩家圆直径）
│   ├── catalog.py       5 种现实障碍类型目录（无装饰性弹幕类型）
│   └── scenario.py      ObstacleScenario：组合、展开 mixed、to_spec、JSON 往返
├── safety/          ★ 场景可行性（有挑战 ≠ 必死）
│   ├── free_space.py    配置空间膨胀 + 自由空间栅格 / 连通域
│   ├── path_search.py   到达时间波前可达性 + 轨迹回溯
│   └── validate.py      validate / is_scenario_valid / find_safe_path / 自动重采样
├── scenarios/       场景系统
│   ├── spec.py          ScenarioSpec + JSON 往返
│   ├── complexity.py    complexity ∈ [0,1] → 具体参数（连续难度）
│   ├── params.py        ComplexityParams：难度 = 可解释的参数函数
│   ├── presets.py       easy / medium / hard / extreme（只是参数预设）
│   └── builder.py       ScenarioSpec → (初始状态, SpawnTimeline)
├── simulator/       ★ 确定性仿真内核
│   ├── world.py         World：reset/step/clone/restore/state_hash
│   ├── env.py           BulletHellEnv：Gymnasium 风格 reset/step/get_state/render/close
│   ├── rewards.py       RewardFunction 协议 + 默认实现
│   └── replay.py        Replay：scenario + seed + actions → 轨迹
├── prediction/      （与物理内核解耦）
│   ├── base.py          Predictor 协议
│   ├── ballistic.py     匀速/匀加速解析外推（零成本基线）
│   ├── rollout.py       simulate_future 的蒙特卡洛/枚举 rollout
│   └── danger_field.py  未来危险场 (H×H 栅格, 随时间衰减)
├── interface/       ★ Simulator Interface 与 Hardware Interface 分离
│   ├── space.py         Space 描述（obs/action），不依赖 gymnasium
│   ├── protocol.py      State/Action 扁平二进制协议（含 CRC/版本/magic）
│   ├── controller.py    ActionProvider / Controller 协议（外部 AI 接入点）
│   └── hardware.py      HardwareLink ABC + LoopbackLink / SerialLink 桩
├── dataset/         面向 ML 的数据集
│   ├── schema.py        版本化字段表（transition / episode）
│   ├── recorder.py      TrajectoryRecorder（headless 友好）
│   ├── writers.py       json / npz / csv / binary
│   └── readers.py       读回并校验一致性
├── benchmark/       性能
│   ├── metrics.py       计时器、分位数
│   ├── runner.py        微基准与端到端基准
│   └── suites.py        标准性能套件（含高密度压力）
├── render/          可视化（只读）
│   ├── base.py          Renderer 协议
│   ├── pygame_view.py   pygame 实时视图（FPS/弹幕数/状态/调试覆盖层）
│   ├── ascii_view.py    无依赖文本视图（CI/SSH 可用）
│   └── overlays.py      危险区/预测轨迹/HUD 绘制
└── cli.py           命令行入口
tests/  examples/  configs/  docs/  benchmarks_output/
```

---

## 3. 核心数据结构

### 3.1 结构选择：SoA + 预分配 + 自由列表

面向 FPGA/CPU 的规则内存布局，避免逐弹对象与运行期分配。

```python
class BulletPool:                     # 容量 CAP 固定
    capacity: int
    count: int                        # 当前存活数
    next_id: int                      # 单调递增 id 源（跨 compact 稳定）
    # 每个字段一条连续数组，dtype 可配 (默认 float64，可切 float32 对齐 FPGA)
    id            : int32[C]          # ★ 规格字段，碰撞报告/追踪使用
    x, y          : float32[C]        # ★ x, y
    vx, vy        : float32[C]        # ★ vx, vy
    ax, ay        : float32[C]        # ★ acceleration
    angle         : float32[C]        # ★ 速度方向（每步同步）
    angular_velocity : float32[C]     # ★ 方向旋转率（曲线弹）
    radius        : float32[C]        # ★ radius
    age, ttl      : float32[C]        # ★ lifetime
    type_id       : int16[C]          # ★ type
    group_id      : int16[C]
    alive         : bool[C]           # ★ active
    free_slots    : int32 stack       # O(1) 分配/回收，确定性顺序
```

`vx,vy` 与 `angle,angular_velocity` 是同一运动的两种视图：`angle` 每步由速度重算，
`angular_velocity != 0` 时由它驱动速度旋转（精确圆弧积分，见 `physics/motion.py`）。

- `alive` 掩码 + `count` 让“活弹压缩”可选：`compact()` 得到紧凑前缀，便于导出/送 FPGA。
- 所有字段在步进中被**整批向量化**更新：`x += vx*dt`（仅活跃掩码），无 Python 循环。

### 3.2 状态快照 S_t

```python
@dataclass(frozen=True)
class EnvState:      field_w, field_h, dt, step_index, sim_time, wrap_mode, bullet_budget
@dataclass(frozen=True)
class PlayerState:   x, y, vx, vy, radius, speed, alive
@dataclass(frozen=True)
class TargetState:   x, y, radius, shape ('circle'|'rect'), half_w, half_h
@dataclass
class WorldSnapshot:
    player: PlayerState
    bullets: BulletPool        # SoA
    target: TargetState
    env: EnvState
    rng_state: dict | None     # 仅当启用运行期随机时使用
    scenario_id: str
    seed: int
    timestamp: float           # 仿真时间戳 (step_index * dt)，非墙钟
```

`S_t = {player_state, bullet_states, target_state, environment_state, timestamp}`

**能力**：`to_dict()/from_dict()`、`to_flat()/from_flat()`、`copy()`、`state_hash()`（blake2b over 规范化字节）。

### 3.3 状态哈希（复现性判定）

`state_hash` 对 `step_index + player + 活跃弹幕前缀(按 slot 升序) + target` 做规范化哈希。
两种仿真得到同一 hash ⇒ 状态完全一致。测试与 CI 直接用它断言确定性。

### 3.4 硬件扁平布局（State Protocol v3，兼容读 v1/v2）

```
Header (84 B, 小端)
  [0]  magic u32 = 'BHL1'       [4]  version u16 = 3
  [6]  flags u16                [8]  step_index u32
  [12] n_bullets u32            [16] sim_time   f32
  [20] field_w   f32            [24] field_h    f32
  [28] dt        f32
  [32..60)  player f32[7]  (x, y, vx, vy, radius, speed, alive)
  [60..84)  target f32[6]  (x, y, radius, shape, half_w, half_h)

Bullet block: stride = 17 × f32 = 68 B   (v3)
  (x, y, vx, vy, ax, ay, angle, angular_velocity, radius, age, ttl,
   type_id, group_id, shape, half_w, half_h, rotation)

  v2 stride = 13 × f32 = 52 B   (无 shape/half_w/half_h/rotation)
  v1 stride = 10 × f32 = 40 B   (无 angle/angular_velocity/age)

Trailer: crc32 u32 over all preceding bytes
Total := 88 + 68 * n_bullets 字节 (v3)
```

固定 stride ⇒ 可直接映射到 FPGA BRAM/DMA 描述符；新增字段只改 stride 常量并升 version。
解 v1/v2 帧时自动回填几何字段默认值（`shape=0, half_w=half_h=radius, rotation=0`）。
实现见 `interface/protocol.py`，详细说明见 `docs/PROTOCOL.md`。

---

## 4. State / Action 接口

### 4.1 语义约定

| 项 | 定义 |
|---|---|
| 坐标系 | 原点左下，x 向右，y 向上；单位 = 像素（可标定为米） |
| 时间 | `t = step_index * dt`，`dt` 固定；渲染帧率与仿真解耦 |
| `observation` | 结构体（dict-like）**且**可展平为 `float32[K]`；两者一致 |
| 动作 | 编码为统一 `np.ndarray`，`ActionCodec` 负责离散↔连续转换 |

### 4.0 统一 Action（规格 §3/§4）

```python
@dataclass(frozen=True)
class Action:
    direction: tuple[float, float] = (0.0, 0.0)   # 单位向量
    magnitude: float = 1.0                        # [0, 1]
```

`ActionSource` 协议是输入设备的唯一契约（`open/close/poll(dt) -> Action | None`）。
`action/` 下禁止 import `simulator`，从结构上保证"输入模块不修改环境状态"。
`action_to_codec_input()` 把统一 Action 适配到既有的 discrete/velocity/acceleration 编解码器。

### 4.2 观测（分三层，按需取用）

```python
observation = {
  # --- 固定维度，RL 可直接用 ---
  "player":       float32[6],    # x,y,vx,vy,radius,speed  (归一化到 [0,1] 可选)
  "target":       float32[4],    # x,y,radius,dist_to_player
  "env":          float32[6],    # field_w,field_h,sim_time,step_index,density,n_alive
  # --- 变长，供预测/规划/监督学习 ---
  "bullets":      float32[N,7],  # x,y,vx,vy,radius,ttl,type  (紧凑前缀)
  "bullets_mask": bool[N],
  # --- 监督信号 ---
  "collision":    bool,
  "danger":       float32|None,  # 可选：当前危险度（由 prediction 提供时填充）
}
```

### 4.3 动作空间

**离散（v1，默认）**：9 个动作 → 单位方向 × `player.speed`

| id | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|---|
| name | stay | left | right | up | down | up_left | up_right | down_left | down_right |
| dir | (0,0) | (-1,0) | (1,0) | (0,1) | (0,-1) | (-1,1) | (1,1) | (-1,-1) | (1,-1) |

对角向量归一化（`1/√2`），保证各方向速度一致。

**统一 Action Interface（可替换）**

```python
class ActionCodec(Protocol):
    dim: int                                # 动作向量维度
    def encode(self, action) -> np.ndarray  # Any -> float32[dim]
    def decode(self, vec) -> Any

# 内置实现
DiscreteActionCodec(9)                      # int -> 单位方向
VelocityActionCodec(2)                      # (vx,vy) 直接给定速度
AccelerationActionCodec(2)                  # (ax,ay) 速度积分（带 |v|<=speed 限制）
```

`World.step(action)` 内部只调用 `codec.encode`，因此策略可输出 int / 向量 / 由 CPU-FPGA 回传的缓冲区，内核无感。

### 4.4 外部 Controller 协议

```python
class ActionProvider(Protocol):
    def reset(self, observation, info) -> None: ...
    def act(self, observation, info) -> Any: ...
```
规则控制器、模仿学习策略、MPC、CPU/FPGA 决策模块均实现该协议即可接入。

---

## 5. Simulator API

### 5.1 World（确定性内核，无渲染无 AI）

```python
world = World(scenario, seed=1234)             # 或 World.from_snapshot(snap)
world.reset(seed=None) -> WorldSnapshot
world.step(action)      -> StepResult(observation, reward, terminated, truncated, info)
world.get_state()       -> WorldSnapshot
world.set_state(snap)
world.clone()           -> World               # 深拷贝，互不影响（含 RNG 状态）
world.state_hash()      -> str
world.active_bullet_count() -> int
world.spawn(batch)                             # 手动注入弹幕（调试/外部注入）
```

### 5.2 Env（Gymnasium 风格，不强制依赖 gymnasium）

```python
env = BulletHellEnv(scenario, seed=..., codec=..., reward=..., collision=..., record=None)
obs, info = env.reset(seed=1234)
obs, reward, terminated, truncated, info = env.step(action)
state = env.get_state()
env.render(mode="human"|"rgb_array"|"ascii")   # 无 render 依赖时静默跳过
env.close()
env.action_space / env.observation_space       # bullet_sim.interface.space
env.state_hash()
# 未来推演（不改变当前世界）
env.simulate_future(action_sequence, horizon, stride=1) -> list[WorldSnapshot]
env.clone_state() -> WorldSnapshot
env.restore_state(snap)
```

### 5.3 未来推演

```python
rollout = world.clone()
for a in action_sequence[:H]:
    rollout.step(a)
# simulate_future 同时提供 rollouts(actions_batch) 批量候选动作评估
```
`prediction/` 在此之上提供：匀速解析外推（无 step 开销）、枚举/采样 rollout、未来危险场栅格化。**预测模块不进入内核，内核只暴露 clone/step。**

### 5.4 数据集 API

```python
rec = TrajectoryRecorder(scenario_id, seed, fields=[...])
env = BulletHellEnv(scenario, record=rec)
...  # 每个 step 自动写入 transition
rec.save("out/ep0001")        # 依扩展名分派: .json / .npz / .csv / .bin
load_dataset("out/ep0001.npz")
```
Transition 字段（版本化 schema）：
`initial_state, state_t, action_t, next_state, collision, reward, terminated, truncated, done, scenario_id, seed, timestamp, step_index, bullet_count, state_hash`

### 5.5 Hardware Interface（与模拟器接口分离）

```python
class HardwareLink(ABC):
    def upload_state(self, frame: bytes) -> None      # CPU → FPGA 输入缓冲
    def fetch_prediction(self) -> bytes               # FPGA → CPU 预测输出缓冲
    def send_action(self, action_vec) -> None         # CPU → FPGA 决策
    def latency_stats(self) -> dict
```
`LoopbackLink`（同进程）、`FileLink`（落盘/回放）、`SerialLink`/`UDPLink`（桩）。
虚拟仿真与真实小车共用同一 `State/Action` 协议：
`Virtual Simulator → CPU → FPGA` 与 `Physical Sensor → CPU → FPGA` 只替换最上游数据源。

---

## 6. 推荐技术栈

| 层 | 选型 | 理由 |
|---|---|---|
| 语言 | **Python 3.12** | 研究迭代快；已有 Keiki/TostEngine 参考实现 |
| 数值 | **NumPy ≥ 2**（SoA 向量化） | 连续内存、批量更新、天然映射 FPGA 布局 |
| 随机 | `numpy.random.Generator(PCG64)` | 显式种子、可序列化状态、跨平台一致；禁用全局 `np.random` |
| 渲染 | **pygame**（可选依赖） | 2D 覆盖层成熟；缺失时自动降级 ascii |
| 曲线/图表 | matplotlib（可选） | 仅供离线 debug 绘图 |
| 数据 | npz / csv / json / 自定义二进制 | 覆盖训练、实验记录、调试、硬件传输 |
| 测试 | **pytest**（缺失时自带 `run_tests.py` 回退） | 标准 |
| 加速（后续） | Numba / C++ 扩展 / 多进程 batch env | 接口不变，仅替换内核后端 |
| 构建/打包 | `pyproject.toml` + 纯 Python 包 | 零编译依赖 |

**明确不引入**：ECS 框架、物理引擎、深度学习框架、游戏引擎、gym 强依赖。

---

## 7. 开发阶段划分（全部已交付 + 验证方式）

| Phase | 内容 | 验收证据 |
|---|---|---|
| **P1** | core(state/rng/clock/actions/config) + player + 单弹幕 + 固定 dt | `tests/test_motion.py`：匀速/匀加速与解析解逐位一致；30/60/120/240 Hz 下 1 秒位移相同 |
| **P2** | BulletPool(SoA+free-list) + 参数化生成器 + 圆碰撞 | `tests/test_pool_actions.py`、`test_collision.py`（含 grid 与 brute-force 一致性） |
| **P3** | 全 pattern + composite + ScenarioSpec + complexity/presets + seed | `tests/test_patterns.py`、`test_scenario.py`；`tests/test_determinism.py` 含**跨进程**哈希一致 |
| **P4** | headless 批量仿真 + dataset recorder + json/npz/csv/bin | `tests/test_dataset.py`（JSON/NPZ/BIN 逐位一致）、`test_headless.py`、`examples/03_*` |
| **P5** | BulletHellEnv (reset/step/get_state/render/close) + ActionProvider | `tests/test_env_interface.py`、`examples/04_*` |
| **P6** | clone_state / simulate_future / ballistic + danger field | `tests/test_prediction.py`、`examples/05_*` |
| **P7** | benchmark：sim FPS / step latency / 碰撞吞吐 / 生成速度 | `examples/06_*`、`benchmarks_output/ladder_full.json` |
| **P8** | pygame 视图 + ascii 视图 + HUD/危险区/预测轨迹 + CLI | `tests/test_headless.py`、`examples/08_*`、`python -m bullet_sim play` |

测试规模：**133 项**，`python -m pytest bullet_sim/tests` 或
`python -m bullet_sim.tests.run_tests`（无 pytest 依赖）均可全绿。

**交付验收（10 项）**：① 一键生成指定难度场景 ② 一键运行指定弹幕量 ③ 一键 headless 批量仿真 ④ 自动保存完整状态轨迹 ⑤ 自动生成训练数据 ⑥ 固定 seed 完全复现 ⑦ 支持外部 AI controller ⑧ 支持未来状态预测 ⑨ 支持性能 benchmark ⑩ 保留清晰 CPU/FPGA 数据协议。

### 实测性能（120 Hz，`circle` 碰撞，单核；`benchmarks_output/ladder_v2_full.json`）

`python -m bullet_sim benchmark --ladder 100 500 1000 2000 5000 10000 --prediction --memory --full`

| 目标弹幕 | 实测峰值 | 测量窗口均值 | 单步均值 | 仿真 FPS | 实时倍率 | us/bullet/step | 碰撞均值 |
|---|---|---|---|---|---|---|---|
| 100 | 95 | 77.7 | 0.113 ms | 8 878 | 74x | 1.45 | 12 µs |
| 500 | 496 | 416.0 | 0.122 ms | 8 204 | 68x | 0.29 | 15 µs |
| 1000 | 994 | 861.6 | 0.176 ms | 5 682 | 47x | 0.20 | 18 µs |
| 2000 | 1999 | 1565.1 | 0.332 ms | 3 017 | 25x | 0.21 | 27 µs |
| 5000 | 4967 | 4735.0 | 0.560 ms | 1 785 | 15x | 0.12 | 52 µs |
| **10000** | **9866** | **9538.0** | **1.449 ms** | **690** | **5.75x** | **0.15** | 91 µs |

弹幕量标定误差 **< 1.5%**（标定器跑真实 `World`，因此 `aimed` 弹幕按实时玩家位置求解——
早期用"固定玩家位置"的廉价近似会低估这类场景约 20%，已修正）。

### 未来预测延迟（H = 90 步 = 0.75 s）

| 弹幕 | 解析外推 ballistic | clone+rollout | 危险场栅格 |
|---|---|---|---|
| 410 | 0.45 ms | 14.8 ms | 5.8 ms |
| 1 631 | 3.8 ms | 37.9 ms | 10.8 ms |
| 9 471 | 24.7 ms | 141.9 ms | 55.6 ms |

结论：**解析外推比逐步 rollout 快约 6–8 倍**，且解码后可直接映射到 FPGA 的
定点乘加流水线；rollout 才是需要 CPU/FPGA 协同（或降 horizon / 采样）的部分。

### 内存占用与稳态分配

| 弹幕 | 存活 | 池容量 | 状态内存 | B/弹幕 | **每步分配** |
|---|---|---|---|---|---|
| 100 | 78 | 182 | 0.017 MiB | 210 | 2.2 B |
| 1 000 | 835 | 1 306 | 0.121 MiB | 141 | 4.1 B |
| 5 000 | 4 718 | 6 272 | 0.580 MiB | 136 | 4.2 B |
| 10 000 | 9 540 | 12 396 | 1.147 MiB | **127** | 4.2 B |

**`alloc_per_step_bytes ≈ 4 B`** 直接验证了"稳态零分配"的设计声明
（剩余量来自 `info` 字典与统计对象），这是能安全映射到 DMA 固定缓冲的前提。

### 碰撞后端对比（1631 存活弹幕）

| 后端 | 1 个玩家 | 8 个玩家 | 64 个玩家 |
|---|---|---|---|
| `circle`（暴力向量化） | 30 µs | 111 µs | 2 000 µs |
| `grid`（均匀网格宽相） | 28 µs | 204 µs | **271 µs** |

结论：**单玩家用暴力向量化最快**（一次 O(N) 扫描胜过建桶开销）；
**玩家数 ≥ 8 时网格宽相胜出**，64 玩家时约 **7.4× 加速、3.9e8 tests/s**。
批量环境（向量化 RL / 策略搜索 / 多候选评估）应使用 `grid`。

### 其它

* **原始运动积分**（不含生成/碰撞/奖励）：10 000 弹幕 596 µs/步 → **1.68e7 弹幕更新/秒**
* **场景生成**：`stress(10000)` 完整构建（含实测标定）约 13 s，一次性成本，可序列化为 JSON 复用

## 8. 复现性与确定性规则（硬约束）

1. 所有随机性来自 `SeedManager`，主种子显式传入；严禁 `np.random.*` 全局函数。
2. 场景构建期把随机展开为**确定性 `SpawnTimeline`**（按 step 排序、同 step 按声明顺序）；步进期无随机。
3. `aimed` 等依赖玩家位置的信息以 `aim` 标志延迟到 spawn 时刻解析，仍为确定性。
4. 浮点：默认 `float64`；可选 `float32` 用于 FPGA 对齐（文档标注精度差异）。同 dtype 同平台逐位一致。
5. 无 Python `set`/`dict` 迭代顺序依赖；活跃弹幕一律按 slot 升序遍历。
6. `WorldSnapshot.state_hash()` 作为复现性断言基准。
7. 场景配置与 seed 随数据集一起持久化，保证任何产物可回放。
