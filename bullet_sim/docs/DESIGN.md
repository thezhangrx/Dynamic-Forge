# 虚拟动态障碍环境平台 — 设计（v3：现实障碍体系 + 圆形角色 + 安全校验闭环）

> 本文件记录**当前**的设计决策与接口变更；更完整的分层说明见 `ARCHITECTURE.md`，
> 动手示例见 `USAGE.md`，本次改造的先导结构分析见 `REFACTOR_ANALYSIS.md`。
>
> 核心原则不变：**正确性 > 可复现性 > 模块化 > 数据接口 > 性能 > UI 美观**；
> Simulator 是核心，渲染/UI/输入设备都不是。

### v3 变更摘要（相对 v2）

| # | 变更 | 位置 |
|---|---|---|
| 1 | 渲染层引入统一 `Viewport`：窗口缩放只改观察尺度，物理世界与 `dt` 不变 | `render/viewport.py` |
| 2 | Player 是**一个圆**：`player_radius` 既是碰撞体也是画面大小（可视 = 碰撞），半径可调 | `scenarios/spec.py` |
| 3 | Bullet → **DynamicObstacle**：`shape/half_w/half_h/rotation`，协议 **v3**（兼容读 v1/v2） | `entities/bullet.py`、`interface/protocol.py` |
| 4 | 新增 **obstacles/** 体系（5 种现实障碍 + `ObstacleScenario` 组合），删除全部装饰性弹幕模式与 `mixed_obstacles` | `obstacles/`、`generators/patterns.py` |
| 5 | 难度 → **参数空间 + 实测 complexity**（自由空间 / 可达集 / 余量 / 相对速度） | `scenarios/params.py`、`safety/validate.py` |
| 6 | 新增 **safety/** 可行性闭环（配置空间膨胀 + 时窗路径搜索 + 自动重采样） | `safety/` |
| 7 | 相对运动语义 `v_rel = v_obs − v_player` | `physics/relative.py` |

---

## 1. 总体架构

```
                          ┌──────────────────────── 输入设备层（只产出 Action）────────────────────────┐
                          │  KeyboardSource        ScriptedSource       HardwareInputAdapter(TBD)    │
  Mode A 人工操纵  ────────┤  (pygame/其他键盘)     (文件/回放/外部程序)  (开发板)                    │
  Mode B 开发板操纵 ───────┤        │                     │                        │                │
                          └────────┼─────────────────────┼────────────────────────┼────────────────┘
                                   ▼                     ▼                        ▼
                        ╔══════════════════════════════════════════════════════════════╗
                        ║           统一 Action Interface                              ║
                        ║   Action = { direction:(dx,dy) 单位向量, magnitude:float }   ║
                        ║   设备 → Action 的转换全部发生在输入层；输入层不接触环境状态  ║
                        ╚══════════════════════════════════════════════════════════════╝
                                                   │  ActionCodec（可替换）
                                                   ▼
        ┌────────────────────────── Simulator Core（headless 友好，无渲染/无 AI）────────────────────┐
        │ World : 固定 dt · SoA 弹幕池 · spawn_pattern() · 运动积分 · 碰撞 · 边界/寿命 · 快照/复现    │
        │ Env   : reset/step/get_state/render/close · observe() · flatten() · simulate_future()       │
        └──────────────────────────────────────────────────────────────────────────────────────────────┘
             │                    │                      │                      │
             ▼                    ▼                      ▼                      ▼
        render/ (pygame/ascii)  dataset/ (json/npz/    prediction/ (ballistic  benchmark/
                                csv/bin)               /rollout/danger)       (FPS/latency/mem)
             ▲
             └── render 只读取 S_t 与 Action，绝不写入环境；键盘事件在 render 层被翻译成
                 `press(name)/release(name)` 再交给 ManualInputSource。
```

**两条硬边界**

1. `输入设备 → Action` 只能发生在 `action/` 与 `hardware_interface/`；
   任何输入模块都**不允许**直接写 `World`/`WorldSnapshot`。
2. 模式 A 与模式 B 在 `Action` 之后**完全共用**同一条环境逻辑：
   两者唯一的区别是 `ActionSource` 的实现。

---

## 2. 模块划分

| 规格建议 | 本项目实际 | 状态 | 说明 |
|---|---|---|---|
| `core/` | `core/` | 已有 | state / rng / clock / numerics / actions / version / errors |
| `entities/` | `entities/` | 已有 | Player / BulletPool / Target |
| `physics/` | `physics/` | **扩展** | motion 增加角速度（精确圆弧积分） |
| `patterns/` | `generators/` | **扩展** | 语义即 patterns；新增 `single` 与运行时 `spawn_pattern` |
| `collision/` | `collision/` | **扩展** | 新增 `shapes.py`（circle/point/rect hitbox）与形状碰撞模型 |
| `scenarios/` | `scenarios/` | **扩展** | `ScenarioSpec` 增加显式 `difficulty` |
| `prediction/` | `prediction/` | 已有 | base / ballistic / rollout / danger_field |
| `action/` | **`action/`（新增）** | 新增 | `Action` 类型 + `ActionSource` + 人工/脚本/控制器适配 |
| `hardware_interface/` | **`hardware_interface/`（新增）** | 新增 | `HardwareInputAdapter` 占位 + TBD 常量 + State/Action 链路衔接 |
| `simulator/` | `simulator/` | 已有 | world / env / rewards / replay |
| `dataset/` `benchmark/` `render/` | 同 | 扩展 | benchmark 增加 prediction latency / memory |
| `interface/` | `interface/` | 保留 | **Simulator Interface**：space / protocol / controller / hardware link |
| `tests/ examples/ configs/ docs/` | 同 | 扩展 | |

`action/` 与 `interface/` 的分工：前者是**输入侧**（设备→Action），
后者是**仿真侧**（观测/动作空间、扁平协议、外部控制器、状态上行链路）。
`hardware_interface/` 是**板卡侧**（板卡→Action、Simulator→FPGA 缓冲），
与 `interface/` 单向依赖，不反向。

---

## 3. Player / Bullet / Environment 数据结构

### 3.1 Player（`float64[7]` 连续向量，FPGA 友好）

| 字段 | 类型 | 说明 |
|---|---|---|
| `x, y` | f64 | 二维位置（世界坐标系，原点左下，单位可标定为米） |
| `vx, vy` | f64 | 二维速度 |
| `radius` | f64 | 碰撞半径（圆形 hitbox 的半径） |
| `speed` | f64 | 最大速度（各方向一致，对角已归一化） |
| `alive` | f64(0/1) | 存活标志 |

hitbox 形状不写死：由 `collision/shapes.py` 的 `Hitbox`（circle / point / rect）
描述，通过 `World.player_hitbox` 配置，默认 circle(radius)。

### 3.2 Bullet / DynamicObstacle（SoA，18 字段）

一个 Bullet 既可以是"弹幕"，也可以是"动态障碍"——区别只在字段的解释，不在结构：
`shape=0` 是圆，`shape=1` 是**可旋转矩形**，因此"移动的墙"和"飞行的子弹"共用同一套
存储、碰撞与协议代码。

| 字段 | dtype | 规格要求 | 说明 |
|---|---|---|---|
| `id` | int32 | ✔ id | **单调递增、跨 compact 稳定**（碰撞报告/追踪用） |
| `x, y` | f64 | ✔ x, y | 位置 |
| `vx, vy` | f64 | ✔ vx, vy | 速度 |
| `ax, ay` | f64 | ✔ acceleration | 加速度（向量，可沿速度或固定方向） |
| `angle` | f64 | ✔ angle | 当前速度方向（rad），每步与 `(vx,vy)` 同步 |
| `angular_velocity` | f64 | ✔ angular_velocity | 速度方向旋转率（rad/s），曲线弹 |
| `radius` | f64 | ✔ radius | 碰撞半径（圆）；矩形时由 SDF 忽略 |
| `age` | f64 | — | 已存活时间 |
| `ttl` | f64 | ✔ lifetime | 生命周期 |
| `active` | bool[N] | ✔ active | 存活掩码（`alive`） |
| `type_id` | int16 | ✔ type | 弹幕/障碍类型（可视化/分组） |
| `group_id` | int16 | — | 发射组（同一 pattern 实例） |
| `shape` | int16 | **v3** | `0`=circle，`1`=rect（`SHAPE_CODES`） |
| `half_w` | f64 | **v3** | 矩形半宽；圆形时 `= radius`（统一 clearance 代码） |
| `half_h` | f64 | **v3** | 矩形半高；圆形时 `= radius` |
| `rotation` | f64 | **v3** | 矩形朝向（rad，世界系，**当前为静态**） |

`vx,vy` 与 `angle,angular_velocity` 是**冗余表示但互为权威**：
`angle` 每步由速度重算；`angular_velocity != 0` 时由它驱动速度旋转。
这样既可以直接喂 FPGA 做线性积分，也可以做曲线弹。

**Player：可见车身 ≠ 碰撞体**

| 项 | 字段 | 默认 | 说明 |
|---|---|---|---|
| 角色圆 | `player_radius` | 3.0（obstacle 场景 10.0） | 既是碰撞体，也是屏幕上画的圆 |

**没有单独的"车身"**：`player_body_radius` / `body_radius` / `body_to_hitbox_ratio` 已删除。
障碍尺寸、缺口宽度、安全余量都以 `player_diameter = 2 × player_radius` 为单位，
"两倍我自己的宽度"这种描述到真实小车上依然成立。
（底层 `collision/` 仍保留 point/rect hitbox 能力，属于引擎能力，场景层不再使用。）

### 3.3 Environment

| 字段 | 说明 |
|---|---|
| `field_w, field_h` | 环境边界（矩形工作空间） |
| `dt` | 固定时间步 |
| `step_index, sim_time` | 仿真时间（与渲染时间解耦） |
| `wrap` | 边界策略 `cull / wrap / bounce` |
| `bullet_budget` | 同时存活弹幕上限（预分配池容量） |
| `cull_margin` | 出界容差 |

---

## 4. State Interface

```
S_t = { player_state, bullet_states, target_state, environment_state, timestamp }
```

三种等价表示，同一份数据：

1. **对象视图**：`WorldSnapshot`（dataclass）— 程序直接读取
2. **结构化观测**：`obs = {player[6], target[4], env[6], bullets[n,7], bullets_mask[n]}`
   + `ObservationEncoder.flatten()` → 定长 `float32`（RL / FPGA 输入缓冲）
3. **扁平协议帧**：`encode_state(snapshot) -> bytes`

### 扁平协议 v3（**核心接口变更**，见 §10 变更说明）

```
Header (84 B, 小端)
  magic u32 'BHL1' | version u16 (=3) | flags u16 | step_index u32 | n_bullets u32
  sim_time f32 | field_w f32 | field_h f32 | dt f32
  player f32[7] (x,y,vx,vy,radius,speed,alive)
  target f32[6] (x,y,radius,shape,half_w,half_h)
Bullet stride = 17 × f32 = 68 B   (v3)
  x, y, vx, vy, ax, ay, angle, angular_velocity, radius, age, ttl,
  type_id, group_id, shape, half_w, half_h, rotation
Trailer: crc32 u32
Total := 88 + 68 × n_bullets
```

**向后兼容**：`decode_state()` 同时接受 v1（stride 10）与 v2（stride 13）帧；
缺失字段有确定性的回填规则（v2 无几何字段 → `shape=0, half_w=half_h=radius,
rotation=0`；v1 还缺 `angle/angular_velocity/age` → 由 `vx,vy` 推导或置 0）。
`describe_protocol()` 上报 `readable_versions: [1, 2, 3]`。

---

## 5. Action Interface

```python
@dataclass(frozen=True)
class Action:
    direction: tuple[float, float] = (0.0, 0.0)  # 单位向量（或零向量）
    magnitude: float = 1.0                        # [0, 1] 归一化强度
```

* `Action.zero()` 不移动；`Action.from_discrete(i)` 由 9 向表构造；
  `Action.from_vector(v)`、`Action.to_velocity(speed)`、`Action.to_array()`
* `magnitude=0` 等价于 stay；`magnitude` 允许未来接模拟摇杆/开发板 PWM/速度档位
* 规格中的 `Action = {direction, magnitude}` 是**唯一对外动作表示**；
  内部 `ActionCodec` 仍然保留（discrete / velocity / acceleration），
  因为 `velocity/acceleration` 是连续控制接口，`Action` 可无损转成它们

### ActionSource（输入层唯一契约）

```python
class ActionSource(Protocol):
    name: str
    def open(self) -> None: ...
    def close(self) -> None: ...
    def poll(self, dt: float) -> Action | None: ...   # None = 本帧无新输入
```

内置实现：

| 实现 | 模式 | 说明 |
|---|---|---|
| `ManualInputSource` | A | 键名 → Action 绑定表；`press()/release()/set_axis()`；同时多键合成 8 向 + 归一化 |
| `ScriptedSource` | A/C | 固定 Action 序列 / 回放 |
| `ControllerSource` | C | 把既有 `ActionProvider`（规则/RL/MPC/NPU）适配成 ActionSource |
| `NullInputSource` | — | 永远零动作（headless 基准） |
| `HardwareInputSource` | B | 包装 `HardwareInputAdapter`（见 §6） |

**输入层禁令**：`action/` 下任何模块禁止 `import bullet_sim.simulator`。
只有 `Env.run(input_source)` / CLI 负责把 `Action` 交给 `World.step()`。

---

## 6. Hardware Input Interface 占位设计

规格明确要求：**不得猜测**通信方式、数据格式、波特率、寄存器地址、包长、字节序、协议。
因此本层只定义**形状**，不定义**内容**：

```
Development Board
   → [HARDWARE_INPUT_INTERFACE_TBD]        ← 未确定：UART/SPI/USB/GPIO/TCP/共享内存/MMIO 之一
   → HardwareInputAdapter                  ← 抽象基类，只声明 open/close/poll
   → Action
   → Simulator
```

```python
class HardwareInputAdapter(ABC):
    transport: str = HARDWARE_INPUT_INTERFACE_TBD   # 占位标记

    @abstractmethod
    def open(self) -> None: ...
    @abstractmethod
    def close(self) -> None: ...
    @abstractmethod
    def poll(self, dt: float) -> Action | None: ...

    def describe(self) -> dict: ...   # 返回 planned_transport 等全部 TBD 字段
```

所有未知量集中在 `hardware_interface/tbd.py`，**默认值全部是 `None` / `"TBD"`**：

| 未知项 | 当前值 |
|---|---|
| 传输方式 `PlannedTransport.kind` | `None` |
| 波特率 / 时钟 | `None` |
| 寄存器地址 / MMIO 基址 | `None` |
| 数据包长度 / 帧格式 | `None` |
| 字节序 | `None` |
| 数据格式 / 编码 | `None` |
| 握手 / 超时 / 重传 | `None` |

未配置时 `PlaceholderHardwareInputAdapter.poll()` 抛
`HardwareInterfaceNotConfigured`，**绝不返回猜测的动作**。

`LoopbackHardwareInputAdapter` 是**测试夹具**（程序内注入 Action），
用于在真实板卡到来之前验证
`Adapter → Action → Simulator` 这一整条链路是通的 —— 它不是协议猜测。

同时保留 `interface/hardware.py` 的 `HardwareLink`（状态上行/预测下行），
两者共同表达：

```
Simulator → FPGA input → FPGA result → CPU → Action → Simulator
```

---

## 7. Scenario 配置结构

```python
@dataclass
class ScenarioSpec:
    # 复现性
    name: str; seed: int; duration: float; dt: float
    # 环境边界
    field_w: float; field_h: float; wrap: str; cull_margin: float
    # 玩家初始状态
    player_x, player_y, player_radius, player_speed
    # 目标
    target_x, target_y, target_radius, target_shape, target_half_w, target_half_h
    # 弹幕生成配置
    patterns: list[PatternSpec]         # 每个 PatternSpec 参数化：起点/角度/数量/
                                        # 初速/加速度/角速度/发射间隔/半径/生命周期/随机扰动
    bullet_capacity: int | None
    # 难度（显式字段 + meta['complexity']；不再是主分类）
    difficulty: str | None
    # 玩家是一个圆（v3）：半径 = 碰撞 = 画面
    player_radius: float
    # 安全校验（v3）：有挑战 ≠ 必死
    safety_margin: float; safety_horizon: float; require_valid_scenario: bool
    collision: str = "auto"             # auto 时由场景自动选碰撞后端
    # 其它
    action_space: Any; meta: dict
```

不变式：**同一 `ScenarioSpec` + 同一 `seed` ⇒ 同一 `SpawnTimeline` ⇒ 同一轨迹**。
`scenario.to_dict()` 随数据集一起持久化，用于离线复现。

---

## 7.5 动态障碍类型与组合（v3 新增）

平台的核心抽象从"弹幕"换成"现实动态障碍"。`ObstacleType` 是一份**不可漂移的文档**：
它同时描述现实语义、运动方式、玩家要解决的问题、该用哪种安全校验，
并直接驱动生成器（`resolve_spec` 是"类型 → 生成调用"的唯一入口）。

```
ObstacleType(key, label, label_zh, simulates, motion, player_problem,
             layout, safety_strategy, shape, defaults)
ObstacleSpawn(type_key, count, size, speed, interval, start_time, ttl,
              region, gap_width, gap_motion, closing_rate,
              corridor_width, spawn_distance, entry_angle, extra)
ObstacleScenario(name, seed, duration, dt, field_w, field_h,
                 player_hitbox_radius, player_speed,
                 obstacles[], safety_margin, safety_horizon, require_valid)
   .to_patterns() / .to_spec() / .build() / .describe() / .to_dict() / .save()/.load()
```

**5 种类型**：`moving_block`、`wall_with_gap`、`small_obstacles`（原 sparse + dense 合并）、
`corridor`、`cross_traffic`。`moving_wall` / `closing_gap` / `mixed_obstacles` 已删除：
"无缺口墙"就是缺口为 0 的墙，"缺口收窄"只是参数，"混合"直接用多个 `--obstacle-type`。

**5 种 layout**：

* `block` - 从边界一段区间**随机位置**进入，速度强制指向场内（给向外的角度也会被纠正）；
* `wall_gap` - 缺口 ≥ 玩家圆直径，缺口位置随机抖动 / 可扫动；
* `small` - 边界进入的圆形小障碍；
* `cross` - 多方向同时进入；
* `corridor` - 角色生成在两堵墙**正中间**，两墙以角色为中心对称、**墙心不动**，
  只通过 `open`/`close`（对称平移改变宽度）、`rotate_same`（同向自转 → 通道整体倾斜，
  宽度恒定）、`rotate_opposite`（反向自转 → 剪刀口，角度按几何上限裁剪）模拟道路变化；
  `walls=1` 只放一堵墙。变化量按整段回放分摊成恒定速度/角速度，最小间距
  ≥ 玩家圆直径 ×1.25。

**尺寸用比例**：`size` = 玩家车身直径的倍数（`SIZE_PRESETS`），
`resolve_spec` 只在**唯一一处**把它换算成世界单位；圆形障碍保持
`half_w == half_h == radius`。

**生成区域与入口方向**：`SPAWN_REGIONS`（`left/right/top/bottom/center_edge/
farthest_side/nearest_side`）给出边界锚点与指向场内的入口角；
`sparse` / `cross` 保留锚点的**边坐标**、沿该边散布——障碍永远从边界进入，
不会在场地中线凭空出现；`min_player_distance` 再兜底丢弃"生成在玩家身上"的障碍。

**相对运动**：`v_rel = v_obs − v_player`（`physics/relative.py`），
提供速度、接近率 `closing_speed`、撞击时间 `time_to_impact`、玩家系快照
`to_player_frame`。

---

## 7.6 难度：参数空间 + 实测分数（v3 新增）

难度不是用户选的标签，而是**参数空间的函数**，并且可以从一次真实校验里**测出来**：

```
ComplexityParams(obstacle_count, obstacle_size, obstacle_speed, gap_width,
                 gap_motion, corridor_width, spawn_distance, relative_speed,
                 reaction_time, prediction_horizon, player_speed,
                 player_radius, player_hitbox_radius)
complexity_from_measurements(obstacle_count, free_fraction, reachable_fraction,
                             min_clearance, player_diameter, relative_speed,
                             player_speed, prediction_horizon, reaction_time)
   -> ComplexityScore(complexity, label, terms, advice)
```

分项与权重全部显式：`crowding` 0.22 / `reach` 0.20 / `tightness` 0.18 /
`speed` 0.12 / `anticipation` 0.16 / `geometry` 0.07 / `path_scarcity` 0.05。

`validate_scenario()` **内置**这一步：它把实测到的自由空间、可达集、路径最小余量、
峰值相对速度喂给 `complexity_from_measurements`，结果挂在
`SafetyReport.complexity / complexity_label / complexity_terms / complexity_advice` 上，
并出现在 `report.format()`（CLI 的 `--validate` 输出）与 `report.to_dict()` 里。
`complexity_label` 只用于报告分箱，**不参与任何仿真语义**。

---

## 8. Simulator API

```python
# ---- World：确定性内核 ----
World(scenario, *, seed, codec, collision, reward, player_hitbox, ...)
  .reset(seed=None)            -> WorldSnapshot
  .step(action)                -> StepResult(state, reward, terminated, truncated, info)
  .get_state() / .set_state()  -> WorldSnapshot
  .clone() / .clone_state() / .restore_state()
  .state_hash()                -> str          # 复现性断言
  .spawn_bullets(...)          # 手动注入（调试/传感器）
  .spawn_pattern(cfg)          # ★ 规格要求的 spawn_pattern(pattern_config)
  .simulate_future(actions, horizon) / .rollout_final(...)
  .describe() / .policy_describe()

# ---- Env：Gymnasium 风格，两种模式共用 ----
env = BulletHellEnv(scenario, seed=..., collision=..., player_hitbox=...)
  .reset(seed=...)
  .step(action)
  .get_state() / .clone_state() / .restore_state()
  .observe(flat=False) / .simulate_future(...) / .evaluate_actions(...)
  .run(input_source=None, steps=None)   # ★ 人工 / 脚本 / 开发板 都走这里
  .render(mode="human"|"rgb_array"|"ascii"|"none")
  .close()
```

CLI：

```bash
python -m bullet_sim play --input keyboard            # 模式 A：人工键盘
python -m bullet_sim play --input scripted            # 回放
python -m bullet_sim play --input board               # 模式 B：开发板（当前为 TBD 占位）
python -m bullet_sim run  --input random --steps 1200 # headless
```

---

## 9. 第一阶段开发计划（Phase 1–12）与本次落地

| Phase | 内容 | 状态 |
|---|---|---|
| 1 | 最小环境 + Player + Bullet + fixed timestep | 已完成（上一轮） |
| 2 | **人工控制** | ✅ 已落地：`action/` + `ManualInputSource` + `play --input keyboard` |
| 3 | Radial / Spiral / Aimed 等 Pattern | ✅ 已完成；已补 `single` + 运行时 `spawn_pattern()` |
| 4 | Collision | ✅ 已完成；已补 circle/point/rect hitbox + id/frame/距离/风险输出 |
| 5 | Scenario + seed + replay | ✅ 已完成；已补显式 `difficulty` / `complexity` |
| 6 | Headless simulation | 已完成 |
| 7 | Dataset generation | 已完成 |
| 8 | **统一 Action Interface** | ✅ 已落地：`Action{direction, magnitude}` + `ActionSource` |
| 9 | **Hardware Input Interface 占位** | ✅ 已落地：`hardware_interface/`，全量 TBD + 故障即报错 |
| 10 | Future Simulation | 已完成（prediction 调用 Simulator，不复制物理） |
| 11 | AI Controller Interface | ✅ 已完成；已补 `ControllerSource` 适配 |
| 12 | Benchmark | ✅ 已完成：100/500/1000/2000/5000/10000 + prediction latency + memory |
| 13 | **动态障碍类型体系 + `ObstacleScenario`** | ✅ 已落地：`obstacles/`（5 种类型、现实语义、边界生成） |
| 14 | **安全校验闭环（有挑战 ≠ 必死）** | ✅ 已落地：`safety/`（配置空间膨胀 + 时窗波前 + 自动重采样） |
| 15 | **难度 = 参数空间 + 实测 complexity** | ✅ 已落地：`scenarios/params.py`，并内置于 `validate_scenario` 报告 |
| 16 | **Viewport 窗口缩放（观察尺度与物理世界解耦）** | ✅ 已落地：`render/viewport.py` |

---

## 9.5 安全校验闭环（v3 新增）

```
Scenario Generator → Obstacle Generation → Free Space Analysis
                   → Path Feasibility Check → Valid Scenario
```

| 公开 API | 语义 |
|---|---|
| `is_scenario_valid(world/spec/built, *, config)` | 是否存在可行安全路径 |
| `find_safe_path(...)` | 返回一条具体可行的无碰撞轨迹 `SafePath`，否则 `None` |
| `compute_safe_region(...)` | 当前 / 时窗自由空间 + 可达区域 `SafeRegion` |
| `predict_collision(world, action, horizon)` | 保持动作时的首次碰撞时间（真实 rollout + 解析估计） |
| `validate_scenario(...)` | 完整 `SafetyReport`（原因、数值、实测难度、建议） |
| `generate_valid_scenario(...)` | 失败即**定向调参重采样**，最多 `max_attempts` 次 |

**判据**（几何 + 动力学 + 时间，而不是当前帧重叠）：

1. 每个采样时刻都在**膨胀后**的障碍之外，膨胀量 = 玩家真实碰撞体外接圆 +
   `safety_margin`（configuration space）；
2. 相邻位移在 `player_speed`（可选 `max_accel`）之内；
3. 从玩家真实出生点出发，覆盖整个验证时窗（`min(回合时长, max_window)`）；
4. **到达时间波前**（arrival-time wavefront）保证小于一格的推进不丢失，
   且**当前被占据的格不再作为传播源**——波前不会穿过移动的墙；
5. 矩形 hitbox 用外接圆，格点判 blocked 的条件是 `sdf < inflate`（刻意保守）。

**配置空间**：`SafetyConfig(margin, horizon, resolution, sample_dt, max_speed,
max_accel, min_free_fraction, min_reachable_cells, validation_window, max_window)`。

---

## 10. 核心接口变更记录（规格 §17 要求）

| 变更 | 原因 | 影响范围 | 兼容性 |
|---|---|---|---|
| `BulletPool` 新增 `id/angle/angular_velocity` | 规格 §5 明确要求这三个字段；`id` 需跨 `compact()` 稳定以支持碰撞报告与追踪 | `entities/bullet.py`、`physics/motion.py`、`collision/*`、`dataset/schema.py` | 旧数据集仍可读（按列名）；协议 v1 帧仍可解码 |
| 协议 `STATE_PROTOCOL_VERSION 1 → 2`，stride `10 → 13` floats | 上述字段需要上线 | `interface/protocol.py`、`docs/PROTOCOL.md` | `decode_state` 同时接受 v1/v2；`encode_state` 只产出 v2 |
| **协议 `2 → 3`，stride `13 → 17` floats**（`shape/half_w/half_h/rotation`） | 动态障碍需要矩形与朝向才能表达"移动的墙""缺口"等现实形态 | `entities/bullet.py`、`interface/protocol.py`、`generators/*`、`collision/obstacles.py`、`docs/PROTOCOL.md` | `decode_state` 同时接受 v1/v2/v3；v<3 帧按确定性规则回填几何默认值 |
| **`Player` 只保留 `player_radius`（圆形角色，可视 = 碰撞）** | 需求要求角色就是圆、改 hitbox 就是改画面大小，不能再有"看起来大、判定小"的两套尺寸 | `scenarios/spec.py`、`render/*`、`safety/validate.py`、`scenarios/params.py` | 删除 `player_body_radius` / `body_radius` / `body_to_hitbox_ratio`；旧 JSON 里这些键被忽略 |
| **新增 `obstacles/`（ObstacleType / ObstacleScenario）** | 场景的主分类从"难度标签"改为"现实障碍类型"，且必须可组合、可单独运行 | 新增包；`generators/spec.py`/`patterns.py` 增加 `obstacle` kind 与 5 种 layout | 见下一条 |
| **删除装饰性弹幕模式**（`single`/`radial`/`spiral`/`aimed`/`burst`/`line`/`wall`/`random`） | 需求：只保留模拟现实的障碍；`--level` 场景不再出现线状/中心发射的物体 | `generators/patterns.py`、`generators/spec.py`（`PATTERN_KINDS`）、`scenarios/complexity.py`（`obstacle_kinds`）、`scenarios/builder.py` | **不兼容**：这些 kind 不再存在，`PatternSpec(kind="radial")` 会报错；level 预设改为障碍场景 |
| **矩形 `angular_velocity` 改为"身体自转"** | 走廊需要"墙心不动、只转向"的道路变化，而旧的 `angular_velocity` 只旋转速度方向 | `physics/motion.py`、`generators/patterns.py`（layout 返回每障碍 spin） | 圆的行为不变（仍是曲线弹）；矩形位置不变、朝向按 ω 前进 |
| **level 目标数量改为"安全上限 + 实测减薄"** | 需求：extreme 必须留有安全区 | `scenarios/complexity.py`（`LEVEL_OBJECT_CAP` / `level_object_count`）、`scenarios/builder.py`（`_thin_until_safe`，`ensure_valid`） | `stress(N)` 不受限；level 的请求值在 `meta['target_bullet_count']`，实际值在 `meta['achieved_bullet_target']` |
| **新增 `safety/`（自由空间 / 路径搜索 / 校验 / 重采样）** | 规格要求"自动生成的环境必须至少存在一条可行路径" | 新增包；`ObstacleScenario.build()` 默认校验 | 既有仿真 API 不变；`World` 多了一种碰撞后端 `obstacle` |
| **`World` 的碰撞后端缺省值改为跟随 `ScenarioSpec.collision`** | 之前显式指定的 `shaped`/`obstacle` 会被静默忽略，矩形障碍退化成圆判定 | `simulator/world.py` | `collision=None` 时行为与 `auto` 一致；显式传参仍是最高优先级 |
| **难度改为实测**：`SafetyReport.complexity` 由自由空间/可达集/余量/相对速度算出 | 规格 §9/§20 要求难度是参数空间与实测量的函数，而不是标签 | `scenarios/params.py`、`safety/validate.py` | `difficulty`/`complexity` 字段保留，旧预设仍可用 |
| `ScenarioSpec` 新增 `difficulty` | 规格 §9 要求 Scenario 显式包含 difficulty | `scenarios/spec.py` | 默认 `None`，旧 JSON 不受影响 |
| `CollisionResult` 新增 `bullet_id/frame/min_distance/risk` | 规格 §7 要求输出碰撞对象 ID、碰撞时间/frame、最小距离、风险 | `collision/*`、`simulator/world.py`（info） | 新增字段，旧字段保留 |
| 新增 `Action` 与 `ActionSource` | 规格 §3/§4 要求人工与开发板共用统一 Action | 新增 `action/`；`simulator/env.py` 增加 `run(input_source=...)` | 旧 `ActionCodec` / `ActionProvider` 全部保留，`ControllerSource` 适配 |
| `ScenarioSpec` 新增 `difficulty` | 规格 §9 要求 Scenario 显式包含 difficulty | `scenarios/spec.py` | 默认 `None`，旧 JSON 不受影响 |

**未变更**（保持稳定）：`World.reset/step/get_state/clone/simulate_future`、
`WorldSnapshot` 语义、pattern 参数结构、数据集数组布局（仅 `bullets` 列数变化）。

---

## 11. 开放项（不得猜测，保持 TBD）

| 项 | 标记 | 说明 |
|---|---|---|
| 开发板通信方式/协议/帧格式/寄存器/波特率/字节序 | `[HARDWARE_INPUT_INTERFACE_TBD]` | 全部集中在 `hardware_interface/tbd.py` |
| CPU↔FPGA 具体总线（DMA/MMIO/中断/polling） | `[HARDWARE_INPUT_INTERFACE_TBD]` | `interface/hardware.py` 只给抽象 `HardwareLink` |
| Keiki 代码直接复用 | `[LICENSE_REVIEW_REQUIRED]` | Keiki 仓库未见 LICENSE 文件，仅 README 要求引用；本项目**只借鉴设计思路，不复制代码** |
| TostEngine 代码直接复用 | 可复用（MIT） | 仍需在文档中标注来源；本项目为独立 Python 设计，未移植其 C++ 代码 |
