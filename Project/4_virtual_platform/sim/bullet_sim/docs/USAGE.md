# 使用指南 (API Quickstart)

## 1. 安装

```bash
pip install -e .            # 只需 NumPy
pip install -e ".[dev]"     # + pygame (可视化) + pytest + matplotlib
```

## 2. 三种典型用法

### A. 跑一个场景（最快路径）

```python
from bullet_sim import BulletHellEnv, stress

env = BulletHellEnv(stress(1000, seed=7))     # ~1000 存活弹幕
obs, info = env.reset(seed=7)
for _ in range(600):
    obs, reward, terminated, truncated, info = env.step(1)
env.close()
```

### B. 用固定 seed 复现一条轨迹

```python
from bullet_sim.simulator.replay import replay, verify_replay

actions = [1, 2, 3, 0] * 100
a = replay(spec, actions, seed=1234)
b = replay(spec, actions, seed=1234)
assert a.state_hashes == b.state_hashes
```

`S_0 + A_0..A_T + seed` 完整决定 `S_1..S_T`；
`WorldSnapshot.state_hash()` 是判定依据。

### C. headless 批量生成训练数据

```bash
python -m bullet_sim record --level hard --count 16 --out data/ --format npz --hashes
```

## 3. 核心 API

### `World`（确定性内核，无渲染无 AI）

| 方法 | 说明 |
|---|---|
| `World(scenario, seed=..., codec=..., collision=..., reward=...)` | 构建 |
| `reset(seed=None)` | 回到 `S_0`；给定 seed 会重建 spawn 时间线 |
| `step(action)` | 推进一个固定 `dt`，返回 `StepResult` |
| `get_state()` / `set_state(snap)` | 完整状态快照（可序列化） |
| `clone()` | 深拷贝（含 RNG 与碰撞模型状态） |
| `clone_state()` / `restore_state(snap)` | 快照式分叉 |
| `simulate_future(actions, horizon, stride)` | 在克隆上推进未来，不影响本体 |
| `rollout_final(actions, horizon)` | 只要最终状态 + 逐步 reward |
| `state_hash()` | 规范化摘要（复现性断言） |
| `spawn_bullets(...)` / `spawn(event)` | 手动注入弹幕（调试/外部传感器） |
| `add_listener(fn)` | 步进事件钩子（数据集记录器用） |
| `describe()` | 平台契约信息 |

### `BulletHellEnv`（Gymnasium 风格）

| 成员 | 说明 |
|---|---|
| `reset(seed=None, options=None) -> (obs, info)` | |
| `step(action) -> (obs, reward, terminated, truncated, info)` | |
| `get_state()` / `clone_state()` / `restore_state()` | |
| `simulate_future(...)` / `evaluate_actions(candidates, horizon)` | 未来推演与候选动作评估 |
| `run(controller, steps, seed)` | headless 回合循环，返回回报/碰撞数/摘要 |
| `render(mode="human"\|"rgb_array"\|"ascii"\|"none")` | 只读渲染 |
| `action_space` / `observation_space` / `describe()` | 空间与元信息 |
| `close()` | 释放渲染资源 |

`obs` 是结构化 dict（`player[6]`, `target[4]`, `env[6]`, `bullets[n,7]`,
`bullets_mask[n]`），并可通过 `env.encoder.flatten(obs)` 得到**定长**
`float32` 向量（`max_bullets` 截断+补零），用于 RL 或直接送 FPGA 输入缓冲。

### 统一 Action 与输入源

```python
from bullet_sim.action import Action, ManualInputSource, ScriptedSource, source_from
from bullet_sim.hardware_interface import HardwareInputSource, LoopbackHardwareInputAdapter

a = Action.from_discrete("up_right")      # 9 向
a = Action.from_angle(210.0, 0.5)         # 角度 + 强度
a = Action.from_vector([3.0, 4.0])        # 任意 2D 向量

env.run(input_source=ManualInputSource(), steps=1000)          # 模式 A 人工
env.run(input_source=HardwareInputSource(my_board_adapter))    # 模式 B 开发板
env.run(input_source=my_policy)                                # 模式 C 外部程序
```

详见 `docs/INPUT_MODES.md`。

### 动作空间

* 统一 `Action{direction, magnitude}`（推荐；`codec="action"` 时可直接 step）
* 离散（默认 9 个）：`stay/left/right/up/down/up_left/up_right/down_left/down_right`
* `codec="velocity"`：`(vx, vy)`
* `codec="acceleration"`：`(ax, ay)`，内部按 `dt` 积分并限速

统一接口是 `ActionCodec.encode(action, ctx) -> 期望速度向量`，
因此任何策略（规则/模仿/RL/MPC/CPU-FPGA 决策模块）都能无侵入接入。

### 外部 Controller

```python
from bullet_sim.interface.controller import ActionProvider

class MyController(ActionProvider):
    def reset(self, observation, info): ...
    def act(self, observation, info) -> int: ...

env.run(MyController())           # 也接受普通 callable / "random" / {"kind": "scripted", ...}
```

## 3.5 碰撞与回合结束

**碰撞是惩罚事件，不是结束事件**：`collision_count += 1`、`reward -= collision_penalty`，
然后继续 `step()`。默认 `collision_terminates_episode = False`。

```python
env = BulletHellEnv(spec, seed=7)                 # 默认：碰撞不终止
for _ in range(1200):
    obs, reward, terminated, truncated, info = env.step(action)
    if info["collision_event"]:
        print("hit", info["collision_count"], reward)   # 继续跑
print(env.episode_metrics())

env = BulletHellEnv(spec, seed=7, terminate_on_collision=True)   # 实验模式 B
```

配置项（`ScenarioSpec` / JSON）：`collision_penalty`、`collision_count_mode`
（`per_contact` / `per_step` / `per_frame`）、`collision_terminates_episode`、
`success_terminates_episode`。详细语义见 `README.md` 第 9 节。

## 4. 未来推演与风险

```python
from bullet_sim.prediction.ballistic import BallisticPredictor
from bullet_sim.prediction.rollout import FutureSimulator, evaluate_candidates, best_candidate, clearance
from bullet_sim.prediction.danger_field import compute_danger_field, danger_from_forecast

snap = env.get_state()

# 零成本解析外推（不步进世界）
forecast = BallisticPredictor().predict(snap, horizon=90, dt=snap.env.dt)

# 完整 rollout（含生成/剔除/碰撞）
sim = FutureSimulator(env.world)
states = sim.simulate_future(90, action=0)

# 候选动作评估（MPC / 风险决策原语）
results = evaluate_candidates(sim, list(range(9)), horizon=60, objective="survival")
best = best_candidate(results)

# 未来危险场（H × rows × cols 栅格）
danger = compute_danger_field(states, resolution=16.0, safety_radius=24.0)
print(danger.risk_at(px, py, h=0))
```

## 5. 场景与难度

```python
from bullet_sim.scenarios.presets import scenario, stress, all_levels
from bullet_sim.scenarios.complexity import profile_for

scenario("hard", seed=7)         # easy/medium/hard/extreme
scenario(0.62, seed=7)           # 连续 complexity ∈ [0,1]
scenario(2000, seed=7)           # 直接指定存活弹幕数（自动标定）
all_levels(seed=7)               # 四个难度一起拿
profile_for(0.8).to_dict()       # 查看难度解析结果
```

`bullet_count` 通过**实际测量**（真实生成 + 运动 + 剔除，无玩家无碰撞）
迭代标定，因此 `stress(2000)` 的场景峰值确实接近 2000。

## 5.5 动态障碍场景与安全校验

```python
from bullet_sim.obstacles.scenario import scenario_from_types
from bullet_sim.safety import (
    SafetyConfig, compute_safe_region, find_safe_path,
    generate_valid_scenario, is_scenario_valid, validate_scenario,
)

# 5 种现实障碍类型，可单独跑、可用多个 --obstacle-type 组合
sc = scenario_from_types(
    [
        {"type": "moving_block", "size": 2.0, "speed": 90.0},
        {"type": "wall_with_gap", "gap_width": 180.0},
        {"type": "small_obstacles", "count": 10},
        {"type": "corridor", "motion": "rotate_same", "change": 18.0},
    ],
    seed=7, duration=30.0,
)
spec = sc.to_spec()          # 之后走既有管线：复现 / 数据集 / 基准
sc.describe()                # 现实语义 + 参数 + 安全配置

# 安全校验：有挑战 ≠ 必死
config = SafetyConfig(margin=6.0, horizon=2.0)
report = validate_scenario(spec, config=config)
print(report.feasible, report.reason, report.suggestions)
is_scenario_valid(spec, config=config)
path = find_safe_path(spec, config)          # SafePath | None
region = compute_safe_region(spec, config)   # 自由空间 / 可达区域 / 安全点

# 自动调参重采样，而不是"生成一次就失败"
fixed, report, attempts = generate_valid_scenario(sc, max_attempts=8, strict=False)
```

CLI：

```bash
python -m bullet_sim obstacles                       # 5 种障碍类型 + 现实语义
python -m bullet_sim run --obstacle-type moving_block --obstacle-type wall_with_gap \
    --gap-width 180 --validate --safe-path --steps 900
python -m bullet_sim play --obstacle-type cross_traffic --seed 1
```

## 6. CLI 速查

```bash
python -m bullet_sim scenarios                       # 列出 pattern/难度/协议
python -m bullet_sim obstacles                       # 列出 5 种动态障碍类型
python -m bullet_sim scenario --level hard --seed 7 -o configs/hard.json
python -m bullet_sim run --level medium --controller random --steps 1200 --json
python -m bullet_sim record --bullets 2000 --count 8 --out data/ --format npz
python -m bullet_sim replay --dataset data/ep_00000.npz
python -m bullet_sim benchmark --ladder 50 200 1000 5000 --out bench.json
python -m bullet_sim play --level hard --interactive --prediction
python -m bullet_sim info
```

## 7. 性能基线（本机参考值）

`bullet_ladder` 套件在 120 Hz、`circle` 碰撞模型下的典型结果：

| 目标弹幕 | 场景峰值 | 平均弹幕 | 单步均值 | 仿真 FPS | 实时倍率 |
|---|---|---|---|---|---|
| 50 | 48 | 39 | ~0.06 ms | ~16 300 | ~136x |
| 200 | 208 | 184 | ~0.12 ms | ~8 050 | ~67x |
| 1000 | 953 | 683 | ~0.14 ms | ~7 220 | ~60x |

实际数值随机器变化；用 `python -m bullet_sim benchmark` 生成自己的报告。
关键指标是 **us / bullet / step**，它直接回答"多少弹幕必须卸载到 FPGA"。
