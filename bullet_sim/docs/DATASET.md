# 训练数据集 Schema (v1)

导出物是**面向机器学习的数据集**，不是游戏存档。每个 transition 都带有
`state_t, action_t, next_state, collision, reward, done` 以及可复现所需的
`scenario_id / seed / timestamp`。

## 1. 目录结构

```
data/
├── ep_00000.npz          # 轨迹数据（格式由 --format 决定）
├── ep_00000.csv
├── scenario_00000.json   # 该 episode 的完整场景配置（可复现）
├── ...
└── manifest.json         # 全部 episode 的汇总（steps / max_bullets / reward ...）
```

`npz`/`bin`/`json` 三种格式都会把 `scenario` 配置与**产生它的环境策略**
写进 `meta`，因此**单个文件即可自复现**：

| meta 键 | 内容 |
|---|---|
| `scenario` / `env.scenario` | 完整 `ScenarioSpec`（含 seed、dt、patterns、bullet_capacity） |
| `env_policy` | `collision_terminates_episode` / `terminate_on_collision`、`collision_penalty`、`collision_count_mode`、`clamp_player`、`collision`、`reward`、`action_space` |
| `seed` / `schema_version` / `dt` / `fps` | 基础元信息 |

> **为什么必须记录 `env_policy`**：重放时若使用不同的 `collision_terminates_episode`，
> 场景会在第一次碰撞处提前结束（模式 B），逐位比对就会失败。`verify_episode()` 默认
> 使用数据集内记录的策略，因此 `record → replay` 天然一致。
>
> 另外注意区分两个信号：`collisions` 是**帧级**重叠标志（压着弹幕 40 帧就是 40 个 True），
> `collision_events` 才是**事件级**计数（同一颗弹幕只算 1 次）。
> reward 是按 `collision_events` 扣的。详见 `README.md` 第 9 节。

## 2. Episode 数组布局

`T` = transition 数量；存储 `T+1` 个状态（`S_0 .. S_T`）。

| key | shape | dtype | 含义 |
|---|---|---|---|
| `player` | (T+1, 7) | float32 | `x, y, vx, vy, radius, speed, alive` |
| `target` | (T+1, 6) | float32 | `x, y, radius, shape, half_w, half_h` |
| `env` | (T+1, 7) | float32 | `field_w, field_h, dt, step_index, sim_time, wrap, bullet_budget` |
| `bullets` | (Σn_t, 10) | float32 | 每步存活弹幕（紧凑前缀），列见下 |
| `bullet_counts` | (T+1,) | int32 | 每个状态的真实存活弹幕数 |
| `bullets_offsets` | (T+1,) | int32 | 该状态在 `bullets` 中的起始行 |
| `bullet_sampled` | (T+1,) | bool | 该状态是否记录了弹幕块（`--bullet-stride>1` 时为 False） |
| `actions` | (T,) | int32 或 float32 | 原始动作（离散索引或向量） |
| `action_vectors` | (T, 2) | float32 | 编码后的世界系期望速度 |
| `collisions` | (T,) | bool | 本步是否**处于重叠**（帧级信号） |
| `collision_events` | (T,) | int32 | 本步**新增碰撞事件数**（惩罚计费依据） |
| `collision_counts` | (T,) | int32 | 同时重叠的弹幕数 |
| `min_dist2` | (T,) | float32 | 最近弹幕的平方中心距 |
| `rewards` | (T,) | float32 | 奖励 |
| `terminated` | (T,) | bool | 碰撞终止 |
| `truncated` | (T,) | bool | 到达场景时长上限 |
| `done` | (T,) | bool | `terminated or truncated` |
| `timestamps` | (T,) | float64 | 该步之后的仿真时间 |
| `step_indices` | (T,) | int32 | 该步之后的步序号 |
| `state_hashes` | (T+1,) | U32 | 每个状态的规范化摘要（`--hashes`） |

弹幕行 `bullets[:, :]` 的 10 列顺序与硬件协议一致：

```
x, y, vx, vy, ax, ay, radius, ttl, type_id, group_id
```

## 3. 读取

```python
from bullet_sim.dataset.readers import load_episode

ep = load_episode("data/ep_00000.npz")
print(ep.summary())
print(ep.transition(0))       # 单条 (S_t, A_t, S_{t+1}, r, done)
print(ep.state(0))            # 单个状态（含弹幕列表）
X = ep.arrays["player"]       # 直接拿 ndarray 喂模型
```

`load_episode` 按后缀分派到 JSON / NPZ / CSV / 二进制读取器，
返回同一个 `Episode` 容器，下游代码无需区分格式。

## 4. 写入

```python
from bullet_sim.dataset.writers import save_episode
save_episode(ep, "out/ep.npz")     # 自动按后缀选择格式
```

| 格式 | 用途 | 特点 |
|---|---|---|
| `json` | 调试 / 人工检查 | 完整无损，体积最大 |
| `npz` | **训练** | 无损、压缩、`np.load` 直接可用 |
| `csv` | 简单实验 / 表格 | 仅标量字段，**有损**（无弹幕块、无 S_0） |
| `bin` | 高性能下游接口 | 自描述二进制容器（magic/dtype/shape/raw），可 mmap |

## 5. 一致性保证

`verify_roundtrip(episode, path)` 会重新读取文件并逐数组比对；
测试套件对 JSON / NPZ / BIN 断言**逐位一致**，对 CSV 断言标量字段一致。

复现性验证：

```python
from bullet_sim.simulator.replay import verify_episode

cmp = verify_episode(ep)          # 使用数据集内记录的 scenario + env_policy
print(cmp.ok, cmp.reason, cmp.first_mismatch, cmp.policy)

# 更低层、需要显式给出 scene/策略时：
from bullet_sim.simulator.replay import verify_replay
actions = [int(a) for a in ep.arrays["actions"]]
hashes  = [str(h) for h in ep.arrays["state_hashes"]][1:]
ok, first_bad = verify_replay(spec, actions, hashes, seed=ep.seed,
                              terminate_on_collision=False)
```

仓库中的 `examples/03_headless_dataset.py` 会真正跑一遍
"记录 → 导出 → 读回 → 重放比对"的完整闭环。

## 6. 生成数据集

```bash
# 32 个 episode，medium 难度，NPZ + 每步状态摘要
python -m bullet_sim record --level medium --count 32 --out data/ \
       --format npz --hashes --controller random --steps 1200

# 高密度压力场景数据集（弹幕块每 10 步记录一次以控制体积）
python -m bullet_sim record --bullets 2000 --count 8 --out data/stress/ \
       --format npz --bullet-stride 10
```

种子按 `base_seed + i * 1000003` 派生，因此只要记录
`(base_seed, count, 难度)` 就能完整重建整个数据集。
