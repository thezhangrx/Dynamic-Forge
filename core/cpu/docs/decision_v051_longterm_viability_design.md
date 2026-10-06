# Dynamic-Forge v0.5.1-LongTermViability 设计稿（第 2 版 · 审查修正）

> **本轮性质：设计审查 + 设计稿修正。未实现任何算法代码。**
>
> 状态基线（不变）
> - 正式预测式版本：**v0.4.1-SideHit-ASE**
>   1000 局：survival 958.2 ± 252.5，collision 61.7%，decision ≈ 1.54 ms，action change 146.8 ± 69.4
>   failure：front_hit 175 / side_hit 398 / corner_trap 44 / other 0
> - `v0.5.0-ReactiveGap`：失败实验，归档保留，不参与 benchmark
> - benchmark 反应式基线：`RepulsionController`（保持原始）
>
> 本文是第 2 版，落实第二轮审查的 10 项修正与 16 项更新要求。

---

## 0. 设计审查结论速览（保留 / 修改）

| 项 | 第 1 版 | 第 2 版 |
|---|---|---|
| 总体两阶段框架 | 保留 | 保留（17 → FAR/ASE → shortlist → LTV → FES → behaviour → task） |
| Top-K | 固定 Top-3 | **改为 K=4 = 3 short-term leaders + 1 diversity**，附 K=3+替换 备选 |
| 未来位置 | 未强制 | **硬性：agent 与 obstacle 都用未来位置** |
| 层级 | `critical > viability > future-risk` | **取消固定 winner**，改 bucketed/分档仲裁 |
| anchor 聚合 | `min` 与 `0.4/0.4/0.2` 并存 | **只保留 `min`**；删除 anchor 权重 |
| SceneState | 计划新增 `long_horizon` | **不新增**；尺度由 evaluator 配置/适配器注入 |
| anchor | 按尺度归一 | 保留归一 + **clamp** + `max_long_horizon` |
| FES 与 LTV | 职责未分离 | 明确时间尺度分工 |
| Escape Corridor | 借用 TEB 措辞偏强 | 明确是**离散局部逃逸通道类**，非 homotopy |
| corridor 动态障碍 | 未强调 | 明确用未来障碍位置 |
| corridor hysteresis | 无 | 新增（Safety > Hysteresis） |
| mobility 定义 | `f(...)` | 明确归一化公式，`[0,1]` |
| 测试 | 计划 A–L | 增加 **A/B toy scenario** 与 **Top-K 截断测试** |

### 0.1 第 3 轮审查确认（Q1–Q4，已锁定）

| 项 | 决定 |
|---|---|
| Q1 | `risk_tolerance` **重命名为 `risk_band_width`**，固定 `0.05`。它是**风险分桶宽度**，**不表示"允许 5% 碰撞风险"**。 |
| Q2 | **K = 4 = 3 short-term leaders + 1 diversity**。diversity 必须在 **LTV 之前**确定，**不得偷看 viability**。 |
| Q3 | `max_long_horizon = 0.8 s` 锁定。anchor 规则改为 `t1 >= H_short`、`t2 > t1`、`t3 > t2`、`t3 <= 0.8`；**不要求 `t1` 严格大于 0.3s**。medium 允许 `0.30/0.55/0.80`。 |
| Q4 | 开启**低威胁 fast-path**，但必须**同时满足 5 个条件**才能跳过 LTV；否则执行 LTV。记录 `viability_skipped` 与 `skip_reason`。 |

### 0.2 第 4 轮审查确认（细化）

| # | 决定 |
|---|---|
| 1 | `dynamic_buffer` 中 `closing_rate` 一律用 `max(0, closing_rate)`；并新增 **`max_dynamic_buffer`** 上限。 |
| 2 | mobility 的 clearance **不用全 8 方向 global min**，改为 **best corridor 内的 `corridor_min_clearance`**。 |
| 3 | mobility 归一化**尽量不用 FPGA 除法**：Python 浮点；FPGA 用**定点乘倒数常量**或**阈值分段**。 |
| 4 | mobility 必须同时使用 **future agent position + future obstacle position**；严禁 future agent + current obstacles。 |
| 5 | **`SceneState` 不新增 `long_horizon`**（重申）。 |
| 6 | evaluator **不用 mutable `set_scene_scale`**；改用显式、不可变的 **`ViabilityContext`**。 |
| 7 | 增加 **Top-K diversity test**：长期最优候选即使不是 short-term Top3，也必须有机会进入 LTV。 |
| 8 | 增加 **A/B toy test**：A 短期更安全但长期死路；B 短期略差但长期开阔；B 满足 hard safety 时必须允许胜出。 |
| 9 | corridor 使用 **hysteresis**，但 **hard unsafe 时立即允许切换**。 |

---

## 1. 本轮真正要解决的问题

两个候选：

```
A：短期前若干步更安全，但继续走进入死路（后段无逃逸空间）
B：短期风险略高，但其后仍有开放逃逸通道
```

只看 v0.4.1 的短期 FAR：**A 很容易获胜**。

目标不是"把 horizon 从 0.3s 拉到 0.8s"，而是：

> 在短期安全之上，增加"**长期仍具有持续运动能力**"的信息。

```
Short-Term Safety  +  Long-Term Viability
```

## 2. 明确不做

禁止 `17 candidates × 96 steps × all obstacles` 的朴素长 rollout：

- 计算量约 ×6；
- FPGA 时序/BRAM 膨胀，破坏 fixed-size 定位；
- 本仿真障碍是弹道的（恒速），线性外推对障碍精确；真正的不确定性来自"agent 后续会继续换动作"，0.8s 恒速是**粗粒度近似**；
- 我们只要**长期信息**（还有多少逃逸空间），不要高精度长期轨迹。

## 3. 最终架构（v0.5.1）

```
                    17 candidates  (base action space 固定不变)
                          │
                 v0.4.1 FAR + ASE
                          │
        ┌─────────────────┴──────────────────┐
        │  HARD SAFETY  (unsafe → 淘汰)       │
        │  CRITICAL SAFETY (critical 档)      │
        │  SHORT-TERM FUTURE RISK (分档)      │
        └─────────────────┬──────────────────┘
                          │
        shortlist = K = 4  (3 short-term leaders + 1 diversity)
                          │
        Long-Term Viability（仅对 shortlist）
          anchor × 8 sectors × future obstacles
          mobility(t1), mobility(t2), mobility(t3)
          viability = min(...)
                          │
        Escape Corridor（离散局部逃逸通道类 + corridor hysteresis）
                          │
        Arbitration：risk band 优先，band 内 viability 决定
                          │
               FES  →  behaviour  →  task
                          │
        CPU final action（action hysteresis / emergency）
```

保留 v0.4.1 全部机制：FAR、ASE、SafetyGate、FES、dwell、adaptive horizon、adaptive smooth、17 base candidates、high-risk refinement ≤8、action hysteresis、threat-aware speed、EmergencyFallback。

---

## 4. Top-K Policy（修正 1：短名单多样性）

### 4.1 漏洞

若 B 的短期排序未进 Top-3，则 viability **根本没有机会评估 B**，本算法逻辑上无法解决 A/B 问题。

### 4.2 两种方案比较

| | 方案 A：K=3 + diversity 替换 | 方案 B：K=4（3 leaders + 1 diversity） |
|---|---|---|
| 基础动作空间 | 17（不变） | 17（不变） |
| 长期评估数量 | 3 | 4 |
| 短期 leader 保留 | 只保留 2 个 leader | 3 个 leader 全保留 |
| 长期风险 | diversity 可能挤掉一个短期强候选 | 不再挤掉短期候选 |
| 计算 | `3·S·N` | `4·S·N`（+33%） |
| 推荐 | 资源极紧时 | **首选** |

**推荐方案 B（K=4）**。原因：本轮目标就是"长期信息纠正短期贪心"；为了多样性牺牲一个短期 leader 会引入新的盲区。`K·S·N = 12N`，相对 v0.4.1 的 `B·H·N` 仍很小。

### 4.3 确定性 diversity 规则

```
leaders = 可行候选中，按 (critical_band, risk_band, index) 升序的前 3 个
diversity = 在剩余可行候选中，选择与其“通道类/方向”最不重叠者：
    1) 若已能廉价得到通道类（见 §8 的当前帧代理类），优先取未被 leaders 覆盖的通道类中
       短期排序最靠前者；
    2) 否则取“到 3 个 leader 方向的最小角距”最大者（最大最小角距），
       并列时取短期排序靠前，再并列取 index 小者。
shortlist = leaders ∪ {diversity}     # ≤ K = 4
```

- 基础 action space **永远保持 17**；shortlist 只是"评估子集"，不是新动作空间。
- diversity 只做 1 个，避免 `17 × ...` 的膨胀。
- **[第3轮确认] diversity 必须在 LTV *之前*确定，不得偷看 viability。**
  diversity 只能使用：短期排序 `(critical_band, risk_band, index)`、候选方向、
  以及**当前帧**扇区/通道代理（`immediate.sectorize` 的当前帧结果，见 §8.1）。
  **禁止**读取 `mobility / viability / corridor_id` 等任何 anchor 级长期输出。
  这样 diversity 选择与长期评估完全解耦，不会出现"因为长期分高才被选进短名单"的自我实现泄漏。
  代价：diversity 可能选到一个长期其实很差的候选——这是可接受的，它只保证"方向覆盖"，
  最终是否胜出仍由 §10 的仲裁决定。

---

## 5. Future Positions（修正 2/9 硬性要求）

在 anchor `t` 处的 mobility/corridor 评估，**必须**用未来位置：

```
p_agent(a, t) = p_agent(0) + v_candidate(a) · t
p_obstacle(t) = p_obstacle(0) + v_obstacle · t        # 障碍同样外推
r(a, t)       = p_obstacle(t) − p_agent(a, t)
v_rel(a)      = v_obstacle − v_candidate(a)            # 与 FAR 同一候选条件化相对速度
```

**禁止**出现：

```
future agent position  +  current obstacle positions  →  sectorize
```

在 360° 高速动态弹幕下，那会让 viability 的含义错误（把已经离开/尚未到达的弹幕当成当前障碍）。
障碍外推与 agent 外推使用**同一个 t**、**同一套线性模型**，与 v0.4.1 FAR 的模型一致。

---

## 6. Anchor Policy（修正 6：归一化 + clamp）

### 6.1 场景尺度时间

```
T_scene = min(field_w, field_h) / max_speed
```

- `field_w/field_h` 来自观测 `env` 块（`ENV_OBS_DIM = 6`，`[field_w, field_h, sim_time, step_index, density, n_alive]`）。
- `max_speed = observation["player"][5]`。
- medium 预设：`T_scene = 480 / 200 = 2.4 s`。

### 6.2 生成 + clamp

```
raw_i  = c_i · T_scene          c = (0.125, 0.229, 0.333)
anchor_i = clamp(raw_i, H_short, max_long_horizon)      # 下界 = H_short（t1 >= H_short）
随后强制单调（保序）：
    anchor_2 = max(anchor_2, anchor_1 + anchor_eps)
    anchor_3 = max(anchor_3, anchor_2 + anchor_eps)
```

约束（实现时必须断言）—— **[第3轮确认] 放宽为 `t1 >= H_short`，不要求严格大于**：

```
H_short <= anchor_1 < anchor_2 < anchor_3 <= max_long_horizon
max_long_horizon = 0.8 s        # v0.5.1 锁定
```

- medium 场景允许 `0.30 / 0.55 / 0.80`，即 `t1 == H_short` 合法。
- **[第3轮确认] 取消 `min_anchor_gap`**：下界统一为 `H_short`，间距由 `anchor_eps` 保证。

`max_long_horizon` 是配置项（建议默认 `0.8 s` 或 `0.4 · T_scene`，取小者），防止大场地产生过长预测时间。

### 6.3 medium 实测标定

```
T_scene = 2.4s
raw = 0.30 / 0.55 / 0.80
clamp 后 = 0.30 / 0.55 / 0.80        # 恰好复现建议值，但值来自标定而非硬编码
max_speed·anchor_3 = 200·0.8 = 160 px < 480 px   # 仍在场内
```

### 6.4 7 项校验清单（保留并扩展）

1. 确认 `observation["env"][0:2] == (field_w, field_h)`。
2. 计算 `T_scene`，打印每个 level 的三锚点实际秒数。
3. `anchor_1 >= H_short`（允许等于；[第3轮确认] 不再要求严格大于）。
4. `H_short <= anchor_1 < anchor_2 < anchor_3 <= max_long_horizon (= 0.8s)`（clamp 后仍成立）。
5. `max_speed · anchor_3 < min(field_w, field_h)`（锚点仍在场内）。
6. `raw` 与 `clamp` 后的差值记录在 diagnostics（便于发现尺度异常）。
7. 任一断言失败 → 缩短锚点，**不得**为保留 0.8s 放宽断言。

---

## 7. Viability Definition（修正 4/14/15）

### 7.1 每个 anchor 的 8 扇区

在 `p_agent(a,t)` 处以 `r(a,t)`、`v_rel(a)` 复用 `immediate.sectorize()`：

**[第4轮确认] 未来位置硬性要求（重复强调，防实现走样）：**

```
p_agent(a,t) = p_agent(0) + v_candidate(a) · t      # future agent
p_obstacle(t) = p_obstacle(0) + v_obstacle · t      # future obstacle
r(a,t)       = p_obstacle(t) − p_agent(a,t)         # 两者都必须外推
```

严禁 `future agent position + current obstacle positions`。

```
closing_pos[d]    = max(0, closing_rate[d])   # [第4轮确认] 只取非负接近速率
dynamic_buffer[d] = min(base_buffer + closing_pos[d] · margin_time,
                        max_dynamic_buffer)   # [第4轮确认] 增加上限
clearance[d]      = nearest_clearance[d]      # 已含 agent_radius + obstacle_radius
future_margin[d]  = clearance[d] − dynamic_buffer[d]
free[d]           = future_margin[d] >= mobility_margin
```

- `closing_rate[d] > 0` 表示接近；`<= 0`（远离/切向）**不产生任何额外 buffer**。
- `max_dynamic_buffer` 防止高速接近时 buffer 无限膨胀（配置项，与 `escape_radius` 同尺度标定）。
- 判据不是 `clearance > 0`，而是**扣除 radii 与 dynamic buffer 后的 margin**。
- footprint 只用圆形（`agent_radius`）；**不引入 polygon footprint**。
- 跨越 DIR7 → DIR0 的**循环**处理 `max_continuous_safe_sector`。
- **ASE / LTV dynamic-buffer 命运（v0.5.1 决定）**：
  > 统一 ASE/LTV dynamic-buffer 语义留作后续独立重构，
  > v0.5.1 不同时改变 baseline。
  即：v0.5.1 的 LTV 使用 `max_dynamic_buffer = 20`，但**不修改** `far.py` / ASE 的
  `dynamic_margin`，以保持 v0.4.1 baseline 可比较。

### 7.2 每 anchor mobility（明确归一化，`[0,1]`）

先取 **best corridor**（最宽循环 free run），再在其内部统计：

```
safe_sector_count          ∈ {0..8}
max_continuous_safe_sector ∈ {0..8}          # 循环，DIR7→DIR0
corridor_min_clearance     = min( clearance[d] for d in best_corridor )   # [第4轮确认]
```

- **[第4轮确认] 不再使用全 8 方向的 global min clearance**：只用 **best corridor 内**的
  min clearance。理由：全向 min 会被"与所选通道无关的另一侧弹幕"污染，低估本通道质量。

```
mobility(t) = ( safe_sector_count / 8
              + max_continuous_safe_sector / 8
              + clear_term ) / 3
```

`clear_term` 的归一化 **[第4轮确认] 不依赖 FPGA 除法**：

```
clear_term        = clip(corridor_min_clearance · inv_clearance_ref, 0, 1)
inv_clearance_ref = 1 / clearance_ref        # 编译期常量，不是运行时除法
```

- Python：普通浮点乘法。
- FPGA：定点常量乘 + 右移（`round(inv_clearance_ref · 2^k)`），或直接用**阈值/分段**
  （例如 `>= ref → 1.0`、`>= 0.5·ref → 0.5`、否则 0），完全避免除法。
- `clearance_ref` 取与 `escape_radius` 同尺度（见 §17 待确认项）。
- **等权三分量**：第一版刻意不做加权搜索，避免多自由度。
- 结果确定、有界、可解释、FPGA 友好。

### 7.3 跨 anchor 聚合（修正 4）

**第一版只用 min：**

```
viability(a) = min( mobility(t1), mobility(t2), mobility(t3) )
```

- **删除** `0.4 / 0.4 / 0.2`（以及任何 anchor 权重 / `aggregate_alpha`）。
- 目的：先验证"中途进入死路会显著降低 viability"这一核心机制。
- `min` 与 `mean` / `weighted` / `min+mean` 的对比留到后续版本，本轮不引入。

---

## 8. Escape Corridor（修正 7/8/9/10）

### 8.1 定义边界（修正 8）

本项目**不实现 homotopy class**。它是：

> **离散局部逃逸通道类（Discrete Local Escape Corridor, DLEC）**

- 每个 anchor 在 `free[8]` 上取**循环连续 free 段（run）**；
- `corridor_width` = 最宽 run 的扇区数；
- `corridor_direction` = 该 run 的中心方向；
- `corridor_min_clearance` = 该（最宽）run 内 `clearance` 的最小值（供 §7.2 mobility 使用）；
- `corridor_id` = 跨 anchor 匹配（中心方向相差 ≤1 扇区视为同类）；
- 类别数量有限（≤8 个方向类），不做图搜索。

> 借鉴 TEB 的"不同候选属于不同长期路线类别 / 有限类别数 / 路线切换滞回"思想；
> **不实现** homotopy graph、不做完整 optimizer、不引入 ROS。

### 8.2 动态障碍（修正 9）

每个 anchor 的 `free[]` 必须来自 §5 的**未来障碍位置** `p_obstacle(t)` 与未来 agent 位置 `p_agent(a,t)`，不得静态化。

### 8.3 Corridor hysteresis（修正 10）

```
previous_corridor (上一帧选中的通道类)
new_corridor
仅当  new_viability − old_viability > corridor_switch_threshold   才切换
若   current corridor  hard unsafe                                → 立即切换
```

- 原则：**Safety > Hysteresis**。
- 该滞回作用于**通道类**；v0.4.1 的 **action hysteresis** 仍作用于最终动作，两者串联：
  先稳定通道类 → 再稳定动作。
- `corridor_persistence`（同类跨 anchor 出现次数）第一版只进 diagnostics，不进 cost。

---

## 9. FES 与 Long-Term Viability 的职责边界（修正 7）

两者都问"有没有出口"，必须用**时间尺度**分开，避免重复计算：

| 机制 | 时间尺度 | 回答的问题 | 数据 |
|---|---|---|---|
| FAR | 0 ~ H_short（0.3s） | 这个候选短期内危险吗？ | 3 个近端采样 |
| ASE | 0 ~ H_short | 是否 critical / unsafe？ | `min_margin`, `lateral_critical_risk` |
| **FES / trap** | 短期轨迹**端点**（`escape_radius`≈50px） | 短期轨迹落点**局部**还有几个逃逸方向？ | 端点前/左/右三向探测 |
| **Long-Term Viability** | 0.3 ~ 0.8s（anchors） | 更远尺度上，是否**仍能持续运动**？ | anchor × 8 扇区 mobility |
| **Escape Corridor** | 同上 | 远期**可持续逃逸方向/通道类别** | 循环 free run |

- **FES = short/intermediate escape；Viability = longer-term sustainability。**
- v0.5.1 **不修改** FES 的既有定义与实现；LTV 使用**短 horizon 之外**的 anchor 与自己的 mobility 特征。
- 若实现后 FES 与 viability 高度相关，应作为"重新划分职责"的信号，而不是简单叠加。

---

## 10. Decision Hierarchy / Arbitration（修正 3）

### 10.1 不做固定 winner

第 1 版的 `critical > viability > future-risk` 会让 viability 绝对压过短期风险，可能让"长期好但当下明显更危险"的候选胜出。改为**分档（bucketed）仲裁**，杜绝无限加权：

```
critical_band(a) = 0 if envelope normal else 1          # 0/1
risk_band(a)     = floor( future_risk_score(a) / risk_band_width )   # 风险量化档

ranking key（越小越优，字典序）：
(
  0 if feasible else +inf,            # ① HARD SAFETY：unsafe 直接淘汰
  critical_band,                      # ② CRITICAL SAFETY：critical 不能被长期分复活
  risk_band,                          # ③ SHORT-TERM FUTURE RISK：分档
  -viability if a in shortlist else +1.25,   # ④ LONG-TERM VIABILITY：仅在档内比较
  fes_score,                          # ⑤ FES
  behaviour_score,                    # ⑥ smooth + dwell
  task_score,                         # ⑦ progress
  index                               # 确定性
)
```

> **[第3轮确认] `risk_band_width = 0.05`（固定）**：它是**风险分桶宽度**，
> **不是**"允许 5% 碰撞风险"，也不是碰撞概率阈值；它只回答"多小的短期风险差异算同一档"，
> 从而决定 viability 是否被允许打破这个差异。

### 10.2 语义（对应审查第 6 节）

1. hard unsafe → 永远淘汰；
2. critical candidate **不能**因长期 viability 高而复活（`critical_band` 优先）；
3. 两个候选都安全且**同一风险档**（短期差异 ≤ `risk_band_width`）→ viability 打破平局（解决 A/B）；
4. 短期风险差异**跨档**（较大）→ 更安全者胜，viability 不能小幅优势推翻（`risk_band` 优先）；
5. `risk_band_width` 是唯一的仲裁尺度参数，**固定 0.05**（[0,1] 风险尺度上）。

这就是"用 Pareto/分档代替无限堆权重"的落地：`viability` 只纠正短期**轻微**贪心，不能覆盖明显即时危险。

### 10.3 短名单外候选

```
viability_cost(a) = 1 − viability(a)      a ∈ shortlist
viability_cost(a) = 1 + shortlist_margin  a ∉ shortlist      (shortlist_margin = 0.25)
```

用 `+1.25` 保证短名单外候选**不可能**在 viability 层胜出（即使短名单内 viability 全为 0）。

---

## 11. API Plan（修正 5：不修改 SceneState）

### 11.1 明确不新增 `SceneState.long_horizon`

`SceneState` 只表达**当前环境状态**；anchors / long horizon / K / thresholds / mobility 参数都属于 **planner / evaluator configuration**。这对未来 Sim-to-Real 很重要。

### 11.2 新增 `core/cpu/decision/viability.py`（实现时）

```python
@dataclass(frozen=True)
class ViabilityConfig:
    # shortlist
    top_k: int = 4
    leaders: int = 3
    diversity: int = 1
    shortlist_margin: float = 0.25
    # anchors（绝对秒；None -> 由 scene_scale × anchor_ratios 派生并 clamp）
    anchors: tuple[float, float, float] | None = None
    anchor_ratios: tuple[float, float, float] = (0.125, 0.229, 0.333)
    max_long_horizon: float = 0.8        # v0.5.1 锁定
    anchor_eps: float = 1e-3
    # mobility
    mobility_margin: float = 0.0
    clearance_ref: float = 90.0          # 与 escape_radius 同尺度（§17 待确认）
    max_dynamic_buffer: float = 20.0     # [第4轮确认] dynamic buffer 上限
    # inv_clearance_ref = 1 / clearance_ref 由 config 派生（编译期常量，非运行时除法）
    # arbitration
    risk_band_width: float = 0.05        # 分桶宽度（非碰撞概率）
    # corridor
    corridor_min_width: int = 2
    corridor_switch_threshold: float = 0.10
    # low-threat fast path（必须“全部满足”才跳过 LTV，见 §14.1）
    fast_risk_band_max: int = 0
    fast_margin_min: float = 4.0
    fast_openness_min: float = 0.5
    fast_decision_gap: float = 0.05

@dataclass(frozen=True)
class FutureMobility:
    anchor: float
    safe_sector_count: np.ndarray
    max_continuous_safe_sector: np.ndarray
    corridor_min_clearance: np.ndarray   # [第4轮确认] best corridor 内，而非全向
    mobility: np.ndarray
    corridor_id: np.ndarray
    corridor_width: np.ndarray
    corridor_direction: np.ndarray

@dataclass(frozen=True)
class ViabilityProfile:
    viability: np.ndarray          # (B,)
    shortlist: np.ndarray          # (B,) bool
    per_anchor: tuple[FutureMobility, ...]
    anchors: np.ndarray            # (S,)

@dataclass(frozen=True)
class ViabilityContext:
    """显式、不可变的每帧上下文；替代 mutable set_scene_scale。"""
    short_horizon: float                 # = H_short
    scene_scale: float | None = None     # = T_scene（适配器/控制器显式提供）

class LongTermViabilityEvaluator:
    """无状态（纯函数式）评估器；场景尺度通过 ViabilityContext 显式传入。"""
    def __init__(self, config: ViabilityConfig | None = None) -> None: ...
    def evaluate(
        self, scene, actions, short_term_cost, critical_band,
        *, context: ViabilityContext,
    ) -> ViabilityProfile: ...
```

- **[第4轮确认] 取消 mutable `set_scene_scale`**：`T_scene` 由**知道场地尺寸的适配器/控制器**
  构造 `ViabilityContext(short_horizon=H_short, scene_scale=T_scene)` 并**显式传入**。
  `scene_scale=None` 时回退到 `anchor_ratios × short_horizon` 再 clamp。
  `SceneState` 保持游戏无关、不新增字段。
- **corridor hysteresis 的记忆状态不放 evaluator**（保持无状态/纯函数）：放在 `ActionPlanner`
  （与 action hysteresis 同处），由 planner 传入/保存 `previous_corridor`。
- `PlanDiagnostics` 新增：`chosen_viability, chosen_safe_sector_count,
  chosen_corridor_min_clearance, chosen_corridor_width, chosen_corridor_direction,
  chosen_corridor_persistence, viability_evaluated, long_anchors, viability_skipped,
  skip_reason, viability_run_reason, short_term_us, viability_us`。
- `CpuDecisionLayer.decide(scene)`、`ActionPlanner.plan(scene)` 签名不变。

---

## 12. FPGA Mapping（修正 17）

FPGA 适合的固定形状运算：

```
candidate (K≤4) × anchor (S=3) × obstacle (N)
  relative position
  distance²
  clearance
  blocked / free
  sector count
  max gap（8 步循环 run）
  min clearance
```

- 只用 add / multiply / square / compare / min / max；一次 guarded divide（与 FAR 同型）。
- `clear_term` 归一化用 `inv_clearance_ref` **常量乘 + 右移**，或用**阈值分段**（§7.2），
  **不做运行时除法**。
- 无 atan2、无大量浮点、无动态对象、无不定长 recursion。
- 向 CPU 提供：`mobility, safe_sector_count, max_gap, corridor_id, corridor_width, min_future_clearance`。
- 上限固定：K=4、S=3、8 扇区 → 最坏 `O(K·S·N)`。

## 13. CPU Role

CPU 保留：**shortlist 选择（含 diversity）、通道类选择与滞回、viability vs risk 仲裁、FES、behaviour、task、action hysteresis、emergency**。FPGA/底层未来最多并行产出 §12 的定长特征，不替 CPU 选动作。

---

## 14. Complexity 与 Performance Budget（修正 16）

| 阶段 | 复杂度 | 说明 |
|---|---|---|
| 短期（v0.4.1，不变） | `O(B·H·N)` + `O(B·S_short·N)` | B=17 |
| 长期（新增，仅 shortlist） | `O(K·S·N)` ≈ `12N` | K=4, S=3 |

禁止：`17 × 96 × N`、Python 多层循环、为长期层保存完整轨迹。要求 NumPy 批量。

### 14.1 低威胁快速跳过（关键）

长期层只在**存在需要打破的短期平局**时执行：

```
skip_ltv = ALL of:
  1) risk_band(best) <= fast_risk_band_max            # 低风险
  2) envelope.level(best) == 0                        # 无 critical（最优候选非 critical）
  3) envelope.min_margin(best) >= fast_margin_min     # 当前安全裕量充足
  4) openness_now(best) >= fast_openness_min          # 当前空间足够开阔（当前帧代理，非 LTV）
  5) risk_score(second_best) - risk_score(best) >= fast_decision_gap   # 短期选择足够明确
若任一条件不满足 → 必须执行 LTV
```

- **[第3轮确认] 不能用"低威胁 + 无短期平局"两条判断**；必须**同时满足上述 5 条**。
- 条件 4 的 `openness_now` 用**当前帧** 8 扇区 free 比例（`immediate.sectorize`，非 anchor 级），
  与 diversity 代理同源，不偷看任何 LTV 输出。
- **记录**：`viability_skipped: bool`、`skip_reason: str`。
  跳过时 `skip_reason = "fast_path:low_risk+no_critical+margin+openness+clear_choice"`；
  未跳过时 `viability_skipped=False`、`skip_reason=""`，并另记
  `viability_run_reason` = 第一个未满足的条件（便于解释"为什么这帧跑了长期层"）。
- 低威胁且 5 条全满足 → 成本≈v0.4.1；A/B 场景短期风险接近（条件 5 不满足）→ 必须执行 LTV。

### 14.2 延迟统计要求

实现后必须**分别**报告：

```
short_term_us   (v0.4.1 部分)
viability_us    (新增部分)
total_us
viability_skipped 比例
```

而不是只报 total。

### 14.3 性能风险

- K 或 N 增大时 `O(K·S·N)` 上升；
- `risk_band_width` 过小 → 频繁触发长期层；
- `ViabilityContext.scene_scale=None` → 回退锚点必须安全（用 `H_short` 派生并 clamp）；
- 与 action hysteresis 交互可能使首选方向抖动 → 监测 action change。

---

## 15. Test Plan（本轮只设计，不实现）

### 15.1 关键 Toy Scenario（A/B/C）

固定场景（数值实现时定稿，结构如下）：

- agent：`p=(0,0)`，`radius=1`，`max_speed=10`，`H_short=0.3`，`T_scene` 使之 `anchors=(0.30,0.55,0.80)`。
- **Candidate A**（heading 0°）：0~0.3s 内 clearance 高（短期风险低）；但在 `t2/t3` 的未来位置被 U 形障碍包围（后段 dead-end）。
- **Candidate B**（heading 45°）：起点附近有一枚弹幕（短期风险**略高**），但三锚点通道持续开放（mobility 高）。
- **Candidate C**（heading 180°）：与障碍硬碰撞（unsafe），但人为给高 viability。

断言：

1. `future_risk(B) > future_risk(A)` 但二者在**同一 risk_band**（差异 ≤ `risk_band_width`）；
2. `viability(B) > viability(A)`（A 在 t2/t3 的 mobility 低）；
3. **最终选择必须允许 B 胜出**（证明长期信息能纠正短期贪心）；
4. `C` 必须被淘汰（证明 hard safety 不能被长期 viability 覆盖）；
5. hard unsafe 场景下 `emergency` 行为不变。

### 15.2 其余测试（A–L）

| 编号 | 测试 |
|---|---|
| A | **A/B long-term toy scenario**：A 短期更安全但长期死路；B 短期略差但长期开阔；B 满足 hard safety 时**必须允许胜出** |
| B | hard unsafe + 高 long-term viability candidate 必须淘汰 |
| C | midway dead-end：`mobility = 0.8 → 0.2 → 0.9`，viability 必须显著低于均匀 `0.7`（min 聚合生效） |
| D | complete dead-end：三锚点全 0 → viability=0 |
| E | 多个 corridor：左/中/右通道能被分成不同 `corridor_id` |
| F | same corridor candidates：同类通道的分辨与排序 |
| G | dynamic obstacles：障碍移动使某候选"现在开阔、未来封闭"；用未来位置后判定正确 |
| H | **Top-K diversity**：长期最优候选即使不是 short-term Top3，也**必须有机会进入 LTV**（由 diversity 短名单保证） |
| I | corridor hysteresis：小幅 viability 优势不切换通道；hard unsafe 立即切换 |
| J | anchor clamp：大/小场地与不同速度下，`anchor_1>H_short`、单调、`≤max_long_horizon`、场内 |
| K | mobility normalization：结果始终 `∈[0,1]`，`DIR7→DIR0` 循环 run 正确 |
| L | performance / allocation：`viability_skipped` 低威胁为真；长期仅对 ≤K 候选计算；分项延迟可测；无 Python 嵌套循环 |

要求：H 与 A/B 是本设计的**逻辑正确性**证明；若不能通过，v0.5.1 无法解决 A/B 问题。

---

## 16. Open-Source References

| 项目 | 借鉴 | 不采用 |
|---|---|---|
| PythonRobotics DWA | 有限时域 candidate rollout | 差速模型、长 horizon 全 rollout |
| Nav2 MPPI | trajectory evaluation、critic separation、obstacle/goal 分离 | 完整 MPPI、大采样、ROS |
| TEB | 不同局部路线类别、有限类别数、路线切换滞回 | homotopy graph、完整 optimizer、ROS |
| CommonRoad | feasibility 与 utility 分离 | 完整 checker |
| F1TENTH | gap 表达、immediate safety / iTTC | 激光/Ackermann、原始 Follow-the-Gap（已证失败） |
| TinyMPC | fixed-size、embedded-friendly、资源受限 | 完整 MPC solver |
| RVO2 / ORCA | relative velocity / velocity-space 思想 | ORCA 完整实现 |

> 只借鉴结构思想；不复制 code / ROS / MPC solver / MPPI solver / ORCA 完整实现 / TEB 完整优化器。依赖仍为 Python + NumPy。

---

## 17. 已确认决定与待评审项

**已确认（第 3 轮审查）**
- Q1：`risk_band_width` **固定 0.05**（风险分桶宽度，**不是**允许 5% 碰撞风险）。
- Q2：**K=4 = 3 short-term leaders + 1 diversity**；diversity 在 LTV 之前确定，**不偷看 viability**。
- Q3：`max_long_horizon = 0.8s` 锁定；anchor 规则 `t1 >= H_short`、`t2 > t1`、`t3 > t2`、`t3 <= 0.8s`；
  medium 允许 `0.30/0.55/0.80`（`t1 == H_short` 合法）；取消 `min_anchor_gap`。
- Q4：低威胁 fast-path **开启**，但必须**同时满足 §14.1 的 5 个条件**；记录 `viability_skipped` 与 `skip_reason`。
- **第 4 轮细化（见 §0.2）**：`max(0, closing_rate)` + `max_dynamic_buffer`；`corridor_min_clearance`
  （best corridor 内）；归一化免 FPGA 除法；future agent + future obstacle；不改 `SceneState`；
  `ViabilityContext` 取代 mutable setter；Top-K diversity 测试；A/B toy 测试；corridor hysteresis + hard-unsafe 立即切换。
- 范围：设计审查 + 设计稿修正，**不实现代码**。
- 实现时版本号：**`v0.5.1-LongTermViability`**；预测式算法基线仍 `v0.4.1-SideHit-ASE`。
- 不新增 `SceneState` 字段；不修改 planner/predictor/risk/cost 算法。

**仍待确认（实现前唯一剩余项）**
- `clearance_ref` 取哪个尺度：`escape_radius`（=50）还是 `max_speed·reaction_time`（medium=90）？
  设计稿默认 `90.0`；建议改为与 `escape_radius` 统一为 `50`，避免项目里出现两套"安全尺度"。
