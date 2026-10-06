# Dynamic-Forge 决策算法 v0.4.0-FAR 设计说明

> 本文说明从 `v0.3-5-SafetyGate` 到 `v0.4.0-FAR` 的算法升级。
> 借鉴的开源项目结构思想在文末统一列出。

---

## 1. v0.3-5 的问题

v0.3-5 已经是「安全门控 + 17 全向候选 + 多目标代价」，但 1000 局实测：

| 控制器 | survival | collision | decision | changes |
|---|---|---|---|---|
| Reactive | 1004.2 ± 251.4 | 50.1% | 27.8 µs | 716.0 |
| Predictive v0.3-5 | 895.6 ± 257.4 | 74.8% | 1933.8 µs | 103.1 |

预测式失败类型：`front_hit 212 / side_hit 482 / corner_trap 48 / other 6`。

三个结构性问题：

1. **风险仍然是「当前风险」**：`_estimate_min_ttc` 用玩家**当前速度**，候选动作只通过离散采样后的 clearance 间接影响结果；
2. **candidate-specific 的未来信息不足**：只在整个 horizon 上取 `min_clearance / ttc_min`，没有刻画「风险是在上升还是下降」；
3. **安全与效用仍混在一个加权和里**：`progress` 与 `collision` 只是权重不同，没有层级保证。

## 2. 为什么 side_hit 成为主要失败类型

`side_hit` = 撞击方向与航向夹角 > 60°。它本质是**侧向弹幕在预测窗口内与候选轨迹相交**，而 v0.3-5：

- 采样点上的 clearance 只能知道「现在已经多近」，不能表达「按这个候选走，未来相对速度是否让它更近」；
- 相对速度 `v_rel = v_o − v_agent` 中，`v_agent` 在 FAR 之前并未真正随候选变化用于风险（只用于碰撞采样）；
- 于是「向左侧移」和「向右侧移」在很多帧里得到几乎相同的风险，规划器只能靠 trap/dwell 等次级项挑方向，侧向被扫中。

**FAR 的核心修复**：把风险显式写成 `v_rel(a)` 的函数，并给出风险随时间的形状。

## 3. FAR 设计（Future Action Risk）

```
17 candidates
   ↓  far.candidate_safety_batch        # 连续最近接近安全判定（可行/不可行）
   ↓  far.future_action_risk_batch      # 未来风险剖面 current/near/mid/far/peak/trend
   ↓  far.future_risk_critic            # Level 2 critical risk ∈ [0,1]
   ↓  TrajectoryCost.utility_terms      # Level 3 FES / Level 4 behaviour / Level 5 task
   ↓  ActionPlanner.plan (argmin)       # CPU 最终决策
```

回答的问题从「当前这个位置危险吗？」变成「执行候选 `a` 后，未来短时间会变安全还是更危险？」。

## 4. Candidate-conditioned relative velocity

真实运动模型仍是项目原有的一阶全向速度模型，**未引入汽车/差速模型**：

```
p_agent(t)    = p_a + v_candidate(a) · t
p_obstacle(t) = p_o + v_o · t
r(t)  = p_o(t) − p_agent(t) = r0 + v_rel(a) · t
r0    = p_o − p_a
v_rel(a) = v_o − v_candidate(a)
```

关键点：`v_rel` 是 **`a` 的函数**，所以 `LEFT` 与 `RIGHT` 的相对速度不同 → 风险不同。
实现见 `far.candidate_safety_batch` / `future_action_risk_batch`，对 `(candidate, obstacle)` 全向量化。

## 5. t_ca / d_ca（连续时间最近接近）

对每个 `candidate × obstacle`，在恒速模型下解析求最近接近：

```
t_ca = clamp( −dot(r0, v_rel) / |v_rel|² , 0, horizon )
d_ca² = |r0|² + 2·dot(r0, v_rel)·t_ca + |v_rel|²·t_ca²
collides = any( d_ca ≤ r_agent + r_obstacle )
```

工程保护（对应「不允许 NaN/Inf 污染 batch」）：

- `|v_rel|² ≈ 0` → 取 `t_ca = 0`，`d_ca = |r0|`（分离恒定）；
- `d_ca²` 负值截断为 0 再开方；
- 已重叠（`|r0| ≤ r_sum`）→ 直接 `collides`，且 `min_ttc = 0`。

**命名纪律**：`t_ca` 是「恒速最近接近时间」，**不是碰撞时间**。严格 TTC 只在
「正在接近 **且** 最近接近进入膨胀半径」时才有定义，此时

```
t_contact = t_ca − sqrt(r_sum² − d_ca²) / |v_rel|
```

代码中只有这个量叫 `min_ttc`；`t_ca`、`d_ca` 始终保留原名。这是对 v0.3-5 中
「把 `-dot/v²` 当 TTC」这一用词问题的修正。

## 6. risk_near / mid / far

为控制 CPU 开销并保持 FPGA 友好，只在 horizon 内取 **3 个固定采样点**：

```
t_near = min(0.08, horizon)
t_mid  = min(0.16, horizon)
t_far  = horizon
```

每个采样点、每个障碍物：

```
d(t)       = |r0 + v_rel·t|
clearance  = d − (r_agent + r_obstacle)
closing    = max(0, −dot(r(t), v_rel)) / d          # 径向接近速度
ttc        = max(clearance,0)/closing               # 不接近则 +inf
risk       = clip(1 − ttc / ttc_reference, 0, 1)    # 线性 iTTC；> ttc_ref 记为 0
risk       = 1  若 clearance ≤ 0                    # 已穿透
risk(t)    = max over obstacles
```

采用线性 iTTC 而非 `ttc_ref/(ttc+ttc_ref)`：后者在 `ttc≈1.6s` 时仍给出 0.385 的风险，
会让 0.3–0.6 s 的预测窗口对远处障碍过度保守。线性形式在 `ttc ≥ ttc_ref` 时归零，
与预测时域尺度一致，且只需 1 次乘法 + 截断。

## 7. risk_trend

```
risk_peak  = max(risk_near, risk_mid, risk_far)
risk_trend = risk_far − risk_near      # 有符号
```

- `risk_trend > 0` → 未来正在恶化（例如持续接近、close 但尚未碰撞）；
- `risk_trend < 0` → 未来正在改善（例如即将掠过并远离）。

critic 只惩罚正部分 `clip(risk_trend·trend_gain, 0, 1)`，但有符号原值保留在
`CandidateRiskProfile` 中用于诊断。这正是「不要只看当前 risk」的落点。

## 8. Hierarchical critics（分层决策）

`HierarchicalCritics` 用**位置制权重**保证严格层级（每层归一到 `[0,1]`）：

| 层级 | 含义 | 权重 |
|---|---|---|
| Level 1 | Safety feasibility（连续碰撞 → 硬淘汰） | 硬过滤 |
| Level 2 | Critical future risk（min_ttc / min_clearance / risk_peak / risk_trend / speed 耦合 / 近碰撞 penalty） | `risk = 1000` |
| Level 3 | Future Escape Space（FES / trap_cost） | `fes = 100` |
| Level 4 | Behaviour quality（smooth + dwell） | `behavior = 10` |
| Level 5 | Task objective（progress） | `task = 1` |

因为 `100 + 10 + 1 = 111 < 1000`，**任何低层级的总分都无法覆盖高层级一个单位**，
即 `progress` 再高也不可能战胜更安全的动作。`HierarchicalCritics.__post_init__`
强制 `risk > fes + behavior + task`，从构造上保证该性质。

**Threat-aware speed selection**：Level 2 内含连续耦合项

```
speed_component = clip( (speed / max_speed) · risk_peak · speed_risk_gain, 0, 1 )
```

高威胁时 FAST 候选被连续惩罚，而非「fast → invalid」的硬开关；低风险时该项为 0，
FAST 不受影响（`speed_risk_gain` 可配置）。

## 9. FES（保留）

`TrajectoryCost.utility_terms()` 复用原有 `_trap_cost`（端点前/左/右三向逃逸空间探测，
`escape_radius` 可配置）与其权重 `w_trap`，仅作为 **Level 3**。v0.3-5 已把 `other`
降到 6，FES 与 dwell 因此被完整保留，没有被 FAR 取代。

## 10. SafetyGate（保留）

`SafetyGate` 保留两层作用：

1. `collision_threshold`（可配置）作为**额外 clearance 余量**，与 FAR 连续碰撞一起构成 Level 1；
2. `critical_penalty`（`near_miss_threshold` / `critical_weight` 可配置）归一化后并入 Level 2。

FAR 的连续最近接近判定是**主**可行性判据，SafetyGate 提供可配置边界与近碰撞惩罚，
两者概念上仍是「可行性 vs 效用」的分离（CommonRoad）。

## 11. FPGA mapping

- 主体运算：`candidate × obstacle`（安全）与 `candidate × sample × obstacle`（风险），
  `S = 3` 固定；全部是 add / multiply / compare / square，加 1 次 sqrt 与受保护的除法；
- 无 `atan2`、无动态对象、无变长循环；
- 每个 candidate 最终压缩为定长特征：
  `safe, min_ttc, min_clearance, risk_near, risk_far, risk_trend, risk_peak`；
- Level 3–5 是每候选标量/短向量工作，可由 CPU 完成。

## 12. CPU role

**CPU 始终拥有最终动作选择权**：FES、goal/progress、smooth、dwell 以及
`argmin` 都在 CPU（`ActionPlanner.plan`）完成。FPGA/底层模块未来最多并行产生上节
的定长特征，**不替 CPU 选动作**。紧急回退也由 CPU 确定性执行。

## 13. Complexity

| 阶段 | 复杂度 |
|---|---|
| 轨迹 rollout（既有 `LinearPredictor`） | `O(B·H·N)` |
| FAR 连续安全 | `O(B·N)` |
| FAR 风险剖面 | `O(B·S·N)`，`S = 3` |
| 分层 critic 汇总 | `O(B)` + 每候选 `O(N)`（trap） |

`B = 17`。相对 v0.3-5 的 `O(B·H·N)` 批量 clearance，FAR 用 `O(B·S·N)`（`S` 远小于 `H`）
替换之，因此核心新增并不带来数量级开销。

## 14. 与 DWA / MPPI / F1TENTH / TinyMPC 的关系

| 项目 | 借鉴 | 不采用 |
|---|---|---|
| PythonRobotics DWA | 候选动作 → 未来轨迹 → 轨迹评价 → 最优 | yaw-rate / 差速模型 |
| Nav2 MPPI | trajectory batch evaluation、critic 分层、true collision 与 near-collision 分开、整条轨迹评价、fallback、向量化 | 大型 MPPI、1000+ 采样、完整 ROS 架构 |
| CommonRoad Drivability Checker | 碰撞/可行性检查与轨迹 utility 概念分离 | 完整 checker |
| F1TENTH | iTTC / instantaneous collision risk（距离 + 接近速度） | Ackermann 模型 |
| TinyMPC | 资源受限预测控制、定长计算、嵌入式友好 | 完整 MPC solver |

> **本项目借鉴这些开源项目的结构思想，并针对 FMQL30TAI 资源受限异构平台进行了简化和重构，
> 不是直接复制这些项目。**

本轮禁止项均未引入：无强化学习、无神经网络、无 NPU、无 C++/Verilog/HLS、
无 CasADi/CVXPY、无完整 MPPI/MPC、无 ORCA、无 A*/Dijkstra、无大规模随机采样、
候选数量仍为 17、依赖仍为 Python + NumPy。
