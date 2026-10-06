# Dynamic-Forge 决策算法 v0.4.1-SideHit-ASE 设计说明

> 本文说明在 `v0.4.0-FAR` 基础上加入 **Adaptive Safety Envelope** 的结构性升级。

---

## 1. v0.4.0 的结果

1000 局（与 Reactive 同一批场景）：

| 控制器 | survival | collision | decision | changes |
|---|---|---|---|---|
| Reactive | 1004.2 ± 251.4 | 50.1% | 28.4 µs | 716.0 |
| Predictive v0.4.0-FAR | 928.1 ± 252.3 | 69.9% | 1004.8 µs | 150.5 |

失败类型：`front_hit 189 / side_hit 447 / corner_trap 62 / other 1`。

## 2. side_hit 问题

`side_hit` = 撞击方向与航向夹角 > 60°。FAR 已经让风险依赖候选动作，但：

- 风险主要来自**距离/几何 clearance**；当障碍物高速**横向**接近时，几何 clearance 在预测窗口内可能仍然为正，于是候选被判为「可行且低风险」；
- 这种危险本质是「按当前相对速度，未来几帧就会贴上」，而不是「现在已经很近」；
- 结果：侧向弹幕扫过时，规划器没有把「横向接近速率」当成一等公民，side_hit 占比最高；
- 同时 FAR 的高频重排让动作变化从 v0.3-5 的 103 升到 150.5。

v0.4.1 因此只做两件事：**建立候选级的横向接近安全包络**，以及**安全优先的动作滞回**。

## 3. closing_rate（候选级横向接近速率）

```
r0    = p_obstacle − p_agent
v_rel(a) = v_obstacle − v_candidate(a)
closing_rate(a) = max over obstacles of  max(0, −dot(r0, v_rel(a)) / max(|r0|, ε))
```

因为 `v_rel` 依赖候选，`LEFT`/`RIGHT`/`DIAGONAL` 得到不同值——这正是 side_hit 的核心。
只取 `> 0`（距离正在减小），单纯的横向平移（无径向接近）得到 0，不会误报。
实现：`far.future_action_risk_batch`，向量化 `(B, N)`。

## 4. safety_margin（动态安全余量）

```
dynamic_margin(a) = base_margin + closing_rate(a) · margin_time
safety_margin(t)  = clearance(t) − dynamic_margin
min_margin        = min over 未来采样点/障碍物 of safety_margin(t)
```

`clearance` 已经扣除 `agent_radius + obstacle_radius`。这不是完整物理预测，而是一个快速
保守缓冲：`margin_time` 把「接近速度」折算成距离。

三档：

| 档位 | 条件 | 处理 |
|---|---|---|
| LEVEL 0 normal | `min_margin ≥ critical_margin` | 正常代价 |
| LEVEL 1 critical | `0 ≤ min_margin < critical_margin` | 可行，加 critical penalty |
| LEVEL 2 unsafe | 真碰撞 **或** `min_margin < 0` | 直接 infeasible |

`base_margin / margin_time / critical_margin` 全部可配置，默认
`0.0 / 0.04 s / 4.0`；第一版未做参数搜索。

## 5. critical dynamic risk（side-hit 专用 critical）

```
rate_frac  = closing_rate / max_speed
rate_gate  = clip((rate_frac − thr_frac) / scale_frac, 0, 1)
margin_term  = clip((critical_margin − min_margin) / critical_margin, 0, 1)
ttc_term     = clip(1 − min_ttc / ttc_reference, 0, 1)          # 仅有限 TTC
trend_term   = clip(risk_trend · trend_gain, 0, 1)
lateral_critical_risk = rate_gate · max(margin_term, ttc_term, risk_peak, trend_term)
```

`rate_gate` 使得 **closing_rate 低于阈值时该项恒为 0**（正常横向移动不触发），
高于阈值后连续增长。它不走「lateral → huge penalty」的硬分支，而是进入
LEVEL 2 critical tier（见第 9 节），由位置制权重保证优先级。实现：
`far.critical_risk_critic`。

## 6. adaptive refinement（高风险局部细化）

基础动作空间**仍是 17**（8 朝向 × 2 速度 + stop），不永久扩张。

触发条件（任一）：

```
feasible_count ≤ feasible_threshold
OR provisional risk_score > risk_threshold
OR provisional min_margin < critical_margin
```

触发后围绕**当前最优非静止方向**增加临时候选：

```
角度偏移：−15°, −7.5°, +7.5°, +15°   × 2 个速度档
```

最多 `max_extra = 8`（常态 17，危险态 ≤ 25）。细节：

- refinement 候选与基础候选一起走**完全相同**的 FAR / SafetyEnvelope / FES / hysteresis；
- **不递归**（只细化一层）；
- 低风险永远不触发：空场景实测 `refine=False, feasible=17`；
- `provisional` 由真实 critic 层级选出（critical → future risk → **较大** margin），
  因而细化一定围绕真正的安全逃逸方向。

## 7. action hysteresis（安全优先滞回）

```
若 held 可行、LEVEL 0，且 risk_trend 未恶化：
    仅当新动作“明显更安全”（critical 增益 > critical_advantage
                              或 future-risk 增益 > risk_advantage
                              或 total 增益 > switch_margin）才切换，否则保持 held
若 held unsafe / critical / trend 恶化 / 不在候选集 / 不可行：立即切换
```

安全永远优先：滞回只在 held 确实安全时生效，绝不会为了平滑而坚持危险动作。
切换原因写入 `PlanDiagnostics.switch_reason`（`held_unsafe` / `held_critical` /
`held_trend_worsening` / `clear_critical_gain` / `clear_risk_gain` / `clear_gain` /
`hysteresis_hold` / `held_not_in_set` / `held_infeasible` / `emergency_fallback`）。

## 8. threat-aware speed

速度惩罚是**连续**的，不做 `if fast → invalid`：

```
threat = max(risk_peak, clip(1 − min_ttc/ttc_ref, 0, 1), clip(1 − min_clearance/near_miss, 0, 1))
speed_component = clip((speed/max_speed) · threat · speed_risk_gain, 0, 1)
```

低威胁场景 `threat = 0` → FAST 完全不受罚；威胁上升时 FAST 承担更高未来风险代价，
但仍可被选择（保留急转/逃逸能力）。

## 9. hierarchical decision（六层）

| 层级 | 内容 | 权重 |
|---|---|---|
| LEVEL 1 | hard safety feasibility（envelope unsafe 淘汰） | 硬过滤 |
| LEVEL 2 | critical dynamic safety（lateral critical / LEVEL 1 / 近碰撞 penalty） | `critical = 10000` |
| LEVEL 3 | future risk（risk_peak / ttc / clearance / trend / speed） | `risk = 1000` |
| LEVEL 4 | FES / escape（trap_cost） | `fes = 100` |
| LEVEL 5 | behaviour（smooth + dwell） | `behavior = 10` |
| LEVEL 6 | task（progress） | `task = 1` |

每层归一到 `[0,1]`，权重是**位置制**：`HierarchicalCritics.__post_init__` 强制
`critical > risk > fes > behavior > task` 且每层严格大于所有更低层之和
（10000 > 1111，1000 > 111，100 > 11，10 > 1）。因此 progress/smooth 再高也无法
「买回」一个更危险的候选。这不是无限提高权重，而是有限的位置制排序。

## 10. FPGA mapping

- 新增 `candidate × obstacle` 的 closing-rate 内核（dot / sqrt / compare / max）；
- 新增 `candidate × sample × obstacle` 的 margin min-reduce（add / multiply / min）；
- 全部是 add / multiply / square / compare / min / max，无 atan2、无动态对象、无递归；
- 最坏情况候选数固定为 **25**（17 + 8）；refinement 的「是否启用」由 CPU 的条件分支决定，
  进入 FPGA 后仍是固定上限；
- 每个候选最终压缩为定长特征：
  `safe, risk_peak, risk_trend, min_ttc, min_clearance, safety_margin, closing_rate`。

## 11. CPU role

CPU 负责：是否触发 refinement、hysteresis 判定、FES、progress/task、最终 `argmin`
以及紧急回退。FPGA/底层未来最多并行产生第 10 节的定长特征，**不替 CPU 选动作**。

## 12. open-source references

| 项目 | 借鉴 | 不采用 |
|---|---|---|
| PythonRobotics DWA | candidate control / rollout / collision 直接淘汰 / 多项代价 / 局部窗口细化 | yaw-rate 差速模型 |
| Nav2 MPPI | trajectory batch、critic 分层、true-collision 与 near-collision 分离、fallback、向量化 | 完整 MPPI、1000+ 采样、ROS 架构 |
| CommonRoad Drivability Checker | feasibility/collision 与 utility 分离 | 完整 checker |
| F1TENTH | iTTC（距离 + 接近速度）、安全层、NaN/inf 谨慎处理 | Ackermann 模型 |
| RVO2 / ORCA | 仅 relative velocity / velocity-obstacle 几何思想 | 不实现 ORCA（障碍物不配合避让，无 reciprocal responsibility） |
| TinyMPC | fixed-size、资源受限、嵌入式友好、结构简单 | 完整 MPC solver |

> **本项目借鉴这些开源项目的结构思想，并针对 FMQL30TAI 资源受限异构平台进行了简化和重构，
> 不是直接复制这些项目的代码、运动模型、求解器或 ROS 架构。**

未引入：强化学习、神经网络、NPU、C++/Verilog/HLS、CasADi/CVXPY、完整 MPPI/MPC、
ORCA 完整实现、A*/Dijkstra、大规模随机采样、候选数量扩张。依赖仍为 Python + NumPy。

## 13. complexity

| 阶段 | 低风险 | 危险 |
|---|---|---|
| FAR 安全 | `O(B·N)` | `O(B'·N)` |
| FAR 风险剖面 | `O(B·S·N)`, S=3 | `O(B'·S·N)` |
| 轨迹 rollout | `O(B·H·N)` | `O(B'·H·N)` |
| 分层 critic + trap | `O(B)` + 每候选 `O(N)` | 同左 |

`B = 17`，危险时 `B' ≤ 25`。refinement 只细化一层，不递归。

## 14. limitations

1. `margin_time / critical_margin / lateral 阈值`为保守默认值，**未做参数搜索**；
   它们决定 LEVEL 1/2 的比例，是后续最可能需要调整的旋钮。
2. `min_margin` 的 dynamic margin 使用候选级 `max` closing_rate，属于保守上界
   （可能比逐障碍精确计算更严格）。
3. hysteresis 会在 held 为 refinement 临时动作、而该动作本帧未被重新生成时失效
   （`held_not_in_set`），此时允许切换。
4. refinement 触发率在 medium 场景约 30%，会带来约 25% 的决策耗时上升
   （低风险场景仍为 17 候选、约 0.4–1.0 ms）。
5. 本轮只做实现 + 单元测试 + ≤5 局 smoke；`side_hit`、`survival`、`corner_trap` 的最终
   结论必须由本地 1000 局 benchmark 给出，不能由 smoke 推断。
