# Dynamic-Forge v0.5.0-DualPlanner 设计说明

> 本轮同时升级 Reactive 与 Predictive 两条决策路径，并**严格保持二者在算法定义上的边界**。

- Stack Version: `v0.5.0-DualPlanner`
- Reactive: `v0.5.0-ReactiveGap`（单帧、8 扇区、无 rollout）
- Predictive: `v0.5.0-PredictiveASE`（FAR + Adaptive Safety Envelope，保留 v0.4.1 全部机制）

---

## 1. 为什么是"双规划器"

两者回答的问题本质不同，不能混为一谈：

| | ReactiveGap | PredictiveASE |
|---|---|---|
| 时间模型 | **仅当前帧** | 候选 × 未来采样 |
| 未来轨迹 | 无 | 有（`LinearPredictor` + FAR） |
| FAR / FES / horizon | **禁止** | 保留并强化 |
| 每帧计算 | `O(N)` + `O(8)` | `O(B·H·N)` + `O(B·S·N)` |
| 定位 | 可部署、便宜、强反应基线 | 预测式规划器 |

共享的只有**底层几何**，不共享未来模型（见第 4 节）。

## 2. 共享底层几何：`cpu/decision/immediate.py`

一套当前帧几何原语，供两条路径复用：

```
r        = p_obstacle − p_agent
v_rel    = v_obstacle − v_agent
closing_rate = −dot(r, v_rel) / max(|r|, ε)      # >0 表示接近
clearance    = |r| − (r_agent + r_obstacle)
sector       = argmax_s dot(unit(r), dir_s)      # 8 方向，无 atan2
sectorize(r, v_rel, radius_sum) -> SectorSummary(nearest_clearance[8],
                                                 closing_rate[8],
                                                 density[8])
gap_scores(clearance, w) = c[d] + w·(c[d−1] + c[d+1])
```

Reactive 只用这些量的**当前帧**；Predictive 在这些量之上展开 candidate × future sample。

## 3. ReactiveGap 架构（`cpu/decision/reactive.py` + `ai/reactive_gap.py`）

```
Current Scene
   ↓ 8 direction sectors
   ↓ immediate clearance / closing rate
   ↓ immediate_margin = clearance − (base_buffer + closing_rate·reaction_time)
   ↓ LEVEL 0 normal / LEVEL 1 critical / LEVEL 2 unsafe
   ↓ gap score + goal alignment + side balance + potential field
   ↓ safety-first hysteresis
   ↓ threat-aware speed（STOP 须通过自身安全校验）
Action
```

- **8 扇区模型**：`nearest_clearance / obstacle_density / directional_threat / closing_rate / gap_width` 全部只由当前帧障碍得到。
- **Immediate TTC / closing rate**：`closing_rate = −dot(r,v_rel)/|r|`；`closing_rate <= 0` 不产生额外惩罚。
- **Immediate safety envelope**：`dynamic_buffer = base_buffer + closing_rate·reaction_time`；`margin < 0 → unsafe`，`0 ≤ margin < critical_margin → critical`，否则 normal。
- **Gap**：`gap_score[d] = clearance[d] + 0.5·(clearance[d−1] + clearance[d+1])`（F1TENTH Follow-the-Gap 的离散 8 向版本）。
- **Goal-aware**：`goal_alignment = (1 + cos(θ_goal − θ_d))/2`，只在安全之后起决定作用（位置制权重）。
- **Potential field 项**：把「斥力 + 目标吸引」向量投影到 8 个扇区（PythonRobotics potential-field 思想）。这一项是防止纯 gap 选择锁定在"空但无用"的死方向（例如场地边缘）的关键。
- **Anti-pinning guard**：若上一帧命令了移动方向而实际速度≈0（被墙/边界挡住），该方向本帧视为不可用。这是**反应式**的（只看当前速度 + 上一动作），不是预测。
- **Action hysteresis（安全优先）**：held 安全且优势边际、且目标对齐没有明显变差时保持；held unsafe/critical/被挡则立即切换。目标追求永不被滞回压制。
- **Threat-aware speed**：`speed_scale = clip(margin/speed_full_margin, 0, 1)`；低威胁允许 FAST，高威胁考虑减速/停止；**STOP 必须通过当前帧安全校验**（`stop_margin ≥ stop_safety_margin`），避免"停下后被撞"。

**Reactive 明确禁止**：future trajectory、rollout、FAR、FES、risk_near/mid/far、adaptive horizon、candidate-conditioned future prediction。测试 `test_K_reactive_contains_no_future_machinery` 用 AST 扫描该模块，确保这些标识符不出现。

## 4. PredictiveASE

完全保留 v0.4.1：FAR 风险剖面（risk_near/mid/far/peak/trend）、candidate-conditioned relative velocity、`t_ca`/`d_ca`、candidate `closing_rate`、`safety_margin`、critical risk、SafetyGate、FES/trap、dwell、adaptive horizon、adaptive smooth、17 基础候选、≤8 高风险局部细化、安全优先滞回、EmergencyFallback。本轮**未改写**其算法，只更新版本标记为 `v0.5.0-PredictiveASE`。

## 5. 边界（benchmark 公平性）

两者使用完全相同的 seeds / scenes / obstacle generation / episode 长度 / 碰撞定义。

- Reactive 允许：当前 x/y、vx/vy、障碍当前 x/y/vx/vy、当前 distance、当前 closing rate、当前 immediate TTC、当前 directional clearance。
- Reactive 禁止：candidate-conditioned future prediction、future risk、FES、FAR、multi-step rollout、risk_near/mid/far、adaptive future horizon。
- 只有 Predictive 允许：candidate-conditioned future state、future risk、FES、horizon、future trajectory。

两者都是 `observation` 访问、无 world 访问。

## 6. 版本与 BUILD_ID

`build.py` 是唯一版本来源：

```
STACK_VERSION      = "v0.5.0-DualPlanner"
REACTIVE_VERSION   = "v0.5.0-ReactiveGap"
PREDICTIVE_VERSION = "v0.5.0-PredictiveASE"
VERSION            = PREDICTIVE_VERSION     # 兼容别名
BUILD_ID           = "cpudec-" + sha256(源码字节)[:12]
```

指纹覆盖（repo 相对路径）：`models/predictor/risk/cost/far/immediate/reactive/planner/core/build.py` 与 `ai/reactive_gap.py`。任意源码改变 → BUILD_ID 改变。

## 7. FPGA mapping

Reactive 侧只需简单的当前帧向量：`sector_clearance[8]`、`sector_density[8]`、`sector_closing[8]`、`emergency_flag`；CPU 负责 gap 选择、goal 对齐、滞回、动作。Predictive 侧的 `candidate × obstacle` / `candidate × sample × obstacle` 内核（≤25 候选）由 CPU 决定，未来可搬 FPGA。

主要运算：add / multiply / compare / min / max / one sqrt / guarded divide；无 atan2、无动态对象、无递归。

## 8. CPU role

两条路径的**最终动作选择都在 CPU**。FPGA/底层未来最多并行产生定长特征，不替 CPU 选动作。

## 9. Open-source references

| 项目 | 借鉴 | 不采用 |
|---|---|---|
| F1TENTH Follow-the-Gap | 最近障碍 bubble → free gap → 方向；8 向离散化 + neighbor 连续空闲度 | 原始激光扫描 / Ackermann |
| F1TENTH iTTC | 当前 distance + closing speed 作即时安全层 | 未来 rollout |
| PythonRobotics Potential Field | 障碍斥力 + 目标吸引 + 局部方向选择 | 全局 potential map、大网格、复杂 reciprocal distance |
| Nav2 MPPI | critic 分层、obstacle/critical/behavior/goal 分离、抗 wobble 死区思想、fallback | 完整 MPPI / ROS 架构 |
| PythonRobotics DWA | candidate/action scoring 结构思想 | 差速运动模型；Reactive 禁止 rollout |
| CommonRoad Drivability Checker | feasibility 与 utility 分离 | 完整 checker |
| TinyMPC | fixed-size、资源受限、嵌入式友好 | 完整 MPC solver |

> **本项目借鉴这些开源项目的结构思想，并针对 FMQL30TAI 资源受限异构平台进行了简化和重构，
> 不是直接复制这些项目的代码、运动模型、求解器或 ROS 架构。**

未引入：强化学习、神经网络、NPU、C++/Verilog/HLS、CasADi/CVXPY、完整 MPPI/MPC、ORCA 完整实现、A*/Dijkstra、大规模随机采样；依赖仍为 Python + NumPy。

## 10. Complexity

| | ReactiveGap | PredictiveASE |
|---|---|---|
| 几何 | `O(N)` | `O(B·N)`（FAR 安全） |
| 风险 | 无 | `O(B·S·N)`, S=3 |
| rollout | 无 | `O(B·H·N)` |
| 打分 | `O(8)` | `O(B)` + 每候选 `O(N)` |

## 11. 本轮 smoke（2 seeds，非结论）

| 控制器 | survival | changes | failures |
|---|---|---|---|
| legacy `RepulsionController` | 877.5 | 635.5 | front_hit 2 |
| **ReactiveGap** | 736.0 | 553.5 | side_hit 2 |
| **PredictiveASE** | 825.5 | 98.0 | side_hit 2 |

ReactiveGap 的决策耗时约 0.4–0.5 ms，PredictiveASE 约 1.25 ms。

## 12. Limitations

1. ReactiveGap 在 2 局 smoke 上略低于 legacy `RepulsionController`（736 vs 877）；这是**小样本**，最终以本地 1000 局为准。为避免"改 benchmark 让 Reactive 更好/更难"，场景与种子完全未改，legacy `RepulsionController` 仍保留在 `BASELINE_CONTROLLERS`（`--ai repulsion`）可直接对照。
2. ReactiveGap 的动作变化数仍偏高（约 550/局）；滞回参数 `switch_score_margin`/`goal_switch_threshold` 未做大规模搜索。
3. 场地边界不是障碍物，ReactiveGap 通过 anti-pinning guard 间接处理；这是反应式近似，不是精确的边界几何。
4. 8 扇区把连续方向离散化，扇区边界存在并列，靠 tier 权重与索引确定性地打破。
