# 决策基准测试结果：RepulsionController vs PredictiveASE (v0.5.3-FutureGapTieBreak)

> Stack Version:      `v0.5.3-FutureGapTieBreak`
> Reactive baseline:  `RepulsionController`（参考基线，保持不变）
> Predictive Version: `v0.5.3-FutureGapTieBreak`
> Build ID:           `cpudec-24db8f7ca606`
> Random Seed:        `42`
> Episode Count:      `1000`

> 注：`v0.5.0-ReactiveGap`（Follow-the-Gap 实验）已归档保留，但**不参与本基准**
> （实测 collision 94.5%）。本表始终对照原始 `RepulsionController`。

> 本文件由 `benchmark_decision.py` 自动生成。复现命令：
> `.venv/bin/python benchmark_decision.py`

## 1. 实验配置

| 项 | 值 |
|---|---|
| 随机种子 | `42`（1000 局场景由 `42 + i` 确定性派生，两控制器共用同一批场景） |
| 场景 | `medium`（`scenario_for_level`），时长 20.0s，`terminate_on_collision=True` |
| 生存上限 | 1200 步（10 s @ 120 Hz） |
| 预测 horizon | 0.3s（仅预测式决策使用；线性外推 lookahead） |
| 局数 | 1000 |

## 2. 对比对象

| 策略 | 实现 | 说明 |
|---|---|---|
| **纯反应式 baseline** | `bullet_sim.ai.baseline.RepulsionController` | 只看**当前帧**附近弹幕密度，沿反方向排斥 + 向目标吸引；O(N)，observation-only。`v0.5.0-ReactiveGap` 实验已归档，不参与本基准 |
| **PredictiveASE (v0.5.3-FutureGapTieBreak)** | `cpu.decision.CpuDecisionLayer` | **v0.4.1 FAR + ASE 全部保留**（候选条件化 `v_rel`、`t_ca`/`d_ca`、`closing_rate`、`min_margin` LEVEL 0/1/2、`lateral_critical_risk`、≤8 局部细化、SafetyGate、FES、dwell、自适应 horizon/smooth、EmergencyFallback），并新增 **v0.5.1 Long-Term Viability**：Top-K=4 短名单（3 短期 leader + 1 个不偷看 viability 的 diversity）→ 3 个 anchor（scene scale 标定，≤0.8s）上用 **future agent × future obstacle** 计算 8 扇区 mobility（`safe_sector_count / max_continuous_safe_sector / corridor_min_clearance`，`mobility∈[0,1]`）→ `viability=min(t1,t2,t3)` → 分档仲裁（critical band > risk band > viability > FES > behaviour > task，`risk_band_width=0.05`）→ corridor hysteresis + action hysteresis。低威胁 5 条件全满足时跳过 LTV（复用 v0.4.1 排序）。**v0.5.2-LTVGuarded** 保留以上全部机制，但把 LTV 降级为**顾问式**：LTV 的提案只有在 *critical band 不劣、risk band 完全相同、viability 增益 ≥ 0.10、且未 long-term degraded* 时才被采纳，否则回退 v0.4.1 short-term best（`short-term safety first, LTV advisory second`）。|

## 3. 结果

| 指标 | 纯反应式 baseline | PredictiveASE |
|---|---|---|
| 生存时间 (步, 均值±标准差) | 1004.2 ± 251.4 | 961.1 ± 248.6 |
| 碰撞率 (1000 局) | 50.1% (501/1000) | 62.4% (624/1000) |
| 决策耗时 (均值±标准差) | 23.2 ± 7.2 µs | 1908.1 ± 540.8 µs |
| 动作平滑度 (动作变化次数, 均值±标准差) | 716.0 ± 186.0 (总 715963 次 / 1004662 步) | 150.6 ± 70.0 (总 150581 次 / 961748 步) |

### 失败类型统计

| 失败原因 | 纯反应式 baseline | PredictiveASE |
|---|---|---|
| front_hit | 169 | 170 |
| side_hit | 323 | 403 |
| corner_trap | 9 | 51 |
| other | 0 | 0 |

### LTV 可观测性（本轮只做观测；override↔碰撞 仅为统计关联，不表示因果）

| 指标 | 值 |
|---|---|
| ENABLE_LTV | `enabled` |
| LTV 决策步数 | 961748 |
| LTV 实际评估 (eval) | 852991 (88.7%) |
| LTV 跳过 (skip) | 80766 (8.4%) |
| **ltv_override 步数** | **6444 (0.7%)** |
| override 发生的 episode 数 | 903 |
| corridor switch 总数 / 每局均值 | 2863 / 2.86 |
| corridor hysteresis hold | 1044 |
| corridor hard-unsafe 立即切换 | 2583 |
| emergency 步数 | 27991 |
| override 后发生碰撞的 episode 数 | 537 |
| ↳ front_hit / side_hit / corner_trap / other | 134 / 356 / 47 / 0 |

### v0.5.2 LTV override guard（short-term safety first；override↔碰撞 仅为统计关联）

| 指标 | 值 |
|---|---|
| LTV shadow 模式 | `off` |
| override guard | `ON`（阈值 viability gain ≥ 0.10） |
| LTV 想改而未被 guard 放行的步数 (proposed_override) | 328340 (34.1%) |
| **实际放行的 ltv_override 步数** | **6444 (0.7%)** |
| guard 否决步数 (veto) | 321896 (33.5%) |
| guard 考察的候选总数 | 2041041（候选级拦截率 99.5%） |
| ↳ blocked by risk band（跨风险档） | 1401314 |
| ↳ blocked by viability margin（优势不足） | 150108 |
| ↳ blocked by long-term degraded | 295981 |
| ↳ blocked by critical band | 182661 |
| ↳ blocked by no LTV reference | 138445 |

### v0.5.3 Future Gap Tie-Break（安全决策之后的 preference；开/关由 `DF_ENABLE_FSC_FUTURE_GAP` 控制）

| 指标 | 值 |
|---|---|
| Future Gap Tie-Break | `ON` |
| 启用帧数 | 961748 / 961748 |
| gap tie-break attempted | 396912 (41.27%) |
| gap tie-break candidate_count（通过全部安全条件的候选数） | 1661 |
| **gap tie-break accepted** | **1022 (0.11%)** |
| ↳ 平均 t3 max_run 增益 (sector) | 2.89 |
| rejected_by_risk（现有 risk/critical 档更差） | 352200 |
| rejected_by_gain（增益 < 2 sector） | 39892 |
| rejected_by_t3（t3 corridor 不可用） | 0 |
| rejected_by_emergency | 27991 |
| rejected_by_ltv（本帧已 LTV override） | 6324 |
| rejected_by_stop | 378034 |
| rejected_by_source（非真实 Top-K） | 152487 |
| rejected_by_tie（t3 宽度并列） | 212 |
| rejected_no_candidate | 3586 |

## 4. 结论

- （结果由脚本实测填写，见上表。）
- 纯反应式 baseline 决策耗时只含「当前帧 O(N) 排斥 + 目标吸引」；PredictiveASE 见下。
- PredictiveASE 决策耗时包含「观测 → 抽象场景 → 17（危险时 ≤25）条候选 × horizon 步线性外推 → FAR 安全/风险 + 自适应安全包络 → v0.5.2 guarded LTV（仅 K≤4 短名单 × 3 anchor）→ 分档 argmin → override guard → corridor/action hysteresis」的完整链路；内部另记 `short_term_us / viability_us / total_us`。
- 失败类型为启发式分类：`corner_trap` = 碰撞时处于场地角落区（15%×短边）；`front_hit` = 撞击方向与航向夹角 ≤ 60°；`side_hit` = 夹角更大；`other` = 静止或无有效撞击方向。

_生成时间：由脚本运行生成；配置见 `benchmark_decision.py`。_
