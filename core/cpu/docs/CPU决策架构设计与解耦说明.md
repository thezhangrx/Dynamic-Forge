# CPU 决策架构设计与解耦说明

> 本文档说明 `core/cpu/` 下两条**刻意解耦**的决策路径：
> 一条面向"未来二维小车的通用算法"，一条面向"对接 FPGA 硬件字节流"。
> 两条路径**互不影响、互不调用**，各自独立演进。

---

## 1. 为什么有两条路径

| 维度 | 新架构（抽象决策） | 旧路径（硬件字节流） |
|---|---|---|
| 目标 | 未来迁移二维小车的**通用算法**（预测 / 风险 / 规划） | 对接 FPGA 硬件的**字节帧回路** |
| 输入 | 抽象状态 `SceneState`（游戏无关） | 字节帧 `BHL1 state frame` + `BHP1 prediction frame` |
| 输出 | 抽象动作 `Action(speed, steering_angle)` | 统一 `Action(direction, magnitude)` |
| 是否感知游戏坐标 | 否（由 adapter 隔离） | 否（只认字节，不认 `World`） |
| 是否依赖仿真内核 | 否 | 否（`CpuFpgaPipeline` 只读 `World.get_state()` 编码成帧） |

关键设计约束：**两条路径都不得 import `bullet_sim.simulator` 内核**，且**彼此不 import**。

---

## 2. 路径 A：新架构（抽象决策）

### 2.1 调用关系

```
                        ┌─────────────────────────────────────────────┐
                        │  cpu.adapters.game_adapter        │
                        │  （唯一的"游戏 ↔ 抽象"翻译层）                  │
                        └─────────────────────────────────────────────┘
   WorldSnapshot / observation ──► snapshot_to_scene() / observation_to_scene()
                                        │
                                        ▼
                                SceneState（抽象状态）
                                        │
                                        ▼
                 ┌──────────────────────────────────────────────┐
                 │  cpu.decision.core                │
                 │      CpuDecisionLayer.decide(state)          │
                 └──────────────────────────────────────────────┘
                                        │ 调用
                        ┌───────────────┴───────────────┐
                        ▼                               ▼
                 ActionPlanner                  (组合 predictor + risk)
                 (planner.py)                              │
                        │ 遍历候选动作                      │
                        │ 每个候选调用 predictor             │
                        ▼                               ▼
                 LinearPredictor ──► Trajectory ──► RiskEvaluator
                 (predictor.py)      (向量化)       (risk.py)
                        │                              │
                        │ 线性外推                       │ Risk = 1/(d+ε)
                        ▼                              ▼
                 ┌───────────── argmax utility = -worst_risk ─────────────┐
                 └────────────────────────────────────────────────────────┘
                                        │
                                        ▼
                          Action(speed, steering_angle)
                                        │  to_game_action()
                                        ▼
                          统一 Action(direction, magnitude)
                                        │
                                        ▼
                                   World.step()
```

### 2.2 模块职责

| 文件 | 职责 |
|---|---|
| `decision/models.py` | `AgentState` / `Obstacle` / `Action` / `SceneState`，纯 2D 抽象，**无游戏坐标** |
| `decision/predictor.py` | `Predictor` 抽象基类 + `LinearPredictor`（线性外推），输出向量化 `Trajectory` |
| `decision/risk.py` | `RiskEvaluator`：`Risk = 1 / (d + ε)`，`risk_trajectory()` 向量化评估整条轨迹 |
| `decision/planner.py` | `ActionPlanner`：候选动作空间（`speed + steering_angle`）→ argmax 最安全动作 |
| `decision/core.py` | `CpuDecisionLayer.decide(state)` 门面，内部就是 `ActionPlanner.plan()` |
| `adapters/game_adapter.py` | `WorldSnapshot`/observation ↔ `SceneState`、`Action` ↔ 统一 `Action` |

### 2.3 关键点

- **游戏无关**：`models/predictor/risk/planner/core` 不 import 任何游戏 / 仿真模块。
- **第一版只用线性模型**：`LinearPredictor` 匀速外推，**无卡尔曼滤波、无神经网络**。
- **未来迁移**：二维小车的传感器数据只要翻译成 `SceneState`（一个 adapter），即可复用整套决策算法。

---

## 3. 路径 B：旧路径（硬件字节流）

### 3.1 调用关系

```
   World ── get_state() ──► WorldSnapshot
                                │
                                │ encode_state()          (bullet_sim.interface.protocol)
                                ▼
                       BHL1 state frame (bytes)
                                │  CpuFpgaPipeline.iterate()
                                ▼
                  FpgaPredictionReference.predict(frame)   (bullet_sim.fpga.prediction_block)
                                │
                                ▼
                       BHP1 prediction frame (bytes)
                                │
                                ▼
                  FrameCpuDecisionLayer.decide(state_frame, prediction_frame)
                                │  （读字节 + 风险栅格，两阶段安全优先规则）
                                ▼
                       统一 Action(direction, magnitude)
                                │  action_to_codec_input()
                                ▼
                            World.step()
```

### 3.2 模块职责

| 文件 | 职责 |
|---|---|
| `pipeline.py` | `PipelineLatency`、`FrameCpuDecisionLayer`（帧/栅格决策）、`CpuFpgaPipeline`（回路）、`CpuFpgaController`（ActionProvider 适配） |
| `interface/protocol.py` | `BHL1` state 帧（v3，CRC32，固定 stride） |
| `fpga/prediction_block.py` | `BHP1` prediction 帧 + `FpgaPredictionReference`（Python golden model） |

### 3.3 关键点

- **只认字节**：`FrameCpuDecisionLayer` / `CpuFpgaPipeline` 从不 import `World` / `simulator`。
- **FPGA 语义**：将来把 `FpgaPredictionReference` 换成真实 RTL，只要预测帧逐字节一致，上层零改动。
- **传输层未定**：CPU↔FPGA 总线仍是 `[HARDWARE_INTERFACE_TBD]`（见 `hardware_interface/tbd.py`），本路径只固定字节布局。

---

## 4. 两条路径的解耦边界（硬约束）

```
                         ┌───────────────────────────────────────┐
                         │           统一 Action                 │
                         │   Action(direction, magnitude)        │
                         └───────────────▲───────────────────────┘
                                         │
        ┌────────────────────────────────┴────────────────────────────────┐
        │                                                                  │
  路径 A（抽象决策）                                              路径 B（硬件字节流）
  SceneState → CpuDecisionLayer → Action                         BHL1帧 → FPGA → BHP1帧
  （预测/风险/规划，面向二维小车）                                  → FrameCpuDecisionLayer → Action
        │                                                                  │
        └───────────────────── 互不 import、互不影响 ──────────────────────┘
```

**规则**：

1. `decision/`（新）与 `pipeline.py`（旧）**互不 import**。
2. 两者都不得 import `bullet_sim.simulator`（有测试断言）。
3. 唯一允许接触游戏/仿真结构的是 `adapters/game_adapter.py`（路径 A 的翻译层）。
4. 两条路径最终都产出统一 `Action`，接入 `World.step()`，但**决策逻辑完全独立**。

---

## 5. 文件总览

```
core/cpu/
├── decision/                    # 路径 A：抽象决策架构
│   ├── models.py                #   AgentState / Obstacle / Action / SceneState
│   ├── predictor.py             #   Predictor + LinearPredictor + Trajectory
│   ├── risk.py                  #   RiskEvaluator (1/(d+ε))
│   ├── planner.py               #   ActionPlanner (argmax)
│   ├── core.py                  #   CpuDecisionLayer.decide(state)
│   └── __init__.py
├── adapters/                    # 路径 A 的翻译层
│   ├── game_adapter.py          #   WorldSnapshot/observation <-> SceneState
│   └── __init__.py
├── pipeline.py                  # 路径 B：帧/栅格决策 + CPU↔FPGA 回路
└── __init__.py                  # 顶层导出：CpuDecisionLayer(新) + 旧路径类
```

> 注意：`CpuDecisionLayer` 现在指**新架构**的抽象决策层（`decision/core.py`）；
> 旧的帧/栅格决策类已更名为 **`FrameCpuDecisionLayer`**（`pipeline.py`），
> 避免同名混淆，也明确"这是硬件字节流路径"。

---

## 6. 变更历史

| 日期 | 变更 |
|---|---|
| 当前 | 从 `cpu/decision.py` 拆分为 `cpu/decision/`（新抽象架构）与 `cpu/pipeline.py`（旧硬件回路）；旧类 `CpuDecisionLayer` 更名 `FrameCpuDecisionLayer`；新增 `cpu/adapters/game_adapter.py` |
