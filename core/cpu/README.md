# `core/cpu/` —— CPU 决策层

CPU 侧的决策代码：**从"看见障碍"到"给出速度指令"的全部算法**，以及 CPU↔FPGA
帧回路。平台侧（仿真器、渲染、数据集）在 `core/bullet_sim/`，图像处理在
`core/vision/`，本目录只依赖前者的类型定义，不依赖后者的任何实现。

> 这个目录是本次交付的全部内容。整个 `core/cpu/` 可以独立搬走：只要有
> `core/bullet_sim/` 的 `Action` / `Observation` 类型，它就能跑。

---

## 1. 一眼看懂：先跑起来

```bash
# 0) 依赖：NumPy 是唯一硬依赖；pygame 只有可视化需要
python -m pip install -e ".[dev]"        # 或者直接用仓库的 .venv

# 1) 算法自检（不需要 pygame、不需要仿真器，4 条断言）
python cpu_run_demo.py selftest

# 2) 缺口墙避障可视化：车朝静止的墙开过去，缺口会动，跑 5 局
python cpu_run_demo.py                    # = cpu_run_demo.py gap
python cpu_run_demo.py gap --headless --episodes 5     # 不开窗口，只打印统计

# 3) 版本与真实源码指纹
python cpu_run_demo.py version

# 4) 本模块的测试（223 项）
python -m pytest core/cpu/tests -q

# 5) 决策基准：纯反应式 baseline vs 预测式（1000 局，约 6 分钟）
python cpu_run_demo.py bench
```

`cpu_run_demo.py` 是仓库约定的根目录入口（前缀 = `core/` 下的模块名），只做参数
转发，不含算法。

---

## 2. 两条交付物，不要混在一起看

| | 交付物 | 入口 | 依赖 | 用途 |
|---|---|---|---|---|
| **A** | **缺口墙避障算法** | `gap_avoid.py` 的 `plan()` | **零依赖**（纯 Python，无 numpy） | 要塞进 FPGA / 队友工程的那份；算子可逐条翻成 Verilog |
| **B** | 通用决策架构 + CPU/FPGA 回路 | `decision/`、`pipeline.py` | numpy | 平台上的自主决策层与硬件边界 |

### A. `plan()` —— 唯一需要移植的算法

```python
from cpu.gap_avoid import (
    WorldView, GapMemory, AlgoConfig, Rect, plan, to_command,
)

view = WorldView(
    player=[x, y, vx, vy, radius, max_speed, alive],   # 与 entities/player.py 同序
    rects=[Rect(ent_id, x, y, vx, vy, half_w, half_h, rotation, angular_velocity), ...],
    dt=1/120, step_index=0, field_w=640.0, field_h=480.0,
)
mem = GapMemory()                       # 跨帧记忆：必须每帧传同一个对象
cfg = AlgoConfig(forward=(0.0, 1.0),    # 固定行驶轴（车一直朝 +y）
                 front_back=True,       # 允许前后移动（含停住）
                 max_turn_rate=720.0)   # 指令方向最大转向角速度（度/秒）

dec = plan(view, mem, cfg, horizon_remaining=2.0)   # Decision
wx, wy = to_command(dec)                            # 归一化速度，模长<=1
```

**接口契约（接视觉模块时最要紧的三条）**

1. `rects` 用**场地坐标**，`x ∈ [0, field_w]`、`y ∈ [0, field_h]`。
   算法会把重建出的孔径按 `[0, field_w]` 裁剪 —— 墙有一半在场地外就会算出偏小的
   可用孔径，所以视觉模块给的坐标必须是场内坐标（不是相机像素坐标）。
2. `plan()` **只读 `view.rects` 和 `view.player`**，不碰仿真器。所以视觉模块只要
   能把"识别出的墙"变成 `Rect` 列表，就能直接用这个算法。
3. `GapMemory` 必须**跨帧复用**同一个对象：缺口宽度、缺口中心方位的记忆都在里面，
   单段可见（实测约 4% 的帧）时靠它补全缺口。

**输出** `Decision` 除 `wx/wy/tier` 外还带全套诊断量，可视化与调参都靠它们：
`gap`（重建的缺口宽）、`usable_half`（可用半孔径 = `gap/2 − r_p − margin`）、
`e_lo/e_hi`（缺口两条内缘）、`aim_lat`、`s_hat`（逼近速度）、`t_arr`（到达时间）、
`gap_line`、`aim_world`、`wall_band`、`n_rects`、`reason`。

**用到的算法**（每个都在代码里有注释说明为什么）：
支撑函数（旋转矩形沿任意方向的精确半投影）、矩形 SDF + 解析梯度、
哈希分桶把矩形组织成"墙"、最近邻数据关联（跨帧配墙）、
一阶最小二乘拟合缺口运动 + 残差包络（区间界）、匀速前向外推、
**拦截时间求解** `|s(T) − lat| = v_max·T`（不动点迭代 4 次）、
分层策略 + 死区（防抖）、全向速度预算 `u = √(v² − w²)`、
到达时间可达性、配置空间膨胀、字典序目标选择（**绝不做加权求和**）、
转向速率限幅、滚动时域重规划、带种子 RNG（可复现）。

### B. `decision/` —— 通用决策架构

```python
from cpu.decision import CpuDecisionLayer, ViabilityConfig
from cpu.adapters import observation_to_scene, to_game_action

layer = CpuDecisionLayer()
scene = observation_to_scene(observation)   # Obs -> 抽象场景
action = layer.decide(scene)                # -> Action
```

| 文件 | 职责 |
|---|---|
| `decision/models.py` | 抽象场景/候选动作的数据模型（与游戏解耦） |
| `decision/predictor.py` | 候选动作的线性外推 |
| `decision/risk.py` | 安全/风险判定与自适应包络 |
| `decision/far.py` | 前向可达（Forward Accessibility Region） |
| `decision/immediate.py` | 近期避让层 |
| `decision/reactive.py` | 纯反应式路径（基线，v0.5.0 失败实验的产物） |
| `decision/viability.py` | 长时域可行性（LTV，v0.5.2 起降级为**带门控的平局裁决**） |
| `decision/future_corridor.py` | 未来安全走廊（FSC 影子模式） |
| `decision/corridor_selector.py` | 预测式走廊选择（v0.6.0，默认关） |
| `decision/planner.py` | 分档 argmin + 迟滞 + override guard |
| `decision/core.py` | `CpuDecisionLayer`：把上面几层串起来 |
| `decision/build.py` | 版本号与**真实源码指纹** `BUILD_ID` |
| `pipeline.py` | 字节帧的 CPU↔FPGA 回路（`CpuFpgaPipeline` / `FrameCpuDecisionLayer`） |
| `adapters/game_adapter.py` | `Observation ↔ 抽象场景`、`决策 ↔ Action` 的双向映射 |
| `benchmark_decision.py` | 离线基准（反应式 vs 预测式，同一批 1000 局） |
| `tools/` | 离线诊断：FSC 影子探针、LTV 回归诊断 |
| `docs/` | 各版本的设计与实测记录（**含失败实验的诚实记录**） |

**设计红线**（`decision/corridor_selector.py` 开头写死了）：决策顺序是
**字典序，绝不是一个大的加权和**；"一般偏好"永远不能压过碰撞安全。
这条约束同时也是给 FPGA 的：映射下来只有比较 / 取最小 / 取最大 / 位运算 / popcount。

---

## 3. 与 `core/bullet_sim/cpu/` 的关系（重要）

新结构里 `core/bullet_sim/cpu/` 还留着一份**旧的单文件版**
（`decision.py`，333 行，导出 `CpuDecisionLayer` / `CpuFpgaPipeline`）。
本目录是它的**重构版 + 扩展版**，是同一个东西的后续：

| | `core/bullet_sim/cpu/`（旧） | `core/cpu/`（本目录） |
|---|---|---|
| 结构 | `decision.py` 单文件 | `decision/` 包 + `pipeline.py` + `adapters/` |
| 导入名 | `bullet_sim.cpu` | `cpu` |
| 导出 | `CpuDecisionLayer` `CpuFpgaPipeline` `CpuFpgaController` `PipelineLatency` | 同上 **+** `FrameCpuDecisionLayer`，另外还有 `gap_avoid` / `benchmark_decision` / `tools` |
| 指纹 | 无 | `BUILD_ID` = 对 14 个源文件做 sha256（`cpudec-xxxxxxxxxxxx`） |

本次改动**没有删除、没有修改** `core/bullet_sim/cpu/` 的任何内容。两份可以共存；
如果要收敛成一份，建议保留本目录、把 `core/bullet_sim/cpu/` 变成一个
re-export shim（`from cpu import *`），但**这是你们的决定**，我没有替你们做。

---

## 4. 本次搬迁做了什么（逐条可核对）

从旧布局搬到 `core/cpu/`，除"路径"外**算法一行没改**：

1. **导入名**：`bullet_sim.cpu.*` → `cpu.*`（97 处，27 个文件）。
   平台侧导入保持不动：`bullet_sim.action.types` / `bullet_sim.core.actions` /
   `bullet_sim.simulator.env` 等仍然有效。
2. **路径引导**：这些文件原来假设自己在仓库根或 `bullet_sim/` 下，现在按
   `core/cpu/` 重算（`benchmark_decision.py`、`tools/*.py`、`conftest.py`）。
3. **产物落点**：按约定集中到 `data/cpu/`（`benchmark_decision.py` 里的
   `DATA_DIR`）；`docs/` 留在 `core/cpu/docs/`。`data/` 已被 `.gitignore` 排除。
4. **`BUILD_ID` 的指纹路径表** `FINGERPRINT_FILES`：`bullet_sim/cpu/...` →
   `cpu/...`，`bullet_sim/ai/reactive_gap.py` → `cpu/baseline_reactive_gap.py`。
   因为源码字节变了，`BUILD_ID` 从 `cpudec-f7be1d81c473` 变成
   **`cpudec-f44a7bfdef51`**（这是真指纹，不是写死的版本字符串）。
5. **`baseline_reactive_gap.py`**：`ReactiveGapController` 在新结构的
   `core/bullet_sim/ai/` 里已被精简掉，但它是 `decision/reactive.py` 的出处、
   也是 A/B 测试的对照组，所以**作为本模块的基线副本**一起发布（不改你们的 `ai/`）。
6. **测试**：`bullet_sim/tests/test_decision_*.py` + `test_cpu_fpga_pipeline.py`
   → `core/cpu/tests/`。里面的硬编码路径断言同步改到新结构
   （`test_decision_{architecture,future_corridor}.py` 的 `parents[1]/"cpu"/"decision"`
   → `parents[1]/"decision"`；`test_decision_ltv_diagnostics.py` 的指纹路径）。

### 验证（全部在新结构下实测）

| 检查 | 结果 |
|---|---|
| `python -m pytest core/cpu/tests -q` | **223 passed** |
| `python cpu_run_demo.py selftest` | 4/4 通过 |
| `python cpu_run_demo.py gap --headless --episodes 5` | 5 局 **0 碰撞**，每局 **3/3 堵墙**全过 |
| `python -m compileall core/cpu` | 全部通过 |

---

## 5. 两个已知问题（不是本次搬迁引入的，请你们决定）

### 5.1 `core/bullet_sim/tests/test_determinism.py` 在 `origin/main` 上就失败

```
FAILED core/bullet_sim/tests/test_determinism.py::test_reproducibility_survives_a_fresh_interpreter
```

原因：该测试会 `subprocess` 起一个**全新解释器**去 `import bullet_sim`，但包现在在
`core/` 下，子进程的 `sys.path` 里没有 `core/`，于是 `ModuleNotFoundError`。
**这是重构后的路径问题，与 `core/cpu/` 无关** —— 我在 `46e60ca` 的干净工作树上复现了
一模一样的失败。临时的绕过办法：

```bash
PYTHONPATH=core python -m pytest core/bullet_sim/tests -q     # 全绿
```

我没动你们的测试文件。要长期修，最小改动是在那个测试的 `subprocess.run(...)` 里
补上 `env={**os.environ, "PYTHONPATH": str(pathlib.Path(__file__).resolve().parents[2])}`
（或给项目加一个根 `conftest.py` 把 `core/` 放进 `sys.path`）。

### 5.2 `pyproject.toml` 的 `include` 与它自己的注释不一致

第 37-38 行注释说"源码都在 `core/` 下（`core/bullet_sim`, `core/cpu`,
`core/fpga`, `core/vision`）"，但 `include` 原本只有 `["bullet_sim*"]`，
`cpu` / `fpga` / `vision` 都不在打包范围内。我按需**只加了 `cpu*`**：

```toml
include = ["bullet_sim*", "cpu*"]
```

`fpga*` / `vision*` 要不要加是你们的事（`vision` 现在靠根入口
`vision_detect.py` 自己插 `sys.path`，不装也能跑），我**没有替你们动**。

---

## 6. 环境与运行前提

* Python ≥ 3.10；实测 3.14 + NumPy 2.5.3 + pytest 9.1.1（仓库 `.venv`）。
* 裸源码检出（没 `pip install`）时，`core/cpu/conftest.py` 会把 `core/` 与
  `core/cpu/` 放进 `sys.path`，所以 `python -m pytest core/cpu/tests` 直接能跑。
* 没有 `pygame` 时：`selftest` / `bench` / `version` / `paths` / `test` 都照常，
  只有 `gap`（可视化）会报缺 pygame。`gap_avoid.py` 本身**连 numpy 都不需要**。
* `gap_avoid.py` 可以单独当脚本跑自检：`python core/cpu/gap_avoid.py`。

---

## 7. 许可

本目录的代码是仓库自身的工作成果，按仓库整体许可（MIT，见根目录
`pyproject.toml` 的 `license`）发布。`decision/` 里各文件头部注明的方法出处
（F1TENTH Follow-the-Gap、CommonRoad-Reach、Nav2 MPPI 等）见
`core/bullet_sim/docs/OPEN_SOURCE_REFERENCES.md`；参考实现中
**GPL-3.0 的项目（如 `pred-occ-planner`）不得逐行抄入**，本目录没有抄。
