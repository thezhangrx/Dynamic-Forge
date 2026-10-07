# `core/cpu/` —— CPU 决策层

CPU 侧的两件事：**从"看见障碍"到"给出速度指令"的避障算法**，以及
**CPU↔FPGA 帧回路**。平台侧（仿真器、渲染、数据集）在 `core/bullet_sim/`，
图像处理在 `core/vision/`，本目录只依赖前者的类型定义，不依赖后者的任何实现。

> 这个目录是本次交付的全部内容。整个 `core/cpu/` 可以独立搬走：只要有
> `core/bullet_sim/` 的 `Action` / `Observation` 类型，它就能跑。

---

## 1. 一眼看懂：先跑起来

```bash
# 0) 依赖：NumPy 是唯一硬依赖；pygame 只有可视化需要
python -m pip install -e ".[dev]"

# 1) 根入口（README 承诺的那个，已就位）
python cpu_run_demo.py selftest            # gap_avoid 零依赖自检，不需要 pygame
python cpu_run_demo.py gap                 # 缺口墙避障演示（incoming 场景）
python cpu_run_demo.py gap --scene drive --episodes 5   # 连过三堵墙
python cpu_run_demo.py gap --headless -e 5 # 不开窗口，只打印统计
python cpu_run_demo.py paths               # 排查环境：打印解析到的模块路径

# 2) 也可以直接跑模块内脚本（根入口只是转发）
python core/cpu/gap_avoid.py
python core/cpu/gap_wall_demo.py --scene incoming --episodes 5

# 3) 本模块的测试
python -m pytest core/cpu/tests -q

# 4) 整条闭环（渲染 → 真实视觉 → 决策 → 动作）
python cpu_vision_loop.py --synthetic --episodes 5 --seconds 16
```

> 根入口 `cpu_run_demo.py` 的 `version` / `bench` 两个子命令**已删除**：
> 它们依赖已移除的 `cpu.decision.build`（BUILD_ID）与
> `cpu.benchmark_decision`，见 §4。

---

## 2. 两条交付物，不要混在一起看

| | 交付物 | 入口 | 依赖 | 用途 |
|---|---|---|---|---|
| **A** | **缺口墙避障算法** | `gap_avoid.py` 的 `plan()` | **零依赖**（纯 Python，无 numpy） | 要塞进 FPGA / 队友工程的那份；算子可逐条翻成 Verilog |
| **B** | **CPU↔FPGA 帧回路** | `pipeline.py` | numpy + `bullet_sim` 的帧协议 | 硬件边界：字节帧进、`Action` 出 |

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
2. `dt` 必须来自**时戳之差**，不能用帧序号之差。
3. 视觉来的数据要先过 `cpu/adapters/vision_adapter.py`，不要自己拼 `WorldView`。

分层策略 + 死区（防抖）、全向速度预算 `u = √(v² − w²)`、
到达时间可达性、配置空间膨胀、字典序目标选择（**绝不做加权求和**）、
转向速率限幅、滚动时域重规划、带种子 RNG（可复现）。

其中 `tier` 有一个容易误读的分支：对"缺口在 x 上不动的墙"，算法会输出
`HOLD`，理由是 `gap_will_pass_me`（缺口会自己滑过角色，**停住就是正确解**）。
所以闭环里"角色前向进展 0"是设计行为，不是没动起来。

### B. `pipeline.py` —— CPU↔FPGA 帧回路

```python
from cpu import FrameCpuDecisionLayer, CpuFpgaPipeline, CpuFpgaController
```

把 FPGA 的结果变成 `Action`：

```
state frame (bytes)  --+
                       |     (FPGA, 见 bullet_sim.fpga)
prediction frame ----+ |
                       v v
                 FrameCpuDecisionLayer -> Action -> Simulator
```

**它只依赖** `bullet_sim.fpga.prediction_block`（参考预测块）、
`bullet_sim.interface.protocol`（帧编解码）、`bullet_sim.action`（动作类型）。
硬件真正到位后，唯一变化的是**预测帧从哪来**；CPU 侧只认字节帧、不认 `World`，
所以上面的一切（仿真器、数据集、UI）都不用动。CPU↔FPGA 之间的传输仍是
`[HARDWARE_INTERFACE_TBD]`，`CpuFpgaPipeline` 先在进程内跑通，让接口可测可量。

| 文件 | 职责 |
|---|---|
| `gap_avoid.py` | 交付算法 A（纯 stdlib、单入口） |
| `pipeline.py` | 交付物 B：字节帧回路 + `FrameCpuDecisionLayer` |
| `adapters/vision_adapter.py` | `VisionFrame` → `WorldView`（**比赛链路用这个**） |
| `adapters/sim_adapter.py` | bullet_sim 池子 → `WorldView`（仿真专用） |
| `gap_wall_demo.py` | 可视化/统计演示（不参与移植） |
| `tests/` | 本模块测试 |

---

## 3. 与 `core/bullet_sim/cpu/` 的关系（**未收敛，待决定**）

`core/bullet_sim/cpu/decision.py`（333 行）里还留着一份**旧的单文件版**
CPU↔FPGA 层，导出 `CpuDecisionLayer` / `CpuFpgaPipeline` / `CpuFpgaController`，
与 `cpu/pipeline.py`（321 行）**是同一个东西**：两份逐行比对只差约 42 行，
且基本只在文档字符串上。

两份**都能用、都被测**（`core/bullet_sim/tests/test_cpu_fpga_pipeline.py`
测旧那份，`core/cpu/tests/test_cpu_fpga_pipeline.py` 测新那份），
所以它是"一份实现、两处维护"——改一处忘另一处不会报错。

收敛方向只剩一个选择，**需要你们拍板**：

* **保留 `core/cpu/pipeline.py` 为唯一实现**，把 `core/bullet_sim/cpu/`
  变成 re-export shim。代价：`bullet_sim` 包会反向依赖 `cpu`
  （包级 import 顺序上可行，但平台包不再自洽）。
* **保留 `core/bullet_sim/cpu/` 为唯一实现**，`core/cpu/pipeline.py` 变成
  `from bullet_sim.cpu.decision import ...` 的薄别名。代价：CPU 交付物的一部分
  留在平台包里（但 `cpu` 本来就依赖 `bullet_sim` 的类型，方向没变）。

无论选哪个，重复的那份 `test_cpu_fpga_pipeline.py` 都要删一个。

---

## 4. 本轮收敛删掉了什么

按"只比三种障碍、逐个单独展示、不做混合"的参赛范围，删掉了与之无关的
**通用决策架构**整条生态（约 6.4k 行源码 + 5k 行测试）：

| 删除 | 行数 | 为什么 |
|---|---|---|
| `decision/`（14 文件） | 6,442 | models/predictor/risk/planner + FAR / LTV / FSC / corridor，跨 v0.40→v0.60 六个版本；默认路径是"6 个手调权重的加权和"，与自述的"绝不加权和"矛盾 |
| `tests/test_decision_*.py`（9 文件） | ~2,400 | 上面那套的测试 |
| `benchmark_decision.py` | 809 | 那套架构的离线基准 |
| `baseline_reactive_gap.py` | ~300 | 只作为上面基准的对照组 |
| `tools/`（fsc_shadow_probe、ltv_regression_diagnostics） | 1,141 | FSC/LTV 的离线诊断，属同一套 |
| `adapters/game_adapter.py` | 192 | `Observation ↔ 抽象 SceneState`，只服务于 `decision.models` |
| `docs/decision_v0*.md`、`benchmark_results.md`、`CPU决策架构设计与解耦说明.md` | — | 上面那套的设计与实测记录 |

**没有动的**：`gap_avoid.py`（交付算法）、`pipeline.py`（硬件边界）、
`adapters/{sim,vision}_adapter.py`、`tests/{test_cpu_fpga_pipeline,test_cxx_portability,
test_vision_adapter,test_vision_loop_e2e}.py`、`docs/CPP_PORT.md`。

删除前后用基线指纹卡过：全量测试 **910 → 701**（正好少 209 项，就是待删测试的
用例数，没有连带损失），闭环 `state_hash` **逐位不变**
（`c7bc07c687f697dd`），阴性对照首次碰撞仍是 9.8 s。

---

## 5. 已知问题

### 5.1 `core/bullet_sim/tests/test_determinism.py` 在 `origin/main` 上就失败

```
FAILED core/bullet_sim/tests/test_determinism.py::test_reproducibility_survives_a_fresh_interpreter
```

该测试会 `subprocess` 起一个**全新解释器**去 `import bullet_sim`，但包现在在
`core/` 下，子进程的 `sys.path` 里没有 `core/`，于是 `ModuleNotFoundError`。
**与 `core/cpu/` 无关** —— 在干净工作树上同样复现。临时绕过：

```bash
PYTHONPATH=core python -m pytest core/bullet_sim/tests -q     # 全绿
```

长期修法：在那个测试的 `subprocess.run(...)` 里补
`env={**os.environ, "PYTHONPATH": str(pathlib.Path(__file__).resolve().parents[2])}`，
或加一个根 `conftest.py` 把 `core/` 放进 `sys.path`。

### 5.2 `pyproject.toml` 的打包范围（**已修**）

`include` 原来只有 `["bullet_sim*", "standard*"]`，注释却写着源码都在 `core/`
下（`bullet_sim` / `cpu` / `fpga` / `vision`）。后果是干净 `pip install .`
装出来的包缺 `cpu` 与 `vision` 两个模块，而仓库内开发（conftest 往 `sys.path`
塞 `core/`）**看不出问题** —— 交付要求是"可复现工程"，这一步不能只在开发者
机器上成立。现已改为：

```toml
include = ["bullet_sim*", "cpu*", "vision*", "standard*"]
```

同一处还修了 `testpaths`：它原来不含 `core/cpu/tests`，于是**裸 `pytest`
会静默跳过整个 CPU 模块**（含闭环端到端），"本地全绿"是假象。

---

## 6. 环境与运行前提

* Python ≥ 3.10；实测 3.12 + NumPy + pytest 9.1.1（仓库 `Env/`）。
* 裸源码检出（没 `pip install`）时，`core/cpu/conftest.py` 会把 `core/` 与
  `core/cpu/` 放进 `sys.path`，所以 `python -m pytest core/cpu/tests` 直接能跑。
* 没有 `pygame` 时：`gap_avoid.py` 自检与 `gap_wall_demo.py --headless` 照常；
  `gap_avoid.py` 本身**连 numpy 都不需要**。
* `gap_avoid.py` 可以单独当脚本跑自检：`python core/cpu/gap_avoid.py`。

---

## 7. 许可

本目录的代码是仓库自身的工作成果，按仓库整体许可（MIT，见根目录
`pyproject.toml` 的 `license`）发布。参考项目清单与方法出处见
`core/bullet_sim/docs/OPEN_SOURCE_REFERENCES.md`；参考实现中
**GPL-3.0 的项目（如 `pred-occ-planner`）不得逐行抄入**，本目录没有抄。
