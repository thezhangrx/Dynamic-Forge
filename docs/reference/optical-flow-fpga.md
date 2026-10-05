# `optical-flow-fpga` 技术总结

> 本文是对本地参考项目 `/home/zhang/Bullet_Platform/Outside/Camera/optical-flow-fpga` 的**只读**技术分析，
> 目标是为「二维动态障碍环境模拟器 → CPU/FPGA 异构部署 → 现实二维小车 + 摄像头验证」这条路线提供光流硬件化的工程参考。
> 所有结论均标注来源（`文件:行号`）。凡代码/报告中无法确认的内容，均显式标记为 **未确认**。
> 写作过程中未修改任何已有文件，未运行 Vivado，未安装依赖，未联网。

> ## ⚠️ 先读这一条（本次分析最重要的发现）
> **这份 RTL 不是"已验证可工作"的设计，而是时序/资源研究性质的工程。**
> - **Python 侧**是完整、可复现、有 CI 回归的（§6、§7.1）；
> - **RTL 侧**存在至少 8 处静态可确认的功能性缺陷，其中前 3 条任意一条都会让金字塔控制 FSM **卡死、跑不完一帧**
>   （`pyramid_control_fsm.sv:181-183` 的 `start` 脉冲机制与子模块 `done` 互相等待；`frame_warper.done`
>   与 `flow_upsampler.done` 恒为 0）。详见 **§5.9**。
> - 提交的 Vivado 报告（`prj/unopt/`）**比最终提交早 45 分钟**，且其资源数字与当前 RTL 的存储需求
>   相差 1.75 倍以上，**无法由当前代码复现**。详见 **§5.8**。
>
> ⇒ **可借鉴的是模块级工程套路（行缓冲 / BRAM 模板 / DSP 标注 / 定点约定），不是顶层控制逻辑。**
> 如果你要用它做 `v_rel` 估计，请从 Python 侧（`python/lucas_kanade_core.py`）起步，硬件部分按 §9② 的思路自己重构。

---

## 1. 一句话定位

**是什么**：一个在 Digilent **Nexys A7-100T**（Xilinx Artix-7 `xc7a100tcsg324-1`，15,850 slices / 240 DSP48E1 / 4,860 Kb BRAM，
`README.md:42-44`）上，用 **SystemVerilog 实现 Lucas-Kanade 稠密光流**、并配套一套完整 Python 参考实现与
Vivado 综合/实现报告的**参考工程**（`README.md:3`）。
> ⚠️ 严格说：RTL **做到了"能综合、能出时序/资源报告"**，但按静态读码**存在让控制 FSM 卡死的缺陷（§5.9）**；
> 而且**并不存在** RTL 与 Python 的数值对照（§6.3）。README 自称的 "real-time" 在本仓库中**没有被实测证明**。
> 所以更准确的定位是：**「光流硬件化的工程/时序研究 + 可直接复用的 Python golden model」**。

**实现的算法**：单尺度（`optical_flow_top`）与 3 层金字塔（`optical_flow_top_pyramidal`）两套 L-K 数据通路。
`scripts/synth_config.tcl:50` 把综合顶设为 `optical_flow_top_pyramidal`，所以**工程实际综合/实现的是金字塔版本**，
单尺度版本只用于 `tb/tb_optical_flow_top.sv` 仿真与文档基线。

**作者 / 许可证**：
- 作者：Joshua Rothe（`README.md:629`，`pyproject.toml:10`，邮箱 `21348884+rothej@users.noreply.github.com`）。
- 上游仓库：`https://github.com/rothej/optical-flow-fpga.git`（`git remote -v`）。
- 许可证：**MIT**（`LICENSE:1-7` 为 MIT 全文，版权行 `Copyright 2026 Joshua Rothe`；`README.md:623-625`、`pyproject.toml:32` 亦声明 MIT）。

**README 测试图与来源（务必正确标注）**：
README 的验证素材不是合成棋盘格，而是**一张自然全景图**（`README.md:7-18`）：

| 项 | 内容 |
|---|---|
| 内容 | Fronalpstock, Swiss Alps（瑞士施维茨州）12 帧全景图 |
| 文件 | `python/test_data/mountain_texture.jpg` |
| 摄影者 | Hannes Röst（Wikimedia Commons 用户页） |
| 来源 | Wikimedia Commons，`https://commons.wikimedia.org/w/index.php?curid=11301841` |
| **许可证** | **CC BY-SA 3.0**（`https://creativecommons.org/licenses/by-sa/3.0/`） |

> ⚠️ 注意：**该图片是 CC BY-SA 3.0，与项目代码的 MIT 不同**。若你的项目复用该图，必须保留署名 + 相同方式共享，
> 且不能只写 "MIT"。建议自己重新生成纹理（如 `generate_smooth_synthetic()`，`python/generate_test_frames_natural.py:49-64`）
> 以避免许可证传染。
> 另注：同一张图在 `python/generate_test_suite.py:34`（`CACHED_IMAGE`）和 `python/generate_test_frames_natural.py:16` 被**两处**引用。

---

## 2. 技术栈与工具链

### 2.1 HDL 与综合

| 项 | 值 | 来源 |
|---|---|---|
| 语言 | SystemVerilog（`.sv`，用了 `always_comb` / `always_ff` / `typedef enum` / `logic` / 参数化数组） | 全部 `rtl/**/*.sv` |
| 综合工具 | Vivado **v.2022.2** (lin64)，要求 2022.2+ | `prj/unopt/timing_postroute_unopt.rpt:4`、`README.md:306` |
| 器件 | `xc7a100tcsg324-1` | `scripts/synth_config.tcl:41` |
| 综合入口 | `vivado -mode batch -source scripts/synth_config.tcl -tclargs <unopt\|opt> [true\|false]` | `scripts/synth_config.tcl:3` |
| 包装脚本 | `./scripts/build.sh unopt [impl]`；`impl` 才跑 place & route | `scripts/build.sh:5-6`、`README.md:330-343` |
| 仿真 | Vivado XSim（`scripts/run_sim.tcl`，`launch_simulation` + `run $run_time`，默认 `100ms`） | `scripts/run_sim.tcl:44-49`、`scripts/run_sim.sh:5` |
| 仿真包装 | `./scripts/run_sim.sh <tb_name> [0\|1(波形)] [run_time]` | `scripts/run_sim.sh:4-6` |

### 2.2 `prj/unopt` 与 `prj/opt` 两个变体的**真实**区别（重要）

`scripts/synth_config.tcl:29-39` 定义的映射是：

| config | RTL 目录 | 约束文件 | 输出目录 |
|---|---|---|---|
| `unopt` | `rtl/unopt` | `constraints/timing_unopt.xdc` | `prj/unopt` |
| `opt` | `rtl/opt` | `constraints/timing_opt.xdc` | `prj/opt` |

**但本仓库当前状态下**：
- `rtl/opt/` **不存在**（`find . -type d` 结果里只有 `rtl/common` 和 `rtl/unopt`）。
- `prj/opt/` **不存在**（只有 `prj/unopt` 和 `prj/sim_tb_optical_flow_top`）。
- 唯一远端分支 `origin/feature/opt-pyramidal-1` 存在，但本地未 checkout（`git branch -a`），`git log` 最新提交是
  `cd64c81 Finished synthesizing and implementing unoptimized pyramidal L-K (#3)`。

⇒ **「unopt vs opt」对比在本仓库中无法完成**：优化版尚未落地。`benchmarks/comparison_template.md` 就是为「Part 2 优化版」预留的空表，
所有 Optimized 列都是 `TBD`。`benchmarks/unopt_baseline_20260216/DESIGN_NOTES.md` 给出了**计划中**的优化路线（见 §5.7）。

### 2.3 约束文件 `constraints/*.xdc`

**`constraints/timing_unopt.xdc`（4 行，实际生效的约束）**：
```tcl
create_clock -period 10.000 -name sys_clk [get_ports clk]        # :4  → 100 MHz
set_input_delay  -clock sys_clk 2.0 [get_ports {rst_n start}]     # :5
set_output_delay -clock sys_clk 2.0 [get_ports {flow_u* flow_v* flow_valid}]  # :6
```
- 只有**时钟 + I/O delay**，**没有任何引脚位置约束（PACKAGE_PIN / IOSTANDARD）**。
- ⇒ 这个工程**不能直接上板**：没有物理引脚分配，也没有输入视频接口（见 §5.1）。它定位是「时序/资源研究 + 仿真验证」，不是可烧录的完整系统。

**`constraints/timing_opt.xdc`（目标 200 MHz，尚未可用）**：
```tcl
create_clock -period 5.000 -name sys_clk [get_ports clk]          # :4  → 200 MHz
set_input_delay  -clock sys_clk 1.0 ...                           # :5
set_output_delay -clock sys_clk 1.0 ...                           # :6
set_max_delay 4.5 -from [get_pins u_gradient_compute/pipe_stage1_reg*/C] \
                   -to [get_pins u_gradient_compute/pipe_stage2_reg*/D]   # :9-10
```
> ⚠️ 第 9–10 行引用的 `u_gradient_compute/pipe_stage1_reg*` / `pipe_stage2_reg*` 在**当前** `rtl/unopt/gradient_compute.sv` 中
> 根本不存在（该模块只有一个输出寄存器级 `always_ff`，`gradient_compute.sv:143-159`）。
> 这是一条**为未来优化版预留、但会报空的约束**。

### 2.4 Python 侧（`pyproject.toml` / `python/`）

`pyproject.toml` 声明（`pyproject.toml:7-53`）：

| 项 | 内容 |
|---|---|
| 包名/版本 | `optical-flow-fpga` / `0.1.0-dev` |
| Python | `requires-python = ">=3.12"`；classifier 只列 3.12 |
| 运行时依赖 | `numpy>=1.24`、`opencv-python>=4.8`、`scipy>=1.11`、`matplotlib>=3.8`、`PyYAML>=6.0` |
| dev 依赖 | `pre-commit`、`black`、`isort`、`flake8`、`mypy`、`pytest`、`pytest-cov`、`types-PyYAML`、`types-Pillow` |
| 包发现 | `[tool.setuptools.packages.find] where=["."], include=["python*"]` —— 把 `python/` 当顶层包 |
| mypy | `disallow_untyped_defs = true`，`cv2/scipy/matplotlib/yaml` 忽略缺失导入 |
| pytest | `testpaths = ["python/tests"]`、`--cov=python`、`pythonpath = ["python"]` |

> ⚠️ **`python/tests/` 目录不存在**（`ls python/` 无 `tests`）。`pytest` 配置指向空路径，
> CI 里也**没有**跑 pytest（见 §2.6）。这是配置与现实的落差。

### 2.5 `python/` 目录结构

| 文件 | 行数 | 作用 |
|---|---|---|
| `lucas_kanade_core.py` | 135 | **单尺度 L-K 核心**（Sobel + 5×5 结构张量 + Cramer 解），被单尺度与金字塔共用 |
| `lucas_kanade_reference.py` | 208 | 单尺度 CLI 驱动：读 `frame_0X.bin` → 算流场 → 存 `flow_u.bin`/`flow_v.bin`/`flow_field_python.txt` + 可视化 |
| `lucas_kanade_pyramidal.py` | 479 | 3 层高斯金字塔 + 由粗到细迭代（含 `warp_image`、`upsample_flow`） |
| `flow_metrics.py` | 201 | MAE / RMSE / EPE / AAE 四个标准指标 |
| `optical_flow_verifier.py` | 923 | 验证主程序（13 模式 × 2 实现 + 阈值分类 + 可视化 + 基线回归） |
| `generate_test_suite.py` | 422 | 13 个合成模式的生成器（OpenCV 变换），导出 `.bin` / `.mem` / `.png` / `metadata.json` |
| `generate_test_frames_natural.py` | 134 | 简易生成器（自然图 + `scipy.ndimage.shift` 亚像素位移，`:67-73`），`run_sim.sh` 实际调用它 |
| `test_data/mountain_texture.jpg` | — | CC BY-SA 3.0 素材（§1） |
| `verification_config.yaml` | 146 | 阈值 / 模式分类 / 金字塔配置 / 可视化参数 |
| `verification_baseline.json` / `verification_results.json` / `verification_results.md` | — | 回归基线与最新结果（§7） |
| `output/`、`verification_plots/` | — | 生成的图（`pyramid_level_*.png`、`flow_comparison.png`、`flow_single/pyramidal.png`、`error_*.png`） |
| `tb/test_frames/` | — | `frame_00.mem`/`frame_01.mem`（与根 `tb/test_frames/` 重复） |

> ⚠️ README 引用了 **不存在的文件**：`python/lucas_kanade.py`（`README.md:125`）、`python/generate_test_frames.py`（`README.md:403`）。
> 实际对应文件是 `lucas_kanade_reference.py` + `lucas_kanade_core.py` 和 `generate_test_frames_natural.py`。

### 2.6 CI 与脚本

**`.github/workflows/verify_optical_flow.yml`（唯一的 GitHub workflow）**：
- 触发：`pull_request`（改动 `python/**` 或 `rtl/**`）+ `push` 到 `main`/`dev`（`:3-8`）。
- 步骤：`actions/setup-python@v4` (3.12) → 缓存 pip → `pip install -e .[dev]` →
  `flake8 python/ --select=E9,F63,F7,F82`（**硬失败**，只查语法级错误）→
  `mypy python/`（`continue-on-error: true`，**不硬失败**）→
  `python python/generate_test_suite.py` → `python python/optical_flow_verifier.py --compare-baseline --no-visualizations --regression-threshold 10.0` →
  失败时上传 `python/verification_results.md`（`:33-45`）。

> **关键事实：CI 完全不涉及 RTL。** 没有 Verilator/Icarus/XSim，没有 testbench 编译，
> 没有 RTL 输出与 Python 的比对。所谓 "Optical Flow Verification" 实际只验证 **Python 算法与基线的一致性**。
> `.gitlab-ci.yml` 是同一套的 GitLab 镜像（Python 3.11 镜像，`only: changes: python/**, rtl/**`），同样不跑 HDL。

**`scripts/` 一览**：

| 脚本 | 作用 | 关键行 |
|---|---|---|
| `build.sh` | 参数校验 + 调 `synth_config.tcl` | `:5-6` |
| `synth_config.tcl` | 建工程 / 加 RTL / 加 XDC / **top = `optical_flow_top_pyramidal`** / synth + 可选 impl + 出报告 | `:41-70`、`:50` |
| `run_sim.sh` | 先 `generate_test_frames_natural.py --displacement-x 2`，再跑 XSim，把 `flow_field.txt` 拷成 `flow_field_rtl.txt`，用 `awk` 统计 x[55:85]/y[105:135] 区域均值 | `:20-22`、`:38-62` |
| `run_sim.tcl` | 建仿真工程、加 `rtl/unopt/*.sv`+`rtl/common/*.sv`+tb、**把 `.mem` 拷到 xsim 工作目录**、`launch_simulation`+`run` | `:29-46` |
| `convert_frames.py` | `.mem`（每行 2 位十六进制）→ PNG，**硬编码 320×240** | `:36-38`、`:52-53` |
| `visualize_flow.py` | `flow_field.txt` → 4 面板诊断图（quiver / 幅度 / 直方图 / 误差热图），默认输出 `results/flow_visualization.png` | `:309-384` |
| `regenerate_flow_plots.sh` | 一键重生成 15px 场景的所有图 | `:9-17` |
| `setup_verible.sh` | 下载 Verible 到 `.tools/verible/bin`（**离线环境下不可用**） | — |
| `pre_merge_check.sh` | 本地跑 CI 六步（装依赖 / 生成测试集 / 验证 / 检查产物 / 回归 / lint） | `:1-94` |
| `copy_files.sh` | 把 `.py/.sh/.sv/.xdc/.tcl/.yaml/.yml/.md` 汇总成 `combined_files.txt`（喂 LLM 用），排除 `.direnv`/`.venv`/`prj` | `:1-47` |

> ⚠️ `README.md:565` 与 `README.md:571` 教用户跑 `python scripts/visualize_flow.py flow_field_rtl.txt --compare`，
> 但 `visualize_flow.py:311-345` 的 `argparse` **没有 `--compare` 选项**（只有 `flow_file --frame --output --ground-truth-u/v --stride --scale`）。
> 该 README 步骤会直接报错。

---

## 3. 目录与模块地图

```
optical-flow-fpga/
├── rtl/
│   ├── common/                 # 两个跨实现复用的基础模块
│   │   ├── line_buffer_5x5.sv      # 167 行：4 整行 + 5 级移位寄存器 → 5×5 窗口
│   │   └── frame_buffer_simple.sv  #  96 行：仿真专用双帧缓冲（$readmemh，综合时会被优化掉）
│   └── unopt/                  # 唯一存在的实现（"会 fail timing"）
│       ├── optical_flow_top.sv              # 160 行：单尺度顶层（流式，无金字塔）
│       ├── optical_flow_top_pyramidal.sv    #1432 行：★ 实际综合顶层，3 层金字塔
│       ├── gradient_compute.sv              # 161 行：Sobel Ix/Iy + It
│       ├── window_accumulator.sv            # 192 行：5×5 结构张量累加（无流水，组合加法链）
│       ├── flow_solver.sv                   # 168 行：Cramer 法则 + 组合 32 位除法 ★ 关键路径
│       ├── pyramid_control_fsm.sv           # 248 行：12 状态总控 FSM
│       ├── pyramid_builder.sv               # 404 行：2×2 平均降采样，建 L0/L1/L2（curr+prev 各一份）
│       ├── frame_memory.sv                  #  51 行：`(* ram_style="block" *)` 双口 BRAM 模板 ★ BRAM 来源
│       ├── pixel_sequencer.sv               # 102 行：BRAM → 光栅扫描像素流（1 px/cycle）
│       ├── frame_warper.sv                  # 247 行：按流场反向 warp 帧（双线性，含 3 次 8×8 乘）
│       ├── flow_upsampler.sv                # 238 行：流场 2× 双线性上采样 + 幅度 ×2
│       ├── flow_accumulator.sv              # 136 行：base flow + residual flow 累加
│       └── bram_read_arbiter.sv             #  64 行：共享 BRAM 读口静态优先级仲裁 ❗**从未被例化**
│   # ⚠️ README 声称存在 rtl/opt/（README.md:34），实际不存在
├── tb/
│   ├── tb_optical_flow_top.sv            # 428 行：单尺度端到端 TB（★ 统计 + 导流场）
│   ├── tb_optical_flow_top_pyramidal.sv  # 127 行：金字塔 TB（只查 FSM 是否到 DONE）
│   ├── tb_frame_buffer.sv                # 122 行：帧缓冲单测
│   └── test_frames/frame_0{0,1}.{mem,png} # 76,800 行 .mem + PNG（由 Python 生成）
├── constraints/{timing_unopt,timing_opt}.xdc
├── prj/
│   ├── unopt/                    # ★ 唯一真实综合/实现产物（顶 = optical_flow_top_pyramidal）
│   │   ├── optical_flow_unopt.xpr
│   │   ├── timing_summary_unopt.rpt / timing_postroute_unopt.rpt
│   │   ├── utilization_unopt.rpt / utilization_postroute_unopt.rpt
│   │   └── route_status_unopt.rpt
│   └── sim_tb_optical_flow_top/sim_project.xpr
│   # ⚠️ README 声称存在 prj/opt/（README.md:29），实际不存在
├── benchmarks/
│   ├── unopt_baseline_20260216/   # 2026-02-16 的「Part 1 未优化」快照 + 设计笔记
│   │   ├── metrics.txt, DESIGN_NOTES.md, logic_levels.txt, rtl_manifest.txt
│   │   ├── critical_path_verbose.txt（150 行，含 -130.411ns 路径全展开）
│   │   ├── critical_path_detail.txt（❗0 字节，空文件）
│   │   └── timing_*/utilization_*/route_status_*.rpt（prj/unopt 的副本）
│   └── comparison_template.md    # 预留的 unopt vs opt 对比表（Optimized 列全 TBD）
├── results/flow_visualization.png  # RTL 流场 4 面板诊断图（README.md:536-539 展示的那张）
├── docs/
│   ├── NOTATION.md                # LaTeX / Mermaid / ASCII 变量命名约定
│   ├── architecture/block_diagram.md  # Mermaid 顶层框图 + 模块层次 + 数据流 + 关键路径（★但内容已过时，见下）
│   └── baseline/                  # ❗这是**另一套**数据：单尺度 optical_flow_top 的 2026-02-07 基线
│       ├── README.md              # WNS=-110.051ns / LUT 3765 / FF 129 / DSP 23 / BRAM 0
│       └── reports/{timing_summary_unopt,utilization_unopt,critical_paths_detail}.txt
└── python/  (见 §2.4/§2.5)
```

**模块层次（金字塔顶层，来自 `optical_flow_top_pyramidal.sv` 的实例名）**：

```
optical_flow_top_pyramidal
├── u_fsm                    pyramid_control_fsm            :93-117
├── u_pyramid_curr           pyramid_builder                :135-159
├── u_pyramid_prev           pyramid_builder                :162-186
├── frame_memory × 19        （见 §5.4 BRAM 预算）           :197-524
├── u_upsample_l0_to_l1      flow_upsampler                 :592-600
├── u_upsample_l1_to_l2      flow_upsampler                 :618-626
├── u_seq_l0 / _l1 / _l2     pixel_sequencer × 3            :657-707
├── u_warp_l1 / u_warp_l2    frame_warper × 2               :753-783
├── u_accum_l1 / u_accum_l2  flow_accumulator × 2           :857-890
└── ★ 每层一整套单尺度 L-K 数据通路（3 份拷贝）：
    ├── u_gradient_compute_l0 / _l1 / _l2   gradient_compute   :942 / :1017 / :1092
    ├── u_window_accumulator_l0 / _l1 / _l2 window_accumulator :964 / :1039 / :1114
    └── u_flow_solver_l0 / _l1 / _l2        flow_solver        :985 / :1060 / :1135
```

> ⚠️ `docs/architecture/block_diagram.md` 描述的是**早期的单尺度结构**（模块名 `lucas_kanade_solver`、`gradient_accumulator`，
> 见该文件模块层次图），与当前 RTL 的实际模块名（`window_accumulator` / `flow_solver`）**不一致**；
> 同一 `docs/baseline/` 下的资源数字也是旧的（§7.4）。阅读时以 RTL 为准。

---

## 4. Lucas-Kanade 光流数学（先理论，再对齐本项目的**实际**实现）

### 4.1 两条基本假设

1. **亮度恒定（brightness constancy）**：同一物理点在相邻两帧中亮度不变，`I(x, y, t) = I(x+u, y+v, t+1)`。
2. **小运动（small motion）+ 局部平移一致**：在窗口内 `(u, v)` 是常量，且足够小以致一阶泰勒展开成立：
   `I_x u + I_y v + I_t = 0`。

这两条假设是本项目全部局限的根源（§8）：旋转 / 缩放 / 大位移直接破坏假设 2，光照变化破坏假设 1。

### 4.2 空间梯度 `I_x, I_y`：**本实现用的是 Sobel（÷8），不是中心差分**

**硬件（`rtl/unopt/gradient_compute.sv`）**：

- 先在**当前帧与前一帧的平均帧**上做卷积，以抑制噪声（`:113-118`）：
  ```systemverilog
  avg_window[i][j] = (window_curr[i][j] + window_prev[i][j]) >> 1;   // :116
  ```
- `I_x` 用左右两列分别求和再相减（`:121-127`）：
  ```
  sobel_x_left  = -(w00) - 2*(w10) - (w20)
  sobel_x_right = +(w02) + 2*(w12) + (w22)
  sobel_x_comb  = (sobel_x_left + sobel_x_right) >>> 3;   // :127  ÷8
  ```
- `I_y` 用上下两行（`:130-136`）：
  ```
  sobel_y_top    = -(w00) - 2*(w01) - (w02)
  sobel_y_bottom = +(w20) + 2*(w21) + (w22)
  sobel_y_comb   = (sobel_y_top + sobel_y_bottom) >>> 3;  // :136  ÷8
  ```
- **位宽增长**：`avg_window` 为 `signed [PIXEL_WIDTH:0]`（9 位，`:109`）；`sobel_*_left/right/top/bottom` 为
  `signed [GRAD_WIDTH+2:0]`（15 位，`:110-111`）；最终**算术右移 3 位截断回 `signed [11:0]`（S12）** 存入 `grad_x/grad_y`。
  ⇒ **Sobel 除以 8 是无条件截断（floor），且丢掉了低位精度**，这是 README 自认的幅度低估原因之一（`README.md:381-384`）。

**Python 参照（`python/lucas_kanade_core.py`）** 与之**完全一致**（这是"golden model"能对上的关键）：
```python
sobel_x = np.array([[-1,0,1],[-2,0,2],[-1,0,1]], dtype=np.float32) / 8.0   # :32
sobel_y = np.array([[-1,-2,-1],[0,0,0],[1,2,1]], dtype=np.float32) / 8.0   # :33
frame_avg = (frame_prev + frame_curr) / 2.0                                # :36
Ix = signal.convolve2d(frame_avg, sobel_x, mode="same", boundary="symm")   # :39
Iy = signal.convolve2d(frame_avg, sobel_y, mode="same", boundary="symm")   # :40
```
> 唯一差别：Python 边界用 `boundary="symm"`（镜像），**RTL 边界用的是补 0**（`line_buffer_5x5.sv:106-138` 的
> `(col >= k) ? line[col-k] : '0`，以及 `window_valid` 在前 4 行/4 列无效，`line_buffer_5x5.sv:82-86`）。
> ⇒ **边界像素的行为不可逐位对齐**（RTL 直接不输出，Python 用镜像外推）。

### 4.3 时间梯度 `I_t`：**简单帧差，取窗口中心像素**

```systemverilog
temporal_comb = $signed({4'b0, window_prev[1][1]}) - $signed({4'b0, window_curr[1][1]});  // gradient_compute.sv:139
```
即 `I_t = I_prev(center) - I_curr(center)`，零扩展 4 位到 12 位，**只取 3×3 中心**（不是 5×5 中心）。
Python 侧：`It = frame_prev - frame_curr`（`lucas_kanade_core.py:43`）——逐像素全图，语义等价。

### 4.4 局部窗口最小二乘（结构张量）

对每个像素，在其 5×5 邻域内最小化 `Σ (I_x u + I_y v + I_t)²`，令偏导为 0：

```
A = [[ Σ I_x² , Σ I_x I_y ],      b = -[ Σ I_x I_t ]
     [ Σ I_x I_y , Σ I_y²  ]]          [ Σ I_y I_t ]
A·d = b ,  d = [u, v]ᵀ  ⇒  d = (AᵀA)⁻¹ Aᵀ b
```
（README `README.md:57` 与 `python/README.md:59-70` 写的公式完全一致。）

**硬件累加（`rtl/unopt/window_accumulator.sv`）**：5 个 `signed [23:0]` 乘积 × 25 个窗口位置：
```
ΣIxIx, ΣIyIy, ΣIxIy, ΣIxIt, ΣIyIt      // :102-106 声明；:130-134 计算
```
累加结果截断进 `signed [31:0]`（`ACCUM_WIDTH=32`，`window_accumulator.sv:23`、`:144-148`、`:181-185`）。

**Python 累加（`lucas_kanade_core.py:107-125`）**：逐像素 Python `for` 循环 + `np.sum`，`float32`；
`b = [-sum_IxIt, -sum_IyIt]`（`:125`）——**注意 y 分量的负号在这里施加**，而 RTL 把符号放在 `flow_solver` 的分子里（§5.5）。

### 4.5 可解性判据

| 实现 | 判据 | 处理 | 来源 |
|---|---|---|---|
| RTL | `\|det\| > DET_THRESHOLD = 1000` | 不满足 → `flow_u = flow_v = 0`（**输出零向量，不是 valid=0**） | `flow_solver.sv:45`、`:123`、`:145-148` |
| Python | `abs(det) > 1e-4` | 不满足 → 保持 0 | `lucas_kanade_core.py:131-133` |

> ⚠️ **这是 RTL 与 Python 之间一个数量级完全不同的阈值**（1000 vs 1e-4）。二者无法逐位对齐。
> 而且 RTL 的门限是**绝对量**，随窗口内梯度幅度（即纹理强度和对比度）变化：
> 在低对比度区域，即使纹理方向明确，`det` 也可能 < 1000 而被判为「不可解」，输出 0。
> `det` 由 `prod_det1[31:0] - prod_det2[31:0]` 得到（`flow_solver.sv:116`），即**把两个 64 位乘积截断到低 32 位再相减**，
> 高位信息丢失（`flow_solver.sv:55-57` 声明为 `[2*ACCUM_WIDTH-1:0]` = 64 位）。
> 关于 `[-8, +8] px` 的饱和见 §5.5。

### 4.6 亚像素位移的定点表示：**S8.7（`FLOW_WIDTH=16`，`FRAC_BITS=7`）**

```systemverilog
parameter int FLOW_WIDTH = 16;   // optical_flow_top.sv:22 / optical_flow_top_pyramidal.sv:22
.FRAC_BITS (7)                   // optical_flow_top.sv:121 / _pyramidal.sv:984
scaled_num_u = numerator_u <<< FRAC_BITS;    // flow_solver.sv:127  ×128
flow_u_comb  = scaled_num_u / det;           // flow_solver.sv:130  → Q7 定点
```
换算：**LSB = 1/128 px**，可表示范围 `[-256, 255.9921875] px`（16 位有符号），
但实际被 clamp 到 `±1024` LSB = **`±8 px`**（`flow_solver.sv:134-144`）。

TB 侧的逆换算（`tb/tb_optical_flow_top.sv:111-115`）：
```systemverilog
result = $signed(fixed_val) / 128.0;   // 2^7 = 128
```
> ⚠️ `flow_solver.sv:127` 的 `scaled_num_u = numerator_u <<< FRAC_BITS` 变量声明为
> `signed [ACCUM_WIDTH+FRAC_BITS-1:0]` = `signed [38:0]`（`:113`），但 `numerator_u` 只有 32 位（`:119`），
> 所以只有低 32 位被左移，**符号扩展依赖 SystemVerilog 的赋值宽度规则**；同时
> `numerator_u <<< 7` 的 32 位结果**可能溢出**（`det` 可达 2³¹ 量级）。
> 这是一个真实的精度/溢出风险点，**未在报告中量化**（未确认）。

### 4.7 金字塔 / 多尺度 / 迭代：**RTL 做了 3 层金字塔；每层只迭代 1 次；不做图像 warp 后再迭代（是 warp 后重新求解，不是迭代精化）**

**Python 侧（`python/lucas_kanade_pyramidal.py`）**：
- 金字塔：`build_gaussian_pyramid(image, num_levels, scale_factor=0.5)`（`:23`），
  先 `gaussian_filter(sigma=1.0/0.5=2.0)` 再 `map_coordinates` 双线性降采样（`:45-56`），
  `pyramid.insert(0, current)` 使列表**由粗到细**（`:58`）。
- 由粗到细：`for level in range(num_levels)`（`:187`）→ 每层 `for iteration in range(num_iterations)`（`:201`）：
  `warp_image(img_curr, flow_u, flow_v)`（`:203`）→ `lucas_kanade_single_scale(img_prev, img_warped, window_size)`（`:206`）→ 累加残差。
- 默认 `num_levels=3, window_size=5, num_iterations=3`（`:144-146`，亦见 `verification_config.yaml` 的 `pyramids.default`）。
- 金字塔尺寸（320×240 输入）：`L0 = 80×60`（最粗），`L1 = 160×120`，`L2 = 320×240`（最细）——见 `README.md:239-248`。

**RTL 侧（`rtl/unopt/optical_flow_top_pyramidal.sv` + `pyramid_control_fsm.sv`）**：
- 尺寸定义：`L0 = 80×60`（最粗）、`L1 = 160×120`、`L2 = 320×240`（`optical_flow_top_pyramidal.sv:45-50`）。
- FSM 12 状态（`pyramid_control_fsm.sv:59-72`）：
  ```
  IDLE → BUILD_PYRAMID → SOLVE_L0 → UPSAMPLE_L0 → WARP_L1 → SOLVE_L1
       → ACCUM_L1 → UPSAMPLE_L1 → WARP_L2 → SOLVE_L2 → ACCUM_L2 → DONE_ST(→IDLE)
  ```
- **结构是「每个尺度单独跑一遍完整单尺度 L-K」**：三份独立的 `gradient_compute`+`window_accumulator`+`flow_solver`
  （`optical_flow_top_pyramidal.sv:942-1135`），每层**只调用一次** `lk_solve_start`（`pyramid_control_fsm.sv:185-188`、
  `:201-206`、`:225-230`），**没有任何"迭代 3 次"的循环**。
- **⇒ 与 README 的表述不一致**：`README.md:62` 说 RTL 是 "Single-Scale"，但实际综合的是金字塔；
  `README.md:443-449` 也说「Following results are for single-scale L-K」。README 的架构章节
  （`README.md:48-62`）描述的是单尺度的 3 级流水，而不是金字塔顶层。**README 整体落后于 RTL。**

---

## 5. 硬件微架构逐级拆解

### 5.1 输入视频源：**testbench 用 `$readmemh` 喂 `.mem`；不是真实相机**

有**两套互不兼容**的像素源：

**(A) 单尺度顶层 `optical_flow_top`** — 内部自带仿真帧缓冲：
`rtl/common/frame_buffer_simple.sv`
- 两个数组 `frame_0[76800]` / `frame_1[76800]`，各 8 位（`:37-38`）；
- `initial` 块里 `$readmemh(FRAME0_FILE, frame_0)` / `FRAME1_FILE`（`:41-48`），默认路径 `"tb/test_frames/frame_00.mem"`（`:20-21`）；
- 线性计数器 `pixel_cnt`，`pixel_x = pixel_cnt % 320`，`pixel_y = pixel_cnt / 320`（`:56-57`）——**每周期 1 像素**；
- 输出映射：**`pixel_curr = frame_1[pixel_cnt]`，`pixel_prev = frame_0[pixel_cnt]`**（`:88-93`）；
- `frame_done` 在最后一像素后拉高 1 周期（`:77-80`）。

> ⚠️ **该帧缓冲在综合时是死的**：`initial` 块不可综合，`frame_0/frame_1` 永远无写入，
> 综合工具把 `pixel_curr/pixel_prev` 视为常量 0，从而把整条数据通路大面积剪掉。
> 这直接解释了 `docs/baseline/` 那份「单尺度」报告的异常资源数字（LUT 3765 / FF 129 / BRAM 0，见 §7.4）。

**(B) 金字塔顶层 `optical_flow_top_pyramidal`** — **像素从端口流入**（真正的可综合输入）：
`pixel_curr[7:0]`, `pixel_prev[7:0]`, `pixel_valid`（`optical_flow_top_pyramidal.sv:34-36`），
文件里原本的 `frame_buffer_simple` 实例已被注释掉（`:59-77`）。
TB `tb/tb_optical_flow_top_pyramidal.sv:50-71` 用 `$readmemh` 加载两个 76800 深数组，然后逐拍驱动。

> ⚠️ ⚠️ **金字塔 TB 里当前帧/前一帧接反了**：
> ```systemverilog
> pixel_curr = frame0_mem[i];   // tb_optical_flow_top_pyramidal.sv:64
> pixel_prev = frame1_mem[i];   // :65
> ```
> 而生成脚本里 `frame_1 = apply_motion(frame_0, ...)`（`python/generate_test_frames_natural.py:106`、`:67-73`），
> 单尺度 TB 也把 `frame_1` 当 current（`frame_buffer_simple.sv:90`）。
> ⇒ 金字塔 TB 会得到**相反符号**的流场，且该 TB **根本不检查流场数值**（只查 FSM 到没到 `DONE_ST`，`:111-115`），
> 所以这个符号错误不会被发现。

**`.mem` 格式**：每行一个 2 位十六进制字节（`python/generate_test_frames_natural.py:113-119` 的 `f"{val:02x}\n"`，
`python/generate_test_suite.py:264-271` 相同）；`frame_00.mem` 共 76,800 行。
Python 同时导出 `.bin`（`np.tofile`，供 `lucas_kanade_reference.py:133-134` 用 `np.fromfile` 读回）与 `.png`。

> ⚠️ `scripts/run_sim.tcl:38-41` 把 `.mem` 拷到 xsim 工作目录的 `tb/test_frames/` 下，
> 说明 TB 里那个相对路径是**运行时约定**，直接跑 `xsim` 而不经 `run_sim.tcl` 会找不到文件。

### 5.2 行缓冲 / 窗口生成：**5×5 窗口，4 条整行 + 5 级移位寄存器**

`rtl/common/line_buffer_5x5.sv`（参数 `WIDTH / HEIGHT / DATA_WIDTH`）：

| 项 | 值 | 行号 |
|---|---|---|
| 窗口大小 | **5×5**（硬编码，数组维度写死 `[5][5]`） | `:30`、`:37-40` |
| 行存储 | `line0[WIDTH]` ~ `line3[WIDTH]`（**4 条整行**）+ `current[5]`（当前行 5 级移位） | `:37-43` |
| 每行位宽 | `DATA_WIDTH`（像素级 = 8，梯度级 = 12） | `:22` |
| 存储总量（每实例） | `4 × WIDTH × DATA_WIDTH` 位 | `:37-40` |
| 行移位 | 每 `data_valid` 拍把 `line0[col]` 推入 `line3←line2←line1←line0` | `:91-98` |
| 窗口输出 | 组合多路选择，`(col >= k) ? lineX[col-k] : '0` | `:101-139` |
| 边界处理 | **前 4 行 / 前 4 列填 0**；`window_valid` 仅当 `row>=4 && col>=4` | `:82-86`、`:141` |
| 窗口中心坐标 | `window_x = col - 2`，`window_y = row - 2`（`valid_q` 时） | `:144-152` |
| 注释声明 | 该文件头注释（`:12-13`）说使用分布式 RAM（FFs），但**综合报告显示 `LUT as Distributed RAM = 0`、`LUT as Memory = 0`**（`prj/unopt/utilization_postroute_unopt.rpt:37`、`:80`） | — |

**顶层实例数**：`gradient_compute` 内 2 个（curr/prev，`gradient_compute.sv:59-87`），
`window_accumulator` 内 3 个（Ix/Iy/It，`window_accumulator.sv:53-92`）⇒ **每个 L-K 尺度 5 个**，
金字塔三层共 **15 个 `line_buffer_5x5`**。

**窗口提取的"居中"细节需要注意**：`line_buffer` 的 `window[4][2]` 被注释标成 5×5 中心（`:136`），
而 `gradient_compute` 又从中取出 `[i+1][j+1]` 作为 3×3（`gradient_compute.sv:90-97`）——**两级都做了偏移**，
所以梯度阶段的 3×3 中心 = `window_5x5[2][2]`，与 `It` 用的 `window[1][1]`（`gradient_compute.sv:139`）
**不是同一个像素**。这是一个**±1 像素的坐标偏差**，会让 `I_x/I_y` 与 `I_t` 采样点错开。
模型/TB 都没有对此做补偿（Python 侧 `convolve2d(mode="same")` 是严格居中的）。**这是最可能导致 RTL↔Python 偏差的系统性 bug 之一。**

### 5.3 梯度计算模块：`gradient_compute`

| 项 | 值 | 行号 |
|---|---|---|
| 端口 | `pixel_curr/prev` (8b) → `grad_x/grad_y/grad_t` (each `signed [11:0]`)，`pixel_valid` → `grad_valid` | `:28-40` |
| 内部 3×3 窗口 | `window_curr[3][3]` / `window_prev[3][3]`（从 5×5 取中心） | `:47-48`、`:90-97` |
| `avg_window` | `logic signed [8:0]`（9 位） | `:109` |
| Sobel 中间量 | `logic signed [14:0]`（`GRAD_WIDTH+2`） | `:110-111` |
| 归一化 | `>>> 3`（÷8，算术右移，截断） | `:127`、`:136` |
| `It` | `prev[1][1] - curr[1][1]`，零扩展 4 位 → 12 位 | `:139` |
| 流水级数 | **1 级**（一个 `always_ff` 输出寄存） | `:143-159` |
| 延迟 | 输入到 `grad_valid` 共 **1 拍** + 行缓冲的 1284 拍 | `:151-158`、`tb_optical_flow_top.sv:118` |
| 注释声明 | 文件头写 "Unoptimized: combinational Sobel with no pipelining"（`:14`），但实际有 1 级输出寄存 | — |

### 5.4 结构张量累加：`window_accumulator`

| 项 | 值 | 行号 |
|---|---|---|
| 输入 | `grad_x/y/t` `signed [11:0]`，`grad_valid` | `:28-31` |
| 3 个 5×5 行缓冲 | `window_Ix/Iy/It` `[5][5]`，`DATA_WIDTH=GRAD_WIDTH=12` | `:46-48`、`:53-92` |
| **Stage 1（寄存）** | 5 组 × 25 = **125 个 `signed [23:0]` 乘积**，全部带 `(* use_dsp = "yes" *)` | `:102-106`、`:112-141` |
| **Stage 2（组合）** | 5 条 **25 项的线性加法链**（`for i, for j` 顺序累加，无加法树） | `:150-167` |
| 输出寄存 | `sum_*` 截断为 `signed [31:0]`（`ACCUM_WIDTH=32`） | `:170-190` |
| 流水级数 | **2 级**（乘 + 存），第 2 级内部还有 25 项组合链 | `:112-141`、`:150-167` |
| 溢出保护 | **没有**。25 × 24 位最大 ≈ 2²⁹ 量级，32 位理论上够；但由于 `prod_*_pipe` 是 24 位、加法在 32 位里做，**没有饱和逻辑**，只有二进制回绕 | `:144-167` |
| 文件头声明 | "Critical path has qty 5 12-bit multiplications … and qty 25 parallel accumulations **all done on a single clock cycle**"（`:11-13`）——与当前的 2 级实现**已不符** | — |

### 5.5 2×2 求解：`flow_solver` —— **用组合 32 位除法（Cramer 法则），无倒数 LUT / 无 Newton-Raphson / 无 CORDIC / 无除法器 IP**

| 项 | 值 | 行号 |
|---|---|---|
| 方程 | `A·[u,v]ᵀ = -[ΣIxIt, ΣIyIt]ᵀ` | `:10-13` |
| Stage 1（寄存） | `prod_det1 = IxIx·IyIy`、`prod_det2 = IxIy·IxIy`；`prod_num_u1 = IyIy·IxIt`、`prod_num_u2 = IxIy·IyIt`；`prod_num_v1 = IxIx·IyIt`、`prod_num_v2 = IxIy·IxIt`（6 个 32×32→64 位乘积，`use_dsp="yes"`） | `:55-57`、`:83-92` |
| Stage 2（组合） | `det = prod_det1[31:0] - prod_det2[31:0]`；分子同理——**64 位乘积被截到低 32 位** | `:116`、`:119-120` |
| 可解性 | `solvable = (det > 1000) \|\| (det < -1000)` | `:45`、`:123` |
| 定点缩放 | `scaled_num = numerator <<< 7`（×2⁷） | `:127-128` |
| **除法** | `flow_u_comb = scaled_num_u / det` —— **纯组合 32 位有符号除法** | `:130-131` |
| 饱和 | 结果 clamp 到 `±16'sd1024` = **±8 px**（`1024/128`） | `:134-144` |
| 不可解 | 输出 `0`（`flow_valid` 仍然有效） | `:145-148` |
| 输出寄存 | `flow_u/flow_v` `signed [15:0]` | `:152-166` |
| 流水级数 | **2 级**（乘 + 组合除法） | — |
| 定点格式 | **S8.7**（`FLOW_WIDTH=16`, `FRAC_BITS=7`） | `:21-22` |

**精度/正确性隐患（务必注意）**：
1. **`>>` vs `/` 的符号语义**：`scaled_num_u = numerator_u <<< 7` 后直接 `/ det`（有符号除法，向零截断），
   而 Python 用 `float32` 真除。负号 + 非整除时两者相差至多 1 LSB，但会**系统性偏向零**。
2. **`prod_det1[31:0]`**：把 64 位乘积直接切片到低 32 位，若真实 `det` 超出 32 位范围，结果是**回绕的错误值**，
   而 `det` 的符号决定了流向。**这是最危险的一处**（`flow_solver.sv:116`）。
3. `/` 综合出的**组合除法器是 ~100 级 LUT 级联**（见 §5.6），这是全设计的关键路径。
4. `DET_THRESHOLD` 是绝对门限且尺度相关（§4.5）。

### 5.6 流水线与吞吐

**单尺度 `optical_flow_top`**：

| 项 | 值 | 来源 |
|---|---|---|
| 流水级数 | 行缓冲（1284 拍）+ 行缓冲（1284 拍）+ 寄存器 2 拍 | `tb_optical_flow_top.sv:118-126` |
| 计算延迟 | `1284 + 1284 + 2 = 2570` 拍 | `tb_optical_flow_top.sv:125-126` |
| 握手 | **无 ready/valid 反压**；只有 `start`/`busy`/`done` 与流式 `flow_valid` | `optical_flow_top.sv:29-38`、`:143-158` |
| 每像素周期 | **1 像素/周期**（帧缓冲每拍推 1 像素，各级均为全流水吞吐） | `frame_buffer_simple.sv:76-93` |
| 每帧周期 | `76800 + 2570 ≈ 79,370` 拍 | 由上推导 |
| 实测日志一致性 | README 的 RTL 仿真日志（`README.md:484-507`）中"首有效流"与"完成"的比值 ≈ 29.5，与 `79,370 / 2,570 ≈ 30.9` 同量级；完成时刻按 1 像素/拍解读为 76,802 拍 | `README.md:489`、`:507` |
| 有效输出数 | `73,289 / 76,800`（README 日志）；理论上按 8 行 + 8 列边界损失应为 `(320-8)×(240-8) = 72,384`，**两者差 905 个，未对账（未确认，疑与 `grad_valid`/坐标寄存的 1 拍错位有关）** | `README.md:512` |
| 吞吐（推导） | @100 MHz：76800 拍 ≈ **768 µs/帧 ≈ 1,302 FPS**；@实测 7.12 MHz：**≈ 93 FPS** | 见 §7.3 |
| TB 报告的 latency | 日志打印 "Latency: 0 clock cycles"，**这是 TB 的 bug**（`first_valid_cycle = valid_flow_count`，首次有效时该值恰为 0） | `tb_optical_flow_top.sv:184-191` |

**金字塔 `optical_flow_top_pyramidal`**（★ 无实测日志，以下为**按 FSM 状态数推导**的估算，标注为估算）：

| 阶段 | 每像素周期 | 像素数 | 估算拍数 | 依据 |
|---|---|---|---|---|
| BUILD：L2 存储 / L1 降采样 / L0 降采样 | 1 / **13** / **13** | 76,800 / 19,200 / 4,800 | 76,800 + 249,600 + 62,400 = 388,800 | `pyramid_builder.sv` 主/ L1 / L0 三个 FSM |
| SOLVE_L0 / L1 / L2（pixel_seq + L-K） | 1 + 2570 流水延迟 | 4,800 / 19,200 / 76,800 | 7,370 + 21,770 + 79,370 = 108,510 | `pixel_sequencer.sv:70-82` |
| UPSAMPLE_L0 / L1（双线性 2×） | **10** | 19,200 / 76,800 | 192,000 + 768,000 = 960,000 | `flow_upsampler.sv:106-227` |
| WARP_L1 / L2（双线性 warp） | **12** | 19,200 / 76,800 | 230,400 + 921,600 = 1,152,000 | `frame_warper.sv:114-235` |
| ACCUM_L1 / L2（base + residual） | **4** | 19,200 / 76,800 | 76,800 + 307,200 = 384,000 | `flow_accumulator.sv:89-127` |
| **合计** | 平均 ≈ **38.9 拍/输出像素** | | **≈ 2.99 M 拍/帧** | |

⇒ 估算吞吐：@100 MHz ≈ **29.9 ms/帧 ≈ 33 FPS**；@实测 7.12 MHz ≈ **420 ms/帧 ≈ 2.4 FPS**。

> ⚠️ **这个估算成立的前提是"所有 `done` 都能正常握手"，而 §5.9 表明当前 HEAD 的金字塔控制 FSM 会死锁**，
> 所以它只是**理论下界**，不是可达吞吐。另外主 FSM **不等** L-K 流水线尾部（`8×WIDTH+12` 拍，见 §5.9 第 7/11 条），
> 实际输出会被截断。
>
> 关键结论：**金字塔版的总吞吐瓶颈不在 L-K 本身，而在 6 个"逐像素走 FSM"的辅助单元**
> （`flow_upsampler` 10 拍/像素、`frame_warper` 12 拍/像素、`flow_accumulator` 4 拍/像素），
> 它们合计占 ~84% 的周期。真正的 L-K 部分（`u_gradient_compute_*`/`u_window_accumulator_*`/`u_flow_solver_*`）
> 反而是**全流水、1 像素/拍**的。这是一条很有价值的优化线索（§9②）。

### 5.7 `unopt` vs `opt` 的具体对比：**opt 不存在，只能对比两份都被标为 "unopt" 的不同设计**

`benchmarks/comparison_template.md` 预留的表（Optimized 列全为 `TBD`）：

| Metric | Unoptimized (Part 1) | Optimized (Part 2) | Improvement |
|---|---|---|---|
| WNS (ns) | -130.409 | TBD | TBD |
| Max Freq (MHz) | ~7.7 | TBD | TBD |
| LUTs | TBD | TBD | TBD |
| DSP48E1 | 222 | TBD | TBD |
| BRAM (36K) | 132 | TBD | TBD |
| Gradient → Flow (cycles) | TBD | TBD | TBD |
| Critical Path Module | flow_solver | TBD | - |
| Critical Path Operation | 32-bit division | TBD | - |
| Logic Levels | ~100+ | TBD | TBD |

**实测数据表（全部来自提交的报告，读取自 `prj/unopt/` 与 `benchmarks/unopt_baseline_20260216/`）**：

| 指标 | 单尺度 `optical_flow_top`（旧，2026-02-07） | 金字塔 `optical_flow_top_pyramidal`（新，2026-02-16） |
|---|---|---|
| 报告文件 | `docs/baseline/reports/utilization_unopt.rpt`、`timing_summary_unopt.rpt` | `prj/unopt/utilization_postroute_unopt.rpt`、`timing_postroute_unopt.rpt` |
| 报告类型 | **仅综合（Synthesized）** | **综合 + 实现（post-route）** |
| Slice LUTs | 3,765 / 63,400 = **5.94%** | 18,816 / 63,400 = **29.68%**（post-route）；综合 27,929 = 44.05% |
| Slice Registers | 129 / 126,800 = **0.10%** | **4,487 / 126,800 = 3.54%**；综合 6,489 = 5.12% |
| Block RAM (RAMB36E1) | **0** / 135 = 0.00% | **132 / 135 = 97.78%** |
| DSP48E1 | **23** / 240 = 9.58% | **222 / 240 = 92.50%** |
| Slice | — | 6,952 / 15,850 = 43.86% |
| Bonded IOB | — | 39 / 210 = 18.57% |
| BUFGCTRL | — | 1 / 32 = 3.13% |
| 时钟 | 10 ns / 100 MHz | 10 ns / 100 MHz |
| **WNS** | **-110.051 ns** | **-130.411 ns**（综合 -103.529 ns） |
| TNS | -3,520.810 ns | -68,003.984 ns |
| 失败端点 | 32 / 345 | **4,692 / 10,784** |
| WHS（保持） | — | **+0.054 ns（通过）** |
| WPWS（脉宽） | — | +4.500 ns（通过） |
| 可达频率（按 10+WNS） | 1/(10+110.051)ns = **8.33 MHz**（README 写 ~8.3，一致） | 1/(10+130.411)ns = **7.12 MHz**（`metrics.txt` 写 ~7.7 MHz —— 那是 `1/130.4ns`，**口径不一致**） |
| 路由 | — | 56,607 逻辑网，35,277 需布线，**全部布通，0 个布线错误**（`prj/unopt/route_status_unopt.rpt`） |

**关键路径（金字塔 post-route，`prj/unopt/timing_postroute_unopt.rpt:202-217`）**：

```
Slack (VIOLATED) : -130.411ns
  Source:      flow_v_reg[15]_i_63/C
  Destination: u_flow_solver_l0/flow_v_reg[9]/D      ← 就在 flow_solver 输出寄存器前
  Data Path Delay: 140.308ns  (logic 91.216ns (65.0%) / route 49.092ns (35.0%))
  Logic Levels:    484  (CARRY4=438, LUT1=2, LUT2=8, LUT3=3, LUT5=31, LUT6=2)
```
- **438 个 CARRY4** 是除法器借位链的直接证据。`benchmarks/unopt_baseline_20260216/logic_levels.txt` 也给出同样的 484 级。
- 第二条违规是 `**async_default**` 组的 `-36.557ns`：`rst_n` → `u_window_accumulator_l0/sum_IxIx_reg[28]/CLR`（`:1746-1760`），
  即**复位网络扇出 3,488，布线延迟 47.3ns**（`:1754`）。这是**复位树没有做同步/扇出控制**导致的，属于另一类问题。
- 综合阶段（`prj/unopt/timing_summary_unopt.rpt:139`）WNS = -103.529 ns，失败端点 96/15,250 ——
  README 说的 "post-route 比 post-synth 差 2-5 ns"（`README.md:347`）在这里实际差了 **27 ns**，README 的估计偏乐观。

**`DESIGN_NOTES.md` 给出的优化路线（尚未实施）**：
1. 用 **Newton-Raphson 倒数**替换除法：`recip = 1/det`（4 级迭代），再 `flow_u = numerator_u * recip`；
   预估每级 4–5 ns，共 20–25 ns（`DESIGN_NOTES.md` 的 "Optimization Strategy" 段）。
2. 若仍不够，**把 25 项加法链拆成 2–3 级流水**（同上；对应 `window_accumulator.sv:150-167`）。
3. 预期：WNS +2~+5 ns @100 MHz；延迟从 4 拍增至 8–10 拍，但时钟快 13 倍 → **净吞吐提升约 13×**。

> ⚠️ 对第 3 点的「13×」要打折：按 §5.6 的估算，金字塔的瓶颈在 warp/upsample/accumulate 这些逐像素 FSM 上（~84% 周期），
> 单纯解决除法只让 L-K 部分提速，**端到端收益远小于 13×**。

### 5.8 ★ 资源数字的可信度警告（必须知道）

提交的资源/时序报告与当前 RTL **在数量级上不自洽**，使用前请自行复核：

1. **两套互相矛盾且标题相同的"unopt 基线"**：
   - `docs/baseline/`（2026-02-07，顶 = `optical_flow_top`）：LUT 3765 / FF 129 / DSP 23 / BRAM 0 / WNS -110.051。
   - `benchmarks/unopt_baseline_20260216/` 与 `prj/unopt/`（2026-02-16，顶 = `optical_flow_top_pyramidal`）：LUT 18816 / FF 4487 / DSP 222 / BRAM 132 / WNS -130.411。
   - 另有一处**层级命名与 RTL 相反**：`benchmarks/unopt_baseline_20260216/metrics.txt` 写
     "3-level pyramid (L0: 320x240, L1: 160x120, L2: 80x60)"，而 `optical_flow_top_pyramidal.sv:45-50` 是
     **L0=80×60（最粗）、L2=320×240（最细）**。
2. **FF 数量与 RTL 声明的存储量严重不符**：仅 `line_buffer_5x5` 就有 15 个实例，
   每个存 `4 × WIDTH × DATA_WIDTH` 位；三层加起来约 **116,480 位**，再加上 3×25×5 个 24 位乘积寄存器、
   3 个 `flow_solver` 的流水寄存器与 19 个 BRAM 的数据通路，**总量会逼近甚至超过器件 126,800 个 FF**，
   而报告只用了 4,487 个 FF；同时报告里 `LUT as Memory = 0`、`LUT as Distributed RAM = 0`
   （`prj/unopt/utilization_postroute_unopt.rpt:37`、`:80`），即这些存储**既没进 FF 也没进 LUTRAM**。
3. **BRAM 位容量也不匹配**：19 个 `frame_memory` 实例按声明深度×位宽求和 = **8,716,800 bit ≈ 8.72 Mbit**
   （`optical_flow_top_pyramidal.sv:197-524` + `frame_memory.sv:16-35`），
   而 132 个 RAMB36E1 只有 `132 × 36,864 = 4,866,048 bit ≈ 4.87 Mbit`，**差 1.79 倍**；
   器件全部 135 个 tile 也只有 4.98 Mbit。仅 L2 那 4 个 16 bit × 76,800 的存储就需要 ≥136 个 RAMB36，
   **单个层级就超过报告值和器件全部 BRAM**。若进一步考虑 RAMB36E1 的离散配置约束
   （8 bit 数据用 x9 模式、16 bit 用 x18 模式，深度分别按 4096 / 2048 计），
   19 个实例合计需要 **≈276 个 RAMB36**（是报告 132 的 2.09 倍、器件 135 的 2.04 倍）。
4. 结论：**`prj/unopt/` 与 `benchmarks/` 的报告很可能不是由当前工作树 RTL 直接复现出来的**
   （不同 revision、或综合时被大量常量剪枝）。`DESIGN_NOTES.md` 声称
   `git checkout <hash> && ./scripts/build.sh unopt impl` 可复现 -130.409 ns，但**本环境无法运行 Vivado 验证（未确认）**。
   ⇒ **引用这些数字时请标注"来自提交的报告，未在本环境复现"**，不要当成当前 RTL 的保证值。
5. **时间戳与文件清单的旁证**：`prj/unopt/utilization_unopt.rpt:4` 的报告日期是 `Mon Feb 16 02:28:33 2026`，
   而最终提交 `cd64c81` 的时间是 `2026-02-16 03:13`，**报告比最终提交早约 45 分钟**；
   `benchmarks/unopt_baseline_20260216/rtl_manifest.txt` 记录的文件行数与当前 HEAD 不同
   （顶层 1424 vs 现在 1432、`flow_upsampler` 224 vs 238、`pyramid_builder` 391 vs 404、`pixel_sequencer` 97 vs 102）。
   ⇒ 报告对应的是**一个未进入当前 HEAD 的中间修订版**。
6. **一个能解释"资源为何这么小"的具体机制**：见 §5.9 第 4 条——顶层把 `pyramid_builder` 的
   `src_read_*` 输入**悬空**，下采样读到的数据是常量，L1/L0 的像素存储与两级 L-K 数据通路
   在综合时会被大面积常量折叠掉。这与 132 BRAM（而不是估算的 ≈276）方向一致，但**具体折叠到哪一步未确认**。

### 5.9 ★ 静态读码发现的确定性缺陷（**未做仿真/工具验证，属静态语义推断**）

以下 8 条是通过通读 RTL 得到的、可复核的功能性缺陷。它们与 §5.8 互相印证：
**当前 HEAD 的金字塔 RTL 在逻辑上跑不完一帧**，所以 §5.6 的吞吐估算只是"理论下界"。

| # | 缺陷 | 证据 | 后果 |
|---|---|---|---|
| 1 | **主 FSM 与子模块互相等待（死锁）** | `pyramid_control_fsm.sv:181-183`（`if (state != next_state) pyramid_build_start <= 1'b1`）→ 需要 `pyramid_build_done` 才能改变 `next_state`（`:95-99`）→ `pyramid_build_done = (builder.state == DONE_ST)`（`pyramid_builder.sv:392`）→ builder 又要 `start` 才能离开 IDLE（`pyramid_builder.sv:103-105`）。**start 只在"状态已经要变"时才发**，而状态要变又要求 done ⇒ 互相等待 | **停在 `BUILD_PYRAMID` 永不前进**；`tb_optical_flow_top_pyramidal.sv:100` 的 `wait(done)` 必然超时（该 TB 有 1,000,000 拍超时看门狗，`:104-107`） |
| 2 | **`frame_warper.done` 恒为 0** | `frame_warper.sv:245`：`done = (state==IDLE) && (out_y == HEIGHT) && (out_x == 0)`；但 `out_y` 最大只到 `HEIGHT-1`（`:223-228`） | 即使进入 `WARP_L1/WARP_L2` 也永远拿不到 `warp_done` ⇒ 再次死锁 |
| 3 | **`flow_upsampler.done` 恒为 0** | `flow_upsampler.sv:236`：`done = (state==IDLE) && (fine_y==FINE_HEIGHT-1) && (fine_x==FINE_WIDTH-1)`；而最后一像素时 `fine_x <= '0` 先于状态切换执行（`:216-217`） | `UPSAMPLE_L0/L1` 永远等不到 `upsample_done` ⇒ 死锁 |
| 4 | **`pyramid_builder` 的下采样读口在顶层悬空** | 全文件 grep `src_read` 在 `optical_flow_top_pyramidal.sv` 中**零命中**；而 `pyramid_builder.sv:55-58` 有 `src_read_data/src_read_addr/src_read_enable` 三个必需端口 | L1/L0 下采样累加的是未驱动值 ⇒ **L1/L0 内容无效**；专门为此写的 `bram_read_arbiter.sv` 反而**从未被例化**（死代码） |
| 5 | **`BUILD_L1/BUILD_L0` 的完成条件不可达** | `pyramid_builder.sv:114` 用 `if (pixel_cnt_l1 == L1_WIDTH*L1_HEIGHT-1 && pyr_l1_we)`；但 `pyr_l1_we` 是寄存器，在写周期为 0、下一拍才为 1，那时计数器已变成 19200 | 主 FSM 的 `BUILD_L1 → BUILD_L0` 迁移条件永不成立 |
| 6 | **BRAM 读路由的状态编码错标** | `optical_flow_top_pyramidal.sv:1298` 把 `4'd3` 当作 `SOLVE_L0`；但枚举里 `3 = UPSAMPLE_L0`、`SOLVE_L0 = 2`（`pyramid_control_fsm.sv:59-72`） | L0 帧存储的读使能在 `SOLVE_L0` 期间不会打开（另 4 个编码 `4'd4/5/8/9` 正确） |
| 7 | **流场写回坐标与数据不同拍** | 写地址用 `seq_lX_coord_valid` 当拍坐标（`optical_flow_top_pyramidal.sv:1200-1212`、`:1240`），但 `flow_valid` 比坐标晚约 `8×WIDTH+12` 拍（`gradient_compute` 的 5×5 行缓冲 + `window_accumulator` 的 5×5 行缓冲）；`flow_solver` 自带的 `pixel_x_out/pixel_y_out` 被**显式悬空**（`:950-951`、`:996-997` 等），`pixel_x_in/pixel_y_in` 全接 `'0` | 流向量会写到错误像素位置 |
| 8 | **`frame_warper` 双线性项位宽不足 + 用了陈旧角点** | `term1..term4` 声明为 `signed [PIXEL_WIDTH+7-1:0]` = **15 位**（`frame_warper.sv:88`），但 `(128-wx)*(128-wy)*corner` 可达 `128×128×255 = 4,177,920`（**22 位**）⇒ 高 9 位被丢；随后 `interp_result[PIXEL_WIDTH+14-1:14]`（`:205`）取值无意义。另外第 4 个角点在同一拍写入同一拍使用（`:169` vs `:200`），读到的是上一个像素的 `corners[3]` | warp 结果数值错误 |

**另有 2 类"可以编译但有隐患"的写法**：
- `next_state` 在 `flow_accumulator.sv:130`、`flow_upsampler.sv:230`、`pixel_sequencer.sv:91` 的 `default:` 分支里被赋值，
  但**这三个文件从未声明 `next_state`**；而 `pyramid_control_fsm.sv:242`、`pyramid_builder.sv:264`/`:380`
  是在**时序块里**给一个已有 `next_state` 赋值（与组合块形成双驱动）。综合工具的具体反应**未确认**。
- `flow_upsampler.sv:169` 在 `always_ff` 里声明局部变量 `u_interp_comb`，用非阻塞赋值后立刻读取（`:181-201` vs `:205`），
  按 NBA 语义读到的是**上一个像素的值**。

> ⚠️ **对使用者的建议**：如果你打算借用这套 RTL 做参考，**不要把它当作"已验证可工作的设计"**——
> 它是一个**时序/资源研究**性质的工程：真正被验证过的只有 Python 侧（§6.3），
> RTL 侧的 testbench 只检查"FSM 到没到 DONE"，而 FSM 在静态语义下根本到不了 DONE（缺陷 1–3）。
> 可复用的部分是 **§9② 的模块级套路**（行缓冲、BRAM 模板、DSP 标注、定点约定），不是顶层的控制逻辑。

---

## 6. Python 参考实现与验证方法（重点）

### 6.1 golden reference 是什么？—— **纯 NumPy/SciPy 的浮点实现，不是 OpenCV 对比**

- **核心**：`python/lucas_kanade_core.py` 的 `compute_gradients()`（`:15-45`）与
  `lucas_kanade_from_gradients()`（`:73-135`）。全程 `np.float32`，**逐像素 Python 双重 for 循环**
  （`:107-108`）——所以对 320×240 的 13 个模式跑一遍很慢，但**逐像素语义与硬件 1:1 对应**，这正是它能当 golden model 的原因。
- **OpenCV 的角色**：只用于**生成测试图**（`generate_test_suite.py:27` 的 `import cv2`；
  `apply_motion_opencv()` 在 `:165`），**不参与 L-K 求解**。
  没有任何 `cv2.calcOpticalFlowPyrLK` / `cv2.goodFeaturesToTrack` 的调用（全仓库 grep 未发现）。
  ⇒ 所谓"OpenCV 对比"在 README/文档里没有出现，也**不存在**。
- **金字塔参考**：`python/lucas_kanade_pyramidal.py`，用 `scipy.ndimage.gaussian_filter` + `map_coordinates`
  实现高斯金字塔与双线性 warp（`:37-56`、`:90-96`），`num_levels=3, window_size=5, num_iterations=3`。

### 6.2 测试向量如何生成

| 生成器 | 素材 | 输出 | 用途 |
|---|---|---|---|
| `python/generate_test_suite.py` | `mountain_texture.jpg` → 灰度 → resize 到 320×240（`:140-162`）；用 **OpenCV** 做平移/旋转/缩放（`:165`） | 每个模式一个目录：`frame_00.bin`/`frame_01.bin`（`:259-261`）、`frame_00.mem`/`frame_01.mem`（`:263-271`）、`frame_00/01.png` + `comparison.png`（`:274-280`）、`metadata.json`（含 `motion_parameters.dx/dy/...`，`:256`）；外加 `suite_index.json`（`:327`） | **Python 验证套件**（13 模式） |
| `python/generate_test_frames_natural.py` | 同一张图（`CACHED_IMAGE` 在 `:16`，`generate_test_suite.py:34` 重复引用同一路径） | 同样四种格式，但只生成**一个**模式（`--displacement-x/y`），位移用 `scipy.ndimage.shift(order=1, mode="constant", cval=128)`（`:72`）；`frame_1 = apply_motion(frame_0, ...)`（`:106`），`.mem` 由 `f"{val:02x}\n"` 逐行写出（`:113-119`） | **RTL 仿真**（`run_sim.sh:20-22` 固定传 `--displacement-x 2`） |

**13 个模式的真实参数（`generate_test_suite.py:57-137`）**：

| 模式 | dx | dy | rotation | scale |
|---|---|---|---|---|
| `translate_small` | 0.5 | 0.5 | 0 | 1 |
| `translate_medium` | 2.0 | 0 | 0 | 1 |
| `translate_large` | 15.0 | 0 | 0 | 1 |
| `translate_vertical` | 0 | 10.0 | 0 | 1 |
| `translate_diagonal` | 10.0 | 10.0 | 0 | 1 |
| `rotate_small` | 0 | 0 | **2.0°** | 1 |
| `rotate_medium` | 0 | 0 | **5.0°** | 1 |
| `rotate_large` | 0 | 0 | **15.0°** | 1 |
| `zoom_in` | 0 | 0 | 0 | **1.1** |
| `zoom_out` | 0 | 0 | 0 | **0.9** |
| `translate_rotate` | 5.0 | 5.0 | 3.0° | 1 |
| `no_motion` | 0 | 0 | 0 | 1 |
| `translate_extreme` | 30.0 | 20.0 | 0 | 1 |

> ⚠️ `python/README.md:123-127` 把参数写错了：`rotate_large` 写成 **10°**（实际 `generate_test_suite.py:100-104` 是 **15°**）、
> zoom 写成 **5%**（实际 `:106-115` 是 **1.1 / 0.9，即 ±10%**）、`translate_vertical` 写成 **5px**（实际 `:77-82` 是 **10px**）；
> `python/README.md:129` 还说输出到 `python/test_patterns/`，实际是 `python/test_suite/`（`generate_test_suite.py:37`）。
> **一律以 `generate_test_suite.py` 的代码为准。**

### 6.3 testbench 如何比对？**—— 没有逐位/逐像素比对；只有统计阈值判定 + 人工看图**

这是本文件最需要说清楚的一点。

**`tb/tb_optical_flow_top.sv`（单尺度）的判定逻辑（`:42-45`、`:291-326`）**：
```systemverilog
localparam real EXPECTED_U_MAGNITUDE = 0.5;   // 只要求幅度 ≥ 0.5 px
localparam real MAGNITUDE_TOLERANCE  = 0.5;   // 声明了但未使用
localparam real DIRECTION_TOLERANCE  = 30.0;  // 声明了但未使用
...
magnitude_ok = flow_magnitude >= EXPECTED_U_MAGNITUDE;
direction_ok = abs_mean_v < 0.5;              // 只要求竖直分量 < 0.5 px
if (magnitude_ok && direction_ok) → "*** TEST PASSED ***"
```
- 统计区域：`x[55:85], y[105:135]`（`:48-51`），对这些像素求 `mean/std`（`:263-271`）。
- **没有容差概念上的"逐像素比对"**：不读 Python 的 `flow_u.bin`，不读 `flow_field_python.txt`，
  不比较任何逐像素数值，**也不检查流向的符号**（`magnitude` 是 `$sqrt(u²+v²)`，恒为正）。
  ⇒ README 日志里的 `u = -0.765` 而 ground truth 是 `+2.000`（`README.md:517-519`），
  **方向完全相反，测试依然 PASS**（`README.md:525`）。这是判定逻辑的实质缺陷。
- TB 会导出 `flow_field.txt`（`:340-361`），**但没有自动比对**，需要人工跑
  `scripts/visualize_flow.py flow_field_rtl.txt`（README `:551-566`）去看图。

**`tb/tb_optical_flow_top_pyramidal.sv`（金字塔）的判定（`:110-115`）**：
```systemverilog
if (dut.u_fsm.state == dut.u_fsm.DONE_ST)  → "*** TEST PASSED: FSM reached DONE ***"
```
- **只检查 FSM 是否到达 `DONE_ST`**，`flow_u()`/`flow_v()` 端口在实例化时**悬空**（`:40-42`）。
- ⇒ 金字塔版**没有任何数值验证**。

**Python 侧 `optical_flow_verifier.py` 实际"验证"的是什么**（`:211-312`）：
它把 **Python 单尺度** 和 **Python 金字塔** 的结果，和**解析 ground truth（`metadata.json` 里的 `dx/dy`）** 比较，
按 `verification_config.yaml` 的 MAE 阈值分类（`:175-203`）：
```
translation: mae_pass 0.5 / mae_warning 2.0
rotation:    1.0 / 3.0
zoom:        1.0 / 3.0
combined:    2.0 / 5.0
```
分类用 `mae_max = max(mae_u, mae_v)`（`:196-203`）。
测试区域：平移模式取去掉 10px 边框的全图；旋转/缩放/组合取中心 80×80（`config["test_region"]["center_crop"]=80`，`:96-138`）。
指标定义见 `flow_metrics.py`（MAE `:14-40`、RMSE `:43-70`、EPE `:73-103`、AAE 用 `(u,v,1)` 三维夹角 `:106-163`）。

> **结论：本仓库不存在 RTL ↔ Python 的自动化数值对照。**
> `python/README.md:286-302` 的 "RTL Verification" 章节提到的
> `python/generate_rtl_testvectors.py` 与 `python/compare_rtl_results.py` **两个文件都不存在**
> （已用 `ls` 确认），该章节末尾还留着 `<!-- TODO: Add RTL comparison scripts -->`。
> README 正文里也没有任何"容差"数字——它引用的是**人工跑出来的单一场景均值**
> （`README.md:388-395`：Ground Truth 2.00 / Python float32 1.34 / RTL S8.7 0.76 / Pyramidal 0.53）。
> 这四个数字**无法从仓库中任何脚本自动复现**（未确认其生成方式）。

**CI 到底跑了什么**（`verify_optical_flow.yml:33-45`）：
```
generate_test_suite.py  →  optical_flow_verifier.py --compare-baseline --no-visualizations --regression-threshold 10.0
```
即：生成 13 个模式 → 跑 Python 单尺度 + Python 金字塔 → 和 `verification_baseline.json` 比 MAE/EPE 的相对变化，
超过 10% 阈值（`mae_u/mae_v/epe`）或 15%（`aae`，见 `verification_config.yaml` 的 `regression.metric_thresholds`）就失败（`optical_flow_verifier.py:586-720`）。
**RTL 一行都没跑。**

---

## 7. 结果与基准

### 7.1 `python/verification_results.md`（Python 精度，可复现）

完整表格（`python/verification_results.md`，与 `verification_results.json` 一致）：

| Pattern | GT (u,v) | 单尺度 MAE(u/v) | 单尺度 EPE | 单尺度状态 | 金字塔 MAE(u/v) | 金字塔 EPE | 金字塔状态 |
|---|---|---|---|---|---|---|---|
| translate_small | (0.5, 0.5) | 0.265 / 0.245 | 0.391 | **Pass** | 0.638 / 0.628 | 0.997 | Warning |
| translate_medium | (2.0, 0.0) | 0.886 / 0.471 | 1.093 | Warning | 0.550 / 0.403 | 0.742 | Warning |
| translate_large | (15.0, 0.0) | 14.735 / 2.280 | 15.120 | Fail | 6.039 / 5.092 | 8.934 | Fail |
| translate_vertical | (0.0, 10.0) | 2.221 / 8.452 | 9.166 | Fail | 5.663 / 2.562 | 6.793 | Fail |
| translate_diagonal | (10.0, 10.0) | 9.531 / 8.685 | 13.309 | Fail | 7.774 / 4.803 | 10.223 | Fail |
| rotate_small | (0,0) | 1.084 / 1.076 | 1.680 | Warning | 0.754 / 0.826 | 1.227 | **Pass** |
| rotate_medium | (0,0) | 1.285 / 1.391 | 2.092 | Warning | 1.771 / 1.796 | 2.738 | Warning |
| rotate_large | (0,0) | 1.243 / 1.603 | 2.257 | Warning | 5.208 / 5.304 | 8.068 | Fail |
| zoom_in | (0,0) | 1.346 / 1.530 | 2.271 | Warning | 2.013 / 2.038 | 3.114 | Warning |
| zoom_out | (0,0) | 1.362 / 1.538 | 2.299 | Warning | 2.067 / 2.166 | 3.268 | Warning |
| translate_rotate | (5.0, 5.0) | 4.589 / 4.834 | 6.881 | Warning | 1.107 / 1.176 | 1.764 | **Pass** |
| no_motion | (0,0) | 0.000 / 0.000 | 0.000 | **Pass** | 0.000 / 0.000 | 0.000 | **Pass** |
| translate_extreme | (30.0, 20.0) | 29.375 / 18.685 | 35.405 | Fail | 36.122 / 22.051 | 46.567 | Fail |

**能读出的结论**：
1. **单尺度在亚像素最优**（`translate_small` EPE 0.391 px），这符合 L-K 的理论预期。
2. **金字塔在大位移/组合运动上收益明显**：`translate_large` EPE 15.120 → 8.934；`translate_rotate` 6.881 → 1.764。
3. **金字塔在极小位移上反而更差**（`translate_small` 0.391 → 0.997），因为最粗层（80×60）已经把 0.5px 运动淹没在降采样里。
4. **旋转/缩放整体都很差**（Warning/Fail 为主）：这是 L-K「窗口内平移一致」假设的必然结果，不是实现 bug。
5. **`translate_extreme`（30,20）两者都 Fail 且金字塔更差**（EPE 46.6 vs 35.4）——金字塔的 warp 误差在大位移下会放大。

> ⚠️ 这些结论**只描述 Python 实现**，与 RTL 无关。

### 7.2 `benchmarks/` 内容与可读结论

| 文件 | 内容 |
|---|---|
| `unopt_baseline_20260216/metrics.txt` | WNS -130.409 / TNS -67998.801 / 目标 100 MHz / 可达 ~7.7 MHz / DSP 222 / BRAM 132 / 瓶颈 = 32 位组合除法 / 逻辑级数 ~100+ |
| `.../DESIGN_NOTES.md` | 根因分析（组合除法 → 484 级 LUT 级联）+ 3 条优化策略（§5.7）+ 复现命令 |
| `.../logic_levels.txt` | 关键路径展开：Data Path 140.308ns，logic 91.216ns (65%) / route 49.092ns (35%)，**484 级**（CARRY4=438, LUT5=31, LUT2=8, LUT3=3, LUT1=2, LUT6=2）；保持路径 0.407ns/1 级 |
| `.../critical_path_verbose.txt` | 150 行，同样的 set/hold/async 三类路径全展开 |
| `.../critical_path_detail.txt` | **空文件（0 字节）** |
| `.../rtl_manifest.txt` | 13 个 RTL 文件的 SHA256 + 行数 + git status；显示 `flow_solver.sv`/`gradient_compute.sv` 当时是 `MM`（已暂存又有未暂存修改），9 个文件是 `??`（未跟踪）——**基线快照本身是在"未提交"状态下抓的** |
| `.../README.md` | Part 1 快速摘要，指向 Part 2 优化 |
| `.../timing_summary_excerpt.txt` | post-route timing 报告的头部 + Design Timing Summary 摘录 |
| `.../timing_*.rpt` / `utilization_*.rpt` / `route_status_unopt.rpt` | `prj/unopt/` 同名报告的副本 |
| `comparison_template.md` | 未使用的对比模板（§5.7） |

**可读结论**：这个 benchmark 目录的**唯一核心发现**是「**组合 32 位除法是唯一的关键路径**」，
并且给出了两级证据（逻辑级数 484 / CARRY4 438，以及改动前后的路径端点 `u_flow_solver_l0/flow_v_reg[9]/D`）。
`README.md:388-395` 的 "RTL 0.76 px vs Python 1.34 px" 表**不在** `benchmarks/` 里，
而是从单尺度 TB 的一次运行手工抄来的，**不可自动复现**。

### 7.3 时钟频率与吞吐汇总表

| 配置 | 目标时钟 | 实测 WNS | 可达频率 | 单帧周期 | 吞吐 @可达频率 | 来源 |
|---|---|---|---|---|---|---|
| 单尺度 `optical_flow_top`（综合） | 100 MHz | -110.051 ns | **8.33 MHz** | ~79,370 | **≈ 105 FPS @320×240** | `docs/baseline/README.md` |
| 金字塔 `..._pyramidal`（post-route） | 100 MHz | -130.411 ns | **7.12 MHz** | ≈ 2.99 M（估算） | **≈ 2.4 FPS**（估算） | `prj/unopt/timing_postroute_unopt.rpt:139` + §5.6 |
| 金字塔 @未优化但假设能到 100 MHz | 100 MHz | — | 100 MHz（假设） | ≈ 2.99 M | **≈ 33 FPS**（估算） | 同上 |

> 注 1：单尺度的 105 FPS 是按 79,370 拍/帧 + 8.33 MHz 推出的；`docs/baseline/README.md` 只给了 WNS 和
> 资源，**没有给 FPS**。金字塔没有任何实测吞吐日志。**上表的 FPS 全部是本文的推导值，不是仓库数据。**
> 注 2：单尺度那一行的 79,370 拍来自 README 的仿真日志（§5.6），**只有一个场景、无回归**；
> 金字塔那一行的 2.99 M 拍是 FSM 状态数推算，且**当前 FSM 死锁（§5.9），所以 2.4 / 33 FPS 都不可达**。
> 注 3：仓库里**没有任何一处的 FPS 是实测出来的**。若你要用这些数字做选型，请先自己跑一次仿真或上板。

### 7.4 `results/` 与 `docs/baseline/`

- `results/flow_visualization.png`：唯一的图，2,049,848 字节，是 `scripts/visualize_flow.py` 生成的
  4 面板诊断图（quiver / 幅度 / 直方图 / 误差图），用 2px 右移场景。README `:536-547` 有解读。
  ⚠️ README 说直方图平均 ~1.8 px（`:544`），但正文表格（`:392`）与 TB 日志（`:517`）都是 **u = -0.765**，
  三处数字互不一致。
- `docs/baseline/`：**旧的单尺度**报告（2026-02-07，顶 = `optical_flow_top`），
  数字见 §5.7 表的左列。它自己的 `README.md` 也承认 "Design features massive combination paths and no pipelining"，
  并说 "Low FF utilization coincides with no pipelining" —— 但 129 个 FF 的真正原因更可能是
  §5.1 说的**仿真专用帧缓冲被综合剪枝**（未确认，但逻辑上更自洽）。
  `docs/baseline/reports/critical_paths_detail.txt` 的路径是
  `u_window_accumulator/sum_IyIy0__22/CLK` → `u_flow_solver/flow_u_reg[10]/D`，与 2 月 16 日那次的端点不同。

---

## 8. 局限与坑

**算法层面**
1. **只支持平移**：窗口内假设 `(u,v)` 恒定。旋转/缩放会产生空间变化的流场，实测 MAE 立刻升到 1–2 px 甚至 Fail（§7.1）。
2. **孔径问题（aperture problem）**：只有单一朝向边缘的窗口，`A` 秩亏（`det≈0`），法向流可解、切向流不可解。
   本项目对它的处理只有「`|det| ≤ 1000` → 输出 0」（`flow_solver.sv:123`），**没有任何纹理度加权、没有置信度输出**。
3. **大位移失效**：5×5 窗口 + 单次求解，位移超过 ~2 px 就迅速退化；RTL 还额外把结果 clamp 到 **±8 px**
   （`flow_solver.sv:134-144`），超过直接饱和。README 自认 "suitable for motions < 5 pixels"（`:62`）。
4. **光照变化敏感**：亮度恒定假设被破坏时，`I_t` 里混入光照项，流场会被系统性污染。本项目**完全没有光照归一化/鲁棒核**。
5. **RTL 无多尺度迭代**：金字塔版每层只解一次，没有 Python 那种 `num_iterations=3` 的迭代精化（§4.7）。
6. **`DET_THRESHOLD=1000` 是绝对门限**，在低对比度场景会把大片有效纹理判成"不可解"并输出零向量（§4.5）。

**硬件/实现层面**
7. **★ 控制 FSM 死锁：当前 HEAD 的 RTL 在逻辑上跑不完一帧。** 见 §5.9 的 8 条清单，其中前 3 条
   （主 FSM 与子模块互相等待、`frame_warper.done` 恒 0、`flow_upsampler.done` 恒 0）任意一条都足以让流程卡死；
   第 4 条（`pyramid_builder` 的下采样读口在顶层悬空）还会让 L1/L0 的内容直接变成垃圾。
   **⇒ 拿这份 RTL 当"可工作参考设计"会踩坑：真正被验证过的只有 Python 侧。**
8. **组合 32 位除法**：484 级逻辑、438 个 CARRY4、WNS -130ns，是整个设计的性能天花板（§5.5/§5.7）。
9. **`prod_det1[31:0]` 切片**：64 位乘积直接取低 32 位，可能回绕出错误符号（`flow_solver.sv:116`）。
10. **梯度与时间梯度的采样点错开 1 像素**（§5.2），`I_x/I_y`（5×5 中心 3×3 的中心）与 `I_t`（3×3 的 `[1][1]`）不是同一像素。
11. **`window_accumulator` 的 25 项组合加法链无流水**（`window_accumulator.sv:150-167`），
    文件头注释还停留在"单周期完成"的旧描述。
12. **`frame_warper` 里是 3 个 8×8 乘法 + 一个右移 14 位的组合 MAC**（`frame_warper.sv:205-215`），
    也是关键路径候选；`interp_result[PIXEL_WIDTH+14-1:14]` 是**截断**而非四舍五入。
    更严重的是 `term1..4` 只有 **15 位**而乘积需要 22 位（§5.9 第 8 条），**warp 数值本身就是错的**。
13. **`flow_accumulator` 直接 `base + residual` 相加，无饱和**（`flow_accumulator.sv:109-110`），
    在多层累加下可能溢出 S8.7 的 16 位。
14. **复位树问题**：`rst_n` 扇出 3,488，异步复位恢复检查 -36.557ns（§5.7），
    没有做复位同步器或复位扇出复制。
15. **无 ready/valid 反压**：所有流式接口是单向的，靠 `start/busy/done` 粗同步；
    而 `start` 只在"状态即将改变"时脉冲（`pyramid_control_fsm.sv:181-183`），**这既不安全也不工作**（§5.9 第 1 条）。
16. **未声明的 `next_state` 标识符**（`flow_accumulator.sv:130`、`flow_upsampler.sv:230`、`pixel_sequencer.sv:91`）
    与**时序块里给 `next_state` 赋值造成的双驱动**（`pyramid_control_fsm.sv:242`、`pyramid_builder.sv:264`/`:380`）——
    属可疑写法，工具是否报错未确认。
17. **死代码**：`bram_read_arbiter.sv` 从未被例化；`pyramid_builder` 的 `block_pixel_cnt`/`block_avg`
    声明后未参与逻辑（`pyramid_builder.sv:82-83`）。

**系统/工程层面**
18. **图像源是静态测试图**，不是真实相机。没有 MIPI/DVP/HDMI 输入，**没有任何引脚约束（PACKAGE_PIN）**，
    `constraints/*.xdc` 只有时钟和 I/O delay（§2.3）⇒ **不能上板运行**。
19. **只能处理两帧**：所有场景都是「前一帧 + 当前帧」，没有连续视频流的状态机，
    没有帧间流水（一帧算完才接受下一帧的 `start`）。
20. **分辨率硬编码 320×240**：`convert_frames.py:36-37` 写死；`FLOW_WIDTH/FRAC_BITS/ACCUM_WIDTH` 等
    在顶层是参数但 `line_buffer_5x5` 的窗口尺寸是硬编码 `[5][5]`（`line_buffer_5x5.sv:30`），改窗口要改代码。
21. **RTL 侧零自动化验证**：TB 只看"幅度 ≥ 0.5 且 |v| < 0.5"，不校验符号、不比对 Python、不检查 `flow_valid` 覆盖率
    （§6.3）；CI 不跑 HDL（§2.6）。**结果是 §5.9 那 8 个足以让设计完全不可用的缺陷一个都没被发现。**
22. **金字塔 TB 的 curr/prev 接反**（§5.1），且该 TB 不验证数值，所以 bug 被掩盖。
23. **RAM 资源吃紧**：BRAM 97.78%（132/135）、DSP 92.50%（222/240）——**几乎没有余量**给更大的窗口或更多层。
    （且按当前 RTL 估算实际需要 ≈276 个 RAMB36，**超过器件总量**，见 §5.8。）
24. **README/文档与代码大面积脱节**：不存在的 `rtl/opt`、`prj/opt`、`python/lucas_kanade.py`、
    `python/generate_test_frames.py`、`generate_rtl_testvectors.py`、`compare_rtl_results.py`、
    `visualize_flow.py --compare`、`python/tests/`；层级命名 L0/L2 与 RTL 相反；
    多个数字在三处互相矛盾（§1、§2、§5.8、§7.4）。**以 RTL 代码为准，文档仅作线索。**

---

## 9. 对「二维动态障碍环境模拟器 + 现实小车 + 视觉」项目的可借鉴点

### ① 用光流估计小车自身运动，修正 `v_rel = v_obs − v_player`

- 本项目的核心价值在于：**它把 L-K 的全套定点处理链路真的做出来了**，而不只是论文公式。
  可以直接借用它的**数值口径**：`I_t = I_prev − I_curr`、Sobel÷8 的平均帧梯度、`A`/`b` 的符号约定
  （`flow_solver.sv:83-92` 与 `lucas_kanade_core.py:115-125` 必须一起看，符号是配平的）。
- **对 `v_rel` 的直接用法（把光流与你的公式对齐）**：
  设相机装在小车上，则**背景像素的光流 `f_bg` 就是相机自身运动的负向像移**，即 `f_bg ≈ −v_player`（乘以像素/米标定系数）；
  而**障碍物像素的光流 `f_obs` 恰好正比于相对运动** `v_rel = v_obs − v_player`。
  所以工程上分两步：
  1. 用**全图（或背景区域）光流的鲁棒中位数**估计 `f_bg`，从而得到 `v_player ≈ −f_bg`；
  2. 在剔除背景的像素上取 `f_obs`，则 `v_rel ≈ f_obs`（两者已在同一参考系），障碍的绝对速度 `v_obs = v_player + f_obs`。
  这正是把「相对运动」变成「光流场的分层」：**背景层 = ego-motion，前景层 = `v_rel`**。
  **注意**：本项目的 RTL **只输出稠密流场，不做任何"背景/前景分离"**——分层要你自己做
  （建议在 Python 侧用中位数/RANSAC 先分离，再决定哪些像素真的值得上 FPGA）。
- **先在你现有的 Python 内核里跑通的捷径**：`python/lucas_kanade_core.py` 与 `lucas_kanade_pyramidal.py` 是**可直接 import 的纯 NumPy 实现**
  （`pyproject.toml:60-63` 把 `python*` 作为包），可以立刻当 baseline，
  再用它去标定「小车真实运动 → 光流 → `v_player`」这条链的误差量级与所需帧率。

### ② 定点化 + 流水线 + 行缓冲的工程套路（可直接抄的结构）

| 套路 | 本项目的具体做法 | 文件:行 |
|---|---|---|
| 行缓冲 5×5 窗口 | 4 条整行 + 5 级移位寄存器，组合多路选择输出 5×5 | `line_buffer_5x5.sv:37-43`、`:101-139` |
| 两级窗口嵌套 | 5×5 窗口再取中心 3×3 做 Sobel（省一次卷积硬件） | `gradient_compute.sv:90-97` |
| 定点统一格式 | 全程 **S8.7**，`<<<7` 缩放、`/det`、clamp `±1024` | `flow_solver.sv:22`、`:127-144` |
| 乘法打 DSP | `(* use_dsp = "yes" *)` 放在乘积寄存器上，实测推断出 222 个 DSP48E1 | `window_accumulator.sv:102-106`、`flow_solver.sv:55-57` |
| BRAM 模板 | `(* ram_style = "block" *)` + 单写口/单读口（寄存输出）的 25 行模板，参数化 DEPTH/位宽 | `frame_memory.sv:35-49` |
| 共享读口仲裁 | 三个消费者（builder/warper/sequencer）用静态优先级复用同一 BRAM 读口 | `bram_read_arbiter.sv:42-57`（**但顶层从未例化它，是死代码**，见 §5.9） |
| 逐像素 FSM 而不是全流水 | 对非关键路径模块（warp/upsample/accumulate）用"读-等-算-写"小 FSM，省资源 | `frame_warper.sv:36-43`、`flow_upsampler.sv:44-50` |
| 分层总控 FSM | 12 状态把"建塔/求解/上采样/warp/累加/完成"串起来，每个子模块自己 done | `pyramid_control_fsm.sv:59-72` |

**反面教材（同样有价值）**：
- 不要在末级做**组合除法**（`flow_solver.sv:130-131`）——这是 484 级关键路径的成因。用 N-R 倒数或
  **查表 + 1 次 Newton 迭代**（`DESIGN_NOTES.md` 的计划），或干脆把 `1/det` 交给 DSP 做定点乘法。
- 复位不要直接扇出给几千个寄存器（§8 第 14 条），用**同步复位 + 局部复位树**。
- 别把 25 项加法写成 `for` 顺序链（`window_accumulator.sv:158-166`），用**平衡加法树**并切流水。
- **★ 最值得吸取的教训：不要用"`start` 只在状态即将变化时脉冲"这种控制风格。**
  它让"发 start"依赖"收到 done"、而"收到 done"又依赖"发 start"，形成死锁（§5.9 第 1 条）。
  正确做法是**每个状态进入时无条件发一拍 start**，或直接用 `valid/ready` 握手。
- **别把 `done` 写成 `(state==IDLE) && (counter==上限)` 这种组合条件**（`frame_warper.sv:245`、
  `flow_upsampler.sv:236`）——计数器在返回 IDLE 的那一拍已经被清零/回绕，条件永远不成立。
  用**进入 IDLE 时的寄存脉冲**。
- **模块的可选端口一定要在顶层接上或显式置常量**。`pyramid_builder` 的 `src_read_*` 三个输入在顶层
  悬空（§5.9 第 4 条），导致整个 L1/L0 数据通路失效——而综合工具**只会给个 warning**。
- **给模块写一个"到 DONE 就 PASS"的 testbench 等于没写验证**（`tb_optical_flow_top_pyramidal.sv:111`）。
  至少要比对输出数值与 golden model。

### ③ Python golden model ↔ HDL testbench 的验证范式（含与你的 `FpgaPredictionReference` 的对比与融合建议）

**本项目的范式（值得学）**：
1. **算法先用 NumPy 写一遍，且写得"逐像素、可映射"**（`lucas_kanade_core.py:107-135` 的双重 for 循环）。
   这比用 `cv2.calcOpticalFlowPyrLK` 好得多，因为硬件无法对齐 OpenCV 的内部实现选择。
2. **同一份数学在两侧严格同参**：Sobel 核都除以 8、都用「平均帧」做空间梯度、`I_t` 都用 `I_prev − I_curr`。
   两边注释互相引用（`gradient_compute.sv:11-13` vs `lucas_kanade_core.py:20-23`）。
3. **量化步长明确**：Python 用 `float32`，RTL 用 S8.7，并在文档里给出 `1/128` 的换算
   （`tb/tb_optical_flow_top.sv:111-115`）。
4. **基线回归**：`verification_baseline.json` + 10%/15% 相对阈值（`optical_flow_verifier.py:586-720`），
   提交进仓库、CI 强制跑。

**本项目缺失的（正是你要补的）**：
- ❌ **没有 test vector 导出器**（`generate_rtl_testvectors.py` 不存在）；
- ❌ **没有逐像素比对器**（`compare_rtl_results.py` 不存在）；
- ❌ **没有容差定义**（README 提到的 0.76 vs 1.34 px 是手工抄的）；
- ❌ **CI 不跑 RTL**；
- ❌ TB 只看幅度不看符号（`tb_optical_flow_top.sv:307`）。

**与你已有的 `FpgaPredictionReference`（`bullet_sim/fpga/prediction_block.py:173-215`）的对比与融合建议**：

| 维度 | `optical-flow-fpga` | 你的 `FpgaPredictionReference` |
|---|---|---|
| 定位 | HDL 的 float golden model | **FPGA 输入/输出字节帧**的纯函数软件参考（`predict(state_frame) -> bytes`，`:201-215`） |
| 接口 | NumPy 数组 in / 数组 out | **bytes in / bytes out**（`decode_state` → 预测 → `encode_prediction_frame`） |
| 决定性 | float32 计算，跨平台可能有末位差 | **逐位确定**（"Same input, same output, always"，`:176-178`），天然适合 HDL 逐位对齐 |
| 自述状态 | — | `status = SOFTWARE_REFERENCE_ONLY`（`:185`），即**还没有 HDL 对手方** |

**融合建议（按优先级）**：
1. **沿用你的字节级接口哲学，但把比对粒度做成"逐位 + 分层容差"两档**：
   - 位精确通路（你的 `encode_prediction_frame` / 状态编解码）：用**逐位相等**断言，任何 1 bit 差异即失败；
   - 浮点/定点算法通路（光流这种）：用**分层容差**——先比 `flow_valid` 位置（必须逐像素一致），
     再比定点原始值（允许 ±1 LSB，吸收 `/` 与 `>>` 的截断差异），最后才比换算后的物理量。
   > 本项目最大的教训是：**它连"位置一致性"都没检查**（`73,289` vs 理论 `72,384` 的差异一直没人发现，§5.6）。
2. **把 `FpgaPredictionReference` 的"纯函数性"扩展到光流**：把 "读 `.mem` → 跑 L-K → 导出流场" 做成
   一个 `flow_reference(frame_prev_bytes, frame_curr_bytes) -> flow_bytes` 的纯函数（无状态、无全局变量），
   这样 HDL TB 和 Python 都调它，天然可回归。
3. **test vector 生成走"三件套"**：`.mem`（给 `$readmemh`）+ `.bin`（给 NumPy）+ `metadata.json`（含 ground truth 与
   量化参数）。本项目已经做到前两个（`generate_test_suite.py:259-271`），**只差把 RTL 输出也纳入 metadata 驱动的比对**。
4. **加一个"符号/方向"断言**：本项目因此漏掉了 RTL 流向完全相反还能 PASS 的问题（`README.md:517-525`）。
   你的 `v_rel` 有明确符号语义，务必断言 `sign(u)` 而不仅是 `|u|`。
5. **CI 分层**：Python-only 的快检查（每次 PR）+ RTL 仿真检查（Verilator/Icarus，允许更长的 path filter）。
   本项目 CI 是"Python 全过、RTL 全无"，你要避免复制这个盲区。

### ④ 资源/吞吐的实测数字作为选型参考

| 参考量 | 提交报告中的值（Nexys A7-100T, `xc7a100tcsg324-1`, -1 speed grade）——**均未在本环境复现，见 §5.8** |
|---|---|
| 单尺度（`optical_flow_top`，旧报告） | 3,765 LUT / 129 FF / 23 DSP / 0 BRAM ⚠️ **极可能是仿真专用帧缓冲被综合剪枝后的退化数字**（§5.1） |
| 3 层金字塔 + 19 块帧存 BRAM | **18,816 LUT (29.7%) / 4,487 FF (3.5%) / 222 DSP (92.5%) / 132 RAMB36 (97.8%)** |
| 单尺度像素率 | **1 像素/时钟**（全流水） |
| 金字塔整体像素率 | 约 **1 像素 / 39 时钟**（≈2.99 M 拍 / 76,800 像素，估算）——瓶颈在 warp(12) + upsample(10) + accumulate(4) |
| 关键路径 | 组合 32 位除法，**140.3 ns**（其中布线 49.1 ns），484 逻辑级 |
| 时钟可达 | 100 MHz 目标 → 报告值 **7.12 MHz**（WNS -130.411 ns） |
| **给小车项目的启示** | ①Artix-7 这一档器件做 320×240 的**3 层金字塔**会把 DSP/BRAM 吃到 90% 以上——**没有余量**（而且按当前 RTL 估算 BRAM 需求 ≈276 tile，**根本放不下**，§5.8）；<br>②**不要**据此认为"单尺度很便宜"：那 23 DSP / 0 BRAM 是剪枝后的退化值；真实单尺度至少要 15 个 5×5 行缓冲（约 11.6 万存储位）+ 125 个 DSP 乘法（§5.4、§5.8）——**务必自己重跑一次综合**；<br>③**不要用"逐像素 FSM"去做 warp/upsample**，那是吞吐杀手（占 ~84% 周期）；<br>④组合除法必须换掉，否则 ~7 MHz 的时钟基本没法用于实时小车；<br>⑤选型时按「分辨率 × 层数 × 窗口」预留 2× 资源余量，因为你还要加背景分离/阈值/输出接口。 |

### ⑤ **诚实提醒**：哪些假设在真实小车上会失效

| 本项目的假设 | 真实二维小车上会发生什么 | 建议 |
|---|---|---|
| **小位移**（< 2–5 px/帧） | 小车快速转向/加速时，画面位移可能几十像素；RTL 还额外 clamp 到 ±8 px 直接饱和（`flow_solver.sv:134-144`） | 提高帧率 / 用金字塔 / 用 IMU 做先验预测初值 |
| **纯平移、窗口内 `(u,v)` 恒定** | 小车**旋转**（原地转向）会产生绕光心的旋转流场；靠近障碍物时同一窗口内出现**运动视差/尺度变化** | 光流只用于**平移分量**估计；旋转用陀螺仪；或改用以极坐标/仿射模型的光流 |
| **亮度恒定** | 室内灯闪烁、地面反光、镜头自动曝光（AGC）都会让 `I_t` 混入光照变化 | 加**光照归一化**（局部均值/方差归一）或改用对光照不敏感的描述子；固定相机曝光 |
| **图像静态、两帧一次** | 真实视频是连续流，需要**帧间流水**；本项目一帧算完才 `start` 下一帧 | 做双缓冲 + 流水化 FSM；考虑只用每 N 帧做一次光流 |
| **纹理丰富** | 白墙、平整地面、纯色障碍物 → 孔径问题 + `det < 1000` → 输出零向量 | 输出**置信度**（`det` 或最小特征值）供上层加权/剔除；必要时补结构光或纹理板 |
| **固定 320×240** | 真实相机分辨率不同 | 参数化 `WIDTH`（本项目部分参数化），注意 `line_buffer_5x5` 窗口尺寸硬编码 |
| **像素无噪声** | 卷帘快门、运动模糊、传感器噪声都会污染梯度 | 空间预滤波（本项目已有帧平均 `gradient_compute.sv:116`，但会加剧运动模糊） |
| **相机固定/静止** | 相机随小车运动，全图流场非零 | 必须做**全局运动补偿**（ego-motion），才能得到 `v_rel`；这正是 §9① 要做的事 |

---

## 附：快速索引（若只想看 6 个地方）

| 你想看什么 | 去哪 |
|---|---|
| ★ 为什么这份 RTL 不能用 | §5.9（`pyramid_control_fsm.sv:181-183`、`frame_warper.sv:245`、`flow_upsampler.sv:236`、`optical_flow_top_pyramidal.sv:135-186`） |
| ★ 资源数字为何不可信 | §5.8（`prj/unopt/utilization_unopt.rpt:4` 报告日期 vs `git log` 提交时间） |
| ★ 真正可复用的算法 | `python/lucas_kanade_core.py:31-135`（有 CI 回归，§7.1） |
| 实测时序 | `prj/unopt/timing_postroute_unopt.rpt:139`（WNS）、`:202-217`（关键路径） |
| 实测资源 | `prj/unopt/utilization_postroute_unopt.rpt:35-40`（LUT/FF/BRAM）、`:98-101`（RAMB36 明细）、DSP 段 |
| 约束与综合顶 | `constraints/timing_unopt.xdc:4-6`、`scripts/synth_config.tcl:50` |
