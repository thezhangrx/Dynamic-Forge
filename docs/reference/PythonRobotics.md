# PythonRobotics 技术总结

> 参考项目路径：`/home/zhang/Bullet_Platform/Outside/Algo/PythonRobotics`
> 本文档为**只读调研**产物：调研过程只读取代码/文档/目录，未修改、未构建、未安装依赖、未联网、未运行任何示例。
> **来源约定**：每条技术结论后括注来源文件（相对项目根的路径）。代码里读不到的内容一律写「未确认」，不做推测性断言。行内 `代码` 表示实现细节，`公式` 用 LaTeX/行内代码给出并解释符号含义。

---

## 1. 一句话定位

**PythonRobotics 是一个「可读性优先」的机器人算法 Python 代码合集，同时配套一本用 Sphinx 写的在线教科书**（`README.md` 第 80 行："PythonRobotics is a Python code collection and a textbook of robotics algorithms"）。项目由 **Atsushi Sakai（@Atsushi_twi）** 创建并维护，接受社区贡献（`LICENSE`、`docs/conf.py` 的 `author = 'Atsushi Sakai'`、`README.md` 的 Authors 一节）。

三大设计哲学（`docs/modules/0_getting_started/1_what_is_python_robotics_main.rst`）：

1. **易于理解每个算法的基本思想**（面向机器人初学者，故刻意不抽象、不用重型框架）；
2. **选取广泛使用且实用的算法**；
3. **依赖最小化**（`README.md` 的 Features 第 3 条 "Minimum dependency"，运行示例只需 NumPy / SciPy / Matplotlib / cvxpy）。

**许可证：MIT License**，`LICENSE` 第 1、3 行：

```text
The MIT License (MIT)
Copyright (c) 2016 - now Atsushi Sakai and other contributors
```

**关键论文/出处**（README 的 Citing 一节要求学术使用本项目时引用）：
- *PythonRobotics: a Python code collection of robotics algorithms*，arXiv:1808.10703（`README.md` 第 100 行给出标题与链接；本地文件中未列出作者名单，**作者信息未确认**）。
- 官方文档（教科书）：`https://atsushisakai.github.io/PythonRobotics/index.html`（`README.md` 第 135 行）。
- 全部动画 GIF 单独存放在 `AtsushiSakai/PythonRoboticsGifs` 仓库（`README.md` 第 137 行）。

---

## 2. 技术栈与工具链

### 2.1 语言与运行时

| 项 | 值 | 来源 |
|---|---|---|
| 语言 | Python | `README.md` |
| 推荐版本（README） | Python 3.13.x | `README.md` 第 107 行 |
| 推荐版本（文档） | Python 3.12.x | `docs/modules/0_getting_started/2_how_to_run_sample_codes_main.rst`（"Install Python 3.12.x"） |
| conda 环境版本 | `python=3.13` | `requirements/environment.yml` |
| CI 测试版本 | Python 3.14（Linux / macOS / Windows 三个 workflow） | `.github/workflows/Linux_CI.yml`、`MacOS_CI.yml`、`Windows_CI.yml` |
| AppVeyor 版本 | `C:\Python313-x64`（VS 2022） | `appveyor.yml` |

> **坑（版本口径不一致）**：README 说 3.13、文档说 3.12、conda 环境是 3.13、CI 实际跑 3.14。到底哪个是"支持基线"**未确认**，以 `requirements/requirements.txt` 的依赖钉版为准更实际。

### 2.2 运行依赖（`requirements/requirements.txt`）

```text
numpy == 2.5.3
scipy == 1.18.1
matplotlib == 3.11.2
cvxpy == 1.9.3
ecos == 2.0.14
```

`requirements/environment.yml`（conda-forge 渠道）：`python=3.13`、`pip`、`scipy`、`numpy`、`cvxpy`、`matplotlib`。

**依赖分工（按代码实际 import 归纳）**：
- `numpy`：几乎所有算法的状态/栅格/矩阵运算（如 `PathPlanning/AStar/a_star.py`、`Localization/particle_filter/particle_filter.py`）；
- `scipy`：`scipy.linalg` 解 Riccati/LQR（`PathTracking/lqr_steer_control/lqr_steer_control.py` 的 `solve_DARE`/`dlqr`）、`scipy.spatial`（`cKDTree` 见 `PathPlanning/HybridAStar/hybrid_a_star.py`，`Voronoi` 见 `PathPlanning/VoronoiRoadMap/voronoi_road_map.py`）、`scipy.spatial.transform.Rotation`（`utils/angle.py`）；
- `matplotlib`：全部示例的动画与绘图（82 个 .py 文件出现 `plt.pause`）；
- `cvxpy` + `ecos`：凸优化，主要服务 `AerialNavigation/rocket_powered_landing/rocket_powered_landing.py`（连续凸化）；追踪类 MPC 用的是 cvxpy 的 **CLARABEL** 求解器（`PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py`）。

**无 `pyproject.toml` / `setup.py`**：项目根目录只有 `requirements/`，说明它不是一个可 `pip install` 的包，而是「进入子目录直接 `python xxx.py`」的示例集合（`docs/modules/0_getting_started/2_how_to_run_sample_codes_main.rst` 第 5 步）。根目录 `__init__.py`（0 字节）只是让测试能以包路径导入。

### 2.3 开发/测试依赖

`requirements/requirements.txt` 中标注为测试用途：`pytest == 9.1.0`、`pytest-xdist == 3.8.0`（并行）、`mypy == 2.3.1`（类型检查）、`ruff == 0.16.9`（lint/格式化）。

代码风格与类型检查配置：
- `ruff.toml`：`line-length = 88`，`select = ["F", "E", "W", "UP"]`，`ignore = ["E501", "E741", "E402"]`，`target-version = "py313"`，`[mccabe] max-complexity = 10`，`[pydocstyle] convention = "numpy"`（注意：`pydocstyle` 并未在 select 里启用）；
- `mypy.ini`：`ignore_missing_imports = True`（**未开启严格模式**，所以类型检查只保证"能过"）。
- README 里还提到 `pycodestyle`，但仓库实际用的是 `ruff`（`ruff.toml` + `tests/test_codestyle.py`）；这一处文档与实现不同步。

### 2.4 文档站（教科书）

- 目录 `docs/`，Sphinx 工程：`docs/conf.py`（`copyright = '2018-now, Atsushi Sakai'`，`master_doc = 'index'`，`language = "en"`）；`docs/index_main.rst` 是主 toctree。
- 扩展：`sphinx.ext.autodoc`、`sphinx.ext.napoleon`（解析 NumPy 风格 docstring）、`sphinx_copybutton`、`sphinx_rtd_dark_mode`（`docs/conf.py` 第 42–51 行）。
- 文档依赖 `docs/doc_requirements.txt`：`sphinx == 7.2.6`、`sphinx_rtd_theme == 2.0.0`、`IPython == 8.20.0`、`sphinxcontrib-napoleon == 0.7`、`sphinx-copybutton`、`sphinx-rtd-dark-mode`。
- 章节目录结构（`docs/modules/`）按域编号：`0_getting_started`、`1_introduction`、`2_localization`、`3_mapping`、`4_slam`、`5_path_planning`、`6_path_tracking`、`7_arm_navigation`、`8_aerial_navigation`、`9_bipedal`、`10_inverted_pendulum`、`11_utils`、`12_appendix`、`13_mission_planning`。
- 构建：`cd docs; make html`（`docs/README.md`、`.circleci/config.yml`）；`docs/_build/html/` 为产物。
- 部署：GitHub Pages，用 `sphinx-notes/pages@v3`（`.github/workflows/gh-pages.yml`）；CircleCI 也构建文档并 `store_artifacts`（`.circleci/config.yml`）。
- `docs/modules/12_appendix/` 是难得的"教科书"内容：`steering_motion_model_main.rst`（自行车模型转弯半径推导）、`Kalmanfilter_basics_main.rst`（高斯/协方差/贝叶斯滤波基础）、`internal_sensors_main.rst`、`external_sensors_main.rst`。

### 2.5 CI

| 系统 | 触发 | 内容 | 来源 |
|---|---|---|---|
| GitHub Actions Linux | push master / PR | Python 3.14 → `pip install -r requirements/requirements.txt` → `bash runtests.sh` | `.github/workflows/Linux_CI.yml` |
| GitHub Actions macOS | 同上 | 同上（先 `brew install bash`） | `.github/workflows/MacOS_CI.yml` |
| GitHub Actions Windows | 同上 | 同上（用 bash 跑 runtests.sh） | `.github/workflows/Windows_CI.yml` |
| AppVeyor | 仅 master | VS2022 + Python 3.13，`pytest tests -n auto -Werror --durations=0` | `appveyor.yml` |
| CircleCI | 每次 | 只构建 Sphinx 文档并归档 HTML | `.circleci/config.yml` |
| CodeQL | push/PR + 每周 cron | 代码扫描，配置 `./.github/codeql/codeql-config.yml` | `.github/workflows/codeql.yml` |
| gh-pages | push master | 构建并部署文档 | `.github/workflows/gh-pages.yml` |

`runtests.sh` 全文就是一个命令：

```bash
pytest tests -l -Werror --durations=0
```

即 **警告即错误**、失败时打印局部变量、并按耗时排序输出。这一点对使用者很重要：任何 NumPy/Matplotlib 的 DeprecationWarning 都会让 CI 红。

### 2.6 仓库治理文件

`CODE_OF_CONDUCT.md`、`CONTRIBUTING.md`、`SECURITY.md`、`users_comments.md`（用户案例收集）、`.gitignore`、`_config.yml`、`icon.png`、`mypy.ini`、`ruff.toml`。
---

## 3. 总览清单（十个域、全部子目录/算法）

> 覆盖方式：先 `find`/`ls` 出十个域下的**全部**子目录与 `.py` 文件，再逐项填表。多文件子目录按"文件即算法"展开（例如 `AStar/` 有 3 个独立算法、`RRT/` 有 3 个、`TimeBasedPathPlanning/` 有 6 个文件）。共 **102 项**。
> 「关键文件」列均为相对项目根路径；`utils/` 是跨域共用工具，单列在 §5。

### 3.1 PathPlanning（48 个子目录 → 58 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| AStar | A\* 网格搜索 | `f=g+h`，h 为欧氏距离，8 邻域最优搜索 | `PathPlanning/AStar/a_star.py` |
| AStar（两端搜索） | 双向交替 A\* | 起终点两侧轮流扩展，启发式用曼哈顿距离 | `PathPlanning/AStar/a_star_searching_from_two_side.py` |
| AStar 变体集 | A\* 变体 | 波束/迭代加深/动态加权/Theta\*/跳点 5 种变体可开关 | `PathPlanning/AStar/a_star_variants.py` |
| BidirectionalAStar | 双向 A\* | 两棵搜索树以前沿对侧节点为启发式参考 | `PathPlanning/BidirectionalAStar/bidirectional_a_star.py` |
| BreadthFirstSearch | 广度优先搜索 | FIFO 队列逐层扩展，无启发式 | `PathPlanning/BreadthFirstSearch/breadth_first_search.py` |
| BidirectionalBreadthFirstSearch | 双向 BFS | 两侧 BFS，靠 closed 集合交叉命中判相遇 | `PathPlanning/BidirectionalBreadthFirstSearch/bidirectional_breadth_first_search.py` |
| DepthFirstSearch | 深度优先搜索 | LIFO 栈，路径非最优 | `PathPlanning/DepthFirstSearch/depth_first_search.py` |
| Dijkstra | 迪杰斯特拉 | 只按 `g` 取最小，8 邻域非负权最短路 | `PathPlanning/Dijkstra/dijkstra.py` |
| GreedyBestFirstSearch | 贪心最佳优先 | 只按 `h` 排序，快但不最优 | `PathPlanning/GreedyBestFirstSearch/greedy_best_first_search.py` |
| ThetaStar | Theta\* 任意角 | 用视线（Bresenham）把节点直连祖父，破除栅格角约束 | `PathPlanning/ThetaStar/theta_star.py` |
| DStar | D\* 动态重规划 | `h/k` 双值 + `process_state` 增量修复，遇新障碍重规划 | `PathPlanning/DStar/dstar.py` |
| DStarLite | D\* Lite | `g/rhs` + key 队列 + `km`，从终点反向增量搜索 | `PathPlanning/DStarLite/d_star_lite.py` |
| ProbabilisticRoadMap | 概率路图 PRM | 自由空间随机采样 + KD 树近邻连边 + Dijkstra | `PathPlanning/ProbabilisticRoadMap/probabilistic_road_map.py` |
| VisibilityRoadMap | 可视图路图 | 障碍顶点外扩成节点，线段不穿障碍即连边 | `PathPlanning/VisibilityRoadMap/visibility_road_map.py`、`geometry.py` |
| VoronoiRoadMap | 维诺图路图 | 取 Voronoi 顶点（离障碍最远）建图 + Dijkstra | `PathPlanning/VoronoiRoadMap/voronoi_road_map.py`、`dijkstra_search.py` |
| RRT | 快速扩展随机树 | 随机采样 + `steer` 最近节点生长，概率完备 | `PathPlanning/RRT/rrt.py` |
| RRT（路径平滑） | 带平滑的 RRT | 采样后对路径做剪枝/平滑处理 | `PathPlanning/RRT/rrt_with_pathsmoothing.py` |
| RRT（Sobol 采样） | 低差异序列 RRT | 用 Sobol 序列替代伪随机采样，覆盖更均匀 | `PathPlanning/RRT/rrt_with_sobol_sampler.py` |
| Sobol 序列库 | Sobol 低差异序列 | 生成 Sobol 点集的纯 Python 实现 | `PathPlanning/RRT/sobol/sobol.py` |
| RRTStar | RRT\* | `choose_parent` + `rewire` + 递减邻域半径，渐进最优 | `PathPlanning/RRTStar/rrt_star.py` |
| InformedRRTStar | 知情 RRT\* | 找到解后用椭圆子集直接采样，加速收敛 | `PathPlanning/InformedRRTStar/informed_rrt_star.py` |
| BatchInformedRRTStar | 批处理知情 RRT\*（BIT\*） | 批量采样 + 启发式排序 + 椭球/球并集采样 | `PathPlanning/BatchInformedRRTStar/batch_informed_rrt_star.py` |
| LQRRRTStar | LQR-RRT\* | 用 LQR 局部规划器代替直线 steer，带动力学扩展 | `PathPlanning/LQRRRTStar/lqr_rrt_star.py` |
| ClosedLoopRRTStar | 闭环 RRT\*（C-LQR） | 树扩展时用闭环 LQR 仿真整段轨迹并碰撞检查 | `PathPlanning/ClosedLoopRRTStar/closed_loop_rrt_star_car.py` |
| RRTDubins | RRT+Dubins | 用 Dubins 曲线连接节点，满足最小转弯半径 | `PathPlanning/RRTDubins/rrt_dubins.py` |
| RRTStarDubins | RRT\*+Dubins | 在 RRT\* 框架内用 Dubins 做 steer 与 rewire | `PathPlanning/RRTStarDubins/rrt_star_dubins.py` |
| RRTStarReedsShepp | RRT\*+Reeds-Shepp | 用可前进/倒车的最短曲线连接，适合泊车场景 | `PathPlanning/RRTStarReedsShepp/rrt_star_reeds_shepp.py` |
| DubinsPath | Dubins 曲线 | 只许前进的最短曲线，6 种字（LSL/RSR/LSR/RSL/RLR/LRL） | `PathPlanning/DubinsPath/dubins_path_planner.py` |
| ReedsSheppPath | Reeds-Shepp 曲线 | 允许倒车的最短曲线，48 种字 + 时间翻转/反射 | `PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py` |
| BezierPath | 贝塞尔曲线路径 | 控制点凸组合生成平滑路径 | `PathPlanning/BezierPath/bezier_path.py` |
| BSplinePath | B 样条路径 | 基函数加权，局部可控、曲率连续 | `PathPlanning/BSplinePath/bspline_path.py` |
| CubicSpline | 三次样条（Spline2D） | 以弧长为参数的 2D 三次样条 `Spline2D`，输出曲率/航向，被大量模块复用 | `PathPlanning/CubicSpline/cubic_spline_planner.py` |
| Catmull_RomSplinePath | Catmull-Rom 样条 | 过控制点的 C¹ 插值样条，配混合函数 | `PathPlanning/Catmull_RomSplinePath/catmull_rom_spline_path.py`、`blending_functions.py` |
| ClothoidPath | 回旋曲线（欧拉螺线） | 曲率随弧长线性变化，边界问题用牛顿法求解 | `PathPlanning/ClothoidPath/clothoid_path_planner.py` |
| Eta3SplinePath | η³ 样条路径 | 以 η³ 样条（曲率连续、闭式）生成轮式机器人平滑路径 | `PathPlanning/Eta3SplinePath/eta3_spline_path.py` |
| Eta3SplineTrajectory | η³ 样条轨迹 | η³ 样条 + 时间参数化（速度/加速度受限） | `PathPlanning/Eta3SplineTrajectory/eta3_spline_trajectory.py` |
| QuinticPolynomialsPlanner | 五次多项式轨迹 | 用五次多项式满足起止位置/速度/加速度 6 个边界条件 | `PathPlanning/QuinticPolynomialsPlanner/quintic_polynomials_planner.py` |
| FrenetOptimalTrajectory | Frenet 最优轨迹 | 横向五次 + 纵向四次多项式采样，代价包含 jerk/时间/横向偏差 | `PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py` |
| CartesianFrenetConverter | 笛卡尔↔Frenet 转换 | 位置/速度/加速度/曲率在两坐标系间解析互换 | `PathPlanning/FrenetOptimalTrajectory/cartesian_frenet_converter.py` |
| DynamicMovementPrimitives | 动态运动基元 DMP | 二阶弹簧阻尼系统 + 高斯基函数学习示教轨迹 | `PathPlanning/DynamicMovementPrimitives/dynamic_movement_primitives.py` |
| ModelPredictiveTrajectoryGenerator | 模型预测轨迹生成器 | 牛顿法解两点边值问题（含 `km/kf` 参数）并生成查表 | `PathPlanning/ModelPredictiveTrajectoryGenerator/trajectory_generator.py`、`motion_model.py`、`lookup_table_generator.py` |
| StateLatticePlanner | 状态栅格规划 | 采样终端状态（均匀/偏置极坐标、车道）+ MPTG 连边 | `PathPlanning/StateLatticePlanner/state_lattice_planner.py` |
| HybridAStar | 混合 A\* | 栅格 A\* + 自行车模型运动基元 + 栅格 Dijkstra 启发 + Reeds-Shepp 解析扩展 | `PathPlanning/HybridAStar/hybrid_a_star.py`、`car.py`、`dynamic_programming_heuristic.py` |
| LQRPlanner | LQR 路径规划 | 双积分器模型上用 LQR 迭代生成点到点路径 | `PathPlanning/LQRPlanner/lqr_planner.py` |
| DynamicWindowApproach | 动态窗口法 DWA | 在速度窗口内采样 `(v,ω)`、前向预测轨迹、按代价函数选优 | `PathPlanning/DynamicWindowApproach/dynamic_window_approach.py` |
| ElasticBands | 弹性带 | 全局路径被内部收缩力 + 障碍斥力形变，保持无碰撞 | `PathPlanning/ElasticBands/elastic_bands.py` |
| PotentialFieldPlanning | 人工势场 | 引力（至目标）+ 斥力（障碍）叠加，梯度下降寻路 | `PathPlanning/PotentialFieldPlanning/potential_field_planning.py` |
| FlowField | 流场寻路 | cost field + integration field + vector field 三层场导航 | `PathPlanning/FlowField/flowfield.py` |
| BugPlanning | Bug 算法族 | 贪心直行 + 遇障沿边界绕行（Bug0/1/2） | `PathPlanning/BugPlanning/bug.py` |
| GridBasedSweepCPP | 栅格牛耕式覆盖 | 按最长边定扫描方向，往返扫描 + 转弯窗口回退 | `PathPlanning/GridBasedSweepCPP/grid_based_sweep_coverage_path_planner.py` |
| SpiralSpanningTreeCPP | 螺旋生成树覆盖 | 2×2 合并建图 + DFS 生成树 + 螺旋绕行（Spiral-STC） | `PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py` |
| WavefrontCPP | 波前覆盖 | 距离/路径变换构造势场，从最大势处"爬山"覆盖 | `PathPlanning/WavefrontCPP/wavefront_coverage_path_planner.py` |
| ParticleSwarmOptimization | 粒子群路径优化 | PSO 在路径点空间搜索，适应度含避障罚项 | `PathPlanning/ParticleSwarmOptimization/particle_swarm_optimization.py` |
| SpaceTimeAStar | 时空 A\* | 状态 `(x,y,t)`，代价 = 时间步，可绕开动态障碍 | `PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar.py` |
| SafeIntervalPathPlanner | SIPP 安全区间规划 | 预计算每格安全时间区间，按区间剪枝节点 | `PathPlanning/TimeBasedPathPlanning/SafeInterval.py` |
| PriorityBasedPlanner | 优先级多机规划 | 按起终点距离降序逐个规划并把路径写入预约矩阵 | `PathPlanning/TimeBasedPathPlanning/PriorityBasedPlanner.py` |
| Grid（动态障碍环境） | 动态障碍 3D 预约栅格 | `reservation_matrix[x,y,t]` + 三种障碍运动生成器 + 安全区间提取 | `PathPlanning/TimeBasedPathPlanning/GridWithDynamicObstacles.py` |
| Node / BaseClasses / Plotting | 时空规划基础设施 | `(x,y,t)` 节点与 `NodePath`、规划器抽象基类、逐时刻动画 | `PathPlanning/TimeBasedPathPlanning/Node.py`、`BaseClasses.py`、`Plotting.py` |

### 3.2 PathTracking（8 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| pure_pursuit | 纯追踪 | 前视距离 `Lf=kv+Lfc` 求曲率，`δ=atan2(2WB sinα/Lf)` | `PathTracking/pure_pursuit/pure_pursuit.py` |
| stanley_control | Stanley 控制 | 前轴横向误差 + 航向误差的反正切转向律 | `PathTracking/stanley_control/stanley_control.py` |
| rear_wheel_feedback_control | 后轮反馈控制 | 基于后轴运动学的非线性反馈 + PID 车速 | `PathTracking/rear_wheel_feedback_control/rear_wheel_feedback_control.py` |
| lqr_steer_control | LQR 转向控制 | 线性化误差状态 `[e,ė,θe,θ̇e]` + DARE 求 `K` + 前馈 | `PathTracking/lqr_steer_control/lqr_steer_control.py` |
| lqr_speed_steer_control | LQR 速度+转向 | 5 维误差状态、2 维输入，同时调车速与转角 | `PathTracking/lqr_speed_steer_control/lqr_speed_steer_control.py` |
| model_predictive_speed_and_steer_control | 迭代线性 MPC 追踪 | 线性化 + 逐次迭代的 MPC，cvxpy/CLARABEL 求解 | `PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py` |
| cgmres_nmpc | C/GMRES 非线性 MPC | 连续时间 NMPC，用 C/GMRES 在线解最优控制 | `PathTracking/cgmres_nmpc/cgmres_nmpc.py` |
| move_to_pose（+ robot 版） | 位姿控制 | 极坐标下的 `ρ-α-β` 控制律 + 速度限幅 | `PathTracking/move_to_pose/move_to_pose.py`、`move_to_pose_robot.py` |

### 3.3 Localization（7 个子目录 → 8 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| extended_kalman_filter | 扩展卡尔曼滤波 | 一阶线性化的非线性 KF，雅可比 `G/V/H` 传播协方差 | `Localization/extended_kalman_filter/extended_kalman_filter.py` |
| ekf_with_velocity_correction | 带速度校正的 EKF | 状态加入尺度因子/速度项，观测同时校正速度 | `Localization/extended_kalman_filter/ekf_with_velocity_correction.py` |
| unscented_kalman_filter | 无迹卡尔曼滤波 | sigma 点传播经非线性函数后重建均值/协方差 | `Localization/unscented_kalman_filter/unscented_kalman_filter.py` |
| cubature_kalman_filter | 容积卡尔曼滤波 | 用 2n 个容积点做三阶球面-径向积分近似 | `Localization/cubature_kalman_filter/cubature_kalman_filter.py` |
| ensemble_kalman_filter | 集合卡尔曼滤波 | 用随机集合样本估计协方差，无需雅可比 | `Localization/ensemble_kalman_filter/ensemble_kalman_filter.py` |
| particle_filter | 粒子滤波 | 加权粒子近似后验 + 重采样，可多峰 | `Localization/particle_filter/particle_filter.py` |
| histogram_filter | 直方图滤波 | 把状态空间离散成网格，逐格贝叶斯更新概率 | `Localization/histogram_filter/histogram_filter.py` |
| gps_imu_fusion | GPS/IMU 融合 + 零偏估计 | 15 维 EKF：位置/速度/姿态/加速度计与陀螺零偏，含 GPS 中断预测 | `Localization/gps_imu_fusion/gps_imu_fusion.py` |

### 3.4 Mapping（11 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| gaussian_grid_map | 高斯栅格地图 | 用高斯分布对每格占位概率做贝叶斯更新 | `Mapping/gaussian_grid_map/gaussian_grid_map.py` |
| ray_casting_grid_map | 射线投射栅格地图 | Bresenham 射线穿过栅格，log-odds 更新占据/空闲 | `Mapping/ray_casting_grid_map/ray_casting_grid_map.py` |
| lidar_to_grid_map | 激光雷达→栅格地图 | 把 2D 量程扫描栅格化并做可视化 | `Mapping/lidar_to_grid_map/lidar_to_grid_map.py` |
| DistanceMap | 距离图 | 距离变换（两遍扫描/`ndimage`）给出到最近障碍的距离 | `Mapping/DistanceMap/distance_map.py` |
| kmeans_clustering | K-means 聚类 | 迭代分配-更新质心，把点云切成目标簇 | `Mapping/kmeans_clustering/kmeans_clustering.py` |
| rectangle_fitting | 矩形/L 形拟合 | 搜索最优朝向把目标簇拟合成矩形框 | `Mapping/rectangle_fitting/rectangle_fitting.py`、`simulator.py` |
| circle_fitting | 圆拟合 | 三点定圆 + RANSAC/最小二乘选内点 | `Mapping/circle_fitting/circle_fitting.py` |
| normal_vector_estimation | 法向量估计 | 邻域 PCA 最小特征向量即法向 | `Mapping/normal_vector_estimation/normal_vector_estimation.py` |
| point_cloud_sampling | 点云采样 | 体素/随机下采样降低点云规模 | `Mapping/point_cloud_sampling/point_cloud_sampling.py` |
| ndt_map | NDT 地图 | 用局部正态分布表示点云，配准得分最大化 | `Mapping/ndt_map/ndt_map.py` |
| grid_map_lib | 栅格地图库 | `GridMap`/`FloatGrid`：世界坐标↔索引、多边形填充、膨胀 | `Mapping/grid_map_lib/grid_map_lib.py` |

### 3.5 SLAM（5 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| EKFSLAM | EKF-SLAM | 机器人位姿与路标联合状态 + 协方差一阶更新 | `SLAM/EKFSLAM/ekf_slam.py` |
| FastSLAM1 | FastSLAM 1.0 | 粒子表位姿 + 每粒子每路标一个 EKF，用运动模型提议分布 | `SLAM/FastSLAM1/fast_slam1.py` |
| FastSLAM2 | FastSLAM 2.0 | 提议分布融合最新观测，减少粒子退化 | `SLAM/FastSLAM2/fast_slam2.py` |
| GraphBasedSLAM | 图优化 SLAM | 位姿图误差函数 + Gauss-Newton/Levenberg-Marquardt 优化 | `SLAM/GraphBasedSLAM/graph_based_slam.py`、`graphslam/graph.py`、`pose/se2.py`、`edge/edge_odometry.py` |
| ICPMatching | ICP 点云匹配 | 最近邻配对 + SVD 求最优 `R,t`，迭代直到收敛 | `SLAM/ICPMatching/icp_matching.py` |

### 3.6 ArmNavigation（5 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| two_joint_arm_to_point_control | 二连杆逆运动学控制 | 余弦定理闭式解 θ1/θ2 + P 控制逼近 | `ArmNavigation/two_joint_arm_to_point_control/two_joint_arm_to_point_control.py` |
| n_joint_arm_to_point_control | N 连杆臂点控制 | 雅可比伪逆求关节增量 + P 控制 | `ArmNavigation/n_joint_arm_to_point_control/n_joint_arm_to_point_control.py`、`NLinkArm.py` |
| n_joint_arm_3d | 3D N 连杆正逆运动学 | 标准 DH 齐次变换 + ZYZ 欧拉角 + 6×n 雅可比数值 IK | `ArmNavigation/n_joint_arm_3d/NLinkArm3d.py`、`random_forward_kinematics.py`、`random_inverse_kinematics.py` |
| arm_obstacle_navigation（2 版） | 机械臂关节空间避障 | 关节角离散成环形栅格，圆-线段碰撞检测 + A\* | `ArmNavigation/arm_obstacle_navigation/arm_obstacle_navigation.py`、`arm_obstacle_navigation_2.py` |
| rrt_star_seven_joint_arm_control | 七自由度臂 RRT\* | 7 维关节空间 RRT\* + 3D 碰撞球检测 | `ArmNavigation/rrt_star_seven_joint_arm_control/rrt_star_seven_joint_arm_control.py` |

### 3.7 AerialNavigation（2 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| drone_3d_trajectory_following | 四旋翼 3D 轨迹跟踪 | 五次多项式轨迹 + 推力 PD + 姿态 P（微分平坦式前馈） | `AerialNavigation/drone_3d_trajectory_following/drone_3d_trajectory_following.py`、`Quadrotor.py`、`TrajectoryGenerator.py` |
| rocket_powered_landing | 火箭动力着陆 | 6-DoF 动力学逐次凸化（SOCP）+ 自由末端时间 + 虚拟控制 | `AerialNavigation/rocket_powered_landing/rocket_powered_landing.py` |

### 3.8 Bipedal（1 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| bipedal_planner | 双足步行规划 | LIPM 积分 + 解析最小二乘修正落脚点 | `Bipedal/bipedal_planner/bipedal_planner.py` |

### 3.9 InvertedPendulum（2 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| inverted_pendulum_lqr_control | 倒立摆 LQR | 线性化 4 状态模型 + DARE + `u=-Kx` | `InvertedPendulum/inverted_pendulum_lqr_control.py` |
| inverted_pendulum_mpc_control | 倒立摆 MPC | 同一线性模型上的有限时域 QP，滚动时域 | `InvertedPendulum/inverted_pendulum_mpc_control.py` |

### 3.10 MissionPlanning（2 项）

| 算法名 | 中文名 | 一句话原理 | 关键文件 |
|---|---|---|---|
| BehaviorTree | 行为树 | XML 描述树，控制/动作/装饰节点按三态 tick 推进 | `MissionPlanning/BehaviorTree/behavior_tree.py`、`robot_behavior_case.py` |
| StateMachine | 有限状态机 | `(源状态, 事件)`→目标状态，带守卫与动作 | `MissionPlanning/StateMachine/state_machine.py`、`robot_behavior_case.py` |

---
## 4. 逐步详解

> 约定：`状态/输入输出` 按代码实际签名写；`复杂度` 若代码与文档都没有声明，一律写 **未确认**（本仓库绝大多数算法都没有复杂度注释，只有少数在 rst 里给了实测数据）。
> **全仓库通用的栅格约定**（AStar/BidirectionalAStar/BFS/BiBFS/DFS/Dijkstra/GBFS/Theta\* 共用）：障碍地图是**布尔二维表** `obstacle_map[x][y]`，`False`=自由、`True`=障碍；由障碍点列表按 `hypot(iox-x, ioy-y) <= robot_radius` 逐格生成（`PathPlanning/AStar/a_star.py` 的 `calc_obstacle_map`）；索引 `grid_index = (y-min_y)*x_width + (x-min_x)`，`position = index*resolution + min`；8 邻域运动模型固定为
> `[[1,0,1],[0,1,1],[-1,0,1],[0,-1,1],[-1,-1,√2],[-1,1,√2],[1,-1,√2],[1,1,√2]]`（dx, dy, cost）。

### 4.1 PathPlanning（一）：图搜索 / 组合式规划

#### A\*（`PathPlanning/AStar/a_star.py`）
- **原理/公式**：`f(n) = g(n) + h(n)`；`g(n) = g(parent) + move_cost`（直行 1、对角 √2）；`h(n) = w·hypot(n.x-goal.x, n.y-goal.y)`，代码里 `w = 1.0` 是函数内局部变量。每轮从 open 里取 `argmin f` 扩展。README 与文档均称启发式为 2D 欧氏距离（`docs/modules/5_path_planning/grid_base_search/grid_base_search_main.rst`）。
- **输入/输出**：`AStarPlanner(ox, oy, resolution, rr)`，`ox/oy` 是障碍点坐标列表（离散点，不是多边形），`rr` 为机器人半径；栅格范围由障碍点包围盒取整决定。`planning(sx, sy, gx, gy) -> (rx, ry)`（米制路径点）；内部节点类 `Node(x, y, cost, parent_index)`，`cost` 即 g。
- **关键步骤**：① 起终点转栅格索引建 `Node`；② `open_set[grid_index] = node`；③ 取 f 最小者；④ 命中终点则继承 parent/cost 并 break；⑤ 移出 open、写入 closed；⑥ 对 8 邻域生成子节点，`verify_node` 查边界与 `obstacle_map`；⑦ 已在 closed 跳过，不在 open 则加入，否则更优才替换；⑧ 沿 `parent_index` 回溯路径。
- **参数默认值**：`sx = 10.0`、`sy = 10.0`、`gx = 50.0`、`gy = 50.0`、`grid_size = 2.0`、`robot_radius = 1.0`；`show_animation = True`。
- **复杂度**：未确认（每轮对 open 线性 `min`）。
- **来源引用**：无论文；只有 Wikipedia 链接（`a_star.py` docstring，作者 Atsushi Sakai、Nikos Kanargias）。

#### A\* 变体（`PathPlanning/AStar/a_star_variants.py`）
模块级开关决定用哪种变体，基础代价是整数 10（正交）/14（对角），启发式 `get_hval` 为 10/14 的八方向（octile）距离。
- **Beam Search**：扩展前按 f 排序并裁到 `beam_capacity = 30`；
- **Iterative Deepening**：用阈值 `curr_f_thresh` 截断，每轮末 `curr_f_thresh = min(f_cost_list)`，空则 `np.inf`；
- **Dynamic Weighting**：`w = 1 + ε − ε·depth/upper_bound_depth`，`h ← w·h`，参数 `w, epsilon, upper_bound_depth = 1, 4, 500`；
- **Theta\***：沿方向推进到最远可通行点 `get_farthest_point`，代价 `offset·counter`，参数 `max_theta = 5`；
- **Jump Point**：先用 `key_points` 找角点（3×3 邻域障碍数为 1 或 3），`only_corners = False` 时再做角点对视线检测取中点，参数 `max_corner = 5`。
其余开关默认全 False；`main()` 起终点 `(5,5)→(35,45)`，`limit_x = limit_y = 101`。出处为 `http://theory.stanford.edu/~amitp/GameProgramming/Variations.html`（网页，非论文）。复杂度未确认。

#### 两端交替 A\*（`PathPlanning/AStar/a_star_searching_from_two_side.py`）
不是标准双向 A\*：两侧各有 open/closed，**每轮各扩展一个节点**。启发式是本仓库唯一的**曼哈顿距离** `hcost = |Δx| + |Δy|`，而步长用 `gcost = hypot`（对角 √2）→ 对角情形 h 会高估（非可采纳）。对角穿越被禁止：两个正交邻居都被占据时删除对角候选。默认 `main(obstacle_number=1500)`，边界 `[0,0]–[60,60]`。复杂度未确认。

#### BidirectionalAStar / BFS / 双向 BFS / DFS / Dijkstra / GreedyBestFirstSearch
| 算法 | 关键公式/机制 | 参数与输出 | 文件 |
|---|---|---|---|
| 双向 A\* | 两侧各取 `f_A = g_A + w·hypot(n, current_B)`、`f_B = g_B + w·hypot(n, current_A)`（`w=1.0`，**以前沿对侧当前节点为启发式参考**）；两侧当前节点坐标相同即相遇 | `planning(sx,sy,gx,gy)`；默认 `10,10→50,50`，`grid_size=2.0`，`robot_radius=1.0` | `BidirectionalAStar/bidirectional_a_star.py` |
| BFS | FIFO：`pop(list(open_set.keys())[0])`；只判"未在 closed 且未在 open"，无代价比较 | 同上默认值 | `BreadthFirstSearch/breadth_first_search.py` |
| 双向 BFS | 两侧各弹队首，相遇判据是"我这轮关闭的格已在对方 closed 中"（`c_id_A in closed_set_B`） | 同上默认值 | `BidirectionalBreadthFirstSearch/bidirectional_breadth_first_search.py` |
| DFS | LIFO：`pop(list(open_set.keys())[-1])`；入栈时**同时写入 closed**（不检查 open），路径远非最优 | 同上默认值 | `DepthFirstSearch/depth_first_search.py` |
| Dijkstra | 无 h，`c_id = min(open_set, key=lambda o: open_set[o].cost)`；松弛条件用 `>=` | **起点默认 `sx = -5.0, sy = -5.0`**（与其它网格算法不同），`gx=50.0, gy=50.0` | `Dijkstra/dijkstra.py` |
| 贪心最佳优先 | 只按 `h`（欧氏、`w=1.0`）排序，`cost` 虽累计但不参与选择 → 不保证最优；先移出 open/写入 closed 再判终点 | 默认 `10,10→50,50` | `GreedyBestFirstSearch/greedy_best_first_search.py` |

以上 6 个算法的时间/空间复杂度在代码与文档中**都未声明 → 未确认**；均使用 `dict` + 线性 `min`/`sort`，实际效率低于标准优先队列实现。`docs/` 下**没有** GreedyBestFirstSearch 与 BidirectionalBreadthFirstSearch 的 rst。

#### Theta\*（`PathPlanning/ThetaStar/theta_star.py`）
在 A\* 框架上加"任意角"修正：若 `current.parent_index` 已在 closed 中，且祖父节点 `grandparent` 到候选节点 `line_of_sight` 成立，则
`node.cost = grandparent.cost + hypot(node - grandparent)`、`node.parent_index = current.parent_index`（直连祖父，绕开栅格角）。
`line_of_sight` 用 **Bresenham 直线算法**逐格 `verify_node`，任一路径格不可通行即失败。参数：`show_animation = True`、`use_theta_star = True`（等价恒真开关）、`w = 1.0`、默认 `10,10→50,50`。出处：AAAI-07 论文 *Theta\*: Any-Angle Path Planning on Grids*（代码里只给了 `https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf`，未写作者/年份 → 作者年份未确认）。

#### D\*（`PathPlanning/DStar/dstar.py`）
- **原理/公式（照代码）**：状态 `State(x, y)`，字段 `parent/state('.'|'#'|'e'|'*'|'s')/t ∈ {new,open,close}/h/k`。代价用**欧氏距离**：`c(a,b) = maxsize`（若任一为 `#` 障碍）否则 `hypot`。
  - `insert(state, h_new)`：`t=="new" → k=h_new`；`t=="open" → k=min(k,h_new)`；`t=="close" → k=min(h,h_new)`；然后 `h=h_new, t="open"` 并入 open。
  - `process_state()`：取 `k` 最小的 `x`，`k_old = kmin`；**RAISE**（`k_old < x.h`）：对邻居取 `x.h = min(x.h, y.h + c(x,y))`；**常规扩展**（`k_old == x.h`）：对满足 `y.t=="new"` 或 `y.parent==x 且 y.h != x.h+c` 或 `y.parent!=x 且 y.h > x.h+c` 的邻居 `y.parent=x; insert(y, x.h+c)`；**LOWER**（含 `y.t=="close" and y.h > k_old` 时 `insert(y, y.h)`）。
  - `modify_cost(x)`：`x.t=="close"` 时 `insert(x, x.parent.h + c(x, x.parent))`；`modify(state)` 先 `modify_cost`，再循环 `process_state()` 直到 `kmin >= state.h`。
- **输入/输出**：`Map(row, col)` + `set_obstacle(point_list)`；`Dstar(map).run(start, end) -> (rx, ry)`（int 栅格坐标）。`run` 里先 `insert(end, 0)` 反向传播，**随后硬编码插入新障碍** `AddNewObstacle`（`(5..20, 40)` 一行墙），再沿 parent 前进，父节点变障碍就 `modify` 增量修复。
- **参数默认值**：`Map(100, 100)`、`start = [10, 10]`、`goal = [50, 50]`；`State` 初值 `parent=None, state=".", t="new", h=0, k=0`。复杂度未确认。出处仅 Wikipedia D\* 条目（无作者/年份）。

#### D\* Lite（`PathPlanning/DStarLite/d_star_lite.py`）
- **原理/公式（照实现）**：`g`、`rhs` 均为 `np.full((x_max, y_max), inf)`。边代价
  `c(u,u') = inf`（`u'` 是障碍）否则匹配 8 邻域 motions 得 1 或 √2；**启发式 `h(s) ≡ 1`**（代码注释明确：不用欧氏距离以免舍入误差破坏可采纳性）；key
  `calculate_key(s) = ( min(g[s], rhs[s]) + h(s) + km, min(g[s], rhs[s]) )`。
  - `update_vertex(u)`：非终点时 `rhs[u] = min_{s'∈succ(u)} (c(u,s') + g[s'])`；若 `g[u] != rhs[u]` 则以新 key 插入队列 U。
  - `compute_shortest_path()`：循环条件 `compare_keys(U[0].key, key(start)) 或 rhs[start] != g[start]`；`k_old` 已过期则重插；`g[u] > rhs[u]` → `g[u]=rhs[u]` 并更新前驱；否则 `g[u]=inf` 并更新前驱与自身。
  - **增量变更**：`km += h(last)`（`last` 是移动前起点），对新障碍格 `g=rhs=inf` 后 `update_vertex` + `compute_shortest_path`；移动时 `start = argmin_{s'∈succ} (c(start,s') + g[s'])`，`g[start]==inf` 即无解。
- **输入/输出**：`DStarLite(ox, oy)`；`main(start, goal, spoofed_ox, spoofed_oy) -> (bool, pathx, pathy)`；`detect_changes()` 支持按时间注入伪造障碍。
- **参数默认值**：`show_animation = True`、`pause_time = 0.001`、`p_create_random_obstacle = 0`、`sx=10, sy=10, gx=50, gy=50`。
- **实现细节坑**：`g/rhs` 是 numpy 数组，`self.g[u.x, u.y]` 返回 0 维数组，`(g[u.x,u.y] > rhs[u.x,u.y]).any()` 对标量恒为 `False`；而 `== math.inf`、`!=` 比较仍按标量语义工作。该行为与 D\* Lite 伪代码不一致，**是否影响最优性未确认**。
- **复杂度**：代码只注释说"可用优先队列显著优化"（U 目前是 list + 每次 sort）→ **未确认**。
- **论文/出处**：`D* Lite`（`http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf`）与 `Improved Fast Replanning for Robot Navigation in Unknown Terrain`（`http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf`）；**作者/年份未在文件中给出 → 未确认**。

#### 路图类：PRM / 可视图 / 维诺图
三者的共同套路是"采样/构造节点 → KD 树近邻 + 碰撞检查连边 → 在路图上跑 Dijkstra"，其中 `PathPlanning/VoronoiRoadMap/dijkstra_search.py` 被 Voronoi 与 Visibility 两个模块**共用**（`visibility_road_map.py` 通过 `sys.path.append(...)` 后 `from VoronoiRoadMap.dijkstra_search import DijkstraSearch`）。

| 算法 | 关键公式/机制 | 关键参数与默认值 | 文件 |
|---|---|---|---|
| PRM | 自由空间均匀采样（拒绝障碍内样本）→ 每样本取 KD 树最近邻并按碰撞检查连边 → Dijkstra。起点/终点**用索引定位**：`open_set[len(road_map)-2]` 是起点、`len(road_map)-1` 是终点 | `N_SAMPLE = 500`、`N_KNN = 10`、`MAX_EDGE_LEN = 30.0`、`robot_size = 5.0`、`sx,sy,gx,gy = 10,10,50,50`；`prm_planning(..., *, rng=None)` 支持注入随机源（测试传 `default_rng(1233)`） | `ProbabilisticRoadMap/probabilistic_road_map.py` |
| 碰撞检查（PRM 与 Voronoi 同构） | 若两端点距离 `d >= MAX_EDGE_LEN` 直接判碰撞；否则以 `D = robot_radius` 为步长沿 `yaw = atan2(dy,dx)` 逐点 KD 查询 `dist <= rr`，并单独检查终点 | 同上 | 两文件的 `is_collision` |
| 维诺路图 | `Voronoi(oxy)` 的顶点（离障碍最远）+ 起终点作节点；每节点 `query([ix,iy], k=n_sample)` 按距离升序取候选，碰撞检查通过则连边，累计 `N_KNN` 条即停 | `self.N_KNN = 10`、`self.MAX_EDGE_LEN = 30.0`、`robot_size = 5.0`；Dijkstra 内节点同一性阈值 `dist <= 0.1` | `VoronoiRoadMap/voronoi_road_map.py` + `dijkstra_search.py` |
| 可视图 | 障碍顶点沿角平分线旋转 90° 外扩 `expand_distance` 后成节点；对每对节点做线段-多边形边相交测试，全部不相交才连边 | `expand_distance = 5.0`、`sx,sy,gx,gy = 10,10,50,50`、去重阈值 `0.1`；`ObstaclePolygon` 会闭合并规范为顺时针 | `VisibilityRoadMap/visibility_road_map.py` + `geometry.py` |

外扩方向公式（`visibility_road_map.py::calc_offset_xy`）：
`p_vec = atan2(y-py, x-px)`，`n_vec = atan2(ny-y, nx-x)`，`offset_vec = atan2(sin p_vec + sin n_vec, cos p_vec + cos n_vec) + π/2`，`offset = (x + d·cos offset_vec, y + d·sin offset_vec)`。
线段相交用 `Geometry.is_seg_intersect`：四个 `orientation` 满足 `(o1 != o2) and (o3 != o4)` 即相交，并额外处理共线 `on_segment`。三者复杂度均**未确认**（PRM/Voronoi 建图对每样本做 `k=n` 的 KD 查询，至少 O(n² log n) 级；可视图为 O(节点对数 × 总边数)）。
出处：PRM 与可视图只有 Wikipedia 链接、维诺图只有 CMU 课件链接；**均无正式论文引用**。

#### 本组"坑"清单
1. **测试文件名与 import 交叉错位**：`tests/test_voronoi_road_map_planner.py` 实际 import 的是 `VisibilityRoadMap.visibility_road_map`，`tests/test_visibility_road_map_planner.py` 实际 import 的是 `VoronoiRoadMap.voronoi_road_map`。
2. `PathPlanning/AStar/a_star_variants.py` 的 `calc_heuristic_2`（若在别处见过）**在本仓库不存在**（全仓 grep 无匹配）；曼哈顿只出现在 `a_star_searching_from_two_side.py`。
3. `tests/test_space_time_astar.py` 里注释写 `# path should have 28 entries`，断言却是 `== 31`，注释与代码矛盾（源码如此）。
4. 障碍地图只有 A\* 系列用布尔表，D\* 用字符串状态，D\* Lite 用集合；**没有任何模块导出 0/1 整数障碍图**。

---
### 4.2 PathPlanning（二）：采样式规划、路径原语、样条与轨迹优化

#### 4.2.1 采样类算法（RRT 家族）
所有采样器的公共约定：障碍用**圆** `(ox, oy, size)` 表示，碰撞判据 `min(d²) ≤ (size + robot_radius)²`；`rand_area = [min, max]` 是正方形采样区。

| 算法 | Steer / 关键公式 | 关键参数默认值 | 文件 / 主类 |
|---|---|---|---|
| RRT | 采样 `rnd` → 树中欧氏最近节点 → 沿方向按 `expand_dis` 步进（`n_expand = floor(extend_length/path_resolution)`，逐步 `x += path_resolution·cosθ`）；`random.randint(0,100) > goal_sample_rate` 时均匀采样，否则采目标（目标偏置≈`goal_sample_rate/101`）；末节点距目标 `≤ expand_dis` 时直连 | `expand_dis=3.0`、`path_resolution=0.5`、`goal_sample_rate=5`、`max_iter=500`、`robot_radius=0.0`、`play_area=None` | `RRT/rrt.py`，`class RRT`（内部 `Node`、`AreaBounds`） |
| RRT+捷径平滑 | 随机取路径上两点（按弧长参数化 `get_target_point`），若直线段无碰撞则替换中间折线；`line_collision_check` 以 `sample_step=0.2` 离散采样 | `maxIter=1000`（demo）、`sample_step=0.2`、`robot_radius=0.0` | `RRT/rrt_with_pathsmoothing.py` |
| RRT+Sobol | 与 RRT 同构，采样改为 **Sobol 低差异序列**（`sobol_quasirand(2, seed)` 返回下一维向量并把 seed 自增），目标是降低 dispersion | 同 RRT；`self.sobol_inter_ = 0` | `RRT/rrt_with_sobol_sampler.py`（`RRTSobol`）+ `RRT/sobol/sobol.py`（911 行的 `i4_sobol` 移植库，要求 `1 ≤ dim ≤ 40`） |
| RRT\* | 新节点 `cost = near.cost + d`；**邻域半径** `r = connect_circle_dist·sqrt(ln(n)/n)`（`n = len(node_list)+1`），并 `r = min(r, expand_dis)`；`choose_parent` 取 `near.cost + d` 最小者；`rewire` 在 `near.cost > new.cost + d` 且无碰撞时改挂父节点并 `propagate_cost_to_leaves` 递归更新子树；目标连接取 `node.cost + dist_to_goal` 最小 | `expand_dis=30.0`、`path_resolution=1.0`、`goal_sample_rate=20`、`max_iter=300`、`connect_circle_dist=50.0`、`search_until_max_iter=False` | `RRTStar/rrt_star.py`，`class RRTStar` |
| Informed RRT\* | `c_min = ‖start − goal‖`；用 SVD 把单位球旋转到 start→goal 主轴：`M = a1·[1,0,0]ᵀ`、`C = U·diag([1,1,det(U)det(Vhᵀ)])·Vh`；找到解后只在**椭圆子集**内采样：`r = [c_best/2, √(c_best²−c_min²)/2, √(c_best²−c_min²)/2]`，`rnd = C·diag(r)·x_ball + x_center`；单位球内均匀采样用 `r=b, φ=2πa/b` | `expand_dis=0.5`、`goal_sample_rate=10`、`max_iter=200`；邻域半径**硬编码** `r = 50.0·sqrt(ln n/n)`（不用 `connect_circle_dist`、不截断） | `InformedRRTStar/informed_rrt_star.py` |
| Batch Informed RRT\*（BIT\*） | 把空间栅格化为**隐式随机几何图**（`RTree`，`resolution=0.01`，节点 id 为混合基数编码，用于去重）；队列耗尽才开新批（`self.r = 2.0`）：已找到目标 `m=200` 且清空旧样本只留目标点，否则 `m=100`；顶点队列按 `g+h` 展开直到 `best_vertex_queue_value() > best_edge_queue_value()` 才处理最佳边；接边需三条判据同时成立（`estVertexOfVertex<g[goal]`、`estCostOfEdge<g[goal]`、`actualCostOfEdge<g[goal]`），`connect` 以 `steps=int(dist*10)` 惰性逐点查碰撞；`update_graph()` 做 Dijkstra 式松弛 | `eta=2.0`（**形参存了但实际半径用 `self.r`，未参与计算**）、`maxIter=80`、`cMin = ‖s−g‖/1.5`（**与 Informed RRT\* 定义不同**）| `BatchInformedRRTStar/batch_informed_rrt_star.py`（`RTree`+`BITStar`） |
| LQR-RRT\* | Steer 改为调用 `LQRPlanner.lqr_planning(from, to)` 得到整段轨迹，再按 `step_size` 加密，代价为各段长度之和；邻域/rewire 复用 RRT\*；目标判据 `dist_to_goal ≤ goal_xy_th` | `goal_sample_rate=10`、`max_iter=200`、`connect_circle_dist=50.0`、`step_size=0.2`、`goal_xy_th=0.5`、`curvature=1.0` | `LQRRRTStar/lqr_rrt_star.py`，`class LQRRRTStar(RRTStar)` |
| Closed-loop RRT\*（C-LQR） | 先用 RRT\*+Reeds-Shepp 建树，再对满足 `xy_th/yaw_th` 的候选路径做**闭环跟踪仿真**（纯追踪转向 + PID 速度，`unicycle_model` 积分），用三项否决：未到目标、`|Δyaw| ≥ yaw_th·10`、`travel/origin_travel ≥ invalid_travel_ratio`、跟踪轨迹碰撞；取 `t[-1]` 最小者 | `max_iter=200`、`connect_circle_dist=50.0`、`target_speed=10/3.6`、`yaw_th=deg2rad(3)`、`xy_th=0.5`、`invalid_travel_ratio=5.0`；unicycle：`dt=0.05, L=0.9, steer_max=deg2rad(40), accel_max=5.0` | `ClosedLoopRRTStar/closed_loop_rrt_star_car.py` + `pure_pursuit.py` + `unicycle_model.py` |
| RRT-Dubins / RRT\*-Dubins / RRT\*-Reeds-Shepp | 把 Steer 换成 Dubins 或 Reeds-Shepp 规划，代价为曲线段长（RS 用 `Σ|lengths|`，倒车也计正代价）；RRT\*-Dubins 复用 RRT\* 的 `choose_parent/rewire`；RRT\*-Reeds-Shepp 额外 `try_goal_path()` 每轮尝试直连目标，且是**纯均匀采样（无目标偏置）** | 三者都 `max_iter=200`、`goal_xy_th=0.5`、`goal_yaw_th=deg2rad(1.0)`、`curvature=1.0`；RS 版另有 `step_size=0.2` | `RRTDubins/`、`RRTStarDubins/`、`RRTStarReedsShepp/` |

**论文/出处**：RRT\* → *Sampling-based Algorithms for Optimal Motion Planning*（arXiv:1105.1186）与 *Incremental Sampling-based Algorithms for Optimal Motion Planning*（arXiv:1005.0416）；Informed RRT\* → arXiv:1404.2334；BIT\* → arXiv:1405.5848；LQR-RRT\* → `lis.csail.mit.edu/pubs/perez-icra12.pdf`；C-LQR → `KuwataGNC08` / `KuwataTCST09` / arXiv:1601.06326；RRT+Sobol 引 LaValle *Planning Algorithms*（2006）与 Bratley-Fox 的 ACM TOMS 算法 659（1988）、Fox 算法 647（1986）、Sobol 1977/1976。**所有这些条目在文件里都只给了标题或 URL，作者/年份多未写出 → 作者年份未确认**（少数例外：LaValle 2006、Bratley & Fox 1988 有年份）。

#### 4.2.2 路径原语：Dubins 与 Reeds-Shepp
**Dubins（`DubinsPath/dubins_path_planner.py`）**：3 段圆弧/直线，6 个 word `{LSL, RSR, LSR, RSL, RLR, LRL}`。先把终点变换到起点局部系并归一化：`d = ‖Δp‖·κ`，`θ = mod2pi(atan2(Δy,Δx))`，`α = mod2pi(−θ)`，`β = mod2pi(goal_yaw − θ)`。以 LSL 为例
`p² = 2 + d² − 2cos(α−β) + 2d(sinα − sinβ)`，`tmp = atan2(cosβ − cosα, d + sinα − sinβ)`，`d1 = mod2pi(−α + tmp)`，`d2 = √p²`，`d3 = mod2pi(β − tmp)`；RLR/LRL 用
`tmp = (6 − d² + 2cos(α−β) + 2d(sinα − sinβ))/8`（`|tmp|>1` 非法），`d2 = mod2pi(2π − acos(tmp))`。选 `cost = |d1|+|d2|+|d3|` 最小者，再还原为物理长度 `length/κ`。
签名 `plan_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, curvature, step_size=0.1, selected_types=None) -> (x_list, y_list, yaw_list, modes, lengths)`。
出处：*On Curves of Minimal Length with a Constraint on Average Curvature…*（JSTOR 2372560，**作者/年份未确认**）+ LaValle 15.3.1。

**Reeds-Shepp（`ReedsSheppPath/reeds_shepp_path_planning.py`）**：允许倒车。先归一化到单位曲率、起点在原点：`x = (cosθ0·dx + sinθ0·dy)·κ_max`、`y = (−sinθ0·dx + cosθ0·dy)·κ_max`、`dth = θ1 − θ0`、`step_size *= κ_max`。
**12 个基字**（`polar(x,y) = (hypot, atan2)`）：CSC 类 `left_straight_left`、`left_straight_right`；CCC 类 `left_x_right_x_left`、`left_x_right_left`、`left_right_x_left`、`left_right_x_left_right`、`left_x_right_left_x_right`；含 90° 段的 CCSC/CSCC 类 5 个（`left_x_right90_straight_left`、`left_straight_right90_x_left`、`left_x_right90_straight_right`、`left_straight_left90_x_right`、`left_x_right90_straight_left90_x_right`）。例如 CCCC 之一：`u2 = (20 − u1²)/16`（要求 `0 ≤ u2 ≤ 1`），`u = acos(u2)`，`A = asin(2 sin u / u1)`，`t = mod2pi(θ + A + π/2)`，`v = mod2pi(t − φ)`，返回 `[t, −u, −u, v]`。
**48 个 word 的来源**：每个基字用 4 组参数各求一次 —— 原式、`timeflip`（`(−x, y, −dth)`，段长取负=倒车）、`reflect`（`(x, −y, −dth)`，`L↔R` 互换）、两者同时。去重规则：同类且总长差 `≤ step_size` 视为重复；`L ≤ step_size` 的路径丢弃；若某段满足 `0.1·Σ|d| < |distance| < step_size` 则打印 `"Step size too large for Reeds-Shepp paths."` 并**返回空列表**。取 `min(abs(path.L))`。
签名 `reeds_shepp_path_planning(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=0.2) -> (x, y, yaw, ctypes, lengths)`，无解返回全 `None`；`class Path` 的 `lengths` 负值表示倒车，`directions` 为 1/−1。
出处：LaValle 15.3.2、*Optimal paths for a car that goes both forwards and backwards*、`ghliu/pyReedsShepp`（**作者/年份未确认**）。

#### 4.2.3 曲线与样条
| 曲线 | 关键公式 | 参数默认值 | 文件 |
|---|---|---|---|
| Bezier（4 控制点） | 控制点沿起终点航向布置：`dist = ‖Δp‖/offset`，`P0=s`，`P1 = s + dist·[cos syaw, sin syaw]`，`P2 = e − dist·[cos eyaw, sin eyaw]`，`P3=e`（`offset` 越大曲线越紧）；求值用 Bernstein 显式求和 `bezier(t) = Σ C(n,i) t^i (1−t)^{n−i} P_i`；曲率 `κ = (dx·ddy − dy·ddx)/(dx²+dy²)^{3/2}`；导数控制点逐层差分 | `n_points=100`，demo `offset=3.0` | `BezierPath/bezier_path.py` |
| B-Spline | 不用 `splprep`：沿**归一化累积弦长**分别对 x、y 做 `scipy.interpolate.UnivariateSpline(distances, x, k=degree, s=s)`；`heading = atan2(dy,dx)`；曲率 `κ = (ddy·dx − ddx·dy)/(dx²+dy²)^{2/3}`（**代码指数写 `2.0/3.0`，与 Bézier/CubicSpline 的 `3/2` 不一致，疑似笔误**） | `degree=3`（要求 `2≤k≤5`）、`s=None`（`s=0` 等价插值）；插值模式就是 `s=0.0` | `BSplinePath/bspline_path.py` |
| CubicSpline（主力工具） | 1D 自然三次样条 `S_i = a_i + b_i Δx + c_i Δx² + d_i Δx³`，`c` 由**稠密** `np.linalg.solve(A,B)` 解（A 为三对角但未用 Thomas 算法），两端自然边界（二阶导 0）；`d_i = (c_{i+1}−c_i)/(3h_i)`、`b_i = (a_{i+1}−a_i)/h_i − h_i(2c_i+c_{i+1})/3`。2D 版 `CubicSpline2D` 以累积弦长 `s` 为自变量分别插值 x、y，给出 `calc_yaw = atan2(y',x')`、`calc_curvature = (y''x' − x''y')/(x'²+y'²)^{3/2}`、`calc_curvature_rate` | `calc_spline_course(x, y, ds=0.1)` | `CubicSpline/cubic_spline_planner.py`（`CubicSpline1D`/`CubicSpline2D`）；`spline_continuity.py` 是 C0/C1/C2 对比 demo |
| Catmull-Rom | 段公式 `P(t) = 0.5(2P1 + (−P0+P2)t + (2P0−5P1+4P2−P3)t² + (−P0+3P1−3P2+P3)t³)`，过控制点的 C¹ 插值；端点复制（`i==0` 用 `p0=p1`）；**只有均匀参数化**（rst 提到的 chordal/centripetal 未实现） | `num_points` 必填；demo `n_course_point=100` | `Catmull_RomSplinePath/catmull_rom_spline_path.py` + `blending_functions.py` |
| Clothoid（回旋曲线） | G1 Hermite 单段拟合：解标量方程 `Y(2A, δ−A, φ1) = 0`（`scipy.optimize.fsolve`，初值 `3(φ1+φ2)`），其中 `X(a,b,c)=∫₀¹cos(a t²/2 + b t + c)dt`、`Y` 同理用 `sin`；再 `L = r/X(2A, δ−A, φ1)`，`κ = (δ−A)/L`，`κ' = 2A/L²`；点列 `x(s) = x0 + s·X(κ's², κs, θ0)`、`y(s) = y0 + s·Y(...)`，`s ∈ linspace(0, L, n)` | `n_path_points` 必填；`fsolve` 容差用 scipy 默认（**文件未写字面值**） | `ClothoidPath/clothoid_path_planner.py` |
| η³ 样条路径 | 每段是二元 7 次多项式 `p(u) = coeffs @ [1,u,…,u⁷]ᵀ`，系数由起终点位姿 + `eta`（6 个整形参数）+ `kappa`（`[κ_A, κ'_A, κ_B, κ'_B]`）闭式装配；弧长率 `s_dot(u) = max(‖d/du p(u)‖, 1e-6)`，段长用 `quad` 积分。**代码里没有 QP / 没有曲率连续性代价优化**——`eta/kappa` 完全由用户给定 | `eta` 长度必须 6、`kappa` 长度必须 4，默认全零；`Eta3Path` 断言相邻段首尾位姿相等 | `Eta3SplinePath/eta3_spline_path.py`（`Eta3Path`、`Eta3PathSegment`） |
| η³ 样条轨迹 | 在 η³ 路径上叠加 **7 段速度剖面**（+max_jerk→max_accel→−max_jerk→巡航→−max_jerk→−max_accel→+max_jerk 归零），无巡航时解二次方程 `a v² + b v + c = 0` 求最大可行速度并自动取消巡航段；`calc_traj_point(t)` 由时间解析求 `s,v,a`，再用牛顿法 `f(u)=f_length(u)−s`、`f'(u)=s_dot(u)`（`tol=0.001`）把弧长反解成参数 `u`，最后算 `ω = (ytt·xt − xtt·yt)/v²` | `max_vel` 必填、`v0=0.0`、`a0=0.0`、`max_accel=2.0`、`max_jerk=5.0`；异常 `MaxVelocityNotReached` | `Eta3SplineTrajectory/eta3_spline_trajectory.py` |

#### 4.2.4 轨迹生成与优化
- **五次多项式（`QuinticPolynomialsPlanner/`）**：`x(t) = a0 + a1 t + a2 t² + a3 t³ + a4 t⁴ + a5 t⁵`，其中 `a0=xs, a1=vxs, a2=axs/2`，`[a3,a4,a5]` 由 3×3 系统
  `A = [[T³,T⁴,T⁵],[3T²,4T³,5T⁴],[6T,12T²,20T³]]`、`b = [xe − a0 − a1T − a2T², vxe − a1 − 2a2T, axe − 2a2]` 解出。二维时把初末速度/加速度投影到 x、y 各解一条多项式（航向/速度/加速度/jerk 由分量合成），并**在时间网格 `T ∈ arange(MIN_T, MAX_T, MIN_T)` 上搜索**：满足 `max|a| ≤ max_accel 且 max|j| ≤ max_jerk` 即 break。参数：`MAX_T=100.0`、`MIN_T=5.0`、demo `dt=0.1, max_accel=1.0, max_jerk=0.5`。**若 19 个候选都不满足，代码不报错而是返回最后一轮（T=95）的轨迹**。出处 *Local Path planning And Motion Control For AGV In Positioning*（**作者/年份未确认**）。
- **Frenet 最优轨迹（`FrenetOptimalTrajectory/`）**★：
  - 横向：目标 `d(s)`，端点 `d(Ti)=di, d'(Ti)=d''(Ti)=0`；高速策略以**时间**为自变量建五次多项式后换算 `d'(s)=d'(t)/s'(t)`、`d''(s)=(d''(t) − d'(s)s''(t))/s'(t)²`；低速策略直接以**弧长**为自变量。
  - 纵向：速度保持用**四次**多项式 `QuarticPolynomial`（`A=[[3T²,4T³],[6T,12T²]]` 解 `a3,a4`），目标速度采样 `arange(TARGET_SPEED ± D_T_S·N_S_SAMPLE, D_T_S)`，终端代价 `K_S_DOT·(TARGET_SPEED − s'[-1])²`；汇入停车用五次多项式，终端代价 `K_S·(STOP_S − s[-1])²`。
  - 代价：`Jp = Σ d'''²`、`Js = Σ s'''²`；`lat_cost = K_J·Jp + K_T·Ti + K_D·d[-1]²`；`lon_cost = K_J·Js + K_T·Ti + calc_destination_cost`；`cf = K_LAT·lat_cost + K_LON·lon_cost`，取最小。
  - 约束筛选 `check_paths`：速度 `> MAX_SPEED`、加速度 `> MAX_ACCEL`、曲率 `> MAX_CURVATURE`、与障碍圆碰撞，分类进 `max_speed_error/max_accel_error/max_curvature_error/collision_error/ok`。
  - **笛卡尔↔Frenet 解析互转**（`cartesian_frenet_converter.py`，本项目里少见的"可直接搬走"的纯函数）：
    `d = copysign(‖Δp‖, cos(rθ)Δy − sin(rθ)Δx)`；`d' = (1−κ_r d)·tan Δθ`；`d'' = −(κ_r d)'·tan Δθ + (1−κ_r d)/cos²Δθ·(κ(1−κ_r d)/cos Δθ − κ_r)`；`s' = v cos Δθ/(1−κ_r d)`；`s'' = (a cos Δθ − s'²(d'Δθ' − (κ_r d)'))/(1−κ_r d)`；反向 `x = rx − sinθ_r d`、`y = ry + cosθ_r d`、`Δθ = atan2(d', 1−κ_r d)`。
  - 参数（默认高速分支）：`MAX_SPEED=50/3.6`、`MAX_ACCEL=5.0`、`MAX_CURVATURE=1.0`、`DT=0.2`、`MAX_T=5.0`、`MIN_T=4.0`、`N_S_SAMPLE=1`、权重 `K_J=0.1, K_T=0.1, K_S_DOT=1.0, K_D=1.0, K_S=1.0, K_LAT=1.0, K_LON=1.0`、`MAX_ROAD_WIDTH=7.0`、`D_ROAD_W=1.0`、`TARGET_SPEED=30/3.6`、`D_T_S=5/3.6`、`ROBOT_RADIUS=2.0`、`STOP_S=25.0`、`D_S=2`、`N_STOP_S_SAMPLE=4`、`SIM_LOOP=500`。出处 *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*（URL 里含 `Moritz_Werling`，**正文未写作者/年份 → 未确认**）。
- **MPTG（`ModelPredictiveTrajectoryGenerator/`）**：优化变量 `p = [s, km, kf]`（弧长 + 中间/末端曲率），残差 `[target.x − x[-1], target.y − y[-1], pi_2_pi(target.yaw − yaw[-1])]`（无权重）；中心差分雅可比 `h = [0.5, 0.02, 0.02]`；牛顿步 `dp = −J⁻¹ dc`，线搜索 `α ∈ {1.0, 1.5}`，`cost = ‖dc‖ ≤ cost_th(=0.1)` 收敛，`max_iter=100`；运动模型 `L=1.0, ds=0.1, v=10/3.6`，曲率用三点二次插值 `interp1d([0, t/2, t], [k0, km, kf], kind="quadratic")`。离线表：状态网格 `x∈{10,15,20,25}`、`y∈0:2:18`、`yaw` 由 `arange(-max_yaw, max_yaw, max_yaw)` 且默认 `max_yaw=deg2rad(-30)` 为负 → 实际只取 `[+30°, 0°]`，共 80 个状态；落盘 `lookup_table.csv`（列 `x,y,yaw,s,km,kf`，成品 82 行=表头+81 行）。**本 checkout 里不存在 `model_predictive_trajectory_generator.py`，也没有 `LookupTable` 类。** 出处 *Optimal rough terrain trajectory generation for wheeled mobile robots*（DOI 10.1177/0278364906075328，**作者/年份未确认**）。
- **LQR Planner（`LQRPlanner/lqr_planner.py`）**：状态是**相对目标的误差** `x = [sx−gx, sy−gy]ᵀ`；`A = [[DT, 1.0], [0.0, DT]]`、`B = [0, 1]ᵀ`（**照抄：A 的对角是 DT、非对角是 1.0，与常见 `[[1,dt],[0,1]]` 不同**）；`solve_dare` 全程用 `*` 逐元素乘（与标准 DARE 不同），增益 `K = inv(BᵀXB + R)BᵀXA`；控制 `u = −Kx`，且**每个仿真步都重解一次 DARE**（不缓存）。参数 `MAX_TIME=100.0, DT=0.1, GOAL_DIST=0.1, MAX_ITER=150, EPS=0.01`，`Q=eye(2), R=eye(1)`。复杂度最坏约 `1000 × 150` 次 2×2 矩阵迭代。出处仅 `# ref Bertsekas, p.151`（书名/年份未确认）。
- **PSO（`ParticleSwarmOptimization/`）**：粒子位置就是候选路径点。`v = w·v + c1 r1 (pbest − x) + c2 r2 (gbest − x)`，`x ← x + v` 并逐维裁剪；`w = w_start − (w_start − w_end)·iter/max_iter`；速度上限 `= 0.05·(搜索域宽)`，初速度 `randn(2)·0.1`；适应度 `f = ‖x − target‖ + Σ penalty`，罚项 `1000`（障碍内）/`50/(d − r + 0.1)`（`d < r+5` 环带）/`0`；线段-圆碰撞用解析二次方程判别；碰撞时把速度 `×0.2`。参数：`w_start=0.9, w_end=0.4, c1=1.5, c2=1.5`；demo `N_PARTICLES=15, MAX_ITER=150, SEARCH_BOUNDS=[(-50,50),(-50,50)], TARGET=[40,35], OBSTACLES=[(10,15,8),(-20,0,12),(20,-25,10),(-5,-30,7)]`。出处：Kennedy & Eberhart 1995；Shi & Eberhart 1998；Clerc & Kennedy 2002。

---
### 4.3 PathPlanning（三）：动力学约束、动态障碍、覆盖与反应式规划

#### 4.3.1 动态障碍环境与时空规划（`PathPlanning/TimeBasedPathPlanning/`）★ 与本项目最相关

**公共数据结构（`Node.py` / `BaseClasses.py` / `GridWithDynamicObstacles.py`）**
- `Position`：`@dataclass(order=True)` 的整数格 `(x, y)`，支持 `+`/`-`/`hash`。
- `Node`：`(position, time, heuristic, parent_index)`，**排序键 `f = time + heuristic`（`__lt__`）**，而 `__eq__`/`__hash__` 只看 `(position, time)`（源码注释：`heuristic/parent_index` 对同一 `(position,time)` 必然相同，`parent_index` 仅用于回溯）。
- `NodePath`：`path`、`positions_at_time`（把停留/等速段展开成"每个时刻的占用位置"）、`expanded_node_count`；`get_position(t)`、`goal_reached_time()`。
- `StartAndGoal(index, start, goal)`，`distance_start_to_goal()` 返回**平方欧氏距离（未开方）**。
- `BaseClasses.py` 在导入时执行 `random.seed(50)` 与 `numpy.random.seed(50)`（`RANDOM_SEED = 50`），**障碍生成因此可复现**。
- `Grid`：`reservation_matrix = np.zeros((grid_size[0], grid_size[1], time_limit))`，**值为 0 表示空闲，障碍物用 `i+1`，多智能体用 `StartAndGoal.index`**。`Interval(start_time, end_time)` 是**闭区间**（`reserve_position` 用 `range(start, end+1)`）。
- 障碍运动生成器 `ObstacleArrangement`：`RANDOM`（随机游走，权重 `[0.05,0.05,0.05,0.05,0.8]`，即以 0.8 概率原地不动）、`ARRANGEMENT1`（y 方向排成一行、x 方向来回移动，`t % 2 == 0` 时原地）、`NARROW_CORRIDOR`（x 取中线的静态窄通道）。
- **关键碰撞语义**：写入预约矩阵时，时刻 t 同时占用"上一时刻所在格"和"当前格"（源码：`reservation_matrix[path[t-1].x, path[t-1].y, t] = obs_idx` 与 `reservation_matrix[position.x, position.y, t] = obs_idx`）；因此 agent 必须 `t+1` 与 `t+2` 两步都空闲才算能进入一格。等价说法：一格被占用的时间是"进入那一步"与"离开那一步"。
- `Grid.valid_position(pos, t)`：界内且 `reservation_matrix[pos.x, pos.y, t] == 0`。
- `get_safe_intervals_at_cell(cell)`：对 `reservation_matrix[x,y,:]` 的零游程做 `np.diff` 切分，得到按 `start_time` 升序的 `Interval` 列表，并**丢弃 `start_time == end_time` 的单步区间**（自由只有 1 步不够"进+出"）。
- 多机预约：`reserve_path` 把每个节点在 `[node.time, 下一个节点.time]` 上预约；`reserve_position` 要求 `agent_index != 0` 且不与他人冲突，冲突直接抛异常；`clear_initial_reservation` 清除该 agent 的初始占位。
- 可视化：`Plotting.PlotNodePath` / `PlotNodePaths` 逐时刻 `plt.pause(0.2)`，按 `path.get_position(t)` 更新，Esc 退出（`PlotNodePaths` 内还有 `assert not path_position in obs_positions` 的一致性断言）。

**Space-time A\*（`SpaceTimeAStar.py`）**
- **原理/公式**：搜索状态是三维 `(x, y, t)`；**代价 `g(n) = n.time`（时间步数，不是格子数）**，每步（含等待）恰好消耗 1 个时间步，`new.time = parent.time + 1`。启发式
  `h(n) = |Δx| + |Δy|`（曼哈顿），因为动作集含"原地等待"，每步最多缩短 1 格曼哈顿距离，所以 h 可采纳且一致；又因 g 的单位就是时间，h 同时等于"剩余最少时间"（文档亦如此说明）。
- **动作集**：`[Position(0,0), (1,0), (-1,0), (0,1), (0,-1)]`（4 连通 + 等待）。
- **碰撞检查**：`all(grid.valid_position(new_pos, parent.time + dt) for dt in [1, 2])`——进入与离开两步都必须空闲。
- **步骤**：① `heapq` 压入 `Node(start, 0, h(start), -1)`；② 弹出最小 f；③ `time + 1 >= time_limit` 则跳过；④ `position == goal` 即沿 `parent_index` 在 `expanded_list` 里回溯并反转；⑤ 记入 `expanded_list` 与 `expanded_set`（本 PR 新增的去重集合，去重键为 `(position,time)`），并以其下标作为子节点 `parent_index`；⑥ 生成后继入堆；⑦ open 空则 `raise Exception("No path found")`。
- **参数默认值**：`main()` 中 `start = Position(1, 5)`、`goal = Position(19, 19)`、`grid_side_length = 21`、`num_obstacles = 40`、`obstacle_arrangement = ARRANGEMENT1`、`time_limit = 100`（`Grid` 默认）、`verbose = False`。
- **复杂度**：代码未声明。按状态空间推导（**推导，非源码声明**）为 `V = W·H·T`、`E ≈ 5V`、`O(E log V)`、内存 `O(V)`。文档给出的实测：加去重集合前 204490 次扩展 / 1.72464 s，加之后 2348 次扩展 / 0.01550 s（`docs/.../time_based_grid_search_main.rst`）。
- **出处**：`https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf`（*Cooperative Pathfinding*，David Silver；**年份未写出 → 未确认**）；去重优化对应 PR #1183。

**SIPP 安全区间规划（`SafeInterval.py`）**★
- **原理/公式**：先对每格预计算"安全时间区间"，节点额外携带所属区间：`SIPPNode(Node)` 多一个 `interval` 字段。剪枝判据是关键不等式：**同一区间内若已扩展过入场时间 `≤ parent.time + 1` 的节点，则跳过**（`visited_intervals[x][y]` 里逐条比较 `interval == visited.interval and visited.entry_time <= parent.time + 1`）。入场时刻取"尽可能早"：`max(interval.start_time, parent.time + 1)`。
- **步骤**：① `grid.get_safe_intervals()`；② 根节点取起点格"第一个（最早）区间"入堆；③ 弹出后同样做 `time+1 >= time_limit` 剪枝与 goal 判定，并把 `EntryTimeAndInterval(node.time, node.interval)` 记入 `visited_intervals`；④ `generate_successors`：对每个动作，遍历目标格的安全区间，跳过"区间起点晚于当前区间终点"（后续更晚，直接 break）与"区间终点早于当前区间起点"，再做上面的 visited 剪枝；⑤ 在 `[max(parent.time+1, interval.start), min(current_interval.end, interval.end))` 内找最早可行时刻并生成节点后立即 break（后续时刻代价更高、启发式相同、同属一个区间）。
- **参数默认值**：`main()` 中 `start = Position(1, 18)`、`goal = Position(19, 19)`、`grid_side_length = 21`、`num_obstacles = 250`、ARRANGEMENT1；`expanded_node_count = len(expanded_list)`（与时空 A\* 用 `len(expanded_set)` 不同）。
- **复杂度**：代码未声明；文档实测优势巨大——同场景（ARRANGEMENT1，起点 (1,18)）SIPP 322 次扩展 / 0.00730 s，而时空 A\* 2717154 次扩展 / 20.51330 s；250 个随机障碍时 SIPP 764 次扩展 / 0.60596 s。
- **实测注意点（读代码时值得知道）**：① 根节点取"第一个区间"但未校验它是否覆盖 `t=0`；② `add_entry_to_visited_intervals_array` 中的 `append` 在 for 循环之外、**无条件执行**（注释写 "Otherwise" 但没有 else），同一区间可能累积多条记录（去重判断仍有效）；③ `for possible_t in ...` 的循环下界就是写入节点的时刻，正常输入下总在第一次迭代 break。
- **出处**：`https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf`（*SIPP: Safe Interval Path Planning for Dynamic Environments*，文件名暗示 ICRA 2011；**作者/年份未在文件中给出 → 未确认**）。

**优先级多机规划（`PriorityBasedPlanner.py`）**
- **原理**：按 start→goal 距离**降序**（用 `distance_start_to_goal()` 的平方距离排序）依次为每个 agent 规划；规划前先用 `Interval(0, 10)` 给所有 agent 的起点占位，轮到某 agent 时 `clear_initial_reservation`，规划完成后 `reserve_path` 把路径写进预约矩阵，后续 agent 必须避开。
- **输入/输出**：`plan(grid, start_and_goals, single_agent_planner_class, verbose=False) -> (list[StartAndGoal], list[NodePath])`；**返回的 `StartAndGoal` 列表已按距离降序重排**，`paths[i]` 与返回列表同序（不是与传入顺序对应）。
- **参数默认值**：占位窗口 `Interval(0, 10)`；`main()` 用 `[StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 16)]`（15 台）、`num_obstacles = 250`、单机规划器传 `SafeIntervalPathPlanner`。
- **次优性（文档明确）**：路径一旦找到就不重规划，既不保证任何单机器人的最短路径，也不保证整体时间/运动量最小 → **它不是 CBS（冲突搜索）**。全仓库 grep `cbs|conflict` 无命中，**本项目没有 CBS/ECBS 实现**。
- **出处**：`https://pure.tudelft.nl/ws/portalfiles/portal/67074672/07138650.pdf`，文档给出标题 *Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots*（**作者/年份未给出 → 未确认**）。

#### 4.3.2 Hybrid A\*（`PathPlanning/HybridAStar/`）
- **原理/公式**：状态离散化为 `(x_ind, y_ind, yaw_ind) = (round(x/XY_RES), round(y/XY_RES), round(yaw/YAW_RES))`，线性索引
  `ind = (yaw_ind − min_yaw)·x_w·y_w + (y_ind − min_y)·x_w + (x_ind − min_x)`。
- **运动基元**：`steer ∈ linspace(-MAX_STEER, MAX_STEER, N_STEER) ∪ {0}` × `d ∈ {+1, −1}`，即每个节点 **42 个候选**；每段弧长 `arc_l = XY_GRID_RESOLUTION·1.5`，按 `MOTION_RESOLUTION` 积分；车辆模型（后轴参考点）
  `x += s·cosθ`，`y += s·sinθ`，`θ = pi_2_pi(θ + s·tanδ/WB)`。
- **代价**：`cost = parent.cost + added_cost + arc_l`，其中 `added_cost` 累加：换向 `SB_COST`、`STEER_COST·|δ|`、`STEER_CHANGE_COST·|δ − δ_parent|`。
- **启发式**：从 goal 反向跑**栅格 Dijkstra**（8 邻域，直行 1 / 对角 √2，障碍按机器人半径膨胀）得到 `h_dp`；`calc_cost = n.cost + H_COST·h_dp[ind]`，`ind` 不在表中则加 `999999999` 视为碰撞。
- **解析扩展**：用 Reeds-Shepp 曲线直接连到 goal，最大曲率 `max_curvature = tan(MAX_STEER)/WB`，取无碰撞且 `calc_rs_path_cost` 最小的一条；RS 代价对倒车段乘 `BACK_COST`、相邻段变号加 `SB_COST`、非直线段加 `STEER_COST·MAX_STEER`、相邻转向差加 `STEER_CHANGE_COST`。
- **碰撞检测**：`BUBBLE_DIST = (LF−LB)/2`、`BUBBLE_R = hypot((LF+LB)/2, W/2)`；先用 `cKDTree.query_ball_point` 在 bubble 圆心处粗筛，再用 `rectangle_check` 把障碍变换到车体系做精确 OBB 判定。
- **参数默认值**（`hybrid_a_star.py`）：`XY_GRID_RESOLUTION = 2.0`、`YAW_GRID_RESOLUTION = deg2rad(15.0)`、`MOTION_RESOLUTION = 0.1`、`N_STEER = 20`、`SB_COST = 100.0`、`BACK_COST = 5.0`、`STEER_CHANGE_COST = 5.0`、`STEER_COST = 1.0`、`H_COST = 5.0`；`car.py`：`WB = 3.0`、`W = 2.0`、`LF = 3.3`、`LB = 1.0`、`MAX_STEER = 0.6`。
- **输出**：`hybrid_a_star_planning(start, goal, ox, oy, xy_resolution, yaw_resolution) -> Path(x_list, y_list, yaw_list, direction_list, cost)`；无解时打印 `"Error: Cannot find path, No open set"` 并返回空 `Path`。open/closed 用 dict + `heapq` 惰性删除（弹出时不在 openList 就 `continue`）。
- **复杂度**：未确认（状态上限约 `x_w·y_w·yaw_w`，每节点 42 个基元 + RS 解析曲线）。**文件与 rst 都没有论文引用**（只有作者 Zheng Zh）→ 论文未确认。

#### 4.3.3 状态栅格规划（`StateLatticePlanner/` + `ModelPredictiveTrajectoryGenerator/`）
- **原理**：采样终端状态 `(x_f, y_f, yaw_f)`，查 `lookup_table.csv`（列注释 `# x, y, yaw, s, km, kf`）找最近行作初值，再调用 MPTG 解两点边值问题；近似度量 `d = sqrt(dx² + dy² + dyaw²)`。
- **采样函数**：均匀极坐标 `calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max)`；偏置极坐标 `calc_biased_polar_states(goal_angle, ns, nxy, nh, ...)`（用 `cnav = π − |a_i − goal_angle|` 归一化后取累积分布分位数）；车道采样 `calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy)`（`xf = xc − δ·sin(l_heading)`、`yf = yc + δ·cos(l_heading)`、`yawf = l_heading`）。
- **MPTG**：牛顿法 `dp = −inv(J)·dc`，步长由 `selection_learning_param` 选 `alpha`，收敛判据 `‖dc‖ ≤ cost_th`；参数 `max_iter = 100`、`h = [0.5, 0.02, 0.02]`、`cost_th = 0.1`；`motion_model.py`：`L = 1.0`、`ds = 0.1`、`v = 10.0/3.6`。
- **注意**：该文件**只做采样 + 边值求解 + 可视化，不做障碍碰撞检测、也不做图搜索**（真正的搜索/碰撞检查需要使用者自己接）。复杂度未确认（每条候选最多 100 次迭代 × 数值雅可比）。
- **出处**：*State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments* 与 *Optimal rough terrain trajectory generation for wheeled mobile robots*（**均无作者/年份 → 未确认**）。

#### 4.3.4 反应式/局部方法
| 算法 | 原理与关键公式 | 关键参数默认值 | 备注 |
|---|---|---|---|
| DWA | 速度窗口 `dw = [max(Vs,Vd)…]` 取本体极限与加速度可达范围的交；对窗口内每个 `(v, ω)` 前向积分 `predict_time`；代价 `J = to_goal·Δθ + speed·(v_max − v_end) + obstacle·(1/min_r)`；矩形机器人用 OBB 判定、圆形用 `r <= robot_radius`，碰撞返回 `Inf`；速度≈0 且朝向无差时强制 `ω = −max_delta_yaw_rate` 破卡死 | `max_speed=1.0`、`min_speed=-0.5`、`max_yaw_rate=40°/s`、`max_accel=0.2`、`max_delta_yaw_rate=40°/s`、`v_resolution=0.01`、`yaw_rate_resolution=0.1°/s`、`dt=0.1`、`predict_time=3.0`、`to_goal_cost_gain=0.15`、`speed_cost_gain=1.0`、`obstacle_cost_gain=1.0`、`robot_radius=1.0`、`robot_width=0.5`、`robot_length=1.2` | 状态 `x=[x,y,yaw,v,ω]`；`motion()` **先更新 yaw 再更新位置**；障碍是静态点表，靠每步重评实现反应式避障；文件无参考文献，rst 引 *The Dynamic Window Approach to Collision Avoidance*（URL 暗示 Fox 等 1997，**年份未写出 → 未确认**） |
| 人工势场 | 引力 `U_att = 0.5·KP·‖p − g‖`（**线性距离，非平方**）；斥力 `U_rep = 0.5·ETA·(1/dq − 1/rr)²`（`dq ≤ rr`，`dq` 下限截断到 0.1），`dq` 为最近障碍距离、`rr` 为影响半径；总场 `U = U_att + U_rep`，8 邻域梯度下降；振荡检测窗口 `OSCILLATIONS_DETECTION_LENGTH` | `KP = 5.0`、`ETA = 100.0`、`AREA_WIDTH = 30.0`、`OSCILLATIONS_DETECTION_LENGTH = 3`、`reso = 0.5`、`rr = 5.0`；`sx,sy,gx,gy = 0,10,30,30` | `potential_field_planning(sx,sy,gx,gy,ox,oy,reso,rr) -> (rx,ry)`；构建复杂度约 `O(W·H·|O|)`（推导）；无独立 rst，只有 grid_base_search 里一节；出处为 CMU 课件 |
| 流场 | 三层场：cost field（`free=1/medium=7/hard=20`）→ integration field（从目标 BFS，四邻接 `e_cost=10`、对角 `14`）→ vector field（指向 3×3 邻域 integration 最小者）；从起点沿向量场走到目标 | `limit_x = limit_y = 50`、地图 51×51、起终点 `(5,5)/(35,45)` | `FlowField(obs_grid, goal_x, goal_y, start_x, start_y, limit_x, limit_y)`；**队列是 list+`del [0]` 的 FIFO，不是优先队列**；`docs/` 下完全没有 FlowField 页面；出处为博客 `leifnode.com`（非论文） |
| Bug 系列 | 无障碍时按 `sign(goal − p)` 贪心走；遇障沿边界绕行。Bug0：绕到能再次贪心；Bug1：绕一圈回到撞击点、再去离目标最近点、第二圈离开；Bug2：先记录起点-目标直线上的 hit 点，绕行时再次遇到 hit 点即离开 | `main(bug_0, bug_1, bug_2)`；默认障碍为若干矩形块；起点 `(0,0)`、终点 `(167,50)`；文件末尾 `main(bug_0=True, bug_1=False, bug_2=False)` | 边界点集 = 障碍格的 8 邻域自由格；绕行方向固定为东→北→西→南；循环无迭代上限；出处为课程存档链接 |
| 弹性带 | 气泡 `B(b) = {q : ‖q−b‖ < ρ(b)}`，`ρ` 来自距离场（SDF）查表；内部收缩力 `f_c = k_c·(û(prev−b) + û(next−b))`（两端为零）；外部斥力 `f_r = k_r(ρ_0 − ρ)∇ρ`（`ρ<ρ_0`，梯度用中心差分，代码写成 `(ρ(b−h)−ρ(b+h))/2h` 即 `−∇ρ`，与 `k_r = −0.1` 相乘后指向远离障碍）；更新用切向剔除后的合力 `new_pos = b + α·f*`，`α = ρ(b)`，`clip` 到 `[0,499]`；重叠维护：太远则插点、太近则删点 | `MAX_BUBBLE_RADIUS=100`、`MIN_BUBBLE_RADIUS=10`、`RHO0=20.0`、`KC=0.05`、`KR=-0.1`、`LAMBDA=0.7`、`STEP_SIZE=3.0`、`MAX_ITER=50`；工作区 500×500 | 切向剔除在代码里是**逐分量**写法的近似（`f − f·v·v/‖v‖²`），与文档的投影式不同；文档 `elastic_bands_main.rst` 给了完整公式；出处 *Elastic Bands: Connecting Path Planning and Control*（**作者/年份未确认**） |

#### 4.3.5 覆盖规划（CPP）
| 算法 | 原理与机制 | 参数默认值 | 测试断言 |
|---|---|---|---|
| 栅格牛耕式（GridBasedSweepCPP） | 取多边形**最长边**定扫描方向 `th = atan2(dy,dx)`，用 `rot_mat_2d(±th)` 在世界/扫描坐标间转换；建 `GridMap` 并用 `set_value_from_polygon(..., inside=False)` + `expand_grid()`；覆盖状态用 `FloatGrid(0.5)` 标记，真障碍判定用 `FloatGrid(1.0)`；`SweepDirection.UP/DOWN`、`MovingDirection.RIGHT/LEFT`；转弯窗口（源码拼写 `turing_window`）依次尝试"直行/斜前/横移/斜后"，都失败则反向一步，反向也不行则终止 | `planning(ox, oy, resolution, moving_direction=RIGHT, sweeping_direction=UP)`、`offset_grid=10`、`do_animation=True`；`main()` 三个场景的 `resolution = 5.0/1.3/5.0` | `tests/test_grid_based_sweep_coverage_path_planner.py` 四种方向组合，断言 `len(px) >= 5`（**不验证覆盖率**） |
| 螺旋生成树（SpiralSpanningTreeCPP） | 2×2 合并地图（宽高必须是偶数，否则 `sys.exit`），合并节点有效 = 4 个子格全自由；递归 DFS 生成生成树，邻接顺序 `[[1,0],[0,1],[-1,0],[0,-1]]`；`visit_times` 计数，回退时跳过已访问 2 次的节点（形成环路往返）；相邻 route 节点曼哈顿距离 0/1/2 分别用原地往返/单步/公共邻居处理，>2 直接 `sys.exit` | `do_animation = True`；地图 `map/test{,_2,_3}.png`，起点如 `(10, 0)` | `tests/test_spiral_spanning_tree_coverage_path_planner.py` 断言 `len(covered_nodes) == num_free / 4`（**验证完整覆盖**） |
| 波前覆盖（WavefrontCPP） | 先做变换：`distance` 用全 0 的 `eT`，`path` 用 `ndimage.distance_transform_cdt`；BFS 松弛 `T(i,j) = min(T(i,j), T(ni,nj) + cost[k] + alpha·eT[ni,nj])`，8 邻接代价 chessboard=1、`'eculidean'`（源码拼写）= {1, √2}；再从起点沿变换场"爬山"（从路径末尾往回找最大 `T` 的未访问邻居），卡住时回溯 | `transform(grid_map, src, distance_type='chessboard', transform_type='path', alpha=0.01)`；`start=(43,0)`、`goal=(0,0)`；`do_animation=True` | `tests/test_wavefront_coverage_path_planner.py` 断言 `len(DT_path) == num_free` 与 `len(PT_path) == num_free`（**验证每个自由格恰好访问一次**） |

出处：Spiral-STC 标注 *Gabriely et al.*，Wavefront 标注 *Zelinsky et al.*（均只给链接、**年份未给出 → 未确认**）；栅格牛耕式**完全没有论文引用**。

> PSO 路径规划已在 §4.2.4 末尾详述（公式、罚分分档、全部默认参数与引用），此处不重复。

---
### 4.4 PathTracking（路径跟踪 / 控制）

**共同约定**：车辆模型都是**后轴参考点的自行车模型**，状态 `[x, y, yaw, v]`（纯追踪/Stanley 另有 `direction`），输入为 `(加速度 a, 前轮转角 δ)`：
`x += v·cos(yaw)·dt`，`y += v·sin(yaw)·dt`，`yaw += v/L·tan(δ)·dt`，`v += a·dt`；航向一律用 `utils.angle.angle_mod` 归一化。**注意各文件轮距 L 不一致**（纯追踪 `WB=2.9`、Stanley `L=2.9`、`lqr_steer_control` 的 `L=0.5`、MPC 的 `WB=2.5`、C/GMRES 的 `WB=0.25`），搬用时必须重新标定。

| 控制器 | 控制律与关键公式 | 关键参数默认值 | 文件 |
|---|---|---|---|
| 纯追踪 Pure Pursuit | 后轴（代码实际用**车辆中心按 WB/2 反推**的 `rear_x/rear_y`）到前视点的距离 `Lf = k·v + Lfc`；`α = atan2(ty−y, tx−x) − yaw`；`δ = direction·atan2(2·WB·sin α / Lf, 1)`；速度用**比例控制**（docstring 写 PID，实现只有 P：`a = Kp(target − v)`）；支持倒车模式（`is_reverse_mode`）与暂停仿真 | `k=0.1`（前视增益）、`Lfc=2.0`、`Kp=1.0`、`dt=0.1`、`WB=2.9`、`MAX_STEER=π/4`、`target_speed=10/3.6`、`T=100.0` | `pure_pursuit/pure_pursuit.py` |
| Stanley | 用**前轴**位置 `f = (x + L cos yaw, y + L sin yaw)` 的横向误差：`error = ⟨(f − p_nearest), [−cos(yaw+π/2), −sin(yaw+π/2)]⟩`；控制律 `δ = θ_e + atan2(k·error, v)`，其中 `θ_e = angle_mod(cyaw[idx] − yaw)`；速度比例控制 | `k=0.5`、`Kp=1.0`、`dt=0.1`、`L=2.9`、`max_steer=deg2rad(30)`、`target_speed=30/3.6`、`max_simulation_time=100.0` | `stanley_control/stanley_control.py` |
| LQR 转向控制 | 状态为误差向量 `x = [e, ė, θ_e, θ̇_e]ᵀ`（`e` 为带符号横向误差，`θ_e = yaw − cyaw`）；`A[0,1]=dt, A[1,2]=v, A[2,3]=dt`，`B[3,0] = v/L`；自写 DARE 迭代 `X ← AᵀXA − AᵀXB(R+BᵀXB)⁻¹BᵀXA + Q`（`max_iter=150, eps=0.01`），增益 `K = inv(BᵀXB+R)BᵀXA`；`δ = δ_ff + δ_fb`，**前馈** `δ_ff = atan2(L·κ, 1)`（κ 为最近点曲率），**反馈** `δ_fb = −Kx`；最近点索引处的误差符号由 `angle = angle_mod(cyaw − atan2(dy,dx))`，`angle < 0` 时 `mind *= -1` | `Q=eye(4)`、`R=eye(1)`、`dt=0.1`、`L=0.5`、`max_steer=deg2rad(45)`、`T=500.0`、`goal_dis=0.3`、`stop_speed=0.05`、`Kp=1.0` | `lqr_steer_control/lqr_steer_control.py` |
| LQR 速度+转向 | 误差状态维数升到 5（含速度误差），输入 2 维 `(a, δ)`；同样是 DARE + 前馈曲率 | `lqr_Q=eye(5)`、`lqr_R=eye(2)`、`dt=0.1`、`L=0.5`、`max_steer=deg2rad(45)` | `lqr_speed_steer_control/lqr_speed_steer_control.py` |
| 迭代线性 MPC | 每步用当前状态线性化，构造以 `[x;u]` 为变量的 QP：状态 `NX=4`、输入 `NU=2`、时域 `T=5`；目标含状态代价 `Q`、终端代价 `Qf=Q`、输入代价 `R`、输入变化率代价 `Rd`，并加约束 `MAX_STEER/MAX_DSTEER/MAX_SPEED/MIN_SPEED/MAX_ACCEL`；用 **cvxpy + CLARABEL** 求解；外迭代 `MAX_ITER=3` 直到 `max|Δu| < DU_TH` | `R=diag([0.01,0.01])`、`Rd=diag([0.01,1.0])`、`Q=diag([1.0,1.0,0.5,0.5])`、`GOAL_DIS=1.5`、`STOP_SPEED=0.5/3.6`、`MAX_TIME=500.0`、`DU_TH=0.1`、`TARGET_SPEED=10/3.6`、`N_IND_SEARCH=10`、`DT=0.2`、`WB=2.5`、`MAX_SPEED=55/3.6`、`MIN_SPEED=-20/3.6`、`MAX_ACCEL=1.0` | `model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py` |
| C/GMRES 非线性 MPC | 连续时间最优控制问题，用 **C/GMRES** 在线迭代求解（`input_num=6`，含 `u1..u3, ω1..ω3` 与 2 个拉格朗日乘子）；代价含跟踪误差 `zeta=100` 与输入惩罚 `PHI_V=PHI_OMEGA=0.01`；离散化步长 `ht=0.01`、预测时域 `tf=3.0`、`N=10`、`alpha=0.5`、收敛阈值 `0.001`、`max_iteration=60` | `U_A_MAX=1.0`、`U_OMEGA_MAX=deg2rad(45)`、`WB=0.25`；主循环 `dt=0.1`、`iteration_time=150.0` | `cgmres_nmpc/cgmres_nmpc.py` |
| 后轮反馈控制 | `th_e = angle_mod(yaw − yaw_ref)`，路径曲率 `k`、带符号横向误差 `e`：<br>`ω = v·k·cos(th_e)/(1 − k·e) − KTH·|v|·th_e − KE·v·sin(th_e)·e/th_e`<br>再 `δ = atan2(L·ω/v, 1)`（即 `tan δ = Lω/v`）；第一项是**曲率前馈**（含 `1−k·e` 几何修正），后两项是 Lyapunov 型阻尼；路径用 `scipy.interpolate.CubicSpline` + `scipy.optimize.fmin_cg` 找最近点 | `Kp=1.0`、`KTH=1.0`、`KE=0.5`、`dt=0.1`、`L=2.9`、`T=500.0`、`goal_dis=0.3`、`target_speed=10/3.6` | `rear_wheel_feedback_control/rear_wheel_feedback_control.py` |
| 移动到指定位姿 | 极坐标控制律：`ρ = ‖Δp‖`、`α = angle_mod(atan2(Δy,Δx) − θ)`、`β = angle_mod(θ_goal − θ − α)`；`v = Kp_ρ·ρ`、`ω = Kp_α·α − Kp_β·β`；当 `|α| > π/2` 时改取反向目标并把 `v` 取负（**速度符号由初始 α 决定后保持不变**，避免摆动）；先限幅再**先更新 θ、再用新 θ 更新位置** | `PathFinderController(9, 15, 3)`、`dt=0.01`、`MAX_SIM_TIME=5`、`MAX_LINEAR_SPEED=15`、`MAX_ANGULAR_SPEED=7`、收敛 `ρ > 0.001` | `move_to_pose/move_to_pose.py`（robot 版 `move_to_pose_robot.py` 加了 `Robot` 类与多控制器 `(5,8,2)/(5,16,4)/(10,25,6)`，`AT_TARGET_ACCEPTANCE_THRESHOLD=0.01`） |

**论文/出处**：纯追踪与 LQR 转向只引 *A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles*（arXiv:1604.07446）与 Apollo/Wikipedia（**作者/年份未确认**）；Stanley → *Stanley: The robot that won the DARPA grand challenge* 与 *Automatic Steering Methods for Autonomous Automobile Path Tracking*；MPC → `grauonline.de` 的实时 MPC 教程页；move_to_pose → P. I. Corke, *Robotics, Vision and Control*（Springer 2017, p102，**年份来自文档，可确认**）；后轮反馈 → 同一篇 arXiv:1604.07446。

### 4.5 Localization（定位 / 状态估计）

| 滤波器 | 模型与公式 | 关键参数默认值 | 文件 |
|---|---|---|---|
| EKF | 4 维状态 `[x,y,yaw,v]`，控制 `u=[v,ω]`；运动 `x_{t+1} = x_t + [v cosθ, v sinθ, ω]dt`；雅可比 `G = I + ∂f/∂x·dt`、`V = ∂f/∂u·dt`；`P ← GPGᵀ + V·Q·Vᵀ`；观测 GPS `H = [I₂ 0]`，`K = PHᵀ(HPHᵀ+R)⁻¹`，`x ← x + K(z − Hx)`，`P ← (I−KH)P` | `Q=diag([0.1,0.1,deg2rad(1),1])²`、`R=diag([1,1])²`、`INPUT_NOISE=diag([1,deg2rad(30)])²`、`GPS_NOISE=diag([0.5,0.5])²`、`DT=0.1`、`SIM_TIME=50.0` | `extended_kalman_filter/extended_kalman_filter.py` |
| EKF（速度尺度因子校正） | 5 维状态 `[x,y,yaw,v,s]`，`s` 为速度尺度因子；真值用 `true_scale_factor=0.9` 模拟里程计标定误差 | `Q=diag([0.1,0.1,deg2rad(1),0.4,0.1])²`、`R=diag([0.1,0.1])²`、`INPUT_NOISE=diag([0.1,deg2rad(5)])²`、`GPS_NOISE=diag([0.05,0.05])²` | `extended_kalman_filter/ekf_with_velocity_correction.py` |
| UKF | 无迹变换：sigma 点 `χ₀ = x`、`χ_i = x ± √((n+λ)P)_i`，权重 `W₀^m = λ/(n+λ)`、`W₀^c = λ/(n+λ) + (1−α²+β)`、`W_i = 1/(2(n+λ))`；预测/更新按标准 UT | `ALPHA=0.001`、`BETA=2`、`KAPPA=0`，Q/R/DT/SIM_TIME 同 EKF | `unscented_kalman_filter/unscented_kalman_filter.py` |
| 容积卡尔曼滤波 CKF | 用 **CTRV**（恒定转率+速度）模型：状态 `[x,y,yaw,v,ω]`；用 2n 个容积点做三阶球面-径向积分，`P = LLᵀ`，`χ_i = x ± √n·L_i`，权重全为 `1/(2n)`；观测 `[x,y,v,ω]` | `dt=0.1`、`N=100`、`x_0=[0,0,0,1.0,0.1]`、`p_0=diag([1e-3,1e-3,1,1,1])`、`q=diag([1e-11,1e-11,deg2rad(1e-4),1e-4,deg2rad(1e-4)])`、`r=diag([0.015,0.010,0.1,0.01])²` | `cubature_kalman_filter/cubature_kalman_filter.py` |
| 集合卡尔曼滤波 EnKF | 用 `NP` 个随机集合样本传播并统计样本协方差（无需雅可比）；**无重采样** | `NP=20`、`DT=0.1`、`SIM_TIME=50.0`、`MAX_RANGE=20.0`；仿真噪声 `Q_sim=diag([0.2,deg2rad(1)])²`、`R_sim=diag([1,deg2rad(30)])²` | `ensemble_kalman_filter/ensemble_kalman_filter.py` |
| 粒子滤波 PF | 状态 `[x,y,yaw]`，观测为到 RFID/路标距离 `w = exp(−(z−ẑ)²/(2Q))/(√(2πQ))`；归一化后按 `n_eff < NTh` 重采样 | `NP=100`、`NTh=NP/2=50`、`Q=diag([0.2])²`、`R=diag([2,deg2rad(40)])²`、`DT=0.1`、`SIM_TIME=50.0`、`MAX_RANGE=20.0` | `particle_filter/particle_filter.py` |
| 直方图滤波 | 2D 网格贝叶斯滤波（x、y 未知、yaw 已知）：预测 `bel(x,y) ← Σ p(x,y|x',y',u)·bel(x',y')` 用 `MOTION_STD` 高斯核；更新 `bel ← bel·p(z|x,y)` 用 `RANGE_STD` 高斯似然；网格 60×60 | `XY_RESOLUTION=0.5`、`MIN_X=-15.0, MIN_Y=-5.0, MAX_X=15.0, MAX_Y=25.0`、`EXTEND_AREA=10.0`、`DT=0.1`、`MAX_RANGE=10.0`、`MOTION_STD=1.0`、`RANGE_STD=3.0`、`NOISE_RANGE=2.0`、`NOISE_SPEED=0.5` | `histogram_filter/histogram_filter.py` |
| GPS+IMU 融合（含零偏） | **8 维状态** `[p_x,p_y,v_x,v_y,ψ,b_ax,b_ay,b_ω]`（无零偏时退化为 5 维）；体系加速度转世界 `a_w = R(ψ)(a_m − b_a)`；预测 `p += v dt + ½a_w dt²`、`v += a_w dt`、`ψ = wrap(ψ + (ω_m − b_ω)dt)`、零偏随机游走；雅可比按块填 `F[:2,2:4]=I·dt`、`F[:2,4]=½j dt²`、`F[2:4,4]=j dt`、`F[:2,5:7]=−½R dt²`、`F[2:4,5:7]=−R dt`、`F[4,7]=−dt`；`G[:2,:2]=½R dt²`、`G[2:4,:2]=R dt`、`G[4,2]=dt`；噪声 `G diag(IMU_STD²)Gᵀ`（8 维时再加 `diag(BIAS_RW_STD²)·dt`）；GPS 更新 `H=[I₂ 0₂ₓ₆]`、`R=GPS_STD²I₂`，用 `K = solve(S, HP)ᵀ` 避免显式求逆，协方差用 **Joseph 形式** `P⁺ = (I−KH)P(I−KH)ᵀ + KRKᵀ`；真值为解析 8 字轨迹 `p = [20 sin(0.1t), 10 sin(0.2t)]` | `DT=0.05`、`GPS_INTERVAL=20`、`SIM_TIME=50.0`、`IMU_STD=[0.1,0.1,deg2rad(0.3)]`、`GPS_STD=0.8`、`BIAS_RW_STD=[0.002,0.002,deg2rad(0.02)]`、`TRUE_BIAS=[0.04,-0.03,deg2rad(0.4)]`、GPS 中断区间 `(20.0, 30.0)`、`seed=0` | `gps_imu_fusion/gps_imu_fusion.py` |

**关键否定结论（避免误引）**：
1. `gps_imu_fusion` **没有**经纬度→ENU 转换，docstring 明确"位置/速度在局部米制世界坐标系、GPS 以米给出"（任务书中常提到的 ENU 转换在此**不存在**）。
2. `cubature_kalman_filter` 在 `docs/modules/2_localization/` 下**没有 rst**，其引用只是 IEEE 链接（`https://ieeexplore.ieee.org/document/4982682`），**作者/年份未确认**。
3. EnKF 的 `calc_covariance` 返回 3×3 而状态是 4×1（维度不一致，源码如此）。
4. UKF 更新里用 `y = z − observation_model(xPred)`，同时对再生 sigma 观测的均值 `zb` 求互协方差（与文档步骤表述不完全一致，照代码记录）。
5. 各滤波器的引用多为 *Probabilistic Robotics*（Thrun/Burgard/Fox）与 Roger Labbe 的 *Kalman and Bayesian Filters in Python*（见 `docs/modules/12_appendix/Kalmanfilter_basics_main.rst` 的参考文献），这些是**文档里写明的书名**，但**年份与出版社在本地文件中未完整给出 → 年份未确认**；`gps_imu_fusion` 引 Oliver J. Woodman, *An introduction to inertial navigation*, 2007（**作者+标题+年份齐全**）。

---
### 4.6 Mapping（建图与点云处理）

**关键否定结论：本仓库没有 log-odds（对数几率）占据栅格更新。** 全仓库检索 `log_?odds|logit` 无命中；占据栅格只有三种表示：三值 `{0.0 自由, 0.5 未知, 1.0 占据}`（射线投射、lidar→grid）、高斯 CDF 概率（高斯栅格图）。

| 模块 | 原理与公式 | 关键参数默认值 | 文件 |
|---|---|---|---|
| NDT 地图 | 用一组正态分布表示点云：每个栅格存 `μ_i`（2D 均值）与 `Σ_i`（2×2 协方差），PDF `p(x) = 1/(2π√|Σ|)·exp(−½(x−μ)ᵀΣ⁻¹(x−μ))`；两步：① 逐点算栅格索引聚类（`grid_index_map`）；② 对点数 `≥ min_n_points` 的簇求 `np.cov` 并 `np.linalg.eig` 分解，用 `plot_covariance_ellipse(chi2=3.0)` 画椭圆 | `min_n_points = 3`；`width/height = int((max−min)/resolution) + 3`（+3 为余量）；demo `grid_resolution = 10.0` | `ndt_map/ndt_map.py`（`NDTMap`+`NDTGrid`） |
| 射线投射栅格 | **按极角分桶 + 距离比较**（不是 Bresenham 逐格）：桶数 `round(2π/yawreso)+1`；对每个观测算 `d`、`angle`、`angleid`，把该桶内所有 `grid.d > d` 的格写 `0.5`（未命中），观测点自身写 `1.0` | `EXTEND_AREA=10.0`、`xyreso=0.25`、`yawreso=deg2rad(10.0)`；初值 `0.0`；`draw_heatmap(vmax=1.0)` | `ray_casting_grid_map/ray_casting_grid_map.py`（`precastDB`） |
| 激光雷达→栅格 | ① 读 CSV（`angle, distance`）→ `ox = sin(ang)·dist`、`oy = cos(ang)·dist`（**代码把 sin 给 x、cos 给 y，与常规约定相反**）；② **Bresenham 直线算法**（含 doctest：`bresenham((4,4),(6,10))`）标自由格 `0.0`；③ 命中格写 `1.0` 并膨胀 `(ix+1,iy)/(ix,iy+1)/(ix+1,iy+1)` 三格；另有 `breshen=False` 的洪水填充分支（`deque` 四邻域 BFS） | `EXTEND_AREA=1.0`、`xy_resolution=0.02`、`breshen=True`；初值 `np.ones((x_w,y_w))/2` | `lidar_to_grid_map/lidar_to_grid_map.py` |
| 高斯栅格图 | 对每格中心求到所有观测点的最小距离 `mindis`，再取**高斯互补 CDF**：`pdf = 1 − norm.cdf(mindis, 0.0, std)` | `EXTEND_AREA=10.0`、`xyreso=0.5`、`STD=5.0`；`vmax=1.0` | `gaussian_grid_map/gaussian_grid_map.py` |
| grid_map_lib（被多方复用的栅格库） | 一维数组存储，`grid_ind = y_ind·width + x_ind`；`xy_index = floor((pos − lower)/resolution)`（越界返回 `None`）；格中心 = `lower + index·resolution + resolution/2`；`FloatGrid` 用 `@total_ordering` 支持比较；多边形填充用射线穿越；`expand_grid` 复制到 **6 邻域**（缺 `(ix+1,iy-1)` 与 `(ix-1,iy+1)`，不是完整 8 邻域） | `GridMap(width, height, resolution, center_x, center_y, init_val=FloatGrid(0.0))`；`expand_grid(occupied_val=FloatGrid(1.0))`；demo `GridMap(100,120,0.5,10.0,-0.5)` | `grid_map_lib/grid_map_lib.py` |
| 距离图 UDF/SDF | **Felzenszwalb–Huttenlocher 抛物线下包络**线性距离变换：把 `0→INF, 1→0`，逐行做 1D 变换 `d[q] = min_p((q−p)² + d[p])`（维护抛物线下包络，`s = ((d[q]+q²) − (d[v[k]]+v[k]²))/(2q − 2v[k])`，`s ≤ z[k]` 则弹栈），转置后再做一次，最后开方；`SDF = UDF(obstacles) − UDF(1 − obstacles)`；另有 scipy 版 `distance_transform_edt`（docstring 实测：500×500 图上 `compute_sdf` 3 s vs scipy 0.05 s） | `INF=1e20`、`ENABLE_PLOT=True`；输入必须只含 0/1 否则 `ValueError` | `DistanceMap/distance_map.py` |
| K-means 聚类 | `cost = Σ_p min_i ‖p − c_i‖`（欧氏，非平方）；迭代"分配标签 → 重算质心"，`|Δcost| < DCOST_TH` 停；**随机标签初始化，非 k-means++** | `MAX_LOOP=10`、`DCOST_TH=0.1`；demo `n_cluster=2` | `kmeans_clustering/kmeans_clustering.py`（`Clusters`） |
| 矩形/L 形拟合 | ① 自适应距离分割：以半径 `r = R0 + Rd·‖p‖` 收集邻点成簇，再用 `itertools.permutations` 合并有交集的簇；② 在 `θ ∈ arange(0, π/2, dθ)` 上旋转点云，按三准则之一评分：`Area: −(c1max−c1min)(c2max−c2min)`、`Closeness: Σ1/d`（`d` 为点到矩形边的距离，带 `min_dist` 下限）、`Variance: −var(e1) − var(e2)`；③ 取最优角度求 4 条边并用交点拼矩形轮廓 | `criteria=VARIANCE`、`min_dist_of_closeness_criteria=0.01`、`d_theta_deg_for_search=1.0`、`R0=3.0`、`Rd=0.001`；模拟器 `LidarSimulator.range_noise=0.01` | `rectangle_fitting/rectangle_fitting.py` + `simulator.py` |
| 圆拟合 | Kåsa 型代数最小二乘：由 `Σx, Σy, Σx², Σy², Σxy` 组 3×3 线性系统 `F·T = G`，`c_x = T[0]/−2`、`c_y = T[1]/−2`、`r = √(c_x²+c_y²−T[2])`；误差 `Σ(hypot(c−p) − r)`（**带符号残差和，非平方和**）；采样加 `uniform(0.95,1.05)` 半径噪声并按角度分桶做射线滤波取最小距离 | `angle_reso=deg2rad(3.0)`、`cr=1.0`；demo `simtime=15.0, dt=1.0` | `circle_fitting/circle_fitting.py` |
| 法向量估计 | 三点叉积 `n = (v1×v2)/‖v1×v2‖`；平面距离 `|n·(p−origin)|/‖n‖`；**RANSAC**：迭代上限闭式 `max_iter = floor(ln(1−p)/ln(1−(1−inlier_radio_th)³))`，每轮随机取 3 点算法向并统计内点率，`> inlier_radio_th` 立即返回 | `ransac_normal_vector_estimation(points_3d, inlier_radio_th=0.7, inlier_dist=0.1, p=0.99)` | `normal_vector_estimation/normal_vector_estimation.py` |
| 点云采样 | 三种：① **体素** `key = tuple(xyz // voxel_size)` 桶内取均值；② **最远点**（维护 `min_distances` 并 `argsort(-min_distances)`）；③ **泊松盘**（随机抽点，到已选集合最小距离 `≥ min_distance` 才接受，最多 `MAX_ITER=1000` 次） | `voxel_size=20.0`、最远点 `n_points=20`、泊松盘 `n_points=20, min_distance=10.0`；demo `n_points=1000, seed=1234` | `point_cloud_sampling/point_cloud_sampling.py` |

**论文/出处**：距离图 → **Felzenszwalb & Huttenlocher, "Distance Transforms of Sampled Functions"**（年份未给出 → 未确认）；lidar→grid → **Moravec & Elfes, "High resolution maps from wide angle sonar", Proc. IEEE ICRA (1985)**（作者+标题+年份齐全）；矩形拟合 → **"Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners", CMU Robotics Institute**（作者/年份未确认）；NDT、射线投射、高斯栅格、k-means、圆拟合、grid_map_lib、点云采样、法向估计 → **无正式论文引用**（只有 Wikipedia/StackOverflow 链接或完全没有）。

### 4.7 SLAM

| 算法 | 原理与公式 | 关键参数默认值 | 文件 |
|---|---|---|---|
| EKF-SLAM | 联合状态 `X = [x,y,θ, m1x,m1y, …, mnx,mny]ᵀ`（`STATE_SIZE=3, LM_SIZE=2`），协方差 `P` 含位姿/地图/互协方差块。预测：`F=I`、`B=[[dt cosθ,0],[dt sinθ,0],[0,dt]]`，`jF = [[0,0,−dt·v sinθ],[0,0,dt·v cosθ],[0,0,0]]`，`G = I + Fxᵀ jF Fx`，`P ← GᵀPG + Fxᵀ Cx Fx`。观测 `ẑ = [√q, atan2(δy,δx) − θ]`，`y = z − ẑ`（角度项 `pi_2_pi`）；雅可比 `G_h = (1/q)[[−√q δx, −√q δy, 0, √q δx, √q δy],[δy, −δx, −q, −δy, δx]]`，`H = G_h F`；`S = HPHᵀ + Cx[0:2,0:2]`、`K = PHᵀS⁻¹`、`X ← X + Ky`、`P ← (I−KH)P`。**数据关联用马氏距离** `d_i = y_iᵀ S_i⁻¹ y_i`，并把阈值 `M_DIST_TH` 追加到列表尾取 `argmin`；若最小者是阈值本身则判为**新路标**并用逆观测模型 `m = [x + r cos(θ+φ), y + r sin(θ+φ)]` 增广（`initP = eye(2)`） | `Cx=diag([0.5,0.5,deg2rad(30)])²`、`Q_sim=diag([0.2,deg2rad(1)])²`、`R_sim=diag([1,deg2rad(10)])²`、`DT=0.1`、`SIM_TIME=50.0`、`MAX_RANGE=20.0`、`M_DIST_TH=2.0`、4 个路标 | `EKFSLAM/ekf_slam.py` |
| FastSLAM 1.0 | 每粒子 = 位姿 + 每路标一个独立 2×2 EKF。采样用运动模型加噪；观测雅可比 `Hv = [[−dx/d, −dy/d, 0],[dy/d², −dx/d², −1]]`、`Hf = [[dx/d, dy/d],[−dy/d², dx/d²]]`、`Sf = Hf Pf Hfᵀ + Q`；权重 `w ← w·exp(−½ dzᵀSf⁻¹dz)/(2π√det Sf)`；路标 EKF 用 **Cholesky 平方根更新**（`S ← (S+Sᵀ)/2`、`W1 = PHfᵀ(s_chol)⁻¹`、`P ← Pf − W1W1ᵀ`）；**低方差重采样**（`w_cum = cumsum(pw)`，`base = cumsum(0·pw + 1/N) − 1/N`，`resample_id = base + rand/N`），`n_eff = 1/(pw·pwᵀ)` | `Q=diag([3.0,deg2rad(10)])²`、`R=diag([1,deg2rad(20)])²`、`Q_SIM=diag([0.3,deg2rad(2)])²`、`R_SIM=diag([0.5,deg2rad(10)])²`、`N_PARTICLE=100`、`NTH=N_PARTICLE/1.5`、`DT=0.1`、`SIM_TIME=50.0`、`MAX_RANGE=20.0`、8 个路标；新路标判据 `abs(lm[id,0]) <= 0.01` | `FastSLAM1/fast_slam1.py` |
| FastSLAM 2.0 | 与 1.0 唯一实质差别：**提议分布吸收最新观测**。`proposal_sampling` 中 `particle.P ← (HvᵀSfi Hv + P⁻¹)⁻¹`，`x ← x + particle.P Hvᵀ Sfi dz`（即"已知地图下 EKF 后验"）；其余（运动预测、Cholesky 路标更新、低方差重采样、加权均值）与 1.0 相同 | 参数与 1.0 **完全相同**；`Particle` 多一个 `P = eye(3)` | `FastSLAM2/fast_slam2.py` |
| 图优化 SLAM（教学版） | 位姿图，边由"同一路标被两次观测"构造虚拟观测；残差 `e = [x2−x1−d1cos(θ1+φ1)+d2cos(θ2+φ2), y2−y1−d1sin(θ1+φ1)+d2sin(θ2+φ2), 0]`（**第 3 行被硬编码为 0**，与 rst 文档不一致）；信息矩阵 `Ω = (R_{t1}ΣR_{t1}ᵀ + R_{t2}ΣR_{t2}ᵀ)⁻¹`；目标 `χ² = Σ eᵀΩe`；解析雅可比 `A = [[−1,0,d1 sin t1],[0,−1,−d1 cos t1],[0,0,0]]`、`B = [[1,0,−d2 sin t2],[0,1,d2 cos t2],[0,0,0]]`；装配 `H[id1,id1] += AᵀΩA` 等；**纯 Gauss-Newton**（`dx = −H⁻¹b`，`H[0:3,0:3] += I` 固定原点），收敛 `dxᵀdx < 1e-5`，**没有 Levenberg–Marquardt/阻尼**（全仓 grep `levenberg\|damping` 无命中） | `C_SIGMA1=0.1, C_SIGMA2=0.1, C_SIGMA3=deg2rad(1.0)`、`DT=2.0`、`SIM_TIME=100.0`、`MAX_RANGE=30.0`、`MAX_ITR=20`、5 个路标 | `GraphBasedSLAM/graph_based_slam.py` |
| 图优化 SLAM（`graphslam/` 包） | 移植自 `python-graphslam`（Jeff Irion, 2020）的通用 SE(2) 求解器。`PoseSE2` 继承 `np.ndarray`：复合 `p1⊕p2 = [x1 + x2 cosθ1 − y2 sinθ1, y1 + x2 sinθ1 + y2 cosθ1, θ1+θ2]`，逆复合 `p1⊖p2 = [(x1−x2)cosθ2 + (y1−y2)sinθ2, (x2−x1)sinθ2 + (y1−y2)cosθ2, θ1−θ2]`；边误差 `e = z − (p2 ⊖ p1)`，`χ² = ΣeᵀΩe`，**雅可比用数值差分**（`EPSILON=1e-6`）；`Graph.optimize(tol=1e-4, max_iter=20, fix_first_pose=True)` 用 `scipy.sparse.linalg.spsolve` 解 `Δx = −H⁻¹b`，收敛判据是**相对 χ² 下降** `(χ²_prev − χ²)/(χ²_prev+eps) < tol`；`load_g2o_se2` 解析 `VERTEX_SE2`/`EDGE_SE2` 与上三角信息矩阵 | `EPSILON=1e-6`、`optimize(tol=1e-4, max_iter=20)`；数据集 `data/input_INTEL.g2o`（1228 顶点 / 1483 边，其中里程计边 1227、scan-matching 边 256，见 rst） | `GraphBasedSLAM/graphslam/{graph.py, pose/se2.py, edge/edge_odometry.py, vertex.py, util.py, load.py}` |
| ICP 匹配 | ① 最近邻关联（全对全暴力，无 KD 树）；② **SVD 闭式刚体配准**：`pm=mean(prev)`、`cm=mean(cur)`，`W = (cur−cm)(prev−pm)ᵀ`，`u,s,vh = svd(W)`，`R = (u·vh)ᵀ`，`t = pm − R·cm`；③ `dError = preError − error`，`< 0` 打印 "Not Converge" 退出，`≤ EPS` 收敛；④ 齐次矩阵累积 `H = H·[R|t]` | `EPS=0.0001`、`MAX_ITER=100`、`nPoint=1000`、`fieldLength=50.0`、`nsim=3`；有 2D 与 3D 两个入口 | `ICPMatching/icp_matching.py` |

**关键否定结论**：本 scope 的 ICP **没有 RANSAC 外点剔除**（RANSAC 只出现在 `Mapping/normal_vector_estimation` 里）；`graphslam/` 与 `ndt_map.py` **没有对应测试**。

**论文/出处**：EKF-SLAM、FastSLAM 1/2 → *Probabilistic Robotics*（Thrun/Burgard/Fox）+ Tim Bailey 的 SLAM 仿真页（**作者在链接中但年份未给出**）；图优化（教学版）→ **Grisetti, Kummerle, Stachniss, Burgard, "A tutorial on graph-based SLAM", IEEE ITS Magazine 2(4):31–43, 2010**（作者+标题+年份齐全）+ Blanco 的 SE(3) 教程（2010）；`graphslam` 包 → Jeff Irion, 2020（`python-graphslam`），数据来源 **Carlone & Censi, IEEE T-RO 30(2):475–492, 2014**；ICP → *Introduction to Mobile Robotics: Iterative Closest Point Algorithm*（GMU 课件，**作者/年份未确认**）。

---
### 4.8 ArmNavigation / AerialNavigation / Bipedal / InvertedPendulum / MissionPlanning

#### 4.8.1 ArmNavigation
| 模块 | 原理与公式 | 关键参数默认值 | 文件 |
|---|---|---|---|
| 二连杆臂点到点 | 平面 2R 正运动学 `elbow = shoulder + l1[cosθ1, sinθ1]`、`wrist = elbow + l2[cos(θ1+θ2), sin(θ1+θ2)]`；**闭式逆解** `θ2 = acos((x²+y²−l1²−l2²)/(2 l1 l2))`、`β = atan2(l2 sinθ2, l1 + l2 cosθ2)`、`θ1 = atan2(y,x) − β`（不可达时 `θ2_goal = 0`；`θ1_goal < 0` 时取肘部另一构型）；控制为 P 律 `θ += Kp·angle_mod(θ_goal − θ)·dt`；鼠标点击设目标 | `Kp=15`、`dt=0.01`、`l1=l2=1`、`GOAL_TH=0.0`（animation 用 0.01） | `two_joint_arm_to_point_control/` |
| N 连杆平面臂 | 正运动学 `x = Σ l_i cos(Σ_{k≤i}θ_k)`、`y = Σ l_i sin(...)`；雅可比 `J[0,i] = −Σ_{j≥i} l_j sin(Σ_{k≤j}θ_k)`、`J[1,i] = +Σ_{j≥i} l_j cos(...)`，`jacobian_inverse` **返回伪逆** `pinv(J)`；IK 迭代 `θ += pinv(J)·[x_goal−x, y_goal−y]`（成功判据 `distance < 0.1`）；跟踪用 P 律 | `Kp=2`、`dt=0.1`、`N_LINKS=10`、`N_ITERATIONS=10000`、随机目标域 `SAREA=15.0` | `n_joint_arm_to_point_control/`（`NLinkArm` 类） |
| 3D N 连杆（DH 参数） | 标准 DH 齐次变换 `T = [[cθ, −sθcα, sθsα, a cθ],[sθ, cθcα, −cθsα, a sθ],[0, sα, cα, d],[0,0,0,1]]`（参数序 `[theta, alpha, a, d]`），链式累乘求末端位姿并提取 **ZYZ 欧拉角**；基础雅可比列为 `[z_prev × (p_ee − p_prev); z_prev]`（6×1）；IK 用姿态速率矩阵 `K_zyz = [[0, −sinα, cosα sinβ],[0, cosα, sinα sinβ],[1, 0, cosβ]]`、`K_alpha = blkdiag(I3, K_zyz)`，更新 `θ̇ = pinv(J)·K_alpha·Δpose`，步长 `/100`，迭代 500 次 | IK 硬编码 `range(500)`；demo 用 **PR2 的 7 组 DH 参数**，随机关节角/目标位姿各 10 次 | `n_joint_arm_3d/NLinkArm3d.py`（`Link`、`NLinkArm`）+ `random_forward_kinematics.py`、`random_inverse_kinematics.py` |
| 机械臂关节空间避障 | 把关节角 `[-π, π]` 离散成 **100×100 环形（torus）栅格**，逐格用圆-线段最近距离判碰撞；启发式为曼哈顿距离再做"绕环松弛" `h[i,j] = min(h[i,j], M−i−1 + h[M−1,j], i + h[0,j], M−j−1 + h[i,M−1], j + h[i,0])`；A\* 每轮用 `np.argmin(explored_heuristic_map)` 选点（**只放 h、没加 g**，`distance_map` 记录了 g 但未参与排序） | `M=100`、`start=(10,50)`、`goal=(58,56)`；v1 障碍 `[[1.75,0.75,0.6],[0.55,1.5,0.5],[0,-1,0.25]]`、v2 用 `link_length=[0.5,1.5]`；复杂度最坏 `O(M⁴)≈1e8`（每轮对整个数组 `argmin`） | `arm_obstacle_navigation/`（两版） |
| 七自由度臂 RRT\* | 复用 3D DH 链，距离 `d = sqrt(ΣΔx²)`，`steer` 按 `path_resolution` 步进；邻域 `r = connect_circle_dist·sqrt(ln n/n)` 且 `min(r, expand_dis)`；目标偏置采样 `goal_sample_rate=20`；碰撞把臂上的离散点与球形障碍 `(x,y,z,size)` 比距离 | 默认 `expand_dis=0.30`、`path_resolution=0.1`、`goal_sample_rate=20`、`max_iter=300`、`connect_circle_dist=50.0`；demo Panda DH、`max_iter=200`、`end=[1.5]*7`；**`generate_final_course` 里 `reversed(path)` 未赋值（不生效），返回顺序是 goal→start** | `rrt_star_seven_joint_arm_control/` |

出处：二连杆 → **P. I. Corke, "Robotics, Vision & Control", Springer 2017, p102**（作者+书名+年份齐全）；3D 臂、七自由度 RRT\*、N 连杆、臂避障 → **文件内均无论文引用 → 未确认**（臂避障的碰撞判定注释引了一个 `doswa.com` 的圆-线段相交网页）。

#### 4.8.2 AerialNavigation
- **四旋翼 3D 轨迹跟踪（`drone_3d_trajectory_following/`）**：轨迹用**五次多项式**，系数由 6×6 线性系统解出
  `A = [[0,0,0,0,0,1],[T⁵,T⁴,T³,T²,T,1],[0,0,0,0,1,0],[5T⁴,4T³,3T²,2T,1,0],[0,0,0,2,0,0],[20T³,12T²,6T,2,0,0]]`，`b = [start_pos, des_pos, start_vel, des_vel, start_acc, des_acc]`；
  高度通道 PD：`thrust = m(g + des_z_acc + Kp_z(des_z_pos − z) + Kd_z(des_z_vel − ż))`；
  姿态通道 P（微分平坦式前馈 + 反馈）：`roll_torque = Kp_roll·(((des_x_acc sin ψ − des_y_acc cos ψ)/g) − roll)`、`pitch_torque = Kp_pitch·(((des_x_acc cos ψ − des_y_acc sin ψ)/g) − pitch)`、`yaw_torque = Kp_yaw(des_yaw − yaw)`；角速度 `+= torque·dt/I`；再 `acc = (R(roll,pitch,yaw)·[0,0,thrust] − [0,0,mg])/m` 积分。
  参数：`g=9.81, m=0.2, Ixx=Iyy=Izz=1, T=5, Kp_roll=Kp_pitch=Kp_yaw=25, Kp_z=1, Kd_z=1, dt=0.1, n_run=8`；航点 `[[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]]`。文件内**无论文引用**；注意 `rotation_matrix` 第 3 行末项写的是 `cos(pitch)·cos(yaw)`（常规应为 `cos(roll)`），且 `Kp_x/Kp_y/Kd_x/Kd_y` 定义了但未被使用（对应的位置前馈被注释掉）。
- **火箭动力着陆（`rocket_powered_landing/`）**：6-DoF 状态 `x = [m, r_I(3), v_I(3), q_B_I(4), w_B(3)]`（`n_x=14`），输入为推力矢量 `u`（`n_u=3`）；动力学含 `ṁ = −0.01‖u‖`、`v̇ = C_I_B(q)u/m + g_I`、四元数 `q̇ = ½Ω(w)q`、`ẇ = J_B⁻¹(skew(r_T_B)u − skew(w)J_B w)`；用**逐次凸化（SCvx）+ 自由末端时间**：每次迭代由 ODE 积分的状态转移矩阵得到线性时变离散模型 `X_{k+1} = Ā X_k + B̄ U_k + C̄ U_{k+1} + S̄σ + z̄ + ν_k`，求解一个含二阶锥约束（滑翔斜率、最大倾角、云台角、推力上界/下界）与信赖域约束的 SOCP，目标 `min W_σ·σ + W_ν‖ν‖_∞ + W_δ·delta_norm + W_δσ·sigma_norm`；外层最多 `iterations=30`，收敛判据 `delta_norm<1e-3 且 sigma_norm<1e-3 且 nu_norm<1e-7`，且每轮 `w_delta *= 1.5`。参数：`K=50`（离散点）、`m_wet=3.0, m_dry=2.2, t_f_guess=10.0`、`T_max=5.0, T_min=0.3`、`max_gimbal=20°, max_angle=90°, glidelslope_angle=20°`、`J_B=1e-2·I₃`、`r_T_B=(-1e-2,0,0)`、求解器 **CLARABEL**（docstring 注释为 CVXOPT）。出处：**"Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time" by Michael Szmuk and Behcet Acıkmese**，并注明移植自 `EmbersArc/SuccessiveConvexificationFreeFinalTime`（**年份未给出 → 未确认**）。

#### 4.8.3 Bipedal
`bipedal_planner.py`：**LIPM**（线性倒立摆）`ẍ = (g/z_c)(x − p_x*)`、`ÿ = (g/z_c)(y − p_y*)`，欧拉积分（`time_split=100` 步）；时间常数 `Tc = √(z_c/g)`，`C = cosh(t_sup/Tc)`、`S = sinh(t_sup/Tc)`；参考质心/速度
`x_ = R(θ_next)·[f_x_next/2, (−1)ⁿ f_y_next/2]`，`vx_ = R(θ_next)·[(1+C)/(Tc S)·x_, (C−1)/(Tc S)·y_]`，`xd = p_x + x_`；
**落脚点修正的解析最小二乘解**：`D = a(C−1)² + b(S/Tc)²`，
`p_x* = −a(C−1)/D·(xd − C x_i − Tc S ẋ_i) − bS/(Tc D)·(ẋd − (S/Tc)x_i − C ẋ_i)`（`a` 为位置误差权重、`b` 为速度误差权重），`p_y*` 同式。
参数：`g=9.8`、`walk(t_sup=0.8, z_c=0.8, a=10, b=1, plot=False)`；输出 `self.ref_p`、`self.act_p`、`self.com_trajectory`。**文件内无论文引用 → 未确认**。

#### 4.8.4 InvertedPendulum（LQR 与 MPC）
状态 `x = [x, ẋ, θ, θ̇]`（`nx=4`），输入 `u`（`nu=1`）；线性化并**前向欧拉离散**：
`A = [[0,1,0,0],[0,0,mg/M,0],[0,0,0,1],[0,0,g(M+m)/(l̄M),0]]`、`B = [0, 1/M, 0, 1/(l̄M)]ᵀ`，`A ← I + A·δt`、`B ← B·δt`。
LQR：DARE 迭代求 `P`，`K = (BᵀPB + R)⁻¹BᵀPA`，`u = −Kx`。MPC：`min Σ (xᵀQx + uᵀRu) s.t. x_{t+1} = Ax_t + Bu_t, x_0`，cvxpy + **CLARABEL**，只施加 `u[0]`（滚动时域）。
参数（两版公共）：`l_bar=2.0, M=1.0, m=0.3, g=9.8, Q=diag([0,1,1,0]), R=diag([0.01]), delta_t=0.1, sim_time=5.0, x0=[0,0,0.3,0]`；LQR 的 `solve_DARE(maxiter=150, eps=0.01)`（**每个仿真步都重解 DARE**）；MPC 的 `T=30`。
出处：LQR 代码引 `# ref Bertsekas, p.151`；文档 `docs/modules/10_inverted_pendulum/inverted_pendulum_main.rst` 给出完整 Lagrange 推导与 DARE 公式（**MPC 版无论文引用 → 未确认**）。

#### 4.8.5 MissionPlanning
- **行为树（`BehaviorTree/behavior_tree.py`）**：**用 `xml.etree.ElementTree` 解析 XML**（`build_tree(xml_string)` / `build_tree_from_file`），三态 `Status = {SUCCESS, FAILURE, RUNNING}`，三类节点 `{ControlNode, ActionNode, DecoratorNode}`。组合语义（照代码）：`SequenceNode` 子返回 SUCCESS 时索引 +1 并返回 **RUNNING**（一次 tick 只推进一个子节点），FAILURE 则复位并 FAILURE；`SelectorNode` 失败则索引 +1 并 RUNNING，成功则复位并 SUCCESS；`WhileDoElseNode` 要求恰好 2 或 3 个子节点，条件 SUCCESS → tick do 节点并 RUNNING，条件 FAILURE → 有 else 则透传、无 else 则复位并 SUCCESS，条件 RUNNING 直接 `ValueError("Unknown status")`。装饰节点：`Inverter`（**把 RUNNING 也映射成 FAILURE**，与其 docstring 不一致）、`Timeout`、`Delay`、`ForceSuccess`、`ForceFailure`；动作节点 `Sleep`、`Echo`。`BehaviorTreeFactory` 注册内置 tag 并做结构校验（Control 必须有子节点、Decorator 恰好 1 个、Action 无子节点、未知 tag → `ValueError`）。**已知瑕疵**：`"Timeout"` 分支误用 `SelectorNode.__name__` 作为默认名。出处仅 Wikipedia 行为树条目（另有 IEEE 链接，**未给作者/标题 → 未确认**）。
- **有限状态机（`StateMachine/state_machine.py`）**：`State(name, on_enter, on_exit)`；转移表键 `(src_state_name, event)` → `(dst, guard, action)`；`add_transition` 支持字符串（自动注册状态/事件，`guard/action` 用 `getattr(self._model, func, None)`，**找不到时静默为 None**）；`process(event)` 未注册事件抛 `ValueError`；`state_transition` 中 guard 为假只打印 "skipping transition" 不转移，自转移跳过 exit/enter。`generate_plantuml()` 生成 PlantUML 文本并**通过 `urllib.request` 访问 `http://www.plantuml.com/plantuml/img/{deflate_and_encode(...)}` 拉取 PNG**（异常被 `except Exception` 吞掉）——这是本仓库里少见的**联网行为**。案例状态机 `patrolling → executing_task → returning_to_base → charging → patrolling`；注意 `charge_complete` 的 `action="battery_full"` 在 `Robot` 中无对应方法（按 `getattr(..., None)` 会被静默忽略）。

---
## 5. 横切关注点

### 5.1 共享数据结构与工具函数

**`utils/` 包（很小，只有两个模块）**
- `utils/angle.py`（83 行）：只有两个公开函数。
  - `rot_mat_2d(angle)` → 2×2 旋转矩阵（`Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]`，基于 `scipy.spatial.transform`）；被 Dubins 规划、`utils/plot.py`、栅格 CPP 等复用。
  - `angle_mod(x, zero_2_2pi=False, degree=False)` → 默认把角度归一化到 `[-pi, pi)`（`(x + pi) % (2pi) - pi`），`zero_2_2pi=True` 时归到 `[0, 2pi)`；输入 `float` 返回 `float`，否则返回 `ndarray`。
  - **注意**：`utils/angle.py` 里**没有** `pi_2_pi`；`pi_2_pi(angle)` 在各算法文件里被重复定义（至少出现在 `SLAM/EKFSLAM`、`FastSLAM1/2`、`GraphBasedSLAM`、`EnKF`、各 PathTracking 文件里），而 `PathTracking/lqr_steer_control` 干脆定义一个 `pi_2_pi` 直接转发 `angle_mod`。
- `utils/plot.py`（234 行）：Matplotlib 绘图工具，**没有** `plot_anime` / `get_transform` / `draw_car` / `draw_arrow` / `draw_rectangle`（全仓 grep 无定义），也没有 `plot_vehicle`（`plot_vehicle` 只是 3 个 PathTracking 文件里的局部函数）。公开 API：
  `plot_covariance_ellipse(x, y, cov, chi2=3.0, color="-r", ax=None)`（把 2×2 协方差画成置信椭圆，特征值定长短轴、特征向量定方向）、`plot_ellipse(x, y, a, b, angle, color="-r", ax=None, **kwargs)`、`plot_arrow(x, y, yaw, arrow_length=1.0, origin_point_plot_style="xr", head_width=0.1, fc="r", ec="k", **kwargs)`（对标量/数组自动分发）、`plot_curvature(x_list, y_list, heading_list, curvature, k=0.01, c="-c", label="Curvature")`（把曲率画成路径法向的偏移线）、`class Arrow3D(FancyArrowPatch)`（3D 箭头，含 `draw`/`do_3d_projection`）、`_arrow3D(...)`、`plot_3d_vector_arrow(ax, p1, p2)`（**运行时给 `Axes3D` 猴子补丁 `arrow3D`**）、`plot_triangle(p1, p2, p3, ax)`、`set_equal_3d_axis(ax, x_lims, y_lims, z_lims)`。
  文档只 autodoc 了 `plot_curvature` 一个函数（`docs/modules/11_utils/plot/plot_main.rst`）。

**被跨模块复用的"事实标准"组件**
| 组件 | 作用 | 复用情况 |
|---|---|---|
| `Mapping/grid_map_lib/grid_map_lib.py` 的 `GridMap` / `FloatGrid` | 栅格地图：世界坐标↔索引、多边形填充、膨胀、热图 | 被 `PathPlanning/GridBasedSweepCPP`、`Mapping/ndt_map` 等复用 |
| `PathPlanning/CubicSpline/cubic_spline_planner.py` 的 `CubicSpline2D` / `calc_spline_course` | 以弧长为参数的三次样条，输出位置/航向/曲率/曲率变化率 | 被 PathTracking（LQR、Stanley、MPC、CGMRes）、Frenet、Reeds-Shepp、Hybrid A\* 等大量引用，是全仓库最重要的公共件 |
| `PathPlanning/VoronoiRoadMap/dijkstra_search.py` 的 `DijkstraSearch` | 路图上的 Dijkstra（节点等价阈值 0.1 m） | 被 `VoronoiRoadMap` 与 `VisibilityRoadMap` **两个模块共用** |
| `PathPlanning/TimeBasedPathPlanning/{Node,BaseClasses,GridWithDynamicObstacles}` | `(x,y,t)` 节点、单机/多机规划器抽象基类、3D 预约栅格与安全区间 | 只在 TimeBasedPathPlanning 内部使用，但设计上最容易抽成独立"动态环境内核" |
| 各文件内重复的局部函数 | `plot_arrow`、`pi_2_pi`、`angle_mod` 的副本、`get_motion_model` 的 8 邻域表 | 说明**尚未**完成"工具收敛到 utils"的重构；搬用时要注意各副本默认值不同（例如 `plot_arrow` 的 `length/width` 默认值在 Bezier/RS/DWA 里各不相同） |

### 5.2 仿真循环与动画是如何组织的

- **没有统一的仿真器/引擎类**。每个算法是一个"脚本"：模块级 `show_animation = True` 开关 + `main()` 函数 + `if __name__ == '__main__': main()`（全仓 168 个 .py 有 main guard）。大量文件在顶部用 `sys.path.append(...)` 手工注入路径来互相 import（例如 `PathPlanning/RRT/sobol/__init__.py` 只导出别名，`ArmNavigation/n_joint_arm_3d/__init__.py` 只做 `sys.path.append`）。
- **动画方式以"增量式 matplotlib + `plt.pause`"为主**：82 个文件出现 `plt.pause`，典型循环是
  `while True: 规划/控制一步 → plt.cla()/set_data() 重画 → plt.pause(0.001~0.2) → 判定是否到达`，结束时 `plt.show()`。绘图代码通常带 `# pragma: no cover` 以免计入覆盖率。
- **只有 3 个文件用 `matplotlib.animation.FuncAnimation`**：`Localization/gps_imu_fusion/gps_imu_fusion.py`、`PathPlanning/ParticleSwarmOptimization/particle_swarm_optimization.py`、`PathPlanning/ClothoidPath/clothoid_path_planner.py`（另有 `PathPlanning/ElasticBands` 用逐帧 `plot_background()`）。
- **交互与退出**：多个 demo 用 `plt.gcf().canvas.mpl_connect("key_release_event", lambda e: exit(0) if e.key == "escape" else None)` 支持 Esc 退出；`ArmNavigation/arm_obstacle_navigation_2.py` 用 `key_press_event` 按 q 退出。`ArmNavigation` 的两个 demo 支持鼠标左键设目标。
- **测试靠关闭动画**：测试统一设 `m.show_animation = False`（少数用 `m.do_plot`、`m.ENABLE_PLOT`、`m.SHOW_ANIMATION`）再调 `main()`；`PathPlanning/ParticleSwarmOptimization` 在非动画模式下 `plt.switch_backend("Agg")`。
- **没有把仿真结果导出为数据文件/日志的机制**（除 `SpiralSpanningTreeCPP`/`WavefrontCPP` 读 PNG 地图、`ElasticBands` 读 `path.npy`/`obstacles.npy`、`GraphBasedSLAM` 读 `.g2o`、`lidar_to_grid_map` 读 CSV、`StateLatticePlanner` 读 `lookup_table.csv` 之外）。

### 5.3 参数与坐标系约定

- **世界坐标系**：2D 平面，`x` 向右、`y` 向上（右手系），**朝向 `yaw/theta` 从 `+x` 轴逆时针为正**；`utils.angle.angle_mod` 统一归一到 `[-pi, pi)`。角度单位在参数表里通常是 rad（个别入口用 `np.deg2rad` 包装，`Localization/*` 的噪声用 `deg2rad` 与 `**2` 组合）。
- **速度模型（按文件区分，务必逐文件确认）**：
  - 自行车模型（后轴参考点，输入 `(a, δ)`）：`x += v cosθ dt`、`y += v sinθ dt`、`θ += v/L tanδ dt`、`v += a dt`。`L`（或 `WB`）在各文件里差别很大：纯追踪 `WB=2.9`、Stanley `L=2.9`、LQR 转向 `L=0.5`、LQR 速度+转向 `L=0.5`、MPC `WB=2.5`、Hybrid A\* `WB=3.0`、C/GMRES `WB=0.25`、MPTG `L=1.0`、Pure-pursuit(ClosedLoopRRTStar) `L=0.9`。
  - 差速/全向（DWA）：状态 `x = [x, y, yaw, v, ω]`，输入 `u = [v, ω]`，**`motion()` 先更新 yaw 再用新 yaw 更新位置**。
  - 误差状态模型：Tracking 里 `x=[x,y,yaw,v]`；LQR 转向的误差向量 `[e, ė, θe, θ̇e]`；LQR 速度+转向是 5 维；`LQRPlanner` 用 2 维"相对目标误差"。
  - 估计器状态：EKF `[x,y,yaw,v]`；EKF 校正版 `[x,y,yaw,v,s]`；CKF 用 **CTRV** `[x,y,yaw,v,ω]`；GPS/IMU 用 8 维 `[p,v,ψ,bias_a,bias_ω]`。
- **障碍表示（三种不统一，搬用时必须转换）**：① 点列表 `ox, oy`（栅格类、势场、DWA 的默认 `ob`）；② 圆元组 `(x, y, size)`（RRT 家族的 `obstacle_list`）；③ 多边形/线段（可视图的 `ObstaclePolygon`）；④ 栅格占据图（`Mapping/*`、CPP）；⑤ **时间维预约张量 `reservation_matrix[x, y, t]`**（仅 `TimeBasedPathPlanning`）。
- **时间步 `dt` 不统一**：`0.01`（move_to_pose、二连杆臂）、`0.05`（C-LQR 的 unicycle、GPS/IMU）、`0.1`（A\* 等网格类、DWA、PathTracking、EKF、PF）、`0.2`（Frenet、rectangle_fitting）、`1.0`（k-means demo）、`2.0`（GraphBasedSLAM）；`TimeBasedPathPlanning` 则用**整数时间步**（每步 = 1 个 tick），这是本仓库里唯一"按 tick 而非秒"建模动态环境的模块。
- **确定性**：只有 `TimeBasedPathPlanning/BaseClasses.py` 显式 `random.seed(50)` + `numpy.random.seed(50)`；PRM 的入口支持注入 `rng`；测试里偶有 `random.seed(12345)`/`np.random.seed(1234)`；其余地方用全局随机数。**整个仓库没有"固定时间步保证可复现"的统一机制**。
- **其它数值约定**：`PathPlanning/*` 的栅格类用**布尔** `obstacle_map[x][y]`（`False` 自由 / `True` 障碍）而不是 0/1；D\* 用字符串状态 `"." / "#" / "e" / "*" / "s"`；NDT/lidar→grid 用 `{0.0, 0.5, 1.0}`；`GridMap` 内部是**一维 list + `grid_ind = y_ind*width + x_ind`**。

---

## 6. 测试与验证

**规模**：`tests/` 下 **91 个 `test_*.py`**，共 **150 个 `def test*` 函数**；`tests/__init__.py` 为空；`tests/conftest.py`（13 行）只做两件事——把 `tests/` 与项目根加进 `sys.path`，并提供 `run_this_test(file)`；**没有任何 fixture、没有 conftest 级别的 mock**。

**运行方式**
- `runtests.sh`：`pytest tests -l -Werror --durations=0`（**警告即错误**、失败打印局部变量、按耗时排序）。
- `.github/workflows/{Linux,MacOS,Windows}_CI.yml`：push master / PR 时用 Python **3.14** 装 `requirements/requirements.txt` 后跑 `bash runtests.sh`。
- `appveyor.yml`：VS2022 + Python 3.13，`pytest tests -n auto -Werror --durations=0`（用 pytest-xdist 并行）。
- `tests/test_codestyle.py`：**用 ruff 检查"相对 origin/master 分支点以来改动的 .py 文件"**（`--config=ruff.toml`，且带 `--fix`），有 lint 输出即失败；`tests/test_mypy_type_check.py`：对 `["AerialNavigation","ArmNavigation","Bipedal","Localization","Mapping","PathPlanning","PathTracking","SLAM","InvertedPendulum"]` 逐个跑 `mypy --config-file mypy.ini -p <包名>`，非 0 即失败。这两个是"以测试形式存在的 CI 门禁"。
- `.circleci/config.yml` 只构建 Sphinx 文档并归档；`.github/workflows/codeql.yml` 做代码扫描；`gh-pages.yml` 部署文档。

**断言强度分布（重要，别误以为"有测试 = 有正确性保证"）**
- **56 个测试文件只做冒烟测试**：`import conftest` → `m.show_animation = False`（或 `m.do_plot/m.ENABLE_PLOT/m.SHOW_ANIMATION = False`）→ `m.main()`，**零断言**，靠"不抛异常"通过。必要时用模块级 monkeypatch 缩短时间：`FastSLAM1/2` 设 `m.SIM_TIME = 3.0`、`GraphBasedSLAM` 设 `m.SIM_TIME = 20.0`、`histogram_filter` 设 `m.SIM_TIME = 1.0`。
- **24 个测试文件含 assert**。其中真正做数值验证的代表：
  - `tests/test_gps_imu_fusion.py`（最扎实）：解析校验 `motion_model`、用**有限差分校验 `motion_jacobians`**（5 维与 8 维两种）、静止时预测噪声量级、GPS 更新与线性 KF 闭式解对照、航向 wrap、按 seed 参数化比较 RMSE（融合 < 纯惯导/5，且"无零偏版本"与纯 IMU 一致性检查）、GPS 计划/中断/恢复的协方差增长与收缩、`3σ` 椭圆的马氏距离等于 9.0、零方差椭圆、`animation.save`。
  - `tests/test_move_to_pose.py`：稳定性窗口检查（`window_size=10, count_threshold=4`）、对 `theta_start_list × x_goal_list × y_goal_list` 的笛卡尔积断言到达目标 `rho < 0.001`、最大速度冒烟。
  - `tests/test_utils.py`：`angle_mod` 的四种输入形态与返回值类型（`float` vs `ndarray`）精确对齐 docstring 例子。
  - `tests/test_grid_map_lib.py`：索引往返 `calc_grid_index_from_xy_index ↔ calc_xy_index_from_grid_index` 全覆盖断言。
  - `tests/test_distance_map.py`：SDF 符号（障碍内 `<0`、外 `>0`）、UDF 在障碍处为 0 且四邻为 1、非法输入抛 `ValueError`。
  - 覆盖规划三个测试有**覆盖率断言**：`test_spiral_...` 断言 `len(covered_nodes) == num_free/4`；`test_wavefront_...` 断言 `len(DT_path) == num_free` 与 `len(PT_path) == num_free`；`test_grid_based_sweep_...` 只断言 `len(px) >= 5`。
  - `tests/test_point_cloud_sampling.py`：用 `capsys` 断言 stdout 里三种采样的点数（27 / 20 / 20）。
  - `tests/test_behavior_tree.py`（7 个用例，断言最密集）：逐 tick 断言 `root.status` 与各子节点 `status`（含 `reset_children` 后子状态回到 `None`），并用 `pytest.raises(ValueError)` 覆盖 6 种非法树结构。
  - `tests/test_dubins_path_planning.py` / `test_reeds_shepp_path_planning.py`：断言端点/航向误差 `<= 0.01`、路径长度与分段长度一致性；RS 版还断言随机 10 组位姿、`step_size` 过大时返回 `None`。
  - `tests/test_rrt_star_reeds_shepp.py`：断言相邻路径点距离 `< step_size + 1e-14`，以及"过大 step_size → `path is None`"。
- **8 个模块没有对应测试文件**（按目录名在 `tests/` 里无命中）：`ArmNavigation/arm_obstacle_navigation`、`ArmNavigation/n_joint_arm_3d`、`Localization/ensemble_kalman_filter`、`Mapping/lidar_to_grid_map`、`Mapping/ndt_map`、`PathPlanning/BidirectionalAStar`、`PathPlanning/BidirectionalBreadthFirstSearch`、`PathPlanning/CubicSpline`、`PathPlanning/Eta3SplineTrajectory`、`PathPlanning/ModelPredictiveTrajectoryGenerator`（其中 `CubicSpline` 被大量测试间接覆盖）、`SLAM/GraphBasedSLAM/graphslam` 包。
- **已知测试缺陷**：① `tests/test_voronoi_road_map_planner.py` 实际 import `VisibilityRoadMap.visibility_road_map`，`tests/test_visibility_road_map_planner.py` 实际 import `VoronoiRoadMap.voronoi_road_map`——**文件名与 import 交叉错位**；② `tests/test_frenet_optimal_trajectory.py` 的 5 个函数名不以 `test` 开头（`default_scenario_test` 等），pytest 默认**不收集**，等价于 0 个用例；③ `tests/test_move_to_pose_robot.py` import 的是 `move_to_pose as m`（**没测** `move_to_pose_robot`）；④ `tests/test_space_time_astar.py` 注释写 `# path should have 28 entries` 而断言是 `== 31`。

---

## 7. 局限与坑

1. **定位是"教学合集"，不是工程库**：没有 `pyproject.toml`/`setup.py`，不能 `pip install`；各 demo 靠 `sys.path.append` 拼路径互相 import（`n_joint_arm_3d`、`rrt_star_seven_joint_arm_control`、`drone_3d_trajectory_following`、`PathPlanning/RRT/sobol` 等）；根 `__init__.py` 只是为了让测试能按包导入。**没有 API 稳定性承诺、没有版本号、没有 changelog**。
2. **动态障碍几乎只覆盖一个模块**：全仓库只有 `PathPlanning/TimeBasedPathPlanning/` 真正建模"随时间移动的障碍"（3D 预约矩阵 + 三种障碍运动生成器），且其假设相当强：障碍运动**完全已知**、栅格整数坐标、每步 1 格、无动力学/不确定性、`time_limit` 有限。其它模块的"动态"能力是**反应式**的：DWA 每步对**静态点表**重新评价（把点表换成预测位置即可动态化，但仓库没做）；C/GMRES 与 DWA 里出现 `predict_trajectory` 类函数，但**没有任何模块做动态障碍的轨迹预测**（grep `trajectory prediction` 无命中）。
3. **没有 CBS/ECBS**：全仓库 grep `cbs|conflict` 无命中。多机只有 `PriorityBasedPlanner`（顺序规划 + 预约，文档明确写着次优、不重规划）。
4. **实时性完全没有保证**：多处"每个仿真步重解一次重优化"（`LQRPlanner`/`inverted_pendulum_lqr_control` 每步解 DARE；MPC 每步解 cvxpy；C/GMRES 每次在线迭代；RRT\* 每轮全树近邻 + rewire），且都是纯 Python 循环 + Matplotlib 绘图。**绝大多数算法在代码和文档里都没有声明时间复杂度**（只有极少数在 rst 里给了实测扩展数/耗时）。
5. **大量循环没有迭代上限**：`Clothoid` 的 `fsolve`（容差用 scipy 默认）、`Eta3SplineTrajectory` 的牛顿迭代、`SIPP`/`SpaceTimeAStar`（只受 `time_limit` 约束）、图优化 SLAM 教学版（有 `MAX_ITR`）、ICP（有 `MAX_ITER`）、`graphslam`（有 `max_iter`）。搬用时必须逐个补上"最大迭代/最大时间"。
6. **动力学约束多数缺失**：栅格搜索（A\*/D\*/D\* Lite/SIPP/时空 A\*）都是 4/8 邻域**完整约束**运动；RRT 家族的 `steer` 是直线（只有 Dubins/Reeds-Shepp 变体才有最小转弯半径）；加速度/jerk 约束只在五次多项式、Frenet、η³ 轨迹里出现。**没有任何模块把车辆动力学嵌进全局图搜索**（Hybrid A\* 是最接近的，但它用的是运动基元 + Reeds-Shepp 解析扩展）。
7. **障碍/机器人几何高度简化**：多数模块把障碍当点或圆，机器人当圆；只有 DWA（`RobotType.rectangle` 的 OBB 判定）、Hybrid A\*（`rectangle_check` + KD 树 bubble）、`Mapping/rectangle_fitting` 真正处理矩形轮廓；机械臂避障用关节空间栅格 + 圆-线段近似。
8. **实现瑕疵（静态阅读发现，未运行验证）**：
   - `PathPlanning/DStarLite/d_star_lite.py`：`(g[u.x,u.y] > rhs[u.x,u.y]).any()` 对 0 维数组恒为 `False`，与 D\* Lite 伪代码不一致（是否影响最优性未确认）。
   - `Mapping/ndt_map/ndt_map.py`：`_create_grid_index_map` 在 `self.grid_map` 赋值之前被调用 → 构造 `NDTMap` 会抛 `AttributeError`。
   - `Mapping/grid_map_lib`：`set_value_from_xy_pos` 用 `if (not x_ind) or (not y_ind): return False`，**索引 0 会被误判为失败**；`set_value_from_xy_index` 有一处 `return False, False`（返回类型不一致）；`set_value_from_polygon` 的闭环 `np.append(pol_x, pol_x[0])` 返回值被丢弃；`expand_grid` 只扩 6 邻域（缺两个对角）。
   - `SLAM/GraphBasedSLAM/graph_based_slam.py`：角度残差被硬编码 `edge.e[2,0] = 0`、雅可比第三行全 0，与 `graphSLAM_doc.rst` 的公式不一致；噪声写法 `randn()*Q[0,0]`（**没有 `**0.5`**）。
   - `SLAM/FastSLAM1/2`：`M_DIST_TH` 定义后未使用（数据关联直接用真实 `landmark_id`）；`resampling` 用列表浅拷贝且**没有拷 `fast_slam2` 的提议协方差 `P`**；`fast_slam2.update_landmark` 用全局 `Q` 而非形参 `Q_cov`。
   - `Mapping/circle_fitting.py`：主循环里 `cy += math.cos(theta)`（疑为 `sin` 之误）。
   - `PathPlanning/TimeBasedPathPlanning/SafeInterval.py`：`add_entry_to_visited_intervals_array` 的 `append` 在 for 循环外**无条件执行**（注释写 "Otherwise" 但没有 else），同一区间会累积多条记录。
   - `PathPlanning/AStar/a_star_searching_from_two_side.py`：启发式用曼哈顿、步长用欧氏，对角情形 h **高估**（非可采纳）。
   - `PathPlanning/AStar/a_star_variants.py`：Theta\* 与跳点变体的 `get_farthest_point`/角点逻辑是"近似 Theta\*/跳点"，与标准算法不等价。
   - `PathPlanning/BatchInformedRRTStar`：`best_edge_queue_value()` 取的是最大值（与 `best_in_edge_queue()` 取最小值不一致），疑似实现瑕疵。
   - `AerialNavigation/drone_3d_trajectory_following`：`rotation_matrix` 第 3 行末项写作 `cos(pitch)cos(yaw)`（常规应为 `cos(roll)`）；`Kp_x/Kp_y/Kd_x/Kd_y` 定义未用。
   - `PathPlanning/QuarticPolynomial`?（不存在）／`QuinticPolynomialsPlanner`：所有候选 T 都不满足约束时**静默返回最后一轮（T=95）的轨迹**。
   - `PathTracking/stanley_control`：**没有**经典 Stanley 的 softening 项（分母直接是 `v`，无 `k_soft`）；`pure_pursuit` 声称 PID 实际只有比例；纯追踪跟踪的是**车辆中心反推 WB/2 的点**而不是后轴。
   - `Localization/cubature_kalman_filter` 的 `calc_covariance` 返回 3×3 而状态是 4/5 维；`ensemble_kalman_filter` 同样有维度不一致。
   - `Mapping/ndt_map`：**没有配准/score 函数**（只建图不匹配）；`SLAM/ICPMatching`：**没有 RANSAC 外点剔除**；`gps_imu_fusion`：**没有经纬度→ENU 转换**；`GraphBasedSLAM`：**没有 Levenberg–Marquardt 阻尼**；`Mapping` 全组：**没有 log-odds 栅格更新**。
9. **文档与代码不同步**（选读时注意）：`bspline_path` 的曲率指数是 `2/3`、Bézier/CubicSpline 是 `3/2`，而 `cubic_spline_main.rst` 又写 `2/3`；`graphSLAM_doc.rst` 引用旧函数名 `calc_rotational_matrix`（现名 `calc_3d_rotational_matrix`）；`state_lattice_planner` docstring 写 `plookuptable.csv` 而常量指向 `lookup_table.csv`。**没有 rst 的模块**：FlowField、GreedyBestFirstSearch、BidirectionalBreadthFirstSearch、Potential Field（只有一节）、`n_joint_arm_3d`、`rrt_star_seven_joint_arm_control`、`arm_obstacle_navigation_2`、Eta3SplineTrajectory、Cubature KF、Ensemble KF、NDT map、`graphslam` 包、`GridBasedSweepCPP` 的引用条目。`docs/modules/0_getting_started/2_how_to_run_sample_codes_main.rst` 说 Python 3.12，README 说 3.13，CI 跑 3.14（**口径不一致**）。
10. **代码风格/版本口径**：README 提到 `pycodestyle`，实际用 `ruff`（`ruff.toml`）；`mypy.ini` 只设 `ignore_missing_imports = True`（未开严格模式），所以 `test_mypy_type_check` 只保证"能过"。

---

## 8. 对用户项目的可借鉴点（二维动态障碍模拟器 + CPU/FPGA + 真实二维小车）

> 说明：以下"可直接搬 / 需改造 / 本仓库没有"的判断，都基于第 1–7 节读到的代码事实；标注 **本仓库没有** 的条目是需要自己实现的部分。

### 8.1 动态障碍环境内核（最贴合的现成参考）
| 你的模块 | 可借鉴对象 | 怎么搬 |
|---|---|---|
| 环境表示（SoA） | `PathPlanning/TimeBasedPathPlanning/GridWithDynamicObstacles.py` 的 `reservation_matrix[x, y, t]` | 现成实现是 `np.zeros((X, Y, T))`，**天然就是 SoA 的雏形**：把它改成 `(T, X, Y)` 或 `(X, Y)` 位平面 + T 层 bitmask（每层 `uint64` 位图）即可直接映射到 FPGA 的 BRAM/LUT；值语义 `0=空闲 / i+1=第 i 个障碍 / agent.index=被规划体占用` 可直接沿用。**关键语义**：时刻 t 同时占用"上一格 + 当前格"（`path[t-1]` 与 `path[t]` 都写），因此进入一格需要 `t+1`、`t+2` 两步都空闲——这个"进/出双步"约定请照抄，否则会低估碰撞。 |
| 确定性 | `BaseClasses.py` 的 `RANDOM_SEED = 50` + 导入时 `random.seed/numpy.random.seed` | 你已有固定时间步；把"随机源显式注入"做成接口（PRM 的 `rng=None` 参数是仓库里最好的范例），不要用全局 `random`。 |
| 安全区间预处理 | `GridWithDynamicObstacles.get_safe_intervals()` / `get_safe_intervals_at_cell()` | 用 `np.diff(zero_mask)` 把每格的 0 游程切成 `Interval(start, end)`，并**丢弃单步区间**。这是"离线预处理、在线 O(1) 查询"的典型结构，最适合做成 FPGA 的查表 ROM（`start/end` 数组）。 |
| 动态障碍运动生成 | `generate_dynamic_obstacles` / `obstacle_arrangement_1` / `generate_narrow_corridor_obstacles` | 三种生成器（随机游走带"0.8 概率原地"权重、y 向排列来回扫、静态窄通道）可直接作为你仿真器的**场景生成器**与回归基准。 |
| 结果的时空表示 | `Node.NodePath.positions_at_time` + `Plotting.PlotNodePath/PlotNodePaths` | `NodePath` 把"停留段/等速段"展开成"每个时刻的位置字典"，这是"规划结果 → 逐 tick 执行/渲染"的最小接口，建议在你的内核里保留同名字段（`positions_at_time[t]`）。 |
| 可视化/回放 | `Plotting.py`（逐时刻 `set_data` + `plt.pause`，Esc 退出） | 与控制解耦的"按时间重放"非常适合验证确定性：同一 `NodePath` 重放两次必须完全一致。 |

### 8.2 动态障碍避障 / 实时规划（他最需要的部分）
1. **首选：SIPP（`TimeBasedPathPlanning/SafeInterval.py`）**。理由（均为代码/文档事实）：状态仍是 `(x,y,t)`，但用"安全区间剪枝"把扩展数从时空 A\* 的 2717154 降到 322（同场景，文档实测），在 250 个随机障碍下 764 次扩展/0.6 s。搬用要点：① 节点携带 `interval`，② 剪枝不等式"同一区间内已扩展过入场时间 ≤ 候选入场时间的节点则跳过"，③ 入场时间取 `max(interval.start, parent.time+1)`。**改造建议**：把 `visited_intervals` 用定长数组（每格固定几个区间槽）替代 Python list-of-lists，`open_set` 用**按时间分桶的桶队列**替代 `heapq`（时间步是整数，桶队列 O(1) 且 FPGA 友好）。
2. **次选：时空 A\*（`SpaceTimeAStar.py`）**。当状态空间小或需要"时间最优"的严格解释时用它：`g(n) = n.time`、`h(n) = 曼哈顿距离`（可采纳且一致）、动作 5 个（4 连通 + 等待）。它是**最容易先在你现有网格内核里跑通**的动态避障算法。
3. **多动态体 / 多车：`PriorityBasedPlanner`**。它按"起终点距离降序"逐个规划并把已规划路径 `reserve_path` 进预约矩阵，后续规划者必须避开——**这正好对应"把其它动态障碍/其它车的预测轨迹当成已预约占用"**。注意它**不是 CBS**：次优、不重规划；若你要最优多机解，需要自己实现 CBS（本仓库没有）。
4. **高频局部避障：DWA（`DynamicWindowApproach/`）**。评价函数是三项目标（朝目标角差、速度、`1/min_r` 障碍代价）+ 速度窗口约束（本体极限 ∩ 加速度可达窗口），`predict_time=3.0` 前向预测。**动态化改造**很直接：把你的动态障碍按预测轨迹在每个预测时刻采样成点，替换 `config.ob` 的静态数组（或在 `calc_obstacle_cost` 里按轨迹索引的时间片取障碍位置），即可得到"考虑运动障碍的 DWA"。其评价函数只有加减乘除和比较，**非常适合定点化的 FPGA 流水线**。
5. **有参考线/结构化场景：Frenet 最优轨迹（`FrenetOptimalTrajectory/`）**。横向五次 + 纵向四次多项式采样、代价 = `K_J·jerk² + K_T·T + K_D·d² (+ 终端速度/停车代价)`，并有速度/加速度/曲率/碰撞四类约束过滤。**把 `check_collision` 从静态障碍数组改成"按预测时刻取障碍位置"**，就是论文原本针对的动态街道场景。另外 `cartesian_frenet_converter.py` 的解析互转是纯函数、无状态，**可以直接搬进你的内核**当"世界坐标 ↔ 弧长坐标"的转换层（FPGA 上三角函数是难点，可查表）。
6. **在线路径形变：ElasticBands**。用距离场（SDF）查表得到气泡半径 `ρ`，内部收缩力 + 障碍斥力 + 重叠维护，迭代固定次数（`MAX_ITER=50`）。**计算量可控、可截断**，适合做"全局路径已给出、局部避障微调"的 FPGA 模块；障碍变化时只需重算局部 SDF。注意代码里切向剔除是逐分量近似写法（`f − f·v·v/‖v‖²`），搬用时建议改成标准投影。
7. **预计算场方法（FPGA 最友好）：PotentialField / FlowField**。
   - `PotentialFieldPlanning`：`U_att = 0.5 KP·‖p−g‖`（线性引力）、`U_rep = 0.5 ETA(1/dq − 1/rr)²`（`dq ≤ rr`），整场预计算后 8 邻域贪心下降 + 振荡检测。场是**离线/低频**算好、在线只查表的典型结构。
   - `FlowField`：cost field（可通行代价 `1/7/20`）→ integration field（从目标 BFS，邻接代价 `10/14`）→ vector field（3×3 邻域最小 integration 的格）。**方向是"离散查表 + 无搜索"**，非常适合 FPGA；障碍变化时可只对受影响区域增量重算 integration field。
   - 两者都有局部极小/无动力学的问题，建议作为"全局引导 + 局部 DWA/弹性带"分层中的引导层。
8. **安全距离/间隙：`Mapping/DistanceMap` 的 Felzenszwalb 线性距离变换**（`O(W·H)`，两趟 1D 抛物线下包络 + 转置），以及 `compute_sdf` 的有符号版本。它是纯数组运算、**流水线友好**，可以直接用来做"到最近障碍/最近动态障碍的距离"以及弹性带/势场的 `ρ` 查询。仓库 docstring 还给了 scipy 版 (`distance_transform_edt`) 的实测速度对比（500×500 上 3 s vs 0.05 s），Python 端先用 scipy 版、硬件端再自己实现两趟变换是好路径。
9. **本仓库没有、你必须自己写的**：① 动态障碍的**轨迹预测**（CV/CA/CTRV 模型 + 过程噪声；可参考 `Localization/*` 里 EKF 的模型与 Q 参数化，但仓库没有现成的"障碍预测器"）；② 多机**最优**（CBS/ECBS）或带时间窗的冲突消解（只有优先级法）；③ 带动力学的时空搜索（时空 A\*/SIPP 都是完整约束、1 格/步）；④ 实时性预算内的"随时可中断/随时返回次优解"机制（仓库所有循环都是"跑到收敛"）。

### 8.3 状态估计与传感器模型（你的小车上会用到）
- **`Localization/gps_imu_fusion/gps_imu_fusion.py`**：8 维状态含**加速度计/陀螺零偏 + 随机游走**，预测用 Jacobian 解析式、更新用 **Joseph 形式**协方差（数值更稳）、并有 `GPS_INTERVAL` 与**中断区间**模拟。它的参数化（`IMU_STD/BIAS_RW_STD/GPS_STD/TRUE_BIAS`）与 8 字真值轨迹可直接当作你仿真器的"定位误差注入 + 估计器"基准。注意它**不做经纬度→ENU**，GPS 直接给米制位置——与你的"局部米制世界 + 确定性内核"完全吻合。
- **EKF / UKF / CKF / EnKF / PF / Histogram**：可作为"同一观测数据下不同估计器"的对比集。从 FPGA 视角：**直方图滤波（`histogram_filter`）把状态空间离散成 60×60 网格并做高斯核卷积 + 似然乘**，是最容易定点化/流水线化的；PF/EnKF 的随机采样与集合运算最不适合硬件；EKF/UKF/CKF 的矩阵维度小（4–5 维），可定点化但要处理三角函数与矩阵分解。
- **注意**：`ExtendedKalmanFilter`-类代码里 `Q/R` 的写法是 `diag([...])**2`（即把"标准差"平方成方差），搬参数时不要重复平方。

### 8.4 路径/轨迹表示与跟踪（从规划到小车执行）
- **路径参数化统一用 `CubicSpline2D`（`PathPlanning/CubicSpline/cubic_spline_planner.py`）**：它是全仓库的公共接口——`calc_position(s)`、`calc_yaw(s)`、`calc_curvature(s)`、`calc_curvature_rate(s)`，`calc_spline_course(x, y, ds)` 直接给离散课程。你的"规划输出 → 跟踪输入"用同一套 `(s, x, y, yaw, κ, κ̇)` 接口最省事。
- **跟踪器选择**：
  - 若你的小车是差速/阿克曼且只需求简单可靠，**纯追踪**（前视距离 `Lf = k v + Lfc`、`δ = atan2(2 WB sinα / Lf, 1)`）与 **Stanley**（`δ = θ_e + atan2(k·e_ct, v)`）都可直接搬；注意两者在本仓库里速度控制都只是**比例控制**（不是 PID），且 Stanley 没有 softening 项，低速时要自己加保护。
  - 若要"误差状态最优控制"，**LQR 转向**的 4 维误差模型 `[e, ė, θ_e, θ̇_e]`、`A[0,1]=dt, A[1,2]=v, A[2,3]=dt`、`B[3,0]=v/L`、前馈 `δ_ff = atan2(Lκ, 1)` 是现成的教学模板；但注意其 DARE 是"每步重解"且 `Q/R` 无标定，且 `L=0.5` 与其它模块不一致。
  - 若要**约束**（转向角/转角速率/速度/加速度）显式进入优化，**迭代线性 MPC** 的约束集与权重（`Q=diag([1,1,0.5,0.5])`、`R=diag([0.01,0.01])`、`Rd=diag([0.01,1.0])`、`MAX_STEER=45°, MAX_DSTEER=30°/s, MAX_SPEED=55/3.6, MIN_SPEED=-20/3.6, MAX_ACCEL=1.0`）可直接作为你 CPU 侧"高保真参考实现"，再与 FPGA 上的定点次优解对比。**注意它依赖 cvxpy/CLARABEL，无法直接在 FPGA 上跑**——FPGA 侧建议用 DWA 或"离线参数化的显式 MPC"。
- **带时间的轨迹原语**：`QuinticPolynomialsPlanner`（5 次多项式 + 时间网格搜索）、`Frenet`（横纵多项式）、`Eta3SplineTrajectory`（7 段速度剖面，受 `max_vel/max_accel/max_jerk` 约束）三者都可作为"点到点 + 时间参数化"的生成器；**它们的共同坑是循环/时间网格有隐含截断**（五次多项式不满足约束时静默返回 T=95；η³ 的牛顿迭代无上限），搬用时务必补显式失败返回。
- **可执行路径原语**：**Dubins（6 word）** 与 **Reeds-Shepp（12 基字 × 4 变换 = 48）** 是"给定最小转弯半径下的最短可执行路径"的闭式解，适合作为你小车的局部运动基元库；两者的 word/段长公式、离散化与代价定义都已在 §4.2.2 列出。**注意 RS 的 `step_size` 过大会直接返回空列表**（会打印提示）。

### 8.5 数据结构与工程做法
1. **把障碍几何转成栅格**：`Mapping/grid_map_lib/GridMap` 的 `set_value_from_polygon(..., inside=False)` + `expand_grid()`（膨胀机器人半径）是"几何 → 栅格 + 安全膨胀"的现成实现；**但要避开它的两个坑**（索引 0 被判失败、`expand_grid` 只扩 6 邻域）。
2. **统一 `Node` 的排序/相等/哈希约定**：`TimeBasedPathPlanning/Node.py` 的做法值得照抄——`__lt__` 用 `f = g + h`，`__eq__`/`__hash__` 用 "状态元组"（`(position, time)`），把"路径回溯用的 `parent_index`"排除在相等性之外。你自己的 SoA 内核里，这一步决定了去重集合（`expanded_set`）是否可靠。
3. **测试与验证做法**（本仓库最值得学的三点）：
   - `tests/conftest.py` 的"把项目根注入 `sys.path`" + 每个测试文件 `import conftest`，让测试能以包路径 import；（对你的项目：用 `pyproject.toml` + `pytest` 的更现代做法即可，但这个"最小可用"模式很省事。）
   - **`-Werror` 把警告当错误**（`runtests.sh`、`conftest.run_this_test`）——对确定性数值内核特别有价值，能及早暴露 NumPy 隐式类型转换/除零等隐患。
   - `tests/test_gps_imu_fusion.py` 的**用有限差分校验解析雅可比**、以及"用固定 seed 比较 RMSE"的写法，是给动力学/估计器写测试的好模板；反过来，仓库里 56 个"只调 `main()` 不断言"的冒烟测试**不要照抄**——你的确定性内核需要"给定固定输入 → 断言精确输出"的回归测试。
4. **论文索引习惯**：几乎每个文件头都写了来源（有的只有 URL，有的给出完整题录）。你可以在自己项目里沿用"文件头写参考"的做法，并把"作者+标题+年份"补全（见附录 A 的确认/未确认清单，本仓库大量条目缺作者或年份）。
5. **可复现性检查清单（结合本仓库的教训）**：固定 `dt` 与整数 tick；显式注入随机源；每个 `while` 循环都有迭代上限；把"收敛判据"（`eps/tol/cost_th`）全部参数化；把"安全区间/距离场/流场"这类预计算产物序列化落盘（本仓库的 `lookup_table.csv`、`path.npy`、`obstacles.npy` 是范例），这样 CPU 与 FPGA 可以共享同一份预处理数据。

### 8.6 CPU/FPGA 视角的"能不能搬"速查
| 算法族 | 定点/流水线友好度 | 理由（来自代码事实） |
|---|---|---|
| 势场 / 流场 / 距离变换 / 直方图滤波 / 栅格 Dijkstra | **高** | 全是数组化的查表、加法、乘法、比较；`FlowField` 的方向场与 `DistanceMap` 的两趟 1D 变换都是规则的顺序访问 |
| DWA | **中高** | 评价函数是三项加权和 + 若干比较；瓶颈是"速度窗口内双层采样 × 前向预测"，可固定采样档位并流水 |
| SIPP / 时空 A\* / D\* Lite | **中** | 状态与代价是整数运算，但有优先队列（可用桶队列替代）与指针式邻接；`H_COST`/`g/rhs` 需要定点标度 |
| 弹性带 | **中** | 迭代次数可截断（`MAX_ITER=50`），力与距离场查表都是卷积式运算；但是逐节点顺序更新，需并行化改造 |
| LQR / EKF / UKF / CKF | **中** | 矩阵维度小但含 `tan/atan2/sqrt/矩阵求逆/特征分解`；DARE 迭代（`solve_DARE`、`max_iter=150`）与小矩阵求逆是主要成本 |
| MPC（cvxpy/CLARABEL）/ C/GMRES | **低** | 需要通用凸优化/在线迭代求解器；适合放 CPU 做参考实现或"离线训练查表" |
| RRT\* / BIT\* / PSO / ICP / PF / EnKF / 图优化 SLAM | **低** | 随机采样、`sort`/`argmin` 全扫、SVD/稀疏线性求解、大规模浮点迭代——与 FPGA 的规则数据流不匹配 |

**分层建议（可直接落到你的项目结构）**：① **离线/低频（CPU）**：安全区间/距离场/流场/查找表预计算 + 全局时空规划（SIPP）；② **在线/高频（CPU→FPGA）**：DWA 或势场/流场梯度 + 弹性带微调，配上"动态障碍预测轨迹"作为时变障碍输入；③ **执行层**：纯追踪/Stanley/LQR 跟踪 + EKF/直方图滤波定位；④ **验证层**：固定 seed 的回归测试 + `-Werror` + "同一规划结果两次重放必须逐 tick 一致"的确定性检查。

---

## 附录 A：关键论文 / 出处索引

**作者 + 标题 + 年份齐全（可直接引用）**
| 算法/模块 | 出处 |
|---|---|
| lidar→grid map | Moravec, H. & Elfes, A., *High resolution maps from wide angle sonar*, Proc. IEEE ICRA, **1985** |
| 图优化 SLAM（教学版公式） | Grisetti, G., Kümmerle, R., Stachniss, C., Burgard, W., *A tutorial on graph-based SLAM*, IEEE Intelligent Transportation Systems Magazine 2(4):31–43, **2010** |
| 图优化 SLAM（SE(3) 参数化） | Blanco, J.-L., *A tutorial on SE(3) transformation parameterization and on-manifold optimization*, University of Málaga Tech. Rep. 3, **2010** |
| `graphslam/` 包与数据集 | Irion, J., **2020**（`python-graphslam`）；数据：Carlone, L. & Censi, A., *From angular manifolds to the integer lattice…*, IEEE T-RO 30(2):475–492, **2014** |
| PSO | Kennedy, J. & Eberhart, R., *Particle Swarm Optimization*, Proc. IEEE ICNN IV:1942–1948, **1995**；Shi, Y. & Eberhart, R., *A Modified Particle Swarm Optimizer*, IEEE ICEC, **1998**；Clerc, M. & Kennedy, J., IEEE TEC 6(1):58–73, **2002** |
| 二连杆逆运动学 | Corke, P. I., *Robotics, Vision and Control*, Springer, **2017**（p102） |
| RRT/Sobol | LaValle, S. M., *Planning Algorithms*, Cambridge University Press, **2006**；Bratley, P. & Fox, B. L., *Algorithm 659*, ACM TOMS 14(1):88–100, **1988**；Fox, B. L., *Algorithm 647*, ACM TOMS 12(4):362–376, **1986**；Sobol, I., USSR Comput. Math. & Math. Phys. 16:236–242, **1977**；Sobol & Levitan, Preprint IPM Akad. Nauk SSSR No. 40, Moscow, **1976** |
| GPS/IMU | Woodman, O. J., *An introduction to inertial navigation*, **2007**（Cambridge Tech. Report UCAM-CL-TR-696） |
| 火箭着陆 | Szmuk, M. & Acıkmese, B., *Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time*（**年份未在文件中给出 → 未确认**） |
| 距离变换 | Felzenszwalb, P. F. & Huttenlocher, D. P., *Distance Transforms of Sampled Functions*（**年份未确认**） |
| 弹性带 | *Elastic Bands: Connecting Path Planning and Control*（**作者/年份未确认**） |
| Spiral-STC | Gabriely et al., *Spiral-STC: An On-Line Coverage Algorithm of Grid Environments by a Mobile Robot*（**年份未确认**） |
| Wavefront CPP | Zelinsky et al., *Planning paths of complete coverage of an unstructured environment by a mobile robot*（**年份未确认**） |
| Frenet 轨迹 | Werling, M. et al., *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*（**正文未写作者/年份，仅 URL 含 `Moritz_Werling`**） |
| 其它 | 多数 Grid-based / Sampling / Tracking / Localization 条目只给出标题或 URL（详见 §4 各算法末尾的"论文/出处"），**作者/年份未在本地文件中出现**。 |

**项目自身引用**：*PythonRobotics: a Python code collection of robotics algorithms*，arXiv:1808.10703（`README.md` 的 Citing 一节；**作者名单未在本地文件中列出 → 未确认**）。

---

> **文档边界**：本文所有事实来自 `/home/zhang/Bullet_Platform/Outside/Algo/PythonRobotics` 的源码、`docs/`、`tests/`、CI 配置与 `README.md`；调研为只读（未修改、未构建、未安装、未联网、未运行示例）。凡标注「未确认」处，均表示该信息在本地文件中不可见，需要外部资料或实际运行验证。
