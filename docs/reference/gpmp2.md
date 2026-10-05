# GPMP2 技术总结

> 本文档由对本地仓库 `/home/zhang/Bullet_Platform/Outside/Algo/gpmp2` 的**只读**通读整理而成：
> 只读取代码/文档/目录，未修改任何已有文件，未构建、未联网。
> 每条结论后括号内标注来源路径（相对仓库根目录）。凡代码中查不到依据的，明确写“**未确认**”。

**版本基线（来自 git 元数据，`git log -1` / `git remote -v`）**

| 项 | 值 |
|---|---|
| 仓库来源 | `https://github.com/gtrll/gpmp2.git` |
| 分支 | `main` |
| 最新 commit | `4289d9a5a756006dae9ba8f4072ff4b95abf2e39`（2022-08-27，"update readme"） |
| 距最近 tag | `0.3.0-5-g4289d9a` |
| 工程版本号 | `0.3.0`（`CMakeLists.txt` 第 13-16 行；`gpmp2_python/gpmp2_python/version.py`） |

---

## 1. 一句话定位

**GPMP2 = 用「高斯过程先验 + 因子图」把连续时间运动规划问题重写成概率推断（MAP）问题，再用 GTSAM 的稀疏非线性最小二乘求解器求解。**

- 论文：*Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs*, **RSS 2016**（`README.md` 第 6 行，链接 `http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf`）。
- 作者：Jing Dong, Mustafa Mukadam, Frank Dellaert, Byron Boots（`README.md` 第 104-109 行的 BibTeX）。
- 机构：起步于 **Georgia Tech Robot Learning Lab**（`README.md` 第 6 行）；`THANKS.md` 列有贡献者。
- 后续/相关论文（`README.md` 第 94-118 行）：IJRR 2018 期刊版；ICRA 2018 "Sparse Gaussian Processes on Matrix Lie Groups"。
- 许可证：**BSD 3-Clause**，版权 `Copyright (c) 2016, Georgia Tech Research Corporation`（`LICENSE` 第 1-3 行，正文三条条款 + 免责声明）。
- 维护分支说明：`README.md` 第 1 行明确写 **“Version compatible with latest GTSAM is being maintained at [borglab/gpmp2]”**，即本仓库（`gtrll/gpmp2`）是旧版、绑定旧 GTSAM。
- GTSAM 版本要求：`README.md` 第 15 行要求 `GTSAM == wrap-export` 分支，并给出 `git checkout wrap-export` 的安装步骤（第 25 行）。**这是一个 2016-2017 年期的历史分支，不是 master/main。**

---

## 2. 技术栈与工具链

### 2.1 C++ 与 CMake

- CMake 最低版本 `3.0`，项目语言 `CXX C`（`CMakeLists.txt` 第 1-3 行）。
- **C++ 标准 C++14**（`CMakeLists.txt` 第 4 行：`set(CMAKE_CXX_STANDARD 14)`）。
- 全局关闭编译告警：`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")`（`CMakeLists.txt` 第 5 行）。
- Mac 上显式关闭 RPATH（`CMakeLists.txt` 第 7-10 行）。
- 头文件安装位置：各子模块 `CMakeLists.txt` 用 `install(FILES ... DESTINATION include/gpmp2/<subdir>)`。
- `gpmp2/config.h` 由 `gpmp2/config.h.in` 生成并安装，`GPMP2_EXPORT` 宏由此产生（`CMakeLists.txt` 第 53-57 行）。
- 兼容外部工程：`find_package(gpmp2 REQUIRED)` + `target_link_libraries(${TARGET} gpmp2)`（`doc/CplusplusDevelopment.md` 第 12-21 行）。

### 2.2 依赖

- **Boost ≥ 1.50**，需要 `filesystem / system / thread / serialization` 四个组件（`CMakeLists.txt` 第 44-48 行）。Boost.Serialization 被 `SignedDistanceField` 的存盘/读盘使用（`gpmp2/obstacle/SignedDistanceField.h` 第 17-23、198-208 行）；`boost::optional` 在几乎所有 factor 的雅可比参数里使用。
- **GTSAM**：`find_package(GTSAM REQUIRED)`，链接 `gtsam`（`CMakeLists.txt` 第 30-32 行）；使用 GTSAM 自带的 CMake 工具 `GTSAMCMakeTools` / `GtsamMakeConfigFile` / `GtsamBuildTypes` / `GtsamTesting`（第 34-37 行），单元测试通过 `gtsamAddTestsGlob` 注册（各子模块 `CMakeLists.txt`）。
- **为什么用 GTSAM**：README 第 15 行说明“use the factor graph implementations and inference/optimization tools provided by GTSAM”。GTSAM 提供 `NonlinearFactorGraph` / `Values` / `DoglegOptimizer` / `LevenbergMarquardtOptimizer` / `ISAM2` / `PriorFactor` / `Symbol` 等（见 `gpmp2/planner/BatchTrajOptimizer-inl.h`、`gpmp2/planner/ISAM2TrajOptimizer-inl.h`），GPMP2 只负责“建模”。

### 2.3 三个 CMake 编译选项（`CMakeLists.txt` 第 20-26 行）

| 选项 | 默认 | 含义 |
|---|---|---|
| `GPMP2_BUILD_STATIC_LIBRARY` | OFF | 构建静态库；**与 Matlab toolbox 互斥**（第 24-26 行 `FATAL_ERROR`） |
| `GPMP2_BUILD_MATLAB_TOOLBOX` | OFF | 构建 MATLAB mex 包装，需要 shared lib |
| `GPMP2_BUILD_PYTHON_TOOLBOX` | OFF | 构建 Cython/Python 包装，需要 shared lib |

### 2.4 Python 工具箱 —— 注意是 Python 2.7

- `README.md` 第 6 行：**"an optional Python 2.7 toolbox"**；第 47-55 行安装步骤是 `conda create -n gpmp2 pip python=2.7`，依赖 `cython numpy scipy matplotlib`。
- GTSAM 需要以 `-DGTSAM_INSTALL_CYTHON_TOOLBOX:=ON` 编译（`README.md` 第 63 行）。
- Python 包自身 `pip install -e .`，`requirements.txt` 只声明了 `numpy`（`gpmp2_python/requirements.txt`），`setup.py` 读取 `version.py` 作为包版本（`gpmp2_python/setup.py` 第 7-21 行）。
- 包装机制见 §6.1。

### 2.5 MATLAB 部分

- 开关 `GPMP2_BUILD_MATLAB_TOOLBOX`；通过 GTSAM 的 `GtsamMatlabWrap` 模块 `wrap_and_install_library(gpmp2.h ${PROJECT_NAME} ...)` 生成 mex 与 `.m`（`CMakeLists.txt` 第 67-74 行）。
- `matlab/CMakeLists.txt` 只做一件事：`install_matlab_scripts(".../+gpmp2/*.m")`。
- 全部 C++ 接口声明集中在仓库根的 `gpmp2.h`（`doc/CplusplusDevelopment.md` 第 87-99 行）。
- 已知环境坑：Ubuntu + GCC≥5 + 新版 Matlab 需 `export LD_PRELOAD=.../libstdc++.so.6:...`（`doc/Install.md` 第 24-35 行）。

---

## 3. 目录与模块地图

模块清单由 `gpmp2/CMakeLists.txt` 第 4-12 行的主列表决定：`geometry, gp, kinematics, dynamics, obstacle, planner, utils`。

| 目录 | 作用（来源） | 关键内容 |
|---|---|---|
| `gpmp2/gp` | GP 先验与 GP 插值（`doc/CplusplusDevelopment.md` 第 31 行） | `GPutils.h`（数学核心）、`GaussianProcessPrior{Linear,Lie,Pose2,Pose2Vector,Pose3}.h`、`GaussianProcessInterpolator{Linear,Lie,Pose2,Pose2Vector,Pose3}.h` |
| `gpmp2/kinematics` | 运动学 + 球体近似物理模型（第 32 行） | `ForwardKinematics.h`、`Arm.h/.cpp`、`RobotModel.h`、`PointRobot.h/.cpp`、`Pose2Mobile*`、关节/速度限制 factor、工作空间约束 factor |
| `gpmp2/dynamics` | 车辆动力学约束 | `VehicleDynamics.h`、`VehicleDynamicsFactor{Pose2,Vector,Pose2Vector}.h` |
| `gpmp2/obstacle` | SDF + 障碍代价 + 障碍 factor（第 33 行） | `PlanarSDF.h`、`SignedDistanceField.h/.cpp`、`ObstacleCost.h`、`ObstacleSDFFactor*`、`ObstaclePlanarSDFFactor*`、`SelfCollision*.h` |
| `gpmp2/planner` | 批量规划器与增量重规划器（第 34 行） | `TrajOptimizerSetting.h/.cpp`、`BatchTrajOptimizer.h/.cpp/-inl.h`、`ISAM2TrajOptimizer.h/-inl.h`、`TrajUtils.h/.cpp` |
| `gpmp2/geometry` | 动态尺寸李群工具 | `DynamicVector.h`、`Pose2Vector.h`、`ProductDynamicLieGroup.h`、`DynamicLieTraits.h`、`numericalDerivativeDynamic.h`、`utilsDynamic.h` |
| `gpmp2/utils` | 杂项工具（第 35 行） | `Timer.h`、`fileUtils.h/.cpp`（`.vol.head/.vol.data` 体素 SDF 读取）、`OpenRAVEutils`、`matlabUtils` |
| `gpmp2_python` | Python 包 + 示例（`README.md` 第 6 行） | `examples/`、`gpmp2_python/{datasets,robots,utils}`、`setup.py` |
| `matlab` | MATLAB 辅助函数与示例 | `+gpmp2/*.m`（含 `generateArm.m`、`signedDistanceField2D/3D.m`、各种 plot）、14 个 `*Example.m` |
| `projects/jist` | 独立子项目：**JIST**（IROS 2021，采样 + 轨迹优化交替），见 `projects/jist/README.md` | `jist/jist.py`（1098 行）、Hydra 配置 `configs/`、patrol guard / random forest 两个场景 |
| `doc` | 文档 | `index.md`、`Install.md`、`Parameters.md`、`CplusplusDevelopment.md`、`ExampleMatlab2D/3D.md`、`ExampleReplanning.md`、`pics/` |
| 根目录 | 包装声明与工程配置 | `gpmp2.h`（约 39 KB 的 wrap 声明）、`CMakeLists.txt`、`CHANGELOG`、`LICENSE`、`README.md`、`THANKS.md` |

`CHANGELOG` 给出的演化线索：0.1.0（2016-06 首版）→ 0.2.0（2017-06 Lie 群 GP 先验、SE(2)×Vector(N) 移动操作臂、SDF 二进制存取）→ 0.2.1（2021-07 关节/速度限制、workspace 约束、更多机器人模型）→ 0.3.0（2021-08 Python wrapper、自碰撞 factor、`Arm.h` 的 `theta_bias` 修复）。

---

## 4. 核心算法拆解

### 4.1 GP 先验：LTV-SDE 与精度矩阵

**符号约定**

- `θ ∈ R^d`：位姿/构型；`θ̇ ∈ R^d`：速度；`d = dof`。
- 支撑状态 `x_i = [θ_i; θ̇_i] ∈ R^{2d}`，在时间 `t_i` 上（`gpmp2/gp/GaussianProcessPriorLinear.h` 第 68-69 行）。
- `Δt = total_time / total_step`：相邻支撑状态的时间间隔（`gpmp2/planner/BatchTrajOptimizer-inl.h` 第 30 行）。
- `Qc ∈ R^{d×d}`：GP 超参（过程噪声谱密度）；`w(t)` 为白噪声。

**LTV-SDE（线性时变随机微分方程）**

GPMP2 的 GP 先验来源标注为 `Barfoot14rss`（`gpmp2/gp/GaussianProcessPriorLinear.h` 第 3 行），即 “白噪声驱动加速度” 模型：

```
ẋ(t) = A x(t) + u(t) + L w(t),      x(t) = [θ(t); θ̇(t)]
A = [[0, I],
     [0, 0]]        (2d × 2d)        L = [[0],
                                           [I]]        (2d × d)
E[w(t) w(t')ᵀ] = Qc δ(t − t')         （白噪声假设）
```

含义：速度是位姿的导数（`A` 的上三角 `I`），加速度被白噪声驱动（`L` 把噪声注入速度通道）。`u(t)` 是均值/控制项，在 GPMP2 里对应先验均值（初始轨迹）。

**状态转移矩阵**（`gpmp2/gp/GPutils.h` 第 42-46 行 `calcPhi`）：

```
Φ(τ) = [[I, τI],
        [0,  I]]
```

**过程噪声协方差 Q(τ)**（`GPutils.h` 第 25-30 行 `calcQ`）：

```
Q(τ) = [[ (τ³/3) Qc , (τ²/2) Qc ],
        [ (τ²/2) Qc ,   τ    Qc ]]      (2d × 2d)
```

**精度矩阵 / 信息矩阵 Q⁻¹(τ)**（`GPutils.h` 第 33-39 行 `calcQ_inv`）：

```
Q⁻¹(τ) = [[ 12τ⁻³ Qc⁻¹ , −6τ⁻² Qc⁻¹ ],
          [ −6τ⁻² Qc⁻¹ ,  4τ⁻¹ Qc⁻¹ ]]
```

这就是“由 SDE 离散化得到的精度矩阵”：白噪声假设下的 LTV-SDE 在区间 `[t_i, t_{i+1}]` 上解析积分，得到一个只依赖 `(Qc, τ)` 的闭式 `Q` 与 `Q⁻¹`。注意 `Q⁻¹` 是 `Qc⁻¹` 的分块线性组合（系数为 τ 的幂），因此 `Qc` 越“大”，先验越松（轨迹越平滑但约束越弱）。

**Qc 的获取**（`gpmp2/gp/GPutils.cpp` 第 16-20 行 `getQc`）：

```cpp
Matrix getQc(const SharedNoiseModel Qc_model) {
  noiseModel::Gaussian *Gaussian_model = dynamic_cast<noiseModel::Gaussian*>(Qc_model.get());
  return (Gaussian_model->R().transpose() * Gaussian_model->R()).inverse();
}
```

即：把 GTSAM 的 `Gaussian::Covariance(Qc)` 噪声模型转回协方差 `Qc = (RᵀR)⁻¹`。所以调用方传 `Qc_model = noiseModel::Gaussian::Covariance(Qc)` 时，`getQc` 恰好返回 `Qc`。

**GP 先验因子 = 4 元因子**（`gpmp2/gp/GaussianProcessPriorLinear.h`）

- 类型：`GaussianProcessPriorLinear : gtsam::NoiseModelFactor4<Vector,Vector,Vector,Vector>`，连接 `(x_i, v_i, x_{i+1}, v_{i+1})`。
- 噪声模型：`noiseModel::Gaussian::Covariance(calcQ(getQc(Qc_model), Δt))`（第 43 行）。
- 残差（第 82 行）：

```
e_GP(x_i, x_{i+1}) = Φ(Δt) x_i − x_{i+1}
                   = [ θ_i + Δt·θ̇_i − θ_{i+1} ;
                       θ̇_i − θ_{i+1} ]
```

- 雅可比是常数矩阵（第 72-79 行）：`H1 = [I; 0]`，`H2 = [ΔtI; I]`，`H3 = [−I; 0]`，`H4 = [0; −I]`（均为 `2d × d`）。这正是“匀速模型”离散残差。
- `dof()` 返回 `d`，`size()` 返回 4（第 87-92 行）。

**李群版本**（`gpmp2/gp/GaussianProcessPriorLie.h`）：对任意满足 `IsLieGroup` 的 `T`（第 33 行 `BOOST_CONCEPT_ASSERT`），位姿用 `Log(pose1⁻¹ · pose2)` 代替向量差（第 74-77 行），残差为

```
e = [ Log(pose1⁻¹ pose2) − θ̇_i · Δt ;
      θ̇_{i+1} − θ̇_i ]
```

噪声模型同样是 `Covariance(calcQ(getQc(Qc_model), Δt))`（第 49 行）。别名：`GaussianProcessPriorPose2 = GaussianProcessPriorLie<Pose2>`（`GaussianProcessPriorPose2.h` 第 15 行）、`GaussianProcessPriorPose2Vector = GaussianProcessPriorLie<Pose2Vector>`（`GaussianProcessPriorPose2Vector.h` 第 15 行）；`GaussianProcessPriorPose3.h` 同构。

**关于 Matérn 核 / 指数核**：**未确认**。全仓库检索 `matern|kernel`（含 `*.h/*.cpp/*.md/*.py`）没有 Matérn 核或指数核的实现；仓库只有 `Qc` 这一个“超参”接口（`doc/Parameters.md` 第 54 行：“Qc_model: GP hyperparameter”）。也就是说本实现用的是 **LTV-SDE（白噪声驱动加速度）等价核**，而不是显式的 Matérn/指数核函数。若需要 Matérn 核，得自行构造 `K(t,t')` 并求其逆，GPMP2 没提供。

**GP 插值 / 去噪（非优化的连续时间重建）**（`gpmp2/gp/GaussianProcessInterpolatorLinear.h`，数学核心 `GPutils.h` 第 48-59 行）：

```
Λ(τ) = Φ(τ) − Q(τ) Φ(Δt−τ)ᵀ Q⁻¹(Δt) Φ(Δt)
Ψ(τ) = Q(τ) Φ(Δt−τ)ᵀ Q⁻¹(Δt)
z(τ) = Λ(τ) x_i + Ψ(τ) x_{i+1},     τ ∈ [0, Δt]
```

构造时预算并缓存 `Qc_ / Lambda_ / Psi_`（第 52-54 行）；`interpolatePose` 只取前 `d` 行、`interpolateVelocity` 只取后 `d` 行以省算力（第 77-83、115-121 行）。`tau_` 的注释明确说明“以 `t_i` 为起点、而不是论文里的 `t_0`”（第 31 行）。

**李群插值**（`gpmp2/gp/GaussianProcessInterpolatorLie.h` 第 74-99 行）：把 `x_i` 映射到李代数再 Expmap 复合回去：

```
r1 = [0; θ̇_i],  r2 = [ Log(pose1⁻¹ pose2); θ̇_{i+1} ]
pose(τ) = pose1 · Exp( Λ_pose(τ) r1 + Ψ_pose(τ) r2 )
```

位置用姿态切空间表示“相对位移 + 速度”，因此 SE(2) 上插值后仍是合法刚体变换（与 CHOMP/GPMP2 论文中 SE(2) 的处理一致）。

**输入/输出与复杂度（§4.1 小结）**

| 项 | 内容 |
|---|---|
| 输入 | 相邻两状态 `(θ_i, θ̇_i, θ_{i+1}, θ̇_{i+1})`、`Δt`、`Qc_model` |
| 输出 | `GaussianProcessPrior*` factor（残差维度 `2d`） |
| 关键参数 | `Qc`（默认 identity，`TrajOptimizerSetting.cpp` 第 50 行 `Qc_model(noiseModel::Unit::Create(system_dof))`）、`Δt` |
| 复杂度 | 单因子 `evaluateError` 为 `O(d)`（常数矩阵乘向量）；构造时 `calcQ_inv` 需一次 `d×d` 求逆，即 `O(d³)` 一次、之后可缓存 |
| 实现位置 | `gpmp2/gp/GPutils.h`、`gpmp2/gp/GaussianProcessPriorLinear.h`、`.../GaussianProcessPriorLie.h`、`.../GaussianProcessInterpolator*.h` |

---

### 4.2 因子图与 MAP 推断

**MAP 形式化**

记待估变量 `Θ = {x_i, v_i}_{i=0..T}`，观测/约束集合 `e`。贝叶斯公式 `p(Θ|e) ∝ p(Θ) p(e|Θ)`，MAP 解为

```
Θ* = argmax_Θ p(Θ|e)
   = argmin_Θ { ‖e_prior(Θ)‖²_{Σ_prior} + Σ_i ‖e_i(Θ_i)‖²_{Σ_i} }
```

其中 `‖e‖²_Σ = eᵀ Σ⁻¹ e`（白化后的二范数平方）。GPMP2 把三样东西各写成一个 factor：

1. **GP 先验**：`GaussianProcessPrior*`，残差 `Φ(Δt)x_i − x_{i+1}`，协方差 `Q(Δt)`（§4.1）。
2. **端点先验**：GTSAM 的 `PriorFactor`，把 `x_0, v_0, x_T, v_T` 固定到给定起终点（`BatchTrajOptimizer-inl.h` 第 41-48 行）。
3. **障碍软代价**：`Obstacle*SDFFactor*`，一元（每个支撑状态一个）或四元（GP 插值出的中间状态）（§4.3）。

**图的构建循环**（`gpmp2/planner/BatchTrajOptimizer-inl.h` 第 36-81 行）——这是整个规划器的骨架：

```cpp
for (size_t i = 0; i <= setting.total_step; i++) {
  Key pose_key = Symbol('x', i);  Key vel_key = Symbol('v', i);
  if (i == 0)             { add PriorFactor<Pose>(x0, start_conf); add PriorFactor<Vel>(v0, start_vel); }
  else if (i == total_step){ add PriorFactor<Pose>(xT, end_conf);  add PriorFactor<Vel>(vT, end_vel); }
  if (flag_pos_limit) add LIMIT_FACTOR_POS(x_i, ...);
  if (flag_vel_limit) add LIMIT_FACTOR_VEL(v_i, ...);
  add OBS_FACTOR(x_i, arm, sdf, cost_sigma, epsilon);           // 一元障碍代价
  if (i > 0) {
    for (j = 1..obs_check_inter)                                // GP 插值出的中间状态代价
      add OBS_FACTOR_GP(x_{i-1}, v_{i-1}, x_i, v_i, arm, sdf, cost_sigma, epsilon, Qc_model, delta_t, tau);
    add GP(x_{i-1}, v_{i-1}, x_i, v_i, delta_t, Qc_model);      // GP 先验
  }
}
```

**求解器与迭代循环**（`gpmp2/planner/BatchTrajOptimizer.cpp` 第 212-308 行 `optimize()`）

- `setting.opt_type` 三选一：`Dogleg`（默认，第 24/51 行）/ `LM` / `GaussNewton`，分别构造 `DoglegOptimizer` / `LevenbergMarquardtOptimizer` / `GaussNewtonOptimizer`。
- 硬编码的初始信任域/阻尼：Dogleg `setDeltaInitial(0.2)`（第 222 行，注释“0.2 rad or meter”）；LM `setlambdaInitial(100.0)`（第 226 行）。
- `params->setMaxIterations(setting.max_iter)`、`setRelativeErrorTol(setting.rel_thresh)`（第 233-234 行），终止条件用 GTSAM 的 `checkConvergence(rel, abs, errorTol, ...)`（第 285-286 行）。
- **“误差不增”保护**：迭代前 `last_values = opt->values()`，循环结束后若 `opt->error() > currentError` 则返回上一轮的值（第 277-278、296-307 行）。这是 GPMP2 相对 GTSAM 默认 `optimize()` 的唯一行为差异，由 `setting.final_iter_no_increase`（默认 `true`）控制。

**稀疏性来源**

- 图是**链式（chain）结构**：`x_i,v_i` 只与 `x_{i±1},v_{i±1}` 通过 GP 因子相连，障碍因子里 GP 版本也只跨相邻时间步（`OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, ...)`，第 72-73 行）。
- 因此 GTSAM 的消除（elimination）后 Hessian 呈**带状/分块三对角**，等价 Bayes net 深度为 `O(T)`、每个团（clique）规模为常数（与 `d` 和 `obs_check_inter` 有关，与 `T` 无关）。对 `N = 2(T+1)` 个变量，稀疏 Cholesky 复杂度约 `O(N · d³)` 而不是 `O(N³)`。
- **Bayes tree**：批量优化器走的是直接的 LM/Dogleg/GaussNewton（Hessian 分解），仓库里**没有**显式使用 Bayes tree 的批量路径；Bayes tree / 增量消元出现在 **`ISAM2TrajOptimizer`**（`gpmp2/planner/ISAM2TrajOptimizer.h` 第 77 行 `gtsam::ISAM2 isam_`；构造参数见 `ISAM2TrajOptimizer-inl.h` 第 20-22 行 `ISAM2Params(ISAM2GaussNewtonParams(), 1e-3 /*relinearizeThreshold*/, 1 /*relinearizeSkip*/)`）。

**输入/输出**

| 项 | 内容 |
|---|---|
| 输入 | `NonlinearFactorGraph graph`、初始 `Values init_values`、`TrajOptimizerSetting setting`、`bool iter_no_increase=true` |
| 输出 | `gtsam::Values`（键 `x0..xT`, `v0..vT`） |
| 复杂度 | 每次迭代 `O(T·d³)`（稀疏分解）+ 每因子一次 FK 与 SDF 查询；迭代次数 `max_iter` |
| 实现位置 | `gpmp2/planner/BatchTrajOptimizer-inl.h`、`gpmp2/planner/BatchTrajOptimizer.cpp` |

---

### 4.3 障碍因子：SDF、hinge loss 与解析梯度

**有符号距离场（SDF）**

- **2D**：`PlanarSDF`（`gpmp2/obstacle/PlanarSDF.h`）。数据是 `gtsam::Matrix data_`，**行 = Y、列 = X**（第 21-24 行注释、第 90-100 行双线性插值的下标用法）。字段：`origin_ (Point2)`、`field_rows_/field_cols_`、`cell_size_`。
  - 双线性插值 `signed_distance(float_index)`（第 90-100 行）；
  - 梯度 `gradient(float_index)`（第 105-116 行）由四个角点的差分插值得到，单位换算 `g = (g_idx(1), g_idx(0)) / cell_size_`（第 63 行）——**注意 x/y 分量被交换**，这是行列顺序与坐标系差异导致的；
  - 越界抛 `SDFQueryOutOfRange`（第 71-75 行）。
- **3D**：`SignedDistanceField`（`gpmp2/obstacle/SignedDistanceField.h`）。数据是 `std::vector<gtsam::Matrix>`（每个 z 层一个矩阵），三线性插值（第 127-141 行）、三线性梯度（第 146-167 行），梯度换算 `(g_idx(1), g_idx(0), g_idx(2)) / cell_size_`（第 97 行）。
  - 支持 `saveSDF/loadSDF`，按扩展名分派 xml/bin/text 的 Boost.Serialization 归档（第 191-194 行 + `SignedDistanceField.cpp` 第 12-47 行）。
  - `utils/fileUtils.cpp` 另有 `readSDFvolfile()` 读 `.vol.head` + `.vol.data` 体素文件（第 17-62 行）。

**hinge loss 代价（这是障碍“软约束”的核心）**（`gpmp2/obstacle/ObstacleCost.h` 第 26-50 行 / 第 54-78 行）：

```
给定点 p、SDF d(p)（内部为负、外部为正）、安全距离 eps:
  d(p) > eps :  cost = 0,           ∂cost/∂p = 0
  d(p) ≤ eps :  cost = eps − d(p),  ∂cost/∂p = −∇d(p)
```

- 梯度是**解析的**：直接用 SDF 返回的场梯度（第 47、75 行）。只有 SDF 查询越界时才退化为 0 梯度并返回 0 代价（第 36-38、64-66 行），且原来的 warning 打印已被注释掉——**这是一个静默失灵的坑**（见 §8）。
- `eps` 在 factor 内部会被**加上球体半径**：`total_eps = robot_.sphere_radius(sph_idx) + epsilon_`（`ObstaclePlanarSDFFactor-inl.h` 第 40 行）。所以 `epsilon_` 是“球面到障碍的净安全距离”，而不是球心到障碍的距离。

**三元/四元 factor 与链式法则**

- **一元（非插值）**：`ObstaclePlanarSDFFactor<ROBOT> : NoiseModelFactor1<ROBOT::Pose>`（`ObstaclePlanarSDFFactor.h` 第 28-29 行）。
  - 噪声模型：`noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma)`（第 67 行）——误差维度 = **球体个数**，每个球一个残差，权重 `1/cost_sigma²`。
  - `evaluateError`（`ObstaclePlanarSDFFactor-inl.h` 第 17-57 行）：
    1. `robot_.sphereCenters(conf, sph_centers, J_px_jp)`（带雅可比的正运动学，§4.4）；
    2. 对每个球跑 hinge loss 得到 `err(sph_idx)`；
    3. 链式法则 `H1.row(k) = Jerr_point * J_px_jp[k].topRows<2>()`（第 48 行）——因为 `hingeLossObstacleCost` 返回的是对 2D 点 `(x,y)` 的 `1×2` 雅可比，只取前两行（忽视 z）。
- **四元（GP 插值）**：`ObstaclePlanarSDFFactorGP<ROBOT, GPINTER> : NoiseModelFactor4<Pose,Velocity,Pose,Velocity>`（`ObstaclePlanarSDFFactorGP.h` 第 28-30 行）。
  - 成员持有 `GPBase GPbase_`（第 45 行），构造时传 `(Qc_model, delta_t, tau)`（第 79 行）。
  - `evaluateError`（`ObstaclePlanarSDFFactorGP-inl.h` 第 19-78 行）：先用 GP 插值出中间时刻位姿 `conf = GPbase_.interpolatePose(conf1,vel1,conf2,vel2, Jconf_c1,Jconf_v1,Jconf_c2,Jconf_v2)`（第 36 行），再做与一元相同的 FK + hinge loss 得到 `Jerr_conf`，最后 `GPBase::updatePoseJacobians(Jerr_conf, ...)` 把对插值位姿的雅可比回传到四个变量（第 74-75 行）。
  - 这就是“在支撑状态之间加密检查障碍”的机制：`obs_check_inter` 个四元 factor，`tau = j·inter_dt`（`BatchTrajOptimizer-inl.h` 第 70-74 行）。

**Factor 家族（typdef 全清单，来源 `gpmp2/obstacle/*.h`）**

| 基类 | 语义 | 2D 具体别名 | 3D 具体别名 |
|---|---|---|---|
| `ObstaclePlanarSDFFactor<ROBOT>` | SDF(2D) 一元 | `...FactorArm`、`...FactorPointRobot`、`...FactorPose2MobileBase`、`...FactorPose2MobileArm`、`...FactorPose2Mobile2Arms` | — |
| `ObstaclePlanarSDFFactorGP<ROBOT,GPINTER>` | SDF(2D) 四元 + GP 插值 | `...GPArm`（`GPINTER=GaussianProcessInterpolatorLinear`）、`...GPPointRobot`、`...GPPose2MobileBase`（`InterpolatorPose2`）、`...GPPose2MobileArm`、`...GPPose2Mobile2Arms`（`InterpolatorPose2Vector`） | — |
| `ObstacleSDFFactor<ROBOT>` | SDF(3D) 一元 | — | `...Arm`、`...Pose2MobileBase`、`...Pose2MobileArm`、`...Pose2Mobile2Arms`、`...Pose2MobileVetLinArm`、`...Pose2MobileVetLin2Arms` |
| `ObstacleSDFFactorGP<ROBOT,GPINTER>` | SDF(3D) 四元 + GP 插值 | — | 同上对应的 `...GP*` 系列 |

（`ObstaclePlanarSDFFactorArm.h` 第 16 行、`ObstaclePlanarSDFFactorGPArm.h` 第 17 行、`ObstacleSDFFactorArm.h` 第 16 行等提供 typedef。注意 `Pose2MobileVetLin*` 只有 3D 版本。）

**自碰撞**（`gpmp2/obstacle/SelfCollision.h`，CHANGELOG 0.3.0 新增）：`SelfCollision<ROBOT> : NoiseModelFactor1<Pose>`，用 `N×4` 矩阵描述球对 `(idA, idB, eps, sigma)`（第 39-46 行）；代价 `hingeLossSelfCollisionCost` 用 `gtsam::distance3(pointA, pointB)` 的解析梯度（第 108-128 行）；噪声模型 `Diagonal::Sigmas(data.col(3))`（第 60 行）。别名 `SelfCollisionArm` 见 `SelfCollisionArm.h`。

**输入/输出与复杂度**

| 项 | 内容 |
|---|---|
| 输入 | 位姿（或 4 元 `(pose,vel,pose,vel)`）、`ROBOT`、`SDF`、`cost_sigma`、`epsilon`、（GP 版）`Qc_model, delta_t, tau` |
| 输出 | 长度 `= nr_body_spheres()` 的残差向量，可选 `H`（`nr_spheres × dof`） |
| 关键参数 | `epsilon`（安全距离，默认 0.2）、`cost_sigma`（默认 0.1）、`obs_check_inter`（默认 5） |
| 复杂度 | 每因子 `O(nr_spheres · (FK + SDF 查询))`；一元 factor 数 `T+1`，四元 factor 数 `T·obs_check_inter` |
| 实现位置 | `gpmp2/obstacle/{ObstacleCost.h, PlanarSDF.h, SignedDistanceField.h, Obstacle*Factor*.h}` |

---

### 4.4 运动学

**抽象层**

- `ForwardKinematics<POSE, VELOCITY>`（`gpmp2/kinematics/ForwardKinematics.h`）：纯虚 `forwardKinematics(jp, jv, jpx, jvx, J_jpx_jp, J_jvx_jp, J_jvx_jv)`（第 58-62 行），持有 `dof_` 与 `nr_links_`。同时提供给 MATLAB 用的矩阵版本 `forwardKinematicsPose/Position/Vel`（第 70-72 行，实现在 `ForwardKinematics-inl.h`）。
- `RobotModel<FK>`（`gpmp2/kinematics/RobotModel.h`）：把 FK 与**球体近似**组合成物理模型。
  - `struct BodySphere { size_t link_id; double radius; gtsam::Point3 center; }`（第 20-27 行），容器 `typedef std::vector<BodySphere> BodySphereVector`（第 31 行）。
  - `sphereCenters(jp, sph_centers, J_point_conf)`（`RobotModel-inl.h` 第 12-40 行）：先 FK 得各连杆位姿 `link_poses` 与 `J_pose_jp`，再对每个球做 `link_poses[link_id].transform_from(center, J_point_pose)`，链式法则 `J_point_conf = J_point_pose * J_pose_jp[link_id]`（第 33 行）。
  - 便捷接口 `sphereCenter(idx,...)`（单个球，第 44-68 行）、`sphereCentersMat(jp)`（3×N 矩阵，给 MATLAB，第 72-82 行）。

**DH 参数的机械臂**（`gpmp2/kinematics/Arm.h` / `Arm.cpp`）

- `Arm : ForwardKinematics<gtsam::Vector, gtsam::Vector>`；成员 `a_, alpha_, d_, theta_bias_` 与 `mutable gtsam::Pose3 base_pose_`（第 33-35 行）。
- 构造时按 Spong《Robot Modeling and Control》eq. (3.10) 预算**不含 θ 的连杆变换**（`Arm.cpp` 第 22-27 行）：

```
link_trans_notheta_[i] = Trans_z(d_i) · Trans_x(a_i) · Rot_x(alpha_i)
```

- 含 θ 的变换（`Arm.h` 第 93-98 行）：

```
H_i(θ_i) = Rot_z(θ_i + theta_bias_i) · link_trans_notheta_[i]
```

  即教科书 DH 约定，`theta_bias` 用于对齐关节零位（CHANGELOG 0.3.0 提到修过它的 bug）。
- FK 与雅可比（`Arm.cpp` 第 31-143 行）：
  - `Ho[i] = Ho[i-1] * H[i-1]`（第 68 行），`Ho[0] = base_pose_`（第 58 行）；
  - 线速度雅可比列 `Jv_j = skew(z_j) · (p_i − p_j)`（`Arm.h` 第 117-127 行 `getJvj`）；
  - 位姿雅可比通过对 `Ho` 求导得到：缓存 `dHo_dq[i][j]`（`O(dof²)` 个 4×4 矩阵，`Arm.cpp` 第 85-92 行），然后 `sym_se3 = inv(jpx_i) * dHo_dq[i][j]`，取反对称部分与平移部分拼成 `6×dof` 列（第 109-114 行）。**这就是 `gpmp2/kinematics` 里唯一手写的解析雅可比，其余全靠 `numericalDerivativeDynamic` 或链式法则。**

**具体机器人模型清单**

| 抽象 FK 类 | 状态类型 | 说明（来源） |
|---|---|---|
| `Arm(dof, a, alpha, d, base_pose, theta_bias)` | `Vector` | DH 机械臂（`Arm.h` 第 47-55 行） |
| `PointRobot(dof, nr_links)` | `Vector` | FK 是恒等映射 `Pose3(Rot3(), Point3(q0,q1,0))`，雅可比为单位块（`PointRobot.cpp` 第 31-59 行） |
| `Pose2MobileBase` | `Pose2` | SE(2) 移动底盘，`Base(3,1)`（`Pose2MobileBase.h` 第 33 行） |
| `Pose2MobileArm(arm, base_T_arm)` | `Pose2Vector` | 平面底盘 + 机械臂的组合（`Pose2MobileArm.h` 第 27-43 行） |
| `Pose2Mobile2Arms` | `Pose2Vector` | 平面底盘 + 双臂 |
| `Pose2MobileVetLinArm` / `Pose2MobileVetLin2Arms` | `Pose2Vector` | 带垂直直线执行器的底盘 + 单/双臂 |

**物理模型 typedef**（`gpmp2/kinematics/*Model.h`）：`ArmModel = RobotModel<Arm>`（`ArmModel.h` 第 19 行）、`PointRobotModel = RobotModel<PointRobot>`（`PointRobotModel.h`）、`Pose2MobileBaseModel`、`Pose2MobileArmModel`、`Pose2Mobile2ArmsModel`、`Pose2MobileVetLinArmModel`、`Pose2MobileVetLin2ArmsModel`。

**李群乘积类型**（`gpmp2/geometry/`）

- `Pose2Vector : ProductDynamicLieGroup<gtsam::Pose2, DynamicVector>`（`Pose2Vector.h` 第 26 行），访问器 `pose()` 与 `configuration()`（第 44-45 行），并特化 `gtsam::traits<Pose2Vector>`（第 61-73 行）。
- `DynamicVector`：对 `Eigen::VectorXd` 的薄包装，补齐 `VectorSpace` traits（`DynamicVector.h` 第 26-99 行，traits 在第 107-109 行）；`dimension = Eigen::Dynamic`。
- `numericalDerivativeDynamic`：动态尺寸的中心差分（`numericalDerivativeDynamic.h` 第 26-65 行，`delta = 1e-5`），用于**测试**里校验解析雅可比，也用于部分无法解析求导的场景。

**其它 factor（`gpmp2/kinematics/`）**

| 类 | 作用 | 来源 |
|---|---|---|
| `JointLimitFactorVector` | 关节位置软限位，逐关节 hinge loss，雅可比对角 | `JointLimitFactorVector.h` 第 48-79 行 |
| `JointLimitFactorPose2Vector` | 同上，只作用于 `Pose2Vector` 的向量部分 | `JointLimitFactorPose2Vector.h` |
| `VelocityLimitFactorVector` | 速度限幅（用 `hingeLossJointLimitCost` 的对称形式） | `VelocityLimitFactorVector.h`、`JointLimitCost.h` 第 16-31 行 |
| `GoalFactorArm` | 末端位姿目标 | `GoalFactorArm.h` |
| `GaussianPriorWorkspacePosition/Orientation/Pose` | 工作空间位置/姿态/位姿高斯先验（如“末端保持水平”） | `GaussianPriorWorkspacePosition.h` 第 53-71 行等；示意见 `matlab/WAMWorkspaceConstraintsExample.m` |

`hingeLossJointLimitCost(p, down, up, thresh, H_p)` 的分段定义（`JointLimitCost.h`）：

```
p < down + thresh        : return down + thresh − p,   H = −1
down+thresh ≤ p ≤ up−thresh : return 0,                H = 0
p > up − thresh          : return p − up + thresh,      H = +1
```

（`thresh` 是缓冲带，`TrajOptimizerSetting` 默认 `pos_limit_thresh = vel_limit_thresh = 0.001·1`。）

**车辆动力学**（`gpmp2/dynamics/`，用于移动底盘）

```
simple2DVehicleDynamicsPose2(p, v)   = v(1)                       （侧滑速度，越接近 0 越好）
simple2DVehicleDynamicsVector3(p, v) = v(1)cos(p(2)) − v(0)sin(p(2))
```

（`VehicleDynamics.h` 第 19-40 行，两者都给出解析雅可比。）包装为 `VehicleDynamicsFactorVector`（`NoiseModelFactor2<Vector,Vector>`，残差 1 维，`VehicleDynamicsFactorVector.h` 第 45-77 行）、`VehicleDynamicsFactorPose2`、`VehicleDynamicsFactorPose2Vector`。

**输入/输出与复杂度（§4.4 小结）**

| 项 | 内容 |
|---|---|
| 输入 | `jp ∈ R^d`（或 `Pose2` / `Pose2Vector`），可选速度 |
| 输出 | `std::vector<Pose3> jpx`（长度 `dof`）、可选 `jvx`、`J_jpx_jp`（每个 `6×dof`）、`J_jvx_jp`（`3×dof`） |
| 复杂度 | `Arm::forwardKinematics` 带雅可比时 `O(dof²)` 个 4×4 矩阵运算（`dHo_dq` 缓存），不带雅可比时 `O(dof)` 矩阵乘 |
| 实现位置 | `gpmp2/kinematics/{Arm.h,Arm.cpp,RobotModel.h,RobotModel-inl.h,PointRobot.cpp,...}` |

---

### 4.5 GPMP2 vs CHOMP

**未确认 / 仓库内无依据。** 对全仓库（`*.md/*.h/*.cpp/*.m/*.py`）检索 `chomp`（不区分大小写）**零命中**。也就是说：

- 本仓库**没有** CHOMP 实现、没有 CHOMP 对比脚本、数据集或实验结果；
- `README.md` 与 `doc/` 也未做文字层面的 GPMP2 vs CHOMP 论述；
- 仓库内的“对比/评估”只有两处工程化痕迹：`CHANGELOG` 与 `projects/jist`（JIST = 采样 + 轨迹优化对比，论文是 IROS 2021）。

如需 CHOMP 对比结论，只能回到 RSS 2016 / IJRR 2018 论文（不在本仓库内，且本次任务不联网，故不引用）。

---

### 4.6 规划器

#### 4.6.1 批量规划器 `BatchTrajOptimize*`

入口是一组**自由函数**（不是类），声明在 `gpmp2/planner/BatchTrajOptimizer.h`：

```cpp
Values BatchTrajOptimize2DArm(const ArmModel&, const PlanarSDF&,
        const Vector& start_conf, const Vector& start_vel,
        const Vector& end_conf,   const Vector& end_vel,
        const Values& init_values, const TrajOptimizerSetting&);
Values BatchTrajOptimize3DArm(...);                       // SignedDistanceField
Values BatchTrajOptimizePose2MobileArm2D(...);            // Pose2Vector
Values BatchTrajOptimizePose2MobileArm(...);
Values BatchTrajOptimizePose2Mobile2Arms(...);
Values BatchTrajOptimizePose2MobileVetLinArm(...);
Values BatchTrajOptimizePose2MobileVetLin2Arms(...);
```

它们都只是转调同一个模板（`BatchTrajOptimizer.cpp` 第 40-128 行），例如 2D 机械臂：

```cpp
internal::BatchTrajOptimize<ArmModel, GaussianProcessPriorLinear,
    PlanarSDF, ObstaclePlanarSDFFactorArm, ObstaclePlanarSDFFactorGPArm,
    JointLimitFactorVector, VelocityLimitFactorVector>(...);
```

模板参数含义（`BatchTrajOptimizer.h` 第 218-228 行注释）：`ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL`。
**所以“换机器人/换 SDF/换约束”只需换模板实参，图结构不变——这是本项目最重要的一处工程设计。**

流水线（一次调用完整流程）：

1. **初始化**：调用方先用 `initArmTrajStraightLine` / `initPose2VectorTrajStraightLine` / `initPose2TrajStraightLine` 生成配置空间直线轨迹，速度全部填平均速度（`TrajUtils.cpp` 第 25-93 行）。也可以自己构造 `Values`。
2. **算时间参数**：`delta_t = total_time/total_step`，`inter_dt = delta_t/(obs_check_inter+1)`（`BatchTrajOptimizer-inl.h` 第 30-31 行）。注意 `+1`：`obs_check_inter` 个插值点把区间分成 `obs_check_inter+1` 段。
3. **建图**（§4.2 的循环）。
4. **求解**：`optimize(graph, init_values, setting)`（上文 §4.2）。
5. **（可选）加密**：`interpolateArmTraj(values, Qc_model, delta_t, inter_step[, start, end])` 用 GP 插值生成稠密轨迹给执行器用（`TrajUtils.cpp` 第 96-275 行，共 5 个重载：`interpolateArmTraj` ×2、`interpolatePose2MobileArmTraj`、`interpolatePose2Traj`）。内部键值重新编号为 `x0..xN`。
6. **（可选）验算**：`CollisionCost*` 系列函数重放整条轨迹累加 hinge loss（`BatchTrajOptimizer-inl.h` 第 87-100 行，注意这里传 `epsilon = 0`，即纯“侵入深度”度量）。

#### 4.6.2 增量重规划器 `ISAM2TrajOptimizer`

`gpmp2/planner/ISAM2TrajOptimizer.h` 第 57-134 行的模板类，持有 `gtsam::ISAM2 isam_`，接口：

| 方法 | 作用（来源行） |
|---|---|
| `initFactorGraph(start_conf, start_vel, goal_conf, goal_vel)` | 建图，**缓存终点 prior 因子的下标**（`ISAM2TrajOptimizer-inl.h` 第 47-52 行） |
| `initValues(init_values)` | 存初始值 |
| `update()` | `isam_.update(inc_graph_, init_values_, removed_factor_index_)` + `calculateEstimate()`，然后清空增量缓存（第 100-115 行） |
| `changeGoalConfigAndVel(goal_conf, goal_vel)` | 把旧终点 prior 下标加入删除列表，插入新 prior（第 120-140 行） |
| `removeGoalConfigAndVel()` | 删除终点 prior（第 145-156 行） |
| `fixConfigAndVel(idx, conf, vel)` | 在执行后把某个状态钉死（第 161-169 行） |
| `addPoseEstimate / addStateEstimate` | 把执行后的状态估计（均值+协方差）作为 prior 重新入图（第 174-195 行） |

别名：`ISAM2TrajOptimizer2DArm`、`ISAM2TrajOptimizer3DArm`、`ISAM2TrajOptimizerPose2MobileArm2D/`、`ISAM2TrajOptimizerPose2MobileVetLin2Arms`（第 140-171 行）。

#### 4.6.3 参数与默认值（`TrajOptimizerSetting`）

结构体定义在 `gpmp2/planner/TrajOptimizerSetting.h` 第 17-100 行，默认值在 `TrajOptimizerSetting.cpp` 第 32-56 行。**下表的“默认值”取自代码，并与 `doc/Parameters.md` 的建议值对照：**

| 参数 | 代码默认 | 文档建议 / 说明 |
|---|---|---|
| `dof` | 必须显式给（构造函数参数） | 机器人自由度 |
| `total_step` | `10` | ≥10；建议 `total_step·(obs_check_inter+1) ≥ 50~100`（`Parameters.md` 第 42-45 行） |
| `total_time` | `1.0` s | 轨迹总时长 |
| `conf_prior_model` | `Isotropic::Sigma(dof, 0.0001)` | 起点/终点位姿先验，保持 1e-4 |
| `vel_prior_model` | `Isotropic::Sigma(dof, 0.0001)` | 起点/终点速度先验 |
| `flag_pos_limit` / `flag_vel_limit` | `false` / `false` | 是否启用限位 |
| `joint_pos_limits_up/down` | `±1e6·1` | 相当于不激活 |
| `vel_limits` | `1e6·1` | 同上 |
| `pos_limit_thresh` / `vel_limit_thresh` | `0.001·1` | hinge 缓冲带 |
| `pos_limit_model` / `vel_limit_model` | `Isotropic::Sigma(dof, 0.001)` | 限位代价权重 |
| `epsilon` | `0.2` | 2D/3D 通用安全距离，建议 0.05–0.2 m（`Parameters.md` 第 50-51 行） |
| `cost_sigma` | `0.1` | 2D 用 0.05–0.2，3D 用 0.005–0.02；**文档称这是唯一必须按机器人调参的量**（第 52-53 行） |
| `obs_check_inter` | `5` | 每两个支撑状态之间额外检查的插值点数量，0 = 不插值 |
| `Qc_model` | `noiseModel::Unit::Create(dof)` | “不知道就取单位阵”（第 54 行） |
| `opt_type` | `Dogleg` | 文档说 `LMA` 在多数情况下好用（第 55 行） |
| `final_iter_no_increase` | `true` | 保证最后一次迭代误差不增 |
| `rel_thresh` | **`1e-2`** | ⚠️ 文档说“默认 1e-6”（`Parameters.md` 第 56 行）——**代码与文档不一致** |
| `max_iter` | **`50`** | ⚠️ 文档说“默认 100”（`Parameters.md` 第 57 行）——**代码与文档不一致** |
| `opt_verbosity` | `None` | 可设 `Error` |

（`set_*` 系列 setter 见 `TrajOptimizerSetting.h` 第 63-99 行，主要供 MATLAB/Python wrapper 调用。）

---

## 5. 关键数据结构与内存

### 5.1 GTSAM `Values` 的键

- 位姿键：`Symbol('x', i)`；速度键：`Symbol('v', i)`，`i ∈ [0, total_step]`（`BatchTrajOptimizer-inl.h` 第 37-38 行；Python 侧 `symbol(ord("x"), i)`，`gpmp2_python/examples/pointRobot2FactorExample.py` 第 81-82 行）。
- 变量总数 `2(total_step+1)`。末端索引是 `total_step`（**不是** `total_step-1`）。
- `interpolateArmTraj` 输出会**重新编号**为从 0 开始的连续下标（`TrajUtils.cpp` 第 127-141 行），所以优化结果与加密结果的索引语义不同，跨用会串位。
- `TrajUtils.cpp` 第 109-112 行有一条注释 "TODO: gtsam keyvector has issue: free invalid pointer"，随后对 key 做了 `std::sort`；该排序依赖 GTSAM `Symbol` 的整型编码方式，是实现细节而非接口契约，跨 GTSAM 版本升级时需重新验证（**未在本环境编译验证**）。

### 5.2 轨迹参数化

- **支撑状态（support states）**：`total_step+1` 个，等间隔 `delta_t`。只有这些状态是优化变量。
- **GP 插值点**：不进入 `Values`，只在障碍 factor 内部**按需重算**（`ObstaclePlanarSDFFactorGP` 每次 `evaluateError` 都重新 `interpolatePose`，`-inl.h` 第 33-38 行）。因此 `obs_check_inter` 增大 = 因子数线性增加、变量数不变。这是 GPMP2 相对“直接把轨迹离散成很多点”的核心优势：**连续时间表达，检查密度与优化维度解耦**。
- 位置/速度状态类型随机器人不同：`Vector`（机械臂/点机器人）、`Pose2`（移动底盘）、`Pose2Vector`（移动机械臂）。

### 5.3 稀疏矩阵结构

- **Jacobian / Hessian**：链式图 → GTSAM 消元后的 Hessian 为**分块带状（block-tridiagonal 级别）**；每个 GP factor 只有 4 个变量、每个一元障碍 factor 只有 1 个变量。
- **FK 的临时内存**：`Arm::forwardKinematics` 带雅可比时分配 `dHo_dq`（`dof×dof` 个 `Matrix4`，即 `dof²×16` 个 double），`Arm.cpp` 第 85 行注释明确说 "(DOF^2 memory)"。对 7-DOF WAM 约 7.8 KB，不构成瓶颈；但若用于冗余机械臂需要留意。
- **SDF 数据**：2D 是单个稠密 `Matrix`（`rows×cols` double）；3D 是 `vector<Matrix>`，共 `rows×cols×z` double（`SignedDistanceField.h` 第 51 行）。SDF 是只读共享数据，factor 里以**引用**持有（`const PlanarSDF& sdf_`，`ObstaclePlanarSDFFactor.h` 第 48 行），不拷贝。
- **机器人模型**同样以 `const Robot&` 引用持有（第 45 行），因此 `BatchTrajOptimize` 的 robot 生命周期必须覆盖整次优化——这是 C++ 侧的常见踩坑点。
- **因子对象里的预计算量**：`GaussianProcessInterpolator*` 在构造时缓存 `Qc_, Lambda_, Psi_`（各 `2d×2d`），四元障碍 factor 每个实例都持有一份 `GPBase GPbase_` 拷贝（`ObstaclePlanarSDFFactorGP.h` 第 45 行），`obs_check_inter` 个 factor × `T` 段 → 内存与 `lambda/psi` 矩阵数成正比，但每个矩阵很小。

---

## 6. 三方集成细节

### 6.1 Python 绑定：Cython（不是 boost.python）

- `CMakeLists.txt` 第 78-91 行：`include(GtsamCythonWrap)`，然后

```cmake
wrap_and_install_library_cython("gpmp2.h"
                                "from gtsam.gtsam cimport *"
                                "${CMAKE_INSTALL_PREFIX}/cython"
                                gpmp2 "gtsam")
```

  并加编译宏 `-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES`（第 90 行）。
- 声明文件是根目录的 `gpmp2.h`。它顶部注释专门说明 Cython 的写法要求：**必须为 Cython 前置声明基类**（`virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;`）以便生成正确的继承关系，且不要重复声明基类里已有的重载（`gpmp2.h` 第 1-17 行）。随后按 `geometry / gp / kinematics / dynamics / obstacle / planner / utils` 分块 `#include` 具体头文件并声明可包装的类/函数（例如 `class Pose2Vector {...}`、`GaussianProcessPriorLinear`、各 `Obstacle*Factor*`、`PlanarSDF`、`TrajOptimizerSetting`、`BatchTrajOptimize*`、`ISAM2TrajOptimizer*`、`TrajUtils`）。
- 生成的 `.so` 与 `.pxd` 装到 `${CMAKE_INSTALL_PREFIX}/cython`，用户需 `export PYTHONPATH=/usr/local/cython`（`README.md` 第 72 行）。
- Python 侧包名 `gpmp2`（Cython 产出）+ `gpmp2_python`（纯 Python 工具，`gpmp2_python/gpmp2_python/__init__.py` 只含一行 `__version__` 相关导入）。
- 纯 Python 内容：
  - `datasets/generate2Ddataset.py`（190 行，数据集 `Empty` / `TwoObstaclesDataset` / `MultiObstacleDataset` / `MobileMap1`，第 111-159 行）、`datasets/generate3Ddataset.py`（`WAMDeskDataset`，第 92 行）；
  - `robots/generateArm.py`（`SimpleTwoLinksArm` / `SimpleThreeLinksArm` / `WAMArm` / `SAWYERArm` / `PR2Arm` / `JACO2Arm` 共 6 种，第 22-236 行的分支，每个都带球体表 `[link_id x y z r]`）、`robots/generateMobileArm.py`（343 行）；
  - `utils/plot_utils.py`（445 行）、`utils/signedDistanceField2D.py` / `signedDistanceField3D.py`。
- 示例（`gpmp2_python/examples/`）：`pointRobot2FactorExample.py`、`pointRobot2Factor_rh.py`、`pointRobot3FactorExample.py`、`pointRobot3FactorExample_rh.py`、`Arm2FactorGraphExample.py`、`WAMFactorGraphExample.py`、`multi_graph/graph_pointRobot.py` + `graph_utils.py`（537 行，因子图/求解的辅助封装）。示例里同时演示了“手工建图 + `DoglegOptimizer`”和“调用 `BatchTrajOptimize*`”两种用法。

### 6.2 MATLAB mex

- 通过 GTSAM 的 `GtsamMatlabWrap` 自动生成 `.mex` 与 `.m`（`CMakeLists.txt` 第 67-74 行；`doc/CplusplusDevelopment.md` 第 90-99 行）。
- 辅助脚本在 `matlab/+gpmp2/`，每个示例开头 `import gtsam.*; import gpmp2.*;`。
- 14 个示例各自演示什么（读文件头注释）：

| 文件 | 演示内容 |
|---|---|
| `PointRobot2DFactorGraphExample.m` | 2D 点机器人手工建因子图 |
| `Arm2FactorGraphExample.m` / `Arm3FactorGraphExample.m` | 2/3 连杆平面臂避障，手工建图 |
| `Arm3PlannerExample.m` | 3 连杆 + 集成规划器 `BatchTrajOptimize2DArm`，含 GP 插值加密绘图（`doc/ExampleMatlab2D.md`） |
| `Arm3GoalReachExample.m` | 带末端到达目标的避障 |
| `Arm3JointLimitExample.m` | 关节限位 factor |
| `WAMFactorGraphExample.m` / `WAMPlannerExample.m` | 7-DOF WAM，手工建图 / 集成规划器（3D） |
| `WAMReplannerExample.m` | WAM 增量重规划（`ISAM2TrajOptimizer`），见 `doc/ExampleReplanning.md` |
| `WAMWorkspaceConstraintsExample.m` | 末端姿态保持水平 + 末端位姿目标（workspace 约束） |
| `MobileBaseFactorGraphExample.m` / `MobileArm2FactorGraphExample.m` / `Mobile2ArmsFactorGraphExample.m` | SE(2) 底盘 / 底盘+单臂 / 底盘+双臂 |
| `SaveSDFExample.m` | 把 SDF 存成 gpmp2 datatype（配合 `SignedDistanceField::saveSDF`） |

### 6.3 `projects/jist`

- 论文：*Joint Sampling and Trajectory Optimization Over Graphs for Online Motion Planning*, IROS 2021（`projects/jist/README.md` 第 3 行）。
- 依赖：GTSAM + GPMP2、`hydra-core==0.11`、`opencv-python==4.2.0.32`（第 10-12 行）。
- 两个场景：Random Forest（随机障碍林）与 Patrol Guard（巡逻守卫，动态目标），用 `python pointRobot_example.py problem.node_budget=... problem.seed_val=...` 运行（第 20-25 行）；`pointRobot_test.py` / `pointRobot_test_patrol_guard.py` 用于复现论文 Table 1（第 33-39 行）。
- 实现上 `jist/jist.py`（1098 行）在 GPMP2 之上做了“采样生成节点 → 因子图优化 → 交替”的外层调度（**具体策略未逐行通读，此处不作强断言**）。

---

## 7. 测试与验证

- 测试框架：GTSAM 自带的 **CppUnitLite**（`#include <CppUnitLite/TestHarness.h>`，见各 `tests/*.cpp`）。
- 注册：每个模块 `CMakeLists.txt` 一行 `gtsamAddTestsGlob(<module> "tests/*.cpp" "" ${PROJECT_NAME})`；根 `CMakeLists.txt` 第 2 行 `enable_testing()`。跑测试：`make check`（`README.md` 第 42 行）。
- 测试清单（`find gpmp2 -path '*/tests/*.cpp'` 共 **37** 个）：

| 模块 | 测试文件 |
|---|---|
| `gp` | `testGaussianProcessPrior{Linear,Pose2,Pose2Vector,Pose3}.cpp`、`testGaussianProcessInterpolator{Linear,Pose2,Pose2Vector,Pose3}.cpp` |
| `kinematics` | `testArm`、`testArmModel`、`testPointRobotModel`、`testMobileBaseUtils`、`testPose2Mobile{Base,Arm,2Arms,VetLinArm,VetLin2Arms}`、`testGaussianPriorWorkspace{Position,Orientation,Pose}`、`testGoalFactorArm`、`testJointLimitFactor{Vector,Pose2Vector}` |
| `obstacle` | `testPlanarSDF`、`testSignedDistanceField`、`testObstacleSDFFactor{Arm,GPArm}`、`testObstaclePlanarSDFFactor{Arm,GPArm}`、`testSelfCollision` |
| `planner` | `testTrajUtils`、`testISAM2TrajOptimizer` |
| `geometry` | `testDynamicVector`、`testPose2Vector`、`testProductDynamicLieGroup` |
| `dynamics` | `testVehicleDynamics` |
| `utils` | `testTimer` |

- 验证方法（以实际读到的断言为准）：
  1. **解析雅可比 vs 数值差分**：`testGaussianProcessPriorLinear.cpp` 用 `numericalDerivative11` 对四个自变量逐一差分，与 `evaluateError` 输出的 `H1..H4` 比较（第 50-66 行等），容差 1e-6。障碍 factor 同样处理（`testObstaclePlanarSDFFactorArm.cpp` 第 94-97 行）。
  2. **代价函数正确性**：障碍测试手写 `convertSDFtoErr()`，把 SDF 值按 `err = max(0, eps − d)` 转成期望误差再比对（`testObstaclePlanarSDFFactorArm.cpp` 第 28-31、91-96 行）——即**测试本身就复述了 hinge loss 的数学定义**。
  3. **端到端小规模优化**：`testGaussianProcessPriorLinear.cpp` 第 144-202 行搭“两个位姿 prior + 一个 GP factor”的图，用 `GaussNewtonOptimizer` 优化，断言最终误差为 0、速度被还原为真值。这验证了“GP factor 确实在解释速度”。
  4. **构造/集成 smoke test**：`testISAM2TrajOptimizer.cpp` 只做构造（第 24-67 行），没有断言重规划收敛性。
  5. **碰撞验证**：仓库层面暴露了 `CollisionCost2DArm/3DArm/Pose2Mobile*`（`BatchTrajOptimizer.h` 第 135-200 行），语义是“把整条轨迹的 hinge loss 累加”（`epsilon` 传 0）；示例（`matlab/Arm3PlannerExample.m`、Python 示例）靠可视化人工判断。
- **缺失的验证**：仓库内没有“规划结果必然无碰撞”的自动化断言、没有最优性 gap 检查、没有基准数据集文件（数据集都是代码程序化生成的），也没有性能/复杂度基准脚本（`utils/Timer.h` 只提供计时工具，`testTimer.cpp` 只测计时器本身）。

---

## 8. 局限与坑

**版本与依赖**

1. README 明确本仓库只兼容 GTSAM **`wrap-export`** 历史分支（`README.md` 第 1、15、25 行）；升级到现代 GTSAM 需换 `borglab/gpmp2`。这意味着 `find_package(GTSAM REQUIRED)` 在现代 GTSAM 上大概率直接失败（**未在本环境尝试编译验证**，仅依据 README 声明）。
2. **Python 工具箱是 Python 2.7**（`README.md` 第 6、51 行）。而且 `generate2Ddataset.py` 等文件里用了 `dataset_str is "..."` 的字符串身份比较（第 111/124/138/159 行）与裸 `print`，在 Python 3 下行为不正确（**未运行验证，为静态阅读结论**）。
3. CMake 把 `/usr/local/cython` 硬编码进 include 路径（`CMakeLists.txt` 第 80 行），非默认安装前缀需手改。
4. 静态库与 MATLAB toolbox 互斥（`CMakeLists.txt` 第 24-26 行），Python toolbox 同样依赖 shared lib。
5. 全局 `-w` 关告警（`CMakeLists.txt` 第 5 行），隐藏了潜在的未使用变量/隐式转换问题。

**算法层面**

6. **局部最优 & 对初值敏感**：全程是梯度类非线性最小二乘，初值默认是配置空间直线（`TrajUtils.cpp` 第 25-50 行）。窄通道、直线穿墙等场景会收敛到局部解；仓库没有全局规划器兜底。`projects/jist` 正是为了补这个短板而引入采样的。
7. **hinge loss 的“近视”**：`d(p) > eps` 时代价与梯度**恒为 0**（`ObstacleCost.h` 第 41-43 行）。远离障碍的支撑状态对障碍一无所知，只有把状态密度提高到“总有一段落在 eps 带内”才起作用——这正是 `Parameters.md` 第 42-45 行强制要求 `total_step·(obs_check_inter+1) ≥ 50~100` 的原因。
8. **SDF 越界静默失效**：越界时 `hingeLossObstacleCost` 返回 0 代价与 0 梯度，且 warning 被注释掉（`ObstacleCost.h` 第 33-38、61-66 行）。若机器人运动出 SDF 覆盖范围，会“以为没障碍”。使用前必须保证 SDF 外扩足够大。
9. **bilinear/trilinear 插值在单元边界不可导**（`PlanarSDF.h` 第 103-104 行注释；`SignedDistanceField.h` 第 144-145 行同理），梯度在格子界面跳变，可能造成迭代抖动。
10. **超参敏感**：`cost_sigma` 是平滑性 vs 避障的唯一连续调节钮（`Parameters.md` 第 52-53 行）；`Qc` 直接决定先验松紧，文档只说“不知道就用单位阵”；`epsilon` 要按机器人尺度选（0.05–0.2 m）。三者耦合，换机器人基本都要重调。
11. **文档与代码默认值不一致**：`rel_thresh` 代码 1e-2 vs 文档 1e-6；`max_iter` 代码 50 vs 文档 100（`TrajOptimizerSetting.cpp` 第 27-28 行 vs `Parameters.md` 第 56-57 行）。按文档调参会得到与预期不同的停止行为。
12. **`final_iter_no_increase` 的语义代价**：误差上升时直接丢弃本轮、返回上一轮值（`BatchTrajOptimizer.cpp` 第 296-307 行），相当于“最多做 max_iter 次尝试但不一定单调”，收敛速率可能变慢。
13. **每次 `evaluateError` 都重算 FK + SDF**：GP 障碍 factor 每次都重新插值 + 正运动学（`ObstaclePlanarSDFFactorGP-inl.h` 第 36、45 行），没有跨迭代缓存，是主要计算热点；`obs_check_inter` 是线性放大器。
14. **数值噪声**：`calcQ_inv` 含 `τ⁻³`、`τ⁻²`，`Δt` 很小时条件数差（`GPutils.h` 第 36-38 行）；`TrajUtils.cpp` 第 108 行留有 “gtsam keyvector has issue: free invalid pointer” 的 TODO。
15. **`Arm::base_pose_` 是 `mutable` 且 `updateBasePose()` 是 `const` 方法**（`Arm.h` 第 34、80 行）——同名对象在 const 语境里被改状态，多线程/复用模型时容易出隐蔽 bug。
16. **因子持有 `const Robot&` / `const SDF&` 引用**（`ObstaclePlanarSDFFactor.h` 第 45-48 行）：被引对象必须活得比 factor graph 长；临时对象入图是悬垂引用风险。
17. **无碰撞保证**：优化只最小化软代价，最终轨迹是否无碰撞需要外部用 `CollisionCost*` 或独立碰撞检测确认（`BatchTrajOptimizer.h` 第 135-200 行）。

---

## 9. 对用户项目的可借鉴点

> 目标场景：Python + NumPy 确定性仿真内核 → CPU/FPGA 异构 → 真实二维小车；需要**轨迹预测 / 平滑规划 / 优化式避障**。以下按“可直接借鉴 → NumPy 简化实现建议”组织。

### 9.1 可直接借鉴的四个思想

**(1) 用 LTV-SDE 先验做“未来轨迹平滑预测”**

GPMP2 的价值不只在规划，还在**连续时间重建**：给定若干带噪的支撑状态，`Λ(τ), Ψ(τ)` 给出区间内任意时刻的最优插值/去噪（`gpmp2/gp/GPutils.h` 第 48-59 行）。对你的预测模块：

- 把历史观测作为因子图的“观测 prior”，用 GP 先验连接相邻状态，求解后**在任意 `τ` 处查询**得到平滑、可导、速度一致的未来轨迹——输出天然包含 `θ̇`，正好喂给底层控制器。
- 与卡尔曼滤波的区别：这是**批式/滑窗的连续时间平滑**，不受固定步长离散化限制，且天然支持不等间隔观测（`delta_t` 可逐段不同）。对 FPGA 友好之处：`Λ, Ψ, Q` 只依赖 `(Qc, Δt, τ)`，可以**离线预算成查找表**。

**(2) 把避障写成带 hinge loss 的软约束**

这是 GPMP2 最“便宜”也最稳的贡献（`gpmp2/obstacle/ObstacleCost.h`）：

```
cost(p) = max(0, eps − d(p))²,   ∂cost/∂p = −∇d(p)  当 d(p) < eps，否则 0
```

- 优点：一次可导、无约束求解器即可处理、`eps` 直接就是“物理安全距离”（还可以把小车半径直接加进 `eps`，正如 `total_eps = sphere_radius + epsilon_`）。
- 对你的二维小车：把小车包成 1~3 个圆（`BodySphere` 的思路），每个圆对每个时刻出一个残差 —— 与 GPMP2 的 `nr_body_spheres` 完全同构。
- 若用“圆形障碍”而非网格 SDF，`d(p)` 与 `∇d` 有闭式解（`‖p−c‖ − r`），比双线性插值更快更平滑，**更适合 FPGA 定点化**；这也绕开了 §8 第 9 条的格子不可导问题。

**(3) 用因子图把“平滑 + 避障 + 动力学 + 限位”联合优化**

`BatchTrajOptimizer-inl.h` 第 36-81 行就是一个可直接抄的“多目标拼装模板”：每个时间步叠加“一元代价 + 相邻四元先验 + 可选限位”，每项都有独立权重（`Qc` 控平滑、`cost_sigma` 控避障、`pos/vel_limit_model` 控限位、`conf_prior_model` 控端点）。**多目标的权衡全在噪声模型的 sigma 里，不需要手工设计加权和系数的相对尺度**——这正是概率推断写法相对传统优化写法的可维护性优势。

**(4) 增量重规划（iSAM2）而非每帧从头解**

`ISAM2TrajOptimizer` 通过缓存终点 prior 的因子下标、支持换目标/钉状态/注入估计（`gpmp2/planner/ISAM2TrajOptimizer-inl.h` 第 120-195 行），把“每帧重解”变成“增量更新 + 局部重线性化”。对动态障碍环境，你的重规划频率如果高于 10 Hz，这个思路比每帧重跑一遍 batch 省一个数量级。

### 9.2 建议先跳过 / 需要改造的部分

| GPMP2 的做法 | 对你的场景的问题 | 建议 |
|---|---|---|
| 静态 SDF 网格（`PlanarSDF`） | 你是**动态**障碍环境，SDF 逐帧变化 | 时间索引化：`sdf[t]` 或解析圆形障碍 `(c(t), r(t))`；把障碍运动写进因子的期望位置（等价于“时变 SDF”）。若要保留 GP 先验的连续时间优势，可用一阶预测 `c(t) = c(t_k) + v·(t−t_k)` 在因子内部求值 |
| 李群/`Pose2Vector`/DH 全套 | 二维小车状态就是 `(x, y, θ)`，用不上 DH 与 SE(3) | 只用 `Vector`/`Pose2` 两种状态；`calcQ/calcPhi` 是纯矩阵运算（§4.1），与李群无关，可直接移植 |
| 依赖 GTSAM | 你要 NumPy 确定性内核 + FPGA | 自己写 Gauss-Newton + 稀疏 Cholesky（见 §9.3） |
| 仅 hinge loss 的软避障 | 软约束不保证无碰撞 | 加一层**硬约束投影/后验检查**（`CollisionCost*` 的做法），或对关键状态加硬性 barrier |
| Matérn 核 | 仓库里根本没有（§4.1） | 若确实要 Matérn，自己构造 `K(t,t')` 并求逆；否则直接用 LTV-SDE 先验更方便 |

### 9.3 NumPy 简化实现的接口建议

建议在你的仿真内核里按下面的**并行结构**落地（与 GPMP2 一一对应，便于日后对照论文与 C++ 实现）：

```python
# ---- gp.py：与 gpmp2/gp/GPutils.h 一一对应，纯函数、可微、无状态 ----
def calc_phi(dof: int, tau: float) -> np.ndarray: ...            # (2d,2d)
def calc_q(Qc: np.ndarray, tau: float) -> np.ndarray: ...        # (2d,2d)
def calc_q_inv(Qc: np.ndarray, tau: float) -> np.ndarray: ...    # (2d,2d) 解析闭式，勿求逆
def calc_lambda(Qc, delta_t, tau) -> np.ndarray: ...             # (2d,2d)
def calc_psi(Qc, delta_t, tau) -> np.ndarray: ...                # (2d,2d)

class GPInterpolator:                    # 对应 GaussianProcessInterpolatorLinear
    def __init__(self, Qc, delta_t, tau): ...    # 预算并缓存 Lambda, Psi
    def pose(self, x_i, x_ip1): ...              # 返回 (theta_tau,)
    def vel(self, x_i, x_ip1): ...               # 返回 (theta_dot_tau,)
    def pose_jac(self, x_i, x_ip1): ...          # 返回 (d, 2d, 2d) 供链式法则用

# ---- factors.py：每个 factor = (残差函数, 解析雅可比, 噪声 sigma) ----
class GPriorFactor:        # 4 元：x_i, v_i, x_ip1, v_ip1 -> 残差 2d
class ObstacleCostFactor:  # 1 元：x_i -> 残差 nr_circles（hinge loss）
class ObstacleCostGPFactor:# 4 元：先插值再求 hinge loss（对应 *FactorGP）
class JointLimitFactor:    # 1 元：铰接/速度限位
class DynamicsFactor:      # 2 元：侧滑速度 = 0（对应 VehicleDynamicsFactorVector）

# ---- sdf.py ----
class PlanarSDF:           # 与 gpmp2/obstacle/PlanarSDF.h 同构
    def signed_distance(self, p) -> float: ...
    def gradient(self, p) -> np.ndarray: ...      # 返回 (2,)

# ---- solver.py：确定性、无随机、固定迭代次数 ----
def build_problem(robot, sdf, start, goal, setting) -> (residual_fns, x0): ...
def gauss_newton(residual_fns, x0, max_iter, rel_tol) -> np.ndarray:
    """稀疏 GN：J 由各 factor 的解析雅可比拼装；H = J^T W J 分块三对角；
    用 scipy.sparse + splu 或自写分块 Thomas 算法求解。"""
```

落地顺序建议：

1. **第 1 步（半天）**：只实现 `calc_phi/q/q_inv/lambda/psi` + `GPInterpolator`，写单测对齐 GPMP2 的公式（`Λ(0)=I, Ψ(0)=0; Λ(Δt)=0, Ψ(Δt)=I`；`Λ+Ψ=I`；`Λ(τ)` 的位姿部分应给出 `θ_i + τθ̇_i` 的均值行为）。这层与机器人无关，是最高复用的资产。
2. **第 2 步**：实现圆形障碍的 hinge loss + 解析梯度，做一个“两点 + 1 障碍”的最小 GN，验证：无障碍时解退化为直线；有障碍时轨迹绕开且 `eps` 增大则更远离。
3. **第 3 步**：拼装完整的链式因子图（`GP factor × T` + `ObstacleFactor × (T+1)` + `ObstacleGPFactor × T·obs_inter` + 端点 prior），用稀疏 GN 求解，拿 GPMP2 的 `parameters`（`epsilon=0.2`, `cost_sigma` 2D 取 0.05–0.2, `obs_check_inter` 使 `total_step·(obs_check_inter+1) ≥ 50`）做初值。
4. **第 4 步（面向 FPGA）**：
   - 把 `Λ, Ψ, Φ, Q⁻¹` 按 `(Δt, τ)` 离线求好，运行期只做**定点矩阵乘**；
   - 用**固定迭代次数**的 GN 替代 LM/Dogleg（后者的自适应 lambda/信任域是数据相关的分支，FPGA 上时序不友好）——正好 GPMP2 也提供 `GaussNewton` 选项（`TrajOptimizerSetting.h` 第 20 行）；
   - Hessian 是**分块三对角**（§5.3），用分块 Thomas/前向消元实现 `O(T·d³)` 且内存 `O(T·d²)`，非常规则；
   - `hinge loss` 与 `max(0, ·)` 可以直接映射为比较+选择，代价低；但注意 `∇d` 需要除法和开方（圆形障碍）——可用牛顿迭代或查表。
5. **第 5 步（动态障碍）**：先做最简单可靠的方案——**每帧用上一帧解热启动 + 有限次 GN 迭代**（对应 iSAM2 的“增量”思想，但实现代价低得多）；等确定瓶颈后再考虑真正的增量消元。

### 9.4 一句话结论

GPMP2 对你最有价值的不是它的 C++/GTSAM 基础设施（那部分与你的 Python+NumPy+FPGA 路线并不匹配），而是**三个数学组件**：①LTV-SDE 导出的 `Λ/Ψ/Q/Q⁻¹` 闭式解（平滑预测 + 连续时间插值）；②hinge loss + 解析 SDF 梯度的软避障；③链式因子图把多目标统一成“加权最小二乘”的拼装范式。这三者都可以在几百行 NumPy 内复现，且结构规则、确定性强，适合作为 FPGA 定点化的起点。
