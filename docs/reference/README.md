# 参考项目技术总结（Reference Project Notes）

本目录是对 `Outside/` 下克隆的 9 个第三方参考项目的**逐项目技术拆解**，每篇精确到
算法、模块、数据结构与实现文件（含 `文件:行号` 取证）。

> **这些笔记是本项目作者自己写的分析文档**，用于记录"哪些技术可以借鉴、怎么借鉴、
> 哪些坑别踩"。上游代码**不入库**：`Outside/` 已在 `.gitignore` 中整体忽略
> （约 1.4 GB，且这些克隆自身多为 git 仓库，git 无法只跟踪其中的单个文件）。
> 因此这里保留**可提交的镜像**，并在每个项目目录里放一份 `tech.md` 相对符号链接，
> 便于对着代码阅读。

## 项目清单

| 分类 | 项目 | 行数 | 上游仓库 | 克隆版本 | 许可证 | 一句话 |
|---|---|---|---|---|---|---|
| Algo | [PythonRobotics](./PythonRobotics.md) | 814 | [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | `master@cdd0cc8` | MIT | 机器人算法教学合集：10 个域、102 个算法（规划/跟踪/定位/建图/SLAM/机械臂/空中/双足） |
| Algo | [SIPP-IP](./SIPP-IP.md) | 593 | [PathPlanning/SIPP-IP](https://github.com/PathPlanning/SIPP-IP) | `main@77ca749` | MIT | 带运动学约束的安全区间路径规划（A\* / SIPP1 / SIPP2 / SIPP-IP） |
| Algo | [gpmp2](./gpmp2.md) | 744 | [gtrll/gpmp2](https://github.com/gtrll/gpmp2) | `main@4289d9a` | BSD-3-Clause（Georgia Tech） | 用高斯过程 + 因子图把运动规划写成概率推断（RSS 2016） |
| Algo | [Voronoi 路径规划](./Voronoi-PathPlanning.md) | 480 | [SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram](https://github.com/SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram) | `master@c009820` | MIT | 基于 Voronoi 图（路线图法）的二维静态避障规划 |
| Bullet | [Keiki-master](./Keiki-master.md) | 602 | 本地副本（无 `.git`） | — | **无 LICENSE** | Python 弹幕游戏 + 用 GAN（DCGAN / PSGAN / TimeGAN）生成弹幕 |
| Bullet | [TostEngine](./TostEngine.md) | 470 | 本地副本（无 `.git`） | — | README 自称 MIT 但**无正文** | C++/Cinder 弹幕引擎：实体池、空间网格碰撞、六种弹幕 |
| Camera | [optical-flow-fpga](./optical-flow-fpga.md) | 1051 | [rothej/optical-flow-fpga](https://github.com/rothej/optical-flow-fpga) | `main@cd64c81` | MIT（测试图为 CC BY-SA 3.0） | Nexys A7 上的 Lucas-Kanade 光流 |
| Camera | [Image-Processing-Toolbox](./Image-Processing-Toolbox.md) | 559 | [Gowtham1729/Image-Processing-Toolbox](https://github.com/Gowtham1729/Image-Processing-Toolbox) | `master@e14a522` | Apache-2.0 | Basys3/VGA 图像处理工具箱：BRAM 帧缓冲、COE 初始化、Python 参考脚本 |
| Camera | [Red_Tracker](./Red_Tracker.md) | 729 | [delhatch/Red_Tracker](https://github.com/delhatch/Red_Tracker) | `master@bb3fd58` | **无 LICENSE** | DE2-115 + D8M 摄像头：SDRAM 帧缓冲、红色阈值分割、质心跟踪 |

## 结论速览（最值得借鉴 / 最大的坑）

| 项目 | 最值得借鉴 | 最大的坑（已验证） |
|---|---|---|
| **PythonRobotics** | SIPP 实测把扩展数从 2,717,154 降到 322；Frenet 代价 + 笛卡尔↔Frenet 纯函数；DWA 把静态障碍换成"按预测时刻采样"即动态化；GPS/IMU 8 维 EKF（含零偏、Joseph 形式）；**用有限差分校验雅可比**的测试模板 | 91 个测试文件里 56 个是**零断言冒烟测试**；多数循环无迭代上限；**没有 CBS/ECBS**、**没有动态障碍轨迹预测**（只有 `TimeBasedPathPlanning`）；20+ 处实现瑕疵 |
| **SIPP-IP** | **安全区间**表示（替代我们的全帧交集）、**Interval Projection** 处理非瞬时加/减速、时间量化成整数 tick 以保证 CPU/FPGA 逐位一致 | 逐字节 diff 显示 SIPP1/SIPP2/SIPP-IP 真实差异**各只有 3 行**（都在 CLOSED 表处理）；`CLOSED_vec[0]` 失败时越界读；README 未给论文引用 |
| **gpmp2** | **LTV-SDE** 做连续时间平滑轨迹预测、**hinge loss** 软避障、因子图拼装多目标（平滑+避障+动力学）、iSAM2 增量重规划 | 仓库里**没有 Matérn 核**（GP 先验是 LTV-SDE 闭式离散化）；文档写 `rel_thresh=1e-6/max_iter=100` 而代码是 `1e-2/50`；Python 工具箱是 **2.7**；SDF 越界**静默返回 0 代价** |
| **Voronoi-PathPlanning** | 骨架判据改用 NumPy 欧氏距离变换（O(W·H·M)→O(W·H)）；把 clearance 存进 SoA 做**隐式膨胀**；junction 分层思路 | 搜索是贪心（**既非 Dijkstra 也非 A\***）、**完全没有障碍膨胀/机器人半径**、全项目仅 1 次 `delete`、库层 `exit(-1)` |
| **Keiki-master** | 解析式几何展开器（circle/sector/string，string 用单 span 参数在直线↔圆弧连续插值）；`aim_lag_frames` 而非运行期读实时玩家（保可复现）；TimeGAN 三段式结构 | **TimeGAN 判别器损失从未 `backward()`**（D 只被 weight_decay 收缩）；PSGAN 防崩溃的 `MeanVarLoss` 定义了却从未启用；**全仓库零随机种子、零测试**；生成→游戏是纯手工搬运 |
| **TostEngine** | 几乎没有正面可借鉴项；价值在**反面教材**（变步长主循环、AoS+first-fit 池、immediate-mode 渲染、AoS 里每颗弹背着 `PlayerData`） | README 与代码不符：`angular`/`rotation` 是**死字段**（没有曲线弹）、声称 fixed timestep 实为**变步长+0.25s 钳制**；**只有圆-圆碰撞、无障碍物概念**；594 MB 里自有代码仅 4 文件 28 KB |
| **optical-flow-fpga** | **S8.7 定点格式**、4 整行行缓冲 + 5 级移位窗、Sobel÷8、组合 32 位除法、±8 px clamp 的位宽设计；Python golden model 与硬件 1:1 可映射 | **金字塔 FSM 死锁**（`start` 需 `build_done`，builder 需 `start`）；提交的 Vivado 报告**无法由当前 RTL 复现**（比 commit 早 45 分钟、资源数字自相矛盾）；TB **只查幅度不查符号**（流向相反也 PASS）；`rtl/opt`、`prj/opt` 不存在 |
| **Image-Processing-Toolbox** | COE 生成器可当通用 ROM 初始化器；**定点符号判定技巧**（`>1024` 等价"是负数"）；确认 640×480×8bit 整帧 = 2.46 Mbit **超过 Basys3 1.8 Mbit → 必须用行缓冲** | **没有行缓冲**（改用 96-bit BRAM 字一次宽读整窗，代价 ≈96% BRAM）；`blur.v` IOB 233% **只能仿真不能上板**；高斯权重和 17 却除以 16（DC 增益 1.0625）；VGA 刷新率实际 ≈119 Hz 而 README 写 60 Hz |
| **Red_Tracker** | 阈值+游程中点可作 golden model（已给逐位对齐的 NumPy 实现）；`Sdram_Control` 的 WR2/RD2 端口可恢复给 CPU 用；跨时钟域用 toggle 握手 + 双触发器而非推整帧过 FIFO | 时序**不收敛**（根因是 `.sdc` 缺 `set_clock_groups -asynchronous`，非逻辑慢）；`MIPI_PIXEL_CLK` 没有 `create_clock`；算法**不是质心**而是逐行最长游程中点；`RAW2RGB_J` 把 R/B **交叉接线**；两处 build blocker；掩膜滞后一整帧 |

## 许可证风险一览

| 项目 | 状态 | 处理 |
|---|---|---|
| PythonRobotics / SIPP-IP / optical-flow-fpga / Voronoi-PathPlanning | MIT | 可参考，需标注来源 |
| gpmp2 | BSD-3-Clause | 同上，保留版权声明 |
| Image-Processing-Toolbox | Apache-2.0 | 同上，注意 NOTICE 与专利条款 |
| **Keiki-master** | **无 LICENSE 文件** | 仅作公开技术思路参考，**不得复制/派生/打包其代码与数据** |
| **TostEngine** | README 一行 "MIT License"，**无正文文件** | 同上；若要复用源码须先补齐许可证 |
| **Red_Tracker** | **无 LICENSE，README 未声明** | 同上 |

结论：本项目的实现**不依赖上述任何项目的源码**，仅参考公开的技术思路；
许可证未决不影响本项目自身可用性，但会阻止未来无意中的直接复用。

## 引用核验

每篇文档都经过机器核验：脚本抽取文中全部 `文件:行号` 与路径引用，按"参考项目根"
与"本仓库根"两个基准解析并检查存在性。

* 带行号的引用：**9 篇共 159 条，0 条无法解析**；
* 去除花括号/斜杠简写后逐文件名核对：**未发现任何编造的文件名**——少数"查无此名"
  的条目全部属于三种情况：①简写记号（`{a,b}.h`、`Arm.h/.cpp`）；②**文档自己明确
  标注为"仓库中不存在"**（如 optical-flow-fpga 的 `generate_rtl_testvectors.py`、
  Red_Tracker 的 `MIPI_B_I2C.v`）；③命令而非文件名。

## 相关文档

* `docs/算法开源项目.md` — 计划评估的 48 个路径规划 / 运动规划开源项目清单（本目录覆盖其中 4 个）
* `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md` — 本项目与 Keiki / TostEngine 的逐条设计对比与合规动作
* 仓库根 `README.md` 第 21 节 — 开源参考与许可证标记
