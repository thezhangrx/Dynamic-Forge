# 参考项目技术总结（Reference Project Notes）

本目录是对 `Outside/` 下克隆的第三方参考项目的**逐项目技术拆解**，每个项目一份，
精确到算法、模块、数据结构与实现文件。

> **这些笔记是本项目作者自己写的分析文档**，用于记录"哪些技术可以借鉴、怎么借鉴"。
> 上游代码**不入库**：`Outside/` 已在 `.gitignore` 中整体忽略（约 1.4 GB，且这些克隆
> 自身是 git 仓库，git 无法只跟踪其中的单个文件）。因此这里保留一份**可提交的镜像**，
> 同时在每个项目目录里放一份 `tech.md`（指向本目录对应文件的相对符号链接），便于对照代码阅读。

## 项目清单

| 分类 | 项目 | 上游仓库 | 克隆版本 | 许可证 | 一句话 |
|---|---|---|---|---|---|
| Algo | [PythonRobotics](./PythonRobotics.md) | [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | `master@cdd0cc8` | MIT | 机器人算法教学合集：路径规划 / 轨迹跟踪 / 定位 / 建图 / SLAM / 机械臂 / 空中 / 双足 |
| Algo | [SIPP-IP](./SIPP-IP.md) | [PathPlanning/SIPP-IP](https://github.com/PathPlanning/SIPP-IP) | `main@77ca749` | MIT | 带运动学约束的安全区间路径规划（A\*/SIPP1/SIPP2/SIPP-IP） |
| Algo | [gpmp2](./gpmp2.md) | [gtrll/gpmp2](https://github.com/gtrll/gpmp2) | `main@4289d9a` | BSD-3-Clause（Georgia Tech） | 用高斯过程 + 因子图把运动规划写成概率推断（RSS 2016） |
| Algo | [Voronoi 路径规划](./Voronoi-PathPlanning.md) | [SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram](https://github.com/SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram) | `master@c009820` | MIT | 基于 Voronoi 图（路线图法）的二维静态避障规划 |
| Bullet | [Keiki-master](./Keiki-master.md) | 本地副本（无 `.git`） | — | **无 LICENSE** `[LICENSE_REVIEW_REQUIRED]` | Python 弹幕游戏 + 用 GAN（DCGAN/PeriodicSpatialGAN/TimeGAN）生成弹幕 |
| Bullet | [TostEngine](./TostEngine.md) | 本地副本（无 `.git`） | — | README 自称 MIT，但**无许可证正文** `[LICENSE_REVIEW_REQUIRED]` | C++/Cinder 弹幕引擎：实体池、空间网格碰撞、固定步长、六种弹幕 |
| Camera | [optical-flow-fpga](./optical-flow-fpga.md) | [rothej/optical-flow-fpga](https://github.com/rothej/optical-flow-fpga) | `main@cd64c81` | MIT | Nexys A7 上的实时 Lucas-Kanade 光流，含 unopt/opt 对比与 Python 参考模型 |
| Camera | [Image-Processing-Toolbox](./Image-Processing-Toolbox.md) | [Gowtham1729/Image-Processing-Toolbox](https://github.com/Gowtham1729/Image-Processing-Toolbox) | `master@e14a522` | Apache-2.0 | Basys3/VGA 图像处理工具箱：BRAM 帧缓冲、COE 初始化、Python 参考脚本 |
| Camera | [Red_Tracker](./Red_Tracker.md) | [delhatch/Red_Tracker](https://github.com/delhatch/Red_Tracker) | `master@bb3fd58` | **无 LICENSE（README 也未声明）** `[LICENSE_REVIEW_REQUIRED]` | DE2-115 + D8M 摄像头：SDRAM 帧缓冲、红色阈值分割、最大块质心跟踪与十字准星叠加 |

## 许可证风险一览

| 项目 | 状态 | 处理 |
|---|---|---|
| PythonRobotics / SIPP-IP / optical-flow-fpga | MIT | 可参考，仍需标注来源 |
| Voronoi-PathPlanning | MIT | 同上 |
| gpmp2 | BSD-3-Clause | 同上，注意保留版权声明 |
| Image-Processing-Toolbox | Apache-2.0 | 同上，注意 NOTICE 与专利条款 |
| **Keiki-master** | **无 LICENSE 文件** | 仅作公开技术思路参考，**不得复制/派生/打包其代码与数据** |
| **TostEngine** | README 一行 "MIT License"，**无正文文件** | 同上；若要复用源码须先补齐许可证 |
| **Red_Tracker** | **无 LICENSE，README 未声明** | 同上 |

结论：本项目的实现**不依赖上述任何项目的源码**，仅参考公开的技术思路；
许可证未决不影响本项目自身可用性，但会阻止未来无意中的直接复用。

## 相关文档

* `docs/算法开源项目.md` — 计划评估的 48 个路径规划 / 运动规划开源项目清单（本目录目前覆盖其中 4 个）
* `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md` — 本项目与 Keiki / TostEngine 的逐条设计对比与合规动作
* 仓库根 `README.md` 第 21 节 — 开源参考与许可证标记
