# 开源参考来源与许可证审查

> 规格 §18 要求：明确记录参考了哪些功能、遵守原项目许可证、不把原项目作为最终作品。
> 本文档是该要求的执行记录。**本项目没有复制任何参考项目的源代码。**

---

## 1. 参考项目清单

### 1.1 Keiki (`Keiki-master/`)

| 项 | 内容 |
|---|---|
| 项目定位 | "A bullet hell game platform for research purpose (especially danmaku generation) written in Python" |
| 上游说明 | README 要求使用其 bibtex 引用（Wang, Liu, Yannakakis, *Keiki: Towards Realistic Danmaku Generation via Sequential GANs*, IEEE CoG 2021） |
| 许可证文件 | **未找到 LICENSE / COPYING 文件** |
| 审查结论 | `[LICENSE_REVIEW_REQUIRED]` |

**本项目借鉴的设计思路（仅思路，无代码拷贝）**

| 参考点 | Keiki 的做法 | 本项目的独立设计 |
|---|---|---|
| 弹幕组织 | `logic/runtime/bullet_creation.py` 的 `BulletBuilder` / `BatchedBulletBuilder` 家族，用 builder 把"配置"翻译成弹幕 | `generators/` 的 **纯函数生成器**：`PatternSpec -> [SpawnEvent]`，构建期一次性解析，步进期零随机；不引入 builder 对象 |
| 弹幕参数化 | `circle_cfgs / sector_cfgs / string_cfgs` 以 `(pos, angle, bend, radius, ways)` 描述扇形/环形/线形 | `PatternSpec` 单一 dataclass（count/origin/speed/angle/spread/angular_velocity/accel/ttl/radius/jitter），`params` 承载 pattern 专属参数 |
| 弹幕池 | `utils/data_structure.py` 的 `BufferedLazyList`：主列表 + 缓冲列表 + 容量上限，延迟合并 | **SoA 预分配池 + LIFO 自由列表**：连续 numpy 数组、O(1) 回收、稳态零分配、可直接映射 DMA |
| 运动表示 | `(abs)cnt, sp, rho, theta` 的极坐标弹幕表（`self.itab`） | 笛卡尔 `x,y,vx,vy,ax,ay` + `angle,angular_velocity`（协议 v2） |
| 轨迹/预演 | `agents/realtime_sim.py` 的 `RealTimeSimulator.evaluate()` 预演未来帧并做碰撞检查 | `simulator/world.py::simulate_future()` / `prediction/`：clone + step，**不复制物理逻辑** |
| 渲染与逻辑分离 | `logic/` 与 `render/` 分包 | 同样的分层，但额外保证 **内核不 import 渲染库**（有测试断言） |

**未采用**：Keiki 的 danmaku 类继承体系、GAN 生成器（`generator/TimeGAN`、`DCGAN`、`PeriodicSpatialGAN`）、PyTorch 依赖、`.npy` 弹幕数据集格式。

### 1.2 TostEngine Cinder BulletHell (`TostEngine-cinder-bullethell-main/`)

| 项 | 内容 |
|---|---|
| 项目定位 | "A 2D Bullet-Hell (Touhou-style) game engine built with Cinder framework" |
| 许可证 | **MIT License**（README «## License»） |
| 审查结论 | 允许复用，但本项目仍为独立设计，未移植其 C++ 代码 |

**本项目借鉴的设计思路**

| 参考点 | TostEngine 的做法 | 本项目的独立设计 |
|---|---|---|
| 实体结构 | `struct Bullet { Transform, Velocity{linear, angular}, Collider{radius, mask}, ... }`，`Entity` 定长数组 `MAX_ENTITIES = 8192` | `BulletPool` SoA：字段等价但**按列连续**存储（`id,x,y,vx,vy,ax,ay,angle,angular_velocity,radius,age,ttl,type_id,group_id` + `alive` 掩码） |
| **角速度** | `Velocity.angular` 驱动旋转 | `angular_velocity` 字段 + **精确圆弧积分**（闭式弧长积分，见 `physics/motion.py`） |
| 空间网格碰撞 | `GRID_COLS × GRID_ROWS` 静态网格 + `vector<int>` 桶 | `collision/grid.py`：CSR 计数排序桶 + 向量化批量查询；并**实测**单玩家时暴力法更快（见 ARCHITECTURE §7） |
| 实体池 | 定长数组 + `active` 标志 + 自由 ID 回收 | SoA + LIFO 自由列表；额外保证**分配顺序是生成的纯函数**（复现性要求） |
| 固定步长主循环 | fixed timestep game loop，`patternTimer` 驱动 pattern 切换 | `FixedClock`：`t = step_index * dt`，渲染与仿真完全解耦，支持 30/60/120/任意 dt |
| 弹幕类型 | Rain / Spiral / Burst / Aimed / Wall / Mixed 六种 | `single / radial / spiral / aimed / burst / line / wall / random / mixed` 九种，全部参数化且可运行时 `spawn_pattern()` |
| 人工操作 | 方向键/WASD 移动、Shift 聚焦 | `action/` 层：键名绑定 + `Action{direction,magnitude}` + 同时按键归一化；**Shift 表达为 magnitude 缩放而非新方向** |
| HUD 调试 | 弹幕数/难度/命中框开关 | `render/overlays.py`：FPS、弹幕数、玩家/目标状态、碰撞模型；外加危险场与预测轨迹叠加 |

**未采用**：Cinder 框架、C++ 代码、ECS 风格 `Entity` 联合体、Lives/Score/Graze/Invincibility 等**游戏机制**（规格 §13 明确排除：本项目把弹幕抽象为"动态障碍物"）。

---

## 2. 许可证合规动作

| 动作 | 状态 |
|---|---|
| 未复制任何参考项目源码 | ✅ 本项目全部为独立 Python 实现 |
| 未引入参考项目的二进制/数据文件 | ✅ `Keiki-master/danmakus/*.npy`、`data/mat/*.npy` 均未被读取或打包 |
| 未把原项目作为最终作品 | ✅ 两个参考目录保持原样，独立于 `bullet_sim/` |
| 记录借鉴的具体功能点 | ✅ 见 §1 两张表 |
| MIT 项目（TostEngine）来源标注 | ✅ 本文件 |
| **Keiki 许可证不明 → 标记待审查** | `[LICENSE_REVIEW_REQUIRED]` |

### `[LICENSE_REVIEW_REQUIRED]` 说明

`Keiki-master/` 目录下**没有 LICENSE 或 COPYING 文件**，README 只提出引用（bibtex）请求，
未给出授权条款。因此：

1. 本项目**不复制、不派生、不打包** Keiki 的任何代码与数据；
2. 仅把 Keiki 作为**公开的技术思路来源**，并在 §1.1 逐条列出借鉴点与本项目的不同实现；
3. 若后续需要直接复用 Keiki 的任何代码/数据，**必须先联系作者取得明确授权**，
   或在仓库中补充许可证后再进行；
4. 参赛材料中引用 Keiki 时按 README 要求给出 bibtex。

> 结论：当前实现**不依赖任何 Keiki 代码**，因此许可证未决不影响本项目的可用性；
> 该标记仅用于阻止未来无意中的直接复用。

---

## 3. 依赖清单

| 依赖 | 必需性 | 许可证 |
|---|---|---|
| NumPy | **必需** | BSD-3-Clause |
| pygame | 可选（可视化） | LGPL-2.1 |
| matplotlib | 可选（离线绘图） | PSF-based |
| pytest | 可选（测试） | MIT |

本项目**不依赖** Keiki 与 TostEngine 的任何运行时组件。
