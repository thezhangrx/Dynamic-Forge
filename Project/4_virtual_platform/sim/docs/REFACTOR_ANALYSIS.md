# 重构分析：从"弹幕游戏"到"二维动态障碍环境模拟器"

> 本文是这次大范围重构的**动手前结构分析**，逐条对应需求第十六节的 10 个问题。
> 结论先行：**当前平台的核心抽象是"从点发射的小圆点"，而目标抽象是"有尺寸、有来源、
> 有运动规律、并且保证存在可行安全路径的动态障碍物"。**
>
> **后记（重构完成后）**：文中的 `patterns_for()` 等旧接口已不存在，现为
> `obstacle_kinds_for()`；装饰性弹幕生成器（`single`/`radial`/`spiral`/`aimed`/
> `burst`/`line`/`wall`/`random`）已整体删除；角色改为**一个圆**（可视 = 碰撞）；
> 障碍类型从 9 种收敛为 6 种（`moving_wall`、`closing_gap` 删除，`sparse`+`dense`
> 合并为 `small_obstacles`）。当前的权威说明见 `README.md` 与 `DESIGN.md`。

---

## 1. 当前环境模型存在的问题

| 问题 | 证据（重构前） |
|---|---|
| 障碍"凭空出现" | `generators/spec.py` 的 `origin` 默认 `"center"`；`scenarios/builder.py` 的 6 个模板里 3 个用 `origin="center"`，即障碍在场地中心生成，而玩家出生点在 `(0.5w, 0.22h)` 附近 |
| 障碍没有"来源" | `SpawnEvent` 只有 `origin + offsets + angles + speeds`，没有生成区域、入口方向、接近过程 |
| 障碍没有尺寸层级 | 所有弹幕 `radius` 统一常量（模板里 3.0），与玩家 `player_radius=3.0` 等大 |
| 障碍没有形状 | 只有圆；无法表达墙体、方块、走廊 |
| 没有"世界朝角色移动" | 所有运动都是"子弹自己飞"，缺少等价于"玩家前进"的整体反向运动表达 |
| 环境不可校验 | 只统计"生成了多少弹幕"，从不检查"是否还存在可通行空间" |

## 2. 当前 Difficulty 系统的问题

* `scenarios/complexity.py` 的 `LEVEL_VALUES = {easy, medium, hard, extreme}` 是**手工标签**；
* `bullet_count_for()` / `patterns_for()` 把它映射成"数量 + pattern 白名单"，用户**无法选择障碍类型**，只能选一个标量；
* 难度提高 = 弹幕变多，**没有任何几何可行性约束** → `extreme` 极易退化成"必死场景"；
* `autoplay` / `record` 会把这种必死场景当作正常训练数据写入数据集。

## 3. Player / Bullet 尺寸设计的问题

| 项 | 重构前 | 后果 |
|---|---|---|
| `player_radius` | 3.0（场地 640×480） | 占宽 0.47%，是数学点 |
| 弹幕 `radius` | 3.0 | 角色与一颗弹幕等大，无尺寸对比 |
| 视觉 vs 碰撞 | 同一个 `radius` | 无法做"看起来大、hitbox 很小"的实验 |
| body 概念 | 无 | 无法表达小车 / 无人机的机身外形 |
| hitbox 可配置性 | 有 `player_hitbox`（circle/point/rect）但为**附属功能**，CLI 未暴露、HUD 不显示 | 无法把碰撞体积当作实验变量 |

## 4. 当前 Spawn 逻辑的问题

```
PatternSpec(origin="center") → gen_radial/gen_burst/gen_spiral
    → 在场地中心一个点上按角度铺开
```

* 没有 **spawn region**（边界 / 走廊 / 侧向通道）；
* 没有 **entry direction**（从哪个方向进入）；
* 没有 **spawn distance**（离玩家多远出现）；
* 没有"障碍物之间必须留出通路"的任何约束；
* 结果是：环境像"中心喷泉"，而不是"外部世界有东西朝你过来"。

## 5. 当前 Collision 模型的问题

| 项 | 现状 | 缺口 |
|---|---|---|
| 形状组合 | `circle.py` 圆-圆；`shapes.py` 玩家的 circle/point/rect **仅对圆弹幕** | 矩形障碍、旋转障碍无碰撞路径 |
| 语义 | 纯瞬时重叠判定 | 与**相对速度**无关，侧向擦过与迎面撞击等价 |
| 输出 | 有 `hit_ids / frame / min_distance / risk` | 无"相对速度 / 预计碰撞时间" |
| 安全分析 | 无 | 只判断"这一帧是否重叠"，不判断"未来是否还有路" |

## 6. 建议的新 Obstacle 抽象

```python
DynamicObstacle                     # 统一抽象：形状 + 运动 + 来源
    shape        : "circle" | "rect"        # 可扩展
    half_w/half_h: float                    # rect 半尺寸（world 单位）
    radius       : float                    # circle 半径
    rotation     : float                    # 弧度，支持旋转矩形
    x, y, vx, vy, ax, ay                    # 与既有 SoA 一致
    lifetime     : ttl
    entry_direction : 由 spawn region 决定
    movement_pattern: linear / sweep / drift / patrol / closing
    spawn_region : edge / corridor / side / region
```

实现上**不新增对象系统**：在现有 SoA 池上加 `shape / half_w / half_h / rotation` 四列，
保持"连续内存 + 批量更新 + 稳态零分配"，协议升到 v3（仍可读 v1/v2）。

## 7. 建议的 Scenario 系统

```
ObstacleType（目录，9 种）
        ↓ 参数
ObstacleScenario = [ {type, count, speed, size, gap, timing, region, seed流}, ... ]
        ↓ 生成
Obstacle 生成 → Free Space 分析 → 安全路径搜索 → 有效场景
```

* 每种类型可**单独运行**，也可**任意组合**（Mixed 只是组合的特例）；
* 每种类型自带：现实语义说明、运动规律、生成区域、玩家要解决的问题；
* 旧的 `radial / spiral / aimed` 等 pattern **保留但降级**为
  "特殊障碍场景（`pattern:*`）"，不再是环境核心定义。

## 8. 建议的参数体系（Difficulty = function(参数)）

| 参数 | 含义 |
|---|---|
| `obstacle_count` | 同屏障碍数量 |
| `obstacle_size` | 障碍特征尺寸相对玩家的倍数 |
| `obstacle_speed` | 障碍绝对速度 |
| `relative_speed` | **相对速度**（`v_obs - v_player` 的统计量） |
| `obstacle_density` | 单位面积障碍占比 |
| `gap_width` / `gap_motion` | 缺口宽度 / 缺口移动方式 |
| `spawn_distance` | 从边界到玩家的生成距离 |
| `reaction_time` | 玩家可用反应时间 = `spawn_distance / relative_speed` |
| `prediction_horizon` | 需要的预测时长 |
| `field_of_view` | 可见范围（观测裁剪） |

**难度不再由用户选择**，而是由这些参数与**实测算出的自由空间 / 路径余量**派生：

```
complexity = f(free_space_fraction, min_path_clearance, path_count,
               relative_speed, prediction_horizon / reaction_time)
```

## 9. 可复用的旧代码（不重写）

| 保留 | 原因 |
|---|---|
| `core/`（SeedManager / FixedClock / ActionCodec / state_hash） | 复现性契约，重构的前提 |
| `entities/bullet.py` 的 SoA + free-list | 性能核心，只加列 |
| `physics/`（含精确圆弧积分） | 障碍运动规律直接复用 |
| `action/` 全套（Action / ActionSource / 手动 / 切换） | 需求第十四节要求**不得改动** |
| `simulator/env.py` 接口 | reset/step/observe/simulate_future 不变 |
| `dataset/` `benchmark/` `replay.py` | 只加列与字段 |
| `interface/protocol.py` 版本机制 | 已支持多版本 stride，直接扩到 v3 |
| `collision/shapes.py` 的 SDF 思路 | 扩展成障碍物 SDF |
| `collision/events.py` 接触状态机 | 与新碰撞层组合使用 |

## 10. 需要重构的部分

| 重构 | 位置 | 理由 |
|---|---|---|
| 渲染缩放 | 新增 `render/viewport.py`，改写 `render/pygame_view.py` | 需求第一节：统一渲染层缩放，逻辑坐标不变 |
| 玩家几何 | `scenarios/spec.py` + `simulator/world.py` + HUD | body 与 hitbox 分离 |
| 障碍抽象 | `entities/bullet.py` + `interface/protocol.py`(v3) | shape/尺寸/旋转 |
| 障碍类型体系 | 新增 `obstacles/` | 现实语义 + 生成区域 + 运动规律 |
| 场景系统 | 新增 `scenarios/params.py`，改造 `presets.py` | 参数空间取代固定等级 |
| 碰撞 | 新增 `collision/obstacles.py` | 矩形/旋转障碍 + 相对速度 |
| 安全闭环 | 新增 `safety/` | 强制"有挑战 ≠ 必死" |
| CLI / README | `cli.py` / `README.md` | 暴露新体系 |

---

## 附：本次重构的硬约束

1. **不破坏 Action Interface**（需求十四）；
2. **不破坏复现性**：同一 `ScenarioSpec + seed` 仍必须逐位复现；
3. **不破坏性能设计**：仍为 SoA + 批量更新 + 稳态零分配；
4. **不猜测硬件**：`[HARDWARE_INTERFACE_TBD]` 与 `[MODEL_NOT_AVAILABLE_YET]` 保持不变；
5. **有挑战 ≠ 必死**：任何自动生成的场景都必须通过安全路径校验。
