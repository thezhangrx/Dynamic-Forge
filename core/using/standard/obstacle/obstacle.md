# 规范二 · 障碍（Obstacle）

> 文件：`obstacle.md`（本文）· `obstacle.json`（JSON Schema）· `obstacle.py`（Python 定义）
> `STANDARD_VERSION = 1`

## 1. 它是什么

**一个障碍 = 一个圆 或 一个旋转矩形。** 没有第三种原语。

这不是简化，而是平台本来就有的模型：障碍池是 SoA 的 17 列，其中 `shape` 只取
`{"circle": 0, "rect": 1}`；平台文档里写得直白——**"一个 Bullet 既可以是小圆障碍，
也可以是墙，区别只在字段解释，不在结构"**。

现实里的 5 种障碍类型全部由这两种原语组合出来：

| `type` | 现实对应 | 原语 | 玩家要解决的问题 | 安全策略 |
|---|---|---|---|---|
| `moving_block` | 迎面/横向接近的大型物体（移动机器、货架、车辆） | 圆或矩形 | 提前决定从哪一侧绕行 | `bypass` |
| `wall_with_gap` | 围栏/隔断上的局部开口 | **两个矩形**（同 `group_id`） | 定位缺口，判断"能否在墙到达前抵达" | `gap` |
| `small_obstacles` | 树林枝叶 / 人群车流中的细小物体 | 多个小圆 | 保持长期连贯的可行通道 | `local_free` |
| `corridor` | 通道张开-收窄-偏转 | **两个长矩形** | 判断"现在能否通过 / 是否要等" | `corridor` |
| `cross_traffic` | 十字路口 / 多方向来流 | 圆或矩形 | 多方向相对运动之间的间隙选择 | `generic` |

## 2. 字段表

| 字段 | 类型 | 单位 / 取值 | 必填 | 说明 |
|---|---|---|---|---|
| `id` | int | ≥ 0，单调递增 | **是** | **跨帧稳定**的标识；碰撞报告与数据集引用它 |
| `frame_id` | string | 默认 `"field"` | 否 | 位置/速度/加速度所在的坐标系 |
| `type` | enum | 5 个 key 之一 | 否 | 现实类型（`moving_block` / `wall_with_gap` / `small_obstacles` / `corridor` / `cross_traffic`） |
| `type_id` | int | 见 §3.5 | 否 | 线协议用的数值编码；与 `type` 一一对应 |
| `shape` | enum | `"circle"` \| `"rect"` | **是** | 原语种类；线协议 `circle=0, rect=1` |
| `x` | float | 世界单位 | **是** | 中心 x |
| `y` | float | 世界单位 | **是** | 中心 y |
| `vx` | float | 世界单位/秒 | 否（默认 0） | 速度 x |
| `vy` | float | 世界单位/秒 | 否（默认 0） | 速度 y |
| `ax` | float | 世界单位/秒² | 否（默认 0） | 加速度 x |
| `ay` | float | 世界单位/秒² | 否（默认 0） | 加速度 y |
| `radius` | float | 世界单位 | 圆必填 | 圆的碰撞半径 |
| `half_w` | float | 世界单位 | 矩形必填 | **长边**半长（不变量：`half_w ≥ half_h`） |
| `half_h` | float | 世界单位 | 矩形必填 | **短边**半长 |
| `rotation` | float | 弧度，世界系 | 否（默认 0） | **形状朝向**（长边方向） |
| `angle` | float | 弧度 | 否 | **速度方向**（与 `rotation` 差 90°，见 §3.2） |
| `angular_velocity` | float | 弧度/秒 | 否（默认 0） | 按形状解释：矩形 = **自转**（绕自身中心，位置不变）；圆 = 速度方向的旋转率（曲线运动） |
| `ttl` | float | 秒；可 `inf` | 否 | 生命周期；走廊的墙为 `inf` |
| `age` | float | 秒 | 否 | 已存活时间 |
| `group_id` | int | ≥ 0 | 否 | 发射组：**同一面墙的两段共享同一个 `group_id`** |
| `alive` | bool | 默认 `true` | 否 | 存活掩码 |
| `valid` / `occluded` | bool | 默认 `true` / `false` | 否 | 观测可信度（同规范一 §P5） |
| `score` | float | `[0, 1]` | 否 | 检测置信度 |
| `cov` | float[3] | `[σxx, σxy, σyy]` | 否 | 位置协方差 |

### 派生量

| 派生量 | 公式 | 为什么派生 |
|---|---|---|
| `extent_w` / `extent_h` | 圆：`2r` / `2r`；矩形：`2·half_w` / `2·half_h` | 平台外侧常说"宽/高"，结构里只存半尺寸（§3.3） |
| 矩形四角 | `中心 ± half_w·(cosθ,sinθ) ± half_h·(−sinθ,cosθ)`，`θ=rotation` | 纯几何 |
| `angle` | `atan2(vy, vx)` | **在 BHL1 里是冗余存储的**（见下） |

## 3. 关键设计决定

### 3.1 半尺寸不变量：`half_w ≥ half_h`

矩形的**长边恒为 `half_w` 轴**。平台的墙生成器就是这么做的：横跨场地的墙
（`axis == "x"`）取 `half_w = 段长/2`、`half_h = 厚度/2`、`rotation = 0`；
竖过来的墙取 `half_w = 厚度/2`、`half_h = 段长/2`、`rotation = π/2`。

**为什么定成不变量而不是随便写**：识别侧只要保证 `half_w ≥ half_h`，
"长边是哪个轴"就永远不用再判断；否则每个消费者都要写一遍 `if half_w < half_h: swap`。

**圆形的约定**：`half_w = half_h = radius`（平台就是这么回填的），
这样同一套 clearance / SDF 代码对圆和矩形通用。

### 3.2 `angle` 与 `rotation` 差 90°，不是同一个量

| 字段 | 含义 | 平台的墙实际取值 |
|---|---|---|
| `rotation` | **形状**朝向（长边方向） | `0`（横墙）或 `π/2`（竖墙） |
| `angle` | **速度**方向 | 墙整体朝场内运动的方向 `theta` |

实测：一面横跨场地的墙，`rotation = 0` 而 `angle` 指向场内（约 `−π/2`），
**两者差 90°**。把二者混用会让"墙是横着还是竖着"判断错误，进而让碰撞与缺口计算全错。

### 3.3 半尺寸 vs ROS 的总尺寸

`vision_msgs/BoundingBox2D.msg` 用 `size_x`/`size_y`（**总尺寸**），
本规范用 `half_w`/`half_h`（**半尺寸**）。同一个障碍两者相差 **2 倍**。
规范选择半尺寸的理由：碰撞与 SDF 计算天然用半尺寸（`|dx| ≤ half_w`），
存总尺寸则每次都要除以 2，除不尽的浮点误差会累积到碰撞判定里。

### 3.4 缺口是"关系"，不是"实体"

`wall_with_gap` 在平台里就是**两个矩形实体**（同一 `group_id`），
缺口是它们之间的空白。所以本规范**不新增"缺口障碍"**——
缺口在**规范三（视觉输出）**里作为派生结构发布，用 `blockers: [id_a, id_b]` 指回两段墙。

理由：避免两个真值源。若把缺口也做成实体，那么"墙段右端"与"缺口左端"会各存一份，
一旦不一致就无法判断谁对。

### 3.5 `type_id` 目前**没有**权威编码，本规范把它定死

这是本轮核对代码时发现的真实缺口：平台的 `PatternSpec.type_id` 默认是 `None`，
生成器把它写成 **0**：

```python
def _type_ids(spec, n):
    return np.full(n, int(spec.type_id if spec.type_id is not None else 0), ...)
```

也就是说 `type_id` 在当前实现里只是一个**自由标签**（文档里的定位是"可视化 / 分组"），
并不是"5 种现实类型"的编码。但线协议需要一个数值编码，所以本规范定义如下（**只追加、不重排**，
与协议既有规则一致）：

| `type` | `type_id` |
|---|---|
| （未分类 / 未知） | `0` |
| `moving_block` | `1` |
| `wall_with_gap` | `2` |
| `small_obstacles` | `3` |
| `corridor` | `4` |
| `cross_traffic` | `5` |

> 落地动作：平台在构造障碍时应把 `PatternSpec.type_id` 显式设成本表的编码，
> 而不是留 `None`；否则 BHL1 帧里的 `type_id` 全是 0，视觉/决策侧无法按类型分支。

### 3.6 `id` 跨帧稳定，槽位号不稳定

平台里弹幕存在 SoA 池中，`compact()` 会让实体在数组里的**槽位**前移，
但 `id` 是**单调递增且跨 `compact()` 稳定**的。规范里所有跨帧引用（数据集、碰撞报告、
视觉侧的 `track_id` 对应关系）**一律用 `id`**，绝不用槽位号。

### 3.7 `angle` 是"有意冗余"

`angle` 完全可以由 `(vx, vy)` 算出（原则 P1 说能算的就不存），
但 BHL1 v3 里它是**独立的一个 float**。这是**为硬件故意冗余**：
FPGA 每帧对上千个实体做碰撞/接近率计算时，`atan2` 是它最不想要的算子。

**规则**：`angle` 由速度**同步**而来，**不得独立修改**；
消费者可以信任它等于 `atan2(vy, vx)`（相差 ±π 的整倍视为等价）。

## 4. 与 BHL1 v3 的逐字段映射

bullet 块 stride = **17 × float32 = 68 字节**，全程小端：

| # | BHL1 字段 | 本规范 | 备注 |
|---|---|---|---|
| 1 | `x` | `x` | |
| 2 | `y` | `y` | |
| 3 | `vx` | `vx` | |
| 4 | `vy` | `vy` | |
| 5 | `ax` | `ax` | |
| 6 | `ay` | `ay` | |
| 7 | `angle` | `angle` | **速度方向**（有意冗余，§3.7） |
| 8 | `angular_velocity` | `angular_velocity` | 按形状解释（§2） |
| 9 | `radius` | `radius` | 圆用；矩形时无意义 |
| 10 | `ttl` | `ttl` | 走廊墙为 `inf` |
| 11 | `type_id` | `type_id` | **编码见 §3.5**（当前实现恒为 0） |
| 12 | `group_id` | `group_id` | 墙的两段共享 |
| 13 | `id` | `id` | 跨 `compact()` 稳定 |
| 14 | `shape` | `shape` | v3 新增；`circle=0, rect=1` |
| 15 | `half_w` | `half_w` | v3 新增；圆时 `= radius` |
| 16 | `half_h` | `half_h` | v3 新增；圆时 `= radius` |
| 17 | `rotation` | `rotation` | v3 新增；**形状朝向** |

帧头另有 `n_bullets`（本条数）与 `field_w`/`field_h`/`dt` 等环境量。
总长 `88 + 68·n_bullets`（v1 为 `88+40n`，v2 为 `88+52n`，仍可读）。

**BHL1 v3 缺**：`type`（字符串名）、`frame_id`、`stamp`、`score`、`valid`/`occluded`、
协方差 —— 与规范一的缺口相同，建议作为 v4 头扩展统一补。

## 5. JSON 样例

**(a) 圆障碍**（`small_obstacles` 里的一颗）：

```json
{
  "id": 1042, "type": "small_obstacles", "type_id": 3, "shape": "circle",
  "frame_id": "field",
  "x": 88.0, "y": 470.0, "vx": 0.0, "vy": -95.0, "ax": 0.0, "ay": 0.0,
  "radius": 3.5, "angle": -1.5708, "angular_velocity": 0.0,
  "ttl": 12.0, "age": 1.4, "group_id": 7, "alive": true,
  "score": 0.81, "valid": true, "occluded": false
}
```

**(b) `wall_with_gap` 的**两段**墙**（同一 `group_id`，缺口在 `x ∈ [147, 190]`）：

```json
[
  {
    "id": 2001, "type": "wall_with_gap", "type_id": 2, "shape": "rect",
    "frame_id": "field",
    "x": 73.5, "y": 270.0, "vx": 0.0, "vy": -40.0,
    "half_w": 73.5, "half_h": 9.5, "rotation": 0.0, "angle": -1.5708,
    "ttl": 20.0, "group_id": 11, "alive": true, "score": 0.95
  },
  {
    "id": 2002, "type": "wall_with_gap", "type_id": 2, "shape": "rect",
    "frame_id": "field",
    "x": 415.0, "y": 270.0, "vx": 0.0, "vy": -40.0,
    "half_w": 225.0, "half_h": 9.5, "rotation": 0.0, "angle": -1.5708,
    "ttl": 20.0, "group_id": 11, "alive": true, "score": 0.95
  }
]
```

注意：**缺口本身不在这份列表里**——它由这两段推出，作为 `GapObs` 出现在规范三的
`gaps` 数组里，`blockers = [2001, 2002]`。两段的 `rotation` 都是 `0`（横墙），
而 `angle` 是 `−π/2`（朝场内，即向下），正好体现 §3.2 的 90° 差。
