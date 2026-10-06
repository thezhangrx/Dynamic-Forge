# 规范三 · 视觉输出（Vision Output）

> 文件：`vision_output.md`（本文）· `vision_output.json`（JSON Schema）· `vision_output.py`（Python 定义）
> `STANDARD_VERSION = 1`

## 1. 它是什么

**视觉模块对外的唯一输出格式。** 一帧一条 ``VisionFrame``，回答"这一帧看到了什么"：

```
VisionFrame
├── 帧头      时间戳 / 帧号 / 坐标系 / 单位 / 版本
├── 标定      用的是哪次标定、内外参、相机→场地的映射
├── player    角色（规范一）或 null
├── obstacles 障碍列表（规范二）
├── gaps      缺口列表（**派生关系**，不是实体）
├── target    目标区域（可选）
└── diagnostics  诊断计数（可选）
```

**"观测"和"实体"是两回事**：`obstacles` 里的每一项都是规范二的 `Obstacle`，
但视觉侧多了一层——它可能**没看见**（`valid=false`）、**被灯反光洗白了**（`occluded=true`）、
**只信 60%**（`score=0.6`）。这些字段属于"观测质量"，不属于"障碍本身"。

## 2. 帧头字段表

| 字段 | 类型 | 取值 | 必填 | 说明 |
|---|---|---|---|---|
| `standard_version` | int | 当前 `1` | **是** | 本规范版本 |
| `seq` | int | 单调递增 | **是** | 帧序号；**只用于去重与丢帧检测** |
| `stamp` | float | 秒 | **是** | **采集时刻**（`v4l2_buffer.timestamp` 或仿真 `sim_time`） |
| `frame_id` | string | 默认 `"field"` | **是** | 本帧**所有**位置/速度所在的坐标系 |
| `camera_frame_id` | string | 默认 `"camera"` | 否 | 相机自身坐标系（供标定追溯） |
| `units` | enum | `"px"` \| `"m"` | **是** | 载荷的单位。**标定完成前必须是 `"px"`** |
| `metres_per_unit` | float\|null | > 0 | 否 | 1 个世界单位等于多少米；`units="m"` 时为 1.0，`"px"` 且已标定时填实际值 |
| `field_origin` | float[2]\|null | 世界单位 | 否 | 图像像素 `(0,0)` 对应的场地坐标；对应 ROS `MapMetaData.origin` |
| `image_width` / `image_height` | int | px | 否 | 原始图像尺寸（诊断与重投影用） |

### 标定块 `calibration`

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | string | 标定标识；载荷里只带 id，实际参数离线查表 |
| `K` | float[9]\|null | 3×3 内参（**对应畸变原图**），行主序 |
| `D` | float[]\|null | 畸变系数 |
| `H_field_from_image` | float[9]\|null | 3×3 单应，**图像 → 场地**（把像素映射成世界坐标） |
| `rms_reprojection_px` | float\|null | 标定残差（重投影 RMS，像素） |

## 3. 载荷字段表

### `player` —— 规范一（`PlayerState`）或 `null`

见 [`../player/player.md`](../player/player.md)。注意其 `frame_id` 应等于帧头 `frame_id`。

### `obstacles` —— 规范二（`Obstacle`）数组

见 [`../obstacle/obstacle.md`](../obstacle/obstacle.md)。
**在视觉输出里 `Obstacle.id` 就是跟踪 ID（track id）**：由跟踪器分配、跨帧稳定。
它和平台真值的 `id` **不是同一个 ID 空间**——做仿真评估时需要额外一步关联（不在本规范范围）。

### `gaps` —— 缺口数组（派生）

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | int | 缺口自身的跟踪 ID（跨帧稳定，便于跟踪扫动） |
| `center` | float[2] | 缺口中心（世界系） |
| `width` | float | 缺口宽度（**与角色半径同单位**，可直接比 `2·radius`） |
| `axis` | float | 缺口轴向（弧度）= 墙长边方向 |
| `blockers` | int[2] | **构成这个缺口的两段墙的 `id`**，按主轴排序 `[左, 右]` |
| `blocked_by_type` | string | 产生该缺口的障碍类型，通常为 `"wall_with_gap"` |
| `reliable` | bool | 两侧边界是否都可信（有一侧被遮挡则为 `false`） |
| `occluded` | bool | 边界是否被遮挡/高光饱和 |
| `score` | float\|null | 置信度 |

**为什么 `reliable` / `occluded` 是必需字段**：灯在屏幕上的高光会把墙的颜色**完全洗白**
（实测 `R−B` 从 82 掉到 13，与纯眩光的 13.9 无法区分），此时缺口的一侧边界是**推出来的**
而不是看到的。硬报一个精确宽度会误导决策；标记出来，决策层才能按可信度处理
（例如改用"同场景缺口等宽"这条先验）。

### `target` —— 目标区域（可选）

| 字段 | 类型 | 说明 |
|---|---|---|
| `x` / `y` | float | 中心 |
| `radius` | float\|null | 圆形目标半径 |
| `shape` | enum | `"circle"` \| `"rect"` |
| `half_w` / `half_h` | float\|null | 矩形半尺寸 |
| `score` / `valid` / `occluded` | — | 同障碍 |

字段与平台 `target f32[6] = (x, y, radius, shape, half_w, half_h)` 对齐，
所以能直接写回 State 帧。

### `diagnostics` —— 诊断（可选，不参与决策）

| 字段 | 说明 |
|---|---|
| `n_obstacles` | 障碍数 |
| `occluded_fraction` | 被遮挡的列/像素比例 |
| `field_span` | 识别到的场地 x 范围 `[x_min, x_max]`（像素） |
| `elapsed_ms` | 本帧识别耗时 |
| `params` | 产生本帧的参数快照（可复现性） |

## 4. 关键设计决定

### 4.1 一条消息只有一个坐标系
`frame_id` 在**帧头**，不在每条检测里。理由：我们是单传感器单帧，不存在同帧多系混装；
而 ROS 的 `Twist` 正因为不带坐标系，导致三个仓库给出三套互斥约定
（`Odometry` 在 `child_frame_id`、`VelocityStamped` 用 body/reference 三元组、
`TrackedObjectKinematics` 用 `orientation_availability`）。
**规则：消费者不需要（也不允许）自行推断速度的参考系。**

### 4.2 单位必须显式：`units` + `metres_per_unit`
标定完成前 `units="px"`；完成后 `units="m"` 且 `metres_per_unit` 填实际值。
理由：`opencv-python-tutorials` 第 16 章的 README 写 `square_size=25.0`（毫米）、
脚本里是 `1.0`（无量纲）——**这种不一致会让所有距离失去物理意义**。
约定与 `nav_msgs/MapMetaData.resolution` 对齐。

### 4.3 时间：`stamp` 是采集时刻，`seq` 只用于去重
同规范一 §3.4。三个参考项目全部在这里翻车：
`optical-flow-tracker` 从不读 fps、`ByteTrack` 的 `dt` 写死为 1、
`Stereo_Vision_Camera` 定义了 `v4l2_buffer.timestamp` 却从未读取。
**测速必须用 `stamp` 之差**，绝不用 `seq`。

### 4.4 缺口是关系，不是实体
`wall_with_gap` 在平台里就是**两个矩形**（同 `group_id`）；缺口由 `blockers` 指回它们。
理由：若把缺口也做成实体，"墙段右端"与"缺口左端"会各存一份，不一致时无法判断谁对。
**分组也靠 `group_id`**：同一面墙的两段共享它，所以不需要额外的 `walls` 数组。

### 4.5 `id` 在视觉侧是**跟踪 ID**
跨帧稳定，是"从视频算速度"的前提（ByteTrack 的核心产出就是稳定 ID）。
它**不等于**平台真值的 `id`；仿真评估时需额外关联。

### 4.6 `K` 和 `P` 不能混用
`sensor_msgs/CameraInfo.msg` 里 `K` 描述**畸变原图**、`P` 描述**校正后图**，
`binning`/`roi` 还会改变像素的物理尺度。用错一个，坐标整体偏。
**我们的做法**：不发布 `P`（那是双目校正的产物），只发布 `K`（原图内参）与
`H_field_from_image`（**图像 → 场地**的单应）——一条映射，一个真值源。

### 4.7 半尺寸而非总尺寸
`half_w`/`half_h` 与规范二一致；对照 ROS `BoundingBox2D.size_x/size_y`（**总尺寸**）
时必须换算，否则差 2 倍。

### 4.8 不重复存"像素 + 世界"两套坐标
载荷只保留一套（由 `units` 决定），像素测量可通过 `H` 反投影恢复。
理由：两套坐标并存必然出现 `K`/`P` 那类"用错一个"的 bug，而且线协议体积翻倍。

## 5. 与现有实现的对照

现在 `core/vision/wall_with_gap.py` 输出 `FrameObservation`：

| 现有 | 本规范 | 迁移动作 |
|---|---|---|
| `FrameObservation.image / width / height` | 帧头 `image_*`（宽高）；文件名移到 `diagnostics` | 改名 |
| （无） | `seq` / `stamp` / `frame_id` / `units` | **新增**（当前完全没有时间与坐标系信息） |
| `FrameObservation.field`（像素 x 范围） | `diagnostics.field_span` | 降级为诊断 |
| `player: PlayerObs` | `player: PlayerState` | 加 `frame_id`/`stamp`/`score`/`occluded` |
| `walls: [Wall]`（含 `segments`） | `obstacles: [Obstacle]` + `group_id` | **拆开**：段变障碍实体，墙的概念由 `group_id` 表达 |
| `walls[i].gap: Gap` | `gaps: [GapObs]` + `blockers` | **提成顶层派生数组**，并加 `reliable`/`occluded` |
| `target: TargetObs` | 同名字段，补质量位 | 加 `score`/`valid`/`occluded` |
| `diagnostics.params` | `diagnostics.params` | 保留 |

## 6. 与 BHL1 的回灌

一帧 `VisionFrame` 可以组装成 BHL1 State 帧：

| State 帧 | 来自 |
|---|---|
| header `field_w/field_h/dt` | 平台配置（视觉侧不产生） |
| `player f32[7]` | `player.to_bhl1_player()` |
| `target f32[6]` | `target`（字段已对齐） |
| bullet `stride 17` × n | `obstacles[i].to_bhl1_bullet()` |
| 帧头 `sim_time` | 视觉侧应填 `stamp` |

**BHL1 v3 缺、本规范补上并建议进 v4 头**（与规范一/二的缺口一致）：
`frame_id`、`units` + `metres_per_unit`、`field_origin`、`calib_id`、
每实体的 `score` / `occluded` / `valid` / 协方差、以及 `track_id`
（= 视觉侧的 `id`，与平台真值 `id` 不同空间，需明确区分）。

## 7. JSON 样例（一帧）

```json
{
  "standard_version": 1,
  "seq": 741,
  "stamp": 12.345,
  "frame_id": "field",
  "camera_frame_id": "camera0",
  "units": "px",
  "metres_per_unit": null,
  "field_origin": [0.0, 0.0],
  "image_width": 640,
  "image_height": 480,
  "calibration": {
    "id": "cam0-uncalibrated",
    "K": null, "D": null, "H_field_from_image": null,
    "rms_reprojection_px": null
  },
  "player": { "x": 221.3, "y": 333.4, "radius": 9.0, "vx": 12.5, "vy": -30.0,
              "speed": 120.0, "alive": true, "score": 0.92 },
  "obstacles": [
    { "id": 2001, "type": "wall_with_gap", "type_id": 2, "shape": "rect",
      "x": 177.0, "y": 141.0, "vx": 0.0, "vy": -40.0,
      "half_w": 177.5, "half_h": 9.5, "rotation": 0.0, "angle": -1.5708,
      "group_id": 11, "score": 0.95, "valid": true, "occluded": false },
    { "id": 2002, "type": "wall_with_gap", "type_id": 2, "shape": "rect",
      "x": 447.0, "y": 153.0, "vx": 0.0, "vy": -40.0,
      "half_w": 47.5, "half_h": 8.0, "rotation": 0.0, "angle": -1.5708,
      "group_id": 11, "score": 0.71, "valid": true, "occluded": true }
  ],
  "gaps": [
    { "id": 1, "center": [377.0, 147.0], "width": 45.0, "axis": 0.0,
      "blockers": [2001, 2002], "blocked_by_type": "wall_with_gap",
      "reliable": false, "occluded": true, "score": 0.71 }
  ],
  "target": { "x": 221.4, "y": 182.6, "radius": 12.5, "shape": "circle",
              "score": 0.88, "valid": true, "occluded": false },
  "diagnostics": { "n_obstacles": 2, "occluded_fraction": 0.18,
                   "field_span": [0, 494], "elapsed_ms": 144 }
}
```

这一帧正好是**灯反光**的真实场景：第二段墙的右半段被高光洗白（`occluded: true`），
所以它参与的缺口标成 `reliable: false` —— 决策层据此知道"这个宽度是推出来的，
不是看出来的"，可以退回用"同场景缺口等宽"的先验，而不是盲信 45.0。
