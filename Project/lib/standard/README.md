# standard —— 角色 / 障碍 / 视觉输出 的统一数据结构

本目录把**角色**、**障碍**、**视觉输出**三者的数据结构一次定死，作为各模块之间的唯一契约。

```
core/standard/
├── README.md              ← 本文件：三个规范是什么、为什么这样设计、彼此怎么衔接
├── player/                ← 规范一：角色
│   ├── player.md          字段表、单位、坐标系、设计依据
│   ├── player.json        JSON Schema（机器可校验）
│   └── player.py          Python 定义（纯 stdlib，可直接 import）
├── obstacle/              ← 规范二：障碍
│   ├── obstacle.md / .json / .py
└── vision/                ← 规范三：视觉输出
    ├── vision_output.md / .json / .py
```

---

## 1. 为什么要有这个目录

在这之前，"一个障碍"在不同地方有三种说法：

| 位置 | 说法 |
|---|---|
| 平台内部 SoA（`core/bullet_sim/entities/bullet.py`） | 17 个 float 列 + `shape` 编码 + `alive` 掩码 |
| 硬件协议 BHL1 v3 | 固定 stride 17 × float32、小端、CRC32 尾 |
| 视觉模块输出（`core/vision/wall_with_gap.py`） | `Wall` / `WallSegment` / `Gap` 三个 dataclass + 像素坐标 |
| ROS 生态（参考项目 `docs/repo/data_standard/`） | `Detection2D` / `BoundingBox2D.size`（**总尺寸**）/ `Twist`（**不带坐标系**） |

每种说法单独看都自洽，**互相一对接就出错**：单位不同（px vs 世界单位）、尺寸语义不同
（半宽 vs 总宽）、坐标系不同（谁的速度？）、时间不同（帧号 vs 时间戳）。
本目录的作用就是把这些歧义在**一个地方**一次性消灭。

**数据流**（三个规范在链路里的位置）：

```
                      ┌─────────────── 规范 二：障碍 ───────────────┐
                      │  平台 SoA / BHL1 帧  ⇄  视觉识别结果         │
                      └───────────────┬─────────────────────────────┘
                                      │
 摄像头 ──▶ 视觉模块 ──▶【规范 三：视觉输出】──▶ CPU 决策 ──▶ FPGA ──▶ 执行器
                                      │                            │
                                      └────【规范 一：角色】────────┘
```

* **规范一（角色）**：既是**视觉要识别的对象**，也是**决策要控制的量**，还是**State 帧的 player 块**；
* **规范二（障碍）**：既是**平台生成的对象**，也是**视觉要识别的对象**，还是**BHL1 的 bullet 块**；
* **规范三（视觉输出）**：视觉模块对外的**唯一**输出格式，包含角色 + 障碍 + 缺口 + 时间戳 + 标定信息。

---

## 2. 三个规范各是什么

| 规范 | 它定义的实体 | 一句话 | 主要消费者 |
|---|---|---|---|
| **一、角色** `player/` | `PlayerState` | 一个**圆**：半径既是碰撞体也是画面大小；位置/速度在场地世界系 | CPU 决策、FPGA、渲染、数据集 |
| **二、障碍** `obstacle/` | `Obstacle` | 统一实体：**圆** 或 **旋转矩形**；5 种现实类型共用同一套字段，靠 `type` 区分 | 平台生成器、视觉、安全校验、BHL1 |
| **三、视觉输出** `vision/` | `VisionFrame` | 一帧里"看到什么"：角色 + 障碍列表 + 缺口列表 + 时间戳 + 标定元信息 + 质量位 | CPU 决策、（回灌）平台 State 帧 |

三者关系：**规范三是"观测"，规范一/二是"实体"**。视觉输出里的每一条都引用同一个实体定义，
区别只在于多了 `score` / `occluded` / `covariance` 这些**观测质量**字段。

---

## 3. 设计原则（每条都写了依据）

### P1 · 一个量只出现一次
能由其它字段推出的量**不单独存**：角色的朝向由 `(vx, vy)` 推出、缺口的几何由两段墙推出。
理由：两个真值源迟早会不一致。
**已知例外**：BHL1 v3 里弹幕同时存 `vx,vy` 和 `angle`——那是为了硬件侧少做一次 `atan2`，
是**有意冗余**，规范里明确标注它由速度同步而来、不得独立修改。

### P2 · 速度必须声明参考系
`geometry_msgs/Twist.msg` 只有 `linear`/`angular` 两个向量，**不带任何坐标系**，
于是三个 ROS 仓库给出了三套互斥约定：

| 消息 | 速度在哪个系 |
|---|---|
| `nav_msgs/Odometry.msg` | 注释说在 `child_frame_id` |
| `geometry_msgs/VelocityStamped.msg` | `body_frame_id` + `reference_frame_id` 三元组 |
| `autoware_perception_msgs/TrackedObjectKinematics.msg` | 用 `orientation_availability` 标记 |

**我们的选择**：**一条消息一个 `frame_id`**，消息里所有位置与速度都在该系。
理由：我们是单传感器单帧，不存在同一帧里多个坐标系混装的情况；per-entity 的 frame 在
线协议上更贵，而且允许写错。约定 `frame_id = "field"`（场地世界系）。

### P3 · 单位必须显式声明，不允许隐含
`VisionFrame.units` ∈ {`"px"`, `"m"`}，且 `metres_per_unit` 在标定完成后必须填。
理由：`nav_msgs/MapMetaData.msg` 用 `resolution` 明确"一个格是多少米"；而
`opencv-python-tutorials` 第 16 章的 README 写 `square_size=25.0`（毫米）、脚本里却是 `1.0`
（无量纲），**这种不一致会让所有距离失去物理意义**。标定没做完之前 `units="px"` 也必须显式写。

### P4 · 时间只用采集时间戳，不用帧号
`stamp` 是**采集时刻**（秒），`seq` 只是单调帧号。
理由：三个参考项目全部栽在这上面——`optical-flow-tracker` 从不读 `CAP_PROP_FPS`、
`ByteTrack` 的 `dt` 写死为 1（速度单位变成"像素/帧"）、
`Stereo_Vision_Camera` 定义了 `v4l2_buffer.timestamp` 却从未读取。
**测速只能用 `stamp` 之差**；用 `seq` 就必须额外保证帧率恒定，而帧率恰恰不恒定。

### P5 · 有就带上不确定度与质量位
每条观测都带 `score`（0~1 置信度）、`valid`、`occluded`（被遮挡/饱和）、可选协方差。
理由：ROS 里 `ObjectHypothesis.score`、`TrackedObjectKinematics.is_stationary` /
`orientation_availability` 就是干这个的；而我们的**实测教训**是灯在屏幕上的高光会把墙的颜色
完全洗白（`R−B` 从 82 掉到 13，与纯眩光的 13.9 无法区分），此时**必须能标记"这一段不可信"**，
而不是硬报一个错值。

### P6 · 形状只有两种原语：圆 或 旋转矩形
`shape` ∈ {`"circle"`, `"rect"`}，与平台的 `SHAPE_CODES = {"circle":0,"disc":0,"rect":1,"box":1}`
一致。理由：平台的障碍池本身就是这个模型——**"一个 Bullet 既可以是小圆障碍，也可以是墙，
区别只在字段解释，不在结构"**。所有现实障碍（含走廊、墙）都能用这两种原语组合出来。

### P7 · 用半尺寸，不用总尺寸
矩形一律 `half_w`/`half_h`；圆形一律 `radius`（且约定 `half_w = half_h = radius`，与平台一致）。
理由：`vision_msgs/BoundingBox2D.msg` 的 `size_x`/`size_y` 是**总尺寸**，
我们的 `half_w`/`half_h` 是**半尺寸**——**同一个障碍差 2 倍**。规范里把这条写成醒目约定，
并在与 ROS 对照的地方标注换算。

### P8 · `angle` 与 `rotation` 是两个不同的量
* `angle` = **速度方向**（弧度）；
* `rotation` = **形状朝向**（弧度，长边方向）。
理由：平台的墙生成器里，墙整体朝场内运动（`angle = theta`），而其矩形长边朝向是
`0` 或 `π/2`（`rotation`）——**两者差 90°**。把二者混用会让"墙是横着还是竖着"判断错误。

### P9 · 缺口是"关系"，不是"实体"
`wall_with_gap` 在平台里就是**两个矩形**（同一个 `group_id`），缺口是它们之间的一段空白。
所以规范里**不新增"缺口障碍"**，而是在视觉输出里把缺口作为**派生结构**发布，
用 `blockers: [id_a, id_b]` 指回那两段墙。
理由：避免两个真值源；平台侧的实体集合与视觉侧保持一致。

### P10 · 角色的半径只有一份
没有 `body_radius` / `hitbox_radius` / `body_to_hitbox_ratio`——**可视 = 碰撞**。
理由：平台已删除这些字段；尺寸比例（缺口宽度、走廊宽度）全部以 `player_diameter = 2r`
为单位，这样"两倍我自己的宽度"这种描述搬到真实小车上依然成立。

### P11 · 世界系约定：原点在场地左下，y 轴向上
与图像坐标系（原点在左上、y 向下）**相反**。翻转必须在**标定**里一次性完成，
不允许下游各自翻转。理由：平台 `README` 第 12 节早就把这条写成约定；
两处各翻一次等于没翻。

### P12 · 字段只追加，不重排
与 BHL1 的既有规则一致（`DISCRETE_ACTION_NAMES` 的顺序也是"只允许追加"）。
理由：固定 stride 是硬件能直接映射 DMA/BRAM 的前提；重排会静默破坏旧抓包。

---

## 4. 与 BHL1 v3 的对照，以及 BHL1 缺什么

### 4.1 已有字段可直接对上

| 规范 | BHL1 v3 | 位置 |
|---|---|---|
| `PlayerState.x/y/vx/vy/radius/speed/alive` | `player f32[7]` | 头 32..60 字节 |
| `Obstacle.id` | bullet `id` | stride 第 13 个 |
| `Obstacle.x/y/vx/vy/ax/ay` | bullet 前 6 个 | stride 1..6 |
| `Obstacle.angle/angular_velocity` | bullet 第 7、8 个 | — |
| `Obstacle.radius/ttl/type_id/group_id` | bullet 第 9..12 个 | — |
| `Obstacle.shape/half_w/half_h/rotation` | bullet 第 14..17 个（**v3 新增**） | stride 10 → 17 |
| `Target.x/y/radius/shape/half_w/half_h` | `target f32[6]` | 头 60..84 字节 |

**帧头已有**：`magic` / `version` / `flags` / `step_index` / `n_bullets` / `sim_time` /
`field_w` / `field_h` / `dt` + CRC32。总长 `88 + 68·n`。

### 4.2 BHL1 v3 缺的（本规范补上，建议作为 v4 扩展）

| 缺什么 | 为什么要 | 依据 |
|---|---|---|
| `frame_id` | 速度/位置的参考系 | P2；ROS 三套互斥约定 |
| `units` + `metres_per_unit` | 像素还是米、一个单位多少米 | P3；`MapMetaData.resolution` |
| `stamp`（采集时刻） | 测速的时间基准 | P4；三个项目都没读硬件时间戳 |
| `score` | 置信度，决策要按可信度加权 | P5；`ObjectHypothesis.score` |
| `occluded` / `valid` | 灯反光/遮挡时标记不可信 | P5；我们的实测 |
| `track_id` | 跨帧同一目标的稳定身份（测速的前提） | P5；ByteTrack 的核心产出 |
| `field_origin` | 世界原点在图像里的位置 | 与 `MapMetaData.origin` 对齐 |

> 注意：**`n_bullets` 与实体 `id` 不是一回事**。`id` 单调递增且跨 `compact()` 稳定，
> 碰撞报告与数据集引用它；槽位号会变。规范里所有跨帧引用一律用 `id`（视觉侧对应 `track_id`）。

---

## 5. 参考来源（都来自本项目已核验的笔记）

| 来源 | 借了什么 |
|---|---|
| `std_msgs/msg/Header.msg`（`stamp` + `frame_id`） | 每个消息都带"何时 + 何系" |
| `nav_msgs/msg/MapMetaData.msg`（`origin` + `resolution`） | 场地原点 + 每单位多少米 |
| `geometry_msgs/msg/VelocityStamped.msg`（`body_frame_id`/`reference_frame_id`） | 速度参考系的三元组写法 |
| `nav_msgs/msg/Odometry.msg`（twist 在 `child_frame_id`） | 反例：同一个量在不同消息里约定不同 |
| `sensor_msgs/msg/CameraInfo.msg`（`K` 畸变原图 / `P` 校正后图 / `D` / `binning` / `roi`） | 标定元信息该带什么；`K`/`P` 不可混用 |
| `vision_msgs`（`Detection2D` / `BoundingBox2D.size` / `ObjectHypothesisWithPose`） | 检测结果的"类别 + 置信度 + 位姿"三件套；**size 是总尺寸**（对照 P7） |
| `autoware_perception_msgs/TrackedObjectKinematics.msg`（`is_stationary` / `orientation_availability`） | 有效性标记的写法 |
| 平台 `core/bullet_sim/**` | 字段名、类型、单位、`SHAPE_CODES`、5 种障碍类型、`half_w ≥ half_h` |
| `docs/reference/Stereo_Vision_Camera.md` | `Z = f·B/d`、`δZ = Z²δd/(fB)`、V4L2 时间戳未被读取的教训 |
| `docs/reference/ByteTrack.md` | 跨帧 `track_id`；速度存在滤波状态里但**没有访问器** |
| `docs/reference/optical-flow-tracker.md` | 只有 px/帧、从不读 fps ⇒ 单位与时间必须显式 |

---

## 6. 版本策略

* 本目录的三个规范统一用 `STANDARD_VERSION`（见各 `*.py`），初版为 **1**；
* JSON 侧：字段**只追加**、已有字段语义不变；删除字段需升大版本；
* 二进制侧：与 BHL1 的关系是"**规范是语义权威、BHL1 是其线格式之一**"。
  BHL1 v3 已经能承载规范一/二的**全部实体字段**；v4 若加入 §4.2 的元信息，
  按既有规矩升 version 并保持 `readable_versions` 向后可读。

---

## 7. 怎么用

```python
# 各模块直接从 stdlib 导入，不引入第三方依赖
from standard.player.player import PlayerState
from standard.obstacle.obstacle import Obstacle, ObstacleShape, ObstacleType
from standard.vision.vision_output import VisionFrame, GapObs

p = PlayerState(x=320.0, y=105.6, vx=0.0, vy=0.0, radius=10.0, speed=120.0, alive=True)
o = Obstacle(id=7, type=ObstacleType.WALL_WITH_GAP, shape=ObstacleShape.RECT,
             x=150.0, y=270.0, vx=0.0, vy=-40.0, half_w=157.0, half_h=9.5,
             rotation=0.0, angle=-1.5708)
print(o.to_dict())
```

JSON Schema 用于跨语言校验（例如将来核对 FPGA 侧的定点打包是否正确）：
`python -m json.tool core/standard/obstacle/obstacle.json`。
