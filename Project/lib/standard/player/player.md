# 规范一 · 角色（Player）

> 文件：`player.md`（本文）· `player.json`（JSON Schema）· `player.py`（Python 定义）
> `STANDARD_VERSION = 1`

## 1. 它是什么

**角色就是一个圆。** 半径既是碰撞体，也是屏幕上画出来的圆——没有
`body_radius` / `hitbox_radius` / `body_to_hitbox_ratio` 这类"看起来大、判定小"的两套尺寸。

角色同时是三种身份：

| 身份 | 谁在用 |
|---|---|
| 视觉要识别的对象 | `core/vision/` 的识别输出 |
| 决策要控制的量 | `core/bullet_sim/cpu/`、`core/fpga/` |
| State 帧的 `player` 块 | BHL1 v3 协议头 32..60 字节（`f32[7]`） |

## 2. 字段表

| 字段 | 类型 | 单位 / 取值 | 必填 | 说明 |
|---|---|---|---|---|
| `id` | string | 默认 `"player"` | 否 | 身份标识；多角色场景下区分 |
| `frame_id` | string | 默认 `"field"` | 否 | **位置与速度所在的坐标系**（见 §3.3） |
| `stamp` | float | 秒 | 否 | **采集/仿真时刻**，测速的唯一合法时间基准 |
| `seq` | int | 单调递增 | 否 | 帧序号；只用于去重与丢帧检测，**不用于算 dt** |
| `x` | float | 世界单位 | **是** | 位置 x（场地世界系，原点左下，+x 向右） |
| `y` | float | 世界单位 | **是** | 位置 y（+y 向上） |
| `radius` | float | 世界单位 | **是** | **唯一**的半径：可视 = 碰撞 |
| `vx` | float | 世界单位 / 秒 | 否（默认 0） | 速度 x |
| `vy` | float | 世界单位 / 秒 | 否（默认 0） | 速度 y |
| `speed` | float | 世界单位 / 秒 | 否 | **运动学速度上限**，不是瞬时速度（见 §3.2） |
| `alive` | bool | 默认 `true` | 否 | 存活标志 |
| `valid` | bool | 默认 `true` | 否 | 本条观测是否可信（分割失败/遮挡时为 false） |
| `occluded` | bool | 默认 `false` | 否 | 是否被遮挡/高光饱和（**灯反光场景必用**） |
| `score` | float | `[0, 1]` | 否 | 检测置信度（来自 `ObjectHypothesis.score` 的约定） |
| `pose_cov` | float[3] | `[σxx, σxy, σyy]` | 否 | 位置协方差（只保留 2D 独立分量） |
| `vel_cov` | float[3] | `[σxx, σxy, σyy]` | 否 | 速度协方差 |
| `is_stationary` | bool | — | 否 | 明确静止（速度估计在噪声内）；来自 `TrackedObjectKinematics.is_stationary` |

### 派生量（不进结构，用属性算）

| 派生量 | 公式 | 为什么派生 |
|---|---|---|
| `heading` | `atan2(vy, vx)` | 由速度唯一确定，存两份迟早不一致（原则 P1） |
| `speed_measured` | `hypot(vx, vy)` | 同上 |
| `diameter` | `2 · radius` | 平台所有间距都以它为单位（`--player-radius` 语义） |

## 3. 关键设计决定

### 3.1 半径只有一份：可视 = 碰撞

平台已经删掉了 body/hitbox 两套尺寸，原因写在它的重构分析里：**尺寸比例必须能搬到真车**。
障碍尺寸、缺口宽度、走廊宽度全部以 `player_diameter = 2r` 为单位，
所以"两倍我自己的宽度"这种描述在真实小车上依然成立。

**推论**：视觉识别的角色半径**就是**碰撞半径，不做任何收缩/外扩。
如果标定或分割让半径偏了 10%，那么所有"能否通过缺口"的判断都会偏 10%。

### 3.2 `speed` 有两个含义，必须分清

| 名字 | 含义 | 来源 |
|---|---|---|
| `speed` | **上限**：角色最大速度（运动学约束） | 平台 `ScenarioSpec.player_speed` |
| `speed_measured`（派生） | **瞬时速率** `hypot(vx,vy)` | 由 `vx,vy` 算出 |

BHL1 的 `player` 块里第 6 个 float 就是前者。**如果把它当瞬时速度用，控制回路会以为
角色一直在满速跑。** 这是规范里唯一一个"必须靠命名区分"的坑，所以两个名字都写死。

### 3.3 速度必须声明参考系

`geometry_msgs/Twist.msg` 只有 `linear`/`angular`，**不带坐标系**，于是 ROS 生态里出现三套
互斥约定（`nav_msgs/Odometry` 说在 `child_frame_id`、`geometry_msgs/VelocityStamped` 用
`body_frame_id`+`reference_frame_id` 三元组、`TrackedObjectKinematics` 用
`orientation_availability`）。这就是为什么本规范里 `frame_id` 是**字段而不是注释**。

**约定**：`frame_id = "field"`，即场地世界系；角色的位置与速度**都在这个世界系里**。
理由：决策要在世界系里算相对接近率 `v_rel = v_obs − v_player`，
如果两者不在同一系，这个减法没有意义。

### 3.4 时间：`stamp` 用于测速，`seq` 只用于去重

三个参考项目全部栽在这上面：`optical-flow-tracker` 从不读 `CAP_PROP_FPS`；
`ByteTrack` 的 `dt` 在 `kalman_filter.py:41` 写死为 1，速度单位变成"像素/帧"；
`Stereo_Vision_Camera` 定义了 `v4l2_buffer.timestamp` 却**从未读取**。

**规则**：`v = Δx / Δstamp`。只有 `stamp` 缺失时才退化用 `seq`，且必须显式知道帧率恒定。

### 3.5 世界系原点在场地左下、y 轴向上

与图像坐标系（左上、y 向下）**相反**。翻转在**标定**里一次性完成（原则 P11）。
平台默认 `field_w × field_h = 640 × 480`，角色默认出生点 `(0.5·field_w, 0.22·field_h)`。

## 4. 与 BHL1 v3 的映射

`player` 块 = `f32[7]`，位于帧头 32..60 字节：

| BHL1 下标 | 字段 | 本规范 |
|---|---|---|
| 0 | `x` | `x` |
| 1 | `y` | `y` |
| 2 | `vx` | `vx` |
| 3 | `vy` | `vy` |
| 4 | `radius` | `radius` |
| 5 | `speed` | `speed`（**上限**，见 §3.2） |
| 6 | `alive` | `alive`（1.0 / 0.0） |

**BHL1 v3 缺**：`frame_id`、`stamp`、`seq`、`score`、协方差、`occluded`/`valid`。
这些在 §4.2 里被列为建议的 v4 头扩展；在 JSON 里由本规范补上。

## 5. 与 ROS 的对照

| 本规范 | ROS | 差异 |
|---|---|---|
| `x, y` + `pose_cov` | `geometry_msgs/PoseWithCovariance` | ROS 是 6×6=36 个 float64（含 z、roll、pitch、yaw）；我们只保留 2D 的 `σxx,σxy,σyy` |
| `vx, vy` + `vel_cov` | `geometry_msgs/TwistWithCovariance` | 同上；且 ROS 的 Twist **不带坐标系**，我们用 `frame_id` 补齐 |
| `radius` | `vision_msgs/BoundingBox2D.size_x/size_y` | **ROS 是总尺寸，我们是半径**——对照时必须小心（原则 P7） |
| `score` | `vision_msgs/ObjectHypothesis.score` | 一致（`[0,1]`） |
| `stamp` + `frame_id` | `std_msgs/Header` | 一致 |
| `is_stationary` | `autoware_perception_msgs/TrackedObjectKinematics.is_stationary` | 一致 |

## 6. JSON 样例

```json
{
  "id": "player",
  "frame_id": "field",
  "stamp": 12.345,
  "seq": 741,
  "x": 221.3,
  "y": 333.4,
  "radius": 9.0,
  "vx": 12.5,
  "vy": -30.0,
  "speed": 120.0,
  "alive": true,
  "valid": true,
  "occluded": false,
  "score": 0.92,
  "pose_cov": [1.2, 0.0, 1.1],
  "vel_cov": null,
  "is_stationary": false
}
```

含义：角色在场地世界系 `(221.3, 333.4)`，半径 9.0（可视 = 碰撞），
正以 `(12.5, −30.0)` 单位/秒移动（向右下方），最大速度上限 120，
这次观测置信 0.92、位置标准差约 1.1 个单位。
`heading` 与 `speed_measured` 不存——由 `(vx,vy)` 推得（§2 派生量表）。
