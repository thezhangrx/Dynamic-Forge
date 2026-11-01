# bullet_sim 核心算法实现细节提取（逐行 / 原文引用）

本文件是从仓库真实代码中提取的实现细节，供撰写深度技术文档使用。

**提取基线（务必与该版本对齐）**

| 项 | 值 |
|---|---|
| git HEAD | `b34a93f5eb043394093c444c4bd3c7dab98998ca`（commit date: Wed Oct 7 22:02:11 2026 +0800） |
| 工作目录 | `/home/zhang/FPGA` |
| 包根 | `/home/zhang/FPGA/core`（`import bullet_sim...`） |

被提取文件的行数与 md5：

```
159 55779d84171782aad9bac915435835a0  core/bullet_sim/physics/motion.py
191 7257043c4a1290a2b78a9bb68cba7717  core/bullet_sim/physics/relative.py
277 f3acb75a593e2298f81b0ffc77e4edcd  core/bullet_sim/collision/shapes.py
301 2a32cdac8b58e532f3f1a7dd7d23568e  core/bullet_sim/collision/obstacles.py
219 18c730d1b83f740924739527f531adb3  core/bullet_sim/collision/grid.py
196 5e11124acc1e0548fb97d37d5b7d56b2  core/bullet_sim/collision/events.py
267 42cc42718710563e4f944bd06f87060c  core/bullet_sim/safety/free_space.py
370 476c0ce243d864ab419f73ca09d54ce2  core/bullet_sim/safety/path_search.py
613 af033aa1729ccf71c42068704708bdbd  core/bullet_sim/generators/patterns.py
74  df7c09129b71c6cf421a89d21a73ddd0  core/bullet_sim/core/rng.py
112 c7461799191c2f9513cb80fcafce15e5  core/bullet_sim/core/numerics.py
255 2e188697a4fb9fa7e92cb9b2c9f439b7  core/bullet_sim/core/state.py
```

**阅读约定**

- 所有代码块都是仓库原文逐字复制，行号是文件内真实行号（格式 `  行号 代码`，行号前缀不属于代码）。
- 每段引用后面标 `路径:起-止`。
- 我实际运行过的验证实验集中在本文件末尾「实测验证记录」一节；凡是我推导/推不出的地方都明确标注。
- 未做任何代码修改。

---

# 1. `core/bullet_sim/physics/motion.py` —— 积分与精确圆弧

## 1.1 它要解决什么问题

把每条弹幕/障碍物从 `t` 推进到 `t+dt`：位置、速度、加速度积分，并让"角速度"有两种互不冲突的语义（文件头注释就是规格）：

- **圆（circle）**：`angular_velocity != 0` 时速度矢量整体旋转（曲线弹）；
- **矩形（rect）**：`angular_velocity != 0` 时**刚体绕自身中心自转**（`rotation += omega*dt`），速度保持直线 —— 这样一堵墙可以原地旋转（扫动的墙 / 走廊墙）。

两种运动都必须"over one fixed step"是精确的（文件头用 `exact` 一词），以便"解析可测"而不是 Euler 近似。

## 1.2 数学

### (a) 直线 / 匀加速分支（`angular_velocity == 0` 的弹）

```
p(t+dt) = p(t) + v(t)*dt + 0.5*a*dt^2
v(t+dt) = v(t) + a*dt
```

这是常加速度下的**闭式解**（对 `a` 恒定的运动精确）。

### (b) 曲线分支：闭式圆弧积分

设 `ω = angular_velocity`，`θ = ω*dt`，旋转矩阵

```
R(θ) = [[cosθ, -sinθ],
        [sinθ,  cosθ]]
```

代码把这一步的位移写成 `dp = ∫₀^{dt} R(ω·t)·v dt` 的闭式结果。因为

```
∫₀^{θ} R(φ) dφ = [[sinθ/ω,      -(1-cosθ)/ω],
                  [(1-cosθ)/ω,   sinθ/ω    ]]
```

所以

```
dp = (sinθ/ω)·v + ((1-cosθ)/ω)·perp(v)       其中 perp(v) = (-v_y, v_x)
```

代码里就是 `dx = coef_a*vx - coef_b*vy`、`dy = coef_a*vy + coef_b*vx`（见 1.3），即 `a = sinθ/ω`、`b = (1-cosθ)/ω`。

`ω → 0` 时 `sinθ/ω → dt`、`(1-cosθ)/ω → 0`，退化成 `dp = v*dt`（文件头第 17 行原话："which reduces to `v*dt` as `omega -> 0`"）。

> **精确性的边界（我按代码推导，不是注释里的说法）**：位移积分用的是**步末速度 `v_new = v + a*dt`**，即把整步的加速度在步首一次性加到速度上，然后用 `v_new` 做整段弧积分。当 `a = 0` 时这与"速度恒为 v 按 ω 旋转"的 ODE 精确解一致（我数值验证过，见 §1.6 与实测记录 R1）；当 `a ≠ 0` 时它**不等于** `∫R(ωt)(v+a t)dt`。这一点必须按代码原样解释，不能写成"含加速度的曲线精确解"。

## 1.3 真实代码

### 常量

```python
# core/bullet_sim/physics/motion.py:37-38
37  #: Below this |omega| the arc formula is replaced by its linear limit.
38  _OMEGA_EPS = 1e-12
```

### 主积分函数（全文）

```python
# core/bullet_sim/physics/motion.py:41-115
 41  def integrate_bullets(pool: BulletPool, dt: float, mask: np.ndarray | None = None) -> None:
 42      """Advance every live bullet by one fixed step (in place)."""
 43      if pool.count == 0:
 44          return
 45      sel = pool.alive if mask is None else (pool.alive & mask)
 46      if not sel.any():
 47          return
 48  
 49      d = pool.data
 50      x, y = d["x"], d["y"]
 51      vx, vy = d["vx"], d["vy"]
 52      ax, ay = d["ax"], d["ay"]
 53      omega = d["angular_velocity"]
 54  
 55      idx = np.flatnonzero(sel)
 56  
 57      # A *rectangle* uses ``angular_velocity`` to spin its **body** about its own
 58      # centre (a swinging barrier / a corridor wall that rotates); its velocity
 59      # stays straight.  A *circle* keeps the original meaning: the velocity
 60      # vector rotates, i.e. a curved shot.  Spinning bodies are therefore
 61      # excluded from the curved-velocity path below.
 62      is_rect = np.asarray(d["shape"]) != 0
 63      spinning = is_rect & (np.abs(omega) >= _OMEGA_EPS)
 64      if spinning.any():
 65          rot = d["rotation"]
 66          rot[spinning] = (
 67              np.mod(rot[spinning] + omega[spinning] * dt + np.pi, 2.0 * np.pi) - np.pi
 68          )
 69  
 70      w = omega[idx]
 71      w = np.where(is_rect[idx], 0.0, w)
 72      straight = np.abs(w) < _OMEGA_EPS
 73  
 74      if straight.all():
 75          x[sel] += vx[sel] * dt + ax[sel] * (0.5 * dt * dt)
 76          y[sel] += vy[sel] * dt + ay[sel] * (0.5 * dt * dt)
 77          vx[sel] += ax[sel] * dt
 78          vy[sel] += ay[sel] * dt
 79      else:
 80          nvx = vx[idx] + ax[idx] * dt
 81          nvy = vy[idx] + ay[idx] * dt
 82          dx = np.empty_like(nvx)
 83          dy = np.empty_like(nvy)
 84  
 85          s = straight
 86          if s.any():
 87              dx[s] = nvx[s] * dt
 88              dy[s] = nvy[s] * dt
 89  
 90          c = ~straight
 91          if c.any():
 92              th = w[c] * dt
 93              sin_t = np.sin(th)
 94              cos_t = np.cos(th)
 95              small = np.abs(th) < 1e-8
 96              # sin(theta)/omega -> dt  and  (1-cos(theta))/omega -> 0 as theta -> 0
 97              coef_a = np.where(small, dt, sin_t / np.where(small, 1.0, w[c]))
 98              coef_b = np.where(small, 0.0, (1.0 - cos_t) / np.where(small, 1.0, w[c]))
 99              vx_c = nvx[c]
100              vy_c = nvy[c]
101              dx[c] = coef_a * vx_c - coef_b * vy_c
102              dy[c] = coef_a * vy_c + coef_b * vx_c
103              nvx[c] = cos_t * vx_c - sin_t * vy_c
104              nvy[c] = sin_t * vx_c + cos_t * vy_c
105  
106          x[idx] += dx
107          y[idx] += dy
108          vx[idx] = nvx
109          vy[idx] = nvy
110  
111      d["age"][sel] += dt
112      # Keep the ``angle`` view of the motion consistent with the velocity.
113      # NOTE: ``arr[idx] = f(arr[idx])`` writes through; ``np.arctan2(..., out=arr[idx])``
114      # would write into a *fancy-index copy* and silently do nothing.
115      d["angle"][sel] = np.arctan2(d["vy"][sel], d["vx"][sel])
```

`core/bullet_sim/physics/motion.py:41-115`

### 逐行说明

- **L43-47**：`pool.count == 0` 直接返回（O(1) 早退）；`sel = pool.alive` 是长度 = `capacity` 的布尔数组；给了 `mask` 就做逐元素逻辑与 `&`（不是位压缩，是 numpy 布尔数组的逐元素 AND，语义 = "活着 **且** 被选中"）。`sel.any()` 为假则整批都不动。
- **L49-53**：`d = pool.data` 是 SoA 字典（字段表见 `entities/bullet.py:54-74`：`id, x, y, vx, vy, ax, ay, angle, angular_velocity, radius, age, ttl, type_id, group_id, shape, half_w, half_h, rotation`）。这里把 9 个字段取成视图（`x, y, ...` 是**视图**，`+=` 会写回池子）。
- **L55**：`idx = np.flatnonzero(sel)` —— 选中的槽位下标（升序）。
- **L62**：`is_rect = np.asarray(d["shape"]) != 0`。`shape` 列是 int16（`TYPE_DTYPE`），编码 `circle=0, rect=1`（`entities/bullet.py:125`），所以 `!= 0` 就是"是矩形"。注意这里算的是**整个池子**的布尔数组。
- **L63**：`spinning = is_rect & (np.abs(omega) >= _OMEGA_EPS)` —— 只有"矩形 **且** |ω| ≥ 1e-12"才自转。
- **L64-68**：自转路径：`rot += ω*dt`，然后
  `np.mod(rot + ω*dt + π, 2π) - π`。这是一个**把角度折到 (-π, π] 的取模技巧**：先加 π 把区间平移成 [0, 2π)，取模 2π 后减 π 折回。加 π / 减 π 是为了让回绕的奇点在 -π/π（而不是 0），避免角度在 0 附近被突然折成 ±2π。`np.mod` 是 Python/numpy 语义的取模（结果符号跟除数，恒非负），所以负数角也正确回绕。
- **L70**：`w = omega[idx]` —— 注意 `idx` 是花式索引，`w` 此时是**副本**，后面 L71 会再赋一次（`np.where` 返回新数组），所以不会写回池子。
- **L71**：`w = np.where(is_rect[idx], 0.0, w)` —— 把矩形的 ω 在本函数里**局部清零**，于是矩形不会走"速度旋转"分支（这正是"矩形只自转、不拐弯"的实现方式）。
- **L72**：`straight = np.abs(w) < _OMEGA_EPS` —— 分类：|ω| < 1e-12 视为直线。
- **L74-78**：**全直线快路径**：`x += v*dt + 0.5*a*dt²`、`v += a*dt`（常加速度闭式），用布尔掩码 `x[sel] += ...` 写回。
- **L79-109**：**混合/曲线路径**：
  - L80-81：先算步末速度 `nvx, nvy = v + a*dt`（所有被选中的弹都算）。
  - L85-88：混合批里的直线弹走 `dx = nv*dt`（**半隐式**：`v*dt + a*dt²`）。**注意与 L75 的 `v*dt + 0.5*a*dt²` 不同**，差 `0.5*a*dt²`（实测见 R1）。也就是说同一条弹在这一步里走了哪条公式，取决于**同一批里有没有曲线弹**。
  - L92-95：`θ = ω*dt`；`small = |θ| < 1e-8`（内层阈值，比 `_OMEGA_EPS` 宽）。
  - L97-98：系数 `coef_a = sinθ/ω`、`coef_b = (1-cosθ)/ω`。`np.where(small, 1.0, w[c])` 的作用是**把 ω≈0 的分母替换成 1.0 以避免 0/0 警告**，被替换出来的数值随后由外层的 `np.where(small, dt, ...)` / `np.where(small, 0.0, ...)` 丢弃。numpy 的 `where` 两个分支都会被求值，所以这个"防除零"是必需的。
  - L101-102：`dp = coef_a·v_new + coef_b·perp(v_new)`，即 §1.2(b) 的闭式弧积分。
  - L103-104：`v_new ← R(θ)·v_new`（速度矢量整体旋转）。
  - L106-109：用整数下标 `idx` 写回位置与速度。
- **L111**：`age += dt`（年龄）。
- **L115**：`angle = atan2(vy, vx)`（速度方向角）。L113-114 的 NOTE 是真实存在的注释：`d["angle"][sel] = ...` 这种"先取 fancy-index 再赋值"是就地写回，而 `np.arctan2(..., out=d["angle"][sel])` 会写进临时副本、静默失效。

### 快进函数（离线 fast-forward）

```python
# core/bullet_sim/physics/motion.py:118-143
118  def advance_bullets_n(pool: BulletPool, dt: float, n: int) -> None:
119      """Advance ``n`` steps (offline fast-forward).
120  
121      For purely straight/accelerated bullets the whole jump is closed-form.
122      Curved bullets (``angular_velocity != 0``) fall back to repeated integration,
123      because a rotating velocity has no single closed-form jump that also keeps
124      ``angle`` consistent; that fallback is explicit rather than silently wrong.
125      """
126      if n <= 0 or pool.count == 0:
127          return
128      sel = pool.alive
129      if not sel.any():
130          return
131      d = pool.data
132      if np.any(d["angular_velocity"][sel] != 0.0):
133          for _ in range(int(n)):
134              integrate_bullets(pool, dt)
135          return
136  
137      t = float(n) * dt
138      d["x"][sel] += d["vx"][sel] * t + 0.5 * d["ax"][sel] * t * t
139      d["y"][sel] += d["vy"][sel] * t + 0.5 * d["ay"][sel] * t * t
140      d["vx"][sel] += d["ax"][sel] * t
141      d["vy"][sel] += d["ay"][sel] * t
142      d["age"][sel] += t
143      d["angle"][sel] = np.arctan2(d["vy"][sel], d["vx"][sel])
```

`core/bullet_sim/physics/motion.py:118-143`

逐行：L126-130 空/无活体早退；L132 判定条件是 `angular_velocity != 0.0`（**精确不等于 0**，不是 `< _OMEGA_EPS`），所以哪怕 ω=1e-15 也会退化成 n 次循环积分；L137-143 一次跳到 `T = n*dt`：位置用 `v*T + 0.5*a*T²`、速度 `v += a*T`、`age += T`，最后重算 `angle`。

### 关键常数与默认值

| 名字 | 值 | 位置 | 含义 |
|---|---|---|---|
| `_OMEGA_EPS` | `1e-12` | motion.py:38 | 分类"直线 vs 曲线"的 \|ω\| 阈值 |
| 内层 `small` 阈值 | `1e-8` | motion.py:95 | \|θ\|=\|ω·dt\| 低于它就用线性极限系数（`coef_a=dt, coef_b=0`） |
| 角度回绕 | `+π, mod 2π, −π` | motion.py:66-68 | 把 `rotation` 折进 (-π, π] |
| `shape` 编码 | `circle=0, rect=1` | entities/bullet.py:125 | L62 的 `!= 0` 判据 |

### 调用位置（上下文）

- `simulator/world.py:330`：`integrate_bullets(pool, dt)` —— 每步顺序是 `player → time → spawn → integrate → cull/expire → collision → contact events`（world.py:295-356）。
- `simulator/world.py:800`：`advance_bullets_n(self.state.bullets, self.spec.dt, int(steps))`（离线快进）。
- 玩家的积分**不在** motion.py：在 `physics/kinematics.py:33-53`（`apply_velocity_command`：`v = clip_norm(v, min(max_speed, player.speed))`，然后 `p += v*dt`，再 `clamp_to_field` 夹到 `[r, field−r]`）。`AccelerationActionCodec.encode`（`core/actions.py:193-205`）实现 `v_new = clamp(v + a*dt, max_speed)`。

---

# 2. `core/bullet_sim/physics/relative.py` —— 玩家系下的相对运动

## 2.1 它要解决什么问题

平台的核心量是**相对运动**：世界系里"玩家以 (vx,vy) 前进、障碍物以 (ovx,ovy) 移动"和玩家系里"玩家静止、障碍物以 (ovx−vx, ovy−vy) 移动"是同一个物理情形（文件头第 5-18 行）。本模块提供 `v_rel`、接近率、撞击时间，全部是 numpy 向量化的解析式。

## 2.2 数学

设玩家位置 `p`、障碍物位置 `o`、障碍物（相对）速度 `v`：

```
r        = o - p                     距离矢量
v_rel    = v_obstacle - v_player
dist     = |r|
closing_speed = -d(dist)/dt = -(v · r)/|r| = -(v · r̂)     r̂ = r/|r|
```

注意 `closing_speed` 只用了**传进来的速度**，所以调用方必须已经把它换成相对速度（本文件的 `relative_motion_stats` 就是这么做的）。

撞击时间（**玩家视为静止点 + 半径 R 的圆**，障碍物匀速直线）：

```
|r + v t|² = R²
⟺ a t² + b t + c = 0,  a = v·v,  b = 2 r·v,  c = r·r − R²
t = (−b ± √(b² − 4ac)) / (2a)
```

## 2.3 真实代码

```python
# core/bullet_sim/physics/relative.py:32-58
32  def relative_velocity(
33      obstacle_v: np.ndarray, player_v: np.ndarray
34  ) -> np.ndarray:
35      """``v_obstacle - v_player`` for ``(..., 2)`` arrays."""
36      o = np.asarray(obstacle_v, dtype=np.float64)
37      p = np.asarray(player_v, dtype=np.float64)
38      return o - p
39  
40  
41  def closing_speed(
42      positions: np.ndarray, velocities: np.ndarray, player_xy: np.ndarray
43  ) -> np.ndarray:
44      """Rate at which each obstacle closes on the player (positive = approaching).
45  
46      ``-d(dist)/dt`` for straight-line motion.  Zero means "moving parallel or
47      moving away"; positive means the gap is shrinking.
48      """
49      pos = np.asarray(positions, dtype=np.float64).reshape(-1, 2)
50      vel = np.asarray(velocities, dtype=np.float64).reshape(-1, 2)
51      me = np.asarray(player_xy, dtype=np.float64).reshape(2)
52      if pos.size == 0:
53          return np.zeros(0, dtype=np.float64)
54      rel = pos - me[None, :]
55      dist = np.hypot(rel[:, 0], rel[:, 1])
56      safe = np.maximum(dist, 1e-9)
57      unit = rel / safe[:, None]
58      return -np.einsum("ij,ij->i", vel, unit)
```

`core/bullet_sim/physics/relative.py:32-58`

逐行：
- L36-38：`v_rel = o - p`（`(..., 2)` 广播），参数顺序是**障碍物在前**。
- L49-51：`reshape(-1, 2)` 把任意形状拍成 `[N,2]`（单个 2 向量会变成 `[1,2]`）。
- L54：`rel = pos - me[None, :]` —— `me` 升维成 `[1,2]` 后按行广播相减。
- L55：`dist = hypot(dx, dy)`。
- L56：`safe = max(dist, 1e-9)` —— **防 0 除**（玩家与障碍物中心重合时），不是物理截断。
- L57：`unit = rel / safe[:, None]` —— 单位方向 `r̂`（`safe[:, None]` 把 `[N]` 变 `[N,1]` 以便按行除）。
- L58：`-np.einsum("ij,ij->i", vel, unit)` —— 爱因斯坦求和，下标串 `"ij,ij->i"` 的意思是：对两个 `[N,2]` 逐行做点积、消掉 `j`，输出 `[N]`。等价于 `-(vel[:,0]*unit[:,0] + vel[:,1]*unit[:,1])`，即 `-v·r̂`。

```python
# core/bullet_sim/physics/relative.py:61-100
 61  def time_to_impact(
 62      positions: np.ndarray,
 63      velocities: np.ndarray,
 64      player_xy: np.ndarray,
 65      *,
 66      radius: float = 0.0,
 67      max_seconds: float | None = None,
 68  ) -> np.ndarray:
 69      """Analytic first-contact time per obstacle for a *stationary* player.
 70  
 71      Assumes constant velocity (the caller should already be in the player frame
 72      if the player is moving).  ``inf`` when the obstacle never reaches the
 73      player.
 74      """
 75      pos = np.asarray(positions, dtype=np.float64).reshape(-1, 2)
 76      vel = np.asarray(velocities, dtype=np.float64).reshape(-1, 2)
 77      me = np.asarray(player_xy, dtype=np.float64).reshape(2)
 78      if pos.size == 0:
 79          return np.zeros(0, dtype=np.float64)
 80      rel = pos - me[None, :]
 81      a = np.einsum("ij,ij->i", vel, vel)
 82      b = 2.0 * np.einsum("ij,ij->i", rel, vel)
 83      c = np.einsum("ij,ij->i", rel, rel) - float(radius) ** 2
 84      out = np.full(pos.shape[0], np.inf, dtype=np.float64)
 85      moving = a > 1e-12
 86      disc = np.where(moving, b * b - 4.0 * a * c, -1.0)
 87      hit = moving & (disc >= 0.0)
 88      if hit.any():
 89          sq = np.sqrt(disc[hit])
 90          t1 = (-b[hit] - sq) / (2.0 * a[hit])
 91          t2 = (-b[hit] + sq) / (2.0 * a[hit])
 92          cand = np.where(t1 >= 0.0, t1, t2)
 93          cand = np.where(cand >= 0.0, cand, np.inf)
 94          out[hit] = cand
 95      else:
 96          # already overlapping and not moving
 97          out[moving & (c <= 0.0)] = 0.0
 98      if max_seconds is not None:
 99          out = np.where(out <= float(max_seconds), out, np.inf)
100      return out
```

`core/bullet_sim/physics/relative.py:61-100`

逐行 + 边界行为（这些是代码的真实行为，不是注释承诺）：

- **L81-83**：二次方程三个系数，全部用 `einsum("ij,ij->i")` 逐行点积（`a = |v|²`、`b = 2 r·v`、`c = |r|² − R²`）。`radius` 默认 `0.0`（点玩家）。
- **L84**：`out` 预填 `inf`（"永不撞上"的默认值）。
- **L85**：`moving = a > 1e-12`，即 `|v|² > 1e-12`（`|v| > 1e-6`）才算在动；静止的障碍物不参与二次方程。
- **L86**：`disc = where(moving, b²−4ac, −1.0)` —— 不动的直接给判别式 −1（保证不进 `hit`），同时避免除 0。
- **L87**：`hit = moving & (disc >= 0)` —— 有实根才算可能撞上。
- **L89-91**：两个根 `t1 = (−b−√disc)/(2a)`（较小）、`t2 = (−b+√disc)/(2a)`。
- **L92**：`cand = where(t1 >= 0, t1, t2)` —— 取**较早的非负根**；若 `t1 < 0 ≤ t2`（当前已在圆内，因为 `c < 0`），取到的是 **`t2`，即"离开"圆的时间**，而不是 0。
- **L93**：若连 `t2` 也是负的（两者都在过去），置 `inf`。
- **L95-97**：`else` 分支在 `hit.any()` 为假时才执行，其中唯一语句是 `out[moving & (c <= 0.0)] = 0.0`（"已重叠且没在动 → 0"）。**数学上这个分支不可达**：若 `a > 1e-12` 且 `c ≤ 0`，则 `disc = b² − 4ac ≥ b² ≥ 0`，于是 `hit` 必为真，`else` 不会执行；若 `moving` 全假，则 `moving & (c<=0)` 全假，赋值为空操作。也就是说 `out[...] = 0.0` 这一行在现有逻辑下永远不生效（我没有找到能触发它的输入）。
- **L98-99**：给了 `max_seconds` 就把"比它更晚"的答案换成 `inf`（注意 `inf <= max` 为假，`inf` 保持 `inf`）。
- 返回形状：`[N]`，`inf` 表示永不接触。

```python
# core/bullet_sim/physics/relative.py:121-144
121  def to_player_frame(
122      snapshot: WorldSnapshot, *, player_velocity: tuple[float, float] | None = None
123  ) -> WorldSnapshot:
124      """Copy of ``S_t`` in which the player is at rest.
125  
126      Positions are unchanged; the player's velocity is zeroed and subtracted from
127      every obstacle's velocity (and acceleration direction is left alone, since
128      it is a world-frame quantity).  Use this to evaluate relative motion,
129      clearances and time-to-impact without special-casing a moving player.
130      """
131      vel = (
132          np.array(player_velocity, dtype=np.float64)
133          if player_velocity is not None
134          else np.array([snapshot.player.vx, snapshot.player.vy], dtype=np.float64)
135      )
136      out = snapshot.copy()
137      player = replace(out.player, vx=0.0, vy=0.0)
138      out.player = player
139      pool = out.bullets
140      idx = pool.active_indices()
141      if idx.size:
142          pool.data["vx"][idx] = pool.data["vx"][idx] - vel[0]
143          pool.data["vy"][idx] = pool.data["vy"][idx] - vel[1]
144      return out
```

`core/bullet_sim/physics/relative.py:121-144`

逐行：L131-135 取玩家速度（可显式覆盖）；L136 `snapshot.copy()` 是深拷贝（`WorldSnapshot.copy` 见 `core/state.py:135-153`，其中 `bullets=self.bullets.copy()`；`BulletPool.copy` 在 `entities/bullet.py:425-435` 对每个字段 `v.copy()`），所以**不会改到原快照**；L137-138 玩家速度清零；L140-143 只对**活着的槽位**做速度平移（位置不动、加速度不动 —— 注释说明加速度是世界系量）。

`relative_motion_stats`（relative.py:147-180）把这些汇总成 HUD/dataset 用的字典：内部先 `rel = relative_velocity(vel, mine)`（障碍物速度 − 玩家速度），再 `close = closing_speed(pos, rel, me)`、`tti = time_to_impact(pos, rel, me, radius=player.radius)`，返回 `obstacles / player_speed / max_closing_speed / mean_closing_speed / approaching_fraction / min_time_to_impact / horizon`。注意 `min_time_to_impact` 只在 `np.isfinite(tti).any()` 时给值，否则 `None`；`horizon` 参数只是**原样回填**到结果里（函数内不参与计算）。

### 关键常数与默认值

| 名字 | 值 | 位置 | 含义 |
|---|---|---|---|
| `safe` 下限 | `1e-9` | relative.py:56 | 单位向量除法的防零 |
| `moving` 阈值 | `a > 1e-12` | relative.py:85 | \|v\| > 1e-6 才算运动 |
| `radius` | `0.0` | relative.py:66 | `time_to_impact` 默认按点玩家 |
| `relative_motion_stats` 的 `horizon` | `1.0` | relative.py:147 | 只是回填字段 |

---

# 3. `collision/shapes.py` + `collision/obstacles.py` —— SDF 命中判定

## 3.1 它要解决什么问题

把"玩家 hitbox 与弹幕/障碍物是否重叠"统一写成**有符号距离函数**（shapes.py 文件头 5-9 行）：

```
hit       ⟺  sdf(hitbox, bullet_centre) < bullet_radius
clearance =  sdf − bullet_radius        (clearance < 0 表示重叠)
```

好处：圆/点/（轴对齐）矩形都有闭式 SDF，整批弹幕一次向量化算完；自定义形状只要实现 `sdf(x, y)`。

## 3.2 数学：三种玩家 hitbox 的 SDF

**圆（CircleHitbox，`sdf = √(dx²+dy²) − R`）**：到圆周的有符号距离，圆内为负。

**点（PointHitbox，`sdf = √(dx²+dy²)`）**：半径 0 的圆（`describe()` 里 radius 报 `0.0`）。

**轴对齐矩形（RectHitbox，半宽 `half_w`、半高 `half_h`）** —— 这是 2D 盒 SDF 的标准分解：

```
q = |p − c| − h              (逐分量：qx = |dx| − half_w, qy = |dy| − half_h)
outside = |max(q, 0)|        = hypot(max(qx,0), max(qy,0))
inside  = min(max(qx, qy), 0)
sdf     = outside + inside
```

两段各管一种情形：中心在矩形外时 `max(q,·) > 0` 给出到**最近角点/边**的欧氏距离；中心在矩形内时 `outside = 0` 而 `inside = min(max(qx,qy),0) < 0` 给出到**最近边**的（负）距离（`max(qx,qy)` 挑的就是最近的那条边：`q` 越接近 0 说明该方向越贴边）。

**旋转矩形**：`shapes.py` 的 `RectHitbox` **没有旋转**（玩家 hitbox 是轴对齐的）。旋转矩形的 SDF 只在 `obstacles.py:obstacle_sdf` 里，用于**障碍物**：先把偏移旋到障碍物局部系（等价乘 `R(−θ)`），再套上面的盒 SDF：

```
d = c_obstacle − p_centre
lx =  dx·cosθ + dy·sinθ
ly = −dx·sinθ + dy·cosθ        ← 这就是 R(−θ)·d
q  = |l| − h
sdf = |max(q,0)| + min(max(qx,qy),0)
```

**约定（很重要，后面 patterns.py 的旋转要对齐这个约定）**：局部 x 轴 = 世界系方向 θ，`half_w` 沿 θ、`half_h` 沿 θ+90°。

## 3.3 真实代码：玩家 hitbox

```python
# core/bullet_sim/collision/shapes.py:54-105
 54  @dataclass(frozen=True)
 55  class CircleHitbox:
 56      """Disc of radius ``radius`` centred at the player position."""
 57  
 58      radius: float = 3.0
 59      shape: str = "circle"
 60  
 61      def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
 62          dx = bx - px
 63          dy = by - py
 64          return np.sqrt(dx * dx + dy * dy) - float(self.radius)
 65  
 66      def describe(self) -> dict[str, Any]:
 67          return {"shape": self.shape, "radius": float(self.radius)}
 68  
 69  
 70  @dataclass(frozen=True)
 71  class PointHitbox:
 72      """The classic danmaku 1-pixel hitbox."""
 73  
 74      shape: str = "point"
 75  
 76      def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
 77          dx = bx - px
 78          dy = by - py
 79          return np.sqrt(dx * dx + dy * dy)
 80  
 81      def describe(self) -> dict[str, Any]:
 82          return {"shape": self.shape, "radius": 0.0}
 83  
 84  
 85  @dataclass(frozen=True)
 86  class RectHitbox:
 87      """Axis-aligned rectangle of half extents ``(half_w, half_h)``."""
 88  
 89      half_w: float = 4.0
 90      half_h: float = 4.0
 91      shape: str = "rect"
 92  
 93      def sdf(self, px: float, py: float, bx: np.ndarray, by: np.ndarray) -> np.ndarray:
 94          qx = np.abs(bx - px) - float(self.half_w)
 95          qy = np.abs(by - py) - float(self.half_h)
 96          outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
 97          inside = np.minimum(np.maximum(qx, qy), 0.0)
 98          return outside + inside
 99  
100      def describe(self) -> dict[str, Any]:
101          return {
102              "shape": self.shape,
103              "half_w": float(self.half_w),
104              "half_h": float(self.half_h),
105          }
```

`core/bullet_sim/collision/shapes.py:54-105`

- L61-64：`sdf = hypot(dx,dy) − R`，参数是**绝对坐标** `px,py` 与弹幕中心数组 `bx,by`（文档字符串："Signed distance from each bullet centre to the hitbox surface"）。
- L76-79：点的 SDF = 到玩家中心距离（`radius` 视为 0）。
- L89-98：轴对齐矩形，`half_w/half_h` 默认都是 `4.0`；`np.hypot(a,b)` = √(a²+b²)。
- hitbox 工厂：`make_hitbox`（shapes.py:111-136）接受 `None`（→ `CircleHitbox(radius=radius)`）/ 字符串 / `Hitbox` 实例 / dict；字符串别名 `"point"|"pixel"` → `PointHitbox`，`"rect"|"rectangle"|"aabb"|"box"` → `RectHitbox(half_w=radius, half_h=radius)`，`"circle"` → `CircleHitbox`；dict 形式可给 `half_w/half_width`、`half_h/half_height`。

## 3.4 真实代码：判定与 clearance（shapes.py 的 `ShapedCollision`）

`query_hitbox`（shapes.py:182-238）核心几行：

```python
# core/bullet_sim/collision/shapes.py:201-238
201          d = pool.data
202          bx = d["x"][idx]
203          by = d["y"][idx]
204          br = d["radius"][idx]
205          sdf = hitbox.sdf(px, py, bx, by)
206          clearance = sdf - br
207          min_i = int(np.argmin(clearance))
208          min_clearance = float(clearance[min_i])
209  
210          hit_mask = clearance < 0.0 if self.strict else clearance <= 0.0
211          if not hit_mask.any():
212              return CollisionResult.miss(
213                  min_distance=min_clearance,
214                  risk=risk_from_clearance(min_clearance, self.risk_radius),
215                  frame=step,
216              )
217  
218          hit_slots = idx[hit_mask]
219          dx = bx - px
220          dy = by - py
221          d2 = dx * dx + dy * dy
222          # Report the overlapping bullet whose *centre* is closest - that is the
223          # one a consumer wants to highlight or attribute the hit to.
224          centre_d2 = d2[hit_mask]
225          k2 = int(np.argmin(centre_d2))
226          slot = int(hit_slots[k2])
227          return CollisionResult(
228              hit=True,
229              index=slot,
230              bullet_id=int(d["id"][slot]),
231              hit_ids=tuple(int(v) for v in d["id"][hit_slots]),
232              count=int(hit_slots.size),
233              min_dist2=float(centre_d2[k2]),
234              min_distance=min_clearance,
235              frame=step,
236              risk=1.0,
237              position=(float(d["x"][slot]), float(d["y"][slot])),
238          )
```

`core/bullet_sim/collision/shapes.py:201-238`

逐行：L201-205 取活体弹幕的 x/y/半径并算整批 SDF；L206 `clearance = sdf − r_bullet`（**负 = 重叠**）；L207-208 记录最小 clearance（= 最近表面间隙）；L210 `strict=True` 时用 `< 0`，否则 `<= 0`（相切算不算命中）；L211-216 无命中直接返回 `miss`，并把 `risk_from_clearance(min_clearance, risk_radius)` 带上；L218 取命中槽位；L219-226 在命中集合里再按**中心距平方**挑一个代表（注释 L222-223：给消费者高亮/归因用）；L227-238 组装结果：`hit_ids` 是**全部**命中弹幕的 id 元组（接触状态机需要全集，见 §5），`risk=1.0`，`position` 是代表弹幕的位置。

`clearance_at`（shapes.py:240-249）：`float(np.min(sdf - d["radius"][idx]))`（空池返回 `inf`）。

## 3.5 真实代码：旋转矩形 SDF + 相对运动接触率（obstacles.py）

```python
# core/bullet_sim/collision/obstacles.py:42-97
 42  def player_circumradius(hitbox: Any) -> float:
 43      """Conservative radius of a player hitbox (exact for circle/point)."""
 44      shape = getattr(hitbox, "shape", "circle")
 45      if shape in ("circle", "disc"):
 46          return float(getattr(hitbox, "radius", 0.0))
 47      if shape in ("point", "pixel"):
 48          return 0.0
 49      if shape in ("rect", "box"):
 50          return float(
 51              np.hypot(
 52                  float(getattr(hitbox, "half_w", 0.0)),
 53                  float(getattr(hitbox, "half_h", 0.0)),
 54              )
 55          )
 56      raise ValueError(f"unsupported hitbox shape {shape!r}")
 57  
 58  
 59  def obstacle_sdf(
 60      px: float,
 61      py: float,
 62      ox: np.ndarray,
 63      oy: np.ndarray,
 64      shape: np.ndarray,
 65      radius: np.ndarray,
 66      half_w: np.ndarray,
 67      half_h: np.ndarray,
 68      rotation: np.ndarray,
 69  ) -> np.ndarray:
 70      """Signed distance from the player centre to each obstacle surface.
 71  
 72      ``< 0`` means the player centre is inside the obstacle.  Works for circular
 73      and rotated-rectangular obstacles in one vectorised pass.
 74      """
 75      dx = ox - px
 76      dy = oy - py
 77      out = np.empty_like(dx, dtype=np.float64)
 78  
 79      circle = shape == 0
 80      if circle.any():
 81          out[circle] = np.hypot(dx[circle], dy[circle]) - radius[circle]
 82  
 83      rect = ~circle
 84      if rect.any():
 85          cos_r = np.cos(rotation[rect])
 86          sin_r = np.sin(rotation[rect])
 87          ddx = dx[rect]
 88          ddy = dy[rect]
 89          # rotate the offset into the obstacle's local frame
 90          lx = ddx * cos_r + ddy * sin_r
 91          ly = -ddx * sin_r + ddy * cos_r
 92          qx = np.abs(lx) - half_w[rect]
 93          qy = np.abs(ly) - half_h[rect]
 94          outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
 95          inside = np.minimum(np.maximum(qx, qy), 0.0)
 96          out[rect] = outside + inside
 97      return out
```

`core/bullet_sim/collision/obstacles.py:42-97`

- L44-55 `player_circumradius`：圆 = `radius`；点 = `0.0`；**矩形 = `hypot(half_w, half_h)`（外接圆半径）**。文件头 8-14 行明确说明：矩形玩家用外接圆是**保守**的（角擦会被报成命中），并且这是刻意选择的偏差方向。
- L75-77：`dx = ox − px`（注意方向：**障碍物减玩家**，所以 `sdf > 0` 在障碍物外）；`out` 预分配。
- L79-81：`shape == 0` 是圆（`shape` 列是 int16 码，0=circle，1=rect）。
- L83-96：矩形分支：
  - L85-86：逐元素 `cos/sin(rotation)`；
  - L90-91：`lx = dx·cos + dy·sin`，`ly = −dx·sin + dy·cos` —— 把 `(dx,dy)` 旋转 `−θ` 到障碍物局部系（推导：局部向量 = `R(−θ)·d`，`R(−θ) = [[cosθ, sinθ],[−sinθ, cosθ]]`）；
  - L92-95：盒 SDF（与 shapes.py 的轴对齐版完全同式，只是坐标换成 `lx, ly`）；
  - `shape != 0` 一律按矩形处理（编码里只有 0/1）。

矩阵版 `obstacle_sdf_matrix`（obstacles.py:100-140）把同一套公式广播成 `(cells, obstacles)`：`gx,gy` reshape 成 `[-1,1]`（列向量，按 cell 广播），`ox,oy` reshape 成 `[1,-1]`（行向量，按障碍物广播），于是 `dx = ox − gx` 是 `(cells, obstacles)`；圆列与矩形列分别用 `out[:, circle[0]]` 填充；`circle[0]` 是把 `(1,n)` 的布尔行取出来当成列掩码用。注释（L111-116）说明它的用途是自由空间栅格化："one cell is blocked if *any* obstacle is closer than the inflation radius, so this is the exact configuration-space test rather than a bounding-box approximation"。

### 相对运动接触率 `contact_rate`

```python
# core/bullet_sim/collision/obstacles.py:195-217
195      def contact_rate(
196          self, pool: BulletPool, px: float, py: float, pr: float, pvx: float, pvy: float
197      ) -> np.ndarray:
198          """``-d(clearance)/dt`` per obstacle: how fast the gap is closing.
199  
200          Positive = approaching, negative = separating, ~0 = sliding past.  This
201          is the relative-motion signal the planner and the reward can use; the
202          geometric overlap test alone cannot distinguish these cases.
203          """
204          idx = pool.active_indices()
205          if idx.size == 0:
206              return np.zeros(0)
207          d = pool.data
208          ox = d["x"][idx].astype(np.float64)
209          oy = d["y"][idx].astype(np.float64)
210          rel_x = ox - px
211          rel_y = oy - py
212          dist = np.maximum(np.hypot(rel_x, rel_y), 1e-9)
213          unit_x = rel_x / dist
214          unit_y = rel_y / dist
215          rvx = d["vx"][idx].astype(np.float64) - float(pvx)
216          rvy = d["vy"][idx].astype(np.float64) - float(pvy)
217          return -(rvx * unit_x + rvy * unit_y)
```

`core/bullet_sim/collision/obstacles.py:195-217`

逐行：L204-206 无活体返回空数组；L208-211 `rel = 障碍物中心 − 玩家中心`；L212 `dist = max(|rel|, 1e-9)`（防 0 除）；L213-214 单位向量 `û`；L215-216 **相对速度** `v_rel = v_obstacle − v_player`（这里才是"相对运动"的落点）；L217 返回 `−(v_rel · û)`。

数学：`contact_rate = −d(clearance)/dt`。严格地 `d(clearance)/dt = ∇sdf · v_rel`，而代码用的是"中心连线方向" `û`。**对圆形障碍物** `∇sdf = û`，所以是精确的；**对矩形障碍物** `∇sdf` 是面法向/角点方向，只有在"玩家落在矩形某条边的外侧带内"时才等于 `û`，所以这是近似（这一点是我按公式推导的，代码注释没有区分圆和矩形）。

`query_hitbox`（obstacles.py:231-277）：先 `pr = player_circumradius(hitbox)`，再 `clearance = self.clearances(pool, px, py, pr)`（= `sdf − pr`，obstacles.py:183），`hit = clearance < 0.0 if strict else <= 0.0`，然后在命中集合里按**中心距平方**挑代表；`clearances()` 会把最近一次的 `idx` 存进 `self._last_index`、clearance 存进 `self._last_clearance`（L171、183；属性 `last_clearances` / `last_ids` 在 L186-193）。

### 关键常数与默认值

| 名字 | 值 | 位置 | 含义 |
|---|---|---|---|
| `CircleHitbox.radius` | `3.0` | shapes.py:58 | 默认玩家半径 |
| `RectHitbox.half_w/half_h` | `4.0 / 4.0` | shapes.py:89-90 | 默认矩形半宽高 |
| `DEFAULT_RISK_RADIUS` | `32.0` | collision/base.py:35 | `risk = clamp(1 − clearance/32, 0, 1)`；`clearance ≤ 0 → 1.0`（base.py:38-46） |
| `strict` | `True` | shapes.py:158 / obstacles.py:153 | `True` 用严格 `< 0`，`False` 用 `≤ 0` |
| `contact_rate` 的 `1e-9` | — | obstacles.py:212 | 防 0 除 |
| `describe()["broadphase"]` | `"none"` | shapes.py:260 / obstacles.py:289 | 这两个后端不做宽相（全量向量化） |

---

# 4. `core/bullet_sim/collision/grid.py` —— 均匀网格宽相 + CSR 计数排序

## 4.1 它要解决什么问题

**单次**玩家查询在 NumPy 下暴力已经最优（文件头 5-10 行），所以网格的价值在**批量**查询：同一批弹幕 × 多个玩家（向量化 RL、策略搜索、多智能体）。目标是把 `O(P·N)` 变成 `O(P × 每格弹幕数)`。

数据结构是**计数排序的 CSR**：`order` = 按格分组后的活体槽位下标，`start` = 每格的偏移（长度 `ncells+1`），每步 `prepare()` 重建一次。

## 4.2 算法与数学

**桶（cell）编号**（`_cell_ids`）：

```
cx = floor((x + margin) / cell)    ← 夹到 [0, cols−1]
cy = floor((y + margin) / cell)    ← 夹到 [0, rows−1]
cell_id = cy * cols + cx           ← 行主序（row-major）
```

网格尺寸（`_rebuild_dims`）：

```
cell = cell_size 或 max(4·max_r, 1.0)          ← max_r = 当前活体最大半径
cols = max(1, ceil((field_w + 2·margin) / cell))
rows = max(1, ceil((field_h + 2·margin) / cell))
```

**CSR 计数排序**（`prepare`）：

1. `cells = _cell_ids(x_idx, y_idx)`（长度 n）
2. `counts = np.bincount(cells, minlength=ncells)` → `counts[c]` = 第 c 格弹幕数
3. `start[0] = 0`，`np.cumsum(counts, out=start[1:])` → `start[c]` 是前缀和，所以第 c 格占据 `order[start[c] : start[c+1]]`
4. `order = idx[np.argsort(cells, kind="stable")]` → 稳定排序，所以**同一格内的槽位保持升序**（确定性）

**批量查询的邻域**：

```
reach = ceil((max_player_r + 2·max_r) / cell)
for dy in -reach..+reach: for dx in -reach..+reach:   (2·reach+1)² 个格
```

充分性（我推导的，可写进文档）：设玩家中心在格 `i_p`、半径 `pr`，弹幕中心在格 `i_b`、半径 `r_b ≤ max_r`；命中要求 `|p−b| ≤ pr + r_b`，故单轴 `|Δ| ≤ pr + max_r`；由 `|floor(u)−floor(v)| ≤ ceil(|u−v|)` 得 `|i_p − i_b| ≤ ceil((pr + max_r)/cell) ≤ ceil((pr + 2·max_r)/cell) = reach`。所以这个 `reach` 恒足够（`cell>0`、`max_r>0`）。

**去重**：`query_many` 里**没有**任何去重（没有 `np.unique`、没有 set）。正确性来自几何：2D 网格里每条弹幕只属于**一个**格，而 `(2·reach+1)²` 邻域里的格编号互不相同，所以"玩家 × 邻域格"的展开中同一条弹幕对同一玩家最多出现一次。同一个玩家被多次命中的合并靠 `out[pidx[hit]] = True`（布尔 OR 语义，重复赋 True 无害）。

**CSR gather（`within` 的构造）**：对当前 (dx,dy) 偏移，先挑出合法的玩家 `pidx = flatnonzero(ok)`、对应格 `cells`，每格的弹幕数 `counts = start[cells+1] − start[cells]`：

```
reps   = repeat(pidx, counts)                      每个玩家重复"它这一格有几条"
offs   = repeat(start[cells], counts)              每个玩家这一格的桶起点
within = arange(total) − repeat(cumsum(counts) − counts, counts)
slot   = order[offs + within]
```

`cumsum(counts) − counts` 是"到该玩家为止前面所有玩家占用的条目数"（exclusive prefix sum），`arange(total)` 是全局条目序号；两者相减得到**该条目在自己的 bucket 段内的序号**，加上桶起点就能从 `order` 里取到真实槽位。这就是标准 CSR 段内索引。

## 4.3 真实代码

```python
# core/bullet_sim/collision/grid.py:30-101
 30      def __init__(self, cell_size: float | None = None, strict: bool = True) -> None:
 31          self.cell_size = cell_size
 32          self.strict = bool(strict)
 33          self._fallback = CircleCollision(strict=strict)
 34          self._stats = CollisionStats()
 35          self._field_w = 0.0
 36          self._field_h = 0.0
 37          self._margin = 0.0
 38          self._cell = 1.0
 39          self._cols = 1
 40          self._rows = 1
 41          self._start = np.zeros(1, dtype=np.int64)
 42          self._order = np.empty(0, dtype=np.int64)
 43          self._max_r = 1.0
 44  
 45      # ------------------------------------------------------------------
 46      def configure(self, field_w: float, field_h: float, margin: float = 0.0) -> None:
 47          self._field_w = float(field_w)
 48          self._field_h = float(field_h)
 49          self._margin = float(margin)
 50          self._rebuild_dims()
 51  
 52      def _rebuild_dims(self) -> None:
 53          cell = self.cell_size if self.cell_size else max(4.0 * self._max_r, 1.0)
 54          self._cell = max(float(cell), 1e-9)
 55          self._cols = max(1, int(np.ceil((self._field_w + 2 * self._margin) / self._cell)))
 56          self._rows = max(1, int(np.ceil((self._field_h + 2 * self._margin) / self._cell)))
 57          self._start = np.zeros(self._cols * self._rows + 1, dtype=np.int64)
 58  
 59      # ------------------------------------------------------------------
 60      def prepare(self, pool: BulletPool) -> None:
 61          idx = pool.active_indices()
 62          n = int(idx.size)
 63          if self._field_w <= 0.0:
 64              # Never configured explicitly: derive bounds from the bullets so the
 65              # backend still behaves correctly when used standalone.
 66              if n == 0:
 67                  return
 68              d = pool.data
 69              pad = float(np.max(d["radius"][idx])) * 4.0 + 1.0
 70              self.configure(
 71                  float(np.max(d["x"][idx])) + pad, float(np.max(d["y"][idx])) + pad, pad
 72              )
 73          if n == 0:
 74              self._order = np.empty(0, dtype=np.int64)
 75              self._start[:] = 0
 76              return
 77          d = pool.data
 78          max_r = float(np.max(d["radius"][idx])) if n else 1.0
 79          if self.cell_size is None and max_r > self._max_r * 1.5:
 80              self._max_r = max_r
 81              self._rebuild_dims()
 82          elif self.cell_size is None:
 83              self._max_r = max(self._max_r, max_r)
 84          cells = self._cell_ids(d["x"][idx], d["y"][idx])
 85          ncells = self._cols * self._rows
 86          counts = np.bincount(cells, minlength=ncells)
 87          start = np.zeros(ncells + 1, dtype=np.int64)
 88          np.cumsum(counts, out=start[1:])
 89          order = np.empty(n, dtype=np.int64)
 90          # Counting sort; np.argsort(kind='stable') on cells preserves slot order
 91          # inside each bucket, which keeps query results deterministic.
 92          order[:] = idx[np.argsort(cells, kind="stable")]
 93          self._start = start
 94          self._order = order
 95  
 96      def _cell_ids(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
 97          cx = np.floor((x + self._margin) / self._cell).astype(np.int64)
 98          cy = np.floor((y + self._margin) / self._cell).astype(np.int64)
 99          np.clip(cx, 0, self._cols - 1, out=cx)
100          np.clip(cy, 0, self._rows - 1, out=cy)
101          return cy * self._cols + cx
```

`core/bullet_sim/collision/grid.py:30-101`

逐行：
- L31-43：`cell_size=None` 表示自适应；`_fallback = CircleCollision(strict=strict)` 用于单次查询；`_start/_order` 初始为空。
- L53：`cell = self.cell_size if self.cell_size else max(4.0*self._max_r, 1.0)` —— 未指定格宽时取 **4×最大弹幕半径**，下界 1.0。
- L54：`max(cell, 1e-9)` 防 0 格宽。
- L55-56：`ceil((field + 2·margin)/cell)`（`margin` 让实体中心落在 `[−margin, field+margin]` 内的弹幕也有格可放）。
- L57：`start` 长度 = `ncells + 1`（CSR 标准做法，用 `start[c]`/`start[c+1]` 表示桶区间；顺便躲开"最后一格没有上界"的边界判断）。
- L63-72：**从未 `configure` 过**时（`field_w <= 0`）用弹幕自身包围盒推导场地：`pad = 4·max_r + 1`，`configure(max_x+pad, max_y+pad, pad)`。
- L73-76：无活体：`order` 清空，`start[:] = 0`（整数组置 0）。
- L78-83：`max_r` 增长超过 `_max_r` 的 1.5 倍才重建尺寸（避免频繁重建），否则只抬升 `_max_r`；`cell_size` 显式给定时这一支完全不参与。
- L84-88：`cells → bincount → cumsum`，即 §4.2 的计数排序前两步；`np.cumsum(counts, out=start[1:])` 把前缀和直接写进 `start` 的第 1..ncells 位（`start[0]` 保持 0）。
- L90-92：注释说明为什么用 `kind="stable"`：同格内保持**槽位升序**，查询结果确定（如果换成不稳定的 `argsort`，同格内的顺序可能随实现/数据变化）。
- L96-101：格编号；`np.clip(..., out=cx)` 是**就地把越界坐标折回边界格**（所以场地外的玩家/弹幕会被当成在边界格上，属于近似），`cy*cols+cx` 是行主序线性化。

```python
# core/bullet_sim/collision/grid.py:104-175
104      def query(
105          self,
106          pool: BulletPool,
107          px: float,
108          py: float,
109          pr: float,
110          *,
111          frame: int | None = None,
112      ) -> CollisionResult:
113          # Single-query: brute force is cheaper than building/descending buckets.
114          self._stats.queries += 1
115          res = self._fallback.query(pool, px, py, pr, frame=frame)
116          self._stats.candidate_tests += self._fallback.stats().candidate_tests
117          return res
118  
119      def query_many(
120          self, pool: BulletPool, px: np.ndarray, py: np.ndarray, pr: np.ndarray
121      ) -> np.ndarray:
122          """Vectorised batch query for ``P`` players -> ``bool[P]``."""
123          p = np.asarray(px, dtype=np.float64).reshape(-1)
124          q = np.asarray(py, dtype=np.float64).reshape(-1)
125          r = np.asarray(pr, dtype=np.float64).reshape(-1)
126          P = p.size
127          if pool.count == 0 or P == 0 or self._order.size == 0:
128              return np.zeros(P, dtype=bool)
129          self._stats.queries += P
130          d = pool.data
131  
132          # A player can only touch bullets whose centre is within
133          # (pr + max_bullet_radius); with a >= 4*max_r cell size the 3x3
134          # neighbourhood is guaranteed to be sufficient.
135          base = self._cell_ids(p, q)
136          reach = int(np.ceil((float(np.max(r)) + self._max_r * 2.0) / self._cell))
137  
138          pid_list = []
139          slot_list = []
140          for dy in range(-reach, reach + 1):
141              for dx in range(-reach, reach + 1):
142                  cxi = (base % self._cols) + dx
143                  cyi = (base // self._cols) + dy
144                  ok = (cxi >= 0) & (cxi < self._cols) & (cyi >= 0) & (cyi < self._rows)
145                  if not ok.any():
146                      continue
147                  cells = cyi[ok] * self._cols + cxi[ok]
148                  pidx = np.flatnonzero(ok)
149                  counts = self._start[cells + 1] - self._start[cells]
150                  total = int(counts.sum())
151                  if total == 0:
152                      continue
153                  reps = np.repeat(pidx, counts)
154                  offs = np.repeat(self._start[cells], counts)
155                  within = np.arange(total, dtype=np.int64) - np.repeat(
156                      np.cumsum(counts) - counts, counts
157                  )
158                  pid_list.append(reps)
159                  slot_list.append(self._order[offs + within])
160  
161          if not pid_list:
162              return np.zeros(P, dtype=bool)
163  
164          pidx = np.concatenate(pid_list)
165          slots = np.concatenate(slot_list)
166          self._stats.candidate_tests += int(slots.size)
167          ddx = d["x"][slots] - p[pidx]
168          ddy = d["y"][slots] - q[pidx]
169          d2 = ddx * ddx + ddy * ddy
170          rr = d["radius"][slots] + r[pidx]
171          hit = d2 < rr * rr if self.strict else d2 <= rr * rr
172          out = np.zeros(P, dtype=bool)
173          if hit.any():
174              out[pidx[hit]] = True
175          return out
```

`core/bullet_sim/collision/grid.py:104-175`

逐行：
- L113-117：**单次查询直接委托给暴力后端**（`CircleCollision`），并把子后端的 `candidate_tests` 累加进自己的统计；注释解释原因（建/下降桶比暴力更贵）。
- L123-126：`reshape(-1)` 展平；`P = p.size`。
- L127-128：空池/空玩家/未 `prepare`（`_order` 为空）→ 全 False。
- L135：`base = self._cell_ids(p, q)` —— 玩家所在格（**越界玩家被 clip 到边界格**）。
- L136：`reach = ceil((max(pr) + 2·max_r) / cell)`（§4.2 已证明充分；注意用的是**玩家半径的最大值**，一个保守上界，且 `_max_r` 是 `prepare` 维护的最大弹幕半径）。
- L140-141：双层循环遍历 `(2·reach+1)²` 个格偏移。
- L142-143：`base % self._cols` 取列、`base // self._cols` 取行（**这里没有位运算**：`cols` 是 `ceil(宽/格)`，不是 2 的幂，所以用 `%`/`//` 而不是 `& (cols−1)` / `>> shift`）。
- L144：`ok` 是四个布尔条件的逐元素与（`&`），把越界格剔除。
- L147-152：合法格的线性编号、对应玩家下标、每格条目数 = `start[cells+1] − start[cells]`；`total == 0` 的偏移整段跳过。
- L153-157：CSR 段内展开（§4.2 公式）：`reps` = 玩家下标按条数重复；`offs` = 桶起点按条数重复；`within` = 段内序号。
- L159：`self._order[offs + within]` → 真实槽位下标。
- L161-165：所有偏移的 (玩家, 槽位) 对拼起来。
- L167-171：中心的向量化圆-圆判定：`d2 < (r_bullet + r_player)²`（`strict` 时严格小于，否则 `≤`）。
- L172-174：`out[pidx[hit]] = True` —— 把每对命中映射回玩家（同一玩家的多次命中重复赋 True，起 OR 作用）。

### 关键常数与默认值

| 名字 | 值 | 位置 | 含义 |
|---|---|---|---|
| `cell_size` | `None` | grid.py:30 | `None` → 自适应 `max(4·max_r, 1.0)`（grid.py:53） |
| `_max_r` 初值 | `1.0` | grid.py:43 | 自适应格宽用的最大半径 |
| 重建阈值 | `> _max_r * 1.5` | grid.py:79 | 半径涨过 1.5 倍才重建网格尺寸 |
| `_cell` 下限 | `1e-9` | grid.py:54 | 防 0 |
| `pad`（未 configure 时） | `4·max_r + 1` | grid.py:69 | 从弹幕推导场地时的外扩 |
| `strict` | `True` | grid.py:30 | 严格 `<` |
| 单查询路径 | 委托 `CircleCollision` | grid.py:115 | 网格只服务批量查询 |

---

# 5. `core/bullet_sim/collision/events.py` —— 接触状态机

## 5.1 它要解决什么问题

检测回答"这一帧有没有重叠"，而奖励/指标需要的是"**是不是新开始的一次碰撞**"：同一颗弹幕连续重叠 40 帧是 **1 次**碰撞。接触按**弹幕 id**（不是槽位、不是位置）跟踪，所以"第一颗还贴着时又蹭到第二颗"算第二次事件；"从一颗上分开、另一颗还贴着"也不会在之后凭空产生"重新进入"。

## 5.2 状态机与三种计数模式（数学）

用集合表示：`A_prev` = 上一帧的接触 id 集合，`C` = 本帧重叠 id 集合（来自 `CollisionResult.hit_ids`）：

```
entered = C − A_prev        （集合差，本帧新贴上的 id）
exited  = A_prev − C        （本帧分离的 id）
A       = C                 （状态更新为当前集合）
```

事件数 `events`（模式相关）：

- `per_contact`（默认）：`events = |entered|` —— 每个"新贴上的弹幕"一次事件；
- `per_step`：`events = 1 if (entered and contact_frames == 0) else 0` —— 每个"接触段"一次（`contact_frames` 是**连续接触帧计数**，上一帧没接触时它为 0）；
- `per_frame`（legacy）：`events = 1 if C else 0` —— 每个重叠帧都算一次。

字段维护（`update` 内，顺序重要：`events` 在 `contact_frames` 更新**之前**计算）：

| 字段 | 维护规则 | 位置 |
|---|---|---|
| `collision_count` | `+= events` | events.py:121 |
| `entered_total` | `+= len(entered)` | events.py:122 |
| `exited_total` | `+= len(exited)` | events.py:123 |
| `overlap_frames` | 只要 `C` 非空就 `+= 1`（**从不重置**） | events.py:124-125 |
| `contact_frames` | `C` 非空 `+= 1`；`C` 为空则 `= 0`（**连续段计数**） | events.py:124-128 |
| `first_event_frame` | 第一次 `events>0` 时记下帧号，之后不再改 | events.py:129-132 |
| `last_event_frame` | 每次 `events>0` 覆盖为当前帧 | events.py:133 |
| `last_transition` | 本帧的 `ContactTransition`（含 `entered/exited/active/events/hit`） | events.py:135-142 |

`overlap_frames` 与 `contact_frames` 的唯一区别：前者是**累积**重叠帧数，后者是**当前连续段**长度（分离即归零）；两者在连续接触期间同步递增。

## 5.3 真实代码

```python
# core/bullet_sim/collision/events.py:87-143
 87  class ContactTracker:
 88      """Per-bullet enter/exit state machine for the player's hitbox."""
 89  
 90      def __init__(self, mode: str = "per_contact") -> None:
 91          self.mode = validate_count_mode(mode)
 92          self.reset()
 93  
 94      # ------------------------------------------------------------------
 95      def reset(self) -> None:
 96          self._active: set[int] = set()
 97          self.collision_count = 0
 98          self.overlap_frames = 0
 99          self.contact_frames = 0
100          self.first_event_frame: int | None = None
101          self.last_event_frame: int | None = None
102          self.entered_total = 0
103          self.exited_total = 0
104          self.last_transition = ContactTransition(frame=-1)
105  
106      # ------------------------------------------------------------------
107      def update(self, hit_ids: Iterable[int] | None, frame: int) -> ContactTransition:
108          """Advance the state machine with the ids overlapping on ``frame``."""
109          current = {int(i) for i in (hit_ids or ())}
110          entered = current - self._active
111          exited = self._active - current
112          self._active = current
113  
114          if self.mode == "per_contact":
115              events = len(entered)
116          elif self.mode == "per_step":
117              events = 1 if (entered and self.contact_frames == 0) else 0
118          else:  # per_frame
119              events = 1 if current else 0
120  
121          self.collision_count += events
122          self.entered_total += len(entered)
123          self.exited_total += len(exited)
124          if current:
125              self.overlap_frames += 1
126              self.contact_frames += 1
127          else:
128              self.contact_frames = 0
129          if events:
130              self.first_event_frame = (
131                  frame if self.first_event_frame is None else self.first_event_frame
132              )
133              self.last_event_frame = frame
134  
135          self.last_transition = ContactTransition(
136              frame=int(frame),
137              entered=tuple(sorted(entered)),
138              exited=tuple(sorted(exited)),
139              active=tuple(sorted(current)),
140              events=int(events),
141              hit=bool(current),
142          )
143          return self.last_transition
```

`core/bullet_sim/collision/events.py:87-143`

逐行：
- L96：`_active` 是 Python `set[int]`（不是 numpy，弹幕数量级不需要向量化）。
- L109：`current` 由传入的 id 可迭代对象构造（`hit_ids or ()` 兼顾 `None` 与空元组）。
- L110-112：**集合差**（`-`）：`entered` = 本帧有、上帧没有；`exited` = 上帧有、本帧没有；然后状态直接替换为 `current`（所以已分离的 id 不会留在集合里，避免后续伪"重入"）。
- L114-119：三种模式的 `events`（§5.2）。
- L121-128：计数维护；注意 L124-128 的分支决定 `contact_frames` 是"加一"还是"归零"。
- L129-133：事件帧记录（`first` 只在为 `None` 时写，之后保持）。
- L135-142：构造冻结的 `ContactTransition`，三个元组都 `sorted(...)` 以便确定序列化；`hit = bool(current)`。

`ContactTransition`（events.py:56-84）字段：`frame, entered, exited, active, events, hit`，并有 `is_new_event`（`events > 0`）与 `to_dict()`。

其他成员：
- `COUNT_MODES = ("per_contact", "per_step", "per_frame")`（events.py:44）；`validate_count_mode`（events.py:47-53）做 `strip().lower()` 后查表，非法值抛 `ValueError`。
- 属性：`active_ids`（排序元组）、`in_contact`（`bool(self._active)`）、`active_count`（events.py:146-156）。
- `frames_since_event(frame)`：`frame − last_event_frame`，从未发生事件则 `None`（events.py:158-161）。
- `summary(frame=None, dt=0.0)`（events.py:163-182）返回 `collision_count / collision_count_mode / active_ids / in_contact / overlap_frames / entered_total / exited_total / first_event_frame / last_event_frame`，并可选追加 `frames_since_event` 与 `seconds_since_event`（= 帧差 × dt）。
- 调用方：`simulator/world.py:351` `contact = self.contacts.update(result.hit_ids, step_index)`（把 `CollisionResult.hit_ids` 整集交给状态机）。

---

# 6. `safety/free_space.py` + `safety/path_search.py` —— 配置空间膨胀与到达时间波前

## 6.1 它要解决什么问题

不是"现在形状是否重叠"，而是**"玩家的碰撞体放在哪些格子里不会碰到任何东西"**（配置空间，不是工作空间），以及"在给定时窗内玩家**实际能到达**哪里"。

## 6.2 数学与算法

### (a) 配置空间膨胀（free_space）

```
cell 中心 c 是自由的  ⟺  ∀ obstacle i : sdf_i(c) ≥ inflate
代码写法：blocked = any( sdf_i(c) < inflate )  然后 free = ¬blocked
```

**膨胀半径从哪来**（`safety/validate.py:235-237`）：

```
inflate = player_circumradius(hitbox) + margin
        = player_radius + 6.0            （圆 hitbox，margin 默认 6.0）
```

用与碰撞系统**同一套 SDF**（`obstacle_sdf_matrix`）所以是精确配置空间判定，而非包围盒近似（文件头 7-14 行）。

网格参数（`grid_shape`）：

```
res  = max(1.0, resolution)
cols = max(2, ceil(field_w / res))
rows = max(2, ceil(field_h / res))
cell_w = field_w / cols          ← 注意：格宽被重新均分，恰好铺满场地
cell_h = field_h / rows
```

格中心（`cell_centres`）：`xs = (arange(cols) + 0.5)·cell_w`、`ys = (arange(rows)+0.5)·cell_h`，`meshgrid` 后 `reshape(-1)`，顺序是**行主序**（与 `blocked.reshape(rows, cols)` 对应）。`rasterize_free_space` 里 `origin_x = origin_y = 0.0`。

### (b) 到达时间波前（path_search）—— 不是 BFS，也不是优先队列 Dijkstra

算法实质：**每格维护一个标量"最早到达时间" `arrival`，用 numpy 数组平移做多趟松弛（Bellman-Ford / 值迭代风格），每个时间切片再用 `arrival ≤ t_k` 过滤出该时刻的可达集合**。时间展开方式：

```
t_k = (k+1)·dt                  （第 k 个自由空间切片对应的时间）
step = max_speed · dt                                 （无加速度限制）
     = 0.5·(v0 + v_end)·dt,  v_end = min(max_speed, v0 + max_accel·dt)   （有加速度限制）
kernel = { (dr, dc, dist) : dist = hypot(dr·cell_h, dc·cell_w) ≤ step + 0.5·min(cell_w, cell_h) }
         ∪ {(0,0,0.0)}                            （allow_stay 默认 True）
n_passes = passes if passes is not None else max(2, ceil(step / min(cell_w, cell_h)) + 2)
```

单切片流程（真实代码见下）：

```
1. arrival = where(layer_k, arrival, inf)        ← 被占据的格清成 inf（不能当传播源）
2. usable = layer_k & (arrival <= t_k + 1e-9)    ← 该时刻可达集合
3. 重复 n_passes 次：relaxed = min over kernel ( shift(arrival, dr, dc) + dist/speed )
                     relaxed = where(layer_k, relaxed, inf)   ← 只能进入当前空闲格
                     若 relaxed < arrival − 1e-12 就更新（changed）；无变化则提前退出
4. current = layer_k & (arrival <= t_k + 1e-9)   ← 本切片结果；为空则判定"走廊在时窗前关闭"
```

时间推进的语义：`arrival` 的单位是**秒**，每走一个 kernel 偏移加 `dist / speed`。因为每格存的是"最早到达时间"而不是布尔值，**小于一格的位移不会丢**（文件头/函数注释 L217-221：6 px/切片 跨 16 px 的格，三片之后仍能到达邻格）。反之，布尔膨胀（`dilate_with_offsets`，path_search.py:135-139）在 `step < cell` 时会**完全冻住**，所以 `reachable_set` 不用它。

回溯（`backtrack_path`）：终点取**最后一层里离起点最远的可达格**（按格子距离 `hypot(Δr, Δc)`，注意这里乘的是 `1.0`，即**未按 cell_w/cell_h 加权**，而注释说的是 "in travel terms"）；然后从最后一层往前，每层取**离当前格最近的格**（`argmin(Δr²+Δc²)`），最后补上起点并反转。**没有父指针**，所以回溯出来的路径不保证就是松弛时用的那条链。路径每点的 clearance 由 `_cell_clearance`（到最近被堵格的**切比雪夫环距**，上限 `max_ring=6`）乘 `grid.cell_w` 得到（近似世界单位）。

## 6.3 真实代码：free_space

```python
# core/bullet_sim/safety/free_space.py:129-186
129  def grid_shape(field_w: float, field_h: float, resolution: float) -> tuple[int, int, float, float]:
130      res = max(1.0, float(resolution))
131      cols = max(2, int(np.ceil(field_w / res)))
132      rows = max(2, int(np.ceil(field_h / res)))
133      return rows, cols, field_w / cols, field_h / rows
134  
135  
136  def cell_centres(
137      rows: int, cols: int, cell_w: float, cell_h: float
138  ) -> tuple[np.ndarray, np.ndarray]:
139      xs = (np.arange(cols) + 0.5) * cell_w
140      ys = (np.arange(rows) + 0.5) * cell_h
141      gx, gy = np.meshgrid(xs, ys)
142      return gx.reshape(-1), gy.reshape(-1)
143  
144  
145  def occupancy_mask(
146      pool: BulletPool,
147      *,
148      rows: int,
149      cols: int,
150      cell_w: float,
151      cell_h: float,
152      inflate: float,
153      index: np.ndarray | None = None,
154  ) -> np.ndarray:
155      """``(rows, cols)`` mask of cells that are **not** traversable.
156  
157      Exact configuration-space inflation: a cell is blocked iff any obstacle's
158      surface is closer than ``inflate`` to the cell centre.
159      """
160      blocked = np.zeros(rows * cols, dtype=bool)
161      if index is None:
162          index = pool.active_indices()
163      if index.size == 0:
164          return blocked.reshape(rows, cols)
165      gx, gy = cell_centres(rows, cols, cell_w, cell_h)
166      d = pool.data
167      ox_all = d["x"][index].astype(np.float64)
168      oy_all = d["y"][index].astype(np.float64)
169      ext = np.hypot(
170          d["half_w"][index].astype(np.float64), d["half_h"][index].astype(np.float64)
171      )
172      # cheap prefilter: only obstacles whose bounding box can reach the grid
173      x0, x1 = gx.min() - float(inflate), gx.max() + float(inflate)
174      y0, y1 = gy.min() - float(inflate), gy.max() + float(inflate)
175      near = (ox_all + ext >= x0) & (ox_all - ext <= x1) & (oy_all + ext >= y0) & (oy_all - ext <= y1)
176      if near.any():
177          idx = index[near]
178          sdf = obstacle_sdf_matrix(
179              gx, gy,
180              d["x"][idx].astype(np.float64), d["y"][idx].astype(np.float64),
181              d["shape"][idx], d["radius"][idx].astype(np.float64),
182              d["half_w"][idx].astype(np.float64), d["half_h"][idx].astype(np.float64),
183              d["rotation"][idx].astype(np.float64),
184          )
185          blocked = np.any(sdf < float(inflate), axis=1)
186      return blocked.reshape(rows, cols)
```

`core/bullet_sim/safety/free_space.py:129-186`

逐行：
- L130-133：`res` 下界 1.0；`rows/cols` 下界 2；返回的 `cell_w/cell_h` 是**均分后的实际格宽高**（`field/cols`），所以 `cols·cell_w == field_w` 精确成立。
- L139-142：格中心；`meshgrid` 默认 `indexing="xy"`，`gx` 每行相同（沿 x 变），`gy` 每列相同；`.reshape(-1)` 展平后第 `r*cols+c` 个元素对应格 `(r,c)` —— 与后面 `blocked.reshape(rows, cols)` 一致。
- L160-164：先建全 False 的展平数组；取活体下标，空则直接返回（reshape 成 `(rows,cols)`）。
- L169-171：`ext = hypot(half_w, half_h)` = 障碍物的**外接圆半径**（矩形用半宽高、圆用 `radius = half_w = half_h`，见 `obstacles/spec.py:387-391` 的约定"a circle keeps half_w == half_h == radius"）。`ext ≥ radius` 恒成立，所以用它做预筛是保守的。
- L172-175：**廉价预筛**：只保留"外接圆与网格矩形（外扩 inflate）相交"的障碍物；`near` 是四个逐元素比较的与（`&`）。
- L176-185：对预筛后的障碍物调 `obstacle_sdf_matrix` 得到 `(cells, obstacles)` 的 SDF 矩阵，`np.any(sdf < inflate, axis=1)` —— **沿障碍物轴求"是否有一个更近"**，得到每格是否被堵。注意是**严格小于** `inflate`：恰好等于 `inflate` 的格算自由。
- L186：reshape 回 `(rows, cols)`。

```python
# core/bullet_sim/safety/free_space.py:189-209
189  def rasterize_free_space(
190      pool: BulletPool,
191      *,
192      field_w: float,
193      field_h: float,
194      inflate: float,
195      resolution: float = 16.0,
196      index: np.ndarray | None = None,
197  ) -> FreeSpaceGrid:
198      """Level-1 analysis: traversable cells for the *current* obstacle layout."""
199      rows, cols, cw, ch = grid_shape(field_w, field_h, resolution)
200      blocked = occupancy_mask(
201          pool, rows=rows, cols=cols, cell_w=cw, cell_h=ch,
202          inflate=inflate, index=index,
203      )
204      free = ~blocked
205      return FreeSpaceGrid(
206          free=free, rows=rows, cols=cols, cell_w=cw, cell_h=ch,
207          origin_x=0.0, origin_y=0.0, inflate=float(inflate),
208          occupied_fraction=float(blocked.mean()) if blocked.size else 0.0,
209      )
```

`core/bullet_sim/safety/free_space.py:189-209`

其他要点：
- `FreeSpaceGrid`（free_space.py:43-126）：`world_to_cell(x,y)` 用 `floor((x−origin)/cell)` 并 `clip` 到 `[0, rows−1]/[0, cols−1]`；`cell_to_world(r,c)` 返回格中心 `(origin_x+(c+0.5)·cell_w, origin_y+(r+0.5)·cell_h)`；`free_fraction = free.mean()`。
- `components(diagonal=True)`（free_space.py:85-105）：**显式 Python 栈的 DFS** 连通域标注，`offsets = NEIGHBOURS_8` 或 `NEIGHBOURS_4`（free_space.py:34-40：8 邻域含 `(0,0)` ——"including stay"，所以"留在原地"也是一个邻居；`labels` 里 0 表示被占）。`largest_component`（107-114）用 `np.bincount(labels.ravel())` 统计各连通域大小，把 0 号（占用）置 0 后取 `argmax`。
- `FreeSpaceSequence.intersection()`（234-241）：逐层 `out &= g.free` —— **每个采样时刻都自由**的格（"始终安全"）。
- `NEIGHBOURS_8` 的 `(0,0)` 会不会让 DFS 卡住？不会：入栈前判断 `not labels[rr, cc]`，`(0,0)` 映射到自己，已被标记，跳过。

## 6.4 真实代码：path_search

```python
# core/bullet_sim/safety/path_search.py:82-139
 82  def movement_kernel(
 83      radius: float,
 84      cell_w: float,
 85      cell_h: float,
 86      *,
 87      tol: float = 0.5,
 88      allow_stay: bool = True,
 89  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
 90      """Offsets reachable within the physical distance ``radius``.
 91  
 92      Returns ``(dr, dc, distance)``.  A half-cell tolerance lets sub-cell
 93      progress still reach the neighbouring cell *centre*; without it a slice
 94      shorter than one cell would freeze the flood fill entirely (a modelling
 95      artefact, not a physical limit).
 96      """
 97      radius = max(0.0, float(radius))
 98      slack = float(tol) * min(cell_w, cell_h)
 99      r_max = int(np.ceil(radius / max(cell_h, 1e-9))) + 1
100      c_max = int(np.ceil(radius / max(cell_w, 1e-9))) + 1
101      drs, dcs, dists = [], [], []
102      for dr in range(-r_max, r_max + 1):
103          for dc in range(-c_max, c_max + 1):
104              if dr == 0 and dc == 0:
105                  continue
106              d = float(np.hypot(dr * cell_h, dc * cell_w))
107              if d <= radius + slack:
108                  drs.append(dr)
109                  dcs.append(dc)
110                  dists.append(d)
111      if allow_stay:
112          drs.append(0)
113          dcs.append(0)
114          dists.append(0.0)
115      if not drs:
116          drs, dcs, dists = [0], [0], [0.0]
117      return (
118          np.asarray(drs, dtype=np.int64),
119          np.asarray(dcs, dtype=np.int64),
120          np.asarray(dists, dtype=np.float64),
121      )
122  
123  
124  def _shift_or(mask: np.ndarray, dr: int, dc: int) -> np.ndarray:
125      out = np.zeros_like(mask)
126      rows, cols = mask.shape
127      r0, r1 = max(0, dr), min(rows, rows + dr)
128      c0, c1 = max(0, dc), min(cols, cols + dc)
129      if r0 >= r1 or c0 >= c1:
130          return out
131      out[r0:r1, c0:c1] = mask[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
132      return out
133  
134  
135  def dilate_with_offsets(mask: np.ndarray, drs: np.ndarray, dcs: np.ndarray) -> np.ndarray:
136      out = np.zeros_like(mask)
137      for dr, dc in zip(drs.tolist(), dcs.tolist()):
138          out |= _shift_or(mask, int(dr), int(dc))
139      return out
```

`core/bullet_sim/safety/path_search.py:82-139`

逐行：
- L97-100：`slack = tol·min(cell_w,cell_h)`（默认**半格**）；扫描半径按格数向上取整各加 1（保证不漏掉刚好在边界上的偏移）。
- L102-110：枚举 `(dr,dc)`，`dist = hypot(dr·cell_h, dc·cell_w)`（注意 **dr 乘 cell_h、dc 乘 cell_w**：行方向的物理步长是 cell_h），只要 `dist ≤ radius + slack` 就收进核。所以核的形状是**各向异性**的椭圆（当 cell_w≠cell_h 时）。
- L111-116：`allow_stay` 追加 `(0,0,0.0)`；如果核为空（`radius` 极小）则退化为只"停留"。
- L124-132：`_shift_or` 是"把 mask 整体平移 (dr,dc)"的实现——`out[r0:r1, c0:c1] = mask[r0-dr:r1-dr, c0-dc:c1-dc]`，边界自动为 False（`out` 预置 0）。**语义**：`out[r,c] = mask[r-dr, c-dc]`，即"若 (r-dr,c-dc) 在集合里，则 (r,c) 也在"→ 把集合沿 +(dr,dc) 方向扩张。
- L135-139：`dilate_with_offsets` 把所有偏移的平移结果逐位或（`|=`，布尔数组的逐元素 OR）。**这个函数在包内没有被调用**（只在 path_search.py:135 定义、path_search.py:367 出现在 `__all__`），我全仓 grep 没有找到调用点；`reachable_set` 用的是带时间的松弛版本。

```python
# core/bullet_sim/safety/path_search.py:203-295
203  def reachable_set(
204      layers: Sequence[np.ndarray],
205      *,
206      start_cell: tuple[int, int],
207      dt: float,
208      max_speed: float,
209      cell_w: float,
210      cell_h: float,
211      max_accel: float | None = None,
212      initial_speed: float | None = None,
213      passes: int | None = None,
214  ) -> ReachabilityResult:
215      """Time-expanded reachability flood fill with kinematic limits.
216  
217      Propagates an **arrival time** per cell rather than a boolean, so progress
218      smaller than one grid cell is never lost: a player travelling 6 px per slice
219      across 16 px cells still reaches the neighbouring cell after three slices.
220      A boolean dilation would silently freeze, which is a discretisation bug and
221      would make feasible scenes look infeasible.
222      """
223      if not layers:
224          return ReachabilityResult(reason="no free-space layers", start_cell=start_cell)
225      rows, cols = layers[0].shape
226      r0, c0 = int(start_cell[0]), int(start_cell[1])
227      if not layers[0][r0, c0]:
228          return ReachabilityResult(
229              start_free=False, start_cell=start_cell,
230              reason="the player starts inside an inflated obstacle",
231          )
232  
233      speed = float(max_speed)
234      v0 = speed if initial_speed is None else min(float(initial_speed), speed)
235      # per-slice travel distance, honouring an acceleration limit
236      if max_accel is not None and np.isfinite(max_accel) and max_accel > 0.0:
237          v_end = min(speed, v0 + float(max_accel) * dt)
238          step = 0.5 * (v0 + v_end) * dt
239      else:
240          step = speed * dt
241      drs, dcs, dists = movement_kernel(step, cell_w, cell_h)
242      n_passes = passes if passes is not None else max(2, int(np.ceil(step / min(cell_w, cell_h))) + 2)
243  
244      arrival = np.full((rows, cols), np.inf, dtype=np.float64)
245      arrival[r0, c0] = 0.0
246      reachable_layers: list[np.ndarray] = []
247      anytime = np.zeros((rows, cols), dtype=bool)
248      times: list[float] = []
249  
250      for k, layer in enumerate(layers):
251          t_k = (k + 1) * float(dt)
252          # A cell that is blocked at t_k cannot be occupied *now*, so it must not
253          # act as a source for this slice.  Without this invalidation the wavefront
254          # keeps the finite arrival time a cell had in an earlier slice and leaks
255          # straight through a moving wall - accepting scenes that are lethal.
256          arrival = np.where(layer, arrival, np.inf)
257          # cells usable at t_k: free now, and already reached by then
258          usable = layer & (arrival <= t_k + 1e-9)
259          current = usable.copy()
260          for _ in range(int(n_passes)):
261              relaxed = arrival.copy()
262              for dr, dc, dist in zip(drs.tolist(), dcs.tolist(), dists.tolist()):
263                  cand = _shift_min(arrival, int(dr), int(dc)) + (dist / max(speed, 1e-9))
264                  np.minimum(relaxed, cand, out=relaxed)
265              # a cell may only be entered if it is free at this instant
266              relaxed = np.where(layer, relaxed, np.inf)
267              changed = relaxed < arrival - 1e-12
268              if not changed.any():
269                  break
270              arrival = np.where(changed, relaxed, arrival)
271              current = layer & (arrival <= t_k + 1e-9)
272              if not current.any():
273                  break
274          reachable_layers.append(current.copy())
275          anytime |= current
276          times.append(t_k)
277          if not current.any():
278              return ReachabilityResult(
279                  reachable_layers=reachable_layers, reachable_final=current,
280                  reachable_anytime=anytime, times=times,
281                  start_cell=start_cell, start_free=True,
282                  reason=f"the reachable set became empty after {k + 1} slice(s) "
283                         "- the corridor closes before the horizon",
284              )
285  
286      final = reachable_layers[-1] if reachable_layers else np.zeros((rows, cols), dtype=bool)
287      return ReachabilityResult(
288          reachable_layers=reachable_layers,
289          reachable_final=final,
290          reachable_anytime=anytime,
291          times=times,
292          start_cell=start_cell,
293          start_free=True,
294          reason="feasible path found" if final.any() else "no reachable cell at the horizon",
295      )
```

`core/bullet_sim/safety/path_search.py:203-295`

逐行：
- L223-224：无切片 → `reason="no free-space layers"`（`reachable_final` 保持 `None` → `feasible` 为 False）。
- L225-231：起点若在**膨胀后的障碍物里**（`layers[0][r0,c0]` 为假）直接返回 `start_free=False`，`reason="the player starts inside an inflated obstacle"`。
- L233-240：**每切片位移** `step`：无加速度限制时 `speed·dt`；有 `max_accel` 且有限为正时用匀加速的梯形位移 `0.5·(v0+v_end)·dt`，其中 `v_end = min(speed, v0 + max_accel·dt)`，`v0 = min(initial_speed, speed)`（默认 = `speed`）。
- L241：用 `step` 构造运动核。
- L242：`n_passes = max(2, ceil(step / min(cell_w,cell_h)) + 2)` —— 每个切片内最多做几趟松弛（至少要能跨过 `step` 对应的格数，+2 是余量，下限 2）。
- L244-245：`arrival` 全 `inf`，起点 = 0.0 秒。
- L250-251：遍历每个自由空间切片，`t_k = (k+1)·dt`。
- **L256**：`arrival = np.where(layer, arrival, np.inf)` —— **这就是"当前被占据的格不作为传播源"的实现**：被占据格的到达时间被**清成 `inf`**（不是"只把它从结果里剔除"），于是下一行的松弛在 `_shift_min` 读取"源格"时读到的是 `inf`，无法通过它传播；只有等该格重新变自由、并能从某个自由邻居重新到达时，才会重新获得有限值。L252-255 是代码自带的理由注释（原文照抄）。
- L258：`usable = layer & (arrival <= t_k + 1e-9)` —— 该时刻可用 = **当前自由** 且 **已在此之前到达**；`1e-9` 是浮点容差。
- L260-273：`n_passes` 趟松弛：
  - L261 `relaxed = arrival.copy()`（把当前值作为基底，`np.minimum(..., out=relaxed)` 逐偏移取小）；
  - L263 `cand = _shift_min(arrival, dr, dc) + dist/speed` —— `_shift_min` 的语义是 `out[r,c] = arr[r-dr, c-dc]`（边界 `+inf`，path_search.py:142-151），所以 `cand[r,c]` = "从 (r-dr,c-dc) 走一步到 (r,c) 的到达时间"；`dist/speed` 是这一步消耗的时间；`max(speed, 1e-9)` 防 0 除。
  - L266 `relaxed = np.where(layer, relaxed, inf)` —— **目标格必须在当前时刻自由**（被堵格永远拿不到有限值）；
  - L267-269 `changed = relaxed < arrival - 1e-12`，无变化就提前结束这一切片；
  - L270 `arrival = np.where(changed, relaxed, arrival)` —— 只写回真正变小的格（保持"最早"语义）；
  - L271-273 更新 `current`；若为空则提前结束（不再继续松弛）。
- L274-276：记录本切片可达集、把 `anytime` 或上本切片（`|=` 逐元素 OR）、记录时间。
- L277-284：本切片可达集为空 → 提前返回，`reason` 形如 `"the reachable set became empty after N slice(s) - the corridor closes before the horizon"`。
- L286-295：正常结束：`reachable_final` = 最后一层，`reason` 二选一。

`ReachabilityResult`（path_search.py:154-200）：`feasible = start_free and reachable_final is not None and reachable_final.any()`；`reachable_fraction = final.mean()`；`reachable_cells = final.sum()`；`safe_region()` 返回 `reachable_anytime`；`to_dict()` 给出 `feasible/start_free/reachable_cells/reachable_fraction/samples/horizon/reason/strategy`。

```python
# core/bullet_sim/safety/path_search.py:298-359
298  def backtrack_path(
299      reach: ReachabilityResult,
300      layers: Sequence[np.ndarray],
301      *,
302      dt: float,
303      target_cell: tuple[int, int] | None = None,
304      grid: FreeSpaceGrid | None = None,
305  ) -> SafePath | None:
306      """Recover one concrete trajectory from a successful reachability result."""
307      if not reach.feasible or not reach.reachable_layers:
308          return None
309      final = reach.reachable_final
310      rows, cols = final.shape
311      if target_cell is not None and final[target_cell[0], target_cell[1]]:
312          cur = (int(target_cell[0]), int(target_cell[1]))
313      else:
314          rr, cc = np.nonzero(final)
315          # The point of a "safe path" is to make progress: pick the reachable
316          # cell farthest (in travel terms) from the start, not the nearest one,
317          # which would trivially be the start cell itself.
318          r0, c0 = reach.start_cell
319          d = np.hypot((rr - r0) * 1.0, (cc - c0) * 1.0)
320          j = int(np.argmax(d))
321          cur = (int(rr[j]), int(cc[j]))
322  
323      path_cells = [cur]
324      for k in range(len(reach.reachable_layers) - 1, 0, -1):
325          prev_layer = reach.reachable_layers[k - 1]
326          pr, pc = np.nonzero(prev_layer)
327          if pr.size == 0:
328              break
329          d = (pr - cur[0]) ** 2 + (pc - cur[1]) ** 2
330          j = int(np.argmin(d))
331          cur = (int(pr[j]), int(pc[j]))
332          path_cells.append(cur)
333      path_cells.append(reach.start_cell)
334      path_cells.reverse()
335  
336      times = [i * dt for i in range(len(path_cells))]
337      scale = (grid.cell_w if grid is not None else 1.0)
338      clearances: list[float] = []
339      for i, (r, c) in enumerate(path_cells):
340          # path index 0 is the start (before any slice); index i>=1 sits in
341          # reachable layer i-1
342          layer = layers[min(max(i - 1, 0), len(layers) - 1)]
343          clearances.append(float(_cell_clearance(layer, r, c)) * scale)
344      return SafePath(
345          cells=path_cells, times=times, clearances=clearances, grid=grid,
346          strategy=reach.strategy,
347      )
348  
349  
350  def _cell_clearance(layer: np.ndarray, r: int, c: int, *, max_ring: int = 6) -> int:
351      """Chebyshev distance (in cells) from ``(r, c)`` to the nearest blocked cell."""
352      rows, cols = layer.shape
353      for ring in range(1, max_ring + 1):
354          r0, r1 = max(0, r - ring), min(rows, r + ring + 1)
355          c0, c1 = max(0, c - ring), min(cols, c + ring + 1)
356          window = layer[r0:r1, c0:c1]
357          if not window.all():
358              return ring
359      return max_ring
```

`core/bullet_sim/safety/path_search.py:298-359`

逐行：
- L307-308：不可行直接 `None`。
- L311-312：给了 `target_cell` 且它在最后一层可达，就用它当终点。
- L314-321：否则取最后一层里离起点**格子距离最大**的可达格（L319 的 `* 1.0` 表示**不按 cell 尺寸加权**，是格坐标系下的欧氏距离）。
- L323-334：逐层回溯：从第 k 层到第 k−1 层，取第 k−1 层中**离当前格最近**的格（L329 用平方距离避免开方），最后补 `start_cell`、再整体反转。注意注释 L315-317 说明"为什么要取最远"（否则退化成起点本身）。
- L336：`times[i] = i·dt`。
- L337-343：clearance = `_cell_clearance(layer, r, c) × grid.cell_w`；层号取 `i-1` 并夹到合法范围（L340-341 注释：路径下标 0 是起点（在任何切片之前），i≥1 落在第 i−1 层）。
- `_cell_clearance`（L350-359）：从 1 到 `max_ring=6` 逐环扩大**切比雪夫环**（矩形窗口 `[r±ring, c±ring]`），若窗口内出现任何被堵格（`not window.all()`，注意传进来的是 **free** 层）就返回该环号；全部 6 环都自由则返回 6（**上限 6 格**）。

## 6.5 时间窗是怎么被生产出来的（真实链路）

`safety/validate.py:246-279` `free_space_sequence`：克隆一个真实 `World`，**每 `sample_dt` 秒用真实物理前进一步**（`roll.step(0)`，即动作 `stay`），每个采样点都 `rasterize_free_space(...)` → `FreeSpaceSequence(grids, times, dt=sample_dt)`。所以层的动力学与模拟器完全同源，不是重写的模型。

`safety/validate.py:282-310` `compute_safe_region`：`seq = free_space_sequence(...)`；`grid = seq.grids[0]`（当前时刻）；起点 = `grid.world_to_cell(player.x, player.y)`；然后

```python
# core/bullet_sim/safety/validate.py:300-308
300      reach = reachable_set(
301          [g.free for g in seq.grids[1:]] or [grid.free],
302          start_cell=start,
303          dt=config.sample_dt,
304          max_speed=speed,
305          cell_w=grid.cell_w,
306          cell_h=grid.cell_h,
307          max_accel=config.max_accel,
308      )
```

注意 `seq.grids[1:]`：**第 0 层（当前时刻）被丢掉**，时间展开从 `grids[1]` 开始（`t_1 = 1·sample_dt`）。`speed = config.max_speed if not None else spec.player_speed`。回溯时 `layers` 也用 `grids[1:]`（validate.py:325）。

## 6.6 关键常数与默认值

| 名字 | 值 | 位置 | 含义 |
|---|---|---|---|
| `SafetyConfig.margin` | `6.0` | validate.py:64 | 膨胀额外余量（世界单位） |
| `SafetyConfig.horizon` | `2.0` s | validate.py:66 | 可行性必须保持的时长 |
| `SafetyConfig.resolution` | `None` → `max(8.0, player_radius/1.5)` | validate.py:68 / 240-243 | 栅格分辨率 |
| `SafetyConfig.sample_dt` | `0.05` s | validate.py:70 | 自由空间采样间隔 |
| `SafetyConfig.max_speed` / `max_accel` | `None` / `None` | validate.py:72-73 | 默认用场景玩家速度 / 不限加速度 |
| `min_free_fraction` / `min_reachable_cells` | `0.03` / `4` | validate.py:75-77 | 场景校验阈值 |
| `movement_kernel.tol` | `0.5` | path_search.py:87 | 半格松弛（`0.5·min(cell_w,cell_h)`） |
| `max_ring`（clearance） | `6` | path_search.py:350 | 切比雪夫环上限 = clearance 上限（格） |
| `clip_norm`（玩家速度限幅） | `core/numerics.py:107-112` | — | `‖v‖ > max_norm` 时按比例缩放 |
| `grid_shape` 下界 | `res ≥ 1.0`、`rows/cols ≥ 2` | free_space.py:130-132 | |
| 严格比较 | `sdf < inflate` | free_space.py:185 | 恰好等于 `inflate` 视为自由 |

---

# 7. `generators/patterns.py` 的 `wall_gap` 与 `corridor`

两者的公共部分：`gen_obstacle_field`（patterns.py:254-305）按 `params['layout']` 派发到 `_LAYOUT_BUILDERS`（patterns.py:553-559），每个 builder 返回

```
(offsets[N,2], angles[N], speeds[N], half_w[N], half_h[N], rotation[N], spin[N], aimed)
```

然后组装成一个 `SpawnEvent`（patterns.py:280-303）。所以**布局只决定初值**（出生点、朝向、速度、矩形半宽高、初始角、角速度），运动由 §1 的积分器负责。

## 7.1 公共几何工具

```python
# core/bullet_sim/generators/patterns.py:210-251
210  def _region_anchor(region: str, ctx: GenerationContext) -> tuple[float, float, float]:
211      """``(x, y, inward_angle_rad)`` for a spawn region keyword.
212  
213      Angles point *into* the field, so an obstacle spawned on the left edge
214      travels rightwards unless the caller overrides it.
215      """
216      w, h = ctx.field_w, ctx.field_h
217      table = {
218          "left": (0.0, h * 0.5, 0.0),
219          "right": (w, h * 0.5, _PI),
220          "bottom": (w * 0.5, 0.0, _PI / 2),
221          "top": (w * 0.5, h, -_PI / 2),
222      }
223      if region in ("farthest_side", "nearest_side"):
224          # pick the horizontal side the player is *not* near
225          px = float(ctx.player_xy[0])
226          near_left = px < w * 0.5
227          if region == "nearest_side":
228              near_left = not near_left
229          return table["left"] if near_left else table["right"]
230      return table.get(region, table["left"])
231  
232  
233  def _edge_point(
234      region: str, ctx: GenerationContext, t: float, *, outward: float = 0.0
235  ) -> tuple[float, float, float, float]:
236      """A point on the named boundary edge plus its inward unit normal.
237  
238      ``t`` slides along the edge; ``outward`` pushes the point *outside* the
239      field so the obstacle visibly enters instead of appearing on the border.
240      Keeping the edge coordinate (and only randomising along the edge) is what
241      guarantees "born on a boundary, never on the centre line".
242      """
243      x, y, inward = _region_anchor(region, ctx)
244      w, h = ctx.field_w, ctx.field_h
245      vertical_edge = abs(math.cos(inward)) >= 0.5      # left / right edge
246      if vertical_edge:
247          px, py = x, float(t) * h
248      else:
249          px, py = float(t) * w, y
250      nx, ny = math.cos(inward), math.sin(inward)
251      return px - nx * outward, py - ny * outward, nx, ny
```

`core/bullet_sim/generators/patterns.py:210-251`

- L217-222：四个边界锚点与**指向场内**的角度（left→0、right→π、bottom→+π/2、top→−π/2）。
- L223-229：`farthest_side`/`nearest_side` 按玩家 x 是否小于半个场地宽来选左右（注意二者都是**左右**边，不是上下）。
- L245-249：`t ∈ [0,1]` 沿边滑动（左右边沿 y，上下边沿 x）。
- L250-251：内法向 `(nx,ny)`；`outward` 沿法向**反**方向外推（`px - nx*outward`）。

常量（patterns.py:34-37）：`_TWO_PI = 2π`、`_PI = π`、`_DEG = π/180`、`_FULL_RING_EPS = 1e-9`。

## 7.2 `wall_gap`：带缺口的墙

### 几何（数学）

输入参数：`spawn`（边）、`entry_angle`（度）、`thickness`、`gap_width`、`gap_position ∈ [0,1]`、`gap_motion`、`gap_jitter`、`gap_sweep`、`gap_speed`。

```
θ      = entry_angle（度→弧度；默认 = degrees(内法向角)，所以 = 内法向）
axis   = "x" if |cos θ| < 0.5 else "y"        ← "横向跨过另一根轴"
span   = field_w if axis=="x" else field_h
half_gap = min(gap_width/2, span/2)
centre   = clip(gap_position·span, half_gap, span − half_gap)
两段 = [0, centre−half_gap] 与 [centre+half_gap, span]（各去掉长度 ≤1 的退化段）
每段 → 矩形：中点 mid = (a+b)/2、半长 seg_half = (b−a)/2
  axis=="x": 中心 (mid, y_anchor)，half_w = seg_half，half_h = thickness/2
  axis=="y": 中心 (x_anchor, mid)，half_w = thickness/2，half_h = seg_half
rotation = 0.0 (axis=="x") 或 π/2 (axis=="y")
速度方向 = θ，速度大小 = spec.speed
```

缺口扫动（`gap_motion == "sweep"`）：`gap_pos = 0.5 + amp·sin(2π·speed·elapsed) + U(−jitter, jitter)`，其中 `elapsed = (step − start_step)·dt`（patterns.py:272-274），所以是**按仿真时间**的正弦扫动，每个发射步重新计算。`gap_motion == "random"`：`gap_pos = U(0.2, 0.8)`。

### 真实代码

```python
# core/bullet_sim/generators/patterns.py:367-422
367  def _layout_wall_gap(spec, ctx, elapsed, step):
368      """A wall spanning the field with one opening; the opening may sweep."""
369      region = str(spec.params.get("spawn", "top"))
370      x, y, inward = _region_anchor(region, ctx)
371      theta = float(spec.params.get("entry_angle", np.degrees(inward))) * _DEG
372      axis = "x" if abs(math.cos(theta)) < 0.5 else "y"   # span across the other axis
373      span = ctx.field_w if axis == "x" else ctx.field_h
374      thickness = float(spec.params.get("thickness", 26.0))
375      gap_w = float(spec.params.get("gap_width", 90.0))
376      gap_pos = float(spec.params.get("gap_position", 0.5))
377      motion = str(spec.params.get("gap_motion", "static"))
378      jitter = float(spec.params.get("gap_jitter", 0.0))
379      if motion == "sweep":
380          amp = float(spec.params.get("gap_sweep", 0.35))
381          speed = float(spec.params.get("gap_speed", 0.6))
382          gap_pos = 0.5 + amp * math.sin(2.0 * math.pi * speed * elapsed)
383          gap_pos += ctx.rng.uniform(-jitter, jitter)
384      elif motion == "random":
385          gap_pos = float(ctx.rng.uniform(0.2, 0.8))
386      gap_pos = float(np.clip(gap_pos, 0.0, 1.0))
387      half_gap = min(gap_w / 2.0, span / 2.0)
388      centre = float(np.clip(gap_pos * span, half_gap, span - half_gap))
389      segs: list[tuple[float, float]] = []
390      if centre - half_gap > 1.0:
391          segs.append((0.0, centre - half_gap))
392      if centre + half_gap < span - 1.0:
393          segs.append((centre + half_gap, span))
394      offs, hws, hhs = [], [], []
395      for a, b in segs:
396          mid = 0.5 * (a + b)
397          seg_half = 0.5 * (b - a)
398          if axis == "x":
399              offs.append([mid, y])
400              hws.append(seg_half)
401              hhs.append(thickness / 2)
402          else:
403              offs.append([x, mid])
404              hws.append(thickness / 2)
405              hhs.append(seg_half)
406      n = len(offs)
407      if n == 0:
408          empty = np.zeros((0, 2))
409          return (empty, np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0),
410                  np.zeros(0), np.zeros(0), False)
411      # the wall travels inward, with its long axis perpendicular to the travel
412      rot = 0.0 if axis == "x" else _PI / 2
413      return (
414          np.array(offs, dtype=np.float64),
415          np.full(n, theta),
416          np.full(n, float(spec.speed)),
417          np.array(hws, dtype=np.float64),
418          np.array(hhs, dtype=np.float64),
419          np.full(n, rot),
420          np.full(n, float(spec.angular_velocity)),
421          False,
422      )
```

`core/bullet_sim/generators/patterns.py:367-422`

逐行：
- L369-371：边锚点 + 角度；`entry_angle` 默认取 `degrees(inward)` 再乘 `_DEG`（等于 `inward`），所以默认"朝场内飞"。
- L372：`axis` 判定：速度方向接近竖直（`|cos θ| < 0.5`）→ `axis="x"`（墙**横跨 x**，沿 x 铺开）；接近水平 → `axis="y"`。
- L373：`span` 取对应场地边长。
- L374-378：厚度默认 26（但 `resolve_spec` 会覆盖成 `max(8, 0.35·特征尺寸)`）、缺口默认 90、位置比例 0.5、默认 `static`、抖动 0。
- L379-383：扫动：振幅 0.35、频率 0.6 Hz 的**正弦**，再叠加 `±jitter` 均匀噪声（`ctx.rng` 是带种子的子流）。
- L384-385：随机缺口位置在 `[0.2, 0.8]`。
- L386-388：`gap_pos` 夹到 `[0,1]`；`half_gap = min(gap_w/2, span/2)`（缺口不可能超过整个跨度）；`centre = clip(gap_pos·span, half_gap, span−half_gap)`（保证缺口整体落在场内）。
- L389-393：两段墙的区间；`> 1.0` / `< span − 1.0` 是"退化段（长度 ≤ 1）不生成"的判断。
- L394-405：把区间变成矩形参数（§7.2 的公式）。
- L407-410：一段都没有（缺口覆盖全场）→ 返回长度 0 的数组，`gen_obstacle_field` 里 `n == 0` 会 `continue`（patterns.py:278-279）。
- L411-412：**`rot` 这一行**：`axis=="x"` 时 0，否则 π/2。
- L413-421：组装元组；注意 `angles` 全部 = θ（统一朝场内），`rotation` 全部 = `rot`，`spin` = `spec.angular_velocity`，`aimed=False`。

### 数值实例（我实测，`player_radius=10`、`field 640×480`、`gap_motion="static"`、默认 catalogue 参数）

| spawn | gap_width | thickness | 段数 | offsets | angles | half_w | half_h | rotation |
|---|---|---|---|---|---|---|---|---|
| `top`（默认） | 60.0 | 8.0 | 2 | (145, 480)、(495, 480) | −90° | 145.0 | 4.0 | 0.0 |
| `left` | 60.0 | 8.0 | 2 | (0, 105)、(0, 375) | 0° | 4.0 | 105.0 | **90°** |
| `right` | 60.0 | 8.0 | 2 | (640, 105)、(640, 375) | 180° | 4.0 | 105.0 | **90°** |

`top` 分支自洽：`half_w = seg_half = 145` 沿 x（`rot=0`），`half_h = thickness/2 = 4` 沿 y，两段覆盖 `x∈[0,290] ∪ [350,640]`，缺口 60 在 `x=290..350`，墙位于上边界 `y=480` 并朝 `−y` 进入场地 —— 与 SDF 实测一致（点 (320,470) 到两段表面的距离都是 30.59 = `hypot(290−320, 476−470)`）。

**`axis=="y"` 分支存在 90° 的不一致（我实测确认，供文档如实记录）**：该分支在 L403-405 把 `half_w = thickness/2`（本意是"厚度方向"）、`half_h = seg_half`（本意是"沿墙长度方向"），但 L412 又给了 `rot = π/2`。按 §3.2 的约定（`half_w` 沿 `rotation` 方向），旋转 90° 后 `thickness/2` 落到了**世界 y 轴**、`seg_half` 落到了**世界 x 轴**：于是 `left` 情形的两段其实是**两条水平短条**（`x∈[−105,105]`、`y∈[101,109]` 与 `y∈[371,379]`），而不是一堵沿 y 铺开、带 y 方向缺口的墙。用仓库自己的 `obstacle_sdf` 采样验证：点 (320,240) 到两段的 SDF 都是 251.77，正好等于到角点 (105,109) 的距离 `hypot(215,131)`；若按"竖墙"（`rot=0`）解释，最近点会是 (4,210)，距离 `hypot(316,30)=317.4`，与实测不符。同目录的 `corridor` 布局在同一约定下是自洽的（`rot = θ`，`half_w = wall_len/2` 沿通道方向；见 tests/test_patterns.py:334-340 断言 `rot==−π/2` 且 `half_w > half_h`），所以这更像是该分支的 `rot` 赋值与 `hws/hhs` 赋值不匹配，而不是全局约定不同。触发条件是**速度方向接近水平**：`spawn in {left, right}`，或 `farthest_side/nearest_side` 选中左右边，或显式 `entry_angle≈0/180`。仓库里的 `wall_gap` 断言只覆盖默认 `spawn="top"`（`tests/test_patterns.py:153-162`）。

### `wall_gap` 的默认参数（catalogue + resolve_spec）

| 名字 | 值 | 位置 |
|---|---|---|
| `spawn` | `"top"` | obstacles/catalog.py:80 |
| `gap_ratio` | `3.0` → `gap_width = 3.0 × 玩家直径` | catalog.py:81 / obstacles/spec.py:278-288 |
| `gap_position` | `0.5` | catalog.py:83 |
| `gap_motion` | `"sweep"` | catalog.py:84 |
| `thickness` | `max(8.0, 0.35 × 特征尺寸)` | obstacles/spec.py:263-265 |
| `shape` / `safety_strategy` | `"rect"` / `"gap"` | catalog.py:77-78 |
| 缺口下限 | `player_d`（显式更小直接 `ConfigError`） | obstacles/spec.py:277-288 |
| `patterns` 内部默认 | `thickness=26.0, gap_width=90.0, gap_position=0.5, gap_motion="static", gap_jitter=0.0` | patterns.py:374-378 |
| sweep 默认 | `gap_sweep=0.35`、`gap_speed=0.6` | patterns.py:380-381 |
| 退化段阈值 | 段长 `> 1.0` / `< span−1.0` | patterns.py:390-392 |
| 墙速度 | `spec.speed`（catalogue 默认 `90.0`，obstacles/spec.py:373） | patterns.py:416 |

## 7.3 `corridor`：两堵关于玩家对称的墙

### 几何（数学）

```
初值（t=0）：
  c        = 玩家位置（ctx.player_xy）
  θ        = (entry_angle 若给定，否则边内法向) + tilt_deg
  d        = (cos θ, sin θ)              通道方向（也是墙的 long 方向）
  nrm      = (−d_y, d_x)                 通道法向
  width    = corridor_width              两堵墙**内表面**之间的净宽
  wall_t   = wall_thickness              墙厚
  对 side ∈ (−1, +1)（walls=1 时只有 (+1)）：
      centre_side = c + nrm · side · (width/2 + wall_t/2)
      half_w      = wall_len/2            （沿 θ）
      half_h      = wall_t/2              （沿 θ+90°）
      rotation    = θ
```

三种变化（`change` 是**整段 episode 的总量**，按 `duration` 摊成恒速率）：

```
open        ：每堵墙沿 ±nrm 各移动 speed = 0.5·change·player_d / duration，方向相反 → 宽度变化 = change·player_d
close       ：同上，方向相反 → 宽度减少 change·player_d
rotate_same ：spin = radians(change)/duration（两堵墙同号）→ 通道整体倾斜，宽度不变
rotate_opposite：spin = side · radians(change)/duration（两侧反号）→ 剪刀口
static      ：spin = 0、speed = 0
```

注意 `open/close` 的 `angles` 行（patterns.py:536）：`math.atan2(v[1], v[0]) if s > 0 else 0.0` —— 速度为 0 时角度写 0（速度为 0 时方向无意义）。

限角/限幅在 `resolve_spec`（`obstacles/spec.py:344-367`）里完成，**在进生成器之前**：

```
close           : change = clip(change, 0, (width − min_gap) / player_d)
open            : change = clip(change, 0, (0.45·min(field_w,field_h) − width) / player_d)
rotate_same     : change = clip(change, 0, CORRIDOR_MAX_SAME_ANGLE = 45.0)
rotate_opposite : a = wall_len/2; max_tan = (width − min_gap) / (2a); change = clip(change, 0, degrees(atan(max_tan)))
min_gap         = max(floor, CORRIDOR_GAP_MARGIN · player_d)，floor 默认 CORRIDOR_GAP_MARGIN·player_d
CORRIDOR_GAP_MARGIN = 1.25
```

`rotate_opposite` 的公式来自几何：两堵半长 `a` 的墙绕各自中心反向转 θ 时，墙端的净间距是 `width − 2a·tanθ`（obstacles/spec.py:356-359 的注释原文），要求它 ≥ `min_gap` 就得到 `tanθ ≤ (width−min_gap)/(2a)`。

### 真实代码

```python
# core/bullet_sim/generators/patterns.py:499-549
499      params = spec.params
500      cx, cy = float(ctx.player_xy[0]), float(ctx.player_xy[1])
501      region = str(params.get("spawn", "left"))
502      _, _, inward = _region_anchor(region, ctx)
503      tilt = float(params.get("tilt_deg", 0.0)) * _DEG
504      explicit = params.get("entry_angle")
505      theta = (float(explicit) * _DEG if explicit is not None else inward) + tilt
506      d = np.array([math.cos(theta), math.sin(theta)], dtype=np.float64)
507      nrm = np.array([-d[1], d[0]], dtype=np.float64)
508  
509      width = float(params.get("corridor_width", 140.0))
510      wall_t = float(params.get("wall_thickness", 20.0))
511      wall_len = float(params.get("wall_length", 6.0 * 2.0 * 20.0))
512      walls = int(params.get("walls", 2))
513      motion = str(params.get("motion", "static"))
514      change = float(params.get("change", 0.0))
515      duration = max(float(getattr(ctx, "duration", 1.0) or 1.0), 1e-6)
516      # the true player diameter is carried on the spec by ``resolve_spec``
517      player_d = float(params.get("player_diameter", 20.0))
518  
519      sides = (1.0,) if walls == 1 else (-1.0, 1.0)
520      offs, spins = [], []
521      for side in sides:
522          offs.append(np.array([cx, cy]) + nrm * (side * (width / 2.0 + wall_t / 2.0)))
523          if motion == "rotate_same":
524              spins.append(math.radians(change) / duration)
525          elif motion == "rotate_opposite":
526              spins.append(side * math.radians(change) / duration)
527          else:
528              spins.append(0.0)
529      n = len(offs)
530      if motion in ("open", "close"):
531          sign = 1.0 if motion == "open" else -1.0
532          # each wall moves half of the width change, in opposite directions
533          speed = 0.5 * change * player_d / duration
534          vel = [nrm * (side * sign * speed) for side in sides]
535          speeds = [float(np.hypot(v[0], v[1])) for v in vel]
536          angles = [math.atan2(v[1], v[0]) if s > 0 else 0.0 for v, s in zip(vel, speeds)]
537      else:
538          speeds = [0.0] * n
539          angles = [0.0] * n
540      return (
541          np.array(offs, dtype=np.float64),
542          np.array(angles, dtype=np.float64),
543          np.array(speeds, dtype=np.float64),
544          np.full(n, wall_len / 2.0),
545          np.full(n, wall_t / 2.0),
546          np.full(n, theta),
547          np.array(spins, dtype=np.float64),
548          False,
549      )
```

`core/bullet_sim/generators/patterns.py:499-549`

逐行：
- L500-502：`c` = **玩家位置**（不是场中心）；`inward` 只用来取默认朝向（`_` 丢弃锚点坐标）。
- L503-505：`θ = (显式 entry_angle 或 边内法向) + tilt_deg`（tilt 让整条通道倾斜）。
- L506-507：通道方向 `d`、法向 `nrm = (−d_y, d_x)`（`d` 逆时针转 90°）。
- L509-512：`width`（默认 140）、`wall_t`（20）、`wall_len`（默认 `6·2·20 = 240`，但 `resolve_spec` 通常覆盖为 `1.1×场地边长`）、`walls`（1 或 2）。
- L515：`duration = max(ctx.duration, 1e-6)` —— 把总量摊成速率的除数（防 0）。
- L517：`player_d` 由 `resolve_spec` 塞进 params（`obstacles/spec.py:371`：`params["player_diameter"] = player_d`），因为 `open/close` 的 `change` 单位是**玩家直径**。
- L519：`walls == 1` → 只有 `side = +1` 一堵墙。
- L522：`centre = c + nrm·side·(width/2 + wall_t/2)` —— 内表面正好落在 `c ± nrm·width/2`（因为再加半个墙厚）。
- L523-528：`spin` 三种：同向 `radians(change)/duration`、反向带 `side` 符号、其余 0。**注意 `rot` 初值（L546）永远是 θ**，旋转由积分器在 `rotation` 上累加（§1）。
- L530-539：`open/close` 的速度：`speed = 0.5·change·player_d/duration`（每堵墙负责一半的宽度变化），方向 `nrm·side·sign`（`sign` = +1 开、−1 关）；`speeds` 取模长、`angles` 取方向（速度为 0 时给 0）。
- L540-549：返回值顺序 = `(offsets, angles, speeds, half_w, half_h, rotation, spin, aimed=False)`，其中 `half_w = wall_len/2`（沿 θ）、`half_h = wall_t/2`（沿 θ+90°）。

走廊的发射调度（`obstacles/spec.py:377-385`）：`layout == "corridor"` 时若没显式给 `repetitions` 就设成 **1**、没给 `ttl` 就设成 `inf`（注释：走廊是**一对持久**的墙，按 interval 反复发射会把多对墙叠在一起、各自旋转相位不同）。

### 数值实例（我实测：`player_radius=10`、field 640×480、默认 `spawn="left"`、`wall_length = 6×player_d`、`gen_ctx(player_xy=(320,240))`）

| 参数 | 结果 |
|---|---|
| `corridor_width`（默认 `corridor_ratio=4.0`） | 80.0（= 4 × 玩家直径 20） |
| `wall_thickness`（`max(8, 0.4×特征尺寸)`，size 1.5 → 特征 30） | 12.0 |
| `wall_length`（未显式给时 `1.1 × 640`） | 704.0 |
| `min_player_distance` | 0.0（`resolve_spec` 特意置 0：墙本来就贴着玩家） |
| 两墙 centre（θ=0、nrm=(0,1)） | (320, 194)、(320, 286) → 内表面 y = 200/280，净宽 80 |
| `half_w / half_h / rotation` | 352 / 6 / 0.0 |
| `spin`（`rotate_same`, change=30, duration=5） | 0.10472 rad/s（= radians(30)/5） |
| `spin`（`rotate_opposite`, change=80 → 被夹到 4.467） | ∓0.01559 rad/s |
| `close`, change=99 → 夹到 2.75 | 每墙 speed = 0.5·2.75·20/5 = 5.5 px/s |
| 夹取公式核对 | `close`：`(80 − 1.25·20)/20 = 2.75`；`rotate_opposite`：`degrees(atan((80−25)/(2·352))) = 4.467°` |

### `corridor` 的默认参数

| 名字 | 值 | 位置 |
|---|---|---|
| `spawn` / `size` | `"left"` / `1.5` | catalog.py:146-147 |
| `corridor_ratio` / `corridor_min_ratio` | `4.0` / `1.25` | catalog.py:148-149 |
| `walls` | `2` | catalog.py:150 |
| `motion` / `change` | `"static"` / `0.0` | catalog.py:151-152 |
| `wall_length_ratio` / `tilt_deg` | `None`（跨全场）/ `0.0` | catalog.py:154-155 |
| `CORRIDOR_MOTIONS` | `static, open, close, rotate_same, rotate_opposite` | obstacles/spec.py:44-50 |
| `CORRIDOR_GAP_MARGIN` | `1.25` | obstacles/spec.py:53 |
| `CORRIDOR_MAX_SAME_ANGLE` | `45.0` | obstacles/spec.py:56 |
| `CORRIDOR_SPAN_FACTOR` | `1.1` | obstacles/spec.py:60 |
| 生成器内部默认 | `corridor_width=140.0, wall_thickness=20.0, wall_length=240.0, walls=2, motion="static", change=0.0, tilt_deg=0.0, player_diameter=20.0` | patterns.py:509-517 |
| `min_player_distance` | 强制 0 | obstacles/spec.py:370 |

---

# 8. `core/rng.py` + `core/numerics.py`（含 `WorldSnapshot.state_hash`）

## 8.1 种子派生：主种子 → 命名子流

### 算法

```
derive_seed(master_seed, stream):
    h = blake2b(digest_size=8)                      ← 8 字节摘要
    h.update(master_seed.to_bytes(8, "little", signed=False))   ← 8 字节小端
    h.update(b"\x00")                               ← 1 字节分隔
    h.update(stream.encode("utf-8"))                ← 名字的 UTF-8 字节
    return int.from_bytes(h.digest(), "little")     ← 8 字节小端 → 64 位整数
```

字节布局固定为 `[8B 小端种子][0x00][UTF-8 名字]`（8 字节定长本身已无歧义，那 1 字节是代码实打实写进去的）。子流名与其它命名空间拼接：`spawn(child)` 用 `f"child:{child}"`。派生出的 64 位整数喂给 `np.random.PCG64(seed)`。

### 真实代码

```python
# core/bullet_sim/core/rng.py:19-58
19  _MASK64 = 0xFFFFFFFFFFFFFFFF
20  
21  
22  def derive_seed(master_seed: int, stream: str) -> int:
23      """Deterministically derive a 64-bit sub-seed from ``master_seed`` + name."""
24      h = hashlib.blake2b(digest_size=8)
25      h.update(int(master_seed).to_bytes(8, "little", signed=False))
26      h.update(b"\x00")
27      h.update(stream.encode("utf-8"))
28      return int.from_bytes(h.digest(), "little")
29  
30  
31  class SeedManager:
32      """Owns the master seed and hands out named, reproducible generators."""
33  
34      __slots__ = ("_master", "_streams")
35  
36      def __init__(self, master_seed: int) -> None:
37          self._master = int(master_seed) & _MASK64
38          self._streams: Dict[str, np.random.Generator] = {}
39  
40      @property
41      def master_seed(self) -> int:
42          return self._master
43  
44      def generator(self, stream: str) -> np.random.Generator:
45          """Return (and cache) the generator for a named stream."""
46          gen = self._streams.get(stream)
47          if gen is None:
48              gen = np.random.Generator(np.random.PCG64(derive_seed(self._master, stream)))
49              self._streams[stream] = gen
50          return gen
51  
52      def reset_stream(self, stream: str) -> None:
53          """Drop a cached stream so the next access restarts it deterministically."""
54          self._streams.pop(stream, None)
55  
56      def spawn(self, child: str) -> "SeedManager":
57          """Derive an independent child manager (for parallel scenarios)."""
58          return SeedManager(derive_seed(self._master, f"child:{child}"))
```

`core/bullet_sim/core/rng.py:19-58`

**逐位解释两处位运算**：

- **L19 / L37 `& _MASK64`（`0xFFFFFFFFFFFFFFFF`）**：这是一个 64 位掩码 —— 二进制是 64 个 1。`x & 0xFFFF...F` 只保留 `x` 的低 64 位，等价于 `x mod 2^64`（对非负 `x` 是截断；对负的 Python 整数，Python 的按位与使用无限位补码语义，结果等于该负数的 64 位无符号补码表示，例如 `-1 & _MASK64 == 2**64-1`）。所以 `SeedManager` 接受任意整数主种子并规范化成 64 位。
- **L25 `.to_bytes(8, "little", signed=False)`**：把一个整数序列化成**8 字节、小端、无符号**。小端 = 最低有效字节在前；`signed=False` 表示负数和 ≥2^64 的数会抛 `OverflowError`（`int.to_bytes` 的行为）。**因此直接调用 `derive_seed(-1, "x")` 或 `derive_seed(2**64, "x")` 会抛异常**；但经由 `SeedManager`（先 `& _MASK64`）永远不会越界 —— 这是我按代码推导的输入域限制，注释里没有写。
- **L28 `int.from_bytes(h.digest(), "little")`**：把 8 字节摘要按小端读回整数（与写入端同一个字节序约定）。
- L24 `digest_size=8`：blake2b 输出正好 8 字节 = 64 位子种子。
- L46-50：**按名字缓存** generator（同一 stream 名只构造一次；因为生成器是有状态的，缓存决定了"同一 stream 的多次调用是连续的序列"）。
- L52-54：`reset_stream` 通过 `pop` 让下次访问**重新派生**，从而回到流的起点（确定性重启）。
- L56-58：`spawn` 命名空间是 `"child:<name>"`，所以子管理器的流名不会与父层的同名流冲突。
- `snapshot()` / `restore()`（rng.py:60-71）：用 `gen.bit_generator.state`（PCG64 的完整内部状态字典）做可序列化的克隆/恢复；`restore` 对每个名字先 `self.generator(name)`（必要时新建）再赋 `state`。
- `__repr__`（73-74）：`SeedManager(master_seed=..., streams=[...])`。

## 8.2 规范化哈希：`canonical_hash` / `_feed`

```python
# core/bullet_sim/core/numerics.py:59-83
59  _CHUNK = 1 << 20
60  
61  
62  def _feed(hasher: "hashlib._Hash", arr: np.ndarray, dtype: np.dtype) -> None:
63      """Feed an array to the hasher in a canonical (C-contiguous, fixed dtype) form."""
64      a = np.ascontiguousarray(arr, dtype=dtype)
65      flat = a.reshape(-1)
66      # Chunked to bound peak memory for very large arrays.
67      if flat.nbytes <= _CHUNK:
68          hasher.update(flat.tobytes())
69          return
70      for start in range(0, flat.size, _CHUNK // flat.dtype.itemsize):
71          hasher.update(flat[start : start + _CHUNK // flat.dtype.itemsize].tobytes())
72  
73  
74  def canonical_hash(parts: Iterable[tuple[np.ndarray, np.dtype]]) -> str:
75      """Order-sensitive hash over ``(array, dtype)`` pairs -> hex digest.
76  
77      Arrays are cast to an explicit dtype first so that the digest does not
78      depend on memory layout or endianness of the caller's arrays.
79      """
80      hasher = hashlib.blake2b(digest_size=16)
81      for arr, dtype in parts:
82          _feed(hasher, arr, dtype)
83      return hasher.hexdigest()
```

`core/bullet_sim/core/numerics.py:59-83`

- **L59 `_CHUNK = 1 << 20`**：位运算解释 —— `1 << 20` = 2²⁰ = 1048576 字节 = 1 MiB。
- L64：`np.ascontiguousarray(arr, dtype=dtype)` —— 强制 **C 连续** 且**固定 dtype**（消除调用方数组的内存布局差异；dtype 也统一，避免 int32/int64 混用导致字节数不同）。
- L65：`reshape(-1)` 展平（对 C 连续数组是视图）。
- L67-68：总字节 ≤ 1 MiB 就一次 `tobytes()` 喂进去。
- L70-71：否则分块：步长是 **元素个数** `_CHUNK // flat.dtype.itemsize`（1 MiB ÷ 每元素字节数）。例如 float64 → 每块 131072 个元素。`tobytes()` 输出的是**本机字节序**（x86 小端）；代码里没有显式指定字节序，所以严格说"跨大小端平台仍一致"这一条注释只在同字节序平台族内成立（我只按代码事实指出这一点）。
- L80-83：`blake2b(digest_size=16)` → 16 字节摘要 → `hexdigest()` = **32 个十六进制字符**（`tests/test_env_interface.py:150-157` 断言 `len(state_hash()) == 32`）。哈希是**顺序敏感**的：`parts` 的连接顺序就是喂入顺序，且**各段之间没有长度前缀或分隔符**（定长段 + 末尾的弹幕块使连接无歧义）。

## 8.3 `state_hash()` 到底哈希了哪些字节

`WorldSnapshot.state_hash`（`core/state.py:156-177`）：

```python
# core/bullet_sim/core/state.py:156-177
156      def state_hash(self) -> str:
157          """Canonical digest of the *simulation-relevant* state.
158  
159          Only live bullets are hashed, in ascending slot order, so that the
160          digest is invariant to dead-slot bookkeeping and to the pool capacity.
161          """
162          b = self.bullets
163          idx = b.active_indices()
164          parts: list[tuple[np.ndarray, np.dtype]] = [
165              (np.array([self.env.step_index], dtype=INT_DTYPE), INT_DTYPE),
166              (np.array([self.timestamp, self.env.sim_time], dtype=np.float64), np.dtype(np.float64)),
167              (self.player_array(np.float64), np.dtype(np.float64)),
168              (self.target_array(np.float64), np.dtype(np.float64)),
169              (self.env_array(np.float64), np.dtype(np.float64)),
170              (np.array([idx.size], dtype=INT_DTYPE), INT_DTYPE),
171          ]
172          if idx.size:
173              mat = np.empty((idx.size, len(PROTOCOL_FIELDS)), dtype=np.float64)
174              for j, name in enumerate(PROTOCOL_FIELDS):
175                  mat[:, j] = b.data[name][idx]
176              parts.append((mat, np.dtype(np.float64)))
177          return canonical_hash(parts)
```

`core/bullet_sim/core/state.py:156-177`

### 喂给 blake2b 的字节序列（按顺序，我在同一节里给出每段的元素与字节数）

| # | 内容 | dtype | 元素数 | 字节数 | 来源 |
|---|---|---|---|---|---|
| 1 | `[step_index]` | int32 | 1 | 4 | `env.step_index`（`core/state.py:45`） |
| 2 | `[timestamp, env.sim_time]` | float64 | 2 | 16 | 先 `timestamp` 后 `sim_time` |
| 3 | 玩家向量 | float64 | 7 | 56 | `[x, y, vx, vy, radius, speed, 1.0 if alive else 0.0]`（`entities/player.py:66-72`） |
| 4 | 目标向量 | float64 | 6 | 48 | `[x, y, radius, shape_code, half_w, half_h]`（`entities/target.py:21-35`、`SHAPE_CODES={"circle":0.0,"point":0.0,"rect":1.0}`） |
| 5 | 环境向量 | float64 | 7 | 56 | `ENV_FIELDS = (field_w, field_h, dt, step_index, sim_time, wrap, bullet_budget)`；`wrap` 存的是 `WRAP_MODES.index(wrap)` 的浮点（`core/state.py:23-32, 50-62`） |
| 6 | `[活跃弹幕数量 idx.size]` | int32 | 1 | 4 | 数量本身也进哈希（分段边界） |
| 7 | 弹幕矩阵 `mat` | float64 | `N × 17` | `136·N` | `PROTOCOL_FIELDS` 顺序，只取 `idx = active_indices()`（**升序槽位**） |

`PROTOCOL_FIELDS`（`entities/bullet.py:96-122`，v3，17 项）：

```
x, y, vx, vy, ax, ay, angle, angular_velocity, radius, ttl, type_id, group_id, id,
shape, half_w, half_h, rotation
```

`INT_DTYPE = np.dtype(np.int32)`（`numerics.py:30`）。

**哪些东西不在哈希里**（正文没写、按代码可确认）：`age` 字段、已死槽位、`capacity`、`scenario_id`、`seed`、`rng_state`、`BulletPool.next_id`/空闲链表。`WorldSnapshot.__eq__` 就是 `state_hash()` 相等（`core/state.py:179-182`），所以这些量不参与相等判定（`tests/test_determinism.py:87-101` 正是断言"死槽位与容量不影响 state_hash"）。

### 同文件其它常量

- `FLOAT64 / FLOAT32 / DEFAULT_FLOAT_DTYPE = FLOAT64`、`_FLOAT_DTYPES` 别名表、`float_dtype()`（numerics.py:20-52）。
- `INT_DTYPE = int32`、`TYPE_DTYPE = int16`（numerics.py:30-32）。
- `rng_generator(seed)`（numerics.py:91-95）：

```python
# core/bullet_sim/core/numerics.py:91-95
91  def rng_generator(seed: int | None) -> np.random.Generator:
92      """Create an explicitly seeded PCG64 generator (never the global RNG)."""
93      if seed is None:
94          raise ValueError("seed must be an explicit integer for reproducibility")
95      return np.random.Generator(np.random.PCG64(int(seed) & 0xFFFFFFFFFFFFFFFF))
```

`seed=None` 直接抛异常（强制显式种子）；`& 0xFFFFFFFFFFFFFFFF` 与 §8.1 是同一个 64 位掩码（保留低 64 位 / 负数取补码）。

- `as_c_contiguous(arr, dtype=None)`（98-104）：需要才拷贝；`clip_norm(vec, max_norm)`（107-112）：`‖v‖ > max_norm` 时按 `max_norm/‖v‖` 等比缩放（`max_norm` 非正或 `‖v‖=0` 时不动作）。

---

# 9. 关键常数/默认值总表（速查）

| 模块 | 常数 | 值 |
|---|---|---|
| motion | `_OMEGA_EPS` / 内层 `small` 阈值 / 角度回绕 | `1e-12` / `1e-8` / `(−π, π]` |
| relative | 防 0 除 / `moving` 阈值 / 默认 radius | `1e-9` / `a > 1e-12` / `0.0` |
| shapes | 默认玩家半径 / 默认矩形半宽高 | `3.0` / `4.0, 4.0` |
| base | `DEFAULT_RISK_RADIUS` | `32.0` |
| obstacles | 矩形玩家外接圆 / contact_rate 防 0 | `hypot(half_w, half_h)` / `1e-9` |
| grid | 自适应格宽 / 半径重建阈值 / 单查询路径 | `max(4·max_r, 1.0)` / `>1.5·_max_r` / 暴力 |
| events | 默认模式 / 模式集合 | `per_contact` / `per_contact, per_step, per_frame` |
| free_space | `NEIGHBOURS_8`（含 `(0,0)`）/ `NEIGHBOURS_4` / 严格比较 | 9 元素 / 5 元素 / `sdf < inflate` |
| validate | margin / horizon / sample_dt / resolution | `6.0` / `2.0 s` / `0.05 s` / `max(8, r/1.5)` |
| validate | min_free_fraction / min_reachable_cells / max_window | `0.03` / `4` / `8.0 s` |
| path_search | kernel `tol` / `max_ring` / `n_passes` | `0.5` 格 / `6` / `max(2, ceil(step/min(cw,ch))+2)` |
| patterns(wall_gap) | gap_ratio / gap_position / gap_motion / sweep(amp,speed) / 退化段 | `3.0` / `0.5` / `sweep` / `0.35, 0.6` / `>1.0` |
| patterns(corridor) | corridor_ratio / min_ratio / wall_length / span factor | `4.0` / `1.25` / `1.1×场边长` / `1.1` |
| obstacles/spec | `CORRIDOR_GAP_MARGIN` / `CORRIDOR_MAX_SAME_ANGLE` | `1.25` / `45.0°` |
| rng | `_MASK64` / digest_size / PCG64 | `0xFFFFFFFFFFFFFFFF` / 8 B / `np.random.PCG64` |
| numerics | `_CHUNK` / `canonical_hash` digest / `INT_DTYPE` | `1<<20` (1 MiB) / blake2b-16 → 32 hex / int32 |

---

# 10. 实测验证记录（我实际运行的实验）

**R1 —— 弧积分闭式 & 两条直线公式的差异**（`integrate_bullets`，`dt=1/120`，`v=(100,0)`, `a=60`, `ω=2`）：

- 只有一条纯旋转弹（`a=0, ω=2`）：实际位移 `(0.833294753622253, 0.006944283694904074)`，与闭式 `(sin(ωdt)/ω·100, (1−cos(ωdt))/ω·100)` **完全相等** → §1.2(b) 的弧积分确实是闭式解。
- 一条 `a=60` 的直线弹：**全直线批**里位移 `x = 0.8354166666666667`（= `v·dt + 0.5·a·dt²`）；**同一批里加一条曲线弹**后同一条弹的位移变成 `x = 0.8375`（= `(v+a·dt)·dt` = `v·dt + a·dt²`）。差值恰为 `0.5·a·dt² = 0.002083…` → 确认 §1.3 L85-88 与 L75 用的是两条不同公式，选择取决于批内是否有曲线弹。

**R2 —— `time_to_impact` 的 `else` 分支**：按 §2.3 的不等式推导（`a>1e-12` 且 `c≤0` ⟹ `disc≥b²≥0` ⟹ `hit` 为真），`out[moving & (c <= 0.0)] = 0.0` 这一行不可达；我没有构造出能触发它的输入（`tests/` 里也没有针对该分支的用例）。

**R3 —— 波前的"被占据格不作传播源"**：把 `reachable_set` 抄一份、**只删掉 L256 那一行**，在 1×6 网格、`cell=4px`、`speed=200`、`dt=0.05`（每切片 8 px）、切片 0 全自由 / 切片 1 第 2 列被堵 / 切片 2 全自由的算例上对比：两者可达集**完全相同**（`[[1,1,1,0,0,0]], [[1,1,0,1,1,1]], [[1,1,1,1,1,1]]`）。原因：`relaxed = np.where(layer, relaxed, inf)` 已保证**目标格**必须空闲，而传播核本身允许"跨过"一格（`step + 0.5·cell` 的半格容差让 `dc=±2` 也可能进核），所以同一趟松弛里可以绕过被堵格；我另外试过更厚的墙与不同格宽，仍未构造出差异。**结论（只说我验证到的）**：L256 的直接效果是"把被占据格的 `arrival` 清成 `inf`，使其在本次切片的松弛中不能充当源格"；它是否会改变最终可达集，取决于核的可达半径与遮挡持续时间，我的合成算例里没观察到差异。

**R4 —— 网格/核/kernel 的真实数值**：

- `grid_shape(640, 480, 8.0)` → `(rows=60, cols=80, cell_w=8.0, cell_h=8.0)`；`grid_shape(640, 480, 13.3333)` → `(37, 49, 13.061, 12.973)`（格宽被均分，cols·cell_w == 640）。
- `movement_kernel(step=10.0, cell_w=8.0, cell_h=8.0)` → 9 个偏移：8 个一格邻居 + `(0,0)`；**对角 (11.3137) 也被收进核**，因为 `10 + 0.5·8 = 14 ≥ 11.3137`。对角一步的时间代价是 `11.3137/200 = 0.0566 s > dt(0.05)`，所以在 `dt=0.05` 的切片里对角移动要等到下一个切片才"usable"。

**R5 —— wall_gap 的 `axis="y"` 分支**：用仓库自己的 `collision.obstacles.obstacle_sdf` 对 `spawn="left"`、`gap_motion="static"`（`player_radius=10`，`gap_width=60`，`thickness=8`）生成的两个矩形采样：点 (320,240) 的 SDF = 251.77 = 到角点 (105,109) 的距离；点 (20,240) 的 SDF = 131.0 = 到 (20,109) 的距离(y 方向 131)。两个数字都只在"矩形长轴沿 **x**（`rot=π/2` 把 `half_h=105` 转到 x 方向）"时成立；同一测试点若按"长轴沿 y"（`rot=0`）算应为 317.4 / 34.0。对照 `spawn="top"` 分支：点 (320,470) 的 SDF = 30.59 = `hypot(30, 6)`（到缺口边缘角点 (290,476)），与"长轴沿 x、位于 y=480、缺口在 x∈(290,350)"一致。

**R6 —— corridor 数值**：见 §7.3 表（宽度 80、墙厚 12、墙长 704、内表面 y=200/280、`close` 夹到 2.75、`rotate_opposite` 夹到 4.467°）。

---

# 11. 未找到 / 不确定（明确列出，避免编造）

1. `motion.py` 没有"曲线弹的 n 步闭式跳跃"：`advance_bullets_n` 在**任何**活体弹 `angular_velocity != 0`（精确不等）时退化为 n 次单步积分（motion.py:132-135）。含加速度的曲线运动也没有闭式跳跃。
2. `motion.py` 里"含加速度的旋转"没有独立公式；位置增量一律用步末速度 `v_new` 做整段弧积分（§1.2）。
3. `relative.py:97` 那行 `out[moving & (c <= 0.0)] = 0.0` 我判定不可达（R2），但**没有**在仓库里找到说明它意图的文档/测试。
4. `time_to_impact` 对"当前已重叠且在运动"的目标返回的是 `t2`（离开圆的时间），不是 0（relative.py:92）；代码与注释都没有把这一点写成规格。
5. `closing_speed` / `contact_rate` 都用"中心连线方向"当 SDF 梯度方向；对圆精确、对**矩形**是近似（我按 `∇sdf` 推导，代码无说明）。
6. `dilate_with_offsets`（path_search.py:135-139）在包内**没有调用点**（只出现在定义与 `__all__`）。
7. `backtrack_path` 的终点选择注释说 "farthest (in travel terms)"，但 L319 的距离是**未加权**的格坐标欧氏距离（`(rr−r0)*1.0`），不是世界距离，也不是时间。
8. `backtrack_path` 不保存父指针，回溯用的是"上一层里离当前最近的格"，不保证是松弛时的真实前驱链。
9. 我**没有**在 `free_space.py` 里找到 `FreeSpaceGrid.origin_x/origin_y` 的非零用法（`rasterize_free_space` 恒置 0.0）；字段存在但当前无生产者。
10. `wall_gap` 的 `axis="y"` 分支的 `rot=π/2` 与 `hws/hhs` 赋值不一致（R5）；我没有找到说明该分支意图的注释/测试，`tests/test_patterns.py` 只覆盖默认 `spawn="top"`。
11. `state_hash` 不含 `age`、`scenario_id`、`seed`、`rng_state`、死槽位与容量（§8.3 表 + 代码可确认）；`timestamp` 与 `env.sim_time` 通常相同，属重复入哈希。
12. 玩家自身的位置/速度积分**不在** `physics/motion.py`，在 `physics/kinematics.py:33-86`（`apply_velocity_command`）；`motion.py` 只处理 bullet/obstacle 池。
