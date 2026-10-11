# `core/cpu/gap_avoid.py` 深度拆解（供技术文档取材）

本文件是对 `core/cpu/gap_avoid.py` 的**逐行级技术分析**。所有行号对应该文件当前版本（**共 674 行**），
所有代码片段均为仓库原文。

---

## 0. 先说三个读到的事实（避免文档写错）

| 你的假设 | 实际情况 |
|---|---|
| 文件约 600–900 行 | **674 行**（`wc -l` 实测） |
| 存在类型 `DecisionTier` | **没找到**。全文件没有这个符号。分档是 `Decision.tier` 的**字符串字面量**，共 **9 个**取值，不是 4 个 |
| 路径 `core/cpu/gap_avoid.py` | 正确。但文件内部注释自称 `cpu/gap_avoid.py` —— 因为仓库约定 `core/` 在 `sys.path` 上，包名是 `cpu` |

另外，`Decision` 的 4 档（ALIGN / TRIM / HOLD / NO_GAP）只是**横向分支**；代码实际还会输出
`IDLE` / `PANIC` / `SLOW` / `STOP` / `CRUISE`。其中 **`SLOW` 在代码里不可达**（证明见 §4.3、§11.2）。

---

## 1. 数据结构

### 1.1 `WorldView`（`gap_avoid.py:68-76`）—— 唯一输入

```python
@dataclass
class WorldView:
    """plan() 需要的全部输入。接自己的数据源时只要构造这个对象。"""
    player: Sequence[float]
    rects: list
    dt: float
    step_index: int
    field_w: float
    field_h: float = 0.0
```

| 字段 | 类型 | 内容 / 含义 | 单位 | 在 `plan()` 中的实际用途 |
|---|---|---|---|---|
| `player` | `Sequence[float]`，长度 7 | 玩家状态向量，下标见 `gap_avoid.py:59` | 世界单位 / 世界单位每秒 | 读 x、y、vx、vy、radius、speed（见下） |
| `rects` | `list`（元素是 `Rect` 或任何有同名属性的对象） | 当前帧**所有矩形障碍**，缺口墙的两段就在里面 | — | `group_walls(world.rects, ...)`（`:439`） |
| `dt` | `float` | 仿真步长 | **秒** | 线性拟合的时间基（`:520`）、转向限幅的积分步（`:625`） |
| `step_index` | `int` | 步序号，`sim_time = step_index * dt` | 步（无量纲） | 写入 `mem.step_index`（`:437`）、缺口中心历史的时间戳（`:512`） |
| `field_w` | `float` | 场地宽 | 世界单位 | 缺口区间裁剪（`:393`）、单段缺口方位推断（`:375`）、NO_GAP 逃逸边界检查（`:501`） |
| `field_h` | `float = 0.0` | 场地高 | 世界单位 | **在本文件中从未被读取**（仅 `:76` 定义、`:671` 自检里赋值） |

`player` 的下标常量（`gap_avoid.py:59`）：

```python
P_X, P_Y, P_VX, P_VY, P_RADIUS, P_SPEED, P_ALIVE = 0, 1, 2, 3, 4, 5, 6
```

| 下标 | 常量 | 含义 | 单位 | 代码中的读取点 |
|---|---|---|---|---|
| 0 | `P_X` | 位置 x | 世界单位 | `:430` |
| 1 | `P_Y` | 位置 y | 世界单位 | `:430` |
| 2 | `P_VX` | 速度 x | 世界单位/秒 | `:431` |
| 3 | `P_VY` | 速度 y | 世界单位/秒 | `:431` |
| 4 | `P_RADIUS` | 玩家半径（外接圆） | 世界单位 | `:432` |
| 5 | `P_SPEED` | **最大速度** `v_max`（同时是速度预算的标量上限） | 世界单位/秒 | `:433` |
| 6 | `P_ALIVE` | 存活标志 | 布尔 | **本文件从未读取** |

### 1.2 `Rect`（`gap_avoid.py:159-172`）—— 一个矩形实体

```python
@dataclass(frozen=True)
class Rect:
    """一个矩形实体（README :116-132 的字段子集）。"""

    ent_id: int
    x: float
    y: float
    vx: float
    vy: float
    half_w: float
    half_h: float
    rotation: float
    angular_velocity: float
```

| 字段 | 类型 | 含义 | 单位 | 备注 |
|---|---|---|---|---|
| `ent_id` | `int` | 实体 id | — | **`plan()` 里从不读取**；只在未使用的 `Wall.key`（`:189`）里出现 |
| `x`, `y` | `float` | 矩形中心，场地坐标 | 世界单位 | 契约：`x ∈ [0, field_w]`、`y ∈ [0, field_h]` |
| `vx`, `vy` | `float` | 线速度 | 世界单位/秒 | 用于墙法向逼近速度 `v_n`（`:456-457`）和退化定向（`:274-275`） |
| `half_w` | `float` | 沿**局部 x 轴**的半长 | 世界单位 | 组墙后该轴 = 墙的跨度方向 `û` |
| `half_h` | `float` | 沿**局部 y 轴**的半长 | 世界单位 | 组墙后该轴 = 墙的厚度方向 `n̂` |
| `rotation` | `float` | 局部系相对世界的旋转角 | **弧度** | 分桶键量化到 1e-3 rad（`:243`） |
| `angular_velocity` | `float` | 角速度 | 弧度/秒 | **本文件从未读取**（仅定义） |

`frozen=True`，值语义，可哈希，适合做跨帧比较。

### 1.3 `AlgoConfig`（`gap_avoid.py:106-156`）—— 全部可调参数

11 个字段，全部有默认值，`frozen=True`。逐字段的语义、单位、影响见 **§8 参数表**。

> 文档不一致：`core/cpu/docs/CPP_PORT.md` 写"10 个字段"，实际是 **11 个**（多出 `max_turn_rate`）。

### 1.4 `GapMemory`（`gap_avoid.py:290-324`）—— 跨帧状态（**每局必须新建**）

```python
@dataclass
class GapMemory:
    """跨帧记忆。墙身份按"上一帧最近的墙"关联（不能用 min(ent_id)，见文件头第 4 条）。"""

    walls: dict[int, WallMemory] = field(default_factory=dict)
    global_gap_width: Optional[float] = None
    step_index: int = -1
    _next_key: int = 1
    _prev: list = field(default_factory=list)   # [(band, key)]
    last_dir: Optional[tuple[float, float]] = None   # 上一帧下发过的方向（转向限速用）
```

| 字段 | 类型 | 含义 | 单位 |
|---|---|---|---|
| `walls` | `dict[int, WallMemory]` | 墙身份 key → 该墙的记忆。key 由 `match_keys()` 分配，**不是 `ent_id`** | — |
| `global_gap_width` | `Optional[float]` | 任意一堵墙量到过的缺口宽度（全场景同一参数，可跨墙复用） | 世界单位 |
| `step_index` | `int = -1` | 上次处理的步序号（**写入但本文件内不读取**） | 步 |
| `_next_key` | `int = 1` | 新墙身份的计数器，从 1 开始 | — |
| `_prev` | `list[(band, key)]` | 上一帧的 (墙带坐标, key) 列表，用于关联 | 世界单位 / — |
| `last_dir` | `Optional[tuple[float,float]]` | 上一帧**下发过**的非零指令方向（转向限幅的参考） | 无量纲（方向） |

三个方法：

* `get(key)`（`:301-302`）—— `setdefault` 式取记忆，不存在就建。
* `match_keys(bands, tol=24.0)`（`:304-320`）—— **跨帧身份关联**：对每个 band，在上一帧列表里找
  绝对差最小的一项，若 ≤ `tol`（默认 **24.0 世界单位**）就复用其 key 并把该项从候选里删掉
  （`prev.pop(pick[1])`，**一堵旧墙只能被认领一次**）；否则分配新 key。
* `single_segment_frames`（属性，`:322-324`）—— 所有墙"只有一段可见"的帧数总和（诊断）。

### 1.5 `WallMemory`（`gap_avoid.py:282-287`）—— 单堵墙的记忆

```python
@dataclass
class WallMemory:
    ring: deque = field(default_factory=lambda: deque(maxlen=24))
    gap_width_seen: Optional[float] = None      # 曾经亲眼看到过的缺口宽度
    gap_center_seen: Optional[float] = None     # 曾经亲眼看到过的缺口中心
    n_single_frames: int = 0                    # 只有一段可见的帧数（诊断用）
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `ring` | `deque(maxlen=24)`，元素 `(step_index, gap_center)`。**只有被选为目标墙的那一帧才追加**（`:512` 在选墙之后） | 步 / 世界单位 |
| `gap_width_seen` | 两段都可见时量到的缺口宽度 | 世界单位 |
| `gap_center_seen` | 两段都可见时量到的缺口中心（沿 `û`） | 世界单位 |
| `n_single_frames` | 该墙出现"只有一段可见"的帧计数 | 帧 |

### 1.6 `Wall`（`gap_avoid.py:174-189`）—— 组墙结果 + 墙坐标系

```python
@dataclass(frozen=True)
class Wall:
    """一堵缺口墙：1~2 段共线矩形 + 它的墙坐标系。

    û = 墙的跨度方向（缺口沿它排布，实测就是 half_w 轴）
    n̂ = 墙厚方向，**规定从墙指向玩家**，(v_wall - v_p)·n̂ > 0 表示"墙在逼近"。
    """

    rects: tuple[Rect, ...]
    u_hat: tuple[float, float]
    n_hat: tuple[float, float]
```

* `u_hat` = **跨度方向**（缺口沿它排布）；`n_hat` = **厚度方向**，**约定从墙指向车**。
* 由此推出全文的方位约定（文件头 `:27-31`）：**前进（朝墙）= `−n̂`**，**后退（远离墙）= `+n̂`**。
* `Wall.rects` 在 `:278` 被截断成 `tuple(members[:2])` —— **一堵墙最多保留 2 段**，
  第三段及以后被丢弃（`min`/`max` 之类统计也已经按截断后的集合算）。
* `key` 属性（`:186-189`）= `min(ent_id)`，注释自称"仅作诊断"—— 实测**在本文件中无任何读取点**。

### 1.7 `Aperture`（`gap_avoid.py:327-334`）—— 缺口（内部中间量，不在公开接口里）

```python
@dataclass(frozen=True)
class Aperture:
    e_lo: float          # 可用区间的左端（沿 û，已按场地裁剪）
    e_hi: float          # 右端
    gap_raw: float       # 未裁剪的缺口宽度
    center: float        # 可用区间中心
    usable_half: float   # A = 可用宽度 / 2 - r_p - margin
    complete: bool       # True = 两段都看见了
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `e_lo` / `e_hi` | 裁剪后的可用横向区间（沿 `û` 的坐标） | 世界单位 |
| `gap_raw` | **未**裁剪的缺口宽度 `e_hi − e_lo`（裁剪前） | 世界单位 |
| `center` | `0.5*(lo + hi)`，**裁剪后**区间的中心 | 世界单位 |
| `usable_half` | `0.5*(hi − lo) − r_p − margin` = 车心横向允差半宽 | 世界单位 |
| `complete` | `True` = 两段都看见（宽度是量出来的）；`False` = 单段 + 推断 | 布尔 |

失败时返回哨兵值 `Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)`（`:369`、`:381`），
`usable_half = −inf` 会触发 NO_GAP 分支。

### 1.8 `Decision`（`gap_avoid.py:399-417`）—— 唯一输出

```python
@dataclass
class Decision:
    wx: float = 0.0
    wy: float = 0.0
    tier: str = "IDLE"
    reason: str = ""
    t_arr: float = math.inf
    clearance: float = math.inf
    gap: float = math.nan
    usable_half: float = math.nan
    e_lo: float = math.nan
    e_hi: float = math.nan
    aim_lat: float = math.nan
    s_hat: float = 0.0
    n_rects: int = 0
    gap_line: tuple[tuple[float, float], tuple[float, float]] = (
        (math.nan, math.nan), (math.nan, math.nan))
    aim_world: tuple[float, float] = (math.nan, math.nan)
    wall_band: float = math.nan
```

| 字段 | 含义 | 单位 | 性质 |
|---|---|---|---|
| `wx`, `wy` | 归一化速度指令（世界系） | 无量纲，模长 ≤ 1 | **控制量** |
| `tier` | 本帧走的分支名 | — | 诊断（**控制流可读它**，见 `:598`） |
| `reason` | 人读的原因串（含格式化数值） | — | 诊断 |
| `t_arr` | 目标墙到达时间（`inf` = 不逼近 / 已在身后） | 秒 | 诊断 |
| `clearance` | `sdf − r_p − safety_margin`（负 = 已侵入安全裕度） | 世界单位 | 诊断 |
| `gap` | 未裁剪缺口宽度 | 世界单位 | 诊断 |
| `usable_half` | 车心横向允差半宽 `A` | 世界单位 | 诊断 |
| `e_lo`, `e_hi` | 裁剪后可用区间端点（沿 `û`） | 世界单位 | 诊断 |
| `aim_lat` | 瞄准点横向坐标（沿 `û`） | 世界单位 | 诊断 |
| `s_hat` | 缺口中心最小二乘漂移速率 | 世界单位/秒 | 诊断 |
| `n_rects` | 目标墙的矩形段数（截断后，≤2） | 段 | 诊断 |
| `gap_line` | 缺口在墙前表面上的两个世界坐标端点 | 世界坐标 | 可视化 |
| `aim_world` | 瞄准点的世界坐标 `= _band_point(wall, aim_lat, front)` | 世界坐标 | 可视化 |
| `wall_band` | 墙前表面沿 `n̂` 的坐标 `= front` | 世界单位 | 可视化 |

**关键点**：除 `wx`/`wy`/`tier` 外，其余字段**全部只用于诊断与可视化，不参与下一帧决策**
（唯一例外是 `tier` 在 `:598` 被读一次）。

### 1.9 `DecisionTier`

**没找到。** 本文件没有 `DecisionTier` 类、枚举或类型别名。分档完全由**字符串字面量**表达，
全部 9 个取值及其唯一赋值点：

| `tier` | 赋值行 |
|---|---|
| `IDLE` | `:403`（默认值）、`:442` |
| `NO_GAP` | `:504` |
| `HOLD` | `:573` |
| `TRIM` | `:576` |
| `ALIGN` | `:578`、`:599` |
| `PANIC` | `:583` |
| `SLOW` | `:603`（**注意：不可达，见 §4.3**） |
| `STOP` | `:603` |
| `CRUISE` | `:612` |

---

## 2. 主入口 `plan()` 的完整控制流

签名（`gap_avoid.py:427`）：

```python
def plan(world: Any, mem: GapMemory, cfg: AlgoConfig, horizon_remaining: float) -> Decision:
    """每帧调用一次：世界状态 -> Decision。只读，不修改世界。"""
```

按代码顺序：

### 步骤 1：解包世界状态（`:429-437`）

```python
    p = world.player
    px, py = float(p[P_X]), float(p[P_Y])
    pvx, pvy = float(p[P_VX]), float(p[P_VY])
    r_p = float(p[P_RADIUS])
    v_max = float(p[P_SPEED])
    dt = float(world.dt)
    step = int(world.step_index)
    field_w = float(world.field_w)
    mem.step_index = step
```

一次性 `float()` 强转，之后只用局部变量。`mem.step_index` 在这里被写入。

### 步骤 2：组墙（`:439`）

```python
    walls = group_walls(world.rects, px, py, cfg.forward)
```

把散乱矩形聚成"一堵墙"，并给每堵墙定出 `û`/`n̂`。空输入 → 空列表。

### 步骤 3：无墙早退（`:440-442`）

```python
    if not walls:
        mem.match_keys(())
        return Decision(tier="IDLE", reason="no_wall")
```

输出 `IDLE` + 零指令（`wx=wy=0.0` 默认值）。注意 `mem.match_keys(())` 会把 `_prev` 清空 ——
障碍全消失后再出现，会被当成新墙。

### 步骤 4：跨帧身份关联（`:444-446`）

```python
    bands = [sum(_dot((r.x, r.y), w.n_hat) for r in w.rects) / len(w.rects)
             for w in walls]
    keys = mem.match_keys(bands)
```

`band` = 墙段中心沿 `n̂` 的平均坐标（即"这堵墙在厚度方向上的位置"）。
关联容差 24.0 世界单位。这让 `walls` 顺序变化不会打断记忆。

### 步骤 5：遍历所有墙，评估并重建缺口（`:449-488`）

对每堵墙 `w`、其 key：

**(a) 墙带几何（`:451-454`）**

```python
        front = max(_dot((r.x, r.y), w.n_hat) + _half_extent_along(r, w.n_hat)
                    for r in w.rects)
        thick = 2.0 * max(_half_extent_along(r, w.n_hat) for r in w.rects)
        back = front - thick
```

`front` = 墙**朝向车那一侧**的表面坐标；`thick` = 厚度；`back` = 背面。

**(b) 法向相对速度（`:455-457`）**

```python
        pn = _dot((px, py), w.n_hat)
        vn_wall = sum(_dot((r.vx, r.vy), w.n_hat) for r in w.rects) / len(w.rects)
        v_n = vn_wall - _dot((pvx, pvy), w.n_hat)
```

`v_n > 0` 表示**墙在逼近车**。

**(c) 三分类 + 到达时间 + 优先级（`:459-470`）**

```python
        # 必须按玩家半径膨胀墙带、分三段判断（见文件头第 5 条）
        near = front + r_p
        far = back - r_p
        if pn > near:
            t_arr = (pn - near) / v_n if v_n > EPS else math.inf
            prio, key_d = 1, pn - near           # 还在前面：越近越优先
        elif pn >= far:
            t_arr = 0.0                          # 玩家的圆正压在墙带上
            prio, key_d = 0, 0.0                 # 正压着它 => 必须继续盯住这一堵
        else:
            t_arr = math.inf                     # 整面墙已经在身后
            prio, key_d = 2, 0.0                 # 身后 => 不再是约束
```

**配置空间膨胀**就体现在 `near = front + r_p` / `far = back − r_p`：墙带按玩家半径向外扩张，
把"圆 vs 矩形"降维成"点 vs 膨胀矩形"。三段分类决定排序优先级 `prio ∈ {1,0,2}`。

**(d) 间隙与安全裕度（`:479-480`）**

```python
        sdf = min(_rect_sdf(r, px, py)[0] for r in w.rects)
        clearance = sdf - r_p - cfg.safety_margin
```

**(e) 缺口重建 + 全局缺口宽度学习（`:482-486`）**

```python
        wmem = mem.get(key)
        ap_w = reconstruct_aperture(w, wmem, r_p, cfg.safety_margin, field_w,
                                    mem.global_gap_width)
        if ap_w.complete and ap_w.gap_raw > 0.0:
            mem.global_gap_width = ap_w.gap_raw
```

**每一堵墙每帧都重建缺口**（不只是目标墙）—— 目的是让 `global_gap_width` 尽快学到值。

**(f) 收集候选（`:487-488`）**

```python
        cands.append((w, key, t_arr, clearance, v_n, front, thick, ap_w,
                      prio, key_d))
```

元组下标：`0=w, 1=key, 2=t_arr, 3=clearance, 4=v_n, 5=front, 6=thick, 7=ap, 8=prio, 9=key_d`。

### 步骤 6：字典序选目标墙（`:490-491`）—— 唯一的目标选择点

```python
    (wall, key, t_arr, clearance, v_n, front, thickness, ap, _p, _d) = min(
        cands, key=lambda c: (c[8], c[9]))
```

细节见 **§5**。

### 步骤 7：取记忆与横向坐标（`:492-493`）

```python
    wmem = mem.get(key)
    lat = _dot((px, py), wall.u_hat)
```

### 步骤 8：缺口不可用 → NO_GAP 逃逸（`:495-509`）

```python
    if ap.usable_half <= 0.0:
        # 推不出缺口 / 膨胀后装不下人：朝远离最近那段墙的方向横移兜底，
        # 别把自己顶到场地边界上。
        r_near = min(wall.rects, key=lambda r: _rect_sdf(r, px, py)[0])
        proj_near = _dot((r_near.x, r_near.y), wall.u_hat)
        sgn = 1.0 if lat < proj_near else -1.0
        if not (0.0 <= lat + sgn * 40.0 <= field_w):
            sgn = -sgn
        return Decision(wx=sgn * wall.u_hat[0], wy=sgn * wall.u_hat[1],
                        tier="NO_GAP", reason="aperture_unknown_flee",
                        ...)
```

**早退**（这里是 `plan()` 里唯一一个在缺口分析前就返回的分支）。
`usable_half ≤ 0` 包含两种情形：① 膨胀后缺口装不下车；② 缺口根本没推出来（`−inf`）。
逃逸方向是"远离最近那一段墙"，若前方 40 世界单位会出界就反向。

### 步骤 9：缺口轨迹估计（`:512-528`）—— 只有目标墙会被采样

```python
    wmem.ring.append((step, ap.center))
    s_hat, resid = 0.0, 0.0
    if len(wmem.ring) >= 2:
        t0, _s0 = wmem.ring[0]
        n = len(wmem.ring)
        st = stt = ss = sts = 0.0
        for (ti, si) in wmem.ring:
            tt = (ti - t0) * dt
            st += tt
            stt += tt * tt
            ss += si
            sts += tt * si
        den = n * stt - st * st
        s_hat = 0.0 if abs(den) < EPS else (n * sts - st * ss) / den
        b = (ss - s_hat * st) / n
        for (ti, si) in wmem.ring:
            resid = max(resid, abs(si - (b + s_hat * (ti - t0) * dt)))
```

对 `(step, gap_center)` 序列做**最小二乘直线拟合**，输出斜率 `s_hat`（世界单位/秒）
和**最大残差** `resid`（保守包络半宽）。`den` 退化时 `s_hat = 0`。

### 步骤 10：包络上下界（`:530-534`）

```python
    def s_lo(t: float) -> float:
        return ap.center + min(0.0, s_hat) * t - resid * t

    def s_hi(t: float) -> float:
        return ap.center + max(0.0, s_hat) * t + resid * t
```

以当前 `center` 为锚，向漂移方向按 `s_hat` 展开，**两侧同时按 `resid` 线性加宽**。

### 步骤 11：时窗（`:536`）—— **死代码**

```python
    H = min(float(horizon_remaining), cfg.max_window)
```

`H` 被赋值后**在全文件再无任何引用**。连带后果：入参 `horizon_remaining` 与
`cfg.max_window` 对本算法**没有任何行为影响**。

### 步骤 12：可达性工具（`:538-546`）—— **`feasible` 是死代码**

```python
    def worst_offset(t: float) -> float:
        return max(abs(s_lo(t) - lat), abs(s_hi(t) - lat)) + 0.5 * (s_hi(t) - s_lo(t))

    def reach(t: float) -> float:
        return v_max * max(0.0, t)          # README 判据 ②（本脚本不开加速度）

    def feasible(t: float) -> bool:
        off = worst_offset(t)
        return off <= ap.usable_half or (off - ap.usable_half) <= reach(t)
```

`feasible` **从未被调用**（`worst_offset` 只被 `feasible` 调用，因此也事实死亡）。
`reach` 有两处活引用：`:582`（PANIC 判定）与 `:546`（死的）。

### 步骤 13：瞄准点 —— 拦截时间迭代（`:559-563`）

```python
    t_aim = abs(0.5 * (s_lo(0.0) + s_hi(0.0)) - lat) / max(v_max, EPS)
    for _ in range(4):
        cand = abs(0.5 * (s_lo(t_aim) + s_hi(t_aim)) - lat) / max(v_max, EPS)
        t_aim = cand if not math.isfinite(t_arr) else min(t_arr, cand)
    aim_lat = 0.5 * (s_lo(t_aim) + s_hi(t_aim))
```

求不动点 `T = |s(T) − lat| / v_max`，**固定 4 次**迭代（无收敛判据）。
上界由 `t_arr` 钳住；`t_arr = inf` 时不钳。

> 注释 `:551-552` 写的是"瞄准时刻取 `min(t_arr, aim_lookahead)`"，
> **与代码不符**：代码钳的是 `cand`，`aim_lookahead` 从未被读取。

### 步骤 14：横向误差（`:564-566`）

```python
    d_lat = aim_lat - lat
    need = max(0.0, abs(d_lat) - ap.usable_half)
    sgn_lat = 1.0 if d_lat >= 0 else -1.0
```

`need` = 横向还差多少才进得了"膨胀后仍安全"的区间。

### 步骤 15：可视化用缺口线（`:568-569`）

```python
    band = front
    gl = (_band_point(wall, ap.e_lo, band), _band_point(wall, ap.e_hi, band))
```

### 步骤 16：横向分档（`:572-578`）—— HOLD / TRIM / ALIGN

```python
    if abs(d_lat) <= cfg.dead_frac * ap.usable_half:
        tier, reason, u_mag = "HOLD", "gap_will_pass_me", 0.0
    elif abs(d_lat) <= cfg.align_frac * ap.usable_half:
        u_mag = min(1.0, abs(d_lat) / max(ap.usable_half, EPS))
        tier, reason = "TRIM", f"recenter dlat={d_lat:+.1f}"
    else:
        u_mag, tier, reason = 1.0, "ALIGN", f"move_to_gap dlat={d_lat:+.1f}"
```

此处**不 return**，因为下面还要叠加前后分量。`u_mag` 是横向速度的**归一化**比例。

### 步骤 17：PANIC 覆盖（`:582-583`）

```python
    if math.isfinite(t_arr) and need > reach(t_arr) and not cfg.front_back:
        u_mag, tier, reason = 1.0, "PANIC", "unreachable_before_arrival"
```

条件：墙会到达（`t_arr` 有限）**且** 横向缺口在到达前追不上 **且** 不允许前后控制。
`front_back=True` 时**永不 PANIC**（因为减速/停住可以把时间买回来）。

### 步骤 18：前后轴与净距（`:587-588`）

```python
    fwd = (-wall.n_hat[0], -wall.n_hat[1])
    d_n = max(0.0, _dot((px, py), wall.n_hat) - (front + r_p))   # 车到墙的净距离
```

`fwd = −n̂` = 朝墙 = 前进。`d_n` = 车心到膨胀墙面的距离（已减掉 `r_p`）。

### 步骤 19：速度预算（`:590-591`）

```python
    u_speed = u_mag * v_max
    w_cap = math.sqrt(max(0.0, v_max * v_max - u_speed * u_speed))  # 剩余速度预算
```

见 **§6**。

### 步骤 20a：允许前后控制（`:592-604`）

```python
    if cfg.front_back:
        t_align = need / max(u_speed, EPS) if need > 1e-6 else 0.0
        room = max(0.0, d_n - cfg.hold_clearance)
        t_reach = room / max(w_cap, EPS)
        if t_reach >= t_align:
            w = w_cap                                   # 时间够：全速通过
            if tier == "PANIC":
                tier, reason = "ALIGN", reason
        else:
            # 时间不够：减速到"恰好对齐完成时抵达"，必要时直接停住等
            w = min(w_cap, room / max(t_align, EPS))
            tier = "SLOW" if w > 1e-6 else "STOP"
            reason = f"hold_to_align need={need:.1f} t_align={t_align:.2f}"
```

时间够 → 全速前进（`w = w_cap`），并把 PANIC**降级**为 ALIGN（`:598-599`）。
时间不够 → 减速到 `room / t_align`，或停住（`w = 0`）。

### 步骤 20b：禁止前后控制（`:605-612`）

```python
    else:
        # 不允许前后控制：**油门是固定的**（车不能停、不能减速），横向只能用剩下的预算。
        # 这才是"只会左右移动的车"的诚实模型：|v| <= v_max 是同一份预算。
        w = min(cfg.fwd_speed * v_max, v_max)
        u_cap = math.sqrt(max(0.0, v_max * v_max - w * w))
        u_mag = min(u_mag, u_cap / max(v_max, EPS))
        if cfg.fwd_speed > 0.0:
            tier, reason = "CRUISE", reason
```

固定油门 `w`，横向能力被剩余预算 `u_cap` 反向钳住。**注意 `:611-612` 无条件覆盖 `tier`**：
只要 `fwd_speed > 0`，前面对 HOLD/TRIM/ALIGN/PANIC 的判定在 `tier` 字段上全部被写成 `CRUISE`
（实际横向行为仍然生效，只是标签被覆盖）。

### 步骤 21：合成世界指令（`:614-616`）

```python
    w_frac = w / v_max
    wx = u_mag * sgn_lat * wall.u_hat[0] + w_frac * fwd[0]
    wy = u_mag * sgn_lat * wall.u_hat[1] + w_frac * fwd[1]
```

因为是 `û` 与 `n̂` 正交分解后相加，模长满足 `√(u_mag² + w_frac²) ≤ 1`（实测见 §6.3）。

### 步骤 22：转向速率限幅（`:619-628`）

```python
    if cfg.max_turn_rate > 0.0 and mem.last_dir is not None:
        mag = math.hypot(wx, wy)
        if mag > EPS:
            a_new = math.atan2(wy, wx)
            a_old = math.atan2(mem.last_dir[1], mem.last_dir[0])
            d = wrap_pi(a_new - a_old)          # 归一化到 [-pi, pi)，见 wrap_pi 注释
            lim = math.radians(cfg.max_turn_rate) * dt
            if abs(d) > lim:
                a_new = a_old + math.copysign(lim, d)
                wx, wy = mag * math.cos(a_new), mag * math.sin(a_new)
```

见 **§7**。位置在**合成之后**，所以它限的是"横向 + 前后"合成向量的方向。

### 步骤 23：更新方向记忆（`:629-630`）

```python
    if math.hypot(wx, wy) > EPS:
        mem.last_dir = (wx, wy)
```

**零指令不更新 `last_dir`** —— 于是一段 HOLD（零指令）之后，参考方向仍是 HOLD 之前那个。

### 步骤 24：返回（`:632-637`）

```python
    return Decision(wx=wx, wy=wy, tier=tier, reason=reason,
                    t_arr=t_arr, clearance=clearance, gap=ap.gap_raw,
                    usable_half=ap.usable_half, e_lo=ap.e_lo, e_hi=ap.e_hi,
                    aim_lat=aim_lat, s_hat=s_hat, n_rects=len(wall.rects),
                    gap_line=gl, aim_world=_band_point(wall, aim_lat, band),
                    wall_band=band)
```

---

## 3. 每一个内部函数

### 3.1 `_dot(a, b)`（`:79-80`）—— 二维点积

```python
def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])
```

只取前两维，**强制 `float()`**（防止 numpy 标量混进来）。

### 3.2 `wrap_pi(angle)`（`:83-103`）—— 角度归一化到 `[-pi, pi)`

```python
def wrap_pi(angle: float) -> float:
    ...
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
```

公式：`wrap_pi(a) = ((a + π) mod 2π) − π`。区间是**半开** `[−π, π)`，因此 `wrap_pi(π) == −π`。

文档字符串（`:86-101`）详细记录了 **C++ 移植陷阱**：Python 的 `%` 对负数返回非负，
而 `std::fmod` 保留符号，差别在 `angle < −π` 时暴露（`a_new − a_old ∈ [−2π, 2π]`，一定会遇到）。
C++ 等价写法：先 `fmod`，若为负再加 `2π`。

唯一调用点：`:624`。

### 3.3 `_half_extent_along(r, d)`（`:192-200`）—— 旋转矩形在方向 `d` 上的支撑函数

```python
def _half_extent_along(r: Rect, d: Sequence[float]) -> float:
    """旋转矩形在单位方向 d 上的半投影长度（精确支撑函数，不是小角度近似）。

        h(d) = half_w * |d · x̂_local| + half_h * |d · ŷ_local|
    """
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    aw = (ct, st)                 # 局部 x 轴 = half_w 方向
    ah = (-st, ct)                # 局部 y 轴 = half_h 方向
    return r.half_w * abs(_dot(d, aw)) + r.half_h * abs(_dot(d, ah))
```

`h(d) = half_w·|d·x̂| + half_h·|d·ŷ|`，其中 `x̂ = (cos θ, sin θ)`、`ŷ = (−sin θ, cos θ)`。
这是**精确**的（不是小角度近似），大角度旋转也成立。

用途：算墙段内缘位置（`:354-355`）、算墙前后表面（`:451-453`）。

### 3.4 `_rect_sdf(r, px, py)`（`:203-227`）—— 旋转矩形 SDF + 单位梯度

```python
def _rect_sdf(r: Rect, px: float, py: float) -> tuple[float, tuple[float, float]]:
    """旋转矩形 SDF + 它的单位梯度（README :496：引擎自带圆+旋转矩形 SDF）。"""
    ct, st = math.cos(r.rotation), math.sin(r.rotation)
    dx, dy = px - r.x, py - r.y
    lx = dx * ct + dy * st                 # 世界 -> 局部
    ly = -dx * st + dy * ct
    qx = abs(lx) - r.half_w
    qy = abs(ly) - r.half_h
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    sdf = outside + inside
```

算法（标准 IQ 式矩形 SDF）：把点变换到矩形局部系，取 `q = |p_local| − half`，
`outside = hypot(max(qx,0), max(qy,0))`、`inside = min(max(qx,qy), 0)`，`sdf = outside + inside`。
`sdf > 0` 在外、`< 0` 在内。

梯度部分（`:215-227`）：

```python
    sx = 1.0 if lx >= 0 else -1.0
    sy = 1.0 if ly >= 0 else -1.0
    if qx > 0.0 and qy > 0.0:              # 角点区
        gx_l, gy_l = sx * qx, sy * qy
        n = math.hypot(gx_l, gy_l) or 1.0
        gx_l, gy_l = gx_l / n, gy_l / n
    elif qx > 0.0:
        gx_l, gy_l = sx, 0.0
    elif qy > 0.0:
        gx_l, gy_l = 0.0, sy
    else:                                  # 在矩形内部：沿最浅的轴推出
        gx_l, gy_l = (sx, 0.0) if qx > qy else (0.0, sy)
    return sdf, (gx_l * ct - gy_l * st, gx_l * st + gy_l * ct)
```

分四区（角点 / x 面 / y 面 / 内部）给出单位梯度，最后旋回世界系。
**`plan()` 只用了返回值的第一项**（`:479` 的 `[0]`、`:498` 的 `[0]`），梯度部分在本文件中未被使用。

### 3.5 `group_walls(rects, px, py, forward=None)`（`:230-279`）—— 把矩形聚成"一堵墙"

**问题**：一堵缺口墙由 1~2 个共线矩形组成，算法需要知道它们是同一堵墙，并建立 `û`/`n̂`。
**不能用 `group_id`**（注释 `:234`：实测本场景所有墙段 `group_id` 都是 0）。

**算法**：

1. **分桶（`:238-244`）**：对每个矩形算它在自己的"墙厚方向"上的坐标 `band = r.x·nx + r.y·ny`
   （其中 `(nx, ny) = (−sin θ, cos θ)`，即局部 y 轴），把 `(rotation, band)` 量化成键：

```python
        key = (int(round(r.rotation * 1000.0)), int(round(band / 2.0)))
```

   量化精度：**旋转 0.001 rad**、**band 2.0 世界单位**一档。注释 `:236` 说明相邻墙之间 ≥100 单位，不会误并。

2. **显式排序（`:250-252`）**：

```python
    for key in sorted(buckets):
        members = buckets[key]
        members = sorted(members, key=lambda r: r.x)
```

   原因是**决定性**的：`min()` 在并列时返回第一个元素，而 Python `dict` 迭代顺序 = 插入顺序；
   C++ 的 `unordered_map` 顺序与之无关，会让两侧选出不同的墙。

3. **建立墙坐标系（`:253-258`）**：

```python
        ct, st = math.cos(members[0].rotation), math.sin(members[0].rotation)
        u_hat = (ct, st)                      # half_w 轴 = 墙的跨度方向
        # 规范化 û 让它大致指向 +x，这样 e_lo/e_hi、"左侧/右侧"才有稳定含义
        if u_hat[0] < -EPS or (abs(u_hat[0]) <= EPS and u_hat[1] < 0.0):
            u_hat = (-u_hat[0], -u_hat[1])
        n_hat = (-u_hat[1], u_hat[0])
```

   `û` 被规范化到大致指向 `+x`（`u_hat[0] > 0`，或 `u_hat[0] ≈ 0` 时 `u_hat[1] ≥ 0`），
   这样"左/右"才有稳定含义；`n̂ = û` 逆时针转 90°。

4. **定向 `n̂`（从墙指向车）（`:260-277`）**：

```python
        mx = sum(r.x for r in members) / len(members)
        my = sum(r.y for r in members) / len(members)
        to_player = (px - mx, py - my)
        if forward is not None:
            # 固定行驶轴的场景（小车朝 +forward 开）：n̂ 从墙指向车，恒等于 -forward。
            n_hat = (-forward[0], -forward[1])
        elif abs(_dot(n_hat, to_player)) > TOL:
            if _dot(n_hat, to_player) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
        else:
            # 退化：玩家正好落在墙所在的那条线上，用墙自己的速度定向
            vx = sum(r.vx for r in members) / len(members)
            vy = sum(r.vy for r in members) / len(members)
            if _dot((vx, vy), n_hat) < 0.0:
                n_hat = (-n_hat[0], -n_hat[1])
```

   **三条路径**：① `cfg.forward` 给了 → 直接钉死 `n̂ = −forward`（注释解释：静态墙法向速度为 0，
   车恰好在墙线上时点积退化，逐帧重判会让 `n̂` 来回翻、输出隔帧反号）；② 正常 → 按"车在哪一侧"翻转；
   ③ 退化（`|n̂·to_player| ≤ TOL = 1e-6`）→ 用墙自身的平均速度定向。

5. **建墙并截断到 2 段（`:278`）**：

```python
        walls.append(Wall(rects=tuple(members[:2]), u_hat=u_hat, n_hat=n_hat))
```

### 3.6 `reconstruct_aperture(wall, mem, r_p, margin, field_w, global_gap=None)`（`:337-396`）—— 从可见矩形反算缺口

**问题**：视觉/仿真可能只看到缺口墙的**一段**（另一段被遮挡或出画），
但算法必须知道缺口在哪、有多宽。

**两段都可见（`:352-361`）**：

```python
    if len(rects) >= 2:
        rl, rr = rects[0], rects[1]
        inner_a = _dot((rl.x, rl.y), u_hat) + _half_extent_along(rl, u_hat)
        inner_b = _dot((rr.x, rr.y), u_hat) - _half_extent_along(rr, u_hat)
        e_lo, e_hi = min(inner_a, inner_b), max(inner_a, inner_b)
        gap = e_hi - e_lo
        if gap > 0.0:
            mem.gap_width_seen = gap
            mem.gap_center_seen = 0.5 * (e_lo + e_hi)
        complete = True
```

先把两段按沿 `û` 的投影排序（`:349`），然后取两条**内缘**（左段中心 + 自身半投影、右段中心 − 自身半投影），
`min/max` 兜住顺序，得到 `e_lo`、`e_hi`、`gap`。同时写入该墙的记忆。

**只有一段可见（`:362-390`）**：

```python
    else:
        mem.n_single_frames += 1
        r = rects[0]
        proj = _dot((r.x, r.y), u_hat)
        half = _half_extent_along(r, u_hat)
        g_mem = mem.gap_width_seen if mem.gap_width_seen is not None else global_gap
        if g_mem is None or g_mem <= 0.0:
            return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)
```

宽度来源优先级：**① 本墙历史 → ② 全局任意墙历史 → ③ 放弃（返回 `usable_half = −inf`）**。

方位推断（`:371-381`）：

```python
        if mem.gap_center_seen is not None:
            on_left = proj < mem.gap_center_seen
        else:
            free_left = proj - half
            free_right = field_w - (proj + half)
            if free_left + TOL >= g_mem and free_left >= free_right:
                on_left = False                      # 缺口在左边 => 可见的是右段
            elif free_right + TOL >= g_mem:
                on_left = True                       # 缺口在右边 => 可见的是左段
            else:
                return Aperture(0.0, 0.0, 0.0, 0.0, -math.inf, complete=False)
```

有本墙记忆就按记忆方位；否则看哪一侧的自由空间放得下 `g_mem`（用 `TOL = 1e-6` 做容差），
两侧都放不下 → 放弃。然后按方位把缺口"贴"在可见段的内缘上（`:383-389`）。

**裁剪与膨胀（`:392-396`）**：

```python
    lo = max(e_lo, 0.0)
    hi = min(e_hi, field_w)
    usable_w = hi - lo
    return Aperture(lo, hi, gap, 0.5 * (lo + hi),
                    0.5 * usable_w - r_p - margin, complete)
```

**这就是配置空间膨胀**：可用区间先按场地裁剪，再两侧各扣掉 `r_p + margin`：

    A = usable_half = (min(e_hi, field_w) − max(e_lo, 0)) / 2 − r_p − margin

`A ≤ 0` 就表示"膨胀后装不下车"或"没推出来"，触发 NO_GAP。

### 3.7 `_band_point(wall, lateral, band)`（`:420-424`）—— 墙坐标 → 世界坐标

```python
def _band_point(wall: Wall, lateral: float, band: float) -> tuple[float, float]:
    """（沿 û 的横向坐标, 沿 n̂ 的坐标）-> 世界坐标：P = lateral*û + band*n̂。"""
    ux, uy = wall.u_hat
    nx, ny = wall.n_hat
    return (lateral * ux + band * nx, lateral * uy + band * ny)
```

`P = lateral·û + band·n̂`。注意这里**没有加墙的平移量** —— 它隐含"墙坐标系的原点选在世界原点"，
横向坐标本身就是沿 `û` 的绝对投影（`e_lo`/`e_hi`/`lat` 全是同一套绝对投影坐标），
所以公式自洽。用于 `gap_line` 与 `aim_world`。

### 3.8 `plan()` 内的局部函数

| 函数 | 行 | 作用 | 是否被调用 |
|---|---|---|---|
| `s_lo(t)` | `:530-531` | 缺口左界包络 | ✅ `:559-563` |
| `s_hi(t)` | `:533-534` | 缺口右界包络 | ✅ `:559-563` |
| `worst_offset(t)` | `:538-539` | 最坏横向偏移 + 半宽裕度 | 仅被 `feasible` 调用（事实死亡） |
| `reach(t)` | `:541-542` | `v_max·max(0,t)`，满速可达距离 | ✅ `:582`（+ `:546` 死路径） |
| `feasible(t)` | `:544-546` | 可达性判定 | ❌ **从未调用** |

### 3.9 模块级 `__getattr__`（`:648-655`）—— PEP 562 懒 re-export

```python
_LAZY_ADAPTERS = ("rect_from_pool", "observe_rects", "view_from_world")


def __getattr__(name: str):
    if name in _LAZY_ADAPTERS:
        from cpu.adapters import sim_adapter
        return getattr(sim_adapter, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
```

只有访问这 **3 个名字**时才导入 `cpu.adapters.sim_adapter`（避免循环 import）。
**这三个名字来自仿真适配层，不属于本算法的移植范围**；访问它们会连带拉进
`bullet_sim` 与 numpy（实测），但**不访问就不会**。

---

## 4. 分档策略

### 4.1 澄清：没有 `DecisionTier`，共 9 个 tier

`DecisionTier` **没找到**。tier 是 `Decision.tier: str`，取值见 §1.9 表。
你点名的"四档"是**横向分量**的分档（`:572-578`）；完整分支还有 `IDLE` / `PANIC` / `SLOW` / `STOP` / `CRUISE`。

### 4.2 触发条件与输出（逐条，含行号）

设 `A = ap.usable_half`（车心横向允差半宽）、`d_lat = aim_lat − lat`、`need = max(0, |d_lat| − A)`。

| tier | 触发条件（原文） | 行 | 输出 `u_mag`（横向归一化） | 输出 `w`（前后） | 典型指令 |
|---|---|---|---|---|---|
| `IDLE` | `if not walls` | `:440-442` | 无（零指令） | — | `(0, 0)` |
| `NO_GAP` | `if ap.usable_half <= 0.0` | `:495-509` | 无（**早退**） | — | `±û`（单位模长），远离最近那段墙 |
| `HOLD` | `abs(d_lat) <= cfg.dead_frac * A` | `:572-573` | `0.0` | 见 §4.3 | `front_back=False, fwd_speed=0` → `(0,0)`；`front_back=True` → `(0, ±1)` 满油门穿过去 |
| `TRIM` | `abs(d_lat) <= cfg.align_frac * A` | `:574-576` | `min(1, |d_lat| / A)` | 同上 | 部分横向 + 部分前向 |
| `ALIGN` | 其它 | `:577-578` | `1.0` | 同上 | 纯横向（若 `w_cap = 0`）或横向满 + 前向剩余 |
| `PANIC` | `isfinite(t_arr) and need > reach(t_arr) and not cfg.front_back` | `:582-583` | `1.0` | `front_back=False` 路径 | 横向满速，明知来不及 |
| `ALIGN`（降级） | `front_back=True` 且 `t_reach >= t_align` 且原为 PANIC | `:598-599` | 不变 | `w_cap` | PANIC 被"减速买时间"救回 |
| `STOP` | `front_back=True` 且 `t_reach < t_align` 且 `w <= 1e-6` | `:600-604` | 不变 | `0` | 停住等对齐 |
| `SLOW` | 同上但 `w > 1e-6` | `:603` | 不变 | `room / t_align` | **不可达，见 §4.3** |
| `CRUISE` | `not front_back` 且 `cfg.fwd_speed > 0.0`（**无条件覆盖上面的 tier 标签**） | `:611-612` | `min(u_mag, u_cap/v_max)` | `fwd_speed·v_max` | 固定油门 + 剩余横向 |

### 4.3 `gap_will_pass_me` 到底怎么判的

它就是 **HOLD 分支的 reason 字符串**，判定式在 `gap_avoid.py:572-573`：

```python
    if abs(d_lat) <= cfg.dead_frac * ap.usable_half:
        tier, reason, u_mag = "HOLD", "gap_will_pass_me", 0.0
```

即：**瞄准点与车的横向差 `|d_lat|` 落在"允差半宽 A 的 `dead_frac`（默认 5%）"这条死区内**。

含义：缺口（对准点）已经基本压在车身上，横向不需要动 —— 缺口"会自己滑过我"，
所以**停住横向动作是正确解**（README `core/cpu/README.md:80-82` 专门提醒：
闭环里"前向进展 0"是设计行为）。

`d_lat` 的定义在 `:563-564`：

```python
    aim_lat = 0.5 * (s_lo(t_aim) + s_hi(t_aim))
    d_lat = aim_lat - lat
```

注意 `d_lat` 用的是**拦截时刻的预测缺口中心**（`s_lo`/`s_hi` 的均值），不是当前缺口中心。
所以"缺口在动"时，即使当前中心离车很远，只要**预测**它会在车横向到位时压上来，也会判 HOLD。

**实测确认**（构造场景：场地 640×480，墙 y=300 厚 20，缺口中心 x=320 宽 100，车 r=10、v_max=200；
`usable_half = 50 − 10 − 6 = 34`，死区阈值 `0.05 × 34 = 1.7`）：

```
lat=320.00  tier=HOLD  reason=gap_will_pass_me  cmd=(0.0000, 0.0000)  usable_half=34.00  aim_lat=320.00
```

### 4.4 关于 `SLOW` 不可达的证明

要进 `else`（`:600`）必须 `t_reach < t_align`，即 `need > 0`（因为 `need ≤ 1e-6` 时 `t_align = 0`）。
而 `need = max(0, |d_lat| − A) > 0` 要求 `|d_lat| > A`，此时横向分档必然是 **ALIGN**（`|d_lat| > A ≥ align_frac·A`），
于是 `u_mag = 1.0`，于是 `u_speed = v_max`，于是：

```python
    w_cap = math.sqrt(max(0.0, v_max * v_max - u_speed * u_speed))   # = 0
```

`w_cap = 0` ⇒ `w = min(0, room/t_align) = 0` ⇒ `tier = "SLOW" if 0 > 1e-6 else "STOP"` ⇒ **恒为 STOP**。

反过来，`u_mag < 1` 只可能出现在 HOLD（`u_mag = 0`）或 TRIM（`u_mag = |d_lat|/A < 1`），
这两种情况下 `|d_lat| < A` ⇒ `need = 0` ⇒ `t_align = 0` ⇒ `t_reach >= 0 = t_align` 恒成立 ⇒ 走 `if` 分支。

**40,000 次随机参数扫描**（随机化 lat / py / 车速 / 墙速 / 缺口位置宽度 / `safety_margin` /
`align_frac` / `dead_frac` / `front_back` / `fwd_speed` / `hold_clearance` / `max_turn_rate` / `forward`）
观察到的 tier 分布：

```
CRUISE 12327 | ALIGN 11530 | NO_GAP 7133 | STOP 4300 | TRIM 2144 | PANIC 1583 | HOLD 983
SLOW reachable? False
```

`SLOW` 在整个扫描中**一次都没出现**。

---

## 5. "字典序目标选择"的实现

### 5.1 唯一实现点

`gap_avoid.py:490-491`：

```python
    (wall, key, t_arr, clearance, v_n, front, thickness, ap, _p, _d) = min(
        cands, key=lambda c: (c[8], c[9]))
```

`c[8]` 与 `c[9]` 是候选元组里的 `prio` 和 `key_d`（`:487-488` 的构造顺序）：

```python
        cands.append((w, key, t_arr, clearance, v_n, front, thick, ap_w,
                      prio, key_d))
```

下标：`0=w, 1=key, 2=t_arr, 3=clearance, 4=v_n, 5=front, 6=thick, 7=ap_w, 8=prio, 9=key_d`。

### 5.2 两级键的含义

* **第一级 `prio ∈ {0, 1, 2}`**（`:462-470`）—— 相位分类，数值越小越优先：

  | `prio` | 条件 | 含义 |
  |---|---|---|
  | `0` | `pn >= far` 且 `pn <= near` | 玩家的圆**正压在墙带上** → 必须继续盯住这一堵 |
  | `1` | `pn > near` | 墙还在车前方 → 越近越优先（第二级比较） |
  | `2` | `pn < far` | 整面墙已在身后 → 不再是约束 |

* **第二级 `key_d`** —— 只有 `prio = 1` 时才有意义（`:464`）：

```python
            prio, key_d = 1, pn - near           # 还在前面：越近越优先
```

  其它两种情形 `key_d = 0.0`（`:467`、`:470`）。

因此排序语义是：**先按"是否正压着墙"（0 优先），再按"前方的墙由近到远"，最后才是身后的墙**。

### 5.3 三条"不是加权和"的证据

1. **没有乘法权重、没有加法聚合**：`min(..., key=lambda c: (c[8], c[9]))` 是 Python 元组的
   逐元素字典序比较（先比 `prio`，相等才比 `key_d`），不是 `w1·a + w2·b`。
2. **`t_arr` 和 `clearance` 不参与选择**：它们分别是 `c[2]`、`c[3]`，只在返回时作为
   诊断量塞进 `Decision`（`:633`）。也就是说"最近"用的是**位置** `pn − near`，不是时间。
   代码注释 `:472-477` 明确解释了为什么不能用 `t_arr` 选墙：`t_arr = d_n / v_n` 含车自己的速度，
   静态墙下车只要略微后退（`v_n ≤ 0`）所有墙的 `t_arr` 同时变 `inf`，`min()` 就会随便挑一堵，
   目标在两道缺口间来回跳（实测 ±175px）。位置判据单调、不含速度，不会退化。
3. **硬约束优先于软目标**：横向分档（`:572-578`）与 PANIC（`:582-583`）都是**先判硬约束**
   （能不能装下 / 来不来得及），再谈平顺 —— 也不是打分。

### 5.4 并列（tie）行为与可复现性

`min()` 并列时返回**第一个**。为保证"第一个"是确定的，代码在两处强制排序：

* `group_walls` 的 `for key in sorted(buckets)`（`:250`）—— 决定 `walls` 顺序（C++ 移植陷阱，见注释 `:246-249`）。
* `group_walls` 的 `members = sorted(members, key=lambda r: r.x)`（`:252`）。
* `reconstruct_aperture` 的 `rects = sorted(wall.rects, key=lambda r: _dot((r.x, r.y), wall.u_hat))`（`:349`）。

链条：`sorted(buckets)` → `walls` 顺序 → `cands` 顺序 → `min()` 的并列胜负。

`CPP_PORT.md` 也把这条列为必须处理的陷阱，要求 C++ 侧给自定义 strict weak ordering，
并**不要用 `unordered_map`** 存桶。

---

## 6. 全向速度预算 `u = √(v² − w²)`

### 6.1 代码里是**真的开方**（两处）

`gap_avoid.py:590-591`（`front_back=True` 路径）：

```python
    u_speed = u_mag * v_max
    w_cap = math.sqrt(max(0.0, v_max * v_max - u_speed * u_speed))  # 剩余速度预算
```

`gap_avoid.py:608-610`（`front_back=False` 路径）：

```python
        w = min(cfg.fwd_speed * v_max, v_max)
        u_cap = math.sqrt(max(0.0, v_max * v_max - w * w))
        u_mag = min(u_mag, u_cap / max(v_max, EPS))
```

**结论**：

* 是 `math.sqrt`，**没有"避免开方"的写法**（没有在平方域里做比较、没有 `inv_sqrt` 近似、
  没有查表）。两处都先 `max(0.0, ·)` 防止负数开方。
* 名字对照：`u` ↔ 横向（沿 `û`）分量，`w` ↔ 前后（沿 `n̂`）分量；`v_max = player[P_SPEED]`。
* 注意 `math.hypot` 内部也是开方（`:211`、`:219`、`:620`、`:629`、`:663`），
  `CPP_PORT.md` 特别提醒 `hypot`/`atan2`/`cos`/`sin` **不要求正确舍入**，跨语言不能要求逐位一致。

### 6.2 预算的两种用法方向相反

| 路径 | 逻辑 | 结果 |
|---|---|---|
| `front_back=True`（`:590-591`） | 先定横向 `u_mag` → 反推**还剩多少前后** `w_cap` | 横向满速时 `w_cap = 0`（车完全横着走，不前冲） |
| `front_back=False`（`:608-610`） | 先定固定油门 `w` → 反推**还剩多少横向** `u_cap` → 反过来钳 `u_mag` | `fwd_speed=1.0` 时 `u_cap = 0`，横向能力归零 |

第 2 行有实测佐证：`fwd_speed=1.0`、车在缺口左侧 120 单位时，**指令退化为 `(0, +1)`，完全没有横向分量**：

```
tier=CRUISE  cmd=(+0.0000,+1.0000)  |cmd|=1.0000   usable_half=34.00
```

### 6.3 合成后的模长恒等式

因为 `û ⊥ n̂`、`fwd = −n̂`，`:615-616` 的合成满足：

    |cmd| = √(u_mag² + w_frac²)

而 `front_back=True` 时 `w_frac = w_cap/v_max = √(1 − u_mag²)`，所以 `|cmd| = 1`（精确）。
实测扫掠（lat 0→640 每 10 一档 × 4 个 py × 3 种 cfg，墙带法向速度）：

```
max |cmd| over sweep = 1.000000000000 ; violations = 0
```

逐点核对（`front_back=True`）：`lat=150 → |cmd|=1.000000000000`、`lat=333(TRIM) → 1.000000000000`。
`front_back=False, fwd_speed=0` 时 `|cmd| = u_mag ≤ 1`。

### 6.4 边界缺陷

`gap_avoid.py:614` 用的是**裸除法**，没有 `max(v_max, EPS)` 保护：

```python
    w_frac = w / v_max
```

`P_SPEED == 0` 时直接抛异常（实测）：

```
  File "core/cpu/gap_avoid.py", line 614, in plan
    w_frac = w / v_max
ZeroDivisionError: float division by zero
```

同文件其它地方都用了 `max(v_max, EPS)`（`:559`、`:561`、`:610`），只有这一处没有。

---

## 7. 转向速率限幅 `max_turn_rate`

### 7.1 代码原文（`gap_avoid.py:618-630`）

```python
    # 转向限速：把这一帧的方向相对上一帧的夹角限住。这是"真车做不出来"的那一步。
    if cfg.max_turn_rate > 0.0 and mem.last_dir is not None:
        mag = math.hypot(wx, wy)
        if mag > EPS:
            a_new = math.atan2(wy, wx)
            a_old = math.atan2(mem.last_dir[1], mem.last_dir[0])
            d = wrap_pi(a_new - a_old)          # 归一化到 [-pi, pi)，见 wrap_pi 注释
            lim = math.radians(cfg.max_turn_rate) * dt
            if abs(d) > lim:
                a_new = a_old + math.copysign(lim, d)
                wx, wy = mag * math.cos(a_new), mag * math.sin(a_new)
    if math.hypot(wx, wy) > EPS:
        mem.last_dir = (wx, wy)
```

### 7.2 算法（公式）

1. **取极角**：`α_new = atan2(w_y, w_x)`，`α_old = atan2(last_dir_y, last_dir_x)`；幅值 `m = hypot(w_x, w_y)`。
2. **最短路径角差**：`Δ = wrap_pi(α_new − α_old) = ((α_new − α_old) + π) mod 2π − π`。
   值域 `[−π, π)`。**"最短路径"就来自这一步**：把任意角差折进半开区间后，
   `|Δ| ≤ π`，符号即转向方向（正 = 逆时针）。例如角差 `+π`（180° 翻转）在本文件按
   **`wrap_pi(π) = −π`** 处理，即"负方向转 180°"。
3. **单帧允许角**：`lim = (π/180) · max_turn_rate · dt`（`max_turn_rate` 单位是**度/秒**，`dt` 是秒）。
4. **钳位**：若 `|Δ| > lim`，则
   `α_new ← α_old + sign(Δ)·lim`，再 `(w_x, w_y) ← (m·cos α_new, m·sin α_new)`。
   **幅值 `m` 被完整保留**，只旋转方向。

### 7.3 生效条件与状态更新

* 只有 `cfg.max_turn_rate > 0.0` **且** `mem.last_dir is not None`（至少下发过一次非零指令）才生效。
  默认 `max_turn_rate = 0.0` ⇒ **默认不限速**（保持旧行为）。
* `mem.last_dir` 只在**本帧指令模长 > EPS** 时更新（`:629-630`）。
  因此零指令（如 HOLD）**不会**刷新参考方向 —— 一段 HOLD 之后，参考方向仍是 HOLD 之前那个。
* 限幅是**指令层**而非动力学（注释 `:154-155`）：不建模最小转弯半径、不建模横向加速度。

### 7.4 实测验证

场地 640×480，`dt = 1/120`，`max_turn_rate = 720` 度/秒 ⇒ `lim = 720 × (1/120) = 6.0000°`。
构造"目标墙换边导致指令 180° 翻转"的两帧序列：

```
k=0  tier=ALIGN  cmd=(+1.0000,+0.0000)  angle=+0.00°   last_dir=(1.0, 0.0)
k=1  max_turn_rate=720 : cmd=(+0.9945,-0.1045)  angle=-6.00°   |cmd|=1.0000
k=1  max_turn_rate=0   : cmd=(-1.0000,+0.0000)  angle=+180.00°
```

即：不限速时方向瞬间翻 180°，限速 720°/s 时只转了 `lim = 6°`（注意方向是 **−6°**，
符合 `wrap_pi(π) = −π` 的"负方向"约定），且**模长保持 1.0000**。

### 7.5 `wrap_pi` 的移植陷阱（`:83-103`）

    wrap_pi(a) = ((a + π) mod 2π) − π

Python `%` 对负数返回非负；C++ `std::fmod` 保留符号。`a_new − a_old ∈ [−2π, 2π]`，
所以 `a < −π` 一定会遇到。C++ 必须写成：

```cpp
double wrap_pi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;      // 关键：fmod 可能是负的
    return a - M_PI;
}
```

---

## 8. 全部可调参数与默认值

### 8.1 `AlgoConfig`（11 个字段，`gap_avoid.py:106-156`）

| 参数名 | 默认值 | 单位 | 作用（代码位置） | 改大 / 改小会怎样 | 是否真的生效 |
|---|---|---|---|---|---|
| `safety_margin` | `6.0` | 世界单位 | 配置空间膨胀量（`:480` 的 `clearance`；`:483` 传入 `reconstruct_aperture`，在 `:396` 参与 `usable_half = usable_w/2 − r_p − margin`） | **改大**：`A` 变小 → 缺口更容易"装不下" → 更多 NO_GAP / ALIGN，更保守；**改小**：更激进，贴墙更近 | ✅ 生效（两处，其中 `usable_half` 影响所有分档判定） |
| `max_window` | `8.0` | 秒 | 只用于 `H = min(horizon_remaining, max_window)`（`:536`） | **无论改大改小都没有影响** | ❌ **死参数**（`H` 无引用） |
| `align_frac` | `0.6` | 无量纲（相对 `A` 的比例） | TRIM 与 ALIGN 的分界（`:574`） | **改大**：TRIM 区间变宽 → 更多比例微调、更少满速横移；**改小**：更多 ALIGN 满速 | ✅ 生效 |
| `dead_frac` | `0.05` | 无量纲（相对 `A` 的比例） | HOLD 死区（`:572`），防抖 | **改大**：死区更宽，输出更稳但可能停得离缺口中心更远；**改小**：更频繁微调 | ✅ 生效 |
| `allow_cross` | `False` | 布尔 | 注释说"是否允许穿缺口到墙另一侧"（`:117-118`） | 改真/改假都没有影响 | ❌ **死参数**（全文件无读取点） |
| `aim_lookahead` | `0.25` | 秒 | 注释说是"瞄准用的预测时长"（`:119-130`） | 改大/改小都没有影响；实际瞄准时刻由 `:559-563` 的拦截迭代 + `t_arr` 钳位决定 | ❌ **死参数**（全文件无读取点；注释 `:551-552` 与代码不符） |
| `front_back` | `False` | 布尔 | 是否允许前后速度控制（`:582` 抑制 PANIC；`:592` 选分支） | **改真**：启用 `w_cap` 预算与 SLOW/STOP 逻辑，可减速/停住等对齐，PANIC 被降级为 ALIGN；**改假**：禁止前后控制，横向只能用剩余预算 | ✅ 生效 |
| `fwd_speed` | `0.0` | 无量纲（`v_max` 的比例，0~1） | 仅 `front_back=False` 时的固定油门（`:608`、`:611`） | **改大**：前进更快但 `u_cap = √(v_max²−w²)` 变小 → 横向能力下降；**`=1.0` 时横向能力归零**（实测指令退化为纯前向）；**改 0**：纯横向、不动前向 | ✅ 生效（仅 `front_back=False`） |
| `hold_clearance` | `10.0` | 世界单位 | 对齐时与墙保持的净距离，`room = max(0, d_n − hold_clearance)`（`:594`） | **改大**：更早开始减速、更容易 STOP，更保守；**改小**：更贴近墙才减速 | ✅ 生效（仅 `front_back=True`） |
| `forward` | `None` | 单位向量（无量纲） | 固定行驶轴（`:439` → `group_walls` 的 `:263-268`），钉死 `n̂ = −forward` | 给了它 → 不再靠"车在墙哪一侧"判法向，消除静态墙下前进方向抖动；`None` → 自动判 | ✅ 生效 |
| `max_turn_rate` | `0.0` | **度/秒** | 指令方向最大角速度（`:619`、`:625`） | **改大**：转向更快，接近不限速；**`0` = 不限**；**改小**：轨迹更平滑但可能跟不上目标切换 | ✅ 生效（`=0` 时不限） |

### 8.2 模块级常量（`gap_avoid.py:59-65`）

| 常量 | 值 | 单位 | 作用 |
|---|---|---|---|
| `P_X … P_ALIVE` | `0 … 6` | — | `player` 向量下标（`:59`） |
| `SHAPE_RECT` | `1` | — | 矩形形状码（圆 = 0）。**本文件内无读取点**（仅供适配层使用） |
| `EPS` | `1e-9` | 混合 | 除零保护 / 零向量判据 / `den` 退化判据（`:463`、`:525`、`:559`、`:561`、`:610`、`:621`、`:629`） |
| `TOL` | `1e-6` | 世界单位 | 几何比较容差。注释 `:63-64` 解释：实测缺口宽度会算成 `200.00000000000003` 而自由空间是 `200.0`，直接用 `>=` 会因 3e-14 的误差误判"放不下"，算法整帧摆烂。用于 `:256`、`:269`、`:376`、`:378` |

### 8.3 硬编码的"隐形参数"（不在 `AlgoConfig` 里）

| 值 | 位置 | 含义 | 单位 |
|---|---|---|---|
| `24` | `:284` `deque(maxlen=24)` | 缺口中心历史长度（帧） | 帧 |
| `24.0` | `:304` `match_keys(tol=24.0)` | 跨帧墙关联容差 | 世界单位 |
| `2.0` | `:243` `int(round(band / 2.0))` | 分桶的墙带量化档距 | 世界单位 |
| `1000.0` | `:243` `round(r.rotation * 1000.0)` | 分桶的旋转量化（1e-3 rad） | 无量纲 |
| `40.0` | `:501` | NO_GAP 逃逸的边界预判距离 | 世界单位 |
| `4` | `:560` `for _ in range(4)` | 拦截时间迭代次数（固定，无收敛判据） | 次 |
| `1e-6` | `:593` `need > 1e-6` | 判定"需要横向对齐"的门限 | 世界单位 |
| `2` | `:278` `members[:2]` | **一堵墙最多保留的矩形段数** | 段 |
| `1` | `:297` `_next_key: int = 1` | wall key 起始值 | — |

---

## 9. `to_command()`：`Decision` → 归一化速度向量

`gap_avoid.py:658-665`：

```python
def to_command(dec: Decision) -> tuple:
    """Decision -> 归一化速度指令 (wx, wy)，模长 <= 1。零向量 = 不动。

    bullet_sim 里再包一层：Action.from_vector((wx, wy))
    """
    if math.hypot(dec.wx, dec.wy) < EPS:
        return (0.0, 0.0)
    return (dec.wx, dec.wy)
```

行为：

1. **只做一件事：零向量清洗**。若 `hypot(wx, wy) < EPS = 1e-9`，返回 `(0.0, 0.0)`；否则**原样返回** `(wx, wy)`。
2. **不做归一化、不做缩放、不做钳位** —— "模长 ≤ 1" 是 `plan()` 里构造出来的不变式（§6.3），
   `to_command()` 只是把它交出去。
3. 后续包装：`bullet_sim` 侧再包一层 `Action.from_vector((wx, wy))`（文档字符串 `:661`）。
4. 语义：返回的 `(wx, wy)` 是**归一化速度**；乘上 `player[P_SPEED]` 得到世界速度（文件头 `:18`。
   注意文件头 `:18` 写的是"乘上 `player[P_SPEED]` 就是世界速度"）。
5. 零向量语义 = "不动"（不是"刹车"）；在 HOLD + `front_back=False` + `fwd_speed=0` 时就是这种输出。

---

## 10. 依赖审计：有没有 numpy / 第三方库？

### 10.1 逐条确认（`gap_avoid.py:51-56`）

```python
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence
```

| import | 行 | 归类 | 说明 |
|---|---|---|---|
| `from __future__ import annotations` | `:51` | **stdlib**（`__future__`） | 让所有注解变成惰性字符串；因此 `tuple[float, float]`、`dict[int, WallMemory]` 等内建泛型在运行时**不求值** |
| `import math` | `:53` | **stdlib**（C 加速模块） | 用到 `pi`、`sqrt`、`hypot`、`cos`、`sin`、`atan2`、`copysign`、`radians`、`isfinite`、`inf`、`nan` |
| `from collections import deque` | `:54` | **stdlib** | `WallMemory.ring = deque(maxlen=24)`（`:284`） |
| `from dataclasses import dataclass, field` | `:55` | **stdlib** | 全部结构体 |
| `from typing import Any, Optional, Sequence` | `:56` | **stdlib** | 仅注解：`Any`（`:427`）、`Optional`（`:149/231/285/286/295/299/339`）、`Sequence`（`:70`、`:79`、`:192`、`:230`、`:304`、`:338`） |

**没有 numpy，没有任何第三方库。** 全文件仅有的 `numpy` 字样是文档字符串 `:37` 的一句声明
（"依赖：**只有 stdlib**（math / collections / dataclasses / typing），无 numpy"）。

### 10.2 实测验证

**(a) 独立加载（不经过 `cpu` 包）后检查 `sys.modules`**：

```
standalone load OK: /home/zhang/FPGA/core/cpu/gap_avoid.py
numpy loaded: False
any bullet_sim loaded: False
any cpu.* loaded: []
```

**(b) 直接当脚本跑自检**（`:668-674`）：

```
$ python3 core/cpu/gap_avoid.py
gap_avoid.py 自检通过：空场景 -> IDLE (0.0, 0.0)     exit=0
```

### 10.3 两个必须说清的例外（否则文档会误导）

**例外 1：通过包导入会连带拉进 numpy —— 但不是这个文件的锅。**

```python
import cpu.gap_avoid     # -> numpy loaded: True
```

原因链：`core/cpu/__init__.py` 顶部执行 `from cpu.pipeline import (...)`，
而 `cpu/pipeline.py` 依赖 numpy + `bullet_sim`。这是**包级 `__init__` 的副作用**，
`gap_avoid.py` 本身仍然零依赖。要拿到"零依赖"的保证，应当直接加载该文件（或从
`core/cpu/` 单独搬走该文件）—— 文件头 `:37-40` 与 `core/cpu/README.md:7-8` 的
"整个 `core/cpu/` 可以独立搬走"就是这个意思。

**例外 2：PEP 562 懒 re-export 会按需拉进仿真适配层。**

`gap_avoid.py:651-655` 的模块级 `__getattr__` 在访问 `rect_from_pool` / `observe_rects` /
`view_from_world` 这 3 个名字时才 `from cpu.adapters import sim_adapter`。实测：

```
numpy before lazy access: False
view_from_world -> cpu.adapters.sim_adapter
numpy after lazy access:  True
bullet_sim loaded:        True
```

不访问这 3 个名字 ⇒ 不拉进任何东西。`CPP_PORT.md` 明确说这三个名字**不用带到 C++ 侧**。

### 10.4 运行时要求

* `core/cpu/README.md:205` 声明 **Python ≥ 3.10**，实测环境 3.12。
* 文件可单独执行自检，**不需要 pygame、不需要 numpy**（`core/cpu/README.md:208-210`）。
* 包导入路径：仓库约定 `core/` 在 `sys.path` 上，包名是 `cpu`。

---

## 11. 死代码、不可达分支与边界情况（诚实清单）

### 11.1 死代码（本文件内）

| 符号 | 行 | 状态 |
|---|---|---|
| `H` | `:536` | 赋值后**无任何引用** |
| `feasible(t)` | `:544-546` | **从未被调用** |
| `worst_offset(t)` | `:538-539` | 仅被 `feasible` 调用 ⇒ 事实死亡 |
| `cfg.max_window` | `:112` | 只喂给 `H` ⇒ 无行为影响 |
| `horizon_remaining` | `:427` 入参 | 只喂给 `H` ⇒ **对本算法无行为影响** |
| `cfg.allow_cross` | `:118` | 全文件无读取点 |
| `cfg.aim_lookahead` | `:130` | 全文件无读取点（仅注释 `:551` 提及） |
| `SHAPE_RECT` | `:61` | 本文件无读取点 |
| `Wall.key` | `:186-189` | 本文件无读取点 |
| `Rect.ent_id` | `:163` | `plan()` 路径上不读取（仅未使用的 `Wall.key` 用） |
| `Rect.angular_velocity` | `:171` | 全文件无读取点 |
| `player[P_ALIVE]` | `:59` | 全文件无读取点 |
| `WorldView.field_h` | `:76` | 全文件无读取点 |
| `GapMemory.step_index` | `:296` | 写入（`:437`）但不读取 |
| `_rect_sdf` 的梯度返回值 | `:215-227` | 调用点都只取 `[0]`（`:479`、`:498`） |

### 11.2 不可达 / 语义存疑的分支

1. **`SLOW` 不可达**（证明见 §4.3，40,000 次扫描未出现）。`else` 分支实际只会产出 `STOP`。
2. **`CRUISE` 覆盖 tier 标签**：`front_back=False` 且 `fwd_speed > 0` 时，`:611-612` 无条件
   把 `tier` 改写成 `CRUISE`，连 `PANIC` 也被覆盖（`reason` 保留原串，于是会出现
   `tier=CRUISE` 但 `reason='unreachable_before_arrival'` 的组合 —— 实测已观察到）。
3. **注释与代码不一致**：`:551-552` 说瞄准时刻取 `min(t_arr, aim_lookahead)`，
   代码 `:562` 实际是 `min(t_arr, cand)`，且 `aim_lookahead` 从未被读取。
4. **`P_SPEED == 0` 崩溃**：`:614` 的 `w / v_max` 无保护，实测 `ZeroDivisionError`。
5. **`Wall.rects` 截断**：`:278` 的 `members[:2]` 会丢弃第 3 段及以后的矩形；
   一堵由 3 段共线矩形构成的墙（两个缺口）只会保留前两段。
6. **`prior=0`（正压墙带）时 `key_d = 0.0`**：多堵墙同时"正压"时，第二级键全部为 0，
   胜负完全由 `cands` 顺序（即 `sorted(buckets)` 的顺序）决定。

### 11.3 数值容差的由来（`gap_avoid.py:63-65`）

```python
#: 几何比较的容差。实测缺口宽度会算成 200.00000000000003 而自由空间是 200.0，
#: 直接用 >= 会因为这个 3e-14 的误差判成"放不下"，算法就整帧摆烂。
TOL = 1e-6
```

`TOL` 用在 4 处：`u_hat` 规范化判据（`:256`）、`n̂` 退化判据（`:269`）、
单段缺口的左右侧推断（`:376`、`:378`）。

---

## 12. 实测核对数据（可直接引进文档）

**统一场景**：场地 640×480 世界单位，`dt = 1/120` s，玩家 `r_p = 10`、`v_max = 200`；
缺口墙两根矩形 `half_w=135, half_h=10, rotation=0`，中心 y=300，缺口 `x ∈ [270, 370]`（宽 100）；
`n̂ = (0, −1)`（从墙指向车，车在 y < 300 一侧），`û = (1, 0)`；
`safety_margin = 6` ⇒ `usable_half = 50 − 10 − 6 = 34`。

| 场景 | 配置 | 观察结果 |
|---|---|---|
| 车在缺口左侧 120 单位，静止 | `fwd_speed=0` | `tier=ALIGN reason=move_to_gap dlat=+120.0 cmd=(+1.0000,+0.0000) t_arr=inf clr=174.0 s_hat=0.00` |
| 车正在缺口中心 | `fwd_speed=0` | `tier=HOLD reason=gap_will_pass_me cmd=(+0.0000,+0.0000)` |
| 车在 `lat=340`（偏离中心 20 单位，`|d_lat|/A = 20/34 = 0.588`） | `fwd_speed=0` | `tier=TRIM reason=recenter dlat=-20.0 cmd=(-0.5882,+0.0000) \|cmd\|=0.5882` |
| 同上 `lat=340` | `front_back=True` | `tier=TRIM cmd=(-0.5882,+0.8087) \|cmd\|=1.0000`（横向 0.588，前向吃满剩余预算 `√(1−0.588²)=0.809`） |
| 车在 `lat=333`（偏离 13 单位，`u_mag = 13/34 = 0.382`） | `front_back=True` | `tier=TRIM cmd=(-0.3824,+0.9240) \|cmd\|=1.0000` |
| 车在缺口中心 | `front_back=True` | `tier=HOLD cmd=(+0.0000,+1.0000)`（`need=0 → t_align=0 → w=w_cap=v_max`，满油门穿过去） |
| 车靠近墙但对不准 | `front_back=True` | `tier=STOP reason=hold_to_align need=136.0 t_align=0.68 cmd=(+1.0000,+0.0000)` |
| 高速逼近墙（`vy=-1000`），车静止在左侧 | `fwd_speed=0` | `tier=PANIC reason=unreachable_before_arrival t_arr=0.1800` |
| 同场景 | `front_back=True` | `tier=ALIGN`（PANIC 被降级，`:598-599`） |
| 缺口膨胀后装不下（gap=20 ⇒ `A=-6`） | 任意 | `tier=NO_GAP reason=aperture_unknown_flee cmd=(-1.0000,-0.0000)` |
| 只有一段可见且无记忆、无法推断方位 | 任意 | `usable_half=-inf tier=NO_GAP reason=aperture_unknown_flee` |
| 只有一段可见但上一帧两段都见过 | 任意 | `gap=100.0 e=[270.0,370.0] usable_half=34.0 n_rects=1 tier=ALIGN`，`n_single_frames=1` |
| 缺口墙以 60 单位/秒沿 x 滑动 | `fwd_speed=0` | `s_hat=60.00`（最小二乘斜率精确收敛），`aim_lat` 从 320.00 → 341.76 → 345.29（拦截点而非当前位置） |
| 两帧序列：目标墙换边导致指令要翻 180° | `max_turn_rate=720` | 单帧方向变化恰为 `6.0000°`（`= 720 × dt`），模长保持 `1.0000`；不限速时为 `180.00°` |

**速度预算恒等式**（`front_back=True`）：`lat=150 → |cmd|=1.000000000000`、
`lat=250 → 1.000000000000`、`lat=300(TRIM) → 1.000000000000`、`lat=333(TRIM) → 1.000000000000`。
全参数扫掠 `max |cmd| = 1.000000000000`，**越界 0 次**。
