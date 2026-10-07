# `gap_avoid.py` → C++ 移植检查表

> 适用范围：**只有 `core/cpu/gap_avoid.py` 需要移植**。
>
> | 文件 | 移植？ |
> |---|---|
> | `cpu/gap_avoid.py` | ✅ **这是要移植的全部**（645 行，纯 stdlib，单入口 `plan()`） |
> | `cpu/adapters/sim_adapter.py` | ❌ 仿真专用胶水，整份删掉 |
> | `cpu/adapters/vision_adapter.py` | ❌ 视觉胶水，C++ 侧自己写（见 §6） |
> | `cpu/decision/**`、`cpu/pipeline.py` | ❌ 不参赛，不移植 |
>
> 判据：移植完成后，**同一输入在 Python 与 C++ 上得到相同的行为**。
>
> ⚠️ 注意"相同"的**准确含义**（这一点很容易想当然）：
>
> | 量 | 能不能要求逐位相同 | 原因 |
> |---|---|---|
> | `tier` / `reason` / 各分支的走法 | ✅ **可以，应当** | 全是比较与整数逻辑 |
> | `+ - * /`、`sqrt` | ✅ 可以 | IEEE-754 要求正确舍入，两侧一致 |
> | `hypot` / `atan2` / `cos` / `sin` | ❌ **不能** | 这些函数**不要求正确舍入**，CPython 的 `math.hypot` 还有专门的防溢出标度算法，glibc 实现不同 → 末位可能差 1 ulp |
>
> 所以验收标准应当是：**`tier` 逐位一致 + 浮点量相对误差 < 1e-9**。
> 想拿到真正的逐位一致只有两条路：① 在 C++ 里把 `hypot`/`atan2` 也实现成和 CPython
> 完全相同的算法（不值得）；② **走定点**——这本来就是上 FPGA 的必经之路，建议合并到那一步做。
>
> 这和平台的 `fpga/prediction_block.py` 是同一个模式——Python golden model 定契约，
> 实现只要对上就是对的。

---

## 1. 数据结构映射

| Python | C++ | 备注 |
|---|---|---|
| `@dataclass(frozen=True) class Rect` | `struct Rect { int32_t ent_id; double x, y, vx, vy, half_w, half_h, rotation, angular_velocity; };` | 9 个字段，值语义 |
| `@dataclass class WorldView` | `struct WorldView { const double* player; const Rect* rects; size_t n_rects; double dt; int64_t step_index; double field_w, field_h; };` | `player` 是长度 7 的数组 |
| `@dataclass(frozen=True) class AlgoConfig` | `struct AlgoConfig`（同字段同名） | 10 个字段全部有默认值 |
| `@dataclass class GapMemory` | `struct GapMemory` | 含环形缓冲与关联表，见 §3 |
| `@dataclass class Decision` | `struct Decision`（同字段同名） | `math.inf` → `INFINITY`，`math.nan` → `NAN` |
| `deque(maxlen=24)` | `struct Ring { double t[24]; double s[24]; int n, head; }` | 固定容量，无动态分配 |
| `dict[int, WallMemory]` | 定长数组 + 线性查找（墙数量个位数） | **不要用 `unordered_map`**，见 §2.2 |
| `dict[tuple, list[Rect]]`（分桶） | 定长数组 + 排序（`std::sort`） | 同上 |
| `tuple[float, float]` | `struct Vec2 { double x, y; };` | |
| `Optional[tuple]` | `bool has_forward; Vec2 forward;` | 避免 `std::optional` 的额外语义 |
| `math.hypot(a,b)` | `std::hypot(a,b)` | 注意 `hypot` 比 `sqrt(a*a+b*b)` 慢但更稳；精度要对齐 |
| `math.atan2/copysign/cos/sin/radians` | `std::atan2/std::copysign/std::cos/std::sin` + `M_PI/180.0` | |
| `math.isfinite(x)` | `std::isfinite(x)` | |
| `math.inf` | `std::numeric_limits<double>::infinity()` | |
| f-string 诊断串 | `snprintf` 或直接不做 | `reason` 只是给人看的 |

---

## 2. 三个**会改变行为**的陷阱（重点）

### 2.1 负数取模：Python `%` ≠ C++ `fmod`

**位置**：`gap_avoid.py` 的 `wrap_pi()`（转向限幅用）

```python
d = wrap_pi(a_new - a_old)      # (angle + pi) % (2*pi) - pi
```

差别只在 `angle < -pi` 时暴露，但 `a_new - a_old` 的范围是 `[-2pi, 2pi]`，所以**一定会遇到**：

```
angle = -4*pi
Python  : (-4pi + pi) % 2pi - pi = (-3pi) % 2pi - pi = pi - pi = 0
C++ 朴素 : fmod(-3pi, 2pi) - pi  = -pi - pi        = -2pi      ← 错
```

**必须这样写**：

```cpp
static inline double wrap_pi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;        // 关键：fmod 可能是负的
    return a - M_PI;
}
```

**验证**：`cpu/tests/test_cxx_portability.py::test_wrap_pi_differs_from_the_naive_fmod_translation`
——它当场证明"朴素翻译"会越界，防止以后有人"简化"掉这个函数。

> 归一化区间是 **`[-pi, pi)`**（半开）。`wrap_pi(pi) == -pi`。

### 2.2 容器迭代顺序：argmin 并列时的胜负

**位置**：`group_walls()` 的分桶遍历

```python
for key in sorted(buckets):          # 已改为显式排序
    members = buckets[key]
```

原因链：

```
buckets.values() 的顺序  →  walls 的顺序  →  cands 的顺序
                                          →  min(cands, key=lambda c: (c[8], c[9]))
```

Python 的 `min` 在**并列时返回第一个**，而 Python 的 `dict` 迭代顺序 = 插入顺序。
C++ 的 `std::unordered_map` 迭代顺序与插入顺序**无关** → 同一输入可能选出不同的墙。

**做法**：C++ 侧不要用 `unordered_map` 存桶；要么 `std::map`（有序），
要么收集到 `std::vector` 后 `std::sort` —— 与 Python 的 `sorted(buckets)` 语义对齐。

**验证**：`test_plan_is_independent_of_rect_order`（换障碍顺序，决策必须逐位相同）。

### 2.3 元组字典序比较 → 需要自定义比较器

**位置**：`plan()` 里选目标墙

```python
min(cands, key=lambda c: (c[8], c[9]))       # (prio, key_d) 字典序
```

C++ 没有元组 `operator<` 语义可依赖，必须显式给 strict weak ordering：

```cpp
auto better = [](const Cand& a, const Cand& b) {
    if (a.prio != b.prio) return a.prio < b.prio;
    return a.key_d < b.key_d;                 // 与 Python 元组比较等价
};
const Cand& pick = *std::min_element(cands, cands + n, better);
```

**注意**：`std::min_element` 在并列时返回**第一个**，与 Python `min` 一致 ——
前提是 `cands` 的填充顺序一致（由 §2.2 保证）。

---

## 3. 无动态分配（上板前提）

`plan()` 目前每帧分配：`buckets` 字典、`walls` 列表、`cands` 列表、`sorted` 的临时列表、
若干闭包。C++ 侧应全部换成**调用方持有的定长缓冲**：

| 分配点 | 上界 | 建议 |
|---|---|---|
| 分桶 | 矩形数 `n_rects` | `Bucket buf[MAX_RECTS]`，`MAX_RECTS` 取平台池容量 |
| `walls` | ≤ `n_rects` | `Wall walls[MAX_RECTS]`（实测墙数是个位数） |
| `cands` | ≤ `walls` | 同上 |
| `WallMemory.ring` | 24 | `Ring`（定长） |
| 墙身份表 | ≤ 20 | `Key prev[20]; int n_prev;` 线性匹配（原来是 `deque` + 线性查找） |
| 闭包 `s_lo/s_hi/worst_offset/reach` | 0 | 直接写成立即数或 `static inline` 函数 |

**目标**：`plan()` 稳态零分配 —— 这和平台"内核无分配（障碍池按容量预分配）"是同一条纪律，
也是 `alloc_per_step_bytes ≈ 4 B` 那个实测指标的来源。

---

## 4. 定点化（若最终要上 FPGA）

`gap_avoid.py` 目前全程 `double`。上板前需要定标，参考 `optical-flow-fpga` 的做法：

| 量 | 建议 | 理由 |
|---|---|---|
| 坐标 `x/y/half_w/half_h` | `S16.15`（1 符号 + 16 整数 + 15 小数） | 场地 640×480，整数位 16 够；小数位 15 保证 1e-4 精度 |
| 速度 | `S16.15` | 与坐标同标度，乘 `dt` 后仍需精度 |
| 角度 `rotation` | `S1.15`（归一化到 `[-1,1)` 表示 `[-pi,pi)`）or `S16.15` | 三角函数查表需要定点相位 |
| `cos/sin/atan2` | **查表 + 线性插值**（ROM） | 参考 `Image-Processing-Toolbox` 的 COE → ROM 做法 |
| `hypot` | `sqrt` 用逐位逼近或 CORDIC | 注意 `hypot` 的溢出保护在定点里要靠标度保证 |
| 比较 / argmin | **整数比较** | 字典序判据天然适合整数，这也是它比加权和更适合硬件的原因 |

**降精度前必须先做误差预算**：`usable_half = 可用宽度/2 − r_p − margin`，
而判据是 `usable_half >= 0`。若 `usable_half` 的定点误差达到 `margin` 量级，
就会出现"该过的缺口判成过不去"。建议先量化：

```
定标后 usable_half 的绝对误差  ≤  0.1 × margin
```

**验证方式**：与 Python golden model 按上面的口径比对（`tier` 逐位 + 浮点量 `1e-9` 相对误差），
用平台 `fpga/prediction_block.py` 的同一套思路：`same input frame → same output`。

---

## 5. 移植顺序（建议）

1. **先移植 `wrap_pi` + `_dot` + `_half_extent_along` + `_rect_sdf`** —— 四个纯函数，最容易对；
2. **`group_walls`** —— 注意 §2.2 的排序；
3. **`reconstruct_aperture`** —— 只有一段可见的分支要保留（实测约 4% 的帧）；
4. **`plan()` 的判据段**（目标墙选择 / 瞄准 / 可行性 / 分档）；
5. **`to_command`** —— 平凡。

每步都用同一批输入跑 Python 与 C++，逐位比对 `(wx, wy, tier, gap)`。
平台的 `cpu/gap_wall_demo.py --headless --episodes N` 可以直接生成这批输入
（它已经能在 5 局里给出 0 碰撞的基线）。

---

## 6. C++ 侧自己写的那部分：视觉输入

`cpu/adapters/vision_adapter.py` **不要移植**——它的一半价值是"用 Python 的鸭子类型
同时吃 dataclass / dict"，C++ 里没有对应物。

C++ 侧需要的只是它定义的三件事：

| 语义 | 要求 |
|---|---|
| `Rect` 从哪来 | 视觉输出里 `shape == rect` 且 `valid && !occluded`（`score` 低于门限的丢弃） |
| `dt` 从哪来 | **两帧 `stamp` 之差**，绝不用帧号之差；首帧用配置默认值 |
| `field_w/field_h` 从哪来 | **配置**（视觉帧不携带，标准 §6） |
| `player[5]`（速度上限） | 视觉侧常常不知道 → 用配置默认值，并在日志里标出"用了默认" |

对应的错误处理也要照搬：**坐标没标定（`units="px"` 且无 `H_field_from_image`）必须报错**，
不能当世界坐标用 —— 这是"静默错坐标"的唯一防线。

---

## 7. 已完成的改造（本仓库内可核对）

| 改造 | 位置 | 验证 |
|---|---|---|
| 仿真胶水搬出算法文件 | `cpu/adapters/sim_adapter.py` | `test_sim_adapter_is_where_the_platform_glue_lives` |
| 旧导入保持可用（PEP 562 懒 re-export） | `gap_avoid.py` 末尾 `__getattr__` | `test_legacy_imports_still_work` |
| `wrap_pi()` 集中取模语义 + C++ 注释 | `gap_avoid.py` | `test_wrap_pi_differs_from_the_naive_fmod_translation` |
| 分桶遍历显式排序 | `group_walls()` | `test_plan_is_independent_of_rect_order` |
| 顶层依赖只有 stdlib | `gap_avoid.py` | `test_gap_avoid_only_imports_stdlib_at_module_level` |
| 顶层无平台耦合 | `gap_avoid.py` | `test_gap_avoid_has_no_platform_coupling_at_import_time` |

一条命令复核：

```bash
python -m pytest core/cpu/tests/test_cxx_portability.py -q
```
