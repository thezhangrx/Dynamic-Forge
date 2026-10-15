# `gap_avoid.py` 的 C++ 移植（PS 侧跑的那一份）

`core/cpu/gap_avoid.py` 是**唯一的算法真值**（golden model）。本目录是它的
C++17 移植，给板子的 PS（4×Cortex-A7 @1GHz，Ubuntu 20.04 aarch64）用。

配套文档：`core/cpu/docs/CPP_PORT.md`（移植范围、三个行为陷阱、定点化路线）。

---

## 交付标准

| 量 | 口径 | 为什么 |
|---|---|---|
| `tier` / 分支走法 | **逐位一致** | 全是比较与整数逻辑 |
| `+ - * /`、`sqrt` | **逐位一致** | IEEE-754 要求正确舍入 |
| `hypot` / `atan2` / `cos` / `sin` | **相对误差 < 1e-9** | 这些函数**不要求**正确舍入，glibc/编译器的末位可以差 1 ulp |

判据是"**同一输入 → 同一行为**"，不是"源码长得像"。

---

## 文件

| 文件 | 内容 |
|---|---|
| `gap_avoid.hpp` | 数据结构（`Rect` / `WorldView` / `AlgoConfig` / `GapMemory` / `Decision`）+ 函数声明 |
| `gap_avoid.cpp` | 算法本体，逐函数对应 Python 里的同名函数 |
| `gap_avoid_cli.cpp` | **只为交叉验证存在**：吃文本轨迹、吐决策。不是上板程序 |

上板入口是 `core/board/main.cpp`；那边 `#include "gap_avoid.hpp"` 即可。

---

## 速度跟踪移植（`core/vision/motion.py` → `motion.cpp`）

几何帧线协议**不带速度**（障碍记录只有 `id, shape, type, x, y, radius, half_w,
half_h, rotation`），而决策要用 `Rect.vx/vy` 算"墙是不是在逼近"
（`v_n = (v_wall − v_player)·n̂`）。`vision_pkg.vh` 里写着"速度由 PS 从相邻帧
stamp 之差推" —— 这两个文件就是那句话的实现。

| 文件 | 内容 |
|---|---|
| `motion.hpp` / `motion.cpp` | 最近邻关联 + 定增益 α-β 滤波（照抄 `motion.py` 的语义） |
| `motion_cli.cpp` | **只为交叉验证存在**：吃文本序列、吐轨迹与贴好的速度 |

三条与 Python 的口径（见 `tests/test_cpp_motion.py`）：关联结果/计数逐位一致、
`+ - * /` 逐位一致、`hypot`（只用于门限）相对误差 < 1e-9。
**唯一**有意的差异：stamp 不单调时 Python 抛 `ValueError`（宁可炸），C++ 返回
`false` 且不改状态（板子上是实时环，不能崩）—— 这条差异写在测试里，不是"我们知道"。

### 为什么不能按帧里的 `id` 配对

RTL 侧是 `seg_id[seg_k] <= n_obs + 1`，也就是**当帧序号**（按带内顺序 1,2,3…），
下一帧可能换人。所以只能像 `motion.py` 那样按位置做最近邻关联。

### `max_residual`：真实素材上量出来的一个坑

关联门限（`gate=24`）回答的是"**是不是同一个目标**"，不能顺便当成"**这个位移
可信吗**"。实测（`data/vision/run4/screen.mp4` 前 60 帧 @30fps，录屏所以没有
手持抖动）里带合并一变，段的**中心**单帧横移 24 px —— 那是"段变宽了"，不是
"墙在横着飞"。门限 24 恰好放它进来，于是推出来的速度瞬间到 **-66 px/s**，还要
十几帧才衰减掉，而真实运动只有 15~30 px/s：足以让决策误判"墙在远离"。

加 `max_residual = 8`（残差超过它就**只落位、不更新速度**）之后，同一段素材：

| 墙 max|v| | 中位 | 90% 位 | 最大 |
|---|---|---|---|
| 加守卫前 | 15.2 | 50.6 | **67.3** |
| 加守卫后 | 13.8 | 15.2 | **15.9** |

剩下的 13.8 正是从原始位置量出来的真实纵向平移（0.5 px/帧 × 30）。取值依据：
本平台 30fps 下墙的最快步长在 2 单位/帧量级，8 是它的 4 倍；假跳变是 24。
残差对**匀速快目标**本来就小（预测已经吃掉了速度），所以这个阈值不需要随速度放宽。

---

## 零动态分配

`plan()` 稳态**不 new / 不 malloc**，只用调用方持有的定长缓冲：

| 缓冲 | 上界 | 位置 |
|---|---|---|
| `Wall walls[]` | `MAX_WALLS = 64` | `plan()` 栈上 |
| `Cand cands[]` | `MAX_WALLS` | `plan()` 栈上 |
| 桶成员 | `MAX_RECTS = 64` | `group_walls()` 的 `static thread_local` |
| `Wall.rects[2]` | 2 | **按值持有**（不是指针） |
| `Ring` | 24 | `WallMemory` 内 |
| 墙身份表 | `MAX_WALLS` | `GapMemory` 内，线性查找 |

> ⚠ `Wall.rects` 早先写成指向 `group_walls()` 局部数组的 `const Rect*` —— 函数一
> 返回就悬垂。这种 bug 在 `-O0` 下"看起来能跑"，换个优化级别就随机崩。现在按值拷贝。

---

## 构建

```bash
# 只编算法 + CLI（交叉验证用）
g++ -std=c++17 -O2 -Wall -Wextra -ffp-contract=off -Icore/cpu/cpp \
    -o /tmp/gap_avoid_cli core/cpu/cpp/gap_avoid.cpp core/cpu/cpp/gap_avoid_cli.cpp
```

### 为什么必须 `-ffp-contract=off`

GCC 默认会把 `a*b + c` **收缩成一条 FMA**（融合乘加）。FMA 只舍入一次，而分开算
舍入两次 —— 结果不同。`plan()` 里到处是 `u_mag * sgn_lat * û.x + w_frac * fwd.x`
这种式子，一旦收缩，`tier` 就会在判据边界上分家。

`-O0` 下 GCC 默认不收缩，所以**调试构建与发布构建行为不同** —— 这类问题最难查。
写死 `-ffp-contract=off`，两种构建才一致。

### 交叉编译到板子

```bash
aarch64-linux-gnu-g++ -std=c++17 -O2 -Wall -Wextra -ffp-contract=off \
    -Icore/cpu/cpp -c core/cpu/cpp/gap_avoid.cpp -o gap_avoid.o
```

---

## 三个**必须照抄、不能"简化"**的地方

跨平台移植最容易在这里翻车。每一处都有专门的守卫测试，**并且反向验证过**
（改回"朴素写法"，那条守卫当场变红）：

| # | 位置 | 朴素写法 | 为什么错 | 守卫测试 |
|---|---|---|---|---|
| 1 | `py_round_int()` | `std::llround` / `(int)(x+0.5)` | Python 的 `round()` 是**五成双**：`round(0.5)=0`、`round(2.5)=2`；`llround` 是五入。分桶键差 1，同一堵墙会被拆进两个桶 | `test_cpp_port_matches_on_round_half_boundaries` |
| 2 | `wrap_pi()` | `fmod(a+pi,2pi)-pi` | Python 的 `%` 对负数返回**非负**，`fmod` 保留符号。`a_new-a_old ∈ (-2pi,-pi)` 时差 2pi | `test_cpp_port_matches_on_the_wrap_pi_branch_cut` |
| 3 | `std::stable_sort` | `std::sort` | 桶内按 `r.x` 排序，并列时 `members[0].rotation` 决定 û。不稳定排序会换人 | `test_cpp_port_matches_on_stable_bucket_sort` |
| 4 | 分桶键显式排序 | 不排 / 用 `unordered_map` | `min(cands, key=(prio,key_d))` 并列时取第一个，顺序变了结果就变 | `test_cpp_port_is_byte_exact_on_drive` |

> ⚠ 第 3 条的守卫**必须把桶塞到 ≥17 条**：libstdc++ 的 `std::sort` 在 n≤16 时
> 走插入排序，对并列元素**恰好也是稳定的**（实测 n=2..16 完全不打乱，n≥17 全部打乱）。
> 真实场景每堵墙只有 2 段 —— 也就是说，随手造的轨迹永远照不出这个 bug。
> 而且每条矩形的 `rotation` 必须**互不相同**：早先写成"两个值交替"，`std::sort`
> 虽然打乱了顺序，落到下标 0 的却还是同一批，墙一模一样，测试照样全绿。

---

## 怎么验（一条命令）

```bash
python -m pytest core/cpu/tests/test_cpp_gap_avoid.py -q
```

10 条用例，覆盖：incoming / drive / 转向限速 / 倾斜墙 / 边角 / **真场景整局** /
wrap_pi 切割线 / round 的 .5 边界 / 稳定排序 / 反向验证。

---

## 与 Python 对账的**口径**

`tier` 必须逐位相同；浮点量 `rel_err < 1e-9`。

**不要**要求 `wx/wy` 逐位相同 —— 它们中间过了一次 `sqrt`，开转向限速时还要过
`atan2/cos/sin`，这些函数不要求正确舍入。想拿到真正的逐位一致，只有走定点
（见 `CPP_PORT.md` §4），那本来就是上 FPGA 的必经之路。
