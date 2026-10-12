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
