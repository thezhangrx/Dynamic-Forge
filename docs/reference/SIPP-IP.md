# SIPP-IP 参考项目技术总结

> 本文档基于对仓库 `/home/zhang/Bullet_Platform/Outside/Algo/SIPP-IP` 的**只读**通读写成（源码 5 个 `.cpp`、`include/constants.h`、`README.md`、`LICENSE`、`maps/`、`tests/`、`results/`）。
> 所有结论都标注了来源（`文件:行`）。凡源码/文档中没有直接证据的，均显式写"**未确认**"。

---

## 1. 一句话定位

**SIPP-IP（Safe Interval Path Planning with Interval Projection）** 是一个自包含的 C++ 单智能体路径规划参考实现：在"配置图 + 动态障碍给出的安全时间区间"上做 A\* 搜索，并通过**把父节点的等待时间区间投影到后继节点**来正确处理"机器人不能瞬时启停（kinodynamic / 加-减速约束）"的情形。仓库同时给出了 A\*、SIPP1、SIPP2、SIPP-IP 四个算法用于对比实验。

- **论文**：*Safe Interval Path Planning with Kinodynamic Constraints*，**Zain Alabedeen Ali, Konstantin Yakovlev**，AAAI-23（Proceedings of the AAAI Conference on Artificial Intelligence, Vol. 37 No. 10, pp. 12330–12337, DOI `10.1609/aaai.v37i10.26453`；预印本 arXiv:2302.00776）。
- **论文身份的证据链**：README 只写了算法名与"the paper"，**并未给出任何引用/链接**（`README.md:1-2`、`README.md:20`、`README.md:23`）。因此论文题名、作者、会议信息来自：(a) `git log` 的作者为 `Zain` 与 `Konstantin Yakovlev`（提交 `6d64d09`、`d3e085e`、`77ca749`，日期 2022-12-01）；(b) 仓库 `origin` 为 `https://github.com/PathPlanning/SIPP-IP.git`；(c) 网络检索到的 AAAI-23 论文页与 arXiv 摘要页。**"README 未给出引用"是确认的；"论文即上述 AAAI-23 论文"属于高置信推断，非仓库内直接证据。**
- **作者（代码）**：`Zain`（初始提交）、`Konstantin Yakovlev`（README）；`src/` 目录下另有一个 `.vscode/settings.json`（内容只有 `ros.distro: humble` 与 `files.associations.chrono: cpp`，`src/.vscode/settings.json:1-6`），提示作者也使用 ROS 2 环境，但代码本身**不依赖 ROS**。
- **许可证**：MIT License，`Copyright (c) 2022 PathPlanning`（`LICENSE:1-3`）。

---

## 2. 问题定义

### 2.1 图与配置

- 图 `G=(V,E)`，顶点 `v = (x, y, o, v)`：位置（格心）+ 朝向 `o∈{0,1,2,3}` + 离散速度档 `v∈{0,1}`（`include/constants.h:8-9`，`src/SIPP-IP.cpp:18`）。**速度是配置的一部分**，因此同一格不同速度是不同顶点。
- 边 `e=(v,u)` 是**运动基元（motion primitive）**：一段运动学可行的运动片段，整数时长 `w(e)`（时间步数），刻画该片段扫过的每个格子及其被"触碰"的时间窗（`Primitive::move {dx,dy,ftt,swt,isEndCell}`，`src/SIPP-IP.cpp:54-79`）。
  - `ftt`（first-touch-time）= 基元从起点算起**首次触碰**该格的时刻（即论文中的 `t_lower`，见 `src/SIPP-IP.cpp:57`）。
  - `swt`（sweeping time）= 触碰持续时长（论文中的 `t_upper - t_lower`，`src/SIPP-IP.cpp:58`）；对终点格 `isEndCell=true`，`swt` 表示"到基元结束还剩多久"（`src/SIPP-IP.cpp:61`）。
- 时间离散为整数时间步 `T={0,1,…,INF}`，`F=10` 步/秒，故 1 步 = 0.1 s，`INF=40000` 步 = 4000 s（`include/constants.h:7,10`）。

### 2.2 Blocked Interval（阻断区间）与 Safe Interval（安全区间）

对任一配置 `v`，动态障碍在其上留下若干**阻断区间** `B(v) = { [a_i,b_i] }`（闭区间、互不重叠、按 `a_i` 排序）。代码直接存储阻断区间于 `rsrv_tbl[x][y]`，并加上哨兵 `(-1,-1)` 与 `(INF+1,INF+1)` 便于 `lower_bound` 查询（`src/SIPP-IP.cpp:209-222`）。

配置 `v` 的**安全区间集合** `SI(v)` 是 `[0,INF]` 对 `B(v)` 求补得到的**极大不相交区间**序列：

```
SI(v) = { I ⊆ [0,INF] | I 极大, 且 ∀t∈I: t ∉ B(v) }
```

- SIPP 的搜索节点是 `n = (v, [lb, ub])`，其中 `[lb,ub]` 是 `v` 的某个安全区间（`README.md:1-2`；论文 SIPP 定义）。
- **SIPP 的核心**:把"每步等待 1 个时间步"的原子动作**压缩**成区间，只在安全区间之间做转移；因此节点数远小于"A\* + 时间维"。代价 `g(n) = t(n)` 是**最早到达时间**，`cost(π)` 即到达目标顶点的时刻（论文 Problem Statement）。
- `Safe Interval` 的语义保证：只要 `t ∈ SI(v)`，配置 `v` 在 `t` 时刻对**所有**动态障碍都是无碰撞的。

### 2.3 Interval Projection（区间投影）

原版 SIPP 的隐含假设是"机器人可瞬时启停"，于是"在任意配置都能等待"，任何到达时刻都能被推迟到安全区间右端。当加入加/减速后，**速度非零的配置不能等待**，SIPP 会丢失"父节点中还可以晚一点出发"的信息，从而**不完备**（论文 Statement 1 与 Fig. 1/2 的 3×10 例子）。

SIPP-IP 的做法：节点的区间不再取"安全区间"，而取一个 **waiting interval（等待区间）**`[t_l,t_u] ⊆ SI(v)`——它编码了"从根节点一路投影下来，机器人**实际可以在这个配置上开始下一个动作**的全部时刻"。投影操作定义（论文 Method / Projecting intervals）：

> 输入：节点 `n=(v,[t_l,t_u])` 与边 `e=(v,v')`；输出：区间集合 `TI={[t',t'']}`，满足
> 1. `∀ti∈TI, ∃si∈SI(v'): ti ⊆ si`（落在目标顶点的某个安全区间内）；
> 2. `TI` 内部互不重叠；
> 3. `∀t∈[t_l,t_u]`，若 `(e,t)` 无碰撞，则 `t+w(e) ∈ TI`（**不丢任何可行出发时刻**）。

代码里投影由 `applyPrimitive()` 实现（`src/SIPP-IP.cpp:103-146`），其数学条件见 §5.4。

---

## 3. 技术栈与工具链

- **语言/标准**：C++。使用了 `auto`、range-for、`std::move`、`emplace_back`、`to_string`、`default` 成员函数、`<chrono>`，即 **C++11 及以上**；`generate_obstacles.cpp` 还用了 `#include <bits/stdc++.h>` 与 `mt19937`/`random_device`（`src/generate_obstacles.cpp:1,247-252`），偏向 GCC/Clang 工具链。仓库**没有** `Makefile`/`CMakeLists.txt`/构建脚本（已确认目录列举）。
- **"self-contained 无依赖"的含义**：每个算法是一个**单文件可执行程序**，只 `#include "../include/constants.h"`（`src/SIPP-IP.cpp:10`），标准库之外无第三方库；算法源代码里**没有** `#include <queue>` 的 Dijkstra/堆，OPEN 直接用 `std::set<Bot>`（`src/SIPP-IP.cpp:85`）当优先队列。
- **编译**（README `README.md:12` 说 "Just compile the needed file and run it"）。典型命令：
  ```bash
  cd src
  g++ -O2 -std=c++17 SIPP-IP.cpp -o sipp-ip && ./sipp-ip
  ```
  **注意运行目录**：所有 I/O 都用相对路径 `../maps/...`、`../tests/...`、`../results/...`（如 `src/SIPP-IP.cpp:227,238,339`），所以必须在 `src/`（或任何"项目根下一级"的目录）里运行，否则读不到文件。`-O2` 强烈建议（默认 `-O0` 下 SIPP2/SIPP-IP 的搜索会慢一到两个数量级；`results/` 里的耗时量级也说明实验是带优化的）。
- **输出目录前置条件**：`A-star.cpp` 写到 `../results/results Astar/...`（`src/A-star.cpp:321`），但仓库里**没有这个目录**（已确认），因此 A\* 的输出流会静默失败、结果未落盘。SIPP1/2/IP 的三个目录存在。
- **无版本控制产物**：`.gitignore` 未列构建产物；仓库内无二进制。

---

## 4. 目录与模块地图

```
SIPP-IP/
├── README.md              项目说明（38 行；无论文引用）
├── LICENSE                MIT (c) 2022 PathPlanning
├── include/constants.h    全部编译期常量 + 地图选择（唯一共享头）
├── src/
│   ├── A-star.cpp         时间展开 A*（基线）
│   ├── SIPP1.cpp          SIPP 变体 1（区间表示）
│   ├── SIPP2.cpp          SIPP 变体 2（CLOSED 处理修正）
│   ├── SIPP-IP.cpp        本文方法（等待区间投影）
│   ├── generate_obstacles.cpp  测试实例生成器
│   └── .vscode/settings.json   IDE 辅助，无构建配置
├── maps/                  5 张静态地图（格点 0/1 文本）
├── tests/                 6 个目录（按地图名分组）+ empty_1_16 调试目录
└── results/               3 个结果目录（SIPP1/SIPP2/SIPP-IP），无 A* 目录
```

- **`src/A-star.cpp`**（338 行，`main` 在 `:313`）：状态含显式时间 `t`（`Bot{x,y,o,v,t}`，`:18`），OPEN 为 `std::set<Bot>`，节点数即"配置×时间步"；用于体现 SIPP 相对 A\* 的加速（`README.md:23`）。输出路径的目录缺失（见 §3）。
- **`src/SIPP1.cpp`**（372 行，`main` 在 `:347`）：节点为 `Bot{x,y,o,v,t_lower,t_upper}`（`:18`）；`checkCLOSED` 会对 CLOSED 做一次"区间裁剪"但**不重新入队**（`:214-219`）；`v≠0` 的后继被压成**单点区间**（`:161-172`）。
- **`src/SIPP2.cpp`**（387 行，`main` 在 `:362`）：与 SIPP1 逐字节比对仅 3 处语义差异（见 §5.3），核心是 CLOSED 裁剪改成 `last_closed.upper+1` 并把被裁剪节点**重新插入 OPEN**（`:212-220`）。
- **`src/SIPP-IP.cpp`**（357 行，`main` 在 `:331`）：在 SIPP2 基础上，`v≠0` 的后继**保留整个投影区间**（`:142-144`），且终点区间的拓宽仅在 `mp.v==0` 时发生（`:123-126`）；`generateSuccessors` 里"`t_lower<t_upper` 则 `assert(v==0)`"被删除（对比 `src/SIPP2.cpp:176-178`）。
- **`src/generate_obstacles.cpp`**（391 行，`main` 在 `:308`）：为 5 张地图生成"动态障碍轨迹 → 每格阻断区间"文本实例；内部用 `bool rsrv[256][256][4001]`（`src/generate_obstacles.cpp:4-12`）。
- **`include/constants.h`**（42 行）：`INF/MXO/MXV/F`、地图三件套 `MXH/MXW/map`（当前启用 `random128`，`:35-37`）、`NumOfTests`、`factors`、`MAX_NUM_NODES`。
- **`maps/`**：`Sydney_2_256.txt`(256×256)、`random128.txt`(128×128)、`room-64-64-16.txt`(64×64)、`empty_64_64.txt`(64×64)、`warehouse-10-20-10-2-2.txt`(84×170)。格式：首行 `H W`，随后 `H` 行、每行 `W` 个 `0/1`（`1`=静态障碍），见 `maps/random128.txt:1-3`。
- **`tests/`**：`Sydney_2_256/`、`random128/`、`room-64-64-16/`、`empty_64_64/`、`warehouse-10-20-10-2-2/` 各 7 个文件（对应 7 个 `factor`，`testNum=0` 各一份），共 316 MB；另有 `empty_1_16/` 一个 202 字节调试文件，**但 `maps/` 里没有 `empty_1_16.txt`**，所以当前代码无法为它运行（已确认）。
- **`results/`**：`results SIPP1/`、`results SIPP2/`、`results SIPP-IP/` 各 35 个文件（5 地图 × 7 密度），每文件 1 行（生成时 `NumOfTests=1`）。**没有 `results Astar/`**。
- **运行入口**：四个算法各自的 `main()` 都会 `fillActions()` 构建运动基元，遍历 `factors`，对每个 `(map, factor)` 打开结果文件，循环 `testNum∈[0,NumOfTests)`，每次 `clr()` → `readInputs()` → `search()` → 写一行（如 `src/SIPP-IP.cpp:331-355`）。

---

## 5. 核心算法逐个拆解

四个文件是**同一份代码的演化链**：`A-star.cpp` → `SIPP1.cpp` → `SIPP2.cpp` → `SIPP-IP.cpp`（`diff` 显示大量行仅风格不同）。以下逐一把函数级行为写清。

### 5.1 A\*（时间展开基线，`src/A-star.cpp`）

**输入/输出**：输入起点 `(st_x,st_y,st_o)`、初始速度 0、初始时间 0（`:328`）；输出 `pair<int,bool>`（CLOSED_vec 下标 / 是否成功），成功时 `CLOSED_vec[id].t` 即最优到达时间。

**节点与 g/h**：
- 节点 `Bot{x,y,o,v,t,parentID}`（`:18`），`g(n)=t`。
- 启发式 `h(n) = minimalTransitionCost · (|x-end_x| + |y-end_y|)`，其中 `minimalTransitionCost=5`（`:248`），即"每跨一格至少 5 个时间步"。
- 排序键（`operator<`，`:46-58`）依次为：`f = t + h` 小者优先 → `h` 小者优先 → **`t` 大者优先**（`a.t > b.t`，`:53-55`）→ 按 `((x*MXW+y)*4+o)*MXV+v` 编码的状态序（`:56-57`）。`OPEN` 是 `std::set<Bot>`，因此每次取最小/插入都是 `O(log|OPEN|)`，不是二叉堆。

**可采纳性与一致性（证明思路）**：
- 任何一条从 `u` 到目标的路径都要移动至少"曼哈顿距离"格；每一步/每个基元的耗时 ≥ 5 步（巡航 1 格 = 5 步；加速基元 4 格 = 40 步即 10 步/格；原地等待 = 1 步），故 `h ≤ h*`，**可采纳**。
- 对任一边 `u→v`，`|Manhattan(u)-Manhattan(v)| ≤ 1`（基元沿单一朝向直线移动，`fillActions` 的 `dx[o]*j, dy[o]*j`，`:262,273,279`），而边代价 `c ≥ 5`，故 `|h(u)-h(v)| ≤ 5 ≤ c`，**一致**。一致性保证 A\* 首次弹出即最优，无需重开。

**关键步骤（伪代码）**：
```
search(st, 0):
  校验起点所在格在 t=0 是否自由，否则 cerr + exit(0)      # :151-157
  OPEN={Bot(st, v=0, t=0)}; CLOSED_vec=[]
  while OPEN 非空 and cntNodes < MAX_NUM_NODES:
    n = min(OPEN); OPEN.erase(n); --cntNodes
    if CLOSED[x][y][o][v] 含 t: continue                  # :168-170
    CLOSED[x][y][o][v].insert(t); id=|CLOSED_vec|; CLOSED_vec+=n
    if (x,y)==goal and v==end_v(=0): return (id,true)      # :176-178
    for mp in motion_primitives[o][v]: succs += applyPrimitive(...)
    for s in succs: s.parentID=id; OPEN.insert(s)
  return (0,false)
```

**碰撞判定 `applyPrimitive`（`:114-132`）**：沿基元的每个 `move`，把时间平移到该格（`t += mv.ftt - past_time`，`:122`），然后用 `rsrv_tbl` 做两次比较：
```
itr      = rsrv_tbl[xx][yy].lower_bound({t, INF-1})   # 第一个 start ≥ t 的阻断区间
itr_prev = itr; --itr_prev                            # 最后一个 start < t 的阻断区间
若 itr_prev->second >= t            → 碰撞（t 落在阻断区间内）
若 itr->first - mv.swt <= t         → 碰撞（扫掠窗 [t, t+swt] 撞上下一个阻断区间的起点）
```
通过则生成一个后继 `Bot(xx,yy,mp.o,mp.v, min(INF, t+swt_last))`（`:131`）。**注意**：A\* 里"等待"由一个显式的原地基元提供——`fillActions` 把原地动作的 `swt` 从 20 改成 1（`:258-260`），即"在 v=0 处等待 1 个时间步"。

**关键参数**：`minimalTransitionCost=5`（`:248`）、`MAX_NUM_NODES=1e8`（`include/constants.h:42`）。

**复杂度**：状态空间 ≤ `|X|·MXO·MXV·(INF+1)`；每次展开至多 4 个基元（v=0：左转/右转/加速/等待；v=1：减速/巡航，见 §5.5），每次后继生成 `O(|mvs|·log B)`，`B`=单格阻断区间数。总时间 `O(N·A·K·log(B·|OPEN|))`。

### 5.2 SIPP1（`src/SIPP1.cpp`）

**与 A\* 的本质差异**：
1. 节点改为 `Bot{x,y,o,v,t_lower,t_upper,parentID}`（`:18`）——**节点标识含区间**，`g(n)=t_lower`（结果里 `Cost: CLOSED_vec[ans.first].t_lower`，`:364`）。
2. 没有"等待"基元（对比 `A-star.cpp:258-260`）：等待被隐式编码进区间。
3. CLOSED 表按 `(x,y,o,v)` 存**区间集合** `set<pair<int,int>>`（`:107`），而非时间点集合。
4. 起点节点区间 = 起点所在格、包含 `t=0` 的**整个安全区间**：`st_tupper = rsrv.lower_bound({0,INF})->first - 1`（`:197-198`）。

**安全区间的"构造"**：SIPP 并不显式枚举安全区间，而是把 `rsrv_tbl`（阻断区间）当作查询结构；安全区间体现为查询时的补集。起点安全区间在 `search()` 中推算（`:197-206`）；后继安全区间在 `applyPrimitive()` 中逐格求补（`:139-153`）。

**区间上的到达时间传播（核心）**：
`applyPrimitive(x,y,t_lower,t_upper,mp)`（`:125-175`）把"出发时刻集合" `[t_lower,t_upper]` 沿基元的每个 `move` 传播：
```
timeIntervals = [[t_lower,t_upper]]            # 当前帧：上一个 move 格子的首触时刻
past_time = 0
for mv in mp.mvs:                              # 按基元定义顺序
   xx = x+mv.dx ; yy = y+mv.dy                 # 注意：偏移是相对基元起点，不是增量
   if 出界: return {}
   for it in timeIntervals:
       t_l = it.first  + (mv.ftt - past_time)   # 帧切换 → 该格首触时刻
       t_u = it.second + (mv.ftt - past_time)
       itr = rsrv_tbl[xx][yy].lower_bound({t_l, INF-1}); itr_prev = itr-1
       while itr_prev->second < t_u:            # 该阻断区间与 [t_l, t_u] 相交
           new_tlower = max(t_l, itr_prev->second + 1)
           new_tupper = min(t_u, itr->first - 1 - mv.swt)   # 保证整段扫掠都自由
           if isEndCell and 首次: new_tupper = itr->first - 1 - mv.swt  # 终点拓宽
           if new_tlower <= new_tupper: tmp += [new_tlower,new_tupper]
           ++itr; ++itr_prev
   timeIntervals = tmp; tmp.clear(); past_time = mv.ftt
```
**数学含义**：设基元在 `τ0` 启动，则第 `i` 格首触时刻 `τ_i = τ0 + f_i`，占用窗 `[τ_i, τ_i+s_i]`。约束是 `∀i: [τ_i, τ_i+s_i] ∩ B(c_i) = ∅`。由于所有 `τ_i` 是 `τ0` 的平移，合法的 `τ0` 集合在每一步只是**与补集求交**，仍然是一组区间——这正是"投影后仍是区间"的原因。最终：
```
if mp.v == 0: 后继区间 = [it.first+s_last, it.second+s_last]   # 终点可等待 → 保留区间 (:163-167)
else:         后继区间 = [it.first+s_last, it.first+s_last]    # 移动态 → 压成单点 (:168-172)
```
`+s_last`（终点格的 `swt`）把"终点格首触时刻"换算成"基元结束时刻 = 到达时刻"。

**区间冲突合并规则（`checkCLOSED`，`:115-123`）**：
```
若 CLOSED[x][y][o][v] 为空 → 原样返回 [t_lower,t_upper]
it = CLOSED[x][y][o][v] 的最大元素        # 注意：只看 last
tmp = rsrv_tbl[x][y].lower_bound({it->first, INF})->first   # 该区间之后那个阻断区间的起点
返回 [max(t_lower,tmp), t_upper]
```
即"若新到达的区间与已闭合区间重叠，把下界抬高"。**SIPP1 只裁剪、不重新入队**：`search()` 里拿到裁剪结果后直接改 `t_lower/t_upper` 继续展开（`:214-219`），只有 `first>second` 才丢弃。

**节点定义/输入输出**：见上；`getSolutionStates` 沿 `parentID` 回溯并 `reverse`（`:187-194`）；`printSolutionStates` 打印每个节点的 `x,y,o,v,t_lower,t_upper,parentID`（`:338-346`，`README.md:38`）。

**关键参数**：`minimalTransitionCost=5`（`:285`）、`F/INF`（时间步）、`MAX_NUM_NODES`。

**复杂度**：节点数从"配置×时间步"降到"配置×安全区间访问段"，通常少若干数量级；单次 `applyPrimitive` = `O(K·I·log B)`（`K≤5` 个 move、`I` 当前区间数、`B` 单格阻断区间数），每次展开 3（v=0）或 2（v=1）个基元。

### 5.3 SIPP2（`src/SIPP2.cpp`）——与 SIPP1 的真实差异

对 `SIPP1.cpp` 与 `SIPP2.cpp` 做全文 `diff`，**只有 3 处语义差异**，全部围绕 CLOSED 表，**与障碍数量、区间求交或"更紧的区间"无关**：

| 差异 | SIPP1 | SIPP2 | 位置 |
|---|---|---|---|
| CLOSED 裁剪的返回值 | 返回 `pair`：下界取"已闭合区间之后那个阻断区间的起点"`rsrv.lower_bound({it->first,INF})->first`，上界保留 | 只返回新下界 `max(t_lower, last_closed.second+1)`，**上界不动** | `SIPP1.cpp:115-123` vs `SIPP2.cpp:115-121` |
| 被裁剪节点的处理 | 就地用裁剪后的区间继续展开，**不重新入队** | 若 `t_lower` 变了则**改下界后重新插入 OPEN**（`tmp.t_lower=new_t_lower; OPEN.insert(tmp); continue;`） | `SIPP1.cpp:214-219` vs `SIPP2.cpp:212-220` |
| 读入校验 | 仅 `assert` 阻断区间不相交 | 额外 `cerr` 报错"Blocked intervals ... are intersecting!" | `SIPP1.cpp:280-281` vs `SIPP2.cpp:287-291` |

**结论（从代码确认）**：SIPP2 = SIPP1 + **CLOSED 表处理更严谨**（把"与已访问区间重叠的部分"切掉并重新调度，而不是就地接受一个可能不干净的下界）。后继区间宽度、终点拓宽条件、`v==0`/`v==1` 的分支处理，两者**完全相同**。因此实验里 SIPP2 与 SIPP1 成功率/代价的差异，只来自 CLOSED 语义，而非"多障碍/区间交集"。

### 5.4 SIPP-IP（`src/SIPP-IP.cpp`）——Interval Projection

**与 SIPP2 的差（`diff` 确认，共 3 处语义差异）**：
1. **终点区间拓宽加 `mp.v==0` 条件**（`:123-126`）：
   ```cpp
   if (new_tlower <= new_tupper && (mv.isEndCell && !endCellTouched) && mp.v == 0)
       new_tupper = itr->first - 1 - mv.swt;
   ```
   SIPP2 对**所有**基元都在首个终点格做这次"上界放宽"（`SIPP2.cpp:143`）。SIPP-IP 只对**终点速度为 0**的基元放宽：因为只有 `v=0` 的配置可以等待，到达后能一直等到该格安全区间右端；`v≠0` 的到达集合就是投影结果本身，无权外扩。
2. **后继区间不再压成单点**（`:142-144`）：
   ```cpp
   for (auto it: timeIntervals)
       vBot.push_back(Bot(xx,yy,mp.o,mp.v,
                          min(INF,it.first +mp.mvs.back().swt),
                          min(INF,it.second+mp.mvs.back().swt)));   // 恒为区间
   ```
   SIPP2 对 `mp.v!=0` 生成 `[first+s, first+s]`（`SIPP2.cpp:166-172`）。**这一条就是论文所说的 Interval Projection 的落地**：把父节点的等待区间 `[t_l,t_u]` 整体投影到后继，后继的等待区间是 `[t_l+w(e), t_u+w(e)]`（再被沿途各格的安全区间裁剪），而不是只保留"最早到达"这一条时间线。论文的 3×10 反例正是被这一点救活的：在 B 点（`vel=1`）仍能记住"可以在 `[2,7]` 内任何时刻出发"，从而让 uniform 动作在 `t=4` 启动、`t=5` 落进 C 的安全区间。
3. **删除了 `assert(v==0)`**（对比 `SIPP2.cpp:176-178` 与 `SIPP-IP.cpp:147-152`）：SIPP2 断言"非退化区间的节点速度必为 0"；SIPP-IP 允许 `v=1` 的节点携带非退化区间，故该断言不再成立。

**投影到安全区间上的数学条件（严格版）**：
- 源节点 `n=(c,o,v,[t_l,t_u])`，`[t_l,t_u] ⊆ SI(c)`（由构造保证）。
- 边（基元）`e` 扫过格子序列 `c_0,…,c_{K-1}`，各格首触偏移 `f_i`、扫掠时长 `s_i`（`c_{K-1}` 为终点格，`s_{K-1}`= 剩余时长）。启动时刻 `τ∈[t_l,t_u]`。
- **无碰撞条件**：`∀i, ∀t∈[τ+f_i, τ+f_i+s_i]: t ∉ B(c_i)`。
- **投影结果**：`TI = { τ + w(e) | τ∈[t_l,t_u], 上述条件成立 }`，代码里进一步把 `TI` 按各格安全区间切成若干互不相交的闭区间（`while (itr_prev->second < t_u)` 循环内 `tmp.push_back`，`:120-132`）。若 `mp.v==0`，终点对 `TI` 的上界放宽到该格安全区间右端（`:123-126`）。

**可达性/完备性论证思路**：对父节点区间里的**每一个** `τ`，`applyPrimitive` 只在"该 τ 的整条扫掠轨迹逐格无碰撞"时才把它保留进 `TI`；因此 `TI` 中任意 `t̂` 都存在一条具体的、无碰撞的、时间上可达的前缀轨迹到达后继节点（对基元个数归纳）。反过来，任何无碰撞的 `τ` 都不会被丢弃（条件 3 的完备性）。于是搜索树里"SIPP 因只保留最早到达而丢失的可等待信息"被区间整体保留下来；这就是 SIPP-IP 在加/减速下仍**完备且最优**（代价=到达时间）的直觉证明。完整形式化证明见论文（外部资料）。

**输入/输出**：`search(st_x,st_y,st_o,st_v=0,st_tlower=0)`（`:163`）→ `pair<int,bool>`；结果代价为 `CLOSED_vec[ans.first].t_lower`（`:348`）。`getSolutionStates`/`printSolutionStates` 同 SIPP1。

**关键参数**：`minimalTransitionCost=5`（`:269`）；时间步/上界 `F,INF`；节点预算 `MAX_NUM_NODES`；运动基元参数硬编码在 `fillActions()`（见 §5.5）。

**复杂度**：同 SIPP2 的节点模型，但节点数会因"移动态也带区间"而增加；单次投影 `O(K·I·log B)`。实验上 SIPP-IP 仍比 SIPP2 更快且成功率更高（§9）。

### 5.5 运动基元集合（四个算法共用，`fillActions()`）

以 `src/SIPP-IP.cpp:268-305` 为例（`A-star.cpp:247-287` 多一个"原地等待 1 步"）：

- **朝向编码**：`dx[4]={0,-1,0,1}, dy[4]={1,0,-1,0}`（`:280`），`o=0` 为东（`include/constants.h:8`），逆时针递增。
- **`v=0` 可用（`motion_primitives[o][0]`）**：
  - 左转 90°：`{move(0,0, 0, 20, isEnd=1)}`，即原地停 20 步（2 s），`o←(o+1)%4`，`v=0`（`:273-277`）。
  - 右转 90°：同上，`o←(o+3)%4`（`:277-278`）。
  - **加速基元**：`v=1, o` 不变，5 个 move，`costs={(0,20),(0,29),(20,15),(28,12),(34,6)}`（`:283`），即格子偏移 `j·dx[o]`、首触 `costs[j].first`、扫掠 `costs[j].second`，最后一格 `isEndCell`。总时长 `40` 步 = 4 s、跨越 4 格，注释写明 `a=±0.5 m/s², v_max=2 m/s`（`:282`）。
  - （仅 A\*）原地等待 1 步（`A-star.cpp:258-260`）。
- **`v=1` 可用（`motion_primitives[o][1]`）**：
  - **减速基元**：把加速基元的时间轴反向（`ftt = full_cost-(costs[sz-1-j].first+costs[sz-1-j].second)`，`:297`），起点 `v=1`、终点 `v=0`。
  - **巡航一步**：`{move(0,0,0,5,isEnd=0), move(dx[o],dy[o],0,5,isEnd=1)}`，1 格 / 5 步 = 0.5 s，`v=1` 不变（`:300-303`）。
- 因此每格每朝向的动作数：`v=0` → 3（A\* 为 4，多一个"原地等待 1 步"），`v=1` → 2。总基元数：SIPP 系列 `4×3 + 4×2 = 20`，A\* `4×4 + 4×2 = 24`。

---

## 6. 参数总表（`include/constants.h`）

| 常量 | 值 | 含义 | 对实验的影响 | 位置 |
|---|---|---|---|---|
| `INF` | `40000` | 时间轴上界（**时间步**），= `INF/F` = 4000 s | 所有到达时间用 `min(INF,·)` 截断；越大搜索空间越大、`generate_obstacles` 内存越大（注释明说影响存储） | `:7` |
| `MXO` | `4` | 朝向数，0=东，逆时针 | 决定基元维数、状态空间 ×4 | `:8` |
| `MXV` | `2` | 速度档数（0=可停/慢，1=巡航） | 状态空间 ×2；`v` 是配置一部分 | `:9` |
| `F` | `10` | 每秒时间步数（1/T） | 时间离散精度 0.1 s；文件里的"秒"乘 `F` 变成内部时间步（`readInputs`） | `:10` |
| `MXH` | 当前 `128` | 环境最大高度（第一维，行） | 决定静态数组尺寸 `rsrv_tbl[MXH][MXW]`、`CLOSED[MXH][MXW][MXO][MXV]`；**必须 ≥ 地图文件首行的 H**，否则越界 | `:35` |
| `MXW` | 当前 `128` | 环境最大宽度（第二维，列） | 同上；还参与状态编码 `x*MXW+y` 与排序 tie-break | `:36` |
| `map`（`string`） | 当前 `"random128"` | 选中的地图名 | 决定读 `../maps/<map>.txt`、`../tests/<map>/<map>-test-<n>-<obs>.txt`、写 `../results/results <ALGO>/res-<map>-<ALGO>-obs<obs>.txt` | `:37` |
| `NumOfTests` | `1` | 每张地图每个 `factor` 生成的实例数与要跑的实例数 | 生成器为每个 `testNum` 生成一套；当前只有 `testNum=0` 的实例和 1 行结果 | `:40` |
| `factors` | `{25,20,15,10,5,4,3}` | 障碍密度：**动态障碍数 = 地图自由格数 / factor** | factor 越小障碍越多、越难；7 个值产生 7 组实例与 7 个结果文件 | `:41` |
| `MAX_NUM_NODES` | `1e8` | OPEN+CLOSED 节点预算（循环条件 `cntNodes<MAX_NUM_NODES`） | 触发时搜索被截断并返回失败；`cntNodes` 的记账是近似值（见 §10） | `:42` |

**`generate_obstacles.cpp` 内部的常量**（不在 `constants.h`）：

| 常量 | 值 | 含义 | 位置 |
|---|---|---|---|
| `MX_HEIGHT` / `MX_WIDTH` | `256` | 生成器支持的最大地图尺寸（静态数组上界） | `:4-5` |
| `MXT` | `INF/F+1 = 4001` | 阻断时间轴长度；**说明生成器的时间轴单位是"秒"**（0..4000 s，共 4001 个采样），而搜索内部用 0.1 s 步长，`readInputs` 用 `×F` 换算 | `:6` |
| `i_spd` | `randSpeed(1,10)` | 障碍跨越一格的耗时（秒/格），越大越慢 | `:251,269,291` |
| `randTime(0,200)` | — | 障碍出发时刻（秒），仅用于非 `empty_64_64` 分支 | `:252,278` |

---

## 7. 测试实例生成

### 7.1 `generate_obstacles` 的算法

`main()`（`generate_obstacles.cpp:308-380`）对 `maps = {"room-64-64-16","empty_64_64","random128","warehouse-10-20-10-2-2","Sydney_2_256"}` 逐张处理：

1. 读地图，统计自由格数 `sm`（`:313-323`）。
2. `min_factor = min(factors) = 3`（`:324-327`）；`generate_obstacles1(sm/3)` 先按**最大障碍数**生成一批障碍，存入全局 `obs`（`:329-344`）。注意函数名有误导性：它的参数其实是"要生成多少个障碍"。
3. 对每个 `factor`：`num_of_obstacles = sm/factor`（整数除法），`clr()` 清空时间轴，把前 `num_of_obstacles` 个障碍**重放**一次（`generate_obstacles2(cnt)`，`:360-362`），再把时间轴压成文本区间输出（`:363-375`）。
   - **因此同一地图的 7 个实例是"嵌套前缀"关系**：`obs163` ⊂ `obs204` ⊂ … ⊂ `obs1365`（前 k 个障碍完全相同）。这使跨密度对比更干净。
4. **起点/终点固定**（`generate_start_end_points`，`:35-90`）：`random128` 为 `(5,6)→(125,124)`；`empty_64_64` 为 `(5,5)→(59,59)`；`room-64-64-16` 为 `(6,6)→(58,58)`；`warehouse` 为 `(4,4)→(78,163)`；`Sydney_2_256` 为 `(31,23)→(239,239)`；朝向均为 0。生成器把起点/终点格临时标 `mp=true`，避免障碍轨迹起点/终点落在它们上面（`:41,45,...`）。终点线只写 `goal_cell`，不含目标速度（算法内部 `end_v` 固定为 0）。

**动态障碍的布置算法**（两种分支）：

- **时间无关 BFS（默认分支，`find_and_mark_path`，`:181-242`）**：随机起点 `(stx,sty)` + 随机终点 `(ndx,ndy)` + 随机出发时刻 `st_t∈[0,200]`；在 9 邻域（8 连通 + 原地）上 BFS，每步耗时 `i_spd`，用 `mem1[x][y]` 做**只按空间格**去重（不允许重复访问同一格），找一条到终点的空间简单路径；到达后把沿路各格在 `[k+st_t, …]` 上标为阻断，并把**终点格从到达时刻一直阻断到 MXT**（`:192-207`）。若在 `MXT` 内到不了终点则本次尝试作废，重新随机。
  - **关键局限**：此分支**从不查询 `rsrv`**，即障碍之间不会避让，轨迹可以在时空上互相穿越（`:220-236`）。因此生成的实例里障碍并非"物理上相互一致的 AGV"，只是任意叠加的阻断区间。
- **时间相关 BFS（仅 `empty_64_64` 与 `random-64-64-10`，`find_and_mark_timal_path`，`:108-178`）**：状态是 `(x,y,cost)`（`mem[x][y][cost]`），扩展时要求 `[cost, cost+i_spd]` 内当前格与目标格都未被 `rsrv` 占用（`:156-161`），因此障碍之间**互不碰撞**；到达后同样把终点格阻断到 `MXT`（`:125-127`）。若始终找不到则重新随机起终点（`:256-274`）。
- 两种分支都会让障碍在抵达后**永久停在终点格**（阻断到 `MXT`），这在 `tests/` 里表现为大量 `to_timestep: 4000` 的行（例：`tests/random128/random128-test-0-2938.txt:3`）。

**关于"如何保证有解"**：**代码并不保证实例有解。** 生成器只保证"障碍轨迹本身（在时间相关分支下）彼此无碰撞"，从未检查玩家在给定时间内能否从起点到达终点。`results/` 中大量 `Success:0`（§9）正是证据。可操作性上，"有解"完全由 `factor`（密度）决定，且随地图结构剧烈变化。

**生成器实现上的坑（已在 §10 汇总）**：`main` 第 332 行 `in.open("../" + maps[k] + ".txt")` 路径错误（少了 `maps/`），该次重读实际是空操作（`:332-339`）；`st[MX_HEIGHT][MX_WIDTH]`、`st1`、`bool mp[256][256]`、`bool rsrv[256][256][4001]`（约 262 MB）都是静态全局数组。

### 7.2 地图格式（`maps/`）

```
<H> <W>
<H 行 × W 个 0/1，空格分隔>
```
- `1` = 静态障碍（该格在 `[0,INF]` 全程阻断，`src/SIPP-IP.cpp:232-234`），`0` = 空地。行索引为 `x`（第一个坐标，0..H-1），列索引为 `y`（0..W-1）。
- 无注释、无额外字段；算法读入时用 `H`、`W` **覆盖** `MXH/MXW` 全局变量，但静态数组仍按 `constants.h` 的 `MXH/MXW` 分配 → **地图尺寸不得超过 `MXH/MXW`**。

### 7.3 实例格式（`tests/<map>/<map>-test-<n>-<obs>.txt`）

纯文本、字段名 + 数值（用 `>> string >> … >> int` 解析，`src/SIPP-IP.cpp:241-247`）：

```
start_cell: x= <x> y= <y> orientation= <o>
goal_cell: x= <x> y= <y>
reserved_interval_at_cell: x= <x> y= <y> from_timestep: <t1> to_timestep: <t2>
...（每格每段一行，按 x、y、t 升序，同一格多段不重叠）...
```
- **单位澄清**：字段名叫 `timestep`，但生成器的时间轴长度是 `MXT=INF/F+1=4001`，即**实际单位是秒**（0..4000）。算法读入后 `t1*=F; t2*=F` 换算成 0.1 s 的**内部时间步**（`src/SIPP-IP.cpp:245-246`）。两种理解在数值上自洽，但命名极易误读（详见 §10）。
- `from_timestep: … to_timestep: 4000` 表示该格从某时刻起被永久阻断（障碍停在终点格）。
- 目录名与文件名中的 `<obs>` 就是 `自由格数/factor`（整数除法），可用它反查 `factor`。
- `tests/empty_1_16/empty_1_16-test-0-0.txt` 只有 4 行（起点 `(0,0)`、终点 `(0,15)`、2 段阻断），是调试残留；`maps/` 里没有对应地图。

### 7.4 结果文件格式（`results/results <ALGO>/res-<map>-<ALGO>-obs<obs>.txt`）

每行一条记录，字段固定三个（`src/SIPP-IP.cpp:348`，`README.md:31-34`）：

```
Success:<0|1>, Cost:<到达时间步或0>, Runtime:<微秒>
```
- `Success:1` ⇒ `Cost` = `CLOSED_vec[id].t_lower`（SIPP 系列）或 `CLOSED_vec[id].t`（A\*）；`Success:0` ⇒ `Cost:0`。
- `Runtime` 是 `high_resolution_clock` 测得的**微秒**（`duration_cast<microseconds>`，`:347`）。SIPP2/SIPP-IP 的文件里出现 `1.26967e+07` 这类科学计数法，**并非单位不同**，只是 `double` 用默认精度 6 输出时 ≥1e6 会自动转科学计数法；SIPP1 多数 <1e6 微秒，所以显示为整数。
- 文件行数 = `NumOfTests`（当前生成时为 1）。

---

## 8. 关键数据结构

| 结构 | 定义 | 用途 / 布局 | 位置 |
|---|---|---|---|
| 格子表示 | 二维整型格 `(x,y)`，`0≤x<H`，`0≤y<W`；`x`=第一维（行），`y`=第二维（列）；朝向 `o∈[0,4)`，速度 `v∈{0,1}` | 运动基元偏移 `dx[4]/dy[4]`；状态编码 `((x*MXW+y)*4+o)*MXV+v` 仅用于排序 tie-break | `src/SIPP-IP.cpp:47-48,280` |
| 阻断区间表 `rsrv_tbl[MXH][MXW]` | `set<pair<int,int>>`，元素是闭区间 `[a,b]`（含端点），带哨兵 `(-1,-1)`、`(INF+1,INF+1)` | 每格一份"被动态/静态障碍占据的时间段"；查询模式恒为 `lower_bound({t, INF-1})` + 前驱 | `src/SIPP-IP.cpp:82,212-214` |
| 区间结构（节点标识） | `Bot{x,y,o,v,t_lower,t_upper,parentID}`（SIPP 系列）；A\* 为 `Bot{x,y,o,v,t,parentID}` | `[t_lower,t_upper]` 闭区间、整数时间步；SIPP-IP 中它是 waiting interval | `src/SIPP-IP.cpp:16-35`；`src/A-star.cpp:16-43` |
| OPEN 表 | `set<Bot>`，`operator<` 定义"优先队列"顺序 | 取最小 = `*OPEN.begin()`；**不是**堆，插入/弹出 `O(log n)` | `src/SIPP-IP.cpp:85,177-178` |
| CLOSED（去重/已访问） | A\*：`set<int> CLOSED[MXH][MXW][MXO][MXV]`（时间点）；SIPP：`set<pair<int,int>>`（区间） | 按"格×朝向×速度"分桶；SIPP 系列用"已访问区间"判定重叠 | `src/A-star.cpp:104`；`src/SIPP-IP.cpp:87` |
| CLOSED_vec | `vector<Bot>` | 节点流水账，下标即 `parentID`，用于回溯路径与结果代价 | `src/SIPP-IP.cpp:86,192-193` |
| 运动基元表 | `vector<Primitive> motion_primitives[MXO][MXV]`；`Primitive{vector<move> mvs; int o,v}`；`move{dx,dy,ftt,swt,isEndCell}` | `mvs` 里 `dx/dy` 是**相对基元起点**的绝对偏移（不是增量）；`o,v` 是终点配置 | `src/SIPP-IP.cpp:54-83,103-113` |
| 时间离散化 | **离散时间**：整数步长 `1/F=0.1 s`，闭区间 `[l,u]` | 不是连续时间；`min(INF,·)` 在上界截断 | `include/constants.h:7,10`；`src/SIPP-IP.cpp:115` |
| 内存布局 | 全为**静态全局数组**（`rsrv_tbl`、`CLOSED`、`motion_primitives`） | `CLOSED` 的 set 实例数 = `MXH·MXW·MXO·MXV`（当前 128×128×8=131072；若按 256×256 则 524288）；`generate_obstacles` 的 `rsrv` 三维 bool 是 256×256×4001 ≈ 262 MB | `src/SIPP-IP.cpp:82-87`；`src/generate_obstacles.cpp:12` |

**区间/时间语义小结**：`rsrv_tbl` 存"阻断"，安全区间是补集；`applyPrimitive` 内维护的 `timeIntervals` 在每个 move 上按帧平移（`ftt_j - ftt_{j-1}`），最终把"终点格首触时刻"平移 `swt_last` 得到"到达时刻"。闭区间、整数端点，因此判断一律用 `±1`（`itr_prev->second+1`、`itr->first-1-mv.swt`）。

---

## 9. 实验与结论

### 9.1 实验设计

- **算法 × 地图 × 障碍密度**：4 个算法（A\*、SIPP1、SIPP2、SIPP-IP）× 5 张地图 × 7 个 `factor`；但 `results/` 中**只有 SIPP1/SIPP2/SIPP-IP 三组各 35 个文件**，A\* 的结果目录不存在（`src/A-star.cpp:321` 写入 `results Astar/`，该目录缺失）。A\* 的对比只能从论文得到，仓库内无数据。
- 每个文件 1 行（`NumOfTests=1`），所以**每个 (算法,地图,密度) 只有 1 次试验**，无重复、无随机种子记录 —— 结论只能当趋势看。

### 9.2 汇总（由 `results/` 原文直接统计）

成功数 / 7 与平均 `Runtime`（微秒）：

| 地图 | SIPP1 | SIPP2 | SIPP-IP |
|---|---|---|---|
| Sydney_2_256 | 4/7，均 515,420 | 4/7，均 9,096,530 | **5/7**，均 4,155,480 |
| empty_64_64 | 4/7，均 5,552 | 4/7，均 109,299 | 4/7，均 83,381 |
| random128 | 4/7，均 59,531 | 4/7，均 969,350 | 4/7，均 714,670 |
| room-64-64-16 | 2/7，均 9,615 | 4/7，均 200,992 | 4/7，均 128,509 |
| warehouse-10-20-10-2-2 | 5/7，均 44,103 | 5/7，均 1,032,068 | **6/7**，均 542,280 |
| **合计** | **19/35** | **21/35** | **23/35** |

可读出的结论：

1. **SIPP-IP 解出最多实例（23/35）**，且在 `Sydney_2_256` 的 `obs9661` 与 `warehouse` 的 `obs2444` 上**只有它成功**（`results/results SIPP-IP/res-Sydney_2_256-SIPP-IP-obs9661.txt` 为 `Success:1, Cost:9141`；`.../res-warehouse-10-20-10-2-2-SIPP-IP-obs2444.txt` 为 `Success:1, Cost:5536`），而同密度下 SIPP1/SIPP2 均失败。这与论文"SIPP-IP 完备、原 SIPP 在运动学约束下不完备"的论点一致。
2. **SIPP-IP 在成功案例上的代价通常 ≤ SIPP1/SIPP2**（时间步）：`Sydney obs4830`：SIPP1 4087 / SIPP2 3587 / SIPP-IP 3491；`random128 obs1469`：4773 / 4478 / **3551**；`room obs182`：SIPP1 失败、SIPP2 3227、SIPP-IP **2422**。（注意各算法是独立最优，代价不同说明它们**并非解同一个最优问题**或 CLOSED 处理导致不同的最优性，需进一步核对。）
3. **SIPP1 是三者中最快的（单位 µs 量级）但成功率最低**：它不做重新入队、且 CLOSED 裁剪用"阻断区间起点"作下界（§5.2/5.3），牺牲正确性换速度；`empty_64_64` 上平均仅 5.5 ms。
4. **SIPP2 与 SIPP-IP 慢 1–2 个数量级**（重新入队 + 区间节点增多），但 SIPP-IP 比 SIPP2 **稳定更快**（除 `Sydney obs1932/2415` 外基本 ≤ SIPP2 的 1/2），因为区间携带了更多可等待信息、减少了无效重展开。
5. **密度趋势**：障碍越多成功率越低。最典型的是 `room-64-64-16` 与 `random128`：`factor=3`（最密）全员失败。`empty_64_64` 在 `obs1024/1365` 上三个算法都在 0.1 ms 内返回失败（`Success:0, Runtime≈10–100`），说明起点附近连一个完整运动基元都放不下（例如起点东向 5 格的加速/巡航通道被阻断），属于"立即无后继"而非搜索耗尽；**该判断是从耗时量级推断的，未逐步单步验证**。
6. **A\* 无数据**：仓库内无法给出 SIPP 相对 A\* 的加速比，只能引用论文结论（"runtime 比 A\* 低约两个数量级"，外部资料）。

---

## 10. 局限与坑

**建模层面**
1. **2D 四连通 + 4 个固定朝向**：`fillActions()` 只有沿 `dx[o]/dy[o]` 的直线基元（`src/SIPP-IP.cpp:280-303`）；没有任意角度、没有对角移动，拐弯只能"原地转 90°（20 步）"。
2. **匀速/匀加速假设 + 离散速度档**：`MXV=2`，只有"停/巡航"两档；加速基元固定 4 格/40 步（注释 `a=±0.5 m/s², v_max=2 m/s`，`:282`），无法表达"从巡航减到半速再加速"等中间速度，也不是任意加速度的连续取值。
3. **时间是离散的**：所有区间是 0.1 s 的整数闭区间；真实连续时间下的"刚好晚 0.03 s 出发可行"会被量化误差吃掉（保守或激进都取决于取整方向）。上界 `INF+1` 截断也可能把临近地平线的两个安全区间错误合并/截断。
4. **无碰撞体积/无 agent 半径**：状态只到"格心"，障碍就是"被占据的格"；没有把机器人尺寸膨胀进地图（这在用户项目里由 `rasterize_free_space` 的 `inflate` 承担）。两个相邻格都"自由"但机器人实际过不去的情形不会被察觉。
5. **加速基元要求一条 5 格直线通道**（偏移 0..4 的格子在相应时间窗内都自由，`src/SIPP-IP.cpp:114-132`），比真实运动更保守：4 格宽的通道无法完成加速。
6. **障碍被建模为"整格 + 闭区间阻断"**，没有尺寸/形状，且到达终点后永久停在终点格（阻断到 `MXT`，`src/generate_obstacles.cpp:125-127,192-196`）。
7. **成本 = 到达时间**，目标必须**速度归零** `v==end_v(=0)`（`src/SIPP-IP.cpp:194`）；没有把"离目标多近"或"朝向"计入目标判定。

**实现层面**
8. **`CLOSED` 裁剪只看最后一个区间**：`checkCLOSED` 取 `--CLOSED[x][y][o][v].end()`，源码注释直接承认 "there shouldn't be any interval in closed with lower_bound > t_lower"（`src/SIPP-IP.cpp:98`）。若该假设被破坏（例如多次非单调插入后），去重会不完整 → 可能重复展开或漏解。
9. **失败时的越界读**：搜索失败返回 `{0,false}`，但 `main` 仍然打印 `CLOSED_vec[ans.first].t_lower`，即 `CLOSED_vec[0]`——若一次展开都没发生，`CLOSED_vec` 为空，这是未定义行为（`src/SIPP-IP.cpp:205,348`）。结果里 `Cost:0` 只是"碰巧"。
10. **`cntNodes` 不是节点计数**：弹出 `--cntNodes`、入 CLOSED `++cntNodes`、每个后继 `++cntNodes`（`src/SIPP-IP.cpp:179,191,202`），它更像"分配过的节点对象数减弹次数"的启发式预算；`MAX_NUM_NODES` 的语义因此模糊。
11. **静态数组越界风险**：`rsrv_tbl`/`CLOSED` 按 `MXH/MXW`（`constants.h`）分配，但 `readInputs` 用地图文件里的 `H,W` 当循环上界（`src/SIPP-IP.cpp:228-236`）；若地图比常量维度大则越界。`numOfFreeCells()` 还用固定 `bool mp[600][600]`（`:308`）。
12. **I/O 强耦合相对路径**：必须从 `src/` 运行；`A-star` 的输出目录不存在导致结果丢失；`generate_obstacles` 第 332 行路径写错（`"../" + maps[k]`），该次读取静默失败（`:332-339`）。
13. **`assert` 依赖**：`SIPP1/2` 用 `assert` 校验阻断区间不相交（`src/SIPP2.cpp:290`），`-DNDEBUG` 下这些检查消失。
14. **生成器两种分支语义不一致**：只有 `empty_64_64`（与已不存在的 `random-64-64-10`）走"障碍互不碰撞"的时间相关分支；其余地图的障碍轨迹**互不避让、可在时空上重叠**（`find_and_mark_path` 从不读 `rsrv`，`src/generate_obstacles.cpp:220-236`）。跨地图成功率差异部分来自这一点，而非地图难度。
15. **`timestep` 字段名与真实单位不符**：文件里是秒，内部是 0.1 s 步；`readInputs` 的 `×F` 是正确换算，但任何外部消费者按字段名理解都会错 10 倍。
16. **每格只有 1 次试验**（`NumOfTests=1`），无随机种子、无多次平均；`Runtime` 未剔除 I/O 与 `clr()` 开销。
17. **`end_v` 语义**：三个 SIPP 源码把 `end_v` 设为 0（`src/SIPP-IP.cpp:343`），但 `operator<` 的 `h` 里完全不使用朝向/速度，因此目标速度约束只影响可行性判定，不影响启发式质量。

---

## 11. 对用户项目（`bullet_sim/safety/`）的可借鉴点

用户现状（已读代码）：`safety/` 是"配置空间膨胀 + 到达时间波前 + 回溯路径"三级结构：

- `free_space.py:189-209` 的 `rasterize_free_space` 按 `sdf(obstacle, cell_centre) ≥ player_circumradius + margin` 精确膨胀（`free_space.py:145-186`），`FreeSpaceSequence` 保存一个时间窗内的逐帧布尔自由格（`free_space.py:212-255`）。
- `path_search.py:203-295` 的 `reachable_set` 在**每个时间片**上做"到达时间波前"（标量 `arrival` 数组 + 逐片 `np.where(layer, arrival, inf)` 失效化），`movement_kernel` 把 `max_speed`/`max_accel` 折算成"单片的位移半径"（`path_search.py:82-121,236-243`）；`backtrack_path` 再从最后一层贪心回溯（`path_search.py:298-347`）。
- `validate.py:117-118` 的 `SafeRegion.always_free` 是**所有采样帧的交集**，`safe_points` 用 `reachable & always_free`（`validate.py:128-135`）——这会把"早先自由、稍后可枯竭"的格子直接判死，偏保守。

SIPP 的**安全区间**表示恰好能修掉这个过度保守，并把"最大速度/加速度"从"每片位移半径"升级为**带速度状态的区间投影**。以下建议按"可以直接落地成新文件/新函数"的粒度给出。

### 11.1 用"逐格安全区间"替换"逐帧布尔交集"

新增 `bullet_sim/safety/intervals.py`（或并入 `free_space.py`）。核心思想：`FreeSpaceSequence` 已经给了每个采样时刻的自由/占据布尔场，把它**转置**成"每格一条时间轴上的阻断区间列表"，其补集就是安全区间——与 `rsrv_tbl` 完全同构。

```python
@dataclass(frozen=True)
class Interval:
    start: float          # 闭区间左端（秒），与 safety 现有时间单位一致
    end: float            # 闭区间右端；end==inf 表示右开

@dataclass
class IntervalField:
    """每格的阻断区间与安全区间（长度 rows*cols 的扁平列表，SoA 便于向量化）。"""
    rows: int
    cols: int
    cell_w: float
    cell_h: float
    dt: float                      # 采样步长（= SafetyConfig.sample_dt）
    blocked: list[list[Interval]]  # blocked[r*cols+c]：升序、互不相交
    safety:  list[list[Interval]]  # safety[r*cols+c]：blocked 在 [0,horizon] 上的补集

    def intervals_of(self, r: int, c: int) -> list[Interval]: ...


def build_interval_field(
    seq: FreeSpaceSequence,
    *,
    horizon: float | None = None,
    closed: bool = True,
) -> IntervalField:
    """把逐帧 free 布尔场压成逐格安全区间。

    对每个格 c：blocked = merge{ [t_k, t_k+dt] | free_k[c] == False }，
    safety = complement(blocked) ∩ [0, horizon]。与 SIPP 的 rsrv_tbl 一一对应。
    """
```

这样 `SafeRegion.always_free`（`validate.py:117-118`）就可以改为"区间求交"——**只对同一格的重叠时段取交**，而不是对整张图取交：

```python
def intersect_intervals(a: list[Interval], b: list[Interval]) -> list[Interval]: ...
```

### 11.2 把运动学约束表达成"带 (首触时刻, 扫掠时长) 的运动基元"

这是本项目最核心、也最可直接搬用的数据结构（`Primitive::move{dx,dy,ftt,swt,isEndCell}`，`src/SIPP-IP.cpp:56-61`）。用户现在的 `movement_kernel` 只给一个位移半径，丢失了"同一基元扫过哪些格、每格占用多久"的时序信息，也无法表达"必须先加速才能到巡航"。

```python
@dataclass(frozen=True)
class SweptCell:
    dr: int; dc: int          # 相对基元起点的格偏移（绝对偏移，不是增量）
    first_touch: float        # ftt：首次触碰该格的时刻（相对基元起点，秒）
    sweep: float              # swt：持续占用时长（秒）
    is_end: bool = False      # 终点格；swt = 到基元结束的剩余时长

@dataclass(frozen=True)
class KinematicPrimitive:
    swept: tuple[SweptCell, ...]
    v_end: int                # 0 = 终点可等待；>0 = 必须继续动
    duration: float
    cost: float               # 本例中 cost == duration
    label: str = ""


def build_primitives(
    *, cell_w: float, cell_h: float, v_cruise: float, a_max: float,
    dt: float, allow_diagonal: bool = True,
) -> dict[tuple[int, ...], tuple[KinematicPrimitive, ...]]:
    """由物理极限生成基元集合（镜像 constants.h 里硬编码的 costs 表）。

    典型：cruise 1 格 = cell/v_cruise；accel 0->v_cruise 覆盖
    v_cruise^2/(2*a_max) 距离，用离散采样得到每格的 (first_touch, sweep)。
    返回值按 (朝向, 速度档) 索引，与 motion_primitives[o][v] 对应。
    """
```

### 11.3 用区间投影替换"每片位移半径"的波前

```python
def project_interval(
    field: IntervalField,
    cell: tuple[int, int],
    interval: Interval,
    prim: KinematicPrimitive,
    *,
    t_max: float,
) -> list[Interval]:
    """把父节点在 cell 上可出发的时间区间沿 prim 投影到后继配置。

    对 prim.swept 的每个 SweptCell：
      候选出发时刻集合 C（初值 = [interval.start, interval.end]）
      平移到该格的首触帧：C += (sc.first_touch - prev_first_touch)
      与 safety[sc.dr, sc.dc] 求交，并要求整段扫掠 [t, t+sc.sweep] 自由
      （即 new_end = min(cand.end, si.start - sc.sweep)，与 C++ 的
        itr->first - 1 - mv.swt 等价，只是这里连续时间不需要 ±1）
    终点格且 prim.v_end == 0 时，把上界放宽到该格安全区间右端
    返回 [t + last.sweep, t' + last.sweep] 形式的到达区间列表
    """
    ...
```

**注意与 `path_search.py:236-243` 当前近似的关系**：现在用
`v_end = min(speed, v0 + a_max*dt); step = 0.5*(v0+v_end)*dt`
把一个时间片内的加速压成一个"位移半径"，这是**一阶近似**，且丢失了"片中间某刻不能停"的信息。改成基元 + 区间投影后，加速度是**几何上精确**的分段运动（与 `SIPP-IP` 的 `costs` 表同理），且 `v_end==0` 的基元才允许把到达区间延长到安全区间右端——这正是解决"必死场景"误判的关键：**能停下来的时刻集合是可等待区间，停不下来的时刻集合只是投影结果本身**。

### 11.4 搜索节点与主循环（替换 `reachable_set` 的标量到达时间）

```python
@dataclass
class SippPlan:
    cells: list[tuple[int, int]]
    intervals: list[Interval]        # 每个节点的 waiting interval
    times: list[float]               # 取 interval.start 作为执行时刻
    v_end: list[int]
    expansions: int = 0              # 与论文对齐的"扩展节点数"指标

def sipp_search(
    field: IntervalField,
    *,
    start_cell: tuple[int, int],
    start_time: float = 0.0,
    goal_test: Callable[[tuple[int, int]], bool] = lambda rc: True,
    primitives: Callable[[tuple[int, int], int], Sequence[KinematicPrimitive]] = ...,
    heuristic: Callable[[tuple[int, int]], float] | None = None,
    t_max: float | None = None,
    max_expansions: int = 200_000,
    closed_policy: Literal["trim_and_requeue", "skip"] = "trim_and_requeue",
) -> SippPlan | None:
    """(cell, v_bin, [t_lower,t_upper]) 节点上的 A*。

    - g(n) = n.interval.start（最早到达）；
    - 起点区间 = 起点所在格的整个安全区间 ∩ [start_time, ∞)（对应
      src/SIPP-IP.cpp:197-206）；
    - 后继：对每个基元做 project_interval，得到若干候选区间；
    - closed_policy="trim_and_requeue" 时，若候选区间与已闭合区间重叠，
      把 t_lower 抬到 max(t_lower, closed.end) 后重新入队（对应
      src/SIPP2.cpp:212-220 / SIPP-IP.cpp:185-189），这是 SIPP1 与
      SIPP2/SIPP-IP 成功率差异的来源；
    - 返回的 SippPlan 可直接喂给现有的轨迹后处理/渲染。
    """
```

`ReachabilityResult`（`path_search.py:154-200`）可保留作为兼容 API，但建议**新增** `safe_intervals: list[list[Interval]]` 字段（或 `IntervalField` 引用），让 `SafeRegion.safe_points` 用区间而非"always_free 交集"来选点。同时给 `ReachabilityResult` 增加 `expansions: int` / `interval_count: int`，对齐论文里"扩展节点数远小于 A\*"的对比指标。

### 11.5 与现有接口的对接建议

1. **输入不变**：`build_interval_field(free_space_sequence(world, config, spec=spec))` 复用 `validate.py:246-279` 的逐帧栅格化（真实物理克隆），所以障碍动力学仍然是"真实 World"，不引入第二套模型。
2. **`SafetyConfig` 扩展**（`validate.py:59-95`）：新增 `use_intervals: bool = True`、`v_cruise: float | None = None`、`max_expansions: int = 200_000`、`closed_policy: str = "trim_and_requeue"`；`max_accel` 已有（`validate.py:73`）可直接用于 `build_primitives`。
3. **`validate_scenario` 的判定链**（`validate.py:540-572`）不变：仍是"起点自由 → 自由空间下限 → 可达性 → 回溯出路径"；只把第 3/4 步换成 `sipp_search` + 区间存在性判定。`relax_scenario`（`validate.py:663-691`）无需改。
4. **确定性**：建议把时间量化成整数 tick（例如 `F=120`，`INF=horizon*F`），所有区间端点取整数，比较用闭区间 `±1`——这与本项目 `F=10/INF=40000` 的做法一致（`include/constants.h:7,10`），也让未来 CPU/FPGA 定点实现与 Python 浮点实现的结果**逐位可复现**。用浮点秒做区间端点会引入"0.03 s 之差决定生死"的不可复现风险。
5. **验证方法**：照搬本项目的"嵌套密度扫描"思路——对同一场景按 `obstacle_count` 生成的**前缀集合**逐级加密（`generate_obstacles.cpp:329-362`），记录每个密度的成功率/扩展节点数/耗时，就能得到"必死场景率随密度变化"的曲线；这正是 `safety/` 最需要对外证明的指标。
6. **可直接避免的两个代码级坑**：不要只检查"CLOSED 里最后一个区间"（`src/SIPP-IP.cpp:98` 的假设），而要做完整的区间集合求交；不要把失败路径当成功路径返回（本项目 `CLOSED_vec[0]` 越界读，`src/SIPP-IP.cpp:205,348`）。

---

## 附：核验清单（哪些是"读到的"，哪些是"推断的"）

- **读到的（仓库内证据）**：四个算法的全部函数与常量、`constants.h` 每一项、地图/实例/结果三种文件格式、`tests/` 与 `results/` 的规模、`README.md`/`LICENSE` 全文、`git log` 作者与日期、`results/` 全部数值、`A*` 结果目录缺失、`empty_1_16` 无对应地图、`generate_obstacles` 生成逻辑与两处路径/语义坑。
- **网络检索补充（非仓库证据）**：论文题名/作者/会议/DOI、SIPP-IP 的 waiting interval 与投影的形式化定义、论文的完备性证明与选例。已在 §1、§2.3、§5.4 明确标注。
- **明确未确认**：① 仓库与论文的对应关系（README 无引用）；② `results/` 数值是否由当前源码版本生成（无种子/无版本记录）；③ `empty_64_64 obs1024/1365` 的"起点无可行基元"只是耗时量级的推断，未单步验证；④ SIPP1/SIPP2/SIPP-IP 三者最优代价不一致的根因（可能是 CLOSED 语义差异，未做单例复算）。
