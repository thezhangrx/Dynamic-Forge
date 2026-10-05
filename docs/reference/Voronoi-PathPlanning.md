# Path-Planning-Robot-over-Voronoi-Diagram — 技术总结

> 本文只读代码与数据得出，不构建、不联网。所有结论后标注来源（相对项目根的 `文件:行`）。
> 无法核实的地方统一写「未确认（原因）」。我用 Python 复算过的地方标注为「实测」。

---

## 1. 一句话定位

一个 2019 年的**教学/演示级** C++ 头文件库（3 个头文件 + 1 个 `main`）加 11 行 MATLAB 脚本：把二维静态多边形障碍地图**在整数格点上采样**，用「到最近两个障碍等距」这一判据近似出**广义 Voronoi 图（GVD，也就是中轴骨架）**，再在骨架点云上贪心地走出一条从 A 到 B 的折线，最后 dump 成 4 个纯文本文件交给 MATLAB 画图。

- 原作者：**Simone Tinella，University of Catania**（每个源文件首行注释，`programma.cpp:1`、`Punto.h:1`、`Ostacolo.h:1`、`Voronoi.h:1`）。
- 仓库：`https://github.com/SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram.git`（`.git/config` 的 `remote.origin.url`），当前 HEAD = `c009820`（`Update README.md`），`git log --oneline --all` 共 16 个提交。
- 许可证：**MIT License**（`LICENSE:1`），`Copyright (c) 2019 Simone Tinella`（`LICENSE:3`）。git 历史显示 LICENSE 是后期补加（提交 `c01f45a "Create LICENSE"`、`3d4dc5f`/`8753f8a "Update LICENSE"`）。
- 它**不是**一个可复用的规划库：没有构建系统、没有测试、库层会 `exit(-1)`、内存大量泄漏（详见第 8 节）。它的价值在于「算法思路 + 一张可对照的实现地图」。

---

## 2. 技术栈与工具链

| 项目 | 实际情况 | 依据 |
|---|---|---|
| 语言 | C++，且只用了 C++98/03 级别的特性（`std::vector`、裸 `new`、`goto`、`std::sort`）；未见 `auto`/`nullptr`/range-for 等 C++11 特性 | `Voronoi.h:3-7`、`Voronoi.h:231` 等 |
| 标准库头 | `<iostream>` `<vector>` `<fstream>` `<math.h>` `<algorithm>`，全部是系统标准库 | `programma.cpp:3-6`、`Voronoi.h:3-7`、`Ostacolo.h:3-5` |
| 第三方依赖 | **无**。没有 Boost / CGAL / Boost.Polygon，也没有任何 vendored 代码。**Voronoi 图完全自研**（`Voronoi.h:86` `CreaVoronoi` 手写格点采样） | 全目录仅 11 个非 `.git` 文件，见第 3 节 |
| 构建方式 | **没有** Makefile / CMakeLists / 任何构建脚本。只能手工 `g++ programma.cpp -o programma`（此命令是我的推断，README 未给出编译方法） | 目录全量列表 |
| 运行平台 | 可移植 C++（无平台 API）；绘图侧需 MATLAB 或 Octave | `plotta.m:1-4` |
| 数值类型 | 全程 IEEE-754 `double`（`Punto.h:5`），但输入坐标与格点步长都是整数，等价于「整数坐标 + 实数距离」。**无定点、无量化误差控制** | `Punto.h:5`、`Voronoi.h:9` |
| 交互方式 | 无 GUI、无实时；一次性批处理：建图 → 规划 → 写 4 个 txt → MATLAB 画图 | `programma.cpp:66-90`、`plotta.m` |
| 进度反馈 | 建图时按行打印百分比并用 `\r` 覆盖（意大利语） | `Voronoi.h:90-91` |

**是否编译通过：未确认（本次任务禁止构建，且我没有安装/调用 g++ 的必要）。** 仅从代码看有几处可移植性隐患（`exit` 未显式包含 `<cstdlib>`，见第 8 节）。

---

## 3. 目录结构与模块地图

项目极小，除 `.git/` 外共 11 个文件，全部列出：

| 文件 | 行数 | 职责 |
|---|---|---|
| `README.md` | 42 | 唯一的文档：API 用法片段 + 一句「顶点要顺时针插入」+ 一张 GitHub 图片链接 |
| `LICENSE` | 21 | MIT 许可证全文 |
| `Punto.h` | 23 | 二维点类 `Punto`：`double x,y` + `getX()/getY()`。无 setter、无 `const`、无析构、**无 include guard** |
| `Ostacolo.h` | 74 | 障碍物类 `Ostacolo`：顶点环 `vertici` + 边界栅格点集 `ingombro`；构造时把多边形边按 1 单位步长离散；静态欧氏距离 `Distanza` |
| `Voronoi.h` | 339 | **核心**：5 个调参常量 + 类 `Voronoi`（构造即建图）+ `CreaVoronoi` / `getPercorso` / `getPercorsoVoronoi` / `getPercorsoVoronoi2` |
| `programma.cpp` | 91 | `main`：硬编码 5 个多边形 + 1 个点障碍 + 200×200 地图 + 起终点；调用后把障碍/Voronoi 点/路径/交叉点写 4 个 txt |
| `plotta.m` | 11 | MATLAB 可视化：`load` 4 个 txt，5 次 `plot` |
| `ostacoli.txt` | 1017 | 运行产物：所有障碍的边界栅格点（`x y` 每行），顺序拼接 |
| `voronoi.txt` | 1283 | 运行产物：Voronoi 点坐标，行主序 |
| `incroci.txt` | 18 | 运行产物：交叉点（骨架分支节点）坐标，是 9 个几何节点周围的点簇 |
| `percorso.txt` | 29 | 运行产物：最终路径折线，首行起点、末行终点 |
| `img.jpg` | 842×707 | README 配图的本地副本；红=障碍、绿=Voronoi 点、黄=路径、红点=交叉点 |
| `.git/` | — | 版本库；`HEAD` → `refs/heads/master` = `c009820`；另有远端分支 `origin/lic-patch-1-1` = `8753f8a` |

调用链（`main` 视角）：
`Punto` 顶点 → `new Ostacolo(&vertici)`（栅格化边界）→ `new Voronoi(larghezza, lunghezza, ostacoli)`（内部自动追加地图边界障碍并建图）→ `getPuntiVoronoi()/getIncroci()/getOstacoli()` → `getPercorso(partenza, arrivo)` → 4 个 `std::ofstream` dump（`programma.cpp:10-90`）。

---

## 4. 核心算法逐个拆解

### 4.1 障碍物边界栅格化 — `Ostacolo::Ostacolo`

**位置**：`Ostacolo.h:17-64`。

**数学含义**：把多边形边界 `∂P = ∪ᵢ [vᵢ, vᵢ₊₁]` 离散成一个整数格点集合 `ingombro`。
对每条边，代码**按 x 方向均匀步进**（不是按弧长），y 由直线插值给出（`Ostacolo.h:54`）：

```
y(x) = ((x - x1) / (x2 - x1)) * (y2 - y1) + y1
```

**输入**：`std::vector<Punto*>* vert`（顶点环，顺序任意，靠取模自动闭合，`Ostacolo.h:23`）。
**输出**：成员 `ingombro`（`Ostacolo.h:10`），即边界上的格点集。

**关键步骤**（`Ostacolo.h:20-63`）：

1. 若 `vertici.size() > 1`（`Ostacolo.h:20`），对每条边 `i → (i+1) % n`：
   - `tmp = p1`（起点），`tmp_x/tmp_y = p1`（`Ostacolo.h:30-32`）。
   - `do { ingombro.push_back(tmp); 推进一步; tmp = new Punto(tmp_x, tmp_y); } while (!(tmp == p2))`（`Ostacolo.h:34-59`）。
   - 推进一步分三种情况：垂直线 `p1x==p2x` 则 `tmp_y ± 1`（`Ostacolo.h:38-42`）；水平线 `p1y==p2y` 则 `tmp_x ± 1`（`Ostacolo.h:43-47`）；否则斜线，`tmp_x ± 1` 后按上面的插值算 `tmp_y`（`Ostacolo.h:48-55`）。
   - 由于起点由本边 push，终点作为下一条边的起点 push，**每个格点恰好出现一次**（顶点不重复）。
2. 若只有一个顶点，`ingombro` 就是该单点（`Ostacolo.h:62`）——这就是点障碍的实现方式（`programma.cpp:45-47`）。

**关键参数**：栅格精度硬编码为「沿 x 或 y 走 1 个单位」，没有可配参数；与 `Voronoi.h:9` 的 `unit = 1` 是同一个隐含单位（两处各自硬编码，没有共享常量）。

**复杂度**：时间/空间 `O(Σᵢ Lᵢ / unit)`，`Lᵢ` 为第 i 条边的轴对齐长度。

**实测核对**：按代码逐边推算，`programma.cpp` 的 6 个障碍共产生 `40 + 30 + 20 + 70 + 60 + 1 = 221` 个点，地图边界矩形产生 `4 × 199 = 796` 个点，合计 `1017` —— 与 `ostacoli.txt` 的 1017 行**完全吻合**；且该文件第 41 行正好是第 2 个障碍的起点 `80 50`（推算与文件一致）。

**坑**：`ingombro` 名字（意大利语 "encumbrance"）容易被理解成「膨胀后的占据区域」，但它**只是 1 单位间隔的边界采样点集**：既不是填充面片，也不是任何形式的膨胀（详见 6.3）。
**坑**：若相邻顶点重合（`p1 == p2`），斜线分支的分母 `p2x - p1x = 0`、分子也为 0，`tmp_y` 变 `NaN`（`Ostacolo.h:54`），`while` 条件永不成立 ⇒ `ingombro` 无限增长直到内存耗尽。

### 4.2 广义 Voronoi 图构造 — `Voronoi::CreaVoronoi`

**位置**：`Voronoi.h:86-129`（由构造函数 `Voronoi.h:70` 调用）。

**数学原理**：设 site 集合为各障碍物 `O₁…O_K`，定义
`d_k(p) = min_{q ∈ ∂O_k} ‖p − q‖`（代码用 `ingombro` 里的离散点近似，`Voronoi.h:101-106`）。
把 `d_(1)(p) ≤ d_(2)(p) ≤ …` 记为按升序排列的距离，则

```
GVD = { p : d_(1)(p) = d_(2)(p) }
```

即「到最近两个障碍等距」的点集 —— 各 Voronoi 胞腔的公共边界，也就是**最大间隙轨迹（中轴骨架）**：在骨架上的点，局部 clearance 达到极大，这正是它作为机器人路网的价值。

**离散判据**（`Voronoi.h:116`）：

```
| d_(2)(p) − d_(1)(p) | ≤ dist_tolleranza = √2 · unit ≈ 1.4142
```

容差把理论上的「等距曲线」在格点上展宽成 1~2 格宽的带状点集。

**输入**：`larghezza`（x 方向宽）、`lunghezza`（y 方向长）、障碍列表；构造时**自动追加一个地图边界矩形障碍**（`Voronoi.h:41-46`，顶点 `(0,0) (W−1,0) (W−1,L−1) (0,L−1)`）。
**输出**：成员 `punti_voronoi`（`Voronoi.h:21`）与 `incroci`（`Voronoi.h:22`）。注意 `incroci ⊂ punti_voronoi`（同一格点先 push 进前者，`:117`，再可能 push 进后者，`:121`）——**实测**：18 个 incroci 坐标全部出现在 `voronoi.txt` 中。

**关键步骤**：

1. 二重循环遍历内部格点：`tmpY ∈ [unit, lunghezza−unit)`、`tmpX ∈ [unit, larghezza−unit)`，步长 `unit = 1`（`Voronoi.h:88`、`Voronoi.h:93`）。对 200×200 地图即 `198 × 198 = 39204` 个候选点（最外一圈被排除）。
2. 对每个候选点，对**每个障碍**求 `min` 距离：内层再遍历该障碍 `ingombro` 的所有点（`Voronoi.h:99-108`，距离用 `Ostacolo.h:71-73` 的 `sqrt`），得到一个长度 `K` 的距离数组。
3. `std::sort` 对距离数组**全排序**（`Voronoi.h:111`，写成 `distanze.begin(), distanze.begin()+distanze.size()`，等价于 `end()`）；取 `min1 = distanze.at(0)`、`min2 = distanze.at(1)`（`Voronoi.h:113-114`）。
4. `|min2 − min1| ≤ dist_tolleranza` ⇒ 该格点是 Voronoi 点，push 进 `punti_voronoi`（`Voronoi.h:116-117`）；否则 `delete tmp_punto` 丢弃（`Voronoi.h:124`，全文件唯一的 `delete`）。
5. `ostacoli.size() > 2` 且 `|min2 − min3| ≤ tol` 且 `|min3 − min1| ≤ tol` ⇒ 同时是 `incroci`（`Voronoi.h:119-122`）。

**关键参数**（全部定义在 `Voronoi.h:9-14` 的文件级 `const`，即全局调参旋钮）：

| 常量 | 定义处 | 默认值 | 作用 |
|---|---|---|---|
| `unit` | `Voronoi.h:9` | `1` | 格点步长/长度单位 |
| `dist_tolleranza` | `Voronoi.h:11` | `unit·√2 ≈ 1.4142` | 4.2 的等距容差 |
| `raggio_ricerca` | `Voronoi.h:10` | `5·unit·√2 ≈ 7.0711` | 4.5 贪心搜索的邻域半径 |
| `distanza` | `Voronoi.h:13` | `5·unit` | 4.4 的路径抽稀阈值（注释 `//filtro`） |
| `scarto` | `Voronoi.h:14` | `1` | 4.4 的等步长抽稀步长（注释 `//filtro`） |

**复杂度**：时间 `O(W·H·(M + K log K))`，`M = Σ_k |ingombro_k|`；本例 `198×198×1017 ≈ 4.0×10⁷` 次 `sqrt`，外加 39204 次 `K=7` 的排序。空间 `O(N + M)`（`N = |punti_voronoi|`）。此外每个候选点都 `new` 一次再 `delete`（`Voronoi.h:96`/`:124`），分配次数 ≈ 39204。

**实测核对**：
- `voronoi.txt` 共 1283 行，即 `N = 1283`；全量 awk 核查**没有非整数坐标**（与格点扫描一致）；按 y 非递减排列（行主序，与 `y` 外层 `x` 内层的循环顺序一致）；无重复点。
- 用射线法逐个检查 1283 个点：**没有任何一个点落在 5 个多边形障碍的内部或边界上，也不落在点障碍上**（Voronoi 骨架确实在自由空间）。
- 每个 Voronoi 点到最近障碍的 clearance：中位数 ≈ 25.3，最小 10.0（在 `(10,76)` 一带，到左墙距离，因为地图边界也是 site），最大 ≈ 56.5（在 `(142,117)`，两条大间隙交汇处）。

**判据的离散误差来源**：`d_k` 是「到边界采样点集」的距离，因为采样点 `⊂` 真实边界，所以只会**高估** clearance；且等距带用 `√2`（格点对角长，即离散化的最大近邻跳距）作为容差，是**经验取值**，不是误差分析的结果。

### 4.3 骨架分支节点检测（`incroci`）

**位置**：`Voronoi.h:119-122`。

**数学原理**：`p` 是 GVD 的 3 度（或更高）顶点 ⟺ 到最近**三个** site 的距离近似相等（三条 Voronoi 边的交点）。代码用 `|min3 − min1| ≤ √2` 与 `|min2 − min3| ≤ √2` 联合判定。

**输入/输出**：无需额外输入；对通过等距判据的格点追加进 `incroci`（`Voronoi.h:22`）。

**为什么需要它**：4.5 的贪心搜索会走进「局部极小」（四个方向都离目标更远）的死胡同，作者用 `incroci` 当**中转路标**来兜底（`Voronoi.h:245-280`）。

**关键参数**：复用的 `dist_tolleranza`（`Voronoi.h:11`）；`ostacoli.size() > 2` 是防止 `distanze.at(2)` 越界的守卫（`Voronoi.h:119`）——因为边界矩形总会被追加，`K ≥ 2` 恒成立，配 1 个用户障碍时 `K = 2` 正好不取第三个。这是我读代码推出的隐式不变量。

**复杂度**：无额外开销（在 4.2 的循环内顺便判）。

**实测**：`incroci.txt` 有 18 个点，但按欧氏距离 ≤ 2 聚类只有 **9 个几何节点**：`{(57,28),(58,28)}`、`{(119,32)}`、`{(17,63)}`、`{(120,72),(120,73)}`、`{(62,76),(62,77),(62,78),(62,79),(63,77)}`、`{(144,104),(144,105)}`、`{(58,112),(59,111)}`、`{(142,117),(142,118)}`、`{(40,129)}`。也就是说 `incroci` 是**点簇**而不是「唯一节点表」。这在 4.5 的兜底逻辑里会放大问题：那里用**指针相等**判断某个 incrocio 是否用过（`Voronoi.h:251`），同一个几何节点周围的 5 个格点会被当成 5 个不同路标。

### 4.4 全局路径提取 — `Voronoi::getPercorso`

**位置**：`Voronoi.h:133-189`。

**性质**：把「起点/终点」接到骨架上的**近邻吸附 + 骨架行走 + 抽稀**三段式；本身不含搜索。

**输入**：`partenza`、`arrivo`，**按值传 `Punto`**（拷贝）。
**输出**：`std::vector<Punto>`（值语义；内部搜索用 `vector<Punto*>`，到这里才转成值拷贝）。

**关键步骤**：

1. `percorso = [partenza]`（`Voronoi.h:137`，注释说明这只是临时占位）。
2. 线性扫描全部 Voronoi 点，分别找距 `partenza` 最近者 `minimo`、距 `arrivo` 最近者 `minimo_arrivo`（`Voronoi.h:143-155`）。**用 `10000` 当「无穷大」哨兵**（`Voronoi.h:138-139`）。
3. 若任一为 `NULL`，返回只有起点的列表（`Voronoi.h:157-158`）。
4. push `*minimo`（`Voronoi.h:160`）；若 `minimo` 坐标恰等于 `arrivo` 则提前返回（`Voronoi.h:162`）。
5. 调 `getPercorsoVoronoi(*minimo, *minimo_arrivo, NULL)` 得到骨架子路径（`Voronoi.h:164`）。
6. **抽稀 1（弦长阈值）**：保留首点，其后仅当与「上一个保留点」的欧氏距离 `≥ distanza = 5` 时才保留（`Voronoi.h:167-175`）。注意这是**阈值抽稀**，不是 Douglas–Peucker：不看中间点的偏差，因此会**切角**。
7. **抽稀 2（等步长）**：`for(i = 0; i < size; i += scarto)`（`Voronoi.h:177-181`）。默认 `scarto = 1`，因此这一步是**恒等操作**（预留参数/死代码）。
8. push `*minimo_arrivo`、`arrivo`（`Voronoi.h:185-186`）。

**关键参数**：`distanza = 5`（`Voronoi.h:13`）、`scarto = 1`（`Voronoi.h:14`）。

**复杂度**：`O(N)` 找最近点 + `O(P)` 抽稀（`P` 为骨架子路径长度）。

**缺口**：**连接段没有碰撞检查**。`partenza → minimo` 与 `minimo_arrivo → arrivo` 都是直接直线连接，抽稀也会切角，而代码从不检查这些直线是否穿过障碍。

**实测核对（`percorso.txt`，29 点）**：
- 首行 = 起点 `(20,140)`，末行 = 终点 `(160,70)`；除这两个端点外，**其余 27 点全部是 `voronoi.txt` 里的点**（实测集合包含检查通过）。
- 相邻点距离：首段 17.088（跨越 `partenza → minimo` 的连接段，不受抽稀约束）、末段 12.728（同理由 `minimo_arrivo → arrivo`），中间各段 6.32 ~ 7.07，全部 `≥ 5` —— 与抽稀逻辑一致。
- 第 27、28 行**同为 `(169,79)`**：这是 `minimo_arrivo` 被 push 两次（搜索终点已经 push 过一次，`Voronoi.h:234`；随后 `Voronoi.h:185` 又 push 一次）造成的可观测重复点缺陷。
- 28 条线段与任何障碍边都**不相交**，线段中点也不在任何障碍内部（实测）：本例路径无碰撞，但这是场景运气（起终点附近 clearance = 20），**不是算法保证**。

### 4.5 骨架上的行走搜索 — `Voronoi::getPercorsoVoronoi`

**位置**：`Voronoi.h:193-287`。**回答关键问题：既不是 Dijkstra，也不是 A\*。**

**性质**：在**隐式图**（顶点 = `punti_voronoi` 点集；边 = 半径 `raggio_ricerca` 内的近邻关系）上做**贪心最优优先（greedy best-first / hill climbing）**：每步只挑「离目标最近的邻居」，并要求它让 `‖· − arrivo‖` **严格下降**；没有代价累积 `g`、没有 `f = g + h`、没有优先队列、没有闭集松弛。因此它**既不最优也不完备**。

**输入**：`partenza`、`arrivo`（值拷贝）、`incroci_s`（已用过的 incrocio 列表，用于递归时避免重复；首次调用传 `NULL`）。
**输出**：`std::vector<Punto*>`（指向 `punti_voronoi` 中的对象，借用手性）。

**关键步骤**：

1. 初始化：`punti_passati = [&partenza]`（`Voronoi.h:196`，存的是**局部形参 `partenza` 的地址**），`tmp = &partenza`（`Voronoi.h:198`）。
2. `do { … } while (!(tmp == arrivo))`（`Voronoi.h:200`、`:283`）：
   - **邻域查询**：扫描全部 Voronoi 点，取 `‖tmp − p‖ ≤ raggio_ricerca` 且「指针不在 `punti_passati` 里」的点作为 `vicini`（`Voronoi.h:203-215`）。「是否走过」用**指针相等**判断（`Voronoi.h:209`），而这一判断本身又是 `O(|punti_passati|)` ⇒ 邻域查询整体 `O(N·|π|)`。
   - **贪心选择**：若 `vicini` 非空，选其中距 `arrivo` 最小者 `temp`（`Voronoi.h:217-227`）；**仅当** `‖temp − arrivo‖ < ‖tmp − arrivo‖` 才移动（`Voronoi.h:229-231`），否则 `goto alternativo`。移动后 push 进 `percorso` 与 `punti_passati`（`Voronoi.h:234-235`）。
   - **死胡同兜底 `alternativo`**（`Voronoi.h:237-281`）：打印 `"Non ci sono riuscito, ritento"`（「没成功，重试」）→ 清空 `percorso`/`punti_passati`（`Voronoi.h:240-241`）→ 在 `incroci` 中找「距 `arrivo` 最近且未用过」的点（`Voronoi.h:245-260`）→ 若找不到则打印 `"Impossibile trovare un percorso"` 并 **`exit(-1)`**（`Voronoi.h:262-265`）→ 否则递归两段 `partenza → incrocio` 与 `incrocio → arrivo` 并拼接返回（`Voronoi.h:275-280`）。
3. **终止条件**：`tmp` 坐标**精确等于** `arrivo` 坐标（`Voronoi.h:283`）。因为调用方传进来的 `arrivo` 是 `*minimo_arrivo`（堆上 Voronoi 点的拷贝），坐标能精确相等，所以能停；**若调用者传任意实数坐标的 `arrivo`，这个循环不会因坐标相等而退出**。

**关键参数**：`raggio_ricerca = 5·√2 ≈ 7.0711`（`Voronoi.h:10`）。

**复杂度**：单步邻域查询 `O(N + N·|π|)`，最坏步数 `O(N)` ⇒ 最坏 `O(N³)`；递归兜底每层都重新扫全程，最坏可指数级。本例 `N = 1283` 而实际步数只有 ~26（`percorso.txt` 去掉端点与重复点约 26 段）。**是否在所有场景都终止：未确认（仓库没有保存 stdout，也没有超时/深度保护）。**

**实测核对（点云连通性）**：我用 BFS 统计该点集在不同半径下的连通分量数：
- `r = √2`（8 邻域）→ **1 个连通分量，1283/1283 点全部连通**；
- `r = 2`、`r = √5`、`r = 3`、`r = 5√2` 也都是 1 个分量。
所以在 `raggio_ricerca = 5√2` 下每个点有 10 ~ 46 个邻居（中位数 21，无零度点）。**这说明取 5√2 并不是为了连通性（√2 就够了），而是给贪心留余地 —— 代价是半径 7 可以一步跨到相邻的另一条骨架分支上，而代码不检查 `tmp → temp` 这条边是否穿障。** 本例两侧 clearance ≥ 10 恰好没出事，但这是潜在穿障/钻窄缝隐患。

### 4.6 备选方案 — `Voronoi::getPercorsoVoronoi2`（已定义但**未启用**）

**位置**：`Voronoi.h:289-335`；唯一调用点被注释（`Voronoi.h:165`）。

**思路**：用少量 `incroci` 做分层路标 —— 找距 `partenza` 最近的交叉点 `incrocio_partenza`、距 `arrivo` 最近的 `incrocio_arrivo`（`Voronoi.h:297-309`）；再用 `|d(inc, partenza) − d(inc, arrivo)|` 最小来挑一个中继 `incrocio_obiettivo`（`Voronoi.h:313-321`）——这个量最小意味着该节点靠近两点的垂直平分线，是「中途换乘站」的启发式。

**但**：真正的三段拼接（`incrocio_partenza → incrocio_obiettivo → incrocio_arrivo`）被整段注释掉了（`Voronoi.h:325-329`），于是 `incrocio_obiettivo` 算了完全没用，`incrocio_partenza` 也只在挑选循环里自用；实际行为退化为「起点 → 离终点最近的交叉点 → 终点」的单中继方案（`Voronoi.h:323`、`:331-332`）。

**结论**：这是半成品，不要照抄代码；但「用少量高连接度节点（junction）搭分层路网」的方向是对的（见第 9 节第 5 条）。

---

## 5. 关键数据结构

**障碍物表示**（`Ostacolo.h:7-15`）：
- `vertici : std::vector<Punto*>`（`Ostacolo.h:9`）—— 用户传入的多边形顶点环。代码**不做封闭性/自交/方向校验**，靠 `vertici.at((i+1)%vertici.size())` 自动闭合（`Ostacolo.h:23`）。单顶点表示点障碍（`Ostacolo.h:62`）。
- `ingombro : std::vector<Punto*>`（`Ostacolo.h:10`）—— 边界上 1 单位间隔的格点集。**不是**填充区域、**不是**膨胀结果。
- `static double Distanza(Punto, Punto)`（`Ostacolo.h:14`、`:71-73`）—— 静态欧氏距离，被全局复用。

**图 / 顶点 / 边**：
- **没有图结构**。没有邻接表、没有邻接矩阵、没有顶点 id、没有边对象。
- 顶点 = `punti_voronoi : std::vector<Punto*>`（`Voronoi.h:21`）；唯一被显式标记的「图结构」是 `incroci : std::vector<Punto*>`（`Voronoi.h:22`），且它只是 `punti_voronoi` 的子集（4.3 实测确认）。
- 边 = **运行时临时算出来的近邻关系**：`‖p − q‖ ≤ raggio_ricerca`（`Voronoi.h:213`），用过即弃，不存储、不排序、不去重。这就是「隐式图」。
- 「是否访问过」用**指针相等**实现（`Voronoi.h:209`、`:251`），依赖所有顶点都是 `new` 出来的唯一堆对象，并且依赖「不会与坐标相同的另一个对象比较」这一隐式约定。

**坐标精度**：全程 `double`（`Punto.h:5`）；但输入顶点都是整数、`unit = 1`，所以实际是「整数坐标 + 实数距离」。**无定点、无量化误差控制、无坐标范围假设**（唯一的范围假设是哨兵 `10000`，见第 8 节第 11 条）。

**所有权**：值语义与裸指针混用 —— 内部路径 `std::vector<Punto*>`（借用），对外 `getPercorso` 返回 `std::vector<Punto>`（拷贝）；`getOstacoli()`/`getPuntiVoronoi()`/`getIncroci()` 返回内部容器的指针（alias，调用者能改内部状态）。除 `Voronoi.h:124` 外**全项目没有任何 `delete`/`free`**。

---

## 6. 工程细节

### 6.1 错误处理
只有两种：`exit(-1)`（`Voronoi.h:264`）和意大利语 `std::cout` 日志（`Voronoi.h:69`、`:71`、`:91`、`:134`、`:238`、`:263`）。没有异常、没有错误码、没有日志级别。**库层直接 `exit(-1)` 是设计硬伤**：这个「库」无法被嵌入更长的仿真循环（对你的项目尤其致命）。

### 6.2 边界条件
- **地图边界被当成一个额外障碍**：构造函数追加矩形 `(0,0) (W−1,0) (W−1,L−1) (0,L−1)`（`Voronoi.h:41-46`），于是墙不需要在碰撞检测里特判 —— 它就是第 `K+1` 个 site。
- **顶点方向**：README 要求「顺时针插入顶点」（`README.md:8`），但**在本实现里方向无关**。原因：代码只沿边采样、不做内外判定、不算法向。我算过有向面积（shoelace）：`programma.cpp` 的 5 个多边形全为**顺时针**（面积 `−100/−50/−50/−300/−350`），而地图边界矩形是**逆时针**（面积 `+39601`）——两者混用且结果正确，证明方向确实不影响。
- **格点扫描范围**：`tmpY ∈ [1, L−1)`、`tmpX ∈ [1, W−1)`（`Voronoi.h:88`、`:93`），留出最外一圈，配合边界矩形取 `W−1`/`L−1`。
- **机器人半径/障碍膨胀：完全没有做。** 全项目没有膨胀函数、没有 `inflate`/`offset`/Minkowski 相关代码，没有「机器人半径」参数。机器人被当作**质点**，安全性完全依赖「路径贴在 GVD 上（到最近两个障碍等距、局部间隙极大）」这一几何性质。特别注意：`Voronoi.h:13` 的 `distanza = 5*unit` 名字像安全距离，但注释写的是 `//filtro`，用法是 `Voronoi.h:171` 的抽稀阈值 —— **它不是安全间隙**，与碰撞安全无关。
- **单障碍场景**：`ostacoli.size() > 2` 守卫（`Voronoi.h:119`）避免 `distanze.at(2)` 越界；因为边界矩形总被追加，`K ≥ 2` 恒成立，1 个用户障碍时 `K = 2`，不会取第三个。

### 6.3 路径后处理
只有 4.4 里的两级抽稀（弦长阈值 `distanza = 5` + 等步长 `scarto = 1`）。**没有**可见性 shortcut 剪枝、**没有** Douglas–Peucker、**没有**样条/曲率平滑、**没有**终点连接段的碰撞校验。输出仍是格点锯齿折线（例如 `percorso.txt` 的 `(36,146) → (38,140) → (39,133)` 这类 1~2 格的纵横抖动）。

### 6.4 可视化（`plotta.m` 到底画了什么）
11 行，`load` 4 个 txt 后 5 次 `plot`（`plotta.m:5-10`）：

| 行 | 语句要点 | 图上表现 |
|---|---|---|
| `plotta.m:5` | `voronoi` 用 `'r.'` + `MarkerSize 10` + `MarkerEdgeColor 'g'` | **绿点** = GVD 骨架（`'r.'` 的红色被 `MarkerEdgeColor` 覆盖成绿） |
| `plotta.m:7` | `ostacoli` 用 `'r.'` | **红点** = 障碍边界栅格 |
| `plotta.m:8` | `percorso` 无线型参数，`LineWidth 3` | 在 `hold on` 下取 MATLAB 默认色序第 3 色 = **黄线** = 路径折线 |
| `plotta.m:9` | `percorso` 再画 `MarkerSize 15`、`MarkerEdgeColor 'y'` | **黄边大点** = 路点 |
| `plotta.m:10` | `incroci` 用 `MarkerSize 15`、`MarkerEdgeColor 'r'` | **红边大点** = 交叉点 |

这正是 `img.jpg`（842×707）那张图的内容。README 里给的是 GitHub 图片链接（`README.md:42`），本地 `img.jpg` 是否与链接内容一致：**未确认（离线无法比对）**。

### 6.5 进度与输出
建图时逐行打印 `Elaborazione: x%` 并用 `\r` 覆盖（`Voronoi.h:90-91`，百分比用 `(float)tmpY / (float)(lunghezza-unit) * 100`）。4 个结果文件用 `std::ofstream` **覆盖写**到当前工作目录（`programma.cpp:66-90`），没有目录/文件名参数化。

---

## 7. 测试与验证方法

**项目自带测试：没有。** 没有 test 目录、没有断言、没有 CI 配置、没有 golden 数据比对脚本 —— 全量目录列表可证。唯一的验证手段是「跑一遍 → 看 `plotta.m` 画出来的图对不对」。

**可复现性**：全流程无随机数、无时间依赖、无并发、无 `unordered_*` 容器，循环顺序固定（`Voronoi.h:88`/`:93` 的 y 外 x 内）⇒ **同输入必得同输出**，4 个 txt 就是天然的 golden 输出。

### 7.1 数据文件格式与语义
4 个文件都是**纯 ASCII、两列、空格分隔 `x y`、无表头、无注释、无记录分隔符**；值以浮点文本写出但是整数。

| 文件 | 行数 | 内容与语义 |
|---|---|---|
| `ostacoli.txt` | 1017 | 所有障碍 `ingombro` 点的**顺序拼接**。顺序 = `programma.cpp:10-55` 的插入顺序（quadrato, rettangolo, triangolo, rettangolo2, figura, punto）+ 构造函数最后追加的地图边界矩形（`Voronoi.h:46`）。**实测**：`40+30+20+70+60+1 = 221`（6 个障碍）+ `796`（边界）= `1017`，与行数吻合；第 41 行正好是第 2 个障碍的起点 `80 50`。**注意：文件缺少障碍分段，k 个障碍的点被拼在一起，读回来无法恢复归属**（只能靠重跑同样的顺序，或用边界起点 `(0,0)` 反推）。 |
| `voronoi.txt` | 1283 | `punti_voronoi` 坐标，**行主序**（y 非递减，实测），无重复，全为整数格点。 |
| `incroci.txt` | 18 | `incroci` 坐标；是 **9 个几何节点周围的点簇**（r ≤ 2 聚类，见 4.3）。 |
| `percorso.txt` | 29 | `getPercorso` 返回值：第 1 行 = 起点 `(20,140)`，末行 = 终点 `(160,70)`，中间 27 点全在 `voronoi.txt` 中，第 27/28 行重复 `(169,79)`（4.4 实测）。 |

生成代码：`programma.cpp:66-72`（ostacoli）、`:74-78`（voronoi）、`:80-84`（percorso）、`:86-90`（incroci）。

### 7.2 我做过的独立核查（可用 `/usr/bin/python3` 复现，全部只读）
1. 1283 个 Voronoi 点 **无一**落在 5 个多边形障碍内部或边界上，也不落在点障碍上（射线法 + 点在段上判定）。
2. 每个 Voronoi 点到最近障碍的 clearance：中位数 25.30、最小 10.00、最大 56.46。
3. 29 点路径的 28 条线段与所有障碍边**均不相交**，各线段中点也均不在障碍内部 ⇒ 本例路径无碰撞。
4. 点集在 `r = √2`（8 邻域）下就是**单一连通分量**（1283/1283）；`raggio_ricerca = 5√2` 下度为 10 ~ 46、中位数 21。
5. `incroci ⊂ punti_voronoi`（18/18 成立）；`percorso` 中间点 ⊂ `punti_voronoi`（27/27 成立）。
6. `ostacoli.txt`/`voronoi.txt` 的整数性与行数、行主序、重复性检查（awk 全量）。

这些**不是项目自带的测试**，是我为写本文现算的；它们能证明「这份 commit 的数据是自洽且路径无碰撞的」，但**不能替代重跑程序**（本次禁止构建）。

---

## 8. 局限与坑

### 8.1 作者自己承认/留痕的
1. README 只声称实现两件事，没有任何精度/最优性/完备性声明（`README.md:4-6`）。
2. `Voronoi.h:13-14` 两行都以 `//filtro`（滤波器）注释，作者知道它们是后处理参数而非算法参数。
3. 地图边界有两套互为替代的实现（启用 `Voronoi.h:40-46`，注释掉 `Voronoi.h:48-64`），注释写「如果不想连通地图角落就用这套」—— 作者自己在「要不要把墙角接进骨架」之间反复。
4. `Voronoi.h:233` 留着 `//tmp=temp; //oppure questa senza il controllo di sopra`（「或者用这行，不要上面的下降检查」）—— 明确承认「下降约束」是可选开关。
5. `Voronoi.h:165` 注释掉另一种路径算法、`Voronoi.h:325-329` 注释掉三段拼接：实验性代码。
6. `Voronoi.h:238` 打印「Non ci sono riuscito, ritento」（没成功，重试）—— 作者清楚贪心会失败。

### 8.2 我读出来/实测出来的
7. **只支持静态环境**：GVD 在构造函数里一次性建好（`Voronoi.h:70`），没有任何增量更新/局部重算接口；障碍动一步就得整体重建 `O(W·H·M)`。
8. **无动力学约束**：输出是几何折线，没有速度、朝向、最小转弯半径、加减速；格点锯齿 + 5 单位抽稀导致拐角很尖，真车跟踪必须再做平滑。
9. **最优性/完备性都没有**：贪心 + 严格下降判据会漏掉「必须先远离目标」的路；兜底完全依赖 `incroci` 非空（为空则 `exit(-1)`）。
10. **边没有碰撞校验**：邻域半径 ≈ 7 会跨骨架分支，而 `tmp → temp` 从不做障碍相交检测，最坏会切角或钻窄缝。本例 clearance ≥ 10 未暴露。
11. **哨兵 `10000` 当无穷大**（`Voronoi.h:138-139`、`:218`、`:244`、`:295-296`、`:311`）：地图尺度超过 10000 单位时最近点搜索会静默退化。
12. **库层 `exit(-1)`**（`Voronoi.h:264`），无法嵌入更长的仿真循环。
13. **内存/所有权**：全项目只有 `Voronoi.h:124` 一次 `delete`，大量 `new Punto` 泄漏（尤其 39204 次候选点分配）；`get*` 返回内部容器的指针，调用者可改内部状态。
14. **头文件无 include guard / `#pragma once`**：`Punto.h`、`Ostacolo.h`、`Voronoi.h` 都没有（grep 确认），且所有函数都定义在头文件里 ⇒ **被两个编译单元包含必然重复定义链接错误**。这是「不是库、只是示例」的直接证据。
15. **死声明**：`Voronoi.h:32` 声明了 `Punto getPuntoDistanzaMinima(Punto p)`，全仓库**没有定义**（grep 只命中声明行），也从未被调用。
16. **可移植性小坑**：用 `#include <math.h>` 而非 `<cmath>`（`Ostacolo.h:4`、`Voronoi.h:4`）；`exit` 未显式包含 `<cstdlib>`（`Voronoi.h:264`），当前靠 `<iostream>` 的传递包含能过。
17. **`std::sort(distanze.begin(), distanze.begin()+distanze.size())`**（`Voronoi.h:111`）等价于 `end()`，写法冗余；为取前 3 个值做全排序（`K` 很小所以无性能影响，但语义上好用 `nth_element`）。
18. **构造函数参数顺序陷阱**：形参是 `(larghezza, lunghezza, …)`（`Voronoi.h:28`，先「宽度」后「长度」），循环里 `x` 用 `larghezza`、`y` 用 `lunghezza`（`Voronoi.h:88`/`:93`）。调用点 `Voronoi(200,200,…)`（`programma.cpp:57`）恰好正方形所以看不出问题，非正方地图极易把宽长颠倒。
19. **退化边死循环**：相邻顶点重合时 `Ostacolo.h:48-57` 会算出 `NaN` 并让 `while` 永真、`ingombro` 无限增长（4.1）。是否在真实运行中触发过：**未确认（仓库数据不含退化边）**。
20. **`incroci` 是点簇不是节点**（4.3 实测），而兜底逻辑用指针相等区分 `incroci`（`Voronoi.h:251`）⇒ 一个几何节点会被当成 2~5 个路标，放大本就可能指数级的递归。
21. **没有构建系统、README 没有编译命令**（只有 API 片段，`README.md:8-39`）。

---

## 9. 对用户项目的可借鉴点

用户目标：Python + NumPy 的确定性仿真内核（SoA、固定时间步、可复现）→ CPU/FPGA 异构 → 真实二维小车。
下面把本项目的东西按「搬到哪个模块、怎么搬」拆开。

**(1) GVD/中轴骨架当作全局路线层（global roadmap）**
`CreaVoronoi`（`Voronoi.h:86`）的判据 `|d_(1) − d_(2)| ≤ ε` 是「最大间隙轨迹」的定义式，天然适合当静态/半静态的全局路线层。你的动态环境每帧都变，不能每帧重建骨架；正确用法是：**只在障碍物拓扑变化时重建骨架**（新增/消失/合并/分裂），其余时间在骨架上做局部重规划。判断「拓扑是否变了」的信号就是每个骨架点的 `site1/site2` 两个最近障碍 id 是否改变（本项目的 `min1/min2` 已经算出来了，只是没存 —— 见 (4)）。

**(2) 把「障碍物→边界栅格点集」升级成欧氏距离变换（EDT）**
本项目用 `Σ_k min_j dist(p, ingombro[k][j])` 暴力算距离场（`Voronoi.h:99-108`），本质就是 **per-obstacle 距离场**，只是用双层循环实现。搬到 NumPy 就是 `K` 张 `(H, W)` 的 `float32` 距离场（天然 SoA、天然向量化），`CreaVoronoi` 的 `O(W·H·M)` 会降到接近 `O(W·H)`。更好的是：
- 一次带 label 的 EDT 拿到「最近 site id + 距离」（`scipy.ndimage.distance_transform_edt(..., return_indices=True)`，或 Felzenszwalb 精确 EDT，纯 Python/NumPy 也能实现）；
- 再对「屏蔽掉最近 site」的掩码做第二次 EDT 拿 `d2/site2`；
两者组合就免费得到 4.2/4.3 需要的全部信息。
**这条路对你的 FPGA 目标是正解**：距离变换是规则数组、两遍扫描、可流水，比本项目「每格点遍历 1017 个采样点」友好几个数量级。

**(3) 无 sqrt 的整数等距判据（FPGA 友好）**
项目判据是 `|d1 − d2| ≤ √2`（`Voronoi.h:116`）。若距离改用以整数表示的平方距离 `D = d²`，该条件**等价于纯整数判据**：

```
若 D1 + D2 < 2        → 条件成立
否则  (D1 + D2 − 2)² ≤ 4 · D1 · D2     → 成立即为骨架点
```

推导（**这是我由项目判据自行推出的，不在仓库里**）：`|d1−d2| ≤ √2 ⟺ (d1−d2)² ≤ 2 ⟺ D1+D2−2 ≤ 2√(D1·D2)`，两侧非负时平方即得；`unit=1` 时 `t² = 2`。
好处：无 `sqrt`、无除法、无浮点，可直接综合成硬件比较器/乘法器；同理邻域查询（(6) 的 `raggio_ricerca`）也变成 `D ≤ 50` 的整数比较。代价：需要把浮点的 `dist_tolleranza` 语义换算到平方域（`√2` → `2`），一旦你想换容差就要重算阈值。

**(4) 把 clearance 存下来，别丢掉（本项目最大的浪费）**
`CreaVoronoi` 明明算了 `min1`（该点到最近障碍的距离），却只 push 了坐标（`Voronoi.h:113`/`:117` 只保存 `tmp_punto`）。对你的项目，这是改动量最小、收益最大的地方：顶点结构存成 SoA 并列数组

```
vx, vy            : float32 (H*W 稀疏后的 N 个骨架点)
d1, d2            : float32  # 到最近/次近障碍的距离
site1, site2      : int32    # 最近/次近障碍 id
deg               : uint8    # 是否 junction (原 incroci)
```

有了 `d1` 你能立刻拿到三件事：
- **隐式障碍膨胀**：用 `d1 ≥ r_robot` 过滤可通行边。这就是「按机器人半径膨胀障碍」的正确等价做法，而且**不需要做多边形 Minkowski 和 / 偏移**，对任意形状障碍都成立，还能直接吃动态障碍的更新；
- **安全代价**：`edge_cost = length + λ · max(0, r_safe − min d1 on edge)`，实现「离墙远一点」的偏好（这是本项目的骨架完全没有的）；
- **动态响应**：障碍动了只需重算受影响区域的 `d1/d2/site1/site2`，若某骨架点的 `site` 对发生变化就说明拓扑变了，触发重规划；否则只更新代价、不重建图。

**(5) 别抄贪心搜索，抄它的结构直觉：显式图 + 真 A\***
`getPercorsoVoronoi`（`Voronoi.h:193`）是贪心最优优先 + 严格下降 + 死胡同递归兜底，非最优、非完备、可能不终止，还带 `exit(-1)`。你要的是：骨架点当节点、近邻当边、**边权 = 欧氏长度（可加 clearance 惩罚）**、跑 **A\***（启发式 = 到目标的欧氏距离，admissible）。这正是本项目缺失的一层。
而 `getPercorsoVoronoi2`（`Voronoi.h:289`）想做的「用 incroci 分层」其实就是**分层路网（junction graph）**的雏形：建议做成两级 —— 上层把 9 个（本项目实测）junction 连成稀疏图跑 A\*，下层只在起始/目标附近做局部连接。这对你的实时预算非常友好，也是本项目唯一值得继承的搜索结构。

**(6) 地图边界当障碍 + 留一圈格点**
`Voronoi.h:41-46` 把墙变成第 `K+1` 个 site、`Voronoi.h:88`/`:93` 扫描范围留出最外一圈 —— 这个做法直接照搬即可：碰撞检测里不需要为墙写特判，距离场自然包含墙；扫描留边避免越界。

**(7) 文本 dump + golden 回归，直接适配你的「可复现」要求**
`programma.cpp:66-90` 的 4 个 txt 就是最简版的 golden 输出。你的内核是确定性的，等价做法是每步/每场景 dump 固定格式（或 `.npy`）+ SHA256。注意本项目 dump 的是无精度损失的整数/短小数；**你的 `float32` 必须固定格式**（如 `%.9e` 或干脆存二进制 + 哈希），否则文本 diff 会因末位抖动而假失败。另外要守住**定序纪律**：本项目点集是行主序、无重复、整数坐标（实测），所以可复现；你若用 `set`/`dict`、并行归约或非稳定排序就会破坏这一点（`Voronoi.h:111` 的 `std::sort` 恰好只取前 3 个值所以无影响，但这类「排序后取前 k」在浮点相等时依赖实现稳定性，换成 `np.partition` 同理，最好带 site id 做稳定 tie-break）。

**(8) 平滑：本项目只有弦长阈值抽稀，你必须补两步**
`Voronoi.h:171` 的抽稀只看「与上一个保留点的距离 ≥ 5」，不看中间点偏差，因此会切角。对你的车建议：先做**可见性 shortcut**（判断候选连线是否全程 `d1 ≥ r`，用距离场采样判据比多边形求交省事得多），再做**曲率受限平滑**（B 样条 / clothoid）以匹配最小转弯半径。

**(9) 「机器人半径如何膨胀」的最终结论**
本项目**不做任何膨胀**（6.2），它靠「路径在等距线上 ⇒ 两侧 clearance 局部极大」来近似安全。对你的小车（有真实外接半径 `r`）：
- 若障碍是轴对齐矩形/简单多边形：几何膨胀（矩形外扩 `r`、凸多边形半平面偏移）最直观；
- 若障碍形状任意且动态：**不要做多边形偏移**，改为在距离场上做阈值 `d1 ≥ r` —— 把「膨胀」变成「场上的一个比较」，FPGA 极友好，天然支持任意形状与动态更新；
- 顺带自动获得正确性：两个障碍间缝宽 `< 2r` 时，缝附近的骨架点 `d1 < r` 会被过滤掉 ⇒ 得到「走不过去」的正确判断。这正是本项目的缺失环节。

**(10) 不要移植的部分**
裸 `new`/泄漏、头文件内联无 guard、`exit(-1)`、`10000` 哨兵、指针相等当 visited 集合、`goto` 兜底、`incroci` 点簇当节点 —— 全部照抄会直接毁掉你的确定性和可维护性。**只移植 `CreaVoronoi` 的判据语义（4.2/4.3）+ 边界当 site 的做法（6.2）+ 分层 junction 的思路（4.6）**，其余用 NumPy 数据结构重写。

---

## 附录 A：意 → 英 → 中 对照表

| 意大利语原文 | 英文 | 中文 | 出现位置 |
|---|---|---|---|
| `Punto` | point | 点 | `Punto.h:3` |
| `Ostacolo` | obstacle | 障碍物 | `Ostacolo.h:7` |
| `ostacoli` | obstacles | 障碍物列表 | `Voronoi.h:20` |
| `vertici` | vertices | 多边形顶点 | `Ostacolo.h:9` |
| `ingombro` | encumbrance / footprint | （此处）边界栅格点集 | `Ostacolo.h:10` |
| `Distanza` | distance | 欧氏距离 | `Ostacolo.h:14` |
| `Voronoi` | Voronoi | 维诺图 | `Voronoi.h:16` |
| `punti_voronoi` | Voronoi points | Voronoi 点集/骨架点 | `Voronoi.h:21` |
| `incroci` | crossings / junctions | 交叉点、骨架分支节点 | `Voronoi.h:22` |
| `CreaVoronoi` | create Voronoi | 构造 Voronoi 图 | `Voronoi.h:23` |
| `getPercorso` | get path | 求路径 | `Voronoi.h:31` |
| `getPercorsoVoronoi` | get Voronoi path | 在骨架上走 | `Voronoi.h:24` |
| `getPuntoDistanzaMinima` | get min-distance point | 求最近点（**未实现**） | `Voronoi.h:32` |
| `partenza` | start / departure | 起点 | `Voronoi.h:31` |
| `arrivo` | arrival / goal | 终点 | `Voronoi.h:31` |
| `percorso` | path | 路径 | `Voronoi.h:136` |
| `perc_voro` | Voronoi path | 骨架段路径 | `Voronoi.h:164` |
| `mappa` | map | 地图 | `README.md:24` |
| `larghezza` / `lunghezza` | width / length | 宽（x）/ 长（y） | `Voronoi.h:18-19` |
| `unit` | unit | 格点步长/长度单位 | `Voronoi.h:9` |
| `raggio_ricerca` | search radius | 邻域搜索半径 | `Voronoi.h:10` |
| `dist_tolleranza` | distance tolerance | 等距容差 | `Voronoi.h:11` |
| `distanza` | distance | 抽稀阈值（非安全距离） | `Voronoi.h:13` |
| `scarto` | discard / stride | 抽稀步长 | `Voronoi.h:14` |
| `ambiente` | environment | 环境边界矩形 | `Voronoi.h:41` |
| `vicini` | neighbours | 邻居 | `Voronoi.h:201` |
| `punti_passati` | passed points | 已访问点 | `Voronoi.h:195` |
| `minimo` / `minimo_arrivo` | nearest / nearest-to-goal | 距起点最近 / 距终点最近 | `Voronoi.h:140-141` |
| `incrocio_vicino` | nearest junction | 最近的交叉点 | `Voronoi.h:242` |
| `incroci_s` | chosen junctions | 已选交叉点 | `Voronoi.h:193` |
| `retta verticale/orizzontale/obbliqua` | vertical/horizontal/oblique line | 垂直/水平/斜线 | `Ostacolo.h:38/43/48` |
| `figura`,`quadrato`,`rettangolo`,`triangolo` | figure, square, rectangle, triangle | 图形/方形/矩形/三角形 | `programma.cpp:10-43` |
| `Sto cercando i punti di voronoi` | I'm searching Voronoi points | 正在求 Voronoi 点 | `Voronoi.h:69` |
| `Mappa di voronoi creata` | Voronoi map created | Voronoi 图已建好 | `Voronoi.h:71` |
| `Elaborazione` | processing | 处理进度 | `Voronoi.h:91` |
| `Calcolo il percorso` | computing the path | 正在算路径 | `Voronoi.h:134` |
| `Non ci sono riuscito, ritento` | I didn't manage, retrying | 没成功，重试 | `Voronoi.h:238` |
| `Impossibile trovare un percorso` | impossible to find a path | 找不到路径 | `Voronoi.h:263` |

---

## 附录 B：本文做过的核查清单（全部只读）

1. 目录/文件全量列举 + 行数统计（`ls -laR`、`wc -l`）→ 第 3 节。
2. `git config` / `packed-refs` / `logs/HEAD` / `git log --oneline --all` / `git ls-tree` → 第 1 节出处与提交数。
3. `awk` 全量扫描 `voronoi.txt`：整数性、y 非递减（行主序）、重复点 → 4.2。
4. `awk`/`sort` 统计各文件行数与取值范围 → 7.1。
5. Python（`/usr/bin/python3`，仅标准库，无第三方依赖）：
   - 射线法 + 点在段上判定：1283 个 Voronoi 点是否落在 5 个多边形/点障碍内 → 0 命中；
   - 点到线段的精确距离：每个 Voronoi 点的 clearance 分布；
   - 线段相交判定：`percorso.txt` 的 28 条线段与所有障碍边/边界边 → 0 相交，中点不在障碍内；
   - BFS 连通分量：点集在 `r = √2, 2, √5, 3, 5√2` 下的连通性、度分布；
   - r ≤ 2 聚类：`incroci.txt` 的 18 点 → 9 个几何节点；
   - shoelace 面积：`programma.cpp` 5 个多边形均为顺时针、地图边界为逆时针 → 6.2；
   - 集合包含：`incroci ⊂ voronoi`、`percorso 中间点 ⊂ voronoi`。
6. `grep` 全仓库：include guard、`getPuntoDistanzaMinima` 定义 → 4.6/第 8 节。
7. 逐行阅读 `README.md`、`LICENSE`、`Punto.h`、`Ostacolo.h`、`Voronoi.h`、`programma.cpp`、`plotta.m`（全文），并查看 `img.jpg` 确认可视化配色。

## 附录 C：未确认项（明确未核实，不做猜测）

1. **能否编译通过、有无警告**：本次禁止构建，我只读代码（第 2 节末尾列出的两处可移植性隐患仅是从代码看出）。
2. **运行时是否触发过 `alternativo` 兜底 / 是否打印过 `"Non ci sono riuscito, ritento"`**：仓库未保存 stdout/stderr，无从判断。
3. **本地 `img.jpg` 与 `README.md:42` 链接的 GitHub 图是否同一张**：离线无法比对；只能确认本地文件为 842×707 JPEG。
4. **`ostacoli.txt` 等 4 个 txt 是否与当前代码逐位一致**：我做了行数推算（1017 = 221 + 796 + 第 41 行对齐）、整数性、行主序、集合包含等一致性检查且全部吻合，但**没有重跑程序复算**（禁止构建），因此严格说只是「与代码逻辑高度自洽」。
5. **贪心搜索是否在所有输入下终止、递归兜底的收敛性**：只有静态代码分析（4.5），没有运行验证。
6. **历史提交里是否有过 Makefile 或不同版本的算法**：只看了 `git log --oneline --all` 与 HEAD 的文件列表，没有逐个 diff 历史内容。
7. **`Voronoi.h` 里注释掉的另一套边界处理（`Voronoi.h:48-64`）效果如何**：未启用、未验证。
