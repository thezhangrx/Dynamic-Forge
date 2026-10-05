# TostEngine Cinder BulletHell —— 技术总结（只读代码审查）

> 审查对象：`/home/zhang/Bullet_Platform/Outside/Bullet/TostEngine-cinder-bullethell-main`
> 审查方式：仅阅读源码/文档 + 目录与二进制元数据统计，**未修改任何已有文件、未构建、未联网**。
> 本文所有结论均标注来源（`文件:行`）。凡未能从代码确认的，明确写“未确认”。
> 阅读提示：全仓库 594 MB 中，**项目自己写的代码只有 4 个文件、27 908 字节**（`README.md` / `CMakeLists.txt` / `.gitignore` / `src/BulletHell.cpp`），其余 99.99% 是 vendored 的 Cinder 框架与 `build/` 产物。

---

## 1. 一句话定位

- **是什么**：一个用 **Cinder** 框架写的 2D 弹幕（Touhou 风格）游戏小引擎/演示程序。README 原话：*"A 2D Bullet-Hell (Touhou-style) game engine built with Cinder framework."*（`README.md:3`）。
- **作者**：**未确认**。README 没有作者署名，`src/BulletHell.cpp` 没有任何版权头（文件直接从 `#include` 开始，`src/BulletHell.cpp:1`），目录内也没有 `.git`（`ls -a` 只有 `.gitignore/CMakeLists.txt/README.md/build/cinder/src`），无法从本地副本判断作者或上游提交。
- **许可证**：README 末尾写着 **MIT License**（`README.md:71-73`），但**仓库内没有任何 `LICENSE`/`COPYING` 文件**——全目录 `find -iname "*license*"` 只命中 `cinder/COPYING`，那是 Cinder 自己的 BSD 2-Clause 许可（`cinder/COPYING:1-3`）。所以“MIT”目前只是 README 的单方面声明，缺少授权文本（上游单独仓库的 `LICENSE` 未随克隆一起带下来）。工作区自己的 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:37` 记的是同一事实并进一步标成 `[LICENSE_REVIEW_REQUIRED]`（“只有一行 MIT，没有正文”）。
- **体量与形态**：单个 615 行的 `.cpp`（`wc -l src/BulletHell.cpp` = 615），没有拆分头文件，没有资源文件，没有测试。它更像“一个可运行的参考 demo”，而不是可复用引擎。
- **运行形态**：窗口 640×480、Windows 优先（`WIN32` 可执行 + MSVC Debug 运行时，`CMakeLists.txt:19,22`），但本副本里已经有一份 **Linux/GCC Debug 构建产物**可直接印证跨平台可编译（见第 7 节）。

---

## 2. 技术栈与工具链

### 2.1 语言与标准
- **C++20**，强制：`set(CMAKE_CXX_STANDARD 20)` + `CMAKE_CXX_STANDARD_REQUIRED ON`（`CMakeLists.txt:3-4`）。
- **CMake ≥ 3.16**（`CMakeLists.txt:1`）。
- 使用了 POSIX 的 `M_PI`（`src/BulletHell.cpp:238,282`）——在 GCC/Clang 下可用，在 MSVC 上需要 `_USE_MATH_DEFINES` 才能编译（本项目代码未做该处理）。

### 2.2 构建系统（`CMakeLists.txt` 全文 25 行）
| 行 | 内容 |
|---|---|
| 7 | `project(BulletHell)` |
| 10-13 | `CINDER_PATH = ${CMAKE_CURRENT_SOURCE_DIR}/cinder`；若 `cinder/CMakeLists.txt` 不存在则 `FATAL_ERROR`（中文错误信息） |
| 16 | `add_subdirectory("${CINDER_PATH}" cinder_build)` —— **把 Cinder 当子项目一起编译**，没有 `find_package(Cinder)` |
| 19 | `add_executable(BulletHell WIN32 src/BulletHell.cpp)` —— 单文件目标；`WIN32` 在 Unix 生成器下被忽略 |
| 21-22 | `MSVC_RUNTIME_LIBRARY "MultiThreadedDebug"`（注释明确“仅 Windows/MSVC 有效”） |
| 25 | `target_link_libraries(BulletHell PRIVATE cinder)` |

**没有** `find_package(OpenGL/GLM/Boost/...)`：所有第三方库都是由 Cinder 的 target 传递下来的。

### 2.3 依赖（从实际链接行读出，`build/CMakeFiles/BulletHell.dir/link.txt`）
- 静态库：`cinder/lib/linux/x86_64/ogl/Debug/libcinder.a`（178 MB）。
- 图形/窗口：`libOpenGL`、`libGLX`、`libGLU`、`libX11`、`libXext`、`-lXcursor -lXinerama -lXrandr -lXi`（Cinder 在 Linux 上默认走 **GLFW + GLX**，见 `cinder/include/cinder/app/App.h:34-45`）。
- Cinder 顺带拖进来的：`libz`、`libcurl`、`libfontconfig`、`libpulse`、`libmpg123`、`libsndfile`、`libgobject-2.0/libglib-2.0`、`libgstreamer-1.0/libgstbase/libgstapp/libgstvideo/libgstgl`、`-ldl -lpthread`。
- **GLM**：由 Cinder 内置（`cinder/include/glm`，版本 **0.9.9**，`cinder/include/glm/detail/setup.hpp:6-8`）。项目里用的 `vec2/vec4` 就是 `glm::vec2/glm::vec4`（`cinder/include/cinder/Vector.h:38-48`），不是自己的数学库。
- **Boost**：项目本身不用；只在 Cinder 的 Android 预编译产物里出现（`cinder/lib/android/**/libboost_filesystem.a`），本 Linux 构建的链接行里没有 Boost。
- **Box2D/Cairo/Clipper/OSC/TUIO 等 Cinder blocks 存在**（`cinder/blocks/` 共 8 个目录）但本项目**一个都没 include**（`src/BulletHell.cpp:1-10` 只有 `cinder/app/App.h`、`cinder/app/RendererGl.h`、`cinder/gl/gl.h`）。
- 最终可执行文件 ELF 的 `NEEDED` 只有 6 项：`libcurl-gnutls.so.4`、`libfontconfig.so.1`、`libstdc++.so.6`、`libm.so.6`、`libgcc_s.so.1`、`libc.so.6`（`readelf -d build/BulletHell`）。链接行里出现的 GL/X11 **没有**出现在 `NEEDED` 里（推测由 GLFW 运行时 `dlopen` 或 `--as-needed` 丢弃，具体机制**未确认**）。

### 2.4 运行方式
- README 说：打开 `build/BulletHell.sln`，用 Debug|x64 构建，或使用 CMake（`README.md:51`）。**本副本里没有 `.sln`**（`ls build/`：只有 `BulletHell` 可执行、`CMakeCache.txt`、`Makefile`、`CMakeFiles/`、`cinder_build/`、`cmake_install.cmake`）→ README 的构建说明已过时/平台错配。
- 本副本实际状态：Linux + Unix Makefiles + Debug 的 CMake 构建树，产物 `build/BulletHell`（37 260 248 B，ELF 64-bit PIE，`with debug_info, not stripped`，`file build/BulletHell`）。
- 操作：方向键/WASD 移动、Shift 聚焦、Z 射击、H 切换判定点显示、P 暂停、1-6 切弹幕、Esc 退出（`README.md:20-30`，实现见 `src/BulletHell.cpp:570-597`）。**需要 X11/OpenGL 显示环境**（没有 headless 路径）。

---

## 3. 仓库体积构成分析（`du`/`find` 实测）

```
594M  .                                    ← 整个参考项目
384M  cinder/                              ← vendored 第三方框架（64.6%）
210M  build/                               ← 构建产物（35.4%）
 28K  src/                                 ← 项目自己的代码（1 个文件）
```

### 3.1 逐目录实测
| 路径 | 大小 | 文件数 | 说明 |
|---|---|---|---|
| `cinder/lib` | 181 M | — | **单个文件 `cinder/lib/linux/x86_64/ogl/Debug/libcinder.a` = 177 984 696 B ≈ 178 MB**，全仓库最大文件 |
| `cinder/samples` | 70 M | — | Cinder 官方示例 |
| `cinder/test` | 34 M | — | Cinder 自己的测试 |
| `cinder/docs` | 32 M | — | 其中 `docs/htmlsrc` 就 28 M |
| `cinder/blocks` | 26 M | — | Box2D/Cairo/Clipper/OSC/TUIO/… |
| `cinder/src` | 23 M | 243 个 `.cpp` | Cinder 实现 |
| `cinder/include` | 19 M | **450 `.h` + 827 `.hpp` = 1277 个头文件** | 这就是“上千个头文件”的来源 |
| `build/cinder_build` | 173 M | 282 个 `.o` | Cinder 被整体编译产生的中间目标文件 |
| `build/BulletHell` | 36 M | 1 | 未 strip 的 Debug 可执行文件 |
| `src` | 28 K | 1 | `BulletHell.cpp` |

- 全仓库文件总数 **7398**：`cinder/` 6786（91.7%）、`build/` 608（8.2%）、`src/` **1**。
- 项目自有 4 个文件合计 **27 908 字节**（`BulletHell.cpp` 24 408 + `README.md` 2226 + `CMakeLists.txt` 753 + `.gitignore` 521）= 全仓库的 **约 0.0047%**。

### 3.2 `build/` 里是什么
- `CMakeCache.txt`（128 K）：`CMAKE_GENERATOR=Unix Makefiles`、`CMAKE_BUILD_TYPE=Debug`、`CINDER_TARGET=linux`、`CINDER_TARGET_GL=ogl`、编译器 `/usr/bin/c++`（`gcc-ar-13/gcc-ranlib-13`，即 **GCC 13**）。
- `cinder_build/CMakeFiles`（173 M / 282 `.o`）：Cinder 全部源码的目标文件。
- `BulletHell`（36 M）：链接结果；`libcinder.a` 的 mtime（2026-09-19 11:21:03）紧邻可执行文件 mtime（11:21:08），说明 **178 MB 的静态库就是这次构建生成的**（Cinder 的 CMake 把归档输出到 `cinder/lib/...`，所以 `cinder/lib` 也属于“产物”而非纯源码）。
- **关键旁证**：`CMakeCache.txt` 里 `CMAKE_HOME_DIRECTORY = /home/zhang/Bullet_Platform/TostEngine-cinder-bullethell-main`，而当前路径是 `.../Outside/Bullet/TostEngine-cinder-bullethell-main` → **构建发生在项目被移动到 `Outside/Bullet/` 之前**，缓存已失效，直接 `make` 会指向旧路径。

### 3.3 哪些该入库 / 哪些该 ignore（实测 `git check-ignore` / `git add -n`）

> ⚠️ 时效说明：审查期间父仓库的 `.gitignore` 被修改过一次（本节的结论以**写作时的最终状态**为准，已用 `git check-ignore` 与 `git add -n` 复核）。早期版本曾用 `Outside/*/*` + `!Outside/*/*/tech.md` 试图“只跟踪分析笔记”，该写法已废弃。

这个目录**本身不是 git 仓库**（无 `.git`；`git rev-parse --show-toplevel` 返回父仓库 `/home/zhang/Bullet_Platform`）。

| 路径 | 匹配规则（实测） | 结果 |
|---|---|---|
| `Outside/Bullet/TostEngine-cinder-bullethell-main/` 整棵树 | 父仓库 `.gitignore:22`（`Outside/`） | **忽略** |
| `cinder/include/cinder/Cinder.h` | 同上（父仓库整树忽略；本项目 `.gitignore:49` 也声明 `cinder/`） | 忽略 |
| `build/BulletHell` | 同上（本项目 `.gitignore:53` 也声明 `build/`） | 忽略 |
| `src/BulletHell.cpp` | 同上 | 忽略 |
| `tech.md`（本文） | 同上 | 忽略（`git add -n tech.md` 被拒，提示 "paths are ignored … Use -f"） |

父仓库 `.gitignore:15-20` 记录了这一策略的理由：**这些克隆自身是 git 仓库，git 无法只跟踪其中的单个文件**（强制 add 会变成 gitlink/静默无效），所以改为把分析笔记作为**可提交镜像**放在 `docs/reference/<项目>.md`，并在项目目录里放一个指向镜像的相对符号链接 `tech.md`（见 `docs/reference/README.md:6-9`；已落地示例：`Outside/Algo/gpmp2/tech.md -> ../../../docs/reference/gpmp2.md`，`docs/reference/README.md:20` 也登记了 `./TostEngine.md`）。

因此：
- **该 ignore 的**：本目录的 594 MB 全部（`cinder/`、`build/`、`src/`、`README.md`……），上游作者自己的 `.gitignore:49,53`（`cinder/`、`lib/`、`build/`）也持同样立场。
- **该入库的**：本笔记的镜像（约定路径 `docs/reference/TostEngine.md`）——**写作时该镜像尚不存在**；本文当前是以**普通文件**形式落在被要求的位置 `Outside/Bullet/TostEngine-cinder-bullethell-main/tech.md`（不是符号链接）。是否需要按上述约定“镜像 + 符号链接”由上层仓库维护者决定。

---

## 4. 项目自有代码拆解

自有代码 = `src/BulletHell.cpp`（615 行，1 个文件）。下面按文件中的分节顺序拆到“结构体 / 函数 / 类”级别。**没有自有头文件**——所有类型都定义在这一个 `.cpp` 里。

### 4.1 类型定义（`src/BulletHell.cpp:27-50`）
| 类型 | 行 | 字段 | 职责 |
|---|---|---|---|
| `struct Transform` | 30 | `vec2 position`、`float rotation` | 位置/朝向。**`rotation` 全文只在此处出现一次，从未被读或写（死字段）** |
| `struct Velocity` | 31 | `vec2 linear`、`float angular` | 线速度/角速度。**`angular` 同样是死字段**（见 5.3） |
| `struct Bullet` | 32-38 | `radius`、`lifetime=10`、`age=0`、`vec4 color`、`uint8_t flags` | 弹的属性；`flags` 即阵营（`0x01` 玩家弹、`0x02` 敌弹） |
| `struct Collider` | 39 | `radius=4`、`uint8_t mask=0` | 碰撞体；**`mask` 只在生成时被赋值，从未被读取（死字段）** |
| `struct PlayerData` | 40-50 | `hitboxRadius`、`lives`、`score`、`focusing`、`invisible`、`fireTimer`、`iFrames=60`、`iFrameTimer`、`grazeTimer` | 玩家专属状态，**和弹的属性塞在同一个实体里** |
| `struct Entity` | 55-62 | `bool active` + 上面全部 5 个组件 | **AoS + “万能实体”**：每颗子弹都白白携带 `PlayerData`（约 32 B），每个实体都带 `Transform/Velocity/Bullet/Collider` |

这是 README “ECS-inspired entity system”（`README.md:45`）的实际形态：**它不是 ECS**（没有按组件分列存储），而是“一个结构体装下所有组件 + `active` 标志”的对象池。

### 4.2 实体管理（`src/BulletHell.cpp:52-139`）
- `Entity entities[MAX_ENTITIES]`（`65`），`MAX_ENTITIES = 8192`（`64`）——**全局定长数组、静态存储期、零堆分配**；`int playerEntity = -1`（`66`）保存玩家槽位。
- `int createEntity()`（`118-130`）：从 0 开始**线性 first-fit 扫描**找第一个 `!active` 槽，重置 `transform/velocity/bullet/collider`（`122-125`），返回下标；满了返回 `-1`。
  - ⚠️ **不重置 `player` 字段**（`122-126` 漏了 `entities[i].player = PlayerData();`）→ 槽位复用时残留旧玩家的 `lives/score/iFrameTimer`。当前只有槽 0 当玩家，属潜伏 bug。
- `void destroyEntity(int id)`（`132-134`）：**只置 `active = false`**。没有自由链表、没有 generation/版本号 → 槽位会被立刻复用（first-fit 从 0 找），典型 ABA 隐患；`id` 就是数组下标，不是稳定 ID。
- `bool isInBounds(pos, radius)`（`136-139`）：越界剔除（允许半径外扩）。
- `int spawnBullet(pos, vel, radius, color, flags)`（`170-182`）：`createEntity` + 填字段（`collider.radius = radius`，`collider.mask = flags`，`179`）；**调用方全部忽略返回值**，池满时静默不生成。
- `int spawnPlayerBullet(pos)`（`184-186`）：速度 `(0,-800)`、半径 3、青色、`flags = 0x01`。

### 4.3 空间网格（`src/BulletHell.cpp:68-113`）
- 常量：`CELL_SIZE = 32`（`71`）；`GRID_COLS = ceil(640/32) = 20`，`GRID_ROWS = ceil(480/32) = 15`（`72-73`）→ **固定 300 个格子**；`vector<int> grid[300]`（`74`）。
- `gridClear()`（`76-78`）：每帧清空 300 个桶（保容量，不释放内存）。
- `gridInsert(id, pos, radius)`（`80-91`）：按实体 **AABB**（`pos ± radius`）把 id 压入覆盖到的**每一个**格子——保守正确，半径大于格子时会写入多格。
- `template gridQuery(pos, radius, func)`（`94-113`）：遍历覆盖的格子，用一个**局部 `vector<int> seen` 做线性去重**（`100-112`，O(k²)）后回调。
  - ⚠️ 每次调用都堆分配一个 `vector`，且去重是二次复杂度；该函数在一次 update 中会被调用 **2 + 玩家弹数量** 次。

### 4.4 输入（`src/BulletHell.cpp:141-150, 570-607`）
- `uint8_t inputState` 位掩码 + 6 个位常量 `IN_LEFT/RIGHT/UP/DOWN/FOCUS/SHOOT`（`144-150`）。
- `keyDown`（`570-597`）置位并处理一次性按键（H/P/1-6/Esc），`keyUp`（`599-607`）清位。没有 key-repeat 特殊处理，没有手柄。

### 4.5 全局游戏状态（`src/BulletHell.cpp:152-165`）
`gameOver`、`paused`、`showHitbox=true`、`godMode=false`、`currentPattern=1`、`patternTimer`、`difficulty=3.0f`、`hitEffects`/`grazeEffects`（`vector<pair<vec2,float>>`，第二项是“寿命”）。
- `difficulty` **没有任何代码路径修改它**（仅 `161` 定义 + `206,223,235,236,251,253` 使用）→ README 说的“difficulty 倍率”实际是硬编码常量 3.0。
- `godMode` 也**没有绑定任何按键**（`158,397`）→ 只能改代码才能开启。

### 4.6 类 `BulletHell : public App`（`src/BulletHell.cpp:312-608`）
| 成员 | 行 | 职责 |
|---|---|---|
| `setup()` | 316-322 | 记录 `mLastTime`；创建玩家实体（槽 0）、位置 `(320,400)`、3 条命 |
| `update()` | 324-466 | 全部逻辑：读输入 → 积分 → 网格重建 → 碰撞 → 弹幕 → 特效 |
| `draw()` | 468-568 | 全部渲染：弹、玩家、特效、UI |
| `keyDown()/keyUp()` | 570-607 | 位掩码输入 |
| `mLastTime` | 314 | `double`，用于算 dt |

`prepareSettings()`（`610-613`）设窗口 640×480、标题 `"BulletHell Engine"`；`CINDER_APP(BulletHell, RendererGl, prepareSettings)`（`615`）是 Cinder 的入口宏。

### 4.7 自由函数
- `updatePatterns(float dt)`（`191-307`）：弹幕调度，见 5.7。

---

## 5. 引擎核心技术

### 5.1 主循环：**变步长 + dt 钳制，不是固定步长**

```cpp
void update() override {
    if (paused || gameOver) return;            // 324-325
    double currentTime = getElapsedSeconds();  // 327
    float dt = (float)(currentTime - mLastTime);
    mLastTime = currentTime;
    if (dt > 0.25f) dt = 0.25f;                // 330  "clamp to prevent spiral of death"
    ...
    player.transform.position += player.velocity.linear * dt;   // 362
    e.transform.position += e.velocity.linear * dt;             // 381
```

- **没有累加器（accumulator）、没有固定步长、没有追帧循环、没有插值/渲染插值、没有存上一帧状态**。
- 物理直接吃“上一帧到这一帧的墙钟时间”，只做了 0.25 s 上限钳制（`330`）。
- 逻辑在 `update()`、渲染在 `draw()`，**名义上分离**；但 Cinder 每帧各调一次，帧率即仿真步长，所以二者并未真正解耦。
- ⚠️ 与 README 自述矛盾：README 写 “Fixed timestep game loop”（`README.md:46`）、“Delta-time based pattern timing”（`README.md:47`）——后者属实，前者**与代码不符**。工作区自己的 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:48` 也沿用了 “fixed timestep” 的说法，需要更正。
- 后果：**不可复现**。同一段输入在不同帧率下结果不同（尤其弹幕生成见 5.7）。

### 5.2 实体/子弹系统：定长 AoS 数组 + `active` 标志

- 存储：`Entity entities[8192]`（`65`），**AoS、定长、静态**；不是 `std::vector`，没有扩容、没有对象池 free-list、没有 SoA。
- 生成：`createEntity()` 线性 first-fit（`118-130`），最坏 O(8192)；池满静默失败（`129`）。
- 回收：`destroyEntity()` 只清 `active`（`133`）；**槽位立即被下一次 `createEntity` 复用**，没有 generation 计数，因此“下标即 ID”不稳定。
- ID 管理：无。`playerEntity` 是唯一被记住的下标（`66`）。
- 遍历：每帧 **3 次**全量扫 8192 槽（更新/剔除 `378-389`、玩家弹筛选 `398-400`、渲染 `472-473`），绝大多数槽是空的也在被 `if (!entities[i].active) continue` 逐条判定。
- 回收延迟：越界/超龄的弹先收集到 `vector<int> toDestroy`（`376`），循环后再统一销毁（`391`）。

### 5.3 运动模型：纯欧拉直线，**角速度/曲线弹根本没有实现**

```cpp
struct Velocity { vec2 linear = vec2(0); float angular = 0; };   // 31
...
e.transform.position += e.velocity.linear * dt;                   // 381（玩家同理 362）
```

- 积分方式：**前向欧拉**；由于结构体里**不存在加速度/力字段**，步内速度恒定，所以直线运动是精确的（不是近似误差源）。
- **没有任何角度积分**：`Velocity::angular`（`31`）和 `Transform::rotation`（`30`）在全文的引用只有这两行定义处（`grep -n "angular\|rotation" src/BulletHell.cpp` 仅命中 30、31 行），既没被赋值也没被读。
- 因此：**这个引擎没有曲线弹**。README 里的 “Spiral”（`README.md:35`）在代码里是**生成时把方向算好、之后直线飞**的径向弹：
  ```cpp
  angle += dt * 3.0f;                                    // 222
  vec2 vel = vec2(cos(angle)*speed, sin(angle)*speed);   // 224
  spawnBullet(vec2(320,50), vel, ...);                   // 225
  spawnBullet(vec2(320,50), -vel, ...);                  // 226
  ```
- ⚠️ 需要更正的一处事实：工作区 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:45` 把本项目的角速度记为 “`Velocity.angular` 驱动旋转”。**本仓库代码不支持这个说法**——该字段是死字段；用户的“精确圆弧积分”实现（`bullet_sim/physics/motion.py`）实际上**领先于**这个参考项目，而不是从它照搬。
- 另一个帧率耦合点：`angle += dt * 3.0f` 写在 `while (timer >= 0.02f)` 循环体内（`220-222`）——每发弹推进的角度用的是“本帧 dt”而非“发射间隔 0.02 s”，所以螺旋的螺距随帧率/卡顿变化。

### 5.4 碰撞检测：圆-圆 + 固定 32 px 均匀网格宽相

- **碰撞体形状：只有圆**。`Collider{radius}`（`39`）+ 玩家判定半径 3（`21`）。没有 AABB / OBB / 多边形 / 静态障碍物——**这个引擎里完全没有“障碍物”概念**（对本用户项目是重要短板）。
- **宽相：均匀空间网格**（非四叉树/扫描线）。固定 `CELL_SIZE=32`、20×15=300 格（`71-74`）；每帧 `gridClear` → 积分后 `gridInsert`（`375-389`）。玩家不插入网格。
- **窄相：** 两次距离平方/`length` 比较（`403-404`、`417-418`、`445`），复杂度 O(1)/对。注意用的是 `length()`（含开方）而非平方比较。
- **每帧检测次数**（`update()` 内）：
  1. 建格遍历 8192 槽（`378-389`）；
  2. **玩家弹 vs 敌弹**：先扫 8192 槽找 `flags & 0x01`（`398-400`），对每颗玩家弹做一次 `gridQuery`（半径 = 玩家弹半径×2，`401`）；`godMode` 为真时整段跳过（`397`）。命中即销毁敌弹、+10 分（`405-407`）；
  3. **玩家 vs 敌弹**：1 次 `gridQuery`（半径 = `hitboxRadius` = 3，`415`）；命中 → 掉命 + `iFrameTimer = iFrames`（`420`）→ **立刻销毁全场所有敌弹**（`421-426`，逐颗 push 特效）→ `lives<0` 则 gameOver，否则 `invisible=true`（`428-432`）；
  4. **擦弹（graze）**：1 次 `gridQuery`（半径 `GRAZE_RADIUS=12`，`441`），有 0.1 s 冷却（`443,447`），+1 分（`448`）。
- 复杂度：每帧 ≈ O(N)（N=8192 槽）+ ΣO(候选) + O(k²) 去重；README 宣称的 “O(1) collision detection”（`README.md:17`）只在“单次查询命中格很少”的意义上成立，**作为整帧复杂度不准确**，且没有基准数据支撑。
- 正确性备注：因为 `gridInsert` 把实体写进其 AABB 覆盖的所有格子，而 `gridQuery` 也按查询圆的 AABB 取格，所以“两圆相交 ⇒ 存在共享格子”成立，查询是保守完备的（这一点做得对）。销毁后网格不清理，但回调里都先查 `entities[id].active`，所以同帧内不会重复命中。

### 5.5 输入与手感

- 移动：**每帧直接把速度赋成常量**，完全没有加速度/惯性/摩擦力：
  ```cpp
  if (inputState & IN_LEFT)  player.velocity.linear.x = -PLAYER_SPEED;   // 334
  ...
  else player.velocity.linear = vec2(0);                                 // 350-352
  ```
  `PLAYER_SPEED = 250`（`22`）→ 瞬时启停（README 的 “Smooth Player Movement”，`README.md:10`，指的是手感响应快，不是有惯性）。
- 斜向归一化：对角按 ×0.707（`346-349`），与速度常量配合使斜向速率一致。
- **聚焦（slow mode）**：按住 Shift → 速度 ×0.5 并显示判定点（`354-359`、`492-497`）；`focusing` 标志同时用于绘制。有效速度：250 → 125（斜向为 250×0.707×0.5 ≈ 88.4）。
- 射击：`FIRE_RATE = 0.05 s`（`23`），双发 ±8 px（`369-370`）。
  - ⚠️ `fireTimer` **只在按住 Z 时才递减**（`365-368`）：松开期间倒计时冻结，因此点按射击的有效射速低于标称值（不是“更快”，但逻辑不自洽）。
- 与帧率的关系：位移用 `dt` 积分（`362`），**位置推进是帧率无关的**；但输入是每帧采样、且无固定步长 → 手感随帧率变化，且不可复现。
- 玩家位置钳制在 `[16,16]–[624,464]`（`363`），与底部 UI 条（`y=460..480`，`521`）部分重叠。

### 5.6 渲染：纯 immediate mode，每颗弹 3 次绘制调用

- 每帧：`gl::clear`（`469`）→ 遍历 8192 槽（`472`）→ 对每颗活跃弹 **3 次 `gl::color` + 3 次 `gl::drawSolidCircle`**（半径 ×2.5 / ×1.5 / ×0.5，用透明度叠出“光晕”，`476-481`）。
- `gl::drawSolidCircle` 在 Cinder 里是**每次调用都现场生成几何**：`numSegments = floor(radius*2π)`（最小 3，`cinder/src/cinder/gl/draw.cpp:1132-1134`），然后 `new uint8_t[dataSizeBytes]`（`draw.cpp:1166`）→ `bufferSubData` 上传（`1188`）→ `drawArrays(GL_TRIANGLE_FAN, ...)`（`1192`）。
  - 即：**每颗弹 ≈ 3 次堆分配 + 3 次顶点生成 + 3 次 VBO 上传 + 3 次 draw call**。半径 6 的弹单圆就是 37 个三角形扇分段。
- **没有** sprite 批量、纹理图集、实例化、VBO 复用、shader 管理；颜色用 `gl::color` 逐颗切换（`476-480`），因此也无法批处理。
- 玩家：2 个实心圆 + （可选）判定点描边圆（`489-497`）；无敌闪烁用**墙钟时间** `fmod(getElapsedSeconds()*10, 0.2f)`（`488`）——暂停时 `draw()` 仍被调用，闪烁继续（视觉小瑕疵）。
- 特效：`hitEffects`/`grazeEffects` 逐条画圆，alpha = 寿命/10 或 /20（`502-509`）。
- UI：命数圆点（`512-517`）、底栏矩形（`520-521`）、弹幕颜色条（`524-533`）、6 个进度点（`536-540`）、判定点开关指示（`543-545`）、一个硬编码的“录制中”红点（`548-549`，用途不明，未确认）、输入指示圆（`552-554`）、方向键可视化（`557-567`）。
- **全程没有任何文字渲染**：`score` 只被累加（`407,448`），从未显示（`grep -n "score"` 只有 43/407/448）→ README 的 “Score bonus / Graze System” 在画面上不可见。

### 5.7 弹幕模式组织：`if/else` 链 + 函数内 `static` 计时器

`updatePatterns(float dt)`（`191-307`），靠全局 `currentPattern`（`159`）分支。每种 pattern 用 **函数局部 `static`** 保存计时/游标（`199-200, 218-219, 231, 245, 261-262, 274-275`），并用 `while (timer >= 间隔) { timer -= 间隔; ... }` 补发多轮。

| # | 名称 | 行 | 间隔 | 弹型/参数 |
|---|---|---|---|---|
| 1 | Curtain Rain | 198-214 | 0.1 s | 列循环 0-7，`x = 80 + col*60`，速度 `200+30d`，半径 6 红弹；偶数列额外 ±30 px、`vx=±20` 橙弹 |
| 2 | Spiral | 216-228 | 0.02 s | 中心 `(320,50)` 双向旋转直线弹，速度 `120+10d` |
| 3 | Burst | 230-242 | 0.8 s | `count = 20+5d` 环形爆发，半径 5 紫弹 |
| 4 | Aimed | 244-259 | 0.3 s | `atan2` 瞄准玩家，`count = 3+d`，扇形 `spread=0.2 rad`，速度 `180+20d` |
| 5 | Wall | 261-271 | 1.0 s | 5 颗大弹（半径 8）`x=100..500`，`vy=150` |
| 6 | Mixed | 273-298 | 0.5 s | `sub = (sub+1) % 4`，但**只实现 sub==0/1/2，`sub==3` 什么都不发射** → 实际是 3 相循环 + 一个空拍 |

- 调度方式：**参数化硬编码**，没有数据文件、没有脚本、没有状态机抽象；`difficulty` 是常量 3.0（见 4.5）。
- 自动轮换：`patternTimer += dt`，超过 15 s 就 `currentPattern++`（6→1）（`301-306`）；手动 1-6（`589-594`）。
- ⚠️ `static` 状态**在切模式时不重置**：切换到某模式时，它的 `timer`/`sub`/`angle` 还保留上次离开时的值 → 第一发的时间点不可预测；`angle` 也永不取模（长时间运行后浮点精度下降）。
- ⚠️ 帧率耦合：`while (timer >= 0.02f)` 内用 `dt` 推进 `angle`（`220-222`），见 5.3。

---

## 6. 关键数据结构与内存/性能设计

**对象生命周期**：`setup()` 创建玩家（槽 0）→ `spawnBullet` 借槽 → 越界/超龄（`384`）或命中（`406,424`）置 `active=false` → 下次生成复用。全程**没有一次 `delete`/`free` 实体**，也没有 `new` 单个实体。

**分配次数（静态可见）**
- 实体存储：**0 次堆分配**（`Entity entities[8192]` 在 BSS，`65`）。按字段布局估算 `sizeof(Entity) ≈ 100 B`（`4 + 12 + 12 + 32 + 8 + 32`，含对齐填充），整块约 **0.8 MB**——**未经编译器 `sizeof` 实测，属估算**。
- `grid`：300 个 `vector<int>`，容量跨帧保留（`74-78`），`gridInsert` 的 `push_back` 摊还 O(1)。
- **每帧的堆活动**（读代码推断，未实测）：`vector<int> toDestroy`（`376`）+ 每颗玩家弹一次 `gridQuery` 里的 `vector<int> seen`（`100`）+ `hitEffects/grazeEffects` 的 `push_back`（`405,423,427,446`）+ 其 `erase` 逐元素搬移（`456-465`）。被击中时还会**一次性销毁全场敌弹**并逐颗 push 特效（`421-426`）→ 最坏情况单帧数千次 `push_back` 与一次 O(N) 全扫。

**缓存友好性（这是最关键的设计缺陷）**
- AoS + “万能实体” 使得每颗子弹迭代要跨 ~100 B 步长，而子弹真正用到的字段只有 ~24-32 B；每帧 3 次全量 8192 槽扫描 → 大量无用内存流量与分支预测失败。
- 空槽也在被遍历：8192 槽中活跃数通常远小于容量。
- 「每颗弹 3 次现场几何生成」把 CPU 侧顶点计算和 VBO 上传放进热路径（5.6）。

**实测性能数据**：**没有**。README 的 “thousands of bullets”“O(1) collision”（`README.md:17-18`）**没有任何基准或计数代码支撑**；`src/BulletHell.cpp` 中不存在 FPS/计时/统计代码（`grep -i "fps\|profile"` 无命中）。README 里的 8192、O(1) 只能当作**设计意图**，不能当作测得指标。

**确定性**：不满足。变步长 dt（`330`）+ 模式 `static` 状态（5.7）+ 墙钟闪烁（`488`）+ `float` 累加，都使同一输入序列不可复现。

---

## 7. 测试 / 构建产物

- **测试：没有**。项目内只有 `src/BulletHell.cpp` 一个文件，无 `test/`、无 `tests/`、无 CI 配置。（`cinder/test` 是 Cinder 框架自己的测试，34 MB，与本项目无关。）
- **`build/` 的内容**（Linux，`du -sh` 实测）：
  - `build/BulletHell` 36 M —— Debug、含调试信息、未 strip 的 ELF 可执行文件；
  - `build/cinder_build/` 173 M —— Cinder 的 282 个 `.o` 与 Makefile；
  - `build/CMakeCache.txt` 128 K、`build/CMakeFiles` 1.5 M（含 `link.txt`）。
  - 另有 `cinder/lib/linux/x86_64/ogl/Debug/libcinder.a` 178 MB —— 由 Cinder 的 CMake 输出到源码树内。
- **如何自己构建**（依 `CMakeLists.txt` + `CMakeCache.txt` 推断，本次未执行）：
  ```bash
  cd /home/zhang/Bullet_Platform/Outside/Bullet/TostEngine-cinder-bullethell-main
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug   # 会一起编译 Cinder（add_subdirectory）
  cmake --build build -j
  ./build/BulletHell                              # 需要 X11/OpenGL 显示环境
  ```
  前置条件：`cinder/` 必须存在（否则 `CMakeLists.txt:11-13` 直接 FATAL_ERROR）；系统需有 OpenGL/GLX、X11（`libX11/Xext/Xcursor/Xinerama/Xrandr/Xi`）、`libcurl`、`fontconfig`、以及 Cinder 的音频依赖（pulse/gstreamer/mpg123/sndfile，见 `link.txt`）。
  - 由于旧 `build/` 缓存里的源码路径是移动前的 `/home/zhang/Bullet_Platform/TostEngine-cinder-bullethell-main`，**必须重新 configure**（或删掉 `CMakeCache.txt`）；否则 make 会指向不存在的路径。
  - README 提到的 `build/BulletHell.sln` 在本副本中不存在（`README.md:51`）→ Windows 步骤不可直接照做。
- Debug 构建占 210 MB，Release 会更小；但 `cinder/` 源码树本身的 384 MB 与该无关。

---

## 8. 局限与坑

**依赖与环境**
1. Cinder 是重依赖：源码树 384 MB、单个 Debug 静态库 178 MB，且需要 X11/GL + 音频栈；在小车上/无头环境不可用（`cinder/include/cinder/app/App.h:34-37` 有 `CINDER_HEADLESS` 分支，但本项目代码直接用了 `RendererGl` 与 `App`，`src/BulletHell.cpp:1-2,615`）。
2. `CMakeLists.txt` 把 Cinder 当**子目录整体编译**（`16`），无法用系统安装的 Cinder；且强制 Debug MSVC 运行时（`22`）。
3. 构建缓存路径过期（见 7），直接复用 `build/` 会失败。

**代码质量**
4. 单文件 615 行、全局可变状态（`entities`、`currentPattern`、`difficulty`、`godMode`）、无命名空间、无头文件、无单元测试；注释约 73 行（`grep -c "//"`）。
5. 死字段：`Transform::rotation`、`Velocity::angular`、`Collider::mask`（`30,31,39`；`mask` 仅 `179` 赋值）——看起来像“预留能力”，实际未实现，容易误导读者。
6. **无敌帧单位错误**：`iFrames = 60`（`47`）但按秒倒计时 `iFrameTimer -= dt`（`394`）→ 实际约 **60 秒**无敌，不是 60 帧。
7. `createEntity()` 漏重置 `player` 子结构（`122-126`）→ 槽位复用时残留玩家状态。
8. 玩家中弹时 `for (j=0..8192)` 全清弹幕 + 每颗一个特效（`421-426`）→ 卡顿尖峰。
9. `difficulty`、`godMode` 既不可从外部配置也**没有按键**（`161,158,397`）→ README 把它们列为“Game State Variables”，但只能改代码。
10. Pattern 6 的 `sub==3` 空拍（`279-296`）；Pattern 的 `static` 计时器不随切换重置（`199-278`）；`fireTimer` 只在按住时递减（`366`）。
11. `M_PI` 非标准（`238,282`），MSVC 需额外宏。
12. UI 不显示分数（`grep score` 只 43/407/448）；有一个含义不明的硬编码“录制中”红点（`548-549`）。

**能力边界**
13. **只有圆-圆碰撞，没有任何障碍物/矩形/AABB 表达**——对“二维动态障碍环境模拟器”而言，这个参考项目在障碍物、旋转体、连续碰撞（CCD）、隧道效应处理上**提供不了任何参考**。
14. **没有曲线弹**（5.3）：角速度字段是死的，所谓“Spiral”是生成时定向的直线弹。用户若想找“角速度数学”，这个仓库**没有**。
15. 没有固定步长/可复现性设计（5.1），与“确定性仿真内核”的目标完全相反。
16. 无纹理、无音频、无字体、无存档、无配置系统——对“工业级引擎”的期待要放低：它是一个约 600 行的教学 demo，不是引擎。

**许可证**
17. README 声称 MIT（`README.md:71-73`）但**未附 LICENSE 文本**；Cinder 部分是 BSD 2-Clause（`cinder/COPYING:1-3`）。复用前需向上游确认授权文件（工作区 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:37` 已记为 `[LICENSE_REVIEW_REQUIRED]`）。

---

## 9. 对用户项目（`bullet_sim`）的可借鉴点

> 结论先行：这个参考项目**值得看的是“反面教材 + 少数正面细节”**，尤其是实体池的量化代价、空间网格的保守插入、以及“README 声称 ≠ 代码事实”这一点。用户现有 Python 内核在**固定步长、SoA、精确圆弧积分**三件事上已经领先于它。

### 9.1 实体池：AoS 定长数组 vs 用户的 SoA + LIFO 自由表

| 维度 | TostEngine 做法（来源） | 用户现状 | 建议 |
|---|---|---|---|
| 布局 | `Entity entities[8192]`，AoS，每槽 ~100 B 且装下所有组件（`65,55-62`） | `BulletPool.data` 结构化数组，按列连续（`bullet_sim/physics/motion.py:49-53`） | 保持 SoA；**不要**学“万能实体” |
| 生成 | first-fit 线性扫描 O(N)，从 0 开始（`119-128`） | LIFO 自由列表（`OPEN_SOURCE_REFERENCES.md:47`） | 保持 LIFO：O(1)，且**分配顺序是生成序列的纯函数**，利于复现；TostEngine 的 first-fit 会让“槽位 ID”依赖历史，破坏可复现性 |
| 回收 | 只清 `active`，无版本号，槽位立即复用（`133`） | 有 `alive` 掩码 + id 数组 | 若外部持有 bullet id，建议保留版本号/单调 id（用户 `data["id"]` 已是这个思路），TostEngine 的“下标即 ID”是反面案例 |
| 遍历 | 每帧 3 次全量扫 8192 槽，空槽也判分支（`378,398,472`） | `np.flatnonzero(alive)` 只取活跃 | 保持“只碰活跃元素”；`motion.py:43-47` 的 `count==0 / ~sel.any() → return` 早退是正确习惯，TostEngine 没有 |
| 重置 | 漏重置 `player`（`122-126`） | — | 池化时务必“重置**全部**字段”，或改用“创建时写全字段”的不可变风格 |

**可直接量化的教训**：8192 槽 × ~100 B ≈ 0.8 MB（估算），而有效载荷只有 ~24-32 B/弹；AoS 让每次积分循环多读 3-4 倍内存。用户的 SoA 等价字段是 `x,y,vx,vy,ax,ay,angle,angular_velocity,radius,age,ttl,type_id,group_id + alive`（`OPEN_SOURCE_REFERENCES.md:44`）——把热字段（`x,y,vx,vy,omega,alive`）与冷字段（`ttl,type_id,group_id`）分开排布，还能再提高缓存命中。

### 9.2 角速度驱动速度方向旋转：精确圆弧积分（对用户公式的对比确认）

TostEngine **没有实现**（`Velocity::angular` 死字段，`31`）。用户已在 `bullet_sim/physics/motion.py:90-104` 用闭式弧积分：

```
θ = ω·dt
dp    = (sinθ/ω)·v_new + ((1−cosθ)/ω)·perp(v_new)      # perp(v) = (−vy, vx)
v_rot = R(θ)·v_new
```

这与我的独立推导一致（供交叉验证）：设速度方向角 `φ(t)=φ0+ωt`、速率 `s=|v|`，
```
∫₀^{dt} s·cos(φ0+ωτ)dτ = (s/ω)[sin(φ0+ω dt) − sin φ0]
∫₀^{dt} s·sin(φ0+ωτ)dτ = −(s/ω)[cos(φ0+ω dt) − cos φ0]
```
把 `(v_x, v_y) = s(cos φ0, sin φ0)` 代入，即得
```
Δx = (sinθ/ω)·v_x − ((1−cosθ)/ω)·v_y
Δy = (sinθ/ω)·v_y + ((1−cosθ)/ω)·v_x
```
与 `motion.py:101-102` 完全一致。几点对比与提醒：

1. **TostEngine 完全没有这条路径**——用户如果按 `OPEN_SOURCE_REFERENCES.md:45` 的表述以为“借鉴了它的角速度实现”，那是不准确的；用户的实现是自研且更正确。
2. **精确弧 vs 欧拉**：`ω→0` 时公式退化为 `v·dt`（`motion.py:95-98` 已做小角分支）；误差是弦长与弧长之差，量级 `O((ω·dt)³/24)`。若按 TostEngine 式的“先转方向再走直线”（半隐式欧拉）会额外引入每步 `O((ω dt)²)` 的横向偏差——这正是用户已经避免的。
3. **确定性/FPGA 视角**：`sin/cos` 是超越函数。在 float64 下 CPU 逐位复现没问题；但移植到 FPGA 时，**要么用定点 CORDIC/LUT（会与 CPU 结果产生最末位差异），要么规定仿真内核只在 CPU 上跑精确弧、FPGA 只做近似或粗筛**。建议把“弧积分用的 θ、sinθ、cosθ 的求值方式”写进复现契约，否则 CPU/FPGA 两条路径不可比对。
4. **大 `ω·dt` 的几何含义**：弧公式假设步内 `|v|` 与 `ω` 恒定；用户已支持 `v_new = v + a·dt` 后再旋转（`motion.py:80-81,99-108`），这相当于“先加速再沿弧走一步”，是 `a` 与 `ω` 同时非零时的一种近似——值得在代码注释里明确（`motion.py` 目前把它描述为闭式arc integral，隐含了这一约定）。
5. **矩形语义**：用户让矩形用 `angular_velocity` 旋转**自身本体**而不弯速度（`motion.py:57-68`）——这个“按形状解释 ω”的设计在 TostEngine 里完全没有对应物，是该参考项目给不了的东西，建议保留并写进文档。

### 9.3 碰撞宽相

- **可以借鉴的正面细节**：`gridInsert` 按**实体 AABB 覆盖的所有格子**写入（`80-91`），保证“两圆相交 ⇒ 共享格子”，因此查询完备；用户 `grid.py:96-101,135-136` 的 `_cell_ids` + `reach = ceil((max_pr + 2·max_r)/cell)` 是它的向量化等价版本。
- **必须避开的做法**：`gridQuery` 每次 `vector<int> seen` + O(k²) 线性去重（`100-112`）。用户的 CSR（`grid.py:86-94`：`np.bincount` + `np.cumsum` + 稳定 `argsort`）天然无重复、无逐查询分配——方向正确。
- **格子尺寸**：TostEngine 是固定 32 px（`71`），对象半径 3-8 → 格子远大于对象（几乎退化成“很多格只放 1 颗弹”）。用户用 `cell = 4·max_r`（`grid.py:53`）并让 3×3 邻域充分（`grid.py:132-136`），是更稳的参数化。可以借用 TostEngine 的一点经验：**cell 与最大半径绑定**时要处理“出现超大子弹”的情况（用户已用 `_max_r` 自适应重建，`grid.py:79-83`；TostEngine 完全没有这个处理，半径 8 的 Wall 弹与格子 32 的比例尚可，但若有半径 >16 的弹就会退化为“每弹写 4 格”，效率下降）。
- **碰撞体形状**：只有圆（`39`）。用户已有圆/点/矩形玩家与旋转矩形障碍（`grid.py:193-199` 的 `shaped`/`obstacle` 模型），TostEngine 无可借鉴。
- **分层过滤**：TostEngine 用 `bullet.flags`(0x01/0x02) 在回调里过滤（`399,402,416,442`），而同样语义的 `collider.mask`（`179`）从未被读——**教训**：掩码/分组要么真正参与判定，要么删掉，否则会像这里一样成为“看起来有碰撞层、实际靠硬编码标志”的陷阱。用户有 `group_id/type_id`，建议保持显式矩阵过滤并有测试覆盖。
- **单查询 vs 批查询**：用户已经实测“单玩家时暴力法更快”（`grid.py:5-10,113-117`）——TostEngine 每帧为每颗玩家弹单独 `gridQuery`（`398-401`），正是“为少量查询重建/遍历网格”的坏例子，支持用户“单查询回退暴力”的决定。

### 9.4 主循环/时间

- TostEngine 的 `dt` 钳制（`330`）值得注意其**存在理由**（防止卡顿后一次性推进过大），但做法（变步长）对确定性仿真是**禁忌**。用户 `FixedClock`（`t = step_index·dt`，`OPEN_SOURCE_REFERENCES.md:48`）才是正确路线；可借鉴的是“**max substeps / 最大 dt 上限**”的防御性思路——当外部实时接口（小车）喂进乱序/超长间隔时，应**拒绝或补步**而不是可变步长，且这个决策要写进复现契约。
- 渲染/逻辑分离：TostEngine 只做到“函数分离”（`update`/`draw`，`324,468`），没有插值状态。用户若要可视化，应保留“仿真只按固定步推进、渲染只读快照”的严格边界（TostEngine 做不到，且有 `static` 状态 + 墙钟闪烁 `488` 这些反面例子）。

### 9.5 哪些值得用 C++/FPGA 重写，哪些不值得

**值得重写（纯数值、逐元素、无分支的核）**
1. 位置/速度积分：`x += vx·dt + ½ax dt²`（用户 `motion.py:75-78`）+ 圆弧积分（`motion.py:90-108`）。
2. 空间网格构建：CSR 计数排序（`grid.py:84-94`）——这是**内存带宽受限**的典型，用 C++/SIMD 或 `np.bincount` 已经足够；用 C++ 主要是省 Python 开销。
3. 窄相距离/形状测试（`grid.py:167-175`）。
4. 活跃集压缩：`alive` → 索引列表（替代 TostEngine 式 8192 全扫，`378-389`）。

**不值得重写**
5. 弹幕模式/发射器调度（TostEngine 的 `if/else` + `static`，`191-307`）：这是**创作层**，应该留在 Python（或数据文件/脚本）里；它分支多、迭代次数少，C++/FPGA 收益最小、调试成本最高。用户若要“可复现”，应改成**参数化 + 纯函数式生成**（给定 step 索引和参数 ⇒ 确定输出），而不是 TostEngine 的 `static` 状态。
6. 特效/UI/绘制（`468-568`）、碰撞分层簿记、事件统计：留在 Python/主机侧。
7. 渲染：**不要**移植 TostEngine 的 immediate mode（每弹 3 次 `drawSolidCircle`、每次 `new[]` 顶点、`draw.cpp:1120-1193`）。用户若需要高速可视化，正确做法是**单 VBO + 实例化四边形/点精灵批渲染**（一次上传位置/半径/颜色数组，一次 draw call）。这是一个“用它的错误做法省下几百行 C++”的机会。

**FPGA 的量化判断依据（粗算，供决策）**
- 若热字段按 10 个 float32/实体、规模 8192：`8192 × 10 × 4 B ≈ 320 KB/步`；1000 step/s ⇒ 约 **0.33 GB/s** 内存流量。这个量级 **向量化 C++/NumPy 完全吃得下**，FPGA 只有在“整条 spawn→积分→宽相→窄相 流水线化、且规模再上一个数量级（10⁵–10⁶）”时才有明显优势。
- TostEngine 提供了两个值得遵守的**负面约束**：① 每个实体的处理必须**无分支、无堆分配**（它的 `gridQuery` 去重、`hitEffects.push_back`、`erase`，`100,405,456-465` 全都不满足）；② 处理数据必须**按列连续**（它的 AoS 100 B 步长不满足）。用户现有 SoA + 预分配池正好满足这两条，**保持它**比“照搬 TostEngine 的实体结构”重要得多。

### 9.6 文档层面的建议
- 若要在工作区文档里引用本项目，请把 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:45,48` 的两处表述改准：本项目**没有**角速度积分实现，也**不是**固定步长主循环（证据：`src/BulletHell.cpp:31` 死字段、`327-330` 变步长）。本文档可作为引用来源。

---

## 附录 A：README 声称 vs 代码事实

| README 声称 | 出处 | 代码事实 | 判定 |
|---|---|---|---|
| Fixed timestep game loop | `README.md:46` | 变步长 + dt 钳制 0.25 s | ❌ 不符（`src/BulletHell.cpp:327-330`） |
| Delta-time based pattern timing | `README.md:47` | 模式计时用 dt，但 `angle` 增量也误用 dt | ⚠️ 部分成立（`202,220-222`） |
| Spatial Grid, O(1) collision | `README.md:17` | 每查询近似 O(1)，但整帧 O(N) + O(k²) 去重 | ⚠️ 夸大（`378-389,100-112`） |
| Entity Pooling up to 8192 | `README.md:18` | 定长数组 + `active`，无 free-list，生成 O(N) | ⚠️ 说法含糊（`64-65,118-130`） |
| ECS-inspired entity system | `README.md:45` | AoS 单体结构，携带全部组件 | ❌ 不是 ECS（`55-62`） |
| 6 patterns | `README.md:32-39` | 6 个分支存在，但 Pattern 6 有 1/4 空拍 | ⚠️ 基本成立（`279-296`） |
| Graze / Score bonus | `README.md:12` | 计分有效，但 score 从不显示 | ⚠️ 无可见反馈（`407,448`） |
| 3 lives with invincibility frames | `README.md:14` | `iFrames=60` 按秒倒计时 ⇒ 约 60 s | ❌ 单位错误（`47,394`） |
| Build via `build/BulletHell.sln` | `README.md:51` | 本副本无 `.sln`，是 Linux CMake 树 | ❌ 过时/平台错配 |
| MIT License | `README.md:73` | 无 LICENSE 文件，仅 Cinder 的 BSD-2 | ⚠️ 待上游确认 |
| Entity pooling supports 8192 | `README.md:18` | `MAX_ENTITIES = 8192` | ✅ 一致（`64`） |
| 6 键切弹幕 / 15 s 自动轮换 | `README.md:15-16` | 实现一致 | ✅（`589-594,301-306`） |

## 附录 B：关键常量与实测数字速查

| 项 | 值 | 来源 |
|---|---|---|
| 窗口/场地 | 640 × 480 | `src/BulletHell.cpp:19-20` |
| 玩家速度 / 判定半径 / 擦弹半径 | 250 / 3.0 / 12.0 | `21-24` |
| 射速 / 最大实体 | 0.05 s / 8192 | `23,64` |
| 网格 | cell 32 px，20 × 15 = 300 格 | `71-74` |
| 模式间隔 | 0.1 / 0.02 / 0.8 / 0.3 / 1.0 / 0.5 s | `202,220,233,247,264,277` |
| 自动轮换 | 15 s | `302` |
| 库总量 / Cinder / build / 自有源码 | 594 M / 384 M / 210 M / 27 908 B | `du -sh`、`stat` |
| 最大单文件 | `cinder/lib/linux/x86_64/ogl/Debug/libcinder.a` = 177 984 696 B | `find -printf %s` |
| 头文件数（cinder/include） | 450 `.h` + 827 `.hpp` | `find \| wc -l` |
| 全仓库文件数 / 自有文件数 | 7398 / 4 | `find \| wc -l` |
| 构建配置 | CMake 3.16+ / C++20 / Debug / Unix Makefiles / GCC 13 / CINDER_TARGET=linux, GL=ogl | `CMakeLists.txt:1-4`、`build/CMakeCache.txt` |
| Cinder 版本 | 0.9.4dev（`CINDER_VERSION 904`） | `cinder/include/cinder/Cinder.h:39-40`、`cinder/README.md:1` |
| GLM 版本 | 0.9.9（由 Cinder 内置） | `cinder/include/glm/detail/setup.hpp:6-8` |

## 附录 C：未能确认的事项

1. 作者/上游仓库 URL/提交号——本地无 `.git`，README 无署名（唯一外链是截图与 libcinder.org，`README.md:5,43`）。
2. `README.md` 声称的 MIT 许可的完整授权文本（仓库内不存在）。
3. 任何性能数字（子弹数上限下的帧率、碰撞耗时）——代码与文档都没有基准。
4. GL/X11 未出现在可执行文件 `NEEDED` 中的确切机制（推测为 GLFW 运行时加载或 `--as-needed`）。
5. `sizeof(Entity)` 未经编译器实测（本文 100 B/0.8 MB 为按字段布局的估算）。
6. `src/BulletHell.cpp:548-549` 那个红色“录制中”圆点的用途（代码无注释、无关联逻辑）。
