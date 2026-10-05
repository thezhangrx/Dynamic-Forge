# Keiki 技术总结（只读代码审阅报告）

> **本文档的取证约定**
>
> * 全部结论来自对 `/home/zhang/Bullet_Platform/Outside/Bullet/Keiki-master` 的实际阅读，引用格式为 `相对路径:行号`。
> * 未在代码/文档中找到依据的判断，一律标注 **未确认**，不做推测性断言。
> * 本次审阅**只读**：未修改任何既有文件、未安装依赖、未训练模型、未联网。因此“训练是否收敛”“生成质量如何”这类问题**无法**由本报告回答 —— 仓库里既没有 `result/` 目录也没有任何 `.pkl`/`loss.json` 训练产物（已用 `find` 核验）。
> * 数据文件形状未用 numpy 读取（本机无 numpy），改为**纯 Python 解析 `.npy` 头 + `struct` 解包**，结论等价（见 §6）。
> * 全文唯一新建的文件就是本 `tech.md`。

---

## 1. 一句话定位

**Keiki** 是一个用 Python 写的、面向研究（尤其是“弹幕生成”）的**东方风格弹幕地狱（bullet hell）游戏平台**：它把弹幕抽象成可子类化的 `Danmaku` 类，并**保证任何子类都能被编码成一条定长参数序列**，从而用 GAN 学习并生成新的弹幕（`README.md:3-5`）。

| 项 | 内容 | 来源 |
|---|---|---|
| 英文定位 | “A bullet hell game platform for research purpose (especially danmaku generation) written in Python” | `README.md:3` |
| 核心机制 | 子类化 `Danmaku` → 自动编码为参数序列 → 训练 GAN | `README.md:5` |
| 作者 | Ziqi Wang, Jialin Liu, Georgios N. Yannakakis | `README.md:13` |
| 论文 | *Keiki: Towards Realistic Danmaku Generation via Sequential GANs*, IEEE CoG 2021 | `README.md:11-18` |
| 上游要求 | README 只提出**引用请求（bibtex）**，未给出任何授权条款 | `README.md:8-19` |
| **许可证** | **未找到 LICENSE 文件（法律风险）** | 见下 |

### 1.1 许可证核验（对用户最重要的一条）

已做穷尽式检索：`find . -iname '*licen*' -o -iname '*copying*' -o -iname '*notice*'`（含子目录、大小写不敏感）。
**结果：未找到 LICENSE 文件（法律风险）。** 全仓库根目录下的非代码文件只有 `README.md` 与 `.gitignore` 两个（`find` 排除 `*.py/*.npy/*.png/*.pyc` 后的完整结果）。

* 目录内也**没有** `.git/`（是导出快照，不是 clone），因此无法通过提交历史反查许可证变更。当前快照的 34 个 `.npy` 与所有源码 mtime 均集中在 2021-07-05。
* README 的 bibtex 请求（`README.md:8-19`）在法务上**不等价于授权**：它只约束学术引用礼仪，没有授予复制、派生、再分发代码或数据的权利。
* 结论：`[LICENSE_REVIEW_REQUIRED]` 成立。**可读、可学、可独立重写；不可复制代码或 `.npy` 数据**。用户的 `bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:68-80` 已经采用这条结论，本报告予以独立复核确认。

---

## 2. 技术栈与工具链

### 2.1 官方声明环境（README，非机器可验证）

`README.md:21-33` 列出“已测试环境”：**Python 3.7.6 / Pygame 2.0.1 / PyTorch 1.5.0 / Numpy 1.18.1 / Seaborn 0.11.1 / Matplotlib 3.3.1`。

### 2.2 依赖清单（**无 requirements.txt**）

**未找到 `requirements.txt` / `setup.py` / `pyproject.toml` / `environment.yml`（已用 find 核验）**。依赖只能从 import 语句统计得出（`grep -rhoE "^(import|from) ..."`，按顶层模块计数）：

| 类别 | 依赖 | 证据 | 必需性 |
|---|---|---|---|
| 游戏/渲染 | `pygame`（`display`/`image`/`sprite`/`transform`/`time.Clock`/`key`） | `game.py:2-6`、`render/real_time/renderer.py:1-2`、`utils/graphics.py:1-2` | **硬依赖**，且 `root.py:2,6` 在 **import 期**就 `image.load()` |
| 数值 | `numpy` | `game.py:3`、`utils/math.py:1`、`data_make.py:4`、`logic/danmaku/configurations.py:2` | 硬依赖 |
| 深度学习 | **PyTorch**（`torch.nn`、`torch.optim.Adam`、`torch.optim.rmsprop.RMSprop`、`torch.utils.data`） | `generator/*/train.py`、`generator/*/models.py` | 仅训练/生成需要 |
| 可视化 | `matplotlib` + `seaborn`（仅 `generator/visualization.py:1-2` 需要） | — | 仅可视化需要 |
| 其他 | 标准库 `os sys json time argparse inspect importlib heapq copy abc enum math random`；**没有** TensorFlow/Keras/JAX/OpenCV/numba | — | — |

* **深度学习框架是 PyTorch**（不是 TF/Keras）：`generator/DCGAN/models.py:2`、`generator/TimeGAN/models.py:2` 均 `from torch import nn`。
* 三个训练脚本都以 `torch.set_default_dtype(torch.float64)` 开头（`generator/DCGAN/train.py:102`、`generator/PeriodicSpatialGAN/train.py:117`、`generator/TimeGAN/train.py:218`）→ **全程双精度训练**，与数据集 `.double()`（`generator/dataset.py:18,63`）一致，但会明显拖慢速度（代码里未解释原因）。
* 本地 `__pycache__` 里是 `*.cpython-312.pyc`（`game`、`root`、`realtime_sim` 等），说明**至少被人用 Python 3.12 成功 import 过**，与 README 声明的 3.7.6 不同；是否能在 3.12 下完整跑通游戏**未确认**。
* 无 `Dockerfile`、无 CUDA 版本锁定；`--gpuid`/`--gpu` 参数在 `torch.cuda.is_available()` 为假时回退 CPU（`generator/DCGAN/train.py:103`）。

---

## 3. 目录与模块地图

```
Keiki-master/  ├ README.md(唯一文档) root.py(PRJROOT/VOID) run_game.py run_make.py
               ├ game.py(主循环+3 个评估器) data_make.py(Danmaku→参数序列编码器)
               ├ agents/(预演+A*) danmakus/(34 个 .npy，游戏加载源)
               ├ data/code/(34 个手写 Danmaku 子类) data/mat/(编码产物)
               ├ generator/(dataset + DCGAN/PSGAN/TimeGAN + viz)
               ├ logic/(collision|danmaku|objects|runtime) render/real_time/ utils/ assets/
```

逐文件说明（行数来自 `wc -l`）：

| 路径 | 行数 | 职责 | 关键符号 |
|---|---|---|---|
| `root.py` | 7 | 定义 `PRJROOT`（以 `/` 结尾）与全局 `VOID` 贴图 | `PRJROOT`, `VOID` |
| `run_game.py` | 6 | `Game.run()` | — |
| `run_make.py` | 4 | `codes_to_param_seq('data/code', out_path='data/mat')` | — |
| `game.py` | 318 | 主循环 `Game`、按键、全局评估器 | `Game`, `KeyResponser`, `Monitor`, `FeatureBaseMetricEvaluator`, `AgentBaseMetricEvaluator` |
| `data_make.py` | 79 | 枚举 `data/code/*.py` 里的 `Danmaku` 子类并编码落盘 | `DanmakuEncoder`, `get_classes`, `codes_to_param_seq` |
| `logic/runtime/objs.py` | 29 | **全局对象表** + 一帧更新顺序 | `bullets/weapons/others`(BufferedLazyList), `update()`, `clear()` |
| `logic/runtime/bullet_creation.py` | 518 | 4 个 builder + 3 个几何展开器 + **编码方案** | `RuntimeBulletBuilder`, `RuntimeBatchedBulletBuilder`, `EncodingBatchedBulletBuilder`, `SimBatchedBulletBuilder`, `circle_cfgs/sector_cfgs/string_cfgs` |
| `logic/runtime/obj_creation.py` | 48 | 构造玩家/自机/僚机/Boss 并挂 sprite | `create_player`, `create_boss` |
| `logic/objects/bullets.py` | 189 | 4 种子弹实体（含**碰撞触发点**） | `Bullet`, `StaticSimBullet`, `InteractiveSimBullet`, `PlayerBullet` |
| `logic/objects/bullet_enums.py` | 85 | 15 种弹型 / 6 色 / 尺寸→出界范围/图层/权重 | `BulletTypes`, `BulletColors`, `get_weight`, `get_render_layer` |
| `logic/objects/boss.py` | 70 | Boss 状态机：加载 `danmakus/` 全部文件、HP、跳关 | `Boss.load`, `Boss.update` |
| `logic/objects/player.py` | 105 | 玩家（命中半径 2.0、自机狙轨迹 `trace` 500 帧） | `Player`, `MissEvent` |
| `logic/objects/weapons.py` | 60 | 主炮/僚机与冷却 | `Weapon`, `Sub` |
| `logic/collision/bullet2player.py` | 98 | 碰撞体（实际只有圆-圆） | `Collider`, `RoundCollider`（`HexagenCollider` 全注释） |
| `logic/danmaku/basis.py` | 107 | 弹幕基类 + 发射器调度基类 | `Danmaku`, `Shooter`, `PeriodicShooter`, `PlayerPosKeeper` |
| `logic/danmaku/common_shooters.py` | 85 | 3 个通用发射器 | `SpiningShooter`, `FakeLaser`, `TrapezoidShooter` |
| `logic/danmaku/configurations.py` | 80 | 角度/位置展开工具（**未被 builder 复用**，见 §4.5） | `sectoral_angles`, `circular_configs`, `univals`, `chord` |
| `logic/danmaku/cm_danmaku.py` | 138 | **解码**参数序列 → 每帧发射 | `decode_param_seq`, `convert_onehot`, `CMDanmaku`, `BatchedCMDanmaku` |
| `logic/danmaku/example.py` | 38 | 官方示例弹幕 | `ExampleDanmaku` |
| `logic/danmaku/test_danmaku.py` | 17 | **不是测试**，是一条示例弹幕 | `TestDanmaku` |
| `render/real_time/renderer.py` | 34 | 9 图层渲染器（`pygame.sprite.Group`） | `FullRenderer`, `FullRenderer.Layers` |
| `render/real_time/common_sprites.py` | 19 | 子弹/僚机 sprite | `CommonSprite` |
| `render/real_time/player_sprites.py` | 78 | 自机 3×8 动画 + 判定点 | `PlayerBody`, `PlayerHitBox` |
| `render/real_time/boss_sprites.py` | 20 | Boss 旋转贴图 | `HoujuuBallSprites` |
| `render/real_time/background.py` | 20 | 背景（`update` 已注释） | `Background` |
| `utils/math.py` | 210 | `Vec2`（极坐标/渲染坐标）+ `linear_mapping` | `Vec2.from_plr/to_rcs/theta`, `linear_mapping`, `bound` |
| `utils/data_structure.py` | 57 | **惰性缓冲列表（对象池雏形）** | `BufferedLazyList` |
| `utils/graphics.py` | 32 | sprite 基类（旋转/缩放/隐藏） | `BaseSprite` |
| `utils/assets_manage.py` | 70 | 贴图单例 + 精灵表切分 | `TexManager` |
| `agents/realtime_sim.py` | 131 | **离线预演表**（静态/交互子弹分表） | `RealTimeSimulator` |
| `agents/A_star_agent.py` | 187 | 用预演表做 A* 搜索的自动机 | `AStarAgent`, `Tree`, `StateNode` |
| `agents/simple_agents.py` | 62 | 17 个离散动作的随机/脚本 agent | `Agent`, `RandomAgent`, `PreinstalledAgent` |
| `generator/dataset.py` | 72 | 两个 Dataset | `DanmakuDataset`(死代码), `AugmentedDataset` |
| `generator/DCGAN/{models,train}.py` | 42/131 | 逐帧“参数序列图”生成 | `Generator`, `Discriminator` |
| `generator/PeriodicSpatialGAN/{models,train}.py` | 57/148 | 周期潜变量生成 | `Generator(带 sin 潜码)`, `Discriminator` |
| `generator/TimeGAN/{models,train}.py` | 67/253 | 时序隐空间 GAN | `LSTM`, `PeriodicGenerator`, `MixedReconstructor` |
| `generator/visualization.py` | 23 | 参数序列热力图 | `viz_paramseq` |

---

## 4. 运行时架构

### 4.1 入口链与主循环：**按帧定步长，但不用 dt 累加器**

入口链：`run_game.py:1,5` → `game.Game.run()`（`game.py:124`）→ `Game.__init__`（`game.py:104`）→ `objs.update()`（`objs.py:13`）。

```python
# game.py:137-170（节选）
while True:
    clk.tick(60)                      # game.py:138  限帧，不是仿真时钟
    for event in pygame.event.get(): ...   # 输入 / QUIT / ESC 暂停 / T 截图
    if game.pausing: continue         # game.py:158-159  暂停=整帧跳过
    ...                               # simulator 挂接 / agent 决策
    game.update()                     # game.py:169  恰好推进 1 个仿真步
    display.flip()
```

* **是否固定时间步？**：仿真步长**固定为“1 帧”**，但**没有 `dt`**。全仓库唯一的 `dt` 出现在 `logic/objects/bullets.py:48,83,118`，那是子弹加速的**子步积分内部变量**（`rt` 从 1.0 起、`dt = min(rt, speed/(|a|+1e-5)/3)`），与墙钟无关。速度单位是 **px/帧**（如 `speed=1.5`）。
* **帧率与仿真步的关系**：`clk.tick(60)` 只是把循环限速到 ≤60 次/秒；若一帧算不完（弹幕多时会），仿真**直接变慢**（不做追帧累加）→ **墙钟时间与仿真时间不严格对应**。`Game.update()`（`game.py:118-122`）先 `objs.update()` 再 `renderer.draw()`，因此**渲染无法与逻辑解耦成两条独立时间线**。
* **Headless 模式**：`Game(render=False)` 会 `objs.render = False`（`game.py:105-106`），不建窗口、不建 sprite（各 builder 的 `self.render` 分支，如 `bullet_creation.py:176-185`），但**仍然需要 pygame 可 import**（`root.py:2`、`logic/objects/player.py:1`）。
* 其他循环细节：ESC 按下 → `pausing=2`、抬起 → `pausing-=1`（`game.py:143-148`）；`S` 键**抬起**时 `objs.boss.spell.dead = True` 跳关（`game.py:97-100`）；`agent` 模式用 `agent.get_dvec()` 覆写 `player.dire_vec / slow_down`（`game.py:132-135,166-168`）；`RealTimeSimulator` 在 Boss 换弹幕时 `load(deepcopy(boss.spell))`、弹幕结束 `clear()`（`game.py:160-165`）。

### 4.2 实体系统：Python 对象 + 全局表 + 惰性缓冲列表（**不是 SoA，也不是预分配池**）

`objs.py` 是全局单例状态（模块级变量，非类）：

```python
# logic/runtime/objs.py:3-19
render = True
boss = None; player = None
bullets = BufferedLazyList(); weapons = BufferedLazyList(); others = BufferedLazyList()
def update():
    boss.update(); player.update()
    bullets.update(lambda x: x.dead)     # 一帧内移除死亡并合并新子弹
    weapons.update(); others.update(lambda x: x.dead)
```

`BufferedLazyList`（`utils/data_structure.py:1-57`）是**对象池的雏形**，机制值得注意：

* `add()` 只压入 `__buffer` 并置 `__updated=False`；`update(condition)` 才把 buffer 合并进 main（`data_structure.py:8-43`）。
* 合并时若主列表某槽位满足 `condition`（已死），**优先用 buffer 里的新对象原地覆盖该槽位**（`data_structure.py:27-32`）→ 这就是“**死亡槽位复用/自由列表**”，避免 list 删除搬移。
* 若 buffer 用尽而仍有死槽，`continue` 跳过（`data_structure.py:29-32`）：该槽位既不更新也不回收，作为“洞”留到以后复用。
* `__capacity` 支持容量上限，超出者被标记 `dead=True`（`data_structure.py:34-39`），但**全仓库没有任何调用点传入 `capacity`** → 该分支是死代码。
* `__iter__` 在 `update()` 之前被访问会 `raise RuntimeWarning`（`data_structure.py:50-54`）→ 强制“先更新再遍历”的纪律。
* 新加入的对象**当帧不 update**（合并后不会调用其 `update()`），因此“本帧发射的子弹/发射器”下一帧才动 —— 这一点直接影响弹幕编码时的时序（§4.5 有实测证据）。

实体类（都是普通 Python 对象，**每颗子弹一个对象 + 一个 kwargs 字典**，`logic/objects/bullets.py`）：

| 类 | 位置 | 语义 | 运动表示 |
|---|---|---|---|
| `Bullet` | `bullets.py:5-51` | 运行时可碰撞子弹（**唯一会打玩家的**） | `rho` 累积 + `start_pos + from_plr(rho, theta)`，`speed`/`frac`/`power` 做加速 |
| `StaticSimBullet` | `bullets.py:53-85` | 预演用、**无碰撞**的固定极坐标弹 | 同上，但**不做 `__getattr__`/渲染** |
| `InteractiveSimBullet` | `bullets.py:88-125` | 预演用“自机狙”弹：位置依赖玩家轨迹 | `abs_pos(rho, theta, spos, ppos)`（`bullets.py:122-125`） |
| `PlayerBullet` | `bullets.py:128-154` | 自机子弹 | 笛卡尔匀速 `velocity` |
| `Player` | `player.py:48-100` | 自机（半径 2.0、`trace` 保留 500 帧） | `dire_vec.norm() * (speed_low|speed_high)` |
| `Boss` | `boss.py:8-70` | 弹幕机状态机（hp=600、`spell_interval=90`） | 不做移动（移动代码被注释，`basis.py:79-84`） |

关键点：**没有 SoA、没有连续数组、没有预分配池**。每颗子弹是 `Bullet(**kwparas)`（`bullet_creation.py:174`），`kwargs` 字典逐颗重建（`bullet_creation.py:143-144`），且每颗子弹还会新建一个 `CommonSprite`（`bullet_creation.py:179-185`）。这是本项目的性能上限所在。

`Vec2`（`utils/math.py:28-200`）是**可变**向量，且 `__getattr__` 把 `r/m/magnitude` 映射为模长（`math.py:142-150`）、`__add__/__sub__` 对标量做了“改模长”的重载（`math.py:152-166`）。渲染坐标 `to_rcs() = (192+x, 224-y)`（`math.py:113-119`），即世界原点在 384×448 场地中心，`theta` 是“以 +y 为 0°”的屏幕角（`math.py:121-129`）。

### 4.3 碰撞检测：**逐颗圆-圆，O(N)/帧，无任何宽相**

* 触发点不在独立模块里，而在**子弹自己的 update 里**：

```python
# logic/objects/bullets.py:26-38
def update(self):
    if self.dead: return
    if not objs.player.no_collison:
        collision = self.collider.detect(self.pos)   # ← 每颗弹每帧一次
        if collision:
            self.dead = True; objs.player.miss(); return
    self.angle += self.spin; self.move()
    if self.pos.is_out(*self.out_range): self.dead = True
```

* 算法是**圆-圆**：`RoundCollider.detect` 比较平方距离与 `(hr + radius)²`（`logic/collision/bullet2player.py:32-35`），其中 `Collider.hr = Player.hr = 2.0`（`bullet2player.py:8`、`player.py:49`）。半径由**弹型表**给出，不是逐实例：`RuntimeBulletBuilder.collider_mapping`（`bullet_creation.py:104-123`）把 15 种 `BulletTypes` 映射到 2.4～15.0 的固定半径（如 `DOT:2.4`、`S_JADE:4.0`、`G_JADE:15.0`）。
* **复杂度**：每帧对存活子弹各做一次 O(1) 判定 → **O(N_bullets)/帧**，加上 `objs.bullets` 遍历本身也是 O(N)。**没有网格哈希、没有四叉树、没有 AABB 粗筛、没有子弹-子弹碰撞**。
* 其他碰撞/命中判定（都是特例，不是通用系统）：
  * 自机子弹 → Boss：**矩形区间** `-40<x<40 and -48<y<40`（`bullets.py:146-150`）；
  * Boss → 玩家：距离 `< 30` 直接判 miss（`boss.py:52-53`）；
  * 预演里的“安全余量”：临时把 `Collider.hr += 2.0`/`hrsqr += 12.0`，算完再减回去（`agents/realtime_sim.py:31-32,84-85`）—— **用全局可变状态表达安全边距**，不可重入。
* 曾有非圆碰撞体的设计但**未启用**：`HexagenCollider` 整个类被注释（`bullet2player.py:38-98`）。所有弹型的“判定形状”实际都是圆。
* 出界回收用弹型相关的矩形范围 `get_out_range`（`bullet_enums.py:53-62`），不是统一边界。

### 4.4 弹幕（danmaku）表示：**它是“每帧调用 API 的记录”，不是声明式布局**

这是理解整个项目（以及 GAN 那一章）的钥匙。存在**两条语义等价但用途不同的路径**：

**路径 A：手写类（`data/code/*.py` + `logic/danmaku/basis.py`）**

* `Danmaku`（`basis.py:64-97`）：持有 `cnt`（帧计数）、`timeout`（默认 1500）、`shooters`（`BufferedLazyList`）。每帧 `update()` = `finish()` 检查 → `act()` → `shooters.update()`（`basis.py:73-87`）。
* **时间线 = `act()` 里的取模判断**，绝大多数弹幕就是 `if self.cnt % itv == 0: 发射`（`data/code/data1.py:24-37`）。没有独立的时间线数据结构，**时间线就是 Python 控制流**。
* `Shooter`/`PeriodicShooter`（`basis.py:16-61`）：`delay` 倒计时 + `enabled()` 的 `cnt % interval == 0` + `finish()`/`credit` 次数上限。`PlayerPosKeeper`（`basis.py:6-13`）保存“创建瞬间的玩家位置”并每帧 `cnt+=1`，实现**延迟自机狙**（“打 N 帧前玩家所在处”，见 §5.1 snipe 编码）。
* 3 个通用发射器：`SpiningShooter`（`common_shooters.py:7-48`，旋转臂，支持 `speed_range` 均分、`clr_sheme` 轮换配色、`credit` 次数）、`FakeLaser`（`common_shooters.py:51-62`）、`TrapezoidShooter`（`common_shooters.py:65-84`，梯形扩散，`ways` 逐次 +1）。

**路径 B：编码/解码（`bullet_creation.py` + `cm_danmaku.py`）**

* `BatchedBulletBuilder._default_kwargs`（`bullet_creation.py:241-245`）定义 14 个语义参数：`rho, theta, angle, speed, burst, decay, radius, btype, color, strlen, bend, ways, span, snipe`。
* `EncodingBatchedBulletBuilder.encoding_scheme`（`bullet_creation.py:406-421`）= 上面 14 个 + 第 15 列 **delay**，每列带 `range` 与 `dtype`；线性映射到 `vrange=(0.2, 0.8)`（`bullet_creation.py:423,471`）。**这就是数据集张量的 15 个通道**（§5.1）。
* 几何展开器（`bullet_creation.py:29-80`）决定“一次 API 调用生成几颗弹、在哪、朝哪”：
  * `circle_cfgs`：`ways` 颗均布 360°，`angle_diff = 360/ways`；
  * `sector_cfgs`：`span` 扇形，`ways>1` 时首尾含端点；
  * `string_cfgs`：`strlen≥10` 时把弹排成**直线或圆弧**（内角 ≤1° 走直线，否则按弦半径 `R = l/sin(α/2)` 走弧）。
  * 三者的选择逻辑：`strlen ≥ 10 → string`；`not (0.1 < span ≤ 340) → circle`；否则 `sector`（`bullet_creation.py:277-282`，预演版同逻辑 `354-359`）。
* 另有 `logic/danmaku/configurations.py` 一套几乎重复的实现（`sectoral_angles/circular_configs/chord`），但**只在少数手写弹幕里被调用，没被 builder 复用**（`data/code/data3.py:7`、`data4.py:7`、`data7.py:4`）→ 两套几何代码并存。
* 解码：`decode_param_seq`（`cm_danmaku.py:20-56`）做逆映射：枚举列用 `round((v - 0.5/k)*k)` 还原 id；数值列先 `bound(v, 0.2, 0.8)` **再**映射回物理区间；snipe 列用三段阈值 `<0.15→False`、`<0.25→True`、`≥0.25→按 (0.25,0.8) 反解“回溯帧数”`；delay 列 `<0.2 → 0 帧`，否则 `(0.2,0.8)→(0,150)` 帧。
* 播放：`BatchedCMDanmaku.act`（`cm_danmaku.py:106-119`）把序列 `reverse()` 后当作**延迟倒计时栈**：`while data[-1][-1] == 0: pop 并 create`，然后给新的栈顶 `-1`。`load()` 顺带把 `timeout` 设为 `sum(delay)+180`（`cm_danmaku.py:124-137`）。
* **新事件/新发射器当帧不生效**：`BatchedBulletBuilder` 的合并语义（§4.2）导致 `act()` 里 `shooters.add(...)` 的发射器**下一帧才第一次开火**。这一点我用编码数据反向验证过（见 §5.1 的 D-index 映射）：例如 `data1.D5` 在 `cnt=0` 时 add 发射器，其第一条编码记录出现在第 2 帧。

**34 个现成弹幕的归类**（`data/code/data{1,2,3,4,6,7}.py`，共 34 个 `class Dn(Danmaku)`，无 `data5.py`；`grep -c '^class .*(Danmaku)'` 已核验 = 34）

| 家族 | 机制特征 | 代表 |
|---|---|---|
| 旋转臂/螺旋 | `SpiningShooter(ad=…)` 逐次偏转，正反双旋 | `data1.D2`(双旋臂+狙击)、`data1.D7`(半径环正反双旋)、`data2.D5`(四角 decay 循环) |
| 加速/变速（burst+decay） | `burst` 初速增量 + `decay` 衰减系数 | `data1.D1`(巨玉)、`data3.D1`(御札)、`data6.D1`(梯形加速) |
| 几何批量 | `ways/span/radius/bend/strlen` 组合 | `data1.D4`(变长弦 `strlen`)、`data4.D4`(递增 ways)、`data4.D6`(同心环扩张) |
| 自机狙（snipe） | `snipe=True` 瞬时 / `PlayerPosKeeper` 延迟 | `data1.D2`、`data7.D5`(延迟狙击)、`data2.D3`(MovingCircle 自带狙) |
| 自定义发射器 | 各自文件内定义 | `data3.SpiningStar`（`data3.py:53-81`）、`data7.SegmentShooter`（`data7.py:8-25`）、`data4.D6.MovingCircle`（`data4.py:144-162`） |
| 时间线编舞（`act()` 复杂取模/异或） | 多相位、往复、颜色轮换 | `data1.D5`(动态 add 发射器)、`data2.D6`(往复扇形)、`data3.D4`(递增波)、`data4.D5`、`data6.D2`(双周期错相) |
| 弹型专项 | OFUDA/KUNAI/STAR/DROP/SQUAMA/DAGGER/ELLIPSE/R_JADE/B_STAR | `data1.D6`(刀+苦无)、`data6.D6`(御札半径振荡)、`data7.D1`(星形分段)、`data7.D3`(弦+大星)、`data3.D2`(飞刀) |

**D 编号 ↔ 类的映射（重要且易踩坑）**：`get_classes`（`data_make.py:50-61`）用 `os.walk` + `importlib.import_module` + `inspect.getmembers`，落盘时按遍历顺序命名为 `D0..D33`。我用**纯 Python 解码每个 `.npy` 的首条记录**（btype/color），与各类第一个 `create()` 实参逐一比对，得到**唯一自洽的映射**：

| npy | 对应源文件 | 数量 |
|---|---|---|
| `D0–D6` | `data/code/data1.py`（D1–D7） | 7 |
| `D7–D12` | `data/code/data6.py`（D1–D6） | 6 |
| `D13–D17` | `data/code/data7.py`（D1–D5） | 5 |
| `D18–D23` | `data/code/data2.py`（D1–D6） | 6 |
| `D24–D27` | `data/code/data3.py`（D1–D4） | 4 |
| `D28–D33` | `data/code/data4.py`（D1–D6） | 6 |

顺序是 `data1 → data6 → data7 → data2 → data3 → data4`（既非字典序也非文件名序，是 `os.walk` 的目录项顺序）。**这是复现风险**：换文件系统/换平台，`D0..D33` 与类的对应关系可能改变（§7）。

### 4.5 渲染：9 图层 + 每子弹一个 sprite + **每帧实时 rotate，无脏矩形**

* `FullRenderer` 把 `pygame.sprite.Group` 按 `Layers` 枚举开 9 个组（`renderer.py:9-23`），`update()`/`draw()` 就是按序 `group.update()` / `group.draw(tar)`（`renderer.py:25-31`）。**每帧全量重绘 9 层，没有脏矩形、没有 surface 缓存、没有 `blits()` 批量接口**。图层顺序（`renderer.py:9-19`）：背景 → 自机子弹 → Boss → 玩家 → 巨/大/中/小弹 → 判定点；**按弹型分 4 层**是为了让大弹盖住小弹。
* 窗口：`display.set_mode((640,480))`，先 blit `assets/board.png` 作边框，再 `screen.subsurface(32,16,384,448)` 作为实际场地（`game.py:108-110`）；`FullRenderer.size=(384,448)`（`renderer.py:7`）。
* 贴图：`TexManager` 单例，把 4 张精灵表切成 `[弹型][颜色]` 二维元组：tiny 6×8px、mid 6×9（16px）、large 6×4（32px）、giant 6×1（64px）（`utils/assets_manage.py:54-69`）。PNG 实际尺寸已用纯 Python 解析校验：`tiny.png 48x8`、`mid.png 96x144`、`large.png 192x128`、`giant.png 384x128`、`players/body.png 256x144`（8×3 格 32×48）—— **与切图代码完全一致**。
* **性能陷阱**：`BaseSprite.set_angle` 每次都调用 `pygame.transform.rotate(tex, angle)`（`utils/graphics.py:14-18`）。`CommonSprite.update` 对 `spinable`（星形/大星，`bullet_creation.py:124`）与所有非圆弹型**每帧重新旋转**（`common_sprites.py:12-16`）；`PlayerHitBox` 甚至每帧调两次 `set_angle`（`player_sprites.py:71-77`）。没有“按角度缓存旋转结果”，子弹变多时开销是 **O(N) 次 surface 旋转/帧**（能撑多少颗**未实测**）。隐藏 sprite 用 1×1 的 `assets/void.png` 顶替 image（`graphics.py:26-32`、`root.py:6`）。
* 逻辑层**没有**做到与渲染层解耦：`logic/runtime/bullet_creation.py:8-9`、`logic/runtime/obj_creation.py:7-10`、`logic/objects/bullet_enums.py:2` 都在模块顶层 import 渲染模块；`root.py:2,6` 更是在 import 期就 `image.load`。所以“headless 跑仿真”也必须装上 pygame。

---

## 5. GAN 弹幕生成（本项目的核心价值）

### 5.1 数据从哪来：把“API 调用轨迹”编码成 15×(时间) 的张量

**采集/编码流程**（`data_make.py`，入口 `run_make.py:4`）：

```python
# data_make.py:50-74（节选）
def get_classes(path):                       # 遍历 data/code/*.py，找出所有基类名为 Danmaku 的类
    ... importlib.import_module('data.code.' + fname[:-3])
        if cls.__base__.__name__ == 'Danmaku': res.append(cls)
def codes_to_param_seq(path, length=500, onehot=False, out_path='data/mat'):
    for cls in classes:
        param_seq = DanmakuEncoder.inst().class_to_ps(cls, onehot=onehot, L=length)
        np.save(out_path + '/D%d' % cnt, param_seq)
# data_make.py:19-42（核心）
def class_to_ps(self, cls, L=200, augmentation=False, onehot=False):
    danmaku = cls(augmentation=True) if augmentation else cls()
    danmaku.timeout = 100000                       # 关掉自动结束，保证能采到 L 条
    while self.builder.api_cnt < L:                # 以“API 调用次数”为计量单位，不是帧数
        danmaku.update(); self.builder.update()
    param_seq = np.array(self.builder.collect()[:L])
    param_seq[:, 0] = (param_seq[:, 0] + 0.5) / 15  # btype  -> 中点归一化
    param_seq[:, 1] = (param_seq[:, 1] + 0.5) / 6   # color  -> 中点归一化
```

* **张量语义（15 列）**＝`encoding_scheme` 的 14 个 key 顺序 + delay（`bullet_creation.py:406-421`）：`[btype, color, rho, theta, angle, speed, burst, decay, radius, strlen, bend, ways, span, snipe, delay]`。
* **每一行 = 一次 builder 调用**（一次“发射事件”，可含多颗子弹）；因此这是**发射器调用序列**，不是“画面里子弹的坐标快照”。屏幕上的弹幕布局是**回放这条序列后由物理积分涌现**的。
* **归一化**：列 0/1 用 `(id+0.5)/k` 的**中点编码**（`data_make.py:30-31`）；列 2–13 用 `linear_mapping(range, (0.2,0.8), v)`；delay 列：本帧发射→`0.1`，否则 `linear_mapping((0,150),(0.2,0.8),cnt)`（`bullet_creation.py:474-480`）。所以数据值域是 `[0.0333, 0.9667]`，且绝大多数落在 `[0.2,0.8]`。
* **snipe 语义编码**（`bullet_creation.py:441-455`）：`False→0.1`、`True→0.2`、`int n>0` 与 `PlayerPosKeeper` → `linear_mapping((0,100),(0.25,0.8),cnt)`，即“**瞄准 n 帧前玩家所在位置**”。这是把“预判/延迟狙击”参数化的关键设计。
* **增强（augmentation）**：`data/code/augmentation.py` 用 `bounded_gauss(sigma=0.05*val)`（`augmentation.py:5-6`）对参数加噪、`transform_dicts` 对 dict 里所有数值加噪（`augmentation.py:18-25`），弹幕类在 `augmentation=True` 时还会随机化初相/颜色顺序（如 `data1.py:26-29`）。`AugmentedDataset` 每次取样本都**现场重算**（§5.2），所以同一 epoch 内每个样本都被重新增强一次。
* **one-hot 变体（34 维）**：`class_to_ps(onehot=True)` 把 btype 展开成 15 维、color 6 维，其余 13 维保留 → 15+6+13=34（`data_make.py:32-41`）。**但仓库里没有任何训练脚本传入 one_hot/onehot**（`grep` 核验：只有 `dataset.py:46` 的形参与 `data_make.py:19` 的形参存在）→ **这条 34 维路径实际未被使用**（它本来是给 TimeGAN 的 `MixedReconstructor` 准备的，见 §5.5）。

### 5.2 `generator/dataset.py`：数据集两个类，实际只用了一个

| 类 | 位置 | 数据来源 | 形状 | 是否被使用 |
|---|---|---|---|---|
| `DanmakuDataset` | `dataset.py:8-42` | 直接读 `path/D0.npy, D1.npy...` 直到 `FileNotFoundError`（`dataset.py:13-24`） | `datalen` 截断；`transpose=False→(T,15)`，`True→(15,T)` | **死代码**：三个 train.py 都 import 的是 `AugmentedDataset` |
| `AugmentedDataset` | `dataset.py:45-71` | `get_classes(path)` 拿到类，**取样本时才调用 `class_to_ps(..., augmentation=True)`** 现场编码 | `transpose=False→(64,15)`；`DCGAN/PSGAN` 传 `transpose=True→(15,64)` | 三个 GAN 全用它 |

* `DanmakuDataset.__getattr__`（`dataset.py:26-30`）返回 `m=length`、`n_fea=15`；返回非 `AttributeError` 的写法有副作用（拼错的属性返回 `None` 而不报错）。`AugmentedDataset.__getitem__` 同时支持 `int`（单样本）与 `slice`/`list`（批量，`torch.stack`，`dataset.py:38-42,57-71`）；DCGAN/PSGAN 用 `dataset[s:s+n]` 取批（`generator/DCGAN/train.py:42`），TimeGAN 用 `DataLoader`。
* **关键复现性问题**：此处的增强走全局 `random`（`data/code/*.py` 里 `from random import uniform, shuffle`），而**全仓库没有任何 `seed()` 调用**（已 `grep -rn "seed"`，结果为空）→ 数据集内容不可复现。

### 5.3 DCGAN：把 64 步参数序列当成 (15×64) 的“图”来生成

**文件**：`generator/DCGAN/models.py`、`generator/DCGAN/train.py`。

**张量形状**

| 张量 | 形状 | 说明 |
|---|---|---|
| `z`（噪声） | `(n, 64, 1)` | `torch.randn`（`train.py:54`）；`nz=64`，长度维=1 |
| 生成样本 `G(z)` | `(n, 15, 64)` | 15 列参数 × 64 个时间步（`models.py:19`） |
| 真实样本 | `(n, 15, 64)` | `AugmentedDataset(datalen=64, transpose=True)`（`train.py:105`） |
| `D(x)` | `(n, 1, 1)` | Sigmoid 标量（`models.py:34`） |

**生成器**（`models.py:11-23`，`nz=64`, `ncf=32`）：5 层 `ConvTranspose1d`，`BatchNorm1d` + `ReLU`，末层 `Sigmoid`：

| # | 层 | 输出长度 |
|---|---|---|
| 1 | `ConvTranspose1d(64→256, k=4, s=1, p=0)` + BN + ReLU | 4 |
| 2 | `ConvTranspose1d(256→128, k=4, s=2, p=1)` + BN + ReLU | 8 |
| 3 | `ConvTranspose1d(128→64, k=4, s=2, p=1)` + BN + ReLU | 16 |
| 4 | `ConvTranspose1d(64→32, k=4, s=2, p=1)` + BN + ReLU | 32 |
| 5 | `ConvTranspose1d(32→15, k=4, s=2, p=1)` + **Sigmoid** | 64 |

**判别器**（`models.py:26-38`，`nci=32`）：`Conv1d(15→32,k4,s2,p1)+BN+ReLU` → 32 → 16 → 8 → 4 → `Conv1d(256→1,k4,s1,p0)+Sigmoid` → 1。**判别器里也放了 BatchNorm**（`models.py:30-33`），与原始 DCGAN 论文建议相悖（判别器用 BN 会让真实/假样本的批统计互相污染），属于实现选择而非笔误。

**损失 / 优化器 / 超参**（`train.py:26-28`，参数默认值 `train.py:112-130`）

| 项 | 值 | 来源 |
|---|---|---|
| 损失 | `nn.BCELoss()` 两次（`D`: real→1, fake→0；`G`: fake→1） | `train.py:26,48,57,69` |
| 优化器 | `Adam(lr=2e-4)`，G/D 各一个 | `train.py:27-28` |
| 批大小 | 12 | `train.py:116-117` |
| epoch | 5000 | `train.py:114-115` |
| G 更新次数/批 | **10**（`g_credits`） | `train.py:125-126` |
| D 更新次数/批 | 1（`d_credits`） | `train.py:125-126` |
| `nz` / 通道基数 | 64 / 32（`min_channels`） | `train.py:127-128` |
| 数据类型 | `torch.float64` | `train.py:102` |
| 采样/保存 | 每 100 epoch（`save_interval`）存 50 个样本到 `result/samples/iterationN/`；每 50 epoch 存权重 | `train.py:81-93` |

**生成的是什么**：一条 **64 步的“发射器 API 调用序列”**（无 one-hot，直接 15 通道连续值）。它不是单帧弹幕布局图。保存时 `item.transpose()` 把 `(15,64)` 变回 `(64,15)`（`train.py:89-91`）—— **正好是 `BatchedCMDanmaku` 能直接吃的格式**，丢进 `danmakus/` 就能当一条弹幕跑（长度只有 64 个事件，比真实数据 500 短）。

### 5.4 PeriodicSpatialGAN：给潜变量加“周期结构”，为什么可能有用

**文件**：`generator/PeriodicSpatialGAN/models.py`、`train.py`。

**动机（从代码可验证的部分）**：弹幕是**周期/节律性**的（旋转臂、往复扫、间歇发射），而 DCGAN 的噪声在长度维只有 1 个采样点（`(n,64,1)`），必须让转置卷积自己“长出”节律。PSGAN 改为**先造一个显式的时间信号**：

```python
# generator/PeriodicSpatialGAN/models.py:25-38
def forward(self, zg, zl=None, device='cpu'):
    batch_size, _, length = zg.shape              # zg: (n, 64, L)
    mid = self.zp_mid(zg)                         # Conv1d(64→64,1,1)+ReLU
    k   = self.k_gen(mid)                         # Conv1d(64→nzp,1,1)  频率
    phi = self.phi_gen(mid)                       # Conv1d(64→nzp,1,1)  相位
    time[:, :, t] = t for t in range(length)
    zp = torch.sin(k * time + phi)                # 周期潜码 (n, nzp, L)
    z = torch.cat([zg, zp], dim=1)                # (n, 64+16, L)
    return self.convs(z)                          # → (n, 15, 64)
```

* **输入**：`zg = rand(n, 64, 1).expand(-1,-1,L)`（**全局噪声在时间上不变**，`train.py:26`）+ 可选 `zl = rand(n, nzl, L)`（逐帧局部噪声，默认 `nzl=0` 即关闭，`train.py:27,143`）。**频率/相位是逐时间步的**：`k`、`phi` 由 1×1 卷积作用在整条 `zg` 上得到（形状 `(n,16,L)`），所以 `sin(k·t+phi)` 在时间上是调频信号，而 `k/phi` 又是从噪声学出来的 → **“周期结构由网络自己参数化”**。
* **输出**：`(n, 15, 64)`，与 DCGAN 同形。长度可核验：输入 `L=10`（`--lz 10` 默认，`train.py:144`）经 4 层转置卷积 `k4`（无 padding）后为 `4L+24 = 64` —— **与 `datalen=64` 精确吻合**，不是 bug（按 `L_out=(L_in-1)*s+k` 逐层验算）。`models.py:54-57` 的 `__main__` 也会打印 `G(zg).shape`。
* **网络结构**（`models.py:11-52`，`ncf=64`）：生成器 = 两个 1×1 `Conv1d` 造 `k/phi`，主干 `ConvTranspose1d(80→256,k4,s1)+ReLU → (256→128,k4,s2)+ReLU → (128→64,k4,s1)+ReLU → (64→15,k4,s2)+Sigmoid`，**全程无 BatchNorm**（与 DCGAN 的差别）；判别器 = `Conv1d(15→128,k4,s1)+ReLU → (128→64,k4,s2)+ReLU → (64→32,k4,s1)+ReLU → (32→1,k4,s2)+Sigmoid`，**也无 BN**。
* **损失**：只用 `BCELoss`（`train.py:45,62,71,83`）。**`MeanVarLoss` 类在 `train.py:31-39` 定义了却没被实例化或调用**（`grep -rn "MeanVarLoss"` 只命中定义处与 TimeGAN）—— 而代码里还留着注释 `# Matching mean and variance loss to overcome mode collapse`（`train.py:88`）。**即：防模式崩溃的机制写好了但没接上**，这是本项目最值得注意的“坑”。
* **超参**（`train.py:127-147`）：`n_epochs=5000`、`batch_size=12`、`lr=2e-4`、`g_credits=2`（G 每批 2 次）、`nzg=64`、`nzp=16`、`nzl=0`、`lz=10`、`min_channels=64`、`float64`。优化器 `Adam`。
* **潜在 bug（可静态确认）**：`--nzl > 0` 时 `z = cat([zg, zp, zl], dim=1)` 会让第一层卷积的输入通道变成 `80+nzl`，而该层固定为 `nzg+nzp=80`（`models.py:19`）→ **维度不匹配报错**。所以“局部噪声”这条路在当前代码里不可用（TimeGAN 则正确地把 `nzl` 计入了 LSTM 的 `in_channels`，`generator/TimeGAN/models.py:52`）。
* **“spatial”指什么**：从代码看，没有任何显式的空间维度/空间卷积（数据是 15×时间）。命名中的 “spatial” 依据**未确认**（可能来自论文对“弹幕序列在参数空间展开”的表述）；本报告只描述可验证的事实。

### 5.5 TimeGAN：5 个网络的时序生成（自编码 + 监督 + 对抗 + 均值方差）

**文件**：`generator/TimeGAN/models.py`、`train.py`。数据形状 `(n, 64, 15)`（`batch_first=True`，`AugmentedDataset(..., datalen=64)` 不转置）。

**5 个网络**（`generator/TimeGAN/models.py:5-11`）

| 角色 | 定义 | 输入→输出 | 备注 |
|---|---|---|---|
| Embedder `E` | `LSTM(15, nhidden, 24, n_stack)` | `(n,64,15)` → `(n,64,24)` | 隐空间 `h_channels=24` |
| Reconstructor `R` | `LSTM(24, nhidden, 15, n_stack)` | `(n,64,24)` → `(n,64,15)` | 输出 Sigmoid ∈(0,1) |
| Generator `G` | `PeriodicGenerator(64, 0, 16, nhidden, 24, n_stack)` | 噪声 `(n,64,64)` → `(n,64,24)` | 复用 PSGAN 的 sin 潜码 |
| Predictor `P` | `LSTM(24, nhidden, 24, n_stack)` | `(n,64,24)` → `(n,64,24)` | 隐空间一步预测 |
| Discriminator `D` | `LSTM(24, nhidden, 1, n_stack)` | `(n,64,24)` → `(n,64,1)` | Sigmoid，配合 BCE |

* `LSTM` 模块 = `nn.LSTM(n_in, n_hidden, num_layers, batch_first=True)` + `Linear(n_hidden, n_out)` + **Sigmoid**（`models.py:14-26`）。**所有** LSTM 头都带 Sigmoid（包括 D 和 E）——这是“输出必须落在 (0,1) 以匹配归一化数据”的统一约定。
* `PeriodicGenerator`（`models.py:45-67`）：`zp_mid = Linear(64→16)+Sigmoid`，`k_gen/phi_gen = Linear(16→16)`，`zp = sin(k·t+phi)`，与 `[zg, zl, zp]` 拼接后送入 LSTM（`nzg+nzl+nzp` 通道，`nzl` 被正确处理）。
* **`MixedReconstructor`（`models.py:29-42`）是死代码**：它按 `btype=15(Softmax) + color=6(Softmax) + numerical=13(Sigmoid) = 34` 输出设计 —— **正是 one-hot 编码的形状**（§5.1），但 `get_models` 里根本没有使用它（`get_models` 用的是 `LSTM(24,·,15)` 作 R，`models.py:10`），且 train.py 也没有 one-hot 开关。→ **34 维 one-hot 分支在 TimeGAN 里同样不可达**。

**三段式训练流程**（`train.py:42-214`）

| 阶段 | 做什么 | 损失 | epoch 默认 | 优化器 |
|---|---|---|---|---|
| ① 预训练 E,R | `x_hat = R(E(x))` 重建 | `MSE(x, x_hat)` | `r_epoches=5000`，DataLoader `batch_size//2=6`，`shuffle=True` | `RMSprop(lr=2e-3)` |
| ② 预训练 P | 隐空间一步预测 `h_fake = P(h_real)`，目标 `h_real[1:]` vs `h_fake[:-1]` | `MSE`（监督损失） | `s_epoches=500`，`batch_size=12` | `RMSprop` |
| ③ 联合训练 | D / E,R / G,P 交替 | 见下 | `j_epoches=5000` | `RMSprop`，**D 额外 `weight_decay=1e-4`** |

联合阶段的损失组合（`train.py:110-177`）：

* **D**：`BCE(D(h_real),1) + BCE(D(G(z)),0) + BCE(D(P(G(z))),0)`，三项取平均后累加（`train.py:124-131`）。
* **E,R**：`MSE(x, R(E(x))) × 10` 的梯度 + `MSE(h_real[1:], P(h_real)[:-1]) × 0.1` 的梯度（`train.py:140-144`）。
* **G,P**：
  * 监督项 `s_loss = MSE(h_real[1:], P(h_real)[:-1]) × 10`，再乘 100 反传（**等效权重 1000**，`train.py:154-156`）；
  * 对抗项 `u_loss = BCE(D(G(z)),1) + BCE(D(P(G(z))),1)`（`train.py:164-167`）；
  * 均值方差项 `MV = L1(sqrt(var)) + L1(mean)`（`MeanVarLoss`，`train.py:17-25`）对 `E(x)` 与 `P(G(z))` 求，再乘 100 反传（`train.py:169-175`）—— **TimeGAN 里这个防模式崩溃损失是真接上的**（与 PSGAN 不同）。
* **生成/采样**：`sampling = R(P(G(zg, zl)))`（`train.py:35-40`），即“**先隐空间生成+推进，再解码**”。每 100 epoch 存 50 个样本为 `original_i.npy`（`train.py:203-208`），同时 `torch.save` **整个模块对象**（`Reconstructor.pkl / Generator.pkl / Predictor.pkl`，`train.py:197-199`）。
* **超参**（`train.py:230-248`）：`lr=2e-3`、`batch_size=12`、`nzg=64`、`nzp=16`、`nzl=0`、`seqlen=64`、`d_credits=1`、`g_credits=1`、`nhidden=128`、`h_channels=24`、`nstack=3`、`float64`。

**静态可确认的实现缺陷（重要）**

1. **判别器的损失从未反传**：`train.py:117-132` 的 D 分支里只有 `D.zero_grad()` → 计算三个 BCE → `.item()` 记录 → **直接 `optD.step()`**，中间**没有任何 `backward()`**（`grep -n "backward()"` 结果里 118–132 行之间为空）。因此 D 的判别梯度永远为 0；只有在下一批 `D.zero_grad()` 之前，G 的 `u_loss.backward()`（第 167 行）会给 D 累积梯度，但那批梯度在下一次 `optD.step()` 之前已被清零。**净效果：D 只被 RMSprop 的 `weight_decay=1e-4` 收缩，从未被分类损失训练**。这几乎肯定是笔误（对比 ① ② 阶段都规规矩矩写了 `backward()`）。
2. E 的梯度在联合阶段**没有在 G,P 分支前置零**（第 134-135 行只在 E,R 分支清零），所以 `s_loss` 流回 E 的梯度会跨迭代累积（不致命，但语义混乱）。
3. `with open('loss.json','w')` 用**相对路径**（`train.py:201`），会写到当前工作目录而不是 `--result_path` —— 与各 GAN 在 `result_path` 下写样本/权重的约定不一致。
4. `torch.save(R, ...)` 存的是**整模块**（含设备信息），在 GPU 上训练后到 CPU 加载需要额外处理（`map_location`）。

### 5.6 生成结果如何被游戏消费：**pipeline 是“人工搬运”，没有自动闭环**

可验证的链路：

```
[训练] train.py → result/samples/iterationN/sample_i.npy   shape (64,15)   float32/float64
        │  （DCGAN: train.py:85-91；PSGAN: train.py:98-107；TimeGAN: train.py:203-208）
        ▼  手工复制（仓库里没有任何脚本做这一步）
[游戏] danmakus/*.npy
        │  Boss.__init__ 用 os.walk 收集 danmakus/ 下所有文件 → Boss.spells   (logic/objects/boss.py:19-22)
        ▼  Boss.load 判类型：str → BatchedCMDanmaku().load_file(fname)         (boss.py:56-63)
      BatchedCMDanmaku.load: np.load → tolist → reverse → decode_param_seq → timeout=Σdelay+180
                                                                            (cm_danmaku.py:121-137)
        ▼
      act(): 每帧看栈顶 delay 是否为 0，是则 pop 并 BatchedBulletBuilder.instance.create(**kwparas)
                                                                            (cm_danmaku.py:106-119)
        ▼
      RuntimeBatchedBulletBuilder.create → 几何展开 → 每颗 Bullet(...) → objs.bullets
                                                                            (bullet_creation.py:262-327)
```

* **生成样本可直接落进 `danmakus/`**：三个 train.py 保存前都做了 `.transpose()`，把 `(15,64)` 还原成 `(64,15)`，与 `BatchedCMDanmaku` 读取的 15 列格式一致。但 **`data_make.py` 的流水线只负责“真实数据 → 编码”，不负责“生成 → 游戏”**：`codes_to_param_seq` 扫的是 `data/code/*.py` 的**类**（`data_make.py:63-74`），生成结果不在其中 → “GAN 产出接入游戏”目前是**纯手工**步骤。
* **生成值会先被裁剪再解码**：`decode_param_seq` 对数值列做 `bound(x, 0.2, 0.8)`（`cm_danmaku.py:47`），snipe 用三段阈值，delay <0.2 视为 0 帧（`cm_danmaku.py:51-56`）。这只是一层**隐式的合法域钳制**，**不做任何合理性/安全性检查**（同一帧生成 300 颗子弹是完全可能的）。权重与样本**都不在仓库里**（无 `result/`、无 `.pkl`、无 `loss.json`）→ 想复现生成必须自己重训，而重训不可复现（§7）。

### 5.7 如何评估生成质量：可视化只有一张热力图；真正的评估器在 `game.py` 里、且**没和 GAN 接上**

* `generator/visualization.py:6-19` 的 `viz_paramseq(data)` 只做一件事：把 `(T,15)` 转置后画 seaborn 热力图，`vmin=0,vmax=1`，y 轴标签就是那 15 个参数名（`visualization.py:14-17`）。**没有 FID/IS、没有分布距离、没有任何数值指标**；`__main__` 里硬编码读 `../data/example.npy`（相对 cwd）。
* 项目真正的“弹幕质量评估”写在 `game.py` 里，共 3 个类：
  * `Monitor`（`game.py:173-206`）：把场地按 16px 划格，逐帧累计 `weight`；输出 `entropy = Σ|cur_w − prev_w| / Σweight`（归一化的**帧间密度变化量**）与 `max_momentum = max Σ(weight×speed)`。
  * `FeatureBaseMetricEvaluator`（`game.py:209-267`）：给定**已编码文件名**（必须是 `str`，否则 `TypeError`，`game.py:211-212`），headless 跑完整条弹幕，输出 5 个指标：
    | key | 含义 | 公式 |
    |---|---|---|
    | `SF` | 发射频率 | `L / T`（事件数 / 总帧数，`game.py:232-233`） |
    | `EFR` | 持续发射比例 | `EF / T`，`EF = Σ max(0, delay−1)`（`cm_danmaku.py:133-138`） |
    | `MM` | 平均动量 | `Σ(speed×weight) / min(T, cnt)`（`game.py:244,249`） |
    | `DE` | 密度变化熵 | `Σ|w_map1 − w_map0| / (帧数×格子数)`（`game.py:245,250`） |
    | `C` | 覆盖率 | 大弹覆盖格数 / 总格数（`game.py:251,254-267`） |
  * `AgentBaseMetricEvaluator`（`game.py:270-317`）：让 agent 实际打一遍，输出 `Playable`（是否通关）、`Entropy`、`Risk`（采样帧中“距离某颗弹 ≤ 10√weight+4”的比例）。
* **没有任何脚本把 GAN 的输出喂给这些评估器**（`grep` 遍历 `generator/`，无一 import `game`）→ “生成质量”在当前仓库里**没有闭环**，只能靠人眼看热力图。
* 顺带指出该评估器的两个实现瑕疵：方法名拼写为 `evluate`（`game.py:228`，少一个 `a`）；覆盖面统计里 `self.cover_map[i:i, j:j, l:].sum()` 用了空切片 `i:i`（`game.py:262`），条件恒为 0/假，实际覆盖面由随后的矩形写入决定，且 `l`（尺寸等级，0/1/3）被当成 3 通道数组的通道下标，越界靠 `try/except IndexError` 兜住（`game.py:264-267`）。

### 5.8 三种 GAN 的对比与各自的坑

| 维度 | DCGAN | PeriodicSpatialGAN | TimeGAN |
|---|---|---|---|
| 生成对象 | `(15,64)` 参数序列 | `(15,64)` 参数序列 | `(64,15)` 序列（隐空间生成，R 解码） |
| 噪声结构 | `randn(n,64,1)` 无时间结构 | `rand` 全局 + **sin(k·t+φ) 周期潜码** | `rand` 全局 + 周期潜码，再喂 LSTM |
| 主干 | 5 层 ConvTranspose1d（有 BN） | 4 层 ConvTranspose1d（无 BN） | 3 层 LSTM + Linear（nhidden=128） |
| 判别器 | Conv1d 栈 + BN（有污染风险） | Conv1d 栈（无 BN） | LSTM + Sigmoid |
| 损失 | BCE | BCE（**MV 损失写了没用**） | 重建 MSE + 监督 MSE + 对抗 BCE + **均值方差** |
| 训练结构 | 单阶段 | 单阶段 | **三段式**（自编码 → 监督 → 联合） |
| 优化器/lr | Adam 2e-4 | Adam 2e-4 | **RMSprop 2e-3**（D 带 weight_decay） |
| 可用输入长度 | 固定 64 | `L=10`（受 `--lz` 控制，`4L+24` 必须 =64） | 64 |
| **可确认的实现坑** | 判别器带 BN；`g_credits=10` 的不平衡未见解释 | `--nzl>0` 直接维度报错；MV 损失未启用 | **D 的损失缺 `backward()`**；`loss.json` 路径错；one-hot 分支不可达 |

**共有/常见的 GAN 风险（结合代码状态）**

1. **模式崩溃**：PSGAN 团队显然意识到了（写了 `MeanVarLoss` 与注释），但**忘了启用**；DCGAN 完全没有对应机制；只有 TimeGAN 真正用了均值方差损失。三者的“防崩溃”能力因此差别很大。
2. **时序一致性**：TimeGAN 的 `P` 在隐空间做一步预测，`sampling` 递归 `P(G(z))` —— 误差会在长序列上累积；而 `P` 只在一阶监督项上训练，代码里**没有任何 rollout 长度 > 1 的训练**。
3. **评估困难**：仓库没有 FID/IS/下游指标闭环（§5.7），只有热力图；`game.py` 的 5 个指标（SF/EFR/MM/DE/C）是任务相关启发式，无法区分“像真实弹幕”与“像训练集里的某一条”。
4. **数据量极小 + 不可复现**：只有 **34 条**弹幕类（增强后才成批），DCGAN/PSGAN 的 `batch_size=12`、`n_epochs=5000` 在这种规模下极易记忆化（`AugmentedDataset.__len__` 就是 34，`dataset.py:54-55`）；再叠加无 seed、无版本锁定、权重未入库（§7）。**生成能否带来超出训练集的多样性，本报告无法证实**（无实验产物）。

---

## 6. 关键数据结构与文件格式

已用纯 Python 解析 `.npy` 头（`magic + version + header_len + dict`）核验：

| 文件 | dtype | shape | C 序 | 载荷/文件大小 | 说明 |
|---|---|---|---|---|---|
| `data/example.npy` | `<f8` (float64) | `(64, 15)` | False | 7680 B / 7808 B | `data_make.py:78` 用 `ExampleDanmaku, L=64` 生成 |
| `data/mat/D0..D33.npy` | `<f8` | `(500, 15)` | False | 60000 B / 60128 B | `run_make.py` 默认 `length=500`，**非 one-hot** |
| `danmakus/D0..D33.npy` | `<f8` | `(500, 15)` | False | 60000 B / 60128 B | 游戏实际加载的弹幕 |

* **`data/mat/` 与 `danmakus/` 逐一 `cmp` 完全相同（34/34 无差异）**，md5 亦相同（如 `D0.npy = 42c80c89115956d4bb9fd4481285bdb6`）。即：`danmakus/` 是 `run_make.py` 产物的副本。
* 但 `.gitignore:1` 写着 `/danmakus/` —— **该目录被 git 忽略**。本地这份是导出快照（无 `.git`），所以内容在；若从源码 clone 后直接运行而没先跑 `run_make.py`，`Boss.spells` 可能为空（`boss.py:19-22` 会把空列表当“没有弹幕”）。**上游是否 force-add 过该目录：未确认**。
* 列语义（15 列，与 `encoding_scheme` 一一对应）：`0 btype(中点归一) / 1 color(中点归一) / 2 rho / 3 theta / 4 angle / 5 speed / 6 burst / 7 decay / 8 radius / 9 strlen / 10 bend / 11 ways / 12 span / 13 snipe / 14 delay`。全部是**归一化标量**，不是物理量；解码函数是 `decode_param_seq`（`cm_danmaku.py:20-56`）。
* **没有 `.pkl` / `.json` / `.npz` / `.csv` 数据文件**。运行时才产生的格式：训练产物 `result/.../model/*.pkl`（DCGAN/PSGAN 存 `state_dict`，TimeGAN 存整模块）与 `G_loss.json`/`D_loss.json`（`generator/DCGAN/train.py:95-98`）、截图 `screenshots/N.png`（`game.py:152`）。
* `assets/`：14 个 PNG，共约 104 KB（`du -sh`）。尺寸已逐个核验（见 §4.5），最大的是 `board.png 640x480`（外框）与 `background/bg.png 384x448`（场地）。
* 路径皆由 `root.py:5` 的 `PRJROOT`（`__file__` 所在目录 + `/`）拼接，**没有硬编码的绝对路径/盘符**（`grep -rnE "([A-Za-z]:\\\\|/home/|/Users/)"` 无命中）。但 `run_make.py:4`、`data_make.py:78`、`generator/visualization.py:23` 使用了**相对 cwd 的路径**，因此必须在特定工作目录下运行。

---

## 7. 测试与复现

* **没有任何测试**：无 `tests/`、无 `unittest`/`pytest` 引用、无 `assert`（已 `grep` 核验，结果为空）。`logic/danmaku/test_danmaku.py` 只是名为 Test 的弹幕类，不是测试用例。
* **没有任何随机种子管理**：`grep -rn "seed"` **零命中**（既无 `random.seed` 也无 `torch.manual_seed`/`np.random.seed`）→ 增强数据（`uniform/shuffle/gauss`，§5.1）、GAN 训练（`torch.randn`/`torch.rand`）、`RandomAgent`（`simple_agents.py:12,45`）全都不可复现。
* **仿真本身是确定性的**（给定输入时序）：逻辑中无随机（除 agent/增强），`objs.bullets` 是有序 list，速度按帧定步长积分。但 `Boss.__init__` 用 `os.walk` 收集文件（`boss.py:20-22`）→ **弹幕播放顺序依赖文件系统枚举顺序**。
* **“复现一次弹幕”的实际步骤**：① 在项目根目录 `python run_make.py`（把 `data/code/*.py` 的 34 个类编码进 `data/mat/`）→ ② 把 `data/mat/*.npy` 复制到 `danmakus/` → ③ `python run_game.py`（Boss 按 `os.walk` 顺序播放）。只复现某一条：把该 `Dn.npy` 单独留在 `danmakus/`，或按 `README.md:43` 在 `logic/objects/boss.py` 里 import 类塞进 `Boss.spells`。
* **端到端复现的三个断点**：① `D0..D33` 与类的对应关系由 `os.walk` 顺序决定（本文已实测出当前映射，见 §4.4，但换平台可能变）；② 增强路径无种子；③ 训练权重未入库。另外三个 train.py 都支持 `-h/--help`，我核验了**所有 CLI 参数都被真实使用**（正则比对 `add_argument` 与 `args.*`，无“定义未用/使用未定义”）。

---

## 8. 局限与坑（读完再说的清单）

**A. 法务/工程元数据**

1. **无 LICENSE（法律风险）** → `[LICENSE_REVIEW_REQUIRED]`（§1.1）。
2. 无 `requirements.txt`/`setup.py`/`pyproject.toml`；README 声明的版本（Python 3.7.6 / PyTorch 1.5.0）是 2021 年的，距今已久，**依赖能否装起来未实测**。本地 pycache 是 `cpython-312`，与实际声明不一致。
3. **文档几乎为零**：只有 57 行 README；README 承诺的 API 文档在外部 wiki（`README.md:5`），本仓库内没有。

**B. 逻辑与渲染未解耦**

4. `logic/` 顶层 import `render/`（`bullet_creation.py:8-9`、`obj_creation.py:7-10`、`bullet_enums.py:2`），`root.py:2,6` 在 import 期加载贴图 → **headless 也必须装 pygame**，“逻辑层可测性”被破坏。
5. `objs.py` 是模块级全局可变状态（含 `render` 标志）；`RealTimeSimulator` 通过临时改 `Collider.hr` 表达安全边距（`realtime_sim.py:31-32`）→ **不可重入、无法并行跑多个场景**。

**C. 性能**

6. 每颗子弹 = Python 对象 + `kwargs` 字典 + 一个 sprite；碰撞 O(N) 暴力；`pygame.transform.rotate` **每帧每弹**执行（§4.5）。**没有实测数据**，但结构上不具备支撑上万弹的能力（对比用户项目的 SoA+自由列表）；`BufferedLazyList` 的 `capacity` 上限分支从未启用（§4.2）。

**D. 代码质量（静态可确认）**

7. `Bullet.__getattr__`（`bullets.py:22-24`）、`PlayerBullet.__getattr__`（`bullets.py:136-138`）、`Player.__getattr__`（`player.py:71-73`）、`Vec2.__getattr__`（`math.py:142-150`）**对未知属性返回 `None` 而不是抛 `AttributeError`** → 拼写错误静默变成 `None`；`Boss.__getattr__` 的 `return self.pos`（`boss.py:65-69`）在 `pos` 未设置时会**无限递归**。
8. `RuntimeBulletBuilder.create` 读 `kwparas['speed']`（`bullet_creation.py:163`），但 `_default_kwargs`（`bullet_creation.py:90-93`）**没有 `speed` 键** → 不传 `speed` 就 `KeyError`。该 builder 已被 `RuntimeBatchedBulletBuilder` 取代（`game.py:115-116` 两个都建，但 `BatchedBulletBuilder.instance` 被后者覆盖）→ **潜伏 bug**。另外 `RuntimeBatchedBulletBuilder.create` 被标了 `@abstractmethod` 却直接被实例化调用（`bullet_creation.py:260`）。
9. TimeGAN 判别器损失缺 `backward()`（§5.5 缺陷 1）；PSGAN `MeanVarLoss` 未启用（§5.4）；`loss.json` 写错目录（§5.5 缺陷 3）。
10. 死代码/未用分支：`DanmakuDataset`、`CMDanmaku`+`convert_onehot`（10 列老格式，`grep` 只有自身文件引用）、`MixedReconstructor`、`FakeLaser`、`HexagenCollider`（全注释）、one-hot 34 维路径、`configurations.py` 与 `bullet_creation.py` 两套几何实现并存。
11. 未使用的 import：`logic/objects/player.py:1`、`logic/objects/weapons.py:1,4`；截图先保存后建目录导致第一张丢失（`game.py:151-155`）；`FeatureBaseMetricEvaluator` 方法名拼写 `evluate`（`game.py:228`）与 `i:i` 空切片（`game.py:262`）；PSGAN 保存权重文件名仍叫 `DCGAN_%d.pkl`（`generator/PeriodicSpatialGAN/train.py:97`）。

**E. “只在 Windows 可用”？**

12. **不成立**。没有平台特定 API、没有盘符路径、没有 dll 依赖；`os.path`/`os.walk` 跨平台。真实约束是 **pygame + cwd 相对路径**（§6）与**上游声明的旧版本依赖**。

---

## 9. 对用户项目的可借鉴点（面向“二维动态障碍环境模拟器”）

> 下面每条都对照用户现有设计（`bullet_sim/docs/DESIGN.md`、`bullet_sim/docs/OPEN_SOURCE_REFERENCES.md:11-40`）给出**具体到函数/字段**的建议，并明确哪些**不建议**照搬。

### 9.1 弹幕参数化与时间线调度 → “障碍事件 + 时间线”契约

* **Keiki 的实质**：`Danmaku.act()` 的 `cnt % itv`（`data/code/data1.py:24-37`）**就是**一条事件时间线，只是用 Python 控制流表达；编码器把它展开成“每行一次 `create()`，delay 列给出与上次发射的帧距”（`bullet_creation.py:474-480`），播放时再当倒计时栈回放（`cm_danmaku.py:106-119`）。
* **可直接借的抽象**：把 `SpawnTimeline` 定义成 **`(t, spawn_args)` 的稠密/稀疏表**，其中 `delay` 是“相对上一个事件”而不是绝对帧号 —— 这与你 `ObstacleSpawn(interval, start_time)`（DESIGN.md §7.5）语义一致，但 Keiki 的表是**完全扁平、可神经网络生成/可哈希**的。建议：为 `PatternSpec` 增加一个“**可序列化的事件表导出**”（`to_events()` → `np.ndarray(N, F)`），这样它能同时服务三件事：数据集落盘、差分对比、以及（若将来做生成）张量输入。
* **参数族映射**：Keiki 的 `circle_cfgs / sector_cfgs / string_cfgs`（`bullet_creation.py:29-80`）是三种**纯函数几何展开器**（环形均布 / 扇形 / 线-弧），正好对应你 `layouts` 里的“从边界进入的一组障碍”。其中 `string_cfgs` 的“内角 ≤1° 走直线，否则按弦半径 `R=l/sin(α/2)` 走弧”值得一提：**用单个 `span` 参数在直线与圆弧之间连续插值**，很适合你 `corridor` 的“通道弯曲”或 `wall_gap` 的“缺口沿弧线扫动”，且它是**解析式、零随机、可精确复现**的（符合你的确定性要求）。
* **延迟自机狙 = 可参数化的“预判难度”**：`PlayerPosKeeper`（`basis.py:6-13`）+ snipe 编码（`bullet_creation.py:441-455`）把“瞄准玩家 N 帧前的位置”变成**一个标量参数**（0.25–0.8 映射到 0–100 帧）。你的 `prediction/` 已有 rollout/危险场；建议把 `aim_lag_frames` 作为 `ObstacleSpawn` 的一个显式字段，而不是让障碍在生成期读实时玩家位置 —— 这样**同一 spec + 同一 seed 仍然可复现**，同时难度可量化（与 `closing_speed`/`time_to_impact` 并列）。
* **不建议照搬**：继承式 `Danmaku` + `act()` 里的取模编舞（`data1.D5` 那种边跑边 `shooters.add`，`data1.py:155-165`）。你已经选择了 `PatternSpec -> [SpawnEvent]` 的纯函数路线（`generators/patterns.py` 文档头明确“run once, at build time, seeded RNG, never touch the live world”），这比 Keiki 的“运行期读全局玩家状态 + 动态增删发射器”更可测、更适合 FPGA 侧表驱动。Keiki 有个副作用尤其要避免：**新加入的发射器当帧不生效**（§4.4），这种隐式一帧延迟会让时间线难以推理。

### 9.2 碰撞检测的数据结构

* Keiki 的做法是**逐弹圆-圆 O(N)/帧**（`bullets.py:29-34` + `bullet2player.py:32-35`），且**判定半径按“弹型”查表**（`collider_mapping`，`bullet_creation.py:104-123`），不做宽相。对你最有价值的不是算法（你的 `collision/grid.py` 已有 CSR 计数排序桶 + 向量化批量查询，且已实测“单玩家时暴力法更快”，见 `OPEN_SOURCE_REFERENCES.md`）而是组织方式：① **把“判定形状/半径”作为类型目录里的一等字段** —— Keiki 用 `BulletTypes → 半径` 静态映射，与你 `ObstacleType(..., shape, defaults)` + `SIZE_PRESETS`（DESIGN.md §7.5）一致；建议把“半径/半宽半高”统一收进 `CATALOG`，并保证 `circle` 类型 `half_w == half_h == radius`（你们已如此），这样碰撞层只需一个 `shape_code` 分派，不会出现 Keiki 那种“注释掉的六边形碰撞体永远躺在文件里”。② **明确“碰撞触发的归属”** —— Keiki 把检测写在**子弹自己的 `update()`** 里（`bullets.py:26-38`），导致“先更新谁”影响同帧结果，安全边距只能靠改全局 `Collider.hr`（`realtime_sim.py:31-32`）；你的 `World.step()` 统一积分 + 碰撞**要保持**，安全边距作为 `validate()` 的显式参数（你已有 `safety_margin`），**不要**走全局可变状态。③ Keiki 的“出界范围按弹型分档”（`get_out_range`，`bullet_enums.py:53-62`）可作为 `ttl`/边界回收的补充：**用几何边界而非计数器**回收，天然与场地尺寸解耦。

### 9.3 “用生成模型扩充障碍场景”这条路可行吗？——诚实评估

**先给结论**：**技术上可行，但收益/风险比不划算，建议不作为主路径**。理由与可落地做法如下。

**(1) 天然兼容的部分**

* Keiki 最有价值的确认是：**生成的是“参数序列”而不是像素**（§5.1）。这与你平台完全同构 —— 你的 `SpawnEvent` 表本身就是低维参数向量。因此“用生成模型产出障碍配置”不需要任何图像域技术。
* 你的 `generators/` 已经是**纯函数 + 构建期一次性执行 + 零运行期随机**（`patterns.py` 文档头），这正是生成式方法唯一安全的接入点：**生成只发生在数据集构建期**，产出物是静态场景文件（你的 `ScenarioSpec.to_dict()`/`.save()`）。
* TimeGAN 的**三段式训练范式**（先自编码重建 → 再隐空间一步监督 → 最后联合对抗 + 均值方差，`generator/TimeGAN/train.py:53-214`）是三者中结构最完整、最值得“照结构重写”的一个；如果你确实要做时序障碍生成（例如“通道开合序列”），这是唯一有参考价值的骨架。

**(2) 与你的两条硬约束的冲突（必须正面解决）**

* **冲突 A：可复现性。** Keiki 全链路不可复现：**零 seed**、`torch.randn` 未播种、权重未入库、依赖无版本锁定（§7）。而你的不变式是“同一 `ScenarioSpec` + 同一 `seed` ⇒ 同一 `SpawnTimeline` ⇒ 同一轨迹”（DESIGN.md §7 末尾）。**共存方案**：把生成器**移出仿真路径**，做成“数据集构建工具”：spec 里放 **artifact 引用 + 内容哈希**（`generator_id`、`weights_sha256`、`torch_version`、`output_sha256`）而不是“调用某个 GAN”的字段；仿真期只读已冻结的 `SpawnEvent` 表。这样生成是一次性离线数据生产，不破坏 `SeedManager`（`core/rng.py` 的命名子流）的任何保证。**绝不**在 `World.step()`/`reset()` 里调用模型。
* **冲突 B：安全保证。** Keiki 对生成结果**没有任何校验**，只有 `bound(x, 0.2, 0.8)` 的域裁剪（`cm_danmaku.py:47`）——只保证“参数在量程内”，**不保证“有解”**。你的 `safety/validate.py` 已有正确判据（膨胀自由空间 + 时窗路径搜索 + 速度/加速度可达性 + 从实际出生点出发）与 `generate_valid_scenario(...)` 的 **accept/resample** 模式。**共存方案**：把生成模型当作**候选采样器**（proposal），后面接 `is_scenario_valid` 闸门，拒绝则重采样，并把**接受率**记录为生成质量硬指标 —— 安全性归校验器，生成器只负责“提出更有趣的候选”。
* **冲突 C：评估困难。** Keiki 的评估器（`SF/EFR/MM/DE/C`，`game.py:228-251`）与 GAN **完全没有连起来**（§5.7），`viz_paramseq` 只有一张热力图。**共存方案**：用**你自己的** `ComplexityParams` 实测值（DESIGN.md §7.6）作适应度，比较真实场景分布 vs 生成场景分布在可解释维度上的覆盖度（分位数覆盖、两样本检验），**不要**引入 FID/IS —— 你的数据不是图像，Keiki 也没做。

**(3) Keiki 已暴露的坑，直接适用于此路线**

* **模式崩溃**：PSGAN 的 `MeanVarLoss` 定义了却未启用（`generator/PeriodicSpatialGAN/train.py:31-39` vs `88` 的注释）—— 连作者团队都漏掉这一步，说明“防崩溃”必须**写成可断言的训练完备性检查**（例如断言训练循环里存在 MV 损失的 backward）。
* **监督/对抗权重极敏感，且判别器可能没在训练**：TimeGAN 里同一个 `s_loss` 被乘 10 又乘 100（等效 1000，`train.py:154-156`）而对抗项系数为 1；同时 D 缺 `backward()`（§5.5 缺陷 1）。迁移这类代码**务必先写单元测试断言“每轮 D 参数确实变化”**。
* **数据规模**：Keiki 只有 34 条真实弹幕（§5.8）。你的 5 种 `ObstacleType × 5 layout × 参数` 虽然参数连续，但**真实标注样本同样稀少**；在此规模下 GAN 相比“参数空间约束采样 + 校验”几乎没有优势。

**(4) 建议的分阶段路线（按性价比排序）**

| 阶段 | 做法 | 依赖 | 可复现性 | 安全保证 |
|---|---|---|---|---|
| **0（强烈建议先做）** | 在 `PatternSpec` 参数空间内做**约束随机采样**（每个 `ObstacleSpawn` 字段给定合法区间 + `min_player_distance` 等硬约束），过 `validate_scenario` 的 accept/resample | 现有代码，零新依赖 | ✅ `SeedManager` 命名子流完全覆盖 | ✅ 校验器前置 |
| 1（可选） | 若确实要生成模型：先改造编码 —— 定义**显式、带版本号的 `EventSchema`**（字段名/量程/枚举），把它落成 `(N,F)` 数组；参考 Keiki 的 `encoding_scheme`（`bullet_creation.py:406-421`）与 `decode_param_seq`（`cm_danmaku.py:20-56`）的**成对可逆**设计，但用 dataclass + JSON schema 替代魔法列序 | torch（离线） | 生成离线冻结为 artifact + hash | 生成后必须过校验器 |
| 2（研究性） | 只借鉴 **TimeGAN 的三段式结构**做条件时序生成（按 `ObstacleType` 条件），把 `P(G(z))` 的 rollout 长度纳入训练（Keiki 没有），用接受率 + complexity 覆盖度做模型选择 | torch + 大量调参 | 同上 | 同上 |

**一句话**：**把 Keiki 的“参数序列 + 可逆编解码”当作数据接口范式来用，把它的 GAN 当作“离线候选生成器 + 反面教材”来读；不要把生成模型放进你的确定性内核。** 你的平台真正的差异化优势恰恰是 Keiki 缺失的那两样：**命名种子流的可复现性**与**有解性校验闭环**——引入生成式方法时，这两条边界不能松。

---

### 附：本报告用到的核验命令（可复现）

```bash
find . -iname '*licen*' -o -iname '*copying*' -o -iname 'requirements*.txt' -o -iname 'setup.py'  # → 无命中
grep -rn "seed" --include='*.py' .        # → 无命中（无任何种子）
grep -c '^class .*(Danmaku)' data/code/*.py   # → 34
grep -rn "MeanVarLoss" generator/         # → PSGAN 只定义未使用
grep -n "backward()\|optD.step()" generator/TimeGAN/train.py   # → 118..132 之间无 backward
# 纯 Python 解析 .npy 头（无 numpy）→ (500,15) <f8，并解出每文件首条 btype/color 以确定 D 编号映射
cmp -s data/mat/Dn.npy danmakus/Dn.npy    # 34/34 完全相同
```
