# `core/fpga/ref/` —— C 参考模型（RTL 的比对对手）

## 为什么是 C，不是 Python

RTL 要**逐位比对**，对手必须和它一一对应：整数、定标、位宽写死，逐帧处理。
`core/vision/` 那版 Python 留作**语义真值**，用来判断 C 模型有没有跑偏。

**C 模型里没有一步走 `double`**（含角度）。这是刻意的：只要 C 侧还有一步浮点
算完再量化，RTL 就得在最后一位上赌它落在哪边。现在两边是**同一串整数运算**，
逐位一致是构造性的。

## 文件

| 文件 | 作用 |
|---|---|
| `vision_ref.c` | 视觉链路的 C 参考模型；纯 C11，无第三方依赖。`-DVR_NO_MAIN` 编成 .o 给协同仿真用 |
| `ref_api.h` | 协同仿真的接口（`vr_ref_process`）与结果结构体 |
| `export_frames.py` | 把真实录像导成 **P5 PGM / P6 PPM** 激励 |
| `compare_with_python.py` | C 模型 vs Python 视觉链的语义对账（P1 验收） |

## 跑起来

```bash
# 编译（-Wall -Wextra 必须零警告，测试也这么要求）
gcc -O2 -Wall -Wextra -std=c11 -Icore/fpga/ref -o /tmp/vision_ref core/fpga/ref/vision_ref.c

# 导激励（**验墙必须彩色**，见下）
python core/fpga/ref/export_frames.py \
    --video data/vision/run4/screen.mp4 --out /tmp/stim_color --frames 320 \
    --start 200 --stride 2 --color

# 跑一帧（--set 可覆盖任意检测参数，可重复）
/tmp/vision_ref /tmp/stim_color/000000.ppm out.bin
/tmp/vision_ref /tmp/stim_color/000000.ppm out.bin --set min_seg_px=6 --set band_min_rows=4
```

`main` 与协同仿真走的是**同一条代码路径**（`vr_ref_process_ex`），所以命令行
跑出来的数与 tb 里跑的数不会出现"一个对一个不对"。

## 与 RTL 的对应关系

| `vision_ref.c` | `rtl/vision_top.v` / `rtl/vision_ccl.v` |
|---|---|
| `vr_rgb_to_gray()` | `vision_ops.vh: vo_gray` |
| `vr_classify()` | `vision_ops.vh: vo_classify` |
| `vr_find_bands()` / `vr_find_span()` | 状态机 `S_BAND` / `S_SPAN` / `S_SPAN_FIND` |
| `vr_col_step_max()` + 列投影 | 状态机 `S_COL_SCAN`（列主序 + 两级移位寄存器） |
| `vr_segment_stats()` / `vr_atan2_q6()` | 状态机 `S_SEG_X` / `S_SEG_FIN`；`vision_ops.vh: vo_atan2_q6` |
| `vr_extract_walls()` 的缺口段 | 状态机 `S_GAP` |
| `vr_extract_player()` | `vision_ccl.v` 整块 |
| `vr_scale_round()` | `vision_ops.vh: vo_scale_round` |
| `vr_crc32()` / `vr_encode_frame()` | 状态机 `S_PACK` + `vision_ops.vh: vo_crc_step` |

## 第 2 步：与 Python 逐帧对账（P1 验收，**已跑通**）

```bash
python core/fpga/ref/compare_with_python.py --stim /tmp/stim_color --ref /tmp/vision_ref --limit 40
```

真实录像 `run4/screen.mp4` 的 40 帧彩色激励（640×360）：

| 指标 | 结果 |
|---|---|
| **缺口中心误差** | **中位 0.00 px，p90 0.00，max 0.00 px** ← P1 验收线 0.1 场地单位 ✓ |
| 缺口条数 | Python 160 / C **160** |
| 墙段条数 | Python 320 / C **320** |
| 角色位置误差 | 中位 **0.33 px**，max 0.66 px |

第二套语料（`--start 1500 --stride 3`，与上面不重叠的 40 帧）：
缺口 300/**300**、墙段 605/**605**、缺口中心 max **0.00 px**、角色中位 0.57 px。

### 两处曾经"差一点"的根源（都不是算法差异，是**参数**）

1. **判据本身写错了**：C 模型与 RTL 把墙的 G 下限写成 110（Python 是 60）、
   亮色少了 `max-min < bright_spread`、绿色写成 50/90（Python 是 10/90）。
   而 **"RTL == C 模型" 全程全绿** —— 两边错得一模一样。
   修完墙段/缺口条数从 318/159 变成 320/160，缺口中心误差 max 0.16 → 0.00。
   现在有测试钉住"默认参数 Python ↔ C ↔ RTL 三方一致"。
2. **参数要按分辨率缩放**：Python 的 `WallWithGapParams.rescaled(w, h)`
   （640×360 → s=0.75：`min_seg_px` 8→6、`band_min_rows` 5→4、`field_col_min` 5→4、
   `player_min_area` 60→34 …）。C/RTL 的默认值是 640×480 的，所以对账脚本必须
   把缩放后的值喂进去。**做 0 次和做 2 次都会错**，脚本里有守卫会当场报错。

> **教训**：`RTL == C 模型` 只能证明"两边一样"。**只有 Python 那一层是语义真值。**

### 这里修掉过一个**真 bug**（角色那 2.4 px 的来源）

早先 C 模型报"角色差 2.40 px"，当时的解释是"Python 多做了一步 `refine_circle`
亚像素圆拟合，C 用的是连通域矩质心"。

**那个解释是错的。** 真正的原因是并查集 `uf_union()` 只把 `parent[b]` 指向 `a`，
**没有把 b 已经累积的矩搬进 a**：

```
for (i = 0; i < n_cur; i++) { ... 合并 ... }        // 先分
for (i = 0; i < n_cur; i++) { comp[find(label)] += ... }  // 后合
```

连通域一旦"先分后合"（斜接、V 形），被并入那个分量在**之前各行**累出来的
`n / sx / sy / bbox` 就永远留在 `comp[b]` 里没人读了 —— 面积与质心都偏小。
修法是在 union 的那一刻就把矩与包围盒合过去。修完角色误差从 **2.40 px → 0.33 px**。

剩下的 0.33 px 才是 `refine_circle` 的口径差（那确实是估计器不同，不是 bug）。

## 第 3 步：协同仿真（**P2 验收已过**）

`core/fpga/cosim/` 把 Verilator 编出来的 `vision_top` 和本目录的 `vision_ref.c`
链进同一个可执行文件，两边吃同一帧同一批像素，逐位比较中间量与最终字节流。

```bash
cd core/fpga/cosim && make
./obj_dir/Vvision_top /tmp/stim_color 320
#   比对项 221605083，不符 0，有差异的帧 0
```

**共 320 帧真实录像，2.2 亿次比对，零差异。** 细节见 `../rtl/README.md`。

## 已经**验证过**的部分（`../tests/`）

1. **线格式跨语言一致**：C 用手写的逐位 CRC32 打包 → Python 用 zlib 解码，
   魔术/版本/长度/CRC 全过。这是 P1/P2 的前提 —— C 的输出 Python 都认不出，
   RTL 比对就无从谈起。
2. **矩算术正确**：已知位置的亮方块 → 质心 `(29.5, 19.5)`、半径 `10.0`，
   与解析值一致（精度 1/64 px）。这条当场抓到过一个真 bug：半径公式写成
   `/8` 而 Python 的口径是 `(w+h)/4`，半径正好差一倍。
3. **空帧不编数据**：全黑帧报"没有角色"，而不是给一个默认值。
4. **RTL 常量不漂移**：`vision_pkg.vh` 的宏与 `describe_vision_protocol()`
   逐个核对；`vision_ops.vh` 的 arctan 表与 C 逐项核对。

## 调试开关（都在代码里，默认关）

```bash
VR_DEBUG_SEG=1 /tmp/vision_ref frame.ppm out.bin    # C 侧打印每段的 ls/ln/rs/rn/lq/rq/dx/ang
./obj_dir/Vvision_top /tmp/stim_color 1 +dumpseg     # RTL 侧打印同样的量 + 落库后的值
```

两边格式故意做成一样，`diff` 一眼就能看出是哪一段、哪一项先分叉。

## 一个必须记住的限制

**灰度激励会让颜色判据退化**：`classify` 里 wall 的 `R-B`、green 的 `G-max(R,B)`
在灰度下恒为 0，所以用灰度只能验证**亮度类**算子（bright/矩/质心）。
要验墙的列投影与缺口，激励**必须彩色** —— `export_frames.py` 的 manifest 里
`mode: gray8|color8` 就是标记这件事，别把灰度下的指标当成真实输入下的指标。
