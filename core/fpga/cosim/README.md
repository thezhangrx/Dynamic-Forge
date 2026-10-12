# `core/fpga/cosim/` —— RTL ↔ C 参考模型协同仿真

两个测试台，分工不同：

| 测试台 | 打的是哪个模块 | 覆盖什么 |
|---|---|---|
| `tb_vision.cpp` → `Vvision_top` | `vision_top`（算法流水） | **全部中间量 + 整帧字节流**，320 帧逐位一致 |
| `tb_axi.cpp` → `Vvision_axi` | `vision_axi`（PS 面向接口） | 配置寄存器读写回、CTRL/状态位、帧窗口、调试口 |
| `tb_tpg.cpp` → `Vvision_selftest` | `vision_tpg` + `vision_top` | **和手算的几何比**（不用相机，也不用 C 模型） |

为什么要两份：`tb_vision` 直接驱动引脚，测的是**算法**；上板时 PS 碰到的是
**AXI 口**，地址译码/握手/窗口/CTRL 位定义这些只有走 AXI 才测得到。
`dout_last` "每帧拉高两次"那个 bug 就是 `tb_axi` 抓到的 —— `tb_vision`
跑了 320 帧零差异也没发现它。

把 Verilator 编出来的 `vision_top` 和 `../ref/vision_ref.c`（`-DVR_NO_MAIN`
编成 .o）链进**同一个可执行文件**，两边吃同一帧同一批像素，逐位比较：

| 比什么 | 怎么拿到 RTL 的值 |
|---|---|
| 灰度 / 墙 / 亮 三张掩膜、行直方图 | 直接读 RTL 内部帧存（`top->rootp->vision_top__DOT__*_mem`） |
| 行带、场地 x 范围、各类计数、墙段、缺口、角色 | `vision_top.v` 的**调试/回读口**（`dbg_sel/dbg_idx/dbg_sub → dbg_data`） |
| 几何帧字节流（含 CRC32） | `dout_vld/dout_data/dout_last/dout_ready` |

**为什么在同一个进程里比，而不是两边各写文件再 diff**：中间量不必发明一套
dump 格式；出问题的第一现场可以直接打印"哪一帧、哪一类、第几个、期望/实际"。
调试口本身也是**上板后 PS 回读中间量**要用的那一套，不是为测试临时加的。

## 跑

```bash
# 激励：真实录像的彩色 PPM（灰度会让 R-B 判据恒为 0，墙测不出来）
python ../ref/export_frames.py --video ../../../data/vision/run4/screen.mp4 \
    --out /tmp/stim_color --frames 320 --start 200 --stride 2 --width 640 --color

cd core/fpga/cosim
make                                   # 需要 Verilator（见 ../ref/README.md）

# 算法流水（全部中间量 + 整帧字节流）
./obj_dir/Vvision_top /tmp/stim_color 320
#   === 协同仿真结果 ===
#   比对项 221605409，不符 0，有差异的帧 0
#   几何帧指纹 5b119febe6015a37

# PS 面向接口（AXI4-Lite：配置寄存器 → 启动 → 读帧窗口）
./obj_dir/Vvision_axi /tmp/stim_color 20
#   === AXI 协同仿真结果 ===
#   比对项 11490，不符 0

# PL 自检图案（不用激励文件，也不用 C 模型：和手算的几何比）
./obj_dir/Vvision_selftest 8
#   === 自检图案结果 ===
#   比对项 403，不符 0
```

### 关于上面那几个"比对项"数字（重要）

**"不符 0" 是可复现的不变量；"比对项 N" 不是。** 它由两段组成：

| 段 | 大小 | 会不会变 |
|---|---|---|
| 三张掩膜 + 行直方图 | `320 × (3·640·360 + 360)` = **221 299 200** | 不变（只跟帧数和尺寸有关） |
| 角色/障碍/缺口/整帧字节 | 约 306 000 | **会变** —— 跟解码出来的像素有关 |

视频要过 `cv2.VideoCapture` 解码才变成激励，**解码器版本不同 → 像素略有差异 →
检出的障碍/缺口个数不同 → 这一段的比对项就不同**。实测过的三个值：
`221605083`（rtl/README）、`221605405`（本文件旧记录）、`221605589`（本机 cv2 5.0.0）。
三段都对得上"320 帧、不符 0"，**没有一次是回归**：`tb_vision.cpp` 自
`b34a93f` 起只有一个提交，比对逻辑没动过。

所以：

* **判回归看"不符 0"和指纹**，不要盯 N；
* 记录 N 时必须连**指纹 + 解码器版本 + 导出命令**一起记，否则复现不出来；
* 换个环境跑出不同的 N 不等于出了问题 —— 但也**必须先排除**"比对代码被改过"
  才能下这个结论（`git log -- core/fpga/cosim/tb_vision.cpp`）。

pytest 入口在 `../tests/test_cosim.py`（默认只跑 4 帧以保持套件快，
`FPGA_COSIM_FRAMES=320` 跑全量）。

## 命令行开关

```bash
# 中间量对照：两边格式故意做成一样，diff 就能看出哪一段先分叉
./obj_dir/Vvision_top /tmp/stim_color 1 +dumpseg     # RTL 每个墙段的中间量
VR_DEBUG_SEG=1 /tmp/vision_ref frame.ppm out.bin     # C 侧同一批中间量

# 改参数（两边一起改）：验证配置寄存器真的接到了判据上
./obj_dir/Vvision_top /tmp/stim_color 4 --bright 100
./obj_dir/Vvision_axi /tmp/stim_color 4 --bright 100

# **反面控制**：只改 RTL 的寄存器、不改 C 的参数 —— 必须报"不一致"。
# 没有它，"配置寄存器接对了"可能只是"谁都没在看那个寄存器"。
./obj_dir/Vvision_top /tmp/stim_color 4 --rtl-only 2 100
./obj_dir/Vvision_axi /tmp/stim_color 4 --rtl-only 0x00C 100   # 0x00C = BRIGHT_MIN

# 每轮输出几何帧字节的 FNV-1a 指纹：用来判断"这组参数到底有没有改变输出"
```

两边格式**故意做成一样**，`diff` 就能看出是哪一段、哪一项先分叉。
（这个开关就是在 P2 里定位"落库的字节和打印出来的值不一样"用的。）

## 已知的坑

* **给 Verilator 一份自带 `main` 的 tb 时不能用 `--binary`**：它会再生成一个
  `main`，链接报 `multiple definition of main`。要用 `--cc --exe --build`。
* **`VERILATOR_ROOT` 必须导出**：从解包安装（`$HOME/.local/verilator-5.020`）跑时
  不导出就会去 `/usr/share/verilator` 找 `verilated_std.sv`。Makefile 里
  `export VERILATOR_ROOT` 就是为这个。
* **C 侧对象必须用 `-DVR_NO_MAIN` 单独编**：Verilator 会把 `.c` 当 C++ 编，
  而且 `-CFLAGS` 的 `-DFOO` 传不到它。Makefile 里 `$(REF_OBJ)` 单独一条 gcc 规则。
* **函数调用别直接写进"数组元素"的非阻塞赋值 RHS**：见 `../rtl/README.md`
  的"踩过的坑"第 1 条。
