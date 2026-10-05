# Image-Processing-Toolbox 技术总结

> 本文只基于本仓库实际文件写成。所有结论后附 `文件:行号` 来源；凡代码/文档没有明确给出、
> 或需要我推算的，都标注为「推算」或「未确认」。
> 未做综合/仿真（本机无 Vivado），未联网，未修改任何已有文件。

---

## 1. 一句话定位

这是一个 **ES 203（Digital Systems）课程项目**：用 Verilog 在 Xilinx Basys3（Artix-7
`xc7a35tcpg236-1`）上实现一个「图像处理工具箱」，把一张图片预先转成 96-bit 宽的 `.coe`
ROM，由 VGA 控制器读出来实时做 16 种基础/卷积图像算子并显示到 640×480 显示器上；
另有两个纯仿真的 Vivado 工程（`BIPT/`、`Blurring/`）用于在 PC 上通过 BMP 文件读写做同样
的算法验证。

- 课程/学校：ES 203: Digital Systems Project, Prof. Joycee Mekie, IITGN（`README.md:2`）
- 作者：仓库与 testbench 里的绝对路径显示是 Gowtham（`README.md:86`；`Blurring/Blurring.srcs/sim_1/new/blur_tb.v:46`）
- 许可证：Apache License 2.0（`LICENSE:1`、`README.md:205`）
- 创建时间线索：源文件头注释为 2018-10/11（`BIPT/BIPT.srcs/sources_1/new/Modules.v:6`、`Blurring/Blurring.srcs/sources_1/new/blur.v:6`、`Final Project/VGA_1/VGA_1.srcs/sources_1/new/VGA.v:6`）

---

## 2. 技术栈与工具链

| 项目 | 结论 | 来源 |
|---|---|---|
| HDL | 用户 RTL **全部是 Verilog**（3 个文件：`Modules.v`、`blur.v`、`VGA.v`） | `find` 结果；`BIPT/BIPT.srcs/sources_1/new/Modules.v` 等 |
| VHDL | 仅 3 个文件，且**全部是 Vivado 生成的 blk_mem_gen IP 源码/仿真包**，不是用户所写 | `Final Project/VGA_1/VGA_1.srcs/sources_1/ip/image/synth/image.vhd`、`.../hdl/blk_mem_gen_v8_4_vhsyn_rfs.vhd`、`.../misc/blk_mem_gen_v8_4.vhd` |
| 综合/仿真 | Xilinx Vivado（`README.md:25`、`README.md:90`）；仿真波形目录为 `xsim.dir`，日志为 `xsim`，即 **Vivado Simulator (XSim)** | `BIPT/BIPT.sim/sim_1/behav/xsim/`、`Blurring/Blurring.sim/sim_1/behav/xsim/` |
| 目标板/器件 | Basys3 rev B，`xc7a35tcpg236-1`，board part `digilentinc.com:basys3:part0:1.1` | `BIPT/BIPT.xpr`（Part）、`Final Project/VGA_1/VGA_1.runs/synth_1/vga_syncIndex.tcl:2,4`；`constr.xdc:1` |
| 上位机语言 | Python 3 + OpenCV(`cv2`) + NumPy，用于「图片 → COE」 | `scripts/coe_generator.py:9-11`、`scripts/parallel_image_generator.py:14-16` |
| 约束文件 | 1 个用户 `.xdc`（VGA 工程）；BIPT/Blurring 无用户 xdc | `Final Project/VGA_1/VGA_1.srcs/constrs_1/new/constr.xdc` |
| `.qsf` | **不存在**（`.qsf` 是 Quartus 文件，本仓库是 Vivado 流程）；只找到脚本自动生成的 `.tcl` | `find . -name "*.qsf"` 无结果 |
| IP / 配置文件 | `image.xci`（Block Memory Generator v8.4）+ `.mif`/`.coe` | `.../ip/image/image.xci`、`.../ip/image/image.mif` |
| 生成的比特流 | `vga_syncIndex.bit`（已入库） | `Final Project/VGA_1/VGA_1.runs/impl_1/vga_syncIndex.bit` |

### 2.1 三个独立的 Vivado 工程（不是一个工程的三个目录）

| 工程 | 顶层模块 | 定位 | 有无 BRAM/VGA |
|---|---|---|---|
| `BIPT/BIPT.xpr` | `Modules`（`Modules.v:23`） | PC 仿真，基础逐像素算子 | 无（只用 LUT/FF） |
| `Blurring/Blurring.xpr` | `blur`（`blur.v:46`） | PC 仿真，3×3 卷积 | 无 |
| `Final Project/VGA_1/VGA_1.xpr` | `vga_syncIndex`（`VGA.v:22`） | 上板，BRAM-ROM + VGA 显示 | 有 |

- 前两个工程的仿真报告显示 **Block RAM Tile = 0**；上板资源全在 VGA 工程（`BIPT/BIPT.runs/synth_1/Modules_utilization_synth.rpt`、`Blurring/Blurring.runs/synth_1/blur_utilization_synth.rpt`）。
- 注意 `Final Project/` 目录名带空格，所有路径引用都必须加引号。

---

## 3. 目录与模块地图

| 路径 | 内容 | 实现的算法 | 来源 |
|---|---|---|---|
| `BIPT/` | Verilog 模块 + testbench，PC 仿真 | 8 个逐像素算子：RGB2Gray、增/减亮度、颜色反转、红/蓝/绿「滤色」、原图 | `BIPT/BIPT.srcs/sources_1/new/Modules.v:42-238` |
| `Blurring/` | Verilog 模块 + testbench，PC 仿真 | 8 个 3×3 卷积：均值模糊、Sobel、边缘检测(拉普拉斯类)、运动模糊、浮雕、锐化、定向运动模糊、高斯模糊 | `Blurring/Blurring.srcs/sources_1/new/blur.v:24-28,67-311` |
| `Final Project/` | 说明 + 上板工程 | 16 个算子的 opcode 表 | `Final Project/Functions.txt:1-16` |
| `Final Project/VGA_1/` | 上板顶层 + BRAM IP + 约束 + 比特流 | 把上面 8+8 个算子合并进一个 4-bit `sel_module` 选择器，VGA 实时显示 | `Final Project/VGA_1/VGA_1.srcs/sources_1/new/VGA.v` |
| `scripts/` | 3 个 Python 脚本 | 图片→9 张平移图；图片→COE；9 张图+彩图→卷积用 COE | `scripts/*.py` |
| `tests/` | 3 个 `unittest` 文件 | 只测 Python 脚本，不测 HDL | `tests/test_*.py` |
| `images/` | 14 张 `.bmp` | README 特性表里的示例输入/输出图 | `README.md:30-47` |
| `flower.coe` | 根目录 COE，18400 行 × 96-bit，CRLF | 上板用初始化数据（与 `Final Project/VGA_1/flower.coe` 内容仅末字符不同） | 见 §4.3 |
| `flower.jpg` | 输入原图 | 用来生成 COE 的源图 | `README.md:116` |
| `poster.pdf` | 项目海报（676 KB，二进制，未解析） | — | 文件存在，**内容未确认** |

`Final Project/VGA_1/` 下还有 4 份同源 COE：`flower.coe`、`image.coe`、`img.coe`（三者 md5 完全相同，末行以 `;` 结尾）、`imgx.coe`（内容不同，是真·全零占位）。IP 实际绑定的是 `flower.coe`。

---

## 4. 核心算法与硬件微架构

### 4.0 全局数据通路：**没有行缓冲，用「软件预展开 3×3 邻域 + 96-bit 宽 ROM」代替**

这是整个项目最关键的架构决策，也是想学「行缓冲 + 流水线」的人最容易看错的地方：

1. Python 先把原图**平移出 8 张图**（上/下/左/右/左上/…），得到每个像素的 3×3 邻域；
2. Python 把 9 个邻域灰度值（各 8 bit）+ 当前彩色像素 BGR（24 bit）
   **打包成一个 96-bit 字**，写进 COE；
3. 硬件每个像素周期只做一次 96-bit ROM 读，就同时拿到整个 3×3 窗口 → **不需要 line buffer、
   不需要 window shift register、不需要多拍拼窗**；
4. 代价：存储放大 9×（本设计 18400×96 bit ≈ 1.77 Mbit，占满 Basys3 的 BRAM）。

数据通路（`VGA.v`）：

```
addra(15b, addra 计数器) ──► image IP (Single_Port_RAM, 96b×18400, clka=clk)
                                   │  读延迟 1 个 clk（douta 已寄存）
                                   ▼
                            out2[95:0]
                                   │ 按位切片解包 9 邻域 + BGR（VGA.v:119-131）
                                   ▼
              tred/tgreen/tblue + gray/left/right/up/down/leftup/leftdown/rightup/rightdown
                                   │ 组合逻辑按 sel_module 选择算子（16 分支）
                                   ▼
                    red_o/green_o/blue_o (8b) ──► /16 ──► red/green/blue (4b) @pixel_clk
```

**流水线与周期（精确回答）**

- **流水线级数 = 0**（算子本身是纯组合求值，只在末端打一拍输出）；整条链的**寄存器级数 = 2**：
  1 级 BRAM 输出寄存 + 1 级 RGB 输出寄存。
- **每个像素 1 个 `pixel_clk`**（= 2 个输入 `clock`），吞吐 = 1 pixel/pixel_clk，
  即 50 Mpx/s（见 §4.2 时钟推算）。
- 从 `addra` 到 RGB 的延迟 ≈ 2 个 `pixel_clk`（BRAM 读 1 拍 + 输出寄存 1 拍）。
- **没有 valid/ready 握手**，完全是自由跑的行/列计数 + `blank` 门控，`reset` 只用于把输出压 0
  （`VGA.v:111-556`）。

**定点格式**

| 项 | 结论 | 来源 |
|---|---|---|
| 输入像素 | 每通道 **8-bit 无符号**（0–255） | `VGA.v:119-131` 的 `reg [7:0]` 切片 |
| 输出像素 | 每通道 **4-bit**（RGB444，12-bit 色深），由 `/16` 后取 `[3:0]` | `VGA.v:53-54,151-153` |
| VGA 顶层中间量 | `reg [15:0] r, b, g`（16-bit 无符号，靠回绕表示负数） | `VGA.v:34` |
| `BIPT` 中间量 | `reg [8:0] red_x/green_x/blue_x`（9-bit，够放 255+255） | `Modules.v:38` |
| `Blurring` 中间量 | `reg [31:0] red_x/green_x/blue_x`（32-bit，远超需要） | `blur.v:65` |
| 舍入 | **全部是整数截断除法**，无四舍五入：`/9`、`/3`、`/16`、`/2`、`>>1/2/4/5` | `blur.v:80-82,179,282,299`；`VGA.v:143-149,323,337` |
| 溢出/负值 | 16-bit 回绕 + 阈值判别，**没有统一的饱和逻辑**，见 §8 的坑 | `VGA.v:362-370,393-405,444-456` |
| 灰度系数 | `(R>>2)+(R>>5)+(G>>1)+(G>>4)+(B>>4)+(B>>5)` = `(9R+18G+3B)/32`，系数和 30/32（偏暗） | `VGA.v:143`；`Modules.v:50` |

**一个重要的命名/方向陷阱**：`left.bmp` 是把内容**向右**平移得到的，所以
`left.bmp(r,c) = 原图(r, c-1)`，即它存的是「左边那个像素」——名字指的是邻域方向，不是平移方向
（`scripts/parallel_image_generator.py:108-118` 的注释）。代码注释里的矩阵
`| 8 4 6 | / | 3 1 2 | / | 9 5 7 |` 因此是**列方向镜像**的（第一列是右侧邻居），
但实际卷积公式与标准核对得上（见 §4.5、§4.6）。

---

### 4.1 BRAM 使用（帧缓冲）

| 参数 | 值 | 来源 |
|---|---|---|
| IP 类型 | Block Memory Generator v8.4，`Memory_Type = Single_Port_RAM` | `image.xci`（`PARAM_VALUE.Memory_Type`） |
| 字宽 | **96 bit**（`Read_Width_A = Write_Width_A = 96`） | `image.xci`（`PARAM_VALUE.Read_Width_A`） |
| 深度 | **18400** 字（`C_READ_DEPTH_A = 18400`，地址位宽 15） | `image.xci`（`MODELPARAM_VALUE.C_READ_DEPTH_A`） |
| 端口 | 单口；`addra[14:0]`、`dina[95:0]`、`wea[0:0]`、`douta[95:0]` | `.../ip/image/sim/image.v:65-74` |
| 读延迟 | `Register_PortA_Output_of_Memory_Primitives = true`，`Pipeline_Stages = 0` → **1 个 clka** | `image.xci`（`PARAM_VALUE.Register_PortA_Output_of_Memory_Primitives`、`Pipeline_Stages`） |
| 写使能 | 顶层实例把 `wea` 接在 `reg read = 0` 上，而 `read` **从未被赋值** → `wea` 恒 0，实际是 **只读 ROM** | `VGA.v:57,63-69` |
| 读写冲突 | **不存在**：写通道被恒定关闭，没有双口仲裁/冲突处理 | 同上 |
| 初始化 | `Load_Init_File = true`，`Coe_File = ../../../../flower.coe`（相对 `.xci` 解析为 `Final Project/VGA_1/flower.coe`），`Fill_Remaining_Memory_Locations = false` | `image.xci` |
| 容量 | 18400 × 96 bit = **1,766,400 bit ≈ 1.766 Mbit ≈ 220.8 KB** | 推算：18400×96 |
| BRAM 占用 | ≈ **48 个 RAMB36**（1,766,400 / 36,864 = 47.9）；Basys3 XC7A35T 共 **50 个 RAMB36（1800 Kb）** → **≈ 96%** | 推算；器件 RAMB 数由 `xc7a35tcpg236-1` 规格给出 |
| 实际综合报告 | 仓库里**没有** VGA 工程的 utilization 报告（只有 `htr.txt`、`runme.sh` 等运行脚本），上述 BRAM 数字为**推算**，未用 Vivado 验证 | `find Final Project/VGA_1 -name "*.rpt"` 无结果 |

**地址生成**：`addra` 只在「显示窗口内」自增 1，`18400` 处回绕 0（`VGA.v:542-545`）。
显示窗口每帧恰好 160×115 = 18400 个像素，所以地址天然逐帧对齐，无需帧同步复位。

---

### 4.2 VGA 接口

`vga_syncIndex` 里的时序常数**逐字抄自经典 fpga4fun 640×480@60 发生器**：

| 信号事件 | 计数值 | 含义 | 来源 |
|---|---|---|---|
| 行可见区 | `hc` 0–639 | 640 像素 | `VGA.v:82,86` |
| 行消隐开始 `hblankon` | `hc == 639` | — | `VGA.v:86` |
| 行同步开始 `hsyncon` | `hc == 655` | hsync 拉**低**（负极性） | `VGA.v:87,102` |
| 行同步结束 `hsyncoff` | `hc == 751` | hsync 拉高 | `VGA.v:88,102` |
| 行复位 `hreset` | `hc == 799` | 一行共 800 拍 | `VGA.v:89` |
| 帧可见区 | `vc` 0–479 | 480 行 | `VGA.v:83,94` |
| 帧消隐 `vblankon` | `vc == 479` | — | `VGA.v:94` |
| 帧同步开始 `vsyncon` | `vc == 490` | vsync 拉低 | `VGA.v:95,106` |
| 帧同步结束 `vsyncoff` | `vc == 492` | vsync 拉高 | `VGA.v:96,106` |
| 帧复位 `vreset` | `vc == 523` | 一帧共 **524** 行（VGA 标准为 525，此处少 1） | `VGA.v:97` |

由此得同步参数（推算）：行 **640 可见 + 16 前肩 + 96 同步 + 48 后肩 = 800**；
帧 **480 可见 + 10 前肩 + 2 同步 + 31 后肩 = 523/524**。hsync、vsync 均为**负极性**。

**像素时钟与刷新率（重要，README 与代码不一致）**

- 约束：板载 `W5` 输入 `create_clock -period 10.00` → **100 MHz**（`constr.xdc:7-9`）。
- `pcount` 每拍翻转，`ec = (pcount==0)`，`pixel_clk = ec`（`VGA.v:72-75`）；
  行/列计数器也在 `ec` 有效时才自增（`VGA.v:99-107`）。
  → **有效像素时钟 = 输入 / 2 = 50 MHz**（不是标准 640×480@60 所需的 25.175 MHz）。
- 于是帧时间 = 800 × 524 / 50 MHz = **8.384 ms → ≈ 119 Hz**（推算），约为 60 Hz 的 2 倍。
- README 写的是「640×480 @ 60 Hz」（`README.md:195`），且计数器常数确实对应 25 MHz 版本；
  两者对不上，属于本设计的一个隐患（§8）。**实际显示器能否锁定 119 Hz 未确认**。

**另一路 50 MHz**：`reg clk` 在 `always @(posedge clock)` 里翻转，作为 BRAM 的 `clka`
（`VGA.v:37-44,64`）。它与 `ec` 同频但**相位相差半个输入周期**（`clk` 在奇数拍上升，`ec` 在偶数拍上升，由
`VGA.v:41-44` 与 `74` 的初值决定，为阅读代码后的推算）。地址在 `pixel_clk` 域产生、被 `clk` 域的 BRAM 采样，
等于一条「半周期路径」，而 `constr.xdc` 只有对 `clock` 的一条 `create_clock`，没有对这两个派生时钟的约束。

**显示窗口（图像实际贴屏位置）**

- 条件：`blank == 0 && hc >= 100 && hc < 260 && vc >= 100 && vc < 215`（`VGA.v:113`）
- → 图像尺寸 **160 × 115**，贴在屏幕 **(100, 100)** 处，其余区域输出黑（`VGA.v:551-553`）。160×115 = 18400，与 BRAM 深度严格吻合（推算，也解释了深度为何是 18400 这个非 2 的幂）。

---

### 4.3 COE 文件格式与生成

**格式**（`scripts/coe_generator.py:99-112`、`scripts/kernel_coe_generator.py:140-152`）：

```
memory_initialization_radix=2;
memory_initialization_vector=
<96 个 0/1 字符>,
<96 个 0/1 字符>,
...
<96 个 0/1 字符>;      ← 最后一行必须以分号结束
```

- 总共 **18400 个条目**，每行 96 bit。
- 加载方式：**不是** RTL 里手写 `$readmemh`，而是由 `blk_mem_gen` IP 在综合时按
  `Coe_File = ../../../../flower.coe` 读入（`image.xci`）。整个仓库的 RTL 里没有 `$readmemh`。
- 实测细节：所有 COE 都是 **CRLF 行尾**；`Final Project/VGA_1/flower.coe` 末行以 `;` 结尾（合法），
  而**根目录 `flower.coe` 末行以 `,` 结尾、缺分号**（`tail -2` + `cat -A` 实测）。IP 绑定的是前者，所以能过。

**两种打包布局**

| 脚本 | 布局（由高位到低位） | 用途 |
|---|---|---|
| `coe_generator.py` | `[71:0]` 全 0 填充 + `[23:0]` = B,G,R（OpenCV BGR 顺序） | 逐像素算子（0000–0111） |
| `kernel_coe_generator.py` | `[95:24]` = 9×8-bit，顺序 `gray,left,right,up,down,leftup,leftdown,rightup,rightdown` 的**蓝通道**；`[23:0]` = 原图 B,G,R | 卷积算子（1000–1111） |

- VGA 侧的解包位序与 `kernel_coe_generator.py` 完全一致：`gray=[95:88]`、`left=[87:80]`、
  `right=[79:72]`、`up=[71:64]`、`down=[63:56]`、`leftup=[55:48]`、`leftdown=[47:40]`、
  `rightup=[39:32]`、`rightdown=[31:24]`、`tblue=[23:16]`、`tgreen=[15:8]`、`tred=[7:0]`（`VGA.v:119-131`）。
- 手算第 1 条 `flower.coe` 数据可交叉验证：`00011000 00011000 00000000 00000000 00010111 00000000 00010111 00000000 00000000 | 00000000 00101000 00000010`
  → `gray=24,left=24,right=0,up=0,down=23,leftup=0,leftdown=23,rightup=0,rightdown=0, B=0,G=40,R=2`（推算）。
- 注意：邻域数据只取**蓝通道**。因为平移图是灰度图（B=G=R）所以等价于灰度邻域；
  但 `gray.bmp` 其实是**原彩色图的副本**，并没有显式转灰度（`scripts/parallel_image_generator.py:109`、
  `README.md:102`），conv 算子实际用的是「彩色图的蓝通道」当作灰度，并非真正的 luma。这是一个语义坑。

---

### 4.4 基础算子组 `Modules.v`（PC 仿真，8 个算子，`sel_module[2:0]`）

统一接口：`input clk, reset, done_in; input [2:0] sel_module; input [7:0] val, red, green, blue;
output reg done_out; output reg [7:0] red_o, green_o, blue_o`（`Modules.v:23-37`）。
所有运算在**一个 `always @(posedge clk)`** 内完成，`done_in` 相当于 valid，`done_out` 打一拍
表示结果有效（`Modules.v:40-240`）。

| `sel` | 算法 | 公式/行为 | 行号 |
|---|---|---|---|
| 000 | RGB→灰度 | `(R>>2)+(R>>5)+(G>>1)+(G>>4)+(B>>4)+(B>>5)` 复制到三通道 | `Modules.v:50-52` |
| 001 | 增亮度（饱和） | `x+val`，`>255` 则钳到 255；**写的是 `red_x<=` 阻塞/非阻塞混用** | `Modules.v:70-88` |
| 010 | 减亮度（饱和） | `x-val`，`>255` 判负则置 0（见 §8 的判负写法） | `Modules.v:106-124` |
| 011 | 颜色反转 | `255-x` 每通道 | `Modules.v:142-144` |
| 100 | 红滤色（去红） | `red_o=0, green_o=green, blue_o=blue` | `Modules.v:163-165` |
| 101 | 蓝滤色（去蓝） | `blue_o=0`，保留 R、G | `Modules.v:184-186` |
| 110 | 绿滤色（去绿） | `green_o=0`，保留 R、B | `Modules.v:205-207` |
| 111 | 原图 | 直通 | `Modules.v:226-228` |

资源（Vivado 综合报告实测，`BIPT/BIPT.runs/synth_1/Modules_utilization_synth.rpt`）：
**Slice LUT 145 / 20800（0.70%）**，**Slice Register 25 / 41600**，**BRAM 0**，**DSP 0**，
**Bonded IOB 63 / 106（59.4%）**。器件可容纳。

---

### 4.5 卷积算子组 `blur.v`（PC 仿真，8 个算子）

统一接口：9 组 `redN/greenN/blueN`（N=1..9），每组 8-bit，1=中心，2=left，3=right，4=up，5=down，
6=leftup，7=leftdown，8=rightup，9=rightdown（`blur.v:30-62`，映射见 `blur.v:30-38`）。
同样单 `always @(posedge clk)` + `done_in/done_out`（`blur.v:67-311`）。

| `sel` | 算法 | 实际公式（权重按 1..9 标注） | 归一化 | 行号 |
|---|---|---|---|---|
| 000 | 3×3 均值模糊 | `sum(all 9)/9` | /9 | `blur.v:77-85` |
| 001 | 改进 Sobel | `Gx = 8-6+2·3-2·2+9-7`；`Gy = 8+2·4+6-9-2·5-7`；`(|Gx|+|Gy|)/2` 的符号近似 | /2 | `blur.v:113-131` |
| 010 | 拉普拉斯类边缘 | `8·1 - (2+3+4+5+6+7+8+9)`；负值取相反数输出 | — | `blur.v:149-166` |
| 011 | 运动模糊 xy | `(1 + 7 + 8)/3`（中心 + 左下 + 右上） | /3 | `blur.v:178-187` |
| 100 | 浮雕 Emboss | `1 + 2 - 3 - 4 + 5 + 2·7 - 2·8`（= 注释里 `-2 -1 0 / -1 1 1 / 0 1 2`） | 和=1 | `blur.v:214-224` |
| 101 | 锐化 Sharpen | `5·1 - 2 - 3 - 4 - 5`（十字） | 和=1 | `blur.v:253-267` |
| 110 | 运动模糊 x | `(4 + 8 + 6)/3`（上一行三个） | /3 | `blur.v:281-285` |
| 111(else) | 高斯模糊 | `1·8 + 2·4 + 1·6 + 2·3 + 4·1 + 2·2 + 1·9 + 2·5 + 2·7` /16 | 权重和=**17** | `blur.v:298-299` |

- 0/1 分支用的是 `(red8)- red6 + ...`：**Gx/Gy 都取自红通道**（`blur.v:113-114`），
  在灰度输入下等价，在彩色输入下是 bug。
- 0/1 的「符号判别」用阈值 1024：因为 `|Gx|`、`|Gy|` 最大 4×255 = **1020 < 1024**，
  16-bit 回绕后负值必然 ≥ 64516 > 1024，所以 `>1024` 等价于「是负数」（推算，`blur.v:119-127`）。
  VGA 版同法（`VGA.v:362-370`）。这是**本项目最值得学的一个定点小技巧**。
- 资源（实测报告）：**Slice LUT 998 / 20800（4.8%）**，FF 25，BRAM 0；
  但 **Bonded IOB 247 / 106 = 233%** → 端口数远超 Basys3 引脚，**该工程只能仿真，无法上板**
  （`Blurring/Blurring.runs/synth_1/blur_utilization_synth.rpt`）。这正是它被放在「PC-based」目录的原因。

---

### 4.6 VGA 顶层 16 个算子 `vga_syncIndex`（`VGA.v`）

`sel_module[3:0]` 选 16 个算子，`val[7:0]` 调亮度/滤色强度；opcode 表见 `Final Project/Functions.txt`。
卷积算子用的是解包出的灰度邻域（`gray/left/...`），输出**灰度**写入 RGB 三通道；逐像素算子用 `tred/tgreen/tblue`。

| `sel` | 算子 | 公式（VGA 版） | 行号 |
|---|---|---|---|
| 0000 | RGB2Gray | 同 §4.4，但末端额外 `/16` 再取 4-bit | `VGA.v:136-153` |
| 0001 | 增亮度 | `+val`，`>255` 钳 255，`/16` | `VGA.v:157-189` |
| 0010 | 减亮度 | `-val`，`>256` 判负置 0，`/16` | `VGA.v:193-223` |
| 0011 | 颜色反转 | `255-x`，`/16` | `VGA.v:227-243` |
| 0100 | 红滤色 | `r=tred-val`（超阈值→0），G/B `/16` | `VGA.v:247-265` |
| 0101 | 蓝滤色 | `b=tblue-val` | `VGA.v:269-286` |
| 0110 | 绿滤色 | `g=tgreen-val` | `VGA.v:290-307` |
| 0111 | 原图 | 三通道直通 `/16` | `VGA.v:311-323` |
| 1000 | 均值模糊 | `(gray+left+right+up+down+leftup+leftdown+rightup+rightdown)/9` | `VGA.v:330-349` |
| 1001 | Sobel | `Gx=rightup-leftup+2·right-2·left+rightdown-leftdown`；`Gy=rightup+2·up+leftup-rightdown-2·down-leftdown`；`(|Gx|+|Gy|)/2` | `VGA.v:353-380` |
| 1010 | 边缘检测 | `8·gray - 8邻域和`；`>2048`(负)→0，`>255`→255 | `VGA.v:386-412` |
| 1011 | 运动模糊 xy | `(gray+leftdown+rightup)/3` | `VGA.v:416-433` |
| 1100 | 浮雕 | `gray+left-right-up+down+2·leftdown-2·rightup`；`>1280`→0，`>255`→255 | `VGA.v:437-463` |
| 1101 | 锐化 | `5·gray-left-right-up-down`；`>1280`→0，`>255`→**256**（bug，见 §8） | `VGA.v:467-495` |
| 1110 | 运动模糊 x | `(up+leftup+rightup)/3`（实为竖直方向均值） | `VGA.v:499-517` |
| 1111 | 高斯模糊 | `rightup+2·up+leftup+2·right+4·gray+2·left+rightdown+2·down+2·leftdown`，`/16` | `VGA.v:520-538` |

**与 `blur.v` 的差异**（重要，别把两者当同一实现）：

- VGA 的 1010 边缘检测对负值输出 **0**（`VGA.v:393-396`），而 `blur.v` 的 010 对负值输出**相反数**
  （`blur.v:156-160`）；
- VGA 的高斯公式与 `blur.v:298` 数值相同，但 VGA 版对 `leftdown` 也是 ×2（权重和 17，除数 16）；
- VGA 的浮雕/锐化多了 `>255→255/256` 的饱和分支；`blur.v` 没有。

---

## 5. Python 侧工具（`scripts/`）

### 5.1 `parallel_image_generator.py` —— 生成 9 张邻域图

- 读入一张图（`cv2.imread`），输出 `gray.bmp`（原图副本）+ 8 张 1 像素平移图：
  `up/down/left/right/leftup/leftdown/rightup/rightdown.bmp`，边界补 0
  （`parallel_image_generator.py:108-118`、`_shift_image` 在 `:25-72`）。
- 平移方向语义见 `:107-117` 的注释；**注意 `left.bmp` 是内容右移**。
- 用法：
  ```bash
  python scripts/parallel_image_generator.py <input_image> <output_dir_for_shifted/>
  ```
- 输出是 BMP（`cv2.imwrite`），下游 `kernel_coe_generator.py` 只读它们的**蓝通道**。

### 5.2 `coe_generator.py` —— 逐像素算子用的 COE

- 每像素：`'0'*72 + B(8b) + G(8b) + R(8b)`（`coe_generator.py:89-91`）。
- 写出 2 行头 + N 行向量，逗号分隔，末行分号（`:99-112`）。
- 用法：
  ```bash
  python scripts/coe_generator.py <input_image> <output.coe>
  ```

### 5.3 `kernel_coe_generator.py` —— 卷积算子用的 COE

- 每像素：9×8b 邻域蓝通道（固定顺序 `gray,left,right,up,down,leftup,leftdown,rightup,rightdown`）
  + 原图 24b BGR（`kernel_coe_generator.py:25-28,110-130`）。
- 会校验 9 张邻域图与主图尺寸一致（`:102-104`），缺失/尺寸不符会抛异常。
- 用法：
  ```bash
  python scripts/kernel_coe_generator.py <main_color_image> <shifted_dir/> <output.coe>
  ```
- 三个脚本都用 `argparse`，支持 `--version`（版本号 1.0.0），CLI 错误码非 0。

---

## 6. 测试与验证

### 6.1 `tests/` —— 只测 Python，不测 HDL

三个 `unittest` 文件，共约 31 个用例（`tests/test_coe_generator.py`、`test_kernel_coe_generator.py`、
`test_parallel_image_generator.py`）：

- 成功路径：生成文件、逐行/逐位比对 COE 文本；
- **位级断言样例**：`test_kernel_coe_generator.py:97-121` 手工拼出期望的
  `"memory_initialization_radix=2;"`、`memory_initialization_vector=`，再用
  `_int_to_8bit_binary_str_test_util` 重建 72-bit 邻域 + 24-bit BGR，比较数据行（含末尾 `,`/`;`）；
- 错误路径：输入文件不存在、目录不存在、缺某张邻域图、尺寸不匹配 → 断言抛 `FileNotFoundError`/`ValueError`；
- CLI 测试：用 `subprocess` 跑脚本，检查返回码、stdout 关键字、`--version`（`test_kernel_coe_generator.py:196-236`）。

运行方式（未在本机执行）：
```bash
python -m unittest discover tests
```

### 6.2 HDL 侧的「验证」—— 非自检的文件读写 testbench

`BIPT/BIPT.srcs/sim_1/new/tb_modules.v` 与 `Blurring/Blurring.srcs/sim_1/new/blur_tb.v`：

- 用 `$fopen`/`$fread` 把 BMP **整个文件按字节**读进 `reg [7:0] data[0:500*1024]`（`tb_modules.v:41,56`）；
- 从字节数组里手工解析 BMP 头：`size={data[5..2]}`、`start_pos={data[13..10]}`、
  `width={data[21..18]}`、`height={data[25..22]}`、`bitcount={data[29..28]}`
  （`tb_modules.v:59-67`，小端手工拼接）；
- 逐像素 `red=data[i+2]; green=data[i+1]; blue=data[i];`（BMP 是 BGR），`#10; done_in=1;`
  驱动被测模块（`tb_modules.v:168-175`）；
- 收集 `done_out` 结果到 `result[]`，再用 `$fwrite` 把**原 BMP 头 + 新像素**写成 `result.bmp`
  （`tb_modules.v:91-144,128-144`）；
- `blur_tb.v` 同时读入 9 张 BMP（`blur_tb.v:46-54`），把它们对应像素并排送入 9 组端口。

**结论**：HDL 侧**没有自动断言**，靠导出的 `result.bmp` 人眼/外部工具对比；路径是硬编码的
Windows 绝对路径 `C:\Users\Gowtham\Desktop\...`（`tb_modules.v:38`、`blur_tb.v:46-54,187`）；
`width%4` 才继续（`tb_modules.v:73`），即不支持带行填充的 BMP。**VGA 顶层没有任何 testbench**。

---

## 7. 构建 / 上板流程（从图片到 VGA）

```bash
# 1. 依赖
pip install opencv-python numpy                # README.md:93

# 2. 生成 9 张邻域图
python scripts/parallel_image_generator.py flower.jpg shifted/

# 3. 生成 96-bit 卷积 COE（18400 行）
python scripts/kernel_coe_generator.py flower.jpg shifted/ flower.coe

# 4. 放置 COE：IP 绑定的是这个相对路径
cp flower.coe "Final Project/VGA_1/flower.coe"   # 确认末行以 ';' 结尾
```

5. 打开 `Final Project/VGA_1/VGA_1.xpr`；`image` IP 的 `Coe_File` 已指向 `../../../../flower.coe`
   （`image.xci`），若替换了 COE 需在 Vivado 里重新生成 IP 输出产物。
6. 顶层设为 `vga_syncIndex`，综合 → 实现 → 生成比特流（工程里已有 `vga_syncIndex.bit`）。
7. 用 Vivado Hardware Manager 烧写 Basys3，接 VGA 显示器。
8. 用板上拨码开关选择算子：
   - `reset` = V17（`constr.xdc:14`）
   - `sel_module[3:0]` = **V16, W16, W17, W15**（`constr.xdc:17-28`）
   - `val[7:0]` = **W14, W13, V2, T3, T2, R3, W2, U1**（`constr.xdc:31-49`）
   - VGA：`red[3:0]` = G19/H19/J19/N19，`blue[3:0]` = N18/L18/K18/J18，
     `green[3:0]` = J17/H17/G17/D17，`hsync` = P19，`vsync` = R19（`constr.xdc:246-273`）
   - 主时钟 `clock` = W5（`constr.xdc:7`）
9. 图像会以 160×115 显示在屏幕 (100,100) 处；改图必须先重跑 Python 脚本 + 重建 IP。

PC 仿真流程（`BIPT/`、`Blurring/`）：改 Verilog 里的 `read_fileName` 宏指向本机 BMP →
跑 `tb_modules` / `blur_tb` 的 behavioral simulation → 查看生成的 `result.bmp`（`README.md:149-150`）。

---

## 8. 局限与坑

1. **分辨率/尺寸全硬编码**：160×115 写死在 `hc>=100 && hc<260`、`vc>=100 && vc<215`
   （`VGA.v:113`），BRAM 深度 18400 写死在 IP（`image.xci`）和 `if(addra < 18399)`（`VGA.v:542`）。
   换任何一张图都要同步改 Python 输出尺寸、IP 深度、显示窗口三处。
2. **无缩放、无视频输入**：图像只是静态 ROM 贴屏；没有摄像头/帧缓存写入通路（`wea` 恒 0）。
3. **单通道卷积**：卷积只用邻域图的蓝通道（`kernel_coe_generator.py:116`），本质是灰度处理；
   彩色图被当灰度用（`gray.bmp` 未转灰，`parallel_image_generator.py:109`）。
4. **存储爆炸**：为了省掉 line buffer，用 9× 存储换 1 px/clk，占掉 **≈96% 的 BRAM**；
   分辨率再大一点就放不下（§4.1 推算）。
5. **定点精度损失**：所有除法截断（`/9`、`/3`、`/16`）；灰度系数和只有 30/32；
   输出再 `/16` 到 4-bit → 每通道仅 16 级灰阶（`VGA.v:53,151-153`）。
6. **高斯核权重和是 17，却除以 16**：`blur.v:298-299` 与 `VGA.v:526-527` 中 `leftdown` 被赋权 2
   （标准高斯四角都应为 1），DC 增益 17/16 ≈ 1.06，平坦区域会略微变亮（推算）。
7. **锐化的白色饱和写成 256**：`VGA.v:480-482` 把 `red_o/green_o/blue_o` 写 256，随后 `/16` 得 16，
   取 `[3:0]` 得 0 → **本该最亮的地方反而变黑**。
8. **减亮度/滤色用 `>256` 判负**（`VGA.v:202,212,256,278,300`）：因为 `reg [15:0]` 回绕，
   负值必然很大，恰好能工作，但阈值应为 0 或 255，属脆弱写法；`Modules.v`/`blur.v` 用 `>255`/`>2048`。
9. **阻塞/非阻塞混用**：`VGA.v` 里对 `red_o`、`red`、`addra` 大量用 `=`（如 `147-153`、`542-545`），
   `blur.v` 对 `red_x` 用 `=` 对输出用 `<=`（`blur.v:77-85`）。`red_o = red_o/16; red = {red_o[3:0]};`
   依赖语句顺序，仿真/综合虽一致但可读性与可移植性差。
10. **VGA 刷新率与 README 不符**：计数器常数对应 25 MHz/60 Hz，但 100 MHz 输入经 `/2` 得 50 MHz
    像素时钟 → **≈119 Hz**（§4.2 推算），README 却写 60 Hz（`README.md:195`）。且没有 MMCM，
    派生时钟也没写约束，时序是否收敛未知。
11. **帧行数少 1**：`vreset` 在 `vc==523` → 524 行（标准 525），沿用了经典设计的已知偏差
    （`VGA.v:97`）。
12. **两路 50 MHz 的相位关系**：`clk`（BRAM）与 `ec/pixel_clk`（处理）同频反相，
    跨域路径靠半个主周期，未在 `constr.xdc` 中约束（§4.2）。
13. **工具链绑定**：上板工程强依赖 Vivado 的 `blk_mem_gen` IP（`.xci`/`image.v`/VHDL 包），
    没有 IP 就无法用 Icarus/Verilator 直接仿真 `vga_syncIndex`；BIPT/Blurring 才是纯 Verilog。
14. **无 VGA 工程资源报告**：仓库没有 VGA 设计的 `*_utilization_*.rpt`，LUT/BRAM 数字只能按 IP 配置推算。
15. **testbench 不可移植**：硬编码 Windows 路径（`tb_modules.v:38`、`blur_tb.v:46-54`），
    且非自检；BMP 解析假设 `width%4==0`（`tb_modules.v:73`）。
16. **COE 细节**：全部 CRLF；根目录 `flower.coe` **末行缺 `;`**（实测），只有
    `Final Project/VGA_1/flower.coe` 是合法的。
17. **`sel_module` 位宽不一致**：`Modules.v`/`blur.v` 是 3-bit（8 个算子），
    `VGA.v` 是 4-bit（16 个算子），不能直接把 BIPT 的 `sel` 常量搬到上板工程。

---

## 9. 对「二维动态障碍环境模拟器 + CPU/FPGA + 现实小车摄像头」的可借鉴点

结合你项目里已有的 `FpgaPredictionReference`（`bullet_sim/fpga/prediction_block.py:173`，
`predict(state_frame) -> prediction_frame` 的纯函数 golden model）与 README §20 的验收顺序
（`README.md:831-838`：用 golden model 写 HDL 测试向量，要求逐位相同；记录 LUT/FF/BRAM/DSP），
这个仓库能给你的是下面几件**具体可搬**的东西。

### 9.1 关于「行缓冲 + 流水线 + 定点化」——先纠正一个预期

这个项目**没有实现行缓冲（line buffer），也没有像素级流水线**。它用「Python 预先把 3×3 邻域展开成
9 张图，硬件一个 96-bit 字装下整个窗口」来替代。请把它当成**一种极端方案**来理解：

| | 本仓库方案（预展开 96-bit ROM） | camera 流式方案（line buffer） |
|---|---|---|
| 窗口获得方式 | 1 次宽读直接拿到 3×3 | 2–3 条行缓冲 + 移位寄存器拼窗 |
| BRAM | 9× 放大（≈96% 占满） | 与分辨率同阶（2–3 行） |
| 能否处理视频流 | **不能**（数据必须离线预生成） | 能，天然逐拍流入 |
| 每周期像素 | 1 | 1（配流水线后可 ≥1） |
| 适合你 | 静态测试向量、ROM 演示、验证 | **现实小车摄像头感知** |

**可直接照搬的思想**：定点化的「具体做法」而不是「行缓冲代码」。真正能学的是：
`>>` 组合出的定点系数（`VGA.v:143` 的 `(9R+18G+3B)/32`）、用**阈值当符号位**的技巧
（`blur.v:119-127` / `VGA.v:362-370`，因为 `|G| < 1024` 所以 `>1024` 即负数），
以及「负值回绕 + 显式钳位」的处理模式。

### 9.2 你的 FPGA 一定要走 line buffer 路线（有数字支撑）

若将来接摄像头，640×480×8-bit 的**整帧**是 307,200 B = **2.46 Mbit**，
而 Basys3/XC7A35T 的 BRAM 只有 **1.8 Mbit** —— 连一帧灰度都存不下（推算）。
所以「整帧 BRAM 帧缓存」不可行，必须：
- 用 **2–3 条 line buffer**：640×8 bit × 3 = **15.36 Kbit**，占 50 个 RAMB36 的零头；
- 加 **3×3 窗口移位寄存器**（每行 3 个 8-bit + 2 行延迟）；
- 加 **valid/de**（数据有效）流水线，而不是本项目的 `blank` 门控。

### 9.3 定点 golden model ↔ HDL 逐位对照（本项目缺的一环，正是你已有的强项）

本项目验证的最大短板是：**HDL testbench 没有任何断言**，只导出 `result.bmp` 靠肉眼对比（§6.2）。
你已经有 `FpgaPredictionReference`，建议比它做得更彻底：

1. **把 reference 写成整数语义，而不是浮点**：RTL 的除法是「向零截断」还是「floor」，
   负数的补码宽度是多少，决定结果差异。为 reference 增加显式的 `int_bits/frac_bits`
   和 `trunc_toward_zero()`，让 Python 与 Verilog 在**每一步**（不只是最终输出）同语义。
   本项目的 `/9`、`/16`、`>256` 判负就是「语义没写清」的反面教材。
2. **让 golden model 同时产出「激励 + 期望输出」两份 `.mem`/`.coe`**，格式直接用本项目
   `coe_generator.py:97-112` 的 `memory_initialization_radix=2` 头 + 逐行二进制 + 末行 `;`
   （可直接被 `$readmemh`/IP 加载）。
3. **写自检 testbench**：`$readmemh` 激励 → 打 DUT → `$readmemh` 期望 → 逐位
   `if (dut !== exp) $error(...)` 并统计不符个数。这样把「逐位相同」从 README 的一句话
   变成 CI 里能跑的断言。本项目 `tests/` 对 Python 侧已经这么做了（§6.1），HDL 侧照做即可。
4. **边界向量要专门造**：0、255、饱和、全黑、单像素白点、邻域含 0 的图边——本项目因
   预展开补 0，边界像素的卷积是「和零做卷积」，这类系统偏差最容易被肉眼漏掉。

### 9.4 COE / BRAM 初始化流程可以直接照搬的部分

- **COE 文本格式与生成器**：`scripts/coe_generator.py:97-115` 的两个写文件函数可以原样拿来
  当「任意位宽 ROM 初始化器」——把 96-bit 换成你的位宽（如 risk grid 的一个 cell、
  或一行 state frame）即可。
- **IP 参数模板**：`image.xci` 里这几项是宽字 ROM 的关键组合，照抄即可：
  `Memory_Type=Single_Port_RAM`、`Read_Width_A=Write_Width_A=<你的位宽>`、
  `Write_Depth_A=<行数>`、`Load_Init_File=true`、`Coe_File=<相对 .xci 的路径>`、
  `Register_PortA_Output_of_Memory_Primitives=true`（免费换来 1 拍读延迟。
  注意：若你要做 1 px/clk 的流水线，这 1 拍必须算进延迟预算）。
- **陷阱清单**：COE 必须 **末行 `;`**（本项目根目录 `flower.coe` 就漏了）；统一用 **LF** 而不是
  CRLF（本仓库全是 CRLF）；`depth` 不必是 2 的幂（这里是 18400），但地址位宽要够。
- **`wea` 恒定 0 = ROM** 的写法（`VGA.v:57,65`）可以保留作「只读向量表」；一旦要写帧就得改双口。

### 9.5 VGA 部分

- 计数器/同步常数（800×525 那组）可以直接抄 `VGA.v:85-97`，但**不要**抄它的时钟方案：
  用 Vivado Clocking Wizard 生成 25.175 MHz（或你需要的像素时钟），而不是从 100 MHz 硬除 2
  （本项目因此变成 ≈119 Hz，README 却写 60 Hz）。
- Basys3 的 VGA 是 **4-bit/通道（RGB444）**，`basys3` 的 VGA 引脚映射表（`constr.xdc:246-273`）
  和 `clock=W5`、`create_clock -period 10.00`（`constr.xdc:7-9`）可以整段复用。
- 「算子在组合逻辑里一次算完、末端打一拍」这种写法（`VGA.v:111-556`）在 50 MHz 下能过，
  但如果你的 risk grid 运算更重，要按 `Tclk > T_comb` 拆成多级流水，本项目没有可参考的流水级示例。

### 9.6 建议的最小落地顺序

1. 复用 `coe_generator.py` 的写 COE 函数，把你的 `FpgaPredictionReference` 输出（量化成整数）
   导出成一份 `expected.coe`，同时导出 `stimulus.coe`；
2. 先写一个**只有 ROM + 组合运算 + 输出寄存**的 1 px/clk 骨架（学本项目 §4.0 数据通路），
   用 §9.3 的自检 testbench 跑通逐位一致；
3. 再把「整帧 ROM」换成 **line buffer（3 行 BRAM）+ 窗口移位寄存器**，接口改成 `de/valid` 流式，
   这一步是本项目**没有**、但你必须自己写的部分；
4. 摄像头侧先用 `de` 计数对齐行/列，把本项目 `VGA.v:85-107` 的计数器思想平移过来做
   「列地址 → line buffer 写指针」；
5. 最后再接 VGA 或用 UART/以太网把结果送回 CPU 做闭环，接口协议沿用你现有的
   `encode_state/decode_prediction_frame`（`bullet_sim/fpga/prediction_block.py`），别让 RTL 内部
   细节泄漏到协议层。

---

### 附：主要源文件行数（便于定位）

`VGA.v` 559 · `blur.v` 312 · `blur_tb.v` 306 · `constr.xdc` 309 · `Modules.v` 242 ·
`tb_modules.v` 189 · `kernel_coe_generator.py` 225 · `parallel_image_generator.py` 191 ·
`coe_generator.py` 180 · `tests/test_*.py` 240 / 222 / 180
