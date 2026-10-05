# Red_Tracker 技术总结
### Terasic D8M(MIPI) 摄像头 → SDRAM 帧缓冲 → 红色阈值分割 → 最大红色块质心 → 十字准星叠加 → VGA

> **文档约定**
> - 本文是**只读**代码审阅产物：未运行 Quartus、未联网、未修改任何既有文件，仅新增本文件。
> - 每一处结论后标注来源，格式 `文件:行号`（行号以本仓库当前版本为准）。路径相对仓库根
>   `/home/zhang/Bullet_Platform/Outside/Camera/Red_Tracker/`。
> - 凡代码中无据可依或推断得到的，明确标注 **【推断】**；无法确定的标注 **未确认**。
> - 所有"行号"均来自实际读取的文件内容，未凭印象书写。

---

## 1. 一句话定位

Red_Tracker 是 **delhatch** 写的一个 DE2-115（Cyclone IV E）FPGA 红色目标跟踪参考工程：它通过 I2C 配置 Terasic **D8M** 摄像头模块与 MIPI 桥芯片，把摄像头 RAW Bayer 数据流写入 **SDRAM 帧缓冲**，在 VGA 扫描节奏下做**反拜耳 → 红色硬阈值 + 3×3 低通 → 逐行统计最长红色游程 → 输出该游程中点坐标 → 在视频流里画十字准星**，并显示到 VGA。

- 作者：`Del Hatch`（git 提交者，`33526862+delhatch@users.noreply.github.com`），仓库 `https://github.com/delhatch/Red_Tracker.git`（`git log` 输出；`.git/config` 的 `origin`）。
- 许可：**仓库内没有 LICENSE / COPYING / NOTICE 文件**（对根目录做过 `ls -la | grep -i licen|copying`，无匹配；`git ls-files` 共 141 个文件，无许可文件）。因此**本工程整体没有声明许可证**。可确认的只是一部分文件的**厂商授权声明**：
  - `V/VGA_Controller.v:2-19` 是 Terasic Technologies Inc. 的版权头，授权范围明确写为"for use in synthesis for all Terasic Development Boards and Altera Development Kits"，并声明"Other use of this code, including the selling, duplication, or modification of any portion is strictly prohibited"。
  - `V_D8M/RAW2RGB_J.v`、`V_D8M/Line_Buffer_J.v`、`V_D8M/int_line.v`、`V_Sdram_Control/*.v`、`V/pll_test.v`、`V/sdram_pll.v`、`V/SEG7_LUT*.v` 同属 Terasic/Altera 参考设计系列（例如 `V/pll_test.v:19-29` 为 Altera 版权头）。
  - 作者自有代码（`DE2_115_D8M_RTL.v`、`BALL_DETECTOR/*`、`Mod_counter.v`）**没有任何许可证声明**。
- README 功能描述（`README.md:3-9,11`，原文）："This project interfaces with a D8M camera module by Terasic. It detects any red object, and tracks it." / "The incoming camera video (at 60 fps) is filtered for red pixels and creates a frame buffer." / "It then finds the center of the largest red mass, and overlays a crosshair on it."
- README 自述的 8 步（`README.md:13-30`）：配置并连接摄像头 → 原始视频入 SDRAM 帧缓冲 → 同时检测红像素并建第二个帧缓冲 → 对该视频低通滤波 → 在建/滤第二帧缓冲时检测最大红色块 → 生成红块中心 x,y → 在 VGA 输出上叠加十字准星 → FPGA 同时做 VGA 帧缓冲与 VGA 波形。

> **诚实修正**：README 的 a)~h) 里"FPGA also has a VGA frame buffer"的措辞容易误导。逐行读代码后，**VGA 没有独立的帧缓冲**：显示数据来自同一个 SDRAM 帧缓冲（`DE2_115_D8M_RTL.v:277` 的 `RD1_DATA`），由 VGA 的 `oRequest` 节流读出（`V/VGA_Controller.v:163-171`）。"第二个帧缓冲"指的是片内 MRAM 里的 1 bit/像素红色掩膜（`ball_ram`，`ball_ram.v:97-98` `numwords_a = 307200`、`width_a = 1`）。

---

## 2. 目标平台与工具链

### 2.1 器件与开发板

- 器件：`EP4CE115F29C7`，family `Cyclone IV E`，FBGA 封装 780 引脚，速度等级 7（`DE2_115_D8M_RTL.qsf:5,6,11,12,13`）。即 Altera/Intel **DE2-115** 评估板上的 Cyclone IV E EP4CE115F29。
- 顶层实体：`DE2_115_D8M_RTL`（`DE2_115_D8M_RTL.qsf:7`）。
- 工具版本：`ORIGINAL_QUARTUS_VERSION 15.1.0`、`LAST_QUARTUS_VERSION "17.1.0 Lite Edition"`（`DE2_115_D8M_RTL.qsf:8,9`）；IP 生成器版本 17.1（`ball_ram.qip:2`）；编译报告为 `Quartus Prime Version : 17.1.0 Build 590 10/25/2017 SJ Lite Edition`（`output_files/DE2_115_D8M_RTL.fit.summary`）。
- 板级时钟：`CLOCK_50` (PIN_Y2)、`CLOCK2_50` (PIN_AG14)、`CLOCK3_50` (PIN_AG15)，全部 3.3-V LVTTL，SDC 里按 20 ns 约束（`DE2_115_D8M_RTL.qsf:23-28`；`DE2_115_D8M_RTL.sdc:9-11`）。
- 本设计实际只用了 `CLOCK2_50` 作为唯一系统时钟源（`DE2_115_D8M_RTL.v:152,163,174,255,337,347`）。`CLOCK_50`/`CLOCK3_50` 只出现在引脚与 SDC 里，**逻辑中未使用**（对 `DE2_115_D8M_RTL.v` 全文检索无引用）。

### 2.2 摄像头模块

- **Terasic D8M**，通过 DE2-115 的 GPIO 排针接入，顶层端口组注释即 `//////////// GPIO, GPIO connect to D8M-GPIO //////////`（`DE2_115_D8M_RTL.v:62`）。
- D8M 是 **MIPI CSI-2 摄像头 + 并口桥**结构：FPGA 侧看到的不是 MIPI 差分线，而是桥芯片解出的**并行像素总线** `MIPI_PIXEL_D[9:0]` + `MIPI_PIXEL_CLK` + `MIPI_PIXEL_HS` + `MIPI_PIXEL_VS`（`DE2_115_D8M_RTL.v:70-73`）；MIPI 差分对本身在板/模块内部，不在 FPGA 端口上。
- FPGA 侧另有 `MIPI_MCLK`（给桥/传感器的参考时钟输出）、`MIPI_REFCLK`（20 MHz，见 2.5）、`MIPI_RESET_n`、`MIPI_CS_n`，以及两组 I2C：桥芯片 `MIPI_I2C_SCL/SDA` 与传感器 `CAMERA_I2C_SCL/SDA`（`DE2_115_D8M_RTL.v:63-75`）。
- 传感器型号：代码中未直接写型号。传感器 ID 寄存器指针为 `16'h300b`，通过判据 `ID1 != 8'h88` 才继续写表（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:43,116`）；`0x300B/0x300C = 0x88/0x65` 的组合与 OmniVision **OV8865**（8 MP，ID 0x8865）一致，**但代码/注释里没有写型号，故此归属为【推断】，严格来说"未确认"**。
- 关键配置：输出 640×480（`16'h3808=0x02, 16'h3809=0x80` → 0x0280=640；`16'h380a=0x01, 16'h380b=0xE0` → 0x01E0=480，注释 `// 60 fps (combined with pll settings)`，`V_MIPI_C/MIPI_CAMERA_CONFIG.v:427-430`）；10-bit（`16'h3031=8'h0a // 10-bit`，`:310`）；MIPI 4 lane（`16'h3018=8'h72 // MIPI 4 lane`，`:307`）。
- 帧时序寄存器：`0x380C/HTS_H=0x12`、`0x380D/HTS_L=0x00` → **HTS = 0x1200 = 4608**；`0x380E/VTS_H=0x02`、`0x380F/VTS_L=0x1E` → **VTS = 0x021E = 542**（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:431-434`）。**交叉验算【推断，但数值高度自洽】**：4608 × 542 × 60 fps = **149.85 MHz**，正好落在同表 `0x3638` 注释写的 `// SCLK : 150 MHz`（`:298`）附近 —— 这说明"60 fps"这个数字在寄存器层面是自洽的（前提是传感器内部时序时钟 ≈150 MHz）。
- 表中有**作者亲手改动的痕迹**：`{8'h6c,16'h4011,8'h30}; // DRH: added this line. enables offset` 与 `{8'h6c,16'h4013,8'hcf}; // DRH: added this line. offset of 0xcf.`（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:457-458`，相邻注释还提到修 "gray haze"）→ 说明这张 294 项的表**不是原厂原样**，作者（签名 DRH）调过 BLC/暗电流相关寄存器。
- 重要：图像被配置成**翻转 + 镜像**：`16'h3820=8'h06 // flip on`、`16'h3821=8'h70 // hsync_en_o, fst_vbin, mirror on`（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:440-441`）。这直接影响最终坐标的左右/上下手性，做上位机坐标对齐时**必须先做一次正反手性标定**。

### 2.3 存储

- **板载 SDRAM**：顶层 `DRAM_ADDR[12:0]`、`DRAM_BA[1:0]`、`DRAM_DQ[31:0]`、`DRAM_DQM[3:0]`（`DE2_115_D8M_RTL.v:51-60`），即 32 位数据总线 + 4 字节掩码 + 13 位地址 + 2 位 bank，对应 DE2-115 板上两片 ×16 位 SDRAM 拼成 32 位。**具体容量未在代码中给出**（未确认）；控制器参数按 32 位总线、4 bank、12 位行、8 位列组织（见 4.4）。
- **片内存储**：`ball_ram`（红色掩膜帧，307200×1 bit，`ball_ram.v:97-98`）、`red_line_x`（行缓冲，640×1 bit ×4 实例，`BALL_DETECTOR/red_line_x.v:97-98,104-107`）、`int_line`（反拜耳行缓冲，4096×12 bit ×3 实例，`V_D8M/int_line.v:100-101,106-109`）、`Sdram_WR_FIFO`/`Sdram_RD_FIFO`（1024×32 bit 双时钟 FIFO，`V_Sdram_Control/Sdram_WR_FIFO.v:88-91`）。

### 2.4 Quartus 工程文件族（各自作用）

| 文件 | 作用 | 证据 |
|---|---|---|
| `DE2_115_D8M_RTL.qpf` | Quartus **工程文件**：工程名与版本、`PROJECT_REVISION = "DE2_115_D8M_RTL"`。只有 4 行实质内容。 | `DE2_115_D8M_RTL.qpf` 全文 |
| `DE2_115_D8M_RTL.qsf` | **设置/约束主文件**：family/device/top、源文件清单（`VERILOG_FILE`/`QIP_FILE`/`SOURCE_FILE`/`SDC_FILE`）、223 条有效 `set_location_assignment` 引脚分配（全文 224 行，1 行被注释）、216 条 `set_instance_assignment -name IO_STANDARD` I/O 电平。 | `DE2_115_D8M_RTL.qsf:5-7,23-28,504-559` |
| `DE2_115_D8M_RTL.sdc` | **时序约束文件**：仅 3 条 `create_clock`（三个 50 MHz 输入）+ `derive_pll_clocks` + `derive_clock_uncertainty`；其余小节（false path / multicycle / clock groups / input & output delay）**全是空注释块**。 | `DE2_115_D8M_RTL.sdc:9-11,16,29` 及 33-86 行空块 |
| `DE2_115_D8M_RTL.qws` | **工作区/界面状态文件**（二进制）：记录上次打开的源文件（`de2_115_d8m_rtl.v`、`BALL_DETECTOR/ball_detector.v`、`BALL_DETECTOR/red_frame.v`）与文本编辑器窗口布局。对综合无影响。 | `DE2_115_D8M_RTL.qws`（二进制，可见上述文件名串） |
| `DE2_115_D8M_RTL_assignment_defaults.qdf` | Quartus 自动生成的**默认分配文档**（每类 assignment 的出厂默认值说明），非用户约束，**不参与综合**。 | 文件头 `# Quartus Prime Version 17.1.0 ...` |
| `ball_ram.qip` / `BALL_DETECTOR/red_line_x.qip` 等 | **IP 集成文件**：把 wizard 生成的 `*.v` 与黑盒 `*_bb.v` 挂进 `qsf`。 | `ball_ram.qip:3-4` |
| `PLLJ_PLLSPE_INFO.txt` | Altera PLL 仿真/特性数据文件（PLL J 系列 PLLSPE 参数表）。本工程未在 `qsf` 中引用，属工具附带资料。 | 文件名与 `qsf` 源文件清单对比（504-558 行无引用） |

### 2.5 引脚与时序约束

- 引脚分配：全文 224 行，其中 `qsf:465` 被注释，**生效 223 条**；I/O 电平 216 条（另有 1 条 `PARTITION_HIERARCHY`）。电平分两类：3.3-V LVTTL 用于时钟、VGA、SDRAM、摄像头 GPIO；2.5 V 用于 `LEDG`/`LEDR`（`DE2_115_D8M_RTL.qsf:17,32-50,255-264`）。
- 关键引脚（节选，均出自 `DE2_115_D8M_RTL.qsf`）：
  - 时钟：`CLOCK_50=PIN_Y2`(:28)、`CLOCK2_50=PIN_AG14`(:26)、`CLOCK3_50=PIN_AG15`(:27)
  - 摄像头并行口：`MIPI_PIXEL_CLK=PIN_AC15`(:466)、`MIPI_PIXEL_D[0]=PIN_Y17`(:467) … `MIPI_PIXEL_D[9]=PIN_AD19`(:476)、`MIPI_PIXEL_HS=PIN_AG25`(:477)、`MIPI_PIXEL_VS=PIN_AF22`(:478)、`MIPI_REFCLK=PIN_AE22`(:479)、`MIPI_RESET_n=PIN_AH25`(:480)
  - I2C：`CAMERA_I2C_SCL=PIN_AG22`(:457)、`CAMERA_I2C_SDA=PIN_AE24`(:458)、`MIPI_I2C_SCL=PIN_AE20`(:461)、`MIPI_I2C_SDA=PIN_AG23`(:462)
  - VGA：`VGA_CLK=PIN_A12`(:293)、`VGA_HS=PIN_G13`(:302)、`VGA_VS=PIN_C13`(:312)、`VGA_R[7:0]`(:303-310)、`VGA_G[7:0]`(:294-301)、`VGA_B[7:0]`(:285-292)
  - SDRAM：`DRAM_CLK=PIN_AE5`(:391)、`DRAM_ADDR[12:0]`(:374-386)、`DRAM_DQ[31:0]`(:393-424)、`DRAM_DQM[3:0]`(:425-428)
- **已发现的引脚"改动痕迹"**：`V_MIPI` 像素时钟原引脚被注释掉并改用另一引脚——
  `#set_location_assignment PIN_AB22 -to MIPI_PIXEL_CLK` 紧跟 `set_location_assignment PIN_AC15 -to MIPI_PIXEL_CLK`（`DE2_115_D8M_RTL.qsf:465-466`）。说明作者曾改线，硬件接线与官方例程不同。
- `.sdc` 实际约束内容（逐条，`DE2_115_D8M_RTL.sdc`）：
  1. `create_clock -period 20.000ns [get_ports CLOCK2_50]`(:9)
  2. `create_clock -period 20.000ns [get_ports CLOCK3_50]`(:10)
  3. `create_clock -period 20.000ns [get_ports CLOCK_50]`(:11)
  4. `derive_pll_clocks`(:16) —— 自动为两个 PLL 的输出生成时钟
  5. `derive_clock_uncertainty`(:29)
- **约束缺口（重要，见第 8 节）**：  - **`MIPI_PIXEL_CLK` 没有 `create_clock`**：它是外部摄像头送进来的真实时钟，却只被当作普通输入引脚。全文只有上述 3 个 `create_clock`，没有第 4 个。
  - 没有任何 `set_false_path` / `set_clock_groups -asynchronous` / `set_max_delay -datapath_only`：跨时钟域路径会被 TimeQuest 按同步路径悲观分析。这正是 6.2 节负 slack 的来源。
- 其他 `.qsf` 事实：**没有** `SEARCH_PATH`（所有路径都相对工程根）；结温约束为 `MIN_CORE_JUNCTION_TEMP 0` / `MAX_CORE_JUNCTION_TEMP 85`（`DE2_115_D8M_RTL.qsf:491-492`），所以 6.2 节的 85 °C 模型才是本工程声明的最高工作点；**没有**任何 unused/reserve pin 设置。

---

## 3. 目录与模块地图（作者自有代码 vs 厂商参考代码）

### 3.1 作者自有代码（delhatch）

| 文件 | 行数 | 角色 |
|---|---|---|
| `DE2_115_D8M_RTL.v` | 354 | **顶层**：端口定义、复位中继、I2C 配置例化、两个 PLL、SDRAM 控制器、`ball_detector`、`RAW2RGB_J`、VGA 控制器、FPS 显示、LED 调试 |
| `Mod_counter.v` | 24 | 通用模 N 计数器（带 `clk_en` / `max_tick`），注释自述参考 "FPGA Prototyping by Verilog Examples" pg.192（`Mod_counter.v:1`）。**这是作者引入的第三方教材代码**，非 Terasic |
| `BALL_DETECTOR/ball_detector.v` | 113 | 红色检测/叠加的**封装层**：例化 `red_frame`、`ball_ram`、3 个 `Mod_counter`、输出视频 Mux |
| `BALL_DETECTOR/red_frame.v` | 135 | **核心**：红色阈值 + 3×3 多数表决低通 + 逐行最长红游程统计 + 帧末坐标输出；内含 3 状态 FSM |
| `BALL_DETECTOR/red_line_buffer.v` | 117 | 4 个 1 bit 行缓冲 + 旋转写指针 + 3 抽头选择；**红色判据实际写在这里**（`red_line_buffer.v:34`） |
| `BALL_DETECTOR/red_line_x.v` | 213 | Quartus wizard 生成的 altsyncram：640×1 bit 双口 RAM（`red_line_x.v:97-107`）。
| `ball_ram.v` | 220 | Quartus wizard 生成的 altsyncram：307200×1 bit 双口 RAM（`ball_ram.v:97-108`） |
| `Mod_counter.v.bak` / `de2_115_d8m_rtl.v.bak` | — | 作者调试留下的备份（`.bak`）。`de2_115_d8m_rtl.v.bak` 是**未修改的 Terasic 原始顶层**，与当前顶层 diff 可见作者加的 `EX_IO`、`VGA_ADDRESS`、`R_to_vga/G_to_vga/B_to_vga`、`H_active_area`/`V_active_area`、`ball_detector` 例化等 |

### 3.2 厂商 / 第三方参考代码

`V*` 目录**不是"多种变体任选其一"**，而是 Terasic DE2-115 D8M 参考设计的**多套版本混在一起**：`V_D8M`、`V_Sdram_Control` 是**实际被编译并使用的**主机版本；`V_MIPI_B`/`V_MIPI_C` 是被 `V_D8M` 内部调用的**子模块集合**；`V_Auto`/`V_VCM` 是**自动对焦（VCM 音圈马达）**相关代码，只有两个模块被顶层使用。

模块→文件映射（`grep -rn "^module"` 全仓库结果）与**实际编译清单**（`DE2_115_D8M_RTL.qsf:504-558`）交叉后：

| 目录 | 内容 | 是否在 `qsf` 编译清单 | 是否被顶层/子模块真正例化 |
|---|---|---|---|
| `V/` | `VGA_Controller.v`(243)、`VGA_Param.h`、`RESET_DELAY.v`(80)、`CLOCKMEM.v`(22)、`FpsMonitor.v`(82)、`SEG7_LUT.v`、`SEG7_LUT_8.v`、`pll_test*`、`sdram_pll*` | 是（`pll_test.v`、`RESET_DELAY.v`、`CLOCKMEM.v`、`VGA_Controller.v`、`sdram_pll.v`、`FpsMonitor.v` 均列出，:507-513） | `VGA_Controller`、`RESET_DELAY`、`CLOCKMEM`、`FpsMonitor`、两 PLL 被例化；`SEG7_LUT*` **未使用**（未列入 qsf） |
| `V_D8M/` | `RE_TRIGGER.v`、`RAW_RGB_BIN.v`、`Line_Buffer_J.v`、`RAW2RGB_J.v`、`MIPI_BRIDGE_CAMERA_Config.v`、`RAM_READ_COUNTER.v`、`int_line.v`(+`_bb`) | `RE_TRIGGER.v`、`RAW_RGB_BIN.v`、`RAW2RGB_J.v`、`Line_Buffer_J.v`、`int_line.v`、`MIPI_BRIDGE_CAMERA_Config.v`（:505,514-518） | `RAW2RGB_J`(:314)、`Line_Buffer_J`←`RAW2RGB_J:72`、`int_line`←`Line_Buffer_J:56,67,78`、`MIPI_BRIDGE_CAMERA_Config`(:161)、`RE_TRIGGER`(:183，**但输出无消费者**，见 4.2)。`RAW_RGB_BIN.v` **未被任何模块例化**；`RAM_READ_COUNTER.v` **未被例化且不在 qsf**（全仓库只有其 `module` 声明一行） |
| `V_MIPI_B/` | `MIPI_BRIDGE_CONFIG.v`(328)、`I2C_WRITE_2BYTE_B.v`、`I2C_READ_2BYTE_B.v`、`I2C_WRITE_2POINTER_B.v`、`I2C_DELAY_B.v`、`LCD_COUNTER_B.v` | 全部列出（:535-540） | **`MIPI_BRIDGE_CONFIG` 被真正例化**（`V_D8M/MIPI_BRIDGE_CAMERA_Config.v:25`）。其余是它的 I2C 子模块（`MIPI_BRIDGE_CONFIG.v:257,284,309`） |
| `V_MIPI_C/` | `MIPI_CAMERA_CONFIG.v`(592)、`I2C_WRITE_2BYTE_C.v`、`I2C_READ_2BYTE_C.v`、`I2C_READ_BYTE_C.v`、`I2C_WRITE_2POINTER_C.v`、`I2C_DELAY_C.v`、`LCD_COUNTER_C.v`、`MIPI_CAMERA_CONFIG - Copy.v` | 除 `- Copy.v` 外全部列出（:530-534） | **`MIPI_CAMERA_CONFIG` 被真正例化**（`V_D8M/MIPI_BRIDGE_CAMERA_Config.v:13`） |
| `V_Auto/` | `AUTO_FOCUS_ON.v`、`FOCUS_ADJ.v`、`VCM_CTRL_P.v`、`VCM_I2C.v`、`F_VCM.v`、`F_BACK.v`、`MODIFY_SYNC.v`、`AUTO_SYNC_MODIFY.v`、`LCD_COUNTER.v`、`I2C_*.v` | 全部列出（:541-553） | 只有 `AUTO_FOCUS_ON`（顶层 :195）与 `FOCUS_ADJ`（顶层 :203）被例化；其余（`VCM_I2C`、`F_VCM`、`F_BACK`、`MODIFY_SYNC`、`AUTO_SYNC_MODIFY`、`LCD_COUNTER` 等）**未被例化，属死代码** |
| `V_VCM/` | `VCM_I2C_VR.v`、`I2C_WRITE_BYTE_VR.v`、`I2C_WRITE_POINTER_VR.v`、`I2C_READ_2BYTE_VR.v` | 全部列出（:526-529） | **全部未被例化**，是 `V_Auto` 自动对焦方案的另一个变体（VR 后缀），纯死代码 |

> **结论**：作者自有逻辑只有**顶层 + `BALL_DETECTOR/` + `Mod_counter.v` + 两个 `altsyncram` 实例**；其余几乎全部是 Terasic/Altera 参考设计（摄像头配置、MIPI 桥配置、SDRAM 控制器、VGA 控制器、PLL、自动对焦）。**红色检测 + 十字准星这一"有意义的部分"是作者原创**，占整个工程代码量的约 1/5（约 600 行 / 3000+ 行）。

### 3.3 工程文件集的两处硬伤（只读发现，未编译验证）

1. `DE2_115_D8M_RTL.qsf:537` 引用了 **`V_MIPI_B/MIPI_B_I2C.v`，但该文件既不在磁盘上、也不在 `git ls-files` 里**，全仓库也没有 `module MIPI_B_I2C` 定义（`grep -rn "module MIPI_B_I2C"` 无结果）。同时在 :506 引用了 `DE2_115_D8M_RTL.SDC`（大写），磁盘上只有小写 `.sdc`（在大小写敏感文件系统上同样会找不到）。
2. **磁盘上存在 3 组同名模块定义（复制粘贴未改名）**，但当前 `.qsf` 恰好每组只收了一个，所以**尚未真正冲突**——这是一个"随时会踩的地雷"：
   - `V_MIPI_B/I2C_DELAY_B.v:1` 定义 `module I2C_DELAY_B`；`V_MIPI_C/I2C_DELAY_C.v:1` **也**定义 `module I2C_DELAY_B`。qsf 只收了 `_B` 版（`qsf:538`），`_C` 版未列入。
   - `V_MIPI_B/LCD_COUNTER_B.v:1` 与 `V_MIPI_C/LCD_COUNTER_C.v:1` **都**定义 `module LCD_COUNTER_B`。两者**都未被 qsf 列入**（`qsf:541-553` 里只有 `V_Auto/LCD_COUNTER.v`）。
   - `V_MIPI_C/MIPI_CAMERA_CONFIG.v:1` 与 `V_MIPI_C/MIPI_CAMERA_CONFIG - Copy.v:1` **都**定义 `module MIPI_CAMERA_CONFIG`。qsf 只收了主版（`qsf:530`），`- Copy.v` 未列入。
   - 结论：**只要有人"看起来是多余文件"就手工把它们加进工程，立刻会报重复模块定义**；反过来，仅凭目录里同名文件就断定"工程编译不了"是不对的（我最初也误判过，此处更正）。
   - **另有 1 处已编译但从不被例化的死模块**：`V_MIPI_B/I2C_DELAY_B.v`（在 `qsf:538` 中）在全工程无任何实例化。
3. **真正导致"直接 clone 后重建预期失败"的只有第 1 条的两处引用**（缺失的 `MIPI_B_I2C.v` 与大小写不符的 `.SDC`）。我没有运行 Quartus（按任务要求），因此结论写为"文件集存在缺失引用与潜在重复定义，直接重建**预期报错**（未实测）"。

---

## 4. 完整数据通路逐级拆解

### 4.1 总览

```
 [传感器] ══MIPI 差分(模块内部,不在 FPGA 端口)══► [MIPI 桥芯片]
    ▲ I2C 0x6C (V_MIPI_C)                        │ I2C 0x1C (V_MIPI_B)
    │                                            ▼
 CAMERA_I2C_SCL/SDA          MIPI_PIXEL_D[9:0] @MIPI_PIXEL_CLK(≈25MHz) + HS/VS
                                                 │ (RE_TRIGGER 已例化但输出无消费者=死逻辑)
                    Sdram_Control WR1: wrclk=MIPI_PIXEL_CLK, wrreq=HS&VS
                                                 ▼
                      [Sdram_WR_FIFO dcfifo 1024×32]  ← CDC ①
                                                 │ rdclk = SDRAM_CTRL_CLK 100MHz
              ┌───── 仲裁器: 固定优先级/写优先, 突发 256 字/263 拍, 非抢占 ─────┐
              │                    SDRAM 字地址 0 .. 307199                     │
              └──────────────────────────────────────────────────────────────┘
                                                 │
                      [Sdram_RD_FIFO dcfifo 1024×32]  ← CDC ②
                                                 │ rdclk = VGA_CLK 25.180MHz
                                                 ▼ RD1_DATA[9:0] (rdreq=READ_Request)
                          RAW2RGB_J 反拜耳(3 行缓冲, 实际单时钟域 VGA_CLK)
                                                 ▼ {RED[11:4],GREEN[11:4],BLUE[11:4]} (24bit)
        ball_detector → red_line_buffer → red_frame
          ① floor(R/2)>G && floor(R/2)>B   ② 3×3 多数 sum>=5 → 1bit 掩膜
          ③ 逐行最长红游程 + 帧末算中点坐标
                    │                                    │
        ball_ram(1bit×307200, 帧缓冲)          坐标: vert_line=列, horz_line=行
                    └──────────► 视频 Mux(画十字) ◄──────┘
                                        ▼
                VGA_Controller 640×480@59.95Hz, 以 oRequest 反向节流 SDRAM 读
                                        ▼ VGA_R/G/B[7:0] + HS/VS/BLANK
```

### 4.2 摄像头接口

**接口形态**：并口（不是 FPGA 直接收 MIPI）。桥芯片把 MIPI CSI-2 转成 10 位并行 RAW + 行/场有效信号。

**输入/输出端口**（顶层，`DE2_115_D8M_RTL.v:63-75`）：

| 端口 | 方向/位宽 | 说明 |
|---|---|---|
| `CAMERA_I2C_SCL/SDA` | inout | 传感器 I2C（经一个 2:1 开关切换给桥配置或自动对焦使用，见 `DE2_115_D8M_RTL.v:144-145`） |
| `MIPI_I2C_SCL/SDA` | inout | 桥芯片 I2C |
| `MIPI_MCLK` | output | 参考时钟输出。**顶层未驱动**（`DE2_115_D8M_RTL.v` 中无赋值，也未连到任何子模块） |
| `MIPI_REFCLK` | output | 20 MHz，来自 `pll_ref.c0`（:176） |
| `MIPI_RESET_n` | output | `= RESET_N`（:141），由 `RESET_DELAY` 产生 |
| `MIPI_CS_n` | output | 恒 0（:140） |
| `CAMERA_PWDN_n` | output | 恒 1（:139），即摄像头永不掉电 |
| `MIPI_PIXEL_CLK` | input | 桥输出的像素时钟。**波形频率未在代码中断言**；`DE2_115_D8M_RTL.v:349` 的 `CLOCKMEM` 参数按 `CLK_FREQ=25000000`（25 MHz）使用，桥配置注释也写 `// PCLK 25 MHz`（`V_MIPI_B/MIPI_BRIDGE_CONFIG.v:204`）→ 取 25 MHz |
| `MIPI_PIXEL_D[9:0]` | input | 10 位 RAW Bayer，一像素/时钟 |
| `MIPI_PIXEL_HS` | input | 行有效（LVAL） |
| `MIPI_PIXEL_VS` | input | 场有效（FVAL） |

**I2C 配置（两块芯片，两张表）**

`V_D8M/MIPI_BRIDGE_CAMERA_Config.v` 本身**只是 wrapper**（33 行），内部并列例化两个配置器，`TR_IN`/`INT_n` 悬空（:15,:19,:31）：

1. **传感器表**：`V_MIPI_C/MIPI_CAMERA_CONFIG.v`（593 行）
   - 从机地址 `parameter MIPI_I2C_ADDR = 8'h6c`（:42）。写事务格式 `{SLAVE_ADDR[7:0], POINTER[15:0], WORD_DATA[7:0]}` = 8 位从机 + 16 位寄存器 + 8 位数据（:140）。
   - 先读 ID 校验：指针 `0x300b`，判据 `ID1 != 8'h88` 则回到 ST<=1 重试（:43,:78,:116）；通过后才进 ST=28→30 写表。
   - `parameter WORD_NUM_MAX = 314`（:36），但 `case(WCNT)` **实际只有 294 项**（`grep -c 'SLV8_REG16_DATA8<='` = 294，索引 0..293，见 :290-586）。→ WCNT 294..313 无 case 分支，`SLV8_REG16_DATA8` 保持上一拍的值（即 `{8'h6c,16'h0100,8'h01}`），**会重复写 20 次"唤醒/流式"寄存器**。虽无害，但是明显的表长与计数器不一致 bug。
   - 延时用特殊标记实现：`parameter DELAY_TYPE = 8'hAA`（:46），当 `SLV8_REG16_DATA8[31:24] == DELAY_TYPE` 时进入 ST=40 延时分支（:138）。
   - 表内容分簇（每条都有源注释）：软件复位 `0x0103=0x01`(:291-292) → 软待机 `0x0100=0x00`×4(:293-296) → PLL 组 `0x0302=24, 0x0303=0, 0x0304=3, 0x030e/0x030f/0x0312/0x031e`(:301-306) → 时钟分频 `0x3015=0x01`(:307=行号 :307 附近) → MIPI `0x3018=0x72`(:307) → `0x3020=0x93 // clock normal, pclk/1` → `0x3022=0x01` → `0x3031=0x0a // 10-bit` → `0x3106=0x01` → 时序组 `0x3305..0x330f`、`0x3307` → MIPI 组 `0x3641/0x3646/0x3647/0x364a` → 曝光/增益 `0x3500..0x3509` → sensor control `0x3700..0x3708` → 输出尺寸 `0x3808/09/0a/0b = 640×480` → 翻转/镜像 `0x3820=0x06, 0x3821=0x70` → 镜头校正 `0x583d=0xdf` → 白平衡增益 `0x5018=0x19, 0x501a=0x10, 0x501c=0x17` → 最后 `0x0100=0x01 // wake up, streaming`(:586)。
   - I2C 时钟：`CLOCKMEM c1(.CLK(CLK_50), .CLK_FREQ(125), .CK_1HZ(CLK_400K))`（:38）。按 `CLOCKMEM` 逻辑 `CLK_DELAY > CLK_FREQ/2` 翻转（`V/CLOCKMEM.v:12`），周期 ≈ 2×(125/2+1) = 128 个 50 MHz 时钟 → **≈390.6 kHz**，并非精确 400 kHz。（`CLK_FREQ` 参数在此被当作"半周期计数值"，用法与 25 MHz 那几处不同。）

2. **桥芯片表**：`V_MIPI_B/MIPI_BRIDGE_CONFIG.v`（328 行），从机地址 `8'h1C`（:43），格式 `{SLAVE_ADDR, POINTER[15:0], WORD_DATA[15:0]}`（:144），`parameter WORD_NUM_MAX = 13`（:40）。
   - 流程与传感器相反：`ST=0` 直接 `ST<=30`（:68）先写 13 条寄存器，写完（`WCNT == WORD_NUM_MAX`，:166）再进 `ST=1..10` 读一次芯片 ID（指针 `0x0000`，期望 `0x4401`，:46），成功后于 ST=10 置 `MIPI_BRIDGE_CONFIG_RELEASE = 1`（:121）。
   - 寄存器表（完整 13 项，:228-240）：

     | WCNT | 寄存器 | 值 | 代码注释 |
     |---|---|---|---|
     | 0 | `0x0002` | `0x0001` | System Control Register |
     | 1 | — | 延时 `0x0010` | delay |
     | 2 | `0x0002` | `0x0000` | System Control Register |
     | 3 | `0x0016` | `PLLControlRegister0` | PLL Control Register 0 |
     | 4 | `0x0018` | `PLLControlRegister1` | PLL Control Register 1 |
     | 5 | — | 延时 `0x1010` | delay |
     | 6 | `0x0018` | `PLLControlRegister2` | PLL Control Register 1 |
     | 7 | `0x0020` | `PLLControlRegister3` | PLL Control Register 0 |
     | 8 | `0x000C` | `MCLKControlRegister` | MCLK Control Register |
     | 9 | `0x0060` | `0x8006` | （无注释） |
     | 10 | `0x0006` | `FIFO_LEVEL=0x0008` | FIFO Control Register [0~511] |
     | 11 | `0x0008` | `DATA_FORMAT=0x0010` | Data Format Control Register |
     | 12 | `0x0004` | `0x8047` | Configuration Control Register |

     延时项判据：`REG16_DATA16[31:16] == 16'hffff` → 延时 `REG16_DATA16[15:0] * 5` 个 CLK_400K（:142,:176）。
   - 由参数算出的寄存器值（:193-219，代码里是赋值表达式而非字面量）：`FIFO_LEVEL=0x8`、`DATA_FORMAT=0x0010`、`PLL_PRD=1`、`PLL_FBD=39`、`PLL_FRS=1`、`MCLK_HL=1`、`PPICLKDIV=2`、`MCLKREFDIV=2`、`SCLKDIV=0`、`WORDCOUNT=800`。代入 `:215-219` 的移位公式得 **【推断】**：`Reg0=0x1027, Reg1=0x0603, Reg2=0x0613, Reg3=0x028, MCLK=0x0101`。代码注释给出的目标频率为 `REFCLK 20 MHz / PPIrxCLK 100 MHz / PCLK 25 MHz / MCLK 25 MHz`（:202-205）。

**数据格式**：RAW Bayer（10 bit/像素），不是 RGB565。证据链：传感器 `0x3031=0x0a // 10-bit`；顶层把 `SDRAM_RD_DATA[9:0]` 左移 2 位送 `RAW2RGB_J.mCCD_DATA`（`DE2_115_D8M_RTL.v:317`，`{SDRAM_RD_DATA[9:0], 2'b00}`）；反拜耳模块 `RAW2RGB_J` 通过 `mX_Cont[0]` 之类的位置判据轮转 R/G/B（`V_D8M/RAW2RGB_J.v:94`）。**具体 Bayer 相位（RGGB/GRBG…）未在代码注释中说明，未确认**。代码里唯一确定的是 `RAW_RGB_BIN` 的 **2×2 四相位映射表**（`V_D8M/RAW_RGB_BIN.v:38-61`，`X=mX_Cont[0]`、`Y=mY_Cont[0]`）：

| `{Y,X}` | 输出 R | 输出 G | 输出 B |
|---|---|---|---|
| `2'b10` | `D0` | `(rD0+D1)/2` | `rD1` |
| `2'b11` | `rD0` | `(D0+rD1)/2` | `D1` |
| `2'b00` | `D1` | `(D0+rD1)/2` | `rD0` |
| `2'b01` | `rD1` | `(rD0+D1)/2` | `D0` |

其中 G 通道用**算术平均**（位宽 `mCCD_G` 声明为 `[13:0]`，输出时截低 12 位，`V_D8M/RAW2RGB_J.v:38,45`）；`rD0/rD1` 是 `D0/D1` 的一拍延迟，用来补水平邻居（`V_D8M/RAW_RGB_BIN.v:35-36`）。

**⚠ 发现一处 R/B 通道互换（重要，影响颜色语义）**：`RAW2RGB_J` 例化 `RAW_RGB_BIN` 时把输出**交叉接线**——`.R(mCCD_B)`、`.B(mCCD_R)`（`V_D8M/RAW2RGB_J.v:97,99`），而对外输出是 `oRed = mCCD_R`、`oBlue = mCCD_B`（`:44,46`）。代码注释只写 `// De-bayred output. Filtered.`（:97），**没有解释为何交叉**。净效果是**红蓝互换**。它可能与传感器被配置成 `0x3820=0x06 //flip on`、`0x3821=0x70 //…mirror on`（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:440-441`）配套，但**代码无说明，属未确认**。对红色检测的后果很直接：`red_line_buffer.v:34` 拿到的"RED"到底是画面的红还是蓝，取决于这个交叉与 flip/mirror 的组合——**接入自己的系统时必须先用一张纯色卡把 R/G/B 与画面方向一次性标定清楚**。

**关于 `MIPI_PIXEL_CLK` 频率的一处代码内矛盾**：桥芯片配置注释写 `// PCLK 25 MHz`（`V_MIPI_B/MIPI_BRIDGE_CONFIG.v:204`），顶层 `CLOCKMEM` 也按 25 MHz 传参（`DE2_115_D8M_RTL.v:349`）；但传感器寄存器 `0x3638=0xFF` 的注释写 `// PHY_CLK : 600 MHz (data rate,not clock rate // PCLK : 75 MHz // SCLK : 150 MHz`（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:298`）。两者口径不同（一个是传感器内部 PCLK，一个是桥输出的并口 PCLK），**实际并口 PCLK 频率未确认**（既无 SDC 约束、也无实测记录）。

**关键设计事实（与直觉不符，重要）**：`RAW2RGB_J` 在本工程里**是单时钟域**的——顶层把 `.CCD_PIXCLK(VGA_CLK)`（`DE2_115_D8M_RTL.v:316`），`RAW2RGB_J` 内部又把 `Line_Buffer_J` 的 `.CCD_PIXCLK(VGA_CLK)`（`V_D8M/RAW2RGB_J.v:73`），且行缓冲读写地址都取 `mX_Cont`（:76,:81）。`Line_Buffer_J`/`int_line` 的双时钟结构（`.wrclock(CCD_PIXCLK)` / `.rdclock(VGA_CLK)`，`V_D8M/Line_Buffer_J.v:57,61`）在此被接成同一时钟，**没有跨时钟 FIFO**。原因见 4.4：**反拜耳处理的其实是"从 SDRAM 读回、由 VGA 节奏驱动的回放流"，不是实时摄像头流**。

**`RE_TRIGGER` 是死逻辑**：顶层例化（`DE2_115_D8M_RTL.v:183`），输出 `MIPI_PIXEL_D_`/`MIPI_PIXEL_HS_`/`MIPI_PIXEL_VS_` 只出现在声明（:121-123）与连接（:189-191），**没有任何消费者**——唯一的例外是 `MIPI_PIXEL_VS_` 送给了 `FpsMonitor`（:338）。SDRAM 写通路用的是**未重定时**的原始 `MIPI_PIXEL_D/HS/VS`（:267-268）。

### 4.3 时钟域

| 时钟名 | 频率 | 来源 | 证据 |
|---|---|---|---|
| `CLOCK2_50` | 50 MHz | 板载晶振 PIN_AG14 | `DE2_115_D8M_RTL.qsf:26`；`DE2_115_D8M_RTL.sdc:9` |
| `MIPI_REFCLK` | **20.000000 MHz** | `pll_test.c0`。参数：`clk0_divide_by=5, clk0_multiply_by=2` → 50×2/5 = 20 MHz，`inclk0_input_frequency=20000`(ps) | `V/pll_test.v:107,109,116`；顶层 :173-177 |
| `VGA_CLK` | **25.180000 MHz** | `pll_test.c1`。参数：`clk1_divide_by=2500, clk1_multiply_by=1259` → 50×1259/2500 = 25.180 MHz，duty 50% | `V/pll_test.v:111-113`；顶层 :177。另有**未被 qsf 引用**的 17.1 版 IP 变体写 `output_clock_frequency1("25.185185 MHz")`（`V/pll_test/pll_test_0002.v`）——两代文件数值不同，**以被编译的 `V/pll_test.v`（25.180000 MHz）为准** |
| `SDRAM_CTRL_CLK` | **100 MHz, 0°** | `sdram_pll.c0`：`clk0_divide_by=1, multiply_by=2, phase_shift="0"` | `V/sdram_pll.v:111,113,114,120`；顶层 :255-260 |
| `DRAM_CLK` | **100 MHz, −90°** | `sdram_pll.c1`：`divide_by=1, multiply_by=2, phase_shift="-2500"`(ps) = −2.5 ns @10 ns 周期 | `V/sdram_pll.v:115,117,118`；顶层 :258 注释 `//100MHZ -90 degree` |
| `MIPI_PIXEL_CLK` | ≈25 MHz（外部） | 桥芯片输出，**无 PLL、无 SDC 约束** | 顶层 :70,:274；`V_MIPI_B/MIPI_BRIDGE_CONFIG.v:204` |

**共 4 个实际时钟域**：① `MIPI_PIXEL_CLK`（摄像头写侧）② `SDRAM_CTRL_CLK` 100 MHz（SDRAM/控制器/两个 FIFO 的公共侧）③ `VGA_CLK` 25.180 MHz（读侧 + 显示 + 全部图像处理 + 红色检测）④ `CLOCK2_50` 50 MHz（复位、I2C 配置、自动对焦）。

**跨时钟域手法：异步 FIFO（dcfifo），不是双触发器同步器。** 这是本工程最值得学习的一点：

- 跨域点 ①（摄像头→SDRAM）：`Sdram_WR_FIFO`，`dcfifo`，`lpm_width=32`，`lpm_numwords=1024`，`lpm_widthu=10`（1 K 深），`lpm_showahead="ON"`，`rdsync_delaypipe=11`，`wrsync_delaypipe=11`（`V_Sdram_Control/Sdram_WR_FIFO.v:88-99`）。`wrclk=MIPI_PIXEL_CLK`、`rdclk=CLK`(=SDRAM_CTRL_CLK)（`Sdram_Control.v:274-282`）。
- 跨域点 ②（SDRAM→VGA）：`Sdram_RD_FIFO`，同样 `dcfifo` 1024×32，`wrclk=CLK`、`rdclk=RD1_CLK`(=VGA_CLK)（`Sdram_Control.v:294-302`）。
- 跨域点 ③（自动对焦，**反例**）：`FOCUS_ADJ` 用 `CLK_50=CLOCK2_50` 配置 I2C，却直接采 `VIDEO_CLK=VGA_CLK` 域的 `VIDEO_HS/VGA_HS`、`VIDEO_VS/VGA_VS`、`VIDEO_DE/READ_Request`、`iR/iG/iB`（顶层 :203-226）。**这是无同步器的裸跨域**，正是 6.2 节时序不收敛的元凶。
- 复位跨域：`RESET_DELAY` 在 `CLOCK2_50` 域产生 `DLY_RST_0/1/2`（顶层 :150-158），其中 `DLY_RST_0`→SDRAM 写 FIFO 的 `aclr`（:273）、`DLY_RST_1`→读 FIFO 的 `aclr`（:282）、`DLY_RST_2`→`VGA_Controller.iRST_N`（:248）。这些复位信号同样是**无同步器地跨到其它时钟域**（但复位通常可接受，只要满足释放时序；本工程 `oRST_x` 之间刻意错开，见 5.4）。

### 4.4 帧缓冲与仲裁

**SDRAM 组织**（`V_Sdram_Control/Sdram_Params.h`）：

```
`define ROWSTART  8    `define ROWSIZE  12      // 行地址 = SADDR[19:8]  (12 bit → 4096 行)
`define COLSTART  0    `define COLSIZE   8      // 列地址 = SADDR[7:0]   (8 bit  → 256 列)
`define BANKSTART 20   `define BANKSIZE  2      // bank   = SADDR[21:20] (2 bit  → 4 bank)
`define ASIZE     23                     // 控制器内部地址宽度 23 bit
`define DSIZE     32                     // 数据总线宽度 32 bit
```
- 地址映射由 `command.v` 直接切片实现：`rowaddr = SADDR[ROWSTART+ROWSIZE-1:ROWSTART]`、`coladdr = SADDR[7:0]`、`bankaddr = SADDR[21:20]`（`V_Sdram_Control/command.v:124-126`）。
- 时序参数选的是 **100 MHz 一组**（`Sdram_Params.h:30-38`）：`INIT_PER=24000`、`REF_PER=1024`、`SC_CL=3`（CAS latency 3）、`SC_RCD=3`、`SC_RRD=7`、`SC_PM=1`、`SC_BL=1`。由 `SC_PM=1` 得 `SDR_BL = 3'b111`（**full-page 突发**），`SC_CL=3` 得 `SDR_CL=3'b11`，`SDR_BT=1'b0`（顺序模式）（`Sdram_Params.h:54-59`）。
- 容量：`ASIZE=23` 且行/列/bank 只用掉 22 位（`SADDR[21:0]`），`SA` 输出 12 位、`BA` 2 位、`CS_N` 2 位（`Sdram_Control.v:131-136`）。**板上 SDRAM 实际容量未在代码中声明，未确认**；本工程只用其中 `0..307199` 一段。
- 本工程只用**一个** 32 位端口对：`WR1`/`RD1`。`WR2`/`RD2` 在顶层完全未连接，控制器内部的第二对 FIFO 已被注释掉（`Sdram_Control.v:284-292,304-312`），`WR_MASK`/`RD_MASK` 只会有 `2'b01`（`Sdram_Control.v:506,527`）。

**帧缓冲地址空间**（顶层 `DE2_115_D8M_RTL.v:263-295`）：
- `WR1_ADDR=0`、`WR1_MAX_ADDR=640*480=307200`、`WR1_LENGTH=256`
- `RD1_ADDR=0`、`RD1_MAX_ADDR=640*480=307200`、`RD1_LENGTH=256`
- 写侧：`WR1_DATA = MIPI_PIXEL_D[9:0]`（10 位有效，塞进 32 位字 → **22 位浪费**），`WR1 = MIPI_PIXEL_HS & MIPI_PIXEL_VS`，`WR1_CLK = MIPI_PIXEL_CLK`
- 读侧：`RD1_DATA = SDRAM_RD_DATA[9:0]`，`RD1 = READ_Request`（VGA 的 `oRequest`），`RD1_CLK = VGA_CLK`
- 占用空间 = 307200 × 4 B = **1.2 MB**（若按 10 bit/像素紧凑存放只需 384 KB）

**仲裁：固定优先级 + 突发级（256 字）+ 非抢占，不是时分复用。** 逐行读 `Sdram_Control.v:477-555`：

```
条件：mWR==0 && mRD==0 && ST==0 && WR_MASK==0 && RD_MASK==0 && WR1_LOAD==0 && RD1_LOAD==0
  1st  若 write_side_fifo_rusedw1 >= rWR1_LENGTH(256) 且 rWR1_LENGTH != 0
        → 授权写：mADDR<=rWR1_ADDR, mLENGTH<=256, WR_MASK<=2'b01, mWR<=1      (:502-510)
  2nd  else 若 read_side_fifo_wusedw1 <  rRD1_LENGTH(256)
        → 授权读：mADDR<=rRD1_ADDR, mLENGTH<=256, RD_MASK<=2'b01, mRD<=1      (:522-530)
```
要点：
- **写永远优先**；只有当写 FIFO 内的数据不足 256 字时，读请求才可能被授权。因为摄像头写侧与 VGA 读侧速率相近（25 MHz vs 25.180 MHz），读侧仍能持续拿到时间片。
- 授权是**突发级**的：一次授权连续搬 256 个字，中途不被抢占；完成后（`mWR_DONE`/`mRD_DONE`）清 `WR_MASK`/`RD_MASK` 再重新仲裁（:545-554）。
- 地址自动推进也是**按 256 字块**：`rWR1_ADDR += rWR1_LENGTH` 直到 `>= MAX_ADDR-LENGTH` 则回到 `WR1_ADDR`（:444-450）；读侧同理（:460-466）。307200/256 = **1200 个块恰好一帧**。
- **没有显式的帧同步**：读写指针是两个相互独立的循环指针，起始都为 0，仅靠 FIFO 水位耦合。**这是"追踪式（chase）双缓冲"，不是 ping-pong 双缓冲**。若两侧速率失衡（例如换更高 pclk 的摄像头），写指针会追上读指针并撕裂图像。另外 `WR1_LOAD`/`RD1_LOAD`（FIFO 清空）会阻塞新的授权（:493-494）。

**为什么要"两个缓冲"（答案与直觉不同）**：不是两个 SDRAM 区，而是**两种介质、两种用途**：
1. **原始视频帧缓冲 → SDRAM**（1.2 MB，10 bit/像素有效）。摄像头写、VGA 读，靠两个异步 FIFO 解耦。存在这里的目的是**速率/时序解耦**而不是存储容量。
2. **红色掩膜帧缓冲 → 片内 MRAM**：`ball_ram`，`numwords=307200`、`width=1`（307200 bit = 300 Kbit），`DUAL_PORT`，`read_during_write_mode_mixed_ports="OLD_DATA"`，`outdata_reg_b="UNREGISTERED"`，地址 19 位（`ball_ram.v:97-108`）。只存 1 bit/像素，因为它只需要"是不是红"。
3. 另有**行缓冲**若干：`red_line_x` 640×1 bit ×4（`red_line_x.v:97-107`）供 3×3 低通；`int_line` 4096×12 bit ×3（`V_D8M/int_line.v:100-109`）供反拜耳 3 行窗口。注意 `int_line` 只用 640/4096 个地址，**6 倍空间浪费**。

**一个隐蔽的时序后果【推断】**：`ball_ram` 的 `wraddress` 与 `rdaddress` 都接同一个 `vAddress`（`BALL_DETECTOR/ball_detector.v:75,77`）。因为读写在同址同拍发生，而 `altsyncram` 配置为 `OLD_DATA`，读回的是**上一帧**该位置的掩膜 → 掩膜显示天然滞后一整帧。这对于一个"跟踪"演示可以接受，但对闭环控制是有意义的相位滞后。

### 4.5 红色检测

**位置**：两级实现——**阈值判据在 `red_line_buffer.v:34`**，**多数表决/低通在 `red_frame.v:32-42`**。

**判据（硬阈值 + 比例判据，不是 HSV，不是 YUV）**：
```verilog
// BALL_DETECTOR/red_line_buffer.v:34
if( ({1'b0,RED[6:1]} > BLUE[7:0]) && ({1'b0,RED[6:1]} > GREEN[7:0]) ) seems_red = 1'b1; else seems_red = 1'b0;
```
- `{1'b0, RED[6:1]}` 就是 `floor(R/2)`（8 位，右移一位）。
- 等价判据：**`floor(R/2) > G` 且 `floor(R/2) > B`**，即 **`R > 2G + 1` 且 `R > 2B + 1`**（粗略地说 R 至少是 G、B 的两倍）。
- 输入 `RED/GREEN/BLUE` 来自 `pixel[23:16]/[15:8]/[7:0]`（:28-30），即顶层传入的 `{RED[11:4], GREEN[11:4], BLUE[11:4]}`（`DE2_115_D8M_RTL.v:299`）——**12 位被截成高 8 位**，丢失低 4 位精度。
- 特点：**没有任何绝对亮度门限**（所以极暗的像素不会判红），也**没有任何自适应/白平衡归一化**。

**低通（3×3 多数表决）**，`BALL_DETECTOR/red_frame.v`：
- 行缓冲给出"当前填充行上方"的 3 行抽头 `tap_top/tap_middle/tap_bottom`（`red_line_buffer.v:55-77` 的旋转指针 case），再由链式寄存器 `d2_*←d1_*←tap_*`（`red_frame.v:104-111`）补给 2 个水平邻居，构成 3×3 窗口。
- `sum = d2_top+d1_top+tap_top + d2_middle+d1_middle+tap_middle + d2_bottom+d1_bottom+tap_bottom`（`red_frame.v:33-35`，4 位宽）。
- 输出：
  ```verilog
  if (filter_on) red_pixel = (sum >= 5) ? 1'b1 : 1'b0;   // 9 中至少 5 红 → 多数表决
  else          red_pixel = tap_middle;                   // 直通
  ```
  （`red_frame.v:36-42`；`filter_on` 由 `SW[15]` 控制，`ball_detector.v:48` / 顶层 `DE2_115_D8M_RTL.v:309`。）
- 是**完全流水线（组合求和 + 逐拍寄存器）**，无 FSM 参与；代价是**2 行 + 2 像素的延迟**（`red_frame.v:30-31` 注释明确写 "The filtering creates 2 raster lines of latency"）。
- 寄存器/位宽：`sum[3:0]`、`d1_*/d2_*` 各 1 位（:22-23）。整个低通只占极少逻辑。

### 4.6 最大红色块与坐标提取

**算法不是连通域标记，而是"逐行最长红色游程 + 帧内最大值搜索"。** 全部在 `red_frame.v`：

- 计数器与寄存器（`red_frame.v:24-28`）：
  | 寄存器 | 位宽 | 含义 |
  |---|---|---|
  | `cntr` | `[9:0]` | 当前行连续红像素个数（最大 640） |
  | `max_ever` | `[9:0]` | 本帧到目前为止的最大连续红像素数 |
  | `end_x` | `[9:0]` | 创纪录游程结束时所在的列号 |
  | `line_of_max` | `[8:0]` | 含最大游程的行号（0-479） |
  | `state` | `[1:0]` | 3 状态 FSM |
  | `horz_line` | `[8:0]` | **输出**：行坐标（名字误导，实际是 Y） |
  | `vert_line` | `[9:0]` | **输出**：列坐标（名字误导，实际是 X） |
- 统计逻辑（`red_frame.v:47-100`）：`IS_RED` 态里 `red_pixel ? cntr<=cntr+1 : cntr<=0`；第二个 always 块里若 `cntr > max_ever` 则 `end_x<=x_cont; max_ever<=cntr; line_of_max<=y_cont`。
- **坐标计算在帧末、只算一次**（`red_frame.v:63-67`，`WAIT` 态检测 `v_sync==0`）：
  ```verilog
  horz_line <= line_of_max;
  vert_line <= end_x - (max_ever >> 1);
  ```
  - **除法用移位近似**：`max_ever >> 1`，没有除法器 IP，没有乘除法器资源（`fit.summary` 里 `Embedded Multiplier 9-bit elements : 0`，也印证全设计没用 DSP）。
  - 得到的是**最长游程的中点**，不是严格意义的面积质心/几何重心。对凸的、单连通的红球近似成立；对 C 形、分离的、或带孔洞的红色区域会**退化**（"最长一行"决定结果）。
- **更新频率**：每帧一次，即 60 Hz（跟随 VGA 场同步）。
- **有跨帧保持吗？没有平滑。** `max_ever`/`line_of_max` 每帧在 `START_UP` 态清零并回默认值 240（`red_frame.v:87-93`）；上一帧的坐标寄存器 `horz_line`/`vert_line` 一直保持到本帧帧末被覆写 → **帧间是零阶保持（ZOH），没有任何滤波/平滑/预测**。
- **一个真实的边界 bug**：若整帧没有红色像素，`line_of_max` 回默认 240（十字横线居中），但 `vert_line <= end_x - (max_ever>>1)` 中 `max_ever=0`，`end_x` **没有在帧初被复位**（复位列表里只有 `max_ever`/`line_of_max`，`red_frame.v:86-93`）→ 竖线会停在**上一帧遗留的 X 位置**。表现是"没有目标时横线居中、竖线乱指"。

### 4.7 叠加显示（十字准星 + VGA）

**十字准星画法：在像素流里做坐标相等比较（组合 Mux），不写显存。**（`BALL_DETECTOR/ball_detector.v:52-66`）
```verilog
if( (h_value == vert_line) || (v_value == horz_line) )
     video_out = { 8'b0111_1111, 8'b0111_1111, 8'b0111_1111 };  // RGB(127,127,127) 灰白
else video_out = vid_select ? 红色掩膜伪彩 : 摄像头原图;
```
- `h_value`/`v_value` 是当前像素的列/行计数（见下），`vert_line` 是目标列、`horz_line` 是目标行（命名相反，见 4.6）。因此实际画出的是**贯穿全屏的一横一竖两条 1 像素线（"十"字）**，不是十字形标记块。线宽固定 1 像素，无抗锯齿。
- 颜色常量 `8'b0111_1111` = 127/255 的中灰。
- `vid_select = SW[17]`（`DE2_115_D8M_RTL.v:306`）：0 = 显示摄像头原图，1 = 显示红色掩膜伪彩。掩膜伪彩展开为 `{1'b0,{7{bit}},2'b0,{6{bit}},2'b0,{6{bit}}}`（`ball_detector.v:64`），即 R 通道 7 位、G/B 通道 6 位近似重复 → 红色 = 检出，黑 = 未检出。
- `freeze = SW[16]`（`DE2_115_D8M_RTL.v:307`）：置 1 时停止写 `ball_ram`（`wren = active_area & ~freeze`，`ball_detector.v:78`），画面冻结。

**像素坐标计数器**（`BALL_DETECTOR/ball_detector.v:82-104`，均用 `Mod_counter`）：

| 实例 | 参数 | 时钟 | clk_en | reset | 输出 |
|---|---|---|---|---|---|
| `h_count` | `N=10, M=640` | `ball_clock`(=VGA_CLK) | `h_sync` | `~h_sync` | `h_value[9:0]` 0..639 |
| `v_count` | `N=9, M=480` | **`h_sync`**（派生时钟！） | `v_sync` | `~v_sync` | `v_value[8:0]` 0..479 |
| `pixel_count` | `N=19, M=307200` | `ball_clock` | `h_sync & v_sync` | `~v_sync` | 未使用（`q()` 悬空） |

注意 `ball_detector` 的端口命名与实参：`.h_sync(READ_Request)`、`.v_sync(V_active_area)`、`.active_area(READ_Request)`（顶层 `DE2_115_D8M_RTL.v:301-304`）。所以这里的"`h_sync`"其实是 **VGA 有效像素区指示**（`oRequest`，逐行 640 拍），"`v_sync`"是**场有效区指示**（`oVRequest`，480 行）。`v_count` 用 `h_sync` 当时钟、用 `v_sync` 当使能，属于**门控派生时钟**（见第 8 节）。

**VGA 时序**（`V/VGA_Param.h` 全文 + `V/VGA_Controller.v`）：

| 参数 | 值 | 说明 |
|---|---|---|
| `H_SYNC_CYC` | `96+6 = 102` | 行同步脉宽（负极性） |
| `H_SYNC_BACK` | `48-6 = 42` | 行后沿 |
| `H_SYNC_ACT` | 640 | 行有效像素 |
| `H_SYNC_FRONT` | 16 | 行前沿 |
| `H_SYNC_TOTAL` | **800** | 行总周期 |
| `V_SYNC_CYC` | `2+2 = 4` | 场同步脉宽 |
| `V_SYNC_BACK` | `33-2 = 31` | 场后沿 |
| `V_SYNC_ACT` | 480 | 场有效行 |
| `V_SYNC_FRONT` | 10 | 场前沿 |
| `V_SYNC_TOTAL` | **525** | 场总周期 |
| `X_START` | `102+42 = 144` | 行有效起始 |
| `Y_START` | `4+31 = 35` | 场有效起始 |
- 参数自洽性：102+42+640+16 = **800** ✓；4+31+480+10 = **525** ✓。
- **作者对同步脉宽做过改动**：水平 `96+6`/`48-6`、垂直 `2+2`/`33-2`，即把同步脉冲加长、把后沿相应缩短，使 `X_START`(144)/`Y_START`(35) 仍等于标准的 96+48 / 2+33（`V/VGA_Param.h:2-3,12-13,19-20`）。`H_SYNC_FRONT`/`V_SYNC_FRONT` 在 `VGA_Controller.v` 中**从未被引用**。
- 帧率核验【推断】：25.180000 MHz / (800 × 525) = **59.95 Hz**，与"60 fps"一致。
- 计数器：`H_Cont`/`V_Cont` 均 **13 位**（`V/VGA_Controller.v:87-88`）。有效窗口为 `H_Cont = 144…783`（640 px）、`V_Cont = 35…514`（480 行）。
- 同步极性：`mVGA_H_SYNC` 在 `H_Cont < H_SYNC_CYC` 时为 0，否则 1（:210-213）→ **负极性 HS**；`mVGA_V_SYNC` 同理（:236-239）→ **负极性 VS**；`mVGA_BLANK = mVGA_H_SYNC & mVGA_V_SYNC`（:114）——注意这个信号的语义是"**不在同步脉冲期内**"，因此**在前后沿消隐期它也是 1，并不是通常意义上的消隐信号**（`V/VGA_Controller.v:114,210-213,236-239`）。行/场计数器的推进：`H_Cont` 每 `iCLK` 加 1，到 800 归零（:205-208）；`V_Cont` 仅在 `H_Cont==0` 时加 1，到 525 归零（:228-240）。
- 请求/地址生成（:153-173，均在同一 always 块内寄存）：
  ```verilog
  oRequest <= 1;                                   // 有效区内
  oCoord_X <= H_Cont - X_START;
  oCoord_Y <= V_Cont - Y_START;
  oAddress <= oCoord_Y * H_SYNC_ACT + oCoord_X - 3; // 20 位
  ```
  - `oAddress` 声明 **20 位**（:74）。`*640` 的实现是乘常数（Quartus 用移位加法，不耗 DSP），最大 307199 < 2^19，所以 19 位就够，多出的 1 位恒 0。
  - **`-3` 是流水线补偿**（补偿从地址到 SDRAM 读出数据的 3 拍延迟）。副作用：`oCoord_X < 3` 时该表达式**无符号下溢**，产生接近 2^20 的地址（:168）。因为 `ball_ram` 只取低 19 位，实际落在帧缓冲末尾附近 → **每行最左 3 个像素的取址是错的**。这是作者代码里一个确定存在的边界缺陷（未被注释说明）。
  - `oHRequest`/`oVRequest`（:175-192）在时序 always 块里用了**阻塞赋值 `=`**（:184,:186,:188,:190），与同块的 `<=` 混用；功能上等价于"当拍立即生效"，属风格缺陷。`oHRequest` 只与 `H_Cont` 有关、`oVRequest` 只与 `V_Cont` 有关（两者不互斥），所以 `V_active_area` 在行消隐期也是高的——这正好被 `red_frame` 的 FSM 用 `h_sync==0` 判断行边界来兜住（`red_frame.v:73`）。
- 显示延迟：`oRequest` 寄存 1 拍（:165），`VGA_Controller` 输出的 `oVGA_R/G/B` 又寄存 1 拍（:141-143）→ **请求到显示 2 个 VGA 时钟**；`mVGA_R` 的有效区判据在组合逻辑里（:117-125）。`RAW2RGB_J` 的 `READ_Request` 也直接吃 `oRequest`（顶层 :322）。
- 另外：`ball_detector` 的 `video_out` 送到 `VGA_Controller.iRed/iGreen/iBlue`（顶层 :234-236, 305），而 `VGA_Controller` 的 `iRed/iGreen/iBlue` 端口声明为 **10 位**（`V/VGA_Controller.v:70-72`），实际只接了 8 位并零扩展——高 2 位恒 0，配合 `oVGA_R` 也是 10 位（:77）而 VGA DAC 只有 8 位（顶层 `VGA_R[7:0]`，`DE2_115_D8M_RTL.v:46`），**最终有效精度是 8 位**。

---

## 5. 状态机与控制流

### 5.1 `red_frame` 的 3 状态 FSM（作者自有，最核心的控制逻辑）

编码：`localparam START_UP = 0, WAIT = 1, IS_RED = 2;`（`red_frame.v:19`），寄存器 `state[1:0]`。**异步低有效复位** `always @(posedge VGA_clock or negedge reset)`，复位值 `cntr<=0`、`horz_line<=240`、`state<=START_UP`（:47-52）。

| 状态 | 行为 | 转移条件 | 证据 |
|---|---|---|---|
| `START_UP`(0) | `cntr <= 0`；等待新场开始 | `v_sync==0` → 留在 `START_UP`；否则 → `WAIT` | :55-59 |
| `WAIT`(1) | `cntr <= 0`；等行有效区开始；**帧末在此输出坐标** | `v_sync==0` → 回 `START_UP`，同时 `horz_line<=line_of_max`、`vert_line<=end_x-(max_ever>>1)`；否则 `h_sync==0` → 留 `WAIT`；否则 → `IS_RED` | :61-70 |
| `IS_RED`(2) | 在有效区内数连续红像素 | `h_sync==0` → 回 `WAIT`；否则 `red_pixel==1` → `cntr+1` 留 `IS_RED`；否则 `cntr<=0` 留 `IS_RED` | :72-82 |

**状态数 = 3。** 转移全部依赖 `v_sync`/`h_sync`（实为 VGA 有效区信号）。第二个 always 块（:85-100）是**并行**的"破纪录"逻辑，`state==START_UP` 时复位 `max_ever`/`line_of_max`，否则 `cntr > max_ever` 时更新 `end_x/max_ever/line_of_max`。

### 5.2 `Sdram_Control` 的仲裁序列机

`ST` 是一个 **11 位以上的单变量步进序列**（声明见 `Sdram_Control.v` 内部；`mLENGTH[10:0]` 参与比较，`ST` 至少需 9 位以容纳 263）。它**不是** `IDLE/ACT/READ/...` 那种命名状态机，而是"定长流水计数"：

| ST 值 | 行为 | 证据 |
|---|---|---|
| `0` | **IDLE**：用 `Pre_WR/Pre_RD` 检测请求上升沿。写优先：`!Pre_WR && mWR` → `Write<=1, CMD<=2'b10, ST<=1`；否则 `!Pre_RD && mRD` → `Read<=1, CMD<=2'b01, ST<=1`。无请求则 `ST` 保持 0。 | :360-376 |
| `1` | 等 `CMDACK`（`control_interface` 在 `CM_ACK` 后一拍给出）→ `CMD<=2'b00, ST<=2` | :377-383 |
| `2 … SC_CL+SC_RCD+mLENGTH+1` | `default` 分支：`ST != 3+3+256+1=263` 则 `ST<=ST+1`，到 263 则 `ST<=0` | :384-389 |

`ST` 的声明为 **`reg [9:0] ST;  //Controller status`**（10 位，`V_Sdram_Control/Sdram_Control.v:167`），因此最多 1024 个取值，本工程只用 0..263。

**状态总数 = 264 个 ST 取值（0..263）**，其中真正有分支语义的只有 0 和 1，其余 262 个是等长流水。关键派生时刻：

- 写：`IN_REQ` 在 `ST==SC_CL-1=2` 拉高、在 `ST==SC_CL+mLENGTH-1=258` 拉低；`Write` 在 `ST==SC_CL+SC_RCD+mLENGTH=262` 清 0 并置 `mWR_DONE`（:392-403）。
- 读：`OUT_VALID` 在 `ST==SC_CL+SC_RCD+1=7` 拉高、在 `ST==SC_CL+SC_RCD+mLENGTH+1=263` 拉低并置 `mRD_DONE`（:407-417）。
- 输出侧（:325-338，纯 `posedge CLK` 无复位）：在 `ST==SC_CL+mLENGTH=259` 时人为产生 `SA<=12'h200`、`RAS_N<=0`、`CAS_N<=1`、`WE_N<=0`（即 **PRECHARGE**），`DQM` 在 `Write && ST==SC_CL+mLENGTH` 时置 `2'b11`（写屏蔽最后一拍）→ 这是"256 字突发结束后自动 precharge"的实现。
- **三处值得注意的实现细节（都是真实缺陷或近似）**：
  1. `SA<=12'h200` 的置位实际落在 **`SA[9]`**，而 JEDEC 的 PRECHARGE-ALL 标志位是 **A10**（`12'h400`）。即这条 precharge 的"全 bank"语义**存疑**（`Sdram_Control.v:327`）。是否因此在某些 bank 上漏 precharge，**未确认**（未仿真）。
  2. `PM_DONE <= (ST==SC_CL+SC_RCD+mLENGTH+2)`（:335）的比较值是 `3+3+256+2 = 264`，而 `ST` 最大只到 263 就归零（:385-388）→ **`PM_DONE` 恒为 0**，且 `command.v` 声明该端口后**从未引用**（死信号）。
  3. `DQM <= (...) ? 2'b11 : 2'b0 : 2'b11`（:336）——右值是 **2 位**，但 `DQM` 声明为 `` [`DSIZE/8-1:0] `` 即 **4 位**（:179），零扩展后变成 `4'b0011`。也就是说**"屏蔽"时只屏蔽 `DQ[15:0]`，高 16 位不被屏蔽**。本工程写入的高 16 位恒为 0，实际影响**未确认**（未仿真）。
- 一个完整读写突发 = **263 个 100 MHz 周期 = 2.63 µs / 256 字**（效率 256/263 = 97.3%）【推断，由上述常数直接算出】。
- 复位（`!RESET_N`）：`CMD/ST/Pre_RD/Pre_WR/Read/Write/OUT_VALID/IN_REQ/mWR_DONE/mRD_DONE` 全部清 0（:342-354）。注意顶层的 `RESET_N` 接的是 **`KEY[0]`**（`DE2_115_D8M_RTL.v:264`），也就是说 **SDRAM 控制器直接受按键 0 控制，不受 `RESET_DELAY` 的 `DLY_RST_*` 影响**。

### 5.3 `control_interface` + `command`：初始化/刷新时序（厂商参考）

- `control_interface.v` **不是**状态机，而是"命令译码 + 两个计数器"（`timer[15:0]`、`init_timer[15:0]`，:97-98）：
  - 命令译码：`CMD==3'b000/001/010` → `NOP/READA/WRITEA`（:119-132），`SADDR <= ADDR` 打一拍对齐（:116）。
  - `CMD_ACK`：`CM_ACK==1 && CMD_ACK==0` 时置 1（:144-147）→ **单拍脉冲**。
  - 刷新定时器：`REF_ACK` 或 `INIT_REQ` 时装 `REF_PER`(=1024) 或 `REF_PER+200`，否则自减；减到 0 置 `REF_REQ`（:152-177）。
  - 初始化定时器（:180-237，等价于 8 个阶段的序列）：`init_timer < INIT_PER(24000)` → `INIT_REQ=1`；`==INIT_PER+20` → `PRECHARGE=1`；`INIT_PER+40/60/80/100/120/140/160/180` 共 **8 次** → `REFRESH=1`；`==INIT_PER+200` → `LOAD_MODE=1`。**无命名 FSM，只有"计数值 → 命令"的映射表。**
- `command.v`：把 `NOP/READA/WRITEA/REFRESH/PRECHARGE/LOAD_MODE` 翻译成 `CS_N/RAS_N/CAS_N/WE_N` 并插入延迟。内部无命名状态机，用移位链实现流水：`command_delay[7:0]`、`rw_shift[1:0]`、`oe_shift[6:0]`、`rp_shift[3:0]`、`rp_done`（声明见 `command.v` 内部信号段）。地址切片在 `command.v:124-126`。

### 5.4 复位与初始化控制流（顶层）

`RESET_DELAY u2`（`DE2_115_D8M_RTL.v:150-158`）参数：`iRST=KEY[0]`（**低有效按键**）、`iCLK=CLOCK2_50`、`Cont[31:0]` 计数器（`V/RESET_DELAY.v:52`）。阈值：

| 输出 | 触发计数值 | 时间 @50 MHz【推断】 | 语义（`DE2_115_D8M_RTL.v:149` 注释 "Outputs are active low. high = run."） |
|---|---|---|---|
| `oREADY`/`RESET_N` | `Cont >= 32'hfff00`(=1,048,320) | ≈ 21.0 ms | 释放 MIPI 桥/摄像头复位（`MIPI_RESET_n`，:141） |
| `oRST_0`/`DLY_RST_0` | `Cont >= 32'h1FFFFF`(=2,097,151) | ≈ 41.9 ms | 清 SDRAM **写** FIFO（`WR1_LOAD = !DLY_RST_0`，:273） |
| `oRST_1`/`DLY_RST_1` | `Cont >= 32'h1FFFFF + 80`(=2,097,231) | ≈ 41.9 ms + 1.6 µs | 清 SDRAM **读** FIFO（`RD1_LOAD = !DLY_RST_1`，:282） |
| `oRST_2`/`DLY_RST_2` | `Cont >= 32'h11FFFFF`(=18,874,367) | ≈ 377.5 ms | 释放 `VGA_Controller.iRST_N`（:248） |
| 计数器上限 | `Cont != 32'hffffff0` 才继续加 | ≈ 335.5 ms | 到顶后停止计数 |

**要点**：`DLY_RST_0` 与 `DLY_RST_1` 只差 80 个时钟——刻意让读 FIFO 比写 FIFO 晚一点被释放，避免读侧先请求到空数据。`DLY_RST_2` 晚 377 ms 才放 VGA，保证 SDRAM 里已有若干帧数据后才开始扫描显示。

### 5.5 I2C 配置状态机（厂商参考，编号型 case）

两套结构几乎相同，都是 `case(ST)` 数字分支，**没有命名状态**：

- **桥芯片** `MIPI_BRIDGE_CONFIG.v`：触点状态 `0`（出发点，直接跳 30）→ `30/31`（装配并写一条字）→ `32/33/34/35`（握手、`WCNT++`、未到 13 条则回 31）→ `40/41/42`（延时分支，`DELY == REG16_DATA16[15:0]*5`）→ 回 `31`；写完 13 条后在 `35` 分支 `ST<=1`（:166-168）。读 ID 序列：`1→2→3→4→5→6→7→8→9→10`，其中 ST=9 收数据，ST=10 判定 `CNT==1` 时置 `MIPI_BRIDGE_CONFIG_RELEASE<=1`（:118-128）。**复位**：`always @(negedge RESET_N or posedge CLK_400K)`，`!RESET_N` 时 `ST<=0`、三个 `*_GO<=1`、`WCNT/CNT/DELY<=0`、`..._RELEASE<=0`（:53-64）。注意触发沿是 **`negedge RESET_N`**，与其他模块的 `posedge reset` 不一致。
- **传感器** `MIPI_CAMERA_CONFIG.v`：`0→1→2…→10`（读 ID，ST=10 判 `ID1 != 8'h88` 则回 1，:116）→`28`（延时 5 拍）→`30/31/32/33/34/35`（写 294 条表）→`40/41/42`（延时）。复位同上（:50-61），同样用 `negedge RESET_N`。

---

## 6. 资源与性能

### 6.1 资源占用（`output_files/*.summary`，Quartus Prime 17.1.0，2017-12-10 编译）

| 项目 | 用量 | 器件总量 | 占比 | 来源 |
|---|---|---|---|---|
| Total logic elements | **4,795** | 114,480 | 4 % | `fit.summary` |
| Total combinational functions | 4,286 | 114,480 | 4 % | `fit.summary` |
| Dedicated logic registers | **2,231** | 114,480 | 2 % | `fit.summary` |
| Total pins | 238 | 529 | 45 % | `fit.summary` |
| Total memory bits | **426,496** | 3,981,312 | 11 % | `fit.summary` |
| Embedded Multiplier 9-bit elements | **0** | 532 | 0 % | `fit.summary` |
| Total PLLs | **2** | 4 | 50 % | `fit.summary` |
| （A&S 阶段 LE） | 4,972 | — | — | `map.summary` |

- **没有用任何 DSP/乘法器**：整条红色检测链只有加法、移位、比较（4.5/4.6 节）。这对想"照搬算法"的人是好消息——纯 LUT 逻辑可移植。
- 426,496 memory bits 的主要构成【推断】：`ball_ram` 307,200 bit + 两个 1024×32 dcfifo 65,536 bit + 3×`int_line`(4096×12=49,152 bit each，共 147,456 bit) + 4×`red_line_x`(640 bit each) ≈ 522,752 bit。实测值小于此，说明 Quartus 对 `int_line` 未用到的地址或 FIFO 做了合并/优化。**逐项占用未在报告中给出，未确认。**

### 6.2 时序（`output_files/DE2_115_D8M_RTL.sta.summary`、`.sta.rpt`）

- **Fmax（Slow 1200mV 85C）**（`sta.rpt:170-176`）：`pll_ref|...|clk[1]`（=VGA_CLK）**70.37 MHz**；`CLOCK2_50` 146.22 MHz；`u6|...|clk[0]`（=SDRAM_CTRL_CLK）150.06 MHz；`pll_ref|...|clk[0]`（=MIPI_REFCLK）152.25 MHz。→ **每个时钟域自身的 Fmax 都远高于其工作频率**（VGA 70.37 > 25.19；SDRAM 150.06 > 100）。
- **但 setup slack 是负的**：`Slow 1200mV 85C Model Setup 'pll_ref|...|clk[1]' : Slack = -5.338, TNS = -207.818`（`sta.summary` 第 1 项）。0C 模型 -4.863，Fast 0C 模型 -2.542。Recovery 也是负的（85C：Slack -4.597, TNS -763.733）。
- **负 slack 的根因已定位到具体路径**（`sta.rpt:251-258`，最差路径前几行）：
  ```
  ; -5.338 ; AUTO_FOCUS_ON:vd|PULSE[11] ; FOCUS_ADJ:adl|VCM_CTRL_P:pp|peakSUM[1] ; CLOCK2_50 ; pll_ref|...|clk[1] ; ...
  ```
  → **源寄存器在 `AUTO_FOCUS_ON`（`CLOCK2_50` 域），目的寄存器在 `FOCUS_ADJ` 内部的 `VCM_CTRL_P`（`VGA_CLK` 域）**。这就是 4.3 节跨域点 ③ 的裸跨域路径。因为 `.sdc` 里既没有给这条路径 `set_false_path`，也没有 `set_clock_groups -asynchronous`，TimeQuest 按同步路径要求它在 1 个 VGA 周期内稳定，于是不收敛。
  - 换句话说：**这是一个 CDC 约束缺失问题，不是逻辑太慢的问题**（Fmax 70 MHz 证明同域逻辑很宽裕）。同时这条路径**与红色跟踪功能无关**（自动对焦是 V_Auto 的附加功能）。
- `MIPI_PIXEL_CLK` 没有 `create_clock`（2.5 节）→ 该域（`Sdram_WR_FIFO` 的写侧）**完全没有做时序分析**。写侧逻辑很简单（wrreq = HS&VS），实际风险低，但属于"未验证"而非"已验证通过"。
- Hold 全部为正（最小 0.141 ns @Fast 0C `u6|clk[0]`），Minimum Pulse Width 全为正（最小 4.685 ns）。**未确认编译时 Quartus 是否给出了时序不收敛的硬性警告等级**（`fit.summary` 只写 `Fitter Status : Successful`，Fitter 成功不等于时序收敛）。

### 6.3 延迟与帧率

- **端到端延迟（像素→屏幕）**：由三段主导，**这是推算不是实测**：
  1. 异步 FIFO + SDRAM 突发：写 FIFO 最多积压 256+ 字才触发搬运，一次突发 2.63 µs；读 FIFO 同理 → 量级 **1–5 µs**，且随水位抖动。
  2. 反拜耳 3 行缓冲：约 **2 行** = 2 × (800/25.180 MHz) = **63.5 µs**。
  3. 红色低通 3 行缓冲 + 2 拍：约 **2 行** = **63.5 µs**（`red_frame.v:30-31` 注释亦称 "2 raster lines of latency"）。
  4. `ball_ram` 读写同址 + `OLD_DATA` → 掩膜显示额外滞后**整帧**（16.7 ms）【推断，见 4.4】。
  → 视频通道（非掩膜）典型延迟 **≈ 0.13 ms + FIFO 抖动**；掩膜通道 **≈ 16.8 ms**。
- **处理节拍**：红色检测全流水，**1 像素/VGA 时钟**（25.180 M pixel/s），与 640×480@60（18.43 M pixel/s）相比有约 37% 时钟余量。
- **帧率**：摄像头侧配置 60 fps（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:430` 注释）；显示侧 25.180000 MHz/(800×525) = **59.95 Hz**（4.7 节【推断】）。板上用 `FpsMonitor` 把 `MIPI_PIXEL_VS_` 的实测场频显示在 `HEX1`/`HEX0`（两位十进制，`V/FpsMonitor.v`；顶层 :336-343）。
- **SDRAM 带宽**：每帧写入 307200 字 × 32 bit = 9.83 Mbit，60 fps → 590 Mbit/s ≈ 73.7 MB/s 写 + 同等读 = **约 147 MB/s**。可用带宽按 97.3% 效率 × 100 MHz × 4 B = 389 MB/s【推断】→ **占用约 38%**。若按 10 bit/像素紧凑打包，占用可降到 1/3 以下（见第 8 节）。

---

## 7. 构建与上板流程

> 以下为**从工程文件与代码推断出的操作流程**，我没有实际执行（未运行 Quartus、未上板）。凡推断处标注。

1. **环境**：Windows/Linux + **Quartus Prime 17.1.0 Lite Edition**（`qsf:9`；Lite 版支持 Cyclone IV E）。更低版本（如 15.1）可能可用，但 IP（`ball_ram`/`red_line_x`/两个 PLL/两个 dcfifo）是用 15.1 与 17.1 两代 wizard 生成的，混用版本需重新生成 IP。
2. **打开工程**：`File → Open Project`，选 `DE2_115_D8M_RTL.qpf`。工程只有单个 revision `DE2_115_D8M_RTL`（`qpf:6`）。
3. **先修工程文件集**（强烈建议，见 3.3）：把 `qsf:537` 指向的 `V_MIPI_B/MIPI_B_I2C.v`（不存在）移除；把 `qsf:506` 的 `DE2_115_D8M_RTL.SDC` 改为实际文件名；处理 `I2C_DELAY_B`/`LCD_COUNTER_B` 的重复定义（二选一，或改名）。否则**预期无法编译通过**（未实测）。
4. **重新生成 IP**：`ball_ram.qip`、`BALL_DETECTOR/red_line_x.qip`、`V/pll_test.qip`、`V/sdram_pll.qip`、`V_Sdram_Control/Sdram_WR_FIFO.qip`、`Sdram_RD_FIFO.qip`。注意两个 dcfifo 与 `int_line` 的目标器件族写的是 **`Cyclone V`**（`V_Sdram_Control/Sdram_WR_FIFO.v:87`、`V_D8M/int_line.v:98`），与本工程 `Cyclone IV E` 不符，需重新生成。
5. **引脚**：223 条有效 `set_location_assignment` 已在 `qsf` 中（2.5 节），无需手工分配。特别留意 `MIPI_PIXEL_CLK` 被改到 `PIN_AC15`（`qsf:465-466`），**硬件必须按改后的引脚接线/接模块**。
6. **时序约束**：至少补三条（当前缺失，见 2.5/6.2）：① `create_clock` 给 `MIPI_PIXEL_CLK`；② `set_clock_groups -asynchronous` 或用 DCFIFO 的 `set_false_path` 隔离四个时钟域；③ 给自动对焦那条 `CLOCK2_50 → VGA_CLK` 路径补同步器或 false path。
7. **编译**：`Processing → Start Compilation`。产物在 `output_files/`（`PROJECT_OUTPUT_DIRECTORY output_files`，`qsf:15`）：`.sof`（SRAM 目标）、`.pof`（若需烧 EPCS，本工程未见 `.pof`）、`.pin`（引脚报告）、`.fit/.map/.sta.rpt`。
8. **下载**：`Tools → Programmer`，选 `USB-Blaster`，添加 `output_files/DE2_115_D8M_RTL.sof`，勾选 `Program/Configure`，Start。（`output_files/DE2_115_D8M_RTL.cdf` 是 chain description file，记录了 USB-Blaster 与器件链配置。）
9. **上板验证步骤**（按代码行为推断）：
   - 按一下 **KEY[0]** 释放复位（`KEY[0]` 是 `RESET_DELAY`、`Sdram_Control.RESET_N`、`ball_detector.reset` 的复位源，`DE2_115_D8M_RTL.v:152,264,298`）。等待约 0.4 s（`DLY_RST_2` 阈值）。
   - 看 **HEX1:HEX0** 显示的场频，应接近 **60**（`FpsMonitor`，:336-343）。若为 0，说明 `MIPI_PIXEL_VS_` 没进来（桥/摄像头未配置成功）。
   - 看 **LEDR[1]/LEDR[0]**：`CAMERA_MIPI_RELAESE` 与 `MIPI_BRIDGE_RELEASE`（:352）。两者都为 1 才说明两套 I2C 配置已完成（桥在 ST=10 置位，:121）。**LEDR[9:3] 恒为 0**（`5'h0` 填充，:352）；`LEDR` 是 18 位而表达式只有 10 位，高 8 位零扩展。
   - 看 **LEDR[2]/LEDR[3]/LEDR[4]**：分别是以 `VGA_CLK`(25 MHz)、`MIPI_REFCLK`(20 MHz)、`MIPI_PIXEL_CLK`(按 25 MHz) 为基准分频出的"1 Hz"心跳（`CLOCKMEM ck1/ck2/ck3`，:347-349,352）。**这三个是判断三个时钟域是否都活着的关键指示灯**——若 LEDR[4] 不闪，说明摄像头像素时钟没来。
   - **SW[17]=0**：VGA 显示摄像头原图 + 十字；**SW[17]=1**：显示红色掩膜伪彩 + 十字。
   - **SW[16]=1**：冻结掩膜帧缓冲（画面定住）；**SW[15]=0**：关闭 3×3 低通，直接用单像素阈值（用来观察低通的效果）。
   - 拿一个红色物体在镜头前移动，十字应跟随最长红色游程的中点（约 60 Hz 更新）。
   - **KEY[3]** 加 `AUTO_FOC` 使能自动对焦（:207）；**SW[3]** 选择对焦策略（:210）。注意 `LEDG[8:0]` **在顶层从未被赋值**，恒不亮（不是设计意图，是遗漏）。
10. **资源/时序复核**：编译后看 `output_files/DE2_115_D8M_RTL.fit.summary` 与 `.sta.summary`，对照 6.1/6.2 节。

---

## 8. 局限与坑

### 8.1 算法与鲁棒性

1. **固定硬阈值，对光照极敏感**。判据只有 `floor(R/2) > G && floor(R/2) > B`（`red_line_buffer.v:34`），**没有绝对亮度门限、没有白平衡归一化、没有自适应阈值**。曝光/增益由摄像头内部 AEC/AGC 自动调整（配置里只写了初始值 `0x3500-0x3509`），一旦整体偏暗或偏红，判据立刻失效。
2. **只认红色**，换任何其它颜色（哪怕是高饱和蓝/绿）都不行；对"红色但偏暗/偏紫"的目标容易漏检。
3. **不是真正的质心**。算法是"逐行最长红色游程的中点"（`red_frame.v:66`），不是面积质心、不是连通域质心。对 C 形、分离多目标、被遮挡的红球会给出被最长单行主导的错误坐标。**也没有多目标**——全帧只输出一个坐标。
4. **零平滑**。没有 kalman/低通/帧间预测，输出就是每帧一次的 ZOH（4.6 节）。`vid_select` 切换、`SW` 抖动都会直接反映在坐标上。
5. **无目标时的坐标 bug**：`end_x` 未在帧初复位，导致`vert_line` 保留陈旧值（4.6 节末）。
6. **无畸变校正、无标定**：全流程没有内参/外参、没有镜头畸变模型，输出的是"640×480 像素坐标"，不是物理角度/距离。而且图像被配置成 **flip + mirror**（`0x3820=0x06`、`0x3821=0x70`），坐标手性与常规摄像头**相反**。
7. **只在"最长一行"上取中点**，没有垂直方向的信息，Y 只取那一行的行号——对细长/倾斜目标是偏的。

### 8.2 硬件与工程实现

8. **时序不收敛且原因是 CDC 未约束**（6.2 节）：最差路径 `AUTO_FOCUS_ON(CLOCK2_50) → FOCUS_ADJ/VCM_CTRL_P(VGA_CLK)`，Setup Slack **−5.338 ns**，TNS −207.818，Recovery TNS −763.733。虽然这条路径属附加的自动对焦功能、与红球跟踪无关，但它意味着**这份工程不是"时序干净"的参考**。
9. **`MIPI_PIXEL_CLK` 没有时钟约束**（2.5 节）→ 摄像头写入路径**从未被时序分析**。
10. **`ball_detector` 用派生时钟**：`v_count` 的 `.clk(h_sync)`（`ball_detector.v:91`），即用逻辑信号当行计数器时钟，`Mod_counter` 里又有异步复位 `posedge reset`（`Mod_counter.v:15`）。在 Cyclone IV 上能"跑"，但会产生 gated-clock 警告、时钟偏斜不确定，是**不应照搬**的写法。
11. **`VGA_Controller` 的 3 拍偏移 bug**：`oAddress <= oCoord_Y*640 + oCoord_X - 3`（`V/VGA_Controller.v:168`）在 `oCoord_X<3` 时无符号下溢 → 每行最左 3 个像素取址错误。
12. **`oHRequest`/`oVRequest` 在时序 always 里用阻塞赋值**（`V/VGA_Controller.v:184-190`），且二者不互斥（`V_active_area` 在行消隐期仍为高）。
13. **SDRAM 空间浪费**：32 位字里只用 10 位（顶层 `WR1_DATA = MIPI_PIXEL_D[9:0]`，:267）→ 1.2 MB 存了本该 384 KB 能存下的图像，**浪费 3.2 倍**。
14. **两个片上行缓冲都远大于需要**：`int_line` 4096 字只用 640（`V_D8M/int_line.v:100`）；`red_line_x` 640 字正好。前者浪费 6 倍。
15. **没有真正的帧同步/双缓冲**：读写指针是独立的 chase 指针（4.4 节），靠 FIFO 水位耦合，**存在撕裂（tearing）风险**，且无法保证"显示的是完整的一帧"。想改成闭环控制必须换成 ping-pong + 帧就绪握手。
16. **掩膜天然滞后一帧**（`ball_ram` 读写同址 + `OLD_DATA`，4.4 节【推断】）。
17. **工程文件集是"大杂烩"**：磁盘上有 **3 组同名模块定义**（`I2C_DELAY_B`、`LCD_COUNTER_B`、`MIPI_CAMERA_CONFIG` 各被两个文件定义，复制粘贴未改名，见 3.3 节）；`qsf` 引用了 2 个找不到的文件（`MIPI_B/MIPI_B_I2C.v`、大写 `.SDC`）；`.bak`/`- Copy.v`/`*_bb.v` 与真实源码混放；`V_MIPI_B`/`V_MIPI_C`/`V_VCM` 是同一套 I2C 代码的三个变体（`_B`/`_C`/`VR` 后缀），其中 `V_VCM` 完全没被使用。**直接 clone 后重建预期会因缺失引用而失败**（3.3 节，未实测）。
18. **IP 目标器件族不一致**：两个 dcfifo 与 `int_line` 的 `intended_device_family` 写的是 `Cyclone V`（`V_Sdram_Control/Sdram_WR_FIFO.v:87`、`V_D8M/int_line.v:98`），本工程是 `Cyclone IV E`。另有一份 **`V_Sdram_Control/sdram_pll.xml` 残留**（device_family=Cyclone V、输出 133.0 MHz），与顶层实际例化的 `V/sdram_pll.v`（100 MHz）完全不是一套，且**未被任何文件引用**——属于从 Cyclone V（DE10 系列 D8M 套件）工程移植时留下的垃圾。
19. **SDRAM 命令层三处细节缺陷**：① `SA<=12'h200` 把 precharge-all 的 A10 位错放到 `SA[9]`；② `PM_DONE` 的比较值 264 > ST 最大值 263 → 该信号恒 0；③ `DQM` 用了 2 位字面量赋给 4 位寄存器，导致屏蔽时高 16 位不受保护（详见 5.2 节）。
20. **死代码多**：`RE_TRIGGER` 的输出无人使用（4.2 节，且它**没有复位**，上电首拍输出不定）；`RAW_RGB_BIN.v` 的 `DVAL`/`rDVAL` 未连接、`Line_Buffer_J` 的 `mCCD_FVAL`/`VGA_VS`/`V_Cont` 在模块体内未使用、`RAW2RGB_J` 的 `V_Cont`/`oDVAL` 悬空、`RAM_READ_COUNTER.v`/`SEG7_LUT*.v`/`V_VCM/*` 未被例化、`I2C_DELAY_B.v` 已编译但未例化、`pixel_count` 计数器输出悬空（`ball_detector.v:103`）。
21. **未使用的端口/引脚**：`LEDG[8:0]` 从未被赋值；`UART_RTS`/`UART_TXD` 被赋值但不在端口列表里（隐式网络，综合时被优化掉，`DE2_115_D8M_RTL.v:128-129`）；`MIPI_MCLK` 未驱动；`READY` 是隐式网络（`DE2_115_D8M_RTL.v:223` 未声明）；`EX_IO[6:0]` 有引脚分配但**没有 `IO_STANDARD` 分配**（`qsf:497-503` vs `qsf:19-194,255-456` 的差集恰为这 7 条）。
22. **两个顶层 `.bak` 备份**（`de2_115_d8m_rtl.v.bak` 是未改动的 Terasic 原始顶层）说明这是"在厂商例程上打补丁"的开发方式——**这也是这份工程最真实、最有借鉴价值的地方**：它演示了如何把自研模块插进一个现成的 Terasic 视频流水线。
23. **摄像头的 I2C 表长与计数器不一致**：`WORD_NUM_MAX=314` 但表只有 294 项（其中 3 项是延时标记，实际写 291 次寄存器）→ WCNT 294..313 无 case 分支，会**把最后一项 `0x0100=0x01`（唤醒/流式）重复写 20 次**（4.2 节）。
24. **厂商代码的注释已经与接线脱节**：`V_Auto/FOCUS_ADJ.v` 的端口注释写 `AUTO_FOC = KEY[4]`、`SW_Y = SW[1]`、`SW_H_FREQ = SW[2]`、`SW_FUC_ALL_CEN = SW[4]`，而顶层实际接的是 `KEY[3] & AUTO_FOC`、`SW[3]`，且 `SW_Y`/`SW_H_FREQ`/`SW_FUC_ALL_CEN` **恒接 0**（`DE2_115_D8M_RTL.v:207-211`）。另外 `AUTO_FOCUS_ON.v:14` 让自动对焦在复位释放后**再等 1 亿个 CLOCK2_50 周期（≈2 s）**才使能（`assign AUTO_FOC = (PULSE < 100000000)?0:1`）。看注释理解这份工程会得出错误结论，**必须回到顶层接线为准**。

---

## 9. 对"二维动态障碍环境模拟器"项目的可借鉴点

> 你的目标：Python + NumPy 仿真内核 → 部署到 **CPU/FPGA 异构平台** → 用**现实二维小车 + 摄像头**验证；需要**从摄像头里提取障碍物位置**；已有 **`BHP1` 固定 stride 帧**协议。

### 9.1 可以直接复用的部分（结论：**"阈值分割 + 质心提取 + 坐标输出"这条链可以复用，但要降级为"参考实现"而不是"生产实现"**）

- **架构骨架值得照抄**：`像素流 → 1 bit 命中掩膜 → 行缓冲窗口 → 逐行游程统计 → 坐标寄存器` 这条流水线**完全可以平移到你的障碍物检测**，而且它有一个对你特别有价值的性质：**全程无乘法器、无除法器、无 BRAM 大块，只有比较器 + 加法器 + 移位**（6.1 节 `Embedded Multiplier = 0`）。这意味着你的 NumPy 仿真内核里那些 `mask = ...`、`convolve`、`argmax` 都能找到**极廉价的 FPGA 对应物**：
  - `red_line_buffer.v:34` 的阈值 ⇒ NumPy `mask = (R//2 > G) & (R//2 > B)`（注意是整除，且**丢掉了低 4 位**，你仿真时也应模拟 `R>>4` 的量化以对齐）。
  - `red_frame.v:33-38` 的 3×3 多数 ⇒ NumPy `sum3x3 = uniform/convolve(mask.astype(int), ones(3,3))`，`mask2 = sum3x3 >= 5`。这是一次**形态学开/闭的近似**，等价于"去毛刺 + 填小洞"，很便宜。
  - `red_frame.v:63-67` 的坐标 ⇒ NumPy 沿着行做 run-length：
    ```python
    # 等价于 red_frame.v 的逐行最长游程 + 帧内取最大
    rows_of_max, max_ever, end_x = None, 0, 0
    for y in range(480):
        x = np.flatnonzero(mask2[y]); 
        if x.size == 0: continue
        # 找最长连续段
        brk = np.flatnonzero(np.diff(x) > 1)
        segs = np.split(x, brk + 1)
        seg = max(segs, key=len)
        if len(seg) > max_ever: max_ever, end_x, rows_of_max = len(seg), seg[-1], y
    cx, cy = end_x - (max_ever >> 1), rows_of_max   # 与 RTL 逐位对齐
    ```
    **这段 NumPy 就是 `red_frame.v` 的位精确等价模型**，可以直接做你的"软硬件一致性"测试基准（golden vector）。
- **`Mod_counter.v`**（24 行，模 N 计数器）可以直接搬走做行列计数。
- **`ball_ram`/`red_line_x` 的 altsyncram 实例化模板**可以直接复用（只要把地址/位宽改一下），双口 + `OLD_DATA` + 无输出寄存器的配置对"读写同址回放"场景是现成的。
- **`VGA_Controller` + `VGA_Param.h`** 是干净的 640×480@60 时序参考，`oRequest/oAddress` 的"请求-节流"模式（用显示侧的反压控制存储侧读出）对任何"存储回放"设计都通用。

### 9.2 "双缓冲 + 仲裁"怎么迁移到 CPU/FPGA

**不要照搬本工程的 chase 指针**（4.4 节，无帧同步、有撕裂风险）。建议这样迁移：

1. **把 SDRAM 分成固定 stride 的 N 个整帧槽（ping-pong 或三缓冲）**，而不是一个 307200 字的环形区。你的 `BHP1` 帧本来就是固定 stride，天然适合：
   - `SLOT_SIZE = align_up(W*H*bpp, 256)`（对齐到 256 字突发，直接沿用本工程的 `WR1_LENGTH=256`/`rWR1_ADDR += 256` 的块推进逻辑，`Sdram_Control.v:444-450`）。
   - 写指针只在自己拥有的槽内走；读指针只在"已就绪"的槽内走。**用槽号代替地址做同步**。
2. **保留本工程仲裁器的两个正确设计**（可以直接抄 `Sdram_Control.v:477-555` 的结构）：
   - **突发级授权 + 非抢占**：一次授权搬 256 字（2.63 µs @100 MHz），把 100 MHz 控制器与 25 MHz 像素侧、以及 CPU 侧的低速访问解耦，避免每字仲裁的开销。实测效率 97.3%。
   - **写优先、读用"水位"条件触发**：`write_fifo_level >= LENGTH` 才写，`read_fifo_level < LENGTH` 才读。这个"双侧水位窗口"策略比简单轮询更适应速率不等的两端。
3. **CPU 侧接入**：本工程只有 WR1/RD1 一对 FIFO 就够用，因为 CPU 与 FPGA 通过 `BHP1` 帧交互时，**CPU 通常只读不写**（读 FPGA 的检测结果）。给 CPU 开**第三个端口对（RD2）**——`Sdram_Control` 本身就有 WR2/RD2 的完整骨架，只是被注释掉了（`Sdram_Control.v:284-292,304-312`），**取消注释即可恢复双端口**，这正是 Terasic 原设计的意图。让 CPU 读 RD2，就与视频读 RD1 完全隔离，各自有独立 FIFO 和优先级。
4. **仲裁优先级建议改为**：CPU 结果读 > 摄像头写 > 显示读。因为结果是控制回路的输入，显示晚一帧无所谓。改法就是在 `:502-530` 的 if-else 链里插入一条 CPU 分支（注意加上 `RD2_LOAD` 到 `:491-494` 的 idle 判据里）。

### 9.3 跨时钟域同步与 `BHP1` 帧协议的对接

本工程最值钱的经验就在这里：**它用异步 FIFO（dcfifo）而不是双触发器同步器来做跨域**，这是对的。具体建议：

1. **`BHP1` 帧的跨域不要传数据，只传"帧索引"**。做法：
   - FPGA 侧写完第 k 帧后，把 `k`（或 `k mod N`）写进一个寄存器，同时翻转一个 `frame_toggle` 位。
   - 用**双触发器同步器**把这个 toggle 同步到 CPU 侧时钟域（或反过来），CPU 检测到 toggle 变化后，**按固定 stride 直接去读那块内存**（因为 stride 固定，不需要传长度/地址）。
   - 这是"数据走存储、同步走握手"的经典拆分，比把整帧数据推过 FIFO 便宜得多。本工程的 dcfifo 只在**像素流**这种必须逐字搬运的场合使用，你的 `BHP1` 帧不需要。
2. **`usedw` 不能直接跨域读**。本工程把 dcfifo 的 `rdusedw`（`Sdram_Control.v:282`）在**仓库侧时钟域**使用，是合法的；而它的反压判据用的是 `write_side_fifo_rusedw1`（写侧时钟域）和 `read_side_fifo_wusedw1`（写侧）——注意 `read_side_fifo_wusedw1` 是 **DCFIFO 在写侧同步出来的水位**（`Sdram_Control.v:301`），这正是"用 dcfifo 自带的同步后水位"的正确用法。如果你自己写 FIFO，**必须自己把格雷码指针打两拍再比较**，不要直接比较二进制指针。
3. **四个时钟域就够，不要更多**。本工程 50 MHz（控制/I2C）+ 100 MHz（存储）+ 摄像头 pclk + 显示 pclk 是合理划分。你的 CPU 侧如果是 Zynq/SoC，建议把 CPU 域与存储域合并（都走 AXI），只保留"摄像头输入域"和"存储/CPU 域"两个，能省掉一个 CDC。
4. **`.sdc` 一定要写**。本工程最大的工程教训就是：**没写 `set_clock_groups -asynchronous` 导致 TimeQuest 报了 −5.3 ns 的假（对功能而言）违约，而真正该约束的 `MIPI_PIXEL_CLK` 反而没约束**（6.2 节）。你的项目里：① 每个输入时钟都要 `create_clock`；② 所有异步域之间 `set_clock_groups -asynchronous`；③ 对 DCFIFO 两端加 `set_false_path`（或直接用厂商 IP 的自动约束）；④ 复位释放路径加 `set_false_path`（本工程 `DLY_RST_*` 是无同步器跨域的，第 5.4 节）。

### 9.4 诚实劝退：哪些做法**不要**照搬

| 不要照搬 | 为什么 | 替代方案 |
|---|---|---|
| `floor(R/2) > G && floor(R/2) > B` 这个硬阈值 | 对光照/白平衡/曝光敏感；只对"红色且明亮"有效；无自适应（8.1 节） | 用**归一化色度**：`r = R/(R+G+B+1)`，判 `r > tau` 且 `R+G+B > V_min`；或直接在 CPU 侧用 NumPy 做 **Otsu / 自适应阈值 + HSV 的 H 分量双阈值**。若必须在 FPGA 上做，把 `tau` 做成可运行时写的寄存器（AXI-Lite/SPI 配置），由 CPU 按场景下发——这正好用上你的异构平台 |
| "最长单行游程的中点"当质心 | 对 C 形/分离/遮挡目标退化（8.3 节） | ① 先做**真正的连通域标记（CCL）**再求面积质心；FPGA 上用 two-pass CCL 或用 union-find 的流水线版本，成本中等。② 折中：**行列投影** `col_sum = mask.sum(axis=0)`, `row_sum = mask.sum(axis=1)`，用加权质心 `cx = Σ x·col_sum / Σ col_sum`，只需累加器 + 一个除法（或用 `1/sum` 的 LUT 近似）——**比连通域便宜得多，且是真正的质心**。③ 多目标就取 `col_sum` 的多个峰 |
| 32 位字只存 10 位像素 | 3.2× 存储浪费（8.2 节） | 按 10 bit 或 16 bit 紧凑打包；若沿用 32 位字，就把**4 个像素打包进一个字**，或存 RGB888 而不是 RAW |
| `int_line` 4096 字只用 640 | 6× 浪费（8.2 节） | 把行缓冲深度设成 `H_ACTIVE` 向上取整（640→1024 或 512+128） |
| 用 `h_sync` 当时钟（`ball_detector.v:91`） | 门控派生时钟，偏斜不可控（8.2 节） | 用**同一时钟 + clock enable**（就像本工程 `Mod_counter` 的 `clk_en` 那样，`:6,:18`），不要用信号当时钟 |
| chase 指针式帧缓冲 | 无帧同步、可能撕裂（8.2 节） | 固定 stride 的 N 槽 ping-pong（9.2 节） |
| `oCoord_X - 3` 这类无符号下溢 | 每行最左 3 像素错址（8.2 节） | 显式打拍对齐（把地址流水线延迟做成真正的寄存器级），或对下溢做饱和处理 |
| `- Copy.v` / `.bak` / 三套 `V_*` 变体混放 | 9 个重复模块定义、2 个缺失文件引用（3.3 节） | 用 git 分支而不是文件副本管理变体；一个变体一套独立工程目录 |

### 9.5 给你的落地顺序建议（把这份工程当"教材"用）

1. **拿 9.1 节那段 NumPy 等价模型**去对齐你的仿真内核，确认"最长游程中点"和"面积质心"在你的二维动态障碍场景里的差距有多大。**很可能差得很多**（动态场景常有部分遮挡、粘连），这决定了你要不要上 CCL。
2. **只移植三样东西到你的 FPGA 侧**：① 像素流 → 1 bit 掩膜 + 3×3 多数表决（`red_line_buffer`+`red_frame` 的窗口部分）；② `Mod_counter` 行列计数；③ `Sdram_Control` 的**突发级仲裁器**（换成 N 槽 ping-pong 地址推进）。
3. **把"坐标输出"做成 `BHP1` 的一个字段**，用 9.3 节的 toggle 握手跨到 CPU 域。坐标只需要 `(cx[9:0], cy[9:0], valid, frame_id)` 几十个 bit，比传整帧便宜三个数量级。
4. **阈值参数一定做成可配置**，由 CPU 侧根据 NumPy 仿真/离线标定的结果下发。硬编码阈值是这个工程最不适合复制的部分。
5. **第一版就把 `.sdc` 写全**（9.3 节第 4 条）。这份工程 4% 的 LUT 占用率说明资源从来不是瓶颈，**时序约束和 CDC 才是**。

---

## 附录 A：文件 → 模块 → 角色（按编译清单，`qsf:504-558`）

| 文件 | 行数 | 顶层模块 | 归属 | 在本设计中的实际作用 |
|---|---|---|---|---|
| `DE2_115_D8M_RTL.v` | 354 | `DE2_115_D8M_RTL` | 作者 | 顶层连线与整合 |
| `Mod_counter.v` | 24 | `Mod_counter` | 教材 | 行列像素计数（3 实例） |
| `BALL_DETECTOR/ball_detector.v` | 113 | `ball_detector` | 作者 | 检测封装 + 十字 Mux |
| `BALL_DETECTOR/red_frame.v` | 135 | `red_frame` | 作者 | 低通 + 最长游程 + 坐标 FSM |
| `BALL_DETECTOR/red_line_buffer.v` | 117 | `red_line_buffer` | 作者 | 红色阈值 + 4 行缓冲旋转 |
| `BALL_DETECTOR/red_line_x.v` | 213 | `red_line_x` | wizard | 640×1 bit 双口 RAM（×4） |
| `ball_ram.v` | 220 | `ball_ram` | wizard | 307200×1 bit 掩膜帧缓冲 |
| `V/VGA_Controller.v` | 243 | `VGA_Controller` | Terasic | VGA 时序 + 地址生成 + 节流请求 |
| `V/VGA_Param.h` | 20 | — | Terasic | 640×480@60 时序参数 |
| `V/RESET_DELAY.v` | 80 | `RESET_DELAY` | Terasic | 4 级错开复位 |
| `V/CLOCKMEM.v` | 22 | `CLOCKMEM` | Terasic | 分频产生"1 Hz"心跳 |
| `V/FpsMonitor.v` | 82 | `FpsMonitor` | Terasic | 场频计数 + 七段码 |
| `V/pll_test.v` | 341 | `pll_test` | Altera | 50 MHz → 20 MHz + 25.180 MHz |
| `V/sdram_pll.v` | 349 | `sdram_pll` | Altera | 50 MHz → 100 MHz 0° / −90° |
| `V/SEG7_LUT.v` / `SEG7_LUT_8.v` | 70/55 | `SEG7_LUT*` | Terasic | **未编译、未使用** |
| `V_D8M/MIPI_BRIDGE_CAMERA_Config.v` | 34 | `MIPI_BRIDGE_CAMERA_Config` | Terasic | 两个配置器的 wrapper |
| `V_D8M/RAW2RGB_J.v` | 103 | `RAW2RGB_J` | Terasic | 反拜耳（本工程实为单时钟域） |
| `V_D8M/Line_Buffer_J.v` | 89 | `Line_Buffer_J` | Terasic | 3 个行缓冲（`int_line`）但**只暴露 2 个抽头** `taps0x`/`taps1x`，且被 `READ_Request` 门控为 0；`mCCD_FVAL`/`VGA_VS`/`V_Cont` 端口在模块体内未被使用 |
| `V_D8M/int_line.v` | 216 | `int_line` | wizard | 4096×12 bit 双口 RAM（×3） |
| `V_D8M/RE_TRIGGER.v` | 16 | `RE_TRIGGER` | Terasic | **已例化但输出无消费者** |
| `V_D8M/RAW_RGB_BIN.v` | 65 | `RAW_RGB_BIN` | Terasic | **已编译但未被例化** |
| `V_D8M/RAM_READ_COUNTER.v` | 16 | `RAM_READ_COUNTER` | Terasic | **未编译、未使用** |
| `V_MIPI_B/MIPI_BRIDGE_CONFIG.v` | 328 | `MIPI_BRIDGE_CONFIG` | Terasic | 桥芯片 13 条寄存器表 + ID 读 |
| `V_MIPI_B/I2C_*.v` `LCD_COUNTER_B.v` | — | — | Terasic | 桥表所需的 I2C 底层 |
| `V_MIPI_C/MIPI_CAMERA_CONFIG.v` | 592 | `MIPI_CAMERA_CONFIG` | Terasic | 传感器 294 条寄存器表 + ID 读 |
| `V_MIPI_C/I2C_*.v` `LCD_COUNTER_C.v` | — | — | Terasic | 传感器表所需的 I2C 底层 |
| `V_Auto/AUTO_FOCUS_ON.v` | — | `AUTO_FOCUS_ON` | Terasic | 顶层使用（:195），同时是时序违约的源 |
| `V_Auto/FOCUS_ADJ.v` | — | `FOCUS_ADJ` | Terasic | 顶层使用（:203） |
| `V_Auto/` 其余 11 个文件 | — | — | Terasic | **死代码（已编译未例化）** |
| `V_VCM/` 4 个文件 | — | — | Terasic | **死代码（已编译未例化）** |
| `V_Sdram_Control/Sdram_Control.v` | 557 | `Sdram_Control` | Terasic | 仲裁 + 地址循环 |
| `V_Sdram_Control/control_interface.v` | 240 | `control_interface` | Terasic | 命令译码 + 刷新/初始化定时器 |
| `V_Sdram_Control/command.v` | 482 | `command` | Terasic | 命令时序流水 + 地址切片 |
| `V_Sdram_Control/sdr_data_path.v` | 76 | `sdr_data_path` | Terasic | DQM/DQ 输出（`DQOUT = DATAIN`，`DQM <= DM=2'b00`） |
| `V_Sdram_Control/Sdram_Params.h` | 59 | — | Terasic | SDRAM 地址/时序参数 |
| `V_Sdram_Control/Sdram_WR_FIFO.v` / `Sdram_RD_FIFO.v` | 175 each | `Sdram_*_FIFO` | wizard | dcfifo 1024×32（跨时钟域核心） |

## 附录 B：明确的"未确认"清单（避免以讹传讹）

1. 板上 SDRAM 的**具体型号与容量**：代码只给出 32 位总线、4 bank、12 位行、8 位列（`Sdram_Params.h:3-13`），没有容量声明。
2. 传感器**确切型号**：`0x300B/0x300C = 0x88/0x65` 与 OV8865 一致，但代码无型号文字。
3. Bayer **相位**（RGGB/GRBG/GBRG/BGGR）：`RAW_RGB_BIN` 只给出 `{Y,X}` 四相位到 R/G/B 的映射（`V_D8M/RAW_RGB_BIN.v:38-61`，已列于 4.2 节），但**没有指明物理起始相位**，无法判定属于哪一种排列。
4. `MIPI_REFCLK` 20 MHz 与桥芯片 REFCLK 目标频率一致（`MIPI_BRIDGE_CONFIG.v:202-205` 注释写 "REFCLK 20 MHz"），但 20 MHz 是给谁的（FPGA 输出给桥？还是桥的参考输入）**未从代码确认**——`MIPI_REFCLK` 在顶层是 **output**（`DE2_115_D8M_RTL.v:74`）。
5. 桥芯片**型号**（TC3587xx 系列）：未在代码中出现。
6. 桥寄存器 `0x0008 DATA_FORMAT=0x0010` 的**位域含义**：代码只有 "Data Format Control Register" 注释，未展开。
7. 上板**实测帧率/延迟**：本文 6.3 节的数字全部是推算，仓库里没有实测日志（只有 `camera_dark_noise.JPG`/`.xlsx` 与 `aaa_hardware.jpg`/`aaa_demo_video.mp4` 等结果素材）。
8. 当前 `.qsf` 文件集能否**编译通过**：只做了静态检查，发现 2 处缺失引用（`V_MIPI_B/MIPI_B_I2C.v`、大写 `.SDC`）——预期报错；3 组同名模块定义**当前未冲突**（每组 qsf 只收了一个）。**未运行 Quartus 验证**。
9. `int_line` 实为 `Cyclone V` 目标族而被 `Cyclone IV E` 工程使用，Quartus 是否会自动重映射：未编译验证。
10. `fit.rpt` 中 `VGA_CLK`/`MIPI_REFCLK` 的 "Missing drive strength" 提示（`fit.rpt` 内可检索到）是否影响功能：未确认。
11. **`MIPI_PIXEL_CLK` 的实际频率**：桥配置注释写 PCLK 25 MHz（`V_MIPI_B/MIPI_BRIDGE_CONFIG.v:204`），传感器寄存器 `0x3638` 注释写 PCLK 75 MHz（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:298`），顶层 `CLOCKMEM` 按 25 MHz 传参（`DE2_115_D8M_RTL.v:349`）——三者口径不一致，**真实并口 PCLK 未确认**。
12. **`RAW2RGB_J` 交叉接线 `.R(mCCD_B)`/`.B(mCCD_R)` 是否为有意修正**：代码无注释（`V_D8M/RAW2RGB_J.v:97,99`）→ 红蓝互换的**意图未确认**。
13. **`SA<=12'h200`（落在 `SA[9]`）是否符合 JEDEC precharge-all（应为 A10）的意图**：未确认，未仿真。
14. **`DQM` 2 位字面量导致高 16 位不屏蔽的实际影响**：未确认（本工程写入高位恒 0，可能无影响）。
15. **两个 PLL 的内部 M/N 计数器与 VCO 频率**：`V/pll_test.v`、`V/sdram_pll.v` 只给出整链等效倍/分频比（`clk0/1_multiply_by`、`clk0/1_divide_by`），**M/N/C 计数器数值未确认**。
16. **`PLLJ_PLLSPE_INFO.txt` 中数值的单位**：文件未标注（Quartus 惯例为 ps）→ 未确认。该文件是 PLL 抖动/静态相位误差信息，供 `derive_clock_uncertainty` 使用（jitter：`pll_test` 34、`sdram_pll` 30）。
17. **`V_MIPI_C/MIPI_CAMERA_CONFIG.v` 的 `P_ID2`/`TIME_LONG`、桥配置的 ST=29 与传感器配置的 ST=36**：声明后未被使用或无可达跳转 → 用途未确认。
18. **`V_MIPI_B`/`V_MIPI_C`/`V_Auto`/`V_VCM` 的作者归属**：4 个目录内**均无版权/作者/网址字符串**，本文的"厂商参考代码"判定基于与 Terasic D8M 参考设计结构一致这一**结构性推断**，非文件声明。唯一确证的作者改动是传感器表里的 `// DRH: added this line.`（`V_MIPI_C/MIPI_CAMERA_CONFIG.v:457-458`）。
