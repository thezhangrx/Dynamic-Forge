# `core/board/` —— PS 侧（板上）代码

> 目标板：**复旦微悟净 PSOC Lite**，PS 是 4×Cortex-A7，跑 **Ubuntu 20.04 aarch64**。
> 这一层解决"PS 怎么跟 PL 说话"：配置阈值 → 起一帧 → 读几何帧。

## 1. 语言：**C++17**（不是 C）—— 三条依据

| 依据 | 出处 |
|---|---|
| 厂商整套 PS 栈就是 C++17 | `fpai_demo_app/examples/1_single_input+ai/PLin+SingleNet+HDMI/CMakeLists.txt` 的 `set(CMAKE_CXX_STANDARD 17)`；示例源码是 `adder_demo.cpp` / `sdicamera+yolov5+hdmi.cpp` |
| 我们要用的运行时就只有 C++ 接口 | `icraft-xrt` 的 `Device::Open()` / `defaultRegRegion()` / `defaultMemRegion()` |
| 项目自己的规定 | `core/cpu/docs/CPP_PORT.md`：决策要移植到 C++ |

**三者方向一致，所以不纠结 C 还是 C++。**

交叉工具链：`aarch64-linux-gnu-g++`（PetaLinux 里那份即可，Linaro 7.3.1）。
**构建在主机上做，不要在板子上编** —— 厂商 README 明确写了
"注意不要在 30TAI 板载系统上进行编译"。

## 2. 文件

| 文件 | 作用 | 本机能编吗 |
|---|---|---|
| `vision_regs.hpp` | **PS↔PL 寄存器契约**（地址、位定义、帧窗口） | ✅ |
| `geometry_frame.hpp/.cpp` | **几何帧线协议 v3 解码**（长度/magic/版本/CRC 四道校验 + Q6 反定标） | ✅ |
| `vision_driver.hpp/.cpp` | 驱动：配置 / 起帧 / 轮询 / 读窗口 | ✅（旧写法 `parse_header` 仍可用，见 `geometry_frame.hpp`） |
| `vision_driver_icraft.cpp` | icraft 后端（`RegIo` 的板上实现） | ❌ 需要 icraft SDK |
| `main.cpp` | 板上 bring-up 程序：**取帧 → 解码 → 决策 → 打印速度指令** | ❌ 同上 |
| `CMakeLists.txt` | 构建 | — |
| `tests/` | 契约核对 + 驱动自测 + 解码对账 + 决策交叉验证 | ✅ |
| `tools/` | **板上验证用的小工具**（读帧对账 / AXI 写通路加压 / 诊断 / rootfs 修复），单文件、在板子上直接 gcc 编。⚠ 其中三个会写 AXI，**旧位流上会把 CPU 挂死**，先读 `tools/README.md` | ✅（走 gcc，不经 CMake） |

决策用的是 `core/cpu/cpp/gap_avoid.{hpp,cpp}`（`core/cpu/gap_avoid.py` 的 C++ 移植，
验收口径 `tier` 逐位一致 + 浮点量 rel_err < 1e-9）。CMake 里以 `gap_avoid` 目标引入。

## 3. 为什么把寄存器访问抽成 `RegIo`

驱动真正的逻辑是**顺序**："写 start → 等 busy 起 → 等 busy 落 → 按 FLEN
读窗口 → 拼字节"。这个顺序跟"寄存器怎么访问"无关。

抽出来之后：

* 板子上用 `icraft_reg_io()`
* **PC 上用 `MemoryRegIo`（一块假寄存器文件），顺序逻辑不用上板就能测**
* 想更真，可以让假后端去调 C 参考模型（`core/fpga/ref/vision_ref.c`）

没有这一层，"驱动对不对"就只能等板子到了才知道。

## 4. 寄存器契约不能漂移

`vision_regs.hpp` 是 PS 侧唯一"记住"PL 长什么样的地方，而 PS/PL 是两份
独立维护的代码。地址错一个字节，读回来就是**别的寄存器**——不报错，只给错数。

所以有一条测试**直接解析两边源文件**逐项核对：

```
core/fpga/rtl/vision_axi.v   ← localparam [11:0] ADDR_*
core/board/vision_regs.hpp   ← constexpr uint32_t kOff*
```

反向验证过：故意把 `kOffFLen` 从 `0x0D0` 改成 `0x0D4`，测试当场报
`212 == 208` 失败。

## 5. 跑测试（本机，不需要板子）

```bash
python -m pytest core/board/tests -q
```

两条：
* `test_regs_match_rtl.py` —— 契约与 RTL 逐项核对（纯 Python）
* `test_driver_cpp.py` —— 用 g++ 编 `tests/test_vision_driver.cpp` 并运行
  （38 项比对：配置顺序、取帧字节逐位、帧头、两条反面控制）
* `test_geometry_frame_python.py` —— **线协议对账**：Python 的
  `encode_vision_frame()` 产出字节，喂给 C++ 解码器，逐字段比回去（8 条）

### 为什么线协议要单独对账

线协议有**三个独立实现**，缺一个对账就是一处静默错坐标：

| 角色 | 位置 | 怎么验 |
|---|---|---|
| 真值（Python） | `standard/vision/vision_frame.py` | 它就是标准 |
| PL 生产者（Verilog） | `core/fpga/rtl/vision_top.v` | 620 帧逐位协同仿真 |
| **PS 消费者（C++）** | `core/board/geometry_frame.cpp` | `test_geometry_frame_python.py` |

四道校验（长度 / magic / 版本 / CRC32）都做了**单独**的反向验证：逐个删掉，
对应用例当场变红。⚠ 这里有个坑值得记下来：坏帧测试**必须重算 CRC**，
否则 CRC 那一关会先把所有坏帧拦下，版本关/长度关根本没被执行到 ——
"版本检查整段删掉"也能全绿（实测）。

## 6. 上板怎么用

```bash
# 主机上交叉编译（需要先解开 icraft 的 deb 得到 sysroot）
cmake -S core/board -B build/board \
      -DVISION_BOARD_WITH_ICRAFT=ON \
      -DICRAFT_SYSROOT=/path/to/icraft_sysroot
cmake --build build/board -j$(nproc)

# 拷到板子上跑
./vision_board_app --frames 8
# 决策需要场参数（视觉帧**不携带**，标准 §6）：
./vision_board_app --frames 200 --field-w 640 --field-h 480 --player-speed 200
```

第一次跑的顺序（照 `vision_axi.v` 文件头的设计）：

1. `self_test()` —— 写一个配置寄存器读回来。**这条失败就别往下走了**，
   说明基址不对 / 位流没烧 / AXI 没接上
2. `configure()` —— 写 21 个阈值（默认值 = C 参考模型 = Python 真值）
3. `run_frame()` —— 逐帧；每帧解码几何帧、过一遍 `gap_avoid::plan()`，
   打印 `tier` 与速度指令

上板前**必须知道**的三件事（`main.cpp` 的 `decide()` 里都标了）：

* **dt 取两帧 `stamp` 之差**，不是帧号之差（帧号之差在掉帧时是错的）；
* **速度一律填 0** —— v3 线协议**没有**速度字段。对静态墙精确；对移动墙
  `v_n` 会退化成"只算车自己的速度"。真正的修法是 PS 侧按 stamp 差分出速度再填；
* **场宽/场高/速度上限来自命令行**（视觉帧不带），首帧 `dt` 用 `--dt-default`。

⚠ **还有一件必须知道的事**：`VisionParams` 的默认值必须与 C 参考模型
`vr_ref_default_params()` 逐项相同（也就等于 `core/vision/wall_with_gap.py` 的
语义真值）。**这里曾经有 13 项不一样**，表现是"板上几何帧障碍个数对、坐标不对"，
而且完全不报错 —— 因为协同仿真写进 RTL 的是 C 参考模型那套参数，
板上之前的跑法又都绕开了 `VisionParams` 的默认值（TPG 或 PS 参考模型），
于是三处都没验过它。现在由 `tests/test_params_defaults_match_reference.py`
逐项钉住；板上可以用 `--verify-both` 直接对账（配置 21 项 + 几何帧逐字节）。

### 6.1 ⚠ 板上编，不用交叉编译（也不需要 icraft）

**在板子上直接编** —— 板子自带 cmake 3.16.3 / g++ 9.4：

```bash
scp -r core/board core/cpu/cpp core/fpga/ref/vision_ref.c core/fpga/ref/ref_api.h root@192.168.137.242:/root/board_src/
ssh root@192.168.137.242 'cd /root/board_src && cmake -S core/board -B build && cmake --build build -j4'
```

为什么不去链 icraft：
* 我们这块 PL 的寄存器窗口是 **`0x43C00000`**，用 **`/dev/mem`** 就能读写
  （`tools/` 里的 read_frame / axi_soak 已在真硅上验过读写两侧）；
* icraft 的 `defaultRegRegion()` 是**另一个窗口**（`0x400C0000`，NPU 那边）；
* 链 icraft 还要交叉编译 + 解 deb 做 sysroot。
`vision_driver_devmem.cpp` + `vision_driver_factory.cpp` 就是这条后端；
`main.cpp` 只调 `open_reg_io(url)`，不认识任何具体后端，默认 `devmem://0x43C00000`。
（icraft 目标仍在，`-DVISION_BOARD_WITH_ICRAFT=ON` 才编。）

### 6.1.1 看启动日志（板子卡住时唯一的手段）

控制台是板载 **CP210x，插在 Windows 上（COM3）**。WSL 里要读它得先 `usbipd
attach`（要管理员），而且 attach 之后 Windows 那边就读不到了 —— 偏偏要看启动
日志的场合，通常正是 WSL 连不上的时候。所以在 Windows 侧读：

```bash
bash core/board/tools/capture_boot.sh 180      # 抓 180 秒，自动存到 data/fpga/boot_logs/
```

⚠ **UART 没有回放缓冲**：要抓启动日志，必须**先开始抓、再上电**（或按复位），
顺序反了就什么也拿不到 —— 这一点排查时最容易白折腾一轮。
⚠ 一个串口同一时刻只能被一个程序打开：先关掉 MobaXterm / 串口助手。
⚠ `win_console.ps1` 里 DTR/RTS 默认**不动**（有些板子把它们接到复位/BOOT 上），
需要时再加 `-Dtr`。两个 `.ps1` 都必须是 **UTF-8 with BOM**：Windows PowerShell 5.1
对没有 BOM 的脚本按系统 ANSI 代码页解码，中文注释会被解坏、把语法拆掉。

#### 板子"卡住"时怎么查（结论：这次是假警报）

> **2026-10-16 已恢复，板子本身没坏。** 启动日志（`data/fpga/boot_logs/boot_20261016_070211.log`）
> 显示：FSBL 4.51 → `BootMode Register 0x00000145`（**SD Boot Mode**）→ `Running on A53-0`
> → 内核 → systemd → **8.8 秒**进多用户 → `U login:`；`PL: operating`、`pl-autoload: active`。
> 之前"串口全静默 + 网络不通"只是因为**抓取没在上电那一刻开始**（UART 没有回放缓冲），
> 空日志被误读成"板子什么都没打印"。**板子掉电后要先抓、再上电**，这条比什么都重要。

**现象（掉电后未及时抓取时看到的）**：

| 查什么 | 结果 | 说明 |
|---|---|---|
| 板子网口链路 | **Up / 1 Gbps** | 板子有电、PHY 在跑 |
| CP210x(COM3) / JTAG(1-8) | 都在枚举 | USB 侧正常 |
| ARP `192.168.137.242` | **FAILED** | 系统没应答 |
| 控制台 COM3 | 10 秒无输出 + 回车无响应 | 卡在控制台起来**之前** |

**查法就两步**（要人在机器前，远程做不了）：

1. `bash core/board/tools/capture_boot.sh 600` —— **先**开始抓；
2. 给板子**断电 3~5 秒再上电**（或按复位）。

按日志分三种情况处理：

* 有 FSBL/U-Boot/内核字样 → CPU 起来了，卡在启动某一步（掉电常把 rootfs 弄坏 → 卡 fsck/挂载）；
* 一路走到 `login:` → 系统其实起来了，只是网络没通，从串口登进去查网络；
* **仍然一个字都没有** → FSBL 没跑，走恢复：`docs/resource/icraft_v3_ubuntu20.04_aarch64_sd_image.7z`
  （厂商 SD 镜像）。⚠ 会清掉板上那套配置（`pl-autoload.service`、启动卡顿修复、板上二进制、
  `/root/tools`），但这些在仓库里都能复现，重做约 0.5~1 小时；动手前先确认启动方式
  （QSPI 还是 SD、启动模式引脚要不要动）。JTAG 这条**不能**当"板子死没死"的判据 ——
  这块克隆芯片的 DAP 本来就会报 `AXI AP transaction error`。

**四个已经踩过的坑**（工具里都修了，别再踩）：

1. **先抓后上电**：UART 没有回放缓冲，顺序反了一个字都拿不到；
2. **管道不要过 `tr`**：`tr -d '\r'` 输出到文件时是**全缓冲**的，板子的输出会卡在它缓冲区里，
   日志一直 0 字节 —— 看着像"板子什么都没打印"。CR 现在在 `win_console.ps1` 里剥；
3. **kill 抓取任务 ≠ 停掉读串口**：Windows 侧那个 PowerShell 不一定跟着死，会一直占着 COMx
   （下一次抓取打不开端口，而报错又可能被缓冲住）。`capture_boot.sh` 现在有 `trap ... EXIT` 清它；
4. **查进程时别把自己算进去**：过滤命令行含 `win_console` 的进程时，查询命令自身就匹配，
   要排除 `$PID`。

#### ⚠ 同步代码后必须防"时钟偏移"骗过 make

板子的时钟可能比文件 mtime 慢好几个月（这块板子实测慢 7 个月）。`tar`/`scp` 过来的
源码带着"现在"的 mtime，而目标文件与旧可执行文件的 mtime 在**未来** —— make 于是认为
"都是最新的"，**连链接都不做**，只打一句 `Clock skew detected. Your build may be incomplete`。
症状极隐蔽：源码 md5 明明对上了，**二进制 md5 却和上一版一模一样**（我们真踩过一次，
差点把没生效的改动当成"上板验证通过"）。

```bash
# 安全做法：先把板子时钟对齐 PC，再强制重建
ssh root@板子 "date -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
ssh root@板子 'cd /root/board_src/build && make -B -j4 vision_board_devmem'
# 判定标准不是"make 成功"，而是 ① 二进制 md5 **变了** ② 新功能能自证（如 --help 里出现新开关）
```

### 6.2.1 一条命令常驻闭环（推荐入口）

```bash
bash core/board/tools/run_loop.sh              # 一直跑，Ctrl-C 停
bash core/board/tools/run_loop.sh --keys wasd  # 平台用 WASD
bash core/board/tools/run_loop.sh --dry-run    # 只打印要按的键，不注入
```

链路：板子【摄像头 → PL 视觉 → 解码 → 推速度 → 决策】→ `@MOVE n` 到 stdout
→ SSH → PC【按键注入 → 平台】。它替你处理了四件容易出错的事：

| 事 | 为什么必须由脚本管 |
|---|---|
| **开跑前自检**（板子 / PL 是否 `operating` / 相机节点 / 注入器），并**报出当前焦点窗口** | 否则你会对着黑屏猜"是不是我没配好"；而合成按键打到**焦点窗口**，不报出来就是盲按 |
| **5 秒倒计时**再开始按键 | 给你时间把平台窗口点成焦点（`--delay-s 0` 跳过） |
| **断了自动重连**（2 秒，连续失败退避到 10 秒） | SSH 掉线/相机抽风时不会静默变成"什么都没发生" |
| **收尾干净**：先松键（ALLUP）→ 停掉板子上的相机链路 → 断开 | 不松键会让角色一直朝前跑；不停 ffmpeg 会让摄像头卡在 `Device or resource busy` |

全程 tee 到 `data/fpga/loop_logs/loop_<时间>.log`，事后能查。停板子上的相机用
`pkill -INT -x ffmpeg`（按**进程名**精确匹配，不用 `-f`，也**绝不 -9**）。

### 6.2 三条已经跑通的路

| 路 | 命令 | 帧来源 | 用途 |
|---|---|---|---|
| **PL（片上图案）** | `./vision_board_devmem --frames 8` | `tpg_source` → PL 流水 → AXI 窗口 | 自检；验 PL 通路 |
| **PL（真图）** | `--ppm FILE --via-pl` | PPM 像素**推进 PL 的 FIFO** → PL 流水 → 窗口 | **真摄像头走 FPGA 加速** |
| **PS 参照** | `--ppm FILE` / `--ppm-dir DIR` | PS 上跑 C 参考模型（黄金模型） | 真图；也是 PL 出问题时的参照 |
| **键盘** | 默认就发（`--keyboard-dev /dev/ttyPS0`） | 决策 → 动作号 → `@MOVE <n>\n` | 给 RP2040 键盘桥 |

**PL（真图）那条路怎么走的**：PS 把一帧 RGB 按 32 位字推 `0x0D4`（像素 FIFO），
PL 侧 `vision_pixfeed.v` 把字节流还原成像素时序喂给流水线，几何帧照旧从窗口读。

* **为什么不用"DDR + PL AXI master"**：先量过 PS→PL 的吞吐
  （`tools/axi_bw.c`）：691200 B 要 17.4 ms（30fps 预算的 52%）。非缓存写
  ~40 MB/s 是 PS 侧的硬约束 —— 写 PL 还是写 `/dev/mem` 映射的 DDR 都逃不掉。
  所以不引入 AXI master，少一个要写要验、还要 DDR 预留的部件。
* **为什么不存整帧**：消费者每拍 1 像素 = 150 MB/s，比 PS 的 40 MB/s 快 3.75 倍，
  所以 FIFO 只会被抽空、不会被塞满 → **PS 可以盲写**（不必每字回读状态）。
  省掉 150 个 BRAM，也不用把 ADDR_W 从 12 位撑到 20 位。
* **溢出不会静默**：`PIXSTAT` 的 bit16 是粘滞溢出位，PS 推完读一次就知道。

**三条路给出的几何帧必须逐字节相同** —— 这一点由两处证明：

* 协同仿真：`tb_axi --fifo` 把同一批真照片从两条像素通路各跑一遍，逐字节比；
* **真硅上**：`--verify-both` 让**同一张图**分别走 PL（推 FIFO）和 PS（黄金模型），
  再逐字节比。这是"PL 吃真图"最直接的判据：

```bash
# 采一帧真摄像头，然后两条路都跑、逐字节对账
ffmpeg -f v4l2 -input_format mjpeg -video_size 1280x720 -i /dev/video1 \
       -frames:v 1 -vf scale=640:360 -pix_fmt rgb24 /root/cam.ppm
./vision_board_devmem --ppm /root/cam.ppm --verify-both --frames 8 --echo-action
```

不一致时它会打出**第一个差异字节的位置和两边的内容**，便于直接定位到帧头哪个字段。

三条可以任意组合：`--ppm FILE` 换掉帧来源，键盘那一段不变。
决策/解码/键盘是**共用**的下游，所以"真图"和"PL 图"走的是同一条后处理。

**真摄像头（实时）**：一条命令（脚本在 `core/board/tools/cam_live.sh`）：

```bash
ssh root@192.168.137.242 'cd /root/tools && bash cam_live.sh --frames 0 --echo-action --via-pl'
```

它做三件事：**自动识别**摄像头节点、关掉"为长曝光降帧率"、把 `ffmpeg` 的原始
RGB 直接管道喂给程序（不再每帧重开设备 + 落盘）。

实测（JMJP 640×480 → 裁剪 640×360 → PL → 决策 → 键盘）：

| 段 | 耗时 | 说明 |
|---|---|---|
| 等采集 | 20.5 ms | 摄像头 MJPG 640×480 原始交付 48.4 帧/秒（=20.8ms），已到它上限 |
| 推 PL | 20.2 ms | 与 `axi_bw` 的 40 MB/s 吻合，MMIO 上限 |
| 等流水线 | 5.4 ms | 推完像素后的后处理 |
| **合计** | **46.1 ms → 21.7 帧/秒** | 批处理（每帧重开设备）只有 ~1 帧/秒 |

**四个实测坑**（都写进了脚本注释）：
1. **节点号会随重启变**：重启后 capture 从 `video1` 变成 `video0`，而 `video1` 成了
   它的 **metadata 节点**（打开就报 `VIDIOC_G_INPUT: Inappropriate ioctl`）；
   另有 Linlon 的 M2M 编解码器**也列 YUYV**，是个诱饵。脚本按"支持 MJPG"自动识别。
2. **`exposure_auto_priority=1` 会降帧率**（允许为长曝光拖长曝光时间）：
   14.6 → 25.3 帧/秒（YUYV 实测）。
3. **自动曝光要好几帧才收敛**（第 5~9 帧才稳，早期抓到过全黑的第 1 帧）→ `--skip-frames`。
4. **别硬杀正在 stream 的进程**（`kill -9`）：设备会卡在 `Device or resource busy`，
   之后所有采集都失败。复位用 `echo -n 1-1.3 > unbind/bind`（等价拔插）。

**旧的一次一帧做法**（批处理，慢但简单）：

```bash
# 1) 采一帧（USB UVC 摄像头插上后 /dev/video0 就是它）
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 -pix_fmt rgb24 /root/cam.ppm
# 2) 跑完整条：C 参考模型 → 几何帧 → 解码 → 决策 → 发动作
./vision_board_devmem --ppm /root/cam.ppm --echo-action
```

> ⚠ `/dev/video0` 在**没插摄像头**时是 **Linlon 的 M2M 视频编解码器**
> （`platform:mvx`，H.264/HEVC/VP8/VP9/JPEG，≤8192×8192），**不是相机输入** ——
> 别把它当摄像头。`v4l2-ctl --list-devices` 能看出到底是哪个。

键盘那两根线（RP2040 的 GP1 ← 板子控制台 TX、GND ← GND）见
`core/rp2040_keyboard/README.md`。串口线上是**共享的控制台**，所以动作行带
`@MOVE ` 前缀、RP2040 只挑带前缀的行。跑之前 `dmesg -n 1` 把内核日志压下去，
线上会干净很多。

**动作行还能发到哪儿**（`--keyboard-dev`，三种写法，同一份 `@MOVE n\n` 协议）：

| 目标 | 用途 | 状态 |
|---|---|---|
| `/dev/ttyPS0`（默认） | 控制台串口 ↔ RP2040 的 GP1 | ✅ 真机串口实收 |
| `stdout` | **SSH 管道**：`ssh 板子 '… --keyboard-dev stdout' \| pc_key_driver.py --stdin` | ✅ 真机验过（`--via-pl` + SSH + 驱动按键差集） |
| `udp://IP:PORT` | 走网口直发 PC | ⚠ 本机网络下板子→WSL 不通（WSL mirrored 模式 + Hyper-V 防火墙），见 `core/rp2040_keyboard/README.md` |

> `stdout` 那条**最省事**：复用已经在用的 SSH 通道，不用接引脚、不用 `usbipd`
> 挂串口、不用改防火墙。注意写 stdout 必须 `fflush`（管道是块缓冲，不刷就等
> 进程退出才吐，等于没有实时性），这点在 `core/board/tests/test_pc_key_driver.py` 钉住了。

#### 长跑稳定性（2026-10-16）

| 跑法 | 帧数 | 结果 |
|---|---|---|
| **25 分钟**（只跑闭环） | 62,816 | 失败 0、溢出告警 0、**41.6 帧/秒**均摊、收尾正常 |
| **6 分钟**（带寄存器/资源采样，360 点） | 14,862 | 失败 0、40.4 帧/秒；**溢出粘滞位从未置起**（最大占用 5 字节）、最长连续不涨秒 **0**、**VmRSS Δ+0 KB**、**句柄数 Δ+0** |

合计 **77,678 帧无一次失败、无一次溢出**，也没有内存/句柄泄漏。工具：

* `tools/soak_loop.sh` —— 板侧起闭环 → 轮询等程序出现 → 采样 → 到点用 `SIGINT` 停；
* `tools/soak_sampler.c` —— **只读** PIXSTAT/CTRL/FLEN + 进程 VmRSS/句柄数，每秒一行。

> ⚠ 采样**不能**拿 `read_frame` 顶替：它会**写 CTRL 启动**一整帧，会把正在跑的闭环搅乱。
> ⚠ 判 PID **不能**固定 `sleep 5`：相机要热身几秒，第一次长跑就是这么"程序没起来"、
> 白跑 25 分钟还没采到样（程序日志 20 MB 全是正常输出，就是采样器没起）。
> 另外 CTRL 的 [23:8] 是 PL 的**完成帧计数**，程序在跑时它跟着程序帧率走（~40/s），
> 程序不跑时 PL 会自己背靠背跑（~1000/s）—— 它只当存活/卡顿信号，**不是**应用帧率。

#### 分辨率/缩放：720p 值不值得换（2026-10-16 量过）

现在的部署用 **MJPG 640×480 + 裁中间 360 行**（不重采样）。问题是"换成 1280×720
会不会更好/更糟"。用真实平台录像（源 1920×1080）比两条路：

| 路径 | 结果 |
|---|---|
| 一步 1080p → 640×360 | 基准 |
| 两步 1080p → **1280×720** → 640×360（INTER_AREA） | 矩形数 **60/60 帧相同**、对应框最大漂移 **0.09 px**、缺口宽度**完全一致**、角色位置 **0.00** 差异 |

颜色判据的余量也完全一样（全画面 R−B 的 99% 位 51.0 / 中位 −20.0）。

**结论：没有检测精度上的理由去换分辨率**；两条路等价。所以保持 640×480（更省带宽、
解码更便宜）。真要换 720p，注意下面这条 —— 它曾经是个静默 bug：

> ⚠ `cam_live.sh` 原来在"相机比流水线高"时一律 `crop=640:360:0:Y`。对 640×480 是对的
> （裁中间几行，视野不变）；但对 **1280×720 会把 1280 宽的画面只取左半边** —— 右半场的
> 平台直接看不见，而且不报错，只表现为"检测不到东西"。已在真摄像头上验证过两种写法：
> `scale` 拍到整幅画面，`crop=640:360:0:180` 只拍到屏幕左侧。现在判据带上宽度：
> **同宽才裁、不同宽一律缩放**（`tests/test_tools_contract.py` 钉住）。

#### 视觉质检：整段真实录像（2026-10-16）

`run4/screen.mp4` 全长 6247 帧（104 s）。按 **6 段各抽 200 帧**（stride 2，等效 30fps，
合计 **1200 帧**）跑完整视觉通路：

| 段起点 | 矩形数（众数/范围） | 缺口宽度中位 | 角色漏检 | 可疑帧 |
|---|---|---|---|---|
| 0 | 8 (2–12) | 44 | **0** | 11 |
| 1000 | 14 (14–17) | 44 | **0** | 18 |
| 2000 | 14 (14–17) | 44 | **0** | 20 |
| 3000 | 14 (14–17) | 44 | **0** | 19 |
| 4000 | 14 (14–17) | 44 | **0** | 16 |
| 5000 | 14 (14–17) | 44 | **0** | 16 |

* **每一帧都检出了角色**、**每帧至少 2 个矩形**、缺口宽度在每一段都稳定在 **44 px**
  （就是画面里那条竖向通道 x≈201.5~245.5）；
* 矩形数 2~17 是**真实内容**：这段关卡把墙画成"梯状"（8 行橙条、每行左右两段），
  叠加检查确认**每个框都套在橙条上**；四角的 UI 图标与鼠标光标**没有**被误检；
* 每段 5~10% 的帧触发"只落位、不更新速度"的守卫 —— 那是带合并/段形变，属预期范围。

## 7. 还差什么

| # | 缺的东西 | 说明 |
|---|---|---|
| 1 | ~~C++ 决策~~ | ✅ 已完成：`core/cpu/cpp/`，10 条交叉验证用例 |
| 2 | ~~像素从哪来~~ | ✅ PL 侧用 `tpg_source` 已跑通；真图走 PS 参考模型也已跑通（`--ppm`） |
| 3 | ~~PL 也吃真图~~ | ✅ 已完成：PS 推像素 FIFO（`rtl/vision_pixfeed.v`）+ `--via-pl`；`tb_axi --fifo` 逐字节验过 |
| 4 | ~~移动墙的速度~~ | ✅ 已完成：`core/cpu/cpp/motion.{hpp,cpp}`（`core/vision/motion.py` 的移植，逐帧对账）+ `decide()` 里填进 `player[P_VX/P_VY]` 与 `Rect.vx/vy`。真实素材实测（`run4/screen.mp4` 前 60 帧，**它其实是 60fps**，所以要用 `--ppm-fps 60`）：加守卫前墙 max\|v\| 90% 位 54.8 / 最大 **134.6** → 加守卫后 30.2 / **31.9**（那段素材的真实运动是 **28.0**，从原始位置量出来的）。⚠ 第一版表格用了默认 `--ppm-fps 30`，所有速度都算成一半。⚠ **手持**录像（`run3/Camera.mp4`）同一算法给出中位 33.6 / 90% 57.2 —— 抖动主导，所以最终部署要固定机位。

**stamp 的两条路（2026-10-16 修）**：几何帧的 `stamp_ns` 是"采集时刻"，速度 = Δ位置/Δstamp。
**实时路径**现在用 `StdinPump` 读线程打下的**真实采集时刻**（以前写死每帧 `+= 33.3 ms`，
而相机实测交付 **40~48 fps** → 推出来的速度被系统性缩小 (30/实际fps) 倍，40fps 时低 28%）；
**离线序列**（`--ppm`/`--ppm-dir`）改用 `--ppm-fps`（默认 30）合成步长 —— 素材帧率与它
不一致时速度会整体差一个倍数，且不会报错。`tests/test_rgb_source_cpp.py` 钉住这两条。

> 用真实采集时刻后，帧间隔是**突发**的（板子实测 dt 中位 24.7 ms、10% 小于 5 ms、
> 最小 2.09 ms —— 相机成对交付）。理论上 `v += (β/dt)·r` 在小 dt 下会被放大，但
> **实测不必加下限**：真实抖动 σ=0.5 px 下速度估计误差只有 1.77（中位）/ 3.99（90%）
> 单位/秒，而真实速度是 28 —— 因为跟得稳的目标残差本身就很小，放大只在残差大时才可怕，
> 那已经由 `max_residual` 守卫挡住。（量过才决定不加旋钮。）

**快速起步（2026-10-16 加）**：α-β 常规 β 下"物体出现时就在动"要 19 帧才追到 90%，
而 `t_arr` 直接吃这个速度（墙以 60/s 逼近时会被高估成 45 s，真 3.3 s）—— 仿真有真速度、
没有这个滞后，所以这是**部署/训练不一致**的一处来源。现在前 3 帧用 β=0.3：90% 收敛
19→7 帧，稳态噪声不变，**真实录屏 60 帧的决策逐帧完全一致**（代价是轨迹头几帧在检测
抖动大时会凭空造出速度，σ=0.5 px 下峰值 ≈19 单位/秒，与最慢的真实运动同量级）。

**上板逐帧对账（2026-10-16，60 帧真实录屏）**：PC 参考模型、板子参考模型、板子**真 PL 通路**
三者给出的坐标与速度 **228/228 个矩形逐位相同**（角色也相同），墙 max\|v\| 三侧同为
中位 14.0 / 90% 15.3 / 最大 15.9。摄像头闭环 **19.4 帧/秒**（40 帧、0 失败，`@MOVE` 行实收）。`--track-log FILE` 可落盘原始矩形+轨迹，用来分辨"真在动"和"框在抖" |
| 5 | ~~上板实测~~ | ✅ 已完成（2026-10-15）：自检通过、8 帧全解码、决策 8 帧、**72～77 帧/秒**；键盘行串口实收。见 `core/fpga/docs/板上验证.md` §4/§6 |
| 6 | **RP2040 那一端** | 板子侧已验（串口实收 `@MOVE n`）；RP2040 要刷 CircuitPython + 拷 `adafruit_hid` + 接两根线。见 `core/rp2040_keyboard/README.md` |
| 7 | ~~动作怎么送到 PC（零接线）~~ | ✅ 已完成：`--keyboard-dev stdout` + SSH 管道 + `pc_key_driver.py --stdin`，真机验过 |
| 7.5 | ~~像素 FIFO 溢出路径~~ | ✅ 三层都验了：RTL 行为（协同仿真 `tb_axi --fifo`）、**真芯片粘滞位与 flush 恢复**（`tools/pix_overflow.c`：盲写 8 KB 内不丢、16 KB 时置位）、**应用层丢帧**（驱动暴露 `last_push_overflowed()`，`main.cpp` 两处调用点把溢出帧计入 bad 并跳过决策）——溢出帧的几何是"自洽但错"（CRC 会过），只打日志就决策是最难查的一类错 |
| 8 | **平台收不收合成按键** | 唯一还没验的一环，只能人眼判：`pc_key_driver.py --press 1,2,3,4 --arm --delay-s 5`（先把平台窗口点成焦点），有方向动了就说明 C 路成立、RP2040 不用买 |
