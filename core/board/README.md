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

## 7. 还差什么

| # | 缺的东西 | 说明 |
|---|---|---|
| 1 | ~~C++ 决策~~ | ✅ 已完成：`core/cpu/cpp/`，10 条交叉验证用例，见 `core/cpu/cpp/README.md` |
| 2 | **像素从哪来** | 现在 `vision_axi` 的像素输入要外部喂。bring-up 可先用 `vision_tpg`（自检图案），真实相机要再加一层采集 |
| 3 | **移动墙的速度** | 线协议不带速度，PS 侧要按 `stamp` 差分出来再填进 `Rect.vx/vy` |
| 4 | **上板实测** | 本机只验到"驱动逻辑对 + 寄存器契约对 + 决策与 Python 逐位一致"；**基址、AXI 握手、真实时序都要上板才知道** |
