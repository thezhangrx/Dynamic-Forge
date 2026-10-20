# `core/board/tools/` —— 板上验证用的小工具

这些是**在板子上直接 gcc 编**的小程序（不是 `core/board/` 那套 C++17 交叉编译的
驱动），用途是把 PL 侧的设计在真硅上验证掉。每个都是单文件、无依赖。

```bash
# 本机先过一遍告警（和 pytest 里那条守卫同一条命令）
gcc -O2 -Wall -Wextra -std=c11 -Werror -c core/board/tools/axi_soak.c -o /dev/null

# 送到板子上编（板子上 gcc 直接编，不需要交叉工具链）
scp -i data/fpga/board_ssh/board_key core/board/tools/*.c root@192.168.137.242:/root/tools/
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'cd /root/tools && for f in *.c; do gcc -O2 -Wall -Wextra -std=c11 -Werror -o ${f%.c} $f; done'
```

---

## 平台 v2 闭环工具（2026-10-19 起，先看这一组）

> 下面「AXI/位流」那部分是 10-15 的，仍然准，但**不包含** v2 的决策/闭环工具。

| 工具 | 跑在哪 | 用途 |
|---|---|---|
| `field_decision_cli.cpp` | **板上** | M1 代价场决策器。读 `@STATE`，输出 `@MOVE`；`--echo` 额外回 `@ACK <seq> <act>` 用于端到端延迟；`--view-h H` 只按当前视口建势场 |
| `field_loop.py` | 本机 | **真值在环**：仿真 `@STATE` → ssh → 板子 → `@MOVE` → 仿真。出到达/碰撞/间隙/切换/延迟，写 `m1_state/actions/board.log`；`--video-out` 顺带出势场+轨迹 mp4 |
| `full_loop.py` | 本机 | **完整闭环**：跟随视口投屏 + 板子 M1 + 相机白框 VT 新墙暂停/恢复。出 `_follow.mp4` / `_field.mp4` / `_camera_vt.mp4` + `events.jsonl`（pause/resume）+ `full_loop.json`；需要板子相机 8099 与 WSLg |
| `page_view.py` | 本机 | 把 8100 的 MJPEG 页面显示到 WSLg X11 窗口（默认 (320,60) 1280×960），给板子相机拍 |
| `mjpeg_client.py` | 本机 | 最小 MJPEG 客户端（`cv2.VideoCapture(url)` 在 multipart 上会卡；这里按 JPEG SOI/EOI 拆帧） |
| `bench_field_methods.py` | 本机 | M1 vs M4 本地基准（同场景多起点） |
| `test_static_local.py` / `test_field_local.py` | 本机 | 本地 Python 上的纯转向 / 势场对照 |
| `record_field_video.py` | 本机 | 用上面的 state/actions 出势场 + 轨迹视频；`--view-h H` 出跟随视口（640×H） |
| `virt_truth_selftest.cpp` | 本机 | `core/board/virt_truth.hpp` 的 v2 自检（静态墙冻结 / 相对观测 / 零校正） |

```bash
# 真值在环（真板子；120 Hz 才够，30 Hz 过缺口会擦墙）
Env/bin/python core/board/tools/field_loop.py --max-seconds 30 --state-hz 120

# 同上，并录一段"势场 + 墙 + 轨迹 + 角色"的 mp4（20 fps；跑完用完整日志出片）
Env/bin/python core/board/tools/field_loop.py --max-seconds 30 --state-hz 120 \
    --record-hz 20 --video-out /tmp/m1_board_field.mp4

# 视角跟随：40 道墙、640×240 视口；势场只用当前视口里的墙，目标点由 --goal 告知
Env/bin/python core/board/tools/field_loop.py --scenario configs/static_wall_40.json \
    --view-h 240 --max-seconds 60 --state-hz 120 --record-hz 20 \
    --video-out /tmp/m1_follow_40.mp4

# 完整闭环：跟随视口 + 板子 M1 + 相机白框 VT 新墙暂停/恢复
#   需要：板子相机 8099 在推流、WSLg DISPLAY=:0、8100 空闲
#   投屏页是**不带势场**的干净平台（实测势场会把白框 VT 的墙对比翻转成 0 道）；
#   势场/代价场单独录进 _field.mp4。--send-all-walls 与 field_loop 的板子口径一致；
#   去掉它就是"新墙先不发给板子、等视觉确认"的实验模式。
Env/bin/python core/board/tools/full_loop.py --scenario configs/static_wall_wide.json \
    --board ssh --speed 0.5 --max-seconds 30 --send-all-walls \
    --state-hz 120 --render-hz 15 --field-hz 5 --vt-hz 3 \
    --port 8100 --out-dir data/fpga/loop_logs --tag demo --keep-serving

# 同一条链路、无相机自测（只过板子闭环，pause 数必为 0）
Env/bin/python core/board/tools/full_loop.py --board local \
    --local-bin /tmp/field_decision_cli_host --vt-source none --viewer none \
    --send-all-walls --speed 2 --max-seconds 20 --port 8102 \
    --out-dir /tmp/full_loop_test --no-keep-serving

# 无板自测：先编一份本机二进制
g++ -std=c++17 -O2 -o /tmp/field_decision_cli_host core/board/tools/field_decision_cli.cpp
Env/bin/python core/board/tools/field_loop.py --board local \
    --local-bin /tmp/field_decision_cli_host --max-seconds 30 --speed 4

# 虚拟真值口径自检
g++ -std=c++17 -O2 -I core/board -o /tmp/vt_selftest \
    core/board/tools/virt_truth_selftest.cpp && /tmp/vt_selftest
```

口径见根目录 `虚拟真值约定.md`：**VT 是唯一控制真值；视觉只给相对观测和对照。**

---

## ⚠ 先读这一条：哪些工具**会写** AXI

| 工具 | 写寄存器？ | 在**旧**位流（含 AXI 写死锁）上 |
|---|---|---|
| `read_frame --watch` | **不写**（纯读） | 安全，不会挂死 |
| `read_frame` | 写 `CTRL` bit0 | **会挂死 CPU** |
| `axi_soak` | 写配置块 + `CTRL` | **会挂死 CPU** |
| `dbg_probe` | 写配置块 + `DBG_SEL` | **会挂死 CPU** |

"挂死"是指 **CPU 卡在 AXI 上、网络掉线，且复位键无效，只能彻底断电冷启动**。
（原因见 `core/fpga/docs/板上验证.md` §4：`s_awready`/`s_wready`/`s_arready`
原来全绑在 `idle_all` 上，AW 一被接收 `wready` 就掉，W 永远进不来。）

**所以顺序永远是：先用只读工具确认位流加载成功，再用会写的工具。**
`deploy_and_verify.sh` 就是照这个顺序编排的，推荐直接用它。

---

## 推荐入口：`deploy_and_verify.sh`（在 **WSL 侧**跑）

```bash
bash core/board/tools/deploy_and_verify.sh
```

送位流 → 经 `fpga_manager` 加载 → `read_frame --watch`（零风险探针）→
`axi_soak`（真正的判据）→ 汇总。它会打印位流 md5，并且**拒绝烧写比 RTL 还旧的
位流**（D: 上会一直留着上一轮的产物，光看"文件存在"会把含写死锁的旧位流烧上去，
实测拦下过一次）。

可调项：`SOAK_ROUNDS`（默认 3000）、`SOAK_FRAMES`（默认 8）、
`EXPECT_MD5`（精确校验位流）、`BOARD`（默认 `192.168.137.242`）。

---

## 逐个工具

### `read_frame.c` —— 读回几何帧，和协同仿真的期望值逐位对账

端到端验证 `tpg_source → vision_axi → AXI 窗口` 这条链。

```bash
./read_frame              # 启动 1 帧并对账
./read_frame 40           # 连续 40 帧，统计图案命中分布
./read_frame 1 --watch    # **只读**：不启动，光等自由运行的帧
```

期望值来自 `core/fpga/cosim/tb_tpg.cpp` 的 `Vvision_selftest`（8 个图案的帧长
184 与 CRC32），块设计里的图案源是同一个 `vision_tpg`、同样的默认配置，所以两
组常数应当完全相同。

**正确时序是"写 `CTRL` bit0 → 轮询帧计数/busy → 读 `flen` → 读窗口"。**
`vision_top` 在 `S_RECV` 里等 `din_sof` 对齐，所以任意时刻启动都安全，不必和
图案源对相位。

> ⚠ 原版是"只等 `frame_done` 翻转、从不写 start"，**从来不可能跑通**：
> `tpg_source` 只送像素、不启动几何帧，`frame_start` 的唯一来源就是 AXI 的
> `ADDR_CTRL` bit0（已逐条核实，见 `板上验证.md` §6.2）。所以 `--watch` 的
> **预期结果就是"帧计数不动"** —— 那不是故障。

退出码：`0` 全部一致 / `2` 有帧不符或超时 / `3` 地址没被 PL 译码
（位流没加载 —— 程序会明确报出来，而不是崩在 mmap 访问上）。

### `axi_soak.c` —— AXI **写通路**加压，专打两个曾经的盲区

```bash
./axi_soak                 # 2000 轮字级压测 + 8 帧端到端
./axi_soak 20000           # 加码
./axi_soak 2000 --no-frame # 只压写通路，不启动帧
./axi_soak 2000 --seed 7 --frames 32
```

阶段是刻意排序的（先只读、再单次写、再压测、最后端到端）：

| 阶段 | 做什么 | 证明什么 |
|---|---|---|
| A | 快照 21 个可写配置寄存器（纯读） | 地址被译码、默认值对得上 `VR_DEFAULT_REF_PARAMS` |
| B | **金丝雀：单次**配置寄存器写→读回 | 要挂就挂在这一行，失败点最清楚 |
| C | 字级压测：随机地址/数据（按位宽掩码） | 写读回一致；确定性 PRNG + FNV 指纹，同 seed 两次跑结果完全一样 |
| D | **字节/半字选通 + 非对齐窄读**：真的用 8/16 位访问打各字节通道，并读非对齐地址 | WSTRB 修复、以及「窄传输地址译码」修复有没有进这个位流 |
| E | 复原 A 的快照并逐字复核 | 不然后面帧的比对会被污染 |
| F | 端到端 N 帧（写 start → 等帧 → CRC） | 整条链 |
| G | 存活确认 + 汇总 | 能打印到这行 = 整轮写没挂死 CPU |

**为什么 D 必须上板**：仿真覆盖的是时序组合，而"真实主控的 WSTRB 到底长什么样"
是**我们对主控的假设** —— Zynq GP 口对 8/16 位窄访问会产生什么样的字节选通，
只有 CPU 自己知道。oracle 照 AXI-Lite 的字节使能定义在 C 里**独立**算
（`merge_lanes()`），不复用 RTL 的 `wstrb_merge`。

配置块（`0x044`，走「读旧值再合并」）与就地合并的 `SEQ/STAMP` 两条路径都要过。

**窄读同样要打**：RTL 的读侧原来也是 `case (ar_a)` 拿字节地址比字对齐常量，
所以读 `0x0C5` 会返回 0 而不是 `SEQ` 的值。写侧和读侧是**同一个错**的两半，
只测写会漏掉一半。

> 「对齐地址 + 选通掩码」与「非对齐窄传输」都是**合法**的 AXI 主控行为。
> 真硅上暴露的四个 bug 全是「测试台只覆盖了其中一种、假设了另一种」——
> 所以协同仿真里两种都覆盖，板上则覆盖 CPU 真正会发的那一种（非对齐窄传输）。

退出码：`0` 全过 / `2` 有失配 / `3` 地址没被译码。

### `dbg_probe.c` —— 诊断：AXI 通路 + 视觉流水卡在哪一步

```bash
./dbg_probe
```

配置寄存器写读回、`ctrl`/`flen`、调试口（`sel=7` 的流水线内部状态、
`sel=0/1/2` 的帧存抽样）。**会写寄存器**，见上面的警示表。

### `fix_boot_stall.sh` —— 修"上电要干等 90 秒才进得去系统"

```bash
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'bash -s' < core/board/tools/fix_boot_stall.sh
```

这个和 PL 无关，是板子 rootfs 的缺陷：厂商镜像里**没有 systemd-udevd**，
于是 `dev-ttyPS0.device` 永远等不到"设备就绪"，干等 90 秒 JobTimeout，
把 `Reached target Multi-User System` 一起卡住（`systemd-analyze` 实测 userspace
1 分 30.8 秒）。屏蔽掉那个注定失败的重复 getty 后降到 5.7 秒。详见
`core/fpga/docs/板上验证.md` §5。

脚本幂等，且有一道安全闸：若发现提供串口登录的 `getty@ttyPS0.service` 不是
enabled 就直接退出 —— 免得把唯一的登录口弄丢。

---

## 契约测试

`core/board/tests/test_tools_contract.py` 守着这批工具，因为它们是"PS 侧独立维护
的一份 PL 认知"，地址错一个字节不会报错、只会静默读写**别的寄存器**：

- C 工具的 `OFF_*` ↔ `vision_axi.v` 的 `ADDR_*`；
- `axi_soak.c` 的 `CFG[]` 表 ↔ `vision_top.v` 的 `CFG_*` 序号、
  `reg [N:0] r_xxx` 位宽；
- RTL 里每个可写 `CFG_*` 都要在表里（防"新加寄存器但板上压测没跟上"）；
- 每个 `.c` 都要 `-std=c11 -Wall -Wextra -Werror` 零告警编过；
- 每个 `.sh` 过 `bash -n`；禁止裸 `usleep`。

**所以新增偏移量时要用具名常量，不要内联裸数字** —— 裸数字这个测试守不住
（`dbg_probe.c` 原来就是裸数字，已被这条测试抓出来改成具名常量）。
