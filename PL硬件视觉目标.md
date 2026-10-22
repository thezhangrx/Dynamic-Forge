# PL 硬件视觉目标（2026-10-21 定）

> 这份文件是"PL 硬件视觉"这件事的**目标与验收口径**。改动前先读；验收数字必须来自相机链路
> 的真实帧，不许用合成帧糊（见 `AGENTS.md` R1）。
> 相关文档：`板端资源盘点.md`（有什么/没有什么）、`评分标准差距清单.md`（任务与工时）、
> `NPU接入方案.md`（NPU 侧）。

## 0. 先纠正基线（重要，别再用错数字）

此前项目笔记里"LUT 87.94%、时序 not met、PL 上不了板"三条**全部作废**，它们来自
`data/fpga/vivado_synth/` 那次 **200 MHz 纯综合测试**（`synth.tcl` 第 1 行自己写着"综合测试"）。

真实设计（PS7 + vision IP 的 block design，2026-10-15 实现）实测：

| 项 | 实测 | 证据 |
|---|---|---|
| 时钟 | **50 MHz**（`PS7 FCLK_CLK0`，AXI 接口也对齐到 50 MHz） | `D:\FPGAWORKS\vivado_impl\bd_vision.tcl:112-122` |
| 时序 | **All user specified timing constraints are met**，WNS ≈ **+0.9 ns** | `bd_vision_timing.rpt` |
| 资源 | Slice LUT **23,402 / 78,600 = 29.77%**；LUT as Logic 12,286 (15.63%)；FF 6,564 (4.18%)；BRAM 82.5/265 = **31.13%**；DSP 46/400 = 11.5% | `bd_vision_util.rpt` |
| 位流 | `bd_vision.bit` 5,980,023 B（10-15 14:12）→ 板上 `/lib/firmware/bd_vision.bin` 5,979,936 B（差 87 B 头，同一设计） | `ls` + 板上只读 |
| 板上状态 | `/sys/class/fpga_manager/fpga0/state = operating`；`0x43C00000` 寄存器有响应 | 板上只读 |
| 算法等价性 | RTL 与 C 参考模型**逐位一致**（`core/fpga/cosim` + `pytest`） | `core/fpga/tests/test_cosim.py` |

**结论**：PL 已经上板、已经在跑、50 MHz 时序收敛、只用了 30% 资源。
真正缺的只有两件事：**① 从没接进闭环；② 没有"上板投喂→读回"的等价性/时延数据。**

## 1. 目标（一句话）

把板端感知从 CPU 搬到 PL，在**逐位等价 + 闭环指标不掉**的前提下，把感知单帧时间从 CPU 的
**~87 ms（11.5 fps）压到 ≤35 ms**，并释放一个 A53 核给决策与通信。

## 2. 基线数字（要打败的对象）

| 项 | CPU 版（现状，`vt_sparse`） | 来源 |
|---|---|---|
| 感知帧率 | 11.5~12 fps | r11 `summary.json` / `vt.jsonl` |
| 单帧时间 | ≈ 83~87 ms | 1 / 11.5 fps |
| 墙行误差 | median −0.3 px、p90 +0.9 px（r11） | `check_wall_rows.py` |
| 闭环成绩 | 24/24 凭信息、0 碰撞、min_clear −10.0（r11 干净） | `audit_crossings.py` |
| 喂数带宽 | PS 写 ≈ 40 MB/s（`axi_bw.c`）→ 640×480×3 = 921,600 B ≈ **23 ms/帧** | `core/board/tools/axi_bw.c` |
| PL 计算量估算 | 307,200 px ÷ 50 MHz = 6.1 ms/遍；RTL 是多遍（行带/列投影/墙段/CCL）→ **10~15 ms** | 估 |

→ 预期 PL 单帧 ≈ 23（喂数）+ 12 ≈ **35 ms**，比 CPU 快 **2.5 倍**，且时延固定。

## 3. 验收指标

### L0 正确性（必须）
- 同一批 **≥300 帧真实相机帧**（用 r11 的 `cam_rec.mkv`），PL 输出与 CPU 版
  **逐位相等**：墙行、缺口 `gl/gr`、角色、`no_gap_y` 全部相同；
- **不一致帧数 = 0**；任一不一致必须先查清楚，不许"差不多就过"。

### L1 性能（必须）
- 单帧端到端（**PS 投喂首字节 → 结果可读**）**≤ 35 ms**；
- 连续 300 帧不掉帧、不丢帧；
- PS 侧只做投喂+读回，占用 **≤ 1 核的 50%**。

### L2 系统（必须）
- 接进闭环（PL 作为感知来源）后，同一场景跑 **4 轮**：
  - reached = 4/4，碰撞 = 0；
  - `audit_crossings` ≥ 24/24 凭信息（不许有盲穿/侥幸）；
  - 与 CPU 版的决策序列差异 ≤ 5%。
- 达标线是**不低于 CPU 版**，不是"偶尔一次全对"（参考 `评分标准差距清单.md` 表 B）。

### L3 拓展（加分）
- 喂数改 **DMA**（板上已有 `/dev/u-dma-buf-mgr`、`/dev/udmabuf0`）或改灰度/行抽样，
  把 23 ms 喂数压到 **<10 ms** → 单帧 **≤20 ms（≥50 fps）**；
- 或给出"PL 与 AI_Mate 分时复用"的方案与实测（用 `fpga_manager` 运行时重载位流）。

## 4. 边界（明确不做）

- **不重写算法**：以 `core/fpga/ref/vision_ref.c` 为准，逐位一致是硬约束；
- **不动决策**：Dijkstra 留 CPU；
- **不动相机链路**、不追相机 12 fps 以上的上限；
- 先不冲 100 MHz（50 MHz 已收敛，先拿数据；要冲另开一项）。

## 5. 待拍板的 4 个决策

| # | 决策 | 选项 | 备注 |
|---|---|---|---|
| D1 | PL 与 CPU 的关系 | ① 先并行旁路做等价性证据，再切主路；② 直接替换 `vt_sparse` | 建议 L0 通过后再切 |
| D2 | 喂帧方式 | ① 维持 PS 写 `0x0D4`（23 ms/帧，简单）；② 直接上 DMA | ① 够 L1，② 才够 L3 |
| D3 | 时钟 | ① 锁 50 MHz；② 试 100 MHz | ②要重跑实现收时序 |
| D4 | 我能否动 Windows 侧 Vivado | 允许 / 不允许 | `cmd.exe /c D:\Vivado\Vivado\2018.3\bin\vivado.bat -mode batch`；改 RTL、重跑实现、出 bit 都以此为前提 |

## 6. 零风险下一步（都只读，需授权才做）

1. `vision_board --device devmem://0x43C00000` 对**板上已加载的设计**跑 1 帧，
   确认寄存器通、输出与 C 参考一致；
2. 测"投喂 + 读回"的真实毫秒数 → 填 L1 基线；
3. 比 `bd_vision.bin` 与 `bd_vision.bit` 是否同一版本（大小差 87 B 头）。

## 7. 证据索引

| 内容 | 路径 |
|---|---|
| RTL | `core/fpga/rtl/`（`vision_top/axi/ccl/div/pixfeed/tpg/selftest`，3487 行） |
| C 参考模型 | `core/fpga/ref/vision_ref.c` |
| 协同仿真台 | `core/fpga/cosim/tb_vision.cpp`、`core/fpga/tests/test_cosim.py` |
| 板端驱动 | `core/board/vision_driver_devmem.cpp`（`devmem://0x43C00000`）、`vision_regs.hpp` |
| 真实设计工程（Windows） | `D:\FPGAWORKS\vivado_impl\`（`bd_vision.tcl`、`bd_pack.tcl`、`*.bit`、报告） |
| 200 MHz 综合测试（**别当基线**） | `data/fpga/vivado_synth/{synth.tcl,util_final.rpt,timing_final.rpt}` |
| CPU 版感知 | `core/board/tools/vt_sparse.cpp`（板上 `/root/tools/vt_sparse`） |
| 判定仪器 | `check_wall_rows.py`、`audit_crossings.py`、`check_run_logs.py` |

## 8. 维护规则

- 每完成一条，把实测数字填进第 3 节并在第 7 节写 `run_*` 目录；
- 任何"达标"结论必须能指向：**真实帧批次 + run 目录 + 一条可重跑的命令**；
- 参数/口径变化时先改这份文件，再改脚本（不许脚本和文档各说一套）。

---

## 9. 接口层接线结果（2026-10-21 22:36~22:50，只读/只写寄存器，未改 RTL）

目标：把「现在就值得接」的四项（参数表 / 几何帧打包·CRC·序列号 / 帧状态与时序 / 统计汇总）
在**真实相机帧**上验证到"可读、可查、可交叉核对"，全部通过 PS 侧 `/dev/mem` 直接访问寄存器完成。

### 9.1 参数/阈值表（21 个 AXI 寄存器）
读回值（十进制，顺序 = `CfgAddr 0..20`）：

```
[40, 110, 60, 195, 45, 10, 90, 5, 4, 5, 5, 25, 2, 12, 1, 8, 8, 60, 1500, 45, 160]
```

驱动侧 `--verify-both` 报告的「配置对账：21 项，不符 0」与此一致 → **参数面接线可用**。

### 9.2 几何帧打包 / CRC32 / 序列号（寄存器窗口 0x100 起，2048 B）
在 PS 侧独立解析帧头并用 `zlib.crc32` 重算：

```
magic=0x31564842 ('BHV1') ver=1 flags=2 seq=0 n_obs=8 n_gaps=3 img=640x360
need = 76 + 36*n_obs + 36*n_gaps = 472   FRAME_LEN(0x0D0) = 472   -> 一致
CRC32 计算值 0x781475B6 == 帧尾 0x781475B6                       -> 一致
```

→ **帧契约（长度/魔数/版本/序号/CRC）在真实帧上完全对上**，PS 侧可独立校验。

### 9.3 帧状态与时序（控制/状态寄存器 0x0C0）
```
CTRL = 0x00001602 : busy=0 frame_done=1 fcount=22 overflow=0
PIX_STAT(0x0D8) = 0 : 缓冲空、无溢出、未在放
```
- 连跑 21+1 = 22 帧后 `fcount=22` → **计数器准确**，可直接当"丢帧/看门狗"判据；
- `frame_done` 位有效、`busy` 正确回落 → start→done 握手可用；
- 控制面单次寄存器访问 **3.6 µs**；读完整 2048 B 帧窗口 **1.74 ms**（相对 30 ms 的喂数小一个量级）。

### 9.4 统计汇总（调试口 0x080/0x084）
| 选择 | 含义 | 实测 |
|---|---|---|
| sel=7,sub=0 | `n_bands` | 5 |
| sel=7,sub=1/2 | `x_min / x_max` | 0 / 631 |
| sel=7,sub=3 | **`n_obs`** | **8**（与帧头 n_obs 一致）|
| sel=7,sub=4 | **`n_gaps`** | **3**（与帧头 n_gaps 一致）|
| sel=7,sub=5 | `n_labels`（CCL 标签数） | 256 |
| sel=7,sub=6 | `n_segs` | 1 |
| sel=3, idx=100/180/260/340 | 行直方图（每行亮列数） | 130 / 38 / 14 / 293 |
| sel=4, idx=100/320/500 | 列投影（持续台阶） | 40 / 58 / 42 |
| sel=5/6, idx=0,1 | 行带上下沿 | (0,44) (74,155) |

→ **行直方图、列投影、连通域/障碍统计都能从 PS 读到，并且与帧头交叉一致**。

### 9.5 结论
- Tier-1 四项的**接口层已接通并交叉验证**（参数面、帧契约、状态/时序、统计面）；
- 还没接的是"**进闭环**"：目前是脚本/探针在读写寄存器，`vt_sparse` 那条决策链仍然只用 CPU 视觉；
- 下一步（需另开指令）：把 PS 侧的感知调用从 `vt_sparse` 换成"PL 帧窗口 + 统计口"，并按 §3 的 L0~L2 验收。

### 9.6 帧节拍（周期精确性）与参数可写性

在 PS 侧轮询 `CTRL[23:8]`（帧计数）测 PL 的完成节拍（板端同时在推 40 帧）：

```
2s 内 29 帧完成；相邻帧间隔 mean=29.96 ms  min=29.93  max=30.06  sd=0.03 ms
帧计数增量合计 = 29（无丢帧）；最终 fcount=102
驱动侧 40 帧总时 1200 ms = 30.0 ms/帧（33.3 fps）
```

- **抖动 sd = 0.03 ms（约 0.1%）** → 这是"周期精确时序"最直接的证据（周期精确是 PL 相对 Linux 的真实优势，比"更快"更靠得住）；
- 帧计数连续、`overflow=0` → 可直接当"丢帧/看门狗"判据。

参数表**可写可读**（不只是能读）：

```
原值 cfg[0]=40 → 写 30 读回 30（一致）→ 写 40 读回 40（一致）
```

→ PS 每轮下发阈值/参数这条路是通的。

### 9.7 固化工具
上述检查已固化成一个板端探针（可重复执行、有判据、无第三方依赖）：

```
ssh root@192.168.137.242 'python3 /tmp/pl_probe.py'              # 全量自检（参数/帧契约/统计/控制面耗时）
ssh root@192.168.137.242 'python3 /tmp/pl_probe.py --cadence 2'  # 只测帧节拍（需板端有进程在推帧）
```

源码：`core/board/tools/pl_probe.py`。自检通过标准：帧契约 OK（长度+CRC）、
`n_obs/n_gaps` 与帧头一致、帧计数连续。

主机侧一条命令跑完（自动推探针 + 按判据给结论，rc=0 表示通过）：

```
bash core/board/tools/pl_check.sh              # 全量自检
bash core/board/tools/pl_check.sh --cadence 2  # 只测帧节拍
```

### 9.8 PS 触发一帧 → PL 报 done（触发/握手/纯计算时间）

PS 直接写 `CTRL.bit0=1` 触发一帧，然后轮询帧计数等 `done`，20 次：

```
PS 触发 -> PL 报 done: n=20  min=11.34 ms  median=12.96 ms  max=12.97 ms  超时 0
触发后 fcount=122  busy=0  flen=184
```

三个结论：

1. **触发/握手链路可用**：PS 可以自己起帧、等完成，`busy` 正确回落，20/20 无超时；
2. **PL 纯计算一帧 ≈ 13 ms**（11.3~13.0 ms），与 PS 参考模型的 **13.5 ms 基本持平**——
   也就是说 **PL 在算法上并不更快**；
3. 30 ms 的端到端里，**约 17 ms 是"PS 推像素"的过路费**，这才是要消灭的部分（DMA）。

> 修正对 §1/§2 的表述：PL 的价值应改为「**确定性时延（sd 0.03 ms）+ 释放一个 A53 核 +
> 给 AI_Mate 供数**」，而不是"比 CPU 快"。要"更快"必须先上 DMA。
