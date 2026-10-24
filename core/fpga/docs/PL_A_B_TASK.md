# PL 成型（A+B）任务书与进度

> 目标（用户 2026-10-24 定）：**A（视觉前端）+ B（相机采集/预处理）一起搬进 PL 并接进闭环**，
> 指标不劣于锁定基线 `acc9r16`（到达、0 碰撞、24/24、最小间隙 +9.92、|Δy| 6.8 px）。
>
> **范围内**：`core/fpga/`、PL 位流与开机 autoload、PS↔PL 喂数/读回接口、
> "把感知源从 `vt_sparse` 换成 PL"这一处接线。
> **范围外（绝对不动）**：决策（`field_cli_board`/M1）、`board_agent` 的准入/去重/停走、
> 场景与标定参数、相机参数、页面渲染、`@POSE`/`@STEP` 协议、NPU、`core/using/` 锁定内容。

## 验收口径

| 级 | 判据 | 状态 |
|---|---|---|
| **L0** | ≥300 帧真实相机帧，PL 输出与 CPU 参考（`core/fpga/ref/vision_ref.c`）**逐位一致**，不一致帧 = 0 | ⬜ 未做 |
| **L1** | 单帧端到端（PS 投喂首字节 → 结果可读）**≤35 ms**，连续 300 帧不掉 | ⬜ 未做（Python 喂数 560 ms，喂数器本身是瓶颈） |
| **L2** | 接进闭环 4 轮：到达、0 碰撞、24/24、\|Δy\| ≤6.8 | ⬜ 未做 |
| **P1**（前置） | 喂 1 帧 → 读回几何帧，帧契约（magic/长度/CRC）通过 | ✅ **已完成** |

## 接口契约（已核实）

基址 **0x43C00000**（4 KB 窗口，mmap `/dev/mem`）：

| 偏移 | 名称 | 说明 |
|---|---|---|
| 0x000 | CONFIG | 21 个参数寄存器（阈值等；必须与 `vision_ref.c` 的 `VR_DEFAULT_*` 一致） |
| 0x080/0x084 | DBG | 写 `{sub[3:0],idx[17:0],sel[3:0]}`，读数据（统计/直方图/列投影/行带） |
| 0x0C0 | CTRL | 写 bit0=1 启动一帧（自清）；读 bit0=busy bit1=done bit[15:8]=fcount bit16=ovf |
| 0x0C4/0x0C8/0x0CC | SEQ/STAMP | 帧序号 + 纳秒时间戳（进帧头） |
| 0x0D0 | FLEN | 只读：上一帧实际字节数 |
| 0x0D4 | PIXPUSH | 写：像素 FIFO，**整字 4 字节** |
| 0x0D8 | PIXSTAT | 读：[15:0] 缓冲字节 [16] 溢出 [17] 正在放 |
| 0x0DC | PIXCTL | 写：bit0 像素源（0=外部/TPG，1=FIFO）bit1 flush |
| 0x100 | FRAME | 几何帧字节窗口（2048 B） |

**喂帧顺序（关键）**：`PIXCTL=0b11`（flush）→ `PIXCTL=0b01` → 写 SEQ/STAMP →
**写 CTRL bit0=1 启动** → 推像素（整字）→ 等 busy 落 → 读 FLEN/窗口。
（先推像素再 start 会死等：IDLE 时推的像素被丢掉。）

**几何帧布局**（`core/board/geometry_frame.hpp`）：

```
头部 32 B : magic 'BHV1'(0x31564842) | ver u16=1 | flags u8 | seq u32 |
            n_obs u8 | n_gaps u8 | width u16 | height u16 | stamp_ns u64
body      : player 16 B + target 24 B + obstacles 36×n_obs + gaps 36×n_gaps
trailer   : CRC32(zlib) 4 B
总长      = 76 + 36*(n_obs + n_gaps)，必须 == FLEN
```

## 已完成（本轮实测）

1. **P2 位流切换**：手动 `echo bd_vision.bin > /sys/class/fpga_manager/fpga0/firmware`
   → `state=operating`，`0x43C00000` 读回 `0x28`（NPU 的 `0x40000000` 随之下线，符合预期）；
   `pl-autoload.service` 原先 **disabled**，已 `enable`（下次上电 PL 里就是我们的设计）。
2. **P1 帧契约**：新工具 `core/fpga/tools/feed_pl.py` 喂入 640×360 真实帧 ×3：
   `magic=0x31564842 ver=1 seq=1/2/3 640x360 need=76 flen=76 crc OK`、无 overflow。
3. **现状瓶颈（明确）**：Python 逐字写 `0x0D4` → **560–600 ms/帧**，是喂数器本身慢
   （172,800 次 mmap 写），不是 PL 计算慢。文档里 C 路径的基线是 **29.96 ms/帧**。

## L0 的第一轮对拍（2026-10-24，未通过，但有明确结论）

同一帧同时喂 PL 与 C 参考（`/tmp/visref`），**输出不一致**：

| 帧 | 参考 `vision_ref.c` | PL（当前板上位流） |
|---|---|---|
| `stim_color/000000.ppm` | n_obs=8 n_gaps=4 player=1 bands=4 | **n_obs=0 n_gaps=0** |
| `run4_seq/000000.ppm` | n_obs=2 n_gaps=1 player=1 | **n_obs=0** |
| 合成 `page_rgb`（墙色 255,240,200） | **n_obs=4 n_gaps=2 bands=2 span=[20,599]** | **n_obs=0 bands=1 span=[3,635]** |
| 合成 `page_bgr`（墙色 200,240,255） | 0 | 0 |

**已排除**：像素没进 PL。证据：喂全白 → `n_labels=160`，喂全黑 → `n_labels=0`，
喂合成页 → `n_bands=1 / x_min=3 / x_max=635`。**数据确实进了流水线**。

**已排除**：寄存器默认值。21 个参数读回 `[40,110,60,195,45,10,90,5,4,5,5,25,2,12,1,8,8,60,1500,45,160]`，
前 7 个与 `VR_DEFAULT_TH`（40/110/60/195/45/10/90）逐项一致。

**两个待证伪的假设**（下一轮按顺序验证）：

1. **板上位流比当前 RTL 旧**：`bd_vision.bin` 是 10-15 出的，而 `vision_pixfeed.v`
   也是 10-15 才加的（commit `b5428aa`）。若位流早于该 commit，FIFO 路径/判据可能不是现在这份 RTL。
   → 用 Vivado 从当前 RTL 重出 bit（已获授权），同一批帧重跑对拍。
2. **喂数器的字节打包与 FIFO 期望差一个偏移**：证据是 PL 的 `span` 跑到 `[3,635]`
   （参考是 `[20,599]`），像是整行都被判成"墙色"——与"颜色通道/字节错位"一致。
   → 用 RTL 自带的 `tb_axi --fifo` 逐字节对比（sim 已能证明 RTL 侧），再用 1 像素宽的
   标记图案定位偏移量。

## 下一步（按顺序）

1. **P3 / L0**：同一批帧（≥300 张真实 640×360 帧）分别喂 PL 与跑 C 参考
   （`gcc -O2 core/fpga/ref/vision_ref.c -lm` → `ref <ppm> <out.bin>`），**逐字节比**；
   不一致帧必须是 0。先解释当前 `n_obs=0 / n_gaps=0`（测试帧本身很暗，p99=129）。
2. **P4 / L1**：喂数器换成批量写（或 `udmabuf`/DMA），把 560 ms 压到 ≤35 ms 端到端。
3. **P5 / L2**：`board_agent` 的输入从 `/tmp/vt.jsonl`（CPU `vt_sparse`）换成 PL 帧；
   只动这一处接线，跑 4 轮对 L2 判据。
4. **P6 / B**：把 YUV→RGB、正射、行剖面搬进 PL（相机是 USB UVC，PS 必须先收再送 PL）。
