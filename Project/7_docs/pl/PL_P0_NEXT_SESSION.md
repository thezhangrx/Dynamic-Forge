# PL P0 交接（给新对话）—— 口径、已完成、还剩什么

> 写于 2026-10-25 深夜。上一段会话**上下文耗尽**，目标标 blocked。
> 本文只写**本次会话实测到的**与**已经定下的口径**；没验证过的会标「未确认」。
> 目标原文见 `PL_P0_P1_MIGRATION.md` 的 R77/R88 与本文 §3。

---

## 0. 新对话**先读这些**（顺序照做，别跳）

| 顺序 | 文件 | 为什么 |
|---|---|---|
| 1 | `AGENTS.md` | **硬规则**。T0（动手前先复述理解）、R0（提问时不动手）、R1/R1b/R2/R3/R4/R5/R6/R7。R1b 锁定的基线是 `acc9r16`；R7 是"不盲修"的四条铁律 |
| 2 | `core/fpga/docs/PL_HANDOFF.md` | 上一段 A/B 会话的总交接（相机域链路、相机参数、板子地址、运维纪律） |
| 3 | `core/fpga/docs/PL_P0_P1_MIGRATION.md` 的 **R52–R88** | P0（正射搬进 PL）的完整证据链，**每一步都有数字和遇到问题**。R77=整体复盘，R88=待办清单 |
| 4 | 本文 | 口径汇总 + 不要重复做的事 |
| 5 | 代码（按需） | 见 §6 文件地图 |

**项目结构现状（不再造轮子）**：PL 侧 RTL 在 `core/fpga/rtl/`（`vision_axi` 是顶层包装：配置寄存器 + 像素源三选一 + 正射子系统 + `vision_top` 判据），
协同仿真在 `core/fpga/cosim/`（每个模块一个 `--Mdir`，`Makefile` 里有 `run-ortho/run-sys/run-lc/run-axim/run-divp`），
测试在 `core/fpga/tests/`（pytest），构建在 `core/fpga/synth/`（Vivado batch tcl），板端工具在 `core/board/tools/` 与 `core/fpga/tools/`。

---

## 1. 本次会话确立的**口径**（已定，不要再改）

1. **正射（ortho）在 PL 里做，PS 不再做正射、不再逐字推像素。**
   链路：`PS 把 640×480 RGB 帧 memcpy 进 udmabuf → 写 FRMBASE → FRMCTL.start
   → PL 走 HP0 按行读 → 行缓存(32 行) → 正射 → 相机域适配 → vision_top → 几何帧窗口(0x100)`。
   PS 只负责"搬帧 + 下发 13 个系数 + 读几何帧"。
2. **C 参考 ≡ RTL 逐位一致**是硬不变量（`vision_ref.c` ↔ `vision_top`；`ortho_fixed_ref.cpp` ↔ `vision_ortho`；
   `camadapt_ref.py`/pl_vision.cpp:403 ↔ `vision_camadapt`）。任何改动先过对应逐字节门禁。
3. **系数由 PS 从 `quad.json` 现算**（`field_frame::homography → rectify::invert → q16/q24`），
   **不许手抄常数**。探针的 `--quad` 已自证：算出来与 `VR_DBG_HQ` **0 项差异**、`c0..c1=80..560`。
4. **`vision_top` 的 `din_sof` 是"整帧第一像素"**，行靠 `din_eol` 推进。
   正射/适配器都必须按这个口径出 `sof`（**这是 R82 抓到的真 bug，不再修改回"每行行首"**）。
5. **AXI 读地址 = FRMBASE + line*1920**。`vision_ortho_sys` 的 `frm_base` 输入就是干这个的（R84 的 bug：漏加）。
6. **填充完成只认行缓存的 `fill_done`**（不是主口的 `m_done`）；行缓存的读/写都是**像素**（24 位），不按字节。
7. **位流只重建"本地已修好的真 bug"**，不为调试重建；**不 hot-swap 位流**，一律 JTAG 只配 PL（R73）。
8. **量具与测物分开**（R82 教训）：AXI 从机模型的"握手结算"每拍只能做一次；`busy` 有 0 拍窗口，
   探针每帧起跑前必须等 `busy=0`（P5）。
9. **正射/整链的吞吐目标**：50 MHz 下正射 ≈14.7 ms、全 PL（正射+适配+判据+打包）≈20.1 ms；
   **板上实测端到端 20.16 / 20.34 ms**，判据 ≤35 ms 已达标。

---

## 2. 已经完成（都有数字，不用重做）

| 项 | 结论 | 证据位置 |
|---|---|---|
| 行缓存（32 行、像素存储） | `tb_linecache` 2560 点全对、未填充必 miss | R66/R69 |
| 正射（流水线除法 II=1） | 507 ms → **9.9 ms**（直供）/全链路 **14.7 ms**（@50 MHz），逐字节 0 不符 | R63/R66 |
| 全链路（DDR→AXI→行缓存→正射） | `tb_ortho_sys`：376 次填充、90240 拍、**不符 0** | R66 |
| **P0 集成（+ vision_top）** | `tb_p0`：`FLEN=292`、**几何帧逐字节 0/292 不符**、20.14 ms | R82 |
| 相机域适配（RTL） | `tb_camadapt` vs `camadapt_ref.py`：**不符 0** | R72 |
| 位流 | `bd_pack→bd_vision→bootgen` 0 错误，**WNS=+0.748 / WHS=+0.053 ns** | R85 |
| 板上实测 | 端到端 **20.16/20.34 ms**、`fills=376/帧`、`frame_count` 逐帧递增、系数回读全对 | R85/R86 |
| 回归 | `pytest test_ortho+test_cosim+test_rtl_contract` → **50 passed**，lint 全树 0 告警 | R87 |
| PS 算系数 | 与 `VR_DBG_HQ` **0 项差异** | R87 |

**产物路径**
- 位流：`data/fpga/pl_bitstream/bd_vision_p0.bit`（md5 `78f63c19…`）/ `.bin`（`f4d97afc…`）；旧的 `bd_vision_r8.bit(.bin)` 留着可回滚。
- P0 量具/驱动内核：`core/fpga/tools/pl_p0_probe.cpp`（`--quad` 算系数、`--frames`、`--dump-frame`、
  每帧等 busy、端到端计时、`row_count` 观测、`P0_COEF_ONLY=1` 只算系数可本机跑）。
- 本地门禁：`core/fpga/cosim/tb_p0.cpp` + `core/fpga/tests/test_ortho.py::test_p0_integration_bit_exact`（两帧参数化）。
- 安全刷板：`core/board/tools/jtag_boot.tcl`（`default` / `rollback` / `<bit路径> 1 0`）。

---

## 3. 只剩这三件（**不需要再重建位流**）

### ① 写 PS 侧 `pl_vision --p0`
把 `core/board/tools/pl_vision.cpp` 里"正射 + PIXPUSH 推送"那段替换成：
```
每帧：memcpy 640×480×3 → udmabuf（/dev/udmabuf0，phys=0x30100000，128 MB）
      写 /sys/class/u-dma-buf/udmabuf0/sync_for_device
      （系数与 FRMBASE 只需开机写一次）→ FRMCTL=src|clr → FRMCTL=src|start
      → 轮询 CTRL.busy 落下（**每帧起跑前先等 busy=0**）→ 读 0x100 窗口的 FLEN 字节
下游（几何帧 → vt.jsonl → board_agent → M1 → @POSE → 标定）**一行不动**。
```
系数与每帧流程**直接照抄** `core/fpga/tools/pl_p0_probe.cpp`（已跑通）；下游解析沿用 `pl_vision.cpp` 现有代码。

### ② ④ 的"相机域 300 帧"
用上面这个驱动跑 300 帧，统计"几何帧 vs C 参考逐字节一致"的帧数（R4 口径）。
**已知偏差**：板端 `row_count[359]`=229 vs 本地 197、`FLEN` 220 vs 292（cam_01 是 243/256），
**随输入变化**（⇒ 不是读陈旧 DRAM，R86 已排除）。若一致率低，先核对"udmabuf 里的相机帧与 PPM 是否逐字节相同"
（两边都能 dump）。

### ③ ⑤ 闭环一轮
```
跑前：cp core/using/calib/calib_y.json data/fpga/calib_y.json ；清掉残留相机管线
CAM_VT_BIN=/root/tools/pl_vision core/board/tools/loop_standard.sh --tag plp0N
判据：到达 + 0 碰撞 + 24/24 + 最小间隙 ≥ +9.92（基线 acc9r16）
```

---

## 4. **不要重复做的事**（本会话曾遇到/验证过，重做纯浪费）

1. **别重建位流做调试**；WNS 富余这版就是唯一基础。要改 RTL 就本地先用 `tb_p0` 验，再一次重建。
2. **不再用 `echo ... > /sys/class/fpga_manager/fpga0/firmware` 热换位流**（曾在 PL 有未完成 AXI 事务时把 PS 弄挂）；
   一律 `xsdb.bat jtag_boot.tcl default 1 0`。
3. **不再怀疑 `hold`**：`-GHOLD_BYPASS=1` 实测与正常完全一致（R80）。
4. **不再给 `out_sof` 改成每行**（R82 的真 bug）。
5. **不再漏 `frm_base`**（R84 的真 bug；本地 tb 已用非零 FRMBASE 覆盖）。
6. **不再手抄正射系数**（用 `--quad` 现算，已自证 0 差异）。
7. **别在 tb 里让 AXI 从机"同拍结算两次"**（R82 的假死锁元凶）；`tb_p0` 已拆成 `slave_comb()` + `tick()` 内结算。
8. **不再用"等 2000 万次 MMIO"当超时**（会量出 5622/6809 ms 假数）；按**时间**设上限（探针已改）。
9. **别重复搭 cosim**：`tb_ortho`（直供）、`tb_ortho_sys`（全链路）、`tb_p0`（集成 + vision_top）、
   `tb_linecache`、`tb_axim`、`tb_divpipe`、`tb_camadapt` 都已存在，`Makefile` 有对应 target。
10. **别改基线逻辑**：`board_agent.py` / `field_m1.hpp` / `@POSE` 链 / `calib_y.json` 一行不动（R1/R2/R3）。

---

## 5. 环境与命令（照抄，省时间）

```bash
# 本地门禁（一定要用这个 venv：里面有 pytest/numpy/cv2）
/tmp/vpy/bin/python -m pytest core/fpga/tests/test_ortho.py core/fpga/tests/test_cosim.py \
    core/fpga/tests/test_rtl_contract.py -q          # 期望 50 passed

# 本地 P0 集成 cosim（cam_00 的页面图 = ortho oracle 的 dump）
cd core/fpga/cosim && ./obj_p0/Vvision_axi_p0 ../../../data/fpga/ortho_stim/cam_00.ppm /tmp/os1/exp_00001.ppm
# 期望：FLEN=292、几何帧逐字节 0/292、PASS、~20.1 ms

# 位流（WSL 里调 Windows 侧 Vivado；只有改 RTL 后才需要）
rsync -a --delete core/fpga/rtl/ /mnt/c/pl_build/rtl/
cmd.exe /c "cd /d C:\pl_build && D:\Vivado\Vivado\2018.3\bin\vivado.bat -mode batch -source C:/pl_build/bd_pack.tcl -tclargs C:/pl_build/rtl C:/pl_build/bd C:/pl_build/out"
cmd.exe /c "... bd_vision.tcl ..."     # 产物是 out/.bit（隐藏文件，$tag 为空），拷出来再 bootgen
cmd.exe /c "... D:\Vivado\SDK\2018.3\bin\bootgen.bat -image bit2bin.bif -arch zynq -process_bitstream bin -w on -o C:\pl_build\out"

# 上板：只重配 PL、不动 PS
cmd.exe /c "... xsdb.bat \\wsl$\Ubuntu\home\zhang\FPGA\core\board\tools\jtag_boot.tcl default 1 0"

# 板端（ssh -i data/fpga/board_ssh/board_key root@192.168.137.242）
#   探针源码：core/fpga/tools/pl_p0_probe.cpp + core/board/vision_regs.hpp
#   板上编译：g++ -O2 -std=c++17 -I<dir> pl_p0_probe.cpp -o /tmp/pl_p0_probe
#   /tmp/pl_p0_probe --quad quad.json --frames 30 --ppm cam.ppm --dump-frame /tmp/f
#   板上没有 devmem；寄存器一律 mmap（0x43C00000）
```

寄存器：`0x900 FRMCTL`（W: bit0 src / bit1 start / bit2 clr-done；R: bit0 src / bit1 **busy** / bit2 **done** / [31:16] fills）、
`0x904 FRMBASE`、`0x908 {c1,c0}`、`0x90C {r1,r0}`、`0x910+4i h0..h8`、`0x934/938 PY0_16/PS_16`、
`0x93C/940 PY0_24 低/[39:32]`、`0x944 PS_24`、`0x948/94C/950/954` 只读调试、`0x0C0 CTRL`、`0x100 几何帧窗口`。
`vision_top` 调试：`0x080` 写 `{sub[3:0], idx[17:0], sel[3:0]}`、`0x084` 读；`sel=15 sub=1/2/3` = state / {wr_row,wr_col} / frame_count。

---

## 6. 文件地图（改哪儿看哪儿）

| 文件 | 作用 |
|---|---|
| `rtl/vision_axi.v` | 顶层：AXI-Lite 寄存器（含 0x900 P0 块）、像素源三选一、正射子系统、camadapt、`vision_top` |
| `rtl/vision_ortho.v` | 正射：行系数 → ISSUE → 2×流水除法 → FETCH/输出（每行 640 像素，`sof` 只在整帧首像素） |
| `rtl/vision_div_pipe.v` | radix-4 流水除法器（II=1、34 位商） |
| `rtl/vision_linecache.v` | 32 行 × 640 像素窗口，填充重排（8 字节→像素）、`fill_ready` 反压 |
| `rtl/vision_linefill.v` | miss→按行 burst 的填充控制（完成只认 `fill_done`） |
| `rtl/vision_axi_master.v` | 极简 AXI4 读主口（`r_en` 反压、`dbg_left/dbg_state`） |
| `rtl/vision_ortho_sys.v` | 上面四块的连线层（`frm_base` 加法在这里） |
| `rtl/vision_camadapt.v` | 相机域适配（40 px 亮段→暖白；短亮斑→210；延迟 41 像素） |
| `rtl/vision_top.v` | 判据流水（band/seg/gap/CCL/pack）+ 调试口 + `wall_mem/bright_mem` 单写口 |
| `cosim/tb_p0.cpp` | **P0 集成门禁**（DDR 模型 + AXI 从机 + 几何帧 vs C 参考） |
| `tools/pl_p0_probe.cpp` | 板端量具 + `pl_vision --p0` 内核（`--quad` / `--dump-frame` / 端到端计时） |
| `tools/camadapt_ref.py` | 适配器的 oracle（与 pl_vision.cpp:403 逐行对应） |
| `synth/bd_pack.tcl` / `bd_vision.tcl` | 打包 IP（含 `m_axi`）/ 块设计 + HP0 + 综合实现出位流 |
| `core/board/tools/jtag_boot.tcl` | 安全刷板（`default` / `rollback` / 只配 PL） |

---

## 7. 新对话的**开场第一步**（建议）

1. 按 §0 读完 4 份文档 + §6 里 2~3 个关键 RTL/tb；
2. 跑一次 §5 的三条本地命令（pytest / tb_p0 / coef-only），确认起点数字与本文一致
   （**不一致就先查环境，别直接改代码**）；
3. 从 §3 的 ①开始写 `pl_vision --p0`；写完先在板上跑 §3 的 ②（300 帧），再跑 ③（闭环）。
