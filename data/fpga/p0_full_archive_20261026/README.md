# PL P0 全流程存档（2026-10-26）

一句话：**感知链（相机→P0 正射→几何帧→agent→决策）已跑通并出过一轮闭环；判据未达标**
（到达 ✅ / 23-24 / 1 碰撞 / 最小间隙 −10.00 / |Δy| 3.9）。
硅上存在**固定 +37 列的横向偏移**，已用可关闭的运行态补偿 `--x-shift-cols 37` 抵消，
**根因未修复**（收窄到“正射输出 → vision_top 写帧缓冲”之间，需第三次重建确认）。

## 1. 唯一一轮闭环

`run/`（= `data/fpga/loop_logs/run_20261026_184929_p0i` 的关键文件）：

| 判据 | 结果 |
|---|---|
| 到达 | ✅ |
| 碰撞 | ❌ 1 次（仿真计数 0，按"最小间隙<0"口径算 1） |
| 最小间隙 | ❌ −10.00 |
| 墙覆盖 | ❌ 23/24 |
| \|Δy\| | ✅ 3.9 px（6284 行样本） |

## 2. 位流链（`bitstream/`）

| 文件 | md5 | 说明 |
|---|---|---|
| `bd_vision_p0_PRE_fix_78f63c19.bit` | `78f63c1993724759dd22fe2e24e15dc2` | 修复前正式版 |
| `bd_vision_p0_DIAG_0x958.bit` | `836b6d4649c7f1c67a0a50c46f9e1dbc` | +0x958 只读诊断寄存器 |
| `bd_vision_p0_FIX_every-frame-sof.bit` | `d974d009d53264b78a752decfa240090` | + camadapt 每帧 sof（**板上当前**） |

烧写方式：`xsdb.bat ...\jtag_boot.tcl default 1 0`（`DO_RST=0`，不 hot-swap）。

## 3. 板端二进制（`board_bin/`）

| 文件 | md5 | 说明 |
|---|---|---|
| `pl_vision` | `8c52d9f8fd623876c29b6168d6809bdc` | P0 感知（含 `--dbg-frm`、`--x-shift-cols`） |
| `board_agent.py` | `93b48778fae298b560bd98bd478eaf90` | 基线逻辑，**未改** |
| `field_cli_board` | `998553dc65f62e7274d08ba53c37acec` | 基线逻辑，**未改** |
| `vt_sparse` | `4565182915a9a7b35b17d66fd1358278` | 对照用（基线感知源） |
| `board_env.txt` | — | `uname` / fpga state / 相机全部控件 / udmabuf phys / autoload 位流 md5 |

板端环境：Linux `U.FMSH 5.4.52` aarch64，`fpga0=operating`，udmabuf0 `phys=0x30100000`。
相机（全部为文档标准档）：`exposure_auto=3`、`gain=4`、`sharpness=2`、`wb_auto=1`、`wb=4000`、
`brightness=70`、`saturation=16`、`gamma=120`、`contrast=32`、`plc=1`。

## 4. 配置（`config/`）

| 文件 | md5 | 说明 |
|---|---|---|
| `quad.json` | `cb7c863a56cebef5eca9a3bc41b7ba5c` | 本轮现量（4 标记，rms 5.77 px） |
| `calib_y_locked.json` | `e187251da806a4b4318475931b6f6004` | 本轮**起始**用的锁定值 `0.85973/39.868` |
| `calib_y_locked_ref.json` | `c5b74b3fa6eb61b47ed3711b515ebf45` | 仓库锁定值原件（同上来源） |
| `scenario.json` | `0fcaf83861ddba9ce133c26fe2865bf7` | `20_walls_alt_24_dy140_gap8` |
| `task.json` | `c2f419939e6746d51eb9c052130ab5d4` | 推给板子的任务文件（**不含墙**） |
| `loop_standard.sh` | `9019c996f8a30a9e12cef0921b65ba78` | 标准配置（现含 `VT_EXTRA` 透传） |
| `page_display.py` | `7a8e0d66bf80f9621d2996cb7fd178af` | 诊断页（`--content 1440x1080 --margin 0 --fiducial-side 48`） |
| `pl_vision.cpp` | `b7ee3449bf3adfde0318031d563b3775` | P0 感知源 |
| `vision_regs.hpp` | `46c72fdd876c45140fd588395fde2169` | 寄存器契约（含 0x958） |

**有效取景/参数**：`--viewer-size 1440x1080 --viewer-margin 0 --fiducial-side 48 --fiducial-anchor band
--frame-px 2 --field-every 5 --cam-fps 12 --cam-w 640 --cam-h 480 --speed 0.5 --consensus-frames 8
--tol-vy 16 --settle 1.0 --pause-timeout 3.5`，感知源 `CAM_VT_BIN=/root/tools/pl_vision`，
附加 `VT_EXTRA='--x-shift-cols 37'`。

## 5. 源码（`rtl/`）

`vision_camadapt.v`（`b3abb748…`，每帧 sof 修法）、`vision_axi.v`（`ddaed36e…`，0x958 诊断）、
`vision_top.v`、`vision_ortho.v`、`tb_p0.cpp`（`--rd-delay`）、`tb_axi.cpp`（地址表）。

## 6. 更早的取证包

* `data/fpga/pl_p0_fix_20261026/`：+37 列取证（相机帧、oracle 页、两侧几何帧、探针 jsonl）
* `data/fpga/quad_check_20261026_r3/`：相机取景/标记/曝光自检
* `core/fpga/docs/PL_P0_QUAD_OFFSET_FINDING.md`：完整排查记录（§1–§13，含全部排除项）
* `core/board/docs/REAL_CAR_TODO.md`：投影平台 + 实物小车迁移待办
