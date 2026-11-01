# 虚拟平台闭环全流程 · 代码模块化整理

## 1. 概述

本目录将虚拟平台闭环全流程的源代码按功能划分为 8 个模块，共 406 个文件、约 6.1 MB。
所有文件取自仓库工作树（`core/` 等目录），逻辑与对应源文件一致，未作功能修改；
文档（`*.md`）的措辞按技术报告口径作了统一整理。

运行产物（PL 位流、采集日志、板端二进制、相机录像等）按仓库 `.gitignore` 的约定不纳入
版本管理，因此不在本目录内；相关产物存放于仓库 `data/` 目录。

## 2. 系统链路

```
相机 /dev/video0
    │  ffmpeg: MJPEG → RGB24
    ▼
板端视觉（pl_vision / vt_sparse）── 几何帧（墙体/缺口/页面四角）
    │
    ▼
板端决策（board_agent：世界模型 + 路径规划）
    │  @MOVE <act>
    ▼
主机虚拟平台（platform_host：场景渲染 + 真值推进）
    │  @POSE <seq> <x> <y> <r>
    ▼
板端决策（闭环）

PL 数据通路：PS memcpy → /dev/udmabuf0（物理地址 0x30100000）→ FRMCTL 寄存器
             → PL 经 AXI HP0 读取 DDR → 32 行行缓存 → 正射变换（vision_ortho）
             → 相机域适配 → 几何帧写入 0x100，长度见 FLEN(0x0D0)
```

## 3. 模块构成

| 模块 | 职责 | 主要入口文件 |
|---|---|---|
| `1_pl_geometry` | PL 几何引擎：正射变换、行缓存、除法流水、相机域适配、顶层集成；C 参考模型与 Verilator 协同仿真；综合与 IP 打包脚本 | `rtl/vision_top.v`、`ref_model/vision_ref.c`、`cosim/tb_p0.cpp`、`synth/bd_vision.tcl` |
| `2_board_vision` | 板端视觉：由相机帧解算墙体、缺口与页面四角；视觉算法头文件；相机参数与预览工具 | `src/pl_vision.cpp`、`src/vt_sparse.cpp`、`src/player.hpp`、`camera/start_cam.sh` |
| `3_board_decision` | 板端决策：世界模型构建、采样与一致性检查、墙体入表、路径规划（Dijkstra/势场）、`@MOVE` 输出 | `agent/board_agent.py`、`agent/board_world.py`、`planner/`、`headers/field_m1.hpp` |
| `4_virtual_platform` | 主机虚拟平台：场景渲染、真值推进、`@MOVE`/`@POSE` 收发、页面显示与无角色页面工具、仿真内核 | `host/platform_host.py`、`host/full_loop.py`、`host/page_plain.py`、`sim/`、`scenes/` |
| `5_flow_ops` | 全流程编排与运维：标准运行脚本、板端一键流程、位流上电自启动、JTAG 兜底、相机预览、日志验收 | `scripts/loop_standard.sh`、`scripts/board_up.sh`、`scripts/deploy_pl_autoload.sh` |
| `6_tests` | 单元测试：板端、CPU、FPGA 侧 | `board/`、`cpu/` |
| `7_docs` | 文档：PL 交接与偏移调查、真实小车迁移说明、闭环标准配置 | `pl/PL_P0_QUAD_OFFSET_FINDING.md`、`closed_loop/闭环标准配置.md` |
| `8_car_firmware` | 小车固件（STM32F103）：平衡车底盘移植、速度/转向环、差速模式、串口协议、编码器里程计；构建与烧写工具 | `src/main.c`、`src/car_control.c`、`src/car_proto.c`、`src/car_kin.c` |

各模块的职责、目录结构与接口说明见该模块目录下的 `README.md`。

## 4. 运行方式

以下命令在仓库根目录执行。本目录为代码整理副本，脚本内的相对路径指向仓库工作树
（`core/board/tools/...`），直接在本目录内执行脚本将无法定位依赖文件。

```bash
cd /home/zhang/FPGA

# 4.1 环境体检（不启动相机与闭环）
bash core/board/tools/loop_standard.sh --check

# 4.2 标准闭环运行一轮（相机 + 板端 + 虚拟平台）
bash core/board/tools/loop_standard.sh --tag demo

# 4.3 仅显示无角色平台页面（用于投影尺寸测量与相机对位）
Env/bin/python core/board/tools/page_plain.py

# 4.4 板端指令分方向转发（纵向指令驱动平台视角，横向指令驱动小车）
Env/bin/python core/board/tools/plain_car_host.py --board-ssh root@192.168.137.242 --no-car

# 4.5 PL 位流部署（转换为板端可加载格式并写入上电自启动路径）
bash core/board/tools/deploy_pl_autoload.sh data/fpga/pl_bitstream/bd_vision_p0.bit
```

## 5. 接口与端口约定

| 项目 | 取值 |
|---|---|
| 板端主机名 | `root@192.168.137.242`（密钥 `data/fpga/board_ssh/board_key`） |
| 页面 MJPEG 端口 | `8491`（page_plain / page_display）、`8493`（plain_car_host） |
| 主机与板端 TCP 端口 | `9000`（platform_host 监听，板端主动连接） |
| 相机预览端口 | `8099`（板端 `mjpeg_server.py`，独占 `/dev/video0`） |
| PL 帧缓冲 | DDR 起始 `0x30100000`；几何帧写于 `0x100`；帧长度见 `FLEN(0x0D0)` |
| PL 寄存器基址 | `0x43C00000`；只读诊断寄存器 `0x948/0x94C/0x950/0x954/0x958` |

## 6. 文件清单

`00_文件清单.txt` 列出本目录全部文件的相对路径，按模块分组排序。
