# 虚拟全流程 · 按模块整理的代码档案

> 这份 `Project/` 把**虚拟平台闭环全流程**（PL 几何 → 板端视觉/决策 → @MOVE → 主机虚拟平台渲染 → @POSE）
> 的代码按模块整理成 7 块。**原始工作树在仓库的 `core/`**，本目录是阅读/评审/交付用的副本，
> 文件内容与 `core/` 里同名文件一致（生成后未改动一行）。
>
> ⚠ 目录内的脚本沿用原仓库的相对路径（`core/board/tools/...`），**要跑请回仓库根**：
> `cd /home/zhang/FPGA && bash core/board/tools/loop_standard.sh`。
> `data/`（位流、跑批日志、板上二进制）按 `.gitignore` 不进仓库，也不在本目录。

## 一、数据流（一句话链路）

```
真实相机 /dev/video0
   │ ffmpeg(MJPG→RGB)                       ┐
   ▼                                        │  5_flow_ops：相机管线与编排
板端视觉 vt_sparse / pl_vision ──几何帧──┐   │
   │（墙/缺口/四角，P0 版本由 PL 出帧）    │   │
   ▼                                     │   │
板端决策 board_agent（自建世界 + Dijkstra/势场）│  3_board_decision
   │ @MOVE <act>                          │   │
   ▼                                     │   │
主机虚拟平台 platform_host（渲染 + 推进世界的真值）│  4_virtual_platform
   │ @POSE <seq> <x> <y> <r>              │   │
   ▼                                     │   │
回到板端决策闭环  ◀──────────────────────┘   │
                                             │
P0 硬件通路：PS memcpy → /dev/udmabuf0(0x30100000) → FRMCTL → PL(HP0 读 DDR)
             → 32 行缓存 → 正射(vision_ortho) → 相机适配 → 几何帧写到 0x100
             （1_pl_geometry）
```

## 二、模块

| 目录 | 职责 | 关键入口 |
|---|---|---|
| `1_pl_geometry/` | **PL 几何引擎**：正射/行缓存/除法流水/相机适配/顶层 + C 参考模型 + Verilator 对拍 + 综合打包脚本 + L0 对拍 | `rtl/vision_top.v`、`ref_model/vision_ref.c`、`cosim/tb_p0.cpp`、`synth/bd_vision.tcl` |
| `2_board_vision/` | **板端视觉**：从相机帧解出墙/缺口/四角（`player.hpp/band_gap.hpp/...`）+ 两个可执行视觉前端 + 相机参数与预览工具 | `src/pl_vision.cpp`、`src/vt_sparse.cpp`、`src/player.hpp`、`camera/start_cam.sh` |
| `3_board_decision/` | **板端决策**：`board_agent` 自建世界、暂停采样、一致性、入表、Dijkstra/势场、`@MOVE` 输出 | `agent/board_agent.py`、`agent/board_world.py`、`planner/`、`headers/field_m1.hpp` |
| `4_virtual_platform/` | **主机虚拟平台**：渲染页面、推进真值、收发 `@MOVE`/`@POSE`、页面/观众工具、无角色页面、模拟器内核 | `host/platform_host.py`、`host/full_loop.py`、`host/page_plain.py`、`sim/`、`scenes/` |
| `5_flow_ops/` | **全流程编排与运维**：标准一轮、板端一键流程、位流上电自启动、JTAG 兜底、按键/启停、日志验收 | `scripts/loop_standard.sh`、`scripts/board_up.sh`、`scripts/deploy_pl_autoload.sh`、`services/` |
| `6_tests/` | **测试与验收**：板端/CPU/FPGA/小车固件的单元测试 | `board/test_*.py`、`cpu/test_*.py` |
| `7_docs/` | **文档**：PL 交接与 P0 偏移调查、真实小车 TODO、闭环标准配置、无角色平台要点 | `pl/PL_P0_QUAD_OFFSET_FINDING.md`、`closed_loop/闭环标准配置.md` |

## 三、怎么跑（回仓库根执行）

```bash
cd /home/zhang/FPGA

# 1) 体检（不动相机、不开跑）
bash core/board/tools/loop_standard.sh --check

# 2) 标准闭环一轮（相机 + 板端 + 虚拟平台）
bash core/board/tools/loop_standard.sh --tag demo

# 3) 只看"无角色的平台页面"（量投影/对相机用；Ctrl-C 退出）
Env/bin/python core/board/tools/page_plain.py

# 4) 板端指令分方向转发（前后→平台视角，左右→小车；--no-car 只打印）
Env/bin/python core/board/tools/plain_car_host.py --board-ssh root@192.168.137.242 --no-car
```

## 四、端口与路径约定

| 项 | 值 |
|---|---|
| 板子 | `root@192.168.137.242`（key：`data/fpga/board_ssh/board_key`） |
| 页面 MJPEG | `8491`（page_plain / page_display），`plain_car_host` 用 `8493` |
| 主机 ↔ 板 TCP | `9000`（platform_host 监听，板端连） |
| 相机预览 | `8099`（板端 `mjpeg_server.py`，独占 `/dev/video0`） |
| PL 几何帧 | DDR 起始 `0x30100000`，几何帧写入 `0x100`，长度寄存器 `FLEN(0x0D0)` |
| PL 寄存器基址 | `0x43C00000`（只读诊断 `0x948/0x94C/0x950/0x954/0x958`） |

## 五、已知限制（照实写，别信"已经好了"）

1. **P0 板端几何有 +37.5 列的偏移**（仿真一致、硅上偏移），当前用 `pl_vision --x-shift-cols 37` 补偿；
   根因未修（详见 `7_docs/pl/PL_P0_QUAD_OFFSET_FINDING.md`）。
2. **视觉认不出真车**：`player` 检测要"亮盘 + 契约面积"（期望半径 ≈10 单位 ≈2.8 cm），
   24 cm 深色小车面积差 ~17 倍，实测 0/158 帧；位置改用编码器里程计（`@ODOM`）。
3. **小车里程计未标定** + 电机/编码器极性不一致（`@VEL 100 0` 实测左 −206 / 右 +194 mm）⇒
   `@ODOM` 的 x 暂不可信。
4. **冷上电要拔 JTAG**（SMT2 倒灌拖歪 POR）；PL 位流由 `pl-autoload.service` 上电自配。

## 六、文件清单

`00_文件清单.txt` 是本目录 371 个文件的完整相对路径清单（按模块排序）。
