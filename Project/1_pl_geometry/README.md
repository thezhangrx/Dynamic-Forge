# 模块 1：PL 几何引擎

## 职责

在 FPGA 内完成由 DDR 图像帧到场地几何帧的变换：按行读取 DDR、32 行行缓存、
正射变换（含 radix-4 除法流水）、相机域适配、几何帧组帧并按 AXI-Lite 寄存器对外暴露。

## 目录结构

| 目录 | 内容 |
|---|---|
| `rtl/` | Verilog 源文件：`vision_top.v`、`vision_ortho.v`、`vision_linecache.v`、`vision_div_pipe.v`、`vision_camadapt.v`、`vision_pixfeed.v`、`vision_axi.v`、`vision_axi_master.v` 等 |
| `rtl_bd/` | 块设计封装（AXI-Lite 寄存器接口） |
| `ref_model/` | 定点参考模型（C 语言），与 RTL 逐位对齐 |
| `cosim/` | Verilator 协同仿真测试台与 Makefile |
| `synth/` | Vivado 块设计与 IP 打包脚本（`bd_pack.tcl`、`bd_vision.tcl`） |
| `tools/` | L0 全量对拍、正射参考实现、寄存器探针等工具 |
| `tests/` | 正射/C 参考一致性测试 |

## 接口

- 输入：DDR 中的 640×480 RGB 帧（基址由 `FRMBASE` 寄存器给出）；PS 写 `FRMCTL` 触发。
- 输出：几何帧（魔数 `0x31564842`、Q6 定点、含 CRC32）写入帧缓冲 `0x100`，长度由 `FLEN(0x0D0)` 给出。
- 诊断：`0x948`–`0x958` 只读寄存器。

## 构建与验证

1. 协同仿真：`cd cosim && make`（需 Verilator 5.x），产物为各 `tb_*` 可执行文件；
2. 语法与位宽检查：`make -C cosim lint`；
3. 位流：`synth/bd_pack.tcl` → `synth/bd_vision.tcl` → `bootgen`（或使用 `5_flow_ops/scripts/bit2bin.py` 完成 `.bit → .bin` 转换）。
