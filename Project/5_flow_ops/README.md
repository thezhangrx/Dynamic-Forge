# 模块 5：全流程编排与运维

## 职责

封装"一轮闭环"的完整编排、板端与主机进程管理、PL 位流的上电自启动与部署、
相机预览，以及运行日志的自动验收。

## 目录结构

| 目录 | 内容 |
|---|---|
| `scripts/` | `loop_standard.sh`（标准一轮，参数集中于此）、`run_flow.sh`（板端一键流程）、`loop_watchdog.sh`、`board_up.sh`（板端上线与状态检查）、`deploy_pl_autoload.sh`（位流部署与校验）、`install_pl_autoload.sh`、`bit2bin.py`（`.bit → .bin` 转换）、`jtag_boot.tcl`（JTAG 兜底）、`flow_key.py`/`flow_keyd.py`/`keyscan.py`（按键启停）、`check_run_logs.py`、`grade_run.py`、`keyboard_drive.py`、`feed_pl.py`/`feed_pl.c` |
| `services/` | 板端 systemd 单元：`dhcp_retry.service`（网络地址重试）、`flow_keyd.service`（按键与指示灯） |

## 接口与约定

| 项目 | 取值 |
|---|---|
| 板端地址 | `root@192.168.137.242`（密钥 `data/fpga/board_ssh/board_key`） |
| 位流自启动路径 | `/lib/firmware/bd_vision.bin`（由 `pl-autoload.service` 上电加载） |
| 运行日志目录 | `data/fpga/loop_logs/run_*/` |

## 运行

```bash
bash 5_flow_ops/scripts/loop_standard.sh --check      # 体检
bash 5_flow_ops/scripts/loop_standard.sh --tag demo   # 跑一轮
bash 5_flow_ops/scripts/board_up.sh --check           # 板端状态
```
