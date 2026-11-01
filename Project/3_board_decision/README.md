# 模块 3：板端决策

## 职责

依据视觉输出构建"板端认为的世界"，完成墙体入表与冻结、位姿跟踪、路径规划，
并向主机输出动作指令 `@MOVE`。

## 目录结构

| 目录 | 内容 |
|---|---|
| `agent/` | `board_agent.py`（主控与 stdio 协议）、`board_world.py`（世界模型，含玩家状态与墙体队列）、`agent_bridge.py`（键盘/小车桥接）、`make_task.py`（生成任务文件）、`smoke_agent.py`（进程级冒烟测试）、`pf_sim.py` |
| `cli/` | 决策命令行工具源码（`field_decision_cli.cpp`、`field_cli.cpp`、`field_cli_board.cpp`） |
| `planner/` | 势场与 Dijkstra 规划实现（C++） |
| `headers/` | `field_m1.hpp`（代价场与斥力）、`virt_truth.hpp`、CPU 侧公共头文件 |

## 接口

- 输入：`/tmp/vt.jsonl`（视觉逐帧记录）；`@POSE <seq> <x> <y> <r>`（主机下发的位姿）。
- 输出：`@MOVE <act>`（0 停、1 −x、2 +x、3 +y、4 −y、5–8 对角），标准输出供主机读取。

## 运行

主机侧通过 `platform_host.py --board-ssh root@192.168.137.242` 以 ssh 启动
`/root/tools/board_agent.py --stdio`，无需在板端单独执行。
