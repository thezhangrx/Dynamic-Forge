# 硬规则（任何改动都必须遵守，不许绕过）

> 这些是用户明确要求、并已在代码里做了强制校验的约束。改代码前先读这一页。

## R1. 板端决策的输入只能是真实相机

**禁止**把 PC 渲染出来的画面（任何形式：渲染帧、回放视频、录制的画面序列）
送进板端的视觉/决策链路。板子必须自己从相机看到平台。

允许的唯一链路：

```
板子相机 /dev/video0 → ffmpeg 解码/降采样 → 板端 vt_board（白框正射 + 找墙/缺口 + 角色）
   → board_agent（暂停采样、一致性、入表）→ field_cli_board（Dijkstra）
   → @MOVE → 主机（只渲染虚拟平台 + 收动作 + 发位姿 @POSE）
```

代码强制点：

- `core/board/tools/platform_host.py`
  - `--vt-feed` 默认被拒绝：必须显式加 `--i-know-this-is-not-the-camera` 才允许，
    且 summary 里固定写 `decision_source: "synthetic(PC渲染帧)"`，**不能当作验收结果**。
  - `--start-camera-pipe`（默认开）由主机在板子上拉起相机管线，并写
    `/tmp/vt_source=camera`。
- `core/board/tools/board_agent.py`
  - 启动时校验 `--source-file /tmp/vt_source`，值不是 `camera` 直接退出（返回码 3），
    除非显式 `--allow-synthetic-vt`（仅算法单测）。

推论：任何“板子跑得很好”的结论，必须来自相机链路；用合成帧得到的数字只能说明
算法本身没错，不能代表闭环可用。

## R2. 真值（ground truth）只留在 PC 上

- 真值来源：`configs/*.json` 的 `meta`（`walls_y / gaps_x / gap_widths / gap_sides`）
  与仿真器 `env.world`。
- PC 使用真值的范围：渲染页面、算指标（`min_clearance`、`Δy`、覆盖率）、画热力图/轨迹。
- **不许**把真值通过 `@STATE` 或任何通道送给板子（早期 `--send-all-walls` 已删除）。

## R3. 动作用指令，不用真值

板子只输出 `@MOVE <act>`（`0`停、`1`-x、`2`+x、`3`+y、`4`-y、`5..8` 对角）；
主机收到后推进虚拟平台并回 `@POSE`。板子永远不知道目标点真值坐标以外的任何信息
（目标点由参数给出，属于任务设定，不是墙体真值）。

## R4. 证据日志是改动的前提

每次改动前后都要用 `core/board/tools/check_run_logs.py` 对 `run_*/` 目录出报告，
用数字证明有进步（无进步就回退）。不允许“感觉好点了”式改动。

## R5. 环境纪律

- 不用 `kill -9`；批量清进程时不要在同一条命令里写会匹配到自己的 pattern
  （`pkill -f` / `grep "[x]xx"` 都会打到自己的 bash）。
- 采样相机快照必须同时取 mtime，过期（>5 s）的快照不能当“当前画面”用。
