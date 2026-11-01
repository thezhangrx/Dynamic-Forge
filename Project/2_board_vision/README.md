# 模块 2：板端视觉

## 职责

在板端 Zynq 的 ARM 侧，将相机帧（或 PL 产出的几何帧）解算为可决策的几何信息：
页面四角、墙体位置与缺口；同时提供相机参数设置与预览工具。

## 目录结构

| 目录 | 内容 |
|---|---|
| `src/` | 视觉算法头文件与可执行前端：`player.hpp`（角色检测）、`band_gap.hpp`（墙体/缺口带检测）、`contour.hpp`、`fiducial_zone.hpp`（四角标记）、`rectify`/`field_frame` 相关；`pl_vision.cpp`（P0 路径：经 PL 取几何帧）、`vt_sparse.cpp`（CPU 路径：直接处理相机帧）；诊断探针 `player_probe.cpp`、`band_gap_probe.cpp` |
| `camera/` | 相机参数锁定（`cam_settings_locked.sh`）、管线启动（`start_cam.sh`）、预览/曝光/对焦/健康检查工具 |

## 接口

- 输入：`/dev/video0`（MJPG 640×480），或 PL 几何帧（`0x100`，长度见 `FLEN`）。
- 输出：JSONL 逐帧记录（`frame`、`quad_ext`、`walls[]`、`player`、`n`），默认写入 `/tmp/vt.jsonl`；
  供板端决策模块读取。

## 运行

```bash
# 板端：启动相机管线（640×480，12 fps）
CAM_W=640 CAM_H=480 CAM_FPS=12 VT_BIN=/root/tools/vt_sparse \
  sh /root/tools/start_cam.sh /root/vision_state/quad.json
```

相机参数以 `camera/cam_settings_locked.sh` 为准；预览服务占用 `/dev/video0` 时须先停止。
