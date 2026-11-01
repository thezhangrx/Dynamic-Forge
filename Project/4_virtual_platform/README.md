# 模块 4：主机虚拟平台

## 职责

在主机上渲染虚拟场地并推进真值：接收板端 `@MOVE` 动作、推进角色与场景状态、
回送 `@POSE` 位姿；同时提供页面显示、无角色页面、投影尺寸测量等辅助工具。

## 目录结构

| 目录 | 内容 |
|---|---|
| `host/` | `platform_host.py`（主循环与链路）、`full_loop.py`（帧服务与页面推送）、`field_loop.py`、`page_plain.py`（无角色平台页面）、`page_view.py`/`page_display.py`（页面显示）、`plain_car_host.py`（指令分方向转发与里程计位姿）、`static_page_server.py`、`auto_quad.py`（页面四角检测）、渲染与回放工具 |
| `sim/` | `bullet_sim` 仿真内核：场景规格、环境推进、实体、渲染（`render/pygame_view.py`） |
| `scenes/` | 场景 JSON（墙体布置、场地尺寸、角色参数） |

## 接口

- 输入：`@MOVE <act>`（来自板端，TCP `9000` 或 ssh stdio）。
- 输出：`@POSE <seq> <x> <y> <r>`；页面视频流（MJPEG，默认 `8491`）。
- 页面坐标：640×480，y 向下；场地尺寸由场景文件给出（标准场景 640 × 3420 单位）。

## 运行

```bash
# 标准闭环（含板端）
bash core/board/tools/loop_standard.sh --tag demo

# 仅页面（无角色，用于投影测量）
Env/bin/python core/board/tools/page_plain.py
```
