# `core/using/` —— **acc9 全流程快照**（跑出 `acc9r3` 的那一套，原件不动）

> 口径（用户 2026-10-24）：把 **acc9 那一版全流程**放进这里并进 git，**不修改、保存原样**。
> 这里全是**拷贝**：改代码去改原件，改完用 `bash core/using/sync.sh` 同步、`--check` 对账。
>
> 快照时间：**2026-10-24 11:00 那一轮 `run_20261024_110028_acc9r3`**（本目录就是产出它的文件）。

---

## 0. 这一版是什么 / 不是什么

**是**：acc9（git `0806d81`「精度落地: |Δy| 3.1→1.4px、碰撞0、最小间隙+17.4」）那套
"板端视觉 + 板端决策 + 主机仿真渲染"的闭环，回退后在本机真实相机上又跑了一遍，
拿到：**到达、0 碰撞、最小间隙 +12.63、VT 覆盖 24/24、|Δy| 5.5 px**。

**不是**：不是加码带（A 路）/ GPS（G1）/ 新鲜度闸门那一套。那些代码**不在**本目录里。

⚠ **位姿来源**：这一版主机每轮都发 `@POSE x y r`（实测跑了 3600 条），板子直接把它当里程计
—— 也就是**"我在哪"是主机给的真值**，板子不自己积分步数。这与后来定的 R2（位姿必须板子
自己按 `@STEP` 步数积分）**相反**；墙/缺口仍然**只**来自板子自己的相机（这条没破）。
要合 R2 就得把这层换掉，那就不是 acc9 了。

---

## 1. 一条闭环是谁

```
主机 bullet_sim（20_walls_alt_24_dy140，跟随视口 640×480 渲染页面）
  └ page_view 把页面全屏显示到屏幕上（相机就对着这块）
       └ 板子 /dev/video0 ──ffmpeg 640×480@12fps──> vt_sparse（白框定四边形→墙行/缺口/角色）
            └ /tmp/vt.jsonl ──> board_agent（暂停采样·多帧一致·入表）
                 └ @STATE ──> field_cli_board（M1 反 Dijkstra 代价场）
                      └ @MOVE ──> 主机推进虚拟平台 + 回 @POSE（本版确实发位姿）
```

## 2. 文件 → 角色 / 来源

### `host/`（主机侧）

| 文件 | 角色 | 来源 |
|---|---|---|
| `platform_host.py` | 闭环主机：跑仿真、渲染页面、拉板端相机管线、收 `@MOVE`、发 `@POSE`、出指标与录像 | `0806d81` + **本机补的相机设备自动探测**（`/dev/videoX` 拔插后会变号） |
| `page_view.py` | 把页面 MJPEG 全屏显示到屏幕 | 工作树（含 `--present-udp`，本版未启用） |
| `mjpeg_client.py` | 最小 MJPEG 客户端 | 工作树 |
| `full_loop.py` | `FrameServer` / `PagePublisher` | 工作树 |
| `run_log.py` | run 日志、真值行、虚拟真值 | `0806d81` 期 |
| `field_loop.py` | `dist_rect` / `walls_of` 等几何工具 | 工作树 |
| `calib_y.py` | y 换算系数拟合（本轮结束由 `platform_host` 调用重标） | 工作树 |
| `loop_standard.sh` | **标准入口**：体检 → 推 quad/标定 → 开跑 → 自动验收 | 工作树（参数已按 acc9 写死） |
| `grade_run.py` | 验收：到达 / 碰撞 / 最小间隙 / 覆盖率 / \|Δy\| | 工作树 |
| `check_run_logs.py` | 逐 run 数字报告（R4） | 工作树 |
| `audit_crossings.py` | 逐墙审计：凭信息过 vs 侥幸过 | 工作树 |
| `make_task.py` | 生成板上 `task.json`（本版板端**不读**它） | 工作树 |

### `board/`（板端）

| 文件 | 角色 | 来源 |
|---|---|---|
| `vt_sparse.cpp` | 板端视觉前端：白框→单应→墙行亚像素质心/缺口/角色 → `/tmp/vt.jsonl` | **`0806d81`**（无码带解码） |
| `board_agent.py` | 板端决策外壳：暂停测量、8 帧共识、入表、`@STATE` 喂 M1、`@MOVE` | **`0806d81`** |
| `start_cam.sh` | 板端相机管线脚本（本版由 `platform_host` 内建管线取代，留作单跑用） | **`0806d81`** |
| `field_decision_cli.cpp` | M1 的 CLI 外壳（`@STATE` → `@MOVE`） | 工作树 |
| `field_m1.hpp` | M1 决策内核（10 单位格、反 Dijkstra、8 邻下降、EMA、迟滞） | 工作树 |

### `calib/`、`configs/`、包

| 文件 | 说明 |
|---|---|
| `calib/quad.json` | 页面四角（本机当前机位）：`54.28 55.51 556.05 38.34 588.10 422.37 46.38 447.42` |
| `calib/calib_y.json` | **本轮跑完由 `platform_host` 自动重写的那份**（`scale 0.93153 bias 7.489 rmse 8.26`）。注意它每轮都会被重写；开跑前 `loop_standard.sh` 会把 `data/fpga/calib_y.json` 推到板上 `/tmp/calib_y.json` |
| `configs/20_walls_alt_24_dy140.json` | acc9 用过的场景（24 道墙 y=80…3300，缺口宽 ~90–110） |
| `bullet_sim/` | 仿真与渲染整包拷贝（含 `render/pygame_view.py`，本版**不画码带**） |
| `standard/` | 共享契约包（`fiducial.py` 等；`tagstrip.py` 仅随包存在，本版不调用） |

## 3. 怎么复现这一轮

```bash
# 1) 板端二进制（板子是 aarch64，有 g++ 9.4）
scp -i data/fpga/board_ssh/board_key core/using/board/vt_sparse.cpp \
    root@192.168.137.242:/root/board_src/vt/vt_sparse.cpp
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'cd /root/board_src/vt && g++ -O2 -std=c++17 -I. vt_sparse.cpp -o /root/tools/vt_sparse'
scp -i data/fpga/board_ssh/board_key core/using/board/board_agent.py \
    root@192.168.137.242:/root/tools/board_agent.py

# 2) 主机开跑（参数全在脚本里，别手敲）
DISPLAY=:0 bash core/board/tools/loop_standard.sh --tag acc9r3
```

开场前必须确认屏幕干净：**没有残留的 `platform_host` / `page_display` 窗口**压在页面上。
2026-10-24 踩过：一个 4 小时前开的 `page_display --y-speed 40` 残留窗口把相机抢走了，
页面一直在滚，被误判成"平台一开就跳到中途"。

## 4. 这一轮的实测数字（验收）

| 项 | 值 |
|---|---|
| 到达 | ✅（终点圆 r=24，最后位置距终点 23.5） |
| 碰撞 | ✅ 0 次 |
| 最小间隙 | ✅ +12.63（`(315.6, 1780.0)`） |
| VT 覆盖率 | ✅ 24/24（口径见下） |
| \|Δy\| | ❌ 5.5 px（5911 行样本） |
| 板子**入表**墙数 | **20/24**（缺 y=1760 / 2320 / 3020 / 3160） |
| 入表 y 误差 | **−4.5 单位（常数）** |
| 耗时 | `t_sim 92.5 s`（墙钟 185 s），动作 883 次 |

两个必须记住的口径坑：

1. `grade_run.py` 的"覆盖率"用的是 **`board_vt.jsonl` 每帧原始墙行**：某道真值墙只要被
   **拍到过一次**（15 单位内）就计 1 分 → 它量的是"**VT 看见过**"，**不是**"板子入表了"。
   所以"24/24"和"入表 20 道"可以同时成立。
2. 0 碰撞有几何运气成分：全程最贴近墙的时刻（+12.63）恰好发生在**没入表**的 y=1760 旁边。

## 5. 对账

```bash
bash core/using/sync.sh --check     # 逐字节核对：using/ 与原件是否一致
bash core/using/sync.sh             # 原件 → using/（只在你说"同步"时才跑）
```
