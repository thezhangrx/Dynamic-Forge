# 🔒 锁定配置 —— `acc9r16`（2026-10-24，用户确认"目前最好的一轮"）

> 这份锁定说明**写在 `core/using/` 里**：`core/using/` 就是这一轮实际在跑的那套文件的**逐字节拷贝**
> （`bash core/using/sync.sh --check` 可自证）。
>
> **不要改这里**。要改就改原件、跑一轮、用数字证明更好，再同步过来并更新本文件。

---

## 1. 这一轮的数字（唯一的验收依据）

轮次：`data/fpga/loop_logs/run_20261024_181610_acc9r16`

| 项 | 值 | 判 |
|---|---|---|
| 到达 | ✅ 最终 `(342.4, 3351.6)` | PASS |
| 碰撞 | ✅ **0 次** | PASS |
| 最小间隙 | ✅ **+9.92**（`(517.2, 776.8)`，t_sim 20.1） | PASS |
| 墙覆盖率 | ✅ **24/24** | PASS |
| \|Δy\| | ❌ 6.8 px（3602 行样本） | FAIL（唯一不达标项） |
| 盲穿 / 凭信息 | 4 / 20 | — |
| 仿真时长 | `t_sim 71.1 s`，动作 1561 次 | — |
| 判定 | `grade_run.py` → FAIL（只差 \|Δy\|） | — |

**注意**：`|Δy|` 的来源是**标定常数偏置**（本轮 20 道墙的板端 y 比真值系统性低 ~15.6 单位，无一例外），
不是随机误差、也不是泄漏真值（真值会给精确的 220/360/500；板端给的是 208/346/486）。

## 2. 锁定的场景

| 文件 | 说明 |
|---|---|
| `configs/20_walls_alt_24_dy140_gap8.json` | **本轮用的场景**。在 `_dy140` 基础上，把索引 2/5/8/11/14/17/20/23 这 8 道墙的缺口中心改成中等幅度 **165…492**（原始 199…449；过大 79…568 会认不出），随机种子 **20261024** |
| `configs/20_walls_alt_24_dy140.json` | 原始 acc9 场景，**保持不动**（历史轮次复现用） |

`loop_standard.sh` 的 `SCENARIO` 默认值已指向 `_gap8.json`。

## 3. 锁定的参数

```
场景      20_walls_alt_24_dy140_gap8（24 道墙 y=80…3300，缺口宽 91–110）
速度      0.5×（1 s 仿真 = 2 s 墙钟），max-seconds 220
相机      ffmpeg MJPG 640×480（设备只支持 60fps，滤镜 fps=12 降帧）
          ⚠ YUYV 640×480@30 = 18.4 MB/s，这台板子 dwc2 撑不住（会静默断流）→ 不要换
VT        vt_sparse --w 640 --h 480 --quad-file /tmp/quad.json
quad      38.28 60.05 511.86 43.35 547.97 404.16 24.18 419.71   ← 必须与相机机位一致
板端      --from-end --verbose --consensus-frames 8 --tol-vy 16 --settle 1.0 --pause-timeout 3.5
          x_bias = 0（试过 +17：x 方向 81 s 内反转 53 次、卡死在 y=2054 → 已回退）
标定      本轮开始用的 /tmp/calib_y.json = scale 0.86117 / bias 41.07
          （跑完 platform_host 会自动重拟合，属正常；当前 data/fpga/calib_y.json 是重拟合产物）
```

## 4. 锁定的相机参数

`board/cam_settings_locked.sh`（板子上 `bash cam_settings_locked.sh`）：

```
saturation=16   brightness=70   gain=4   power_line_frequency=1
其余保持驱动默认：gamma=120  contrast=32  白平衡=自动  曝光=自动(exposure_auto=3)
```

⚠ 相机参数**是易失的**：Windows 一侧重新枚举（拔插/休眠/它自己的相机服务）会把参数打回默认。
跑之前建议先回读一次（`v4l2-ctl --get-ctrl=...`），不对就重新执行上面的脚本。

## 5. 重启后要恢复的东西（板上）

| 文件 | 板上位置 | 说明 |
|---|---|---|
| quad | `/tmp/quad.json` + `/root/vision_state/quad.json` | 两个都要；`loop_standard.sh` 每轮会重推 |
| calib | `/tmp/calib_y.json` | `loop_standard.sh` 每轮重推 `data/fpga/calib_y.json` |
| 相机 | — | 执行 `cam_settings_locked.sh` |
| 任务 | `/root/vision_state/task.json` | `make_task.py` 生成（**不含任何墙字段**） |

## 6. 复现这一轮

```bash
# 板上（二进制已在位；如需重编）
scp -i data/fpga/board_ssh/board_key core/using/board/vt_sparse.cpp \
    root@192.168.137.242:/root/board_src/vt/vt_sparse.cpp
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'cd /root/board_src/vt && g++ -O2 -std=c++17 -I. vt_sparse.cpp -o /root/tools/vt_sparse'
scp -i data/fpga/board_ssh/board_key core/using/board/board_agent.py \
    root@192.168.137.242:/root/tools/board_agent.py
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 \
    'bash /root/vision_state/cam_settings.sh'   # 相机参数

# 主机（参数全在脚本里，别手敲）
DISPLAY=:0 bash core/board/tools/loop_standard.sh --tag acc9r16repro
```

**开跑前必查**：屏幕上页面窗口在最前（相机只拍最上层）、板上无残留 `ffmpeg`、`/tmp/quad.json` 是上面那组。
**不要**在出流中 `kill` ffmpeg —— 这台相机会静默断流，只能重启板子恢复。

## 7. 这一轮之后**还没解决**的三件事（别当成已修好）

1. `|Δy|` 6.8 px → 标定常数偏置（需要一次静止多点重标，不是改代码）；
2. 盲穿 4 道（板子在穿越前从未把它们写进 `@STATE`）；
3. 位置仍来自主机 `@POSE`（R2 要求板子自己积分；这一版不合规）。
