# PL 感知接手文档（新对话从这里开始）

> 写于 2026-10-25。上一段会话上下文耗尽，目标标为 blocked。这份文档只写**本会话实测到的**，
> 没有验证过的会明确标注「未确认」。目标原文见对话里的 goal（`goal-8ffc4c6b`）。
>
> **2026-10-25 深夜更新（P0 已上板跑通）**：正射 + 相机域适配 + vision_top 在真硅上跑通，
> 端到端 **20.16 ms/帧**（判据④ 第一项达标）；位流 `bd_vision_p0.bit/bin`。剩 ④ 的"相机域 300 帧"
> 与 ⑤ 的闭环。细节见 `PL_P0_P1_MIGRATION.md` 的 R82–R87（含 P1 的两个真 bug、FRMBASE 缺失）。
>
> **2026-10-25 晚更新（P0 正射移植）**：正射已经搬进 RTL 并跑通全链路逐字节对拍，
> 吞吐从 507 ms/帧 降到 **9.9 ms/帧**（50 MHz）；判据③ 的 PC 侧瓶颈消失，但**上板还没做**。
> 细节与数字见 `PL_P0_P1_MIGRATION.md` 的 R52–R63，§1/§7 已按新状态改写。

## 0. 一句话现状

PL（复旦微 JFMQL30TAI676H 的 FPGA 侧）已经能替代板端视觉前端跑闭环：**相机域 300/300 帧几何帧
与 C 参考逐字节一致**、墙/缺口位置与基线 `vt_sparse` 差中位 0.4 px；**4 轮闭环里 1 轮 PASS
（最小间隙 +12.23，优于基线 +9.92）、3 轮 FAIL（都是 −10.00 擦墙）**。剩下的单点任务见 §7。

## 1. 目标与判据状态

| 判据 | 状态 | 证据 |
|---|---|---|
| ① 相机域 ≥300 帧，PL 几何帧 vs `vision_ref` 逐字节一致、0 不符 | **✅ 达成** | 300/300（干净环境 + 推入 xor/sum 校验） |
| ② 同帧墙/缺口与 `vt_sparse` ≤2 px 且条数一致 | **🔶 精度达成、条数 ~75%** | Δy 中位 0.40 / p90 1.90 px；缺口边界中位 0.8 / p90 2.0 px；墙命中 311/414 |
| ③ 端到端 ≤35 ms/帧 | **🔶 本地已够，上板未验** | 正射进 RTL：**9.9 ms/帧**（旧 PC 正射 25.7 + 推字 17.3 + PL 22.5 ≈ 66–78 ms）；预算 9.9 + 22.5 ≈ 32 ms |
| ④ PL 作感知源跑闭环 4 轮，不劣于基线 acc9r16 | **🔶 1 PASS / 3 FAIL** | 见 §5 |

基线 `run_20261024_181610_acc9r16`：到达 ✅、0 碰撞、最小间隙 **+9.92**、覆盖 24/24、`|Δy|` 6.8 px（学到墙的 Δy 均值 2.8）。

## 2. 链路与坐标约定

```
主机 platform_host.py 渲染页面（跟随视口） → page_view.py 全屏
板子 ffmpeg（UVC MJPG 640×480, fps=12）→ 640×480 RGB24 → 感知二进制 → /tmp/vt.jsonl
板端 board_agent.py 读 vt.jsonl → @STATE → field_cli_board(M1) → @MOVE
主机收 @MOVE 推进虚拟平台 → 回 @POSE <seq> <x> <y> <r>（真值里程计）
  ⚠ 2026-10-25 修订：本链路是 `@POSE`，不是 `@STEP`。实测 `board_io.jsonl`
    只有 `pose_sent`/`move_recv`（acc9r16 2624/1560，plvision4 5898/1686），
    `@STEP` 方案已冻结，见 `AGENTS.md` 的「R2/R3 修订」。
```

- 页面/世界：`field_w=640, field_h=3420`（本例场景），y 向上。
- **quad 锚在“当前视口(band)”四角**（`platform_host --fiducial-anchor band`），8 个数：
  `38.28 60.05 511.86 43.35 547.97 404.16 24.18 419.71`（`data/fpga/quad.json` == `core/using/calib/quad.json`，md5 `550c00f0…`）。
- PL 输入固定 **640×360**（`vision_top` 的 WIDTH/HEIGHT）。`pl_vision` 把页面按 **0.75 等比放进
  640×360**（左右各 80 px 黑边带），与 `stim_color` 语料同一摆法；反算页面坐标：
  `x_page=(x_pl-80)/0.75`、`y_page=(360-y_pl)/0.75`。
- ⚠ `field_frame::homography()` 的方向是**相机→页面**，要取 `rectify::invert()` 才是页面→相机（曾遇到）。

## 3. 硬件 / 位流 / 命令

- 板子：`ssh -i data/fpga/board_ssh/board_key root@192.168.137.242`；相机 `/dev/video0`
  （另一节点 `/dev/video1` 无 MJPG）；锁定相机参数 `saturation 16 / brightness 70 / gain 4 /
  power_line 1`（重启会丢，用 `core/board/tools/cam_settings_locked.sh` 重设）。
- 位流：`data/fpga/pl_bitstream/bd_vision_r8.bit{.bin}`（bin md5 `3258d6f4…`，5979936 B），
  板上 `/lib/firmware/bd_vision.bin` 就是它，`pl-autoload.service` 上电加载；重载：
  `echo bd_vision.bin > /sys/class/fpga_manager/fpga0/firmware`。
- 重出位流（**batch 可用**，别走 GUI）：
  ```sh
  cmd.exe /c "D:\Vivado\Vivado\2018.3\bin\vivado.bat -mode batch -source C:/pl_build/bd_pack.tcl   -tclargs C:/pl_build/rtl C:/pl_build/bd C:/pl_build/out"
  cmd.exe /c "D:\Vivado\Vivado\2018.3\bin\vivado.bat -mode batch -source C:/pl_build/bd_vision.tcl -tclargs C:/pl_build/rtl C:/pl_build/bd C:/pl_build/out"
  bootgen -image bit2bin.bif -arch zynq -process_bitstream bin -w on -o C:\pl_build\out
  ```
  问题：`bd_pack.tcl` 里 `expr {[info exists argv] && …}` 会被当表达式解析（已修）；`$tag` 无默认值会把
  位流写成隐藏文件 `out/.bit`（已修）；D: 盘偶尔读失败，**重试即可**。
- PL 寄存器（`0x43C00000`）：`0x000+4i` 21 个阈值；`0x080/0x084` 调试口；
  `0x0C0` CTRL(start/busy/done/ovf)；`0x0D4` PIXPUSH；`0x0D8` PIXSTAT；`0x0DC` PIXCTL；
  **诊断**：`0x0E0/0x0E4/0x0E8` = 推入字节/xor/sum，`0x0EC` 空选通写次数，
  `0x0F0` FIFO 任意字回读，`0x0F4` 字数+蓄水池，`0x0F8` 提交次数；`0x100` 几何帧窗口（2048 B）。
  `0x0E0..0x0F8` 是 r8 位流才有的。

## 4. 关键文件

| 文件 | 作用 |
|---|---|
| `core/board/tools/pl_vision.cpp` | **L2 感知源**：stdin 640×480 RGB → 正射（fit 0.75）→ 相机域适配 → 推 PL → 读几何帧 → 写 vt.jsonl。含推入 xor/sum 校验+重推、quad-state 落盘、缺口去重/宽度过滤/角色假缺口过滤 |
| `core/board/tools/build_pl_vision.sh` | 板端编译（**必须 `-O3 -funroll-loops`**：正射 37.7→25.7 ms） |
| `core/board/tools/loop_standard.sh` | 标准闭环；`CAM_VT_BIN` 环境变量切感知源（默认 `/root/tools/vt_sparse`，PL 用 `/root/tools/pl_vision`） |
| `core/fpga/tools/feed_pl.c` / `feed_pl.py` | 单元级喂数器（L0/L1 用）；`.py` 必须用 `ctypes` 写、不能用 `struct.pack_into` |
| `core/fpga/tools/board_l0.sh` / `l0_sweep.py` | 传位流+重载+L0 对拍（屏幕录像语料） |
| `core/fpga/tools/fifoprobe.py` | FIFO 内部状态/内容探针（定位根因的仪器） |
| `core/fpga/rtl/vision_axi.v` / `vision_pixfeed.v` | W_PIXPUSH 按 WSTRB 判定；pixfeed 诊断口 |
| `core/fpga/cosim/tb_axi.cpp` | `--fifo`、`--mimic-gp`（模拟主控三笔写）、`--word-gap N` |
| `core/fpga/docs/PL_A_B_TASK.md` | **全部证据链**（round 7–11，务必读） |
| `configs/20_walls_alt_24_dy140_gap8.json` | 锁定场景（墙 y=80…3300 步长 140；缺口在 `meta.gaps_x/gap_widths`） |
| `core/using/calib/calib_y.json` | **锁定标定**（scale 0.85973 / bias 39.868，来自 acc9r16） |

## 5. 闭环四轮实测（PL 感知源）

| 轮次 | 到达 | 碰撞/最小间隙 | 覆盖 | \|Δy\| | 学到墙 Δy 均值 |
|---|---|---|---|---|---|
| plvision（r1） | ✅ | 1 / **−10.00** | 24/24 | 3.3 | 7.1 |
| plvision2（r2） | ✅ | 1 / **−10.00** | 24/24 | 3.9 | 6.9 |
| **plvision3（r3）** | ✅ | **0 / +12.23** | 24/24 | 4.1 | — |
| plvision4（r4） | ✅ | 1 / **−10.00** | 24/24 | 3.1 | 7.0 |

三轮失败的最小间隙都是 **−10.00**（同一签名）。已定位的两类机制：

1. **角色把墙切成“假缺口”**（r1/r2）：角色的中性亮像素把墙的连续段截断，缺口逻辑把它当开口；
   实测 y≈220 那道墙报出的缺口在帧间乱跳（93…207 / 135…239 / 342…446 / 466…579），
   x 正好是角色位置 → 机器人瞄准自己脚下的缺口撞墙。已修（角色中心在缺口内且宽度 < 2r+20 则丢弃）。
2. **墙发现太晚**（r4）：真值 y=1900 那道墙“首次见 py=**1894.4**”（距离 ~6），基线是 ~185；
   机器人沿原方向直行穿墙（碰撞点 x=314.6，缺口在 x∈[396.3,502.3]）。**这条还没修，是当前唯一拦路项。**

## 6. 已验证的技术事实（别重复推翻）

- PL 前端在真硅上判不出墙的**原根因是喂数器写法**：Python `struct.pack_into("<I")` 一次 32 位 store
  在硅上变成 **3 次 AXI 提交**（FIFO 进 3 份 → 图像错位，而帧长/CRC 自洽）；`ctypes.c_uint32` / C 的
  volatile 写都是 1 次。板上 `0x0F8` 可以自己数。
  ⚠ round 7 曾把成因写成“1 笔数据 + 2 笔空选通”，但后来实测 `zstrb=0`（空选通计数为 0），
  说明真实形态是“3 笔带数据的写、其中 2 笔数据为 0、数据落在第 3 个槽”。**两种说法都只有部分证据**，
  如要写进结论需重新取证。
- 像素 FIFO 的推入顺序必须是：**等上一帧落 → flush → start → 等 busy → 推**（先 start 后 flush 会
  1/N 帧整幅错位）。
- 相机把暖白墙拍成**中性偏蓝**（R−B 从 +55 变 −5；饱和度调到 100 反而 −37）→ 颜色判据在相机域不可用；
  墙亮度 ≈198、角色 ≈200~255、场地中位 26~32（p90 70~80）→ 亮度也不可分；**只能按形状**分：
  每行亮像素连续段 ≥40 px 视为墙（染暖白 R−B=60），短亮斑抬到 210 保持中性（角色）。
  适配阈值 **150**（40 会把场地染成墙，f1/f2 曾整幅变一堵假墙）。
- 页面白框**不能**在图像上抹掉（判据靠它确定场地范围 span=[80,559]），只能在输出阶段按
  “离页面上下沿 6 px 不算墙”过滤。
- 缺口宽度合法区间按基线自身分布取 **[60,140] 页面单位**（基线 p50 105、min 45、max 162）。
- L0（屏幕录像语料 `data/fpga/stim_color`，320 帧）本来就是 **320/320 通过**；相机域要单独验。

## 7. 下一步（单点任务，按顺序做）

**A. 查清“可见距离 ~6 vs 基线 ~185”**（当前唯一拦路项）
1. 从 `data/fpga/loop_logs/run_…_plvision4/board_vt.jsonl` 取机器人 `py∈[1700,1890]` 的帧；
2. 用同一批相机帧跑 `pl_vision --dump-ppm X`，拿到**适配后**的 640×360 图（想拿适配前的用 `--no-camera-domain`）；
3. 量这两道墙所在行的**亮像素连续段长度**分布；
4. 若普遍 <40 px → 就是 `kAdaptRunMin=40` 挡掉了墙，按实测改门槛（改完必须重跑相机域 300 帧确认仍 300/300）；
   否则转查 band/gap 判据（`band_min_rows`/`wall_col_pct` 等，改判据要同步 `vision_ref.c`，R4）。

**B. 修好后按 R1c 连跑 4 轮闭环**，判据：到达 + **0 碰撞** + 24/24 + 最小间隙 ≥ +9.92。

**C. 判据③（RTL 侧已完成，剩下集成+上板）**：正射已进 RTL（`rtl/vision_ortho.v`，
流水线除法 `vision_div_pipe.v`，行缓存 `vision_linecache.v` 32 行，AXI 读主口
`vision_axi_master.v`，填充控制 `vision_linefill.v`，连线层 `vision_ortho_sys.v`）。
本地全链路逐字节 0 不符、**9.9 ms/帧 @50 MHz**（R63）。**没做的**：

1. `vision_axi` 里加 P0 寄存器块（FRMBASE/frame start/统计 + 13 个系数）与 M_AXI 主口，
   把 `vision_ortho_sys` 例化进去，像素流接到 vision_top 的 din_*（`hold = !busy`）；
2. `bd_vision.tcl` 把 M_AXI 接 `ps7_0/S_AXI_HP0`，重出位流；
3. PS 侧 `pl_vision.cpp` 改成"把 640×480 帧 memcpy 进 udmabuf → 写 FRMBASE → start →
   读几何帧"（去掉 PC 正射与 PIXPUSH 17.3 ms）。

## 8. 运维纪律（今天曾遇到的问题，务必遵守）

1. **跑闭环前**：`cp core/using/calib/calib_y.json data/fpga/calib_y.json` —— 闭环结束会自动**改写**它。
2. **跑闭环后**：清相机管线，否则它会一直占着像素 FIFO，让后续离线测量两个进程抢同一个 FIFO
   （实测推入字节数多 34%、偶发整幅错位）：
   ```sh
   ssh … 'pkill -TERM -f "tools/pl_vision --w"; pkill -TERM -f "ffmpeg -hide_banner -loglevel error -fflags"'
   ```
3. 任何时刻**只能有一个**进程往 `0x0D4` 推像素（`pl_vision` 或 `feed_pl_c`），否则测量无效。
4. 不要 `kill -9`；批量 pkill 的模式别匹配到自己的 shell。
5. `pl_vision` 开机**写全 21 个配置寄存器**——PL 的寄存器会粘住，只写覆盖项会让同一张图两次运行结果不同。
6. 相机快照要同时看 mtime，过期（>5 s）不能当“当前画面”。
7. 一次大改只跑一轮闭环，然后把那一轮日志翻干净（`check_run_logs.py <run>`）。

## 9. 复现命令

```sh
# 相机域 L0（300 帧，PL 帧 vs C 参考逐字节 + 与 vt 比位置）
#   板上：/tmp/rec.mkv 是某轮相机录像；脚本在 /tmp/l0cam.py（本会话临时写的，未入库，需重建）
# 8 帧快速对照
#   板上：/tmp/vs.py + /tmp/cs/raw_*.rgb（同样未入库）

# 板端编译 pl_vision
ssh -i data/fpga/board_ssh/board_key root@192.168.137.242 'sh /root/board_src/vt/../../tools/build_pl_vision.sh'
# （源码在 /root/board_src/vt/pl_vision.cpp + geometry_frame.cpp）

# 闭环（PL 感知源）
CAM_VT_BIN=/root/tools/pl_vision core/board/tools/loop_standard.sh --tag plvisionN
# 翻日志
python3 core/board/tools/check_run_logs.py data/fpga/loop_logs/run_<ts>_plvisionN
```

⚠ 上面 `/tmp/l0cam.py`、`/tmp/vs.py` 是会话内临时脚本，**没有入库**；重建时照 §7 的描述写即可
（要点：每轮先删旧 dump 文件、比对时遮掉 seq(8..11)/stamp(24..31)/crc 末尾 4 字节并各自校验 CRC 自洽）。

## 10. 明确没做到 / 存疑（别当成已完成）

- 判据③（≤35 ms/帧）从未达标。
- 判据②“条数一致”只有 ~75%：贴页面左沿的墙（缺口以场地边界为界）会被 24 px 段长门槛削掉 → 漏报。
  要修就得改缺口判据（允许场地边界作为缺口的一侧），参考与 RTL 要同步改。
- `pl_vision` 的相机域适配参数（连续段 40 px、阈值 150、染色 60、角色过滤 2r+20、宽度 [60,140]）
  都是**在 8~300 帧真实相机帧上量出来的**，不是从原理推的；换场景/换光照需要重新量。
- round 7 关于“3 笔写”成因的两种解释都没有闭环证据（见 §6 第一条）。
