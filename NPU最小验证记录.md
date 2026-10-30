# NPU（AI_Mate / ZG330）打通记录

日期：2026-10-22　板子：JFMQL30TAI676H（`U.FMSH`，192.168.137.242）

## 结论

**NPU 是真的、能用的。** 现在板上已能跑 yolov5s 的前向：
**稳定 5.6 ms/次（≈178 FPS 仅推理），设备侧 `session.totalTime` = 5.47 ms**。

之前所有"位流里没有 AI_Mate / NPU 部署不了"的判断，根因只有一个：
**含 AI_Mate 的位流和配套的 PS 初始化从来没上过板**，不是硬件不存在。

## 关键事实（全部实测）

| 项目 | 结果 |
|---|---|
| iCore 版本 | `FMSHZGV3TECH-AID - 24160628` |
| 设备 ID | `25122301` |
| 设备 URL | `axi://zg330aiu?npu=0x40000000&dma=0x80000000` |
| NPU 阵列 | `getRowsCols() = (4, 2)` |
| DetPost 硬算子 | `DetPostZG Version 3.33.1.0`，`reg_base = 0x100000800` |
| 模型 | `yolov5s_plin_352x640_ZG.json`（93 算子）+ `.raw`（8.6 MB） |
| 输入 | `[1, 352, 640, 3] uint8 NHWC` |
| 输出 | 3 个 tensor（DetPost 输出，`int8`） |
| 单次前向 | 5.6 ms（最小 5.6 / 中位 5.6 / 最大 9.5，10 次） |
| 首次前向 | 9.5 ms（含首帧开销） |

## 正确调用顺序（踩坑后总结，照这个写就不会挂）

```python
from icraft import xir, xrt
from icraft.zg330backend import ZG330Backend
from icraft.host_backend import HostBackend
import numpy as np

net = xir.Network.CreateFromJsonFile(JSON)
net.lazyLoadParamsFromFile(RAW)        # ← 必须！漏了这一步 apply 会段错误
dev = xrt.Device.Open("axi://zg330aiu?npu=0x40000000&dma=0x80000000")
view = net.view(0)                      # ← 单参数（0 表示从头到尾）
sess = xrt.Session.Create([ZG330Backend, HostBackend], view,
                          [dev, xrt.HostDevice.Default()])   # ← 必须双后端，且传**基类** Device
zg = ZG330Backend(sess.backends[0])
zg.disableEtmOptimize()                 # yaml 的 compressFtmp=false 对应
zg.disableMergeHardOp()                 # yaml 的 speedMode=false 对应
sess.apply()
# 输入
val = net.ops[0].outputs[0]             # InputNode 的输出
tt  = val.tensorType()
shape = tt.shape if not callable(tt.shape) else tt.shape()
t = xrt.Tensor(np.random.randint(0,255,shape,dtype=np.uint8), tt)
out = sess.forward([t])                 # 3 个输出
```

## 四个坑（每个都让板子挂过或崩过）

1. **`readReg(cid, rid, offset)` 会挂死板子**。设备能开、`getRowsCols()` 能读，
   但读内部寄存器（如 `readReg(0,0,0)`）会让 AXI 读永远不返回 → 只能断电。
   官方 demo 用的是 `defaultRegRegion().write(...)`，不是这个接口。**别碰 `readReg`**。
2. **`Session.Create(..., devices=[ZG330Device])` 报 TypeError**：要传 `Device::Open`
   返回的**基类** `Device`，不是 cast 出来的 `ZG330Device`。
3. **只用 `ZG330Backend` 会报 `Op is NOT bound`**（`OutputNode` 之类），要
   `[ZG330Backend, HostBackend]` 双后端 + `[dev, HostDevice.Default()]` 双设备。
4. **不 `lazyLoadParamsFromFile(RAW)` 就 `apply()` → 段错误**（进程崩溃，板子没事）。

## 位流与引导（板上现在的状态）

- FAT 分区 `BOOT.bin` 已替换成厂家 `BIT/30tai_detpost/BOOT.bin`
  （md5 `c69904af9d52b9e0fff63213deb4965e`），原文件备份在 FAT 内 `BOOT.BIN.bak.orig`
  和板内 `/root/BOOT.BIN.orig`。
- 该引导里的 FSBL 会配 PS（DDR/时钟）并加载 `ai7030_edif_top` 位流
  （design `ai7030_edif_top`，7z030ffg676，2026/09/16）。
- `pl-autoload.service` **已 disable**（否则开机会把我们自己的 `bd_vision.bin` 覆盖上去）。
- 所以**现在 PL 里是厂家的 AI 参考设计**，我们自己的视觉 IP 不在 PL 里。
  注意 `vt_sparse` 是纯 CPU 的，闭环链路不依赖 PL，不受影响。

### 回滚成"我们自己的 PL 视觉"

```
cp /root/BOOT.BIN.orig /root/bits/BOOT.bin   # 先 mount /dev/mmcblk0p1 /root/bits
systemctl enable pl-autoload.service
reboot
```

## 位流格式（自己载入时需要）

FMSH FPGA manager 要"字节交换后的 .bin"：

1. 跳过 `.bit` 的 13 字节头（`00 09 0f f0 0f f0 0f f0 0f f0 00 00 01`）；
2. 解析 `a/b/c/d`（2 字节长度）+ `e`（4 字节长度）字段，取 `e` 的载荷；
3. **每 4 字节整体反序**；
4. `echo <文件> > /sys/class/fpga_manager/fpga0/firmware`。

不这么做会报 `Invalid bitstream, could not find a sync word`。

## 遗留

- 运行时 3.33.1 解析 3.36.0 编译的 json 会告警（能跑）：
  `You are using Icraft v3.33.1 to parse the json file compiled by Icraft v3.36.0`。
  真要消掉就整套升 `30TAI&100TAI.zip` 里的 3.39.0（deb + wheel 一起换）。
- 输出 tensor 的 shape 元数据是 `(1,1,0,96)`，需要按 demo 的后处理（anchors/NMS，
  `fpga_nms: false`, `detpost: true`）解析才能拿到检测框。
- 板上留下的文件：`/root/icraft_*_test*.py`、`/root/fpai_demo/imodel/ZG/*`、
  `/lib/firmware/fpai_30tai_detpost*.{bit,bin}`、两个 `.deb`、日志
  `/root/fpai_demo/npu_run*.log`。
- 已装包：`icraft 3.33.1`、`customop 3.33.1`（`dpkg -r` 可回滚）。

---

## 2026-10-24 更正：**前面那条"打通"结论不成立**（输出与输入无关）

用 `core/board/tools/npu_probe.py` 把 bus.jpg（有 bus/人）和**全黑图**分别喂进去，
**输出逐字节相同**（3 个输出 tensor 的 min/max/mean/nonzero 完全一致）：

```
out[0] shape=(1,1,3,96)  int8  相同=True  最大绝对差=0.0
out[1] shape=(1,1,53,96) int8  相同=True  最大绝对差=0.0
out[2] shape=(1,1,34,96) int8  相同=True  最大绝对差=0.0
```

试过四种喂数方式都无效（构造参数 `Tensor(buf,tt)` / 复用张量 `copyData` /
每次新建张量 / `forward(in,out)` 显式给输出；`mallocOn(HostMemRegion())` 直接段错误）。
所以**之前 5.6 ms 只是"runtime 在跑"，NPU 并没有真的算我们的图**，
sidecar 里 `out_shapes=[[1,1,0,96]×3]` 的"0 检测"也就解释得通了。

**根因线索（两条，指向同一件事）**：

1. 板上这份模型是 **ICraft/XIR 3.36 编译**的，而板上 runtime 是 **3.39**：
   `[W] You are using Icraft v3.39.0 to parse the json file compiled by Icraft v3.36.0`
   `[W] You are using XIR 3.39.0.0 to parse the json file compiled by XIR 3.36.0.1`
2. 官方命令行 `icraft-run` 在板上直接失败：
   ```
   Exception: Check failed: (is_bound) is false
   Op is NOT bound to any backend, op_id = 671, type_key = icraft::xir::HardOpNode
   ```
   —— **DetPost 那个硬算子没被绑定**（`icraft-run` 不知道 ZG330/DetPost 的注册），
   说明这份 json 与当前 runtime 的算子表对不上。

**结论**：要先把模型用**本机这套 3.39 编译器**重新编译（哪怕先用厂商的现成模型），
再上板验证；`npu_probe.py` 已经备好（支持 `--mode fwd1|reuse|fresh|host` 与
`--blank` 对照），编译出新模型后重跑一次：**bus.jpg 出框、全黑图 0 框**才算真通。
