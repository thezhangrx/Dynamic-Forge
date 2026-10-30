# NPU 编译链现状（P0 进展与卡点）

日期：2026-10-22（第二轮更新）　目标见 `NPU目标_障碍虚拟真值准入.md`

## 一、环境（可用）

| 组件 | 位置 | 状态 |
|---|---|---|
| x86 Python 3.8.20（standalone，免 root） | `Env/py38`（239 MB） | ✅ |
| ICraft 3.39.0 工具链（deb 解包） | `Env/icraft339`（2.8 GB） | ✅ 8 个 CLI + 230 个 .so |
| `icraft` python 包（x86_64 wheel） | `Env/py38/.../site-packages/icraft` | ✅ `xir.FullVersion()=3.39.0.0` |
| 用法 | `LD_LIBRARY_PATH=Env/icraft339/usr/lib/x86_64-linux-gnu`，`PYTHONPATH` 同目录 | ✅ |
| 板端运行时 | 已从 **3.33.1 升级到 3.39.0**（`Icraft_3.39.0_onchip.deb` + aarch64 whl） | ✅ 厂家 yolov5 仍 **5.6 ms** |

> 本机 `sudo` 被禁（no-new-privileges），所以工具链是"解包到工作区 + 环境变量"，没有装进系统。

## 二、整条链跑通到哪一步

```
ONNX ──parse──▶ parsed.json ──(补输入形状)──▶ quantize ──▶ generate ──▶ ZG.json
  ✅               ✅                            ✅            ⚠ 只出目标，不出指令
```

### 2.1 `icraft-parse`：✅

必须同时给这七个参数，格式很挑：

```
--target Zhuge                       # 不是 ZG330 / ZHUGE / Zg330
--inputs 1,480,640,3                 # 4 维，NHWC（只接受 NHWC/FD/NDHWC）
--inputs_dtype FP32
--inputs_layout NHWC
--pre_method Resize                  # 只有 "Resize" 被接受；别的值报 isNum
--pre_scale 1,1,1                    # 长度必须 == 通道数
--pre_mean  0,0,0                    # 同上
--channel_swap 0,1,2                 # 必须是**合法排列**（不能 0,0,0，会报 channel_swap[g]!=channel_swap[j]）
```

ONNX 必须是 **4 维 NCHW**（torch 风格）。网络用小 CNN 才走得通；
`Flatten → Gemm` 那种 MLP 会在 parse 阶段被插进 Transpose/Reshape，后面全崩。

### 2.2 补输入形状（关键一步）

parser 产出的第 0 个算子（InputNode）形状是 `[-1,-1,-1,-1]`，
量化器直接报 `hasNegativeDim`。**把 json 里那一处 shape 改成具体值即可**：

```python
d['ops'][0]['outputs'][0]['dtype']['shape'] = [1,480,640,3]
```

### 2.3 `icraft-quantize`：✅

```
--forward_dir <校准目录> --forward_list <文件列表> --target Zhuge --qdtype int8
```

两个坑：
1. 校准样本必须是**裸二进制**（`.bin`），不是 `.npy`（npy 有 header，报 `filesize == tensorType().bytes()` 失败）；
2. `--forward_list` 里写**相对文件名**（绝对路径报 `fs::exists(filename_string)` 失败）。

**量化之后算子会被打上 `@zhuget(330)` 目标**——这是到目前为止最关键的一步。

### 2.4 `icraft-generate`：⚠️ 只出了目标，没出指令

`icraft-generate` 一开始报 `fs::exists(dll_path)` 失败。原因是它要按目标名去找
`libicraft_<target>_codegen.so`，而包里**只有 `libicraft_buyi_codegen.so`**。
把 buyi 的库改名成 `libicraft_zhuge_codegen.so` 后，generate **能跑完并产出文件**，
但产出的算子只有：

```
['_type_key','compile_target','inputs','name','op_id','outputs','tags','target_dtype']
```

而厂家模型（同样 `@zhuget(330)`）的算子是：

```
['_type_key','compile_target','device_info','inputs','instr','layer_count',
 'name','op_id','origin_ops_id','outputs','params','sub_hard_ops','tags']
```

**差的就是 `instr` / `params` / `layer_count` / `device_info`——即真正的硬件指令。**
换句话说：那个改名过的库是 Buyi 的 codegen，它不认识 Zhuge，只是把目标标签抄了过去。

### 2.5 板上验证（旁证）

把自编的 `cls_cnn_ZG.json` 拷到板子上（已升级到 3.39.0）：

```
Op is NOT bound to any backend, op_id = 5, type_key = icraft::xir::Conv2dNode
```

因为算子没有 `instr`，ZG 后端无法把它绑到硬件——**与 2.4 的结论一致**。

## 三、结论

| 环节 | 状态 |
|---|---|
| 环境（py3.8 + 3.39.0） | ✅ 完全打通 |
| 解析 / 量化 / 生成目标 | ✅ 打通，能产出 `@zhuget(330)` 标号的 int8 模型 |
| **生成硬件指令（Zhuge codegen）** | ❌ **官方包里没有这个组件** |
| 板端运行时 | ✅ 3.39.0，厂家模型 5.6 ms 正常 |
| 自编模型上板 | ❌ 卡在缺 codegen |

**唯一缺件：Zhuge（ZG330）的 codegen 实现**——一个能产出 `instr/params/layer_count`
的 `libicraft_zhuge_codegen.so`（或等价的编译工具）。

## 四、给厂家的问法（可直接发）

> 我们用 `Icraft_3.39.0_amd64.deb` 编 ZG330 模型：
> `icraft-generate` 报 `fs::exists(dll_path)` 失败；检查发现包里只有
> `libicraft_buyi_codegen.so`，没有 Zhuge 的 codegen。把 buyi 的库改名后能生成文件，
> 但算子只有 `target_dtype`，没有 `instr/params/layer_count`，上板报
> `Op is NOT bound ... Conv2dNode`。
> 请问 **ZG330 的 codegen 组件（能产出硬件指令的那个）在哪里获取？**
> 或者是否有能直接产出 `@zhuget(330)` + `instr` 的 Linux 版编译工具/服务？

## 五、不依赖 codegen 的并行工作（下一步）

1. 行剖面特征落盘（`vt_sparse` 加 dump）；
2. 标签生成（123 轮已有 `board_vt.jsonl` + `local_truth.jsonl`）；
3. 小分类模型训练 + **CPU 后端**验证精度/时延（路径已验证可用：`[HostBackend]`）。

这样 codegen 一到，只需 `parse → quantize → generate` 三步即可上板。
