#!/usr/bin/env python3
"""npu_probe.py —— 在**板上**用一张指定图片跑 NPU，把输出 tensor 的真实内容打出来。

为什么需要它
------------
`npu_sidecar.py` 跟相机跑时，`out_shapes` 一直是 `[[1,1,0,96],[1,1,0,96],[1,1,0,96]]`
—— 也就是说 DetPost 每次都输出 **0 个检测**。这有两种完全不同的原因：
  (a) 预处理/张量顺序错（NHWC vs NCHW、uint8 vs float、量化 scale 不对）→ 要修管线；
  (b) 模型对"白框 + 横墙"这种画面**本来就没有目标**（它是 COCO 模型）→ 属预期。
拿一张**确定有目标**的图（bus.jpg 里有 bus/人）跑一次就能分辨：
  - 出框 → 管线是对的，我们场景里没 COCO 目标 → 结论是"要换/训模型"；
  - 仍是 0 框、或输出全 0 → 管线有问题（先修管线，别急着训）。

用法（板上）
-----------
    python3 npu_probe.py --img test.jpg
    python3 npu_probe.py --img test.jpg --json ... --raw ... --nhwc 1
    python3 npu_probe.py --img test.jpg --blank          # 同时跑一张全黑图做对照
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

DEFAULT_JSON = "/root/fpai_demo/imodel/ZG/yolov5s_plin_352x640_ZG.json"
DEFAULT_RAW = "/root/fpai_demo/imodel/ZG/yolov5s_plin_352x640_ZG.raw"
DEV_URL = "axi://zg330aiu?npu=0x40000000&dma=0x80000000"


def letterbox_u8(path: str, th: int, tw: int) -> np.ndarray:
    """读图 → 等比缩放 + 补边到 (th, tw) → uint8 NHWC (1,th,tw,3)。"""
    import cv2

    im = cv2.imread(path, cv2.IMREAD_COLOR)          # BGR
    if im is None:
        raise SystemExit(f"读不到图片：{path}")
    h, w = im.shape[:2]
    s = min(th / float(h), tw / float(w))
    nh, nw = max(1, int(round(h * s))), max(1, int(round(w * s)))
    rs = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((th, tw, 3), 114, np.uint8)     # YOLO 常用灰边
    y0, x0 = (th - nh) // 2, (tw - nw) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = rs
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)    # 模型训练用 RGB
    return rgb.reshape(1, th, tw, 3)


def summarize(name: str, arr: np.ndarray) -> None:
    a = np.asarray(arr)
    nz = int(np.count_nonzero(a))
    print(f"  {name}: shape={a.shape} dtype={a.dtype} "
          f"min={a.min() if a.size else '-'} max={a.max() if a.size else '-'} "
          f"mean={float(a.mean()) if a.size else 0:.3f} nonzero={nz}")
    flat = a.reshape(-1)[:24]
    if flat.size:
        print(f"    前 24 个值: {flat.tolist()}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="板上 NPU 单图探针")
    ap.add_argument("--img", required=True)
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--raw", default=DEFAULT_RAW)
    ap.add_argument("--feed_all", action="store_true",
                    help="三种喂法都试一遍（判定哪种真正生效）")
    ap.add_argument("--h", type=int, default=352)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--blank", action="store_true", help="额外跑一张全黑图对照")
    ap.add_argument("--mode", default="fwd1", choices=("fwd1", "fwd2", "reuse", "fresh", "host"),
                    help="fwd1=forward([in]) 用返回值；fwd2=forward([in],[out]) 显式给输出")
    ap.add_argument("--ready", action="store_true", help="forward 前 setReady(True)")
    ap.add_argument("--feed", default="copy",
                    choices=("copy", "ctor", "write"),
                    help="喂数据方式：copy=copyData(推荐) / ctor=构造参数 / write=Tensor.write")
    a = ap.parse_args(argv)

    from icraft import xir, xrt
    from icraft.zg330backend import ZG330Backend
    from icraft.host_backend import HostBackend

    net = xir.Network.CreateFromJsonFile(a.json)
    net.lazyLoadParamsFromFile(a.raw)              #: 必须！漏了 apply 会段错误
    dev = xrt.Device.Open(DEV_URL)
    view = net.view(0)
    sess = xrt.Session.Create([ZG330Backend, HostBackend], view,
                              [dev, xrt.HostDevice.Default()])
    zg = ZG330Backend(sess.backends[0])
    zg.disableEtmOptimize()
    zg.disableMergeHardOp()
    sess.apply()

    val = net.ops[0].outputs[0]
    tt = val.tensorType()
    shape = tt.shape if not callable(tt.shape) else tt.shape()
    print(f"模型输入 shape={shape} dtype={tt.dtype if hasattr(tt, 'dtype') else '?'}")

    cases = [("real", a.img)]
    if a.blank:
        cases.append(("blank", ""))

    results = {}

    if a.mode == "host":
        #: 标准写法：按类型建张量 → 在 HostMemRegion 上分配内存 → copyData → forward
        xs = {}
        for tag, path in cases:
            xs[tag] = (np.zeros((1, a.h, a.w, 3), np.uint8) if tag == "blank"
                       else letterbox_u8(path, a.h, a.w))
        t = xrt.Tensor(tt)
        try:
            t = t.mallocOn(xrt.HostMemRegion())
        except Exception as exc:                     # noqa: BLE001
            print(f"⚠ mallocOn 失败：{exc}")
        for tag in ("real", "blank"):
            buf = np.ascontiguousarray(xs[tag]).tobytes()
            t.copyData(buf)
            outs = sess.forward([t])
            arrs = [o if isinstance(o, np.ndarray) else np.asarray(o) for o in outs]
            results[tag] = arrs
            print(f"\n[{tag}]")
            for i, arr in enumerate(arrs):
                summarize(f"out[{i}]", arr)
        print("\n=== 判定 ===")
        same_all = True
        for i, (r, b) in enumerate(zip(results["real"], results["blank"])):
            same = r.shape == b.shape and np.array_equal(r, b)
            same_all = same_all and same
            d = (float(np.abs(r.astype(np.int32) - b.astype(np.int32)).max())
                 if r.shape == b.shape and r.size else -1)
            print(f"  out[{i}]: 换图后相同={same}  最大绝对差={d}")
        print("  ✗ 输入没生效" if same_all else "  ✓ 输入生效")
        return 0

    if a.mode in ("reuse", "fresh"):
        #: 两种"每次换图"的做法：
        #:  reuse = 建一次张量（拿不到 chunk 就先 forward 一次），之后只 copyData
        #:  fresh = 每次都新建张量（若输出与 reuse 不同，说明绑定是"首次生效"）
        from icraft import xir as _xir
        del _xir
        xs = {}
        for tag, path in cases:
            if tag == "blank":
                xs[tag] = np.zeros((1, a.h, a.w, 3), np.uint8)
            else:
                xs[tag] = letterbox_u8(path, a.h, a.w)
        key = "real"
        buf0 = np.ascontiguousarray(xs[key]).tobytes()
        t0 = xrt.Tensor(buf0, tt)
        outs0 = sess.forward([t0])
        results[key] = [o if isinstance(o, np.ndarray) else np.asarray(o)
                        for o in outs0]
        buf1 = np.ascontiguousarray(xs["blank"]).tobytes()
        if a.mode == "reuse":
            t0.copyData(buf1)                      #: 复用同一张量
            t = t0
        else:
            t = xrt.Tensor(buf1, tt)               #: 新建张量
        outs1 = sess.forward([t])
        results["blank"] = [o if isinstance(o, np.ndarray) else np.asarray(o)
                            for o in outs1]
        for tag in ("real", "blank"):
            print(f"\n[{tag}] 输出:")
            for i, arr in enumerate(results[tag]):
                summarize(f"out[{i}]", arr)
        print("\n=== 判定：输入是否真的进了 NPU ===")
        same_all = True
        for i, (r, b) in enumerate(zip(results["real"], results["blank"])):
            same = r.shape == b.shape and np.array_equal(r, b)
            d = (float(np.abs(r.astype(np.int32) - b.astype(np.int32)).max())
                 if r.shape == b.shape and r.size else -1)
            same_all = same_all and same
            print(f"  out[{i}]: 换图后相同={same}  最大绝对差={d}")
        print("  ✗ 输入没生效" if same_all else "  ✓ 输入生效")
        return 0

    for tag, path in cases:
        if tag == "blank":
            x = np.full((1, a.h, a.w, 3), 0, np.uint8)
        else:
            x = letterbox_u8(path, a.h, a.w)
        print(f"\n[{tag}] 输入 {path or '(全黑)'} → {x.shape} {x.dtype} "
              f"mean={x.mean():.1f}")
        buf = np.ascontiguousarray(x).tobytes()
        if a.feed == "ctor":
            t = xrt.Tensor(np.ascontiguousarray(x), tt)
        else:
            t = xrt.Tensor(tt)
            if a.feed == "copy":
                #: **这才是喂数据的正确方式**：`xrt.Tensor(arr, tt)` 不会把 arr 拷进去
                #: （实测 bus.jpg 与全黑图输出逐字节相同，就是因为它被忽略了）。
                t.copyData(buf)
            else:
                t.write(0, buf, len(buf))
        if a.ready:
            t.setReady(True)
        if a.mode == "fwd2":
            #: **两参数重载**：输出 Tensor 由调用方给，session 往里写。
            #: 单参 `forward([t])` 的返回值可能根本不是一个被更新过的缓冲
            #: （实测 bus 与全黑图输出逐字节相同，就是这种"看着跑、其实没算"）。
            vals = getattr(view, "outputs", None) or getattr(net, "outputs", None)
            if vals is None:
                print("⚠ 拿不到网络输出 Value，退回 fwd1")
                outs = sess.forward([t])
            else:
                outs_t = [xrt.Tensor(v) for v in vals]
                sess.forward([t], outs_t)
                outs = outs_t
        else:
            outs = sess.forward([t])
        nz_total = 0
        arrs = []
        for i, o in enumerate(outs):
            arr = o if isinstance(o, np.ndarray) else np.asarray(o)
            arrs.append(arr)
            summarize(f"out[{i}]", arr)
            nz_total += int(np.count_nonzero(arr))
        results[tag] = arrs
        print(f"  → 输出非零元素合计 {nz_total}")
        if nz_total == 0:
            print("  ⚠ 输出全 0：**管线问题**（张量布局/量化/后端配置），不是'场景没目标'")
        else:
            print("  ✓ 输出有值：管线是通的；若框数仍为 0，就是'这张图里没有目标'或"
                  "阈值/后处理的问题")
    if "real" in results and "blank" in results:
        print("\n=== 判定：输入是否真的进了 NPU ===")
        same_all = True
        for i, (r, b) in enumerate(zip(results["real"], results["blank"])):
            same = r.shape == b.shape and np.array_equal(r, b)
            d = (float(np.abs(r.astype(np.int32) - b.astype(np.int32)).max())
                 if r.shape == b.shape and r.size else -1)
            same_all = same_all and same
            print(f"  out[{i}]: real vs blank 逐字节相同={same}  最大绝对差={d}")
        if same_all:
            print("  ✗ **输入没生效**：bus.jpg 与全黑图输出完全一样 → 喂数方式/张量绑定错")
        else:
            print("  ✓ 输入生效（输出随图像变化）→ 管线通；再看框数是否 >0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
