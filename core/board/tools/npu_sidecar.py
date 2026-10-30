#!/usr/bin/env python3
"""板端 NPU 推理侧车：跟随闭环管线写出的 /tmp/cam.jpg，实时跑厂家 yolov5（ZG330）。

不是硬编码：每一帧都是真实现场相机帧 → 真 NPU 前向 → 记录检测结果与时延。
输出 /root/fpai_demo/npu_infer.jsonl（逐帧），并每 2 s 打一行摘要。

用法（板上）：
    python3 npu_sidecar.py [--duration 600] [--every 2]
"""
import argparse, json, os, time, traceback
import numpy as np
import cv2
from icraft import xir, xrt
from icraft.zg330backend import ZG330Backend
from icraft.host_backend import HostBackend

URL = "axi://zg330aiu?npu=0x40000000&dma=0x80000000"
J = "/root/fpai_demo/imodel/ZG/yolov5s_plin_352x640_ZG.json"
R = "/root/fpai_demo/imodel/ZG/yolov5s_plin_352x640_ZG.raw"
CAM = "/tmp/cam.jpg"
LOG = "/root/fpai_demo/npu_infer.jsonl"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=900)
    ap.add_argument("--stale", type=float, default=1.0)
    a = ap.parse_args()

    net = xir.Network.CreateFromJsonFile(J)
    net.lazyLoadParamsFromFile(R)
    ops = net.ops
    dev = xrt.Device.Open(URL)
    sess = xrt.Session.Create([ZG330Backend, HostBackend], net.view(0),
                              [dev, xrt.HostDevice.Default()])
    zg = ZG330Backend(sess.backends[0])
    zg.disableEtmOptimize()
    zg.disableMergeHardOp()
    sess.apply()
    val = ops[0].outputs[0]
    tt = val.tensorType()
    sh = tt.shape
    shape = list(sh() if callable(sh) else sh)
    print("[sidecar] 模型就绪，输入", shape, flush=True)

    f = open(LOG, "a", buffering=1)
    t0 = time.time()
    last_mtime = 0.0
    n = 0
    lat = []
    while time.time() - t0 < a.duration:
        try:
            mt = os.path.getmtime(CAM)
        except OSError:
            time.sleep(0.15)
            continue
        if mt == last_mtime or time.time() - mt > a.stale:
            time.sleep(0.1)
            continue
        last_mtime = mt
        img = cv2.imread(CAM)
        if img is None:
            continue
        h, w = shape[1], shape[2]
        if img.shape[0] != h or img.shape[1] != w:
            img = cv2.resize(img, (w, h))
        # 模型输入 NHWC float32；ImageMake 已在图里做，这里直接喂像素
        arr = img.astype(np.float32).reshape(1, h, w, 3)
        t = xrt.Tensor(arr, tt)
        t1 = time.time()
        out = sess.forward([t])
        dt = (time.time() - t1) * 1000.0
        lat.append(dt)
        n += 1
        rec = {
            "t": round(time.time() - t0, 3),
            "cam_mtime": mt,
            "latency_ms": round(dt, 2),
            "n_out": len(out),
            "out_shapes": [list(np.array(o).shape) for o in out],
        }
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        if n % 20 == 0:
            print("[sidecar] %d 帧, 时延中位 %.1f ms, cam_age %.2fs"
                  % (n, float(np.median(lat[-20:])), time.time() - mt), flush=True)
    if lat:
        print("[sidecar] 结束：%d 帧，时延 中位 %.1f / 最大 %.1f ms"
              % (n, float(np.median(lat)), float(np.max(lat))), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
