#!/usr/bin/env python3
"""把**真实照片**导出成 640×360 的 P6 PPM，作为 RTL 协同仿真的激励。

为什么需要它
------------
`export_frames.py` 出的语料来自**录屏**（`data/vision/run4/screen.mp4`），本质是
渲染出来的干净图。它在结构上是"良性的"：段永远够长、`half_w >= half_h` 永远成立。
于是 620 帧逐位对账全绿，却漏掉了两个只在真实输入下出现的缺陷（见
`core/fpga/tests/test_ref_robustness.py` 的模块注释）：

1. 亮掩膜打满标签表 → `label = -1` 被带进 `prev[]` → `uf_find(-1)` 越界读；
2. 检测器把墙切成碎片 → 出现**短于厚度**的段 → `half_w < half_h`
   → PS 侧 `decode_vision_frame().validate()` 当场抛异常，整条链断。

两条都已在 `core/fpga/ref/vision_ref.c` 里修掉，也有 C 侧的守卫测试
（`test_ref_robustness.py`，实测 32 张里 6 张触发）。
**但 RTL 从来没有被真实照片激励过** —— 那条丢弃分支（`vision_top.v` 的
`S_SEG_FIN` / `sg_drop`）在仿真里一次都没走到。合成语料再跑一万帧也补不上，
因为它的结构决定了它产不出这种段。

本脚本产出的语料就是拿来喂 RTL 的：`core/fpga/cosim/tb_vision.cpp` 会把 RTL 的
每一帧和 C 参考模型**逐位**比对。真实照片一进去，丢弃分支、饱和分支、
溢出边角才真正被覆盖。

尺寸为什么是 640×360
--------------------
RTL 的 `WIDTH/HEIGHT` 是编译期常量（默认 640×360），测试台不做缩放 ——
喂进去的 PPM 必须**正好**是这个尺寸，否则 RTL 的地址生成会和图像错位。
C 参考模型的上限则是 640×480（`test_ref_robustness.py` 用的是那个尺寸）。
两边尺寸不同不是问题：**关键是同一张图同时喂给 RTL 和 C**，比对的是两者是否逐位一致。

缩放策略：**先等比放大到覆盖 640×360，再居中裁剪**（cover + center crop）。
不用拉伸 —— 拉伸会改变像素的长宽比，而"段短于厚度"这件事恰恰和形状有关，
拉伸会人为地制造或抹掉要测的那种段。居中裁剪保形。

用法::

    python core/fpga/ref/export_photos.py --out data/fpga/stim_photo

产物::

    <out>/000000.ppm ...        P6 640×360 激励
    <out>/manifest.json         每帧的来源文件 + sha256 前 12 位，可复现
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]


def _fit_cover(im, tw: int, th: int):
    """等比缩放到刚好覆盖 (tw, th)，再居中裁剪。保形，不拉伸。"""
    import cv2

    h, w = im.shape[:2]
    s = max(tw / w, th / h)
    nw, nh = max(tw, int(round(w * s))), max(th, int(round(h * s)))
    interp = cv2.INTER_AREA if s < 1.0 else cv2.INTER_LINEAR
    im = cv2.resize(im, (nw, nh), interpolation=interp)
    x0 = (nw - tw) // 2
    y0 = (nh - th) // 2
    return im[y0:y0 + th, x0:x0 + tw]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="导出真实照片 PPM 作为 RTL 仿真激励")
    ap.add_argument("--photos", type=pathlib.Path, default=ROOT / "data" / "vision",
                    help="照片目录（默认 data/vision）")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "data" / "fpga" / "stim_photo")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--pattern", default="*.jpg",
                    help="按 glob 选照片；默认 *.jpg（不含调试图 *.png）")
    ap.add_argument("--limit", type=int, default=0, help="最多取几张，0 = 全部")
    args = ap.parse_args(argv)

    import cv2

    photos = sorted(args.photos.glob(args.pattern))
    if not photos:
        raise SystemExit(f"{args.photos} 下没有 {args.pattern}（data/ 是 gitignore 的）")
    if args.limit:
        photos = photos[: args.limit]

    args.out.mkdir(parents=True, exist_ok=True)
    for old in args.out.glob("*.ppm"):
        old.unlink()

    manifest = []
    n = 0
    for p in photos:
        im = cv2.imread(str(p))
        if im is None:
            continue
        im = _fit_cover(im, args.width, args.height)
        q = args.out / f"{n:06d}.ppm"
        if not cv2.imwrite(str(q), im):
            raise SystemExit(f"写不出去 {q}")
        manifest.append({
            "index": n,
            "file": str(q.name),
            "source": str(p.relative_to(ROOT)) if ROOT in p.parents else str(p),
            "source_sha256_12": hashlib.sha256(p.read_bytes()).hexdigest()[:12],
            "size": [args.width, args.height],
        })
        n += 1

    (args.out / "manifest.json").write_text(
        json.dumps({"width": args.width, "height": args.height,
                    "frames": manifest}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(f"导出 {n} 张 → {args.out}（{args.width}×{args.height}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
