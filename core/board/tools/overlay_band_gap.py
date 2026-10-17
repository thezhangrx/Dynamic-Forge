#!/usr/bin/env python
"""把 `band_gap` 判出来的**墙带**和**缺口**画到真机位帧上，输出 PNG。

为什么要这个：闭环里板子报的门离真值缺口 200 场地单位、`0/21` 落在真缺口附近，
光看数字已经无法判断那个"暖色缝"到底是什么（网格线？角标留白？镜头辉光？
还是换算算串了）。一张叠加图能一次定案，而且完全不需要相机/板子在场。

用法：
    g++ -std=c++17 -O2 -I core/board -o /tmp/bg core/board/tools/band_gap_probe.cpp
    /tmp/bg frame.ppm                      # 看它报的门
    Env/bin/python core/board/tools/overlay_band_gap.py frame.ppm out.png /tmp/bg
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw


def main(argv) -> int:
    if len(argv) < 4:
        print(__doc__)
        return 2
    frame_path, out_path, probe = Path(argv[1]), Path(argv[2]), argv[3]
    out = subprocess.run([probe, str(frame_path)], capture_output=True, text=True, check=True).stdout
    m = re.search(r"found=(\d) bands=(\d) y0=(-?\d+) y1=(-?\d+) x0=(-?\d+) x1=(-?\d+) "
                  r"cx=(-?\d+) w=(-?\d+) support=([0-9.]+) snr=([0-9.]+)", out)
    ext = re.search(r"extent ok=(\d) x=(-?\d+)\.\.(-?\d+)", out)
    if not m:
        print(f"探针没给出门：{out.strip()}")
        return 1
    found, nb, y0, y1, x0, x1, cx, w, sup, snr = (
        int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)),
        int(m.group(5)), int(m.group(6)), int(m.group(7)), int(m.group(8)),
        float(m.group(9)), float(m.group(10)))
    img = Image.open(frame_path).convert("RGB")
    d = ImageDraw.Draw(img)
    if ext and int(ext.group(1)):
        d.rectangle([int(ext.group(2)), 0, int(ext.group(3)), img.height - 1],
                    outline=(0, 255, 255), width=1)
        d.text((int(ext.group(2)) + 2, 2), f"field {ext.group(2)}..{ext.group(3)}", fill=(0, 255, 255))
    if found:
        d.rectangle([0, y0, img.width - 1, y1], outline=(255, 0, 0), width=2)     # 墙带
        d.rectangle([x0, y0, x1, y1], outline=(0, 255, 0), width=2)               # 报出的缺口
        d.line([cx, max(0, y0 - 12), cx, min(img.height - 1, y1 + 12)], fill=(0, 255, 0), width=1)
        d.text((4, max(0, y0 - 14)), f"gap x={x0}..{x1} w={w} sup={sup:.2f} snr={snr:.0f}",
               fill=(0, 255, 0))
    img.save(out_path)
    print(f"{frame_path.name}: found={found} bands={nb} 带 y={y0}..{y1} 缺口 x={x0}..{x1} "
          f"w={w} sup={sup:.2f} → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
