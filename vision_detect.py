#!/usr/bin/env python3
"""图像识别调用入口 —— 对一张图跑 ``wall_with_gap`` 识别。

    python vision_detect.py                          # 识别 data/vision 里最新的一张
    python vision_detect.py --image data/vision/x.jpg
    python vision_detect.py --no-save                # 只打印，不落盘

实现全在 ``core/vision/``（纯 Python，算子可逐条翻成 Verilog）；本文件只做参数解析、
调用与落盘。第三方库（opencv）只在 ``core/vision/image_io.py`` 里用于读写图片文件。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from vision import image_io  # noqa: E402
from vision import primitives as P  # noqa: E402
from vision.wall_with_gap import WallWithGapParams, annotate, detect  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="wall_with_gap 图像识别")
    ap.add_argument("--image", type=pathlib.Path, default=None,
                    help="输入图片；默认取 data/vision 里最新的一张")
    ap.add_argument("--out-dir", type=pathlib.Path, default=None,
                    help="输出目录；默认 data/vision")
    ap.add_argument("--no-save", action="store_true", help="只打印，不写文件")
    ap.add_argument("--wall-diff", type=int, default=None, help="橙色判据 R-B 下限")
    ap.add_argument("--edge-min", type=int, default=None, help="纵向边缘阈值（抗灯反光）")
    ap.add_argument("--min-gap-px", type=int, default=None, help="小于此宽度不算缺口")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out_dir = args.out_dir or P.data_dir("vision")

    image = args.image
    if image is None:
        image = P.newest_image(out_dir)
        if image is None:
            print("data/vision 里没有图片，先用 core/vision/capture_one_frame.py 抓一张",
                  file=sys.stderr)
            return 2

    params = WallWithGapParams()
    if args.wall_diff is not None:
        params.wall_diff = args.wall_diff
    if args.edge_min is not None:
        params.edge_min = args.edge_min
    if args.min_gap_px is not None:
        params.min_gap_px = args.min_gap_px

    t0 = time.perf_counter()
    frame = image_io.load(image)
    obs = detect(frame, params, name=str(image))
    elapsed = time.perf_counter() - t0

    print(f"image      : {obs.image}  ({obs.width}x{obs.height})")
    print(f"field x    : {obs.field[0]}..{obs.field[1]}")
    if obs.player is not None:
        print(f"player     : x={obs.player.x:7.1f} y={obs.player.y:7.1f} "
              f"r={obs.player.r:5.1f}  fill={obs.player.fill:.3f}")
    else:
        print("player     : 未检出")
    if obs.target is not None:
        print(f"target     : x={obs.target.x:7.1f} y={obs.target.y:7.1f} "
              f"r={obs.target.r:5.1f}  {obs.target.w}x{obs.target.h}")
    else:
        print("target     : 未检出")
    print(f"walls      : {len(obs.walls)} 面")
    for i, w in enumerate(obs.walls, 1):
        segs = " | ".join(f"[{s.x0},{s.x1}] cy={s.cy:.0f} th={s.thickness:.0f} "
                          f"ang={s.angle_deg:+.1f}" for s in w.segments)
        print(f"  wall {i}: y={w.y0}..{w.y1}  {len(w.segments)} 段  {segs}")
        if w.gap is not None:
            print(f"           gap: x[{w.gap.cx - w.gap.width_px / 2:.0f},"
                  f"{w.gap.cx + w.gap.width_px / 2:.0f}] "
                  f"width={w.gap.width_px:.1f}px center=({w.gap.cx:.1f},{w.gap.cy:.1f}) "
                  f"axis={w.gap.axis_deg:+.1f}°")
        else:
            print("           gap: 无内部缺口")
    print(f"elapsed    : {elapsed * 1e3:.0f} ms（纯 Python，无 numpy 运算）")

    if args.no_save:
        return 0

    tag = P.stamp()
    png = out_dir / f"detect_{tag}.png"
    js = out_dir / f"detect_{tag}.json"
    image_io.save(png, annotate(frame, obs))
    js.write_text(json.dumps(obs.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nwrote {png}\nwrote {js}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
