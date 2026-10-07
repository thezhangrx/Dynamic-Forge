#!/usr/bin/env python3
"""标定调用入口 —— 用一枚已知尺寸的参考物建立场地坐标系。

**完全不碰摄像头**：只读图片文件和/或命令行给的坐标，所以相机不在也能标定、
也能把标定结果存下来复用。

    # ① 自动检测参考物（画面里最亮的那块矩形）
    python vision_calibrate.py --image data/vision/marker.jpg \\
        --auto --marker 0.30x0.20 --out data/vision/calibration.json

    # ② 手工给 4 个角点（顺序：左下 右下 右上 左上，图像坐标）
    python vision_calibrate.py \\
        --corners "320,300 506,296 514,400 328,404" \\
        --marker 0.30x0.20 --out data/vision/calibration.json

    # ③ 顺便导一张俯视图，肉眼确认标定对不对
    python vision_calibrate.py --image ... --auto --marker 0.30x0.20 \\
        --warp data/vision/birdseye.png

`--marker` 是参考物的**真实宽×高（米）** —— 度量衡就来自这里：标完之后
"1 个世界单位 = 1 米"，缺口宽度、角色半径、距离全都变成米。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

_CORE = pathlib.Path(__file__).resolve().parent / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from vision import image_io, primitives as P  # noqa: E402
from vision.calibration import (  # noqa: E402
    Marker,
    calibrate_from_marker,
    detect_marker_quad,
)


def _parse_size(text: str) -> tuple[float, float]:
    try:
        w_s, h_s = text.lower().replace("*", "x").split("x")
        return float(w_s), float(h_s)
    except Exception as exc:  # noqa: BLE001
        raise argparse.ArgumentTypeError(
            f"--marker 形如 0.30x0.20（米），得到 {text!r}") from exc


def _parse_corners(text: str) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for chunk in text.replace(";", " ").split():
        x_s, _, y_s = chunk.partition(",")
        pts.append((float(x_s), float(y_s)))
    if len(pts) != 4:
        raise argparse.ArgumentTypeError(
            f"--corners 需要 4 个点，得到 {len(pts)} 个")
    return pts


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="用参考物标定场地坐标系")
    ap.add_argument("--image", type=pathlib.Path, default=None,
                    help="参考物照片（--auto 时必填）")
    ap.add_argument("--auto", action="store_true", help="自动检测参考物四角")
    ap.add_argument("--corners", type=str, default=None,
                    help='手工角点："x1,y1 x2,y2 x3,y3 x4,y4"（左下→左上，逆时针）')
    ap.add_argument("--marker", required=True, type=_parse_size,
                    help="参考物真实尺寸，形如 0.30x0.20（米）")
    ap.add_argument("--name", default="marker", help="参考物名称（写进标定文件）")
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path("data/vision/calibration.json"))
    ap.add_argument("--warp", type=pathlib.Path, default=None,
                    help="可选：导一张俯视图用于目视确认")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    marker = Marker(args.name, args.marker[0], args.marker[1])

    frame = None
    image_size = (0, 0)
    if args.image is not None:
        frame = image_io.load(args.image)
        image_size = (frame.width, frame.height)

    if args.corners is not None:
        quad = _parse_corners(args.corners)
        print(f"用命令行给的 4 个角点标定：{quad}")
    elif args.auto:
        if frame is None:
            print("--auto 需要同时给 --image", file=sys.stderr)
            return 2
        found = detect_marker_quad(frame)
        if found is None:
            print("没检测到参考物。检查：①画面里有明显更亮的矩形；"
                  "②它占画面足够大（>2%）；③用 --corners 手工给点",
                  file=sys.stderr)
            return 3
        quad = list(found)
        print(f"自动检测到参考物四角：{[(round(x, 1), round(y, 1)) for x, y in quad]}")
    else:
        print("要么给 --auto（配 --image），要么给 --corners", file=sys.stderr)
        return 2

    cal = calibrate_from_marker(quad, marker, image_size=image_size)
    cal.validate()
    path = cal.save(args.out)

    print(f"\n参考物      : {marker.name}  {marker.width_m} x {marker.height_m} m")
    print(f"世界单位    : 米（metres_per_unit = {cal.metres_per_unit}）")
    print(f"标定残差    : {cal.rms_px:.4f} px   （越小越可信；>2px 建议重标）")
    print(f"原点(左下)  : 图像 {tuple(round(v, 1) for v in cal.field_to_image(0.0, 0.0))}")
    print(f"写盘        : {path}")

    if args.warp is not None and frame is not None:
        _save_birdseye(frame, cal, args.warp, marker)
    return 0


def _save_birdseye(frame: P.Frame, cal, out: pathlib.Path, marker: Marker) -> None:
    """把画面按标定压成俯视图（纯 Python 手工采样，尺寸很小，只用于目视确认）。

    取场地上一张 ``200x200`` 的栅格，每个格子反查图像位置取像素 —— 这就是
    ``warpPerspective`` 在做的事，只是这里用最近邻、且不用 OpenCV。
    """
    import cv2  # 只在导出这一张图时用
    import numpy as np

    n = 200
    world_w = marker.width_m * 2.0          # 展示参考物周边一片区域
    world_h = marker.height_m * 2.0
    img = np.zeros((n, n, 3), dtype=np.uint8)
    rgb = frame.to_rgb()
    for j in range(n):
        Y = world_h * j / (n - 1)
        for i in range(n):
            X = world_w * i / (n - 1)
            u, v = cal.field_to_image(X, Y)
            x, y = int(round(u)), int(round(v))
            if 0 <= x < frame.width and 0 <= y < frame.height:
                k = (y * frame.width + x) * 3
                img[n - 1 - j, i] = (rgb[k + 2], rgb[k + 1], rgb[k])
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"俯视图      : {out}（{n}x{n}，参考物占左下一格）")


if __name__ == "__main__":
    raise SystemExit(main())
