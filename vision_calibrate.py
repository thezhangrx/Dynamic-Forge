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
import time

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


def _platform_window() -> tuple[int, int, int, int] | None:
    """尽力问一下平台窗口的 X 几何 ``(x, y, w, h)``；拿不到返回 ``None``。

    标定只在"相机与屏幕的相对几何不变"时有效，所以把窗口几何记下来 ——
    用的时候一比就知道这份标定是不是给当前这个窗口的。
    拿不到不是错误（可能没有 X、平台没在跑），所以**不报错，只是记不上**。
    """
    import re
    import subprocess
    try:
        out = subprocess.run(["xwininfo", "-root", "-tree"],
                             capture_output=True, text=True, timeout=3).stdout
    except Exception:                       # noqa: BLE001  没有 X / 没有 xwininfo
        return None
    for line in out.splitlines():
        if "bullet_sim" in line:
            m = re.search(r"(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)", line)
            if m:
                w, h, _, _, ax, ay = (int(v) for v in m.groups())
                return (ax, ay, w, h)
    return None


def _hex_colour(text: str) -> tuple[int, int, int]:
    """``"800080"`` → ``(128, 0, 128)``，与平台的同名校验保持一致。"""
    from bullet_sim.cli import parse_hex_color
    try:
        return parse_hex_color(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _capture_bgr(args: argparse.Namespace):
    """抓一帧（延迟 import cv2 —— 不抓拍时这个脚本不需要 opencv）。"""
    from vision.capture_one_frame import capture_bgr
    return capture_bgr(args.device, args.width, args.height, args.warmup)


def _bgr_to_frame(bgr) -> "P.Frame":
    """BGR ``numpy`` 数组 → `primitives.Frame`（去掉 numpy 依赖的那一步）。"""
    import numpy as np
    a = np.ascontiguousarray(bgr)
    return P.Frame(a.shape[1], a.shape[0], a[:, :, 2].tobytes(),
                   a[:, :, 1].tobytes(), a[:, :, 0].tobytes())


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="用参考物标定场地坐标系")
    ap.add_argument("--image", type=pathlib.Path, default=None,
                    help="参考物照片（--auto 时必填）")
    ap.add_argument("--capture", action="store_true",
                    help="先现场抓一帧再标定（省掉单独跑一次抓拍）")
    ap.add_argument("--device", type=int, default=0, help="V4L2 设备号（--capture）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--warmup", type=int, default=25,
                    help="抓拍前丢弃多少帧（等自动曝光收敛）")
    ap.add_argument("--auto", action="store_true", help="自动检测参考物四角")
    ap.add_argument("--corners", type=str, default=None,
                    help='手工角点："x1,y1 x2,y2 x3,y3 x4,y4"（左下→左上，逆时针）')
    ap.add_argument("--marker", required=True, type=_parse_size,
                    help="参考物真实尺寸：默认单位是**米**（形如 0.30x0.20）；"
                         "加了 --relative 则是**占场地的比例**")
    ap.add_argument("--relative", action="store_true",
                    help="不动尺子：把 --marker 当作占场地的比例，标定结果用"
                         "**场地单位**表达（与平台的 player_diameter 等同一套单位）")
    ap.add_argument("--field", type=float, nargs=2, default=(640.0, 480.0),
                    metavar=("W", "H"), help="场地尺寸（--relative 时用来折算）")
    ap.add_argument("--name", default="marker", help="参考物名称（写进标定文件）")
    ap.add_argument("--marker-color", default="800080",
                    help="参考物颜色 RRGGBB（默认 800080 半强度洋红；与平台的 "
                         "--calib-marker-color 保持一致）")
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path("data/vision/calibration.json"))
    ap.add_argument("--warp", type=pathlib.Path, default=None,
                    help="可选：导一张俯视图用于目视确认")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    fw, fh = float(args.field[0]), float(args.field[1])
    if args.relative:
        # --marker 是"占场地的比例"，折算成场地单位（参考物 0.30x0.20 -> 192x96）
        mw, mh = args.marker[0] * fw, args.marker[1] * fh
        note = (f"世界单位 = 场地单位（场地 {fw:g}x{fh:g}）；"
                f"1 场地单位 = 屏宽/{fw:g}，等有尺子时乘 '屏宽(m)/{fw:g}' 即得米")
    else:
        mw, mh = args.marker
        note = None
    marker = Marker(args.name, mw, mh)

    frame = None
    image_size = (0, 0)
    if args.capture:
        bgr = _capture_bgr(args)
        if bgr is None:
            print(f"抓拍失败（/dev/video{args.device} 打不开或读不到帧）。"
                  "相机不可用时可以先用 --image 指定一张已有照片。", file=sys.stderr)
            return 4
        frame = _bgr_to_frame(bgr)
        image_size = (frame.width, frame.height)
        shot = args.image or (pathlib.Path("data/vision")
                              / f"calib_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
        shot.parent.mkdir(parents=True, exist_ok=True)
        image_io.save(shot, frame)
        print(f"抓拍到 {frame.width}x{frame.height} -> {shot}")
    elif args.image is not None:
        frame = image_io.load(args.image)
        image_size = (frame.width, frame.height)

    if args.corners is not None:
        quad = _parse_corners(args.corners)
        print(f"用命令行给的 4 个角点标定：{quad}")
    elif args.auto:
        if frame is None:
            print("--auto 需要同时给 --image", file=sys.stderr)
            return 2
        found = detect_marker_quad(frame, colour=_hex_colour(args.marker_color))
        if found is None:
            print(f"没检测到参考物（找的是 {args.marker_color} 的矩形）。检查："
                  "①平台用 --calib-marker 起过、画面里能看到它；"
                  "②颜色和 --marker-color 一致；"
                  "③它占画面足够大；④用 --corners 手工给点",
                  file=sys.stderr)
            return 3
        quad = list(found)
        print(f"自动检测到参考物四角：{[(round(x, 1), round(y, 1)) for x, y in quad]}")
    else:
        print("要么给 --auto（配 --image），要么给 --corners", file=sys.stderr)
        return 2

    cal = calibrate_from_marker(quad, marker, image_size=image_size, note=note,
                                field_size=(fw, fh),
                                unit="field" if args.relative else "m",
                                window=_platform_window())
    cal.validate()
    path = cal.save(args.out)

    unit = "场地单位" if args.relative else "米"
    print(f"\n参考物      : {marker.name}  {marker.width_m:g} x {marker.height_m:g} {unit}")
    print(f"世界单位    : {unit}（metres_per_unit = {cal.metres_per_unit}）")
    print(f"标定残差    : {cal.rms_px:.4f} px")
    print(f"参考物覆盖  : {cal.marker_coverage * 100:.1f}% 的场地面积")
    print(f"原点(左下)  : 图像 {tuple(round(v, 1) for v in cal.field_to_image(0.0, 0.0))}")
    if cal.window is not None:
        print(f"平台窗口    : x={cal.window[0]} y={cal.window[1]} "
              f"{cal.window[2]}x{cal.window[3]}（窗口一挪这份标定就作废）")
    if args.relative:
        print(f"场地在画面里: {cal.image_width} px 宽 == {fw:g} 场地单位")
        print(f"换算成米    : 量一次屏幕显示区的物理宽度 W 米，"
              f"则 1 场地单位 = W/{fw:g} 米（乘上去即可，标定本身不用重做）")
    if cal.marker_coverage < 0.5:
        print(f"\n⚠  参考物只占场地 {cal.marker_coverage * 100:.0f}%，单应是在**外推**：",
              file=sys.stderr)
        print("   4 个角点越集中，外推越不适定；而 4 点输入的残差恒为 0，"
              "它看不出这个问题。", file=sys.stderr)
        print("   实测 0.30x0.20（全挤在左下角）预测的场地比真实场地小一大截、"
              "透视被拉过头，", file=sys.stderr)
        print("   换成 0.90x0.90 立刻贴合。请加大参考物："
              "平台加 --calib-marker-w 0.9 --calib-marker-h 0.9。", file=sys.stderr)
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
