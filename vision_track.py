#!/usr/bin/env python3
"""任务二调用入口 —— 从**视频**里取坐标、距离与速度。

链条（每一步都能单独测，接缝都是标准结构）::

    图像帧 ──detect──▶ 像素域观测 ──to_standard──▶ VisionFrame(px)
           ──to_world──▶ VisionFrame(世界单位) ──Tracker──▶ 速度

    python vision_track.py --video data/vision/clip.mp4 --calibration data/vision/calibration.json
    python vision_track.py --frames data/vision/seq --calibration ...
    python vision_track.py --capture 60 --calibration ...        # 现场抓 60 帧

输出是一行一帧的 JSONL（标准 ``VisionFrame``），所以可以直接喂给
``core/standard`` 的消费方，也可以拿 ``vision_track.py --summary`` 只看统计。

**时间戳一律取采集时刻**（``--fps`` 或视频自带 PTS），不用处理耗时 ——
dt 错了速度就全错，这是三个参考项目都翻车的地方（见 ``core/vision/motion.py``）。
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

from vision import motion  # noqa: E402
from vision import primitives as P  # noqa: E402
from vision import to_standard  # noqa: E402
from vision.calibration import FieldCalibration  # noqa: E402
from vision.localize import field_roi, out_of_field, to_world  # noqa: E402
from vision.wall_with_gap import WallWithGapParams, detect  # noqa: E402


# --------------------------------------------------------------------------
# 取帧
# --------------------------------------------------------------------------
def frames_from_video(path: pathlib.Path, *, max_frames: int | None = None):
    """逐帧产出 ``(bgr, stamp)``；时间戳取视频自带 PTS（拿不到才退回 fps）。"""
    import cv2
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    i = 0
    while max_frames is None or i < max_frames:
        ok, fr = cap.read()
        if not ok:
            break
        # CAP_PROP_POS_MSEC 就是 PTS；读不到时用 fps 推。
        ms = cap.get(cv2.CAP_PROP_POS_MSEC)
        stamp = ms / 1000.0 if ms and ms > 0 else (i / fps if fps > 0 else float(i))
        yield fr, stamp
        i += 1
    cap.release()


def frames_from_dir(path: pathlib.Path, *, fps: float, max_frames: int | None = None):
    import cv2
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files = sorted(f for f in path.iterdir() if f.suffix.lower() in exts)
    if not files:
        raise SystemExit(f"{path} 里没有图片")
    for i, f in enumerate(files[:max_frames] if max_frames else files):
        img = cv2.imread(str(f))
        if img is None:
            continue
        yield img, i / fps


def frames_from_camera(count: int, *, device: int, width: int, height: int,
                       fps_hint: float | None = None):
    """现场抓 ``count`` 帧；时间戳用**本机单调时钟**（这是真正的采集时刻）。"""
    import cv2
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"打不开 /dev/video{device}")
    for _ in range(25):
        cap.grab()
    n = 0
    while n < count:
        ok, fr = cap.read()
        if not ok:
            break
        yield fr, time.monotonic()
        n += 1
    cap.release()


# --------------------------------------------------------------------------
def _to_frame(bgr) -> P.Frame:
    import numpy as np
    a = np.ascontiguousarray(bgr)
    return P.Frame(a.shape[1], a.shape[0], a[:, :, 2].tobytes(),
                   a[:, :, 1].tobytes(), a[:, :, 0].tobytes())


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="视频 → 坐标/距离/速度（任务二）")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", type=pathlib.Path, help="视频文件")
    src.add_argument("--frames", type=pathlib.Path, help="图片序列目录")
    src.add_argument("--capture", type=int, metavar="N", help="现场抓 N 帧")
    ap.add_argument("--calibration", type=pathlib.Path,
                    default=pathlib.Path("data/vision/calibration.json"),
                    help="vision_calibrate.py 产出的标定文件（不给就输出像素单位）")
    ap.add_argument("--fps", type=float, default=30.0, help="--frames 的假定帧率")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--out", type=pathlib.Path, default=None,
                    help="JSONL 输出路径（默认只打印统计）")
    ap.add_argument("--gate", type=float, default=24.0,
                    help="关联门限，单位同输出（世界单位或像素）")
    ap.add_argument("--track", default="player",
                    choices=("player", "target", "obstacles"),
                    help="跟谁（默认角色）")
    ap.add_argument("--summary", action="store_true", help="只打印统计，不写 JSONL")
    ap.add_argument("--decide", action="store_true",
                    help="顺便接到决策层：每帧喂给 cpu.adapters.VisionWorldStream，"
                         "打印 cpu.gap_avoid 的决策（需要 --calibration）")
    return ap


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def _pick(det: dict, what: str) -> tuple[float, float] | None:
    if what == "player":
        p = det.get("player")
        return None if p is None else (float(p["x"]), float(p["y"]))
    if what == "target":
        t = det.get("target")
        return None if t is None else (float(t["x"]), float(t["y"]))
    return None


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    roi = None
    calib = None
    if args.calibration is not None and args.calibration.exists():
        calib = FieldCalibration.load(args.calibration)
        print(f"标定     : {args.calibration}（覆盖 {calib.marker_coverage * 100:.0f}% 场地）")
        roi = field_roi(calib)
        if roi is None:
            print("场地 ROI : 算不出来（标定里没有场地尺寸），在整幅画面里找目标")
        else:
            print(f"场地 ROI : ({roi[0]:.0f},{roi[1]:.0f})-({roi[2]:.0f},{roi[3]:.0f})"
                  f"  —— 只在这里面找目标")
    else:
        print(f"标定     : 无（输出像素单位）", file=sys.stderr)

    if args.video is not None:
        src = frames_from_video(args.video, max_frames=args.max_frames)
    elif args.frames is not None:
        src = frames_from_dir(args.frames, fps=args.fps, max_frames=args.max_frames)
    else:
        src = frames_from_camera(args.capture, device=args.device,
                                 width=args.width, height=args.height)

    stream = None
    if args.decide:
        # 这一句就是"视觉 → 决策"的全部接缝：核心是 vision 的标准 VisionFrame，
        # 决策层通过鸭子类型读它的字段（不反向依赖 core/vision）。
        from cpu.adapters.vision_adapter import VisionWorldConfig, VisionWorldStream
        stream = VisionWorldStream(VisionWorldConfig(field_w=640.0, field_h=480.0))

    tracker = motion.Tracker(gate=args.gate)
    out = open(args.out, "w", encoding="utf-8") if (args.out and not args.summary) else None
    n_frames = n_det = n_dec = 0
    last_action = None
    rejected = [0]
    out_warned: list[str] = []
    speeds: list[float] = []
    last: motion.Track | None = None

    try:
        for seq, (bgr, stamp) in enumerate(src):
            px_frame = _to_frame(bgr)
            obs = detect(px_frame, WallWithGapParams(), name=f"frame{seq}", roi=roi)
            std = to_standard.frame_to_standard(obs, seq=seq, stamp=float(stamp))
            world = to_world(std, calib) if calib is not None else std
            n_frames += 1

            # 标定还成立吗？错的单应不会报错，只会吐出一堆"看着像坐标"的数字，
            # 所以这里必须主动看一眼，而且**说清楚是哪儿不对**。
            if calib is not None:
                bad = out_of_field(world, calib)
                if bad:
                    out_warned.extend(bad)
                    if len(out_warned) <= len(bad):        # 只说第一次，别刷屏
                        print(f"  ⚠ 第{seq}帧越界（{len(bad)} 项）：{bad[0]}",
                              file=sys.stderr)
                        if calib.window is not None:
                            print(f"    这份标定记的是窗口 x={calib.window[0]} "
                                  f"y={calib.window[1]} {calib.window[2]}x{calib.window[3]}；"
                                  f"窗口挪过就会这样", file=sys.stderr)

            det = _pick(world.to_dict(), args.track)
            tracks = tracker.update([det] if det else [], float(stamp))
            if det:
                n_det += 1
            live = max(tracks, key=lambda t: t.hits, default=None)
            if live is not None:
                last = live
                speeds.append(live.speed)
                # 把速度写回这一帧（标准结构的 vx/vy 就是给这件事留的）
                if world.player is not None and args.track == "player":
                    world.player.vx, world.player.vy = live.vx, live.vy
                    world.player.speed = live.speed
            if stream is not None:
                try:
                    view, dec = stream.step(world)
                    n_dec += 1
                    last_action = getattr(dec, "action", None) or getattr(dec, "choice", dec)
                except Exception as exc:                      # noqa: BLE001
                    # 决策层拒绝是**有信息**的（坐标系/单位/字段不合契约），
                    # 所以要显式报出来，而不是吞掉继续跑。
                    print(f"  第{seq}帧被决策层拒绝：{type(exc).__name__}: {exc}",
                          file=sys.stderr)
                    rejected[0] += 1
            if out is not None:
                out.write(json.dumps(world.to_dict(), ensure_ascii=False) + "\n")
    finally:
        if out is not None:
            out.close()

    if out_warned:
        print(f"越界警告 : {len(out_warned)} 项 —— 标定很可能已经失效"
              f"（相机/屏幕/窗口动过），结果不可信", file=sys.stderr)
    unit = "场地单位/秒" if calib is not None else "像素/秒"
    print(f"帧数     : {n_frames}（其中 {n_det} 帧检测到{args.track}）")
    if speeds:
        print(f"速率     : 中位 {sorted(speeds)[len(speeds)//2]:.2f}  "
              f"最小 {min(speeds):.2f}  最大 {max(speeds):.2f}  {unit}")
    if last is not None:
        print(f"末帧位置 : ({last.x:.1f}, {last.y:.1f})  速度 ({last.vx:.2f}, {last.vy:.2f})  {unit}")
    if stream is not None:
        print(f"决策     : {n_dec}/{n_frames} 帧被 cpu.plan() 接受"
              + (f"，{rejected[0]} 帧被拒绝" if rejected[0] else ""))
        if last_action is not None:
            print(f"末帧决策 : {last_action}")
    if out is not None:
        print(f"写盘     : {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
