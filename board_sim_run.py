#!/usr/bin/env python3
"""**板子全流程仿真**的调用入口：录像 → PL(视觉) → 几何帧 → PS(单应+决策) → 可视化。

    python board_sim_run.py --video data/vision/run4/screen.mp4 --frames 200
    python board_sim_run.py --video data/vision/run4/screen.mp4 --headless --frames 60
    python board_sim_run.py --video data/vision/try.mp4 --fps 30 --width 640

屏幕上左右两栏：

    左：**相机看到什么**（像素坐标）—— 原始帧 + 视觉检出的障碍/角色/缺口
    右：**决策在想什么**（世界坐标）—— 障碍、角色、瞄准点、缺口线、档位

本机每一段拿什么顶替、保真度如何，见 ``core/harness/board_sim.py`` 文件头的对照表。
一句话：**PL 用与 RTL 逐位一致的 C 参考模型，几何帧、单应、决策全是同一份代码。**

按键：``ESC`` / ``q`` 退出。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from harness.board_sim import (          # noqa: E402
    BoardSim,
    BoardView,
    RefPl,
    VideoSource,
    calibrate_from_video,
    compile_ref,
    load_calibration,
    run,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="板子全流程仿真：录像 → PL → 几何帧 → PS → 决策（带可视化）")
    src = ap.add_argument_group("取图（顶替摄像头）")
    src.add_argument("--video", type=pathlib.Path,
                     default=_ROOT / "data" / "vision" / "run4" / "screen.mp4",
                     help="录像文件（默认用 run4 的屏录）")
    src.add_argument("--frames", type=int, default=0, help="最多处理多少帧（0=全片）")
    src.add_argument("--start", type=int, default=0, help="从第几帧开始")
    src.add_argument("--stride", type=int, default=1, help="每几帧取一帧")
    src.add_argument("--width", type=int, default=640,
                     help="缩放到该宽度（0=原尺寸）。C 模型上限 640")

    ap.add_argument("--calibration", type=pathlib.Path, default=None,
                    help="标定文件；**不给就从录像里的四角标记现标**（推荐）")
    ap.add_argument("--no-auto-calibrate", action="store_true",
                    help="禁止自动标定（此时必须给 --calibration）")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="播放限速（0=全速跑，用于测吞吐）")
    ap.add_argument("--headless", action="store_true", help="不开窗，只跑链")
    ap.add_argument("--save-png", type=pathlib.Path, default=None,
                    help="把某帧画面存成 PNG（**用来验证渲染本身**，不是只证明窗口能开）")
    ap.add_argument("--save-at", type=int, default=0, help="存第几帧（默认 0）")
    ap.add_argument("--no-calibration-check", action="store_true",
                    help="关掉适配器的标定校验（**只在国际单位就是像素时用**，日志会说明）")
    ap.add_argument("--rebuild", action="store_true", help="强制重编 C 参考模型")
    ap.add_argument("--verbose", action="store_true")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    print("=" * 72)
    print("板子全流程仿真（本机）")
    print("  ① 取图   : 录像（顶替摄像头）", args.video.name)
    print("  ② PL     : C 参考模型 vision_ref.c（与 RTL 逐位一致）")
    print("  ③ 几何帧 : v3 线协议字节（与板上同一份格式）")
    print("  ④ PS 单应: core/vision/localize.to_world（同一份代码）")
    print("  ⑤ 决策   : cpu.adapters.vision_adapter → gap_avoid.plan()")
    if args.no_calibration_check:
        print("  ⚠ 已关掉标定校验：把**像素**当世界单位用，仅供早期联调")
    print("=" * 72)

    so = compile_ref(force=args.rebuild)
    print(f"C 参考模型：{so}")

    # 标定：**优先用录像里的四角标记现标**。
    # 为什么不默认拿 data/vision/calibration.json：它的 image_width 是 0（没记），
    # 实测和这段录像对不上（见 board_sim.calibrate_from_video 的文档）。
    if args.calibration is not None:
        calib = load_calibration(args.calibration)
        print(f"标定：{args.calibration}（指定文件）")
    elif args.no_auto_calibrate:
        raise SystemExit("--no-auto-calibrate 时必须给 --calibration")
    else:
        calib, rep, at = calibrate_from_video(
            args.video, process_width=args.width or 640,
            field_w=640.0, field_h=480.0)
        print(f"标定：从录像现标（第 {at} 帧，{getattr(rep, 'n_markers', '?')} 枚标记，"
              f"重投影 RMS {getattr(rep, 'rms_px', float('nan')):.3f} px）")
    print(f"      field={calib.field_width:.0f}×{calib.field_height:.0f}"
          f"  metres_per_unit={getattr(calib, 'metres_per_unit', 1.0)}")

    sim = BoardSim(pl=RefPl(so), calib=calib,
                   field_w=float(calib.field_width), field_h=float(calib.field_height),
                   require_calibration=not args.no_calibration_check)
    view = None if (args.headless and args.save_png is None) else BoardView(
        title="board sim — 左:相机 右:决策")
    if view is not None:
        view.field_w = float(calib.field_width)
        view.field_h = float(calib.field_height)

    source = VideoSource(path=args.video, width=args.width, stride=args.stride,
                         start=args.start, limit=args.frames)
    print(f"录像：{source.fps:.1f} fps")
    try:
        st = run(source, sim, view=view, fps=args.fps, verbose=args.verbose,
                 save_png=args.save_png, save_at=args.save_at)
    finally:
        source.close()
        if view is not None:
            view.close()

    s = st.summary()
    print("-" * 72)
    print(f"帧数 {s['frames']}   出决策 {s['decided']}   被拒 {s['rejected']}")
    print(f"PL 中位 {s['pl_ms_median']:.2f} ms/帧   全链中位 {s['total_ms_median']:.2f} ms/帧")
    if s["tiers"]:
        print("档位分布: " + "  ".join(f"{k}={v}" for k, v in sorted(s["tiers"].items())))
    if s["reject_reasons"]:
        print("被拒原因: " + " | ".join(s["reject_reasons"]))
    return 0 if st.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
