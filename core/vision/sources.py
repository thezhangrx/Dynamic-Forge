"""取帧源与标定：从**视频文件**或**图序目录**流式取帧，并据此标定。

从根脚本 ``vision_reconcile.py`` 下沉而来（那里 1195 行、78% 是非 main 逻辑，
违反"根目录只放调用代码"的约定）。

三条纪律（都是踩出来的，别改）
------------------------------
1. **流式，不保留帧。** 1920x1080 的 1181 帧一次性读进内存约 7.3 GB，
   实测进程被 OOM 杀掉，只留一句 ``Killed``。
2. **标定与识别必须共用同一个出口。** ``process_width`` 会缩小画面，
   而单应只对**同一个分辨率**成立；两边各缩各的会得到一张整体差一个比例的单应，
   误差看起来还很"正常"。
3. **时戳按原始序号取。** ``stride > 1`` 时若用取样下标取时戳，整条时戳序列
   会整体前移，配对照样能成但配到的是错误时刻。
"""

from __future__ import annotations

import json
import pathlib

def _load_fiducial_layout(video_or_dir: pathlib.Path):
    """读 ``fiducials.json``（录制台会写）。没有就用默认布局。

    标记的**边长与位置必须和平台画的一致**才能标定 —— 读它而不是猜。
    """
    from standard.fiducial import FiducialLayout
    for cand in (video_or_dir / "fiducials.json",
                 video_or_dir.with_suffix("") / "fiducials.json",
                 video_or_dir.parent / "fiducials.json"):
        if cand.exists():
            d = json.loads(cand.read_text())
            return FiducialLayout(field_w=float(d["field_w"]),
                                  field_h=float(d["field_h"]),
                                  marker_side=float(d["marker_side"]),
                                  margin=float(d["margin"])), cand
    return FiducialLayout(), None

def _shrink(bgr, process_width: int):
    """按 ``process_width`` 等比缩小；``0`` 表示原样。"""
    if not process_width or bgr.shape[1] <= process_width:
        return bgr
    import cv2
    s = process_width / float(bgr.shape[1])
    return cv2.resize(bgr, (int(round(bgr.shape[1] * s)), int(round(bgr.shape[0] * s))),
                      interpolation=cv2.INTER_AREA)

def _stream_frames(source: pathlib.Path, images: pathlib.Path | None, *,
                   fps: float = 30.0, stamps: list[float] | None = None,
                   max_frames: int | None = None, process_width: int = 0,
                   stride: int = 1, with_stamps: bool = False):
    """**流式**产出帧，标定和识别共用这一个出口。

    ``with_stamps=False`` -> ``(帧号, bgr)``（标定只要帧）；
    ``with_stamps=True``  -> ``(时戳, bgr)``（识别用；视频取自带 PTS，
    图序取 ``stamps[i]``）。

    为什么标定和识别必须共用同一个函数：``process_width`` 会把画面缩小，
    标定出的单应只对**同一个分辨率**成立 —— 两边各缩各的（或一边缩一边不缩）
    就会得到一张整体差一个比例的单应，而且误差看起来很正常。放在一处
    就没有"两边不一致"的机会。

    ``stride`` 隔帧取样。真值日志只有 ~28 Hz，而录屏常见 60 fps，
    多出来的帧不提供新信息，却要按帧付识别开销（实测 1080p 每帧 ~0.84s）。
    """
    import cv2

    if images is not None:
        files = sorted(p for p in images.iterdir()
                       if p.suffix.lower() in (".jpg", ".jpeg", ".png")
                       and p.name not in ("marker.jpg", "probe.jpg"))
        picked = files[::max(1, stride)]
        if max_frames:
            picked = picked[:int(max_frames)]
        for i, f in enumerate(picked):
            b = cv2.imread(str(f), cv2.IMREAD_COLOR)
            if b is None:
                continue
            # 时戳要按**原始序号**取（取样下标 i 对应的原下标是 i*stride），
            # 否则 stride>1 时整条时戳序列会整体前移。
            j = i * max(1, stride)
            stamp = stamps[j] if (stamps is not None and j < len(stamps)) else float(j)
            yield (stamp, _shrink(b, process_width)) if with_stamps else (j, _shrink(b, process_width))
        return

    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {source}")
    try:
        i = 0          # 已读原始帧数
        kept = 0       # 已产出帧数
        while True:
            if max_frames is not None and kept >= int(max_frames):
                break
            ok, b = cap.read()
            if not ok or b is None:
                break
            if i % max(1, stride) == 0:
                if with_stamps:
                    # 逐帧时戳取视频自带的 PTS。拿不到才退回 fps —— 声明 fps 与实际
                    # 采集率不符是踩过的坑（clip.avi 声明 25 实际 8.6，速度全错 2.9 倍）。
                    ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                    yield (ms / 1000.0 if ms and ms > 0 else i / fps), _shrink(b, process_width)
                else:
                    yield (i, _shrink(b, process_width))
                kept += 1
            i += 1
    finally:
        cap.release()

def _iter_probe_frames(source: pathlib.Path, images: pathlib.Path | None, limit: int,
                       *, process_width: int = 0, stride: int = 1):
    """标定用的前 ``limit`` 帧（``(帧号, bgr)``）。见 :func:`_stream_frames`。"""
    for item in _stream_frames(source, images, max_frames=limit,
                               process_width=process_width, stride=stride):
        yield item

def _calibrate_from_frames(source: pathlib.Path, layout, *, images: pathlib.Path | None = None,
                           scan_frames: int = 15, search_width: int = 1600,
                           process_width: int = 0, stride: int = 1):
    """扫开头若干帧，找到四角标记并标定。

    标记**全程可见**（只占场地约 5%），所以不必像原来那块洋红矩形那样
    只挑"开头几秒"那一段 —— 随便一帧就行。这里扫前几帧只是为了跳过
    可能存在的黑场/淡入。

    ``images`` 给了就从图序里扫（录制台的产物），否则从 ``source`` 视频里扫。

    返回 ``(标定, 质量报告, 帧号)``；一帧都没找到抛 ``SystemExit``。
    """
    from standard.fiducial import FiducialLayout  # noqa: F401
    from vision.fiducial import calibrate_from_fiducials, detect_markers
    from vision.primitives import Frame
    import cv2      # 下面降采样要用 —— _iter_probe_frames 里的 import 只在它自己作用域里

    best = None
    for idx, b in _iter_probe_frames(source, images, max(1, scan_frames),
                                     process_width=process_width, stride=stride):
        h0, w0 = b.shape[:2]
        scale = 1.0 if w0 <= search_width else search_width / float(w0)
        small = b if scale == 1.0 else cv2.resize(
            b, (int(w0 * scale), int(h0 * scale)), interpolation=cv2.INTER_AREA)
        fr = Frame(small.shape[1], small.shape[0], small[:, :, 2].tobytes(),
                   small[:, :, 1].tobytes(), small[:, :, 0].tobytes())
        obs = detect_markers(fr)
        if not obs:
            continue
        # 角点从降采样图放大回原分辨率（单应是尺度协变的，放大后标定等价）
        up = [type(o)(id=o.id,
                      corners=tuple((x / scale, y / scale) for x, y in o.corners))
              for o in obs]
        try:
            calib, rep = calibrate_from_fiducials(up, layout, (w0, h0))
        except Exception:                       # noqa: BLE001 - 标记不够就继续扫
            continue
        if best is None or rep.rms_px < best[1].rms_px:
            best = (calib, rep, idx)
        if rep.n_markers == 4 and rep.rms_px < 1.0:
            break                                # 够好就不扫了
    if best is None:
        raise SystemExit(
            f"开头 {scan_frames} 帧里找不到四角标定标记 —— 没有它就换算不成场地单位，"
            "对账做不了。\n"
            "  平台侧要画标记：用 vision_capture_rig.py 开平台（默认就画）。\n"
            "  标记在**场地四角**、黑白、很小（约占场地面积 5%），"
            "并且需要有对比度才能被解出。")
    return best

def _read_truth_log(path: pathlib.Path):
    """读平台状态日志 -> ``list[FrameTruth]``。

    支持两种形状（都能带缺口真值）：

    **① JSONL**（推荐，可流式；平台侧一边跑一边追加）::

        {"stamp": 1.234, "player": {"x": 320, "y": 70, "radius": 10},
         "gaps": [{"cx": 180, "width": 60, "y": 400}]}

    **② JSON 数组**（最简，只有角色位置 —— ``data/vision/seq_truth.json`` 就是这个）::

        [[0.0083, [320.0, 70.3]], [0.0167, [320.0, 68.7]], ...]

    形状 ②没有半径也没有缺口，于是这两项会被如实报成"真值缺失"，
    **不会**被填上 0（编一个真值出来比缺更危险）。
    """
    from vision.reconcile import FrameTruth

    text = path.read_text(encoding="utf-8", errors="replace").strip()
    if not text:
        raise SystemExit(f"状态日志是空的：{path}")
    # 平台自己的**启动日志**（bullet_sim debug log）经常被当成"那个 log"递过来。
    # 它只有开跑/结束时间与总步数，**没有逐帧真值** —— 对账需要每帧的角色与缺口。
    # 这里必须说清楚，否则报出来的是一个看不懂的 JSONDecodeError。
    if not text.startswith(("[", "{")):
        first = text.splitlines()[0] if text.splitlines() else ""
        raise SystemExit(
            f"这不是逐帧状态日志，无法对账：{path}\n"
            f"  首行是: {first[:70]!r}\n"
            "  平台自带的 debug_*.log 是**启动日志**，只有开跑/结束时间与总步数，"
            "没有逐帧真值。\n"
            "  逐帧状态日志这样产（平台侧，一条命令；会自动带标定参考物）:\n"
            "    python vision_capture_rig.py --out data/vision/s1 --no-camera "
            "--seconds 60 --fullscreen\n"
            "  它写出 data/vision/s1/truth.jsonl —— 把那个文件和你的录像一起给我。\n"
            "  格式：JSONL，每行 {\"stamp\", \"wall\", \"marker\", "
            "\"player\":{x,y,radius}, \"gaps\":[{cx,width,y}]}")

    out = []
    if text.startswith("["):
        data = json.loads(text)
        for k, item in enumerate(data):
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise SystemExit(f"{path}: 第 {k} 项不是 [stamp, [x, y]]：{item!r}")
            stamp = float(item[0])
            xy = item[1]
            out.append(FrameTruth(stamp=stamp,
                                  player=(float(xy[0]), float(xy[1]), float("nan")),
                                  gaps=()))
        return out

    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rec = json.loads(line)
        pl = rec.get("player")
        player = (None if pl is None else
                  (float(pl["x"]), float(pl["y"]),
                   float(pl.get("radius", float("nan")))))
        gaps = tuple((float(g["cx"]), float(g["width"]), float(g.get("y", float("nan"))))
                     for g in rec.get("gaps", ()))
        out.append(FrameTruth(stamp=float(rec["stamp"]), player=player, gaps=gaps))
    if not out:
        raise SystemExit(f"{path}: 一行都没读出来")
    return out

def _load_calibration(args):
    """标定来源二选一：现成的 JSON，或一张带参考物的照片现场标定。"""
    from vision.calibration import (FieldCalibration, Marker,
                                    calibrate_from_marker, detect_marker_quad)
    from vision.primitives import Frame
    import cv2

    if args.calibration is not None:
        cal = FieldCalibration.load(args.calibration)
        if cal.image_width and cal.image_height:
            pass
        print(f"标定   : {args.calibration}（unit={cal.unit} "
              f"覆盖率={cal.marker_coverage:.2f}）")
        if cal.image_width == 0:
            print("        ⚠ 这份标定没记 image_width/height —— "
                  "不是 vision_calibrate.py 产出的，可能是手搓的残留")
        return cal

    if args.marker_image is None:
        raise SystemExit("真实数据模式需要 --calibration 或 --marker-image 之一")
    b = cv2.imread(str(args.marker_image), cv2.IMREAD_COLOR)
    if b is None:
        raise SystemExit(f"读不到参考物照片 {args.marker_image}")
    fr = Frame(b.shape[1], b.shape[0], b[:, :, 2].tobytes(),
               b[:, :, 1].tobytes(), b[:, :, 0].tobytes())
    quad = detect_marker_quad(fr, colour=args.marker_color)
    if quad is None:
        raise SystemExit(f"在 {args.marker_image} 里没找到参考物（颜色 {args.marker_color}）")
    mw, mh = args.marker
    cal = calibrate_from_marker(quad, Marker("m", mw, mh),
                               image_size=(fr.width, fr.height),
                               field_size=(args.field[0], args.field[1]),
                               unit="field")
    print(f"标定   : 由 {args.marker_image.name} 现场标定  参考物 {mw:g}x{mh:g} "
          f"覆盖率={cal.marker_coverage:.2f}  rms={cal.rms_px:.3g}")
    # 自检：参考物四角映射回场地，应当正好是 (0,0)..(mw,mh)。
    # 注意 rms_px **结构性恒为 0**（H 就是用这四个点解的），所以这个自检才是有效信息。
    print("        自检（四角 -> 场地）: " + "  ".join(
        f"({cal.image_to_field(x, y)[0]:.0f},{cal.image_to_field(x, y)[1]:.0f})"
        for x, y in quad))
    return cal
