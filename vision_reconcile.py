"""视觉识别 vs **平台真值** 的对账 —— "识别出来的数"和"平台自己产生的数"差多少。

为什么要单独一个脚本
--------------------
`core/vision/tests/test_pipeline.py` 已经在量这件事，但它只量了**角色**的
位置/半径/速度，只有一个场景、无退化。真正决定 CPU 算法能不能用的是：

* **缺口中心 / 缺口宽度** —— `gap_avoid` 就是靠这两个数决定往哪走；
* **墙的位置** —— 决定"来不来得及"；
* 退化条件（模糊 / 噪声）下的**检出率**；
* 多个种子，避免只对一局好看。

所以这里把平台当**真值发生器**：渲染出画面 → 过一台已知单应的虚拟相机 →
（可选）加模糊/噪声 → 跑完整视觉链 → 和同一时刻的 `world.state` 逐帧对账。

纪律
----
**真值只用来打分，不参与识别。** 识别用的每一帧都只是像素；
真值在另一个循环里独立读取。偷看真值这个脚本就什么都证明不了。

单位
----
合成标定用 `unit="field"`，所以误差单位是**场地单位**（场地 640x480）。
百分数一律相对场宽 640。
"""

from __future__ import annotations

import argparse

import json

import pathlib

import statistics

import sys

_ROOT = pathlib.Path(__file__).resolve().parent

_CORE = _ROOT / "core"

if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from bullet_sim.action.types import Action  # noqa: E402

from harness.loop import (  # noqa: E402
    FIELD,
    SIM_DT,
    SyntheticCamera,
    _classifier,
    build_env,
)

from vision import to_standard  # noqa: E402

from vision.align import DEFAULT_MAX_DIFF  # noqa: E402
from vision.align import (  # noqa: E402
    _align_affine,
    _align_by_content,
)
from vision.sources import (  # noqa: E402
    _calibrate_from_frames,
    _load_calibration,
    _load_fiducial_layout,
    _read_truth_log,
    _stream_frames,
)

from vision.localize import field_roi, out_of_field, to_world  # noqa: E402

from vision.wall_with_gap import WallWithGapParams, detect  # noqa: E402

FIELD_W, FIELD_H = FIELD

def _parse_size(text: str) -> tuple[float, float]:
    """``0.9x0.9`` / ``576x432`` -> ``(576.0, 432.0)``（与 vision_calibrate 的写法一致）。"""
    try:
        w_s, h_s = str(text).lower().replace("*", "x").split("x")
        return float(w_s), float(h_s)
    except Exception as exc:  # noqa: BLE001
        raise argparse.ArgumentTypeError(f"--marker 形如 576x432，得到 {text!r}") from exc

from bullet_sim.simulator.truth import truth_gaps, truth_player  # noqa: E402,F401

def vision_gaps(frame) -> list[tuple[float, float, float, float]]:
    """从视觉结果里取**所有**缺口。语义和 :func:`truth_gaps` 一致，但只用了像素。

    ``to_standard`` 已经把每条墙的两段归成一个缺口（``GapObs``），
    所以这里直接用它的结果 —— 对账的是**整条链**，不是重算一遍几何。
    """
    by_id = {ob.id: ob for ob in frame.obstacles}
    out: list[tuple[float, float, float, float]] = []
    for g in frame.gaps:
        if g.blocked_by_type not in (None, "wall_with_gap"):
            continue
        segs = [by_id[b] for b in g.blockers if b in by_id]
        if len(segs) != 2:
            continue
        thick = sum(2.0 * float(s.half_h or 0.0) for s in segs)
        out.append((float(g.center[0]), float(g.width),
                    0.5 * (segs[0].y + segs[1].y), thick))
    return out

def run(args) -> tuple[list[dict], dict]:
    from cpu.adapters.vision_adapter import VisionWorldConfig, VisionWorldStream

    rows: list[dict] = []
    total = {"frames": 0, "player_seen": 0, "wall_seen": 0, "gap_seen": 0,
             "gap_frames": 0, "gap_unmatched": 0,
             "out_of_field": 0, "rejected": 0, "accepted": 0}

    vehicle = build_env(args.seed, args.gap_width, args.seconds,
                        args.obstacle_speed, args.gap_position, args.gap_motion)
    camera = SyntheticCamera(seed=args.seed, blur=args.blur, noise=args.noise)
    calib = camera.calibrate(vehicle)
    roi = field_roi(calib)
    classifier = _classifier(args.classifier)
    params = WallWithGapParams()
    stream = VisionWorldStream(VisionWorldConfig(field_w=FIELD_W, field_h=FIELD_H))

    steps = int(args.seconds / SIM_DT)
    every = max(1, int(round(1.0 / (args.control_hz * SIM_DT))))

    vehicle.reset(seed=int(vehicle.spec.seed))
    action = Action((0.0, 0.0), 0.0)
    k = 0
    prev_est: tuple[float, float, float] | None = None
    prev_true: tuple[float, float, float] | None = None

    for step in range(steps):
        if step % every == 0:
            frame = camera.grab(vehicle)
            if frame is None:
                vehicle.world.step(action)
                continue

            stamp = float(vehicle.world.state.timestamp)
            std = to_standard.frame_to_standard(
                detect(frame, params, roi=roi, classifier=classifier),
                seq=k, stamp=stamp)
            wf = to_world(std, calib)

            tx, ty, tr = truth_player(vehicle.world)
            tgs = truth_gaps(vehicle.world)

            row: dict = {"k": k, "stamp": stamp}
            total["frames"] += 1
            if out_of_field(wf, calib):
                total["out_of_field"] += 1

            # -- 角色 --
            pl = wf.player
            if pl is None:
                row["player_ok"] = 0
            else:
                row["player_ok"] = 1
                total["player_seen"] += 1
                ex, ey = float(pl.x), float(pl.y)
                dx, dy = ex - tx, ey - ty
                row["player_dx"], row["player_dy"] = dx, dy
                row["player_err"] = (dx * dx + dy * dy) ** 0.5
                row["player_dr"] = float(pl.radius) - tr
                # 速度：估计值用**估计位置**的差分，真值用**真值位置**的差分。
                # 两个都用同一段 dt，所以可以直接比。
                if prev_est is not None and stamp > prev_est[2]:
                    dt = stamp - prev_est[2]
                    row["v_est"] = (((ex - prev_est[0]) ** 2
                                     + (ey - prev_est[1]) ** 2) ** 0.5) / dt
                    row["v_true"] = (((tx - prev_true[0]) ** 2
                                      + (ty - prev_true[1]) ** 2) ** 0.5) / dt
                prev_est = (ex, ey, stamp)
                prev_true = (tx, ty, stamp)

            # -- 缺口：按 **y 最近** 配对。
            # 两堵墙可以同时在场上（实测 t=1.6s 起就有两堵），
            # 不配对就会拿 A 墙的估计去减 B 墙的真值，误差直接上百。
            vgs = vision_gaps(wf)
            if vgs:
                total["wall_seen"] += 1
                row["wall_ok"] = 1
                # 两堵墙可以同时在场，所以一帧可能配到多个缺口；
                # 分母用"有缺口的帧数"，不然这个百分比会超过 100%。
                total["gap_frames"] += 1
                for vg in vgs:
                    if not tgs:
                        total["gap_unmatched"] += 1
                        continue
                    tg = min(tgs, key=lambda t: abs(t[2] - vg[2]))
                    if abs(tg[2] - vg[2]) > args.gap_match_margin:
                        total["gap_unmatched"] += 1     # 对不上任何一堵真值墙
                        continue
                    total["gap_seen"] += 1
                    row["gap_cx_err"] = vg[0] - tg[0]
                    row["gap_w_err"] = vg[1] - tg[1]
                    row["wall_y_err"] = vg[2] - tg[2]
            else:
                row["wall_ok"] = 0

            # -- 决策层接缝：视觉结果能不能被 cpu.plan() 吃下去 --
            try:
                _, dec = stream.step(wf)
                if dec is None:
                    total["rejected"] += 1
                else:
                    total["accepted"] += 1
            except Exception as exc:                   # noqa: BLE001
                total["rejected"] += 1
                row["plan_error"] = f"{type(exc).__name__}: {exc}"

            rows.append(row)
            k += 1

            # 让画面里有运动、墙会靠近：用真值给一个"朝缺口走"的指令。
            # **这不是识别的一部分**，识别那一路只用上面的像素。
            if tgs:
                nearest = min(tgs, key=lambda t: abs(t[2] - 200.0))
                direction = 1.0 if nearest[0] > tx else -1.0
                action = Action((direction, 0.0), 1.0)

        vehicle.world.step(action)

    return rows, total

def _stats(values: list[float]) -> str:
    if not values:
        return "—"
    a = sorted(abs(v) for v in values)
    n = len(a)
    return f"{statistics.median(a):.2f}/{a[int(0.9 * (n - 1))]:.2f}/{a[-1]:.2f}"

def _run_real(args) -> int:
    """对一段真实录像 + 平台状态日志做对账。"""
    from vision import to_standard
    from vision.localize import field_roi, out_of_field, to_world
    from vision.reconcile import FrameEstimate, reconcile
    from vision.wall_with_gap import WallWithGapParams, detect
    from vision.primitives import Frame
    import cv2

    auto_frame_idx = None
    if (args.calibration is None and args.marker_image is None
            and (args.video is not None or args.images is not None)):
        # 四角标记**全程可见**，所以随便一帧都能标定 —— 不必像原来那块
        # 洋红矩形那样只在开头露一小段。
        layout, lpath = _load_fiducial_layout(
            args.images if args.images is not None else args.video.parent)
        print(f"标定   : 四角标记布局来自 "
              f"{lpath if lpath else '默认值（没找到 fiducials.json）'}  "
              f"边长 {layout.marker_side:g} 场地单位")
        calib, frep, auto_frame_idx = _calibrate_from_frames(
            args.video, layout, images=args.images,
            scan_frames=args.marker_scan_frames,
            process_width=args.process_width, stride=args.stride)
        print(f"        第 {auto_frame_idx} 帧 -> {frep.summary()}")
        if frep.missing:
            print(f"        ⚠ 缺 id {list(frep.missing)}：只用 {frep.n_markers} 枚标记，"
                  "精度与留出校验都会变弱", file=sys.stderr)
    else:
        calib = _load_calibration(args)
    roi = field_roi(calib)
    truth = _read_truth_log(args.log)
    print(f"真值日志: {args.log.name}  {len(truth)} 帧  "
          f"含角色 {sum(1 for f in truth if f.player)} / 含缺口 {sum(1 for f in truth if f.gaps)}")
    print(f"场地 ROI: {None if roi is None else tuple(round(v, 1) for v in roi)}")

    # -- 取帧源：图序（录制台）或视频（你自己在 Windows 录的）-------------
    #
    # **流式读取，绝不把整段素材读进内存。** 早先是先把所有帧 append 进
    # ``bgr_list`` 再统一处理，于是 1920x1080 的 1181 帧要占
    # ``1181 x 1920 x 1080 x 3 ≈ 7.3 GB`` —— 实测进程直接被 OOM 杀掉
    # （只留下一句 "Killed"，看不出原因）。40 秒的录像就能触发，不是极端情况。
    #
    # 所以：**每帧读进来、识别完、丢掉**。内存只和"估计序列"（几个 float）有关，
    # 与视频长度无关。
    seq_stamps: list[float] = []
    n_frames_src = 0
    fps_v = 30.0

    if args.video is not None:
        _cap0 = cv2.VideoCapture(str(args.video))
        if not _cap0.isOpened():
            raise SystemExit(f"打不开视频 {args.video}")
        fps_v = _cap0.get(cv2.CAP_PROP_FPS) or 30.0
        n_frames_src = int(_cap0.get(cv2.CAP_PROP_FRAME_COUNT))
        _cap0.release()
        if args.max_frames:
            n_frames_src = min(n_frames_src, int(args.max_frames))
        print(f"视频    : {args.video.name}  {n_frames_src} 帧 @ {fps_v:.1f} fps")
        print(f"        （时戳取视频 PTS；--estimate-offset 不适用，用内容对齐）")
    else:
        if args.images is None:
            raise SystemExit("需要 --images 或 --video 之一")
        # 场景帧清单：录制台会写 frames.json（**按采集顺序**）。有它就照它取，
        # 不靠文件名猜 —— 目录里还有 marker.jpg，glob 会多出一帧、与时戳对不上。
        manifest = args.images / "frames.json"
        if manifest.exists():
            names = json.loads(manifest.read_text())
            images = [args.images / n for n in names]
        else:
            images = sorted(p for p in args.images.iterdir()
                            if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
            drop = {args.marker_image.resolve()} if args.marker_image else set()
            drop |= {(args.images / n).resolve() for n in ("marker.jpg", "probe.jpg")}
            images = [p for p in images if p.resolve() not in drop]
        if not images:
            raise SystemExit(f"{args.images} 里没有图片")
        n_frames_src = len(images)
        # 混合输入守卫：真值日志比最新的场景图还旧 -> 两者不是同一次录制的产物。
        # 这种组合**照样能配上对**（时戳序列是确定性的），报告看起来正常但毫无意义
        # —— 实测踩到过一次（新图配旧真值，报出检出率 13.6%、误差 387 单位）。
        if not args.allow_stale_log:
            log_m = args.log.stat().st_mtime
            newest = max(p.stat().st_mtime for p in images)
            if log_m < newest - 1.0:
                import datetime as _dt
                fmt = lambda t: _dt.datetime.fromtimestamp(t).strftime("%H:%M:%S")  # noqa: E731
                raise SystemExit(
                    f"**真值日志与图像不是同一次录制**，已拒绝出报告。\n"
                    f"  {args.log.name} 最后修改 {fmt(log_m)}，最新场景图 {fmt(newest)}。\n"
                    "  多半是上一次录制没跑完，而新的图已经落盘。\n"
                    "  强行对账加 --allow-stale-log。")
        print(f"图像    : {args.images.name}  {len(images)} 张")

        if args.stamps is not None:
            raw = [float(v) for v in json.loads(args.stamps.read_text())]
            if len(raw) != len(images):
                raise SystemExit(f"{args.stamps}: {len(raw)} 个时戳 != {len(images)} 张图")
            seq_stamps = raw
        else:
            # 不给时戳就用真值日志的前 N 个 —— **只在锁步采图时成立**
            seq_stamps = [f.stamp for f in truth[:len(images)]]
            if len(seq_stamps) != len(images):
                raise SystemExit(
                    f"没有 --stamps 且真值日志只有 {len(truth)} 帧，无法给 {len(images)} 张图配时戳")

    # -- 时钟对齐 ---------------------------------------------------------
    # 录制台（图序）自带逐帧采集时戳，所以能用 kalibr 凸包法从
    # (仿真时刻, 相机时刻) 的锁步配对里估出仿射关系。
    # 但**视频没有这个条件**（两边时钟不同、开始时间也不同），只能靠内容对齐：
    # 视频里拍的就是平台，所以"平台日志里的缺口位置"与"视频里检出的缺口位置"
    # 在**同一个场地坐标系**下应当重合 —— 哪个时间偏移让它们最吻合就是它。
    clock = None
    est_stamps = list(seq_stamps)          # 视频模式下为空，时戳在流式循环里逐帧定
    if args.estimate_offset and args.video is None:
        from vision.align import TimeOffsetEstimator
        n_pair = min(len(truth), len(est_stamps))
        clk = TimeOffsetEstimator()
        for k in range(n_pair):
            clk.add(truth[k].stamp, est_stamps[k])
        clock = clk
        slope, offset = clk.slope_offset()
        if slope <= 1e-9:
            raise SystemExit("估出的时钟斜率 <= 0，数据不可用")
        print(f"时钟   : {clk.health()}  -> 隐含时延 {offset * 1e3:+.0f} ms")
        est_stamps = [(t - offset) / slope for t in est_stamps]

    from vision.adaptive import AdaptiveParams, classify_adaptive
    classifier = None
    if args.classifier == "adaptive":
        def classifier(fr):
            masks, _ = classify_adaptive(fr, AdaptiveParams())
            return masks

    est = []
    n_oof = 0
    n_noplayer = 0
    for k, (stamp, b) in enumerate(_stream_frames(
            args.video, args.images, fps=fps_v, stamps=est_stamps,
            max_frames=args.max_frames, process_width=args.process_width,
            stride=args.stride, with_stamps=True)):
        fr = Frame(b.shape[1], b.shape[0], b[:, :, 2].tobytes(),
                   b[:, :, 1].tobytes(), b[:, :, 0].tobytes())
        obs = detect(fr, WallWithGapParams(), name=f"f{k}", roi=roi,
                     classifier=classifier)
        wf = to_world(to_standard.frame_to_standard(obs, seq=k, stamp=stamp), calib)
        if out_of_field(wf, calib):
            n_oof += 1
        pl = wf.player
        if pl is None:
            n_noplayer += 1
        # GapObs.center 是 (cx, cy) —— **y 也要取**（早先写死成 0.0，
        # 刚体拟合因此报出"RMS 332 单位、旋转 90°"的垃圾）。
        gaps = tuple((float(g.center[0]), float(g.width), float(g.center[1]))
                     for g in wf.gaps)
        est.append(FrameEstimate(
            stamp=stamp,
            player=None if pl is None else (float(pl.x), float(pl.y), float(pl.radius)),
            gaps=gaps))
    if not est:
        raise SystemExit(f"{args.video or args.images}: 一帧都没读出来")
    if args.process_width or args.stride > 1:
        print(f"处理    : 缩放 {args.process_width or '原分辨率'}"
              + (f"  隔 {args.stride} 帧取 1" if args.stride > 1 else "")
              + f"  -> 实际识别 {len(est)} 帧")

    # -- 视频模式：内容对齐，自动求出时间偏移 -----------------------------
    align_note = None
    if args.video is not None and args.offset == 0.0:
        # 先试**仿射**（含漂移）：录屏掉帧会让录像时间轴与平台时钟有稳定速率差，
        # 此时常数 δ 对不齐整段（见 _align_affine 的实测数据）。
        # 时间足够长才估得出来；短序列自动退回常数 δ。
        a, b, rows, ok = _align_affine(truth, est, span=args.align_span, step=args.align_step)
        if ok:
            print("对齐   : 分窗内容对齐（找录像时间轴与平台时钟的关系）")
            for t_mid, d, g, p, r in rows:
                _p = f" 角色 {p:6.2f}" if p == p else " 角色 未参与"
                print(f"         t={t_mid:6.1f}s  δ={d:+.2f}s  缺口 {g:5.2f}{_p}"
                      f"  歧义比 {r:4.2f}" if r == r else
                      f"         t={t_mid:6.1f}s  δ={d:+.2f}s  缺口 {g:5.2f}{_p}")
            import dataclasses as _dc
            # 把漂移**烘进 est 的时戳**：修正后 = 原始 - δ(原始)。
            # 这样下游 pair_series 不用改（它只认常数偏移，此时偏移为 0）。
            est = [_dc.replace(e, stamp=e.stamp - (a * e.stamp + b)) for e in est]
            rate = -a * 100.0
            print(f"         -> δ(t) = {a:+.5f}·t {b:+.3f}   即录像时间轴比平台"
                  f"{'快' if a > 0 else '慢'} {abs(rate):.3f}%")
            if abs(rate) > 0.3:
                print(f"            （{abs(rate):.2f}% 不是小量：多半是录屏**掉帧**，"
                      f"而封装仍按标称帧率写均匀时戳 —— 已按速率差修正）")
            align_note = (f"录像时间轴 -> 平台时钟: δ(t)={a:+.5f}·t{b:+.3f}"
                          f"（仿射内容对齐，速率差 {rate:+.3f}%）")
            args.offset = 0.0            # 已烘进时戳
            rivalry = min((r[4] for r in rows if r[4] == r[4]), default=float("nan"))
            delta, score, pscore = float("nan"), min(r[2] for r in rows), float("nan")
        else:
            delta, score, pscore, rivalry = _align_by_content(truth, est)
            if delta is None:
                print("对齐   : **算不出来** —— 视频里的缺口与日志里的缺口对不上，"
                      "继续按 offset=0 跑", file=sys.stderr)
            else:
                args.offset = delta
                _p = (f"，角色残差中位 {pscore:.2f}" if pscore == pscore else
                      "，角色未参与（未检出或没动）")
                align_note = (f"视频时钟 -> 平台时钟: offset={delta:+.2f}s"
                              f"（内容对齐，缺口残差中位 {score:.2f} 场地单位{_p}）")
                print(f"对齐   : {align_note}")
        if align_note is not None and not ok:
            # 墙按固定周期生成 -> 只看缺口时画面周期性重复 -> 评分有等间距多重极小值。
            # 角色是**非周期**的，把它计进评分就能把隔壁周期拉开；
            # 角色没动 / 没检出时这一项失效，歧义比会暴露出来。
            import math as _math
            if _math.isfinite(rivalry):
                if rivalry < 1.5:
                    print(
                        f"警告   : 内容对齐**有歧义** —— 次优偏移的评分只差 "
                        f"{rivalry:.2f} 倍（墙的生成周期让画面重复，而角色这一项"
                        f"没能拉开）。已取全局最优 {delta:+.2f}s，但 **offset 可能"
                        f"差了一个墙周期（1.6s 的整数倍）**。\n"
                        f"         最稳的做法：录像时**把小车开起来**（角色一动，"
                        f"非周期信息就足够消歧），或同时传 --stamps。",
                        file=sys.stderr)
                else:
                    print(f"         歧义比 {rivalry:.1f}x（远离次优解，落点可信）")
        elif align_note is not None and ok:
            import math as _math
            if _math.isfinite(rivalry) and rivalry < 1.5:
                print(f"警告   : 有窗口的内容对齐**有歧义**（最优歧义比 {rivalry:.2f}），"
                      f"该窗口的 δ 可能差一个墙周期 —— 请核对上面每行的歧义比。",
                      file=sys.stderr)

    print(f"识别    : 检出玩家 {len(est) - n_noplayer}/{len(est)}  "
          f"越界帧 {n_oof}  分类层 {args.classifier}")
    print()

    rep = reconcile(truth, est, max_diff=args.max_diff, offset_truth=args.offset,
                    gap_gate=args.gap_gate, with_scale=args.with_scale)
    if clock is not None:
        rep.notes.insert(0, f"相机时钟->仿真时钟: slope={clock.slope:.6f} "
                            f"offset={clock.offset:+.4f}s（kalibr 凸包法，{len(clock.hull)} 个凸包点）")
    if align_note is not None:
        rep.notes.insert(0, align_note)
    print(rep.summary())

    if args.csv:
        import csv
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        rows = []
        for k in range(len(rep.pairing)):
            ti = rep.pairing.truth_indices[k]
            ei = rep.pairing.est_indices[k]
            t = rep.pairing.truth_at[k]
            e = est[ei]
            row = {"est_index": ei, "truth_index": ti, "stamp_est": e.stamp,
                   "truth_stamp": truth[ti].stamp, "time_diff": rep.pairing.diffs[k]}
            if t.player and e.player:
                row["player_dx"] = e.player[0] - t.player[0]
                row["player_dy"] = e.player[1] - t.player[1]
                row["player_dr"] = e.player[2] - t.player[2]
            row["n_truth_gaps"] = len(t.gaps)
            row["n_est_gaps"] = len(e.gaps)
            rows.append(row)
        keys = sorted({k for r in rows for k in r})
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n逐帧明细 -> {args.csv}")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(rep.to_dict(), indent=2, ensure_ascii=False),
                                 encoding="utf-8")
        print(f"报告 JSON -> {args.json_out}")
    return 0

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="视觉识别 vs 平台真值 对账（缺口中心/宽度、角色位置/半径、退化下的检出率）")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--gap-width", type=float, default=60.0)
    ap.add_argument("--gap-position", type=float, default=0.28)
    ap.add_argument("--gap-motion", default="static", choices=("static", "sweep"))
    ap.add_argument("--obstacle-speed", type=float, default=40.0)
    ap.add_argument("--control-hz", type=float, default=30.0)
    ap.add_argument("--blur", type=int, default=1, help="模糊核边长（1 = 不糊）")
    ap.add_argument("--noise", type=float, default=0.0, help="高斯噪声标准差（灰阶）")
    ap.add_argument("--classifier", default="absolute", choices=("absolute", "adaptive"))
    ap.add_argument("--gap-match-margin", type=float, default=25.0,
                    help="视觉缺口与真值墙按 y 配对的最大容差（场地单位）")
    ap.add_argument("--csv", type=pathlib.Path, default=None, help="逐帧明细落盘")
    real = ap.add_argument_group("真实数据模式")
    real.add_argument("--images", type=pathlib.Path,
                      help="图序目录（录制台的输出）；与 --video 二选一")
    real.add_argument("--video", type=pathlib.Path,
                      help="视频文件（你自己在 Windows 录的）。时戳取视频 PTS，"
                           "时间偏移由**内容对齐**自动求出")
    real.add_argument("--marker-scan-frames", type=int, default=15,
                      help="--video 时在开头扫多少帧找四角标记")
    real.add_argument("--max-frames", type=int, default=None,
                      help="--video 时最多读多少帧（调试用）")
    real.add_argument("--process-width", type=int, default=0, metavar="PX",
                      help="识别前把画面等比缩到这么宽（0=原分辨率）。1080p 每帧约 0.84s，"
                           "缩到 1280 约 0.35s。**标定与识别共用同一缩放**，"
                           "所以换算关系不会错；代价是亚像素精度按比例下降")
    real.add_argument("--stride", type=int, default=1, metavar="N",
                      help="每隔 N 帧取一帧。真值日志约 28Hz，录屏常为 60fps，"
                           "多出来的帧不提供新信息却要按帧付识别开销")
    real.add_argument("--align-span", type=float, default=4.0, metavar="S",
                      help="内容对齐搜索 δ 的半宽（秒）")
    real.add_argument("--align-step", type=float, default=0.02, metavar="S",
                      help="内容对齐搜索 δ 的步长（秒）")
    real.add_argument("--log", type=pathlib.Path, help="平台状态日志（JSONL 或 JSON 数组）")
    real.add_argument("--stamps", type=pathlib.Path,
                      help="逐帧采集时戳的 JSON 数组；不给就用日志前 N 个")
    real.add_argument("--calibration", type=pathlib.Path, default=None,
                      help="现成的标定 JSON")
    real.add_argument("--marker-image", type=pathlib.Path, default=None,
                      help="带参考物的照片，现场标定")
    real.add_argument("--marker", type=_parse_size, default=(576.0, 432.0),
                      help="参考物真实尺寸（场地单位），默认 0.9x0.9 的 640x480。"
                           "--video 时程序会自己在视频开头找参考物帧")
    real.add_argument("--marker-color", default="800080", help="参考物颜色 RRGGBB")
    real.add_argument("--field", type=float, nargs=2, default=(640.0, 480.0))
    real.add_argument("--max-diff", type=float, default=DEFAULT_MAX_DIFF,
                      help="配对时差硬门限（秒）")
    real.add_argument("--offset", type=float, default=0.0,
                      help="真值时钟相对估计时钟的固定偏移（秒）")
    real.add_argument("--allow-stale-log", action="store_true",
                      help="允许真值日志比图像旧（默认拒绝：那是两次录制的混合物）")
    real.add_argument("--estimate-offset", action="store_true",
                      help="用 kalibr 凸包法**估计**相机时钟到仿真时钟的仿射关系再配对。"
                           "录制台产的数据**必须开**：相机墙上时钟与仿真时钟不同源，"
                           "不开会一对都配不上")
    real.add_argument("--gap-gate", type=float, default=40.0,
                      help="缺口跨帧关联门限（场地单位）")
    real.add_argument("--with-scale", action="store_true",
                      help="Umeyama 连尺度一起对齐。**默认关**：开了会把"
                           "尺度标定错误吸收掉，标定问题看起来像识别很好")
    real.add_argument("--json-out", type=pathlib.Path, default=None)
    real.add_argument("-v", "--verbose", action="store_true", help="打印前几帧的识别结果")
    ap.add_argument("--sweep", action="store_true",
                    help="扫一遍 (模糊, 噪声) 组合，看契合度怎么退化")
    args = ap.parse_args(argv)

    if args.images is not None or args.video is not None:
        if args.log is None:
            raise SystemExit("真实数据模式需要 --log（平台状态日志）")
        return _run_real(args)

    combos = ([(1, 0.0), (3, 0.0), (5, 0.0), (1, 5.0), (3, 5.0), (5, 10.0)]
              if args.sweep else [(args.blur, args.noise)])

    print(f"对账：分类层={args.classifier}  场地={FIELD_W:.0f}x{FIELD_H:.0f}  "
          f"缺口={args.gap_width:.0f}@{args.gap_position}  "
          f"每局 {args.seconds:.0f}s × {args.episodes} 局")
    print("误差单位=场地单位，绝对值的 中位/p90/max\n")
    hdr = (f"{'模糊':>4} {'噪声':>5} | {'帧数':>5} {'角色%':>6} {'缺口%':>6} | "
           f"{'角色位置':>18} {'角色半径':>18} | {'缺口中心':>18} {'缺口宽':>18} | "
           f"{'越界':>5} {'未配对':>6}")
    print(hdr)
    print("-" * len(hdr))

    grand: list[dict] = []
    for blur, noise in combos:
        sub = argparse.Namespace(**vars(args))
        sub.blur, sub.noise = blur, noise
        all_rows: list[dict] = []
        agg = {"frames": 0, "player_seen": 0, "wall_seen": 0, "gap_seen": 0,
               "gap_frames": 0, "gap_unmatched": 0,
               "out_of_field": 0, "rejected": 0, "accepted": 0}
        for ep in range(args.episodes):
            sub.seed = args.seed + ep
            rows, tot = run(sub)
            all_rows.extend(rows)
            for key in agg:
                agg[key] += tot[key]
        n = max(1, agg["frames"])

        def col(key: str) -> str:
            return _stats([r[key] for r in all_rows if key in r])

        print(f"{blur:>4} {noise:>5.1f} | {agg['frames']:>5} "
              f"{100 * agg['player_seen'] / n:>5.0f}% "
              f"{100 * agg['gap_frames'] / n:>5.0f}% | "
              f"{col('player_err'):>18} {col('player_dr'):>18} | "
              f"{col('gap_cx_err'):>18} {col('gap_w_err'):>18} | "
              f"{agg['out_of_field']:>5} {agg['gap_unmatched']:>6}")
        for r in all_rows:
            r["blur"], r["noise"] = blur, noise
        grand.extend(all_rows)

    if grand:
        def med(key: str) -> float | None:
            v = [abs(r[key]) for r in grand if key in r]
            return statistics.median(v) if v else None

        print()
        for label, key in (("角色位置", "player_err"),
                           ("角色半径", "player_dr"),
                           ("缺口中心", "gap_cx_err"),
                           ("缺口宽度", "gap_w_err")):
            m = med(key)
            if m is None:
                print(f"{label}误差中位：一次都没检出")
                continue
            tail = (f"（真值 {args.gap_width:.0f} 场地单位）"
                    if key == "gap_w_err" else "")
            print(f"{label}误差中位 {m:6.2f} 场地单位 = {100 * m / FIELD_W:5.2f}% 场宽{tail}")

        ve = [r["v_est"] for r in grand if "v_est" in r and r.get("v_true", 0) > 1.0]
        vt = [r["v_true"] for r in grand if "v_est" in r and r.get("v_true", 0) > 1.0]
        if ve and vt:
            me, mt = statistics.median(ve), statistics.median(vt)
            print(f"角色速率  中位 估计 {me:6.1f} / 真值 {mt:6.1f} 场地单位每秒 "
                  f"→ 相对误差 {100 * (me - mt) / mt:+.1f}%")
        ok = sum(1 for r in grand if "plan_error" not in r)
        print(f"决策层接缝：{ok}/{len(grand)} 帧没有抛异常")
        bad = [r["plan_error"] for r in grand if "plan_error" in r]
        if bad:
            print(f"  首个异常：{bad[0]}")

    if args.csv:
        import csv
        keys = sorted({k for r in grand for k in r})
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(grand)
        print(f"\n逐帧明细 -> {args.csv}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())

