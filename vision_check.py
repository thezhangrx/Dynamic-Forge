"""这段录像**能不能用**？—— 在上板之前先把不可用的录像挡掉。

为什么需要它
------------
实测一段真实录像（`data/vision/test.mp4`，摄像机拍显示器）里：

* 平台把场地底画成 ``(34,40,55)``，相机收到的是 ``(80 … 254)`` —— **3.2 倍跨度**；
* 平台把墙画成 ``(240,160,48)``（``R-B = 192``），相机收到 ``(255,255,211)``
  （``R-B = 44``，**掉 77%**），另一段直接被曝成 ``(255,255,255)``（``R-B = 0``）；
* 于是墙的判据 ``(R-B)*100 > 40*R``（要 ≥102）在**两段墙上都不成立** —— 墙 0% 检出，
  缺口永远算不出来；
* 同时场地本身有一片被判成了"玩家"（``(132,144,150)``，``min=132 > 2*bg=114``）。

**根因不是背景亮，是截顶。** 色度判据对整体亮度缩放**本来是不变的**
（``(R-B)`` 和 ``R`` 同倍缩放），唯一能破坏它的是**相机把通道截到 255**。
所以判据是死的、可算的：

    场地底必须落在  ``254 / (目标最大通道 / 场地最大通道)``  以下

平台现用的配色是 墙 240 / 场上 55 —— 比值 4.4，于是**场地底必须 < 58 灰阶**
才能保证墙不被曝白。这要求相当苛刻，而且相机自动曝光在"背景变暗"时
会把场地**提亮**去补偿，反而更容易截顶。这就是为什么"把亮背景挪走"是必要的、
但**不够**。

这个脚本把上面这些算出来，逐项给 PASS / WARN / FAIL 和对应的修法。

用法
----
    python vision_check.py --video data/vision/new.mp4
    python vision_check.py --image data/vision/frame.png --calibration data/vision/calibration.json
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

# ---------------------------------------------------------------------------
# 平台画面用到的**全部**固定颜色。左边是"平台渲染值"，也就是真值。
# 每一项都注了出处 —— 改了渲染器就要跟着改这里。
# ---------------------------------------------------------------------------
from bullet_sim.render.pygame_view import WALL_COLOR  # noqa: E402

#: 名称 -> (渲染 RGB, 出处)
PALETTE: dict[str, tuple[tuple[int, int, int], str]] = {
    "场地底":   ((34, 40, 55), "pygame_view.py: field_rect 填充"),
    "场地边框": ((60, 70, 95), "pygame_view.py: field_rect 描边"),
    "网格线":   ((70, 82, 105), "pygame_view.py: _draw_world_axes"),
    "墙":       (WALL_COLOR, "pygame_view.py: WALL_COLOR（视觉链路契约）"),
    "玩家盘":   ((235, 240, 250), "pygame_view.py: 玩家填充"),
    "玩家描边": ((255, 255, 255), "pygame_view.py: 玩家描边（必须亮）"),
    "朝向线":   ((90, 110, 150), "pygame_view.py: 玩家朝向"),
    "标定参考物": ((128, 0, 128), "cli.py: --calib-marker-color 默认 800080"),
    #: 场地之外的暗背景（窗口外/暗房间）。不是平台颜色，但必须在表里，
    #: 否则它会被就近配到某个暗的平台颜色上，看起来像"平台颜色没还原"。
    "画面外暗背景": ((18, 18, 20), "非平台颜色，仅用于标注"),
}

#: 视觉侧真正依赖色度的目标（其余靠亮度）。
WALL_CHROMA_PCT = 40          # adaptive.AdaptiveParams.wall_chroma_pct
BRIGHT_RATIO_PCT = 200        # adaptive.AdaptiveParams.bright_ratio_pct


# ---------------------------------------------------------------------------
def _cosine(a: tuple[int, int, int], b: tuple[int, int, int]) -> float:
    """两个 RGB 向量的夹角余弦。**对整体亮度缩放不变** —— 正好用来配颜色。"""
    na = sum(v * v for v in a) ** 0.5
    nb = sum(v * v for v in b) ** 0.5
    if na == 0.0 or nb == 0.0:
        return 0.0
    return sum(x * y for x, y in zip(a, b)) / (na * nb)


def _luma(c: tuple[int, int, int]) -> float:
    return c[0] * 0.299 + c[1] * 0.587 + c[2] * 0.114


def _match_palette(rgb: tuple[int, int, int]) -> tuple[str, float]:
    """把实测颜色配到平台渲染色上。

    **只靠余弦相似度不行**：RGB 的余弦分不开"近灰色"之间的明暗 ——
    实测 ``(18,17,20)``（暗背景）和玩家 ``(235,240,250)`` 的余弦高达 0.9997，
    于是暗背景被配成了"玩家盘"。所以要再乘一个**亮度偏离的惩罚项**。
    """
    best, score = "?", -9.9
    lr = _luma(rgb)
    for name, (ref, _) in PALETTE.items():
        pen = abs(math.log((lr + 8.0) / (_luma(ref) + 8.0)))
        c = _cosine(rgb, ref) - 0.6 * pen
        if c > score:
            best, score = name, c
    return best, score


def _clusters(bgr, np, top: int = 8):
    """画面里最主要的几种颜色。

    平台画的是**纯色平面**（没有光照模型），所以过了相机之后画面仍然是
    "少数几个颜色 + 噪声"。按 8 级量化做直方图就够了，不需要真的聚类。
    """
    q = (bgr >> 3).astype(np.int32)
    key = q[:, :, 0] * 1024 + q[:, :, 1] * 32 + q[:, :, 2]
    vals, counts = np.unique(key.ravel(), return_counts=True)
    order = np.argsort(-counts)[:top]
    out = []
    flat = bgr.reshape(-1, 3).astype(np.int64)
    for o in order:
        sel = key.ravel() == vals[o]
        mean = flat[sel].mean(axis=0)
        # bgr -> rgb
        out.append((int(mean[2]), int(mean[1]), int(mean[0]), int(counts[o])))
    return out


def _verdict(ok: bool, warn: bool = False) -> str:
    return "PASS" if ok else ("WARN" if warn else "FAIL")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="检查一段录像/一帧图能不能用来做视觉识别")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", type=pathlib.Path)
    src.add_argument("--image", type=pathlib.Path)
    ap.add_argument("--frames", type=int, default=5, help="从视频里等间隔取几帧")
    ap.add_argument("--calibration", type=pathlib.Path, default=None)
    ap.add_argument("--device", type=int, default=0, help="改用现场抓拍（/dev/videoN）")
    ap.add_argument("--capture", action="store_true", help="不用文件，直接抓一帧")
    ap.add_argument("--classifier", default="absolute", choices=("absolute", "adaptive"))
    args = ap.parse_args(argv)

    try:
        import cv2
        import numpy as np
    except ImportError as exc:                        # noqa: BLE001
        print(f"需要 opencv-python 和 numpy：{exc}", file=sys.stderr)
        return 2

    from vision.primitives import Frame

    # -- 取帧 --------------------------------------------------------------
    frames = []
    meta = ""
    if args.capture:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        for _ in range(25):
            cap.grab()
        ok, bgr = cap.read()
        cap.release()
        if not ok:
            print(f"打不开 /dev/video{args.device}", file=sys.stderr)
            return 2
        frames.append(bgr)
        meta = f"现场抓拍 /dev/video{args.device}"
    elif args.image:
        bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"读不到图片 {args.image}", file=sys.stderr)
            return 2
        frames.append(bgr)
        meta = str(args.image)
    else:
        cap = cv2.VideoCapture(str(args.video))
        if not cap.isOpened():
            print(f"打不开视频 {args.video}", file=sys.stderr)
            return 2
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        idxs = [int(total * (i + 0.5) / args.frames) for i in range(args.frames)]
        for i in idxs:
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, bgr = cap.read()
            if ok:
                frames.append(bgr)
        cap.release()
        meta = f"{args.video}（{total} 帧 @ {fps:.1f} fps，取样 {len(frames)} 帧）"

    if not frames:
        print("一帧都没取到", file=sys.stderr)
        return 2

    h, w = frames[0].shape[:2]
    print(f"来源   : {meta}")
    print(f"分辨率 : {w}x{h}")
    print()

    # -- ① 标定参考物：没有它一切归零 --------------------------------------
    from vision.calibration import detect_marker_quad, marker_channels
    mark_ok = 0
    coverage = []
    for bgr in frames:
        f = Frame(w, h, bgr[:, :, 2].tobytes(), bgr[:, :, 1].tobytes(),
                  bgr[:, :, 0].tobytes())
        quad = detect_marker_quad(f)
        if quad is not None:
            mark_ok += 1
            xs = [p[0] for p in quad]
            ys = [p[1] for p in quad]
            coverage.append((max(xs) - min(xs)) * (max(ys) - min(ys)) / (w * h))
    print("① 标定参考物")
    print(f"   命中 {mark_ok}/{len(frames)} 帧", end="")
    if coverage:
        print(f"，画面占比 {100 * min(coverage):.1f}%~{100 * max(coverage):.1f}%")
    else:
        print()
    print(f"   -> {_verdict(mark_ok == len(frames), mark_ok > 0)}  "
          "没有参考物就没有尺度、没有 ROI，检测结果**表达不成平台单位**。")
    if mark_ok == 0:
        print("      修法二选一：(a) 平台加 `--calib-marker` 让参考物**每帧都在画面里**；"
              "\n      (b) 另拍一张**同一机位**的参考物照片去标定 —— "
              "但机位一动标定立刻失效，\n      实测吃过这个亏：整屏那张参考物照片配紧贴场地的序列，"
              "误差从 1.8 单位变成 261 单位。")
    print()

    # -- ② 截顶：色度的唯一敌人 -------------------------------------------
    clip_any = []
    field_level = []
    for bgr in frames:
        a = bgr.astype(np.int32)
        mx = a.max(axis=2)
        clip_any.append(float((mx >= 254).mean()))
        # 场地底：画面里最暗的一批"低饱和"像素，用低分位估
        lo = np.percentile(mx, 20)
        field_level.append(float(lo))
    wall_max = max(WALL_COLOR)
    field_max = max(PALETTE["场地底"][0])
    ratio = wall_max / field_max
    need = 254.0 / ratio
    clip = max(clip_any)
    lvl = max(field_level)

    print("② 曝光 / 截顶")
    print(f"   截顶像素占比（任一道 ≥254）      : {100 * clip:.2f}%")
    print(f"   场地底电平（画面最暗 20 分位）    : {lvl:.0f}")
    print(f"   墙/场地 亮度比（平台配色）        : {ratio:.2f}  "
          f"（墙最大通道 {wall_max} / 场地最大通道 {field_max}）")
    print(f"   -> 要让墙不被曝白，场地底必须 < {need:.0f} 灰阶；实测 {lvl:.0f}")
    if lvl < need and clip < 0.005:
        print(f"   -> PASS  色度有存活空间")
    elif lvl < need:
        print(f"   -> WARN  场地电平够低，但有 {100 * clip:.2f}% 的像素截顶——"
              "检查是不是眩光/反光斑")
    else:
        print(f"   -> FAIL  场地底 {lvl:.0f} ≥ {need:.0f}，墙会被曝到 255，"
              "色度（R-B）归零，**墙永远认不出来**。")
        print(f"      修法：把显示器亮度调低（让场地底落到 {need * 0.7:.0f} 以下）；"
              f"或把 {WALL_COLOR} 换成半强度暖色（如 (160,80,32)），"
              f"墙/场地比降到 {max((160, 80, 32)) / field_max:.2f}，"
              f"门槛放宽到 {254 / (max((160, 80, 32)) / field_max):.0f}")
    print()

    # -- ③ 颜色保真：相机把平台的颜色变成了什么 ---------------------------
    print("③ 颜色保真（画面主要颜色 ↔ 平台渲染值，按色向匹配）")
    print(f"   {'平台颜色':<12}{'渲染 RGB':>16}{'相机实测 RGB':>18}{'R-B 保留':>10}{'占比':>8}   判定")
    bgr0 = frames[len(frames) // 2]
    for r, g, b, cnt in _clusters(bgr0, np):
        name, score = _match_palette((r, g, b))
        ref = PALETTE[name][0]
        ref_diff = ref[0] - ref[2]
        got_diff = r - b
        keep = (got_diff / ref_diff) if ref_diff else float("nan")
        frac = cnt / (w * h)
        note = ""
        if name == "墙":
            ok = got_diff * 100 > WALL_CHROMA_PCT * r
            note = "墙判据成立" if ok else "**墙判据不成立**（色度不够）"
        elif name in ("玩家盘", "玩家描边"):
            note = "截顶" if max(r, g, b) >= 254 else "在线性区"
        print(f"   {name:<12}{str(ref):>16}{str((r, g, b)):>18}"
              f"{keep:>9.0%}{100 * frac:>7.1f}%   {note}")
    # 墙是画面里**色度最强**的东西，但面积小，进不了"按占比"的前几名。
    # 所以单独看最暖的那个像素 —— 它的 R-B 就是相机给墙留了多少色度。
    a = bgr0.astype(np.int32)
    warm = a[:, :, 2] - a[:, :, 0]           # BGR 下 R 是 [2]
    iy, ix = np.unravel_index(int(np.argmax(warm)), warm.shape)
    wr, wg, wb = (int(a[iy, ix, 2]), int(a[iy, ix, 1]), int(a[iy, ix, 0]))
    ref = PALETTE["墙"][0]
    keep = (wr - wb) / (ref[0] - ref[2])
    ok = (wr - wb) * 100 > WALL_CHROMA_PCT * wr
    print(f"   最暖像素 @({ix},{iy}) RGB=({wr},{wg},{wb})  R-B={wr - wb:+d}"
          f"（渲染 {ref[0] - ref[2]:+d}，保留 {keep:.0%}）"
          f"  -> {'墙判据成立' if ok else '**墙判据不成立**'}")
    print()

    # -- ④ 目标判定：直接跑分类器 ----------------------------------------
    from vision.primitives import classify
    from vision.adaptive import AdaptiveParams, classify_adaptive
    from vision.wall_with_gap import WallWithGapParams, detect

    bgr = bgr0
    f = Frame(w, h, bgr[:, :, 2].tobytes(), bgr[:, :, 1].tobytes(), bgr[:, :, 0].tobytes())
    if args.classifier == "adaptive":
        masks, fields = classify_adaptive(f, AdaptiveParams())
        print(f"④ 分类结果（自适应用时另计）  噪声标量={fields.noise_level} "
              f"弃权={100 * fields.unknown_frac:.1f}%")
    else:
        masks = classify(f)
        print("④ 分类结果")
    print(f"   墙={sum(masks.wall):7d}  亮={sum(masks.bright):7d}  绿={sum(masks.green):6d}"
          f"   （画面共 {w * h} 像素）")
    if sum(masks.wall) == 0:
        print("   -> FAIL  一个墙像素都没有，缺口算不出来")
    if sum(masks.bright) > 0.05 * w * h:
        print(f"   -> WARN  '亮'占画面的 {100 * sum(masks.bright) / (w * h):.0f}%"
              "——背景/场地被当成玩家了")
    if args.calibration is not None:
        from vision.calibration import FieldCalibration
        from vision.localize import field_roi
        calib = FieldCalibration.load(args.calibration)
        roi = field_roi(calib)
        obs = detect(f, WallWithGapParams(), name=str(args.video or args.image),
                     classifier=None)
        print(f"   ROI  = {roi}")
        print(f"   检出: 玩家={obs.player}  墙={len(obs.walls)}")
    else:
        print("   （加 --calibration 可以顺便看 ROI 和检出结果）")
    print()
    print("提示：③ 里 'R-B 保留' 接近 100% 才是好录像。掉到个位数说明该颜色已被曝白，"
          "阈值怎么调都没用。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
