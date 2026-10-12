#!/usr/bin/env python3
"""第 2 步：**C 参考模型 vs Python 视觉链**，在同一批真实帧上逐帧对账。

    python core/fpga/ref/compare_with_python.py \
        --stim data/fpga/stim --ref /tmp/vision_ref [--limit 50]

干什么
------
同一张 PPM 分别喂给：
  * **Python 链**（`core/vision`）——语义真值；
  * **C 参考模型**（`vision_ref.c`）——将来 RTL 的比对对手；
然后把两者的**缺口中心/宽度**与**角色位置/半径**对齐比较。

**比较在像素单位下做**：C 模型输出的是像素坐标（PL 不做单应，单应在 PS），
Python 侧取 `to_standard` 的像素输出 —— 同一坐标系的数才能相减。

为什么这个脚本值得存在
----------------------
P1 的验收标准就是它：**缺口中心误差 ≤ 0.1 场地单位**。
C 模型只要和 Python 有语义偏差（分带、边沿分支、连通域邻接、填充分数……），
这里会立刻显形，而不是等到 P2 拿 RTL 比对时才发现"到底谁错了"。
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core"))

from standard.vision.vision_frame import decode_vision_frame  # noqa: E402


#: Python 字段 → C 模型 `--set` 参数名。**只有长度/面积类会随分辨率缩放**
#: （见 WallWithGapParams.rescaled）。灰度差类（阈值）与比例类不缩放。
_SETTABLE = {
    "wall_col_min": "wall_col_min",
    "field_col_min": "field_col_min",
    "band_join_rows": "band_join_rows",
    "band_min_rows": "band_min_rows",
    "min_seg_px": "min_seg_px",
    "min_gap_px": "min_gap_px",
    "player_min_area": "player_min_area",
    "player_max_area": "player_max_area",
}


def _c_args_for(p) -> dict[str, int]:
    """把 Python 侧**缩放后**的参数转成 C 模型的 --set 参数。

    为什么必须做这一步，而不是拿 C 的默认值：Python 的 `rescaled(w, h)` 会按
    分辨率缩放长度/面积类参数（640x360 → s = 0.75：min_seg_px 8→6、
    band_min_rows 5→4、field_col_min 5→4、player_min_area 60→34 …）。
    C 模型与 RTL 的默认值是 640x480 的。不喂进去，比出来的差异**不是算法的**，
    是参数的 —— 实测 000021 那一帧因此差 2 条墙段 / 1 个缺口，喂进去就完全一致。
    """
    assert p.edge_avg == 2, (
        f"edge_avg 缩放后是 {p.edge_avg}，而 RTL 的持续台阶是**两级移位寄存器**"
        f"（结构上固定 k=2）。这个分辨率下 RTL 与 Python 不可比。")
    return {c_name: int(getattr(p, py_name)) for py_name, c_name in _SETTABLE.items()}


def _check_params_agree(effective: dict, c_args: dict[str, int]) -> None:
    """核对"Python 实际用的参数"与"喂给 C 的参数"逐项相等。

    为什么要有这一条：参数是**按分辨率缩放**的，而缩放很容易做两次或不做 ——
    实测踩过两次：C 侧没缩放（差 2 条墙段 / 1 个缺口）、Python 侧缩放两次
    （band_min_rows 从 4 变成 3，多出半条墙带）。两次都表现为"算法不一致"，
    实际是参数不一致。`detect()` 会把生效参数放进 `diagnostics["params"]`，
    直接拿它来比，就不用靠人记得缩放了几次。
    """
    for py_name, c_name in _SETTABLE.items():
        want = int(c_args[c_name])
        got = int(effective[py_name])
        assert got == want, (
            f"参数不一致：Python 实际用了 {py_name}={got}，喂给 C 的是 "
            f"{c_name}={want}。多半是缩放做了 0 次或 2 次（见本函数注释）。")


def _python_chain(bgr, seq: int = 0, stamp: float = 0.0):
    """跑 Python 视觉链，返回**像素单位的 ``VisionFrame``** 与生效参数。

    ⚠ **必须比到 `to_standard` 这一层，不能停在 `detect()`**。
    C 参考模型的产物是"转成标准结构之后"的帧；而 `detect()` 的原始墙段里
    还带着**结构性的碎片**（实测 8 px、17 px 的），它们在 Python 侧的
    `to_standard._segment_obstacle` 里会被丢掉（`DEFAULT_MIN_SEGMENT_PX = 24`
    以及 `half_w >= half_h` 的不变量）。

    早先这里停在 `detect()`，于是拿"含碎片的段数"去比"过滤后的障碍数" ——
    两侧阶段不同，一旦 Ref 侧补上过滤就会**凭空多出一堆"差异"**。
    比同一层，才是对账。

    参数走 `detect()` 内部的 `rescaled()`，所以 C 侧也要按同一套缩放后的值跑
    （见 `_c_args_for`）。
    """
    from vision.primitives import Frame
    from vision.to_standard import frame_to_standard
    from vision.wall_with_gap import WallWithGapParams, detect

    fr = Frame(bgr.shape[1], bgr.shape[0], bgr[:, :, 2].tobytes(),
               bgr[:, :, 1].tobytes(), bgr[:, :, 0].tobytes())
    # 传**未缩放**的参数：`detect()` 内部会 `rescaled()` 一次。
    # 千万别在这里先 rescaled 再传进去 —— 那就是缩放两次（0.75² = 0.5625），
    # band_min_rows 会从 4 变成 3，多出半条墙带。C 侧的 `_c_args_for` 是
    # 独立算一次的，两边各自一次才对得上。
    obs = detect(fr, WallWithGapParams(), name="cmp")
    vf = frame_to_standard(obs, seq=int(seq), stamp=float(stamp))
    return vf, obs.diagnostics["params"]


def _nearest_gap_error(py_gaps, c_gaps) -> float | None:
    """倒角距离的中位：每个 Python 缺口到最近 C 缺口的距离。

    用倒角而不是一一配对：两边条数可能不同（一个漏检、一个多检），
    强行配对会把"少一个"变成"配错了"。
    """
    if not py_gaps or not c_gaps:
        return None
    ds = []
    for (px, _pw, py) in py_gaps:
        ds.append(min(((px - cx) ** 2 + (py - cy) ** 2) ** 0.5 for (cx, _w, cy) in c_gaps))
    return statistics.median(ds)


def _nearest_angle_error(py_angled, c_obstacles) -> float | None:
    """按**位置**配对之后比较角度（弧度）。

    为什么要先配对再比：两边的记录顺序不同（C 按行带顺序吐，Python 按墙带的段序），
    直接按索引比会把"顺序不同"误报成"角度不同"。

    配对的可靠性：墙段之间相距远大于它们自身厚度，所以"位置最近的 C 记录"
    就是同一条段。

    **这一项为什么必须有**：决策的 `group_walls()` 用
    ``key = (round(rotation * 1000), ...)`` 分桶 —— 把角度的微小差异放大 1000 倍。
    实测 C（定点 atan2）与 Python（浮点 atan2）的角度差约 0.012 rad，
    桶键就从 {1,2} 变成 {−9,4,5}，**墙被分进不同的桶**，闭环里直接撞墙。
    而当时这份对账只比缺口中心/角色位置，**角度从来没进过比较**，
    所以它报的是"0.00 px，满分" —— 差异就这样漏到了闭环。
    """
    if not py_angled or not c_obstacles:
        return None
    ds = []
    for (px, py, pa) in py_angled:
        best = None
        for o in c_obstacles:
            d2 = (px - float(o.x)) ** 2 + (py - float(o.y)) ** 2
            if best is None or d2 < best[0]:
                best = (d2, o)
        if best is None:
            continue
        da = abs(pa - float(best[1].rotation))
        # 角度是周期量：绕回来取最小差
        da = min(da, abs(2.0 * math.pi - da))
        ds.append(da)
    return statistics.median(ds) if ds else None


def main(argv: list[str] | None = None) -> int:
    import cv2

    ap = argparse.ArgumentParser(description="C 参考模型 vs Python 视觉链")
    ap.add_argument("--stim", type=pathlib.Path, required=True, help="PPM/PGM 目录")
    ap.add_argument("--ref", type=pathlib.Path, required=True, help="vision_ref 可执行文件")
    ap.add_argument("--limit", type=int, default=50)
    ap.add_argument("--json-out", type=pathlib.Path, default=None)
    args = ap.parse_args(argv)

    man = args.stim / "manifest.json"
    names = ([f["file"] for f in json.loads(man.read_text())["frames"]]
             if man.exists() else sorted(p.name for p in args.stim.glob("*.p[pn]m")))
    names = names[:args.limit]
    if not names:
        raise SystemExit(f"{args.stim} 里没有 PPM/PGM")

    rows = []
    for i, name in enumerate(names):
        path = args.stim / name
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"  跳过读不到的 {name}", file=sys.stderr)
            continue
        out_bin = args.stim / (name + ".ref.bin")
        import cv2 as _cv2
        _h, _w = bgr.shape[0], bgr.shape[1]
        from vision.wall_with_gap import WallWithGapParams as _W
        c_map = _c_args_for(_W().rescaled(_w, _h))
        c_args: list[str] = []
        for k, v in c_map.items():
            c_args += ["--set", f"{k}={v}"]
        r = subprocess.run([str(args.ref), str(path), str(out_bin), *c_args],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"  参考模型失败于 {name}: {r.stderr.strip()}", file=sys.stderr)
            continue
        cf = decode_vision_frame(out_bin.read_bytes())     # 顺带验证线格式
        out_bin.unlink(missing_ok=True)

        py_vf, py_eff = _python_chain(bgr, seq=i, stamp=float(i))
        _check_params_agree(py_eff, c_map)
        c_player = None if cf.player is None else (cf.player.x, cf.player.y, cf.player.radius)
        c_gaps = [(g.center[0], g.width, g.center[1]) for g in cf.gaps]

        # 两边现在都是**标准帧**（转标准结构之后），可以直接逐项比。
        py_player = (None if py_vf.player is None
                     else (py_vf.player.x, py_vf.player.y, py_vf.player.radius))
        py_gaps = [(g.center[0], g.width, g.center[1]) for g in py_vf.gaps]
        py_obs = list(py_vf.obstacles or [])
        c_obs = list(cf.obstacles or [])
        # 角度：两边都是**弧度、同一个坐标系**，直接取最小绝对差
        py_angled = [(float(o.x), float(o.y), float(o.rotation)) for o in py_obs]

        rows.append({
            "file": name,
            "py_n_gaps": len(py_gaps), "c_n_gaps": len(c_gaps),
            "py_n_obs": len(py_obs), "c_n_obs": len(c_obs),
            "gap_err": _nearest_gap_error(py_gaps, c_gaps),
            "angle_err": _nearest_angle_error(py_angled, c_obs),
            "py_player": py_player, "c_player": c_player,
            "player_err": (None if not py_player or not c_player else
                           ((py_player[0] - c_player[0]) ** 2
                            + (py_player[1] - c_player[1]) ** 2) ** 0.5),
        })

    if not rows:
        raise SystemExit("一帧都没比成")

    n = len(rows)
    gap_errs = [r["gap_err"] for r in rows if r["gap_err"] is not None]
    pl_errs = [r["player_err"] for r in rows if r["player_err"] is not None]
    both = sum(1 for r in rows if r["py_n_gaps"] and r["c_n_gaps"])
    print(f"比较 {n} 帧（缺口两边都有检出的 {both} 帧）")
    print(f"  缺口条数: Python 合计 {sum(r['py_n_gaps'] for r in rows)}，"
          f"C 合计 {sum(r['c_n_gaps'] for r in rows)}")
    print(f"  墙段/障碍记录: Python 合计 {sum(r['py_n_obs'] for r in rows)}，"
          f"C 合计 {sum(r['c_n_obs'] for r in rows)}")
    if gap_errs:
        gap_errs_s = sorted(gap_errs)
        print(f"  缺口中心误差(像素): 中位 {statistics.median(gap_errs):.2f} "
              f"p90 {gap_errs_s[int(0.9 * len(gap_errs_s)) - 1]:.2f} max {max(gap_errs):.2f}")
    else:
        print("  缺口中心误差: 无可用对比帧")
    if pl_errs:
        print(f"  角色位置误差(像素): 中位 {statistics.median(pl_errs):.2f} max {max(pl_errs):.2f}")
    else:
        print("  角色位置误差: 无可用对比帧")

    # ---- 墙段角度（关键的一项，别再漏）-----------------------------------
    # 决策按 `round(rotation*1000)` 给墙分桶，把角度的微小差异放大 1000 倍。
    # 这里如果报大数，**闭环一定出问题** —— 见 _nearest_angle_error 的文档。
    ang_errs = [r["angle_err"] for r in rows if r["angle_err"] is not None]
    if ang_errs:
        med = statistics.median(ang_errs)
        worst = max(ang_errs)
        # 1e-3 rad 会让分桶键（×1000）动 1 档 —— 这是"能不能分到同一个桶"的量级
        flag = "  ⚠ **超过 1e-3 rad：分桶键会跨档，闭环墙分组会不同**" if med > 1e-3 else ""
        print(f"  墙段角度误差(弧度): 中位 {med:.6f} max {worst:.6f}{flag}")
        print(f"    （决策分桶键 = round(rotation×1000)；角度差 ×1000 就是桶键差）")
    else:
        print("  墙段角度误差: 无可用对比帧")

    if args.json_out:
        args.json_out.write_text(json.dumps(rows, indent=2, ensure_ascii=False),
                                 encoding="utf-8")
        print(f"  逐帧明细 -> {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
