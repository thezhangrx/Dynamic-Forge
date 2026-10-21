#!/usr/bin/env python3
"""make_full_videos.py —— 一轮完整闭环 → 三段视频（信息来源各自独立、都在画面上标注）

产出（写到 <run_dir>/videos/）：
  1. camera_vt.mp4   **[真实相机 | 板端视觉 VT]** 左右并排 + 暂停横幅
     左：板端 /dev/video0 实拍（/tmp/cam_rec.mkv，从管线启动录到结束）
     右：板端 VT 自己输出的墙/缺口/角色（board_vt.jsonl，不含任何真值）
     横幅：该时刻主机动作=0（板端在停/暂停测量）时显示
  2. field.mp4       **用虚拟真值渲染势场**（record_field_video.py 的同一套算法，
     和 m1_board_field.mp4 一致）：热力图=视口内代价场、橙墙=真值墙、
     白线=真实轨迹、白圈=角色、绿圈=目标、HUD=act/切换/最小间隙
     —— 输入是"真值 @STATE 流"（由 configs 真值 + timeline 位姿合成），
     动作流来自板子真实发出的 @MOVE（board_io.jsonl）
  3. truth.mp4       纯虚拟真值（render_truth_video.py）

用法：
    Env/bin/python core/board/tools/make_full_videos.py <run_dir> [--speed 0.3]

约定：run_dir 里要有 board_vt.jsonl / timeline.jsonl / board_io.jsonl /
      local_truth.jsonl / truth_rows.jsonl / meta.json；相机录像从板端拉。
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BOARD = "root@192.168.137.242"
KEY = str(ROOT / "data/fpga/board_ssh/board_key")
FW, FH = 640, 480


def read_jsonl(p: Path) -> list[dict]:
    out = []
    if not p.exists():
        return out
    for line in p.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def truth_walls(cfg: dict) -> list[tuple[float, float, float]]:
    """config meta → [(y, gl, gr)] 真值每道墙的缺口左右边界。"""
    m = cfg.get("meta", {})
    out = []
    for y, g, w in zip(m.get("walls_y") or [], m.get("gaps_x") or [],
                       m.get("gap_widths") or []):
        out.append((float(y), float(g) - float(w) / 2.0, float(g) + float(w) / 2.0))
    return out


def draw_vt(rec: dict) -> np.ndarray:
    """把板端 VT 的一帧（walls/player）画成画面。坐标是页面局部（0..640/0..480）。"""
    img = np.full((FH, FW, 3), (26, 30, 40), np.uint8)
    cv2.rectangle(img, (1, 1), (FW - 2, FH - 2), (235, 235, 235), 1)
    for w in rec.get("walls") or []:
        y = float(w.get("y", 0))
        cy = int(round(FH - 1 - y))
        gl, gr = float(w.get("gl", 0)), float(w.get("gr", FW))
        for (x0, x1) in ((0, gl), (gr, FW)):
            if x1 - x0 > 1:
                cv2.line(img, (int(x0), cy), (int(x1), cy), (60, 190, 250), 3)
        cv2.putText(img, f"gap {gl:.0f}-{gr:.0f}", (int(gl), max(10, cy - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 190, 250), 1, cv2.LINE_AA)
    pl = rec.get("player")
    if pl:
        px, py = float(pl[0]), float(pl[1])
        cv2.circle(img, (int(round(px)), int(round(FH - 1 - py))), 6, (255, 255, 255), -1)
    cv2.putText(img, "board VT (camera only)", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 220, 255), 1, cv2.LINE_AA)
    return img


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="完整闭环 → camera_vt / field / truth 三段视频")
    ap.add_argument("run_dir")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="闭环 speed（用于把 sim 时间换算成墙钟时间对齐录相）")
    ap.add_argument("--out-dir", default="")
    a = ap.parse_args(argv)

    run = Path(a.run_dir)
    out = Path(a.out_dir) if a.out_dir else run / "videos"
    out.mkdir(parents=True, exist_ok=True)
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    cfg = json.loads((ROOT / meta["args"]["scenario"]).read_text(encoding="utf-8"))
    tl = read_jsonl(run / "timeline.jsonl")
    vt = read_jsonl(run / "board_vt.jsonl")
    io = read_jsonl(run / "board_io.jsonl")
    if not tl or not vt:
        print("缺 timeline.jsonl 或 board_vt.jsonl", file=sys.stderr)
        return 2

    # ---------- 1. 相机录像 ----------
    cam_path = out / "camera_raw.mp4"
    got = subprocess.run(["scp", "-q", "-o", "StrictHostKeyChecking=no",
                          "-i", KEY, f"{BOARD}:/tmp/cam_rec.mkv", str(cam_path)],
                         capture_output=True)
    if got.returncode != 0 or not cam_path.exists():
        print("⚠ 板端没有 /tmp/cam_rec.mkv（本轮没开 --cam-record）", file=sys.stderr)
        cam_path = None

    # ---------- 2. camera_vt：左相机 / 右 VT ----------
    if cam_path is not None:
        cap = cv2.VideoCapture(str(cam_path))
        n_cam = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        cam_fps_v = cap.get(cv2.CAP_PROP_FPS) or 15.0
        print(f"  相机录像 {n_cam} 帧 @{cam_fps_v:.0f}fps")
        t_sim_end = max(float(r.get("t_sim", 0)) for r in tl)
        w = cv2.VideoWriter(str(out / "camera_vt.mp4"),
                            cv2.VideoWriter_fourcc(*"mp4v"), 15.0, (FW * 2, FH))
        #: VT 帧的时间轴：用它自己的 t（板端单调时钟）比例对齐相机帧
        t0, t1 = float(vt[0]["t"]), float(vt[-1]["t"])
        act_at = {}
        for r in tl:
            act_at[round(float(r["t_sim"]), 2)] = int(r.get("act", 0) or 0)
        paused_span = []
        for r in tl:
            if int(r.get("act", 0) or 0) == 0:
                paused_span.append(float(r["t_sim"]))

        n_written = 0
        for rec in vt:
            t = float(rec.get("t", 0))
            frac = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            idx = int(round(frac * max(0, n_cam - 1)))
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, min(idx, n_cam - 1)))
            ok, frame = cap.read()
            if not ok:
                frame = np.zeros((FH, FW, 3), np.uint8)
            frame = cv2.resize(frame, (FW, FH))
            cv2.putText(frame, "camera /dev/video0 (board)", (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 255, 170), 1, cv2.LINE_AA)
            right = draw_vt(rec)
            canvas = np.hstack([frame, right])
            #: 暂停横幅：用"该 t_sim 的动作是否为 0"判断（板子在停/测量）
            t_sim_now = frac * t_sim_end
            near = min(paused_span, key=lambda v: abs(v - t_sim_now)) if paused_span else None
            if near is not None and abs(near - t_sim_now) < 1.5:
                cv2.rectangle(canvas, (0, FH - 34), (FW * 2, FH), (30, 30, 190), -1)
                cv2.putText(canvas, "PAUSE: board measuring (act=0)", (10, FH - 11),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            w.write(canvas)
            n_written += 1
        w.release()
        cap.release()
        cam_path.unlink(missing_ok=True)
        print(f"  → {out/'camera_vt.mp4'}（{n_written} 帧 @15fps，左=相机 右=VT）")

    # ---------- 3. field.mp4：**虚拟真值**渲染势场 ----------
    walls_t = truth_walls(cfg)
    fh = float(cfg.get("field_h") or 3420.0)
    spawn = cfg.get("meta", {}).get("spawn") or [320.0, 40.0]
    #: 真值 @STATE 流：每 20 Hz 一条，障碍段 = 每道真值墙的左右两段
    state_path = out / "truth_state.txt"
    act_path = out / "board_actions.txt"
    with state_path.open("w", encoding="utf-8") as f:
        last_t = -1.0
        seq = 0
        for r in tl:
            t_sim = float(r["t_sim"])
            if t_sim - last_t < 0.05:
                continue
            last_t = t_sim
            px, py, prr = float(r["px"]), float(r["py"]), 10.0
            segs = []
            for (wy, gl, gr) in walls_t:
                if abs(wy - py) > 400:      # 只发视口附近，和板子 @STATE 的量级一致
                    continue
                if gl > 2:
                    segs.append((0.0, wy, gl, 4.0))
                if gr < FW - 2:
                    segs.append((gr, wy, FW, 4.0))
            f.write(f"@STATE {seq} {px:.1f} {py:.1f} {prr:.1f} {len(segs)} " +
                    " ".join(f"{x0:.1f} {y:.1f} {x1:.1f} {h:.1f} 0.0"
                             for (x0, y, x1, h) in segs) + "\n")
            seq += 1
    with act_path.open("w", encoding="utf-8") as f:
        for rec in io:
            if rec.get("ev") in ("move_recv", "move"):
                f.write(f"@MOVE {int(rec.get('act', 0))}\n")
    if act_path.stat().st_size == 0:      # 没有 @MOVE 记录就用 timeline 的动作
        with act_path.open("w", encoding="utf-8") as f:
            prev = None
            for r in tl:
                act = int(r.get("act", 0) or 0)
                if act != prev:
                    f.write(f"@MOVE {act}\n")
                    prev = act
    goal = cfg.get("meta", {}).get("goal") or [320.0, 3360.0]
    subprocess.run([sys.executable, str(ROOT / "core/board/tools/record_field_video.py"),
                    "--state", str(state_path), "--actions", str(act_path),
                    "--out", str(out / "field.mp4"), "--field-h", str(int(fh)),
                    "--view-h", "240", "--goal", f"{goal[0]},{goal[1]}",
                    "--seconds", "600", "--fps", "20"], check=False)
    print(f"  → {out/'field.mp4'}（势场由**虚拟真值**重算；动作来自板端 @MOVE）")

    # ---------- 4. truth.mp4 ----------
    lt = run / "local_truth.jsonl"
    if lt.exists():
        fps = 2.0 if len(read_jsonl(lt)) < 200 else 20.0
        subprocess.run([sys.executable, str(ROOT / "core/board/tools/render_truth_video.py"),
                        "--state", str(lt), "--out", str(out / "truth.mp4"),
                        "--fps", f"{fps:g}", "--every", "1"], check=False)
        print(f"  → {out/'truth.mp4'}")

    print("\n== 产出 ==")
    for f in sorted(out.iterdir()):
        if f.suffix == ".mp4":
            cap = cv2.VideoCapture(str(f))
            n = int(cap.get(7) or 0)
            print(f"  {f.name:22s} {f.stat().st_size/1e6:6.1f} MB  {n} 帧")
            cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
