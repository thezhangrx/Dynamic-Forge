#!/usr/bin/env python3
"""cam_exposure.py —— 自动挑一个相机曝光值，让"暗场地 + 亮墙"的对比最大。

为什么需要
----------
这台相机的自动曝光会按"整幅画面平均灰度"收敛。我们的页面是**近黑场地(12,14,20)
+ 亮墙(255,240,200)**，平均灰度很低 → AE 一路开大 → 场地被抬到 ~160、墙还是 255，
墙/场地/缺口的对比被压平。实测后果：连续取流 40 帧里，前 10 帧（AE 未收敛）全对，
后 30 帧几乎全灭（n=0）。

所以要把曝光**锁死在一个"场地还是黑的、墙已经饱和"的值**。这个值不该手调：

    score(曝光) = 墙像素占比 + (1 - 场地中位灰度/255) 的加权
    选 score 最高、且"场地中位 < 60 且墙像素占比在 1%~25%"的候选

用法（主机侧，相机必须独占；先停预览/VT/闭环）：
    Env/bin/python core/board/tools/cam_exposure.py            # 自动搜索并写结果
    Env/bin/python core/board/tools/cam_exposure.py --apply    # 顺便锁到相机上
    Env/bin/python core/board/tools/cam_exposure.py --settle-only   # 只恢复自动曝光

结果写 `data/fpga/cam_exposure.json`；`loop_standard.sh` / `start_cam.sh` 会优先用它
（`CAM_CTRL_LOCK=1 EXPOSURE_ABS=<值>`）。
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BOARD = "root@192.168.137.242"
KEY = str(ROOT / "data/fpga/board_ssh/board_key")
OUT = ROOT / "data/fpga/cam_exposure.json"


def ssh(cmd: str, timeout: float = 40.0) -> bytes:
    res = subprocess.run(["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                          "-o", "ConnectTimeout=8", BOARD, cmd],
                         capture_output=True, timeout=timeout)
    return res.stdout or b""


def set_exposure(value: int | None) -> None:
    if value is None:
        ssh("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3 2>/dev/null; "
            "v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=1 2>/dev/null; "
            "echo ok")
        return
    ssh(f"v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 "
        f"--set-ctrl=exposure_absolute={value} 2>/dev/null; echo ok")


def grab_gray() -> bytes:
    """抓 1 帧灰度（丢弃前 3 帧，让参数生效）。"""
    out = ssh("rm -f /tmp/gray.raw; ffmpeg -hide_banner -loglevel error "
              "-f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 "
              "-frames:v 4 -f rawvideo -pix_fmt gray -y /tmp/gray.raw 2>/dev/null; "
              "cat /tmp/gray.raw", timeout=60)
    n = 640 * 480
    if len(out) < 2 * n:
        return b""
    return out[-n:]          # 最后一帧


def score(frame: bytes) -> dict:
    if not frame:
        return {"ok": False}
    lo = 0
    hist = [0] * 256
    for v in frame:
        hist[v] += 1
    total = len(frame)
    acc = 0
    med = 0
    for v in range(256):
        acc += hist[v]
        if acc >= total // 2:
            med = v
            break
    walls = sum(hist[240:]) / total
    return {"ok": True, "median": med, "wall_frac": walls,
            "score": walls * 4.0 + max(0.0, (60 - med) / 60.0)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="自动挑相机曝光（让暗场地/亮墙对比最大）")
    ap.add_argument("--apply", action="store_true", help="把选中值锁到相机上")
    ap.add_argument("--settle-only", action="store_true", help="只恢复自动曝光")
    ap.add_argument("--candidates", default="8,15,25,40,60,90",
                    help="候选曝光值（逗号分隔，粗搜即可）")
    a = ap.parse_args(argv)

    if a.settle_only:
        set_exposure(None)
        print("已恢复自动曝光")
        return 0

    cands = [int(v) for v in a.candidates.split(",") if v.strip()]
    best = None
    print(f"{'曝光':>6} {'场地中位':>8} {'墙占比':>7} {'score':>7}")
    for c in cands:
        set_exposure(c)
        s = score(grab_gray())
        if not s.get("ok"):
            print(f"{c:>6}   抓帧失败")
            continue
        print(f"{c:>6} {s['median']:>8} {s['wall_frac']*100:>6.2f}% {s['score']:>7.3f}")
        ok = s["median"] <= 60 and 0.005 <= s["wall_frac"] <= 0.25
        if ok and (best is None or s["score"] > best[1]["score"]):
            best = (c, s)
    if best is None:
        print("⚠ 没找到合适的曝光（都没有'场地还黑 + 墙饱和'的帧）", file=sys.stderr)
        set_exposure(None)
        return 2

    value, s = best
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({"exposure_absolute": value,
                               "median_luma": s["median"],
                               "wall_frac": round(s["wall_frac"], 4),
                               "candidates": cands}, indent=2) + "\n",
                   encoding="utf-8")
    print(f"选中曝光 = {value}（场地中位 {s['median']}，墙占比 {s['wall_frac']*100:.2f}%）"
          f" → {OUT}")
    if a.apply:
        set_exposure(value)
        print("已锁到相机")
    else:
        set_exposure(None)
        print("（未加 --apply：已恢复自动曝光；闭环里用 CAM_CTRL_LOCK=1 时读取该值）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
