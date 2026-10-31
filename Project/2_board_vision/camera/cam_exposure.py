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

实测结论（2026-10-21，页面在屏时逐档测量）：

    曝光   10     20     35     60    100    200
    中位  197    218    242    254    255    255
    p10   156    174    195    222    250    255

也就是说**手动曝光的最小档(10) 都比自动曝光收敛值亮 5 倍以上**（自动时中位≈85）。
这台相机靠手动锁曝光得不到"暗场地"，因此本项目的结论是：
**保持自动曝光（exposure_auto=3），不要去锁**；亮度变化交给 VT 的结构化判据
（墙是画面里最亮的那一簇、缺口比两侧暗），不要用绝对阈值。

本工具仍然保留，用于：
  * 判断"画面是不是过曝/全白"（中位 > 250 且墙占比 > 90% → 不可用）
  * 换相机/换屏幕后重新量一次响应曲线
结果写 `data/fpga/cam_exposure.json`（仅供参考，闭环不依赖它）。
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
    """设置曝光。**必须先清 exposure_auto_priority**，否则手动值被驱动忽略。

    实测：这个 UVC 头默认 `exposure_auto_priority=1`，此时无论把
    exposure_absolute 设成 8 还是 90，出来的画面一模一样（都跟自动曝光收敛值
    相同）——我第一次搜索就是被这个坑成"所有候选都一样亮"的。
    """
    if value is None:
        #: **恢复自动时要把 priority 交还给自动**（=1）。只设 exposure_auto=3 而
        #: priority 仍为 0，自动曝光不会真正收敛 —— 我这样把相机留在近黑状态，
        #: 后面连续两次闭环被"相机看不到页面"闸门拦下（页面其实在屏上，只是全黑）。
        ssh("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto_priority=1 2>/dev/null; "
            "v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3 2>/dev/null; "
            "v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=1 2>/dev/null; "
            "echo ok")
        return
    ssh("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto_priority=0 2>/dev/null; "
        f"v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 "
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
    ap.add_argument("--candidates", default="10,20,35,60,100,200",
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
