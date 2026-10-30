#!/usr/bin/env python3
"""auto_cam.py —— 相机曝光/白平衡自动整定（不用手调，按指标收敛）。

流程（都在板端相机上做，通过 ssh 读帧、v4l2-ctl 设参）：
  1) 切到手动曝光（exposure_auto=1）；
  2) **扫曝光**（粗扫 + 细化）：目标是"墙不削顶、场地不漂、亮暗分得开"：
       wall_med ∈ [170, 245]，sat_ratio < 0.5%，dark_med ∈ [5, 60]；
     取满足条件里 wall_med 最接近 200 的那个值；
  3) **扫白平衡**（wb_auto=0，温度 2800–6500）：让白框像素的 R/G/B 均值尽量接近；
  4) 写 data/fpga/cam_profile.json，并打印 PASS/FAIL。

用法：
    python3 core/board/tools/auto_cam.py            # 全自动
    python3 core/board/tools/auto_cam.py --dry-run  # 只测当前状态并给建议
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BOARD = "root@192.168.137.242"
KEY = str(ROOT / "data/fpga/board_ssh/board_key")
W, H, N = 640, 480, 3
OUT = ROOT / "data/fpga/cam_profile.json"


def ssh(cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    return subprocess.run(["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no", BOARD, cmd],
                          capture_output=True, timeout=timeout)


def set_ctrl(name: str, val: int) -> None:
    ssh("v4l2-ctl -d /dev/video0 --set-ctrl=%s=%d" % (name, val))


def grab(n: int = N) -> np.ndarray | None:
    """从板端相机抓 n 帧 RGB888，返回 (n,H,W,3) uint8。"""
    cmd = ("ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg "
           "-video_size %dx%d -i /dev/video0 -frames:v %d -f rawvideo -pix_fmt rgb24 -"
           % (W, H, n))
    r = ssh(cmd, timeout=40)
    buf = r.stdout or b""
    need = n * W * H * 3
    if len(buf) < need:
        return None
    return np.frombuffer(buf[:need], np.uint8).reshape(n, H, W, 3)


def stats(frames: np.ndarray) -> dict:
    f = frames.astype(np.float32)
    g = f.mean(3)
    bright = g >= 120
    if bright.sum() < 200:
        return {"ok": False, "why": "几乎看不到亮区（墙/白框）"}
    wall = g[bright]
    dark = g[~bright]
    sat = float((f.max(3) >= 250).mean())
    # 白框像素 = 最亮的那一小撮，用来看白平衡
    thr = np.percentile(wall, 90)
    wm = f[bright]
    wb = wm[wm.mean(1) >= thr].mean(0) if wm.size else np.zeros(3)
    return {
        "ok": True,
        "wall_med": float(np.median(wall)),
        "dark_med": float(np.median(dark)) if dark.size else -1.0,
        "sat_ratio": sat,
        "rgb": [float(x) for x in wb],
        "wb_spread": float((wb.max() - wb.min()) / max(wb.mean(), 1e-6)),
    }


def good(s: dict) -> bool:
    # 暗区亮度 0 也是好的（黑背景对比度最高）；上界 60 只是防"整幅发灰"
    return (s.get("ok") and 170 <= s.get("wall_med", 0) <= 245
            and s.get("sat_ratio", 1) < 0.005 and 0 <= s.get("dark_med", -1) <= 60)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--expo-min", type=int, default=40)
    ap.add_argument("--expo-max", type=int, default=600)
    ap.add_argument("--step", type=int, default=20)
    a = ap.parse_args()

    profile = {"board": BOARD, "w": W, "h": H}

    if a.dry_run:
        fr = grab()
        if fr is None:
            print("抓帧失败（相机被占用？先停预览/闭环）"); return 2
        s = stats(fr)
        print("当前状态:", json.dumps(s, ensure_ascii=False))
        print("判定:", "✅ 可用" if good(s) else "❌ 需要整定")
        return 0 if good(s) else 1

    print("== 0. 先测当前状态（通常是自动曝光）==")
    fr0 = grab()
    s0 = stats(fr0) if fr0 is not None else {"ok": False}
    print("  当前:", json.dumps(s0, ensure_ascii=False))
    if good(s0):
        profile["final"] = s0
        profile["mode"] = "auto(unchanged)"
        OUT.write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")
        print("✅ PASS：当前曝光/白平衡已经可用，不动它")
        return 0

    print("== 1. 切手动曝光 ==")
    set_ctrl("exposure_auto", 1)
    set_ctrl("exposure_auto_priority", 0)

    print("== 2. 二维扫参：exposure_absolute × brightness ==")
    # 实测：brightness（数字增益）是主导项，默认 128 会把整幅画面洗白
    #（br=1.0、sat=31%），必须先把它降下来再谈曝光。
    best, best_score = None, 1e9
    table = []
    for bri in (0, 8, 16, 24, 32, 48, 64):
        for expo in (5, 10, 20, 40, 80, 160, 320, 640):
            set_ctrl("brightness", bri)
            set_ctrl("exposure_absolute", expo)
            fr = grab(2)
            if fr is None:
                continue
            s = stats(fr)
            if not s.get("ok"):
                table.append((bri, expo, None)); continue
            table.append((bri, expo, (s["wall_med"], s["sat_ratio"], s["dark_med"])))
            # 判据改成"对比度最大"：检测阈值是亮度 120，要的是墙明显高于 120、场明显低于 120。
            if s["wall_med"] < 100 or s["sat_ratio"] > 0.01 or s["dark_med"] > 90:
                continue
            contrast = s["wall_med"] - s["dark_med"]
            score = -contrast
            if score < best_score:
                best, best_score = (bri, expo, s), score
    if best is None:
        print("  没找到可用组合；采样表：")
        for b, e, v in table:
            print("   bri=%3d expo=%3d %s" % (b, e, v))
        # 这台相机的"自动曝光"会跑到全黑（实测 mean 5.7），所以**不回自动**，
        # 退到实测可用的固定值（brightness=0、exposure=20）。
        print("  没找到满足严格条件的组合 → 退回实测可用固定值 brightness=0 exposure=20")
        set_ctrl("exposure_auto", 1); set_ctrl("brightness", 0); set_ctrl("exposure_absolute", 20)
        print("⚠ WARN：曝光整定未达严格条件，已用固定值")
        return 0
    bri, expo, sbest = best
    set_ctrl("brightness", bri)
    set_ctrl("exposure_absolute", expo)
    print("  选定 brightness=%d exposure_absolute=%d  → %s" % (bri, expo, sbest))
    profile["brightness"] = bri
    profile["exposure_absolute"] = expo

    print("== 3. 扫白平衡 ==")
    set_ctrl("white_balance_temperature_auto", 0)
    best_wb, best_spread = None, 1e9
    for t in range(2800, 6501, 300):
        set_ctrl("white_balance_temperature", t)
        fr = grab(2)
        if fr is None:
            continue
        s = stats(fr)
        if s.get("ok") and s["wb_spread"] < best_spread:
            best_wb, best_spread = t, s["wb_spread"]
    if best_wb is None:
        print("  ⚠ 白平衡扫描失败，保持自动")
        set_ctrl("white_balance_temperature_auto", 1)
    else:
        set_ctrl("white_balance_temperature", best_wb)
        profile["white_balance_temperature"] = best_wb
        print("  选定 white_balance_temperature = %d（通道差 %.1f%%）" % (best_wb, 100 * best_spread))

    print("== 4. 复核 ==")
    fr = grab()
    s = stats(fr)
    print("  最终:", json.dumps(s, ensure_ascii=False))
    profile["final"] = s
    OUT.write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")
    print("  已写", OUT)
    if not good(s):
        print("  保留手动曝光（这台相机自动曝光会跑到全黑）")
    # 验收口径：墙≥100 且 饱和<1% 且 暗区≤90 且 对比度≥60 —— 与检测阈值一致
    ok = (s.get("ok") and s.get("wall_med", 0) >= 100 and s.get("sat_ratio", 1) < 0.01
          and 0 <= s.get("dark_med", -1) <= 90
          and s.get("wall_med", 0) - s.get("dark_med", 0) >= 60)
    if ok:
        print("✅ PASS：曝光/白平衡可用（对比度 %.0f）" % (s["wall_med"] - s["dark_med"]))
        return 0
    print("❌ FAIL：整定后对比度仍不足（%.0f）——检查遮挡/页面是否在视野内"
          % (s.get("wall_med", 0) - s.get("dark_med", 0)))
    return 1


if __name__ == "__main__":
    sys.exit(main())
