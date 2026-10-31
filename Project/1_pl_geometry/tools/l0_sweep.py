#!/usr/bin/env python3
"""l0_sweep.py —— 板端跑 L0 全量对拍（≥300 帧：PL 输出与 C 参考**逐字节一致**）

为什么单独一个脚本：L0 的判据是"每一帧都与参考逐字节相同"，单帧手工对拍只能抽查；
而且帧一多就必须有**汇总**（通过数/失败帧号/耗时分布），否则没法当证据。

跑在板子上（要 /dev/mem、/tmp/l0 下已有 visref 与 feed_pl_c）：

    python3 l0_sweep.py /tmp/l0/frames             # 目录里是 *.jpg 或 *.ppm
    python3 l0_sweep.py /tmp/l0/frames --limit 320

流程（每帧）：
    jpg → ppm（cv2，**参考与 PL 吃同一份 PPM 字节**）
        → ./visref f.ppm f.ref.bin          （C 参考）
        → ./feed_pl_c f.ppm --dump f.pl.bin （C 喂数器 → PL → 读回几何帧）
        → cmp_frame 规则：各自 CRC 自洽 + 帧体（除 seq/stamp/crc）逐字节相同

判据：**失败帧必须为 0**；同时打印耗时分布（L1 目标 ≤35 ms/帧）。
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import struct
import subprocess
import sys
import zlib

L0 = "/tmp/l0"


def need_of(b: bytes) -> int:
    return 76 + 36 * (b[12] + b[16])


def selfcheck(b: bytes, tag: str) -> bool:
    n = need_of(b)
    if len(b) < n:
        print("    %s: 长度 %d < 声明 %d" % (tag, len(b), n))
        return False
    crc_in = struct.unpack_from("<I", b, n - 4)[0]
    crc_calc = zlib.crc32(b[: n - 4]) & 0xFFFFFFFF
    if crc_in != crc_calc:
        print("    %s: 自身 CRC 不符（0x%08x / 0x%08x）" % (tag, crc_in, crc_calc))
        return False
    return True


def body_equal(a: bytes, b: bytes) -> bool:
    n = need_of(a)
    if need_of(b) != n:
        print("    帧长不同：%d / %d" % (need_of(a), need_of(b)))
        return False

    def mask(x: bytes) -> bytes:
        return x[:8] + b"\0" * 4 + x[12:24] + b"\0" * 8 + x[32 : n - 4]

    if mask(a) != mask(b):
        d = [i for i in range(n) if mask(a)[i] != mask(b)[i]]
        print("    帧体不一致：%d 处，前 5 个偏移 %s" % (len(d), d[:5]))
        return False
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="板端 L0 全量对拍")
    ap.add_argument("dir")
    ap.add_argument("--limit", type=int, default=0, help="只跑前 N 帧（默认全部）")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)

    files = sorted(glob.glob(os.path.join(a.dir, "*.jpg")) +
                   glob.glob(os.path.join(a.dir, "*.ppm")))
    if a.limit:
        files = files[: a.limit]
    if not files:
        print("目录里没有 jpg/ppm：%s" % a.dir)
        return 2

    import cv2  # 板上已有（feed_pl.py 也用它）

    os.chdir(L0)
    work = os.path.join(L0, "sweep")
    os.makedirs(work, exist_ok=True)
    ppm = os.path.join(work, "f.ppm")
    ref = os.path.join(work, "f.ref.bin")
    pl = os.path.join(work, "f.pl.bin")
    cmp_py = os.path.join(L0, "cmp_frame.py")
    if not os.path.exists(cmp_py):
        print("缺少 %s（board_l0.sh 会生成它）" % cmp_py)
        return 2

    fails: list[str] = []
    times: list[float] = []
    for i, path in enumerate(files, 1):
        if path.lower().endswith(".jpg"):
            img = cv2.imread(path, cv2.IMREAD_COLOR)
            if img is None:
                print("[%d/%d] %s 读不出" % (i, len(files), path))
                fails.append(path)
                continue
            if img.shape[1] != 640 or img.shape[0] != 360:
                img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
            cv2.imwrite(ppm, img)
        else:
            shutil.copyfile(path, ppm)

        r = subprocess.run(["./visref", ppm, ref], capture_output=True, text=True)
        #: 不要加 `--quiet`：每帧的"推字/整帧 ms"就打在那一行里（L1 的耗时证据）
        p = subprocess.run(["./feed_pl_c", ppm, "--dump", pl],
                           capture_output=True, text=True)
        ok = r.returncode == 0 and p.returncode == 0
        msg = (r.stderr.strip().splitlines() or [""])[-1]
        import re
        #: 要从**整段 stdout** 里找（最后一行是"总耗时"，不含"整帧"）
        m = re.search(r"整帧 ([0-9.]+) ms", p.stdout)
        if m:
            times.append(float(m.group(1)))
        if ok:
            c = subprocess.run([sys.executable, cmp_py, ref, pl],
                               capture_output=True, text=True)
            if c.returncode != 0:
                ok = False
                msg += " | " + (c.stdout.strip() or c.stderr.strip())
        else:
            ok = False
            msg += " | 喂数器 rc=%d" % p.returncode
        if a.verbose or not ok:
            print("[%d/%d] %-14s %s  %s" % (i, len(files), os.path.basename(path),
                                            "✓" if ok else "✗", msg))
        if not ok:
            fails.append(os.path.basename(path))

    others = [f for f in files if f.lower().endswith(".ppm")]
    print("\n=== L0 汇总")
    print("  帧数        : %d（jpg %d + ppm %d）"
          % (len(files), len(files) - len(others), len(others)))
    print("  逐字节一致  : %d" % (len(files) - len(fails)))
    print("  不一致      : %d %s" % (len(fails), fails[:10]))
    if times:
        times.sort()
        print("  PL 单帧耗时 : min %.2f / 中位 %.2f / max %.2f ms（L1 目标 ≤35）"
              % (times[0], times[len(times) // 2], times[-1]))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
