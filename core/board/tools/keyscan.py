#!/usr/bin/env python3
"""keyscan.py —— 找出"按键到底接在哪个 MIO"（只读、不改任何输出脚）

为什么需要：原理图说 PS_KEY = PS_MIO47，但 MIO47 按下没有电平变化（一直读 1），
而 D4 = MIO7 的灯是好的 —— 说明 GPIO 通路本身没问题，是这个脚没接到按键。
所以要扫一遍：把 **MIO 0..53** 里凡是"已经是输入"的脚都盯住，看按一下按键时
哪个脚翻转。

安全边界（很重要）：
    * 只 export / 设 direction=in —— **绝不把任何脚改成输出**；
    * 已经是 out 的脚**跳过不动**（那些可能在驱动 LED/复位/使能，改成输入会出事）；
    * 被驱动占用的脚 export 会 EBUSY，直接跳过；
    * 结束后把**我们改过方向的脚还原**、export 过的 unexport。

用法（板上 root）：
    python3 keyscan.py 0               # 0 = 一直扫（推荐：按键随时按都记得到）
    python3 keyscan.py 1200            # 或只扫 1200 s
    cat /tmp/keyscan.log               # 看哪个脚动过（MIO16/22 是外设噪声，已标注）
"""
import os
import sys
import time

GPIO = "/sys/class/gpio"
# Zynq PS GPIO 四个 bank（DT 固定）：MIO0..31 -> 480, MIO32..53 -> 458
MIO_BASE = {i: 480 + i for i in range(0, 32)}
MIO_BASE.update({i: 458 + (i - 32) for i in range(32, 54)})
LOG = "/tmp/keyscan.log"


def f(n, name):
    return "%s/gpio%d/%s" % (GPIO, n, name)


def export(n):
    if os.path.isdir("%s/gpio%d" % (GPIO, n)):
        return "exists"
    try:
        with open(GPIO + "/export", "w") as fh:
            fh.write(str(n))
    except OSError:
        return "busy"
    for _ in range(50):
        if os.path.isdir("%s/gpio%d" % (GPIO, n)):
            return "ok"
        time.sleep(0.01)
    return "busy"


def rd(n, name):
    try:
        with open(f(n, name)) as fh:
            return fh.read().strip()
    except OSError:
        return None


def wr(n, name, val):
    try:
        with open(f(n, name), "w") as fh:
            fh.write(val)
        return True
    except OSError:
        return False


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    forever = (dur <= 0)
    dur = 1e9 if forever else dur
    changed = {}        # 我们自己 export 的脚 -> 原方向（用于还原）
    watch = {}
    skipped = []
    for mio, g in sorted(MIO_BASE.items()):
        st = export(g)
        if st == "busy":
            skipped.append((mio, g, "busy"))
            continue
        d = rd(g, "direction")
        if d is None:
            skipped.append((mio, g, "无 direction"))
            continue
        if d.startswith("out"):
            # 已在驱动东西：绝对不碰，只在日志里记一下
            skipped.append((mio, g, "out(跳过)"))
            continue
        if st == "ok":
            changed[g] = d
        wr(g, "direction", "in")
        v = rd(g, "value")
        if v is None:
            continue
        watch[(mio, g)] = v

    with open(LOG, "w") as fh:
        fh.write("# keyscan %s  监视 %d 个输入脚，跳过 %d 个\n"
                 % (time.strftime("%F %T"), len(watch), len(skipped)))
        fh.write("# 跳过: %s\n" % ", ".join("MIO%d/%d(%s)" % s for s in skipped))
        fh.write("# 起始电平: %s\n"
                 % " ".join("MIO%d=%s" % (m, v) for (m, _), v in sorted(watch.items())))

    print("监视 %d 个 MIO 输入脚，跑 %s，日志 %s"
          % (len(watch), "一直扫（Ctrl-C/systemctl stop 才停）" if forever else "%.0fs" % dur, LOG))
    t_end = time.time() + dur
    last_report = 0.0
    try:
        while time.time() < t_end:
            for (mio, g), old in list(watch.items()):
                v = rd(g, "value")
                if v is None or v == old:
                    continue
                watch[(mio, g)] = v
                noisy = mio in (16, 22)
                with open(LOG, "a") as fh:
                    fh.write("%.3f MIO%d gpio%d %s -> %s%s\n"
                             % (time.time(), mio, g, old, v,
                                "  (外设噪声)" if noisy else "  <== 候选按键"))
                if not noisy:
                    print(">>> 候选按键: MIO%d(gpio%d) %s -> %s" % (mio, g, old, v), flush=True)
            if time.time() - last_report > 60:
                last_report = time.time()
                print("  ... 已扫 %.0fs" % (dur - (t_end - time.time())), flush=True)
            time.sleep(0.03)
    finally:
        for g, d in changed.items():
            wr(g, "direction", d)
            try:
                with open(GPIO + "/unexport", "w") as fh:
                    fh.write(str(g))
            except OSError:
                pass
        print("结束，已还原 %d 个脚的方向" % len(changed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
