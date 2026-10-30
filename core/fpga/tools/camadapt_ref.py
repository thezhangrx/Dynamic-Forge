#!/usr/bin/env python3
"""camadapt_ref.py —— 相机域适配的**参考实现**（RTL `vision_camadapt.v` 的 oracle）

为什么单独一个文件
------------------
这段适配本来在 `core/board/tools/pl_vision.cpp:403` 里（PS 侧、正射之后、推送之前）。
P0 把正射搬进 PL 之后，PS 手上不再有页面图，所以这段逻辑必须跟着进 RTL
（否则 PL 看到的还是相机原色，而它的墙判据是**绝对阈值** —— 相机把暖白墙拍成
中性偏蓝，判据直接失效，R62 那 300/300 逐字节一致的结论也随之作废）。

本文件是**逐行照抄** pl_vision.cpp 那段（常数在这里再写一遍，方便核对）：

    gray = (77R + 150G + 29B) >> 8
    逐行：gray >= 150 的连续段，长度 >= 40 → 整段算"墙"
    逐像素：墙        → R=G=v, B = v > 60 ? v-60 : 0      （暖白，R-B=60 让判据认出来）
            v >= 150  → R=G=B = 210                       （短亮斑=角色/反光，保持中性）
            else      → R=G=B = v

用法：
    python3 camadapt_ref.py in.ppm out.ppm
    # 输入一般是 `ortho_fixed_ref --dump` 出来的页面图（640×360），
    # 与 "RTL 正射 + RTL camadapt" 的输出逐字节比。

⚠ 常数（150/40/60/210）是**在真实相机帧上量出来的**（见 PL_HANDOFF §10），
  不是从原理推的；RTL 里必须与这里逐字相同。
"""
import sys

ADAPT_DARK = 150     # >= 它算"亮"
ADAPT_RUN_MIN = 40   # 水平连续亮段至少这么长才算"墙"
ADAPT_TINT = 60      # 暖白重建的 R-B 差
ADAPT_BRIGHT = 210   # 非墙亮斑抬到的亮度


def adapt_rgb(img: bytearray, w: int, h: int) -> bytearray:
    """就地返回适配后的 RGB（输入输出都是 w*h*3 的 bytes）。"""
    out = bytearray(img)
    gray = bytearray(w * h)
    for i in range(w * h):
        r, g, b = out[3 * i], out[3 * i + 1], out[3 * i + 2]
        gray[i] = (77 * r + 150 * g + 29 * b) >> 8
    is_wall = bytearray(w * h)
    for r in range(h):
        base = r * w
        c = 0
        while c < w:
            if gray[base + c] < ADAPT_DARK:
                c += 1
                continue
            e = c
            while e < w and gray[base + e] >= ADAPT_DARK:
                e += 1
            if e - c >= ADAPT_RUN_MIN:
                for k in range(c, e):
                    is_wall[base + k] = 1
            c = e
    for i in range(w * h):
        v = gray[i]
        o = 3 * i
        if is_wall[i]:
            out[o] = out[o + 1] = v
            out[o + 2] = v - ADAPT_TINT if v > ADAPT_TINT else 0
        elif v >= ADAPT_DARK:
            out[o] = out[o + 1] = out[o + 2] = ADAPT_BRIGHT
        else:
            out[o] = out[o + 1] = out[o + 2] = v
    return out


def read_ppm(path: str):
    b = open(path, "rb").read()
    i, nl = 0, 0
    while nl < 3:
        if b[i:i + 1] == b"\n":
            nl += 1
        i += 1
    hdr = b[:i].split()
    assert hdr[0] == b"P6", "只吃 P6 PPM"
    w, h = int(hdr[1]), int(hdr[2])
    return w, h, bytearray(b[i:])


def write_ppm(path: str, w: int, h: int, data: bytes) -> None:
    with open(path, "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (w, h))
        f.write(data)


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    w, h, img = read_ppm(sys.argv[1])
    assert len(img) == w * h * 3, f"像素数不对：{len(img)} != {w*h*3}"
    write_ppm(sys.argv[2], w, h, adapt_rgb(img, w, h))
    print(f"camadapt_ref: {w}x{h} -> {sys.argv[2]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
