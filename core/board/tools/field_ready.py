#!/usr/bin/env python3
"""判断"相机现在看到的是不是场地"——验收跑开始前的就绪门。

为什么需要它（第 11 轮的教训）：脚本里 `sleep 6` 之后开始记录，但 pygame 切全屏比这慢，
于是每轮开头十几秒相机拍的是**桌面/编辑器**。那段时间真值里 `obstacles=[]`，而视觉却
报出两道"墙"、角色偏 120 单位——数据被污染，所有统计都不可信（而且看起来像算法错了）。

判据（只看画面本身，不看真值、不看算法输出）：
  1. 有一道**厚 ≥5 px 的连续亮横带**（墙），且亮列占比 ≥0.72
  2. 有四个**深色方块**（定位标记）——用"深色像素占比高的 3×3 区块"粗查

两者同时满足就认为"场地已就绪"。返回 0 = 就绪，1 = 未就绪。

用法： python3 field_ready.py frame1.ppm [frame2.ppm ...]
"""
import sys

import numpy as np
from PIL import Image


def luma(a):
    return (77.0 * a[:, :, 0] + 150.0 * a[:, :, 1] + 29.0 * a[:, :, 2]) / 256.0


def has_wall(L, min_rows=5, bright_frac=0.72):
    h, w = L.shape
    # 阈值：整幅 (p50 + p99.5)/2，与曝光无关
    t = (np.percentile(L, 50) + np.percentile(L, 99.5)) / 2.0
    rows = (L >= t)
    frac = rows.mean(axis=1)
    y = 0
    while y < h:
        if frac[y] >= 0.20:
            y0 = y
            while y < h and frac[y] >= 0.20:
                y += 1
            if y - y0 >= min_rows:
                band = rows[y0:y, :]
                col = band.mean(axis=0)
                # 只看"墙的跨度"内的亮列占比
                xs = np.where(col >= 0.55)[0]
                if xs.size > 0.25 * w:
                    sub = col[xs.min():xs.max() + 1]
                    if sub.mean() >= bright_frac:
                        return True
        else:
            y += 1
    return False


def has_markers(L, blk=6, dark_frac=0.55, need=4):
    h, w = L.shape
    t = (np.percentile(L, 10) + np.percentile(L, 50)) / 2.0
    dark = (L <= t)
    hh, ww = h // blk * blk, w // blk * blk
    cells = dark[:hh, :ww].reshape(hh // blk, blk, ww // blk, blk).mean(axis=(1, 3))
    return int((cells >= dark_frac).sum()) >= need


def main(argv):
    ok = 0
    total = 0
    for path in argv[1:]:
        try:
            a = np.asarray(Image.open(path).convert('RGB')).astype(np.float32)
        except Exception as exc:                     # noqa: BLE001
            print(f'[ready] 读不了 {path}: {exc}', file=sys.stderr)
            continue
        total += 1
        L = luma(a)
        wall = has_wall(L)
        mk = has_markers(L)
        print(f'[ready] {path}: 墙={wall} 标记={mk}')
        if wall and mk:
            ok += 1
    if total == 0:
        return 1
    return 0 if ok * 2 >= total else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
