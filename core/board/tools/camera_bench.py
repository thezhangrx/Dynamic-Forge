#!/usr/bin/env python3
"""全屏测试图案 —— 量相机噪声用的"标板"，不需要仿真在跑。

为什么需要它：板子相机拍屏幕时的"噪点"到底是**传感器噪声**还是**拍屏幕的摩尔纹/
压缩块**，只看实拍帧是分不清的（两种都长得像斑点）。这两个结论对应的修法完全相反：
传感器噪声 → 加光/加曝光时间；摩尔纹 → 换采样率/缩放手法和锐化。
标板能把它们分开：给一块**大面积均匀灰**，真噪声在均匀区上依然有 σ，摩尔纹也在，
但两者的**时间相关性**和**随分辨率的变化**完全不同（见 camera_sweep.sh 的说明）。

图案（模拟真实场地的关键元素，尺寸按场地单位给）：
    · 底色：均匀灰（`--level`）
    · 四角定位标记：深方块 + 亮内部图案（和 scene 的 contract 标记同形）
    · 一条白横条（墙）、一个白圆（角色）
    · 一条细深线（场线）
全屏铺满，不做任何缩放 —— 相机看到什么就是什么。

用法：
    Env/bin/python core/board/tools/camera_bench.py            # 灰 128
    Env/bin/python core/board/tools/camera_bench.py --level 200
按 ESC 或 q 退出。
"""
import argparse
import os

import pygame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', type=int, default=128, help='底色灰度 0~255')
    ap.add_argument('--w', type=int, default=0, help='窗口宽（0 = 全屏）')
    ap.add_argument('--h', type=int, default=0)
    args = ap.parse_args()

    pygame.init()
    if args.w and args.h:
        screen = pygame.display.set_mode((args.w, args.h))
    else:
        info = pygame.display.Info()
        screen = pygame.display.set_mode((info.current_w, info.current_h), pygame.FULLSCREEN)
    W, H = screen.get_size()
    pygame.display.set_caption('相机标板')
    pygame.mouse.set_visible(False)

    lv = max(0, min(255, args.level))
    bg = (lv, lv, lv)
    dark = (max(0, lv - 96),) * 3
    bright = (min(255, lv + 96),) * 3

    # 场地按 640x480 场地单位布局，等比铺满窗口（留 4% 边）
    sx = W * 0.92 / 640.0
    sy = H * 0.92 / 480.0
    ox = (W - 640 * sx) / 2.0
    oy = (H - 480 * sy) / 2.0

    def px(fx, fy):
        return int(ox + fx * sx), int(oy + fy * sy)

    screen.fill((0, 0, 0))
    # 场地
    pygame.draw.rect(screen, bg, pygame.Rect(px(0, 0), px(640, 480)))
    # 四角标记（和 contract 的 (32,32),(608,32),(608,448),(32,448) 一致）
    half = 14
    for (mx, my) in ((32, 32), (608, 32), (608, 448), (32, 448)):
        cx, cy = px(mx, my)
        pygame.draw.rect(screen, dark, pygame.Rect(cx - half, cy - half, 2 * half, 2 * half))
        inner = half // 2
        pygame.draw.rect(screen, bright,
                         pygame.Rect(cx - inner, cy - inner, 2 * inner, 2 * inner))
        pygame.draw.rect(screen, dark, pygame.Rect(cx - 3, cy - 3, 6, 6))
    # 墙：一条横条（长 400、高 12 场地单位）
    pygame.draw.rect(screen, bright, pygame.Rect(px(120, 200), px(520, 212)))
    # 角色：半径 10 的白圆
    cx, cy = px(320, 300)
    pygame.draw.circle(screen, bright, (cx, cy), int(10 * sx))
    # 场线：细深线
    pygame.draw.line(screen, dark, px(0, 120), px(640, 120), max(1, int(2 * sy)))

    pygame.display.flip()
    print(f'[标板] 窗口 {W}x{H}，底色 {lv}；ESC/q 退出', flush=True)
    clock = pygame.time.Clock()
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False
        clock.tick(10)
    pygame.quit()


if __name__ == '__main__':
    main()
