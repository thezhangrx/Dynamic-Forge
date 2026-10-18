#!/usr/bin/env python3
"""把板子发来的**原始 RGB 帧**（管道）实时显示成一个窗口。

为什么要有它：相机在板子上、屏幕在 Windows 侧，而**主机上没有 ffmpeg**（实测
`which ffmpeg` 为空），所以"看板子此刻识别到什么"没有现成的显示通路。
这个脚本只做一件事：stdin 读 640x360 原始 RGB → pygame 窗口 → 按键退出。
配上 `wall_gap_view`（板子侧把检测结果画进画面）就是**板子视角的实时可视化**。

用法：
    ssh 板子 '采集 | wall_gap_view' | python3 view_raw_rgb.py
    python3 view_raw_rgb.py --w 640 --h 360 --scale 1.5

按键：ESC / q 退出；空格 暂停/继续；+/- 缩放。
"""

from __future__ import annotations

import argparse
import sys
import time

import pygame


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=360)
    ap.add_argument("--scale", type=float, default=1.6)
    ap.add_argument("--title", default="board view")
    a = ap.parse_args(argv)

    n = a.w * a.h * 3
    pygame.init()
    pygame.display.set_caption(a.title)
    scale = float(a.scale)
    screen = pygame.display.set_mode((int(a.w * scale), int(a.h * scale)))
    font = pygame.font.SysFont("Consolas", 16)
    alive = True
    fps = 0.0
    t_last = time.monotonic()
    frames = 0
    buf = sys.stdin.buffer
    while alive:
        need = n
        chunk = bytearray()
        while need > 0:
            b = buf.read(need)
            if not b:
                print("[view] 输入结束（板子那边停了？）", file=sys.stderr)
                return 0
            chunk += b
            need -= len(b)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return 0
            if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q):
                return 0
            if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_PLUS, pygame.K_EQUALS):
                scale = min(3.0, scale * 1.15)
                screen = pygame.display.set_mode((int(a.w * scale), int(a.h * scale)))
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_MINUS:
                scale = max(0.5, scale / 1.15)
                screen = pygame.display.set_mode((int(a.w * scale), int(a.h * scale)))
        surf = pygame.image.frombuffer(bytes(chunk), (a.w, a.h), "RGB")
        if scale != 1.0:
            surf = pygame.transform.scale(surf, (int(a.w * scale), int(a.h * scale)))
        screen.blit(surf, (0, 0))
        # 帧率角标（画面已经被检测结果画满了，这里只补一个数字）
        frames += 1
        now = time.monotonic()
        if now - t_last >= 1.0:
            fps = frames / (now - t_last)
            frames = 0
            t_last = now
        screen.blit(font.render(f"{fps:4.1f} fps", True, (255, 255, 0)), (6, 4))
        pygame.display.flip()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
