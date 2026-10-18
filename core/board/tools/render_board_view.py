#!/usr/bin/env python3
"""把**板子自己的世界模型**渲染成画面（不是相机画面，也不是仿真真值）。

为什么要这个：相机画面只能告诉你"屏幕上长什么样"，仿真真值只能告诉你"现实是什么"，
而**板子决策时用的是它自己那套估计**（角色场地坐标、选中墙的 y、缺口中心、动作）。
角色"乱跑"到底是现实里乱跑、还是板子以为它在乱跑，只有把**板子的世界**画出来才分得清。

数据来源是板子的逐帧诊断日志（`--diag-log`），里面每帧都有：

    player_field   [x, y]        角色（场地单位，y 朝上）
    wall_y_units   y             决策选中那道墙的 y
    gap_units      x             缺口中心
    decision       [vx, vy]      决策算出的速度方向
    action         1..8 / 0      实际发出的动作号
    walls[]        像素行带       每道墙的行带与缺口（画深色参考带用）

用法：
    # 回放一次跑完的日志
    python3 render_board_view.py --diag /tmp/crit4/diag_crit4.jsonl --fps 16

    # 直播：板子上正在跑时，把诊断日志流过来
    ssh 板子 'tail -f /tmp/diag_live.jsonl' | python3 render_board_view.py --stdin

    # 导出 PNG 序列（想存档时用；不需要就不加）
    python3 render_board_view.py --diag X.jsonl --png-dir /tmp/boardview

窗口里：
    青圈 = 板子认为的角色     虚线框 = 板子的几何是否可信（红=不可信）
    橙色横条 = 板子选中的墙（缺口位置留空）   黄色十字 = 缺口中心（瞄准点）
    绿色箭头 = 本帧决策方向     左上角 = 帧号/动作/缺口/墙 y/角色来源
ESC / q 退出，空格暂停。
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import pygame

FIELD_W, FIELD_H = 640.0, 480.0
#: 墙在契约里的半厚（场地单位）。仿真的 wall_with_gap 用 4 → 全厚 8。
WALL_HALF_H = 4.0


def parse(line: str) -> dict | None:
    line = line.strip()
    if not line:
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def to_px(x: float, y: float, scale: float, pad: float):
    """场地坐标（y 朝上）→ 窗口像素（y 朝下）。"""
    return (pad + x * scale, pad + (FIELD_H - y) * scale)


def draw(surf, font, r: dict, scale: float, pad: float, frame_no: int) -> None:
    surf.fill((18, 22, 30))
    fg = (90, 100, 120)
    # 场地边框 + 10x10 网格（和仿真窗口的观感对齐，便于肉眼比对）
    pygame.draw.rect(surf, fg, pygame.Rect(pad, pad, FIELD_W * scale, FIELD_H * scale), 1)
    for i in range(1, 10):
        x = pad + FIELD_W * scale * i / 10
        y = pad + FIELD_H * scale * i / 10
        pygame.draw.line(surf, (34, 40, 52), (x, pad), (x, pad + FIELD_H * scale))
        pygame.draw.line(surf, (34, 40, 52), (pad, y), (pad + FIELD_W * scale, y))

    ok = bool(r.get("field_valid"))
    # 几何不可信的帧：整幅画一层红框，提醒"这一帧板子其实在瞎猜"
    if not ok:
        pygame.draw.rect(surf, (190, 60, 60),
                         pygame.Rect(pad, pad, FIELD_W * scale, FIELD_H * scale), 2)

    wy = r.get("wall_y_units") or 0.0
    gx = r.get("gap_units") or 0.0
    has_wall = bool(r.get("walls"))
    if has_wall:
        gw = 70.0   # 缺口宽度：诊断日志里没直接给"场地单位宽度"，按契约量级画个示意
        y0 = wy - WALL_HALF_H
        y1 = wy + WALL_HALF_H
        a = to_px(0.0, y1, scale, pad)
        left_end = to_px(max(0.0, gx - gw * 0.5), y1, scale, pad)
        right_start = to_px(min(FIELD_W, gx + gw * 0.5), y1, scale, pad)
        b = to_px(FIELD_W, y0, scale, pad)
        th = max(2, int((y1 - y0) * scale))
        pygame.draw.rect(surf, (255, 120, 0),
                         pygame.Rect(int(a[0]), int(a[1]), int(left_end[0] - a[0]), th))
        pygame.draw.rect(surf, (255, 120, 0),
                         pygame.Rect(int(right_start[0]), int(b[1] - th), int(b[0] - right_start[0]), th))
        # 瞄准点：缺口中心
        cx, cy = to_px(gx, wy, scale, pad)
        pygame.draw.line(surf, (250, 230, 80), (cx - 8, cy), (cx + 8, cy), 2)
        pygame.draw.line(surf, (250, 230, 80), (cx, cy - 8), (cx, cy + 8), 2)

    if r.get("player_field_valid"):
        px, py = to_px(float(r["player_field"][0]), float(r["player_field"][1]), scale, pad)
        pygame.draw.circle(surf, (120, 240, 255), (int(px), int(py)),
                           max(3, int(10 * scale)), 2)

    d = r.get("decision") or [0.0, 0.0]
    if abs(d[0]) + abs(d[1]) > 1e-6:
        px, py = to_px(float(r["player_field"][0]), float(r["player_field"][1]), scale, pad) \
            if r.get("player_field_valid") else to_px(FIELD_W / 2, FIELD_H / 2, scale, pad)
        n = max(1e-9, (d[0] ** 2 + d[1] ** 2) ** 0.5)
        ex = px + d[0] / n * 40 * scale
        ey = py - d[1] / n * 40 * scale     # 场地 y 朝上 → 屏幕 y 朝下
        pygame.draw.line(surf, (90, 240, 120), (px, py), (ex, ey), 3)

    info = (f"frame {frame_no:5d}  act {r.get('action')}  "
            f"gap {gx:6.0f}  wall_y {wy:6.0f}  "
            f"player {r.get('player_src')}  geo {r.get('geo_why')}")
    surf.blit(font.render(info, True, (220, 230, 245)), (pad, 4))
    if not ok:
        surf.blit(font.render("GEOMETRY INVALID — 这一帧板子在靠上一条/模型外推",
                              True, (255, 150, 150)), (pad, 22))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag", default=None, help="板子诊断日志（JSONL）")
    ap.add_argument("--stdin", action="store_true", help="从 stdin 读（配合 ssh tail -f 看直播）")
    ap.add_argument("--fps", type=float, default=16.0, help="回放帧率（--stdin 时忽略）")
    ap.add_argument("--scale", type=float, default=1.2)
    ap.add_argument("--png-dir", default=None, help="每帧存 PNG（可选）")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args(argv)

    src = sys.stdin if a.stdin else (open(a.diag, encoding="utf-8") if a.diag else None)
    if src is None:
        print("要给 --diag 或 --stdin", file=sys.stderr)
        return 2

    pad = 40
    win = (int(FIELD_W * a.scale + 2 * pad), int(FIELD_H * a.scale + 2 * pad + 24))
    pygame.init()
    pygame.display.set_caption("board world view (板子的世界)")
    screen = pygame.display.set_mode(win)
    font = pygame.font.SysFont("Consolas", 15)
    paused = False
    n = 0
    dt = 1.0 / max(1e-6, a.fps)
    t_next = time.monotonic()
    for line in src:
        if a.png_dir:
            pass
        r = parse(line)
        if r is None:
            continue
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return 0
            if ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    return 0
                if ev.key == pygame.K_SPACE:
                    paused = not paused
        draw(screen, font, r, a.scale, pad, n)
        pygame.display.flip()
        if a.png_dir:
            import os
            os.makedirs(a.png_dir, exist_ok=True)
            pygame.image.save(screen, f"{a.png_dir}/{n:05d}.png")
        n += 1
        if a.max_frames and n >= a.max_frames:
            return 0
        if not a.stdin:
            # 回放：按固定节拍走；落后太多就跳帧，别越积越慢
            t_next += dt
            now = time.monotonic()
            if t_next > now:
                time.sleep(t_next - now)
            else:
                t_next = now
        while paused:
            time.sleep(0.05)
            for ev in pygame.event.get():
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    paused = False
                if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    return 0
    pygame.time.wait(200)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
