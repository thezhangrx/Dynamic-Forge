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


#: 墙速的合理区间（场地单位/板秒）：契约 128 单位/仿真秒 ≈ 156 单位/板秒，
#: 给一倍余量。估出来的速度落在这个区间外，说明样本混进了别的墙 → 不许冻结。
VY_MIN, VY_MAX = 60.0, 260.0


class WallSlot:
    """虚拟真值里**一个墙位**（一面墙）。一句话规则：入队后不再被视觉刷新。"""

    def __init__(self, t: float, y: float, gap: float, n: int):
        self.seen = [(t, y, gap)]      # 入队前的观测（用来判"稳定"和取中位数）
        self.n = max(1, n)
        self.y = y
        self.gap = gap
        self.vy = 0.0
        self.t_last = t
        self.frozen = False
        self.age = 0

    def observe(self, t: float, y: float, gap: float) -> None:
        """**未冻结**时才接受视觉更新；稳定到 n 帧**且观测到明显位移**才冻结。"""
        if self.frozen:
            return
        self.seen.append((t, y, gap))
        if len(self.seen) > 24:            # 兜底：攒太多只留最近 24 个
            self.seen = self.seen[-24:]
        moved = max(v for _, v, _ in self.seen) - min(v for _, v, _ in self.seen)
        # 冻结条件：① 攒够帧；② 观测到位移；③ **测出来的速度必须是"墙的速度"**。
        # ③ 是关键：`wall_y_units` 是"决策选中的那扇门"的 y，选中的门切换时样本会
        # 混进另一面墙，算出来的速度偏小 → 冻结后这面墙**爬得很慢、一直出不了场地**，
        # 把队列位占死（用户看到的"墙移动缓慢、卡在队列里"就是这个）。
        vy_ok = True
        if len(self.seen) >= 2:
            dtb = self.seen[-1][0] - self.seen[0][0]
            if dtb > 0.05:
                vy_est = (self.seen[-1][1] - self.seen[0][1]) / dtb
                vy_ok = (VY_MIN <= abs(vy_est) <= VY_MAX)
        if len(self.seen) >= self.n and moved >= 6.0 and vy_ok:
            ys = sorted(v for _, v, _ in self.seen)
            gs = sorted(g for _, _, g in self.seen if g)
            self.y = ys[len(ys) // 2]
            self.gap = gs[len(gs) // 2] if gs else self.gap
            dt = self.seen[-1][0] - self.seen[0][0]
            if dt > 0.05:
                self.vy = (self.seen[-1][1] - self.seen[0][1]) / dt
            self.frozen = True

    def advance(self, t: float) -> None:
        """冻结后只按自己的 vy 积分（不再看视觉）。"""
        if self.frozen:
            self.y += self.vy * max(0.0, t - self.t_last)
        self.t_last = t
        self.age += 1

    def alive(self) -> bool:
        # 兜底：即使速度估错，**活了太久也必须释放**，否则队列位被永久占死
        return (-8.0 <= self.y <= FIELD_H + 8.0) and self.age < 16.0 * 12


#: 角色的虚拟真值积分器（只在这里用；板子内部是同一套公式：位置 = 初始 + Σ方向×速度×dt）
_PV = {"locked": False, "x": 0.0, "y": 0.0, "t": None, "buf": []}
SPEED_BOARD = 156.6      # 场地单位/板秒（boot_cal 实测）
_DIRS = {0: (0.0, 0.0), 1: (-1.0, 0.0), 2: (1.0, 0.0), 3: (0.0, 1.0), 4: (0.0, -1.0),
         5: (-0.70710678, 0.70710678), 6: (0.70710678, 0.70710678),
         7: (-0.70710678, -0.70710678), 8: (0.70710678, -0.70710678)}


def virt_player(r: dict) -> tuple:
    """把诊断里的原始视觉角色换成**虚拟真值**（锁初始位置 + 指令积分）。

    为什么必须这么做：视频里原来画的是 `player_field` ——那是**逐帧视觉**结果。
    板子决策用的却是虚拟真值（零校正），所以视频里的"乱飘"根本不是决策看到的东西；
    要在视频里看板子的世界，就得在这里按同一套公式积分。
    """
    t = float(r.get("host_t") or (r.get("frame") or 0) / 16.0)
    if not _PV["locked"]:
        if r.get("player_field_valid"):
            _PV["buf"].append((t, r["player_field"][0], r["player_field"][1]))
            if len(_PV["buf"]) >= 8:
                xs = sorted(b[1] for b in _PV["buf"]); ys = sorted(b[2] for b in _PV["buf"])
                if xs[-1] - xs[0] <= 20 and ys[-1] - ys[0] <= 20:
                    _PV["x"], _PV["y"] = xs[len(xs)//2], ys[len(ys)//2]
                    _PV["locked"] = True
        return (None, None)
    if _PV["t"] is not None:
        dt = max(0.0, min(0.5, t - _PV["t"]))
        ux, uy = _DIRS.get(int(r.get("action") or 0), (0.0, 0.0))
        _PV["x"] = min(max(_PV["x"] + ux * SPEED_BOARD * dt, 11.0), 629.0)
        _PV["y"] = min(max(_PV["y"] + uy * SPEED_BOARD * dt, 11.0), 469.0)
    _PV["t"] = t
    return (_PV["x"], _PV["y"])


def apply_virt(r: dict, q: list, n: int, hide: int,
               cap: int = 2, similar: float = 40.0) -> dict:
    """按**队列**维护墙：准入要"稳定(N 帧)"且"不与已有墙重复"，出门就释放。

    为什么必须是队列而不是"每帧匹配一下"：墙一路走到画面底部时，视觉还会继续
    报（甚至报到画面外的行带），如果允许刷新，虚拟真值就跟着它一直动 —— 这正是
    视频里"墙到底了还在刷"的原因。入队后**永不刷新**，只在离场时释放。
    """
    rr = dict(r)
    y = r.get("wall_y_units") or 0.0
    g = r.get("gap_units") or 0.0
    t = float(r.get("host_t") or (r.get("frame") or 0) / 16.0)
    have = bool(r.get("walls")) and bool(y)

    # 1) 每帧推进已有墙（积分 + 离场判定）
    for slot in list(q):
        slot.advance(t)
        if not slot.alive():
            q.remove(slot)

    # 2) 有观测：找"同一面墙"（已入队的）→ 只允许未冻结的更新；否则尝试入队
    matched = None
    if have:
        for slot in q:
            ref = slot.y if slot.frozen else slot.seen[-1][1]
            if abs(ref - y) <= similar:
                matched = slot
                break
        if matched is not None:
            matched.observe(t, y, g)       # 冻结后这里直接 return（不刷新）
            # 关键：已冻结的墙绝不被这一帧的 y 拉走
        elif len(q) < cap:
            q.append(WallSlot(t, y, g, n))  # 新墙：先攒观测，够稳才画

    # 3) 画：只画**已冻结**的墙（--hide-unfrozen 0 时可画未冻结的）
    if q:
        draw = [sl for sl in q if sl.frozen or not hide]
        if draw:
            slot = draw[0]        # 先只画最近的一堵；两堵时下面的补一条
            rr["wall_y_units"] = slot.y
            rr["gap_units"] = slot.gap
            rr["walls"] = [{"frozen": True}]
            rr["_extra"] = [{"y": sl.y, "gap": sl.gap} for sl in draw[1:]]
        else:
            rr["walls"], rr["wall_y_units"], rr["gap_units"] = [], 0.0, 0.0
    else:
        rr["walls"] = []
    return rr


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag", default=None, help="板子诊断日志（JSONL）")
    ap.add_argument("--stdin", action="store_true", help="从 stdin 读（配合 ssh tail -f 看直播）")
    ap.add_argument("--fps", type=float, default=16.0, help="回放帧率（--stdin 时忽略）")
    ap.add_argument("--scale", type=float, default=1.2)
    ap.add_argument("--png-dir", default=None, help="每帧存 PNG（可选）")
    ap.add_argument("--mp4", default=None, help="导出 mp4（用 cv2 编码，无窗口）")
    #: **墙的虚拟真值**：新墙出现后取前 N 帧的中位数把 y / 缺口**固定**，之后按第 N 帧
    #: 时测到的 vy 匀速积分；未固定的墙默认**不画**（`--hide-unfrozen 0` 可画成虚线）。
    #: 这样视频里就不会出现"墙 y/缺口每帧乱跳"——那本来就是逐帧视觉的抖动，
    #: 虚拟真值的目的就是把它定住。
    ap.add_argument("--freeze-wall-n", type=int, default=4)
    ap.add_argument("--hide-unfrozen", type=int, default=1)
    #: 墙队列：最多几个墙位（画面里最多同时两道墙）+ 判"同一面墙"的 y 距离
    ap.add_argument("--queue-cap", type=int, default=2)
    ap.add_argument("--similar-units", type=float, default=40.0)
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args(argv)

    src = sys.stdin if a.stdin else (open(a.diag, encoding="utf-8") if a.diag else None)
    if src is None:
        print("要给 --diag 或 --stdin", file=sys.stderr)
        return 2

    pad = 40
    win = (int(FIELD_W * a.scale + 2 * pad), int(FIELD_H * a.scale + 2 * pad + 24))
    tracks: list = []      # 墙队列（最多 2 个墙位）
    if a.mp4:
        # 无窗口导出：不初始化 pygame 显示，直接用一张 Surface + cv2 写 mp4。
        import numpy as np
        pygame.init()
        surf = pygame.Surface((int(FIELD_W * a.scale + 2 * pad),
                               int(FIELD_H * a.scale + 2 * pad + 24)))
        font = pygame.font.SysFont("Consolas", 15)
        import cv2
        vw = cv2.VideoWriter(a.mp4, cv2.VideoWriter_fourcc(*"mp4v"), a.fps,
                             (surf.get_width(), surf.get_height()))
        n = 0
        for line in src:
            r = parse(line)
            if r is None:
                continue
            r = apply_virt(r, tracks, a.freeze_wall_n, a.hide_unfrozen,
                           a.queue_cap, a.similar_units)
            vx_, vy_ = virt_player(r)
            if vx_ is not None:
                r["player_field"] = [vx_, vy_]
                r["player_field_valid"] = True
                r["player_src"] = "virt"
            draw(surf, font, r, a.scale, pad, n)
            arr = pygame.surfarray.array3d(surf).swapaxes(0, 1)
            vw.write(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
            n += 1
            if a.max_frames and n >= a.max_frames:
                break
        vw.release()
        print(f"导出 {a.mp4}（{n} 帧）")
        return 0
    pygame.init()
    if not tracks:
        pass
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
        r = apply_virt(r, tracks, a.freeze_wall_n, a.hide_unfrozen,
                       a.queue_cap, a.similar_units)
        vx_, vy_ = virt_player(r)
        if vx_ is not None:
            r["player_field"] = [vx_, vy_]
            r["player_field_valid"] = True
            r["player_src"] = "virt"
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
