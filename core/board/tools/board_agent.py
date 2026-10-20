#!/usr/bin/env python3
"""板端 agent：**VT + 决策都在板子上**，主机只提供位姿和回放 @MOVE。

输入：
  * `/tmp/vt.jsonl`：板上 `vt_board` 每帧写的 VT 结果（y/gl/gr/player/frame_geom）；
  * TCP：主机发来的 `@POSE <x> <y> <r>`（相当于机器人自己的里程计）。
输出：
  * `/root/tools/field_cli_board` 的 stdin：`@STATE ...`（墙来自 VT）；
  * TCP：把 `@MOVE <act>` 发回主机。

门控也在这里：VT 看到"前方还没确认的墙"就暂停（向主机发 `@MOVE 0`），
连续两帧稳定后才把墙写进已知集合、再继续。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time

WALL_THICK = 8.0


def vt_world_y(vy: float, py: float, half: float, y_scale: float, y_bias: float,
               field_h: float = 3420.0) -> float:
    """VT 帧内 y → 场地 y（**按跟随视口被场地边界夹住后的可见带**算）。

    页面视口中心是 (场地中线, py)，接近场地两端时可见带被夹在 [0, field_h]。
    实测页面在 py=40 时把真值 y=80 / 220 的两道墙画在像素行 200 / 60，也就是
    **vy 是从可见带上沿往下数的像素**，所以场地 y = 可见带上沿 - vy：
    上沿 = min(py+half, field_h)，起点时 = 280 → 280-200=80、280-60=220 ✓。
    旧公式 py+(vy-240) 既翻错了符号又没管夹取，墙 y 才一直偏。
    """
    top = py + half
    if top > field_h:
        top = field_h
    return (top - float(vy) * (2.0 * half / 480.0)) * y_scale + y_bias


class VtTail:
    """跟读 vt_board 写的 jsonl，只保留最新一帧。"""

    def __init__(self, path: str, from_end: bool = False) -> None:
        self.path = path
        self.from_end = bool(from_end)
        self._latest: dict | None = None
        self._seq = 0
        self._buf = b""
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        f = None
        while not self._stop:
            if f is None:
                try:
                    f = open(self.path, "rb")
                    #: 默认从头读：vt_board 可能先写了几帧；换场景重跑时用 --from-end
                    #: 跳过上一轮留下的旧帧，避免把旧场景的墙喂进本轮。
                    f.seek(0, 2 if self.from_end else 0)
                except OSError:
                    time.sleep(0.2)
                    continue
            chunk = f.read()
            if not chunk:
                time.sleep(0.02)
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    rec = json.loads(line)
                except Exception:                 # noqa: BLE001
                    continue
                self._seq += 1
                rec["seq"] = self._seq
                self._latest = rec

    def latest(self) -> dict | None:
        return self._latest

    def stop(self) -> None:
        self._stop = True


class HostLink:
    """TCP 客户端：连主机，收 @POSE，发 @MOVE。"""

    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((host, int(port)))
        self.pose = (320.0, 60.0, 10.0)
        self.pose_seq = 0
        self.pose_t = time.monotonic()
        self._buf = b""
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        while True:
            try:
                chunk = self.sock.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                t = line.decode("utf-8", "ignore").strip()
                if not t.startswith("@POSE"):
                    continue
                try:
                    parts = t.split()
                    if len(parts) >= 5:
                        seq, x, y, r = parts[1], parts[2], parts[3], parts[4]
                    else:
                        seq, x, y, r = 0, parts[1], parts[2], parts[3]
                    with self._lock:
                        self.pose = (float(x), float(y), float(r))
                        self.pose_seq = int(seq)
                        self.pose_t = time.monotonic()
                except (ValueError, IndexError):
                    continue

    def get_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self.pose

    def get_pose_meta(self) -> tuple[int, float]:
        with self._lock:
            return int(self.pose_seq), float(self.pose_t)

    def send_move(self, act: int) -> None:
        try:
            self.sock.sendall(f"@MOVE {int(act)}\n".encode())
        except OSError:
            pass


class StdioLink:
    """走 SSH 的 stdio：stdin 收 @POSE，stdout 发 @MOVE（板子连不到主机时用）。"""

    def __init__(self) -> None:
        self.pose = (320.0, 60.0, 10.0)
        self.pose_seq = 0
        self.pose_t = time.monotonic()
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in sys.stdin:
            t = line.strip()
            if not t.startswith("@POSE"):
                continue
            try:
                parts = t.split()
                if len(parts) >= 5:
                    seq, x, y, r = parts[1], parts[2], parts[3], parts[4]
                else:
                    seq, x, y, r = 0, parts[1], parts[2], parts[3]
                with self._lock:
                    self.pose = (float(x), float(y), float(r))
                    self.pose_seq = int(seq)
                    self.pose_t = time.monotonic()
            except (ValueError, IndexError):
                continue

    def get_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self.pose

    def get_pose_meta(self) -> tuple[int, float]:
        with self._lock:
            return int(self.pose_seq), float(self.pose_t)

    def send_move(self, act: int) -> None:
        try:
            sys.stdout.write(f"@MOVE {int(act)}\n")
            sys.stdout.flush()
        except (BrokenPipeError, OSError):
            pass


def consensus(seen, tol_y: float = 25.0, tol_g: float = 15.0, need: int = 2):
    """把暂停期间的多帧候选合成一份：≥need 帧一致才算，取一致帧的中位数。"""
    import statistics
    out: list[tuple[float, float, float]] = []
    for gy, gl, gr in seen:
        m = [(a, b, c) for a, b, c in seen
             if abs(a - gy) <= tol_y and abs(b - gl) <= tol_g and abs(c - gr) <= tol_g]
        if len(m) < need:
            continue
        if any(abs(gy - o[0]) <= tol_y and abs(gl - o[1]) <= tol_g for o in out):
            continue
        out.append(tuple(round(float(statistics.median([x[i] for x in m])), 1)
                         for i in range(3)))
    return out


def kill_stale_clis(keep: int = 0) -> None:
    """清掉上一轮 agent 死后残留的决策进程。

    残留的 `field_cli_board` 会以 120 Hz 空转，4 个就把板子 4 核吃满，
    vt_board 从 1.4 Hz 掉到 0.4 Hz，VT 数据全废。
    """
    killed = 0
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == keep or int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "ignore")
        except OSError:
            continue
        if "field_cli_board" not in cmd or "board_agent" in cmd:
            continue
        try:
            os.kill(int(pid), 15)
            killed += 1
        except OSError:
            pass
    if killed:
        time.sleep(0.3)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="板端：VT → @STATE → 决策 → @MOVE")
    ap.add_argument("--vt-jsonl", default="/tmp/vt.jsonl")
    ap.add_argument("--host", default="192.168.137.1")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--cli", default="/root/tools/field_cli_board")
    ap.add_argument("--field-w", type=float, default=640.0)
    ap.add_argument("--field-h", type=float, default=3420.0)
    ap.add_argument("--goal", type=float, nargs=2, default=(320.0, 3360.0))
    ap.add_argument("--view-h", type=float, default=480.0)
    ap.add_argument("--inflate", type=float, default=26.0)
    ap.add_argument("--pause-ahead", type=float, default=220.0)
    ap.add_argument("--y-bias", type=float, default=0.0)
    ap.add_argument("--y-scale", type=float, default=1.0)
    ap.add_argument("--gap-margin", type=float, default=10.0)
    ap.add_argument("--state-file", default="/tmp/board_state.txt",
                    help="把发给决策的 @STATE 逐行落盘（事后核查墙覆盖率）")
    ap.add_argument("--player-tol", type=float, default=30.0,
                    help="VT 角色 x 与主机 px 偏差超过这个值 → 判为几何坏帧（整帧作废）")
    ap.add_argument("--warmup", type=float, default=2.5,
                    help="开局预热：前 N 秒不收墙（白框四角还在收敛）")
    ap.add_argument("--settle", type=float, default=0.9,
                    help="暂停后等多久再收候选帧（VT 有 ~0.5 s 延迟）")
    ap.add_argument("--from-end", action="store_true",
                    help="从 jsonl 末尾开始读（换场景重跑）")
    ap.add_argument("--stdio", action="store_true",
                    help="走 stdin/stdout（主机用 ssh 起本进程时）")
    ap.add_argument("--events-file", default="/tmp/board_events.jsonl",
                    help="板端结构化事件（JSONL）：VT 帧/pause/add_wall/fix/nudge/move")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)

    t_boot = time.monotonic()
    half = a.view_h * 0.5
    kill_stale_clis()
    vt = VtTail(a.vt_jsonl, from_end=a.from_end)
    link = StdioLink() if a.stdio else HostLink(a.host, a.port)
    cli = subprocess.Popen(
        [a.cli, "--echo", "--field-w", f"{a.field_w:g}", "--field-h", f"{a.field_h:g}",
         "--goal", f"{a.goal[0]:g}", f"{a.goal[1]:g}", "--view-h", f"{a.view_h:g}",
         "--inflate", f"{a.inflate:g}"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1)

    ev_f = None
    if a.events_file:
        try:
            ev_f = open(a.events_file, "w", buffering=1)
        except OSError:
            ev_f = None

    def ev(name: str, **kw) -> None:
        """板端结构化事件：带板端单调时间 + 当时用的 pose_seq（跨机器对齐用）。"""
        if ev_f is None:
            return
        pseq, _pt = link.get_pose_meta()
        px_, py_, _pr = link.get_pose()
        rec = {"ev": name, "t": round(time.monotonic() - t_boot, 4),
               "pose_seq": pseq, "px": round(px_, 1), "py": round(py_, 1)}
        rec.update(kw)
        try:
            ev_f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except (OSError, TypeError, ValueError):
            pass

    state_f = None
    if a.state_file:
        try:
            state_f = open(a.state_file, "w")
        except OSError:
            state_f = None
    known: list[float] = []
    known_detail: list[dict] = []
    seq = 0
    last_act = 0
    last_vt_seq = -1
    prev_extras: list[tuple[float, float, float]] = []
    extras_t = 0.0
    force_fix = False
    nudge_until = 0.0
    nudge_act = 0
    stuck_tries = 0
    pause_seen: list[tuple[float, float, float]] = []
    last_prog_py = -1e9
    last_prog_t = time.monotonic()
    paused = False
    pause_t0 = 0.0
    cooldown_until = 0.0
    cooldown_gaps: list[tuple[float, float]] = []

    def emit_state(px: float, py: float, pr: float) -> None:
        nonlocal seq
        obs = []
        for d in known_detail:
            y = float(d["y"])
            gl = float(d["gl"]); gr = float(d["gr"])
            if gl > 4.0:
                obs.append(f"{gl * 0.5:.1f} {y:.1f} {gl * 0.5:.1f} 4.0 0.0")
            if gr < 636.0:
                obs.append(f"{(gr + 640.0) * 0.5:.1f} {y:.1f} "
                           f"{(640.0 - gr) * 0.5:.1f} 4.0 0.0")
        line = (f"@STATE {seq} {px:.1f} {py:.1f} {pr:.1f} {len(obs)} "
                + " ".join(obs) + "\n")
        seq += 1
        if state_f is not None:
            try:
                state_f.write(line)
                state_f.flush()
            except (OSError, ValueError):
                pass
        if seq % 10 == 1:
            #: @STATE 本身没有时间戳（格式是给决策进程吃的），这里每 10 条
            #: 打一个锚点，事后可用 seq 线性插值出任意一条 @STATE 的时间。
            ev("state_tick", seq=seq - 1, n_obs=len(obs))
        try:
            assert cli.stdin is not None
            cli.stdin.write(line)
            cli.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    # 决策输出线程：@MOVE → 主机
    def reader() -> None:
        nonlocal last_act
        assert cli.stdout is not None
        for line in cli.stdout:
            t = line.strip()
            if not t.startswith("@MOVE"):
                continue
            try:
                act = int(t.split()[1])
            except (IndexError, ValueError):
                continue
            changed = act != last_act
            last_act = act
            if not paused and time.monotonic() >= nudge_until:
                link.send_move(act)
                if changed:
                    ev("move_sent", act=int(act), src="cli")

    threading.Thread(target=reader, daemon=True).start()
    print(f"  board agent: vt={a.vt_jsonl} host={a.host}:{a.port} "
          f"field={a.field_w:g}×{a.field_h:g}", file=sys.stderr, flush=True)

    #: 开局先停住：板端 VT 要 0.5~1 s 才出第一帧，若一上来就按"无墙直线冲向
    #: 终点"走，0.18 s 就会顶进第一道墙（实测 min_clearance=-10）。
    paused = True
    pause_t0 = time.monotonic()
    t_start = time.monotonic()
    link.send_move(0)
    ev("agent_start", field_w=a.field_w, field_h=a.field_h, view_h=a.view_h,
       inflate=a.inflate, y_bias=a.y_bias, y_scale=a.y_scale,
       gap_margin=a.gap_margin, settle=a.settle, warmup=a.warmup,
       goal=list(a.goal))
    geom_rej_n = 0
    geom_warn_t = 0.0
    last_status = 0.0
    extras: list[tuple[float, float, float]] = []
    while True:
        s = vt.latest()
        px, py, pr = link.get_pose()
        now_m = time.monotonic()
        if py > last_prog_py + 2.0:
            last_prog_py = py
            last_prog_t = now_m
        elif not paused and now_m - last_prog_t > 1.5:
            #: 位置 1.5 s 没前进：多半是贴着墙沿推不过去，停下来按近距离重新测缺口。
            paused = True
            force_fix = True
            pause_t0 = now_m
            last_prog_t = now_m
            link.send_move(0)
            ev("stuck", t_sim=None)
            if a.verbose:
                print(f"  stuck at y={py:.1f} → refit", file=sys.stderr, flush=True)
        if extras_t and time.monotonic() - extras_t > 1.5:
            #: VT 断流/停帧后候选已经过期：清掉，别拿旧数据反复触发暂停。
            extras = []
            prev_extras = []
        if s is not None and s["seq"] > last_vt_seq and s.get("frame_geom"):
            last_vt_seq = s["seq"]
            walls = [w for w in s.get("walls", [])
                     if 60.0 <= float(w["y"]) <= 480.0]
            extras = []
            #: 几何自检：页面把角色画在它的场地 x 上（视口中线固定 320），所以
            #: VT 量到的角色 x 应该 ≈ 主机 px。偏了就说明白框/四边形检测坏了，
            #: 这一帧的缺口 x 一律不可信 —— 整帧丢掉，别让它污染 known。
            pv = s.get("player")
            if pv is not None and a.player_tol > 0:
                dx_p = float(pv[0]) - px
                if abs(dx_p) > a.player_tol:
                    geom_rej_n += 1
                    ev("geom_reject", dx=round(dx_p, 1), vt_seq=int(s["seq"]),
                       player=[round(float(pv[0]), 1), round(float(pv[1]), 1)],
                       px=round(px, 1), n_vt=len(walls))
                    walls = []
                    if (geom_rej_n >= 6
                            and time.monotonic() - geom_warn_t > 5.0
                            and not known):
                        geom_warn_t = time.monotonic()
                        print(f"GEOM_BAD rejects={geom_rej_n} "
                              f"player_x={float(pv[0]):.0f} px={px:.0f} "
                              f"dx={dx_p:+.0f}（相机没看到页面？）",
                              file=sys.stderr, flush=True)
            now_m = time.monotonic()
            #: 暂停时姿态是停住的，VT 帧（0.5 s 延迟）用的就是当前 py；再叠一层
            #: "用帧内角色 y 反推拍照位姿"反而会把视口被夹住的情况算错，所以直接
            #: 用当前 py + 可见带夹取。
            for w in walls:
                gy = vt_world_y(w["y"], py, half, a.y_scale, a.y_bias, a.field_h)
                ahead = gy - py
                if not (5.0 < gy < a.field_h - 8.0) or ahead < -20.0:
                    continue
                #: 贴近的墙（视野下沿）几何误差更大，缺口再放宽一点。
                margin = a.gap_margin if ahead >= 60.0 else a.gap_margin + 6.0
                gl = float(w.get("gl", 0.0)) + margin
                gr = float(w.get("gr", 640.0)) - margin
                if gr - gl < 60.0:
                    c = 0.5 * (gl + gr)
                    gl, gr = c - 35.0, c + 35.0
                gl, gr = max(0.0, gl), min(640.0, gr)
                #: 刚加入的墙（按缺口识别）冷却 2 s，否则角色一动、VT 帧变旧，
                #: 同一道墙的映射 y 就变了，会无限"暂停→恢复"。
                if (now_m < cooldown_until
                        and any(abs(g[0] - gl) <= 30.0 and abs(g[1] - gr) <= 30.0
                                for g in cooldown_gaps)):
                    continue
                extras.append((gy, gl, gr))
            _pv = s.get("player") or [None, None]
            ev("vt_tick", vt_seq=int(s["seq"]), frame=s.get("frame"),
               geom=int(s.get("frame_geom") or 0), n_vt=len(s.get("walls", [])),
               n_used=len(extras), player=s.get("player"),
               player_dx=None if _pv[0] is None else round(float(_pv[0]) - px, 1),
               cand=[[round(e[0], 1), round(e[1], 1), round(e[2], 1)] for e in extras[:6]])
            raw = list(extras)
            #: 已经进 known 的墙不再是"新墙"，否则恢复运动后立刻又被同一批触发暂停。
            extras = [e for e in raw
                      if not any(abs(e[0] - k) <= 55.0 for k in known)]
            prev_extras = list(extras)
            extras_t = time.monotonic()
            if extras and not paused:
                paused = True
                pause_t0 = extras_t
                pause_seen = []
                link.send_move(0)          # 立刻停住，等确认
                ev("pause", cands=[[round(e[0], 1), round(e[1], 1), round(e[2], 1)]
                                   for e in extras[:6]],
                   vt_seq=int(s["seq"]))
                if a.verbose:
                    print(f"  pause: {[round(e[0], 1) for e in extras]}",
                          file=sys.stderr, flush=True)
            if paused:
                #: 暂停刚发出时 VT 帧还可能是"动着的"那一帧：等 0.35 s 再开始收，
                #: 之后的帧才是停稳后拍的（否则 y/缺口带着运动模糊的偏差）。
                if time.monotonic() - pause_t0 >= a.settle:
                    pause_seen.extend(raw)
                del pause_seen[:-30]
                waited = time.monotonic() - pause_t0
                #: 板端 VT 2 Hz，单帧缺口偶尔整体偏几十像素：同一道墙要在 ≥2 帧里
                #: 一致（y±25、缺口±15）才算数；等不到就退化成超时整批收下。
                warmed = (time.monotonic() - t_start) >= a.warmup
                agree = consensus(pause_seen)
                accept = agree if (warmed and waited >= 0.9 and agree) else []
                if force_fix:
                    fixed = 0
                    for gy, gl, gr in (agree or raw):
                        near = [d for d in known_detail
                                if abs(float(d["y"]) - gy) <= 70.0]
                        if not near:
                            continue
                        d = min(near, key=lambda q: abs(float(q["y"]) - gy))
                        if (abs(float(d["gl"]) - gl) > 0.5
                                or abs(float(d["gr"]) - gr) > 0.5):
                            d["gl"], d["gr"] = round(gl, 1), round(gr, 1)
                            fixed += 1
                    if fixed:
                        ev("fix_gap", n=int(fixed))
                        if a.verbose:
                            print(f"  fix gap: {fixed} 条墙按近距离重测",
                                  file=sys.stderr, flush=True)
                    else:
                        stuck_tries += 1
                    if fixed or stuck_tries >= 2:
                        if not fixed:
                            #: 还是测不出这条墙 → 朝已知缺口中心侧向滑一下再试。
                            blk = [d for d in known_detail if float(d["y"]) > py - 20.0]
                            if blk:
                                d = min(blk, key=lambda q: float(q["y"]) - py)
                                center = 0.5 * (float(d["gl"]) + float(d["gr"]))
                                nudge_act = 2 if px < center else 1
                                nudge_until = time.monotonic() + 0.9
                                ev("nudge", act=int(nudge_act),
                                   center=round(center, 1), px=round(px, 1))
                                if a.verbose:
                                    print(f"  nudge act={nudge_act} → center {center:.0f}",
                                          file=sys.stderr, flush=True)
                            stuck_tries = 0
                        force_fix = False
                        paused = False
                        prev_extras = []
                        pause_seen = []
                        last_prog_py, last_prog_t = py, time.monotonic()
                        link.send_move(nudge_act if time.monotonic() < nudge_until
                                       else last_act)
                if paused:
                    for gy, gl, gr in accept:
                        if any(abs(gy - k) <= 55.0 for k in known):
                            continue
                        known.append(gy)
                        known_detail.append({"y": round(gy, 1), "gl": round(gl, 1),
                                             "gr": round(gr, 1),
                                             "source": "board_vt"})
                        ev("add_wall", y=round(gy, 1), gl=round(gl, 1),
                           gr=round(gr, 1), gap_w=round(gr - gl, 1),
                           ahead=round(gy - py, 1), vt_seq=int(s["seq"]),
                           vt_frame=s.get("frame"), waited=round(waited, 2),
                           n_agree=len(accept))
                        if a.verbose:
                            print(f"  + wall y={gy:.1f} gap {gl:.0f}-{gr:.0f}",
                                  file=sys.stderr, flush=True)
                    if a.verbose and not accept and waited > 2.5:
                        print(f"  [dbg] seen={len(pause_seen)} agree={len(agree)} "
                              f"raw={[round(v[0], 1) for v in raw[:3]]}",
                              file=sys.stderr, flush=True)
                    known.sort(key=lambda v: v)
                    remain = [e for e in extras
                              if not any(abs(e[0] - k) <= 30.0 for k in known)]
                    if (accept and not remain) or (waited > 4.5 and warmed):
                        added_gaps = [(gl, gr) for _gy, gl, gr in accept]
                        if added_gaps:
                            cooldown_gaps[:] = (cooldown_gaps + added_gaps)[-12:]
                            cooldown_until = time.monotonic() + 2.0
                        paused = False
                        prev_extras = []
                        pause_seen = []
                        last_prog_py, last_prog_t = py, time.monotonic()
                        link.send_move(last_act)
                        ev("resume", kind="accepted", n_known=len(known),
                           next_act=int(last_act))
        #: 暂停超时要在主循环里判：VT 断流时（没有新帧）也要能恢复。
        if (paused and time.monotonic() - pause_t0 > 4.0
                and time.monotonic() - t_start >= a.warmup):
            paused = False
            prev_extras = []
            last_prog_py, last_prog_t = py, time.monotonic()
            link.send_move(last_act)
            ev("resume", kind="timeout", n_known=len(known), next_act=int(last_act))
            if a.verbose:
                print("  resume (timeout)", file=sys.stderr, flush=True)
        if time.monotonic() < nudge_until:
            link.send_move(nudge_act)
            last_prog_py, last_prog_t = py, time.monotonic()
        emit_state(px, py, pr)
        if time.monotonic() - last_status >= 2.0:
            last_status = time.monotonic()
            print(f"  board: known={len(known)} paused={paused} "
                  f"vt_walls={len(s.get('walls', [])) if s else 0}", file=sys.stderr,
                  flush=True)
        time.sleep(1.0 / 120.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        kill_stale_clis()
