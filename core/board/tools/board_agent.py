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
from pathlib import Path
import math
import os
import socket
import subprocess
import sys
import threading
import time

WALL_THICK = 8.0


def vt_world_y(vy: float, py: float, half: float, y_scale: float, y_bias: float,
               field_h: float = 3420.0, q_c: float = 0.0) -> float:
    """VT 帧内 y → 场地 y。

    标定方法（对照画面里缺口已知的两道墙）：起点 py=40、真值 y=220 的墙 VT 报
    vy=424，真值 y=80 的墙报 vy=280 —— 两点拟合得

        world = (py - half) + y_scale*vy + y_bias,  y_scale=0.972, y_bias=+8

    也就是 VT 的 vy 是"从可见带下沿往上数"的像素，而白框加粗后行坐标被放大
    1.0286 倍（所以 scale≈1/1.0286）。**不要**再按 clamp 后的可见带算：那会把
    起点附近的墙整体错一行，而且缺口会配到隔壁那道墙上（实测左右侧各错 180）。
    """
    #: q_c ≠ 0 时用**二次**模型（吸收广角畸变带来的"同一像素行在不同高度对应不同
    #: 场地 y"）。实测线性模型整轮残差 9.2 px(max 40)、二次 1.2 px；线性残差会让
    #: 板子的世界模型里出现"错位的墙"，规划器以为走廊堵死就直接停住不动。
    return ((float(py) - float(half)) + y_scale * float(vy) + y_bias
            + q_c * float(vy) * float(vy))


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


def consensus(seen, tol_y: float = 25.0, tol_g: float = 15.0, need: int = 2,
              tol_vy: float = 7.0):
    """把暂停期间的多帧候选合成一份：要求**同一道墙**在 ≥need 帧里一致。

    关键：除了场地位姿 y（和缺口左右边界），还要比**帧内像素 vy**。
    为什么：墙的场地 y 是用"当前 py"映射出来的，一帧旧画面的 vy 不同、映射出来
    的 y 却可能恰好落在同一道墙上（尤其 py 只差几十时）——只比 y 就会把
    "隔壁那道墙的缺口"当成同一道墙收下（实测就是这么换上来的）。vy 是帧内
    坐标，与 py 无关，能直接把这种帧分开。
    """
    import statistics
    out: list[tuple] = []
    for item in seen:
        gy, gl, gr = float(item[0]), float(item[1]), float(item[2])
        vy = float(item[3]) if len(item) > 3 else None

        def ok(o) -> bool:
            if abs(o[0] - gy) > tol_y or abs(o[1] - gl) > tol_g or abs(o[2] - gr) > tol_g:
                return False
            if vy is not None and len(o) > 3 and abs(float(o[3]) - vy) > tol_vy:
                return False
            return True

        m = [o for o in seen if ok(o)]
        if len(m) < need:
            continue
        dup = False
        for o in out:
            if abs(o[0] - gy) <= tol_y and abs(o[1] - gl) <= tol_g:
                if vy is None or len(o) <= 3 or abs(float(o[3]) - vy) <= tol_vy:
                    dup = True
                    break
        if dup:
            continue
        n = len(item)
        out.append(tuple(round(float(statistics.median([o[i] for o in m])), 1)
                         for i in range(n)))
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
    ap.add_argument("--y-bias", type=float, default=0.0,
                    help="同上的截距；外部四边形正确时为 0")
    ap.add_argument("--calib-file", default="/tmp/calib_y.json",
                    help="自动标定结果文件（scale/bias），存在时覆盖命令行默认值")
    ap.add_argument("--y-scale", type=float, default=1.0,
                    help="换算系数。**用主机下发的四边形（VT 的 quad_ext=1）时必须是 "
                         "1.0**：此时 vy 就是页面局部坐标(0=可见带下沿)，"
                         "world=(py-half)+vy 本来就成立；0.972/8 是当年白框检测"
                         "退化时的补丁，已被 calib_y 自动标定取代")
    ap.add_argument("--gap-margin", type=float, default=10.0)
    ap.add_argument("--state-file", default="/tmp/board_state.txt",
                    help="把发给决策的 @STATE 逐行落盘（事后核查墙覆盖率）")
    ap.add_argument("--player-tol", type=float, default=30.0,
                    help="VT 角色 x 与主机 px 偏差超过这个值 → 判为几何坏帧（整帧作废）")
    ap.add_argument("--warmup", type=float, default=2.5,
                    help="开局预热：前 N 秒不收墙（白框四角还在收敛）")
    ap.add_argument("--consensus-frames", type=int, default=3,
                    help="同一道墙要几帧一致才入表（2 帧时偶发错行缺口会被收下）")
    ap.add_argument("--y-qc", type=float, default=0.0,
                    help="y 模型的二次项（标定文件优先）；0 = 纯线性")
    ap.add_argument("--x-bias-a", type=float, default=None,
                    help="横向偏差常数项（标定文件优先）")
    ap.add_argument("--x-bias-b", type=float, default=None,
                    help="横向偏差随 y 的斜率（标定文件优先）")
    ap.add_argument("--x-bias", type=float, default=0.0,
                    help="缺口横向固定偏移（像素，正=VT 比真值偏右）。"
                         "标定文件里给了就用文件里的；这里只是回退默认值")
    ap.add_argument("--pulse-cycle", type=float, default=1.0,
                    help="被互锁挡住时的'凑近-重测'周期（秒）")
    ap.add_argument("--pulse-on", type=float, default=0.25,
                    help="一个周期里放行动作的时间（秒）；其余时间停住重测。"
                         "只用来'凑近看清'，每步都很短，不可能盲穿")
    ap.add_argument("--react-s", type=float, default=0.5,
                    help="从'相机看到'到'动作生效'的延迟上界（秒）。用于算可刹停距离"
                         "= v*react_s*2；v 由位姿历史在线估计。这是实测的管道延迟，"
                         "不是场景参数；换相机/降分辨率后应重新量一次")
    ap.add_argument("--tol-vy", type=float, default=7.0,
                    help="多帧一致时允许的帧内墙行差（像素）。**这是噪声参数，不是场景参数**："
                         "实测静止条件下同一道墙的 vy 重复性 1σ≈8 px，卡在 7 会把一半"
                         "好帧剔掉、留下有偏的样本；准度优先时放宽到 15~20 再多帧取中位数")
    ap.add_argument("--max-stale", type=float, default=8.0,
                    help="测量用的帧，帧内角色与预期位置最多差这么多像素。"
                         "超过的帧只用来触发暂停、不进测量（宁可多等几帧，"
                         "也不要用旧位姿映射墙的 y —— 时间不重要，安全最重要）")
    ap.add_argument("--collect-moving", action="store_true",
                    help="发现新墙时**不停车**，边走边收（需要 VT ≥8 fps）。"
                         "原来每道墙都停一次：12 fps 下墙在可见带里只待 ~7 s，"
                         "一停一开会把连续的几道墙全错过（实测漏 500/640/780/920）")
    ap.add_argument("--pause-timeout", type=float, default=4.0,
                    help="一次暂停最多等这么久；VT 越快这个值越该小（现在 6 fps"
                         "下 consensus 通常 <0.5 s 就够，4 s 会把整轮时间全吃掉）")
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
    #: 横向偏差的默认值归一（标定文件里没有就退回常数项/0）
    if a.x_bias_a is None:
        a.x_bias_a = float(a.x_bias)
    if a.x_bias_b is None:
        a.x_bias_b = 0.0

    t_boot = time.monotonic()
    #: 自动标定的 y 换算系数：优先用标定文件（由 calib_y.py 自动解出），
    #: 没有标定文件时才退回命令行默认值。用户要求"不要手调系数"。
    calib_file = Path(getattr(a, "calib_file", "/tmp/calib_y.json"))
    if calib_file.exists():
        try:
            _c = json.loads(calib_file.read_text(encoding="utf-8"))
            if "scale" in _c and "bias" in _c:
                a.y_scale = float(_c["scale"])
                a.y_bias = float(_c["bias"])
                #: 横向固定偏差（页面四角单应挡不住桶形畸变，实测 +9.4 px）。
                #: 和 scale/bias 一样是**标定产物**，不是手调常数：由 calib_y 用
                #: 一组静止画面 + 真值算中位数得到，写进同一个 calib 文件。
                a.x_bias = float(_c.get("x_bias", 0.0) or 0.0)
                #: 横向偏差**随 y 线性变化**的两个系数（x_corr = a + b*(y-240)）。
                #: 单常数只能折中：实测同一画面 wall@280 偏 +0.9、wall@420 偏 +8.4。
                a.y_qc = float(_c.get("q_c", 0.0) or 0.0)
                a.x_bias_a = float(_c.get("x_bias_a", a.x_bias) or 0.0)
                a.x_bias_b = float(_c.get("x_bias_b", 0.0) or 0.0)
                print(f"  calib_loaded: scale={a.y_scale} bias={a.y_bias} "
                      f"({calib_file})", file=sys.stderr, flush=True)
        except (OSError, ValueError):
            pass
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
    #: 缺口窗口 -> 已入库的墙 y。同一道物理墙远/近两次测量会被映射成不同的 y
    #: （映射依赖 py，实测 215 → 345），按 y 去重就会重复入表、出现"同侧两道墙"。
    #: 缺口位置与 py 无关，用它当身份最稳。
    #: **在线噪声估计**（不写死容差）：
    #:   sig_y = 墙行 y 的重复性；sig_e = 缺口左右边界的重复性。
    #: 由每次"多帧一致"里同一道墙的散度做 EWMA 得到。所有"同一道墙/缺口合理"
    #: 的判据都用它们乘系数，而不是 ±25 / ±30 / ±45 这类凭经验写死的数。
    sig_y = 4.0
    sig_e = 3.0

    def row_tol() -> float:
        """同一道墙的 y 容差：由在线噪声与**已学到的行间距**共同决定。

        两道真墙的 y 相差"行间距"（本场景 140，但这里不写死：取已知行间距的
        最小值）。容差取行间距的 1/3 与 6σ_y 的较小者，保证既不会把两道墙
        合成一道，也不会把同一道墙的两次测量当成两道。
        """
        ys = sorted(known)
        gaps = [b - a for a, b in zip(ys, ys[1:]) if b - a > 8.0]
        spacing = min(gaps) if gaps else 140.0
        return max(6.0, min(spacing / 3.0, 6.0 * sig_y))
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
    #: @POSE 历史（单调时钟 → py）。vt_sparse 每帧带 `t`（板端 CLOCK_MONOTONIC，
    #: 与 Python time.monotonic() 同一时钟），据此能把"这一帧拍的时候平台在哪"
    #: 精确取出来，不再靠"角色在帧里的位置"猜新鲜度 —— 跟随视口里角色恒在页面
    #: 正中，那个信号**根本不含新旧信息**，以前那套 dy_p 修正全是噪音。
    pose_hist: list[tuple[float, float]] = []

    def py_at(t: float) -> float | None:
        """t 时刻的 py（线性插值；超出历史范围返回 None）。"""
        if not pose_hist:
            return None
        if t <= pose_hist[0][0] or t >= pose_hist[-1][0]:
            return None
        lo, hi = 0, len(pose_hist) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if pose_hist[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0_, y0_), (t1_, y1_) = pose_hist[lo], pose_hist[hi]
        if t1_ - t0_ < 1e-9:
            return y1_
        return y0_ + (y1_ - y0_) * (t - t0_) / (t1_ - t0_)
    paused = False
    collecting = False        #: 边走边收：与 paused 共用 pause_seen/pause_t0，
                              #: 区别是**不发 move(0)**、不压住 @MOVE
    pause_t0 = 0.0
    cooldown_until = 0.0
    cooldown_gaps: list[tuple[float, float]] = []

    def gap_plausible(gl: float, gr: float) -> bool:
        """缺口形状是否像"这道墙上的一个洞"（不含场景真值，只用几何常识）。

        本场景真缺口宽 90~110；VT 观测到 45~200 都算正常。太窄（<40）是墙缝/像素线，
        太宽（>210）或贴到画面边缘（gl<15 / gr>625）基本都是配错行或页面白框。
        """
        w = gr - gl
        return 40.0 <= w <= 210.0 and gl >= 15.0 and gr <= 625.0

    def gap_width_outlier(gl: float, gr: float) -> bool:
        """用**已知墙的缺口宽度中位数**做参考，挡掉"宽度明显不像缺口"的行。

        本场景所有墙的缺口宽度都差不多（真值 91~106，板端入表后 ~67~92）。
        一旦某一行量出来的宽度偏离中位数 >45，多半是暗段配错了行（实测
        wall@220 被量成 152 宽、右边界偏 +80）。不依赖任何真值，只用已学到的墙。
        """
        import statistics
        ws = [float(d["gr"]) - float(d["gl"]) for d in known_detail]
        if len(ws) < 4:
            return False
        return abs((gr - gl) - statistics.median(ws)) > 45.0

    def gap_suspect(gy: float, gl: float, gr: float) -> str:
        """判断这条候选的缺口是不是"配错行配来的"。

        依据（来自日志实测）：同一暂停窗口里两道墙都可见时，VT 偶尔把 A 行的缺口
        贴到 B 行上 —— 表现就是"两行的缺口完全相同"，而真缺口左右交替、中心
        相差 60~150。所以：
          dup_known : 与某道已知墙（y 差 ≤250）的缺口在 ±25 内完全相同
          dup_batch : 与本次同批候选的缺口重复（另一道墙已在 y 差 ≥100 处）
          edge      : 缺口贴到画面边缘（gl<20 或 gr>620），这一行多半没测准
        """
        if gl < 20.0 or gr > 620.0:
            return "edge"
        for d in known_detail:
            if abs(float(d["y"]) - gy) <= 250.0 \
                    and abs(float(d["gl"]) - gl) <= 25.0 \
                    and abs(float(d["gr"]) - gr) <= 25.0:
                return "dup_known"
        return ""

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

    #: 读者线程与主循环共享的"安全互锁"状态（跨线程只能通过可变对象传递）
    lock_state = {"unknown_ahead": 0, "block_t0": time.monotonic()}

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
            if lock_state["unknown_ahead"] > 0:
                #: 前方有未测明的墙：**先停住**（互锁，不依赖阈值）。
                #: 但"一直停着"会死锁：有些墙在当前位置就是量不出缺口（贴画面上下沿、
                #: 被反光/角色挡住），只有凑近才看得到 —— 实测 demo3/demo4 就卡在
                #: 墙前 20 单位处一动不动（act=0 十几秒）。
                #: 所以加一个**有限接近**：每 cycle_s 里只放行 pulse_s 的动作，
                #: 剩下的时间停住重测。每步都很短，不可能盲穿过去，但能凑近把墙量清。
                _now = time.monotonic()
                _t0 = float(lock_state.get("block_t0") or _now)
                _ph = (_now - _t0) % max(0.2, a.pulse_cycle)
                if _ph < a.pulse_on:
                    link.send_move(act)          # 短脉冲：接近一点点
                else:
                    link.send_move(0)            # 其余时间停住、让 VT 重新测
                continue
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
       inflate=a.inflate, y_bias=a.y_bias, y_scale=a.y_scale, x_bias=a.x_bias,
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
        pose_hist.append((now_m, py))
        if len(pose_hist) > 600:      # ≈5 s @110 Hz
            del pose_hist[:-600]
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
            raw_dbg: list[tuple] = []
            #: 几何自检：页面把角色画在它的场地 x 上（视口中线固定 320），所以
            #: VT 量到的角色 x 应该 ≈ 主机 px。偏了就说明白框/四边形检测坏了，
            #: 这一帧的缺口 x 一律不可信 —— 整帧丢掉，别让它污染 known。
            #: **前方有"看得见但量不出缺口"的墙 → 不许前进**。
            #: 只靠"learned 的墙"来判断安全是不够的：开局或换行时，VT 明明看到
            #: 一条墙带却量不出缺口，板子却以为自己什么都不知道、于是直着开过去
            #: （实测 s1e：t=2.42 在 wall@360 上 -10）。这条互锁不看真值、不看
            #: 场景参数，只用板端自己的观测：只要前方有未测明的墙，就发 0 等测。
            _ngy = s.get("no_gap_y") or []
            #: **绝不许朝"没量过缺口的墙"前进**（互锁，不依赖任何阈值）。
            #: 之前那一版有两个错：
            #:   ① 把 no_gap_y 的**页面坐标**直接和**世界 py** 比大小 —— 量纲不对，
            #:      互锁形同虚设；实测 s2b 第一道墙 wall@80 被盲穿（余量 −10）。
            #:   ② 用"可刹停距离"做阈值 —— v≈0 时永远不触发，开局正好最危险。
            #: 现在的规则不需要阈值：把"已知墙"和"看见但没量出缺口的墙"都换算成
            #: 世界 y，取**最近的一道在前方的墙**；只要最近那道是"没量过的"，就停。
            def _w(vy: float) -> float:
                return vt_world_y(vy, py, half, a.y_scale, a.y_bias, a.field_h, a.y_qc)

            _nearest_known = min((k for k in known if k > py + 10.0), default=None)
            _nearest_unknown = None
            for _y in _ngy:
                try:
                    _wy = _w(float(_y))
                except (TypeError, ValueError):
                    continue
                if _wy > py + 10.0 and (_nearest_unknown is None or _wy < _nearest_unknown):
                    _nearest_unknown = _wy
            if _nearest_unknown is not None and (
                    _nearest_known is None or _nearest_unknown < _nearest_known - 5.0):
                if not lock_state["unknown_ahead"]:
                    lock_state["block_t0"] = time.monotonic()
                lock_state["unknown_ahead"] = 1
                lock_state["blocked_at"] = round(_nearest_unknown, 1)
            else:
                lock_state["unknown_ahead"] = 0
                lock_state["blocked_at"] = None
            pv = s.get("player")
            dx_p = None if pv is None else float(pv[0]) - px
            #: 竖直方向的"帧新鲜度"：页面**不做夹取**，它永远把角色画在可见带
            #: 正中，所以角色在**页面坐标**里的 vy 恒等于 half（实测 240±1，
            #: 与 py 无关、与标定系数无关）。
            #:
            #: 这里以前写的是 `(half - y_bias) / y_scale` —— 把"页面坐标"和
            #: "世界坐标"混在一起了：标定一变（自动标定后 scale 0.976/bias −5.4），
            #: vy_expect 就变成 251，于是**每一帧都被判成旧帧**（dy≈−10.8），
            #: |dy|>8 的测量闸门把所有帧全丢掉（acc1：stale_skip 1229 次、
            #: add_wall 只有 4 道、覆盖率 4/24）。期望值就是 half，不要掺标定。
            vy_expect = half
            dy_p = None if pv is None else float(pv[1]) - vy_expect
            #: 帧新鲜度门：VT 帧比当前位姿旧 ~0.5 s，角色一秒走 200 单位，
            #: 所以"帧里的角色 x"和主机 px 差多少，就等于这一帧旧了多少。
            #: 暂停后 settle 秒内的帧一律不用；settle 之后只收 |dx|≤12 的新鲜帧。
            #: 这样入表的 y/缺口都来自"停稳那几帧"，不会再出现整行错位。
            #: **首选：按帧时间戳配对位姿**（vt_sparse 起每帧带 "t"）。这是
            #: "这一帧拍的时候平台在哪"的真答案；角色位置法在跟随视口里恒 240，
            #: 判不出新旧，纯噪音（acc1 就是被它把所有帧判成旧帧的）。
            t_fr = s.get("t")
            stale_ok = None      # None = 没有时间戳，退回角色法
            lag_ms = None
            py_cap = py
            contemporaneous = True
            if t_fr is not None:
                _py_at = py_at(float(t_fr))
                if _py_at is not None:
                    py_cap = float(_py_at)
                    lag_ms = round((now_m - float(t_fr)) * 1000.0, 1)
                    contemporaneous = True
                    stale_ok = True
                else:
                    stale_ok = False     # 超出历史：映射不了，丢掉这帧的墙
            if stale_ok is None and pv is not None and dy_p is not None:
                #: 帧里的角色比"当前 py 应有的位置"偏下多少，就说明这一帧是
                #: 角色还低那么多的时候拍的 → 用 py_cap 去映射，y 就不会偏。
                #: 丢帧会让墙学不到（实测只学到 4 道、整轮卡死），修正更好。
                #: 只有"角色检测可信"时才敢用它修正位姿：跟随视口里角色恒在页面
                #: 正中，所以 dy_p 超过 ~80 一定是检测被墙/白框骗走了；这时把
                #: py_cap 拿去映射墙 y 会偏上百单位，多帧永远一致不了（实测
                #: collect-moving 下一道墙都学不到就是这么来的）。放弃这一帧比
                #: 用错位姿更安全。
                if abs(dy_p) > 80.0 or abs(dx_p) > 110.0:
                    ev("stale_drop", dy=round(dy_p, 1), dx=round(dx_p, 1),
                       vt_seq=int(s["seq"]), py=round(py, 1))
                    walls = []
                py_cap = py - dy_p * a.y_scale
                #: 只有"帧里的角色与当前位姿基本重合"的帧才算同拍（|dy|≤5 → 位移
                #: ≤5 单位）。这类帧的墙行才可信；旧帧只用来触发暂停，不进测量。
                contemporaneous = abs(dy_p) <= 5.0
                if abs(dy_p) > 6.0:
                    ev("stale_corrected", dy=round(dy_p, 1),
                       vt_seq=int(s["seq"]), py=round(py, 1),
                       py_cap=round(py_cap, 1))
            #: **测量只用"同期"的帧**：contemporaneous 以前算了却没用来闸门，
            #: 于是旧帧（角色已经移动了 30~100 单位）也会被 py_cap 修正后拿去
            #: 映射墙 y，残差 p95 到 ~+19。用户要求"障碍识别准"，那就宁可多停
            #: 一会儿等新帧，也不用旧帧凑数。
            if stale_ok is False:
                ev("stale_skip", why="no_pose_at_t", vt_seq=int(s["seq"]),
                   t=float(t_fr) if t_fr is not None else None, py=round(py, 1))
                walls = []
            if (stale_ok is None and pv is not None and not contemporaneous
                    and abs(dy_p) > max(5.0, a.max_stale)):
                ev("stale_skip", dy=round(dy_p, 1), vt_seq=int(s["seq"]),
                   py=round(py, 1))
                walls = []
            if pv is not None and paused and time.monotonic() - pause_t0 < a.settle:
                ev("settle_skip", dx=round(dx_p, 1), vt_seq=int(s["seq"]))
                walls = []
            #: 粗判据只在"离谱"时触发：角色在动时 VT 帧旧 0.5 s，200~300 单位/s
            #: 下 |dx| 天然 100~150（实测把阈值放在 4×tol=120 会误杀 48% 的好帧，
            #: 导致暂停不触发、覆盖率掉到 10/24；完全没有阈值又会放进旧帧导致撞墙）。
            #: 取 tol*20（≈600）：只挡"相机根本没看页面"那种 dx≈500 的情况。
            elif (pv is not None and a.player_tol > 0
                  and abs(dx_p) > a.player_tol * 20):
                geom_rej_n += 1
                ev("geom_reject", dx=round(dx_p, 1), vt_seq=int(s["seq"]),
                   player=[round(float(pv[0]), 1), round(float(pv[1]), 1)],
                   px=round(px, 1), n_vt=len(walls))
                walls = []
            for w in walls:
                gy = vt_world_y(w["y"], py_cap, half, a.y_scale, a.y_bias, a.field_h,
                                a.y_qc)
                ahead = gy - py
                if not (5.0 < gy < a.field_h - 8.0) or ahead < -5.0:
                    continue
                #: 贴近的墙（视野下沿）几何误差更大，缺口再放宽一点。
                margin = a.gap_margin if ahead >= 60.0 else a.gap_margin + 6.0
                #: 单帧原始缺口宽度先过一道闸：本场景墙缺口 90~110（VT 实测
                #: 45~200 属正常）。超出这个范围的多半是"配错行"（把页面白框、
                #: 场外黑区、被角色挡住的段落当成缺口），宽度离谱时宁可丢帧。
                _raw_w = float(w.get("gr", 640.0)) - float(w.get("gl", 0.0))
                if not (45.0 <= _raw_w <= 200.0):
                    ev("reject_gap", why="raw_width", y=round(gy, 1),
                       gl=round(float(w.get("gl", 0.0)), 1),
                       gr=round(float(w.get("gr", 640.0)), 1),
                       w=round(_raw_w, 1))
                    continue
                #: 横向修正按**这道墙在帧内的页面 y** 取（畸变是"画面内位置"的函数；
                #: 用绝对场地 y 会在远处墙上算出上百 px 的荒谬修正）
                _vy_w = float(w.get("y", 0.0))
                _xc = a.x_bias_a + a.x_bias_b * (_vy_w - a.view_h * 0.5)
                gl = float(w.get("gl", 0.0)) - _xc + margin
                gr = float(w.get("gr", 640.0)) - _xc - margin
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
                extras.append((gy, gl, gr, float(w["y"])))
                raw_dbg.append((gy, round(float(w["y"]), 1),
                                round(float(w.get("gl", 0.0)), 1),
                                round(float(w.get("gr", 640.0)), 1)))
            _pv = s.get("player") or [None, None]
            ev("vt_tick", vt_seq=int(s["seq"]), frame=s.get("frame"),
               geom=int(s.get("frame_geom") or 0), n_vt=len(s.get("walls", [])),
               n_used=len(extras), player=s.get("player"),
               player_dx=None if _pv[0] is None else round(float(_pv[0]) - px, 1),
               cand=[[round(e[0], 1), round(e[1], 1), round(e[2], 1)] for e in extras[:6]],
               lag_ms=lag_ms, py_used=round(py_cap, 1))
            raw = list(extras)
            #: 已经进 known 的墙不再是"新墙"，否则恢复运动后立刻又被同一批触发暂停。
            #: 触发"停车测量"前先按**缺口身份**过滤：同一个缺口窗口（±25 px）我们
            #: 已经量过了，就不该再停一次车。以前只按 y 过滤（±55），而同一道墙
            #: 在移动中映射出来的 y 会差 60~150，于是同一道墙被反复"发现"→ 反复
            #: 停车 → 一轮 274 次 wall_same_gap、59 次暂停，时间全耗在停车上。
            #: 回退记录（std8）：试过"按缺口身份预过滤候选"来减少重复停车，结果
            #: 覆盖率 24/24 → 17/24、min_clearance +18.2 → −10 —— 因为相邻两道墙
            #: 的缺口窗口可能落在 ±25 内，把**真的新墙**也滤掉了，板子于是盲开撞墙。
            #: 这个预过滤已删除；减少停车只能从"暂停时长"想办法，不能牺牲候选。
            extras = [e for e in raw
                      if not any(abs(e[0] - k) <= row_tol() for k in known)]
            prev_extras = list(extras)
            extras_t = time.monotonic()
            if extras and not paused and not collecting:
                if a.collect_moving:
                    #: 不停车：墙在可见带里只待几秒，停车会把后面的墙也错过
                    collecting = True
                    pause_t0 = extras_t
                    pause_seen = []
                    ev("collect", cands=[[round(e[0], 1), round(e[1], 1),
                                          round(e[2], 1)] for e in extras[:6]])
                else:
                    paused = True
                    pause_t0 = extras_t
                    pause_seen = []
                    link.send_move(0)      # 立刻停住，等确认
                    ev("pause", cands=[[round(e[0], 1), round(e[1], 1),
                                        round(e[2], 1)] for e in extras[:6]],
                       vt_seq=int(s["seq"]))
                    if a.verbose:
                        print(f"  pause: {[round(e[0], 1) for e in extras]}",
                              file=sys.stderr, flush=True)
            if paused or collecting:
                #: 暂停刚发出时 VT 帧还可能是"动着的"那一帧：等 0.35 s 再开始收，
                #: 之后的帧才是停稳后拍的（否则 y/缺口带着运动模糊的偏差）。
                if time.monotonic() - pause_t0 >= a.settle:
                    #: 注意：不要用 |dy|（帧里角色位置与预期之差）当"同时性"闸门 ——
                    #: 实测它不是抖动而是系统偏 ~101 px（页面可见带实际比 follow_h
                    #: 假设的高），一加就得 0 道墙。留作诊断字段即可。
                    pause_seen.extend(raw)
                del pause_seen[:-30]
                waited = time.monotonic() - pause_t0
                #: 板端 VT 2 Hz，单帧缺口偶尔整体偏几十像素：同一道墙要在 ≥2 帧里
                #: 一致（y±25、缺口±15）才算数；等不到就退化成超时整批收下。
                warmed = (time.monotonic() - t_start) >= a.warmup
                agree = consensus(pause_seen, need=max(2, a.consensus_frames),
                                  tol_vy=a.tol_vy)
                accept = agree if (warmed and waited >= 0.9 and agree) else []
                if force_fix:
                    fixed = 0
                    #: **只用多帧一致的结果**。以前写的是 `(agree or raw)`：等不到
                    #: 一致就退化成"单帧原始值"，于是把别的一行的缺口（实测
                    #: 21.5–172、宽 150）直接改到已知墙上，板子照着它侧移撞墙。
                    for _fix in (agree if agree else []):
                        gy, gl, gr = _fix[0], _fix[1], _fix[2]
                        if not gap_plausible(gl, gr):
                            ev("fix_reject", why="implausible", y=round(gy, 1),
                               gl=round(gl, 1), gr=round(gr, 1))
                            continue
                        near = [d for d in known_detail
                                if abs(float(d["y"]) - gy) <= 70.0]
                        if not near:
                            continue
                        d = min(near, key=lambda q: abs(float(q["y"]) - gy))
                        if (abs(float(d["gl"]) - gl) > 0.5
                                or abs(float(d["gr"]) - gr) > 0.5):
                            #: "重测"是细化，不是搬家：中心跳 >70 说明配错行了
                            _dc = abs(0.5 * (float(d["gl"]) + float(d["gr"]))
                                      - 0.5 * (gl + gr))
                            if _dc > 70.0:
                                ev("fix_reject", why="jump",
                                   y=round(gy, 1), gl=round(gl, 1), gr=round(gr, 1),
                                   old_gl=round(float(d["gl"]), 1),
                                   old_gr=round(float(d["gr"]), 1),
                                   dc=round(_dc, 1))
                                continue
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
                        collecting = False
                        prev_extras = []
                        pause_seen = []
                        last_prog_py, last_prog_t = py, time.monotonic()
                        link.send_move(nudge_act if time.monotonic() < nudge_until
                                       else last_act)
                #: 这里以前写的是 `if paused:` —— 于是"边走边收"收到了好数据也
                #: **永远不入表**（实测 std2/std3：21 次 collect、候选每帧都一致，
                #: add_wall 却只有开局那 2 道）。入表必须对 collecting 同样生效。
                if paused or collecting:
                    _batch_gaps: list[tuple[float, float, float]] = []
                    for _acc in accept:
                        gy, gl, gr = _acc[0], _acc[1], _acc[2]
                        if any(abs(gy - k) <= row_tol() for k in known):
                            continue
                        #: **按"行身份"去重**（以前按"缺口身份"：gl/gr 相差 ≤25
                        #: 且完全不看 y 就当成同一道墙）。后者会把真的新墙吃掉：
                        #: 实测 acc9 里 wall@1480（gap 240.8–332.8）被 52 次判成
                        #: 已学的 wall@1198 —— 两道墙差 282 单位却"因为缺口像"而
                        #: 不许入表，于是板子盲穿过去（audit_crossings 判为 blind）。
                        #: 现在同一道墙 = y 相差 ≤ row_tol()，缺口只做二次确认。
                        _near = [d for d in known_detail
                                 if abs(float(d["y"]) - gy) <= row_tol()]
                        if _near:
                            d = min(_near, key=lambda q: abs(float(q["y"]) - gy))
                            _dc = abs(0.5 * (float(d["gl"]) + float(d["gr"]))
                                      - 0.5 * (gl + gr))
                            if (abs(float(d["gl"]) - gl) > 0.5
                                    or abs(float(d["gr"]) - gr) > 0.5):
                                #: 同一条墙的重测：只许"细化"，不许搬家
                                if _dc > 4.0 * sig_e:
                                    ev("reject_gap", why="same_row_far_gap",
                                       y=round(gy, 1), gl=round(gl, 1),
                                       gr=round(gr, 1), old_y=round(float(d["y"]), 1),
                                       old_gl=round(float(d["gl"]), 1),
                                       old_gr=round(float(d["gr"]), 1), dc=round(_dc, 1))
                                    continue
                                d["gl"], d["gr"] = round(gl, 1), round(gr, 1)
                            ev("wall_same_row", y=round(gy, 1),
                               old_y=round(float(d["y"]), 1), gl=round(gl, 1),
                               gr=round(gr, 1), refined=bool(
                                   abs(float(d["gl"]) - gl) > 0.5
                                   or abs(float(d["gr"]) - gr) > 0.5))
                            continue
                        _why = gap_suspect(gy, gl, gr)
                        if not _why and gap_width_outlier(gl, gr):
                            _why = "width_outlier"
                        if not _why:
                            for (_by, _bgl, _bgr) in _batch_gaps:
                                if abs(_by - gy) <= row_tol():
                                    _why = "dup_batch"
                                    break
                        if _why:
                            ev("reject_gap", why=_why, y=round(gy, 1),
                               gl=round(gl, 1), gr=round(gr, 1))
                            continue
                        _batch_gaps.append((gy, gl, gr))
                        known.append(gy)
                        known_detail.append({"y": round(gy, 1), "gl": round(gl, 1),
                                             "gr": round(gr, 1),
                                             "source": "board_vt"})
                        _dbg = min(raw_dbg, key=lambda t: abs(t[0] - gy)) if raw_dbg else None
                        ev("add_wall", y=round(gy, 1), gl=round(gl, 1),
                           gr=round(gr, 1), gap_w=round(gr - gl, 1),
                           ahead=round(gy - py, 1), vt_seq=int(s["seq"]),
                           vt_frame=s.get("frame"), waited=round(waited, 2),
                           n_agree=len(accept),
                           vt_vy=None if _dbg is None else _dbg[1],
                           raw_gl=None if _dbg is None else _dbg[2],
                           raw_gr=None if _dbg is None else _dbg[3],
                           margin=round(margin, 1))
                        if a.verbose:
                            print(f"  + wall y={gy:.1f} gap {gl:.0f}-{gr:.0f}",
                                  file=sys.stderr, flush=True)
                    if a.verbose and not accept and waited > 2.5:
                        print(f"  [dbg] seen={len(pause_seen)} agree={len(agree)} "
                              f"raw={[round(v[0], 1) for v in raw[:3]]}",
                              file=sys.stderr, flush=True)
                    #: 用本次"同一道墙"的原始帧散度更新噪声估计（EWMA）
                    if accept and len(pause_seen) >= 3:
                        _gy0 = accept[0][0]
                        _grp = [o for o in pause_seen
                                if abs(o[0] - _gy0) <= row_tol()]
                        if len(_grp) >= 3:
                            import statistics as _st
                            _sy = _st.pstdev([o[0] for o in _grp])
                            _se = _st.pstdev([o[i] for o in _grp for i in (1, 2)])
                            sig_y = 0.7 * sig_y + 0.3 * max(_sy, 0.3)
                            sig_e = 0.7 * sig_e + 0.3 * max(_se, 0.3)
                            ev("noise_est", sig_y=round(sig_y, 2), sig_e=round(sig_e, 2),
                               n=len(_grp))
                    known.sort(key=lambda v: v)
                    remain = [e for e in extras
                              if not any(abs(e[0] - k) <= 30.0 for k in known)]
                    if ((accept and not remain)
                            or (waited > 4.5 and warmed and (known or waited > 10.0))):
                        added_gaps = [(a_[1], a_[2]) for a_ in accept]
                        if added_gaps:
                            cooldown_gaps[:] = (cooldown_gaps + added_gaps)[-12:]
                            cooldown_until = time.monotonic() + 2.0
                        paused = False
                        collecting = False
                        prev_extras = []
                        pause_seen = []
                        last_prog_py, last_prog_t = py, time.monotonic()
                        link.send_move(last_act)
                        ev("resume", kind="accepted", n_known=len(known),
                           next_act=int(last_act))
        #: 暂停超时要在主循环里判：VT 断流时（没有新帧）也要能恢复。
        if ((paused or collecting) and time.monotonic() - pause_t0 > a.pause_timeout
                and time.monotonic() - t_start >= a.warmup
                and (known or time.monotonic() - pause_t0 > 10.0)):
            #: **必须同时清 collecting**：否则"(paused or collecting)"永远为真，
            #: 这个分支会每轮迭代都打一次 resume（实测 3418 次/38s ≈ 90 Hz），
            #: 而"发现新墙 → 开始收"的条件里有 `not collecting`，从此再也不会触发
            #: ——整轮一道墙都学不到，板子以为全场没墙，直线穿墙到终点（sparse7）。
            paused = False
            collecting = False
            pause_t0 = time.monotonic()
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
