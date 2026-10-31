#!/usr/bin/env python3
"""flow_key.py —— PC 侧：盯板子上的 PS_KEY，按一下开跑、再按一下干净收尾

    python3 core/board/tools/flow_key.py
    python3 core/board/tools/flow_key.py --cmd "bash core/board/tools/loop_standard.sh --tag btn"
    python3 core/board/tools/flow_key.py --dry-run        # 只打印，不真跑
    python3 core/board/tools/flow_key.py --no-hotkeys     # 不用 PC 键盘撒手锏

除了板子上的 PS_KEY，**PC 键盘也能控**（终端里）：`s` 开始 / `x` 停止 / `q` 退出。
板子按键还没查到引脚时，这就是保底手段。

做三件事
  1. 每 0.5 s 通过 ssh 读板子 `/run/flow/press` 的计数，变了就翻转状态；
  2. **开跑**：起 `--cmd`（独立进程组），然后往板子写 `run=1` → 板上指示灯亮；
  3. **停止**：`SIGTERM` 整个进程组 → 等 → 不退再 `SIGINT` → 按 T0c 收尾
     （板端 ffmpeg/pl_vision/vt_sparse/mjpeg_server/board_agent/field_cli_board 全 TERM；
       本机 platform_host/page_view/page_display 也清；最后查 8498/9006/8099 不在 LISTEN），
     收尾完写 `run=0` → 灯灭。

为什么不许 kill -9：R5。停不下来就先查为什么（通常是有 CHILD 没收到 TERM）。
"""
from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import subprocess
import termios
import time
import tty

BOARD = "root@192.168.137.242"
KEY = "data/fpga/board_ssh/board_key"
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

BOARD_PATTERNS = [
    "[f]fmpeg", "[p]l_vision", "[v]t_sparse", "[v]t_board", "[m]jpeg_server.py",
    "[b]oard_agent.py", "[f]ield_cli_board", "[a]gent_bridge.py", "[r]un_flow.sh",
    "[s]tart_cam.sh",
]
LOCAL_PATTERNS = ["[p]latform_host.py", "[p]age_view.py"]


class Hotkeys:
    """PC 终端单键：s=开始 x=停止 q=退出（raw 模式，不用回车）"""

    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else -1
        self.old = None

    def __enter__(self):
        if self.enabled:
            try:
                self.old = termios.tcgetattr(self.fd)
                tty.setraw(self.fd)
                os.set_blocking(self.fd, False)
            except (termios.error, OSError):
                self.enabled = False
        return self

    def __exit__(self, *exc):
        if self.enabled and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def poll(self) -> str:
        if not self.enabled:
            return ""
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return ""
        try:
            ch = os.read(self.fd, 1).decode(errors="ignore")
        except (OSError, BlockingIOError):
            return ""
        return ch


def ssh(cmd: str, timeout: float = 8.0) -> str:
    try:
        r = subprocess.run(
            ["ssh", "-i", os.path.join(ROOT, KEY), "-o", "StrictHostKeyChecking=no",
             "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BOARD, cmd],
            capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except (subprocess.TimeoutExpired, OSError):
        return ""


def read_press() -> int:
    out = ssh("cat /run/flow/press 2>/dev/null | head -1")
    try:
        return int(out.split()[0])
    except (IndexError, ValueError):
        return -1


def set_run(on: bool) -> None:
    ssh("mkdir -p /run/flow; echo %d > /run/flow/run" % (1 if on else 0))


def sweep_board() -> None:
    for pat in BOARD_PATTERNS:
        ssh("pkill -TERM -f '%s' 2>/dev/null; true" % pat, timeout=6)
    time.sleep(0.6)
    for pat in BOARD_PATTERNS:
        ssh("pkill -INT -f '%s' 2>/dev/null; true" % pat, timeout=6)
    time.sleep(0.4)
    left = ssh("ps -eo comm | grep -cE '^(ffmpeg|pl_vision|vt_sparse|vt_board|board_agent)$'")


def sweep_local() -> None:
    for pat in LOCAL_PATTERNS:
        subprocess.run(["pkill", "-TERM", "-f", pat], capture_output=True)
    time.sleep(0.5)
    subprocess.run(["pkill", "-INT", "-f", "[p]age_display.py"], capture_output=True)


def report() -> None:
    cnt = (ssh("ps -eo args | grep -cE '([f]fmpeg|[p]l_vision|[v]t_sparse|[m]jpeg_server|"
              "[b]oard_agent|[f]ield_cli_board)'") or "?").strip()
    ports = ssh("ss -ltn 2>/dev/null | grep -cE ':(8498|9006|8099)\\b'") or "?"
    local = subprocess.run(["bash", "-c",
                            "ps -eo args | grep -cE '([p]latform_host|[p]age_view|[p]age_display)'"],
                           capture_output=True, text=True).stdout.strip()
    print("  收尾结果：板端相关进程 %s 个 ｜ 本机平台进程 %s 个 ｜ 8498/9006/8099 LISTEN %s 个"
          % (cnt, local, ports))


class Runner:
    """管"全流程"这一个进程组；active 是逻辑状态（dry-run 也一致）"""

    def __init__(self, cmd: str, dry: bool, grace: float):
        self.cmd, self.dry, self.grace = cmd, dry, grace
        self.proc: subprocess.Popen | None = None
        self.active = False

    @property
    def running(self) -> bool:
        return self.active

    @property
    def self_exited(self) -> bool:
        return (self.active and self.proc is not None and self.proc.poll() is not None)

    def start(self) -> None:
        if self.active:
            return
        print("[flow] 开始：%s" % self.cmd)
        self.active = True
        if self.dry:
            return
        self.proc = subprocess.Popen(self.cmd, shell=True, cwd=ROOT,
                                     start_new_session=True)
        set_run(True)

    def stop(self, why: str = "") -> None:
        if not self.active:
            return
        print("[flow] 停止%s" % (("：" + why) if why else ""))
        self.active = False
        if not self.dry and self.proc is not None:
            if self.proc.poll() is not None:
                print("[flow] 进程已自己退出（code=%s），只做收尾" % self.proc.returncode)
            else:
                print("[flow] SIGTERM 进程组 %d" % self.proc.pid)
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                t0 = time.time()
                while time.time() - t0 < self.grace and self.proc.poll() is None:
                    time.sleep(0.2)
                if self.proc.poll() is None:
                    print("[flow] %.0fs 没退，改 SIGINT（不许 kill -9）" % self.grace)
                    try:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    t0 = time.time()
                    while time.time() - t0 < 8 and self.proc.poll() is None:
                        time.sleep(0.2)
        if self.dry:
            print("[flow] （dry-run，没有真进程/收尾）")
            self.proc = None
            return
        print("[flow] 板端 + 本机收尾（T0c）")
        sweep_board()
        sweep_local()
        report()
        set_run(False)
        self.proc = None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", default="bash core/board/tools/loop_standard.sh --tag btn",
                    help="开跑时执行的命令")
    ap.add_argument("--poll", type=float, default=0.5, help="轮询按键的周期 s")
    ap.add_argument("--grace", type=float, default=20.0, help="SIGTERM 后等多久再 SIGINT")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不真起进程")
    ap.add_argument("--no-hotkeys", action="store_true", help="不用 PC 键盘 s/x 控制")
    a = ap.parse_args()

    print("flow_key: 轮询 %s:/run/flow/press（每 %.1fs），命令 = %s" % (BOARD, a.poll, a.cmd))
    print("          板子 PS_KEY：开始 / 再按一次停止")
    print("          PC 键盘：s 开始 / x 停止 / q 退出（终端里直接按，不用回车）")
    run = Runner(a.cmd, a.dry_run, a.grace)

    last = read_press()
    if last < 0:
        print("⚠ 读不到 /run/flow/press：板上 flow_keyd 起了吗？"
              "（systemctl status flow-keyd）先当 0 处理", file=sys.stderr)
        last = 0
    set_run(False)

    hk = Hotkeys(not a.no_hotkeys)
    hk.__enter__()
    try:
        while True:
            ch = hk.poll()
            if ch:
                if ch in ("s", "S"):
                    print("[flow] PC 键 s → 开始")
                    run.start()
                elif ch in ("x", "X"):
                    print("[flow] PC 键 x → 停止")
                    run.stop("PC 键 x")
                elif ch in ("q", "Q", "\x03"):
                    raise KeyboardInterrupt
            time.sleep(a.poll)
            if run.self_exited:
                run.stop("进程自己退出了（code=%s）" % run.proc.returncode)
            cur = read_press()
            if cur < 0 or cur == last:
                continue
            last = cur
            print("[flow] 收到第 %d 次按键" % cur)
            if run.running:
                run.stop("再按一次 PS_KEY")
            else:
                run.start()
    except KeyboardInterrupt:
        print("\n[flow] 退出 → 收尾")
        run.stop("退出")
    finally:
        hk.__exit__()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
