#!/usr/bin/env python3
"""在 PC 上按板子控制台发的 `@MOVE <n>` 注入方向键 —— **不用 RP2040**。

板子算完决策就把动作号打到控制台（`@MOVE n`），而这根控制台线在 PC 上本来
就能读到（CP210x）。这个脚本把它变成 PC 上的按键，于是**整条闭环不需要额外硬件**：

    板子(PS)：视觉 → 决策 → "@MOVE n" ──控制台──▶ CP210x ──USB──▶ 本脚本
                                                                    │
                                        Windows：keybd_event 注入方向键
                                                                    ▼
                                                        平台里的角色移动

## 与 RP2040 方案的关系

| | 本脚本（软件注入） | RP2040（真 HID 键盘） |
|---|---|---|
| 额外硬件 | **不要** | RP2040 + 两根线（或 PC 转发） |
| 普通窗口程序 | ✅ 能进 | ✅ 能进 |
| **DirectInput/原始输入的游戏** | ❌ **忽略合成输入** | ✅ 因为是"真键盘" |
| 延迟 | 一次 USB 往返 + 注入 | 类似 |

所以：**先用这个脚本实测平台收不收**。能用就先用它把闭环跑起来；
用不了（多半是游戏用原始输入）再回去做 RP2040 —— 那条路的板子侧已经验过了。

## 用法

    # 1) 干跑：只打印它要按的键，不注入（**默认就是干跑**，防误触）
    python3 pc_key_driver.py --console /dev/ttyUSB0

    # 2) 真注入（会往当前焦点窗口按键！先把平台窗口点出来）
    python3 pc_key_driver.py --console /dev/ttyUSB0 --arm

    # 平台用 WASD 而不是方向键
    python3 pc_key_driver.py --console /dev/ttyUSB0 --arm --keys wasd

⚠ **控制台同一时刻只能被一个程序读**：跑这个脚本时别再开着 MobaXterm 读 COM3。
⚠ `--arm` 会真的按键。先把平台窗口点成焦点，别让按键打进编辑器里。
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

# 解析器只有一份：与 console_relay / code.py / C++ 发射器同一个协议
from console_relay import PREFIX, _open_serial, parse_moves  # noqa: E402

from bullet_sim.core.actions import DISCRETE_ACTION_NAMES, DISCRETE_DIRECTIONS  # noqa: E402

#: 方向名 → Windows 虚拟键码（VK）。
#: 注意这里**没有**"动作号 → 按键"的表 —— 那个由 DISCRETE_DIRECTIONS 推出来
#: （见 `keys_for_action`），免得又多一份真值。`code.py` 的 KEYS 表也是同一个
#: 方向来源，`tests/test_pc_key_driver.py` 会把两边对起来。
VK = {
    "arrows": {"LEFT": 0x25, "UP": 0x26, "RIGHT": 0x27, "DOWN": 0x28},
    "wasd":   {"LEFT": 0x41, "UP": 0x57, "RIGHT": 0x44, "DOWN": 0x53},  # A W D S
}

DEFAULT_TIMEOUT_MS = 500          # 与 code.py 的 TIMEOUT_MS 一致：断流就全松


def keys_for_action(action: int, keymap: str = "arrows") -> tuple[int, ...]:
    """动作号 → 要按住的虚拟键码元组。

    方向**不另立一张表**：直接读协议的 `DISCRETE_DIRECTIONS`（那份顺序本身就是
    公开协议），再翻成虚拟键码。`code.py` 的 KEYS 表用的也是同一份方向真值。
    """
    if not 0 <= action < len(DISCRETE_ACTION_NAMES):
        return ()
    x, y = float(DISCRETE_DIRECTIONS[action][0]), float(DISCRETE_DIRECTIONS[action][1])
    names = []
    if y > 0:
        names.append("UP")
    if y < 0:
        names.append("DOWN")
    if x < 0:
        names.append("LEFT")
    if x > 0:
        names.append("RIGHT")
    table = VK[keymap]
    return tuple(table[n] for n in names)


def key_diff(held: tuple[int, ...], want: tuple[int, ...]) -> tuple[list[int], list[int]]:
    """算出"从 held 切到 want"要按哪些、松哪些。

    只动差集（与 `code.py` 的 `apply()` 同一策略）：换向时共用的那个键不松开，
    免得平台把它当成"松了又按"而重新起步 —— 斜向切斜向时抖动就是这么来的。
    """
    press = [k for k in want if k not in held]
    release = [k for k in held if k not in want]
    return press, release


def _spawn_injector(quiet: bool = True) -> subprocess.Popen:
    """起常驻的 PowerShell 注入器。

    每次按键都起一个进程要几十毫秒，30 fps 跟不上，所以起一次、逐行喂命令。
    从 WSL 调 Windows 的 powershell.exe（互操作），路径要转成 Windows 形式。
    """
    ps1 = HERE / "win_sendinput.ps1"
    try:
        win_path = subprocess.run(["wslpath", "-w", str(ps1)],
                                  capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        win_path = str(ps1)
    cmd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
           "-File", win_path]
    if quiet:
        cmd.append("-Quiet")
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE, text=True, bufsize=1)


def main() -> int:
    ap = argparse.ArgumentParser(description="板子控制台 @MOVE → PC 按键注入")
    ap.add_argument("--console", required=True,
                    help="板子控制台串口（/dev/ttyUSB0、COM3、socket://host:port）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--keys", choices=sorted(VK), default="arrows",
                    help="平台认哪套键（默认方向键）")
    ap.add_argument("--arm", action="store_true",
                    help="**真的注入按键**（默认只干跑打印）。会打到当前焦点窗口，先点出平台窗口")
    ap.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS,
                    help="断流多久就全松（默认 500ms，与 code.py 一致）")
    args = ap.parse_args()

    con = _open_serial(args.console, args.baud)
    inj = None
    if args.arm:
        try:
            inj = _spawn_injector(quiet=True)
        except FileNotFoundError:
            sys.exit("找不到 powershell.exe —— 这个脚本要在 WSL 里跑（Windows 互操作）")
        time.sleep(1.0)                      # 等 Add-Type 编完
        if inj.poll() is not None:
            err = inj.stderr.read() if inj.stderr else ""
            sys.exit(f"注入器起不来：\n{err}")

    print(f"控制台 {args.console}  键位 {args.keys}  "
          f"{'**真注入**' if args.arm else '干跑（加 --arm 才真的按键）'}"
          f"   Ctrl-C 退出")

    buf = bytearray()
    held: tuple[int, ...] = ()
    last_ok = time.monotonic()
    n_actions = 0

    def do(press: list[int], release: list[int]) -> None:
        for k in press:
            if inj:
                inj.stdin.write(f"DN {k}\n"); inj.stdin.flush()
        for k in release:
            if inj:
                inj.stdin.write(f"UP {k}\n"); inj.stdin.flush()
        if press or release:
            tag = f"  [{' '.join(f'{k:#04x}' for k in press)}] " \
                  f"-[{' '.join(f'{k:#04x}' for k in release)}]"
            print(f"  → 按住 {press} 松开 {release}{tag if not inj else ''}", flush=True)

    try:
        while True:
            chunk = con.read(256)
            if chunk:
                for n in parse_moves(chunk, buf):
                    want = keys_for_action(n, args.keys)
                    p, r = key_diff(held, want)
                    do(p, r)
                    held = want
                    last_ok = time.monotonic()
                    n_actions += 1
            # 看门狗：断流（板子崩了/线断了/脚本被打断）就把键全松开，
            # 否则角色会一直朝最后那个方向走。与 code.py 同一条规矩。
            if held and (time.monotonic() - last_ok) * 1000.0 > args.timeout_ms:
                do([], list(held))
                held = ()
                print("  [看门狗] 断流，已全松", flush=True)
    except KeyboardInterrupt:
        print(f"\n收到 {n_actions} 个动作后退出")
    finally:
        if inj:
            try:
                inj.stdin.write("ALLUP\nQUIT\n"); inj.stdin.flush()
                inj.wait(timeout=3)
            except Exception:
                inj.kill()
        con.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
