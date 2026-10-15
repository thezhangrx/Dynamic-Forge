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

    # 0) 先问"注入这条路通不通"（不按键、不用板子，1 秒出结果）
    python3 pc_key_driver.py --probe

    # 1) 干跑：只打印它要按的键，不注入（**默认就是干跑**，防误触）
    python3 pc_key_driver.py --console /dev/ttyUSB0

    # 2) 真注入（会往当前焦点窗口按键！先把平台窗口点出来）
    python3 pc_key_driver.py --console /dev/ttyUSB0 --arm

    # 3) 平台到底收不收合成按键？四个方向依次按一遍，倒计时留出点窗口的时间：
    #    跑起来后 5 秒内把平台窗口点成焦点，看角色动不动、动的是不是那个方向
    python3 pc_key_driver.py --press 1,2,3,4 --arm --delay-s 5
    # 平台用 WASD 就把 --keys wasd 加上；先去掉 --arm 干跑一遍确认键位

    # 4) 零接线：板子用 --keyboard-dev udp://<本机IP>:9999 直发，这里收 UDP
    python3 pc_key_driver.py --udp 9999 --arm

    # 5) **最省事的一条**：走已有的 SSH 通道，板子把动作打到 stdout，管道喂进来。
    #    不用接引脚、不用 usbipd 挂串口、也不用动 Windows 防火墙：
    ssh -i <key> root@192.168.137.242 \
        'cd /root/board_src/build && ./vision_board_devmem --via-pl --frames 0 \
         --keyboard-dev stdout' \
      | python3 pc_key_driver.py --stdin --arm

    注意 `--keyboard-dev stdout` 必须**成对**用 `--stdin` 收：两边是同一个协议，
    管道里混着的日志（"帧 3: …"）会被解析器按前缀自动丢掉。

    # 平台用 WASD 而不是方向键
    python3 pc_key_driver.py --console /dev/ttyUSB0 --arm --keys wasd

`--press` 是判定"平台收不收合成输入"的唯一可靠办法：**一次按键、你能亲眼看到**，
比看日志猜强。给一串动作号（`--press 1,2,3,4`）它会把四个方向依次按一遍，
一次运行就同时回答"收不收"和"键位对不对"。按完就退出，不会把键卡住。

⚠ **控制台同一时刻只能被一个程序读**：跑这个脚本时别再开着 MobaXterm 读 COM3。
⚠ `--arm` 会真的按键。先把平台窗口点成焦点，别让按键打进编辑器里。
"""

from __future__ import annotations

import argparse
import os
import pathlib
import select
import socket
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


def probe() -> int:
    """只验证"PowerShell 注入器能不能起来"，不按任何键。

    失败原因通常就三种：不在 WSL 里跑（没有 powershell.exe）、
    Windows 那边禁了脚本执行、或者 WSL 互操作被关掉了。
    先在这里把它排除掉，免得后面调平台时误判成"平台不认合成按键"。
    """
    print(f"注入器脚本：{HERE / 'win_sendinput.ps1'}")
    try:
        inj = _spawn_injector(quiet=True)
    except FileNotFoundError:
        print("✗ 找不到 powershell.exe —— 要在 WSL 里跑（Windows 互操作）")
        return 1
    time.sleep(1.0)                      # 等 Add-Type 编完
    if inj.poll() is not None:
        err = (inj.stderr.read() if inj.stderr else "") or "(无输出)"
        print(f"✗ 注入器起不来（退出码 {inj.returncode}）：\n{err}")
        return 1
    try:
        inj.stdin.write("ALLUP\nQUIT\n"); inj.stdin.flush()
        inj.wait(timeout=3)
    except Exception:
        inj.kill()
    print("✓ PowerShell 注入器可用 —— 合成按键这条路是通的")
    print("  下一步：平台窗口点出来，然后 --press 1,2,3,4 --arm --delay-s 5")
    return 0


def press_sequence(actions: list[int], keymap: str, delay_s: float, hold_ms: int,
                   gap_ms: int, arm: bool) -> int:
    """依次单发几个动作（按住 → 松开 → 间隔），用来肉眼判定平台收不收合成按键。

    为什么要能"依次发几个"：一次运行就能回答两个问题 ——
    ① 平台到底收不收合成按键（有一个方向动了就是收）；② 这套键位对不对
    （四个方向都试，动的是不是你按的那个）。单发一个的话，得跑好几轮才知道。
    """
    names = []
    for a in actions:
        if not 0 <= a < len(DISCRETE_ACTION_NAMES):
            print(f"动作号 {a} 越界（协议只有 0..{len(DISCRETE_ACTION_NAMES) - 1}）")
            return 2
        if not keys_for_action(a, keymap):
            # 动作 0 是 stay —— 合法，但它本来就不按键，拿它测"平台收不收"没意义
            print(f"动作 {a}（{DISCRETE_ACTION_NAMES[a]}）不按任何键，"
                  f"换 1..{len(DISCRETE_ACTION_NAMES) - 1} 里的来测")
            return 2
        names.append(DISCRETE_ACTION_NAMES[a])

    inj = None
    if arm:
        try:
            inj = _spawn_injector(quiet=True)
        except FileNotFoundError:
            sys.exit("找不到 powershell.exe —— 这个脚本要在 WSL 里跑（Windows 互操作）")
        time.sleep(1.0)                  # 等 Add-Type 编完
        if inj.poll() is not None:
            err = inj.stderr.read() if inj.stderr else ""
            sys.exit(f"注入器起不来：\n{err}")

    print(f"键位 {keymap}  依次按 {actions}（{', '.join(names)}）"
          f"  每个按住 {hold_ms} ms、间隔 {gap_ms} ms"
          f"{'' if arm else '  [干跑，加 --arm 才真的按]'}", flush=True)
    if delay_s > 0:
        print(f"  {delay_s:g} 秒后开始，现在把平台窗口点成焦点 …", flush=True)
        time.sleep(delay_s)

    def lines(press: list[int], release: list[int]) -> None:
        if inj is None:
            return
        for k in press:
            inj.stdin.write(f"DN {k}\n"); inj.stdin.flush()
        for k in release:
            inj.stdin.write(f"UP {k}\n"); inj.stdin.flush()

    try:
        for idx, a in enumerate(actions, 1):
            want = keys_for_action(a, keymap)
            print(f"  [{idx}/{len(actions)}] 动作 {a}（{DISCRETE_ACTION_NAMES[a]}）"
                  f" → 按住 {list(want)}", flush=True)
            lines(list(want), [])
            time.sleep(max(0, hold_ms) / 1000.0)
            lines([], list(want))
            if idx < len(actions) and gap_ms > 0:
                time.sleep(gap_ms / 1000.0)
    finally:
        if inj is not None:
            try:
                inj.stdin.write("ALLUP\nQUIT\n"); inj.stdin.flush()
                inj.wait(timeout=3)
            except Exception:
                inj.kill()
    if arm:
        print("  逐个看：有方向动了 → 平台收合成按键，RP2040 不用买了；")
        print("  动了但不是你按的那个方向 → 换一套键位（--keys wasd / --keys arrows）再试；")
        print("  完全不动 → 平台多半走原始输入，回去做 RP2040（README 路径 A/B）。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="板子 @MOVE → PC 按键注入（不用 RP2040）")
    ap.add_argument("--console",
                    help="板子控制台串口（/dev/ttyUSB0、COM3、socket://host:port）")
    ap.add_argument("--udp", type=int, metavar="PORT",
                    help="改成**收 UDP**：板子用 --keyboard-dev udp://<本机IP>:PORT 直发。"
                         "零接线，也不和控制台日志抢串口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--stdin", action="store_true",
                    help="从标准输入读 `@MOVE` 行（配 SSH 管道："
                         "板子 --keyboard-dev stdout | 本脚本 --stdin）")
    ap.add_argument("--keys", choices=sorted(VK), default="arrows",
                    help="平台认哪套键（默认方向键）")
    ap.add_argument("--arm", action="store_true",
                    help="**真的注入按键**（默认只干跑打印）。会打到当前焦点窗口，先点出平台窗口")
    ap.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS,
                    help="断流多久就全松（默认 500ms，与 code.py 一致）")
    ap.add_argument("--probe", action="store_true",
                    help="只检查注入器能不能起来（不按键、不用板子），然后退出")
    ap.add_argument("--press", metavar="ACTION[,ACTION...]",
                    help="依次发这些动作号后退出（如 --press 1,2,3,4 四个方向都试一遍）；"
                         "配合 --delay-s/--hold-ms/--gap-ms 用来判定平台收不收合成按键")
    ap.add_argument("--delay-s", type=float, default=3.0,
                    help="--press 前留多少秒给你点平台窗口（默认 3）")
    ap.add_argument("--hold-ms", type=int, default=300,
                    help="--press 每个动作按住多久（默认 300ms）")
    ap.add_argument("--gap-ms", type=int, default=400,
                    help="--press 相邻两个动作之间隔多久（默认 400ms）")
    args = ap.parse_args()

    if args.probe:
        return probe()
    if args.press is not None:
        try:
            acts = [int(t) for t in args.press.replace(",", " ").split()]
        except ValueError:
            ap.error(f"--press 只接受动作号（逗号或空格分隔），拿到 {args.press!r}")
        if not acts:
            ap.error("--press 后面要跟至少一个动作号")
        return press_sequence(acts, args.keys, args.delay_s, args.hold_ms,
                              args.gap_ms, args.arm)
    n_src = sum([bool(args.console), args.udp is not None, args.stdin])
    if n_src == 0:
        ap.error("要么给 --console / --udp / --stdin，要么用 --probe / --press 自检")
    if n_src > 1:
        ap.error("--console / --udp / --stdin 只能选一个（三路都是动作来源）")

    sk = None
    if args.stdin:
        con = None
        src = "标准输入（SSH 管道）"
    elif args.udp is not None:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sk.bind(("0.0.0.0", args.udp))
        sk.settimeout(0.05)              # 短超时：否则看门狗没机会跑
        con = None
        src = f"UDP :{args.udp}"
    else:
        con = _open_serial(args.console, args.baud)
        src = f"控制台 {args.console}"
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

    print(f"{src}  键位 {args.keys}  "
          f"{'**真注入**' if args.arm else '干跑（加 --arm 才真的按键）'}"
          f"   Ctrl-C 退出", flush=True)

    def read_chunk() -> bytes | None:
        """UDP / 串口 / 标准输入统一成一个"可能返回空"的读接口，看门狗才能插进来。

        返回 `None` 专门表示**输入结束**（管道那头关了）：串口/UDP 没有"结束"这个
        状态，只有 stdin 有。EOF 时不能一直空转，得干净收工并把键松开。
        """
        if sk is not None:
            try:
                return sk.recv(512)
            except socket.timeout:
                return b""
        if con is not None:
            return con.read(256)
        # stdin：管道是块缓冲的，select 能等到"有数据或 EOF"，两边都不空转
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not r:
            return b""
        data = os.read(sys.stdin.fileno(), 512)
        return None if not data else data

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
            chunk = read_chunk()
            if chunk is None:
                print("输入结束（管道那头关了），收工", flush=True)
                break
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
        if sk is not None:
            sk.close()
        if con is not None:
            con.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
