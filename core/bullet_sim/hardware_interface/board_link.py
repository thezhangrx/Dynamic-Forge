"""板子 → 仿真器的**动作信号**链路（`@MOVE n` → ``Action``）。

## 协议已经不是 TBD 了

`tbd.py` 里的 `HARDWARE_INPUT_INTERFACE_TBD` 是"还没选板子"时的占位。现在板子和
传输都定了（见 `core/board/action_codec.hpp` 与 `core/fpga/docs/板上验证.md`）：

* **线格式**：一行一个动作号 —— ``@MOVE <n>\\n``，``n`` 取 0..8（9 个离散动作）；
* **传输**：板子控制台 UART / stdout（SSH 管道）/ UDP，三选一；
* **规矩**：与键盘桥完全一样 —— 只认**行首**就是 ``@MOVE `` 的行（这条线上混着
  内核日志/程序日志），前缀后必须是 0..8 的整数，否则**静默丢弃**；**断流就看门狗
  回中**（否则平台会一直朝最后那个方向跑）。

所以本模块是 `PlaceholderHardwareInputAdapter` 的**真实现**，不是新的猜测：
解析规则与 `core/rp2040_keyboard/console_relay.py:parse_moves` 逐条一致
（`core/bullet_sim/tests/test_board_link.py` 会把两边对起来跑同一批字节）。

## 为什么不 import console_relay

`core/rp2040_keyboard/` 不是包（没有 `__init__.py`），而且那是**工具目录**；
`bullet_sim` 依赖它会把分层弄反（依赖只能从专用层流向通用层）。所以这里自己写
一份十几行的解析器，用**测试**钉住两边行为一致 —— 和仓库里 RTL/Python/code.py
三份实现互相钉住是同一个套路。
"""

from __future__ import annotations

import os
import select
import socket
import time
from typing import Any

from bullet_sim.action.base import coerce_action
from bullet_sim.action.types import Action
from bullet_sim.hardware_interface.input_adapter import HardwareInputAdapter

#: 动作行前缀。**必须**与 `action_codec.hpp` / `code.py` / `console_relay.py` 一致。
PREFIX = b"@MOVE "

#: 离散动作号的上界（0..8 共 9 个）。与 `DISCRETE_ACTION_NAMES` 对齐。
MAX_N = 8

#: 半行残渣的上界：控制台日志量大时，没有换行的残渣不能越积越多（会吃光内存）。
_BUF_LIMIT = 4096

#: 断流多久回中（秒）。与键盘桥/`code.py` 同一条规矩：500 ms。
DEFAULT_HOLD_S = 0.5


def parse_moves(chunk: bytes, buf: bytearray) -> list[int]:
    """从任意字节块里挑出动作号；行为与 `console_relay.parse_moves` **逐条一致**。

    规则（少一条就会"看起来也能跑，只是偶尔不动"）：

    * 只按 ``\\n`` 拆行，半行留到下一批（串口/UDP 都会把一行切开）；
    * 只认**行首**就是 ``@MOVE `` 的行（这条线上混着日志）；
    * 前缀后必须是 0..8 的整数，否则静默丢弃（``@MOVE 9`` / ``@MOVE -1`` /
      ``@MOVE 2.5`` / ``@MOVE`` 都丢）；
    * 残渣超过上界就丢掉（否则没有换行的日志会把内存吃光）。
    """
    buf.extend(chunk)
    out: list[int] = []
    while True:
        nl = buf.find(b"\n")
        if nl < 0:
            break
        line = bytes(buf[:nl])
        del buf[: nl + 1]
        line = line.rstrip(b"\r")
        if not line.startswith(PREFIX):
            continue
        tail = line[len(PREFIX):].strip()
        try:
            n = int(tail)
        except ValueError:
            continue
        if 0 <= n <= MAX_N:
            out.append(n)
    if len(buf) > _BUF_LIMIT:
        del buf[: len(buf) - _BUF_LIMIT]
    return out


# ---------------------------------------------------------------------------
# 传输：三种都提供 `_read_bytes()`（非阻塞、没数据回 b""）
# ---------------------------------------------------------------------------
class BoardInputAdapter(HardwareInputAdapter):
    """把 `@MOVE n` 字节流变成 `Action` 的通用适配器（子类只管传输）。

    看门狗：**断流超过 `hold_s` 就回中**（返回 stay）。不放的话，板子一断/脚本一停，
    仿真里的角色会一直朝最后那个方向飞 —— 与键盘桥的规矩一致。
    """

    transport = "board"

    def __init__(self, *, hold_s: float = DEFAULT_HOLD_S,
                 warn_after_s: float = 3.0, quiet: bool = False,
                 **kwargs: Any) -> None:
        # ⚠ CLI 会把一堆通用参数一起塞进来（stdin/udp/port/seed/...），只把基类认识的
        #   传下去 —— 否则一句 "unexpected keyword argument 'stdin'" 就会把整次演示
        #   卡在启动阶段，而错误信息看上去跟"板子没信号"毫无关系。
        base = {k: v for k, v in kwargs.items() if k in ("transport", "frame_format")}
        # ⚠ 这里**不再**走 `PlannedTransport.require()`：协议已经定了，不是占位。
        super().__init__(**base)
        self.hold_s = float(hold_s)
        #: 打开后多少秒还没收到**任何**动作行就提醒一次（0 = 不提醒）。
        #: 为什么要有：板子静默时画面和"板子在发 stay"**长得一模一样**，
        #: 没有这句话人只能盯着不动的角色猜（已经这么猜过一次了）。
        self.warn_after_s = float(warn_after_s)
        self.quiet = bool(quiet)
        self._warned_no_signal = False
        self._opened_t = 0.0
        self._buf = bytearray()
        self._pending: list[int] = []
        self._last_action: Action = Action.zero()
        self._last_line_t = 0.0
        self.n_actions = 0          # 累计收到的动作行
        self.n_dropped = 0          # 解析阶段静默丢弃的行（诊断用）
        self.n_holds = 0            # 看门狗回中次数

    # -- 子类实现 ---------------------------------------------------------
    def _open_transport(self) -> None:      # pragma: no cover - 子类覆盖
        raise NotImplementedError

    def _read_bytes(self) -> bytes:         # pragma: no cover - 子类覆盖
        raise NotImplementedError

    def _close_transport(self) -> None:
        pass

    # -- 生命周期 ---------------------------------------------------------
    def open(self) -> None:
        self._open_transport()
        self._buf.clear()
        self._pending.clear()
        self._last_action = Action.zero()
        self._last_line_t = time.monotonic()      # 刚打开时不该立刻判成"断流"
        self._opened_t = self._last_line_t
        self._warned_no_signal = False
        self._open = True

    def close(self) -> None:
        self._close_transport()
        self._open = False

    # -- 数据通路 ---------------------------------------------------------
    def read_payload(self) -> Any:
        """返回一个待执行的动作号（int），没有新动作就返回 None。"""
        chunk = self._read_bytes()
        if chunk:
            got = parse_moves(chunk, self._buf)
            if got:
                self.n_actions += len(got)
                self._pending.extend(got)
                self._last_line_t = time.monotonic()
        if not self._pending:
            return None
        return self._pending.pop(0)

    def decode_payload(self, payload: Any) -> Action:
        """动作号 → ``Action``（走仿真现成的离散解码，不另立一张表）。"""
        return coerce_action(int(payload))

    def _warn_if_silent(self) -> None:
        if (self.quiet or self._warned_no_signal or self.warn_after_s <= 0
                or self.n_actions > 0 or not self._open):
            return
        if time.monotonic() - self._opened_t < self.warn_after_s:
            return
        self._warned_no_signal = True
        import sys as _sys
        print(f"⚠ 已经 {self.warn_after_s:.0f} 秒没收到任何 @MOVE 行 —— 板子可能在静默。查：\n"
              f"    ① 板子上那条命令真的在跑吗（要看得到 --keyboard-dev stdout）\n"
              f"    ② 传输选对了吗（--stdin / --udp / --port）\n"
              f"    ③ 看门狗是不是把它回中了（断流超过 {self.hold_s:.2f}s）",
              file=_sys.stderr)

    def poll(self, dt: float) -> Action | None:
        """读 + 解码；**没有新动作但断流时返回"回中"**（看门狗）。"""
        action = super().poll(dt)
        self._warn_if_silent()
        if action is not None:
            self._last_action = action
            return action
        if self.hold_s > 0 and self._last_action != Action.zero():
            if (time.monotonic() - self._last_line_t) > self.hold_s:
                self._last_action = Action.zero()
                self.n_holds += 1
                return self._last_action
        return None

    def hud_line(self, now: float | None = None) -> str:
        """给屏幕 HUD 用的一行"板子链路状态"。

        为什么必须有：板子没在发信号时，**画面和"板子在发 stay"长得一模一样** ——
        没有这一行，人就只能盯着不动的角色猜（已经这么猜过一次了）。
        """
        t = time.monotonic() if now is None else now
        age = (t - self._last_line_t) if self._last_line_t else float("inf")
        if self.n_actions == 0:
            return f"board     : **还没收到任何动作行**（{age:.1f}s）"
        d = self._last_action.direction
        who = "stay" if (d[0] == 0.0 and d[1] == 0.0) else f"({d[0]:+.2f},{d[1]:+.2f})"
        tag = "  ⚠断流中" if age > self.hold_s else ""
        return (f"board     : {self.n_actions} 行  最后 {who}  {age:.2f}s 前"
                f"  回中 {self.n_holds} 次{tag}")

    def describe(self) -> dict[str, Any]:
        data = super().describe()
        data.pop("marker", None)          # 协议已定，不再是 TBD
        data.update({
            "protocol": PREFIX.decode("ascii").strip(),
            "n_actions": self.n_actions,
            "holds": self.n_holds,
            "hold_s": self.hold_s,
        })
        return data


class StdinBoardAdapter(BoardInputAdapter):
    """从**标准输入**读（`ssh 板子 '... --keyboard-dev stdout' | python -m bullet_sim ...`）。"""

    transport = "stdin (SSH 管道)"

    def __init__(self, fd: int = 0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.fd = int(fd)

    def _open_transport(self) -> None:
        os.set_blocking(self.fd, False)

    def _read_bytes(self) -> bytes:
        r, _, _ = select.select([self.fd], [], [], 0.0)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""


class UdpBoardAdapter(BoardInputAdapter):
    """收 UDP 报文（板子侧 `--keyboard-dev udp://<本机IP>:<port>`）。"""

    transport = "udp"

    def __init__(self, port: int = 0, host: str = "0.0.0.0", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.host = host
        self.port = int(port)
        self._sock: socket.socket | None = None

    def _open_transport(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self.port = self._sock.getsockname()[1]     # port=0 时回填真实端口
        self._sock.setblocking(False)

    def _read_bytes(self) -> bytes:
        if self._sock is None:
            return b""
        try:
            data, _peer = self._sock.recvfrom(4096)
            return data
        except BlockingIOError:
            return b""

    def _close_transport(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class SerialBoardAdapter(BoardInputAdapter):
    """读串口（板子控制台 CP210x / 或 RP2040 的 USB CDC）。

    ⚠ 同一个串口**同一时刻只能被一个程序打开**：跑这个的时候别再开 MobaXterm。
    """

    transport = "serial"

    def __init__(self, port: str = "", baud: int = 115200, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if not port:
            raise ValueError("serial 传输需要给串口设备名（--port /dev/ttyUSB0）")
        self.port = str(port)
        self.baud = int(baud)
        self._ser: Any = None

    def _open_transport(self) -> None:
        try:
            import serial                     # pyserial
            from serial.tools import list_ports
        except ImportError as exc:            # pragma: no cover - 环境问题
            raise RuntimeError(
                "读串口需要 pyserial：pip install pyserial"
            ) from exc
        try:
            self._ser = serial.serial_for_url(self.port, self.baud, timeout=0)
        except Exception as exc:
            # 换成能照着做的提示。原始 traceback 只会让人以为"程序坏了"，
            # 而真正的原因通常是**串口没挂到 WSL**（CP210x 要 usbipd attach），
            # 或者被 MobaXterm 占着。
            ports = [p.device for p in list_ports.comports()]
            hint = [
                f"打不开串口 {self.port}：{exc}",
                f"  当前系统里的串口：{', '.join(ports) if ports else '（一个都没有）'}",
            ]
            if not ports:
                hint += [
                    "  一个串口都没有 → 设备没接到这台机器上。WSL 里要先把 USB 串口"
                    "挂进来（Windows 侧、需要管理员）：",
                    "      usbipd list",
                    "      usbipd attach --wsl --busid <CP210x 的 BUSID>",
                    "  不想动 usbipd 的话，用另外两条路（都不需要串口）：",
                    "      --stdin   （ssh 板子 '... --keyboard-dev stdout' | ...）",
                    "      --udp PORT（板子 --keyboard-dev udp://本机IP:PORT）",
                ]
            else:
                hint.append("  设备在、但打不开 → 多半被别的程序占着（MobaXterm / 串口助手）")
            raise RuntimeError("\n".join(hint)) from exc

    def _read_bytes(self) -> bytes:
        if self._ser is None:
            return b""
        try:
            n = self._ser.in_waiting
            return self._ser.read(n if n > 0 else 1)
        except Exception:                     # pragma: no cover - 拔线等
            return b""

    def _close_transport(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None


#: 传输名 → 类。`board` / `auto` 由 kwargs 猜（给了 port 走串口、给了 udp 走 UDP…）。
ADAPTERS: dict[str, type[BoardInputAdapter]] = {
    "stdin": StdinBoardAdapter,
    "pipe": StdinBoardAdapter,
    "udp": UdpBoardAdapter,
    "serial": SerialBoardAdapter,
    "uart": SerialBoardAdapter,
}


def make_board_adapter(kind: str = "auto", **kwargs: Any) -> BoardInputAdapter:
    """按 `kind` 建一个板子适配器；`auto` 时按给的参数猜传输。"""
    key = str(kind or "auto").lower()
    if key in ("auto", "board", "hardware", "fpga"):
        if kwargs.get("port"):
            key = "serial"
        elif kwargs.get("udp") is not None:
            key = "udp"
            kwargs["port"] = kwargs.pop("udp")
        elif kwargs.get("stdin") or kwargs.get("fd") is not None:
            key = "stdin"
        else:
            raise ValueError(
                "board 输入要指定传输：--port <串口> 或 --udp <端口> 或 --stdin"
            )
    try:
        cls = ADAPTERS[key]
    except KeyError as exc:
        raise ValueError(
            f"unknown board transport {kind!r}; available: {sorted(ADAPTERS)}"
        ) from exc
    # 只把该类认识的参数传下去（其余是 CLI 的通用 kwargs）
    return cls(**kwargs)


__all__ = [
    "PREFIX",
    "MAX_N",
    "DEFAULT_HOLD_S",
    "parse_moves",
    "BoardInputAdapter",
    "StdinBoardAdapter",
    "UdpBoardAdapter",
    "SerialBoardAdapter",
    "ADAPTERS",
    "make_board_adapter",
]
