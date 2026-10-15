#!/usr/bin/env python3
"""`console_relay.py` 的解析层必须和 RP2040 侧 `code.py` 的解析器逐条一致。

## 为什么

零接线方案（PC 转发）和接线方案（RP2040 直接听板子 TX）**必须表现出同样的
行为**，否则"换根线就不好使了"这种问题会极难查。两边的规则是同一套：

    * 只按 `\\n` 拆行，半行留到下一批
    * 只认带 `@MOVE ` 前缀的行（共享的控制台，混着内核日志）
    * 前缀后必须是 0..8 的整数，否则**静默丢弃**

`code.py` 没法在主机上 import（`import board`），所以这里
① 用解析源码的方式钉住它确实有这两条输入路（`USE_UART_INPUT`/`USE_USB_INPUT`
   和 USB 那条读 `sys.stdin`），② 对 `console_relay.parse_moves` 做行为测试。
"""

from __future__ import annotations

import pathlib
import re
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
RP2040 = ROOT / "core" / "rp2040_keyboard"
CODE_PY = RP2040 / "code.py"

sys.path.insert(0, str(RP2040))
import console_relay  # noqa: E402


# ---------------------------------------------------------------------------
# 纯解析层的行为
# ---------------------------------------------------------------------------
def test_parses_valid_lines() -> None:
    buf = bytearray()
    assert console_relay.parse_moves(b"@MOVE 3\n@MOVE 0\n@MOVE 8\n", buf) == [3, 0, 8]


def test_ignores_console_noise() -> None:
    """这条线是共享的控制台，内核日志/shell 提示混在一起。"""
    noisy = (b"[    2.67] ttyPS0 - failed to request DMA\r\n"
             b"root@U:~# echo @MOVE-not-a-command\r\n"
             b"@MOVE \r\n"                 # 前缀后没数字
             b"@MOVE 9\r\n"                # 越界
             b"@MOVE -1\r\n"               # 负数（也越界）
             b"@MOVE 2.5\r\n"              # 不是整数
             b"@MOVE 4\r\n")               # 唯一一条合法的
    buf = bytearray()
    assert console_relay.parse_moves(noisy, buf) == [4]
    assert buf == bytearray(), "整批都是完整行，残渣应为空"


def test_handles_lines_split_across_chunks() -> None:
    """串口按字节流读，一行被切成两批是常态。"""
    buf = bytearray()
    assert console_relay.parse_moves(b"@MO", buf) == []
    assert console_relay.parse_moves(b"VE 5", buf) == []
    assert console_relay.parse_moves(b"\n", buf) == [5]
    assert console_relay.parse_moves(b"@MOVE 6\n@MOVE", buf) == [6]
    assert console_relay.parse_moves(b" 7\n", buf) == [7]


def test_half_line_residue_does_not_grow_forever() -> None:
    """控制台日志量大时，没有换行的残渣不能越积越多（会吃光内存）。"""
    buf = bytearray()
    for _ in range(20):
        console_relay.parse_moves(b"x" * 1000, buf)
    assert len(buf) <= console_relay._BUF_LIMIT


def test_prefix_matches_rp2040_and_emitter() -> None:
    """前缀在三处必须一致：本转发、code.py、C++ 发射器。"""
    code = CODE_PY.read_text(encoding="utf-8")
    m = re.search(r'^PREFIX\s*=\s*b"([^"]*)"', code, re.M)
    assert m, "code.py 里找不到 PREFIX"
    assert m.group(1).encode("ascii") == console_relay.PREFIX, (
        f"前缀不一致：console_relay {console_relay.PREFIX!r}，code.py {m.group(1)!r}")
    hpp = (ROOT / "core" / "board" / "action_codec.hpp").read_text(encoding="utf-8")
    h = re.search(r'kActionPrefix\s*=\s*"([^"]*)"', hpp)
    assert h, "action_codec.hpp 里找不到 kActionPrefix"
    assert h.group(1) == console_relay.PREFIX.decode("ascii"), (
        f"前缀不一致：console_relay {console_relay.PREFIX!r}，"
        f"action_codec.hpp {h.group(1)!r}")


def test_action_range_matches_protocol() -> None:
    sys.path.insert(0, str(ROOT / "core"))
    from bullet_sim.core.actions import DISCRETE_ACTION_NAMES  # noqa: E402
    assert console_relay.MAX_N == len(DISCRETE_ACTION_NAMES) - 1, (
        f"MAX_N={console_relay.MAX_N}，而协议有 {len(DISCRETE_ACTION_NAMES)} 个动作")


# ---------------------------------------------------------------------------
# code.py 的两条输入路都在（主机上没法 import 它，解析源码）
# ---------------------------------------------------------------------------
def test_code_py_has_both_input_paths() -> None:
    src = CODE_PY.read_text(encoding="utf-8")
    assert re.search(r"^USE_UART_INPUT\s*=\s*True", src, re.M), \
        "code.py 少了 USE_UART_INPUT 开关（接两根线那条路）"
    assert re.search(r"^USE_USB_INPUT\s*=\s*True", src, re.M), \
        "code.py 少了 USE_USB_INPUT 开关（零接线那条路，靠 console_relay 转发）"
    assert "sys.stdin" in src, "code.py 的 USB 输入应当读 sys.stdin"
    # 两条输入必须共用**同一个**解析器，否则换条路行为就变了
    assert src.count("def feed(") == 1, "解析器应当只有一个 feed()"
    # 只数**调用点**（缩进的独立一行）—— `def feed(chunk)` 里也含这个子串，
    # 用 count("feed(chunk)") 会把它一起数进去（这条断言最初就是这么写错的）。
    calls = re.findall(r"^\s+feed\(chunk\)\s*$", src, re.M)
    assert len(calls) == 2, (
        f"feed() 应当被两条输入各调一次（UART 一条、USB 一条），实际 {len(calls)} 次")


def test_code_py_parse_rules_match_relay() -> None:
    """code.py 的解析规则要和 console_relay 对齐（关键词级核对）。

    主机上跑不了 CircuitPython，所以这里核对"规则的存在"：
    前缀过滤、int() 转换的 try/except、0..8 范围检查、按 \\n 拆行。
    """
    src = CODE_PY.read_text(encoding="utf-8")
    for needle in ("startswith(PREFIX)", "int(line[len(PREFIX):])",
                   "except ValueError", "0 <= n <= 8", 'partition(b"\\n")'):
        assert needle in src, f"code.py 的解析少了 {needle!r}（与 console_relay 分家了）"
