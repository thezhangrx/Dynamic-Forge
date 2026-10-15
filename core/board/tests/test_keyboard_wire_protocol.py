#!/usr/bin/env python3
"""决策方向 → 按键 → RP2040 解析：这条链上的每一环都不能各说各话。

## 为什么必须有

"板子算出动作号 → 写成 `@MOVE <n>` → RP2040 解析 → 按下方向键"这条链上，
**每一环错了都不会报错**：角色会朝错方向走、或者干脆不动。而这条链跨了三种
语言/三个文件：

| 环节 | 文件 |
|---|---|
| 编出 `@MOVE <n>\\n` | `core/board/action_codec.cpp`（C++，板上跑） |
| 解析回动作号 | `core/rp2040_keyboard/code.py`（CircuitPython，RP2040 上跑） |
| 动作号 → 按键 | 同上 `code.py` 的 `KEYS` 表 |

`code.py` 没法在主机上 import（它 `import board` / `usb_hid`），所以这里用
**解析源码**的方式对账（本项目 `test_regs_match_rtl.py` 同一套路），
外加一条**往返**测试：把 C++ 编出来的字节，按 `code.py` 的解析规则读回去，
必须得到同一个动作号。
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
RP2040 = ROOT / "core" / "rp2040_keyboard"
CODE_PY = RP2040 / "code.py"
HPP = BOARD / "action_codec.hpp"
CLI_SRC = BOARD / "tests" / "action_codec_cli.cpp"
CPP = BOARD / "action_codec.cpp"

import sys  # noqa: E402

sys.path.insert(0, str(ROOT / "core"))
from bullet_sim.core.actions import (  # noqa: E402
    DISCRETE_ACTION_NAMES,
    DISCRETE_DIRECTIONS,
)


def _py_literal(name: str) -> str:
    """从 code.py 里抠一个 `NAME = <字面量>`（只支持字符串/数字）。"""
    src = CODE_PY.read_text(encoding="utf-8")
    m = re.search(rf"^{name}\s*=\s*(.+?)\s*(?:#.*)?$", src, re.M)
    assert m, f"code.py 里找不到 {name}"
    return m.group(1).strip()


def _keys_table() -> dict[int, tuple[str, ...]]:
    """解析 code.py 的 KEYS 表：动作号 → 按键名元组。"""
    src = CODE_PY.read_text(encoding="utf-8")
    m = re.search(r"^KEYS\s*=\s*\{(.*?)^\}", src, re.M | re.S)
    assert m, "code.py 里找不到 KEYS 表"
    out: dict[int, tuple[str, ...]] = {}
    for idx, body in re.findall(r"(\d+)\s*:\s*\(([^)]*)\)", m.group(1)):
        out[int(idx)] = tuple(re.findall(r"Keycode\.(\w+)", body))
    return out


def test_prefix_identical_everywhere() -> None:
    """`@MOVE ` 前缀三处必须逐字节相同。"""
    cpp = re.search(r'kActionPrefix\s*=\s*"([^"]*)"', HPP.read_text(encoding="utf-8"))
    assert cpp, "action_codec.hpp 里找不到 kActionPrefix"
    py_prefix = _py_literal("PREFIX")
    assert py_prefix.startswith('b"') or py_prefix.startswith("b'"), (
        f"code.py 的 PREFIX 应该是 bytes 字面量，实际是 {py_prefix!r}")
    assert cpp.group(1) == py_prefix[2:-1], (
        f"前缀不一致：C++ {cpp.group(1)!r}，code.py {py_prefix[2:-1]!r}")
    # action_sender.py 也必须一样（第三处）
    sender = (RP2040 / "action_sender.py").read_text(encoding="utf-8")
    sp = re.search(r'^PREFIX\s*=\s*"([^"]*)"', sender, re.M)
    assert sp, "action_sender.py 里找不到 PREFIX"
    assert sp.group(1) == cpp.group(1), (
        f"前缀不一致：action_codec.hpp {cpp.group(1)!r}，action_sender.py {sp.group(1)!r}")


def test_keys_cover_every_action_and_match_directions() -> None:
    """9 个动作都要有按键；而且**按的键必须真的是那个方向**。

    后半句是重点：动作号 2 叫 `right`，但它到底按没按 RIGHT_ARROW，
    光看下标是看不出来的 —— 这里从 `DISCRETE_DIRECTIONS` 反推出每个方向
    "应该按哪些键"，再和 `code.py` 的表逐项比。
    """
    keys = _keys_table()
    assert set(keys) == set(range(len(DISCRETE_ACTION_NAMES))), (
        f"KEYS 覆盖的动作号是 {sorted(keys)}，协议要求 0..{len(DISCRETE_ACTION_NAMES)-1}")

    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        x, y = (float(DISCRETE_DIRECTIONS[i][0]), float(DISCRETE_DIRECTIONS[i][1]))
        want = set()
        if x < 0: want.add("LEFT_ARROW")
        if x > 0: want.add("RIGHT_ARROW")
        if y > 0: want.add("UP_ARROW")
        if y < 0: want.add("DOWN_ARROW")
        got = set(keys[i])
        assert got == want, (
            f"动作 {i}（{name}）按的键不对：code.py 是 {sorted(got)}，"
            f"而方向 {DISCRETE_DIRECTIONS[i]} 应对应 {sorted(want)}。"
            "这一环错了角色会朝错方向走，且不会报错")


def test_watchdog_exists_and_is_bounded() -> None:
    """必须有一个"收不到有效指令就松开所有键"的看门狗，且时间有界。

    没有它：板子崩了/线断了，RP2040 会一直按着上一个方向键，角色永远往那边走。
    """
    timeout = _py_literal("TIMEOUT_MS")
    assert timeout.isdigit(), f"TIMEOUT_MS 应该是整数毫秒，实际 {timeout!r}"
    ms = int(timeout)
    assert 50 <= ms <= 5000, (
        f"TIMEOUT_MS={ms} 不合理：太小会在正常丢帧时抖动，太大则失控窗口过长")
    src = CODE_PY.read_text(encoding="utf-8")
    assert "release_all" in src and "time.monotonic" in src, (
        "code.py 里应有基于 time.monotonic 的超时松开逻辑")


@pytest.fixture(scope="module")
def cli(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++")
    out = tmp_path_factory.mktemp("kw") / "action_codec_cli"
    r = subprocess.run(
        [gxx, "-O2", "-Wall", "-Wextra", "-Werror", "-std=c++17",
         "-ffp-contract=off", f"-I{BOARD}", str(CLI_SRC), str(CPP), "-o", str(out)],
        capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, f"编译失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_round_trip_emit_then_parse_like_rp2040(cli: pathlib.Path) -> None:
    """往返：C++ 编出的字节，按 `code.py` 的规则解析，必须回到同一个动作号。

    `code.py` 的解析规则（照抄它循环里的三行）：
        if not line.startswith(PREFIX): 忽略
        n = int(line[len(PREFIX):])
    """
    prefix = _py_literal("PREFIX")[2:-1].encode("ascii")
    r = subprocess.run([str(cli), "--lines"], capture_output=True, timeout=60)
    assert r.returncode == 0, r.stderr
    lines = [ln for ln in r.stdout.split(b"\n") if ln]
    assert len(lines) == len(DISCRETE_ACTION_NAMES), (
        f"C++ 编出 {len(lines)} 行，协议有 {len(DISCRETE_ACTION_NAMES)} 个动作")

    for expect, raw in enumerate(lines):
        assert raw.startswith(prefix), (
            f"第 {expect} 行动作不以 {prefix!r} 开头（RP2040 会整行忽略）：{raw!r}")
        n = int(raw[len(prefix):].decode("ascii"))     # ← code.py 的解析规则
        assert n == expect, f"往返不一致：编出的是 {expect}，解析回来是 {n}"
