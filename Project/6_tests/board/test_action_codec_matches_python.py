#!/usr/bin/env python3
"""`action_codec.cpp` 与 Python 那两份实现必须逐点一致。

## 为什么必须有这一条

"动作号"是 **PS ↔ RP2040 ↔ 平台** 三方共用的协议：角标含义错一个，角色就朝
反方向走 —— 而且**不会报错**。项目里已经有一份公开真值
（`core/bullet_sim/core/actions.py` 的 `DISCRETE_ACTION_NAMES` /
`DISCRETE_DIRECTIONS`，源码注释：*Order is part of the public protocol —
never reorder, only append*），`core/rp2040_keyboard/action_sender.py` 直接复用它。

但板上跑的是 **C++**，`action_codec.cpp` 是它的第二份实现（镜像）。两份实现
各改各的就会分家，所以这里做两件事：

1. **表对表**：解析 `action_codec.cpp` 的名字与方向表，和 Python 真值逐项比；
2. **函数对函数**（差分对账）：用**同一批速度**分别喂 C++ 的
   `action_from_velocity`（经 `action_codec_cli`）与 Python 的同一函数，
   断言动作号逐点相同 —— 和 `core/cpu/tests/test_cpp_gap_avoid.py` 对
   `tier` 的做法同一套路。

只比表不比函数是不够的：点积取最大的那段逻辑（含 deadzone、含 45° 边界上的
浮点舍入）才是真正会分家的地方。
"""

from __future__ import annotations

import math
import pathlib
import re
import shutil
import struct
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
BOARD = ROOT / "core" / "board"
CPP = BOARD / "action_codec.cpp"
HPP = BOARD / "action_codec.hpp"
CLI_SRC = BOARD / "tests" / "action_codec_cli.cpp"

sys.path.insert(0, str(ROOT / "core"))
sys.path.insert(0, str(ROOT / "core" / "rp2040_keyboard"))

from bullet_sim.core.actions import (  # noqa: E402
    DISCRETE_ACTION_NAMES,
    DISCRETE_DIRECTIONS,
)
import action_sender  # noqa: E402


# ---------------------------------------------------------------------------
# 解析 C++ 侧的两张表
# ---------------------------------------------------------------------------
def _strip_line_comments(text: str) -> str:
    return "\n".join(line.split("//")[0] for line in text.splitlines())


def _cpp_names() -> list[str]:
    src = _strip_line_comments(CPP.read_text(encoding="utf-8"))
    m = re.search(r"kDiscreteActionNames\[\w+\]\s*=\s*\{(.*?)\};", src, re.S)
    assert m, "action_codec.cpp 里找不到 kDiscreteActionNames"
    return re.findall(r'"([^"]+)"', m.group(1))


def _cpp_directions() -> list[tuple[float, float]]:
    """方向表。`kInvSqrt2` 要解析成数值 —— 与 C++ 里那个常数同值。"""
    src = _strip_line_comments(CPP.read_text(encoding="utf-8"))
    # C++ 侧的 kInvSqrt2：**照它的表达式算**，不要只抠十进制字面量。
    # （这里踩过：C++ 改用 `1.0 / std::sqrt(2.0)` 之后，只抠数字的解析器会把
    #   `1.0` 当成常数，于是这条测试反而误报。照算法求值才与"逐位相同"的判据自洽。）
    mm = re.search(r"kInvSqrt2\s*=\s*([^;]+);", src)
    assert mm, "action_codec.cpp 里找不到 kInvSqrt2"
    expr = mm.group(1).strip()
    inv = (1.0 / math.sqrt(2.0)) if "sqrt" in expr else float(expr)
    m = re.search(r"kDiscreteDirections\[\w+\]\[2\]\s*=\s*\{(.*?)\n\};", src, re.S)
    assert m, "action_codec.cpp 里找不到 kDiscreteDirections"
    out: list[tuple[float, float]] = []
    for row in re.findall(r"\{([^{}]*)\}", m.group(1)):
        vals = []
        for tok in row.split(","):
            tok = tok.strip()
            if not tok:
                continue
            vals.append(-inv if tok == "-kInvSqrt2" else
                        inv if tok == "kInvSqrt2" else float(tok))
        assert len(vals) == 2, f"方向行解析出 {len(vals)} 个分量：{row!r}"
        out.append((vals[0], vals[1]))
    return out


def test_action_names_match_python_protocol() -> None:
    """动作名与顺序必须等于公开协议。"""
    cpp = _cpp_names()
    assert cpp == list(DISCRETE_ACTION_NAMES), (
        f"C++ 的动作名/顺序与 core/bullet_sim/core/actions.py 不一致：\n"
        f"  C++   : {cpp}\n  Python: {list(DISCRETE_ACTION_NAMES)}\n"
        "角标含义是三方共用的协议，顺序只能追加、不能重排")


def test_action_directions_match_python_protocol() -> None:
    """9 个单位方向必须**逐位相同**（不是"近似相等"）。

    ⚠ 判据必须严格到 bit：选向是"比点积最大"，45° 边界上相邻两方向的点积
      本来相等，所以**差 1 个 ulp 的 1/√2 就足以让两份实现破到不同方向**。
      这条测试最初写成 `pytest.approx(abs=1e-12)` 时放过了那个 1 ulp 的分家，
      是后面那条差分对账把它抓出来的 —— 所以这里改成逐位比。
    """
    cpp = _cpp_directions()
    py = [(float(x), float(y)) for x, y in DISCRETE_DIRECTIONS]
    assert len(cpp) == len(py), f"方向个数不一致：C++ {len(cpp)}，Python {len(py)}"
    for i, (c, p) in enumerate(zip(cpp, py)):
        assert struct.pack("<d", c[0]) == struct.pack("<d", p[0]) and \
               struct.pack("<d", c[1]) == struct.pack("<d", p[1]), (
            f"第 {i} 个方向不一致（{DISCRETE_ACTION_NAMES[i]}）：\n"
            f"  C++    ({c[0]!r}, {c[1]!r})\n"
            f"  Python ({p[0]!r}, {p[1]!r})\n"
            "差 1 个 ulp 就够在 45° 边界上选到不同方向（选向就是比点积最大）")


def test_num_actions_constant_matches() -> None:
    n = re.search(r"kNumDiscreteActions\s*=\s*(\d+)",
                  HPP.read_text(encoding="utf-8"))
    assert n, "action_codec.hpp 里找不到 kNumDiscreteActions"
    assert int(n.group(1)) == len(DISCRETE_ACTION_NAMES), (
        f"kNumDiscreteActions={n.group(1)}，而协议里有 {len(DISCRETE_ACTION_NAMES)} 个动作")


# ---------------------------------------------------------------------------
# 差分对账：同一批速度，两份实现必须给出同一个动作号
# ---------------------------------------------------------------------------
def _velocity_grid() -> list[tuple[float, float]]:
    """覆盖到会分家的每一处：

    * 8 个方向轴 + 8 个 45° 对角线（含正负号）；
    * **45° 边界**附近（两份实现的浮点舍入不同就会在这里分家）；
    * deadzone 两侧（0、极小、恰好等于）；
    * 各象限的随机刻度（同一个方向、不同的模长）；
    * 零向量与极大值。
    """
    g: list[tuple[float, float]] = [(0.0, 0.0)]
    # 轴 + 对角线
    for x, y in DISCRETE_DIRECTIONS:
        for s in (1.0, 2.5, 200.0, 1e-2):
            g.append((float(x) * s, float(y) * s))
    # 45° 边界扫（22.5°±ε 附近是相邻两个方向点积相等的临界处）
    for deg in [0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225,
                247.5, 270, 292.5, 315, 337.5]:
        for eps in (-1e-9, -1e-12, 0.0, 1e-12, 1e-9):
            r = math.radians(deg + eps)
            g.append((math.cos(r) * 100.0, math.sin(r) * 100.0))
    # deadzone 两侧
    for eps in (-1e-9, -1e-12, 0.0, 1e-12, 1e-9):
        g.append((1e-3 + eps, 1e-3 + eps))
        g.append((5e-4 + eps, 0.0))
    # 各象限随机刻度（确定性伪随机，便于复现）
    seed = 12345
    for _ in range(400):
        seed = (1103515245 * seed + 12345) & 0x7FFFFFFF
        x = (seed / 0x7FFFFFFF) * 400.0 - 200.0
        seed = (1103515245 * seed + 12345) & 0x7FFFFFFF
        y = (seed / 0x7FFFFFFF) * 400.0 - 200.0
        g.append((x, y))
    return g


@pytest.fixture(scope="module")
def cli(tmp_path_factory: pytest.TempPathFactory) -> pathlib.Path:
    """把 C++ CLI 编出来（本机 g++，不走 CMake）。"""
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++")
    out = tmp_path_factory.mktemp("act") / "action_codec_cli"
    r = subprocess.run(
        # -ffp-contract=off：**必须**，和 CMake 里那条一致。GCC 默认把
        # `dx*wx + dy*wy` 收缩成 FMA，改变最后一位舍入 → 45° 边界上两份实现
        # 会破不同的方向（这条测试最初就是在这个坑上失败的）。
        [gxx, "-O2", "-Wall", "-Wextra", "-Werror", "-std=c++17",
         "-ffp-contract=off",
         f"-I{BOARD}", str(CLI_SRC), str(CPP), "-o", str(out)],
        capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, f"编译 action_codec_cli 失败：\n{r.stdout}\n{r.stderr}"
    return out


def test_cpp_and_python_agree_on_the_whole_grid(cli: pathlib.Path) -> None:
    """差分对账：整批速度上，C++ 与 Python 的动作号必须逐点相同。"""
    grid = _velocity_grid()
    stdin = "".join(f"{x!r} {y!r}\n" for x, y in grid)
    r = subprocess.run([str(cli)], input=stdin, capture_output=True,
                       text=True, timeout=180)
    assert r.returncode == 0, r.stderr
    cpp = [int(t) for t in r.stdout.split()]
    assert len(cpp) == len(grid), (
        f"C++ 只回了 {len(cpp)} 行，喂了 {len(grid)} 组速度")

    mism: list[str] = []
    for (x, y), got in zip(grid, cpp):
        want = action_sender.action_from_velocity(x, y)
        if got != want:
            mism.append(f"  v=({x!r}, {y!r})  C++={got}  Python={want}")
    assert not mism, (
        f"{len(mism)}/{len(grid)} 组速度上两份实现分家了：\n"
        + "\n".join(mism[:20])
        + "\n（角标含义是三方共用的协议，分家会让角色朝错方向走且不报错）")


def test_action_line_format_matches() -> None:
    """线格式也必须一致：`@MOVE <n>\\n`。"""
    hpp = HPP.read_text(encoding="utf-8")
    m = re.search(r'kActionPrefix\s*=\s*"([^"]*)"', hpp)
    assert m, "action_codec.hpp 里找不到 kActionPrefix"
    assert m.group(1) == action_sender.PREFIX, (
        f"前缀不一致：C++ {m.group(1)!r}，Python {action_sender.PREFIX!r}")
    assert m.group(1).startswith("@"), (
        "前缀必须以 @ 开头 —— 这条线是共享的控制台，"
        "RP2040 靠它把指令从内核日志里挑出来")
