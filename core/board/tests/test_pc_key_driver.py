#!/usr/bin/env python3
"""PC 侧按键注入的映射必须和 RP2040 那份**同义**，否则"换台机器就不好使"。

`pc_key_driver.py`（软件注入，不用额外硬件）和 `code.py`（RP2040 真 HID）是
同一件事的两条实现。两边各写一张"动作号 → 按键"的表迟早会分家，而且**分家了
不报错，只会朝错方向走**。所以：

1. `pc_key_driver` 的键**从协议的 `DISCRETE_DIRECTIONS` 推导**，不另立一张表；
2. 这里把它和 `code.py` 的 `KEYS` 表（CircuitPython 的 Keycode 名）**逐项对起来** ——
   两份独立实现、同一个协议真值，必须给出同一组方向；
3. 差集逻辑（换向时不松开共用的键）单独测，这决定斜向切换会不会抖。
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
RP2040 = ROOT / "core" / "rp2040_keyboard"
CODE_PY = RP2040 / "code.py"

sys.path.insert(0, str(RP2040))
sys.path.insert(0, str(ROOT / "core"))

import pc_key_driver as drv  # noqa: E402
from bullet_sim.core.actions import (  # noqa: E402
    DISCRETE_ACTION_NAMES,
    DISCRETE_DIRECTIONS,
)

#: 键位方案 → (虚拟键码 → 方向名)。**每套各建一份**：arrows 与 wasd 的键码不同，
#: 只建 arrows 那套的话查 wasd 会 KeyError（这条测试最初就是这么错的）。
VK_TO_NAME = {km: {vk: name for name, vk in table.items()}
              for km, table in drv.VK.items()}


def _code_py_keys() -> dict[int, set[str]]:
    """解析 code.py 的 KEYS 表：动作号 → 方向名集合。"""
    src = CODE_PY.read_text(encoding="utf-8")
    m = re.search(r"^KEYS\s*=\s*\{(.*?)^\}", src, re.M | re.S)
    assert m, "code.py 里找不到 KEYS 表"
    out: dict[int, set[str]] = {}
    for idx, body in re.findall(r"(\d+)\s*:\s*\(([^)]*)\)", m.group(1)):
        names = set()
        for kc in re.findall(r"Keycode\.(\w+)_ARROW", body):
            names.add(kc)
        out[int(idx)] = names
    return out


# ---------------------------------------------------------------------------
# 1) 从协议真值推导，且与 code.py 一致
# ---------------------------------------------------------------------------
def test_every_action_matches_protocol_direction() -> None:
    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        x, y = float(DISCRETE_DIRECTIONS[i][0]), float(DISCRETE_DIRECTIONS[i][1])
        want = set()
        if x < 0: want.add("LEFT")
        if x > 0: want.add("RIGHT")
        if y > 0: want.add("UP")
        if y < 0: want.add("DOWN")
        got = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i)}
        assert got == want, (
            f"动作 {i}（{name}）推导出的方向不对：拿到 {sorted(got)}，"
            f"协议方向 {DISCRETE_DIRECTIONS[i]} 应为 {sorted(want)}")


def test_pc_driver_and_rp2040_agree_on_every_action() -> None:
    """PC 软件注入与 RP2040 真 HID 必须按同一组键 —— 否则换个方案就朝错方向走。"""
    code_keys = _code_py_keys()
    assert set(code_keys) == set(range(len(DISCRETE_ACTION_NAMES))), \
        f"code.py 的 KEYS 覆盖不全：{sorted(code_keys)}"
    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        pc = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i)}
        rp = code_keys[i]
        assert pc == rp, (
            f"动作 {i}（{name}）两边按的键不一致：\n"
            f"  pc_key_driver（软件注入）: {sorted(pc)}\n"
            f"  code.py（RP2040 HID）    : {sorted(rp)}\n"
            "两者是同一件事的两条实现，必须同义")


def test_wasd_option_maps_same_directions() -> None:
    """WASD 那套只是换键码，方向语义必须完全一样。"""
    for i in range(len(DISCRETE_ACTION_NAMES)):
        a = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i, "arrows")}
        b = {VK_TO_NAME["wasd"][k] for k in drv.keys_for_action(i, "wasd")}
        assert a == b, f"动作 {i} 在 arrows 与 wasd 下方向不一致：{a} vs {b}"
    assert drv.keys_for_action(2, "wasd") == (0x44,)      # right → D
    assert drv.keys_for_action(1, "wasd") == (0x41,)      # left  → A
    assert drv.keys_for_action(3, "wasd") == (0x57,)      # up    → W
    assert drv.keys_for_action(4, "wasd") == (0x53,)      # down  → S


# ---------------------------------------------------------------------------
# 2) 差集逻辑（决定斜向切换抖不抖）
# ---------------------------------------------------------------------------
def test_stay_presses_and_releases_nothing() -> None:
    assert drv.key_diff((), ()) == ([], [])


def test_switching_diagonals_keeps_the_shared_key_down() -> None:
    """右上 → 右下：RIGHT 必须**保持按住**，只换 UP→DOWN。

    若把 RIGHT 也松开再按，平台会把它当成"松了又按"，角色会一顿一顿的。
    """
    up_right = drv.keys_for_action(6)
    down_right = drv.keys_for_action(8)
    press, release = drv.key_diff(up_right, down_right)
    assert drv.VK["arrows"]["RIGHT"] not in release, "RIGHT 不该被松开"
    assert drv.VK["arrows"]["RIGHT"] not in press, "RIGHT 不该被重复按下"
    assert press == [drv.VK["arrows"]["DOWN"]]
    assert release == [drv.VK["arrows"]["UP"]]


def test_going_to_stay_releases_everything() -> None:
    held = drv.keys_for_action(5)                      # up_left
    press, release = drv.key_diff(held, ())
    assert press == []
    assert set(release) == set(held)


def test_out_of_range_action_is_a_no_op() -> None:
    """越界动作号不能按出乱七八糟的键（控制台噪声就可能撞上）。"""
    assert drv.keys_for_action(-1) == ()
    assert drv.keys_for_action(99) == ()


# ---------------------------------------------------------------------------
# 3) Windows 注入器能起来（**只 PING，绝不按键**）
# ---------------------------------------------------------------------------
def test_powershell_injector_loads_and_pings() -> None:
    """注入器的 `Add-Type`（P/Invoke 声明）必须能编译过。

    这是最容易写错、又最难在真按键时发现的一环（写错了只是按不动，
    而"按不动"会被误当成"平台不吃合成输入"）。所以这里只发 `PING`：
    **不注入任何按键**，只验证脚本能加载、命令循环能应答。
    """
    ps = shutil.which("powershell.exe")
    if ps is None:
        pytest.skip("没有 powershell.exe（不在 WSL / 不是 Windows）")
    ps1 = RP2040 / "win_sendinput.ps1"
    assert ps1.exists(), "缺 win_sendinput.ps1"
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ps1)],
        input="PING\nQUIT\n", capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, f"注入器退出码 {r.returncode}：\n{r.stdout}\n{r.stderr}"
    assert "PONG" in r.stdout, f"注入器没应答 PONG：\n{r.stdout}\n{r.stderr}"
